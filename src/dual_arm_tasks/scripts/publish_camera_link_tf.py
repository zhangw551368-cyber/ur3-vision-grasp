#!/usr/bin/python3

import os

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped


def normalize_quat(q):
    norm = sum(value * value for value in q) ** 0.5
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in q)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_multiply(a, b):
    return normalize_quat(quat_multiply_raw(a, b))


def rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    rotated = quat_multiply_raw(quat_multiply_raw(q, qv), quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


def compose_transform(first, second):
    t1, q1 = first
    t2, q2 = second
    rt2 = rotate_vector(q1, t2)
    return (
        (t1[0] + rt2[0], t1[1] + rt2[1], t1[2] + rt2[2]),
        quat_multiply(q1, q2),
    )


def invert_transform(transform):
    t, q = transform
    q_inv = quat_conjugate(q)
    rt = rotate_vector(q_inv, (-t[0], -t[1], -t[2]))
    return (rt, q_inv)


def transform_msg_to_tuple(msg):
    t = msg.transform.translation
    q = msg.transform.rotation
    return (
        (t.x, t.y, t.z),
        normalize_quat((q.x, q.y, q.z, q.w)),
    )


def load_handeye_transform(filename):
    with open(os.path.expanduser(filename), "r") as calibration_file:
        calibration = yaml.safe_load(calibration_file)

    params = calibration["parameters"]
    transform = calibration["transformation"]
    t = (transform["x"], transform["y"], transform["z"])
    q = normalize_quat(
        (
            transform["qx"],
            transform["qy"],
            transform["qz"],
            transform["qw"],
        )
    )
    return params, (t, q)


def make_transform_msg(parent, child, transform):
    t, q = transform
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = t[0]
    msg.transform.translation.y = t[1]
    msg.transform.translation.z = t[2]
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    return msg


def main():
    rospy.init_node("publish_camera_link_tf")

    default_file = os.path.expanduser(
        "~/.ros/easy_handeye/ur3_right_realsense_handeyecalibration_eye_on_base.yaml"
    )
    calibration_file = rospy.get_param("~calibration_file", default_file)
    camera_link_frame = rospy.get_param("~camera_link_frame", "camera_link")

    params, base_to_optical = load_handeye_transform(calibration_file)
    robot_base_frame = rospy.get_param(
        "~robot_base_frame", params.get("robot_base_frame", "base")
    )
    camera_optical_frame = rospy.get_param(
        "~camera_optical_frame",
        params.get("tracking_base_frame", "camera_color_optical_frame"),
    )

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.loginfo(
        "Loading hand-eye calibration %s as %s -> %s via RealSense root %s",
        calibration_file,
        robot_base_frame,
        camera_optical_frame,
        camera_link_frame,
    )

    optical_in_camera_link_msg = tf_buffer.lookup_transform(
        camera_link_frame,
        camera_optical_frame,
        rospy.Time(0),
        rospy.Duration(10.0),
    )
    camera_link_to_optical = transform_msg_to_tuple(optical_in_camera_link_msg)
    base_to_camera_link = compose_transform(
        base_to_optical, invert_transform(camera_link_to_optical)
    )

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_msg = make_transform_msg(
        robot_base_frame, camera_link_frame, base_to_camera_link
    )
    broadcaster.sendTransform(static_msg)

    rospy.loginfo(
        "Published calibrated static TF %s -> %s: xyz=[%.6f, %.6f, %.6f], "
        "quat=[%.6f, %.6f, %.6f, %.6f]",
        static_msg.header.frame_id,
        static_msg.child_frame_id,
        static_msg.transform.translation.x,
        static_msg.transform.translation.y,
        static_msg.transform.translation.z,
        static_msg.transform.rotation.x,
        static_msg.transform.rotation.y,
        static_msg.transform.rotation.z,
        static_msg.transform.rotation.w,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
