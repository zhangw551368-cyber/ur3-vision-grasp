#!/usr/bin/env python3
import yaml

import rospy
from sensor_msgs.msg import CameraInfo


def _matrix_data(node, key, expected_len):
    data = node.get(key, {}).get("data")
    if data is None or len(data) != expected_len:
        raise ValueError("{} must contain {} values".format(key, expected_len))
    return [float(value) for value in data]


def load_camera_info(path, frame_id):
    with open(path, "r") as stream:
        node = yaml.safe_load(stream)

    msg = CameraInfo()
    msg.width = int(node["image_width"])
    msg.height = int(node["image_height"])
    msg.distortion_model = str(node.get("distortion_model", "plumb_bob"))
    msg.K = _matrix_data(node, "camera_matrix", 9)
    msg.D = _matrix_data(node, "distortion_coefficients", len(node["distortion_coefficients"]["data"]))
    msg.R = _matrix_data(node, "rectification_matrix", 9)
    msg.P = _matrix_data(node, "projection_matrix", 12)
    msg.header.frame_id = frame_id
    return msg


def main():
    rospy.init_node("camera_info_from_yaml_publisher")

    camera_info_yaml = rospy.get_param("~camera_info_yaml")
    source_topic = rospy.get_param("~source_camera_info_topic")
    output_topic = rospy.get_param("~output_camera_info_topic")
    frame_id = rospy.get_param("~frame_id", "")
    queue_size = int(rospy.get_param("~queue_size", 10))

    pub = rospy.Publisher(output_topic, CameraInfo, queue_size=queue_size)
    calibrated_info = load_camera_info(camera_info_yaml, frame_id)

    def republish(source_info):
        msg = CameraInfo()
        msg.header = source_info.header
        msg.width = calibrated_info.width
        msg.height = calibrated_info.height
        msg.distortion_model = calibrated_info.distortion_model
        msg.D = list(calibrated_info.D)
        msg.K = list(calibrated_info.K)
        msg.R = list(calibrated_info.R)
        msg.P = list(calibrated_info.P)
        msg.binning_x = source_info.binning_x
        msg.binning_y = source_info.binning_y
        msg.roi = source_info.roi
        if frame_id:
            msg.header.frame_id = frame_id
        pub.publish(msg)

    rospy.loginfo("Publishing calibrated CameraInfo from %s to %s", camera_info_yaml, output_topic)
    rospy.Subscriber(source_topic, CameraInfo, republish, queue_size=queue_size)
    rospy.spin()


if __name__ == "__main__":
    main()
