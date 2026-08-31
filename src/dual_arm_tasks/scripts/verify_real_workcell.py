#!/usr/bin/env python3

"""Read-only verification of the deployed UR3 workcell and collision scene."""

import sys
import time

import moveit_commander
import rospy
import tf2_ros
from sensor_msgs.msg import JointState


RIGHT_ARM_JOINTS = {
    "right_arm_shoulder_pan_joint",
    "right_arm_shoulder_lift_joint",
    "right_arm_elbow_joint",
    "right_arm_wrist_1_joint",
    "right_arm_wrist_2_joint",
    "right_arm_wrist_3_joint",
}

EXPECTED_COLLISION_OBJECTS = {
    "right_arm_lab_20260824__support_cabinet",
    "right_arm_lab_20260824__green_cloth_work_surface",
}


def wait_for_collision_objects(scene, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        known = set(scene.get_known_object_names())
        if EXPECTED_COLLISION_OBJECTS.issubset(known):
            return known
        rospy.sleep(0.2)
    return set(scene.get_known_object_names())


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("verify_real_workcell", anonymous=True)
    timeout = float(rospy.get_param("~timeout", 15.0))
    require_camera = bool(rospy.get_param("~require_camera", True))
    require_robot_driver = bool(rospy.get_param("~require_robot_driver", True))

    joint_state = rospy.wait_for_message("/joint_states", JointState, timeout=timeout)
    available_joints = set(joint_state.name)
    missing_joints = sorted(RIGHT_ARM_JOINTS - available_joints)
    if missing_joints:
        raise RuntimeError("right-arm joint states are incomplete: {}".format(missing_joints))

    if require_robot_driver:
        rospy.wait_for_service(
            "/right_arm/ur_hardware_interface/dashboard/program_running", timeout=timeout
        )
    rospy.wait_for_service("/get_planning_scene", timeout=timeout)

    scene = moveit_commander.PlanningSceneInterface()
    known_objects = wait_for_collision_objects(scene, timeout)
    missing_objects = sorted(EXPECTED_COLLISION_OBJECTS - known_objects)
    if missing_objects:
        raise RuntimeError(
            "measured collision objects are missing: {}; known={}".format(
                missing_objects, sorted(known_objects)
            )
        )

    if require_camera:
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        tf2_ros.TransformListener(tf_buffer)
        tf_buffer.lookup_transform(
            "right_arm_base",
            "camera_color_optical_frame",
            rospy.Time(0),
            rospy.Duration(timeout),
        )

    rospy.logwarn("REAL WORKCELL READ-ONLY VERIFICATION PASSED")
    rospy.logwarn("  right-arm joints: %d", len(RIGHT_ARM_JOINTS))
    rospy.logwarn("  collision objects: %s", sorted(EXPECTED_COLLISION_OBJECTS))
    rospy.logwarn("  camera TF required: %s", require_camera)
    rospy.logwarn("  physical robot driver required: %s", require_robot_driver)
    rospy.logwarn("This check did not enable External Control or move either robot.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logerr("Real workcell verification failed: %s", exc)
        else:
            print("ERROR: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
    finally:
        moveit_commander.roscpp_shutdown()
