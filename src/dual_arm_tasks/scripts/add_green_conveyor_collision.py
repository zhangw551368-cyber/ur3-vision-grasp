#!/usr/bin/env python3

import math
import sys

import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped


def get_required_param(name):
    private_name = "~{}".format(name)
    if not rospy.has_param(private_name):
        raise KeyError("Missing required ROS parameter: {}".format(private_name))
    return rospy.get_param(private_name)


def get_float_param(name):
    value = float(get_required_param(name))
    if not math.isfinite(value):
        raise ValueError("Parameter ~{} must be finite, got {}".format(name, value))
    return value


def get_bool_param(name):
    value = get_required_param(name)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    raise ValueError("Parameter ~{} must be a boolean, got {!r}".format(name, value))


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def wait_for_object_state(scene, object_name, should_exist, timeout):
    start = rospy.Time.now()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
        is_known = object_name in scene.get_known_object_names()
        is_attached = object_name in scene.get_attached_objects([object_name])
        if is_known == should_exist and not is_attached:
            return True
        rate.sleep()

    return False


def make_planning_scene_interface():
    try:
        return moveit_commander.PlanningSceneInterface(synchronous=True)
    except TypeError:
        rospy.logwarn(
            "This moveit_commander does not support synchronous PlanningSceneInterface; "
            "falling back to asynchronous scene updates."
        )
        return moveit_commander.PlanningSceneInterface()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("add_green_conveyor_collision", anonymous=False)

    object_name = str(get_required_param("object_name"))
    frame_id = str(get_required_param("frame_id"))
    conveyor_length = get_float_param("conveyor_length")
    conveyor_width = get_float_param("conveyor_width")
    conveyor_center_x = get_float_param("conveyor_center_x")
    conveyor_center_y = get_float_param("conveyor_center_y")
    conveyor_top_z = get_float_param("conveyor_top_z")
    collision_thickness = get_float_param("collision_thickness")
    safety_margin = get_float_param("safety_margin")
    conveyor_yaw = get_float_param("conveyor_yaw")
    remove_only = get_bool_param("remove_only")

    if not object_name:
        raise ValueError("Parameter ~object_name must not be empty")
    if not frame_id:
        raise ValueError("Parameter ~frame_id must not be empty")
    if conveyor_length <= 0.0:
        raise ValueError("Parameter ~conveyor_length must be > 0")
    if conveyor_width <= 0.0:
        raise ValueError("Parameter ~conveyor_width must be > 0")
    if collision_thickness <= 0.0:
        raise ValueError("Parameter ~collision_thickness must be > 0")

    size_x = conveyor_length
    size_y = conveyor_width
    size_z = collision_thickness

    center_x = conveyor_center_x
    center_y = conveyor_center_y
    center_z = conveyor_top_z + safety_margin - collision_thickness / 2.0

    scene = make_planning_scene_interface()
    rospy.sleep(1.0)

    rospy.loginfo("Removing existing collision object '%s' before update.", object_name)
    scene.remove_world_object(object_name)
    wait_for_object_state(scene, object_name, should_exist=False, timeout=5.0)

    if remove_only:
        rospy.loginfo("remove_only=true; removed collision object '%s' and skipped add.", object_name)
        return

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = center_x
    pose.pose.position.y = center_y
    pose.pose.position.z = center_z
    qx, qy, qz, qw = yaw_to_quaternion(conveyor_yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    scene.add_box(object_name, pose, size=(size_x, size_y, size_z))
    if not wait_for_object_state(scene, object_name, should_exist=True, timeout=5.0):
        raise RuntimeError(
            "Timed out waiting for collision object '{}' in the planning scene".format(
                object_name
            )
        )

    rospy.loginfo("Added MoveIt collision object:")
    rospy.loginfo("  object_name: %s", object_name)
    rospy.loginfo("  frame_id: %s", frame_id)
    rospy.loginfo("  box size: x=%.6f y=%.6f z=%.6f", size_x, size_y, size_z)
    rospy.loginfo(
        "  box center pose: position=(%.6f, %.6f, %.6f), orientation=(%.6f, %.6f, %.6f, %.6f)",
        center_x,
        center_y,
        center_z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    )
    rospy.loginfo("  conveyor_yaw: %.6f rad", conveyor_yaw)
    rospy.loginfo("  safety_margin: %.6f m", safety_margin)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("Failed to update green conveyor collision object: %s", exc)
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
