#!/usr/bin/env python3

"""Load measured lab geometry into a MoveIt PlanningScene.

The node only manages collision objects whose names begin with ``scene_id``.
It never sends a robot trajectory.  Geometry is supplied through ROS params so
the same node can be reused after the physical measurements are refined.
"""

import math
import sys
import time

import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest


def as_bool(value, label):
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
    raise ValueError("{} must be boolean, got {!r}".format(label, value))


def finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must contain {} values".format(label, length))
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError("{} contains a non-finite value".format(label))
    return result


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_pose(frame_id, pose_values):
    x, y, z, roll, pitch, yaw = pose_values
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def make_scene_interface():
    try:
        return moveit_commander.PlanningSceneInterface(synchronous=True)
    except TypeError:
        rospy.logwarn(
            "synchronous PlanningSceneInterface is unavailable; using asynchronous mode"
        )
        return moveit_commander.PlanningSceneInterface()


def wait_for_known_state(scene, names, should_exist, timeout):
    # Wall time keeps the safety timeout effective even when /use_sim_time is
    # enabled but no /clock has been published yet.
    deadline = time.monotonic() + timeout
    rate = rospy.Rate(10)
    names = set(names)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        known = set(scene.get_known_object_names())
        attached = set(scene.get_attached_objects(list(names)).keys())
        if should_exist:
            if names.issubset(known) and not (names & attached):
                return True
        elif not (names & known) and not (names & attached):
            return True
        rate.sleep()
    return False


def normalize_objects(raw_objects, default_frame, scene_id):
    if not isinstance(raw_objects, list):
        raise ValueError("~objects must be a list")

    normalized = []
    seen = set()
    for index, raw in enumerate(raw_objects):
        label = "objects[{}]".format(index)
        if not isinstance(raw, dict):
            raise ValueError("{} must be a dictionary".format(label))
        enabled = as_bool(raw.get("enabled", True), "{}.enabled".format(label))
        measured = as_bool(raw.get("measured", False), "{}.measured".format(label))
        short_name = str(raw.get("name", "")).strip()
        if not short_name:
            raise ValueError("{}.name must not be empty".format(label))
        if short_name in seen:
            raise ValueError("duplicate object name: {}".format(short_name))
        seen.add(short_name)

        object_type = str(raw.get("type", "box")).strip().lower()
        if object_type not in ("box", "cylinder"):
            raise ValueError(
                "{}.type must be 'box' or 'cylinder'".format(label)
            )
        frame_id = str(raw.get("frame_id", default_frame)).strip()
        if not frame_id:
            raise ValueError("{}.frame_id must not be empty".format(label))
        pose = finite_vector(raw.get("pose", []), 6, "{}.pose".format(label))

        if object_type == "box":
            size = finite_vector(raw.get("size", []), 3, "{}.size".format(label))
            if any(item <= 0.0 for item in size):
                raise ValueError("{}.size values must be > 0".format(label))
        else:
            size = finite_vector(raw.get("size", []), 2, "{}.size".format(label))
            if any(item <= 0.0 for item in size):
                raise ValueError(
                    "{}.size [height, radius] values must be > 0".format(label)
                )

        color = finite_vector(
            raw.get("color", [0.55, 0.55, 0.55, 1.0]),
            4,
            "{}.color".format(label),
        )
        if any(component < 0.0 or component > 1.0 for component in color):
            raise ValueError("{}.color RGBA values must be in [0, 1]".format(label))

        normalized.append(
            {
                "name": "{}__{}".format(scene_id, short_name),
                "short_name": short_name,
                "type": object_type,
                "enabled": enabled,
                "measured": measured,
                "frame_id": frame_id,
                "pose": pose,
                "size": size,
                "color": color,
                "note": str(raw.get("note", "")).strip(),
            }
        )
    return normalized


def validate_measurements(objects, require_measured):
    unmeasured = [
        item["short_name"]
        for item in objects
        if item["enabled"] and not item["measured"]
    ]
    if require_measured and unmeasured:
        raise RuntimeError(
            "enabled collision objects are not confirmed as measured: {}. "
            "Edit the YAML and set measured=true only after checking dimensions and pose."
            .format(", ".join(unmeasured))
        )
    return unmeasured


def apply_object_colors(objects, timeout):
    if not objects:
        return

    service_name = "/apply_planning_scene"
    rospy.wait_for_service(service_name, timeout=timeout)
    request = ApplyPlanningSceneRequest()
    request.scene = PlanningScene()
    request.scene.is_diff = True

    for item in objects:
        object_color = ObjectColor()
        object_color.id = item["name"]
        object_color.color.r = item["color"][0]
        object_color.color.g = item["color"][1]
        object_color.color.b = item["color"][2]
        object_color.color.a = item["color"][3]
        request.scene.object_colors.append(object_color)

    response = rospy.ServiceProxy(service_name, ApplyPlanningScene)(request)
    if not response.success:
        raise RuntimeError("MoveIt refused the collision-object color update")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("publish_real_environment_scene", anonymous=False)

    scene_id = str(rospy.get_param("~scene_id", "right_arm_lab")).strip()
    default_frame = str(rospy.get_param("~frame_id", "base")).strip()
    require_measured = as_bool(
        rospy.get_param("~require_measured", True), "~require_measured"
    )
    dry_run = as_bool(rospy.get_param("~dry_run", False), "~dry_run")
    remove_only = as_bool(rospy.get_param("~remove_only", False), "~remove_only")
    timeout = float(rospy.get_param("~scene_timeout", 8.0))

    if not scene_id or "__" in scene_id:
        raise ValueError("~scene_id must be non-empty and must not contain '__'")
    if not default_frame:
        raise ValueError("~frame_id must not be empty")
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("~scene_timeout must be finite and > 0")

    objects = normalize_objects(
        rospy.get_param("~objects", []), default_frame, scene_id
    )
    unmeasured = validate_measurements(objects, require_measured)
    enabled = [item for item in objects if item["enabled"]]

    rospy.loginfo(
        "Validated environment scene '%s': objects=%d enabled=%d frame=%s dry_run=%s",
        scene_id,
        len(objects),
        len(enabled),
        default_frame,
        dry_run,
    )
    for item in enabled:
        rospy.loginfo(
            "  %s type=%s frame=%s pose=%s size=%s color=%s measured=%s note=%s",
            item["name"],
            item["type"],
            item["frame_id"],
            item["pose"],
            item["size"],
            item["color"],
            item["measured"],
            item["note"],
        )
    if unmeasured:
        rospy.logwarn(
            "UNMEASURED PREVIEW ONLY: %s. This scene must not be used to approve real execution.",
            ", ".join(unmeasured),
        )
    if dry_run:
        rospy.loginfo("dry_run=true; validation complete, PlanningScene was not changed")
        return

    rospy.wait_for_service("/get_planning_scene", timeout=timeout)
    scene = make_scene_interface()
    rospy.sleep(1.0)

    prefix = "{}__".format(scene_id)
    managed_names = [
        name for name in scene.get_known_object_names() if name.startswith(prefix)
    ]
    # Remove only objects MoveIt reports as present. Publishing REMOVE for
    # every configured-but-absent object produces misleading warning spam.
    names_to_remove = sorted(set(managed_names))
    for name in names_to_remove:
        scene.remove_world_object(name)
    if names_to_remove and not wait_for_known_state(
        scene, names_to_remove, should_exist=False, timeout=timeout
    ):
        raise RuntimeError("timed out removing old scene objects")

    if remove_only:
        rospy.loginfo("remove_only=true; removed %d managed objects", len(names_to_remove))
        return

    for item in enabled:
        pose = make_pose(item["frame_id"], item["pose"])
        if item["type"] == "box":
            scene.add_box(item["name"], pose, size=tuple(item["size"]))
        else:
            height, radius = item["size"]
            scene.add_cylinder(item["name"], pose, height=height, radius=radius)

    enabled_names = [item["name"] for item in enabled]
    if enabled_names and not wait_for_known_state(
        scene, enabled_names, should_exist=True, timeout=timeout
    ):
        known = scene.get_known_object_names()
        raise RuntimeError(
            "timed out adding scene objects; known objects are: {}".format(known)
        )

    apply_object_colors(enabled, timeout)

    rospy.loginfo(
        "Applied environment scene '%s' with %d collision objects",
        scene_id,
        len(enabled_names),
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("Environment scene update refused/failed: %s", exc)
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
