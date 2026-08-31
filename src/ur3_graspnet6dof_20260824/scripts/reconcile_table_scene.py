#!/usr/bin/python3

"""Replace the stale low tabletop proxy with one surface at the RGB-D height.

Only PlanningScene objects are changed.  The shared real-environment YAML is
never edited, and relaunching that environment restores its original objects.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))

import moveit_commander
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest

from ur3_graspnet6dof.config import load_config


def close_vector(actual, expected, tolerance):
    return len(actual) == len(expected) and all(
        abs(float(a) - float(e)) <= tolerance for a, e in zip(actual, expected)
    )


def wait_known(scene, object_id, expected, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        present = object_id in scene.get_known_object_names()
        if present == expected:
            return
        rospy.sleep(0.05)
    raise RuntimeError("PlanningScene state timeout for {}".format(object_id))


def set_color(object_id, rgba):
    request = ApplyPlanningSceneRequest()
    request.scene = PlanningScene()
    request.scene.is_diff = True
    colour = ObjectColor()
    colour.id = object_id
    colour.color.r, colour.color.g, colour.color.b, colour.color.a = rgba
    request.scene.object_colors.append(colour)
    rospy.wait_for_service("/apply_planning_scene", timeout=3.0)
    response = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)(request)
    if not response.success:
        raise RuntimeError("MoveIt refused tabletop colour update")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", default=str(PROJECT_ROOT / "config/right_arm_green_table.yaml")
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    config = load_config(args.config)
    settings = config["scene_reconciliation"]
    if not settings.get("enabled", False):
        raise RuntimeError("scene reconciliation is disabled")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_graspnet6d_reconcile_table_scene")
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    rospy.sleep(0.5)

    object_id = settings["source_object_id"]
    objects = scene.get_objects([object_id])
    if object_id not in objects:
        raise RuntimeError("required tabletop collision object is absent: {}".format(object_id))
    item = objects[object_id]
    if len(item.primitives) != 1 or item.primitives[0].type != 1:
        raise RuntimeError("tabletop is not the expected single BOX primitive")
    size = [float(v) for v in item.primitives[0].dimensions]
    tolerance = float(settings["tolerance"])
    source_size = [float(v) for v in settings["expected_source_size"]]
    corrected_size = [float(v) for v in settings["corrected_size"]]
    if not (
        close_vector(size, source_size, tolerance)
        or close_vector(size, corrected_size, tolerance)
    ):
        raise RuntimeError(
            "tabletop size changed: {} matches neither source {} nor corrected {}".format(
                size, source_size, corrected_size
            )
        )

    current_top = float(item.pose.position.z) + float(size[2]) / 2.0
    old_top = float(settings["expected_old_top_z"])
    corrected_top = float(settings["corrected_top_z"])
    if min(abs(current_top - old_top), abs(current_top - corrected_top)) > tolerance:
        raise RuntimeError(
            "tabletop top z {:.4f} matches neither old nor corrected height".format(current_top)
        )

    guard_id = settings["obsolete_guard_id"]
    if guard_id in scene.get_known_object_names():
        scene.remove_world_object(guard_id)
        wait_known(scene, guard_id, False)

    pose = PoseStamped()
    pose.header.frame_id = item.header.frame_id or "base"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x, pose.pose.position.y = [
        float(value) for value in settings["corrected_center_xy"]
    ]
    pose.pose.position.z = corrected_top - float(corrected_size[2]) / 2.0
    quaternion = tf.transformations.quaternion_from_euler(
        0.0, 0.0, float(settings.get("corrected_yaw", 0.0))
    )
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = quaternion
    scene.remove_world_object(object_id)
    wait_known(scene, object_id, False)
    scene.add_box(object_id, pose, size=tuple(corrected_size))
    wait_known(scene, object_id, True)
    set_color(object_id, [float(v) for v in settings["color"]])

    report = {
        "schema_version": 1,
        "created_at": time.time(),
        "object_id": object_id,
        "removed_guard_id": guard_id,
        "previous_top_z": current_top,
        "corrected_top_z": corrected_top,
        "center": [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        "source_size": size,
        "corrected_size": corrected_size,
        "corrected_yaw": float(settings.get("corrected_yaw", 0.0)),
        "source_files_modified": False,
    }
    runtime = PROJECT_ROOT / "runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    temporary = runtime / "table_scene_reconciliation.json.tmp"
    temporary.write_text(json.dumps(report, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(runtime / "table_scene_reconciliation.json"))
    rospy.loginfo(
        "Single tabletop ready: %s size %s -> %s center=(%.3f,%.3f) top %.4f -> %.4f; obsolete guard removed",
        object_id,
        size,
        corrected_size,
        pose.pose.position.x,
        pose.pose.position.y,
        current_top,
        corrected_top,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logerr("%s", exc)
        else:
            print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
