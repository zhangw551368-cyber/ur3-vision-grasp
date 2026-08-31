#!/usr/bin/python3

"""Plan or execute an ordered RGB-D pick sequence into checkerboard slots."""

import argparse
import copy
import json
import math
import os
import pickle
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import moveit_commander
import numpy as np
import rospy
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from graspnet_pick_executor import GuardedGraspExecutor
from ur3_graspnet6dof.config import load_config
from ur3_graspnet6dof.disk_center import detect_circular_disk
from ur3_graspnet6dof.geometry import transform_point
from ur3_graspnet6dof.hex_opening import (
    detect_hexagonal_opening,
    fit_circle_center_xy,
)
from ur3_graspnet6dof.rgb_fallback import (
    build_topdown_candidate,
    pixel_on_horizontal_plane,
)
from ur3_graspnet6dof.ros_image import decode_color, decode_depth_metres, intrinsic_matrix


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", default=str(PROJECT_ROOT / "config/right_arm_green_table.yaml")
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--inspect-slots", action="store_true")
    parser.add_argument(
        "--categories",
        default="",
        help="comma-separated detected categories to plan, preserving configured order",
    )
    parser.add_argument(
        "--max-objects", type=int, default=0, help="stop after this many visible objects"
    )
    parser.add_argument(
        "--object-indices",
        default="",
        help="comma-separated 1-based indices in the frozen pickable detection list",
    )
    parser.add_argument(
        "--target-specs",
        default="",
        help=(
            "frozen ordered targets as category:u:v:radius entries separated by "
            "commas; bypasses category-label jitter"
        ),
    )
    parser.add_argument(
        "--placement-offset",
        type=int,
        default=0,
        help=(
            "skip this many checkerboard placement candidates; useful when "
            "incrementally planning additional objects into distinct locations"
        ),
    )
    parser.add_argument(
        "--drop-target-base",
        default="",
        help=(
            "reuse one explicit TCP release point x:y:z in the base frame for "
            "every object; bypasses checkerboard placement sampling"
        ),
    )
    parser.add_argument(
        "--drop-target-pixel",
        default="",
        help="optional audited RGB pixel u:v corresponding to --drop-target-base",
    )
    parser.add_argument(
        "--hex-tube-drop",
        action="store_true",
        help=(
            "detect the dark hexagonal tube opening inside the white-paper "
            "region and release at its geometric centre"
        ),
    )
    parser.add_argument(
        "--disk-center-drop",
        action="store_true",
        help=(
            "detect the outer rim of a circular disk inside the white-paper "
            "region and release over its metric centre"
        ),
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def live_detections(config):
    topic = config["ros"]["namespace"] + "/detected_objects_json"
    message = rospy.wait_for_message(topic, String, timeout=5.0)
    payload = json.loads(message.data)
    stamp = payload.get("stamp", {})
    stamp_value = float(stamp.get("secs", 0)) + float(stamp.get("nsecs", 0)) * 1e-9
    age = rospy.Time.now().to_sec() - stamp_value
    if age > float(config["sequence"]["max_detection_age"]):
        raise RuntimeError("object detections are stale by {:.2f}s".format(age))
    return payload


def ordered_objects(payload, settings):
    # Keep every physical instance.  A dict keyed only by category silently
    # discarded duplicates such as the two independently detected metal discs.
    by_category = {}
    for item in payload["objects"]:
        if item.get("pickable", False):
            by_category.setdefault(item["category"], []).append(item)
    ordered = []
    for category in settings["object_order"]:
        ordered.extend(by_category.get(category, []))
    return ordered


def board_candidates(payload, settings):
    boards = [
        item
        for item in payload["objects"]
        if item["category"] == settings["board_category"]
    ]
    if len(boards) != 1:
        raise RuntimeError("expected exactly one detected checkerboard")
    x1, y1, x2, y2 = [float(value) for value in boards[0]["bbox"]]
    inset = float(settings["board_bbox_inset"])
    x1, x2 = x1 + inset * (x2 - x1), x2 - inset * (x2 - x1)
    y1, y2 = y1 + inset * (y2 - y1), y2 - inset * (y2 - y1)
    return [
        [int(round(x1 + float(slot[0]) * (x2 - x1))),
         int(round(y1 + float(slot[1]) * (y2 - y1)))]
        for slot in settings["board_candidate_points_normalized"]
    ]


def depth_at(depth, u, v, radius):
    y1, y2 = max(0, v - radius), min(depth.shape[0], v + radius + 1)
    x1, x2 = max(0, u - radius), min(depth.shape[1], u + radius + 1)
    values = depth[y1:y2, x1:x2]
    values = values[np.isfinite(values) & (values > 0.20) & (values < 1.20)]
    if values.size < 12:
        raise RuntimeError("insufficient board depth near pixel ({},{})".format(u, v))
    return float(np.median(values))


def placement_targets(config, executor, payload, candidate_pixels):
    depth_message = rospy.wait_for_message(config["camera"]["depth_topic"], Image, timeout=4.0)
    info = rospy.wait_for_message(config["camera"]["camera_info_topic"], CameraInfo, timeout=4.0)
    depth = decode_depth_metres(depth_message)
    intrinsic = intrinsic_matrix(info)
    base_from_camera = executor.camera_transform(depth_message.header.frame_id)
    release_height = float(config["sequence"]["release_height_above_board"])
    expected_z = float(config["selector"]["support_plane_z"])
    radius = int(config["sequence"]["board_depth_window_px"])
    results = []
    for index, (u, v) in enumerate(candidate_pixels):
        ray_camera = np.array(
            [
                (u - intrinsic[0, 2]) / intrinsic[0, 0],
                (v - intrinsic[1, 2]) / intrinsic[1, 1],
                1.0,
            ],
            dtype=float,
        )
        method = "aligned_depth"
        try:
            z = depth_at(depth, u, v, radius)
            point_base = transform_point(base_from_camera, ray_camera * z)
            if abs(float(point_base[2]) - expected_z) > 0.035:
                raise RuntimeError("depth does not lie on the support surface")
        except RuntimeError:
            # Checkerboard black/white areas commonly contain RealSense depth
            # holes.  Since the board is physically flat on the validated
            # table, intersect its calibrated camera ray with that plane.
            method = "calibrated_ray_table_intersection"
            ray_base = base_from_camera[:3, :3].dot(ray_camera)
            origin_base = base_from_camera[:3, 3]
            if abs(float(ray_base[2])) < 1e-8:
                raise RuntimeError("board pixel ray is parallel to the table")
            scale = (expected_z - float(origin_base[2])) / float(ray_base[2])
            if scale <= 0.0:
                raise RuntimeError("board table-plane intersection is behind camera")
            point_base = origin_base + scale * ray_base
        drop_tcp = point_base.copy()
        drop_tcp[2] += release_height
        results.append(
            {
                "candidate_id": index + 1,
                "pixel": [u, v],
                "board_point_base": point_base.tolist(),
                "drop_tcp_base": drop_tcp.tolist(),
                "release_height": release_height,
                "projection_method": method,
            }
        )
    return results


def explicit_placement_targets(args, config, object_count, image_size):
    values = [value.strip() for value in args.drop_target_base.split(":")]
    if len(values) != 3:
        raise RuntimeError("--drop-target-base must be x:y:z in metres")
    drop_tcp = [float(value) for value in values]
    if not all(np.isfinite(drop_tcp)):
        raise RuntimeError("explicit drop target contains a non-finite value")
    bounds = config["selector"]["workspace_bounds"]
    for value, axis in zip(drop_tcp[:2], ("x", "y")):
        lower, upper = [float(v) for v in bounds[axis]]
        if not lower <= value <= upper:
            raise RuntimeError("explicit drop target is outside workspace_{}".format(axis))
    table_z = float(config["selector"]["support_plane_z"])
    if not table_z + 0.05 <= drop_tcp[2] <= 0.40:
        raise RuntimeError("explicit drop target z is outside the guarded release range")

    pixel = None
    if args.drop_target_pixel:
        pixel_values = [value.strip() for value in args.drop_target_pixel.split(":")]
        if len(pixel_values) != 2:
            raise RuntimeError("--drop-target-pixel must be u:v")
        pixel = [int(pixel_values[0]), int(pixel_values[1])]
        width, height = [int(value) for value in image_size]
        if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            raise RuntimeError("explicit drop target pixel is outside the RGB image")

    return [
        {
            "candidate_id": index + 1,
            "pixel": pixel,
            "drop_tcp_base": list(drop_tcp),
            "projection_method": "explicit_rgbd_target_in_base",
            "target_type": "shared_release_point",
        }
        for index in range(args.placement_offset + object_count)
    ]


def hex_tube_placement_targets(args, config, executor, payload, object_count):
    boards = [
        item
        for item in payload["objects"]
        if item["category"] == config["sequence"]["board_category"]
    ]
    if len(boards) != 1:
        raise RuntimeError("hex tube detection requires exactly one white-paper region")
    color_message = rospy.wait_for_message(
        config["camera"]["color_topic"], Image, timeout=4.0
    )
    rgb = decode_color(color_message)
    detection = detect_hexagonal_opening(
        rgb, boards[0]["bbox"], config.get("tube_drop", {})
    )
    camera_info = rospy.wait_for_message(
        config["camera"]["camera_info_topic"], CameraInfo, timeout=4.0
    )
    intrinsic = intrinsic_matrix(camera_info)
    intrinsics = [intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]]
    base_from_camera = executor.camera_transform(color_message.header.frame_id)
    table_z = float(config["selector"]["support_plane_z"])
    tube_height = float(config["tube_drop"]["tube_height"])
    release_above = float(config["tube_drop"]["release_above_opening"])
    if not 0.05 <= tube_height <= 0.30:
        raise RuntimeError("configured tube height is outside the guarded range")
    if not 0.005 <= release_above <= 0.10:
        raise RuntimeError("configured tube release clearance is outside the guarded range")
    top_z = table_z + tube_height
    image_center = detection["center"]
    circle_observations = []
    min_radius = float(config["tube_drop"].get("min_opening_radius", 0.005))
    max_radius = float(config["tube_drop"].get("max_opening_radius", 0.040))
    for boundary_set in detection["boundary_pixel_sets"]:
        boundary_base = np.asarray(
            [
                pixel_on_horizontal_plane(pixel, intrinsics, base_from_camera, top_z)
                for pixel in boundary_set["pixels"]
            ],
            dtype=float,
        )
        circle_center, circle_radius, circle_rms = fit_circle_center_xy(
            boundary_base[:, :2]
        )
        if not min_radius <= circle_radius <= max_radius:
            continue
        circle_observations.append(
            {
                "threshold": int(boundary_set["threshold"]),
                "center_base_xy": circle_center.tolist(),
                "radius_m": circle_radius,
                "rms_m": circle_rms,
            }
        )
    if len(circle_observations) < 3:
        raise RuntimeError("fewer than three valid plane-unprojected aperture circles")
    circle_centers = np.asarray(
        [item["center_base_xy"] for item in circle_observations], dtype=float
    )
    initial_circle_center = np.median(circle_centers, axis=0)
    circle_distances = np.linalg.norm(
        circle_centers - initial_circle_center, axis=1
    )
    circle_outlier = float(
        config["tube_drop"].get("circle_center_outlier_m", 0.003)
    )
    circle_inliers = circle_distances <= circle_outlier
    if np.count_nonzero(circle_inliers) < 3:
        raise RuntimeError("plane-unprojected aperture circles are inconsistent")
    refined_circle_center = np.median(circle_centers[circle_inliers], axis=0)
    tube_top = np.array(
        [refined_circle_center[0], refined_circle_center[1], top_z], dtype=float
    )
    release_tcp = tube_top.copy()
    release_tcp[2] = top_z + release_above
    bounds = config["selector"]["workspace_bounds"]
    for value, axis in zip(release_tcp[:2], ("x", "y")):
        lower, upper = [float(v) for v in bounds[axis]]
        if not lower <= float(value) <= upper:
            raise RuntimeError("detected tube opening is outside workspace_{}".format(axis))

    detection.update(
        {
            "image_center": list(image_center),
            "image_center_method": detection["center_method"],
            "center_method": "median_plane_unprojected_circle_center",
            "circle_fit_observations": circle_observations,
            "circle_fit_inlier_count": int(np.count_nonzero(circle_inliers)),
            "rgb_stamp": color_message.header.stamp.to_sec(),
            "rgb_frame_id": color_message.header.frame_id,
            "tube_height": tube_height,
            "tube_top_base": tube_top.tolist(),
            "release_above_opening": release_above,
            "release_tcp_base": release_tcp.tolist(),
        }
    )
    placements = [
        {
            "candidate_id": index + 1,
            "pixel": [float(image_center[0]), float(image_center[1])],
            "drop_tcp_base": release_tcp.tolist(),
            "projection_method": "aperture_boundary_to_tube_plane_circle_fit",
            "target_type": "shared_hex_tube_release_point",
            "hex_vertices": detection["vertices"],
            "center_method": detection["center_method"],
        }
        for index in range(args.placement_offset + object_count)
    ]
    return placements, detection


def disk_center_placement_targets(args, config, executor, payload, object_count):
    boards = [
        item
        for item in payload["objects"]
        if item["category"] == config["sequence"]["board_category"]
    ]
    if len(boards) != 1:
        raise RuntimeError("disk detection requires exactly one white-paper region")
    color_message = rospy.wait_for_message(
        config["camera"]["color_topic"], Image, timeout=4.0
    )
    rgb = decode_color(color_message)
    policy = config.get("disk_drop", {})
    detection = detect_circular_disk(rgb, boards[0]["bbox"], policy)
    camera_info = rospy.wait_for_message(
        config["camera"]["camera_info_topic"], CameraInfo, timeout=4.0
    )
    intrinsic = intrinsic_matrix(camera_info)
    intrinsics = [intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]]
    base_from_camera = executor.camera_transform(color_message.header.frame_id)
    table_z = float(config["selector"]["support_plane_z"])
    disk_height = float(policy["disk_height"])
    release_above = float(policy["release_above_center"])
    if not 0.005 <= disk_height <= 0.10:
        raise RuntimeError("configured disk height is outside the guarded range")
    if not 0.005 <= release_above <= 0.10:
        raise RuntimeError("configured disk release clearance is outside the guarded range")
    top_z = table_z + disk_height

    circle_observations = []
    for boundary_set in detection["boundary_pixel_sets"]:
        boundary_base = np.asarray(
            [
                pixel_on_horizontal_plane(pixel, intrinsics, base_from_camera, top_z)
                for pixel in boundary_set["pixels"]
            ],
            dtype=float,
        )
        center_xy, radius, rms = fit_circle_center_xy(boundary_base[:, :2])
        if not float(policy["min_disk_radius"]) <= radius <= float(
            policy["max_disk_radius"]
        ):
            continue
        circle_observations.append(
            {
                "edge_thresholds": boundary_set["threshold"],
                "center_base_xy": center_xy.tolist(),
                "radius_m": radius,
                "rms_m": rms,
            }
        )
    if len(circle_observations) < 3:
        raise RuntimeError("fewer than three valid plane-unprojected disk circles")
    centers = np.asarray(
        [item["center_base_xy"] for item in circle_observations], dtype=float
    )
    initial_center = np.median(centers, axis=0)
    distances = np.linalg.norm(centers - initial_center, axis=1)
    inliers = distances <= float(policy.get("circle_center_outlier_m", 0.003))
    if np.count_nonzero(inliers) < 3:
        raise RuntimeError("plane-unprojected disk circles are inconsistent")
    center_xy = np.median(centers[inliers], axis=0)
    disk_top = np.array([center_xy[0], center_xy[1], top_z], dtype=float)
    release_tcp = disk_top.copy()
    release_tcp[2] = top_z + release_above
    bounds = config["selector"]["workspace_bounds"]
    for value, axis in zip(release_tcp[:2], ("x", "y")):
        lower, upper = [float(v) for v in bounds[axis]]
        if not lower <= float(value) <= upper:
            raise RuntimeError("detected disk centre is outside workspace_{}".format(axis))

    detection.update(
        {
            "image_center": list(detection["center"]),
            "image_center_method": detection["center_method"],
            "center_method": "median_plane_unprojected_outer_disk_circle_center",
            "circle_fit_observations": circle_observations,
            "circle_fit_inlier_count": int(np.count_nonzero(inliers)),
            "rgb_stamp": color_message.header.stamp.to_sec(),
            "rgb_frame_id": color_message.header.frame_id,
            "disk_height": disk_height,
            "disk_top_base": disk_top.tolist(),
            "release_above_center": release_above,
            "release_tcp_base": release_tcp.tolist(),
        }
    )
    placements = [
        {
            "candidate_id": index + 1,
            "pixel": [float(value) for value in detection["image_center"]],
            "drop_tcp_base": release_tcp.tolist(),
            "projection_method": "outer_disk_rim_to_top_plane_circle_fit",
            "target_type": "shared_disk_center_release_point",
            "center_method": detection["center_method"],
        }
        for index in range(args.placement_offset + object_count)
    ]
    return placements, detection


def trajectory_summary(executor, plan):
    return [
        {
            "name": name,
            "points": len(trajectory.joint_trajectory.points),
            "duration": executor.trajectory_duration(trajectory),
            "joint_motion": executor.trajectory_joint_motion(trajectory),
        }
        for name, trajectory in plan["trajectories"]
    ]


def recover_initial(executor):
    current = executor.arm.get_current_state()
    trajectory, _ = executor.plan_initial_joints(current)
    if not executor.arm.execute(trajectory, wait=True):
        executor.arm.stop()
        raise RuntimeError("failed to execute recovery return-to-initial")
    executor.arm.stop()


def write_report(report):
    runtime = PROJECT_ROOT / "runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    temporary = runtime / "multi_sequence_plan.json.tmp"
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(runtime / "multi_sequence_plan.json"))


def main():
    args = parse_args()
    base_config = load_config(args.config)
    target_modes = sum(
        bool(value)
        for value in (
            args.hex_tube_drop,
            args.disk_center_drop,
            args.drop_target_base,
        )
    )
    if target_modes > 1:
        raise RuntimeError(
            "choose only one of --hex-tube-drop, --disk-center-drop, or --drop-target-base"
        )
    if args.execute and not base_config["execution"].get("enabled", False):
        raise RuntimeError("real sequence execution is locked by execution.enabled=false")
    if args.execute and not args.yes:
        answer = input("Plan inspected, table clear and E-stop reachable? Type EXECUTE_SEQUENCE: ")
        if answer != "EXECUTE_SEQUENCE":
            print("Cancelled")
            return

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_graspnet6d_multi_object_sequence", anonymous=True)
    detections = live_detections(base_config)
    settings = base_config["sequence"]
    if args.target_specs:
        objects = []
        for spec in args.target_specs.split(","):
            fields = [value.strip() for value in spec.split(":")]
            if len(fields) not in (4, 6):
                raise RuntimeError(
                    "each target spec must be category:u:v:radius[:axis_u:axis_v]"
                )
            category, u, v, radius = fields[:4]
            major_axis = (
                [float(fields[4]), float(fields[5])] if len(fields) == 6 else None
            )
            objects.append(
                {
                    "category": category,
                    "category_zh": category,
                    "center": [float(u), float(v)],
                    "target_radius": int(radius),
                    "pickable": True,
                    "source": "explicit_frozen_snapshot",
                    "major_axis_image": major_axis,
                }
            )
    elif args.object_indices:
        frozen_pickable = [
            item for item in detections["objects"] if item.get("pickable", False)
        ]
        requested_indices = [
            int(value.strip())
            for value in args.object_indices.split(",")
            if value.strip()
        ]
        if not requested_indices or any(
            index < 1 or index > len(frozen_pickable) for index in requested_indices
        ):
            raise RuntimeError(
                "object indices must address the frozen pickable list 1..{}".format(
                    len(frozen_pickable)
                )
            )
        objects = [frozen_pickable[index - 1] for index in requested_indices]
    else:
        objects = ordered_objects(detections, settings)
    if args.categories and not args.object_indices and not args.target_specs:
        selected = {value.strip() for value in args.categories.split(",") if value.strip()}
        objects = [item for item in objects if item["category"] in selected]
    if args.max_objects > 0:
        objects = objects[: args.max_objects]
    if not objects:
        raise RuntimeError("no ordered pickable objects are visible")

    # One executor provides TF and validates all slot pixels before any plan.
    probe = GuardedGraspExecutor(base_config, False, "pick_hold")
    tube_drop_detection = None
    disk_drop_detection = None
    if args.hex_tube_drop:
        placements, tube_drop_detection = hex_tube_placement_targets(
            args, base_config, probe, detections, len(objects)
        )
        placement_kind = "detected hexagonal tube opening"
    elif args.disk_center_drop:
        placements, disk_drop_detection = disk_center_placement_targets(
            args, base_config, probe, detections, len(objects)
        )
        placement_kind = "detected circular disk centre"
    elif args.drop_target_base:
        placements = explicit_placement_targets(
            args, base_config, len(objects), detections["image_size"]
        )
        placement_kind = "explicit shared drop target"
    else:
        candidate_pixels = board_candidates(detections, settings)
        if len(candidate_pixels) < len(objects):
            raise RuntimeError("not enough checkerboard search samples")
        placements = placement_targets(
            base_config, probe, detections, candidate_pixels
        )
        placement_kind = "checkerboard"
    camera_info = rospy.wait_for_message(
        base_config["camera"]["camera_info_topic"], CameraInfo, timeout=4.0
    )
    rgb_intrinsics = [
        camera_info.K[0],
        camera_info.K[4],
        camera_info.K[2],
        camera_info.K[5],
    ]
    base_from_camera = probe.camera_transform(detections["frame_id"])
    rospy.loginfo("%s placements in base: %s", placement_kind, json.dumps(placements))
    if args.inspect_slots:
        print(json.dumps(placements, ensure_ascii=False, indent=2))
        return
    display_pub = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
    )
    display = DisplayTrajectory()
    report = {
        "schema_version": 1,
        "created_at": time.time(),
        "execute_requested": bool(args.execute),
        "requested_object_indices": args.object_indices,
        "requested_target_specs": args.target_specs,
        "requested_placement_offset": args.placement_offset,
        "requested_drop_target_base": args.drop_target_base,
        "requested_drop_target_pixel": args.drop_target_pixel,
        "hex_tube_drop_requested": bool(args.hex_tube_drop),
        "tube_drop_detection": tube_drop_detection,
        "disk_center_drop_requested": bool(args.disk_center_drop),
        "disk_drop_detection": disk_drop_detection,
        "detection_snapshot": {
            "frame_id": detections.get("frame_id"),
            "stamp": detections.get("stamp"),
            "objects": objects,
        },
        "snapshot_policy": (
            "all object detections, RGB-D grasp candidates and trajectories are "
            "cached before the first robot motion; execution performs no camera reads"
        ),
        "failure_policy": "return_initial_and_continue",
        "release_policy": (
            "unproject multi-threshold aperture boundaries to the tube-top plane, fit the physical circle centre, and release {:.3f} m above the {:.3f} m tube opening".format(
                float(base_config["tube_drop"]["release_above_opening"]),
                float(base_config["tube_drop"]["tube_height"]),
            )
            if args.hex_tube_drop
            else (
                "unproject the outer disk rim to the {:.3f} m disk-top plane, fit its physical circle centre, and release {:.3f} m above it".format(
                    float(base_config["disk_drop"]["disk_height"]),
                    float(base_config["disk_drop"]["release_above_center"]),
                )
                if args.disk_center_drop
                else (
                    "reuse the explicit base-frame TCP point above the target and open "
                    "the gripper"
                    if args.drop_target_base
                    else "choose any reachable free point inside the detected board; open gripper at board surface + {:.3f} m".format(
                        float(settings["release_height_above_board"])
                    )
                )
            )
        ),
        "objects": [],
    }

    if args.placement_offset < 0 or args.placement_offset >= len(placements):
        raise RuntimeError("placement offset is outside the placement target list")
    available_placements = list(placements)[args.placement_offset :]
    # Deliberately separate perception/planning from execution.  Every item is
    # planned while the arm is still at the unobstructed initial pose; only
    # after this loop may real motion begin.  This prevents an arm occlusion
    # from changing the coordinates of later items in a continuous sequence.
    execution_queue = []
    for index, item in enumerate(objects):
        category = item["category"]
        center = [int(round(value)) for value in item["center"]]
        radius = int(item.get("target_radius", settings["target_radius"][category]))
        entry = {
            "order": index + 1,
            "category": category,
            "category_zh": item.get("category_zh", category),
            "target_pixel": center,
            "target_radius": radius,
            "status": "planning",
            "placement_attempts": [],
        }
        rospy.loginfo(
            "Sequence %d/%d: %s pixel=%s -> %s",
            index + 1,
            len(objects),
            category,
            center,
            placement_kind,
        )
        executor = None
        plan = None
        try:
            last_error = None
            placement_limit = int(settings["max_board_candidates_per_object"])
            if settings.get("fixed_board_slot_per_object", False):
                placement_index = args.placement_offset + index
                if placement_index >= len(placements):
                    raise RuntimeError("no dedicated placement target for object")
                object_placements = [placements[placement_index]]
            else:
                object_placements = list(available_placements)[:placement_limit]
            for placement in object_placements:
                item_config = copy.deepcopy(base_config)
                item_config["execution"]["drop_pose"]["position"] = placement[
                    "drop_tcp_base"
                ]
                grasp_policy = settings.get("grasp_policies", {}).get(category, {})
                for key in (
                    "opening_axis_base_xy",
                    "max_opening_axis_error_deg",
                    "min_height_above_plane",
                    "max_height_above_plane",
                    "min_gripper_width",
                    "max_gripper_width",
                ):
                    if key in grasp_policy:
                        item_config["selector"][key] = copy.deepcopy(grasp_policy[key])
                # Bound the cost of an unreachable sample while retaining a
                # configurable search over alternative GraspNet candidates.
                item_config["moveit"]["planning_time"] = min(
                    float(settings.get("planning_time_per_attempt", 5.0)),
                    float(item_config["moveit"]["planning_time"]),
                )
                item_config["moveit"]["plan_retries"] = min(
                    int(settings.get("plan_retries_per_candidate", 2)),
                    int(item_config["moveit"]["plan_retries"]),
                )
                item_config["selector"]["max_candidates_to_plan"] = min(
                    int(settings.get("network_candidates_per_object", 3)),
                    int(item_config["selector"]["max_candidates_to_plan"]),
                )
                executor = GuardedGraspExecutor(
                    item_config,
                    execute=args.execute,
                    mode="pick_drop",
                    target_pixel=center,
                    target_radius=radius,
                )
                attempt = {"board_candidate": placement, "status": "planning"}
                attempt["inference_attempts"] = []
                inference_limit = max(
                    1, int(settings.get("inference_retries_per_object", 1))
                )
                for inference_index in range(inference_limit):
                    inference_attempt = {
                        "attempt": inference_index + 1,
                        "status": "planning",
                    }
                    try:
                        # Request a fresh synchronized RGB-D inference each time.
                        # GraspNet point sampling is stochastic; retries improve
                        # robustness without weakening any safety filter.
                        payload = executor.request_candidates()
                        candidates = executor.prepare_candidates(payload)
                        if not candidates:
                            raise RuntimeError(
                                "all candidates rejected by safety filters"
                            )
                        plan = executor.select_plan(candidates)
                        inference_attempt["status"] = "selected"
                        attempt["inference_attempts"].append(inference_attempt)
                        break
                    except Exception as inference_error:
                        last_error = inference_error
                        inference_attempt["status"] = "rejected"
                        inference_attempt["error"] = str(inference_error)
                        attempt["inference_attempts"].append(inference_attempt)
                        if inference_index + 1 < inference_limit:
                            rospy.logwarn(
                                "%s inference %d/%d rejected: %s; retrying fresh RGB-D",
                                category,
                                inference_index + 1,
                                inference_limit,
                                inference_error,
                            )
                fallback_policy = (
                    settings.get("rgb_fallback", {})
                    .get("categories", {})
                    .get(category)
                )
                if (
                    plan is None
                    and fallback_policy
                    and bool(fallback_policy.get("enabled", True))
                ):
                    fallback_attempt = {
                        "method": "rgb_table_plane_fallback",
                        "status": "planning",
                    }
                    try:
                        prepared = build_topdown_candidate(
                            item,
                            fallback_policy,
                            rgb_intrinsics,
                            base_from_camera,
                            float(item_config["selector"]["support_plane_z"]),
                        )
                        plane = np.array(
                            [
                                0.0,
                                0.0,
                                1.0,
                                -float(item_config["selector"]["support_plane_z"]),
                            ],
                            dtype=float,
                        )
                        executor.support_plane_base = plane
                        rejections = executor.candidate_rejections(
                            prepared["source"],
                            prepared["center"],
                            prepared["approach"],
                            plane,
                        )
                        fallback_attempt["center_base"] = prepared["center"].tolist()
                        fallback_attempt["gripper_width"] = float(
                            prepared["source"]["width"]
                        )
                        fallback_attempt["rejections"] = rejections
                        if rejections:
                            raise RuntimeError(
                                "RGB fallback rejected: {}".format(",".join(rejections))
                            )
                        plan = executor.select_plan([prepared])
                        fallback_attempt["status"] = "selected"
                    except Exception as fallback_error:
                        last_error = fallback_error
                        fallback_attempt["status"] = "rejected"
                        fallback_attempt["error"] = str(fallback_error)
                    attempt["rgb_fallback"] = fallback_attempt
                if plan is not None:
                    attempt["status"] = "selected"
                    entry["placement_attempts"].append(attempt)
                    entry["placement"] = placement
                    if placement in available_placements:
                        available_placements.remove(placement)
                    break
                else:
                    attempt["status"] = "rejected"
                    attempt["error"] = str(last_error)
                    entry["placement_attempts"].append(attempt)
                    rospy.logwarn(
                        "%s placement candidate %d rejected: %s",
                        category,
                        int(placement["candidate_id"]),
                        last_error,
                    )
            if plan is None:
                raise RuntimeError(
                    "no complete path to the requested placement target: {}".format(
                        last_error
                    )
                )
            source = plan["candidate"]["source"]
            # Publish the chosen grasp pose/approach arrow as well as the path.
            # The combined six-segment DisplayTrajectory is published below;
            # this call keeps the selected grasp visually distinct in RViz.
            executor.publish_plan(plan)
            entry.update(
                {
                    "status": "planned",
                    "network_rank": int(source["id"]),
                    "grasp_source": source.get("method", "graspnet"),
                    "score": float(source["score"]),
                    "gripper_width": float(source["width"]),
                    "grasp_center_base": plan["candidate"]["center"].tolist(),
                    "height_above_plane": plan["candidate"]["height_above_plane"],
                    "opening_axis_flip": bool(plan["flip"]),
                    "opening_axis_base": plan["candidate"]["grasp_rotation"][:, 1].tolist(),
                    "opening_axis_error_deg": plan["candidate"].get(
                        "opening_axis_error_deg"
                    ),
                    "trajectories": trajectory_summary(executor, plan),
                }
            )
            if not display.trajectory:
                display.trajectory_start = plan["start_state"]
            display.trajectory.extend(
                [trajectory for _, trajectory in plan["trajectories"]]
            )
            execution_queue.append((entry, executor, plan))
        except Exception as exc:
            entry["status"] = "planning_failed"
            entry["error"] = str(exc)
            rospy.logwarn("Skipping %s: %s", category, exc)
        report["objects"].append(entry)
        write_report(report)

    planned = [item for item in report["objects"] if item["status"] in ("planned", "executed")]
    if display.trajectory:
        for _ in range(3):
            display_pub.publish(display)
            rospy.sleep(0.2)
    report["planned_count"] = len(planned)
    report["failed_count"] = len(report["objects"]) - len(planned)
    report["display_trajectory_count"] = len(display.trajectory)
    report["all_perception_completed_before_motion"] = True
    cache_path = PROJECT_ROOT / "runtime" / "cached_sequence_plan.pkl"
    cache_temp = cache_path.with_suffix(".pkl.tmp")
    cache_payload = {
        "schema_version": 1,
        "created_at": time.time(),
        "planning_frame": base_config["selector"]["planning_frame"],
        "detection_snapshot": report["detection_snapshot"],
        "items": [
            {
                "order": entry["order"],
                "category": entry["category"],
                "category_zh": entry["category_zh"],
                "target_pixel": entry["target_pixel"],
                "target_radius": entry["target_radius"],
                "config": executor.config,
                "plan": plan,
            }
            for entry, executor, plan in execution_queue
        ],
    }
    with cache_temp.open("wb") as stream:
        pickle.dump(cache_payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(str(cache_temp), str(cache_path))
    report["cached_plan_file"] = str(cache_path)
    report["cached_plan_count"] = len(cache_payload["items"])
    write_report(report)
    if not planned:
        raise RuntimeError("no object has a complete pick-place-return path")
    if args.execute:
        rospy.logwarn(
            "PRE-MOTION SNAPSHOT complete: %d objects and %d trajectory segments cached; camera will not be read during execution.",
            len(execution_queue),
            report["display_trajectory_count"],
        )
        for entry, executor, plan in execution_queue:
            try:
                executor.execute_plan(plan)
                entry["status"] = "executed"
            except Exception as exc:
                entry["status"] = "execution_failed"
                entry["error"] = str(exc)
                executor.recover_after_failure(exc)
                recover_initial(executor)
                entry["recovery"] = "returned_initial; continuing"
            write_report(report)
    if args.execute:
        rospy.logwarn(
            "SEQUENCE EXECUTION complete: %d executed, %d skipped, %d trajectory segments.",
            report["planned_count"],
            report["failed_count"],
            report["display_trajectory_count"],
        )
    else:
        rospy.logwarn(
            "SEQUENCE PLAN-ONLY complete: %d planned, %d skipped, %d trajectory segments. No command sent.",
            report["planned_count"],
            report["failed_count"],
            report["display_trajectory_count"],
        )
    rospy.sleep(1.0)


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
