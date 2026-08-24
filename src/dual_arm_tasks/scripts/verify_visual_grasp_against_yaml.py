#!/usr/bin/python3

import argparse
import math
import sys

import rospy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare a visual grasp point with the taught YAML grasp point."
    )
    parser.add_argument(
        "--config",
        default="/home/gzu/gzu_ws/right_arm_visual_aruco_pick_place.yaml",
        help="YAML containing poses.grasp.translation.",
    )
    parser.add_argument("--object-index", type=int, default=0)
    parser.add_argument("--object-name", default="")
    parser.add_argument("--pose-topic", default="/detected_object_pose_base")
    parser.add_argument("--threshold", type=float, default=0.02)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--offset",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Offset from incoming visual pose to grasp point. Defaults to visual_aruco.marker_to_object_center.",
    )
    parser.add_argument(
        "--no-offset",
        action="store_true",
        help="Compare the incoming visual pose directly, ignoring YAML visual offsets.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("Invalid YAML: {}".format(path))
    return data


def select_object(data, index, name):
    if "objects" not in data:
        return data
    objects = data.get("objects")
    if not isinstance(objects, list) or not objects:
        raise ValueError("objects must be a non-empty list")
    selected = None
    if name:
        for candidate in objects:
            if candidate.get("name") == name:
                selected = candidate
                break
        if selected is None:
            raise ValueError("object name not found: {}".format(name))
    else:
        if index < 0 or index >= len(objects):
            raise ValueError("object-index {} is out of range".format(index))
        selected = objects[index]
    merged = {key: value for key, value in data.items() if key != "objects"}
    merged["poses"] = selected.get("poses")
    return merged


def translation_from_record(record):
    if isinstance(record, dict):
        values = record.get("translation")
    else:
        values = record
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("pose record must contain a 3-element translation")
    return [float(value) for value in values]


def default_offset(config):
    visual = config.get("visual_aruco", {})
    if isinstance(visual, dict) and "marker_to_object_center" in visual:
        return [float(value) for value in visual["marker_to_object_center"]]
    if "marker_to_object_center" in config:
        return [float(value) for value in config["marker_to_object_center"]]
    return [0.0, 0.0, 0.0]


def wait_for_visual_xyz(topic, timeout):
    msg = rospy.wait_for_message(topic, rospy.AnyMsg, timeout=timeout)
    msg_type = msg._connection_header.get("type", "")
    if msg_type == "geometry_msgs/PoseStamped":
        pose = rospy.wait_for_message(topic, PoseStamped, timeout=1.0)
        p = pose.pose.position
        return pose.header.frame_id, [p.x, p.y, p.z]
    if msg_type == "geometry_msgs/PointStamped":
        point = rospy.wait_for_message(topic, PointStamped, timeout=1.0)
        p = point.point
        return point.header.frame_id, [p.x, p.y, p.z]
    raise ValueError("{} has unsupported type {}".format(topic, msg_type))


def main():
    args = parse_args()
    rospy.init_node("verify_visual_grasp_against_yaml")

    config = select_object(load_yaml(args.config), args.object_index, args.object_name)
    poses = config.get("poses")
    if not isinstance(poses, dict) or "grasp" not in poses:
        raise ValueError("YAML must contain poses.grasp")

    yaml_grasp = translation_from_record(poses["grasp"])
    offset = [0.0, 0.0, 0.0] if args.no_offset else (args.offset or default_offset(config))
    frame_id, visual_xyz = wait_for_visual_xyz(args.pose_topic, args.timeout)
    visual_grasp = [visual_xyz[i] + float(offset[i]) for i in range(3)]
    error = [visual_grasp[i] - yaml_grasp[i] for i in range(3)]
    norm = math.sqrt(sum(value * value for value in error))

    rospy.loginfo("YAML grasp xyz:   [%.4f, %.4f, %.4f]", *yaml_grasp)
    rospy.loginfo("Visual pose xyz:  [%.4f, %.4f, %.4f] frame=%s", visual_xyz[0], visual_xyz[1], visual_xyz[2], frame_id)
    rospy.loginfo("Applied offset:   [%.4f, %.4f, %.4f]", *offset)
    rospy.loginfo("Visual grasp xyz: [%.4f, %.4f, %.4f]", *visual_grasp)
    rospy.loginfo("Error xyz/norm:   [%.4f, %.4f, %.4f] / %.4f m", error[0], error[1], error[2], norm)

    if norm > args.threshold:
        rospy.logerr("FAILED: %.4f m exceeds threshold %.4f m", norm, args.threshold)
        return 1
    rospy.loginfo("PASSED: %.4f m is within threshold %.4f m", norm, args.threshold)
    return 0


if __name__ == "__main__":
    sys.exit(main())
