#!/usr/bin/python3

import argparse
import math
import os
import sys
import time

import moveit_commander
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from single_arm_pick_place import SingleArmPickPlace, load_config


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a fixed right-arm pick/place sequence with small visual XY bias correction."
    )
    parser.add_argument("--config", required=True, help="Task YAML file.")
    parser.add_argument("--execute", action="store_true", help="Allow real robot execution.")
    parser.add_argument("--yes", action="store_true", help="Skip typed execution confirmation.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def wait_for_stable_point(config):
    topic = config.get("vision_target_topic", "/red_block/point_base")
    frame = config.get("planning_frame", "right_arm_base")
    sample_count = int(config.get("target_sample_count", 5))
    timeout = float(config.get("target_timeout", 8.0))
    max_spread = float(config.get("target_max_spread", 0.005))

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    deadline = time.time() + timeout
    samples = []
    while len(samples) < sample_count and time.time() < deadline:
        try:
            msg = rospy.wait_for_message(topic, PointStamped, timeout=1.0)
            if msg.header.frame_id != frame:
                transform = tf_buffer.lookup_transform(
                    frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.5),
                )
                from tf2_geometry_msgs import do_transform_point

                msg = do_transform_point(msg, transform)
            samples.append([msg.point.x, msg.point.y, msg.point.z])
        except (rospy.ROSException, tf2_ros.TransformException) as exc:
            rospy.logwarn_throttle(1.0, "Waiting for visual target point: %s", exc)

    if len(samples) < sample_count:
        raise RuntimeError("No stable visual target point on {}".format(topic))

    center = [sum(sample[i] for sample in samples) / len(samples) for i in range(3)]
    spread = max(distance(sample, center) for sample in samples)
    if spread > max_spread:
        raise RuntimeError(
            "Visual target point is unstable: spread {:.4f}m > {:.4f}m".format(
                spread, max_spread
            )
        )

    rospy.loginfo(
        "Stable visual point in %s: [%.4f, %.4f, %.4f], spread=%.4fm",
        frame,
        center[0],
        center[1],
        center[2],
        spread,
    )
    return center


def ensure_external_control(config, execute):
    if not execute:
        return
    topic = config.get(
        "robot_program_topic", "/right_arm/ur_hardware_interface/robot_program_running"
    )
    running = rospy.wait_for_message(topic, Bool, timeout=2.0)
    if not running.data:
        raise RuntimeError("External Control is not running on the right arm")


def apply_visual_bias(config, visual_point):
    reference = config.get("vision_reference_point")
    if reference is None:
        reference = list(visual_point)
        rospy.logwarn(
            "vision_reference_point is not set; using the current visual point as reference."
        )
    reference = [float(v) for v in reference]

    axes = config.get("apply_visual_delta_axes", [True, True, False])
    axes = [bool(v) for v in axes]
    max_delta = [float(v) for v in config.get("max_visual_delta", [0.05, 0.05, 0.01])]
    raw_delta = [visual_point[i] - reference[i] for i in range(3)]
    applied_delta = [raw_delta[i] if axes[i] else 0.0 for i in range(3)]

    for i, value in enumerate(applied_delta):
        if abs(value) > max_delta[i]:
            raise RuntimeError(
                "Visual delta axis {} is too large: {:.4f}m > {:.4f}m".format(
                    i, abs(value), max_delta[i]
                )
            )

    reference_pose_name = config.get("vision_reference_pose", "grasp")
    reference_pose = config["poses"][reference_pose_name]["translation"]
    fixed_bias = [reference_pose[i] - reference[i] for i in range(3)]
    rospy.loginfo(
        "Fixed visual->%s bias: [%.4f, %.4f, %.4f]m",
        reference_pose_name,
        fixed_bias[0],
        fixed_bias[1],
        fixed_bias[2],
    )
    rospy.loginfo(
        "Raw visual delta: [%.4f, %.4f, %.4f]m; applied delta: [%.4f, %.4f, %.4f]m",
        raw_delta[0],
        raw_delta[1],
        raw_delta[2],
        applied_delta[0],
        applied_delta[1],
        applied_delta[2],
    )

    shift_names = config.get("shift_visual_pose_names", ["pre_grasp", "grasp", "lift"])
    for name in shift_names:
        if name not in config["poses"]:
            raise RuntimeError("shift_visual_pose_names contains unknown pose: {}".format(name))
        xyz = config["poses"][name]["translation"]
        config["poses"][name]["translation"] = [xyz[i] + applied_delta[i] for i in range(3)]
        rospy.loginfo(
            "Corrected %-10s TCP=[%.4f, %.4f, %.4f]",
            name,
            config["poses"][name]["translation"][0],
            config["poses"][name]["translation"][1],
            config["poses"][name]["translation"][2],
        )


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError(
            "Real execution is locked. Set enabled: true in the YAML after planning checks."
        )

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("vision_biased_fixed_pick_place")

    visual_point = wait_for_stable_point(config)
    apply_visual_bias(config, visual_point)
    ensure_external_control(config, args.execute)

    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, and RViz plans checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    SingleArmPickPlace(config, args.execute).run()
    rospy.loginfo("Vision-biased fixed pick-and-place cycle complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
