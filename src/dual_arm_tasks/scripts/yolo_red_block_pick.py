#!/usr/bin/python3

import argparse
import copy
import math
import os
import sys
import time

import moveit_commander
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from single_arm_pick_place import SingleArmPickPlace, parse_pose_record


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pick the YOLO-detected red block using fixed safe right-arm grasp geometry."
    )
    parser.add_argument("--config", required=True, help="YAML config file.")
    parser.add_argument("--execute", action="store_true", help="Allow real robot execution.")
    parser.add_argument("--yes", action="store_true", help="Skip typed execution confirmation.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def float_list(values, label, expected_len):
    if not isinstance(values, (list, tuple)) or len(values) != expected_len:
        raise ValueError("{} must be a list of {} numbers".format(label, expected_len))
    return [float(v) for v in values]


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Config is empty or invalid: {}".format(path))
    return config


def wait_for_stable_point(config):
    topic = config.get("yolo_point_topic", "/yolo_red_block/point_base")
    frame = config.get("planning_frame", "right_arm_base")
    sample_count = int(config.get("target_sample_count", 5))
    timeout = float(config.get("target_timeout", 8.0))
    max_spread = float(config.get("target_max_spread", 0.008))

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    deadline = time.time() + timeout
    samples = []
    while len(samples) < sample_count and time.time() < deadline:
        try:
            msg = rospy.wait_for_message(topic, PointStamped, timeout=1.0)
            if msg.header.frame_id != frame:
                transform = tf_buffer.lookup_transform(
                    frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(0.5)
                )
                from tf2_geometry_msgs import do_transform_point

                msg = do_transform_point(msg, transform)
            samples.append([msg.point.x, msg.point.y, msg.point.z])
        except (rospy.ROSException, tf2_ros.TransformException) as exc:
            rospy.logwarn_throttle(1.0, "Waiting for YOLO red block point: %s", exc)

    if len(samples) < sample_count:
        raise RuntimeError("No stable YOLO red block point on {}".format(topic))
    center = [sum(sample[i] for sample in samples) / len(samples) for i in range(3)]
    spread = max(distance(sample, center) for sample in samples)
    if spread > max_spread:
        raise RuntimeError(
            "YOLO red block point is unstable: spread {:.4f}m > {:.4f}m".format(
                spread, max_spread
            )
        )
    rospy.loginfo(
        "Stable YOLO point in %s: [%.4f, %.4f, %.4f], spread=%.4fm",
        frame,
        center[0],
        center[1],
        center[2],
        spread,
    )
    return center


def check_bounds(name, xyz, bounds):
    for axis, value in zip(("x", "y", "z"), xyz):
        if axis not in bounds:
            continue
        low, high = [float(v) for v in bounds[axis]]
        if value < low or value > high:
            raise RuntimeError(
                "{} {}={:.4f} is outside safe bound [{:.4f}, {:.4f}]".format(
                    name, axis, value, low, high
                )
            )


def add(first, second):
    return [first[i] + second[i] for i in range(3)]


def build_dynamic_pick_config(config, yolo_point):
    dynamic = copy.deepcopy(config)
    poses = dynamic["poses"]

    bias = float_list(
        config.get("yolo_point_to_grasp_bias_base", [0.0, 0.0, 0.0]),
        "yolo_point_to_grasp_bias_base",
        3,
    )
    grasp = add(yolo_point, bias)
    if bool(config.get("use_fixed_grasp_z", True)):
        grasp[2] = float(config["fixed_grasp_z"])
    else:
        low, high = [float(v) for v in config.get("grasp_z_bounds", [0.34, 0.40])]
        grasp[2] = min(high, max(low, grasp[2]))

    pre_delta = float_list(
        config.get("pre_grasp_delta_from_grasp", [-0.029, -0.033, -0.043]),
        "pre_grasp_delta_from_grasp",
        3,
    )
    lift_delta = float_list(
        config.get("lift_delta_from_grasp", pre_delta),
        "lift_delta_from_grasp",
        3,
    )
    pre_grasp = add(grasp, pre_delta)
    lift = add(grasp, lift_delta)

    bounds = config.get("safe_grasp_bounds", {})
    check_bounds("grasp", grasp, bounds)
    check_bounds("pre_grasp", pre_grasp, config.get("safe_pre_grasp_bounds", bounds))
    check_bounds("lift", lift, config.get("safe_lift_bounds", bounds))

    poses["pre_grasp"]["translation"] = pre_grasp
    poses["grasp"]["translation"] = grasp
    poses["lift"]["translation"] = lift

    for pose_name, pose in poses.items():
        poses[pose_name] = parse_pose_record(pose, "poses.{}".format(pose_name))

    rospy.loginfo(
        "YOLO grasp TCP=[%.4f, %.4f, %.4f] from point=[%.4f, %.4f, %.4f] bias=[%.4f, %.4f, %.4f]",
        grasp[0],
        grasp[1],
        grasp[2],
        yolo_point[0],
        yolo_point[1],
        yolo_point[2],
        bias[0],
        bias[1],
        bias[2],
    )
    rospy.loginfo(
        "YOLO dynamic pre_grasp TCP=[%.4f, %.4f, %.4f], lift TCP=[%.4f, %.4f, %.4f]",
        pre_grasp[0],
        pre_grasp[1],
        pre_grasp[2],
        lift[0],
        lift[1],
        lift[2],
    )
    return dynamic


def ensure_external_control(config, execute):
    if not execute:
        return
    topic = config.get(
        "robot_program_topic", "/right_arm/ur_hardware_interface/robot_program_running"
    )
    running = rospy.wait_for_message(topic, Bool, timeout=2.0)
    if not running.data:
        raise RuntimeError("External Control is not running on the right arm")


def run_sequence(task, config):
    if task.execute:
        task.setup_io_gripper()
        task.command_gripper("open")
    if bool(config.get("go_home_first", False)):
        task.move("home")
    for pose_name in ("pre_grasp", "grasp"):
        task.move(pose_name)
    if task.execute:
        task.command_gripper("close")
    for pose_name in ("lift", "pre_place", "place"):
        task.move(pose_name)
    if task.execute:
        task.command_gripper("open")
    if bool(config.get("go_retreat", True)):
        task.move("retreat")
    if bool(config.get("go_home_after", True)):
        task.move("home")


def main():
    args = parse_args()
    config = load_yaml(args.config)
    if args.execute and not bool(config.get("enabled", False)):
        raise RuntimeError("Real execution is locked. Set enabled: true after plan-only checks.")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("yolo_red_block_pick")

    yolo_point = wait_for_stable_point(config)
    dynamic = build_dynamic_pick_config(config, yolo_point)
    ensure_external_control(dynamic, args.execute)

    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, and YOLO plan checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    task = SingleArmPickPlace(dynamic, args.execute)
    run_sequence(task, dynamic)
    rospy.loginfo("YOLO red block pick cycle complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
