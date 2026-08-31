#!/usr/bin/python3

"""Read-only IK scan of the detected checkerboard placement region."""

import json
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from graspnet_pick_executor import GuardedGraspExecutor
from multi_object_sequence import (
    board_candidates,
    live_detections,
    placement_targets,
)
from ur3_graspnet6dof.config import load_config
from ur3_graspnet6dof.geometry import matrix_to_quaternion


def vertical_rotation(yaw):
    opening = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    approach = np.array([0.0, 0.0, -1.0])
    tool_x = np.cross(opening, approach)
    return np.column_stack((tool_x, opening, approach))


def pose(position, rotation):
    result = PoseStamped()
    result.header.frame_id = "base"
    result.header.stamp = rospy.Time.now()
    result.pose.position.x, result.pose.position.y, result.pose.position.z = position
    quaternion = matrix_to_quaternion(rotation)
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = quaternion
    return result


def main():
    config = load_config(PROJECT_ROOT / "config/right_arm_green_table.yaml")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_graspnet6d_board_ik_probe", anonymous=True)
    executor = GuardedGraspExecutor(config, False, "pick_hold")
    detections = live_detections(config)
    pixels = board_candidates(detections, config["sequence"])
    placements = placement_targets(config, executor, detections, pixels)
    current = executor.arm.get_current_state()
    rospy.wait_for_service("/compute_ik", timeout=3.0)
    compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    tcp_offset = float(config["tool"]["tcp_offset_from_tool0"])
    sanity = GetPositionIKRequest()
    sanity.ik_request.group_name = config["moveit"]["arm_group"]
    sanity.ik_request.robot_state = current
    sanity.ik_request.avoid_collisions = True
    sanity.ik_request.ik_link_name = config["moveit"]["end_effector_link"]
    sanity.ik_request.pose_stamped = executor.arm.get_current_pose(
        config["moveit"]["end_effector_link"]
    )
    sanity.ik_request.timeout = rospy.Duration(0.5)
    sanity_response = compute_ik(sanity)
    if sanity_response.error_code.val != sanity_response.error_code.SUCCESS:
        raise RuntimeError(
            "compute_ik sanity check failed for current pose: {}".format(
                sanity_response.error_code.val
            )
        )
    rospy.loginfo("compute_ik sanity check passed for current tool pose")
    results = []
    for placement in placements:
        successes = []
        geometric_successes = []
        board_z = float(placement["board_point_base"][2])
        for release_height in (0.06, 0.10, 0.14, 0.18):
            for degrees in range(0, 360, 45):
                rotation = vertical_rotation(math.radians(degrees))
                tcp_position = np.asarray(placement["drop_tcp_base"], dtype=float)
                tcp_position[2] = board_z + release_height
                tool_position = tcp_position - rotation[:, 2] * tcp_offset
                request = GetPositionIKRequest()
                request.ik_request.group_name = config["moveit"]["arm_group"]
                request.ik_request.robot_state = current
                request.ik_request.avoid_collisions = True
                request.ik_request.ik_link_name = config["moveit"]["end_effector_link"]
                request.ik_request.pose_stamped = pose(tool_position, rotation)
                request.ik_request.timeout = rospy.Duration(0.10)
                response = compute_ik(request)
                if response.error_code.val == response.error_code.SUCCESS:
                    successes.append(
                        {"release_height": release_height, "yaw_deg": degrees}
                    )
                else:
                    request.ik_request.avoid_collisions = False
                    geometric_response = compute_ik(request)
                    if geometric_response.error_code.val == geometric_response.error_code.SUCCESS:
                        geometric_successes.append(
                            {"release_height": release_height, "yaw_deg": degrees}
                        )
        item = dict(placement)
        item["reachable_yaw_deg"] = successes
        item["geometric_only_solutions"] = geometric_successes
        item["reachable"] = bool(successes)
        results.append(item)
        rospy.loginfo(
            "board candidate %d reachable=%s collision_free=%s geometric_only=%s",
            placement["candidate_id"],
            bool(successes),
            successes,
            geometric_successes,
        )
    output = PROJECT_ROOT / "runtime/board_reachability.json"
    output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    try:
        main()
    finally:
        moveit_commander.roscpp_shutdown()
