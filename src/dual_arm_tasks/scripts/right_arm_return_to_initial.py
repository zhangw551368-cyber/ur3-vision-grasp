#!/usr/bin/env python3

"""Plan and optionally execute a guarded return to a recorded right-arm pose."""

import argparse
import math
import sys

import moveit_commander
import rospy
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool


DEFAULT_JOINTS = [
    3.731812000274658,
    -1.5940335432635706,
    -1.8345826307879847,
    -1.190634552632467,
    0.13940387964248657,
    1.652244210243225,
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--joints", nargs=6, type=float, default=DEFAULT_JOINTS)
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.05)
    parser.add_argument("--tolerance", type=float, default=0.02)
    return parser.parse_args(rospy.myargv()[1:])


def program_is_running():
    try:
        msg = rospy.wait_for_message(
            "/right_arm/ur_hardware_interface/robot_program_running", Bool, timeout=2.0
        )
        return bool(msg.data)
    except rospy.ROSException:
        return False


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_arm_return_to_initial", anonymous=True)
    args = parse_args()

    if not all(math.isfinite(value) for value in args.joints):
        raise RuntimeError("Joint target contains a non-finite value")

    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("right_arm")
    arm.set_planner_id("RRTConnect")
    arm.set_planning_time(10.0)
    arm.set_num_planning_attempts(12)
    arm.set_max_velocity_scaling_factor(args.velocity)
    arm.set_max_acceleration_scaling_factor(args.acceleration)
    arm.set_goal_joint_tolerance(args.tolerance)
    arm.stop()
    arm.clear_pose_targets()
    arm.set_start_state_to_current_state()
    arm.set_joint_value_target(args.joints)

    result = arm.plan()
    success = result[0] if isinstance(result, tuple) else True
    trajectory = result[1] if isinstance(result, tuple) else result
    points = trajectory.joint_trajectory.points
    if not success or not points:
        raise RuntimeError("MoveIt could not plan a collision-free return trajectory")

    display = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
    )
    preview = DisplayTrajectory()
    preview.trajectory_start = robot.get_current_state()
    preview.trajectory.append(trajectory)
    rospy.sleep(0.3)
    display.publish(preview)

    duration = points[-1].time_from_start.to_sec()
    rospy.loginfo(
        "RETURN_PLAN_OK points=%d duration=%.3fs target=%s",
        len(points), duration, [round(v, 6) for v in args.joints]
    )
    if not args.execute:
        rospy.loginfo("Plan-only mode: robot was not moved")
        return

    if not program_is_running():
        raise RuntimeError("External Control program is not running")

    if not arm.execute(trajectory, wait=True):
        arm.stop()
        raise RuntimeError("Trajectory execution failed")
    arm.stop()

    actual = arm.get_current_joint_values()
    errors = [abs(a - b) for a, b in zip(actual, args.joints)]
    max_error = max(errors)
    rospy.loginfo(
        "RETURN_EXECUTION_OK actual=%s max_joint_error=%.6f",
        [round(v, 6) for v in actual], max_error
    )
    if max_error > args.tolerance:
        raise RuntimeError("Return completed outside joint tolerance")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("RETURN_FAILED: %s", exc)
        raise
    finally:
        moveit_commander.roscpp_shutdown()
