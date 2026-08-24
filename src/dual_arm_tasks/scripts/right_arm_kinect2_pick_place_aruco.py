#!/usr/bin/python3

import argparse
import math
import sys
import time

import moveit_commander
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool


def parse_args():
    parser = argparse.ArgumentParser(
        description="Direct Kinect2 red-block pick and ArUco-board place for the right arm."
    )
    parser.add_argument("--red-topic", default="/red_object/point_base")
    parser.add_argument("--marker-frame", default="place_board_marker")
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--arm-group", default="right_arm")
    parser.add_argument("--end-effector-link", default="right_arm_tool0")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--target-max-spread", type=float, default=0.015)
    parser.add_argument("--tool-to-center", type=float, default=0.050)
    parser.add_argument("--pre-grasp-clearance", type=float, default=0.140)
    parser.add_argument("--lift-distance", type=float, default=0.100)
    parser.add_argument("--place-margin", type=float, default=0.008)
    parser.add_argument("--min-block-height", type=float, default=0.040)
    parser.add_argument("--max-block-height", type=float, default=0.075)
    parser.add_argument("--open-position", type=int, default=0)
    parser.add_argument("--close-position", type=int, default=210)
    parser.add_argument("--speed", type=int, default=80)
    parser.add_argument("--force", type=int, default=80)
    parser.add_argument("--velocity-scaling", type=float, default=0.04)
    parser.add_argument("--acceleration-scaling", type=float, default=0.04)
    parser.add_argument("--planning-time", type=float, default=12.0)
    parser.add_argument("--plan-retries", type=int, default=4)
    parser.add_argument("--cartesian-step", type=float, default=0.005)
    parser.add_argument("--cartesian-min-fraction", type=float, default=0.990)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def make_gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = position
    command.rSP = speed
    command.rFR = force
    return command


class Kinect2PickPlace:
    ORIENTATION = [-0.5, 0.5, 0.5, 0.5]
    APPROACH = np.array([0.0, 1.0, 0.0], dtype=float)

    def __init__(self, args):
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.gripper = rospy.Publisher(
            "/right_arm/Robotiq2FGripperRobotOutput",
            Robotiq2FGripper_robot_output,
            queue_size=1,
        )
        self.display = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )
        self.arm = moveit_commander.MoveGroupCommander(args.arm_group)
        self.arm.set_pose_reference_frame(args.base_frame)
        self.arm.set_end_effector_link(args.end_effector_link)
        self.arm.set_max_velocity_scaling_factor(args.velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(args.acceleration_scaling)
        self.arm.set_planning_time(args.planning_time)
        self.arm.set_num_planning_attempts(max(1, args.plan_retries))
        self.arm.set_goal_position_tolerance(0.012)
        self.arm.set_goal_orientation_tolerance(0.10)
        self.start_state = None

    @staticmethod
    def distance(a, b):
        return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))

    def ensure_external_control(self):
        msg = rospy.wait_for_message(
            "/right_arm/ur_hardware_interface/robot_program_running",
            Bool,
            timeout=3.0,
        )
        if not msg.data:
            raise RuntimeError("Right-arm External Control is not running")
        rospy.loginfo("Right-arm External Control is running.")

    def wait_gripper_ready(self):
        status = rospy.wait_for_message(
            "/right_arm/Robotiq2FGripperRobotInput",
            Robotiq2FGripper_robot_input,
            timeout=3.0,
        )
        if status.gACT != 1 or status.gSTA != 3 or status.gFLT != 0:
            raise RuntimeError(
                "Right gripper is not ready: gACT={} gSTA={} gFLT={}".format(
                    status.gACT, status.gSTA, status.gFLT
                )
            )
        rospy.loginfo("Right gripper ready: gPO=%d", status.gPO)

    def publish_gripper(self, position, label, settle=1.0):
        rospy.loginfo("Gripper %s: rPR=%d", label, position)
        deadline = time.time() + 5.0
        while self.gripper.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.1)
        if self.gripper.get_num_connections() == 0:
            raise RuntimeError("No subscriber on right gripper command topic")
        command = make_gripper_command(position, self.args.speed, self.args.force)
        for _ in range(10):
            self.gripper.publish(command)
            rospy.sleep(0.05)
        rospy.sleep(settle)

    def require_grasp(self):
        deadline = time.time() + 2.5
        last = None
        while time.time() < deadline:
            try:
                last = rospy.wait_for_message(
                    "/right_arm/Robotiq2FGripperRobotInput",
                    Robotiq2FGripper_robot_input,
                    timeout=0.5,
                )
            except rospy.ROSException:
                continue
            if last.gFLT != 0:
                raise RuntimeError("Gripper fault while closing: gFLT={}".format(last.gFLT))
            if last.gOBJ == 2:
                rospy.loginfo("Grasp contact detected: gPO=%d", last.gPO)
                return
        if last is None:
            raise RuntimeError("No gripper status after close")
        raise RuntimeError(
            "No object detected after close: gOBJ={} gPO={}".format(last.gOBJ, last.gPO)
        )

    def sample_red_top(self):
        samples = []
        rospy.loginfo("Sampling red block from %s", self.args.red_topic)
        while len(samples) < self.args.samples and not rospy.is_shutdown():
            msg = rospy.wait_for_message(
                self.args.red_topic, PointStamped, timeout=8.0
            )
            if msg.header.frame_id != self.args.base_frame:
                raise RuntimeError(
                    "Red point frame is {}, expected {}".format(
                        msg.header.frame_id, self.args.base_frame
                    )
                )
            samples.append(np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float))
        mean = np.mean(samples, axis=0)
        spread = max(self.distance(sample, mean) for sample in samples)
        if spread > self.args.target_max_spread:
            raise RuntimeError(
                "Red block point unstable: spread {:.3f}m".format(spread)
            )
        rospy.loginfo(
            "Red top mean xyz=[%.3f, %.3f, %.3f], spread=%.3fm",
            mean[0],
            mean[1],
            mean[2],
            spread,
        )
        return mean

    def sample_marker(self):
        samples = []
        rospy.loginfo("Sampling place board marker %s", self.args.marker_frame)
        while len(samples) < self.args.samples and not rospy.is_shutdown():
            transform = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.marker_frame,
                rospy.Time(0),
                rospy.Duration(3.0),
            ).transform
            samples.append(
                np.array(
                    [
                        transform.translation.x,
                        transform.translation.y,
                        transform.translation.z,
                    ],
                    dtype=float,
                )
            )
            rospy.sleep(0.12)
        mean = np.mean(samples, axis=0)
        spread = max(self.distance(sample, mean) for sample in samples)
        if spread > self.args.target_max_spread:
            raise RuntimeError("Marker point unstable: spread {:.3f}m".format(spread))
        rospy.loginfo(
            "Marker mean xyz=[%.3f, %.3f, %.3f], spread=%.3fm",
            mean[0],
            mean[1],
            mean[2],
            spread,
        )
        return mean

    def pose(self, xyz):
        msg = PoseStamped()
        msg.header.frame_id = self.args.base_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose = Pose()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = self.ORIENTATION[0]
        msg.pose.orientation.y = self.ORIENTATION[1]
        msg.pose.orientation.z = self.ORIENTATION[2]
        msg.pose.orientation.w = self.ORIENTATION[3]
        return msg

    def plan_to_pose(self, name, xyz):
        target = self.pose(xyz)
        trajectory = None
        for attempt in range(1, self.args.plan_retries + 1):
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target)
            result = self.arm.plan()
            candidate = result[1] if isinstance(result, tuple) else result
            self.arm.clear_pose_targets()
            if candidate.joint_trajectory.points:
                trajectory = candidate
                break
            rospy.logwarn("No MoveIt plan for %s attempt %d", name, attempt)
        if trajectory is None:
            raise RuntimeError("No valid MoveIt plan for {}".format(name))
        rospy.loginfo(
            "Executing %s xyz=[%.3f, %.3f, %.3f], points=%d",
            name,
            xyz[0],
            xyz[1],
            xyz[2],
            len(trajectory.joint_trajectory.points),
        )
        if not self.arm.execute(trajectory, wait=True):
            self.arm.stop()
            raise RuntimeError("Execution failed at {}".format(name))
        self.arm.stop()

    def cartesian_to_pose(self, name, xyz):
        target = self.pose(xyz)
        self.arm.set_start_state_to_current_state()
        trajectory, fraction = self.arm.compute_cartesian_path(
            [target.pose],
            self.args.cartesian_step,
            avoid_collisions=True,
        )
        if (
            fraction < self.args.cartesian_min_fraction
            or not trajectory.joint_trajectory.points
        ):
            raise RuntimeError(
                "Incomplete Cartesian path for {}: {:.1%}".format(name, fraction)
            )
        rospy.loginfo(
            "Executing Cartesian %s xyz=[%.3f, %.3f, %.3f], fraction=%.1f%%",
            name,
            xyz[0],
            xyz[1],
            xyz[2],
            fraction * 100.0,
        )
        if not self.arm.execute(trajectory, wait=True):
            self.arm.stop()
            raise RuntimeError("Execution failed at {}".format(name))
        self.arm.stop()

    def run(self):
        self.ensure_external_control()
        self.wait_gripper_ready()

        red_top = self.sample_red_top()
        marker = self.sample_marker()
        raw_height = red_top[2] - marker[2]
        block_height = float(
            np.clip(raw_height, self.args.min_block_height, self.args.max_block_height)
        )
        rospy.loginfo(
            "Block height estimate raw=%.3fm clipped=%.3fm",
            raw_height,
            block_height,
        )

        red_center = red_top.copy()
        red_center[2] = red_top[2] - block_height * 0.5
        place_center = marker.copy()
        place_center[2] = marker[2] + block_height * 0.5 + self.args.place_margin

        grasp = red_center - self.APPROACH * self.args.tool_to_center
        pre_grasp = grasp - self.APPROACH * self.args.pre_grasp_clearance
        lift = grasp + np.array([0.0, 0.0, self.args.lift_distance])
        place = place_center - self.APPROACH * self.args.tool_to_center
        pre_place = place + np.array([0.0, 0.0, self.args.lift_distance])
        retreat = pre_place - self.APPROACH * 0.060

        rospy.loginfo(
            "Computed red_center=[%.3f, %.3f, %.3f], place_center=[%.3f, %.3f, %.3f]",
            red_center[0],
            red_center[1],
            red_center[2],
            place_center[0],
            place_center[1],
            place_center[2],
        )
        rospy.loginfo(
            "Tool poses pre_grasp=%s grasp=%s lift=%s pre_place=%s place=%s",
            [round(x, 3) for x in pre_grasp],
            [round(x, 3) for x in grasp],
            [round(x, 3) for x in lift],
            [round(x, 3) for x in pre_place],
            [round(x, 3) for x in place],
        )

        self.publish_gripper(self.args.open_position, "open before pick", settle=0.5)
        self.plan_to_pose("pre_grasp", pre_grasp)
        self.cartesian_to_pose("grasp", grasp)
        self.publish_gripper(self.args.close_position, "close on red block", settle=0.8)
        self.require_grasp()
        self.cartesian_to_pose("lift", lift)
        self.plan_to_pose("pre_place", pre_place)
        self.cartesian_to_pose("place", place)
        self.publish_gripper(self.args.open_position, "open on board", settle=0.7)
        self.cartesian_to_pose("retreat", retreat)
        rospy.loginfo("Kinect2 red-block pick-and-place complete.")


def main():
    args = parse_args()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_arm_kinect2_pick_place_aruco")
    try:
        Kinect2PickPlace(args).run()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
