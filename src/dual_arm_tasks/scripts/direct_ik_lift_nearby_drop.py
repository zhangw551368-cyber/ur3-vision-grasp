#!/usr/bin/python3

import argparse
import math
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


RIGHT_ARM_JOINTS = [
    "right_arm_shoulder_pan_joint",
    "right_arm_shoulder_lift_joint",
    "right_arm_elbow_joint",
    "right_arm_wrist_1_joint",
    "right_arm_wrist_2_joint",
    "right_arm_wrist_3_joint",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Close at current pose, lift 5 cm, find a nearby IK-reachable drop point, lower 4 cm, open."
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--group", default="right_arm")
    parser.add_argument("--base-frame", default="right_arm_base")
    parser.add_argument("--tool-frame", default="right_arm_tool0")
    parser.add_argument("--lift-dz", type=float, default=0.05)
    parser.add_argument("--lower-dz", type=float, default=0.04)
    parser.add_argument("--search-half-width", type=float, default=0.025)
    parser.add_argument("--ik-timeout", type=float, default=0.25)
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--segment-time", type=float, default=0.75)
    parser.add_argument("--gripper-topic", default="/right_arm/Robotiq2FGripperRobotOutput")
    parser.add_argument("--robot-program-topic", default="/right_arm/ur_hardware_interface/robot_program_running")
    parser.add_argument("--trajectory-topic", default="/right_arm/scaled_pos_joint_traj_controller/command")
    parser.add_argument("--controller-state-topic", default="/right_arm/scaled_pos_joint_traj_controller/state")
    parser.add_argument("--open-position", type=int, default=0)
    parser.add_argument("--close-position", type=int, default=255)
    parser.add_argument("--gripper-speed", type=int, default=120)
    parser.add_argument("--gripper-force", type=int, default=80)
    parser.add_argument("--joint-tolerance", type=float, default=0.035)
    parser.add_argument("--trajectory-timeout", type=float, default=8.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def nearest_equivalent(value, reference):
    adjusted = float(value)
    while adjusted - reference > math.pi:
        adjusted -= 2.0 * math.pi
    while adjusted - reference < -math.pi:
        adjusted += 2.0 * math.pi
    return adjusted


def make_pose_like(pose, dx, dy, dz):
    out = Pose()
    out.position.x = pose.position.x + dx
    out.position.y = pose.position.y + dy
    out.position.z = pose.position.z + dz
    out.orientation = pose.orientation
    return out


def make_gripper_command(position, speed, force):
    msg = Robotiq2FGripper_robot_output()
    msg.rACT = 1
    msg.rGTO = 1
    msg.rATR = 0
    msg.rPR = int(position)
    msg.rSP = int(speed)
    msg.rFR = int(force)
    return msg


class DirectIKLiftNearbyDrop:
    def __init__(self, args):
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.gripper = rospy.Publisher(
            args.gripper_topic, Robotiq2FGripper_robot_output, queue_size=1
        )
        self.trajectory_pub = rospy.Publisher(
            args.trajectory_topic, JointTrajectory, queue_size=1
        )

    def wait_ready(self):
        msg = rospy.wait_for_message(self.args.robot_program_topic, Bool, timeout=3.0)
        if not msg.data:
            raise RuntimeError("Right arm External Control is not running.")
        rospy.wait_for_service("/compute_ik", timeout=5.0)

    def current_tool_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.args.base_frame,
            self.args.tool_frame,
            rospy.Time(0),
            rospy.Duration(3.0),
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def current_robot_state(self):
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
        state = RobotState()
        state.joint_state = msg
        return state

    def joint_positions_from_state(self, state):
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        return [float(values[name]) for name in RIGHT_ARM_JOINTS]

    def seed_from_positions(self, positions):
        state = self.current_robot_state()
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        for name, position in zip(RIGHT_ARM_JOINTS, positions):
            values[name] = position
        state.joint_state.position = [values[name] for name in state.joint_state.name]
        state.joint_state.velocity = []
        state.joint_state.effort = []
        return state

    def solve_ik(self, pose, seed_state, reference_positions):
        request = PositionIKRequest()
        request.group_name = self.args.group
        request.ik_link_name = self.args.tool_frame
        request.avoid_collisions = bool(self.args.avoid_collisions)
        request.timeout = rospy.Duration(self.args.ik_timeout)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.args.base_frame
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = pose
        request.pose_stamped = pose_stamped
        request.robot_state = seed_state
        response = self.ik(request)
        if response.error_code.val != response.error_code.SUCCESS:
            return None
        values = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        result = []
        for name, reference in zip(RIGHT_ARM_JOINTS, reference_positions):
            result.append(nearest_equivalent(values[name], reference))
        return result

    def candidate_offsets(self):
        h = float(self.args.search_half_width)
        raw = [
            (h, h),
            (h, -h),
            (-h, h),
            (-h, -h),
            (h, 0.0),
            (0.0, h),
            (-h, 0.0),
            (0.0, -h),
            (0.0, 0.0),
        ]
        return raw

    def find_trajectory(self, current_pose):
        start_state = self.current_robot_state()
        current_joints = self.joint_positions_from_state(start_state)
        lift_pose = make_pose_like(current_pose, 0.0, 0.0, self.args.lift_dz)
        lift_joints = self.solve_ik(lift_pose, start_state, current_joints)
        if lift_joints is None:
            raise RuntimeError("No IK for 5 cm lift from current pose.")

        lift_seed = self.seed_from_positions(lift_joints)
        for dx, dy in self.candidate_offsets():
            high_pose = make_pose_like(current_pose, dx, dy, self.args.lift_dz)
            release_pose = make_pose_like(
                current_pose, dx, dy, self.args.lift_dz - self.args.lower_dz
            )
            high_joints = self.solve_ik(high_pose, lift_seed, lift_joints)
            if high_joints is None:
                continue
            release_seed = self.seed_from_positions(high_joints)
            release_joints = self.solve_ik(release_pose, release_seed, high_joints)
            if release_joints is None:
                continue
            rospy.loginfo(
                "Selected nearby drop offset dx=%.3f dy=%.3f release tool xyz=[%.3f, %.3f, %.3f]",
                dx,
                dy,
                release_pose.position.x,
                release_pose.position.y,
                release_pose.position.z,
            )
            return current_joints, lift_joints, high_joints, release_joints
        raise RuntimeError("No IK-reachable nearby drop point in 5cm x 5cm search area.")

    def command_gripper(self, position, label):
        rospy.loginfo("Gripper %s position=%d", label, position)
        if not self.args.execute:
            return
        deadline = time.time() + 5.0
        while self.gripper.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.gripper.get_num_connections() == 0:
            raise RuntimeError("No gripper subscriber on {}".format(self.args.gripper_topic))
        command = make_gripper_command(
            position, self.args.gripper_speed, self.args.gripper_force
        )
        for _ in range(8):
            self.gripper.publish(command)
            rospy.sleep(0.04)
        rospy.sleep(0.7)

    def publish_trajectory(self, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(RIGHT_ARM_JOINTS)
        for index, joint_positions in enumerate(positions[1:], start=1):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in joint_positions]
            point.time_from_start = rospy.Duration(self.args.segment_time * index)
            trajectory.points.append(point)
        rospy.loginfo(
            "Trajectory points: %d, total time %.2fs",
            len(trajectory.points),
            self.args.segment_time * len(trajectory.points),
        )
        if not self.args.execute:
            return
        deadline = time.time() + 5.0
        while self.trajectory_pub.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.trajectory_pub.get_num_connections() == 0:
            raise RuntimeError("No controller subscriber on {}".format(self.args.trajectory_topic))
        self.trajectory_pub.publish(trajectory)
        self.wait_for_final(positions[-1])

    def wait_for_final(self, target):
        deadline = time.time() + self.args.trajectory_timeout
        last_error = None
        while time.time() < deadline and not rospy.is_shutdown():
            current = self.joint_positions_from_state(self.current_robot_state())
            last_error = max(abs(a - b) for a, b in zip(current, target))
            if last_error <= self.args.joint_tolerance:
                rospy.loginfo("Trajectory reached, max joint error=%.3frad", last_error)
                return
            rospy.sleep(0.1)
        raise RuntimeError(
            "Trajectory did not finish; last joint error={:.3f}rad".format(
                last_error if last_error is not None else float("nan")
            )
        )

    def run(self):
        self.wait_ready()
        current_pose = self.current_tool_pose()
        rospy.loginfo(
            "Current tool xyz=[%.3f, %.3f, %.3f]",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
        )
        positions = self.find_trajectory(current_pose)
        self.command_gripper(self.args.close_position, "close")
        self.publish_trajectory(positions)
        self.command_gripper(self.args.open_position, "open")


def main():
    args = parse_args()
    rospy.init_node("direct_ik_lift_nearby_drop")
    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    DirectIKLiftNearbyDrop(args).run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
