#!/usr/bin/python3

import argparse
import copy
import math
import sys
import time

import moveit_commander
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import Pose
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool


def parse_args():
    parser = argparse.ArgumentParser(
        description="Close the gripper at the current taught grasp pose, move to a nearby place point, release, and return home."
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--arm-group", default="right_arm")
    parser.add_argument("--planning-frame", default="right_arm_base")
    parser.add_argument("--tool-link", default="right_arm_tool0")
    parser.add_argument("--tcp-offset", nargs=3, type=float, default=[0.0, 0.0, 0.155])
    parser.add_argument("--place-dx", type=float, default=0.15)
    parser.add_argument("--place-dy", type=float, default=0.0)
    parser.add_argument("--lift-dz", type=float, default=0.10)
    parser.add_argument("--release-dz", type=float, default=0.05)
    parser.add_argument("--retreat-dz", type=float, default=0.10)
    parser.add_argument(
        "--pre-place-tcp",
        nargs=7,
        type=float,
        default=None,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Explicit TCP pose for the pre-place waypoint.",
    )
    parser.add_argument(
        "--place-tcp",
        nargs=7,
        type=float,
        default=None,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Explicit TCP pose where the gripper opens.",
    )
    parser.add_argument(
        "--place-input-frame",
        choices=("tcp", "tool"),
        default="tcp",
        help="Interpret --pre-place-tcp/--place-tcp as TCP poses or tool-link poses.",
    )
    parser.add_argument("--eef-step", type=float, default=0.01)
    parser.add_argument("--velocity-scale", type=float, default=0.04)
    parser.add_argument("--accel-scale", type=float, default=0.04)
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--gripper-topic", default="/right_arm/Robotiq2FGripperRobotOutput")
    parser.add_argument("--gripper-status-topic", default="/right_arm/Robotiq2FGripperRobotInput")
    parser.add_argument("--robot-program-topic", default="/right_arm/ur_hardware_interface/robot_program_running")
    parser.add_argument("--open-position", type=int, default=0)
    parser.add_argument("--close-position", type=int, default=255)
    parser.add_argument("--gripper-speed", type=int, default=80)
    parser.add_argument("--gripper-force", type=int, default=80)
    parser.add_argument("--require-object", action="store_true")
    parser.add_argument(
        "--home-tcp",
        nargs=7,
        type=float,
        default=[-0.298, 0.191, 0.541, -0.037, 0.378, 0.605, 0.700],
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def make_gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = int(position)
    command.rSP = int(speed)
    command.rFR = int(force)
    return command


def pose_copy_with_position(pose, xyz):
    result = copy.deepcopy(pose)
    result.position.x = float(xyz[0])
    result.position.y = float(xyz[1])
    result.position.z = float(xyz[2])
    return result


def pose_xyz(pose):
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


def pose_quat(pose):
    return [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]


def add(a, b):
    return [float(x + y) for x, y in zip(a, b)]


def tool_from_tcp(tcp_xyz, quat, offset):
    matrix = tf.transformations.quaternion_matrix(quat)
    rotated = [
        matrix[row][0] * offset[0]
        + matrix[row][1] * offset[1]
        + matrix[row][2] * offset[2]
        for row in range(3)
    ]
    return [float(tcp_xyz[index] - rotated[index]) for index in range(3)]


def tcp_from_tool(tool_xyz, quat, offset):
    matrix = tf.transformations.quaternion_matrix(quat)
    rotated = [
        matrix[row][0] * offset[0]
        + matrix[row][1] * offset[1]
        + matrix[row][2] * offset[2]
        for row in range(3)
    ]
    return [float(tool_xyz[index] + rotated[index]) for index in range(3)]


class CurrentPoseGraspPlace:
    def __init__(self, args):
        self.args = args
        self.arm = moveit_commander.MoveGroupCommander(args.arm_group)
        self.arm.set_pose_reference_frame(args.planning_frame)
        self.arm.set_end_effector_link(args.tool_link)
        self.arm.set_max_velocity_scaling_factor(args.velocity_scale)
        self.arm.set_max_acceleration_scaling_factor(args.accel_scale)
        self.arm.set_planning_time(args.planning_time)
        self.gripper = rospy.Publisher(
            args.gripper_topic, Robotiq2FGripper_robot_output, queue_size=1
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def wait_for_robot_program(self):
        msg = rospy.wait_for_message(
            self.args.robot_program_topic, Bool, timeout=3.0
        )
        if not msg.data:
            raise RuntimeError("Right arm External Control program is not running.")

    def command_gripper(self, position, label):
        rospy.loginfo("Gripper %s position=%d", label, position)
        if not self.args.execute:
            return
        deadline = time.time() + 5.0
        while self.gripper.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.gripper.get_num_connections() == 0:
            raise RuntimeError("No subscriber on {}".format(self.args.gripper_topic))
        command = make_gripper_command(
            position, self.args.gripper_speed, self.args.gripper_force
        )
        for _ in range(5):
            self.gripper.publish(command)
            rospy.sleep(0.05)
        rospy.sleep(1.0)

    def wait_for_grasp(self):
        if not self.args.execute or not self.args.require_object:
            return
        deadline = time.time() + 4.0
        last = None
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                last = rospy.wait_for_message(
                    self.args.gripper_status_topic,
                    Robotiq2FGripper_robot_input,
                    timeout=0.5,
                )
            except rospy.ROSException:
                continue
            rospy.loginfo(
                "Gripper status: gOBJ=%d gPR=%d gPO=%d gCU=%d",
                last.gOBJ,
                last.gPR,
                last.gPO,
                last.gCU,
            )
            if last.gOBJ == 2:
                return
        if last is None:
            raise RuntimeError("No gripper status on {}".format(self.args.gripper_status_topic))
        raise RuntimeError(
            "Gripper did not report object contact: gOBJ={} gPR={} gPO={} gCU={}".format(
                last.gOBJ, last.gPR, last.gPO, last.gCU
            )
        )

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.args.planning_frame,
            self.args.tool_link,
            rospy.Time(0),
            rospy.Duration(3.0),
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        xyz = pose_xyz(pose)
        quat = pose_quat(pose)
        tcp = tcp_from_tool(xyz, quat, self.args.tcp_offset)
        rospy.loginfo(
            "Current tool xyz=[%.3f, %.3f, %.3f], tcp xyz=[%.3f, %.3f, %.3f]",
            xyz[0],
            xyz[1],
            xyz[2],
            tcp[0],
            tcp[1],
            tcp[2],
        )
        return pose, xyz, quat, tcp

    def execute_cartesian_transfer(self, start_pose, start_xyz):
        lift_xyz = add(start_xyz, [0.0, 0.0, self.args.lift_dz])
        place_high_xyz = add(
            start_xyz, [self.args.place_dx, self.args.place_dy, self.args.lift_dz]
        )
        release_xyz = add(
            start_xyz, [self.args.place_dx, self.args.place_dy, self.args.release_dz]
        )
        retreat_xyz = add(
            start_xyz, [self.args.place_dx, self.args.place_dy, self.args.retreat_dz]
        )
        waypoints = [
            pose_copy_with_position(start_pose, lift_xyz),
            pose_copy_with_position(start_pose, place_high_xyz),
            pose_copy_with_position(start_pose, release_xyz),
        ]
        rospy.loginfo("Transfer waypoints tool frame:")
        for name, xyz in (
            ("lift", lift_xyz),
            ("place_high", place_high_xyz),
            ("release", release_xyz),
        ):
            rospy.loginfo("  %-10s [%.3f, %.3f, %.3f]", name, xyz[0], xyz[1], xyz[2])

        plan, fraction = self.arm.compute_cartesian_path(
            waypoints, self.args.eef_step, True
        )
        rospy.loginfo("Cartesian transfer fraction=%.3f", fraction)
        if fraction >= 0.95:
            if self.args.execute and not self.arm.execute(plan, wait=True):
                raise RuntimeError("Transfer execution failed.")
            self.arm.stop()
            self.arm.clear_pose_targets()
        else:
            rospy.logwarn(
                "Cartesian path fraction %.3f is low; falling back to waypoint-by-waypoint planning.",
                fraction,
            )
            self.move_to_pose(waypoints[0], "lift")
            self.move_to_pose(waypoints[1], "place_high")
            self.move_to_pose(waypoints[2], "release")
        return pose_copy_with_position(start_pose, retreat_xyz)

    def move_to_pose(self, pose, name):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, self.args.tool_link)
        plan = self.arm.plan()
        trajectory = plan[1] if isinstance(plan, tuple) else plan
        if not trajectory.joint_trajectory.points:
            self.arm.clear_pose_targets()
            raise RuntimeError("No plan for {}".format(name))
        rospy.loginfo("Planned %s with %d points", name, len(trajectory.joint_trajectory.points))
        if self.args.execute and not self.arm.execute(trajectory, wait=True):
            self.arm.clear_pose_targets()
            raise RuntimeError("Execution failed for {}".format(name))
        self.arm.stop()
        self.arm.clear_pose_targets()

    def home_pose(self):
        values = self.args.home_tcp
        return self.tcp_pose(values)

    def tcp_pose(self, values):
        quat = values[3:]
        if self.args.place_input_frame == "tool":
            tool_xyz = values[:3]
        else:
            tcp_xyz = values[:3]
            tool_xyz = tool_from_tcp(tcp_xyz, quat, self.args.tcp_offset)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = tool_xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

    def run(self):
        self.wait_for_robot_program()
        start_pose, start_xyz, _, start_tcp = self.current_pose()
        self.command_gripper(self.args.close_position, "close")
        self.wait_for_grasp()
        if self.args.pre_place_tcp is not None and self.args.place_tcp is not None:
            pre_place = self.tcp_pose(self.args.pre_place_tcp)
            place = self.tcp_pose(self.args.place_tcp)
            rospy.loginfo(
                "Using explicit pre_place TCP [%.3f, %.3f, %.3f]",
                self.args.pre_place_tcp[0],
                self.args.pre_place_tcp[1],
                self.args.pre_place_tcp[2],
            )
            rospy.loginfo(
                "Using explicit place TCP [%.3f, %.3f, %.3f]",
                self.args.place_tcp[0],
                self.args.place_tcp[1],
                self.args.place_tcp[2],
            )
            self.move_to_pose(pre_place, "pre_place")
            self.move_to_pose(place, "place")
        else:
            rospy.loginfo(
                "Place release TCP will be approximately [%.3f, %.3f, %.3f]",
                start_tcp[0] + self.args.place_dx,
                start_tcp[1] + self.args.place_dy,
                start_tcp[2] + self.args.release_dz,
            )
            self.execute_cartesian_transfer(start_pose, start_xyz)
        self.command_gripper(self.args.open_position, "open")
        self.move_to_pose(self.home_pose(), "home")


def main():
    args = parse_args()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("current_pose_grasp_place")
    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    try:
        CurrentPoseGraspPlace(args).run()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
