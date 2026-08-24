#!/usr/bin/python3

import argparse
import math
import sys
import time

import moveit_commander
import rospy
import tf.transformations
import yaml
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import PositionIKRequest
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Header

try:
    from ur_msgs.srv import SetIO
    from ur_msgs.srv import SetIORequest
except ImportError:
    SetIO = None
    SetIORequest = None


def parse_args():
    parser = argparse.ArgumentParser(description="Plan or execute one guarded pick-and-place cycle.")
    parser.add_argument("--config", required=True, help="Task YAML file.")
    parser.add_argument("--execute", action="store_true", help="Allow commands to reach the real robot.")
    parser.add_argument("--yes", action="store_true", help="Skip the final typed confirmation.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def make_gripper_command(position, speed=80, force=80, activate=True):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1 if activate else 0
    command.rGTO = 1 if activate else 0
    command.rATR = 0
    command.rPR = position
    command.rSP = int(speed)
    command.rFR = int(force)
    return command


class SingleArmPickPlace:
    def __init__(self, config, execute):
        self.config = config
        self.execute = execute
        self.arm = moveit_commander.MoveGroupCommander(config["arm_group"])
        if config.get("planning_frame"):
            self.arm.set_pose_reference_frame(config["planning_frame"])
        if config.get("end_effector_link"):
            self.arm.set_end_effector_link(config["end_effector_link"])
        self.arm.set_max_velocity_scaling_factor(config["velocity_scaling"])
        self.arm.set_max_acceleration_scaling_factor(config["acceleration_scaling"])
        self.arm.set_planning_time(config["planning_time"])
        self.gripper_mode = config.get("gripper_mode", "robotiq_topic")
        self.gripper = None
        self.io_service = None
        self.io_tool_voltage = None
        if self.gripper_mode == "io":
            if SetIO is None or SetIORequest is None:
                raise RuntimeError("gripper_mode is io, but ur_msgs/SetIO is not available.")
            default_service = "/{}/ur_hardware_interface/set_io".format(config["arm_group"])
            self.io_service_name = config.get("io_service", default_service)
            self.io_fun = int(config.get("io_fun", SetIORequest.FUN_SET_DIGITAL_OUT))
            self.io_pin = int(config["io_pin"])
            self.io_open_state = float(config.get("io_open_state", 0.0))
            self.io_close_state = float(config.get("io_close_state", 1.0))
            self.io_tool_voltage = config.get("io_tool_voltage", None)
            self.io_service = rospy.ServiceProxy(self.io_service_name, SetIO)
        elif self.gripper_mode == "robotiq_topic":
            self.gripper = rospy.Publisher(
                config["gripper_topic"], Robotiq2FGripper_robot_output, queue_size=1
            )
        else:
            raise RuntimeError("Unsupported gripper_mode: {}".format(self.gripper_mode))
        self.display = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=1
        )
        q = tf.transformations.quaternion_from_euler(*config["orientation_rpy"])
        self.default_orientation = q
        self.pose_target_is_tcp = bool(config.get("pose_target_is_tcp", False))
        self.tcp_offset_from_end_effector = float_list(
            config.get("tcp_offset_from_end_effector", [0.0, 0.0, 0.0]),
            "tcp_offset_from_end_effector",
            3,
        )
        self.use_joint_targets_from_ik = bool(config.get("use_joint_targets_from_ik", False))
        self.ik_service = None
        self.plan_start_state = None
        self.max_joint_delta_before_execute = config.get("max_joint_delta_before_execute")
        self.max_wrist_joint_delta_before_execute = config.get(
            "max_wrist_joint_delta_before_execute"
        )
        self.min_end_effector_z_before_execute = config.get(
            "min_end_effector_z_before_execute"
        )
        self.fk_service = None
        if self.min_end_effector_z_before_execute is not None:
            self.fk_service = rospy.ServiceProxy("/compute_fk", GetPositionFK)
            rospy.wait_for_service("/compute_fk", timeout=5.0)
        if self.use_joint_targets_from_ik:
            self.ik_service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
            rospy.wait_for_service("/compute_ik", timeout=5.0)

    def pose(self, pose_values):
        pose = Pose()
        xyz = list(pose_values["translation"])
        if pose_values.get("quaternion") is not None:
            orientation = pose_values["quaternion"]
        elif pose_values.get("rpy_radian") is not None:
            orientation = tf.transformations.quaternion_from_euler(
                *pose_values["rpy_radian"]
            )
        else:
            orientation = self.default_orientation
        if self.pose_target_is_tcp:
            xyz = self.end_effector_position_from_tcp(xyz, orientation)
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        return pose

    def end_effector_position_from_tcp(self, tcp_xyz, orientation):
        rotation = tf.transformations.quaternion_matrix(orientation)
        offset = self.tcp_offset_from_end_effector
        rotated_offset = [
            rotation[row][0] * offset[0]
            + rotation[row][1] * offset[1]
            + rotation[row][2] * offset[2]
            for row in range(3)
        ]
        return [tcp_xyz[i] - rotated_offset[i] for i in range(3)]

    def move(self, name):
        target = self.pose(self.config["poses"][name])
        if self.plan_start_state is not None:
            self.arm.set_start_state(self.plan_start_state)
        else:
            self.arm.set_start_state_to_current_state()
        if self.use_joint_targets_from_ik:
            self.arm.set_joint_value_target(self.ik_joint_target(name, target))
        else:
            self.arm.set_pose_target(target)
        plan = self.arm.plan()
        trajectory = plan[1] if isinstance(plan, tuple) else plan
        points = trajectory.joint_trajectory.points
        if not points:
            self.arm.clear_pose_targets()
            raise RuntimeError("No valid MoveIt plan for pose: {}".format(name))
        rospy.loginfo("Planned %-10s with %d trajectory points", name, len(points))
        self.check_joint_delta(name, trajectory)
        self.check_trajectory_height(name, trajectory)
        display = DisplayTrajectory()
        display.trajectory_start = self.arm.get_current_state()
        display.trajectory.append(trajectory)
        self.display.publish(display)
        time.sleep(0.8)
        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                self.arm.clear_pose_targets()
                raise RuntimeError("Execution failed at pose: {}".format(name))
            self.arm.stop()
            self.plan_start_state = None
        else:
            self.plan_start_state = self.state_from_trajectory_end(trajectory)
        self.arm.clear_pose_targets()

    def ik_joint_target(self, name, target):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.config.get("planning_frame", self.arm.get_planning_frame())
        pose_stamped.pose = target
        request = PositionIKRequest()
        request.group_name = self.config["arm_group"]
        request.ik_link_name = self.config.get("end_effector_link", self.arm.get_end_effector_link())
        request.pose_stamped = pose_stamped
        request.robot_state = self.plan_start_state or self.arm.get_current_state()
        request.timeout = rospy.Duration(float(self.config.get("ik_timeout", 1.0)))
        request.avoid_collisions = bool(self.config.get("avoid_collisions", True))
        response = self.ik_service(request)
        if response.error_code.val != response.error_code.SUCCESS:
            raise RuntimeError(
                "No IK solution for pose: {} (error_code={})".format(
                    name, response.error_code.val
                )
            )
        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position
        solution = dict(zip(names, positions))
        reference = self.joint_reference_positions(request.robot_state)
        target = {}
        for joint in self.arm.get_active_joints():
            value = solution[joint]
            if joint in reference:
                value = self.nearest_equivalent_joint(value, reference[joint])
            target[joint] = value
        return target

    def joint_reference_positions(self, robot_state):
        return dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))

    def nearest_equivalent_joint(self, value, reference):
        adjusted = float(value)
        while adjusted - reference > math.pi:
            adjusted -= 2.0 * math.pi
        while adjusted - reference < -math.pi:
            adjusted += 2.0 * math.pi
        return adjusted

    def check_joint_delta(self, name, trajectory):
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if len(points) < 1:
            return
        start = points[0].positions
        end = points[-1].positions
        deltas = {
            joint: abs(float(end[index]) - float(start[index]))
            for index, joint in enumerate(names)
        }
        summary = ", ".join(
            "{}={:.3f}".format(joint.replace("{}_".format(self.config["arm_group"]), ""), delta)
            for joint, delta in deltas.items()
        )
        rospy.loginfo("Joint deltas for %s: %s", name, summary)
        if not self.execute:
            return

        max_joint = self.max_joint_delta_before_execute
        if max_joint is not None:
            offenders = [
                "{}={:.3f}".format(joint, delta)
                for joint, delta in deltas.items()
                if delta > float(max_joint)
            ]
            if offenders:
                raise RuntimeError(
                    "Refusing to execute {}: joint delta exceeds {:.3f} rad: {}".format(
                        name, float(max_joint), ", ".join(offenders)
                    )
                )

        max_wrist = self.max_wrist_joint_delta_before_execute
        if max_wrist is not None:
            offenders = [
                "{}={:.3f}".format(joint, delta)
                for joint, delta in deltas.items()
                if "wrist" in joint and delta > float(max_wrist)
            ]
            if offenders:
                raise RuntimeError(
                    "Refusing to execute {}: wrist delta exceeds {:.3f} rad: {}".format(
                        name, float(max_wrist), ", ".join(offenders)
                    )
                )

    def check_trajectory_height(self, name, trajectory):
        if self.min_end_effector_z_before_execute is None:
            return
        min_allowed = float(self.min_end_effector_z_before_execute)
        min_z = None
        min_index = None
        for index, point in enumerate(trajectory.joint_trajectory.points):
            pose_stamped = self.fk_for_trajectory_point(trajectory, point)
            z = pose_stamped.pose.position.z
            if min_z is None or z < min_z:
                min_z = z
                min_index = index
        if min_z is None:
            return
        rospy.loginfo(
            "Trajectory %-10s minimum %s z=%.3f at point %d",
            name,
            self.config.get("end_effector_link", self.arm.get_end_effector_link()),
            min_z,
            min_index,
        )
        if self.execute and min_z < min_allowed:
            raise RuntimeError(
                "Refusing to execute {}: planned {} z {:.3f} is below safety floor {:.3f}".format(
                    name,
                    self.config.get("end_effector_link", self.arm.get_end_effector_link()),
                    min_z,
                    min_allowed,
                )
            )

    def fk_for_trajectory_point(self, trajectory, point):
        state = self.arm.get_current_state()
        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)
        position_by_name = dict(zip(names, positions))
        for joint, position in zip(trajectory.joint_trajectory.joint_names, point.positions):
            position_by_name[joint] = position
        state.joint_state.name = names
        state.joint_state.position = [position_by_name[name] for name in names]
        state.joint_state.velocity = []
        state.joint_state.effort = []
        response = self.fk_service(
            Header(frame_id=self.config.get("planning_frame", self.arm.get_planning_frame())),
            [self.config.get("end_effector_link", self.arm.get_end_effector_link())],
            state,
        )
        if response.error_code.val != response.error_code.SUCCESS:
            raise RuntimeError("FK failed while checking trajectory height.")
        return response.pose_stamped[0]

    def state_from_trajectory_end(self, trajectory):
        state = self.arm.get_current_state()
        if not trajectory.joint_trajectory.points:
            return state
        names = trajectory.joint_trajectory.joint_names
        positions = trajectory.joint_trajectory.points[-1].positions
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        current.update(dict(zip(names, positions)))
        state.joint_state.name = list(current.keys())
        state.joint_state.position = list(current.values())
        state.joint_state.velocity = []
        state.joint_state.effort = []
        return state

    def setup_io_gripper(self):
        if self.gripper_mode != "io":
            return
        if self.io_tool_voltage is None:
            return
        if not self.execute:
            rospy.loginfo(
                "Plan-only: would set tool voltage to %s V", self.io_tool_voltage
            )
            return
        rospy.wait_for_service(self.io_service_name, timeout=5.0)
        response = self.io_service(
            SetIORequest.FUN_SET_TOOL_VOLTAGE, 0, float(self.io_tool_voltage)
        )
        if not response.success:
            raise RuntimeError("Failed to set tool voltage to {} V".format(
                self.io_tool_voltage
            ))

    def command_gripper(self, label):
        rospy.loginfo("Gripper command: %s via %s", label, self.gripper_mode)
        if self.gripper_mode == "io":
            state = self.io_close_state if label == "close" else self.io_open_state
            if self.execute:
                rospy.wait_for_service(self.io_service_name, timeout=5.0)
                response = self.io_service(self.io_fun, self.io_pin, state)
                if not response.success:
                    raise RuntimeError(
                        "SetIO failed: service={} fun={} pin={} state={}".format(
                            self.io_service_name, self.io_fun, self.io_pin, state
                        )
                    )
            else:
                rospy.loginfo(
                    "Plan-only: would call SetIO service=%s fun=%d pin=%d state=%.1f",
                    self.io_service_name,
                    self.io_fun,
                    self.io_pin,
                    state,
                )
        else:
            position = self.config.get(
                "close_position" if label == "close" else "open_position",
                255 if label == "close" else 0,
            )
            if self.execute:
                deadline = time.time() + 5.0
                while self.gripper.get_num_connections() == 0 and time.time() < deadline:
                    time.sleep(0.1)
                if self.gripper.get_num_connections() == 0:
                    raise RuntimeError("No subscriber connected to gripper topic: {}".format(
                        self.config["gripper_topic"]
                    ))
                speed = self.config.get("gripper_speed", 80)
                force = self.config.get("gripper_force", 80)
                self.gripper.publish(make_gripper_command(position, speed, force))
        time.sleep(self.config["gripper_settle_seconds"])

    def run(self):
        self.setup_io_gripper()
        self.command_gripper("open")
        for pose_name in ("home", "pre_grasp", "grasp"):
            self.move(pose_name)
        self.command_gripper("close")
        for pose_name in ("lift", "pre_place", "place"):
            self.move(pose_name)
        self.command_gripper("open")
        for pose_name in ("retreat", "home"):
            self.move(pose_name)


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Config file is empty or invalid: {}".format(path))
    required_poses = {"home", "pre_grasp", "grasp", "lift", "pre_place", "place", "retreat"}
    poses = config.get("poses")
    if not isinstance(poses, dict):
        raise ValueError(
            "poses must map pose names to pose records."
        )
    missing = required_poses - set(poses)
    if missing:
        raise ValueError("Missing poses: {}".format(", ".join(sorted(missing))))
    config["orientation_rpy"] = float_list(
        config.get("orientation_rpy"), "orientation_rpy", 3
    )
    for pose_name in sorted(required_poses):
        config["poses"][pose_name] = parse_pose_record(
            config["poses"][pose_name], "poses.{}".format(pose_name)
        )
    return config


def parse_pose_record(values, label):
    if isinstance(values, dict):
        record = {
            "translation": float_list(values.get("translation"), label + ".translation", 3),
            "quaternion": optional_float_list(
                values.get("quaternion"), label + ".quaternion", 4
            ),
            "rpy_radian": optional_float_list(
                values.get("rpy_radian", values.get("rpy")), label + ".rpy_radian", 3
            ),
            "rpy_degree": optional_float_list(
                values.get("rpy_degree"), label + ".rpy_degree", 3
            ),
        }
        if values.get("time") is not None:
            try:
                record["time"] = float(values["time"])
            except (TypeError, ValueError):
                raise ValueError("{} must be a number; found {!r}.".format(
                    label + ".time", values["time"]
                ))
        else:
            record["time"] = None
        return record

    if not isinstance(values, (list, tuple)) or len(values) not in (3, 6):
        raise ValueError(
            "{} must be a pose record, [x, y, z], or [x, y, z, roll, pitch, yaw].".format(
                label
            )
        )
    values = float_list(values, label, len(values))
    return {
        "translation": values[:3],
        "quaternion": None,
        "rpy_radian": values[3:] if len(values) == 6 else None,
        "rpy_degree": None,
        "time": None,
    }


def optional_float_list(values, label, expected_len):
    if values is None:
        return None
    return float_list(values, label, expected_len)


def float_list(values, label, expected_len):
    if not isinstance(values, (list, tuple)) or len(values) != expected_len:
        raise ValueError("{} must be a list of {} numbers.".format(label, expected_len))
    result = []
    for value in values:
        try:
            result.append(float(value))
        except (TypeError, ValueError):
            raise ValueError("{} must contain only numbers; found {!r}.".format(
                label, value
            ))
    return result


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError("Real execution is locked. Calibrate poses, then set enabled: true in the YAML file.")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("single_arm_pick_place")

    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, and RViz plans checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    SingleArmPickPlace(config, args.execute).run()
    rospy.loginfo("Pick-and-place cycle complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
