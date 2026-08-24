#!/usr/bin/python3

import math
import sys
import time

import moveit_commander
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from moveit_msgs.msg import RobotState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool


def gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = int(position)
    command.rSP = int(speed)
    command.rFR = int(force)
    return command


class SimplePickRedBlock:
    def __init__(self):
        self.execute = bool(rospy.get_param("~execute", False))
        self.test_only_pregrasp = bool(rospy.get_param("~test_only_pregrasp", True))
        self.target_topic = rospy.get_param("~target_topic", "/red_block/point_base")
        self.planning_frame = rospy.get_param("~planning_frame", "right_arm_base")
        self.arm_group = rospy.get_param("~arm_group", "right_arm")
        self.tcp_frame = rospy.get_param("~tcp_frame", "right_arm_tcp")
        self.requested_effector_frame = rospy.get_param(
            "~robot_effector_frame", self.tcp_frame
        )
        self.moveit_effector_frame = rospy.get_param(
            "~moveit_end_effector_link", self.requested_effector_frame
        )
        self.allow_tcp_offset_planning = bool(
            rospy.get_param("~allow_tcp_offset_planning", True)
        )
        self.tcp_frame = rospy.get_param("~tcp_frame", "right_arm_tcp")
        self.fallback_tcp_frame = rospy.get_param("~fallback_tcp_frame", "right_arm_tool0_controller")
        self.tcp_offset = [float(v) for v in rospy.get_param("~tcp_offset_from_effector", [0.0, 0.0, 0.155])]
        self.orientation = [float(v) for v in rospy.get_param("~orientation_quaternion", [-0.5, 0.5, 0.5, 0.5])]
        self.table_z = float(rospy.get_param("~table_z", 0.0))
        self.block_height = float(rospy.get_param("~block_height", 0.03))
        self.height_reference_mode = rospy.get_param("~height_reference_mode", "table")
        self.pre_grasp_height = float(rospy.get_param("~pre_grasp_height", 0.10))
        self.lift_height = float(rospy.get_param("~lift_height", 0.12))
        self.x_offset = float(rospy.get_param("~x_offset", 0.0))
        self.y_offset = float(rospy.get_param("~y_offset", 0.0))
        self.z_offset = float(rospy.get_param("~z_offset", 0.0))
        self.target_sample_count = int(rospy.get_param("~target_sample_count", 5))
        self.target_timeout = float(rospy.get_param("~target_timeout", 8.0))
        self.target_max_spread = float(rospy.get_param("~target_max_spread", 0.015))
        self.pause_before_descent = float(rospy.get_param("~pause_before_descent", 0.0))
        self.move_to_observe_first = bool(rospy.get_param("~move_to_observe_first", False))
        self.observe_pose_name = rospy.get_param("~observe_pose_name", "")
        self.observe_joint_values = rospy.get_param("~observe_joint_values", [])
        self.enable_second_view_correction = bool(
            rospy.get_param("~enable_second_view_correction", True)
        )

        self.arm = moveit_commander.MoveGroupCommander(self.arm_group)
        self.arm.set_pose_reference_frame(self.planning_frame)
        try:
            self.arm.set_end_effector_link(self.moveit_effector_frame)
        except Exception as exc:
            if (
                self.moveit_effector_frame == self.tcp_frame
                and self.allow_tcp_offset_planning
            ):
                rospy.logwarn(
                    "MoveIt model does not accept end effector link %s: %s. "
                    "Planning with right_arm_tool0 while converting TCP targets through TF.",
                    self.tcp_frame,
                    exc,
                )
                self.moveit_effector_frame = "right_arm_tool0"
                self.arm.set_end_effector_link(self.moveit_effector_frame)
            else:
                raise
        self.arm.set_max_velocity_scaling_factor(float(rospy.get_param("~velocity_scaling", 0.05)))
        self.arm.set_max_acceleration_scaling_factor(float(rospy.get_param("~acceleration_scaling", 0.05)))
        self.arm.set_planning_time(float(rospy.get_param("~planning_time", 10.0)))
        self.arm.set_num_planning_attempts(int(rospy.get_param("~num_planning_attempts", 10)))
        self.arm.set_goal_position_tolerance(float(rospy.get_param("~goal_position_tolerance", 0.01)))
        self.arm.set_goal_orientation_tolerance(float(rospy.get_param("~goal_orientation_tolerance", 0.08)))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.virtual_start_state = None
        self.gripper = rospy.Publisher(
            rospy.get_param("~gripper_topic", "/right_arm/Robotiq2FGripperRobotOutput"),
            Robotiq2FGripper_robot_output,
            queue_size=1,
        )

    @staticmethod
    def distance(first, second):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    def update_tcp_offset_from_tf(self):
        for frame in (self.tcp_frame, self.fallback_tcp_frame):
            if not frame:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.moveit_effector_frame,
                    frame,
                    rospy.Time(0),
                    rospy.Duration(1.0),
                ).transform
                self.tcp_offset = [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
                rospy.loginfo(
                    "Using TCP offset from TF %s -> %s: [%.4f, %.4f, %.4f]",
                    self.moveit_effector_frame,
                    frame,
                    self.tcp_offset[0],
                    self.tcp_offset[1],
                    self.tcp_offset[2],
                )
                return
            except tf2_ros.TransformException:
                continue
        rospy.logwarn(
            "No TF for %s -> %s or fallback %s. Using configured TCP offset %s",
            self.moveit_effector_frame,
            self.tcp_frame,
            self.fallback_tcp_frame,
            self.tcp_offset,
        )

    def move_to_observe_pose(self):
        if not self.move_to_observe_first:
            return
        self.arm.set_start_state_to_current_state()
        if self.observe_joint_values:
            active_joints = self.arm.get_active_joints()
            if len(self.observe_joint_values) != len(active_joints):
                raise RuntimeError(
                    "observe_joint_values has {} values, but group {} has {} active joints".format(
                        len(self.observe_joint_values), self.arm_group, len(active_joints)
                    )
                )
            self.arm.set_joint_value_target(
                dict(zip(active_joints, [float(v) for v in self.observe_joint_values]))
            )
            label = "observe_joint_values"
        elif self.observe_pose_name:
            self.arm.set_named_target(self.observe_pose_name)
            label = self.observe_pose_name
        else:
            raise RuntimeError(
                "move_to_observe_first is true but observe_joint_values/observe_pose_name is empty"
            )
        result = self.arm.plan()
        trajectory = result[1] if isinstance(result, tuple) else result
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("No MoveIt plan to observation pose {}".format(label))
        rospy.loginfo(
            "%s observation pose %s with %d trajectory points",
            "Executing" if self.execute else "Planned",
            label,
            len(trajectory.joint_trajectory.points),
        )
        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                self.arm.stop()
                raise RuntimeError("Execution failed moving to observation pose")
            self.arm.stop()
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)

    def wait_for_stable_target(self):
        deadline = time.time() + self.target_timeout
        samples = []
        while len(samples) < self.target_sample_count and time.time() < deadline:
            try:
                msg = rospy.wait_for_message(self.target_topic, PointStamped, timeout=1.0)
                if msg.header.frame_id != self.planning_frame:
                    transform = self.tf_buffer.lookup_transform(
                        self.planning_frame,
                        msg.header.frame_id,
                        rospy.Time(0),
                        rospy.Duration(0.5),
                    )
                    from tf2_geometry_msgs import do_transform_point

                    msg = do_transform_point(msg, transform)
                samples.append([msg.point.x, msg.point.y, msg.point.z])
            except (rospy.ROSException, tf2_ros.TransformException) as exc:
                rospy.logwarn_throttle(1.0, "Waiting for red block point: %s", exc)
        if len(samples) < self.target_sample_count:
            raise RuntimeError("No stable red block point on {}".format(self.target_topic))
        center = [sum(sample[i] for sample in samples) / len(samples) for i in range(3)]
        spread = max(self.distance(sample, center) for sample in samples)
        if spread > self.target_max_spread:
            raise RuntimeError(
                "Red block point is unstable: spread {:.3f}m > {:.3f}m".format(
                    spread, self.target_max_spread
                )
            )
        rospy.loginfo(
            "Stable red block point in %s: [%.3f, %.3f, %.3f], spread=%.3fm",
            self.planning_frame,
            center[0],
            center[1],
            center[2],
            spread,
        )
        return center

    def ensure_external_control(self):
        if not self.execute:
            return
        topic = rospy.get_param(
            "~robot_program_topic", "/right_arm/ur_hardware_interface/robot_program_running"
        )
        running = rospy.wait_for_message(topic, Bool, timeout=2.0)
        if not running.data:
            raise RuntimeError("External Control is not running on the right arm")

    def tcp_pose_as_effector_pose(self, tcp_xyz):
        matrix = tf.transformations.quaternion_matrix(self.orientation)
        rotated_offset = [
            matrix[row][0] * self.tcp_offset[0]
            + matrix[row][1] * self.tcp_offset[1]
            + matrix[row][2] * self.tcp_offset[2]
            for row in range(3)
        ]
        effector_xyz = [tcp_xyz[i] - rotated_offset[i] for i in range(3)]
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose = Pose()
        pose.pose.position.x = effector_xyz[0]
        pose.pose.position.y = effector_xyz[1]
        pose.pose.position.z = effector_xyz[2]
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = self.orientation
        return pose

    def end_state_from_trajectory(self, trajectory):
        state = RobotState()
        state.joint_state = self.arm.get_current_state().joint_state
        state.joint_state.name = list(state.joint_state.name)
        state.joint_state.position = list(state.joint_state.position)
        state.joint_state.velocity = list(state.joint_state.velocity)
        state.joint_state.effort = list(state.joint_state.effort)
        if not trajectory.joint_trajectory.points:
            return state

        final_point = trajectory.joint_trajectory.points[-1]
        for joint_name, position in zip(
            trajectory.joint_trajectory.joint_names, final_point.positions
        ):
            if joint_name in state.joint_state.name:
                index = state.joint_state.name.index(joint_name)
                state.joint_state.position[index] = position
                while len(state.joint_state.velocity) <= index:
                    state.joint_state.velocity.append(0.0)
                while len(state.joint_state.effort) <= index:
                    state.joint_state.effort.append(0.0)
            else:
                state.joint_state.name.append(joint_name)
                state.joint_state.position.append(position)
                state.joint_state.velocity.append(0.0)
                state.joint_state.effort.append(0.0)
        return state

    def plan_or_execute(self, name, tcp_xyz, cartesian=False):
        target = self.tcp_pose_as_effector_pose(tcp_xyz)
        if self.virtual_start_state is not None:
            self.arm.set_start_state(self.virtual_start_state)
        else:
            self.arm.set_start_state_to_current_state()
        if cartesian:
            trajectory, fraction = self.arm.compute_cartesian_path(
                [target.pose],
                float(rospy.get_param("~cartesian_step", 0.005)),
                avoid_collisions=True,
            )
            if fraction < float(rospy.get_param("~cartesian_min_fraction", 0.995)):
                raise RuntimeError("Cartesian path {} incomplete: {:.1%}".format(name, fraction))
        else:
            self.arm.set_pose_target(target)
            result = self.arm.plan()
            trajectory = result[1] if isinstance(result, tuple) else result
            self.arm.clear_pose_targets()
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("No MoveIt plan for {}".format(name))
        rospy.loginfo(
            "%s %s TCP=[%.3f, %.3f, %.3f] with %d trajectory points",
            "Executing" if self.execute else "Planned",
            name,
            tcp_xyz[0],
            tcp_xyz[1],
            tcp_xyz[2],
            len(trajectory.joint_trajectory.points),
        )
        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                self.arm.stop()
                raise RuntimeError("Execution failed at {}".format(name))
            self.arm.stop()
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

    def publish_gripper(self, position, label):
        rospy.loginfo("Gripper %s rPR=%s", label, position)
        if not self.execute:
            return
        deadline = time.time() + 5.0
        while self.gripper.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.1)
        if self.gripper.get_num_connections() == 0:
            raise RuntimeError("No subscriber on right gripper command topic")
        self.gripper.publish(
            gripper_command(
                position,
                rospy.get_param("~gripper_speed", 80),
                rospy.get_param("~gripper_force", 80),
            )
        )
        rospy.sleep(float(rospy.get_param("~gripper_settle_seconds", 1.0)))

    def run(self):
        self.update_tcp_offset_from_tf()
        self.move_to_observe_pose()
        target = self.wait_for_stable_target()
        x = target[0] + self.x_offset
        y = target[1] + self.y_offset
        if self.height_reference_mode == "detected":
            z_pre = target[2] + self.pre_grasp_height + self.z_offset
            z_grasp = target[2] + self.z_offset
            z_lift = target[2] + self.lift_height + self.z_offset
        elif self.height_reference_mode == "table":
            z_pre = self.table_z + self.block_height + self.pre_grasp_height + self.z_offset
            z_grasp = self.table_z + self.block_height * 0.5 + self.z_offset
            z_lift = self.table_z + self.block_height + self.lift_height + self.z_offset
        else:
            raise RuntimeError(
                "height_reference_mode must be 'detected' or 'table', got {}".format(
                    self.height_reference_mode
                )
            )
        pre = [x, y, z_pre]
        grasp = [x, y, z_grasp]
        lift = [x, y, z_lift]

        rospy.loginfo("Mode: %s", "EXECUTE" if self.execute else "PLAN ONLY")
        self.ensure_external_control()
        self.publish_gripper(rospy.get_param("~open_position", 0), "open")
        self.plan_or_execute("pre_grasp", pre, cartesian=False)
        if self.test_only_pregrasp:
            rospy.loginfo("Stopped after pre_grasp test. Re-run with test_only_pregrasp:=false for descent/grasp.")
            return
        if self.enable_second_view_correction:
            target = self.wait_for_stable_target()
            x = target[0] + self.x_offset
            y = target[1] + self.y_offset
            if self.height_reference_mode == "detected":
                z_grasp = target[2] + self.z_offset
                z_lift = target[2] + self.lift_height + self.z_offset
            grasp = [x, y, z_grasp]
            lift = [x, y, z_lift]
            rospy.loginfo(
                "Second-view correction updated TCP x/y to [%.3f, %.3f]", x, y
            )
        if self.pause_before_descent > 0.0:
            rospy.loginfo("Pausing %.1fs before descent for visual check.", self.pause_before_descent)
            rospy.sleep(self.pause_before_descent)
        self.plan_or_execute("grasp", grasp, cartesian=True)
        self.publish_gripper(rospy.get_param("~close_position", 210), "close")
        self.plan_or_execute("lift", lift, cartesian=True)


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_pick_red_block")
    try:
        SimplePickRedBlock().run()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
