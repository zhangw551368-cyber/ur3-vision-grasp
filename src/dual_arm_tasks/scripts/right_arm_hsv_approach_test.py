#!/usr/bin/env python3

import math
import statistics
import sys
import inspect
from copy import deepcopy
from collections import deque

import moveit_commander
import rospy
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker, MarkerArray


class RightArmHsvApproachTest:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/hsv_grasp/object_point_base"
        )
        self.input_frame = rospy.get_param("~input_frame", "right_arm_base")
        self.planning_frame = rospy.get_param("~planning_frame", "base")
        self.group_name = rospy.get_param("~group_name", "right_arm")
        self.end_effector_link = rospy.get_param(
            "~end_effector_link", "right_arm_tool0"
        )
        self.planning_mode = rospy.get_param("~planning_mode", "position_only")
        if self.planning_mode not in ("position_only", "pose"):
            raise RuntimeError(
                "Unsupported planning_mode={!r}; use 'position_only' or 'pose'".format(
                    self.planning_mode
                )
            )
        self.z_offset_frame = rospy.get_param("~z_offset_frame", "planning_frame")
        if self.z_offset_frame not in ("planning_frame", "input_frame"):
            raise RuntimeError(
                "Unsupported z_offset_frame={!r}; use 'planning_frame' or 'input_frame'".format(
                    self.z_offset_frame
                )
            )
        self.grasp_strategy = rospy.get_param("~grasp_strategy", "side_x_neg")
        self.valid_grasp_strategies = (
            "top_down",
            "side_x_pos",
            "side_x_neg",
            "side_y_pos",
            "side_y_neg",
            "auto_any_face",
        )
        if self.grasp_strategy not in self.valid_grasp_strategies:
            raise RuntimeError(
                "Unsupported grasp_strategy={!r}; use one of {}".format(
                    self.grasp_strategy, ", ".join(self.valid_grasp_strategies)
                )
            )
        self.grasp_stage = rospy.get_param("~grasp_stage", "side_grasp_prepare")
        self.valid_grasp_stages = (
            "side_grasp_prepare",
            "side_approach",
            "full_side_path",
            "full_side_path_debug",
            "local_pick_place_preview",
            "top_down_pick_preview",
            "top_down_pick_place_preview",
            "full_pick_place_preview",
        )
        if self.grasp_stage not in self.valid_grasp_stages:
            raise RuntimeError(
                "Unsupported grasp_stage={!r}; use one of {}".format(
                    self.grasp_stage, ", ".join(self.valid_grasp_stages)
                )
            )
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.08))
        self.pregrasp_distance = float(rospy.get_param("~pregrasp_distance", 0.08))
        self.final_approach_distance = float(
            rospy.get_param("~final_approach_distance", 0.04)
        )
        self.object_point_semantic = rospy.get_param(
            "~object_point_semantic", "top_center"
        )
        self.valid_object_point_semantics = ("top_center", "cube_center", "center")
        if self.object_point_semantic not in self.valid_object_point_semantics:
            raise RuntimeError(
                "Unsupported object_point_semantic={!r}; use one of {}".format(
                    self.object_point_semantic,
                    ", ".join(self.valid_object_point_semantics),
                )
            )
        self.pregrasp_clearance = float(
            rospy.get_param("~pregrasp_clearance", 0.10)
        )
        self.final_clearance = float(rospy.get_param("~final_clearance", 0.08))
        self.grasp_clearance = float(rospy.get_param("~grasp_clearance", 0.025))
        self.lift_height = float(rospy.get_param("~lift_height", 0.10))
        self.local_lift_height = float(rospy.get_param("~local_lift_height", 0.06))
        self.place_offset_x = float(rospy.get_param("~place_offset_x", 0.0))
        self.place_offset_y = float(rospy.get_param("~place_offset_y", 0.20))
        self.place_offset_z = float(rospy.get_param("~place_offset_z", 0.0))
        self.retreat_distance = float(rospy.get_param("~retreat_distance", 0.08))
        self.local_retreat_distance = float(
            rospy.get_param("~local_retreat_distance", 0.04)
        )
        self.local_place_radius = float(rospy.get_param("~local_place_radius", 0.05))
        self.local_place_offset = float(rospy.get_param("~local_place_offset", 0.035))
        self.local_max_candidates = int(rospy.get_param("~local_max_candidates", 20))
        self.local_max_joint_motion = float(
            rospy.get_param("~local_max_joint_motion", 6.0)
        )
        self.top_down_hover_height = float(
            rospy.get_param("~top_down_hover_height", 0.20)
        )
        self.top_down_lift_height = float(
            rospy.get_param("~top_down_lift_height", 0.12)
        )
        self.top_down_place_offset_x = float(
            rospy.get_param("~top_down_place_offset_x", 0.0)
        )
        self.top_down_place_offset_y = float(
            rospy.get_param("~top_down_place_offset_y", 0.08)
        )
        self.top_down_place_max_distance = float(
            rospy.get_param("~top_down_place_max_distance", 0.10)
        )
        self.top_down_min_cartesian_fraction = float(
            rospy.get_param("~top_down_min_cartesian_fraction", 0.995)
        )
        self.top_down_max_joint_motion = float(
            rospy.get_param("~top_down_max_joint_motion", 6.0)
        )
        self.top_down_max_tilt_deg = float(
            rospy.get_param("~top_down_max_tilt_deg", 40.0)
        )
        self.top_down_min_tilt_deg = float(
            rospy.get_param("~top_down_min_tilt_deg", 0.0)
        )
        # Local +Y is the modeled Robotiq finger-opening axis.  Keeping this
        # axis nearly horizontal makes both fingertips descend at comparable
        # heights instead of placing one finger on the cube top first.
        self.top_down_max_finger_axis_vertical = float(
            rospy.get_param("~top_down_max_finger_axis_vertical", 1.0)
        )
        self.approach_height = float(rospy.get_param("~approach_height", 0.10))
        self.cube_size = float(rospy.get_param("~cube_size", 0.055))
        self.safe_min_z = float(rospy.get_param("~safe_min_z", 0.18))
        self.z_safety_epsilon = float(rospy.get_param("~z_safety_epsilon", 1e-4))
        self.snap_cube_to_support_plane_for_preview = self.get_bool_param(
            "snap_cube_to_support_plane_for_preview", False
        )
        self.use_raw_detected_object_z_for_preview = self.get_bool_param(
            "use_raw_detected_object_z_for_preview", True
        )
        self.support_plane_z = float(rospy.get_param("~support_plane_z", 0.0))
        self.cartesian_step = float(rospy.get_param("~cartesian_step", 0.005))
        self.jump_threshold = float(rospy.get_param("~jump_threshold", 0.0))
        self.min_cartesian_fraction = float(
            rospy.get_param("~min_cartesian_fraction", 0.95)
        )
        self.side_approach_cartesian_step = float(
            rospy.get_param("~side_approach_cartesian_step", self.cartesian_step)
        )
        self.side_approach_jump_threshold = float(
            rospy.get_param("~side_approach_jump_threshold", self.jump_threshold)
        )
        self.side_approach_min_fraction = float(
            rospy.get_param("~side_approach_min_fraction", self.min_cartesian_fraction)
        )
        self.execute_delay = float(rospy.get_param("~execute_delay", 0.0))
        self.object_to_grasp_offset_x = float(
            rospy.get_param("~object_to_grasp_offset_x", 0.0)
        )
        self.object_to_grasp_offset_y = float(
            rospy.get_param("~object_to_grasp_offset_y", 0.0)
        )
        self.object_to_grasp_offset_z = float(
            rospy.get_param("~object_to_grasp_offset_z", 0.0)
        )
        self.object_z_check_enabled = self.get_bool_param(
            "object_z_check_enabled", False
        )
        self.enable_z_clamp = self.get_bool_param("enable_z_clamp", True)
        self.clamp_grasp_points_for_preview = self.get_bool_param(
            "clamp_grasp_points_for_preview", False
        )
        self.clamp_grasp_points_for_execution = self.get_bool_param(
            "clamp_grasp_points_for_execution", True
        )
        self.allow_low_grasp_execution = self.get_bool_param(
            "allow_low_grasp_execution", False
        )
        self.object_min_z = float(rospy.get_param("~object_min_z", -10.0))
        self.object_max_z = float(rospy.get_param("~object_max_z", 10.0))
        self.auto_marker_topic = rospy.get_param(
            "~auto_marker_topic", "/hsv_grasp/grasp_debug_markers"
        )
        self.auto_marker_scale = float(rospy.get_param("~auto_marker_scale", 0.025))
        self.object_preview_marker_topic = rospy.get_param(
            "~object_preview_marker_topic",
            "/hsv_grasp/right_arm_debug_object_marker",
        )
        self.suggested_object_marker_enabled = self.get_bool_param(
            "suggested_object_marker_enabled", False
        )
        self.suggested_object_marker_topic = rospy.get_param(
            "~suggested_object_marker_topic", self.object_preview_marker_topic
        )
        self.suggested_object_use_selected_place = self.get_bool_param(
            "suggested_object_use_selected_place", True
        )
        self.suggested_object_verify_plan = self.get_bool_param(
            "suggested_object_verify_plan", True
        )
        self.suggested_object_max_candidates = int(
            rospy.get_param("~suggested_object_max_candidates", 4)
        )
        self.suggested_object_offset_x = float(
            rospy.get_param("~suggested_object_offset_x", 0.035)
        )
        self.suggested_object_offset_y = float(
            rospy.get_param("~suggested_object_offset_y", 0.0)
        )
        self.suggested_object_offset_z = float(
            rospy.get_param("~suggested_object_offset_z", 0.0)
        )
        self.tcp_offset_enabled = self.get_bool_param("tcp_offset_enabled", True)
        self.tool0_to_grasp_center_offset_x = float(
            rospy.get_param("~tool0_to_grasp_center_offset_x", 0.0)
        )
        self.tool0_to_grasp_center_offset_y = float(
            rospy.get_param("~tool0_to_grasp_center_offset_y", 0.0)
        )
        self.tool0_to_grasp_center_offset_z = float(
            rospy.get_param("~tool0_to_grasp_center_offset_z", 0.13)
        )
        self.filter_window = int(rospy.get_param("~filter_window", 7))
        self.filter_window = max(5, min(10, self.filter_window))
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.8))
        self.sample_reset_timeout = float(
            rospy.get_param("~sample_reset_timeout", 0.8)
        )
        self.max_abs_coordinate_m = float(rospy.get_param("~max_abs_coordinate_m", 10.0))
        self.tf_timeout = rospy.Duration(float(rospy.get_param("~tf_timeout", 0.5)))
        self.plan_period = float(rospy.get_param("~plan_period", 2.0))
        self.plan_once = bool(rospy.get_param("~plan_once", True))
        self.execute = bool(rospy.get_param("~execute", False))
        self.confirm = bool(rospy.get_param("~confirm", False))
        self.gripper_preview_enabled = self.get_bool_param(
            "gripper_preview_enabled", True
        )
        self.gripper_execute = self.get_bool_param("gripper_execute", False)
        self.allow_full_pick_place_preview = self.get_bool_param(
            "allow_full_pick_place_preview", False
        )
        self.gripper_open_position = float(
            rospy.get_param("~gripper_open_position", 0.085)
        )
        self.gripper_close_position = float(
            rospy.get_param("~gripper_close_position", 0.025)
        )
        self.attach_object_preview = self.get_bool_param(
            "attach_object_preview", False
        )
        self.orientation_mode = rospy.get_param("~orientation_mode", "auto_side")
        self.valid_orientation_modes = (
            "position_only",
            "auto_side",
            "fixed_side",
            "current",
            "fixed",
        )
        if self.orientation_mode not in self.valid_orientation_modes:
            raise RuntimeError(
                "Unsupported orientation_mode={!r}; use one of {}".format(
                    self.orientation_mode, ", ".join(self.valid_orientation_modes)
                )
            )
        self.fixed_orientation_quaternion = [
            float(value)
            for value in rospy.get_param(
                "~fixed_orientation_quaternion", [-0.5, 0.5, 0.5, 0.5]
            )
        ]
        self.fixed_side_orientation_quaternion = [
            float(value)
            for value in rospy.get_param(
                "~fixed_side_orientation_quaternion",
                self.fixed_orientation_quaternion,
            )
        ]
        self.robot_program_topic = rospy.get_param(
            "~robot_program_topic",
            "/right_arm/ur_hardware_interface/robot_program_running",
        )
        self.require_external_control_for_execute = bool(
            rospy.get_param("~require_external_control_for_execute", True)
        )
        self.apply_preview_stage_policy()
        self.validate_grasp_params()

        self.samples = deque(maxlen=self.filter_window)
        self.last_sample_time = rospy.Time(0)
        self.last_plan_time = rospy.Time(0)
        self.planned_once = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_pose_reference_frame(self.planning_frame)
        self.group.set_end_effector_link(self.end_effector_link)
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.10))
        self.acceleration_scaling = float(
            rospy.get_param("~acceleration_scaling", 0.10)
        )
        self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.group.set_planning_time(float(rospy.get_param("~planning_time", 10.0)))
        self.group.set_num_planning_attempts(
            int(rospy.get_param("~num_planning_attempts", 20))
        )
        self.group.set_goal_position_tolerance(
            float(rospy.get_param("~goal_position_tolerance", 0.02))
        )
        self.group.set_goal_orientation_tolerance(
            float(rospy.get_param("~goal_orientation_tolerance", 0.10))
        )
        planner_id = rospy.get_param("~planner_id", "")
        if planner_id:
            self.group.set_planner_id(planner_id)
        self.cartesian_path_service_name = rospy.get_param(
            "~cartesian_path_service", "/compute_cartesian_path"
        )
        self.cartesian_path_service = rospy.ServiceProxy(
            self.cartesian_path_service_name, GetCartesianPath
        )
        self.fk_service_name = rospy.get_param("~fk_service", "/compute_fk")
        self.fk_service = rospy.ServiceProxy(self.fk_service_name, GetPositionFK)

        self.display = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )
        self.auto_marker_publisher = rospy.Publisher(
            self.auto_marker_topic,
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.object_preview_marker_publisher = rospy.Publisher(
            self.object_preview_marker_topic,
            Marker,
            queue_size=10,
            latch=True,
        )
        self.suggested_object_marker_publisher = rospy.Publisher(
            self.suggested_object_marker_topic,
            Marker,
            queue_size=10,
            latch=True,
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointStamped, self.point_callback, queue_size=1
        )

        rospy.loginfo(
            "Right-arm HSV approach test started: input=%s frame=%s -> planning_frame=%s group=%s ee=%s planning_mode=%s z_offset_frame=%s grasp_stage=%s grasp_strategy=%s approach_distance=%.3f pregrasp_distance=%.3f final_approach_distance=%.3f approach_height=%.3f safe_min_z=%.3f object_z_check_enabled=%s execute=%s confirm=%s",
            self.input_topic,
            self.input_frame,
            self.planning_frame,
            self.group_name,
            self.end_effector_link,
            self.planning_mode,
            self.z_offset_frame,
            self.grasp_stage,
            self.grasp_strategy,
            self.approach_distance,
            self.pregrasp_distance,
            self.final_approach_distance,
            self.approach_height,
            self.safe_min_z,
            self.object_z_check_enabled,
            self.execute,
            self.confirm,
        )
        if self.execute and not self.confirm:
            rospy.logwarn(
                "execute is true but confirm is false; this run remains PLAN ONLY."
            )
        if self.gripper_execute and not (self.execute and self.confirm):
            rospy.logwarn(
                "gripper_execute is true, but execute and confirm are not both true; gripper remains PREVIEW ONLY."
            )
        if (
            self.planning_mode == "position_only"
            or self.orientation_mode == "position_only"
        ) and self.grasp_stage not in (
            "full_side_path_debug",
            "local_pick_place_preview",
            "top_down_pick_preview",
        ):
            rospy.logwarn(
                "position_only does not constrain gripper orientation; it cannot guarantee real side grasp."
            )
        if self.grasp_stage == "local_pick_place_preview":
            rospy.loginfo(
                "local_pick_place_preview locks Cartesian waypoint orientation from pre_grasp FK to reduce wrist rotation."
            )
        if self.grasp_stage in (
            "top_down_pick_preview",
            "top_down_pick_place_preview",
        ):
            rospy.loginfo(
                "%s points tool local +Z downward (with bounded outward tilt when strict vertical is unreachable).",
                self.grasp_stage,
            )
        if self.orientation_mode == "fixed_side":
            rospy.logwarn(
                "orientation_mode=fixed_side forces PoseStamped targets with fixed_side_orientation_quaternion; set_position_target will not be used for those targets."
            )
        if self.orientation_mode == "auto_side":
            rospy.loginfo(
                "orientation_mode=auto_side aligns the Robotiq local +Z axis with the side approach direction and keeps finger opening horizontal."
            )
        if self.tcp_offset_enabled:
            rospy.logwarn(
                "tcp_offset_enabled=true; tool0_to_grasp_center_offset will be applied before sending right_arm_tool0 targets to MoveIt."
            )
        else:
            rospy.loginfo(
                "tcp_offset_enabled=false; tool0_to_grasp_center_offset is only reported for future calibration."
            )
        rospy.logwarn(
            "right_arm_tool0 may not be the real gripper center; TCP offset may be required for real grasp."
        )
        rospy.loginfo(
            "tool0_to_grasp_center_offset: enabled=%s x=%.6f y=%.6f z=%.6f",
            self.tcp_offset_enabled,
            self.tool0_to_grasp_center_offset_x,
            self.tool0_to_grasp_center_offset_y,
            self.tool0_to_grasp_center_offset_z,
        )
        rospy.loginfo(
            "grasp z clamp policy: preview=%s execution=%s allow_low_grasp_execution=%s",
            self.clamp_grasp_points_for_preview,
            self.clamp_grasp_points_for_execution,
            self.allow_low_grasp_execution,
        )
        rospy.loginfo(
            "support plane preview: snap=%s support_plane_z=%.6f use_raw_detected_object_z_for_preview=%s",
            self.snap_cube_to_support_plane_for_preview,
            self.support_plane_z,
            self.use_raw_detected_object_z_for_preview,
        )
        rospy.loginfo(
            "runtime params: support_plane_z=%.6f snap_cube_to_support_plane_for_preview=%s clamp_grasp_points_for_preview=%s object_z_check_enabled=%s safe_min_z=%.6f cube_size=%.6f object_point_semantic=%s use_raw_detected_object_z_for_preview=%s object_preview_marker_topic=%s",
            self.support_plane_z,
            self.snap_cube_to_support_plane_for_preview,
            self.clamp_grasp_points_for_preview,
            self.object_z_check_enabled,
            self.safe_min_z,
            self.cube_size,
            self.object_point_semantic,
            self.use_raw_detected_object_z_for_preview,
            self.object_preview_marker_topic,
        )
        rospy.loginfo(
            "object preview marker topic: %s",
            self.object_preview_marker_topic,
        )
        rospy.loginfo(
            "suggested object marker: enabled=%s topic=%s use_selected_place=%s verify_plan=%s max_candidates=%d offset=(%.3f, %.3f, %.3f)",
            self.suggested_object_marker_enabled,
            self.suggested_object_marker_topic,
            self.suggested_object_use_selected_place,
            self.suggested_object_verify_plan,
            self.suggested_object_max_candidates,
            self.suggested_object_offset_x,
            self.suggested_object_offset_y,
            self.suggested_object_offset_z,
        )
        rospy.loginfo(
            "local place preview params: radius=%.3f offset=%.3f lift=%.3f retreat=%.3f max_candidates=%d max_joint_motion=%.3f",
            self.local_place_radius,
            self.local_place_offset,
            self.local_lift_height,
            self.local_retreat_distance,
            self.local_max_candidates,
            self.local_max_joint_motion,
        )
        rospy.loginfo(
            "fixed_side_orientation_quaternion: x=%.6f y=%.6f z=%.6f w=%.6f",
            self.fixed_side_orientation_quaternion[0],
            self.fixed_side_orientation_quaternion[1],
            self.fixed_side_orientation_quaternion[2],
            self.fixed_side_orientation_quaternion[3],
        )

    def is_valid_point(self, point):
        values = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in values):
            return False
        return all(abs(value) <= self.max_abs_coordinate_m for value in values)

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param("~{}".format(name), default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ("true", "1", "yes", "on"):
                return True
            if normalized in ("false", "0", "no", "off"):
                return False
        if isinstance(value, (int, float)):
            return bool(value)
        raise RuntimeError("Parameter ~{} must be a boolean".format(name))

    def apply_preview_stage_policy(self):
        if self.grasp_stage in (
            "full_side_path_debug",
            "local_pick_place_preview",
            "top_down_pick_preview",
            "top_down_pick_place_preview",
        ):
            if self.execute or self.confirm or self.gripper_execute:
                rospy.logwarn(
                    "%s is RViz/plan-only; forcing execute=false confirm=false gripper_execute=false.",
                    self.grasp_stage,
                )
            self.execute = False
            self.confirm = False
            self.gripper_execute = False

        if self.grasp_stage == "full_side_path_debug" and self.grasp_strategy != "side_x_neg":
            rospy.logwarn(
                "full_side_path_debug fixes grasp_strategy to side_x_neg; overriding %r.",
                self.grasp_strategy,
            )
            self.grasp_strategy = "side_x_neg"

        if self.grasp_stage in (
            "top_down_pick_preview",
            "top_down_pick_place_preview",
        ) and self.grasp_strategy != "top_down":
            rospy.logwarn(
                "%s fixes grasp_strategy to top_down; overriding %r.",
                self.grasp_stage,
                self.grasp_strategy,
            )
            self.grasp_strategy = "top_down"

        if (
            self.grasp_stage
            in (
                "full_side_path_debug",
                "local_pick_place_preview",
                "top_down_pick_preview",
                "top_down_pick_place_preview",
            )
            and self.object_point_semantic != "top_center"
        ):
            rospy.logwarn(
                "%s expects object_point_semantic=top_center; overriding %r.",
                self.grasp_stage,
                self.object_point_semantic,
            )
            self.object_point_semantic = "top_center"

    def validate_grasp_params(self):
        values = {
            "approach_distance": self.approach_distance,
            "pregrasp_distance": self.pregrasp_distance,
            "final_approach_distance": self.final_approach_distance,
            "pregrasp_clearance": self.pregrasp_clearance,
            "final_clearance": self.final_clearance,
            "grasp_clearance": self.grasp_clearance,
            "lift_height": self.lift_height,
            "local_lift_height": self.local_lift_height,
            "place_offset_x": self.place_offset_x,
            "place_offset_y": self.place_offset_y,
            "place_offset_z": self.place_offset_z,
            "retreat_distance": self.retreat_distance,
            "local_retreat_distance": self.local_retreat_distance,
            "local_place_radius": self.local_place_radius,
            "local_place_offset": self.local_place_offset,
            "local_max_candidates": float(self.local_max_candidates),
            "local_max_joint_motion": self.local_max_joint_motion,
            "top_down_hover_height": self.top_down_hover_height,
            "top_down_lift_height": self.top_down_lift_height,
            "top_down_place_offset_x": self.top_down_place_offset_x,
            "top_down_place_offset_y": self.top_down_place_offset_y,
            "top_down_place_max_distance": self.top_down_place_max_distance,
            "top_down_min_cartesian_fraction": self.top_down_min_cartesian_fraction,
            "top_down_max_joint_motion": self.top_down_max_joint_motion,
            "top_down_max_tilt_deg": self.top_down_max_tilt_deg,
            "suggested_object_max_candidates": float(
                self.suggested_object_max_candidates
            ),
            "approach_height": self.approach_height,
            "cube_size": self.cube_size,
            "safe_min_z": self.safe_min_z,
            "z_safety_epsilon": self.z_safety_epsilon,
            "support_plane_z": self.support_plane_z,
            "cartesian_step": self.cartesian_step,
            "jump_threshold": self.jump_threshold,
            "min_cartesian_fraction": self.min_cartesian_fraction,
            "side_approach_cartesian_step": self.side_approach_cartesian_step,
            "side_approach_jump_threshold": self.side_approach_jump_threshold,
            "side_approach_min_fraction": self.side_approach_min_fraction,
            "execute_delay": self.execute_delay,
            "object_to_grasp_offset_x": self.object_to_grasp_offset_x,
            "object_to_grasp_offset_y": self.object_to_grasp_offset_y,
            "object_to_grasp_offset_z": self.object_to_grasp_offset_z,
            "object_min_z": self.object_min_z,
            "object_max_z": self.object_max_z,
            "auto_marker_scale": self.auto_marker_scale,
            "tool0_to_grasp_center_offset_x": self.tool0_to_grasp_center_offset_x,
            "tool0_to_grasp_center_offset_y": self.tool0_to_grasp_center_offset_y,
            "tool0_to_grasp_center_offset_z": self.tool0_to_grasp_center_offset_z,
            "gripper_open_position": self.gripper_open_position,
            "gripper_close_position": self.gripper_close_position,
            "suggested_object_offset_x": self.suggested_object_offset_x,
            "suggested_object_offset_y": self.suggested_object_offset_y,
            "suggested_object_offset_z": self.suggested_object_offset_z,
            "max_input_age": self.max_input_age,
            "sample_reset_timeout": self.sample_reset_timeout,
        }
        for name, value in values.items():
            if not math.isfinite(value):
                raise RuntimeError("Parameter ~{} must be finite".format(name))
        if self.approach_distance < 0.0:
            raise RuntimeError("Parameter ~approach_distance must be >= 0")
        if self.pregrasp_distance < 0.0:
            raise RuntimeError("Parameter ~pregrasp_distance must be >= 0")
        if self.final_approach_distance < 0.0:
            raise RuntimeError("Parameter ~final_approach_distance must be >= 0")
        if self.pregrasp_clearance < 0.0:
            raise RuntimeError("Parameter ~pregrasp_clearance must be >= 0")
        if self.final_clearance < 0.0:
            raise RuntimeError("Parameter ~final_clearance must be >= 0")
        if self.grasp_clearance < 0.0:
            raise RuntimeError("Parameter ~grasp_clearance must be >= 0")
        if self.lift_height < 0.0:
            raise RuntimeError("Parameter ~lift_height must be >= 0")
        if self.local_lift_height < 0.0:
            raise RuntimeError("Parameter ~local_lift_height must be >= 0")
        if self.retreat_distance < 0.0:
            raise RuntimeError("Parameter ~retreat_distance must be >= 0")
        if self.local_retreat_distance < 0.0:
            raise RuntimeError("Parameter ~local_retreat_distance must be >= 0")
        if self.local_place_radius < 0.0:
            raise RuntimeError("Parameter ~local_place_radius must be >= 0")
        if self.local_place_radius > 0.05 + 1e-9:
            raise RuntimeError(
                "Parameter ~local_place_radius must be <= 0.05 for local_pick_place_preview"
            )
        if self.local_place_offset < 0.0:
            raise RuntimeError("Parameter ~local_place_offset must be >= 0")
        if self.local_max_candidates <= 0:
            raise RuntimeError("Parameter ~local_max_candidates must be > 0")
        if self.local_max_joint_motion <= 0.0:
            raise RuntimeError("Parameter ~local_max_joint_motion must be > 0")
        if self.top_down_hover_height <= 0.0:
            raise RuntimeError("Parameter ~top_down_hover_height must be > 0")
        if self.top_down_lift_height <= 0.0:
            raise RuntimeError("Parameter ~top_down_lift_height must be > 0")
        if self.top_down_lift_height > self.top_down_hover_height:
            raise RuntimeError(
                "Parameter ~top_down_lift_height must be <= ~top_down_hover_height"
            )
        if self.top_down_place_max_distance <= 0.0:
            raise RuntimeError("Parameter ~top_down_place_max_distance must be > 0")
        top_down_place_distance = math.hypot(
            self.top_down_place_offset_x, self.top_down_place_offset_y
        )
        if (
            self.grasp_stage == "top_down_pick_place_preview"
            and top_down_place_distance < 0.01
        ):
            raise RuntimeError(
                "top_down_pick_place_preview requires a place offset of at least 0.01 m"
            )
        if top_down_place_distance > self.top_down_place_max_distance + 1e-9:
            raise RuntimeError(
                "top-down place distance {:.3f} exceeds limit {:.3f}".format(
                    top_down_place_distance, self.top_down_place_max_distance
                )
            )
        if not 0.0 <= self.top_down_min_cartesian_fraction <= 1.0:
            raise RuntimeError(
                "Parameter ~top_down_min_cartesian_fraction must be in [0, 1]"
            )
        if self.top_down_max_joint_motion <= 0.0:
            raise RuntimeError("Parameter ~top_down_max_joint_motion must be > 0")
        if not 0.0 <= self.top_down_max_tilt_deg <= 45.0:
            raise RuntimeError("Parameter ~top_down_max_tilt_deg must be in [0, 45]")
        if not 0.0 <= self.top_down_min_tilt_deg <= self.top_down_max_tilt_deg:
            raise RuntimeError(
                "Parameter ~top_down_min_tilt_deg must be in [0, top_down_max_tilt_deg]"
            )
        if not 0.0 <= self.top_down_max_finger_axis_vertical <= 1.0:
            raise RuntimeError(
                "Parameter ~top_down_max_finger_axis_vertical must be in [0, 1]"
            )
        if self.suggested_object_max_candidates <= 0:
            raise RuntimeError("Parameter ~suggested_object_max_candidates must be > 0")
        if self.approach_height < 0.0:
            raise RuntimeError("Parameter ~approach_height must be >= 0")
        if self.cube_size <= 0.0:
            raise RuntimeError("Parameter ~cube_size must be > 0")
        if self.auto_marker_scale <= 0.0:
            raise RuntimeError("Parameter ~auto_marker_scale must be > 0")
        if self.cartesian_step <= 0.0:
            raise RuntimeError("Parameter ~cartesian_step must be > 0")
        if self.min_cartesian_fraction < 0.0 or self.min_cartesian_fraction > 1.0:
            raise RuntimeError("Parameter ~min_cartesian_fraction must be in [0, 1]")
        if self.side_approach_cartesian_step <= 0.0:
            raise RuntimeError("Parameter ~side_approach_cartesian_step must be > 0")
        if self.side_approach_min_fraction < 0.0 or self.side_approach_min_fraction > 1.0:
            raise RuntimeError("Parameter ~side_approach_min_fraction must be in [0, 1]")
        if self.execute_delay < 0.0:
            raise RuntimeError("Parameter ~execute_delay must be >= 0")
        if self.max_input_age <= 0.0:
            raise RuntimeError("Parameter ~max_input_age must be > 0")
        if self.sample_reset_timeout <= 0.0:
            raise RuntimeError("Parameter ~sample_reset_timeout must be > 0")
        if self.z_safety_epsilon < 0.0:
            raise RuntimeError("Parameter ~z_safety_epsilon must be >= 0")
        if self.object_min_z > self.object_max_z:
            raise RuntimeError("Parameter ~object_min_z must be <= ~object_max_z")
        if self.gripper_open_position < 0.0:
            raise RuntimeError("Parameter ~gripper_open_position must be >= 0")
        if self.gripper_close_position < 0.0:
            raise RuntimeError("Parameter ~gripper_close_position must be >= 0")
        if self.grasp_stage in (
            "side_approach",
            "full_side_path",
            "full_side_path_debug",
            "full_pick_place_preview",
        ) and self.grasp_strategy == "top_down":
            raise RuntimeError(
                "grasp_stage={!r} requires a side_x/side_y grasp_strategy".format(
                    self.grasp_stage
                )
            )
        if (
            self.grasp_stage == "full_side_path"
            and self.final_approach_distance > self.pregrasp_distance
            and self.grasp_strategy != "auto_any_face"
        ):
            raise RuntimeError(
                "Parameter ~final_approach_distance must be <= ~pregrasp_distance for full_side_path"
            )
        if (
            self.grasp_stage
            in (
                "full_side_path_debug",
                "local_pick_place_preview",
                "full_pick_place_preview",
            )
            and self.final_clearance > self.pregrasp_clearance
        ):
            raise RuntimeError(
                "Parameter ~final_clearance must be <= ~pregrasp_clearance for {}".format(
                    self.grasp_stage
                )
            )
        if (
            self.grasp_stage == "full_pick_place_preview"
            and not self.allow_full_pick_place_preview
        ):
            raise RuntimeError(
                "full_pick_place_preview is disabled until full_side_path_debug reaches Cartesian fraction >= 0.95. "
                "After RViz verification, set ~allow_full_pick_place_preview:=true explicitly."
            )
        if (
            self.grasp_strategy == "auto_any_face"
            and self.grasp_stage
            not in ("full_side_path", "full_pick_place_preview")
        ):
            raise RuntimeError(
                "grasp_strategy='auto_any_face' currently requires grasp_stage='full_side_path' or 'full_pick_place_preview'"
            )
        if self.grasp_strategy == "auto_any_face" and self.final_clearance > self.pregrasp_clearance:
            raise RuntimeError(
                "Parameter ~final_clearance must be <= ~pregrasp_clearance for auto_any_face"
            )
        if self.grasp_clearance > self.final_clearance:
            raise RuntimeError(
                "Parameter ~grasp_clearance must be <= ~final_clearance"
            )
        for name, values in (
            ("fixed_orientation_quaternion", self.fixed_orientation_quaternion),
            ("fixed_side_orientation_quaternion", self.fixed_side_orientation_quaternion),
        ):
            if len(values) != 4 or not all(math.isfinite(value) for value in values):
                raise RuntimeError("Parameter ~{} must contain four finite values".format(name))

    def point_callback(self, msg):
        if msg.header.frame_id.strip() != self.input_frame:
            rospy.logwarn_throttle(
                2.0,
                "Unexpected object frame_id=%r, expected %r; ignoring point",
                msg.header.frame_id,
                self.input_frame,
            )
            return
        if not self.is_valid_point(msg.point):
            rospy.logwarn_throttle(
                2.0,
                "Invalid object point: x=%.6f y=%.6f z=%.6f; ignoring point",
                msg.point.x,
                msg.point.y,
                msg.point.z,
            )
            return

        now = rospy.Time.now()
        if not msg.header.stamp.is_zero():
            input_age = (now - msg.header.stamp).to_sec()
            if input_age > self.max_input_age:
                rospy.logwarn_throttle(
                    1.0,
                    "Stale object point is %.3fs old (limit %.3fs); clearing samples and refusing to plan",
                    input_age,
                    self.max_input_age,
                )
                self.samples.clear()
                return
        if (
            not self.last_sample_time.is_zero()
            and (now - self.last_sample_time).to_sec() > self.sample_reset_timeout
        ):
            rospy.logwarn(
                "Live object stream had a %.3fs gap; clearing the old planning sample window",
                (now - self.last_sample_time).to_sec(),
            )
            self.samples.clear()
        self.last_sample_time = now

        self.samples.append((msg.point.x, msg.point.y, msg.point.z))
        if len(self.samples) < self.filter_window:
            rospy.loginfo_throttle(
                1.0,
                "Collecting HSV object samples: %d/%d",
                len(self.samples),
                self.filter_window,
            )
            return

        if self.plan_once and self.planned_once:
            return

        if (now - self.last_plan_time).to_sec() < self.plan_period:
            return
        self.last_plan_time = now

        try:
            self.plan_to_filtered_object()
            self.planned_once = True
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "HSV approach planning failed: %s; no motion executed",
                exc,
            )

    def median_object_point(self):
        xs = [sample[0] for sample in self.samples]
        ys = [sample[1] for sample in self.samples]
        zs = [sample[2] for sample in self.samples]
        return (
            float(statistics.median(xs)),
            float(statistics.median(ys)),
            float(statistics.median(zs)),
        )

    def transform_to_planning_frame(self, point):
        return self.tf_buffer.transform(
            point, self.planning_frame, timeout=self.tf_timeout
        )

    def make_object_point(self, object_xyz):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.input_frame
        point.point.x = object_xyz[0]
        point.point.y = object_xyz[1]
        point.point.z = object_xyz[2]
        return point

    def pre_grasp_in_planning_frame(self, object_xyz):
        object_input = self.make_object_point(object_xyz)
        object_planning = self.transform_to_planning_frame(object_input)

        grasp_target = self.make_grasp_target(object_planning)
        target_before_safety, stage_target = self.make_stage_target(grasp_target)
        return (
            object_input,
            object_planning,
            grasp_target,
            target_before_safety,
            stage_target,
        )

    def make_grasp_target(self, object_planning):
        grasp_target = PointStamped()
        grasp_target.header.stamp = rospy.Time.now()
        grasp_target.header.frame_id = self.planning_frame
        grasp_target.point.x = (
            object_planning.point.x + self.object_to_grasp_offset_x
        )
        grasp_target.point.y = (
            object_planning.point.y + self.object_to_grasp_offset_y
        )
        grasp_target.point.z = (
            object_planning.point.z + self.object_to_grasp_offset_z
        )
        return grasp_target

    @staticmethod
    def copy_point_stamped(point):
        copied = PointStamped()
        copied.header.stamp = point.header.stamp
        copied.header.frame_id = point.header.frame_id
        copied.point.x = point.point.x
        copied.point.y = point.point.y
        copied.point.z = point.point.z
        return copied

    def side_offset_direction(self):
        if self.grasp_strategy == "side_x_pos":
            return (1.0, 0.0, 0.0)
        if self.grasp_strategy == "side_x_neg":
            return (-1.0, 0.0, 0.0)
        if self.grasp_strategy == "side_y_pos":
            return (0.0, 1.0, 0.0)
        if self.grasp_strategy == "side_y_neg":
            return (0.0, -1.0, 0.0)
        raise RuntimeError("grasp_strategy={!r} is not a side strategy".format(self.grasp_strategy))

    @staticmethod
    def auto_candidate_directions():
        return (
            ("side_x_neg", (-1.0, 0.0, 0.0)),
            ("side_x_pos", (1.0, 0.0, 0.0)),
            ("side_y_neg", (0.0, -1.0, 0.0)),
            ("side_y_pos", (0.0, 1.0, 0.0)),
        )

    def approach_motion_direction(self):
        direction = self.side_offset_direction()
        return (-direction[0], -direction[1], -direction[2])

    @staticmethod
    def normalize_vector(vector, label):
        norm = math.sqrt(sum(component * component for component in vector))
        if norm <= 1e-9:
            raise RuntimeError("{} has near-zero length".format(label))
        return tuple(component / norm for component in vector)

    @staticmethod
    def cross_vectors(first, second):
        return (
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        )

    @staticmethod
    def dot_vectors(first, second):
        return sum(a * b for a, b in zip(first, second))

    @staticmethod
    def vector_norm(vector):
        return math.sqrt(sum(component * component for component in vector))

    @staticmethod
    def quaternion_from_axes(x_axis, y_axis, z_axis):
        # Rotation matrix columns are the local axes expressed in the planning frame.
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (m21 - m12) / scale
            qy = (m02 - m20) / scale
            qz = (m10 - m01) / scale
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / scale
            qx = 0.25 * scale
            qy = (m01 + m10) / scale
            qz = (m02 + m20) / scale
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / scale
            qx = (m01 + m10) / scale
            qy = 0.25 * scale
            qz = (m12 + m21) / scale
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / scale
            qx = (m02 + m20) / scale
            qy = (m12 + m21) / scale
            qz = 0.25 * scale

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            raise RuntimeError("auto side orientation produced a zero quaternion")
        return Quaternion(qx / norm, qy / norm, qz / norm, qw / norm)

    def side_grasp_orientation(self, direction):
        z_axis = self.normalize_vector(
            (-direction[0], -direction[1], -direction[2]),
            "side grasp approach axis",
        )
        world_up = (0.0, 0.0, 1.0)
        y_axis = self.normalize_vector(
            self.cross_vectors(world_up, z_axis),
            "side grasp horizontal finger axis",
        )
        x_axis = self.normalize_vector(
            self.cross_vectors(y_axis, z_axis),
            "side grasp remaining tool axis",
        )
        return self.quaternion_from_axes(x_axis, y_axis, z_axis)

    def cube_center_from_object_point(self, object_base):
        cube_center = self.copy_point_stamped(object_base)
        if self.object_point_semantic == "top_center":
            cube_center.point.z -= self.cube_size / 2.0
        elif self.object_point_semantic in ("center", "cube_center"):
            pass
        else:
            raise RuntimeError(
                "Unknown object_point_semantic={!r}".format(
                    self.object_point_semantic
                )
            )
        return cube_center

    def stage_cube_center(self, object_base):
        raw_cube_center = self.cube_center_from_object_point(object_base)
        cube_center = self.copy_point_stamped(raw_cube_center)
        raw_cube_center_z = raw_cube_center.point.z
        snapped_cube_center_z = self.support_plane_z + self.cube_size / 2.0

        snap_to_support_plane = (
            self.grasp_stage == "local_pick_place_preview"
            and self.snap_cube_to_support_plane_for_preview
            and not self.real_execution_requested()
        )

        if self.use_raw_detected_object_z_for_preview:
            final_cube_center_z = raw_cube_center_z
        elif snap_to_support_plane:
            final_cube_center_z = snapped_cube_center_z
            rospy.logwarn(
                "local preview cube_center z snapped from %.6f to %.6f using support_plane_z=%.6f because use_raw_detected_object_z_for_preview=false.",
                raw_cube_center_z,
                final_cube_center_z,
                self.support_plane_z,
            )
        else:
            final_cube_center_z = raw_cube_center_z

        cube_center.point.z = final_cube_center_z
        z_delta = final_cube_center_z - raw_cube_center_z

        rospy.loginfo(
            "cube_center_z decision: object_in_base.z=%.6f object_point_semantic=%s cube_size=%.6f raw_cube_center_z=%.6f support_plane_z=%.6f snap_cube_to_support_plane_for_preview=%s use_raw_detected_object_z_for_preview=%s final_cube_center_z=%.6f final_minus_raw=%.6f",
            object_base.point.z,
            self.object_point_semantic,
            self.cube_size,
            raw_cube_center_z,
            self.support_plane_z,
            self.snap_cube_to_support_plane_for_preview,
            self.use_raw_detected_object_z_for_preview,
            final_cube_center_z,
            z_delta,
        )
        if (
            self.use_raw_detected_object_z_for_preview
            and abs(z_delta) > 0.005
        ):
            rospy.logerr(
                "BUG: use_raw_detected_object_z_for_preview=true but final_cube_center_z=%.6f differs from raw_cube_center_z=%.6f by %.6f; refusing to continue planning.",
                final_cube_center_z,
                raw_cube_center_z,
                z_delta,
            )
            return None
        if (
            not self.use_raw_detected_object_z_for_preview
            and not self.snap_cube_to_support_plane_for_preview
            and abs(final_cube_center_z - snapped_cube_center_z) < 1e-9
            and abs(final_cube_center_z - raw_cube_center_z) > 1e-9
        ):
            rospy.logerr(
                "BUG: snap_cube_to_support_plane_for_preview=false but final_cube_center_z=%.6f equals support_plane_z + cube_size/2=%.6f instead of raw_cube_center_z=%.6f.",
                final_cube_center_z,
                snapped_cube_center_z,
                raw_cube_center_z,
            )
            return None
        if abs(z_delta) > 0.05:
            rospy.logwarn(
                "cube_center z changed significantly: raw_cube_center_z=%.6f final_cube_center_z=%.6f delta=%.6f. Z was clearly modified; real execution is not recommended until this is verified.",
                raw_cube_center_z,
                final_cube_center_z,
                z_delta,
            )
        return cube_center

    def point_with_direction_offset(self, source, direction, distance):
        target = self.copy_point_stamped(source)
        target.point.x += direction[0] * distance
        target.point.y += direction[1] * distance
        target.point.z += direction[2] * distance
        return target

    def point_with_xyz_offset(self, source, offset_x, offset_y, offset_z):
        target = self.copy_point_stamped(source)
        target.point.x += offset_x
        target.point.y += offset_y
        target.point.z += offset_z
        return target

    def real_execution_requested(self):
        return self.execute and self.confirm and self.gripper_execute

    def should_clamp_grasp_points(self):
        if self.real_execution_requested():
            return (
                self.clamp_grasp_points_for_execution
                and not self.allow_low_grasp_execution
            )
        return self.clamp_grasp_points_for_preview

    def clamped_tcp_target_z(self, point, label, strategy, emit_warning=True):
        before = self.copy_point_stamped(point)
        target = self.copy_point_stamped(point)
        clamp_applied = False
        if target.point.z < self.safe_min_z:
            clamp_applied = True
            if self.enable_z_clamp:
                if emit_warning:
                    rospy.logwarn(
                        "%s %s z=%.6f is below safe_min_z=%.6f; %s was raised by safe_min_z; this may prevent real grasp.",
                        strategy,
                        label,
                        target.point.z,
                        self.safe_min_z,
                        label,
                    )
                target.point.z = self.safe_min_z
            else:
                if emit_warning:
                    rospy.logwarn(
                        "%s %s z=%.6f is below safe_min_z=%.6f and enable_z_clamp=false; target will be rejected.",
                        strategy,
                        label,
                        target.point.z,
                        self.safe_min_z,
                    )
        return before, target, clamp_applied

    def clamp_tcp_target_z(self, point, label, strategy):
        before, target, _ = self.clamped_tcp_target_z(
            point, label, strategy, emit_warning=True
        )
        return before, target

    def candidate_target_from_raw(self, raw_point, label, strategy):
        raw = self.copy_point_stamped(raw_point)
        _, clamped, clamp_applied = self.clamped_tcp_target_z(
            raw, label, strategy, emit_warning=False
        )
        use_clamped = self.should_clamp_grasp_points()
        target = self.copy_point_stamped(clamped if use_clamped else raw)
        if clamp_applied:
            if use_clamped:
                rospy.logwarn(
                    "%s %s raw z=%.6f is below safe_min_z=%.6f; using clamped execution/preview target z=%.6f.",
                    strategy,
                    label,
                    raw.point.z,
                    self.safe_min_z,
                    target.point.z,
                )
            else:
                rospy.logwarn(
                    "%s raw %s z=%.6f is below safe_min_z=%.6f; preview geometry is NOT clamped because clamp_grasp_points_for_preview=false.",
                    strategy,
                    label,
                    raw.point.z,
                    self.safe_min_z,
                )
        return raw, clamped, target, clamp_applied

    def tcp_target_z_is_safe(self, point, label, strategy):
        threshold = self.safe_min_z - self.z_safety_epsilon
        if point.point.z < threshold:
            if self.real_execution_requested() and not self.allow_low_grasp_execution:
                rospy.logwarn(
                    "%s %s z=%.6f is below safe_min_z=%.6f (epsilon=%.6f); rejecting unsafe execution target. Set allow_low_grasp_execution:=true only after confirming this is safe.",
                    strategy,
                    label,
                    point.point.z,
                    self.safe_min_z,
                    self.z_safety_epsilon,
                )
                return False
            rospy.logwarn(
                "%s %s z=%.6f is below safe_min_z=%.6f (epsilon=%.6f); allowing plan-only preview target.",
                strategy,
                label,
                point.point.z,
                self.safe_min_z,
                self.z_safety_epsilon,
            )
            return True

        if self.tcp_offset_enabled and self.real_execution_requested():
            tool0_target = self.tool0_target_from_grasp_center(
                point, self.target_orientation()
            )
            if tool0_target.point.z < threshold:
                if self.allow_low_grasp_execution:
                    rospy.logwarn(
                        "%s %s right_arm_tool0 target z=%.6f is below safe_min_z=%.6f (epsilon=%.6f) after TCP offset compensation; allow_low_grasp_execution=true, allowing execution target.",
                        strategy,
                        label,
                        tool0_target.point.z,
                        self.safe_min_z,
                        self.z_safety_epsilon,
                    )
                else:
                    rospy.logwarn(
                        "%s %s right_arm_tool0 target z=%.6f is below safe_min_z=%.6f (epsilon=%.6f) after TCP offset compensation; rejecting unsafe TCP target.",
                        strategy,
                        label,
                        tool0_target.point.z,
                        self.safe_min_z,
                        self.z_safety_epsilon,
                    )
                    return False
        return True

    def make_auto_candidate(self, cube_center, strategy, direction):
        grasp_target = self.make_grasp_target(cube_center)
        side_face_center = self.point_with_direction_offset(
            cube_center, direction, self.cube_size / 2.0
        )
        pre_grasp_raw = self.point_with_direction_offset(
            grasp_target,
            direction,
            self.cube_size / 2.0 + self.pregrasp_clearance,
        )
        approach_raw = self.point_with_direction_offset(
            grasp_target,
            direction,
            self.cube_size / 2.0 + self.final_clearance,
        )
        grasp_raw = self.point_with_direction_offset(
            grasp_target,
            direction,
            self.cube_size / 2.0 + self.grasp_clearance,
        )

        pre_raw, pre_clamped, pre_grasp, pre_clamp_applied = (
            self.candidate_target_from_raw(pre_grasp_raw, "pre_grasp", strategy)
        )
        approach_raw, approach_clamped, approach_point, approach_clamp_applied = (
            self.candidate_target_from_raw(
                approach_raw, "approach_point", strategy
            )
        )
        grasp_raw, grasp_clamped, grasp_point, grasp_clamp_applied = (
            self.candidate_target_from_raw(grasp_raw, "grasp_point", strategy)
        )
        return {
            "strategy": strategy,
            "direction": direction,
            "cube_center": self.copy_point_stamped(cube_center),
            "grasp_target": grasp_target,
            "side_face_center": side_face_center,
            "pre_grasp_before": pre_raw,
            "pre_grasp_raw": pre_raw,
            "pre_grasp_clamped": pre_clamped,
            "pre_grasp_clamp_applied": pre_clamp_applied,
            "pre_grasp": pre_grasp,
            "approach_before": approach_raw,
            "approach_point_raw": approach_raw,
            "approach_point_clamped": approach_clamped,
            "approach_point_clamp_applied": approach_clamp_applied,
            "approach_point": approach_point,
            "grasp_before": grasp_raw,
            "grasp_point_raw": grasp_raw,
            "grasp_point_clamped": grasp_clamped,
            "grasp_point_clamp_applied": grasp_clamp_applied,
            "grasp_point": grasp_point,
        }

    def make_auto_candidates(self, cube_center):
        return [
            self.make_auto_candidate(cube_center, strategy, direction)
            for strategy, direction in self.auto_candidate_directions()
        ]

    def make_pick_place_candidate(self, cube_center, strategy, direction):
        candidate = self.make_auto_candidate(cube_center, strategy, direction)

        lift_raw = self.point_with_xyz_offset(
            candidate["grasp_point_raw"], 0.0, 0.0, self.lift_height
        )
        lift_raw, lift_clamped, lift_point, lift_clamp_applied = (
            self.candidate_target_from_raw(lift_raw, "lift_point", strategy)
        )

        place_raw = self.point_with_xyz_offset(
            lift_raw,
            self.place_offset_x,
            self.place_offset_y,
            self.place_offset_z,
        )
        place_raw, place_clamped, place_point, place_clamp_applied = (
            self.candidate_target_from_raw(place_raw, "place_point", strategy)
        )

        retreat_raw = self.point_with_direction_offset(
            place_raw, direction, self.retreat_distance
        )
        retreat_raw, retreat_clamped, retreat_point, retreat_clamp_applied = (
            self.candidate_target_from_raw(retreat_raw, "retreat_point", strategy)
        )

        candidate.update(
            {
                "lift_before": lift_raw,
                "lift_point_raw": lift_raw,
                "lift_point_clamped": lift_clamped,
                "lift_point_clamp_applied": lift_clamp_applied,
                "lift_point": lift_point,
                "place_before": place_raw,
                "place_point_raw": place_raw,
                "place_point_clamped": place_clamped,
                "place_point_clamp_applied": place_clamp_applied,
                "place_point": place_point,
                "retreat_before": retreat_raw,
                "retreat_point_raw": retreat_raw,
                "retreat_point_clamped": retreat_clamped,
                "retreat_point_clamp_applied": retreat_clamp_applied,
                "retreat_point": retreat_point,
            }
        )
        return candidate

    def make_pick_place_candidates(self, cube_center):
        return [
            self.make_pick_place_candidate(cube_center, strategy, direction)
            for strategy, direction in self.auto_candidate_directions()
        ]

    def local_place_offsets(self):
        radius = min(self.local_place_radius, 0.05)
        offset = min(self.local_place_offset, radius)
        offsets = []
        if offset > 1e-6:
            offsets.extend(
                [
                    (offset, 0.0),
                    (0.0, offset),
                    (-offset, 0.0),
                    (0.0, -offset),
                ]
            )
        offsets.append((0.0, 0.0))
        if offset > 1e-6:
            diagonal = min(offset, radius / math.sqrt(2.0))
            if diagonal > 1e-6:
                offsets.extend(
                    [
                        (diagonal, diagonal),
                        (-diagonal, diagonal),
                        (diagonal, -diagonal),
                        (-diagonal, -diagonal),
                    ]
                )

        unique_offsets = []
        seen = set()
        for dx, dy in offsets:
            if math.hypot(dx, dy) > radius + 1e-9:
                continue
            key = (round(dx, 6), round(dy, 6))
            if key in seen:
                continue
            seen.add(key)
            unique_offsets.append((dx, dy))
        return unique_offsets

    def make_local_pick_place_candidate(
        self, cube_center, strategy, direction, place_offset_xy
    ):
        candidate = self.make_auto_candidate(cube_center, strategy, direction)
        place_dx, place_dy = place_offset_xy
        place_distance = math.hypot(place_dx, place_dy)

        place_cube_center = self.point_with_xyz_offset(
            cube_center, place_dx, place_dy, 0.0
        )
        place_side_face_center = self.point_with_direction_offset(
            place_cube_center, direction, self.cube_size / 2.0
        )

        # pre_grasp and approach_point intentionally remain outside the selected
        # cube face.  At the close/place waypoints, however, the configured TCP
        # is the center between the fingers, so it must coincide with the cube
        # center rather than stop at the face plus an extra clearance.
        grasp_raw = self.copy_point_stamped(candidate["grasp_target"])
        lift_raw = self.point_with_xyz_offset(
            grasp_raw, 0.0, 0.0, self.local_lift_height
        )
        place_lift_raw = self.point_with_xyz_offset(
            place_cube_center,
            0.0,
            0.0,
            self.local_lift_height,
        )
        place_raw = self.copy_point_stamped(place_cube_center)
        retreat_raw = self.point_with_direction_offset(
            place_raw,
            direction,
            self.cube_size / 2.0 + self.local_retreat_distance,
        )

        grasp_raw, grasp_clamped, grasp_point, grasp_clamp_applied = (
            self.candidate_target_from_raw(grasp_raw, "grasp_point", strategy)
        )
        lift_raw, lift_clamped, lift_point, lift_clamp_applied = (
            self.candidate_target_from_raw(lift_raw, "lift_point", strategy)
        )
        place_lift_raw, place_lift_clamped, place_lift_point, place_lift_clamp_applied = (
            self.candidate_target_from_raw(
                place_lift_raw, "place_lift_point", strategy
            )
        )
        place_raw, place_clamped, place_point, place_clamp_applied = (
            self.candidate_target_from_raw(place_raw, "place_point", strategy)
        )
        retreat_raw, retreat_clamped, retreat_point, retreat_clamp_applied = (
            self.candidate_target_from_raw(retreat_raw, "retreat_point", strategy)
        )

        candidate.update(
            {
                "candidate_id": "{}_{:+.3f}_{:+.3f}".format(
                    strategy,
                    place_dx,
                    place_dy,
                ),
                "place_offset_xy": (place_dx, place_dy),
                "place_distance": place_distance,
                "place_cube_center": place_cube_center,
                "place_side_face_center": place_side_face_center,
                "grasp_before": grasp_raw,
                "grasp_point_raw": grasp_raw,
                "grasp_point_clamped": grasp_clamped,
                "grasp_point_clamp_applied": grasp_clamp_applied,
                "grasp_point": grasp_point,
                "lift_before": lift_raw,
                "lift_point_raw": lift_raw,
                "lift_point_clamped": lift_clamped,
                "lift_point_clamp_applied": lift_clamp_applied,
                "lift_point": lift_point,
                "place_lift_before": place_lift_raw,
                "place_lift_point_raw": place_lift_raw,
                "place_lift_point_clamped": place_lift_clamped,
                "place_lift_point_clamp_applied": place_lift_clamp_applied,
                "place_lift_point": place_lift_point,
                "place_before": place_raw,
                "place_point_raw": place_raw,
                "place_point_clamped": place_clamped,
                "place_point_clamp_applied": place_clamp_applied,
                "place_point": place_point,
                "retreat_before": retreat_raw,
                "retreat_point_raw": retreat_raw,
                "retreat_point_clamped": retreat_clamped,
                "retreat_point_clamp_applied": retreat_clamp_applied,
                "retreat_point": retreat_point,
            }
        )
        return candidate

    def make_local_pick_place_candidates(self, cube_center):
        candidates = []
        for place_offset_xy in self.local_place_offsets():
            for strategy, direction in self.auto_candidate_directions():
                candidates.append(
                    self.make_local_pick_place_candidate(
                        cube_center, strategy, direction, place_offset_xy
                    )
                )
                if len(candidates) >= self.local_max_candidates:
                    return candidates
        return candidates

    def candidate_tcp_targets_are_safe(self, candidate, keys):
        strategy = candidate["strategy"]
        for key in keys:
            if not self.tcp_target_z_is_safe(candidate[key], key, strategy):
                return False
        return True

    def make_stage_target(self, grasp_target):
        target = PointStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = self.planning_frame
        target.point.x = grasp_target.point.x
        target.point.y = grasp_target.point.y
        target.point.z = grasp_target.point.z

        if self.grasp_strategy == "top_down":
            target.point.z += self.approach_height
        else:
            direction = self.side_offset_direction()
            target.point.x += direction[0] * self.approach_distance
            target.point.y += direction[1] * self.approach_distance
            target.point.z += direction[2] * self.approach_distance

        return self.clamp_tcp_target_z(target, "target", self.grasp_stage)

    def make_side_target(self, grasp_target, distance, label):
        target = self.copy_point_stamped(grasp_target)
        direction = self.side_offset_direction()
        target.point.x += direction[0] * distance
        target.point.y += direction[1] * distance
        target.point.z += direction[2] * distance

        return self.clamp_tcp_target_z(target, label, self.grasp_strategy)

    def make_full_side_targets(self, grasp_target):
        pre_before, pre_grasp = self.make_side_target(
            grasp_target, self.pregrasp_distance, "pre_grasp"
        )
        approach_before, approach = self.make_side_target(
            grasp_target, self.final_approach_distance, "approach"
        )
        return pre_before, pre_grasp, approach_before, approach

    def object_depth_is_safe(self, object_base):
        if not self.is_valid_point(object_base.point):
            rospy.logwarn(
                "Transformed object point in %s is invalid: x=%.6f y=%.6f z=%.6f; skipping planning/execution",
                self.planning_frame,
                object_base.point.x,
                object_base.point.y,
                object_base.point.z,
            )
            return False

        if not self.object_z_check_enabled:
            return True

        if object_base.point.z < self.object_min_z or object_base.point.z > self.object_max_z:
            rospy.logwarn(
                "object z=%.6f in %s is outside enabled object z range [%.6f, %.6f]; skipping planning/execution",
                object_base.point.z,
                self.planning_frame,
                self.object_min_z,
                self.object_max_z,
            )
            return False

        return True

    def current_tool_orientation(self):
        pose = self.group.get_current_pose(self.end_effector_link)
        return pose.pose.orientation

    def fixed_orientation(self):
        q = self.fixed_orientation_quaternion
        norm = math.sqrt(sum(value * value for value in q))
        if norm <= 0.0:
            raise RuntimeError("fixed_orientation_quaternion has zero norm")
        return Quaternion(q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    def fixed_side_orientation(self):
        q = self.fixed_side_orientation_quaternion
        norm = math.sqrt(sum(value * value for value in q))
        if norm <= 0.0:
            raise RuntimeError("fixed_side_orientation_quaternion has zero norm")
        return Quaternion(q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    @staticmethod
    def rotate_vector_by_quaternion(vector, quaternion):
        x, y, z = vector
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            raise RuntimeError("Cannot rotate TCP offset by a zero-norm quaternion")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        # Rotate vector by q * v * q^-1 without pulling in an extra dependency.
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def target_orientation(self):
        if self.orientation_mode in ("position_only", "current"):
            return self.current_tool_orientation()
        if self.orientation_mode == "auto_side":
            if self.grasp_strategy == "auto_any_face":
                rospy.logwarn_once(
                    "orientation_mode=auto_side needs a concrete side direction; using current tool orientation outside candidate evaluation."
                )
                return self.current_tool_orientation()
            return self.side_grasp_orientation(self.side_offset_direction())
        if self.orientation_mode == "fixed":
            rospy.logwarn_once(
                "orientation_mode='fixed' is deprecated for this node; use 'fixed_side'."
            )
            return self.fixed_orientation()
        if self.orientation_mode == "fixed_side":
            return self.fixed_side_orientation()
        raise RuntimeError(
            "Unknown orientation_mode={!r}; use one of {}".format(
                self.orientation_mode, ", ".join(self.valid_orientation_modes)
            )
        )

    def local_preview_seed_orientation(self):
        if self.orientation_mode == "fixed_side":
            return self.fixed_side_orientation()
        if self.orientation_mode == "fixed":
            return self.fixed_orientation()
        return self.current_tool_orientation()

    def local_candidate_orientation(self, candidate, fallback_orientation):
        if self.orientation_mode == "auto_side":
            return self.side_grasp_orientation(candidate["direction"])
        return fallback_orientation

    def tool0_target_from_grasp_center(self, grasp_center_base, orientation):
        target = self.copy_point_stamped(grasp_center_base)
        if not self.tcp_offset_enabled:
            return target

        offset_tool = (
            self.tool0_to_grasp_center_offset_x,
            self.tool0_to_grasp_center_offset_y,
            self.tool0_to_grasp_center_offset_z,
        )
        offset_base = self.rotate_vector_by_quaternion(offset_tool, orientation)
        target.point.x -= offset_base[0]
        target.point.y -= offset_base[1]
        target.point.z -= offset_base[2]
        return target

    @staticmethod
    def orientation_text(orientation):
        return "x={:.6f} y={:.6f} z={:.6f} w={:.6f}".format(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def make_target_pose(self, grasp_center_base):
        orientation = self.target_orientation()
        return self.make_target_pose_with_orientation(grasp_center_base, orientation)

    def make_target_pose_with_orientation(self, grasp_center_base, orientation):
        tool0_target = self.tool0_target_from_grasp_center(
            grasp_center_base, orientation
        )
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = self.planning_frame
        target.pose.position.x = tool0_target.point.x
        target.pose.position.y = tool0_target.point.y
        target.pose.position.z = tool0_target.point.z
        target.pose.orientation = orientation
        return target

    def set_moveit_target(self, grasp_center_base):
        orientation = self.target_orientation()
        tool0_target = self.tool0_target_from_grasp_center(
            grasp_center_base, orientation
        )
        target_position = [
            tool0_target.point.x,
            tool0_target.point.y,
            tool0_target.point.z,
        ]
        if (
            self.planning_mode == "position_only"
            and self.orientation_mode == "position_only"
            and not self.tcp_offset_enabled
        ):
            self.group.set_position_target(target_position, self.end_effector_link)
            return target_position

        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = self.planning_frame
        target.pose.position.x = tool0_target.point.x
        target.pose.position.y = tool0_target.point.y
        target.pose.position.z = tool0_target.point.z
        target.pose.orientation = orientation
        self.group.set_pose_target(target, self.end_effector_link)
        return target_position

    @staticmethod
    def unpack_plan(plan_result):
        success_flag = None
        trajectory = plan_result
        if isinstance(plan_result, tuple):
            if len(plan_result) >= 1:
                success_flag = bool(plan_result[0])
            if len(plan_result) >= 2:
                trajectory = plan_result[1]

        points = getattr(
            getattr(trajectory, "joint_trajectory", None), "points", []
        )
        has_points = bool(points)
        success = has_points if success_flag is None else success_flag and has_points
        return success, trajectory, len(points)

    @staticmethod
    def unpack_cartesian_plan(plan_result):
        trajectory = None
        fraction = 0.0
        if isinstance(plan_result, tuple) and len(plan_result) >= 2:
            first, second = plan_result[0], plan_result[1]
            if hasattr(first, "joint_trajectory"):
                trajectory = first
                fraction = float(second)
            else:
                fraction = float(first)
                trajectory = second
        else:
            trajectory = plan_result

        points = getattr(
            getattr(trajectory, "joint_trajectory", None), "points", []
        )
        return trajectory, fraction, len(points)

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
        parameters = inspect.signature(self.group.compute_cartesian_path).parameters
        if "jump_threshold" in parameters:
            return self.group.compute_cartesian_path(
                waypoints,
                eef_step,
                jump_threshold,
                True,
            )

        if abs(jump_threshold) > 1e-9:
            rospy.logwarn_once(
                "This moveit_commander compute_cartesian_path binding does not expose jump_threshold; using avoid_collisions=True."
            )
        return self.group.compute_cartesian_path(waypoints, eef_step, True)

    def plan_to_stage_target(self, stage_target):
        target_position = [
            stage_target.point.x,
            stage_target.point.y,
            stage_target.point.z,
        ]
        if self.grasp_stage == "side_approach":
            target_pose = self.make_target_pose(stage_target)
            plan_result = self.compute_cartesian_path(
                [target_pose.pose],
                self.side_approach_cartesian_step,
                self.side_approach_jump_threshold,
            )
            trajectory, fraction, point_count = self.unpack_cartesian_plan(plan_result)
            success = (
                fraction >= self.side_approach_min_fraction and point_count > 0
            )
            return target_position, success, trajectory, point_count, fraction

        self.set_moveit_target(stage_target)
        plan_result = self.group.plan()
        self.group.clear_pose_targets()
        success, trajectory, point_count = self.unpack_plan(plan_result)
        return target_position, success, trajectory, point_count, None

    @staticmethod
    def points_have_same_positions(first_point, second_point, tolerance=1e-9):
        if len(first_point.positions) != len(second_point.positions):
            return False
        return all(
            abs(a - b) <= tolerance
            for a, b in zip(first_point.positions, second_point.positions)
        )

    def robot_state_from_trajectory_end(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("Cannot build robot state from empty trajectory")

        final_point = points[-1]
        final_positions = dict(
            zip(trajectory.joint_trajectory.joint_names, final_point.positions)
        )
        state = self.group.get_current_state()
        state_positions = list(state.joint_state.position)
        for index, name in enumerate(state.joint_state.name):
            if name in final_positions:
                state_positions[index] = final_positions[name]
        state.joint_state.position = state_positions
        return state

    def fk_pose_from_robot_state(self, robot_state, link_name=None):
        link_name = link_name or self.end_effector_link
        try:
            rospy.wait_for_service(self.fk_service_name, timeout=2.0)
            response = self.fk_service(
                Header(
                    stamp=rospy.Time.now(),
                    frame_id=self.planning_frame,
                ),
                [link_name],
                robot_state,
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise RuntimeError(
                "FK service {} failed for {}: {}".format(
                    self.fk_service_name,
                    link_name,
                    exc,
                )
            )

        if response.error_code.val != response.error_code.SUCCESS:
            raise RuntimeError(
                "FK service {} returned error code {} for {}".format(
                    self.fk_service_name,
                    response.error_code.val,
                    link_name,
                )
            )
        if not response.pose_stamped:
            raise RuntimeError(
                "FK service {} returned no pose for {}".format(
                    self.fk_service_name,
                    link_name,
                )
            )
        return response.pose_stamped[0]

    @staticmethod
    def format_joint_positions(names, positions, precision=4):
        return ", ".join(
            "{}={:.{}f}".format(name, value, precision)
            for name, value in zip(names, positions)
        )

    def log_trajectory_end_joint_state(self, trajectory, label):
        points = trajectory.joint_trajectory.points
        if not points:
            rospy.logwarn("%s end joint state unavailable: empty trajectory", label)
            return
        rospy.loginfo(
            "%s end joint state: %s",
            label,
            self.format_joint_positions(
                trajectory.joint_trajectory.joint_names,
                points[-1].positions,
            ),
        )

    @staticmethod
    def trajectory_joint_motion(trajectory):
        points = getattr(trajectory.joint_trajectory, "points", [])
        if len(points) < 2:
            return 0.0
        total = 0.0
        for previous, current in zip(points[:-1], points[1:]):
            if len(previous.positions) != len(current.positions):
                continue
            total += math.sqrt(
                sum(
                    (b - a) * (b - a)
                    for a, b in zip(previous.positions, current.positions)
                )
            )
        return total

    def log_robot_state_source(self, label, source, state, joint_names=None):
        state_positions = dict(zip(state.joint_state.name, state.joint_state.position))
        names = list(joint_names or self.group.get_active_joints())
        positions = [state_positions[name] for name in names if name in state_positions]
        used_names = [name for name in names if name in state_positions]
        rospy.loginfo(
            "%s start_state source: %s",
            label,
            source,
        )
        rospy.loginfo(
            "%s start_state joints: %s",
            label,
            self.format_joint_positions(used_names, positions),
        )

    def merge_trajectories(self, first_trajectory, second_trajectory):
        first_names = list(first_trajectory.joint_trajectory.joint_names)
        second_names = list(second_trajectory.joint_trajectory.joint_names)
        if first_names != second_names:
            raise RuntimeError(
                "Cannot merge trajectories with different joint names: {} vs {}".format(
                    first_names, second_names
                )
            )

        merged = RobotTrajectory()
        merged.joint_trajectory.header = deepcopy(
            first_trajectory.joint_trajectory.header
        )
        merged.joint_trajectory.joint_names = first_names

        for point in first_trajectory.joint_trajectory.points:
            merged.joint_trajectory.points.append(deepcopy(point))

        for index, point in enumerate(second_trajectory.joint_trajectory.points):
            if (
                index == 0
                and merged.joint_trajectory.points
                and self.points_have_same_positions(
                    merged.joint_trajectory.points[-1], point
                )
            ):
                continue
            merged.joint_trajectory.points.append(deepcopy(point))

        if len(merged.joint_trajectory.points) < 2:
            raise RuntimeError("Merged full_side_path trajectory has fewer than 2 points")

        for point in merged.joint_trajectory.points:
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = rospy.Duration(0.0)

        return merged

    def merge_trajectory_sequence(self, trajectories):
        non_empty = [
            trajectory
            for trajectory in trajectories
            if trajectory is not None
            and getattr(trajectory, "joint_trajectory", None) is not None
            and trajectory.joint_trajectory.points
        ]
        if not non_empty:
            raise RuntimeError("Cannot merge an empty trajectory sequence")

        merged = deepcopy(non_empty[0])
        for trajectory in non_empty[1:]:
            merged = self.merge_trajectories(merged, trajectory)
        return merged

    def retime_trajectory(self, trajectory):
        try:
            return self.group.retime_trajectory(
                self.group.get_current_state(),
                trajectory,
                self.velocity_scaling,
                self.acceleration_scaling,
            )
        except Exception as exc:
            rospy.logwarn(
                "Failed to retime merged trajectory: %s; using unretimed merged trajectory",
                exc,
            )
            return trajectory

    def plan_regular_segment_to_point(self, target_point, start_state=None):
        if start_state is None:
            self.group.set_start_state_to_current_state()
            rospy.loginfo(
                "regular planning start_state source: current real robot state"
            )
        else:
            self.group.set_start_state(start_state)
            self.log_robot_state_source(
                "regular planning",
                "previous trajectory end RobotState",
                start_state,
            )

        self.set_moveit_target(target_point)
        plan_result = self.group.plan()
        self.group.clear_pose_targets()
        success, trajectory, point_count = self.unpack_plan(plan_result)
        return success, trajectory, point_count

    def plan_pose_segment_to_point(
        self, target_point, orientation, start_state=None, segment_name="pose_segment"
    ):
        if start_state is None:
            self.group.set_start_state_to_current_state()
            rospy.loginfo("%s start_state source: current real robot state", segment_name)
        else:
            self.group.set_start_state(start_state)
            self.log_robot_state_source(
                segment_name,
                "previous trajectory end RobotState",
                start_state,
            )

        target_pose = self.make_target_pose_with_orientation(target_point, orientation)
        self.group.set_pose_target(target_pose, self.end_effector_link)
        plan_result = self.group.plan()
        self.group.clear_pose_targets()
        success, trajectory, point_count = self.unpack_plan(plan_result)
        return success, trajectory, point_count

    def plan_cartesian_segment_to_point(self, target_point, start_state):
        trajectory, fraction, point_count = self.plan_cartesian_segment_to_points(
            [target_point], start_state, "cartesian_segment"
        )
        return trajectory, fraction, point_count

    def plan_cartesian_segment_to_points(
        self, target_points, start_state, segment_name="cartesian_segment"
    ):
        waypoints = [self.make_target_pose(point).pose for point in target_points]
        return self.plan_cartesian_waypoint_poses(
            waypoints,
            start_state,
            segment_name,
        )

    def plan_cartesian_waypoint_poses(
        self,
        waypoint_poses,
        start_state,
        segment_name="cartesian_segment",
        allow_fallback=True,
    ):
        self.log_robot_state_source(
            segment_name,
            "previous trajectory end RobotState",
            start_state,
        )
        self.group.set_start_state(start_state)
        try:
            rospy.wait_for_service(self.cartesian_path_service_name, timeout=2.0)
            request = GetCartesianPathRequest()
            request.header.stamp = rospy.Time.now()
            request.header.frame_id = self.planning_frame
            request.start_state = start_state
            request.group_name = self.group_name
            request.link_name = self.end_effector_link
            request.waypoints = waypoint_poses
            request.max_step = self.cartesian_step
            request.jump_threshold = self.jump_threshold
            request.avoid_collisions = True
            rospy.loginfo(
                "%s Cartesian path start_state: explicit RobotState passed to %s",
                segment_name,
                self.cartesian_path_service_name,
            )
            response = self.cartesian_path_service(request)
            trajectory = response.solution
            fraction = float(response.fraction)
            points = getattr(trajectory.joint_trajectory, "points", [])
            return trajectory, fraction, len(points)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            if not allow_fallback:
                raise RuntimeError(
                    "{} Cartesian service failed and fallback is disabled: {}".format(
                        segment_name,
                        exc,
                    )
                )
            rospy.logwarn(
                "%s Cartesian service failed (%s); falling back to MoveGroupCommander.compute_cartesian_path, which may use the current robot state.",
                segment_name,
                exc,
            )
            plan_result = self.compute_cartesian_path(
                waypoint_poses,
                self.cartesian_step,
                self.jump_threshold,
            )
            return self.unpack_cartesian_plan(plan_result)

    def plan_full_side_path(self, object_input, object_base, cube_center):
        candidate = self.make_auto_candidate(
            cube_center, self.grasp_strategy, self.side_offset_direction()
        )
        pre_before = candidate["pre_grasp_before"]
        pre_grasp = candidate["pre_grasp"]
        approach_before = candidate["approach_before"]
        approach = candidate["approach_point"]
        grasp_before = candidate["grasp_before"]
        grasp_point = candidate["grasp_point"]
        direction = self.side_offset_direction()
        approach_direction = self.approach_motion_direction()

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_in_base: x=%.6f y=%.6f z=%.6f",
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("grasp_strategy: %s", self.grasp_strategy)
        rospy.loginfo("object_point_semantic: %s", self.object_point_semantic)
        rospy.loginfo(
            "cube_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            cube_center.point.x,
            cube_center.point.y,
            cube_center.point.z,
        )
        rospy.loginfo(
            "side_face_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            candidate["side_face_center"].point.x,
            candidate["side_face_center"].point.y,
            candidate["side_face_center"].point.z,
        )
        rospy.loginfo(
            "side offset direction in %s: x=%.1f y=%.1f z=%.1f",
            self.planning_frame,
            direction[0],
            direction[1],
            direction[2],
        )
        rospy.loginfo(
            "approach motion direction in %s: x=%.1f y=%.1f z=%.1f",
            self.planning_frame,
            approach_direction[0],
            approach_direction[1],
            approach_direction[2],
        )
        rospy.loginfo("pregrasp_clearance: %.6f", self.pregrasp_clearance)
        rospy.loginfo("final_clearance: %.6f", self.final_clearance)
        rospy.loginfo("grasp_clearance: %.6f", self.grasp_clearance)
        rospy.loginfo("safe_min_z: %.6f", self.safe_min_z)
        rospy.loginfo(
            "pre_grasp before safety clamp in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            pre_before.point.x,
            pre_before.point.y,
            pre_before.point.z,
        )
        rospy.loginfo(
            "pre_grasp in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            pre_grasp.point.x,
            pre_grasp.point.y,
            pre_grasp.point.z,
        )
        rospy.loginfo(
            "approach before safety clamp in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            approach_before.point.x,
            approach_before.point.y,
            approach_before.point.z,
        )
        rospy.loginfo(
            "approach in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            approach.point.x,
            approach.point.y,
            approach.point.z,
        )
        rospy.loginfo(
            "grasp_point before safety clamp in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            grasp_before.point.x,
            grasp_before.point.y,
            grasp_before.point.z,
        )
        rospy.loginfo(
            "grasp_point in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            grasp_point.point.x,
            grasp_point.point.y,
            grasp_point.point.z,
        )
        rospy.loginfo(
            "execute flag: execute=%s confirm=%s effective_execute=%s",
            self.execute,
            self.confirm,
            self.execute and self.confirm,
        )

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: skipped because object depth/safety check failed"
            )
            return False
        if not (
            self.tcp_target_z_is_safe(pre_grasp, "pre_grasp", self.grasp_strategy)
            and self.tcp_target_z_is_safe(approach, "approach", self.grasp_strategy)
            and self.tcp_target_z_is_safe(
                grasp_point, "grasp_point", self.grasp_strategy
            )
        ):
            rospy.logwarn(
                "planning result: stage=%s success=False because TCP target is below safe_min_z",
                self.grasp_stage,
            )
            return False

        self.group.set_start_state_to_current_state()
        pre_pose = self.make_target_pose(pre_grasp)
        self.group.set_pose_target(pre_pose, self.end_effector_link)
        first_result = self.group.plan()
        self.group.clear_pose_targets()
        first_success, first_trajectory, first_points = self.unpack_plan(first_result)
        rospy.loginfo(
            "full_side_path first segment: success=%s trajectory_points=%d",
            first_success,
            first_points,
        )
        if not first_success:
            rospy.loginfo(
                "planning result: stage=%s success=False trajectory_points=0",
                self.grasp_stage,
            )
            return False

        self.log_trajectory_end_joint_state(
            first_trajectory, "{} pre_grasp trajectory".format(self.grasp_strategy)
        )
        pre_state = self.robot_state_from_trajectory_end(first_trajectory)
        second_trajectory, cartesian_fraction, second_points = (
            self.plan_cartesian_segment_to_points(
                [approach, grasp_point],
                pre_state,
                "{} approach_grasp".format(self.grasp_strategy),
            )
        )
        self.group.set_start_state_to_current_state()
        rospy.loginfo(
            "Cartesian fraction: %.3f required>=%.3f",
            cartesian_fraction,
            self.min_cartesian_fraction,
        )
        if (
            cartesian_fraction < self.min_cartesian_fraction
            or second_points == 0
        ):
            rospy.logwarn(
                "planning result: stage=%s success=False trajectory_points=%d",
                self.grasp_stage,
                first_points + second_points,
            )
            return False

        merged = self.merge_trajectories(first_trajectory, second_trajectory)
        retimed = self.retime_trajectory(merged)
        total_points = len(retimed.joint_trajectory.points)
        rospy.loginfo(
            "planning result: stage=%s success=True trajectory_points=%d",
            self.grasp_stage,
            total_points,
        )
        candidate.update(
            {
                "valid": True,
                "cartesian_fraction": cartesian_fraction,
                "trajectory_points": total_points,
                "trajectory": retimed,
            }
        )
        self.publish_auto_candidate_markers(
            [candidate], self.grasp_strategy, object_raw=object_base
        )
        self.publish_display_trajectory(retimed)

        if self.execution_allowed():
            rospy.logwarn(
                "Executing FULL SIDE PATH only: current TCP -> pre_grasp -> approach_point -> grasp_point, no descent, no gripper, no grasp."
            )
            self.wait_before_execution()
            if not self.group.execute(retimed, wait=True):
                self.group.stop()
                raise RuntimeError("MoveIt execution of full_side_path failed")
            self.group.stop()
            rospy.loginfo("Full side path execution finished at grasp_point.")
        else:
            rospy.loginfo("PLAN ONLY: full_side_path trajectory was not executed.")
        return True

    def plan_full_side_path_debug(self, object_input, object_base, cube_center):
        strategy = "side_x_neg"
        direction = (-1.0, 0.0, 0.0)
        candidate = self.make_auto_candidate(cube_center, strategy, direction)
        pre_grasp = candidate["pre_grasp"]
        approach_point = candidate["approach_point"]
        approach_direction = (1.0, 0.0, 0.0)

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_in_base: x=%.6f y=%.6f z=%.6f",
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("grasp_strategy: %s", strategy)
        rospy.loginfo("object_point_semantic: %s", self.object_point_semantic)
        rospy.loginfo("cube_size: %.6f", self.cube_size)
        rospy.loginfo(
            "cube_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            cube_center.point.x,
            cube_center.point.y,
            cube_center.point.z,
        )
        rospy.loginfo(
            "side_face_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            candidate["side_face_center"].point.x,
            candidate["side_face_center"].point.y,
            candidate["side_face_center"].point.z,
        )
        rospy.loginfo(
            "side offset direction in %s: x=%.1f y=%.1f z=%.1f",
            self.planning_frame,
            direction[0],
            direction[1],
            direction[2],
        )
        rospy.loginfo(
            "approach motion direction in %s: x=%.1f y=%.1f z=%.1f",
            self.planning_frame,
            approach_direction[0],
            approach_direction[1],
            approach_direction[2],
        )
        rospy.loginfo("pregrasp_clearance: %.6f", self.pregrasp_clearance)
        rospy.loginfo("final_clearance: %.6f", self.final_clearance)
        rospy.loginfo("safe_min_z: %.6f", self.safe_min_z)
        rospy.loginfo(
            "clamp policy: clamp_grasp_points_for_preview=%s active_clamp=%s",
            self.clamp_grasp_points_for_preview,
            self.should_clamp_grasp_points(),
        )
        rospy.loginfo("planning_mode: %s", self.planning_mode)
        rospy.loginfo(
            "execute flag locked for debug: execute=%s confirm=%s gripper_execute=%s",
            self.execute,
            self.confirm,
            self.gripper_execute,
        )

        self.log_debug_side_target_geometry(candidate)
        self.publish_full_side_path_debug_markers(candidate, object_raw=object_base)

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: stage=full_side_path_debug success=False because object depth/safety check failed"
            )
            return False
        if not self.candidate_tcp_targets_are_safe(
            candidate,
            ("pre_grasp", "approach_point"),
        ):
            rospy.logwarn(
                "planning result: stage=full_side_path_debug success=False because TCP target is below safe_min_z"
            )
            return False

        first_success, first_trajectory, first_points = self.plan_regular_segment_to_point(
            pre_grasp
        )
        rospy.loginfo(
            "full_side_path_debug pre_grasp planning success=%s trajectory_points=%d",
            first_success,
            first_points,
        )
        if not first_success:
            candidate.update(
                {
                    "valid": False,
                    "planning_success": False,
                    "cartesian_fraction": 0.0,
                    "trajectory_points": first_points,
                    "trajectory": None,
                }
            )
            self.publish_full_side_path_debug_markers(candidate, object_raw=object_base)
            rospy.logwarn(
                "planning result: stage=full_side_path_debug success=False trajectory_points=%d",
                first_points,
            )
            return False

        self.log_trajectory_end_joint_state(
            first_trajectory, "{} debug pre_grasp trajectory".format(strategy)
        )
        pre_state = self.robot_state_from_trajectory_end(first_trajectory)
        pre_fk_pose = self.fk_pose_from_robot_state(pre_state)
        locked_orientation = deepcopy(pre_fk_pose.pose.orientation)
        approach_pose = self.make_target_pose_with_orientation(
            approach_point,
            locked_orientation,
        )

        rospy.loginfo(
            "full_side_path_debug pre_grasp FK pose in %s: x=%.6f y=%.6f z=%.6f orientation=%s",
            pre_fk_pose.header.frame_id,
            pre_fk_pose.pose.position.x,
            pre_fk_pose.pose.position.y,
            pre_fk_pose.pose.position.z,
            self.orientation_text(pre_fk_pose.pose.orientation),
        )
        if self.planning_mode == "position_only":
            rospy.loginfo(
                "planning_mode=position_only: using pre_grasp end FK orientation for approach_point: %s",
                self.orientation_text(locked_orientation),
            )
        rospy.loginfo(
            "full_side_path_debug Cartesian waypoints keep orientation: %s",
            self.orientation_text(locked_orientation),
        )

        second_trajectory, cartesian_fraction, second_points = (
            self.plan_cartesian_waypoint_poses(
                [deepcopy(pre_fk_pose.pose), approach_pose.pose],
                pre_state,
                "{} debug pre_to_approach".format(strategy),
                allow_fallback=False,
            )
        )
        self.group.set_start_state_to_current_state()
        rospy.loginfo(
            "full_side_path_debug Cartesian fraction=%.3f required>=%.3f trajectory_points=%d",
            cartesian_fraction,
            self.min_cartesian_fraction,
            second_points,
        )

        total_points = first_points + second_points
        if cartesian_fraction < self.min_cartesian_fraction or second_points == 0:
            candidate.update(
                {
                    "valid": False,
                    "planning_success": True,
                    "cartesian_fraction": cartesian_fraction,
                    "trajectory_points": total_points,
                    "trajectory": None,
                }
            )
            self.publish_full_side_path_debug_markers(candidate, object_raw=object_base)
            rospy.logwarn(
                "planning result: stage=full_side_path_debug success=False Cartesian fraction=%.3f trajectory_points=%d",
                cartesian_fraction,
                total_points,
            )
            rospy.logwarn(
                "Keep full_pick_place_preview disabled until full_side_path_debug Cartesian fraction >= 0.95."
            )
            return False

        merged = self.merge_trajectories(first_trajectory, second_trajectory)
        retimed = self.retime_trajectory(merged)
        total_points = len(retimed.joint_trajectory.points)
        candidate.update(
            {
                "valid": True,
                "planning_success": True,
                "cartesian_fraction": cartesian_fraction,
                "trajectory_points": total_points,
                "trajectory": retimed,
            }
        )
        self.publish_full_side_path_debug_markers(
            candidate,
            object_raw=object_base,
            selected=True,
        )
        self.publish_display_trajectory(retimed)
        rospy.loginfo(
            "planning result: stage=full_side_path_debug success=True Cartesian fraction=%.3f trajectory_points=%d",
            cartesian_fraction,
            total_points,
        )
        rospy.loginfo(
            "full_side_path_debug verified current -> pre_grasp -> approach_point only; no close, lift, place, retreat, execution, or gripper command."
        )
        rospy.loginfo(
            "Only after confirming this RViz path should full_pick_place_preview be re-enabled."
        )
        return True

    def evaluate_auto_candidate(self, candidate):
        strategy = candidate["strategy"]
        if not (
            self.tcp_target_z_is_safe(candidate["pre_grasp"], "pre_grasp", strategy)
            and self.tcp_target_z_is_safe(
                candidate["approach_point"], "approach_point", strategy
            )
            and self.tcp_target_z_is_safe(
                candidate["grasp_point"], "grasp_point", strategy
            )
        ):
            rospy.loginfo(
                "candidate %s planning success=False Cartesian fraction=0.000 trajectory_points=0",
                strategy,
            )
            candidate.update(
                {
                    "valid": False,
                    "planning_success": False,
                    "cartesian_fraction": 0.0,
                    "trajectory_points": 0,
                    "trajectory": None,
                    "score": None,
                }
            )
            return candidate

        self.group.set_start_state_to_current_state()
        pre_pose = self.make_target_pose(candidate["pre_grasp"])
        self.group.set_pose_target(pre_pose, self.end_effector_link)
        first_result = self.group.plan()
        self.group.clear_pose_targets()
        first_success, first_trajectory, first_points = self.unpack_plan(first_result)

        if not first_success:
            self.group.set_start_state_to_current_state()
            rospy.loginfo(
                "candidate %s planning success=False Cartesian fraction=0.000 trajectory_points=%d",
                strategy,
                first_points,
            )
            candidate.update(
                {
                    "valid": False,
                    "planning_success": False,
                    "cartesian_fraction": 0.0,
                    "trajectory_points": first_points,
                    "trajectory": None,
                    "score": None,
                }
            )
            return candidate

        self.log_trajectory_end_joint_state(
            first_trajectory, "{} pre_grasp trajectory".format(strategy)
        )
        pre_state = self.robot_state_from_trajectory_end(first_trajectory)
        second_trajectory, cartesian_fraction, second_points = (
            self.plan_cartesian_segment_to_points(
                [candidate["approach_point"], candidate["grasp_point"]],
                pre_state,
                "{} approach_grasp".format(strategy),
            )
        )
        self.group.set_start_state_to_current_state()

        if cartesian_fraction < self.min_cartesian_fraction or second_points == 0:
            trajectory_points = first_points + second_points
            rospy.loginfo(
                "candidate %s planning success=False Cartesian fraction=%.3f trajectory_points=%d",
                strategy,
                cartesian_fraction,
                trajectory_points,
            )
            candidate.update(
                {
                    "valid": False,
                    "planning_success": True,
                    "cartesian_fraction": cartesian_fraction,
                    "trajectory_points": trajectory_points,
                    "trajectory": None,
                    "score": None,
                }
            )
            return candidate

        merged = self.merge_trajectories(first_trajectory, second_trajectory)
        retimed = self.retime_trajectory(merged)
        trajectory_points = len(retimed.joint_trajectory.points)
        score = (
            trajectory_points,
            -cartesian_fraction,
            -candidate["approach_point"].point.z,
        )
        rospy.loginfo(
            "candidate %s planning success=True Cartesian fraction=%.3f trajectory_points=%d",
            strategy,
            cartesian_fraction,
            trajectory_points,
        )
        candidate.update(
            {
                "valid": True,
                "planning_success": True,
                "cartesian_fraction": cartesian_fraction,
                "trajectory_points": trajectory_points,
                "trajectory": retimed,
                "score": score,
            }
        )
        return candidate

    def mark_pick_place_candidate_failed(
        self, candidate, planning_success, fractions, trajectory_points, reason
    ):
        strategy = candidate["strategy"]
        min_fraction = min(fractions.values()) if fractions else 0.0
        rospy.loginfo(
            "candidate %s planning success=False Cartesian fraction=%.3f trajectory_points=%d reason=%s",
            strategy,
            min_fraction,
            trajectory_points,
            reason,
        )
        candidate.update(
            {
                "valid": False,
                "planning_success": planning_success,
                "cartesian_fraction": min_fraction,
                "cartesian_fractions": fractions,
                "trajectory_points": trajectory_points,
                "trajectory": None,
                "score": None,
                "failure_reason": reason,
            }
        )
        return candidate

    def evaluate_local_pick_place_candidate(self, candidate, seed_orientation):
        strategy = candidate["strategy"]
        fractions = {}
        trajectory_points = 0
        try:
            target_keys = (
                "pre_grasp",
                "approach_point",
                "grasp_point",
                "lift_point",
                "place_lift_point",
                "place_point",
                "retreat_point",
            )
            if not self.candidate_tcp_targets_are_safe(candidate, target_keys):
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    False,
                    fractions,
                    trajectory_points,
                    "TCP target below safe_min_z",
                )

            candidate_orientation = self.local_candidate_orientation(
                candidate, seed_orientation
            )
            if (
                self.planning_mode == "position_only"
                and self.orientation_mode == "position_only"
                and not self.tcp_offset_enabled
            ):
                first_success, first_trajectory, first_points = (
                    self.plan_regular_segment_to_point(candidate["pre_grasp"])
                )
                rospy.loginfo(
                    "candidate %s local pre_grasp uses position-only target; Cartesian orientation will be locked from pre_grasp FK.",
                    strategy,
                )
            else:
                first_success, first_trajectory, first_points = (
                    self.plan_pose_segment_to_point(
                        candidate["pre_grasp"],
                        candidate_orientation,
                        segment_name="{} local pre_grasp".format(strategy),
                    )
                )
            trajectory_points += first_points
            rospy.loginfo(
                "candidate %s local pre_grasp planning success=%s trajectory_points=%d place_offset=(%.3f, %.3f)",
                strategy,
                first_success,
                first_points,
                candidate["place_offset_xy"][0],
                candidate["place_offset_xy"][1],
            )
            if not first_success:
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    False,
                    fractions,
                    trajectory_points,
                    "pre_grasp pose planning failed",
                )

            self.log_trajectory_end_joint_state(
                first_trajectory, "{} local pre_grasp trajectory".format(strategy)
            )
            pre_state = self.robot_state_from_trajectory_end(first_trajectory)
            pre_fk_pose = self.fk_pose_from_robot_state(pre_state)
            locked_orientation = deepcopy(pre_fk_pose.pose.orientation)
            waypoint_keys = (
                "approach_point",
                "grasp_point",
                "lift_point",
                "place_lift_point",
                "place_point",
                "retreat_point",
            )
            waypoints = [deepcopy(pre_fk_pose.pose)]
            for key in waypoint_keys:
                waypoints.append(
                    self.make_target_pose_with_orientation(
                        candidate[key], locked_orientation
                    ).pose
                )

            cartesian_trajectory, cartesian_fraction, cartesian_points = (
                self.plan_cartesian_waypoint_poses(
                    waypoints,
                    pre_state,
                    "{} local pick_place_cartesian".format(strategy),
                    allow_fallback=False,
                )
            )
            fractions["local_pick_place"] = cartesian_fraction
            trajectory_points += cartesian_points
            rospy.loginfo(
                "candidate %s local Cartesian fraction=%.3f trajectory_points=%d orientation=%s",
                strategy,
                cartesian_fraction,
                cartesian_points,
                self.orientation_text(locked_orientation),
            )
            if (
                cartesian_fraction < self.min_cartesian_fraction
                or cartesian_points == 0
            ):
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "local Cartesian fraction below threshold",
                )

            merged = self.merge_trajectories(first_trajectory, cartesian_trajectory)
            retimed = self.retime_trajectory(merged)
            trajectory_points = len(retimed.joint_trajectory.points)
            joint_motion = self.trajectory_joint_motion(retimed)
            if joint_motion > self.local_max_joint_motion:
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "joint_motion {:.4f} exceeds local_max_joint_motion {:.4f}".format(
                        joint_motion,
                        self.local_max_joint_motion,
                    ),
                )
            zero_place_penalty = 1 if candidate["place_distance"] < 0.005 else 0
            score = (
                zero_place_penalty,
                joint_motion,
                trajectory_points,
                -cartesian_fraction,
                candidate["place_distance"],
            )
            rospy.loginfo(
                "candidate %s local planning success=True Cartesian fraction=%.3f trajectory_points=%d joint_motion=%.4f place_distance=%.3f",
                strategy,
                cartesian_fraction,
                trajectory_points,
                joint_motion,
                candidate["place_distance"],
            )
            candidate.update(
                {
                    "valid": True,
                    "planning_success": True,
                    "cartesian_fraction": cartesian_fraction,
                    "cartesian_fractions": fractions,
                    "trajectory_points": trajectory_points,
                    "trajectory": retimed,
                    "joint_motion": joint_motion,
                    "score": score,
                    "locked_orientation": locked_orientation,
                }
            )
            return candidate
        except Exception as exc:
            return self.mark_pick_place_candidate_failed(
                candidate,
                False,
                fractions,
                trajectory_points,
                "exception: {}".format(exc),
            )
        finally:
            self.group.set_start_state_to_current_state()

    def evaluate_pick_place_candidate(self, candidate):
        strategy = candidate["strategy"]
        fractions = {}
        trajectory_points = 0
        try:
            if not self.candidate_tcp_targets_are_safe(
                candidate,
                (
                    "pre_grasp",
                    "approach_point",
                    "grasp_point",
                    "lift_point",
                    "place_point",
                    "retreat_point",
                ),
            ):
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    False,
                    fractions,
                    trajectory_points,
                    "TCP target below safe_min_z",
                )

            first_success, first_trajectory, first_points = (
                self.plan_regular_segment_to_point(candidate["pre_grasp"])
            )
            trajectory_points += first_points
            rospy.loginfo(
                "candidate %s pre_grasp planning success=%s trajectory_points=%d",
                strategy,
                first_success,
                first_points,
            )
            if not first_success:
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    False,
                    fractions,
                    trajectory_points,
                    "pre_grasp regular planning failed",
                )

            self.log_trajectory_end_joint_state(
                first_trajectory, "{} pre_grasp trajectory".format(strategy)
            )
            pre_state = self.robot_state_from_trajectory_end(first_trajectory)
            approach_grasp_trajectory, approach_grasp_fraction, approach_grasp_points = (
                self.plan_cartesian_segment_to_points(
                    [candidate["approach_point"], candidate["grasp_point"]],
                    pre_state,
                    "{} approach_grasp".format(strategy),
                )
            )
            fractions["approach_grasp"] = approach_grasp_fraction
            trajectory_points += approach_grasp_points
            rospy.loginfo(
                "candidate %s approach/grasp Cartesian fraction=%.3f trajectory_points=%d",
                strategy,
                approach_grasp_fraction,
                approach_grasp_points,
            )
            if (
                approach_grasp_fraction < self.min_cartesian_fraction
                or approach_grasp_points == 0
            ):
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "approach/grasp Cartesian fraction below threshold",
                )

            self.log_trajectory_end_joint_state(
                approach_grasp_trajectory,
                "{} approach_grasp trajectory".format(strategy),
            )
            grasp_state = self.robot_state_from_trajectory_end(approach_grasp_trajectory)
            lift_trajectory, lift_fraction, lift_points = (
                self.plan_cartesian_segment_to_points(
                    [candidate["lift_point"]],
                    grasp_state,
                    "{} lift".format(strategy),
                )
            )
            fractions["lift"] = lift_fraction
            trajectory_points += lift_points
            rospy.loginfo(
                "candidate %s lift Cartesian fraction=%.3f trajectory_points=%d",
                strategy,
                lift_fraction,
                lift_points,
            )
            if lift_fraction < self.min_cartesian_fraction or lift_points == 0:
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "lift Cartesian fraction below threshold",
                )

            self.log_trajectory_end_joint_state(
                lift_trajectory, "{} lift trajectory".format(strategy)
            )
            lift_state = self.robot_state_from_trajectory_end(lift_trajectory)
            self.log_robot_state_source(
                "{} place".format(strategy),
                "lift trajectory end RobotState",
                lift_state,
            )
            place_success, place_trajectory, place_points = (
                self.plan_regular_segment_to_point(
                    candidate["place_point"], lift_state
                )
            )
            trajectory_points += place_points
            rospy.loginfo(
                "candidate %s place planning success=%s trajectory_points=%d",
                strategy,
                place_success,
                place_points,
            )
            if not place_success:
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "place regular planning failed",
                )

            self.log_trajectory_end_joint_state(
                place_trajectory, "{} place trajectory".format(strategy)
            )
            place_state = self.robot_state_from_trajectory_end(place_trajectory)
            retreat_trajectory, retreat_fraction, retreat_points = (
                self.plan_cartesian_segment_to_points(
                    [candidate["retreat_point"]],
                    place_state,
                    "{} retreat".format(strategy),
                )
            )
            fractions["retreat"] = retreat_fraction
            trajectory_points += retreat_points
            rospy.loginfo(
                "candidate %s retreat Cartesian fraction=%.3f trajectory_points=%d",
                strategy,
                retreat_fraction,
                retreat_points,
            )
            if (
                retreat_fraction < self.min_cartesian_fraction
                or retreat_points == 0
            ):
                return self.mark_pick_place_candidate_failed(
                    candidate,
                    True,
                    fractions,
                    trajectory_points,
                    "retreat Cartesian fraction below threshold",
                )

            merged = self.merge_trajectory_sequence(
                [
                    first_trajectory,
                    approach_grasp_trajectory,
                    lift_trajectory,
                    place_trajectory,
                    retreat_trajectory,
                ]
            )
            retimed = self.retime_trajectory(merged)
            trajectory_points = len(retimed.joint_trajectory.points)
            min_fraction = min(fractions.values()) if fractions else 1.0
            score = (
                -min_fraction,
                trajectory_points,
                -candidate["retreat_point"].point.z,
            )
            rospy.loginfo(
                "candidate %s planning success=True Cartesian fraction=%.3f trajectory_points=%d",
                strategy,
                min_fraction,
                trajectory_points,
            )
            candidate.update(
                {
                    "valid": True,
                    "planning_success": True,
                    "cartesian_fraction": min_fraction,
                    "cartesian_fractions": fractions,
                    "trajectory_points": trajectory_points,
                    "trajectory": retimed,
                    "score": score,
                }
            )
            return candidate
        except Exception as exc:
            return self.mark_pick_place_candidate_failed(
                candidate,
                False,
                fractions,
                trajectory_points,
                "exception: {}".format(exc),
            )
        finally:
            self.group.set_start_state_to_current_state()

    def log_pick_place_candidate_geometry(self, candidate):
        self.log_auto_candidate_geometry(candidate)
        strategy = candidate["strategy"]
        for key in ("lift_point", "place_point", "retreat_point"):
            raw = candidate.get("{}_raw".format(key), candidate[key])
            clamped = candidate.get("{}_clamped".format(key), candidate[key])
            clamp_applied = candidate.get("{}_clamp_applied".format(key), False)
            rospy.loginfo(
                "%s raw %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                raw.point.x,
                raw.point.y,
                raw.point.z,
            )
            rospy.loginfo(
                "%s clamped %s in %s: x=%.6f y=%.6f z=%.6f clamp_applied=%s",
                strategy,
                key,
                self.planning_frame,
                clamped.point.x,
                clamped.point.y,
                clamped.point.z,
                clamp_applied,
            )
            rospy.loginfo(
                "%s planning target %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                candidate[key].point.x,
                candidate[key].point.y,
                candidate[key].point.z,
            )

    def log_selected_pick_place_candidate(self, selected):
        rospy.loginfo("selected_strategy: %s", selected["strategy"])
        for key in (
            "cube_center",
            "side_face_center",
            "pre_grasp",
            "approach_point",
            "grasp_point",
            "lift_point",
            "place_point",
            "retreat_point",
        ):
            point = selected.get("{}_raw".format(key), selected[key])
            rospy.loginfo(
                "selected raw %s in %s: x=%.6f y=%.6f z=%.6f",
                key,
                self.planning_frame,
                point.point.x,
                point.point.y,
                point.point.z,
            )
        fractions = selected.get("cartesian_fractions", {})
        if fractions:
            rospy.loginfo(
                "Cartesian fractions: approach_grasp=%.3f lift=%.3f retreat=%.3f required>=%.3f",
                fractions.get("approach_grasp", 0.0),
                fractions.get("lift", 0.0),
                fractions.get("retreat", 0.0),
                self.min_cartesian_fraction,
            )
        rospy.loginfo(
            "trajectory_points: %d",
            selected["trajectory_points"],
        )

    def log_gripper_preview_or_stub(self):
        if self.gripper_preview_enabled:
            rospy.loginfo(
                "gripper preview enabled: close_preview=%.3f open_preview=%.3f",
                self.gripper_close_position,
                self.gripper_open_position,
            )
        if self.gripper_execute and self.execute and self.confirm:
            rospy.logwarn(
                "gripper_execute=true, but no reliable real gripper IO topic/service is configured in this node; real close/open commands are skipped."
            )
        else:
            rospy.loginfo(
                "GRIPPER PREVIEW ONLY: no real gripper IO will be called."
            )

    def top_down_orientation_candidates(self, cube_center=None):
        """Return roll variants with local +Z pointing down or down/outward."""
        z_axis = (0.0, 0.0, -1.0)
        finger_axes = (
            ("yaw_0", (0.0, 1.0, 0.0)),
            ("yaw_90", (1.0, 0.0, 0.0)),
            ("yaw_180", (0.0, -1.0, 0.0)),
            ("yaw_270", (-1.0, 0.0, 0.0)),
        )
        candidates = []
        if self.top_down_min_tilt_deg <= 1e-6:
            for label, y_axis in finger_axes:
                x_axis = self.normalize_vector(
                    self.cross_vectors(y_axis, z_axis),
                    "top-down remaining tool axis",
                )
                candidates.append(
                    (label, self.quaternion_from_axes(x_axis, y_axis, z_axis), 0.0)
                )

        if cube_center is None or self.top_down_max_tilt_deg < 1.0:
            return candidates

        radial_xy = (cube_center.point.x, cube_center.point.y, 0.0)
        radial_norm = math.hypot(radial_xy[0], radial_xy[1])
        if radial_norm <= 1e-6:
            return candidates
        outward = (radial_xy[0] / radial_norm, radial_xy[1] / radial_norm, 0.0)

        for tilt_deg in (10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0):
            if tilt_deg + 1e-6 < self.top_down_min_tilt_deg:
                continue
            if tilt_deg > self.top_down_max_tilt_deg + 1e-6:
                continue
            tilt = math.radians(tilt_deg)
            tilted_z = (
                outward[0] * math.sin(tilt),
                outward[1] * math.sin(tilt),
                -math.cos(tilt),
            )
            reference = (1.0, 0.0, 0.0)
            projection = self.dot_vectors(reference, tilted_z)
            y_seed = (
                reference[0] - projection * tilted_z[0],
                reference[1] - projection * tilted_z[1],
                reference[2] - projection * tilted_z[2],
            )
            if self.vector_norm(y_seed) < 0.1:
                reference = (0.0, 1.0, 0.0)
                projection = self.dot_vectors(reference, tilted_z)
                y_seed = (
                    reference[0] - projection * tilted_z[0],
                    reference[1] - projection * tilted_z[1],
                    reference[2] - projection * tilted_z[2],
                )
            y_seed = self.normalize_vector(y_seed, "top-down tilted tool Y axis")
            x_seed = self.normalize_vector(
                self.cross_vectors(y_seed, tilted_z),
                "top-down tilted tool X axis",
            )
            for roll_index in range(4):
                roll = roll_index * math.pi / 2.0
                x_axis = (
                    math.cos(roll) * x_seed[0] + math.sin(roll) * y_seed[0],
                    math.cos(roll) * x_seed[1] + math.sin(roll) * y_seed[1],
                    math.cos(roll) * x_seed[2] + math.sin(roll) * y_seed[2],
                )
                y_axis = (
                    -math.sin(roll) * x_seed[0] + math.cos(roll) * y_seed[0],
                    -math.sin(roll) * x_seed[1] + math.cos(roll) * y_seed[1],
                    -math.sin(roll) * x_seed[2] + math.cos(roll) * y_seed[2],
                )
                candidates.append(
                    (
                        "tilt_{:.0f}_roll_{}".format(tilt_deg, roll_index * 90),
                        self.quaternion_from_axes(x_axis, y_axis, tilted_z),
                        tilt_deg,
                    )
                )
        return candidates

    def plan_top_down_pick_preview(self, object_input, object_base, cube_center):
        """Plan-only top-down pick, with optional local place and retreat."""
        include_place = self.grasp_stage == "top_down_pick_place_preview"
        grasp_reference = self.point_with_xyz_offset(
            cube_center,
            self.object_to_grasp_offset_x,
            self.object_to_grasp_offset_y,
            self.object_to_grasp_offset_z,
        )
        hover_point = self.point_with_xyz_offset(
            grasp_reference, 0.0, 0.0, self.top_down_hover_height
        )
        grasp_point = self.copy_point_stamped(grasp_reference)
        lift_point = self.point_with_xyz_offset(
            grasp_reference, 0.0, 0.0, self.top_down_lift_height
        )
        place_cube_center = self.point_with_xyz_offset(
            cube_center,
            self.top_down_place_offset_x,
            self.top_down_place_offset_y,
            0.0,
        )
        place_grasp_reference = self.point_with_xyz_offset(
            place_cube_center,
            self.object_to_grasp_offset_x,
            self.object_to_grasp_offset_y,
            self.object_to_grasp_offset_z,
        )
        place_lift_point = self.point_with_xyz_offset(
            place_grasp_reference, 0.0, 0.0, self.top_down_lift_height
        )
        place_point = self.copy_point_stamped(place_grasp_reference)
        retreat_point = self.copy_point_stamped(place_lift_point)

        rospy.loginfo(
            "top-down object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "top-down cube_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            cube_center.point.x,
            cube_center.point.y,
            cube_center.point.z,
        )
        rospy.loginfo(
            "top-down hover/grasp/lift z: %.6f -> %.6f -> %.6f",
            hover_point.point.z,
            grasp_point.point.z,
            lift_point.point.z,
        )
        rospy.loginfo(
            "top-down grasp reference offset from cube center: (%.3f, %.3f, %.3f) m",
            self.object_to_grasp_offset_x,
            self.object_to_grasp_offset_y,
            self.object_to_grasp_offset_z,
        )
        if include_place:
            rospy.loginfo(
                "top-down place offset=(%.3f, %.3f) m; place_lift/place/retreat z: %.6f -> %.6f -> %.6f",
                self.top_down_place_offset_x,
                self.top_down_place_offset_y,
                place_lift_point.point.z,
                place_point.point.z,
                retreat_point.point.z,
            )
        rospy.loginfo(
            "top-down preview policy: PLAN ONLY, tool local +Z points downward with at most %.1f deg outward tilt, close marker at grasp%s, no real gripper IO",
            self.top_down_max_tilt_deg,
            " and open marker at place" if include_place else "",
        )

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: stage=%s success=False because object depth/safety check failed",
                self.grasp_stage,
            )
            return False
        for label, point in (
            ("hover_point", hover_point),
            ("grasp_point", grasp_point),
            ("lift_point", lift_point),
            ("place_lift_point", place_lift_point),
            ("place_point", place_point),
            ("retreat_point", retreat_point),
        ):
            if not include_place and label.startswith(("place_", "retreat_")):
                continue
            if not self.tcp_target_z_is_safe(point, label, "top_down"):
                rospy.logwarn(
                    "planning result: stage=%s success=False because %s is below safe_min_z",
                    self.grasp_stage,
                    label,
                )
                return False

        evaluated = []
        smallest_successful_tilt = None
        for candidate_id, orientation, requested_tilt_deg in self.top_down_orientation_candidates(
            cube_center
        ):
            candidate = {
                "candidate_id": candidate_id,
                "strategy": "top_down",
                "cube_center": self.copy_point_stamped(cube_center),
                "pre_grasp": self.copy_point_stamped(hover_point),
                "approach_point": self.copy_point_stamped(hover_point),
                "grasp_point": self.copy_point_stamped(grasp_point),
                "lift_point": self.copy_point_stamped(lift_point),
                "valid": False,
                "trajectory": None,
                "requested_tilt_deg": requested_tilt_deg,
            }
            if include_place:
                candidate.update(
                    {
                        "place_cube_center": self.copy_point_stamped(
                            place_cube_center
                        ),
                        "place_lift_point": self.copy_point_stamped(
                            place_lift_point
                        ),
                        "place_point": self.copy_point_stamped(place_point),
                        "retreat_point": self.copy_point_stamped(retreat_point),
                    }
                )
            try:
                desired_finger_axis = self.rotate_vector_by_quaternion(
                    (0.0, 1.0, 0.0), orientation
                )
                if (
                    abs(desired_finger_axis[2])
                    > self.top_down_max_finger_axis_vertical
                ):
                    rospy.loginfo(
                        "top-down candidate %s rejected before planning: finger-axis vertical %.4f > %.4f",
                        candidate_id,
                        abs(desired_finger_axis[2]),
                        self.top_down_max_finger_axis_vertical,
                    )
                    evaluated.append(candidate)
                    continue
                first_success, first_trajectory, first_points = (
                    self.plan_pose_segment_to_point(
                        hover_point,
                        orientation,
                        segment_name="top_down {} current_to_hover".format(
                            candidate_id
                        ),
                    )
                )
                if not first_success:
                    rospy.loginfo(
                        "top-down candidate %s failed current_to_hover planning",
                        candidate_id,
                    )
                    evaluated.append(candidate)
                    continue

                hover_state = self.robot_state_from_trajectory_end(first_trajectory)
                hover_fk = self.fk_pose_from_robot_state(hover_state)
                locked_orientation = deepcopy(hover_fk.pose.orientation)
                locked_finger_axis = self.rotate_vector_by_quaternion(
                    (0.0, 1.0, 0.0), locked_orientation
                )
                if (
                    abs(locked_finger_axis[2])
                    > self.top_down_max_finger_axis_vertical
                ):
                    rospy.loginfo(
                        "top-down candidate %s rejected: actual finger-axis vertical %.4f > %.4f",
                        candidate_id,
                        abs(locked_finger_axis[2]),
                        self.top_down_max_finger_axis_vertical,
                    )
                    evaluated.append(candidate)
                    continue
                tool_z_axis = self.rotate_vector_by_quaternion(
                    (0.0, 0.0, 1.0), locked_orientation
                )
                downward_alignment = -tool_z_axis[2]
                if downward_alignment < math.cos(
                    math.radians(self.top_down_max_tilt_deg + 2.0)
                ):
                    rospy.logwarn(
                        "top-down candidate %s rejected: tool +Z downward alignment %.4f exceeds %.1f deg tilt limit",
                        candidate_id,
                        downward_alignment,
                        self.top_down_max_tilt_deg,
                    )
                    evaluated.append(candidate)
                    continue

                waypoint_poses = [deepcopy(hover_fk.pose)]
                waypoint_poses.append(
                    self.make_target_pose_with_orientation(
                        grasp_point, locked_orientation
                    ).pose
                )
                waypoint_poses.append(
                    self.make_target_pose_with_orientation(
                        lift_point, locked_orientation
                    ).pose
                )
                if include_place:
                    for point in (place_lift_point, place_point, retreat_point):
                        waypoint_poses.append(
                            self.make_target_pose_with_orientation(
                                point, locked_orientation
                            ).pose
                        )
                cartesian_trajectory, fraction, cartesian_points = (
                    self.plan_cartesian_waypoint_poses(
                        waypoint_poses,
                        hover_state,
                        "top_down {} pick_place_cartesian".format(candidate_id),
                        allow_fallback=False,
                    )
                )
                if (
                    fraction < self.top_down_min_cartesian_fraction
                    or cartesian_points == 0
                ):
                    rospy.loginfo(
                        "top-down candidate %s failed Cartesian fraction %.3f < %.3f",
                        candidate_id,
                        fraction,
                        self.top_down_min_cartesian_fraction,
                    )
                    candidate.update(
                        {
                            "cartesian_fraction": fraction,
                            "trajectory_points": first_points + cartesian_points,
                        }
                    )
                    evaluated.append(candidate)
                    continue

                merged = self.merge_trajectories(
                    first_trajectory, cartesian_trajectory
                )
                retimed = self.retime_trajectory(merged)
                joint_motion = self.trajectory_joint_motion(retimed)
                trajectory_points = len(retimed.joint_trajectory.points)
                if joint_motion > self.top_down_max_joint_motion:
                    rospy.loginfo(
                        "top-down candidate %s rejected: joint_motion %.4f > %.4f",
                        candidate_id,
                        joint_motion,
                        self.top_down_max_joint_motion,
                    )
                    candidate.update(
                        {
                            "cartesian_fraction": fraction,
                            "trajectory_points": trajectory_points,
                            "joint_motion": joint_motion,
                        }
                    )
                    evaluated.append(candidate)
                    continue

                candidate.update(
                    {
                        "valid": True,
                        "trajectory": retimed,
                        "cartesian_fraction": fraction,
                        "trajectory_points": trajectory_points,
                        "joint_motion": joint_motion,
                        "locked_orientation": locked_orientation,
                        "downward_alignment": downward_alignment,
                        # Prefer the least total joint travel.  Tilt is only a
                        # secondary tie-breaker; this avoids visually dramatic
                        # shoulder/wrist winding when a calmer path exists.
                        "score": (
                            joint_motion,
                            requested_tilt_deg,
                            trajectory_points,
                        ),
                    }
                )
                rospy.loginfo(
                    "top-down candidate %s success=True requested_tilt=%.1fdeg Cartesian fraction=%.3f trajectory_points=%d joint_motion=%.4f downward_alignment=%.4f",
                    candidate_id,
                    requested_tilt_deg,
                    fraction,
                    trajectory_points,
                    joint_motion,
                    downward_alignment,
                )
                rospy.loginfo(
                    "top-down candidate %s locked_orientation=[%.10f, %.10f, %.10f, %.10f]",
                    candidate_id,
                    locked_orientation.x,
                    locked_orientation.y,
                    locked_orientation.z,
                    locked_orientation.w,
                )
                evaluated.append(candidate)
                if smallest_successful_tilt is None:
                    smallest_successful_tilt = requested_tilt_deg
            except Exception as exc:
                rospy.logwarn(
                    "top-down candidate %s failed: %s", candidate_id, exc
                )
                evaluated.append(candidate)
            finally:
                self.group.set_start_state_to_current_state()

        valid_candidates = [candidate for candidate in evaluated if candidate["valid"]]
        if not valid_candidates:
            self.publish_auto_candidate_markers(evaluated, object_raw=object_base)
            rospy.logwarn(
                "planning result: stage=%s success=False trajectory_points=0",
                self.grasp_stage,
            )
            return False

        selected = min(valid_candidates, key=lambda candidate: candidate["score"])
        self.publish_auto_candidate_markers(
            evaluated,
            "top_down",
            object_raw=object_base,
            selected_candidate_id=selected["candidate_id"],
        )
        self.publish_selected_object_preview_marker(selected)
        self.publish_display_trajectory(selected["trajectory"])
        self.log_gripper_preview_or_stub()
        rospy.loginfo(
            "planning result: stage=%s success=True selected_orientation=%s requested_tilt=%.1fdeg Cartesian fraction=%.3f trajectory_points=%d joint_motion=%.4f downward_alignment=%.4f",
            self.grasp_stage,
            selected["candidate_id"],
            selected["requested_tilt_deg"],
            selected["cartesian_fraction"],
            selected["trajectory_points"],
            selected["joint_motion"],
            selected["downward_alignment"],
        )
        if include_place:
            rospy.loginfo(
                "PLAN ONLY: hover -> descend -> close preview -> lift -> local translate -> place -> open preview -> retreat was published to RViz; no robot or gripper command."
            )
        else:
            rospy.loginfo(
                "PLAN ONLY: top-down hover -> vertical descend -> grasp-close preview -> vertical lift was published to RViz; no robot or gripper command."
            )
        return True

    def plan_local_pick_place_preview(self, object_input, object_base, cube_center):
        candidates = self.make_local_pick_place_candidates(cube_center)
        seed_orientation = self.local_preview_seed_orientation()
        candidate_names = sorted({candidate["strategy"] for candidate in candidates})
        candidate_offsets = sorted(
            {
                (
                    round(candidate["place_offset_xy"][0], 4),
                    round(candidate["place_offset_xy"][1], 4),
                )
                for candidate in candidates
            }
        )

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_in_base: x=%.6f y=%.6f z=%.6f",
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("local candidate strategies: %s", ", ".join(candidate_names))
        rospy.loginfo("local candidate place offsets: %s", candidate_offsets)
        rospy.loginfo("object_point_semantic: %s", self.object_point_semantic)
        rospy.loginfo("cube_size: %.6f", self.cube_size)
        rospy.loginfo(
            "cube_center in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            cube_center.point.x,
            cube_center.point.y,
            cube_center.point.z,
        )
        rospy.loginfo(
            "local place radius: %.6f local_place_offset: %.6f local_lift_height: %.6f local_retreat_distance: %.6f",
            self.local_place_radius,
            self.local_place_offset,
            self.local_lift_height,
            self.local_retreat_distance,
        )
        rospy.loginfo(
            "seed orientation for minimal rotation: %s",
            self.orientation_text(seed_orientation),
        )
        rospy.loginfo(
            "clamp policy: clamp_grasp_points_for_preview=%s active_clamp=%s",
            self.clamp_grasp_points_for_preview,
            self.should_clamp_grasp_points(),
        )
        rospy.loginfo(
            "execute flag locked for local preview: execute=%s confirm=%s gripper_execute=%s",
            self.execute,
            self.confirm,
            self.gripper_execute,
        )

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: stage=local_pick_place_preview success=False because object depth/safety check failed"
            )
            self.publish_auto_candidate_markers(candidates, object_raw=object_base)
            return False

        evaluated = [
            self.evaluate_local_pick_place_candidate(candidate, seed_orientation)
            for candidate in candidates
        ]
        valid_candidates = [candidate for candidate in evaluated if candidate["valid"]]

        if not valid_candidates:
            rospy.logwarn(
                "planning result: stage=local_pick_place_preview success=False trajectory_points=0"
            )
            self.publish_auto_candidate_markers(evaluated, object_raw=object_base)
            return False

        selected = min(valid_candidates, key=lambda candidate: candidate["score"])
        if selected["place_distance"] < 0.005:
            rospy.logwarn(
                "Selected fallback place target is nearly the original cube center; no nonzero local offset was feasible."
            )
        rospy.loginfo("selected_strategy: %s", selected["strategy"])
        rospy.loginfo(
            "selected place_offset in %s: dx=%.6f dy=%.6f radius=%.6f <= %.6f",
            self.planning_frame,
            selected["place_offset_xy"][0],
            selected["place_offset_xy"][1],
            selected["place_distance"],
            self.local_place_radius,
        )
        for key in (
            "cube_center",
            "side_face_center",
            "pre_grasp",
            "approach_point",
            "grasp_point",
            "lift_point",
            "place_lift_point",
            "place_point",
            "retreat_point",
        ):
            if key not in selected:
                continue
            point = selected[key]
            rospy.loginfo(
                "selected %s in %s: x=%.6f y=%.6f z=%.6f",
                key,
                self.planning_frame,
                point.point.x,
                point.point.y,
                point.point.z,
            )
        rospy.loginfo(
            "planning result: stage=local_pick_place_preview success=True selected_strategy=%s Cartesian fraction=%.3f trajectory_points=%d joint_motion=%.4f",
            selected["strategy"],
            selected["cartesian_fraction"],
            selected["trajectory_points"],
            selected["joint_motion"],
        )

        suggested_cube_center = self.suggested_object_cube_center(cube_center, selected)
        suggested_verified = None
        if self.suggested_object_marker_enabled and self.suggested_object_verify_plan:
            suggested_verified = self.verify_suggested_object_plan(
                suggested_cube_center,
                seed_orientation,
            )
        if self.suggested_object_marker_enabled:
            self.publish_suggested_object_marker(
                suggested_cube_center,
                suggested_verified,
            )

        self.publish_auto_candidate_markers(
            evaluated,
            selected["strategy"],
            object_raw=object_base,
            selected_candidate_id=selected.get("candidate_id", ""),
        )
        self.publish_selected_object_preview_marker(selected)
        self.publish_display_trajectory(selected["trajectory"])
        self.log_gripper_preview_or_stub()
        rospy.loginfo(
            "PLAN ONLY: local_pick_place_preview trajectory was published to RViz; no real robot execution and no gripper command."
        )
        return True

    def plan_pick_place_preview(self, object_input, object_base, cube_center):
        if self.grasp_strategy == "auto_any_face":
            candidates = self.make_pick_place_candidates(cube_center)
        else:
            candidates = [
                self.make_pick_place_candidate(
                    cube_center, self.grasp_strategy, self.side_offset_direction()
                )
            ]
        candidate_names = [candidate["strategy"] for candidate in candidates]

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_in_base: x=%.6f y=%.6f z=%.6f",
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("grasp_strategy: %s", self.grasp_strategy)
        rospy.loginfo("candidate directions: %s", ", ".join(candidate_names))
        rospy.loginfo("object_point_semantic: %s", self.object_point_semantic)
        rospy.loginfo("cube_size: %.6f", self.cube_size)
        rospy.loginfo("pregrasp_clearance: %.6f", self.pregrasp_clearance)
        rospy.loginfo("final_clearance: %.6f", self.final_clearance)
        rospy.loginfo("grasp_clearance: %.6f", self.grasp_clearance)
        rospy.loginfo("lift_height: %.6f", self.lift_height)
        rospy.loginfo(
            "place_offset in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            self.place_offset_x,
            self.place_offset_y,
            self.place_offset_z,
        )
        rospy.loginfo("retreat_distance: %.6f", self.retreat_distance)
        rospy.loginfo("safe_min_z: %.6f", self.safe_min_z)
        rospy.loginfo("orientation_mode: %s", self.orientation_mode)
        rospy.loginfo(
            "fixed orientation: %s",
            self.orientation_text(self.fixed_side_orientation()),
        )
        rospy.loginfo(
            "tcp offset: enabled=%s tool0_to_grasp_center_offset=(%.6f, %.6f, %.6f)",
            self.tcp_offset_enabled,
            self.tool0_to_grasp_center_offset_x,
            self.tool0_to_grasp_center_offset_y,
            self.tool0_to_grasp_center_offset_z,
        )
        rospy.loginfo(
            "clamp policy: clamp_grasp_points_for_preview=%s clamp_grasp_points_for_execution=%s allow_low_grasp_execution=%s active_clamp=%s",
            self.clamp_grasp_points_for_preview,
            self.clamp_grasp_points_for_execution,
            self.allow_low_grasp_execution,
            self.should_clamp_grasp_points(),
        )
        rospy.loginfo("enable_z_clamp: %s", self.enable_z_clamp)
        rospy.loginfo("object_z_check_enabled: %s", self.object_z_check_enabled)
        rospy.loginfo(
            "execute flag: execute=%s confirm=%s effective_execute=%s gripper_execute=%s",
            self.execute,
            self.confirm,
            self.execute and self.confirm,
            self.gripper_execute,
        )

        for candidate in candidates:
            self.log_pick_place_candidate_geometry(candidate)

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: skipped because object depth/safety check failed"
            )
            self.publish_auto_candidate_markers(candidates, object_raw=object_base)
            return False

        evaluated = [
            self.evaluate_pick_place_candidate(candidate) for candidate in candidates
        ]
        valid_candidates = [candidate for candidate in evaluated if candidate["valid"]]

        if not valid_candidates:
            rospy.logwarn(
                "planning result: stage=full_pick_place_preview success=False trajectory_points=0"
            )
            self.publish_auto_candidate_markers(evaluated, object_raw=object_base)
            return False

        selected = min(valid_candidates, key=lambda candidate: candidate["score"])
        self.log_selected_pick_place_candidate(selected)
        rospy.loginfo(
            "planning result: stage=full_pick_place_preview success=True selected_strategy=%s Cartesian fraction=%.3f trajectory_points=%d",
            selected["strategy"],
            selected["cartesian_fraction"],
            selected["trajectory_points"],
        )

        self.publish_auto_candidate_markers(
            evaluated, selected["strategy"], object_raw=object_base
        )
        self.publish_display_trajectory(selected["trajectory"])
        self.log_gripper_preview_or_stub()

        if self.execution_allowed():
            rospy.logwarn(
                "Executing FULL PICK/PLACE PREVIEW robot trajectory only: no real gripper IO is hardcoded in this node."
            )
            self.wait_before_execution()
            if not self.group.execute(selected["trajectory"], wait=True):
                self.group.stop()
                raise RuntimeError(
                    "MoveIt execution of full_pick_place_preview failed"
                )
            self.group.stop()
            rospy.loginfo(
                "Full pick/place preview trajectory execution finished using %s.",
                selected["strategy"],
            )
        else:
            rospy.loginfo(
                "PLAN ONLY: full_pick_place_preview trajectory was not executed."
            )
        return True

    def log_auto_candidate_geometry(self, candidate):
        strategy = candidate["strategy"]
        rospy.loginfo(
            "%s cube_center in %s: x=%.6f y=%.6f z=%.6f",
            strategy,
            self.planning_frame,
            candidate["cube_center"].point.x,
            candidate["cube_center"].point.y,
            candidate["cube_center"].point.z,
        )
        rospy.loginfo(
            "%s side_face_center in %s: x=%.6f y=%.6f z=%.6f",
            strategy,
            self.planning_frame,
            candidate["side_face_center"].point.x,
            candidate["side_face_center"].point.y,
            candidate["side_face_center"].point.z,
        )
        for key in ("pre_grasp", "approach_point", "grasp_point"):
            raw = candidate.get("{}_raw".format(key), candidate[key])
            clamped = candidate.get("{}_clamped".format(key), candidate[key])
            clamp_applied = candidate.get("{}_clamp_applied".format(key), False)
            rospy.loginfo(
                "%s raw %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                raw.point.x,
                raw.point.y,
                raw.point.z,
            )
            rospy.loginfo(
                "%s clamped %s in %s: x=%.6f y=%.6f z=%.6f clamp_applied=%s",
                strategy,
                key,
                self.planning_frame,
                clamped.point.x,
                clamped.point.y,
                clamped.point.z,
                clamp_applied,
            )
            rospy.loginfo(
                "%s planning target %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                candidate[key].point.x,
                candidate[key].point.y,
                candidate[key].point.z,
            )

    def log_debug_side_target_geometry(self, candidate):
        strategy = candidate["strategy"]
        rospy.loginfo(
            "%s cube_center in %s: x=%.6f y=%.6f z=%.6f",
            strategy,
            self.planning_frame,
            candidate["cube_center"].point.x,
            candidate["cube_center"].point.y,
            candidate["cube_center"].point.z,
        )
        rospy.loginfo(
            "%s side_face_center in %s: x=%.6f y=%.6f z=%.6f",
            strategy,
            self.planning_frame,
            candidate["side_face_center"].point.x,
            candidate["side_face_center"].point.y,
            candidate["side_face_center"].point.z,
        )
        for key in ("pre_grasp", "approach_point"):
            raw = candidate["{}_raw".format(key)]
            clamped = candidate["{}_clamped".format(key)]
            target = candidate[key]
            clamp_applied = candidate.get("{}_clamp_applied".format(key), False)
            rospy.loginfo(
                "%s raw %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                raw.point.x,
                raw.point.y,
                raw.point.z,
            )
            rospy.loginfo(
                "%s clamped %s in %s: x=%.6f y=%.6f z=%.6f clamp_applied=%s",
                strategy,
                key,
                self.planning_frame,
                clamped.point.x,
                clamped.point.y,
                clamped.point.z,
                clamp_applied,
            )
            rospy.loginfo(
                "%s planning target %s in %s: x=%.6f y=%.6f z=%.6f",
                strategy,
                key,
                self.planning_frame,
                target.point.x,
                target.point.y,
                target.point.z,
            )

    def plan_auto_any_face(self, object_input, object_base):
        cube_center = self.stage_cube_center(object_base)
        if cube_center is None:
            return False
        candidates = self.make_auto_candidates(cube_center)
        candidate_names = [candidate["strategy"] for candidate in candidates]

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_in_base: x=%.6f y=%.6f z=%.6f",
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("grasp_strategy: %s", self.grasp_strategy)
        rospy.loginfo("auto candidate directions: %s", ", ".join(candidate_names))
        rospy.loginfo("object_point_semantic: %s", self.object_point_semantic)
        rospy.loginfo("cube_size: %.6f", self.cube_size)
        rospy.loginfo("pregrasp_clearance: %.6f", self.pregrasp_clearance)
        rospy.loginfo("final_clearance: %.6f", self.final_clearance)
        rospy.loginfo("safe_min_z: %.6f", self.safe_min_z)
        rospy.loginfo(
            "execute flag: execute=%s confirm=%s effective_execute=%s",
            self.execute,
            self.confirm,
            self.execute and self.confirm,
        )

        for candidate in candidates:
            self.log_auto_candidate_geometry(candidate)

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: skipped because object depth/safety check failed"
            )
            self.publish_auto_candidate_markers(candidates, object_raw=object_base)
            return False

        evaluated = [
            self.evaluate_auto_candidate(candidate) for candidate in candidates
        ]
        valid_candidates = [candidate for candidate in evaluated if candidate["valid"]]

        if not valid_candidates:
            rospy.logwarn(
                "planning result: stage=full_side_path auto_any_face success=False trajectory_points=0"
            )
            self.publish_auto_candidate_markers(evaluated, object_raw=object_base)
            return False

        selected = min(valid_candidates, key=lambda candidate: candidate["score"])
        selected_strategy = selected["strategy"]
        rospy.loginfo("selected_strategy: %s", selected_strategy)
        rospy.loginfo(
            "selected pre_grasp in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            selected["pre_grasp"].point.x,
            selected["pre_grasp"].point.y,
            selected["pre_grasp"].point.z,
        )
        rospy.loginfo(
            "selected approach_point in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            selected["approach_point"].point.x,
            selected["approach_point"].point.y,
            selected["approach_point"].point.z,
        )
        rospy.loginfo(
            "selected grasp_point in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            selected["grasp_point"].point.x,
            selected["grasp_point"].point.y,
            selected["grasp_point"].point.z,
        )
        rospy.loginfo(
            "Cartesian fraction: %.3f required>=%.3f",
            selected["cartesian_fraction"],
            self.min_cartesian_fraction,
        )
        rospy.loginfo(
            "planning result: stage=full_side_path auto_any_face success=True selected_strategy=%s trajectory_points=%d",
            selected_strategy,
            selected["trajectory_points"],
        )

        self.publish_auto_candidate_markers(
            evaluated, selected_strategy, object_raw=object_base
        )
        self.publish_display_trajectory(selected["trajectory"])

        if self.execution_allowed():
            rospy.logwarn(
                "Executing AUTO FULL SIDE PATH only: current TCP -> pre_grasp -> approach_point, no descent, no gripper, no grasp."
            )
            self.wait_before_execution()
            if not self.group.execute(selected["trajectory"], wait=True):
                self.group.stop()
                raise RuntimeError(
                    "MoveIt execution of auto_any_face full_side_path failed"
                )
            self.group.stop()
            rospy.loginfo(
                "Auto full side path execution finished using %s.",
                selected_strategy,
            )
        else:
            rospy.loginfo("PLAN ONLY: auto_any_face trajectory was not executed.")
        return True

    def wait_before_execution(self):
        if self.execute_delay <= 0.0:
            return

        end_time = rospy.Time.now() + rospy.Duration(self.execute_delay)
        rospy.logwarn(
            "execute_delay is %.1f seconds; execution will start after countdown unless node is stopped.",
            self.execute_delay,
        )
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            remaining = (end_time - rospy.Time.now()).to_sec()
            if remaining <= 0.0:
                break
            rospy.logwarn("Executing in %.1f seconds...", remaining)
            rate.sleep()

    def publish_display_trajectory(self, trajectory):
        display = DisplayTrajectory()
        display.trajectory_start = self.group.get_current_state()
        display.trajectory.append(trajectory)
        self.display.publish(display)

    def make_debug_marker(
        self,
        point,
        marker_id,
        namespace,
        color,
        scale,
        marker_type=Marker.SPHERE,
        text="",
    ):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.planning_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        return marker

    def make_debug_text_marker(self, point, marker_id, namespace, text, selected):
        marker = self.make_debug_marker(
            point,
            marker_id,
            namespace,
            (0.1, 1.0, 0.1, 1.0) if selected else (0.8, 0.8, 0.8, 0.55),
            0.035 if selected else 0.025,
            marker_type=Marker.TEXT_VIEW_FACING,
            text=text,
        )
        marker.pose.position.z += 0.05 if selected else 0.035
        marker.scale.x = 0.0
        marker.scale.y = 0.0
        marker.scale.z = 0.04 if selected else 0.03
        return marker

    def suggested_object_cube_center(self, current_cube_center, selected):
        if (
            self.suggested_object_use_selected_place
            and "place_cube_center" in selected
        ):
            return self.copy_point_stamped(selected["place_cube_center"])
        return self.point_with_xyz_offset(
            current_cube_center,
            self.suggested_object_offset_x,
            self.suggested_object_offset_y,
            self.suggested_object_offset_z,
        )

    def verify_suggested_object_plan(self, suggested_cube_center, seed_orientation):
        rospy.loginfo(
            "verifying suggested object position as a new pick target..."
        )
        candidates = self.make_local_pick_place_candidates(suggested_cube_center)[
            : self.suggested_object_max_candidates
        ]
        evaluated = [
            self.evaluate_local_pick_place_candidate(candidate, seed_orientation)
            for candidate in candidates
        ]
        valid_candidates = [candidate for candidate in evaluated if candidate["valid"]]
        if not valid_candidates:
            rospy.logwarn(
                "suggested object verification failed: no local_pick_place_preview candidate reached Cartesian fraction >= %.3f",
                self.min_cartesian_fraction,
            )
            return None

        verified = min(valid_candidates, key=lambda candidate: candidate["score"])
        rospy.loginfo(
            "suggested object verification success: selected_strategy=%s Cartesian fraction=%.3f trajectory_points=%d joint_motion=%.4f place_offset=(%.3f, %.3f)",
            verified["strategy"],
            verified["cartesian_fraction"],
            verified["trajectory_points"],
            verified["joint_motion"],
            verified["place_offset_xy"][0],
            verified["place_offset_xy"][1],
        )
        return verified

    def publish_suggested_object_marker(self, suggested_cube_center, verified_candidate):
        if not self.suggested_object_marker_enabled:
            return

        verified = verified_candidate is not None
        for marker_id in (30, 31):
            clear_marker = Marker()
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.header.frame_id = self.planning_frame
            clear_marker.ns = "hsv_suggested_object"
            clear_marker.id = marker_id
            clear_marker.action = Marker.DELETE
            self.suggested_object_marker_publisher.publish(clear_marker)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.planning_frame
        marker.ns = "hsv_suggested_object"
        marker.id = 30
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = suggested_cube_center.point.x
        marker.pose.position.y = suggested_cube_center.point.y
        marker.pose.position.z = suggested_cube_center.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size
        if verified:
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.25
            marker.color.a = 0.75
        else:
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.7
        self.suggested_object_marker_publisher.publish(marker)

        text_marker = self.make_debug_text_marker(
            suggested_cube_center,
            31,
            "hsv_suggested_object",
            "suggested_pick_ok" if verified else "suggested_pick_failed",
            verified,
        )
        text_marker.color.r = marker.color.r
        text_marker.color.g = marker.color.g
        text_marker.color.b = marker.color.b
        text_marker.color.a = 1.0
        self.suggested_object_marker_publisher.publish(text_marker)

        suggested_top_center = self.point_with_xyz_offset(
            suggested_cube_center,
            0.0,
            0.0,
            self.cube_size / 2.0,
        )
        rospy.loginfo(
            "suggested object marker published: topic=%s verified=%s marker.header.frame_id=%s marker.pose.position=(%.6f, %.6f, %.6f) cube_center=(%.6f, %.6f, %.6f) top_center=(%.6f, %.6f, %.6f)",
            self.suggested_object_marker_topic,
            verified,
            marker.header.frame_id,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            suggested_cube_center.point.x,
            suggested_cube_center.point.y,
            suggested_cube_center.point.z,
            suggested_top_center.point.x,
            suggested_top_center.point.y,
            suggested_top_center.point.z,
        )

    def publish_cube_center_debug_marker(self, cube_center, clear_existing=True):
        if clear_existing:
            clear_marker = Marker()
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.header.frame_id = self.planning_frame
            clear_marker.ns = "hsv_object"
            clear_marker.id = 20
            clear_marker.action = Marker.DELETE
            self.object_preview_marker_publisher.publish(clear_marker)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.planning_frame
        marker.ns = "hsv_object"
        marker.id = 20
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cube_center.point.x
        marker.pose.position.y = cube_center.point.y
        marker.pose.position.z = cube_center.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size
        marker.color.r = 0.0
        marker.color.g = 0.15
        marker.color.b = 1.0
        marker.color.a = 0.75
        self.object_preview_marker_publisher.publish(marker)
        rospy.loginfo(
            "object preview marker published: topic=%s marker_type=CUBE marker.header.frame_id=%s marker.pose.position=(%.6f, %.6f, %.6f)",
            self.object_preview_marker_topic,
            marker.header.frame_id,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        )

    def publish_selected_object_preview_marker(self, selected):
        for marker_id in (20, 21):
            clear_marker = Marker()
            clear_marker.header.stamp = rospy.Time.now()
            clear_marker.header.frame_id = self.planning_frame
            clear_marker.ns = "hsv_object"
            clear_marker.id = marker_id
            clear_marker.action = Marker.DELETE
            self.object_preview_marker_publisher.publish(clear_marker)

        if "cube_center" in selected:
            self.publish_cube_center_debug_marker(
                selected["cube_center"], clear_existing=False
            )

        if not self.attach_object_preview:
            return

        preview_points = []
        if "cube_center" in selected:
            preview_points.append(deepcopy(selected["cube_center"].point))
            preview_points.append(
                deepcopy(
                    self.point_with_xyz_offset(
                        selected["cube_center"],
                        0.0,
                        0.0,
                        self.local_lift_height,
                    ).point
                )
            )
        if "place_cube_center" in selected:
            preview_points.append(
                deepcopy(
                    self.point_with_xyz_offset(
                        selected["place_cube_center"],
                        0.0,
                        0.0,
                        self.local_lift_height,
                    ).point
                )
            )
            preview_points.append(deepcopy(selected["place_cube_center"].point))
        preview_points = (
            preview_points
            if preview_points
            else [
                deepcopy(selected[key].point)
                for key in (
                    "grasp_point",
                    "lift_point",
                    "place_lift_point",
                    "place_point",
                )
                if key in selected
            ]
        )
        if not preview_points:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.planning_frame
        marker.ns = "hsv_object"
        marker.id = 21
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size
        marker.color.r = 0.0
        marker.color.g = 0.15
        marker.color.b = 1.0
        marker.color.a = 0.45
        marker.points = preview_points
        self.object_preview_marker_publisher.publish(marker)
        rospy.loginfo(
            "object preview marker published: topic=%s marker_type=CUBE_LIST marker.header.frame_id=%s points=%d",
            self.object_preview_marker_topic,
            marker.header.frame_id,
            len(preview_points),
        )

    def publish_auto_candidate_markers(
        self,
        candidates,
        selected_strategy="",
        object_raw=None,
        selected_candidate_id="",
    ):
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.header.frame_id = self.planning_frame
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        base_colors = {
            "cube_center": (0.2, 0.4, 1.0, 0.45),
            "side_face_center": (0.0, 1.0, 1.0, 0.45),
            "pre_grasp": (1.0, 0.7, 0.0, 0.45),
            "approach_point": (1.0, 0.0, 1.0, 0.45),
            "grasp_point": (1.0, 1.0, 1.0, 0.55),
            "lift_point": (0.2, 1.0, 0.2, 0.45),
            "place_lift_point": (0.4, 1.0, 0.8, 0.45),
            "place_cube_center": (0.0, 0.2, 1.0, 0.45),
            "place_point": (1.0, 0.4, 0.0, 0.45),
            "retreat_point": (0.8, 0.8, 1.0, 0.45),
        }
        selected_colors = {
            "cube_center": (0.1, 0.4, 1.0, 1.0),
            "side_face_center": (0.0, 1.0, 0.5, 1.0),
            "pre_grasp": (1.0, 1.0, 0.0, 1.0),
            "approach_point": (0.0, 1.0, 0.0, 1.0),
            "grasp_point": (1.0, 1.0, 1.0, 1.0),
            "lift_point": (0.1, 1.0, 0.1, 1.0),
            "place_lift_point": (0.2, 1.0, 0.9, 1.0),
            "place_cube_center": (0.0, 0.15, 1.0, 1.0),
            "place_point": (1.0, 0.2, 0.0, 1.0),
            "retreat_point": (0.5, 0.5, 1.0, 1.0),
        }
        point_keys = (
            "cube_center",
            "side_face_center",
            "pre_grasp",
            "approach_point",
            "grasp_point",
            "lift_point",
            "place_lift_point",
            "place_cube_center",
            "place_point",
            "retreat_point",
        )
        stage_labels = {
            "cube_center": "cube_center",
            "side_face_center": "side_face_center",
            "pre_grasp": "pre_grasp",
            "approach_point": "approach_point",
            "grasp_point": "grasp_point",
            "lift_point": "lift_point",
            "place_lift_point": "place_lift",
            "place_cube_center": "place_cube_center",
            "place_point": "place_point",
            "retreat_point": "retreat_point",
        }

        marker_id = 1
        if object_raw is not None:
            markers.markers.append(
                self.make_debug_marker(
                    object_raw,
                    marker_id,
                    "grasp_debug_object_point_raw",
                    (0.0, 0.2, 1.0, 1.0),
                    self.auto_marker_scale * 1.4,
                    marker_type=Marker.CUBE,
                )
            )
            marker_id += 1
            markers.markers.append(
                self.make_debug_text_marker(
                    object_raw,
                    marker_id,
                    "grasp_debug_object_point_raw",
                    "object_point_raw",
                    True,
                )
            )
            marker_id += 1

        for candidate in candidates:
            strategy = candidate["strategy"]
            candidate_id = candidate.get("candidate_id", strategy)
            selected = (
                candidate_id == selected_candidate_id
                if selected_candidate_id
                else strategy == selected_strategy
            )
            namespace = "auto_any_face_{}".format(strategy)
            scale = self.auto_marker_scale * (1.6 if selected else 1.0)
            colors = selected_colors if selected else base_colors
            for key in point_keys:
                if key not in candidate:
                    continue
                marker_point = candidate.get("{}_raw".format(key), candidate[key])
                marker_type = (
                    Marker.CUBE
                    if key in ("cube_center", "place_cube_center")
                    else Marker.SPHERE
                )
                marker_scale = (
                    self.cube_size
                    if key in ("cube_center", "place_cube_center")
                    else scale
                )
                markers.markers.append(
                    self.make_debug_marker(
                        marker_point,
                        marker_id,
                        namespace,
                        colors[key],
                        marker_scale,
                        marker_type=marker_type,
                    )
                )
                marker_id += 1
                if selected:
                    markers.markers.append(
                        self.make_debug_text_marker(
                            marker_point,
                            marker_id,
                            "{}_stage_labels".format(namespace),
                            stage_labels[key],
                            selected,
                        )
                    )
                    marker_id += 1
            markers.markers.append(
                self.make_debug_text_marker(
                    candidate.get("approach_point_raw", candidate["approach_point"]),
                    marker_id,
                    namespace,
                    "{}{}".format(
                        "selected_strategy: " if selected else "",
                        strategy,
                    ),
                    selected,
                )
            )
            marker_id += 1
            if selected and self.gripper_preview_enabled:
                if "grasp_point" in candidate:
                    markers.markers.append(
                        self.make_debug_text_marker(
                            candidate.get("grasp_point_raw", candidate["grasp_point"]),
                            marker_id,
                            "{}_gripper_preview".format(namespace),
                            "gripper_close_preview",
                            True,
                        )
                    )
                    marker_id += 1
                if "place_point" in candidate:
                    markers.markers.append(
                        self.make_debug_text_marker(
                            candidate.get("place_point_raw", candidate["place_point"]),
                            marker_id,
                            "{}_gripper_preview".format(namespace),
                            "gripper_open_preview",
                            True,
                        )
                    )
                    marker_id += 1
            if selected and self.attach_object_preview:
                for key in (
                    "grasp_point",
                    "lift_point",
                    "place_lift_point",
                    "place_point",
                ):
                    if key not in candidate:
                        continue
                    marker_point = candidate[key]
                    markers.markers.append(
                        self.make_debug_marker(
                            marker_point,
                            marker_id,
                            "{}_object_preview".format(namespace),
                            (0.0, 0.15, 1.0, 0.35),
                            self.cube_size,
                            marker_type=Marker.CUBE,
                        )
                    )
                    marker_id += 1

        self.auto_marker_publisher.publish(markers)

    def publish_full_side_path_debug_markers(
        self, candidate, object_raw=None, selected=False
    ):
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.header.frame_id = self.planning_frame
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        marker_id = 1
        namespace = "full_side_path_debug"
        scale = self.auto_marker_scale * (1.4 if selected else 1.0)

        def add_point(point, name, color, marker_type=Marker.SPHERE, marker_scale=None):
            nonlocal marker_id
            markers.markers.append(
                self.make_debug_marker(
                    point,
                    marker_id,
                    namespace,
                    color,
                    marker_scale if marker_scale is not None else scale,
                    marker_type=marker_type,
                )
            )
            marker_id += 1
            markers.markers.append(
                self.make_debug_text_marker(
                    point,
                    marker_id,
                    "{}_labels".format(namespace),
                    name,
                    selected,
                )
            )
            marker_id += 1

        if object_raw is not None:
            add_point(
                object_raw,
                "object_point_raw_top_center",
                (0.0, 0.25, 1.0, 1.0),
                marker_type=Marker.CUBE,
                marker_scale=self.auto_marker_scale * 1.4,
            )

        add_point(
            candidate["cube_center"],
            "cube_center",
            (0.1, 0.4, 1.0, 0.65),
            marker_type=Marker.CUBE,
            marker_scale=self.cube_size,
        )
        add_point(
            candidate["side_face_center"],
            "side_face_center",
            (0.0, 1.0, 1.0, 0.8),
        )

        debug_points = (
            (
                "pre_grasp_raw",
                candidate["pre_grasp_raw"],
                (1.0, 0.45, 0.0, 0.75),
            ),
            (
                "pre_grasp_clamped",
                candidate["pre_grasp_clamped"],
                (1.0, 1.0, 0.0, 1.0),
            ),
            (
                "pre_grasp_planning",
                candidate["pre_grasp"],
                (1.0, 1.0, 1.0, 1.0),
            ),
            (
                "approach_point_raw",
                candidate["approach_point_raw"],
                (1.0, 0.0, 1.0, 0.75),
            ),
            (
                "approach_point_clamped",
                candidate["approach_point_clamped"],
                (0.0, 1.0, 0.0, 1.0),
            ),
            (
                "approach_point_planning",
                candidate["approach_point"],
                (0.3, 1.0, 1.0, 1.0),
            ),
        )
        for name, point, color in debug_points:
            add_point(point, name, color)

        if "cartesian_fraction" in candidate:
            fraction_text = "fraction={:.3f} points={}".format(
                candidate.get("cartesian_fraction", 0.0),
                candidate.get("trajectory_points", 0),
            )
            markers.markers.append(
                self.make_debug_text_marker(
                    candidate["approach_point"],
                    marker_id,
                    "{}_result".format(namespace),
                    fraction_text,
                    selected,
                )
            )

        self.auto_marker_publisher.publish(markers)

    def execution_allowed(self):
        if not (self.execute and self.confirm):
            return False
        if not self.gripper_execute:
            rospy.logwarn(
                "execute=true and confirm=true, but gripper_execute=false; refusing real robot execution in this grasp preview node."
            )
            return False
        if not self.require_external_control_for_execute:
            return True
        try:
            running = rospy.wait_for_message(
                self.robot_program_topic, Bool, timeout=2.0
            )
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Cannot confirm right arm External Control on %s: %s; not executing",
                self.robot_program_topic,
                exc,
            )
            return False
        if not running.data:
            rospy.logwarn("Right arm External Control is not running; not executing")
            return False
        return True

    def plan_to_filtered_object(self):
        object_xyz = self.median_object_point()
        object_input = self.make_object_point(object_xyz)
        object_base = self.transform_to_planning_frame(object_input)
        cube_center_base = self.stage_cube_center(object_base)
        if cube_center_base is None:
            return False
        self.publish_cube_center_debug_marker(cube_center_base)

        if self.grasp_stage == "full_side_path_debug":
            return self.plan_full_side_path_debug(
                object_input, object_base, cube_center_base
            )

        if self.grasp_stage == "local_pick_place_preview":
            return self.plan_local_pick_place_preview(
                object_input, object_base, cube_center_base
            )

        if self.grasp_stage in (
            "top_down_pick_preview",
            "top_down_pick_place_preview",
        ):
            return self.plan_top_down_pick_preview(
                object_input, object_base, cube_center_base
            )

        if self.grasp_stage == "full_pick_place_preview":
            return self.plan_pick_place_preview(
                object_input, object_base, cube_center_base
            )

        if (
            self.grasp_stage == "full_side_path"
            and self.grasp_strategy == "auto_any_face"
        ):
            return self.plan_auto_any_face(object_input, object_base)

        grasp_target_base = self.make_grasp_target(cube_center_base)

        if self.grasp_stage == "full_side_path":
            return self.plan_full_side_path(
                object_input, object_base, cube_center_base
            )

        stage_target_before_safety_base, stage_target_base = self.make_stage_target(
            grasp_target_base
        )
        stage_target_label = (
            "pre_grasp"
            if self.grasp_stage == "side_grasp_prepare"
            else "side_approach target"
        )

        rospy.loginfo(
            "object_point in %s: x=%.6f y=%.6f z=%.6f",
            self.input_frame,
            object_input.point.x,
            object_input.point.y,
            object_input.point.z,
        )
        rospy.loginfo(
            "object_point transformed to %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            object_base.point.x,
            object_base.point.y,
            object_base.point.z,
        )
        rospy.loginfo("grasp_stage: %s", self.grasp_stage)
        rospy.loginfo("grasp_strategy: %s", self.grasp_strategy)
        rospy.loginfo("approach_distance: %.6f", self.approach_distance)
        rospy.loginfo("safe_min_z: %.6f", self.safe_min_z)
        if self.grasp_strategy != "top_down":
            approach_direction = self.approach_motion_direction()
            rospy.loginfo(
                "side approach motion direction in %s: x=%.1f y=%.1f z=%.1f",
                self.planning_frame,
                approach_direction[0],
                approach_direction[1],
                approach_direction[2],
            )
        rospy.loginfo("object_z_check_enabled: %s", self.object_z_check_enabled)
        if self.object_z_check_enabled:
            rospy.loginfo(
                "object z range in %s: min=%.6f max=%.6f",
                self.planning_frame,
                self.object_min_z,
                self.object_max_z,
            )
        rospy.loginfo(
            "object_to_grasp_offset in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            self.object_to_grasp_offset_x,
            self.object_to_grasp_offset_y,
            self.object_to_grasp_offset_z,
        )
        rospy.loginfo(
            "grasp target in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            grasp_target_base.point.x,
            grasp_target_base.point.y,
            grasp_target_base.point.z,
        )
        rospy.loginfo(
            "%s before safety clamp in %s: x=%.6f y=%.6f z=%.6f",
            stage_target_label,
            self.planning_frame,
            stage_target_before_safety_base.point.x,
            stage_target_before_safety_base.point.y,
            stage_target_before_safety_base.point.z,
        )
        rospy.loginfo(
            "%s after safety clamp in %s: x=%.6f y=%.6f z=%.6f",
            stage_target_label,
            self.planning_frame,
            stage_target_base.point.x,
            stage_target_base.point.y,
            stage_target_base.point.z,
        )
        rospy.loginfo("approach_height: %.3f", self.approach_height)
        rospy.loginfo("z_offset_frame: %s", self.z_offset_frame)
        rospy.loginfo("group_name: %s", self.group_name)
        rospy.loginfo("planning_frame: %s", self.group.get_planning_frame())
        rospy.loginfo("end_effector_link: %s", self.end_effector_link)
        rospy.loginfo("planning_mode: %s", self.planning_mode)
        rospy.loginfo(
            "execute flag: execute=%s confirm=%s effective_execute=%s",
            self.execute,
            self.confirm,
            self.execute and self.confirm,
        )

        if not self.object_depth_is_safe(object_base):
            rospy.logwarn(
                "planning result: skipped because object depth/safety check failed"
            )
            return False
        if not self.tcp_target_z_is_safe(
            stage_target_base, stage_target_label, self.grasp_strategy
        ):
            rospy.logwarn(
                "planning result: skipped because %s is below safe_min_z",
                stage_target_label,
            )
            return False

        self.group.set_start_state_to_current_state()
        if self.grasp_stage == "side_approach":
            rospy.logwarn(
                "side_approach plans a Cartesian approach from the current TCP pose; run side_grasp_prepare first before executing this stage."
            )
        target_position, success, trajectory, point_count, cartesian_fraction = (
            self.plan_to_stage_target(stage_target_base)
        )
        rospy.loginfo(
            "target position in %s: x=%.6f y=%.6f z=%.6f",
            self.planning_frame,
            target_position[0],
            target_position[1],
            target_position[2],
        )
        if cartesian_fraction is not None:
            rospy.loginfo(
                "side_approach Cartesian fraction: %.3f required>=%.3f",
                cartesian_fraction,
                self.side_approach_min_fraction,
            )
        rospy.loginfo(
            "planning result: stage=%s success=%s trajectory_points=%d",
            self.grasp_stage,
            success,
            point_count,
        )

        if not success:
            return False

        self.publish_display_trajectory(trajectory)
        if self.execution_allowed():
            rospy.logwarn(
                "Executing APPROACH TEST only: moving to %s, no descent, no gripper, no grasp.",
                stage_target_label,
            )
            self.wait_before_execution()
            if not self.group.execute(trajectory, wait=True):
                self.group.stop()
                raise RuntimeError(
                    "MoveIt execution to {} failed".format(stage_target_label)
                )
            self.group.stop()
            rospy.loginfo("Approach test execution finished at %s.", stage_target_label)
        else:
            rospy.loginfo("PLAN ONLY: trajectory was not executed.")
        return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_arm_hsv_approach_test")
    RightArmHsvApproachTest()
    rospy.spin()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
