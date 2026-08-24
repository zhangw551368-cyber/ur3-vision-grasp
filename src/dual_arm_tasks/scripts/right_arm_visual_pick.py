#!/usr/bin/python3

import argparse
import math
import sys
import time

import cv2
import moveit_commander
import numpy as np
import rospy
import tf.transformations
import tf2_geometry_msgs
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from ur_dashboard_msgs.srv import IsProgramRunning


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plan or execute a guarded right-arm visual pick of the red block."
    )
    parser.add_argument("--config", required=True, help="Task YAML file.")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Allow motion execution and gripper commands. Default is plan-only.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the typed EXECUTE confirmation when --execute is used.",
    )
    parser.add_argument(
        "--phase",
        choices=("full", "approach", "grasp", "retreat"),
        default="full",
        help="Run the complete cycle, stop at hover, or continue from a verified hover.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def make_gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = position
    command.rSP = speed
    command.rFR = force
    return command


class RightArmVisualPick:
    def __init__(self, config, execute):
        self.config = config
        self.execute = execute
        self.arm = moveit_commander.MoveGroupCommander(config["arm_group"])
        self.arm.set_pose_reference_frame(config.get("planning_frame", "base"))
        self.arm.set_end_effector_link(config.get("end_effector_link", "right_arm_tool0"))
        self.arm.set_max_velocity_scaling_factor(config["velocity_scaling"])
        self.arm.set_max_acceleration_scaling_factor(config["acceleration_scaling"])
        self.arm.set_planning_time(config["planning_time"])
        self.arm.set_num_planning_attempts(config.get("num_planning_attempts", 5))
        if config.get("planner_id"):
            self.arm.set_planner_id(config["planner_id"])
        self.arm.set_goal_position_tolerance(config.get("goal_position_tolerance", 0.01))
        self.arm.set_goal_orientation_tolerance(
            config.get("goal_orientation_tolerance", 0.08)
        )

        self.gripper = rospy.Publisher(
            config["gripper_topic"], Robotiq2FGripper_robot_output, queue_size=1
        )
        self.gripper_preview = rospy.Publisher(
            config.get("gripper_preview_topic", "/right_gripper/preview_command"),
            Robotiq2FGripper_robot_output,
            queue_size=1,
        )
        self.display = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )
        if "orientation_quaternion" in config:
            self.orientation = config["orientation_quaternion"]
        else:
            self.orientation = tf.transformations.quaternion_from_euler(
                *config["orientation_rpy"]
            )
        self.pose_target_is_tcp = bool(config.get("pose_target_is_tcp", False))
        self.tcp_offset_from_end_effector = [
            float(value)
            for value in config.get("tcp_offset_from_end_effector", [0.0, 0.0, 0.0])
        ]
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.virtual_start_state = None
        self.display_start_state = None
        self.display_trajectories = []
        configured_initial = config.get("initial_joint_target")
        if configured_initial is None:
            configured_initial = self.arm.get_current_joint_values()
        if len(configured_initial) != len(self.arm.get_active_joints()):
            raise RuntimeError(
                "initial_joint_target must contain exactly {} joints".format(
                    len(self.arm.get_active_joints())
                )
            )
        self.start_joint_target = dict(
            zip(self.arm.get_active_joints(), [float(v) for v in configured_initial])
        )
        self.grasp_confirmed = False
        self.object_released = False

    @staticmethod
    def distance(first, second):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    @staticmethod
    def rotation_matrix(quaternion):
        x, y, z, w = quaternion
        return np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                ],
            ]
        )

    def roi_bounds(self, image_shape):
        height, width = image_shape[:2]
        x_min = max(0, min(width, int(self.config.get("roi_x_min", 0))))
        y_min = max(0, min(height, int(self.config.get("roi_y_min", 0))))
        x_max = int(self.config.get("roi_x_max", 0)) or width
        y_max = int(self.config.get("roi_y_max", 0)) or height
        x_max = max(x_min, min(width, x_max))
        y_max = max(y_min, min(height, y_max))
        return x_min, y_min, x_max, y_max

    def apply_roi(self, mask, image_shape):
        if not self.config.get("use_roi", False):
            return mask
        x_min, y_min, x_max, y_max = self.roi_bounds(image_shape)
        limited = np.zeros_like(mask)
        limited[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
        return limited

    @staticmethod
    def candidate_score(area, width, height, fill_ratio):
        aspect = float(width) / float(height)
        aspect_score = 1.0 / (1.0 + abs(np.log(max(aspect, 1e-6))))
        return area * (0.7 + fill_ratio) * aspect_score

    def select_red_block_candidate(self, contours):
        candidates = []
        rejected = 0
        min_area = self.config.get("min_area_pixels", 500)
        max_area = self.config.get("max_area_pixels", 0)
        min_width = self.config.get("min_bbox_width_pixels", 0)
        max_width = self.config.get("max_bbox_width_pixels", 0)
        min_height = self.config.get("min_bbox_height_pixels", 0)
        max_height = self.config.get("max_bbox_height_pixels", 0)
        min_aspect = self.config.get("min_aspect_ratio", 0.0)
        max_aspect = self.config.get("max_aspect_ratio", 0.0)
        min_fill = self.config.get("min_fill_ratio", 0.0)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                rejected += 1
                continue
            if max_area and area > max_area:
                rejected += 1
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if min_width and width < min_width:
                rejected += 1
                continue
            if max_width and width > max_width:
                rejected += 1
                continue
            if min_height and height < min_height:
                rejected += 1
                continue
            if max_height and height > max_height:
                rejected += 1
                continue
            aspect = float(width) / float(height)
            if min_aspect and aspect < min_aspect:
                rejected += 1
                continue
            if max_aspect and aspect > max_aspect:
                rejected += 1
                continue
            fill_ratio = float(area) / float(max(1, width * height))
            if min_fill and fill_ratio < min_fill:
                rejected += 1
                continue
            candidates.append(
                (
                    self.candidate_score(area, width, height, fill_ratio),
                    contour,
                    (x, y, width, height, area, fill_ratio),
                )
            )
        if not candidates:
            raise RuntimeError(
                "Red block contour was not found after filtering: contours={} rejected={}".format(
                    len(contours), rejected
                )
            )
        return max(candidates, key=lambda candidate: candidate[0])

    def estimate_visual_grasp_center(self):
        color_topic = self.config.get("color_topic", "/camera/color/image_raw")
        depth_topic = self.config.get(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        info_topic = self.config.get("camera_info_topic", "/camera/color/camera_info")
        timeout = self.config.get("image_timeout", 3.0)
        color_msg = rospy.wait_for_message(color_topic, Image, timeout=timeout)
        depth_msg = rospy.wait_for_message(depth_topic, Image, timeout=timeout)
        info = rospy.wait_for_message(info_topic, CameraInfo, timeout=timeout)
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = np.asarray(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"),
            dtype=np.float64,
        )
        if depth_msg.encoding in ("16UC1", "mono16"):
            depth *= 0.001

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        low_1 = np.array(self.config.get("red_low_1", [0, 80, 15]), dtype=np.uint8)
        high_1 = np.array(self.config.get("red_high_1", [15, 255, 255]), dtype=np.uint8)
        low_2 = np.array(self.config.get("red_low_2", [165, 80, 15]), dtype=np.uint8)
        high_2 = np.array(self.config.get("red_high_2", [180, 255, 255]), dtype=np.uint8)
        mask = cv2.inRange(hsv, low_1, high_1) | cv2.inRange(hsv, low_2, high_2)
        mask = self.apply_roi(mask, color.shape)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, contour, stats = self.select_red_block_candidate(contours)
        x, y, width, height, area, fill_ratio = stats
        rospy.loginfo(
            "Selected red block contour bbox=[%d,%d,%d,%d] area=%.0f fill=%.2f",
            x,
            y,
            width,
            height,
            area,
            fill_ratio,
        )
        component = np.zeros_like(mask)
        cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
        component = cv2.erode(component, np.ones((3, 3), np.uint8), iterations=1)
        rows, cols = np.nonzero((component > 0) & np.isfinite(depth) & (depth > 0))
        if len(rows) < self.config.get("min_depth_samples", 300):
            raise RuntimeError("Not enough valid red-block depth pixels: {}".format(len(rows)))

        z = depth[rows, cols]
        fx, fy = info.K[0], info.K[4]
        cx, cy = info.K[2], info.K[5]
        points_camera = np.stack(
            ((cols - cx) * z / fx, (rows - cy) * z / fy, z),
            axis=1,
        )
        transform = self.tf_buffer.lookup_transform(
            self.config.get("planning_frame", "base"),
            info.header.frame_id,
            rospy.Time(0),
            rospy.Duration(timeout),
        ).transform
        rotation = self.rotation_matrix(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        )
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        points_base = (rotation @ points_camera.T).T + translation

        bounds_quantile = self.config.get("visible_bounds_quantile", 0.02)
        lower = np.quantile(points_base, bounds_quantile, axis=0)
        upper = np.quantile(points_base, 1.0 - bounds_quantile, axis=0)
        visible_span = upper - lower
        span_limits = self.config.get("visible_span_limits", {})
        axis_names = ("x", "y", "z")
        span_violations = []
        for index, axis in enumerate(axis_names):
            limit = span_limits.get(axis)
            if limit is not None and visible_span[index] > limit:
                span_violations.append(
                    "{} span {:.3f}m > {:.3f}m".format(
                        axis, visible_span[index], limit
                    )
                )
        if span_violations:
            raise RuntimeError(
                "Red-block depth cloud span is too large: {}".format(
                    ", ".join(span_violations)
                )
            )
        top_threshold = np.quantile(
            points_base[:, 2], self.config.get("top_face_quantile", 0.75)
        )
        top_points = points_base[points_base[:, 2] >= top_threshold]
        if len(top_points) < self.config.get("min_top_face_samples", 80):
            raise RuntimeError("Not enough red-block top-face pixels: {}".format(len(top_points)))
        top_bounds_quantile = self.config.get("top_bounds_quantile", 0.05)
        top_lower = np.quantile(top_points, top_bounds_quantile, axis=0)
        top_upper = np.quantile(top_points, 1.0 - top_bounds_quantile, axis=0)
        xyz = [
            float((top_lower[0] + top_upper[0]) / 2.0),
            float((top_lower[1] + top_upper[1]) / 2.0),
            float((lower[2] + upper[2]) / 2.0),
        ]
        offset = self.config.get("target_offset", [0.0, 0.0, 0.0])
        xyz = [xyz[index] + offset[index] for index in range(3)]
        rospy.loginfo(
            "Visual red block center xyz=[%.3f, %.3f, %.3f], "
            "visible bounds x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f], "
            "pixels=%d top_pixels=%d",
            xyz[0],
            xyz[1],
            xyz[2],
            lower[0],
            upper[0],
            lower[1],
            upper[1],
            lower[2],
            upper[2],
            len(points_base),
            len(top_points),
        )
        return xyz

    def wait_for_stable_target(self, expected=None):
        sample_count = self.config.get("target_sample_count", 5)
        timeout = self.config.get("target_timeout", 10.0)
        deadline = time.time() + timeout
        samples = []
        target_source = self.config.get("target_source", "contour_depth")
        if target_source not in ("contour_depth", "point_topic"):
            raise RuntimeError(
                "target_source must be 'contour_depth' or 'point_topic', got {!r}".format(
                    target_source
                )
            )
        rospy.loginfo(
            "Waiting for %d stable %s target estimates", sample_count, target_source
        )
        while len(samples) < sample_count and time.time() < deadline:
            try:
                if target_source == "point_topic":
                    candidate = self.estimate_point_topic_target()
                else:
                    candidate = self.estimate_visual_grasp_center()
                self.validate_target_bounds(candidate)
                samples.append(candidate)
            except (RuntimeError, rospy.ROSException, tf2_ros.TransformException) as exc:
                rospy.logwarn_throttle(1.0, "Waiting for visual grasp center: %s", exc)
                continue
        if len(samples) < sample_count:
            raise RuntimeError("Red block target was not visible long enough")

        xyz = [
            sum(sample[index] for sample in samples) / len(samples)
            for index in range(3)
        ]
        spread = max(self.distance(sample, xyz) for sample in samples)
        max_spread = self.config.get("target_max_spread", 0.012)
        if spread > max_spread:
            raise RuntimeError(
                "Red block target is unstable: spread {:.3f}m exceeds {:.3f}m".format(
                    spread, max_spread
                )
            )
        if expected is not None:
            shift = self.distance(xyz, expected)
            max_shift = self.config.get("target_max_shift", 0.025)
            if shift > max_shift:
                raise RuntimeError(
                    "Red block moved before descent: shift {:.3f}m exceeds {:.3f}m".format(
                        shift, max_shift
                    )
                )
        rospy.loginfo(
            "Stable contour-depth red block in %s: x=%.3f y=%.3f z=%.3f spread=%.3fm",
            self.config.get("planning_frame", "base"),
            xyz[0],
            xyz[1],
            xyz[2],
            spread,
        )
        self.validate_target_bounds(xyz)
        return xyz

    def estimate_point_topic_target(self):
        """Read a live PointStamped target and convert it to a grasp reference.

        The HSV synchronizer publishes a non-latched, validated point.  Keeping
        this conversion here lets the existing guarded executor reuse its
        staged motion, External Control check, and Robotiq grasp verification.
        """
        topic = self.config.get("target_topic", "/hsv_grasp/stable_object_point")
        timeout = float(self.config.get("point_topic_timeout", 2.0))
        message = rospy.wait_for_message(topic, PointStamped, timeout=timeout)
        if not message.header.frame_id:
            raise RuntimeError("PointStamped target has an empty frame_id")

        planning_frame = self.config.get("planning_frame", "base")
        if message.header.frame_id != planning_frame:
            stamp = message.header.stamp
            if stamp == rospy.Time(0):
                stamp = rospy.Time(0)
            transform = self.tf_buffer.lookup_transform(
                planning_frame,
                message.header.frame_id,
                stamp,
                rospy.Duration(float(self.config.get("tf_timeout", 0.5))),
            )
            message = tf2_geometry_msgs.do_transform_point(message, transform)

        xyz = [message.point.x, message.point.y, message.point.z]
        semantic = self.config.get("point_topic_semantic", "top_center")
        cube_size = float(self.config.get("cube_size", 0.055))
        if semantic == "top_center":
            xyz[2] -= cube_size / 2.0
        elif semantic not in ("center", "cube_center", "grasp_reference"):
            raise RuntimeError("Unsupported point_topic_semantic={!r}".format(semantic))

        offset = self.config.get("target_offset", [0.0, 0.0, 0.0])
        if len(offset) != 3:
            raise RuntimeError("target_offset must contain exactly three values")
        xyz = [xyz[index] + float(offset[index]) for index in range(3)]
        rospy.loginfo(
            "Live point target in %s: xyz=[%.6f, %.6f, %.6f] semantic=%s offset=%s",
            planning_frame,
            xyz[0],
            xyz[1],
            xyz[2],
            semantic,
            offset,
        )
        return xyz

    def validate_target_bounds(self, xyz):
        bounds = self.config.get("target_bounds")
        if not bounds:
            return
        axis_names = ("x", "y", "z")
        violations = []
        for index, axis in enumerate(axis_names):
            lower, upper = bounds.get(axis, (None, None))
            if lower is not None and xyz[index] < lower:
                violations.append("{}={:.3f} < {:.3f}".format(axis, xyz[index], lower))
            if upper is not None and xyz[index] > upper:
                violations.append("{}={:.3f} > {:.3f}".format(axis, xyz[index], upper))
        if violations:
            raise RuntimeError(
                "Red block target is outside the allowed planning workspace: {}".format(
                    ", ".join(violations)
                )
            )

    def current_xyz(self):
        current = self.arm.get_current_pose().pose
        xyz = [current.position.x, current.position.y, current.position.z]
        if self.pose_target_is_tcp:
            quaternion = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w,
            ]
            rotation = tf.transformations.quaternion_matrix(quaternion)
            offset = self.tcp_offset_from_end_effector
            rotated_offset = [
                rotation[row][0] * offset[0]
                + rotation[row][1] * offset[1]
                + rotation[row][2] * offset[2]
                for row in range(3)
            ]
            xyz = [xyz[index] + rotated_offset[index] for index in range(3)]
        return xyz

    @staticmethod
    def add(first, second):
        return [a + b for a, b in zip(first, second)]

    @staticmethod
    def scale(vector, factor):
        return [value * factor for value in vector]

    def approach_vector(self):
        vector = self.config.get("approach_vector", [0.0, 1.0, 0.0])
        norm = math.sqrt(sum(value * value for value in vector))
        if norm == 0.0:
            raise RuntimeError("approach_vector cannot be zero")
        return [value / norm for value in vector]

    def grasp_poses(self, target):
        approach = self.approach_vector()
        tool_to_center = self.config.get("tool_to_grasp_center", 0.130)
        clearance = self.config.get("pre_grasp_clearance", 0.08)
        grasp = self.add(target, self.scale(approach, -tool_to_center))
        pre_grasp = self.add(grasp, self.scale(approach, -clearance))
        lift = self.add(grasp, self.config.get("post_grasp_lift", [0.0, 0.0, 0.03]))
        shift = self.add(lift, self.config.get("post_grasp_shift", [0.0, -0.02, 0.0]))
        return pre_grasp, grasp, lift, shift

    def require_hover(self, expected):
        if not self.execute:
            return
        actual = self.current_xyz()
        error = self.distance(actual, expected)
        tolerance = self.config.get("hover_position_tolerance", 0.035)
        if error > tolerance:
            raise RuntimeError(
                "Right arm is not at verified hover: actual={} expected={} error={:.3f}m".format(
                    [round(value, 3) for value in actual],
                    [round(value, 3) for value in expected],
                    error,
                )
            )
        rospy.loginfo("Verified hover position error=%.3fm", error)

    def ensure_external_control(self):
        if not self.execute:
            return
        topic = self.config.get(
            "robot_program_topic",
            "/right_arm/ur_hardware_interface/robot_program_running",
        )
        try:
            running = rospy.wait_for_message(topic, Bool, timeout=2.0)
        except rospy.ROSException as exc:
            raise RuntimeError("No External Control feedback on {}: {}".format(topic, exc))
        if not running.data:
            raise RuntimeError("External Control is not running on the right arm")

        # The driver topic can remain True briefly (or across a reconnect) even
        # after the pendant program has been paused.  Query the dashboard as an
        # independent, live interlock before allowing any real trajectory.
        dashboard_service = self.config.get(
            "dashboard_program_running_service",
            "/right_arm/ur_hardware_interface/dashboard/program_running",
        )
        try:
            rospy.wait_for_service(dashboard_service, timeout=2.0)
            dashboard = rospy.ServiceProxy(
                dashboard_service, IsProgramRunning, persistent=False
            )
            dashboard_state = dashboard()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise RuntimeError(
                "Cannot verify External Control through {}: {}".format(
                    dashboard_service, exc
                )
            )
        if not dashboard_state.success or not dashboard_state.program_running:
            raise RuntimeError(
                "External Control dashboard state is not running: success={} "
                "program_running={} answer={!r}".format(
                    dashboard_state.success,
                    dashboard_state.program_running,
                    dashboard_state.answer,
                )
            )
        rospy.loginfo(
            "External Control confirmed by %s and dashboard", topic
        )

    def pose(self, xyz):
        xyz = list(xyz)
        if self.pose_target_is_tcp:
            xyz = self.end_effector_position_from_tcp(xyz)
        target = PoseStamped()
        target.header.frame_id = self.config.get("planning_frame", "base")
        target.header.stamp = rospy.Time.now()
        target.pose = Pose()
        target.pose.position.x = xyz[0]
        target.pose.position.y = xyz[1]
        target.pose.position.z = xyz[2]
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = self.orientation
        return target

    def end_effector_position_from_tcp(self, tcp_xyz):
        rotation = tf.transformations.quaternion_matrix(self.orientation)
        offset = self.tcp_offset_from_end_effector
        rotated_offset = [
            rotation[row][0] * offset[0]
            + rotation[row][1] * offset[1]
            + rotation[row][2] * offset[2]
            for row in range(3)
        ]
        return [tcp_xyz[index] - rotated_offset[index] for index in range(3)]

    def current_tcp_position(self):
        current = self.arm.get_current_pose().pose
        end_effector = np.asarray(
            [current.position.x, current.position.y, current.position.z],
            dtype=float,
        )
        if not self.pose_target_is_tcp:
            return end_effector
        quaternion = [
            current.orientation.x, current.orientation.y,
            current.orientation.z, current.orientation.w,
        ]
        rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        return end_effector + rotation.dot(
            np.asarray(self.tcp_offset_from_end_effector, dtype=float)
        )

    def verify_reached_tcp(self, name, requested_xyz):
        if not self.execute:
            return
        rospy.sleep(0.15)
        actual = self.current_tcp_position()
        requested = np.asarray(requested_xyz, dtype=float)
        error = float(np.linalg.norm(actual - requested))
        limit = float(self.config.get("actual_tcp_max_error", 0.012))
        # "pre_grasp" is a high transit pose and uses the general tolerance.
        # Only the final blockN_grasp pose uses the tighter closing tolerance.
        if name.startswith("block") and name.endswith("_grasp"):
            limit = float(
                self.config.get("actual_grasp_tcp_max_error", limit)
            )
        rospy.loginfo(
            "Reached %-14s requested=%s actual_tcp=%s error=%.2fmm limit=%.2fmm",
            name,
            np.round(requested, 5).tolist(),
            np.round(actual, 5).tolist(),
            error * 1000.0,
            limit * 1000.0,
        )
        if error > limit:
            self.arm.stop()
            raise RuntimeError(
                "Actual TCP error at {} is {:.1f}mm > {:.1f}mm".format(
                    name, error * 1000.0, limit * 1000.0
                )
            )

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

    def plan_to_pose(self, name, xyz):
        target = self.pose(xyz)
        trajectory = None
        retries = max(1, int(self.config.get("plan_retries", 3)))
        max_duration = float(self.config.get("max_pose_duration", 0.0))
        max_joint_motion = float(self.config.get("max_pose_joint_motion", 0.0))
        if name == "pre_grasp":
            max_duration = float(
                self.config.get("max_pregrasp_duration", max_duration)
            )
            max_joint_motion = float(
                self.config.get("max_pregrasp_joint_motion", max_joint_motion)
            )
        for attempt in range(1, retries + 1):
            if self.virtual_start_state is not None:
                self.arm.set_start_state(self.virtual_start_state)
            else:
                self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(target)
            plan_result = self.arm.plan()
            candidate = plan_result[1] if isinstance(plan_result, tuple) else plan_result
            self.arm.clear_pose_targets()
            if candidate.joint_trajectory.points:
                points = candidate.joint_trajectory.points
                duration = points[-1].time_from_start.to_sec()
                joint_motion = sum(
                    abs(current - previous)
                    for first, second in zip(points, points[1:])
                    for previous, current in zip(first.positions, second.positions)
                )
                if max_duration > 0.0 and duration > max_duration:
                    rospy.logwarn(
                        "Rejected %s plan %d/%d: duration %.2fs > %.2fs",
                        name, attempt, retries, duration, max_duration,
                    )
                    continue
                if (
                    max_joint_motion > 0.0
                    and joint_motion > max_joint_motion
                ):
                    rospy.logwarn(
                        "Rejected %s plan %d/%d: joint motion %.3frad > %.3frad",
                        name, attempt, retries, joint_motion, max_joint_motion,
                    )
                    continue
                trajectory = candidate
                break
            rospy.logwarn("No valid MoveIt plan for %s on attempt %d/%d", name, attempt, retries)

        if trajectory is None:
            raise RuntimeError("No valid MoveIt plan for {}".format(name))

        rospy.loginfo(
            "Planned %-10s xyz=[%.3f, %.3f, %.3f] with %d points duration=%.2fs joint_motion=%.3frad",
            name,
            xyz[0],
            xyz[1],
            xyz[2],
            len(trajectory.joint_trajectory.points),
            trajectory.joint_trajectory.points[-1].time_from_start.to_sec(),
            sum(
                abs(current - previous)
                for first, second in zip(
                    trajectory.joint_trajectory.points,
                    trajectory.joint_trajectory.points[1:],
                )
                for previous, current in zip(first.positions, second.positions)
            ),
        )
        self.publish_display_trajectory(trajectory)

        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                raise RuntimeError("Execution failed at {}".format(name))
            self.arm.stop()
            self.verify_reached_tcp(name, xyz)
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

    def publish_display_trajectory(self, trajectory):
        if self.display_start_state is None:
            self.display_start_state = self.arm.get_current_state()
        self.display_trajectories.append(trajectory)
        display = DisplayTrajectory()
        display.trajectory_start = self.display_start_state
        display.trajectory.extend(self.display_trajectories)
        self.display.publish(display)
        time.sleep(self.config.get("display_pause_seconds", 1.0))

    def plan_to_initial_joint_target(self, name="return_to_initial"):
        self.arm.stop()
        self.arm.clear_pose_targets()
        if self.virtual_start_state is not None:
            self.arm.set_start_state(self.virtual_start_state)
        else:
            self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(self.start_joint_target)
        plan_result = self.arm.plan()
        trajectory = plan_result[1] if isinstance(plan_result, tuple) else plan_result
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("No valid MoveIt plan for {}".format(name))

        rospy.loginfo(
            "Planned %-16s with %d points target=%s",
            name,
            len(trajectory.joint_trajectory.points),
            [round(self.start_joint_target[j], 6) for j in self.arm.get_active_joints()],
        )
        self.publish_display_trajectory(trajectory)
        if self.execute:
            self.ensure_external_control()
            if not self.arm.execute(trajectory, wait=True):
                self.arm.stop()
                raise RuntimeError("Execution failed at {}".format(name))
            self.arm.stop()
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

    def cartesian_to_pose(self, name, xyz):
        target = self.pose(xyz)
        if self.virtual_start_state is not None:
            self.arm.set_start_state(self.virtual_start_state)
            trajectory_start = self.virtual_start_state
        else:
            self.arm.set_start_state_to_current_state()
            trajectory_start = self.arm.get_current_state()
        trajectory, fraction = self.arm.compute_cartesian_path(
            [target.pose],
            self.config.get("cartesian_step", 0.005),
            avoid_collisions=True,
        )
        min_fraction = self.config.get("cartesian_min_fraction", 0.995)
        if fraction < min_fraction or not trajectory.joint_trajectory.points:
            raise RuntimeError(
                "Incomplete Cartesian path for {}: {:.1%}".format(name, fraction)
            )

        rospy.loginfo(
            "Planned Cartesian %-10s xyz=[%.3f, %.3f, %.3f] with %d points",
            name,
            xyz[0],
            xyz[1],
            xyz[2],
            len(trajectory.joint_trajectory.points),
        )
        if self.display_start_state is None:
            self.display_start_state = trajectory_start
        self.publish_display_trajectory(trajectory)

        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                self.arm.stop()
                raise RuntimeError("Execution failed at {}".format(name))
            self.arm.stop()
            self.verify_reached_tcp(name, xyz)
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

    def publish_gripper(self, position, label):
        rospy.loginfo("Gripper command: %s rPR=%d", label, position)
        if not self.execute:
            self.gripper_preview.publish(
                make_gripper_command(
                    position,
                    self.config.get("gripper_speed", 80),
                    self.config.get("gripper_force", 80),
                )
            )
            return
        deadline = time.time() + 5.0
        while self.gripper.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.1)
        if self.gripper.get_num_connections() == 0:
            raise RuntimeError(
                "No subscriber connected to gripper topic: {}".format(
                    self.config["gripper_topic"]
                )
            )
        self.gripper.publish(
            make_gripper_command(
                position,
                self.config.get("gripper_speed", 80),
                self.config.get("gripper_force", 80),
            )
        )
        time.sleep(self.config.get("gripper_settle_seconds", 1.0))

    def require_grasp(self):
        if not self.execute or not self.config.get("require_grasp_detection", True):
            return
        status_topic = self.config.get(
            "gripper_status_topic", "/right_arm/Robotiq2FGripperRobotInput"
        )
        deadline = time.time() + self.config.get("grasp_detection_timeout", 2.0)
        last_status = None
        while time.time() < deadline:
            try:
                last_status = rospy.wait_for_message(
                    status_topic, Robotiq2FGripper_robot_input, timeout=0.5
                )
            except rospy.ROSException:
                continue
            if last_status.gFLT != 0:
                raise RuntimeError(
                    "Right gripper fault while closing: gFLT={}".format(last_status.gFLT)
                )
            if last_status.gOBJ == 2:
                rospy.loginfo(
                    "Grasp detected: gOBJ=%d gPO=%d", last_status.gOBJ, last_status.gPO
                )
                self.grasp_confirmed = True
                return

        self.publish_gripper(self.config.get("open_position", 0), "open after missed grasp")
        if last_status is None:
            raise RuntimeError("No right gripper status received after close")
        raise RuntimeError(
            "No object detected after close: gOBJ={} gPO={}; opened gripper and stopped".format(
                last_status.gOBJ, last_status.gPO
            )
        )

    def clear_grasp_area_before_return(self):
        retreat_distance = self.config.get("failure_retreat_distance", 0.080)
        lift_distance = self.config.get("failure_lift_distance", 0.080)
        current = self.current_xyz()
        retreat = self.add(current, self.scale(self.approach_vector(), -retreat_distance))
        lift = self.add(retreat, [0.0, 0.0, lift_distance])
        rospy.logwarn(
            "Clearing grasp area before return: retreat %.3fm then lift %.3fm",
            retreat_distance,
            lift_distance,
        )
        try:
            self.cartesian_to_pose("failure_retreat", retreat)
            self.cartesian_to_pose("failure_lift", lift)
        except RuntimeError as exc:
            rospy.logwarn("Could not fully clear grasp area before return: %s", exc)

    def return_to_start_after_failure(self, reason):
        if not self.execute:
            return
        rospy.logwarn(
            "Execution failed before a confirmed grasp: %s. Returning to start joint position.",
            reason,
        )
        try:
            self.ensure_external_control()
        except RuntimeError as exc:
            rospy.logerr("Cannot return to start because External Control is unavailable: %s", exc)
            return

        try:
            self.publish_gripper(self.config.get("open_position", 0), "open before return")
        except RuntimeError as exc:
            rospy.logwarn("Could not command gripper before return: %s", exc)

        self.clear_grasp_area_before_return()
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.virtual_start_state = None
        self.arm.set_start_state_to_current_state()
        self.arm.set_joint_value_target(self.start_joint_target)
        plan_result = self.arm.plan()
        trajectory = plan_result[1] if isinstance(plan_result, tuple) else plan_result
        if not trajectory.joint_trajectory.points:
            rospy.logerr("No valid MoveIt plan back to the start joint position")
            return

        rospy.loginfo(
            "Planned return_to_start with %d points",
            len(trajectory.joint_trajectory.points),
        )
        self.publish_display_trajectory(trajectory)
        if not self.arm.execute(trajectory, wait=True):
            self.arm.stop()
            rospy.logerr("Return-to-start execution failed")
            return
        self.arm.stop()
        rospy.logwarn("Returned to start joint position after failed grasp attempt.")

    def run_phase(self, phase):
        if phase == "retreat":
            rospy.loginfo("Mode: %s", "REAL EXECUTION" if self.execute else "PLAN ONLY")
            self.ensure_external_control()
            self.publish_gripper(self.config.get("open_position", 0), "open")
            retreat = self.add(
                self.current_xyz(),
                self.scale(
                    self.approach_vector(),
                    -self.config.get("manual_retreat_distance", 0.100),
                ),
            )
            self.cartesian_to_pose("retreat", retreat)
            rospy.loginfo("Retreat phase complete.")
            return

        target = self.wait_for_stable_target()
        pre_grasp, _, _, _ = self.grasp_poses(target)

        rospy.loginfo("Mode: %s", "REAL EXECUTION" if self.execute else "PLAN ONLY")
        self.ensure_external_control()
        if phase in ("full", "approach"):
            self.publish_gripper(self.config.get("open_position", 0), "open")
            self.plan_to_pose("pre_grasp", pre_grasp)
            if phase == "approach":
                rospy.loginfo("Approach phase complete. Stopped at verified hover candidate.")
                return
        else:
            self.require_hover(pre_grasp)

        if self.config.get("recheck_target_before_descent", True):
            target = self.wait_for_stable_target(expected=target)
        else:
            rospy.logwarn(
                "Target recheck before descent is disabled: using the stable target "
                "locked before approach because the verified hover occludes the camera."
            )
        pre_grasp, grasp, lift, shift = self.grasp_poses(target)

        self.require_hover(pre_grasp)
        self.cartesian_to_pose("grasp", grasp)
        self.publish_gripper(self.config.get("close_position", 210), "close")
        self.require_grasp()
        self.cartesian_to_pose("lift", lift)
        self.cartesian_to_pose("shift_right", shift)

        if self.config.get("release_after_shift", False):
            self.publish_gripper(self.config.get("open_position", 0), "release")
            self.grasp_confirmed = False
            self.object_released = True
        if self.config.get("return_to_initial_after_success", False):
            if not self.config.get("release_after_shift", False):
                raise RuntimeError(
                    "Refusing automatic return while still holding the object; "
                    "set release_after_shift: true"
                )
            self.plan_to_initial_joint_target()
            rospy.loginfo("Task complete: returned to configured initial joint position.")

        if not self.execute:
            rospy.logwarn(
                "Plan-only mode completed the full approach path: "
                "pre_grasp -> grasp -> lift -> shift_right. "
                "No robot motion or gripper motion was sent."
            )

    def run(self, phase):
        try:
            self.run_phase(phase)
        except Exception as exc:
            if (
                self.execute
                and not self.grasp_confirmed
                and not self.object_released
                and phase != "retreat"
            ):
                self.return_to_start_after_failure(exc)
            raise


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError(
            "Real execution is locked. After RViz checks, set enabled: true in YAML."
        )

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_arm_visual_pick")

    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, RViz plan checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    RightArmVisualPick(config, args.execute).run(args.phase)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
