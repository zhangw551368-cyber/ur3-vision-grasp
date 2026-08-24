#!/usr/bin/env python3

import argparse
import math
import os
import sys
import threading
import time

import cv2
import moveit_commander
import numpy as np
import rospy
import tf.transformations
import tf2_ros
import torch
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from sensor_msgs.msg import CameraInfo, Image
from skimage.feature import peak_local_max
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def parse_args():
    parser = argparse.ArgumentParser(
        description="Click an RGB-D image, predict a Jacquard/GRCNN grasp, then plan or execute with the right UR3."
    )
    parser.add_argument("--config", required=True, help="Task YAML file.")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Allow robot motion and gripper commands. Default is plan-only.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the typed EXECUTE confirmation when --execute is used.",
    )
    parser.add_argument(
        "--phase",
        choices=("full", "approach", "align"),
        default="full",
        help="Run full grasp, stop at pre-grasp, or stop at the open gripper alignment pose.",
    )
    parser.add_argument(
        "--auto-once",
        action="store_true",
        help="Automatically scan the RGB-D image once, pick the highest-scoring valid grasp, and run it.",
    )
    parser.add_argument("--click-u", type=int, default=None, help="Run one fixed image click at pixel u.")
    parser.add_argument("--click-v", type=int, default=None, help="Run one fixed image click at pixel v.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def make_gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = int(position)
    command.rSP = int(speed)
    command.rFR = int(force)
    return command


def normalize(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm == 0.0:
        raise RuntimeError("approach_vector cannot be zero")
    return [value / norm for value in vector]


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RightArmJacquardClickPick:
    def __init__(self, config, execute, phase, auto_once=False, fixed_click=None):
        self.config = config
        self.execute = execute
        self.phase = phase
        self.auto_once = auto_once
        self.fixed_click = fixed_click
        self.auto_done = False
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color_bgr = None
        self.depth_m = None
        self.camera_info = None
        self.pending_click = None
        self.processing = False
        self.last_prediction = None
        self.last_status = "Waiting for RGB-D camera frames"

        self.post_process_output = None
        self.CameraData = None
        self.model = None
        self.device = None
        self.cam_data = None
        self.load_grcnn_model()

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
        self.display = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )
        self.markers = rospy.Publisher(
            config.get("debug_marker_topic", "/right_arm/jacquard_debug_markers"),
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.virtual_start_state = None
        self.display_start_state = None
        self.display_trajectories = []
        self.grasp_confirmed = False

        rospy.Subscriber(
            config.get("color_topic", "/camera/color/image_raw"),
            Image,
            self.color_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.Subscriber(
            config.get("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.Subscriber(
            config.get("camera_info_topic", "/camera/color/camera_info"),
            CameraInfo,
            self.info_callback,
            queue_size=1,
        )

        self.window_name = config.get("window_name", "right_arm_jacquard_click_pick")
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        rospy.loginfo(
            "Click-to-grasp window '%s' is ready. Left-click a target object; press q or Esc to exit.",
            self.window_name,
        )

    def load_grcnn_model(self):
        grcnn_root = os.path.abspath(os.path.expanduser(self.config["grcnn_root"]))
        if grcnn_root not in sys.path:
            sys.path.insert(0, grcnn_root)
        from inference.post_process import post_process_output
        from utils.data.camera_data import CameraData

        self.post_process_output = post_process_output
        self.CameraData = CameraData

        use_cuda = bool(self.config.get("use_cuda", True)) and torch.cuda.is_available()
        self.device = torch.device("cuda:0" if use_cuda else "cpu")
        model_path = os.path.expanduser(self.config["model_path"])
        rospy.loginfo("Loading GRCNN model %s on %s", model_path, self.device)
        try:
            self.model = torch.load(
                model_path, map_location=self.device, weights_only=False
            )
        except TypeError:
            self.model = torch.load(model_path, map_location=self.device)
        self.model.to(self.device)
        self.model.eval()
        crop_size = int(self.config.get("crop_size", 300))
        self.cam_data = self.CameraData(
            width=crop_size,
            height=crop_size,
            output_size=crop_size,
            include_depth=bool(self.config.get("include_depth", True)),
            include_rgb=bool(self.config.get("include_rgb", True)),
        )
        rospy.loginfo("GRCNN model loaded, crop_size=%d", crop_size)

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Color image conversion failed: %s", exc)
            return
        with self.lock:
            self.color_bgr = image

    def depth_callback(self, msg):
        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
                dtype=np.float32,
            )
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Depth image conversion failed: %s", exc)
            return
        if msg.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        depth[~np.isfinite(depth)] = 0.0
        max_depth = float(self.config.get("max_depth_m", 0.0))
        if max_depth > 0.0:
            depth[depth > max_depth] = 0.0
        min_depth = float(self.config.get("min_depth_m", 0.0))
        if min_depth > 0.0:
            depth[depth < min_depth] = 0.0
        with self.lock:
            self.depth_m = depth

    def info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    def mouse_callback(self, event, x, y, flags, param):
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self.lock:
            if self.processing:
                self.last_status = "Busy planning current click"
                return
            self.pending_click = (int(x), int(y))
            self.last_status = "Queued click at u={} v={}".format(x, y)

    def snapshot(self):
        with self.lock:
            color = None if self.color_bgr is None else self.color_bgr.copy()
            depth = None if self.depth_m is None else self.depth_m.copy()
            info = self.camera_info
        if color is None or depth is None or info is None:
            return None, None, None
        return color, depth, info

    def pop_click(self):
        with self.lock:
            click = self.pending_click
            self.pending_click = None
        return click

    def set_processing(self, value):
        with self.lock:
            self.processing = value

    def set_status(self, message):
        rospy.loginfo("%s", message)
        with self.lock:
            self.last_status = message

    def crop_around_click(self, color_bgr, depth, click):
        crop_size = int(self.config.get("crop_size", 300))
        height, width = color_bgr.shape[:2]
        if height < crop_size or width < crop_size:
            raise RuntimeError(
                "Camera image {}x{} is smaller than crop_size={}".format(
                    width, height, crop_size
                )
            )
        click_x = clamp(click[0], 0, width - 1)
        click_y = clamp(click[1], 0, height - 1)
        half = crop_size // 2
        left = clamp(click_x - half, 0, width - crop_size)
        top = clamp(click_y - half, 0, height - crop_size)
        right = left + crop_size
        bottom = top + crop_size
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        rgb_crop = color_rgb[top:bottom, left:right].copy()
        depth_crop = depth[top:bottom, left:right].copy()
        return rgb_crop, np.expand_dims(depth_crop, axis=2), top, left

    def detect_grasp_candidates(self, q_img, ang_img, width_img):
        peaks = peak_local_max(
            q_img,
            min_distance=int(self.config.get("grasp_peak_min_distance", 20)),
            threshold_abs=float(self.config.get("grasp_quality_threshold", 0.20)),
            num_peaks=int(self.config.get("num_grasp_candidates", 8)),
        )
        candidates = []
        for row, col in peaks:
            width_px = float(width_img[row, col]) if width_img is not None else 60.0
            width_px = max(0.0, width_px)
            candidates.append(
                {
                    "row": int(row),
                    "col": int(col),
                    "angle": float(ang_img[row, col]),
                    "width_px": width_px,
                    "quality": float(q_img[row, col]),
                }
            )
        return candidates

    def choose_candidate(self, candidates):
        if not candidates:
            raise RuntimeError("GRCNN did not detect a grasp above the quality threshold")
        crop_size = float(self.config.get("crop_size", 300))
        center = np.array([crop_size / 2.0, crop_size / 2.0])
        radius = float(self.config.get("selection_radius_pixels", 120))
        distance_weight = float(self.config.get("distance_score_weight", 0.20))
        for candidate in candidates:
            point = np.array([candidate["row"], candidate["col"]], dtype=np.float64)
            distance = float(np.linalg.norm(point - center))
            candidate["distance_to_click"] = distance
            candidate["selection_score"] = (
                candidate["quality"] - distance_weight * distance / crop_size
            )
        nearby = [candidate for candidate in candidates if candidate["distance_to_click"] <= radius]
        if nearby:
            pool = nearby
        elif self.config.get("allow_outside_click_radius", False):
            pool = candidates
        else:
            raise RuntimeError(
                "No GRCNN grasp candidate within {:.0f}px of the clicked point".format(
                    radius
                )
            )
        return max(pool, key=lambda candidate: candidate["selection_score"])

    def predict_grasp(self, color_bgr, depth, click):
        rgb_crop, depth_crop, top, left = self.crop_around_click(color_bgr, depth, click)
        x, _, _ = self.cam_data.get_data(rgb=rgb_crop, depth=depth_crop)
        with torch.no_grad():
            pred = self.model.predict(x.to(self.device))
        q_img, ang_img, width_img = self.post_process_output(
            pred["pos"], pred["cos"], pred["sin"], pred["width"]
        )
        candidates = self.detect_grasp_candidates(q_img, ang_img, width_img)
        best = self.choose_candidate(candidates)
        best["full_u"] = int(left + best["col"])
        best["full_v"] = int(top + best["row"])
        best["crop_left"] = int(left)
        best["crop_top"] = int(top)
        best["crop_size"] = int(self.config.get("crop_size", 300))
        return best

    def depth_at(self, depth, u, v):
        base_radius = max(1, int(self.config.get("depth_window_pixels", 9)) // 2)
        min_depth = float(self.config.get("min_depth_m", 0.05))
        max_depth = float(self.config.get("max_depth_m", 1.2))
        for radius in (base_radius, 12, 20, 32):
            y0 = max(0, v - radius)
            y1 = min(depth.shape[0], v + radius + 1)
            x0 = max(0, u - radius)
            x1 = min(depth.shape[1], u + radius + 1)
            values = depth[y0:y1, x0:x1]
            values = values[np.isfinite(values) & (values > min_depth)]
            if max_depth > 0.0:
                values = values[values < max_depth]
            if values.size:
                return float(np.median(values))
        raise RuntimeError("No valid depth near grasp pixel u={} v={}".format(u, v))

    def pixel_to_camera_point(self, info, u, v, depth):
        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("Invalid camera intrinsics: fx={} fy={}".format(fx, fy))
        return np.array(
            [
                (float(u) - cx) * depth / fx,
                (float(v) - cy) * depth / fy,
                depth,
            ],
            dtype=np.float64,
        )

    def transform_camera_to_base(self, camera_frame, point_camera):
        planning_frame = self.config.get("planning_frame", "base")
        transform = self.tf_buffer.lookup_transform(
            planning_frame, camera_frame, rospy.Time(0), rospy.Duration(2.0)
        ).transform
        translation = np.array(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            dtype=np.float64,
        )
        quaternion = (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        return rotation.dot(point_camera) + translation

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
                "Clicked grasp target is outside allowed workspace: {}".format(
                    ", ".join(violations)
                )
            )

    def orientation_quaternion(self, grasp_angle):
        if self.config.get("orientation_mode", "top_down") == "fixed":
            q = self.config.get("fixed_orientation_quaternion", [-0.5, 0.5, 0.5, 0.5])
            return (q[0], q[1], q[2], q[3])
        roll, pitch, base_yaw = self.config.get(
            "top_down_rpy", [-math.pi, 0.0, 0.0]
        )
        sign = -1.0 if self.config.get("invert_model_angle", True) else 1.0
        yaw = base_yaw + float(self.config.get("yaw_offset", math.pi / 2.0)) + sign * grasp_angle
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return (q[0], q[1], q[2], q[3])

    def pose(self, xyz, grasp_angle):
        target = PoseStamped()
        target.header.frame_id = self.config.get("planning_frame", "base")
        target.header.stamp = rospy.Time.now()
        target.pose = Pose()
        target.pose.position.x = float(xyz[0])
        target.pose.position.y = float(xyz[1])
        target.pose.position.z = float(xyz[2])
        q = self.orientation_quaternion(grasp_angle)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = q
        return target

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

    def publish_display_trajectory(self, trajectory):
        if self.display_start_state is None:
            self.display_start_state = self.arm.get_current_state()
        self.display_trajectories.append(trajectory)
        display = DisplayTrajectory()
        display.trajectory_start = self.display_start_state
        display.trajectory.extend(self.display_trajectories)
        self.display.publish(display)
        time.sleep(float(self.config.get("display_pause_seconds", 0.8)))

    def plan_to_pose(self, name, xyz, grasp_angle):
        target = self.pose(xyz, grasp_angle)
        trajectory = None
        retries = max(1, int(self.config.get("plan_retries", 3)))
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
                trajectory = candidate
                break
            rospy.logwarn("No valid MoveIt plan for %s on attempt %d/%d", name, attempt, retries)
        if trajectory is None:
            raise RuntimeError("No valid MoveIt plan for {}".format(name))
        rospy.loginfo(
            "Planned %-10s xyz=[%.3f, %.3f, %.3f] with %d points",
            name,
            xyz[0],
            xyz[1],
            xyz[2],
            len(trajectory.joint_trajectory.points),
        )
        self.publish_display_trajectory(trajectory)
        if self.execute:
            if not self.arm.execute(trajectory, wait=True):
                raise RuntimeError("Execution failed at {}".format(name))
            self.arm.stop()
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

    def cartesian_to_pose(self, name, xyz, grasp_angle):
        target = self.pose(xyz, grasp_angle)
        if self.virtual_start_state is not None:
            self.arm.set_start_state(self.virtual_start_state)
            trajectory_start = self.virtual_start_state
        else:
            self.arm.set_start_state_to_current_state()
            trajectory_start = self.arm.get_current_state()
        trajectory, fraction = self.arm.compute_cartesian_path(
            [target.pose],
            float(self.config.get("cartesian_step", 0.005)),
            avoid_collisions=True,
        )
        min_fraction = float(self.config.get("cartesian_min_fraction", 0.995))
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
            self.virtual_start_state = None
        else:
            self.virtual_start_state = self.end_state_from_trajectory(trajectory)
        return trajectory

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
        rospy.loginfo("External Control confirmed on %s", topic)

    def publish_gripper(self, position, label):
        rospy.loginfo("Gripper command: %s rPR=%d", label, position)
        if not self.execute:
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
        time.sleep(float(self.config.get("gripper_settle_seconds", 1.0)))

    def require_grasp(self):
        if not self.execute or not self.config.get("require_grasp_detection", True):
            return
        status_topic = self.config.get(
            "gripper_status_topic", "/right_arm/Robotiq2FGripperRobotInput"
        )
        deadline = time.time() + float(self.config.get("grasp_detection_timeout", 2.0))
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

    def publish_debug_markers(self, grasp_point, pre_grasp, grasp, lift, shift):
        frame_id = self.config.get("planning_frame", "base")
        now = rospy.Time.now()
        marker_array = MarkerArray()

        def sphere(marker_id, name, xyz, rgba, scale):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(xyz[0])
            marker.pose.position.y = float(xyz[1])
            marker.pose.position.z = float(xyz[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)

        def line(marker_id, name, points, rgba):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.006
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            marker.pose.orientation.w = 1.0
            for xyz in points:
                point = Point()
                point.x = float(xyz[0])
                point.y = float(xyz[1])
                point.z = float(xyz[2])
                marker.points.append(point)
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)

        sphere(0, "clicked_grasp_point", grasp_point, (1.0, 0.1, 0.1, 1.0), 0.035)
        sphere(1, "tool0_pre_grasp", pre_grasp, (1.0, 0.7, 0.0, 1.0), 0.030)
        sphere(2, "tool0_grasp", grasp, (0.1, 0.2, 1.0, 1.0), 0.030)
        sphere(3, "tool0_lift", lift, (0.1, 0.8, 1.0, 1.0), 0.025)
        line(4, "tool0_path", [pre_grasp, grasp, lift, shift], (0.0, 1.0, 0.2, 1.0))
        self.markers.publish(marker_array)

    def save_debug_image(self, color_bgr, click, prediction):
        path = self.config.get("debug_image_path", "")
        if not path:
            return
        image = color_bgr.copy()
        left = prediction["crop_left"]
        top = prediction["crop_top"]
        size = prediction["crop_size"]
        cv2.rectangle(image, (left, top), (left + size, top + size), (255, 180, 0), 1)
        cv2.circle(image, (int(click[0]), int(click[1])), 7, (0, 0, 255), -1)
        cv2.putText(
            image,
            "click",
            (int(click[0]) + 8, max(20, int(click[1]) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
        model_point = (int(prediction["full_u"]), int(prediction["full_v"]))
        cv2.circle(image, model_point, 6, (0, 255, 255), -1)
        cv2.putText(
            image,
            "grcnn",
            (model_point[0] + 8, max(20, model_point[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
        target_point = (int(prediction["target_u"]), int(prediction["target_v"]))
        cv2.circle(image, target_point, 10, (255, 0, 0), 2)
        cv2.putText(
            image,
            "target",
            (target_point[0] + 8, min(image.shape[0] - 12, target_point[1] + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 0),
            2,
        )
        text = "base=[{:.3f},{:.3f},{:.3f}] depth={:.3f}m".format(
            prediction["point_base"][0],
            prediction["point_base"][1],
            prediction["point_base"][2],
            prediction["depth_m"],
        )
        cv2.putText(
            image,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            3,
        )
        cv2.putText(
            image,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            1,
        )
        directory = os.path.dirname(os.path.abspath(os.path.expanduser(path)))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        cv2.imwrite(os.path.expanduser(path), image)
        rospy.loginfo("Saved Jacquard click debug image: %s", os.path.expanduser(path))

    def build_grasp_sequence(self, point_base, grasp_angle):
        grasp_point = np.array(point_base, dtype=np.float64)
        grasp_point += np.array(
            self.config.get("target_offset", [0.0, 0.0, 0.0]), dtype=np.float64
        )
        mode = self.config.get("tool0_target_mode", "approach_offset")
        if mode == "tool_offset":
            q = self.orientation_quaternion(grasp_angle)
            rotation = tf.transformations.quaternion_matrix(q)[:3, :3]
            tool_offset = np.array(
                self.config.get("tool0_to_grasp_point", [0.0, 0.0, 0.131]),
                dtype=np.float64,
            )
            offset_base = rotation.dot(tool_offset)
            approach = normalize(offset_base.tolist())
            target = grasp_point - offset_base
        else:
            approach = normalize(self.config.get("approach_vector", [0.0, 0.0, -1.0]))
            target = grasp_point
            target += np.array(approach, dtype=np.float64) * float(
                self.config.get("grasp_depth_along_approach", 0.0)
            )
        pre_grasp = target - np.array(approach, dtype=np.float64) * float(
            self.config.get("pre_grasp_clearance", 0.12)
        )
        lift = target - np.array(approach, dtype=np.float64) * float(
            self.config.get("lift_distance", 0.08)
        )
        shift = lift + np.array(self.config.get("post_grasp_shift", [0.0, 0.0, 0.0]), dtype=np.float64)
        for xyz in (target, pre_grasp, lift, shift):
            self.validate_target_bounds(xyz)
        return (
            grasp_point.tolist(),
            pre_grasp.tolist(),
            target.tolist(),
            lift.tolist(),
            shift.tolist(),
        )

    def handle_click(self, click):
        color_bgr, depth, info = self.snapshot()
        if color_bgr is None:
            raise RuntimeError("RGB-D frames are not ready yet")
        best = self.predict_grasp(color_bgr, depth, click)
        target_u = best["full_u"]
        target_v = best["full_v"]
        target_source = "grcnn"
        if self.config.get("use_clicked_pixel_as_target", False):
            target_u = int(click[0])
            target_v = int(click[1])
            target_source = "clicked_pixel"
        z = self.depth_at(depth, target_u, target_v)
        point_camera = self.pixel_to_camera_point(info, target_u, target_v, z)
        camera_frame = info.header.frame_id or "camera_color_optical_frame"
        point_base = self.transform_camera_to_base(camera_frame, point_camera)
        self.validate_target_bounds(point_base)

        best["depth_m"] = z
        best["point_base"] = point_base.tolist()
        best["target_u"] = target_u
        best["target_v"] = target_v
        best["target_source"] = target_source
        with self.lock:
            self.last_prediction = best
        self.save_debug_image(color_bgr, click, best)

        rospy.loginfo(
            "Clicked u=%d v=%d -> grasp u=%d v=%d q=%.3f angle=%.3f width_px=%.1f; target=%s u=%d v=%d depth=%.3f base=[%.3f, %.3f, %.3f]",
            click[0],
            click[1],
            best["full_u"],
            best["full_v"],
            best["quality"],
            best["angle"],
            best["width_px"],
            target_source,
            target_u,
            target_v,
            z,
            point_base[0],
            point_base[1],
            point_base[2],
        )

        grasp_point, pre_grasp, grasp, lift, shift = self.build_grasp_sequence(
            point_base, best["angle"]
        )
        self.publish_debug_markers(grasp_point, pre_grasp, grasp, lift, shift)
        rospy.loginfo(
            "Grasp geometry in %s: clicked_point=[%.3f, %.3f, %.3f] tool0_pre=[%.3f, %.3f, %.3f] tool0_grasp=[%.3f, %.3f, %.3f] tool0_retreat=[%.3f, %.3f, %.3f]",
            self.config.get("planning_frame", "base"),
            grasp_point[0],
            grasp_point[1],
            grasp_point[2],
            pre_grasp[0],
            pre_grasp[1],
            pre_grasp[2],
            grasp[0],
            grasp[1],
            grasp[2],
            lift[0],
            lift[1],
            lift[2],
        )
        self.virtual_start_state = None
        self.display_start_state = None
        self.display_trajectories = []
        self.grasp_confirmed = False

        mode = "REAL EXECUTION" if self.execute else "PLAN ONLY"
        self.set_status("{}: planning clicked Jacquard grasp".format(mode))
        self.ensure_external_control()
        self.publish_gripper(self.config.get("open_position", 0), "open")
        self.plan_to_pose("pre_grasp", pre_grasp, best["angle"])
        if self.phase == "approach":
            self.set_status("{} approach complete; stopped at clicked hover".format(mode))
            return
        self.cartesian_to_pose("grasp", grasp, best["angle"])
        if self.phase == "align":
            self.set_status(
                "{} alignment complete; open gripper is at clicked grasp pose".format(
                    mode
                )
            )
            return
        self.publish_gripper(self.config.get("close_position", 210), "close")
        try:
            self.require_grasp()
        except Exception:
            if self.config.get("retreat_on_missed_grasp", True):
                try:
                    self.cartesian_to_pose("miss_retreat", pre_grasp, best["angle"])
                except Exception as retreat_exc:
                    rospy.logerr("Missed-grasp retreat failed: %s", retreat_exc)
            raise
        self.cartesian_to_pose("lift", lift, best["angle"])
        if np.linalg.norm(np.array(shift) - np.array(lift)) > 1e-6:
            self.cartesian_to_pose("shift", shift, best["angle"])
        self.set_status("{} complete for clicked grasp".format(mode))

    def wait_for_rgbd(self, timeout=10.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            color_bgr, depth, info = self.snapshot()
            if color_bgr is not None and depth is not None and info is not None:
                return color_bgr, depth, info
            time.sleep(0.1)
        raise RuntimeError("Timed out waiting for RGB-D camera frames")

    def auto_select_click(self):
        color_bgr, depth, info = self.wait_for_rgbd()
        height, width = color_bgr.shape[:2]
        crop_size = int(self.config.get("crop_size", 300))
        half = crop_size // 2
        stride = int(self.config.get("auto_scan_stride_pixels", 100))
        xs = list(range(half, max(half + 1, width - half + 1), stride))
        ys = list(range(half, max(half + 1, height - half + 1), stride))
        if not xs or xs[-1] != width - half:
            xs.append(width - half)
        if not ys or ys[-1] != height - half:
            ys.append(height - half)

        candidates = []
        for y in ys:
            for x in xs:
                click = (int(x), int(y))
                try:
                    best = self.predict_grasp(color_bgr, depth, click)
                    z = self.depth_at(depth, best["full_u"], best["full_v"])
                    point_camera = self.pixel_to_camera_point(
                        info, best["full_u"], best["full_v"], z
                    )
                    camera_frame = info.header.frame_id or "camera_color_optical_frame"
                    point_base = self.transform_camera_to_base(camera_frame, point_camera)
                    self.validate_target_bounds(point_base)
                except Exception as exc:
                    rospy.logwarn_throttle(
                        1.0,
                        "Auto scan rejected crop centered at u=%d v=%d: %s",
                        click[0],
                        click[1],
                        exc,
                    )
                    continue
                best["scan_click"] = click
                best["depth_m"] = z
                best["point_base"] = point_base.tolist()
                candidates.append(best)

        if not candidates:
            raise RuntimeError("Auto scan did not find any valid Jacquard grasp")
        best = max(candidates, key=lambda item: item["quality"])
        rospy.loginfo(
            "Auto-selected grasp u=%d v=%d q=%.3f angle=%.3f depth=%.3f base=[%.3f, %.3f, %.3f]",
            best["full_u"],
            best["full_v"],
            best["quality"],
            best["angle"],
            best["depth_m"],
            best["point_base"][0],
            best["point_base"][1],
            best["point_base"][2],
        )
        return (best["full_u"], best["full_v"])

    def draw_prediction(self, image):
        with self.lock:
            prediction = None if self.last_prediction is None else dict(self.last_prediction)
            status = self.last_status
            click = self.pending_click
        if click is not None:
            cv2.circle(image, click, 5, (255, 180, 0), -1)
        if prediction:
            left = prediction["crop_left"]
            top = prediction["crop_top"]
            size = prediction["crop_size"]
            cv2.rectangle(image, (left, top), (left + size, top + size), (255, 180, 0), 1)
            u = prediction["full_u"]
            v = prediction["full_v"]
            angle = prediction["angle"]
            length = clamp(int(prediction.get("width_px", 60)), 25, 160)
            dx = int(math.cos(angle) * length / 2.0)
            dy = int(-math.sin(angle) * length / 2.0)
            cv2.circle(image, (u, v), 5, (0, 255, 255), -1)
            cv2.line(image, (u - dx, v - dy), (u + dx, v + dy), (0, 255, 0), 2)
            cv2.putText(
                image,
                "q={:.2f} z={:.3f}m".format(
                    prediction.get("quality", 0.0), prediction.get("depth_m", 0.0)
                ),
                (u + 8, max(24, v - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
        cv2.putText(
            image,
            status[:110],
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            image,
            "left-click target | q/Esc exit | {}".format(
                "EXECUTE" if self.execute else "PLAN ONLY"
            ),
            (12, image.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return image

    def display_frame(self):
        color_bgr, _, _ = self.snapshot()
        if color_bgr is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                image,
                "Waiting for /camera RGB-D topics...",
                (40, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )
            return image
        return self.draw_prediction(color_bgr)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            image = self.display_frame()
            cv2.imshow(self.window_name, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            click = self.pop_click()
            if self.fixed_click is not None and not self.auto_done and click is None:
                self.auto_done = True
                try:
                    self.wait_for_rgbd()
                except Exception as exc:
                    rospy.logerr("%s", exc)
                    self.set_status("Fixed click failed: {}".format(exc))
                    break
                rospy.loginfo(
                    "Using fixed click at u=%d v=%d",
                    self.fixed_click[0],
                    self.fixed_click[1],
                )
                click = self.fixed_click
            if self.auto_once and not self.auto_done and click is None:
                self.auto_done = True
                try:
                    click = self.auto_select_click()
                    with self.lock:
                        self.pending_click = click
                except Exception as exc:
                    rospy.logerr("%s", exc)
                    self.set_status("Auto grasp failed: {}".format(exc))
                    break
                click = self.pop_click()
            if click is not None:
                self.set_processing(True)
                try:
                    self.handle_click(click)
                except Exception as exc:
                    rospy.logerr("%s", exc)
                    self.set_status("Click grasp failed: {}".format(exc))
                finally:
                    self.set_processing(False)
                if self.fixed_click is not None or self.auto_once:
                    break
            rate.sleep()
        cv2.destroyWindow(self.window_name)


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError(
            "Real execution is locked. After RViz checks, set enabled: true in YAML."
        )

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("right_arm_jacquard_click_pick")
    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, RViz plan checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    fixed_click = None
    if args.click_u is not None or args.click_v is not None:
        if args.click_u is None or args.click_v is None:
            raise RuntimeError("--click-u and --click-v must be used together")
        fixed_click = (args.click_u, args.click_v)

    RightArmJacquardClickPick(
        config, args.execute, args.phase, args.auto_once, fixed_click
    ).spin()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
