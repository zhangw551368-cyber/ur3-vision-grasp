#!/usr/bin/python3

import argparse
import copy
import math
import os
import sys
import time

import cv2
import moveit_commander
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401, registers PoseStamped transforms.
import tf2_ros
import yaml
from cv_bridge import CvBridge
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from single_arm_pick_place import SingleArmPickPlace
from single_arm_pick_place import float_list
from single_arm_pick_place import parse_pose_record


DEFAULT_FIXED_CONFIG = "/home/gzu/gzu_ws/right_arm_two_objects_pick_place.yaml"


def parse_bool_arg(value):
    if isinstance(value, bool):
        return value
    lowered = str(value).strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("Expected a boolean value, got {!r}".format(value))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plan or execute right-arm ArUco visual pick with fixed place poses."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_FIXED_CONFIG,
        help="Fixed pick/place YAML. Multi-object YAML with objects[] is supported.",
    )
    parser.add_argument(
        "--object-index",
        type=int,
        default=0,
        help="Zero-based object index used for fixed home/place/retreat poses.",
    )
    parser.add_argument(
        "--object-name",
        default="",
        help="Object name in objects[]. Overrides --object-index when set.",
    )
    parser.add_argument(
        "--aruco-pose-topic",
        default=None,
        help="PoseStamped topic from aruco_ros, for example /aruco_single/pose.",
    )
    parser.add_argument(
        "--use-opencv-detection",
        action="store_true",
        help="Detect the marker directly from the Kinect RGB/depth topics.",
    )
    parser.add_argument("--marker-id", type=int, default=None, help="ArUco marker ID.")
    parser.add_argument(
        "--aruco-dictionary",
        default=None,
        help="OpenCV dictionary name, for example DICT_4X4_50.",
    )
    parser.add_argument("--image-topic", default=None, help="Color image topic.")
    parser.add_argument("--depth-topic", default=None, help="Registered depth image topic.")
    parser.add_argument("--camera-info-topic", default=None, help="CameraInfo topic.")
    parser.add_argument(
        "--detection-scale",
        type=float,
        default=None,
        help="Upscale factor before marker detection.",
    )
    parser.add_argument(
        "--depth-window",
        type=int,
        default=None,
        help="Odd pixel window size used for median marker depth.",
    )
    parser.add_argument(
        "--camera-frame",
        default=None,
        help="Frame to use if the ArUco pose topic has an empty header.frame_id.",
    )
    parser.add_argument(
        "--base-frame",
        default=None,
        help="Target frame for the detected object pose. Must match planning_frame.",
    )
    parser.add_argument(
        "--detected-pose-topic",
        default=None,
        help="Published detected object pose in the planning frame.",
    )
    parser.add_argument(
        "--aruco-to-grasp-offset",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Offset from ArUco center to TCP grasp center in the planning frame.",
    )
    parser.add_argument(
        "--visual-compensation-mode",
        choices=("absolute_offset", "absolute_yaml_approach", "aruco_geometry", "yaml_delta"),
        default=None,
        help=(
            "absolute_offset builds grasp from marker directly; "
            "absolute_yaml_approach builds grasp from marker and reuses the taught "
            "YAML approach/lift vectors; aruco_geometry builds grasp from marker and "
            "object dimensions only; yaml_delta shifts YAML poses by visual delta."
        ),
    )
    parser.add_argument(
        "--object-size",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Object size in meters. Z is used for top-marker to object-center offset.",
    )
    parser.add_argument(
        "--marker-to-object-center",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Vector from ArUco top-center to object grasp center in the planning frame.",
    )
    parser.add_argument(
        "--geometry-approach-vector",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Vector from grasp to pre_grasp in aruco_geometry mode.",
    )
    parser.add_argument(
        "--geometry-lift-vector",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Vector from grasp to lift in aruco_geometry mode.",
    )
    parser.add_argument(
        "--geometry-orientation-source",
        default=None,
        help="Pose name whose orientation is reused for dynamic grasp poses.",
    )
    parser.add_argument(
        "--expected-object-center-z",
        type=float,
        default=None,
        help="Expected object center z in planning frame, used to detect hand-eye z error.",
    )
    parser.add_argument(
        "--max-handeye-z-error",
        type=float,
        default=None,
        help="Reject execution when expected and visual object-center z differ more than this.",
    )
    parser.add_argument(
        "--apply-handeye-z-correction",
        action="store_true",
        help="If set, replace visual object-center z with --expected-object-center-z.",
    )
    parser.add_argument(
        "--require-grasp-detection",
        nargs="?",
        const=True,
        default=None,
        type=parse_bool_arg,
        help="After closing the Robotiq gripper, require gOBJ=2 before lift/place.",
    )
    parser.add_argument(
        "--gripper-status-topic",
        default=None,
        help="Robotiq input/status topic used by --require-grasp-detection.",
    )
    parser.add_argument(
        "--nominal-marker-base",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Marker xyz in planning frame when the object is at the successful taught YAML position.",
    )
    parser.add_argument(
        "--compensation-axes",
        default=None,
        help="Axes to apply from visual delta, for example xy or xyz.",
    )
    parser.add_argument(
        "--max-visual-delta",
        type=float,
        default=None,
        help="Reject visual compensation larger than this distance in meters.",
    )
    parser.add_argument(
        "--pre-grasp-lift",
        type=float,
        default=None,
        help="Vertical distance from grasp to pre_grasp in meters.",
    )
    parser.add_argument(
        "--lift-height",
        type=float,
        default=None,
        help="Vertical distance from grasp to lift in meters.",
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=None,
        help="Number of stable ArUco pose samples to average.",
    )
    parser.add_argument(
        "--pose-timeout",
        type=float,
        default=None,
        help="Seconds to wait for stable ArUco pose samples.",
    )
    parser.add_argument(
        "--target-max-spread",
        type=float,
        default=None,
        help="Maximum allowed spread among ArUco samples in meters.",
    )
    parser.add_argument(
        "--skip-initial-home",
        nargs="?",
        const=True,
        default=None,
        type=parse_bool_arg,
        help="Start visual sampling from the current robot pose instead of moving home first.",
    )
    parser.add_argument("--execute", action="store_true", help="Execute on the real robot.")
    parser.add_argument("--yes", action="store_true", help="Skip typed EXECUTE confirmation.")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("Config file is empty or invalid: {}".format(path))
    return data


def select_object_config(raw_config, object_index, object_name):
    if "objects" not in raw_config:
        return copy.deepcopy(raw_config), ""

    objects = raw_config.get("objects")
    if not isinstance(objects, list) or not objects:
        raise ValueError("objects must be a non-empty list.")

    selected = None
    if object_name:
        for candidate in objects:
            if candidate.get("name") == object_name:
                selected = candidate
                break
        if selected is None:
            raise ValueError("Object named {!r} was not found.".format(object_name))
    else:
        if object_index < 0 or object_index >= len(objects):
            raise ValueError(
                "object-index {} is outside objects[0..{}].".format(
                    object_index, len(objects) - 1
                )
            )
        selected = objects[object_index]

    config = {key: copy.deepcopy(value) for key, value in raw_config.items() if key != "objects"}
    poses = copy.deepcopy(selected.get("poses"))
    if not isinstance(poses, dict):
        raise ValueError("Selected object must contain poses.")
    if config.get("common_home") is not None and "home" not in poses:
        poses["home"] = copy.deepcopy(config["common_home"])
    config["poses"] = poses
    return config, selected.get("name", "object_{}".format(object_index + 1))


def config_value(config, name, default=None):
    visual = config.get("visual_aruco", {})
    if name in config:
        return config[name]
    if isinstance(visual, dict) and name in visual:
        return visual[name]
    return default


def parse_task_config(path, args):
    raw_config = load_yaml(path)
    config, selected_name = select_object_config(raw_config, args.object_index, args.object_name)
    if selected_name:
        config["selected_object_name"] = selected_name

    required_poses = {"home", "pre_grasp", "grasp", "lift", "pre_place", "place", "retreat"}
    poses = config.get("poses")
    if not isinstance(poses, dict):
        raise ValueError("poses must map pose names to pose records.")
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

    planning_frame = config.get("planning_frame", "base")
    base_frame = args.base_frame or config_value(config, "base_frame", planning_frame)
    if base_frame != planning_frame:
        raise ValueError(
            "base_frame ({}) must match planning_frame ({}) so dynamic and fixed poses "
            "use the same coordinates.".format(base_frame, planning_frame)
        )
    config["base_frame"] = base_frame

    object_size = (
        args.object_size
        if args.object_size is not None
        else config_value(config, "object_size", [0.060, 0.070, 0.020])
    )
    config["object_size"] = float_list(object_size, "object_size", 3)
    object_height = float(config_value(config, "object_height", config["object_size"][2]))
    config["object_height"] = object_height
    config["box_height"] = float(config_value(config, "box_height", 0.070))

    if args.aruco_pose_topic is not None:
        config["aruco_pose_topic"] = args.aruco_pose_topic
    else:
        config["aruco_pose_topic"] = config_value(
            config, "aruco_pose_topic", "/aruco_single/pose"
        )
    config["use_frozen_pose"] = bool(config_value(config, "use_frozen_pose", False))
    frozen_center = config_value(config, "frozen_center_base", None)
    config["frozen_center_base"] = (
        float_list(frozen_center, "frozen_center_base", 3)
        if frozen_center is not None
        else None
    )

    if args.camera_frame is not None:
        config["camera_frame"] = args.camera_frame
    else:
        config["camera_frame"] = config_value(
            config, "camera_frame", "kinect2_0_rgb_optical_frame"
        )

    if args.detected_pose_topic is not None:
        config["detected_pose_topic"] = args.detected_pose_topic
    else:
        config["detected_pose_topic"] = config_value(
            config, "detected_pose_topic", "/detected_object_pose_base"
        )

    if args.aruco_to_grasp_offset is not None:
        config["aruco_to_grasp_offset"] = list(args.aruco_to_grasp_offset)
    elif config_value(config, "aruco_to_grasp_offset") is not None:
        config["aruco_to_grasp_offset"] = float_list(
            config_value(config, "aruco_to_grasp_offset"),
            "aruco_to_grasp_offset",
            3,
        )
    else:
        config["aruco_to_grasp_offset"] = [0.0, 0.0, -0.5 * object_height]

    config["pre_grasp_lift"] = float(
        args.pre_grasp_lift
        if args.pre_grasp_lift is not None
        else config_value(config, "pre_grasp_lift", 0.080)
    )
    config["lift_height"] = float(
        args.lift_height
        if args.lift_height is not None
        else config_value(config, "lift_height", config["pre_grasp_lift"])
    )
    marker_to_center = (
        args.marker_to_object_center
        if args.marker_to_object_center is not None
        else config_value(
            config,
            "marker_to_object_center",
            [0.0, 0.0, 0.5 * object_height],
        )
    )
    config["marker_to_object_center"] = float_list(
        marker_to_center, "marker_to_object_center", 3
    )
    approach_vector = (
        args.geometry_approach_vector
        if args.geometry_approach_vector is not None
        else config_value(config, "geometry_approach_vector", None)
    )
    config["geometry_approach_vector"] = (
        float_list(approach_vector, "geometry_approach_vector", 3)
        if approach_vector is not None
        else [0.0, 0.0, -config["pre_grasp_lift"]]
    )
    lift_vector = (
        args.geometry_lift_vector
        if args.geometry_lift_vector is not None
        else config_value(config, "geometry_lift_vector", None)
    )
    config["geometry_lift_vector"] = (
        float_list(lift_vector, "geometry_lift_vector", 3)
        if lift_vector is not None
        else [0.0, 0.0, -config["lift_height"]]
    )
    config["geometry_orientation_source"] = (
        args.geometry_orientation_source
        if args.geometry_orientation_source is not None
        else config_value(config, "geometry_orientation_source", "grasp")
    )
    expected_z = (
        args.expected_object_center_z
        if args.expected_object_center_z is not None
        else config_value(config, "expected_object_center_z", None)
    )
    config["expected_object_center_z"] = (
        float(expected_z) if expected_z is not None else None
    )
    config["max_handeye_z_error"] = float(
        args.max_handeye_z_error
        if args.max_handeye_z_error is not None
        else config_value(config, "max_handeye_z_error", 0.035)
    )
    config["apply_handeye_z_correction"] = bool(
        args.apply_handeye_z_correction
        or config_value(config, "apply_handeye_z_correction", False)
    )
    config["require_grasp_detection"] = bool(
        args.require_grasp_detection
        if args.require_grasp_detection is not None
        else config_value(config, "require_grasp_detection", False)
    )
    config["gripper_status_topic"] = (
        args.gripper_status_topic
        if args.gripper_status_topic is not None
        else config_value(
            config,
            "gripper_status_topic",
            "/right_arm/Robotiq2FGripperRobotInput",
        )
    )
    config["aruco_sample_count"] = int(
        args.sample_count
        if args.sample_count is not None
        else config_value(config, "aruco_sample_count", 3)
    )
    config["aruco_pose_timeout"] = float(
        args.pose_timeout
        if args.pose_timeout is not None
        else config_value(config, "aruco_pose_timeout", 15.0)
    )
    config["target_max_spread"] = float(
        args.target_max_spread
        if args.target_max_spread is not None
        else config_value(config, "target_max_spread", 0.015)
    )
    config["skip_initial_home"] = bool(
        args.skip_initial_home
        if args.skip_initial_home is not None
        else config_value(config, "skip_initial_home", False)
    )
    config["tf_timeout"] = float(config_value(config, "tf_timeout", 2.0))
    config["use_latest_tf"] = bool(config_value(config, "use_latest_tf", True))
    config["use_opencv_detection"] = bool(
        args.use_opencv_detection or config_value(config, "use_opencv_detection", False)
    )
    config["marker_id"] = int(
        args.marker_id if args.marker_id is not None else config_value(config, "marker_id", 1)
    )
    config["aruco_dictionary"] = (
        args.aruco_dictionary
        if args.aruco_dictionary is not None
        else config_value(config, "aruco_dictionary", "DICT_4X4_50")
    )
    config["image_topic"] = (
        args.image_topic
        if args.image_topic is not None
        else config_value(config, "image_topic", "/kinect_0/kinect2/qhd/image_color_rect")
    )
    config["depth_topic"] = (
        args.depth_topic
        if args.depth_topic is not None
        else config_value(config, "depth_topic", "/kinect_0/kinect2/qhd/image_depth_rect")
    )
    config["camera_info_topic"] = (
        args.camera_info_topic
        if args.camera_info_topic is not None
        else config_value(config, "camera_info_topic", "/kinect_0/kinect2/qhd/camera_info")
    )
    config["detection_scale"] = float(
        args.detection_scale
        if args.detection_scale is not None
        else config_value(config, "detection_scale", 2.0)
    )
    config["depth_window"] = int(
        args.depth_window
        if args.depth_window is not None
        else config_value(config, "depth_window", 9)
    )
    config["min_depth"] = float(config_value(config, "min_depth", 0.05))
    config["max_depth"] = float(config_value(config, "max_depth", 5.0))
    config["visual_compensation_mode"] = (
        args.visual_compensation_mode
        if args.visual_compensation_mode is not None
        else config_value(config, "visual_compensation_mode", "absolute_offset")
    )
    nominal_marker = (
        args.nominal_marker_base
        if args.nominal_marker_base is not None
        else config_value(config, "nominal_marker_base", None)
    )
    config["nominal_marker_base"] = (
        float_list(nominal_marker, "nominal_marker_base", 3)
        if nominal_marker is not None
        else None
    )
    axes = (
        args.compensation_axes
        if args.compensation_axes is not None
        else config_value(config, "compensation_axes", "xy")
    )
    axes = str(axes).lower()
    invalid_axes = set(axes) - set("xyz")
    if invalid_axes:
        raise ValueError("compensation_axes may only contain x, y, z.")
    config["compensation_axes"] = axes
    config["max_visual_delta"] = float(
        args.max_visual_delta
        if args.max_visual_delta is not None
        else config_value(config, "max_visual_delta", 0.12)
    )
    return config


def xyz_from_pose(pose_stamped):
    p = pose_stamped.pose.position
    return [float(p.x), float(p.y), float(p.z)]


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class VisualArucoPickPlace(SingleArmPickPlace):
    def __init__(self, config, execute):
        super().__init__(config, execute)
        self.base_frame = config["base_frame"]
        self.camera_frame = config["camera_frame"]
        self.aruco_pose_topic = config["aruco_pose_topic"]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.detected_pose_pub = rospy.Publisher(
            config["detected_pose_topic"], PoseStamped, queue_size=1, latch=True
        )
        self.execute_cancel_pub = rospy.Publisher(
            "/execute_trajectory/cancel", GoalID, queue_size=1
        )
        controller_cancel_topic = "/{}/scaled_pos_joint_traj_controller/follow_joint_trajectory/cancel".format(
            config["arm_group"]
        )
        self.controller_cancel_pub = rospy.Publisher(
            controller_cancel_topic, GoalID, queue_size=1
        )
        self.bridge = CvBridge()

    @staticmethod
    def add(a, b):
        return [float(x + y) for x, y in zip(a, b)]

    def transform_aruco_pose(self, msg):
        pose = copy.deepcopy(msg)
        if not pose.header.frame_id:
            pose.header.frame_id = self.camera_frame
        if self.config.get("use_latest_tf", True):
            pose.header.stamp = rospy.Time(0)
        return self.tf_buffer.transform(
            pose, self.base_frame, rospy.Duration(self.config["tf_timeout"])
        )

    def wait_for_aruco_pose_base(self):
        if self.config.get("use_frozen_pose", False):
            frozen = self.config.get("frozen_center_base")
            if frozen is None:
                raise RuntimeError("use_frozen_pose is true, but frozen_center_base is not set.")
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = frozen[0]
            pose.pose.position.y = frozen[1]
            pose.pose.position.z = frozen[2]
            pose.pose.orientation.w = 1.0
            self.detected_pose_pub.publish(pose)
            rospy.loginfo(
                "Using frozen object center in %s: x=%.3f y=%.3f z=%.3f",
                self.base_frame,
                frozen[0],
                frozen[1],
                frozen[2],
            )
            return pose

        if self.config.get("use_opencv_detection", False):
            return self.wait_for_opencv_aruco_pose_base()

        sample_count = max(1, int(self.config["aruco_sample_count"]))
        timeout = float(self.config["aruco_pose_timeout"])
        deadline = time.time() + timeout
        samples = []
        last_pose = None
        rospy.loginfo(
            "Waiting for %d ArUco pose sample(s) on %s, target frame %s",
            sample_count,
            self.aruco_pose_topic,
            self.base_frame,
        )
        while len(samples) < sample_count and not rospy.is_shutdown():
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            try:
                msg = rospy.wait_for_message(
                    self.aruco_pose_topic, PoseStamped, timeout=min(remaining, 2.0)
                )
                pose_base = self.transform_aruco_pose(msg)
            except (rospy.ROSException, tf2_ros.TransformException) as exc:
                rospy.logwarn_throttle(1.0, "Waiting for ArUco pose: %s", exc)
                continue
            samples.append(xyz_from_pose(pose_base))
            last_pose = pose_base

        if len(samples) < sample_count or last_pose is None:
            raise RuntimeError(
                "ArUco pose was not visible long enough on {}".format(
                    self.aruco_pose_topic
                )
            )

        mean = [
            sum(sample[index] for sample in samples) / len(samples)
            for index in range(3)
        ]
        spread = max(distance(sample, mean) for sample in samples)
        if spread > self.config["target_max_spread"]:
            raise RuntimeError(
                "ArUco pose is unstable: spread {:.3f}m exceeds {:.3f}m".format(
                    spread, self.config["target_max_spread"]
                )
            )

        detected = copy.deepcopy(last_pose)
        detected.header.frame_id = self.base_frame
        detected.header.stamp = rospy.Time.now()
        detected.pose.position.x = mean[0]
        detected.pose.position.y = mean[1]
        detected.pose.position.z = mean[2]
        self.detected_pose_pub.publish(detected)
        rospy.loginfo(
            "Detected ArUco in %s: x=%.3f y=%.3f z=%.3f spread=%.3fm",
            self.base_frame,
            mean[0],
            mean[1],
            mean[2],
            spread,
        )
        return detected

    def opencv_dictionary(self):
        dictionary_name = self.config["aruco_dictionary"]
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError("Unknown OpenCV ArUco dictionary: {}".format(dictionary_name))
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        if hasattr(cv2.aruco, "Dictionary_get"):
            return cv2.aruco.Dictionary_get(dictionary_id)
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def detect_opencv_aruco_pose_camera(self):
        timeout = min(float(self.config["aruco_pose_timeout"]), 3.0)
        color_msg = rospy.wait_for_message(self.config["image_topic"], Image, timeout=timeout)
        depth_msg = rospy.wait_for_message(self.config["depth_topic"], Image, timeout=timeout)
        info_msg = rospy.wait_for_message(
            self.config["camera_info_topic"], CameraInfo, timeout=timeout
        )
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        scale = max(1.0, float(self.config["detection_scale"]))
        detection_image = color
        if scale != 1.0:
            detection_image = cv2.resize(
                color, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
            )
        gray = cv2.cvtColor(detection_image, cv2.COLOR_BGR2GRAY)
        params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 51
        params.adaptiveThreshWinSizeStep = 4
        params.minMarkerPerimeterRate = 0.01
        params.maxMarkerPerimeterRate = 0.5
        params.polygonalApproxAccuracyRate = 0.08
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.opencv_dictionary(), parameters=params
        )
        if ids is None:
            raise RuntimeError("No OpenCV ArUco marker detected in {}".format(
                self.config["image_topic"]
            ))

        target_id = int(self.config["marker_id"])
        selected = None
        for marker_id, corner in zip(ids.flatten().tolist(), corners):
            if int(marker_id) == target_id:
                selected = corner.reshape(-1, 2) / scale
                break
        if selected is None:
            raise RuntimeError(
                "Detected marker IDs {}, but target ID {} was not present.".format(
                    ids.flatten().tolist(), target_id
                )
            )

        center_u, center_v = selected.mean(axis=0)
        window = max(3, int(self.config["depth_window"]))
        if window % 2 == 0:
            window += 1
        radius = window // 2
        u0 = max(0, int(round(center_u)) - radius)
        u1 = min(depth.shape[1], int(round(center_u)) + radius + 1)
        v0 = max(0, int(round(center_v)) - radius)
        v1 = min(depth.shape[0], int(round(center_v)) + radius + 1)
        patch = np.asarray(depth[v0:v1, u0:u1], dtype=np.float64)
        if depth_msg.encoding in ("16UC1", "mono16"):
            patch *= 0.001
        valid = patch[
            np.isfinite(patch)
            & (patch > self.config["min_depth"])
            & (patch < self.config["max_depth"])
        ]
        if len(valid) == 0:
            raise RuntimeError(
                "No valid depth around marker ID {} at pixel [{:.1f}, {:.1f}]".format(
                    target_id, center_u, center_v
                )
            )
        depth_m = float(np.median(valid))
        fx = float(info_msg.K[0])
        fy = float(info_msg.K[4])
        cx = float(info_msg.K[2])
        cy = float(info_msg.K[5])
        x = (float(center_u) - cx) * depth_m / fx
        y = (float(center_v) - cy) * depth_m / fy

        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame or info_msg.header.frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = depth_m
        pose.pose.orientation.w = 1.0
        rospy.loginfo(
            "OpenCV ArUco ID %d pixel=[%.1f, %.1f] depth=%.3fm camera_xyz=[%.3f, %.3f, %.3f]",
            target_id,
            center_u,
            center_v,
            depth_m,
            x,
            y,
            depth_m,
        )
        return pose

    def wait_for_opencv_aruco_pose_base(self):
        sample_count = max(1, int(self.config["aruco_sample_count"]))
        timeout = float(self.config["aruco_pose_timeout"])
        deadline = time.time() + timeout
        samples = []
        last_pose = None
        rospy.loginfo(
            "Waiting for %d OpenCV ArUco sample(s): dictionary=%s id=%d",
            sample_count,
            self.config["aruco_dictionary"],
            self.config["marker_id"],
        )
        while len(samples) < sample_count and not rospy.is_shutdown():
            if time.time() >= deadline:
                break
            try:
                pose_camera = self.detect_opencv_aruco_pose_camera()
                pose_base = self.transform_aruco_pose(pose_camera)
            except (RuntimeError, rospy.ROSException, tf2_ros.TransformException) as exc:
                rospy.logwarn_throttle(1.0, "Waiting for OpenCV ArUco pose: %s", exc)
                continue
            samples.append(xyz_from_pose(pose_base))
            last_pose = pose_base

        if len(samples) < sample_count or last_pose is None:
            raise RuntimeError("OpenCV ArUco pose was not visible long enough")

        mean = [
            sum(sample[index] for sample in samples) / len(samples)
            for index in range(3)
        ]
        spread = max(distance(sample, mean) for sample in samples)
        if spread > self.config["target_max_spread"]:
            raise RuntimeError(
                "OpenCV ArUco pose is unstable: spread {:.3f}m exceeds {:.3f}m".format(
                    spread, self.config["target_max_spread"]
                )
            )
        detected = copy.deepcopy(last_pose)
        detected.header.frame_id = self.base_frame
        detected.header.stamp = rospy.Time.now()
        detected.pose.position.x = mean[0]
        detected.pose.position.y = mean[1]
        detected.pose.position.z = mean[2]
        self.detected_pose_pub.publish(detected)
        rospy.loginfo(
            "Detected OpenCV ArUco in %s: x=%.3f y=%.3f z=%.3f spread=%.3fm",
            self.base_frame,
            mean[0],
            mean[1],
            mean[2],
            spread,
        )
        return detected

    def pose_record_with_translation(self, source_name, translation, fallback_name="grasp"):
        source = self.config["poses"].get(source_name) or self.config["poses"][fallback_name]
        record = copy.deepcopy(source)
        record["translation"] = [float(value) for value in translation]
        return record

    def generate_dynamic_grasp_poses(self, aruco_pose_base):
        if self.config["visual_compensation_mode"] == "yaml_delta":
            return self.generate_yaml_delta_grasp_poses(aruco_pose_base)
        if self.config["visual_compensation_mode"] == "aruco_geometry":
            return self.generate_aruco_geometry_grasp_poses(aruco_pose_base)
        if self.config["visual_compensation_mode"] == "absolute_yaml_approach":
            return self.generate_absolute_yaml_approach_grasp_poses(aruco_pose_base)

        aruco_xyz = xyz_from_pose(aruco_pose_base)
        offset = self.config["aruco_to_grasp_offset"]
        grasp = self.add(aruco_xyz, offset)
        pre_grasp = self.add(grasp, [0.0, 0.0, self.config["pre_grasp_lift"]])
        lift = self.add(grasp, [0.0, 0.0, self.config["lift_height"]])

        self.config["poses"]["pre_grasp"] = self.pose_record_with_translation(
            "pre_grasp", pre_grasp
        )
        self.config["poses"]["grasp"] = self.pose_record_with_translation("grasp", grasp)
        self.config["poses"]["lift"] = self.pose_record_with_translation("lift", lift)

        rospy.loginfo("Dynamic grasp points in %s:", self.base_frame)
        rospy.loginfo(
            "  ArUco      [%.3f, %.3f, %.3f]",
            aruco_xyz[0],
            aruco_xyz[1],
            aruco_xyz[2],
        )
        rospy.loginfo(
            "  offset     [%.3f, %.3f, %.3f]",
            offset[0],
            offset[1],
            offset[2],
        )
        rospy.loginfo(
            "  pre_grasp  [%.3f, %.3f, %.3f]",
            pre_grasp[0],
            pre_grasp[1],
            pre_grasp[2],
        )
        rospy.loginfo("  grasp      [%.3f, %.3f, %.3f]", grasp[0], grasp[1], grasp[2])
        rospy.loginfo("  lift       [%.3f, %.3f, %.3f]", lift[0], lift[1], lift[2])
        rospy.loginfo(
            "Object height %.3fm, box height %.3fm",
            self.config["object_height"],
            self.config["box_height"],
        )
        return pre_grasp, grasp, lift

    def generate_aruco_geometry_grasp_poses(self, aruco_pose_base):
        aruco_xyz = xyz_from_pose(aruco_pose_base)
        marker_to_center = self.config["marker_to_object_center"]
        object_center = self.add(aruco_xyz, marker_to_center)

        expected_z = self.config.get("expected_object_center_z")
        if expected_z is not None:
            z_error = float(expected_z) - object_center[2]
            rospy.loginfo(
                "Hand-eye z check: visual object center z=%.3f expected z=%.3f error=%.3fm",
                object_center[2],
                expected_z,
                z_error,
            )
            if self.config.get("apply_handeye_z_correction", False):
                rospy.logwarn(
                    "Applying z correction %.3fm to visual object center.", z_error
                )
                object_center[2] = float(expected_z)
            elif abs(z_error) > self.config["max_handeye_z_error"]:
                raise RuntimeError(
                    "Hand-eye z error {:.3f}m exceeds {:.3f}m. "
                    "Refusing to execute without --apply-handeye-z-correction.".format(
                        z_error,
                        self.config["max_handeye_z_error"],
                    )
                )

        pre_grasp = self.add(object_center, self.config["geometry_approach_vector"])
        grasp = list(object_center)
        lift = self.add(object_center, self.config["geometry_lift_vector"])
        source = self.config["geometry_orientation_source"]
        if source not in self.config["poses"]:
            raise RuntimeError(
                "geometry_orientation_source {!r} is not a pose in the config.".format(
                    source
                )
            )

        self.config["poses"]["pre_grasp"] = self.pose_record_with_translation(
            source, pre_grasp
        )
        self.config["poses"]["grasp"] = self.pose_record_with_translation(source, grasp)
        self.config["poses"]["lift"] = self.pose_record_with_translation(source, lift)

        size = self.config["object_size"]
        rospy.loginfo("ArUco geometry grasp in %s:", self.base_frame)
        rospy.loginfo(
            "  ArUco top center [%.3f, %.3f, %.3f]",
            aruco_xyz[0],
            aruco_xyz[1],
            aruco_xyz[2],
        )
        rospy.loginfo(
            "  object size      [%.3f, %.3f, %.3f]",
            size[0],
            size[1],
            size[2],
        )
        rospy.loginfo(
            "  marker->center   [%.3f, %.3f, %.3f]",
            marker_to_center[0],
            marker_to_center[1],
            marker_to_center[2],
        )
        rospy.loginfo(
            "  object center    [%.3f, %.3f, %.3f]",
            object_center[0],
            object_center[1],
            object_center[2],
        )
        rospy.loginfo(
            "  pre_grasp        [%.3f, %.3f, %.3f] vector=%s",
            pre_grasp[0],
            pre_grasp[1],
            pre_grasp[2],
            [round(value, 3) for value in self.config["geometry_approach_vector"]],
        )
        rospy.loginfo(
            "  grasp            [%.3f, %.3f, %.3f]",
            grasp[0],
            grasp[1],
            grasp[2],
        )
        rospy.loginfo(
            "  lift             [%.3f, %.3f, %.3f] vector=%s",
            lift[0],
            lift[1],
            lift[2],
            [round(value, 3) for value in self.config["geometry_lift_vector"]],
        )
        rospy.loginfo("  orientation from fixed pose: %s", source)
        return pre_grasp, grasp, lift

    def generate_absolute_yaml_approach_grasp_poses(self, aruco_pose_base):
        aruco_xyz = xyz_from_pose(aruco_pose_base)
        offset = self.config["aruco_to_grasp_offset"]
        grasp = self.add(aruco_xyz, offset)

        fixed_grasp = list(self.config["poses"]["grasp"]["translation"])
        approach_delta = [
            self.config["poses"]["pre_grasp"]["translation"][index] - fixed_grasp[index]
            for index in range(3)
        ]
        lift_delta = [
            self.config["poses"]["lift"]["translation"][index] - fixed_grasp[index]
            for index in range(3)
        ]
        pre_grasp = self.add(grasp, approach_delta)
        lift = self.add(grasp, lift_delta)

        self.config["poses"]["pre_grasp"] = self.pose_record_with_translation(
            "pre_grasp", pre_grasp
        )
        self.config["poses"]["grasp"] = self.pose_record_with_translation("grasp", grasp)
        self.config["poses"]["lift"] = self.pose_record_with_translation("lift", lift)

        rospy.loginfo("Marker-absolute grasp with taught YAML approach in %s:", self.base_frame)
        rospy.loginfo(
            "  ArUco      [%.3f, %.3f, %.3f]",
            aruco_xyz[0],
            aruco_xyz[1],
            aruco_xyz[2],
        )
        rospy.loginfo(
            "  offset     [%.3f, %.3f, %.3f]",
            offset[0],
            offset[1],
            offset[2],
        )
        rospy.loginfo("  grasp      [%.3f, %.3f, %.3f]", grasp[0], grasp[1], grasp[2])
        rospy.loginfo(
            "  approach d [%.3f, %.3f, %.3f]",
            approach_delta[0],
            approach_delta[1],
            approach_delta[2],
        )
        rospy.loginfo(
            "  pre_grasp  [%.3f, %.3f, %.3f]",
            pre_grasp[0],
            pre_grasp[1],
            pre_grasp[2],
        )
        rospy.loginfo(
            "  lift       [%.3f, %.3f, %.3f]",
            lift[0],
            lift[1],
            lift[2],
        )
        return pre_grasp, grasp, lift

    def generate_yaml_delta_grasp_poses(self, aruco_pose_base):
        nominal = self.config.get("nominal_marker_base")
        if nominal is None:
            raise RuntimeError(
                "yaml_delta mode requires --nominal-marker-base X Y Z."
            )
        aruco_xyz = xyz_from_pose(aruco_pose_base)
        raw_delta = [aruco_xyz[index] - nominal[index] for index in range(3)]
        axes = self.config["compensation_axes"]
        delta = [
            raw_delta[0] if "x" in axes else 0.0,
            raw_delta[1] if "y" in axes else 0.0,
            raw_delta[2] if "z" in axes else 0.0,
        ]
        max_delta = self.config["max_visual_delta"]
        if distance([0.0, 0.0, 0.0], delta) > max_delta:
            raise RuntimeError(
                "Visual delta {} exceeds max_visual_delta {:.3f}m".format(
                    [round(value, 3) for value in delta], max_delta
                )
            )

        fixed = {
            name: list(self.config["poses"][name]["translation"])
            for name in ("pre_grasp", "grasp", "lift")
        }
        pre_grasp = self.add(fixed["pre_grasp"], delta)
        grasp = self.add(fixed["grasp"], delta)
        lift = self.add(fixed["lift"], delta)

        self.config["poses"]["pre_grasp"] = self.pose_record_with_translation(
            "pre_grasp", pre_grasp
        )
        self.config["poses"]["grasp"] = self.pose_record_with_translation("grasp", grasp)
        self.config["poses"]["lift"] = self.pose_record_with_translation("lift", lift)

        rospy.loginfo("YAML-relative visual compensation in %s:", self.base_frame)
        rospy.loginfo(
            "  nominal marker [%.3f, %.3f, %.3f]",
            nominal[0],
            nominal[1],
            nominal[2],
        )
        rospy.loginfo(
            "  current marker [%.3f, %.3f, %.3f]",
            aruco_xyz[0],
            aruco_xyz[1],
            aruco_xyz[2],
        )
        rospy.loginfo(
            "  raw delta      [%.3f, %.3f, %.3f]",
            raw_delta[0],
            raw_delta[1],
            raw_delta[2],
        )
        rospy.loginfo(
            "  applied delta  [%.3f, %.3f, %.3f] axes=%s",
            delta[0],
            delta[1],
            delta[2],
            axes,
        )
        rospy.loginfo(
            "  pre_grasp      [%.3f, %.3f, %.3f] from fixed %s",
            pre_grasp[0],
            pre_grasp[1],
            pre_grasp[2],
            [round(value, 3) for value in fixed["pre_grasp"]],
        )
        rospy.loginfo(
            "  grasp          [%.3f, %.3f, %.3f] from fixed %s",
            grasp[0],
            grasp[1],
            grasp[2],
            [round(value, 3) for value in fixed["grasp"]],
        )
        rospy.loginfo(
            "  lift           [%.3f, %.3f, %.3f] from fixed %s",
            lift[0],
            lift[1],
            lift[2],
            [round(value, 3) for value in fixed["lift"]],
        )
        return pre_grasp, grasp, lift

    def cancel_motion(self):
        cancel = GoalID()
        cancel.stamp = rospy.Time(0)
        self.execute_cancel_pub.publish(cancel)
        self.controller_cancel_pub.publish(cancel)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.sleep(0.2)

    def move(self, name):
        try:
            return super().move(name)
        except Exception:
            self.cancel_motion()
            raise

    def verify_grasp_or_recover(self):
        if (not self.execute) or (not self.config.get("require_grasp_detection", False)):
            return
        topic = self.config["gripper_status_topic"]
        deadline = time.time() + 4.0
        last_status = None
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                last_status = rospy.wait_for_message(
                    topic, Robotiq2FGripper_robot_input, timeout=0.5
                )
            except rospy.ROSException:
                continue
            rospy.loginfo(
                "Gripper status after close: gOBJ=%d gPR=%d gPO=%d gCU=%d",
                last_status.gOBJ,
                last_status.gPR,
                last_status.gPO,
                last_status.gCU,
            )
            if last_status.gOBJ == 2:
                rospy.loginfo("Robotiq reports object detected while closing.")
                return
            if last_status.gOBJ == 3:
                break

        if last_status is None:
            message = "No Robotiq status received on {}".format(topic)
        else:
            message = (
                "Robotiq did not report a grasp: gOBJ={} gPR={} gPO={} gCU={}".format(
                    last_status.gOBJ,
                    last_status.gPR,
                    last_status.gPO,
                    last_status.gCU,
                )
            )
        rospy.logerr("%s", message)
        self.command_gripper("open")
        for pose_name in ("lift", "retreat", "home"):
            try:
                self.move(pose_name)
            except Exception as exc:
                rospy.logwarn("Recovery move %s failed: %s", pose_name, exc)
        raise RuntimeError(message)

    def run(self):
        self.setup_io_gripper()
        self.command_gripper("open")
        if self.config.get("skip_initial_home", False):
            rospy.loginfo("Skipping initial home move; using current robot pose for visual sampling.")
        else:
            self.move("home")
        aruco_pose_base = self.wait_for_aruco_pose_base()
        self.generate_dynamic_grasp_poses(aruco_pose_base)
        for pose_name in ("pre_grasp", "grasp"):
            self.move(pose_name)
        self.command_gripper("close")
        self.verify_grasp_or_recover()
        self.move("lift")
        if self.config.get("stop_after_lift", False):
            rospy.loginfo("Stopping after lift as requested by config.")
            return
        for pose_name in ("pre_place", "place"):
            self.move(pose_name)
        self.command_gripper("open")
        for pose_name in ("retreat", "home"):
            self.move(pose_name)


def main():
    args = parse_args()
    config = parse_task_config(args.config, args)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError(
            "Real execution is locked. Set enabled: true in the selected YAML first."
        )

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("visual_aruco_pick_place")

    if args.execute and not args.yes:
        answer = input("Workspace clear, E-stop reachable, and RViz plans checked? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return

    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    rospy.loginfo("Fixed pose config: %s", args.config)
    if config.get("selected_object_name"):
        rospy.loginfo("Selected fixed place object: %s", config["selected_object_name"])
    try:
        VisualArucoPickPlace(config, args.execute).run()
        rospy.loginfo("Visual ArUco pick-and-place cycle complete.")
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
