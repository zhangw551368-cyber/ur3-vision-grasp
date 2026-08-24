#!/usr/bin/python3

"""Guarded three-block pick/place planner for the right UR3 arm."""

import argparse
import os
import re
import sys
import time

import moveit_commander
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
import yaml
import tf.transformations
from geometry_msgs.msg import PointStamped, PoseArray
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import String

# catkin's devel-space executable is a Python relay.  When this script imports
# the shared visual-pick implementation, prefer the real source directory over
# the neighbouring relay, whose symbols live in an isolated exec() context.
SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from right_arm_visual_pick import RightArmVisualPick, load_config, make_gripper_command


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument(
        "--accept-provisional-board",
        action="store_true",
        help="Explicitly accept the locked provisional checkerboard calibration.",
    )
    parser.add_argument(
        "--external-scene-validated",
        action="store_true",
        help="Use when an external live check has validated targets after board occlusion.",
    )
    parser.add_argument(
        "--reuse-targets",
        action="store_true",
        help="In plan-only mode, reuse the previously locked target file.",
    )
    parser.add_argument(
        "--targets-file",
        default="/tmp/right_arm_three_blue_locked_targets.yaml",
        help="Plan-only writes this file; real execution reuses it.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class TargetLock:
    def __init__(self, config):
        self.config = config
        self.frame = config.get("planning_frame", "base")
        self.object_topic = config["object_topic"]
        self.place_topic = config["place_topic"]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform_array(
        self, message, include_axis_yaws=False, allow_variable_count=False
    ):
        if not allow_variable_count and len(message.poses) != 3:
            raise RuntimeError(
                "Expected exactly three poses on {}, got {}".format(
                    self.object_topic, len(message.poses)
                )
            )
        if not message.header.frame_id:
            raise RuntimeError("PoseArray frame_id is empty")
        transform = None
        frame_rotation = np.eye(3)
        if message.header.frame_id != self.frame:
            transform = self.tf_buffer.lookup_transform(
                self.frame,
                message.header.frame_id,
                rospy.Time(0),
                rospy.Duration(float(self.config.get("tf_timeout", 0.5))),
            )
            transform_quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            frame_rotation = tf.transformations.quaternion_matrix(
                transform_quaternion
            )[:3, :3]
        result = []
        axis_yaws = []
        for pose in message.poses:
            if transform is None:
                result.append([pose.position.x, pose.position.y, pose.position.z])
            else:
                point = PointStamped()
                point.header = message.header
                point.header.stamp = rospy.Time(0)
                point.point = pose.position
                converted = tf2_geometry_msgs.do_transform_point(point, transform).point
                result.append([converted.x, converted.y, converted.z])
            if include_axis_yaws:
                pose_quaternion = [
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ]
                axis = frame_rotation.dot(
                    tf.transformations.quaternion_matrix(pose_quaternion)[:3, 0]
                )
                yaw = np.arctan2(axis[1], axis[0])
                # Long axes are equivalent modulo pi.
                yaw = (yaw + np.pi / 2.0) % np.pi - np.pi / 2.0
                axis_yaws.append(yaw)
        if include_axis_yaws:
            return np.asarray(result), np.asarray(axis_yaws)
        return np.asarray(result)

    @staticmethod
    def circular_axis_mean(values):
        values = np.asarray(values, dtype=float)
        return 0.5 * np.arctan2(
            np.mean(np.sin(2.0 * values)),
            np.mean(np.cos(2.0 * values)),
        )

    def refresh_object(self, reference_xyz, reference_yaw, object_label):
        """Acquire the latest camera target, then freeze it for one pick."""
        count = int(self.config.get("per_pick_lock_sample_count", 30))
        timeout = float(self.config.get("per_pick_lock_timeout", 12.0))
        association_radius = float(
            self.config.get("per_pick_association_radius", 0.080)
        )
        samples = []
        yaws = []
        deadline = time.time() + timeout
        reference_xyz = np.asarray(reference_xyz, dtype=float)
        while len(samples) < count and time.time() < deadline and not rospy.is_shutdown():
            try:
                message = rospy.wait_for_message(
                    self.object_topic, PoseArray, timeout=1.0
                )
                points, candidate_yaws = self.transform_array(
                    message,
                    include_axis_yaws=True,
                    allow_variable_count=True,
                )
                if len(points) == 0:
                    continue
                distances = np.linalg.norm(points - reference_xyz, axis=1)
                candidate_index = int(np.argmin(distances))
                if distances[candidate_index] > association_radius:
                    continue
                samples.append(points[candidate_index])
                yaws.append(candidate_yaws[candidate_index])
            except (rospy.ROSException, tf2_ros.TransformException, RuntimeError) as exc:
                rospy.logwarn_throttle(
                    1.0, "Refreshing block %d target: %s", object_label, exc
                )
        if len(samples) < count:
            raise RuntimeError(
                "Only received {}/{} associated live samples for block {}".format(
                    len(samples), count, object_label
                )
            )
        samples = np.asarray(samples)
        seed = np.median(samples, axis=0)
        errors = np.linalg.norm(samples - seed, axis=1)
        inliers = samples[
            errors <= float(self.config.get("object_inlier_radius", 0.012))
        ]
        min_inliers = int(np.ceil(count * float(self.config.get("min_inlier_ratio", 0.70))))
        if len(inliers) < min_inliers:
            raise RuntimeError(
                "Live block {} target unstable: {}/{} inliers".format(
                    object_label, len(inliers), count
                )
            )
        locked_xyz = np.median(inliers, axis=0)
        mad = np.median(np.abs(inliers - locked_xyz), axis=0)
        if np.max(mad) > float(self.config.get("object_max_mad", 0.005)):
            raise RuntimeError(
                "Live block {} target unstable: MAD={}mm".format(
                    object_label, np.round(mad * 1000.0, 2).tolist()
                )
            )
        inlier_mask = errors <= float(
            self.config.get("object_inlier_radius", 0.012)
        )
        locked_yaw = self.circular_axis_mean(np.asarray(yaws)[inlier_mask])
        yaw_delta = abs(
            (locked_yaw - reference_yaw + np.pi / 2.0) % np.pi - np.pi / 2.0
        )
        rospy.loginfo(
            "Block %d latest camera target locked before motion: xyz=%s "
            "shift_mm=%.2f long_axis=%.2fdeg axis_shift=%.2fdeg MAD_mm=%s",
            object_label,
            np.round(locked_xyz, 6).tolist(),
            np.linalg.norm(locked_xyz - reference_xyz) * 1000.0,
            np.rad2deg(locked_yaw),
            np.rad2deg(yaw_delta),
            np.round(mad * 1000.0, 2).tolist(),
        )
        return locked_xyz, locked_yaw

    def sample_topic(self, topic, count, timeout):
        samples = []
        deadline = time.time() + timeout
        while len(samples) < count and time.time() < deadline and not rospy.is_shutdown():
            try:
                message = rospy.wait_for_message(topic, PoseArray, timeout=1.0)
                samples.append(self.transform_array(message))
            except (rospy.ROSException, tf2_ros.TransformException, RuntimeError) as exc:
                rospy.logwarn_throttle(1.0, "Waiting for %s: %s", topic, exc)
        if len(samples) < count:
            raise RuntimeError(
                "Only received {}/{} valid samples from {}".format(
                    len(samples), count, topic
                )
            )
        return np.asarray(samples)

    def sample_object_topic(self, count, timeout):
        point_samples = []
        yaw_samples = []
        deadline = time.time() + timeout
        while (
            len(point_samples) < count
            and time.time() < deadline
            and not rospy.is_shutdown()
        ):
            try:
                message = rospy.wait_for_message(
                    self.object_topic, PoseArray, timeout=1.0
                )
                points, yaws = self.transform_array(
                    message, include_axis_yaws=True
                )
                point_samples.append(points)
                yaw_samples.append(yaws)
            except (rospy.ROSException, tf2_ros.TransformException, RuntimeError) as exc:
                rospy.logwarn_throttle(
                    1.0, "Waiting for %s: %s", self.object_topic, exc
                )
        if len(point_samples) < count:
            raise RuntimeError(
                "Only received {}/{} valid samples from {}".format(
                    len(point_samples), count, self.object_topic
                )
            )
        return np.asarray(point_samples), np.asarray(yaw_samples)

    def robust_lock(
        self, samples, label, radius, min_inlier_ratio, max_mad,
        required_indices=None,
    ):
        required_indices = set(range(3) if required_indices is None else required_indices)
        locked = []
        for index in range(3):
            values = samples[:, index, :]
            seed = np.median(values, axis=0)
            errors = np.linalg.norm(values - seed, axis=1)
            inliers = values[errors <= radius]
            ratio = float(len(inliers)) / float(len(values))
            if index in required_indices and ratio < min_inlier_ratio:
                raise RuntimeError(
                    "{} {} unstable: {:.0%} inliers, need {:.0%}".format(
                        label, index + 1, ratio, min_inlier_ratio
                    )
                )
            point = np.median(inliers, axis=0)
            mad = np.median(np.abs(inliers - point), axis=0)
            if index in required_indices and np.max(mad) > max_mad:
                raise RuntimeError(
                    "{} {} unstable: MAD={} mm".format(
                        label, index + 1, np.round(mad * 1000.0, 2).tolist()
                    )
                )
            rospy.loginfo(
                "Locked %s %d xyz=[%.6f, %.6f, %.6f], inliers=%d/%d MAD_mm=%s",
                label,
                index + 1,
                point[0], point[1], point[2],
                len(inliers), len(values),
                np.round(mad * 1000.0, 2).tolist(),
            )
            locked.append(point)
        return np.asarray(locked)

    def robust_axis_lock(self, samples, required_indices=None):
        required_indices = set(
            range(3) if required_indices is None else required_indices
        )
        max_error = np.deg2rad(
            float(self.config.get("object_axis_inlier_deg", 10.0))
        )
        max_mad = np.deg2rad(
            float(self.config.get("object_axis_max_mad_deg", 3.0))
        )
        min_ratio = float(self.config.get("min_inlier_ratio", 0.70))
        locked = []
        for index in range(3):
            values = samples[:, index]
            seed = 0.5 * np.arctan2(
                np.mean(np.sin(2.0 * values)),
                np.mean(np.cos(2.0 * values)),
            )
            errors = np.abs(
                (values - seed + np.pi / 2.0) % np.pi - np.pi / 2.0
            )
            inliers = values[errors <= max_error]
            ratio = float(len(inliers)) / float(len(values))
            if index in required_indices and ratio < min_ratio:
                raise RuntimeError(
                    "object axis {} unstable: {:.0%} inliers".format(
                        index + 1, ratio
                    )
                )
            yaw = 0.5 * np.arctan2(
                np.mean(np.sin(2.0 * inliers)),
                np.mean(np.cos(2.0 * inliers)),
            )
            deviations = np.abs(
                (inliers - yaw + np.pi / 2.0) % np.pi - np.pi / 2.0
            )
            mad = float(np.median(deviations))
            if index in required_indices and mad > max_mad:
                raise RuntimeError(
                    "object axis {} unstable: MAD={:.2f}deg".format(
                        index + 1, np.rad2deg(mad)
                    )
                )
            rospy.loginfo(
                "Locked object axis %d yaw=%.2fdeg, inliers=%d/%d MAD=%.2fdeg",
                index + 1, np.rad2deg(yaw), len(inliers), len(values),
                np.rad2deg(mad),
            )
            locked.append(yaw)
        return np.asarray(locked)

    def collect(self):
        count = int(self.config.get("lock_sample_count", 40))
        timeout = float(self.config.get("lock_timeout", 15.0))
        objects_raw, object_yaws_raw = self.sample_object_topic(count, timeout)
        places_raw = self.sample_topic(self.place_topic, count, timeout)
        active_indices = [
            int(value)
            for value in self.config.get("active_object_indices", [0, 1, 2])
        ]
        max_blocks = min(
            int(self.config.get("max_blocks", 3)), len(active_indices)
        )
        object_indices = active_indices[:max_blocks]
        place_indices = [
            int(v) for v in self.config.get("placement_indices", [0, 1, 2])
        ][:max_blocks]
        objects = self.robust_lock(
            objects_raw,
            "object",
            float(self.config.get("object_inlier_radius", 0.012)),
            float(self.config.get("min_inlier_ratio", 0.70)),
            float(self.config.get("object_max_mad", 0.004)),
            required_indices=object_indices,
        )
        object_yaws = self.robust_axis_lock(
            object_yaws_raw, required_indices=object_indices
        )
        places = self.robust_lock(
            places_raw,
            "place",
            float(self.config.get("place_inlier_radius", 0.006)),
            float(self.config.get("min_inlier_ratio", 0.70)),
            float(self.config.get("place_max_mad", 0.002)),
            required_indices=place_indices,
        )
        return objects, places, object_yaws


def board_plane(places):
    matrix = np.column_stack((places[:, 0], places[:, 1], np.ones(3)))
    return np.linalg.solve(matrix, places[:, 2])


def geometry_from_points(config, objects, places):
    a, b, c = board_plane(places)
    heights = objects[:, 2] - (a * objects[:, 0] + b * objects[:, 1] + c)
    low, high = config.get("object_height_bounds", [0.015, 0.080])
    if np.any(heights < low) or np.any(heights > high):
        raise RuntimeError(
            "Object heights outside [{:.0f}, {:.0f}] mm: {}".format(
                low * 1000.0, high * 1000.0, np.round(heights * 1000.0, 1).tolist()
            )
        )
    return heights


def write_targets(path, frame, objects, places, heights, object_yaws, config):
    payload = {
        "frame_id": frame,
        "object_top_points": objects.tolist(),
        "place_surface_points": places.tolist(),
        "object_heights": heights.tolist(),
        "object_long_axis_yaws": object_yaws.tolist(),
        "square_size_m": float(config.get("checkerboard_square_size_m", 0.0)),
        "square_size_confirmed": bool(config.get("checkerboard_square_size_confirmed", False)),
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, sort_keys=False)
    rospy.loginfo("Locked targets saved to %s", path)


def load_targets(path):
    with open(path, "r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    objects = np.asarray(payload["object_top_points"], dtype=float)
    return (
        payload,
        objects,
        np.asarray(payload["place_surface_points"], dtype=float),
        np.asarray(payload["object_heights"], dtype=float),
        np.asarray(payload.get("object_long_axis_yaws", [0.0] * len(objects)), dtype=float),
    )


def validate_live(locker, locked_objects, locked_places, locked_yaws, config):
    live_objects, live_places, live_yaws = locker.collect()
    object_shift = np.linalg.norm(live_objects - locked_objects, axis=1)
    place_shift = np.linalg.norm(live_places - locked_places, axis=1)
    max_object_shift = float(config.get("execute_object_max_shift", 0.012))
    max_place_shift = float(config.get("execute_place_max_shift", 0.008))
    yaw_shift = np.abs(
        (live_yaws - locked_yaws + np.pi / 2.0) % np.pi - np.pi / 2.0
    )
    max_yaw_shift = np.deg2rad(
        float(config.get("execute_object_axis_max_shift_deg", 8.0))
    )
    active_indices = [
        int(value) for value in config.get("active_object_indices", [0, 1, 2])
    ]
    max_blocks = min(int(config.get("max_blocks", 3)), len(active_indices))
    object_indices = active_indices[:max_blocks]
    place_indices = [
        int(v) for v in config.get("placement_indices", [0, 1, 2])
    ][:max_blocks]
    active_object_shift = object_shift[object_indices]
    active_place_shift = place_shift[place_indices]
    if (
        np.max(active_object_shift) > max_object_shift
        or np.max(active_place_shift) > max_place_shift
        or np.max(yaw_shift[object_indices]) > max_yaw_shift
    ):
        raise RuntimeError(
            "Scene moved since planning: object_shift_mm={} place_shift_mm={} "
            "axis_shift_deg={}".format(
                np.round(active_object_shift * 1000.0, 1).tolist(),
                np.round(active_place_shift * 1000.0, 1).tolist(),
                np.round(np.rad2deg(yaw_shift[object_indices]), 1).tolist(),
            )
        )
    rospy.loginfo(
        "Live scene matches locked plan: object_shift_mm=%s place_shift_mm=%s "
        "axis_shift_deg=%s",
        np.round(active_object_shift * 1000.0, 1).tolist(),
        np.round(active_place_shift * 1000.0, 1).tolist(),
        np.round(np.rad2deg(yaw_shift[object_indices]), 1).tolist(),
    )


def validate_live_places(locker, locked_places, config):
    """Validate the fixed checkerboard while objects refresh per pick."""
    count = int(config.get("lock_sample_count", 40))
    timeout = float(config.get("lock_timeout", 15.0))
    placement_indices = [
        int(value)
        for value in config.get("placement_indices", [0, 1, 2])
    ]
    max_blocks = min(int(config.get("max_blocks", 3)), len(placement_indices))
    placement_indices = placement_indices[:max_blocks]
    live_places = locker.robust_lock(
        locker.sample_topic(locker.place_topic, count, timeout),
        "place",
        float(config.get("place_inlier_radius", 0.006)),
        float(config.get("min_inlier_ratio", 0.70)),
        float(config.get("place_max_mad", 0.002)),
        required_indices=placement_indices,
    )
    shifts = np.linalg.norm(live_places - locked_places, axis=1)[
        placement_indices
    ]
    limit = float(config.get("execute_place_max_shift", 0.008))
    if np.max(shifts) > limit:
        raise RuntimeError(
            "Checkerboard moved since planning: place_shift_mm={}".format(
                np.round(shifts * 1000.0, 1).tolist()
            )
        )
    rospy.loginfo(
        "Checkerboard matches locked plan: place_shift_mm=%s; "
        "block 1 keeps the planned target; later blocks refresh after each release",
        np.round(shifts * 1000.0, 1).tolist(),
    )


def initialize_right_gripper(picker, config):
    """Run the required Robotiq reset -> activate/open sequence."""
    if not picker.execute:
        rospy.loginfo(
            "PLAN ONLY gripper initialization preview: reset -> activate/open rPR=%d",
            int(config.get("open_position", 0)),
        )
        picker.publish_gripper(config.get("open_position", 0), "open preview")
        return

    deadline = time.time() + 5.0
    while picker.gripper.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.05)
    if picker.gripper.get_num_connections() == 0:
        raise RuntimeError("Right gripper driver is not subscribed")

    speed = int(config.get("gripper_speed", 120))
    force = int(config.get("gripper_force", 80))
    reset = Robotiq2FGripper_robot_output()
    reset.rACT = 0
    reset.rGTO = 0
    reset.rATR = 0
    reset.rPR = 0
    reset.rSP = speed
    reset.rFR = force
    rospy.loginfo("Gripper initialization: reset rACT=0")
    reset_deadline = time.time() + float(config.get("gripper_reset_duration", 1.0))
    while time.time() < reset_deadline:
        picker.gripper.publish(reset)
        rospy.sleep(0.1)

    rospy.sleep(float(config.get("gripper_reset_settle", 0.5)))
    activate = make_gripper_command(
        int(config.get("open_position", 0)), speed, force
    )
    rospy.loginfo("Gripper initialization: activate/open rACT=1 rPR=%d", activate.rPR)
    activate_deadline = time.time() + float(
        config.get("gripper_activate_duration", 2.0)
    )
    while time.time() < activate_deadline:
        picker.gripper.publish(activate)
        rospy.sleep(0.1)

    status_topic = config.get(
        "gripper_status_topic", "/right_arm/Robotiq2FGripperRobotInput"
    )
    status_deadline = time.time() + 3.0
    last = None
    open_position = int(config.get("open_position", 0))
    open_tolerance = int(config.get("gripper_open_tolerance", 5))
    while time.time() < status_deadline:
        try:
            last = rospy.wait_for_message(
                status_topic, Robotiq2FGripper_robot_input, timeout=0.5
            )
        except rospy.ROSException:
            continue
        # gSTA=3 only means activation completed.  Wait until the fingers have
        # actually stopped at the requested open position as well; otherwise
        # the arm can start moving while the gripper is still opening.
        if (
            last.gFLT == 0
            and last.gACT == 1
            and last.gSTA == 3
            and last.gOBJ == 3
            and abs(int(last.gPO) - open_position) <= open_tolerance
        ):
            rospy.loginfo(
                "Right gripper ready: gACT=%d gSTA=%d gOBJ=%d gFLT=%d gPO=%d",
                last.gACT, last.gSTA, last.gOBJ, last.gFLT, last.gPO,
            )
            return
    if last is None:
        raise RuntimeError("No right gripper status after activation")
    raise RuntimeError(
        "Right gripper activation failed: gACT={} gSTA={} gOBJ={} gFLT={} gPO={}".format(
            last.gACT, last.gSTA, last.gOBJ, last.gFLT, last.gPO
        )
    )


def top_down_orientation_for_long_axis(long_axis_yaw, yaw_offset=0.0):
    """Point tool Z down and tool X along a block's long side.

    The Robotiq fingers are separated along tool Y, so this makes their closing
    direction perpendicular to the long axis and places the pads on the two
    larger long side faces.
    """
    yaw = float(long_axis_yaw) + float(yaw_offset)
    x_axis = np.asarray([np.cos(yaw), np.sin(yaw), 0.0])
    z_axis = np.asarray([0.0, 0.0, -1.0])
    y_axis = np.cross(z_axis, x_axis)
    matrix = np.eye(4)
    matrix[:3, 0] = x_axis
    matrix[:3, 1] = y_axis
    matrix[:3, 2] = z_axis
    return tf.transformations.quaternion_from_matrix(matrix)


def outward_tilt_orientation(xyz, tilt_radians, roll_radians=0.0):
    """Point tool Z downward with an outward radial tilt at xyz."""
    radial = np.asarray([xyz[0], xyz[1], 0.0], dtype=float)
    radial /= np.linalg.norm(radial)
    z_axis = np.asarray(
        [
            radial[0] * np.sin(tilt_radians),
            radial[1] * np.sin(tilt_radians),
            -np.cos(tilt_radians),
        ]
    )
    reference = np.asarray([1.0, 0.0, 0.0])
    y_axis = reference - np.dot(reference, z_axis) * z_axis
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    rotated_x = np.cos(roll_radians) * x_axis + np.sin(roll_radians) * y_axis
    rotated_y = -np.sin(roll_radians) * x_axis + np.cos(roll_radians) * y_axis
    matrix = np.eye(4)
    matrix[:3, 0] = rotated_x
    matrix[:3, 1] = rotated_y
    matrix[:3, 2] = z_axis
    return tf.transformations.quaternion_from_matrix(matrix)


def plan_or_execute(
    config, execute, objects, places, heights, object_yaws, locker=None
):
    picker = RightArmVisualPick(config, execute)
    pick_orientation = list(config["orientation_quaternion"])
    place_orientation = list(config.get("place_orientation_quaternion", pick_orientation))
    picker.ensure_external_control()
    initialize_right_gripper(picker, config)
    pick_offset = float(config.get("grasp_above_center", 0.020))
    release_clearance = float(config.get("release_clearance", 0.003))
    release_tcp_clearance = float(
        config.get("release_tcp_clearance", 0.0)
    )
    pre_pick = float(config.get("pre_pick_clearance", 0.180))
    lift_distance = float(config.get("lift_distance", 0.140))
    pre_place = float(config.get("pre_place_clearance", 0.150))

    placement_indices = [int(v) for v in config.get("placement_indices", [0, 1, 2])]
    active_indices = [
        int(value) for value in config.get("active_object_indices", [0, 1, 2])
    ]
    max_blocks = min(
        int(config.get("max_blocks", 3)),
        len(active_indices),
        len(placement_indices),
    )
    tasks = [
        (
            object_index,
            objects[object_index],
            places[placement_indices[task_index]],
            heights[object_index],
            object_yaws[object_index],
        )
        for task_index, object_index in enumerate(active_indices[:max_blocks])
    ]
    aligned_indices = {
        int(value) for value in config.get("align_long_axis_object_indices", [0, 1])
    }
    fixed_orientations = config.get("fixed_grasp_orientation_quaternions", [])
    yaw_offset = np.deg2rad(float(config.get("long_axis_yaw_offset_deg", 0.0)))
    for task_number, (object_index, obj, place, height, long_axis_yaw) in enumerate(
        tasks, start=1
    ):
        object_label = object_index + 1
        # The plan-only lock supplies all three initial targets.  Block 1 uses
        # that planned coordinate unchanged.  Only after a completed block do
        # we reacquire the next block, then freeze that new coordinate before
        # its arm motion begins.
        if (
            execute
            and task_number > 1
            and config.get("refresh_target_after_each_completed_pick", False)
        ):
            if locker is None:
                raise RuntimeError("Per-pick target refresh requested without TargetLock")
            rospy.loginfo(
                "Block %d/%d: refreshing its latest camera coordinate before arm motion",
                task_number,
                max_blocks,
            )
            obj, long_axis_yaw = locker.refresh_object(
                obj, long_axis_yaw, object_label
            )
            rospy.loginfo(
                "Block %d target is frozen until this pick/release finishes",
                object_label,
            )
        fixed_orientation = None
        if object_index < len(fixed_orientations):
            fixed_orientation = fixed_orientations[object_index]
        if fixed_orientation is not None:
            if len(fixed_orientation) != 4:
                raise RuntimeError(
                    "Fixed grasp orientation for block {} must be xyzw".format(
                        object_label
                    )
                )
            picker.orientation = [float(value) for value in fixed_orientation]
            rospy.loginfo(
                "Block %d archived successful fixed grasp orientation q=%s",
                object_label,
                np.round(picker.orientation, 10).tolist(),
            )
        elif object_index in aligned_indices:
            picker.orientation = top_down_orientation_for_long_axis(
                long_axis_yaw, yaw_offset
            )
            rospy.loginfo(
                "Block %d oriented top grasp: long_axis=%.2fdeg, "
                "tool_X parallel / finger_close_Y perpendicular, q=%s",
                object_label, np.rad2deg(long_axis_yaw),
                np.round(picker.orientation, 6).tolist(),
            )
        else:
            picker.orientation = pick_orientation
            rospy.loginfo("Block %d standard top grasp (cube)", object_label)
        grasp = obj.copy()
        grasp[2] = obj[2] - height / 2.0 + pick_offset
        grasp_pre = grasp + np.asarray([0.0, 0.0, pre_pick])
        lift = grasp + np.asarray([0.0, 0.0, lift_distance])
        release = place.copy()
        release[2] = place[2] + height / 2.0 + pick_offset + release_clearance
        if release_tcp_clearance > 0.0:
            release[2] = max(
                release[2], place[2] + release_tcp_clearance
            )
        release_pre = release + np.asarray([0.0, 0.0, pre_place])

        rospy.loginfo(
            "Block %d height=%.1fmm grasp=%s release=%s "
            "tcp_clearance=%.1fmm object_bottom_clearance=%.1fmm",
            object_label,
            height * 1000.0,
            np.round(grasp, 4).tolist(),
            np.round(release, 4).tolist(),
            (release[2] - place[2]) * 1000.0,
            (
                release[2]
                - place[2]
                - height / 2.0
                - pick_offset
            )
            * 1000.0,
        )
        pre_grasp_was_planned = False
        tilted_grasp_indices = {
            int(value)
            for value in config.get("tilted_grasp_object_indices", [])
        }
        if object_index in tilted_grasp_indices:
            original_retries = picker.config.get("plan_retries", 5)
            original_planning_time = float(config.get("planning_time", 6.0))
            picker.config["plan_retries"] = int(
                config.get("tilted_grasp_plan_retries", 2)
            )
            picker.arm.set_planning_time(
                float(config.get("tilted_grasp_planning_time", 3.0))
            )
            last_error = None
            try:
                for tilt_deg in config.get(
                    "grasp_outward_tilt_candidates_deg", [15.0, 25.0, 40.0]
                ):
                    picker.orientation = outward_tilt_orientation(
                        grasp, np.deg2rad(float(tilt_deg))
                    )
                    rospy.loginfo(
                        "Block %d centered tilted-grasp test: tilt=%.1fdeg q=%s",
                        object_label,
                        float(tilt_deg),
                        np.round(picker.orientation, 6).tolist(),
                    )
                    try:
                        picker.plan_to_pose("pre_grasp", grasp_pre.tolist())
                        pre_grasp_was_planned = True
                        rospy.loginfo(
                            "Block %d centered tilted grasp selected: %.1fdeg",
                            object_label,
                            float(tilt_deg),
                        )
                        break
                    except RuntimeError as exc:
                        last_error = exc
                        rospy.logwarn(
                            "Block %d tilt %.1fdeg unavailable: %s",
                            object_label,
                            float(tilt_deg),
                            exc,
                        )
            finally:
                picker.config["plan_retries"] = original_retries
                picker.arm.set_planning_time(original_planning_time)
            if not pre_grasp_was_planned:
                raise RuntimeError(
                    "No centered tilted grasp for block {}: {}".format(
                        object_label, last_error
                    )
                )
        if not pre_grasp_was_planned:
            picker.plan_to_pose("pre_grasp", grasp_pre.tolist())
        joint_grasp_indices = {
            int(value)
            for value in config.get("joint_planned_grasp_object_indices", [])
        }
        if object_index in joint_grasp_indices:
            rospy.loginfo(
                "Block %d grasp uses collision-checked joint-space planning "
                "with the same target pose",
                object_label,
            )
            picker.plan_to_pose(
                "block{}_grasp".format(object_label), grasp.tolist()
            )
        else:
            picker.cartesian_to_pose(
                "block{}_grasp".format(object_label), grasp.tolist()
            )
        picker.publish_gripper(
            config.get("close_position", 210),
            "close block {}".format(object_label),
        )
        picker.require_grasp()
        picker.cartesian_to_pose("block{}_lift".format(object_label), lift.tolist())
        release_was_planned = False
        place_mode = config.get("place_orientation_mode", "vertical")
        if place_mode == "outward_tilt":
            tilt = np.deg2rad(float(config.get("place_outward_tilt_deg", 40.0)))
            roll = np.deg2rad(float(config.get("place_roll_deg", 0.0)))
            picker.orientation = outward_tilt_orientation(release, tilt, roll)
            rospy.loginfo(
                "Block %d drop orientation: outward tilt=%.1fdeg roll=%.1fdeg q=%s",
                object_label,
                np.rad2deg(tilt),
                np.rad2deg(roll),
                np.round(picker.orientation, 6).tolist(),
            )
        elif place_mode == "vertical_yaw_scan":
            if not config.get("direct_high_release", False):
                raise RuntimeError(
                    "vertical_yaw_scan requires direct_high_release=true"
                )
            candidates = config.get(
                "place_vertical_tool_x_yaw_candidates_deg",
                [90.0, 0.0, 180.0, -90.0],
            )
            original_retries = picker.config.get("plan_retries", 5)
            original_planning_time = float(config.get("planning_time", 6.0))
            picker.config["plan_retries"] = int(
                config.get("place_vertical_plan_retries", 2)
            )
            picker.arm.set_planning_time(
                float(config.get("place_vertical_planning_time", 2.5))
            )
            last_error = None
            try:
                for candidate_deg in candidates:
                    picker.orientation = top_down_orientation_for_long_axis(
                        np.deg2rad(float(candidate_deg))
                    )
                    rospy.loginfo(
                        "Block %d vertical high-release test: tool_X_yaw=%.1fdeg q=%s",
                        object_label,
                        float(candidate_deg),
                        np.round(picker.orientation, 6).tolist(),
                    )
                    try:
                        picker.plan_to_pose(
                            "block{}_release".format(object_label),
                            release.tolist(),
                        )
                        release_was_planned = True
                        rospy.loginfo(
                            "Block %d vertical high-release selected: tool_X_yaw=%.1fdeg",
                            object_label,
                            float(candidate_deg),
                        )
                        break
                    except RuntimeError as exc:
                        last_error = exc
                        rospy.logwarn(
                            "Block %d vertical yaw %.1fdeg unavailable: %s",
                            object_label,
                            float(candidate_deg),
                            exc,
                        )
            finally:
                picker.config["plan_retries"] = original_retries
                picker.arm.set_planning_time(original_planning_time)
            if not release_was_planned:
                raise RuntimeError(
                    "No vertical high-release orientation for block {}: {}".format(
                        object_label, last_error
                    )
                )
        else:
            picker.orientation = place_orientation
        if config.get("direct_high_release", False) and not release_was_planned:
            rospy.loginfo(
                "Block %d direct high release: no downward placement segment",
                object_label,
            )
            picker.plan_to_pose("block{}_release".format(object_label), release.tolist())
        elif not release_was_planned:
            picker.plan_to_pose("block{}_pre_place".format(object_label), release_pre.tolist())
            picker.cartesian_to_pose("block{}_release".format(object_label), release.tolist())
        picker.publish_gripper(
            config.get("open_position", 0),
            "release block {}".format(object_label),
        )
        picker.grasp_confirmed = False
        picker.object_released = True
        picker.cartesian_to_pose(
            "block{}_retreat".format(object_label), release_pre.tolist()
        )

    picker.plan_to_initial_joint_target()
    return max_blocks, len(picker.display_trajectories)


def main():
    args = parse_args()
    config = load_config(args.config)
    rospy.init_node("right_arm_three_blue_pick_place")
    locker = TargetLock(config)

    if args.execute:
        if not config.get("enabled", False):
            raise RuntimeError("Real execution is locked: enabled is not true in YAML")
        payload, objects, places, heights, object_yaws = load_targets(args.targets_file)
        if payload.get("frame_id") != config.get("planning_frame", "base"):
            raise RuntimeError("Locked target frame does not match planning frame")
        if (
            not payload.get("square_size_confirmed", False)
            and not args.accept_provisional_board
        ):
            raise RuntimeError("Real execution blocked: checkerboard square size is unconfirmed")
        if not payload.get("square_size_confirmed", False):
            rospy.logwarn(
                "Operator explicitly accepted provisional checkerboard square size %.4fm",
                float(payload.get("square_size_m", 0.0)),
            )
        if config.get("use_locked_scene_without_live_revalidation", False):
            rospy.logwarn(
                "Using the three coordinates frozen during plan-only; "
                "camera targets will not be read after planning"
            )
        elif args.external_scene_validated:
            rospy.logwarn(
                "Using externally validated live object targets and locked board points"
            )
        elif config.get("refresh_target_after_each_completed_pick", False):
            validate_live_places(locker, places, config)
        else:
            validate_live(locker, objects, places, object_yaws, config)
        if not args.yes:
            answer = input("Workspace clear, E-stop reachable, targets unchanged? Type EXECUTE: ")
            if answer != "EXECUTE":
                rospy.logwarn("Cancelled")
                return
    elif args.reuse_targets:
        _, objects, places, heights, object_yaws = load_targets(args.targets_file)
        rospy.loginfo("Reusing locked plan-only targets from %s", args.targets_file)
    else:
        objects, places, object_yaws = locker.collect()
        heights = geometry_from_points(config, objects, places)
        write_targets(
            args.targets_file,
            config.get("planning_frame", "base"),
            objects,
            places,
            heights,
            object_yaws,
            config,
        )

    block_count, segment_count = plan_or_execute(
        config, args.execute, objects, places, heights, object_yaws, locker=locker
    )
    rospy.loginfo(
        "%s complete: %d block(s), %d motion segments",
        "REAL EXECUTION" if args.execute else "PLAN ONLY",
        block_count,
        segment_count,
    )


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
