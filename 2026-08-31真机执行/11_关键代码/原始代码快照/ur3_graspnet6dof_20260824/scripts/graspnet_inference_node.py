#!/usr/bin/env python3

import argparse
import json
import os
import sys
import threading
import time
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import message_filters
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from PIL import Image as PilImage
from PIL import ImageDraw
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from ur3_graspnet6dof.backend import GraspNetBackend
from ur3_graspnet6dof.config import load_config, resolve_project_path
from ur3_graspnet6dof.geometry import matrix_to_quaternion, transform_matrix
from ur3_graspnet6dof.ros_image import decode_color, decode_depth_metres, intrinsic_matrix, roi_mask


def parse_args():
    parser = argparse.ArgumentParser(description="GraspNet ROS inference node")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "right_arm_green_table.yaml"),
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class InferenceNode:
    def __init__(self, config):
        self.config = config
        self.camera = config["camera"]
        self.ros_config = config["ros"]
        self.network = config["network"]
        self.runtime_dir = resolve_project_path(config, config["paths"]["runtime_dir"])
        self.runtime_dir.mkdir(parents=True, exist_ok=True)

        rospy.loginfo("Loading GraspNet checkpoint; this can take several seconds")
        self.backend = GraspNetBackend(config)
        rospy.loginfo(
            "GraspNet ready on %s, checkpoint epoch=%d",
            self.backend.device,
            self.backend.checkpoint_epoch,
        )

        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.latest_frame = None
        self.last_inference_time = 0.0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.candidates_pub = rospy.Publisher(
            self.ros_config["candidates_json_topic"], String, queue_size=1, latch=True
        )
        self.pose_pub = rospy.Publisher(
            self.ros_config["candidates_pose_topic"], PoseArray, queue_size=1, latch=True
        )
        self.best_pub = rospy.Publisher(
            self.ros_config["best_grasp_camera_topic"], PoseStamped, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            self.ros_config["marker_topic"], MarkerArray, queue_size=1, latch=True
        )
        self.valid_pub = rospy.Publisher(
            self.ros_config["valid_topic"], Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.ros_config["status_topic"], String, queue_size=1, latch=True
        )

        color_sub = message_filters.Subscriber(self.camera["color_topic"], Image)
        depth_sub = message_filters.Subscriber(self.camera["depth_topic"], Image)
        info_sub = message_filters.Subscriber(self.camera["camera_info_topic"], CameraInfo)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=int(self.camera["sync_queue_size"]),
            slop=float(self.camera["sync_slop"]),
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.frame_callback)
        self.service = rospy.Service(
            self.ros_config["inference_service"], Trigger, self.handle_inference
        )
        self.status("waiting_for_synchronized_rgbd")

    def status(self, value):
        rospy.loginfo("GraspNet status: %s", value)
        self.status_pub.publish(String(data=value))

    def frame_callback(self, color_msg, depth_msg, info_msg):
        try:
            expected_frame = self.camera.get("expected_frame", "")
            frames = [color_msg.header.frame_id, depth_msg.header.frame_id, info_msg.header.frame_id]
            if expected_frame and any(frame != expected_frame for frame in frames):
                rospy.logwarn_throttle(
                    2.0,
                    "Rejecting RGB-D frame IDs %s; expected %s",
                    frames,
                    expected_frame,
                )
                return
            color = decode_color(color_msg)
            depth = decode_depth_metres(depth_msg)
            if color.shape[:2] != depth.shape:
                raise ValueError("RGB shape {} and depth shape {} differ".format(color.shape[:2], depth.shape))
            if info_msg.width != color.shape[1] or info_msg.height != color.shape[0]:
                raise ValueError("camera_info dimensions do not match RGB-D")
            intrinsic = intrinsic_matrix(info_msg)
            valid = roi_mask(depth.shape, self.camera["roi_normalized"])
            valid &= np.isfinite(depth)
            valid &= depth >= float(self.camera["min_depth_m"])
            valid &= depth <= float(self.camera["max_depth_m"])
            with self.frame_lock:
                self.latest_frame = {
                    "color": color,
                    "depth": depth,
                    "intrinsic": intrinsic,
                    "valid": valid,
                    "frame_id": info_msg.header.frame_id,
                    "stamp": info_msg.header.stamp,
                }
            rospy.loginfo_throttle(5.0, "Synchronized RGB-D frame ready: %s", info_msg.header.frame_id)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "RGB-D conversion failed: %s", exc)

    def wait_for_frame(self):
        deadline = time.time() + float(self.camera["frame_timeout"])
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.frame_lock:
                if self.latest_frame is not None:
                    return dict(self.latest_frame)
            rospy.sleep(0.05)
        raise RuntimeError("no synchronized RGB-D frame received before timeout")

    def handle_inference(self, _request):
        if not self.inference_lock.acquire(False):
            return TriggerResponse(success=False, message="inference is already running")
        try:
            cooldown = float(self.network.get("inference_cooldown", 0.0))
            elapsed = time.time() - self.last_inference_time
            if elapsed < cooldown:
                rospy.sleep(cooldown - elapsed)
            frame = self.wait_for_frame()
            transform = self.tf_buffer.lookup_transform(
                self.config["selector"]["planning_frame"],
                frame["frame_id"],
                rospy.Time(0),
                rospy.Duration(float(self.config["selector"]["tf_timeout"])),
            ).transform
            camera_to_planning = transform_matrix(
                [transform.translation.x, transform.translation.y, transform.translation.z],
                [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ],
            )
            target_pixel = rospy.get_param(
                self.ros_config["namespace"] + "/target_pixel", None
            )
            if target_pixel is not None:
                if not isinstance(target_pixel, list) or len(target_pixel) != 3:
                    raise RuntimeError(
                        "target_pixel must be [u, v, radius] or absent"
                    )
            self.status("inferencing")
            candidates, diagnostics = self.backend.infer(
                frame["color"],
                frame["depth"],
                frame["intrinsic"],
                frame["valid"],
                camera_to_planning=camera_to_planning,
                target_pixel=target_pixel,
            )
            self.last_inference_time = time.time()
            payload = self.publish_result(frame, candidates, diagnostics)
            self.save_debug(frame, payload)
            if not candidates:
                self.valid_pub.publish(Bool(data=False))
                self.status("no_collision_free_candidates")
                return TriggerResponse(success=False, message="no collision-free candidates")
            self.valid_pub.publish(Bool(data=True))
            self.status("ready:{}_candidates".format(len(candidates)))
            return TriggerResponse(
                success=True,
                message="published {} candidates".format(len(candidates)),
            )
        except Exception as exc:
            self.valid_pub.publish(Bool(data=False))
            self.status("error:{}".format(exc))
            rospy.logerr("Inference failed: %s\n%s", exc, traceback.format_exc())
            return TriggerResponse(success=False, message=str(exc))
        finally:
            self.inference_lock.release()

    def publish_result(self, frame, candidates, diagnostics):
        stamp = frame["stamp"]
        payload = {
            "schema_version": 1,
            "frame_id": frame["frame_id"],
            "stamp": {"secs": int(stamp.secs), "nsecs": int(stamp.nsecs)},
            "image": {
                "width": int(frame["color"].shape[1]),
                "height": int(frame["color"].shape[0]),
                "intrinsic": frame["intrinsic"].tolist(),
            },
            "diagnostics": diagnostics,
            "candidates": candidates,
        }
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.candidates_pub.publish(String(data=encoded))

        pose_array = PoseArray()
        pose_array.header.frame_id = frame["frame_id"]
        pose_array.header.stamp = stamp
        for candidate in candidates:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = candidate["translation"]
            quaternion = matrix_to_quaternion(candidate["rotation"])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
            pose_array.poses.append(pose)
        self.pose_pub.publish(pose_array)

        if pose_array.poses:
            best = PoseStamped()
            best.header = pose_array.header
            best.pose = pose_array.poses[0]
            self.best_pub.publish(best)
        self.marker_pub.publish(self.make_markers(frame["frame_id"], stamp, candidates))
        return payload

    @staticmethod
    def make_markers(frame_id, stamp, candidates):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        clear.pose.orientation.w = 1.0
        markers.markers.append(clear)
        for index, candidate in enumerate(candidates):
            translation = np.asarray(candidate["translation"], dtype=float)
            rotation = np.asarray(candidate["rotation"], dtype=float)
            approach = rotation[:, 0]
            opening = rotation[:, 1]

            arrow = Marker()
            arrow.header.frame_id = frame_id
            arrow.header.stamp = stamp
            arrow.ns = "approach"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            start = translation - approach * 0.08
            arrow.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])), Point(x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))]
            arrow.scale.x = 0.004
            arrow.scale.y = 0.010
            arrow.scale.z = 0.014
            arrow.color.r = float(candidate["score"])
            arrow.color.g = 0.2
            arrow.color.b = float(1.0 - min(1.0, candidate["score"]))
            arrow.color.a = 0.9
            markers.markers.append(arrow)

            opening_line = Marker()
            opening_line.header = arrow.header
            opening_line.ns = "opening"
            opening_line.id = index
            opening_line.type = Marker.LINE_LIST
            opening_line.action = Marker.ADD
            opening_line.pose.orientation.w = 1.0
            half = opening * min(float(candidate["width"]) / 2.0, 0.05)
            opening_line.points = [Point(x=float(translation[0] - half[0]), y=float(translation[1] - half[1]), z=float(translation[2] - half[2])), Point(x=float(translation[0] + half[0]), y=float(translation[1] + half[1]), z=float(translation[2] + half[2]))]
            opening_line.scale.x = 0.004
            opening_line.color.r = 0.1
            opening_line.color.g = 1.0
            opening_line.color.b = 0.1
            opening_line.color.a = 0.9
            markers.markers.append(opening_line)
        return markers

    def save_debug(self, frame, payload):
        temp = self.runtime_dir / "latest_result.json.tmp"
        final = self.runtime_dir / "latest_result.json"
        temp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        os.replace(str(temp), str(final))

        image = PilImage.fromarray(frame["color"].astype(np.uint8), mode="RGB")
        draw = ImageDraw.Draw(image)
        intrinsic = frame["intrinsic"]
        for candidate in payload["candidates"][:20]:
            x, y, z = candidate["translation"]
            if z <= 0:
                continue
            u = int(round(intrinsic[0, 0] * x / z + intrinsic[0, 2]))
            v = int(round(intrinsic[1, 1] * y / z + intrinsic[1, 2]))
            radius = 7
            draw.ellipse((u - radius, v - radius, u + radius, v + radius), outline=(255, 32, 32), width=2)
            draw.text((u + 8, v - 8), "{:.2f}".format(candidate["score"]), fill=(255, 255, 0))
        image.save(self.runtime_dir / "latest_candidates.png")


def main():
    args = parse_args()
    config = load_config(args.config)
    rospy.init_node("ur3_graspnet6d_inference")
    InferenceNode(config)
    rospy.spin()


if __name__ == "__main__":
    main()
