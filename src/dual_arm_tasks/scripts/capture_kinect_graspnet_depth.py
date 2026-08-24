#!/usr/bin/env python3

import argparse
import os
import sys
import threading
import time

import cv2
import message_filters
import numpy as np
import rospy
import scipy.io as scio
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture Kinect RGB-D frames in a GraspNet-like uint16 depth format."
    )
    parser.add_argument("--color-topic", default="/kinect_1/kinect2/qhd/image_color_rect")
    parser.add_argument("--depth-topic", default="/kinect_1/kinect2/qhd/image_depth_rect")
    parser.add_argument("--camera-info-topic", default="/kinect_1/kinect2/qhd/camera_info")
    parser.add_argument("--output", default="/home/gzu/gzu_ws/datasets/kinect_graspnet_capture")
    parser.add_argument("--scene-id", type=int, default=0)
    parser.add_argument("--camera-name", default="kinect", choices=["kinect", "realsense"])
    parser.add_argument("--mode", default="manual", choices=["manual", "auto", "once"])
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-index", type=int, default=-1)
    parser.add_argument("--sync-slop", type=float, default=0.08)
    parser.add_argument("--queue-size", type=int, default=10)
    parser.add_argument("--target-width", type=int, default=0)
    parser.add_argument("--target-height", type=int, default=0)
    parser.add_argument(
        "--geometry-space",
        default="color",
        choices=["color", "depth"],
        help="Optical frame represented by saved depth pixels and intrinsics.",
    )
    parser.add_argument("--workspace-x-min", type=int, default=0)
    parser.add_argument("--workspace-y-min", type=int, default=0)
    parser.add_argument("--workspace-x-max", type=int, default=0)
    parser.add_argument("--workspace-y-max", type=int, default=0)
    parser.add_argument("--min-depth-mm", type=int, default=1)
    parser.add_argument("--max-depth-mm", type=int, default=5000)
    parser.add_argument("--show-preview", nargs="?", const=True, default=False, type=str_to_bool)
    parser.add_argument("--no-demo-copy", nargs="?", const=True, default=False, type=str_to_bool)
    parser.add_argument("--write-empty-labels", nargs="?", const=True, default=False, type=str_to_bool)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    if value in ("1", "true", "yes", "y", "on"):
        return True
    if value in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("expected true or false, got {}".format(value))


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def scene_dir(root, scene_id, camera_name):
    return os.path.join(root, "scenes", "scene_{:04d}".format(scene_id), camera_name)


def next_index(rgb_dir):
    ensure_dir(rgb_dir)
    indices = []
    for name in os.listdir(rgb_dir):
        stem, ext = os.path.splitext(name)
        if ext.lower() == ".png" and stem.isdigit():
            indices.append(int(stem))
    return max(indices) + 1 if indices else 0


def camera_info_to_k(info):
    return np.array(info.K, dtype=np.float32).reshape(3, 3)


def scale_intrinsics(k_matrix, sx, sy):
    scaled = k_matrix.copy()
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def scale_intrinsics_between_sizes(k_matrix, source_size, target_size):
    source_width, source_height = source_size
    target_width, target_height = target_size
    if source_width <= 0 or source_height <= 0:
        return k_matrix
    sx = float(target_width) / float(source_width)
    sy = float(target_height) / float(source_height)
    return scale_intrinsics(k_matrix, sx, sy)


def resize_pair(color, depth_mm, k_matrix, target_width, target_height):
    height, width = depth_mm.shape[:2]
    if target_width <= 0 or target_height <= 0:
        return color, depth_mm, k_matrix
    if width == target_width and height == target_height:
        return color, depth_mm, k_matrix

    sx = float(target_width) / float(width)
    sy = float(target_height) / float(height)
    color_resized = cv2.resize(
        color, (target_width, target_height), interpolation=cv2.INTER_AREA
    )
    depth_resized = cv2.resize(
        depth_mm, (target_width, target_height), interpolation=cv2.INTER_NEAREST
    )
    return color_resized, depth_resized, scale_intrinsics(k_matrix, sx, sy)


def depth_to_uint16_mm(depth, encoding):
    depth = np.asarray(depth)
    if depth.dtype == np.uint16:
        return depth.copy()

    depth_float = depth.astype(np.float32)
    valid = np.isfinite(depth_float) & (depth_float > 0)
    depth_mm = np.zeros(depth_float.shape, dtype=np.float32)
    if np.any(valid):
        finite_max = float(np.nanmax(depth_float[valid]))
        if encoding in ("32FC1", "64FC1") or finite_max < 20.0:
            depth_mm[valid] = depth_float[valid] * 1000.0
        else:
            depth_mm[valid] = depth_float[valid]
    return np.clip(np.rint(depth_mm), 0, 65535).astype(np.uint16)


def workspace_mask(depth_mm, args):
    mask = (
        (depth_mm >= int(args.min_depth_mm))
        & (depth_mm <= int(args.max_depth_mm))
    ).astype(np.uint8)

    height, width = depth_mm.shape[:2]
    x_min = max(0, min(width, int(args.workspace_x_min)))
    y_min = max(0, min(height, int(args.workspace_y_min)))
    x_max = int(args.workspace_x_max) if args.workspace_x_max else width
    y_max = int(args.workspace_y_max) if args.workspace_y_max else height
    x_max = max(x_min, min(width, x_max))
    y_max = max(y_min, min(height, y_max))

    roi = np.zeros_like(mask)
    roi[y_min:y_max, x_min:x_max] = 1
    return (mask & roi).astype(np.uint8)


def write_yaml_summary(path, data):
    with open(path, "w", encoding="utf-8") as stream:
        for key, value in data.items():
            stream.write("{}: {}\n".format(key, value))


def checked_imwrite(path, image):
    if not cv2.imwrite(path, image):
        raise IOError("failed to write image {}".format(path))


def object_string(value):
    return np.array([value or ""], dtype=object)


def scalar_float(value):
    return np.array([[float(value)]], dtype=np.float64)


def scalar_int(value):
    return np.array([[int(value)]], dtype=np.int32)


class KinectGraspNetCapture:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.latest = None
        self.latest_lock = threading.Lock()
        self.saved = 0
        self.last_auto_save = 0.0

        self.base_dir = scene_dir(args.output, args.scene_id, args.camera_name)
        self.rgb_dir = os.path.join(self.base_dir, "rgb")
        self.depth_dir = os.path.join(self.base_dir, "depth")
        self.meta_dir = os.path.join(self.base_dir, "meta")
        self.workspace_dir = os.path.join(self.base_dir, "workspace_mask")
        self.label_dir = os.path.join(self.base_dir, "label")
        self.rect_dir = os.path.join(self.base_dir, "rect")
        self.annotations_dir = os.path.join(self.base_dir, "annotations")
        for directory in (
            self.rgb_dir,
            self.depth_dir,
            self.meta_dir,
            self.workspace_dir,
        ):
            ensure_dir(directory)
        if args.write_empty_labels:
            for directory in (self.label_dir, self.rect_dir, self.annotations_dir):
                ensure_dir(directory)

        self.index = args.start_index if args.start_index >= 0 else next_index(self.rgb_dir)

        color_sub = message_filters.Subscriber(args.color_topic, Image)
        depth_sub = message_filters.Subscriber(args.depth_topic, Image)
        info_sub = message_filters.Subscriber(args.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=args.queue_size,
            slop=args.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_callback)

        rospy.loginfo("Saving GraspNet-like data under %s", self.base_dir)
        rospy.loginfo(
            "Listening color=%s depth=%s camera_info=%s geometry_space=%s",
            args.color_topic,
            args.depth_topic,
            args.camera_info_topic,
            args.geometry_space,
        )
        if args.mode == "manual":
            rospy.loginfo("Manual mode: focus preview window and press s to save, q to quit")

    def warn_geometry_if_needed(self, color_msg, depth_msg, info_msg, color_shape, depth_shape):
        color_frame = color_msg.header.frame_id or ""
        depth_frame = depth_msg.header.frame_id or ""
        info_frame = info_msg.header.frame_id or ""
        info_size = (int(info_msg.width), int(info_msg.height))
        color_size = (int(color_shape[1]), int(color_shape[0]))
        depth_size = (int(depth_shape[1]), int(depth_shape[0]))

        if color_size != depth_size:
            rospy.logwarn_throttle(
                5.0,
                "RGB and depth image sizes differ: color=%s depth=%s. "
                "RGB will be resized for saved visualization only; make sure depth, "
                "CameraInfo, and GraspNet point-cloud generation use one geometry space.",
                color_size,
                depth_size,
            )
        if info_size != depth_size:
            rospy.logwarn_throttle(
                5.0,
                "CameraInfo size %s does not match depth size %s. Intrinsics will be "
                "scaled to the saved depth image size; this is valid only for resized "
                "images in the same optical frame, not for raw RGB/depth frame mixing.",
                info_size,
                depth_size,
            )
        if self.args.geometry_space == "color" and depth_frame and info_frame and depth_frame != info_frame:
            rospy.logwarn_throttle(
                5.0,
                "geometry_space=color but depth frame_id (%s) differs from CameraInfo "
                "frame_id (%s). Use registered depth-to-color topics or set "
                "--geometry-space depth with depth-space CameraInfo.",
                depth_frame,
                info_frame,
            )
        if self.args.geometry_space == "depth" and depth_frame and info_frame and depth_frame != info_frame:
            rospy.logwarn_throttle(
                5.0,
                "geometry_space=depth but depth frame_id (%s) differs from CameraInfo "
                "frame_id (%s). Use depth/IR CameraInfo for raw depth-space data.",
                depth_frame,
                info_frame,
            )
        if color_frame and depth_frame and color_frame != depth_frame:
            rospy.logwarn_throttle(
                5.0,
                "Color frame_id (%s) differs from depth frame_id (%s). This is fine "
                "only if the depth image has already been registered into the saved "
                "geometry frame.",
                color_frame,
                depth_frame,
            )

    def synced_callback(self, color_msg, depth_msg, info_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(3.0, "Image conversion failed: %s", exc)
            return

        depth_mm = depth_to_uint16_mm(depth, depth_msg.encoding)
        raw_color_shape = color.shape[:2]
        raw_depth_shape = depth_mm.shape[:2]
        raw_k_matrix = camera_info_to_k(info_msg)
        k_matrix = raw_k_matrix.copy()
        info_size = (
            int(info_msg.width) if info_msg.width else int(depth_mm.shape[1]),
            int(info_msg.height) if info_msg.height else int(depth_mm.shape[0]),
        )
        depth_size = (int(depth_mm.shape[1]), int(depth_mm.shape[0]))
        k_scaled_from_info = info_size != depth_size
        if k_scaled_from_info:
            k_matrix = scale_intrinsics_between_sizes(k_matrix, info_size, depth_size)

        self.warn_geometry_if_needed(
            color_msg,
            depth_msg,
            info_msg,
            raw_color_shape,
            raw_depth_shape,
        )

        if color.shape[:2] != depth_mm.shape[:2]:
            color = cv2.resize(
                color,
                (depth_mm.shape[1], depth_mm.shape[0]),
                interpolation=cv2.INTER_AREA,
            )

        color, depth_mm, k_matrix = resize_pair(
            color,
            depth_mm,
            k_matrix,
            self.args.target_width,
            self.args.target_height,
        )
        mask = workspace_mask(depth_mm, self.args)
        stamp = max(color_msg.header.stamp, depth_msg.header.stamp)

        with self.latest_lock:
            self.latest = {
                "color": color,
                "depth_mm": depth_mm,
                "workspace_mask": mask,
                "k_matrix": k_matrix,
                "raw_k_matrix": raw_k_matrix,
                "stamp": stamp.to_sec(),
                "frame_id": (
                    info_msg.header.frame_id
                    if self.args.geometry_space == "color"
                    else depth_msg.header.frame_id
                ) or depth_msg.header.frame_id or color_msg.header.frame_id,
                "geometry_space": self.args.geometry_space,
                "color_frame_id": color_msg.header.frame_id,
                "depth_frame_id": depth_msg.header.frame_id,
                "camera_info_frame_id": info_msg.header.frame_id,
                "color_encoding": color_msg.encoding,
                "depth_encoding": depth_msg.encoding,
                "raw_color_width": raw_color_shape[1],
                "raw_color_height": raw_color_shape[0],
                "raw_depth_width": raw_depth_shape[1],
                "raw_depth_height": raw_depth_shape[0],
                "camera_info_width": info_size[0],
                "camera_info_height": info_size[1],
                "k_scaled_from_camera_info": k_scaled_from_info,
            }

    def meta_dict(self, sample, include_image_size=False):
        data = {
            "intrinsic_matrix": sample["k_matrix"].astype(np.float32),
            "raw_intrinsic_matrix": sample["raw_k_matrix"].astype(np.float32),
            "factor_depth": np.array([[1000.0]], dtype=np.float32),
            "depth_unit": object_string("millimeter"),
            "frame_id": object_string(sample["frame_id"]),
            "geometry_space": object_string(sample["geometry_space"]),
            "color_frame_id": object_string(sample["color_frame_id"]),
            "depth_frame_id": object_string(sample["depth_frame_id"]),
            "camera_info_frame_id": object_string(sample["camera_info_frame_id"]),
            "color_encoding": object_string(sample["color_encoding"]),
            "depth_encoding": object_string(sample["depth_encoding"]),
            "timestamp": scalar_float(sample["stamp"]),
            "raw_color_width": scalar_int(sample["raw_color_width"]),
            "raw_color_height": scalar_int(sample["raw_color_height"]),
            "raw_depth_width": scalar_int(sample["raw_depth_width"]),
            "raw_depth_height": scalar_int(sample["raw_depth_height"]),
            "camera_info_width": scalar_int(sample["camera_info_width"]),
            "camera_info_height": scalar_int(sample["camera_info_height"]),
            "saved_width": scalar_int(sample["depth_mm"].shape[1]),
            "saved_height": scalar_int(sample["depth_mm"].shape[0]),
            "k_scaled_from_camera_info": scalar_int(sample["k_scaled_from_camera_info"]),
        }
        if include_image_size:
            data["image_width"] = scalar_int(sample["depth_mm"].shape[1])
            data["image_height"] = scalar_int(sample["depth_mm"].shape[0])
        return data

    def save_latest(self):
        with self.latest_lock:
            sample = None if self.latest is None else dict(self.latest)
        if sample is None:
            rospy.logwarn("No synchronized RGB-D frame received yet")
            return False

        ann_id = self.index
        stem = "{:04d}".format(ann_id)
        rgb_path = os.path.join(self.rgb_dir, stem + ".png")
        depth_path = os.path.join(self.depth_dir, stem + ".png")
        meta_path = os.path.join(self.meta_dir, stem + ".mat")
        workspace_path = os.path.join(self.workspace_dir, stem + ".png")

        written_paths = []
        try:
            checked_imwrite(rgb_path, sample["color"])
            written_paths.append(rgb_path)
            checked_imwrite(depth_path, sample["depth_mm"])
            written_paths.append(depth_path)
            checked_imwrite(workspace_path, sample["workspace_mask"])
            written_paths.append(workspace_path)
            scio.savemat(meta_path, self.meta_dict(sample))
            written_paths.append(meta_path)
            np.save(os.path.join(self.base_dir, "camK.npy"), sample["k_matrix"])

            if self.args.write_empty_labels:
                label = np.zeros(sample["depth_mm"].shape, dtype=np.uint16)
                label_path = os.path.join(self.label_dir, stem + ".png")
                rect_path = os.path.join(self.rect_dir, stem + ".npy")
                annotation_path = os.path.join(self.annotations_dir, stem + ".xml")
                checked_imwrite(label_path, label)
                written_paths.append(label_path)
                np.save(rect_path, np.zeros((0, 0), dtype=np.float32))
                written_paths.append(rect_path)
                with open(annotation_path, "w", encoding="utf-8") as stream:
                    stream.write("<annotation></annotation>\n")
                written_paths.append(annotation_path)
        except Exception:
            for path in written_paths:
                try:
                    os.remove(path)
                except OSError:
                    pass
            raise

        self.update_scene_side_files(sample)
        if not self.args.no_demo_copy:
            self.write_demo_copy(sample)

        self.index += 1
        self.saved += 1
        rospy.loginfo(
            "Saved %s depth dtype=%s range=[%d,%d] valid=%d",
            depth_path,
            sample["depth_mm"].dtype,
            int(sample["depth_mm"].min()),
            int(sample["depth_mm"].max()),
            int(np.count_nonzero(sample["depth_mm"])),
        )
        return True

    def update_scene_side_files(self, sample):
        scene_root = os.path.dirname(self.base_dir)
        object_id_list = os.path.join(scene_root, "object_id_list.txt")
        if not os.path.exists(object_id_list):
            open(object_id_list, "a", encoding="utf-8").close()

        pose_count = max(self.index + 1, 1)
        poses = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], pose_count, axis=0)
        np.save(os.path.join(self.base_dir, "camera_poses.npy"), poses)
        np.save(os.path.join(self.base_dir, "cam0_wrt_table.npy"), np.eye(4, dtype=np.float32))
        if self.args.camera_name == "kinect":
            np.save(os.path.join(scene_root, "rs_wrt_kn.npy"), poses)

        write_yaml_summary(
            os.path.join(self.base_dir, "capture_info.yaml"),
            {
                "color_topic": self.args.color_topic,
                "depth_topic": self.args.depth_topic,
                "camera_info_topic": self.args.camera_info_topic,
                "geometry_space": sample["geometry_space"],
                "frame_id": sample["frame_id"],
                "color_frame_id": sample["color_frame_id"],
                "depth_frame_id": sample["depth_frame_id"],
                "camera_info_frame_id": sample["camera_info_frame_id"],
                "color_encoding": sample["color_encoding"],
                "depth_encoding": sample["depth_encoding"],
                "depth_format": "uint16_png_millimeters",
                "factor_depth": 1000,
                "raw_color_width": sample["raw_color_width"],
                "raw_color_height": sample["raw_color_height"],
                "raw_depth_width": sample["raw_depth_width"],
                "raw_depth_height": sample["raw_depth_height"],
                "camera_info_width": sample["camera_info_width"],
                "camera_info_height": sample["camera_info_height"],
                "width": sample["depth_mm"].shape[1],
                "height": sample["depth_mm"].shape[0],
                "k_scaled_from_camera_info": int(sample["k_scaled_from_camera_info"]),
            },
        )

    def write_demo_copy(self, sample):
        demo_dir = os.path.join(self.args.output, "latest_demo")
        ensure_dir(demo_dir)
        checked_imwrite(os.path.join(demo_dir, "color.png"), sample["color"])
        checked_imwrite(os.path.join(demo_dir, "depth.png"), sample["depth_mm"])
        checked_imwrite(os.path.join(demo_dir, "workspace_mask.png"), sample["workspace_mask"])
        scio.savemat(os.path.join(demo_dir, "meta.mat"), self.meta_dict(sample, True))

    def preview_image(self):
        with self.latest_lock:
            sample = None if self.latest is None else dict(self.latest)
        if sample is None:
            return None
        color = sample["color"]
        depth_mm = sample["depth_mm"]
        mask = sample["workspace_mask"]
        depth_vis = np.zeros_like(depth_mm, dtype=np.uint8)
        valid = depth_mm > 0
        if np.any(valid):
            near = np.percentile(depth_mm[valid], 2)
            far = np.percentile(depth_mm[valid], 98)
            far = max(far, near + 1)
            depth_vis = np.clip((depth_mm.astype(np.float32) - near) * 255.0 / (far - near), 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
        depth_color[mask == 0] = (0, 0, 0)
        preview = np.hstack([color, depth_color])
        max_width = 1500
        if preview.shape[1] > max_width:
            scale = float(max_width) / float(preview.shape[1])
            preview = cv2.resize(preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.putText(
            preview,
            "s: save  q: quit  saved={} next={:04d}".format(self.saved, self.index),
            (20, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        return preview

    def spin(self):
        if self.args.mode == "once":
            deadline = time.time() + 10.0
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.save_latest():
                    return
                rospy.sleep(0.2)
            raise RuntimeError("Timed out waiting for synchronized RGB-D frame")

        if self.args.mode == "auto":
            rate = rospy.Rate(max(self.args.rate, 0.1))
            while not rospy.is_shutdown() and self.saved < self.args.count:
                now = time.time()
                if now - self.last_auto_save >= 1.0 / max(self.args.rate, 0.1):
                    if self.save_latest():
                        self.last_auto_save = now
                rate.sleep()
            return

        show_preview = self.args.show_preview and bool(os.environ.get("DISPLAY"))
        while not rospy.is_shutdown() and self.saved < self.args.count:
            if show_preview:
                preview = self.preview_image()
                if preview is not None:
                    cv2.imshow("kinect_graspnet_capture", preview)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("s"), ord("S")):
                    self.save_latest()
                elif key in (ord("q"), ord("Q"), 27):
                    break
            else:
                input("Press Enter to save one RGB-D frame, or Ctrl-C to stop...")
                self.save_latest()
        if show_preview:
            cv2.destroyWindow("kinect_graspnet_capture")


def main():
    args = parse_args()
    rospy.init_node("capture_kinect_graspnet_depth", anonymous=True)
    capture = KinectGraspNetCapture(args)
    capture.spin()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
