#!/usr/bin/python3

import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image


class YoloRedBlockLocator:
    def __init__(self):
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError("Cannot import ultralytics YOLO: {}".format(exc))

        self.model_path = rospy.get_param(
            "~model_path",
            "/home/gzu/gzu_ws/src/ultralytics_ros/models/red_block.pt",
        )
        if not os.path.exists(self.model_path):
            raise RuntimeError(
                "YOLO model file does not exist: {}. Put a public/fine-tuned "
                "colored-block model there or pass _model_path:=/path/to/model.pt".format(
                    self.model_path
                )
            )

        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.target_frame = rospy.get_param("~target_frame", "right_arm_base")
        self.output_topic = rospy.get_param(
            "~output_topic", "/yolo_red_block/point_base"
        )
        self.point_camera_topic = rospy.get_param(
            "~point_camera_topic", "/yolo_red_block/point_camera"
        )
        self.pixel_topic = rospy.get_param("~pixel_topic", "/yolo_red_block/pixel")
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/yolo_red_block/image"
        )
        self.debug_image_path = rospy.get_param(
            "~debug_image_path", "/tmp/yolo_red_block_debug.jpg"
        )

        self.class_names = [
            str(name).strip().lower()
            for name in rospy.get_param(
                "~class_names", ["red", "red cube", "red_block", "red block"]
            )
            if str(name).strip()
        ]
        self.class_ids = [int(v) for v in rospy.get_param("~class_ids", [])]
        self.conf = float(rospy.get_param("~conf_thres", 0.35))
        self.iou = float(rospy.get_param("~iou_thres", 0.45))
        self.min_red_score = float(rospy.get_param("~min_red_score", 0.08))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.max_rate = float(rospy.get_param("~max_rate", 5.0))
        self.depth_window = int(rospy.get_param("~depth_window_pixels", 9))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.05))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 1.5))
        self.min_depth_samples = int(rospy.get_param("~min_depth_samples", 20))
        self.max_depth_age = float(rospy.get_param("~max_depth_age_sec", 1.0))
        self.use_plane_if_depth_missing = bool(
            rospy.get_param("~use_plane_if_depth_missing", False)
        )
        self.table_z = float(rospy.get_param("~table_z", 0.59))
        self.block_height = float(rospy.get_param("~block_height", 0.03))
        self.z_offset = float(rospy.get_param("~z_offset", 0.0))

        rospy.loginfo("Loading YOLO model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_depth = None
        self.latest_info = None
        self.last_process_time = 0.0

        self.point_pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=1)
        self.camera_point_pub = rospy.Publisher(
            self.point_camera_topic, PointStamped, queue_size=1
        )
        self.pixel_pub = rospy.Publisher(self.pixel_topic, Point, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)

        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback, queue_size=1
        )
        self.info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.info_callback, queue_size=1
        )
        self.color_sub = rospy.Subscriber(
            self.color_topic, Image, self.color_callback, queue_size=1
        )

    def depth_callback(self, msg):
        self.latest_depth = msg

    def info_callback(self, msg):
        self.latest_info = msg

    def color_callback(self, msg):
        now = time.time()
        if self.max_rate > 0.0 and now - self.last_process_time < 1.0 / self.max_rate:
            return
        self.last_process_time = now
        if self.latest_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for camera_info")
            return

        try:
            color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detection = self.detect_red_block(color_bgr)
            if detection is None:
                rospy.logwarn_throttle(1.0, "YOLO did not detect the red block")
                return
            u, v = self.pixel_from_detection(detection)
            point_camera, depth_m, source = self.point_from_detection(
                detection, u, v, color_bgr.shape, msg.header
            )
            point_base = self.transform_point(point_camera)
            self.publish(point_camera, point_base, u, v, depth_m, source, color_bgr, detection)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "YOLO red block locate failed: %s", exc)

    def depth_to_meters(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)

    def detect_red_block(self, color_bgr):
        results = self.model.predict(
            source=color_bgr,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        if not results:
            return None
        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return None

        names = getattr(result, "names", None) or getattr(self.model, "names", {})
        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)

        best = None
        for xyxy, conf, cls in zip(boxes, confs, classes):
            class_name = str(names.get(int(cls), cls)).lower()
            if not self.class_matches(int(cls), class_name):
                continue
            red_score = self.red_score(color_bgr, xyxy)
            if red_score < self.min_red_score:
                continue
            score = float(conf) + 0.15 * red_score
            if best is None or score > best["score"]:
                best = {
                    "xyxy": xyxy,
                    "conf": float(conf),
                    "class_id": int(cls),
                    "class_name": class_name,
                    "score": score,
                }
        return best

    def class_matches(self, class_id, class_name):
        if self.class_ids and class_id in self.class_ids:
            return True
        if self.class_names and class_name in self.class_names:
            return True
        if self.class_names and any(name in class_name for name in self.class_names):
            return True
        return not self.class_ids and not self.class_names

    @staticmethod
    def red_score(color_bgr, xyxy):
        x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
        h, w = color_bgr.shape[:2]
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return 0.0
        roi = color_bgr[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 80, 40]), np.array([12, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([168, 80, 40]), np.array([180, 255, 255]))
        return float(np.count_nonzero(mask1 | mask2)) / float(mask1.size)

    def pixel_from_detection(self, detection):
        x1, y1, x2, y2 = detection["xyxy"]
        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))
        return float(u), float(v)

    def point_from_detection(self, detection, u, v, color_shape, image_header):
        depth_m = None
        if self.depth_is_fresh(image_header):
            depth = self.depth_to_meters(self.latest_depth)
            depth_m = self.median_depth(depth, int(round(u)), int(round(v)), self.depth_window)
            if depth_m is None:
                depth_m = self.median_depth_in_box(depth, detection["xyxy"])
            if depth_m is not None:
                point = self.project_pixel(
                    u, v, depth_m, color_shape, image_header, self.latest_info
                )
                return point, float(depth_m), "depth"

        if not self.use_plane_if_depth_missing:
            if self.latest_depth is None:
                raise RuntimeError("Waiting for aligned depth")
            raise RuntimeError("No valid depth inside YOLO detection")

        point = self.camera_point_from_table_plane(u, v, color_shape, image_header)
        return point, 0.0, "table"

    def depth_is_fresh(self, image_header):
        if self.latest_depth is None:
            return False
        if self.max_depth_age <= 0.0:
            return True
        if self.latest_depth.header.stamp == rospy.Time(0) or image_header.stamp == rospy.Time(0):
            return True
        age = abs((image_header.stamp - self.latest_depth.header.stamp).to_sec())
        return age <= self.max_depth_age

    def median_depth(self, depth, u, v, window):
        radius = max(1, window // 2)
        h, w = depth.shape[:2]
        x1, x2 = max(0, u - radius), min(w, u + radius + 1)
        y1, y2 = max(0, v - radius), min(h, v + radius + 1)
        return self.valid_depth_median(depth[y1:y2, x1:x2])

    def median_depth_in_box(self, depth, xyxy):
        x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
        h, w = depth.shape[:2]
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        return self.valid_depth_median(depth[y1:y2, x1:x2])

    def valid_depth_median(self, roi):
        values = roi[np.isfinite(roi)]
        values = values[(values >= self.min_depth_m) & (values <= self.max_depth_m)]
        if values.size < self.min_depth_samples:
            return None
        return float(np.median(values))

    def scaled_camera_pixel(self, u, v, color_shape, camera_info):
        info_w = camera_info.width or color_shape[1]
        info_h = camera_info.height or color_shape[0]
        ui = float(u) * float(info_w) / float(max(1, color_shape[1]))
        vi = float(v) * float(info_h) / float(max(1, color_shape[0]))
        return ui, vi

    def project_pixel(self, u, v, depth_m, color_shape, image_header, camera_info):
        ui, vi = self.scaled_camera_pixel(u, v, color_shape, camera_info)
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("Invalid camera intrinsics")
        point = PointStamped()
        point.header.stamp = image_header.stamp
        point.header.frame_id = image_header.frame_id or camera_info.header.frame_id
        point.point.x = (ui - cx) * depth_m / fx
        point.point.y = (vi - cy) * depth_m / fy
        point.point.z = depth_m
        return point

    def camera_point_from_table_plane(self, u, v, color_shape, image_header):
        ui, vi = self.scaled_camera_pixel(u, v, color_shape, self.latest_info)
        fx = self.latest_info.K[0]
        fy = self.latest_info.K[4]
        cx = self.latest_info.K[2]
        cy = self.latest_info.K[5]
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("Invalid camera intrinsics")

        ray_camera = np.array([(ui - cx) / fx, (vi - cy) / fy, 1.0], dtype=np.float64)
        camera_frame = image_header.frame_id or self.latest_info.header.frame_id
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            camera_frame,
            rospy.Time(0),
            rospy.Duration(0.5),
        ).transform
        rotation = self.rotation_matrix_from_quaternion(transform.rotation)
        origin_base = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            dtype=np.float64,
        )
        ray_base = rotation @ ray_camera
        target_z = self.table_z + self.block_height * 0.5 + self.z_offset
        if abs(ray_base[2]) < 1e-6:
            raise RuntimeError("Camera ray is parallel to table plane")
        distance = (target_z - origin_base[2]) / ray_base[2]
        if distance <= 0.0:
            raise RuntimeError("Table plane is behind camera ray")

        point = PointStamped()
        point.header.stamp = image_header.stamp
        point.header.frame_id = camera_frame
        point.point.x = float(ray_camera[0] * distance)
        point.point.y = float(ray_camera[1] * distance)
        point.point.z = float(ray_camera[2] * distance)
        return point

    def transform_point(self, point_camera):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            point_camera.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.5),
        )
        rotation = self.rotation_matrix_from_quaternion(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        source = np.array(
            [point_camera.point.x, point_camera.point.y, point_camera.point.z],
            dtype=np.float64,
        )
        target = rotation @ source + translation

        point = PointStamped()
        point.header.stamp = point_camera.header.stamp
        point.header.frame_id = self.target_frame
        point.point.x = float(target[0])
        point.point.y = float(target[1])
        point.point.z = float(target[2])
        return point

    @staticmethod
    def rotation_matrix_from_quaternion(q):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise RuntimeError("Invalid zero-length quaternion")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        return np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )

    def publish(self, point_camera, point_base, u, v, depth_m, source, color_bgr, detection):
        self.camera_point_pub.publish(point_camera)
        self.point_pub.publish(point_base)
        self.pixel_pub.publish(Point(float(u), float(v), depth_m))

        x1, y1, x2, y2 = [int(round(value)) for value in detection["xyxy"]]
        debug = color_bgr.copy()
        cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.drawMarker(debug, (int(u), int(v)), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        label = "{} {:.2f} {}={:.3f}".format(
            detection["class_name"], detection["conf"], source, depth_m
        )
        cv2.putText(debug, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        if self.debug_image_path:
            cv2.imwrite(self.debug_image_path, debug)

        rospy.loginfo_throttle(
            0.5,
            "YOLO red block %s conf=%.2f pixel=(%.0f, %.0f) %s=%.3f base=[%.3f, %.3f, %.3f]",
            detection["class_name"],
            detection["conf"],
            u,
            v,
            source,
            depth_m,
            point_base.point.x,
            point_base.point.y,
            point_base.point.z,
        )


if __name__ == "__main__":
    rospy.init_node("yolo_red_block_locator")
    try:
        YoloRedBlockLocator()
        rospy.spin()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
