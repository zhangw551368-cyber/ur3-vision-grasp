#!/usr/bin/python3

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image


class RedBlockLocatorKinect:
    def __init__(
        self,
        label="Kinect",
        default_color_topic="/kinect_0/kinect2/hd/image_color_rect",
        default_depth_topic="/kinect_0/kinect2/hd/image_depth_rect",
        default_info_topic="/kinect_0/kinect2/hd/camera_info",
        default_output_topic="/red_block/point_base_aux",
        default_use_plane_if_depth_missing=True,
    ):
        self.label = label
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_encoding = None
        self.camera_info = None
        self.last_color_shape = None

        self.target_frame = rospy.get_param("~target_frame", "right_arm_base")
        self.color_topic = rospy.get_param("~color_topic", default_color_topic)
        self.depth_topic = rospy.get_param("~depth_topic", default_depth_topic)
        self.info_topic = rospy.get_param("~camera_info_topic", default_info_topic)
        self.output_topic = rospy.get_param("~output_topic", default_output_topic)

        self.low_1 = np.array(rospy.get_param("~red_low_1", [0, 100, 50]), dtype=np.uint8)
        self.high_1 = np.array(rospy.get_param("~red_high_1", [12, 255, 255]), dtype=np.uint8)
        self.low_2 = np.array(rospy.get_param("~red_low_2", [168, 100, 50]), dtype=np.uint8)
        self.high_2 = np.array(rospy.get_param("~red_high_2", [180, 255, 255]), dtype=np.uint8)
        self.min_area = float(rospy.get_param("~min_area_pixels", 1000))
        self.max_area = float(rospy.get_param("~max_area_pixels", 0))
        self.min_fill = float(rospy.get_param("~min_fill_ratio", 0.35))
        self.min_aspect = float(rospy.get_param("~min_aspect_ratio", 0.2))
        self.max_aspect = float(rospy.get_param("~max_aspect_ratio", 4.0))
        self.depth_window = int(rospy.get_param("~depth_window_pixels", 9))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.05))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 2.0))
        self.use_roi = bool(rospy.get_param("~use_roi", False))
        self.roi_x_min = int(rospy.get_param("~roi_x_min", 0))
        self.roi_y_min = int(rospy.get_param("~roi_y_min", 0))
        self.roi_x_max = int(rospy.get_param("~roi_x_max", 0))
        self.roi_y_max = int(rospy.get_param("~roi_y_max", 0))
        self.use_plane_if_depth_missing = bool(
            rospy.get_param("~use_plane_if_depth_missing", default_use_plane_if_depth_missing)
        )
        self.table_z = float(rospy.get_param("~table_z", 0.0))
        self.block_height = float(rospy.get_param("~block_height", 0.03))
        self.z_offset = float(rospy.get_param("~z_offset", 0.0))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.point_base_pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=1)
        self.point_camera_pub = rospy.Publisher("/red_block/point_camera", PointStamped, queue_size=1)
        self.pixel_pub = rospy.Publisher("/red_block/pixel", Point, queue_size=1)
        self.image_pub = rospy.Publisher("/red_block/image", Image, queue_size=1)
        self.mask_pub = rospy.Publisher("/red_block/mask", Image, queue_size=1)

        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback, queue_size=1)
        rospy.Subscriber(self.color_topic, Image, self.color_callback, queue_size=1)
        rospy.loginfo(
            "%s red block locator: color=%s depth=%s info=%s target_frame=%s output=%s",
            self.label,
            self.color_topic,
            self.depth_topic,
            self.info_topic,
            self.target_frame,
            self.output_topic,
        )

    def depth_callback(self, msg):
        self.depth_image = np.asarray(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"),
            dtype=np.float64,
        )
        self.depth_encoding = msg.encoding
        if msg.encoding in ("16UC1", "mono16"):
            self.depth_image *= 0.001

    def info_callback(self, msg):
        self.camera_info = msg

    def roi_bounds(self, image_shape):
        height, width = image_shape[:2]
        x_max = self.roi_x_max if self.roi_x_max else width
        y_max = self.roi_y_max if self.roi_y_max else height
        return (
            max(0, min(width, self.roi_x_min)),
            max(0, min(height, self.roi_y_min)),
            max(0, min(width, x_max)),
            max(0, min(height, y_max)),
        )

    def apply_roi(self, mask, image_shape):
        if not self.use_roi:
            return mask
        x_min, y_min, x_max, y_max = self.roi_bounds(image_shape)
        limited = np.zeros_like(mask)
        limited[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
        return limited

    def select_contour(self, contours):
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            if self.max_area and area > self.max_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            aspect = float(width) / float(max(1, height))
            if aspect < self.min_aspect or aspect > self.max_aspect:
                continue
            fill = float(area) / float(max(1, width * height))
            if fill < self.min_fill:
                continue
            aspect_score = 1.0 / (1.0 + abs(np.log(max(aspect, 1e-6))))
            candidates.append((area * (0.7 + fill) * aspect_score, contour, (x, y, width, height, area, fill)))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])

    def scaled_pixel(self, u, v, source_shape, target_shape):
        source_h, source_w = source_shape[:2]
        target_h, target_w = target_shape[:2]
        x = int(round(float(u) * float(target_w) / float(max(1, source_w))))
        y = int(round(float(v) * float(target_h) / float(max(1, source_h))))
        return max(0, min(target_w - 1, x)), max(0, min(target_h - 1, y))

    def depth_at(self, u, v, color_shape):
        if self.depth_image is None:
            return None
        du, dv = self.scaled_pixel(u, v, color_shape, self.depth_image.shape)
        radius = max(1, self.depth_window // 2)
        y0 = max(0, dv - radius)
        y1 = min(self.depth_image.shape[0], dv + radius + 1)
        x0 = max(0, du - radius)
        x1 = min(self.depth_image.shape[1], du + radius + 1)
        values = self.depth_image[y0:y1, x0:x1]
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return None
        depth = float(np.median(values))
        if depth > 20.0:
            depth *= 0.001
        if depth < self.min_depth_m or depth > self.max_depth_m:
            return None
        return depth

    def camera_pixel(self, u, v, color_shape):
        if self.camera_info is None:
            return None, None
        info_w = self.camera_info.width or color_shape[1]
        info_h = self.camera_info.height or color_shape[0]
        ui = float(u) * float(info_w) / float(max(1, color_shape[1]))
        vi = float(v) * float(info_h) / float(max(1, color_shape[0]))
        return ui, vi

    def camera_point_from_depth(self, u, v, color_shape, depth):
        ui, vi = self.camera_pixel(u, v, color_shape)
        if ui is None or depth is None:
            return None
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        if fx == 0.0 or fy == 0.0:
            return None
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.camera_info.header.frame_id
        point.point.x = (ui - cx) * depth / fx
        point.point.y = (vi - cy) * depth / fy
        point.point.z = depth
        return point

    def camera_point_from_table_plane(self, u, v, color_shape):
        ui, vi = self.camera_pixel(u, v, color_shape)
        if ui is None:
            return None
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        ray = np.array([(ui - cx) / fx, (vi - cy) / fy, 1.0])
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.camera_info.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.5),
        ).transform
        qx, qy, qz, qw = (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        rotation = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )
        origin = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        ray_base = rotation @ ray
        target_z = self.table_z + self.block_height * 0.5 + self.z_offset
        if abs(ray_base[2]) < 1e-6:
            return None
        distance = (target_z - origin[2]) / ray_base[2]
        if distance <= 0.0:
            return None
        camera_xyz = ray * distance
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.camera_info.header.frame_id
        point.point.x = float(camera_xyz[0])
        point.point.y = float(camera_xyz[1])
        point.point.z = float(camera_xyz[2])
        return point

    def publish_point(self, point_camera):
        if point_camera is None:
            return None
        self.point_camera_pub.publish(point_camera)
        point_base = self.tf_buffer.transform(
            point_camera, self.target_frame, timeout=rospy.Duration(0.5)
        )
        self.point_base_pub.publish(point_base)
        return point_base

    def color_callback(self, msg):
        if self.camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for Kinect camera_info")
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_color_shape = image.shape
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_1, self.high_1) | cv2.inRange(hsv, self.low_2, self.high_2)
        mask = self.apply_roi(mask, image.shape)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        selected = self.select_contour(contours)

        label = "red block not found"
        if self.use_roi:
            cv2.rectangle(image, self.roi_bounds(image.shape)[:2], self.roi_bounds(image.shape)[2:], (255, 180, 0), 1)
        if selected is not None:
            _, contour, stats = selected
            x, y, width, height, area, fill = stats
            moments = cv2.moments(contour)
            if moments["m00"] > 0:
                u = int(moments["m10"] / moments["m00"])
                v = int(moments["m01"] / moments["m00"])
            else:
                u, v = x + width // 2, y + height // 2
            depth = self.depth_at(u, v, image.shape)
            point_camera = self.camera_point_from_depth(u, v, image.shape, depth)
            source = "depth"
            if point_camera is None and self.use_plane_if_depth_missing:
                try:
                    point_camera = self.camera_point_from_table_plane(u, v, image.shape)
                    source = "table"
                except tf2_ros.TransformException as exc:
                    rospy.logwarn_throttle(2.0, "Cannot use table plane fallback: %s", exc)
            try:
                point_base = self.publish_point(point_camera)
            except tf2_ros.TransformException as exc:
                point_base = None
                rospy.logwarn_throttle(2.0, "Cannot transform red block to %s: %s", self.target_frame, exc)

            self.pixel_pub.publish(Point(float(u), float(v), float(depth or 0.0)))
            cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.circle(image, (u, v), 5, (255, 255, 255), -1)
            if point_base is not None:
                label = "red {} base=[{:.3f},{:.3f},{:.3f}] area={:.0f}".format(
                    source,
                    point_base.point.x,
                    point_base.point.y,
                    point_base.point.z,
                    area,
                )
            else:
                label = "red waiting TF/depth u={} v={} area={:.0f}".format(u, v, area)
        cv2.putText(image, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))


if __name__ == "__main__":
    rospy.init_node("red_block_locator_kinect")
    RedBlockLocatorKinect()
    rospy.spin()
