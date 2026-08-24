#!/usr/bin/python3

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


class RedObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_encoding = None
        self.camera_info = None

        self.low_1 = np.array(rospy.get_param("~red_low_1", [0, 90, 70]), dtype=np.uint8)
        self.high_1 = np.array(rospy.get_param("~red_high_1", [10, 255, 255]), dtype=np.uint8)
        self.low_2 = np.array(rospy.get_param("~red_low_2", [170, 90, 70]), dtype=np.uint8)
        self.high_2 = np.array(rospy.get_param("~red_high_2", [180, 255, 255]), dtype=np.uint8)
        self.min_area = rospy.get_param("~min_area_pixels", 500)
        self.max_area = rospy.get_param("~max_area_pixels", 0)
        self.min_width = rospy.get_param("~min_bbox_width_pixels", 0)
        self.max_width = rospy.get_param("~max_bbox_width_pixels", 0)
        self.min_height = rospy.get_param("~min_bbox_height_pixels", 0)
        self.max_height = rospy.get_param("~max_bbox_height_pixels", 0)
        self.min_aspect = rospy.get_param("~min_aspect_ratio", 0.0)
        self.max_aspect = rospy.get_param("~max_aspect_ratio", 0.0)
        self.min_fill = rospy.get_param("~min_fill_ratio", 0.0)
        self.depth_window = rospy.get_param("~depth_window_pixels", 7)
        self.max_cloud_points = rospy.get_param("~max_cloud_points", 3000)
        self.use_roi = rospy.get_param("~use_roi", False)
        self.roi_x_min = rospy.get_param("~roi_x_min", 0)
        self.roi_y_min = rospy.get_param("~roi_y_min", 0)
        self.roi_x_max = rospy.get_param("~roi_x_max", 0)
        self.roi_y_max = rospy.get_param("~roi_y_max", 0)

        color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")

        self.annotated_pub = rospy.Publisher("/red_object/image", Image, queue_size=1)
        self.mask_pub = rospy.Publisher("/red_object/mask", Image, queue_size=1)
        self.cloud_pub = rospy.Publisher("/red_object/cloud", point_cloud2.PointCloud2, queue_size=1)
        self.pixel_pub = rospy.Publisher("/red_object/pixel", Point, queue_size=1)
        self.point_pub = rospy.Publisher("/red_object/point_camera", PointStamped, queue_size=1)

        rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(info_topic, CameraInfo, self.info_callback, queue_size=1)
        rospy.Subscriber(color_topic, Image, self.color_callback, queue_size=1)
        rospy.loginfo("Listening for color=%s depth=%s camera_info=%s", color_topic, depth_topic, info_topic)

    def roi_bounds(self, image_shape):
        height, width = image_shape[:2]
        x_min = max(0, min(width, int(self.roi_x_min)))
        y_min = max(0, min(height, int(self.roi_y_min)))
        x_max = int(self.roi_x_max) if self.roi_x_max else width
        y_max = int(self.roi_y_max) if self.roi_y_max else height
        x_max = max(x_min, min(width, x_max))
        y_max = max(y_min, min(height, y_max))
        return x_min, y_min, x_max, y_max

    def apply_roi(self, mask, image_shape):
        if not self.use_roi:
            return mask
        x_min, y_min, x_max, y_max = self.roi_bounds(image_shape)
        limited = np.zeros_like(mask)
        limited[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
        return limited

    def candidate_score(self, area, width, height, fill_ratio):
        aspect = float(width) / float(height)
        aspect_score = 1.0 / (1.0 + abs(np.log(max(aspect, 1e-6))))
        return area * (0.7 + fill_ratio) * aspect_score

    def select_candidate(self, contours):
        candidates = []
        rejected = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                rejected += 1
                continue
            if self.max_area and area > self.max_area:
                rejected += 1
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if self.min_width and width < self.min_width:
                rejected += 1
                continue
            if self.max_width and width > self.max_width:
                rejected += 1
                continue
            if self.min_height and height < self.min_height:
                rejected += 1
                continue
            if self.max_height and height > self.max_height:
                rejected += 1
                continue
            aspect = float(width) / float(height)
            if self.min_aspect and aspect < self.min_aspect:
                rejected += 1
                continue
            if self.max_aspect and aspect > self.max_aspect:
                rejected += 1
                continue
            fill_ratio = float(area) / float(max(1, width * height))
            if self.min_fill and fill_ratio < self.min_fill:
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
            return None, rejected
        return max(candidates, key=lambda candidate: candidate[0]), rejected

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_encoding = msg.encoding
        except CvBridgeError as exc:
            rospy.logwarn_throttle(3.0, "Depth conversion failed: %s", exc)

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_at(self, u, v):
        if self.depth_image is None:
            return None
        radius = max(1, self.depth_window // 2)
        y0, y1 = max(0, v - radius), min(self.depth_image.shape[0], v + radius + 1)
        x0, x1 = max(0, u - radius), min(self.depth_image.shape[1], u + radius + 1)
        values = np.asarray(self.depth_image[y0:y1, x0:x1], dtype=np.float32)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return None
        depth = float(np.median(values))
        if self.depth_encoding in ("16UC1", "mono16"):
            depth *= 0.001
        return depth

    def camera_point(self, u, v, depth):
        if self.camera_info is None or depth is None:
            return None
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        if fx == 0 or fy == 0:
            return None
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.camera_info.header.frame_id
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth
        return point

    def publish_selected_cloud(self, contour, stamp):
        if self.depth_image is None or self.camera_info is None:
            return
        component = np.zeros(self.depth_image.shape[:2], dtype=np.uint8)
        cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
        component = cv2.erode(component, np.ones((3, 3), np.uint8), iterations=1)

        depth = np.asarray(self.depth_image, dtype=np.float32)
        if self.depth_encoding in ("16UC1", "mono16"):
            depth = depth * 0.001
        rows, cols = np.nonzero((component > 0) & np.isfinite(depth) & (depth > 0))
        if rows.size == 0:
            return
        if rows.size > self.max_cloud_points:
            stride = int(np.ceil(float(rows.size) / float(self.max_cloud_points)))
            rows = rows[::stride]
            cols = cols[::stride]

        z = depth[rows, cols]
        fx, fy = self.camera_info.K[0], self.camera_info.K[4]
        cx, cy = self.camera_info.K[2], self.camera_info.K[5]
        if fx == 0 or fy == 0:
            return
        points = np.stack(
            ((cols - cx) * z / fx, (rows - cy) * z / fy, z),
            axis=1,
        ).astype(np.float32)
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_info.header.frame_id
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(3.0, "Color conversion failed: %s", exc)
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_1, self.high_1) | cv2.inRange(hsv, self.low_2, self.high_2)
        mask = self.apply_roi(mask, image.shape)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        selected, rejected = self.select_candidate(contours)

        if self.use_roi:
            x_min, y_min, x_max, y_max = self.roi_bounds(image.shape)
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 180, 0), 1)

        if selected:
            _, contour, stats = selected
            x, y, width, height, area, fill_ratio = stats
            u, v = x + width // 2, y + height // 2
            depth = self.depth_at(u, v)
            point = self.camera_point(u, v, depth)

            pixel = Point(x=float(u), y=float(v), z=float(depth or 0.0))
            self.pixel_pub.publish(pixel)
            if point is not None:
                self.point_pub.publish(point)
            self.publish_selected_cloud(contour, msg.header.stamp)

            cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 2)
            label = "red u={} v={} depth={} area={:.0f} fill={:.2f}".format(
                u,
                v,
                "{:.3f}m".format(depth) if depth else "waiting",
                area,
                fill_ratio,
            )
            cv2.putText(image, label, (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            label = "red object not found; contours={} rejected={}".format(
                len(contours), rejected
            )
            cv2.putText(image, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))


if __name__ == "__main__":
    rospy.init_node("red_object_detector")
    RedObjectDetector()
    rospy.spin()
