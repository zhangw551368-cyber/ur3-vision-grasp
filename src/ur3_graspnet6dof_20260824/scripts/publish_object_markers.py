#!/usr/bin/env python3

"""Publish live classified tabletop objects as RViz MarkerArray messages."""

import json
import math
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


DETECTIONS_TOPIC = "/ur3_graspnet6d/detected_objects_json"
MARKERS_TOPIC = "/ur3_graspnet6d/classified_object_markers"


COLORS = {
    "large_washer": (0.95, 0.82, 0.15),
    "metal_flange": (0.20, 0.75, 1.00),
    "metal_disc": (0.70, 0.70, 0.95),
    "nut_or_washer": (1.00, 0.45, 0.15),
    "yellow_pliers": (1.00, 0.85, 0.05),
    "blue_cutters": (0.10, 0.45, 1.00),
    "large_ring": (0.25, 1.00, 0.45),
    "silver_bolt": (0.85, 0.85, 0.85),
    "black_connector": (0.45, 0.25, 0.75),
}


class ObjectMarkerPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth = None
        self.depth_header = None
        self.info = None
        self.publisher = rospy.Publisher(
            MARKERS_TOPIC, MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            self.on_depth,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )
        rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.on_info, queue_size=1
        )
        rospy.Subscriber(DETECTIONS_TOPIC, String, self.on_detections, queue_size=1)

    def on_depth(self, message):
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        depth = np.asarray(depth, dtype=np.float32)
        if message.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        with self.lock:
            self.depth = depth
            self.depth_header = message.header

    def on_info(self, message):
        with self.lock:
            self.info = message

    @staticmethod
    def object_depth(depth, bbox):
        height, width = depth.shape[:2]
        x1, y1, x2, y2 = [int(round(value)) for value in bbox]
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        values = depth[y1:y2, x1:x2]
        values = values[np.isfinite(values) & (values > 0.20) & (values < 1.20)]
        if values.size < 5:
            return None
        # Expanded 2-D boxes include table pixels. The closest stable depth
        # band represents the object surface for the overhead camera.
        near = float(np.percentile(values, 12.0))
        band = values[(values >= near - 0.006) & (values <= near + 0.025)]
        if band.size < 5:
            return near
        return float(np.median(band))

    @staticmethod
    def color(marker, rgb, alpha=0.88):
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = alpha

    def on_detections(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "invalid object JSON: %s", exc)
            return
        with self.lock:
            if self.depth is None or self.info is None or self.depth_header is None:
                return
            depth = self.depth.copy()
            header = self.depth_header
            info = self.info
        fx, fy, cx, cy = float(info.K[0]), float(info.K[4]), float(info.K[2]), float(info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return

        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = header.frame_id
        clear.header.stamp = rospy.Time(0)
        clear.pose.orientation.w = 1.0
        result.markers.append(clear)
        order = 0
        for item in payload.get("objects", []):
            if not item.get("pickable", False):
                continue
            z = self.object_depth(depth, item["bbox"])
            if z is None:
                continue
            order += 1
            u, v = [float(value) for value in item["center"]]
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            x1, y1, x2, y2 = [float(value) for value in item["bbox"]]
            sx = min(0.16, max(0.018, abs(x2 - x1) * z / fx))
            sy = min(0.16, max(0.018, abs(y2 - y1) * z / fy))
            rgb = COLORS.get(item["category"], (1.0, 0.3, 0.3))

            body = Marker()
            body.header.frame_id = header.frame_id
            body.header.stamp = rospy.Time(0)
            body.ns = "classified_objects"
            body.id = order * 2
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, z
            body.pose.orientation.w = 1.0
            body.scale.x, body.scale.y, body.scale.z = sx, sy, 0.025
            self.color(body, rgb, 0.58)
            result.markers.append(body)

            label = Marker()
            label.header = body.header
            label.ns = "classified_labels"
            label.id = order * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = z - 0.055
            label.pose.orientation.w = 1.0
            label.scale.z = 0.026
            self.color(label, rgb, 1.0)
            label.text = "{}: {}".format(order, item["category"])
            result.markers.append(label)
        self.publisher.publish(result)
        rospy.loginfo_throttle(5.0, "Published %d classified 3-D object markers", order)


def main():
    rospy.init_node("ur3_graspnet6d_object_markers")
    ObjectMarkerPublisher()
    rospy.loginfo("Object markers: %s -> %s", DETECTIONS_TOPIC, MARKERS_TOPIC)
    rospy.spin()


if __name__ == "__main__":
    main()
