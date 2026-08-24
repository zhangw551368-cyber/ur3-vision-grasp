#!/usr/bin/env python3

"""Lock one validated HSV target before robot motion and republish it unchanged."""

import math
import statistics
import threading

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped


class TargetLocker:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/hsv_grasp/object_point_base"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/hsv_grasp/locked_object_point"
        )
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.sample_count = int(rospy.get_param("~sample_count", 7))
        self.max_xy_span = float(rospy.get_param("~max_xy_span", 0.015))
        self.x_bounds = [float(v) for v in rospy.get_param("~x_bounds")]
        self.y_bounds = [float(v) for v in rospy.get_param("~y_bounds")]
        self.z_bounds = [float(v) for v in rospy.get_param("~z_bounds")]
        self.locked_z = float(rospy.get_param("~locked_z", float("nan")))
        self.project_to_locked_z_plane = bool(
            rospy.get_param("~project_to_locked_z_plane", False)
        )
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_color_optical_frame"
        )
        self.samples = []
        self.locked = None
        self.mutex = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, PointStamped, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointStamped, self.callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_locked)
        rospy.loginfo(
            "HSV target locker waiting on %s; accepted box x=%s y=%s z=%s",
            self.input_topic,
            self.x_bounds,
            self.y_bounds,
            self.z_bounds,
        )

    @staticmethod
    def finite(point):
        return all(math.isfinite(v) for v in (point.x, point.y, point.z))

    def to_target_frame(self, message):
        if message.header.frame_id == self.target_frame:
            return message
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            message.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.5),
        )
        return tf2_geometry_msgs.do_transform_point(message, transform)

    def project_camera_ray_to_locked_plane(self, transformed):
        """Recover XY from the image ray when depth falls through to the table."""
        if not self.project_to_locked_z_plane or not math.isfinite(self.locked_z):
            return transformed
        camera_origin = PointStamped()
        camera_origin.header.stamp = rospy.Time(0)
        camera_origin.header.frame_id = self.camera_frame
        origin = self.to_target_frame(camera_origin)
        dz = transformed.point.z - origin.point.z
        if abs(dz) <= 1e-6:
            raise RuntimeError("camera ray is parallel to the locked-Z plane")
        scale = (self.locked_z - origin.point.z) / dz
        if not (0.0 < scale <= 1.05):
            raise RuntimeError(
                "locked-Z plane intersection is outside the observed camera ray"
            )
        projected = PointStamped()
        projected.header.stamp = transformed.header.stamp
        projected.header.frame_id = self.target_frame
        projected.point.x = origin.point.x + scale * (
            transformed.point.x - origin.point.x
        )
        projected.point.y = origin.point.y + scale * (
            transformed.point.y - origin.point.y
        )
        projected.point.z = self.locked_z
        return projected

    def callback(self, message):
        with self.mutex:
            if self.locked is not None:
                return
        try:
            transformed = self.to_target_frame(message)
            transformed = self.project_camera_ray_to_locked_plane(transformed)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2.0, "Target lock TF failed: %s", exc)
            return
        except RuntimeError as exc:
            rospy.logwarn_throttle(2.0, "Target lock ray projection failed: %s", exc)
            return

        p = transformed.point
        if not self.finite(p):
            return
        if not (self.x_bounds[0] <= p.x <= self.x_bounds[1]):
            return
        if not (self.y_bounds[0] <= p.y <= self.y_bounds[1]):
            return
        # Reject the support-plane readings that intermittently pass the HSV mask.
        if not (self.z_bounds[0] <= p.z <= self.z_bounds[1]):
            return

        with self.mutex:
            self.samples.append((p.x, p.y, p.z))
            self.samples = self.samples[-self.sample_count:]
            if len(self.samples) < self.sample_count:
                rospy.loginfo_throttle(
                    1.0, "Target lock collecting %d/%d", len(self.samples), self.sample_count
                )
                return
            x_values = [v[0] for v in self.samples]
            y_values = [v[1] for v in self.samples]
            if max(x_values) - min(x_values) > self.max_xy_span:
                self.samples.pop(0)
                return
            if max(y_values) - min(y_values) > self.max_xy_span:
                self.samples.pop(0)
                return
            z_value = (
                self.locked_z
                if math.isfinite(self.locked_z)
                else statistics.median(v[2] for v in self.samples)
            )
            self.locked = (
                statistics.median(x_values),
                statistics.median(y_values),
                z_value,
            )
            self.subscriber.unregister()
            rospy.logwarn(
                "TARGET_LOCKED frame=%s xyz=[%.6f, %.6f, %.6f]; "
                "future camera changes are ignored until this node restarts",
                self.target_frame,
                self.locked[0],
                self.locked[1],
                self.locked[2],
            )

    def publish_locked(self, _event):
        with self.mutex:
            locked = self.locked
        if locked is None:
            return
        message = PointStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.target_frame
        message.point.x, message.point.y, message.point.z = locked
        self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("lock_hsv_grasp_target")
    TargetLocker()
    rospy.spin()
