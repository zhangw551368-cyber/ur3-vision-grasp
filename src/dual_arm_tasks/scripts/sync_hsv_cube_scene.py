#!/usr/bin/env python3

"""Validate a live HSV/depth cube position and keep RViz state fresh.

The detector reports one point on the cube.  This node transforms it into the
MoveIt planning frame, filters a short live window, validates it against the
measured support surface, and publishes a *separate* stable point for planning.
It deliberately does not command or plan robot motion.

Important behaviour:
  * the raw observation is always visible for diagnosis;
  * only a stable, in-workspace observation is shown as a blue cube;
  * all markers are deleted and the valid flag becomes false after a timeout;
  * the planning topic is not latched, so an old cube cannot silently become a
    target after restarting a planner.
"""

import math
import statistics
from collections import deque

import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped conversions.
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


def get_bool_param(name, default):
    value = rospy.get_param("~{}".format(name), default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        value = value.strip().lower()
        if value in ("true", "1", "yes", "on"):
            return True
        if value in ("false", "0", "no", "off"):
            return False
    return bool(value)


def finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError("{} must be finite".format(name))
    return result


def median(values):
    return float(statistics.median(values))


class LiveHsvCubeScene:
    RAW_CUBE_ID = 0
    VALID_CUBE_ID = 1
    TOP_POINT_ID = 2
    TEXT_ID = 3

    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/hsv_grasp/object_point_base"
        )
        self.target_frame = rospy.get_param("~target_frame", "base").strip()
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/hsv_grasp/live_cube_markers"
        )
        self.stable_topic = rospy.get_param(
            "~stable_topic", "/hsv_grasp/stable_object_point"
        )
        self.valid_topic = rospy.get_param(
            "~valid_topic", "/hsv_grasp/live_cube_valid"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/hsv_grasp/live_cube_status"
        )
        self.legacy_marker_topic = rospy.get_param(
            "~legacy_marker_topic", "/hsv_grasp/object_marker"
        )

        self.cube_size = finite(rospy.get_param("~cube_size", 0.055), "cube_size")
        self.semantic = rospy.get_param("~object_point_semantic", "top_center")
        self.filter_window = int(rospy.get_param("~filter_window", 9))
        self.min_samples = int(rospy.get_param("~min_samples", 7))
        self.stale_timeout = finite(
            rospy.get_param("~stale_timeout", 0.8), "stale_timeout"
        )
        self.tf_timeout = rospy.Duration(
            finite(rospy.get_param("~tf_timeout", 0.5), "tf_timeout")
        )
        self.max_input_age = finite(
            rospy.get_param("~max_input_age", 0.5), "max_input_age"
        )
        self.max_mad_xy = finite(
            rospy.get_param("~max_mad_xy", 0.006), "max_mad_xy"
        )
        self.max_mad_z = finite(
            rospy.get_param("~max_mad_z", 0.012), "max_mad_z"
        )

        self.workspace_validation = get_bool_param("workspace_validation", True)
        self.table_center_x = finite(
            rospy.get_param("~table_center_x", 0.291), "table_center_x"
        )
        self.table_center_y = finite(
            rospy.get_param("~table_center_y", -0.225), "table_center_y"
        )
        self.table_size_x = finite(
            rospy.get_param("~table_size_x", 0.282), "table_size_x"
        )
        self.table_size_y = finite(
            rospy.get_param("~table_size_y", 0.450), "table_size_y"
        )
        self.table_yaw = finite(rospy.get_param("~table_yaw", 0.0), "table_yaw")
        self.table_top_z = finite(
            rospy.get_param("~table_top_z", 0.018), "table_top_z"
        )
        self.edge_margin = finite(
            rospy.get_param("~edge_margin", 0.015), "edge_margin"
        )
        self.max_support_z_error = finite(
            rospy.get_param("~max_support_z_error", 0.040),
            "max_support_z_error",
        )
        self.snap_to_support_plane = get_bool_param("snap_to_support_plane", True)
        self.show_raw_observation = get_bool_param("show_raw_observation", True)

        self._validate_params()

        self.samples = deque(maxlen=self.filter_window)
        self.last_input_wall = None
        self.last_input_stamp = rospy.Time(0)
        self.last_valid = False
        self.last_status = "STARTING: waiting for live HSV/depth observations"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.stable_pub = rospy.Publisher(
            self.stable_topic, PointStamped, queue_size=1, latch=False
        )
        self.valid_pub = rospy.Publisher(
            self.valid_topic, Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        self.legacy_marker_pub = rospy.Publisher(
            self.legacy_marker_topic, Marker, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointStamped, self.point_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.on_shutdown(self.clear_visuals)
        rospy.sleep(0.15)
        self.clear_legacy_marker()
        self.publish_validity(False, self.last_status)
        rospy.loginfo(
            "Live HSV cube synchronizer: %s -> %s, markers=%s, stable=%s, "
            "window=%d/%d stale=%.2fs workspace_check=%s table=(center %.3f %.3f, "
            "size %.3f %.3f, top_z %.3f)",
            self.input_topic,
            self.target_frame,
            self.marker_topic,
            self.stable_topic,
            self.min_samples,
            self.filter_window,
            self.stale_timeout,
            self.workspace_validation,
            self.table_center_x,
            self.table_center_y,
            self.table_size_x,
            self.table_size_y,
            self.table_top_z,
        )

    def _validate_params(self):
        if not self.target_frame:
            raise RuntimeError("target_frame must not be empty")
        if self.semantic not in ("top_center", "cube_center"):
            raise RuntimeError("object_point_semantic must be top_center or cube_center")
        if self.cube_size <= 0.0:
            raise RuntimeError("cube_size must be > 0")
        if not 3 <= self.filter_window <= 101:
            raise RuntimeError("filter_window must be in [3, 101]")
        if not 3 <= self.min_samples <= self.filter_window:
            raise RuntimeError("min_samples must be in [3, filter_window]")
        if self.stale_timeout <= 0.0 or self.max_input_age <= 0.0:
            raise RuntimeError("stale_timeout and max_input_age must be > 0")
        if self.max_mad_xy <= 0.0 or self.max_mad_z <= 0.0:
            raise RuntimeError("MAD limits must be > 0")
        if self.table_size_x <= 0.0 or self.table_size_y <= 0.0:
            raise RuntimeError("table sizes must be > 0")
        if self.edge_margin < 0.0 or self.max_support_z_error <= 0.0:
            raise RuntimeError("edge_margin must be >= 0 and z tolerance > 0")

    @staticmethod
    def point_is_finite(point):
        return all(math.isfinite(v) for v in (point.x, point.y, point.z))

    def transform(self, msg):
        if not msg.header.frame_id.strip():
            raise RuntimeError("input PointStamped has an empty frame_id")
        if msg.header.frame_id == self.target_frame:
            result = PointStamped()
            result.header = msg.header
            result.header.frame_id = self.target_frame
            result.point = msg.point
            return result
        return self.tf_buffer.transform(msg, self.target_frame, timeout=self.tf_timeout)

    def top_z_from_sample(self, xyz):
        return xyz[2] if self.semantic == "top_center" else xyz[2] + self.cube_size / 2.0

    def raw_center_from_sample(self, xyz):
        center_z = xyz[2]
        if self.semantic == "top_center":
            center_z -= self.cube_size / 2.0
        return xyz[0], xyz[1], center_z

    def filtered_sample(self):
        return (
            median([p[0] for p in self.samples]),
            median([p[1] for p in self.samples]),
            median([p[2] for p in self.samples]),
        )

    def sample_mad(self, filtered):
        return (
            median([abs(p[0] - filtered[0]) for p in self.samples]),
            median([abs(p[1] - filtered[1]) for p in self.samples]),
            median([abs(p[2] - filtered[2]) for p in self.samples]),
        )

    def inside_table(self, x, y):
        dx = x - self.table_center_x
        dy = y - self.table_center_y
        c = math.cos(self.table_yaw)
        s = math.sin(self.table_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        half_cube = self.cube_size / 2.0
        allowed_x = self.table_size_x / 2.0 - half_cube - self.edge_margin
        allowed_y = self.table_size_y / 2.0 - half_cube - self.edge_margin
        return (
            allowed_x >= 0.0
            and allowed_y >= 0.0
            and abs(local_x) <= allowed_x
            and abs(local_y) <= allowed_y
        ), local_x, local_y, allowed_x, allowed_y

    def validate_filtered(self, filtered):
        if len(self.samples) < self.min_samples:
            return False, "COLLECTING: {}/{} live samples".format(
                len(self.samples), self.min_samples
            ), None

        mad = self.sample_mad(filtered)
        if mad[0] > self.max_mad_xy or mad[1] > self.max_mad_xy or mad[2] > self.max_mad_z:
            return False, (
                "UNSTABLE: MAD xyz=({:.1f},{:.1f},{:.1f}) mm"
                .format(1000.0 * mad[0], 1000.0 * mad[1], 1000.0 * mad[2])
            ), mad

        if not self.workspace_validation:
            return True, "VALID PREVIEW: workspace validation disabled", mad

        inside, lx, ly, ax, ay = self.inside_table(filtered[0], filtered[1])
        if not inside:
            return False, (
                "REJECTED: outside table; local xy=({:.3f},{:.3f}) allowed=+/-({:.3f},{:.3f})"
                .format(lx, ly, ax, ay)
            ), mad

        inferred_support_z = self.top_z_from_sample(filtered) - self.cube_size
        z_error = inferred_support_z - self.table_top_z
        if abs(z_error) > self.max_support_z_error:
            return False, (
                "REJECTED: support height mismatch; detected={:.3f} configured={:.3f} error={:+.3f} m"
                .format(inferred_support_z, self.table_top_z, z_error)
            ), mad

        return True, (
            "VALID: stable on table; MAD=({:.1f},{:.1f},{:.1f}) mm"
            .format(1000.0 * mad[0], 1000.0 * mad[1], 1000.0 * mad[2])
        ), mad

    def stable_top_point(self, filtered):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.target_frame
        point.point.x = filtered[0]
        point.point.y = filtered[1]
        if self.snap_to_support_plane:
            point.point.z = self.table_top_z + self.cube_size
        else:
            point.point.z = self.top_z_from_sample(filtered)
        return point

    def make_cube_marker(self, marker_id, center, color, alpha):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.ns = "hsv_live_cube"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha
        marker.lifetime = rospy.Duration(self.stale_timeout + 0.2)
        return marker

    def make_point_marker(self, point, valid):
        marker = Marker()
        marker.header = point.header
        marker.ns = "hsv_live_cube"
        marker.id = self.TOP_POINT_ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.012
        marker.color.r = 0.1 if valid else 1.0
        marker.color.g = 1.0 if valid else 0.2
        marker.color.b = 0.2 if valid else 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(self.stale_timeout + 0.2)
        return marker

    def make_text_marker(self, x, y, z, text, valid):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.ns = "hsv_live_cube"
        marker.id = self.TEXT_ID
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.09
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.025
        marker.color.r = 0.2 if valid else 1.0
        marker.color.g = 1.0 if valid else 0.35
        marker.color.b = 1.0 if valid else 0.05
        marker.color.a = 1.0
        marker.text = text
        marker.lifetime = rospy.Duration(self.stale_timeout + 0.2)
        return marker

    def delete_marker(self, marker_id):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.ns = "hsv_live_cube"
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def clear_legacy_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.action = Marker.DELETEALL
        try:
            self.legacy_marker_pub.publish(marker)
        except rospy.ROSException:
            pass

    def clear_visuals(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.action = Marker.DELETEALL
        try:
            self.marker_pub.publish(MarkerArray(markers=[marker]))
        except rospy.ROSException:
            pass
        self.clear_legacy_marker()

    def publish_validity(self, valid, status):
        self.last_valid = valid
        self.last_status = status
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=status))

    def publish_observation(self, filtered, valid, status):
        if rospy.is_shutdown():
            return
        markers = []
        raw_center = self.raw_center_from_sample(filtered)
        raw_top = PointStamped()
        raw_top.header.stamp = rospy.Time.now()
        raw_top.header.frame_id = self.target_frame
        raw_top.point.x = filtered[0]
        raw_top.point.y = filtered[1]
        raw_top.point.z = self.top_z_from_sample(filtered)

        if self.show_raw_observation:
            markers.append(
                self.make_cube_marker(
                    self.RAW_CUBE_ID, raw_center, (1.0, 0.35, 0.0), 0.28
                )
            )
        else:
            markers.append(self.delete_marker(self.RAW_CUBE_ID))

        if valid:
            stable_top = self.stable_top_point(filtered)
            stable_center = (
                stable_top.point.x,
                stable_top.point.y,
                stable_top.point.z - self.cube_size / 2.0,
            )
            markers.append(
                self.make_cube_marker(
                    self.VALID_CUBE_ID, stable_center, (0.05, 0.35, 1.0), 0.90
                )
            )
            markers.append(self.make_point_marker(stable_top, True))
            self.stable_pub.publish(stable_top)
            text_x, text_y, text_z = stable_center
        else:
            markers.append(self.delete_marker(self.VALID_CUBE_ID))
            markers.append(self.make_point_marker(raw_top, False))
            text_x, text_y, text_z = raw_center

        markers.append(self.make_text_marker(text_x, text_y, text_z, status, valid))
        if rospy.is_shutdown():
            return
        try:
            self.marker_pub.publish(MarkerArray(markers=markers))
            self.publish_validity(valid, status)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def reject_input(self, reason):
        self.samples.clear()
        self.publish_validity(False, reason)
        self.clear_visuals()
        rospy.logwarn_throttle(1.0, "%s", reason)

    def point_callback(self, msg):
        if rospy.is_shutdown():
            return
        if not self.point_is_finite(msg.point):
            self.reject_input("REJECTED: non-finite input point")
            return
        if not msg.header.stamp.is_zero():
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if age > self.max_input_age:
                self.reject_input(
                    "REJECTED: input timestamp is {:.3f}s old".format(age)
                )
                return
        try:
            point = self.transform(msg)
        except (tf2_ros.TransformException, RuntimeError) as exc:
            self.reject_input("REJECTED: TF failed: {}".format(exc))
            return

        now_wall = rospy.get_time()
        if (
            self.last_input_wall is not None
            and now_wall - self.last_input_wall > self.stale_timeout
        ):
            self.samples.clear()
        self.last_input_wall = now_wall
        self.last_input_stamp = rospy.Time.now()
        self.samples.append((point.point.x, point.point.y, point.point.z))

        filtered = self.filtered_sample()
        valid, status, _ = self.validate_filtered(filtered)
        self.publish_observation(filtered, valid, status)
        rospy.loginfo_throttle(
            1.0,
            "live HSV cube: valid=%s filtered_%s=(%.4f, %.4f, %.4f) %s",
            valid,
            self.target_frame,
            filtered[0],
            filtered[1],
            filtered[2],
            status,
        )

    def timer_callback(self, _event):
        if rospy.is_shutdown():
            return
        if self.last_input_wall is None:
            return
        age = rospy.get_time() - self.last_input_wall
        if age <= self.stale_timeout:
            return
        if self.samples or self.last_valid:
            self.samples.clear()
            status = "STALE: no live detection for {:.2f}s; target deleted".format(age)
            self.publish_validity(False, status)
            self.clear_visuals()
            rospy.logwarn("%s", status)


def main():
    rospy.init_node("sync_hsv_cube_scene", anonymous=False)
    LiveHsvCubeScene()
    rospy.spin()


if __name__ == "__main__":
    main()
