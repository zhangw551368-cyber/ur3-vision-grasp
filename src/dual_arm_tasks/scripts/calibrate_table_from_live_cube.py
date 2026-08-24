#!/usr/bin/env python3

"""Measure an axis-aligned table in ``base`` with a detected calibration cube.

The operator places the known cube flush with the middle of each of the four
table edges and once near the table centre.  No robot command is published.
The script converts every live camera observation to the requested frame,
rejects unstable sets, infers the table bounds, and writes a review-only YAML
report.  It never edits the active MoveIt scene automatically.
"""

import argparse
import math
import os
import statistics
import sys
import threading
import time

import rospy
import tf2_geometry_msgs  # noqa: F401 - register PointStamped conversions.
import tf2_ros
import yaml
from geometry_msgs.msg import PointStamped


def med(values):
    return float(statistics.median(values))


def mad(values, center):
    return med([abs(value - center) for value in values])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Use five live cube placements to infer table bounds in base."
    )
    parser.add_argument("--topic", default="/hsv_grasp/object_point_base")
    parser.add_argument("--target-frame", default="base")
    parser.add_argument("--cube-size", type=float, default=0.055)
    parser.add_argument("--table-thickness", type=float, default=0.018)
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument(
        "--timeout",
        type=float,
        default=90.0,
        help="maximum total capture time per placement in seconds",
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=12.0,
        help="fail only after this many seconds without a new valid point",
    )
    parser.add_argument("--max-mad-xy", type=float, default=0.006)
    parser.add_argument("--max-mad-z", type=float, default=0.012)
    parser.add_argument("--max-height-spread", type=float, default=0.020)
    parser.add_argument("--expected-long", type=float, default=0.450)
    parser.add_argument("--expected-short", type=float, default=0.282)
    parser.add_argument("--size-tolerance", type=float, default=0.035)
    parser.add_argument(
        "--output",
        default="/home/gzu/gzu_ws/calibration/environment/right_arm_table_cube_probe.yaml",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class TableCubeProbe:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.collecting = False
        self.samples = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sub = rospy.Subscriber(
            args.topic, PointStamped, self.callback, queue_size=10
        )

    def callback(self, msg):
        if not self.collecting:
            return
        if not all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            return
        try:
            if msg.header.frame_id == self.args.target_frame:
                point = msg
            else:
                point = self.tf_buffer.transform(
                    msg, self.args.target_frame, timeout=rospy.Duration(0.3)
                )
        except tf2_ros.TransformException:
            return
        with self.lock:
            if self.collecting:
                self.samples.append((point.point.x, point.point.y, point.point.z))

    def capture(self, label, instruction):
        answer = input(
            "\n[{}] {}\n放稳后按 Enter 开始采集；输入 q 后 Enter 退出： ".format(
                label, instruction
            )
        ).strip().lower()
        if answer == "q":
            raise KeyboardInterrupt

        with self.lock:
            self.samples = []
            self.collecting = True
        started_at = time.monotonic()
        deadline = time.monotonic() + self.args.timeout
        last_sample_at = started_at
        last_count = -1
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                count = len(self.samples)
            if count >= self.args.samples:
                break
            if count != last_count:
                now = time.monotonic()
                if count > last_count and count > 0:
                    last_sample_at = now
                elapsed = max(now - started_at, 1.0e-6)
                rate = count / elapsed
                print(
                    "\r采集中：{}/{}，当前平均 {:.2f} Hz，已用 {:.1f} s".format(
                        count, self.args.samples, rate, elapsed
                    ),
                    end="",
                    flush=True,
                )
                last_count = count
            if time.monotonic() - last_sample_at >= self.args.idle_timeout:
                break
            time.sleep(0.03)

        with self.lock:
            self.collecting = False
            captured = list(self.samples[: self.args.samples])
        print()
        if len(captured) < self.args.samples:
            elapsed = max(time.monotonic() - started_at, 1.0e-6)
            raise RuntimeError(
                "{} only received {}/{} samples from {} in {:.1f}s "
                "(average {:.2f} Hz). Keep the blue face detected and retry; "
                "increase --timeout only if points are still arriving.".format(
                    label,
                    len(captured),
                    self.args.samples,
                    self.args.topic,
                    elapsed,
                    len(captured) / elapsed,
                )
            )

        center = tuple(med([sample[i] for sample in captured]) for i in range(3))
        dispersion = tuple(
            mad([sample[i] for sample in captured], center[i]) for i in range(3)
        )
        stable = (
            dispersion[0] <= self.args.max_mad_xy
            and dispersion[1] <= self.args.max_mad_xy
            and dispersion[2] <= self.args.max_mad_z
        )
        result = {
            "label": label,
            "top_center": [round(value, 6) for value in center],
            "mad_xyz": [round(value, 6) for value in dispersion],
            "stable": bool(stable),
            "sample_count": len(captured),
        }
        print(
            "{}: top_center=({:.4f}, {:.4f}, {:.4f}) m, "
            "MAD=({:.1f}, {:.1f}, {:.1f}) mm, stable={}".format(
                label,
                center[0],
                center[1],
                center[2],
                1000.0 * dispersion[0],
                1000.0 * dispersion[1],
                1000.0 * dispersion[2],
                stable,
            )
        )
        return result


def infer_geometry(records, args):
    edge_records = records[1:]
    max_x_record = max(edge_records, key=lambda item: item["top_center"][0])
    min_x_record = min(edge_records, key=lambda item: item["top_center"][0])
    max_y_record = max(edge_records, key=lambda item: item["top_center"][1])
    min_y_record = min(edge_records, key=lambda item: item["top_center"][1])
    selected_labels = {
        max_x_record["label"],
        min_x_record["label"],
        max_y_record["label"],
        min_y_record["label"],
    }

    half_cube = args.cube_size / 2.0
    bounds = {
        "x_min": min_x_record["top_center"][0] - half_cube,
        "x_max": max_x_record["top_center"][0] + half_cube,
        "y_min": min_y_record["top_center"][1] - half_cube,
        "y_max": max_y_record["top_center"][1] + half_cube,
    }
    size_x = bounds["x_max"] - bounds["x_min"]
    size_y = bounds["y_max"] - bounds["y_min"]
    center_x = (bounds["x_min"] + bounds["x_max"]) / 2.0
    center_y = (bounds["y_min"] + bounds["y_max"]) / 2.0

    support_heights = [item["top_center"][2] - args.cube_size for item in records]
    table_top_z = med(support_heights)
    height_spread = max(support_heights) - min(support_heights)
    center_record = records[0]
    center_error = math.hypot(
        center_record["top_center"][0] - center_x,
        center_record["top_center"][1] - center_y,
    )

    observed_sizes = sorted((size_x, size_y))
    expected_sizes = sorted((args.expected_short, args.expected_long))
    sizes_match = all(
        abs(observed - expected) <= args.size_tolerance
        for observed, expected in zip(observed_sizes, expected_sizes)
    )
    unique_edges = len(selected_labels) == 4
    all_stable = all(item["stable"] for item in records)
    height_ok = height_spread <= args.max_height_spread
    center_ok = center_error <= 0.050
    quality_ok = unique_edges and all_stable and height_ok and sizes_match and center_ok

    return {
        "method": "five live cube placements; four edge midpoints plus table centre",
        "frame_id": args.target_frame,
        "cube_size": args.cube_size,
        "table_thickness": args.table_thickness,
        "raw_captures": records,
        "selected_edge_labels": {
            "x_min": min_x_record["label"],
            "x_max": max_x_record["label"],
            "y_min": min_y_record["label"],
            "y_max": max_y_record["label"],
        },
        "table_bounds": {key: round(value, 6) for key, value in bounds.items()},
        "table_top_z": round(table_top_z, 6),
        "height_spread": round(height_spread, 6),
        "table_center": [
            round(center_x, 6),
            round(center_y, 6),
            round(table_top_z - args.table_thickness / 2.0, 6),
        ],
        "table_size": [round(size_x, 6), round(size_y, 6), args.table_thickness],
        "table_yaw": 0.0,
        "center_placement_error": round(center_error, 6),
        "quality": {
            "all_captures_stable": all_stable,
            "four_unique_edge_placements": unique_edges,
            "height_consistent": height_ok,
            "size_matches_caliper": sizes_match,
            "center_placement_consistent": center_ok,
            "quality_ok": quality_ok,
        },
        "safe_to_copy_into_scene": False,
        "note": (
            "Review this report and physically cross-check table_top_z and all bounds. "
            "The active PlanningScene is never modified by this tool."
        ),
    }


def write_report(report, output):
    output = os.path.abspath(output)
    directory = os.path.dirname(output)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, allow_unicode=True, sort_keys=False)
    return output


def main():
    rospy.init_node("calibrate_table_from_live_cube", anonymous=False)
    args = parse_args()
    if args.cube_size <= 0.0 or args.table_thickness <= 0.0:
        raise RuntimeError("cube size and table thickness must be > 0")
    if args.samples < 15:
        raise RuntimeError("use at least 15 samples per placement")
    if args.timeout <= 0.0 or args.idle_timeout <= 0.0:
        raise RuntimeError("timeout and idle timeout must be > 0")

    probe = TableCubeProbe(args)
    print("\n本工具只读取相机检测，不会规划或移动机械臂。")
    print("四次边缘放置都应位于对应边的中点，魔方侧面与桌板边缘贴齐。")
    print("四条边可以按任意顺序采集，程序将按 base 坐标自动识别 +/-X、+/-Y。")
    records = [
        probe.capture("center", "把魔方放在桌板几何中心附近。"),
        probe.capture("edge_1", "把魔方贴齐第一条边的中点。"),
        probe.capture("edge_2", "把魔方贴齐第二条边的中点。"),
        probe.capture("edge_3", "把魔方贴齐第三条边的中点。"),
        probe.capture("edge_4", "把魔方贴齐第四条边的中点。"),
    ]
    report = infer_geometry(records, args)
    output = write_report(report, args.output)

    print("\n推算结果（尚未写入 MoveIt）：")
    print("  bounds = {}".format(report["table_bounds"]))
    print("  top_z = {:.4f} m".format(report["table_top_z"]))
    print("  center = {}".format(report["table_center"]))
    print("  size = {}".format(report["table_size"]))
    print("  quality = {}".format(report["quality"]))
    print("报告已保存：{}".format(output))
    if not report["quality"]["quality_ok"]:
        print("质量检查未通过：不要据此更新或执行 MoveIt 场景。")
        return 2
    print("质量检查通过，但仍需把报告发给我并用卷尺复核后才能更新场景。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n用户取消，PlanningScene 未改变。")
        sys.exit(130)
    except Exception as exc:
        rospy.logerr("Table cube probe failed: %s", exc)
        sys.exit(1)
