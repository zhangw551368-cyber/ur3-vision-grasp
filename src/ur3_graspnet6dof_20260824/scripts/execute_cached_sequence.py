#!/usr/bin/python3

"""Execute a previously inspected sequence without reading perception topics."""

import argparse
import copy
import json
import math
import pickle
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import moveit_commander
import rospy
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import String

from graspnet_pick_executor import GuardedGraspExecutor
from ur3_graspnet6dof.config import load_config
from ur3_graspnet6dof.target_validation import nearest_target


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", default=str(PROJECT_ROOT / "config/right_arm_green_table.yaml")
    )
    parser.add_argument(
        "--cache", default=str(PROJECT_ROOT / "runtime/cached_sequence_plan.pkl")
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--max-age", type=float, default=1800.0)
    parser.add_argument("--start-tolerance", type=float, default=0.03)
    parser.add_argument("--max-target-drift-px", type=float, default=18.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def angle_error(current, expected):
    return abs(math.atan2(math.sin(current - expected), math.cos(current - expected)))


def verify_cached_start(executor, plan, tolerance):
    expected = dict(
        zip(
            plan["start_state"].joint_state.name,
            plan["start_state"].joint_state.position,
        )
    )
    current_state = executor.arm.get_current_state().joint_state
    current = dict(zip(current_state.name, current_state.position))
    errors = {
        name: angle_error(float(current[name]), float(expected[name]))
        for name in executor.arm.get_active_joints()
        if name in current and name in expected
    }
    if len(errors) != len(executor.arm.get_active_joints()):
        raise RuntimeError("cached/current right-arm joint sets do not match")
    worst_name = max(errors, key=errors.get)
    if errors[worst_name] > float(tolerance):
        raise RuntimeError(
            "robot moved since planning: {} differs by {:.4f} rad".format(
                worst_name, errors[worst_name]
            )
        )


def verify_frozen_target(item, max_drift_px):
    frozen = item.get("target_pixel")
    if frozen is None:
        raise RuntimeError("cached item has no frozen target pixel")
    topic = item["config"]["ros"]["namespace"] + "/detected_objects_json"
    message = rospy.wait_for_message(topic, String, timeout=5.0)
    payload = json.loads(message.data)
    stamp = payload.get("stamp", {})
    stamp_value = float(stamp.get("secs", 0)) + float(stamp.get("nsecs", 0)) * 1e-9
    age = rospy.Time.now().to_sec() - stamp_value
    if age > float(item["config"]["sequence"]["max_detection_age"]):
        raise RuntimeError("target recheck image is stale by {:.2f}s".format(age))
    nearest, distance = nearest_target(payload["objects"], item["category"], frozen)
    if nearest is None:
        raise RuntimeError("target category is no longer visible at observation pose")
    if distance > float(max_drift_px):
        raise RuntimeError(
            "target moved {:.1f}px from frozen coordinate (limit {:.1f}px)".format(
                distance, float(max_drift_px)
            )
        )
    return distance, nearest["center"]


def return_initial(executor):
    trajectory, _ = executor.plan_initial_joints(executor.arm.get_current_state())
    if not executor.arm.execute(trajectory, wait=True):
        executor.arm.stop()
        raise RuntimeError("failed to return to cached initial state")
    executor.arm.stop()


def main():
    args = parse_args()
    config = load_config(args.config)
    if not args.execute or not args.yes:
        raise RuntimeError("cached real execution requires --execute --yes")
    if not config["execution"].get("enabled", False):
        raise RuntimeError("cached real execution is locked by execution.enabled=false")

    cache_path = Path(args.cache).resolve()
    if not str(cache_path).startswith(str(PROJECT_ROOT.resolve())):
        raise RuntimeError("cached plan must be inside the isolated project")
    with cache_path.open("rb") as stream:
        cache = pickle.load(stream)
    if cache.get("schema_version") != 1 or not cache.get("items"):
        raise RuntimeError("cached sequence is empty or unsupported")
    age = time.time() - float(cache["created_at"])
    if age < 0.0 or age > float(args.max_age):
        raise RuntimeError("cached sequence age {:.1f}s exceeds limit".format(age))

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_graspnet6d_execute_cached_sequence", anonymous=True)
    queue = []
    display = DisplayTrajectory()
    for item in cache["items"]:
        item_config = copy.deepcopy(item["config"])
        item_config["execution"]["enabled"] = True
        executor = GuardedGraspExecutor(item_config, True, "pick_drop")
        plan = item["plan"]
        queue.append((item, executor, plan))
        if not display.trajectory:
            display.trajectory_start = plan["start_state"]
        display.trajectory.extend(
            [trajectory for _, trajectory in plan["trajectories"]]
        )

    display_pub = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
    )
    for _ in range(3):
        display_pub.publish(display)
        rospy.sleep(0.2)

    results = []
    target_recheck_count = 0
    for item, executor, plan in queue:
        result = {
            "order": item["order"],
            "category": item["category"],
            "status": "executing",
        }
        try:
            verify_cached_start(executor, plan, args.start_tolerance)
            # Every cached item returns to the high observation pose. Recheck
            # only there, never while the arm occludes the fixed camera.
            if config["execution"].get("recheck_frozen_targets", False):
                rospy.sleep(0.8)
                try:
                    drift, observed = verify_frozen_target(
                        item, args.max_target_drift_px
                    )
                    target_recheck_count += 1
                    result["target_drift_px"] = drift
                    result["observed_pixel"] = observed
                except Exception as drift_error:
                    result["status"] = "skipped_target_moved_or_missing"
                    result["error"] = str(drift_error)
                    results.append(result)
                    output = {
                        "schema_version": 1,
                        "cache_file": str(cache_path),
                        "cache_age_at_start": age,
                        "camera_reads_during_execution": target_recheck_count,
                        "camera_reads_while_arm_occluding": 0,
                        "results": results,
                    }
                    (PROJECT_ROOT / "runtime/cached_execution_result.json").write_text(
                        json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
                    )
                    continue
            else:
                result["target_recheck"] = "disabled_static_scene"
            executor.execute_plan(plan)
            result["status"] = "executed"
        except Exception as exc:
            result["status"] = "execution_failed"
            result["error"] = str(exc)
            executor.recover_after_failure(exc)
            return_initial(executor)
            result["recovery"] = "returned_initial; sequence_stopped"
            results.append(result)
            output = {
                "schema_version": 1,
                "cache_file": str(cache_path),
                "cache_age_at_start": age,
                "camera_reads_during_execution": target_recheck_count,
                "camera_reads_while_arm_occluding": 0,
                "results": results,
            }
            (PROJECT_ROOT / "runtime/cached_execution_result.json").write_text(
                json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            break
        results.append(result)
        output = {
            "schema_version": 1,
            "cache_file": str(cache_path),
            "cache_age_at_start": age,
            "camera_reads_during_execution": target_recheck_count,
            "camera_reads_while_arm_occluding": 0,
            "results": results,
        }
        (PROJECT_ROOT / "runtime/cached_execution_result.json").write_text(
            json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    if target_recheck_count:
        rospy.logwarn(
            "CACHED SEQUENCE complete: %d considered; %d target rechecks were performed only at the unobstructed observation pose.",
            len(results),
            target_recheck_count,
        )
    else:
        rospy.logwarn(
            "CACHED SEQUENCE complete: %d considered; static-scene mode used no camera reads during execution.",
            len(results),
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logerr(str(exc))
        else:
            print("ERROR: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
