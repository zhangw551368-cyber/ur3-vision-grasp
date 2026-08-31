#!/usr/bin/python3

import argparse
import copy
import json
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))

import moveit_commander
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsProgramRunning
from visualization_msgs.msg import Marker

from ur3_graspnet6dof.config import load_config
from ur3_graspnet6dof.geometry import (
    grasp_to_tool_rotation,
    matrix_to_quaternion,
    normalize,
    plane_point,
    point_plane_distance,
    quaternion_to_matrix,
    transform_matrix,
    transform_point,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Select, plan and optionally execute a guarded GraspNet UR3 pick"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "right_arm_green_table.yaml"),
    )
    parser.add_argument(
        "--mode",
        choices=("pick_hold", "pick_drop"),
        default="",
        help="Path to plan. Default comes from execution.default_mode.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Send robot and gripper commands. Default is strictly plan-only.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip typed EXECUTE confirmation when --execute is set.",
    )
    parser.add_argument(
        "--network-rank",
        type=int,
        default=-1,
        help="Only consider one zero-based GraspNet rank. Default considers all.",
    )
    parser.add_argument(
        "--target-pixel",
        nargs=2,
        type=int,
        metavar=("U", "V"),
        help="Restrict inference to one pictured workpiece near pixel U,V.",
    )
    parser.add_argument(
        "--target-radius",
        type=int,
        default=65,
        help="Pixel radius used with --target-pixel (default: 65).",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def make_gripper_command(position, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = int(position)
    command.rSP = int(speed)
    command.rFR = int(force)
    return command


class GuardedGraspExecutor:
    def __init__(
        self,
        config,
        execute,
        mode,
        network_rank=-1,
        target_pixel=None,
        target_radius=65,
    ):
        self.config = config
        self.execute = bool(execute)
        self.mode = mode
        self.network_rank = network_rank
        self.target_pixel = target_pixel
        self.target_radius = int(target_radius)
        self.selector = config["selector"]
        self.tool = config["tool"]
        self.moveit = config["moveit"]
        self.execution = config["execution"]
        self.safety_scene = config.get("safety_scene", {})
        self.ros_config = config["ros"]

        self.arm = moveit_commander.MoveGroupCommander(self.moveit["arm_group"])
        self.arm.set_pose_reference_frame(self.selector["planning_frame"])
        self.arm.set_end_effector_link(self.moveit["end_effector_link"])
        self.arm.set_max_velocity_scaling_factor(float(self.moveit["velocity_scaling"]))
        self.arm.set_max_acceleration_scaling_factor(float(self.moveit["acceleration_scaling"]))
        self.arm.set_planning_time(float(self.moveit["planning_time"]))
        self.arm.set_num_planning_attempts(int(self.moveit["planning_attempts"]))
        self.arm.set_planner_id(self.moveit["planner_id"])
        self.arm.set_goal_position_tolerance(float(self.moveit["goal_position_tolerance"]))
        self.arm.set_goal_orientation_tolerance(float(self.moveit["goal_orientation_tolerance"]))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.display_pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
        )
        self.selected_pub = rospy.Publisher(
            self.ros_config["namespace"] + "/selected_grasp_base",
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.selected_marker_pub = rospy.Publisher(
            self.ros_config["namespace"] + "/selected_marker",
            Marker,
            queue_size=1,
            latch=True,
        )
        self.gripper_pub = rospy.Publisher(
            self.execution["gripper_topic"],
            Robotiq2FGripper_robot_output,
            queue_size=1,
        )
        self.grasp_confirmed = False
        self.last_execution_stage = "not_started"
        self.support_plane_base = None

    @staticmethod
    def trajectory(result):
        return result[1] if isinstance(result, tuple) else result

    @staticmethod
    def trajectory_duration(trajectory):
        if not trajectory.joint_trajectory.points:
            return math.inf
        return trajectory.joint_trajectory.points[-1].time_from_start.to_sec()

    @staticmethod
    def trajectory_joint_motion(trajectory):
        points = trajectory.joint_trajectory.points
        if len(points) < 2:
            return 0.0
        return sum(
            abs(current - previous)
            for first, second in zip(points, points[1:])
            for previous, current in zip(first.positions, second.positions)
        )

    def end_state(self, start_state, trajectory):
        state = copy.deepcopy(start_state)
        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)
        velocities = list(state.joint_state.velocity)
        efforts = list(state.joint_state.effort)
        final = trajectory.joint_trajectory.points[-1]
        for name, position in zip(trajectory.joint_trajectory.joint_names, final.positions):
            if name in names:
                index = names.index(name)
                positions[index] = position
            else:
                names.append(name)
                positions.append(position)
                velocities.append(0.0)
                efforts.append(0.0)
        state.joint_state.name = names
        state.joint_state.position = positions
        state.joint_state.velocity = velocities
        state.joint_state.effort = efforts
        return state

    def pose_stamped(self, position, rotation):
        target = PoseStamped()
        target.header.frame_id = self.selector["planning_frame"]
        target.header.stamp = rospy.Time.now()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        quaternion = matrix_to_quaternion(rotation)
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = quaternion
        return target

    def plan_pose(self, name, target, start_state):
        retries = max(1, int(self.moveit["plan_retries"]))
        for attempt in range(1, retries + 1):
            self.arm.set_start_state(start_state)
            self.arm.set_pose_target(target)
            trajectory = self.trajectory(self.arm.plan())
            self.arm.clear_pose_targets()
            if not trajectory.joint_trajectory.points:
                continue
            duration = self.trajectory_duration(trajectory)
            motion = self.trajectory_joint_motion(trajectory)
            if duration > float(self.moveit["max_pose_duration"]):
                rospy.logwarn(
                    "%s plan %d rejected: duration %.2fs", name, attempt, duration
                )
                continue
            if motion > float(self.moveit["max_pose_joint_motion"]):
                rospy.logwarn(
                    "%s plan %d rejected: joint motion %.2frad", name, attempt, motion
                )
                continue
            return trajectory, self.end_state(start_state, trajectory)
        raise RuntimeError("no accepted MoveIt pose plan for {}".format(name))

    def plan_cartesian(self, name, target, start_state):
        self.arm.set_start_state(start_state)
        trajectory, fraction = self.arm.compute_cartesian_path(
            [target.pose],
            float(self.moveit["cartesian_step"]),
            avoid_collisions=True,
        )
        if (
            fraction < float(self.moveit["cartesian_min_fraction"])
            or not trajectory.joint_trajectory.points
        ):
            raise RuntimeError(
                "Cartesian {} fraction {:.3f} < {:.3f}".format(
                    name, fraction, float(self.moveit["cartesian_min_fraction"])
                )
            )
        return trajectory, self.end_state(start_state, trajectory)

    def plan_initial_joints(self, start_state):
        values = self.execution.get("initial_joint_target")
        joints = self.arm.get_active_joints()
        if values is None or len(values) != len(joints):
            raise RuntimeError("initial_joint_target does not match MoveIt active joints")
        self.arm.set_start_state(start_state)
        self.arm.set_joint_value_target(
            dict(zip(joints, [float(value) for value in values]))
        )
        trajectory = self.trajectory(self.arm.plan())
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("no MoveIt plan back to initial joints")
        if self.trajectory_duration(trajectory) > float(self.moveit["max_pose_duration"]):
            raise RuntimeError("return-to-initial trajectory duration is too long")
        if self.trajectory_joint_motion(trajectory) > float(
            self.moveit["max_pose_joint_motion"]
        ):
            raise RuntimeError("return-to-initial joint motion is too large")
        return trajectory, self.end_state(start_state, trajectory)

    def request_candidates(self):
        service_name = self.ros_config["inference_service"]
        target_param = self.ros_config["namespace"] + "/target_pixel"
        if self.target_pixel is None:
            if rospy.has_param(target_param):
                rospy.delete_param(target_param)
        else:
            if self.target_radius <= 0:
                raise RuntimeError("target radius must be positive")
            rospy.set_param(
                target_param,
                [int(self.target_pixel[0]), int(self.target_pixel[1]), self.target_radius],
            )
        rospy.wait_for_service(service_name, timeout=10.0)
        trigger = rospy.ServiceProxy(service_name, Trigger, persistent=False)
        try:
            response = trigger()
        finally:
            if self.target_pixel is not None and rospy.has_param(target_param):
                rospy.delete_param(target_param)
        if not response.success:
            raise RuntimeError("GraspNet inference failed: {}".format(response.message))
        message = rospy.wait_for_message(
            self.ros_config["candidates_json_topic"], String, timeout=3.0
        )
        payload = json.loads(message.data)
        if payload.get("schema_version") != 1:
            raise RuntimeError("unsupported candidate schema")
        if not payload.get("candidates"):
            raise RuntimeError("GraspNet returned no candidates")
        age = rospy.Time.now().to_sec() - (
            float(payload["stamp"]["secs"]) + float(payload["stamp"]["nsecs"]) * 1e-9
        )
        if age > float(self.selector["max_candidate_age"]):
            raise RuntimeError("candidate RGB-D frame is stale by {:.2f}s".format(age))
        return payload

    def camera_transform(self, frame_id):
        transform = self.tf_buffer.lookup_transform(
            self.selector["planning_frame"],
            frame_id,
            rospy.Time(0),
            rospy.Duration(float(self.selector["tf_timeout"])),
        ).transform
        return transform_matrix(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
        )

    @staticmethod
    def transformed_plane(plane_camera, base_from_camera):
        plane_camera = np.asarray(plane_camera, dtype=np.float64)
        point_camera = plane_point(plane_camera)
        normal_base = normalize(base_from_camera[:3, :3].dot(plane_camera[:3]))
        point_base = transform_point(base_from_camera, point_camera)
        if normal_base[2] < 0.0:
            normal_base = -normal_base
        return np.r_[normal_base, -normal_base.dot(point_base)]

    @staticmethod
    def plane_z_at(plane, x, y):
        if abs(float(plane[2])) < 1e-8:
            raise ValueError("support plane is vertical")
        return -(float(plane[0]) * x + float(plane[1]) * y + float(plane[3])) / float(plane[2])

    def candidate_rejections(self, candidate, center_base, approach_base, plane_base):
        reasons = []
        score = float(candidate["score"])
        width = float(candidate["width"])
        if score < float(self.selector["min_score"]):
            reasons.append("score")
        if not (
            float(self.selector["min_gripper_width"])
            <= width
            <= float(self.selector["max_gripper_width"])
        ):
            reasons.append("width")

        down_alignment = float(np.dot(normalize(approach_base), [0.0, 0.0, -1.0]))
        required = math.cos(math.radians(float(self.selector["max_approach_tilt_deg"])))
        if down_alignment < required:
            reasons.append("approach_tilt")

        bounds = self.selector["workspace_bounds"]
        for index, axis in enumerate(("x", "y", "z")):
            lower, upper = [float(value) for value in bounds[axis]]
            if not lower <= center_base[index] <= upper:
                reasons.append("workspace_{}".format(axis))

        if plane_base is not None:
            height = float(np.dot(plane_base[:3], center_base) + plane_base[3])
            if not (
                float(self.selector["min_height_above_plane"])
                <= height
                <= float(self.selector["max_height_above_plane"])
            ):
                reasons.append("height_above_plane")
        return reasons

    def prepare_candidates(self, payload):
        base_from_camera = self.camera_transform(payload["frame_id"])
        plane_camera = payload.get("diagnostics", {}).get("support_plane_camera")
        plane_base = None
        if plane_camera is not None:
            plane_base = self.transformed_plane(plane_camera, base_from_camera)
            self.support_plane_base = plane_base
            center_x = sum(self.selector["workspace_bounds"]["x"]) / 2.0
            center_y = sum(self.selector["workspace_bounds"]["y"]) / 2.0
            fitted_z = self.plane_z_at(plane_base, center_x, center_y)
            expected_z = float(self.selector["support_plane_z"])
            rospy.loginfo(
                "Fitted support plane in %s: normal=%s z_center=%.4f expected=%.4f",
                self.selector["planning_frame"],
                np.round(plane_base[:3], 4).tolist(),
                fitted_z,
                expected_z,
            )
            if abs(fitted_z - expected_z) > float(
                self.selector["support_plane_z_tolerance"]
            ):
                raise RuntimeError(
                    "fitted support plane z {:.3f} differs from configured {:.3f}".format(
                        fitted_z, expected_z
                    )
                )
            if abs(float(plane_base[2])) < math.cos(math.radians(15.0)):
                raise RuntimeError("fitted support plane is not approximately horizontal")
        elif self.selector.get("require_support_plane", True):
            raise RuntimeError("GraspNet result has no fitted support plane")

        prepared = []
        candidates = payload["candidates"]
        if self.network_rank >= 0:
            if self.network_rank >= len(candidates):
                raise RuntimeError("network rank {} does not exist".format(self.network_rank))
            candidates = [candidates[self.network_rank]]

        for candidate in candidates:
            rotation_camera = np.asarray(candidate["rotation"], dtype=np.float64)
            center_camera = np.asarray(candidate["translation"], dtype=np.float64)
            rotation_base = base_from_camera[:3, :3].dot(rotation_camera)
            center_base = transform_point(base_from_camera, center_camera)
            approach_base = normalize(rotation_base[:, 0])
            if self.tool.get("force_vertical_approach", False):
                approach_base = np.array([0.0, 0.0, -1.0])
                opening = np.asarray(rotation_base[:, 1], dtype=np.float64)
                opening = opening - approach_base * float(np.dot(opening, approach_base))
                opening = normalize(opening)
                lateral = normalize(np.cross(approach_base, opening))
                rotation_base = np.column_stack((approach_base, opening, lateral))
            center_base = center_base + approach_base * float(
                self.tool.get("grasp_center_approach_offset", 0.0)
            )
            reasons = []
            opening_axis_error = None
            desired_opening_xy = self.selector.get("opening_axis_base_xy")
            if desired_opening_xy is not None:
                desired = normalize(
                    np.array(
                        [float(desired_opening_xy[0]), float(desired_opening_xy[1]), 0.0]
                    )
                )
                actual = np.asarray(rotation_base[:, 1], dtype=np.float64).copy()
                actual[2] = 0.0
                actual = normalize(actual)
                # Opening axes are bidirectional; +axis and -axis represent
                # the same pair of long faces.
                cosine = max(-1.0, min(1.0, abs(float(np.dot(actual, desired)))))
                opening_axis_error = math.degrees(math.acos(cosine))
                limit = float(self.selector.get("max_opening_axis_error_deg", 20.0))
                if opening_axis_error > limit:
                    reasons.append(
                        "opening_axis_error={:.1f}deg>{:.1f}deg".format(
                            opening_axis_error, limit
                        )
                    )
            reasons.extend(self.candidate_rejections(
                candidate, center_base, approach_base, plane_base
            ))
            if reasons:
                rospy.loginfo(
                    "Reject grasp rank=%d score=%.3f: %s",
                    int(candidate["id"]),
                    float(candidate["score"]),
                    ",".join(reasons),
                )
                continue
            prepared.append(
                {
                    "source": candidate,
                    "center": center_base,
                    "grasp_rotation": rotation_base,
                    "approach": approach_base,
                    "height_above_plane": (
                        float(np.dot(plane_base[:3], center_base) + plane_base[3])
                        if plane_base is not None
                        else None
                    ),
                    "opening_axis_error_deg": opening_axis_error,
                }
            )
        return prepared

    def update_observed_surface_guard(self):
        if not self.safety_scene.get("enabled", True):
            return
        if self.support_plane_base is None:
            raise RuntimeError("cannot create observed surface guard without a plane")
        from moveit_commander import PlanningSceneInterface

        plane = self.support_plane_base
        normal = normalize(plane[:3])
        base_x = np.array([1.0, 0.0, 0.0])
        local_x = base_x - normal * float(np.dot(base_x, normal))
        if np.linalg.norm(local_x) < 1e-6:
            base_y = np.array([0.0, 1.0, 0.0])
            local_x = base_y - normal * float(np.dot(base_y, normal))
        local_x = normalize(local_x)
        local_y = normalize(np.cross(normal, local_x))
        rotation = np.column_stack((local_x, local_y, normal))

        center_x, center_y = [float(value) for value in self.safety_scene["center_xy"]]
        surface_z = self.plane_z_at(plane, center_x, center_y)
        thickness = float(self.safety_scene["thickness"])
        center = np.array([center_x, center_y, surface_z]) - normal * thickness / 2.0
        pose = self.pose_stamped(center, rotation)
        object_id = self.safety_scene["object_id"]
        size_x, size_y = [float(value) for value in self.safety_scene["size_xy"]]
        scene = PlanningSceneInterface(synchronous=True)
        scene.add_box(object_id, pose, size=(size_x, size_y, thickness))
        deadline = time.time() + float(self.safety_scene["update_timeout"])
        while time.time() < deadline:
            if object_id in scene.get_known_object_names():
                rospy.loginfo(
                    "Observed surface guard ready: %s centre=%s normal=%s size=[%.3f,%.3f,%.3f]",
                    object_id,
                    np.round(center, 4).tolist(),
                    np.round(normal, 4).tolist(),
                    size_x,
                    size_y,
                    thickness,
                )
                return
            rospy.sleep(0.05)
        raise RuntimeError("observed support-surface guard was not accepted by MoveIt")

    def targets_for_candidate(self, prepared, flip):
        approach = prepared["approach"]
        center = prepared["center"]
        tool_rotation = grasp_to_tool_rotation(prepared["grasp_rotation"], flip)
        tool0_grasp = center - approach * float(self.tool["tcp_offset_from_tool0"])
        tool0_pregrasp = tool0_grasp - approach * float(self.tool["pregrasp_distance"])
        tool0_lift = tool0_grasp + np.array([0.0, 0.0, float(self.tool["lift_distance"])])
        return {
            "pregrasp": self.pose_stamped(tool0_pregrasp, tool_rotation),
            "grasp": self.pose_stamped(tool0_grasp, tool_rotation),
            "lift": self.pose_stamped(tool0_lift, tool_rotation),
        }

    def append_drop_targets(self, targets):
        if self.mode != "pick_drop":
            return targets
        if not self.execution.get("drop_enabled", False):
            raise RuntimeError(
                "pick_drop is locked: teach a clear drop pose and set execution.drop_enabled=true"
            )
        drop = self.execution["drop_pose"]
        if drop["frame_id"] != self.selector["planning_frame"]:
            raise RuntimeError("drop pose must currently use the planning frame")
        if self.execution.get("drop_keep_grasp_orientation", False):
            grasp_orientation = targets["grasp"].pose.orientation
            rotation = quaternion_to_matrix(
                [
                    grasp_orientation.x,
                    grasp_orientation.y,
                    grasp_orientation.z,
                    grasp_orientation.w,
                ]
            )
        else:
            rotation = quaternion_to_matrix(drop["quaternion"])
        tcp_position = np.asarray(drop["position"], dtype=float)
        tool_position = tcp_position
        if self.execution.get("drop_pose_is_tcp", True):
            tool_position = tcp_position - rotation[:, 2] * float(
                self.tool["tcp_offset_from_tool0"]
            )
        pre_position = tool_position - rotation[:, 2] * float(
            self.execution["drop_pre_clearance"]
        )
        targets["drop_pre"] = self.pose_stamped(pre_position, rotation)
        targets["drop"] = self.pose_stamped(tool_position, rotation)
        return targets

    def try_plan_candidate(self, prepared, flip):
        targets = self.append_drop_targets(self.targets_for_candidate(prepared, flip))
        current = self.arm.get_current_state()
        trajectories = []
        pre, state = self.plan_pose("pregrasp", targets["pregrasp"], current)
        trajectories.append(("pregrasp", pre))
        descent, state = self.plan_cartesian("grasp", targets["grasp"], state)
        trajectories.append(("grasp", descent))
        lift, state = self.plan_cartesian("lift", targets["lift"], state)
        trajectories.append(("lift", lift))
        if self.mode == "pick_drop":
            drop_pre, state = self.plan_pose("drop_pre", targets["drop_pre"], state)
            trajectories.append(("drop_pre", drop_pre))
            drop, state = self.plan_cartesian("drop", targets["drop"], state)
            trajectories.append(("drop", drop))
            if self.execution.get("return_to_initial_after_drop", True):
                return_trajectory, state = self.plan_initial_joints(state)
                trajectories.append(("return_initial", return_trajectory))
        return {
            "candidate": prepared,
            "flip": flip,
            "targets": targets,
            "start_state": current,
            "trajectories": trajectories,
        }

    def select_plan(self, candidates):
        limit = min(len(candidates), int(self.selector["max_candidates_to_plan"]))
        flip_options = [False, True] if self.selector.get("try_opening_axis_flip", True) else [False]
        for prepared in candidates[:limit]:
            for flip in flip_options:
                try:
                    rospy.loginfo(
                        "Planning grasp rank=%d score=%.3f width=%.1fmm flip=%s",
                        int(prepared["source"]["id"]),
                        float(prepared["source"]["score"]),
                        float(prepared["source"]["width"]) * 1000.0,
                        flip,
                    )
                    return self.try_plan_candidate(prepared, flip)
                except Exception as exc:
                    rospy.logwarn(
                        "Candidate rank=%d flip=%s is not executable: %s",
                        int(prepared["source"]["id"]),
                        flip,
                        exc,
                    )
        raise RuntimeError("none of the filtered GraspNet candidates has a complete MoveIt path")

    def publish_plan(self, plan):
        display = DisplayTrajectory()
        display.trajectory_start = plan["start_state"]
        display.trajectory = [trajectory for _, trajectory in plan["trajectories"]]
        self.display_pub.publish(display)
        selected = plan["targets"]["grasp"]
        self.selected_pub.publish(selected)

        marker = Marker()
        marker.header = selected.header
        marker.ns = "selected_grasp"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        approach = plan["candidate"]["approach"]
        center = plan["candidate"]["center"]
        start = center - approach * float(self.tool["pregrasp_distance"])
        marker.points = [
            Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
            Point(x=float(center[0]), y=float(center[1]), z=float(center[2])),
        ]
        marker.scale.x = 0.008
        marker.scale.y = 0.020
        marker.scale.z = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.selected_marker_pub.publish(marker)
        self.write_plan_report(plan)
        time.sleep(float(self.moveit["display_pause_seconds"]))

    def write_plan_report(self, plan):
        runtime_dir = Path(self.config["_project_root"]) / self.config["paths"]["runtime_dir"]
        runtime_dir.mkdir(parents=True, exist_ok=True)
        source = plan["candidate"]["source"]
        report = {
            "schema_version": 1,
            "created_at": time.time(),
            "execute_requested": self.execute,
            "mode": self.mode,
            "candidate": {
                "network_rank": int(source["id"]),
                "score": float(source["score"]),
                "width": float(source["width"]),
                "center_base": plan["candidate"]["center"].tolist(),
                "approach_base": plan["candidate"]["approach"].tolist(),
                "height_above_plane": plan["candidate"]["height_above_plane"],
                "opening_axis_flip": bool(plan["flip"]),
            },
            "trajectories": [
                {
                    "name": name,
                    "points": len(trajectory.joint_trajectory.points),
                    "duration": self.trajectory_duration(trajectory),
                    "joint_motion": self.trajectory_joint_motion(trajectory),
                }
                for name, trajectory in plan["trajectories"]
            ],
        }
        temp = runtime_dir / "latest_plan.json.tmp"
        final = runtime_dir / "latest_plan.json"
        temp.write_text(json.dumps(report, indent=2), encoding="utf-8")
        os.replace(str(temp), str(final))

    def ensure_external_control(self):
        topic = self.execution["robot_program_topic"]
        running = rospy.wait_for_message(topic, Bool, timeout=2.0)
        if not running.data:
            raise RuntimeError("External Control topic reports not running")
        service_name = self.execution["dashboard_program_running_service"]
        rospy.wait_for_service(service_name, timeout=2.0)
        response = rospy.ServiceProxy(service_name, IsProgramRunning)()
        if not response.success or not response.program_running:
            raise RuntimeError("dashboard reports External Control not running")

    def command_gripper(self, position, label):
        rospy.loginfo("Gripper %s rPR=%d", label, int(position))
        deadline = time.time() + 5.0
        while self.gripper_pub.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.gripper_pub.get_num_connections() == 0:
            raise RuntimeError("no subscriber on {}".format(self.execution["gripper_topic"]))
        self.gripper_pub.publish(
            make_gripper_command(
                position,
                self.execution["gripper_speed"],
                self.execution["gripper_force"],
            )
        )
        rospy.sleep(float(self.execution["gripper_settle_seconds"]))

    def require_grasp(self):
        if not self.execution.get("require_grasp_detection", True):
            self.grasp_confirmed = True
            return
        deadline = time.time() + float(self.execution["grasp_detection_timeout"])
        last = None
        while time.time() < deadline:
            try:
                last = rospy.wait_for_message(
                    self.execution["gripper_status_topic"],
                    Robotiq2FGripper_robot_input,
                    timeout=0.5,
                )
            except rospy.ROSException:
                continue
            if last.gFLT:
                raise RuntimeError("Robotiq fault gFLT={}".format(last.gFLT))
            if last.gOBJ == 2:
                self.grasp_confirmed = True
                rospy.loginfo("Robotiq detected object: gOBJ=2 gPO=%d", last.gPO)
                return
        if last is None:
            raise RuntimeError("no Robotiq status received after closing")
        raise RuntimeError("Robotiq did not detect an object: gOBJ={} gPO={}".format(last.gOBJ, last.gPO))

    def execute_plan(self, plan):
        self.ensure_external_control()
        self.command_gripper(self.execution["open_position"], "open")
        for name, trajectory in plan["trajectories"]:
            self.last_execution_stage = name
            if name == "lift":
                self.command_gripper(self.execution["close_position"], "close")
                try:
                    self.require_grasp()
                except Exception:
                    self.command_gripper(self.execution["open_position"], "open_after_miss")
                    raise
            if not self.arm.execute(trajectory, wait=True):
                self.arm.stop()
                raise RuntimeError("trajectory execution failed at {}".format(name))
            # execute(wait=True) has already reached the segment endpoint.
            # Calling stop() here publishes an asynchronous cancellation that
            # can race with and PREEMPT the immediately following segment.
            rospy.sleep(float(self.execution.get("segment_settle_seconds", 0.20)))
            if name == "drop":
                self.command_gripper(self.execution["open_position"], "release")
                self.grasp_confirmed = False
        self.last_execution_stage = "complete"

    def recover_after_failure(self, reason):
        if not self.execute or self.last_execution_stage in ("not_started", "complete"):
            return
        rospy.logerr(
            "Execution stopped at %s: %s. Attempting open + vertical clearance.",
            self.last_execution_stage,
            reason,
        )
        try:
            self.ensure_external_control()
            if not self.grasp_confirmed:
                self.command_gripper(self.execution["open_position"], "recovery_open")
            current = self.arm.get_current_pose(self.moveit["end_effector_link"])
            target = copy.deepcopy(current)
            target.header.frame_id = self.selector["planning_frame"]
            target.pose.position.z += 0.080
            trajectory, fraction = self.arm.compute_cartesian_path(
                [target.pose],
                float(self.moveit["cartesian_step"]),
                avoid_collisions=True,
            )
            if fraction >= float(self.moveit["cartesian_min_fraction"]):
                if not self.arm.execute(trajectory, wait=True):
                    self.arm.stop()
                    raise RuntimeError("vertical recovery trajectory was preempted")
            else:
                rospy.logerr("Recovery vertical path fraction %.3f; motion not sent", fraction)
        except Exception as recovery_error:
            rospy.logerr("Automatic recovery was not possible: %s", recovery_error)

    def run(self):
        payload = self.request_candidates()
        candidates = self.prepare_candidates(payload)
        if not candidates:
            raise RuntimeError("all GraspNet candidates were rejected by real-robot filters")
        self.update_observed_surface_guard()
        rospy.loginfo("%d candidates passed geometric and safety filters", len(candidates))
        plan = self.select_plan(candidates)
        self.publish_plan(plan)
        source = plan["candidate"]["source"]
        rospy.loginfo(
            "Selected rank=%d score=%.3f width=%.1fmm center=%s height=%.1fmm",
            int(source["id"]),
            float(source["score"]),
            float(source["width"]) * 1000.0,
            np.round(plan["candidate"]["center"], 4).tolist(),
            (plan["candidate"]["height_above_plane"] or 0.0) * 1000.0,
        )
        if not self.execute:
            rospy.logwarn(
                "PLAN-ONLY complete. No robot or gripper command was sent. Inspect RViz selected red arrow and full trajectory."
            )
            return
        try:
            self.execute_plan(plan)
        except Exception as exc:
            self.recover_after_failure(exc)
            raise
        if self.mode == "pick_hold":
            rospy.logwarn("Pick complete. Robot is holding the detected workpiece after lift.")
        else:
            rospy.loginfo("Pick-and-drop complete.")


def main():
    args = parse_args()
    config = load_config(args.config)
    mode = args.mode or config["execution"]["default_mode"]
    if args.execute and not config["execution"].get("enabled", False):
        raise RuntimeError(
            "Real execution is locked. Complete plan-only inspection, then set execution.enabled=true in the new YAML."
        )
    if args.execute and not args.yes:
        answer = input(
            "Green table clear, E-stop reachable, camera/TF/plan checked? Type EXECUTE: "
        )
        if answer != "EXECUTE":
            print("Cancelled")
            return

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_graspnet6d_pick_executor")
    executor = GuardedGraspExecutor(
        config,
        execute=args.execute,
        mode=mode,
        network_rank=args.network_rank,
        target_pixel=args.target_pixel,
        target_radius=args.target_radius,
    )
    executor.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if rospy.core.is_initialized():
            rospy.logerr("%s", exc)
        else:
            print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
