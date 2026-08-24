#!/usr/bin/env python3
import argparse
import math
import os
import re
import sys
import time

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def matrix_from_tq(translation, quaternion_xyzw):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def tq_from_matrix(transform):
    return (
        transform[:3, 3].tolist(),
        Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
    )


def matrix_from_vector(vector):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    transform[:3, 3] = vector[3:6]
    return transform


def vector_from_matrix(transform):
    vector = np.zeros(6)
    vector[:3] = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    vector[3:6] = transform[:3, 3]
    return vector


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3].dot(transform[:3, 3])
    return result


def se3_residual(transform, rotation_weight, translation_weight):
    residual = np.zeros(6)
    residual[:3] = rotation_weight * Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    residual[3:6] = translation_weight * transform[:3, 3]
    return residual


def sample_to_matrix(sample, key):
    transform = sample[key]
    return matrix_from_tq(transform["translation"], transform["quaternion_xyzw"])


def load_samples(path):
    with open(os.path.expanduser(path), "r") as stream:
        data = yaml.safe_load(stream)
    samples = data.get("samples", [])
    if len(samples) < 3:
        raise ValueError("At least 3 samples are required; got {}".format(len(samples)))
    return data, samples


def load_base_to_camera_from_static_launch(path):
    with open(os.path.expanduser(path), "r") as stream:
        text = stream.read()
    match = re.search(r'args="([^"]+)"', text)
    if not match:
        raise ValueError("No static_transform_publisher args found in {}".format(path))
    tokens = match.group(1).split()
    if len(tokens) < 9:
        raise ValueError("Expected x y z qx qy qz qw parent child in static args")
    translation = [float(value) for value in tokens[0:3]]
    quaternion = [float(value) for value in tokens[3:7]]
    parent = tokens[7]
    child = tokens[8]
    return matrix_from_tq(translation, quaternion), parent, child


def save_yaml(path, data):
    path = os.path.expanduser(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)


def transform_error(base_to_camera, tool_to_target, base_to_tool, camera_to_target):
    # Eye-on-base with target rigidly attached to tool:
    # base_T_tool * tool_T_target = base_T_camera * camera_T_target
    return inverse(base_to_camera).dot(base_to_tool).dot(tool_to_target).dot(
        inverse(camera_to_target)
    )


def initial_guess(samples):
    candidates = []
    for sample in samples:
        base_to_tool = sample_to_matrix(sample, "base_to_tool")
        camera_to_target = sample_to_matrix(sample, "camera_to_target")
        candidates.append(base_to_tool.dot(inverse(camera_to_target)))

    translations = np.asarray([candidate[:3, 3] for candidate in candidates])
    quats = np.asarray([Rotation.from_matrix(candidate[:3, :3]).as_quat() for candidate in candidates])
    reference = quats[0]
    for index in range(len(quats)):
        if np.dot(reference, quats[index]) < 0.0:
            quats[index] *= -1.0
    quat = np.mean(quats, axis=0)
    quat /= np.linalg.norm(quat)

    base_to_camera = np.eye(4)
    base_to_camera[:3, :3] = Rotation.from_quat(quat).as_matrix()
    base_to_camera[:3, 3] = np.mean(translations, axis=0)
    tool_to_target = np.eye(4)
    return base_to_camera, tool_to_target


def initial_guess_from_base_to_camera(samples, base_to_camera):
    tool_to_target_estimates = []
    for sample in samples:
        base_to_tool = sample_to_matrix(sample, "base_to_tool")
        camera_to_target = sample_to_matrix(sample, "camera_to_target")
        tool_to_target_estimates.append(
            inverse(base_to_tool).dot(base_to_camera).dot(camera_to_target)
        )
    return base_to_camera, average_transforms(tool_to_target_estimates)


def optimize(
    samples,
    rotation_weight=1.0,
    translation_weight=100.0,
    base_to_camera_initial=None,
):
    if base_to_camera_initial is None:
        base_to_camera0, tool_to_target0 = initial_guess(samples)
    else:
        base_to_camera0, tool_to_target0 = initial_guess_from_base_to_camera(
            samples, base_to_camera_initial
        )
    x0 = np.concatenate([vector_from_matrix(base_to_camera0), vector_from_matrix(tool_to_target0)])

    def residuals(vector):
        base_to_camera = matrix_from_vector(vector[:6])
        tool_to_target = matrix_from_vector(vector[6:12])
        residual = []
        for sample in samples:
            base_to_tool = sample_to_matrix(sample, "base_to_tool")
            camera_to_target = sample_to_matrix(sample, "camera_to_target")
            error = transform_error(
                base_to_camera, tool_to_target, base_to_tool, camera_to_target
            )
            residual.extend(se3_residual(error, rotation_weight, translation_weight))
        return np.asarray(residual)

    result = least_squares(residuals, x0, loss="soft_l1", f_scale=1.0, max_nfev=2000)
    base_to_camera = matrix_from_vector(result.x[:6])
    tool_to_target = matrix_from_vector(result.x[6:12])
    raw_residuals = residuals(result.x).reshape((-1, 6))
    rotation_errors = np.linalg.norm(raw_residuals[:, :3], axis=1) / rotation_weight
    translation_errors = np.linalg.norm(raw_residuals[:, 3:6], axis=1) / translation_weight
    return result, base_to_camera, tool_to_target, rotation_errors, translation_errors


def average_transforms(transforms):
    translations = np.asarray([transform[:3, 3] for transform in transforms])
    quats = np.asarray([Rotation.from_matrix(transform[:3, :3]).as_quat() for transform in transforms])
    reference = quats[0]
    for index in range(len(quats)):
        if np.dot(reference, quats[index]) < 0.0:
            quats[index] *= -1.0
    quat = np.mean(quats, axis=0)
    quat /= np.linalg.norm(quat)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quat).as_matrix()
    result[:3, 3] = np.mean(translations, axis=0)
    return result


def evaluate_fixed_base_to_camera(samples, base_to_camera):
    tool_to_target_estimates = []
    for sample in samples:
        base_to_tool = sample_to_matrix(sample, "base_to_tool")
        camera_to_target = sample_to_matrix(sample, "camera_to_target")
        tool_to_target_estimates.append(
            inverse(base_to_tool).dot(base_to_camera).dot(camera_to_target)
        )

    tool_to_target = average_transforms(tool_to_target_estimates)
    rotation_errors = []
    translation_errors = []
    for sample in samples:
        base_to_tool = sample_to_matrix(sample, "base_to_tool")
        camera_to_target = sample_to_matrix(sample, "camera_to_target")
        error = transform_error(base_to_camera, tool_to_target, base_to_tool, camera_to_target)
        rotation_errors.append(np.linalg.norm(Rotation.from_matrix(error[:3, :3]).as_rotvec()))
        translation_errors.append(np.linalg.norm(error[:3, 3]))
    return tool_to_target, np.asarray(rotation_errors), np.asarray(translation_errors)


def output_data(args, input_data, result, base_to_camera, tool_to_target, rot_errors, trans_errors):
    base_t, base_q = tq_from_matrix(base_to_camera)
    tool_t, tool_q = tq_from_matrix(tool_to_target)
    return {
        "frames": input_data.get("frames", {}),
        "method": "eye_on_base_se3_lie_least_squares",
        "cost": float(result.cost),
        "success": bool(result.success),
        "message": result.message,
        "sample_count": int(len(input_data.get("samples", []))),
        "base_to_camera": {
            "translation": base_t,
            "quaternion_xyzw": base_q,
        },
        "tool_to_target": {
            "translation": tool_t,
            "quaternion_xyzw": tool_q,
        },
        "error_summary": {
            "rotation_rad_mean": float(np.mean(rot_errors)),
            "rotation_rad_max": float(np.max(rot_errors)),
            "rotation_deg_mean": float(np.degrees(np.mean(rot_errors))),
            "rotation_deg_max": float(np.degrees(np.max(rot_errors))),
            "translation_m_mean": float(np.mean(trans_errors)),
            "translation_m_max": float(np.max(trans_errors)),
        },
        "weights": {
            "rotation_weight": float(args.rotation_weight),
            "translation_weight": float(args.translation_weight),
        },
    }


def collect_samples(args):
    import rospy
    import tf2_ros

    rospy.init_node("handeye_eye_on_base_lie_sample_collector")
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
    listener = tf2_ros.TransformListener(buffer)
    del listener

    path = os.path.expanduser(args.samples_file)
    if os.path.exists(path):
        with open(path, "r") as stream:
            data = yaml.safe_load(stream) or {}
    else:
        data = {}

    data.setdefault(
        "frames",
        {
            "base_frame": args.base_frame,
            "tool_frame": args.tool_frame,
            "camera_frame": args.camera_frame,
            "target_frame": args.target_frame,
        },
    )
    data.setdefault("samples", [])

    print("Collecting eye-on-base samples.")
    print("Move the robot, wait until ChArUco detection is stable, then press ENTER.")
    print("Type q then ENTER to finish.")
    while not rospy.is_shutdown():
        text = input("sample {} > ".format(len(data["samples"]) + 1)).strip().lower()
        if text in ("q", "quit", "exit"):
            break
        try:
            now = rospy.Time(0)
            base_to_tool = buffer.lookup_transform(
                args.base_frame, args.tool_frame, now, rospy.Duration(args.timeout)
            )
            camera_to_target = buffer.lookup_transform(
                args.camera_frame, args.target_frame, now, rospy.Duration(args.timeout)
            )
        except Exception as exc:
            print("TF lookup failed: {}".format(exc))
            continue

        def transform_to_dict(msg):
            t = msg.transform.translation
            q = msg.transform.rotation
            return {
                "translation": [float(t.x), float(t.y), float(t.z)],
                "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            }

        data["samples"].append(
            {
                "stamp": time.time(),
                "base_to_tool": transform_to_dict(base_to_tool),
                "camera_to_target": transform_to_dict(camera_to_target),
            }
        )
        save_yaml(path, data)
        print("Saved sample {} to {}".format(len(data["samples"]), path))


def optimize_samples(args):
    input_data, samples = load_samples(args.samples_file)
    base_to_camera_initial = None
    if args.base_to_camera_launch:
        try:
            base_to_camera_initial, parent, child = load_base_to_camera_from_static_launch(
                args.base_to_camera_launch
            )
            print(
                "Using initial base_to_camera from {} ({} -> {}).".format(
                    os.path.expanduser(args.base_to_camera_launch), parent, child
                )
            )
        except Exception as exc:
            print("Could not load initial base_to_camera: {}".format(exc))
    result, base_to_camera, tool_to_target, rot_errors, trans_errors = optimize(
        samples,
        rotation_weight=args.rotation_weight,
        translation_weight=args.translation_weight,
        base_to_camera_initial=base_to_camera_initial,
    )
    data = output_data(args, input_data, result, base_to_camera, tool_to_target, rot_errors, trans_errors)
    save_yaml(args.output_file, data)
    print("Wrote optimized result:", os.path.expanduser(args.output_file))
    print("base_to_camera translation:", data["base_to_camera"]["translation"])
    print("base_to_camera quaternion xyzw:", data["base_to_camera"]["quaternion_xyzw"])
    print("mean translation error m:", data["error_summary"]["translation_m_mean"])
    print("mean rotation error deg:", data["error_summary"]["rotation_deg_mean"])


def evaluate_samples(args):
    input_data, samples = load_samples(args.samples_file)
    base_to_camera, parent, child = load_base_to_camera_from_static_launch(
        args.base_to_camera_launch
    )
    tool_to_target, rot_errors, trans_errors = evaluate_fixed_base_to_camera(
        samples, base_to_camera
    )
    base_t, base_q = tq_from_matrix(base_to_camera)
    tool_t, tool_q = tq_from_matrix(tool_to_target)
    data = {
        "frames": input_data.get("frames", {}),
        "method": "evaluate_fixed_base_to_camera",
        "base_to_camera_launch": os.path.expanduser(args.base_to_camera_launch),
        "static_transform_parent": parent,
        "static_transform_child": child,
        "sample_count": int(len(samples)),
        "base_to_camera": {
            "translation": base_t,
            "quaternion_xyzw": base_q,
        },
        "estimated_tool_to_target": {
            "translation": tool_t,
            "quaternion_xyzw": tool_q,
        },
        "error_summary": {
            "rotation_rad_mean": float(np.mean(rot_errors)),
            "rotation_rad_max": float(np.max(rot_errors)),
            "rotation_deg_mean": float(np.degrees(np.mean(rot_errors))),
            "rotation_deg_max": float(np.degrees(np.max(rot_errors))),
            "translation_m_mean": float(np.mean(trans_errors)),
            "translation_m_max": float(np.max(trans_errors)),
        },
    }
    save_yaml(args.output_file, data)
    print("Wrote evaluation result:", os.path.expanduser(args.output_file))
    print("samples:", data["sample_count"])
    print("mean translation error m:", data["error_summary"]["translation_m_mean"])
    print("max translation error m:", data["error_summary"]["translation_m_max"])
    print("mean rotation error deg:", data["error_summary"]["rotation_deg_mean"])
    print("max rotation error deg:", data["error_summary"]["rotation_deg_max"])


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Collect and optimize eye-on-base hand-eye samples on SE(3)."
    )
    parser.add_argument("--collect", action="store_true", help="collect TF samples interactively")
    parser.add_argument("--optimize", action="store_true", help="optimize an existing samples YAML")
    parser.add_argument("--evaluate", action="store_true", help="evaluate a fixed base->camera launch against samples")
    parser.add_argument(
        "--samples-file",
        default="/home/gzu/gzu_ws/calibration/handeye/camera2_eye_on_base_charuco_samples.yaml",
    )
    parser.add_argument(
        "--output-file",
        default="/home/gzu/gzu_ws/calibration/handeye/camera2_eye_on_base_charuco_lie_optimized.yaml",
    )
    parser.add_argument(
        "--base-to-camera-launch",
        default="/home/gzu/.ros/easy_handeye/riht_arm_eyeonbase_611.launch",
    )
    parser.add_argument("--base-frame", default="right_arm_base")
    parser.add_argument("--tool-frame", default="right_arm_tool0")
    parser.add_argument("--camera-frame", default="camera2_color_optical_frame")
    parser.add_argument("--target-frame", default="handeye_target")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--rotation-weight", type=float, default=1.0)
    parser.add_argument("--translation-weight", type=float, default=100.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if args.collect:
        collect_samples(args)
    if args.optimize:
        optimize_samples(args)
    if args.evaluate:
        evaluate_samples(args)
    if not args.collect and not args.optimize and not args.evaluate:
        raise SystemExit("Use --collect, --optimize, --evaluate, or a combination.")


if __name__ == "__main__":
    main()
