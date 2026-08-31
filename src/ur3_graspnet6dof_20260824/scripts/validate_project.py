#!/usr/bin/python3

import argparse
import hashlib
import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from ur3_graspnet6dof.config import load_config, resolve_project_path


def parse_args():
    parser = argparse.ArgumentParser(description="Validate isolated UR3 GraspNet project")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "right_arm_green_table.yaml"),
    )
    parser.add_argument("--live", action="store_true", help="Also validate ROS topics and TF")
    return parser.parse_args()


def validate_static(config):
    errors = []
    warnings = []
    if config["project"].get("schema_version") != 1:
        errors.append("unsupported project schema")
    if Path(config["_project_root"]) != PROJECT_ROOT:
        errors.append("config is not inside this isolated project")
    for key in ("graspnet_baseline", "graspnet_api"):
        path = resolve_project_path(config, config["paths"][key])
        if not path.is_dir():
            errors.append("missing {}: {}".format(key, path))
    checkpoint = resolve_project_path(config, config["paths"]["checkpoint"])
    if not checkpoint.is_file() or checkpoint.stat().st_size < 1024 * 1024:
        errors.append("missing or implausibly small checkpoint: {}".format(checkpoint))
    elif config["paths"].get("checkpoint_sha256"):
        digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        if digest != config["paths"]["checkpoint_sha256"]:
            errors.append("checkpoint SHA-256 does not match the configured value")
    env_python = PROJECT_ROOT / ".conda_env" / "bin" / "python"
    if not env_python.is_file():
        errors.append("isolated Python environment is missing")
    else:
        check = subprocess.run(
            [
                str(env_python),
                "-c",
                (
                    "import torch, open3d, graspnetAPI, pointnet2; "
                    "assert torch.cuda.is_available(); "
                    "print(torch.__version__, torch.cuda.get_device_name(0))"
                ),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if check.returncode:
            errors.append("isolated inference imports/CUDA failed: {}".format(check.stderr.strip()))
        else:
            print("ENV OK", check.stdout.strip())
    if config["execution"].get("enabled", False):
        warnings.append("REAL EXECUTION IS ENABLED in the isolated config")
    if config["execution"].get("drop_enabled", False):
        position = config["execution"]["drop_pose"]["position"]
        dynamic_board_pose = config["execution"].get(
            "drop_pose_dynamic_from_board", False
        )
        if all(abs(float(value)) < 1e-9 for value in position) and not dynamic_board_pose:
            errors.append("drop is enabled but the drop position is still zero")
        if dynamic_board_pose and "sequence" not in config:
            errors.append("dynamic board drop is enabled but sequence config is absent")
    return errors, warnings


def validate_live(config):
    import rospy
    import tf2_ros
    from sensor_msgs.msg import CameraInfo, Image

    rospy.init_node("ur3_graspnet6d_project_validator", anonymous=True)
    errors = []
    camera = config["camera"]
    topics = [
        (camera["color_topic"], Image),
        (camera["depth_topic"], Image),
        (camera["camera_info_topic"], CameraInfo),
    ]
    messages = []
    for topic, message_type in topics:
        try:
            message = rospy.wait_for_message(topic, message_type, timeout=3.0)
            messages.append(message)
            print("LIVE OK topic {} frame={}".format(topic, message.header.frame_id))
        except Exception as exc:
            errors.append("{}: {}".format(topic, exc))
    if len(messages) == 3:
        frames = [message.header.frame_id for message in messages]
        if len(set(frames)) != 1:
            errors.append("RGB/depth/info frames differ: {}".format(frames))
        expected = camera.get("expected_frame")
        if expected and any(frame != expected for frame in frames):
            errors.append("live camera frame {} != expected {}".format(frames, expected))
        if messages[0].width != messages[1].width or messages[0].height != messages[1].height:
            errors.append("RGB/depth image dimensions differ")

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        rospy.sleep(0.5)
        try:
            transform = buffer.lookup_transform(
                config["selector"]["planning_frame"],
                frames[0],
                rospy.Time(0),
                rospy.Duration(float(config["selector"]["tf_timeout"])),
            )
            print(
                "LIVE OK TF {} <- {} translation=({:.4f},{:.4f},{:.4f})".format(
                    config["selector"]["planning_frame"],
                    frames[0],
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                )
            )
        except Exception as exc:
            errors.append("camera TF: {}".format(exc))
    return errors


def main():
    args = parse_args()
    config = load_config(args.config)
    errors, warnings = validate_static(config)
    for warning in warnings:
        print("WARNING:", warning)
    if args.live:
        errors.extend(validate_live(config))
    if errors:
        for error in errors:
            print("ERROR:", error)
        return 1
    print("Project validation passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
