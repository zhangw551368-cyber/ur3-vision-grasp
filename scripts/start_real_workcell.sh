#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
hardware_file="${UR3_HARDWARE_CONFIG:-$repo_root/config/lab_hardware.env}"

if [[ ! -f "$repo_root/devel/setup.bash" ]]; then
  echo "ERROR: workspace is not built. Run: bash scripts/restore_workspace.sh" >&2
  exit 2
fi
if [[ ! -f "$hardware_file" ]]; then
  echo "ERROR: hardware config not found: $hardware_file" >&2
  exit 3
fi

source "$hardware_file"

if [[ "${SKIP_HARDWARE_CHECK:-false}" != "true" ]]; then
  UR3_HARDWARE_CONFIG="$hardware_file" bash "$repo_root/scripts/check_hardware_connections.sh"
fi

source /opt/ros/noetic/setup.bash
source "$repo_root/devel/setup.bash"

exec roslaunch dual_arm_tasks right_arm_real_workcell.launch \
  left_robot_ip:="$LEFT_ARM_IP" \
  right_robot_ip:="$RIGHT_ARM_IP" \
  right_gripper_device:="$RIGHT_GRIPPER_DEVICE" \
  start_left_arm:="$START_LEFT_ARM" \
  start_right_arm:="$START_RIGHT_ARM" \
  start_camera:="$START_CAMERA" \
  start_handeye:="$START_HAND_EYE" \
  start_moveit:="$START_MOVEIT" \
  allow_trajectory_execution:="$ALLOW_TRAJECTORY_EXECUTION" \
  start_collision_scene:="$START_COLLISION_SCENE" \
  start_right_gripper:="$START_RIGHT_GRIPPER" \
  activate_right_gripper:="$ACTIVATE_RIGHT_GRIPPER" \
  start_gripper_visualization:="$START_GRIPPER_VISUALIZATION" \
  start_rviz:="$START_RVIZ" \
  enable_pointcloud:="$ENABLE_POINTCLOUD" \
  headless_mode:="$HEADLESS_MODE" \
  moveit_controller_manager:="$MOVEIT_CONTROLLER_MANAGER"
