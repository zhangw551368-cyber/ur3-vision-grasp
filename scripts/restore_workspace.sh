#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ERROR: ROS Noetic is not installed. See dependencies/README.md." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash

if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep is missing. Install python3-rosdep." >&2
  exit 1
fi

if ! command -v catkin >/dev/null 2>&1; then
  echo "ERROR: catkin_tools is missing. Install python3-catkin-tools." >&2
  exit 1
fi

cd "$repo_root"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "rosdep has not been initialized; requesting sudo once for rosdep init."
  sudo rosdep init
fi
if ! rosdep update; then
  echo "WARN: rosdep index update failed; continuing with the explicitly documented Noetic packages." >&2
fi

# ROS Noetic is end-of-life and current rosdistro indexes can omit Noetic-only
# keys such as moveit_commander, realsense2_camera, aruco_ros and easy_handeye.
# Install every still-resolvable dependency, then let the focused build below
# report any genuinely missing package. dependencies/README.md lists the
# required Noetic apt baseline explicitly.
if ! rosdep install --from-paths src --ignore-src -r -y; then
  echo "WARN: rosdep could not resolve one or more EOL Noetic keys." >&2
  echo "WARN: continuing; see dependencies/README.md if the build reports a missing package." >&2
fi
catkin config --workspace "$repo_root" --init --extend /opt/ros/noetic --link-devel
catkin build --workspace "$repo_root" \
  ur_description \
  robotiq_2f_85_gripper_visualization \
  robotiq_2f_gripper_control \
  ur_robot_driver \
  ur3_dual_moveit_config \
  dual_arm_tasks \
  ur3_graspnet6dof_20260824

echo
echo "Restore complete. Run: source $repo_root/devel/setup.bash"
