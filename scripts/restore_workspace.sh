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

# Force catkin_tools to treat this repository as its own workspace even when
# it is cloned below a directory that is already a catkin workspace.
mkdir -p "$repo_root/.catkin_tools/profiles"

if [[ ! -e src/CMakeLists.txt ]]; then
  catkin init --workspace "$repo_root"
fi

rosdep install --from-paths src --ignore-src -r -y
catkin config --workspace "$repo_root" --extend /opt/ros/noetic --link-devel
catkin build --workspace "$repo_root" \
  ur_description \
  robotiq_2f_85_gripper_visualization \
  robotiq_2f_gripper_control \
  ur_robot_driver \
  ur3_dual_moveit_config \
  dual_arm_tasks

echo
echo "Restore complete. Run: source $repo_root/devel/setup.bash"
