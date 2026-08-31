#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
hardware_file="${UR3_HARDWARE_CONFIG:-$repo_root/config/lab_hardware.env}"

source "$hardware_file"
source /opt/ros/noetic/setup.bash
source "$repo_root/devel/setup.bash"

require_camera="$START_CAMERA"
exec rosrun dual_arm_tasks verify_real_workcell.py _require_camera:="$require_camera" _timeout:=20.0
