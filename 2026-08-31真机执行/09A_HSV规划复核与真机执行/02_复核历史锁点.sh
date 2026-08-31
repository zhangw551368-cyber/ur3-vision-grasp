#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载锁点复核程序。
source /home/gzu/gzu_ws/devel/setup.bash
# 用最新 30 帧比较三个历史锁点，限制位移 12 mm、MAD 6 mm、关联半径 80 mm。
exec rosrun dual_arm_tasks validate_locked_blue_targets.py --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/blue_blocks_checkerboard_perception.yaml --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml --samples 30 --max-shift 0.012 --max-mad 0.006 --association-radius 0.080

