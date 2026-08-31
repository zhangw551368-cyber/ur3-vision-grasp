#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入工作空间。
cd /home/gzu/gzu_ws
# 退出 Conda 环境并加载 ROS 工作空间。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载 dual_arm_tasks。
source /home/gzu/gzu_ws/devel/setup.bash
# 使用历史成功配置和锁点完成全流程规划；没有 --execute，所以不会真机运动。
exec rosrun dual_arm_tasks right_arm_three_blue_pick_place.py --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml

