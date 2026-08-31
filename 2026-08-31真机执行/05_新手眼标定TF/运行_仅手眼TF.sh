#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入工作空间。
cd /home/gzu/gzu_ws
# 退出可能冲突的 Conda 环境。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载工作空间环境。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动 2026-08-17 标定数值对应的运行安全版静态 TF。
exec roslaunch /home/gzu/gzu_ws/calibration/handeye/right_arm_camera_eye_on_base_20260817_safe.launch

