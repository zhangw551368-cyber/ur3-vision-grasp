#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入工作空间。
cd /home/gzu/gzu_ws
# 退出可能影响 ROS 相机节点的 Conda 环境。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载工作空间中的 RealSense ROS 包。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动 RealSense，并把深度对齐到彩色图且开启同步发布。
exec roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_sync:=true

