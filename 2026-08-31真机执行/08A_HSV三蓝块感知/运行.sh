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
# 加载三蓝块感知节点。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动 HSV、对齐深度和棋盘感知，并显示调试窗口。
exec roslaunch dual_arm_tasks blue_blocks_checkerboard_perception.launch show_window:=true

