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
# 加载真实环境发布器所在的工作空间。
source /home/gzu/gzu_ws/devel/setup.bash
# 向 MoveIt 发布实测机柜和绿布桌面的碰撞模型。
exec roslaunch dual_arm_tasks right_arm_real_environment.launch

