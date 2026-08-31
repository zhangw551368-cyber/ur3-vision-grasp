#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入工作空间。
cd /home/gzu/gzu_ws
# 退出可能冲突的 Conda 环境。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic 环境。
source /opt/ros/noetic/setup.bash
# 加载工作空间内的 MoveIt 配置。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动双臂 MoveIt 的已保存 RViz 配置。
exec roslaunch ur3_dual_moveit_config moveit_rviz.launch rviz_config:=/home/gzu/gzu_ws/src/ur3_dual_moveit_config/launch/moveit.rviz

