#!/usr/bin/env bash
# 上一行指定使用 Bash 解释脚本。
# 开启严格错误检查，避免环境加载失败后继续。
set -euo pipefail
# 进入 ROS 工作空间。
cd /home/gzu/gzu_ws
# 有 Conda 时退出当前 Conda 环境，保持 ROS Noetic 的系统 Python 兼容性。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载当前 catkin 工作空间。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动 MoveIt，并明确允许轨迹执行、只选择右臂的简单控制器管理器。
exec roslaunch ur3_dual_moveit_config move_group.launch allow_trajectory_execution:=true moveit_controller_manager:=simple_right

