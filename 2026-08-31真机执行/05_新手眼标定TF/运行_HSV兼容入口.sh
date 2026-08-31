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
# 加载 dual_arm_tasks 包。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动同一个 safe 手眼 TF，并附带 HSV 相机点到右臂基座点的转换节点。
exec roslaunch dual_arm_tasks right_arm_hsv_eye_on_base_runtime.launch

