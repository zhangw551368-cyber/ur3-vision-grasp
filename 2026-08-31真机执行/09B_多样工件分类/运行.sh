#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入独立 GraspNet 工程。
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
# 加载工程的 ROS、Python 和第三方库路径。
source /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/env.sh
# 启动实时分类器，发布带框图像和冻结目标所需的 JSON。
exec /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/.conda_env/bin/python /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/scripts/classify_pictured_objects.py

