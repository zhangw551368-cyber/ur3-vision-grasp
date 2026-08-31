#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入独立 GraspNet 工程。
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
# 调用工程自带入口；它会加载 ROS、独立 Conda/CUDA 环境和固定配置。
exec ./scripts/start_inference.sh

