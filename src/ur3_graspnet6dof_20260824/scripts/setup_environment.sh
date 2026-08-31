#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$PROJECT_ROOT/.conda_env"
THIRD_PARTY="$PROJECT_ROOT/third_party"

mkdir -p "$THIRD_PARTY" "$PROJECT_ROOT/checkpoints" "$PROJECT_ROOT/runtime"

if [ ! -x "$ENV_PREFIX/bin/python" ]; then
  # Ignore stale user-level channels without mutating ~/.condarc.
  conda create -p "$ENV_PREFIX" python=3.8 -y \
    --override-channels -c defaults
fi

PYTHON="$ENV_PREFIX/bin/python"
PIP="$ENV_PREFIX/bin/pip"

PIP_NO_CACHE_DIR=1 "$PYTHON" -m pip install --upgrade pip setuptools wheel ninja
PIP_NO_CACHE_DIR=1 "$PIP" install \
  torch==2.1.2 --index-url https://download.pytorch.org/whl/cu121
PIP_NO_CACHE_DIR=1 "$PIP" install \
  'numpy<2' 'scipy<1.11' open3d==0.18.0 Pillow tqdm tensorboard \
  pyyaml rospkg catkin_pkg netifaces empy==3.3.4 gdown

if [ ! -d "$THIRD_PARTY/graspnet-baseline/.git" ]; then
  git clone --depth 1 https://github.com/graspnet/graspnet-baseline.git \
    "$THIRD_PARTY/graspnet-baseline"
fi
if [ ! -d "$THIRD_PARTY/graspnetAPI/.git" ]; then
  git clone --depth 1 https://github.com/graspnet/graspnetAPI.git \
    "$THIRD_PARTY/graspnetAPI"
fi

PIP_NO_CACHE_DIR=1 "$PIP" install -e "$THIRD_PARTY/graspnetAPI"
PIP_NO_CACHE_DIR=1 "$PIP" install -r "$THIRD_PARTY/graspnet-baseline/requirements.txt"

# RTX 4090 uses 8.9. Override TORCH_CUDA_ARCH_LIST before running this script
# when the deployment computer has a different CUDA-capable GPU.
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}"
PIP_NO_CACHE_DIR=1 "$PIP" install -v "$THIRD_PARTY/graspnet-baseline/pointnet2"
PIP_NO_CACHE_DIR=1 "$PIP" install -v "$THIRD_PARTY/graspnet-baseline/knn"

"$PYTHON" - <<'PY'
import torch
import open3d
import graspnetAPI
import pointnet2

print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit("CUDA is not available in the isolated environment")
print("gpu:", torch.cuda.get_device_name(0))
print("Environment validation passed")
PY

echo "Environment ready: $ENV_PREFIX"
echo "Checkpoint target: $PROJECT_ROOT/checkpoints/checkpoint-rs.tar"
