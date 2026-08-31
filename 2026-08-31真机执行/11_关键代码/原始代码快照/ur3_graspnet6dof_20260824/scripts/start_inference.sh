#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/env.sh"

CONFIG="${UR3_GRASPNET6D_CONFIG:-$PROJECT_ROOT/config/right_arm_green_table.yaml}"
exec "$PROJECT_ROOT/.conda_env/bin/python" \
  "$PROJECT_ROOT/scripts/graspnet_inference_node.py" --config "$CONFIG" "$@"
