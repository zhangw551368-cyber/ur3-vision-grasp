#!/usr/bin/env bash

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$PROJECT_ROOT/../.." && pwd)"
ENV_PREFIX="$PROJECT_ROOT/.conda_env"

source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE_ROOT/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE_ROOT/devel/setup.bash; run scripts/restore_workspace.sh from the repository first." >&2
  return 2 2>/dev/null || exit 2
fi
source "$WORKSPACE_ROOT/devel/setup.bash"

export UR3_GRASPNET6D_ROOT="$PROJECT_ROOT"
export ROS_PACKAGE_PATH="$PROJECT_ROOT:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="$PROJECT_ROOT/python:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
export PATH="$ENV_PREFIX/bin:$PATH"

echo "UR3 GraspNet project: $PROJECT_ROOT"
echo "Python: $(command -v python3)"
