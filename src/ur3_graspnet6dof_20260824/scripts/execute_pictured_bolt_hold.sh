#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$PROJECT_ROOT/scripts/execute_pick_hold.sh" \
  --target-pixel 781 262 --target-radius 75 "$@"
