#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$PROJECT_ROOT/.conda_env/bin/python"
TARGET="$PROJECT_ROOT/checkpoints/checkpoint-rs.tar"
EXPECTED_SHA256="60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868"

if [ ! -x "$PYTHON" ]; then
  echo "Run scripts/setup_environment.sh first" >&2
  exit 2
fi
if [ -s "$TARGET" ]; then
  ACTUAL_SHA256="$(sha256sum "$TARGET" | awk '{print $1}')"
  if [ "$ACTUAL_SHA256" = "$EXPECTED_SHA256" ]; then
    echo "Checkpoint already exists and passed SHA-256: $TARGET"
    exit 0
  fi
  echo "Existing checkpoint has the wrong SHA-256; refusing to overwrite it." >&2
  exit 3
fi

"$PYTHON" -m gdown 1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk -O "$TARGET"
test -s "$TARGET"
echo "$EXPECTED_SHA256  $TARGET" | sha256sum --check --status
echo "Downloaded: $TARGET"
