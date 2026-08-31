#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
hardware_file="${UR3_HARDWARE_CONFIG:-$repo_root/config/lab_hardware.env}"

if [[ ! -f "$hardware_file" ]]; then
  echo "ERROR: hardware config not found: $hardware_file" >&2
  exit 2
fi

source "$hardware_file"

failures=0
pc_address="${PC_IPV4_CIDR%/*}"
if ! ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "$pc_address"; then
  echo "ERROR: this computer does not own $PC_IPV4_CIDR" >&2
  failures=$((failures + 1))
else
  echo "OK: computer network address $PC_IPV4_CIDR"
fi

check_robot() {
  local label="$1"
  local address="$2"
  if ping -c 1 -W 2 "$address" >/dev/null 2>&1; then
    echo "OK: $label responds at $address"
  else
    echo "ERROR: $label does not respond at $address" >&2
    failures=$((failures + 1))
  fi
}

if [[ "${START_LEFT_ARM:-true}" == "true" ]]; then
  check_robot "left UR3" "$LEFT_ARM_IP"
fi
if [[ "${START_RIGHT_ARM:-true}" == "true" ]]; then
  check_robot "right UR3" "$RIGHT_ARM_IP"
fi

if [[ "${START_RIGHT_GRIPPER:-true}" == "true" ]]; then
  if [[ -e "$RIGHT_GRIPPER_DEVICE" ]]; then
    echo "OK: right gripper adapter $RIGHT_GRIPPER_DEVICE"
  else
    echo "ERROR: right gripper adapter is missing: $RIGHT_GRIPPER_DEVICE" >&2
    failures=$((failures + 1))
  fi
fi

if [[ "${START_CAMERA:-true}" == "true" ]]; then
  if command -v rs-enumerate-devices >/dev/null 2>&1 && rs-enumerate-devices 2>/dev/null | grep -q 'Device info'; then
    echo "OK: RealSense device detected"
  elif compgen -G '/dev/video*' >/dev/null; then
    echo "WARN: video device exists, but RealSense identity could not be confirmed"
  else
    echo "ERROR: no RealSense/video device detected" >&2
    failures=$((failures + 1))
  fi
fi

if (( failures > 0 )); then
  echo "Hardware preflight failed with $failures problem(s)." >&2
  exit 1
fi

echo "Hardware preflight passed. No robot command was sent."
