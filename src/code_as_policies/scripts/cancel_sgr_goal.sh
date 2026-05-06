#!/usr/bin/env bash
set -euo pipefail

ARM_NAME="sgr532"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm-name)
      ARM_NAME="${2:?missing value for --arm-name}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--arm-name sgr532]" >&2
      exit 2
      ;;
  esac
done

TOPIC="/${ARM_NAME}/sgr_ctrl/cancel"
echo "[INFO] publishing cancel to ${TOPIC}"
rostopic pub -1 "${TOPIC}" actionlib_msgs/GoalID '{}'
echo "[RESULT] cancel message sent"
