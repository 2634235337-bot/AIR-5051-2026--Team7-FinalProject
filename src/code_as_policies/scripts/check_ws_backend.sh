#!/usr/bin/env bash
set -euo pipefail

ARM_NAME="sgr532"
CAMERA_TOPIC="/usb_cam/image_raw"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm-name)
      ARM_NAME="${2:?missing value for --arm-name}"
      shift 2
      ;;
    --camera-topic)
      CAMERA_TOPIC="${2:?missing value for --camera-topic}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--arm-name sgr532] [--camera-topic /usb_cam/image_raw]" >&2
      exit 2
      ;;
  esac
done

if ! command -v rostopic >/dev/null 2>&1; then
  echo "[FAIL] rostopic not found. Please source ROS first." >&2
  exit 1
fi
if ! command -v rosservice >/dev/null 2>&1; then
  echo "[FAIL] rosservice not found. Please source ROS first." >&2
  exit 1
fi

echo "[INFO] arm_name=${ARM_NAME}"
echo "[INFO] camera_topic=${CAMERA_TOPIC}"

TOPICS="$(rostopic list || true)"
SERVICES="$(rosservice list || true)"

check_contains() {
  local haystack="$1"
  local needle="$2"
  local label="$3"
  if printf '%s\n' "$haystack" | grep -q "$needle"; then
    echo "[PASS] ${label}: ${needle}"
  else
    echo "[FAIL] ${label}: ${needle}"
    return 1
  fi
}

FAIL=0

check_contains "$TOPICS" "/${ARM_NAME}/sgr_ctrl" "action namespace" || FAIL=1
check_contains "$TOPICS" "sagittarius_arm_controller/follow_joint_trajectory" "arm trajectory action" || FAIL=1
check_contains "$TOPICS" "sagittarius_gripper_controller/follow_joint_trajectory" "gripper trajectory action" || FAIL=1
check_contains "$TOPICS" "sagittarius_joint_states" "joint state topic" || FAIL=1
check_contains "$SERVICES" "/${ARM_NAME}/get_servo_info" "servo info service" || FAIL=1
check_contains "$SERVICES" "/${ARM_NAME}/get_robot_info" "robot info service" || FAIL=1

echo "[INFO] checking one camera header message from ${CAMERA_TOPIC}/header ..."
if command -v timeout >/dev/null 2>&1; then
  if timeout 8 rostopic echo -n1 "${CAMERA_TOPIC}/header" >/dev/null 2>&1; then
    echo "[PASS] camera stream is alive"
  else
    echo "[FAIL] camera stream timeout on ${CAMERA_TOPIC}"
    FAIL=1
  fi
else
  if rostopic echo -n1 "${CAMERA_TOPIC}/header" >/dev/null 2>&1; then
    echo "[PASS] camera stream is alive"
  else
    echo "[FAIL] camera stream unavailable on ${CAMERA_TOPIC}"
    FAIL=1
  fi
fi

if [[ "$FAIL" -ne 0 ]]; then
  echo "[RESULT] backend check failed"
  exit 1
fi

echo "[RESULT] backend check passed"
