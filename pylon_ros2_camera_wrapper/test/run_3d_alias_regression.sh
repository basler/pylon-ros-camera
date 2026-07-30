#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

CAMERA_ID="${CAMERA_ID:-my_camera}"
NODE_NAME="${NODE_NAME:-pylon_ros2_camera_node}"
CONFIG_FILE="${CONFIG_FILE:-}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-30}"

LEGACY_ACTION="/${CAMERA_ID}/${NODE_NAME}/grab_blaze_data"
GENERIC_3D_ACTION="/${CAMERA_ID}/${NODE_NAME}/grab_3d_data"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "Missing ${REPO_ROOT}/install/setup.bash. Build the workspace first:"
  echo "  colcon build --packages-select pylon_ros2_camera_component pylon_ros2_camera_wrapper"
  exit 1
fi

# shellcheck source=/dev/null
source "${REPO_ROOT}/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command is not available in the current environment"
  exit 1
fi

launch_cmd=(
  ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
  "camera_profile:=3d"
  "camera_id:=${CAMERA_ID}"
  "node_name:=${NODE_NAME}"
)

if [[ -n "${CONFIG_FILE}" ]]; then
  launch_cmd+=("config_file:=${CONFIG_FILE}")
fi

echo "Starting camera launch for 3D profile..."
"${launch_cmd[@]}" >/tmp/pylon_3d_alias_regression.launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_action() {
  local action_name="$1"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ros2 action list | grep -Fxq "${action_name}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo "Waiting for legacy action endpoint: ${LEGACY_ACTION}"
if ! wait_for_action "${LEGACY_ACTION}"; then
  echo "Timed out waiting for ${LEGACY_ACTION}"
  echo "Launch log: /tmp/pylon_3d_alias_regression.launch.log"
  exit 1
fi

echo "Waiting for generic 3D action endpoint: ${GENERIC_3D_ACTION}"
if ! wait_for_action "${GENERIC_3D_ACTION}"; then
  echo "Timed out waiting for ${GENERIC_3D_ACTION}"
  echo "Launch log: /tmp/pylon_3d_alias_regression.launch.log"
  exit 1
fi

echo "Running legacy endpoint action client..."
ros2 run pylon_ros2_camera_wrapper test_grab_blaze_data_action_client

echo "Running generic 3D endpoint action client..."
ros2 run pylon_ros2_camera_wrapper test_grab_3d_data_action_client

echo "3D alias compatibility regression passed"