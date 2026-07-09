#!/usr/bin/env bash
# RSLG-SLAM task63 same-floor Gazebo PID execution wrapper.
#
# A thin variant of tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh that runs
# a task63 (task-local) PID runtime input instead of the hardcoded task56c path.
# It reuses the same TurtleBot3 burger world and the proven PID follower. It runs
# headless gzserver only, never Nav2/AMCL/map_server, never Stage-A/raw RGB-D.
#
# Required env:
#   RSLG_PID_INPUT        absolute/relative path to a task63 *_pid_runtime_input.json
#   RSLG_GAZEBO_OUTPUT_DIR output directory for the run
# Optional env:
#   RSLG_GAZEBO_FLOOR_ID  floor filter (default floor_2)
#   RSLG_QUERY_ID         label for logs (default task63_run)
#   RSLG_PROFILE_ID       PID profile id (default practical_zero_collision)
# Args:
#   --duration-sec N      goal timeout seconds (default 90)
#   --no-gazebo           skip launching gzserver (external odom source)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

DURATION_SEC=90
MODE_NO_GAZEBO=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec) shift; DURATION_SEC="${1:?--duration-sec requires a value}" ;;
    --no-gazebo) MODE_NO_GAZEBO=1 ;;
    *) echo "Unknown argument: $1" >&2; exit 64 ;;
  esac
  shift
done

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    # shellcheck source=/opt/ros/foxy/setup.bash
    source /opt/ros/foxy/setup.bash
  else
    echo "ROS 2 is not sourced and /opt/ros/foxy/setup.bash was not found." >&2
  fi
fi

PID_INPUT="${RSLG_PID_INPUT:?RSLG_PID_INPUT is required}"
OUTPUT_DIR="${RSLG_GAZEBO_OUTPUT_DIR:?RSLG_GAZEBO_OUTPUT_DIR is required}"
FLOOR_ID="${RSLG_GAZEBO_FLOOR_ID:-floor_2}"
QUERY_ID="${RSLG_QUERY_ID:-task63_run}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
FOLLOWER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_pid_follower.py"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

if [[ ! -f "${PID_INPUT}" ]]; then echo "PID input not found: ${PID_INPUT}" >&2; exit 66; fi
if [[ ! -f "${PROFILE_JSON}" ]]; then echo "Profile JSON not found: ${PROFILE_JSON}" >&2; exit 66; fi

TB3_PREFIX=""
if command -v ros2 >/dev/null 2>&1; then
  TB3_PREFIX="$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null || true)"
fi
ROBOT_MODEL="rslg_diff_drive"
WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty.world"
if [[ -n "${TB3_PREFIX}" ]]; then
  ROBOT_MODEL="turtlebot3_burger"
  WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty_turtlebot3_burger.world"
fi

mkdir -p "${OUTPUT_DIR}"
printf 'robot_model=%s\nworld=%s\npid_input=%s\nfloor_id=%s\n' \
  "${ROBOT_MODEL}" "${WORLD}" "${PID_INPUT}" "${FLOOR_ID}" > "${OUTPUT_DIR}/robot_model_strategy.txt"

FOLLOWER_ARGS=(
  --pid-input-json "${PID_INPUT}"
  --profile-json "${PROFILE_JSON}"
  --profile-id "${PROFILE_ID}"
  --output-dir "${OUTPUT_DIR}"
  --frame-id odom
  --goal-timeout-sec "${DURATION_SEC}"
  --rate-hz "${RSLG_GAZEBO_RATE_HZ:-10}"
  --start-delay-sec "${RSLG_GAZEBO_START_DELAY_SEC:-2}"
  --query-id "${QUERY_ID}"
  --floor-id "${FLOOR_ID}"
  --anchor-first-waypoint-to-odom-start
  --stop-at-end
)

GAZEBO_PID=""
cleanup() {
  if [[ -n "${GAZEBO_PID}" ]]; then kill "${GAZEBO_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT

if [[ "${MODE_NO_GAZEBO}" -eq 0 ]]; then
  if ! command -v gzserver >/dev/null 2>&1; then
    echo "gzserver is not available; use --no-gazebo with an external odom source." >&2
    exit 69
  fi
  export GAZEBO_MODEL_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/models:${GAZEBO_MODEL_PATH:-}"
  if [[ -n "${TB3_PREFIX}" ]]; then
    export GAZEBO_MODEL_PATH="${TB3_PREFIX}/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"
  fi
  export GAZEBO_RESOURCE_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds:${GAZEBO_RESOURCE_PATH:-}"
  gzserver --verbose "${WORLD}" > "${OUTPUT_DIR}/gzserver.log" 2>&1 &
  GAZEBO_PID=$!
  sleep "${RSLG_GAZEBO_BOOT_WAIT_SEC:-6}"
fi

"${ROS_PYTHON}" "${FOLLOWER}" "${FOLLOWER_ARGS[@]}"
