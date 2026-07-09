#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

MODE_DRY_RUN=0
MODE_HEADLESS=0
MODE_WITH_RVIZ=0
MODE_NO_GAZEBO=0
DURATION_SEC=60

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      MODE_DRY_RUN=1
      ;;
    --headless)
      MODE_HEADLESS=1
      ;;
    --with-rviz)
      MODE_WITH_RVIZ=1
      ;;
    --no-gazebo)
      MODE_NO_GAZEBO=1
      ;;
    --duration-sec)
      shift
      DURATION_SEC="${1:?--duration-sec requires a value}"
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 64
      ;;
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

QUERY_ID="${RSLG_QUERY_ID:-00843_object_in_room_curtain_room14}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
FLOOR_ID="${RSLG_GAZEBO_FLOOR_ID:-floor_2}"
ROBOT_MODEL_REQUEST="${RSLG_GAZEBO_ROBOT_MODEL:-auto}"
TASK58A_DIR="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task58a_gazebo_pid_same_floor_simulation_scaffold"
TASK56C_ROOT="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56c_pid_profile_promotion_and_regression_validation/regression_pack"
PID_INPUT="${TASK56C_ROOT}/runtime_adapter_inputs/pid_follower_inputs/${QUERY_ID}_pid_runtime_input.json"
PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
FOLLOWER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_pid_follower.py"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${RSLG_GAZEBO_OUTPUT_DIR:-${TASK58A_DIR}/gazebo_sim_pack/runs/${QUERY_ID}_${RUN_ID}}"

TB3_PREFIX=""
if command -v ros2 >/dev/null 2>&1; then
  TB3_PREFIX="$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null || true)"
fi

ROBOT_MODEL="rslg_diff_drive"
WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty.world"
if [[ "${ROBOT_MODEL_REQUEST}" == "turtlebot3_burger" ]]; then
  if [[ -z "${TB3_PREFIX}" ]]; then
    echo "RSLG_GAZEBO_ROBOT_MODEL=turtlebot3_burger requested, but turtlebot3_gazebo was not found." >&2
    exit 69
  fi
  ROBOT_MODEL="turtlebot3_burger"
  WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty_turtlebot3_burger.world"
elif [[ "${ROBOT_MODEL_REQUEST}" == "auto" && -n "${TB3_PREFIX}" ]]; then
  ROBOT_MODEL="turtlebot3_burger"
  WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty_turtlebot3_burger.world"
elif [[ "${ROBOT_MODEL_REQUEST}" != "auto" && "${ROBOT_MODEL_REQUEST}" != "rslg_diff_drive" ]]; then
  echo "Unknown RSLG_GAZEBO_ROBOT_MODEL: ${ROBOT_MODEL_REQUEST}" >&2
  exit 64
fi

if [[ ! -f "${PID_INPUT}" ]]; then
  echo "PID input not found: ${PID_INPUT}" >&2
  exit 66
fi

if [[ ! -f "${PROFILE_JSON}" ]]; then
  echo "Profile JSON not found: ${PROFILE_JSON}" >&2
  exit 66
fi

mkdir -p "${OUTPUT_DIR}"
printf 'robot_model=%s\nworld=%s\n' "${ROBOT_MODEL}" "${WORLD}" > "${OUTPUT_DIR}/robot_model_strategy.txt"

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

if [[ "${MODE_DRY_RUN}" -eq 1 ]]; then
  "${ROS_PYTHON}" "${FOLLOWER}" "${FOLLOWER_ARGS[@]}" --dry-run
  exit 0
fi

GAZEBO_PID=""
RVIZ_PID=""

cleanup() {
  if [[ -n "${RVIZ_PID}" ]]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GAZEBO_PID}" ]]; then
    kill "${GAZEBO_PID}" 2>/dev/null || true
  fi
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
  if [[ "${MODE_HEADLESS}" -eq 1 ]]; then
    gzserver --verbose "${WORLD}" > "${OUTPUT_DIR}/gzserver.log" 2>&1 &
  else
    gazebo --verbose "${WORLD}" > "${OUTPUT_DIR}/gazebo.log" 2>&1 &
  fi
  GAZEBO_PID=$!
  sleep "${RSLG_GAZEBO_BOOT_WAIT_SEC:-6}"
fi

if [[ "${MODE_WITH_RVIZ}" -eq 1 ]]; then
  RSLG_QUERY_ID="${QUERY_ID}" RSLG_ROS_PYTHON="${ROS_PYTHON}" \
    "${REPO_ROOT}/tools/rslg_pipeline/rviz/run_rviz_showcase.sh" --publisher-only \
    > "${OUTPUT_DIR}/rviz_showcase_publisher.log" 2>&1 &
  RVIZ_PID=$!
fi

"${ROS_PYTHON}" "${FOLLOWER}" "${FOLLOWER_ARGS[@]}"
