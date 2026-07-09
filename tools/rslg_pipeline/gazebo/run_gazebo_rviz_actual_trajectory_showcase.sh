#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

MODE_DRY_RUN=0
MODE_WITH_RVIZ_GUI=0
DURATION_SEC=0
QUERY_ID="${RSLG_QUERY_ID:-00843_object_in_room_curtain_room14}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
FLOOR_ID="${RSLG_GAZEBO_FLOOR_ID:-floor_2}"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

usage() {
  cat <<'EOF'
Usage: run_gazebo_rviz_actual_trajectory_showcase.sh [options]

Starts only the RSLG-SLAM Gazebo actual-trajectory RViz publisher. It does not
start Gazebo, the PID follower, Nav2, AMCL, map_server, or physical robot code.

Options:
  --dry-run              Validate inputs and print planned visualization topics.
  --with-rviz-gui        Start rviz2 with the Gazebo actual trajectory config.
  --no-rviz-gui          Run only the visualization publisher node.
  --query-id ID          Query id to visualize (default: 00843_object_in_room_curtain_room14).
  --duration-sec SEC     Optional publisher duration limit; 0 means run until interrupted.
  --floor-id ID          Floor filter for selected waypoints (default: floor_2).
  --profile-id ID        PID profile id (default: practical_zero_collision).
  --help                 Show this help.

Run the Gazebo world and Gazebo PID follower separately when doing a live manual
validation. This script subscribes to /odom and publishes RViz overlay topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      MODE_DRY_RUN=1
      ;;
    --with-rviz-gui)
      MODE_WITH_RVIZ_GUI=1
      ;;
    --no-rviz-gui)
      MODE_WITH_RVIZ_GUI=0
      ;;
    --query-id)
      shift
      QUERY_ID="${1:?--query-id requires a value}"
      ;;
    --duration-sec)
      shift
      DURATION_SEC="${1:?--duration-sec requires a value}"
      ;;
    --floor-id)
      shift
      FLOOR_ID="${1:?--floor-id requires a value}"
      ;;
    --profile-id)
      shift
      PROFILE_ID="${1:?--profile-id requires a value}"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 64
      ;;
  esac
  shift
done

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 does not appear to be sourced. Source your ROS 2 setup before live mode." >&2
fi

TASK56C_ROOT="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56c_pid_profile_promotion_and_regression_validation/regression_pack"
TASK58B_DIR="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task58b_gazebo_actual_trajectory_rviz_alignment"
PACK_DIR="${TASK58B_DIR}/gazebo_rviz_alignment_pack"
PID_INPUT="${TASK56C_ROOT}/runtime_adapter_inputs/pid_follower_inputs/${QUERY_ID}_pid_runtime_input.json"
ROUTE_RESULT="${TASK56C_ROOT}/route_results/${QUERY_ID}_route_result.json"
MARKER_INPUT="${TASK56C_ROOT}/runtime_adapter_inputs/rviz_marker_inputs/${QUERY_ID}_rviz_marker_input.json"
PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
PUBLISHER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/publish_gazebo_actual_trajectory_rviz.py"
RVIZ_CONFIG="${REPO_ROOT}/tools/rslg_pipeline/rviz/config/rslg_gazebo_actual_trajectory_showcase.rviz"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${RSLG_GAZEBO_RVIZ_OUTPUT_DIR:-${PACK_DIR}/runs/${QUERY_ID}_${RUN_ID}}"

if [[ ! -f "${PID_INPUT}" ]]; then
  echo "PID input not found: ${PID_INPUT}" >&2
  exit 66
fi
if [[ ! -f "${PROFILE_JSON}" ]]; then
  echo "Profile JSON not found: ${PROFILE_JSON}" >&2
  exit 66
fi

mkdir -p "${OUTPUT_DIR}"

PUBLISHER_ARGS=(
  --pid-input-json "${PID_INPUT}"
  --profile-json "${PROFILE_JSON}"
  --profile-id "${PROFILE_ID}"
  --query-id "${QUERY_ID}"
  --floor-id "${FLOOR_ID}"
  --frame-id odom
  --odom-topic /odom
  --anchor-first-waypoint-to-odom-start
  --max-path-points "${RSLG_GAZEBO_RVIZ_MAX_PATH_POINTS:-10000}"
  --publish-rate-hz "${RSLG_GAZEBO_RVIZ_RATE_HZ:-10}"
  --output-dir "${OUTPUT_DIR}"
)

if [[ -f "${ROUTE_RESULT}" ]]; then
  PUBLISHER_ARGS+=(--route-result-json "${ROUTE_RESULT}")
fi
if [[ -f "${MARKER_INPUT}" ]]; then
  PUBLISHER_ARGS+=(--rviz-marker-input-json "${MARKER_INPUT}")
fi

if [[ "${MODE_DRY_RUN}" -eq 1 ]]; then
  "${ROS_PYTHON}" "${PUBLISHER}" "${PUBLISHER_ARGS[@]}" --dry-run
  exit 0
fi

PUBLISHER_PID=""
cleanup() {
  if [[ -n "${PUBLISHER_PID}" ]]; then
    kill "${PUBLISHER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

run_publisher() {
  if [[ "${DURATION_SEC}" != "0" ]]; then
    timeout --signal=INT "${DURATION_SEC}" "${ROS_PYTHON}" "${PUBLISHER}" "${PUBLISHER_ARGS[@]}"
  else
    "${ROS_PYTHON}" "${PUBLISHER}" "${PUBLISHER_ARGS[@]}"
  fi
}

if [[ "${MODE_WITH_RVIZ_GUI}" -eq 1 ]]; then
  run_publisher > "${OUTPUT_DIR}/gazebo_actual_trajectory_rviz_node.log" 2>&1 &
  PUBLISHER_PID=$!
  if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "${RVIZ_CONFIG}"
  else
    echo "rviz2 was not found. The actual trajectory publisher is running; open RViz manually if available." >&2
    wait "${PUBLISHER_PID}"
  fi
else
  run_publisher
fi
