#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

MODE_DRY_RUN=0
MODE_NO_GAZEBO=0
MODE_WITH_GAZEBO_GUI=0
MODE_HEADLESS=0
MODE_WITH_RVIZ_GUI=0
DURATION_SEC=420
QUERY_ID="${RSLG_QUERY_ID:-00843_cross_floor_object_curtain_room14}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
RVIZ_ONLY_STAIR_FALLBACK=0
# TASK_DIR defaults to the task60 output directory. Callers (e.g. the task62
# unified launcher) may set RSLG_SCRIPTED_STAIR_TASK_DIR to redirect all
# generated inputs, runs, and summaries elsewhere so prior task outputs are
# never overwritten. Default behaviour is unchanged when the env var is unset.
TASK_DIR="${RSLG_SCRIPTED_STAIR_TASK_DIR:-${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task60_scripted_stair_transition_gazebo_rviz_demo}"
PACK_DIR="${TASK_DIR}/scripted_stair_pack"
PYTHON="${RSLG_TOOL_PYTHON:-/home/ws/miniconda3/envs/boxfusion/bin/python}"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

usage() {
  cat <<'EOF'
Usage: run_scripted_stair_transition_demo.sh [options]

Builds and optionally executes the task60 RSLG-SLAM scripted stair-transition
Gazebo/RViz demo. It does not start or depend on Nav2, AMCL, map_server,
Stage-A, raw RGB-D inference, or physical robot code.

Options:
  --dry-run                    Build inputs and run no-ROS dry-run checks.
  --no-gazebo                  Do not start Gazebo; use an external /odom source.
  --with-gazebo-gui            Start gazebo GUI.
  --headless                   Start gzserver headless.
  --with-rviz-gui              Start rviz2 with task60 config.
  --no-rviz-gui                Do not start rviz2 GUI.
  --duration-sec SEC           Executor duration/timeout in seconds.
  --query-id ID                Cross-floor query id.
  --profile-id ID              PID profile id.
  --rviz-only-stair-fallback   If /set_entity_state is unavailable, animate the stair segment in RViz only.
  --help                       Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      MODE_DRY_RUN=1
      ;;
    --no-gazebo)
      MODE_NO_GAZEBO=1
      ;;
    --with-gazebo-gui)
      MODE_WITH_GAZEBO_GUI=1
      ;;
    --headless)
      MODE_HEADLESS=1
      ;;
    --with-rviz-gui)
      MODE_WITH_RVIZ_GUI=1
      ;;
    --no-rviz-gui)
      MODE_WITH_RVIZ_GUI=0
      ;;
    --duration-sec)
      shift
      DURATION_SEC="${1:?--duration-sec requires a value}"
      ;;
    --query-id)
      shift
      QUERY_ID="${1:?--query-id requires a value}"
      ;;
    --profile-id)
      shift
      PROFILE_ID="${1:?--profile-id requires a value}"
      ;;
    --rviz-only-stair-fallback)
      RVIZ_ONLY_STAIR_FALLBACK=1
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

if [[ "${MODE_WITH_GAZEBO_GUI}" -eq 1 && "${MODE_HEADLESS}" -eq 1 ]]; then
  echo "--with-gazebo-gui and --headless are mutually exclusive." >&2
  exit 64
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 does not appear to be sourced. Live Gazebo/RViz mode will source /opt/ros/foxy when available." >&2
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    # shellcheck source=/opt/ros/foxy/setup.bash
    source /opt/ros/foxy/setup.bash
  fi
fi

mkdir -p "${PACK_DIR}/runtime_inputs" "${PACK_DIR}/runs"

PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
BUILDER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/build_scripted_stair_transition_runtime_input.py"
ORCHESTRATOR="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_scripted_stair_demo_orchestrator.py"
RVIZ_NODE="${REPO_ROOT}/tools/rslg_pipeline/gazebo/publish_scripted_stair_composite_rviz.py"
ANIMATOR="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_scripted_stair_animator.py"
WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_scripted_stair_transition_turtlebot3_burger.world"
RVIZ_CONFIG="${REPO_ROOT}/tools/rslg_pipeline/rviz/config/rslg_scripted_stair_transition_showcase.rviz"
RUNTIME_INPUT="${PACK_DIR}/runtime_inputs/${QUERY_ID}_scripted_stair_transition_runtime_input.json"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RSLG_SCRIPTED_STAIR_OUTPUT_DIR:-${PACK_DIR}/runs/${QUERY_ID}_${RUN_ID}}"
mkdir -p "${RUN_DIR}"

"${PYTHON}" "${BUILDER}" \
  --query-id "${QUERY_ID}" \
  --profile-id "${PROFILE_ID}" \
  --task-dir "${TASK_DIR}" \
  --output-json "${RUNTIME_INPUT}" \
  --write-audits \
  --write-helper-scripts > "${RUN_DIR}/build_runtime_input.log"

COMMON_EXEC_ARGS=(
  --runtime-input-json "${RUNTIME_INPUT}"
  --profile-json "${PROFILE_JSON}"
  --profile-id "${PROFILE_ID}"
  --output-dir "${RUN_DIR}"
  --frame-id odom
  --duration-sec "${DURATION_SEC}"
  --rate-hz "${RSLG_SCRIPTED_STAIR_RATE_HZ:-10}"
  --query-id "${QUERY_ID}"
  --anchor-first-waypoint-to-odom-start
  --stop-at-end
)
if [[ "${RVIZ_ONLY_STAIR_FALLBACK}" -eq 1 ]]; then
  COMMON_EXEC_ARGS+=(--rviz-only-stair-fallback)
fi

if [[ "${MODE_DRY_RUN}" -eq 1 ]]; then
  "${PYTHON}" "${ORCHESTRATOR}" "${COMMON_EXEC_ARGS[@]}" \
    --dry-run \
    --summary-json "${TASK_DIR}/09_scripted_stair_dry_run_summary.json" \
    --summary-md "${TASK_DIR}/10_scripted_stair_dry_run_summary.md" > "${RUN_DIR}/orchestrator_dry_run.log"
  "${PYTHON}" "${RVIZ_NODE}" \
    --runtime-input-json "${RUNTIME_INPUT}" \
    --query-id "${QUERY_ID}" \
    --frame-id odom \
    --anchor-first-waypoint-to-odom-start \
    --output-dir "${RUN_DIR}" \
    --dry-run > "${RUN_DIR}/rviz_node_dry_run.log"
  "${PYTHON}" "${ANIMATOR}" \
    --runtime-input-json "${RUNTIME_INPUT}" \
    --query-id "${QUERY_ID}" \
    --frame-id odom \
    --output-dir "${RUN_DIR}" \
    --dry-run > "${RUN_DIR}/animator_dry_run.log"
  cat "${TASK_DIR}/09_scripted_stair_dry_run_summary.json"
  exit 0
fi

export GAZEBO_MODEL_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/models:/opt/ros/foxy/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
if command -v ros2 >/dev/null 2>&1; then
  TB3_PREFIX="$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null || true)"
  if [[ -n "${TB3_PREFIX}" ]]; then
    export GAZEBO_MODEL_PATH="${TB3_PREFIX}/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"
  fi
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
  if [[ "${MODE_WITH_GAZEBO_GUI}" -eq 1 ]]; then
    if ! command -v gazebo >/dev/null 2>&1; then
      echo "gazebo is not available; use --headless or --no-gazebo." >&2
      exit 69
    fi
    gazebo --verbose "${WORLD}" > "${RUN_DIR}/gazebo.log" 2>&1 &
  else
    if ! command -v gzserver >/dev/null 2>&1; then
      echo "gzserver is not available; use --no-gazebo with an external odom source." >&2
      exit 69
    fi
    gzserver --verbose "${WORLD}" > "${RUN_DIR}/gzserver.log" 2>&1 &
  fi
  GAZEBO_PID=$!
  sleep "${RSLG_GAZEBO_BOOT_WAIT_SEC:-8}"
fi

if [[ "${MODE_WITH_RVIZ_GUI}" -eq 1 ]]; then
  if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "${RVIZ_CONFIG}" > "${RUN_DIR}/rviz2.log" 2>&1 &
    RVIZ_PID=$!
  else
    echo "rviz2 was not found; continuing without RViz GUI." >&2
  fi
fi

"${ROS_PYTHON}" "${ORCHESTRATOR}" "${COMMON_EXEC_ARGS[@]}" \
  --summary-json "${TASK_DIR}/11_headless_or_service_demo_execution_summary.json" \
  --summary-md "${TASK_DIR}/12_headless_or_service_demo_execution_summary.md" | tee "${RUN_DIR}/scripted_stair_orchestrator.log"
