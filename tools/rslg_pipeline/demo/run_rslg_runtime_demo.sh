#!/usr/bin/env bash
set -euo pipefail

# RSLG-SLAM unified runtime demo launcher (task62).
#
# Single stable entrypoint for the RSLG-SLAM runtime validation demos. It hides
# task-specific path complexity behind a demo route registry and reuses the
# stable task58 (same-floor Gazebo PID) and task60 (scripted stair transition)
# tools. It never starts or depends on Nav2, AMCL, map_server, Stage-A, raw
# RGB-D inference, or physical robot code, and it never claims physical stair
# climbing or real robot deployment.
#
# Robust repo-root discovery: walk upward until a directory containing both
# tools/rslg_pipeline and stage_outputs is found (no fragile fixed-depth paths).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
while [[ "${REPO_ROOT}" != "/" ]]; do
  if [[ -d "${REPO_ROOT}/tools/rslg_pipeline" && -d "${REPO_ROOT}/stage_outputs" ]]; then
    break
  fi
  REPO_ROOT="$(dirname "${REPO_ROOT}")"
done
if [[ ! -d "${REPO_ROOT}/tools/rslg_pipeline" ]]; then
  echo "Could not locate RSLG-SLAM repo root from ${SCRIPT_DIR}" >&2
  exit 1
fi

REGISTRY="${RSLG_DEMO_REGISTRY:-${REPO_ROOT}/tools/rslg_pipeline/demo/rslg_demo_route_registry_v0_1.json}"
HELPER_PY="${REPO_ROOT}/tools/rslg_pipeline/demo/run_rslg_runtime_demo.py"
TOOL_PYTHON="${RSLG_TOOL_PYTHON:-/home/ws/miniconda3/envs/boxfusion/bin/python}"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"
TASK62_DIR="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task62_demo_package_cleanup_and_route_generalization"

MODE_LIST=0
MODE_PRINT_PATHS=0
MODE_DRY_RUN=0
MODE_HEADLESS=0
MODE_WITH_GAZEBO_GUI=0
MODE_WITH_RVIZ_GUI=0
NO_RVIZ_GUI=0
QUERY_ID=""
REQUESTED_MODE="auto"
DURATION_SEC=""
OUTPUT_DIR=""
PROFILE_ID="practical_zero_collision"
ROS_DOMAIN_ID_OVERRIDE=""
GAZEBO_MASTER_URI_OVERRIDE=""

usage() {
  cat <<'EOF'
Usage: run_rslg_runtime_demo.sh [options]

Selection / info:
  --list                       List supported demo routes from the registry.
  --print-paths                Print REPO_ROOT, registry, tool, and route paths.
  --query-id ID                Select a demo route by query id.
  --mode MODE                  auto|same_floor_pid|scripted_stair_transition|connector_scripted_transition

Validation:
  --dry-run                    Pure-static dry-run validation (no Gazebo/RViz).

Live execution (optional; reuses stable task58/task60 tools):
  --headless                   Run selected route headless (gzserver, no GUI).
  --with-gazebo-gui            Launch the appropriate Gazebo world GUI.
  --with-rviz-gui              Launch the appropriate RViz config.
  --no-rviz-gui                Do not launch RViz.

Common:
  --duration-sec SEC           Executor duration/timeout (default: registry value).
  --output-dir DIR             Output dir (default: under task62 demo outputs).
  --profile-id ID              PID profile id (default: practical_zero_collision).
  --ros-domain-id N            Optional ROS_DOMAIN_ID override.
  --gazebo-master-uri URI      Optional GAZEBO_MASTER_URI override.
  --help                       Show this help.

This launcher never runs Nav2/AMCL/map_server, Stage-A, or raw RGB-D inference,
and never claims physical stair climbing or real robot deployment. The scripted
stair transition is a visual/runtime-interface animation only.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --list) MODE_LIST=1 ;;
    --print-paths) MODE_PRINT_PATHS=1 ;;
    --dry-run) MODE_DRY_RUN=1 ;;
    --headless) MODE_HEADLESS=1 ;;
    --with-gazebo-gui) MODE_WITH_GAZEBO_GUI=1 ;;
    --with-rviz-gui) MODE_WITH_RVIZ_GUI=1 ;;
    --no-rviz-gui) NO_RVIZ_GUI=1 ;;
    --query-id) shift; QUERY_ID="${1:?--query-id requires a value}" ;;
    --mode) shift; REQUESTED_MODE="${1:?--mode requires a value}" ;;
    --duration-sec) shift; DURATION_SEC="${1:?--duration-sec requires a value}" ;;
    --output-dir) shift; OUTPUT_DIR="${1:?--output-dir requires a value}" ;;
    --profile-id) shift; PROFILE_ID="${1:?--profile-id requires a value}" ;;
    --ros-domain-id) shift; ROS_DOMAIN_ID_OVERRIDE="${1:?--ros-domain-id requires a value}" ;;
    --gazebo-master-uri) shift; GAZEBO_MASTER_URI_OVERRIDE="${1:?--gazebo-master-uri requires a value}" ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 64 ;;
  esac
  shift
done

if [[ ! -f "${HELPER_PY}" ]]; then
  echo "Demo helper not found: ${HELPER_PY}" >&2
  exit 66
fi

# --- Info / validation modes (pure static, no ROS) ---------------------------
if [[ "${MODE_LIST}" -eq 1 ]]; then
  exec "${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --list
fi

if [[ "${MODE_PRINT_PATHS}" -eq 1 ]]; then
  echo "SCRIPT_DIR=${SCRIPT_DIR}"
  echo "REPO_ROOT=${REPO_ROOT}"
  echo "REGISTRY=${REGISTRY}"
  echo "HELPER_PY=${HELPER_PY}"
  echo "TOOL_PYTHON=${TOOL_PYTHON}"
  echo "ROS_PYTHON=${ROS_PYTHON}"
  echo "TASK62_DIR=${TASK62_DIR}"
  if [[ -n "${QUERY_ID}" ]]; then
    "${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --print-paths --query-id "${QUERY_ID}"
  else
    "${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --print-paths
  fi
  exit 0
fi

if [[ "${MODE_DRY_RUN}" -eq 1 ]]; then
  if [[ -z "${QUERY_ID}" ]]; then
    exec "${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --dry-run-all \
      ${OUTPUT_DIR:+--output-dir "${OUTPUT_DIR}"}
  fi
  exec "${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --dry-run --query-id "${QUERY_ID}" \
    ${OUTPUT_DIR:+--output-dir "${OUTPUT_DIR}"}
fi

# --- Live execution modes ----------------------------------------------------
if [[ "${MODE_HEADLESS}" -eq 0 && "${MODE_WITH_GAZEBO_GUI}" -eq 0 && "${MODE_WITH_RVIZ_GUI}" -eq 0 ]]; then
  echo "No action specified. Use --list, --print-paths, --dry-run, --headless, --with-gazebo-gui, or --with-rviz-gui." >&2
  usage >&2
  exit 64
fi

if [[ -z "${QUERY_ID}" ]]; then
  echo "Live execution requires --query-id." >&2
  exit 64
fi

# Resolve the registered mode for the selected route.
RESOLVED_MODE="$("${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --resolve --query-id "${QUERY_ID}" --field MODE)"
FLOOR_ID_FILTER="$("${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --resolve --query-id "${QUERY_ID}" --field FLOOR_ID_FILTER)"
REG_DURATION="$("${TOOL_PYTHON}" "${HELPER_PY}" --registry "${REGISTRY}" --resolve --query-id "${QUERY_ID}" --field RECOMMENDED_DURATION_SEC)"

EFFECTIVE_MODE="${REQUESTED_MODE}"
if [[ "${REQUESTED_MODE}" == "auto" ]]; then
  EFFECTIVE_MODE="${RESOLVED_MODE}"
fi
if [[ "${EFFECTIVE_MODE}" != "${RESOLVED_MODE}" ]]; then
  echo "Requested mode '${REQUESTED_MODE}' does not match registry mode '${RESOLVED_MODE}' for ${QUERY_ID}." >&2
  exit 64
fi
if [[ -z "${DURATION_SEC}" ]]; then
  DURATION_SEC="${REG_DURATION:-120}"
fi

if [[ "${EFFECTIVE_MODE}" == "dry_run_only" ]]; then
  echo "Route ${QUERY_ID} is dry_run_only; live execution is refused." >&2
  echo "Use: run_rslg_runtime_demo.sh --query-id ${QUERY_ID} --dry-run" >&2
  exit 65
fi

if [[ -n "${ROS_DOMAIN_ID_OVERRIDE}" ]]; then
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_OVERRIDE}"
fi
if [[ -n "${GAZEBO_MASTER_URI_OVERRIDE}" ]]; then
  export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI_OVERRIDE}"
fi

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
DEFAULT_OUT="${TASK62_DIR}/demo_package_pack/runs/${QUERY_ID}_${RUN_ID}"
RUN_OUT="${OUTPUT_DIR:-${DEFAULT_OUT}}"
mkdir -p "${RUN_OUT}"

echo "== RSLG-SLAM demo launcher =="
echo "query_id=${QUERY_ID} mode=${EFFECTIVE_MODE} duration=${DURATION_SEC}s output=${RUN_OUT}"

case "${EFFECTIVE_MODE}" in
  same_floor_pid)
    SMOKE="${REPO_ROOT}/tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh"
    SMOKE_ARGS=(--duration-sec "${DURATION_SEC}")
    if [[ "${MODE_HEADLESS}" -eq 1 ]]; then
      SMOKE_ARGS+=(--headless)
    fi
    if [[ "${MODE_WITH_RVIZ_GUI}" -eq 1 && "${NO_RVIZ_GUI}" -eq 0 ]]; then
      SMOKE_ARGS+=(--with-rviz)
    fi
    RSLG_QUERY_ID="${QUERY_ID}" \
      RSLG_PROFILE_ID="${PROFILE_ID}" \
      RSLG_GAZEBO_FLOOR_ID="${FLOOR_ID_FILTER:-floor_2}" \
      RSLG_GAZEBO_OUTPUT_DIR="${RUN_OUT}" \
      "${SMOKE}" "${SMOKE_ARGS[@]}"
    ;;
  scripted_stair_transition|connector_scripted_transition)
    STAIR="${REPO_ROOT}/tools/rslg_pipeline/gazebo/run_scripted_stair_transition_demo.sh"
    STAIR_ARGS=(--query-id "${QUERY_ID}" --profile-id "${PROFILE_ID}" --duration-sec "${DURATION_SEC}")
    if [[ "${MODE_HEADLESS}" -eq 1 ]]; then
      STAIR_ARGS+=(--headless --rviz-only-stair-fallback)
    fi
    if [[ "${MODE_WITH_GAZEBO_GUI}" -eq 1 ]]; then
      STAIR_ARGS+=(--with-gazebo-gui)
    fi
    if [[ "${MODE_WITH_RVIZ_GUI}" -eq 1 && "${NO_RVIZ_GUI}" -eq 0 ]]; then
      STAIR_ARGS+=(--with-rviz-gui)
    else
      STAIR_ARGS+=(--no-rviz-gui)
    fi
    # Isolate all scripted-stair outputs under the task62 run dir so task60
    # outputs are never overwritten (honoured additively by the task60 tool).
    RSLG_SCRIPTED_STAIR_TASK_DIR="${RUN_OUT}/scripted_stair_task_dir" \
      RSLG_SCRIPTED_STAIR_OUTPUT_DIR="${RUN_OUT}/run" \
      "${STAIR}" "${STAIR_ARGS[@]}"
    ;;
  dry_run_only)
    echo "Route ${QUERY_ID} is dry_run_only; live execution is refused." >&2
    echo "Use: run_rslg_runtime_demo.sh --query-id ${QUERY_ID} --dry-run" >&2
    exit 65
    ;;
  *)
    echo "Unknown resolved mode: ${EFFECTIVE_MODE}" >&2
    exit 70
    ;;
esac
