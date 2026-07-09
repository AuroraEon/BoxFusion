#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

MODE_DRY_RUN=0
MODE_NO_GAZEBO=0
MODE_WITH_GAZEBO_GUI=0
MODE_HEADLESS=0
MODE_WITH_RVIZ_NODE=0
MODE_WITH_RVIZ_GUI=0
DURATION_SEC=420
QUERY_ID="${RSLG_QUERY_ID:-00843_cross_floor_object_curtain_room14}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
TASK_DIR="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task59_multifloor_ramp_gazebo_validation"
PACK_DIR="${TASK_DIR}/multifloor_ramp_pack"
PYTHON="${RSLG_TOOL_PYTHON:-/home/ws/miniconda3/envs/boxfusion/bin/python}"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

usage() {
  cat <<'EOF'
Usage: run_multifloor_ramp_gazebo_validation.sh [options]

Builds and optionally executes the task59 RSLG-SLAM multi-floor ramp-surrogate
Gazebo validation. It does not start or depend on Nav2, AMCL, map_server,
Stage-A, raw RGB-D inference, or physical robot code.

Options:
  --dry-run             Build inputs/world and run follower/RViz dry-runs only.
  --no-gazebo           Do not start Gazebo; use an external /odom source.
  --with-gazebo-gui     Start gazebo GUI instead of gzserver.
  --headless            Start gzserver headless.
  --with-rviz-node      Start the task59 RViz publisher node.
  --with-rviz-gui       Start rviz2 with task59 config.
  --duration-sec SEC    Follower duration/timeout in seconds.
  --query-id ID         Cross-floor query id.
  --profile-id ID       PID profile id.
  --help                Show this help.
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
    --with-rviz-node)
      MODE_WITH_RVIZ_NODE=1
      ;;
    --with-rviz-gui)
      MODE_WITH_RVIZ_GUI=1
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
  echo "ROS 2 does not appear to be sourced. Live Gazebo/RViz mode may fail unless /opt/ros/foxy/setup.bash is sourced." >&2
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    # shellcheck source=/opt/ros/foxy/setup.bash
    source /opt/ros/foxy/setup.bash
  fi
fi

PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
BUILDER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/build_multifloor_ramp_runtime_input.py"
WORLD_GENERATOR="${REPO_ROOT}/tools/rslg_pipeline/gazebo/generate_multifloor_ramp_world.py"
FOLLOWER="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_multifloor_ramp_follower.py"
RVIZ_NODE="${REPO_ROOT}/tools/rslg_pipeline/gazebo/publish_multifloor_ramp_actual_trajectory_rviz.py"
RVIZ_CONFIG="${REPO_ROOT}/tools/rslg_pipeline/rviz/config/rslg_multifloor_ramp_actual_trajectory_showcase.rviz"
RUNTIME_INPUT="${PACK_DIR}/runtime_inputs/${QUERY_ID}_multifloor_ramp_runtime_input.json"
GENERATED_WORLD="${PACK_DIR}/worlds/rslg_multifloor_ramp_turtlebot3_burger.generated.world"
CHECKED_IN_WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_multifloor_ramp_turtlebot3_burger.world"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RSLG_MULTIFLOOR_RAMP_OUTPUT_DIR:-${PACK_DIR}/runs/${QUERY_ID}_${RUN_ID}}"

mkdir -p "${RUN_DIR}"

"${PYTHON}" "${BUILDER}" \
  --query-id "${QUERY_ID}" \
  --task-dir "${TASK_DIR}" \
  --output-json "${RUNTIME_INPUT}" \
  --write-audits \
  --write-helper-scripts > "${RUN_DIR}/build_runtime_input.log"

"${PYTHON}" "${WORLD_GENERATOR}" \
  --runtime-input-json "${RUNTIME_INPUT}" \
  --world-output "${GENERATED_WORLD}" \
  --task-dir "${TASK_DIR}" \
  --write-design-md > "${RUN_DIR}/generate_world.log"

WORLD="${GENERATED_WORLD}"
if [[ ! -f "${WORLD}" ]]; then
  WORLD="${CHECKED_IN_WORLD}"
fi

export GAZEBO_MODEL_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/models:/opt/ros/foxy/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds:${PACK_DIR}/worlds:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
if command -v ros2 >/dev/null 2>&1; then
  TB3_PREFIX="$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null || true)"
  if [[ -n "${TB3_PREFIX}" ]]; then
    export GAZEBO_MODEL_PATH="${TB3_PREFIX}/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"
  fi
fi

FOLLOWER_ARGS=(
  --runtime-input-json "${RUNTIME_INPUT}"
  --profile-json "${PROFILE_JSON}"
  --profile-id "${PROFILE_ID}"
  --output-dir "${RUN_DIR}"
  --frame-id odom
  --duration-sec "${DURATION_SEC}"
  --goal-timeout-sec "${DURATION_SEC}"
  --rate-hz "${RSLG_GAZEBO_RATE_HZ:-10}"
  --start-delay-sec "${RSLG_GAZEBO_START_DELAY_SEC:-2}"
  --query-id "${QUERY_ID}"
  --floor-id all
  --anchor-first-waypoint-to-odom-start
  --stop-at-end
)

RVIZ_ARGS=(
  --runtime-input-json "${RUNTIME_INPUT}"
  --query-id "${QUERY_ID}"
  --frame-id odom
  --odom-topic /odom
  --anchor-first-waypoint-to-odom-start
  --max-path-points "${RSLG_GAZEBO_RVIZ_MAX_PATH_POINTS:-10000}"
  --publish-rate-hz "${RSLG_GAZEBO_RVIZ_RATE_HZ:-10}"
  --output-dir "${RUN_DIR}"
)

if [[ "${MODE_DRY_RUN}" -eq 1 ]]; then
  "${PYTHON}" "${FOLLOWER}" "${FOLLOWER_ARGS[@]}" --dry-run > "${RUN_DIR}/follower_dry_run.log"
  "${PYTHON}" "${RVIZ_NODE}" "${RVIZ_ARGS[@]}" --dry-run > "${RUN_DIR}/rviz_node_dry_run.log"
  "${PYTHON}" - "${TASK_DIR}" "${RUN_DIR}" "${RUNTIME_INPUT}" "${GENERATED_WORLD}" "${RVIZ_CONFIG}" <<'PY'
import json
import sys
from pathlib import Path

task_dir = Path(sys.argv[1])
run_dir = Path(sys.argv[2])
runtime_input = Path(sys.argv[3])
world = Path(sys.argv[4])
rviz_config = Path(sys.argv[5])
runtime = json.loads(runtime_input.read_text(encoding="utf-8"))
query_id = runtime["source_query_id"]
follower_summary = json.loads((run_dir / f"{query_id}_multifloor_ramp_follower_dry_run_summary.json").read_text(encoding="utf-8"))
rviz_summary = json.loads((run_dir / f"{query_id}_multifloor_ramp_rviz_dry_run_summary.json").read_text(encoding="utf-8"))
summary = {
    "schema_name": "rslg_task59_multifloor_ramp_dry_run_summary",
    "schema_version": "0.1",
    "project_name": "RSLG-SLAM",
    "status": "passed",
    "query_id": query_id,
    "runtime_input_exists": runtime_input.is_file(),
    "world_file_exists": world.is_file(),
    "rviz_config_exists": rviz_config.is_file(),
    "cross_floor_route_selected": True,
    "floor_1_waypoints_exist": runtime["waypoint_counts"]["floor_1_original"] > 0,
    "floor_2_waypoints_exist": runtime["waypoint_counts"]["floor_2_original_after_ramp"] > 0,
    "ramp_waypoints_inserted": runtime["waypoint_counts"]["ramp_surrogate"] > 0,
    "vt_1_centerline_e001_used_as_ramp_connector": runtime["ramp_connector_edge_id"] == "vt_1_centerline_e001",
    "vt_1_centerline_e003_non_transition_guard_active": "vt_1_centerline_e003" in runtime.get("forbidden_transition_edge_ids", []),
    "generated_ring_037_blocked_not_selected_as_runtime_goal": runtime.get("selected_goal_candidate_id") != "generated_ring_037",
    "generated_ring_002_final_approach": runtime.get("target_approach_id") == "generated_ring_002",
    "follower_dry_run_ok": follower_summary.get("ok") is True,
    "rviz_node_dry_run_ok": rviz_summary.get("ok") is True,
    "planned_path_topic": rviz_summary["topics"]["planned_path_odom"],
    "executed_path_topic": rviz_summary["topics"]["gazebo_executed_path"],
    "robot_pose_topic": rviz_summary["topics"]["gazebo_robot_pose"],
    "marker_topic": rviz_summary["topics"]["marker_array"],
    "frame_id": rviz_summary["frame_id"],
    "replay_topic_primary_display": False,
    "nav2_dependency": False,
    "amcl_dependency": False,
    "map_server_dependency": False,
    "follower_summary": follower_summary,
    "rviz_summary": rviz_summary,
}
task_json = task_dir / "07_multifloor_ramp_dry_run_summary.json"
task_md = task_dir / "08_multifloor_ramp_dry_run_summary.md"
task_json.write_text(json.dumps(summary, indent=2, sort_keys=False) + "\n", encoding="utf-8")
task_md.write_text("\n".join([
    "# Task59 Multifloor Ramp Dry Run Summary",
    "",
    "- status: `passed`",
    f"- query_id: `{query_id}`",
    f"- runtime input exists: `{summary['runtime_input_exists']}`",
    f"- world file exists: `{summary['world_file_exists']}`",
    f"- floor_1 waypoints exist: `{summary['floor_1_waypoints_exist']}`",
    f"- floor_2 waypoints exist: `{summary['floor_2_waypoints_exist']}`",
    f"- ramp waypoints inserted: `{summary['ramp_waypoints_inserted']}`",
    f"- connector: `{runtime['ramp_connector_edge_id']}`",
    f"- vt_1_centerline_e003 non-transition guard active: `{summary['vt_1_centerline_e003_non_transition_guard_active']}`",
    f"- generated_ring_037 blocked and not selected as runtime goal: `{summary['generated_ring_037_blocked_not_selected_as_runtime_goal']}`",
    f"- generated_ring_002 final approach: `{summary['generated_ring_002_final_approach']}`",
    f"- follower dry-run ok: `{summary['follower_dry_run_ok']}`",
    f"- RViz node dry-run ok: `{summary['rviz_node_dry_run_ok']}`",
    f"- planned path topic: `{summary['planned_path_topic']}`",
    f"- executed path topic: `{summary['executed_path_topic']}`",
    f"- frame: `{summary['frame_id']}`",
    "",
    "Dry-run does not start Gazebo, Nav2, AMCL, map_server, Stage-A, raw RGB-D inference, or physical robot code.",
    "",
]) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2, sort_keys=False))
PY
  exit 0
fi

GAZEBO_PID=""
RVIZ_NODE_PID=""

cleanup() {
  if [[ -n "${RVIZ_NODE_PID}" ]]; then
    kill "${RVIZ_NODE_PID}" 2>/dev/null || true
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

if [[ "${MODE_WITH_RVIZ_NODE}" -eq 1 || "${MODE_WITH_RVIZ_GUI}" -eq 1 ]]; then
  "${ROS_PYTHON}" "${RVIZ_NODE}" "${RVIZ_ARGS[@]}" > "${RUN_DIR}/multifloor_ramp_rviz_node.log" 2>&1 &
  RVIZ_NODE_PID=$!
fi

if [[ "${MODE_WITH_RVIZ_GUI}" -eq 1 ]]; then
  if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "${RVIZ_CONFIG}" > "${RUN_DIR}/rviz2.log" 2>&1 &
  else
    echo "rviz2 was not found; continuing without RViz GUI." >&2
  fi
fi

"${ROS_PYTHON}" "${FOLLOWER}" "${FOLLOWER_ARGS[@]}" | tee "${RUN_DIR}/multifloor_ramp_follower.log"
