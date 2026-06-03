#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SCENE_ID="00843-DYehNKdT76V"
FLOOR_ID="floor_2"
ROUTE_ID="floor_2_room11_to_room14"
STAGE_OUTPUT_DIR="$REPO_ROOT/stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun"
ROUTE_DIR="$STAGE_OUTPUT_DIR/routes/room_routes/$ROUTE_ID"
SEMANTIC_WAYPOINTS_JSON="$ROUTE_DIR/semantic_route_waypoints_v0_1.json"
EXECUTABLE_WAYPOINTS_JSON="$ROUTE_DIR/executable_route_waypoints_v0_1.json"
EXECUTABLE_DIAGNOSTICS_JSON="$ROUTE_DIR/executable_route_diagnostics_v0_1.json"
MAP_YAML="$STAGE_OUTPUT_DIR/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
RUNTIME_PROFILE="$STAGE_OUTPUT_DIR/runtime/profiles/floor_2_nav2/runtime_profile.json"
BASELINE_RUNTIME_PROFILE="$RUNTIME_PROFILE"
TASK12_ROBUST_RUNTIME_PROFILE="$STAGE_OUTPUT_DIR/runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
RVIZ_CONFIG="$STAGE_OUTPUT_DIR/runtime/rviz/00843_floor2_route_debug_marker_map.rviz"
MAP_MARKER_TOPIC="/stage1_nav/scene_00843/floor_2/map_overlay_markers"
ROUTE_MARKER_TOPIC="/stage1_nav/scene_00843/floor_2/route_debug_markers"
EXPECTED_ROOM_CHAIN="room_11,room_7,room_13,room_14"
EXPECTED_GATEWAY_SEQUENCE="00843_floor2_gateway_005,00843_floor2_gateway_004,00843_floor2_gateway_006"
ROBOT_MODEL_NAME="00843_floor2_turtlebot3"
EXECUTION_STRATEGY="split_follow_path_with_current_pose_gateway_handoff"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"

GUI=0
RVIZ=0
KEEP_OPEN_SEC=20
RUN_ID=""
RECORD_CONTROLLER_BAG=0
SKIP_BRINGUP=0
SKIP_ROUTE=0
DRY_RUN=0
CONTROLLER_PROFILE="baseline"

usage() {
  cat <<'EOF'
Usage:
  run_00843_floor2_demo.sh [--gui|--no-gui] [--rviz|--no-rviz] [--controller-profile baseline|task12_robust] [--keep-open-sec N] [--run-id ID] [--record-controller-bag] [--skip-bringup] [--skip-route] [--dry-run]

Examples:
  tools/stage1_runtime/run_00843_floor2_demo.sh --gui --rviz
  tools/stage1_runtime/run_00843_floor2_demo.sh --gui --rviz --controller-profile task12_robust
  tools/stage1_runtime/run_00843_floor2_demo.sh --skip-bringup --rviz
  tools/stage1_runtime/run_00843_floor2_demo.sh --dry-run --controller-profile task12_robust
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --gui) GUI=1; shift ;;
    --no-gui|--headless) GUI=0; shift ;;
    --rviz) RVIZ=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --keep-open-sec) KEEP_OPEN_SEC="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --record-controller-bag) RECORD_CONTROLLER_BAG=1; shift ;;
    --controller-profile|--nav2-profile) CONTROLLER_PROFILE="$2"; shift 2 ;;
    --skip-bringup) SKIP_BRINGUP=1; shift ;;
    --skip-route) SKIP_ROUTE=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[task11_demo][ERROR] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$CONTROLLER_PROFILE" in
  baseline)
    RUNTIME_PROFILE="$BASELINE_RUNTIME_PROFILE"
    ;;
  task12_robust)
    RUNTIME_PROFILE="$TASK12_ROBUST_RUNTIME_PROFILE"
    ;;
  *)
    echo "[task11_demo][ERROR] unknown controller profile: $CONTROLLER_PROFILE" >&2
    usage >&2
    exit 2
    ;;
esac

if [ -z "$RUN_ID" ]; then
  RUN_ID="task12_00843_floor2_demo_${CONTROLLER_PROFILE}_$(date -u +%Y%m%dT%H%M%SZ)"
fi
RUN_DIR="$STAGE_OUTPUT_DIR/runs/active/$RUN_ID"
ROUTE_RESULT="$RUN_DIR/route_execution_result_v0_1.json"
WALL_CROSSING="$RUN_DIR/wall_crossing_validation.json"
SENT_CONTROLLER_PATHS="$RUN_DIR/sent_controller_paths"
TRANSCRIPT="$RUN_DIR/task11_demo_commands.log"

mkdir -p "$RUN_DIR"
: > "$TRANSCRIPT"

echo "RUN_ID=$RUN_ID"
echo "RUN_DIR=$RUN_DIR"
echo "controller_profile=$CONTROLLER_PROFILE"
echo "runtime_profile=$RUNTIME_PROFILE"
echo "route_result=$ROUTE_RESULT"
echo "rviz_config=$RVIZ_CONFIG"
echo "wall_crossing=$WALL_CROSSING"
echo "sent_controller_paths=$SENT_CONTROLLER_PATHS"

quote_cmd() {
  printf '%q ' "$@"
  printf '\n'
}

record_cmd() {
  local label="$1"; shift
  {
    echo "[$label]"
    quote_cmd "$@"
  } >> "$TRANSCRIPT"
}

run_cmd() {
  local label="$1"; shift
  local log_file="$RUN_DIR/${label}.log"
  record_cmd "$label" "$@"
  echo "[task11_demo] $label -> $log_file"
  "$@" > "$log_file" 2>&1
  return $?
}

source_ros_setup() {
  local had_nounset=0
  case "$-" in *u*) had_nounset=1 ;; esac
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/foxy/setup.bash
  if [ "$had_nounset" = "1" ]; then set -u; else set +u; fi
}

BRINGUP_CMD=(
  "$SCRIPT_DIR/launch_scene_gazebo_nav2.sh"
  --scene-id "$SCENE_ID"
  --floor-id "$FLOOR_ID"
  --stage-output-dir "$STAGE_OUTPUT_DIR"
  --map-yaml "$MAP_YAML"
  --runtime-profile "$RUNTIME_PROFILE"
  --run-id "$RUN_ID"
  --ros-domain-id "$ROS_DOMAIN_ID_VALUE"
  --readiness-output-json "$RUN_DIR/lifecycle_readiness_report_v0_1.json"
  --readiness-output-md "$RUN_DIR/lifecycle_readiness_report_v0_1.md"
)
if [ "$GUI" = "1" ]; then BRINGUP_CMD+=(--gui); else BRINGUP_CMD+=(--headless); fi

ROUTE_CMD=(
  /usr/bin/python3 "$SCRIPT_DIR/run_scene_route.py"
  --scene-id "$SCENE_ID"
  --floor-id "$FLOOR_ID"
  --runtime-profile "$RUNTIME_PROFILE"
  --stage-output-dir "$STAGE_OUTPUT_DIR"
  --waypoints-json "$EXECUTABLE_WAYPOINTS_JSON"
  --expected-room-chain "$EXPECTED_ROOM_CHAIN"
  --expected-gateway-sequence "$EXPECTED_GATEWAY_SEQUENCE"
  --allow-non-scene-truth
  --reset-to-route-start
  --from-start
  --robot-model-name "$ROBOT_MODEL_NAME"
  --execution-strategy "$EXECUTION_STRATEGY"
  --split-at-through-room-anchors
  --validate-wall-crossing
  --wall-crossing-map-yaml "$MAP_YAML"
  --wall-crossing-output-json "$WALL_CROSSING"
  --current-pose-gateway-handoff
  --handoff-target-distance-m 1.0
  --handoff-target-waypoint-index 51
  --output-json "$ROUTE_RESULT"
  --output-md "$RUN_DIR/route_execution_result_v0_1.md"
  --trajectory-output-json "$RUN_DIR/trajectory_sample_result_v0_1.json"
  --latest-slice-output-json "$RUN_DIR/latest_follow_path_slice_v0_1.json"
)

MAP_OVERLAY_CMD=(
  /usr/bin/python3 "$SCRIPT_DIR/publish_scene_map_overlay.py"
  --map-yaml "$MAP_YAML"
  --marker-topic "$MAP_MARKER_TOPIC"
  --namespace "scene_00843/floor_2/map_overlay"
  --rate-hz 1.0
)

ROUTE_OVERLAY_CMD=(
  /usr/bin/python3 "$SCRIPT_DIR/publish_scene_route_debug_overlay.py"
  --scene-id "$SCENE_ID"
  --floor-id "$FLOOR_ID"
  --stage-output-dir "$STAGE_OUTPUT_DIR"
  --runtime-profile "$RUNTIME_PROFILE"
  --semantic-waypoints-json "$SEMANTIC_WAYPOINTS_JSON"
  --executable-waypoints-json "$EXECUTABLE_WAYPOINTS_JSON"
  --diagnostics-json "$EXECUTABLE_DIAGNOSTICS_JSON"
  --run-dir "$RUN_DIR"
  --trajectory-trace-jsonl "$RUN_DIR/route_execution_trace_v0_1.jsonl"
  --sent-controller-paths-dir "$SENT_CONTROLLER_PATHS"
  --route-execution-json "$ROUTE_RESULT"
  --marker-topic "$ROUTE_MARKER_TOPIC"
  --show-sent-controller-paths
  --show-handoff-paths
  --show-success-trajectory
  --show-key-waypoints
  --show-phase-trajectory
  --rate-hz 1.0
)

RVIZ_CMD=(rviz2 -d "$RVIZ_CONFIG")
BAG_CMD=(ros2 bag record -o "$RUN_DIR/controller_bag" /tf /tf_static /cmd_vel /plan /local_plan /follow_path/_action/feedback)

record_cmd "planned_bringup" "${BRINGUP_CMD[@]}"
record_cmd "planned_route" "${ROUTE_CMD[@]}"
record_cmd "planned_map_overlay" "${MAP_OVERLAY_CMD[@]}"
record_cmd "planned_route_overlay" "${ROUTE_OVERLAY_CMD[@]}"
record_cmd "planned_rviz" "${RVIZ_CMD[@]}"
if [ "$RECORD_CONTROLLER_BAG" = "1" ]; then record_cmd "planned_controller_bag" "${BAG_CMD[@]}"; fi

if [ "$DRY_RUN" = "1" ]; then
  echo "[task11_demo] dry-run only; commands were written to $TRANSCRIPT"
  sed -n '1,120p' "$TRANSCRIPT"
  exit 0
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"

if [ "$RVIZ" = "1" ]; then
  source_ros_setup
  setsid "${MAP_OVERLAY_CMD[@]}" > "$RUN_DIR/map_overlay_publisher.log" 2>&1 &
  PIDS+=("$!")
  if [ "$SKIP_ROUTE" = "1" ]; then
    setsid "${ROUTE_OVERLAY_CMD[@]}" > "$RUN_DIR/route_debug_overlay_publisher.log" 2>&1 &
    PIDS+=("$!")
  fi
  setsid "${RVIZ_CMD[@]}" > "$RUN_DIR/rviz2.log" 2>&1 &
  PIDS+=("$!")
fi

BRINGUP_RC=0
if [ "$SKIP_BRINGUP" = "0" ]; then
  run_cmd "bringup" "${BRINGUP_CMD[@]}"
  BRINGUP_RC=$?
else
  echo "[task11_demo] skipping bringup"
fi

BAG_PID=""
if [ "$RECORD_CONTROLLER_BAG" = "1" ] && [ "$SKIP_ROUTE" = "0" ]; then
  source_ros_setup
  setsid "${BAG_CMD[@]}" > "$RUN_DIR/controller_bag_record.log" 2>&1 &
  BAG_PID="$!"
  PIDS+=("$BAG_PID")
fi

ROUTE_RC=0
if [ "$SKIP_ROUTE" = "0" ]; then
  run_cmd "route" "${ROUTE_CMD[@]}"
  ROUTE_RC=$?
  if [ -n "$BAG_PID" ]; then kill "$BAG_PID" >/dev/null 2>&1 || true; fi
  if [ "$RVIZ" = "1" ]; then
    setsid "${ROUTE_OVERLAY_CMD[@]}" > "$RUN_DIR/route_debug_overlay_publisher.log" 2>&1 &
    PIDS+=("$!")
  fi
else
  echo "[task11_demo] skipping route execution"
fi

/usr/bin/python3 - "$ROUTE_RESULT" "$RUN_DIR" "$WALL_CROSSING" <<'PY'
import json
import sys
from pathlib import Path

route_result = Path(sys.argv[1])
run_dir = Path(sys.argv[2])
wall_path = Path(sys.argv[3])
if not route_result.exists():
    payload = {
        "succeeded": None,
        "clean_runtime_success": False,
        "terminal_reached": None,
        "execution_strategy": None,
        "split_follow_path_used": None,
        "sparse_fallback_used": None,
        "failure_layer": "route_execution",
        "failure_reason": f"missing route result: {route_result}",
        "wall_crossing_validation_passed": None,
        "RUN_DIR": run_dir.as_posix(),
        "interpretation": "failure",
        "controller_profile": None,
        "runtime_profile": None,
    }
else:
    payload = json.load(route_result.open(encoding="utf-8"))
    wall = {}
    if wall_path.exists():
        wall = json.load(wall_path.open(encoding="utf-8"))
    sample_count = wall.get("sample_count")
    no_wall_samples = sample_count == 0
    clean = (
        bool(payload.get("clean_runtime_success"))
        and bool(payload.get("terminal_reached"))
        and payload.get("execution_strategy") == "split_follow_path_with_current_pose_gateway_handoff"
        and not bool(payload.get("sparse_fallback_used"))
        and not no_wall_samples
    )
    fallback = bool(payload.get("succeeded")) and bool(payload.get("sparse_fallback_used"))
    interpretation = "clean success" if clean else "fallback success" if fallback else "failure"
    if no_wall_samples:
        interpretation += " (wall crossing had no valid trajectory samples)"
    payload = {
        "succeeded": payload.get("succeeded"),
        "clean_runtime_success": payload.get("clean_runtime_success"),
        "terminal_reached": payload.get("terminal_reached"),
        "execution_strategy": payload.get("execution_strategy"),
        "split_follow_path_used": payload.get("split_follow_path_used"),
        "sparse_fallback_used": payload.get("sparse_fallback_used"),
        "failure_layer": payload.get("failure_layer"),
        "failure_reason": payload.get("failure_reason"),
        "wall_crossing_validation_passed": payload.get("wall_crossing_validation_passed"),
        "wall_crossing_sample_count": sample_count,
        "RUN_DIR": run_dir.as_posix(),
        "interpretation": interpretation,
        "clean_success_contract": {
            "requires_clean_runtime_success": True,
            "requires_terminal_reached": True,
            "requires_execution_strategy": "split_follow_path_with_current_pose_gateway_handoff",
            "requires_sparse_fallback_used": False,
            "requires_wall_crossing_samples": True,
        },
        "controller_profile": payload.get("controller_profile"),
        "runtime_profile": payload.get("runtime_profile"),
    }
print(json.dumps(payload, indent=2, sort_keys=True))
(run_dir / "task11_demo_result_summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
SUMMARY_RC=$?

if [ "$RVIZ" = "1" ] && [ "$KEEP_OPEN_SEC" != "0" ]; then
  echo "[task11_demo] keeping RViz/overlays open for ${KEEP_OPEN_SEC}s"
  sleep "$KEEP_OPEN_SEC"
fi

if [ "$BRINGUP_RC" -ne 0 ]; then exit "$BRINGUP_RC"; fi
if [ "$ROUTE_RC" -ne 0 ]; then exit "$ROUTE_RC"; fi
exit "$SUMMARY_RC"
