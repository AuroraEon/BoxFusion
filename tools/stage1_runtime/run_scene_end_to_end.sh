#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SCENE_ID=""
FLOOR_ID="floor_2"
STAGE_OUTPUT_DIR=""
STAGE_A_OUTPUT_DIR=""
MAP_YAML=""
ROUTE_QUERY_JSON=""
WAYPOINTS_JSON=""
RUNTIME_PROFILE=""
OVERLAY_TOPIC=""
START_ROOM=""
GOAL_ROOM=""
THROUGH_ROOMS=""
TERMINAL_ROOM=""
RUN_ID="scene_runtime_check"
GUI=0
KEEP_GUI_OPEN_SEC=20
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"

usage() {
  cat <<'EOF'
Usage:
  run_scene_end_to_end.sh --scene-id ID --floor-id floor_2 --stage-output-dir DIR --map-yaml MAP --route-query-json ROUTE --waypoints-json WAYPOINTS --runtime-profile PROFILE --overlay-topic TOPIC --start-room room_X --goal-room room_Y [--through-rooms room_A,room_B] [--terminal-room room_Y]
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --scene-id) SCENE_ID="$2"; shift 2 ;;
    --floor-id) FLOOR_ID="$2"; shift 2 ;;
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --stage-a-output-dir) STAGE_A_OUTPUT_DIR="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --route-query-json) ROUTE_QUERY_JSON="$2"; shift 2 ;;
    --waypoints-json) WAYPOINTS_JSON="$2"; shift 2 ;;
    --runtime-profile) RUNTIME_PROFILE="$2"; shift 2 ;;
    --overlay-topic) OVERLAY_TOPIC="$2"; shift 2 ;;
    --start-room) START_ROOM="$2"; shift 2 ;;
    --goal-room) GOAL_ROOM="$2"; shift 2 ;;
    --through-rooms) THROUGH_ROOMS="$2"; shift 2 ;;
    --terminal-room) TERMINAL_ROOM="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --gui) GUI=1; shift ;;
    --headless|--no-gui) GUI=0; shift ;;
    --keep-gui-open-sec|--post-run-hold-sec) KEEP_GUI_OPEN_SEC="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[stage1_runtime][ERROR] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ -z "$TERMINAL_ROOM" ]; then TERMINAL_ROOM="$GOAL_ROOM"; fi
for required in SCENE_ID FLOOR_ID STAGE_OUTPUT_DIR MAP_YAML ROUTE_QUERY_JSON WAYPOINTS_JSON RUNTIME_PROFILE OVERLAY_TOPIC START_ROOM GOAL_ROOM TERMINAL_ROOM; do
  if [ -z "${!required}" ]; then echo "[stage1_runtime][ERROR] missing --${required,,}" >&2; usage >&2; exit 2; fi
done
if [[ "$STAGE_OUTPUT_DIR" != /* ]]; then STAGE_OUTPUT_DIR="$REPO_ROOT/$STAGE_OUTPUT_DIR"; fi
RUN_DIR="$STAGE_OUTPUT_DIR/runs/active/$RUN_ID"
mkdir -p "$RUN_DIR"
TRANSCRIPT="$RUN_DIR/command_transcript_v0_1.md"
: > "$TRANSCRIPT"

log_cmd() {
  local label="$1"; shift
  local log_file="$RUN_DIR/${label}.log"
  {
    echo ""
    echo "## $label"
    printf '```bash\n'
    printf '%q ' "$@"
    printf '\n```\n'
  } >> "$TRANSCRIPT"
  "$@" > "$log_file" 2>&1
  local rc=$?
  printf '\nExit code: `%s`\nLog: `%s`\n' "$rc" "$log_file" >> "$TRANSCRIPT"
  return "$rc"
}

if [ -n "$WAYPOINTS_JSON" ]; then
  WAYPOINTS_BASENAME="$(basename "$WAYPOINTS_JSON")"
  ROUTE_DIR="$(dirname "$WAYPOINTS_JSON")"
  EXECUTABLE_WAYPOINTS_JSON="$ROUTE_DIR/executable_route_waypoints_v0_1.json"
  EXECUTABLE_DIAGNOSTICS_JSON="$ROUTE_DIR/executable_route_diagnostics_v0_1.json"
  EXECUTABLE_REPORT_MD="$ROUTE_DIR/executable_route_report_v0_1.md"
  if [ "$WAYPOINTS_BASENAME" = "semantic_route_waypoints_v0_1.json" ]; then
    log_cmd executable_route_builder /usr/bin/python3 "$SCRIPT_DIR/build_scene_executable_route.py" \
      --stage-output-dir "$STAGE_OUTPUT_DIR" --floor-id "$FLOOR_ID" --map-yaml "$MAP_YAML" \
      --semantic-waypoints-json "$WAYPOINTS_JSON" --output-json "$EXECUTABLE_WAYPOINTS_JSON" \
      --diagnostics-json "$EXECUTABLE_DIAGNOSTICS_JSON" --output-md "$EXECUTABLE_REPORT_MD"
    BUILDER_RC=$?
    if [ "$BUILDER_RC" -ne 0 ]; then exit "$BUILDER_RC"; fi
    if [ -f "$EXECUTABLE_WAYPOINTS_JSON" ]; then
      WAYPOINTS_JSON="$EXECUTABLE_WAYPOINTS_JSON"
    fi
  elif [ -f "$EXECUTABLE_WAYPOINTS_JSON" ]; then
    WAYPOINTS_JSON="$EXECUTABLE_WAYPOINTS_JSON"
  fi
fi

MODE_ARG="--headless"
if [ "$GUI" = "1" ]; then MODE_ARG="--gui"; fi

log_cmd launch "$SCRIPT_DIR/launch_scene_gazebo_nav2.sh" \
  --scene-id "$SCENE_ID" --floor-id "$FLOOR_ID" --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --map-yaml "$MAP_YAML" --runtime-profile "$RUNTIME_PROFILE" --run-id "$RUN_ID" \
  --ros-domain-id "$ROS_DOMAIN_ID_VALUE" "$MODE_ARG" \
  --readiness-output-json "$RUN_DIR/lifecycle_readiness_report_v0_1.json" \
  --readiness-output-md "$RUN_DIR/lifecycle_readiness_report_v0_1.md"
BRINGUP_RC=$?
if [ "$BRINGUP_RC" -ne 0 ]; then exit "$BRINGUP_RC"; fi

log_cmd dataplane /usr/bin/python3 "$SCRIPT_DIR/probe_scene_dataplane.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" --timeout-sec 15 \
  --output-json "$RUN_DIR/dataplane_probe_result_v0_1.json" \
  --output-md "$RUN_DIR/dataplane_probe_result_v0_1.md"
DATAPLANE_RC=$?

setsid /usr/bin/python3 "$SCRIPT_DIR/publish_scene_rviz_overlay.py" \
  --scene-id "$SCENE_ID" --floor-id "$FLOOR_ID" --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --stage-a-output-dir "$STAGE_A_OUTPUT_DIR" --route-query-json "$ROUTE_QUERY_JSON" \
  --waypoints-json "$WAYPOINTS_JSON" --runtime-profile "$RUNTIME_PROFILE" \
  --overlay-topic "$OVERLAY_TOPIC" --run-id "$RUN_ID" > "$RUN_DIR/rviz_overlay_publisher.log" 2>&1 &
OVERLAY_PID=$!
sleep 4
RVIZ_PID=""
if [ "$GUI" = "1" ]; then
  RVIZ_CONFIG="$(/usr/bin/python3 - "$RUNTIME_PROFILE" <<'PY'
import json, sys
print(json.load(open(sys.argv[1], encoding="utf-8")).get("rviz_config", ""))
PY
)"
  if [ -n "$RVIZ_CONFIG" ]; then
    setsid rviz2 -d "$RVIZ_CONFIG" > "$RUN_DIR/rviz2.log" 2>&1 &
    RVIZ_PID=$!
    sleep 4
  fi
fi

EXPECTED_ROOMS="$(/usr/bin/python3 - "$ROUTE_QUERY_JSON" <<'PY'
import json, sys
d=json.load(open(sys.argv[1], encoding="utf-8"))
print(",".join(d.get("selected_route") or d.get("requested_route") or []))
PY
)"
EXPECTED_GATEWAYS="$(/usr/bin/python3 - "$WAYPOINTS_JSON" <<'PY'
import json, sys
d=json.load(open(sys.argv[1], encoding="utf-8"))
print(",".join([w["gateway_id"] for w in d.get("waypoints", []) if w.get("gateway_id")]))
PY
)"
ROBOT_MODEL_NAME="${SCENE_ID%%-*}_${FLOOR_ID/_/}_turtlebot3"
log_cmd route /usr/bin/python3 "$SCRIPT_DIR/run_scene_route.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" --waypoints-json "$WAYPOINTS_JSON" \
  --expected-room-chain "$EXPECTED_ROOMS" --expected-gateway-sequence "$EXPECTED_GATEWAYS" \
  --allow-non-scene-truth --reset-to-route-start --from-start --robot-model-name "$ROBOT_MODEL_NAME" \
  --output-json "$RUN_DIR/route_execution_result_v0_1.json" \
  --output-md "$RUN_DIR/route_execution_result_v0_1.md" \
  --trajectory-output-json "$RUN_DIR/trajectory_sample_result_v0_1.json" \
  --latest-slice-output-json "$RUN_DIR/latest_follow_path_slice_v0_1.json"
ROUTE_RC=$?

log_cmd physical /usr/bin/python3 "$SCRIPT_DIR/validate_scene_physical.py" \
  --scene-id "$SCENE_ID" --floor-id "$FLOOR_ID" --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --runtime-profile "$RUNTIME_PROFILE" --map-yaml "$MAP_YAML" \
  --route-query-json "$ROUTE_QUERY_JSON" --waypoints-json "$WAYPOINTS_JSON" \
  --route-execution-json "$RUN_DIR/route_execution_result_v0_1.json" \
  --trajectory-json "$RUN_DIR/trajectory_sample_result_v0_1.json" \
  --through-output-json "$RUN_DIR/through_room_physical_visit_validation_v0_1.json" \
  --through-output-md "$RUN_DIR/through_room_physical_visit_validation_v0_1.md" \
  --terminal-output-json "$RUN_DIR/terminal_quality_validation_v0_1.json" \
  --terminal-output-md "$RUN_DIR/terminal_quality_validation_v0_1.md" \
  --wall-output-json "$RUN_DIR/trajectory_wall_crossing_validation_v0_1.json" \
  --wall-output-md "$RUN_DIR/trajectory_wall_crossing_validation_v0_1.md" \
  --spin-output-json "$RUN_DIR/local_looping_spin_validation_v0_1.json" \
  --spin-output-md "$RUN_DIR/local_looping_spin_validation_v0_1.md" \
  --through-rooms "$THROUGH_ROOMS" --terminal-room "$TERMINAL_ROOM"
PHYSICAL_RC=$?

if [ "$GUI" = "1" ]; then sleep "$KEEP_GUI_OPEN_SEC"; fi
kill "$OVERLAY_PID" >/dev/null 2>&1 || true
if [ -n "$RVIZ_PID" ]; then kill "$RVIZ_PID" >/dev/null 2>&1 || true; fi

/usr/bin/python3 - "$RUN_DIR/run_summary.json" "$BRINGUP_RC" "$DATAPLANE_RC" "$ROUTE_RC" "$PHYSICAL_RC" <<'PY'
import json, sys
out, bringup, dataplane, route, physical = sys.argv[1:]
payload = {
  "artifact_type": "scene_runtime_chain_summary",
  "return_codes": {"bringup": int(bringup), "dataplane": int(dataplane), "route": int(route), "physical": int(physical)},
}
payload["succeeded"] = all(v == 0 for v in payload["return_codes"].values())
open(out, "w", encoding="utf-8").write(json.dumps(payload, indent=2, sort_keys=True)+"\n")
sys.exit(0 if payload["succeeded"] else 1)
PY
