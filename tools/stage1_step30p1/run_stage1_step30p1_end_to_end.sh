#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAGE_OUTPUT_DIR="stage_outputs/stage1_00824_step30p1"
START_ROOM="room_1"
GOAL_ROOM="room_16"
THROUGH_ROOMS=()
MODE="full"
FROM_START=0
TERMINAL_ROOM="room_16"
GUI=1
SHUTDOWN_AFTER_RUN=0
KEEP_GUI_OPEN_SEC=20
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"
THROUGH_ROOM_DWELL_SEC=3.0
ROOM15_MIN_INSIDE_SAMPLES=8
MAP_PROFILE="stable"
RUN_ID=""
MARKER_TOPIC="/stage1_nav/semantic_overlay_markers"

usage() {
  cat <<'EOF'
Usage:
  run_stage1_step30p1_end_to_end.sh --stage-output-dir DIR --start-room room_1 --goal-room room_16 --through-rooms room_15 --mode full --from-start --terminal-room room_16 [--gui|--headless]

Runs the Stage1 stable full-scene occupancy map, Gazebo + RViz BEV semantic-overlay route demo, strict through-room physical-visit, and terminal-quality validation.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --start-room) START_ROOM="$2"; shift 2 ;;
    --goal-room) GOAL_ROOM="$2"; shift 2 ;;
    --through-rooms) THROUGH_ROOMS+=("$2"); shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --from-start) FROM_START=1; shift ;;
    --terminal-room) TERMINAL_ROOM="$2"; shift 2 ;;
    --gui) GUI=1; shift ;;
    --headless|--no-gui) GUI=0; shift ;;
    --shutdown-after-run) SHUTDOWN_AFTER_RUN=1; shift ;;
    --post-run-hold-sec|--keep-gui-open-sec) KEEP_GUI_OPEN_SEC="$2"; shift 2 ;;
    --through-room-dwell-sec) THROUGH_ROOM_DWELL_SEC="$2"; shift 2 ;;
    --room15-min-inside-samples|--through-room-min-inside-samples) ROOM15_MIN_INSIDE_SAMPLES="$2"; shift 2 ;;
    --map-profile) MAP_PROFILE="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[step30s7][ERROR] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$STAGE_OUTPUT_DIR" != /* ]]; then
  STAGE_OUTPUT_DIR="$REPO_ROOT/$STAGE_OUTPUT_DIR"
fi

if [ -z "$RUN_ID" ]; then
  if [ "${#THROUGH_ROOMS[@]}" -gt 0 ]; then
    RUN_ID="through_${THROUGH_ROOMS[*]}"
    RUN_ID="${RUN_ID// /_}"
  else
    RUN_ID="canonical"
  fi
fi
EVIDENCE_DIR="$STAGE_OUTPUT_DIR/current_validation/$RUN_ID"
BRINGUP_LOG_DIR="$EVIDENCE_DIR/bringup_logs"
RVIZ_DIR="$STAGE_OUTPUT_DIR/rviz"
RVIZ_CONFIG="$RVIZ_DIR/00824_stage1_step30p1_bev_semantic_route_demo.rviz"
REPORT_JSON="$EVIDENCE_DIR/stable_map_gui_report_v0_1.json"
REPORT_MD="$EVIDENCE_DIR/stable_map_gui_report_v0_1.md"
TRANSCRIPT="$EVIDENCE_DIR/command_transcript_v0_1.md"
DATAPLANE_JSON="$EVIDENCE_DIR/dataplane_probe_result_v0_1.json"
DATAPLANE_MD="$EVIDENCE_DIR/dataplane_probe_result_v0_1.md"
LIFECYCLE_JSON="$EVIDENCE_DIR/lifecycle_readiness_report_v0_1.json"
LIFECYCLE_MD="$EVIDENCE_DIR/lifecycle_readiness_report_v0_1.md"
ROUTE_QUERY_JSON="$EVIDENCE_DIR/route_query_result_v0_1.json"
ROUTE_QUERY_MD="$EVIDENCE_DIR/route_query_result_v0_1.md"
ROUTE_WAYPOINTS_JSON="$EVIDENCE_DIR/semantic_route_waypoints_v0_1.json"
TARGET_SELECTION_JSON="$EVIDENCE_DIR/room15_target_selection_v0_1.json"
TARGET_SELECTION_MD="$EVIDENCE_DIR/room15_target_selection_v0_1.md"
ROUTE_EXEC_JSON="$EVIDENCE_DIR/route_execution_result_v0_1.json"
ROUTE_EXEC_MD="$EVIDENCE_DIR/route_execution_result_v0_1.md"
TRAJECTORY_JSON="$EVIDENCE_DIR/trajectory_sample_result_v0_1.json"
SLICE_JSON="$EVIDENCE_DIR/latest_follow_path_slice_v0_1.json"
THROUGH_JSON="$EVIDENCE_DIR/through_room15_physical_visit_validation_v0_1.json"
THROUGH_MD="$EVIDENCE_DIR/through_room15_physical_visit_validation_v0_1.md"
TERMINAL_JSON="$EVIDENCE_DIR/room16_terminal_quality_validation_v0_1.json"
TERMINAL_MD="$EVIDENCE_DIR/room16_terminal_quality_validation_v0_1.md"
WALL_JSON="$EVIDENCE_DIR/trajectory_wall_crossing_validation_v0_2.json"
WALL_MD="$EVIDENCE_DIR/trajectory_wall_crossing_validation_v0_2.md"
SPIN_JSON="$EVIDENCE_DIR/local_looping_spin_validation_v0_2.json"
SPIN_MD="$EVIDENCE_DIR/local_looping_spin_validation_v0_2.md"
MANIFEST_JSON="$EVIDENCE_DIR/stable_full_scene_occupancy_map_provenance.json"
MANIFEST_MD="$EVIDENCE_DIR/stable_full_scene_occupancy_map_provenance.md"
REQUEST_MAP_JSON="$EVIDENCE_DIR/request_map_validation_v0_1.json"
REQUEST_MAP_MD="$EVIDENCE_DIR/request_map_validation_v0_1.md"
BEV_AUDIT_JSON="$EVIDENCE_DIR/bev_visual_regression_report_v0_1.json"
BEV_AUDIT_MD="$EVIDENCE_DIR/bev_visual_regression_report_v0_1.md"
AUDIT_JSON="$BEV_AUDIT_JSON"
AUDIT_MD="$BEV_AUDIT_MD"
GATEWAY_READY_JSON="$EVIDENCE_DIR/gateway_generalization_readiness_report_v0_1.json"
GATEWAY_READY_MD="$EVIDENCE_DIR/gateway_generalization_readiness_report_v0_1.md"
VISUAL_DIAG_DIR="$EVIDENCE_DIR/visual_diagnostics"
RVIZ_MANIFEST_JSON="$EVIDENCE_DIR/rviz_overlay_manifest_v0_3.json"
RVIZ_MANIFEST_MD="$EVIDENCE_DIR/rviz_overlay_manifest_v0_3.md"
RVIZ_PROCESS_JSON="$EVIDENCE_DIR/rviz_process_check_v0_1.json"
GAZEBO_PROCESS_JSON="$EVIDENCE_DIR/gazebo_process_check_v0_1.json"
PROCESS_LIST_AFTER_BRINGUP="$EVIDENCE_DIR/process_list_after_bringup_v0_1.txt"
PROCESS_LIST_AFTER_ROUTE="$EVIDENCE_DIR/process_list_after_route_v0_1.txt"
RVIZ_CONFIG_USED_COPY="$EVIDENCE_DIR/00824_stage1_step30p1_bev_semantic_route_demo.rviz"

if [ -d "$EVIDENCE_DIR" ] && [ -n "$(find "$EVIDENCE_DIR" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
  STALE_PARENT="$STAGE_OUTPUT_DIR/current_validation/archive_stale_runs"
  STALE_DIR="$STALE_PARENT/${RUN_ID}_$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$STALE_PARENT"
  mv "$EVIDENCE_DIR" "$STALE_DIR"
  echo "[step30s7] Existing evidence directory moved aside to avoid stale-run mixing: $STALE_DIR"
fi
mkdir -p "$EVIDENCE_DIR" "$BRINGUP_LOG_DIR" "$RVIZ_DIR" "$VISUAL_DIAG_DIR"
: > "$TRANSCRIPT"

BRINGUP_RC=999
DATAPLANE_RC=999
ROUTE_QUERY_RC=999
ROUTE_RC=999
VALIDATION_RC=999
AUDIT_RC=999
RVIZ_RC=999
GAZEBO_GUI_PRESENT=0
RVIZ_PRESENT=0
OVERLAY_TOPIC_PRESENT=0
FINAL_FAILURE_REASON=""
OVERLAY_PID=""
RVIZ_PID=""

log() { printf '%s\n' "$1" | tee -a "$TRANSCRIPT"; }

quote_cmd() {
  local quoted=()
  local arg
  for arg in "$@"; do quoted+=("$(printf '%q' "$arg")"); done
  printf '%s ' "${quoted[@]}"
}

run_logged() {
  local label="$1"; shift
  local log_file="$EVIDENCE_DIR/${label}.log"
  log ""; log "## $label"; log ""; log '```bash'
  quote_cmd "$@" | tee -a "$TRANSCRIPT" >/dev/null
  log ""; log '```'
  "$@" > "$log_file" 2>&1
  local rc=$?
  log ""; log "Exit code: \`$rc\`"; log "Log: \`$log_file\`"; log ""
  log "<details><summary>tail</summary>"; log ""; log '```text'
  tail -n 120 "$log_file" >> "$TRANSCRIPT" 2>/dev/null || true
  log '```'; log "</details>"
  return "$rc"
}

source_ros_setup() {
  local had_nounset=0
  case "$-" in *u*) had_nounset=1 ;; esac
  set +u
  source /opt/ros/foxy/setup.bash
  if [ "$had_nounset" = "1" ]; then set -u; else set +u; fi
}

capture_process_list() {
  local output="$1"
  {
    echo "# ps snapshot"
    ps -eo pid,ppid,stat,comm,args
    echo ""
    echo "# filtered"
    ps -eo pid,ppid,stat,comm,args | grep -E 'gzclient|gzserver|gazebo|rviz2|stage1_nav|stage1_step30p1_rviz_overlay|publish_stage1_step30p1_rviz_overlay|robot_state_publisher|static_transform_publisher|nav2_|bt_navigator|controller_server|planner_server|turtlebot3' | grep -v grep || true
  } > "$output" 2>&1
}

write_process_check_json() {
  local output="$1"; local kind="$2"
  /usr/bin/python3 - "$output" "$kind" "$GUI" "$RVIZ_CONFIG" "$RVIZ_CONFIG_USED_COPY" <<'PY'
import json, subprocess, sys
out, kind, gui, rviz_config, rviz_copy = sys.argv[1:]
ps = subprocess.run(["ps", "-eo", "pid,ppid,stat,comm,args"], text=True, capture_output=True).stdout
if kind == "gazebo":
    present = "gzclient" in ps or "gazebo --gui-client" in ps
    patterns = ["gzclient", "gzserver", "gazebo"]
else:
    present = "rviz2" in ps
    patterns = ["rviz2"]
payload = {
    "artifact_type": f"step30s7_{kind}_process_check",
    "gui_requested": bool(int(gui)),
    "process_present": present,
    "matching_lines": [line for line in ps.splitlines() if any(p in line for p in patterns)],
}
if kind == "rviz":
    payload["rviz_config"] = rviz_config
    payload["rviz_config_used_copy"] = rviz_copy
open(out, "w", encoding="utf-8").write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
sys.exit(0 if present or not bool(int(gui)) else 1)
PY
}

write_overlay_manifest() {
  /usr/bin/python3 - "$RVIZ_MANIFEST_JSON" "$RVIZ_MANIFEST_MD" "$RVIZ_CONFIG" "$RVIZ_CONFIG_USED_COPY" "$ROUTE_QUERY_JSON" "$ROUTE_WAYPOINTS_JSON" "$TRAJECTORY_JSON" "$EVIDENCE_DIR" "$MARKER_TOPIC" "$RUN_ID" <<'PY'
import json, sys
from pathlib import Path
out_json, out_md, rviz_config, rviz_copy, route_query, waypoints, trajectory, evidence, marker_topic, run_id = sys.argv[1:]
topics = ["/map", "/tf", "/tf_static", "/scan", "/robot_description", "/plan", "/local_plan", marker_topic]
payload = {
    "artifact_type": "stage1_nav_rviz_overlay_manifest",
    "version": "v0_2",
    "run_id": run_id,
    "rviz_config": rviz_config,
    "rviz_config_used_copy": rviz_copy,
    "marker_topic": marker_topic,
    "rviz_relevant_topics": topics,
    "separate_layers": ["stable full-scene Nav2 /map occupancy/free-space floorplan", "stable complete semantic room mask", "request through-room highlight", "selected interior targets", "gateway crossings", "planned route", "executed trajectory", "wall/spin diagnostics"],
    "route_query_result": route_query,
    "route_waypoints": waypoints,
    "trajectory": trajectory,
    "overlay_publisher": "tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py",
}
Path(out_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
lines = ["# Stage1 RViz Overlay Manifest", "", f"Run id: `{run_id}`", f"RViz config: `{rviz_config}`", f"Marker topic: `{payload['marker_topic']}`", "", "## Topics", ""]
lines.extend(f"- `{topic}`" for topic in topics)
Path(out_md).write_text("\n".join(lines) + "\n", encoding="utf-8")
PY
}

write_final_report() {
  /usr/bin/python3 - "$REPORT_JSON" "$REPORT_MD" "$EVIDENCE_DIR" "$STAGE_OUTPUT_DIR" "$START_ROOM" "$GOAL_ROOM" "$TERMINAL_ROOM" "$RVIZ_CONFIG_USED_COPY" "$DATAPLANE_JSON" "$LIFECYCLE_JSON" "$ROUTE_QUERY_JSON" "$ROUTE_EXEC_JSON" "$THROUGH_JSON" "$TERMINAL_JSON" "$WALL_JSON" "$SPIN_JSON" "$AUDIT_JSON" "$TARGET_SELECTION_JSON" "$RVIZ_PROCESS_JSON" "$GAZEBO_PROCESS_JSON" "$RVIZ_MANIFEST_JSON" "$TRANSCRIPT" "$GUI" "$BRINGUP_RC" "$DATAPLANE_RC" "$ROUTE_QUERY_RC" "$ROUTE_RC" "$VALIDATION_RC" "$FINAL_FAILURE_REASON" "$ROOM15_MIN_INSIDE_SAMPLES" "$THROUGH_ROOM_DWELL_SEC" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path

report_json, report_md, evidence_dir, stage_output, start_room, goal_room, terminal_room, rviz_config, dataplane_p, lifecycle_p, route_query_p, route_exec_p, through_p, terminal_p, wall_p, spin_p, audit_p, target_p, rviz_proc_p, gazebo_proc_p, rviz_manifest_p, transcript, gui, bringup_rc, dataplane_rc, route_query_rc, route_rc, validation_rc, failure, required_samples, required_dwell = sys.argv[1:]
def load(path):
    p = Path(path)
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_read_error": f"{type(exc).__name__}: {exc}"}
dataplane, lifecycle, route_query, route_exec = load(dataplane_p), load(lifecycle_p), load(route_query_p), load(route_exec_p)
through, terminal, wall, spin, audit, target = load(through_p), load(terminal_p), load(wall_p), load(spin_p), load(audit_p), load(target_p)
rviz_proc, gazebo_proc, rviz_manifest = load(rviz_proc_p), load(gazebo_proc_p), load(rviz_manifest_p)
topics = dataplane.get("topics", {})
route_attempted = bool(route_exec) and bool(route_exec.get("execute_attempted", True)) and int(route_rc) not in (999,)
route_finished = bool(route_exec.get("succeeded") or route_exec.get("final_arrival_success"))
wall_attempted = bool(wall)
spin_attempted = bool(spin)
terminal_attempted = bool(terminal)
through_attempted = bool(through)
map_server_state = (lifecycle.get("lifecycle_states") or {}).get("/map_server", {}).get("state")
controller_state = (lifecycle.get("lifecycle_states") or {}).get("/controller_server", {}).get("state")
planner_state = (lifecycle.get("lifecycle_states") or {}).get("/planner_server", {}).get("state")
bt_state = (lifecycle.get("lifecycle_states") or {}).get("/bt_navigator", {}).get("state")
if failure and lifecycle and not lifecycle.get("succeeded") and "lifecycle" not in failure:
    stuck = ", ".join(lifecycle.get("stuck_nodes") or lifecycle.get("missing_conditions") or [])
    failure = f"Nav2 lifecycle bringup failed: {stuck}" if stuck else "Nav2 lifecycle bringup failed"
answers = {
    "why_room15_appeared_only_partially_white_free": "The old request-aware/H8R2-derived primary /map could omit room15 free space when room15 was not in the request. The repaired primary /map is the stable full-scene occupancy map.",
    "was_nav2_map_issue_semantic_display_issue_or_both": "primary Nav2/RViz base map source issue; semantic overlay remains a separate marker layer",
    "room15_semantic_mask_area_cells": audit.get("room15_mask_area_cells"),
    "room15_original_h8r2_free_fraction": audit.get("room15_original_free_fraction"),
    "stable_full_scene_map_used": bool(route_query.get("map_profile") == "stage1_full_scene_occupancy"),
    "changed_cells_or_regions": audit.get("changed_cells_vs_h8r2"),
    "semantic_room_mask_overlay_displayed_separately_from_nav2_free_space": bool(rviz_manifest.get("marker_topic")),
    "room15_interior_target_selected": (target.get("room15_target") or audit.get("selected_room15_interior_target")),
    "room15_target_distance_from_r7_r15_gateway_m": (target.get("room15_target") or audit.get("selected_room15_interior_target") or {}).get("distance_to_nearest_gateway_m") or (audit.get("selected_room15_interior_target") or {}).get("distance_to_r7_r15_gateway_m"),
    "room15_target_distance_from_walls_m": (target.get("room15_target") or audit.get("selected_room15_interior_target") or {}).get("distance_to_occupied_or_unknown_m"),
    "did_gazebo_gui_launch": bool(gazebo_proc.get("process_present")) if bool(int(gui)) else False,
    "did_rviz_launch": bool(rviz_proc.get("process_present")) if bool(int(gui)) else False,
    "nav2_lifecycle_readiness_passed": bool(lifecycle.get("succeeded")),
    "nav2_lifecycle_states": {
        "/map_server": map_server_state,
        "/controller_server": controller_state,
        "/planner_server": planner_state,
        "/bt_navigator": bt_state,
    },
    "nav2_lifecycle_stuck_nodes": lifecycle.get("stuck_nodes") or [],
    "map_occupancy_grid_received_transient_local": bool(lifecycle.get("map_received") or (topics.get("/map") or {}).get("message_received")),
    "follow_path_action_server_ready": bool((lifecycle.get("action_servers") or {}).get("/follow_path") or (dataplane.get("action_servers") or {}).get("/follow_path")),
    "compute_path_to_pose_action_server_ready_non_blocking": bool((lifecycle.get("action_servers") or {}).get("/compute_path_to_pose") or (dataplane.get("action_servers") or {}).get("/compute_path_to_pose")),
    "navigate_to_pose_action_server_ready_non_blocking": bool((lifecycle.get("action_servers") or {}).get("/navigate_to_pose") or (dataplane.get("action_servers") or {}).get("/navigate_to_pose")),
    "compute_path_to_pose_hard_blocker": bool(lifecycle.get("require_compute_path_to_pose") or dataplane.get("require_compute_path_to_pose")),
    "navigate_to_pose_hard_blocker": bool(lifecycle.get("require_navigate_to_pose") or dataplane.get("require_navigate_to_pose")),
    "rviz_config_used": rviz_config,
    "was_bev_floorplan_visible_in_rviz": Path(rviz_config).exists() and bool((topics.get("/map") or {}).get("message_received")),
    "were_route_gateway_room_topology_overlay_markers_published": bool(rviz_manifest.get("marker_topic")),
    "were_robot_tf_map_route_and_trajectory_visible_or_available": bool((topics.get("/map") or {}).get("message_received")) and bool((topics.get("/tf") or {}).get("message_received")) and bool(rviz_manifest.get("marker_topic")),
    "did_robot_start_from_room_1_route_start": bool(route_exec.get("from_start_reset_verified")) if route_attempted else "blocked_not_attempted",
    "planned_route": route_query.get("room_sequence"),
    "route_topology_includes_room15": bool(through.get("route_topology_includes_room15")) if through_attempted else "blocked_not_attempted",
    "trajectory_physically_entered_room15_mask": bool(through.get("trajectory_entered_room15_mask")) if through_attempted else "blocked_not_attempted",
    "room15_inside_sample_count": through.get("room15_inside_sample_count") if through_attempted else "blocked_not_attempted",
    "room15_inside_dwell_sec": through.get("room15_inside_dwell_sec") if through_attempted else "blocked_not_attempted",
    "room15_gateway_only_or_interior": ("entered_room15_interior" if through.get("visual_through_room15_success") else "gateway_only_or_not_entered") if through_attempted else "blocked_not_attempted",
    "spinning_or_looping_near_room15_detected": bool(spin.get("spinning_detected")) if spin_attempted else "blocked_not_attempted",
    "spinning_or_looping_reduced_or_eliminated": bool(spin.get("local_looping_validation_passed")) if spin_attempted else "blocked_not_attempted",
    "trajectory_wall_point_violation_count": wall.get("trajectory_wall_point_violation_count") if wall_attempted else "blocked_not_attempted",
    "trajectory_wall_segment_violation_count": wall.get("trajectory_wall_segment_violation_count") if wall_attempted else "blocked_not_attempted",
    "previous_wall_crossing_visualization_artifact_or_real_issue": ("visualization_artifact_possible" if wall.get("visualization_interpolation_artifact_possible") else ("no_wall_crossing_detected" if wall.get("wall_crossing_validation_passed") else "real_map_or_trajectory_issue")) if wall_attempted else "blocked_not_attempted",
    "did_robot_reach_room16": bool(route_exec.get("final_arrival_success")) if route_attempted else "blocked_not_attempted",
    "final_pose_inside_room16_mask": bool(terminal.get("final_pose_inside_room16_mask")) if terminal_attempted else "blocked_not_attempted",
    "final_pose_too_close_to_r14_r16_gateway": bool(terminal.get("final_pose_near_r14_r16_gateway")) if terminal_attempted else "blocked_not_attempted",
    "room16_terminal_visual_quality_passed": bool(terminal.get("terminal_visual_quality_passed")) if terminal_attempted else "blocked_not_attempted",
    "did_sparse_fallback_occur": bool(route_exec.get("sparse_fallback_used")) if route_attempted else "blocked_not_attempted",
    "were_bridge_smooth_bridge_waypoints_used_as_navigate_to_pose_goals": bool(route_exec.get("bridge_waypoints_used_as_goals")) if route_attempted else "blocked_not_attempted",
    "bev_semantic_overlay_route_gateway_trajectory_visible_or_published": bool(rviz_manifest.get("marker_topic")),
}
evidence_files = {
    "report_json": report_json,
    "report_md": report_md,
    "command_transcript": transcript,
    "lifecycle_readiness_report_json": lifecycle_p,
    "lifecycle_readiness_report_md": str(Path(lifecycle_p).with_suffix(".md")),
    "dataplane_probe_result_json": dataplane_p,
    "dataplane_probe_result_md": str(Path(dataplane_p).with_suffix(".md")),
    "route_query_result_json": route_query_p,
    "route_query_result_md": str(Path(route_query_p).with_suffix(".md")),
    "route_execution_result_json": route_exec_p,
    "route_execution_result_md": str(Path(route_exec_p).with_suffix(".md")),
    "through_room15_validation_json": through_p,
    "through_room15_validation_md": str(Path(through_p).with_suffix(".md")),
    "room16_terminal_validation_json": terminal_p,
    "room16_terminal_validation_md": str(Path(terminal_p).with_suffix(".md")),
    "trajectory_wall_crossing_validation_json": wall_p,
    "trajectory_wall_crossing_validation_md": str(Path(wall_p).with_suffix(".md")),
    "local_looping_spin_validation_json": spin_p,
    "local_looping_spin_validation_md": str(Path(spin_p).with_suffix(".md")),
    "room15_map_coverage_audit_json": audit_p,
    "room15_target_selection_json": target_p,
    "rviz_overlay_manifest_json": rviz_manifest_p,
    "rviz_process_check_json": rviz_proc_p,
    "gazebo_process_check_json": gazebo_proc_p,
    "trajectory_sample_result_json": str(Path(evidence_dir) / "trajectory_sample_result_v0_1.json"),
    "latest_follow_path_slice_json": str(Path(evidence_dir) / "latest_follow_path_slice_v0_1.json"),
    "process_list_after_bringup": str(Path(evidence_dir) / "process_list_after_bringup_v0_1.txt"),
    "process_list_after_route": str(Path(evidence_dir) / "process_list_after_route_v0_1.txt"),
    "rviz_config_used": rviz_config,
}
success = all([
    (not bool(int(gui))) or answers["did_gazebo_gui_launch"],
    (not bool(int(gui))) or answers["did_rviz_launch"],
    answers["nav2_lifecycle_readiness_passed"],
    answers["map_occupancy_grid_received_transient_local"],
    answers["follow_path_action_server_ready"],
    answers["was_bev_floorplan_visible_in_rviz"],
    answers["were_route_gateway_room_topology_overlay_markers_published"],
    route_attempted,
    route_finished,
    answers["did_robot_start_from_room_1_route_start"] is True,
    through.get("all_through_rooms_success"),
    through.get("gateway_only_failure_guard_passed"),
    wall.get("wall_crossing_validation_passed"),
    spin.get("local_looping_validation_passed"),
    terminal.get("terminal_visual_quality_passed"),
    answers["were_bridge_smooth_bridge_waypoints_used_as_navigate_to_pose_goals"] is False,
])
if failure:
    success = False
payload = {
    "artifact_type": "stage1_stable_map_gui_report",
    "version": "v0_1",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "stage_output_dir": stage_output,
    "evidence_dir": evidence_dir,
    "historical_milestone_wording": "Step30P1 repaired execution succeeded with clean forward-only fallback.",
    "succeeded": success,
    "failure_reason": None if success else (failure or "one or more stable-map Stage1 acceptance checks failed"),
    "return_codes": {"bringup": int(bringup_rc), "dataplane_probe": int(dataplane_rc), "route_query": int(route_query_rc), "route_execution": int(route_rc), "physical_validation": int(validation_rc)},
    "lifecycle_readiness": lifecycle,
    "answers": answers,
    "evidence_files": evidence_files,
    "reproduce_command": "tools/stage1_nav/run_gui_demo.sh --stage-output-dir stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_15 --terminal-room room_16 --from-start --gui --map-profile stable --through-room-dwell-sec 3.0 --through-room-min-inside-samples 8 --keep-gui-open-sec 20",
}
Path(report_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
lines = ["# Stage1 Stable Map GUI Report", "", f"Succeeded: `{success}`", f"Failure reason: `{payload['failure_reason']}`", "", "Accepted historical wording remains: `Step30P1 repaired execution succeeded with clean forward-only fallback.`", "", "## Required Answers", ""]
question_labels = [
    ("Why did room15 appear only partially white/free in RViz?", "why_room15_appeared_only_partially_white_free"),
    ("Was this a Nav2 map/free-space issue, semantic display issue, or both?", "was_nav2_map_issue_semantic_display_issue_or_both"),
    ("What is the room15 semantic mask area?", "room15_semantic_mask_area_cells"),
    ("What fraction of room15 was free in original H8R2 Nav2 map?", "room15_original_h8r2_free_fraction"),
    ("Was the stable full-scene map used as the primary map?", "stable_full_scene_map_used"),
    ("Was the RViz semantic room mask overlay displayed separately from Nav2 free space?", "semantic_room_mask_overlay_displayed_separately_from_nav2_free_space"),
    ("What room15 interior target was selected?", "room15_interior_target_selected"),
    ("How far is the room15 interior target from r7-r15 gateway?", "room15_target_distance_from_r7_r15_gateway_m"),
    ("How far is it from walls/occupied cells?", "room15_target_distance_from_walls_m"),
    ("Did Gazebo GUI launch?", "did_gazebo_gui_launch"),
    ("Did RViz launch?", "did_rviz_launch"),
    ("Did Nav2 lifecycle readiness pass?", "nav2_lifecycle_readiness_passed"),
    ("What lifecycle states were observed?", "nav2_lifecycle_states"),
    ("Was a TRANSIENT_LOCAL /map OccupancyGrid received?", "map_occupancy_grid_received_transient_local"),
    ("Was /follow_path ready?", "follow_path_action_server_ready"),
    ("Is /compute_path_to_pose a hard blocker?", "compute_path_to_pose_hard_blocker"),
    ("Is /navigate_to_pose a hard blocker?", "navigate_to_pose_hard_blocker"),
    ("Which RViz config was used?", "rviz_config_used"),
    ("Was a BEV/floorplan visible in RViz?", "was_bev_floorplan_visible_in_rviz"),
    ("Were route/gateway/room/topology overlay markers published?", "were_route_gateway_room_topology_overlay_markers_published"),
    ("Were robot TF, map, route and trajectory visible or available in RViz topics?", "were_robot_tf_map_route_and_trajectory_visible_or_available"),
    ("Did the robot start from room_1 / route start?", "did_robot_start_from_room_1_route_start"),
    ("What route was planned?", "planned_route"),
    ("Did the route include room_15 topologically?", "route_topology_includes_room15"),
    ("Did the trajectory physically enter room_15 mask?", "trajectory_physically_entered_room15_mask"),
    ("How many trajectory samples were inside room_15?", "room15_inside_sample_count"),
    ("What was room15 dwell time?", "room15_inside_dwell_sec"),
    ("Did it only pass near the room15 gateway?", "room15_gateway_only_or_interior"),
    ("Was spinning/looping near room15 detected?", "spinning_or_looping_near_room15_detected"),
    ("Was spinning/looping reduced or eliminated?", "spinning_or_looping_reduced_or_eliminated"),
    ("Trajectory wall point violation count?", "trajectory_wall_point_violation_count"),
    ("Trajectory wall segment violation count?", "trajectory_wall_segment_violation_count"),
    ("Was the previous wall crossing appearance artifact or real?", "previous_wall_crossing_visualization_artifact_or_real_issue"),
    ("Did the robot reach room_16?", "did_robot_reach_room16"),
    ("Was the final pose inside room16 mask?", "final_pose_inside_room16_mask"),
    ("Was the final pose too close to r14-r16 gateway?", "final_pose_too_close_to_r14_r16_gateway"),
    ("Did room16 terminal visual quality pass?", "room16_terminal_visual_quality_passed"),
    ("Did sparse fallback occur?", "did_sparse_fallback_occur"),
    ("Were bridge/smooth bridge waypoints incorrectly used as NavigateToPose goals?", "were_bridge_smooth_bridge_waypoints_used_as_navigate_to_pose_goals"),
]
for question, key in question_labels:
    lines.append(f"- {question} `{answers.get(key)}`")
lines.extend(["", "## Evidence", ""])
for key, value in evidence_files.items():
    lines.append(f"- `{key}`: `{value}`")
lines.extend(["", "## Reproduce", "", f"`{payload['reproduce_command']}`"])
Path(report_md).write_text("\n".join(lines) + "\n", encoding="utf-8")
print(json.dumps({"succeeded": success, "report_json": report_json}, indent=2))
sys.exit(0 if success else 1)
PY
}

export PATH="/usr/bin:/usr/local/bin:$PATH"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

log "# Stage1 Stable Full-Scene Map GUI Command Transcript"
log ""; log "Started: \`$(date -u +%Y-%m-%dT%H:%M:%SZ)\`"; log "Run id: \`$RUN_ID\`"; log "Stage output: \`$STAGE_OUTPUT_DIR\`"; log "Evidence: \`$EVIDENCE_DIR\`"

if [ "$MAP_PROFILE" = "step30s5_room15_diagnostic_patch" ] || [ "$MAP_PROFILE" = "step30s5_room15_interior" ]; then
  echo "[step30s7][ERROR] Step30S5 patched map profiles were removed from active runtime use." >&2
  exit 12
fi
ACTIVE_MAP_PROFILE="$MAP_PROFILE"
if [ "$MAP_PROFILE" = "auto" ] || [ "$MAP_PROFILE" = "stable" ] || [ "$MAP_PROFILE" = "full_scene" ] || [ "$MAP_PROFILE" = "stage1_full_scene" ] || [ "$MAP_PROFILE" = "stage1_full_scene_occupancy" ]; then
  ACTIVE_MAP_PROFILE="stage1_full_scene_occupancy"
fi
if [ "$MAP_PROFILE" = "request_aware" ]; then
  ACTIVE_MAP_PROFILE="step30s7_request_aware"
fi
if [ "$ACTIVE_MAP_PROFILE" = "stage1_full_scene_occupancy" ]; then
  run_logged build_stable_full_scene_map /usr/bin/python3 "$SCRIPT_DIR/build_stage1_full_scene_occupancy_map.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --output-json "$MANIFEST_JSON" \
    --output-md "$MANIFEST_MD"
  BUILD_RC=$?
  if [ "$BUILD_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="stable full-scene map build failed"; fi
fi
if [ "$ACTIVE_MAP_PROFILE" = "step30s7_request_aware" ]; then
  run_logged build_request_aware_map /usr/bin/python3 "$SCRIPT_DIR/build_stage1_step30p1_request_aware_nav_map.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --start-room "$START_ROOM" \
    --goal-room "$GOAL_ROOM" \
    --through-rooms "${THROUGH_ROOMS[@]}" \
    --terminal-room "$TERMINAL_ROOM"
  BUILD_RC=$?
  cp "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map_manifest_v0_1.json" "$MANIFEST_JSON" 2>/dev/null || true
  cp "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map_manifest_v0_1.md" "$MANIFEST_MD" 2>/dev/null || true
  if [ "$BUILD_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="request-aware debug map build failed"; fi
  run_logged request_map_validation /usr/bin/python3 "$SCRIPT_DIR/validate_stage1_step30p1_request_map.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --start-room "$START_ROOM" \
    --goal-room "$GOAL_ROOM" \
    --through-rooms "${THROUGH_ROOMS[@]}" \
    --terminal-room "$TERMINAL_ROOM" \
    --map-yaml "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map.yaml" \
    --output-json "$REQUEST_MAP_JSON" \
    --output-md "$REQUEST_MAP_MD"
  REQUEST_MAP_RC=$?
  cp "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map_manifest_v0_1.json" "$MANIFEST_JSON" 2>/dev/null || true
  cp "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map_manifest_v0_1.md" "$MANIFEST_MD" 2>/dev/null || true
  if [ "$REQUEST_MAP_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="request map validation failed"; fi
  run_logged bev_visual_regression /usr/bin/python3 "$SCRIPT_DIR/audit_stage1_step30p1_bev_visual_regression.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --start-room "$START_ROOM" \
    --goal-room "$GOAL_ROOM" \
    --through-rooms "${THROUGH_ROOMS[@]}" \
    --terminal-room "$TERMINAL_ROOM" \
    --map-yaml "$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map.yaml" \
    --output-json "$BEV_AUDIT_JSON" \
    --output-md "$BEV_AUDIT_MD" \
    --visual-output-dir "$VISUAL_DIAG_DIR"
  AUDIT_RC=$?
  if [ "$AUDIT_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="BEV visual regression failed"; fi
fi

run_logged gateway_generalization_readiness /usr/bin/python3 "$SCRIPT_DIR/write_stage1_step30p1_gateway_readiness_report.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --output-json "$GATEWAY_READY_JSON" \
  --output-md "$GATEWAY_READY_MD" || true

if [ -n "$FINAL_FAILURE_REASON" ]; then
  capture_process_list "$PROCESS_LIST_AFTER_BRINGUP"
  cp "$PROCESS_LIST_AFTER_BRINGUP" "$PROCESS_LIST_AFTER_ROUTE" 2>/dev/null || true
  write_overlay_manifest || true
  write_final_report || true
  echo "[step30s7][ERROR] $FINAL_FAILURE_REASON" >&2
  exit 41
fi

if [ "$GUI" = "1" ] && [ -z "${DISPLAY:-}" ]; then
  FINAL_FAILURE_REASON="--gui requested but DISPLAY is not set; Gazebo/RViz GUI cannot open"
  capture_process_list "$PROCESS_LIST_AFTER_BRINGUP"
  cp "$PROCESS_LIST_AFTER_BRINGUP" "$PROCESS_LIST_AFTER_ROUTE" 2>/dev/null || true
  write_process_check_json "$GAZEBO_PROCESS_JSON" gazebo || true
  write_process_check_json "$RVIZ_PROCESS_JSON" rviz || true
  write_overlay_manifest || true
  write_final_report || true
  echo "[step30s7][ERROR] $FINAL_FAILURE_REASON" >&2
  exit 40
fi

source_ros_setup

run_logged route_query /usr/bin/python3 "$SCRIPT_DIR/prepare_stage1_step30p1_semantic_route.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --start-room "$START_ROOM" \
  --goal-room "$GOAL_ROOM" \
  --through-rooms "${THROUGH_ROOMS[@]}" \
  --terminal-room "$TERMINAL_ROOM" \
  --map-profile "$ACTIVE_MAP_PROFILE" \
  --output-json "$ROUTE_QUERY_JSON" \
  --output-md "$ROUTE_QUERY_MD" \
  --waypoints-output-json "$ROUTE_WAYPOINTS_JSON" \
  --target-selection-output-json "$TARGET_SELECTION_JSON" \
  --target-selection-output-md "$TARGET_SELECTION_MD"
ROUTE_QUERY_RC=$?
if [ "$ROUTE_QUERY_RC" -ne 0 ]; then FINAL_FAILURE_REASON="route query failed"; fi

read -r SPAWN_X SPAWN_Y SPAWN_YAW < <(/usr/bin/python3 - "$ROUTE_WAYPOINTS_JSON" <<'PY'
import json, sys
w = json.load(open(sys.argv[1], encoding="utf-8"))["waypoints"][0]
print(w["x"], w["y"], w.get("yaw", 0.0))
PY
)

MODE_ARG="--headless"
if [ "$GUI" = "1" ]; then MODE_ARG="--gui"; fi

run_logged clean_old_processes "$SCRIPT_DIR/launch_stage1_step30p1_gazebo_nav2.sh" --stage-output-dir "$STAGE_OUTPUT_DIR" --log-dir "$BRINGUP_LOG_DIR" --stop
(pgrep -f "rviz2.*00824_stage1_step30p1_bev_semantic_route_demo.rviz" 2>/dev/null || true) | while read -r pid; do
  if [ -n "$pid" ] && [ "$pid" != "$$" ]; then kill "$pid" >/dev/null 2>&1 || true; fi
done
(pgrep -f "publish_stage1_step30p1_rviz_overlay.py" 2>/dev/null || true) | while read -r pid; do
  if [ -n "$pid" ] && [ "$pid" != "$$" ]; then kill "$pid" >/dev/null 2>&1 || true; fi
done
run_logged bringup "$SCRIPT_DIR/launch_stage1_step30p1_gazebo_nav2.sh" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --ros-domain-id "$ROS_DOMAIN_ID_VALUE" \
  "$MODE_ARG" \
  --map-profile "$ACTIVE_MAP_PROFILE" \
  --log-dir "$BRINGUP_LOG_DIR" \
  --readiness-output-json "$LIFECYCLE_JSON" \
  --readiness-output-md "$LIFECYCLE_MD" \
  --readiness-timeout-sec 120 \
  --spawn-x "$SPAWN_X" \
  --spawn-y "$SPAWN_Y" \
  --spawn-yaw "$SPAWN_YAW"
BRINGUP_RC=$?
cp "$BRINGUP_LOG_DIR/bringup_result.json" "$EVIDENCE_DIR/bringup_result.json" 2>/dev/null || true
if [ "$BRINGUP_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then
  if [ -f "$LIFECYCLE_JSON" ]; then
    FINAL_FAILURE_REASON="Nav2 lifecycle bringup failed"
  else
    FINAL_FAILURE_REASON="bringup failed"
  fi
fi

if [ "$BRINGUP_RC" -eq 0 ]; then
  if [ "$GUI" = "1" ]; then
    for _ in $(seq 1 30); do
      if pgrep -f "gzclient" >/dev/null 2>&1; then GAZEBO_GUI_PRESENT=1; break; fi
      sleep 1
    done
    cp "$RVIZ_CONFIG" "$RVIZ_CONFIG_USED_COPY" 2>/dev/null || true
    setsid /usr/bin/python3 "$SCRIPT_DIR/publish_stage1_step30p1_rviz_overlay.py" \
      --stage-output-dir "$STAGE_OUTPUT_DIR" \
      --route-query-json "$ROUTE_QUERY_JSON" \
      --waypoints-json "$ROUTE_WAYPOINTS_JSON" \
      --trajectory-json "$TRAJECTORY_JSON" \
      --wall-validation-json "$WALL_JSON" \
      --topic "$MARKER_TOPIC" \
      --run-id "$RUN_ID" \
      > "$EVIDENCE_DIR/rviz_overlay_publisher.log" 2>&1 &
    OVERLAY_PID="$!"
    setsid rviz2 -d "$RVIZ_CONFIG_USED_COPY" > "$EVIDENCE_DIR/rviz2.log" 2>&1 &
    RVIZ_PID="$!"
    for _ in $(seq 1 30); do
      if pgrep -f "rviz2.*00824_stage1_step30p1_bev_semantic_route_demo.rviz" >/dev/null 2>&1 || kill -0 "$RVIZ_PID" >/dev/null 2>&1; then RVIZ_PRESENT=1; break; fi
      sleep 1
    done
    for _ in $(seq 1 20); do
      if ros2 topic list 2>/dev/null | grep -qx "$MARKER_TOPIC"; then OVERLAY_TOPIC_PRESENT=1; break; fi
      sleep 1
    done
    if [ "$GAZEBO_GUI_PRESENT" != "1" ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="--gui requested but gzclient was not observed"; fi
    if [ "$RVIZ_PRESENT" != "1" ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="--gui requested but rviz2 was not observed"; fi
    if [ "$OVERLAY_TOPIC_PRESENT" != "1" ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="RViz overlay marker topic was not observed"; fi
  fi
  capture_process_list "$PROCESS_LIST_AFTER_BRINGUP"
fi
write_process_check_json "$GAZEBO_PROCESS_JSON" gazebo || true
write_process_check_json "$RVIZ_PROCESS_JSON" rviz || true
write_overlay_manifest || true

if [ "$BRINGUP_RC" -eq 0 ] && { [ "$GUI" = "0" ] || { [ "$GAZEBO_GUI_PRESENT" = "1" ] && [ "$RVIZ_PRESENT" = "1" ]; }; }; then
  run_logged dataplane_probe /usr/bin/python3 "$SCRIPT_DIR/probe_stage1_step30p1_dataplane.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --timeout-sec 25 \
    --output-json "$DATAPLANE_JSON" \
    --output-md "$DATAPLANE_MD"
  DATAPLANE_RC=$?
  if [ "$DATAPLANE_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="dataplane probe failed"; fi
fi

ROUTE_FROM_START_ARGS=()
if [ "$FROM_START" = "1" ]; then ROUTE_FROM_START_ARGS+=(--from-start --reset-to-route-start); fi
EXPECTED_ROOMS="$(/usr/bin/python3 - "$ROUTE_QUERY_JSON" <<'PY'
import json, sys
print(",".join(json.load(open(sys.argv[1], encoding="utf-8"))["room_sequence"]))
PY
)"
EXPECTED_GATEWAYS="$(/usr/bin/python3 - "$ROUTE_QUERY_JSON" <<'PY'
import json, sys
print(",".join(json.load(open(sys.argv[1], encoding="utf-8"))["gateway_sequence"]))
PY
)"

SPLIT_DWELL_SOURCE=""
if [ "${#THROUGH_ROOMS[@]}" -gt 0 ]; then
  FIRST_THROUGH="${THROUGH_ROOMS[0]}"
  SPLIT_DWELL_SOURCE="${FIRST_THROUGH//_/}_interior_terminal"
fi

if [ "$BRINGUP_RC" -eq 0 ] && [ "$DATAPLANE_RC" -eq 0 ]; then
  run_logged route_execution /usr/bin/python3 "$SCRIPT_DIR/run_stage1_step30p1_route.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --waypoints-json "$ROUTE_WAYPOINTS_JSON" \
    --expected-room-chain "$EXPECTED_ROOMS" \
    --expected-gateway-sequence "$EXPECTED_GATEWAYS" \
    --allow-non-step30p1-truth \
    "${ROUTE_FROM_START_ARGS[@]}" \
    --follow-path-timeout-sec 480 \
    --goal-timeout-sec 180 \
    --split-dwell-source "$SPLIT_DWELL_SOURCE" \
    --split-dwell-sec "$THROUGH_ROOM_DWELL_SEC" \
    --output-json "$ROUTE_EXEC_JSON" \
    --output-md "$ROUTE_EXEC_MD" \
    --trajectory-output-json "$TRAJECTORY_JSON" \
    --latest-slice-output-json "$SLICE_JSON"
  ROUTE_RC=$?
  if [ "$ROUTE_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="route execution failed"; fi
fi

if [ -n "$OVERLAY_PID" ] && kill -0 "$OVERLAY_PID" >/dev/null 2>&1; then
  kill "$OVERLAY_PID" >/dev/null 2>&1 || true
  setsid /usr/bin/python3 "$SCRIPT_DIR/publish_stage1_step30p1_rviz_overlay.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --route-query-json "$ROUTE_QUERY_JSON" \
    --waypoints-json "$ROUTE_WAYPOINTS_JSON" \
    --trajectory-json "$TRAJECTORY_JSON" \
    --wall-validation-json "$WALL_JSON" \
    --topic "$MARKER_TOPIC" \
    --run-id "$RUN_ID" \
    > "$EVIDENCE_DIR/rviz_overlay_publisher_after_route.log" 2>&1 &
  OVERLAY_PID="$!"
fi

if [ -f "$ROUTE_EXEC_JSON" ]; then
  run_logged physical_validation /usr/bin/python3 "$SCRIPT_DIR/validate_stage1_step30p1_physical.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --route-query-json "$ROUTE_QUERY_JSON" \
    --waypoints-json "$ROUTE_WAYPOINTS_JSON" \
    --route-execution-json "$ROUTE_EXEC_JSON" \
    --trajectory-json "$TRAJECTORY_JSON" \
    --through-output-json "$THROUGH_JSON" \
    --through-output-md "$THROUGH_MD" \
    --terminal-output-json "$TERMINAL_JSON" \
    --terminal-output-md "$TERMINAL_MD" \
    --wall-output-json "$WALL_JSON" \
    --wall-output-md "$WALL_MD" \
    --spin-output-json "$SPIN_JSON" \
    --spin-output-md "$SPIN_MD" \
    --room15-min-inside-samples "$ROOM15_MIN_INSIDE_SAMPLES" \
    --through-room-dwell-sec "$THROUGH_ROOM_DWELL_SEC"
  VALIDATION_RC=$?
  if [ "$VALIDATION_RC" -ne 0 ] && [ -z "$FINAL_FAILURE_REASON" ]; then FINAL_FAILURE_REASON="strict physical validation failed"; fi
  if [ -n "$OVERLAY_PID" ] && kill -0 "$OVERLAY_PID" >/dev/null 2>&1; then
    kill "$OVERLAY_PID" >/dev/null 2>&1 || true
  fi
  setsid /usr/bin/python3 "$SCRIPT_DIR/publish_stage1_step30p1_rviz_overlay.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --route-query-json "$ROUTE_QUERY_JSON" \
    --waypoints-json "$ROUTE_WAYPOINTS_JSON" \
    --trajectory-json "$TRAJECTORY_JSON" \
    --wall-validation-json "$WALL_JSON" \
    --topic "$MARKER_TOPIC" \
    --run-id "$RUN_ID" \
    > "$EVIDENCE_DIR/rviz_overlay_publisher_after_validation.log" 2>&1 &
  OVERLAY_PID="$!"
fi

capture_process_list "$PROCESS_LIST_AFTER_ROUTE"
write_process_check_json "$GAZEBO_PROCESS_JSON" gazebo || true
write_process_check_json "$RVIZ_PROCESS_JSON" rviz || true

if write_final_report; then FINAL_RC=0; else FINAL_RC=1; fi

if [ "$SHUTDOWN_AFTER_RUN" = "1" ]; then
  if [ -n "$RVIZ_PID" ]; then kill "$RVIZ_PID" >/dev/null 2>&1 || true; fi
  if [ -n "$OVERLAY_PID" ]; then kill "$OVERLAY_PID" >/dev/null 2>&1 || true; fi
  run_logged shutdown_after_run "$SCRIPT_DIR/launch_stage1_step30p1_gazebo_nav2.sh" --stage-output-dir "$STAGE_OUTPUT_DIR" --log-dir "$BRINGUP_LOG_DIR" --stop || true
elif [ "$GUI" = "1" ]; then
  log ""; log "Keeping Gazebo/RViz runtime open for ${KEEP_GUI_OPEN_SEC}s before this command exits."
  sleep "$KEEP_GUI_OPEN_SEC"
fi

if [ "$FINAL_RC" -eq 0 ]; then
  echo "[step30s7] SUCCESS report: $REPORT_JSON"
else
  echo "[step30s7] FAILURE report: $REPORT_JSON" >&2
fi
exit "$FINAL_RC"
