#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/ws/workspace/BoxFusion
cd "$ROOT"

set +u
source /opt/ros/foxy/setup.bash
set -u
export ROS_DOMAIN_ID=84

PY=/usr/bin/python3
TASK_DIR=stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture
OUTPUT_DIR="$TASK_DIR/runs"
RUN_ID=gui_smoke
KEEP_OPEN_SEC=45
START_RVIZ=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --keep-open)
      KEEP_OPEN_SEC="${2:-120}"
      shift 2
      ;;
    --no-rviz)
      START_RVIZ=0
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

RUN_DIR="$OUTPUT_DIR/$RUN_ID"
LOG_DIR="$TASK_DIR/logs"
PROCESS_SNAPSHOT_DIR="$TASK_DIR/process_snapshots"
GUI_SAMPLE_DIR="$RUN_DIR/gui_samples"
GUI_ENV_FILE="$TASK_DIR/gui_environment.txt"
RVIZ_CONFIG=stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun/runtime/rviz/00843_floor2_route_debug_safe_no_map.rviz
mkdir -p "$OUTPUT_DIR" "$LOG_DIR" "$PROCESS_SNAPSHOT_DIR"
rm -rf "$RUN_DIR" "$PROCESS_SNAPSHOT_DIR"
mkdir -p "$GUI_SAMPLE_DIR" "$PROCESS_SNAPSHOT_DIR"

snapshot_processes() {
  local name="$1"
  ps -eo pid=,ppid=,stat=,comm=,args= \
    | awk '/gzserver|gzclient|rviz2|quadruped_kinematic_proxy_node|run_lightweight_object_nav.py|champ_reference/ && !/awk/ {print}' \
    >"$PROCESS_SNAPSHOT_DIR/${name}.txt" || true
}

capture_gui_environment() {
  {
    printf 'created_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'DISPLAY=%s\n' "${DISPLAY:-}"
    printf 'WAYLAND_DISPLAY=%s\n' "${WAYLAND_DISPLAY:-}"
    printf 'XDG_SESSION_TYPE=%s\n' "${XDG_SESSION_TYPE:-}"
    printf 'XAUTHORITY=%s\n' "${XAUTHORITY:-}"
    printf 'QT_QPA_PLATFORM=%s\n' "${QT_QPA_PLATFORM:-}"
    printf 'LIBGL_ALWAYS_SOFTWARE=%s\n' "${LIBGL_ALWAYS_SOFTWARE:-}"
    printf 'MESA_LOADER_DRIVER_OVERRIDE=%s\n' "${MESA_LOADER_DRIVER_OVERRIDE:-}"
    printf 'GAZEBO_MASTER_URI=%s\n' "${GAZEBO_MASTER_URI:-}"
    printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-}"
    printf 'gzserver_path=%s\n' "$(command -v gzserver || true)"
    printf 'gzclient_path=%s\n' "$(command -v gzclient || true)"
    printf 'rviz2_path=%s\n' "$(command -v rviz2 || true)"
    printf 'ros2_path=%s\n' "$(command -v ros2 || true)"
  } >"$GUI_ENV_FILE"
}

copy_launch_logs() {
  [[ -f "$RUN_DIR/launch_logs/gazebo.log" ]] && cp "$RUN_DIR/launch_logs/gazebo.log" "$LOG_DIR/${RUN_ID}_gzserver.log"
  [[ -f "$RUN_DIR/launch_logs/gazebo_gui.log" ]] && cp "$RUN_DIR/launch_logs/gazebo_gui.log" "$LOG_DIR/${RUN_ID}_gzclient.log"
  [[ -f "$RUN_DIR/launch_logs/quadruped_proxy_node.log" ]] && cp "$RUN_DIR/launch_logs/quadruped_proxy_node.log" "$LOG_DIR/${RUN_ID}_quadruped_proxy_node.log"
  [[ -f "$RUN_DIR/launch_logs/spawn_entity.log" ]] && cp "$RUN_DIR/launch_logs/spawn_entity.log" "$LOG_DIR/${RUN_ID}_spawn_entity.log"
  [[ -f "$LOG_DIR/${RUN_ID}_gzserver.log" ]] || : >"$LOG_DIR/${RUN_ID}_gzserver.log"
  [[ -f "$LOG_DIR/${RUN_ID}_gzclient.log" ]] || : >"$LOG_DIR/${RUN_ID}_gzclient.log"
  [[ -f "$LOG_DIR/${RUN_ID}_quadruped_proxy_node.log" ]] || : >"$LOG_DIR/${RUN_ID}_quadruped_proxy_node.log"
  [[ -f "$LOG_DIR/${RUN_ID}_spawn_entity.log" ]] || : >"$LOG_DIR/${RUN_ID}_spawn_entity.log"
}

capture_gui_environment
snapshot_processes "000_before_launch"

RVIZ_PID=""
RVIZ_LAUNCH_ATTEMPTED=false
RVIZ_REASON=""
if [[ "$START_RVIZ" == "1" ]]; then
  if [[ -f "$RVIZ_CONFIG" ]] && command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "$RVIZ_CONFIG" >"$LOG_DIR/${RUN_ID}_rviz2.log" 2>&1 &
    RVIZ_PID=$!
    RVIZ_LAUNCH_ATTEMPTED=true
  elif [[ ! -f "$RVIZ_CONFIG" ]]; then
    RVIZ_REASON="rviz_config_missing: $RVIZ_CONFIG"
    echo "RViz is not configured for this demo: $RVIZ_CONFIG" >"$LOG_DIR/${RUN_ID}_rviz2.log"
  else
    RVIZ_REASON="rviz2_command_missing"
    echo "rviz2 is not available on PATH" >"$LOG_DIR/${RUN_ID}_rviz2.log"
  fi
else
  RVIZ_REASON="rviz_disabled_by_user"
  echo "rviz2 launch disabled by --no-rviz" >"$LOG_DIR/${RUN_ID}_rviz2.log"
fi
snapshot_processes "001_after_rviz_launch"

COMMAND=(
  "$PY" tools/object_nav/run_lightweight_object_nav.py
  --query bed
  --object-id obj_178
  --start-room room_11
  --floor-id floor_2
  --stage-output-dir stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun
  --map-yaml stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml
  --robot-profile tools/object_nav/robot_profiles/champ_reference_kinematic_proxy.yaml
  --output-dir "$OUTPUT_DIR"
  --run-id "$RUN_ID"
  --timeout-sec 420
  --max-linear-speed 0.10
  --lookahead-distance 0.60
  --lookahead-min 0.40
  --lookahead-max 1.00
  --angular-smoothing-alpha 0.40
  --angular-rate-limit 0.15
  --gui
  --keep-open-sec "$KEEP_OPEN_SEC"
  --execute
)

printf '%s\n' "${COMMAND[@]}" >"$LOG_DIR/${RUN_ID}_formal_command.txt"
"${COMMAND[@]}" >"$LOG_DIR/${RUN_ID}.log" 2>&1 &
RUNNER_PID=$!
snapshot_processes "002_after_runner_launch"

for sample_idx in $(seq 1 420); do
  sample_name="$(printf 'sample_%03d' "$sample_idx")"
  snapshot_processes "$sample_name"
  cp "$PROCESS_SNAPSHOT_DIR/${sample_name}.txt" "$GUI_SAMPLE_DIR/processes.txt"

  if (( sample_idx % 5 == 0 )); then
    ros2 topic list >"$GUI_SAMPLE_DIR/topics_${sample_idx}.txt" 2>"$GUI_SAMPLE_DIR/topics_${sample_idx}.err" || true
    ros2 service list >"$GUI_SAMPLE_DIR/services_${sample_idx}.txt" 2>"$GUI_SAMPLE_DIR/services_${sample_idx}.err" || true
    ros2 topic info /cmd_vel >"$GUI_SAMPLE_DIR/cmd_vel_info_${sample_idx}.txt" 2>"$GUI_SAMPLE_DIR/cmd_vel_info_${sample_idx}.err" || true
    cp "$GUI_SAMPLE_DIR/topics_${sample_idx}.txt" "$GUI_SAMPLE_DIR/topics.txt"
    cp "$GUI_SAMPLE_DIR/services_${sample_idx}.txt" "$GUI_SAMPLE_DIR/services.txt"
    cp "$GUI_SAMPLE_DIR/cmd_vel_info_${sample_idx}.txt" "$GUI_SAMPLE_DIR/cmd_vel_info.txt"
  fi

  if [[ -f "$RUN_DIR/runtime_result.json" ]]; then
    snapshot_processes "runtime_result_observed"
    break
  fi
  if ! kill -0 "$RUNNER_PID" >/dev/null 2>&1; then
    snapshot_processes "runner_exited_early"
    break
  fi
  sleep 1
done

set +e
wait "$RUNNER_PID"
RUNNER_RC=$?
set -e
snapshot_processes "after_runner_wait"

if [[ -n "$RVIZ_PID" ]]; then
  if kill -0 "$RVIZ_PID" >/dev/null 2>&1; then
    sleep 2
    snapshot_processes "before_rviz_stop"
    kill "$RVIZ_PID" >/dev/null 2>&1 || true
    wait "$RVIZ_PID" >/dev/null 2>&1 || true
  fi
fi
snapshot_processes "final_after_cleanup"
copy_launch_logs

"$PY" tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_outputs.py \
  --run-dir "$RUN_DIR" \
  --report-json "$RUN_DIR/task23a_validation_report.json" \
  --report-md "$RUN_DIR/task23a_validation_report.md" || true

"$PY" tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_gui_smoke.py \
  --task-dir "$TASK_DIR" \
  --run-dir "$RUN_DIR" \
  --run-id "$RUN_ID" \
  --runner-returncode "$RUNNER_RC" \
  --rviz-launch-attempted "$RVIZ_LAUNCH_ATTEMPTED" \
  --rviz-reason "$RVIZ_REASON" \
  --rviz-config "$RVIZ_CONFIG" \
  --keep-open-sec "$KEEP_OPEN_SEC" \
  --report-json "$TASK_DIR/task23b_gui_smoke_report.json" \
  --report-md "$TASK_DIR/task23b_gui_smoke_summary.md" \
  -- "${COMMAND[@]}"

exit "$RUNNER_RC"
