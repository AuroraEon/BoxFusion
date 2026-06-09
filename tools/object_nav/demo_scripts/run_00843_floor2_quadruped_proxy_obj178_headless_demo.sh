#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/ws/workspace/BoxFusion
cd "$ROOT"

set +u
source /opt/ros/foxy/setup.bash
set -u
export ROS_DOMAIN_ID=84

PY=/usr/bin/python3
TASK_DIR=stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23a_clean_obj178_quadruped_proxy_gui_demo_rebuild
RUN_ID="${1:-obj178_headless_demo}"
OUTPUT_DIR="$TASK_DIR/runs"
RUN_DIR="$OUTPUT_DIR/$RUN_ID"
LOG_DIR="$TASK_DIR/logs"
mkdir -p "$OUTPUT_DIR" "$LOG_DIR"
rm -rf "$RUN_DIR"

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
  --execute
)

"${COMMAND[@]}" 2>&1 | tee "$LOG_DIR/${RUN_ID}.log"

"$PY" - "$RUN_DIR" "$LOG_DIR/${RUN_ID}_formal_command.json" "${COMMAND[@]}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

run_dir = Path(sys.argv[1])
target = Path(sys.argv[2])
command = sys.argv[3:]
payload = {
    "artifact_type": "task23a_formal_headless_demo_command",
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "run_dir": str(run_dir),
    "command": command,
    "python": "/usr/bin/python3",
    "ros_domain_id": "84",
}
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
(run_dir / "formal_demo_command.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

"$PY" tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_outputs.py \
  --run-dir "$RUN_DIR" \
  --report-json "$RUN_DIR/task23a_validation_report.json" \
  --report-md "$RUN_DIR/task23a_validation_report.md"
