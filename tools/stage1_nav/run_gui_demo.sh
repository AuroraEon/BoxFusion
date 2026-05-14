#!/usr/bin/env bash
# Stage1 Navigation GUI Demo - consolidated entry point
# Wraps the internal end-to-end orchestration with clean argument interface.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INTERNAL="$REPO_ROOT/tools/stage1_step30p1"

STAGE_OUTPUT_DIR=""
START_ROOM="room_1"
GOAL_ROOM="room_16"
THROUGH_ROOMS=()
TERMINAL_ROOM="room_16"
MAP_PROFILE="stable"
KEEP_GUI_OPEN_SEC=20
THROUGH_ROOM_MIN_INSIDE_SAMPLES=8
THROUGH_ROOM_DWELL_SEC=3.0
FROM_START=0
GUI=1
STOP_ONLY=0
EXTRA_ARGS=()
RUN_ID=""

usage() {
  cat <<'EOF'
Usage:
  tools/stage1_nav/run_gui_demo.sh \
    --stage-output-dir stage_outputs/stage1_00824_step30p1 \
    --start-room room_1 --goal-room room_16 \
    --through-rooms room_8 \
    --terminal-room room_16 \
    --from-start --gui \
    --map-profile stable \
    --keep-gui-open-sec 20

Active map profiles: stable (default), auto, reference_h8r2
Debug/reference only: request_aware
Deprecated/removed: step30s5_room15_diagnostic_patch, step30s5_room15_interior
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --start-room) START_ROOM="$2"; shift 2 ;;
    --goal-room) GOAL_ROOM="$2"; shift 2 ;;
    --through-rooms) THROUGH_ROOMS+=("$2"); shift 2 ;;
    --terminal-room) TERMINAL_ROOM="$2"; shift 2 ;;
    --map-profile) MAP_PROFILE="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --keep-gui-open-sec) KEEP_GUI_OPEN_SEC="$2"; shift 2 ;;
    --through-room-min-inside-samples) THROUGH_ROOM_MIN_INSIDE_SAMPLES="$2"; shift 2 ;;
    --through-room-dwell-sec) THROUGH_ROOM_DWELL_SEC="$2"; shift 2 ;;
    --from-start) FROM_START=1; shift ;;
    --gui) GUI=1; shift ;;
    --headless|--no-gui) GUI=0; shift ;;
    --stop) STOP_ONLY=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

if [ -z "$STAGE_OUTPUT_DIR" ]; then
  echo "[stage1_nav][ERROR] --stage-output-dir is required" >&2
  usage >&2
  exit 2
fi

# Reject deprecated Step30S5 profiles
case "$MAP_PROFILE" in
  step30s5_room15_diagnostic_patch|step30s5_room15_interior)
    echo "[stage1_nav][ERROR] Map profile '$MAP_PROFILE' is deprecated and removed from active use." >&2
    exit 12
    ;;
esac

# Map profile aliases
case "$MAP_PROFILE" in
  stable|full_scene|stage1_full_scene|stage1_full_scene_occupancy) MAP_PROFILE="stage1_full_scene_occupancy" ;;
  auto) MAP_PROFILE="stage1_full_scene_occupancy" ;;
  request_aware) MAP_PROFILE="step30s7_request_aware" ;;
  reference_h8r2) MAP_PROFILE="h8r2" ;;
esac

# Build argument list for internal script
ARGS=(
  --stage-output-dir "$STAGE_OUTPUT_DIR"
  --start-room "$START_ROOM"
  --goal-room "$GOAL_ROOM"
  --terminal-room "$TERMINAL_ROOM"
  --map-profile "$MAP_PROFILE"
  --keep-gui-open-sec "$KEEP_GUI_OPEN_SEC"
  --through-room-min-inside-samples "$THROUGH_ROOM_MIN_INSIDE_SAMPLES"
  --through-room-dwell-sec "$THROUGH_ROOM_DWELL_SEC"
  --mode full
)
if [ -n "$RUN_ID" ]; then ARGS+=(--run-id "$RUN_ID"); fi

for room in "${THROUGH_ROOMS[@]}"; do
  ARGS+=(--through-rooms "$room")
done

if [ "$FROM_START" = "1" ]; then ARGS+=(--from-start); fi
if [ "$GUI" = "1" ]; then ARGS+=(--gui); else ARGS+=(--headless); fi

ARGS+=("${EXTRA_ARGS[@]}")

echo "[stage1_nav] Running GUI demo: start=$START_ROOM goal=$GOAL_ROOM through=${THROUGH_ROOMS[*]:-none} profile=$MAP_PROFILE"
exec "$INTERNAL/run_stage1_step30p1_end_to_end.sh" "${ARGS[@]}"
