#!/usr/bin/env bash
# Stage1 Navigation Stop - clean shutdown of Gazebo/Nav2/RViz
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INTERNAL="$REPO_ROOT/tools/stage1_step30p1"

STAGE_OUTPUT_DIR=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --help|-h) echo "Usage: tools/stage1_nav/stop.sh --stage-output-dir DIR"; exit 0 ;;
    *) shift ;;
  esac
done

if [ -z "$STAGE_OUTPUT_DIR" ]; then
  STAGE_OUTPUT_DIR="stage_outputs/stage1_00824_step30p1"
fi

echo "[stage1_nav] Stopping all Stage1 navigation processes..."

# Ask any live RViz session to clear Stage1 markers before publishers die.
if [ -f /opt/ros/foxy/setup.bash ]; then
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}" /usr/bin/python3 "$INTERNAL/publish_stage1_step30p1_rviz_overlay.py" \
    --stage-output-dir "$STAGE_OUTPUT_DIR" \
    --topic /stage1_nav/semantic_overlay_markers \
    --run-id cleanup \
    --clear-only >/dev/null 2>&1 || true
fi

# Stop Gazebo/Nav2 stack
"$INTERNAL/launch_stage1_step30p1_gazebo_nav2.sh" --stage-output-dir "$STAGE_OUTPUT_DIR" --log-dir "$STAGE_OUTPUT_DIR/current_validation/stop_logs" --stop || true

# Kill any lingering overlay publishers. Avoid pkill -f here because the
# parent shell command can contain the same pattern during validation.
for pattern in "publish_stage1_step30p1_rviz_overlay" "boxfusion_stage1_nav_overlay_publisher"; do
  ps -eo pid=,comm=,args= | awk -v pat="$pattern" '$2 == "python3" && index($0, pat) {print $1}' | while read -r pid; do
    if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
done

# Kill any lingering RViz2 instances for this project
for pattern in "00824_stage1_step30p1" "stage1_nav" "stage1_00824_step30p1"; do
  ps -eo pid=,comm=,args= | awk -v pat="$pattern" '$2 == "rviz2" && index($0, pat) {print $1}' | while read -r pid; do
    if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
done

echo "[stage1_nav] All processes stopped."
