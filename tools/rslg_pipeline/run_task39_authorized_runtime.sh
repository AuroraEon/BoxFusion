#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

if [[ "${RSLG_TASK38_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "RSLG_TASK38_ALLOW_RUNTIME=1 is required." >&2
  exit 2
fi
if [[ "${RSLG_TASK39_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "RSLG_TASK39_ALLOW_RUNTIME=1 is required." >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

had_nounset=0
case "$-" in *u*) had_nounset=1 ;; esac
set +u
source /opt/ros/foxy/setup.bash
if [[ "$had_nounset" == "1" ]]; then set -u; fi

exec /usr/bin/python3 \
  "$SCRIPT_DIR/run_task39_cross_floor_runtime.py" \
  --execute "$@"
