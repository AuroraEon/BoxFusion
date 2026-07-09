#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

QUERY_ID="${RSLG_QUERY_ID:-00843_cross_floor_object_curtain_room14}"
PROFILE_ID="${RSLG_PROFILE_ID:-practical_zero_collision}"
TASK56C_ROOT="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56c_pid_profile_promotion_and_regression_validation/regression_pack"
PUBLISHER="${REPO_ROOT}/tools/rslg_pipeline/rviz/publish_rslg_rviz_showcase.py"
RVIZ_CONFIG="${REPO_ROOT}/tools/rslg_pipeline/rviz/config/rslg_practical_profile_showcase.rviz"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

ROUTE_RESULT="${TASK56C_ROOT}/route_results/${QUERY_ID}_route_result.json"
MARKER_INPUT="${TASK56C_ROOT}/runtime_adapter_inputs/rviz_marker_inputs/${QUERY_ID}_rviz_marker_input.json"
Z_AWARE_INPUT="${TASK56C_ROOT}/runtime_adapter_inputs/z_aware_overlay_inputs/${QUERY_ID}_z_aware_overlay_input.json"
TRAJECTORY_CSV="${TASK56C_ROOT}/practical_zero_collision_pid_replay/trajectories/${QUERY_ID}_trajectory.csv"
PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"

PUBLISHER_ARGS=(
  --rviz-marker-input-json "${MARKER_INPUT}"
  --z-aware-overlay-input-json "${Z_AWARE_INPUT}"
  --trajectory-csv "${TRAJECTORY_CSV}"
  --route-result-json "${ROUTE_RESULT}"
  --profile-json "${PROFILE_JSON}"
  --profile-id "${PROFILE_ID}"
  --frame-id map
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}'
  --z-scale 1.0
  --rate "${RSLG_PLAYBACK_RATE:-10}"
  --loop
  --no-canonical-write
)

if [[ "${1:-}" == "--dry-run" ]]; then
  "${ROS_PYTHON}" "${PUBLISHER}" "${PUBLISHER_ARGS[@]}" --dry-run
  exit 0
fi

"${ROS_PYTHON}" "${PUBLISHER}" "${PUBLISHER_ARGS[@]}" &
PUBLISHER_PID=$!
trap 'kill "${PUBLISHER_PID}" 2>/dev/null || true' EXIT

if [[ "${1:-}" == "--publisher-only" ]]; then
  wait "${PUBLISHER_PID}"
elif command -v rviz2 >/dev/null 2>&1; then
  rviz2 -d "${RVIZ_CONFIG}"
else
  echo "rviz2 was not found. Publisher is running; add topics manually in an RViz-enabled shell." >&2
  wait "${PUBLISHER_PID}"
fi

