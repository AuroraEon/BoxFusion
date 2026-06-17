#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ws/workspace/BoxFusion"
SCENE_ID="00843-DYehNKdT76V"
RVIZ_DIR="${ROOT}/stage_outputs/rslg_slam/${SCENE_ID}/canonical/demo_evidence_pack/rviz_3d_dynamic"
RVIZ_CONFIG="${RVIZ_DIR}/task42b_3d_demo.rviz"
MARKERS_JSON="${RVIZ_DIR}/task42b_3d_static_markers_v0_1.json"
FRAMES_JSON="${RVIZ_DIR}/task42b_dynamic_replay_frames_v0_1.json"
PUBLISHER="${ROOT}/tools/rslg_pipeline/publish_task42b_3d_replay.py"

if [[ "${RSLG_TASK42B_ALLOW_RVIZ:-0}" != "1" && "${RSLG_TASK42_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "RSLG_TASK42B_ALLOW_RVIZ=1 or RSLG_TASK42_ALLOW_RUNTIME=1 is required for task42b RViz showcase commands." >&2
  exit 2
fi

cd "${ROOT}"

if [[ ! -f "${MARKERS_JSON}" || ! -f "${FRAMES_JSON}" || ! -f "${RVIZ_CONFIG}" ]]; then
  /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/generate_task42b_3d_replay_assets.py
fi

if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck source=/opt/ros/foxy/setup.bash
  source /opt/ros/foxy/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"

PUBLISHER_PID=""
cleanup() {
  if [[ -n "${PUBLISHER_PID}" ]] && kill -0 "${PUBLISHER_PID}" >/dev/null 2>&1; then
    kill "${PUBLISHER_PID}" >/dev/null 2>&1 || true
    wait "${PUBLISHER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

/usr/bin/python3 "${PUBLISHER}" --markers-json "${MARKERS_JSON}" --frames-json "${FRAMES_JSON}" --static-only &
PUBLISHER_PID="$!"

echo "Task42b static 3D MarkerArray publisher started on /task42b_3d_demo_markers (pid ${PUBLISHER_PID})."
echo "Gazebo, Nav2, map_server, and route executors are not launched."

if command -v rviz2 >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  rviz2 -d "${RVIZ_CONFIG}"
else
  echo "rviz2 launch skipped because rviz2 is unavailable or DISPLAY is not set."
  echo "Open a display-enabled ROS2 Foxy shell and run:"
  echo "  rviz2 -d ${RVIZ_CONFIG}"
  echo "Press Ctrl+C to stop the static publisher."
  wait "${PUBLISHER_PID}"
fi
