#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ws/workspace/BoxFusion"
SCENE_ID="00843-DYehNKdT76V"
DEMO_DIR="${ROOT}/stage_outputs/rslg_slam/${SCENE_ID}/canonical/demo_evidence_pack"
RVIZ_CONFIG="${DEMO_DIR}/rviz/task42_demo.rviz"
MARKERS_JSON="${DEMO_DIR}/rviz/task42_demo_markers_v0_1.json"

if [[ "${RSLG_TASK42_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "RSLG_TASK42_ALLOW_RUNTIME=1 is required for RViz showcase commands." >&2
  exit 2
fi

cd "${ROOT}"

if [[ ! -f "${MARKERS_JSON}" || ! -f "${RVIZ_CONFIG}" ]]; then
  /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/export_task42_marker_overlays.py
fi

echo "Task42 RViz replay package is ready."
echo "MarkerArray JSON: ${MARKERS_JSON}"
echo "RViz config: ${RVIZ_CONFIG}"
echo "Preferred topic: /task42_demo_markers"
echo "RViz Map display is not required."

if command -v rviz2 >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  echo "Launching rviz2 with task42 MarkerArray config."
  rviz2 -d "${RVIZ_CONFIG}"
else
  echo "rviz2 launch skipped because rviz2 is unavailable or DISPLAY is not set."
  echo "Use the files above on a display-enabled ROS2 Foxy environment."
fi
