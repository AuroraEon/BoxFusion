#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MARKER_JSON="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24d_cross_floor_rviz_overlay_adapter_with_transition_semantics_fix/cross_floor_rviz_markers_v0_1.json"
TOPIC="/rslg/cross_floor_markers"
Z_VISUAL_SCALE="1.0"
PUBLISH_RATE="0.5"
RVIZ_MODE="auto"
RVIZ_CONFIG="${SCRIPT_DIR}/cross_floor_3d_overlay.rviz"
RVIZ_PID=""
STATIC_TF_MODE="on"
STATIC_TF_PID=""

cleanup() {
  if [[ -n "${RVIZ_PID}" ]]; then
    kill "${RVIZ_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${STATIC_TF_PID}" ]]; then
    kill "${STATIC_TF_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --marker-json)
      MARKER_JSON="$2"
      shift 2
      ;;
    --topic)
      TOPIC="$2"
      shift 2
      ;;
    --z-visual-scale)
      Z_VISUAL_SCALE="$2"
      shift 2
      ;;
    --publish-rate)
      PUBLISH_RATE="$2"
      shift 2
      ;;
    --rviz)
      RVIZ_MODE="force"
      shift
      ;;
    --no-rviz)
      RVIZ_MODE="off"
      shift
      ;;
    --no-static-tf)
      STATIC_TF_MODE="off"
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -f /opt/ros/foxy/setup.bash ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/foxy/setup.bash
  set -u
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "ROS2 setup.bash was not found under /opt/ros; continuing with current environment." >&2
fi

echo "RSLG-SLAM task24e MarkerArray demo"
echo "Project path: ${REPO_ROOT}"
echo "Marker JSON: ${MARKER_JSON}"
echo "Marker topic: ${TOPIC}"
echo "RViz Fixed Frame: world"
echo "Marker frame: map"
echo "Static TF: world -> map"
echo "Claim boundary: topological overlay only; no Gazebo/Nav2/Habitat/robot execution."

if [[ "${STATIC_TF_MODE}" != "off" ]]; then
  if command -v ros2 >/dev/null 2>&1; then
    echo "Launching static TF publisher: world -> map"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map >/tmp/rslg_task24e2_static_tf.log 2>&1 &
    STATIC_TF_PID=$!
    echo "static tf pid: ${STATIC_TF_PID}"
  else
    echo "ros2 was not found; static TF publisher was not launched." >&2
  fi
else
  echo "Static TF disabled by --no-static-tf."
fi

if [[ "${RVIZ_MODE}" != "off" ]]; then
  if command -v rviz2 >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" || "${RVIZ_MODE}" == "force" ]]; then
    echo "Launching rviz2 with ${RVIZ_CONFIG}"
    rviz2 -d "${RVIZ_CONFIG}" >/tmp/rslg_task24e_rviz2.log 2>&1 &
    RVIZ_PID=$!
    echo "rviz2 pid: ${RVIZ_PID}"
  else
    echo "RViz was not launched automatically. Open RViz manually, set Fixed Frame to world, add MarkerArray, and choose ${TOPIC}."
  fi
fi

/usr/bin/python3 "${SCRIPT_DIR}/publish_cross_floor_rviz_markers.py" \
  --marker-json "${MARKER_JSON}" \
  --topic "${TOPIC}" \
  --z-visual-scale "${Z_VISUAL_SCALE}" \
  --publish-rate "${PUBLISH_RATE}"
