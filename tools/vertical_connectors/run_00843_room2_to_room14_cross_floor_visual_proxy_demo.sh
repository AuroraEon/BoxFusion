#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PY="/usr/bin/python3"
PLAYER="${SCRIPT_DIR}/cross_floor_visual_traversal_player.py"
ROUTE_CONTRACT="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24f_visual_kinematic_proxy_cross_floor_traversal_feasibility_and_plan/cross_floor_3d_route_contract_v0_1.json"
OUT_DIR="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g_room2_to_room14_cross_floor_visual_proxy_traversal_demo"
RUNTIME_DIR="${OUT_DIR}/runtime"
LOG_DIR="${OUT_DIR}/launch_logs"
PROFILE="${REPO_ROOT}/tools/object_nav/robot_profiles/champ_reference_kinematic_proxy.yaml"
ENTITY_NAME="rslg_cross_floor_quadruped_proxy"
URDF="${RUNTIME_DIR}/rslg_cross_floor_quadruped_proxy_visual_only_fixed.urdf"
WORLD="${RUNTIME_DIR}/task24g_empty_world_with_state_plugin.sdf"
SOURCE_WORLD="/usr/share/gazebo-11/worlds/empty.world"
RVIZ_CONFIG="${SCRIPT_DIR}/rviz_room2_to_room14_cross_floor_visual_proxy.rviz"
RVIZ_MODE="auto"
GAZEBO_MODE="auto"
KEEP_OPEN="0"
HOLD_FINAL_SEC="8.0"
PUBLISH_RATE="10.0"
HORIZONTAL_SPEED="0.28"
VERTICAL_SPEED="0.15"
Z_VISUAL_SCALE="1.0"
STATIC_TF_PID=""
GZSERVER_PID=""
GZCLIENT_PID=""
RVIZ_PID=""
ORIGINAL_COMMAND="tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh"
for arg in "$@"; do
  ORIGINAL_COMMAND+=" $(printf '%q' "${arg}")"
done

usage() {
  cat <<'EOF'
Usage:
  tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh [options]

Options:
  --no-rviz          Do not auto-start RViz.
  --rviz            Try to start RViz even if DISPLAY is not set.
  --no-gazebo       Do not launch Gazebo; run player dry-run and write blocked validation artifacts.
  --keep-open       Keep Gazebo/RViz open after traversal until Ctrl-C.
  --hold-final SEC  Keep publishing final pose for SEC seconds after traversal.
EOF
}

cleanup() {
  if [[ "${KEEP_OPEN}" == "1" ]]; then
    return
  fi
  for pid in "${RVIZ_PID}" "${GZCLIENT_PID}" "${STATIC_TF_PID}" "${GZSERVER_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      RVIZ_MODE="off"
      shift
      ;;
    --rviz)
      RVIZ_MODE="force"
      shift
      ;;
    --no-gazebo)
      GAZEBO_MODE="off"
      shift
      ;;
    --keep-open)
      KEEP_OPEN="1"
      shift
      ;;
    --hold-final)
      HOLD_FINAL_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${RUNTIME_DIR}" "${LOG_DIR}"

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

echo "RSLG-SLAM task24g cross-floor visual proxy traversal demo"
echo "Project path: ${REPO_ROOT}"
echo "Route contract: ${ROUTE_CONTRACT}"
echo "Output dir: ${OUT_DIR}"
echo "Gazebo entity: ${ENTITY_NAME}"
echo "Claim boundary: visual_kinematic_proxy_only"
echo "Claim boundary: topological_vertical_transition_only"
echo "physical_stair_climbing_supported=false"
echo "No gait, no footstep planning, no contact-based stair climbing, no real quadruped stair locomotion."
echo "Nav2 will not be launched by this script."

"${PY}" "${PLAYER}" \
  --route-contract "${ROUTE_CONTRACT}" \
  --robot-profile "${PROFILE}" \
  --entity-name "${ENTITY_NAME}" \
  --prepare-urdf "${URDF}" \
  --prepare-world "${WORLD}" \
  --source-world "${SOURCE_WORLD}" \
  --prepare-only

read -r SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW < <("${PY}" "${PLAYER}" --route-contract "${ROUTE_CONTRACT}" --print-start-pose)

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local start
  start="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      return 0
    fi
    if (( "$(date +%s)" - start >= timeout_sec )); then
      return 1
    fi
    sleep 0.5
  done
}

service_exists() {
  local service_name="$1"
  ros2 service list 2>/dev/null | grep -qx "${service_name}"
}

if [[ "${GAZEBO_MODE}" != "off" ]]; then
  if service_exists "/set_entity_state" || service_exists "/gazebo/set_entity_state"; then
    echo "Reusing existing Gazebo SetEntityState service."
  else
    if command -v gzserver >/dev/null 2>&1; then
      echo "Launching gzserver with Gazebo ROS factory/state plugins."
      gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "${WORLD}" >"${LOG_DIR}/gzserver.log" 2>&1 &
      GZSERVER_PID=$!
      echo "gzserver pid: ${GZSERVER_PID}"
    else
      echo "gzserver was not found; player will write blocked validation artifacts." >&2
      GAZEBO_MODE="off"
    fi
  fi
fi

if [[ "${GAZEBO_MODE}" != "off" ]]; then
  if ! wait_for_service "/spawn_entity" 45 && ! wait_for_service "/gazebo/spawn_entity" 5; then
    echo "Gazebo spawn service was not observed; continuing so the player can report the blocker." >&2
  else
    echo "Spawning or reusing visual-only CHAMP-shaped proxy at route start."
    timeout 60s "${PY}" /opt/ros/foxy/lib/gazebo_ros/spawn_entity.py \
      -file "${URDF}" \
      -entity "${ENTITY_NAME}" \
      -x "${SPAWN_X}" \
      -y "${SPAWN_Y}" \
      -z "${SPAWN_Z}" \
      -Y "${SPAWN_YAW}" \
      >"${LOG_DIR}/spawn_entity.log" 2>&1 || true
  fi

  if [[ -n "${DISPLAY:-}" ]] && command -v gzclient >/dev/null 2>&1; then
    echo "Launching gzclient."
    gzclient >"${LOG_DIR}/gzclient.log" 2>&1 &
    GZCLIENT_PID=$!
    echo "gzclient pid: ${GZCLIENT_PID}"
  else
    echo "Gazebo GUI client was not launched automatically; gzserver playback can still run."
  fi
fi

if command -v ros2 >/dev/null 2>&1; then
  echo "Publishing static TF world -> map for RViz compatibility."
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map >"${LOG_DIR}/static_tf_world_map.log" 2>&1 &
  STATIC_TF_PID=$!
fi

RVIZ_AUTO_STARTED_ARG=()
if [[ "${RVIZ_MODE}" != "off" ]]; then
  if command -v rviz2 >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" || "${RVIZ_MODE}" == "force" ]]; then
    echo "Launching RViz with ${RVIZ_CONFIG}."
    rviz2 -d "${RVIZ_CONFIG}" >"${LOG_DIR}/rviz2.log" 2>&1 &
    RVIZ_PID=$!
    RVIZ_AUTO_STARTED_ARG=(--rviz-auto-started)
    echo "rviz2 pid: ${RVIZ_PID}"
  else
    echo "RViz was not launched automatically. Open RViz manually with ${RVIZ_CONFIG}."
  fi
fi

PLAYER_ARGS=(
  --route-contract "${ROUTE_CONTRACT}"
  --output-dir "${OUT_DIR}"
  --entity-name "${ENTITY_NAME}"
  --frame-id world
  --base-frame rslg_cross_floor_base_link
  --horizontal-speed "${HORIZONTAL_SPEED}"
  --vertical-speed-limit "${VERTICAL_SPEED}"
  --publish-rate "${PUBLISH_RATE}"
  --z-visual-scale "${Z_VISUAL_SCALE}"
  --hold-final-sec "${HOLD_FINAL_SEC}"
  --command-label "${ORIGINAL_COMMAND}"
  "${RVIZ_AUTO_STARTED_ARG[@]}"
)

if [[ "${GAZEBO_MODE}" == "off" ]]; then
  PLAYER_ARGS+=(--dry-run)
fi

"${PY}" "${PLAYER}" "${PLAYER_ARGS[@]}"

echo "task24g outputs written under: ${OUT_DIR}"
echo "Report: ${OUT_DIR}/task24g_report.md"
echo "Final answer: ${OUT_DIR}/final_answer_for_user.md"

if [[ "${KEEP_OPEN}" == "1" ]]; then
  echo "Keeping Gazebo/RViz processes open. Press Ctrl-C to end the demo."
  while true; do
    sleep 1
  done
fi
