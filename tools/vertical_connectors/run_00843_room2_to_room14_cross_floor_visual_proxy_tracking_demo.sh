#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PY="/usr/bin/python3"
PLAYER="${SCRIPT_DIR}/cross_floor_visual_tracking_player.py"
PLANNED_ROUTE="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/planned_occupancy_aware_3d_route_v0_1.json"
FALLBACK_ROUTE="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24f_visual_kinematic_proxy_cross_floor_traversal_feasibility_and_plan/cross_floor_3d_route_contract_v0_1.json"
OUT_DIR="${REPO_ROOT}/stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24h_cross_floor_visual_proxy_tracking_mode"
RUNTIME_DIR="${OUT_DIR}/runtime"
LOG_DIR="${OUT_DIR}/launch_logs"
PROFILE="${REPO_ROOT}/tools/object_nav/robot_profiles/champ_reference_kinematic_proxy.yaml"
ENTITY_NAME="rslg_cross_floor_quadruped_proxy_tracking"
URDF="${RUNTIME_DIR}/rslg_cross_floor_quadruped_proxy_tracking_visual_only_fixed.urdf"
WORLD="${RUNTIME_DIR}/task24h_empty_world_with_state_plugin.sdf"
SOURCE_WORLD="/usr/share/gazebo-11/worlds/empty.world"
RVIZ_CONFIG="${SCRIPT_DIR}/rviz_room2_to_room14_cross_floor_visual_proxy_tracking.rviz"
RVIZ_MODE="auto"
GAZEBO_MODE="auto"
KEEP_OPEN="0"
HOLD_FINAL_SEC="8.0"
PUBLISH_RATE="20.0"
LINEAR_SPEED_MAX="0.25"
ANGULAR_SPEED_MAX="0.55"
VERTICAL_SPEED_LIMIT="0.12"
LOOKAHEAD_DISTANCE="0.35"
Z_VISUAL_SCALE="1.0"
REALTIME_SCALE="1.0"
STATIC_TF_PID=""
GZSERVER_PID=""
GZCLIENT_PID=""
RVIZ_PID=""
ORIGINAL_COMMAND="tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh"
for arg in "$@"; do
  ORIGINAL_COMMAND+=" $(printf '%q' "${arg}")"
done

usage() {
  cat <<'EOF'
Usage:
  tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh [options]

Options:
  --no-rviz          Do not auto-start RViz.
  --rviz             Try to start RViz even if DISPLAY is not set.
  --no-gazebo        Do not launch Gazebo; run dry-run integration and report Gazebo blocker.
  --manual-demo      Recommended human GUI mode: realtime scale 2.0, hold final 30s, keep open.
  --demo-speed MODE  Demo preset: slow, normal, or fast.
                     slow:   human GUI mode; realtime scale 2.0, hold final 30s, keep open.
                     normal: default demo pace; realtime scale 1.0, hold final 8s.
                     fast:   validation-only; realtime scale 0.0, hold final 0s.
  --keep-open        Keep Gazebo/RViz open after traversal until Ctrl-C.
  --hold-final SEC   Keep publishing final pose for SEC seconds after traversal.
  --realtime-scale S Pace live SetEntityState updates by this multiplier.
EOF
}

apply_demo_speed() {
  local mode="$1"
  case "${mode}" in
    slow)
      REALTIME_SCALE="2.0"
      HOLD_FINAL_SEC="30"
      KEEP_OPEN="1"
      ;;
    normal)
      REALTIME_SCALE="1.0"
      HOLD_FINAL_SEC="8"
      ;;
    fast)
      REALTIME_SCALE="0.0"
      HOLD_FINAL_SEC="0"
      ;;
    *)
      echo "Unknown --demo-speed mode: ${mode}" >&2
      usage >&2
      exit 2
      ;;
  esac
}

require_option_value() {
  local option_name="$1"
  local option_value="${2:-}"
  if [[ -z "${option_value}" ]]; then
    echo "Missing value for ${option_name}" >&2
    usage >&2
    exit 2
  fi
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
    --manual-demo)
      apply_demo_speed slow
      shift
      ;;
    --demo-speed)
      require_option_value "$1" "${2:-}"
      apply_demo_speed "$2"
      shift 2
      ;;
    --keep-open)
      KEEP_OPEN="1"
      shift
      ;;
    --hold-final)
      require_option_value "$1" "${2:-}"
      HOLD_FINAL_SEC="$2"
      shift 2
      ;;
    --realtime-scale)
      require_option_value "$1" "${2:-}"
      REALTIME_SCALE="$2"
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

echo "RSLG-SLAM task24h cross-floor visual proxy tracking-mode demo"
echo "Project path: ${REPO_ROOT}"
echo "Planned route: ${PLANNED_ROUTE}"
echo "Fallback route contract: ${FALLBACK_ROUTE}"
echo "Output dir: ${OUT_DIR}"
echo "Gazebo entity: ${ENTITY_NAME}"
echo "Realtime scale: ${REALTIME_SCALE}"
echo "Hold final seconds: ${HOLD_FINAL_SEC}"
echo "Keep open: ${KEEP_OPEN}"
echo "Tracking mode enabled: true"
echo "occupancy_aware_same_floor_tracking=true"
echo "stair_connector_2p5d_tracking=true"
echo "Claim boundary: visual_kinematic_proxy_only"
echo "Claim boundary: topological_vertical_transition_only"
echo "physical_stair_climbing_supported=false"
echo "No gait, no footstep planning, no contact-based stair climbing, no real quadruped stair locomotion."
echo "Nav2 and AMCL will not be launched by this script."

"${PY}" "${PLAYER}" \
  --planned-route-json "${PLANNED_ROUTE}" \
  --fallback-route-contract "${FALLBACK_ROUTE}" \
  --robot-profile "${PROFILE}" \
  --entity-name "${ENTITY_NAME}" \
  --prepare-urdf "${URDF}" \
  --prepare-world "${WORLD}" \
  --source-world "${SOURCE_WORLD}" \
  --prepare-only

read -r SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW < <("${PY}" "${PLAYER}" --planned-route-json "${PLANNED_ROUTE}" --fallback-route-contract "${FALLBACK_ROUTE}" --print-start-pose)

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
    echo "Spawning or reusing visual-only CHAMP-shaped tracking proxy at integrated route start."
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
    echo "Gazebo GUI client was not launched automatically; gzserver tracking can still run."
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
  --planned-route-json "${PLANNED_ROUTE}"
  --fallback-route-contract "${FALLBACK_ROUTE}"
  --output-dir "${OUT_DIR}"
  --entity-name "${ENTITY_NAME}"
  --frame-id world
  --base-frame rslg_cross_floor_tracking_base_link
  --publish-rate "${PUBLISH_RATE}"
  --linear-speed-max "${LINEAR_SPEED_MAX}"
  --angular-speed-max "${ANGULAR_SPEED_MAX}"
  --vertical-speed-limit "${VERTICAL_SPEED_LIMIT}"
  --lookahead-distance "${LOOKAHEAD_DISTANCE}"
  --z-visual-scale "${Z_VISUAL_SCALE}"
  --hold-final-sec "${HOLD_FINAL_SEC}"
  --realtime-scale "${REALTIME_SCALE}"
  --command-label "${ORIGINAL_COMMAND}"
  "${RVIZ_AUTO_STARTED_ARG[@]}"
)

if [[ "${GAZEBO_MODE}" == "off" ]]; then
  PLAYER_ARGS+=(--dry-run)
fi

"${PY}" "${PLAYER}" "${PLAYER_ARGS[@]}"

echo "task24h outputs written under: ${OUT_DIR}"
echo "Report: ${OUT_DIR}/task24h_report.md"
echo "Final answer: ${OUT_DIR}/final_answer_for_user.md"

if [[ "${KEEP_OPEN}" == "1" ]]; then
  echo "Keeping Gazebo/RViz processes open. Press Ctrl-C to end the demo."
  wait
fi
