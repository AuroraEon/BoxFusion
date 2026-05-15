#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/ws/workspace/BoxFusion}"
STAGE_OUTPUT_DIR="$REPO_ROOT/stage_outputs/stage1_00824_step30p1"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL:-burger}"
GUI="false"
ACTION="start"
CLEAN_FIRST=1
SPAWN_X="5.3067"
SPAWN_Y="2.1181"
SPAWN_Z="0.08"
SPAWN_YAW="-2.221151"
LOG_DIR_OVERRIDE=""
MAP_PROFILE="stable"
MAP_YAML_OVERRIDE=""
READINESS_TIMEOUT_SEC=90
READINESS_OUTPUT_JSON=""
READINESS_OUTPUT_MD=""

usage() {
  cat <<'EOF'
Usage:
  launch_stage1_step30p1_gazebo_nav2.sh [--stage-output-dir DIR] [--ros-domain-id ID] [--gui] [--headless] [--map-profile stable|h8r2|step30s7_request_aware] [--log-dir DIR] [--stop|--status]

Starts the post-restructure 00824 Step30P1 Gazebo/TurtleBot3/staticloc/Nav2 runtime.
Logs and pid files are written under stage_outputs/stage1_00824_step30p1/current_validation/bringup_logs/.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --gui) GUI="true"; shift ;;
    --headless|--no-gui) GUI="false"; shift ;;
    --stop) ACTION="stop"; shift ;;
    --status) ACTION="status"; shift ;;
    --no-clean) CLEAN_FIRST=0; shift ;;
    --spawn-x) SPAWN_X="$2"; shift 2 ;;
    --spawn-y) SPAWN_Y="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --spawn-yaw) SPAWN_YAW="$2"; shift 2 ;;
    --map-profile) MAP_PROFILE="$2"; shift 2 ;;
    --map-yaml) MAP_YAML_OVERRIDE="$2"; shift 2 ;;
    --log-dir) LOG_DIR_OVERRIDE="$2"; shift 2 ;;
    --readiness-timeout-sec) READINESS_TIMEOUT_SEC="$2"; shift 2 ;;
    --readiness-output-json) READINESS_OUTPUT_JSON="$2"; shift 2 ;;
    --readiness-output-md) READINESS_OUTPUT_MD="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[stage1_step30p1][ERROR] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$STAGE_OUTPUT_DIR" != /* ]]; then
  STAGE_OUTPUT_DIR="$REPO_ROOT/$STAGE_OUTPUT_DIR"
fi

WORLD="$STAGE_OUTPUT_DIR/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf"
MAP_YAML="$STAGE_OUTPUT_DIR/maps/stage1_full_scene_occupancy_map.yaml"
if [ "$MAP_PROFILE" = "step30s5_room15_diagnostic_patch" ] || [ "$MAP_PROFILE" = "step30s5_room15_interior" ]; then
  echo "[stage1_step30p1][ERROR] Step30S5 patched map profiles were removed from active Step30S7 runtime use." >&2
  exit 12
fi
if [ "$MAP_PROFILE" = "stable" ] || [ "$MAP_PROFILE" = "full_scene" ] || [ "$MAP_PROFILE" = "stage1_full_scene" ] || [ "$MAP_PROFILE" = "stage1_full_scene_occupancy" ] || [ "$MAP_PROFILE" = "auto" ]; then
  MAP_PROFILE="stage1_full_scene_occupancy"
  MAP_YAML="$STAGE_OUTPUT_DIR/maps/stage1_full_scene_occupancy_map.yaml"
elif [ "$MAP_PROFILE" = "step30s7_request_aware" ] || [ "$MAP_PROFILE" = "request_aware" ]; then
  MAP_YAML="$STAGE_OUTPUT_DIR/maps/step30s7_request_aware_nav_map.yaml"
elif [ "$MAP_PROFILE" = "h8r2" ] || [ "$MAP_PROFILE" = "reference_h8r2" ]; then
  MAP_YAML="$STAGE_OUTPUT_DIR/maps/h8r2_gateway_preserving_nav_map.yaml"
elif [ "$MAP_PROFILE" != "h8r2" ] && [ "$MAP_PROFILE" != "reference_h8r2" ]; then
  echo "[stage1_step30p1][ERROR] unsupported --map-profile: $MAP_PROFILE" >&2
  exit 12
fi
if [ -n "$MAP_YAML_OVERRIDE" ]; then
  MAP_YAML="$MAP_YAML_OVERRIDE"
  if [[ "$MAP_YAML" != /* ]]; then
    MAP_YAML="$REPO_ROOT/$MAP_YAML"
  fi
fi
NAV2_PARAMS="$STAGE_OUTPUT_DIR/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml"
NAV2_LAUNCH="$STAGE_OUTPUT_DIR/nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py"
VALIDATION_DIR="$STAGE_OUTPUT_DIR/current_validation"
LOG_DIR="$VALIDATION_DIR/bringup_logs"
if [ -n "$LOG_DIR_OVERRIDE" ]; then
  LOG_DIR="$LOG_DIR_OVERRIDE"
  if [[ "$LOG_DIR" != /* ]]; then
    LOG_DIR="$REPO_ROOT/$LOG_DIR"
  fi
fi
PID_DIR="$LOG_DIR/pids"
mkdir -p "$LOG_DIR" "$PID_DIR"
if [ -z "$READINESS_OUTPUT_JSON" ]; then
  READINESS_OUTPUT_JSON="$LOG_DIR/lifecycle_readiness_report_v0_1.json"
fi
if [ -z "$READINESS_OUTPUT_MD" ]; then
  READINESS_OUTPUT_MD="$LOG_DIR/lifecycle_readiness_report_v0_1.md"
fi
if [[ "$READINESS_OUTPUT_JSON" != /* ]]; then
  READINESS_OUTPUT_JSON="$REPO_ROOT/$READINESS_OUTPUT_JSON"
fi
if [[ "$READINESS_OUTPUT_MD" != /* ]]; then
  READINESS_OUTPUT_MD="$REPO_ROOT/$READINESS_OUTPUT_MD"
fi

source_ros_setup() {
  local had_nounset=0
  case "$-" in *u*) had_nounset=1 ;; esac
  set +u
  source /opt/ros/foxy/setup.bash
  if [ "$had_nounset" = "1" ]; then set -u; else set +u; fi
}

export PATH="/usr/bin:/usr/local/bin:$PATH"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL_VALUE"
source_ros_setup

TB3_GAZEBO_SHARE="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo"
TB3_MODELS="$TB3_GAZEBO_SHARE/models"
TB3_RSP_LAUNCH="$TB3_GAZEBO_SHARE/launch/robot_state_publisher.launch.py"
TB3_SDF_MODEL="$TB3_MODELS/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
export GAZEBO_MODEL_PATH="$TB3_MODELS:$STAGE_OUTPUT_DIR/nav2/worlds:${GAZEBO_MODEL_PATH:-}"

pid_file() { printf '%s/%s.pid' "$PID_DIR" "$1"; }

stop_one() {
  local name="$1"
  local file
  file="$(pid_file "$name")"
  if [ -s "$file" ]; then
    local pid
    pid="$(cat "$file")"
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  fi
}

stop_all() {
  for name in nav2 static_tf robot_state_publisher gazebo; do
    stop_one "$name"
  done
  (pgrep -f "$WORLD" 2>/dev/null || true) | while read -r pid; do
    if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  for pattern in "turtlebot3_burger.urdf" "static_transform_publisher 0 0 0 0 0 0 map odom" "turtlebot3_diff_drive" "turtlebot3_laserscan" "lib/nav2_map_server/map_server" "lib/nav2_controller/controller_server" "lib/nav2_planner/planner_server" "lib/nav2_recoveries/recoveries_server" "lib/nav2_bt_navigator/bt_navigator" "lib/nav2_waypoint_follower/waypoint_follower" "lifecycle_manager_navigation"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
        kill "$pid" >/dev/null 2>&1 || true
      fi
    done
  done
  for pattern in "gzclient" "gzserver" "gazebo.*00824_step30p1_large_continuous_floor_world"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
        kill "$pid" >/dev/null 2>&1 || true
      fi
    done
  done
  sleep 2
  for name in nav2 static_tf robot_state_publisher gazebo; do
    local file pid
    file="$(pid_file "$name")"
    if [ -s "$file" ]; then
      pid="$(cat "$file")"
      if kill -0 "$pid" >/dev/null 2>&1; then
        kill -9 "$pid" >/dev/null 2>&1 || true
      fi
    fi
  done
  (pgrep -f "$WORLD" 2>/dev/null || true) | while read -r pid; do
    if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
      kill -9 "$pid" >/dev/null 2>&1 || true
    fi
  done
  for pattern in "turtlebot3_burger.urdf" "static_transform_publisher 0 0 0 0 0 0 map odom" "turtlebot3_diff_drive" "turtlebot3_laserscan" "lib/nav2_map_server/map_server" "lib/nav2_controller/controller_server" "lib/nav2_planner/planner_server" "lib/nav2_recoveries/recoveries_server" "lib/nav2_bt_navigator/bt_navigator" "lib/nav2_waypoint_follower/waypoint_follower" "lifecycle_manager_navigation"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
        kill -9 "$pid" >/dev/null 2>&1 || true
      fi
    done
  done
  for pattern in "gzclient" "gzserver" "gazebo.*00824_step30p1_large_continuous_floor_world"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ]; then
        kill -9 "$pid" >/dev/null 2>&1 || true
      fi
    done
  done
}

status_all() {
  for name in gazebo robot_state_publisher static_tf nav2; do
    file="$(pid_file "$name")"
    if [ -s "$file" ] && kill -0 "$(cat "$file")" >/dev/null 2>&1; then
      echo "$name: running pid=$(cat "$file")"
    else
      echo "$name: not running"
    fi
  done
}

wait_for_service() {
  local service="$1"
  local seconds="$2"
  for _ in $(seq 1 "$seconds"); do
    if ros2 service list 2>/dev/null | grep -qx "$service"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic_once() {
  local topic="$1"
  local seconds="$2"
  timeout "${seconds}s" /usr/bin/python3 - "$topic" <<'PY' >/dev/null 2>&1
import sys
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

topic = sys.argv[1]
rclpy.init(args=None)
node = Node("boxfusion_stage1_step30p1_wait_for_clock")
count = {"value": 0}
node.create_subscription(Clock, topic, lambda _msg: count.__setitem__("value", count["value"] + 1), 10)
while rclpy.ok() and count["value"] < 1:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
PY
}

if [ "$ACTION" = "stop" ]; then
  stop_all
  status_all
  exit 0
fi

if [ "$ACTION" = "status" ]; then
  status_all
  exit 0
fi

test -f "$WORLD" || { echo "[stage1_step30p1][ERROR] Missing world: $WORLD" >&2; exit 10; }
test -f "$MAP_YAML" || { echo "[stage1_step30p1][ERROR] Missing map: $MAP_YAML" >&2; exit 10; }
test -f "$NAV2_PARAMS" || { echo "[stage1_step30p1][ERROR] Missing Nav2 params: $NAV2_PARAMS" >&2; exit 10; }
test -f "$NAV2_LAUNCH" || { echo "[stage1_step30p1][ERROR] Missing Nav2 launch: $NAV2_LAUNCH" >&2; exit 10; }
test -f "$TB3_SDF_MODEL" || { echo "[stage1_step30p1][ERROR] Missing TurtleBot3 SDF: $TB3_SDF_MODEL" >&2; exit 10; }

if [ "$CLEAN_FIRST" = "1" ]; then
  stop_all
fi

if [ "$GUI" = "true" ] && [ -z "${DISPLAY:-}" ]; then
  echo "[stage1_step30p1][ERROR] --gui requested but DISPLAY is not set; Gazebo client cannot open." >&2
  exit 18
fi

echo "[stage1_step30p1] starting Gazebo world: $WORLD"
setsid ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" gui:="$GUI" > "$LOG_DIR/gazebo.log" 2>&1 &
echo "$!" > "$(pid_file gazebo)"

wait_for_service /spawn_entity 45 || wait_for_service /gazebo/spawn_entity 5 || {
  echo "[stage1_step30p1][ERROR] Gazebo spawn service did not appear. See $LOG_DIR/gazebo.log" >&2
  exit 20
}
if ros2 service list | grep -qx /unpause_physics; then
  timeout 10s ros2 service call /unpause_physics std_srvs/srv/Empty "{}" > "$LOG_DIR/unpause_physics.log" 2>&1 || true
elif ros2 service list | grep -qx /gazebo/unpause_physics; then
  timeout 10s ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty "{}" > "$LOG_DIR/unpause_physics.log" 2>&1 || true
fi
wait_for_topic_once /clock 30 || {
  echo "[stage1_step30p1][ERROR] /clock did not publish. See $LOG_DIR/gazebo.log" >&2
  exit 21
}
if [ "$GUI" = "true" ]; then
  gui_seen=0
  for _ in $(seq 1 30); do
    if pgrep -f "gzclient" >/dev/null 2>&1; then
      gui_seen=1
      break
    fi
    sleep 1
  done
  if [ "$gui_seen" != "1" ]; then
    ps -eo pid,ppid,stat,comm,args | grep -E 'gzclient|gzserver|gazebo' | grep -v grep > "$LOG_DIR/gazebo_processes_after_gui_failure.txt" 2>/dev/null || true
    echo "[stage1_step30p1][ERROR] --gui requested but no gzclient process was observed. See $LOG_DIR/gazebo.log" >&2
    exit 23
  fi
fi

echo "[stage1_step30p1] starting robot_state_publisher"
setsid ros2 launch "$TB3_RSP_LAUNCH" use_sim_time:=true > "$LOG_DIR/robot_state_publisher.log" 2>&1 &
echo "$!" > "$(pid_file robot_state_publisher)"
sleep 2

echo "[stage1_step30p1] spawning TurtleBot3 ${TURTLEBOT3_MODEL}"
timeout 60s ros2 run gazebo_ros spawn_entity.py \
  -file "$TB3_SDF_MODEL" \
  -entity "turtlebot3_${TURTLEBOT3_MODEL}" \
  -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y "$SPAWN_YAW" \
  > "$LOG_DIR/spawn_entity.log" 2>&1 || {
    echo "[stage1_step30p1][ERROR] TurtleBot3 spawn failed. See $LOG_DIR/spawn_entity.log" >&2
    exit 22
  }

echo "[stage1_step30p1] starting static map->odom TF"
setsid ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > "$LOG_DIR/static_map_odom_tf.log" 2>&1 &
echo "$!" > "$(pid_file static_tf)"

echo "[stage1_step30p1] starting Nav2 staticloc stack"
setsid ros2 launch "$NAV2_LAUNCH" map:="$MAP_YAML" params_file:="$NAV2_PARAMS" use_sim_time:=true autostart:=true > "$LOG_DIR/nav2.log" 2>&1 &
echo "$!" > "$(pid_file nav2)"

cat > "$LOG_DIR/bringup_result.json" <<EOF
{
  "artifact_type": "step30s2_bringup_launch_result",
  "stage_output_dir": "$STAGE_OUTPUT_DIR",
  "world": "$WORLD",
  "map": "$MAP_YAML",
  "map_profile": "$MAP_PROFILE",
  "nav2_params": "$NAV2_PARAMS",
  "nav2_launch": "$NAV2_LAUNCH",
  "turtlebot3_model_sdf": "$TB3_SDF_MODEL",
  "ros_domain_id": "$ROS_DOMAIN_ID",
  "gui": $GUI,
  "spawn_pose": {"x": $SPAWN_X, "y": $SPAWN_Y, "z": $SPAWN_Z, "yaw": $SPAWN_YAW},
  "log_dir": "$LOG_DIR",
  "readiness_report_json": "$READINESS_OUTPUT_JSON",
  "readiness_report_md": "$READINESS_OUTPUT_MD",
  "readiness_timeout_sec": $READINESS_TIMEOUT_SEC,
  "readiness_succeeded": false
}
EOF

echo "[stage1_step30p1] waiting for Nav2 lifecycle/map/FollowPath readiness"
/usr/bin/python3 "$REPO_ROOT/tools/stage1_step30p1/wait_stage1_step30p1_nav2_readiness.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --timeout-sec "$READINESS_TIMEOUT_SEC" \
  --output-json "$READINESS_OUTPUT_JSON" \
  --output-md "$READINESS_OUTPUT_MD" \
  > "$LOG_DIR/lifecycle_readiness_wait.log" 2>&1 || {
    echo "[stage1_step30p1][ERROR] Nav2 lifecycle readiness failed. See $READINESS_OUTPUT_JSON and $LOG_DIR/nav2.log" >&2
    exit 24
  }

cat > "$LOG_DIR/bringup_result.json" <<EOF
{
  "artifact_type": "step30s2_bringup_launch_result",
  "stage_output_dir": "$STAGE_OUTPUT_DIR",
  "world": "$WORLD",
  "map": "$MAP_YAML",
  "map_profile": "$MAP_PROFILE",
  "nav2_params": "$NAV2_PARAMS",
  "nav2_launch": "$NAV2_LAUNCH",
  "turtlebot3_model_sdf": "$TB3_SDF_MODEL",
  "ros_domain_id": "$ROS_DOMAIN_ID",
  "gui": $GUI,
  "spawn_pose": {"x": $SPAWN_X, "y": $SPAWN_Y, "z": $SPAWN_Z, "yaw": $SPAWN_YAW},
  "log_dir": "$LOG_DIR",
  "readiness_report_json": "$READINESS_OUTPUT_JSON",
  "readiness_report_md": "$READINESS_OUTPUT_MD",
  "readiness_timeout_sec": $READINESS_TIMEOUT_SEC,
  "readiness_succeeded": true
}
EOF

echo "[stage1_step30p1] launched. Logs: $LOG_DIR"
status_all
