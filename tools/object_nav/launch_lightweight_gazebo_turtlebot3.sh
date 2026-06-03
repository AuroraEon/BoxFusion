#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

STAGE_OUTPUT_DIR=""
FLOOR_ID="floor_2"
MAP_YAML=""
RUNTIME_PROFILE=""
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL:-burger}"
LOG_DIR=""
GUI="false"
ACTION="start"
SPAWN_X=""
SPAWN_Y=""
SPAWN_Z="0.08"
SPAWN_YAW=""

usage() {
  printf '%s\n' "Usage: $0 --stage-output-dir DIR [--floor-id floor_2] [--map-yaml FILE] [--runtime-profile FILE] [--headless|--gui] [--stop|--status]"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --floor-id) FLOOR_ID="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --runtime-profile) RUNTIME_PROFILE="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --spawn-x) SPAWN_X="$2"; shift 2 ;;
    --spawn-y) SPAWN_Y="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --spawn-yaw) SPAWN_YAW="$2"; shift 2 ;;
    --gui) GUI="true"; shift ;;
    --headless|--no-gui) GUI="false"; shift ;;
    --stop) ACTION="stop"; shift ;;
    --status) ACTION="status"; shift ;;
    --help|-h) usage; exit 0 ;;
    *) printf '[task17][ERROR] unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ -z "$STAGE_OUTPUT_DIR" ]; then
  printf '[task17][ERROR] --stage-output-dir is required\n' >&2
  exit 2
fi
if [[ "$STAGE_OUTPUT_DIR" != /* ]]; then STAGE_OUTPUT_DIR="$REPO_ROOT/$STAGE_OUTPUT_DIR"; fi
if [ -z "$RUNTIME_PROFILE" ]; then
  RUNTIME_PROFILE="$STAGE_OUTPUT_DIR/runtime/profiles/${FLOOR_ID}_nav2_task12_controller_robust/runtime_profile.json"
fi
if [[ "$RUNTIME_PROFILE" != /* ]]; then RUNTIME_PROFILE="$REPO_ROOT/$RUNTIME_PROFILE"; fi

profile_value() {
  /usr/bin/python3 - "$RUNTIME_PROFILE" "$1" <<'PY'
import json
import sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
value = data.get(sys.argv[2], "")
if isinstance(value, dict):
    print(json.dumps(value))
else:
    print(value)
PY
}

WORLD="$(profile_value gazebo_world)"
if [[ "$WORLD" != /* ]]; then WORLD="$REPO_ROOT/$WORLD"; fi
if [ -z "$MAP_YAML" ]; then MAP_YAML="$STAGE_OUTPUT_DIR/maps/$FLOOR_ID/stage1_${FLOOR_ID}_stable_occupancy_map.yaml"; fi
if [[ "$MAP_YAML" != /* ]]; then MAP_YAML="$REPO_ROOT/$MAP_YAML"; fi
if [ -z "$LOG_DIR" ]; then LOG_DIR="$STAGE_OUTPUT_DIR/runs/active/task17_lightweight/bringup_logs"; fi
if [[ "$LOG_DIR" != /* ]]; then LOG_DIR="$REPO_ROOT/$LOG_DIR"; fi

read_spawn_field() {
  /usr/bin/python3 - "$RUNTIME_PROFILE" "$1" <<'PY'
import json
import sys
data = json.load(open(sys.argv[1], encoding="utf-8"))
print(data.get("spawn_pose", {}).get(sys.argv[2], 0.0))
PY
}
if [ -z "$SPAWN_X" ]; then SPAWN_X="$(read_spawn_field x)"; fi
if [ -z "$SPAWN_Y" ]; then SPAWN_Y="$(read_spawn_field y)"; fi
if [ -z "$SPAWN_YAW" ]; then SPAWN_YAW="$(read_spawn_field yaw)"; fi

PID_DIR="$LOG_DIR/pids"
mkdir -p "$LOG_DIR" "$PID_DIR"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL_VALUE"
export PATH="/usr/bin:/usr/local/bin:$PATH"
set +u
source /opt/ros/foxy/setup.bash
set -u

TB3_GAZEBO_SHARE="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo"
TB3_MODELS="$TB3_GAZEBO_SHARE/models"
TB3_RSP_LAUNCH="$TB3_GAZEBO_SHARE/launch/robot_state_publisher.launch.py"
TB3_SDF_MODEL="$TB3_MODELS/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
export GAZEBO_MODEL_PATH="$TB3_MODELS:$(dirname "$WORLD"):${GAZEBO_MODEL_PATH:-}"

stop_all() {
  for file in "$PID_DIR"/*.pid; do
    [ -e "$file" ] || continue
    pid="$(cat "$file" 2>/dev/null || true)"
    [ -n "$pid" ] && kill "$pid" >/dev/null 2>&1 || true
  done
  for pattern in "$WORLD" "turtlebot3_gazebo/launch/robot_state_publisher.launch.py" \
      "/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher" \
      "static_transform_publisher 0 0 0 0 0 0 map odom" \
      "lib/nav2_map_server/map_server" "lib/nav2_controller/controller_server" \
      "lib/nav2_planner/planner_server" "lib/nav2_bt_navigator/bt_navigator" \
      "lifecycle_manager_navigation"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      [ -n "$pid" ] && [ "$pid" != "$$" ] && kill "$pid" >/dev/null 2>&1 || true
    done
  done
  sleep 2
}

status_all() {
  ps -eo pid,ppid,stat,comm,args | grep -E 'gzserver|gazebo|nav2_|bt_navigator|controller_server|planner_server|map_server|turtlebot3|static_transform_publisher' | grep -v grep || true
}

wait_for_service() {
  local service="$1"
  local seconds="$2"
  for _ in $(seq 1 "$seconds"); do
    ros2 service list 2>/dev/null | grep -qx "$service" && return 0
    sleep 1
  done
  return 1
}

wait_for_clock() {
  timeout "${1}s" /usr/bin/python3 - <<'PY' >/dev/null 2>&1
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init()
node = Node("task17_wait_for_clock")
seen = {"clock": False}
node.create_subscription(Clock, "/clock", lambda _msg: seen.__setitem__("clock", True), 10)
while rclpy.ok() and not seen["clock"]:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
PY
}

if [ "$ACTION" = "stop" ]; then stop_all; status_all; exit 0; fi
if [ "$ACTION" = "status" ]; then status_all; exit 0; fi

test -f "$WORLD" || { printf '[task17][ERROR] missing world: %s\n' "$WORLD" >&2; exit 10; }
test -f "$MAP_YAML" || { printf '[task17][ERROR] missing stable map: %s\n' "$MAP_YAML" >&2; exit 10; }
test -f "$TB3_SDF_MODEL" || { printf '[task17][ERROR] missing TurtleBot3 SDF: %s\n' "$TB3_SDF_MODEL" >&2; exit 10; }
if [ "$GUI" = "true" ] && [ -z "${DISPLAY:-}" ]; then
  printf '[task17][ERROR] --gui requested without DISPLAY\n' >&2
  exit 18
fi

stop_all
printf '[task17] starting Gazebo without Nav2: %s\n' "$WORLD"
setsid ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" gui:="$GUI" > "$LOG_DIR/gazebo.log" 2>&1 &
printf '%s\n' "$!" > "$PID_DIR/gazebo.pid"
wait_for_service /spawn_entity 45 || wait_for_service /gazebo/spawn_entity 5 || exit 20
wait_for_clock 30 || exit 21

printf '[task17] starting robot state publisher and static map-to-odom transform\n'
setsid ros2 launch "$TB3_RSP_LAUNCH" use_sim_time:=true > "$LOG_DIR/robot_state_publisher.log" 2>&1 &
printf '%s\n' "$!" > "$PID_DIR/robot_state_publisher.pid"
sleep 2
if grep -q "turtlebot3" "$WORLD"; then
  printf '%s\n' "skipped: runtime world already contains TurtleBot3" > "$LOG_DIR/spawn_entity.log"
else
  timeout 60s ros2 run gazebo_ros spawn_entity.py \
    -file "$TB3_SDF_MODEL" -entity "turtlebot3_${TURTLEBOT3_MODEL}" \
    -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y "$SPAWN_YAW" \
    > "$LOG_DIR/spawn_entity.log" 2>&1 || exit 22
fi
setsid ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > "$LOG_DIR/static_map_odom_tf.log" 2>&1 &
printf '%s\n' "$!" > "$PID_DIR/static_tf.pid"
sleep 3
printf '[task17] lightweight bringup is running; Nav2 was not launched\n'
status_all
