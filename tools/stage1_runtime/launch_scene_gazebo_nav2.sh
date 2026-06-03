#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SCENE_ID=""
FLOOR_ID="floor_2"
STAGE_OUTPUT_DIR=""
MAP_YAML=""
RUNTIME_PROFILE=""
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-84}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL:-burger}"
GUI="false"
ACTION="start"
WORLD_ONLY=0
CLEAN_FIRST=1
SPAWN_X=""
SPAWN_Y=""
SPAWN_Z="0.08"
SPAWN_YAW=""
SKIP_SPAWN=0
LOG_DIR=""
READINESS_TIMEOUT_SEC=90
READINESS_OUTPUT_JSON=""
READINESS_OUTPUT_MD=""
RUN_ID=""

usage() {
  cat <<'EOF'
Usage:
  launch_scene_gazebo_nav2.sh --stage-output-dir DIR --floor-id floor_2 [--runtime-profile FILE] [--world-only] [--gui|--headless] [--stop|--status]
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --scene-id) SCENE_ID="$2"; shift 2 ;;
    --floor-id) FLOOR_ID="$2"; shift 2 ;;
    --stage-output-dir) STAGE_OUTPUT_DIR="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --runtime-profile) RUNTIME_PROFILE="$2"; shift 2 ;;
    --ros-domain-id) ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --gui) GUI="true"; shift ;;
    --headless|--no-gui) GUI="false"; shift ;;
    --world-only) WORLD_ONLY=1; shift ;;
    --stop) ACTION="stop"; shift ;;
    --status) ACTION="status"; shift ;;
    --no-clean) CLEAN_FIRST=0; shift ;;
    --spawn-x) SPAWN_X="$2"; shift 2 ;;
    --spawn-y) SPAWN_Y="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --spawn-yaw) SPAWN_YAW="$2"; shift 2 ;;
    --skip-spawn) SKIP_SPAWN=1; shift ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --readiness-timeout-sec) READINESS_TIMEOUT_SEC="$2"; shift 2 ;;
    --readiness-output-json) READINESS_OUTPUT_JSON="$2"; shift 2 ;;
    --readiness-output-md) READINESS_OUTPUT_MD="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "[stage1_runtime][ERROR] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ -z "$STAGE_OUTPUT_DIR" ]; then
  echo "[stage1_runtime][ERROR] --stage-output-dir is required" >&2
  exit 2
fi
if [[ "$STAGE_OUTPUT_DIR" != /* ]]; then STAGE_OUTPUT_DIR="$REPO_ROOT/$STAGE_OUTPUT_DIR"; fi
if [ -z "$RUNTIME_PROFILE" ]; then RUNTIME_PROFILE="$STAGE_OUTPUT_DIR/runtime/profiles/${FLOOR_ID}_nav2/runtime_profile.json"; fi
if [[ "$RUNTIME_PROFILE" != /* ]]; then RUNTIME_PROFILE="$REPO_ROOT/$RUNTIME_PROFILE"; fi

profile_value() {
  /usr/bin/python3 - "$RUNTIME_PROFILE" "$1" <<'PY'
import json, sys
path, key = sys.argv[1:]
data = json.load(open(path, encoding="utf-8"))
print(data.get(key, ""))
PY
}

if [ -z "$SCENE_ID" ]; then SCENE_ID="$(profile_value scene_id)"; fi
WORLD="$(profile_value gazebo_world)"
NAV2_PARAMS="$(profile_value nav2_params)"
NAV2_LAUNCH="$(profile_value nav2_launch)"
if [ -z "$MAP_YAML" ]; then MAP_YAML="$(profile_value map_yaml)"; fi
if [ -z "$SPAWN_X" ]; then SPAWN_X="$(/usr/bin/python3 - "$RUNTIME_PROFILE" <<'PY'
import json, sys
p=json.load(open(sys.argv[1], encoding="utf-8")).get("spawn_pose", {})
print(p.get("x", 0.0))
PY
)"; fi
if [ -z "$SPAWN_Y" ]; then SPAWN_Y="$(/usr/bin/python3 - "$RUNTIME_PROFILE" <<'PY'
import json, sys
p=json.load(open(sys.argv[1], encoding="utf-8")).get("spawn_pose", {})
print(p.get("y", 0.0))
PY
)"; fi
if [ -z "$SPAWN_YAW" ]; then SPAWN_YAW="$(/usr/bin/python3 - "$RUNTIME_PROFILE" <<'PY'
import json, sys
p=json.load(open(sys.argv[1], encoding="utf-8")).get("spawn_pose", {})
print(p.get("yaw", 0.0))
PY
)"; fi

for var in WORLD NAV2_PARAMS NAV2_LAUNCH MAP_YAML; do
  value="${!var}"
  if [[ "$value" != /* ]]; then value="$REPO_ROOT/$value"; fi
  printf -v "$var" '%s' "$value"
done
if [ -z "$LOG_DIR" ]; then
  if [ -n "$RUN_ID" ]; then
    LOG_DIR="$STAGE_OUTPUT_DIR/runs/active/$RUN_ID/bringup_logs"
  else
    LOG_DIR="$STAGE_OUTPUT_DIR/runs/active/manual/bringup_logs"
  fi
fi
if [[ "$LOG_DIR" != /* ]]; then LOG_DIR="$REPO_ROOT/$LOG_DIR"; fi
PID_DIR="$LOG_DIR/pids"
mkdir -p "$LOG_DIR" "$PID_DIR"
if [ -z "$READINESS_OUTPUT_JSON" ]; then READINESS_OUTPUT_JSON="$LOG_DIR/lifecycle_readiness_report_v0_1.json"; fi
if [ -z "$READINESS_OUTPUT_MD" ]; then READINESS_OUTPUT_MD="$LOG_DIR/lifecycle_readiness_report_v0_1.md"; fi
if [[ "$READINESS_OUTPUT_JSON" != /* ]]; then READINESS_OUTPUT_JSON="$REPO_ROOT/$READINESS_OUTPUT_JSON"; fi
if [[ "$READINESS_OUTPUT_MD" != /* ]]; then READINESS_OUTPUT_MD="$REPO_ROOT/$READINESS_OUTPUT_MD"; fi

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
export GAZEBO_MODEL_PATH="$TB3_MODELS:$(dirname "$WORLD"):${GAZEBO_MODEL_PATH:-}"

pid_file() { printf '%s/%s.pid' "$PID_DIR" "$1"; }

stop_all() {
  for file in "$PID_DIR"/*.pid; do
    [ -e "$file" ] || continue
    pid="$(cat "$file" 2>/dev/null || true)"
    [ -n "$pid" ] && kill "$pid" >/dev/null 2>&1 || true
  done
  for pattern in "$WORLD" "runtime_world.sdf" "lib/nav2_map_server/map_server" "lib/nav2_controller/controller_server" "lib/nav2_planner/planner_server" "lib/nav2_recoveries/recoveries_server" "lib/nav2_bt_navigator/bt_navigator" "lib/nav2_waypoint_follower/waypoint_follower" "lifecycle_manager_navigation" "static_transform_publisher 0 0 0 0 0 0 map odom"; do
    (pgrep -f "$pattern" 2>/dev/null || true) | while read -r pid; do
      [ -n "$pid" ] && [ "$pid" != "$$" ] && kill "$pid" >/dev/null 2>&1 || true
    done
  done
  sleep 2
}

status_all() {
  ps -eo pid,ppid,stat,comm,args | grep -E 'gzserver|gazebo|rviz2|nav2_|bt_navigator|controller_server|planner_server|map_server|turtlebot3' | grep -v grep || true
}

wait_for_service() {
  local service="$1"; local seconds="$2"
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
node = Node("boxfusion_scene_wait_clock")
seen = {"value": False}
node.create_subscription(Clock, "/clock", lambda _msg: seen.__setitem__("value", True), 10)
while rclpy.ok() and not seen["value"]:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
PY
}

if [ "$ACTION" = "stop" ]; then stop_all; status_all; exit 0; fi
if [ "$ACTION" = "status" ]; then status_all; exit 0; fi

test -f "$WORLD" || { echo "[stage1_runtime][ERROR] Missing world: $WORLD" >&2; exit 10; }
test -f "$MAP_YAML" || { echo "[stage1_runtime][ERROR] Missing map: $MAP_YAML" >&2; exit 10; }
test -f "$NAV2_PARAMS" || { echo "[stage1_runtime][ERROR] Missing Nav2 params: $NAV2_PARAMS" >&2; exit 10; }
test -f "$NAV2_LAUNCH" || { echo "[stage1_runtime][ERROR] Missing Nav2 launch: $NAV2_LAUNCH" >&2; exit 10; }
test -f "$TB3_SDF_MODEL" || { echo "[stage1_runtime][ERROR] Missing TurtleBot3 SDF: $TB3_SDF_MODEL" >&2; exit 10; }

if [ "$CLEAN_FIRST" = "1" ]; then stop_all; fi
if [ "$GUI" = "true" ] && [ -z "${DISPLAY:-}" ]; then
  echo "[stage1_runtime][ERROR] --gui requested but DISPLAY is not set" >&2
  exit 18
fi

echo "[stage1_runtime] scene=$SCENE_ID floor=$FLOOR_ID"
echo "[stage1_runtime] starting Gazebo world: $WORLD"
{
  echo "[stage1_runtime] gazebo_world=$WORLD"
  echo "[stage1_runtime] gazebo_world_name=$(basename "$WORLD" .sdf)"
} > "$LOG_DIR/gazebo.log"
setsid ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" gui:="$GUI" >> "$LOG_DIR/gazebo.log" 2>&1 &
echo "$!" > "$(pid_file gazebo)"
wait_for_service /spawn_entity 45 || wait_for_service /gazebo/spawn_entity 5 || {
  echo "[stage1_runtime][ERROR] Gazebo spawn service did not appear. See $LOG_DIR/gazebo.log" >&2
  exit 20
}
wait_for_clock 30 || {
  echo "[stage1_runtime][ERROR] /clock did not publish. See $LOG_DIR/gazebo.log" >&2
  exit 21
}

cat > "$LOG_DIR/bringup_result.json" <<EOF
{
  "artifact_type": "scene_bringup_launch_result",
  "scene_id": "$SCENE_ID",
  "floor_id": "$FLOOR_ID",
  "world": "$WORLD",
  "map": "$MAP_YAML",
  "nav2_params": "$NAV2_PARAMS",
  "nav2_launch": "$NAV2_LAUNCH",
  "ros_domain_id": "$ROS_DOMAIN_ID",
  "gui": $GUI,
  "world_only": $WORLD_ONLY,
  "spawn_pose": {"x": $SPAWN_X, "y": $SPAWN_Y, "z": $SPAWN_Z, "yaw": $SPAWN_YAW},
  "log_dir": "$LOG_DIR",
  "readiness_succeeded": false
}
EOF

if [ "$WORLD_ONLY" = "1" ]; then
  echo "[stage1_runtime] world-only mode active; leaving Gazebo running for caller validation"
  status_all
  exit 0
fi

echo "[stage1_runtime] starting robot_state_publisher"
setsid ros2 launch "$TB3_RSP_LAUNCH" use_sim_time:=true > "$LOG_DIR/robot_state_publisher.log" 2>&1 &
echo "$!" > "$(pid_file robot_state_publisher)"
sleep 2

if [ "$SKIP_SPAWN" = "0" ] && grep -q "turtlebot3" "$WORLD"; then
  SKIP_SPAWN=1
fi
if [ "$SKIP_SPAWN" = "1" ]; then
  echo "[stage1_runtime] skipping TurtleBot3 spawn; world already contains runtime robot"
  echo "skipped: world already contains runtime robot" > "$LOG_DIR/spawn_entity.log"
else
  echo "[stage1_runtime] spawning TurtleBot3 ${TURTLEBOT3_MODEL}"
  timeout 60s ros2 run gazebo_ros spawn_entity.py \
    -file "$TB3_SDF_MODEL" -entity "turtlebot3_${TURTLEBOT3_MODEL}" \
    -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" -Y "$SPAWN_YAW" \
    > "$LOG_DIR/spawn_entity.log" 2>&1 || {
      echo "[stage1_runtime][ERROR] TurtleBot3 spawn failed. See $LOG_DIR/spawn_entity.log" >&2
      exit 22
    }
fi

echo "[stage1_runtime] starting static map->odom TF"
setsid ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > "$LOG_DIR/static_map_odom_tf.log" 2>&1 &
echo "$!" > "$(pid_file static_tf)"

echo "[stage1_runtime] starting Nav2 staticloc stack"
setsid ros2 launch "$NAV2_LAUNCH" map:="$MAP_YAML" params_file:="$NAV2_PARAMS" use_sim_time:=true autostart:=true > "$LOG_DIR/nav2.log" 2>&1 &
echo "$!" > "$(pid_file nav2)"

echo "[stage1_runtime] waiting for Nav2 lifecycle/map/FollowPath readiness"
/usr/bin/python3 "$SCRIPT_DIR/wait_scene_nav2_readiness.py" \
  --stage-output-dir "$STAGE_OUTPUT_DIR" \
  --timeout-sec "$READINESS_TIMEOUT_SEC" \
  --output-json "$READINESS_OUTPUT_JSON" \
  --output-md "$READINESS_OUTPUT_MD" \
  > "$LOG_DIR/lifecycle_readiness_wait.log" 2>&1 || {
    echo "[stage1_runtime][ERROR] Nav2 lifecycle readiness failed. See $READINESS_OUTPUT_JSON and $LOG_DIR/nav2.log" >&2
    exit 24
  }

/usr/bin/python3 - "$LOG_DIR/bringup_result.json" <<'PY'
import json, sys
p=sys.argv[1]
data=json.load(open(p, encoding="utf-8"))
data["readiness_succeeded"]=True
open(p, "w", encoding="utf-8").write(json.dumps(data, indent=2, sort_keys=True)+"\n")
PY
echo "[stage1_runtime] launched. Logs: $LOG_DIR"
status_all
