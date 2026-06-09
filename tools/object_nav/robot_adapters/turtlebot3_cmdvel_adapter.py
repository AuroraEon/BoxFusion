"""TurtleBot3 `/cmd_vel` adapter for the lightweight object-navigation runner."""

from __future__ import annotations

import math
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .base_adapter import BaseRobotAdapter


NAV2_ACTIONS = {"/compute_path_to_pose", "/follow_path", "/navigate_to_pose"}
NAV2_PROCESS_PATTERN = re.compile(
    r"planner_server|controller_server|bt_navigator|behavior_server|recoveries_server|"
    r"waypoint_follower|nav2_map_server|lifecycle_manager_navigation|nav2_costmap"
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    import json

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def yaw_from_q(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def process_snapshot() -> dict[str, Any]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,stat=,comm=,args="],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    relevant = [
        line.strip()
        for line in result.stdout.splitlines()
        if re.search(r"gzserver|gazebo|turtlebot3|nav2_|planner_server|controller_server|bt_navigator|costmap", line, re.I)
        and "ps -eo" not in line
    ]
    forbidden = [line for line in relevant if NAV2_PROCESS_PATTERN.search(line)]
    return {
        "relevant_processes": relevant,
        "forbidden_nav2_processes": forbidden,
        "no_nav2_processes_present": not forbidden,
    }


def graph_snapshot(root: Path, env: dict[str, str]) -> dict[str, Any]:
    def capture(command: list[str]) -> dict[str, Any]:
        try:
            result = subprocess.run(
                command,
                cwd=root,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10,
            )
            return {"stdout": result.stdout, "returncode": result.returncode}
        except Exception as exc:
            return {"stdout": "", "returncode": None, "error": str(exc)}

    actions = capture(["ros2", "action", "list"])
    action_rows = {line.strip() for line in (actions.get("stdout") or "").splitlines() if line.strip()}
    topics = capture(["ros2", "topic", "list"])
    nodes = capture(["ros2", "node", "list"])
    forbidden_actions = sorted(action_rows.intersection(NAV2_ACTIONS))
    forbidden_nodes = sorted(
        {line.strip() for line in (nodes.get("stdout") or "").splitlines() if NAV2_PROCESS_PATTERN.search(line)}
    )
    return {
        "topics": topics,
        "actions": actions,
        "nodes": nodes,
        "forbidden_nav2_actions": forbidden_actions,
        "forbidden_nav2_nodes": forbidden_nodes,
        "no_nav2_actions_or_nodes_present": not forbidden_actions and not forbidden_nodes,
    }


class TurtleBot3CmdVelAdapter(BaseRobotAdapter):
    """Behavior-preserving adapter for task18 TurtleBot3 Burger runtime."""

    def __init__(
        self,
        profile: dict[str, Any],
        *,
        root: Path,
        out: Path,
        stage_output_dir: Path,
        floor_id: str,
        map_yaml: Path,
        ros_domain_id: str,
        gui: bool,
        top_level_command: list[str],
    ) -> None:
        super().__init__(profile)
        self.root = root
        self.out = out
        self.stage_output_dir = stage_output_dir
        self.floor_id = floor_id
        self.map_yaml = map_yaml
        self.ros_domain_id = str(ros_domain_id)
        self.gui = gui
        self.top_level_command = list(top_level_command)
        self.env = self._build_env()
        self.runtime_profile = (
            self.stage_output_dir / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
        )
        self.launcher = self._resolve_profile_path(str(profile["spawn_launch"]))
        self.start_command = self._build_start_command()
        self.stop_command = self._build_stop_command()
        self.launch_result: subprocess.CompletedProcess[str] | None = None
        self.graph: dict[str, Any] = {}
        self.process_after: dict[str, Any] = {}
        self.ready = False
        self.node = None

    def _resolve_profile_path(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else self.root / path

    def _build_env(self) -> dict[str, str]:
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = self.ros_domain_id
        for key, value in (self.profile.get("required_env") or {}).items():
            env[str(key)] = str(value)
        env.setdefault("TURTLEBOT3_MODEL", "burger")
        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
        os.environ["TURTLEBOT3_MODEL"] = env["TURTLEBOT3_MODEL"]
        return env

    def _build_start_command(self) -> list[str]:
        return [
            str(self.launcher),
            "--stage-output-dir",
            str(self.stage_output_dir),
            "--floor-id",
            self.floor_id,
            "--map-yaml",
            str(self.map_yaml),
            "--runtime-profile",
            str(self.runtime_profile),
            "--ros-domain-id",
            self.ros_domain_id,
            "--log-dir",
            str(self.out / "bringup_logs"),
            "--gui" if self.gui else "--headless",
        ]

    def _build_stop_command(self) -> list[str]:
        return [
            str(self.launcher),
            "--stage-output-dir",
            str(self.stage_output_dir),
            "--runtime-profile",
            str(self.runtime_profile),
            "--ros-domain-id",
            self.ros_domain_id,
            "--log-dir",
            str(self.out / "bringup_logs"),
            "--stop",
        ]

    def start_bringup(self) -> dict[str, Any]:
        pre_process = process_snapshot()
        write_json(self.out / "process_list_before_bringup.json", pre_process)
        write_json(
            self.out / "exact_runtime_commands.json",
            {
                "artifact_type": "task17c_exact_runtime_commands",
                "created_utc": now_iso(),
                "top_level_command": self.top_level_command,
                "launcher_start": self.start_command,
                "launcher_stop": self.stop_command,
                "prohibited_nav2_actions": sorted(NAV2_ACTIONS),
                "robot_profile": self.profile.get("robot_name"),
                "robot_profile_path": self.profile.get("profile_path"),
            },
        )
        self.launch_result = subprocess.run(
            self.start_command,
            cwd=self.root,
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        write_text(self.out / "no_nav2_bringup.log", self.launch_result.stdout)
        return {
            "returncode": self.launch_result.returncode,
            "stdout": self.launch_result.stdout,
            "start_command": self.start_command,
        }

    def wait_until_ready(self) -> dict[str, Any]:
        self.graph = self.collect_runtime_graph()
        self.process_after = process_snapshot()
        topics = self.graph.get("topics", {}).get("stdout") or ""
        launch_ok = bool(self.launch_result and self.launch_result.returncode == 0)
        self.ready = bool(
            launch_ok
            and self.graph["no_nav2_actions_or_nodes_present"]
            and self.process_after["no_nav2_processes_present"]
            and "/clock" in topics
        )
        report = {
            "artifact_type": "task17c_bringup_readiness",
            "created_utc": now_iso(),
            "gazebo_started_without_nav2": self.ready,
            "failure_reason": None if self.ready else "bringup failed",
            "robot_profile": self.profile.get("robot_name"),
        }
        write_json(self.out / "bringup_readiness.json", report)
        return report

    def stop_bringup(self) -> dict[str, Any]:
        result = subprocess.run(
            self.stop_command,
            cwd=self.root,
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return {"returncode": result.returncode, "stdout": result.stdout, "stop_command": self.stop_command}

    def initialize_ros_node(self) -> Any:
        if self.node is None:
            self.node = self._make_control_node()
        return self.node

    def _make_control_node(self) -> Any:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.duration import Duration
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformException, TransformListener

        profile = self.profile
        cmd_topic = str(profile["cmd_topic"])
        map_frame = str(profile["map_frame"])
        base_frame = str(profile["base_frame"])
        source = f"/tf {map_frame}->{base_frame}"

        class ControlNode(Node):
            def __init__(self) -> None:
                super().__init__("task18_lightweight_executor")
                self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.publisher = self.create_publisher(Twist, cmd_topic, 10)

            def pose(self) -> dict[str, Any] | None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        map_frame,
                        base_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.15),
                    )
                    return {
                        "x": float(transform.transform.translation.x),
                        "y": float(transform.transform.translation.y),
                        "yaw": yaw_from_q(transform.transform.rotation),
                        "frame_id": map_frame,
                        "source": source,
                        "stamp_ns": int(transform.header.stamp.sec) * 1000000000
                        + int(transform.header.stamp.nanosec),
                    }
                except TransformException:
                    return None

            def command(self, linear: float, angular: float) -> None:
                msg = Twist()
                msg.linear.x = float(linear)
                msg.angular.z = float(angular)
                self.publisher.publish(msg)

            def stop(self) -> None:
                for _ in range(4):
                    self.command(0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.03)

        return ControlNode()

    def destroy_ros_node(self) -> None:
        if self.node is not None:
            self.node.stop()
            self.node.destroy_node()
            self.node = None

    def get_pose(self) -> dict[str, Any] | None:
        if self.node is None:
            return None
        return self.node.pose()

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        if self.node is None:
            raise RuntimeError("ROS node is not initialized")
        self.node.command(linear_x, angular_z)

    def stop(self) -> None:
        if self.node is not None:
            self.node.stop()

    def collect_runtime_graph(self) -> dict[str, Any]:
        graph = graph_snapshot(self.root, self.env)
        write_json(self.out / "ros_graph_no_nav2.json", graph)
        return graph

    def validate_no_nav2(self) -> dict[str, Any]:
        graph = self.graph or self.collect_runtime_graph()
        processes = self.process_after or process_snapshot()
        return {
            "no_nav2_action_servers_active": not graph.get("forbidden_nav2_actions"),
            "no_nav2_actions_or_nodes_present": graph.get("no_nav2_actions_or_nodes_present"),
            "no_nav2_processes_present": processes.get("no_nav2_processes_present"),
            "forbidden_nav2_actions": graph.get("forbidden_nav2_actions", []),
            "forbidden_nav2_nodes": graph.get("forbidden_nav2_nodes", []),
            "forbidden_nav2_processes": processes.get("forbidden_nav2_processes", []),
        }

    def report_dataplane_status(self) -> dict[str, Any]:
        return {
            "robot_profile": self.profile.get("robot_name"),
            "cmd_topic": self.profile.get("cmd_topic"),
            "cmd_msg_type": self.profile.get("cmd_msg_type"),
            "pose_source": self.profile.get("pose_source"),
            "map_frame": self.profile.get("map_frame"),
            "base_frame": self.profile.get("base_frame"),
            "supports_cmd_vel": self.profile.get("supports_cmd_vel"),
            "requires_gait_controller": self.profile.get("requires_gait_controller"),
            "ready": self.ready,
        }
