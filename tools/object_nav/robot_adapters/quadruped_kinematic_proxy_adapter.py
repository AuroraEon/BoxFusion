"""CHAMP visual model plus planar kinematic proxy adapter."""

from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .base_adapter import BaseRobotAdapter
from .turtlebot3_cmdvel_adapter import (
    graph_snapshot,
    now_iso,
    process_snapshot,
    write_json,
    write_text,
    yaw_from_q,
)


class QuadrupedKinematicProxyAdapter(BaseRobotAdapter):
    """Visual-only CHAMP adapter driven by a planar kinematic proxy."""

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
        validation_mode: str = "route",
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
        self.validation_mode = validation_mode
        self.env = dict(os.environ)
        self.env["ROS_DOMAIN_ID"] = self.ros_domain_id
        os.environ["ROS_DOMAIN_ID"] = self.ros_domain_id
        self.processes: dict[str, subprocess.Popen[str]] = {}
        self.log_handles: list[Any] = []
        self.graph: dict[str, Any] = {}
        self.process_after: dict[str, Any] = {}
        self.ready = False
        self.spawn_success = False
        self.spawn_stdout = ""
        self.node = None
        self.runtime_dir = self.out / "runtime_artifacts"
        self.logs_dir = self.out / "launch_logs"
        self.proxy_manifest = self.runtime_dir / "quadruped_proxy_node_manifest.json"
        self.runtime_profile = (
            self.stage_output_dir / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
        )
        self.spawn_pose = self._load_spawn_pose()
        self.world = self._resolve_world()

    def _load_spawn_pose(self) -> dict[str, float]:
        if self.validation_mode == "empty":
            return {"x": 0.0, "y": 0.0, "yaw": 0.0}
        data = json.loads(self.runtime_profile.read_text(encoding="utf-8"))
        pose = data.get("spawn_pose") or {}
        return {key: float(pose.get(key, 0.0)) for key in ("x", "y", "yaw")}

    def _resolve_world(self) -> Path:
        if self.validation_mode == "empty":
            return Path(str(self.profile.get("empty_world") or "/usr/share/gazebo-11/worlds/empty.world"))
        data = json.loads(self.runtime_profile.read_text(encoding="utf-8"))
        return Path(data["gazebo_world"])

    def _resolve(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else self.root / path

    def _make_visual_only_urdf(self) -> Path:
        source = self._resolve(str(self.profile["model_urdf"]))
        mesh_dir = self._resolve(str(self.profile["model_mesh_dir"]))
        tree = ET.parse(str(source))
        robot = tree.getroot()
        robot.set("name", str(self.profile["robot_name"]))
        for mesh in robot.findall(".//mesh"):
            filename = mesh.get("filename")
            if filename:
                mesh.set("filename", (mesh_dir / Path(filename).name).as_uri())
        for joint in robot.findall("joint"):
            if joint.get("type") in {"revolute", "continuous", "prismatic"}:
                joint.set("type", "fixed")
                for child_name in ("axis", "limit", "dynamics", "safety_controller", "calibration", "mimic"):
                    child = joint.find(child_name)
                    if child is not None:
                        joint.remove(child)
        for gazebo in list(robot.findall("gazebo")):
            robot.remove(gazebo)
        gazebo = ET.SubElement(robot, "gazebo")
        ET.SubElement(gazebo, "static").text = "true"
        target = self.runtime_dir / "champ_reference_visual_only_fixed.urdf"
        target.parent.mkdir(parents=True, exist_ok=True)
        tree.write(str(target), encoding="utf-8", xml_declaration=True)
        return target

    def _make_proxy_world(self) -> Path:
        tree = ET.parse(str(self.world))
        root = tree.getroot()
        world = root.find("world") if root.tag == "sdf" else root
        if world is None:
            raise ValueError(f"Gazebo world element missing from {self.world}")
        for include in list(world.findall("include")):
            uri = include.findtext("uri") or ""
            if "turtlebot3" in uri:
                world.remove(include)
        for plugin in list(world.findall("plugin")):
            if plugin.get("filename") in {"libgazebo_ros_init.so", "libgazebo_ros_factory.so"}:
                world.remove(plugin)
        if not any(plugin.get("filename") == "libgazebo_ros_state.so" for plugin in world.findall("plugin")):
            ET.SubElement(
                world,
                "plugin",
                {"name": "task22_gazebo_ros_state", "filename": "libgazebo_ros_state.so"},
            )
        target = self.runtime_dir / f"{self.world.stem}_task22_proxy_state.sdf"
        target.parent.mkdir(parents=True, exist_ok=True)
        tree.write(str(target), encoding="utf-8", xml_declaration=True)
        return target

    def _start_process(self, name: str, command: list[str]) -> None:
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        handle = (self.logs_dir / f"{name}.log").open("w", encoding="utf-8")
        self.log_handles.append(handle)
        self.processes[name] = subprocess.Popen(
            command,
            cwd=self.root,
            env=self.env,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def _capture(self, command: list[str], timeout: float = 60.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            command,
            cwd=self.root,
            env=self.env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
        )

    def _wait_for(self, command: list[str], expected: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            result = self._capture(command, timeout=10.0)
            if expected in result.stdout:
                return True
            time.sleep(0.5)
        return False

    def start_bringup(self) -> dict[str, Any]:
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        write_json(self.out / "process_list_before_bringup.json", process_snapshot())
        visual_urdf = self._make_visual_only_urdf()
        proxy_world = self._make_proxy_world()
        gazebo_command = [
            "gzserver",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            str(proxy_world),
        ]
        self._start_process("gazebo", gazebo_command)
        gazebo_gui_command = ["gzclient"]
        if self.gui:
            self._start_process("gazebo_gui", gazebo_gui_command)
        spawn_service_ready = self._wait_for(["ros2", "service", "list"], "/spawn_entity", 45.0)
        state_service_ready = self._wait_for(["ros2", "service", "list"], "/set_entity_state", 15.0)
        spawn_command = [
            "/usr/bin/python3", "/opt/ros/foxy/lib/gazebo_ros/spawn_entity.py",
            "-file", str(visual_urdf),
            "-entity", str(self.profile["gazebo_entity_name"]),
            "-x", str(self.spawn_pose["x"]),
            "-y", str(self.spawn_pose["y"]),
            "-z", str(self.profile.get("gazebo_entity_z", 0.35)),
            "-Y", str(self.spawn_pose["yaw"]),
        ]
        spawn = self._capture(spawn_command, timeout=60.0) if spawn_service_ready else None
        self.spawn_success = bool(spawn and spawn.returncode == 0)
        self.spawn_stdout = spawn.stdout if spawn else "spawn service unavailable"
        write_text(self.logs_dir / "spawn_entity.log", self.spawn_stdout)
        proxy_command = [
            "/usr/bin/python3", "-m", "tools.object_nav.robot_adapters.quadruped_kinematic_proxy_node",
            "--robot-profile", str(self.profile["profile_path"]),
            "--initial-x", str(self.spawn_pose["x"]),
            "--initial-y", str(self.spawn_pose["y"]),
            "--initial-yaw", str(self.spawn_pose["yaw"]),
            "--log-path", str(self.out / "trajectory_logs/proxy_node_events.jsonl"),
            "--manifest-path", str(self.proxy_manifest),
        ]
        self._start_process("quadruped_proxy_node", proxy_command)
        commands = {
            "artifact_type": "task22_exact_runtime_commands",
            "created_utc": now_iso(),
            "top_level_command": self.top_level_command,
            "gazebo": gazebo_command,
            "gazebo_gui": gazebo_gui_command if self.gui else None,
            "spawn": spawn_command,
            "proxy_node": proxy_command,
            "world": str(self.world),
            "proxy_world": str(proxy_world),
            "validation_mode": self.validation_mode,
            "visual_only_urdf": str(visual_urdf),
        }
        write_json(self.out / "exact_runtime_commands.json", commands)
        return {
            "returncode": 0 if spawn and spawn.returncode == 0 else 1,
            "spawn_service_ready": spawn_service_ready,
            "state_service_ready": state_service_ready,
            "spawn_stdout": spawn.stdout if spawn else "",
        }

    def wait_until_ready(self) -> dict[str, Any]:
        deadline = time.monotonic() + 20.0
        topics = ""
        while time.monotonic() < deadline:
            topics = self._capture(["ros2", "topic", "list"], timeout=10.0).stdout
            if all(name in topics for name in ("/clock", str(self.profile["cmd_topic"]), "/odom", "/tf")):
                break
            time.sleep(0.5)
        services = self._capture(["ros2", "service", "list"], timeout=10.0).stdout
        self.graph = self.collect_runtime_graph()
        self.process_after = process_snapshot()
        proxy_alive = self.processes.get("quadruped_proxy_node") is not None and self.processes["quadruped_proxy_node"].poll() is None
        spawn_ok = self.spawn_success
        self.ready = bool(
            proxy_alive
            and spawn_ok
            and all(name in topics for name in ("/clock", str(self.profile["cmd_topic"]), "/odom", "/tf"))
            and "/set_entity_state" in services
            and self.graph.get("no_nav2_actions_or_nodes_present")
        )
        report = {
            "artifact_type": "task22_quadruped_proxy_bringup_readiness",
            "created_utc": now_iso(),
            "gazebo_started_without_nav2": self.ready,
            "gazebo_spawn_success": spawn_ok,
            "gazebo_set_entity_state_service": "/set_entity_state" if "/set_entity_state" in services else None,
            "proxy_node_alive": proxy_alive,
            "clock_available": "/clock" in topics,
            "failure_reason": None if self.ready else "quadruped proxy bringup dataplane incomplete",
            "robot_profile": self.profile.get("robot_name"),
        }
        write_json(self.out / "bringup_readiness.json", report)
        write_text(self.out / "topic_tf_odom_samples/topics.txt", topics)
        write_text(self.out / "topic_tf_odom_samples/services.txt", services)
        write_text(self.out / "topic_tf_odom_samples/model_list.txt", self.spawn_stdout)
        return report

    def stop_bringup(self) -> dict[str, Any]:
        self.stop()
        stopped = []
        for name in ("quadruped_proxy_node", "gazebo_gui", "gazebo"):
            process = self.processes.get(name)
            if process is None or process.poll() is not None:
                continue
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=8)
            except Exception:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            stopped.append(name)
        for handle in self.log_handles:
            handle.close()
        return {"returncode": 0, "stopped_processes": stopped}

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

        cmd_topic = str(self.profile["cmd_topic"])
        map_frame = str(self.profile["map_frame"])
        base_frame = str(self.profile["base_frame"])
        max_linear = float(self.profile["max_linear_speed"])
        max_angular = float(self.profile["max_angular_speed"])

        class ControlNode(Node):
            def __init__(self) -> None:
                super().__init__("task22_lightweight_executor")
                self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.publisher = self.create_publisher(Twist, cmd_topic, 10)

            def pose(self) -> dict[str, Any] | None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        map_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.15)
                    )
                    return {
                        "x": float(transform.transform.translation.x),
                        "y": float(transform.transform.translation.y),
                        "yaw": yaw_from_q(transform.transform.rotation),
                        "frame_id": map_frame,
                        "source": f"/tf {map_frame}->{base_frame}",
                        "stamp_ns": int(transform.header.stamp.sec) * 1000000000 + int(transform.header.stamp.nanosec),
                    }
                except TransformException:
                    return None

            def command(self, linear: float, angular: float) -> None:
                msg = Twist()
                msg.linear.x = max(-max_linear, min(max_linear, float(linear)))
                msg.angular.z = max(-max_angular, min(max_angular, float(angular)))
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
        return self.node.pose() if self.node is not None else None

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        if self.node is None:
            raise RuntimeError("ROS node is not initialized")
        self.node.command(linear_x, angular_z)

    def stop(self) -> None:
        if self.node is not None:
            self.node.stop()
            return
        try:
            self._capture(
                ["ros2", "topic", "pub", "--once", str(self.profile["cmd_topic"]), "geometry_msgs/msg/Twist", "{}"],
                timeout=5.0,
            )
        except Exception:
            pass

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
        manifest = {}
        if self.proxy_manifest.exists():
            manifest = json.loads(self.proxy_manifest.read_text(encoding="utf-8"))
        return {
            "robot_profile": self.profile.get("robot_name"),
            "adapter_type": self.profile.get("adapter_type"),
            "cmd_topic": self.profile.get("cmd_topic"),
            "cmd_msg_type": self.profile.get("cmd_msg_type"),
            "pose_source": self.profile.get("pose_source"),
            "map_frame": self.profile.get("map_frame"),
            "base_frame": self.profile.get("base_frame"),
            "supports_cmd_vel": self.profile.get("supports_cmd_vel"),
            "requires_gait_controller": self.profile.get("requires_gait_controller"),
            "is_physical_locomotion": self.profile.get("is_physical_locomotion"),
            "claim_boundary": self.profile.get("claim_boundary"),
            "ready": self.ready,
            "proxy_manifest": manifest,
        }
