#!/usr/bin/env python3
"""Wait for the Stage1 static-localization Nav2 runtime to become usable."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


os.environ.setdefault("ROS_DOMAIN_ID", "84")
os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    from lifecycle_msgs.srv import GetState
    import rclpy
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


REQUIRED_LIFECYCLE_NODES = ["/map_server", "/controller_server", "/planner_server", "/bt_navigator"]
DIAGNOSTIC_LIFECYCLE_NODES = ["/recoveries_server", "/waypoint_follower"]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Stage1 Nav2 Lifecycle Readiness Report",
        "",
        f"Created: `{payload.get('created_utc')}`",
        f"Succeeded: `{payload.get('succeeded')}`",
        f"Failure reason: `{payload.get('failure_reason')}`",
        f"FollowPath-only runtime: `{payload.get('follow_path_only_runtime')}`",
        "",
        "## Lifecycle",
        "",
    ]
    for name, item in payload.get("lifecycle_states", {}).items():
        lines.append(f"- `{name}`: state=`{item.get('state')}` available=`{item.get('available')}` timed_out=`{item.get('timed_out')}`")
    lines.extend(["", "## Map And Actions", ""])
    map_sample = payload.get("map_sample") or {}
    lines.append(f"- `/map` OccupancyGrid received with TRANSIENT_LOCAL QoS: `{payload.get('map_received')}`")
    if map_sample:
        lines.append(f"- `/map` sample: `{map_sample.get('width')}x{map_sample.get('height')}` at `{map_sample.get('resolution')}` m/cell")
    for name, ready in payload.get("action_servers", {}).items():
        role = (payload.get("action_server_roles") or {}).get(name)
        lines.append(f"- `{name}`: ready=`{ready}` role=`{role}`")
    if payload.get("missing_conditions"):
        lines.extend(["", "## Missing Conditions", ""])
        lines.extend(f"- `{item}`" for item in payload["missing_conditions"])
    if payload.get("stuck_nodes"):
        lines.extend(["", "## Stuck Nodes", ""])
        lines.extend(f"- `{item}`" for item in payload["stuck_nodes"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def lifecycle_get(node: Node, node_name: str, timeout_sec: float = 0.5) -> dict[str, Any]:
    started = time.time()
    service_name = f"{node_name}/get_state"
    client = node.create_client(GetState, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return {
                "available": False,
                "state": None,
                "state_id": None,
                "service": service_name,
                "returncode": None,
                "stdout": "",
                "stderr": "lifecycle get_state service not available",
                "timed_out": False,
                "query_duration_sec": round(time.time() - started, 3),
            }
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return {
                "available": False,
                "state": None,
                "state_id": None,
                "service": service_name,
                "returncode": None,
                "stdout": "",
                "stderr": "lifecycle get_state call timed out",
                "timed_out": True,
                "query_duration_sec": round(time.time() - started, 3),
            }
        current = future.result().current_state
        state = current.label
        return {
            "available": state is not None,
            "state": state,
            "state_id": int(current.id),
            "service": service_name,
            "returncode": 0,
            "stdout": f"{state} [{int(current.id)}]",
            "stderr": "",
            "timed_out": False,
            "query_duration_sec": round(time.time() - started, 3),
        }
    finally:
        node.destroy_client(client)


class ReadinessNode(Node):  # pragma: no cover - ROS runtime only
    def __init__(self) -> None:
        super().__init__("boxfusion_stage1_scene_nav2_readiness")
        qos_map = QoSProfile(depth=1)
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sample: dict[str, Any] | None = None
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos_map)
        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")
        self.compute_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.map_sample = {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
            },
        }


def missing_and_stuck(
    lifecycle_states: dict[str, Any],
    map_received: bool,
    action_servers: dict[str, bool],
    require_compute_path: bool,
    require_navigate_to_pose: bool,
) -> tuple[list[str], list[str]]:
    missing: list[str] = []
    stuck: list[str] = []
    for node_name in REQUIRED_LIFECYCLE_NODES:
        item = lifecycle_states.get(node_name) or {}
        if item.get("state") != "active":
            state = "query_timeout" if item.get("timed_out") else item.get("state")
            state = state or "unavailable"
            missing.append(f"{node_name} not active ({state})")
            stuck.append(f"{node_name}: {state}")
    if not map_received:
        missing.append("/map OccupancyGrid sample not received with TRANSIENT_LOCAL QoS")
    if not action_servers.get("/follow_path"):
        missing.append("/follow_path action server not ready")
    if require_compute_path and not action_servers.get("/compute_path_to_pose"):
        missing.append("/compute_path_to_pose action server not ready")
    if require_navigate_to_pose and not action_servers.get("/navigate_to_pose"):
        missing.append("/navigate_to_pose action server not ready")
    return missing, stuck


def run_wait(
    stage_output_dir: Path,
    timeout_sec: float,
    require_compute_path: bool,
    require_navigate_to_pose: bool,
) -> dict[str, Any]:
    if IMPORT_ERROR:
        return {
            "artifact_type": "stage1_nav2_lifecycle_readiness_report",
            "version": "v0_1",
            "created_utc": now_iso(),
            "stage_output_dir": stage_output_dir.as_posix(),
            "succeeded": False,
            "failure_reason": "rclpy import failed",
            "rclpy_import_error": IMPORT_ERROR,
            "missing_conditions": ["rclpy_import_error"],
        }

    rclpy.init(args=None)
    node = ReadinessNode()
    lifecycle_states: dict[str, Any] = {}
    action_servers = {"/follow_path": False, "/compute_path_to_pose": False, "/navigate_to_pose": False}
    deadline = time.time() + timeout_sec
    next_lifecycle_query = 0.0
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            action_servers = {
                "/follow_path": bool(node.follow_path_client.wait_for_server(timeout_sec=0.05)),
                "/compute_path_to_pose": bool(node.compute_client.wait_for_server(timeout_sec=0.05)),
                "/navigate_to_pose": bool(node.navigate_client.wait_for_server(timeout_sec=0.05)),
            }
            if time.time() >= next_lifecycle_query:
                for node_name in REQUIRED_LIFECYCLE_NODES + DIAGNOSTIC_LIFECYCLE_NODES:
                    lifecycle_states[node_name] = lifecycle_get(node, node_name)
                    lifecycle_states[node_name]["last_checked_utc"] = now_iso()
                next_lifecycle_query = time.time() + 1.0
            missing, stuck = missing_and_stuck(
                lifecycle_states,
                node.map_sample is not None,
                action_servers,
                require_compute_path,
                require_navigate_to_pose,
            )
            if not missing:
                break

        # Capture one final state, especially for timeout diagnostics.
        rclpy.spin_once(node, timeout_sec=0.1)
        for node_name in REQUIRED_LIFECYCLE_NODES + DIAGNOSTIC_LIFECYCLE_NODES:
            lifecycle_states[node_name] = lifecycle_get(node, node_name)
            lifecycle_states[node_name]["last_checked_utc"] = now_iso()
        action_servers = {
            "/follow_path": bool(node.follow_path_client.wait_for_server(timeout_sec=0.25)),
            "/compute_path_to_pose": bool(node.compute_client.wait_for_server(timeout_sec=0.25)),
            "/navigate_to_pose": bool(node.navigate_client.wait_for_server(timeout_sec=0.25)),
        }
        missing, stuck = missing_and_stuck(
            lifecycle_states,
            node.map_sample is not None,
            action_servers,
            require_compute_path,
            require_navigate_to_pose,
        )
        succeeded = not missing
        return {
            "artifact_type": "stage1_nav2_lifecycle_readiness_report",
            "version": "v0_1",
            "created_utc": now_iso(),
            "stage_output_dir": stage_output_dir.as_posix(),
            "timeout_sec": timeout_sec,
            "succeeded": succeeded,
            "failure_reason": None if succeeded else "Nav2 lifecycle bringup failed or required dataplane readiness was not reached",
            "follow_path_only_runtime": not require_compute_path and not require_navigate_to_pose,
            "required_lifecycle_nodes": REQUIRED_LIFECYCLE_NODES,
            "diagnostic_lifecycle_nodes": DIAGNOSTIC_LIFECYCLE_NODES,
            "lifecycle_states": lifecycle_states,
            "map_received": node.map_sample is not None,
            "map_sample": node.map_sample,
            "map_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
            "action_servers": action_servers,
            "action_server_roles": {
                "/follow_path": "hard_blocker_for_current_route_execution",
                "/compute_path_to_pose": "non_blocking_diagnostic_unless_planning_gate_requested",
                "/navigate_to_pose": "non_blocking_diagnostic_for_follow_path_runtime",
            },
            "require_compute_path_to_pose": require_compute_path,
            "require_navigate_to_pose": require_navigate_to_pose,
            "missing_conditions": missing,
            "stuck_nodes": stuck,
        }
    finally:
        try:
            node.follow_path_client.destroy()
            node.compute_client.destroy()
            node.navigate_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id")
    parser.add_argument("--floor-id")
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--require-compute-path-to-pose", action="store_true")
    parser.add_argument("--require-navigate-to-pose", action="store_true")
    args = parser.parse_args()

    payload = run_wait(
        args.stage_output_dir.resolve(),
        args.timeout_sec,
        args.require_compute_path_to_pose,
        args.require_navigate_to_pose,
    )
    write_json(args.output_json, payload)
    write_md(args.output_md, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("succeeded") else 1


if __name__ == "__main__":
    sys.exit(main())
