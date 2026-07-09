#!/usr/bin/env python3
"""Standalone Gazebo/RViz scripted stair-transition animator for task60."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_scripted_stair_demo_orchestrator import pose_stamped_msg, query_id_from_runtime_input  # noqa: E402
from scripted_stair_transition_common import (  # noqa: E402
    PROJECT_NAME,
    TOPICS,
    TRUE_TRANSITION_EDGE,
    Pose3D,
    read_json,
    stair_points,
    utc_now,
    write_json,
    yaw_to_quaternion,
)


class ScriptedStairAnimator:
    def __init__(
        self,
        *,
        rclpy: Any,
        Node: Any,
        PoseStamped: Any,
        SetEntityState: Any | None,
        EntityState: Any | None,
        args: argparse.Namespace,
        runtime_input: dict[str, Any],
    ) -> None:
        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.node = _Node("rslg_gazebo_scripted_stair_animator")
        self.PoseStamped = PoseStamped
        self.SetEntityState = SetEntityState
        self.EntityState = EntityState
        self.args = args
        self.runtime_input = runtime_input
        self.query_id = query_id_from_runtime_input(runtime_input, args.query_id)
        self.keyframes = stair_points(runtime_input)
        self.index = 0
        self.start_monotonic = time.monotonic()
        self.done = False
        self.service_import_available = SetEntityState is not None and EntityState is not None
        self.service_available = False
        self.service_used = False
        self.failure_reason: list[str] = []
        self.pose_pub = self.node.create_publisher(PoseStamped, TOPICS["robot_pose"], 10)
        self.client = None
        if self.service_import_available:
            self.client = self.node.create_client(SetEntityState, args.set_entity_state_service)
            self.service_available = bool(self.client.wait_for_service(timeout_sec=args.service_wait_sec))
        self.timer = self.node.create_timer(1.0 / float(args.rate_hz), self._on_timer)

    def _call_set_entity_state(self, pose: Pose3D) -> None:
        if self.client is None or self.SetEntityState is None or self.EntityState is None:
            return
        request = self.SetEntityState.Request()
        state = self.EntityState()
        state.name = self.args.robot_model_name
        state.pose.position.x = float(pose.x)
        state.pose.position.y = float(pose.y)
        state.pose.position.z = float(pose.z)
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.reference_frame = "world"
        request.state = state
        self.client.call_async(request)
        self.service_used = True

    def _on_timer(self) -> None:
        if self.done:
            return
        if self.index >= len(self.keyframes):
            self.done = True
            return
        if not self.service_available and not self.args.rviz_only_stair_fallback:
            self.failure_reason.append("gazebo_set_entity_state_unavailable")
            self.done = True
            return
        pose = self.keyframes[self.index]
        if self.service_available:
            self._call_set_entity_state(pose)
        stamp = self.node.get_clock().now().to_msg()
        self.pose_pub.publish(pose_stamped_msg(self.PoseStamped, self.args.frame_id, stamp, pose))
        self.index += 1

    def summary(self) -> dict[str, Any]:
        status = "passed_scripted" if self.index >= len(self.keyframes) and not self.failure_reason else "failed"
        if self.failure_reason and self.args.rviz_only_stair_fallback and self.index >= len(self.keyframes):
            status = "passed_rviz_only_fallback"
        return {
            "schema_name": "rslg_scripted_stair_animator_summary",
            "schema_version": "0.1",
            "project_name": PROJECT_NAME,
            "generated_utc": utc_now(),
            "status": status,
            "query_id": self.query_id,
            "transition_edge_id": TRUE_TRANSITION_EDGE,
            "keyframe_count": len(self.keyframes),
            "keyframes_published": min(self.index, len(self.keyframes)),
            "frame_id": self.args.frame_id,
            "robot_pose_topic": TOPICS["robot_pose"],
            "robot_model_name": self.args.robot_model_name,
            "service_import_available": self.service_import_available,
            "set_entity_state_service": self.args.set_entity_state_service,
            "set_entity_state_service_available": self.service_available,
            "set_entity_state_service_used": self.service_used,
            "rviz_only_stair_fallback": bool(self.args.rviz_only_stair_fallback),
            "failure_reason": self.failure_reason,
            "physical_stair_climbing_claim": False,
        }


def dry_run(args: argparse.Namespace, runtime_input: dict[str, Any]) -> int:
    keyframes = stair_points(runtime_input)
    summary = {
        "schema_name": "rslg_scripted_stair_animator_dry_run",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id_from_runtime_input(runtime_input, args.query_id),
        "transition_edge_id": TRUE_TRANSITION_EDGE,
        "keyframe_count": len(keyframes),
        "first_keyframe": keyframes[0].__dict__ if keyframes else None,
        "last_keyframe": keyframes[-1].__dict__ if keyframes else None,
        "set_entity_state_service": args.set_entity_state_service,
        "robot_model_name": args.robot_model_name,
        "rviz_only_stair_fallback_supported": True,
        "physical_stair_climbing_claim": False,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / f"{summary['query_id']}_scripted_stair_animator_dry_run_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--robot-model-name", default="turtlebot3_burger")
    parser.add_argument("--set-entity-state-service", default="/set_entity_state")
    parser.add_argument("--service-wait-sec", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=4.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rviz-only-stair-fallback", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime_input = read_json(args.runtime_input_json)
    if args.dry_run:
        return dry_run(args, runtime_input)
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise SystemExit(f"ROS 2 Python dependencies are unavailable: {exc}") from exc
    try:
        from gazebo_msgs.msg import EntityState
        from gazebo_msgs.srv import SetEntityState
    except Exception:
        EntityState = None
        SetEntityState = None

    rclpy.init(args=None)
    animator = ScriptedStairAnimator(
        rclpy=rclpy,
        Node=Node,
        PoseStamped=PoseStamped,
        SetEntityState=SetEntityState,
        EntityState=EntityState,
        args=args,
        runtime_input=runtime_input,
    )
    try:
        while rclpy.ok() and not animator.done:
            rclpy.spin_once(animator.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        animator.failure_reason.append("keyboard_interrupt")
    finally:
        summary = animator.summary()
        write_json(args.output_dir / f"{animator.query_id}_scripted_stair_animator_summary.json", summary)
        print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
        animator.node.destroy_node()
        rclpy.shutdown()
    return 0 if animator.summary()["status"].startswith("passed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
