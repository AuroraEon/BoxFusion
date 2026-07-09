#!/usr/bin/env python3
"""Publish task60 scripted stair composite RViz overlays."""

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

from rslg_gazebo_pid_follower import DEFAULT_ODOM_TOPIC, yaw_from_quaternion  # noqa: E402
from rslg_gazebo_scripted_stair_demo_orchestrator import (  # noqa: E402
    marker_array_msg,
    path_msg,
    pose_stamped_msg,
    query_id_from_runtime_input,
)
from scripted_stair_transition_common import (  # noqa: E402
    PROJECT_NAME,
    TOPICS,
    AnchorTransform,
    Pose3D,
    count_route_segments,
    read_json,
    route_points,
    stair_points,
    transform_points,
    utc_now,
    write_json,
)


class CompositeRvizPublisher:
    def __init__(
        self,
        *,
        rclpy: Any,
        Node: Any,
        Odometry: Any,
        RosPath: Any,
        PoseStamped: Any,
        MarkerArray: Any,
        Marker: Any,
        args: argparse.Namespace,
        runtime_input: dict[str, Any],
    ) -> None:
        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.node = _Node("rslg_scripted_stair_composite_rviz_publisher")
        self.Odometry = Odometry
        self.RosPath = RosPath
        self.PoseStamped = PoseStamped
        self.MarkerArray = MarkerArray
        self.Marker = Marker
        self.args = args
        self.runtime_input = runtime_input
        self.query_id = query_id_from_runtime_input(runtime_input, args.query_id)
        self.original_points = route_points(runtime_input)
        self.original_stair = stair_points(runtime_input)
        self.planned_points = self.original_points
        self.stair = self.original_stair
        self.executed_points: list[Pose3D] = []
        self.anchor_transform: AnchorTransform | None = None
        self.anchor_applied = False
        self.start_monotonic = time.monotonic()
        self.odom_received = False
        self.planned_pub = self.node.create_publisher(RosPath, TOPICS["composite_planned_path_odom"], 10)
        self.executed_pub = self.node.create_publisher(RosPath, TOPICS["composite_executed_path"], 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, TOPICS["robot_pose"], 10)
        self.stair_path_pub = self.node.create_publisher(RosPath, TOPICS["stair_transition_path"], 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, TOPICS["marker_array"], 10)
        self.odom_sub = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.timer = self.node.create_timer(1.0 / float(args.publish_rate_hz), self._on_timer)

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        current = Pose3D(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            segment="gazebo_odom",
        )
        if not self.anchor_applied and self.args.anchor_first_waypoint_to_odom_start:
            first = self.original_points[0]
            self.anchor_transform = AnchorTransform(
                route_x=first.x,
                route_y=first.y,
                route_z=first.z,
                route_yaw=first.yaw,
                odom_x=current.x,
                odom_y=current.y,
                odom_z=current.z,
                odom_yaw=current.yaw,
            )
            self.planned_points = transform_points(self.original_points, self.anchor_transform)
            self.stair = transform_points(self.original_stair, self.anchor_transform)
            self.anchor_applied = True
        self.odom_received = True
        self.executed_points.append(current)
        if len(self.executed_points) > int(self.args.max_path_points):
            self.executed_points = self.executed_points[-int(self.args.max_path_points) :]

    def _on_timer(self) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self.planned_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.planned_points))
        self.executed_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.executed_points))
        self.stair_path_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.stair))
        self.marker_pub.publish(marker_array_msg(self.MarkerArray, self.Marker, self.args.frame_id, stamp, self.planned_points))
        if self.executed_points:
            self.robot_pose_pub.publish(pose_stamped_msg(self.PoseStamped, self.args.frame_id, stamp, self.executed_points[-1]))

    def summary(self) -> dict[str, Any]:
        return {
            "schema_name": "rslg_scripted_stair_composite_rviz_publisher_summary",
            "schema_version": "0.1",
            "project_name": PROJECT_NAME,
            "generated_utc": utc_now(),
            "status": "running",
            "query_id": self.query_id,
            "frame_id": self.args.frame_id,
            "topics": TOPICS,
            "planned_point_count": len(self.planned_points),
            "stair_keyframe_count": len(self.stair),
            "executed_point_count": len(self.executed_points),
            "segment_counts": count_route_segments(self.planned_points),
            "odom_received": self.odom_received,
            "anchor_applied": self.anchor_applied,
        }


def dry_run(args: argparse.Namespace, runtime_input: dict[str, Any]) -> int:
    points = route_points(runtime_input)
    stair = stair_points(runtime_input)
    payload = {
        "schema_name": "rslg_scripted_stair_composite_rviz_dry_run",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id_from_runtime_input(runtime_input, args.query_id),
        "frame_id": args.frame_id,
        "topics": TOPICS,
        "planned_point_count": len(points),
        "stair_keyframe_count": len(stair),
        "segment_counts": count_route_segments(points),
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "replay_topic_primary_display": False,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{payload['query_id']}_scripted_stair_rviz_dry_run_summary.json"
    write_json(output, payload)
    print(json.dumps(payload, indent=2, sort_keys=False), flush=True)
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--max-path-points", type=int, default=10000)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime_input = read_json(args.runtime_input_json)
    if args.dry_run:
        return dry_run(args, runtime_input)
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry, Path as RosPath
        from rclpy.node import Node
        from visualization_msgs.msg import Marker, MarkerArray
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise SystemExit(f"ROS 2 Python dependencies are unavailable: {exc}") from exc

    rclpy.init(args=None)
    publisher = CompositeRvizPublisher(
        rclpy=rclpy,
        Node=Node,
        Odometry=Odometry,
        RosPath=RosPath,
        PoseStamped=PoseStamped,
        MarkerArray=MarkerArray,
        Marker=Marker,
        args=args,
        runtime_input=runtime_input,
    )
    try:
        rclpy.spin(publisher.node)
    except KeyboardInterrupt:
        pass
    finally:
        print(json.dumps(publisher.summary(), indent=2, sort_keys=False), flush=True)
        publisher.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
