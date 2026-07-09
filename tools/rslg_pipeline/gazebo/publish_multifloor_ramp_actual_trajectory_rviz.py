#!/usr/bin/env python3
"""Publish task59 multi-floor ramp planned-vs-actual RViz overlays."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_multifloor_ramp_follower import (  # noqa: E402
    BLOCKED_CANDIDATE_ID,
    FORBIDDEN_NON_TRANSITION_EDGE,
    SELECTED_APPROACH_ID,
    TRUE_TRANSITION_EDGE,
    Waypoint,
    count_by_segment,
    extract_waypoints,
    query_id_from_runtime_input,
)
from rslg_gazebo_pid_follower import DEFAULT_ODOM_TOPIC, normalize_angle, read_json, yaw_from_quaternion  # noqa: E402


TOPICS = {
    "planned_path_odom": "/rslg/multifloor_planned_path_odom",
    "gazebo_executed_path": "/rslg/multifloor_gazebo_executed_path",
    "gazebo_robot_pose": "/rslg/multifloor_gazebo_robot_pose",
    "marker_array": "/rslg/multifloor_ramp_marker_array",
}


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class AnchorTransform:
    route_x: float
    route_y: float
    route_z: float
    route_yaw: float
    odom_x: float
    odom_y: float
    odom_z: float
    odom_yaw: float

    @property
    def yaw_offset(self) -> float:
        return normalize_angle(self.odom_yaw - self.route_yaw)

    def apply(self, x: float, y: float, z: float, yaw: float | None = None) -> Pose3D:
        offset = self.yaw_offset
        dx = float(x) - self.route_x
        dy = float(y) - self.route_y
        cos_yaw = math.cos(offset)
        sin_yaw = math.sin(offset)
        out_yaw = self.odom_yaw if yaw is None else normalize_angle(float(yaw) + offset)
        return Pose3D(
            x=self.odom_x + dx * cos_yaw - dy * sin_yaw,
            y=self.odom_y + dx * sin_yaw + dy * cos_yaw,
            z=self.odom_z + (float(z) - self.route_z),
            yaw=out_yaw,
        )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def point_dict(pose: Pose3D) -> dict[str, float]:
    return {"x": round(pose.x, 6), "y": round(pose.y, 6), "z": round(pose.z, 6)}


def pose_from_waypoint(waypoint: Waypoint) -> Pose3D:
    return Pose3D(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw)


def marker_record(
    *,
    marker_id: int,
    ns: str,
    marker_type: str,
    frame_id: str,
    position: dict[str, float] | None = None,
    points: list[dict[str, float]] | None = None,
    scale: dict[str, float] | None = None,
    color_rgba: list[float] | None = None,
    text: str | None = None,
    yaw: float | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "ns": ns,
        "id": marker_id,
        "type": marker_type,
        "frame_id": frame_id,
        "position": position,
        "points": points or [],
        "scale": scale or {"x": 0.18, "y": 0.18, "z": 0.18},
        "color_rgba": color_rgba or [1.0, 1.0, 1.0, 1.0],
        "text": text,
        "yaw": yaw,
        "metadata": metadata or {},
    }


def segment_points(waypoints: list[Waypoint], key: str) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for waypoint in waypoints:
        if key == "ramp" and waypoint.is_ramp_waypoint:
            points.append({"x": waypoint.x, "y": waypoint.y, "z": waypoint.z + 0.04})
        elif key == "floor_1" and waypoint.floor_id == "floor_1" and not waypoint.is_ramp_waypoint:
            points.append({"x": waypoint.x, "y": waypoint.y, "z": waypoint.z + 0.04})
        elif key == "floor_2" and waypoint.floor_id == "floor_2" and not waypoint.is_ramp_waypoint:
            points.append({"x": waypoint.x, "y": waypoint.y, "z": waypoint.z + 0.04})
    return points


def build_raw_marker_records(runtime_input: dict[str, Any], waypoints: list[Waypoint], frame_id: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    marker_id = 1
    segment_specs = [
        ("floor_1", "rslg_multifloor_floor_1_segment", [0.25, 0.72, 1.0, 1.0]),
        ("ramp", "rslg_multifloor_ramp_surrogate_vt_1_centerline_e001", [0.70, 0.35, 1.0, 1.0]),
        ("floor_2", "rslg_multifloor_floor_2_segment", [0.20, 0.95, 0.45, 1.0]),
    ]
    for key, ns, color in segment_specs:
        points = segment_points(waypoints, key)
        if len(points) >= 2:
            records.append(
                marker_record(
                    marker_id=marker_id,
                    ns=ns,
                    marker_type="LINE_STRIP",
                    frame_id=frame_id,
                    points=points,
                    scale={"x": 0.055, "y": 0.055, "z": 0.055},
                    color_rgba=color,
                    metadata={"visualization_role": key},
                )
            )
            marker_id += 1

    ramp_points = segment_points(waypoints, "ramp")
    if ramp_points:
        mid = ramp_points[len(ramp_points) // 2]
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_vt_1_centerline_e001_ramp_surrogate_label",
                marker_type="TEXT_VIEW_FACING",
                frame_id=frame_id,
                position={"x": mid["x"], "y": mid["y"], "z": mid["z"] + 0.35},
                scale={"x": 0.18, "y": 0.18, "z": 0.18},
                color_rgba=[0.86, 0.72, 1.0, 1.0],
                text=f"{TRUE_TRANSITION_EDGE} ramp surrogate",
                metadata={
                    "visualization_role": "ramp_connector_label",
                    "transition_edge": TRUE_TRANSITION_EDGE,
                    "not_physical_stair_climbing": True,
                },
            )
        )
        marker_id += 1
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_vt_1_centerline_e003_forbidden_label",
                marker_type="TEXT_VIEW_FACING",
                frame_id=frame_id,
                position={"x": mid["x"] + 0.45, "y": mid["y"], "z": mid["z"] + 0.12},
                scale={"x": 0.16, "y": 0.16, "z": 0.16},
                color_rgba=[1.0, 0.30, 0.18, 1.0],
                text=f"{FORBIDDEN_NON_TRANSITION_EDGE} forbidden/non-transition",
                metadata={"visualization_role": "forbidden_non_transition_edge_label", "transition_edge_used": False},
            )
        )
        marker_id += 1

    final = waypoints[-1]
    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_generated_ring_002_final_approach",
            marker_type="SPHERE",
            frame_id=frame_id,
            position={"x": final.x, "y": final.y, "z": final.z + 0.18},
            scale={"x": 0.28, "y": 0.28, "z": 0.28},
            color_rgba=[0.05, 0.88, 0.30, 1.0],
            yaw=final.yaw,
            metadata={
                "visualization_role": "final_target_approach",
                "candidate_id": runtime_input.get("target_approach_id") or SELECTED_APPROACH_ID,
                "used_as_runtime_goal": runtime_input.get("target_approach_id") == SELECTED_APPROACH_ID,
            },
        )
    )
    marker_id += 1
    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_blocked_candidate_guard",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position={"x": final.x, "y": final.y - 0.6, "z": final.z + 0.5},
            scale={"x": 0.16, "y": 0.16, "z": 0.16},
            color_rgba=[1.0, 0.42, 0.35, 1.0],
            text=f"{BLOCKED_CANDIDATE_ID} blocked/rejected only",
            metadata={"visualization_role": "blocked_candidate_guard", "used_as_runtime_goal": False},
        )
    )
    marker_id += 1
    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_multifloor_ramp_claim_boundary",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position={"x": final.x, "y": final.y - 1.0, "z": final.z + 0.76},
            scale={"x": 0.16, "y": 0.16, "z": 0.16},
            color_rgba=[1.0, 0.95, 0.72, 1.0],
            text="Gazebo ramp simulation only; not physical stair climbing",
            metadata={
                "visualization_role": "claim_boundary",
                "gazebo_simulation_only": True,
                "physical_stair_climbing_claimed": False,
            },
        )
    )
    return records


def transform_pose(pose: Pose3D, transform: AnchorTransform | None) -> Pose3D:
    if transform is None:
        return pose
    return transform.apply(pose.x, pose.y, pose.z, pose.yaw)


def transform_marker_records(records: list[dict[str, Any]], transform: AnchorTransform | None, frame_id: str) -> list[dict[str, Any]]:
    transformed: list[dict[str, Any]] = []
    for record in records:
        out = dict(record)
        out["frame_id"] = frame_id
        position = record.get("position")
        if isinstance(position, dict):
            yaw = record.get("yaw")
            pose = Pose3D(as_float(position.get("x")), as_float(position.get("y")), as_float(position.get("z")), as_float(yaw))
            new_pose = transform_pose(pose, transform)
            out["position"] = point_dict(new_pose)
            if yaw is not None:
                out["yaw"] = new_pose.yaw
        points: list[dict[str, float]] = []
        for point in record.get("points") or []:
            if isinstance(point, dict):
                pose = Pose3D(as_float(point.get("x")), as_float(point.get("y")), as_float(point.get("z")), 0.0)
                points.append(point_dict(transform_pose(pose, transform)))
        out["points"] = points
        transformed.append(out)
    return transformed


def dry_run(args: argparse.Namespace, runtime_input: dict[str, Any], waypoints: list[Waypoint]) -> dict[str, Any]:
    query_id = query_id_from_runtime_input(runtime_input, args.query_id)
    markers = build_raw_marker_records(runtime_input, waypoints, args.frame_id)
    summary = {
        "schema_name": "rslg_multifloor_ramp_actual_trajectory_rviz_dry_run",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id,
        "runtime_input_json": str(args.runtime_input_json),
        "frame_id": args.frame_id,
        "odom_topic": args.odom_topic,
        "topics": dict(TOPICS),
        "selected_waypoint_count": len(waypoints),
        "segment_waypoints_total": count_by_segment(waypoints),
        "planned_path_point_count": len(waypoints),
        "semantic_marker_count": len(markers),
        "first_selected_waypoint": waypoints[0].__dict__,
        "last_selected_waypoint": waypoints[-1].__dict__,
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "anchor_status": "pending_first_odom_in_live_mode"
        if args.anchor_first_waypoint_to_odom_start
        else "not_requested",
        "max_path_points": args.max_path_points,
        "publish_rate_hz": args.publish_rate_hz,
        "rviz_display_policy": {
            "primary_planned_topic": TOPICS["planned_path_odom"],
            "primary_actual_topic": TOPICS["gazebo_executed_path"],
            "primary_robot_pose_topic": TOPICS["gazebo_robot_pose"],
            "primary_marker_topic": TOPICS["marker_array"],
            "replay_path_primary": False,
            "replay_robot_pose_primary": False,
        },
        "guard_policy": {
            "ramp_connector_edge_id": TRUE_TRANSITION_EDGE,
            "forbidden_transition_edge_ids": [FORBIDDEN_NON_TRANSITION_EDGE],
            "blocked_goal_candidate_ids": [BLOCKED_CANDIDATE_ID],
            "target_approach_id": runtime_input.get("target_approach_id"),
        },
        "claim_boundary": runtime_input.get("claim_boundary"),
    }
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_json(args.output_dir / f"{query_id}_multifloor_ramp_rviz_dry_run_summary.json", summary)
        write_json(args.output_dir / "dry_run_summary.json", summary)
    return summary


def import_ros_messages() -> dict[str, Any]:
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as NavPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    return {
        "rclpy": rclpy,
        "Point": Point,
        "PoseStamped": PoseStamped,
        "Odometry": Odometry,
        "NavPath": NavPath,
        "Node": Node,
        "DurabilityPolicy": DurabilityPolicy,
        "QoSProfile": QoSProfile,
        "Marker": Marker,
        "MarkerArray": MarkerArray,
    }


class MultifloorRampRvizPublisher:
    def __init__(self, *, ros: dict[str, Any], args: argparse.Namespace, runtime_input: dict[str, Any], waypoints: list[Waypoint]) -> None:
        Node = ros["Node"]
        QoSProfile = ros["QoSProfile"]
        DurabilityPolicy = ros["DurabilityPolicy"]
        Odometry = ros["Odometry"]
        NavPath = ros["NavPath"]
        PoseStamped = ros["PoseStamped"]
        MarkerArray = ros["MarkerArray"]

        class _Node(Node):
            pass

        self.node = _Node("rslg_multifloor_ramp_actual_trajectory_rviz")
        self.ros = ros
        self.args = args
        self.runtime_input = runtime_input
        self.raw_waypoints = list(waypoints)
        self.query_id = query_id_from_runtime_input(runtime_input, args.query_id)
        self.frame_id = args.frame_id
        self.start_monotonic = time.monotonic()
        self.first_odom: Pose3D | None = None
        self.latest_pose: Pose3D | None = None
        self.anchor_transform: AnchorTransform | None = None
        self.planned_poses: list[Pose3D] = []
        self.executed_poses: list[Pose3D] = []
        self.odom_count = 0
        self.publish_count = 0
        self.raw_marker_records = build_raw_marker_records(runtime_input, waypoints, args.frame_id)
        self.marker_records: list[dict[str, Any]] = []
        self.output_dir = args.output_dir
        self.csv_handle: Any | None = None
        self.csv_writer: csv.DictWriter[str] | None = None
        self.trajectory_csv: Path | None = None
        self.summary_json: Path | None = None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.trajectory_csv = self.output_dir / f"{self.query_id}_multifloor_ramp_rviz_trajectory.csv"
            self.summary_json = self.output_dir / f"{self.query_id}_multifloor_ramp_rviz_summary.json"
            self.csv_handle = self.trajectory_csv.open("w", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_handle,
                fieldnames=["wall_time_utc", "elapsed_sec", "odom_x", "odom_y", "odom_z", "odom_yaw", "odom_sequence", "event"],
            )
            self.csv_writer.writeheader()

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.planned_pub = self.node.create_publisher(NavPath, TOPICS["planned_path_odom"], latched_qos)
        self.executed_pub = self.node.create_publisher(NavPath, TOPICS["gazebo_executed_path"], 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, TOPICS["gazebo_robot_pose"], 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, TOPICS["marker_array"], latched_qos)
        self.subscription = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 50)
        self.timer = self.node.create_timer(1.0 / max(float(args.publish_rate_hz), 0.1), self._publish)

    def _pose_from_odom(self, msg: Any) -> Pose3D:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        return Pose3D(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)),
        )

    def _initialize_anchor(self, odom_pose: Pose3D) -> None:
        self.first_odom = odom_pose
        first = self.raw_waypoints[0]
        if self.args.anchor_first_waypoint_to_odom_start:
            self.anchor_transform = AnchorTransform(
                route_x=first.x,
                route_y=first.y,
                route_z=first.z,
                route_yaw=first.yaw,
                odom_x=odom_pose.x,
                odom_y=odom_pose.y,
                odom_z=odom_pose.z,
                odom_yaw=odom_pose.yaw,
            )
        self.planned_poses = [transform_pose(pose_from_waypoint(wp), self.anchor_transform) for wp in self.raw_waypoints]
        self.marker_records = transform_marker_records(self.raw_marker_records, self.anchor_transform, self.frame_id)

    def _on_odom(self, msg: Any) -> None:
        pose = self._pose_from_odom(msg)
        self.latest_pose = pose
        if self.first_odom is None:
            self._initialize_anchor(pose)
        self.odom_count += 1
        self.executed_poses.append(pose)
        if self.args.max_path_points > 0 and len(self.executed_poses) > self.args.max_path_points:
            self.executed_poses = self.executed_poses[-self.args.max_path_points :]
        if self.csv_writer is not None:
            self.csv_writer.writerow(
                {
                    "wall_time_utc": utc_now(),
                    "elapsed_sec": f"{time.monotonic() - self.start_monotonic:.6f}",
                    "odom_x": f"{pose.x:.6f}",
                    "odom_y": f"{pose.y:.6f}",
                    "odom_z": f"{pose.z:.6f}",
                    "odom_yaw": f"{pose.yaw:.6f}",
                    "odom_sequence": self.odom_count,
                    "event": "odom",
                }
            )
            if self.odom_count % 10 == 0 and self.csv_handle is not None:
                self.csv_handle.flush()

    def _pose_stamped(self, pose: Pose3D, stamp: Any) -> Any:
        PoseStamped = self.ros["PoseStamped"]
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        msg.pose.position.z = float(pose.z)
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _path_msg(self, poses: list[Pose3D], stamp: Any) -> Any:
        NavPath = self.ros["NavPath"]
        msg = NavPath()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.poses = [self._pose_stamped(pose, stamp) for pose in poses]
        return msg

    def _marker_from_record(self, record: dict[str, Any], stamp: Any) -> Any:
        Point = self.ros["Point"]
        Marker = self.ros["Marker"]
        marker = Marker()
        type_map = {
            "ARROW": Marker.ARROW,
            "CUBE": Marker.CUBE,
            "SPHERE": Marker.SPHERE,
            "CYLINDER": Marker.CYLINDER,
            "LINE_STRIP": Marker.LINE_STRIP,
            "LINE_LIST": Marker.LINE_LIST,
            "TEXT_VIEW_FACING": Marker.TEXT_VIEW_FACING,
        }
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = str(record.get("ns") or "rslg_multifloor_ramp")
        marker.id = int(record.get("id", 0))
        marker.type = type_map.get(str(record.get("type") or "SPHERE"), Marker.SPHERE)
        marker.action = Marker.ADD
        scale = record.get("scale") or {}
        marker.scale.x = as_float(scale.get("x"), 0.18)
        marker.scale.y = as_float(scale.get("y"), marker.scale.x)
        marker.scale.z = as_float(scale.get("z"), marker.scale.x)
        color = record.get("color_rgba") or [1.0, 1.0, 1.0, 1.0]
        marker.color.r = as_float(color[0], 1.0)
        marker.color.g = as_float(color[1], 1.0)
        marker.color.b = as_float(color[2], 1.0)
        marker.color.a = as_float(color[3], 1.0)
        position = record.get("position")
        if isinstance(position, dict):
            marker.pose.position.x = as_float(position.get("x"))
            marker.pose.position.y = as_float(position.get("y"))
            marker.pose.position.z = as_float(position.get("z"))
        yaw = record.get("yaw")
        if yaw is not None:
            qx, qy, qz, qw = yaw_to_quaternion(as_float(yaw))
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
        else:
            marker.pose.orientation.w = 1.0
        for point in record.get("points") or []:
            if isinstance(point, dict):
                pt = Point()
                pt.x = as_float(point.get("x"))
                pt.y = as_float(point.get("y"))
                pt.z = as_float(point.get("z"))
                marker.points.append(pt)
        marker.text = str(record.get("text") or "")
        return marker

    def _marker_array_msg(self, stamp: Any) -> Any:
        MarkerArray = self.ros["MarkerArray"]
        msg = MarkerArray()
        msg.markers = [self._marker_from_record(record, stamp) for record in self.marker_records]
        return msg

    def _publish(self) -> None:
        if self.first_odom is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        self.planned_pub.publish(self._path_msg(self.planned_poses, stamp))
        self.executed_pub.publish(self._path_msg(self.executed_poses, stamp))
        if self.latest_pose is not None:
            self.robot_pose_pub.publish(self._pose_stamped(self.latest_pose, stamp))
        self.marker_pub.publish(self._marker_array_msg(stamp))
        self.publish_count += 1

    def summary(self) -> dict[str, Any]:
        first = self.raw_waypoints[0]
        return {
            "schema_name": "rslg_multifloor_ramp_actual_trajectory_rviz_live_summary",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "query_id": self.query_id,
            "frame_id": self.frame_id,
            "odom_topic": self.args.odom_topic,
            "topics": dict(TOPICS),
            "selected_waypoint_count": len(self.raw_waypoints),
            "segment_waypoints_total": count_by_segment(self.raw_waypoints),
            "planned_path_point_count": len(self.planned_poses),
            "executed_path_point_count": len(self.executed_poses),
            "semantic_marker_count": len(self.marker_records),
            "odom_received": self.first_odom is not None,
            "odom_count": self.odom_count,
            "publish_count": self.publish_count,
            "anchor_first_waypoint_to_odom_start": bool(self.args.anchor_first_waypoint_to_odom_start),
            "anchor_applied": self.anchor_transform is not None,
            "route_anchor": {"x": first.x, "y": first.y, "z": first.z, "yaw": first.yaw},
            "odom_anchor": None if self.first_odom is None else self.first_odom.__dict__,
            "trajectory_csv": None if self.trajectory_csv is None else str(self.trajectory_csv),
            "claim_boundary": self.runtime_input.get("claim_boundary"),
        }

    def close(self) -> None:
        if self.csv_handle is not None:
            self.csv_handle.flush()
            self.csv_handle.close()
        if self.summary_json is not None:
            write_json(self.summary_json, self.summary())


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--max-path-points", type=int, default=10000)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime_input = read_json(args.runtime_input_json)
    waypoints = extract_waypoints(runtime_input, "all")
    if args.dry_run:
        summary = dry_run(args, runtime_input, waypoints)
        print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
        return 0

    ros = import_ros_messages()
    rclpy = ros["rclpy"]
    rclpy.init(args=None)
    publisher = MultifloorRampRvizPublisher(ros=ros, args=args, runtime_input=runtime_input, waypoints=waypoints)
    try:
        while rclpy.ok():
            rclpy.spin_once(publisher.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()
        publisher.node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(publisher.summary(), indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
