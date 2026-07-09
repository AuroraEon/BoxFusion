#!/usr/bin/env python3
"""Publish Gazebo actual trajectory overlays for RSLG-SLAM RViz validation.

This visualization node compares the selected RSLG-SLAM route against the
actual Gazebo odom trajectory. It intentionally does not import or launch Nav2,
AMCL, map_server, Stage-A, RGB-D inference, navigation actions, Gazebo APIs, or
physical robot code.
"""

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
from typing import Any, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_pid_follower import (  # noqa: E402
    DEFAULT_PROFILE_ID,
    extract_waypoints,
    floor_counts,
    load_profile,
    normalize_angle,
    query_id_from_pid_input,
    read_json,
    yaw_from_quaternion,
)


DEFAULT_ODOM_TOPIC = "/odom"
TOPICS = {
    "planned_path_odom": "/rslg/planned_path_odom",
    "gazebo_executed_path": "/rslg/gazebo_executed_path",
    "gazebo_robot_pose": "/rslg/gazebo_robot_pose",
    "gazebo_actual_marker_array": "/rslg/gazebo_actual_marker_array",
}
SELECTED_CANDIDATE_ID = "generated_ring_002"
BLOCKED_CANDIDATE_ID = "generated_ring_037"
TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
FORBIDDEN_NON_TRANSITION_EDGE = "vt_1_centerline_e003"
DEFAULT_FLOOR_Z_MAP = {"floor_1": 0.0, "floor_2": 1.6}


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

    def apply(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> Pose3D:
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


def parse_floor_z_map(pid_input: dict[str, Any]) -> dict[str, float]:
    raw = pid_input.get("floor_z_map")
    if not isinstance(raw, dict):
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(key): float(value) for key, value in raw.items()}


def selected_approach(route_result: dict[str, Any] | None) -> dict[str, Any] | None:
    if not route_result:
        return None
    approach = route_result.get("approach") or {}
    for candidate in approach.get("approach_candidates") or []:
        if candidate.get("is_runtime_goal") or candidate.get("candidate_id") == SELECTED_CANDIDATE_ID:
            return candidate
    return None


def marker_containing(marker_input: dict[str, Any] | None, token: str) -> dict[str, Any] | None:
    if not marker_input:
        return None
    for marker in marker_input.get("markers") or []:
        if token in json.dumps(marker, sort_keys=True):
            return marker
    return None


def marker_position(marker: dict[str, Any] | None) -> dict[str, float] | None:
    if not marker:
        return None
    pose = marker.get("pose") or {}
    position = pose.get("position") if isinstance(pose, dict) else None
    if not isinstance(position, dict):
        return None
    return {
        "x": as_float(position.get("x")),
        "y": as_float(position.get("y")),
        "z": as_float(position.get("z")),
    }


def marker_points(marker: dict[str, Any] | None) -> list[dict[str, float]]:
    if not marker:
        return []
    points: list[dict[str, float]] = []
    for point in marker.get("points") or []:
        if isinstance(point, dict):
            points.append(
                {
                    "x": as_float(point.get("x")),
                    "y": as_float(point.get("y")),
                    "z": as_float(point.get("z")),
                }
            )
    return points


def point_dict(pose: Pose3D) -> dict[str, float]:
    return {"x": round(pose.x, 6), "y": round(pose.y, 6), "z": round(pose.z, 6)}


def raw_point(x: float, y: float, z: float) -> dict[str, float]:
    return {"x": float(x), "y": float(y), "z": float(z)}


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


def build_raw_marker_records(
    *,
    route_result: dict[str, Any] | None,
    marker_input: dict[str, Any] | None,
    pid_input: dict[str, Any],
    query_id: str,
    profile_id: str,
    frame_id: str,
    floor_id: str | None,
) -> list[dict[str, Any]]:
    floor_z_map = parse_floor_z_map(pid_input)
    target = (route_result or {}).get("target") or {}
    selected = selected_approach(route_result) or {}
    selected_xy = selected.get("world_xy") or []
    selected_floor = str(selected.get("floor_id") or target.get("target_floor_id") or floor_id or "floor_2")
    selected_z = floor_z_map.get(selected_floor, 0.0) + 0.12
    selected_yaw = as_float(selected.get("yaw"), -2.09057)
    records: list[dict[str, Any]] = []
    marker_id = 1

    selected_line = marker_containing(marker_input, "seg_06_object_approach_metric_generated_ring_002")
    selected_line_points = marker_points(selected_line)
    if selected_line_points:
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_selected_approach_path_generated_ring_002",
                marker_type="LINE_STRIP",
                frame_id=frame_id,
                points=selected_line_points,
                scale={"x": 0.045, "y": 0.045, "z": 0.045},
                color_rgba=[0.15, 0.95, 0.45, 1.0],
                metadata={
                    "visualization_role": "selected_approach_path",
                    "candidate_id": SELECTED_CANDIDATE_ID,
                    "used_as_runtime_goal": True,
                },
            )
        )
        marker_id += 1

    if len(selected_xy) >= 2:
        selected_pos = raw_point(float(selected_xy[0]), float(selected_xy[1]), selected_z)
    else:
        selected_marker_pos = marker_position(marker_containing(marker_input, SELECTED_CANDIDATE_ID))
        selected_pos = selected_marker_pos or raw_point(-7.020484, 1.558795, selected_z)
    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_selected_approach_generated_ring_002",
            marker_type="SPHERE",
            frame_id=frame_id,
            position=selected_pos,
            scale={"x": 0.28, "y": 0.28, "z": 0.28},
            color_rgba=[0.05, 0.85, 0.30, 1.0],
            yaw=selected_yaw,
            metadata={
                "visualization_role": "selected_object_approach_goal",
                "candidate_id": SELECTED_CANDIDATE_ID,
                "used_as_runtime_goal": True,
            },
        )
    )
    marker_id += 1
    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_selected_approach_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=raw_point(selected_pos["x"], selected_pos["y"], selected_pos["z"] + 0.34),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[0.55, 1.0, 0.65, 1.0],
            text="generated_ring_002 selected approach",
            metadata={"visualization_role": "selected_approach_label"},
        )
    )
    marker_id += 1

    distance = as_float(selected.get("endpoint_to_object_distance"), 0.480217)
    target_x = selected_pos["x"] + math.cos(selected_yaw) * distance
    target_y = selected_pos["y"] + math.sin(selected_yaw) * distance
    target_z = floor_z_map.get(selected_floor, 0.0) + 0.36
    object_id = str(target.get("target_object_id") or "obj_175")
    category = str(target.get("target_object_category") or "curtain")
    room_id = str(target.get("target_room_id") or "room_14")
    records.append(
        marker_record(
            marker_id=marker_id,
            ns=f"rslg_target_{object_id}_{category}",
            marker_type="CYLINDER",
            frame_id=frame_id,
            position=raw_point(target_x, target_y, target_z),
            scale={"x": 0.30, "y": 0.30, "z": 0.56},
            color_rgba=[1.0, 0.20, 0.80, 1.0],
            metadata={
                "visualization_role": "object_target_marker",
                "object_id": object_id,
                "object_category": category,
                "room_id": room_id,
            },
        )
    )
    marker_id += 1
    records.append(
        marker_record(
            marker_id=marker_id,
            ns=f"rslg_target_{object_id}_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=raw_point(target_x, target_y, target_z + 0.48),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[1.0, 0.78, 0.95, 1.0],
            text=f"{object_id} {category} / {room_id}",
            metadata={"visualization_role": "object_target_label"},
        )
    )
    marker_id += 1

    blocked_marker = marker_containing(marker_input, BLOCKED_CANDIDATE_ID)
    blocked_pos = marker_position(blocked_marker)
    if blocked_pos is not None:
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_generated_ring_037_blocked_evidence_only",
                marker_type="CUBE",
                frame_id=frame_id,
                position=blocked_pos,
                scale={"x": 0.22, "y": 0.22, "z": 0.22},
                color_rgba=[0.95, 0.08, 0.08, 0.85],
                metadata={
                    "visualization_role": "blocked_legacy_evidence_only",
                    "candidate_id": BLOCKED_CANDIDATE_ID,
                    "used_as_runtime_goal": False,
                    "status": "blocked",
                },
            )
        )
        marker_id += 1
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_generated_ring_037_blocked_label",
                marker_type="TEXT_VIEW_FACING",
                frame_id=frame_id,
                position=raw_point(blocked_pos["x"], blocked_pos["y"], blocked_pos["z"] + 0.34),
                scale={"x": 0.18, "y": 0.18, "z": 0.18},
                color_rgba=[1.0, 0.24, 0.20, 1.0],
                text="generated_ring_037 blocked/rejected only",
                metadata={"visualization_role": "blocked_candidate_label", "used_as_runtime_goal": False},
            )
        )
        marker_id += 1

    transition_marker = marker_containing(marker_input, TRUE_TRANSITION_EDGE)
    transition_points = marker_points(transition_marker)
    if transition_points:
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_vt_1_centerline_e001_transition_evidence",
                marker_type="LINE_STRIP",
                frame_id=frame_id,
                points=transition_points,
                scale={"x": 0.055, "y": 0.055, "z": 0.055},
                color_rgba=[0.70, 0.35, 1.0, 0.82],
                metadata={
                    "visualization_role": "transition_evidence",
                    "transition_edge": TRUE_TRANSITION_EDGE,
                    "visualization_only": True,
                    "not_physical_stair_climbing": True,
                },
            )
        )
        marker_id += 1
        mid = transition_points[len(transition_points) // 2]
        records.append(
            marker_record(
                marker_id=marker_id,
                ns="rslg_vt_1_centerline_e003_forbidden_label",
                marker_type="TEXT_VIEW_FACING",
                frame_id=frame_id,
                position=raw_point(mid["x"] + 0.35, mid["y"], mid["z"] + 0.20),
                scale={"x": 0.17, "y": 0.17, "z": 0.17},
                color_rgba=[1.0, 0.32, 0.20, 1.0],
                text="vt_1_centerline_e003 non-transition only",
                metadata={
                    "visualization_role": "forbidden_non_transition_edge_label",
                    "edge_id": FORBIDDEN_NON_TRANSITION_EDGE,
                    "transition_edge_used": False,
                },
            )
        )
        marker_id += 1

    records.append(
        marker_record(
            marker_id=marker_id,
            ns="rslg_gazebo_actual_visualization_claim_boundary",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=raw_point(selected_pos["x"], selected_pos["y"] - 0.75, selected_pos["z"] + 0.62),
            scale={"x": 0.17, "y": 0.17, "z": 0.17},
            color_rgba=[1.0, 0.95, 0.72, 1.0],
            text=f"{query_id} / {profile_id}: Gazebo odom trajectory vs planned path",
            metadata={
                "visualization_role": "claim_boundary_text",
                "gazebo_simulation_only": True,
                "replay_primary_evidence": False,
            },
        )
    )
    return records


def transform_waypoints_visual(waypoints: list[Any], transform: AnchorTransform | None) -> list[Pose3D]:
    poses: list[Pose3D] = []
    for waypoint in waypoints:
        if transform is None:
            poses.append(Pose3D(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw))
        else:
            poses.append(transform.apply(waypoint.x, waypoint.y, waypoint.z, waypoint.yaw))
    return poses


def transform_marker_records(records: list[dict[str, Any]], transform: AnchorTransform | None) -> list[dict[str, Any]]:
    transformed: list[dict[str, Any]] = []
    for record in records:
        out = dict(record)
        out["frame_id"] = record.get("frame_id")
        position = record.get("position")
        if isinstance(position, dict):
            yaw = record.get("yaw")
            pose = (
                Pose3D(as_float(position.get("x")), as_float(position.get("y")), as_float(position.get("z")), as_float(yaw))
                if transform is None
                else transform.apply(
                    as_float(position.get("x")),
                    as_float(position.get("y")),
                    as_float(position.get("z")),
                    as_float(yaw) if yaw is not None else None,
                )
            )
            out["position"] = point_dict(pose)
            if yaw is not None:
                out["yaw"] = pose.yaw
        points: list[dict[str, float]] = []
        for point in record.get("points") or []:
            if isinstance(point, dict):
                pose = (
                    Pose3D(as_float(point.get("x")), as_float(point.get("y")), as_float(point.get("z")), 0.0)
                    if transform is None
                    else transform.apply(as_float(point.get("x")), as_float(point.get("y")), as_float(point.get("z")), None)
                )
                points.append(point_dict(pose))
        out["points"] = points
        transformed.append(out)
    return transformed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-input-json", type=Path, required=True)
    parser.add_argument("--route-result-json", type=Path, default=None)
    parser.add_argument("--rviz-marker-input-json", type=Path, default=None)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--floor-id", default=None)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--max-path-points", type=int, default=10000)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def load_optional_json(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    return read_json(path)


def dry_run(
    args: argparse.Namespace,
    *,
    pid_input: dict[str, Any],
    profile: dict[str, Any],
    waypoints: list[Any],
    route_result: dict[str, Any] | None,
    marker_input: dict[str, Any] | None,
) -> dict[str, Any]:
    query_id = query_id_from_pid_input(pid_input, args.query_id)
    markers = build_raw_marker_records(
        route_result=route_result,
        marker_input=marker_input,
        pid_input=pid_input,
        query_id=query_id,
        profile_id=args.profile_id,
        frame_id=args.frame_id,
        floor_id=args.floor_id,
    )
    summary = {
        "schema_name": "rslg_gazebo_actual_trajectory_rviz_dry_run",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id,
        "profile_id": args.profile_id,
        "profile_loaded": isinstance(profile, dict),
        "pid_input_json": str(args.pid_input_json),
        "route_result_json": None if args.route_result_json is None else str(args.route_result_json),
        "rviz_marker_input_json": None if args.rviz_marker_input_json is None else str(args.rviz_marker_input_json),
        "profile_json": str(args.profile_json),
        "floor_id_filter": args.floor_id,
        "floor_counts": floor_counts(pid_input),
        "frame_id": args.frame_id,
        "odom_topic": args.odom_topic,
        "topics": dict(TOPICS),
        "runtime_waypoint_count": len(pid_input.get("runtime_waypoints") or []),
        "selected_waypoint_count": len(waypoints),
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
        "requires_nav2": bool(pid_input.get("requires_nav2")),
        "requires_amcl": bool(pid_input.get("requires_amcl")),
        "requires_map_server": bool((pid_input.get("runtime_policy") or {}).get("requires_map_server")),
        "runtime_policy": pid_input.get("runtime_policy"),
        "profile_evidence_summary": profile.get("evidence_summary"),
        "rviz_display_policy": {
            "primary_planned_topic": TOPICS["planned_path_odom"],
            "primary_actual_topic": TOPICS["gazebo_executed_path"],
            "primary_robot_pose_topic": TOPICS["gazebo_robot_pose"],
            "primary_marker_topic": TOPICS["gazebo_actual_marker_array"],
            "replay_path_primary": False,
            "replay_robot_pose_primary": False,
        },
        "claim_boundary": {
            "gazebo_simulation_only": True,
            "physical_robot_claimed": False,
            "physical_stair_climbing_claimed": False,
            "global_collision_free_guarantee_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
        },
    }
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_json(args.output_dir / f"{query_id}_gazebo_actual_trajectory_rviz_dry_run_summary.json", summary)
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


class GazeboActualTrajectoryPublisher:
    def __init__(
        self,
        *,
        ros: dict[str, Any],
        args: argparse.Namespace,
        pid_input: dict[str, Any],
        profile: dict[str, Any],
        waypoints: list[Any],
        route_result: dict[str, Any] | None,
        marker_input: dict[str, Any] | None,
    ) -> None:
        Node = ros["Node"]
        QoSProfile = ros["QoSProfile"]
        DurabilityPolicy = ros["DurabilityPolicy"]
        Odometry = ros["Odometry"]
        NavPath = ros["NavPath"]
        PoseStamped = ros["PoseStamped"]
        MarkerArray = ros["MarkerArray"]

        class _Node(Node):
            pass

        self.node = _Node("rslg_gazebo_actual_trajectory_rviz")
        self.ros = ros
        self.args = args
        self.pid_input = pid_input
        self.profile = profile
        self.raw_waypoints = list(waypoints)
        self.query_id = query_id_from_pid_input(pid_input, args.query_id)
        self.frame_id = args.frame_id
        self.start_monotonic = time.monotonic()
        self.first_odom: Pose3D | None = None
        self.latest_pose: Pose3D | None = None
        self.anchor_transform: AnchorTransform | None = None
        self.planned_poses: list[Pose3D] = []
        self.executed_poses: list[Pose3D] = []
        self.odom_count = 0
        self.publish_count = 0
        self.raw_marker_records = build_raw_marker_records(
            route_result=route_result,
            marker_input=marker_input,
            pid_input=pid_input,
            query_id=self.query_id,
            profile_id=args.profile_id,
            frame_id=args.frame_id,
            floor_id=args.floor_id,
        )
        self.marker_records: list[dict[str, Any]] = []
        self.output_dir = args.output_dir
        self.csv_handle: Any | None = None
        self.csv_writer: csv.DictWriter[str] | None = None
        self.trajectory_csv: Path | None = None
        self.summary_json: Path | None = None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.trajectory_csv = self.output_dir / f"{self.query_id}_gazebo_actual_trajectory_rviz_trajectory.csv"
            self.summary_json = self.output_dir / f"{self.query_id}_gazebo_actual_trajectory_rviz_summary.json"
            self.csv_handle = self.trajectory_csv.open("w", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_handle,
                fieldnames=[
                    "wall_time_utc",
                    "elapsed_sec",
                    "odom_x",
                    "odom_y",
                    "odom_z",
                    "odom_yaw",
                    "odom_sequence",
                    "event",
                ],
            )
            self.csv_writer.writeheader()

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.planned_pub = self.node.create_publisher(NavPath, TOPICS["planned_path_odom"], latched_qos)
        self.executed_pub = self.node.create_publisher(NavPath, TOPICS["gazebo_executed_path"], 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, TOPICS["gazebo_robot_pose"], 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, TOPICS["gazebo_actual_marker_array"], latched_qos)
        self.subscription = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 50)
        self.timer = self.node.create_timer(1.0 / max(float(args.publish_rate_hz), 0.1), self._publish)
        self.node.get_logger().info(
            "RSLG-SLAM Gazebo actual trajectory RViz publisher ready: "
            f"query={self.query_id}, waypoints={len(self.raw_waypoints)}, frame={self.frame_id}, "
            f"odom_topic={args.odom_topic}"
        )

    def _pose_from_odom(self, msg: Any) -> Pose3D:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        return Pose3D(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
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
        self.planned_poses = transform_waypoints_visual(self.raw_waypoints, self.anchor_transform)
        self.marker_records = transform_marker_records(self.raw_marker_records, self.anchor_transform)
        for record in self.marker_records:
            record["frame_id"] = self.frame_id
        self.node.get_logger().info(
            "RSLG-SLAM Gazebo RViz odom anchor set: "
            f"route=({first.x:.3f}, {first.y:.3f}, {first.z:.3f}, yaw={first.yaw:.3f}) -> "
            f"odom=({odom_pose.x:.3f}, {odom_pose.y:.3f}, {odom_pose.z:.3f}, yaw={odom_pose.yaw:.3f})"
        )

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
        marker.ns = str(record.get("ns") or "rslg_gazebo_actual")
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
            "schema_name": "rslg_gazebo_actual_trajectory_rviz_live_summary",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "query_id": self.query_id,
            "profile_id": self.args.profile_id,
            "frame_id": self.frame_id,
            "odom_topic": self.args.odom_topic,
            "topics": dict(TOPICS),
            "selected_waypoint_count": len(self.raw_waypoints),
            "planned_path_point_count": len(self.planned_poses),
            "executed_path_point_count": len(self.executed_poses),
            "semantic_marker_count": len(self.marker_records),
            "odom_received": self.first_odom is not None,
            "odom_count": self.odom_count,
            "publish_count": self.publish_count,
            "anchor_first_waypoint_to_odom_start": bool(self.args.anchor_first_waypoint_to_odom_start),
            "anchor_applied": self.anchor_transform is not None,
            "route_anchor": {"x": first.x, "y": first.y, "z": first.z, "yaw": first.yaw},
            "odom_anchor": None
            if self.first_odom is None
            else {
                "x": self.first_odom.x,
                "y": self.first_odom.y,
                "z": self.first_odom.z,
                "yaw": self.first_odom.yaw,
            },
            "trajectory_csv": None if self.trajectory_csv is None else str(self.trajectory_csv),
            "claim_boundary": {
                "gazebo_simulation_only": True,
                "replay_primary_evidence": False,
                "nav2_required": False,
                "amcl_required": False,
                "map_server_required": False,
                "physical_robot_claimed": False,
                "physical_stair_climbing_claimed": False,
                "global_collision_free_guarantee_claimed": False,
            },
        }

    def close(self) -> None:
        if self.csv_handle is not None:
            self.csv_handle.flush()
            self.csv_handle.close()
        if self.summary_json is not None:
            write_json(self.summary_json, self.summary())


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    pid_input = read_json(args.pid_input_json)
    profile, _params = load_profile(args.profile_json, args.profile_id)
    waypoints = extract_waypoints(pid_input, args.floor_id)
    route_result = load_optional_json(args.route_result_json)
    marker_input = load_optional_json(args.rviz_marker_input_json)

    if args.dry_run:
        summary = dry_run(
            args,
            pid_input=pid_input,
            profile=profile,
            waypoints=waypoints,
            route_result=route_result,
            marker_input=marker_input,
        )
        print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
        return 0

    ros = import_ros_messages()
    rclpy = ros["rclpy"]
    rclpy.init(args=None)
    publisher = GazeboActualTrajectoryPublisher(
        ros=ros,
        args=args,
        pid_input=pid_input,
        profile=profile,
        waypoints=waypoints,
        route_result=route_result,
        marker_input=marker_input,
    )
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
