#!/usr/bin/env python3
"""Shared helpers for the task60 scripted stair-transition demo."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCENE_ID = "00843-DYehNKdT76V"
PROJECT_NAME = "RSLG-SLAM"
TASK_NAME = "task60_scripted_stair_transition_gazebo_rviz_demo"
DEFAULT_QUERY_ID = "00843_cross_floor_object_curtain_room14"
FALLBACK_QUERY_ID = "00843_cross_floor_room_room2_to_room14"
DEFAULT_PROFILE_ID = "practical_zero_collision"
TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
FORBIDDEN_NON_TRANSITION_EDGE = "vt_1_centerline_e003"
SELECTED_APPROACH_ID = "generated_ring_002"
BLOCKED_CANDIDATE_ID = "generated_ring_037"
FLOOR_1_Z = 0.0
FLOOR_2_Z = 1.6
WORLD_RELATIVE_PATH = "tools/rslg_pipeline/gazebo/worlds/rslg_scripted_stair_transition_turtlebot3_burger.world"
RVIZ_RELATIVE_PATH = "tools/rslg_pipeline/rviz/config/rslg_scripted_stair_transition_showcase.rviz"

TOPICS = {
    "composite_planned_path_odom": "/rslg/scripted_stair/composite_planned_path_odom",
    "composite_executed_path": "/rslg/scripted_stair/composite_executed_path",
    "robot_pose": "/rslg/scripted_stair/robot_pose",
    "stair_transition_path": "/rslg/scripted_stair/stair_transition_path",
    "marker_array": "/rslg/scripted_stair/marker_array",
}


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    yaw: float
    segment: str = "unknown"
    index: int = 0
    source_index: int | None = None
    floor_id: str = ""
    semantic_source: str = ""
    source_edge_id: str | None = None
    scripted_transition: bool = False


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

    def apply_pose(self, pose: Pose3D) -> Pose3D:
        offset = self.yaw_offset
        dx = pose.x - self.route_x
        dy = pose.y - self.route_y
        cos_yaw = math.cos(offset)
        sin_yaw = math.sin(offset)
        return Pose3D(
            x=self.odom_x + dx * cos_yaw - dy * sin_yaw,
            y=self.odom_y + dx * sin_yaw + dy * cos_yaw,
            z=self.odom_z + (pose.z - self.route_z),
            yaw=normalize_angle(pose.yaw + offset),
            segment=pose.segment,
            index=pose.index,
            source_index=pose.source_index,
            floor_id=pose.floor_id,
            semantic_source=pose.semantic_source,
            source_edge_id=pose.source_edge_id,
            scripted_transition=pose.scripted_transition,
        )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_between(a: dict[str, Any] | Pose3D, b: dict[str, Any] | Pose3D) -> float:
    ax = as_float(a.x if isinstance(a, Pose3D) else a.get("x"))
    ay = as_float(a.y if isinstance(a, Pose3D) else a.get("y"))
    bx = as_float(b.x if isinstance(b, Pose3D) else b.get("x"))
    by = as_float(b.y if isinstance(b, Pose3D) else b.get("y"))
    return math.atan2(by - ay, bx - ax)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_to_point_dict(pose: Pose3D) -> dict[str, float]:
    return {"x": round(pose.x, 6), "y": round(pose.y, 6), "z": round(pose.z, 6)}


def runtime_task_dir(repo_root: Path) -> Path:
    return repo_root / "stage_outputs" / "rslg_slam" / SCENE_ID / "tasks" / TASK_NAME


def runtime_pack_dir(repo_root: Path) -> Path:
    return runtime_task_dir(repo_root) / "scripted_stair_pack"


def task56c_root(repo_root: Path) -> Path:
    return (
        repo_root
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "tasks"
        / "task56c_pid_profile_promotion_and_regression_validation"
        / "regression_pack"
    )


def runtime_input_path(repo_root: Path, query_id: str) -> Path:
    return runtime_pack_dir(repo_root) / "runtime_inputs" / f"{query_id}_scripted_stair_transition_runtime_input.json"


def route_result_path(repo_root: Path, query_id: str) -> Path:
    return task56c_root(repo_root) / "route_results" / f"{query_id}_route_result.json"


def pid_input_path(repo_root: Path, query_id: str) -> Path:
    return (
        task56c_root(repo_root)
        / "runtime_adapter_inputs"
        / "pid_follower_inputs"
        / f"{query_id}_pid_runtime_input.json"
    )


def route_points(runtime_input: dict[str, Any]) -> list[Pose3D]:
    points: list[Pose3D] = []
    for raw in ((runtime_input.get("floor_1_segment") or {}).get("waypoints") or []):
        points.append(raw_point_to_pose(raw, "floor_1"))
    for raw in runtime_input.get("scripted_stair_keyframes") or []:
        points.append(raw_point_to_pose(raw, "scripted_stair"))
    for raw in ((runtime_input.get("floor_2_segment") or {}).get("waypoints") or []):
        points.append(raw_point_to_pose(raw, "floor_2"))
    return points


def stair_points(runtime_input: dict[str, Any]) -> list[Pose3D]:
    return [raw_point_to_pose(raw, "scripted_stair") for raw in runtime_input.get("scripted_stair_keyframes") or []]


def raw_point_to_pose(raw: dict[str, Any], default_segment: str) -> Pose3D:
    segment = str(raw.get("composite_segment") or raw.get("segment_key") or default_segment)
    source_index_raw = raw.get("source_index")
    source_index = None if source_index_raw in {None, ""} else int(source_index_raw)
    return Pose3D(
        x=as_float(raw.get("x")),
        y=as_float(raw.get("y")),
        z=as_float(raw.get("z")),
        yaw=as_float(raw.get("yaw")),
        segment=segment,
        index=int(raw.get("index", raw.get("waypoint_index", raw.get("keyframe_index", 0)))),
        source_index=source_index,
        floor_id=str(raw.get("floor_id") or ""),
        semantic_source=str(raw.get("semantic_source") or raw.get("source") or ""),
        source_edge_id=raw.get("source_edge_id"),
        scripted_transition=bool(raw.get("scripted_transition")),
    )


def transform_points(points: list[Pose3D], anchor: AnchorTransform | None) -> list[Pose3D]:
    if anchor is None:
        return points
    return [anchor.apply_pose(point) for point in points]


def count_route_segments(points: list[Pose3D]) -> dict[str, int]:
    counts = {"floor_1": 0, "scripted_stair": 0, "floor_2": 0, "other": 0}
    for point in points:
        if point.segment in counts:
            counts[point.segment] += 1
        else:
            counts["other"] += 1
    return counts


def distance_xy(a: Pose3D, b: Pose3D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
