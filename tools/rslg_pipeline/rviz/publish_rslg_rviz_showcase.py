#!/usr/bin/env python3
"""RViz live visualization publisher for the RSLG-SLAM practical replay chain.

This node is visualization-only. It publishes RouteResult-derived marker records,
planned/replay Path messages, and a moving robot pose/marker from a practical PID
trajectory CSV. It does not import or launch Nav2, AMCL, map_server, Gazebo,
Stage-A, navigation actions, or physical robot code.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import shutil
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Optional


TOPICS = {
    "marker_array": "/rslg/marker_array",
    "planned_path": "/rslg/planned_path",
    "pid_replay_path": "/rslg/pid_replay_path",
    "robot_pose": "/rslg/robot_pose",
}

DEFAULT_FLOOR_Z_MAP = {"floor_1": 0.0, "floor_2": 1.6}
PRIMARY_QUERY_ID = "00843_cross_floor_object_curtain_room14"
PROFILE_ID = "practical_zero_collision"
TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"
BLOCKED_CANDIDATE_ID = "generated_ring_037"
SELECTED_CANDIDATE_ID = "generated_ring_002"


@dataclass(frozen=True)
class PosePoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    floor_id: str = ""
    t: float = 0.0
    event: str = ""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def parse_floor_z_map(raw: str | None) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    data = json.loads(raw)
    return {str(key): float(value) for key, value in data.items()}


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


def point_dict(x: float, y: float, z: float) -> dict[str, float]:
    return {"x": round(float(x), 6), "y": round(float(y), 6), "z": round(float(z), 6)}


def floor_z(floor_id: str | None, floor_z_map: dict[str, float], z_scale: float) -> float:
    return float(floor_z_map.get(str(floor_id), 0.0)) * float(z_scale)


def marker_points(markers: Iterable[dict[str, Any]]) -> list[PosePoint]:
    points: list[PosePoint] = []
    for marker in markers:
        for point in marker.get("points") or []:
            if isinstance(point, dict):
                points.append(
                    PosePoint(
                        x=as_float(point.get("x")),
                        y=as_float(point.get("y")),
                        z=as_float(point.get("z")),
                    )
                )
        pose = marker.get("pose") or {}
        position = pose.get("position") if isinstance(pose, dict) else None
        if isinstance(position, dict):
            points.append(
                PosePoint(
                    x=as_float(position.get("x")),
                    y=as_float(position.get("y")),
                    z=as_float(position.get("z")),
                    yaw=as_float(marker.get("yaw")),
                    floor_id=str(marker.get("floor_id") or ""),
                )
            )
    return points


def read_trajectory_csv(path: Path, floor_z_map: dict[str, float], z_scale: float) -> list[PosePoint]:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows: list[PosePoint] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            floor_id = str(row.get("floor_id") or "")
            raw_z = as_float(row.get("z"), floor_z(floor_id, floor_z_map, 1.0))
            rows.append(
                PosePoint(
                    x=as_float(row.get("x")),
                    y=as_float(row.get("y")),
                    z=raw_z * float(z_scale),
                    yaw=as_float(row.get("yaw")),
                    floor_id=floor_id,
                    t=as_float(row.get("t")),
                    event=str(row.get("event") or ""),
                )
            )
    if not rows:
        raise ValueError(f"trajectory CSV is empty: {path}")
    return rows


def planned_path_points(
    route_result: dict[str, Any],
    marker_input: dict[str, Any],
    z_overlay: dict[str, Any],
    floor_z_map: dict[str, float],
    z_scale: float,
) -> list[PosePoint]:
    overlay_waypoints = z_overlay.get("overlay_waypoints")
    if isinstance(overlay_waypoints, list) and overlay_waypoints:
        return [
            PosePoint(
                x=as_float(point.get("x")),
                y=as_float(point.get("y")),
                z=as_float(point.get("z")) * float(z_scale),
                yaw=as_float(point.get("yaw")),
                floor_id=str(point.get("floor_id") or ""),
            )
            for point in overlay_waypoints
            if isinstance(point, dict) and "x" in point and "y" in point
        ]

    metric_path = (route_result.get("metric_path") or {}).get("path")
    if isinstance(metric_path, list) and metric_path:
        semantic = route_result.get("semantic_route") or {}
        floor_sequence = semantic.get("floor_sequence") or []
        default_floor = str(floor_sequence[0]) if floor_sequence else "floor_1"
        return [
            PosePoint(
                x=as_float(point.get("x")),
                y=as_float(point.get("y")),
                z=floor_z(str(point.get("floor_id") or default_floor), floor_z_map, z_scale),
                floor_id=str(point.get("floor_id") or default_floor),
            )
            for point in metric_path
            if isinstance(point, dict) and "x" in point and "y" in point
        ]

    for marker in marker_input.get("markers") or []:
        role = (marker.get("metadata") or {}).get("visualization_role")
        if role == "metric_path_marker" and marker.get("points"):
            return [
                PosePoint(x=as_float(point.get("x")), y=as_float(point.get("y")), z=as_float(point.get("z")))
                for point in marker.get("points")
                if isinstance(point, dict)
            ]
    return []


def route_query_id(route_result: dict[str, Any], marker_input: dict[str, Any]) -> str:
    for payload in (route_result, marker_input):
        identity = payload.get("identity") or {}
        if identity.get("query_id"):
            return str(identity["query_id"])
    return "unknown_query"


def selected_approach(route_result: dict[str, Any]) -> dict[str, Any] | None:
    approach = route_result.get("approach") or {}
    for candidate in approach.get("approach_candidates") or []:
        if candidate.get("is_runtime_goal") or candidate.get("candidate_id") == SELECTED_CANDIDATE_ID:
            return candidate
    validation = route_result.get("validation") or {}
    target = route_result.get("target") or {}
    if validation.get("endpoint_clearance") is not None and target:
        return {
            "candidate_id": SELECTED_CANDIDATE_ID,
            "floor_id": target.get("target_floor_id"),
            "yaw": -2.09057,
        }
    return None


def object_target_marker_position(
    route_result: dict[str, Any], floor_z_map: dict[str, float], z_scale: float
) -> tuple[float, float, float, str]:
    target = route_result.get("target") or {}
    approach = selected_approach(route_result) or {}
    xy = approach.get("world_xy") or []
    floor_id = str(target.get("target_floor_id") or approach.get("floor_id") or "floor_2")
    z = floor_z(floor_id, floor_z_map, z_scale) + 0.28
    if len(xy) >= 2:
        yaw = as_float(approach.get("yaw"), -2.09057)
        distance = as_float(approach.get("endpoint_to_object_distance"), 0.48)
        return float(xy[0]) + math.cos(yaw) * distance, float(xy[1]) + math.sin(yaw) * distance, z, floor_id
    return -7.25, 1.15, z, floor_id


def route_bounds(points: list[PosePoint]) -> tuple[float, float, float, float]:
    if not points:
        return -2.0, 2.0, -2.0, 2.0
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def next_marker_id(markers: list[dict[str, Any]]) -> int:
    ids = [int(marker.get("id", 0)) for marker in markers if str(marker.get("id", "")).lstrip("-").isdigit()]
    return max(ids, default=0) + 1


def add_marker(
    markers: list[dict[str, Any]],
    marker_id: int,
    *,
    ns: str,
    marker_type: str,
    frame_id: str,
    position: dict[str, float] | None = None,
    points: list[dict[str, float]] | None = None,
    scale: dict[str, float] | None = None,
    color_rgba: list[float] | None = None,
    text: str | None = None,
    yaw: float | None = None,
    floor_id: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> int:
    markers.append(
        {
            "ns": ns,
            "id": marker_id,
            "type": marker_type,
            "frame_id": frame_id,
            "floor_id": floor_id,
            "points": points or [],
            "pose": {"position": position} if position is not None else None,
            "scale": scale or {"x": 0.16, "y": 0.16, "z": 0.16},
            "color_rgba": color_rgba or [1.0, 1.0, 1.0, 1.0],
            "text": text,
            "yaw": yaw,
            "metadata": metadata or {},
        }
    )
    return marker_id + 1


def build_supplemental_markers(
    route_result: dict[str, Any],
    marker_input: dict[str, Any],
    z_overlay: dict[str, Any],
    planned_points: list[PosePoint],
    replay_points: list[PosePoint],
    profile_id: str,
    floor_z_map: dict[str, float],
    frame_id: str,
    z_scale: float,
) -> list[dict[str, Any]]:
    base_markers = list(marker_input.get("markers") or [])
    markers = [dict(marker) for marker in base_markers]
    marker_id = next_marker_id(markers)
    all_points = planned_points + replay_points + marker_points(base_markers)
    min_x, max_x, min_y, max_y = route_bounds(all_points)
    pad = 0.9
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    scale_x = max(1.0, (max_x - min_x) + 2.0 * pad)
    scale_y = max(1.0, (max_y - min_y) + 2.0 * pad)

    floor_colors = {
        "floor_1": [0.05, 0.30, 0.95, 0.10],
        "floor_2": [0.95, 0.45, 0.05, 0.10],
    }
    for floor_id in sorted(floor_z_map):
        z = floor_z(floor_id, floor_z_map, z_scale)
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_floor_overlay_{floor_id}",
            marker_type="CUBE",
            frame_id=frame_id,
            position=point_dict(center_x, center_y, z - 0.015),
            scale={"x": scale_x, "y": scale_y, "z": 0.025},
            color_rgba=floor_colors.get(floor_id, [0.4, 0.4, 0.4, 0.08]),
            floor_id=floor_id,
            metadata={"visualization_role": "z_aware_floor_overlay", "visualization_only": True},
        )

    if replay_points:
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_{profile_id}_pid_replay_trajectory",
            marker_type="LINE_STRIP",
            frame_id=frame_id,
            points=[point_dict(point.x, point.y, point.z + 0.035) for point in replay_points],
            scale={"x": 0.035, "y": 0.035, "z": 0.035},
            color_rgba=[1.0, 0.92, 0.10, 1.0],
            metadata={
                "visualization_role": "practical_pid_replay_trajectory",
                "profile_id": profile_id,
                "visualization_only": True,
            },
        )
        marker_id = add_marker(
            markers,
            marker_id,
            ns="rslg_pid_replay_start",
            marker_type="SPHERE",
            frame_id=frame_id,
            position=point_dict(replay_points[0].x, replay_points[0].y, replay_points[0].z + 0.12),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[0.10, 0.85, 1.0, 1.0],
            text=None,
            metadata={"visualization_role": "pid_replay_start"},
        )
        marker_id = add_marker(
            markers,
            marker_id,
            ns="rslg_pid_replay_finish",
            marker_type="SPHERE",
            frame_id=frame_id,
            position=point_dict(replay_points[-1].x, replay_points[-1].y, replay_points[-1].z + 0.12),
            scale={"x": 0.22, "y": 0.22, "z": 0.22},
            color_rgba=[1.0, 0.95, 0.10, 1.0],
            text=None,
            metadata={"visualization_role": "pid_replay_finish"},
        )

    target = route_result.get("target") or {}
    obj_x, obj_y, obj_z, obj_floor = object_target_marker_position(route_result, floor_z_map, z_scale)
    object_id = str(target.get("target_object_id") or "obj_175")
    object_category = str(target.get("target_object_category") or "curtain")
    room_id = str(target.get("target_room_id") or "room_14")
    marker_id = add_marker(
        markers,
        marker_id,
        ns=f"rslg_target_{object_id}_{object_category}",
        marker_type="CYLINDER",
        frame_id=frame_id,
        position=point_dict(obj_x, obj_y, obj_z),
        scale={"x": 0.30, "y": 0.30, "z": 0.56},
        color_rgba=[1.0, 0.20, 0.80, 1.0],
        floor_id=obj_floor,
        metadata={
            "visualization_role": "object_target_marker",
            "object_id": object_id,
            "object_category": object_category,
            "room_id": room_id,
            "position_source": "selected_approach_yaw_distance_estimate_if_no_centroid",
        },
    )
    marker_id = add_marker(
        markers,
        marker_id,
        ns=f"rslg_target_{object_id}_label",
        marker_type="TEXT_VIEW_FACING",
        frame_id=frame_id,
        position=point_dict(obj_x, obj_y, obj_z + 0.52),
        scale={"x": 0.22, "y": 0.22, "z": 0.22},
        color_rgba=[1.0, 0.82, 0.96, 1.0],
        text=f"{object_id} {object_category} / {room_id}",
        floor_id=obj_floor,
        metadata={"visualization_role": "object_target_label"},
    )

    selected = selected_approach(route_result) or {}
    selected_xy = selected.get("world_xy") or [-7.020484, 1.558795]
    if len(selected_xy) >= 2:
        selected_z = floor_z(str(selected.get("floor_id") or obj_floor), floor_z_map, z_scale) + 0.45
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_{SELECTED_CANDIDATE_ID}_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=point_dict(float(selected_xy[0]), float(selected_xy[1]), selected_z),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[0.30, 1.0, 0.45, 1.0],
            text=f"{SELECTED_CANDIDATE_ID} selected valid approach",
            floor_id=str(selected.get("floor_id") or obj_floor),
            metadata={"visualization_role": "selected_approach_label"},
        )

    blocked_pose = None
    for marker in markers:
        if BLOCKED_CANDIDATE_ID in json.dumps(marker, sort_keys=True):
            pose = marker.get("pose") or {}
            blocked_pose = pose.get("position") if isinstance(pose, dict) else None
            break
    if isinstance(blocked_pose, dict):
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_{BLOCKED_CANDIDATE_ID}_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=point_dict(
                as_float(blocked_pose.get("x")),
                as_float(blocked_pose.get("y")),
                as_float(blocked_pose.get("z")) + 0.35,
            ),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[1.0, 0.20, 0.20, 1.0],
            text=f"{BLOCKED_CANDIDATE_ID} blocked/rejected only",
            floor_id="floor_2",
            metadata={"visualization_role": "blocked_candidate_label", "used_as_runtime_goal": False},
        )

    handoffs = z_overlay.get("connector_handoffs") or []
    handoff = handoffs[0] if handoffs else z_overlay.get("connector_handoff")
    if isinstance(handoff, dict):
        start = handoff.get("start") or {}
        end = handoff.get("end") or {}
        mid_x = (as_float(start.get("x")) + as_float(end.get("x"))) * 0.5
        mid_y = (as_float(start.get("y")) + as_float(end.get("y"))) * 0.5
        mid_z = (as_float(start.get("z")) + as_float(end.get("z"))) * 0.5
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_{TRUE_TRANSITION_EDGE}_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=point_dict(mid_x, mid_y, mid_z + 0.22),
            scale={"x": 0.20, "y": 0.20, "z": 0.20},
            color_rgba=[0.82, 0.45, 1.0, 1.0],
            text=f"{TRUE_TRANSITION_EDGE} connector handoff",
            metadata={"visualization_role": "connector_handoff_label", "transition_edge": TRUE_TRANSITION_EDGE},
        )
        marker_id = add_marker(
            markers,
            marker_id,
            ns=f"rslg_{NON_TRANSITION_EDGE}_forbidden_label",
            marker_type="TEXT_VIEW_FACING",
            frame_id=frame_id,
            position=point_dict(mid_x + 0.35, mid_y, mid_z - 0.18),
            scale={"x": 0.18, "y": 0.18, "z": 0.18},
            color_rgba=[1.0, 0.30, 0.20, 1.0],
            text=f"{NON_TRANSITION_EDGE} non-transition only",
            metadata={
                "visualization_role": "forbidden_non_transition_edge_label",
                "transition_edge_used": False,
                "edge_id": NON_TRANSITION_EDGE,
            },
        )

    identity = marker_input.get("identity") or route_result.get("identity") or {}
    query_id = str(identity.get("query_id") or PRIMARY_QUERY_ID)
    marker_id = add_marker(
        markers,
        marker_id,
        ns="rslg_showcase_title",
        marker_type="TEXT_VIEW_FACING",
        frame_id=frame_id,
        position=point_dict(center_x, min_y - 0.55, max(floor_z_map.values(), default=1.6) * z_scale + 0.85),
        scale={"x": 0.24, "y": 0.24, "z": 0.24},
        color_rgba=[0.92, 0.95, 1.0, 1.0],
        text=f"{query_id} / {profile_id} / visualization-only RViz replay",
        metadata={"visualization_role": "query_profile_label", "visualization_only": True},
    )
    marker_id = add_marker(
        markers,
        marker_id,
        ns="rslg_claim_boundary_label",
        marker_type="TEXT_VIEW_FACING",
        frame_id=frame_id,
        position=point_dict(center_x, min_y - 1.05, max(floor_z_map.values(), default=1.6) * z_scale + 0.55),
        scale={"x": 0.18, "y": 0.18, "z": 0.18},
        color_rgba=[1.0, 0.95, 0.72, 1.0],
        text="RViz replay only: no Nav2, AMCL, map_server, physical robot, stair-climbing, or global guarantee",
        metadata={"visualization_role": "claim_boundary_text", "visualization_only": True},
    )
    return markers


def marker_has_runtime_goal(markers: Iterable[dict[str, Any]], candidate_id: str) -> bool:
    for marker in markers:
        blob = json.dumps(marker, sort_keys=True)
        metadata = marker.get("metadata") or {}
        if candidate_id in blob and bool(metadata.get("used_as_runtime_goal")):
            return True
    return False


def forbidden_transition_used(z_overlay: dict[str, Any], markers: Iterable[dict[str, Any]]) -> bool:
    if z_overlay.get("transition_edge_used") == NON_TRANSITION_EDGE:
        return True
    for handoff in z_overlay.get("connector_handoffs") or []:
        if isinstance(handoff, dict) and handoff.get("transition_edge") == NON_TRANSITION_EDGE:
            return True
    for marker in markers:
        metadata = marker.get("metadata") or {}
        if metadata.get("transition_edge") == NON_TRANSITION_EDGE and metadata.get("transition_edge_used") is not False:
            return True
    return False


def build_showcase_payload(args: argparse.Namespace) -> dict[str, Any]:
    floor_z_map = parse_floor_z_map(args.floor_z_map)
    route_result = read_json(args.route_result_json)
    marker_input = read_json(args.rviz_marker_input_json)
    z_overlay = read_json(args.z_aware_overlay_input_json)
    profile_payload = read_json(args.profile_json)
    profiles = profile_payload.get("profiles") or {}
    profile = profiles.get(args.profile_id)
    if not isinstance(profile, dict):
        raise KeyError(f"profile id not found: {args.profile_id}")
    replay_points = read_trajectory_csv(args.trajectory_csv, floor_z_map, args.z_scale)
    planned_points = planned_path_points(route_result, marker_input, z_overlay, floor_z_map, args.z_scale)
    markers = build_supplemental_markers(
        route_result=route_result,
        marker_input=marker_input,
        z_overlay=z_overlay,
        planned_points=planned_points,
        replay_points=replay_points,
        profile_id=args.profile_id,
        floor_z_map=floor_z_map,
        frame_id=args.frame_id,
        z_scale=args.z_scale,
    )
    query_id = route_query_id(route_result, marker_input)
    evidence = profile.get("evidence_summary") or {}
    summary = {
        "schema_name": "rslg_rviz_live_practical_profile_showcase_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "query_id": query_id,
        "profile_id": args.profile_id,
        "fixed_frame": args.frame_id,
        "topics": dict(TOPICS),
        "loads": {
            "route_result": True,
            "rviz_marker_input": True,
            "z_aware_overlay_input": True,
            "trajectory_csv": True,
            "profile_json": True,
        },
        "input_paths": {
            "route_result_json": args.route_result_json.as_posix(),
            "rviz_marker_input_json": args.rviz_marker_input_json.as_posix(),
            "z_aware_overlay_input_json": args.z_aware_overlay_input_json.as_posix(),
            "trajectory_csv": args.trajectory_csv.as_posix(),
            "profile_json": args.profile_json.as_posix(),
        },
        "marker_count": len(markers) + (1 if replay_points else 0),
        "static_marker_count": len(markers),
        "planned_path_point_count": len(planned_points),
        "replay_trajectory_point_count": len(replay_points),
        "floor_z_map": floor_z_map,
        "z_scale": args.z_scale,
        "rviz2_available": shutil.which("rviz2") is not None,
        "ros2_available": shutil.which("ros2") is not None,
        "rclpy_available_in_current_python": importlib.util.find_spec("rclpy") is not None,
        "dry_run": bool(args.dry_run),
        "loop": bool(args.loop),
        "rate": args.rate,
        "claim_boundary": {
            "visualization_only": True,
            "requires_nav2": False,
            "requires_amcl": False,
            "requires_map_server": False,
            "requires_gazebo": False,
            "physical_robot_claimed": False,
            "physical_stair_climbing_claimed": False,
            "global_collision_free_guarantee_claimed": False,
        },
        "guards": {
            "generated_ring_037_selected": bool(marker_has_runtime_goal(markers, BLOCKED_CANDIDATE_ID)),
            "generated_ring_037_selected_count_from_profile": int(
                evidence.get("generated_ring_037_selected_count", 0)
            ),
            "vt_1_centerline_e003_transition_used": bool(forbidden_transition_used(z_overlay, markers)),
            "vt_1_centerline_e003_transition_count_from_profile": int(
                evidence.get("vt_1_centerline_e003_transition_count", 0)
            ),
            "transition_edge_used": z_overlay.get("transition_edge_used"),
            "selected_approach_expected": SELECTED_CANDIDATE_ID,
            "true_transition_edge_expected": TRUE_TRANSITION_EDGE,
        },
        "profile_evidence": evidence,
    }
    return {
        "route_result": route_result,
        "marker_input": marker_input,
        "z_overlay": z_overlay,
        "profile": profile,
        "planned_points": planned_points,
        "replay_points": replay_points,
        "markers": markers,
        "summary": summary,
    }


def startup_summary(summary: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": "rslg_rviz_showcase_ready",
        "query_id": summary["query_id"],
        "profile_id": summary["profile_id"],
        "fixed_frame": summary["fixed_frame"],
        "topics": summary["topics"],
        "marker_count": summary["marker_count"],
        "planned_path_point_count": summary["planned_path_point_count"],
        "replay_trajectory_point_count": summary["replay_trajectory_point_count"],
        "rviz2_available": summary["rviz2_available"],
        "ros2_available": summary["ros2_available"],
        "rclpy_available_in_current_python": summary["rclpy_available_in_current_python"],
        "claim_boundary": summary["claim_boundary"],
    }


def import_ros_messages() -> dict[str, Any]:
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path as NavPath
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    return {
        "rclpy": rclpy,
        "Point": Point,
        "PoseStamped": PoseStamped,
        "NavPath": NavPath,
        "Duration": Duration,
        "Node": Node,
        "DurabilityPolicy": DurabilityPolicy,
        "QoSProfile": QoSProfile,
        "Marker": Marker,
        "MarkerArray": MarkerArray,
    }


def run_ros_publisher(args: argparse.Namespace, payload: dict[str, Any]) -> int:
    ros = import_ros_messages()
    rclpy = ros["rclpy"]
    Point = ros["Point"]
    PoseStamped = ros["PoseStamped"]
    NavPath = ros["NavPath"]
    Duration = ros["Duration"]
    Node = ros["Node"]
    DurabilityPolicy = ros["DurabilityPolicy"]
    QoSProfile = ros["QoSProfile"]
    Marker = ros["Marker"]
    MarkerArray = ros["MarkerArray"]

    type_map = {
        "ARROW": Marker.ARROW,
        "CUBE": Marker.CUBE,
        "SPHERE": Marker.SPHERE,
        "CYLINDER": Marker.CYLINDER,
        "LINE_STRIP": Marker.LINE_STRIP,
        "LINE_LIST": Marker.LINE_LIST,
        "CUBE_LIST": Marker.CUBE_LIST,
        "SPHERE_LIST": Marker.SPHERE_LIST,
        "TEXT_VIEW_FACING": Marker.TEXT_VIEW_FACING,
        "MESH_RESOURCE": Marker.MESH_RESOURCE,
        "TRIANGLE_LIST": Marker.TRIANGLE_LIST,
    }

    def to_point(data: dict[str, Any]) -> Any:
        point = Point()
        point.x = as_float(data.get("x"))
        point.y = as_float(data.get("y"))
        point.z = as_float(data.get("z"))
        return point

    class RslgRvizShowcasePublisher(Node):
        def __init__(self) -> None:
            super().__init__("rslg_practical_profile_rviz_showcase")
            latched_qos = QoSProfile(depth=1)
            latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.marker_pub = self.create_publisher(MarkerArray, TOPICS["marker_array"], latched_qos)
            self.planned_path_pub = self.create_publisher(NavPath, TOPICS["planned_path"], latched_qos)
            self.replay_path_pub = self.create_publisher(NavPath, TOPICS["pid_replay_path"], latched_qos)
            self.robot_pose_pub = self.create_publisher(PoseStamped, TOPICS["robot_pose"], 10)
            self.frame_id = args.frame_id
            self.markers = list(payload["markers"])
            self.planned_points = list(payload["planned_points"])
            self.replay_points = list(payload["replay_points"])
            self.index = 0
            self.publish_count = 0
            self.timer = self.create_timer(1.0 / max(args.rate, 0.1), self.publish_once)

        def marker_from_dict(self, record: dict[str, Any]) -> Any:
            marker = Marker()
            marker.header.frame_id = str(record.get("frame_id") or self.frame_id)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = str(record.get("ns") or "rslg_showcase")
            marker.id = int(record.get("id", 0))
            marker.type = type_map.get(str(record.get("type") or "SPHERE"), Marker.SPHERE)
            marker.action = Marker.ADD
            scale = record.get("scale") or {}
            marker.scale.x = as_float(scale.get("x"), 0.16)
            marker.scale.y = as_float(scale.get("y"), marker.scale.x)
            marker.scale.z = as_float(scale.get("z"), marker.scale.x)
            color = record.get("color_rgba") or [1.0, 1.0, 1.0, 1.0]
            marker.color.r = as_float(color[0], 1.0)
            marker.color.g = as_float(color[1], 1.0)
            marker.color.b = as_float(color[2], 1.0)
            marker.color.a = as_float(color[3], 1.0)
            for point in record.get("points") or []:
                if isinstance(point, dict):
                    marker.points.append(to_point(point))
            pose = record.get("pose") or {}
            position = pose.get("position") if isinstance(pose, dict) else None
            if isinstance(position, dict):
                marker.pose.position = to_point(position)
            orientation = pose.get("orientation") if isinstance(pose, dict) else None
            if isinstance(orientation, dict):
                marker.pose.orientation.x = as_float(orientation.get("x"))
                marker.pose.orientation.y = as_float(orientation.get("y"))
                marker.pose.orientation.z = as_float(orientation.get("z"))
                marker.pose.orientation.w = as_float(orientation.get("w"), 1.0)
            elif record.get("yaw") is not None:
                qx, qy, qz, qw = yaw_to_quaternion(as_float(record.get("yaw")))
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
            else:
                marker.pose.orientation.w = 1.0
            marker.text = str(record.get("text") or "")
            marker.lifetime = Duration(seconds=0).to_msg()
            return marker

        def path_msg(self, points: list[PosePoint]) -> Any:
            stamp = self.get_clock().now().to_msg()
            path = NavPath()
            path.header.frame_id = self.frame_id
            path.header.stamp = stamp
            for point in points:
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = stamp
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.position.z = point.z
                qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path.poses.append(pose)
            return path

        def pose_msg(self, point: PosePoint) -> Any:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z + 0.10
            qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            return pose

        def robot_marker(self, point: PosePoint) -> dict[str, Any]:
            return {
                "ns": "rslg_moving_robot_replay_marker",
                "id": 1,
                "type": "ARROW",
                "frame_id": self.frame_id,
                "pose": {"position": point_dict(point.x, point.y, point.z + 0.22)},
                "scale": {"x": 0.42, "y": 0.16, "z": 0.16},
                "color_rgba": [1.0, 1.0, 1.0, 1.0],
                "yaw": point.yaw,
                "metadata": {
                    "visualization_role": "moving_robot_marker",
                    "profile_id": args.profile_id,
                    "trajectory_index": self.index,
                    "visualization_only": True,
                },
            }

        def publish_once(self) -> None:
            if not self.replay_points:
                return
            if self.index >= len(self.replay_points):
                if args.loop:
                    self.index = 0
                else:
                    self.index = len(self.replay_points) - 1
            robot_point = self.replay_points[self.index]
            marker_array = MarkerArray()
            for marker_record in self.markers:
                marker_array.markers.append(self.marker_from_dict(marker_record))
            marker_array.markers.append(self.marker_from_dict(self.robot_marker(robot_point)))
            self.marker_pub.publish(marker_array)
            self.planned_path_pub.publish(self.path_msg(self.planned_points))
            self.replay_path_pub.publish(self.path_msg(self.replay_points))
            self.robot_pose_pub.publish(self.pose_msg(robot_point))
            if self.index < len(self.replay_points) - 1:
                self.index += 1
            elif args.loop:
                self.index = 0
            self.publish_count += 1

    rclpy.init(args=None)
    node = RslgRvizShowcasePublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rviz-marker-input-json", type=Path, required=True)
    parser.add_argument("--z-aware-overlay-input-json", type=Path, required=True)
    parser.add_argument("--trajectory-csv", type=Path, required=True)
    parser.add_argument("--route-result-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--profile-id", default=PROFILE_ID)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--z-scale", type=float, default=1.0)
    parser.add_argument("--floor-z-map", default=json.dumps(DEFAULT_FLOOR_Z_MAP))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--no-canonical-write", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.no_canonical_write and args.summary_json and "canonical" in args.summary_json.resolve().parts:
        raise SystemExit("--no-canonical-write refused summary output under a canonical directory")
    payload = build_showcase_payload(args)
    summary = payload["summary"]
    summary["dry_run_success"] = bool(args.dry_run)
    if args.summary_json:
        write_json(args.summary_json, summary)
    print(json.dumps(startup_summary(summary), indent=2, sort_keys=False), flush=True)
    if args.dry_run:
        return 0
    if not summary["rclpy_available_in_current_python"]:
        raise SystemExit(
            "rclpy is not available in this Python. Source a ROS 2 environment and run with "
            "/usr/bin/python3 or set RSLG_ROS_PYTHON to the ROS-enabled interpreter."
        )
    return run_ros_publisher(args, payload)


if __name__ == "__main__":
    raise SystemExit(main())

