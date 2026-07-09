#!/usr/bin/env python3
"""Replay RouteResult-derived PID runtime inputs with a lightweight model.

This Layer 4 validation tool consumes ``rslg_route_result_pid_runtime_input``
JSON files and runs a deterministic 2D unicycle/proportional waypoint follower.
It does not import ROS, Nav2, AMCL, Gazebo, RViz, ``stage_a_demo.py``, or
``demo.py``. Cross-floor connector records are handled as semantic handoff
events only; no physical stair-climbing trajectory is synthesized.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

try:  # NumPy is used only for stable-map-backed footprint checks.
    import numpy as np
except Exception:  # pragma: no cover - exercised only when NumPy is unavailable.
    np = None  # type: ignore[assignment]

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_ID,
    NON_TRANSITION_EDGE,
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
)

SCHEMA_NAME = "rslg_pid_runtime_replay_summary"
REPORT_SCHEMA_NAME = "rslg_pid_runtime_execution_report"
PID_SCHEMA_NAME = "rslg_route_result_pid_runtime_input"
DEFAULT_FLOOR_Z_MAP = {"floor_1": 0.0, "floor_2": 1.6}


@dataclass
class ReplayParams:
    dt: float
    max_linear_velocity: float
    max_angular_velocity: float
    linear_gain: float
    angular_gain: float
    waypoint_tolerance: float
    yaw_tolerance: float
    timeout_sec: float
    stuck_window_sec: float
    stuck_progress_epsilon: float
    robot_radius: float
    floor_z_map: dict[str, float]
    title: str


@dataclass
class State:
    x: float
    y: float
    yaw: float
    floor_id: str
    z: float
    segment_id: Optional[str]
    waypoint_index: int
    t: float


@dataclass
class StableMap:
    floor_id: str
    resolution: float
    origin_x: float
    origin_y: float
    shape_hw: tuple[int, int]
    source_npz: str
    source_metadata: str
    source_package: str
    free_mask: Any
    occupied_mask: Any
    unknown_mask: Any

    def row_col(self, x: float, y: float) -> tuple[int, int]:
        row = int(round((y - self.origin_y) / self.resolution))
        col = int(round((x - self.origin_x) / self.resolution))
        return row, col

    def in_bounds(self, row: int, col: int) -> bool:
        height, width = self.shape_hw
        return 0 <= row < height and 0 <= col < width


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def distance_xy(a: dict[str, Any] | State, b: dict[str, Any]) -> float:
    ax = a.x if isinstance(a, State) else float(a["x"])
    ay = a.y if isinstance(a, State) else float(a["y"])
    return math.hypot(float(b["x"]) - ax, float(b["y"]) - ay)


def parse_floor_z_map(raw: Optional[str]) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(key): float(value) for key, value in json.loads(raw).items()}


def _resolve_path(raw: str | Path, *, bases: list[Path]) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    for base in bases:
        candidate = base / path
        if candidate.exists():
            return candidate
    return Path.cwd() / path


def find_json_by_query(directory: Optional[Path], query_id: str, suffix: str) -> Optional[Path]:
    if directory is None or not directory.exists():
        return None
    exact = directory / f"{query_id}_{suffix}.json"
    if exact.exists():
        return exact
    matches = sorted(directory.glob(f"{query_id}*{suffix}.json"))
    return matches[0] if matches else None


def load_route_result(
    pid_input: dict[str, Any],
    pid_path: Path,
    route_results_dir: Optional[Path],
) -> tuple[Optional[dict[str, Any]], Optional[Path]]:
    query_id = str((pid_input.get("identity") or {}).get("query_id") or "")
    candidates: list[Path] = []
    source = pid_input.get("source_route_result")
    if source:
        candidates.append(_resolve_path(str(source), bases=[Path.cwd(), pid_path.parent]))
    matched = find_json_by_query(route_results_dir, query_id, "route_result")
    if matched:
        candidates.append(matched)
    for candidate in candidates:
        try:
            if candidate.exists():
                return read_json(candidate), candidate
        except (OSError, json.JSONDecodeError):
            continue
    return None, None


def load_z_aware_input(
    query_id: str,
    z_aware_inputs_dir: Optional[Path],
) -> tuple[Optional[dict[str, Any]], Optional[Path]]:
    path = find_json_by_query(z_aware_inputs_dir, query_id, "z_aware_overlay_input")
    if not path:
        return None, None
    try:
        return read_json(path), path
    except (OSError, json.JSONDecodeError):
        return None, path


def discover_canonical_roots(
    pid_inputs: list[dict[str, Any]],
    route_results: list[Optional[dict[str, Any]]],
) -> list[Path]:
    roots: list[Path] = []
    for route_result in route_results:
        if not route_result:
            continue
        root = (route_result.get("provenance") or {}).get("canonical_root")
        if root:
            path = _resolve_path(str(root), bases=[Path.cwd()])
            if path.exists() and path not in roots:
                roots.append(path)
    for pid_input in pid_inputs:
        scene_id = (pid_input.get("identity") or {}).get("scene_id")
        if scene_id:
            path = Path("stage_outputs") / "rslg_slam" / str(scene_id) / "canonical"
            if path.exists() and path not in roots:
                roots.append(path)
    return roots


def load_stable_maps(
    canonical_roots: list[Path],
) -> tuple[dict[str, StableMap], dict[str, Any]]:
    info: dict[str, Any] = {
        "mode": "not_available",
        "reason": None,
        "map_sources": [],
        "map_resolution": None,
    }
    if np is None:
        info["reason"] = "numpy_not_available"
        return {}, info

    maps: dict[str, StableMap] = {}
    for root in canonical_roots:
        package_path = root / "layer2_formal_artifacts" / "stable_maps" / "stable_occupancy_map_package_v0_1.json"
        if not package_path.exists():
            continue
        try:
            package = read_json(package_path)
        except (OSError, json.JSONDecodeError):
            continue
        for floor_map in package.get("floor_maps") or []:
            floor_id = str(floor_map.get("floor_id") or "")
            outputs = floor_map.get("outputs") or {}
            npz_path_raw = outputs.get("npz")
            if not floor_id or not npz_path_raw:
                continue
            npz_path = _resolve_path(str(npz_path_raw), bases=[Path.cwd(), package_path.parent])
            if not npz_path.exists():
                continue
            try:
                data = np.load(npz_path)
                free_mask = data["free_mask"]
                occupied_mask = data["occupied_mask"]
                unknown_mask = data["unknown_mask"]
            except Exception:
                continue
            origin = floor_map.get("origin") or [0.0, 0.0, 0.0]
            shape_hw = floor_map.get("shape_hw") or list(free_mask.shape)
            stable_map = StableMap(
                floor_id=floor_id,
                resolution=float(floor_map.get("resolution_m_per_cell") or 0.05),
                origin_x=float(origin[0]),
                origin_y=float(origin[1]),
                shape_hw=(int(shape_hw[0]), int(shape_hw[1])),
                source_npz=npz_path.as_posix(),
                source_metadata=str(floor_map.get("metadata_json") or ""),
                source_package=package_path.as_posix(),
                free_mask=free_mask,
                occupied_mask=occupied_mask,
                unknown_mask=unknown_mask,
            )
            maps[floor_id] = stable_map

    if maps:
        resolutions = sorted({m.resolution for m in maps.values()})
        info["mode"] = "stable_map_footprint"
        info["reason"] = None
        info["map_sources"] = [
            {
                "floor_id": floor_id,
                "npz": stable_map.source_npz,
                "metadata": stable_map.source_metadata,
                "package": stable_map.source_package,
                "resolution_m_per_cell": stable_map.resolution,
                "origin": [stable_map.origin_x, stable_map.origin_y],
                "shape_hw": list(stable_map.shape_hw),
            }
            for floor_id, stable_map in sorted(maps.items())
        ]
        info["map_resolution"] = resolutions[0] if len(resolutions) == 1 else resolutions
    else:
        info["reason"] = "stable_map_package_not_found_or_unloadable"
    return maps, info


def footprint_offsets(radius: float, resolution: float) -> list[tuple[int, int]]:
    cell_radius = int(math.ceil(radius / resolution))
    offsets: list[tuple[int, int]] = []
    for dr in range(-cell_radius, cell_radius + 1):
        for dc in range(-cell_radius, cell_radius + 1):
            if math.hypot(dr * resolution, dc * resolution) <= radius + 1e-9:
                offsets.append((dr, dc))
    return offsets


def center_cell_state(stable_map: StableMap, row: int, col: int) -> str:
    if not stable_map.in_bounds(row, col):
        return "out_of_bounds"
    if bool(stable_map.occupied_mask[row, col]):
        return "occupied"
    if bool(stable_map.unknown_mask[row, col]):
        return "unknown"
    if bool(stable_map.free_mask[row, col]):
        return "free"
    return "nonfree"


def footprint_check(
    stable_maps: dict[str, StableMap],
    state: State,
    robot_radius: float,
) -> dict[str, Any]:
    stable_map = stable_maps.get(state.floor_id)
    if stable_map is None:
        return {
            "checked": False,
            "mode": "not_available",
            "cell_state": "not_checked",
            "collision": False,
            "invalid": False,
            "min_clearance_m": None,
        }

    row, col = stable_map.row_col(state.x, state.y)
    state_name = center_cell_state(stable_map, row, col)
    collision = False
    invalid = state_name == "out_of_bounds"
    for dr, dc in footprint_offsets(robot_radius, stable_map.resolution):
        rr = row + dr
        cc = col + dc
        if not stable_map.in_bounds(rr, cc):
            invalid = True
            continue
        if bool(stable_map.occupied_mask[rr, cc]):
            collision = True
        elif bool(stable_map.unknown_mask[rr, cc]) or not bool(stable_map.free_mask[rr, cc]):
            invalid = True

    min_clearance = local_min_occupied_clearance(stable_map, state.x, state.y, robot_radius)
    return {
        "checked": True,
        "mode": "stable_map_footprint",
        "cell_state": state_name,
        "collision": collision,
        "invalid": invalid,
        "min_clearance_m": min_clearance,
    }


def local_min_occupied_clearance(
    stable_map: StableMap,
    x: float,
    y: float,
    robot_radius: float,
    search_radius_m: float = 0.8,
) -> Optional[float]:
    if np is None:
        return None
    row, col = stable_map.row_col(x, y)
    search_cells = int(math.ceil(search_radius_m / stable_map.resolution))
    r0 = max(0, row - search_cells)
    r1 = min(stable_map.shape_hw[0], row + search_cells + 1)
    c0 = max(0, col - search_cells)
    c1 = min(stable_map.shape_hw[1], col + search_cells + 1)
    if r0 >= r1 or c0 >= c1:
        return None
    occupied = stable_map.occupied_mask[r0:r1, c0:c1]
    if not bool(occupied.any()):
        return None
    occ_rows, occ_cols = np.nonzero(occupied)
    world_y = stable_map.origin_y + (occ_rows + r0) * stable_map.resolution
    world_x = stable_map.origin_x + (occ_cols + c0) * stable_map.resolution
    distances = np.sqrt((world_x - x) ** 2 + (world_y - y) ** 2)
    center_clearance = float(distances.min()) - robot_radius
    return round(center_clearance, 6)


def sanitize_waypoints(pid_input: dict[str, Any]) -> list[dict[str, Any]]:
    waypoints: list[dict[str, Any]] = []
    for raw in pid_input.get("runtime_waypoints") or []:
        if not isinstance(raw, dict) or "x" not in raw or "y" not in raw:
            continue
        floor_id = str(raw.get("floor_id") or "")
        waypoints.append(
            {
                "waypoint_index": int(raw.get("waypoint_index", len(waypoints))),
                "x": float(raw["x"]),
                "y": float(raw["y"]),
                "z": float(raw.get("z") or 0.0),
                "floor_id": floor_id,
                "yaw": float(raw["yaw"]) if raw.get("yaw") is not None else None,
                "segment_id": raw.get("segment_id"),
            }
        )
    return waypoints


def floor_sequence_from_waypoints(waypoints: list[dict[str, Any]]) -> list[str]:
    floors: list[str] = []
    for wp in waypoints:
        floor_id = str(wp.get("floor_id") or "")
        if floor_id and (not floors or floors[-1] != floor_id):
            floors.append(floor_id)
    return floors


def planned_path_length(waypoints: list[dict[str, Any]]) -> float:
    total = 0.0
    for before, after in zip(waypoints, waypoints[1:]):
        if before.get("floor_id") != after.get("floor_id"):
            continue
        total += math.hypot(float(after["x"]) - float(before["x"]), float(after["y"]) - float(before["y"]))
    return round(total, 6)


def build_floor_polylines(waypoints: list[dict[str, Any]]) -> dict[str, list[list[tuple[float, float]]]]:
    polylines: dict[str, list[list[tuple[float, float]]]] = {}
    current_floor: Optional[str] = None
    current_line: list[tuple[float, float]] = []
    for wp in waypoints:
        floor_id = str(wp.get("floor_id") or "")
        point = (float(wp["x"]), float(wp["y"]))
        if floor_id != current_floor:
            if current_floor and current_line:
                polylines.setdefault(current_floor, []).append(current_line)
            current_floor = floor_id
            current_line = [point]
        else:
            current_line.append(point)
    if current_floor and current_line:
        polylines.setdefault(current_floor, []).append(current_line)
    return polylines


def point_to_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp((wx * vx + wy * vy) / denom, 0.0, 1.0)
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def tracking_error_to_plan(
    state: State,
    floor_polylines: dict[str, list[list[tuple[float, float]]]],
) -> float:
    best: Optional[float] = None
    for line in floor_polylines.get(state.floor_id, []):
        if len(line) == 1:
            dist = math.hypot(state.x - line[0][0], state.y - line[0][1])
            best = dist if best is None else min(best, dist)
        for before, after in zip(line, line[1:]):
            dist = point_to_segment_distance(state.x, state.y, before[0], before[1], after[0], after[1])
            best = dist if best is None else min(best, dist)
    return round(best if best is not None else 0.0, 6)


def yaw_toward_next(
    waypoints: list[dict[str, Any]],
    index: int,
    fallback: float = 0.0,
) -> float:
    if index >= len(waypoints):
        return fallback
    current = waypoints[index]
    for nxt in waypoints[index + 1 :]:
        if nxt.get("floor_id") == current.get("floor_id"):
            dx = float(nxt["x"]) - float(current["x"])
            dy = float(nxt["y"]) - float(current["y"])
            if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                return math.atan2(dy, dx)
        else:
            break
    yaw = current.get("yaw")
    return float(yaw) if yaw is not None else fallback


def remaining_planned_distance(
    state: State,
    waypoints: list[dict[str, Any]],
    active_index: int,
) -> float:
    if active_index >= len(waypoints):
        return 0.0
    total = 0.0
    prev_floor = state.floor_id
    prev_x = state.x
    prev_y = state.y
    for wp in waypoints[active_index:]:
        floor_id = str(wp.get("floor_id") or "")
        if floor_id == prev_floor:
            total += math.hypot(float(wp["x"]) - prev_x, float(wp["y"]) - prev_y)
        prev_floor = floor_id
        prev_x = float(wp["x"])
        prev_y = float(wp["y"])
    return total


def route_sequences(route_result: Optional[dict[str, Any]], waypoints: list[dict[str, Any]]) -> dict[str, Any]:
    if route_result:
        semantic = route_result.get("semantic_route") or {}
        return {
            "floor_sequence": semantic.get("floor_sequence") or floor_sequence_from_waypoints(waypoints),
            "room_sequence": semantic.get("room_sequence") or [],
            "connector_sequence": semantic.get("connector_sequence") or [],
        }
    return {
        "floor_sequence": floor_sequence_from_waypoints(waypoints),
        "room_sequence": [],
        "connector_sequence": [],
    }


def transition_edges(pid_input: dict[str, Any], route_result: Optional[dict[str, Any]]) -> list[str]:
    edges: list[str] = []
    for handoff in pid_input.get("connector_handoffs") or []:
        edge = (handoff or {}).get("transition_edge")
        if edge:
            edges.append(str(edge))
    if route_result:
        for connector in (route_result.get("semantic_route") or {}).get("connector_sequence") or []:
            edge = (connector or {}).get("transition_edge")
            if edge:
                edges.append(str(edge))
        for segment in route_result.get("route_segments") or []:
            if not isinstance(segment, dict):
                continue
            if segment.get("segment_type") in {"vertical_transition", "connector_handoff"}:
                edge = segment.get("transition_edge") or segment.get("connector_edge")
                if edge:
                    edges.append(str(edge))
    return edges


def selected_object_approach(pid_input: dict[str, Any], route_result: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    selected = pid_input.get("selected_goal") or pid_input.get("selected_approach")
    if isinstance(selected, dict) and selected.get("candidate_id"):
        return selected
    if route_result:
        rr_selected = (route_result.get("approach") or {}).get("selected_approach")
        if isinstance(rr_selected, dict):
            return rr_selected
    return None


def segment_report_template(pid_input: dict[str, Any], waypoints: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    by_segment: dict[str, dict[str, Any]] = {}
    segment_meta: dict[str, dict[str, Any]] = {}
    for seg in pid_input.get("runtime_segments") or []:
        if not isinstance(seg, dict):
            continue
        sid = str(seg.get("segment_id") or "")
        if sid:
            segment_meta[sid] = seg
    for wp in waypoints:
        sid = str(wp.get("segment_id") or "unknown_segment")
        meta = segment_meta.get(sid, {})
        record = by_segment.setdefault(
            sid,
            {
                "segment_id": sid,
                "segment_type": meta.get("segment_type"),
                "floor_id": wp.get("floor_id") or meta.get("floor_id"),
                "waypoints_total": 0,
                "waypoints_reached": 0,
                "collision_count": 0,
                "invalid_cell_count": 0,
                "duration_sec": 0.0,
                "path_length_executed": 0.0,
                "status": "not_started",
            },
        )
        record["waypoints_total"] += 1
    return by_segment


def trajectory_row(
    state: State,
    *,
    event: str,
    v: float,
    w: float,
    tracking_error: float,
    footprint: dict[str, Any],
) -> dict[str, Any]:
    return {
        "t": round(state.t, 3),
        "x": round(state.x, 6),
        "y": round(state.y, 6),
        "yaw": round(state.yaw, 6),
        "floor_id": state.floor_id,
        "z": round(state.z, 6),
        "segment_id": state.segment_id or "",
        "waypoint_index": state.waypoint_index,
        "event": event,
        "v": round(v, 6),
        "w": round(w, 6),
        "tracking_error": tracking_error,
        "cell_state": footprint.get("cell_state"),
        "collision": bool(footprint.get("collision")),
        "invalid_cell": bool(footprint.get("invalid")),
        "min_clearance_m": footprint.get("min_clearance_m"),
    }


def simulate_query(
    pid_input: dict[str, Any],
    pid_path: Path,
    route_result: Optional[dict[str, Any]],
    route_result_path: Optional[Path],
    z_aware_path: Optional[Path],
    stable_maps: dict[str, StableMap],
    stable_map_info: dict[str, Any],
    params: ReplayParams,
    output_dir: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    identity = pid_input.get("identity") or {}
    query_id = str(identity.get("query_id") or pid_path.stem.replace("_pid_runtime_input", ""))
    query_type = identity.get("query_type")
    waypoints = sanitize_waypoints(pid_input)
    reports_dir = output_dir / "reports"
    trajectories_dir = output_dir / "trajectories"
    report_json_path = reports_dir / f"{query_id}_execution_report.json"
    report_md_path = reports_dir / f"{query_id}_execution_report.md"
    trajectory_csv_path = trajectories_dir / f"{query_id}_trajectory.csv"
    trajectory_svg_path = trajectories_dir / f"{query_id}_trajectory.svg"

    selected = selected_object_approach(pid_input, route_result)
    selected_candidate_id = selected.get("candidate_id") if isinstance(selected, dict) else None
    blocked_candidate_selected = selected_candidate_id in BLOCKED_LEGACY_APPROACH_IDS
    edges = transition_edges(pid_input, route_result)
    forbidden_transition_used = NON_TRANSITION_EDGE in edges
    sequences = route_sequences(route_result, waypoints)

    if not waypoints:
        report = base_failure_report(
            query_id=query_id,
            query_type=query_type,
            pid_path=pid_path,
            route_result_path=route_result_path,
            z_aware_path=z_aware_path,
            selected=selected,
            sequences=sequences,
            reason="no_runtime_waypoints",
            params=params,
            stable_map_info=stable_map_info,
            report_json_path=report_json_path,
            report_md_path=report_md_path,
            trajectory_csv_path=trajectory_csv_path,
            trajectory_svg_path=trajectory_svg_path,
        )
        return report, []

    floor_polylines = build_floor_polylines(waypoints)
    planned_length = planned_path_length(waypoints)
    state = State(
        x=float(waypoints[0]["x"]),
        y=float(waypoints[0]["y"]),
        yaw=yaw_toward_next(waypoints, 0),
        floor_id=str(waypoints[0].get("floor_id") or ""),
        z=float(waypoints[0].get("z") or params.floor_z_map.get(str(waypoints[0].get("floor_id")), 0.0)),
        segment_id=waypoints[0].get("segment_id"),
        waypoint_index=int(waypoints[0].get("waypoint_index") or 0),
        t=0.0,
    )

    segment_reports = segment_report_template(pid_input, waypoints)
    waypoint_reached_by_segment: dict[str, int] = {}
    collision_count = 0
    invalid_cell_count = 0
    min_clearance_values: list[float] = []
    tracking_errors: list[float] = []
    trajectory: list[dict[str, Any]] = []
    handoff_events: list[dict[str, Any]] = []
    path_length_executed = 0.0
    timeout = False
    stuck_detected = False
    failure_reasons: list[str] = []
    handoff_index = 0
    active_index = 0
    waypoints_reached = 0

    def record_sample(event: str, v: float = 0.0, w: float = 0.0) -> None:
        nonlocal collision_count, invalid_cell_count
        tracking_error = tracking_error_to_plan(state, floor_polylines)
        footprint = footprint_check(stable_maps, state, params.robot_radius)
        if footprint.get("collision"):
            collision_count += 1
            if state.segment_id:
                segment_reports.setdefault(str(state.segment_id), {}).setdefault("collision_count", 0)
                segment_reports[str(state.segment_id)]["collision_count"] += 1
        if footprint.get("invalid"):
            invalid_cell_count += 1
            if state.segment_id:
                segment_reports.setdefault(str(state.segment_id), {}).setdefault("invalid_cell_count", 0)
                segment_reports[str(state.segment_id)]["invalid_cell_count"] += 1
        if footprint.get("min_clearance_m") is not None:
            min_clearance_values.append(float(footprint["min_clearance_m"]))
        tracking_errors.append(tracking_error)
        trajectory.append(
            trajectory_row(
                state,
                event=event,
                v=v,
                w=w,
                tracking_error=tracking_error,
                footprint=footprint,
            )
        )

    def mark_waypoint_reached(wp: dict[str, Any]) -> None:
        nonlocal waypoints_reached
        waypoints_reached += 1
        sid = str(wp.get("segment_id") or "unknown_segment")
        waypoint_reached_by_segment[sid] = waypoint_reached_by_segment.get(sid, 0) + 1
        if sid in segment_reports:
            segment_reports[sid]["waypoints_reached"] = waypoint_reached_by_segment[sid]
            segment_reports[sid]["status"] = "in_progress"

    record_sample("start")
    mark_waypoint_reached(waypoints[0])
    active_index = 1
    progress_window: deque[tuple[float, float]] = deque()
    progress_window.append((state.t, remaining_planned_distance(state, waypoints, active_index)))

    while active_index < len(waypoints):
        if state.t > params.timeout_sec:
            timeout = True
            failure_reasons.append("timeout")
            break
        target = waypoints[active_index]
        target_floor = str(target.get("floor_id") or "")
        if target_floor != state.floor_id:
            before = state_to_record(state)
            handoff = (pid_input.get("connector_handoffs") or [])
            handoff_record = handoff[handoff_index] if handoff_index < len(handoff) else {}
            state.x = float(target["x"])
            state.y = float(target["y"])
            state.floor_id = target_floor
            state.z = float(target.get("z") or params.floor_z_map.get(target_floor, 0.0))
            state.segment_id = target.get("segment_id")
            state.waypoint_index = int(target.get("waypoint_index") or active_index)
            state.yaw = yaw_toward_next(waypoints, active_index, fallback=state.yaw)
            after = state_to_record(state)
            event = {
                "event_index": handoff_index,
                "time_sec": round(state.t, 3),
                "connector_id": handoff_record.get("connector_id"),
                "transition_edge": handoff_record.get("transition_edge"),
                "from_floor": handoff_record.get("from_floor") or before["floor_id"],
                "to_floor": handoff_record.get("to_floor") or after["floor_id"],
                "before_state": before,
                "after_state": after,
                "semantic_handoff_only": True,
                "physical_stair_climbing_claimed": False,
            }
            handoff_events.append(event)
            handoff_index += 1
            record_sample("connector_handoff")
            mark_waypoint_reached(target)
            active_index += 1
            progress_window.clear()
            progress_window.append((state.t, remaining_planned_distance(state, waypoints, active_index)))
            continue

        dist = distance_xy(state, target)
        if dist <= params.waypoint_tolerance:
            state.segment_id = target.get("segment_id")
            state.waypoint_index = int(target.get("waypoint_index") or active_index)
            mark_waypoint_reached(target)
            record_sample("waypoint_reached")
            active_index += 1
            progress_window.clear()
            progress_window.append((state.t, remaining_planned_distance(state, waypoints, active_index)))
            continue

        heading = math.atan2(float(target["y"]) - state.y, float(target["x"]) - state.x)
        heading_error = normalize_angle(heading - state.yaw)
        w_cmd = clamp(params.angular_gain * heading_error, -params.max_angular_velocity, params.max_angular_velocity)
        if abs(heading_error) > params.yaw_tolerance:
            v_cmd = 0.0
        else:
            v_cmd = clamp(params.linear_gain * dist, 0.0, params.max_linear_velocity)

        prev_x, prev_y = state.x, state.y
        state.x += v_cmd * math.cos(state.yaw) * params.dt
        state.y += v_cmd * math.sin(state.yaw) * params.dt
        state.yaw = normalize_angle(state.yaw + w_cmd * params.dt)
        state.t = round(state.t + params.dt, 10)
        state.segment_id = target.get("segment_id")
        state.waypoint_index = int(target.get("waypoint_index") or active_index)
        step_distance = math.hypot(state.x - prev_x, state.y - prev_y)
        path_length_executed += step_distance
        if state.segment_id and str(state.segment_id) in segment_reports:
            seg = segment_reports[str(state.segment_id)]
            seg["duration_sec"] = round(float(seg.get("duration_sec") or 0.0) + params.dt, 6)
            seg["path_length_executed"] = round(float(seg.get("path_length_executed") or 0.0) + step_distance, 6)
            seg["status"] = "in_progress"
        record_sample("follow", v=v_cmd, w=w_cmd)

        remaining = remaining_planned_distance(state, waypoints, active_index)
        progress_window.append((state.t, remaining))
        while progress_window and state.t - progress_window[0][0] > params.stuck_window_sec:
            progress_window.popleft()
        if progress_window and state.t >= params.stuck_window_sec:
            old_t, old_remaining = progress_window[0]
            if state.t - old_t >= params.stuck_window_sec - 1e-9:
                if old_remaining - remaining < params.stuck_progress_epsilon and dist > params.waypoint_tolerance:
                    stuck_detected = True
                    failure_reasons.append("stuck_detected")
                    break

    final_yaw_error: Optional[float] = None
    final_yaw = None
    endpoint = pid_input.get("endpoint") or pid_input.get("selected_goal") or {}
    if isinstance(endpoint, dict) and endpoint.get("yaw") is not None:
        final_yaw = float(endpoint["yaw"])
    if active_index >= len(waypoints) and final_yaw is not None:
        while state.t <= params.timeout_sec:
            final_yaw_error = abs(normalize_angle(final_yaw - state.yaw))
            if final_yaw_error <= params.yaw_tolerance:
                break
            w_cmd = clamp(params.angular_gain * normalize_angle(final_yaw - state.yaw), -params.max_angular_velocity, params.max_angular_velocity)
            state.yaw = normalize_angle(state.yaw + w_cmd * params.dt)
            state.t = round(state.t + params.dt, 10)
            record_sample("final_yaw_align", v=0.0, w=w_cmd)
        final_yaw_error = abs(normalize_angle(final_yaw - state.yaw))
        if final_yaw_error > params.yaw_tolerance:
            failure_reasons.append("final_yaw_error_exceeds_tolerance")

    if state.t > params.timeout_sec and not timeout:
        timeout = True
        failure_reasons.append("timeout")

    for sid, seg in segment_reports.items():
        if seg.get("waypoints_reached", 0) >= seg.get("waypoints_total", 0):
            seg["status"] = "passed"
        elif seg.get("waypoints_reached", 0) > 0:
            seg["status"] = "partial"
        else:
            seg["status"] = "not_reached"

    final_target = waypoints[-1]
    final_position_error = math.hypot(state.x - float(final_target["x"]), state.y - float(final_target["y"]))
    waypoints_total = len(waypoints)
    reached_ratio = waypoints_reached / waypoints_total if waypoints_total else 0.0
    tracking_complete = (
        waypoints_reached == waypoints_total
        and final_position_error <= params.waypoint_tolerance
        and not timeout
        and not stuck_detected
    )
    if not tracking_complete and "incomplete_waypoint_tracking" not in failure_reasons:
        failure_reasons.append("incomplete_waypoint_tracking")
    if blocked_candidate_selected:
        failure_reasons.append("blocked_candidate_selected")
    if forbidden_transition_used:
        failure_reasons.append("forbidden_transition_used")
    if stable_map_info.get("mode") == "stable_map_footprint" and collision_count > 0:
        failure_reasons.append("collision_detected_in_lightweight_replay")
    if stable_map_info.get("mode") == "stable_map_footprint" and invalid_cell_count > 0:
        failure_reasons.append("invalid_or_unknown_cells_in_lightweight_replay")

    hard_failure = (
        not tracking_complete
        or timeout
        or stuck_detected
        or blocked_candidate_selected
        or forbidden_transition_used
    )
    guard_failure = collision_count > 0 or invalid_cell_count > 0 or bool(
        final_yaw_error is not None and final_yaw_error > params.yaw_tolerance
    )
    success = not hard_failure and not guard_failure
    classification = "success" if success else ("failed" if hard_failure else "partial")

    map_sources = [
        source for source in stable_map_info.get("map_sources", [])
        if source.get("floor_id") in {str(wp.get("floor_id")) for wp in waypoints}
    ]
    report = {
        "schema_name": REPORT_SCHEMA_NAME,
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "query_id": query_id,
        "query_type": query_type,
        "success": success,
        "classification": classification,
        "failure_reason": sorted(set(failure_reasons)) if failure_reasons else [],
        "route_result_path": route_result_path.as_posix() if route_result_path else None,
        "pid_input_path": pid_path.as_posix(),
        "z_aware_input_path": z_aware_path.as_posix() if z_aware_path else None,
        "floor_sequence": sequences["floor_sequence"],
        "room_sequence": sequences["room_sequence"],
        "connector_sequence": sequences["connector_sequence"],
        "selected_object_approach": selected,
        "selected_generated_ring_002": selected_candidate_id == CURRENT_OBJECT_APPROACH_ID,
        "blocked_candidate_selected": blocked_candidate_selected,
        "forbidden_transition_used": forbidden_transition_used,
        "transition_edges": sorted(set(edges)),
        "trajectory_point_count": len(trajectory),
        "simulated_duration_sec": round(state.t, 3),
        "path_length_executed": round(path_length_executed, 6),
        "planned_path_length": planned_length,
        "planned_path_length_source": "runtime_waypoints_same_floor_excluding_connector_handoffs",
        "route_result_metric_path_length": pid_input.get("metric_path_length"),
        "final_position_error": round(final_position_error, 6),
        "final_yaw_error_if_available": round(final_yaw_error, 6) if final_yaw_error is not None else None,
        "waypoints_total": waypoints_total,
        "waypoints_reached": waypoints_reached,
        "waypoints_reached_ratio": round(reached_ratio, 6),
        "max_tracking_error": round(max(tracking_errors), 6) if tracking_errors else None,
        "mean_tracking_error": round(sum(tracking_errors) / len(tracking_errors), 6) if tracking_errors else None,
        "collision_check_mode": stable_map_info.get("mode"),
        "map_source": map_sources,
        "map_resolution": stable_map_info.get("map_resolution"),
        "collision_count": collision_count,
        "collision_free_in_lightweight_replay": collision_count == 0,
        "invalid_cell_count": invalid_cell_count,
        "min_clearance_if_available": min(min_clearance_values) if min_clearance_values else None,
        "stuck_detected": stuck_detected,
        "timeout": timeout,
        "connector_handoff_count": len(pid_input.get("connector_handoffs") or []),
        "floor_transition_event_count": len(handoff_events),
        "connector_handoff_events": handoff_events,
        "segment_reports": list(segment_reports.values()),
        "pid_parameters": params_to_dict(params),
        "claim_boundary": {
            "no_nav2_dependency": True,
            "no_amcl_dependency": True,
            "no_map_server_runtime": True,
            "no_gazebo_runtime": True,
            "no_rviz_live_runtime": True,
            "no_real_robot_claim": True,
            "no_physical_stair_climbing_claim": True,
            "no_collision_free_guarantee": True,
        },
        "trajectory_csv": trajectory_csv_path.as_posix(),
        "trajectory_svg": trajectory_svg_path.as_posix(),
        "report_json": report_json_path.as_posix(),
        "report_md": report_md_path.as_posix(),
    }
    return report, trajectory


def base_failure_report(
    *,
    query_id: str,
    query_type: Any,
    pid_path: Path,
    route_result_path: Optional[Path],
    z_aware_path: Optional[Path],
    selected: Optional[dict[str, Any]],
    sequences: dict[str, Any],
    reason: str,
    params: ReplayParams,
    stable_map_info: dict[str, Any],
    report_json_path: Path,
    report_md_path: Path,
    trajectory_csv_path: Path,
    trajectory_svg_path: Path,
) -> dict[str, Any]:
    return {
        "schema_name": REPORT_SCHEMA_NAME,
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "query_id": query_id,
        "query_type": query_type,
        "success": False,
        "classification": "failed",
        "failure_reason": [reason],
        "route_result_path": route_result_path.as_posix() if route_result_path else None,
        "pid_input_path": pid_path.as_posix(),
        "z_aware_input_path": z_aware_path.as_posix() if z_aware_path else None,
        "floor_sequence": sequences.get("floor_sequence") or [],
        "room_sequence": sequences.get("room_sequence") or [],
        "connector_sequence": sequences.get("connector_sequence") or [],
        "selected_object_approach": selected,
        "selected_generated_ring_002": False,
        "blocked_candidate_selected": False,
        "forbidden_transition_used": False,
        "transition_edges": [],
        "trajectory_point_count": 0,
        "simulated_duration_sec": 0.0,
        "path_length_executed": 0.0,
        "planned_path_length": 0.0,
        "route_result_metric_path_length": None,
        "final_position_error": None,
        "final_yaw_error_if_available": None,
        "waypoints_total": 0,
        "waypoints_reached": 0,
        "waypoints_reached_ratio": 0.0,
        "max_tracking_error": None,
        "mean_tracking_error": None,
        "collision_check_mode": stable_map_info.get("mode"),
        "map_source": [],
        "map_resolution": stable_map_info.get("map_resolution"),
        "collision_count": 0,
        "collision_free_in_lightweight_replay": False,
        "invalid_cell_count": 0,
        "min_clearance_if_available": None,
        "stuck_detected": False,
        "timeout": False,
        "connector_handoff_count": 0,
        "floor_transition_event_count": 0,
        "connector_handoff_events": [],
        "segment_reports": [],
        "pid_parameters": params_to_dict(params),
        "claim_boundary": {
            "no_nav2_dependency": True,
            "no_amcl_dependency": True,
            "no_map_server_runtime": True,
            "no_real_robot_claim": True,
            "no_physical_stair_climbing_claim": True,
            "no_collision_free_guarantee": True,
        },
        "trajectory_csv": trajectory_csv_path.as_posix(),
        "trajectory_svg": trajectory_svg_path.as_posix(),
        "report_json": report_json_path.as_posix(),
        "report_md": report_md_path.as_posix(),
    }


def state_to_record(state: State) -> dict[str, Any]:
    return {
        "x": round(state.x, 6),
        "y": round(state.y, 6),
        "yaw": round(state.yaw, 6),
        "floor_id": state.floor_id,
        "z": round(state.z, 6),
        "segment_id": state.segment_id,
        "waypoint_index": state.waypoint_index,
        "t": round(state.t, 3),
    }


def params_to_dict(params: ReplayParams) -> dict[str, Any]:
    return {
        "dt": params.dt,
        "max_linear_velocity": params.max_linear_velocity,
        "max_angular_velocity": params.max_angular_velocity,
        "linear_gain": params.linear_gain,
        "angular_gain": params.angular_gain,
        "waypoint_tolerance": params.waypoint_tolerance,
        "yaw_tolerance": params.yaw_tolerance,
        "timeout_sec": params.timeout_sec,
        "stuck_window_sec": params.stuck_window_sec,
        "stuck_progress_epsilon": params.stuck_progress_epsilon,
        "robot_radius": params.robot_radius,
        "floor_z_map": params.floor_z_map,
    }


def write_trajectory_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "t",
        "x",
        "y",
        "yaw",
        "floor_id",
        "z",
        "segment_id",
        "waypoint_index",
        "event",
        "v",
        "w",
        "tracking_error",
        "cell_state",
        "collision",
        "invalid_cell",
        "min_clearance_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def svg_polyline(points: list[tuple[float, float]], transform: Any, color: str, width: float, dash: str = "") -> str:
    if not points:
        return ""
    encoded = " ".join(f"{transform(x, y)[0]:.2f},{transform(x, y)[1]:.2f}" for x, y in points)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polyline points="{encoded}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round" stroke-linejoin="round"{dash_attr}/>'


def write_trajectory_svg(
    path: Path,
    query_id: str,
    waypoints: list[dict[str, Any]],
    trajectory: list[dict[str, Any]],
    report: dict[str, Any],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    floors = floor_sequence_from_waypoints(waypoints) or sorted({str(row.get("floor_id")) for row in trajectory})
    panel_w = 900
    panel_h = 320
    margin = 34
    sections: list[str] = []
    for floor_index, floor_id in enumerate(floors):
        planned = [(float(wp["x"]), float(wp["y"])) for wp in waypoints if wp.get("floor_id") == floor_id]
        executed = [(float(row["x"]), float(row["y"])) for row in trajectory if row.get("floor_id") == floor_id]
        all_points = planned + executed
        if not all_points:
            continue
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 0.5)
        span_y = max(max_y - min_y, 0.5)
        scale = min((panel_w - margin * 2) / span_x, (panel_h - margin * 2) / span_y)
        y_offset = floor_index * panel_h

        def transform(x: float, y: float) -> tuple[float, float]:
            sx = margin + (x - min_x) * scale
            sy = y_offset + panel_h - margin - (y - min_y) * scale
            return sx, sy

        sections.append(
            f'<rect x="0" y="{y_offset}" width="{panel_w}" height="{panel_h}" fill="#ffffff"/>'
        )
        sections.append(
            f'<text x="18" y="{y_offset + 26}" font-family="Arial, sans-serif" font-size="16" fill="#1f2937">{html.escape(floor_id)}</text>'
        )
        sections.append(svg_polyline(planned, transform, "#9ca3af", 3.0, "7 5"))
        sections.append(svg_polyline(executed, transform, "#2563eb", 2.4))
        if planned:
            sx, sy = transform(*planned[0])
            gx, gy = transform(*planned[-1])
            sections.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5" fill="#16a34a"/>')
            sections.append(f'<circle cx="{gx:.2f}" cy="{gy:.2f}" r="5" fill="#dc2626"/>')
        for row in trajectory:
            if row.get("floor_id") == floor_id and row.get("event") == "connector_handoff":
                hx, hy = transform(float(row["x"]), float(row["y"]))
                sections.append(f'<circle cx="{hx:.2f}" cy="{hy:.2f}" r="6" fill="#f59e0b" stroke="#92400e" stroke-width="1.5"/>')
    status = report.get("classification")
    height = max(panel_h, panel_h * max(1, len(floors)))
    svg = "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{panel_w}" height="{height}" viewBox="0 0 {panel_w} {height}">',
            f'<title>{html.escape(query_id)} PID replay trajectory</title>',
            f'<desc>Planned path is gray dashed, executed replay is blue, start is green, final target is red, connector handoff is orange. Status: {html.escape(str(status))}.</desc>',
            *sections,
            "</svg>",
        ]
    )
    path.write_text(svg + "\n", encoding="utf-8")


def render_report_md(report: dict[str, Any]) -> str:
    lines = [
        f"# PID Replay Execution Report: {report['query_id']}",
        "",
        f"- classification: `{report['classification']}`",
        f"- success: `{report['success']}`",
        f"- failure_reason: `{report['failure_reason']}`",
        f"- PID input: `{report['pid_input_path']}`",
        f"- RouteResult: `{report['route_result_path']}`",
        f"- floor_sequence: `{report['floor_sequence']}`",
        f"- room_sequence: `{report['room_sequence']}`",
        f"- selected_object_approach: `{(report.get('selected_object_approach') or {}).get('candidate_id')}`",
        f"- generated_ring_037_selected: `{report['blocked_candidate_selected']}`",
        f"- vt_1_centerline_e003_transition: `{report['forbidden_transition_used']}`",
        f"- waypoint reach: `{report['waypoints_reached']}/{report['waypoints_total']}` ({report['waypoints_reached_ratio']})",
        f"- final_position_error: `{report['final_position_error']}`",
        f"- final_yaw_error_if_available: `{report['final_yaw_error_if_available']}`",
        f"- max_tracking_error: `{report['max_tracking_error']}`",
        f"- mean_tracking_error: `{report['mean_tracking_error']}`",
        f"- collision_check_mode: `{report['collision_check_mode']}`",
        f"- collision_count: `{report['collision_count']}`",
        f"- invalid_cell_count: `{report['invalid_cell_count']}`",
        f"- min_clearance_if_available: `{report['min_clearance_if_available']}`",
        f"- stuck_detected: `{report['stuck_detected']}`",
        f"- timeout: `{report['timeout']}`",
        f"- connector_handoff_events: `{report['floor_transition_event_count']}`",
        f"- trajectory_csv: `{report['trajectory_csv']}`",
        f"- trajectory_svg: `{report['trajectory_svg']}`",
        "",
    ]
    if report["collision_count"] == 0:
        lines.append("This route is collision-free in lightweight replay only; this is not a global collision-free guarantee.")
    else:
        lines.append("This route recorded collision samples in lightweight replay; the failure is intentionally reported.")
    lines.extend(
        [
            "",
            "Cross-floor connector handling, when present, is semantic handoff only and not physical stair climbing.",
            "",
            "## Segment Reports",
            "",
            "| segment_id | floor | waypoints | status | collisions | invalid cells |",
            "|---|---|---:|---|---:|---:|",
        ]
    )
    for segment in report.get("segment_reports") or []:
        lines.append(
            f"| `{segment.get('segment_id')}` | `{segment.get('floor_id')}` | "
            f"{segment.get('waypoints_reached')}/{segment.get('waypoints_total')} | "
            f"`{segment.get('status')}` | {segment.get('collision_count')} | {segment.get('invalid_cell_count')} |"
        )
    lines.append("")
    return "\n".join(lines)


def aggregate_reports(reports: list[dict[str, Any]], stable_map_info: dict[str, Any], params: ReplayParams) -> dict[str, Any]:
    query_count = len(reports)
    success_count = sum(1 for report in reports if report.get("classification") == "success")
    partial_count = sum(1 for report in reports if report.get("classification") == "partial")
    failure_count = sum(1 for report in reports if report.get("classification") == "failed")
    final_errors = [
        float(report["final_position_error"])
        for report in reports
        if report.get("final_position_error") is not None
    ]
    reach_ratios = [
        float(report["waypoints_reached_ratio"])
        for report in reports
        if report.get("waypoints_reached_ratio") is not None
    ]
    return {
        "schema_name": SCHEMA_NAME,
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "classification": "pid_runtime_replay_completed",
        "ok": query_count > 0,
        "query_count": query_count,
        "success_count": success_count,
        "failure_count": failure_count,
        "partial_count": partial_count,
        "collision_free_in_sim_count": sum(1 for report in reports if report.get("collision_count") == 0),
        "no_invalid_cell_count": sum(1 for report in reports if report.get("invalid_cell_count") == 0),
        "generated_ring_037_selected_count": sum(1 for report in reports if report.get("blocked_candidate_selected")),
        "vt_1_centerline_e003_transition_count": sum(1 for report in reports if report.get("forbidden_transition_used")),
        "average_final_error": round(sum(final_errors) / len(final_errors), 6) if final_errors else None,
        "average_waypoint_reached_ratio": round(sum(reach_ratios) / len(reach_ratios), 6) if reach_ratios else None,
        "collision_check_mode": stable_map_info.get("mode"),
        "map_sources": stable_map_info.get("map_sources") or [],
        "map_resolution": stable_map_info.get("map_resolution"),
        "pid_parameters": params_to_dict(params),
        "reports": reports,
        "claim_boundary": {
            "no_nav2_dependency": True,
            "no_amcl_dependency": True,
            "no_map_server_runtime": True,
            "no_gazebo_runtime": True,
            "no_rviz_live_runtime": True,
            "no_real_robot_claim": True,
            "no_physical_stair_climbing_claim": True,
            "no_collision_free_guarantee": True,
        },
    }


def render_index_md(summary: dict[str, Any]) -> str:
    lines = [
        f"# {summary.get('project_name')} PID Runtime Replay",
        "",
        f"- generated_utc: `{summary['generated_utc']}`",
        f"- query_count: `{summary['query_count']}`",
        f"- success_count: `{summary['success_count']}`",
        f"- partial_count: `{summary['partial_count']}`",
        f"- failure_count: `{summary['failure_count']}`",
        f"- average_final_error: `{summary['average_final_error']}`",
        f"- average_waypoint_reached_ratio: `{summary['average_waypoint_reached_ratio']}`",
        f"- collision_check_mode: `{summary['collision_check_mode']}`",
        f"- generated_ring_037_selected_count: `{summary['generated_ring_037_selected_count']}`",
        f"- vt_1_centerline_e003_transition_count: `{summary['vt_1_centerline_e003_transition_count']}`",
        "",
        "This is lightweight executable replay over RouteResult-derived PID inputs. It is not Nav2, AMCL, `map_server`, Gazebo, RViz live execution, real robot deployment, physical stair climbing, or a global collision-free guarantee.",
        "",
        "| query_id | class | success | final error | reach ratio | collisions | invalid cells | stuck | timeout | report | svg |",
        "|---|---|---|---:|---:|---:|---:|---|---|---|---|",
    ]
    for report in summary.get("reports") or []:
        lines.append(
            f"| `{report['query_id']}` | `{report['classification']}` | `{report['success']}` | "
            f"{report['final_position_error']} | {report['waypoints_reached_ratio']} | "
            f"{report['collision_count']} | {report['invalid_cell_count']} | "
            f"{report['stuck_detected']} | {report['timeout']} | "
            f"[md]({Path(report['report_md']).name if False else report['report_md']}) | "
            f"[svg]({report['trajectory_svg']}) |"
        )
    lines.append("")
    return "\n".join(lines)


def render_index_html(summary: dict[str, Any]) -> str:
    rows: list[str] = []
    for report in summary.get("reports") or []:
        rows.append(
            "<tr>"
            f"<td>{html.escape(str(report['query_id']))}</td>"
            f"<td>{html.escape(str(report['classification']))}</td>"
            f"<td>{html.escape(str(report['success']))}</td>"
            f"<td>{html.escape(str(report['final_position_error']))}</td>"
            f"<td>{html.escape(str(report['waypoints_reached_ratio']))}</td>"
            f"<td>{html.escape(str(report['collision_count']))}</td>"
            f"<td>{html.escape(str(report['invalid_cell_count']))}</td>"
            f"<td><a href=\"{html.escape(Path(report['report_md']).relative_to(Path(report['report_md']).parents[1]).as_posix())}\">report</a></td>"
            f"<td><a href=\"{html.escape(Path(report['trajectory_svg']).relative_to(Path(report['trajectory_svg']).parents[1]).as_posix())}\">svg</a></td>"
            "</tr>"
        )
    return "\n".join(
        [
            "<!doctype html>",
            "<html lang=\"en\">",
            "<head>",
            "<meta charset=\"utf-8\">",
            f"<title>{html.escape(str(summary.get('project_name')))} PID Runtime Replay</title>",
            "<style>body{font-family:Arial,sans-serif;margin:24px;color:#172033}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #d1d5db;padding:7px;text-align:left}th{background:#f3f4f6}.note{max-width:980px;line-height:1.45}</style>",
            "</head>",
            "<body>",
            f"<h1>{html.escape(str(summary.get('project_name')))} PID Runtime Replay</h1>",
            f"<p class=\"note\">Query count: {summary['query_count']}; success: {summary['success_count']}; partial: {summary['partial_count']}; failed: {summary['failure_count']}; collision check mode: {html.escape(str(summary['collision_check_mode']))}.</p>",
            "<p class=\"note\">This is lightweight executable replay over RouteResult-derived PID inputs. It is not Nav2, AMCL, map_server, Gazebo, RViz live execution, real robot deployment, physical stair climbing, or a global collision-free guarantee.</p>",
            "<table>",
            "<thead><tr><th>query</th><th>class</th><th>success</th><th>final error</th><th>reach ratio</th><th>collisions</th><th>invalid cells</th><th>report</th><th>trajectory</th></tr></thead>",
            "<tbody>",
            *rows,
            "</tbody>",
            "</table>",
            "</body>",
            "</html>",
        ]
    )


def validate_pid_payload(path: Path, payload: dict[str, Any]) -> None:
    if payload.get("schema_name") != PID_SCHEMA_NAME:
        raise ValueError(f"{path} is not a {PID_SCHEMA_NAME}: {payload.get('schema_name')!r}")
    policy = payload.get("runtime_policy") or {}
    if payload.get("requires_nav2") is not False or policy.get("requires_nav2") is not False:
        raise ValueError(f"{path} declares requires_nav2")
    if payload.get("requires_amcl") is not False or policy.get("requires_amcl") is not False:
        raise ValueError(f"{path} declares requires_amcl")


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "reports").mkdir(parents=True, exist_ok=True)
    (output_dir / "trajectories").mkdir(parents=True, exist_ok=True)

    pid_paths = sorted(args.pid_inputs_dir.glob("*_pid_runtime_input.json"))
    if not pid_paths:
        raise SystemExit(f"no PID runtime inputs found under {args.pid_inputs_dir}")

    params = ReplayParams(
        dt=float(args.dt),
        max_linear_velocity=float(args.max_linear_velocity),
        max_angular_velocity=float(args.max_angular_velocity),
        linear_gain=float(args.linear_gain),
        angular_gain=float(args.angular_gain),
        waypoint_tolerance=float(args.waypoint_tolerance),
        yaw_tolerance=float(args.yaw_tolerance),
        timeout_sec=float(args.timeout_sec),
        stuck_window_sec=float(args.stuck_window_sec),
        stuck_progress_epsilon=float(args.stuck_progress_epsilon),
        robot_radius=float(args.robot_radius),
        floor_z_map=parse_floor_z_map(args.floor_z_map),
        title=str(args.title),
    )

    loaded: list[tuple[Path, dict[str, Any], Optional[dict[str, Any]], Optional[Path], Optional[Path]]] = []
    route_results: list[Optional[dict[str, Any]]] = []
    pid_payloads: list[dict[str, Any]] = []
    for pid_path in pid_paths:
        payload = read_json(pid_path)
        validate_pid_payload(pid_path, payload)
        query_id = str((payload.get("identity") or {}).get("query_id") or "")
        route_result, route_result_path = load_route_result(payload, pid_path, args.route_results_dir)
        _, z_aware_path = load_z_aware_input(query_id, args.z_aware_inputs_dir)
        loaded.append((pid_path, payload, route_result, route_result_path, z_aware_path))
        route_results.append(route_result)
        pid_payloads.append(payload)

    stable_maps, stable_map_info = load_stable_maps(discover_canonical_roots(pid_payloads, route_results))

    reports: list[dict[str, Any]] = []
    for pid_path, payload, route_result, route_result_path, z_aware_path in loaded:
        report, trajectory = simulate_query(
            payload,
            pid_path,
            route_result,
            route_result_path,
            z_aware_path,
            stable_maps,
            stable_map_info,
            params,
            output_dir,
        )
        query_id = report["query_id"]
        waypoints = sanitize_waypoints(payload)
        write_trajectory_csv(Path(report["trajectory_csv"]), trajectory)
        write_trajectory_svg(Path(report["trajectory_svg"]), query_id, waypoints, trajectory, report)
        write_json(Path(report["report_json"]), report)
        Path(report["report_md"]).write_text(render_report_md(report), encoding="utf-8")
        reports.append(report)

    summary = aggregate_reports(reports, stable_map_info, params)
    summary["title"] = params.title
    summary["output_dir"] = output_dir.as_posix()
    write_json(output_dir / "summary.json", summary)
    (output_dir / "index.md").write_text(render_index_md(summary), encoding="utf-8")
    (output_dir / "index.html").write_text(render_index_html(summary), encoding="utf-8")
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "query_count": summary["query_count"],
                "success_count": summary["success_count"],
                "partial_count": summary["partial_count"],
                "failure_count": summary["failure_count"],
                "collision_check_mode": summary["collision_check_mode"],
                "generated_ring_037_selected_count": summary["generated_ring_037_selected_count"],
                "vt_1_centerline_e003_transition_count": summary["vt_1_centerline_e003_transition_count"],
                "output_dir": output_dir.as_posix(),
            },
            indent=2,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-inputs-dir", type=Path, required=True)
    parser.add_argument("--route-results-dir", type=Path, default=None)
    parser.add_argument("--z-aware-inputs-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--max-linear-velocity", type=float, default=0.25)
    parser.add_argument("--max-angular-velocity", type=float, default=0.8)
    parser.add_argument("--linear-gain", type=float, default=0.8)
    parser.add_argument("--angular-gain", type=float, default=1.5)
    parser.add_argument("--waypoint-tolerance", type=float, default=0.12)
    parser.add_argument("--yaw-tolerance", type=float, default=0.25)
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--stuck-window-sec", type=float, default=8.0)
    parser.add_argument("--stuck-progress-epsilon", type=float, default=0.02)
    parser.add_argument("--robot-radius", type=float, default=0.18)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--title", default="RSLG-SLAM PID Runtime Replay")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
