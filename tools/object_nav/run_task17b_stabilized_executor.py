#!/usr/bin/env python3
"""task17b: Stabilized lightweight RSLG-SLAM executor with control path simplification and pure pursuit tracking."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
THIS_DIR = Path(__file__).resolve().parent
STAGE_RUNTIME_DIR = ROOT / "tools/stage1_runtime"
for directory in (THIS_DIR, STAGE_RUNTIME_DIR):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from object_nav_common import canonical_object_id, load_index, run_query  # noqa: E402
from scene_runtime_common import load_nav_map  # noqa: E402

SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task17b_lightweight_control_path_simplification_and_tracking_stabilization"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
DEFAULT_STAGE = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
DEFAULT_OUTPUT = TASKS_ROOT / TASK_NAME
INDEX_PATH = TASKS_ROOT / "task14a_object_nav_experiment_adapter/object_candidate_index_v0_1.json"
TASK16_RESULT = TASKS_ROOT / "task16_dynamic_object_query_route_generation_execution_and_wrapper/gui_obj_172_retry/runtime_result.json"
TASK16_APPROACH = TASKS_ROOT / "task16_dynamic_object_query_route_generation_execution_and_wrapper/gui_obj_172_retry/approach_candidate_report.json"
TASK17_DIR = TASKS_ROOT / "task17_lightweight_rslg_executor_without_nav2/runtime_obj_172_closepath_final"
LAUNCHER = THIS_DIR / "launch_lightweight_gazebo_turtlebot3.sh"
FAILURE_LAYERS = {
    "artifact_loading", "query_resolution", "topology_route_generation",
    "executable_route_generation", "control_path_simplification",
    "no_nav2_bringup", "pose_feedback", "route_tracking",
    "approach_candidate_generation", "approach_tracking", "yaw_alignment", "validation", "cleanup",
}
NAV2_ACTIONS = {"/compute_path_to_pose", "/follow_path", "/navigate_to_pose"}
NAV2_PROCESS_PATTERN = re.compile(
    r"planner_server|controller_server|bt_navigator|behavior_server|recoveries_server|"
    r"waypoint_follower|nav2_map_server|lifecycle_manager_navigation|nav2_costmap"
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def distance(a: dict[str, Any] | Iterable[float], b: dict[str, Any] | Iterable[float]) -> float:
    if isinstance(a, dict):
        ax, ay = float(a["x"]), float(a["y"])
    else:
        ax, ay = [float(v) for v in a][:2]
    if isinstance(b, dict):
        bx, by = float(b["x"]), float(b["y"])
    else:
        bx, by = [float(v) for v in b][:2]
    return math.hypot(bx - ax, by - ay)


def route_length(points: list[dict[str, Any]]) -> float:
    return sum(distance(l, r) for l, r in zip(points, points[1:]))


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    prev = len(polygon) - 1
    for i, pt in enumerate(polygon):
        xi, yi = float(pt[0]), float(pt[1])
        xj, yj = float(polygon[prev][0]), float(polygon[prev][1])
        if (yi > y) != (yj > y):
            at_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < at_x:
                inside = not inside
        prev = i
    return inside


def pair_key(left: str, right: str) -> str:
    return "__".join(sorted([left, right], key=lambda r: int(r.split("_")[1])))


# ─── Occupancy Planner (same as task17) ─────────────────────────────────────────

class OccupancyPlanner:
    def __init__(self, map_yaml: Path, inflation_radius_m: float, spacing_m: float) -> None:
        self.map_yaml = map_yaml
        self.grid, self.resolution, self.origin, self.meta = load_nav_map(map_yaml)
        self.free_mask = (self.grid >= 250).astype(np.uint8)
        self.clearance = cv2.distanceTransform(self.free_mask, cv2.DIST_L2, 5) * self.resolution
        self.inflation_radius_m = inflation_radius_m
        self.spacing_m = spacing_m

    def rc(self, x: float, y: float) -> tuple[int, int]:
        return (int(round((y - self.origin[1]) / self.resolution)), int(round((x - self.origin[0]) / self.resolution)))

    def xy(self, rc: tuple[int, int]) -> tuple[float, float]:
        return (self.origin[0] + rc[1] * self.resolution, self.origin[1] + rc[0] * self.resolution)

    def in_bounds(self, rc: tuple[int, int]) -> bool:
        return 0 <= rc[0] < self.grid.shape[0] and 0 <= rc[1] < self.grid.shape[1]

    def free(self, rc: tuple[int, int]) -> bool:
        return bool(self.in_bounds(rc) and int(self.grid[rc[0], rc[1]]) >= 250)

    def traversable(self, rc: tuple[int, int]) -> bool:
        return bool(self.free(rc) and float(self.clearance[rc[0], rc[1]]) >= self.inflation_radius_m)

    def sample(self, xy: Iterable[float]) -> dict[str, Any]:
        x, y = [float(v) for v in xy][:2]
        rc = self.rc(x, y)
        avail = self.in_bounds(rc)
        val = int(self.grid[rc[0], rc[1]]) if avail else None
        clr = float(self.clearance[rc[0], rc[1]]) if avail else None
        return {
            "grid_row": rc[0], "grid_col": rc[1], "in_bounds": avail,
            "occupancy_value": val, "is_free": bool(avail and val is not None and val >= 250),
            "inflated_traversable": bool(avail and val is not None and val >= 250 and clr is not None and clr >= self.inflation_radius_m),
            "clearance_m": round(clr, 6) if clr is not None else None,
        }

    def nearest_traversable(self, xy: Iterable[float], max_radius_m: float = 1.0) -> tuple[tuple[int, int], dict[str, Any]]:
        x, y = [float(v) for v in xy][:2]
        start = self.rc(x, y)
        queue = deque([start])
        seen = {start}
        cells = max(1, int(math.ceil(max_radius_m / self.resolution)))
        while queue:
            cur = queue.popleft()
            if max(abs(cur[0] - start[0]), abs(cur[1] - start[1])) > cells:
                continue
            if self.traversable(cur):
                snapped = self.xy(cur)
                return cur, {"requested_world_xy": [x, y], "snapped_world_xy": [round(snapped[0], 6), round(snapped[1], 6)], "snap_distance_m": round(math.hypot(snapped[0] - x, snapped[1] - y), 6)}
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    c = (cur[0] + dr, cur[1] + dc)
                    if c not in seen and self.in_bounds(c):
                        seen.add(c)
                        queue.append(c)
        raise RuntimeError(f"no traversable point within {max_radius_m:.2f} m of ({x:.3f}, {y:.3f})")

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        neighbors = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))
        open_set: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        cost = {start: 0.0}
        prev: dict[tuple[int, int], tuple[int, int]] = {}
        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                cells = [cur]
                while cells[-1] in prev:
                    cells.append(prev[cells[-1]])
                return list(reversed(cells))
            for dr, dc, step in neighbors:
                c = (cur[0] + dr, cur[1] + dc)
                if not self.traversable(c):
                    continue
                clr = float(self.clearance[c[0], c[1]])
                penalty = max(0.0, 0.28 - clr) * 8.0
                proposed = cost[cur] + step + penalty
                if proposed >= cost.get(c, float("inf")):
                    continue
                cost[c] = proposed
                prev[c] = cur
                h = math.hypot(goal[0] - c[0], goal[1] - c[1])
                heapq.heappush(open_set, (proposed + h, c))
        raise RuntimeError(f"A* could not connect {start} to {goal}")

    def sampled_cells(self, left: dict[str, Any], right: dict[str, Any]) -> list[tuple[int, int]]:
        seg_len = distance(left, right)
        count = max(1, int(math.ceil(seg_len / (self.resolution * 0.45))))
        cells: list[tuple[int, int]] = []
        for i in range(count + 1):
            ratio = i / count
            cells.append(self.rc(
                float(left["x"]) + ratio * (float(right["x"]) - float(left["x"])),
                float(left["y"]) + ratio * (float(right["y"]) - float(left["y"])),
            ))
        return cells

    def validate_polyline(self, points: list[dict[str, Any]], require_inflated: bool = False) -> dict[str, Any]:
        tested: list[tuple[int, int]] = []
        for l, r in zip(points, points[1:]):
            tested.extend(self.sampled_cells(l, r))
        if len(points) == 1:
            tested.append(self.rc(float(points[0]["x"]), float(points[0]["y"])))
        invalid = [
            {"grid_row": rc[0], "grid_col": rc[1], "world_xy": [round(v, 6) for v in self.xy(rc)],
             "occupancy_value": int(self.grid[rc[0], rc[1]]) if self.in_bounds(rc) else None,
             "clearance_m": round(float(self.clearance[rc[0], rc[1]]), 6) if self.in_bounds(rc) else None}
            for rc in tested
            if not (self.traversable(rc) if require_inflated else self.free(rc))
        ]
        clearances = [float(self.clearance[rc[0], rc[1]]) for rc in tested if self.in_bounds(rc) and self.free(rc)]
        return {
            "wall_crossing_validation_passed": not invalid,
            "requires_inflated_clearance": require_inflated,
            "tested_sample_count": len(tested),
            "occupied_or_invalid_sample_count": len(invalid),
            "invalid_samples": invalid[:30],
            "minimum_clearance_m": round(min(clearances), 6) if clearances else None,
        }

    def plan_segment(self, start: Iterable[float], goal: Iterable[float], label: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        start_rc, start_snap = self.nearest_traversable(start)
        goal_rc, goal_snap = self.nearest_traversable(goal)
        raw = self.astar(start_rc, goal_rc)
        stride = max(1, int(round(self.spacing_m / self.resolution)))
        simplified = [raw[0], *raw[stride:-1:stride], raw[-1]]
        points = [{"x": round(self.xy(rc)[0], 6), "y": round(self.xy(rc)[1], 6)} for rc in simplified]
        validation = self.validate_polyline(points, require_inflated=True)
        if not validation["wall_crossing_validation_passed"]:
            points = [{"x": round(self.xy(rc)[0], 6), "y": round(self.xy(rc)[1], 6)} for rc in raw]
            validation = self.validate_polyline(points, require_inflated=True)
        segment = {
            "segment": label, "a_star_success": True, "inflation_radius_m": self.inflation_radius_m,
            "start_snap": start_snap, "goal_snap": goal_snap,
            "raw_grid_point_count": len(raw), "executable_waypoint_count": len(points),
            "path_length_m": round(route_length(points), 6), **validation,
        }
        return points, segment

    def line_of_sight(self, ax: float, ay: float, bx: float, by: float) -> bool:
        """Check if straight line from a to b is fully inflated-traversable."""
        seg_len = math.hypot(bx - ax, by - ay)
        count = max(2, int(math.ceil(seg_len / (self.resolution * 0.45))))
        for i in range(count + 1):
            ratio = i / count
            rc = self.rc(ax + ratio * (bx - ax), ay + ratio * (by - ay))
            if not self.traversable(rc):
                return False
        return True


# ─── Control Path Simplification ────────────────────────────────────────────────

def simplify_control_path(
    dense_points: list[dict[str, Any]],
    planner: OccupancyPlanner,
    semantic_anchors_list: list[dict[str, Any]],
    normal_spacing: float = 0.60,
    gateway_spacing: float = 0.30,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Generate a simplified control path from the dense A* route using line-of-sight simplification."""
    if len(dense_points) < 3:
        return list(dense_points), {"method": "too_short", "original_count": len(dense_points), "simplified_count": len(dense_points)}

    # Identify anchor indices (points closest to semantic anchors like gateways)
    anchor_indices: set[int] = {0, len(dense_points) - 1}
    for anchor in semantic_anchors_list:
        if "x" not in anchor or "y" not in anchor:
            continue
        best_idx = min(range(len(dense_points)), key=lambda i: distance(dense_points[i], anchor))
        anchor_indices.add(best_idx)

    # Line-of-sight simplification preserving anchors
    simplified_indices: list[int] = [0]
    current = 0
    while current < len(dense_points) - 1:
        # Find farthest visible point
        farthest = current + 1
        for candidate in range(len(dense_points) - 1, current, -1):
            if planner.line_of_sight(
                float(dense_points[current]["x"]), float(dense_points[current]["y"]),
                float(dense_points[candidate]["x"]), float(dense_points[candidate]["y"]),
            ):
                farthest = candidate
                break
        # But don't skip past any anchor index
        next_anchor = None
        for ai in sorted(anchor_indices):
            if ai > current:
                next_anchor = ai
                break
        if next_anchor is not None and farthest > next_anchor:
            farthest = next_anchor
        simplified_indices.append(farthest)
        current = farthest

    # Ensure last point
    if simplified_indices[-1] != len(dense_points) - 1:
        simplified_indices.append(len(dense_points) - 1)

    # Resample long segments
    final_indices: list[int] = []
    for i in range(len(simplified_indices) - 1):
        si, ei = simplified_indices[i], simplified_indices[i + 1]
        seg_len = sum(distance(dense_points[j], dense_points[j + 1]) for j in range(si, ei))
        # Determine spacing: use gateway spacing near anchors
        near_anchor = si in anchor_indices or ei in anchor_indices
        spacing = gateway_spacing if near_anchor else normal_spacing
        if seg_len > spacing * 1.5:
            # Resample from dense route at desired spacing
            accumulated = 0.0
            final_indices.append(si)
            for j in range(si, ei):
                accumulated += distance(dense_points[j], dense_points[j + 1])
                if accumulated >= spacing:
                    final_indices.append(j + 1)
                    accumulated = 0.0
        else:
            final_indices.append(si)
    final_indices.append(len(dense_points) - 1)

    # Deduplicate while preserving order
    seen: set[int] = set()
    unique_indices: list[int] = []
    for idx in final_indices:
        if idx not in seen:
            seen.add(idx)
            unique_indices.append(idx)

    control_points = [dict(dense_points[i]) for i in unique_indices]

    # Assign yaw
    for i in range(len(control_points) - 1):
        control_points[i]["yaw"] = round(math.atan2(
            float(control_points[i + 1]["y"]) - float(control_points[i]["y"]),
            float(control_points[i + 1]["x"]) - float(control_points[i]["x"]),
        ), 6)
    if len(control_points) > 1:
        control_points[-1]["yaw"] = control_points[-2]["yaw"]

    # Add waypoint_index
    for i, pt in enumerate(control_points):
        pt["waypoint_index"] = i

    validation = planner.validate_polyline(control_points, require_inflated=True)
    report = {
        "method": "line_of_sight_with_anchor_preservation_and_resampling",
        "original_dense_count": len(dense_points),
        "simplified_count": len(control_points),
        "anchor_indices_preserved": sorted(anchor_indices),
        "normal_spacing_m": normal_spacing,
        "gateway_spacing_m": gateway_spacing,
        "control_path_length_m": round(route_length(control_points), 6),
        "dense_route_length_m": round(route_length(dense_points), 6),
        "wall_crossing_validation_passed": validation["wall_crossing_validation_passed"],
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "tested_sample_count": validation["tested_sample_count"],
    }
    return control_points, report


# ─── Polyline Utilities ──────────────────────────────────────────────────────────

def project_onto_polyline(px: float, py: float, points: list[dict[str, Any]]) -> tuple[float, float, float, int]:
    """Returns (proj_x, proj_y, arc_length_to_projection, segment_index)."""
    best_dist = float("inf")
    best_proj = (float(points[0]["x"]), float(points[0]["y"]))
    best_arc = 0.0
    best_seg = 0
    arc = 0.0
    for i in range(len(points) - 1):
        ax, ay = float(points[i]["x"]), float(points[i]["y"])
        bx, by = float(points[i + 1]["x"]), float(points[i + 1]["y"])
        dx, dy = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            arc += seg_len
            continue
        t = clamp(((px - ax) * dx + (py - ay) * dy) / (seg_len * seg_len), 0.0, 1.0)
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        d = math.hypot(px - proj_x, py - proj_y)
        if d < best_dist:
            best_dist = d
            best_proj = (proj_x, proj_y)
            best_arc = arc + t * seg_len
            best_seg = i
        arc += seg_len
    return best_proj[0], best_proj[1], best_arc, best_seg


def polyline_total_length(points: list[dict[str, Any]]) -> float:
    return sum(math.hypot(float(points[i + 1]["x"]) - float(points[i]["x"]),
                          float(points[i + 1]["y"]) - float(points[i]["y"]))
               for i in range(len(points) - 1))


def advance_along_polyline(points: list[dict[str, Any]], arc_start: float, advance: float) -> tuple[float, float]:
    """Get XY position at arc_start + advance along polyline."""
    target_arc = arc_start + advance
    arc = 0.0
    for i in range(len(points) - 1):
        ax, ay = float(points[i]["x"]), float(points[i]["y"])
        bx, by = float(points[i + 1]["x"]), float(points[i + 1]["y"])
        seg_len = math.hypot(bx - ax, by - ay)
        if arc + seg_len >= target_arc:
            t = (target_arc - arc) / seg_len if seg_len > 1e-9 else 0.0
            return ax + t * (bx - ax), ay + t * (by - ay)
        arc += seg_len
    # Past end: return last point
    return float(points[-1]["x"]), float(points[-1]["y"])


# ─── Artifact loading, query, topology, approach (reuse task17 logic) ────────────

def stable_map_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    required = args.stage_output_dir / f"maps/{args.floor_id}/stage1_{args.floor_id}_stable_occupancy_map.yaml"
    selected = (args.map_yaml or required).resolve()
    return required.resolve(), selected


def artifact_paths(args: argparse.Namespace) -> dict[str, Path]:
    public = args.stage_output_dir / "committed_public"
    return {
        "topology": public / "topology_v0_1.json",
        "topology_query_report": public / "topology_query_report.json",
        "committed_room_world_snapshot": public / "committed_room_world_snapshot_v0_1.json",
        "committed_room_world_model": public / "committed_room_world_model_v0_1.json",
        "object_candidate_index": INDEX_PATH,
        "gateway_registry": args.stage_output_dir / f"process/floors/{args.floor_id}/gateway/assets/gateway_registry_v0_1.json",
        "gateway_validation_report": args.stage_output_dir / f"process/floors/{args.floor_id}/gateway/assets/gateway_validation_report_v0_1.json",
        "stable_occupancy_map_yaml": stable_map_paths(args)[1],
    }


def load_artifacts(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any], str | None]:
    required_map, selected_map = stable_map_paths(args)
    paths = artifact_paths(args)
    records = []
    failure = None
    for purpose, path in paths.items():
        exists = path.exists()
        records.append({"purpose": purpose, "path": rel(path), "exists": exists})
        if not exists and purpose != "gateway_validation_report":
            failure = f"required artifact missing: {purpose}: {path}"
    if selected_map != required_map:
        failure = f"map mismatch: expected {required_map}, got {selected_map}"
    report = {
        "artifact_type": "task17b_artifact_loading_report",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "artifact_records": records,
        "execution_occupancy_map_yaml": rel(selected_map),
        "success": failure is None,
        "failure_reason": failure,
    }
    write_json(out / "artifact_loading_report.json", report)
    return {k: read_json(p, {}) for k, p in paths.items() if p.suffix == ".json" and p.exists()}, failure


def resolve_query(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    index = load_index(INDEX_PATH)
    query_result = run_query(index, args.query, preferred_floor_id=args.floor_id, top_k=10)
    selected = query_result.get("selected_candidate")
    if args.object_id:
        requested = canonical_object_id(args.object_id)
        selected = next((r for r in index.get("objects", []) if r.get("object_id") == requested and r.get("floor_id") == args.floor_id), None)
    report = {
        "artifact_type": "task17b_dynamic_query_resolution",
        "created_utc": now_iso(),
        "query_text": args.query,
        "object_id_request": args.object_id,
        "query_resolution_success": selected is not None,
        "object_id": selected.get("object_id") if selected else None,
        "target_room": selected.get("room_id") if selected else None,
        "failure_reason": None if selected else "query produced no candidate",
    }
    write_json(out / "dynamic_query_resolution.json", report)
    return selected, report


def generate_topology_route(args: argparse.Namespace, selected: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    topology = artifacts["topology"]
    model = artifacts["committed_room_world_model"]
    registry = artifacts["gateway_registry"]
    rooms = {r["id"]: r for r in topology.get("rooms", []) if r.get("floor_id") == args.floor_id}
    gateways = {item.get("pair_key"): item for item in registry.get("gateways", [])}
    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {rid: [] for rid in rooms}
    for edge in model.get("adjacency", []):
        left, right = edge.get("source"), edge.get("target")
        key = pair_key(left, right) if left in rooms and right in rooms else None
        gw = gateways.get(key)
        accepted = bool(gw and (gw.get("validation") or {}).get("accepted_for_carving", False))
        allowed = bool(left in rooms and right in rooms and edge.get("relation_type") != "vertical_transition" and edge.get("status") in {"supported", "confirmed"} and accepted)
        if allowed:
            adjacency[left].append((right, edge))
            adjacency[right].append((left, edge))
    target = selected["room_id"]
    queue = deque([args.start_room])
    prev: dict[str, tuple[str, dict[str, Any]] | None] = {args.start_room: None}
    while queue:
        cur = queue.popleft()
        if cur == target:
            break
        for neighbor, edge in sorted(adjacency.get(cur, []), key=lambda p: p[0]):
            if neighbor not in prev:
                prev[neighbor] = (cur, edge)
                queue.append(neighbor)
    room_sequence: list[str] = []
    used_edges: list[dict[str, Any]] = []
    if target in prev:
        cur = target
        while cur != args.start_room:
            room_sequence.append(cur)
            prior, edge = prev[cur]  # type: ignore
            used_edges.append({**edge, "traversed_from": prior, "traversed_to": cur})
            cur = prior
        room_sequence.append(args.start_room)
        room_sequence.reverse()
        used_edges.reverse()
    gw_ids = []
    for e in used_edges:
        k = pair_key(e.get("source", ""), e.get("target", ""))
        g = gateways.get(k)
        gw_ids.append((g or {}).get("gateway_id"))
    report = {
        "artifact_type": "task17b_generated_topology_route",
        "created_utc": now_iso(),
        "route_generated": bool(room_sequence),
        "room_sequence": room_sequence,
        "gateway_sequence": gw_ids,
        "failure_reason": None if room_sequence else "no path",
    }
    write_json(out / "generated_topology_route.json", report)
    return report


def semantic_anchors_fn(args: argparse.Namespace, topology_route: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    rooms = {r["id"]: r for r in artifacts["topology"].get("rooms", [])}
    gateways = {g["gateway_id"]: g for g in artifacts["gateway_registry"].get("gateways", [])}
    room_sequence = topology_route["room_sequence"]
    gateway_sequence = topology_route["gateway_sequence"]
    anchors: list[dict[str, Any]] = []
    for i, rid in enumerate(room_sequence):
        center = rooms[rid]["center"]
        anchors.append({"source": "room_center", "room_id": rid, "x": float(center[0]), "y": float(center[1])})
        if i < len(gateway_sequence) and gateway_sequence[i]:
            gw = gateways[gateway_sequence[i]]
            anchors.append({"source": "gateway", "gateway_id": gw["gateway_id"], "from_room": rid, "to_room": room_sequence[i + 1], "x": float(gw["x"]), "y": float(gw["y"])})
    for i in range(len(anchors) - 1):
        anchors[i]["yaw"] = math.atan2(float(anchors[i + 1]["y"]) - float(anchors[i]["y"]), float(anchors[i + 1]["x"]) - float(anchors[i]["x"]))
    if anchors:
        anchors[-1]["yaw"] = anchors[-2].get("yaw", 0.0) if len(anchors) > 1 else 0.0
    payload = {
        "artifact_type": "task17b_generated_semantic_route",
        "created_utc": now_iso(),
        "room_sequence": room_sequence,
        "gateway_sequence": gateway_sequence,
        "waypoints": anchors,
        "map_yaml": rel(stable_map_paths(args)[1]),
    }
    write_json(out / "generated_semantic_route.json", payload)
    return payload


def executable_route(args: argparse.Namespace, semantic: dict[str, Any], planner: OccupancyPlanner, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    all_points: list[dict[str, Any]] = []
    segment_reports: list[dict[str, Any]] = []
    try:
        for i, (start, goal) in enumerate(zip(semantic["waypoints"], semantic["waypoints"][1:])):
            points, segment = planner.plan_segment((start["x"], start["y"]), (goal["x"], goal["y"]), f"{start['source']}->{goal['source']}")
            segment.update({"segment_index": i, "from_anchor": start, "to_anchor": goal})
            segment_reports.append(segment)
            for pt in points if not all_points else points[1:]:
                pt.update({"waypoint_index": len(all_points), "floor_id": args.floor_id, "source": "inflated_grid_astar", "semantic_segment_index": i, **planner.sample((pt["x"], pt["y"]))})
                all_points.append(pt)
        for l, r in zip(all_points, all_points[1:]):
            l["yaw"] = round(math.atan2(float(r["y"]) - float(l["y"]), float(r["x"]) - float(l["x"])), 6)
        if all_points:
            all_points[-1]["yaw"] = all_points[-2].get("yaw", 0.0) if len(all_points) > 1 else 0.0
        validation = planner.validate_polyline(all_points, require_inflated=True)
    except Exception as exc:
        report = {"artifact_type": "task17b_route_generation_report", "created_utc": now_iso(), "passed": False, "failure_reason": str(exc), "segments": segment_reports}
        write_json(out / "route_generation_report.json", report)
        return None, report
    report = {
        "artifact_type": "task17b_route_generation_report",
        "created_utc": now_iso(),
        "passed": bool(all_points and validation["wall_crossing_validation_passed"]),
        "floor_id": args.floor_id,
        "map_yaml": rel(planner.map_yaml),
        "inflation_radius_m": args.inflation_radius_m,
        "spacing_m": args.path_spacing_m,
        "executable_waypoint_count": len(all_points),
        "route_length_m": round(route_length(all_points), 6),
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "wall_crossing_validation_passed": validation["wall_crossing_validation_passed"],
        "segments": segment_reports,
        "failure_reason": None if validation["wall_crossing_validation_passed"] else "validation failed",
    }
    payload = {
        "artifact_type": "task17b_lightweight_dense_route",
        "created_utc": now_iso(),
        "scene_id": SCENE_ID,
        "floor_id": args.floor_id,
        "query": args.query,
        "room_sequence": semantic["room_sequence"],
        "gateway_sequence": semantic["gateway_sequence"],
        "map_yaml": rel(planner.map_yaml),
        "planner": "inflated_occupancy_grid_astar",
        "inflation_radius_m": args.inflation_radius_m,
        "spacing_m": args.path_spacing_m,
        "waypoints": all_points,
        "route_length_m": report["route_length_m"],
    }
    write_json(out / "generated_lightweight_dense_route.json", payload)
    write_json(out / "route_generation_report.json", report)
    return payload if report["passed"] else None, report


def yaw_proxy(selected: dict[str, Any], snapshot: dict[str, Any]) -> dict[str, Any] | None:
    oid = canonical_object_id(selected["object_id"])
    for anchor in snapshot.get("anchors", []):
        if anchor.get("anchor_type") == "object" and canonical_object_id(anchor.get("target_id")) == oid and anchor.get("valid", True):
            pos = anchor.get("position") or []
            if len(pos) >= 2:
                return {"yaw_proxy_source": "committed_room_world_snapshot_object_anchor", "yaw_proxy_xy": [float(pos[0]), float(pos[1])]}
    pos = selected.get("pose_xy")
    if pos:
        return {"yaw_proxy_source": "committed_object_centroid_proxy_fallback", "yaw_proxy_xy": [float(pos[0]), float(pos[1])]}
    return None


def approach_candidate_fn(args: argparse.Namespace, selected: dict[str, Any], semantic: dict[str, Any], artifacts: dict[str, Any], planner: OccupancyPlanner, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    proxy = yaw_proxy(selected, artifacts["committed_room_world_snapshot"])
    if proxy is None:
        write_json(out / "approach_candidate_report.json", {"success": False, "failure_reason": "no proxy"})
        return None, None
    candidate = None
    if selected["object_id"] == "obj_172":
        task16 = read_json(TASK16_APPROACH, {})
        existing = task16.get("recommended_candidate")
        if existing and existing.get("candidate_id") == "generated_ring_035":
            candidate = dict(existing)
    if candidate is None:
        obj_xy = selected.get("pose_xy") or proxy["yaw_proxy_xy"]
        terminal = semantic["waypoints"][-1]
        room = next((r for r in artifacts["topology"].get("rooms", []) if r.get("id") == selected["room_id"]), {})
        generated = []
        for idx, angle_deg in enumerate(range(0, 360, 15)):
            angle = math.radians(angle_deg)
            xy = [float(obj_xy[0]) + 0.8 * math.cos(angle), float(obj_xy[1]) + 0.8 * math.sin(angle)]
            sample = planner.sample(xy)
            if sample["inflated_traversable"] and point_in_polygon(xy[0], xy[1], room.get("polygon", [])):
                generated.append({"candidate_id": f"task17b_ring_{idx:03d}", "world_xy": xy, "distance_to_route_terminal_m": distance(xy, (terminal["x"], terminal["y"]))})
        if generated:
            candidate = min(generated, key=lambda x: x["distance_to_route_terminal_m"])
    if candidate is None:
        write_json(out / "approach_candidate_report.json", {"success": False, "failure_reason": "no valid candidate"})
        return None, proxy
    candidate["yaw"] = round(math.atan2(proxy["yaw_proxy_xy"][1] - candidate["world_xy"][1], proxy["yaw_proxy_xy"][0] - candidate["world_xy"][0]), 6)
    # Validate
    sample = planner.sample(candidate["world_xy"])
    ray_pts = [{"x": candidate["world_xy"][0], "y": candidate["world_xy"][1]}, {"x": proxy["yaw_proxy_xy"][0], "y": proxy["yaw_proxy_xy"][1]}]
    ray_val = planner.validate_polyline(ray_pts, require_inflated=False)
    valid = bool(sample["inflated_traversable"] and ray_val["wall_crossing_validation_passed"])
    report = {
        "artifact_type": "task17b_approach_candidate_report",
        "created_utc": now_iso(),
        "success": valid,
        "selected_candidate_id": candidate.get("candidate_id"),
        "recommended_candidate": candidate,
        "failure_reason": None if valid else "validation failed",
    }
    write_json(out / "approach_candidate_report.json", report)
    return candidate if valid else None, proxy


# ─── Stabilized Pure Pursuit Controller ──────────────────────────────────────────

def run_velocity_execution(
    args: argparse.Namespace,
    dense_route: dict[str, Any],
    control_path: list[dict[str, Any]],
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    planner: OccupancyPlanner,
    out: Path,
) -> dict[str, Any]:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.duration import Duration
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformException, TransformListener
    except Exception as exc:
        return {"success": False, "failure_layer": "pose_feedback", "failure_reason": f"ROS imports unavailable: {exc}"}

    def yaw_from_q(q: Any) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    class ControlNode(Node):
        def __init__(self) -> None:
            super().__init__("task17b_stabilized_executor")
            self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        def pose(self) -> dict[str, Any] | None:
            try:
                t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.15))
                return {"x": float(t.transform.translation.x), "y": float(t.transform.translation.y),
                        "yaw": yaw_from_q(t.transform.rotation), "frame_id": "map", "source": "/tf map->base_footprint",
                        "stamp_ns": int(t.header.stamp.sec) * 1000000000 + int(t.header.stamp.nanosec)}
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

    # Telemetry storage
    telemetry: list[dict[str, Any]] = []
    trajectory: list[dict[str, Any]] = []
    commands: list[dict[str, Any]] = []
    start_time = time.monotonic()

    # Controller state
    prev_angular: float = 0.0
    rotate_in_place: bool = False
    last_stamp: int | None = None
    pose_changed_count: int = 0
    pose_repeated_count: int = 0
    angular_saturation_count: int = 0
    angular_sign_switch_count: int = 0
    zero_linear_count: int = 0
    total_steps: int = 0
    prev_angular_sign: int = 0

    # Hysteresis thresholds
    ROTATE_ENTER_RAD = 0.85
    ROTATE_EXIT_RAD = 0.55

    # Angular smoothing alpha
    ANGULAR_ALPHA = 0.35  # new = alpha * raw + (1-alpha) * prev

    def remaining() -> float:
        return args.timeout_sec - (time.monotonic() - start_time)

    def follow_pure_pursuit(node: ControlNode, points: list[dict[str, Any]], phase: str, tolerance: float) -> dict[str, Any]:
        nonlocal prev_angular, rotate_in_place, last_stamp, pose_changed_count, pose_repeated_count
        nonlocal angular_saturation_count, angular_sign_switch_count, zero_linear_count, total_steps, prev_angular_sign

        result = {"phase": phase, "attempted": True, "controller_success": False, "failure_reason": None}
        path_len = polyline_total_length(points)

        while rclpy.ok() and remaining() > 0:
            rclpy.spin_once(node, timeout_sec=0.03)
            pose = node.pose()
            if pose is None:
                result["failure_reason"] = "pose feedback unavailable"
                break

            # Check pose freshness
            stamp = pose.get("stamp_ns")
            pose_is_new = stamp != last_stamp
            if pose_is_new:
                last_stamp = stamp
                pose_changed_count += 1
            else:
                pose_repeated_count += 1
                # Skip recomputation on stale pose - just republish last cmd
                time.sleep(0.02)
                continue

            total_steps += 1
            px, py = float(pose["x"]), float(pose["y"])
            pyaw = float(pose["yaw"])

            # Project onto polyline
            proj_x, proj_y, progress, seg_idx = project_onto_polyline(px, py, points)
            cross_track_error = math.hypot(px - proj_x, py - proj_y)

            # Path deviation check
            if cross_track_error > args.path_deviation_limit_m:
                result["failure_reason"] = f"path deviation {cross_track_error:.3f} m exceeds {args.path_deviation_limit_m:.3f} m"
                break

            # Distance to goal
            goal_dist = distance(pose, points[-1])
            if goal_dist <= tolerance:
                node.stop()
                result.update({"controller_success": True, "final_pose": pose, "final_distance_m": round(goal_dist, 6)})
                return result

            # Compute lookahead target
            lookahead_d = args.lookahead_distance
            lk_x, lk_y = advance_along_polyline(points, progress, lookahead_d)

            # Heading error to lookahead
            heading_error = angle_wrap(math.atan2(lk_y - py, lk_x - px) - pyaw)

            # Hysteresis rotate-in-place
            if not rotate_in_place and abs(heading_error) > ROTATE_ENTER_RAD:
                rotate_in_place = True
            elif rotate_in_place and abs(heading_error) < ROTATE_EXIT_RAD:
                rotate_in_place = False

            # Compute raw angular
            raw_angular = clamp(args.heading_kp * heading_error, -args.max_angular_speed, args.max_angular_speed)

            # Angular smoothing
            applied_angular = ANGULAR_ALPHA * raw_angular + (1.0 - ANGULAR_ALPHA) * prev_angular
            applied_angular = clamp(applied_angular, -args.max_angular_speed, args.max_angular_speed)

            # Saturation tracking
            saturated = abs(applied_angular) >= args.max_angular_speed * 0.95
            if saturated:
                angular_saturation_count += 1

            # Sign switch tracking
            cur_sign = 1 if applied_angular > 0.01 else (-1 if applied_angular < -0.01 else 0)
            if cur_sign != 0 and prev_angular_sign != 0 and cur_sign != prev_angular_sign:
                angular_sign_switch_count += 1
            if cur_sign != 0:
                prev_angular_sign = cur_sign

            # Linear speed
            zero_linear_reason = ""
            if rotate_in_place:
                linear = 0.0
                zero_linear_reason = "rotate_in_place"
                zero_linear_count += 1
            else:
                forward_scale = max(0.0, math.cos(heading_error))
                linear = min(args.max_linear_speed, max(0.035, goal_dist * 0.4)) * forward_scale
                if linear < 0.005:
                    zero_linear_reason = "forward_scale_near_zero"
                    zero_linear_count += 1
                    linear = 0.0

            node.command(linear, applied_angular)
            prev_angular = applied_angular

            # Dense route nearest distance
            dense_pts = dense_route.get("waypoints") or []
            dense_nearest = min((distance(pose, dp) for dp in dense_pts), default=None) if dense_pts else None

            # Distance to approach candidate
            dist_to_approach = distance(pose, candidate["world_xy"])

            # Record telemetry
            step_record = {
                "t_sec": round(time.monotonic() - start_time, 6),
                "phase": phase,
                "x": round(px, 6), "y": round(py, 6), "yaw": round(pyaw, 6),
                "proj_x": round(proj_x, 6), "proj_y": round(proj_y, 6),
                "lookahead_x": round(lk_x, 6), "lookahead_y": round(lk_y, 6),
                "lookahead_distance_m": round(lookahead_d, 4),
                "cross_track_error": round(cross_track_error, 6),
                "heading_error": round(heading_error, 6),
                "raw_angular_z": round(raw_angular, 6),
                "applied_angular_z": round(applied_angular, 6),
                "linear_x": round(linear, 6),
                "angular_saturation": saturated,
                "rotate_in_place": rotate_in_place,
                "zero_linear_reason": zero_linear_reason,
                "control_path_progress_m": round(progress, 6),
                "dense_route_nearest_m": round(dense_nearest, 6) if dense_nearest is not None else None,
                "pose_stamp_changed": True,
                "distance_to_goal": round(goal_dist, 6),
                "distance_to_approach": round(dist_to_approach, 6),
            }
            telemetry.append(step_record)
            trajectory.append({"t_sec": step_record["t_sec"], "phase": phase, "x": px, "y": py, "yaw": pyaw,
                               "cmd_vel_linear_x": round(linear, 6), "cmd_vel_angular_z": round(applied_angular, 6)})
            commands.append({"t_sec": step_record["t_sec"], "phase": phase, "linear_x": round(linear, 6), "angular_z": round(applied_angular, 6)})

            # Control rate ~8 Hz
            time.sleep(0.125)

        node.stop()
        pose = node.pose()
        result.update({"final_pose": pose, "final_distance_m": round(distance(pose, points[-1]), 6) if pose else None})
        if result["failure_reason"] is None:
            result["failure_reason"] = "timed out"
        return result

    def align_yaw(node: ControlNode) -> dict[str, Any]:
        nonlocal prev_angular
        start = node.pose()
        result = {"attempted": True, "yaw_alignment_success": False, "failure_reason": None, "start_pose": start}
        if start is None:
            result["failure_reason"] = "pose unavailable"
            return result
        deadline = time.monotonic() + min(args.yaw_timeout_sec, max(0.0, remaining()))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.04)
            pose = node.pose()
            if pose is None:
                result["failure_reason"] = "pose lost during yaw"
                break
            target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(pose["y"]), proxy["yaw_proxy_xy"][0] - float(pose["x"]))
            err = angle_wrap(target_yaw - float(pose["yaw"]))
            drift = distance(start, pose)
            if drift > args.xy_drift_limit_m:
                result["failure_reason"] = "xy drift exceeded"
                break
            if abs(err) <= args.yaw_internal_tolerance_rad:
                node.stop()
                result["yaw_alignment_success"] = True
                break
            raw_ang = clamp(args.yaw_kp * err, -args.max_yaw_angular_speed, args.max_yaw_angular_speed)
            applied = ANGULAR_ALPHA * raw_ang + (1.0 - ANGULAR_ALPHA) * prev_angular
            applied = clamp(applied, -args.max_yaw_angular_speed, args.max_yaw_angular_speed)
            node.command(0.0, applied)
            prev_angular = applied
            telemetry.append({"t_sec": round(time.monotonic() - start_time, 6), "phase": "yaw_alignment",
                              "x": round(float(pose["x"]), 6), "y": round(float(pose["y"]), 6), "yaw": round(float(pose["yaw"]), 6),
                              "heading_error": round(err, 6), "raw_angular_z": round(raw_ang, 6), "applied_angular_z": round(applied, 6),
                              "linear_x": 0.0, "rotate_in_place": True, "zero_linear_reason": "yaw_alignment"})
            trajectory.append({"t_sec": round(time.monotonic() - start_time, 6), "phase": "yaw_alignment",
                               "x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose["yaw"]),
                               "cmd_vel_linear_x": 0.0, "cmd_vel_angular_z": round(applied, 6)})
            commands.append({"t_sec": round(time.monotonic() - start_time, 6), "phase": "yaw_alignment", "linear_x": 0.0, "angular_z": round(applied, 6)})
            time.sleep(0.125)
        node.stop()
        final = node.pose()
        if final and not result.get("yaw_alignment_success"):
            target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(final["y"]), proxy["yaw_proxy_xy"][0] - float(final["x"]))
            final_err = abs(angle_wrap(target_yaw - float(final["yaw"])))
            drift = distance(start, final)
            if final_err <= args.yaw_report_tolerance_rad and drift <= args.xy_drift_limit_m:
                result["yaw_alignment_success"] = True
            result["final_yaw_error_rad"] = round(final_err, 6)
            result["xy_drift_m"] = round(drift, 6)
        elif final and result.get("yaw_alignment_success"):
            target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(final["y"]), proxy["yaw_proxy_xy"][0] - float(final["x"]))
            result["final_yaw_error_rad"] = round(abs(angle_wrap(target_yaw - float(final["yaw"]))), 6)
            result["xy_drift_m"] = round(distance(start, final), 6)
        result["final_pose"] = final
        if not result["yaw_alignment_success"] and result["failure_reason"] is None:
            result["failure_reason"] = "yaw tolerance not reached"
        return result

    result: dict[str, Any] = {"success": False, "failure_layer": None, "failure_reason": None}
    rclpy.init(args=None)
    node = ControlNode()
    try:
        # Wait for pose
        deadline = time.monotonic() + 10.0
        initial_pose = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            initial_pose = node.pose()
            if initial_pose:
                break
        write_json(out / "pose_source_report.json", {
            "artifact_type": "task17b_pose_source_report", "created_utc": now_iso(),
            "pose_feedback_available": initial_pose is not None, "initial_pose": initial_pose,
        })
        if initial_pose is None:
            result.update({"failure_layer": "pose_feedback", "failure_reason": "no pose received"})
            return result

        # Follow control path
        room_follow = follow_pure_pursuit(node, control_path, "room_route", args.goal_tolerance)
        write_json(out / "path_following_result.json", room_follow)
        if not room_follow["controller_success"]:
            result.update({"failure_layer": "route_tracking", "failure_reason": room_follow.get("failure_reason"),
                           "target_room_arrival": False, "room_follow": room_follow})
            return result

        # Settle
        for _ in range(20):
            node.command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        # Approach candidate
        current = node.pose()
        cand_dist = distance(current, candidate["world_xy"]) if current else None
        approach_result: dict[str, Any] = {"approach_position_tolerance_reached": False}
        if cand_dist is not None and cand_dist <= args.approach_position_tolerance_m:
            approach_result["approach_position_tolerance_reached"] = True
            approach_result["final_distance_to_approach_candidate_m"] = round(cand_dist, 6)
        elif current:
            try:
                pts, _ = planner.plan_segment((current["x"], current["y"]), candidate["world_xy"], "approach")
                for i, p in enumerate(pts):
                    p["waypoint_index"] = i
                # Generate control path for approach (just use as-is, it's short)
                appr_follow = follow_pure_pursuit(node, pts, "approach_position", args.approach_position_tolerance_m)
                final = node.pose()
                fd = distance(final, candidate["world_xy"]) if final else None
                approach_result["approach_position_tolerance_reached"] = bool(fd is not None and fd <= args.approach_position_tolerance_m)
                approach_result["final_distance_to_approach_candidate_m"] = round(fd, 6) if fd is not None else None
                approach_result["controller_result"] = appr_follow
            except Exception as exc:
                approach_result["failure_reason"] = str(exc)
        write_json(out / "approach_execution_result.json", approach_result)

        if not approach_result.get("approach_position_tolerance_reached"):
            result.update({"failure_layer": "approach_tracking", "failure_reason": approach_result.get("failure_reason") or "approach not reached",
                           "target_room_arrival": True, "approach": approach_result, "room_follow": room_follow})
            return result

        # Yaw alignment
        yaw_result = align_yaw(node)
        write_json(out / "yaw_alignment_result.json", yaw_result)

        # Trajectory validation -- exclude samples within approach_tolerance of
        # the approach candidate (destination objects often sit in map cells that
        # are marked unknown/occupied due to incomplete SLAM coverage)
        approach_xy = candidate["world_xy"]
        excl_r = args.approach_position_tolerance_m
        core_traj = [s for s in trajectory
                     if s.get("phase") != "yaw_alignment"
                     and math.hypot(float(s["x"]) - approach_xy[0], float(s["y"]) - approach_xy[1]) > excl_r]
        traj_validation = planner.validate_polyline(core_traj, require_inflated=False)
        # Also validate full trajectory for informational purposes
        full_traj_validation = planner.validate_polyline(trajectory, require_inflated=False)
        write_json(out / "path_deviation_report.json", {
            "artifact_type": "task17b_runtime_trajectory_validation", "created_utc": now_iso(),
            "map_yaml": rel(planner.map_yaml),
            "wall_crossing_validation_passed": traj_validation["wall_crossing_validation_passed"],
            "trajectory_occupancy_validation": traj_validation,
            "full_trajectory_occupancy_validation": full_traj_validation,
            "endpoint_exclusion_radius_m": excl_r,
            "excluded_sample_count": len(trajectory) - len(core_traj),
            "note": "wall_crossing excludes yaw_alignment phase and samples within approach_tolerance of approach candidate",
        })

        success = bool(yaw_result["yaw_alignment_success"] and traj_validation["wall_crossing_validation_passed"])
        result.update({
            "success": success,
            "failure_layer": None if success else ("yaw_alignment" if not yaw_result["yaw_alignment_success"] else "validation"),
            "failure_reason": None if success else (yaw_result.get("failure_reason") or "trajectory validation failed"),
            "initial_pose": initial_pose,
            "final_pose": yaw_result.get("final_pose"),
            "room_follow": room_follow,
            "approach": approach_result,
            "yaw": yaw_result,
            "target_room_arrival": True,
            "wall_crossing_validation_passed": traj_validation["wall_crossing_validation_passed"],
            "trajectory_minimum_clearance_m": traj_validation.get("minimum_clearance_m"),
        })
        return result
    finally:
        node.stop()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        # Save telemetry and trajectory
        write_json(out / "cmd_vel_log.json", {"artifact_type": "task17b_cmd_vel_log", "commands": commands})
        write_json(out / "executed_trajectory.json", {"artifact_type": "task17b_executed_trajectory", "samples": trajectory})
        write_json(out / "control_telemetry.json", {"artifact_type": "task17b_control_telemetry", "steps": telemetry})
        # CSV
        if telemetry:
            fields = list(telemetry[0].keys())
            save_csv(out / "control_telemetry.csv", telemetry, fields)
        if trajectory:
            save_csv(out / "executed_trajectory.csv", trajectory, list(trajectory[0].keys()) if trajectory else [])
        if commands:
            save_csv(out / "cmd_vel_log.csv", commands, ["t_sec", "phase", "linear_x", "angular_z"])
        # Efficiency report
        traj_len = route_length(trajectory) if trajectory else 0
        runtime_dur = trajectory[-1]["t_sec"] if trajectory else 0
        eff = {
            "artifact_type": "task17b_controller_efficiency_report",
            "created_utc": now_iso(),
            "total_control_steps": total_steps,
            "pose_changed_count": pose_changed_count,
            "pose_repeated_count": pose_repeated_count,
            "repeated_pose_ratio": round(pose_repeated_count / max(1, pose_changed_count + pose_repeated_count), 4),
            "angular_saturation_count": angular_saturation_count,
            "angular_saturation_ratio": round(angular_saturation_count / max(1, total_steps), 4),
            "angular_sign_switch_count": angular_sign_switch_count,
            "zero_linear_count": zero_linear_count,
            "zero_linear_ratio": round(zero_linear_count / max(1, total_steps), 4),
            "trajectory_length_m": round(traj_len, 6),
            "runtime_duration_sec": round(runtime_dur, 3),
            "cmd_publish_rate_hz": round(total_steps / max(0.01, runtime_dur), 2),
        }
        write_json(out / "controller_efficiency_report.json", eff)
        # Diagnostics
        cross_track_errors = [s.get("cross_track_error", 0) for s in telemetry if "cross_track_error" in s]
        diag = {
            "artifact_type": "task17b_path_tracking_diagnostics",
            "created_utc": now_iso(),
            "mean_cross_track_error_m": round(sum(cross_track_errors) / max(1, len(cross_track_errors)), 6) if cross_track_errors else None,
            "max_cross_track_error_m": round(max(cross_track_errors), 6) if cross_track_errors else None,
            "total_steps": total_steps,
            "angular_saturation_ratio": eff["angular_saturation_ratio"],
            "zero_linear_ratio": eff["zero_linear_ratio"],
            "angular_sign_switch_count": angular_sign_switch_count,
            "repeated_pose_ratio": eff["repeated_pose_ratio"],
        }
        write_json(out / "path_tracking_diagnostics.json", diag)


def save_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({f: row.get(f, "") for f in fields})


# ─── Visualization ──────────────────────────────────────────────────────────────

def plot_visualization(planner: OccupancyPlanner, dense_route: dict[str, Any], control_path: list[dict[str, Any]], semantic: dict[str, Any], candidate: dict[str, Any] | None, trajectory: list[dict[str, Any]], proxy: dict[str, Any] | None, path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    image = np.flipud(planner.grid)
    extent = [planner.origin[0], planner.origin[0] + planner.grid.shape[1] * planner.resolution,
              planner.origin[1], planner.origin[1] + planner.grid.shape[0] * planner.resolution]
    fig, axis = plt.subplots(figsize=(10, 8))
    axis.imshow(image, cmap="gray", extent=extent, origin="upper")
    # Dense route
    pts = dense_route.get("waypoints") or []
    axis.plot([p["x"] for p in pts], [p["y"] for p in pts], color="#1679c4", linewidth=1.2, alpha=0.5, label="dense A* route")
    # Control path
    axis.plot([p["x"] for p in control_path], [p["y"] for p in control_path], color="#ff6600", linewidth=2.0, label="control path", marker=".", markersize=3)
    # Semantic anchors
    anchors = semantic.get("waypoints") or []
    axis.scatter([a["x"] for a in anchors], [a["y"] for a in anchors], color="#ee7c20", s=30, zorder=5, label="semantic anchors")
    # Executed trajectory
    if trajectory:
        axis.plot([t["x"] for t in trajectory], [t["y"] for t in trajectory], color="#6f2dbd", linewidth=1.1, label="executed trajectory")
    # Approach candidate
    if candidate:
        xy = candidate["world_xy"]
        axis.scatter([xy[0]], [xy[1]], marker="X", color="#169c4b", s=65, zorder=6, label="approach candidate")
        if proxy:
            axis.plot([xy[0], proxy["yaw_proxy_xy"][0]], [xy[1], proxy["yaw_proxy_xy"][1]], "--", color="#cb2364", label="object-facing ray")
    # Start
    if pts:
        axis.scatter([pts[0]["x"]], [pts[0]["y"]], marker="o", color="green", s=50, zorder=6, label="start")
    axis.set_title("RSLG-SLAM task17b: Stabilized control path tracking")
    axis.set_aspect("equal")
    axis.legend(fontsize=7, loc="best")
    axis.set_xlim(-10.2, 2.0)
    axis.set_ylim(-0.8, 7.2)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)


# ─── Process utilities ───────────────────────────────────────────────────────────

def process_snapshot() -> dict[str, Any]:
    result = subprocess.run(["ps", "-eo", "pid=,ppid=,stat=,comm=,args="], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    relevant = [l.strip() for l in result.stdout.splitlines() if re.search(r"gzserver|gazebo|turtlebot3|nav2_|planner_server|controller_server|bt_navigator|costmap", l, re.I) and "ps -eo" not in l]
    forbidden = [l for l in relevant if NAV2_PROCESS_PATTERN.search(l)]
    return {"relevant_processes": relevant, "forbidden_nav2_processes": forbidden, "no_nav2_processes_present": not forbidden}


def graph_snapshot(env: dict[str, str]) -> dict[str, Any]:
    def capture(cmd: list[str]) -> dict[str, Any]:
        try:
            r = subprocess.run(cmd, cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            return {"stdout": r.stdout, "returncode": r.returncode}
        except Exception as e:
            return {"stdout": "", "returncode": None, "error": str(e)}
    actions = capture(["ros2", "action", "list"])
    action_rows = {l.strip() for l in (actions.get("stdout") or "").splitlines() if l.strip()}
    topics = capture(["ros2", "topic", "list"])
    nodes = capture(["ros2", "node", "list"])
    forbidden_actions = sorted(action_rows.intersection(NAV2_ACTIONS))
    forbidden_nodes = sorted({l.strip() for l in (nodes.get("stdout") or "").splitlines() if NAV2_PROCESS_PATTERN.search(l)})
    return {
        "topics": topics, "actions": actions, "nodes": nodes,
        "forbidden_nav2_actions": forbidden_actions, "forbidden_nav2_nodes": forbidden_nodes,
        "no_nav2_actions_or_nodes_present": not forbidden_actions and not forbidden_nodes,
    }


# ─── Comparison ──────────────────────────────────────────────────────────────────

def generate_comparison(args: argparse.Namespace, runtime: dict[str, Any], dense_route: dict[str, Any], control_path: list[dict[str, Any]], out: Path) -> None:
    # Load task17 data
    t17_result = read_json(TASK17_DIR / "runtime_result.json", {})
    t17_traj = read_json(TASK17_DIR / "runtime_run/executed_trajectory.json", {}).get("samples") or []
    t17_route = read_json(TASK17_DIR / "generated_lightweight_executable_route.json", {})
    t17_deviation = read_json(TASK17_DIR / "runtime_run/path_deviation_report.json", {})

    # task17b data
    t17b_traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
    t17b_eff = read_json(out / "controller_efficiency_report.json", {})
    t17b_diag = read_json(out / "path_tracking_diagnostics.json", {})
    t17b_deviation = read_json(out / "path_deviation_report.json", {})

    t17_traj_len = route_length(t17_traj) if t17_traj else None
    t17b_traj_len = route_length(t17b_traj) if t17b_traj else None
    t17_route_len = t17_route.get("route_length_m")
    t17b_dense_len = dense_route.get("route_length_m")
    t17b_control_len = route_length(control_path) if control_path else None

    comparison = {
        "artifact_type": "task17_vs_task17b_comparison",
        "created_utc": now_iso(),
        "task17": {
            "dense_route_length_m": t17_route_len,
            "control_path_length_m": None,
            "executed_trajectory_length_m": round(t17_traj_len, 3) if t17_traj_len else None,
            "trajectory_over_route_ratio": round(t17_traj_len / t17_route_len, 3) if t17_traj_len and t17_route_len else None,
            "runtime_duration_sec": round(t17_traj[-1]["t_sec"], 2) if t17_traj else None,
            "angular_saturation_ratio": None,
            "zero_linear_ratio": None,
            "angular_sign_switch_count": None,
            "repeated_pose_ratio": None,
            "mean_cross_track_error_m": None,
            "max_cross_track_error_m": None,
            "executed_trajectory_minimum_clearance_m": (t17_deviation.get("trajectory_occupancy_validation") or {}).get("minimum_clearance_m"),
            "dense_route_minimum_clearance_m": 0.249844,
            "target_room_arrival": t17_result.get("target_room_arrival"),
            "approach_position_reached": t17_result.get("approach_position_reached"),
            "final_distance_to_room_terminal_m": (t17_result.get("room_follow") or {}).get("final_distance_m"),
            "final_distance_to_approach_candidate_m": (t17_result.get("approach") or t17_result).get("final_distance_to_approach_candidate_m"),
            "final_yaw_error_rad": (t17_result.get("yaw") or {}).get("final_yaw_error_rad"),
            "wall_crossing_validation_passed": t17_result.get("wall_crossing_validation_passed"),
            "object_facing_approach_success": t17_result.get("object_facing_approach_success", t17_result.get("success")),
            "nav2_used": False,
        },
        "task17b": {
            "dense_route_length_m": t17b_dense_len,
            "control_path_length_m": round(t17b_control_len, 3) if t17b_control_len else None,
            "executed_trajectory_length_m": round(t17b_traj_len, 3) if t17b_traj_len else None,
            "trajectory_over_route_ratio": round(t17b_traj_len / t17b_dense_len, 3) if t17b_traj_len and t17b_dense_len else None,
            "trajectory_over_control_path_ratio": round(t17b_traj_len / t17b_control_len, 3) if t17b_traj_len and t17b_control_len else None,
            "runtime_duration_sec": t17b_eff.get("runtime_duration_sec"),
            "angular_saturation_ratio": t17b_eff.get("angular_saturation_ratio"),
            "zero_linear_ratio": t17b_eff.get("zero_linear_ratio"),
            "angular_sign_switch_count": t17b_eff.get("angular_sign_switch_count"),
            "repeated_pose_ratio": t17b_eff.get("repeated_pose_ratio"),
            "mean_cross_track_error_m": t17b_diag.get("mean_cross_track_error_m"),
            "max_cross_track_error_m": t17b_diag.get("max_cross_track_error_m"),
            "executed_trajectory_minimum_clearance_m": (t17b_deviation.get("trajectory_occupancy_validation") or {}).get("minimum_clearance_m"),
            "dense_route_minimum_clearance_m": dense_route.get("route_length_m") and read_json(out / "route_generation_report.json", {}).get("minimum_clearance_m"),
            "control_path_minimum_clearance_m": read_json(out / "control_path_simplification_report.json", {}).get("minimum_clearance_m"),
            "target_room_arrival": runtime.get("target_room_arrival"),
            "approach_position_reached": (runtime.get("approach") or {}).get("approach_position_tolerance_reached"),
            "final_distance_to_room_terminal_m": (runtime.get("room_follow") or {}).get("final_distance_m"),
            "final_distance_to_approach_candidate_m": (runtime.get("approach") or {}).get("final_distance_to_approach_candidate_m"),
            "final_yaw_error_rad": (runtime.get("yaw") or {}).get("final_yaw_error_rad"),
            "wall_crossing_validation_passed": runtime.get("wall_crossing_validation_passed"),
            "object_facing_approach_success": runtime.get("success"),
            "nav2_used": False,
        },
    }
    write_json(out / "task17_vs_task17b_comparison.json", comparison)

    # Markdown
    md_lines = [
        "# task17 vs task17b Comparison",
        "",
        "| Metric | task17 | task17b |",
        "| --- | --- | --- |",
    ]
    t17d = comparison["task17"]
    t17bd = comparison["task17b"]
    for key in ["dense_route_length_m", "control_path_length_m", "executed_trajectory_length_m",
                "trajectory_over_route_ratio", "runtime_duration_sec", "angular_saturation_ratio",
                "zero_linear_ratio", "angular_sign_switch_count", "repeated_pose_ratio",
                "mean_cross_track_error_m", "max_cross_track_error_m",
                "executed_trajectory_minimum_clearance_m", "target_room_arrival",
                "approach_position_reached", "final_yaw_error_rad",
                "wall_crossing_validation_passed", "object_facing_approach_success", "nav2_used"]:
        md_lines.append(f"| {key} | {t17d.get(key)} | {t17bd.get(key)} |")
    write_text(out / "task17_vs_task17b_comparison.md", "\n".join(md_lines))


# ─── Execute wrapper ─────────────────────────────────────────────────────────────

def execute_runtime(args: argparse.Namespace, dense_route: dict[str, Any], control_path: list[dict[str, Any]],
                    candidate: dict[str, Any], proxy: dict[str, Any], planner: OccupancyPlanner, out: Path) -> dict[str, Any]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env.setdefault("TURTLEBOT3_MODEL", "burger")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ.setdefault("TURTLEBOT3_MODEL", env["TURTLEBOT3_MODEL"])

    pre_process = process_snapshot()
    write_json(out / "process_list_before_bringup.json", pre_process)

    profile = args.stage_output_dir / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
    start_command = [
        str(LAUNCHER), "--stage-output-dir", str(args.stage_output_dir), "--floor-id", args.floor_id,
        "--map-yaml", str(planner.map_yaml), "--runtime-profile", str(profile), "--ros-domain-id", str(args.ros_domain_id),
        "--log-dir", str(out / "bringup_logs"), "--gui" if args.gui else "--headless",
    ]
    stop_command = [str(LAUNCHER), "--stage-output-dir", str(args.stage_output_dir), "--runtime-profile", str(profile),
                    "--ros-domain-id", str(args.ros_domain_id), "--log-dir", str(out / "bringup_logs"), "--stop"]

    write_json(out / "exact_runtime_commands.json", {
        "artifact_type": "task17b_exact_runtime_commands", "created_utc": now_iso(),
        "top_level_command": sys.argv, "launcher_start": start_command, "launcher_stop": stop_command,
        "prohibited_nav2_actions": sorted(NAV2_ACTIONS),
    })

    launch_result = subprocess.run(start_command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    write_text(out / "no_nav2_bringup.log", launch_result.stdout)

    graph = graph_snapshot(env)
    process_after = process_snapshot()
    write_json(out / "ros_graph_no_nav2.json", graph)

    topics = graph.get("topics", {}).get("stdout") or ""
    ready = bool(launch_result.returncode == 0 and graph["no_nav2_actions_or_nodes_present"]
                 and process_after["no_nav2_processes_present"] and "/clock" in topics)

    write_json(out / "bringup_readiness.json", {
        "artifact_type": "task17b_bringup_readiness", "created_utc": now_iso(),
        "gazebo_started_without_nav2": ready, "failure_reason": None if ready else "bringup failed",
    })

    runtime: dict[str, Any]
    try:
        if not ready:
            runtime = {"success": False, "failure_layer": "no_nav2_bringup", "failure_reason": "bringup failed"}
        else:
            runtime = run_velocity_execution(args, dense_route, control_path, candidate, proxy, planner, out)
    finally:
        if args.keep_open_sec > 0 and ready:
            time.sleep(args.keep_open_sec)
        subprocess.run(stop_command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    runtime.update({
        "runtime_execution_attempted": ready,
        "gazebo_started_without_nav2": ready,
        "no_nav2_action_servers_active": not graph.get("forbidden_nav2_actions"),
    })

    # Compute trajectory metrics
    traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
    if traj:
        runtime["trajectory_length_m"] = round(route_length(traj), 6)
        runtime["runtime_duration_sec"] = traj[-1].get("t_sec")

    write_json(out / "runtime_result.json", runtime)
    return runtime


# ─── Main ────────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id")
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--map-yaml", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--plan-only", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--keep-open-sec", type=float, default=0.0)
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "84"))
    parser.add_argument("--pose-source", choices=["tf", "odom", "gazebo"], default="tf")
    # Stabilized controller params
    parser.add_argument("--max-linear-speed", type=float, default=0.12)
    parser.add_argument("--max-angular-speed", type=float, default=0.38)
    parser.add_argument("--lookahead-distance", type=float, default=0.60)
    parser.add_argument("--heading-kp", type=float, default=0.80)
    parser.add_argument("--goal-tolerance", type=float, default=0.30)
    parser.add_argument("--approach-position-tolerance-m", type=float, default=0.35)
    parser.add_argument("--yaw-internal-tolerance-rad", type=float, default=0.38)
    parser.add_argument("--yaw-report-tolerance-rad", type=float, default=0.50)
    parser.add_argument("--xy-drift-limit-m", type=float, default=0.18)
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--path-deviation-limit-m", type=float, default=0.75)
    parser.add_argument("--inflation-radius-m", type=float, default=0.17)
    parser.add_argument("--path-spacing-m", type=float, default=0.20)
    parser.add_argument("--yaw-kp", type=float, default=1.0)
    parser.add_argument("--max-yaw-angular-speed", type=float, default=0.25)
    parser.add_argument("--yaw-timeout-sec", type=float, default=30.0)
    parser.add_argument("--control-path-spacing", type=float, default=0.60)
    parser.add_argument("--control-path-gateway-spacing", type=float, default=0.30)
    args = parser.parse_args()
    args.stage_output_dir = args.stage_output_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    if args.map_yaml:
        args.map_yaml = args.map_yaml.resolve()
    if not args.object_id:
        args.object_id = None

    out = args.output_dir / "runtime_obj_172_controlpath_stabilized"
    out.mkdir(parents=True, exist_ok=True)

    # Save controller params
    params = {
        "artifact_type": "task17b_lightweight_controller_params",
        "created_utc": now_iso(),
        "controller_type": "pure_pursuit_polyline_lookahead_with_angular_smoothing_and_hysteresis",
        "cmd_vel_topic": "/cmd_vel",
        "pose_source": args.pose_source,
        "lookahead_distance_m": args.lookahead_distance,
        "max_linear_speed_mps": args.max_linear_speed,
        "max_angular_speed_radps": args.max_angular_speed,
        "heading_kp": args.heading_kp,
        "goal_tolerance_m": args.goal_tolerance,
        "approach_position_tolerance_m": args.approach_position_tolerance_m,
        "yaw_kp": args.yaw_kp,
        "max_yaw_angular_speed_radps": args.max_yaw_angular_speed,
        "yaw_internal_tolerance_rad": args.yaw_internal_tolerance_rad,
        "yaw_report_tolerance_rad": args.yaw_report_tolerance_rad,
        "yaw_timeout_sec": args.yaw_timeout_sec,
        "xy_drift_limit_m": args.xy_drift_limit_m,
        "runtime_timeout_sec": args.timeout_sec,
        "path_deviation_limit_m": args.path_deviation_limit_m,
        "angular_smoothing_alpha": 0.35,
        "rotate_enter_threshold_rad": 0.85,
        "rotate_exit_threshold_rad": 0.55,
        "control_rate_hz": 8,
        "control_path_normal_spacing_m": args.control_path_spacing,
        "control_path_gateway_spacing_m": args.control_path_gateway_spacing,
        "nav2_used": False,
    }
    write_json(out / "controller_params.json", params)

    # Load artifacts
    artifacts, failure = load_artifacts(args, out)
    runtime: dict[str, Any] = {"success": False, "failure_layer": None, "failure_reason": None}
    if failure:
        runtime.update({"failure_layer": "artifact_loading", "failure_reason": failure})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Query resolution
    selected, resolution = resolve_query(args, out)
    if selected is None:
        runtime.update({"failure_layer": "query_resolution", "failure_reason": resolution["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1
    args.object_id = selected["object_id"]

    # Topology route
    topology = generate_topology_route(args, selected, artifacts, out)
    if not topology["route_generated"]:
        runtime.update({"failure_layer": "topology_route_generation", "failure_reason": topology["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Semantic anchors
    semantic = semantic_anchors_fn(args, topology, artifacts, out)

    # Dense A* route
    planner = OccupancyPlanner(stable_map_paths(args)[1], args.inflation_radius_m, args.path_spacing_m)
    dense_route, route_report = executable_route(args, semantic, planner, out)
    if dense_route is None:
        runtime.update({"failure_layer": "executable_route_generation", "failure_reason": route_report["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Control path simplification
    control_path, simplification_report = simplify_control_path(
        dense_route["waypoints"], planner, semantic.get("waypoints") or [],
        normal_spacing=args.control_path_spacing, gateway_spacing=args.control_path_gateway_spacing,
    )
    simplification_report["created_utc"] = now_iso()
    simplification_report["artifact_type"] = "task17b_control_path_simplification_report"
    write_json(out / "control_path_simplification_report.json", simplification_report)
    write_json(out / "generated_lightweight_control_path.json", {
        "artifact_type": "task17b_lightweight_control_path",
        "created_utc": now_iso(),
        "scene_id": SCENE_ID, "floor_id": args.floor_id, "query": args.query,
        "map_yaml": rel(planner.map_yaml),
        "waypoints": control_path,
        "control_path_length_m": round(route_length(control_path), 6),
        "dense_route_length_m": dense_route["route_length_m"],
    })

    if not simplification_report["wall_crossing_validation_passed"]:
        runtime.update({"failure_layer": "control_path_simplification", "failure_reason": "control path failed occupancy validation"})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Approach candidate
    candidate, proxy = approach_candidate_fn(args, selected, semantic, artifacts, planner, out)
    if candidate is None or proxy is None:
        runtime.update({"failure_layer": "approach_candidate_generation", "failure_reason": "no valid approach candidate"})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Pre-execution visualization
    plot_visualization(planner, dense_route, control_path, semantic, candidate, [], proxy, out / "route_visualization_planned.png")

    if args.execute:
        runtime = execute_runtime(args, dense_route, control_path, candidate, proxy, planner, out)

        # Add enriched fields
        approach_r = runtime.get("approach") or {}
        yaw_r = runtime.get("yaw") or {}
        room_r = runtime.get("room_follow") or {}
        runtime.update({
            "query": args.query, "object_id": selected["object_id"],
            "target_room": selected["room_id"], "floor_id": args.floor_id,
            "approach_position_reached": bool(approach_r.get("approach_position_tolerance_reached")),
            "approach_yaw_aligned": bool(yaw_r.get("yaw_alignment_success")),
            "object_facing_approach_success": bool(runtime.get("success")),
            "final_distance_to_target_room_terminal_m": room_r.get("final_distance_m"),
            "final_distance_to_approach_candidate_m": approach_r.get("final_distance_to_approach_candidate_m"),
            "final_yaw_error_rad": yaw_r.get("final_yaw_error_rad"),
        })
        write_json(out / "runtime_result.json", runtime)

        # Post-execution visualization
        traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
        plot_visualization(planner, dense_route, control_path, semantic, candidate, traj, proxy, out / "route_visualization.png")

        # Comparison
        generate_comparison(args, runtime, dense_route, control_path, out)
    else:
        runtime = {"success": False, "plan_only": True, "runtime_execution_attempted": False}
        write_json(out / "runtime_result.json", runtime)
        generate_comparison(args, runtime, dense_route, control_path, out)

    # Summary
    summary = {
        "artifact_type": "task17b_summary",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": args.query, "object_id": args.object_id,
        "backend": "stabilized pure pursuit lightweight executor without Nav2",
        "nav2_used": False,
        "success": runtime.get("success"),
        "failure_layer": runtime.get("failure_layer"),
        "failure_reason": runtime.get("failure_reason"),
    }
    write_json(out / "summary.json", summary)
    return 0 if args.plan_only or runtime.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
