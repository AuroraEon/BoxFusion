#!/usr/bin/env python3
"""Build a dense occupancy-grid-valid executable route from semantic anchors."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from collections import Counter, deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from scene_runtime_common import load_nav_map, now_iso, read_json, write_json, yaw_between


FREE_THRESHOLD = 250


def xy_to_rc(x: float, y: float, resolution: float, origin: tuple[float, float]) -> tuple[int, int]:
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def rc_to_xy(row: int, col: int, resolution: float, origin: tuple[float, float]) -> tuple[float, float]:
    return origin[0] + col * resolution, origin[1] + row * resolution


def in_bounds(grid: np.ndarray, rc: tuple[int, int]) -> bool:
    row, col = rc
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]


def is_free(grid: np.ndarray, rc: tuple[int, int]) -> bool:
    return in_bounds(grid, rc) and int(grid[rc[0], rc[1]]) >= FREE_THRESHOLD


def nearest_free_index(
    grid: np.ndarray,
    x: float,
    y: float,
    resolution: float,
    origin: tuple[float, float],
    max_radius_m: float,
) -> tuple[tuple[int, int], dict[str, Any]]:
    start = xy_to_rc(x, y, resolution, origin)
    queue = deque([start])
    seen = {start}
    max_radius_cells = max(1, int(math.ceil(max_radius_m / resolution)))
    while queue:
        row, col = queue.popleft()
        if max(abs(row - start[0]), abs(col - start[1])) > max_radius_cells:
            continue
        if is_free(grid, (row, col)):
            sx, sy = rc_to_xy(row, col, resolution, origin)
            return (row, col), {
                "requested_rc": list(start),
                "snapped_rc": [row, col],
                "snap_distance_m": round(math.hypot(sx - x, sy - y), 6),
                "requested_in_bounds": in_bounds(grid, start),
                "requested_occupancy_value": int(grid[start[0], start[1]]) if in_bounds(grid, start) else None,
            }
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nxt = (row + dr, col + dc)
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
    raise RuntimeError(f"no free cell found within {max_radius_m:.2f} m of ({x:.3f}, {y:.3f})")


def astar(grid: np.ndarray, start_rc: tuple[int, int], goal_rc: tuple[int, int], wall_clearance_cells: int) -> list[tuple[int, int]]:
    if start_rc == goal_rc:
        return [start_rc]
    free_mask = (grid >= FREE_THRESHOLD).astype(np.uint8)
    dist_from_wall = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 1.414),
        (-1, 1, 1.414),
        (1, -1, 1.414),
        (1, 1, 1.414),
    ]
    heap: list[tuple[float, tuple[int, int]]] = [(0.0, start_rc)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start_rc: 0.0}
    while heap:
        _priority, current = heapq.heappop(heap)
        if current == goal_rc:
            path = [current]
            while path[-1] in came:
                path.append(came[path[-1]])
            return list(reversed(path))
        row, col = current
        for dr, dc, step in neighbors:
            nxt = (row + dr, col + dc)
            if not is_free(grid, nxt):
                continue
            wall_dist = float(dist_from_wall[nxt[0], nxt[1]])
            wall_penalty = max(0.0, (wall_clearance_cells - wall_dist) * 0.5) if wall_dist < wall_clearance_cells else 0.0
            new_cost = cost[current] + step + wall_penalty
            if new_cost >= cost.get(nxt, 1e18):
                continue
            cost[nxt] = new_cost
            came[nxt] = current
            heapq.heappush(heap, (new_cost + math.hypot(goal_rc[0] - nxt[0], goal_rc[1] - nxt[1]), nxt))
    raise RuntimeError(f"A* found no path from {start_rc} to {goal_rc}")


def simplify_path(path: list[tuple[int, int]], resolution: float, spacing_m: float) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    step = max(1, int(round(spacing_m / resolution)))
    return [path[0], *[path[index] for index in range(step, len(path) - 1, step)], path[-1]]


def path_length(path: list[tuple[int, int]], resolution: float) -> float:
    total = 0.0
    for (ra, ca), (rb, cb) in zip(path, path[1:]):
        total += math.hypot(rb - ra, cb - ca) * resolution
    return total


def line_occupied_count(grid: np.ndarray, start_rc: tuple[int, int], goal_rc: tuple[int, int]) -> int:
    steps = max(abs(goal_rc[0] - start_rc[0]), abs(goal_rc[1] - start_rc[1])) + 1
    count = 0
    for idx in range(steps):
        t = 0.0 if steps == 1 else idx / (steps - 1)
        row = int(round(start_rc[0] + (goal_rc[0] - start_rc[0]) * t))
        col = int(round(start_rc[1] + (goal_rc[1] - start_rc[1]) * t))
        count += 0 if is_free(grid, (row, col)) else 1
    return count


def occupancy_at(grid: np.ndarray, rc: tuple[int, int]) -> int | None:
    return int(grid[rc[0], rc[1]]) if in_bounds(grid, rc) else None


def waypoint_with_grid_metrics(
    waypoint: dict[str, Any],
    grid: np.ndarray,
    clearance_m: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
) -> dict[str, Any]:
    out = dict(waypoint)
    rc = xy_to_rc(float(out["x"]), float(out["y"]), resolution, origin)
    out["grid_row"] = rc[0]
    out["grid_col"] = rc[1]
    out["occupancy_value"] = occupancy_at(grid, rc)
    out["occupancy_value_at_waypoint"] = out["occupancy_value"]
    out["in_map_bounds"] = in_bounds(grid, rc)
    out["is_free"] = is_free(grid, rc)
    out["clearance_m"] = round(float(clearance_m[rc[0], rc[1]]), 6) if in_bounds(grid, rc) else None
    return out


def semantic_gateway_sequence(waypoints: list[dict[str, Any]]) -> list[str]:
    return [str(w["gateway_id"]) for w in waypoints if w.get("gateway_id")]


def semantic_room_sequence(waypoints: list[dict[str, Any]]) -> list[str]:
    rooms: list[str] = []
    for waypoint in waypoints:
        for key in ("from_room", "to_room"):
            room = waypoint.get(key)
            if room and room not in rooms:
                rooms.append(room)
    return rooms


def add_bridge_waypoint(
    rows: list[dict[str, Any]],
    base: dict[str, Any],
    rc: tuple[int, int],
    grid: np.ndarray,
    clearance_m: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    segment_index: int,
) -> None:
    x, y = rc_to_xy(rc[0], rc[1], resolution, origin)
    rows.append(waypoint_with_grid_metrics({
        "scene_id": base.get("scene_id"),
        "floor_id": base.get("floor_id"),
        "route_id": base.get("route_id"),
        "waypoint_index": len(rows),
        "x": round(x, 4),
        "y": round(y, 4),
        "yaw": 0.0,
        "source": "bridge",
        "from_room": base.get("from_room"),
        "to_room": base.get("to_room"),
        "gateway_id": None,
        "pair_key": base.get("pair_key"),
        "segment_index": segment_index,
        "reason": {"type": "occupancy_grid_astar_bridge_between_semantic_anchors"},
    }, grid, clearance_m, resolution, origin))


def append_semantic_anchor(
    rows: list[dict[str, Any]],
    anchor: dict[str, Any],
    snapped_rc: tuple[int, int],
    snap_meta: dict[str, Any],
    grid: np.ndarray,
    clearance_m: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
) -> None:
    x, y = rc_to_xy(snapped_rc[0], snapped_rc[1], resolution, origin)
    out = dict(anchor)
    out["waypoint_index"] = len(rows)
    out["x"] = round(x, 4)
    out["y"] = round(y, 4)
    out["semantic_requested_x"] = anchor.get("x")
    out["semantic_requested_y"] = anchor.get("y")
    out["nearest_free_snap"] = snap_meta
    rows.append(waypoint_with_grid_metrics(out, grid, clearance_m, resolution, origin))


def build_executable_route(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    stage_output = args.stage_output_dir.resolve()
    map_yaml = args.map_yaml.resolve()
    semantic_path = args.semantic_waypoints_json.resolve()
    semantic_payload = read_json(semantic_path)
    semantic_waypoints = list(semantic_payload.get("waypoints") or [])
    if len(semantic_waypoints) < 2:
        raise ValueError("semantic waypoint payload must contain at least two waypoints")

    grid, resolution, origin, map_meta = load_nav_map(map_yaml)
    free_mask = (grid >= FREE_THRESHOLD).astype(np.uint8)
    clearance_m = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5) * resolution

    output_rows: list[dict[str, Any]] = []
    segment_diagnostics: list[dict[str, Any]] = []
    snapped: list[tuple[tuple[int, int], dict[str, Any]]] = []
    for anchor in semantic_waypoints:
        snapped.append(nearest_free_index(grid, float(anchor["x"]), float(anchor["y"]), resolution, origin, args.max_snap_radius_m))

    append_semantic_anchor(output_rows, semantic_waypoints[0], snapped[0][0], snapped[0][1], grid, clearance_m, resolution, origin)
    for segment_index, (start, target) in enumerate(zip(semantic_waypoints, semantic_waypoints[1:])):
        start_rc, start_snap = snapped[segment_index]
        target_rc, target_snap = snapped[segment_index + 1]
        raw_path = astar(grid, start_rc, target_rc, args.wall_clearance_cells)
        sparse_path = simplify_path(raw_path, resolution, args.spacing_m)
        bridge_count = max(0, len(sparse_path) - 2)
        source_pair = f"{start.get('source')}->{target.get('source')}"
        segment_base = {
            "scene_id": semantic_payload.get("scene_id") or start.get("scene_id"),
            "floor_id": args.floor_id or semantic_payload.get("floor_id") or start.get("floor_id"),
            "route_id": semantic_payload.get("route_id") or start.get("route_id"),
            "from_room": start.get("from_room") or target.get("from_room"),
            "to_room": target.get("to_room") or start.get("to_room"),
            "pair_key": target.get("pair_key") or start.get("pair_key"),
        }
        for rc in sparse_path[1:-1]:
            add_bridge_waypoint(output_rows, segment_base, rc, grid, clearance_m, resolution, origin, segment_index)
        append_semantic_anchor(output_rows, target, target_rc, target_snap, grid, clearance_m, resolution, origin)
        straight_length = math.hypot(target_rc[0] - start_rc[0], target_rc[1] - start_rc[1]) * resolution
        segment_waypoints = output_rows[-(bridge_count + 1):]
        segment_diagnostics.append({
            "segment_index": segment_index,
            "start_waypoint_index": int(start.get("waypoint_index", segment_index)),
            "target_waypoint_index": int(target.get("waypoint_index", segment_index + 1)),
            "start_source": start.get("source"),
            "target_source": target.get("source"),
            "start_room": start.get("from_room") or start.get("to_room"),
            "target_room": target.get("to_room") or target.get("from_room"),
            "start_gateway_id": start.get("gateway_id"),
            "target_gateway_id": target.get("gateway_id"),
            "source_pair": source_pair,
            "a_star_success": True,
            "start_snap": start_snap,
            "target_snap": target_snap,
            "raw_grid_path_length": len(raw_path),
            "simplified_waypoint_count": len(sparse_path),
            "bridge_waypoint_count": bridge_count,
            "path_length_m": round(path_length(raw_path, resolution), 6),
            "straight_length_m": round(straight_length, 6),
            "detour_ratio": round(path_length(raw_path, resolution) / straight_length, 6) if straight_length > 1e-9 else 1.0,
            "straight_line_occupied_cell_count": line_occupied_count(grid, start_rc, target_rc),
            "occupied_waypoint_count": len([w for w in segment_waypoints if not w.get("is_free")]),
            "min_clearance_m": round(min(float(w["clearance_m"]) for w in segment_waypoints if w.get("clearance_m") is not None), 6) if segment_waypoints else None,
            "contains_gateway_anchor": bool(start.get("gateway_id") or target.get("gateway_id")),
            "gateway_id": target.get("gateway_id") or start.get("gateway_id"),
        })

    for idx, waypoint in enumerate(output_rows[:-1]):
        waypoint["yaw"] = yaw_between(waypoint, output_rows[idx + 1])
    if output_rows:
        output_rows[-1]["yaw"] = output_rows[-2].get("yaw") if len(output_rows) > 1 else 0.0

    source_counts = dict(Counter(str(w.get("source")) for w in output_rows))
    occupied = [w for w in output_rows if not w.get("is_free")]
    out_of_bounds = [w for w in output_rows if not w.get("in_map_bounds")]
    route_gateway_sequence = semantic_gateway_sequence(output_rows)
    semantic_gateways = semantic_gateway_sequence(semantic_waypoints)
    diagnostics = {
        "artifact_type": "scene_executable_route_diagnostics",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "floor_id": args.floor_id or semantic_payload.get("floor_id"),
        "map_yaml": map_yaml.as_posix(),
        "map_meta": map_meta,
        "semantic_waypoints_json": semantic_path.as_posix(),
        "semantic_waypoint_count": len(semantic_waypoints),
        "executable_waypoint_count": len(output_rows),
        "source_counts": source_counts,
        "semantic_gateway_sequence": semantic_gateways,
        "executable_gateway_sequence": route_gateway_sequence,
        "gateway_sequence_preserved": route_gateway_sequence == semantic_gateways,
        "occupied_output_waypoint_count": len(occupied),
        "out_of_bounds_waypoint_count": len(out_of_bounds),
        "min_clearance_m": round(min(float(w["clearance_m"]) for w in output_rows if w.get("clearance_m") is not None), 6),
        "spacing_m": args.spacing_m,
        "wall_clearance_cells": args.wall_clearance_cells,
        "max_snap_radius_m": args.max_snap_radius_m,
        "segments": segment_diagnostics,
        "passed": len(output_rows) > len(semantic_waypoints) and not occupied and not out_of_bounds and route_gateway_sequence == semantic_gateways,
        "claim": "occupancy-grid-valid only; not a full robot collision-free guarantee",
    }
    payload = {
        "artifact_type": "scene_executable_route_waypoints",
        "version": "v0_1",
        "created_utc": now_iso(),
        "scene_id": semantic_payload.get("scene_id"),
        "floor_id": args.floor_id or semantic_payload.get("floor_id"),
        "route_id": semantic_payload.get("route_id"),
        "coordinate_frame": semantic_payload.get("coordinate_frame"),
        "source": "semantic_constraints_expanded_by_occupancy_grid_astar",
        "map_yaml": map_yaml.as_posix(),
        "semantic_waypoints_json": semantic_path.as_posix(),
        "room_sequence": semantic_payload.get("room_sequence") or semantic_room_sequence(semantic_waypoints),
        "gateway_sequence": semantic_gateways,
        "spacing_m": args.spacing_m,
        "waypoints": output_rows,
        "diagnostics_path": args.diagnostics_json.as_posix() if args.diagnostics_json else None,
    }
    return payload, diagnostics


def write_report(path: Path, route: dict[str, Any], diagnostics: dict[str, Any]) -> None:
    lines = [
        "# Scene Executable Route Builder",
        "",
        f"Passed: `{diagnostics['passed']}`",
        f"Semantic waypoints: `{diagnostics['semantic_waypoint_count']}`",
        f"Executable waypoints: `{diagnostics['executable_waypoint_count']}`",
        f"Source counts: `{json.dumps(diagnostics['source_counts'], sort_keys=True)}`",
        f"Gateway sequence preserved: `{diagnostics['gateway_sequence_preserved']}`",
        f"Gateways: `{' -> '.join(diagnostics['executable_gateway_sequence'])}`",
        f"Occupied output waypoints: `{diagnostics['occupied_output_waypoint_count']}`",
        f"Out-of-bounds output waypoints: `{diagnostics['out_of_bounds_waypoint_count']}`",
        f"Minimum clearance: `{diagnostics['min_clearance_m']}` m",
        "",
        "This route is occupancy-grid-valid evidence, not a full collision-free navigation guarantee.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--semantic-waypoints-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--diagnostics-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--spacing-m", type=float, default=0.25)
    parser.add_argument("--max-snap-radius-m", type=float, default=1.0)
    parser.add_argument("--wall-clearance-cells", type=int, default=5)
    args = parser.parse_args()

    route, diagnostics = build_executable_route(args)
    write_json(args.output_json, route)
    write_json(args.diagnostics_json, diagnostics)
    if args.output_md:
        write_report(args.output_md, route, diagnostics)
    print(json.dumps(diagnostics, indent=2, sort_keys=True))
    return 0 if diagnostics.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
