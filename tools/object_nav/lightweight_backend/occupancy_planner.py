"""Occupancy-grid-based planner: map loading, A*, clearance, validation, line-of-sight."""

from __future__ import annotations

import heapq
import math
from collections import deque
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from .schemas import distance


def _load_nav_map(map_yaml: Path):
    """Load navigation map - deferred import to avoid circular deps."""
    import sys
    runtime_dir = str(Path(__file__).resolve().parents[2] / "stage1_runtime")
    if runtime_dir not in sys.path:
        sys.path.insert(0, runtime_dir)
    from scene_runtime_common import load_nav_map
    return load_nav_map(map_yaml)


class OccupancyPlanner:
    """Inflated-occupancy-grid planner with A* and validation."""

    def __init__(self, map_yaml: Path, inflation_radius_m: float, spacing_m: float) -> None:
        self.map_yaml = map_yaml
        self.grid, self.resolution, self.origin, self.meta = _load_nav_map(map_yaml)
        self.free_mask = (self.grid >= 250).astype(np.uint8)
        self.clearance = cv2.distanceTransform(self.free_mask, cv2.DIST_L2, 5) * self.resolution
        self.inflation_radius_m = inflation_radius_m
        self.spacing_m = spacing_m

    def rc(self, x: float, y: float) -> tuple[int, int]:
        return (int(round((y - self.origin[1]) / self.resolution)),
                int(round((x - self.origin[0]) / self.resolution)))

    def xy(self, rc: tuple[int, int]) -> tuple[float, float]:
        return (self.origin[0] + rc[1] * self.resolution,
                self.origin[1] + rc[0] * self.resolution)

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
                return cur, {"requested_world_xy": [x, y],
                             "snapped_world_xy": [round(snapped[0], 6), round(snapped[1], 6)],
                             "snap_distance_m": round(math.hypot(snapped[0] - x, snapped[1] - y), 6)}
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
                     (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                     (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))
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
        invalid_segments: list[dict[str, Any]] = []
        for segment_index, (l, r) in enumerate(zip(points, points[1:])):
            segment_cells = self.sampled_cells(l, r)
            tested.extend(segment_cells)
            segment_invalid = [
                rc for rc in segment_cells
                if not (self.traversable(rc) if require_inflated else self.free(rc))
            ]
            if segment_invalid:
                invalid_segments.append({
                    "segment_index": segment_index,
                    "start": {"x": float(l["x"]), "y": float(l["y"])},
                    "end": {"x": float(r["x"]), "y": float(r["y"])},
                    "invalid_sample_count": len(segment_invalid),
                    "first_invalid_world_xy": [round(v, 6) for v in self.xy(segment_invalid[0])],
                })
        if len(points) == 1:
            tested.append(self.rc(float(points[0]["x"]), float(points[0]["y"])))
        invalid = [
            {"grid_row": rc[0], "grid_col": rc[1],
             "world_xy": [round(v, 6) for v in self.xy(rc)],
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
            "invalid_segment_count": len(invalid_segments),
            "invalid_segments": invalid_segments,
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
        from .schemas import route_length as _rl
        segment = {
            "segment": label, "a_star_success": True, "inflation_radius_m": self.inflation_radius_m,
            "start_snap": start_snap, "goal_snap": goal_snap,
            "raw_grid_point_count": len(raw), "executable_waypoint_count": len(points),
            "path_length_m": round(_rl(points), 6), **validation,
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
