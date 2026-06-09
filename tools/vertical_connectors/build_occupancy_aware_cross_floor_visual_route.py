#!/usr/bin/env python3
"""Build task24g2 occupancy-aware visual-kinematic cross-floor route.

Same-floor portions are backfilled with A* over the existing stable occupancy
maps. The stair portion remains the task24f graph-level 3D connector centerline.
This is visual-kinematic playback evidence only.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image


REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
TASK_ROOT = REPO_ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V"
DEFAULT_ROUTE_CONTRACT = (
    TASK_ROOT
    / "tasks/task24f_visual_kinematic_proxy_cross_floor_traversal_feasibility_and_plan/"
    / "cross_floor_3d_route_contract_v0_1.json"
)
DEFAULT_FLOOR_1_MAP = TASK_ROOT / "clean_rerun/maps/floor_1/stage1_floor_1_stable_occupancy_map.yaml"
DEFAULT_FLOOR_2_MAP = TASK_ROOT / "clean_rerun/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
DEFAULT_OUTPUT_DIR = (
    TASK_ROOT / "tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal"
)
ROUTE_NAME = "planned_occupancy_aware_3d_route_v0_1.json"
CONNECTOR_NODES = [
    "vt_1_centerline_n000",
    "vt_1_centerline_n001",
    "vt_1_centerline_n002",
    "vt_1_centerline_n003",
    "vt_1_centerline_n004",
]
CLAIM_BOUNDARY = {
    "claim_boundary": "visual_kinematic_proxy_only",
    "topological_vertical_transition_only": True,
    "occupancy_aware_same_floor_visual_playback": True,
    "physical_stair_climbing_supported": False,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "real_quadruped_stair_locomotion_supported": False,
    "nav2_execution": False,
    "amcl_localization": False,
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def parse_scalar(text: str) -> Any:
    text = text.strip()
    if text.startswith("[") and text.endswith("]"):
        return [parse_scalar(part) for part in text[1:-1].split(",")]
    if text in {"true", "True"}:
        return True
    if text in {"false", "False"}:
        return False
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text.strip("\"'")


def load_simple_yaml(path: Path) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        clean = line.split("#", 1)[0].strip()
        if not clean or ":" not in clean:
            continue
        key, value = clean.split(":", 1)
        data[key.strip()] = parse_scalar(value)
    return data


def waypoint_xyz(raw: dict[str, Any]) -> tuple[float, float, float]:
    values = raw.get("position_xyz") or raw.get("xyz") or raw.get("position") or [0.0, 0.0, 0.0]
    return float(values[0]), float(values[1]), float(values[2])


def point_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.sqrt(
        (float(a["x"]) - float(b["x"])) ** 2
        + (float(a["y"]) - float(b["y"])) ** 2
        + (float(a["z"]) - float(b["z"])) ** 2
    )


class StableMap:
    def __init__(self, floor_id: str, yaml_path: Path, inflation_radius_m: float) -> None:
        self.floor_id = floor_id
        self.yaml_path = yaml_path
        self.meta = load_simple_yaml(yaml_path)
        self.resolution = float(self.meta["resolution"])
        origin = self.meta.get("origin") or [0.0, 0.0, 0.0]
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]) if len(origin) > 2 else 0.0)
        image_path = Path(str(self.meta["image"]))
        self.pgm_path = image_path if image_path.is_absolute() else yaml_path.parent / image_path
        self.npz_path = yaml_path.with_suffix(".npz")
        self.inflation_radius_m = inflation_radius_m
        self.occupancy, self.free_mask = self._load_grid()
        self.clearance_m = cv2.distanceTransform(self.free_mask.astype(np.uint8), cv2.DIST_L2, 5) * self.resolution
        self.traversable_mask = self.free_mask & (self.clearance_m >= self.inflation_radius_m)

    def _load_grid(self) -> tuple[np.ndarray, np.ndarray]:
        if self.npz_path.exists():
            payload = np.load(self.npz_path)
            occupancy = np.asarray(payload["occupancy"], dtype=np.int16)
            free = np.asarray(payload.get("free", occupancy == 0), dtype=bool)
            occupied = np.asarray(payload.get("occupied", occupancy > 0), dtype=bool)
            free_mask = free & ~occupied & (occupancy == 0)
            return occupancy, free_mask
        image = np.asarray(Image.open(self.pgm_path).convert("L"))
        occupied_thresh = float(self.meta.get("occupied_thresh", 0.65))
        free_thresh = float(self.meta.get("free_thresh", 0.196))
        negate = int(self.meta.get("negate", 0))
        occ_prob = image.astype(np.float32) / 255.0
        if not negate:
            occ_prob = 1.0 - occ_prob
        occupied = occ_prob >= occupied_thresh
        free = occ_prob <= free_thresh
        occupancy = np.where(occupied, 100, np.where(free, 0, -1)).astype(np.int16)
        return occupancy, free

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(round((y - self.origin[1]) / self.resolution)),
            int(round((x - self.origin[0]) / self.resolution)),
        )

    def grid_to_world(self, rc: tuple[int, int]) -> tuple[float, float]:
        return (
            self.origin[0] + rc[1] * self.resolution,
            self.origin[1] + rc[0] * self.resolution,
        )

    def in_bounds(self, rc: tuple[int, int]) -> bool:
        return 0 <= rc[0] < self.occupancy.shape[0] and 0 <= rc[1] < self.occupancy.shape[1]

    def is_free(self, rc: tuple[int, int]) -> bool:
        return bool(self.in_bounds(rc) and self.free_mask[rc[0], rc[1]])

    def is_traversable(self, rc: tuple[int, int]) -> bool:
        return bool(self.in_bounds(rc) and self.traversable_mask[rc[0], rc[1]])

    def sample(self, rc: tuple[int, int]) -> dict[str, Any]:
        return {
            "grid_row": rc[0],
            "grid_col": rc[1],
            "in_bounds": self.in_bounds(rc),
            "occupancy_value": int(self.occupancy[rc[0], rc[1]]) if self.in_bounds(rc) else None,
            "is_free": self.is_free(rc),
            "inflated_traversable": self.is_traversable(rc),
            "clearance_m": round(float(self.clearance_m[rc[0], rc[1]]), 6) if self.in_bounds(rc) else None,
        }

    def nearest_traversable(
        self,
        point_xyz: tuple[float, float, float],
        *,
        max_radius_m: float,
        node_id: str,
        waypoint_id: str,
    ) -> tuple[tuple[int, int], dict[str, Any]]:
        x, y, _z = point_xyz
        requested = self.world_to_grid(x, y)
        queue = deque([requested])
        seen = {requested}
        max_cells = max(1, int(math.ceil(max_radius_m / self.resolution)))
        while queue:
            current = queue.popleft()
            if max(abs(current[0] - requested[0]), abs(current[1] - requested[1])) > max_cells:
                continue
            if self.is_traversable(current):
                sx, sy = self.grid_to_world(current)
                meta = {
                    "node_id": node_id,
                    "waypoint_id": waypoint_id,
                    "original_world_xyz": [round(x, 6), round(y, 6), round(float(point_xyz[2]), 6)],
                    "snapped_world_xyz": [round(sx, 6), round(sy, 6), round(float(point_xyz[2]), 6)],
                    "requested_grid_rc": list(requested),
                    "snapped_grid_rc": list(current),
                    "snap_distance_m": round(math.hypot(sx - x, sy - y), 6),
                    "requested_sample": self.sample(requested),
                    "snapped_sample": self.sample(current),
                    "reason": "requested_cell_not_inflated_traversable" if not self.is_traversable(requested) else "grid_center_alignment",
                }
                return current, meta
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nxt = (current[0] + dr, current[1] + dc)
                    if nxt not in seen:
                        seen.add(nxt)
                        queue.append(nxt)
        raise RuntimeError(
            f"no inflated free cell within {max_radius_m:.2f} m of {node_id} on {self.floor_id}"
        )

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        if start == goal:
            return [start]
        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]
        heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        cost = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        while heap:
            _priority, current = heapq.heappop(heap)
            if current == goal:
                path = [current]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                return list(reversed(path))
            for dr, dc, step in neighbors:
                nxt = (current[0] + dr, current[1] + dc)
                if not self.is_traversable(nxt):
                    continue
                clearance = float(self.clearance_m[nxt[0], nxt[1]])
                clearance_penalty = max(0.0, (self.inflation_radius_m * 1.5) - clearance) * 4.0
                proposed = cost[current] + step + clearance_penalty
                if proposed >= cost.get(nxt, float("inf")):
                    continue
                cost[nxt] = proposed
                came_from[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(heap, (proposed + heuristic, nxt))
        raise RuntimeError(f"A* found no path from {start} to {goal} on {self.floor_id}")

    def validate_world_polyline(self, points: list[dict[str, Any]], *, require_inflated: bool = True) -> dict[str, Any]:
        tested: list[tuple[int, int]] = []
        invalid_segments: list[dict[str, Any]] = []
        for segment_index, (left, right) in enumerate(zip(points, points[1:])):
            length = math.hypot(float(right["x"]) - float(left["x"]), float(right["y"]) - float(left["y"]))
            steps = max(1, int(math.ceil(length / (self.resolution * 0.45))))
            segment_cells: list[tuple[int, int]] = []
            for i in range(steps + 1):
                alpha = i / steps
                rc = self.world_to_grid(
                    float(left["x"]) + (float(right["x"]) - float(left["x"])) * alpha,
                    float(left["y"]) + (float(right["y"]) - float(left["y"])) * alpha,
                )
                segment_cells.append(rc)
            tested.extend(segment_cells)
            bad = [
                rc for rc in segment_cells
                if not (self.is_traversable(rc) if require_inflated else self.is_free(rc))
            ]
            if bad:
                invalid_segments.append(
                    {
                        "segment_index": segment_index,
                        "start_xyz": [left["x"], left["y"], left["z"]],
                        "end_xyz": [right["x"], right["y"], right["z"]],
                        "invalid_sample_count": len(bad),
                        "first_invalid_sample": self.sample(bad[0]),
                    }
                )
        if len(points) == 1:
            tested.append(self.world_to_grid(float(points[0]["x"]), float(points[0]["y"])))
        invalid = [
            {"grid_rc": list(rc), **self.sample(rc)}
            for rc in tested
            if not (self.is_traversable(rc) if require_inflated else self.is_free(rc))
        ]
        clearances = [
            float(self.clearance_m[rc[0], rc[1]])
            for rc in tested
            if self.in_bounds(rc) and self.is_free(rc)
        ]
        return {
            "wall_obstacle_crossing_check_passed": not invalid,
            "requires_inflated_clearance": require_inflated,
            "tested_sample_count": len(tested),
            "invalid_sample_count": len(invalid),
            "invalid_samples": invalid[:30],
            "invalid_segment_count": len(invalid_segments),
            "invalid_segments": invalid_segments,
            "minimum_clearance_m": round(min(clearances), 6) if clearances else None,
        }

    def path_length(self, cells: list[tuple[int, int]]) -> float:
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) * self.resolution
            for a, b in zip(cells, cells[1:])
        )


def semantic_waypoint(raw: dict[str, Any], index: int) -> dict[str, Any]:
    x, y, z = waypoint_xyz(raw)
    node_id = str(raw.get("node_id") or raw.get("room_id") or raw.get("waypoint_id") or f"wp_{index:03d}")
    return {
        "semantic_index": index,
        "waypoint_id": str(raw.get("waypoint_id") or f"wp_{index:03d}"),
        "node_id": node_id,
        "waypoint_kind": raw.get("waypoint_kind"),
        "floor_id": raw.get("floor_id"),
        "room_id": raw.get("room_id"),
        "connector_id": raw.get("connector_id"),
        "x": x,
        "y": y,
        "z": z,
        "source_position_xyz": [x, y, z],
        "physical_execution_supported": False,
    }


def build_segment_waypoints(
    *,
    floor_map: StableMap,
    segment_id: str,
    source: dict[str, Any],
    target: dict[str, Any],
    max_snap_radius_m: float,
    spacing_m: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source_rc, source_snap = floor_map.nearest_traversable(
        (float(source["x"]), float(source["y"]), float(source["z"])),
        max_radius_m=max_snap_radius_m,
        node_id=source["node_id"],
        waypoint_id=source["waypoint_id"],
    )
    target_rc, target_snap = floor_map.nearest_traversable(
        (float(target["x"]), float(target["y"]), float(target["z"])),
        max_radius_m=max_snap_radius_m,
        node_id=target["node_id"],
        waypoint_id=target["waypoint_id"],
    )
    raw_cells = floor_map.astar(source_rc, target_rc)
    stride = max(1, int(round(spacing_m / floor_map.resolution)))
    simplified_cells = [raw_cells[0], *raw_cells[stride:-1:stride], raw_cells[-1]]

    def cells_to_points(cells: list[tuple[int, int]], simplification: str) -> list[dict[str, Any]]:
        points: list[dict[str, Any]] = []
        denom = max(1, len(cells) - 1)
        for i, rc in enumerate(cells):
            x, y = floor_map.grid_to_world(rc)
            alpha = i / denom
            z = float(source["z"]) + (float(target["z"]) - float(source["z"])) * alpha
            points.append(
                {
                    "x": round(x, 6),
                    "y": round(y, 6),
                    "z": round(z, 6),
                    "floor_id": source["floor_id"],
                    "source_segment_id": segment_id,
                    "source_node_id": source["node_id"],
                    "target_node_id": target["node_id"],
                    "pose_source": "astar_stable_occupancy_map",
                    "grid_row": rc[0],
                    "grid_col": rc[1],
                    "path_cell_index": i,
                    "path_simplification": simplification,
                }
            )
        return points

    points = cells_to_points(simplified_cells, "stride_subsampled_astar_cells")
    validation = floor_map.validate_world_polyline(points, require_inflated=True)
    smoothing_applied = True
    smoothing_validated = validation["wall_obstacle_crossing_check_passed"]
    if not smoothing_validated:
        points = cells_to_points(raw_cells, "raw_astar_cells")
        validation = floor_map.validate_world_polyline(points, require_inflated=True)
        smoothing_applied = False
        smoothing_validated = None
    straight_check = floor_map.validate_world_polyline(
        [
            {"x": source["x"], "y": source["y"], "z": source["z"]},
            {"x": target["x"], "y": target["y"], "z": target["z"]},
        ],
        require_inflated=True,
    )
    segment = {
        "segment_id": segment_id,
        "floor_id": source["floor_id"],
        "source_waypoint_id": source["waypoint_id"],
        "source_node_id": source["node_id"],
        "target_waypoint_id": target["waypoint_id"],
        "target_node_id": target["node_id"],
        "planner_type": "astar_stable_occupancy_map",
        "map_yaml_path": str(floor_map.yaml_path),
        "map_pgm_path": str(floor_map.pgm_path),
        "astar_connectivity": "8_connected",
        "inflation_radius_m": floor_map.inflation_radius_m,
        "raw_grid_waypoint_count": len(raw_cells),
        "waypoint_count": len(points),
        "path_length_m": round(floor_map.path_length(raw_cells), 6),
        "minimum_clearance_estimate_m": validation["minimum_clearance_m"],
        "wall_obstacle_crossing_check_result": validation,
        "z_policy": "constant_same_floor_reference_z",
        "source_snap": source_snap,
        "target_snap": target_snap,
        "smoothing_or_simplification_applied": smoothing_applied,
        "smoothing_or_simplification_validated": smoothing_validated,
        "straight_topology_interpolation_validation": straight_check,
        "same_floor_route_is_not_only_straight_topology_interpolation": len(points) > 2,
        "status": "success" if validation["wall_obstacle_crossing_check_passed"] else "failed",
    }
    return points, segment


def stair_segment(semantic: dict[str, dict[str, Any]], transition_edge_id: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    points: list[dict[str, Any]] = []
    for i, node_id in enumerate(CONNECTOR_NODES):
        wp = semantic[node_id]
        points.append(
            {
                "x": round(float(wp["x"]), 6),
                "y": round(float(wp["y"]), 6),
                "z": round(float(wp["z"]), 6),
                "floor_id": wp.get("floor_id"),
                "source_segment_id": "seg_002_stair_vt_1_centerline",
                "node_id": node_id,
                "waypoint_id": wp["waypoint_id"],
                "connector_id": "vc_vt_1",
                "pose_source": "stair_connector_centerline",
                "path_cell_index": i,
            }
        )
    length = sum(point_distance(a, b) for a, b in zip(points, points[1:]))
    z_values = [p["z"] for p in points]
    segment = {
        "segment_id": "seg_002_stair_vt_1_centerline",
        "floor_id": "floor_1_to_floor_2",
        "source_waypoint_id": semantic["vt_1_centerline_n000"]["waypoint_id"],
        "source_node_id": "vt_1_centerline_n000",
        "target_waypoint_id": semantic["vt_1_centerline_n004"]["waypoint_id"],
        "target_node_id": "vt_1_centerline_n004",
        "planner_type": "stair_connector_centerline",
        "map_yaml_path": None,
        "path_length_m": round(length, 6),
        "waypoint_count": len(points),
        "minimum_clearance_estimate_m": None,
        "wall_obstacle_crossing_check_result": "not_applicable_graph_level_3d_connector_centerline",
        "z_policy": "task24f_connector_centerline_real_z_values",
        "connector_id": "vc_vt_1",
        "centerline_nodes": CONNECTOR_NODES,
        "transition_edge_id": transition_edge_id,
        "z_monotonic_non_decreasing": all(b >= a for a, b in zip(z_values, z_values[1:])),
        "z_delta_m": round(z_values[-1] - z_values[0], 6),
        "physical_stair_climbing_supported": False,
        "status": "success",
    }
    return points, segment


def append_without_duplicate(route: list[dict[str, Any]], points: list[dict[str, Any]]) -> None:
    for point in points:
        if route and all(abs(float(route[-1][axis]) - float(point[axis])) <= 1.0e-6 for axis in ("x", "y", "z")):
            continue
        out = dict(point)
        out["route_waypoint_index"] = len(route)
        route.append(out)


def checkpoint_mapping(semantic_waypoints: list[dict[str, Any]], dense_route: list[dict[str, Any]]) -> list[dict[str, Any]]:
    mapping: list[dict[str, Any]] = []
    for wp in semantic_waypoints:
        if dense_route:
            best = min(dense_route, key=lambda pt: point_distance(wp, pt))
            err = point_distance(wp, best)
            idx = best["route_waypoint_index"]
        else:
            best = None
            err = float("inf")
            idx = None
        mapping.append(
            {
                "node_id": wp["node_id"],
                "waypoint_id": wp["waypoint_id"],
                "floor_id": wp.get("floor_id"),
                "room_id": wp.get("room_id"),
                "connector_id": wp.get("connector_id"),
                "nearest_dense_route_waypoint_index": idx,
                "nearest_error_m": None if not math.isfinite(err) else round(err, 6),
                "semantic_xyz": [round(float(wp["x"]), 6), round(float(wp["y"]), 6), round(float(wp["z"]), 6)],
                "nearest_dense_xyz": None if best is None else [best["x"], best["y"], best["z"]],
            }
        )
    return mapping


def checkpoint_passes(mapping: list[dict[str, Any]], node_id: str, tolerance_m: float) -> bool:
    for item in mapping:
        err = item.get("nearest_error_m")
        if item.get("node_id") == node_id and err is not None and float(err) <= tolerance_m:
            return True
    return False


def build(args: argparse.Namespace) -> dict[str, Any]:
    contract = read_json(args.route_contract)
    raw_waypoints = contract.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("task24f route contract does not contain waypoints")
    semantic_waypoints = [semantic_waypoint(raw, i) for i, raw in enumerate(raw_waypoints)]
    semantic_by_node = {wp["node_id"]: wp for wp in semantic_waypoints}
    missing = [node for node in ["room_2", "room_3", *CONNECTOR_NODES, "room_7", "room_13", "room_14"] if node not in semantic_by_node]
    if missing:
        raise ValueError(f"route contract is missing required nodes: {missing}")

    maps = {
        "floor_1": StableMap("floor_1", args.floor_1_map_yaml, args.inflation_radius_m),
        "floor_2": StableMap("floor_2", args.floor_2_map_yaml, args.inflation_radius_m),
    }
    same_floor_specs = [
        ("seg_000_floor_1_room_2_to_room_3", "floor_1", "room_2", "room_3"),
        ("seg_001_floor_1_room_3_to_vt_1_centerline_n000", "floor_1", "room_3", "vt_1_centerline_n000"),
        ("seg_003_floor_2_vt_1_centerline_n004_to_room_7", "floor_2", "vt_1_centerline_n004", "room_7"),
        ("seg_004_floor_2_room_7_to_room_13", "floor_2", "room_7", "room_13"),
        ("seg_005_floor_2_room_13_to_room_14", "floor_2", "room_13", "room_14"),
    ]
    dense_route: list[dict[str, Any]] = []
    segments_by_id: dict[str, dict[str, Any]] = {}
    same_floor_segments: list[dict[str, Any]] = []
    failed_segments: list[dict[str, Any]] = []

    for segment_id, floor_id, source_node, target_node in same_floor_specs[:2]:
        try:
            points, segment = build_segment_waypoints(
                floor_map=maps[floor_id],
                segment_id=segment_id,
                source=semantic_by_node[source_node],
                target=semantic_by_node[target_node],
                max_snap_radius_m=args.max_snap_radius_m,
                spacing_m=args.route_spacing_m,
            )
            append_without_duplicate(dense_route, points)
        except Exception as exc:
            segment = {
                "segment_id": segment_id,
                "floor_id": floor_id,
                "source_node_id": source_node,
                "target_node_id": target_node,
                "planner_type": "astar_stable_occupancy_map",
                "map_yaml_path": str(maps[floor_id].yaml_path),
                "status": "failed",
                "error": str(exc),
            }
            failed_segments.append(segment)
        segments_by_id[segment_id] = segment
        same_floor_segments.append(segment)

    stair_points, stair = stair_segment(
        semantic_by_node,
        str(contract.get("transition_edge_id") or (contract.get("transition_edge") or {}).get("edge_id")),
    )
    append_without_duplicate(dense_route, stair_points)
    segments_by_id[stair["segment_id"]] = stair

    for segment_id, floor_id, source_node, target_node in same_floor_specs[2:]:
        try:
            points, segment = build_segment_waypoints(
                floor_map=maps[floor_id],
                segment_id=segment_id,
                source=semantic_by_node[source_node],
                target=semantic_by_node[target_node],
                max_snap_radius_m=args.max_snap_radius_m,
                spacing_m=args.route_spacing_m,
            )
            append_without_duplicate(dense_route, points)
        except Exception as exc:
            segment = {
                "segment_id": segment_id,
                "floor_id": floor_id,
                "source_node_id": source_node,
                "target_node_id": target_node,
                "planner_type": "astar_stable_occupancy_map",
                "map_yaml_path": str(maps[floor_id].yaml_path),
                "status": "failed",
                "error": str(exc),
            }
            failed_segments.append(segment)
        segments_by_id[segment_id] = segment
        same_floor_segments.append(segment)

    for idx, point in enumerate(dense_route):
        point["route_waypoint_index"] = idx

    semantic_map = checkpoint_mapping(semantic_waypoints, dense_route)
    transition_edge_id = str(contract.get("transition_edge_id") or (contract.get("transition_edge") or {}).get("edge_id"))
    route_checks = {
        "route_contract_loaded": True,
        "floor_1_stable_occupancy_map_loaded": args.floor_1_map_yaml.exists(),
        "floor_2_stable_occupancy_map_loaded": args.floor_2_map_yaml.exists(),
        "all_same_floor_astar_segments_succeeded": not failed_segments and all(s.get("status") == "success" for s in same_floor_segments),
        "same_floor_route_is_no_longer_only_straight_topology_node_interpolation": all(
            bool(s.get("same_floor_route_is_not_only_straight_topology_interpolation")) for s in same_floor_segments
        ),
        "stair_connector_segment_remains_vt_1_centerline": stair["centerline_nodes"] == CONNECTOR_NODES,
        "transition_edge_is_vt_1_centerline_e001": transition_edge_id == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": transition_edge_id != "vt_1_centerline_e003",
        "planned_dense_3d_route_starts_at_room_2": bool(semantic_map and semantic_map[0]["node_id"] == "room_2"),
        "planned_dense_3d_route_passes_room_3": checkpoint_passes(semantic_map, "room_3", args.checkpoint_tolerance_m),
        "planned_dense_3d_route_passes_vt_1_centerline_nodes": all(
            checkpoint_passes(semantic_map, node, args.checkpoint_tolerance_m)
            for node in CONNECTOR_NODES
        ),
        "planned_dense_3d_route_passes_room_7_room_13_room_14": all(
            checkpoint_passes(semantic_map, node, args.checkpoint_tolerance_m)
            for node in ["room_7", "room_13", "room_14"]
        ),
        "z_increases_from_floor_1_to_floor_2_along_connector": bool(stair["z_delta_m"] > 0.0 and stair["z_monotonic_non_decreasing"]),
        "no_astar_across_floors": True,
        "no_astar_through_stair_connector": True,
        "no_stage_a_rerun": True,
    }
    classification = (
        "task24g2_route_backfill_astar_segments_succeeded"
        if route_checks["all_same_floor_astar_segments_succeeded"]
        else "task24g2_blocked_by_same_floor_astar_segment"
    )

    route = {
        "artifact_type": "task24g2_planned_occupancy_aware_3d_route_v0_1",
        "created_utc": now_iso(),
        "route_id": "task24g2_room2_room3_vt1_room7_room13_room14_occupancy_aware_visual_3d_v0_1",
        "source_route_contract": str(args.route_contract),
        "source_route_contract_route_id": contract.get("route_id"),
        "map_sources": {
            floor_id: {
                "yaml": str(stable_map.yaml_path),
                "pgm": str(stable_map.pgm_path),
                "npz": str(stable_map.npz_path) if stable_map.npz_path.exists() else None,
                "resolution": stable_map.resolution,
                "origin": list(stable_map.origin),
                "shape_rc": list(stable_map.occupancy.shape),
            }
            for floor_id, stable_map in maps.items()
        },
        "planner_config": {
            "same_floor_planner_type": "astar_stable_occupancy_map",
            "astar_connectivity": "8_connected",
            "inflation_radius_m": args.inflation_radius_m,
            "max_snap_radius_m": args.max_snap_radius_m,
            "route_spacing_m": args.route_spacing_m,
            "occupied_unknown_treatment": "occupied_and_unknown_cells_are_not_traversable",
        },
        "segments": [segments_by_id[key] for key in sorted(segments_by_id)],
        "dense_3d_route": {
            "waypoint_count": len(dense_route),
            "waypoints": dense_route,
        },
        "semantic_checkpoints": semantic_waypoints,
        "semantic_checkpoint_mapping": semantic_map,
        "transition_edge_id": transition_edge_id,
        "transition_edge": contract.get("transition_edge"),
        "claim_boundary": CLAIM_BOUNDARY,
        "classification": classification,
        "route_checks": route_checks,
    }

    return route


def write_validation_artifacts(out: Path, route: dict[str, Any]) -> None:
    same_floor = [segment for segment in route["segments"] if segment.get("planner_type") == "astar_stable_occupancy_map"]
    stair = [segment for segment in route["segments"] if segment.get("planner_type") == "stair_connector_centerline"]
    backfill_validation = {
        "artifact_type": "task24g2_route_backfill_validation",
        "created_utc": now_iso(),
        "classification": route["classification"],
        "route_checks": route["route_checks"],
        "failed_same_floor_segments": [s for s in same_floor if s.get("status") != "success"],
        "claim_boundary": CLAIM_BOUNDARY,
    }
    astar_validation = {
        "artifact_type": "task24g2_astar_segment_validation",
        "created_utc": now_iso(),
        "all_same_floor_astar_segments_succeeded": route["route_checks"]["all_same_floor_astar_segments_succeeded"],
        "segments": same_floor,
    }
    same_floor_validation = {
        "artifact_type": "task24g2_same_floor_path_validation",
        "created_utc": now_iso(),
        "same_floor_route_is_no_longer_only_straight_topology_node_interpolation": route["route_checks"][
            "same_floor_route_is_no_longer_only_straight_topology_node_interpolation"
        ],
        "segments": [
            {
                "segment_id": s.get("segment_id"),
                "source_node_id": s.get("source_node_id"),
                "target_node_id": s.get("target_node_id"),
                "path_length_m": s.get("path_length_m"),
                "waypoint_count": s.get("waypoint_count"),
                "minimum_clearance_estimate_m": s.get("minimum_clearance_estimate_m"),
                "wall_obstacle_crossing_check_result": s.get("wall_obstacle_crossing_check_result"),
                "same_floor_route_is_not_only_straight_topology_interpolation": s.get(
                    "same_floor_route_is_not_only_straight_topology_interpolation"
                ),
                "status": s.get("status"),
            }
            for s in same_floor
        ],
    }
    stair_validation = {
        "artifact_type": "task24g2_stair_connector_segment_validation",
        "created_utc": now_iso(),
        "stair_connector_segment_remains_vt_1_centerline": route["route_checks"]["stair_connector_segment_remains_vt_1_centerline"],
        "transition_edge_is_vt_1_centerline_e001": route["route_checks"]["transition_edge_is_vt_1_centerline_e001"],
        "transition_edge_is_not_vt_1_centerline_e003": route["route_checks"]["transition_edge_is_not_vt_1_centerline_e003"],
        "segments": stair,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    write_json(out / "route_backfill_validation.json", backfill_validation)
    write_json(out / "astar_segment_validation.json", astar_validation)
    write_json(out / "same_floor_path_validation.json", same_floor_validation)
    write_json(out / "stair_connector_segment_validation.json", stair_validation)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-contract", type=Path, default=DEFAULT_ROUTE_CONTRACT)
    parser.add_argument("--floor-1-map-yaml", type=Path, default=DEFAULT_FLOOR_1_MAP)
    parser.add_argument("--floor-2-map-yaml", type=Path, default=DEFAULT_FLOOR_2_MAP)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--inflation-radius-m", type=float, default=0.22)
    parser.add_argument("--max-snap-radius-m", type=float, default=1.50)
    parser.add_argument("--route-spacing-m", type=float, default=0.15)
    parser.add_argument("--checkpoint-tolerance-m", type=float, default=0.40)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        route = build(args)
        write_json(args.output_dir / ROUTE_NAME, route)
        write_validation_artifacts(args.output_dir, route)
        print(json.dumps({"classification": route["classification"], "route": str(args.output_dir / ROUTE_NAME)}, sort_keys=True))
        return 0 if route["classification"] != "task24g2_blocked_by_same_floor_astar_segment" else 2
    except Exception as exc:
        blocked = {
            "artifact_type": "task24g2_route_backfill_validation",
            "created_utc": now_iso(),
            "classification": "task24g2_blocked_by_same_floor_astar_segment",
            "error": str(exc),
            "route_contract": str(args.route_contract),
            "map_sources": {"floor_1": str(args.floor_1_map_yaml), "floor_2": str(args.floor_2_map_yaml)},
            "claim_boundary": CLAIM_BOUNDARY,
        }
        write_json(args.output_dir / "route_backfill_validation.json", blocked)
        print(json.dumps({"classification": blocked["classification"], "error": str(exc)}, sort_keys=True), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
