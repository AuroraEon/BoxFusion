#!/usr/bin/env python3
"""Execute an artifact-backed RSLG-SLAM object route in Gazebo without Nav2."""

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
TASK_NAME = "task17_lightweight_rslg_executor_without_nav2"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
DEFAULT_STAGE = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
DEFAULT_OUTPUT = TASKS_ROOT / TASK_NAME
INDEX_PATH = TASKS_ROOT / "task14a_object_nav_experiment_adapter/object_candidate_index_v0_1.json"
TASK16_RESULT = TASKS_ROOT / "task16_dynamic_object_query_route_generation_execution_and_wrapper/gui_obj_172_retry/runtime_result.json"
TASK16_APPROACH = TASKS_ROOT / "task16_dynamic_object_query_route_generation_execution_and_wrapper/gui_obj_172_retry/approach_candidate_report.json"
LAUNCHER = THIS_DIR / "launch_lightweight_gazebo_turtlebot3.sh"
FAILURE_LAYERS = {
    "artifact_loading",
    "query_resolution",
    "topology_route_generation",
    "executable_route_generation",
    "no_nav2_bringup",
    "pose_feedback",
    "route_tracking",
    "approach_candidate_generation",
    "approach_tracking",
    "yaw_alignment",
    "validation",
    "cleanup",
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
        ax, ay = [float(value) for value in a][:2]
    if isinstance(b, dict):
        bx, by = float(b["x"]), float(b["y"])
    else:
        bx, by = [float(value) for value in b][:2]
    return math.hypot(bx - ax, by - ay)


def route_length(points: list[dict[str, Any]]) -> float:
    return sum(distance(left, right) for left, right in zip(points, points[1:]))


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    previous = len(polygon) - 1
    for index, point in enumerate(polygon):
        xi, yi = float(point[0]), float(point[1])
        xj, yj = float(polygon[previous][0]), float(polygon[previous][1])
        if (yi > y) != (yj > y):
            at_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < at_x:
                inside = not inside
        previous = index
    return inside


def pair_key(left: str, right: str) -> str:
    return "__".join(sorted([left, right], key=lambda room: int(room.split("_")[1])))


class OccupancyPlanner:
    """Inflated-grid A* and validation over the Stage-A stable occupancy map."""

    def __init__(self, map_yaml: Path, inflation_radius_m: float, spacing_m: float) -> None:
        self.map_yaml = map_yaml
        self.grid, self.resolution, self.origin, self.meta = load_nav_map(map_yaml)
        self.free_mask = (self.grid >= 250).astype(np.uint8)
        self.clearance = cv2.distanceTransform(self.free_mask, cv2.DIST_L2, 5) * self.resolution
        self.inflation_radius_m = inflation_radius_m
        self.spacing_m = spacing_m

    def rc(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(round((y - self.origin[1]) / self.resolution)),
            int(round((x - self.origin[0]) / self.resolution)),
        )

    def xy(self, rc: tuple[int, int]) -> tuple[float, float]:
        return (
            self.origin[0] + rc[1] * self.resolution,
            self.origin[1] + rc[0] * self.resolution,
        )

    def in_bounds(self, rc: tuple[int, int]) -> bool:
        return 0 <= rc[0] < self.grid.shape[0] and 0 <= rc[1] < self.grid.shape[1]

    def free(self, rc: tuple[int, int]) -> bool:
        return bool(self.in_bounds(rc) and int(self.grid[rc[0], rc[1]]) >= 250)

    def traversable(self, rc: tuple[int, int]) -> bool:
        return bool(self.free(rc) and float(self.clearance[rc[0], rc[1]]) >= self.inflation_radius_m)

    def sample(self, xy: Iterable[float]) -> dict[str, Any]:
        x, y = [float(value) for value in xy][:2]
        rc = self.rc(x, y)
        available = self.in_bounds(rc)
        value = int(self.grid[rc[0], rc[1]]) if available else None
        clearance = float(self.clearance[rc[0], rc[1]]) if available else None
        return {
            "grid_row": rc[0],
            "grid_col": rc[1],
            "in_bounds": available,
            "occupancy_value": value,
            "is_free": bool(available and value is not None and value >= 250),
            "inflated_traversable": bool(available and value is not None and value >= 250 and clearance is not None and clearance >= self.inflation_radius_m),
            "clearance_m": round(clearance, 6) if clearance is not None else None,
        }

    def nearest_traversable(self, xy: Iterable[float], max_radius_m: float = 1.0) -> tuple[tuple[int, int], dict[str, Any]]:
        x, y = [float(value) for value in xy][:2]
        start = self.rc(x, y)
        queue = deque([start])
        seen = {start}
        cells = max(1, int(math.ceil(max_radius_m / self.resolution)))
        while queue:
            current = queue.popleft()
            if max(abs(current[0] - start[0]), abs(current[1] - start[1])) > cells:
                continue
            if self.traversable(current):
                snapped = self.xy(current)
                return current, {
                    "requested_world_xy": [x, y],
                    "requested_rc": list(start),
                    "snapped_rc": list(current),
                    "snapped_world_xy": [round(snapped[0], 6), round(snapped[1], 6)],
                    "snap_distance_m": round(math.hypot(snapped[0] - x, snapped[1] - y), 6),
                    "requested_sample": self.sample((x, y)),
                    "snapped_sample": self.sample(snapped),
                }
            for row_shift in (-1, 0, 1):
                for col_shift in (-1, 0, 1):
                    if row_shift == 0 and col_shift == 0:
                        continue
                    candidate = (current[0] + row_shift, current[1] + col_shift)
                    if candidate not in seen and self.in_bounds(candidate):
                        seen.add(candidate)
                        queue.append(candidate)
        raise RuntimeError(f"no inflated-grid traversable point within {max_radius_m:.2f} m of ({x:.3f}, {y:.3f})")

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        open_set: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        cost = {start: 0.0}
        previous: dict[tuple[int, int], tuple[int, int]] = {}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                cells = [current]
                while cells[-1] in previous:
                    cells.append(previous[cells[-1]])
                return list(reversed(cells))
            for dr, dc, step in neighbors:
                candidate = (current[0] + dr, current[1] + dc)
                if not self.traversable(candidate):
                    continue
                clearance = float(self.clearance[candidate[0], candidate[1]])
                penalty = max(0.0, 0.28 - clearance) * 8.0
                proposed = cost[current] + step + penalty
                if proposed >= cost.get(candidate, float("inf")):
                    continue
                cost[candidate] = proposed
                previous[candidate] = current
                heuristic = math.hypot(goal[0] - candidate[0], goal[1] - candidate[1])
                heapq.heappush(open_set, (proposed + heuristic, candidate))
        raise RuntimeError(f"inflated-grid A* could not connect {start} to {goal}")

    def sampled_cells(self, left: dict[str, Any], right: dict[str, Any]) -> list[tuple[int, int]]:
        segment_length = distance(left, right)
        count = max(1, int(math.ceil(segment_length / (self.resolution * 0.45))))
        cells: list[tuple[int, int]] = []
        for index in range(count + 1):
            ratio = index / count
            cells.append(self.rc(
                float(left["x"]) + ratio * (float(right["x"]) - float(left["x"])),
                float(left["y"]) + ratio * (float(right["y"]) - float(left["y"])),
            ))
        return cells

    def validate_polyline(self, points: list[dict[str, Any]], require_inflated: bool = False) -> dict[str, Any]:
        tested: list[tuple[int, int]] = []
        for left, right in zip(points, points[1:]):
            tested.extend(self.sampled_cells(left, right))
        if len(points) == 1:
            tested.append(self.rc(float(points[0]["x"]), float(points[0]["y"])))
        invalid = [
            {
                "grid_row": rc[0],
                "grid_col": rc[1],
                "world_xy": [round(value, 6) for value in self.xy(rc)],
                "occupancy_value": int(self.grid[rc[0], rc[1]]) if self.in_bounds(rc) else None,
                "clearance_m": round(float(self.clearance[rc[0], rc[1]]), 6) if self.in_bounds(rc) else None,
            }
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
            "segment": label,
            "a_star_success": True,
            "inflation_radius_m": self.inflation_radius_m,
            "start_snap": start_snap,
            "goal_snap": goal_snap,
            "raw_grid_point_count": len(raw),
            "executable_waypoint_count": len(points),
            "path_length_m": round(route_length(points), 6),
            **validation,
        }
        return points, segment


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
        records.append({"purpose": purpose, "path": rel(path), "absolute_path": str(path.resolve()), "exists": exists})
        if not exists and purpose != "gateway_validation_report":
            failure = f"required artifact missing: {purpose}: {path}"
    if selected_map != required_map:
        failure = (
            "ambiguous or incorrect runtime occupancy raster: --map-yaml must resolve to "
            f"{required_map}, received {selected_map}"
        )
    image = None
    if selected_map.exists():
        planner_probe = OccupancyPlanner(selected_map, args.inflation_radius_m, args.path_spacing_m)
        image_ref = Path(planner_probe.meta["image"])
        image = image_ref if image_ref.is_absolute() else selected_map.parent / image_ref
        records.append({"purpose": "stable_occupancy_map_image", "path": rel(image), "absolute_path": str(image.resolve()), "exists": image.exists()})
        if not image.exists():
            failure = f"runtime stable occupancy image is missing: {image}"
    report = {
        "artifact_type": "task17_artifact_loading_report",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "artifact_records": records,
        "execution_occupancy_map_yaml": rel(selected_map),
        "execution_occupancy_map_image": rel(image) if image else None,
        "execution_map_role": "stable occupancy map used for A*, inflation, route validation, and runtime trajectory wall-crossing validation",
        "rasters_explicitly_not_used_for_execution": [
            "process/floors/floor_2/room_segmentation/assets/segmentation_wall_processed*",
            "room-segmentation floorplan/raster products",
            "gateway_wall_preclose visualization or segmentation provenance raster",
        ],
        "gateway_wall_preclose_role": "gateway preservation and navigation projection provenance only; not directly loaded as the controller occupancy grid",
        "segmentation_wall_processed_role": "room segmentation provenance only; not a gateway extraction or navigation execution source",
        "map_source_unambiguous": failure is None,
        "success": failure is None,
        "failure_reason": failure,
    }
    write_json(out / "artifact_loading_report.json", report)
    return {key: read_json(path, {}) for key, path in paths.items() if path.suffix == ".json" and path.exists()}, failure


def resolve_query(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    index = load_index(INDEX_PATH)
    query_result = run_query(index, args.query, preferred_floor_id=args.floor_id, top_k=10)
    selected = query_result.get("selected_candidate")
    selection_reason = "highest-ranked artifact-backed candidate meeting parsed constraints"
    if args.object_id:
        requested = canonical_object_id(args.object_id)
        selected = next((row for row in index.get("objects", []) if row.get("object_id") == requested and row.get("floor_id") == args.floor_id), None)
        selection_reason = "explicit --object-id selected a same-floor artifact-backed query candidate"
    report = {
        "artifact_type": "task17_dynamic_query_resolution",
        "created_utc": now_iso(),
        "query_text": args.query,
        "object_id_request": args.object_id,
        "floor_id_request": args.floor_id,
        "start_room": args.start_room,
        "query_resolution_success": selected is not None,
        "object_id": selected.get("object_id") if selected else None,
        "label": selected.get("label") if selected else None,
        "target_room": selected.get("room_id") if selected else None,
        "target_floor": selected.get("floor_id") if selected else None,
        "ambiguity_status": "disambiguated_by_object_id" if args.object_id and selected and query_result.get("ambiguous_query") else ("ambiguous" if query_result.get("ambiguous_query") else "unambiguous"),
        "warnings": list(selected.get("warnings") or []) if selected else [],
        "provenance": selected.get("source_artifact_provenance") if selected else None,
        "selection_reason": selection_reason if selected else None,
        "candidate_query_result": query_result,
        "non_claims": [
            "Artifact-backed query resolution is not semantic ground-truth accuracy.",
            "No open-vocabulary CLIP retrieval is implemented or claimed.",
            "No visual object detection or confirmation is claimed.",
        ],
    }
    if args.object_id and selected is None:
        report["failure_reason"] = "explicit object id was not present on the requested floor in the artifact-backed index"
    elif selected is None:
        report["failure_reason"] = query_result.get("failure_reason") or "query produced no selected artifact-backed candidate"
    else:
        report["failure_reason"] = None
    write_json(out / "dynamic_query_resolution.json", report)
    return selected, report


def generate_topology_route(args: argparse.Namespace, selected: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    topology = artifacts["topology"]
    model = artifacts["committed_room_world_model"]
    registry = artifacts["gateway_registry"]
    rooms = {room["id"]: room for room in topology.get("rooms", []) if room.get("floor_id") == args.floor_id}
    gateways = {item.get("pair_key"): item for item in registry.get("gateways", [])}
    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {room_id: [] for room_id in rooms}
    eligible: list[dict[str, Any]] = []
    for edge in model.get("adjacency", []):
        left, right = edge.get("source"), edge.get("target")
        key = pair_key(left, right) if left in rooms and right in rooms else None
        gateway = gateways.get(key)
        accepted = bool(gateway and (gateway.get("validation") or {}).get("accepted_for_carving", False))
        allowed = bool(left in rooms and right in rooms and edge.get("relation_type") != "vertical_transition" and edge.get("status") in {"supported", "confirmed"} and accepted)
        item = {**edge, "pair_key": key, "gateway_id": (gateway or {}).get("gateway_id"), "eligible": allowed}
        if allowed:
            eligible.append(item)
            adjacency[left].append((right, item))
            adjacency[right].append((left, item))
    target = selected["room_id"]
    queue = deque([args.start_room])
    previous: dict[str, tuple[str, dict[str, Any]] | None] = {args.start_room: None}
    while queue:
        current = queue.popleft()
        if current == target:
            break
        for neighbor, edge in sorted(adjacency.get(current, []), key=lambda pair: pair[0]):
            if neighbor not in previous:
                previous[neighbor] = (current, edge)
                queue.append(neighbor)
    room_sequence: list[str] = []
    used_edges: list[dict[str, Any]] = []
    if target in previous:
        current = target
        while current != args.start_room:
            room_sequence.append(current)
            prior, edge = previous[current]  # type: ignore[misc]
            used_edges.append({**edge, "traversed_from": prior, "traversed_to": current})
            current = prior
        room_sequence.append(args.start_room)
        room_sequence.reverse()
        used_edges.reverse()
    report = {
        "artifact_type": "task17_generated_topology_route",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "query": args.query,
        "object_id": selected["object_id"],
        "floor_id": args.floor_id,
        "start_room": args.start_room,
        "target_room": target,
        "route_generated": bool(room_sequence),
        "room_sequence": room_sequence,
        "topology_edges_used": used_edges,
        "gateway_sequence": [row["gateway_id"] for row in used_edges],
        "eligible_same_floor_edges": eligible,
        "authority_artifacts": [rel(artifact_paths(args)[key]) for key in ("topology", "topology_query_report", "committed_room_world_snapshot", "committed_room_world_model", "gateway_registry")],
        "generation_method": "breadth-first search over same-floor committed relations with accepted gateway projection assets",
        "failure_reason": None if room_sequence else "no same-floor gateway-supported topology path from start room to target room",
    }
    write_json(out / "generated_topology_route.json", report)
    return report


def semantic_anchors(args: argparse.Namespace, topology_route: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    rooms = {room["id"]: room for room in artifacts["topology"].get("rooms", [])}
    gateways = {gateway["gateway_id"]: gateway for gateway in artifacts["gateway_registry"].get("gateways", [])}
    room_sequence = topology_route["room_sequence"]
    gateway_sequence = topology_route["gateway_sequence"]
    anchors: list[dict[str, Any]] = []
    for index, room_id in enumerate(room_sequence):
        center = rooms[room_id]["center"]
        anchors.append({"source": "room_center", "room_id": room_id, "x": float(center[0]), "y": float(center[1])})
        if index < len(gateway_sequence):
            gateway = gateways[gateway_sequence[index]]
            anchors.append({
                "source": "gateway",
                "gateway_id": gateway["gateway_id"],
                "from_room": room_id,
                "to_room": room_sequence[index + 1],
                "x": float(gateway["x"]),
                "y": float(gateway["y"]),
            })
    for left, right in zip(anchors, anchors[1:]):
        left["yaw"] = math.atan2(float(right["y"]) - float(left["y"]), float(right["x"]) - float(left["x"]))
    if anchors:
        anchors[-1]["yaw"] = anchors[-2].get("yaw", 0.0) if len(anchors) > 1 else 0.0
    payload = {
        "artifact_type": "task17_generated_semantic_route",
        "created_utc": now_iso(),
        "scene_id": SCENE_ID,
        "floor_id": args.floor_id,
        "query": args.query,
        "room_sequence": room_sequence,
        "gateway_sequence": gateway_sequence,
        "waypoints": anchors,
        "map_yaml": rel(stable_map_paths(args)[1]),
        "source": "dynamic_topology_room_sequence_with_gateway_registry_anchors",
    }
    write_json(out / "generated_semantic_route.json", payload)
    return payload


def executable_route(args: argparse.Namespace, semantic: dict[str, Any], planner: OccupancyPlanner, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    all_points: list[dict[str, Any]] = []
    segment_reports: list[dict[str, Any]] = []
    try:
        for index, (start, goal) in enumerate(zip(semantic["waypoints"], semantic["waypoints"][1:])):
            points, segment = planner.plan_segment((start["x"], start["y"]), (goal["x"], goal["y"]), f"{start['source']}->{goal['source']}")
            segment.update({"segment_index": index, "from_anchor": start, "to_anchor": goal})
            segment_reports.append(segment)
            for point in points if not all_points else points[1:]:
                point.update({
                    "waypoint_index": len(all_points),
                    "floor_id": args.floor_id,
                    "source": "inflated_grid_astar",
                    "semantic_segment_index": index,
                    **planner.sample((point["x"], point["y"])),
                })
                all_points.append(point)
        for left, right in zip(all_points, all_points[1:]):
            left["yaw"] = round(math.atan2(float(right["y"]) - float(left["y"]), float(right["x"]) - float(left["x"])), 6)
        if all_points:
            all_points[-1]["yaw"] = all_points[-2].get("yaw", 0.0) if len(all_points) > 1 else 0.0
        validation = planner.validate_polyline(all_points, require_inflated=True)
    except Exception as exc:
        report = {
            "artifact_type": "task17_route_generation_report",
            "created_utc": now_iso(),
            "passed": False,
            "failure_reason": f"{type(exc).__name__}: {exc}",
            "map_yaml": rel(planner.map_yaml),
            "claim": "This route is occupancy-grid-valid only; it is not a full robot collision-free guarantee.",
            "segments": segment_reports,
        }
        write_json(out / "route_generation_report.json", report)
        return None, report
    report = {
        "artifact_type": "task17_route_generation_report",
        "created_utc": now_iso(),
        "passed": bool(all_points and validation["wall_crossing_validation_passed"]),
        "floor_id": args.floor_id,
        "map_yaml": rel(planner.map_yaml),
        "map_meta": planner.meta,
        "map_role": "stable occupancy map used for inflated-grid A* and collision validation",
        "rasters_not_used": ["segmentation_wall_processed", "room-segmentation raster products"],
        "inflation_radius_m": args.inflation_radius_m,
        "spacing_m": args.path_spacing_m,
        "executable_waypoint_count": len(all_points),
        "route_length_m": round(route_length(all_points), 6),
        "occupied_waypoint_count": len([point for point in all_points if not point["is_free"]]),
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "wall_crossing_validation_passed": validation["wall_crossing_validation_passed"],
        "polyline_validation": validation,
        "segments": segment_reports,
        "claim": "This route is occupancy-grid-valid only; it is not a full robot collision-free guarantee.",
        "failure_reason": None if validation["wall_crossing_validation_passed"] else "inflated occupancy-grid route validation failed",
    }
    payload = {
        "artifact_type": "task17_lightweight_executable_route",
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
        "claim": report["claim"],
    }
    write_json(out / "generated_lightweight_executable_route.json", payload)
    write_json(out / "route_generation_report.json", report)
    return payload if report["passed"] else None, report


def yaw_proxy(selected: dict[str, Any], snapshot: dict[str, Any]) -> dict[str, Any] | None:
    object_id = canonical_object_id(selected["object_id"])
    for anchor in snapshot.get("anchors", []):
        if anchor.get("anchor_type") == "object" and canonical_object_id(anchor.get("target_id")) == object_id and anchor.get("valid", True):
            position = anchor.get("position") or []
            if len(position) >= 2:
                return {
                    "yaw_proxy_source": "committed_room_world_snapshot_object_anchor",
                    "anchor_id": anchor.get("id"),
                    "yaw_proxy_xy": [float(position[0]), float(position[1])],
                    "provenance_statement": "committed proxy anchor, not visual object confirmation",
                }
    position = selected.get("pose_xy")
    if position:
        return {
            "yaw_proxy_source": "committed_object_centroid_proxy_fallback",
            "anchor_id": None,
            "yaw_proxy_xy": [float(position[0]), float(position[1])],
            "provenance_statement": "committed centroid proxy, not visual object confirmation",
        }
    return None


def ray_validation(planner: OccupancyPlanner, start: Iterable[float], goal: Iterable[float]) -> dict[str, Any]:
    points = [{"x": float(list(start)[0]), "y": float(list(start)[1])}, {"x": float(list(goal)[0]), "y": float(list(goal)[1])}]
    return planner.validate_polyline(points, require_inflated=False)


def approach_candidate(
    args: argparse.Namespace,
    selected: dict[str, Any],
    semantic: dict[str, Any],
    artifacts: dict[str, Any],
    planner: OccupancyPlanner,
    out: Path,
) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    proxy = yaw_proxy(selected, artifacts["committed_room_world_snapshot"])
    if proxy is None:
        report = {"artifact_type": "task17_approach_candidate_report", "success": False, "failure_reason": "no committed object anchor or proxy pose was available"}
        write_json(out / "approach_candidate_report.json", report)
        return None, None
    candidate = None
    reuse_source = None
    if selected["object_id"] == "obj_172":
        task16 = read_json(TASK16_APPROACH, {})
        existing = task16.get("recommended_candidate")
        if existing and existing.get("candidate_id") == "generated_ring_035":
            candidate = dict(existing)
            reuse_source = rel(TASK16_APPROACH)
    room = next((item for item in artifacts["topology"].get("rooms", []) if item.get("id") == selected["room_id"]), {})
    if candidate is None:
        object_xy = selected.get("pose_xy") or proxy["yaw_proxy_xy"]
        terminal = semantic["waypoints"][-1]
        generated = []
        for index, angle_deg in enumerate(range(0, 360, 15)):
            angle = math.radians(angle_deg)
            xy = [float(object_xy[0]) + 0.8 * math.cos(angle), float(object_xy[1]) + 0.8 * math.sin(angle)]
            sample = planner.sample(xy)
            if sample["inflated_traversable"] and point_in_polygon(xy[0], xy[1], room.get("polygon", [])):
                generated.append({
                    "candidate_id": f"task17_ring_{index:03d}",
                    "world_xy": xy,
                    "candidate_source": "generated_standoff_ring_from_committed_proxy",
                    "map_sample": sample,
                    "distance_to_route_terminal_m": distance(xy, (terminal["x"], terminal["y"])),
                })
        if generated:
            candidate = min(generated, key=lambda item: item["distance_to_route_terminal_m"])
    if candidate is None:
        report = {"artifact_type": "task17_approach_candidate_report", "success": False, "failure_reason": "no inflated-grid-valid approach candidate could be reused or generated"}
        write_json(out / "approach_candidate_report.json", report)
        return None, proxy
    candidate["map_sample_task17"] = planner.sample(candidate["world_xy"])
    candidate["yaw_proxy_ray_validation_task17"] = ray_validation(planner, candidate["world_xy"], proxy["yaw_proxy_xy"])
    candidate["yaw"] = round(math.atan2(proxy["yaw_proxy_xy"][1] - candidate["world_xy"][1], proxy["yaw_proxy_xy"][0] - candidate["world_xy"][0]), 6)
    valid = bool(candidate["map_sample_task17"]["inflated_traversable"] and candidate["yaw_proxy_ray_validation_task17"]["wall_crossing_validation_passed"])
    report = {
        "artifact_type": "task17_approach_candidate_report",
        "created_utc": now_iso(),
        "object_id": selected["object_id"],
        "success": valid,
        "selected_candidate_id": candidate.get("candidate_id"),
        "recommended_candidate": candidate,
        "candidate_reuse_source": reuse_source,
        "candidate_policy": "reuse task16 validated standoff when applicable, then revalidate against task17 stable occupancy map; otherwise generate from committed proxy",
        "map_yaml": rel(planner.map_yaml),
        "failure_reason": None if valid else "selected approach candidate did not pass task17 inflated occupancy/ray validation",
    }
    proxy_report = {
        "artifact_type": "task17_yaw_proxy_report",
        "created_utc": now_iso(),
        "available": True,
        "object_id": selected["object_id"],
        "selected_candidate_id": candidate.get("candidate_id"),
        "expected_yaw_rad": candidate["yaw"],
        **proxy,
        "statement": "Yaw uses a committed object proxy anchor; this is not visual object confirmation.",
    }
    write_json(out / "approach_candidate_report.json", report)
    write_json(out / "yaw_proxy_report.json", proxy_report)
    return candidate if valid else None, proxy_report


def plot_visualization(planner: OccupancyPlanner, route: dict[str, Any], semantic: dict[str, Any], candidate: dict[str, Any] | None, trajectory: list[dict[str, Any]], path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    image = np.flipud(planner.grid)
    extent = [
        planner.origin[0],
        planner.origin[0] + planner.grid.shape[1] * planner.resolution,
        planner.origin[1],
        planner.origin[1] + planner.grid.shape[0] * planner.resolution,
    ]
    fig, axis = plt.subplots(figsize=(9, 7))
    axis.imshow(image, cmap="gray", extent=extent, origin="upper")
    points = route.get("waypoints") or []
    axis.plot([item["x"] for item in points], [item["y"] for item in points], color="#1679c4", linewidth=1.6, label="lightweight A* route")
    anchors = semantic.get("waypoints") or []
    axis.scatter([item["x"] for item in anchors], [item["y"] for item in anchors], color="#ee7c20", s=22, label="topology/gateway anchors")
    if candidate:
        xy = candidate["world_xy"]
        proxy = candidate.get("visible_proxy_xy") or []
        axis.scatter([xy[0]], [xy[1]], marker="X", color="#169c4b", s=65, label="approach candidate")
        if len(proxy) >= 2:
            axis.plot([xy[0], proxy[0]], [xy[1], proxy[1]], "--", color="#cb2364", label="object-facing proxy ray")
    if trajectory:
        axis.plot([item["x"] for item in trajectory], [item["y"] for item in trajectory], color="#6f2dbd", linewidth=1.1, label="executed trajectory")
    axis.set_title("RSLG-SLAM task17 stable-map planning and execution evidence")
    axis.set_aspect("equal")
    axis.legend(fontsize=8, loc="best")
    axis.set_xlim(-10.2, 2.0)
    axis.set_ylim(-0.8, 7.2)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def capture_command(command: list[str], env: dict[str, str], timeout: float = 10.0) -> dict[str, Any]:
    try:
        result = subprocess.run(command, cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        return {"command": command, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    except Exception as exc:
        return {"command": command, "returncode": None, "error": f"{type(exc).__name__}: {exc}"}


def process_snapshot() -> dict[str, Any]:
    result = subprocess.run(["ps", "-eo", "pid=,ppid=,stat=,comm=,args="], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    relevant = [
        line.strip() for line in result.stdout.splitlines()
        if re.search(r"gzserver|gazebo|turtlebot3|static_transform_publisher|nav2_|planner_server|controller_server|bt_navigator|behavior_server|costmap", line, re.I)
        and "ps -eo" not in line
    ]
    forbidden = [line for line in relevant if NAV2_PROCESS_PATTERN.search(line)]
    return {"created_utc": now_iso(), "relevant_processes": relevant, "forbidden_nav2_processes": forbidden, "no_nav2_processes_present": not forbidden}


def graph_snapshot(env: dict[str, str]) -> dict[str, Any]:
    nodes = capture_command(["ros2", "node", "list"], env)
    topics = capture_command(["ros2", "topic", "list"], env)
    actions = capture_command(["ros2", "action", "list"], env)
    action_rows = {line.strip() for line in (actions.get("stdout") or "").splitlines() if line.strip()}
    forbidden_actions = sorted(action_rows.intersection(NAV2_ACTIONS))
    node_text = nodes.get("stdout") or ""
    forbidden_nodes = sorted({line.strip() for line in node_text.splitlines() if NAV2_PROCESS_PATTERN.search(line)})
    return {
        "created_utc": now_iso(),
        "nodes": nodes,
        "topics": topics,
        "actions": actions,
        "forbidden_nav2_actions": forbidden_actions,
        "forbidden_nav2_nodes": forbidden_nodes,
        "no_nav2_actions_or_nodes_present": not forbidden_actions and not forbidden_nodes,
    }


def save_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def run_velocity_execution(
    args: argparse.Namespace,
    route: dict[str, Any],
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    planner: OccupancyPlanner,
    out: Path,
) -> dict[str, Any]:
    try:
        import rclpy
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.duration import Duration
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformException, TransformListener
    except Exception as exc:
        return {"success": False, "failure_layer": "pose_feedback", "failure_reason": f"ROS Python imports unavailable: {type(exc).__name__}: {exc}"}

    def yaw_from_quaternion(quaternion: Any) -> float:
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    class VelocityNode(Node):
        def __init__(self) -> None:
            super().__init__("task17_lightweight_rslg_executor")
            self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.odom: dict[str, Any] | None = None
            self.gazebo: dict[str, Any] | None = None
            self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 20)
            self.create_subscription(ModelStates, "/gazebo/model_states", self.on_models, 10)

        def on_odom(self, message: Any) -> None:
            pose = message.pose.pose
            self.odom = {
                "x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw_from_quaternion(pose.orientation),
                "frame_id": message.header.frame_id or "odom", "source": "/odom",
                "stamp_ns": int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec),
            }

        def on_models(self, message: Any) -> None:
            for index, name in enumerate(message.name):
                if "turtlebot3" in name:
                    pose = message.pose[index]
                    self.gazebo = {
                        "x": float(pose.position.x), "y": float(pose.position.y), "yaw": yaw_from_quaternion(pose.orientation),
                        "frame_id": "gazebo_world", "source": "/gazebo/model_states", "stamp_ns": time.monotonic_ns(),
                    }
                    return

        def pose(self) -> dict[str, Any] | None:
            if args.pose_source == "tf":
                try:
                    transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.15))
                    return {
                        "x": float(transform.transform.translation.x), "y": float(transform.transform.translation.y),
                        "yaw": yaw_from_quaternion(transform.transform.rotation), "frame_id": "map", "source": "/tf map->base_footprint",
                        "stamp_ns": int(transform.header.stamp.sec) * 1000000000 + int(transform.header.stamp.nanosec),
                    }
                except TransformException:
                    return None
            if args.pose_source == "odom":
                return self.odom
            return self.gazebo

        def command(self, linear: float, angular: float) -> None:
            message = Twist()
            message.linear.x = float(linear)
            message.angular.z = float(angular)
            self.publisher.publish(message)

        def stop(self) -> None:
            for _ in range(4):
                self.command(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.03)

    commands: list[dict[str, Any]] = []
    trajectory: list[dict[str, Any]] = []
    start_time = time.monotonic()

    def remaining_time() -> float:
        return args.timeout_sec - (time.monotonic() - start_time)

    def observe(node: VelocityNode, timeout: float = 10.0) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            found = node.pose()
            if found:
                return found
        return None

    def append_sample(pose: dict[str, Any], phase: str, linear: float, angular: float, target_index: int | None) -> None:
        trajectory.append({
            "t_sec": round(time.monotonic() - start_time, 6),
            "phase": phase,
            "x": round(float(pose["x"]), 6),
            "y": round(float(pose["y"]), 6),
            "yaw": round(float(pose["yaw"]), 6),
            "pose_source": pose["source"],
            "target_waypoint_index": target_index,
            "cmd_vel_linear_x": round(linear, 6),
            "cmd_vel_angular_z": round(angular, 6),
        })
        commands.append({
            "t_sec": round(time.monotonic() - start_time, 6),
            "phase": phase,
            "linear_x": round(linear, 6),
            "angular_z": round(angular, 6),
        })

    def follow(node: VelocityNode, points: list[dict[str, Any]], phase: str, tolerance: float) -> dict[str, Any]:
        result = {"phase": phase, "attempted": True, "controller_success": False, "failure_reason": None, "start_pose": node.pose()}
        progress = 0
        last_stamp: int | None = None
        last_fresh_wall = time.monotonic()
        while rclpy.ok() and remaining_time() > 0:
            rclpy.spin_once(node, timeout_sec=0.03)
            pose = node.pose()
            if pose is None:
                result["failure_reason"] = "pose feedback became unavailable during direct velocity control"
                break
            pose_stamp = pose.get("stamp_ns")
            if pose_stamp != last_stamp:
                last_stamp = pose_stamp
                last_fresh_wall = time.monotonic()
            elif time.monotonic() - last_fresh_wall > 2.0:
                result["failure_reason"] = "pose feedback became stale during direct velocity control"
                break
            nearest_index, nearest_distance = min(
                ((index, distance(pose, point)) for index, point in enumerate(points)),
                key=lambda item: item[1],
            )
            progress = max(progress, nearest_index)
            if nearest_distance > args.path_deviation_limit_m:
                result["failure_reason"] = f"robot deviated {nearest_distance:.3f} m from planned path, exceeding {args.path_deviation_limit_m:.3f} m"
                break
            goal_distance = distance(pose, points[-1])
            if goal_distance <= tolerance:
                node.stop()
                append_sample(pose, phase, 0.0, 0.0, len(points) - 1)
                result.update({"controller_success": True, "final_pose": pose, "final_distance_m": round(goal_distance, 6)})
                return result
            lookahead_index = progress
            while lookahead_index < len(points) - 1 and distance(pose, points[lookahead_index]) < args.lookahead_distance:
                lookahead_index += 1
            target = points[lookahead_index]
            heading_error = angle_wrap(math.atan2(float(target["y"]) - float(pose["y"]), float(target["x"]) - float(pose["x"])) - float(pose["yaw"]))
            angular = clamp(args.heading_kp * heading_error, -args.max_angular_speed, args.max_angular_speed)
            forward_scale = max(0.0, math.cos(heading_error))
            if abs(heading_error) > args.forward_heading_limit_rad:
                forward_scale = 0.0
            linear = min(args.max_linear_speed, max(0.035, goal_distance * 0.45)) * forward_scale
            applied_angular = args.angular_command_sign * angular
            node.command(linear, applied_angular)
            append_sample(pose, phase, linear, applied_angular, lookahead_index)
            time.sleep(0.07)
        node.stop()
        pose = node.pose()
        result.update({"final_pose": pose, "final_distance_m": round(distance(pose, points[-1]), 6) if pose else None})
        if result["failure_reason"] is None:
            result["failure_reason"] = "direct velocity controller timed out"
        return result

    def align_yaw(node: VelocityNode) -> dict[str, Any]:
        start = node.pose()
        result = {
            "artifact_type": "task17_yaw_alignment_result",
            "created_utc": now_iso(),
            "attempted": True,
            "method": "bounded_p_control_direct_cmd_vel",
            "linear_velocity_during_alignment": 0.0,
            "yaw_proxy_source": proxy["yaw_proxy_source"],
            "yaw_proxy_xy": proxy["yaw_proxy_xy"],
            "start_pose": start,
            "samples": [],
        }
        if start is None:
            result.update({"yaw_alignment_success": False, "failure_reason": "pose unavailable before yaw alignment"})
            return result
        deadline = time.monotonic() + min(args.yaw_timeout_sec, max(0.0, remaining_time()))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.04)
            pose = node.pose()
            if pose is None:
                result["failure_reason"] = "pose feedback became unavailable during yaw alignment"
                break
            target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(pose["y"]), proxy["yaw_proxy_xy"][0] - float(pose["x"]))
            signed_error = angle_wrap(target_yaw - float(pose["yaw"]))
            drift = distance(start, pose)
            row = {
                "t_sec": round(time.monotonic() - start_time, 6),
                "yaw_error_rad": round(abs(signed_error), 6),
                "yaw_error_deg": round(math.degrees(abs(signed_error)), 3),
                "target_yaw_rad": round(target_yaw, 6),
                "x": round(float(pose["x"]), 6),
                "y": round(float(pose["y"]), 6),
                "yaw": round(float(pose["yaw"]), 6),
                "xy_drift_m": round(drift, 6),
            }
            result["samples"].append(row)
            if drift > args.xy_drift_limit_m:
                result["failure_reason"] = "xy drift limit exceeded during yaw alignment"
                break
            if abs(signed_error) <= args.yaw_internal_tolerance_rad:
                node.stop()
                result["controller_internal_stop_reached"] = True
                break
            angular = clamp(args.yaw_kp * signed_error, -args.max_yaw_angular_speed, args.max_yaw_angular_speed)
            applied_angular = args.angular_command_sign * angular
            node.command(0.0, applied_angular)
            append_sample(pose, "yaw_alignment", 0.0, applied_angular, None)
            time.sleep(0.07)
        node.stop()
        final_pose = node.pose()
        final_target = math.atan2(proxy["yaw_proxy_xy"][1] - float(final_pose["y"]), proxy["yaw_proxy_xy"][0] - float(final_pose["x"])) if final_pose else None
        final_error = abs(angle_wrap(final_target - float(final_pose["yaw"]))) if final_pose and final_target is not None else None
        drift = distance(start, final_pose) if final_pose else None
        success = bool(final_error is not None and final_error <= args.yaw_report_tolerance_rad and drift is not None and drift <= args.xy_drift_limit_m)
        result.update({
            "target_yaw_final_rad": round(final_target, 6) if final_target is not None else None,
            "final_pose": final_pose,
            "final_yaw_error_rad": round(final_error, 6) if final_error is not None else None,
            "final_yaw_error_deg": round(math.degrees(final_error), 3) if final_error is not None else None,
            "xy_drift_during_yaw_alignment_m": round(drift, 6) if drift is not None else None,
            "internal_stop_yaw_tolerance_rad": args.yaw_internal_tolerance_rad,
            "reported_yaw_tolerance_rad": args.yaw_report_tolerance_rad,
            "xy_drift_limit_m": args.xy_drift_limit_m,
            "yaw_alignment_success": success,
            "failure_reason": None if success else result.get("failure_reason") or "final object-facing yaw was outside reported tolerance",
        })
        return result

    def settle_after_translation(node: VelocityNode, duration_sec: float = 2.5) -> dict[str, Any]:
        start = node.pose()
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            node.command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            pose = node.pose()
            if pose:
                append_sample(pose, "zero_velocity_settle", 0.0, 0.0, None)
            time.sleep(0.05)
        node.stop()
        final = node.pose()
        return {
            "duration_sec": duration_sec,
            "start_pose": start,
            "final_pose": final,
            "observed_xy_drift_m": round(distance(start, final), 6) if start and final else None,
        }

    result: dict[str, Any] = {"success": False, "failure_layer": None, "failure_reason": None}
    rclpy.init(args=None)
    node = VelocityNode()
    try:
        initial_pose = observe(node)
        pose_report = {
            "artifact_type": "task17_pose_source_report",
            "created_utc": now_iso(),
            "requested_pose_source": args.pose_source,
            "pose_feedback_available": initial_pose is not None,
            "initial_pose_observed": initial_pose,
            "pose_frame": (initial_pose or {}).get("frame_id"),
            "map_odom_assumptions": {
                "tf": "static map->odom transform plus robot TF is treated as map-aligned for this controlled simulation",
                "odom": "odom is accepted only under the intentionally launched static map->odom identity alignment",
                "gazebo": "Gazebo world coordinates are treated as aligned with the generated floor map in the controlled runtime world",
            }.get(args.pose_source),
            "non_claim": "Controlled simulation pose feedback is assumed available; this is not a SLAM/localization accuracy claim.",
        }
        write_json(out / "pose_source_report.json", pose_report)
        if initial_pose is None:
            result.update({"failure_layer": "pose_feedback", "failure_reason": f"no pose received from requested source {args.pose_source}"})
            return result
        room_follow = follow(node, route["waypoints"], "room_route", args.goal_tolerance)
        write_json(out / "path_following_result.json", room_follow)
        terminal_distance = room_follow.get("final_distance_m")
        target_arrival = bool(room_follow["controller_success"] and terminal_distance is not None and terminal_distance <= args.goal_tolerance)
        if not target_arrival:
            result.update({"failure_layer": "route_tracking", "failure_reason": room_follow.get("failure_reason") or "target room terminal was not reached"})
            return {**result, "target_room_arrival": False, "room_follow": room_follow, "initial_pose": initial_pose}
        settle_result = settle_after_translation(node)
        current = node.pose()
        candidate_xy = candidate["world_xy"]
        candidate_distance = distance(current, candidate_xy) if current else None
        approach: dict[str, Any] = {
            "artifact_type": "task17_approach_execution_result",
            "created_utc": now_iso(),
            "approach_candidate_id": candidate["candidate_id"],
            "approach_position_tolerance_m": args.approach_position_tolerance_m,
            "distance_at_precheck_m": round(candidate_distance, 6) if candidate_distance is not None else None,
            "approach_position_controller_attempted": False,
            "approach_position_controller_success": False,
            "pre_approach_zero_velocity_settle": settle_result,
        }
        if candidate_distance is not None and candidate_distance <= args.approach_position_tolerance_m:
            approach.update({
                "approach_position_reach_basis": "precheck_within_tolerance",
                "approach_position_tolerance_reached": True,
                "final_distance_to_approach_candidate_m": round(candidate_distance, 6),
            })
        elif current is None:
            approach.update({"approach_position_reach_basis": "pose_unavailable", "approach_position_tolerance_reached": False, "failure_reason": "pose unavailable before approach"})
        else:
            try:
                points, segment_report = planner.plan_segment((current["x"], current["y"]), candidate_xy, "runtime_current_pose->approach_candidate")
                for index, point in enumerate(points):
                    point["waypoint_index"] = index
                approach["approach_path_generation"] = segment_report
                approach["approach_position_controller_attempted"] = True
                follow_result = follow(node, points, "approach_position", args.approach_position_tolerance_m)
                final = node.pose()
                final_distance = distance(final, candidate_xy) if final else None
                controller_success = bool(follow_result["controller_success"])
                tolerance_reached = bool(final_distance is not None and final_distance <= args.approach_position_tolerance_m)
                approach.update({
                    "approach_position_reach_basis": "lightweight_controller_execution" if controller_success else "post_controller_tolerance_check",
                    "approach_position_controller_success": controller_success,
                    "approach_position_tolerance_reached": tolerance_reached,
                    "final_distance_to_approach_candidate_m": round(final_distance, 6) if final_distance is not None else None,
                    "controller_result": follow_result,
                })
            except Exception as exc:
                approach.update({"approach_position_reach_basis": "planning_failure", "approach_position_tolerance_reached": False, "failure_reason": f"{type(exc).__name__}: {exc}"})
        write_json(out / "approach_execution_result.json", approach)
        if not approach.get("approach_position_tolerance_reached"):
            result.update({"failure_layer": "approach_tracking", "failure_reason": approach.get("failure_reason") or "approach position was not reached"})
            return {**result, "target_room_arrival": True, "approach": approach, "room_follow": room_follow}
        yaw_result = align_yaw(node)
        write_json(out / "yaw_alignment_result.json", yaw_result)
        trajectory_validation = planner.validate_polyline(trajectory, require_inflated=False)
        write_json(out / "path_deviation_report.json", {
            "artifact_type": "task17_runtime_trajectory_validation",
            "created_utc": now_iso(),
            "map_yaml": rel(planner.map_yaml),
            "path_deviation_limit_m": args.path_deviation_limit_m,
            "wall_crossing_validation_passed": trajectory_validation["wall_crossing_validation_passed"],
            "trajectory_occupancy_validation": trajectory_validation,
        })
        object_success = bool(yaw_result["yaw_alignment_success"] and trajectory_validation["wall_crossing_validation_passed"])
        result.update({
            "success": object_success,
            "failure_layer": None if object_success else ("yaw_alignment" if not yaw_result["yaw_alignment_success"] else "validation"),
            "failure_reason": None if object_success else (yaw_result.get("failure_reason") if not yaw_result["yaw_alignment_success"] else "executed trajectory crossed occupied stable-map cells"),
            "initial_pose": initial_pose,
            "final_pose": yaw_result.get("final_pose"),
            "room_follow": room_follow,
            "approach": approach,
            "yaw": yaw_result,
            "target_room_arrival": True,
            "wall_crossing_validation_passed": trajectory_validation["wall_crossing_validation_passed"],
        })
        return result
    finally:
        node.stop()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        write_json(out / "cmd_vel_log.json", {"artifact_type": "task17_cmd_vel_log", "commands": commands})
        write_json(out / "executed_trajectory.json", {"artifact_type": "task17_executed_trajectory", "samples": trajectory})
        save_csv(out / "cmd_vel_log.csv", commands, ["t_sec", "phase", "linear_x", "angular_z"])
        save_csv(out / "executed_trajectory.csv", trajectory, ["t_sec", "phase", "x", "y", "yaw", "pose_source", "target_waypoint_index", "cmd_vel_linear_x", "cmd_vel_angular_z"])


def controller_parameters(args: argparse.Namespace, out: Path) -> dict[str, Any]:
    payload = {
        "artifact_type": "task17_lightweight_controller_params",
        "created_utc": now_iso(),
        "controller_type": "lookahead_waypoint_follower_with_heading_p_control_and_bounded_final_yaw_p_control",
        "cmd_vel_topic": "/cmd_vel",
        "pose_source": args.pose_source,
        "lookahead_distance_m": args.lookahead_distance,
        "max_linear_speed_mps": args.max_linear_speed,
        "max_angular_speed_radps": args.max_angular_speed,
        "angular_command_sign": args.angular_command_sign,
        "angular_command_sign_basis": "configured from observed Gazebo TurtleBot3 yaw response; reported cmd_vel values are applied values",
        "heading_kp": args.heading_kp,
        "waypoint_tolerance_m": args.waypoint_tolerance,
        "goal_tolerance_m": args.goal_tolerance,
        "path_deviation_limit_m": args.path_deviation_limit_m,
        "forward_heading_limit_rad": args.forward_heading_limit_rad,
        "approach_position_tolerance_m": args.approach_position_tolerance_m,
        "yaw_kp": args.yaw_kp,
        "max_yaw_angular_speed_radps": args.max_yaw_angular_speed,
        "yaw_internal_tolerance_rad": args.yaw_internal_tolerance_rad,
        "yaw_report_tolerance_rad": args.yaw_report_tolerance_rad,
        "yaw_timeout_sec": args.yaw_timeout_sec,
        "xy_drift_limit_m": args.xy_drift_limit_m,
        "runtime_timeout_sec": args.timeout_sec,
        "safety": ["zero velocity on stop or failure", "pose-loss failure", "path-deviation failure", "post-execution stable occupancy wall-crossing validation"],
    }
    write_json(out / "controller_params.json", payload)
    return payload


def marker_manifest(args: argparse.Namespace, topology: dict[str, Any], semantic: dict[str, Any], route: dict[str, Any], candidate: dict[str, Any], proxy: dict[str, Any], out: Path) -> None:
    manifest = {
        "artifact_type": "task17_marker_manifest",
        "created_utc": now_iso(),
        "optional_marker_topic": "/task17/lightweight_rslg_markers",
        "published": False,
        "rviz_visible_validation_claimed": False,
        "markers_saved": [
            {"type": "topology_room_sequence", "value": topology["room_sequence"]},
            {"type": "gateway_sequence", "value": topology["gateway_sequence"]},
            {"type": "route_polyline", "waypoint_count": len(route["waypoints"])},
            {"type": "approach_candidate", "id": candidate["candidate_id"], "xy": candidate["world_xy"]},
            {"type": "yaw_proxy", "source": proxy["yaw_proxy_source"], "xy": proxy["yaw_proxy_xy"]},
            {"type": "facing_ray", "from_xy": candidate["world_xy"], "to_xy": proxy["yaw_proxy_xy"]},
            {"type": "runtime_status_text", "value": "planned; runtime status is reported separately"},
        ],
        "source_files": [rel(out / "generated_topology_route.json"), rel(out / "generated_semantic_route.json"), rel(out / "generated_lightweight_executable_route.json")],
    }
    write_json(out / "marker_manifest.json", manifest)


def comparison_report(args: argparse.Namespace, runtime: dict[str, Any], route: dict[str, Any], out: Path) -> dict[str, Any]:
    task16 = read_json(TASK16_RESULT, {})
    trajectory = read_json(out / "runtime_run/executed_trajectory.json", {}).get("samples") or []
    route_time = trajectory[-1]["t_sec"] if trajectory else None
    task17 = {
        "backend": "lightweight RSLG executor using direct /cmd_vel without Nav2",
        "target_room_arrival": runtime.get("target_room_arrival"),
        "approach_position_reached": (runtime.get("approach") or {}).get("approach_position_tolerance_reached"),
        "yaw_aligned": (runtime.get("yaw") or {}).get("yaw_alignment_success"),
        "object_facing_success": runtime.get("success"),
        "route_length_m": route.get("route_length_m"),
        "trajectory_length_m": round(route_length(trajectory), 6) if trajectory else None,
        "runtime_duration_sec": route_time,
        "final_distance_to_room_terminal_m": (runtime.get("room_follow") or {}).get("final_distance_m"),
        "final_distance_to_approach_candidate_m": (runtime.get("approach") or {}).get("final_distance_to_approach_candidate_m"),
        "final_yaw_error_rad": (runtime.get("yaw") or {}).get("final_yaw_error_rad"),
        "wall_crossing_validation_passed": runtime.get("wall_crossing_validation_passed"),
        "fallback_usage": False,
        "failure_layer": runtime.get("failure_layer"),
        "failure_reason": runtime.get("failure_reason"),
    }
    reference = {
        "backend": "Nav2 plus direct /cmd_vel yaw fallback",
        "target_room_arrival": task16.get("target_room_arrival"),
        "approach_position_reached": task16.get("approach_position_reached"),
        "yaw_aligned": task16.get("approach_yaw_aligned"),
        "object_facing_success": task16.get("object_facing_approach_success"),
        "route_length_m": None,
        "trajectory_length_m": None,
        "runtime_duration_sec": None,
        "final_distance_to_room_terminal_m": task16.get("final_distance_to_target_room_terminal_m"),
        "final_distance_to_approach_candidate_m": task16.get("final_distance_to_approach_candidate_m"),
        "final_yaw_error_rad": task16.get("final_yaw_error_rad"),
        "wall_crossing_validation_passed": task16.get("wall_crossing_validation_passed"),
        "fallback_usage": task16.get("fallback_used"),
        "failure_layer": task16.get("failure_layer"),
        "failure_reason": task16.get("failure_reason"),
    }
    payload = {
        "artifact_type": "task17_task16_backend_comparison",
        "created_utc": now_iso(),
        "query": args.query,
        "object_id": args.object_id,
        "task16_reference_path": rel(TASK16_RESULT),
        "task16": reference,
        "task17": task17,
        "scope_statement": "Controlled static-environment Gazebo validation only; this is not dynamic obstacle avoidance or real-world deployment.",
    }
    write_json(out / "task16_task17_comparison_report.json", payload)
    return payload


def execute(args: argparse.Namespace, route: dict[str, Any], topology: dict[str, Any], semantic: dict[str, Any], candidate: dict[str, Any], proxy: dict[str, Any], planner: OccupancyPlanner, out: Path) -> dict[str, Any]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env.setdefault("TURTLEBOT3_MODEL", "burger")
    # rclpy reads domain configuration from this process, while bringup uses
    # the explicit subprocess environment below.
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ.setdefault("TURTLEBOT3_MODEL", env["TURTLEBOT3_MODEL"])
    run_dir = out / "runtime_run"
    log_dir = run_dir / "bringup_logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    pre_process = process_snapshot()
    write_json(run_dir / "process_list_before_bringup.json", pre_process)
    profile = args.stage_output_dir / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
    start_command = [
        str(LAUNCHER), "--stage-output-dir", str(args.stage_output_dir), "--floor-id", args.floor_id,
        "--map-yaml", str(planner.map_yaml), "--runtime-profile", str(profile), "--ros-domain-id", str(args.ros_domain_id),
        "--log-dir", str(log_dir), "--gui" if args.gui else "--headless",
    ]
    stop_command = [str(LAUNCHER), "--stage-output-dir", str(args.stage_output_dir), "--runtime-profile", str(profile), "--ros-domain-id", str(args.ros_domain_id), "--log-dir", str(log_dir), "--stop"]
    write_json(out / "exact_runtime_commands.json", {
        "artifact_type": "task17_exact_runtime_commands",
        "created_utc": now_iso(),
        "top_level_command": sys.argv,
        "launcher_start_command": start_command,
        "launcher_stop_command": stop_command,
        "environment": {"ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"], "TURTLEBOT3_MODEL": env["TURTLEBOT3_MODEL"]},
        "prohibited_nav2_actions_not_called": sorted(NAV2_ACTIONS),
    })
    bringup_log = (run_dir / "no_nav2_bringup.log")
    launch_result = subprocess.run(start_command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    write_text(bringup_log, launch_result.stdout)
    graph = graph_snapshot(env)
    process_after = process_snapshot()
    write_json(run_dir / "ros_graph_no_nav2.json", graph)
    write_json(run_dir / "process_list_no_nav2.json", process_after)
    topics = graph.get("topics", {}).get("stdout") or ""
    ready = bool(
        launch_result.returncode == 0
        and graph["no_nav2_actions_or_nodes_present"]
        and process_after["no_nav2_processes_present"]
        and "/clock" in topics
    )
    readiness = {
        "artifact_type": "task17_bringup_readiness",
        "created_utc": now_iso(),
        "gazebo_started_without_nav2": ready,
        "launcher_returncode": launch_result.returncode,
        "gazebo_clock_present": "/clock" in topics,
        "cmd_vel_publisher_will_be_provided_by_lightweight_executor": True,
        "requested_pose_source": args.pose_source,
        "no_nav2_action_servers_active": not graph["forbidden_nav2_actions"],
        "no_nav2_nodes_active": not graph["forbidden_nav2_nodes"] and process_after["no_nav2_processes_present"],
        "failure_reason": None if ready else "Gazebo no-Nav2 bringup readiness or no-Nav2 graph assertion failed",
    }
    write_json(run_dir / "bringup_readiness.json", readiness)
    runtime: dict[str, Any]
    try:
        if not ready:
            runtime = {"success": False, "failure_layer": "no_nav2_bringup", "failure_reason": readiness["failure_reason"], "target_room_arrival": False}
        else:
            runtime = run_velocity_execution(args, route, candidate, proxy, planner, run_dir)
    finally:
        if args.keep_open_sec > 0 and ready:
            time.sleep(args.keep_open_sec)
        cleanup_result = subprocess.run(stop_command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        final_process = process_snapshot()
        cleanup = {
            "artifact_type": "task17_cleanup_report",
            "created_utc": now_iso(),
            "stop_command": stop_command,
            "stop_returncode": cleanup_result.returncode,
            "stop_output": cleanup_result.stdout,
            "post_cleanup_process_snapshot": final_process,
            "cleanup_passed": cleanup_result.returncode == 0 and final_process["no_nav2_processes_present"],
        }
        write_json(run_dir / "cleanup_report.json", cleanup)
    trajectory = read_json(run_dir / "executed_trajectory.json", {}).get("samples") or []
    if trajectory:
        trajectory_validation = planner.validate_polyline(trajectory, require_inflated=False)
        write_json(run_dir / "path_deviation_report.json", {
            "artifact_type": "task17_runtime_trajectory_validation",
            "created_utc": now_iso(),
            "map_yaml": rel(planner.map_yaml),
            "path_deviation_limit_m": args.path_deviation_limit_m,
            "wall_crossing_validation_passed": trajectory_validation["wall_crossing_validation_passed"],
            "trajectory_occupancy_validation": trajectory_validation,
        })
        runtime["wall_crossing_validation_passed"] = trajectory_validation["wall_crossing_validation_passed"]
        runtime["trajectory_length_m"] = round(route_length(trajectory), 6)
        runtime["runtime_duration_sec"] = trajectory[-1].get("t_sec")
    runtime.update({
        "runtime_execution_attempted": ready,
        "gazebo_started_without_nav2": ready,
        "no_nav2_action_servers_active": not graph["forbidden_nav2_actions"],
        "no_nav2_nodes_active": not graph["forbidden_nav2_nodes"] and process_after["no_nav2_processes_present"],
        "runtime_run_dir": rel(run_dir),
    })
    write_json(out / "runtime_result.json", runtime)
    trajectory = read_json(run_dir / "executed_trajectory.json", {}).get("samples") or []
    manifest_path = out / "marker_manifest.json"
    manifest = read_json(manifest_path, {})
    if manifest:
        manifest["markers_saved"].extend([
            {"type": "executed_trajectory", "source": rel(run_dir / "executed_trajectory.json"), "sample_count": len(trajectory)},
            {"type": "runtime_status_text", "value": "success" if runtime.get("success") else f"failure: {runtime.get('failure_layer')}"},
        ])
        write_json(manifest_path, manifest)
    plot_visualization(planner, route, semantic, candidate, trajectory, out / "route_visualization.png")
    return runtime


def write_summary(args: argparse.Namespace, out: Path, topology: dict[str, Any] | None, route: dict[str, Any] | None, runtime: dict[str, Any], comparison: dict[str, Any] | None) -> None:
    strong_success = bool(runtime.get("success"))
    summary = {
        "artifact_type": "task17_lightweight_rslg_executor_summary",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": args.query,
        "object_id": args.object_id,
        "room_sequence": (topology or {}).get("room_sequence"),
        "map_used_for_execution": rel(stable_map_paths(args)[1]),
        "map_not_used_for_execution": ["segmentation_wall_processed raster", "room-segmentation floorplan/raster products"],
        "backend": "lightweight direct /cmd_vel executor without Nav2",
        "runtime_result": rel(out / "runtime_result.json"),
        "comparison_report": rel(out / "task16_task17_comparison_report.json") if comparison else None,
        "route_length_m": (route or {}).get("route_length_m"),
        **runtime,
        "allowed_claims": [
            "RSLG-SLAM can execute a generated object-navigation route using a lightweight backend without Nav2 in controlled Gazebo simulation.",
            "RSLG-SLAM committed artifacts support query resolution, topology route generation, occupancy-grid path generation, direct velocity-level execution, and runtime validation.",
            "The lightweight executor validates the backend-agnostic nature of RSLG-SLAM executable route artifacts.",
        ] if strong_success else [
            "A lightweight no-Nav2 execution attempt was made and its failure/success evidence was preserved.",
        ],
        "forbidden_claims": [
            "semantic ground-truth accuracy",
            "open-vocabulary CLIP retrieval",
            "visual object found or visual object confirmation",
            "dynamic obstacle avoidance",
            "real-world deployment",
            "SLAM front-end accuracy",
            "cross-floor navigation or TurtleBot3 stair traversal",
            "full collision-free guarantee beyond occupancy-grid validation",
            "RViz visible validation without manually informative screenshot evidence",
            "Nav2 execution for task17",
        ],
    }
    write_json(out / "summary.json", summary)
    lines = [
        "# task17 Lightweight RSLG Executor Without Nav2",
        "",
        f"- Query/object: `{args.query}` / `{args.object_id}`",
        f"- Generated topology route: `{' -> '.join((topology or {}).get('room_sequence') or [])}`",
        f"- Backend: `direct /cmd_vel lightweight controller without Nav2`",
        f"- Stable occupancy map used for execution: `{rel(stable_map_paths(args)[1])}`",
        "- Not used for execution: `segmentation_wall_processed` and room-segmentation raster/floorplan products.",
        "- Route guarantee: occupancy-grid-valid only; not a full robot collision-free guarantee.",
        "",
        "| runtime attempted | no Nav2 nodes/actions | target room | approach position | yaw aligned | object-facing success | failure layer |",
        "| --- | --- | --- | --- | --- | --- | --- |",
        f"| {runtime.get('runtime_execution_attempted')} | {bool(runtime.get('no_nav2_nodes_active') and runtime.get('no_nav2_action_servers_active'))} | {runtime.get('target_room_arrival')} | {(runtime.get('approach') or {}).get('approach_position_tolerance_reached')} | {(runtime.get('yaw') or {}).get('yaw_alignment_success')} | {runtime.get('success')} | {runtime.get('failure_layer')} |",
        "",
        "This is controlled static-environment validation; it does not establish visual object confirmation, dynamic obstacle avoidance, or real-world readiness.",
    ]
    write_text(out / "report.md", "\n".join(lines))


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
    parser.add_argument("--max-linear-speed", type=float, default=0.12)
    parser.add_argument("--max-angular-speed", type=float, default=0.45)
    parser.add_argument("--angular-command-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--lookahead-distance", type=float, default=0.35)
    parser.add_argument("--waypoint-tolerance", type=float, default=0.25)
    parser.add_argument("--goal-tolerance", type=float, default=0.30)
    parser.add_argument("--approach-position-tolerance-m", type=float, default=0.35)
    parser.add_argument("--yaw-internal-tolerance-rad", type=float, default=0.40)
    parser.add_argument("--yaw-report-tolerance-rad", type=float, default=0.50)
    parser.add_argument("--xy-drift-limit-m", type=float, default=0.12)
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--path-deviation-limit-m", type=float, default=0.75)
    parser.add_argument("--forward-heading-limit-rad", type=float, default=0.40)
    parser.add_argument("--inflation-radius-m", type=float, default=0.17)
    parser.add_argument("--path-spacing-m", type=float, default=0.20)
    parser.add_argument("--heading-kp", type=float, default=1.2)
    parser.add_argument("--yaw-kp", type=float, default=1.2)
    parser.add_argument("--max-yaw-angular-speed", type=float, default=0.25)
    parser.add_argument("--yaw-timeout-sec", type=float, default=30.0)
    args = parser.parse_args()
    args.stage_output_dir = args.stage_output_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    if args.map_yaml:
        args.map_yaml = args.map_yaml.resolve()
    if not args.object_id:
        args.object_id = None
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    controller_parameters(args, out)
    artifacts, failure = load_artifacts(args, out)
    runtime: dict[str, Any] = {"runtime_execution_attempted": False, "success": False, "failure_layer": None, "failure_reason": None}
    if failure:
        runtime.update({"failure_layer": "artifact_loading", "failure_reason": failure})
        write_json(out / "runtime_result.json", runtime)
        write_summary(args, out, None, None, runtime, None)
        return 1
    selected, resolution = resolve_query(args, out)
    if selected is None:
        runtime.update({"failure_layer": "query_resolution", "failure_reason": resolution["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        write_summary(args, out, None, None, runtime, None)
        return 1
    args.object_id = selected["object_id"]
    topology = generate_topology_route(args, selected, artifacts, out)
    if not topology["route_generated"]:
        runtime.update({"failure_layer": "topology_route_generation", "failure_reason": topology["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        write_summary(args, out, topology, None, runtime, None)
        return 1
    semantic = semantic_anchors(args, topology, artifacts, out)
    planner = OccupancyPlanner(stable_map_paths(args)[1], args.inflation_radius_m, args.path_spacing_m)
    route, route_report = executable_route(args, semantic, planner, out)
    if route is None:
        runtime.update({"failure_layer": "executable_route_generation", "failure_reason": route_report["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        write_summary(args, out, topology, None, runtime, None)
        return 1
    candidate, proxy = approach_candidate(args, selected, semantic, artifacts, planner, out)
    if candidate is None or proxy is None:
        runtime.update({"failure_layer": "approach_candidate_generation", "failure_reason": "valid approach candidate or yaw proxy unavailable"})
        write_json(out / "runtime_result.json", runtime)
        write_summary(args, out, topology, route, runtime, None)
        return 1
    marker_manifest(args, topology, semantic, route, candidate, proxy, out)
    plot_visualization(planner, route, semantic, candidate, [], out / "route_visualization.png")
    if args.execute:
        runtime = execute(args, route, topology, semantic, candidate, proxy, planner, out)
        approach_result = runtime.get("approach") or {}
        yaw_result = runtime.get("yaw") or {}
        room_result = runtime.get("room_follow") or {}
        runtime.update({
            "query_resolution_success": True,
            "dynamic_room_route_generated": True,
            "executable_route_generated": True,
            "approach_candidate_generated_or_selected": True,
            "yaw_proxy_available": True,
            "query": args.query,
            "object_id": selected["object_id"],
            "target_room": selected["room_id"],
            "floor_id": args.floor_id,
            "route_execution_attempted": runtime.get("runtime_execution_attempted"),
            "approach_position_reached": bool(approach_result.get("approach_position_tolerance_reached")),
            "approach_yaw_aligned": bool(yaw_result.get("yaw_alignment_success")),
            "object_facing_approach_success": bool(runtime.get("success")),
            "final_distance_to_target_room_terminal_m": room_result.get("final_distance_m"),
            "final_distance_to_approach_candidate_m": approach_result.get("final_distance_to_approach_candidate_m"),
            "final_yaw_error_rad": yaw_result.get("final_yaw_error_rad"),
            "final_yaw_error_deg": yaw_result.get("final_yaw_error_deg"),
            "xy_drift_during_yaw_alignment_m": yaw_result.get("xy_drift_during_yaw_alignment_m"),
        })
        write_json(out / "runtime_result.json", runtime)
    else:
        runtime = {
            "runtime_execution_attempted": False,
            "success": False,
            "failure_layer": None,
            "failure_reason": None,
            "query_resolution_success": True,
            "dynamic_room_route_generated": True,
            "executable_route_generated": True,
            "plan_only": True,
        }
        write_json(out / "runtime_result.json", runtime)
    comparison = comparison_report(args, runtime, route, out)
    write_summary(args, out, topology, route, runtime, comparison)
    return 0 if args.plan_only or runtime.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
