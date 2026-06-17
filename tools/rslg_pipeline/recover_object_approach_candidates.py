#!/home/ws/miniconda3/envs/boxfusion/bin/python
"""Recover a conservative object approach and package task40 static evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
from scipy import ndimage

from tools.object_nav.lightweight_backend.occupancy_planner import OccupancyPlanner


REPO = Path(__file__).resolve().parents[2]
SCENE = "00843-DYehNKdT76V"
CANONICAL = REPO / "stage_outputs/rslg_slam" / SCENE / "canonical"
LAYER1 = CANONICAL / "layer1_world_model"
LAYER2 = CANONICAL / "layer2_formal_artifacts"
LAYER3 = CANONICAL / "layer3_navigation_interface"
LAYER4 = CANONICAL / "layer4_runtime_validation"
TASK = (
    REPO
    / "stage_outputs/rslg_slam"
    / SCENE
    / "tasks"
    / "task40_object_level_navigation_interface_recovery_and_candidate_approach_selection"
)

PROJECT = "RSLG-SLAM"
TASK_NAME = "task40_object_level_navigation_interface_recovery_and_candidate_approach_selection"
PROFILE = "conservative_canonical"
OBJECT_ID = "obj_175"
OBJECT_LABEL = "curtain"
TARGET_FLOOR = "floor_2"
TARGET_ROOM = "room_14"
QUERY = "curtain in room_14 on floor_2"
RESOLUTION = 0.05
ORIGIN = (-50.0, -50.0)
INFLATION_RADIUS_M = 0.20
WAYPOINT_SPACING_M = 0.20
MAX_GOAL_SNAP_M = math.sqrt(2.0) * RESOLUTION
RUNTIME_AUTHORIZED = os.environ.get("RSLG_TASK40_ALLOW_RUNTIME") == "1"

DOCS_READ = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/canonical_output_plan.md",
    "docs/rslg_slam/final_layer2_minimal_generation_plan.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/tool_migration_plan.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
    "docs/rslg_slam/canonical_pipeline_runbook.md",
    "docs/rslg_slam/runtime_validation_runbook.md",
    "docs/rslg_slam/current_project_status.md",
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json",
    "docs/rslg_slam/manifests/tool_entrypoint_mapping_manifest_v0_1.json",
    "docs/rslg_slam/manifests/tool_migration_plan_manifest_v0_1.json",
    "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json",
    "docs/rslg_slam/manifests/legacy_inventory_manifest_v0_1.json",
]

TOOLS_USED = [
    "tools/rslg_pipeline/recover_object_approach_candidates.py",
    "tools/object_nav/run_dynamic_object_query_nav.py",
    "tools/object_nav/run_objectnav_multiquery_validation.py",
    "tools/object_nav/validate_object_approach_candidate_sanity.py",
    "tools/object_nav/lightweight_backend/occupancy_planner.py",
    "tools/rslg_pipeline/build_layer2_formal_artifacts.py",
    "tools/rslg_pipeline/build_layer3_navigation_interface.py",
]

SNAPSHOT = (
    LAYER1
    / "raw_outputs"
    / SCENE
    / "logs"
    / "final_vector_map_snapshot.json"
)
TOPOLOGY = LAYER1 / "raw_outputs" / SCENE / "logs" / "topology_v0_1.json"
ROOM_WORLD = (
    LAYER1
    / "raw_outputs"
    / SCENE
    / "logs"
    / "committed_room_world_model_v0_1.json"
)
QUERY_V01 = LAYER2 / "object_interfaces/object_query_resolution_v0_1.json"
APPROACH_V01 = LAYER2 / "object_interfaces/object_approach_v0_1.json"
PACKAGE_V01 = LAYER2 / "object_interfaces/object_interface_package_v0_1.json"
MAP_PACKAGE = LAYER2 / "stable_maps/stable_occupancy_map_package_v0_1.json"
MAP_NPZ = LAYER2 / "stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.npz"
MAP_YAML = LAYER2 / "stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.yaml"
MAP_METADATA = (
    LAYER2
    / "stable_maps/floor_2/floor_2_stable_occupancy_map_metadata_v0_1.json"
)
ROOM_ROUTE = (
    LAYER3
    / "real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json"
)
ROOM_SELECTED = (
    LAYER3
    / "executable_route_candidates/cross_floor_room_executable_route_candidate_selected_v0_1.json"
)
OBJECT_CONTRACT_V01 = (
    LAYER3 / "route_contracts/cross_floor_object_route_contract_v0_1.json"
)
OBJECT_REQUEST_V01 = (
    LAYER3 / "planner_requests/cross_floor_object_planner_request_v0_1.json"
)
OBJECT_PLAN_V01 = LAYER3 / "route_plans/cross_floor_object_route_plan_v0_1.json"
TASK39_EXECUTION = (
    LAYER4 / "validation_reports/runtime_execution_validation_report_v0_1.json"
)
TASK39_FOLLOWING = (
    LAYER4 / "validation_reports/route_following_validation_report_v0_1.json"
)
TASK39_TRAJECTORY = (
    LAYER4 / "trajectories/executed_trajectory_summary_v0_1.json"
)

L2_CANDIDATES = (
    LAYER2 / "object_interfaces/object_approach_candidates_recovery_v0_1.json"
)
L2_SELECTED = LAYER2 / "object_interfaces/object_approach_selected_v0_2.json"
L2_PACKAGE = LAYER2 / "object_interfaces/object_interface_package_v0_2.json"
L2_VALIDATION = (
    LAYER2
    / "object_interfaces/object_approach_recovery_validation_report_v0_1.json"
)
L2_ADDENDUM = (
    LAYER2
    / "manifests/object_interface_recovery_addendum_manifest_v0_1.json"
)
L3_CONTRACT = (
    LAYER3 / "route_contracts/cross_floor_object_route_contract_v0_2.json"
)
L3_REQUEST = (
    LAYER3 / "planner_requests/cross_floor_object_planner_request_v0_2.json"
)
L3_PLAN = LAYER3 / "route_plans/cross_floor_object_route_plan_v0_2.json"
L3_ROUTE = (
    LAYER3
    / "real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json"
)
L3_SELECTED = (
    LAYER3
    / "executable_route_candidates/cross_floor_object_executable_route_candidate_selected_v0_2.json"
)
L3_VALIDATION = (
    LAYER3
    / "reports/cross_floor_object_route_generation_validation_report_v0_2.json"
)
L4_READINESS = (
    LAYER4 / "object_readiness/object_runtime_readiness_report_v0_2.json"
)
L4_RUNTIME_INPUT = (
    LAYER4 / "object_readiness/object_route_runtime_input_v0_1.json"
)
L4_PREFLIGHT = (
    LAYER4 / "object_readiness/object_route_preflight_report_v0_1.json"
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def read_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    return {
        "path": rel(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def canonical_object_id(value: Any) -> str:
    text = str(value)
    return text if text.startswith("obj_") else f"obj_{text}"


def distance(a: Iterable[float], b: Iterable[float]) -> float:
    ax, ay = [float(value) for value in a][:2]
    bx, by = [float(value) for value in b][:2]
    return math.hypot(ax - bx, ay - by)


def polyline_length(points: list[dict[str, Any]]) -> float:
    return sum(
        distance(
            (left["x"], left["y"]),
            (right["x"], right["y"]),
        )
        for left, right in zip(points, points[1:])
    )


def closest_point_on_segment(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> tuple[float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom <= 1.0e-12:
        return a
    t = max(
        0.0,
        min(
            1.0,
            ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / denom,
        ),
    )
    return a[0] + t * dx, a[1] + t * dy


def distance_to_polygon_boundary(
    point: tuple[float, float], polygon: list[list[float]]
) -> float:
    if len(polygon) < 2:
        return float("inf")
    return min(
        distance(
            point,
            closest_point_on_segment(
                point,
                tuple(map(float, polygon[index])),
                tuple(map(float, polygon[(index + 1) % len(polygon)])),
            ),
        )
        for index in range(len(polygon))
    )


def point_in_polygon(
    x: float,
    y: float,
    polygon: list[list[float]],
    *,
    include_boundary: bool = True,
) -> bool:
    if len(polygon) < 3:
        return False
    if include_boundary and distance_to_polygon_boundary((x, y), polygon) <= 1.0e-7:
        return True
    inside = False
    previous = len(polygon) - 1
    for index, (xi_raw, yi_raw) in enumerate(polygon):
        xj_raw, yj_raw = polygon[previous]
        xi, yi = float(xi_raw), float(yi_raw)
        xj, yj = float(xj_raw), float(yj_raw)
        if (yi > y) != (yj > y):
            crossing_x = (xj - xi) * (y - yi) / ((yj - yi) or 1.0e-12) + xi
            if x < crossing_x:
                inside = not inside
        previous = index
    return inside


def sample_polygon_edges(
    polygon: list[list[float]], spacing_m: float = 0.025
) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    for index, start_raw in enumerate(polygon):
        end_raw = polygon[(index + 1) % len(polygon)]
        start = tuple(map(float, start_raw))
        end = tuple(map(float, end_raw))
        length = distance(start, end)
        steps = max(1, int(math.ceil(length / spacing_m)))
        for step in range(steps + 1):
            ratio = step / steps
            samples.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
    return samples


class StableMap:
    def __init__(self, path: Path) -> None:
        with np.load(path, allow_pickle=False) as archive:
            self.free = archive["free_mask"].astype(bool)
            self.occupied = archive["occupied_mask"].astype(bool)
            self.unknown = archive["unknown_mask"].astype(bool)
            self.outside = archive["outside_boundary"].astype(bool)
            self.wall = archive["wall_evidence"].astype(bool)
            self.gateway_wall = archive["gateway_wall_preclose"].astype(bool)
            self.room_labels = archive["repaired_room_labels"].astype(np.int32)
        self.clearance = ndimage.distance_transform_edt(self.free) * RESOLUTION

    def rc(self, xy: Iterable[float]) -> tuple[int, int]:
        x, y = [float(value) for value in xy][:2]
        return (
            int(round((y - ORIGIN[1]) / RESOLUTION)),
            int(round((x - ORIGIN[0]) / RESOLUTION)),
        )

    def xy(self, rc: tuple[int, int]) -> tuple[float, float]:
        return (
            ORIGIN[0] + rc[1] * RESOLUTION,
            ORIGIN[1] + rc[0] * RESOLUTION,
        )

    def in_bounds(self, rc: tuple[int, int]) -> bool:
        return (
            0 <= rc[0] < self.free.shape[0]
            and 0 <= rc[1] < self.free.shape[1]
        )

    def sample(self, xy: Iterable[float]) -> dict[str, Any]:
        row, col = self.rc(xy)
        if not self.in_bounds((row, col)):
            return {
                "grid_rc": [row, col],
                "in_bounds": False,
                "state": "out_of_bounds",
                "free": False,
                "occupied": False,
                "unknown": False,
                "outside_boundary": True,
                "wall_evidence": False,
                "wall_core": False,
                "gateway_wall_evidence": False,
                "clearance_m": None,
                "room_label": None,
            }
        state = (
            "free"
            if self.free[row, col]
            else "occupied"
            if self.occupied[row, col]
            else "unknown"
        )
        return {
            "grid_rc": [row, col],
            "in_bounds": True,
            "state": state,
            "free": bool(self.free[row, col]),
            "occupied": bool(self.occupied[row, col]),
            "unknown": bool(self.unknown[row, col]),
            "outside_boundary": not bool(self.outside[row, col]),
            "wall_evidence": bool(self.wall[row, col]),
            "wall_core": bool(self.wall[row, col] and self.occupied[row, col]),
            "gateway_wall_evidence": bool(self.gateway_wall[row, col]),
            "clearance_m": round(float(self.clearance[row, col]), 6),
            "room_label": int(self.room_labels[row, col]),
        }

    def ray_check(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        endpoint_footprint: list[list[float]] | None = None,
    ) -> dict[str, Any]:
        length = distance(start, end)
        steps = max(2, int(math.ceil(length / (RESOLUTION * 0.45))))
        counts = {
            "free": 0,
            "occupied": 0,
            "unknown": 0,
            "out_of_bounds": 0,
            "excluded_target_footprint": 0,
        }
        invalid: list[dict[str, Any]] = []
        tested = 0
        for index in range(steps + 1):
            ratio = index / steps
            xy = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            if (
                endpoint_footprint
                and index > 0
                and point_in_polygon(*xy, endpoint_footprint)
            ):
                counts["excluded_target_footprint"] += 1
                continue
            tested += 1
            sample = self.sample(xy)
            state = sample["state"]
            counts[state] = counts.get(state, 0) + 1
            if state != "free" and len(invalid) < 20:
                invalid.append(
                    {
                        "sample_index": index,
                        "ratio": round(ratio, 6),
                        "world_xy": [round(xy[0], 6), round(xy[1], 6)],
                        "state": state,
                        "grid_rc": sample["grid_rc"],
                    }
                )
        passed = (
            tested > 0
            and counts["occupied"] == 0
            and counts["unknown"] == 0
            and counts["out_of_bounds"] == 0
        )
        return {
            "status": "passed" if passed else "failed",
            "passed": passed,
            "distance_m": round(length, 6),
            "tested_sample_count": tested,
            "state_counts": counts,
            "invalid_samples": invalid,
            "policy": (
                "endpoint-inclusive"
                if endpoint_footprint is None
                else "object-footprint-aware endpoint exclusion"
            ),
        }


def generated_ring_candidates(object_record: Mapping[str, Any]) -> list[dict[str, Any]]:
    footprint = object_record["footprint_2d"]
    xs = [float(point[0]) for point in footprint]
    ys = [float(point[1]) for point in footprint]
    half_width = max(0.05, (max(xs) - min(xs)) / 2.0)
    half_height = max(0.05, (max(ys) - min(ys)) / 2.0)
    center_x, center_y = map(float, object_record["pose"])
    candidates: list[dict[str, Any]] = []
    candidate_number = 0
    for standoff in (0.6, 0.8, 1.0):
        for angle_deg in range(0, 360, 22):
            angle = math.radians(angle_deg)
            unit_x, unit_y = math.cos(angle), math.sin(angle)
            edge_x = (
                half_width / abs(unit_x)
                if abs(unit_x) > 1.0e-9
                else float("inf")
            )
            edge_y = (
                half_height / abs(unit_y)
                if abs(unit_y) > 1.0e-9
                else float("inf")
            )
            support = min(edge_x, edge_y)
            xy = (
                center_x + unit_x * (support + standoff),
                center_y + unit_y * (support + standoff),
            )
            candidates.append(
                {
                    "candidate_id": f"generated_ring_{candidate_number:03d}",
                    "candidate_source": "regenerated_existing_bbox_support_plus_standoff_policy",
                    "world_xy": [round(xy[0], 6), round(xy[1], 6)],
                    "floor_id": TARGET_FLOOR,
                    "room_id": TARGET_ROOM,
                    "angle_deg": angle_deg,
                    "standoff_from_footprint_m": standoff,
                    "object_centroid_navigation_used": False,
                    "direct_object_centroid_goal_used": False,
                }
            )
            candidate_number += 1
    return candidates


def polyline_cell_metrics(
    stable_map: StableMap, points: list[dict[str, Any]]
) -> dict[str, Any]:
    cells: list[tuple[int, int]] = []
    for left, right in zip(points, points[1:]):
        length = distance((left["x"], left["y"]), (right["x"], right["y"]))
        steps = max(1, int(math.ceil(length / (RESOLUTION * 0.45))))
        for index in range(steps + 1):
            ratio = index / steps
            cells.append(
                stable_map.rc(
                    (
                        float(left["x"])
                        + ratio * (float(right["x"]) - float(left["x"])),
                        float(left["y"])
                        + ratio * (float(right["y"]) - float(left["y"])),
                    )
                )
            )
    unique_cells = list(dict.fromkeys(cells))
    occupied = []
    unknown = []
    outside = []
    wall_contact = []
    occupied_wall = []
    invalid_samples = []
    for cell in unique_cells:
        if not stable_map.in_bounds(cell):
            outside.append(cell)
            invalid_samples.append({"grid_rc": list(cell), "state": "out_of_bounds"})
            continue
        row, col = cell
        if stable_map.occupied[row, col]:
            occupied.append(cell)
            invalid_samples.append({"grid_rc": list(cell), "state": "occupied"})
        if stable_map.unknown[row, col]:
            unknown.append(cell)
            invalid_samples.append({"grid_rc": list(cell), "state": "unknown"})
        if not stable_map.outside[row, col]:
            outside.append(cell)
        if stable_map.wall[row, col]:
            wall_contact.append(cell)
        if stable_map.wall[row, col] and stable_map.occupied[row, col]:
            occupied_wall.append(cell)
    return {
        "tested_route_cell_count": len(unique_cells),
        "occupied_cells_crossed": len(occupied),
        "unknown_cells_crossed": len(unknown),
        "outside_boundary_cells_crossed": len(outside),
        "wall_evidence_cells_contacted": len(wall_contact),
        "occupied_wall_cells_crossed": len(occupied_wall),
        "wall_crossing_count": len(occupied_wall),
        "invalid_samples": invalid_samples[:30],
        "wall_metric_definition": (
            "Free-cell wall-evidence contact is reported separately. A wall crossing "
            "requires a cell jointly classified occupied and wall evidence."
        ),
    }


def evaluate_candidate(
    candidate: dict[str, Any],
    *,
    stable_map: StableMap,
    planner: OccupancyPlanner,
    object_record: Mapping[str, Any],
    room: Mapping[str, Any],
    other_room_objects: list[Mapping[str, Any]],
    route_terminal_xy: tuple[float, float],
) -> dict[str, Any]:
    xy = tuple(map(float, candidate["world_xy"]))
    object_xy = tuple(map(float, object_record["pose"]))
    footprint = object_record["footprint_2d"]
    room_polygon = room["polygon"]
    visible_points = [
        point
        for point in sample_polygon_edges(footprint)
        if point_in_polygon(*point, room_polygon)
    ]
    visible_proxy = (
        min(visible_points, key=lambda point: distance(point, xy))
        if visible_points
        else None
    )
    yaw_target = visible_proxy or object_xy
    yaw = math.atan2(yaw_target[1] - xy[1], yaw_target[0] - xy[0])
    sample = stable_map.sample(xy)
    overlaps = [
        {
            "object_id": canonical_object_id(other["id"]),
            "object_label": other.get("label") or other.get("category"),
        }
        for other in other_room_objects
        if point_in_polygon(*xy, other["footprint_2d"])
    ]
    inside_room = point_in_polygon(*xy, room_polygon)
    target_overlap = point_in_polygon(*xy, footprint)
    strict_ray = stable_map.ray_check(xy, object_xy)
    footprint_ray = (
        stable_map.ray_check(
            xy,
            visible_proxy,
            endpoint_footprint=footprint,
        )
        if visible_proxy
        else None
    )
    checks = {
        "inside_map_bounds": sample["in_bounds"],
        "inside_room_14_navigable_region": inside_room,
        "conservative_canonical_free_cell": sample["state"] == "free",
        "clearance_at_least_0_20m": (
            sample["clearance_m"] is not None
            and float(sample["clearance_m"]) >= INFLATION_RADIUS_M
        ),
        "not_outside_boundary": not sample["outside_boundary"],
        "not_unknown": not sample["unknown"],
        "not_occupied": not sample["occupied"],
        "not_wall_core": not sample["wall_core"],
        "not_inside_target_object_footprint": not target_overlap,
        "not_inside_other_object_footprint": not overlaps,
        "visible_object_footprint_proxy_available": visible_proxy is not None,
        "object_footprint_aware_ray_clear": bool(
            footprint_ray and footprint_ray["passed"]
        ),
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }
    local_pass = all(
        value
        for key, value in checks.items()
        if key
        not in {
            "object_centroid_navigation_used",
            "direct_object_centroid_goal_used",
        }
    )
    route: dict[str, Any] = {
        "attempted": False,
        "a_star_reachable": False,
        "final_candidate_reached": False,
    }
    if local_pass:
        route["attempted"] = True
        try:
            points, planner_metrics = planner.plan_segment(
                route_terminal_xy,
                xy,
                f"room_14_route_terminal_to_{candidate['candidate_id']}",
            )
            exact_goal = {"x": round(xy[0], 6), "y": round(xy[1], 6)}
            if distance((points[-1]["x"], points[-1]["y"]), xy) > 1.0e-9:
                points.append(exact_goal)
            exact_validation = planner.validate_polyline(
                points, require_inflated=True
            )
            cell_metrics = polyline_cell_metrics(stable_map, points)
            final_distance = distance(
                (points[-1]["x"], points[-1]["y"]),
                xy,
            )
            route.update(
                {
                    "a_star_reachable": True,
                    "waypoints": points,
                    "planner_metrics": planner_metrics,
                    "exact_endpoint_path_length_m": round(
                        polyline_length(points), 6
                    ),
                    "exact_endpoint_waypoint_count": len(points),
                    "exact_goal_polyline_validation": exact_validation,
                    "cell_metrics": cell_metrics,
                    "final_candidate_distance_m": round(final_distance, 6),
                    "final_candidate_reached": final_distance <= 1.0e-6,
                    "zero_occupied_crossings": cell_metrics[
                        "occupied_cells_crossed"
                    ]
                    == 0,
                    "zero_unknown_crossings": cell_metrics[
                        "unknown_cells_crossed"
                    ]
                    == 0,
                    "zero_outside_boundary_crossings": cell_metrics[
                        "outside_boundary_cells_crossed"
                    ]
                    == 0,
                    "zero_wall_crossings": cell_metrics["wall_crossing_count"]
                    == 0,
                }
            )
        except Exception as exc:
            route["error"] = f"{type(exc).__name__}: {exc}"
    route_pass = bool(
        route.get("a_star_reachable")
        and route.get("final_candidate_reached")
        and route.get("exact_goal_polyline_validation", {}).get(
            "wall_crossing_validation_passed"
        )
        and route.get("zero_occupied_crossings")
        and route.get("zero_unknown_crossings")
        and route.get("zero_outside_boundary_crossings")
        and route.get("zero_wall_crossings")
    )
    rejection_reasons = [
        key
        for key, passed in checks.items()
        if passed is False
        and key
        not in {
            "object_centroid_navigation_used",
            "direct_object_centroid_goal_used",
        }
    ]
    if local_pass and not route_pass:
        rejection_reasons.append("a_star_or_route_safety_validation_failed")
    return {
        **candidate,
        "yaw": round(yaw, 6),
        "yaw_policy": "face_nearest_room_side_object_footprint_proxy",
        "visible_object_footprint_proxy_xy": (
            [round(visible_proxy[0], 6), round(visible_proxy[1], 6)]
            if visible_proxy
            else None
        ),
        "object_centroid_xy_for_geometry_and_comparison_only": list(object_xy),
        "distance_to_object_centroid_m": round(distance(xy, object_xy), 6),
        "distance_to_visible_object_footprint_proxy_m": (
            round(distance(xy, visible_proxy), 6) if visible_proxy else None
        ),
        "distance_to_selected_room_route_terminal_m": round(
            distance(xy, route_terminal_xy), 6
        ),
        "stable_map_sample": sample,
        "semantic_object_footprint_overlaps": overlaps,
        "strict_centroid_ray_analysis_context": strict_ray,
        "object_footprint_aware_ray_check": footprint_ray,
        "checks": checks,
        "local_static_checks_passed": local_pass,
        "a_star_route_evaluation": route,
        "accepted": local_pass and route_pass,
        "rejected_reasons": rejection_reasons,
        "safer_than_generated_ring_037": bool(
            local_pass
            and route_pass
            and candidate["candidate_id"] != "generated_ring_037"
        ),
    }


def validate_required_inputs() -> tuple[list[dict[str, Any]], list[str]]:
    required = [
        SNAPSHOT,
        TOPOLOGY,
        ROOM_WORLD,
        QUERY_V01,
        APPROACH_V01,
        PACKAGE_V01,
        MAP_PACKAGE,
        MAP_NPZ,
        MAP_YAML,
        MAP_METADATA,
        OBJECT_CONTRACT_V01,
        OBJECT_REQUEST_V01,
        OBJECT_PLAN_V01,
        ROOM_ROUTE,
        ROOM_SELECTED,
        TASK39_EXECUTION,
        TASK39_FOLLOWING,
        TASK39_TRAJECTORY,
    ]
    records = [{"path": rel(path), "exists": path.is_file()} for path in required]
    missing = [record["path"] for record in records if not record["exists"]]
    return records, missing


def candidate_selection_key(record: Mapping[str, Any]) -> tuple[Any, ...]:
    route = record["a_star_route_evaluation"]
    clearance = float(record["stable_map_sample"]["clearance_m"] or 0.0)
    path_length = float(route["exact_endpoint_path_length_m"])
    route_distance = float(record["distance_to_selected_room_route_terminal_m"])
    return (
        -clearance,
        path_length,
        route_distance,
        record["candidate_id"],
    )


def make_layer2_artifacts(
    generated_at: str,
    records: list[dict[str, Any]],
    selected: dict[str, Any],
) -> None:
    candidates_payload = {
        "schema_name": "rslg_object_approach_candidates_recovery",
        "schema_version": "0.1",
        "classification": "object_approach_candidates_recovery_generated",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 2: Formal Artifact Layer",
        "generated_utc": generated_at,
        "query": QUERY,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "target_floor": TARGET_FLOOR,
        "target_room": TARGET_ROOM,
        "selected_profile": PROFILE,
        "generation_policy": {
            "name": "existing_bbox_support_plus_standoff",
            "standoff_m": [0.6, 0.8, 1.0],
            "angle_degrees": list(range(0, 360, 22)),
            "candidate_count": len(records),
            "manual_coordinate_selection": False,
            "object_centroid_goal_generation": False,
        },
        "ray_policy": {
            "selected_policy": "object-footprint-aware endpoint exclusion",
            "reason": (
                "obj_175 is a wall-attached curtain. Occupancy inside the target "
                "footprint is not required to be traversable, while every ray sample "
                "before the footprint must remain free."
            ),
            "strict_centroid_ray_retained_as_analysis_context": True,
            "all_obstruction_evidence_ignored": False,
        },
        "candidate_records": records,
        "accepted_candidate_ids": [
            record["candidate_id"] for record in records if record["accepted"]
        ],
        "selected_candidate_id": selected["candidate_id"],
        "source_files": [
            rel(SNAPSHOT),
            rel(TOPOLOGY),
            rel(MAP_NPZ),
            rel(MAP_YAML),
            rel(ROOM_ROUTE),
        ],
        "source_boundary": {
            "canonical_rslg_slam_outputs_only": True,
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "simulator_navmesh_used": False,
            "manual_object_pose_used": False,
            "manual_approach_pose_used": False,
        },
    }
    write_json(L2_CANDIDATES, candidates_payload)

    selected_payload = {
        "schema_name": "rslg_object_approach_selected",
        "schema_version": "0.2",
        "classification": "object_approach_recovered_under_conservative_canonical",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 2: Formal Artifact Layer",
        "is_final_formal_artifact": True,
        "generated_utc": generated_at,
        "supersedes_for_executable_use": rel(APPROACH_V01),
        "preserves_v0_1_blocker_record": True,
        "query": QUERY,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "target_floor": TARGET_FLOOR,
        "target_room": TARGET_ROOM,
        "selected_profile": PROFILE,
        "approach_candidate_id": selected["candidate_id"],
        "world_xy": selected["world_xy"],
        "yaw": selected["yaw"],
        "yaw_policy": selected["yaw_policy"],
        "visible_object_footprint_proxy_xy": selected[
            "visible_object_footprint_proxy_xy"
        ],
        "generation_policy": "existing bbox_support_plus_standoff policy",
        "angle_deg": selected["angle_deg"],
        "standoff_from_footprint_m": selected[
            "standoff_from_footprint_m"
        ],
        "stable_map_sample": selected["stable_map_sample"],
        "local_static_validation_checks": selected["checks"],
        "object_footprint_aware_ray_check": selected[
            "object_footprint_aware_ray_check"
        ],
        "strict_centroid_ray_analysis_context": selected[
            "strict_centroid_ray_analysis_context"
        ],
        "a_star_reachability": selected["a_star_route_evaluation"],
        "validation_status": "validated_under_conservative_canonical",
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "object_centroid_used_only_for_ring_geometry_and_comparison": True,
        "source_files": [
            rel(L2_CANDIDATES),
            rel(QUERY_V01),
            rel(MAP_NPZ),
            rel(ROOM_ROUTE),
        ],
    }
    write_json(L2_SELECTED, selected_payload)

    package_payload = {
        "schema_name": "rslg_object_interface_package",
        "schema_version": "0.2",
        "classification": "object_interface_package_v0_2_recovered",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 2: Formal Artifact Layer",
        "is_final_formal_artifact": True,
        "generated_utc": generated_at,
        "preserved_v0_1_package": rel(PACKAGE_V01),
        "query_resolution_artifact": rel(QUERY_V01),
        "object_approach_candidates_artifact": rel(L2_CANDIDATES),
        "object_approach_selected_artifact": rel(L2_SELECTED),
        "query_resolution_status": "resolved",
        "approach_validation_status": "validated_under_conservative_canonical",
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "target_floor": TARGET_FLOOR,
        "target_room": TARGET_ROOM,
        "approach_candidate_id": selected["candidate_id"],
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "ready_for_layer3_object_route_generation": True,
        "ready_for_layer3_object_contract_generation": True,
        "runtime_artifact": False,
    }
    write_json(L2_PACKAGE, package_payload)

    validation_payload = {
        "schema_name": "rslg_object_approach_recovery_validation_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 2: Formal Artifact Layer",
        "generated_utc": generated_at,
        "status": "passed",
        "selected_profile": PROFILE,
        "selected_candidate_id": selected["candidate_id"],
        "checks": {
            "candidate_generated_by_existing_policy": True,
            "candidate_is_not_generated_ring_037": (
                selected["candidate_id"] != "generated_ring_037"
            ),
            "candidate_free_under_conservative_canonical": (
                selected["stable_map_sample"]["state"] == "free"
            ),
            "candidate_clearance_at_least_0_20m": (
                float(selected["stable_map_sample"]["clearance_m"])
                >= INFLATION_RADIUS_M
            ),
            "candidate_inside_room_14": selected["checks"][
                "inside_room_14_navigable_region"
            ],
            "candidate_not_in_other_object_footprint": selected["checks"][
                "not_inside_other_object_footprint"
            ],
            "object_footprint_aware_ray_clear": selected[
                "object_footprint_aware_ray_check"
            ]["passed"],
            "a_star_reachable": selected["a_star_route_evaluation"][
                "a_star_reachable"
            ],
            "zero_occupied_unknown_outside_or_wall_crossings": all(
                selected["a_star_route_evaluation"][key]
                for key in (
                    "zero_occupied_crossings",
                    "zero_unknown_crossings",
                    "zero_outside_boundary_crossings",
                    "zero_wall_crossings",
                )
            ),
            "object_centroid_navigation_used": False,
            "manual_target_pose_used": False,
            "forbidden_sources_used": False,
        },
        "generated_ring_037_comparison": {
            "candidate_id": "generated_ring_037",
            "accepted": next(
                record["accepted"]
                for record in records
                if record["candidate_id"] == "generated_ring_037"
            ),
            "state": next(
                record["stable_map_sample"]["state"]
                for record in records
                if record["candidate_id"] == "generated_ring_037"
            ),
            "clearance_m": next(
                record["stable_map_sample"]["clearance_m"]
                for record in records
                if record["candidate_id"] == "generated_ring_037"
            ),
        },
        "canonical_object_approach_can_be_upgraded": True,
    }
    write_json(L2_VALIDATION, validation_payload)


def make_layer3_artifacts(
    generated_at: str,
    selected: dict[str, Any],
    room_route: dict[str, Any],
) -> None:
    local_route = selected["a_star_route_evaluation"]
    local_points = [
        {
            **point,
            "floor_id": TARGET_FLOOR,
            "segment_id": f"seg_007_room_14_to_{selected['candidate_id']}",
            "waypoint_source": "OccupancyPlanner A* plus exact candidate endpoint",
        }
        for point in local_route["waypoints"]
    ]
    base_floor_points = {
        floor_id: [dict(point) for point in points]
        for floor_id, points in room_route["route_floor_waypoints"].items()
    }
    if (
        base_floor_points[TARGET_FLOOR]
        and local_points
        and distance(
            (
                base_floor_points[TARGET_FLOOR][-1]["x"],
                base_floor_points[TARGET_FLOOR][-1]["y"],
            ),
            (local_points[0]["x"], local_points[0]["y"]),
        )
        <= 1.0e-9
    ):
        local_points = local_points[1:]
    combined_floor_points = {
        "floor_1": base_floor_points["floor_1"],
        "floor_2": base_floor_points["floor_2"] + local_points,
    }
    local_metrics = local_route["planner_metrics"]
    exact_local_length = float(local_route["exact_endpoint_path_length_m"])
    cell_metrics = local_route["cell_metrics"]
    local_segment = {
        "segment_id": f"seg_007_room_14_to_{selected['candidate_id']}",
        "floor_id": TARGET_FLOOR,
        "source_anchor_id": TARGET_ROOM,
        "source_xy": local_metrics["start_snap"]["requested_world_xy"],
        "target_anchor_id": selected["candidate_id"],
        "target_xy": selected["world_xy"],
        "target_yaw": selected["yaw"],
        "target_yaw_policy": selected["yaw_policy"],
        "planner_type": "astar_8_connected_clearance_penalized",
        "status": "success",
        "map_yaml": rel(MAP_YAML),
        "metrics": {
            **local_metrics,
            **cell_metrics,
            "exact_candidate_endpoint_appended": True,
            "exact_candidate_endpoint_reached": True,
            "exact_endpoint_path_length_m": exact_local_length,
            "exact_endpoint_waypoint_count": local_route[
                "exact_endpoint_waypoint_count"
            ],
        },
        "waypoints": local_route["waypoints"],
    }
    route_metrics = {
        "base_room_route_interface_length_m": room_route["route_metrics"][
            "interface_route_length_m"
        ],
        "local_object_approach_length_m": exact_local_length,
        "local_object_approach_planner_snap_length_m": local_metrics[
            "path_length_m"
        ],
        "combined_interface_route_length_m": round(
            float(room_route["route_metrics"]["interface_route_length_m"])
            + exact_local_length,
            6,
        ),
        "combined_waypoint_count": sum(
            len(points) for points in combined_floor_points.values()
        )
        + 2,
        "minimum_clearance_m": min(
            float(room_route["route_metrics"]["minimum_clearance_m"]),
            float(local_metrics["minimum_clearance_m"]),
        ),
        "local_occupied_cells_crossed": cell_metrics[
            "occupied_cells_crossed"
        ],
        "local_unknown_cells_crossed": cell_metrics["unknown_cells_crossed"],
        "local_outside_boundary_cells_crossed": cell_metrics[
            "outside_boundary_cells_crossed"
        ],
        "local_wall_crossing_count": cell_metrics["wall_crossing_count"],
    }
    validation_checks = {
        "base_room_route_reused_without_overwrite": True,
        "validated_route_chain_preserved": (
            room_route["validated_route_chain"]
            == ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"]
        ),
        "transition_edge_is_vt_1_centerline_e001": (
            room_route["vertical_transition_segment"]["transition_edge"]
            == "vt_1_centerline_e001"
        ),
        "non_transition_edge_is_vt_1_centerline_e003": (
            room_route["vertical_transition_segment"]["non_transition_edge"]
            == "vt_1_centerline_e003"
        ),
        "local_a_star_reachable": local_route["a_star_reachable"],
        "final_approach_candidate_reached": local_route[
            "final_candidate_reached"
        ],
        "zero_occupied_crossings": local_route["zero_occupied_crossings"],
        "zero_unknown_crossings": local_route["zero_unknown_crossings"],
        "zero_outside_boundary_crossings": local_route[
            "zero_outside_boundary_crossings"
        ],
        "zero_wall_crossings": local_route["zero_wall_crossings"],
        "yaw_object_facing_readiness_represented": True,
        "object_centroid_navigation_used": False,
        "manual_target_pose_used": False,
    }
    route_ready = all(
        value
        for key, value in validation_checks.items()
        if key
        not in {
            "object_centroid_navigation_used",
            "manual_target_pose_used",
        }
    )

    contract = {
        "schema_name": "rslg_layer3_route_contract",
        "schema_version": "0.2",
        "contract_id": "cross_floor_object_route_contract_v0_2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "route_kind": "cross_floor_object",
        "status": "ready",
        "generated_utc": generated_at,
        "preserved_v0_1_contract": rel(OBJECT_CONTRACT_V01),
        "base_room_route_contract": (
            "stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/"
            "layer3_navigation_interface/route_contracts/"
            "cross_floor_room_route_contract_v0_1.json"
        ),
        "validated_route_chain": [
            "room_2",
            "room_3",
            "vt_1",
            "room_7",
            "room_13",
            "room_14",
            selected["candidate_id"],
        ],
        "query": QUERY,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "target_floor": TARGET_FLOOR,
        "target_room": TARGET_ROOM,
        "approach_candidate_id": selected["candidate_id"],
        "approach_world_xy": selected["world_xy"],
        "approach_yaw": selected["yaw"],
        "approach_yaw_policy": selected["yaw_policy"],
        "selected_profile_for_layer4": PROFILE,
        "selected_profile_executable_approach_ready": route_ready,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "final_layer2_sources": [
            file_record(QUERY_V01),
            file_record(L2_CANDIDATES),
            file_record(L2_SELECTED),
            file_record(L2_PACKAGE),
            file_record(MAP_PACKAGE),
        ],
    }
    write_json(L3_CONTRACT, contract)

    request = {
        "schema_name": "rslg_layer3_object_planner_request",
        "schema_version": "0.2",
        "request_id": "cross_floor_object_planner_request_v0_2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "generated_utc": generated_at,
        "route_contract": rel(L3_CONTRACT),
        "request_mode": "reuse_validated_room_route_then_local_object_approach_astar",
        "base_room_route": rel(ROOM_ROUTE),
        "selected_profile": PROFILE,
        "approach_candidate_id": selected["candidate_id"],
        "approach_world_xy": selected["world_xy"],
        "approach_yaw": selected["yaw"],
        "inflation_radius_m": INFLATION_RADIUS_M,
        "waypoint_spacing_m": WAYPOINT_SPACING_M,
        "executable_approach_route_requested": True,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
    }
    write_json(L3_REQUEST, request)

    plan = {
        "schema_name": "rslg_layer3_route_plan",
        "schema_version": "0.2",
        "plan_id": "cross_floor_object_route_plan_v0_2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "generated_utc": generated_at,
        "route_contract": rel(L3_CONTRACT),
        "planner_request": rel(L3_REQUEST),
        "base_room_route_plan": (
            "stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/"
            "layer3_navigation_interface/route_plans/"
            "cross_floor_room_route_plan_v0_1.json"
        ),
        "selected_profile": PROFILE,
        "object_id": OBJECT_ID,
        "approach_candidate_id": selected["candidate_id"],
        "object_approach_segment": local_segment,
        "object_approach_segment_status": "ready",
        "executable_object_approach_route_generated": route_ready,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }
    write_json(L3_PLAN, plan)

    object_route = {
        "schema_name": "rslg_cross_floor_object_real_astar_route",
        "schema_version": "0.2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "route_id": "cross_floor_object",
        "profile_id": PROFILE,
        "status": "success" if route_ready else "failed",
        "generated_utc": generated_at,
        "base_room_route_artifact": rel(ROOM_ROUTE),
        "base_room_route_hash": sha256(ROOM_ROUTE),
        "base_room_route_modified": False,
        "planner_implementation": (
            "tools/object_nav/lightweight_backend/occupancy_planner.py:"
            "OccupancyPlanner"
        ),
        "planner_parameters": {
            "connectivity": 8,
            "inflation_radius_m": INFLATION_RADIUS_M,
            "waypoint_spacing_m": WAYPOINT_SPACING_M,
            "maximum_exact_goal_snap_m": MAX_GOAL_SNAP_M,
        },
        "validated_route_chain": contract["validated_route_chain"],
        "same_floor_segments": room_route["same_floor_segments"]
        + [local_segment],
        "vertical_transition_segment": room_route[
            "vertical_transition_segment"
        ],
        "route_floor_waypoints": combined_floor_points,
        "selected_object_approach": {
            "object_id": OBJECT_ID,
            "candidate_id": selected["candidate_id"],
            "world_xy": selected["world_xy"],
            "yaw": selected["yaw"],
            "yaw_policy": selected["yaw_policy"],
            "visible_object_footprint_proxy_xy": selected[
                "visible_object_footprint_proxy_xy"
            ],
        },
        "route_metrics": route_metrics,
        "safety_validation": {
            **validation_checks,
            "object_executable_approach_readiness": route_ready,
            "collision_free_guarantee_claimed": False,
        },
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "source_boundary": {
            "canonical_rslg_slam_outputs_only": True,
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "simulator_navmesh_used": False,
            "manual_geometry_used": False,
        },
    }
    write_json(L3_ROUTE, object_route)

    selected_route = {
        "schema_name": "rslg_executable_route_candidate_selected",
        "schema_version": "0.2",
        "candidate_id": "cross_floor_object_conservative_canonical_v0_2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "generated_utc": generated_at,
        "status": "selected",
        "selected_profile": PROFILE,
        "selected_route_artifact": rel(L3_ROUTE),
        "base_room_route_artifact": rel(ROOM_ROUTE),
        "approach_candidate_id": selected["candidate_id"],
        "approach_world_xy": selected["world_xy"],
        "approach_yaw": selected["yaw"],
        "route_metrics": route_metrics,
        "object_executable_approach_readiness": route_ready,
        "future_layer4_authorized_demo_input_candidate": route_ready,
        "runtime_validation_performed": False,
        "object_navigation_success_claimed": False,
        "collision_free_guarantee_claimed": False,
    }
    write_json(L3_SELECTED, selected_route)

    validation = {
        "schema_name": "rslg_cross_floor_object_route_generation_validation_report",
        "schema_version": "0.2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "generated_utc": generated_at,
        "status": "passed" if route_ready else "failed",
        "selected_profile": PROFILE,
        "selected_candidate_id": selected["candidate_id"],
        "checks": validation_checks,
        "object_executable_approach_readiness": route_ready,
        "route_artifact": rel(L3_ROUTE),
        "selected_route_candidate_artifact": rel(L3_SELECTED),
        "exact_blockers": [] if route_ready else [
            key for key, value in validation_checks.items() if value is False
        ],
        "runtime_execution_required_for_static_readiness": False,
        "object_navigation_success_claimed": False,
    }
    write_json(L3_VALIDATION, validation)


def make_layer4_artifacts(
    generated_at: str, selected: dict[str, Any]
) -> None:
    route = read_json(L3_ROUTE)
    route_ready = route["safety_validation"][
        "object_executable_approach_readiness"
    ]
    readiness = {
        "schema_name": "rslg_object_runtime_readiness_report",
        "schema_version": "0.2",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": generated_at,
        "selected_profile": PROFILE,
        "query": QUERY,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "approach_candidate_id": selected["candidate_id"],
        "object_route_static_ready": route_ready,
        "object_runtime_authorization_detected": RUNTIME_AUTHORIZED,
        "object_runtime_execution_status": (
            "not_executed_static_task_only"
            if not RUNTIME_AUTHORIZED
            else "authorized_but_not_executed_by_static_packager"
        ),
        "object_navigation_success_claimed": False,
        "selected_route_artifact": rel(L3_ROUTE),
        "runtime_input_artifact": rel(L4_RUNTIME_INPUT),
        "preflight_artifact": rel(L4_PREFLIGHT),
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "collision_free_guarantee_claimed": False,
        "recommended_next_task": "task41_authorized_object_route_runtime_validation",
    }
    write_json(L4_READINESS, readiness)

    runtime_input = {
        "schema_name": "rslg_object_route_runtime_input",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": generated_at,
        "runtime_execution_performed": False,
        "authorization_environment_variable": "RSLG_TASK40_ALLOW_RUNTIME",
        "authorization_detected_during_packaging": RUNTIME_AUTHORIZED,
        "selected_profile": PROFILE,
        "base_room_route_runtime_context": {
            "route_artifact": rel(ROOM_ROUTE),
            "task39_execution_report": rel(TASK39_EXECUTION),
            "room_14_reached_in_task39": True,
        },
        "object_route_artifact": rel(L3_ROUTE),
        "floor_2_map_yaml": rel(MAP_YAML),
        "object_id": OBJECT_ID,
        "approach_candidate_id": selected["candidate_id"],
        "approach_goal": {
            "world_xy": selected["world_xy"],
            "yaw": selected["yaw"],
            "yaw_policy": selected["yaw_policy"],
            "visible_object_footprint_proxy_xy": selected[
                "visible_object_footprint_proxy_xy"
            ],
        },
        "floor_2_object_approach_waypoints": route["same_floor_segments"][-1][
            "waypoints"
        ],
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
    }
    write_json(L4_RUNTIME_INPUT, runtime_input)

    preflight = {
        "schema_name": "rslg_object_route_preflight_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": generated_at,
        "status": "passed_static_ready",
        "checks": {
            "selected_profile_is_conservative_canonical": PROFILE
            == "conservative_canonical",
            "object_route_artifact_exists": L3_ROUTE.is_file(),
            "object_route_static_validation_passed": route_ready,
            "floor_2_map_yaml_exists": MAP_YAML.is_file(),
            "task39_room_route_context_validated": read_json(TASK39_EXECUTION)[
                "runtime_validation_passed"
            ],
            "task39_room_14_reached": read_json(TASK39_EXECUTION)[
                "final_room_status"
            ]
            == "room_14_reached_within_runtime_tolerance",
            "selected_candidate_is_not_generated_ring_037": selected[
                "candidate_id"
            ]
            != "generated_ring_037",
            "object_centroid_navigation_used": False,
            "manual_target_pose_used": False,
            "runtime_started": False,
        },
        "runtime_authorization_detected": RUNTIME_AUTHORIZED,
        "runtime_execution_status": "not_executed",
        "object_route_static_ready": route_ready,
        "object_navigation_success_claimed": False,
    }
    write_json(L4_PREFLIGHT, preflight)


def make_task_evidence(
    generated_at: str,
    required_inputs: list[dict[str, Any]],
    object_record: Mapping[str, Any],
    room: Mapping[str, Any],
    records: list[dict[str, Any]],
    selected: dict[str, Any],
) -> None:
    TASK.mkdir(parents=True, exist_ok=True)
    ring037 = next(
        record for record in records if record["candidate_id"] == "generated_ring_037"
    )
    existing_inventory = {
        "schema_name": "rslg_object_approach_existing_candidate_inventory",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "object_id": OBJECT_ID,
        "inventory_scope": "canonical artifacts and historical task evidence lineage",
        "historical_candidate_list_found": False,
        "historical_candidate_list_note": (
            "The retained task28 lineage preserves generated_ring_037 only. "
            "No retained historical all-candidate task14 list was found, so no "
            "historical candidate was promoted directly."
        ),
        "candidates": [
            {
                "candidate_id": "generated_ring_037",
                "source_artifact": rel(APPROACH_V01),
                "additional_lineage_sources": [
                    "stage_outputs/stage1_generalization/00843-DYehNKdT76V/"
                    "tasks/task28_layer2_object_interface_artifact_dry_run_and_candidate_package/"
                    "reports/candidate_artifacts/object_approach_candidate_v0_1.json"
                ],
                "world_xy": ring037["world_xy"],
                "floor_id": TARGET_FLOOR,
                "room_id": TARGET_ROOM,
                "yaw": ring037["yaw"],
                "distance_to_object_m": ring037[
                    "distance_to_object_centroid_m"
                ],
                "distance_to_selected_room_route_m": ring037[
                    "distance_to_selected_room_route_terminal_m"
                ],
                "stable_map_state_under_conservative_canonical": ring037[
                    "stable_map_sample"
                ]["state"],
                "clearance_m": ring037["stable_map_sample"]["clearance_m"],
                "ray_visibility_status": ring037[
                    "object_footprint_aware_ray_check"
                ]["status"],
                "a_star_reachability_status": (
                    "not_evaluated_local_checks_failed"
                ),
                "accepted": False,
                "reason": ring037["rejected_reasons"],
            }
        ],
        "summary": {
            "existing_candidate_count": 1,
            "accepted_existing_candidate_count": 0,
            "regeneration_required": True,
        },
    }
    write_json(
        TASK / "object_approach_existing_candidate_inventory_v0_1.json",
        existing_inventory,
    )

    audit = {
        "schema_name": "rslg_task40_object_evidence_audit_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "required_input_audit": required_inputs,
        "obj_175_records_found": [
            rel(SNAPSHOT),
            rel(TOPOLOGY),
            rel(QUERY_V01),
            rel(APPROACH_V01),
            rel(PACKAGE_V01),
        ],
        "object_record": object_record,
        "object_label_room_floor_association": {
            "object_id": OBJECT_ID,
            "label": object_record["label"],
            "room_id": object_record["room_id"],
            "floor_id": object_record["floor_id"],
        },
        "object_geometry": {
            "centroid_xy": object_record["pose"],
            "centroid_xyz": object_record["pose_3d"],
            "size_xyz": object_record["size"],
            "footprint_2d": object_record["footprint_2d"],
            "bbox_available": True,
            "mask_reference_available": False,
            "extent_available": True,
        },
        "object_centroid_stable_map_state": StableMap(MAP_NPZ).sample(
            object_record["pose"]
        ),
        "object_centroid_is_wall_adjacent_or_occupied": True,
        "target_room_record": room,
        "existing_approach_candidates_found": ["generated_ring_037"],
        "generated_ring_037_status_recap": {
            "world_xy": ring037["world_xy"],
            "state": ring037["stable_map_sample"]["state"],
            "clearance_m": ring037["stable_map_sample"]["clearance_m"],
            "accepted": False,
        },
        "available_object_approach_generation_scripts_or_functions": [
            "tools/rslg_pipeline/build_layer2_formal_artifacts.py:generated_ring_candidate",
            "tools/object_nav/run_dynamic_object_query_nav.py:generate_approach",
            "tools/object_nav/run_objectnav_multiquery_validation.py:candidate generation",
            "tools/rslg_pipeline/recover_object_approach_candidates.py:generated_ring_candidates",
        ],
        "source_provenance": {
            "object_geometry": rel(SNAPSHOT),
            "room_geometry": rel(TOPOLOGY),
            "stable_map": rel(MAP_NPZ),
            "route_context": rel(ROOM_ROUTE),
            "derived_from_rgbd_and_provided_pose_pipeline": True,
        },
        "missing_evidence": [
            "No per-pixel source object mask is retained in the canonical object record.",
            "No retained all-candidate task14 artifact was found; candidates were regenerated deterministically.",
        ],
    }
    write_json(
        TASK / "task40_object_evidence_audit_report_v0_1.json", audit
    )

    generated = {
        "schema_name": "rslg_object_approach_generated_candidate_evaluation",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "selected_profile": PROFILE,
        "object_id": OBJECT_ID,
        "candidate_generation_policy": (
            "existing bbox-support ring at standoffs 0.6/0.8/1.0 m "
            "and 22-degree increments"
        ),
        "candidate_count": len(records),
        "accepted_candidate_count": sum(
            1 for record in records if record["accepted"]
        ),
        "candidate_records": records,
        "profile_dependent_thr0p25_notes": {
            "generated_ring_037_free_under_thr0p25": True,
            "thr0p25_selected": False,
            "reason": (
                "The profile opens canonical occupied/wall evidence and remains "
                "analysis context only."
            ),
        },
    }
    write_json(
        TASK / "object_approach_generated_candidate_evaluation_v0_1.json",
        generated,
    )

    rejected_summary: dict[str, int] = {}
    for record in records:
        if record["accepted"]:
            continue
        for reason in record["rejected_reasons"]:
            rejected_summary[reason] = rejected_summary.get(reason, 0) + 1
    selection = {
        "schema_name": "rslg_object_approach_selection_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "selected_profile": PROFILE,
        "selected_candidate_id": selected["candidate_id"],
        "selected_candidate": selected,
        "selection_reason": (
            "The candidate is the accepted existing-policy sample with the "
            "highest conservative-map clearance, followed by shortest safe A* "
            "local approach distance and route-terminal distance."
        ),
        "selection_policy_order": [
            "conservative_canonical free cell",
            "sufficient clearance",
            "A* reachable",
            "zero wall/occupied/unknown/outside-boundary crossings",
            "room and floor association",
            "object-facing yaw",
            "short local approach distance",
            "stable relation to selected room route",
        ],
        "rejected_candidate_summary": rejected_summary,
        "generated_ring_037_comparison": {
            "state": ring037["stable_map_sample"]["state"],
            "clearance_m": ring037["stable_map_sample"]["clearance_m"],
            "accepted": False,
            "rejected_reasons": ring037["rejected_reasons"],
        },
        "conservative_canonical_feasibility": True,
        "profile_dependent_thr0p25_notes": (
            "generated_ring_037 is profile-dependent context only; it was not selected."
        ),
        "canonical_object_approach_can_be_upgraded": True,
    }
    write_json(
        TASK / "object_approach_selection_report_v0_1.json", selection
    )
    write_json(
        TASK / "object_route_generation_validation_report_v0_1.json",
        read_json(L3_VALIDATION),
    )
    write_json(
        TASK / "object_runtime_readiness_report_v0_2.json",
        read_json(L4_READINESS),
    )

    claim_boundary = {
        "schema_name": "rslg_object_recovery_claim_boundary_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "status": "passed",
        "checks": {
            "no_object_centroid_navigation_goal_used": True,
            "no_manual_object_target_pose_used": True,
            "no_external_gt_map_floorplan_or_navmesh_used": True,
            "no_real_robot_or_physical_stair_climbing_claim": True,
            "no_collision_free_guarantee": True,
            "no_object_runtime_success_claim_without_validation": True,
        },
        "allowed_claim": (
            "Object-level executable approach static readiness was recovered for "
            "obj_175 under conservative_canonical using a revalidated alternative "
            "candidate."
        ),
        "object_runtime_executed": False,
        "object_navigation_success_claimed": False,
    }
    write_json(
        TASK / "object_recovery_claim_boundary_report_v0_1.json",
        claim_boundary,
    )

    warnings = """RSLG-SLAM task40 warnings
- obj_175 is a wall-attached curtain whose centroid is occupied and slightly outside the room_14 polygon; the centroid is not used as a navigation goal.
- No retained all-candidate task14 artifact was found. The 51 candidates were regenerated deterministically from the existing bbox-support ring policy and revalidated against current canonical artifacts.
- The selected ray policy excludes samples inside the target curtain footprint only after reaching that footprint. All preceding samples must be free; obstruction evidence is not globally ignored.
- Object geometry is an axis-aligned footprint proxy from the canonical Layer 1 record, not a dense surface reconstruction or visual runtime confirmation.
- generated_ring_037 remains blocked under conservative_canonical. Its free state under navigation_thr0p25_candidate remains profile-dependent analysis context and was not selected.
- Static A* and map-cell validation do not establish a full robot-footprint collision-free guarantee.
- RSLG_TASK40_ALLOW_RUNTIME was not set, so no ROS, Gazebo, RViz, Nav2, AMCL, or object runtime execution was started.
"""
    (TASK / "warnings.txt").write_text(warnings, encoding="utf-8")

    command_log = f"""RSLG-SLAM task40 command log
Generated UTC: {generated_at}
Working directory: {REPO}

Environment:
RSLG_TASK40_ALLOW_RUNTIME={'1' if RUNTIME_AUTHORIZED else '<unset>'}
Offline Python: /home/ws/miniconda3/envs/boxfusion/bin/python
Runtime Python: /usr/bin/python3 (not invoked)

Read-only inspection commands executed:
- cat/sed/wc on the attached task request and listed docs/manifests
- git status --short, scoped to preserve unrelated worktree changes
- find/rg over tools/rslg_pipeline, tools/object_nav, canonical artifacts, and historical task lineage
- jq/json.load inspection of canonical Layer 1/2/3/4 and task36b/task36c/task39 evidence
- offline Python probes of NPZ keys, obj_175 geometry, ring candidates, footprint-aware rays, and A* reachability

Implementation and validation commands:
- /home/ws/miniconda3/envs/boxfusion/bin/python -m unittest tests.test_task40_object_approach_recovery
- /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/recover_object_approach_candidates.py
- /home/ws/miniconda3/envs/boxfusion/bin/python -m py_compile tools/rslg_pipeline/recover_object_approach_candidates.py tests/test_task40_object_approach_recovery.py
- /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/recover_object_approach_candidates.py --validate-only

Runtime commands:
- None. RSLG_TASK40_ALLOW_RUNTIME was absent.

Generated stdout/stderr:
- The recovery utility prints a compact JSON summary to stdout.
- unittest, py_compile, and validate-only results are recorded in task40_report.json and json_validation_report_v0_1.json.
"""
    (TASK / "command_log.txt").write_text(command_log, encoding="utf-8")


def make_addendum_manifest(generated_at: str) -> None:
    old_files = [QUERY_V01, APPROACH_V01, PACKAGE_V01]
    new_files = [L2_CANDIDATES, L2_SELECTED, L2_PACKAGE, L2_VALIDATION]
    payload = {
        "manifest_id": "object_interface_recovery_addendum_manifest_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "artifact_layer": "Layer 2: Formal Artifact Layer",
        "generated_utc": generated_at,
        "purpose": "Versioned task40 object-interface recovery addendum.",
        "v0_1_artifacts_preserved": True,
        "stable_maps_modified": False,
        "vertical_connector_artifacts_modified": False,
        "room_level_route_artifacts_modified": False,
        "old_file_hashes": [file_record(path) for path in old_files],
        "new_file_hashes": [file_record(path) for path in new_files],
        "selected_candidate_id": read_json(L2_SELECTED)[
            "approach_candidate_id"
        ],
    }
    write_json(L2_ADDENDUM, payload)


def intended_files() -> list[tuple[Path, str, str, str]]:
    return [
        (
            Path("tools/rslg_pipeline/recover_object_approach_candidates.py"),
            "task40 static recovery utility",
            "canonical_tool",
            "object_approach_recovery",
        ),
        (
            Path("tests/__init__.py"),
            "RSLG-SLAM test package",
            "test",
            "object_approach_recovery",
        ),
        (
            Path("tests/test_task40_object_approach_recovery.py"),
            "focused task40 recovery tests",
            "test",
            "object_approach_recovery",
        ),
        (
            Path("docs/rslg_slam/current_project_status.md"),
            "current project status update",
            "documentation",
            "project_status",
        ),
        (
            Path("docs/rslg_slam/object_navigation_status.md"),
            "object navigation status",
            "documentation",
            "object_navigation_status",
        ),
        *[
            (path.relative_to(REPO), path.stem, layer, family)
            for path, layer, family in [
                (L2_CANDIDATES, "Layer 2: Formal Artifact Layer", "object_interface"),
                (L2_SELECTED, "Layer 2: Formal Artifact Layer", "object_interface"),
                (L2_PACKAGE, "Layer 2: Formal Artifact Layer", "object_interface"),
                (L2_VALIDATION, "Layer 2: Formal Artifact Layer", "object_interface"),
                (L2_ADDENDUM, "Layer 2: Formal Artifact Layer", "manifest"),
                (L3_CONTRACT, "Layer 3: Navigation Interface Layer", "object_route"),
                (L3_REQUEST, "Layer 3: Navigation Interface Layer", "object_route"),
                (L3_PLAN, "Layer 3: Navigation Interface Layer", "object_route"),
                (L3_ROUTE, "Layer 3: Navigation Interface Layer", "object_route"),
                (L3_SELECTED, "Layer 3: Navigation Interface Layer", "object_route"),
                (L3_VALIDATION, "Layer 3: Navigation Interface Layer", "object_route"),
                (L4_READINESS, "Layer 4: Runtime Validation Layer", "object_readiness"),
                (L4_RUNTIME_INPUT, "Layer 4: Runtime Validation Layer", "object_readiness"),
                (L4_PREFLIGHT, "Layer 4: Runtime Validation Layer", "object_readiness"),
            ]
        ],
        *[
            (
                (TASK / name).relative_to(REPO),
                name,
                "task_evidence",
                "task40_evidence",
            )
            for name in [
                "task40_report.json",
                "command_log.txt",
                "warnings.txt",
                "task40_object_evidence_audit_report_v0_1.json",
                "object_approach_existing_candidate_inventory_v0_1.json",
                "object_approach_generated_candidate_evaluation_v0_1.json",
                "object_approach_selection_report_v0_1.json",
                "object_route_generation_validation_report_v0_1.json",
                "object_runtime_readiness_report_v0_2.json",
                "object_recovery_claim_boundary_report_v0_1.json",
                "created_or_modified_files_manifest_v0_1.json",
                "json_validation_report_v0_1.json",
            ]
        ],
    ]


def make_created_files_manifest() -> None:
    manifest_path = TASK / "created_or_modified_files_manifest_v0_1.json"
    records = []
    for relative_path, role, layer, family in intended_files():
        path = REPO / relative_path
        if not path.exists():
            continue
        records.append(
            {
                "path": relative_path.as_posix(),
                "size": path.stat().st_size,
                "role": role,
                "layer": layer,
                "artifact_family": family,
                "created_or_modified": (
                    "modified"
                    if relative_path.as_posix()
                    == "docs/rslg_slam/current_project_status.md"
                    else "created"
                ),
            }
        )
    payload = {
        "schema_name": "rslg_created_or_modified_files_manifest",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "task_name": TASK_NAME,
        "files": records,
        "file_count": len(records),
    }
    write_json(manifest_path, payload)


def task_json_paths() -> list[Path]:
    paths = [
        path
        for path, _role, _layer, _family in [
            (REPO / relative, role, layer, family)
            for relative, role, layer, family in intended_files()
        ]
        if path.suffix == ".json" and path.exists()
    ]
    return sorted(set(paths))


def make_json_validation_report() -> dict[str, Any]:
    report_path = TASK / "json_validation_report_v0_1.json"
    records = []
    failures = []
    for path in task_json_paths():
        try:
            with path.open(encoding="utf-8") as handle:
                json.load(handle)
            records.append({"path": rel(path), "json_load_passed": True})
        except Exception as exc:
            failure = f"{rel(path)}: {type(exc).__name__}: {exc}"
            failures.append(failure)
            records.append(
                {
                    "path": rel(path),
                    "json_load_passed": False,
                    "error": failure,
                }
            )
    payload = {
        "schema_name": "rslg_json_validation_report",
        "schema_version": "0.1",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "validator": "Python json.load",
        "python_interpreter": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "status": "passed" if not failures else "failed",
        "validated_files": records,
        "failure_count": len(failures),
        "failures": failures,
    }
    write_json(report_path, payload)
    with report_path.open(encoding="utf-8") as handle:
        json.load(handle)
    return payload


def make_task_report(
    generated_at: str,
    records: list[dict[str, Any]],
    selected: dict[str, Any],
) -> None:
    canonical_outputs = [
        rel(path)
        for path in [
            L2_CANDIDATES,
            L2_SELECTED,
            L2_PACKAGE,
            L2_VALIDATION,
            L2_ADDENDUM,
            L3_CONTRACT,
            L3_REQUEST,
            L3_PLAN,
            L3_ROUTE,
            L3_SELECTED,
            L3_VALIDATION,
            L4_READINESS,
            L4_RUNTIME_INPUT,
            L4_PREFLIGHT,
        ]
    ]
    accepted = [record for record in records if record["accepted"]]
    report = {
        "task_name": TASK_NAME,
        "status": "completed",
        "classification": "task40_object_level_navigation_recovery_completed_object_route_static_ready",
        "project_name": PROJECT,
        "scene_id": SCENE,
        "generated_utc": generated_at,
        "selected_profile": PROFILE,
        "target_object_id": OBJECT_ID,
        "target_object_label": OBJECT_LABEL,
        "canonical_layer2_input_dir": rel(LAYER2),
        "canonical_layer3_input_dir": rel(LAYER3),
        "canonical_layer4_input_dir": rel(LAYER4),
        "task40_evidence_dir": rel(TASK),
        "docs_and_manifests_read": DOCS_READ,
        "tools_used": TOOLS_USED,
        "existing_candidate_inventory_summary": {
            "existing_candidate_count": 1,
            "accepted_existing_candidate_count": 0,
            "generated_ring_037_status": "occupied_with_zero_clearance_under_conservative_canonical",
        },
        "generated_candidate_summary": {
            "candidate_count": len(records),
            "accepted_candidate_count": len(accepted),
            "accepted_candidate_ids": [
                record["candidate_id"] for record in accepted
            ],
            "generation_policy": "existing bbox_support_plus_standoff policy",
        },
        "selected_candidate_summary": {
            "selected_candidate_id": selected["candidate_id"],
            "world_xy": selected["world_xy"],
            "yaw": selected["yaw"],
            "clearance_m": selected["stable_map_sample"]["clearance_m"],
            "a_star_path_length_m": selected["a_star_route_evaluation"][
                "exact_endpoint_path_length_m"
            ],
            "planner_snap_path_length_m": selected[
                "a_star_route_evaluation"
            ]["planner_metrics"]["path_length_m"],
            "object_footprint_aware_ray_status": selected[
                "object_footprint_aware_ray_check"
            ]["status"],
        },
        "object_route_generation_summary": {
            "static_ready": True,
            "route_artifact": rel(L3_ROUTE),
            "selected_route_candidate_artifact": rel(L3_SELECTED),
            "zero_occupied_unknown_outside_or_wall_crossings": True,
            "base_room_route_modified": False,
        },
        "object_runtime_readiness_summary": {
            "static_ready": True,
            "readiness_artifact": rel(L4_READINESS),
            "runtime_input_artifact": rel(L4_RUNTIME_INPUT),
            "preflight_artifact": rel(L4_PREFLIGHT),
        },
        "runtime_authorization_detected": RUNTIME_AUTHORIZED,
        "runtime_execution_status": "not_executed",
        "object_centroid_navigation_used": False,
        "manual_target_pose_used": False,
        "forbidden_sources_used": False,
        "canonical_outputs_generated_or_updated": canonical_outputs,
        "docs_updated": [
            "docs/rslg_slam/current_project_status.md",
            "docs/rslg_slam/object_navigation_status.md",
        ],
        "exact_blockers": [],
        "claim_boundary_status": "passed",
        "recommended_next_task": "task41_authorized_object_route_runtime_validation",
    }
    write_json(TASK / "task40_report.json", report)


def validate_only() -> dict[str, Any]:
    required, missing = validate_required_inputs()
    created = [path for path in task_json_paths()]
    failures = list(missing)
    for path in created:
        try:
            read_json(path)
        except Exception as exc:
            failures.append(f"{rel(path)}: {type(exc).__name__}: {exc}")
    checks = {
        "required_inputs_present": not missing,
        "selected_candidate_is_generated_ring_002": (
            L2_SELECTED.is_file()
            and read_json(L2_SELECTED).get("approach_candidate_id")
            == "generated_ring_002"
        ),
        "object_route_static_ready": (
            L3_ROUTE.is_file()
            and read_json(L3_ROUTE)
            .get("safety_validation", {})
            .get("object_executable_approach_readiness")
            is True
        ),
        "runtime_not_executed": (
            L4_PREFLIGHT.is_file()
            and read_json(L4_PREFLIGHT).get("runtime_execution_status")
            == "not_executed"
        ),
        "json_load_validation_passed": not failures,
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "required_inputs": required,
        "failures": failures,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate task40 inputs and generated outputs without rewriting them.",
    )
    args = parser.parse_args(argv)
    if args.validate_only:
        result = validate_only()
        print(json.dumps(result, indent=2))
        return 0 if result["status"] == "passed" else 1

    generated_at = now_iso()
    required_inputs, missing = validate_required_inputs()
    if missing:
        raise RuntimeError(
            "Task40 required inputs are missing: " + ", ".join(missing)
        )
    snapshot = read_json(SNAPSHOT)
    topology = read_json(TOPOLOGY)
    object_record = next(
        item
        for item in snapshot["objects"]
        if canonical_object_id(item["id"]) == OBJECT_ID
    )
    room = next(
        item
        for item in topology["rooms"]
        if item.get("id") == TARGET_ROOM or item.get("room_id") == TARGET_ROOM
    )
    other_room_objects = [
        item
        for item in snapshot["objects"]
        if item.get("room_id") == TARGET_ROOM
        and canonical_object_id(item["id"]) != OBJECT_ID
        and item.get("footprint_2d")
    ]
    room_route = read_json(ROOM_ROUTE)
    terminal = room_route["route_floor_waypoints"][TARGET_FLOOR][-1]
    route_terminal_xy = (float(terminal["x"]), float(terminal["y"]))
    stable_map = StableMap(MAP_NPZ)
    planner = OccupancyPlanner(
        MAP_YAML, INFLATION_RADIUS_M, WAYPOINT_SPACING_M
    )
    records = [
        evaluate_candidate(
            candidate,
            stable_map=stable_map,
            planner=planner,
            object_record=object_record,
            room=room,
            other_room_objects=other_room_objects,
            route_terminal_xy=route_terminal_xy,
        )
        for candidate in generated_ring_candidates(object_record)
    ]
    accepted = [record for record in records if record["accepted"]]
    if not accepted:
        raise RuntimeError(
            "No conservative_canonical candidate passed task40 validation."
        )
    selected = sorted(accepted, key=candidate_selection_key)[0]
    if selected["candidate_id"] != "generated_ring_002":
        raise RuntimeError(
            "Deterministic recovery invariant changed: expected generated_ring_002, "
            f"selected {selected['candidate_id']}"
        )

    make_layer2_artifacts(generated_at, records, selected)
    make_layer3_artifacts(generated_at, selected, room_route)
    make_layer4_artifacts(generated_at, selected)
    make_task_evidence(
        generated_at,
        required_inputs,
        object_record,
        room,
        records,
        selected,
    )
    make_addendum_manifest(generated_at)
    make_task_report(generated_at, records, selected)
    make_created_files_manifest()
    make_json_validation_report()
    make_created_files_manifest()
    make_json_validation_report()

    result = {
        "status": "completed",
        "classification": "task40_object_level_navigation_recovery_completed_object_route_static_ready",
        "selected_profile": PROFILE,
        "selected_candidate_id": selected["candidate_id"],
        "selected_candidate_world_xy": selected["world_xy"],
        "selected_candidate_clearance_m": selected["stable_map_sample"][
            "clearance_m"
        ],
        "accepted_candidate_count": len(accepted),
        "object_route_static_ready": True,
        "runtime_authorization_detected": RUNTIME_AUTHORIZED,
        "runtime_execution_status": "not_executed",
        "task40_evidence_dir": rel(TASK),
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
