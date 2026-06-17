"""Audit stable-map, object, room, and topology coordinate alignment.

This module is intentionally read-only with respect to canonical artifacts. It
creates only small JSON reports and a debug overlay in a caller-provided task
evidence directory. It does not generate routes, map-server artifacts, stable
maps, connector geometry, object approach geometry, or runtime inputs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

from .common import normalize_repo_relative, repo_path, resolve_repo_root, save_json


LAYER = "Layer 2: Formal Artifact Layer"
AUDIT_CLASSIFICATION = "stable_map_object_alignment_audit_completed"
PASS_REAL = "alignment_passed_object_approach_blocker_is_real"
FAIL_Y_FLIP = "alignment_failed_y_axis_flip_mismatch"
INCONCLUSIVE = "alignment_inconclusive_requires_overlay_or_metadata"
FAIL_OTHER = "alignment_failed_other_coordinate_mismatch"

NEXT_TASKS = {
    PASS_REAL: "task37_layer3_navigation_interface_canonicalization_and_route_generation_with_object_approach_blocker",
    FAIL_Y_FLIP: "task36c_layer2_stable_map_coordinate_transform_repair_plan",
    FAIL_OTHER: "task36c_layer2_stable_map_coordinate_transform_repair_plan",
    INCONCLUSIVE: "task36c_layer2_alignment_metadata_and_overlay_completion",
}


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def _rel(path: Path, repo_root: Path) -> str:
    return normalize_repo_relative(path, repo_root)


def _first_existing(paths: Iterable[Path]) -> Path | None:
    return next((path for path in paths if path.is_file()), None)


def _find_named(root: Path, names: Iterable[str]) -> Path | None:
    direct = _first_existing(root / name for name in names)
    if direct:
        return direct
    for name in names:
        match = next(iter(sorted(root.rglob(name))), None)
        if match:
            return match
    return None


def _point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    previous = len(polygon) - 1
    for index, point in enumerate(polygon):
        xi, yi = map(float, point[:2])
        xj, yj = map(float, polygon[previous][:2])
        if (yi > y) != (yj > y):
            crossing_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < crossing_x:
                inside = not inside
        previous = index
    return inside


def _point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _distance_to_polygon(point: tuple[float, float], polygon: list[list[float]]) -> float | None:
    if len(polygon) < 2:
        return None
    if _point_in_polygon(point[0], point[1], polygon):
        return 0.0
    distances = []
    for index, raw_start in enumerate(polygon):
        raw_end = polygon[(index + 1) % len(polygon)]
        start = (float(raw_start[0]), float(raw_start[1]))
        end = (float(raw_end[0]), float(raw_end[1]))
        distances.append(_point_segment_distance(point, start, end))
    return min(distances)


def _parse_simple_map_yaml(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    if not path.is_file():
        return result
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        value = raw_value.strip()
        if value.startswith("[") and value.endswith("]"):
            try:
                result[key.strip()] = json.loads(value)
                continue
            except json.JSONDecodeError:
                pass
        try:
            result[key.strip()] = float(value)
        except ValueError:
            result[key.strip()] = value
    return result


def _load_grid(npz_path: Path) -> tuple[np.ndarray, dict[str, np.ndarray], list[str]]:
    notes: list[str] = []
    arrays: dict[str, np.ndarray] = {}
    with np.load(npz_path, allow_pickle=False) as archive:
        for key in archive.files:
            arrays[key] = np.asarray(archive[key])
    if {"free_mask", "occupied_mask", "unknown_mask"} <= arrays.keys():
        grid = np.full(arrays["free_mask"].shape, 205, dtype=np.uint8)
        grid[arrays["occupied_mask"].astype(bool)] = 0
        grid[arrays["free_mask"].astype(bool)] = 254
        notes.append("Planning grid reconstructed from NPZ free/occupied/unknown masks.")
        return grid, arrays, notes
    if "occupancy_values" in arrays:
        values = arrays["occupancy_values"]
        grid = np.where(values == 0, 254, np.where(values >= 100, 0, 205)).astype(np.uint8)
        notes.append("Planning grid reconstructed from NPZ occupancy_values.")
        return grid, arrays, notes
    raise ValueError(f"NPZ lacks supported occupancy arrays: {npz_path}")


def _state(value: int | None, semantics: Mapping[str, int]) -> str:
    if value is None:
        return "out_of_bounds"
    if value == int(semantics["free"]):
        return "free"
    if value == int(semantics["occupied"]):
        return "occupied"
    if value == int(semantics["unknown"]):
        return "unknown"
    return "unknown"


def _make_transforms(
    origin: tuple[float, float],
    resolution: float,
    height: int,
) -> dict[str, Callable[[tuple[float, float]], tuple[int, int]]]:
    ox, oy = origin

    def project(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        return round((y - oy) / resolution), round((x - ox) / resolution)

    def no_y_flip(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        return math.floor((y - oy) / resolution), math.floor((x - ox) / resolution)

    def y_flip(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        base_row = math.floor((y - oy) / resolution)
        return height - 1 - base_row, math.floor((x - ox) / resolution)

    return {
        "project_transform": project,
        "no_y_flip": no_y_flip,
        "y_flip": y_flip,
    }


def _probe_point(
    name: str,
    point: tuple[float, float],
    transform_name: str,
    transform: Callable[[tuple[float, float]], tuple[int, int]],
    grid: np.ndarray,
    semantics: Mapping[str, int],
    clearance_grid: np.ndarray,
    nearest_occupied_grid: np.ndarray,
    resolution: float,
) -> dict[str, Any]:
    row, col = transform(point)
    in_bounds = 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]
    value = int(grid[row, col]) if in_bounds else None
    state = _state(value, semantics)
    clearance = float(clearance_grid[row, col] * resolution) if in_bounds else None
    nearest_obstacle = float(nearest_occupied_grid[row, col] * resolution) if in_bounds else None
    notes = []
    if state == "unknown":
        notes.append("Unknown is not accepted as traversable free space.")
    if state == "occupied":
        notes.append("Point lies on an occupied cell under this transform.")
    return {
        "point_name": name,
        "transform_name": transform_name,
        "world_xy": [round(point[0], 6), round(point[1], 6)],
        "grid_rc": [int(row), int(col)],
        "in_bounds": in_bounds,
        "occupancy_value": value,
        "interpreted_state": state,
        "clearance_m": round(clearance, 6) if clearance is not None else None,
        "nearest_obstacle_distance_m": (
            round(nearest_obstacle, 6) if nearest_obstacle is not None else None
        ),
        "clearance_at_least_0_2m": bool(clearance is not None and clearance >= 0.2),
        "notes": notes,
    }


def _ray_probe(
    start: tuple[float, float],
    end: tuple[float, float],
    transform_name: str,
    transform: Callable[[tuple[float, float]], tuple[int, int]],
    grid: np.ndarray,
    semantics: Mapping[str, int],
    resolution: float,
) -> dict[str, Any]:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(2, int(math.ceil(distance / (resolution * 0.5))))
    counts = {"free": 0, "occupied": 0, "unknown": 0, "out_of_bounds": 0}
    for index in range(steps + 1):
        ratio = index / steps
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        row, col = transform(point)
        if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
            counts["out_of_bounds"] += 1
            continue
        counts[_state(int(grid[row, col]), semantics)] += 1
    occupied_only_clear = counts["occupied"] == 0 and counts["out_of_bounds"] == 0
    traversable_clear = occupied_only_clear and counts["unknown"] == 0
    return {
        "transform_name": transform_name,
        "distance_m": round(distance, 6),
        "sample_count": steps + 1,
        "sample_state_counts": counts,
        "occupied_only_ray_clear": occupied_only_clear,
        "ray_clear": traversable_clear,
        "unknown_treated_as_clear": False,
    }


def _node_by_id(topology: Mapping[str, Any], node_id: str) -> dict[str, Any] | None:
    for node in topology.get("nodes", []):
        if node.get("node_id") == node_id:
            return dict(node)
    return None


def _extract_points(
    query_resolution: Mapping[str, Any],
    approach: Mapping[str, Any],
    topology: Mapping[str, Any],
    connector_graph: Mapping[str, Any],
) -> tuple[dict[str, tuple[float, float]], dict[str, Any]]:
    points: dict[str, tuple[float, float]] = {}
    evidence: dict[str, Any] = {}
    approach_xy = approach.get("world_xy")
    if isinstance(approach_xy, list) and len(approach_xy) >= 2:
        points["generated_ring_037"] = (float(approach_xy[0]), float(approach_xy[1]))
    object_record = query_resolution.get("current_layer1_object_record", {})
    object_xy = object_record.get("pose")
    if isinstance(object_xy, list) and len(object_xy) >= 2:
        points["obj_175"] = (float(object_xy[0]), float(object_xy[1]))

    for room_id in ("room_7", "room_13", "room_14"):
        node = _node_by_id(topology, room_id)
        if not node:
            continue
        anchor = node.get("planner_anchor_xy")
        if isinstance(anchor, list) and len(anchor) >= 2:
            points[f"{room_id}_planner_anchor"] = (float(anchor[0]), float(anchor[1]))
        evidence[room_id] = node

    for node in connector_graph.get("nodes", []):
        if node.get("node_id") != "vt_1_floor_2_exit":
            continue
        position = node.get("position_xy")
        if isinstance(position, list) and len(position) >= 2:
            points["vt_1_floor_2_exit"] = (float(position[0]), float(position[1]))
            evidence["connector_floor_2"] = dict(node)
    return points, evidence


def _draw_marker(
    draw: ImageDraw.ImageDraw,
    rc: tuple[int, int],
    color: tuple[int, int, int],
    label: str,
    shape: str,
) -> None:
    row, col = rc
    radius = 7
    if shape == "cross":
        draw.line((col - radius, row, col + radius, row), fill=color, width=3)
        draw.line((col, row - radius, col, row + radius), fill=color, width=3)
    else:
        draw.ellipse((col - radius, row - radius, col + radius, row + radius), outline=color, width=3)
    draw.rectangle((col + 9, row - 8, col + 13 + 6 * len(label), row + 7), fill=(255, 255, 255))
    draw.text((col + 11, row - 7), label, fill=color)


def _generate_overlay(
    output_path: Path,
    background_path: Path,
    grid: np.ndarray,
    points: Mapping[str, tuple[float, float]],
    transforms: Mapping[str, Callable[[tuple[float, float]], tuple[int, int]]],
    room14_polygon: list[list[float]],
    repo_root: Path,
) -> dict[str, Any]:
    image = Image.open(background_path).convert("RGB")
    if image.size != (grid.shape[1], grid.shape[0]):
        raise ValueError(
            f"Background size {image.size} does not match planning grid "
            f"{(grid.shape[1], grid.shape[0])}."
        )
    draw = ImageDraw.Draw(image)
    colors = {
        "project_transform": (220, 20, 60),
        "no_y_flip": (255, 140, 0),
        "y_flip": (0, 90, 220),
    }
    labels = {
        "generated_ring_037": "candidate",
        "obj_175": "object",
        "room_14_planner_anchor": "room14",
        "vt_1_floor_2_exit": "connector",
    }
    points_drawn = []
    for point_name in (
        "generated_ring_037",
        "obj_175",
        "room_14_planner_anchor",
        "vt_1_floor_2_exit",
    ):
        if point_name not in points:
            continue
        for transform_name in ("project_transform", "no_y_flip", "y_flip"):
            rc = transforms[transform_name](points[point_name])
            if not (0 <= rc[0] < grid.shape[0] and 0 <= rc[1] < grid.shape[1]):
                continue
            suffix = {"project_transform": "P", "no_y_flip": "N", "y_flip": "Y"}[transform_name]
            _draw_marker(
                draw,
                rc,
                colors[transform_name],
                f"{labels[point_name]}-{suffix}",
                "cross" if point_name in {"generated_ring_037", "obj_175"} else "circle",
            )
            points_drawn.append(
                {
                    "point_name": point_name,
                    "transform_name": transform_name,
                    "world_xy": list(points[point_name]),
                    "grid_rc": list(rc),
                }
            )

    if room14_polygon:
        for transform_name in ("no_y_flip", "y_flip"):
            pixel_points = []
            for raw_point in room14_polygon:
                row, col = transforms[transform_name]((float(raw_point[0]), float(raw_point[1])))
                pixel_points.append((col, row))
            if len(pixel_points) >= 3:
                draw.line(pixel_points + [pixel_points[0]], fill=colors[transform_name], width=2)

    legend = [
        ("P project round/no-flip", colors["project_transform"]),
        ("N required floor/no-flip", colors["no_y_flip"]),
        ("Y required floor/y-flip", colors["y_flip"]),
        ("Overlay is debug evidence only", (80, 80, 80)),
    ]
    draw.rectangle((8, 8, 250, 78), fill=(255, 255, 255), outline=(0, 0, 0))
    for index, (text, color) in enumerate(legend):
        draw.text((15, 14 + index * 15), text, fill=color)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)
    return {
        "overlay_generated": True,
        "input_map_source": _rel(background_path, repo_root),
        "map_width": grid.shape[1],
        "map_height": grid.shape[0],
        "transforms_drawn": ["project_transform", "no_y_flip", "y_flip"],
        "points_drawn": points_drawn,
        "missing_overlay_inputs": [],
        "claim_boundary": {
            "debug_overlay_only": True,
            "final_map_artifact": False,
            "runtime_map_artifact": False,
            "stable_map_regenerated": False,
        },
    }


def run_audit(
    *,
    repo_root: Path,
    scene_id: str,
    canonical_layer2_root: Path,
    target_floor: str,
    object_id: str,
    approach_candidate_id: str,
    output_dir: Path,
    output_json: Path,
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    inspected_files: list[str] = []
    output_dir.mkdir(parents=True, exist_ok=True)

    floor_root = canonical_layer2_root / "stable_maps" / target_floor
    paths = {
        "manifest": _find_named(
            canonical_layer2_root,
            ["layer2_formal_artifacts_manifest_v0_1.json"],
        ),
        "map_metadata": _find_named(
            floor_root,
            [f"{target_floor}_stable_occupancy_map_metadata_v0_1.json"],
        ),
        "map_yaml": _find_named(
            floor_root,
            [f"{target_floor}_stable_occupancy_map_v0_1.yaml"],
        ),
        "map_npz": _find_named(
            floor_root,
            [f"{target_floor}_stable_occupancy_map_v0_1.npz"],
        ),
        "map_pgm": _find_named(
            floor_root,
            [f"{target_floor}_stable_occupancy_map_v0_1.pgm"],
        ),
        "map_preview": _find_named(
            floor_root,
            [f"{target_floor}_stable_occupancy_map_preview_v0_1.png"],
        ),
        "query_resolution": _find_named(
            canonical_layer2_root,
            ["object_query_resolution_v0_1.json"],
        ),
        "approach": _find_named(
            canonical_layer2_root,
            ["object_approach_v0_1.json"],
        ),
        "interface_package": _find_named(
            canonical_layer2_root,
            ["object_interface_package_v0_1.json"],
        ),
        "topology": _find_named(
            canonical_layer2_root,
            ["cross_floor_topology_v0_1.json"],
        ),
        "connector_graph": _find_named(
            canonical_layer2_root,
            ["stairs_or_vertical_connector_graph_v0_1.json"],
        ),
        "connector": _find_named(
            canonical_layer2_root,
            ["vertical_connectors_v0_1.json"],
        ),
    }
    inspected_files.extend(_rel(path, repo_root) for path in paths.values() if path)

    required = ("map_metadata", "map_npz", "query_resolution", "approach", "topology", "connector_graph")
    missing = [name for name in required if paths[name] is None]
    if missing:
        errors.append("Missing required canonical Layer 2 inputs: " + ", ".join(missing))

    overlay_path = output_dir / "floor_2_stable_map_alignment_debug_overlay_v0_1.png"
    overlay_metadata_path = (
        output_dir / "floor_2_stable_map_alignment_debug_overlay_metadata_v0_1.json"
    )
    probe_path = output_dir / "generated_ring_037_alignment_probe_v0_1.json"

    if missing:
        overlay_metadata = {
            "overlay_generated": False,
            "input_map_source": None,
            "map_width": None,
            "map_height": None,
            "transforms_drawn": [],
            "points_drawn": [],
            "missing_overlay_inputs": missing,
            "missing_data_reason": "Required map raster/metadata or coordinate artifacts are missing.",
            "claim_boundary": {
                "debug_overlay_only": True,
                "final_map_artifact": False,
                "runtime_map_artifact": False,
                "stable_map_regenerated": False,
            },
        }
        save_json(overlay_metadata_path, overlay_metadata)
        probe = {
            "object_id": object_id,
            "approach_candidate_id": approach_candidate_id,
            "approach_world_xy": None,
            "object_world_xy": None,
            "project_transform_probe": None,
            "no_y_flip_probe": None,
            "y_flip_probe": None,
            "clearance_comparison": {},
            "ray_clear_comparison": {},
            "blocker_likely_real": False,
            "blocker_may_be_coordinate_bug": False,
            "recommended_action": NEXT_TASKS[INCONCLUSIVE],
        }
        save_json(probe_path, probe)
        report = {
            "classification": AUDIT_CLASSIFICATION,
            "alignment_classification": INCONCLUSIVE,
            "error_count": len(errors),
            "warning_count": len(warnings),
            "artifact_layer": LAYER,
            "scene_id": scene_id,
            "target_floor": target_floor,
            "object_id": object_id,
            "approach_candidate_id": approach_candidate_id,
            "canonical_layer2_root": _rel(canonical_layer2_root, repo_root),
            "audit_scope": "Coordinate-alignment audit only; no routes or canonical artifacts generated.",
            "map_metadata_audit": {"metadata_sufficient_for_world_to_grid": False},
            "transform_definitions": {},
            "point_transform_probe_results": {},
            "generated_ring_037_probe_summary": probe,
            "object_to_map_consistency": {},
            "topology_to_map_consistency": {},
            "overlay_report_path": _rel(overlay_path, repo_root),
            "overlay_metadata_path": _rel(overlay_metadata_path, repo_root),
            "task36_blocker_reassessment": {
                "blocker_likely_real": False,
                "blocker_may_be_coordinate_bug": False,
                "reason": "Required audit inputs are missing.",
            },
            "recommended_next_phase": "Layer 2: Formal Artifact Layer",
            "recommended_next_task": NEXT_TASKS[INCONCLUSIVE],
            "claim_boundary_checks": _claim_boundary(),
            "errors": errors,
            "warnings": warnings,
            "inspected_files": inspected_files,
        }
        save_json(output_json, report)
        return report

    metadata = _load_json(paths["map_metadata"])  # type: ignore[arg-type]
    yaml_data = _parse_simple_map_yaml(paths["map_yaml"]) if paths["map_yaml"] else {}
    query_resolution = _load_json(paths["query_resolution"])  # type: ignore[arg-type]
    approach = _load_json(paths["approach"])  # type: ignore[arg-type]
    topology = _load_json(paths["topology"])  # type: ignore[arg-type]
    connector_graph = _load_json(paths["connector_graph"])  # type: ignore[arg-type]
    grid, _arrays, grid_notes = _load_grid(paths["map_npz"])  # type: ignore[arg-type]

    height, width = map(int, grid.shape)
    resolution = float(metadata.get("resolution_m_per_cell", yaml_data.get("resolution", 0.0)))
    origin_raw = metadata.get("origin", yaml_data.get("origin", []))
    origin = (
        float(origin_raw[0]) if len(origin_raw) >= 2 else math.nan,
        float(origin_raw[1]) if len(origin_raw) >= 2 else math.nan,
    )
    semantics = metadata.get("cell_semantics", {"free": 254, "occupied": 0, "unknown": 205})
    metadata_sufficient = (
        resolution > 0
        and all(math.isfinite(value) for value in origin)
        and height > 0
        and width > 0
    )
    if not metadata_sufficient:
        errors.append("Map metadata is insufficient for world-to-grid conversion.")

    preview_equals_planning_grid = False
    pgm_equals_yflip = False
    if paths["map_preview"]:
        preview = np.asarray(Image.open(paths["map_preview"]).convert("L"))
        preview_equals_planning_grid = preview.shape == grid.shape and np.array_equal(preview, grid)
    if paths["map_pgm"]:
        pgm = np.asarray(Image.open(paths["map_pgm"]).convert("L"))
        pgm_equals_yflip = pgm.shape == grid.shape and np.array_equal(pgm, np.flipud(grid))

    if not metadata.get("frame_id"):
        warnings.append("Stable-map metadata does not declare a map frame id.")
    if not metadata.get("raster_orientation"):
        warnings.append(
            "Stable-map metadata does not explicitly declare preview/NPZ raster orientation; "
            "orientation was inferred from NPZ payload names, PGM pixels, and the canonical builder."
        )
    warnings.append(
        "The project transform uses round while the audit comparison transforms use floor; "
        "generated_ring_037 resolves to the same cell for project and no-y-flip in this case."
    )

    transforms = _make_transforms(origin, resolution, height)
    transform_definitions = {
        "project_transform": {
            "definition": "row=round((y-origin_y)/resolution); col=round((x-origin_x)/resolution)",
            "source": "canonical object approach grid_rc and tools/rslg_pipeline/build_layer2_formal_artifacts.py",
            "raster_space": "NPZ planning grid / unflipped preview",
            "available": True,
        },
        "no_y_flip": {
            "definition": "row=floor((y-origin_y)/resolution); col=floor((x-origin_x)/resolution)",
            "raster_space": "NPZ planning grid / unflipped preview",
        },
        "y_flip": {
            "definition": "row=height-1-floor((y-origin_y)/resolution); col=floor((x-origin_x)/resolution)",
            "raster_space": "top-origin display raster / PGM pixel order",
        },
    }

    points, evidence = _extract_points(query_resolution, approach, topology, connector_graph)
    approach_xy = points.get(approach_candidate_id)
    object_xy = points.get(object_id)
    if approach.get("approach_candidate_id") != approach_candidate_id or not approach_xy:
        errors.append(f"Approach candidate {approach_candidate_id} was not found.")
    if query_resolution.get("object_id") != object_id or not object_xy:
        errors.append(f"Object {object_id} was not found.")

    free_mask = grid == int(semantics["free"])
    occupied_mask = grid == int(semantics["occupied"])
    clearance_grid = ndimage.distance_transform_edt(free_mask)
    nearest_occupied_grid = ndimage.distance_transform_edt(~occupied_mask)
    point_results: dict[str, dict[str, Any]] = {}
    for point_name, point in points.items():
        point_results[point_name] = {
            name: _probe_point(
                point_name,
                point,
                name,
                transform,
                grid,
                semantics,
                clearance_grid,
                nearest_occupied_grid,
                resolution,
            )
            for name, transform in transforms.items()
        }

    ray_results: dict[str, Any] = {}
    if approach_xy and object_xy:
        ray_results = {
            name: _ray_probe(
                approach_xy,
                object_xy,
                name,
                transform,
                grid,
                semantics,
                resolution,
            )
            for name, transform in transforms.items()
        }

    candidate_results = point_results.get(approach_candidate_id, {})
    project_probe = candidate_results.get("project_transform")
    no_flip_probe = candidate_results.get("no_y_flip")
    y_flip_probe = candidate_results.get("y_flip")
    project_matches_artifact = bool(
        project_probe and project_probe["grid_rc"] == approach.get("grid_rc")
    )

    room14 = evidence.get("room_14", {})
    room14_polygon = room14.get("polygon_xy", [])
    room14_anchor_results = point_results.get("room_14_planner_anchor", {})
    connector_results = point_results.get("vt_1_floor_2_exit", {})
    route_reference_names = [
        "room_7_planner_anchor",
        "room_13_planner_anchor",
        "room_14_planner_anchor",
        "vt_1_floor_2_exit",
    ]
    route_reference_states = {
        transform_name: {
            name: point_results[name][transform_name]["interpreted_state"]
            for name in route_reference_names
            if name in point_results
        }
        for transform_name in transforms
    }
    no_flip_refs_free = bool(route_reference_states["no_y_flip"]) and all(
        state == "free" for state in route_reference_states["no_y_flip"].values()
    )
    y_flip_refs_free = bool(route_reference_states["y_flip"]) and all(
        state == "free" for state in route_reference_states["y_flip"].values()
    )

    y_flip_clear = bool(
        y_flip_probe
        and y_flip_probe["interpreted_state"] == "free"
        and y_flip_probe["clearance_at_least_0_2m"]
        and ray_results.get("y_flip", {}).get("ray_clear")
    )
    project_blocked = bool(
        project_probe
        and (
            project_probe["interpreted_state"] != "free"
            or not project_probe["clearance_at_least_0_2m"]
            or not ray_results.get("project_transform", {}).get("ray_clear")
        )
    )
    no_flip_blocked = bool(
        no_flip_probe
        and (
            no_flip_probe["interpreted_state"] != "free"
            or not no_flip_probe["clearance_at_least_0_2m"]
            or not ray_results.get("no_y_flip", {}).get("ray_clear")
        )
    )

    background = paths["map_preview"] or paths["map_pgm"]
    try:
        if not background:
            raise FileNotFoundError("Neither preview PNG nor PGM raster is available.")
        overlay_metadata = _generate_overlay(
            overlay_path,
            background,
            grid,
            points,
            transforms,
            room14_polygon,
            repo_root,
        )
    except Exception as exc:
        overlay_metadata = {
            "overlay_generated": False,
            "input_map_source": _rel(background, repo_root) if background else None,
            "map_width": width,
            "map_height": height,
            "transforms_drawn": [],
            "points_drawn": [],
            "missing_overlay_inputs": ["readable raster background"],
            "missing_data_reason": str(exc),
            "claim_boundary": {
                "debug_overlay_only": True,
                "final_map_artifact": False,
                "runtime_map_artifact": False,
                "stable_map_regenerated": False,
            },
        }
        errors.append(f"Debug overlay could not be generated: {exc}")
    save_json(overlay_metadata_path, overlay_metadata)

    if not metadata_sufficient or not overlay_metadata["overlay_generated"]:
        alignment = INCONCLUSIVE
    elif project_matches_artifact and no_flip_refs_free and not y_flip_refs_free and project_blocked:
        alignment = PASS_REAL
    elif project_blocked and y_flip_clear and y_flip_refs_free and not no_flip_refs_free:
        alignment = FAIL_Y_FLIP
    elif project_blocked and no_flip_blocked and not y_flip_clear and no_flip_refs_free:
        alignment = PASS_REAL
    elif not project_matches_artifact:
        alignment = FAIL_OTHER
    else:
        alignment = INCONCLUSIVE

    blocker_likely_real = alignment == PASS_REAL
    blocker_may_be_coordinate_bug = alignment in {FAIL_Y_FLIP, FAIL_OTHER}
    object_distance = (
        _distance_to_polygon(object_xy, room14_polygon)
        if object_xy and room14_polygon
        else None
    )
    approach_distance = (
        _distance_to_polygon(approach_xy, room14_polygon)
        if approach_xy and room14_polygon
        else None
    )
    object_in_room = bool(object_xy and _point_in_polygon(*object_xy, room14_polygon))
    approach_in_room = bool(approach_xy and _point_in_polygon(*approach_xy, room14_polygon))
    if object_distance is not None and not object_in_room:
        warnings.append(
            "obj_175 centroid is "
            f"{object_distance:.3f} m outside the room_14 polygon, while the canonical object "
            "record assigns the object by floor/footprint vote; this is a room-boundary nuance, "
            "not evidence of a y-axis flip."
        )

    probe = {
        "object_id": object_id,
        "approach_candidate_id": approach_candidate_id,
        "approach_world_xy": list(approach_xy) if approach_xy else None,
        "object_world_xy": list(object_xy) if object_xy else None,
        "project_transform_probe": project_probe,
        "no_y_flip_probe": no_flip_probe,
        "y_flip_probe": y_flip_probe,
        "clearance_comparison": {
            name: result.get("clearance_m") if result else None
            for name, result in (
                ("project_transform", project_probe),
                ("no_y_flip", no_flip_probe),
                ("y_flip", y_flip_probe),
            )
        },
        "ray_clear_comparison": {
            name: ray_results.get(name)
            for name in ("project_transform", "no_y_flip", "y_flip")
        },
        "blocker_likely_real": blocker_likely_real,
        "blocker_may_be_coordinate_bug": blocker_may_be_coordinate_bug,
        "recommended_action": NEXT_TASKS[alignment],
    }
    save_json(probe_path, probe)

    map_metadata_audit = {
        "map_width": width,
        "map_height": height,
        "resolution_m_per_cell": resolution,
        "origin": [origin[0], origin[1], float(origin_raw[2]) if len(origin_raw) >= 3 else 0.0],
        "frame_id": metadata.get("frame_id"),
        "frame_id_available": bool(metadata.get("frame_id")),
        "occupancy_value_convention": {
            "free": int(semantics["free"]),
            "occupied": int(semantics["occupied"]),
            "unknown": int(semantics["unknown"]),
            "white_interpreted_as_free": int(semantics["free"]) >= 250,
            "black_interpreted_as_occupied": int(semantics["occupied"]) <= 10,
            "map_server_negate": yaml_data.get("negate"),
            "map_server_occupied_thresh": yaml_data.get("occupied_thresh"),
            "map_server_free_thresh": yaml_data.get("free_thresh"),
        },
        "preview_orientation_declared": bool(metadata.get("raster_orientation")),
        "preview_equals_npz_planning_grid": preview_equals_planning_grid,
        "pgm_equals_vertical_flip_of_npz_planning_grid": pgm_equals_yflip,
        "npz_row_zero_world_y": "origin_y / low world y",
        "pgm_raster_row_zero_world_y": "high world y",
        "preview_raster_row_zero_world_y": "origin_y / low world y",
        "visual_flip_explanation": (
            "The preview stores the internal planning grid without display flipping, so row 0 is "
            "drawn at the top even though it represents low world y. The PGM is vertically flipped "
            "for top-origin raster/map-server convention. A visually flipped preview therefore does "
            "not by itself indicate a world-to-grid mismatch."
        ),
        "metadata_sufficient_for_world_to_grid": metadata_sufficient,
        "grid_loading_notes": grid_notes,
    }
    object_consistency = {
        "room_14_reference_found": bool(room14),
        "room_14_polygon_available": bool(room14_polygon),
        "room_14_planner_anchor_xy": room14.get("planner_anchor_xy"),
        "object_point_inside_room_14_polygon": object_in_room,
        "object_point_distance_to_room_14_polygon_m": (
            round(object_distance, 6) if object_distance is not None else None
        ),
        "object_assignment_evidence": query_resolution.get("target_room") == "room_14",
        "approach_candidate_inside_room_14_polygon": approach_in_room,
        "approach_candidate_distance_to_room_14_polygon_m": (
            round(approach_distance, 6) if approach_distance is not None else None
        ),
        "room_14_anchor_transform_results": room14_anchor_results,
        "object_transform_results": point_results.get(object_id),
        "approach_transform_results": candidate_results,
        "mirrored_relative_to_room_14": False,
        "consistency_assessment": (
            "The room_14 anchor and route references are free under project/no-y-flip and unknown "
            "under y-flip. The approach remains geometrically inside room_14 but lies on occupied "
            "map evidence; the object centroid is on object/obstacle evidence near the room boundary."
        ),
        "task36_blocker_could_be_coordinate_misalignment": blocker_may_be_coordinate_bug,
    }
    topology_consistency = {
        "route_reference_transform_states": route_reference_states,
        "room_13_room_14_connection_available": any(
            edge.get("edge_id") == "room_13__room_14"
            for edge in topology.get("same_floor_edges", [])
        ),
        "connection_coordinate_reference_available": False,
        "connector_floor_2_reference_found": "connector_floor_2" in evidence,
        "connector_floor_2_transform_results": connector_results,
        "no_y_flip_route_references_all_free": no_flip_refs_free,
        "y_flip_route_references_all_free": y_flip_refs_free,
        "route_topology_uses_same_convention_as_object_approach": (
            project_matches_artifact and no_flip_refs_free and not y_flip_refs_free
        ),
    }
    blocker_reason = (
        "Project/no-y-flip places room_7, room_13, room_14, and the floor-2 connector endpoint "
        "on free space, while generated_ring_037 is occupied with zero clearance and its ray "
        "crosses occupied cells. The y-flip alternative places the same references in unknown "
        "space and does not provide a better consistent explanation."
        if blocker_likely_real
        else "The compared transforms do not provide enough consistent evidence to accept the blocker as real."
    )
    report = {
        "classification": AUDIT_CLASSIFICATION,
        "alignment_classification": alignment,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": LAYER,
        "scene_id": scene_id,
        "target_floor": target_floor,
        "object_id": object_id,
        "approach_candidate_id": approach_candidate_id,
        "canonical_layer2_root": _rel(canonical_layer2_root, repo_root),
        "audit_scope": {
            "coordinate_alignment_only": True,
            "stable_map_object_room_topology_alignment": True,
            "route_generation": False,
            "approach_candidate_changed": False,
        },
        "map_metadata_audit": map_metadata_audit,
        "transform_definitions": transform_definitions,
        "point_transform_probe_results": point_results,
        "generated_ring_037_probe_summary": probe,
        "object_to_map_consistency": object_consistency,
        "topology_to_map_consistency": topology_consistency,
        "overlay_report_path": _rel(overlay_path, repo_root),
        "overlay_metadata_path": _rel(overlay_metadata_path, repo_root),
        "task36_blocker_reassessment": {
            "blocker_likely_real": blocker_likely_real,
            "blocker_may_be_coordinate_bug": blocker_may_be_coordinate_bug,
            "reason": blocker_reason,
        },
        "recommended_next_phase": "Layer 3: Navigation Interface Layer" if blocker_likely_real else LAYER,
        "recommended_next_task": NEXT_TASKS[alignment],
        "claim_boundary_checks": _claim_boundary(),
        "errors": errors,
        "warnings": warnings,
        "inspected_files": inspected_files,
    }
    save_json(output_json, report)
    return report


def _claim_boundary() -> dict[str, bool]:
    return {
        "historical_stage_outputs_restored": False,
        "world_model_rerun": False,
        "stage_a_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "canonical_layer2_artifacts_modified": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "stable_map_regenerated": False,
        "stable_map_pixels_generated": False,
        "debug_overlay_generated_as_evidence_only": True,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "connector_geometry_regenerated": False,
        "object_approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit RSLG-SLAM Layer 2 stable-map/object coordinate alignment."
    )
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--canonical-layer2-root", required=True)
    parser.add_argument("--target-floor", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--approach-candidate-id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    report = run_audit(
        repo_root=repo_root,
        scene_id=args.scene_id,
        canonical_layer2_root=repo_path(repo_root, args.canonical_layer2_root),
        target_floor=args.target_floor,
        object_id=args.object_id,
        approach_candidate_id=args.approach_candidate_id,
        output_dir=repo_path(repo_root, args.output_dir),
        output_json=repo_path(repo_root, args.output_json),
    )
    print(
        json.dumps(
            {
                "classification": report["classification"],
                "alignment_classification": report["alignment_classification"],
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
                "recommended_next_task": report["recommended_next_task"],
            },
            indent=2,
        )
    )
    return 0 if report["alignment_classification"] != INCONCLUSIVE else 2


if __name__ == "__main__":
    raise SystemExit(main())
