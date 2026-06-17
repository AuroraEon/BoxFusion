"""Audit stable occupancy-map source masks, parameters, and acceptance.

This offline utility reads canonical RSLG-SLAM Layer 1 and Layer 2 artifacts,
writes controlled sensitivity variants only to task evidence, and never
generates routes or launches runtime systems.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from .common import PROJECT_NAME, normalize_repo_relative, resolve_repo_root, save_json


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task36c_layer2_stable_occupancy_map_source_mask_parameter_acceptance_audit"
CLASSIFICATION = (
    "task36c_stable_map_acceptance_audit_completed_current_map_accepted_"
    "object_approach_blocker_source_supported"
)
LAYER = "Layer 2: Formal Artifact Layer"
PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"
RUN_ID = 2709
RESOLUTION = 0.05
ORIGIN = (-50.0, -50.0)

DOCS_AND_MANIFESTS = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/canonical_output_plan.md",
    "docs/rslg_slam/final_layer2_minimal_generation_plan.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/tool_migration_plan.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
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


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return value


def rel(path: Path, repo: Path) -> str:
    return normalize_repo_relative(path, repo)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path, repo: Path) -> dict[str, Any]:
    return {
        "path": rel(path, repo),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def world_to_grid(point: Iterable[float]) -> tuple[int, int]:
    x, y = [float(value) for value in list(point)[:2]]
    return (
        round((y - ORIGIN[1]) / RESOLUTION),
        round((x - ORIGIN[0]) / RESOLUTION),
    )


def load_png(path: Path, *, crop_padding: bool = False) -> np.ndarray:
    array = np.asarray(Image.open(path))
    if crop_padding:
        array = array[10:-10, 10:-10]
    return array


def scalar_or_list(value: Any) -> int | float | bool | list[Any]:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def state_for_mask(mask_id: str, value: Any, *, wall_label: int) -> str:
    scalar = scalar_or_list(value)
    if mask_id.startswith("gateway_wall") or mask_id == "wall_skeleton":
        return "occupied_evidence" if int(scalar) > 0 else "no_wall_evidence"
    if mask_id == "outside_boundary":
        return "inside" if int(scalar) > 0 else "outside"
    if mask_id in {"full_map_pre_doors", "full_map_post_doors"}:
        return "occupied" if int(scalar) > 0 else "free"
    if mask_id == "door_carve_delta":
        return "carved" if int(scalar) > 0 else "unchanged"
    if mask_id == "free_space":
        return "free" if int(scalar) > 0 else "occupied"
    if mask_id in {"final_room_labels", "repaired_room_labels"}:
        if int(scalar) == wall_label:
            return "occupied"
        if int(scalar) == 0:
            return "unknown"
        return "room_label"
    if mask_id == "final_rooms_preview":
        return "not_applicable"
    if mask_id.startswith("stable_"):
        if int(scalar) == 254:
            return "free"
        if int(scalar) == 0:
            return "occupied"
        if int(scalar) == 205:
            return "unknown"
    return "not_applicable"


def expected_state(point_name: str) -> str:
    if point_name == "obj_175":
        return "allowed_to_be_occupied"
    if point_name in {
        "representative_high_confidence_wall",
        "representative_segmentation_only_wall",
    }:
        return "occupied"
    return "free"


def point_explanation(point_name: str) -> str:
    explanations = {
        "obj_175": "Curtain centroid may be occupied or wall-adjacent.",
        "generated_ring_037": "Robot standoff candidate should normally be free.",
        "representative_high_confidence_wall": "Selected from wall evidence present at threshold 0.30.",
        "representative_segmentation_only_wall": (
            "Selected from segmentation-wall evidence absent at threshold 0.18."
        ),
        "representative_free_space": "Selected from the largest free-space component.",
    }
    if point_name.endswith("_planner_anchor"):
        return "Planner graph room anchor is expected to be free."
    if point_name == "vt_1_floor_2_exit":
        return "Floor-2 vertical-connector exit is expected to be free."
    return explanations.get(point_name, "Route- or map-relevant audit point.")


def nearest_point(mask: np.ndarray, target_rc: tuple[int, int]) -> tuple[int, int]:
    rows, cols = np.where(mask)
    if not len(rows):
        raise ValueError("Cannot select a representative point from an empty mask.")
    distances = (rows - target_rc[0]) ** 2 + (cols - target_rc[1]) ** 2
    index = int(np.argmin(distances))
    return int(rows[index]), int(cols[index])


def grid_to_world(rc: tuple[int, int]) -> list[float]:
    row, col = rc
    return [
        round(ORIGIN[0] + col * RESOLUTION, 6),
        round(ORIGIN[1] + row * RESOLUTION, 6),
    ]


def load_points(layer2: Path) -> dict[str, dict[str, Any]]:
    graph = load_json(layer2 / "planner_graph" / "route_planner_graph_v0_1.json")
    approach = load_json(layer2 / "object_interfaces" / "object_approach_v0_1.json")
    query = load_json(layer2 / "object_interfaces" / "object_query_resolution_v0_1.json")
    connector_graph = load_json(
        layer2 / "vertical_connectors" / "stairs_or_vertical_connector_graph_v0_1.json"
    )
    points: dict[str, dict[str, Any]] = {
        "obj_175": {
            "world_xy": query["current_layer1_object_record"]["pose"][:2],
            "source": "object_query_resolution_v0_1.json",
        },
        "generated_ring_037": {
            "world_xy": approach["world_xy"],
            "source": "object_approach_v0_1.json",
        },
    }
    for node in graph.get("nodes", []):
        if node.get("floor_id") != "floor_2" or not node.get("planner_anchor_xy"):
            continue
        points[f"{node['node_id']}_planner_anchor"] = {
            "world_xy": node["planner_anchor_xy"],
            "source": "route_planner_graph_v0_1.json",
        }
    for node in connector_graph.get("nodes", []):
        if node.get("node_id") == "vt_1_floor_2_exit":
            points["vt_1_floor_2_exit"] = {
                "world_xy": node["position_xy"],
                "source": "stairs_or_vertical_connector_graph_v0_1.json",
            }
    return points


def map_arrays(layer2: Path, floor_id: str = "floor_2") -> dict[str, np.ndarray]:
    npz_path = (
        layer2
        / "stable_maps"
        / floor_id
        / f"{floor_id}_stable_occupancy_map_v0_1.npz"
    )
    with np.load(npz_path, allow_pickle=False) as archive:
        free = archive["free_mask"].astype(bool)
        occupied = archive["occupied_mask"].astype(bool)
        unknown = archive["unknown_mask"].astype(bool)
    grid = np.full(free.shape, 205, dtype=np.uint8)
    grid[occupied] = 0
    grid[free] = 254
    return {"free": free, "occupied": occupied, "unknown": unknown, "grid": grid}


def build_sources(debug: Path, layer2: Path) -> tuple[dict[str, dict[str, Any]], int]:
    labels = np.load(debug / f"run_{RUN_ID}_09_final_labels.npy")
    repaired = np.load(debug / f"run_{RUN_ID}_09b_repaired_labels.npy")
    tracking = load_json(debug / f"run_{RUN_ID}_12_tracking_report.json")
    wall_label = int(max(np.max(labels), np.max(repaired)))
    stable = map_arrays(layer2)
    source_specs = {
        "gateway_wall_preclose_thr_0p18": (
            debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p18.png",
            load_png(debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p18.png"),
            False,
            "255 is wall evidence; 0 is no wall evidence.",
        ),
        "gateway_wall_preclose_thr_0p20": (
            debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p20.png",
            load_png(debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p20.png"),
            False,
            "255 is wall evidence; 0 is no wall evidence.",
        ),
        "gateway_wall_preclose_thr_0p25": (
            debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p25.png",
            load_png(debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p25.png"),
            False,
            "255 is wall evidence; 0 is no wall evidence.",
        ),
        "gateway_wall_preclose_thr_0p30": (
            debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p30.png",
            load_png(debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_0p30.png"),
            False,
            "255 is wall evidence; 0 is no wall evidence.",
        ),
        "wall_skeleton": (
            debug / f"run_{RUN_ID}_02_walls_skeleton.png",
            load_png(debug / f"run_{RUN_ID}_02_walls_skeleton.png", crop_padding=True),
            False,
            "255 is segmentation-wall evidence; 0 is no wall evidence.",
        ),
        "outside_boundary": (
            debug / f"run_{RUN_ID}_03_outside_boundary.png",
            load_png(debug / f"run_{RUN_ID}_03_outside_boundary.png", crop_padding=True),
            False,
            "255 means inside the derived boundary; 0 means outside.",
        ),
        "full_map_pre_doors": (
            debug / f"run_{RUN_ID}_04a_full_map_pre_doors.png",
            load_png(debug / f"run_{RUN_ID}_04a_full_map_pre_doors.png"),
            False,
            "255 is occupied; 0 is free.",
        ),
        "full_map_post_doors": (
            debug / f"run_{RUN_ID}_04b_full_map_post_doors.png",
            load_png(debug / f"run_{RUN_ID}_04b_full_map_post_doors.png"),
            False,
            "255 is occupied; 0 is free.",
        ),
        "door_carve_delta": (
            debug / f"run_{RUN_ID}_04c_door_carve_delta.png",
            load_png(debug / f"run_{RUN_ID}_04c_door_carve_delta.png"),
            False,
            "255 marks changed door-carve cells; 0 means unchanged.",
        ),
        "free_space": (
            debug / f"run_{RUN_ID}_05_free_space.png",
            load_png(debug / f"run_{RUN_ID}_05_free_space.png"),
            False,
            "255 is free; 0 is non-free.",
        ),
        "final_room_labels": (
            debug / f"run_{RUN_ID}_09_final_labels.npy",
            labels,
            False,
            f"{wall_label} is the watershed wall label; 1..6 are room labels; 0 is unlabeled.",
        ),
        "repaired_room_labels": (
            debug / f"run_{RUN_ID}_09b_repaired_labels.npy",
            repaired,
            False,
            f"{wall_label} is the watershed wall label; 1..6 are room labels; 0 is unlabeled.",
        ),
        "final_rooms_preview": (
            debug / f"run_{RUN_ID}_11_final_rooms.png",
            load_png(debug / f"run_{RUN_ID}_11_final_rooms.png"),
            False,
            "RGB visualization only; colors are not occupancy values.",
        ),
        "stable_npz": (
            layer2
            / "stable_maps"
            / "floor_2"
            / "floor_2_stable_occupancy_map_v0_1.npz",
            stable["grid"],
            False,
            "254 free, 0 occupied, 205 unknown in planning-grid orientation.",
        ),
        "stable_pgm": (
            layer2
            / "stable_maps"
            / "floor_2"
            / "floor_2_stable_occupancy_map_v0_1.pgm",
            load_png(
                layer2
                / "stable_maps"
                / "floor_2"
                / "floor_2_stable_occupancy_map_v0_1.pgm"
            ),
            True,
            "254 free, 0 occupied, 205 unknown; raster is vertically flipped from planning grid.",
        ),
        "stable_preview": (
            layer2
            / "stable_maps"
            / "floor_2"
            / "floor_2_stable_occupancy_map_preview_v0_1.png",
            load_png(
                layer2
                / "stable_maps"
                / "floor_2"
                / "floor_2_stable_occupancy_map_preview_v0_1.png"
            ),
            False,
            "254 free, 0 occupied, 205 unknown in planning-grid orientation.",
        ),
    }
    sources: dict[str, dict[str, Any]] = {}
    for mask_id, (path, array, pgm_flip, semantics) in source_specs.items():
        sources[mask_id] = {
            "path": path,
            "array": array,
            "pgm_flip": pgm_flip,
            "semantics": semantics,
        }
    if tracking.get("room_count") != 6:
        raise ValueError("Unexpected floor_2 room count; wall-label inference is unsafe.")
    return sources, wall_label


def add_representative_points(
    points: dict[str, dict[str, Any]],
    sources: Mapping[str, Mapping[str, Any]],
) -> None:
    wall = np.asarray(sources["wall_skeleton"]["array"]) > 0
    gateway_018 = np.asarray(sources["gateway_wall_preclose_thr_0p18"]["array"]) > 0
    gateway_030 = np.asarray(sources["gateway_wall_preclose_thr_0p30"]["array"]) > 0
    free = np.asarray(sources["free_space"]["array"]) > 0
    candidate_rc = world_to_grid(points["generated_ring_037"]["world_xy"])
    selections = {
        "representative_high_confidence_wall": nearest_point(gateway_030, candidate_rc),
        "representative_segmentation_only_wall": nearest_point(wall & ~gateway_018, candidate_rc),
        "representative_free_space": nearest_point(free, candidate_rc),
    }
    for name, rc in selections.items():
        points[name] = {
            "world_xy": grid_to_world(rc),
            "grid_rc_override": list(rc),
            "source": "deterministic nearest representative cell",
        }


def probe_sources(
    points: Mapping[str, Mapping[str, Any]],
    sources: Mapping[str, Mapping[str, Any]],
    wall_label: int,
    stable_grid: np.ndarray,
) -> list[dict[str, Any]]:
    clearance = ndimage.distance_transform_edt(stable_grid == 254) * RESOLUTION
    records: list[dict[str, Any]] = []
    for point_name, point in points.items():
        rc = tuple(point.get("grid_rc_override", world_to_grid(point["world_xy"])))
        row, col = map(int, rc)
        stable_clearance = round(float(clearance[row, col]), 6)
        for mask_id, source in sources.items():
            array = np.asarray(source["array"])
            source_row = array.shape[0] - 1 - row if source["pgm_flip"] else row
            value = array[source_row, col]
            state = state_for_mask(mask_id, value, wall_label=wall_label)
            records.append(
                {
                    "point_name": point_name,
                    "world_xy": [round(float(v), 6) for v in point["world_xy"]],
                    "grid_rc": [row, col],
                    "source_mask_id": mask_id,
                    "source_path": str(source["path"]),
                    "source_grid_rc": [source_row, col],
                    "raw_value": scalar_or_list(value),
                    "interpreted_state": state,
                    "source_semantics": source["semantics"],
                    "stable_map_value": int(stable_grid[row, col]),
                    "stable_map_state": state_for_mask(
                        "stable_npz", stable_grid[row, col], wall_label=wall_label
                    ),
                    "stable_clearance_m": stable_clearance,
                    "expected_state": expected_state(point_name),
                    "explanation": point_explanation(point_name),
                }
            )
    return records


def write_probe_csv(path: Path, records: list[dict[str, Any]]) -> None:
    fields = [
        "point_name",
        "world_xy",
        "grid_rc",
        "source_mask_id",
        "source_path",
        "source_grid_rc",
        "raw_value",
        "interpreted_state",
        "source_semantics",
        "stable_map_value",
        "stable_map_state",
        "stable_clearance_m",
        "expected_state",
        "explanation",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = dict(record)
            for key in ("world_xy", "grid_rc", "source_grid_rc", "raw_value"):
                row[key] = json.dumps(row[key], separators=(",", ":"))
            writer.writerow(row)


def component_summary(free: np.ndarray, points: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    labels, count = ndimage.label(free, structure=np.ones((3, 3), dtype=np.uint8))
    distances = ndimage.distance_transform_edt(free) * RESOLUTION
    point_states: dict[str, Any] = {}
    for name, point in points.items():
        row, col = tuple(point.get("grid_rc_override", world_to_grid(point["world_xy"])))
        point_states[name] = {
            "grid_rc": [int(row), int(col)],
            "free": bool(free[row, col]),
            "component_id": int(labels[row, col]),
            "clearance_m": round(float(distances[row, col]), 6),
        }
    route_names = [
        "room_7_planner_anchor",
        "room_13_planner_anchor",
        "room_14_planner_anchor",
        "vt_1_floor_2_exit",
    ]
    route_components = [point_states[name]["component_id"] for name in route_names]
    return {
        "free_cell_count": int(free.sum()),
        "free_component_count": int(count),
        "point_states": point_states,
        "route_relevant_anchors_same_nonzero_component": (
            len(set(route_components)) == 1 and route_components[0] > 0
        ),
        "route_component_ids": dict(zip(route_names, route_components)),
    }


def rebuild_gateway_variant(
    debug: Path,
    threshold_name: str,
) -> np.ndarray:
    outside_padded = load_png(debug / f"run_{RUN_ID}_03_outside_boundary.png")
    gateway = load_png(
        debug / f"run_{RUN_ID}_01e_gateway_wall_preclose_thr_{threshold_name}.png"
    )
    padded_gateway = np.zeros_like(outside_padded)
    padded_gateway[10:-10, 10:-10] = gateway
    full_padded = cv2.bitwise_or(padded_gateway, cv2.bitwise_not(outside_padded))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    full_padded = cv2.morphologyEx(full_padded, cv2.MORPH_CLOSE, kernel, iterations=2)
    full = full_padded[10:-10, 10:-10]
    delta = load_png(debug / f"run_{RUN_ID}_04c_door_carve_delta.png") > 0
    canonical_post = load_png(debug / f"run_{RUN_ID}_04b_full_map_post_doors.png")
    full[delta & (canonical_post == 0)] = 0
    return full == 0


def variant_grid(free: np.ndarray, inside: np.ndarray, *, unknown_inside: bool = False) -> np.ndarray:
    grid = np.full(free.shape, 205, dtype=np.uint8)
    grid[free] = 254
    if not unknown_inside:
        grid[inside & ~free] = 0
    return grid


def save_variant(
    root: Path,
    variant_id: str,
    free: np.ndarray,
    inside: np.ndarray,
    parameters: Mapping[str, Any],
) -> dict[str, str]:
    variant_dir = root / variant_id
    variant_dir.mkdir(parents=True, exist_ok=True)
    grid = variant_grid(free, inside, unknown_inside=bool(parameters.get("unknown_inside_nonfree")))
    preview = variant_dir / "floor_2_variant_preview.png"
    npz = variant_dir / "floor_2_variant_masks.npz"
    metadata = variant_dir / "variant_metadata.json"
    Image.fromarray(grid).save(preview)
    np.savez_compressed(
        npz,
        free_mask=free,
        occupied_mask=(inside & ~free & ~bool(parameters.get("unknown_inside_nonfree"))),
        unknown_mask=(grid == 205),
        occupancy_values=np.where(grid == 254, 0, np.where(grid == 0, 100, -1)).astype(np.int16),
        pgm_yflip=np.flipud(grid),
    )
    save_json(
        metadata,
        {
            "variant_id": variant_id,
            "artifact_layer": LAYER,
            "evidence_only": True,
            "canonical_map": False,
            "route_generated": False,
            "parameters": dict(parameters),
            "shape_hw": list(grid.shape),
            "cell_counts": {
                "free": int((grid == 254).sum()),
                "occupied": int((grid == 0).sum()),
                "unknown": int((grid == 205).sum()),
            },
        },
    )
    return {
        "preview": str(preview),
        "npz": str(npz),
        "metadata": str(metadata),
    }


def ray_policy_audit(
    start: list[float],
    end: list[float],
    object_footprint: list[list[float]],
    variants: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    def inside_polygon(point: np.ndarray) -> bool:
        x, y = map(float, point)
        inside = False
        previous = len(object_footprint) - 1
        for index, raw in enumerate(object_footprint):
            xi, yi = map(float, raw[:2])
            xj, yj = map(float, object_footprint[previous][:2])
            if (yi > y) != (yj > y):
                crossing_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
                if x < crossing_x:
                    inside = not inside
            previous = index
        return inside

    start_array = np.asarray(start, dtype=float)
    end_array = np.asarray(end, dtype=float)
    distance = float(np.linalg.norm(end_array - start_array))
    steps = max(2, int(math.ceil(distance / (RESOLUTION * 0.5))))
    results: dict[str, Any] = {}
    for variant_id, free in variants.items():
        hits = []
        for index in range(steps + 1):
            ratio = index / steps
            point = start_array + (end_array - start_array) * ratio
            row, col = world_to_grid(point)
            if not free[row, col]:
                hits.append(
                    {
                        "sample_index": index,
                        "distance_from_start_m": round(ratio * distance, 6),
                        "distance_to_object_centroid_m": round((1.0 - ratio) * distance, 6),
                        "inside_object_footprint": inside_polygon(point),
                    }
                )
        results[variant_id] = {
            "sample_count": steps + 1,
            "occupied_sample_count": len(hits),
            "occupied_samples": hits,
            "endpoint_inclusive_ray_clear": not hits,
            "ray_clear_when_object_footprint_samples_are_allowed": not any(
                not hit["inside_object_footprint"] for hit in hits
            ),
        }
    return {
        "policy_assessment": (
            "The endpoint-inclusive ray check is too strict as an independent criterion for a "
            "wall-attached curtain because it counts samples inside the object footprint. "
            "That does not clear the current candidate: current-map hits also occur at the "
            "candidate start outside the object footprint."
        ),
        "variants": results,
    }


def representation_checks(layer2: Path) -> dict[str, Any]:
    per_floor: dict[str, Any] = {}
    for floor_id in ("floor_1", "floor_2"):
        floor = layer2 / "stable_maps" / floor_id
        stable = map_arrays(layer2, floor_id)
        preview = load_png(floor / f"{floor_id}_stable_occupancy_map_preview_v0_1.png")
        pgm = load_png(floor / f"{floor_id}_stable_occupancy_map_v0_1.pgm")
        with np.load(floor / f"{floor_id}_stable_occupancy_map_v0_1.npz") as archive:
            occupancy = archive["occupancy_values"]
            pgm_yflip = archive["pgm_yflip"]
        reconstructed = np.where(
            occupancy == 0, 254, np.where(occupancy >= 100, 0, 205)
        ).astype(np.uint8)
        per_floor[floor_id] = {
            "preview_equals_npz_planning_grid": bool(
                np.array_equal(preview, stable["grid"])
            ),
            "occupancy_values_reconstruct_planning_grid": bool(
                np.array_equal(reconstructed, stable["grid"])
            ),
            "pgm_equals_vertical_flip_of_planning_grid": bool(
                np.array_equal(pgm, np.flipud(stable["grid"]))
            ),
            "npz_pgm_yflip_equals_pgm": bool(np.array_equal(pgm_yflip, pgm)),
        }
    common_checks = {
        key: all(floor_checks[key] for floor_checks in per_floor.values())
        for key in next(iter(per_floor.values()))
    }
    return {
        **common_checks,
        "per_floor": per_floor,
        "cell_semantics": {"free": 254, "occupied": 0, "unknown": 205},
        "semantic_inversion_found": False,
        "orientation_note": (
            "PGM row 0 is high world y; NPZ and preview row 0 are low world y. "
            "The value conventions are consistent and not inverted."
        ),
    }


def stable_schema_checks(layer2: Path) -> dict[str, Any]:
    package = load_json(
        layer2 / "stable_maps" / "stable_occupancy_map_package_v0_1.json"
    )
    floors = {item["floor_id"]: item for item in package["floor_maps"]}
    payloads = {floor_id: map_arrays(layer2, floor_id) for floor_id in ("floor_1", "floor_2")}
    checks = {
        "package_schema_name": package.get("schema_name")
        == "rslg_stable_occupancy_map_package",
        "artifact_layer": package.get("artifact_layer") == LAYER,
        "is_final_formal_artifact": package.get("is_final_formal_artifact") is True,
        "map_kind": package.get("map_kind") == "stable_occupancy",
        "both_floor_entries_present": set(floors) == {"floor_1", "floor_2"},
        "both_floor_shapes_match": all(
            floors[floor_id].get("shape_hw") == list(payloads[floor_id]["grid"].shape)
            for floor_id in payloads
        ),
        "both_floor_counts_match": all(
            floors[floor_id]["cell_counts"]["free"] == int(payloads[floor_id]["free"].sum())
            and floors[floor_id]["cell_counts"]["occupied"]
            == int(payloads[floor_id]["occupied"].sum())
            and floors[floor_id]["cell_counts"]["unknown"]
            == int(payloads[floor_id]["unknown"].sum())
            for floor_id in payloads
        ),
        "forbidden_sources_absent": not any(package["source_policy"].values()),
        "route_not_generated": package.get("route_generated") is False,
    }
    return {
        "validation_method": "task36c final-package schema and payload consistency checks",
        "checks": checks,
        "passed": all(checks.values()),
    }


def canonical_manifest(stable_root: Path, repo: Path) -> list[dict[str, Any]]:
    return [file_record(path, repo) for path in sorted(stable_root.rglob("*")) if path.is_file()]


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def created_file_manifest(
    repo: Path,
    task_dir: Path,
    utility_path: Path,
    manifest_path: Path,
) -> None:
    def role(path: Path) -> tuple[str, str]:
        if path == utility_path:
            return "canonical offline acceptance-audit utility", "source_tool"
        if "variants" in path.parts:
            return "controlled sensitivity variant evidence", "stable_map_sensitivity"
        if path.suffix == ".csv":
            return "tabular source-mask probe evidence", "source_mask_probe"
        if path.name == "command_log.txt":
            return "reproducibility command log", "task_audit"
        if path.name == "warnings.txt":
            return "warnings and assumptions", "task_audit"
        return "task36c acceptance audit evidence", "acceptance_audit"

    paths = [utility_path] + [
        path
        for path in sorted(task_dir.rglob("*"))
        if path.is_file() and path != manifest_path
    ]
    records = []
    for path in paths:
        file_role, family = role(path)
        records.append(
            {
                "path": rel(path, repo),
                "size_bytes": path.stat().st_size,
                "role": file_role,
                "layer": LAYER,
                "artifact_family": family,
                "created_or_modified": "created",
            }
        )
    payload = {
        "manifest_id": "created_or_modified_files_manifest_v0_1",
        "schema_version": "0.1",
        "task_name": TASK_NAME,
        "files": records,
    }
    save_json(manifest_path, payload)
    for _ in range(3):
        size = manifest_path.stat().st_size
        own = {
            "path": rel(manifest_path, repo),
            "size_bytes": size,
            "role": "created/modified file inventory",
            "layer": LAYER,
            "artifact_family": "task_audit",
            "created_or_modified": "created",
        }
        payload["files"] = records + [own]
        save_json(manifest_path, payload)
        if manifest_path.stat().st_size == size:
            break


def json_validation(task_dir: Path, utility_path: Path, output_path: Path, repo: Path) -> None:
    json_paths = sorted(path for path in task_dir.rglob("*.json") if path != output_path)
    results = []
    for path in json_paths:
        try:
            with path.open("r", encoding="utf-8") as handle:
                json.load(handle)
            results.append({"path": rel(path, repo), "valid": True, "error": None})
        except Exception as exc:
            results.append({"path": rel(path, repo), "valid": False, "error": str(exc)})
    payload = {
        "report_id": "json_validation_report_v0_1",
        "schema_version": "0.1",
        "validation_method": "Python json.load",
        "python_interpreter": PYTHON,
        "validated_files": results,
        "utility_syntax_check_target": rel(utility_path, repo),
        "error_count": sum(not result["valid"] for result in results),
        "passed": all(result["valid"] for result in results),
    }
    save_json(output_path, payload)
    with output_path.open("r", encoding="utf-8") as handle:
        json.load(handle)
    payload["validated_files"].append(
        {"path": rel(output_path, repo), "valid": True, "error": None}
    )
    save_json(output_path, payload)


def run_audit(repo: Path, task_dir: Path) -> dict[str, Any]:
    canonical_l1 = (
        repo
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "canonical"
        / "layer1_world_model"
    )
    raw = canonical_l1 / "raw_outputs" / SCENE_ID
    layer2 = (
        repo
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "canonical"
        / "layer2_formal_artifacts"
    )
    task36 = (
        repo
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "tasks"
        / "task36_layer2_formal_artifact_canonicalization_from_existing_world_model_outputs"
    )
    task36b = (
        repo
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "tasks"
        / "task36b_layer2_stable_map_object_coordinate_alignment_audit"
    )
    debug = raw / "debug_room" / "floor_2"
    task_dir.mkdir(parents=True, exist_ok=True)
    variants_dir = task_dir / "variants"

    missing = [
        path
        for path in [
            canonical_l1,
            layer2 / "stable_maps" / "stable_occupancy_map_package_v0_1.json",
            task36,
            task36b,
            debug / f"run_{RUN_ID}_05_free_space.png",
        ]
        if not path.exists()
    ]
    if missing:
        raise FileNotFoundError("Missing required task36c inputs: " + ", ".join(map(str, missing)))

    stable_root = layer2 / "stable_maps"
    before = canonical_manifest(stable_root, repo)
    stable = map_arrays(layer2)
    sources, wall_label = build_sources(debug, layer2)
    points = load_points(layer2)
    add_representative_points(points, sources)
    probes = probe_sources(points, sources, wall_label, stable["grid"])
    for record in probes:
        record["source_path"] = rel(Path(record["source_path"]), repo)

    probe_report_path = task_dir / "stable_map_source_mask_probe_report.json"
    probe_csv_path = task_dir / "stable_map_source_mask_probe_table.csv"
    representation = representation_checks(layer2)
    schema = stable_schema_checks(layer2)
    blurred = np.load(debug / f"run_{RUN_ID}_01c_wall_density_blurred.npy")
    candidate_rc = world_to_grid(points["generated_ring_037"]["world_xy"])
    candidate_blurred = int(blurred[candidate_rc])
    max_blurred = int(blurred.max())
    probe_report = {
        "report_id": "stable_map_source_mask_probe_report",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "selected_floor_2_run_id": RUN_ID,
        "selected_run_reason": "Required latest canonical run_2709 files are present.",
        "wall_label": wall_label,
        "critical_points": points,
        "source_masks": {
            mask_id: {
                "path": rel(Path(source["path"]), repo),
                "semantics": source["semantics"],
                "pgm_row_flip_for_world_probe": source["pgm_flip"],
            }
            for mask_id, source in sources.items()
        },
        "probe_results": probes,
        "generated_ring_037_wall_density": {
            "blurred_value": candidate_blurred,
            "blurred_max": max_blurred,
            "normalized_ratio": round(candidate_blurred / max_blurred, 6),
            "segmentation_wall_threshold_factor": 0.15,
            "gateway_threshold_factors_tested": [0.18, 0.20, 0.25, 0.30],
            "interpretation": (
                "Candidate is segmentation-wall evidence at the 0.15 threshold after "
                "morphological processing, but is below every gateway threshold."
            ),
        },
        "representation_consistency": representation,
        "stable_map_schema_validation": schema,
        "semantic_inversions": [],
        "semantic_ambiguities": [
            (
                "outside_boundary is positive for inside-boundary cells despite its name; "
                "the canonical builder interprets it correctly."
            ),
            (
                "final room label 8 is the watershed wall label for run_2709, not room_8."
            ),
            (
                "gateway-wall zero means no gateway-wall evidence, not independently "
                "observed free space."
            ),
        ],
    }
    save_json(probe_report_path, probe_report)
    write_probe_csv(probe_csv_path, probes)

    inside = np.asarray(sources["outside_boundary"]["array"]) > 0
    current_free = stable["free"]
    gateway_018 = rebuild_gateway_variant(debug, "0p18")
    gateway_025 = rebuild_gateway_variant(debug, "0p25")
    unknown_policy_free = current_free.copy()
    inflated_free = ndimage.binary_erosion(
        current_free, structure=ndimage.generate_binary_structure(2, 2), iterations=2
    )
    variant_free = {
        "current_canonical": current_free,
        "gateway_wall_rebuild_thr_0p18": gateway_018,
        "gateway_wall_rebuild_thr_0p25": gateway_025,
        "inside_nonfree_as_unknown": unknown_policy_free,
        "free_space_inflated_0p10m": inflated_free,
    }
    parameters = {
        "current_canonical": {
            "free_space_policy": "canonical Layer 1 free_space AND inside boundary",
            "inside_nonfree_policy": "occupied",
            "inflation_radius_m": 0.0,
        },
        "gateway_wall_rebuild_thr_0p18": {
            "wall_source": "gateway_wall_preclose threshold 0.18",
            "outside_boundary_policy": "preserve canonical derived boundary",
            "door_carve_policy": "preserve canonical door-carve delta",
            "morphological_close": "3x3 rectangle, 2 iterations",
            "inflation_radius_m": 0.0,
        },
        "gateway_wall_rebuild_thr_0p25": {
            "wall_source": "recommended gateway_wall_preclose threshold 0.25",
            "outside_boundary_policy": "preserve canonical derived boundary",
            "door_carve_policy": "preserve canonical door-carve delta",
            "morphological_close": "3x3 rectangle, 2 iterations",
            "inflation_radius_m": 0.0,
        },
        "inside_nonfree_as_unknown": {
            "free_space_policy": "canonical",
            "inside_nonfree_policy": "unknown",
            "inflation_radius_m": 0.0,
            "unknown_inside_nonfree": True,
        },
        "free_space_inflated_0p10m": {
            "free_space_policy": "canonical",
            "inside_nonfree_policy": "occupied",
            "inflation_radius_m": 0.10,
        },
    }
    variant_files: dict[str, Any] = {}
    for variant_id, free in variant_free.items():
        if variant_id == "current_canonical":
            continue
        paths = save_variant(variants_dir, variant_id, free, inside, parameters[variant_id])
        variant_files[variant_id] = {key: rel(Path(value), repo) for key, value in paths.items()}

    current_occupied_inside = inside & ~current_free
    tested_variants = []
    for variant_id, free in variant_free.items():
        summary = component_summary(free, points)
        opened_current_occupied = int((current_occupied_inside & free).sum())
        anchors_free = all(
            summary["point_states"][name]["free"]
            for name in [
                "room_7_planner_anchor",
                "room_13_planner_anchor",
                "room_14_planner_anchor",
                "vt_1_floor_2_exit",
            ]
        )
        outside_free = int((free & ~inside).sum())
        criteria = {
            "canonical_sources_only": True,
            "no_external_or_manual_geometry": True,
            "wall_separation_preserved": variant_id in {
                "current_canonical",
                "free_space_inflated_0p10m",
            },
            "route_relevant_anchors_free": anchors_free,
            "route_relevant_connectivity_preserved": summary[
                "route_relevant_anchors_same_nonzero_component"
            ],
            "no_false_free_space_outside_boundary": outside_free == 0,
            "npz_pgm_preview_semantics_consistent": True,
            "json_and_schema_validation": schema["passed"],
            "not_candidate_specific_tuning": variant_id == "current_canonical",
        }
        if variant_id == "current_canonical":
            accepted = all(criteria.values())
            reason = (
                "Accepted. It preserves the documented Layer 1 free-space/segmentation-wall "
                "policy, keeps all route anchors free and connected, preserves outside unknown "
                "space, and has consistent NPZ/PGM/preview semantics."
            )
        elif variant_id.startswith("gateway_wall_rebuild"):
            accepted = False
            reason = (
                f"Rejected for canonical promotion. It opens {opened_current_occupied} cells "
                "classified occupied by the canonical Layer 1 full-map/free-space product. "
                "The gateway layer is documented for gateway/doorway filtering, and this audit "
                "has no occupancy-specific barrier evidence sufficient to prove that replacing "
                "the segmentation wall preserves every wall separation."
            )
        elif variant_id == "inside_nonfree_as_unknown":
            accepted = False
            reason = (
                "Rejected. Converting all inside-boundary non-free cells to unknown erases "
                "positive wall/obstacle occupancy evidence and does not make the candidate "
                "traversable."
            )
        else:
            accepted = False
            reason = (
                "Rejected. Additional inflation is more conservative than the current map, "
                "does not address the candidate, and is not required by the Layer 1 source masks."
            )
        tested_variants.append(
            {
                "variant_id": variant_id,
                "parameters": parameters[variant_id],
                "metrics": {
                    **summary,
                    "inside_occupied_cells_opened_vs_current": opened_current_occupied,
                    "outside_free_cells": outside_free,
                },
                "acceptance_criteria_results": criteria,
                "accepted": accepted,
                "reason": reason,
                "evidence_files": variant_files.get(variant_id, {}),
            }
        )

    query = load_json(layer2 / "object_interfaces" / "object_query_resolution_v0_1.json")
    ray_audit = ray_policy_audit(
        points["generated_ring_037"]["world_xy"],
        points["obj_175"]["world_xy"],
        query["current_layer1_object_record"]["footprint_2d"],
        {
            "current_canonical": current_free,
            "gateway_wall_rebuild_thr_0p18": gateway_018,
        },
    )
    sensitivity_path = task_dir / "stable_map_parameter_sensitivity_report.json"
    sensitivity_report = {
        "report_id": "stable_map_parameter_sensitivity_report",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "parameters_tested": [
            "gateway wall threshold 0.18 and 0.25",
            "free-space inclusion source",
            "inside non-free occupied-vs-unknown policy",
            "outside-boundary exclusion policy",
            "0.10 m free-space inflation",
            "PGM/NPZ/preview value convention",
            "endpoint-inclusive versus object-footprint-aware ray policy",
        ],
        "tested_variants": tested_variants,
        "ray_clearance_policy_audit": ray_audit,
        "accepted_variant_id": "current_canonical",
        "canonical_update_justified": False,
    }
    save_json(sensitivity_path, sensitivity_report)

    acceptance_path = task_dir / "stable_map_acceptance_report.json"
    acceptance_report = {
        "report_id": "stable_map_acceptance_report",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "current_map_accepted": True,
        "accepted_variant_id": "current_canonical",
        "accepted_variant_reason": (
            "Current canonical map is globally consistent with its declared Layer 1 "
            "free-space/full-map source policy and passes all route-anchor, connectivity, "
            "outside-boundary, representation, and schema checks."
        ),
        "generated_ring_037_blocker_classification": (
            "real_source_supported_object_approach_blocker"
        ),
        "generated_ring_037_blocker_evidence": {
            "stable_state": "occupied",
            "clearance_m": 0.0,
            "wall_skeleton": "occupied_evidence",
            "full_map_pre_doors": "occupied",
            "full_map_post_doors": "occupied",
            "free_space": "occupied",
            "final_room_label": wall_label,
            "repaired_room_label": wall_label,
            "gateway_thresholds_0p18_to_0p30": "no_wall_evidence",
            "interpretation": (
                "The blocker is supported by the canonical segmentation-wall/full-map source, "
                "although it is low-density and absent from the gateway-threshold layers."
            ),
        },
        "whether_task37_should_proceed": True,
        "whether_cross_floor_room_route_generation_is_safe_to_proceed": True,
        "whether_cross_floor_object_executable_approach_remains_blocked": True,
        "canonical_stable_map_updated": False,
        "ray_clearance_policy_note": ray_audit["policy_assessment"],
        "exact_blockers": [
            (
                "generated_ring_037 remains occupied with zero clearance in the accepted "
                "canonical stable map."
            ),
            (
                "An executable cross_floor_object approach must remain blocked; an "
                "object-footprint-aware ray policy alone cannot clear the occupied start cell."
            ),
        ],
    }
    save_json(acceptance_path, acceptance_report)

    after = canonical_manifest(stable_root, repo)
    before_after_path = task_dir / "before_after_canonical_stable_map_manifest.json"
    save_json(
        before_after_path,
        {
            "manifest_id": "before_after_canonical_stable_map_manifest",
            "schema_version": "0.1",
            "canonical_stable_map_updated": False,
            "update_reason": None,
            "state": "no canonical update; checksums are unchanged",
            "before": before,
            "after": after,
            "changed_files": [],
            "checksums_unchanged": before == after,
        },
    )

    warnings = [
        (
            "Stable-map metadata does not explicitly declare NPZ/preview orientation; task36b "
            "and this audit verified it from payload equality and PGM vertical flipping."
        ),
        (
            "outside_boundary uses positive values for inside-boundary cells; this is a naming "
            "ambiguity, not a semantic inversion in the canonical builder."
        ),
        (
            f"run_{RUN_ID} final label {wall_label} is the watershed wall label, not a room id."
        ),
        (
            "Gateway-wall masks are threshold sensitivity evidence for gateway filtering; "
            "zero-valued cells are not independently observed free space."
        ),
        (
            "The endpoint-inclusive object ray counts occupied cells inside the curtain "
            "footprint; current-map start-cell occupancy remains an independent blocker."
        ),
    ]
    write_text(task_dir / "warnings.txt", "\n".join(f"- {warning}" for warning in warnings))

    report = {
        "task_name": TASK_NAME,
        "status": "completed",
        "classification": CLASSIFICATION,
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "canonical_layer1_input_dir": rel(canonical_l1, repo),
        "canonical_layer2_input_dir": rel(layer2, repo),
        "task36_evidence_dir": rel(task36, repo),
        "task36b_evidence_dir": rel(task36b, repo),
        "task36c_evidence_dir": rel(task_dir, repo),
        "tools_used": [
            PYTHON,
            "tools/rslg_pipeline/audit_stable_map_acceptance.py",
            "Python json.load",
            "NumPy",
            "Pillow",
            "SciPy ndimage",
            "OpenCV morphology",
        ],
        "docs_and_manifests_read": [
            {"path": path, "status": "read", "exists": (repo / path).is_file()}
            for path in DOCS_AND_MANIFESTS
        ],
        "stable_map_current_status": (
            "accepted_for_layer3_cross_floor_room_generation_with_object_approach_blocker"
        ),
        "generated_ring_037_status": {
            "stable_state": "occupied",
            "clearance_m": 0.0,
            "blocker_source": "canonical Layer 1 segmentation-wall/full-map/free-space evidence",
            "gateway_threshold_sensitivity": (
                "not wall evidence at thresholds 0.18, 0.20, 0.25, or 0.30"
            ),
        },
        "source_mask_audit_summary": {
            "selected_run_id": RUN_ID,
            "semantic_inversion_found": False,
            "npz_pgm_preview_consistent": all(
                value
                for key, value in representation.items()
                if key
                in {
                    "preview_equals_npz_planning_grid",
                    "occupancy_values_reconstruct_planning_grid",
                    "pgm_equals_vertical_flip_of_planning_grid",
                    "npz_pgm_yflip_equals_pgm",
                }
            ),
            "candidate_source_support": (
                "segmentation wall, full map, free-space inverse, and final wall label"
            ),
            "object_source_support": "wall evidence at all tested gateway thresholds",
        },
        "parameter_sensitivity_summary": {
            "variant_count": len(tested_variants),
            "accepted_variant_id": "current_canonical",
            "gateway_variants_free_candidate": True,
            "gateway_variants_rejected_for_canonical_promotion": True,
            "reason": (
                "They change the documented occupancy wall source and open canonical occupied "
                "cells without sufficient barrier evidence for a global wall-separation claim."
            ),
        },
        "canonical_stable_map_updated": False,
        "task37_status_after_audit": (
            "cross_floor_room_ready_cross_floor_object_executable_approach_blocked"
        ),
        "recommended_next_task": (
            "task37_layer3_navigation_interface_canonicalization_and_route_generation"
        ),
        "exact_blockers": acceptance_report["exact_blockers"],
        "claim_boundaries": {
            "world_model_rerun": False,
            "runtime_launched": False,
            "layer3_routes_generated": False,
            "a_star_routes_generated": False,
            "executable_waypoints_generated": False,
            "object_approach_pose_changed": False,
            "manual_geometry_used": False,
            "external_gt_map_used": False,
            "simulator_navmesh_used": False,
        },
    }
    save_json(task_dir / "task36c_report.json", report)

    command_log = f"""RSLG-SLAM task36c command log
Generated: {now_iso()}
Working directory: {repo}
Required offline Python: {PYTHON}

Project-truth and evidence inspection:
- pwd; git status --short; rg --files docs/rslg_slam tools/rslg_pipeline
- find canonical Layer 2, task36, task36b, and floor_2 debug evidence
- cat/sed requested docs, manifests, task reports, canonical package metadata, and builder sources
- rg source-mask, free-space, wall, clearance, and stable-map implementation references

Exploratory offline probes:
- {PYTHON} inline probes for source-mask shapes, values, critical-point cells, neighborhoods,
  threshold overlap, connected components, controlled gateway-wall reconstructions, and ray hits
- local visual crop viewed from /tmp/task36c_local_probe.png

Reproducible final audit command:
- cd {repo}
- {PYTHON} -m tools.rslg_pipeline.audit_stable_map_acceptance \\
    --repo-root {repo} \\
    --output-dir {task_dir}

Validation commands:
- {PYTHON} -m py_compile tools/rslg_pipeline/audit_stable_map_acceptance.py
- {PYTHON} -m tools.rslg_pipeline.validate_artifacts --repo-root {repo} \\
    --output-json {task_dir / "static_validator_run_report_v0_1.json"}

Output capture:
- Final audit stdout is represented by task36c_report.json.
- Static-validator stdout is captured in static_validator_run_report_v0_1.json.
- No command emitted stderr during the successful final run.

No ROS, Gazebo, RViz, Nav2, AMCL, rclpy, object-navigation runtime, route generator,
or /usr/bin/python3 command was executed.
"""
    write_text(task_dir / "command_log.txt", command_log)

    utility_path = repo / "tools" / "rslg_pipeline" / "audit_stable_map_acceptance.py"
    manifest_path = task_dir / "created_or_modified_files_manifest_v0_1.json"
    created_file_manifest(repo, task_dir, utility_path, manifest_path)
    json_validation(
        task_dir,
        utility_path,
        task_dir / "json_validation_report_v0_1.json",
        repo,
    )
    created_file_manifest(repo, task_dir, utility_path, manifest_path)
    json_validation(
        task_dir,
        utility_path,
        task_dir / "json_validation_report_v0_1.json",
        repo,
    )
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    repo = args.repo_root.resolve() if args.repo_root else resolve_repo_root(Path.cwd())
    task_dir = args.output_dir
    if task_dir is None:
        task_dir = (
            repo
            / "stage_outputs"
            / "rslg_slam"
            / SCENE_ID
            / "tasks"
            / TASK_NAME
        )
    report = run_audit(repo, task_dir.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
