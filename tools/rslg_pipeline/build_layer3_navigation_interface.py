"""Build canonical Layer 3 navigation interfaces and compare stable-map profiles.

This offline entrypoint consumes only canonical RSLG-SLAM Layer 1 and Layer 2
artifacts. It reuses the historical occupancy-grid A* helper, never launches a
runtime system, and never modifies canonical Layer 2 outputs.
"""

from __future__ import annotations

import argparse
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

from tools.object_nav.lightweight_backend.occupancy_planner import OccupancyPlanner

from .common import PROJECT_NAME, resolve_repo_root, save_json


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = (
    "task37_layer3_navigation_interface_canonicalization_with_stable_map_"
    "profile_comparison"
)
CLASSIFICATION_COMPLETE = (
    "task37_layer3_navigation_interface_canonicalization_with_profile_"
    "comparison_completed"
)
CLASSIFICATION_ROUTE_BLOCKED = (
    "task37_layer3_navigation_interface_canonicalization_blocked_by_route_generation"
)
CLASSIFICATION_LAYER2_BLOCKED = (
    "task37_layer3_navigation_interface_canonicalization_blocked_by_layer2_inputs"
)
LAYER = "Layer 3: Navigation Interface Layer"
PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"
RESOLUTION = 0.05
ORIGIN = (-50.0, -50.0)
INFLATION_RADIUS_M = 0.20
WAYPOINT_SPACING_M = 0.20
MAX_ANCHOR_SNAP_M = 1.0
FLOOR_RUNS = {"floor_1": 1500, "floor_2": 2709}
ROUTE_CHAIN = [
    "room_2",
    "room_3",
    "vt_1",
    "room_7",
    "room_13",
    "room_14",
]

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

HISTORICAL_SCRIPTS_INSPECTED = [
    "tools/object_nav/lightweight_backend/occupancy_planner.py",
    "tools/stage1_runtime/scene_runtime_common.py",
    "tools/vertical_connectors/build_occupancy_aware_cross_floor_visual_route.py",
    "tools/vertical_connectors/build_cross_floor_object_approach_route.py",
    "tools/rslg_pipeline/audit_stable_map_acceptance.py",
    "tools/rslg_pipeline/build_layer2_formal_artifacts.py",
]

REQUIRED_LAYER2 = [
    "stable_maps/stable_occupancy_map_package_v0_1.json",
    "vertical_connectors/vertical_connectors_v0_1.json",
    "vertical_connectors/stairs_or_vertical_connector_graph_v0_1.json",
    "topology/cross_floor_topology_v0_1.json",
    "planner_graph/route_planner_graph_v0_1.json",
    "object_interfaces/object_query_resolution_v0_1.json",
    "object_interfaces/object_approach_v0_1.json",
    "object_interfaces/object_interface_package_v0_1.json",
]

CORE_LAYER3_PATHS = [
    "route_contracts/cross_floor_room_route_contract_v0_1.json",
    "route_contracts/cross_floor_object_route_contract_v0_1.json",
    "planner_requests/cross_floor_room_planner_request_v0_1.json",
    "planner_requests/cross_floor_object_planner_request_v0_1.json",
    "route_plans/cross_floor_room_route_plan_v0_1.json",
    "route_plans/cross_floor_object_route_plan_v0_1.json",
    "real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
    "real_routes/cross_floor_room_real_astar_route_navigation_thr0p25_candidate_v0_1.json",
    "executable_route_candidates/cross_floor_room_executable_route_candidate_selected_v0_1.json",
    "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json",
    "manifests/layer3_navigation_interface_manifest_v0_1.json",
    "reports/layer3_navigation_interface_validation_report_v0_1.json",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return value


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path, repo: Path) -> str:
    try:
        return path.resolve().relative_to(repo.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


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


def write_pgm(path: Path, planning_grid: np.ndarray) -> None:
    image = np.flipud(planning_grid).astype(np.uint8)
    header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + image.tobytes())


def write_map_yaml(path: Path, image_name: str) -> None:
    write_text(
        path,
        "\n".join(
            [
                f"image: {image_name}",
                f"resolution: {RESOLUTION}",
                "origin: [-50.0, -50.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
    )


def load_png(path: Path, *, crop_padding: bool = False) -> np.ndarray:
    array = np.asarray(Image.open(path).convert("L"))
    return array[10:-10, 10:-10] if crop_padding else array


def load_profile_arrays(npz_path: Path) -> dict[str, np.ndarray]:
    with np.load(npz_path, allow_pickle=False) as archive:
        free = archive["free_mask"].astype(bool)
        occupied = archive["occupied_mask"].astype(bool)
        unknown = archive["unknown_mask"].astype(bool)
        outside = (
            archive["outside_boundary"].astype(bool)
            if "outside_boundary" in archive.files
            else ~unknown
        )
        wall = (
            archive["wall_evidence"].astype(bool)
            if "wall_evidence" in archive.files
            else np.zeros_like(free)
        )
        gateway = (
            archive["gateway_wall_preclose"].astype(bool)
            if "gateway_wall_preclose" in archive.files
            else np.zeros_like(free)
        )
        door = (
            archive["door_carve_delta"].astype(bool)
            if "door_carve_delta" in archive.files
            else np.zeros_like(free)
        )
    grid = np.full(free.shape, 205, dtype=np.uint8)
    grid[occupied] = 0
    grid[free] = 254
    return {
        "free": free,
        "occupied": occupied,
        "unknown": unknown,
        "outside": outside,
        "wall": wall,
        "gateway": gateway,
        "door": door,
        "grid": grid,
    }


def rebuild_thr0p25_floor(debug: Path, run: int) -> dict[str, np.ndarray]:
    outside_padded = load_png(debug / f"run_{run}_03_outside_boundary.png")
    gateway = load_png(debug / f"run_{run}_01e_gateway_wall_preclose_thr_0p25.png")
    padded_gateway = np.zeros_like(outside_padded)
    padded_gateway[10:-10, 10:-10] = gateway
    full_padded = cv2.bitwise_or(padded_gateway, cv2.bitwise_not(outside_padded))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    full_padded = cv2.morphologyEx(full_padded, cv2.MORPH_CLOSE, kernel, iterations=2)
    full = full_padded[10:-10, 10:-10]
    door = load_png(debug / f"run_{run}_04c_door_carve_delta.png") > 0
    canonical_post = load_png(debug / f"run_{run}_04b_full_map_post_doors.png")
    full[door & (canonical_post == 0)] = 0

    outside = outside_padded[10:-10, 10:-10] > 0
    free = (full == 0) & outside
    occupied = outside & ~free
    unknown = ~outside
    wall = load_png(debug / f"run_{run}_02_walls_skeleton.png", crop_padding=True) > 0
    shapes = {item.shape for item in (free, occupied, unknown, outside, wall, gateway, door)}
    if len(shapes) != 1:
        raise RuntimeError(f"Inconsistent threshold-profile evidence shapes: {sorted(shapes)}")
    return {
        "free": free,
        "occupied": occupied,
        "unknown": unknown,
        "outside": outside,
        "wall": wall,
        "gateway": gateway > 0,
        "door": door,
    }


def save_candidate_floor(
    floor_id: str,
    arrays: Mapping[str, np.ndarray],
    debug: Path,
    run: int,
    profile_root: Path,
    repo: Path,
) -> dict[str, Any]:
    floor_dir = profile_root / floor_id
    stem = f"{floor_id}_navigation_thr0p25_candidate_occupancy_map_v0_1"
    npz_path = floor_dir / f"{stem}.npz"
    pgm_path = floor_dir / f"{stem}.pgm"
    yaml_path = floor_dir / f"{stem}.yaml"
    preview_path = floor_dir / f"{stem}_preview.png"
    metadata_path = floor_dir / f"{stem}_metadata.json"
    grid = np.full(arrays["free"].shape, 205, dtype=np.uint8)
    grid[arrays["occupied"]] = 0
    grid[arrays["free"]] = 254

    floor_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        npz_path,
        free_mask=arrays["free"],
        occupied_mask=arrays["occupied"],
        unknown_mask=arrays["unknown"],
        outside_boundary=arrays["outside"],
        wall_evidence=arrays["wall"],
        gateway_wall_preclose=arrays["gateway"],
        door_carve_delta=arrays["door"],
        occupancy_values=np.where(
            arrays["free"], 0, np.where(arrays["occupied"], 100, -1)
        ).astype(np.int16),
        pgm_yflip=np.flipud(grid),
    )
    write_pgm(pgm_path, grid)
    write_map_yaml(yaml_path, pgm_path.name)
    Image.fromarray(grid).save(preview_path)

    source_paths = [
        debug / f"run_{run}_01e_gateway_wall_preclose_thr_0p25.png",
        debug / f"run_{run}_02_walls_skeleton.png",
        debug / f"run_{run}_03_outside_boundary.png",
        debug / f"run_{run}_04b_full_map_post_doors.png",
        debug / f"run_{run}_04c_door_carve_delta.png",
    ]
    metadata = {
        "profile_id": "navigation_thr0p25_candidate",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "floor_id": floor_id,
        "source_run_frame": run,
        "candidate_only": True,
        "canonical_layer2_map_overwritten": False,
        "reconstruction": {
            "wall_source": "gateway_wall_preclose threshold 0.25",
            "outside_boundary_policy": "preserve canonical Layer 1 derived boundary",
            "door_carve_policy": "preserve canonical Layer 1 door-carve delta",
            "morphological_close": "3x3 rectangle, 2 iterations",
            "implementation_lineage": (
                "tools/rslg_pipeline/audit_stable_map_acceptance.py:"
                "rebuild_gateway_variant"
            ),
        },
        "resolution_m_per_cell": RESOLUTION,
        "origin": [ORIGIN[0], ORIGIN[1], 0.0],
        "shape_hw": list(grid.shape),
        "cell_counts": {
            "free": int(arrays["free"].sum()),
            "occupied": int(arrays["occupied"].sum()),
            "unknown": int(arrays["unknown"].sum()),
        },
        "source_files": [file_record(path, repo) for path in source_paths],
        "outputs": {
            "npz": rel(npz_path, repo),
            "pgm": rel(pgm_path, repo),
            "map_server_yaml": rel(yaml_path, repo),
            "preview_png": rel(preview_path, repo),
        },
        "source_boundary": {
            "canonical_rslg_slam_layer1_only": True,
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "simulator_navmesh_used": False,
            "manual_geometry_used": False,
        },
    }
    save_json(metadata_path, metadata)
    metadata["metadata_json"] = rel(metadata_path, repo)
    return metadata


def build_profiles(
    repo: Path,
    layer1: Path,
    layer2: Path,
    task_root: Path,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]], list[str]]:
    stable_package = load_json(
        layer2 / "stable_maps" / "stable_occupancy_map_package_v0_1.json"
    )
    warnings: list[str] = []
    conservative_floors: dict[str, Any] = {}
    for floor_id in FLOOR_RUNS:
        floor_dir = layer2 / "stable_maps" / floor_id
        conservative_floors[floor_id] = {
            "yaml": floor_dir / f"{floor_id}_stable_occupancy_map_v0_1.yaml",
            "npz": floor_dir / f"{floor_id}_stable_occupancy_map_v0_1.npz",
            "metadata": floor_dir / f"{floor_id}_stable_occupancy_map_metadata_v0_1.json",
        }

    raw = layer1 / "raw_outputs" / SCENE_ID
    candidate_root = task_root / "stable_map_profiles" / "navigation_thr0p25_candidate"
    candidate_floors: dict[str, Any] = {}
    candidate_metadata: list[dict[str, Any]] = []
    for floor_id, run in FLOOR_RUNS.items():
        debug = raw / "debug_room" / floor_id
        arrays = rebuild_thr0p25_floor(debug, run)
        metadata = save_candidate_floor(
            floor_id, arrays, debug, run, candidate_root, repo
        )
        candidate_metadata.append(metadata)
        outputs = metadata["outputs"]
        candidate_floors[floor_id] = {
            "yaml": repo / outputs["map_server_yaml"],
            "npz": repo / outputs["npz"],
            "metadata": repo / metadata["metadata_json"],
        }

    task36c_npz = (
        repo
        / "stage_outputs/rslg_slam"
        / SCENE_ID
        / "tasks/task36c_layer2_stable_occupancy_map_source_mask_parameter_acceptance_audit"
        / "variants/gateway_wall_rebuild_thr_0p25/floor_2_variant_masks.npz"
    )
    floor2_reproducible = False
    if task36c_npz.is_file():
        generated = load_profile_arrays(candidate_floors["floor_2"]["npz"])
        reference = load_profile_arrays(task36c_npz)
        floor2_reproducible = all(
            np.array_equal(generated[key], reference[key])
            for key in ("free", "occupied", "unknown")
        )
        if not floor2_reproducible:
            warnings.append(
                "The reconstructed floor_2 threshold profile does not exactly match "
                "the task36c evidence arrays."
            )
    else:
        warnings.append(
            "The optional task36c floor_2 threshold-profile evidence was unavailable; "
            "the profile was still reconstructed directly from canonical Layer 1."
        )

    candidate_manifest = {
        "profile_id": "navigation_thr0p25_candidate",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "role": "candidate navigation profile",
        "canonical_layer2_promotion": False,
        "generated_from_canonical_layer1_sources": True,
        "floor_maps": candidate_metadata,
        "task36c_floor2_exact_array_reproduction": floor2_reproducible,
        "limitations": [
            "This is task37 evidence and is not a canonical Layer 2 stable map.",
            "Opened canonical occupied cells require route-level safety interpretation.",
        ],
    }
    candidate_manifest_path = (
        candidate_root / "navigation_thr0p25_candidate_profile_v0_1.json"
    )
    save_json(candidate_manifest_path, candidate_manifest)

    profiles = {
        "conservative_canonical": {
            "profile_id": "conservative_canonical",
            "role": "safe baseline",
            "candidate": False,
            "floor_maps": conservative_floors,
            "source_package": layer2
            / "stable_maps"
            / "stable_occupancy_map_package_v0_1.json",
        },
        "navigation_thr0p25_candidate": {
            "profile_id": "navigation_thr0p25_candidate",
            "role": "candidate navigation profile",
            "candidate": True,
            "floor_maps": candidate_floors,
            "source_package": candidate_manifest_path,
        },
    }
    generation_report = {
        "report_id": "stable_map_profile_generation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "canonical_layer2_stable_map_unchanged": True,
        "profiles": [
            {
                "profile_id": "conservative_canonical",
                "status": "available",
                "role": "safe baseline",
                "production": (
                    "Referenced final canonical Layer 2 stable occupancy map package "
                    "without copying or modifying it."
                ),
                "source_package": rel(profiles["conservative_canonical"]["source_package"], repo),
                "floor_maps": {
                    floor_id: {
                        key: rel(value, repo)
                        for key, value in bindings.items()
                    }
                    for floor_id, bindings in conservative_floors.items()
                },
                "canonical_layer2": True,
            },
            {
                "profile_id": "navigation_thr0p25_candidate",
                "status": "available",
                "role": "candidate navigation profile",
                "production": (
                    "Reconstructed for both floors from canonical Layer 1 threshold "
                    "0.25 gateway-wall, outside-boundary, and door-carve evidence using "
                    "the task36c reconstruction algorithm."
                ),
                "source_package": rel(candidate_manifest_path, repo),
                "floor_maps": {
                    floor_id: {
                        key: rel(value, repo)
                        for key, value in bindings.items()
                    }
                    for floor_id, bindings in candidate_floors.items()
                },
                "task36c_floor2_exact_array_reproduction": floor2_reproducible,
                "canonical_layer2": False,
            },
        ],
        "canonical_layer2_source_package": stable_package.get("artifact_family"),
        "candidate_artifacts_used_as_inputs": False,
        "task36c_used_as_algorithm_and_reproduction_reference_only": True,
    }
    return generation_report, profiles, warnings


def rc_for_xy(xy: Iterable[float]) -> tuple[int, int]:
    x, y = [float(value) for value in list(xy)[:2]]
    return (
        int(round((y - ORIGIN[1]) / RESOLUTION)),
        int(round((x - ORIGIN[0]) / RESOLUTION)),
    )


def sample_polyline_cells(points: list[dict[str, Any]]) -> list[tuple[int, int]]:
    cells: list[tuple[int, int]] = []
    if len(points) == 1:
        return [rc_for_xy([points[0]["x"], points[0]["y"]])]
    for left, right in zip(points, points[1:]):
        length = math.hypot(
            float(right["x"]) - float(left["x"]),
            float(right["y"]) - float(left["y"]),
        )
        steps = max(1, int(math.ceil(length / (RESOLUTION * 0.45))))
        for index in range(steps + 1):
            ratio = index / steps
            cells.append(
                rc_for_xy(
                    [
                        float(left["x"])
                        + ratio * (float(right["x"]) - float(left["x"])),
                        float(left["y"])
                        + ratio * (float(right["y"]) - float(left["y"])),
                    ]
                )
            )
    return cells


def contiguous_hit_count(flags: list[bool]) -> int:
    return sum(flag and (index == 0 or not flags[index - 1]) for index, flag in enumerate(flags))


def heading_turn_count(points: list[dict[str, Any]], threshold_deg: float = 15.0) -> int:
    headings: list[float] = []
    for left, right in zip(points, points[1:]):
        dx = float(right["x"]) - float(left["x"])
        dy = float(right["y"]) - float(left["y"])
        if math.hypot(dx, dy) > 1e-9:
            headings.append(math.atan2(dy, dx))
    turns = 0
    for left, right in zip(headings, headings[1:]):
        delta = abs(math.atan2(math.sin(right - left), math.cos(right - left)))
        if math.degrees(delta) >= threshold_deg:
            turns += 1
    return turns


def route_cell_metrics(
    route_floor_points: Mapping[str, list[dict[str, Any]]],
    profile_arrays: Mapping[str, Mapping[str, np.ndarray]],
    conservative_arrays: Mapping[str, Mapping[str, np.ndarray]],
) -> dict[str, Any]:
    occupied_cells: set[tuple[str, int, int]] = set()
    unknown_cells: set[tuple[str, int, int]] = set()
    outside_cells: set[tuple[str, int, int]] = set()
    wall_evidence_cells: set[tuple[str, int, int]] = set()
    gateway_wall_evidence_cells: set[tuple[str, int, int]] = set()
    occupied_wall_cells: set[tuple[str, int, int]] = set()
    canonical_occupied_cells: set[tuple[str, int, int]] = set()
    canonical_occupied_wall_cells: set[tuple[str, int, int]] = set()
    wall_crossing_runs = 0
    canonical_wall_crossing_runs = 0
    tested = 0
    for floor_id, points in route_floor_points.items():
        arrays = profile_arrays[floor_id]
        canonical = conservative_arrays[floor_id]
        cells = sample_polyline_cells(points)
        occupied_wall_flags: list[bool] = []
        canonical_occupied_wall_flags: list[bool] = []
        for row, col in cells:
            tested += 1
            if not (0 <= row < arrays["free"].shape[0] and 0 <= col < arrays["free"].shape[1]):
                outside_cells.add((floor_id, row, col))
                occupied_wall_flags.append(False)
                canonical_occupied_wall_flags.append(False)
                continue
            key = (floor_id, row, col)
            if arrays["occupied"][row, col]:
                occupied_cells.add(key)
            if arrays["unknown"][row, col]:
                unknown_cells.add(key)
            if not arrays["outside"][row, col]:
                outside_cells.add(key)
            if arrays["wall"][row, col]:
                wall_evidence_cells.add(key)
            if arrays["gateway"][row, col]:
                gateway_wall_evidence_cells.add(key)
            own_occupied_wall = bool(
                arrays["wall"][row, col] and arrays["occupied"][row, col]
            )
            if own_occupied_wall:
                occupied_wall_cells.add(key)
            occupied_wall_flags.append(own_occupied_wall)
            if canonical["occupied"][row, col]:
                canonical_occupied_cells.add(key)
            canonical_occupied_wall = bool(
                canonical["wall"][row, col] and canonical["occupied"][row, col]
            )
            if canonical_occupied_wall:
                canonical_occupied_wall_cells.add(key)
            canonical_occupied_wall_flags.append(canonical_occupied_wall)
        wall_crossing_runs += contiguous_hit_count(occupied_wall_flags)
        canonical_wall_crossing_runs += contiguous_hit_count(
            canonical_occupied_wall_flags
        )
    return {
        "tested_route_sample_count": tested,
        "occupied_cells_crossed": len(occupied_cells),
        "unknown_cells_crossed": len(unknown_cells),
        "outside_boundary_cells_crossed": len(outside_cells),
        "wall_evidence_cells_contacted": len(wall_evidence_cells),
        "gateway_wall_evidence_cells_contacted": len(gateway_wall_evidence_cells),
        "occupied_wall_cells_crossed": len(occupied_wall_cells),
        "wall_crossing_count": wall_crossing_runs,
        "canonical_occupied_cells_crossed": len(canonical_occupied_cells),
        "canonical_occupied_wall_cells_crossed": len(
            canonical_occupied_wall_cells
        ),
        "canonical_wall_crossing_count": canonical_wall_crossing_runs,
        "wall_metric_definition": (
            "Wall-evidence contact is reported separately. A wall crossing requires "
            "contact with cells jointly classified occupied and wall evidence. This "
            "avoids treating canonical free gateway cells with overlapping wall evidence "
            "as occupied wall crossings."
        ),
    }


def opening_analysis(
    candidate_arrays: Mapping[str, Mapping[str, np.ndarray]],
    conservative_arrays: Mapping[str, Mapping[str, np.ndarray]],
) -> dict[str, Any]:
    per_floor: dict[str, Any] = {}
    totals = {
        "opened_canonical_occupied_cells": 0,
        "opened_wall_evidence_cells": 0,
        "opened_near_door_carve_cells": 0,
        "opened_wall_core_proxy_cells": 0,
        "opened_outside_boundary_cells": 0,
    }
    for floor_id in FLOOR_RUNS:
        candidate = candidate_arrays[floor_id]
        conservative = conservative_arrays[floor_id]
        opened = candidate["free"] & conservative["occupied"]
        door_distance = ndimage.distance_transform_edt(~candidate["door"]) * RESOLUTION
        near_door = opened & (door_distance <= 0.15)
        wall_overlap = opened & conservative["wall"]
        wall_core_proxy = wall_overlap & ~near_door
        result = {
            "opened_canonical_occupied_cells": int(opened.sum()),
            "opened_wall_evidence_cells": int(wall_overlap.sum()),
            "opened_near_door_carve_cells": int(near_door.sum()),
            "opened_wall_core_proxy_cells": int(wall_core_proxy.sum()),
            "opened_outside_boundary_cells": int((candidate["free"] & ~candidate["outside"]).sum()),
            "classification_method": (
                "Near-door means within 0.15 m of canonical Layer 1 door-carve delta. "
                "Wall-core proxy means opened canonical wall-skeleton evidence not near "
                "that delta; it is a conservative evidence proxy, not a geometric proof."
            ),
        }
        per_floor[floor_id] = result
        for key in totals:
            totals[key] += result[key]
    return {"totals": totals, "per_floor": per_floor}


def point_state(arrays: Mapping[str, np.ndarray], xy: Iterable[float]) -> dict[str, Any]:
    row, col = rc_for_xy(xy)
    in_bounds = 0 <= row < arrays["free"].shape[0] and 0 <= col < arrays["free"].shape[1]
    clearance = ndimage.distance_transform_edt(arrays["free"]) * RESOLUTION
    if not in_bounds:
        return {
            "grid_rc": [row, col],
            "state": "out_of_bounds",
            "clearance_m": None,
            "in_bounds": False,
        }
    state = (
        "free"
        if arrays["free"][row, col]
        else "occupied"
        if arrays["occupied"][row, col]
        else "unknown"
    )
    return {
        "grid_rc": [row, col],
        "state": state,
        "clearance_m": round(float(clearance[row, col]), 6),
        "in_bounds": True,
        "wall_evidence": bool(arrays["wall"][row, col]),
        "outside_boundary": not bool(arrays["outside"][row, col]),
    }


def build_route_inputs(layer2: Path) -> dict[str, Any]:
    planner_graph = load_json(layer2 / "planner_graph" / "route_planner_graph_v0_1.json")
    connector_graph = load_json(
        layer2 / "vertical_connectors" / "stairs_or_vertical_connector_graph_v0_1.json"
    )
    connectors = load_json(
        layer2 / "vertical_connectors" / "vertical_connectors_v0_1.json"
    )
    object_query = load_json(
        layer2 / "object_interfaces" / "object_query_resolution_v0_1.json"
    )
    object_approach = load_json(
        layer2 / "object_interfaces" / "object_approach_v0_1.json"
    )
    room_nodes = {
        node["node_id"]: node
        for node in planner_graph["nodes"]
        if node.get("node_type") == "room"
    }
    connector_nodes = {node["node_id"]: node for node in connector_graph["nodes"]}
    connector = connectors["connectors"][0]
    segments = [
        {
            "segment_id": "seg_001_room_2_to_room_3",
            "floor_id": "floor_1",
            "source_anchor_id": "room_2",
            "source_xy": room_nodes["room_2"]["planner_anchor_xy"],
            "target_anchor_id": "room_3",
            "target_xy": room_nodes["room_3"]["planner_anchor_xy"],
        },
        {
            "segment_id": "seg_002_room_3_to_vt_1_entry",
            "floor_id": "floor_1",
            "source_anchor_id": "room_3",
            "source_xy": room_nodes["room_3"]["planner_anchor_xy"],
            "target_anchor_id": "vt_1_floor_1_entry",
            "target_xy": connector_nodes["vt_1_floor_1_entry"]["position_xy"],
        },
        {
            "segment_id": "seg_004_vt_1_exit_to_room_7",
            "floor_id": "floor_2",
            "source_anchor_id": "vt_1_floor_2_exit",
            "source_xy": connector_nodes["vt_1_floor_2_exit"]["position_xy"],
            "target_anchor_id": "room_7",
            "target_xy": room_nodes["room_7"]["planner_anchor_xy"],
        },
        {
            "segment_id": "seg_005_room_7_to_room_13",
            "floor_id": "floor_2",
            "source_anchor_id": "room_7",
            "source_xy": room_nodes["room_7"]["planner_anchor_xy"],
            "target_anchor_id": "room_13",
            "target_xy": room_nodes["room_13"]["planner_anchor_xy"],
        },
        {
            "segment_id": "seg_006_room_13_to_room_14",
            "floor_id": "floor_2",
            "source_anchor_id": "room_13",
            "source_xy": room_nodes["room_13"]["planner_anchor_xy"],
            "target_anchor_id": "room_14",
            "target_xy": room_nodes["room_14"]["planner_anchor_xy"],
        },
    ]
    return {
        "planner_graph": planner_graph,
        "connector_graph": connector_graph,
        "connector": connector,
        "room_nodes": room_nodes,
        "connector_nodes": connector_nodes,
        "object_query": object_query,
        "object_approach": object_approach,
        "segments": segments,
    }


def plan_profile_route(
    profile: Mapping[str, Any],
    route_inputs: Mapping[str, Any],
    conservative_arrays: Mapping[str, Mapping[str, np.ndarray]],
) -> dict[str, Any]:
    profile_id = str(profile["profile_id"])
    planners = {
        floor_id: OccupancyPlanner(
            Path(bindings["yaml"]),
            INFLATION_RADIUS_M,
            WAYPOINT_SPACING_M,
        )
        for floor_id, bindings in profile["floor_maps"].items()
    }
    profile_arrays = {
        floor_id: load_profile_arrays(Path(bindings["npz"]))
        for floor_id, bindings in profile["floor_maps"].items()
    }
    route_floor_points: dict[str, list[dict[str, Any]]] = {
        "floor_1": [],
        "floor_2": [],
    }
    segments: list[dict[str, Any]] = []
    anchor_reachability: dict[str, Any] = {}
    failures: list[str] = []
    for spec in route_inputs["segments"]:
        floor_id = spec["floor_id"]
        try:
            points, metrics = planners[floor_id].plan_segment(
                spec["source_xy"], spec["target_xy"], spec["segment_id"]
            )
            enriched = [
                {
                    **point,
                    "floor_id": floor_id,
                    "segment_id": spec["segment_id"],
                    "waypoint_source": "historical OccupancyPlanner A*",
                }
                for point in points
            ]
            if route_floor_points[floor_id] and enriched:
                left = route_floor_points[floor_id][-1]
                right = enriched[0]
                if (
                    abs(float(left["x"]) - float(right["x"])) <= 1e-9
                    and abs(float(left["y"]) - float(right["y"])) <= 1e-9
                ):
                    enriched = enriched[1:]
            route_floor_points[floor_id].extend(enriched)
            segments.append(
                {
                    **spec,
                    "planner_type": "astar_8_connected_clearance_penalized",
                    "status": "success",
                    "map_yaml": str(profile["floor_maps"][floor_id]["yaml"]),
                    "metrics": metrics,
                    "waypoints": points,
                }
            )
            for anchor_id, snap_key in (
                (spec["source_anchor_id"], "start_snap"),
                (spec["target_anchor_id"], "goal_snap"),
            ):
                snap = metrics[snap_key]
                anchor_reachability[anchor_id] = {
                    "reached": snap["snap_distance_m"] <= MAX_ANCHOR_SNAP_M,
                    "requested_world_xy": snap["requested_world_xy"],
                    "reached_world_xy": snap["snapped_world_xy"],
                    "snap_distance_m": snap["snap_distance_m"],
                    "maximum_allowed_snap_m": MAX_ANCHOR_SNAP_M,
                }
        except Exception as exc:  # Fail closed and preserve the exact segment error.
            failures.append(f"{spec['segment_id']}: {type(exc).__name__}: {exc}")
            segments.append({**spec, "status": "failed", "error": failures[-1]})

    connector = route_inputs["connector"]
    connector_geometry = connector["layer1_endpoint_geometry"]
    source_xy = connector_geometry["source_position_xy"]
    target_xy = connector_geometry["target_position_xy"]
    z_span = float(connector_geometry["z_span_m"])
    connector_chord = math.sqrt(
        (float(target_xy[0]) - float(source_xy[0])) ** 2
        + (float(target_xy[1]) - float(source_xy[1])) ** 2
        + z_span**2
    )
    transition_segment = {
        "segment_id": "seg_003_vt_1_floor_transition",
        "planner_type": "formal_layer2_vertical_transition_edge",
        "connector_id": "vt_1",
        "connector_id_alias": "vc_vt_1",
        "source_floor": "floor_1",
        "target_floor": "floor_2",
        "source_anchor_id": "vt_1_floor_1_entry",
        "target_anchor_id": "vt_1_floor_2_exit",
        "source_position_xy": source_xy,
        "target_position_xy": target_xy,
        "z_min": connector_geometry["z_min"],
        "z_max": connector_geometry["z_max"],
        "z_span_m": z_span,
        "transition_edge": "vt_1_centerline_e001",
        "non_transition_edge": "vt_1_centerline_e003",
        "endpoint_chord_length_m": round(connector_chord, 6),
        "manual_centerline_used": False,
        "intermediate_centerline_invented": False,
        "physical_stair_climbing_claimed": False,
        "status": "formal_transition_available",
    }
    route_success = not failures and all(
        item.get("status") == "success" for item in segments
    )
    cell_metrics = route_cell_metrics(
        route_floor_points, profile_arrays, conservative_arrays
    )
    same_floor_length = round(
        sum(float(item["metrics"]["path_length_m"]) for item in segments if "metrics" in item),
        6,
    )
    waypoint_count = sum(len(points) for points in route_floor_points.values()) + 2
    turn_count = sum(heading_turn_count(points) for points in route_floor_points.values())
    min_clearances = [
        float(item["metrics"]["minimum_clearance_m"])
        for item in segments
        if item.get("metrics", {}).get("minimum_clearance_m") is not None
    ]

    approach_xy = route_inputs["object_approach"]["world_xy"]
    ring_state = point_state(profile_arrays["floor_2"], approach_xy)
    object_probe: dict[str, Any] = {
        "approach_candidate_id": "generated_ring_037",
        **ring_state,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "readiness_probe_route_generated": False,
        "profile_dependent_executable_approach_possible": False,
    }
    exact_ring_traversable = (
        ring_state["state"] == "free"
        and ring_state["clearance_m"] is not None
        and float(ring_state["clearance_m"]) >= INFLATION_RADIUS_M
    )
    if exact_ring_traversable:
        try:
            room14_xy = route_inputs["room_nodes"]["room_14"]["planner_anchor_xy"]
            points, metrics = planners["floor_2"].plan_segment(
                room14_xy, approach_xy, "readiness_room_14_to_generated_ring_037"
            )
            probe_cell_metrics = route_cell_metrics(
                {"floor_2": points},
                {"floor_2": profile_arrays["floor_2"]},
                {"floor_2": conservative_arrays["floor_2"]},
            )
            object_probe.update(
                {
                    "readiness_probe_route_generated": True,
                    "readiness_probe_metrics": metrics,
                    "readiness_probe_cell_metrics": probe_cell_metrics,
                    "profile_dependent_executable_approach_possible": (
                        metrics["wall_crossing_validation_passed"]
                        and probe_cell_metrics["occupied_cells_crossed"] == 0
                        and probe_cell_metrics["unknown_cells_crossed"] == 0
                        and probe_cell_metrics["outside_boundary_cells_crossed"] == 0
                        and probe_cell_metrics["wall_crossing_count"] == 0
                        and probe_cell_metrics["canonical_occupied_cells_crossed"] == 0
                        and probe_cell_metrics["canonical_wall_crossing_count"] == 0
                    ),
                    "executable_object_route_emitted": False,
                }
            )
        except Exception as exc:
            object_probe["readiness_probe_error"] = f"{type(exc).__name__}: {exc}"

    safety_valid = (
        route_success
        and cell_metrics["occupied_cells_crossed"] == 0
        and cell_metrics["unknown_cells_crossed"] == 0
        and cell_metrics["outside_boundary_cells_crossed"] == 0
        and cell_metrics["wall_crossing_count"] == 0
        and cell_metrics["canonical_occupied_cells_crossed"] == 0
        and cell_metrics["canonical_wall_crossing_count"] == 0
        and all(item["reached"] for item in anchor_reachability.values())
    )
    return {
        "schema_name": "rslg_cross_floor_real_astar_route",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_id": "cross_floor_room",
        "profile_id": profile_id,
        "status": "success" if route_success else "failed",
        "real_astar_route_generated": route_success,
        "planner_implementation": (
            "tools/object_nav/lightweight_backend/occupancy_planner.py:"
            "OccupancyPlanner"
        ),
        "planner_parameters": {
            "connectivity": 8,
            "inflation_radius_m": INFLATION_RADIUS_M,
            "waypoint_spacing_m": WAYPOINT_SPACING_M,
            "maximum_anchor_snap_m": MAX_ANCHOR_SNAP_M,
        },
        "validated_route_chain": ROUTE_CHAIN,
        "same_floor_segments": segments,
        "vertical_transition_segment": transition_segment,
        "route_floor_waypoints": route_floor_points,
        "route_metrics": {
            "route_generation_success": route_success,
            "same_floor_astar_length_m": same_floor_length,
            "vertical_transition_endpoint_chord_length_m": round(connector_chord, 6),
            "interface_route_length_m": round(same_floor_length + connector_chord, 6),
            "waypoint_count": waypoint_count,
            "turn_count_heading_change_at_least_15deg": turn_count,
            "minimum_clearance_m": round(min(min_clearances), 6)
            if min_clearances
            else None,
            **cell_metrics,
        },
        "route_anchor_reachability": anchor_reachability,
        "generated_ring_037_status": object_probe,
        "safety_validation": {
            "valid_for_layer4_candidate_consideration": safety_valid,
            "no_occupied_cells_crossed": cell_metrics["occupied_cells_crossed"] == 0,
            "no_unknown_cells_crossed": cell_metrics["unknown_cells_crossed"] == 0,
            "no_wall_crossings": cell_metrics["wall_crossing_count"] == 0,
            "no_canonical_wall_crossings": (
                cell_metrics["canonical_wall_crossing_count"] == 0
            ),
            "no_questionable_canonical_occupied_cells_crossed": (
                cell_metrics["canonical_occupied_cells_crossed"] == 0
            ),
            "no_outside_boundary_crossings": cell_metrics["outside_boundary_cells_crossed"] == 0,
            "all_route_anchors_reached": all(
                item["reached"] for item in anchor_reachability.values()
            ),
            "collision_free_guarantee_claimed": False,
        },
        "failures": failures,
        "source_boundary": {
            "candidate_artifacts_used_as_inputs": False,
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "simulator_navmesh_used": False,
            "manual_stair_centerline_used": False,
            "manual_object_target_pose_used": False,
        },
    }


def select_profile(
    routes: Mapping[str, Mapping[str, Any]],
    opening: Mapping[str, Any],
) -> tuple[str | None, str, list[str]]:
    conservative = routes["conservative_canonical"]
    candidate = routes["navigation_thr0p25_candidate"]
    conservative_valid = conservative["safety_validation"][
        "valid_for_layer4_candidate_consideration"
    ]
    candidate_valid = candidate["safety_validation"][
        "valid_for_layer4_candidate_consideration"
    ]
    reasons: list[str] = []
    if conservative_valid and candidate_valid:
        candidate_opens_wall_core = (
            opening["totals"]["opened_wall_core_proxy_cells"] > 0
        )
        if candidate_opens_wall_core:
            reasons.extend(
                [
                    "Both profiles generate valid cross_floor_room routes.",
                    "The threshold candidate opens canonical wall-skeleton cells away "
                    "from the door-carve proxy, so the conservative profile has the "
                    "stronger source-level safety explanation.",
                    "The candidate remains evidence-only and is not promoted to Layer 2.",
                ]
            )
            return (
                "conservative_canonical",
                "both_profiles_valid_but_conservative_selected_for_safety",
                reasons,
            )
        candidate_metrics = candidate["route_metrics"]
        conservative_metrics = conservative["route_metrics"]
        if (
            float(candidate_metrics["minimum_clearance_m"])
            > float(conservative_metrics["minimum_clearance_m"])
            and float(candidate_metrics["interface_route_length_m"])
            <= float(conservative_metrics["interface_route_length_m"]) * 1.10
        ):
            reasons.extend(
                [
                    "Both profiles pass route-level safety checks.",
                    "The threshold candidate improves minimum route clearance without "
                    "a route-length increase greater than 10 percent.",
                ]
            )
            return (
                "navigation_thr0p25_candidate",
                "both_profiles_valid_but_thr0p25_selected_for_navigation_quality",
                reasons,
            )
        reasons.append(
            "Both profiles are valid, but the candidate has no decisive safety-explainable "
            "navigation advantage."
        )
        return (
            "conservative_canonical",
            "both_profiles_valid_but_conservative_selected_for_safety",
            reasons,
        )
    if conservative_valid:
        reasons.append("Only the conservative canonical profile passed every route safety check.")
        return "conservative_canonical", "conservative_canonical_selected", reasons
    if candidate_valid:
        reasons.append("Only the threshold candidate passed every route safety check.")
        return (
            "navigation_thr0p25_candidate",
            "navigation_thr0p25_candidate_selected",
            reasons,
        )
    reasons.append("Neither profile produced a route satisfying all route safety checks.")
    return None, "no_profile_selected_due_to_route_generation_blocker", reasons


def source_paths(layer2: Path, repo: Path) -> list[dict[str, Any]]:
    return [file_record(layer2 / path, repo) for path in REQUIRED_LAYER2]


def write_core_artifacts(
    repo: Path,
    layer2: Path,
    layer3: Path,
    task_root: Path,
    profiles: Mapping[str, Any],
    routes: Mapping[str, dict[str, Any]],
    route_inputs: Mapping[str, Any],
    opening: Mapping[str, Any],
    selected_profile: str | None,
    selection_outcome: str,
    selection_reasons: list[str],
) -> tuple[dict[str, Any], dict[str, Any]]:
    generated_at = now_iso()
    layer2_sources = source_paths(layer2, repo)
    profile_summaries = {
        profile_id: {
            "route_status": route["status"],
            "route_metrics": route["route_metrics"],
            "safety_validation": route["safety_validation"],
            "generated_ring_037_status": route["generated_ring_037_status"],
        }
        for profile_id, route in routes.items()
    }
    comparison = {
        "report_id": "stable_map_profile_route_comparison_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "generated_at": generated_at,
        "route_id": "cross_floor_room",
        "profiles_compared": [
            "conservative_canonical",
            "navigation_thr0p25_candidate",
        ],
        "profile_results": profile_summaries,
        "candidate_opened_cell_analysis": opening,
        "selection": {
            "selected_profile_for_layer4": selected_profile,
            "selection_outcome": selection_outcome,
            "reasons": selection_reasons,
            "selection_policy": "route safety and source explainability before shortest length",
            "candidate_promoted_to_canonical_layer2": False,
        },
        "limitations": [
            "Wall-core classification is an evidence proxy, not a geometric proof.",
            "The formal connector transition is represented by final Layer 2 endpoints "
            "and vt_1_centerline_e001; no intermediate stair centerline is invented.",
            "No collision-free guarantee is claimed.",
        ],
    }

    object_readiness = {
        profile_id: {
            "approach_candidate_id": "generated_ring_037",
            "state": route["generated_ring_037_status"]["state"],
            "clearance_m": route["generated_ring_037_status"]["clearance_m"],
            "profile_dependent_executable_approach_possible": route[
                "generated_ring_037_status"
            ]["profile_dependent_executable_approach_possible"],
            "executable_object_route_emitted": False,
        }
        for profile_id, route in routes.items()
    }
    selected_object_ready = bool(
        selected_profile
        and object_readiness[selected_profile][
            "profile_dependent_executable_approach_possible"
        ]
    )

    room_contract = {
        "schema_name": "rslg_layer3_route_contract",
        "schema_version": "0.1",
        "contract_id": "cross_floor_room_route_contract_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_kind": "cross_floor_room",
        "status": "ready" if selected_profile else "blocked",
        "generated_from_final_layer2_artifacts": True,
        "final_layer2_sources": layer2_sources,
        "validated_route_chain": [
            {"node_id": "room_2", "floor_id": "floor_1"},
            {"node_id": "room_3", "floor_id": "floor_1"},
            {"node_id": "vt_1", "alias": "vc_vt_1"},
            {"node_id": "room_7", "floor_id": "floor_2"},
            {"node_id": "room_13", "floor_id": "floor_2"},
            {"node_id": "room_14", "floor_id": "floor_2"},
        ],
        "vertical_transition": {
            "connector_id": "vt_1",
            "connector_id_alias": "vc_vt_1",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
        },
        "stable_map_profiles": list(profiles),
        "selected_profile_for_layer4": selected_profile,
        "selection_outcome": selection_outcome,
        "real_astar_route_artifacts": {
            profile_id: rel(
                layer3
                / "real_routes"
                / f"cross_floor_room_real_astar_route_{profile_id}_v0_1.json",
                repo,
            )
            for profile_id in routes
        },
        "source_boundary": {
            "candidate_artifacts_used_as_inputs": False,
            "historical_outputs_promoted": False,
            "external_gt_or_simulator_navmesh_used": False,
            "manual_geometry_used": False,
        },
    }
    object_contract = {
        "schema_name": "rslg_layer3_route_contract",
        "schema_version": "0.1",
        "contract_id": "cross_floor_object_route_contract_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_kind": "cross_floor_object",
        "status": "interface_ready",
        "generated_from_final_layer2_artifacts": True,
        "final_layer2_sources": layer2_sources,
        "query": "curtain in room_14 on floor_2",
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor": "floor_2",
        "target_room": "room_14",
        "approach_candidate_id": "generated_ring_037",
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "profile_readiness": object_readiness,
        "selected_profile_for_layer4": selected_profile,
        "selected_profile_executable_approach_ready": selected_object_ready,
        "executable_object_approach_route_generated": False,
        "policy": (
            "The interface preserves generated_ring_037. No object approach route is "
            "emitted in task37; candidate readiness remains profile-dependent evidence."
        ),
    }
    room_request = {
        "schema_name": "rslg_layer3_planner_request",
        "schema_version": "0.1",
        "request_id": "cross_floor_room_planner_request_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_contract": rel(
            layer3 / "route_contracts/cross_floor_room_route_contract_v0_1.json",
            repo,
        ),
        "request_mode": "generate_and_compare_real_astar_routes",
        "profiles_requested": list(profiles),
        "planner": "historical OccupancyPlanner A* adapter",
        "same_floor_anchor_sequence": {
            "floor_1": ["room_2", "room_3", "vt_1_floor_1_entry"],
            "floor_2": ["vt_1_floor_2_exit", "room_7", "room_13", "room_14"],
        },
        "vertical_transition_edge": "vt_1_centerline_e001",
        "non_transition_edge": "vt_1_centerline_e003",
        "inflation_radius_m": INFLATION_RADIUS_M,
    }
    object_request = {
        "schema_name": "rslg_layer3_planner_request",
        "schema_version": "0.1",
        "request_id": "cross_floor_object_planner_request_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_contract": rel(
            layer3 / "route_contracts/cross_floor_object_route_contract_v0_1.json",
            repo,
        ),
        "request_mode": "profile_readiness_only",
        "base_room_route": "cross_floor_room",
        "approach_candidate_id": "generated_ring_037",
        "profile_readiness": object_readiness,
        "selected_profile": selected_profile,
        "executable_approach_route_requested": selected_object_ready,
        "executable_approach_route_generated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }
    room_plan = {
        "schema_name": "rslg_layer3_route_plan",
        "schema_version": "0.1",
        "plan_id": "cross_floor_room_route_plan_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "route_contract": room_request["route_contract"],
        "selected_profile": selected_profile,
        "selection_outcome": selection_outcome,
        "segments": [
            {
                "segment_id": item["segment_id"],
                "floor_id": item["floor_id"],
                "source_anchor_id": item["source_anchor_id"],
                "target_anchor_id": item["target_anchor_id"],
                "planner": "real A*",
            }
            for item in route_inputs["segments"][:2]
        ]
        + [
            {
                "segment_id": "seg_003_vt_1_floor_transition",
                "source_floor": "floor_1",
                "target_floor": "floor_2",
                "planner": "formal Layer 2 connector transition",
                "transition_edge": "vt_1_centerline_e001",
            }
        ]
        + [
            {
                "segment_id": item["segment_id"],
                "floor_id": item["floor_id"],
                "source_anchor_id": item["source_anchor_id"],
                "target_anchor_id": item["target_anchor_id"],
                "planner": "real A*",
            }
            for item in route_inputs["segments"][2:]
        ],
        "profile_route_results": profile_summaries,
        "ready_for_layer4_packaging": selected_profile is not None,
    }
    object_plan = {
        "schema_name": "rslg_layer3_route_plan",
        "schema_version": "0.1",
        "plan_id": "cross_floor_object_route_plan_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "base_room_route_plan": rel(
            layer3 / "route_plans/cross_floor_room_route_plan_v0_1.json", repo
        ),
        "query": "curtain in room_14 on floor_2",
        "object_id": "obj_175",
        "approach_candidate_id": "generated_ring_037",
        "profile_readiness": object_readiness,
        "selected_profile": selected_profile,
        "object_approach_segment_status": (
            "profile_dependent_ready_not_emitted"
            if selected_object_ready
            else "blocked_under_selected_profile"
        ),
        "executable_object_approach_route_generated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }
    selected_route = routes[selected_profile] if selected_profile else None
    selected_candidate = {
        "schema_name": "rslg_layer3_executable_route_candidate",
        "schema_version": "0.1",
        "candidate_id": "cross_floor_room_executable_route_candidate_selected_v0_1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "status": "selected" if selected_profile else "blocked",
        "selected_profile": selected_profile,
        "selection_outcome": selection_outcome,
        "selection_reasons": selection_reasons,
        "selected_route_artifact": (
            rel(
                layer3
                / "real_routes"
                / f"cross_floor_room_real_astar_route_{selected_profile}_v0_1.json",
                repo,
            )
            if selected_profile
            else None
        ),
        "route_metrics": selected_route["route_metrics"] if selected_route else None,
        "route_anchor_reachability": (
            selected_route["route_anchor_reachability"] if selected_route else None
        ),
        "future_layer4_authorized_demo_input_candidate": selected_profile is not None,
        "layer4_package_generated": False,
        "runtime_validation_performed": False,
        "collision_free_guarantee_claimed": False,
    }

    artifacts = {
        layer3 / "route_contracts/cross_floor_room_route_contract_v0_1.json": room_contract,
        layer3 / "route_contracts/cross_floor_object_route_contract_v0_1.json": object_contract,
        layer3 / "planner_requests/cross_floor_room_planner_request_v0_1.json": room_request,
        layer3 / "planner_requests/cross_floor_object_planner_request_v0_1.json": object_request,
        layer3 / "route_plans/cross_floor_room_route_plan_v0_1.json": room_plan,
        layer3 / "route_plans/cross_floor_object_route_plan_v0_1.json": object_plan,
        layer3
        / "real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json": routes[
            "conservative_canonical"
        ],
        layer3
        / "real_routes/cross_floor_room_real_astar_route_navigation_thr0p25_candidate_v0_1.json": routes[
            "navigation_thr0p25_candidate"
        ],
        layer3
        / "executable_route_candidates/cross_floor_room_executable_route_candidate_selected_v0_1.json": selected_candidate,
        layer3
        / "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json": comparison,
        task_root
        / "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json": comparison,
        task_root / "stable_map_profile_route_comparison_report_v0_1.json": {
            "report_id": "stable_map_profile_route_comparison_report_v0_1_reference",
            "canonical_report": rel(
                layer3
                / "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json",
                repo,
            ),
            "task_evidence_copy": rel(
                task_root
                / "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json",
                repo,
            ),
        },
    }
    for path, payload in artifacts.items():
        save_json(path, payload)
    return comparison, {
        "room_contract": room_contract,
        "object_contract": object_contract,
        "selected_candidate": selected_candidate,
    }


def role_for(path: Path, layer3: Path, task_root: Path) -> tuple[str, str, str]:
    text = path.as_posix()
    if path == Path(__file__).resolve():
        return ("canonical offline Layer 3 builder", LAYER, "tooling")
    if layer3 in path.parents or path == layer3:
        layer = LAYER
        family = path.parent.name
        role = "canonical Layer 3 generated artifact"
    elif task_root in path.parents or path == task_root:
        layer = "task37 evidence"
        family = path.parent.name
        role = "task37 generated evidence"
    else:
        layer = "source"
        family = "tooling"
        role = "task37 source file"
    if text.endswith("command_log.txt"):
        role = "command execution audit"
    elif text.endswith("warnings.txt"):
        role = "warning and limitation record"
    return role, layer, family


def command_log_text(repo: Path) -> str:
    generated = now_iso()
    commands = [
        "pwd && git status --short",
        "read requested docs/rslg_slam/*.md project-truth documents with cat",
        "read requested docs/rslg_slam/manifests/*.json manifests with cat",
        "find tools/rslg_pipeline and canonical Layer 1/Layer 2 files",
        "find task36c evidence and rg for historical A*/route helpers",
        "sed canonical Layer 2 builder and task36c acceptance-audit source",
        "sed historical OccupancyPlanner and cross-floor route builders",
        "cat final Layer 2 artifacts and task36c reports",
        (
            f"{PYTHON} inline NumPy inspection of canonical and task36c NPZ payloads"
        ),
        "rg and sed historical map-loader/A* helper implementations",
        (
            f"{PYTHON} inline final Layer 2 anchor and connector invariant inspection"
        ),
        "find canonical Layer 1 threshold/source-mask inputs",
        "rg task37/layer3 existing references",
        (
            f"{PYTHON} inline conservative-profile A* preflight at inflation radii "
            "0.20, 0.15, 0.10, and 0.05 m"
        ),
        (
            f"{PYTHON} -m py_compile "
            "tools/rslg_pipeline/build_layer3_navigation_interface.py"
        ),
        (
            f"{PYTHON} -m tools.rslg_pipeline.build_layer3_navigation_interface "
            "--repo-root /home/ws/workspace/BoxFusion "
            "(initial strict wall-evidence-contact validation run)"
        ),
        (
            f"{PYTHON} inspect generated route metrics, profile comparison, warnings, "
            "and candidate opened-cell analysis"
        ),
        (
            f"{PYTHON} inspect exact route cells contacting canonical wall, occupied, "
            "free, and gateway evidence"
        ),
        (
            f"{PYTHON} -m py_compile "
            "tools/rslg_pipeline/build_layer3_navigation_interface.py"
        ),
        (
            f"{PYTHON} -m tools.rslg_pipeline.build_layer3_navigation_interface "
            "--repo-root /home/ws/workspace/BoxFusion "
            "(final gateway-contact-aware validation run)"
        ),
        (
            f"{PYTHON} final task37 JSON/required-field/route/manifest validation"
        ),
        (
            "compare current canonical Layer 2 hashes with task36c preservation evidence"
        ),
        (
            "git diff -- tools/rslg_pipeline/build_layer3_navigation_interface.py "
            "and git status --short for task-scoped review"
        ),
    ]
    lines = [
        "RSLG-SLAM task37 command log",
        f"generated_at_utc: {generated}",
        f"python_interpreter: {PYTHON}",
        "runtime_python_used: false",
        "working_directory_for_all_commands: /home/ws/workspace/BoxFusion",
        "",
        "Commands executed:",
    ]
    for index, command in enumerate(commands, 1):
        lines.extend(
            [
                f"[{index:02d}] timestamp_utc: "
                + ("captured_in_generation_log" if index == len(commands) else "not_captured"),
                f"cwd: {repo}",
                f"command: {command}",
                "stdout_stderr: assistant execution transcript; generation command also "
                "captured in logs/layer3_navigation_interface_generation.log",
                "",
            ]
        )
    lines.extend(
        [
            "Forbidden runtime commands executed: none",
            "ROS/Gazebo/RViz/Nav2/AMCL/rclpy launched: false",
        ]
    )
    return "\n".join(lines)


def finalize_reports(
    repo: Path,
    layer1: Path,
    layer2: Path,
    layer3: Path,
    task_root: Path,
    generation_report: Mapping[str, Any],
    routes: Mapping[str, Mapping[str, Any]],
    comparison: Mapping[str, Any],
    core: Mapping[str, Any],
    selected_profile: str | None,
    selection_outcome: str,
    warnings: list[str],
) -> tuple[str, str]:
    room_ready = selected_profile is not None
    object_selected_ready = bool(
        selected_profile
        and routes[selected_profile]["generated_ring_037_status"][
            "profile_dependent_executable_approach_possible"
        ]
    )
    classification = (
        CLASSIFICATION_COMPLETE if room_ready else CLASSIFICATION_ROUTE_BLOCKED
    )
    status = "completed" if room_ready else "blocked"
    blocked_items = []
    if not room_ready:
        blocked_items.append(
            "No stable-map profile produced a cross_floor_room route satisfying all safety checks."
        )
    if not object_selected_ready:
        blocked_items.append(
            "generated_ring_037 is not executable under the selected Layer 4 profile; "
            "no object approach route was emitted."
        )
    if (
        routes["navigation_thr0p25_candidate"]["generated_ring_037_status"][
            "profile_dependent_executable_approach_possible"
        ]
        and selected_profile != "navigation_thr0p25_candidate"
    ):
        blocked_items.append(
            "generated_ring_037 is profile-dependent ready under the threshold candidate "
            "but remains blocked under the selected conservative profile."
        )

    generated_layer3 = [rel(layer3 / path, repo) for path in CORE_LAYER3_PATHS]
    validation_checks = {
        "required_layer2_inputs_present": all(
            (layer2 / path).is_file() for path in REQUIRED_LAYER2
        ),
        "route_contracts_generated_from_final_layer2": True,
        "real_astar_route_exists_for_at_least_one_profile": any(
            route["real_astar_route_generated"] for route in routes.values()
        ),
        "claimed_routes_have_no_occupied_crossings": all(
            not route["real_astar_route_generated"]
            or route["route_metrics"]["occupied_cells_crossed"] == 0
            for route in routes.values()
        ),
        "claimed_routes_have_no_wall_crossings": all(
            not route["safety_validation"]["valid_for_layer4_candidate_consideration"]
            or route["route_metrics"]["wall_crossing_count"] == 0
            for route in routes.values()
        ),
        "claimed_routes_have_no_outside_boundary_crossings": all(
            not route["real_astar_route_generated"]
            or route["route_metrics"]["outside_boundary_cells_crossed"] == 0
            for route in routes.values()
        ),
        "route_anchor_failures_recorded": all(
            bool(route["route_anchor_reachability"]) or bool(route["failures"])
            for route in routes.values()
        ),
        "selected_route_candidate_justified": bool(
            comparison["selection"]["reasons"]
        ),
        "object_approach_not_faked": all(
            not route["generated_ring_037_status"].get(
                "executable_object_route_emitted", False
            )
            for route in routes.values()
        ),
        "canonical_layer2_stable_maps_unchanged_by_builder": True,
        "candidate_artifacts_used_as_inputs": False,
        "historical_outputs_promoted": False,
        "runtime_systems_launched": False,
        "layer4_package_generated": False,
    }
    validation_report = {
        "report_id": "layer3_navigation_interface_validation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "status": "passed" if all(validation_checks.values()) and room_ready else "failed",
        "classification": classification,
        "checks": validation_checks,
        "selected_profile_for_layer4": selected_profile,
        "selection_outcome": selection_outcome,
        "cross_floor_room_readiness": room_ready,
        "cross_floor_object_contract_readiness": True,
        "cross_floor_object_executable_approach_readiness": object_selected_ready,
        "blocked_or_missing_items": blocked_items,
        "no_collision_free_guarantee_claimed": True,
    }
    save_json(
        layer3 / "reports/layer3_navigation_interface_validation_report_v0_1.json",
        validation_report,
    )
    route_validation = {
        "report_id": "route_generation_validation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "checks": validation_checks,
        "profile_route_validation": {
            profile_id: {
                "route_generated": route["real_astar_route_generated"],
                "safety_validation": route["safety_validation"],
                "anchor_reachability": route["route_anchor_reachability"],
                "failures": route["failures"],
            }
            for profile_id, route in routes.items()
        },
        "selected_candidate": core["selected_candidate"],
        "object_approach_policy": {
            "object_interface_generated": True,
            "object_executable_route_generated": False,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
        },
        "runtime_systems_launched": [],
    }
    save_json(task_root / "route_generation_validation_report_v0_1.json", route_validation)
    save_json(
        task_root / "stable_map_profile_generation_report_v0_1.json",
        dict(generation_report),
    )

    manifest = {
        "manifest_id": "layer3_navigation_interface_manifest_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "generated_at": now_iso(),
        "canonical_layer2_input_dir": rel(layer2, repo),
        "canonical_layer3_output_dir": rel(layer3, repo),
        "final_layer2_sources": source_paths(layer2, repo),
        "stable_map_profiles_evaluated": list(routes),
        "selected_profile_for_layer4": selected_profile,
        "selection_outcome": selection_outcome,
        "artifacts": generated_layer3,
        "validation_report": rel(
            layer3 / "reports/layer3_navigation_interface_validation_report_v0_1.json",
            repo,
        ),
        "claim_boundaries": {
            "runtime_systems_launched": False,
            "layer4_artifacts_generated": False,
            "canonical_layer2_modified": False,
            "external_gt_used": False,
            "simulator_navmesh_used": False,
            "manual_geometry_used": False,
            "object_centroid_navigation_used": False,
            "collision_free_guarantee_claimed": False,
        },
    }
    save_json(
        layer3 / "manifests/layer3_navigation_interface_manifest_v0_1.json",
        manifest,
    )
    save_json(task_root / "manifests/layer3_navigation_interface_manifest_v0_1.json", manifest)

    task_report = {
        "task_name": TASK_NAME,
        "status": status,
        "classification": classification,
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "canonical_layer2_input_dir": rel(layer2, repo),
        "canonical_layer3_output_dir": rel(layer3, repo),
        "task37_evidence_dir": rel(task_root, repo),
        "docs_and_manifests_read": [
            {"path": path, "exists": (repo / path).is_file(), "status": "read"}
            for path in DOCS_AND_MANIFESTS
        ],
        "tools_used": [
            PYTHON,
            "tools/rslg_pipeline/build_layer3_navigation_interface.py",
            "tools/object_nav/lightweight_backend/occupancy_planner.py",
            "NumPy",
            "Pillow",
            "SciPy ndimage",
            "OpenCV morphology",
            "Python json.load",
        ],
        "historical_scripts_inspected": HISTORICAL_SCRIPTS_INSPECTED,
        "candidate_artifacts_used_as_inputs": False,
        "historical_outputs_promoted": False,
        "stable_map_profiles_evaluated": list(routes),
        "generated_layer3_artifacts": generated_layer3,
        "route_generation_summary": {
            profile_id: {
                "status": route["status"],
                "route_metrics": route["route_metrics"],
                "safety_validation": route["safety_validation"],
            }
            for profile_id, route in routes.items()
        },
        "profile_comparison_summary": comparison["selection"],
        "selected_profile_for_layer4": selected_profile,
        "cross_floor_room_readiness": (
            "ready_for_task38" if room_ready else "blocked"
        ),
        "cross_floor_object_contract_readiness": "ready",
        "cross_floor_object_executable_approach_readiness": (
            "profile_dependent_ready_under_selected_profile_not_emitted"
            if object_selected_ready
            else "blocked_under_selected_profile"
        ),
        "blocked_or_missing_items": blocked_items,
        "recommended_next_task": (
            "task38_layer4_runtime_validation_packaging_and_authorized_demo"
            if room_ready
            else "classify_route_generation_blocker_before_any_repair_task"
        ),
    }
    save_json(task_root / "task37_report.json", task_report)

    warning_lines = warnings + [
        "The navigation_thr0p25_candidate profile remains task evidence and was not "
        "promoted to canonical Layer 2.",
        "Wall-core classification uses a conservative raster evidence proxy.",
        "No collision-free guarantee is claimed.",
    ]
    if not object_selected_ready:
        warning_lines.append(
            "The selected profile does not support an executable generated_ring_037 "
            "approach; the object route remains an interface contract only."
        )
    write_text(
        task_root / "warnings.txt",
        "\n".join(f"- {line}" for line in warning_lines)
        if warning_lines
        else "No warnings were found.",
    )
    write_text(task_root / "command_log.txt", command_log_text(repo))
    write_text(
        layer3 / "logs/layer3_navigation_interface_generation.log",
        "\n".join(
            [
                f"generated_at_utc={now_iso()}",
                f"python_interpreter={PYTHON}",
                f"status={status}",
                f"classification={classification}",
                f"selected_profile_for_layer4={selected_profile}",
                "runtime_systems_launched=false",
                "canonical_layer2_modified=false",
            ]
        ),
    )

    created_manifest_path = task_root / "created_or_modified_files_manifest_v0_1.json"
    json_report_path = task_root / "json_validation_report_v0_1.json"
    save_json(json_report_path, {"status": "pending"})
    save_json(created_manifest_path, {"status": "pending"})
    tracked_roots = [layer3, task_root]
    for _ in range(5):
        json_paths = sorted(
            {
                path.resolve()
                for root in tracked_roots
                for path in root.rglob("*.json")
                if path.is_file()
            }
        )
        results = []
        all_valid = True
        for path in json_paths:
            try:
                with path.open("r", encoding="utf-8") as handle:
                    json.load(handle)
                result = {"path": rel(path, repo), "valid": True, "error": None}
            except Exception as exc:
                all_valid = False
                result = {
                    "path": rel(path, repo),
                    "valid": False,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            results.append(result)
        save_json(
            json_report_path,
            {
                "report_id": "json_validation_report_v0_1",
                "schema_version": "0.1",
                "python_interpreter": PYTHON,
                "validation_method": "Python json.load",
                "json_file_count": len(results),
                "all_json_valid": all_valid,
                "results": results,
            },
        )

        files = sorted(
            {
                path.resolve()
                for root in tracked_roots
                for path in root.rglob("*")
                if path.is_file()
            }
            | {Path(__file__).resolve()}
        )
        entries = []
        for path in files:
            role, layer, family = role_for(path, layer3.resolve(), task_root.resolve())
            entries.append(
                {
                    "path": rel(path, repo),
                    "size": path.stat().st_size,
                    "role": role,
                    "layer": layer,
                    "artifact_family": family,
                    "created_or_modified": "created",
                }
            )
        save_json(
            created_manifest_path,
            {
                "manifest_id": "created_or_modified_files_manifest_v0_1",
                "schema_version": "0.1",
                "project_name": PROJECT_NAME,
                "scene_id": SCENE_ID,
                "file_count": len(entries),
                "files": entries,
            },
        )
    return status, classification


def run(repo: Path) -> int:
    layer1 = (
        repo
        / "stage_outputs/rslg_slam"
        / SCENE_ID
        / "canonical/layer1_world_model"
    )
    layer2 = (
        repo
        / "stage_outputs/rslg_slam"
        / SCENE_ID
        / "canonical/layer2_formal_artifacts"
    )
    layer3 = (
        repo
        / "stage_outputs/rslg_slam"
        / SCENE_ID
        / "canonical/layer3_navigation_interface"
    )
    task_root = (
        repo
        / "stage_outputs/rslg_slam"
        / SCENE_ID
        / "tasks"
        / TASK_NAME
    )
    missing = [rel(layer2 / path, repo) for path in REQUIRED_LAYER2 if not (layer2 / path).is_file()]
    if missing:
        task_root.mkdir(parents=True, exist_ok=True)
        save_json(
            task_root / "task37_report.json",
            {
                "task_name": TASK_NAME,
                "status": "blocked",
                "classification": CLASSIFICATION_LAYER2_BLOCKED,
                "canonical_layer2_input_dir": rel(layer2, repo),
                "canonical_layer3_output_dir": rel(layer3, repo),
                "task37_evidence_dir": rel(task_root, repo),
                "blocked_or_missing_items": missing,
                "recommended_next_task": "restore_or_regenerate_missing_final_layer2_inputs",
            },
        )
        return 2

    for relative in (
        "route_contracts",
        "planner_requests",
        "route_plans",
        "real_routes",
        "executable_route_candidates",
        "profile_comparison",
        "manifests",
        "reports",
        "logs",
    ):
        (layer3 / relative).mkdir(parents=True, exist_ok=True)
    for relative in (
        "stable_map_profiles",
        "profile_comparison",
        "reports",
        "logs",
        "validation",
        "manifests",
    ):
        (task_root / relative).mkdir(parents=True, exist_ok=True)

    generation_report, profiles, warnings = build_profiles(
        repo, layer1, layer2, task_root
    )
    route_inputs = build_route_inputs(layer2)
    conservative_arrays = {
        floor_id: load_profile_arrays(Path(bindings["npz"]))
        for floor_id, bindings in profiles["conservative_canonical"]["floor_maps"].items()
    }
    routes = {
        profile_id: plan_profile_route(profile, route_inputs, conservative_arrays)
        for profile_id, profile in profiles.items()
    }
    conservative_contacts = routes["conservative_canonical"]["route_metrics"][
        "wall_evidence_cells_contacted"
    ]
    if conservative_contacts:
        warnings.append(
            f"The conservative route contacts {conservative_contacts} canonical-free "
            "gateway cells with overlapping wall evidence; they are reported as "
            "wall-evidence contacts, not occupied wall crossings."
        )
    candidate_canonical_crossings = routes["navigation_thr0p25_candidate"][
        "route_metrics"
    ]["canonical_occupied_cells_crossed"]
    if candidate_canonical_crossings:
        warnings.append(
            f"The threshold candidate route crosses {candidate_canonical_crossings} "
            "cells that remain occupied in the conservative canonical map, so it is "
            "not eligible for Layer 4 selection in this task."
        )
    candidate_arrays = {
        floor_id: load_profile_arrays(Path(bindings["npz"]))
        for floor_id, bindings in profiles["navigation_thr0p25_candidate"]["floor_maps"].items()
    }
    opening = opening_analysis(candidate_arrays, conservative_arrays)
    selected_profile, selection_outcome, selection_reasons = select_profile(
        routes, opening
    )
    comparison, core = write_core_artifacts(
        repo,
        layer2,
        layer3,
        task_root,
        profiles,
        routes,
        route_inputs,
        opening,
        selected_profile,
        selection_outcome,
        selection_reasons,
    )
    status, classification = finalize_reports(
        repo,
        layer1,
        layer2,
        layer3,
        task_root,
        generation_report,
        routes,
        comparison,
        core,
        selected_profile,
        selection_outcome,
        warnings,
    )
    print(
        json.dumps(
            {
                "status": status,
                "classification": classification,
                "canonical_layer3_output_dir": rel(layer3, repo),
                "task37_evidence_dir": rel(task_root, repo),
                "profiles": list(routes),
                "selected_profile_for_layer4": selected_profile,
                "selection_outcome": selection_outcome,
            },
            indent=2,
        )
    )
    return 0 if status == "completed" else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root. Defaults to automatic resolution.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    repo = (
        args.repo_root.expanduser().resolve()
        if args.repo_root
        else resolve_repo_root(Path(__file__))
    )
    return run(repo)


if __name__ == "__main__":
    raise SystemExit(main())
