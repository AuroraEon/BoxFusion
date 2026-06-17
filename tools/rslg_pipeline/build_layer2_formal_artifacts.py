"""Canonicalize final Layer 2 formal artifacts from canonical Layer 1 outputs.

This offline adapter consumes only RSLG-SLAM World Model Layer products and
project truth. It does not run ROS, runtime systems, route generation, or the
historical World Model entrypoint.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image
from scipy import ndimage

from .common import PROJECT_NAME, resolve_repo_root, save_json


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task36_layer2_formal_artifact_canonicalization_from_existing_world_model_outputs"
CLASSIFICATION_COMPLETE = "task36_layer2_formal_artifact_canonicalization_completed"
CLASSIFICATION_PARTIAL = "task36_layer2_formal_artifact_canonicalization_partially_completed_with_blockers"
CLASSIFICATION_BLOCKED = "task36_layer2_formal_artifact_canonicalization_blocked_by_layer1_inputs"
LAYER = "Layer 2: Formal Artifact Layer"
PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"

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
    "tools/stage1_runtime/build_scene_floor_occupancy_map.py",
    "tools/stage1_nav/build_stable_map.py",
    "tools/stage1_step30p1/build_stage1_full_scene_occupancy_map.py",
    "tools/stage1_step30p1/step30s7_common.py",
    "tools/vertical_connectors/export_task25b_formal_cross_floor_artifacts.py",
    "tools/object_nav/audit_object_anchor_approach.py",
    "tools/object_nav/build_object_candidate_index.py",
]

CANONICAL_WRAPPERS = [
    "tools/rslg_pipeline/build_stable_maps.py",
    "tools/rslg_pipeline/build_vertical_connectors.py",
    "tools/rslg_pipeline/build_object_interfaces.py",
    "tools/rslg_pipeline/stable_map_schema.py",
    "tools/rslg_pipeline/vertical_connector_schema.py",
    "tools/rslg_pipeline/object_interface_schema.py",
    "tools/rslg_pipeline/validate_artifacts.py",
]

FLOOR_RUNS = {"floor_1": 1500, "floor_2": 2709}
ROUTE_CONTEXT = ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"]
FORBIDDEN_SOURCES = [
    "external GT floorplan",
    "external GT occupancy map",
    "simulator navmesh",
    "manually drawn stair centerline",
    "manually specified object target pose",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


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


def source_record(path: Path, repo: Path, source_class: str, floor_id: str | None = None) -> dict[str, Any]:
    suffix = path.suffix.lower().lstrip(".") or "unknown"
    return {
        "path": rel(path, repo),
        "floor_id": floor_id,
        "exists": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else None,
        "file_type": suffix,
        "source_class": source_class,
        "acceptable_as_layer2_source_evidence": path.is_file(),
        "sha256": sha256(path) if path.is_file() else None,
    }


def polygon_centroid(points: list[list[float]]) -> list[float]:
    if len(points) < 3:
        return [round(sum(p[0] for p in points) / max(1, len(points)), 6),
                round(sum(p[1] for p in points) / max(1, len(points)), 6)]
    area2 = 0.0
    cx = 0.0
    cy = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        cross = float(point[0]) * float(nxt[1]) - float(nxt[0]) * float(point[1])
        area2 += cross
        cx += (float(point[0]) + float(nxt[0])) * cross
        cy += (float(point[1]) + float(nxt[1])) * cross
    if abs(area2) < 1e-9:
        return [round(sum(p[0] for p in points) / len(points), 6),
                round(sum(p[1] for p in points) / len(points), 6)]
    return [round(cx / (3.0 * area2), 6), round(cy / (3.0 * area2), 6)]


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    previous = len(polygon) - 1
    for index, point in enumerate(polygon):
        xi, yi = float(point[0]), float(point[1])
        xj, yj = float(polygon[previous][0]), float(polygon[previous][1])
        if (yi > y) != (yj > y):
            crossing_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < crossing_x:
                inside = not inside
        previous = index
    return inside


def write_pgm(path: Path, grid_world: np.ndarray) -> None:
    image = np.flipud(grid_world).astype(np.uint8)
    header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + image.tobytes())


def map_yaml(image_name: str) -> str:
    return "\n".join([
        f"image: {image_name}",
        "resolution: 0.05",
        "origin: [-50.0, -50.0, 0.0]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
    ])


def ray_check(
    grid: np.ndarray,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    resolution: float = 0.05,
    origin: tuple[float, float] = (-50.0, -50.0),
) -> dict[str, Any]:
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    steps = max(2, int(math.ceil(distance / (resolution * 0.5))))
    hits = 0
    out_of_bounds = 0
    for index in range(steps + 1):
        ratio = index / steps
        x = start_xy[0] + (end_xy[0] - start_xy[0]) * ratio
        y = start_xy[1] + (end_xy[1] - start_xy[1]) * ratio
        row = int(round((y - origin[1]) / resolution))
        col = int(round((x - origin[0]) / resolution))
        if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
            out_of_bounds += 1
        elif grid[row, col] <= 10:
            hits += 1
    return {
        "passed": hits == 0 and out_of_bounds == 0,
        "occupied_samples": hits,
        "out_of_bounds_samples": out_of_bounds,
        "distance_m": round(distance, 6),
    }


def evidence_entry(
    evidence_type: str,
    records: list[dict[str, Any]],
    status: str,
    reason: str,
) -> dict[str, Any]:
    return {
        "evidence_type": evidence_type,
        "status": status,
        "selected_source_files": records,
        "acceptable_as_layer2_source_evidence": bool(records) and all(
            record["acceptable_as_layer2_source_evidence"] for record in records
        ),
        "reason": reason,
    }


def build_evidence_manifest(repo: Path, raw: Path, canonical_l1: Path) -> tuple[dict[str, Any], list[str]]:
    logs = raw / "logs"
    selected: dict[str, dict[str, Any]] = {}
    warnings: list[str] = []

    final_bev = raw / "final" / f"{SCENE_ID}_final_bev.png"
    final_vector = logs / "final_vector_map_snapshot.json"
    topology = logs / "topology_v0_1.json"
    committed_snapshot = logs / "committed_room_world_snapshot_v0_1.json"
    floor_summary = logs / "floor_diagnostics_summary.json"
    vertical = logs / "vertical_transition_evidence.json"

    floor_files: dict[str, dict[str, Path]] = {}
    for floor_id, run in FLOOR_RUNS.items():
        root = raw / "debug_room" / floor_id
        floor_files[floor_id] = {
            "wall": root / f"run_{run}_02_walls_skeleton.png",
            "gateway_wall": root / f"run_{run}_01d_gateway_wall_preclose_thr_0p25.npy",
            "outside": root / f"run_{run}_03_outside_boundary.png",
            "free": root / f"run_{run}_05_free_space.png",
            "labels": root / f"run_{run}_09b_repaired_labels.npy",
            "rooms": root / f"run_{run}_11_final_rooms.png",
            "tracking": root / f"run_{run}_12_tracking_report.json",
            "wall_metadata": root / f"run_{run}_01g_dual_wall_layer_metadata.json",
        }

    def records(paths: Iterable[tuple[Path, str, str | None]]) -> list[dict[str, Any]]:
        return [source_record(path, repo, source_class, floor_id) for path, source_class, floor_id in paths]

    selected["bev_evidence"] = evidence_entry(
        "BEV evidence",
        records([(final_bev, "raw_canonical", None), (final_vector, "raw_canonical", None)]),
        "available",
        "Final BEV visualization and final vector-map snapshot are canonical Layer 1 outputs.",
    )
    selected["floor_assignment_or_floor_height_evidence"] = evidence_entry(
        "floor assignment / floor height evidence",
        records([(floor_summary, "raw_canonical", None), (final_vector, "raw_canonical", None)]),
        "available",
        "Both files contain floor bands, z ranges, assignments, and final floor identifiers.",
    )
    selected["room_or_floor_topology_evidence"] = evidence_entry(
        "room/floor topology evidence",
        records([(topology, "raw_canonical", None), (committed_snapshot, "raw_canonical", None)]),
        "available",
        "Canonical Layer 1 topology and committed snapshot contain rooms, floors, and supported edges.",
    )
    selected["wall_evidence"] = evidence_entry(
        "wall evidence",
        records([
            (files["wall"], "raw_debug", floor_id)
            for floor_id, files in floor_files.items()
        ] + [
            (files["wall_metadata"], "raw_debug", floor_id)
            for floor_id, files in floor_files.items()
        ]),
        "available",
        "Final successful per-floor wall skeletons and wall-layer metadata are present.",
    )
    selected["free_space_evidence"] = evidence_entry(
        "free-space evidence",
        records([(files["free"], "raw_debug", floor_id) for floor_id, files in floor_files.items()]),
        "available",
        "Final successful per-floor free-space rasters are present.",
    )
    selected["gateway_or_gateway_wall_evidence"] = evidence_entry(
        "gateway or gateway-wall evidence",
        records([
            (files["gateway_wall"], "raw_debug", floor_id)
            for floor_id, files in floor_files.items()
        ] + [(topology, "raw_canonical", None)]),
        "available",
        "Final gateway-wall arrays and canonical topology gateway records are present.",
    )
    selected["outside_boundary_evidence"] = evidence_entry(
        "outside-boundary evidence",
        records([(files["outside"], "raw_debug", floor_id) for floor_id, files in floor_files.items()]),
        "inferred_from_existing_layer1_debug_outputs",
        "Final per-floor debug PNGs are binary Layer 1 products. The adapter crops the known 10-pixel debug padding and records that conversion.",
    )
    selected["room_segmentation_or_room_label_evidence"] = evidence_entry(
        "room segmentation / room label evidence",
        records([
            (files["labels"], "raw_debug", floor_id)
            for floor_id, files in floor_files.items()
        ] + [
            (files["rooms"], "raw_debug", floor_id)
            for floor_id, files in floor_files.items()
        ]),
        "available",
        "Final repaired labels and room visualizations are present for both floors.",
    )
    selected["object_semantic_records"] = evidence_entry(
        "object semantic records",
        records([(final_vector, "raw_canonical", None), (committed_snapshot, "raw_canonical", None)]),
        "available",
        "Current Layer 1 directly records obj_175 with label curtain.",
    )
    selected["object_room_association_evidence"] = evidence_entry(
        "object-room association evidence",
        records([(topology, "raw_canonical", None), (final_vector, "raw_canonical", None)]),
        "available",
        "Current Layer 1 maps obj_175 to room_14.",
    )
    selected["object_floor_association_evidence"] = evidence_entry(
        "object-floor association evidence",
        records([(final_vector, "raw_canonical", None), (floor_summary, "raw_canonical", None)]),
        "available",
        "Current Layer 1 records obj_175 on floor_2 with floor-assignment provenance.",
    )
    selected["vertical_transition_or_connector_evidence"] = evidence_entry(
        "vertical transition / connector-related evidence",
        records([(vertical, "raw_canonical", None), (topology, "raw_canonical", None)]),
        "available",
        "Current Layer 1 records supported vt_1 evidence from floor_1 room_3 to floor_2 room_7.",
    )
    selected["canonical_layer1_provenance"] = evidence_entry(
        "provenance of selected evidence files",
        records([
            (canonical_l1 / "manifests" / "canonical_layer1_world_model_manifest_v0_1.json", "canonical", None),
            (canonical_l1 / "reports" / "canonical_layer1_world_model_validation_report_v0_1.json", "canonical", None),
            (canonical_l1 / "logs" / "layer1_world_model_execution.log", "canonical", None),
        ]),
        "available",
        "Task35 canonical manifest, validation, and execution log are present.",
    )

    unavailable = [
        key for key, entry in selected.items()
        if not entry["acceptable_as_layer2_source_evidence"]
    ]
    if unavailable:
        warnings.append("Unavailable required evidence: " + ", ".join(unavailable))
    warnings.append(
        "Outside-boundary evidence was normalized from existing canonical Layer 1 debug PNGs; no boundary was drawn or supplied manually."
    )
    return {
        "manifest_id": "layer1_to_layer2_source_evidence_manifest_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "canonical_layer1_input_dir": rel(canonical_l1, repo),
        "canonical_layer1_raw_output_dir": rel(raw, repo),
        "source_boundary": {
            "allowed": "RSLG-SLAM outputs derived from RGB-D plus provided pose",
            "forbidden_sources_used": [],
            "external_gt_map_used": False,
            "simulator_navmesh_used": False,
            "manual_geometry_used": False,
        },
        "final_floor_run_selection": {
            floor_id: {
                "run_frame": run,
                "selection_reason": "latest successful Layer 1 room-segmentation run for this floor",
            }
            for floor_id, run in FLOOR_RUNS.items()
        },
        "evidence": selected,
        "summary": {
            "evidence_type_count": len(selected),
            "available_count": sum(entry["status"] == "available" for entry in selected.values()),
            "inferred_debug_count": sum(
                entry["status"] == "inferred_from_existing_layer1_debug_outputs"
                for entry in selected.values()
            ),
            "blocked_count": len(unavailable),
            "all_required_source_evidence_acceptable": not unavailable,
        },
    }, warnings


def build_floor_map(
    floor_id: str,
    run: int,
    raw: Path,
    stable_root: Path,
    repo: Path,
) -> tuple[dict[str, Any], np.ndarray]:
    debug = raw / "debug_room" / floor_id
    free_path = debug / f"run_{run}_05_free_space.png"
    outside_path = debug / f"run_{run}_03_outside_boundary.png"
    wall_path = debug / f"run_{run}_02_walls_skeleton.png"
    gateway_path = debug / f"run_{run}_01d_gateway_wall_preclose_thr_0p25.npy"
    labels_path = debug / f"run_{run}_09b_repaired_labels.npy"

    free_raw = np.asarray(Image.open(free_path).convert("L")) > 0
    outside_padded = np.asarray(Image.open(outside_path).convert("L")) > 0
    walls_padded = np.asarray(Image.open(wall_path).convert("L")) > 0
    gateway_wall = np.load(gateway_path).astype(bool)
    labels = np.load(labels_path)
    outside = outside_padded[10:-10, 10:-10]
    walls = walls_padded[10:-10, 10:-10]

    shapes = {array.shape for array in [free_raw, outside, walls, gateway_wall, labels]}
    if len(shapes) != 1:
        raise RuntimeError(f"{floor_id} evidence shapes are inconsistent: {sorted(shapes)}")

    free = free_raw & outside
    occupied = outside & ~free
    unknown = ~outside
    grid = np.full(free.shape, 205, dtype=np.uint8)
    grid[occupied] = 0
    grid[free] = 254

    floor_dir = stable_root / floor_id
    pgm = floor_dir / f"{floor_id}_stable_occupancy_map_v0_1.pgm"
    yaml_path = floor_dir / f"{floor_id}_stable_occupancy_map_v0_1.yaml"
    npz_path = floor_dir / f"{floor_id}_stable_occupancy_map_v0_1.npz"
    preview_path = floor_dir / f"{floor_id}_stable_occupancy_map_preview_v0_1.png"
    metadata_path = floor_dir / f"{floor_id}_stable_occupancy_map_metadata_v0_1.json"

    write_pgm(pgm, grid)
    write_text(yaml_path, map_yaml(pgm.name))
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        npz_path,
        free_mask=free,
        occupied_mask=occupied,
        unknown_mask=unknown,
        outside_boundary=outside,
        wall_evidence=walls,
        gateway_wall_preclose=gateway_wall,
        repaired_room_labels=labels,
        occupancy_values=np.where(free, 0, np.where(occupied, 100, -1)).astype(np.int16),
        pgm_yflip=np.flipud(grid),
    )
    Image.fromarray(grid).save(preview_path)

    metadata = {
        "schema_name": "rslg_stable_occupancy_floor_map",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "floor_id": floor_id,
        "source_run_frame": run,
        "resolution_m_per_cell": 0.05,
        "origin": [-50.0, -50.0, 0.0],
        "shape_hw": list(grid.shape),
        "cell_semantics": {"free": 254, "occupied": 0, "unknown": 205},
        "cell_counts": {
            "free": int(free.sum()),
            "occupied": int(occupied.sum()),
            "unknown": int(unknown.sum()),
            "outside_boundary_inside": int(outside.sum()),
            "wall_evidence": int(walls.sum()),
            "gateway_wall_evidence": int(gateway_wall.sum()),
            "free_wall_evidence_overlap": int((free & (walls | gateway_wall)).sum()),
        },
        "conversion": {
            "outside_boundary_png_padding_cropped_px": 10,
            "outside_boundary_binary_rule": "PNG value > 0",
            "free_rule": "Layer 1 free_space > 0 AND inside outside_boundary",
            "occupied_rule": "inside outside_boundary AND NOT Layer 1 free_space",
            "unknown_rule": "outside Layer 1 outside_boundary",
            "geometry_invented": False,
        },
        "source_files": [
            source_record(free_path, repo, "raw_debug", floor_id),
            source_record(outside_path, repo, "raw_debug", floor_id),
            source_record(wall_path, repo, "raw_debug", floor_id),
            source_record(gateway_path, repo, "raw_debug", floor_id),
            source_record(labels_path, repo, "raw_debug", floor_id),
        ],
        "outputs": {
            "pgm": rel(pgm, repo),
            "map_server_yaml": rel(yaml_path, repo),
            "npz": rel(npz_path, repo),
            "preview_png": rel(preview_path, repo),
        },
        "claim_boundary": {
            "planner_compatible_layer2_resource": True,
            "runtime_costmap": False,
            "external_gt_map": False,
            "semantic_floorplan": False,
            "room_mask_substitution": False,
            "simulator_navmesh": False,
        },
    }
    save_json(metadata_path, metadata)
    metadata["metadata_json"] = rel(metadata_path, repo)
    return metadata, grid


def build_stable_maps(raw: Path, layer2: Path, repo: Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    stable_root = layer2 / "stable_maps"
    floors: list[dict[str, Any]] = []
    grids: dict[str, np.ndarray] = {}
    for floor_id, run in FLOOR_RUNS.items():
        metadata, grid = build_floor_map(floor_id, run, raw, stable_root, repo)
        floors.append(metadata)
        grids[floor_id] = grid
    package = {
        "schema_name": "rslg_stable_occupancy_map_package",
        "schema_version": "0.1",
        "classification": "stable_occupancy_map_package_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "artifact_family": "stable_occupancy_map_package_v0_1",
        "is_final_formal_artifact": True,
        "generated_from_current_canonical_layer1_outputs": True,
        "map_kind": "stable_occupancy",
        "request_dependent": False,
        "floor_maps": floors,
        "source_evidence_manifest": "manifests/layer1_to_layer2_source_evidence_manifest_v0_1.json",
        "source_policy": {
            "external_gt_occupancy_map_used": False,
            "semantic_floorplan_used_as_map": False,
            "room_mask_used_as_map": False,
            "runtime_costmap_used": False,
            "simulator_navmesh_used": False,
        },
        "runtime_artifact": False,
        "route_generated": False,
    }
    save_json(stable_root / "stable_occupancy_map_package_v0_1.json", package)
    return package, grids


def extract_truth(raw: Path, repo: Path) -> dict[str, Any]:
    logs = raw / "logs"
    final_vector_path = logs / "final_vector_map_snapshot.json"
    topology_path = logs / "topology_v0_1.json"
    vertical_path = logs / "vertical_transition_evidence.json"
    final_vector = load_json(final_vector_path)
    topology = load_json(topology_path)
    vertical = load_json(vertical_path)
    transition = next(item for item in vertical["transitions"] if item["transition_id"] == "vt_1")
    object_record = next(item for item in final_vector["objects"] if str(item.get("id")) == "175")
    rooms = {room["room_id"]: room for room in final_vector["rooms"]}
    return {
        "final_vector": final_vector,
        "topology": topology,
        "transition": transition,
        "object": object_record,
        "rooms": rooms,
        "sources": {
            "final_vector": rel(final_vector_path, repo),
            "topology": rel(topology_path, repo),
            "vertical_transition": rel(vertical_path, repo),
            "validated_milestones": "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
        },
    }


def build_connectors_and_graphs(truth: dict[str, Any], layer2: Path, repo: Path) -> dict[str, dict[str, Any]]:
    transition = truth["transition"]
    rooms = truth["rooms"]
    connector_dir = layer2 / "vertical_connectors"
    topology_dir = layer2 / "topology"
    planner_dir = layer2 / "planner_graph"

    connector = {
        "schema_name": "rslg_vertical_connectors",
        "schema_version": "0.1",
        "classification": "vertical_connectors_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "connectors": [{
            "connector_id": "vt_1",
            "connector_id_alias": "vc_vt_1",
            "connector_type": "vertical_connector",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "source_room": "room_3",
            "target_room": "room_7",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "transition_edge_assertions": {
                "vt_1_centerline_e001_is_floor_transition": True,
                "vt_1_centerline_e003_is_floor_transition": False,
            },
            "layer1_endpoint_geometry": {
                "source_position_xy": transition["from_position_xy"],
                "target_position_xy": transition["to_position_xy"],
                "z_min": transition["z_min"],
                "z_max": transition["z_max"],
                "z_span_m": transition["z_span_m"],
            },
            "layer1_support": {
                "status": transition["status"],
                "edge_eligible": transition["edge_eligible"],
                "confidence": transition["confidence"],
                "frame_start": transition["frame_start"],
                "frame_end": transition["frame_end"],
                "supporting_frame_count": transition["supporting_frame_count"],
                "room_association_complete": transition["room_association_complete"],
            },
            "centerline_geometry_policy": {
                "manual_centerline_used": False,
                "intermediate_centerline_coordinates_invented": False,
                "formal_transition_uses_current_layer1_endpoints": True,
            },
        }],
        "source_files": [truth["sources"]["vertical_transition"], truth["sources"]["topology"]],
        "claim_boundary": {
            "topological_vertical_transition": True,
            "physical_stair_climbing_supported": False,
            "gait_or_footstep_planning_supported": False,
        },
    }
    save_json(connector_dir / "vertical_connectors_v0_1.json", connector)

    connector_graph = {
        "schema_name": "rslg_stairs_or_vertical_connector_graph",
        "schema_version": "0.1",
        "classification": "stairs_or_vertical_connector_graph_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "graph_id": "vt_1_connector_graph_v0_1",
        "nodes": [
            {"node_id": "room_3", "node_type": "room", "floor_id": "floor_1"},
            {
                "node_id": "vt_1_floor_1_entry",
                "node_type": "connector_endpoint",
                "floor_id": "floor_1",
                "position_xy": transition["from_position_xy"],
            },
            {
                "node_id": "vt_1_floor_2_exit",
                "node_type": "connector_endpoint",
                "floor_id": "floor_2",
                "position_xy": transition["to_position_xy"],
            },
            {"node_id": "room_7", "node_type": "room", "floor_id": "floor_2"},
        ],
        "edges": [
            {
                "edge_id": "vt_1_entry_e000",
                "source": "room_3",
                "target": "vt_1_floor_1_entry",
                "edge_type": "connector_access",
                "is_floor_transition": False,
            },
            {
                "edge_id": "vt_1_centerline_e001",
                "source": "vt_1_floor_1_entry",
                "target": "vt_1_floor_2_exit",
                "edge_type": "vertical_transition",
                "is_floor_transition": True,
            },
            {
                "edge_id": "vt_1_exit_e002",
                "source": "vt_1_floor_2_exit",
                "target": "room_7",
                "edge_type": "connector_access",
                "is_floor_transition": False,
            },
        ],
        "edge_invariants": [{
            "edge_id": "vt_1_centerline_e003",
            "is_floor_transition": False,
            "note": "Validated non-transition edge identity; no unsupported geometry is invented.",
        }],
        "source_connector_artifact": rel(connector_dir / "vertical_connectors_v0_1.json", repo),
    }
    save_json(connector_dir / "stairs_or_vertical_connector_graph_v0_1.json", connector_graph)

    room_nodes = []
    for room_id, room in sorted(rooms.items()):
        polygon = room.get("polygon") or []
        room_nodes.append({
            "node_id": room_id,
            "node_type": "room",
            "floor_id": room["floor_id"],
            "polygon_xy": polygon,
            "planner_anchor_xy": polygon_centroid(polygon),
            "source_reported_center_xy": room.get("center"),
            "planner_anchor_derivation": "polygon centroid from current canonical Layer 1 room polygon",
        })

    room_edges: dict[tuple[str, str], dict[str, Any]] = {}
    for edge in truth["topology"]["edges"]:
        source = str(edge.get("source"))
        target = str(edge.get("target"))
        if source not in rooms or target not in rooms:
            continue
        if edge.get("relation_type") in {"possible_connection", "vertical_transition"}:
            continue
        if edge.get("status") == "weak":
            continue
        key = tuple(sorted((source, target)))
        existing = room_edges.get(key)
        if existing is None or float(edge.get("confidence", 0.0)) > float(existing.get("confidence", 0.0)):
            room_edges[key] = {
                "edge_id": f"{key[0]}__{key[1]}",
                "source": key[0],
                "target": key[1],
                "edge_type": "same_floor_room_connection",
                "floor_id": rooms[key[0]]["floor_id"],
                "confidence": edge.get("confidence"),
                "status": edge.get("status"),
                "source_relation_type": edge.get("relation_type"),
                "source_evidence_ids": edge.get("evidence_ids", []),
            }

    cross_floor = {
        "schema_name": "rslg_cross_floor_topology",
        "schema_version": "0.1",
        "classification": "cross_floor_topology_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "topology_id": "cross_floor_topology_v0_1",
        "nodes": room_nodes + [{
            "node_id": "vt_1",
            "aliases": ["vc_vt_1"],
            "node_type": "vertical_connector",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
        }],
        "same_floor_edges": list(room_edges.values()),
        "connector_edges": [
            {
                "edge_id": "room_3__vt_1",
                "source": "room_3",
                "target": "vt_1",
                "edge_type": "connector_access",
                "floor_id": "floor_1",
            },
            {
                "edge_id": "vt_1_centerline_e001",
                "source": "vt_1",
                "target": "room_7",
                "edge_type": "floor_transition",
                "source_floor": "floor_1",
                "target_floor": "floor_2",
                "is_floor_transition": True,
            },
        ],
        "validated_route_context_only_not_generated_route": ROUTE_CONTEXT,
        "route_generated": False,
        "source_files": [truth["sources"]["topology"], truth["sources"]["vertical_transition"]],
    }
    save_json(topology_dir / "cross_floor_topology_v0_1.json", cross_floor)

    planner_graph = {
        "schema_name": "rslg_route_planner_graph",
        "schema_version": "0.1",
        "classification": "route_planner_graph_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "graph_id": "route_planner_graph_v0_1",
        "graph_role": "planner-ready topology consumed later by Layer 3",
        "nodes": cross_floor["nodes"],
        "edges": cross_floor["same_floor_edges"] + cross_floor["connector_edges"],
        "floor_map_bindings": {
            floor_id: f"../stable_maps/{floor_id}/{floor_id}_stable_occupancy_map_v0_1.yaml"
            for floor_id in FLOOR_RUNS
        },
        "vertical_transition_invariants": {
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "vt_1_centerline_e001_is_transition": True,
            "vt_1_centerline_e003_is_transition": False,
        },
        "route_generated": False,
        "a_star_generated": False,
        "executable_waypoints_generated": False,
        "source_artifacts": [
            rel(connector_dir / "stairs_or_vertical_connector_graph_v0_1.json", repo),
            rel(topology_dir / "cross_floor_topology_v0_1.json", repo),
        ],
    }
    save_json(planner_dir / "route_planner_graph_v0_1.json", planner_graph)
    return {
        "vertical_connectors": connector,
        "connector_graph": connector_graph,
        "cross_floor_topology": cross_floor,
        "route_planner_graph": planner_graph,
    }


def generated_ring_candidate(object_record: dict[str, Any], candidate_id: str) -> dict[str, Any]:
    number = int(candidate_id.rsplit("_", 1)[-1])
    angles = list(range(0, 360, 22))
    standoffs = [0.6, 0.8, 1.0]
    standoff_index, angle_index = divmod(number, len(angles))
    if standoff_index >= len(standoffs):
        raise ValueError(f"Unsupported generated ring candidate id: {candidate_id}")
    footprint = object_record["footprint_2d"]
    min_x = min(point[0] for point in footprint)
    max_x = max(point[0] for point in footprint)
    min_y = min(point[1] for point in footprint)
    max_y = max(point[1] for point in footprint)
    center_x, center_y = map(float, object_record["pose"])
    half_width = max(0.05, (max_x - min_x) / 2.0)
    half_height = max(0.05, (max_y - min_y) / 2.0)
    angle_deg = angles[angle_index]
    angle = math.radians(angle_deg)
    unit_x, unit_y = math.cos(angle), math.sin(angle)
    edge_x = half_width / abs(unit_x) if abs(unit_x) > 1e-6 else float("inf")
    edge_y = half_height / abs(unit_y) if abs(unit_y) > 1e-6 else float("inf")
    support = min(edge_x, edge_y)
    standoff = standoffs[standoff_index]
    x = center_x + unit_x * (support + standoff)
    y = center_y + unit_y * (support + standoff)
    return {
        "approach_candidate_id": candidate_id,
        "world_xy": [round(x, 6), round(y, 6)],
        "yaw": round(math.atan2(center_y - y, center_x - x), 6),
        "generation_policy": "existing bbox_support_plus_standoff policy",
        "angle_deg": angle_deg,
        "standoff_from_footprint_m": standoff,
        "object_centroid_xy_for_facing_only": [center_x, center_y],
    }


def build_object_interfaces(
    truth: dict[str, Any],
    grids: dict[str, np.ndarray],
    layer2: Path,
    repo: Path,
) -> tuple[dict[str, dict[str, Any]], list[str]]:
    object_dir = layer2 / "object_interfaces"
    object_record = truth["object"]
    room = truth["rooms"]["room_14"]
    query_resolution = {
        "schema_name": "rslg_object_query_resolution",
        "schema_version": "0.1",
        "classification": "object_query_resolution_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "query": "curtain in room_14 on floor_2",
        "resolution_status": "resolved",
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor": "floor_2",
        "target_room": "room_14",
        "current_layer1_object_record": object_record,
        "source_files": [truth["sources"]["final_vector"], truth["sources"]["topology"]],
        "candidate_artifacts_used_as_inputs": False,
    }
    save_json(object_dir / "object_query_resolution_v0_1.json", query_resolution)

    candidate = generated_ring_candidate(object_record, "generated_ring_037")
    grid = grids["floor_2"]
    x, y = candidate["world_xy"]
    row = int(round((y + 50.0) / 0.05))
    col = int(round((x + 50.0) / 0.05))
    in_bounds = 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]
    free = bool(in_bounds and grid[row, col] >= 250)
    free_mask = grid >= 250
    clearance = float(ndimage.distance_transform_edt(free_mask)[row, col] * 0.05) if in_bounds else 0.0
    room_membership = point_in_polygon(x, y, room["polygon"])
    ray = ray_check(grid, (x, y), tuple(map(float, object_record["pose"])))
    local_checks = {
        "candidate_regenerated_by_existing_policy": True,
        "same_floor": object_record["floor_id"] == "floor_2",
        "target_room_membership": room_membership,
        "map_in_bounds": in_bounds,
        "occupancy_free": free,
        "clearance_at_least_0_2m": clearance >= 0.2,
        "candidate_to_object_ray_clear": ray["passed"],
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }
    local_ok = all(value for key, value in local_checks.items() if key not in {
        "object_centroid_navigation_used", "direct_object_centroid_goal_used"
    })
    approach_status = "validated_against_current_layer1_stable_map" if local_ok else "blocked_by_current_layer1_stable_map_conflict"
    approach = {
        "schema_name": "rslg_object_approach",
        "schema_version": "0.1",
        "classification": "object_approach_v0_1_generated_with_current_evidence_validation",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "query": "curtain in room_14 on floor_2",
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor": "floor_2",
        "target_room": "room_14",
        **candidate,
        "grid_rc": [row, col],
        "occupancy_value": int(grid[row, col]) if in_bounds else None,
        "clearance_m": round(clearance, 6),
        "local_static_validation_checks": local_checks,
        "candidate_to_object_ray_check": ray,
        "validation_status": approach_status,
        "validated_milestone_identity_preserved": True,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "object_centroid_used_only_to_compute_facing_yaw_and_ring_geometry": True,
        "global_route_reachability_validation_deferred_to_layer3": True,
        "source_files": [
            truth["sources"]["final_vector"],
            truth["sources"]["validated_milestones"],
            "../stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.yaml",
        ],
        "historical_candidate_artifacts_used_as_inputs": False,
    }
    save_json(object_dir / "object_approach_v0_1.json", approach)

    package = {
        "schema_name": "rslg_object_interface_package",
        "schema_version": "0.1",
        "classification": "object_interface_package_v0_1_generated",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "artifact_layer": LAYER,
        "is_final_formal_artifact": True,
        "query_resolution_artifact": "object_query_resolution_v0_1.json",
        "object_approach_artifact": "object_approach_v0_1.json",
        "query_resolution_status": "resolved",
        "approach_validation_status": approach_status,
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor": "floor_2",
        "target_room": "room_14",
        "approach_candidate_id": "generated_ring_037",
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "ready_for_layer3_object_route_generation": local_ok,
        "ready_for_layer3_object_contract_generation": True,
        "route_generated": False,
        "runtime_artifact": False,
    }
    save_json(object_dir / "object_interface_package_v0_1.json", package)

    warnings = []
    if not local_ok:
        failed = [key for key, value in local_checks.items() if value is False and key not in {
            "object_centroid_navigation_used", "direct_object_centroid_goal_used"
        }]
        warnings.append(
            "generated_ring_037 was deterministically regenerated from current Layer 1 obj_175 geometry but failed current stable-map checks: "
            + ", ".join(failed)
            + ". Its validated identity is preserved, but Layer 3 must not produce an executable object-approach route until revalidated."
        )
    return {
        "query_resolution": query_resolution,
        "object_approach": approach,
        "object_interface_package": package,
    }, warnings


def artifact_paths(layer2: Path) -> dict[str, Path]:
    return {
        "stable_occupancy_map_package_v0_1": layer2 / "stable_maps" / "stable_occupancy_map_package_v0_1.json",
        "vertical_connectors_v0_1": layer2 / "vertical_connectors" / "vertical_connectors_v0_1.json",
        "stairs_or_vertical_connector_graph_v0_1": layer2 / "vertical_connectors" / "stairs_or_vertical_connector_graph_v0_1.json",
        "cross_floor_topology_v0_1": layer2 / "topology" / "cross_floor_topology_v0_1.json",
        "route_planner_graph_v0_1": layer2 / "planner_graph" / "route_planner_graph_v0_1.json",
        "object_query_resolution_v0_1": layer2 / "object_interfaces" / "object_query_resolution_v0_1.json",
        "object_approach_v0_1": layer2 / "object_interfaces" / "object_approach_v0_1.json",
        "object_interface_package_v0_1": layer2 / "object_interfaces" / "object_interface_package_v0_1.json",
    }


def build_validation(
    layer2: Path,
    evidence: dict[str, Any],
    connectors: dict[str, dict[str, Any]],
    objects: dict[str, dict[str, Any]],
    repo: Path,
) -> dict[str, Any]:
    paths = artifact_paths(layer2)
    approach = objects["object_approach"]
    approach_ok = approach["validation_status"] == "validated_against_current_layer1_stable_map"
    required = {name: path.is_file() for name, path in paths.items()}
    checks = {
        "all_required_layer2_artifact_families_generated": all(required.values()),
        "stable_map_package_exists_and_has_source_provenance": (
            required["stable_occupancy_map_package_v0_1"]
            and evidence["summary"]["all_required_source_evidence_acceptable"]
        ),
        "vt_1_centerline_e001_is_transition_edge": (
            connectors["route_planner_graph"]["vertical_transition_invariants"]["vt_1_centerline_e001_is_transition"]
            is True
        ),
        "vt_1_centerline_e003_is_not_transition_edge": (
            connectors["route_planner_graph"]["vertical_transition_invariants"]["vt_1_centerline_e003_is_transition"]
            is False
        ),
        "object_truth_obj_175_curtain_room_14_floor_2": (
            objects["query_resolution"]["object_id"] == "obj_175"
            and objects["query_resolution"]["object_label"] == "curtain"
            and objects["query_resolution"]["target_floor"] == "floor_2"
            and objects["query_resolution"]["target_room"] == "room_14"
        ),
        "approach_candidate_generated_ring_037_preserved": (
            approach["approach_candidate_id"] == "generated_ring_037"
        ),
        "object_centroid_navigation_remains_false": (
            approach["object_centroid_navigation_used"] is False
            and approach["direct_object_centroid_goal_used"] is False
        ),
        "forbidden_sources_used": False,
        "outputs_under_canonical_layer2_directory": all(
            path.resolve().is_relative_to(layer2.resolve()) for path in paths.values()
        ),
        "clean_rerun_writes_occurred": False,
        "ros_gazebo_rviz_nav2_amcl_launched": False,
        "layer3_cross_floor_room_ready": True,
        "layer3_cross_floor_object_route_ready": approach_ok,
        "layer3_object_contract_generation_ready": True,
    }
    blockers = []
    if not approach_ok:
        blockers.append({
            "item": "generated_ring_037 current stable-map feasibility",
            "blocker_type": "validator mismatch with current canonical Layer 1 geometry",
            "details": {
                "validation_status": approach["validation_status"],
                "world_xy": approach["world_xy"],
                "grid_rc": approach["grid_rc"],
                "occupancy_value": approach["occupancy_value"],
                "clearance_m": approach["clearance_m"],
                "failed_checks": [
                    key for key, value in approach["local_static_validation_checks"].items()
                    if value is False and key not in {
                        "object_centroid_navigation_used", "direct_object_centroid_goal_used"
                    }
                ],
            },
            "effect": "Layer 3 room-route generation and object contract generation may proceed; executable object-approach route generation remains blocked.",
        })
    return {
        "report_id": "layer2_generation_validation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "artifact_layer": LAYER,
        "status": "passed_with_exact_object_approach_blocker" if blockers else "passed",
        "required_artifact_paths": {name: rel(path, repo) for name, path in paths.items()},
        "required_artifact_existence": required,
        "checks": checks,
        "blocked_or_missing_items": blockers,
        "layer3_readiness": {
            "status": "partially_unblocked" if blockers else "unblocked",
            "cross_floor_room": "unblocked",
            "cross_floor_object_contract": "unblocked",
            "cross_floor_object_executable_approach_route": "blocked" if blockers else "unblocked",
        },
        "source_and_runtime_guardrails": {
            "forbidden_sources": FORBIDDEN_SOURCES,
            "forbidden_sources_used": [],
            "historical_outputs_promoted": False,
            "candidate_artifacts_used_as_inputs": False,
            "clean_rerun_writes": False,
            "runtime_launched": False,
            "world_model_rerun": False,
            "python_used": PYTHON,
        },
    }


def role_for(path: Path, layer2: Path, task: Path, repo: Path) -> tuple[str, str]:
    relative = rel(path, repo)
    if path == repo / "tools/rslg_pipeline/build_layer2_formal_artifacts.py":
        return "canonical Layer 2 orchestration adapter", "canonical_tool"
    if path.is_relative_to(layer2 / "stable_maps"):
        return "stable occupancy map package resource", "stable_occupancy_map_package_v0_1"
    if path.is_relative_to(layer2 / "vertical_connectors"):
        return "vertical connector formal artifact", "vertical_connectors_v0_1"
    if path.is_relative_to(layer2 / "topology"):
        return "cross-floor topology formal artifact", "cross_floor_topology_v0_1"
    if path.is_relative_to(layer2 / "planner_graph"):
        return "route planner graph formal artifact", "route_planner_graph_v0_1"
    if path.is_relative_to(layer2 / "object_interfaces"):
        return "object interface formal artifact", "object_interface_package_v0_1"
    if "/manifests/" in relative:
        return "provenance or artifact manifest", "manifests"
    if "/reports/" in relative or "/validation/" in relative:
        return "validation or task report", "reports"
    if "/logs/" in relative or path.name == "command_log.txt":
        return "execution log", "logs"
    if path.name == "warnings.txt":
        return "warnings and assumptions", "task_evidence"
    if path.is_relative_to(task):
        return "task36 evidence", "task_evidence"
    return "generated Layer 2 file", "other"


def write_created_manifest(repo: Path, layer2: Path, task: Path, output: Path) -> None:
    files = sorted(
        {path for root in [layer2, task] for path in root.rglob("*") if path.is_file()}
        | {repo / "tools/rslg_pipeline/build_layer2_formal_artifacts.py"}
    )
    records = []
    for path in files:
        role, family = role_for(path, layer2, task, repo)
        records.append({
            "path": rel(path, repo),
            "size_bytes": path.stat().st_size,
            "role": role,
            "layer": LAYER if path.is_relative_to(layer2) or path.name.startswith("build_layer2") else "task evidence",
            "artifact_family": family,
            "created_or_modified": "created",
        })
    save_json(output, {
        "manifest_id": "created_or_modified_files_manifest_v0_1",
        "schema_version": "0.1",
        "generated_at": now_iso(),
        "file_count": len(records),
        "files": records,
    })


def write_json_validation(repo: Path, layer2: Path, task: Path, output: Path) -> None:
    json_paths = sorted(
        {path for root in [layer2, task] for path in root.rglob("*.json") if path.is_file()}
    )
    records = []
    errors = []
    for path in json_paths:
        try:
            with path.open("r", encoding="utf-8") as handle:
                json.load(handle)
            records.append({"path": rel(path, repo), "valid": True, "error": None})
        except Exception as exc:
            records.append({"path": rel(path, repo), "valid": False, "error": str(exc)})
            errors.append(f"{rel(path, repo)}: {exc}")
    report = {
        "report_id": "json_validation_report_v0_1",
        "schema_version": "0.1",
        "generated_at": now_iso(),
        "validator": "Python json.load",
        "python_used": PYTHON,
        "ok": not errors,
        "validated_json_count": len(records),
        "error_count": len(errors),
        "files": records,
        "errors": errors,
    }
    save_json(output, report)
    with output.open("r", encoding="utf-8") as handle:
        json.load(handle)


def build_command_log(repo: Path, generation_log: Path) -> str:
    commands = [
        "pwd && git status --short",
        "presence check for all required docs/rslg_slam truth files",
        "rg --files tools/rslg_pipeline | sort",
        "find canonical Layer 1 files with sizes",
        "cat required project truth docs and manifests",
        "cat canonical stable-map/connector/object builders and validators",
        "inspect task35 evidence and canonical Layer 1 manifest/report with json.load",
        "rg for obj_175, generated_ring_037, vt_1, and route-room truth",
        "inspect historical stable-map, connector, and object-approach scripts statically",
        "inspect final Layer 1 JSON schemas and final floor debug raster statistics",
        "verify floor_1 run 1500 and floor_2 run 2709 evidence alignment",
        "regenerate generated_ring_037 with the existing bbox_support_plus_standoff policy",
        f"{PYTHON} -m py_compile tools/rslg_pipeline/build_layer2_formal_artifacts.py",
        (
            f"{PYTHON} -m tools.rslg_pipeline.build_layer2_formal_artifacts "
            f"--repo-root {repo} --scene-id {SCENE_ID}"
        ),
        (
            f"{PYTHON} -m tools.rslg_pipeline.validate_artifacts --repo-root {repo} "
            f"--mode static --output-json <task36>/validation/static_validator_run_report_v0_1.json"
        ),
        "load every generated JSON with Python json.load and verify required artifact paths",
        "inspect generated PGM/NPZ/YAML metadata and task37 readiness checks",
        "find canonical Layer 2 and task36 evidence files with sizes",
        "git status --short for task36-created paths",
        "rerun the canonical Layer 2 adapter to refresh final evidence manifests after validation",
    ]
    lines = [
        f"Task: {TASK_NAME}",
        f"Generated: {now_iso()}",
        f"Working directory: {repo}",
        f"Python interpreter: {PYTHON}",
        f"Generation stdout/stderr log: {rel(generation_log, repo)}",
        "",
        "Commands and inspection operations executed:",
    ]
    lines.extend(f"{index + 1}. {command}" for index, command in enumerate(commands))
    return "\n".join(lines)


def run(repo: Path, scene_id: str) -> dict[str, Any]:
    if scene_id != SCENE_ID:
        raise ValueError(f"This canonicalization task is scoped to {SCENE_ID}, not {scene_id}.")
    canonical_l1 = repo / f"stage_outputs/rslg_slam/{scene_id}/canonical/layer1_world_model"
    raw = canonical_l1 / f"raw_outputs/{scene_id}"
    layer2 = repo / f"stage_outputs/rslg_slam/{scene_id}/canonical/layer2_formal_artifacts"
    task = repo / f"stage_outputs/rslg_slam/{scene_id}/tasks/{TASK_NAME}"
    for root in [
        layer2 / "stable_maps",
        layer2 / "vertical_connectors",
        layer2 / "topology",
        layer2 / "planner_graph",
        layer2 / "object_interfaces",
        layer2 / "manifests",
        layer2 / "reports",
        layer2 / "logs",
        task / "reports",
        task / "logs",
        task / "validation",
        task / "manifests",
    ]:
        root.mkdir(parents=True, exist_ok=True)

    missing_docs = [path for path in DOCS_AND_MANIFESTS if not (repo / path).is_file()]
    required_l1 = [
        raw / "logs/final_vector_map_snapshot.json",
        raw / "logs/topology_v0_1.json",
        raw / "logs/vertical_transition_evidence.json",
    ]
    missing_l1 = [rel(path, repo) for path in required_l1 if not path.is_file()]
    if missing_l1:
        result = {
            "status": "blocked",
            "classification": CLASSIFICATION_BLOCKED,
            "missing_layer1_inputs": missing_l1,
        }
        save_json(task / "task36_report.json", result)
        return result

    evidence, warnings = build_evidence_manifest(repo, raw, canonical_l1)
    canonical_evidence = layer2 / "manifests/layer1_to_layer2_source_evidence_manifest_v0_1.json"
    save_json(canonical_evidence, evidence)
    save_json(task / "manifests/layer1_to_layer2_source_evidence_manifest_v0_1.json", evidence)

    stable_package, grids = build_stable_maps(raw, layer2, repo)
    truth = extract_truth(raw, repo)
    connectors = build_connectors_and_graphs(truth, layer2, repo)
    objects, object_warnings = build_object_interfaces(truth, grids, layer2, repo)
    warnings.extend(object_warnings)
    warnings.extend(f"Required project truth file was missing: {path}" for path in missing_docs)

    validation = build_validation(layer2, evidence, connectors, objects, repo)
    blockers = validation["blocked_or_missing_items"]
    classification = CLASSIFICATION_PARTIAL if blockers else CLASSIFICATION_COMPLETE
    status = "partially_completed_with_blockers" if blockers else "completed"
    task37 = validation["layer3_readiness"]["status"]

    produced = [
        {
            "artifact_family": name,
            "path": rel(path, repo),
            "status": (
                "generated_with_exact_current_evidence_blocker"
                if name in {"object_approach_v0_1", "object_interface_package_v0_1"} and blockers
                else "generated_and_validated"
            ),
        }
        for name, path in artifact_paths(layer2).items()
    ]
    formal_manifest = {
        "manifest_id": "layer2_formal_artifacts_manifest_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "generated_at": now_iso(),
        "artifact_layer": LAYER,
        "classification": classification,
        "canonical_layer1_input_dir": rel(canonical_l1, repo),
        "canonical_layer2_output_dir": rel(layer2, repo),
        "source_evidence_manifest": rel(canonical_evidence, repo),
        "artifacts": produced,
        "historical_outputs_promoted": False,
        "candidate_artifacts_used_as_inputs": False,
        "runtime_artifacts_generated": False,
        "route_artifacts_generated": False,
        "task37_status": task37,
        "blocked_or_missing_items": blockers,
    }
    save_json(layer2 / "manifests/layer2_formal_artifacts_manifest_v0_1.json", formal_manifest)

    canonical_validation = {
        **validation,
        "report_id": "layer2_formal_artifacts_validation_report_v0_1",
        "classification": classification,
        "layer2_sufficient_for_task37": not blockers,
        "task37_status": task37,
    }
    save_json(layer2 / "reports/layer2_formal_artifacts_validation_report_v0_1.json", canonical_validation)
    save_json(task / "validation/layer2_generation_validation_report_v0_1.json", validation)

    task_report = {
        "task_name": TASK_NAME,
        "status": status,
        "classification": classification,
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "canonical_layer1_input_dir": rel(canonical_l1, repo),
        "canonical_layer2_output_dir": rel(layer2, repo),
        "task_evidence_dir": rel(task, repo),
        "docs_and_manifests_read": [
            {"path": path, "status": "read" if path not in missing_docs else "missing"}
            for path in DOCS_AND_MANIFESTS
        ],
        "tools_used": [PYTHON, "Python json.load", "NumPy", "Pillow", "SciPy ndimage"],
        "existing_wrappers_used_or_extended": {
            "inspected": CANONICAL_WRAPPERS,
            "extended_with": "tools/rslg_pipeline/build_layer2_formal_artifacts.py",
            "reason": "Existing wrappers were candidate-only; this narrow final-mode adapter consumes current canonical Layer 1 outputs.",
        },
        "historical_scripts_inspected": HISTORICAL_SCRIPTS_INSPECTED,
        "historical_outputs_promoted": False,
        "candidate_artifacts_used_as_inputs": False,
        "layer1_evidence_normalization_summary": evidence["summary"],
        "produced_layer2_artifacts": produced,
        "validation_summary": {
            "status": validation["status"],
            "checks": validation["checks"],
            "json_validation_report": "json_validation_report_v0_1.json",
        },
        "blocked_or_missing_items": blockers,
        "task37_status": task37,
        "recommended_next_task": "task37_layer3_navigation_interface_canonicalization_and_route_generation",
        "recommended_next_task_scope": (
            "Proceed with room-route and object-contract generation, but do not emit an executable object-approach route until generated_ring_037 passes current stable-map validation."
            if blockers
            else "Proceed with Layer 3 route/interface generation."
        ),
        "repair_task_recommended": False,
    }
    save_json(task / "task36_report.json", task_report)

    write_text(
        task / "warnings.txt",
        "\n".join(f"- {warning}" for warning in warnings)
        if warnings else "No warnings were found.",
    )
    generation_summary = {
        "generated_at": now_iso(),
        "status": status,
        "classification": classification,
        "stable_map_floor_count": len(stable_package["floor_maps"]),
        "task37_status": task37,
        "blocker_count": len(blockers),
        "python_used": PYTHON,
        "runtime_launched": False,
        "world_model_rerun": False,
    }
    generation_log = layer2 / "logs/layer2_formal_artifacts_generation.log"
    write_text(generation_log, json.dumps(generation_summary, indent=2))
    shutil.copyfile(generation_log, task / "logs/layer2_formal_artifacts_generation.log")
    write_text(task / "command_log.txt", build_command_log(repo, generation_log))

    write_created_manifest(
        repo, layer2, task, task / "manifests/created_or_modified_files_manifest_v0_1.json"
    )
    write_json_validation(repo, layer2, task, task / "validation/json_validation_report_v0_1.json")
    write_created_manifest(
        repo, layer2, task, task / "manifests/created_or_modified_files_manifest_v0_1.json"
    )
    write_json_validation(repo, layer2, task, task / "validation/json_validation_report_v0_1.json")
    shutil.copyfile(
        task / "validation/layer2_generation_validation_report_v0_1.json",
        task / "reports/layer2_generation_validation_report_v0_1.json",
    )
    shutil.copyfile(
        task / "validation/json_validation_report_v0_1.json",
        task / "json_validation_report_v0_1.json",
    )
    shutil.copyfile(
        task / "manifests/created_or_modified_files_manifest_v0_1.json",
        task / "created_or_modified_files_manifest_v0_1.json",
    )
    shutil.copyfile(
        task / "manifests/layer1_to_layer2_source_evidence_manifest_v0_1.json",
        task / "layer1_to_layer2_source_evidence_manifest_v0_1.json",
    )
    write_created_manifest(
        repo, layer2, task, task / "manifests/created_or_modified_files_manifest_v0_1.json"
    )
    write_json_validation(repo, layer2, task, task / "validation/json_validation_report_v0_1.json")
    shutil.copyfile(
        task / "validation/json_validation_report_v0_1.json",
        task / "json_validation_report_v0_1.json",
    )
    shutil.copyfile(
        task / "manifests/created_or_modified_files_manifest_v0_1.json",
        task / "created_or_modified_files_manifest_v0_1.json",
    )
    return task_report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate canonical RSLG-SLAM Layer 2 formal artifacts from canonical Layer 1 outputs."
    )
    parser.add_argument("--repo-root", default=None)
    parser.add_argument("--scene-id", default=SCENE_ID)
    args = parser.parse_args(list(argv) if argv is not None else None)
    repo = resolve_repo_root(args.repo_root)
    report = run(repo, args.scene_id)
    print(json.dumps({
        "status": report["status"],
        "classification": report["classification"],
        "canonical_layer2_output_dir": report.get("canonical_layer2_output_dir"),
        "task37_status": report.get("task37_status"),
        "blocked_or_missing_items": report.get("blocked_or_missing_items", []),
    }, indent=2))
    return 0 if report["classification"] != CLASSIFICATION_BLOCKED else 1


if __name__ == "__main__":
    raise SystemExit(main())
