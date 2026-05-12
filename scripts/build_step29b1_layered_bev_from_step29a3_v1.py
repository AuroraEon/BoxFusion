"""
Build Step29B1 layered BEV artifacts from Step29A3 final raster exports.

This step remaps local watershed marker labels into persistent global room IDs.
It does not extract gateways, augment topology, or use topology/old-BEV geometry.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Tuple

import cv2
import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT_SCENE_ID = "00824"
VERSION = "v0_1"
REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29a3_00824_stage_a_debug_raster_export"
SOURCE_SCENE_DIR = SOURCE_ROOT / "scenes" / SCENE_ID
SOURCE_FINAL_DIR = SOURCE_SCENE_DIR / "final_raster_export"
SOURCE_TRACKING_REPORT = (
    SOURCE_SCENE_DIR
    / "debug_room"
    / "floor_1"
    / "run_2252_12_tracking_report.json"
)
OUTPUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3"
ASSET_DIR = OUTPUT_ROOT / "generated" / "assets"
VIS_DIR = OUTPUT_ROOT / "generated" / "visualizations"

LAYERED_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.json"
LAYERED_NPZ = ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.npz"
MAPPING_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_room_label_mapping_{VERSION}.json"
GLOBAL_MASK_NPY = ASSET_DIR / f"{SHORT_SCENE_ID}_global_room_mask_{VERSION}.npy"
SUMMARY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_summary_{VERSION}.json"
VALIDATION_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_validation_results_{VERSION}.json"
README_PATH = OUTPUT_ROOT / "README_step29b1.md"

REQUIRED_INPUTS = {
    "final_tracked_room_labels": SOURCE_FINAL_DIR / "final_tracked_room_labels.npy",
    "final_repaired_room_labels": SOURCE_FINAL_DIR / "final_repaired_room_labels.npy",
    "final_walls_skeleton": SOURCE_FINAL_DIR / "final_walls_skeleton.png",
    "final_outside_boundary": SOURCE_FINAL_DIR / "final_outside_boundary.png",
    "final_full_map": SOURCE_FINAL_DIR / "final_full_map.png",
    "final_free_space": SOURCE_FINAL_DIR / "final_free_space.png",
    "final_grid_metadata": SOURCE_FINAL_DIR / "final_grid_metadata.json",
    "final_room_id_summary": SOURCE_FINAL_DIR / "final_room_id_summary.json",
    "cycle_manifest": SOURCE_FINAL_DIR / "cycle_manifest.json",
    "final_tracking_report": SOURCE_TRACKING_REPORT,
}

EXPECTED_KEY_GLOBAL_ROOM_IDS = [1, 3, 7, 8, 11]
EXPECTED_LOCAL_TO_GLOBAL_IF_REPORT_MATCHES = {5: 11, 6: 7, 4: 3, 8: 8}
KEY_ROOM_PAIRS = [(3, 11), (11, 7), (3, 7), (11, 8)]


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_ready(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_ready(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, Path):
        return rel(value)
    return value


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_ready(payload), indent=2, sort_keys=False) + "\n")


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def require_inputs() -> Dict[str, bool]:
    return {name: path.is_file() for name, path in REQUIRED_INPUTS.items()}


def load_gray_image_aligned(path: Path, target_shape: Tuple[int, int]) -> Tuple[np.ndarray, Dict[str, Any]]:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not read image: {path}")

    source_shape = tuple(int(v) for v in image.shape)
    target_h, target_w = target_shape
    if source_shape == target_shape:
        return image, {
            "source_shape": list(source_shape),
            "target_shape": list(target_shape),
            "operation": "none",
            "detail": "PNG shape already matches label grid.",
        }

    src_h, src_w = source_shape
    delta_h = src_h - target_h
    delta_w = src_w - target_w
    if delta_h >= 0 and delta_w >= 0 and delta_h % 2 == 0 and delta_w % 2 == 0:
        top = delta_h // 2
        left = delta_w // 2
        cropped = image[top : top + target_h, left : left + target_w]
        return cropped, {
            "source_shape": list(source_shape),
            "target_shape": list(target_shape),
            "operation": "center_crop",
            "crop_top": int(top),
            "crop_bottom": int(delta_h - top),
            "crop_left": int(left),
            "crop_right": int(delta_w - left),
            "detail": "Detected debug PNG padding and center-cropped to label grid shape.",
        }

    raise ValueError(
        f"Unsupported raster shape mismatch for {path}: source={source_shape}, target={target_shape}"
    )


def parse_tracking_mapping(tracking_report: Dict[str, Any], metadata: Dict[str, Any]) -> Dict[str, Any]:
    tracking = tracking_report.get("tracking", {})
    matched = tracking.get("matched", [])
    new_rooms = tracking.get("new_rooms", [])

    local_to_global: Dict[int, int] = {}
    metrics: Dict[int, Dict[str, Any]] = {}
    for entry in list(matched) + list(new_rooms):
        if not isinstance(entry, dict):
            continue
        local = (
            entry.get("marker_label")
            if entry.get("marker_label") is not None
            else entry.get("local_label", entry.get("label"))
        )
        global_id = entry.get("global_id", entry.get("tracked_id"))
        if local is None or global_id is None:
            continue
        local_i = int(local)
        global_i = int(global_id)
        local_to_global[local_i] = global_i
        metrics[local_i] = {
            "global_room_id": global_i,
            "iou": entry.get("iou"),
            "intersection": entry.get("intersection"),
            "source_field_for_local_label": "marker_label"
            if entry.get("marker_label") is not None
            else "local_label_or_label",
        }

    metadata_map = {
        int(local): int(global_id)
        for local, global_id in metadata.get("label_to_global_id_map", {}).items()
    }
    if not local_to_global and metadata_map:
        local_to_global = dict(metadata_map)
        for local_i, global_i in local_to_global.items():
            metrics[local_i] = {
                "global_room_id": global_i,
                "iou": None,
                "intersection": None,
                "source_field_for_local_label": "final_grid_metadata.label_to_global_id_map",
            }

    global_to_local = {global_id: local for local, global_id in local_to_global.items()}
    return {
        "local_marker_label_to_global_room_id": local_to_global,
        "global_room_id_to_local_marker_label": global_to_local,
        "metadata_label_to_global_id_map": metadata_map,
        "matched_metrics_by_local_marker_label": metrics,
        "retained_missing": tracking.get("retained_missing", []),
        "dropped_missing": tracking.get("dropped_missing", []),
        "tracking_threshold": tracking.get("threshold"),
        "tracking_missed_cycles_limit": tracking.get("missed_cycles_limit"),
    }


def remap_global_room_mask(local_labels: np.ndarray, mapping: Dict[int, int]) -> np.ndarray:
    global_mask = np.zeros(local_labels.shape, dtype=np.int32)
    for local_label, global_id in sorted(mapping.items()):
        global_mask[local_labels == int(local_label)] = int(global_id)
    return global_mask


def compute_room_counts(global_mask: np.ndarray) -> Dict[int, int]:
    values, counts = np.unique(global_mask, return_counts=True)
    return {
        int(value): int(count)
        for value, count in zip(values.tolist(), counts.tolist())
        if int(value) > 0
    }


def build_layered_bev() -> Dict[str, Any]:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)

    input_checks = require_inputs()
    missing = [name for name, exists in input_checks.items() if not exists]
    if missing:
        raise FileNotFoundError(f"Missing required Step29A3 inputs: {missing}")

    metadata = read_json(REQUIRED_INPUTS["final_grid_metadata"])
    room_summary = read_json(REQUIRED_INPUTS["final_room_id_summary"])
    cycle_manifest = read_json(REQUIRED_INPUTS["cycle_manifest"])
    tracking_report = read_json(REQUIRED_INPUTS["final_tracking_report"])

    raw_labels = np.load(REQUIRED_INPUTS["final_tracked_room_labels"]).astype(np.int32, copy=False)
    repaired_labels = np.load(REQUIRED_INPUTS["final_repaired_room_labels"]).astype(np.int32, copy=False)
    if raw_labels.shape != repaired_labels.shape:
        raise ValueError(f"Raw/repaired label shape mismatch: {raw_labels.shape} vs {repaired_labels.shape}")
    target_shape = tuple(int(v) for v in repaired_labels.shape)

    image_adjustments: Dict[str, Dict[str, Any]] = {}
    walls_image, image_adjustments["final_walls_skeleton"] = load_gray_image_aligned(
        REQUIRED_INPUTS["final_walls_skeleton"], target_shape
    )
    free_image, image_adjustments["final_free_space"] = load_gray_image_aligned(
        REQUIRED_INPUTS["final_free_space"], target_shape
    )
    outside_image, image_adjustments["final_outside_boundary"] = load_gray_image_aligned(
        REQUIRED_INPUTS["final_outside_boundary"], target_shape
    )
    full_map_image, image_adjustments["final_full_map"] = load_gray_image_aligned(
        REQUIRED_INPUTS["final_full_map"], target_shape
    )

    mapping_info = parse_tracking_mapping(tracking_report, metadata)
    local_to_global = mapping_info["local_marker_label_to_global_room_id"]
    global_mask = remap_global_room_mask(repaired_labels, local_to_global)

    structural_wall = (walls_image > 127).astype(np.uint8)
    free_space = (free_image > 127).astype(np.uint8)
    outside_boundary = (outside_image > 127).astype(np.uint8)
    full_map_reference = (full_map_image > 127).astype(np.uint8)
    unknown_layer = (outside_boundary == 0).astype(np.uint8)
    gateway_candidate_placeholder = np.zeros(target_shape, dtype=np.uint8)
    object_obstacle_placeholder = np.zeros(target_shape, dtype=np.uint8)

    np.savez_compressed(
        LAYERED_NPZ,
        room_mask_local_label_raw=raw_labels,
        room_mask_local_label_repaired=repaired_labels,
        room_mask_global_id=global_mask,
        structural_wall=structural_wall,
        free_space=free_space,
        outside_boundary=outside_boundary,
        unknown_layer=unknown_layer,
        full_map_post_doors_reference=full_map_reference,
        gateway_candidate_placeholder=gateway_candidate_placeholder,
        object_obstacle_placeholder=object_obstacle_placeholder,
    )
    np.save(GLOBAL_MASK_NPY, global_mask)

    raw_positive_labels = sorted(int(v) for v in np.unique(raw_labels) if int(v) > 0)
    repaired_positive_labels = sorted(int(v) for v in np.unique(repaired_labels) if int(v) > 0)
    unmapped_positive_labels = [
        label for label in repaired_positive_labels if int(label) not in local_to_global
    ]
    global_ids_present = sorted(int(v) for v in np.unique(global_mask) if int(v) > 0)
    room_cell_counts = compute_room_counts(global_mask)
    wall_room_overlap_count = int(np.logical_and(structural_wall > 0, global_mask > 0).sum())
    unknown_wall_overlap_count = int(np.logical_and(structural_wall > 0, unknown_layer > 0).sum())

    mapping_artifact = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1_room_label_mapping",
        "version": VERSION,
        "local_marker_label_to_global_room_id": local_to_global,
        "global_room_id_to_local_marker_label": mapping_info["global_room_id_to_local_marker_label"],
        "source_tracking_report_path": rel(REQUIRED_INPUTS["final_tracking_report"]),
        "source_final_labels_path": rel(REQUIRED_INPUTS["final_repaired_room_labels"]),
        "repaired_labels_used_for_global_remapping": True,
        "matched_iou_values_by_local_marker_label": {
            local: values.get("iou")
            for local, values in mapping_info["matched_metrics_by_local_marker_label"].items()
        },
        "intersection_values_by_local_marker_label": {
            local: values.get("intersection")
            for local, values in mapping_info["matched_metrics_by_local_marker_label"].items()
        },
        "matched_metrics_by_local_marker_label": mapping_info["matched_metrics_by_local_marker_label"],
        "retained_missing": mapping_info["retained_missing"],
        "dropped_missing": mapping_info["dropped_missing"],
        "unmapped_positive_local_labels_in_repaired_mask": unmapped_positive_labels,
        "notes": [
            "The label grid stores local watershed marker labels, not persistent topology room IDs.",
            "Only labels present in the final tracking report/global-id metadata are remapped.",
            "Unmapped positive local labels are written as 0 in room_mask_global_id.",
        ],
    }
    write_json(MAPPING_JSON, mapping_artifact)

    layer_definitions = {
        "room_mask_local_label_raw": {
            "source": rel(REQUIRED_INPUTS["final_tracked_room_labels"]),
            "meaning": "Raw local marker labels from segmentation/tracking debug output; provenance only.",
        },
        "room_mask_local_label_repaired": {
            "source": rel(REQUIRED_INPUTS["final_repaired_room_labels"]),
            "meaning": "Tier2-repaired local marker labels; preferred remapping input.",
        },
        "room_mask_global_id": {
            "source": "room_mask_local_label_repaired remapped through local_marker_label_to_global_room_id",
            "meaning": "Downstream-safe room mask storing persistent topology global room IDs per cell.",
        },
        "structural_wall": {
            "source": rel(REQUIRED_INPUTS["final_walls_skeleton"]),
            "meaning": "Authoritative structural wall evidence from Stage-A skeleton raster.",
        },
        "free_space": {
            "source": rel(REQUIRED_INPUTS["final_free_space"]),
            "meaning": "Stage-A free-space evidence.",
        },
        "outside_boundary": {
            "source": rel(REQUIRED_INPUTS["final_outside_boundary"]),
            "meaning": "Explored footprint evidence from Stage-A.",
        },
        "unknown_layer": {
            "source": "derived from outside_boundary == 0",
            "meaning": "Exterior/unsupported cells. These are unknown, not structural walls.",
        },
        "full_map_post_doors_reference": {
            "source": rel(REQUIRED_INPUTS["final_full_map"]),
            "meaning": "Reference occupancy-style map after door carving. Not authoritative wall geometry.",
        },
        "gateway_candidate_placeholder": {
            "source": "empty Step29B1 placeholder",
            "meaning": "All zeros. Gateway extraction is intentionally deferred.",
        },
        "object_obstacle_placeholder": {
            "source": "empty Step29B1 placeholder",
            "meaning": "All zeros. Object/furniture inference is intentionally deferred.",
        },
    }

    quality_checks = {
        "required_inputs_exist": input_checks,
        "raw_repaired_label_shape": list(target_shape),
        "image_alignment_adjustments": image_adjustments,
        "local_to_global_mapping_non_empty": bool(local_to_global),
        "global_room_ids_present": global_ids_present,
        "expected_key_rooms_present": {
            f"room_{room_id}": room_id in global_ids_present
            for room_id in EXPECTED_KEY_GLOBAL_ROOM_IDS
        },
        "room_cell_counts_by_global_room_id": room_cell_counts,
        "wall_room_overlap_count": wall_room_overlap_count,
        "unknown_wall_overlap_count": unknown_wall_overlap_count,
        "gateway_placeholder_nonzero_count": int(gateway_candidate_placeholder.sum()),
        "object_obstacle_placeholder_nonzero_count": int(object_obstacle_placeholder.sum()),
        "unmapped_positive_local_labels_in_repaired_mask": unmapped_positive_labels,
        "forbidden_geometry_sources_used": {
            "room_polygons": False,
            "old_bev": False,
            "selected_public_route_edges_carved_w0p6": False,
            "old_route_candidate_bev": False,
            "topology_json_edges": False,
        },
        "gateway_extraction_performed": False,
    }

    layered_metadata = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1_layered_bev_from_step29a3",
        "version": VERSION,
        "source_step": "Step29A3",
        "source_step29a3_directory": rel(SOURCE_ROOT),
        "map_frame": "map",
        "resolution": float(metadata.get("resolution_m_per_pixel", 0.05)),
        "origin": metadata.get("origin", [-50.0, -50.0]),
        "width": int(target_shape[1]),
        "height": int(target_shape[0]),
        "coordinate_convention": "2D BEV grid in map frame, meters, x increases with columns and y increases with rows.",
        "grid_indexing_convention": "Arrays are indexed as [row, col] with shape [height, width].",
        "map_xy_from_grid_formula": {
            "x": "origin_x + col * resolution",
            "y": "origin_y + row * resolution",
        },
        "all_source_artifact_paths": {name: rel(path) for name, path in REQUIRED_INPUTS.items()},
        "files_not_used": [
            "room polygons",
            "old navigation BEV",
            "selected_public_route_edges_carved_w0p6",
            "old route candidate BEV",
            "topology JSON edges",
            "Nav2 outputs",
            "Gazebo outputs",
            "AMCL/TF/DWB/ROS execution artifacts",
        ],
        "layer_definitions": layer_definitions,
        "local_to_global_room_mapping": local_to_global,
        "global_room_ids_present": global_ids_present,
        "quality_checks": quality_checks,
        "known_limitations": [
            "Gateway candidates are placeholders only; no doorway/gateway extraction is performed in Step29B1.",
            "Object obstacle inference is a placeholder only.",
            "Local marker label 10 is present in the repaired label raster but absent from the final tracking report, so it is remapped to global ID 0.",
            "The structural wall layer is the Stage-A wall skeleton raster, not the full_map exterior occupancy.",
        ],
        "step29b2_readiness": {
            "can_proceed": None,
            "status": "pending_validation",
            "condition": "Step29B2 can proceed only if validate_step29b1_layered_bev_v1.py passes.",
        },
        "cycle_manifest": cycle_manifest,
        "final_room_id_summary": room_summary,
        "outputs": {
            "layered_bev_json": rel(LAYERED_JSON),
            "layered_bev_npz": rel(LAYERED_NPZ),
            "room_label_mapping_json": rel(MAPPING_JSON),
            "global_room_mask_npy": rel(GLOBAL_MASK_NPY),
            "summary_json": rel(SUMMARY_JSON),
            "validation_results_json": rel(VALIDATION_JSON),
            "readme": rel(README_PATH),
            "visualization_dir": rel(VIS_DIR),
        },
    }
    write_json(LAYERED_JSON, layered_metadata)

    summary = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1_summary",
        "version": VERSION,
        "overall_status": "built_pending_validation",
        "output_directory": rel(OUTPUT_ROOT),
        "layered_bev_json": rel(LAYERED_JSON),
        "layered_bev_npz": rel(LAYERED_NPZ),
        "global_room_mask_path": rel(GLOBAL_MASK_NPY),
        "mapping_json_path": rel(MAPPING_JSON),
        "validation_results_path": rel(VALIDATION_JSON),
        "readme_path": rel(README_PATH),
        "global_room_ids_present": global_ids_present,
        "room_11_present": 11 in global_ids_present,
        "room_cell_counts_by_global_room_id": room_cell_counts,
        "local_marker_label_to_global_room_id": local_to_global,
        "repaired_labels_used_for_global_remapping": True,
        "image_alignment_adjustments": image_adjustments,
        "step29b2_can_proceed": None,
        "step29b2_readiness_note": "Pending validation.",
    }
    write_json(SUMMARY_JSON, summary)

    return {
        "layered_metadata": layered_metadata,
        "summary": summary,
        "mapping": mapping_artifact,
    }


def main() -> None:
    result = build_layered_bev()
    summary = result["summary"]
    print("Step29B1 layered BEV build complete (pending validation).")
    print(f"Output directory: {summary['output_directory']}")
    print(f"Layered BEV JSON: {summary['layered_bev_json']}")
    print(f"Layered BEV NPZ: {summary['layered_bev_npz']}")
    print(f"Global room mask: {summary['global_room_mask_path']}")
    print(f"Mapping JSON: {summary['mapping_json_path']}")
    print(f"Room 11 present: {summary['room_11_present']}")


if __name__ == "__main__":
    main()
