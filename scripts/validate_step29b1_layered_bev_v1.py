"""
Validate Step29B1 layered BEV artifacts and write README_step29b1.md.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

from build_step29b1_layered_bev_from_step29a3_v1 import (
    ASSET_DIR,
    EXPECTED_KEY_GLOBAL_ROOM_IDS,
    EXPECTED_LOCAL_TO_GLOBAL_IF_REPORT_MATCHES,
    GLOBAL_MASK_NPY,
    KEY_ROOM_PAIRS,
    LAYERED_JSON,
    LAYERED_NPZ,
    MAPPING_JSON,
    OUTPUT_ROOT,
    README_PATH,
    REQUIRED_INPUTS,
    SCENE_ID,
    SHORT_SCENE_ID,
    SUMMARY_JSON,
    VALIDATION_JSON,
    VERSION,
    VIS_DIR,
    compute_room_counts,
    json_ready,
    load_gray_image_aligned,
    parse_tracking_mapping,
    read_json,
    rel,
    write_json,
)


REQUIRED_NPZ_LAYERS = [
    "room_mask_local_label_raw",
    "room_mask_local_label_repaired",
    "room_mask_global_id",
    "structural_wall",
    "free_space",
    "outside_boundary",
    "unknown_layer",
    "full_map_post_doors_reference",
    "gateway_candidate_placeholder",
    "object_obstacle_placeholder",
]

REQUIRED_VISUALIZATIONS = [
    f"{SHORT_SCENE_ID}_step29b1_global_room_mask.png",
    f"{SHORT_SCENE_ID}_step29b1_local_vs_global_room_mask.png",
    f"{SHORT_SCENE_ID}_step29b1_structural_wall_layer.png",
    f"{SHORT_SCENE_ID}_step29b1_free_unknown_layer.png",
    f"{SHORT_SCENE_ID}_step29b1_room_wall_overlay_global_ids.png",
    f"{SHORT_SCENE_ID}_step29b1_key_rooms_debug.png",
    f"{SHORT_SCENE_ID}_step29b1_room3_room11_debug.png",
    f"{SHORT_SCENE_ID}_step29b1_room11_room7_debug.png",
    f"{SHORT_SCENE_ID}_step29b1_room3_room7_debug.png",
    f"{SHORT_SCENE_ID}_step29b1_room11_room8_debug.png",
]


def get_mapping_int(mapping: Dict[str, Any]) -> Dict[int, int]:
    return {
        int(local): int(global_id)
        for local, global_id in mapping.get("local_marker_label_to_global_room_id", {}).items()
    }


def add_check(results: Dict[str, Any], name: str, passed: bool, detail: Any) -> None:
    results["checks"][name] = {"pass": bool(passed), "detail": json_ready(detail)}
    if not passed:
        results["overall_pass"] = False
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}")


def load_required_artifacts() -> Dict[str, Any]:
    return {
        "metadata": read_json(LAYERED_JSON),
        "mapping": read_json(MAPPING_JSON),
        "summary": read_json(SUMMARY_JSON),
        "tracking_report": read_json(REQUIRED_INPUTS["final_tracking_report"]),
        "grid_metadata": read_json(REQUIRED_INPUTS["final_grid_metadata"]),
        "npz": np.load(LAYERED_NPZ),
        "global_npy": np.load(GLOBAL_MASK_NPY),
    }


def update_readiness(overall_pass: bool, validation_results: Dict[str, Any]) -> None:
    readiness = {
        "can_proceed": bool(overall_pass),
        "status": "validation_passed" if overall_pass else "validation_failed",
        "condition": "Step29B2 gateway extraction may proceed only after Step29B1 validation passes.",
        "validation_results_path": rel(VALIDATION_JSON),
    }
    for path in [LAYERED_JSON, SUMMARY_JSON]:
        if not path.is_file():
            continue
        payload = read_json(path)
        if path == LAYERED_JSON:
            payload["step29b2_readiness"] = readiness
            payload.setdefault("quality_checks", {})["validation_overall_pass"] = bool(overall_pass)
            payload.setdefault("quality_checks", {})["validation_results_path"] = rel(VALIDATION_JSON)
        else:
            payload["overall_status"] = "validation_passed" if overall_pass else "validation_failed"
            payload["step29b2_can_proceed"] = bool(overall_pass)
            payload["step29b2_readiness_note"] = readiness["condition"]
            payload["validation_results_path"] = rel(VALIDATION_JSON)
            payload["validation_failed_checks"] = validation_results.get("failed_checks", [])
        write_json(path, payload)


def make_mapping_table(mapping: Dict[int, int], metrics: Dict[str, Any]) -> str:
    lines = [
        "| Local marker label | Global room ID | IoU | Intersection |",
        "|---:|---:|---:|---:|",
    ]
    for local, global_id in sorted(mapping.items()):
        metric = metrics.get(str(local), metrics.get(local, {}))
        iou = metric.get("iou")
        intersection = metric.get("intersection")
        iou_text = "n/a" if iou is None else str(iou)
        intersection_text = "n/a" if intersection is None else str(intersection)
        lines.append(f"| {local} | {global_id} | {iou_text} | {intersection_text} |")
    return "\n".join(lines)


def write_readme(validation_results: Dict[str, Any]) -> None:
    metadata = read_json(LAYERED_JSON)
    mapping_artifact = read_json(MAPPING_JSON)
    summary = read_json(SUMMARY_JSON)
    mapping = get_mapping_int(mapping_artifact)
    metrics = mapping_artifact.get("matched_metrics_by_local_marker_label", {})
    visual_paths = [VIS_DIR / name for name in REQUIRED_VISUALIZATIONS]
    input_lines = "\n".join(
        f"- `{name}`: `{rel(path)}`" for name, path in REQUIRED_INPUTS.items()
    )
    not_used_lines = "\n".join(f"- {item}" for item in metadata.get("files_not_used", []))
    layers_lines = "\n".join(
        f"- `{name}`: {definition.get('meaning')}"
        for name, definition in metadata.get("layer_definitions", {}).items()
    )
    visual_lines = "\n".join(f"- `{rel(path)}`" for path in visual_paths)
    key_presence = metadata.get("quality_checks", {}).get("expected_key_rooms_present", {})
    key_presence_lines = "\n".join(f"- {room}: {present}" for room, present in key_presence.items())
    global_ids = metadata.get("global_room_ids_present", [])
    validation_status = "PASS" if validation_results.get("overall_pass") else "FAIL"
    step29b2 = "YES" if validation_results.get("overall_pass") else "NO"

    readme = f"""# Step29B1: Layered BEV from Step29A3 for {SCENE_ID}

## Summary

Step29B1 converted the successful Step29A3 final raster exports into a downstream-safe layered BEV artifact. The critical output is `room_mask_global_id`, where each occupied room cell stores the persistent topology global room ID, not the local watershed marker label.

Validation status: **{validation_status}**

Step29B2 gateway extraction can proceed: **{step29b2}**

## Exact Input Files Used

{input_lines}

## Exact Files Not Used

{not_used_lines}

Room polygons were not used.

Old BEV artifacts were not used.

Topology JSON was not used as geometry.

Gateway extraction was not performed.

## Local Labels Are Not Global Room IDs

The Step29A3 final label rasters store local watershed marker labels. Those local marker labels are frame/run-local segmentation labels and are not safe to use as persistent topology room IDs. Step29B1 remaps them through the final tracking report before writing `room_mask_global_id`.

{make_mapping_table(mapping, metrics)}

## Global Room IDs Present

{global_ids}

## Key Room Presence

{key_presence_lines}

Room 11 present in `room_mask_global_id`: **{summary.get("room_11_present")}**

## Grid Metadata

- Grid shape: `{metadata.get("height")} x {metadata.get("width")}`
- Resolution: `{metadata.get("resolution")}` m/pixel
- Origin: `{metadata.get("origin")}`
- Coordinate convention: {metadata.get("coordinate_convention")}
- Grid indexing convention: {metadata.get("grid_indexing_convention")}
- Map formula: `x = origin_x + col * resolution`, `y = origin_y + row * resolution`

## Layer Descriptions

{layers_lines}

## Validation Summary

- Overall pass: `{validation_results.get("overall_pass")}`
- Failed checks: `{validation_results.get("failed_checks", [])}`
- Room cell counts by global room ID: `{validation_results.get("room_cell_counts_by_global_room_id", {})}`
- Wall-room overlap count: `{validation_results.get("wall_room_overlap_count")}`
- Unknown-wall overlap count: `{validation_results.get("unknown_wall_overlap_count")}`
- Debug PNG padding corrections: `{validation_results.get("png_alignment_adjustments", {})}`

## Visualization Paths

{visual_lines}

## Known Limitations

{chr(10).join(f"- {item}" for item in metadata.get("known_limitations", []))}

## Step29B2 Readiness

Step29B2 can proceed only if this validation passes. Current result: **{step29b2}**.
"""
    README_PATH.parent.mkdir(parents=True, exist_ok=True)
    README_PATH.write_text(readme)


def validate() -> Dict[str, Any]:
    results: Dict[str, Any] = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1_validation_results",
        "version": VERSION,
        "overall_pass": True,
        "checks": {},
    }

    for name, path in REQUIRED_INPUTS.items():
        add_check(results, f"input_exists_{name}", path.is_file(), rel(path))

    generated_paths = {
        "layered_json": LAYERED_JSON,
        "layered_npz": LAYERED_NPZ,
        "mapping_json": MAPPING_JSON,
        "global_room_mask_npy": GLOBAL_MASK_NPY,
        "summary_json": SUMMARY_JSON,
    }
    for name, path in generated_paths.items():
        add_check(results, f"generated_exists_{name}", path.is_file(), rel(path))

    if not all(path.is_file() for path in list(REQUIRED_INPUTS.values()) + list(generated_paths.values())):
        results["failed_checks"] = [
            name for name, check in results["checks"].items() if not check["pass"]
        ]
        write_json(VALIDATION_JSON, results)
        update_readiness(False, results)
        write_readme(results)
        return results

    artifacts = load_required_artifacts()
    metadata = artifacts["metadata"]
    mapping_artifact = artifacts["mapping"]
    tracking_report = artifacts["tracking_report"]
    grid_metadata = artifacts["grid_metadata"]
    npz = artifacts["npz"]
    global_npy = artifacts["global_npy"]

    for layer in REQUIRED_NPZ_LAYERS:
        add_check(results, f"npz_contains_{layer}", layer in npz.files, list(npz.files))

    raw = np.load(REQUIRED_INPUTS["final_tracked_room_labels"])
    repaired = np.load(REQUIRED_INPUTS["final_repaired_room_labels"])
    add_check(results, "tracked_repaired_label_shapes_match", raw.shape == repaired.shape, {
        "tracked": raw.shape,
        "repaired": repaired.shape,
    })
    target_shape = repaired.shape

    png_alignment: Dict[str, Any] = {}
    for name in [
        "final_walls_skeleton",
        "final_free_space",
        "final_outside_boundary",
        "final_full_map",
    ]:
        image, adjustment = load_gray_image_aligned(REQUIRED_INPUTS[name], target_shape)
        png_alignment[name] = adjustment
        add_check(results, f"{name}_shape_compatible_after_alignment", image.shape == target_shape, adjustment)
        if adjustment["operation"] != "none":
            add_check(
                results,
                f"{name}_padding_documented",
                name in metadata.get("quality_checks", {}).get("image_alignment_adjustments", {}),
                metadata.get("quality_checks", {}).get("image_alignment_adjustments", {}).get(name),
            )
    results["png_alignment_adjustments"] = png_alignment

    resolution = metadata.get("resolution")
    origin = metadata.get("origin")
    add_check(results, "resolution_documented_as_0p05", abs(float(resolution) - 0.05) < 1e-9, resolution)
    add_check(results, "origin_documented_as_minus50_minus50", origin == [-50.0, -50.0], origin)

    mapping = get_mapping_int(mapping_artifact)
    report_mapping = parse_tracking_mapping(tracking_report, grid_metadata)[
        "local_marker_label_to_global_room_id"
    ]
    add_check(results, "label_to_global_id_map_non_empty", bool(mapping), mapping)
    add_check(results, "mapping_matches_tracking_report", mapping == report_mapping, {
        "artifact": mapping,
        "tracking_report": report_mapping,
    })

    global_mask = npz["room_mask_global_id"]
    add_check(results, "global_mask_matches_global_npy", np.array_equal(global_mask, global_npy), rel(GLOBAL_MASK_NPY))
    add_check(results, "global_room_mask_shape_matches_local_label_grid", global_mask.shape == target_shape, {
        "global_mask": global_mask.shape,
        "local": target_shape,
    })
    global_ids_present = sorted(int(v) for v in np.unique(global_mask) if int(v) > 0)
    add_check(results, "global_room_mask_contains_key_global_ids", all(room in global_ids_present for room in EXPECTED_KEY_GLOBAL_ROOM_IDS), global_ids_present)
    add_check(results, "room_11_present_as_global_id_11", 11 in global_ids_present, global_ids_present)

    for local, expected_global in EXPECTED_LOCAL_TO_GLOBAL_IF_REPORT_MATCHES.items():
        actual_report_global = report_mapping.get(local)
        actual_artifact_global = mapping.get(local)
        expected_or_report = actual_report_global if actual_report_global is not None else expected_global
        add_check(
            results,
            f"local_marker_label_{local}_maps_to_global_room_{expected_or_report}",
            actual_artifact_global == expected_or_report,
            {
                "expected_from_step29a3_context": expected_global,
                "actual_tracking_report_global_id": actual_report_global,
                "artifact_global_id": actual_artifact_global,
            },
        )

    forbidden = metadata.get("quality_checks", {}).get("forbidden_geometry_sources_used", {})
    add_check(results, "no_old_room_polygons_used", forbidden.get("room_polygons") is False, forbidden)
    add_check(results, "no_old_bev_used", forbidden.get("old_bev") is False, forbidden)
    add_check(results, "no_topology_json_used_as_geometry", forbidden.get("topology_json_edges") is False, forbidden)
    add_check(results, "no_route_carved_bev_used", forbidden.get("selected_public_route_edges_carved_w0p6") is False, forbidden)

    gateway_nonzero = int(npz["gateway_candidate_placeholder"].sum())
    object_nonzero = int(npz["object_obstacle_placeholder"].sum())
    add_check(results, "no_gateway_extraction_performed", gateway_nonzero == 0 and metadata.get("quality_checks", {}).get("gateway_extraction_performed") is False, gateway_nonzero)
    add_check(results, "no_object_obstacle_inference_performed", object_nonzero == 0, object_nonzero)

    structural_wall = npz["structural_wall"] > 0
    unknown_layer = npz["unknown_layer"] > 0
    wall_room_overlap_count = int(np.logical_and(structural_wall, global_mask > 0).sum())
    unknown_wall_overlap_count = int(np.logical_and(structural_wall, unknown_layer).sum())
    results["wall_room_overlap_count"] = wall_room_overlap_count
    results["unknown_wall_overlap_count"] = unknown_wall_overlap_count
    add_check(results, "unknown_exterior_not_counted_as_structural_wall", unknown_wall_overlap_count == 0, unknown_wall_overlap_count)
    add_check(results, "wall_room_overlap_computed_and_reported", metadata.get("quality_checks", {}).get("wall_room_overlap_count") == wall_room_overlap_count, wall_room_overlap_count)

    room_counts = compute_room_counts(global_mask)
    results["room_cell_counts_by_global_room_id"] = room_counts
    metadata_counts = {
        int(k): int(v)
        for k, v in metadata.get("quality_checks", {}).get("room_cell_counts_by_global_room_id", {}).items()
    }
    add_check(results, "room_cell_counts_computed_by_global_room_id", metadata_counts == room_counts and bool(room_counts), room_counts)

    for name in REQUIRED_VISUALIZATIONS:
        path = VIS_DIR / name
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED) if path.is_file() else None
        add_check(results, f"visualization_exists_{name}", path.is_file() and image is not None, rel(path))

    for room_a, room_b in KEY_ROOM_PAIRS:
        path = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_room{room_a}_room{room_b}_debug.png"
        add_check(results, f"key_room_pair_debug_generated_room_{room_a}_room_{room_b}", path.is_file(), rel(path))

    files_not_used_text = " ".join(metadata.get("files_not_used", [])).lower()
    add_check(results, "forbidden_sources_documented_as_not_used", all(
        token in files_not_used_text
        for token in ["room polygons", "old navigation bev", "topology json"]
    ), metadata.get("files_not_used", []))

    results["global_room_ids_present"] = global_ids_present
    results["room_11_presence_result"] = 11 in global_ids_present
    results["validation_results_path"] = rel(VALIDATION_JSON)
    results["readme_path"] = rel(README_PATH)
    results["step29b2_can_proceed"] = bool(results["overall_pass"])
    results["failed_checks"] = [
        name for name, check in results["checks"].items() if not check["pass"]
    ]

    write_json(VALIDATION_JSON, results)
    update_readiness(bool(results["overall_pass"]), results)
    write_readme(results)
    return results


def main() -> None:
    results = validate()
    print("")
    print(f"Step29B1 validation overall_pass: {results['overall_pass']}")
    print(f"Validation results: {rel(VALIDATION_JSON)}")
    print(f"README: {rel(README_PATH)}")
    print(f"Step29B2 can proceed: {results['step29b2_can_proceed']}")


if __name__ == "__main__":
    main()
