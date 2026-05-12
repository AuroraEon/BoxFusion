"""
Diagnostic Step29B2R gateway sensitivity review for scene 00824-Dd4bFSTQ8gi.

This script reads Step29B1 and Step29B2 artifacts, reruns diagnostic-only
connectivity checks under relaxed wall/interior/boundary policies, and writes
new Step29B2R artifacts. It never modifies Step29B2 outputs and never produces
topology augmentation or navigation goal files.
"""

from __future__ import annotations

import hashlib
import json
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from extract_step29b2_gateway_candidates_v1 import (
    ORIGIN,
    REPO_ROOT,
    RESOLUTION,
    SCENE_ID,
    SHORT_SCENE_ID,
    VERSION,
    bbox_for_mask,
    build_boundary_band,
    connected_components,
    crop_slices_for_rooms,
    erode_disk,
    extract_pair_candidates,
    grid_to_xy,
    json_ready,
    meters_to_cells,
    normalize_pair,
    reachable_room_interiors,
    read_json,
    rel,
    shifted_or,
)


STEP29B1_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3"
STEP29B1_ASSET_DIR = STEP29B1_ROOT / "generated" / "assets"
STEP29B2_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b2_00824_gateway_candidates_from_layered_bev"
STEP29B2_ASSET_DIR = STEP29B2_ROOT / "generated" / "assets"
STEP29B2R_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b2r_00824_gateway_sensitivity_review"
ASSET_DIR = STEP29B2R_ROOT / "generated" / "assets"
VIS_DIR = STEP29B2R_ROOT / "generated" / "visualizations"
README_PATH = STEP29B2R_ROOT / "README_step29b2r.md"

STEP29B1_INPUTS = {
    "layered_bev_json": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.json",
    "layered_bev_npz": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.npz",
    "global_room_mask": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_global_room_mask_{VERSION}.npy",
    "room_label_mapping": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_room_label_mapping_{VERSION}.json",
    "summary": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_summary_{VERSION}.json",
    "validation_results": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_validation_results_{VERSION}.json",
}

STEP29B2_INPUTS = {
    "gateway_candidates": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_candidates_{VERSION}.json",
    "gateway_graph": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_graph_{VERSION}.json",
    "gateway_extraction_summary": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_extraction_summary_{VERSION}.json",
    "gateway_candidate_validation_results": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_candidate_validation_results_{VERSION}.json",
    "room_pair_adjacency_candidates": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_room_pair_adjacency_candidates_{VERSION}.json",
    "rejected_room_edges": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_rejected_room_edges_{VERSION}.json",
    "label10_and_wall_overlap_audit": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_label10_and_wall_overlap_audit_{VERSION}.json",
    "readme": STEP29B2_ROOT / "README_step29b2.md",
}

REVIEW_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_review_{VERSION}.json"
SUMMARY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_summary_{VERSION}.json"
VALIDATION_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_validation_results_{VERSION}.json"
KEY_PAIR_REVIEW_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_key_pair_relaxed_candidate_review_{VERSION}.json"
WALL_LAYER_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_wall_layer_sensitivity_{VERSION}.json"
LABEL10_UNKNOWN_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_label10_unknown_gateway_impact_{VERSION}.json"
RECLASSIFIED_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_reclassified_rejected_candidates_{VERSION}.json"
PUBLIC_PATH_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_public_path_042_sensitivity_readiness_{VERSION}.json"

KEY_ROOM_PAIRS = [(7, 11), (11, 8), (3, 7), (3, 11), (1, 3)]
PUBLIC_PATH_042 = [1, 3, 7, 11, 8]
PUBLIC_PATH_TRANSITIONS = list(zip(PUBLIC_PATH_042[:-1], PUBLIC_PATH_042[1:]))
INTERIOR_EROSION_RADII_M = [0.05, 0.10, 0.15, 0.20]
BOUNDARY_DILATION_RADII_M = [0.15, 0.25, 0.40, 0.60, 0.80]
WALL_POLICY_NAMES = [
    "strict_current",
    "eroded_wall_1cell",
    "eroded_wall_2cell",
    "wall_ignored_diagnostic",
    "free_space_only_connectivity",
    "full_map_post_doors_reference_check",
]
FORBIDDEN_OUTPUT_NAMES = {
    "gateway_augmented_topology_v0_1.json",
    "nav2_goals_v0_1.json",
    "route_waypoints_v0_1.json",
    f"{SHORT_SCENE_ID}_gateway_candidates_{VERSION}.json",
}
SOURCE_LAYERS_USED = [
    "room_mask_global_id",
    "structural_wall",
    "free_space",
    "outside_boundary",
    "unknown_layer",
    "full_map_post_doors_reference",
    "room_mask_local_label_repaired_label10_only",
]
FILES_NOT_USED = [
    "room polygons",
    "old BEV",
    "selected_public_route_edges_carved_w0p6",
    "topology JSON edges as geometry",
    "Nav2 outputs",
    "Gazebo outputs",
    "AMCL/TF/DWB/ROS execution artifacts",
    "Step24H artifacts",
    "Step25 artifacts",
]


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_ready(payload), indent=2, sort_keys=False) + "\n")
    print(f"Written: {rel(path)}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pair_key(pair: Tuple[int, int]) -> str:
    a, b = normalize_pair(*pair)
    return f"room_{a}<->room_{b}"


def display_pair_dict(pair: Tuple[int, int]) -> Dict[str, int]:
    a, b = normalize_pair(*pair)
    return {"room_a": a, "room_b": b}


def pose_to_row_col(pose: Dict[str, Any]) -> Tuple[float, float]:
    return (float(pose["y"]) - ORIGIN[1]) / RESOLUTION, (float(pose["x"]) - ORIGIN[0]) / RESOLUTION


def candidate_mask(shape: Tuple[int, int], candidate: Dict[str, Any]) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    r0, c0, _r1, _c1 = [int(v) for v in candidate["debug_component_rle"]["bbox_cells"]]
    for rr, cc, length in candidate["debug_component_rle"]["runs_row_offset_col_offset_length"]:
        row = r0 + int(rr)
        col0 = c0 + int(cc)
        col1 = col0 + int(length)
        if 0 <= row < shape[0] and col1 > 0 and col0 < shape[1]:
            mask[row, max(0, col0) : min(shape[1], col1)] = True
    return mask


def bbox_map_xy_from_cells(bbox_cells: Sequence[int]) -> List[float]:
    r0, c0, r1, c1 = [float(v) for v in bbox_cells]
    xy0 = grid_to_xy(r0, c0)
    xy1 = grid_to_xy(r1, c1)
    return [xy0["x"], xy0["y"], xy1["x"], xy1["y"]]


def component_width_m(mask: np.ndarray) -> float:
    if not np.any(mask):
        return 0.0
    r0, c0, r1, c1 = bbox_for_mask(mask)
    return round(float(min(r1 - r0 + 1, c1 - c0 + 1) * RESOLUTION), 4)


def load_inputs() -> Dict[str, Any]:
    missing = [rel(path) for path in list(STEP29B1_INPUTS.values()) + list(STEP29B2_INPUTS.values()) if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"Missing Step29B2R inputs: {missing}")
    npz = np.load(STEP29B1_INPUTS["layered_bev_npz"])
    global_room = npz["room_mask_global_id"].astype(np.int32)
    global_room_npy = np.load(STEP29B1_INPUTS["global_room_mask"]).astype(np.int32)
    if global_room.shape != global_room_npy.shape or not np.array_equal(global_room, global_room_npy):
        raise ValueError("room_mask_global_id in NPZ does not match 00824_global_room_mask_v0_1.npy")
    return {
        "step29b1_validation": read_json(STEP29B1_INPUTS["validation_results"]),
        "step29b1_summary": read_json(STEP29B1_INPUTS["summary"]),
        "step29b2_validation": read_json(STEP29B2_INPUTS["gateway_candidate_validation_results"]),
        "step29b2_summary": read_json(STEP29B2_INPUTS["gateway_extraction_summary"]),
        "step29b2_candidates_payload": read_json(STEP29B2_INPUTS["gateway_candidates"]),
        "step29b2_adjacency": read_json(STEP29B2_INPUTS["room_pair_adjacency_candidates"]),
        "step29b2_rejected": read_json(STEP29B2_INPUTS["rejected_room_edges"]),
        "step29b2_label10_audit": read_json(STEP29B2_INPUTS["label10_and_wall_overlap_audit"]),
        "npz": npz,
        "global_room": global_room,
        "structural_wall": npz["structural_wall"] > 0,
        "free_space": npz["free_space"] > 0,
        "outside_boundary": npz["outside_boundary"] > 0,
        "unknown_layer": npz["unknown_layer"] > 0,
        "full_map_post_doors_reference": npz["full_map_post_doors_reference"] > 0,
        "local_label10": npz["room_mask_local_label_repaired"].astype(np.int32) == 10,
    }


def wall_policies(wall: np.ndarray) -> Dict[str, np.ndarray]:
    return {
        "strict_current": wall.astype(bool, copy=True),
        "eroded_wall_1cell": erode_disk(wall, 1),
        "eroded_wall_2cell": erode_disk(wall, 2),
        "wall_ignored_diagnostic": np.zeros(wall.shape, dtype=bool),
    }


def evaluate_candidate_connectivity(
    candidate: Dict[str, Any],
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    outside: np.ndarray,
    unknown: np.ndarray,
    label10: np.ndarray,
    *,
    interior_erosion_radius_m: float = 0.15,
    free_near_radius_m: float = 0.10,
    unknown_permissive: bool = False,
    label10_weak: bool = False,
    free_space_only: bool = False,
) -> Dict[str, Any]:
    room_a, room_b = normalize_pair(int(candidate["room_a"]), int(candidate["room_b"]))
    comp = candidate_mask(global_room.shape, candidate)
    comp_coords = np.argwhere(comp)
    room_a_mask = global_room == room_a
    room_b_mask = global_room == room_b
    other_room = (global_room > 0) & (global_room != room_a) & (global_room != room_b)
    interior_a = erode_disk(room_a_mask, meters_to_cells(interior_erosion_radius_m))
    interior_b = erode_disk(room_b_mask, meters_to_cells(interior_erosion_radius_m))
    if not np.any(interior_a):
        interior_a = room_a_mask
    if not np.any(interior_b):
        interior_b = room_b_mask
    free_near = shifted_or(free, meters_to_cells(free_near_radius_m))
    label10_near = shifted_or(label10, 1) if label10_weak else np.zeros(label10.shape, dtype=bool)
    if free_space_only:
        allowed = outside & ~other_room & (free_near | room_a_mask | room_b_mask | comp)
    else:
        allowed = outside & ~wall & ~other_room & (free_near | room_a_mask | room_b_mask | comp | label10_near)
    if not unknown_permissive:
        allowed &= ~unknown
    connects_a, connects_b, reachable_cells = reachable_room_interiors(comp_coords, allowed, interior_a, interior_b)
    context = shifted_or(comp, meters_to_cells(0.15))
    context_count = max(1, int(np.count_nonzero(context)))
    comp_count = max(1, int(np.count_nonzero(comp)))
    return {
        "connects_room_a": bool(connects_a),
        "connects_room_b": bool(connects_b),
        "two_sided_connectivity": bool(connects_a and connects_b),
        "reachable_cell_count": int(reachable_cells),
        "interior_erosion_radius_m": float(interior_erosion_radius_m),
        "unknown_permissive": bool(unknown_permissive),
        "label10_weak": bool(label10_weak),
        "free_space_only": bool(free_space_only),
        "width_m": float(candidate.get("width_m", component_width_m(comp))),
        "component_cell_count": int(np.count_nonzero(comp)),
        "free_fraction_component": round(float(np.count_nonzero(comp & free) / comp_count), 6),
        "free_fraction_context": round(float(np.count_nonzero(context & free) / context_count), 6),
        "unknown_fraction_context": round(float(np.count_nonzero(context & unknown) / context_count), 6),
        "label10_fraction_context": round(float(np.count_nonzero(context & label10) / context_count), 6),
        "wall_fraction_context": round(float(np.count_nonzero(context & wall) / context_count), 6),
        "outside_fraction_context": round(float(np.count_nonzero(context & outside) / context_count), 6),
    }


def diagnostic_extract_pair(
    pair: Tuple[int, int],
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    outside: np.ndarray,
    unknown: np.ndarray,
    label10: np.ndarray,
    *,
    interior_erosion_radius_m: float,
    boundary_dilation_radius_m: float,
    unknown_permissive: bool = False,
    label10_weak: bool = False,
    free_space_only: bool = False,
) -> Dict[str, Any]:
    room_a, room_b = normalize_pair(*pair)
    if not np.any(global_room == room_a) or not np.any(global_room == room_b):
        return {
            "room_a": room_a,
            "room_b": room_b,
            "raw_component_count": 0,
            "two_sided_component_count": 0,
            "max_two_sided_width_m": 0.0,
            "max_component_width_m": 0.0,
            "near_boundary_detected": False,
            "component_summaries": [],
        }

    rs, cs = crop_slices_for_rooms(global_room, [room_a, room_b], margin=meters_to_cells(1.25))
    row_offset = int(rs.start or 0)
    col_offset = int(cs.start or 0)
    g = global_room[rs, cs]
    wall_c = wall[rs, cs]
    free_c = free[rs, cs]
    outside_c = outside[rs, cs]
    unknown_c = unknown[rs, cs]
    label10_c = label10[rs, cs]
    room_a_mask = g == room_a
    room_b_mask = g == room_b
    other_room = (g > 0) & (g != room_a) & (g != room_b)
    interior_a = erode_disk(room_a_mask, meters_to_cells(interior_erosion_radius_m))
    interior_b = erode_disk(room_b_mask, meters_to_cells(interior_erosion_radius_m))
    if not np.any(interior_a):
        interior_a = room_a_mask
    if not np.any(interior_b):
        interior_b = room_b_mask
    band = build_boundary_band(room_a_mask, room_b_mask, meters_to_cells(boundary_dilation_radius_m), outside_c)
    near_boundary = bool(np.any(build_boundary_band(room_a_mask, room_b_mask, meters_to_cells(0.60), outside_c)))
    free_near = shifted_or(free_c, meters_to_cells(0.10))
    label10_near = shifted_or(label10_c, 1) if label10_weak else np.zeros(label10_c.shape, dtype=bool)
    if free_space_only:
        traversable = band & outside_c & ~other_room & (free_near | room_a_mask | room_b_mask)
        allowed = outside_c & ~other_room & (free_near | room_a_mask | room_b_mask | label10_near)
    else:
        traversable = band & outside_c & ~wall_c & ~other_room & (free_near | room_a_mask | room_b_mask | label10_near)
        allowed = outside_c & ~wall_c & ~other_room & (free_near | room_a_mask | room_b_mask | label10_near)
    if not unknown_permissive:
        traversable &= ~unknown_c
        allowed &= ~unknown_c
    comps = connected_components(traversable, min_cells=3)
    summaries: List[Dict[str, Any]] = []
    for idx, comp in enumerate(comps, start=1):
        comp_mask = np.zeros(g.shape, dtype=bool)
        comp_mask[comp[:, 0], comp[:, 1]] = True
        allowed_with_comp = allowed | comp_mask
        connects_a, connects_b, reachable_cells = reachable_room_interiors(comp, allowed_with_comp, interior_a, interior_b)
        r0, c0, r1, c1 = bbox_for_mask(comp_mask)
        width_m = round(float(min(r1 - r0 + 1, c1 - c0 + 1) * RESOLUTION), 4)
        context = shifted_or(comp_mask, meters_to_cells(0.15))
        summaries.append(
            {
                "index": idx,
                "bbox_cells": [r0 + row_offset, c0 + col_offset, r1 + row_offset, c1 + col_offset],
                "bbox_map_xy": bbox_map_xy_from_cells([r0 + row_offset, c0 + col_offset, r1 + row_offset, c1 + col_offset]),
                "cell_count": int(comp.shape[0]),
                "width_m": width_m,
                "connects_room_a": bool(connects_a),
                "connects_room_b": bool(connects_b),
                "two_sided_connectivity": bool(connects_a and connects_b),
                "reachable_cell_count": int(reachable_cells),
                "free_fraction": round(float(np.count_nonzero(comp_mask & free_c) / max(1, comp.shape[0])), 6),
                "unknown_fraction_context": round(float(np.count_nonzero(context & unknown_c) / max(1, np.count_nonzero(context))), 6),
                "label10_fraction_context": round(float(np.count_nonzero(context & label10_c) / max(1, np.count_nonzero(context))), 6),
            }
        )
    two_sided = [s for s in summaries if s["two_sided_connectivity"]]
    return {
        "room_a": room_a,
        "room_b": room_b,
        "near_boundary_detected": near_boundary,
        "interior_erosion_radius_m": float(interior_erosion_radius_m),
        "boundary_dilation_radius_m": float(boundary_dilation_radius_m),
        "unknown_permissive": bool(unknown_permissive),
        "label10_weak": bool(label10_weak),
        "free_space_only": bool(free_space_only),
        "raw_component_count": len(summaries),
        "two_sided_component_count": len(two_sided),
        "max_component_width_m": max([s["width_m"] for s in summaries], default=0.0),
        "max_two_sided_width_m": max([s["width_m"] for s in two_sided], default=0.0),
        "wide_two_sided_component_count": sum(1 for s in two_sided if float(s["width_m"]) >= 0.35),
        "component_summaries": summaries[:12],
    }


def candidate_entries_for_pair(candidates: Sequence[Dict[str, Any]], pair: Tuple[int, int]) -> List[Dict[str, Any]]:
    want = normalize_pair(*pair)
    return [c for c in candidates if normalize_pair(int(c["room_a"]), int(c["room_b"])) == want]


def strict_pair_baseline(adjacency: Dict[str, Any], pair: Tuple[int, int]) -> Dict[str, Any]:
    want = normalize_pair(*pair)
    for entry in adjacency["room_pair_adjacency_candidates"]:
        if normalize_pair(int(entry["room_a"]), int(entry["room_b"])) == want:
            return entry
    a, b = want
    return {
        "room_a": a,
        "room_b": b,
        "near_boundary_detected": False,
        "raw_candidate_count": 0,
        "valid_candidate_count": 0,
        "ambiguous_candidate_count": 0,
        "misleading_candidate_count": 0,
        "rejected_candidate_count": 0,
        "selected_gateway_ids": [],
        "notes": "not present in Step29B2 adjacency artifact",
    }


def summarize_policy_candidates(candidates: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    counts = {status: sum(1 for c in candidates if c.get("status") == status) for status in ["valid", "ambiguous", "misleading", "rejected"]}
    two_sided = [c for c in candidates if c.get("validation", {}).get("two_sided_connectivity")]
    return {
        "raw_candidate_count": len(candidates),
        "valid_candidate_count": counts["valid"],
        "ambiguous_candidate_count": counts["ambiguous"],
        "misleading_candidate_count": counts["misleading"],
        "rejected_candidate_count": counts["rejected"],
        "two_sided_candidate_count": len(two_sided),
        "wide_two_sided_candidate_count": sum(1 for c in two_sided if float(c.get("width_m", 0.0)) >= 0.35),
        "max_width_m": max([float(c.get("width_m", 0.0)) for c in candidates], default=0.0),
        "max_two_sided_width_m": max([float(c.get("width_m", 0.0)) for c in two_sided], default=0.0),
        "diagnostic_candidate_ids": [c.get("gateway_id") for c in candidates[:12]],
    }


def reclassify_candidate(candidate: Dict[str, Any], sensitivity: Dict[str, Any]) -> Tuple[str, str, str]:
    strict = candidate.get("validation", {})
    strict_two_sided = bool(strict.get("two_sided_connectivity"))
    width = float(candidate.get("width_m", 0.0))
    free_frac = float(strict.get("free_fraction", 0.0))
    label10_frac = float(strict.get("label10_fraction", 0.0))
    unknown_frac = float(strict.get("unknown_fraction", 0.0))
    mild_wall = sensitivity["candidate_wall_policy_results"].get("eroded_wall_1cell", {})
    eroded2 = sensitivity["candidate_wall_policy_results"].get("eroded_wall_2cell", {})
    unknown_perm = sensitivity["unknown_label10_results"].get("unknown_permissive", {})
    label10_weak = sensitivity["unknown_label10_results"].get("label10_weak", {})
    interior_any = any(v.get("two_sided_connectivity") for v in sensitivity["interior_erosion_results"].values())

    if candidate.get("status") == "valid" and candidate.get("selected_for_topology"):
        return "strict_valid_existing", "strict selected gateway remains the only topology-level evidence", "keep_rejected"
    if strict_two_sided and width < 0.35:
        return "too_narrow_candidate", "two-sided in strict or near-strict connectivity but below 0.35m width threshold", "inspect_png"
    if not strict_two_sided and (mild_wall.get("two_sided_connectivity") or eroded2.get("two_sided_connectivity")) and width >= 0.18 and free_frac >= 0.20:
        return "wall_blocked_candidate", "two-sided connectivity appears only after wall erosion; diagnostic only", "consider_wall_layer_adjustment"
    if not strict_two_sided and (unknown_perm.get("two_sided_connectivity") or label10_weak.get("two_sided_connectivity")):
        return "unknown_or_label10_blocked_candidate", "connectivity depends on unknown or local label10 permissiveness", "consider_room_mask_boundary_adjustment"
    if not strict_two_sided and interior_any and width >= 0.18 and free_frac >= 0.20:
        return "relaxed_two_sided_candidate", "two-sided only under interior erosion sensitivity; not promoted", "inspect_png"
    if not strict_two_sided and width >= 0.25 and free_frac >= 0.70:
        return "one_sided_but_gateway_like", "large/free component reaches one side only and is plausible enough for review", "inspect_png"
    if label10_frac >= 0.30 or unknown_frac >= 0.05:
        return "unknown_or_label10_blocked_candidate", "strict rejection occurs in a label10/unknown-heavy local context", "inspect_png"
    if free_frac < 0.20 and width < 0.20:
        return "likely_false_positive", "little free-space support and no robust two-sided connectivity", "keep_rejected"
    return "hard_rejected", "no two-sided connectivity under tested diagnostic settings", "keep_rejected"


def interpretation_for_pair(
    pair: Tuple[int, int],
    strict: Dict[str, Any],
    reclasses: Sequence[Dict[str, Any]],
    wall_results: Dict[str, Any],
    grid_results: Sequence[Dict[str, Any]],
) -> Tuple[str, str, str]:
    norm = normalize_pair(*pair)
    if strict.get("selected_gateway_ids"):
        return "strict_ready", "keep_rejected", "strict selected gateway exists and positive-control evidence is stable"
    mild_two_sided = bool(wall_results.get("eroded_wall_1cell", {}).get("wide_two_sided_candidate_count", 0))
    eroded2_two_sided = bool(wall_results.get("eroded_wall_2cell", {}).get("wide_two_sided_candidate_count", 0))
    wall_ignored_two_sided = bool(wall_results.get("wall_ignored_diagnostic", {}).get("wide_two_sided_candidate_count", 0))
    free_only_two_sided = any(
        r.get("free_space_only") and r.get("wide_two_sided_component_count", 0) > 0 for r in grid_results
    )
    labels = {r["diagnostic_reclassification"] for r in reclasses}
    if norm == normalize_pair(3, 11):
        if mild_two_sided:
            return "manual_review_needed", "inspect_png", "room_3<->room_11 appears under mild relaxation but must not be revived automatically"
        if wall_ignored_two_sided:
            return "strict_absent", "keep_rejected", "only aggressive diagnostic settings suggest a direct edge; keep false-connectivity guard"
        return "strict_absent", "keep_rejected", "no robust direct room_3<->room_11 gateway evidence under mild relaxation"
    if "wall_blocked_candidate" in labels or mild_two_sided or eroded2_two_sided:
        return "likely_wall_overblocked", "consider_wall_layer_adjustment", "candidate evidence is sensitive to structural_wall erosion"
    if "unknown_or_label10_blocked_candidate" in labels:
        return "likely_unknown_or_label10_gap", "consider_room_mask_boundary_adjustment", "candidate evidence depends on label10/unknown context"
    if labels & {"one_sided_but_gateway_like", "relaxed_two_sided_candidate", "too_narrow_candidate"}:
        return "manual_review_needed", "inspect_png", "gateway-like evidence exists but is not topology-safe"
    if free_only_two_sided:
        return "manual_review_needed", "inspect_png", "free-space-only connectivity suggests a possible missed gateway without wall support"
    if wall_ignored_two_sided:
        return "manual_review_needed", "inspect_png", "connectivity appears only when wall evidence is ignored"
    return "likely_true_missing_gateway", "route_not_supported", "no reviewable gateway evidence survived the diagnostic checks"


def evaluate_reference_band(
    pair: Tuple[int, int],
    global_room: np.ndarray,
    structural_wall: np.ndarray,
    full_map_reference: np.ndarray,
    outside: np.ndarray,
) -> Dict[str, Any]:
    room_a, room_b = normalize_pair(*pair)
    band = build_boundary_band(global_room == room_a, global_room == room_b, meters_to_cells(0.40), outside)
    count = max(1, int(np.count_nonzero(band)))
    wall_in_band = structural_wall & band
    return {
        "room_a": room_a,
        "room_b": room_b,
        "reference_layer": "full_map_post_doors_reference",
        "authority": "reference_only_not_used_as_gateway_geometry",
        "band_cell_count": int(np.count_nonzero(band)),
        "structural_wall_fraction_in_band": round(float(np.count_nonzero(wall_in_band) / count), 6),
        "full_map_nonzero_fraction_in_band": round(float(np.count_nonzero(full_map_reference & band) / count), 6),
        "full_map_zero_fraction_in_band": round(float(np.count_nonzero((~full_map_reference) & band) / count), 6),
        "wall_cells_with_full_map_nonzero_fraction": round(
            float(np.count_nonzero(wall_in_band & full_map_reference) / max(1, int(np.count_nonzero(wall_in_band)))), 6
        ),
        "interpretation": "reported as agreement/disagreement context only; no topology promotion is allowed",
    }


def review_step29b2r() -> Dict[str, Any]:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    inputs = load_inputs()
    global_room = inputs["global_room"]
    structural_wall = inputs["structural_wall"]
    free = inputs["free_space"]
    outside = inputs["outside_boundary"]
    unknown = inputs["unknown_layer"]
    label10 = inputs["local_label10"]
    full_map_reference = inputs["full_map_post_doors_reference"]
    candidates = inputs["step29b2_candidates_payload"]["gateway_candidates"]
    policies = wall_policies(structural_wall)

    b2_input_hashes = {name: sha256_file(path) for name, path in STEP29B2_INPUTS.items()}
    pair_reviews: List[Dict[str, Any]] = []
    candidate_reviews: List[Dict[str, Any]] = []
    wall_layer_pair_results: Dict[str, Any] = {}
    label10_unknown_pair_results: Dict[str, Any] = {}

    for pair in KEY_ROOM_PAIRS:
        strict = strict_pair_baseline(inputs["step29b2_adjacency"], pair)
        pair_candidates = candidate_entries_for_pair(candidates, pair)
        wall_results: Dict[str, Any] = {}
        for policy_name in ["strict_current", "eroded_wall_1cell", "eroded_wall_2cell", "wall_ignored_diagnostic"]:
            extracted, _summary = extract_pair_candidates(
                normalize_pair(*pair),
                global_room,
                policies[policy_name],
                free,
                outside,
                unknown,
                inputs["npz"]["room_mask_local_label_repaired"].astype(np.int32),
                policies[policy_name] & (global_room > 0),
            )
            policy_summary = summarize_policy_candidates(extracted)
            policy_summary["policy"] = policy_name
            policy_summary["promotion_allowed"] = policy_name == "strict_current"
            if policy_name != "strict_current":
                policy_summary["promotion_allowed"] = False
                policy_summary["diagnostic_only_reason"] = "relaxed wall policy sensitivity result"
            wall_results[policy_name] = policy_summary

        free_space_only = diagnostic_extract_pair(
            pair,
            global_room,
            np.zeros_like(structural_wall, dtype=bool),
            free,
            outside,
            unknown,
            label10,
            interior_erosion_radius_m=0.15,
            boundary_dilation_radius_m=0.40,
            free_space_only=True,
        )
        wall_results["free_space_only_connectivity"] = {
            "policy": "free_space_only_connectivity",
            "promotion_allowed": False,
            "diagnostic_only_reason": "free-space-only connectivity ignores structural_wall validity",
            "raw_component_count": free_space_only["raw_component_count"],
            "two_sided_component_count": free_space_only["two_sided_component_count"],
            "wide_two_sided_component_count": free_space_only["wide_two_sided_component_count"],
            "max_two_sided_width_m": free_space_only["max_two_sided_width_m"],
        }
        wall_results["full_map_post_doors_reference_check"] = evaluate_reference_band(
            pair, global_room, structural_wall, full_map_reference, outside
        )
        wall_layer_pair_results[pair_key(pair)] = wall_results

        grid_results: List[Dict[str, Any]] = []
        for interior_radius in INTERIOR_EROSION_RADII_M:
            for boundary_radius in BOUNDARY_DILATION_RADII_M:
                grid_results.append(
                    diagnostic_extract_pair(
                        pair,
                        global_room,
                        structural_wall,
                        free,
                        outside,
                        unknown,
                        label10,
                        interior_erosion_radius_m=interior_radius,
                        boundary_dilation_radius_m=boundary_radius,
                    )
                )
        grid_results.append(free_space_only)

        pair_label10_unknown = {
            "room_pair": display_pair_dict(pair),
            "candidate_count": len(pair_candidates),
            "candidate_unknown_fraction_max": max(
                [float(c.get("validation", {}).get("unknown_fraction", 0.0)) for c in pair_candidates], default=0.0
            ),
            "candidate_label10_fraction_max": max(
                [float(c.get("validation", {}).get("label10_fraction", 0.0)) for c in pair_candidates], default=0.0
            ),
            "candidate_free_fraction_max": max(
                [float(c.get("validation", {}).get("free_fraction", 0.0)) for c in pair_candidates], default=0.0
            ),
            "label10_or_unknown_heavy_candidate_ids": [
                c["gateway_id"]
                for c in pair_candidates
                if float(c.get("validation", {}).get("label10_fraction", 0.0)) >= 0.30
                or float(c.get("validation", {}).get("unknown_fraction", 0.0)) >= 0.05
            ],
        }
        label10_unknown_pair_results[pair_key(pair)] = pair_label10_unknown

        pair_reclasses: List[Dict[str, Any]] = []
        for candidate in pair_candidates:
            candidate_wall_results = {
                policy_name: evaluate_candidate_connectivity(
                    candidate, global_room, wall_mask, free, outside, unknown, label10
                )
                for policy_name, wall_mask in policies.items()
            }
            interior_results = {
                f"{radius:.2f}m": evaluate_candidate_connectivity(
                    candidate,
                    global_room,
                    structural_wall,
                    free,
                    outside,
                    unknown,
                    label10,
                    interior_erosion_radius_m=radius,
                )
                for radius in INTERIOR_EROSION_RADII_M
            }
            unknown_label10_results = {
                "unknown_permissive": evaluate_candidate_connectivity(
                    candidate,
                    global_room,
                    structural_wall,
                    free,
                    outside,
                    unknown,
                    label10,
                    unknown_permissive=True,
                ),
                "label10_weak": evaluate_candidate_connectivity(
                    candidate,
                    global_room,
                    structural_wall,
                    free,
                    outside,
                    unknown,
                    label10,
                    label10_weak=True,
                ),
                "unknown_permissive_and_label10_weak": evaluate_candidate_connectivity(
                    candidate,
                    global_room,
                    structural_wall,
                    free,
                    outside,
                    unknown,
                    label10,
                    unknown_permissive=True,
                    label10_weak=True,
                ),
            }
            sensitivity = {
                "candidate_wall_policy_results": candidate_wall_results,
                "interior_erosion_results": interior_results,
                "unknown_label10_results": unknown_label10_results,
            }
            reclassification, reason, action = reclassify_candidate(candidate, sensitivity)
            review = {
                "gateway_id": candidate["gateway_id"],
                "room_a": int(candidate["room_a"]),
                "room_b": int(candidate["room_b"]),
                "width_m": float(candidate.get("width_m", 0.0)),
                "bbox_cells": candidate.get("bbox_cells"),
                "bbox_map_xy": candidate.get("bbox_map_xy"),
                "strict_status": candidate.get("status"),
                "strict_reason": candidate.get("warning_or_rejection_reason"),
                "strict_validation": candidate.get("validation"),
                "candidate_wall_policy_results": candidate_wall_results,
                "interior_erosion_results": interior_results,
                "unknown_label10_results": unknown_label10_results,
                "free_support": {
                    "strict_free_fraction": candidate.get("validation", {}).get("free_fraction"),
                    "component_area_m2": candidate.get("component_area_m2"),
                    "component_cell_count": candidate.get("component_cell_count"),
                },
                "diagnostic_reclassification": reclassification,
                "reclassification_reason": reason,
                "recommended_next_action": action,
                "promotion_allowed": False if candidate.get("status") != "valid" else bool(candidate.get("selected_for_topology")),
            }
            pair_reclasses.append(review)
            candidate_reviews.append(review)

        interpretation, next_action, interpretation_reason = interpretation_for_pair(pair, strict, pair_reclasses, wall_results, grid_results)
        pair_reviews.append(
            {
                "room_pair": display_pair_dict(pair),
                "step29b2_strict_result": strict,
                "raw_candidate_count": strict.get("raw_candidate_count", 0),
                "candidate_count_in_review": len(pair_candidates),
                "wall_policy_sensitivity": wall_results,
                "erosion_dilation_sensitivity": grid_results,
                "any_candidate_two_sided_under_eroded_wall_1cell": any(
                    r["candidate_wall_policy_results"]["eroded_wall_1cell"]["two_sided_connectivity"] for r in pair_reclasses
                ),
                "any_candidate_two_sided_under_eroded_wall_2cell": any(
                    r["candidate_wall_policy_results"]["eroded_wall_2cell"]["two_sided_connectivity"] for r in pair_reclasses
                ),
                "any_candidate_depends_on_unknown_or_label10": any(
                    r["unknown_label10_results"]["unknown_permissive"]["two_sided_connectivity"]
                    or r["unknown_label10_results"]["label10_weak"]["two_sided_connectivity"]
                    or float(r["strict_validation"].get("label10_fraction", 0.0)) >= 0.30
                    or float(r["strict_validation"].get("unknown_fraction", 0.0)) >= 0.05
                    for r in pair_reclasses
                ),
                "free_space_support_summary": {
                    "max_candidate_free_fraction": label10_unknown_pair_results[pair_key(pair)]["candidate_free_fraction_max"],
                    "free_space_only_connectivity": wall_results["free_space_only_connectivity"],
                },
                "candidate_reclassifications": pair_reclasses,
                "recommended_interpretation": interpretation,
                "recommended_next_action": next_action,
                "interpretation_reason": interpretation_reason,
            }
        )

    public_path = public_path_readiness(pair_reviews)
    summary = build_summary(inputs, pair_reviews, candidate_reviews, public_path)
    provenance = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2r_gateway_sensitivity_review",
        "version": VERSION,
        "output_directory": rel(STEP29B2R_ROOT),
        "source_step29b1_artifacts": {name: rel(path) for name, path in STEP29B1_INPUTS.items()},
        "source_step29b2_artifacts": {name: rel(path) for name, path in STEP29B2_INPUTS.items()},
        "source_step29b2_sha256_before_review": b2_input_hashes,
        "room_identity_source": "room_mask_global_id",
        "room_mask_global_id_used": True,
        "local_labels_used_as_room_ids": False,
        "local_label_usage": "room_mask_local_label_repaired was used only to analyze local label 10 impact.",
        "room_polygons_used": False,
        "old_bev_used": False,
        "selected_public_route_edges_carved_w0p6_used": False,
        "topology_json_edges_used_as_geometry": False,
        "topology_augmented": False,
        "step29b2_overwritten": False,
        "nav2_run": False,
        "gazebo_run": False,
        "ros_execution_run": False,
        "amcl_tf_dwb_run": False,
        "step24h_or_step25_work_done": False,
        "source_layers_used": SOURCE_LAYERS_USED,
        "files_not_used": FILES_NOT_USED,
        "wall_policies_evaluated": WALL_POLICY_NAMES,
        "interior_erosion_radii_m": INTERIOR_EROSION_RADII_M,
        "boundary_dilation_radii_m": BOUNDARY_DILATION_RADII_M,
        "diagnostic_only": True,
        "relaxed_results_promoted_to_topology": False,
    }
    review_payload = {
        **provenance,
        "step29b2_strict_baseline_summary": strict_baseline_summary(inputs["step29b2_summary"]),
        "key_room_pairs": [display_pair_dict(p) for p in KEY_ROOM_PAIRS],
        "public_path_042": PUBLIC_PATH_042,
        "pair_reviews": pair_reviews,
        "candidate_reviews": candidate_reviews,
        "public_path_042_sensitivity_readiness": public_path,
        "summary": summary,
    }
    wall_payload = {
        **provenance,
        "artifact_type": "step29b2r_wall_layer_sensitivity",
        "wall_layer_sensitivity_by_pair": wall_layer_pair_results,
    }
    label_payload = {
        **provenance,
        "artifact_type": "step29b2r_label10_unknown_gateway_impact",
        "label10_unknown_gateway_impact_by_pair": label10_unknown_pair_results,
        "candidate_label10_unknown_reviews": [
            {
                "gateway_id": c["gateway_id"],
                "room_a": c["room_a"],
                "room_b": c["room_b"],
                "strict_unknown_fraction": c["strict_validation"].get("unknown_fraction"),
                "strict_label10_fraction": c["strict_validation"].get("label10_fraction"),
                "strict_free_fraction": c["strict_validation"].get("free_fraction"),
                "unknown_permissive_two_sided": c["unknown_label10_results"]["unknown_permissive"]["two_sided_connectivity"],
                "label10_weak_two_sided": c["unknown_label10_results"]["label10_weak"]["two_sided_connectivity"],
                "diagnostic_reclassification": c["diagnostic_reclassification"],
            }
            for c in candidate_reviews
        ],
    }
    reclass_payload = {
        **provenance,
        "artifact_type": "step29b2r_reclassified_rejected_candidates",
        "reclassification_policy": "diagnostic labels only; Step29B2 candidates are not modified and no topology is selected",
        "reclassified_candidates": candidate_reviews,
        "reclassification_counts": count_reclassifications(candidate_reviews),
    }
    key_pair_payload = {
        **provenance,
        "artifact_type": "step29b2r_key_pair_relaxed_candidate_review",
        "key_pair_reviews": pair_reviews,
    }
    public_payload = {
        **provenance,
        "artifact_type": "step29b2r_public_path_042_sensitivity_readiness",
        "public_path_042_sensitivity_readiness": public_path,
    }
    summary_payload = {
        **provenance,
        "artifact_type": "step29b2r_gateway_sensitivity_summary",
        **summary,
        "review_json": rel(REVIEW_JSON),
        "reclassification_json": rel(RECLASSIFIED_JSON),
        "validation_results_json": rel(VALIDATION_JSON),
        "readme_path": rel(README_PATH),
        "visualization_directory": rel(VIS_DIR),
    }

    write_json(REVIEW_JSON, review_payload)
    write_json(SUMMARY_JSON, summary_payload)
    write_json(KEY_PAIR_REVIEW_JSON, key_pair_payload)
    write_json(WALL_LAYER_JSON, wall_payload)
    write_json(LABEL10_UNKNOWN_JSON, label_payload)
    write_json(RECLASSIFIED_JSON, reclass_payload)
    write_json(PUBLIC_PATH_JSON, public_payload)
    return summary_payload


def strict_baseline_summary(step29b2_summary: Dict[str, Any]) -> Dict[str, Any]:
    keys = [
        "raw_candidate_count",
        "valid_candidate_count",
        "ambiguous_candidate_count",
        "misleading_candidate_count",
        "rejected_candidate_count",
        "selected_gateway_count",
        "selected_gateway_ids",
        "public_path_042_gateway_readiness",
        "step29c_topology_augmentation_can_proceed",
    ]
    return {key: step29b2_summary.get(key) for key in keys}


def count_reclassifications(candidate_reviews: Sequence[Dict[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for review in candidate_reviews:
        label = str(review["diagnostic_reclassification"])
        counts[label] = counts.get(label, 0) + 1
    return dict(sorted(counts.items()))


def public_path_readiness(pair_reviews: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    by_pair = {normalize_pair(r["room_pair"]["room_a"], r["room_pair"]["room_b"]): r for r in pair_reviews}
    transitions: List[Dict[str, Any]] = []
    route_ready = True
    has_manual = False
    for room_a, room_b in PUBLIC_PATH_TRANSITIONS:
        review = by_pair.get(normalize_pair(room_a, room_b))
        if review is None:
            status = "still_missing"
            evidence = "not_evaluated"
            reason = "transition missing from Step29B2R review"
        elif review["step29b2_strict_result"].get("selected_gateway_ids"):
            status = "strict_ready"
            evidence = "strict_selected_gateway"
            reason = "Step29B2 selected a gateway for this transition"
        elif review["recommended_interpretation"] in {"likely_wall_overblocked", "likely_unknown_or_label10_gap"}:
            status = "relaxed_candidate_exists"
            evidence = review["recommended_interpretation"]
            reason = review["interpretation_reason"]
            route_ready = False
        elif review["recommended_interpretation"] == "manual_review_needed":
            status = "manual_review_needed"
            evidence = "gateway_like_but_not_topology_safe"
            reason = review["interpretation_reason"]
            route_ready = False
        elif review["recommended_interpretation"] == "strict_absent":
            status = "contradicted_by_evidence" if normalize_pair(room_a, room_b) == normalize_pair(3, 11) else "still_missing"
            evidence = "strict_absent"
            reason = review["interpretation_reason"]
            route_ready = False
        else:
            status = "still_missing"
            evidence = review["recommended_interpretation"]
            reason = review["interpretation_reason"]
            route_ready = False
        if status in {"manual_review_needed", "relaxed_candidate_exists"}:
            has_manual = True
        transitions.append(
            {
                "room_a": room_a,
                "room_b": room_b,
                "diagnostic_status": status,
                "evidence_class": evidence,
                "reason": reason,
                "topology_promotion_allowed": status == "strict_ready",
            }
        )
    if route_ready:
        overall = "strict_ready"
    elif has_manual:
        overall = "manual_review_needed"
    else:
        overall = "still_missing"
    return {
        "route_name": "public_path_042",
        "route": PUBLIC_PATH_042,
        "transitions": transitions,
        "diagnostic_route_status": overall,
        "route_gateway_ready_for_step29c": False,
        "route_gateway_ready_for_step29c_reason": "Step29B2R is diagnostic only and at least one public_path_042 transition lacks strict selected gateway evidence.",
    }


def build_summary(
    inputs: Dict[str, Any],
    pair_reviews: Sequence[Dict[str, Any]],
    candidate_reviews: Sequence[Dict[str, Any]],
    public_path: Dict[str, Any],
) -> Dict[str, Any]:
    conclusions = {pair_key((r["room_pair"]["room_a"], r["room_pair"]["room_b"])): r["recommended_interpretation"] for r in pair_reviews}
    next_actions = {pair_key((r["room_pair"]["room_a"], r["room_pair"]["room_b"])): r["recommended_next_action"] for r in pair_reviews}
    return {
        "scene_id": SCENE_ID,
        "version": VERSION,
        "step29b1_validation_passed": bool(inputs["step29b1_validation"].get("overall_pass")),
        "step29b2_validation_passed": bool(inputs["step29b2_validation"].get("overall_pass")),
        "step29b2_strict_baseline_summary": strict_baseline_summary(inputs["step29b2_summary"]),
        "key_pair_conclusions": conclusions,
        "key_pair_next_actions": next_actions,
        "reclassification_counts": count_reclassifications(candidate_reviews),
        "room_3_room_11_conclusion": conclusions.get(pair_key((3, 11))),
        "room_7_room_11_conclusion": conclusions.get(pair_key((7, 11))),
        "room_11_room_8_conclusion": conclusions.get(pair_key((11, 8))),
        "room_3_room_7_conclusion": conclusions.get(pair_key((3, 7))),
        "room_1_room_3_positive_control_conclusion": conclusions.get(pair_key((1, 3))),
        "public_path_042_sensitivity_readiness": public_path,
        "step29c_recommendation": {
            "can_proceed_from_step29b2r": False,
            "recommendation": "Do not proceed to Step29C for public_path_042 unless strict gateway extraction is tuned or manual gateway hints are explicitly reviewed in a later non-diagnostic step.",
            "keep_step29b2_strict_result": True,
            "tune_gateway_extraction": True,
            "manual_gateway_hint": "consider only after inspecting Step29B2R PNGs for room_7<->room_11, room_11<->room_8, and room_3<->room_7",
            "gateway_level_route_demo": "public_path_042 is not supported by strict Step29B2 gateway evidence",
        },
    }


def main() -> None:
    summary = review_step29b2r()
    print("\nStep29B2R sensitivity review summary")
    print(f"Output directory: {summary['output_directory']}")
    print(f"Sensitivity review JSON: {summary['review_json']}")
    print(f"Reclassification JSON: {rel(RECLASSIFIED_JSON)}")
    print(f"Validation results path: {rel(VALIDATION_JSON)}")
    print(f"README path: {rel(README_PATH)}")
    print(f"Visualization directory: {summary['visualization_directory']}")
    print(f"room_3 <-> room_11 conclusion: {summary['room_3_room_11_conclusion']}")
    print(f"room_7 <-> room_11 conclusion: {summary['room_7_room_11_conclusion']}")
    print(f"room_11 <-> room_8 conclusion: {summary['room_11_room_8_conclusion']}")
    print(f"room_3 <-> room_7 conclusion: {summary['room_3_room_7_conclusion']}")
    print(f"public_path_042 sensitivity readiness: {summary['public_path_042_sensitivity_readiness']['diagnostic_route_status']}")
    print(f"recommendation for Step29C: {summary['step29c_recommendation']['recommendation']}")


if __name__ == "__main__":
    main()
