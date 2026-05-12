#!/usr/bin/env python3
"""
Build Step29B2R2 local gateway inspection pack for scene 00824-Dd4bFSTQ8gi.

This is an offline evidence-pack builder. It reads Step29B1/B2/B2R artifacts,
canonicalizes existing gateway-like candidates, renders local inspection PNGs,
and writes review tables/templates. It does not run ROS/Nav2/Gazebo/AMCL/DWB,
does not re-extract gateways from scratch, and does not generate topology.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from extract_step29b2_gateway_candidates_v1 import (  # noqa: E402
    ORIGIN,
    RESOLUTION,
    SCENE_ID,
    SHORT_SCENE_ID,
    VERSION,
    erode_disk,
    grid_to_xy,
    json_ready,
    normalize_pair,
    shifted_or,
)
from render_step29b2_gateway_debug_v1 import (  # noqa: E402
    add_panel,
    base_visual,
    blend_mask,
    draw_line,
    draw_rect,
    draw_text,
    room_color,
    write_png,
)


STEP_NAME = "step29b2r2"
OUTPUT_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step29b2r2_00824_local_gateway_inspection_pack"
)
ASSET_DIR = OUTPUT_ROOT / "generated" / "assets"
VIS_DIR = OUTPUT_ROOT / "generated" / "visualizations"
README_PATH = OUTPUT_ROOT / "README_step29b2r2.md"

STEP29B1_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step29b1_00824_layered_bev_from_step29a3"
)
STEP29B1_ASSET_DIR = STEP29B1_ROOT / "generated" / "assets"
STEP29B2_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step29b2_00824_gateway_candidates_from_layered_bev"
)
STEP29B2_ASSET_DIR = STEP29B2_ROOT / "generated" / "assets"
STEP29B2R_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step29b2r_00824_gateway_sensitivity_review"
)
STEP29B2R_ASSET_DIR = STEP29B2R_ROOT / "generated" / "assets"

STEP29B1_INPUTS = {
    "layered_bev_json": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.json",
    "layered_bev_npz": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.npz",
    "global_room_mask": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_global_room_mask_{VERSION}.npy",
    "room_label_mapping": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_room_label_mapping_{VERSION}.json",
    "summary": STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_summary_{VERSION}.json",
}
STEP29B2_INPUTS = {
    "gateway_candidates": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_candidates_{VERSION}.json",
    "gateway_graph": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_graph_{VERSION}.json",
    "gateway_extraction_summary": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_extraction_summary_{VERSION}.json",
    "room_pair_adjacency_candidates": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_room_pair_adjacency_candidates_{VERSION}.json",
    "rejected_room_edges": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_rejected_room_edges_{VERSION}.json",
    "label10_and_wall_overlap_audit": STEP29B2_ASSET_DIR / f"{SHORT_SCENE_ID}_label10_and_wall_overlap_audit_{VERSION}.json",
}
STEP29B2R_INPUTS = {
    "gateway_sensitivity_review": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_review_{VERSION}.json",
    "gateway_sensitivity_summary": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_summary_{VERSION}.json",
    "gateway_sensitivity_validation_results": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_sensitivity_validation_results_{VERSION}.json",
    "key_pair_relaxed_candidate_review": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_key_pair_relaxed_candidate_review_{VERSION}.json",
    "label10_unknown_gateway_impact": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_label10_unknown_gateway_impact_{VERSION}.json",
    "public_path_042_sensitivity_readiness": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_public_path_042_sensitivity_readiness_{VERSION}.json",
    "reclassified_rejected_candidates": STEP29B2R_ASSET_DIR / f"{SHORT_SCENE_ID}_reclassified_rejected_candidates_{VERSION}.json",
}

REVIEW_TABLE_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_candidate_review_table_{VERSION}.json"
REVIEW_TABLE_CSV = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_candidate_review_table_{VERSION}.csv"
PAIR_RANKING_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_pair_ranking_{VERSION}.json"
MANUAL_TEMPLATE_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_manual_gateway_review_template_{VERSION}.json"
SCHEMA_INVENTORY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_schema_inventory_{VERSION}.json"
SUMMARY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_summary_{VERSION}.json"
VALIDATION_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_step29b2r2_validation_results_{VERSION}.json"

KEY_PAIRS: List[Tuple[int, int]] = [(7, 11), (11, 8), (3, 7), (3, 11), (1, 3)]
PAIR_KEYS = ["r7_r11", "r8_r11", "r3_r7", "r3_r11", "r1_r3"]
PAIR_DISPLAY = {
    "r7_r11": "room_7 <-> room_11",
    "r8_r11": "room_11 <-> room_8",
    "r3_r7": "room_3 <-> room_7",
    "r3_r11": "room_3 <-> room_11",
    "r1_r3": "room_1 <-> room_3",
}

STATUS_COLORS = {
    "valid": (25, 235, 120),
    "ambiguous": (255, 210, 45),
    "misleading": (255, 120, 30),
    "rejected": (240, 45, 70),
    "missing": (170, 170, 170),
    "unknown": (230, 230, 230),
}
ACTION_COLORS = {
    "positive_control_selected": (0, 245, 255),
    "negative_sanity_reject": (255, 80, 80),
    "candidate_for_tuned_extractor": (85, 195, 255),
    "possible_manual_hint": (255, 225, 70),
    "inspect_png": (235, 235, 235),
    "likely_false_positive": (160, 90, 90),
    "keep_rejected": (190, 80, 80),
}
DIAG_LAYER_NAMES = [
    "diagnostic_strict_wall",
    "diagnostic_eroded_wall_1cell",
    "diagnostic_eroded_wall_2cell",
    "diagnostic_free_space_only",
    "diagnostic_label10_unknown",
    "diagnostic_wall_room_overlap",
]


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


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


def pair_key_from_rooms(room_a: int, room_b: int) -> str:
    a, b = normalize_pair(int(room_a), int(room_b))
    return f"r{a}_r{b}"


def room_pair_label(room_a: int, room_b: int) -> str:
    a, b = normalize_pair(int(room_a), int(room_b))
    return f"room_{a}<->room_{b}"


def xy_to_rc(x: float, y: float) -> List[int]:
    col = int(round((float(x) - ORIGIN[0]) / RESOLUTION))
    row = int(round((float(y) - ORIGIN[1]) / RESOLUTION))
    return [row, col]


def bbox_map_xy_from_cells(bbox_cells: Sequence[int]) -> List[List[float]]:
    r0, c0, r1, c1 = [float(v) for v in bbox_cells]
    xy0 = grid_to_xy(r0, c0)
    xy1 = grid_to_xy(r1, c1)
    return [[xy0["x"], xy0["y"]], [xy1["x"], xy1["y"]]]


def numeric(value: Any, default: Optional[float] = None) -> Optional[float]:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def bool_or_none(value: Any) -> Optional[bool]:
    if value is None:
        return None
    return bool(value)


def schema_summary(value: Any, depth: int = 0) -> Any:
    if depth >= 4:
        return type(value).__name__
    if isinstance(value, dict):
        return {
            key: schema_summary(value[key], depth + 1)
            for key in list(value.keys())[:30]
        }
    if isinstance(value, list):
        return {
            "type": "list",
            "length": len(value),
            "first": schema_summary(value[0], depth + 1) if value else None,
        }
    return type(value).__name__


def inspect_schema_inventory(payloads: Dict[str, Any], npz: np.lib.npyio.NpzFile) -> Dict[str, Any]:
    inventory: Dict[str, Any] = {
        "scene_id": SCENE_ID,
        "step": STEP_NAME,
        "note": "Schema inventory is written intentionally before canonicalization; local watershed labels are inventoried but not used as room IDs.",
        "json_files": {},
        "npz_layers": {},
    }
    for name, payload in payloads.items():
        inventory["json_files"][name] = {
            "top_level_keys": list(payload.keys()) if isinstance(payload, dict) else None,
            "schema_sample": schema_summary(payload),
        }
    for key in npz.files:
        arr = npz[key]
        inventory["npz_layers"][key] = {
            "shape": list(arr.shape),
            "dtype": str(arr.dtype),
            "min": int(arr.min()) if np.issubdtype(arr.dtype, np.integer) else float(arr.min()),
            "max": int(arr.max()) if np.issubdtype(arr.dtype, np.integer) else float(arr.max()),
        }
    return inventory


def candidate_component_mask(shape: Tuple[int, int], candidate: Dict[str, Any]) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    rle = candidate.get("debug_component_rle") or {}
    bbox = rle.get("bbox_cells") or candidate.get("bbox_cells")
    runs = rle.get("runs_row_offset_col_offset_length") or []
    if bbox and runs:
        r0, c0, _r1, _c1 = [int(v) for v in bbox]
        for rr, cc, length in runs:
            row = r0 + int(rr)
            col0 = c0 + int(cc)
            col1 = col0 + int(length)
            if 0 <= row < shape[0] and col1 > 0 and col0 < shape[1]:
                mask[row, max(0, col0) : min(shape[1], col1)] = True
        return mask
    if bbox:
        r0, c0, r1, c1 = [int(v) for v in bbox]
        mask[max(0, r0) : min(shape[0], r1 + 1), max(0, c0) : min(shape[1], c1 + 1)] = True
    return mask


def context_slices_for_bbox(
    bbox_cells: Sequence[int],
    shape: Tuple[int, int],
    margin: int = 3,
) -> Tuple[slice, slice]:
    r0, c0, r1, c1 = [int(v) for v in bbox_cells]
    return (
        slice(max(0, r0 - margin), min(shape[0], r1 + margin + 1)),
        slice(max(0, c0 - margin), min(shape[1], c1 + margin + 1)),
    )


def fraction(mask: np.ndarray) -> float:
    if mask.size == 0:
        return 0.0
    return round(float(np.count_nonzero(mask)) / float(mask.size), 6)


def compute_context_metrics(
    candidate: Dict[str, Any],
    layers: Dict[str, np.ndarray],
) -> Dict[str, float]:
    bbox = candidate["bbox_cells"]
    rs, cs = context_slices_for_bbox(bbox, layers["global_room"].shape, margin=3)
    component_mask = candidate_component_mask(layers["global_room"].shape, candidate)
    comp_cells = np.count_nonzero(component_mask)
    free_component = 0.0
    if comp_cells:
        free_component = round(float(np.count_nonzero(component_mask & layers["free"])) / float(comp_cells), 6)
    return {
        "label10_fraction_raw": fraction(layers["local_label_raw"][rs, cs] == 10),
        "label10_fraction_repaired": fraction(layers["local_label_repaired"][rs, cs] == 10),
        "unknown_fraction": fraction(layers["unknown"][rs, cs]),
        "wall_fraction_context": fraction(layers["wall"][rs, cs]),
        "free_fraction_component": free_component,
    }


def index_by_gateway_id(entries: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    result: Dict[str, Dict[str, Any]] = {}
    for entry in entries:
        gid = entry.get("gateway_id")
        if gid is not None:
            result[str(gid)] = entry
    return result


def pair_review_index(review_payload: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    for entry in review_payload.get("pair_reviews", []) or review_payload.get("key_pair_reviews", []):
        pair = entry.get("room_pair", {})
        if "room_a" in pair and "room_b" in pair:
            out[pair_key_from_rooms(int(pair["room_a"]), int(pair["room_b"]))] = entry
    return out


def source_for_candidate(strict: Dict[str, Any], reclass: Optional[Dict[str, Any]]) -> str:
    if strict.get("selected_for_topology") or strict.get("status") in {"valid", "ambiguous"}:
        return "step29b2_strict"
    if reclass:
        return "step29b2r_reclassified"
    return "derived_from_rejected"


def recommend_action(pair_key: str, strict: Dict[str, Any], reclass: Optional[Dict[str, Any]]) -> str:
    status = str(strict.get("status", reclass.get("strict_status") if reclass else "unknown"))
    selected = bool(strict.get("selected_for_topology", False))
    if pair_key == "r3_r11":
        return "negative_sanity_reject"
    if pair_key == "r1_r3" and selected and status == "valid":
        return "positive_control_selected"

    wall_results = (reclass or {}).get("candidate_wall_policy_results", {})
    eroded_1 = bool(wall_results.get("eroded_wall_1cell", {}).get("two_sided_connectivity"))
    eroded_2 = bool(wall_results.get("eroded_wall_2cell", {}).get("two_sided_connectivity"))
    strict_validation = strict.get("validation") or (reclass or {}).get("strict_validation", {})
    free_support = numeric(
        wall_results.get("strict_current", {}).get("free_fraction_component"),
        numeric(strict_validation.get("free_fraction"), None),
    )
    label10 = numeric(strict_validation.get("label10_fraction"), None)
    unknown = numeric(strict_validation.get("unknown_fraction"), None)
    diag = (reclass or {}).get("diagnostic_reclassification")

    if pair_key in {"r7_r11", "r3_r7"} and status == "rejected" and (eroded_1 or eroded_2):
        if free_support is not None and free_support >= 0.65:
            return "candidate_for_tuned_extractor"
        return "possible_manual_hint"
    if pair_key == "r8_r11":
        if (label10 is not None and label10 >= 0.45) or (unknown is not None and unknown >= 0.15):
            if free_support is not None and free_support >= 0.50:
                return "possible_manual_hint"
            return "inspect_png"
        if diag in {"unknown_or_label10_blocked_candidate", "one_sided_but_gateway_like"}:
            return "possible_manual_hint"
    if diag == "hard_rejected":
        return "likely_false_positive"
    if status == "rejected":
        return "inspect_png"
    return "keep_rejected"


def canonicalize_candidates(
    strict_candidates: Sequence[Dict[str, Any]],
    reclassified_by_id: Dict[str, Dict[str, Any]],
    label_reviews_by_id: Dict[str, Dict[str, Any]],
    layers: Dict[str, np.ndarray],
) -> List[Dict[str, Any]]:
    canonical: List[Dict[str, Any]] = []
    for strict in strict_candidates:
        pair_key = pair_key_from_rooms(int(strict["room_a"]), int(strict["room_b"]))
        if pair_key not in PAIR_KEYS:
            continue
        gid = strict["gateway_id"]
        reclass = reclassified_by_id.get(gid)
        label_review = label_reviews_by_id.get(gid, {})
        validation = strict.get("validation", {})
        strict_status = str(strict.get("status", (reclass or {}).get("strict_status", "unknown")))
        bbox_cells = [int(v) for v in strict.get("bbox_cells", (reclass or {}).get("bbox_cells", [0, 0, 0, 0]))]
        center = strict.get("center") or strict.get("crossing_pose")
        if center:
            center_xy = [float(center["x"]), float(center["y"])]
            center_rc = xy_to_rc(center["x"], center["y"])
        else:
            r0, c0, r1, c1 = bbox_cells
            center_rc = [int(round((r0 + r1) / 2.0)), int(round((c0 + c1) / 2.0))]
            xy = grid_to_xy(center_rc[0], center_rc[1])
            center_xy = [xy["x"], xy["y"]]

        context = compute_context_metrics(strict, layers)
        wall_results = (reclass or {}).get("candidate_wall_policy_results", {})
        strict_wall = wall_results.get("strict_current", {})
        eroded_1 = wall_results.get("eroded_wall_1cell", {})
        eroded_2 = wall_results.get("eroded_wall_2cell", {})
        recommended = recommend_action(pair_key, strict, reclass)
        warnings: List[str] = []
        if pair_key == "r3_r11":
            warnings.append("negative_sanity_pair_do_not_restore_direct_edge")
        if pair_key == "r1_r3" and strict.get("selected_for_topology"):
            warnings.append("positive_control_strict_selected")
        if context["label10_fraction_repaired"] >= 0.45:
            warnings.append("high_local_label10_context")
        if context["unknown_fraction"] >= 0.20:
            warnings.append("high_unknown_context")

        candidate = {
            "gateway_id": gid,
            "source": source_for_candidate(strict, reclass),
            "room_a": int(normalize_pair(strict["room_a"], strict["room_b"])[0]),
            "room_b": int(normalize_pair(strict["room_a"], strict["room_b"])[1]),
            "pair_key": pair_key,
            "display_pair": PAIR_DISPLAY[pair_key],
            "center_xy": center_xy,
            "center_rc": center_rc,
            "bbox_cells": bbox_cells,
            "bbox_map_xy": bbox_map_xy_from_cells(bbox_cells),
            "width_m": numeric(strict.get("width_m"), numeric((reclass or {}).get("width_m"), None)),
            "clearance_m": numeric(strict.get("clearance_m"), None),
            "component_area_m2": numeric(strict.get("component_area_m2"), None),
            "strict_status": strict_status if strict_status in {"valid", "ambiguous", "rejected"} else "unknown",
            "strict_reason": strict.get("warning_or_rejection_reason")
            or (reclass or {}).get("strict_reason")
            or "unknown",
            "diagnostic_reclassification": (reclass or {}).get("diagnostic_reclassification", "missing"),
            "selected_for_topology": bool(strict.get("selected_for_topology", False)),
            "connects_room_a": bool_or_none(validation.get("connects_room_a", strict_wall.get("connects_room_a"))),
            "connects_room_b": bool_or_none(validation.get("connects_room_b", strict_wall.get("connects_room_b"))),
            "label10_fraction": numeric(
                label_review.get("strict_label10_fraction"),
                numeric(validation.get("label10_fraction"), context["label10_fraction_repaired"]),
            ),
            "label10_fraction_raw": context["label10_fraction_raw"],
            "label10_fraction_repaired": context["label10_fraction_repaired"],
            "unknown_fraction": numeric(
                label_review.get("strict_unknown_fraction"),
                numeric(validation.get("unknown_fraction"), context["unknown_fraction"]),
            ),
            "wall_fraction_context": numeric(strict_wall.get("wall_fraction_context"), context["wall_fraction_context"]),
            "free_fraction_component": numeric(strict_wall.get("free_fraction_component"), context["free_fraction_component"]),
            "free_fraction_context": numeric(strict_wall.get("free_fraction_context"), None),
            "eroded_wall_1cell_two_sided": bool_or_none(eroded_1.get("two_sided_connectivity")),
            "eroded_wall_2cell_two_sided": bool_or_none(eroded_2.get("two_sided_connectivity")),
            "unknown_permissive_two_sided": bool_or_none(label_review.get("unknown_permissive_two_sided")),
            "label10_weak_two_sided": bool_or_none(label_review.get("label10_weak_two_sided")),
            "approach_from_room_a": strict.get("approach_from_room_a"),
            "approach_from_room_b": strict.get("approach_from_room_b"),
            "crossing_pose": strict.get("crossing_pose"),
            "manual_review_priority": None,
            "recommended_action": recommended,
            "warnings": warnings,
        }
        canonical.append(candidate)

    canonical.sort(key=lambda c: (PAIR_KEYS.index(c["pair_key"]), c["gateway_id"]))
    return canonical


def outline_mask(mask: np.ndarray) -> np.ndarray:
    src = mask.astype(bool, copy=False)
    eroded = src.copy()
    eroded[1:, :] &= src[:-1, :]
    eroded[:-1, :] &= src[1:, :]
    eroded[:, 1:] &= src[:, :-1]
    eroded[:, :-1] &= src[:, 1:]
    return src & ~eroded


def draw_marker(
    img: np.ndarray,
    r: float,
    c: float,
    status: str,
    action: str,
    color: Tuple[int, int, int],
    size: int = 8,
) -> None:
    rr = int(round(r))
    cc = int(round(c))
    black = (0, 0, 0)
    shape = "square" if action == "manual_review_needed" else "circle"
    if action == "negative_sanity_reject":
        shape = "hollow_x"
    elif action in {"possible_manual_hint", "candidate_for_tuned_extractor"}:
        shape = "triangle"
    elif status == "valid":
        shape = "star" if action == "positive_control_selected" else "circle"
    elif status == "ambiguous":
        shape = "diamond"
    elif status == "rejected":
        shape = "x"

    def point(row: int, col: int, colr: Tuple[int, int, int]) -> None:
        if 0 <= row < img.shape[0] and 0 <= col < img.shape[1]:
            img[row, col] = colr

    def line(a: Tuple[int, int], b: Tuple[int, int], colr: Tuple[int, int, int], thick: int = 1) -> None:
        draw_line(img, a[0], a[1], b[0], b[1], colr, thickness=thick)

    if shape == "circle":
        for ang in range(0, 360, 8):
            rad = math.radians(ang)
            point(rr + int(round(math.sin(rad) * size)), cc + int(round(math.cos(rad) * size)), black)
            point(rr + int(round(math.sin(rad) * (size - 1))), cc + int(round(math.cos(rad) * (size - 1))), color)
    elif shape == "star":
        for ang in range(0, 180, 36):
            rad = math.radians(ang)
            line(
                (rr - int(round(math.sin(rad) * size)), cc - int(round(math.cos(rad) * size))),
                (rr + int(round(math.sin(rad) * size)), cc + int(round(math.cos(rad) * size))),
                black,
                2,
            )
            line(
                (rr - int(round(math.sin(rad) * size)), cc - int(round(math.cos(rad) * size))),
                (rr + int(round(math.sin(rad) * size)), cc + int(round(math.cos(rad) * size))),
                color,
                1,
            )
    elif shape == "diamond":
        pts = [(rr - size, cc), (rr, cc + size), (rr + size, cc), (rr, cc - size), (rr - size, cc)]
        for a, b in zip(pts, pts[1:]):
            line(a, b, black, 2)
            line(a, b, color, 1)
    elif shape == "triangle":
        pts = [(rr - size, cc), (rr + size, cc - size), (rr + size, cc + size), (rr - size, cc)]
        for a, b in zip(pts, pts[1:]):
            line(a, b, black, 2)
            line(a, b, color, 1)
    elif shape == "square":
        draw_rect(img, rr - size, cc - size, rr + size, cc + size, black, thickness=2)
        draw_rect(img, rr - size, cc - size, rr + size, cc + size, color, thickness=1)
    elif shape == "hollow_x":
        line((rr - size, cc - size), (rr + size, cc + size), black, 3)
        line((rr - size, cc + size), (rr + size, cc - size), black, 3)
        line((rr - size, cc - size), (rr + size, cc + size), color, 1)
        line((rr - size, cc + size), (rr + size, cc - size), color, 1)
        draw_rect(img, rr - size - 2, cc - size - 2, rr + size + 2, cc + size + 2, color, thickness=1)
    else:
        line((rr - size, cc - size), (rr + size, cc + size), color, 1)
        line((rr - size, cc + size), (rr + size, cc - size), color, 1)


def draw_candidate_annotations(
    img: np.ndarray,
    candidates: Sequence[Dict[str, Any]],
    crop_origin: Tuple[int, int],
    numbered: bool = True,
    label_scale: int = 2,
) -> None:
    row0, col0 = crop_origin
    used_label_slots: List[Tuple[int, int]] = []
    for idx, cand in enumerate(candidates, start=1):
        color = ACTION_COLORS.get(cand["recommended_action"], STATUS_COLORS.get(cand["strict_status"], (255, 255, 255)))
        r0, c0, r1, c1 = [int(v) for v in cand["bbox_cells"]]
        rr0, cc0, rr1, cc1 = r0 - row0, c0 - col0, r1 - row0, c1 - col0
        draw_rect(img, rr0, cc0, rr1, cc1, (0, 0, 0), thickness=3)
        draw_rect(img, rr0, cc0, rr1, cc1, color, thickness=1)
        center_r, center_c = cand["center_rc"]
        local_r = center_r - row0
        local_c = center_c - col0
        draw_marker(img, local_r, local_c, cand["strict_status"], cand["recommended_action"], color, size=8)

        if cand.get("approach_from_room_a") and cand.get("approach_from_room_b"):
            ar, ac = xy_to_rc(cand["approach_from_room_a"]["x"], cand["approach_from_room_a"]["y"])
            br, bc = xy_to_rc(cand["approach_from_room_b"]["x"], cand["approach_from_room_b"]["y"])
            draw_line(img, ar - row0, ac - col0, br - row0, bc - col0, (245, 245, 245), thickness=1)

        if not numbered:
            continue
        text = str(idx)
        label_r = max(3, rr0 - 20)
        label_c = max(3, min(img.shape[1] - 40, cc0 + 8 + (idx % 3) * 12))
        while any(abs(label_r - ur) < 18 and abs(label_c - uc) < 28 for ur, uc in used_label_slots):
            label_r += 18
            label_c += 24
            if label_r > img.shape[0] - 24:
                label_r = max(3, rr1 + 8)
        used_label_slots.append((label_r, label_c))
        draw_line(img, local_r, local_c, label_r + 8, label_c + 8, color, thickness=1)
        draw_rect(img, label_r - 2, label_c - 2, label_r + 18, label_c + 22, (0, 0, 0), thickness=10)
        draw_text(img, text, label_r, label_c, color, scale=label_scale)


def crop_for_pair(
    pair_key: str,
    candidates: Sequence[Dict[str, Any]],
    global_room: np.ndarray,
    margin: int = 30,
) -> Tuple[slice, slice, Dict[str, Any]]:
    shape = global_room.shape
    if candidates:
        r0 = min(int(c["bbox_cells"][0]) for c in candidates)
        c0 = min(int(c["bbox_cells"][1]) for c in candidates)
        r1 = max(int(c["bbox_cells"][2]) for c in candidates)
        c1 = max(int(c["bbox_cells"][3]) for c in candidates)
        centers_r = [int(c["center_rc"][0]) for c in candidates]
        centers_c = [int(c["center_rc"][1]) for c in candidates]
        r0 = min(r0, min(centers_r))
        c0 = min(c0, min(centers_c))
        r1 = max(r1, max(centers_r))
        c1 = max(c1, max(centers_c))
        reason = "candidate_union_bbox_plus_1p5m_margin"
    else:
        a, b = [int(x) for x in pair_key[1:].split("_r")]
        room_a = global_room == a
        room_b = global_room == b
        band = shifted_or(room_a, 8) & shifted_or(room_b, 8)
        if not np.any(band):
            band = shifted_or(room_a, 20) & shifted_or(room_b, 20)
        rows, cols = np.where(band)
        if rows.size:
            mid = len(rows) // 2
            r0 = r1 = int(rows[mid])
            c0 = c1 = int(cols[mid])
            reason = "closest_shared_boundary_diagnostic_area"
        else:
            rows_a, cols_a = np.where(room_a)
            rows_b, cols_b = np.where(room_b)
            if rows_a.size and rows_b.size:
                r0 = r1 = int(round((float(rows_a.mean()) + float(rows_b.mean())) / 2.0))
                c0 = c1 = int(round((float(cols_a.mean()) + float(cols_b.mean())) / 2.0))
            else:
                r0 = r1 = shape[0] // 2
                c0 = c1 = shape[1] // 2
            reason = "room_centroid_midpoint_fallback"

    r0 = max(0, r0 - margin)
    c0 = max(0, c0 - margin)
    r1 = min(shape[0] - 1, r1 + margin)
    c1 = min(shape[1] - 1, c1 + margin)
    min_size = 420
    if r1 - r0 + 1 < min_size:
        extra = (min_size - (r1 - r0 + 1)) // 2 + 1
        r0 = max(0, r0 - extra)
        r1 = min(shape[0] - 1, r1 + extra)
    if c1 - c0 + 1 < min_size:
        extra = (min_size - (c1 - c0 + 1)) // 2 + 1
        c0 = max(0, c0 - extra)
        c1 = min(shape[1] - 1, c1 + extra)
    return slice(r0, r1 + 1), slice(c0, c1 + 1), {
        "reason": reason,
        "crop_cells": [r0, c0, r1, c1],
        "margin_pixels": margin,
        "margin_m": round(margin * RESOLUTION, 3),
    }


def draw_pair_base(
    layers: Dict[str, np.ndarray],
    rs: slice,
    cs: slice,
    rooms: Tuple[int, int],
    diagnostic_layer: Optional[str] = None,
) -> np.ndarray:
    room_a, room_b = rooms
    global_crop = layers["global_room"][rs, cs]
    wall = layers["wall"]
    if diagnostic_layer == "eroded_wall_1cell":
        wall = layers["eroded_wall_1cell"]
    elif diagnostic_layer == "eroded_wall_2cell":
        wall = layers["eroded_wall_2cell"]
    elif diagnostic_layer == "free_space_only":
        wall = np.zeros_like(layers["wall"], dtype=bool)

    img = base_visual(global_crop, wall[rs, cs], layers["free"][rs, cs], layers["unknown"][rs, cs], layers["outside"][rs, cs])
    blend_mask(img, outline_mask(global_crop == room_a), room_color(room_a), 1.0)
    blend_mask(img, outline_mask(global_crop == room_b), room_color(room_b), 1.0)
    blend_mask(img, shifted_or(layers["local_label_repaired"] == 10, 1)[rs, cs], (255, 255, 20), 0.88)
    if diagnostic_layer == "label10_unknown":
        blend_mask(img, shifted_or(layers["unknown"], 1)[rs, cs], (165, 165, 175), 0.95)
        blend_mask(img, shifted_or(layers["local_label_raw"] == 10, 1)[rs, cs], (255, 175, 30), 0.80)
    if diagnostic_layer == "wall_room_overlap":
        blend_mask(img, shifted_or(layers["wall"] & (layers["global_room"] > 0), 1)[rs, cs], (255, 0, 255), 0.94)
    if diagnostic_layer == "free_space_only":
        blend_mask(img, shifted_or(layers["free"], 1)[rs, cs], (45, 205, 90), 0.70)
    return img


def render_pair_overview(
    out_dir: Path,
    pair_key: str,
    candidates: Sequence[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
    crop_info: Dict[str, Any],
    numbered: bool,
) -> Path:
    a, b = [int(x) for x in pair_key[1:].split("_r")]
    rs = slice(crop_info["crop_cells"][0], crop_info["crop_cells"][2] + 1)
    cs = slice(crop_info["crop_cells"][1], crop_info["crop_cells"][3] + 1)
    img = draw_pair_base(layers, rs, cs, (a, b))
    draw_candidate_annotations(img, candidates, (rs.start or 0, cs.start or 0), numbered=numbered)
    legend = [
        (f"R{a}", room_color(a)),
        (f"R{b}", room_color(b)),
        ("WALL", (235, 238, 242)),
        ("FREE", (35, 120, 55)),
        ("UNKNOWN", (74, 74, 78)),
        ("LABEL10", (255, 255, 20)),
        ("VALID/O", STATUS_COLORS["valid"]),
        ("AMBIG/DIA", STATUS_COLORS["ambiguous"]),
        ("REJECT/X", STATUS_COLORS["rejected"]),
        ("RELAX/TRI", ACTION_COLORS["candidate_for_tuned_extractor"]),
        ("NEG/HX", ACTION_COLORS["negative_sanity_reject"]),
    ]
    suffix = "_numbered" if numbered else ""
    path = out_dir / f"local_overview_{pair_key}{suffix}.png"
    title = f"B2R2 {pair_key.upper()} LOCAL CROP"
    write_png(path, add_panel(img, title, legend))
    return path


def render_diagnostic(
    out_dir: Path,
    pair_key: str,
    candidates: Sequence[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
    crop_info: Dict[str, Any],
    diagnostic_name: str,
) -> Path:
    layer_name = {
        "diagnostic_strict_wall": "strict_wall",
        "diagnostic_eroded_wall_1cell": "eroded_wall_1cell",
        "diagnostic_eroded_wall_2cell": "eroded_wall_2cell",
        "diagnostic_free_space_only": "free_space_only",
        "diagnostic_label10_unknown": "label10_unknown",
        "diagnostic_wall_room_overlap": "wall_room_overlap",
    }[diagnostic_name]
    a, b = [int(x) for x in pair_key[1:].split("_r")]
    rs = slice(crop_info["crop_cells"][0], crop_info["crop_cells"][2] + 1)
    cs = slice(crop_info["crop_cells"][1], crop_info["crop_cells"][3] + 1)
    img = draw_pair_base(layers, rs, cs, (a, b), diagnostic_layer=layer_name)
    draw_candidate_annotations(img, candidates, (rs.start or 0, cs.start or 0), numbered=True, label_scale=1)
    legend = [
        (f"R{a}", room_color(a)),
        (f"R{b}", room_color(b)),
        ("WALL", (235, 238, 242)),
        ("FREE", (35, 120, 55)),
        ("UNKNOWN", (74, 74, 78)),
        ("LABEL10", (255, 255, 20)),
        ("WALL-ROOM", (255, 0, 255)),
    ]
    path = out_dir / f"{diagnostic_name}_{pair_key}.png"
    short_diag = diagnostic_name.replace("diagnostic_", "").replace("_", "-").upper()
    title = f"B2R2 {pair_key.upper()} {short_diag}"
    write_png(path, add_panel(img, title, legend))
    return path


def resize_nearest(img: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    scale = min(max_w / img.shape[1], max_h / img.shape[0], 1.0)
    if scale >= 0.999:
        return img
    new_h = max(1, int(round(img.shape[0] * scale)))
    new_w = max(1, int(round(img.shape[1] * scale)))
    row_idx = np.minimum((np.arange(new_h) / scale).astype(int), img.shape[0] - 1)
    col_idx = np.minimum((np.arange(new_w) / scale).astype(int), img.shape[1] - 1)
    return img[row_idx[:, None], col_idx[None, :]]


def candidate_panel_image(
    candidate: Dict[str, Any],
    layers: Dict[str, np.ndarray],
    max_inner: Tuple[int, int] = (430, 360),
) -> np.ndarray:
    shape = layers["global_room"].shape
    r0, c0, r1, c1 = [int(v) for v in candidate["bbox_cells"]]
    margin = 30
    rr0 = max(0, r0 - margin)
    cc0 = max(0, c0 - margin)
    rr1 = min(shape[0] - 1, r1 + margin)
    cc1 = min(shape[1] - 1, c1 + margin)
    min_size = 110
    if rr1 - rr0 + 1 < min_size:
        extra = (min_size - (rr1 - rr0 + 1)) // 2 + 1
        rr0 = max(0, rr0 - extra)
        rr1 = min(shape[0] - 1, rr1 + extra)
    if cc1 - cc0 + 1 < min_size:
        extra = (min_size - (cc1 - cc0 + 1)) // 2 + 1
        cc0 = max(0, cc0 - extra)
        cc1 = min(shape[1] - 1, cc1 + extra)
    rs, cs = slice(rr0, rr1 + 1), slice(cc0, cc1 + 1)
    img = draw_pair_base(layers, rs, cs, (candidate["room_a"], candidate["room_b"]))
    draw_candidate_annotations(img, [candidate], (rr0, cc0), numbered=False)
    resized = resize_nearest(img, max_inner[0], max_inner[1])
    header_h = 128
    info_h = 96
    out_w = max(resized.shape[1], 760)
    out = np.zeros((header_h + resized.shape[0] + info_h, out_w, 3), dtype=np.uint8)
    out[:] = (8, 10, 14)
    draw_text(out, candidate["gateway_id"], 12, 12, (255, 255, 255), scale=2)
    draw_text(
        out,
        f"STATUS {candidate['strict_status']}  WIDTH_M {candidate['width_m']}",
        42,
        12,
        (230, 235, 240),
        scale=1,
    )
    draw_text(
        out,
        f"RECLASS {candidate['diagnostic_reclassification']}",
        60,
        12,
        (230, 235, 240),
        scale=1,
    )
    draw_text(
        out,
        f"ACTION {candidate['recommended_action']}",
        78,
        12,
        ACTION_COLORS.get(candidate["recommended_action"], (230, 235, 240)),
        scale=1,
    )
    legend_items = [
        ("CENTER", ACTION_COLORS.get(candidate["recommended_action"], (255, 255, 255))),
        ("BBOX", ACTION_COLORS.get(candidate["recommended_action"], (255, 255, 255))),
        ("R_A", room_color(candidate["room_a"])),
        ("R_B", room_color(candidate["room_b"])),
        ("L10", (255, 255, 20)),
    ]
    x = 12
    for label, color in legend_items:
        draw_rect(out, 104, x, 118, x + 18, color, thickness=7)
        draw_text(out, label, 104, x + 26, (230, 235, 240), scale=1)
        x += 112
    out[header_h : header_h + resized.shape[0], : resized.shape[1]] = resized
    y = header_h + resized.shape[0] + 8
    lines = [
        f"CENTER_RC {candidate['center_rc']} CENTER_XY {candidate['center_xy']}",
        f"APPROACH_A {pose_short(candidate.get('approach_from_room_a'))}",
        f"APPROACH_B {pose_short(candidate.get('approach_from_room_b'))}",
        f"L10 {candidate['label10_fraction']} UNK {candidate['unknown_fraction']} FREE {candidate['free_fraction_component']}",
        f"E1 {candidate['eroded_wall_1cell_two_sided']} E2 {candidate['eroded_wall_2cell_two_sided']}",
    ]
    for line in lines:
        draw_text(out, line[:95], y, 12, (230, 235, 240), scale=1)
        y += 16
    return out


def pose_short(pose: Optional[Dict[str, Any]]) -> str:
    if not pose:
        return "NA"
    return f"{float(pose['x']):.2f},{float(pose['y']):.2f},{float(pose.get('yaw', 0.0)):.2f}"


def render_candidate_panels(
    pair_dir: Path,
    pair_key: str,
    candidates: Sequence[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
) -> Tuple[List[Path], List[Path]]:
    panel_dir = pair_dir / "candidate_panels"
    panel_dir.mkdir(parents=True, exist_ok=True)
    panel_paths: List[Path] = []
    panel_images: List[np.ndarray] = []
    for candidate in candidates:
        img = candidate_panel_image(candidate, layers)
        path = panel_dir / f"{candidate['gateway_id']}.png"
        write_png(path, img)
        panel_paths.append(path)
        panel_images.append(img)

    contact_paths: List[Path] = []
    if not panel_images:
        img = np.zeros((240, 620, 3), dtype=np.uint8)
        img[:] = (8, 10, 14)
        draw_text(img, f"{PAIR_DISPLAY[pair_key]} NO CANDIDATES", 36, 24, (255, 255, 255), scale=2)
        path = pair_dir / f"candidate_contact_sheet_{pair_key}.png"
        write_png(path, img)
        contact_paths.append(path)
        return panel_paths, contact_paths

    per_page = 9
    for page_index in range(0, len(panel_images), per_page):
        page = panel_images[page_index : page_index + per_page]
        thumbs = [resize_nearest(img, 560, 520) for img in page]
        cols = 3 if len(thumbs) > 2 else len(thumbs)
        rows = int(math.ceil(len(thumbs) / cols))
        pad = 16
        cell_h = max(img.shape[0] for img in thumbs)
        cell_w = max(img.shape[1] for img in thumbs)
        sheet = np.zeros((rows * cell_h + (rows + 1) * pad, cols * cell_w + (cols + 1) * pad, 3), dtype=np.uint8)
        sheet[:] = (8, 10, 14)
        for idx, img in enumerate(thumbs):
            row = idx // cols
            col = idx % cols
            y = pad + row * (cell_h + pad)
            x = pad + col * (cell_w + pad)
            sheet[y : y + img.shape[0], x : x + img.shape[1]] = img
        page_num = page_index // per_page + 1
        if len(panel_images) <= per_page:
            path = pair_dir / f"candidate_contact_sheet_{pair_key}.png"
        else:
            path = pair_dir / f"candidate_contact_sheet_{pair_key}_page{page_num:02d}.png"
        write_png(path, sheet)
        contact_paths.append(path)
    return panel_paths, contact_paths


def score_candidate(candidate: Dict[str, Any]) -> Tuple[float, Dict[str, float]]:
    width = numeric(candidate.get("width_m"), 0.0) or 0.0
    free_support = numeric(candidate.get("free_fraction_component"), 0.0) or 0.0
    label10 = numeric(candidate.get("label10_fraction"), 0.0) or 0.0
    unknown = numeric(candidate.get("unknown_fraction"), 0.0) or 0.0
    width_score = min(width / 0.35, 1.0) * 0.22
    free_support_score = min(free_support, 1.0) * 0.23
    eroded_two_sided_score = 0.0
    if candidate.get("eroded_wall_1cell_two_sided"):
        eroded_two_sided_score += 0.14
    if candidate.get("eroded_wall_2cell_two_sided"):
        eroded_two_sided_score += 0.11
    strict_positive_bonus = 0.22 if candidate.get("selected_for_topology") or candidate.get("strict_status") == "valid" else 0.0
    negative_sanity_penalty = -0.35 if candidate["pair_key"] == "r3_r11" else 0.0
    label10_penalty = -min(label10, 1.0) * 0.08
    unknown_penalty = -min(unknown, 1.0) * 0.08
    action_bonus = 0.06 if candidate["recommended_action"] in {"candidate_for_tuned_extractor", "possible_manual_hint"} else 0.0
    score = (
        width_score
        + free_support_score
        + eroded_two_sided_score
        + strict_positive_bonus
        + negative_sanity_penalty
        + label10_penalty
        + unknown_penalty
        + action_bonus
    )
    terms = {
        "width_score": round(width_score, 6),
        "free_support_score": round(free_support_score, 6),
        "eroded_two_sided_score": round(eroded_two_sided_score, 6),
        "label10_penalty": round(label10_penalty, 6),
        "unknown_penalty": round(unknown_penalty, 6),
        "strict_positive_bonus": round(strict_positive_bonus, 6),
        "negative_sanity_penalty": round(negative_sanity_penalty, 6),
        "manual_hint_bonus": round(action_bonus, 6),
    }
    return round(max(0.0, min(1.0, score)), 6), terms


def build_rankings(candidates: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    rankings: List[Dict[str, Any]] = []
    for pair_key in PAIR_KEYS:
        pair_candidates = [c for c in candidates if c["pair_key"] == pair_key]
        scored = []
        for candidate in pair_candidates:
            score, terms = score_candidate(candidate)
            scored.append((score, terms, candidate))
        scored.sort(key=lambda item: (-item[0], item[2]["gateway_id"]))
        top_candidates = []
        for rank, (score, terms, candidate) in enumerate(scored, start=1):
            candidate["manual_review_priority"] = rank
            top_candidates.append(
                {
                    "rank": rank,
                    "gateway_id": candidate["gateway_id"],
                    "score": score,
                    "score_terms": terms,
                    "recommended_action": candidate["recommended_action"],
                    "diagnostic_reclassification": candidate["diagnostic_reclassification"],
                    "strict_status": candidate["strict_status"],
                }
            )
        note = "manual review only"
        if pair_key == "r3_r11":
            note = "negative sanity check; do not promote"
        elif pair_key == "r1_r3":
            note = "positive control; strict selected candidate should rank at or near the top"
        rankings.append({"pair_key": pair_key, "display_pair": PAIR_DISPLAY[pair_key], "note": note, "top_candidates": top_candidates})
    return {"scene_id": SCENE_ID, "step": STEP_NAME, "ranking_is_not_acceptance": True, "pair_rankings": rankings}


def write_review_table(candidates: Sequence[Dict[str, Any]]) -> None:
    payload = {
        "scene_id": SCENE_ID,
        "step": STEP_NAME,
        "artifact_type": "step29b2r2_candidate_review_table",
        "version": VERSION,
        "candidate_count": len(candidates),
        "canonical_candidates": candidates,
    }
    write_json(REVIEW_TABLE_JSON, payload)
    fields = [
        "gateway_id",
        "source",
        "pair_key",
        "display_pair",
        "room_a",
        "room_b",
        "center_xy",
        "center_rc",
        "bbox_cells",
        "width_m",
        "clearance_m",
        "component_area_m2",
        "strict_status",
        "strict_reason",
        "diagnostic_reclassification",
        "selected_for_topology",
        "connects_room_a",
        "connects_room_b",
        "label10_fraction",
        "label10_fraction_raw",
        "label10_fraction_repaired",
        "unknown_fraction",
        "wall_fraction_context",
        "free_fraction_component",
        "eroded_wall_1cell_two_sided",
        "eroded_wall_2cell_two_sided",
        "manual_review_priority",
        "recommended_action",
        "warnings",
    ]
    REVIEW_TABLE_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REVIEW_TABLE_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for candidate in candidates:
            row = {}
            for field in fields:
                value = candidate.get(field)
                if isinstance(value, (list, dict)):
                    value = json.dumps(json_ready(value), sort_keys=True)
                row[field] = value
            writer.writerow(row)
    print(f"Written: {rel(REVIEW_TABLE_CSV)}")


def write_manual_template(candidates: Sequence[Dict[str, Any]]) -> None:
    reviews = []
    for candidate in candidates:
        reviews.append(
            {
                "pair_key": candidate["pair_key"],
                "gateway_id": candidate["gateway_id"],
                "manual_status": "unreviewed",
                "selected_for_topology": False,
                "reason": "",
                "corrected_center_xy": None,
                "corrected_crossing_pose": None,
                "corrected_approach_from_room_a": None,
                "corrected_approach_from_room_b": None,
                "reviewer_notes": "",
            }
        )
    payload = {
        "scene_id": SCENE_ID,
        "step": STEP_NAME,
        "instructions": "Fill this file manually after inspecting local crop PNGs. Do not treat this file as authoritative until a later step explicitly consumes it.",
        "reviews": reviews,
    }
    write_json(MANUAL_TEMPLATE_JSON, payload)


def write_readme(summary: Dict[str, Any]) -> None:
    text = f"""# Step29B2R2 Local Gateway Inspection Pack

Scene: `{SCENE_ID}`

Step29B2R2 makes the existing strict, relaxed, rejected, and reclassified gateway candidates visually inspectable. It builds tight local crops, candidate panels, per-pair contact sheets, a JSON/CSV review table, pair rankings for manual inspection, and a manual review template.

## What This Does Not Do

- Does not run ROS, Nav2, Gazebo, AMCL, DWB, or live robot/navigation processes.
- Does not proceed to Step29C.
- Does not generate a gateway-augmented topology.
- Does not overwrite Step29B2 or Step29B2R artifacts.
- Does not re-extract gateways from scratch.
- Does not use local watershed labels as global room IDs.

## Inputs

- Step29B1 layered BEV JSON/NPZ, global room mask, room label mapping, and summary.
- Step29B2 gateway candidates, graph, extraction summary, room-pair adjacency, rejected edges, and label10/wall-overlap audit.
- Step29B2R sensitivity review, key-pair relaxed review, label10/unknown impact, readiness, validation, and reclassified candidates.

The authoritative room identity layer is `room_mask_global_id`. Local labels are shown only as diagnostics. Local label `10` is treated as artifact/background/unknown-like context.

## Outputs

- `generated/assets/00824_step29b2r2_candidate_review_table_v0_1.json`
- `generated/assets/00824_step29b2r2_candidate_review_table_v0_1.csv`
- `generated/assets/00824_step29b2r2_pair_ranking_v0_1.json`
- `generated/assets/00824_step29b2r2_manual_gateway_review_template_v0_1.json`
- `generated/assets/00824_step29b2r2_schema_inventory_v0_1.json`
- `generated/assets/00824_step29b2r2_summary_v0_1.json`
- `generated/assets/00824_step29b2r2_validation_results_v0_1.json`
- Per-pair local figures under `generated/visualizations/pair_*`.

## How To Read Figures

Each pair directory has:

- `local_overview_<pair>.png`: tight local crop with candidate boxes and markers.
- `local_overview_<pair>_numbered.png`: same crop with readable candidate numbers and leader lines.
- `candidate_contact_sheet_<pair>.png`: one panel per candidate, paginated if needed.
- `diagnostic_strict_wall_<pair>.png`: current strict wall layer.
- `diagnostic_eroded_wall_1cell_<pair>.png`: diagnostic-only 1-cell wall erosion.
- `diagnostic_eroded_wall_2cell_<pair>.png`: diagnostic-only 2-cell wall erosion.
- `diagnostic_free_space_only_<pair>.png`: diagnostic-only free-space view.
- `diagnostic_label10_unknown_<pair>.png`: label10 and unknown emphasis.
- `diagnostic_wall_room_overlap_<pair>.png`: wall/room overlap emphasis.
- `candidate_panels/<gateway_id>.png`: individual local candidate panel.

## Marker Legend

- Circle: strict valid candidate.
- Star: strict selected positive-control candidate.
- Diamond: ambiguous candidate.
- X: strict rejected candidate.
- Triangle: relaxed/reclassified/manual-hint style candidate.
- Hollow X/cross: negative sanity candidate.
- Candidate boxes show the candidate bounding cells; labels use leader lines when crowded.

## Recommended Actions

- `positive_control_selected`: strict selected candidate for room_1 <-> room_3.
- `negative_sanity_reject`: room_3 <-> room_11 sanity check; keep rejected.
- `candidate_for_tuned_extractor`: possible candidate sensitive to wall erosion and worth tuned-extractor review.
- `possible_manual_hint`: plausible manual hint candidate, not topology acceptance.
- `inspect_png`: inspect the local panels before making a decision.
- `likely_false_positive`: diagnostic evidence looks weak.
- `keep_rejected`: preserve rejection unless later evidence says otherwise.

## room_3 <-> room_11

`room_3 <-> room_11` is a negative sanity check. Step29B2 correctly rejected direct connectivity, and this pack preserves that recommendation even if relaxed diagnostic evidence appears. Do not restore this direct edge in Step29B2R2.

## Recommended Next Step

1. Manually inspect the local crop PNGs and candidate panels.
2. Fill `generated/assets/00824_step29b2r2_manual_gateway_review_template_v0_1.json`.
3. Then decide whether Step29B3 should consume manual gateway hints, use a tuned extractor, adjust the wall layer, or repair label10/unknown boundary artifacts.

## Run Command

```bash
cd {REPO_ROOT}
/usr/bin/python3 tools/step29b2r2_build_local_gateway_inspection_pack.py
```

## Summary

- Canonical candidates: {summary['canonical_candidate_count']}
- Candidates per pair: {json.dumps(summary['candidate_counts_by_pair'], sort_keys=True)}
- Figure count: {summary['figure_count']}
- Coordinate convention: rows increase with map y using `row = round((y - origin_y) / resolution)`, columns increase with map x.
"""
    README_PATH.parent.mkdir(parents=True, exist_ok=True)
    README_PATH.write_text(text)
    print(f"Written: {rel(README_PATH)}")


def input_payloads() -> Tuple[Dict[str, Any], np.lib.npyio.NpzFile]:
    missing = [
        rel(path)
        for path in list(STEP29B1_INPUTS.values()) + list(STEP29B2_INPUTS.values()) + list(STEP29B2R_INPUTS.values())
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError("Missing required inputs: " + ", ".join(missing))
    payloads: Dict[str, Any] = {}
    for name, path in {**STEP29B1_INPUTS, **STEP29B2_INPUTS, **STEP29B2R_INPUTS}.items():
        if path.suffix == ".json":
            payloads[name] = read_json(path)
    npz = np.load(STEP29B1_INPUTS["layered_bev_npz"])
    return payloads, npz


def build_layers(npz: np.lib.npyio.NpzFile) -> Dict[str, np.ndarray]:
    global_room = npz["room_mask_global_id"].astype(np.int32)
    global_room_npy = np.load(STEP29B1_INPUTS["global_room_mask"]).astype(np.int32)
    if global_room.shape != global_room_npy.shape or not np.array_equal(global_room, global_room_npy):
        raise ValueError("room_mask_global_id in NPZ does not match 00824_global_room_mask_v0_1.npy")
    wall = npz["structural_wall"] > 0
    return {
        "global_room": global_room,
        "local_label_raw": npz["room_mask_local_label_raw"].astype(np.int32),
        "local_label_repaired": npz["room_mask_local_label_repaired"].astype(np.int32),
        "wall": wall,
        "eroded_wall_1cell": erode_disk(wall, 1),
        "eroded_wall_2cell": erode_disk(wall, 2),
        "free": npz["free_space"] > 0,
        "outside": npz["outside_boundary"] > 0,
        "unknown": npz["unknown_layer"] > 0,
        "full_map_post_doors_reference": npz["full_map_post_doors_reference"] > 0,
    }


def render_all_visuals(candidates: Sequence[Dict[str, Any]], layers: Dict[str, np.ndarray]) -> Dict[str, Any]:
    outputs: Dict[str, Any] = {"pairs": {}, "figure_paths": []}
    for pair_key in PAIR_KEYS:
        pair_candidates = [c for c in candidates if c["pair_key"] == pair_key]
        pair_dir = VIS_DIR / f"pair_{pair_key}"
        pair_dir.mkdir(parents=True, exist_ok=True)
        _rs, _cs, crop_info = crop_for_pair(pair_key, pair_candidates, layers["global_room"], margin=30)
        pair_outputs: Dict[str, Any] = {"crop": crop_info, "candidate_count": len(pair_candidates), "figures": []}

        for numbered in (False, True):
            path = render_pair_overview(pair_dir, pair_key, pair_candidates, layers, crop_info, numbered)
            pair_outputs["figures"].append(rel(path))
            outputs["figure_paths"].append(rel(path))

        for diagnostic_name in DIAG_LAYER_NAMES:
            path = render_diagnostic(pair_dir, pair_key, pair_candidates, layers, crop_info, diagnostic_name)
            pair_outputs["figures"].append(rel(path))
            outputs["figure_paths"].append(rel(path))

        panel_paths, contact_paths = render_candidate_panels(pair_dir, pair_key, pair_candidates, layers)
        pair_outputs["candidate_panels"] = [rel(p) for p in panel_paths]
        pair_outputs["contact_sheets"] = [rel(p) for p in contact_paths]
        outputs["figure_paths"].extend(rel(p) for p in panel_paths + contact_paths)
        outputs["pairs"][pair_key] = pair_outputs
    outputs["figure_count"] = len(outputs["figure_paths"])
    return outputs


def validate_outputs(
    candidates: Sequence[Dict[str, Any]],
    visual_outputs: Dict[str, Any],
    b2_sha_before: Dict[str, str],
    b2r_sha_before: Dict[str, str],
) -> Dict[str, Any]:
    b2_sha_after = {name: sha256_file(path) for name, path in STEP29B2_INPUTS.items()}
    b2r_sha_after = {name: sha256_file(path) for name, path in STEP29B2R_INPUTS.items()}
    checks: List[Dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    for pair_key in PAIR_KEYS:
        pair_dir = VIS_DIR / f"pair_{pair_key}"
        add(f"{pair_key}_directory_exists", pair_dir.is_dir(), rel(pair_dir))
        add(
            f"{pair_key}_local_overview_exists",
            (pair_dir / f"local_overview_{pair_key}.png").is_file()
            and (pair_dir / f"local_overview_{pair_key}_numbered.png").is_file(),
            rel(pair_dir),
        )
        has_contact = bool(visual_outputs["pairs"].get(pair_key, {}).get("contact_sheets"))
        add(f"{pair_key}_contact_sheet_or_no_candidate_record", has_contact, str(visual_outputs["pairs"].get(pair_key, {}).get("contact_sheets")))

    add("candidate_review_table_json_exists", REVIEW_TABLE_JSON.is_file(), rel(REVIEW_TABLE_JSON))
    add("candidate_review_table_csv_exists", REVIEW_TABLE_CSV.is_file(), rel(REVIEW_TABLE_CSV))
    add("manual_review_template_exists", MANUAL_TEMPLATE_JSON.is_file(), rel(MANUAL_TEMPLATE_JSON))
    add("room_mask_global_id_read_successfully", True, "NPZ room_mask_global_id matched global room mask NPY")
    add("local_labels_not_used_as_global_room_ids", True, "Local labels used only for label10 diagnostics")
    r3r11 = [c for c in candidates if c["pair_key"] == "r3_r11"]
    add(
        "room_3_room_11_negative_sanity_keep_rejected",
        bool(r3r11) and all(c["recommended_action"] == "negative_sanity_reject" for c in r3r11),
        "all r3_r11 candidates retain negative_sanity_reject",
    )
    forbidden_topology = list(OUTPUT_ROOT.rglob("*topology*.json")) + list(OUTPUT_ROOT.rglob("*gateway_augmented*.json"))
    add("no_topology_augmentation_generated", len(forbidden_topology) == 0, str([rel(p) for p in forbidden_topology]))
    add("step29b2_artifacts_not_modified", b2_sha_before == b2_sha_after, "SHA256 before/after comparison")
    add("step29b2r_artifacts_not_modified", b2r_sha_before == b2r_sha_after, "SHA256 before/after comparison")
    add("ros_nav2_gazebo_not_run", True, "Script contains no ROS/Nav2/Gazebo execution path")
    add("did_not_proceed_to_step29c", True, "No Step29C artifacts generated")

    payload = {
        "scene_id": SCENE_ID,
        "step": STEP_NAME,
        "all_passed": all(check["passed"] for check in checks),
        "checks": checks,
        "step29b2_sha256_before": b2_sha_before,
        "step29b2_sha256_after": b2_sha_after,
        "step29b2r_sha256_before": b2r_sha_before,
        "step29b2r_sha256_after": b2r_sha_after,
    }
    write_json(VALIDATION_JSON, payload)
    return payload


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    b2_sha_before = {name: sha256_file(path) for name, path in STEP29B2_INPUTS.items()}
    b2r_sha_before = {name: sha256_file(path) for name, path in STEP29B2R_INPUTS.items()}

    payloads, npz = input_payloads()
    layers = build_layers(npz)
    schema_inventory = inspect_schema_inventory(payloads, npz)
    write_json(SCHEMA_INVENTORY_JSON, schema_inventory)

    strict_candidates = payloads["gateway_candidates"]["gateway_candidates"]
    reclassified = payloads["reclassified_rejected_candidates"].get("reclassified_candidates", [])
    label_reviews = payloads["label10_unknown_gateway_impact"].get("candidate_label10_unknown_reviews", [])
    canonical = canonicalize_candidates(
        strict_candidates,
        index_by_gateway_id(reclassified),
        index_by_gateway_id(label_reviews),
        layers,
    )
    rankings = build_rankings(canonical)
    write_json(PAIR_RANKING_JSON, rankings)
    write_review_table(canonical)
    write_manual_template(canonical)
    visual_outputs = render_all_visuals(canonical, layers)

    counts_by_pair = {pair_key: len([c for c in canonical if c["pair_key"] == pair_key]) for pair_key in PAIR_KEYS}
    assumptions = [
        "Step29B2 and Step29B2R candidates are joined by gateway_id.",
        "Map/grid conversion uses row = round((y - origin_y) / resolution), col = round((x - origin_x) / resolution).",
        "room_mask_global_id is authoritative; local labels are diagnostic only.",
        "Metrics missing from Step29B2R are computed inside a 3-cell bbox context from Step29B1 layers.",
    ]
    missing_fields = []
    for candidate in canonical:
        for field in ["width_m", "center_xy", "bbox_cells", "diagnostic_reclassification"]:
            if candidate.get(field) in (None, "missing", []):
                missing_fields.append({"gateway_id": candidate["gateway_id"], "field": field})

    summary = {
        "scene_id": SCENE_ID,
        "step": STEP_NAME,
        "artifact_type": "step29b2r2_summary",
        "version": VERSION,
        "output_directory": rel(OUTPUT_ROOT),
        "script_path": rel(Path(__file__)),
        "run_command": f"cd {REPO_ROOT} && /usr/bin/python3 tools/step29b2r2_build_local_gateway_inspection_pack.py",
        "canonical_candidate_count": len(canonical),
        "candidate_counts_by_pair": counts_by_pair,
        "figure_count": visual_outputs["figure_count"],
        "visual_outputs": visual_outputs,
        "review_table_json": rel(REVIEW_TABLE_JSON),
        "review_table_csv": rel(REVIEW_TABLE_CSV),
        "pair_ranking_json": rel(PAIR_RANKING_JSON),
        "manual_review_template_json": rel(MANUAL_TEMPLATE_JSON),
        "schema_inventory_json": rel(SCHEMA_INVENTORY_JSON),
        "validation_results_json": rel(VALIDATION_JSON),
        "coordinate_convention": {
            "origin": ORIGIN,
            "resolution": RESOLUTION,
            "grid_shape": list(layers["global_room"].shape),
            "row_formula": "row = round((y - origin_y) / resolution)",
            "col_formula": "col = round((x - origin_x) / resolution)",
            "row_direction": "rows increase as map y increases in these artifacts",
            "col_direction": "columns increase as map x increases",
        },
        "schema_assumptions": assumptions,
        "missing_fields": missing_fields,
        "r3_r11_negative_sanity_rejected": all(
            c["recommended_action"] == "negative_sanity_reject" for c in canonical if c["pair_key"] == "r3_r11"
        ),
        "topology_augmented": False,
        "ros_nav2_gazebo_run": False,
        "step29c_started": False,
    }
    write_json(SUMMARY_JSON, summary)
    validation = validate_outputs(canonical, visual_outputs, b2_sha_before, b2r_sha_before)
    summary["validation_all_passed"] = validation["all_passed"]
    write_json(SUMMARY_JSON, summary)
    write_readme(summary)

    print("\nStep29B2R2 complete")
    print(f"script: {rel(Path(__file__))}")
    print(f"output: {rel(OUTPUT_ROOT)}")
    print(f"canonical_candidates: {len(canonical)}")
    print(f"candidates_by_pair: {json.dumps(counts_by_pair, sort_keys=True)}")
    print(f"figure_count: {visual_outputs['figure_count']}")
    print(f"validation: {rel(VALIDATION_JSON)} all_passed={validation['all_passed']}")
    print(f"schema_assumptions: {len(assumptions)} missing_fields={len(missing_fields)}")
    print(f"r3_r11_negative_sanity_rejected: {summary['r3_r11_negative_sanity_rejected']}")


if __name__ == "__main__":
    main()
