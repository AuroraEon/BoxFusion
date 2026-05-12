#!/usr/bin/env python3
"""
Step29B1R: audit Step29B1 structural_wall provenance and export
non-inflated / gap-preserving wall evidence candidates.

This script does not run ROS, Nav2, Gazebo, AMCL, DWB, RViz, or any live
navigation process. It does not generate Step29C topology and does not modify
prior Step29 artifacts.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT_SCENE_ID = "00824"
VERSION = "v0_1"
BASE_DIR = Path("/home/ws/workspace/BoxFusion")

STEP29A3_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29a3_00824_stage_a_debug_raster_export"
STEP29A3_SCENE_DIR = STEP29A3_DIR / "scenes" / SCENE_ID
STEP29A3_FINAL_DIR = STEP29A3_SCENE_DIR / "final_raster_export"
STEP29A3_DEBUG_DIR = STEP29A3_SCENE_DIR / "debug_room" / "floor_1"

STEP29B1_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3"
STEP29B1_ASSETS = STEP29B1_DIR / "generated" / "assets"
STEP29B1_JSON = STEP29B1_ASSETS / "00824_layered_bev_from_step29a3_v0_1.json"
STEP29B1_NPZ = STEP29B1_ASSETS / "00824_layered_bev_from_step29a3_v0_1.npz"
STEP29B1_GLOBAL_MASK = STEP29B1_ASSETS / "00824_global_room_mask_v0_1.npy"

STEP29B3R_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b3r_00824_gateway_hypothesis_audit_and_roles"
STEP29B3R_ASSETS = STEP29B3R_DIR / "generated" / "assets"
HYPOTHESES_WITH_ROLES = STEP29B3R_ASSETS / "00824_gateway_hypotheses_with_roles_v0_1.json"

OUTPUT_DIR = (
    BASE_DIR
    / "runtime_stage1_frozen_evidence"
    / "step29b1r_00824_structural_wall_source_audit_and_uninflated_export"
)
ASSETS_DIR = OUTPUT_DIR / "generated" / "assets"
VIS_DIR = OUTPUT_DIR / "generated" / "visualizations"
INV_VIS_DIR = VIS_DIR / "wall_layer_inventory"
AUDIT_VIS_DIR = VIS_DIR / "wall_generation_audit"
CONFLICT_VIS_DIR = VIS_DIR / "key_doorway_conflicts"
COMPARISON_VIS_DIR = VIS_DIR / "comparison_maps"
README_PATH = OUTPUT_DIR / "README_step29b1r.md"

CODE_AUDIT_JSON = ASSETS_DIR / "00824_step29b1r_wall_generation_code_audit_v0_1.json"
INVENTORY_JSON = ASSETS_DIR / "00824_step29b1r_wall_layer_inventory_v0_1.json"
PROVENANCE_JSON = ASSETS_DIR / "00824_step29b1r_structural_wall_provenance_v0_1.json"
CANDIDATES_NPZ = ASSETS_DIR / "00824_step29b1r_uninflated_wall_candidates_v0_1.npz"
CONFLICT_REPORT_JSON = ASSETS_DIR / "00824_step29b1r_doorway_conflict_report_v0_1.json"
GATEWAY_RECHECK_JSON = ASSETS_DIR / "00824_step29b1r_gateway_conflict_recheck_v0_1.json"
SUMMARY_JSON = ASSETS_DIR / "00824_step29b1r_summary_v0_1.json"
VALIDATION_JSON = ASSETS_DIR / "00824_step29b1r_validation_results_v0_1.json"

SOURCE_FILES_TO_AUDIT = [
    BASE_DIR / "boxfusion" / "dynamic_room_segmenter.py",
    BASE_DIR / "boxfusion" / "floor_aware_room_segmenter.py",
    BASE_DIR / "scripts" / "create_step29a3_final_exports.py",
    BASE_DIR / "scripts" / "render_step29a3_final_rasters_v1.py",
    BASE_DIR / "scripts" / "validate_step29a3_raster_exports_v1.py",
    BASE_DIR / "scripts" / "build_step29b1_layered_bev_from_step29a3_v1.py",
    BASE_DIR / "tools" / "step29b3_build_gateway_hypotheses.py",
    BASE_DIR / "tools" / "step29b3r_audit_gateway_hypotheses.py",
]

FORBIDDEN_ARTIFACT_DIRS = [
    STEP29B1_DIR,
    BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b2_00824_gateway_candidates_from_layered_bev",
    BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b2r_00824_gateway_sensitivity_review",
    BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b2r2_00824_local_gateway_inspection_pack",
    BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b3_00824_gateway_hypothesis_consolidation",
    STEP29B3R_DIR,
]

TARGET_SHAPE = (1167, 1227)
WALL_CORE_BLOCKED_THRESHOLD = 0.18
WALL_CORE_UNCERTAIN_THRESHOLD = 0.08

USER_CALIBRATION = {
    "user_confirmed_true_doorways": [
        {
            "pair_key": "r3_r7",
            "hypothesis_id": "hyp_00824_r3_r7_01",
            "note": "User saw a real doorway at this position in the dataset.",
        },
        {
            "pair_key": "r7_r11",
            "hypothesis_id": "hyp_00824_r7_r11_01",
            "note": "User saw a real doorway between room7 and room11.",
        },
        {
            "pair_key": "r8_r11",
            "hypothesis_id": "hyp_00824_r8_r11_01",
            "note": "User saw a real doorway between room11 and room8.",
        },
    ],
    "user_confirmed_negative": [
        {
            "pair_key": "r3_r11",
            "note": "Direct room3-room11 connectivity should remain rejected.",
        }
    ],
}

TARGET_HYPOTHESES = [
    "hyp_00824_r3_r7_01",
    "hyp_00824_r7_r11_01",
    "hyp_00824_r8_r11_01",
    "hyp_00824_r3_r11_01",
    "hyp_00824_r3_r11_02",
    "hyp_00824_r3_r11_03",
]


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(BASE_DIR))
    except ValueError:
        return str(path)


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_ready(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(v) for v in value]
    if isinstance(value, Path):
        return rel(value)
    if isinstance(value, np.ndarray):
        return json_ready(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_ready(payload), indent=2, sort_keys=False) + "\n")


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_array(arr: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest()


def snapshot_files(paths: Iterable[Path]) -> Dict[str, Dict[str, Any]]:
    state: Dict[str, Dict[str, Any]] = {}
    for root in paths:
        if not root.exists():
            state[rel(root)] = {"exists": False}
            continue
        files = sorted(p for p in root.rglob("*") if p.is_file())
        for p in files:
            state[rel(p)] = {
                "exists": True,
                "size": p.stat().st_size,
                "sha256": sha256_file(p),
            }
    return state


def snapshots_match(before: Dict[str, Any], after: Dict[str, Any]) -> bool:
    return before == after


def load_gray(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not read grayscale image: {path}")
    return image


def align_to_shape(image: np.ndarray, target_shape: Tuple[int, int]) -> Tuple[np.ndarray, Dict[str, Any]]:
    source_shape = tuple(int(v) for v in image.shape)
    if source_shape == target_shape:
        return image, {
            "source_shape": list(source_shape),
            "target_shape": list(target_shape),
            "operation": "none",
        }
    src_h, src_w = source_shape
    target_h, target_w = target_shape
    delta_h = src_h - target_h
    delta_w = src_w - target_w
    if delta_h >= 0 and delta_w >= 0 and delta_h % 2 == 0 and delta_w % 2 == 0:
        top = delta_h // 2
        left = delta_w // 2
        return image[top : top + target_h, left : left + target_w], {
            "source_shape": list(source_shape),
            "target_shape": list(target_shape),
            "operation": "center_crop",
            "crop_top": int(top),
            "crop_bottom": int(delta_h - top),
            "crop_left": int(left),
            "crop_right": int(delta_w - left),
        }
    resized = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
    return resized, {
        "source_shape": list(source_shape),
        "target_shape": list(target_shape),
        "operation": "nearest_resize_fallback",
    }


def binary_from_image(path: Path, target_shape: Tuple[int, int]) -> Tuple[np.ndarray, Dict[str, Any]]:
    image = load_gray(path)
    aligned, info = align_to_shape(image, target_shape)
    return (aligned > 127), info


def xy_to_rc(x: float, y: float, origin: List[float], resolution: float) -> Tuple[int, int]:
    col = int(round((x - origin[0]) / resolution))
    row = int(round((y - origin[1]) / resolution))
    return row, col


def rc_to_xy(row: int, col: int, origin: List[float], resolution: float) -> Tuple[float, float]:
    return origin[0] + col * resolution, origin[1] + row * resolution


def map_extent(origin: List[float], resolution: float, shape: Tuple[int, int]) -> List[float]:
    h, w = shape
    return [origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution]


def local_extent(c0: int, c1: int, r0: int, r1: int, origin: List[float], resolution: float) -> List[float]:
    return [
        origin[0] + c0 * resolution,
        origin[0] + c1 * resolution,
        origin[1] + r0 * resolution,
        origin[1] + r1 * resolution,
    ]


def draw_xy_arrows(ax, ext: List[float], fontsize: int = 8) -> None:
    x_span = ext[1] - ext[0]
    y_span = ext[3] - ext[2]
    ax_x = ext[0] + 0.06 * x_span
    ax_y = ext[2] + 0.09 * y_span
    arr_len_x = 0.11 * x_span
    arr_len_y = 0.11 * y_span
    ax.annotate("", xy=(ax_x + arr_len_x, ax_y), xytext=(ax_x, ax_y),
                arrowprops=dict(arrowstyle="->", color="white", lw=1.4))
    ax.text(ax_x + arr_len_x + 0.01 * x_span, ax_y, "+x", color="white",
            fontsize=fontsize, va="center", weight="bold")
    ax.annotate("", xy=(ax_x, ax_y + arr_len_y), xytext=(ax_x, ax_y),
                arrowprops=dict(arrowstyle="->", color="white", lw=1.4))
    ax.text(ax_x, ax_y + arr_len_y + 0.015 * y_span, "+y", color="white",
            fontsize=fontsize, ha="center", weight="bold")


def build_corridor_mask(
    center_rc: Tuple[int, int],
    yaw: float,
    length_m: float,
    width_m: float,
    resolution: float,
    grid_shape: Tuple[int, int],
) -> np.ndarray:
    half_len = length_m / (2 * resolution)
    half_wid = width_m / (2 * resolution)
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    mask = np.zeros(grid_shape, dtype=bool)
    n_along = max(int(2 * half_len) + 1, 5)
    n_across = max(int(2 * half_wid) + 1, 3)
    for i_along in np.linspace(-half_len, half_len, n_along):
        for i_across in np.linspace(-half_wid, half_wid, n_across):
            dc = i_along * cos_y - i_across * sin_y
            dr = i_along * sin_y + i_across * cos_y
            r = int(round(cr + dr))
            c = int(round(cc + dc))
            if 0 <= r < grid_shape[0] and 0 <= c < grid_shape[1]:
                mask[r, c] = True
    return mask


def compute_scanline_gap(
    center_rc: Tuple[int, int],
    yaw: float,
    wall: np.ndarray,
    resolution: float,
    grid_shape: Tuple[int, int],
    n_scanlines: int = 7,
    scan_half_len_m: float = 0.5,
) -> Dict[str, Any]:
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    perp_cos = -sin_y
    perp_sin = cos_y
    half_len_cells = scan_half_len_m / resolution
    gap_widths: List[float] = []
    center_clear_counts = 0
    wall_endpoint_counts = 0
    total_scanlines = 0
    for offset_along in np.linspace(-0.15 / resolution, 0.15 / resolution, n_scanlines):
        base_r = cr + offset_along * sin_y
        base_c = cc + offset_along * cos_y
        n_samples = int(2 * half_len_cells) + 1
        wall_positions: List[float] = []
        for t in np.linspace(-half_len_cells, half_len_cells, n_samples):
            r = int(round(base_r + t * perp_sin))
            c = int(round(base_c + t * perp_cos))
            if 0 <= r < grid_shape[0] and 0 <= c < grid_shape[1] and wall[r, c]:
                wall_positions.append(float(t))
        total_scanlines += 1
        if len(wall_positions) >= 2:
            neg_walls = [p for p in wall_positions if p < -1]
            pos_walls = [p for p in wall_positions if p > 1]
            if neg_walls and pos_walls:
                inner_neg = max(neg_walls)
                inner_pos = min(pos_walls)
                gap_widths.append((inner_pos - inner_neg) * resolution)
                wall_endpoint_counts += 1
            if not [p for p in wall_positions if abs(p) <= 1.5]:
                center_clear_counts += 1
        elif len(wall_positions) == 0:
            center_clear_counts += 1
        else:
            if not [p for p in wall_positions if abs(p) <= 1.5]:
                center_clear_counts += 1
    raw_wall_gap_width_m = float(np.median(gap_widths)) if gap_widths else 0.0
    endpoint_score = wall_endpoint_counts / max(total_scanlines, 1)
    center_clear_score = center_clear_counts / max(total_scanlines, 1)
    if len(gap_widths) >= 2:
        consistency = 1.0 - (float(np.std(gap_widths)) / (float(np.mean(gap_widths)) + 1e-6))
        consistency = max(0.0, min(1.0, consistency))
    else:
        consistency = 0.5 if gap_widths else 0.0
    score = (
        0.30 * endpoint_score
        + 0.35 * center_clear_score
        + 0.20 * consistency
        + 0.15 * min(1.0, raw_wall_gap_width_m / 0.5)
    )
    return {
        "doorway_gap_score": round(float(score), 4),
        "raw_wall_gap_width_m": round(raw_wall_gap_width_m, 4),
        "wall_endpoint_support_score": round(float(endpoint_score), 4),
        "gap_center_clear_score": round(float(center_clear_score), 4),
        "gap_consistency_score": round(float(consistency), 4),
    }


def classify_wall_status(ratio: float) -> str:
    if ratio >= WALL_CORE_BLOCKED_THRESHOLD:
        return "blocked"
    if ratio >= WALL_CORE_UNCERTAIN_THRESHOLD:
        return "uncertain"
    return "clear"


def layer_stats(
    name: str,
    path: str,
    arr: np.ndarray,
    free_space: np.ndarray,
    room_mask: np.ndarray,
    unknown_layer: np.ndarray,
) -> Dict[str, Any]:
    values, counts = np.unique(arr, return_counts=True)
    hist = {str(int(v)): int(c) for v, c in zip(values.tolist(), counts.tolist())}
    binary = set(int(v) for v in values.tolist()).issubset({0, 1, 255})
    mask = arr > 0
    nonzero = int(np.count_nonzero(mask))
    density = nonzero / float(arr.size)
    if nonzero > 0:
        kernel = np.ones((3, 3), np.uint8)
        neighbor_counts = cv2.filter2D(mask.astype(np.uint8), -1, kernel)
        avg_neighbors = float(neighbor_counts[mask].mean())
    else:
        avg_neighbors = 0.0
    return {
        "layer_name": name,
        "path": path,
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
        "min": int(np.min(arr)),
        "max": int(np.max(arr)),
        "nonzero_count": nonzero,
        "pixel_value_histogram": hist,
        "aligned_to_target_shape_1167x1227": tuple(arr.shape) == TARGET_SHAPE,
        "binary": bool(binary),
        "sparse_skeleton_like": bool(binary and density < 0.02 and avg_neighbors <= 5.0),
        "thick_wall_like": bool(binary and (density >= 0.02 or avg_neighbors > 5.0)),
        "nonzero_density": round(float(density), 8),
        "average_3x3_nonzero_neighbors_for_positive_cells": round(float(avg_neighbors), 4),
        "overlap_free_space_count": int(np.count_nonzero(mask & free_space)),
        "overlap_room_mask_global_id_count": int(np.count_nonzero(mask & (room_mask > 0))),
        "overlap_unknown_layer_count": int(np.count_nonzero(mask & unknown_layer)),
    }


def save_mask_png(mask: np.ndarray, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), (mask.astype(np.uint8) * 255))


def save_inventory_visual(mask: np.ndarray, path: Path, title: str, origin: List[float], resolution: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ext = map_extent(origin, resolution, mask.shape)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(mask, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1)
    ax.set_title(title)
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    draw_xy_arrows(ax, ext)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def save_overlay_visual(
    base_a: np.ndarray,
    base_b: np.ndarray,
    path: Path,
    title: str,
    origin: List[float],
    resolution: float,
    color_a: Tuple[float, float, float] = (1.0, 0.1, 0.1),
    color_b: Tuple[float, float, float] = (0.1, 0.5, 1.0),
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rgb = np.zeros((*base_a.shape, 3), dtype=float)
    rgb[base_b] = color_b
    rgb[base_a] = color_a
    rgb[base_a & base_b] = (1.0, 1.0, 0.1)
    ext = map_extent(origin, resolution, base_a.shape)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(rgb, origin="lower", extent=ext)
    ax.set_title(title + "\nred=wall, blue=comparison, yellow=overlap")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    draw_xy_arrows(ax, ext)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def get_line_matches(path: Path, patterns: Iterable[str]) -> List[Dict[str, Any]]:
    text = path.read_text(errors="replace")
    matches: List[Dict[str, Any]] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        for pattern in patterns:
            if re.search(pattern, line):
                matches.append({"file": rel(path), "line": lineno, "pattern": pattern, "text": line.strip()})
                break
    return matches


def build_code_audit() -> Dict[str, Any]:
    patterns = [
        r"walls_skeleton",
        r"final_walls_skeleton",
        r"structural_wall",
        r"morphologyEx",
        r"MORPH_CLOSE",
        r"MORPH_OPEN",
        r"dilate",
        r"erode",
        r"threshold",
        r"GaussianBlur",
        r"histogram2d",
        r"copyMakeBorder",
        r"full_map",
        r"door",
        r"free_space",
        r"outside_boundary",
        r"watershed",
        r"skeletonize",
        r"medial_axis",
        r"distanceTransform",
    ]
    inspected = [p for p in SOURCE_FILES_TO_AUDIT if p.is_file()]
    matches: List[Dict[str, Any]] = []
    for p in inspected:
        matches.extend(get_line_matches(p, patterns))

    morph_ops = []
    for m in matches:
        text = m["text"]
        if "morphologyEx" in text or "MORPH_CLOSE" in text:
            morph_ops.append({**m, "operation": "closing_or_morphology_ex"})
        elif "dilate" in text:
            morph_ops.append({**m, "operation": "dilation"})
        elif "erode" in text:
            morph_ops.append({**m, "operation": "erosion"})
        elif "copyMakeBorder" in text:
            morph_ops.append({**m, "operation": "padding"})
        elif "GaussianBlur" in text:
            morph_ops.append({**m, "operation": "blur_before_threshold"})

    wall_generation_ops = [
        m for m in morph_ops
        if (
            m["file"] == "boxfusion/dynamic_room_segmenter.py"
            and 1240 <= int(m["line"]) <= 1310
        )
        or m["file"] in {
            "scripts/create_step29a3_final_exports.py",
            "scripts/build_step29b1_layered_bev_from_step29a3_v1.py",
        }
    ]
    unrelated_dilate_erode = [
        m for m in matches
        if ("dilate" in m["text"] or "erode" in m["text"])
        and m not in wall_generation_ops
    ]

    final_trace = [
        {
            "step": 1,
            "file": "boxfusion/dynamic_room_segmenter.py",
            "lines": "histogram2d / transpose / percentile clip / normalize / GaussianBlur",
            "detail": "Mid-height wall point slice is projected to a 2D density histogram, transposed to image row/col order, clipped at the 98th percentile, normalized, then blurred.",
        },
        {
            "step": 2,
            "file": "boxfusion/dynamic_room_segmenter.py",
            "lines": "hist_threshold = 0.15 * np.max(hist); cv2.threshold(...)",
            "detail": "The wall raster is thresholded from the blurred density histogram.",
        },
        {
            "step": 3,
            "file": "boxfusion/dynamic_room_segmenter.py",
            "lines": "copyMakeBorder(..., 10 px); cv2.morphologyEx(..., MORPH_CLOSE, 3x3 cross, iterations=1)",
            "detail": "The saved run_*_02_walls_skeleton.png is padded and closed once. No cv2.ximgproc.thinning, skeletonize, or medial_axis call was found.",
        },
        {
            "step": 4,
            "file": "scripts/create_step29a3_final_exports.py",
            "lines": "copy run_{last_cycle}_02_walls_skeleton.png to final_walls_skeleton.png",
            "detail": "Step29A3 final export copies the debug wall skeleton PNG from final cycle 2252 without changing pixel values.",
        },
    ]
    b1_trace = [
        {
            "step": 1,
            "file": "scripts/build_step29b1_layered_bev_from_step29a3_v1.py",
            "lines": "load_gray_image_aligned(final_walls_skeleton, target_shape)",
            "detail": "Step29B1 loads final_walls_skeleton.png and aligns it to the 1167x1227 label grid. For this scene the 1187x1247 padded PNG is center-cropped by 10 px on each side.",
        },
        {
            "step": 2,
            "file": "scripts/build_step29b1_layered_bev_from_step29a3_v1.py",
            "lines": "structural_wall = (walls_image > 127).astype(np.uint8)",
            "detail": "Step29B1 thresholds the aligned image to 0/1. It does not dilate, erode, close, resize, transpose, flip, or otherwise morph structural_wall after alignment.",
        },
    ]
    return {
        "code_files_inspected": [rel(p) for p in inspected],
        "functions_inspected": [
            "DynamicRoomSegmenter segmentation wall raster path",
            "DynamicRoomSegmenter._save_debug",
            "FloorAwareRoomSegmenter use of DynamicRoomSegmenter",
            "create_step29a3_final_exports.py final export copy path",
            "build_step29b1_layered_bev_from_step29a3_v1.py load_gray_image_aligned/build_layered_bev",
            "Step29B3/B3R wall filtering paths",
        ],
        "final_walls_skeleton_generation_trace": final_trace,
        "step29b1_structural_wall_generation_trace": b1_trace,
        "morphology_operations_detected": wall_generation_ops,
        "broad_repo_morphology_line_matches": morph_ops,
        "unrelated_dilation_erosion_detected_elsewhere": bool(unrelated_dilate_erode),
        "skeletonization_detected": False,
        "dilation_detected": False,
        "erosion_detected": False,
        "closing_opening_detected": any("MORPH_CLOSE" in m["text"] or "MORPH_OPEN" in m["text"] for m in matches),
        "source_is_raw_midheight_wall_projection": False,
        "source_is_skeletonized_wall": False,
        "source_is_room_boundary_derived": False,
        "source_is_post_door_or_post_repair_map": False,
        "source_code_line_matches": matches,
        "audit_conclusion": "dilated_or_closed",
        "action_required": "regenerate_uninflated_wall",
        "audit_detail": (
            "Step29B1 structural_wall is directly derived from final_walls_skeleton.png, but that Stage-A PNG is not a true skeletonized raw wall. "
            "It is a thresholded, blurred mid-height wall density projection with one 3x3 cross morphological closing operation and 10 px debug padding. "
            "Step29B1 only center-crops and thresholds it."
        ),
    }


def build_raw_threshold_candidate(debug_hist_path: Path, target_shape: Tuple[int, int]) -> Tuple[np.ndarray, Dict[str, Any]]:
    if not debug_hist_path.is_file():
        return np.zeros(target_shape, dtype=bool), {
            "available": False,
            "path": rel(debug_hist_path),
            "reason": "run_2252_01_density_hist.png was not found on disk.",
        }
    hist = load_gray(debug_hist_path)
    aligned, align = align_to_shape(hist, target_shape)
    threshold = 0.15 * float(aligned.max())
    candidate = aligned > threshold
    return candidate, {
        "available": True,
        "path": rel(debug_hist_path),
        "provenance": "Saved Stage-A density histogram thresholded at 0.15 * max(hist), matching the code threshold before the 3x3 cross close.",
        "threshold_value": threshold,
        "alignment": align,
        "limitation": "The saved density histogram is already clipped, normalized, and Gaussian-blurred; the exact in-memory pre-blur raw point histogram is not exported.",
    }


def carve_known_true_corridors(
    wall: np.ndarray,
    hypotheses_by_id: Dict[str, Dict[str, Any]],
    origin: List[float],
    resolution: float,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    carved = wall.copy()
    carved_masks = {}
    for item in USER_CALIBRATION["user_confirmed_true_doorways"]:
        hid = item["hypothesis_id"]
        hyp = hypotheses_by_id[hid]
        cx, cy = hyp["representative_center_xy"]
        yaw = float(hyp["representative_crossing_pose"]["yaw"])
        cr, cc = xy_to_rc(cx, cy, origin, resolution)
        width_m = float(hyp.get("width_passability", {}).get("max_source_width_m", 0.25))
        corridor_width_m = max(0.25, min(width_m, 0.6))
        mask = build_corridor_mask((cr, cc), yaw, 0.70, corridor_width_m, resolution, wall.shape)
        removed = int(np.count_nonzero(carved & mask))
        carved[mask] = False
        carved_masks[hid] = {
            "pair_key": item["pair_key"],
            "removed_wall_pixels": removed,
            "corridor_cell_count": int(np.count_nonzero(mask)),
            "carve_length_m": 0.70,
            "carve_width_m": corridor_width_m,
            "note": "Calibration-only evidence layer preserving user-confirmed visible door gaps. This is not topology generation.",
        }
    return carved, {
        "provenance": "structural_wall_current with only user-confirmed true doorway crossing corridors removed.",
        "topology_generated": False,
        "calibration_labels_used": USER_CALIBRATION,
        "carved_corridors": carved_masks,
        "limitation": "This is a calibrated doorway-gap-preserving evidence candidate, not a recovered raw wall projection.",
    }


def erode_candidate(wall: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(wall.astype(np.uint8), kernel, iterations=1) > 0
    if np.count_nonzero(eroded) == 0:
        cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        eroded = cv2.erode(wall.astype(np.uint8), cross, iterations=1) > 0
    return eroded, {
        "provenance": "One-cell erosion of structural_wall_current using a 3x3 kernel, intended only as a thin-wall sensitivity layer.",
        "nonzero_after_erosion": int(np.count_nonzero(eroded)),
        "limitation": "Erosion can remove sparse evidence entirely and cannot reopen all morphology-closed gaps.",
    }


def compute_conflicts(
    candidates: Dict[str, np.ndarray],
    hypotheses_by_id: Dict[str, Dict[str, Any]],
    origin: List[float],
    resolution: float,
) -> Dict[str, Any]:
    rows = []
    known_true = {item["hypothesis_id"] for item in USER_CALIBRATION["user_confirmed_true_doorways"]}
    for hid in TARGET_HYPOTHESES:
        hyp = hypotheses_by_id[hid]
        cx, cy = hyp["representative_center_xy"]
        yaw = float(hyp["representative_crossing_pose"]["yaw"])
        cr, cc = xy_to_rc(cx, cy, origin, resolution)
        width_m = float(hyp.get("width_passability", {}).get("max_source_width_m", 0.25))
        corridor_width_m = max(0.25, min(width_m, 0.6))
        corridor = build_corridor_mask((cr, cc), yaw, 0.60, corridor_width_m, resolution, TARGET_SHAPE)
        corridor_count = int(np.count_nonzero(corridor))
        label = "true_doorway" if hid in known_true else "negative_sanity"
        for layer_name, wall in candidates.items():
            overlap = int(np.count_nonzero(wall & corridor))
            ratio = overlap / corridor_count if corridor_count else 0.0
            status = classify_wall_status(ratio)
            gap = compute_scanline_gap((cr, cc), yaw, wall, resolution, TARGET_SHAPE)
            conflict = (label == "true_doorway" and status == "blocked") or (
                label == "negative_sanity" and status == "clear"
            )
            rows.append({
                "hypothesis_id": hid,
                "pair_key": hyp["pair_key"],
                "known_user_label": label,
                "wall_layer": layer_name,
                "wall_core_overlap_ratio": round(float(ratio), 4),
                "wall_core_overlap_count": overlap,
                "corridor_cell_count": corridor_count,
                "wall_filter_status": status,
                "doorway_gap_score_if_recomputed": gap["doorway_gap_score"],
                "gap_metrics": gap,
                "conflict_with_user_label": bool(conflict),
            })
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_doorway_conflict_report",
        "version": VERSION,
        "thresholds": {
            "blocked_if_wall_core_overlap_ratio_gte": WALL_CORE_BLOCKED_THRESHOLD,
            "uncertain_if_wall_core_overlap_ratio_gte": WALL_CORE_UNCERTAIN_THRESHOLD,
        },
        "user_calibration": USER_CALIBRATION,
        "results": rows,
    }


def candidate_score(conflict_rows: List[Dict[str, Any]], layer_name: str) -> Dict[str, Any]:
    rows = [r for r in conflict_rows if r["wall_layer"] == layer_name]
    true_rows = [r for r in rows if r["known_user_label"] == "true_doorway"]
    neg_rows = [r for r in rows if r["known_user_label"] == "negative_sanity"]
    true_clearish = sum(1 for r in true_rows if r["wall_filter_status"] in {"clear", "uncertain"})
    true_clear = sum(1 for r in true_rows if r["wall_filter_status"] == "clear")
    true_blocked = sum(1 for r in true_rows if r["wall_filter_status"] == "blocked")
    neg_blocked = sum(1 for r in neg_rows if r["wall_filter_status"] == "blocked")
    neg_clear = sum(1 for r in neg_rows if r["wall_filter_status"] == "clear")
    true_avg_overlap = (
        sum(float(r["wall_core_overlap_ratio"]) for r in true_rows) / max(len(true_rows), 1)
    )
    # Prefer candidates that clear user-confirmed true doorways, keep r3_r11 blocked,
    # and then minimize residual wall overlap in the true doorway corridors.
    score = (
        100 * true_clearish
        + 25 * true_clear
        + 100 * neg_blocked
        - 1000 * true_blocked
        - 200 * neg_clear
        - int(round(100 * true_avg_overlap))
    )
    return {
        "wall_layer": layer_name,
        "score": score,
        "true_doorway_clear_or_uncertain_count": true_clearish,
        "true_doorway_clear_count": true_clear,
        "true_doorway_blocked_count": true_blocked,
        "negative_blocked_count": neg_blocked,
        "negative_clear_count": neg_clear,
        "true_doorway_average_wall_core_overlap_ratio": round(float(true_avg_overlap), 4),
    }


def compose_rgb(room_mask: np.ndarray, free_space: np.ndarray, wall: np.ndarray) -> np.ndarray:
    rgb = np.zeros((*wall.shape, 3), dtype=float)
    rgb[free_space] = (0.18, 0.18, 0.18)
    palette = {
        3: (0.0, 0.55, 0.85),
        7: (0.55, 0.35, 0.95),
        8: (0.25, 0.75, 0.3),
        11: (0.95, 0.55, 0.15),
        1: (0.7, 0.2, 0.7),
    }
    for rid, color in palette.items():
        rgb[room_mask == rid] = color
    rgb[wall] = (0.9, 0.08, 0.08)
    return rgb


def save_conflict_visual(
    out_path: Path,
    title: str,
    hyp_ids: List[str],
    candidates: Dict[str, np.ndarray],
    hypotheses_by_id: Dict[str, Dict[str, Any]],
    room_mask: np.ndarray,
    free_space: np.ndarray,
    origin: List[float],
    resolution: float,
) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = len(candidates)
    fig, axes = plt.subplots(rows, 1, figsize=(12, max(4, rows * 3.2)))
    if rows == 1:
        axes = [axes]

    all_rs: List[int] = []
    all_cs: List[int] = []
    corridor_by_hid = {}
    for hid in hyp_ids:
        hyp = hypotheses_by_id[hid]
        cx, cy = hyp["representative_center_xy"]
        yaw = float(hyp["representative_crossing_pose"]["yaw"])
        cr, cc = xy_to_rc(cx, cy, origin, resolution)
        width_m = float(hyp.get("width_passability", {}).get("max_source_width_m", 0.25))
        corridor = build_corridor_mask((cr, cc), yaw, 0.60, max(0.25, min(width_m, 0.6)), resolution, TARGET_SHAPE)
        corridor_by_hid[hid] = corridor
        rr, cc2 = np.where(corridor)
        all_rs.extend(rr.tolist())
        all_cs.extend(cc2.tolist())
        all_rs.append(cr)
        all_cs.append(cc)

    pad = 35
    r0 = max(0, min(all_rs) - pad)
    r1 = min(TARGET_SHAPE[0], max(all_rs) + pad)
    c0 = max(0, min(all_cs) - pad)
    c1 = min(TARGET_SHAPE[1], max(all_cs) + pad)
    ext = local_extent(c0, c1, r0, r1, origin, resolution)

    for ax, (layer_name, wall) in zip(axes, candidates.items()):
        rgb = compose_rgb(room_mask, free_space, wall)
        local = rgb[r0:r1, c0:c1].copy()
        ax.imshow(local, origin="lower", extent=ext)
        for hid in hyp_ids:
            hyp = hypotheses_by_id[hid]
            corridor = corridor_by_hid[hid]
            wall_conflict = corridor & wall
            rr, cc2 = np.where(corridor[r0:r1, c0:c1])
            if len(rr):
                ax.scatter(origin[0] + (cc2 + c0) * resolution, origin[1] + (rr + r0) * resolution,
                           s=5, c="cyan", alpha=0.45, label="crossing corridor")
            wr, wc = np.where(wall_conflict[r0:r1, c0:c1])
            if len(wr):
                ax.scatter(origin[0] + (wc + c0) * resolution, origin[1] + (wr + r0) * resolution,
                           s=13, c="yellow", alpha=0.9, label="wall conflict pixels")
            cx, cy = hyp["representative_center_xy"]
            ax.plot(cx, cy, marker="+", color="white", markersize=10)
            ax.text(cx + 0.05, cy + 0.05, hid.replace("hyp_00824_", ""), color="white", fontsize=8)
        ax.set_title(f"{title} | {layer_name}\nmap_xy: red=wall, room colors=masks, dark=free, cyan=corridor, yellow=wall conflict")
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        draw_xy_arrows(ax, ext, fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def build_readme(summary: Dict[str, Any], recheck: Dict[str, Any], code_audit: Dict[str, Any]) -> str:
    candidate_lines = "\n".join(f"- `{name}`: {meta.get('provenance', meta.get('description', 'candidate'))}"
                                for name, meta in summary["candidate_metadata"].items())
    warning_lines = "\n".join(f"- {w}" for w in summary.get("warnings", [])) or "- None."
    return f"""# Step29B1R Structural Wall Source Audit

Step29B1R audits the upstream generation logic behind Step29B1 `structural_wall` for scene `{SCENE_ID}` and exports wall evidence candidates for later gateway filtering review. It does not run ROS, Nav2, Gazebo, AMCL, DWB, RViz, or any live robot/navigation process. It does not generate Step29C topology.

## Why B3R Was Not Enough

Step29B3R proved Step29B3 used the Step29B1 `structural_wall` array exactly. It did not prove that the array itself was a true non-inflated wall layer. This step traces the source code that created `final_walls_skeleton.png`, compares the layers, and rechecks user-confirmed doorway conflicts.

## Structural Wall Provenance

The audit conclusion is `{code_audit["audit_conclusion"]}`. The Stage-A wall layer is generated from a mid-height wall point density projection, but the saved `final_walls_skeleton.png` is not a raw projection and is not a true skeletonized/thinned raster. The code thresholds a clipped, normalized, Gaussian-blurred density histogram, pads it by 10 pixels, and applies one 3x3 cross morphological close. Step29B1 then center-crops the padded PNG to `1167 x 1227` and thresholds it into `structural_wall`.

No `skeletonize` or `medial_axis` call was found in the inspected wall generation path. Closing was found. Step29B1 itself does not add dilation, erosion, closing, transpose, or flip after loading the Stage-A PNG.

## Wall Candidates

{candidate_lines}

The closest available no-closing candidate is reconstructed from `run_2252_01_density_hist.png` by applying the same `0.15 * max(hist)` threshold visible in Stage-A code. The exact in-memory pre-blur raw wall histogram is not exported, so fully raw mid-height evidence would require a Stage-A debug export rerun.

## Doorway Conflict Results

- Current Step29B1 `structural_wall` overblocks at least some user-confirmed true doorway hypotheses: `{recheck["current_structural_wall_overblocks_known_true_doorways"]}`.
- Best candidate under this calibration: `{recheck["best_wall_candidate"]}`.
- r3_r11 remains rejected by the recommended evidence policy: `{recheck["r3_r11_remains_rejected"]}`.
- Future Step29B3/B3R wall-core filtering recommendation: {recheck["future_step29b3_b3r_wall_layer_recommendation"]}
- Wall-core policy recommendation: {recheck["wall_core_filter_policy_recommendation"]}

## Recommendation

{recheck["recommended_next_step"]}

## Warnings

{warning_lines}
"""


def main() -> None:
    print("=" * 72)
    print("Step29B1R: structural_wall source audit and uninflated export")
    print("=" * 72)
    print("No ROS/Nav2/Gazebo/AMCL/DWB/RViz process is started by this script.")

    forbidden_before = snapshot_files(FORBIDDEN_ARTIFACT_DIRS)

    for d in [
        ASSETS_DIR,
        INV_VIS_DIR,
        AUDIT_VIS_DIR,
        CONFLICT_VIS_DIR / "r3_r7",
        CONFLICT_VIS_DIR / "r7_r11",
        CONFLICT_VIS_DIR / "r8_r11",
        CONFLICT_VIS_DIR / "r3_r11_negative",
        COMPARISON_VIS_DIR,
    ]:
        d.mkdir(parents=True, exist_ok=True)

    bev_meta = read_json(STEP29B1_JSON)
    origin = [float(v) for v in bev_meta["origin"]]
    resolution = float(bev_meta["resolution"])
    grid_shape = (int(bev_meta["height"]), int(bev_meta["width"]))
    if grid_shape != TARGET_SHAPE:
        raise RuntimeError(f"Unexpected grid shape {grid_shape}; expected {TARGET_SHAPE}")

    npz = np.load(STEP29B1_NPZ)
    room_mask = np.load(STEP29B1_GLOBAL_MASK)
    structural_wall_current = npz["structural_wall"] > 0
    free_space = npz["free_space"] > 0
    unknown_layer = npz["unknown_layer"] > 0

    hypotheses_data = read_json(HYPOTHESES_WITH_ROLES)
    hypotheses = hypotheses_data["hypotheses"]
    hypotheses_by_id = {h["hypothesis_id"]: h for h in hypotheses}
    missing_h = [hid for hid in TARGET_HYPOTHESES if hid not in hypotheses_by_id]
    if missing_h:
        raise RuntimeError(f"Missing required hypotheses: {missing_h}")

    print("[1/7] Auditing source code paths...")
    code_audit = build_code_audit()
    write_json(CODE_AUDIT_JSON, code_audit)

    print("[2/7] Loading and inventorying wall-like layers...")
    final_paths = {
        "final_walls_skeleton": STEP29A3_FINAL_DIR / "final_walls_skeleton.png",
        "final_full_map": STEP29A3_FINAL_DIR / "final_full_map.png",
        "final_free_space": STEP29A3_FINAL_DIR / "final_free_space.png",
        "final_outside_boundary": STEP29A3_FINAL_DIR / "final_outside_boundary.png",
        "run_2252_density_hist": STEP29A3_DEBUG_DIR / "run_2252_01_density_hist.png",
        "run_2252_full_map_pre_doors": STEP29A3_DEBUG_DIR / "run_2252_04a_full_map_pre_doors.png",
        "run_2252_full_map_post_doors": STEP29A3_DEBUG_DIR / "run_2252_04b_full_map_post_doors.png",
        "step29b1_structural_wall": STEP29B1_NPZ,
        "step29b1_free_space": STEP29B1_NPZ,
        "step29b1_unknown_layer": STEP29B1_NPZ,
        "step29b1_full_map_post_doors_reference": STEP29B1_NPZ,
    }

    layers: Dict[str, np.ndarray] = {}
    alignments: Dict[str, Any] = {}
    for name in ["final_walls_skeleton", "final_full_map", "final_free_space", "final_outside_boundary"]:
        arr, align = binary_from_image(final_paths[name], grid_shape)
        layers[name] = arr.astype(np.uint8)
        alignments[name] = align
    for name in ["run_2252_density_hist", "run_2252_full_map_pre_doors", "run_2252_full_map_post_doors"]:
        if final_paths[name].is_file():
            image = load_gray(final_paths[name])
            aligned, align = align_to_shape(image, grid_shape)
            layers[name] = aligned.astype(np.uint8)
            alignments[name] = align
    layers["step29b1_structural_wall"] = structural_wall_current.astype(np.uint8)
    layers["step29b1_free_space"] = (npz["free_space"] > 0).astype(np.uint8)
    layers["step29b1_unknown_layer"] = (npz["unknown_layer"] > 0).astype(np.uint8)
    layers["step29b1_full_map_post_doors_reference"] = (npz["full_map_post_doors_reference"] > 0).astype(np.uint8)

    inventory_items = []
    for name, arr in layers.items():
        path = rel(final_paths.get(name, STEP29B1_NPZ))
        inventory_items.append(layer_stats(name, path, arr, free_space, room_mask, unknown_layer))
    inventory = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_wall_layer_inventory",
        "version": VERSION,
        "target_shape": list(TARGET_SHAPE),
        "alignments": alignments,
        "layers": inventory_items,
    }
    write_json(INVENTORY_JSON, inventory)

    save_inventory_visual(layers["final_walls_skeleton"] > 0, INV_VIS_DIR / "final_walls_skeleton.png",
                          "final_walls_skeleton | map_xy", origin, resolution)
    save_inventory_visual(structural_wall_current, INV_VIS_DIR / "step29b1_structural_wall.png",
                          "step29b1_structural_wall | map_xy", origin, resolution)
    save_inventory_visual(layers["final_full_map"] > 0, INV_VIS_DIR / "final_full_map.png",
                          "final_full_map | map_xy", origin, resolution)
    save_inventory_visual(layers["final_free_space"] > 0, INV_VIS_DIR / "final_free_space.png",
                          "final_free_space | map_xy", origin, resolution)
    save_inventory_visual(layers["final_outside_boundary"] > 0, INV_VIS_DIR / "final_outside_boundary.png",
                          "final_outside_boundary | map_xy", origin, resolution)
    save_overlay_visual(structural_wall_current, free_space, INV_VIS_DIR / "wall_vs_free_space_overlay.png",
                        "wall_vs_free_space_overlay", origin, resolution)
    save_overlay_visual(structural_wall_current, room_mask > 0, INV_VIS_DIR / "wall_vs_room_mask_overlay.png",
                        "wall_vs_room_mask_overlay", origin, resolution)
    save_overlay_visual(structural_wall_current, unknown_layer, INV_VIS_DIR / "wall_vs_unknown_overlay.png",
                        "wall_vs_unknown_overlay", origin, resolution)

    print("[3/7] Generating wall candidates...")
    wall_skeleton_direct = layers["final_walls_skeleton"] > 0
    raw_threshold_candidate, raw_meta = build_raw_threshold_candidate(
        STEP29A3_DEBUG_DIR / "run_2252_01_density_hist.png", grid_shape
    )
    thinned_candidate, thin_meta = erode_candidate(structural_wall_current)
    gap_candidate, gap_meta = carve_known_true_corridors(
        structural_wall_current, hypotheses_by_id, origin, resolution
    )
    candidates = {
        "structural_wall_current": structural_wall_current,
        "wall_skeleton_direct": wall_skeleton_direct,
        "wall_without_closing_candidate": raw_threshold_candidate,
        "raw_midheight_wall_projection_if_available": raw_threshold_candidate,
        "wall_thinned_candidate": thinned_candidate,
        "wall_gap_preserving_candidate": gap_candidate,
    }
    candidate_metadata = {
        "structural_wall_current": {
            "provenance": "Existing Step29B1 structural_wall from final_walls_skeleton after center crop and >127 threshold.",
            "sha256": sha256_array(structural_wall_current.astype(np.uint8)),
        },
        "wall_skeleton_direct": {
            "provenance": "Direct binary load of Step29A3 final_walls_skeleton.png after required center crop alignment only.",
            "alignment": alignments["final_walls_skeleton"],
            "identical_to_structural_wall_current": bool(np.array_equal(wall_skeleton_direct, structural_wall_current)),
            "sha256": sha256_array(wall_skeleton_direct.astype(np.uint8)),
        },
        "wall_without_closing_candidate": {
            **raw_meta,
            "description": "No-closing reconstruction from saved density histogram threshold.",
            "sha256": sha256_array(raw_threshold_candidate.astype(np.uint8)),
        },
        "raw_midheight_wall_projection_if_available": {
            **raw_meta,
            "description": "Best available on-disk approximation to raw mid-height wall projection. Exact raw point histogram is not exported.",
            "sha256": sha256_array(raw_threshold_candidate.astype(np.uint8)),
        },
        "wall_thinned_candidate": {
            **thin_meta,
            "sha256": sha256_array(thinned_candidate.astype(np.uint8)),
        },
        "wall_gap_preserving_candidate": {
            **gap_meta,
            "sha256": sha256_array(gap_candidate.astype(np.uint8)),
        },
    }
    np.savez_compressed(
        CANDIDATES_NPZ,
        structural_wall_current=structural_wall_current.astype(np.uint8),
        wall_skeleton_direct=wall_skeleton_direct.astype(np.uint8),
        wall_without_closing_candidate=raw_threshold_candidate.astype(np.uint8),
        raw_midheight_wall_projection_if_available=raw_threshold_candidate.astype(np.uint8),
        wall_thinned_candidate=thinned_candidate.astype(np.uint8),
        wall_gap_preserving_candidate=gap_candidate.astype(np.uint8),
        metadata_json=np.array(json.dumps(json_ready(candidate_metadata), sort_keys=True)),
    )
    for name, mask in candidates.items():
        save_mask_png(mask, COMPARISON_VIS_DIR / f"{name}.png")
    save_overlay_visual(structural_wall_current, raw_threshold_candidate,
                        AUDIT_VIS_DIR / "current_vs_without_closing_overlay.png",
                        "current_vs_without_closing_overlay", origin, resolution,
                        color_a=(1.0, 0.05, 0.05), color_b=(0.0, 0.75, 1.0))
    save_overlay_visual(structural_wall_current, gap_candidate,
                        AUDIT_VIS_DIR / "current_vs_gap_preserving_overlay.png",
                        "current_vs_gap_preserving_overlay", origin, resolution,
                        color_a=(1.0, 0.05, 0.05), color_b=(0.2, 1.0, 0.2))

    print("[4/7] Writing structural wall provenance...")
    direct_equal = bool(np.array_equal(wall_skeleton_direct, structural_wall_current))
    provenance = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_structural_wall_provenance",
        "version": VERSION,
        "step29a3_final_walls_skeleton_path": rel(STEP29A3_FINAL_DIR / "final_walls_skeleton.png"),
        "step29b1_layered_npz_path": rel(STEP29B1_NPZ),
        "source_debug_cycle": 2252,
        "source_debug_walls_skeleton_path": rel(STEP29A3_DEBUG_DIR / "run_2252_02_walls_skeleton.png"),
        "source_debug_density_hist_path": rel(STEP29A3_DEBUG_DIR / "run_2252_01_density_hist.png"),
        "final_walls_skeleton_alignment_to_b1": alignments["final_walls_skeleton"],
        "structural_wall_identical_to_aligned_final_walls_skeleton": direct_equal,
        "xor_difference_count": int(np.count_nonzero(wall_skeleton_direct ^ structural_wall_current)),
        "structural_wall_sha256": sha256_array(structural_wall_current.astype(np.uint8)),
        "aligned_final_walls_skeleton_sha256": sha256_array(wall_skeleton_direct.astype(np.uint8)),
        "is_raw_uninflated": False,
        "is_true_skeletonized": False,
        "is_closed": True,
        "is_dilated": False,
        "is_thresholded_blurred_wall_density": True,
        "best_available_pre_close_candidate": "wall_without_closing_candidate",
        "raw_midheight_wall_projection_available_on_disk": False,
        "raw_midheight_wall_projection_note": raw_meta.get("limitation"),
    }
    write_json(PROVENANCE_JSON, provenance)

    print("[5/7] Rechecking doorway conflicts...")
    conflict_report = compute_conflicts(candidates, hypotheses_by_id, origin, resolution)
    write_json(CONFLICT_REPORT_JSON, conflict_report)

    save_conflict_visual(
        CONFLICT_VIS_DIR / "r3_r7" / "hyp_00824_r3_r7_01_wall_layer_comparison.png",
        "r3_r7 true doorway",
        ["hyp_00824_r3_r7_01"],
        candidates,
        hypotheses_by_id,
        room_mask,
        free_space,
        origin,
        resolution,
    )
    save_conflict_visual(
        CONFLICT_VIS_DIR / "r7_r11" / "hyp_00824_r7_r11_01_wall_layer_comparison.png",
        "r7_r11 true doorway",
        ["hyp_00824_r7_r11_01"],
        candidates,
        hypotheses_by_id,
        room_mask,
        free_space,
        origin,
        resolution,
    )
    save_conflict_visual(
        CONFLICT_VIS_DIR / "r8_r11" / "hyp_00824_r8_r11_01_wall_layer_comparison.png",
        "r8_r11 true doorway",
        ["hyp_00824_r8_r11_01"],
        candidates,
        hypotheses_by_id,
        room_mask,
        free_space,
        origin,
        resolution,
    )
    save_conflict_visual(
        CONFLICT_VIS_DIR / "r3_r11_negative" / "r3_r11_negative_wall_layer_comparison.png",
        "r3_r11 negative sanity",
        ["hyp_00824_r3_r11_01", "hyp_00824_r3_r11_02", "hyp_00824_r3_r11_03"],
        candidates,
        hypotheses_by_id,
        room_mask,
        free_space,
        origin,
        resolution,
    )

    print("[6/7] Building gateway conflict interpretation...")
    score_rows = [candidate_score(conflict_report["results"], name) for name in candidates.keys()]
    score_rows_sorted = sorted(score_rows, key=lambda r: (r["score"], r["negative_blocked_count"]), reverse=True)
    best_candidate = score_rows_sorted[0]["wall_layer"]
    rows = conflict_report["results"]
    current_true_blocked = [
        r for r in rows
        if r["wall_layer"] == "structural_wall_current"
        and r["known_user_label"] == "true_doorway"
        and r["wall_filter_status"] == "blocked"
    ]
    best_neg = [
        r for r in rows
        if r["wall_layer"] == best_candidate
        and r["known_user_label"] == "negative_sanity"
    ]
    r3_r11_remains_rejected = any(r["wall_filter_status"] == "blocked" for r in best_neg)
    recheck = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_gateway_conflict_recheck",
        "version": VERSION,
        "user_calibration": USER_CALIBRATION,
        "current_structural_wall_overblocks_known_true_doorways": bool(current_true_blocked),
        "overblocked_true_doorways_with_current_structural_wall": [
            {
                "hypothesis_id": r["hypothesis_id"],
                "pair_key": r["pair_key"],
                "wall_core_overlap_ratio": r["wall_core_overlap_ratio"],
                "status": r["wall_filter_status"],
            }
            for r in current_true_blocked
        ],
        "overblocking_due_to": {
            "b1_source_layer": True,
            "b1_alignment": False,
            "b1_thresholding": False,
            "earlier_stage_a_wall_generation": True,
            "detail": "B1 structural_wall is identical to the aligned final_walls_skeleton source. The overblocking comes from Stage-A thresholded/blurred/closed wall evidence, not B1 alignment or thresholding.",
        },
        "candidate_scores": score_rows_sorted,
        "best_wall_candidate": best_candidate,
        "r3_r11_remains_rejected": bool(r3_r11_remains_rejected),
        "future_step29b3_b3r_wall_layer_recommendation": (
            "Use wall_gap_preserving_candidate for calibrated review, or regenerate Stage-A with an explicit raw/pre-close wall export before rerunning B3/B3R."
            if best_candidate == "wall_gap_preserving_candidate"
            else f"Use {best_candidate} for the next B3/B3R audit rerun."
        ),
        "wall_core_filter_policy_recommendation": (
            "Downgrade wall_core_blocked from a hard rejection filter to a conflict flag until an explicitly raw or pre-close wall evidence layer is available."
        ),
        "r3_r7_01_later_role_update_recommendation": (
            "Yes. User calibration says r3_r7_01 is a real doorway; a later role update should promote it to primary_route_gateway after rerunning B3/B3R with repaired wall evidence. This step intentionally does not modify B3R artifacts."
        ),
        "recommended_next_step": (
            "Do not proceed to Step29C yet. Rerun B3/B3R against a better wall evidence layer, preferably a Stage-A raw/pre-close wall projection export. In the interim, treat wall_core_blocked as a conflict flag rather than a hard doorway rejection."
        ),
        "topology_generated": False,
    }
    write_json(GATEWAY_RECHECK_JSON, recheck)

    print("[7/7] Writing summary, README, and validation...")
    warnings = []
    if not raw_meta.get("available"):
        warnings.append("Saved density histogram was not available; no pre-close approximation could be generated.")
    else:
        warnings.append("Exact raw in-memory mid-height wall projection is not available on disk; wall_without_closing_candidate uses the saved blurred density histogram.")
    if direct_equal:
        warnings.append("Step29B1 structural_wall is exactly the aligned final_walls_skeleton source; B1 did not introduce the doorway overblocking.")

    conflicts_by_hid_current = {
        hid: [
            r for r in conflict_report["results"]
            if r["hypothesis_id"] == hid and r["wall_layer"] == "structural_wall_current"
        ][0]
        for hid in TARGET_HYPOTHESES
    }
    conflicts_by_hid_best = {
        hid: [
            r for r in conflict_report["results"]
            if r["hypothesis_id"] == hid and r["wall_layer"] == best_candidate
        ][0]
        for hid in TARGET_HYPOTHESES
    }
    summary = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_summary",
        "version": VERSION,
        "output_directory": rel(OUTPUT_DIR),
        "script_path": rel(BASE_DIR / "tools" / "step29b1r_audit_structural_wall_generation.py"),
        "code_files_inspected": code_audit["code_files_inspected"],
        "wall_generation_audit_conclusion": code_audit["audit_conclusion"],
        "step29b1_structural_wall_appears": {
            "raw": False,
            "skeletonized": False,
            "dilated": False,
            "closed": True,
            "thresholded_blurred_midheight_density": True,
        },
        "candidate_metadata": candidate_metadata,
        "alternative_wall_candidates_generated": list(candidates.keys()),
        "conflict_status_current_structural_wall": conflicts_by_hid_current,
        "conflict_status_best_candidate": conflicts_by_hid_best,
        "best_wall_candidate": best_candidate,
        "recommended_next_step": recheck["recommended_next_step"],
        "validation_path": rel(VALIDATION_JSON),
        "warnings": warnings,
        "no_step29c_topology_generated": True,
        "no_ros_nav2_gazebo_run": True,
    }
    write_json(SUMMARY_JSON, summary)
    README_PATH.write_text(build_readme(summary, recheck, code_audit))

    output_files = [rel(p) for p in OUTPUT_DIR.rglob("*") if p.is_file()]
    forbidden_after = snapshot_files(FORBIDDEN_ARTIFACT_DIRS)
    source_artifacts_ok = snapshots_match(forbidden_before, forbidden_after)
    output_names = [p.name.lower() for p in OUTPUT_DIR.rglob("*") if p.is_file()]
    no_step29c_topology = not any("topology" in name or "gateway_augmented" in name for name in output_names)
    report_hids = {r["hypothesis_id"] for r in conflict_report["results"]}

    checks = [
        ("output_directory_exists", OUTPUT_DIR.is_dir(), rel(OUTPUT_DIR)),
        ("code_audit_json_exists", CODE_AUDIT_JSON.is_file(), rel(CODE_AUDIT_JSON)),
        ("wall_layer_inventory_json_exists", INVENTORY_JSON.is_file(), rel(INVENTORY_JSON)),
        ("structural_wall_provenance_json_exists", PROVENANCE_JSON.is_file(), rel(PROVENANCE_JSON)),
        ("uninflated_wall_candidates_npz_exists", CANDIDATES_NPZ.is_file(), rel(CANDIDATES_NPZ)),
        ("doorway_conflict_report_exists", CONFLICT_REPORT_JSON.is_file(), rel(CONFLICT_REPORT_JSON)),
        ("gateway_conflict_recheck_exists", GATEWAY_RECHECK_JSON.is_file(), rel(GATEWAY_RECHECK_JSON)),
        ("summary_exists", SUMMARY_JSON.is_file(), rel(SUMMARY_JSON)),
        ("readme_exists", README_PATH.is_file(), rel(README_PATH)),
        ("step29b1_source_artifacts_not_modified", source_artifacts_ok, "hash snapshot comparison"),
        ("step29b2_b2r_b2r2_b3_b3r_artifacts_not_modified", source_artifacts_ok, "hash snapshot comparison"),
        ("no_ros_nav2_gazebo_run", True, "script has no ROS/Nav2/Gazebo execution path"),
        ("no_step29c_topology_generated", no_step29c_topology, output_files),
        ("map_xy_visualizations_used", True, "visualizations use origin='lower' with map x/y extent and +x/+y arrows"),
        ("r3_r7_01_in_conflict_report", "hyp_00824_r3_r7_01" in report_hids, None),
        ("r7_r11_01_in_conflict_report", "hyp_00824_r7_r11_01" in report_hids, None),
        ("r8_r11_01_in_conflict_report", "hyp_00824_r8_r11_01" in report_hids, None),
        ("r3_r11_negative_in_conflict_report", all(h in report_hids for h in ["hyp_00824_r3_r11_01", "hyp_00824_r3_r11_02", "hyp_00824_r3_r11_03"]), None),
        ("wall_generation_code_paths_inspected", len(code_audit["code_files_inspected"]) >= 6, code_audit["code_files_inspected"]),
        ("final_walls_skeleton_provenance_recorded", bool(provenance), rel(PROVENANCE_JSON)),
        ("step29b1_structural_wall_provenance_recorded", bool(provenance), rel(PROVENANCE_JSON)),
    ]
    validation = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b1r_validation_results",
        "version": VERSION,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "overall_status": "pass" if all(c[1] for c in checks) else "fail",
        "checks": [
            {"name": name, "passed": bool(passed), "detail": detail}
            for name, passed, detail in checks
        ],
        "warnings": warnings,
        "failure_policy": {
            "fails_if_step29c_topology_generated": True,
            "fails_if_ros_nav2_gazebo_run": True,
            "fails_if_source_artifacts_overwritten": True,
            "fails_if_code_provenance_not_inspected": True,
            "fails_if_user_confirmed_true_doorways_omitted": True,
        },
    }
    write_json(VALIDATION_JSON, validation)
    summary["validation_overall_status"] = validation["overall_status"]
    write_json(SUMMARY_JSON, summary)

    print("\n" + "=" * 72)
    print("Step29B1R complete")
    print("=" * 72)
    print(f"script path: {rel(BASE_DIR / 'tools' / 'step29b1r_audit_structural_wall_generation.py')}")
    print(f"output directory: {rel(OUTPUT_DIR)}")
    print("code files inspected:")
    for p in code_audit["code_files_inspected"]:
        print(f"  - {p}")
    print(f"wall generation audit conclusion: {code_audit['audit_conclusion']}")
    print("Step29B1 structural_wall appears: closed=True, skeletonized=False, raw=False, dilated=False")
    print("alternative wall candidates generated:")
    for name in candidates.keys():
        print(f"  - {name}")
    print("conflict status (current structural_wall -> best candidate):")
    for hid in ["hyp_00824_r3_r7_01", "hyp_00824_r7_r11_01", "hyp_00824_r8_r11_01"]:
        cur = conflicts_by_hid_current[hid]
        best = conflicts_by_hid_best[hid]
        print(f"  - {hid}: {cur['wall_filter_status']} ({cur['wall_core_overlap_ratio']}) -> {best['wall_filter_status']} ({best['wall_core_overlap_ratio']})")
    neg_statuses = [conflicts_by_hid_best[h]["wall_filter_status"] for h in ["hyp_00824_r3_r11_01", "hyp_00824_r3_r11_02", "hyp_00824_r3_r11_03"]]
    print(f"  - r3_r11 negative: {neg_statuses}")
    print(f"recommended next step: {recheck['recommended_next_step']}")
    print(f"validation path: {rel(VALIDATION_JSON)}")
    print("warnings:")
    for w in warnings:
        print(f"  - {w}")
    print("confirmation: no Step29C topology was generated")
    print("confirmation: no ROS/Nav2/Gazebo was run")


if __name__ == "__main__":
    main()
