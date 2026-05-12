#!/usr/bin/env python3
"""
Step29A3R: Export gateway-suitable wall evidence layers from Stage-A and
benchmark them against known doorway positives/negatives.

This script is offline only. It does not run ROS, Nav2, Gazebo, AMCL, DWB,
RViz, or any live navigation process. It does not generate Step29C topology.
"""

import hashlib
import json
import math
import os
import pathlib
from datetime import datetime

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
BASE_DIR = pathlib.Path("/home/ws/workspace/BoxFusion")
RUNTIME_DIR = BASE_DIR / "runtime_stage1_frozen_evidence"

STEP29A3_DIR = RUNTIME_DIR / "step29a3_00824_stage_a_debug_raster_export"
STEP29B1_DIR = RUNTIME_DIR / "step29b1_00824_layered_bev_from_step29a3"
STEP29B1R_DIR = RUNTIME_DIR / "step29b1r_00824_structural_wall_source_audit_and_uninflated_export"
STEP29B3R_DIR = RUNTIME_DIR / "step29b3r_00824_gateway_hypothesis_audit_and_roles"

SCENE_DIR = STEP29A3_DIR / "scenes" / SCENE_ID
FINAL_RASTER_DIR = SCENE_DIR / "final_raster_export"
DEBUG_ROOM_DIR = SCENE_DIR / "debug_room" / "floor_1"
LOGS_DIR = SCENE_DIR / "logs"

OUTPUT_DIR = RUNTIME_DIR / "step29a3r_00824_gateway_wall_evidence_export"
GENERATED_DIR = OUTPUT_DIR / "generated"
ASSETS_DIR = GENERATED_DIR / "assets"
VIS_DIR = GENERATED_DIR / "visualizations"

BEV_NPZ_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_layered_bev_from_step29a3_v0_1.npz"
GLOBAL_ROOM_MASK_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_global_room_mask_v0_1.npy"
BEV_META_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_layered_bev_from_step29a3_v0_1.json"
B1R_UNINFLATED_NPZ_PATH = STEP29B1R_DIR / "generated" / "assets" / "00824_step29b1r_uninflated_wall_candidates_v0_1.npz"
B1R_CODE_AUDIT_PATH = STEP29B1R_DIR / "generated" / "assets" / "00824_step29b1r_wall_generation_code_audit_v0_1.json"
B3R_HYPOTHESES_PATH = STEP29B3R_DIR / "generated" / "assets" / "00824_gateway_hypotheses_with_roles_v0_1.json"

FINAL_WALLS_PATH = FINAL_RASTER_DIR / "final_walls_skeleton.png"
FINAL_FULL_MAP_PATH = FINAL_RASTER_DIR / "final_full_map.png"
FINAL_FREE_SPACE_PATH = FINAL_RASTER_DIR / "final_free_space.png"
FINAL_OUTSIDE_BOUNDARY_PATH = FINAL_RASTER_DIR / "final_outside_boundary.png"
FINAL_GRID_META_PATH = FINAL_RASTER_DIR / "final_grid_metadata.json"
DENSITY_HIST_PATH = DEBUG_ROOM_DIR / "run_2252_01_density_hist.png"
ROOM_SEGMENTATION_DIAGNOSTICS_PATH = LOGS_DIR / "room_segmentation_diagnostics.json"

SCRIPT_PATH = BASE_DIR / "tools" / "step29a3r_export_gateway_wall_evidence.py"
THRESHOLDS = [0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.25]

POSITIVE_IDS = [
    "hyp_00824_r1_r3_01",
    "hyp_00824_r3_r7_01",
    "hyp_00824_r7_r11_01",
    "hyp_00824_r8_r11_01",
]
NEGATIVE_IDS = [
    "hyp_00824_r3_r11_01",
    "hyp_00824_r3_r11_02",
    "hyp_00824_r3_r11_03",
]

CASE_DIRS = {
    "hyp_00824_r1_r3_01": "r1_r3_positive",
    "hyp_00824_r3_r7_01": "r3_r7_positive",
    "hyp_00824_r7_r11_01": "r7_r11_positive",
    "hyp_00824_r8_r11_01": "r8_r11_positive",
    "hyp_00824_r3_r11_01": "r3_r11_negative",
    "hyp_00824_r3_r11_02": "r3_r11_negative",
    "hyp_00824_r3_r11_03": "r3_r11_negative",
}

VALIDATION_CHECKS = [
    "output_directory_exists",
    "wall_layer_candidates_npz_exists",
    "candidate_metadata_json_exists",
    "candidate_scores_json_exists",
    "height_band_sweep_json_exists",
    "threshold_sweep_json_exists",
    "doorway_benchmark_json_exists",
    "recommendation_json_exists",
    "summary_json_exists",
    "readme_exists",
    "no_step29c_topology_generated",
    "no_ros_nav2_gazebo_run",
    "source_artifacts_not_modified",
    "map_xy_visualizations_used",
    "segmentation_wall_processed_current_exported",
    "density_hist_saved_exported",
    "binary_preclose_threshold_sweep_exported",
    "wall_gap_preserving_reference_loaded_or_explained",
    "r1_r3_positive_in_benchmark",
    "r3_r7_positive_in_benchmark",
    "r7_r11_positive_in_benchmark",
    "r8_r11_positive_in_benchmark",
    "r3_r11_negative_in_benchmark",
    "recommendation_records_whether_layer_is_automatic_or_calibrated",
    "recommendation_records_whether_stage_a_rerun_needed",
]


def ensure_dirs():
    for path in [
        ASSETS_DIR,
        VIS_DIR / "layer_inventory",
        VIS_DIR / "height_band_sweep",
        VIS_DIR / "threshold_sweep",
        VIS_DIR / "doorway_benchmark" / "r1_r3_positive",
        VIS_DIR / "doorway_benchmark" / "r3_r7_positive",
        VIS_DIR / "doorway_benchmark" / "r7_r11_positive",
        VIS_DIR / "doorway_benchmark" / "r8_r11_positive",
        VIS_DIR / "doorway_benchmark" / "r3_r11_negative",
        VIS_DIR / "recommended_layer",
    ]:
        path.mkdir(parents=True, exist_ok=True)


def now_iso():
    return datetime.now().isoformat(timespec="seconds")


def rel(path):
    try:
        return str(pathlib.Path(path).relative_to(BASE_DIR))
    except ValueError:
        return str(path)


def read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=False)
        f.write("\n")


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def array_sha256(arr):
    arr = np.ascontiguousarray(arr)
    return hashlib.sha256(arr.tobytes()).hexdigest()


def load_gray(path):
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load grayscale image: {path}")
    return img


def align_to_shape(arr, target_shape):
    """Center-crop or zero-pad an array to target_shape."""
    out = arr
    h, w = out.shape[:2]
    th, tw = target_shape

    if h > th:
        top = (h - th) // 2
        out = out[top:top + th, :]
    if w > tw:
        left = (w - tw) // 2
        out = out[:, left:left + tw]

    h, w = out.shape[:2]
    if h < th or w < tw:
        padded = np.zeros((th, tw), dtype=out.dtype)
        top = (th - h) // 2
        left = (tw - w) // 2
        padded[top:top + h, left:left + w] = out
        out = padded
    return out


def layer_name_for_threshold(thr):
    return f"binary_preclose_thr_{thr:.2f}".replace(".", "p")


def map_extent(origin, resolution, grid_shape):
    x_min = origin[0]
    x_max = origin[0] + grid_shape[1] * resolution
    y_min = origin[1]
    y_max = origin[1] + grid_shape[0] * resolution
    return [x_min, x_max, y_min, y_max]


def local_extent(r0, r1, c0, c1, origin, resolution):
    return [
        origin[0] + c0 * resolution,
        origin[0] + c1 * resolution,
        origin[1] + r0 * resolution,
        origin[1] + r1 * resolution,
    ]


def draw_xy_arrows(ax, ext, color="white"):
    x_span = ext[1] - ext[0]
    y_span = ext[3] - ext[2]
    ax_x = ext[0] + 0.06 * x_span
    ax_y = ext[2] + 0.08 * y_span
    ax.annotate("", xy=(ax_x + 0.12 * x_span, ax_y), xytext=(ax_x, ax_y),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.4))
    ax.text(ax_x + 0.13 * x_span, ax_y, "+x", color=color, fontsize=8, va="center")
    ax.annotate("", xy=(ax_x, ax_y + 0.12 * y_span), xytext=(ax_x, ax_y),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.4))
    ax.text(ax_x, ax_y + 0.13 * y_span, "+y", color=color, fontsize=8, ha="center")


def rc_to_xy(row, col, origin, resolution):
    return origin[0] + col * resolution, origin[1] + row * resolution


def xy_to_rc(x, y, origin, resolution):
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def build_corridor_mask(center_rc, yaw, length_m, width_m, resolution, grid_shape):
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


def compute_scanline_gap(center_rc, yaw, wall, resolution, grid_shape,
                         n_scanlines=7, scan_half_len_m=0.5):
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    perp_cos = -sin_y
    perp_sin = cos_y
    half_len_cells = scan_half_len_m / resolution
    gap_widths = []
    center_clear_counts = 0
    wall_endpoint_counts = 0
    total_scanlines = 0

    for offset_along in np.linspace(-0.15 / resolution, 0.15 / resolution, n_scanlines):
        base_r = cr + offset_along * sin_y
        base_c = cc + offset_along * cos_y
        n_samples = int(2 * half_len_cells) + 1
        wall_positions = []
        for t in np.linspace(-half_len_cells, half_len_cells, n_samples):
            r = int(round(base_r + t * perp_sin))
            c = int(round(base_c + t * perp_cos))
            if 0 <= r < grid_shape[0] and 0 <= c < grid_shape[1] and wall[r, c]:
                wall_positions.append(t)
        total_scanlines += 1
        if len(wall_positions) >= 2:
            neg_walls = [p for p in wall_positions if p < -1]
            pos_walls = [p for p in wall_positions if p > 1]
            if neg_walls and pos_walls:
                inner_neg = max(neg_walls)
                inner_pos = min(pos_walls)
                gap_widths.append((inner_pos - inner_neg) * resolution)
                wall_endpoint_counts += 1
            if len([p for p in wall_positions if abs(p) <= 1.5]) == 0:
                center_clear_counts += 1
        elif len(wall_positions) == 0:
            center_clear_counts += 1
        elif len([p for p in wall_positions if abs(p) <= 1.5]) == 0:
            center_clear_counts += 1

    raw_wall_gap_width_m = float(np.median(gap_widths)) if gap_widths else 0.0
    wall_endpoint_support_score = wall_endpoint_counts / max(total_scanlines, 1)
    gap_center_clear_score = center_clear_counts / max(total_scanlines, 1)
    if len(gap_widths) >= 2:
        gap_consistency_score = 1.0 - (float(np.std(gap_widths)) / (float(np.mean(gap_widths)) + 1e-6))
    elif gap_widths:
        gap_consistency_score = 0.5
    else:
        gap_consistency_score = 0.0
    gap_consistency_score = max(0.0, min(1.0, gap_consistency_score))
    doorway_gap_score = (
        0.30 * wall_endpoint_support_score
        + 0.35 * gap_center_clear_score
        + 0.20 * gap_consistency_score
        + 0.15 * min(1.0, raw_wall_gap_width_m / 0.5)
    )
    return {
        "doorway_gap_score": round(float(doorway_gap_score), 4),
        "raw_wall_gap_width_m": round(float(raw_wall_gap_width_m), 4),
        "wall_endpoint_support_score": round(float(wall_endpoint_support_score), 4),
        "gap_center_clear_score": round(float(gap_center_clear_score), 4),
        "gap_consistency_score": round(float(gap_consistency_score), 4),
    }


def wall_filter_status(ratio):
    if ratio < 0.08:
        return "clear"
    if ratio < 0.18:
        return "uncertain"
    return "blocked"


def hypothesis_width_m(hyp):
    bbox = hyp.get("merged_bbox_map_xy") or []
    if len(bbox) == 2:
        (x0, y0), (x1, y1) = bbox
        return max(0.25, min(max(abs(x1 - x0), abs(y1 - y0)), 0.6))
    width = ((hyp.get("width_passability") or {}).get("merged_span_width_m")
             or (hyp.get("width_passability") or {}).get("max_source_width_m")
             or 0.35)
    return max(0.25, min(float(width), 0.6))


def compute_hypothesis_metrics(layer, hyp, label, resolution, grid_shape):
    wall = layer > 0
    center_rc = hyp["representative_center_rc"]
    cr, cc = int(center_rc[0]), int(center_rc[1])
    yaw = float((hyp.get("representative_crossing_pose") or {}).get("yaw", 0.0))
    corridor_width_m = hypothesis_width_m(hyp)
    corridor_length_m = 0.6
    corridor_mask = build_corridor_mask((cr, cc), yaw, corridor_length_m, corridor_width_m, resolution, grid_shape)
    corridor_count = int(corridor_mask.sum())
    wall_overlap_mask = wall & corridor_mask
    wall_count = int(wall_overlap_mask.sum())
    ratio = float(wall_count / corridor_count) if corridor_count else 0.0
    status = wall_filter_status(ratio)
    gap = compute_scanline_gap((cr, cc), yaw, wall, resolution, grid_shape)
    expected_positive = label == "positive"
    conflict = (expected_positive and status == "blocked") or ((not expected_positive) and status != "blocked")
    positive_passed = bool(expected_positive and status in {"clear", "uncertain"})
    negative_passed = bool((not expected_positive) and status == "blocked")
    return {
        "hypothesis_id": hyp["hypothesis_id"],
        "label": label,
        "room_a": hyp.get("room_a"),
        "room_b": hyp.get("room_b"),
        "wall_core_overlap_ratio": round(ratio, 4),
        "wall_core_overlap_count": wall_count,
        "corridor_cell_count": corridor_count,
        "wall_filter_status": status,
        "doorway_gap_score": gap["doorway_gap_score"],
        "raw_wall_gap_width_m": gap["raw_wall_gap_width_m"],
        "wall_endpoint_support_score": gap["wall_endpoint_support_score"],
        "gap_center_clear_score": gap["gap_center_clear_score"],
        "gap_consistency_score": gap["gap_consistency_score"],
        "conflict_with_user_label": bool(conflict),
        "positive_passed": positive_passed,
        "negative_passed": negative_passed,
        "representative_center_rc": [cr, cc],
        "representative_center_xy": hyp.get("representative_center_xy"),
        "representative_yaw": yaw,
        "corridor_width_m": round(float(corridor_width_m), 4),
        "corridor_length_m": corridor_length_m,
    }


def connected_component_summary(binary):
    arr = (binary > 0).astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(arr, connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA] if num_labels > 1 else np.array([], dtype=np.int32)
    return {
        "component_count": int(max(0, num_labels - 1)),
        "largest_component_area": int(areas.max()) if areas.size else 0,
        "median_component_area": float(np.median(areas)) if areas.size else 0.0,
    }


def aggregate_candidate_score(layer_metrics):
    positives = [m for m in layer_metrics if m["label"] == "positive"]
    negatives = [m for m in layer_metrics if m["label"] == "negative"]
    positive_clear_count = sum(1 for m in positives if m["wall_filter_status"] == "clear")
    positive_uncertain_count = sum(1 for m in positives if m["wall_filter_status"] == "uncertain")
    positive_blocked_count = sum(1 for m in positives if m["wall_filter_status"] == "blocked")
    negative_blocked_count = sum(1 for m in negatives if m["wall_filter_status"] == "blocked")
    negative_clear_count = sum(1 for m in negatives if m["wall_filter_status"] == "clear")
    negative_uncertain_count = sum(1 for m in negatives if m["wall_filter_status"] == "uncertain")
    avg_pos = float(np.mean([m["wall_core_overlap_ratio"] for m in positives])) if positives else 0.0
    avg_neg = float(np.mean([m["wall_core_overlap_ratio"] for m in negatives])) if negatives else 0.0
    score = (
        100 * positive_clear_count
        + 40 * positive_uncertain_count
        - 200 * positive_blocked_count
        + 80 * negative_blocked_count
        - 300 * negative_clear_count
        - 120 * negative_uncertain_count
        - 50 * avg_pos
        + 20 * avg_neg
    )
    return {
        "positive_clear_count": int(positive_clear_count),
        "positive_uncertain_count": int(positive_uncertain_count),
        "positive_blocked_count": int(positive_blocked_count),
        "negative_blocked_count": int(negative_blocked_count),
        "negative_uncertain_count": int(negative_uncertain_count),
        "negative_clear_count": int(negative_clear_count),
        "average_positive_overlap": round(avg_pos, 4),
        "average_negative_overlap": round(avg_neg, 4),
        "score": round(float(score), 4),
    }


def make_metadata(layer_name, arr, category, automatic, calibrated, source_path,
                  generation_method, threshold_factor=None, blurred=False,
                  pre_close=False, post_close=False, close=False,
                  dilation=False, erosion=False, height_band=None,
                  limitations=None):
    return {
        "layer_name": layer_name,
        "category": category,
        "automatic": bool(automatic),
        "calibrated_from_user_labels": bool(calibrated),
        "source_path": None if source_path is None else rel(source_path),
        "generation_method": generation_method,
        "height_band": height_band,
        "threshold_factor": threshold_factor,
        "blurred": bool(blurred),
        "pre_close": bool(pre_close),
        "post_close": bool(post_close),
        "morphological_close_applied": bool(close),
        "dilation_applied": bool(dilation),
        "erosion_applied": bool(erosion),
        "limitations": limitations or [],
        "nonzero_count": int(np.count_nonzero(arr)),
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
        "sha256": array_sha256(arr),
    }


def add_layer(layers, metadata, name, arr, **meta_kwargs):
    layers[name] = np.asarray(arr)
    metadata.append(make_metadata(name, layers[name], **meta_kwargs))


def layer_display(arr):
    if arr.dtype.kind == "f":
        return arr
    if arr.max() > 1:
        return arr.astype(float) / max(float(arr.max()), 1.0)
    return arr


def plot_layer(path, arr, title, origin, resolution, cmap="gray"):
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ext = map_extent(origin, resolution, arr.shape)
    ax.imshow(layer_display(arr), origin="lower", extent=ext, cmap=cmap)
    ax.set_title(f"{title}\ncoordinate_view=map_xy")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    draw_xy_arrows(ax, ext, color="white")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def make_overlay_rgb(room_mask, free_space, wall, corridor=None, overlap=None):
    rgb = np.zeros((*wall.shape, 3), dtype=float)
    rgb[free_space > 0] = [0.12, 0.12, 0.12]
    colors = {
        1: [0.45, 0.20, 0.65],
        3: [0.10, 0.55, 0.65],
        7: [0.16, 0.28, 0.78],
        8: [0.16, 0.58, 0.26],
        11: [0.75, 0.42, 0.10],
        14: [0.68, 0.55, 0.18],
        15: [0.45, 0.45, 0.18],
        16: [0.22, 0.45, 0.75],
    }
    for rid, color in colors.items():
        rgb[room_mask == rid] = color
    rgb[wall > 0] = [0.95, 0.10, 0.08]
    if corridor is not None:
        rgb[corridor > 0] = 0.55 * rgb[corridor > 0] + 0.45 * np.array([0.0, 0.85, 1.0])
    if overlap is not None:
        rgb[overlap > 0] = [1.0, 1.0, 0.0]
    return np.clip(rgb, 0, 1)


def crop_bounds_for_hypotheses(hyps, grid_shape, pad=45):
    rows = []
    cols = []
    for hyp in hyps:
        r0, c0, r1, c1 = hyp.get("merged_bbox_cells", [0, 0, 0, 0])
        cr, cc = hyp["representative_center_rc"]
        rows.extend([r0, r1, cr])
        cols.extend([c0, c1, cc])
    r0 = max(0, int(min(rows)) - pad)
    r1 = min(grid_shape[0], int(max(rows)) + pad + 1)
    c0 = max(0, int(min(cols)) - pad)
    c1 = min(grid_shape[1], int(max(cols)) + pad + 1)
    return r0, r1, c0, c1


def plot_doorway_case(path, case_title, hyps, layer_names, layers, metrics_by_layer,
                      room_mask, free_space, origin, resolution):
    n = len(layer_names)
    fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 5.4), squeeze=False)
    axes = axes[0]
    grid_shape = room_mask.shape
    r0, r1, c0, c1 = crop_bounds_for_hypotheses(hyps, grid_shape)
    ext = local_extent(r0, r1, c0, c1, origin, resolution)

    for ax, lname in zip(axes, layer_names):
        wall = layers[lname] > 0
        all_corridor = np.zeros(grid_shape, dtype=bool)
        all_overlap = np.zeros(grid_shape, dtype=bool)
        lines = []
        for hyp in hyps:
            mid = hyp["hypothesis_id"]
            m = metrics_by_layer[lname][mid]
            corridor = build_corridor_mask(
                tuple(m["representative_center_rc"]),
                m["representative_yaw"],
                m["corridor_length_m"],
                m["corridor_width_m"],
                resolution,
                grid_shape,
            )
            all_corridor |= corridor
            all_overlap |= (corridor & wall)
            cr, cc = m["representative_center_rc"]
            x, y = rc_to_xy(cr, cc, origin, resolution)
            ax.plot(x, y, marker="*", color="white", markeredgecolor="black", markersize=9)
            short_id = mid.replace("hyp_00824_", "")
            lines.append(f"{short_id}: {m['wall_filter_status']} ov={m['wall_core_overlap_ratio']:.2f}")
        rgb = make_overlay_rgb(room_mask, free_space, wall, all_corridor, all_overlap)
        ax.imshow(rgb[r0:r1, c0:c1], origin="lower", extent=ext)
        ax.set_title(f"{lname}\n" + "\n".join(lines[:3]), fontsize=8)
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        draw_xy_arrows(ax, ext, color="white")
    fig.suptitle(f"{case_title} layer comparison, coordinate_view=map_xy", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def load_current_height_slice():
    if not ROOM_SEGMENTATION_DIAGNOSTICS_PATH.exists():
        return {}
    diagnostics = read_json(ROOM_SEGMENTATION_DIAGNOSTICS_PATH)
    runs = []
    for floor in diagnostics.get("per_floor", []):
        runs.extend(floor.get("runs", []))
    successful = [r for r in runs if r.get("success")]
    if not successful:
        return {}
    last = max(successful, key=lambda r: int(r.get("frame_idx", -1)))
    height_slice = dict(last.get("height_slice") or {})
    height_slice.update({
        "run_id": last.get("run_id"),
        "frame_idx": last.get("frame_idx"),
        "wall_slice_point_count": last.get("wall_slice_point_count"),
        "grid_width": last.get("grid_width"),
        "grid_height": last.get("grid_height"),
        "height_convention": "absolute map/world z; slice bounds are estimated from floor_z/ceiling_z percentiles",
    })
    return height_slice


def build_candidates(bev, target_shape):
    layers = {}
    metadata = []
    warnings = []

    structural_wall = (bev["structural_wall"] > 0).astype(np.uint8)
    free_space = (bev["free_space"] > 0).astype(np.uint8)
    unknown_layer = (bev["unknown_layer"] > 0).astype(np.uint8)
    room_mask_global = bev["room_mask_global_id"].astype(np.int32)

    add_layer(
        layers, metadata, "segmentation_wall_processed_current", structural_wall,
        category="segmentation_wall_processed",
        automatic=True,
        calibrated=False,
        source_path=FINAL_WALLS_PATH,
        generation_method=(
            "Step29B1 structural_wall aligned from Step29A3 final_walls_skeleton.png; "
            "source was blurred, thresholded, padded, and morphologically closed."
        ),
        blurred=True,
        pre_close=False,
        post_close=True,
        close=True,
        limitations=[
            "Processed segmentation support layer, not a raw gateway blocker.",
            "May close real door gaps.",
        ],
    )

    density_u8 = align_to_shape(load_gray(DENSITY_HIST_PATH), target_shape)
    density_norm = density_u8.astype(np.float32) / max(float(density_u8.max()), 1.0)
    add_layer(
        layers, metadata, "density_hist_saved", density_norm,
        category="gateway_wall_candidate",
        automatic=True,
        calibrated=False,
        source_path=DENSITY_HIST_PATH,
        generation_method=(
            "Saved Stage-A run_2252_01_density_hist.png, aligned to Step29B1 grid. "
            "This is clipped/normalized and Gaussian-blurred; exact raw pre-blur histogram is unavailable."
        ),
        blurred=True,
        pre_close=True,
        limitations=[
            "Saved density image is already clipped, normalized, and blurred.",
            "Not exact raw pre-blur wall evidence.",
        ],
    )

    max_hist = float(density_u8.max())
    for thr in THRESHOLDS:
        arr = (density_u8 > (thr * max_hist)).astype(np.uint8)
        lname = layer_name_for_threshold(thr)
        add_layer(
            layers, metadata, lname, arr,
            category="gateway_wall_candidate",
            automatic=True,
            calibrated=False,
            source_path=DENSITY_HIST_PATH,
            generation_method=f"Threshold saved blurred density histogram at {thr:.2f} * max(hist), without morphological close.",
            threshold_factor=thr,
            blurred=True,
            pre_close=True,
            limitations=[
                "Approximates pre-close binary layer from saved blurred density, not raw pre-blur data.",
            ],
        )

    layers["binary_preclose_saved_threshold_0p15"] = layers["binary_preclose_thr_0p15"].copy()
    metadata.append(make_metadata(
        "binary_preclose_saved_threshold_0p15",
        layers["binary_preclose_saved_threshold_0p15"],
        category="gateway_wall_candidate",
        automatic=True,
        calibrated=False,
        source_path=DENSITY_HIST_PATH,
        generation_method="Alias required by Step29A3R: saved blurred density thresholded at 0.15 * max(hist), no close.",
        threshold_factor=0.15,
        blurred=True,
        pre_close=True,
        limitations=["Duplicate alias of binary_preclose_thr_0p15 for explicit provenance."],
    ))

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    thinned = cv2.erode(structural_wall.astype(np.uint8), kernel, iterations=1)
    add_layer(
        layers, metadata, "wall_thinned_from_processed", thinned,
        category="diagnostic_reference",
        automatic=True,
        calibrated=False,
        source_path=FINAL_WALLS_PATH,
        generation_method="One-cell cross erosion of processed segmentation wall; diagnostic only.",
        blurred=True,
        pre_close=False,
        post_close=True,
        close=True,
        erosion=True,
        limitations=[
            "Derived from already-closed segmentation wall.",
            "Can reduce overblocking but does not recover true raw wall evidence.",
        ],
    )

    b1r_loaded = False
    if B1R_UNINFLATED_NPZ_PATH.exists():
        with np.load(B1R_UNINFLATED_NPZ_PATH, allow_pickle=True) as b1r_npz:
            if "wall_gap_preserving_candidate" in b1r_npz.files:
                calibrated = align_to_shape((b1r_npz["wall_gap_preserving_candidate"] > 0).astype(np.uint8), target_shape)
                add_layer(
                    layers, metadata, "wall_gap_preserving_calibrated_reference", calibrated,
                    category="calibrated_reference",
                    automatic=False,
                    calibrated=True,
                    source_path=B1R_UNINFLATED_NPZ_PATH,
                    generation_method=(
                        "Loaded Step29B1R wall_gap_preserving_candidate. This candidate used user-confirmed "
                        "corridor carving and is a calibrated reference, not a fully automatic raw wall layer."
                    ),
                    blurred=True,
                    pre_close=False,
                    post_close=True,
                    close=True,
                    limitations=[
                        "Calibrated from user doorway labels.",
                        "Useful review/reference layer but not an automatic solution.",
                    ],
                )
                b1r_loaded = True
    if not b1r_loaded:
        warnings.append("Step29B1R wall_gap_preserving_candidate was not available; calibrated reference layer not exported.")

    add_layer(
        layers, metadata, "free_space_reference", free_space,
        category="diagnostic_reference",
        automatic=True,
        calibrated=False,
        source_path=BEV_NPZ_PATH,
        generation_method="Step29B1 free_space layer reference.",
        limitations=["Reference layer only; not a wall candidate."],
    )
    add_layer(
        layers, metadata, "room_mask_global_id_reference", room_mask_global,
        category="diagnostic_reference",
        automatic=True,
        calibrated=False,
        source_path=BEV_NPZ_PATH,
        generation_method="Step29B1 room_mask_global_id reference.",
        limitations=["Integer room ID reference; not a wall candidate."],
    )
    add_layer(
        layers, metadata, "unknown_layer_reference", unknown_layer,
        category="diagnostic_reference",
        automatic=True,
        calibrated=False,
        source_path=BEV_NPZ_PATH,
        generation_method="Step29B1 unknown_layer reference.",
        limitations=["Reference layer only; unknown is not structural wall."],
    )

    raw_explanation = {
        "raw_midheight_wall_hist_if_reconstructable": {
            "available": False,
            "reason": (
                "No raw wall point set, raw histogram, or pre-blur histogram was found under the Step29A3 "
                "artifact tree. The only saved wall-density raster is run_2252_01_density_hist.png, "
                "which is already clipped/normalized and Gaussian-blurred."
            ),
        },
        "pre_blur_wall_density_if_reconstructable": {
            "available": False,
            "reason": (
                "DynamicRoomSegmenter._save_debug stores the histogram after clipping, normalization, "
                "and GaussianBlur. It does not store pre-blur histogram values."
            ),
        },
        "stage_a_new_raw_debug_export": {
            "performed": False,
            "reason": (
                "A raw debug rerun would require the Stage-A point accumulation/runtime context. "
                "This Step29A3R run is offline and does not reload vision models or rerun Stage-A."
            ),
        },
    }
    warnings.extend([
        raw_explanation["raw_midheight_wall_hist_if_reconstructable"]["reason"],
        raw_explanation["pre_blur_wall_density_if_reconstructable"]["reason"],
    ])
    return layers, metadata, warnings, raw_explanation


def benchmark_layers(layers, metadata, hypotheses, resolution, grid_shape, room_mask, free_space):
    hyp_by_id = {h["hypothesis_id"]: h for h in hypotheses}
    benchmark_ids = POSITIVE_IDS + NEGATIVE_IDS
    missing = [hid for hid in benchmark_ids if hid not in hyp_by_id]
    if missing:
        raise RuntimeError(f"Missing benchmark hypotheses: {missing}")

    wall_layer_names = [
        m["layer_name"] for m in metadata
        if m["category"] in {"segmentation_wall_processed", "gateway_wall_candidate", "calibrated_reference", "diagnostic_reference"}
        and m["layer_name"] not in {"free_space_reference", "room_mask_global_id_reference", "unknown_layer_reference", "density_hist_saved"}
    ]

    per_layer = {}
    score_rows = []
    threshold_rows = []
    for lname in wall_layer_names:
        arr = layers[lname]
        metrics = []
        metric_map = {}
        for hid in benchmark_ids:
            label = "positive" if hid in POSITIVE_IDS else "negative"
            m = compute_hypothesis_metrics(arr, hyp_by_id[hid], label, resolution, grid_shape)
            metrics.append(m)
            metric_map[hid] = m
        agg = aggregate_candidate_score(metrics)
        meta = next(x for x in metadata if x["layer_name"] == lname)
        comp = connected_component_summary(arr > 0)
        overlap = {
            "nonzero_count": int(np.count_nonzero(arr)),
            "free_space_overlap_count": int(np.logical_and(arr > 0, free_space > 0).sum()),
            "room_mask_overlap_count": int(np.logical_and(arr > 0, room_mask > 0).sum()),
            "unknown_overlap_count": int(np.logical_and(arr > 0, layers["unknown_layer_reference"] > 0).sum()),
        }
        row = {
            "layer_name": lname,
            "category": meta["category"],
            "automatic": meta["automatic"],
            "calibrated_from_user_labels": meta["calibrated_from_user_labels"],
            **agg,
            **comp,
            **overlap,
        }
        score_rows.append(row)
        per_layer[lname] = {
            "aggregate": row,
            "hypotheses": metric_map,
        }
        if lname.startswith("binary_preclose_thr_"):
            threshold_rows.append({
                "layer_name": lname,
                "threshold_factor": meta["threshold_factor"],
                "source": "density_hist_saved",
                **row,
            })

    score_rows.sort(key=lambda r: r["score"], reverse=True)
    threshold_rows.sort(key=lambda r: r["threshold_factor"])

    benchmark = {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "positive_hypotheses": POSITIVE_IDS,
        "negative_hypotheses": NEGATIVE_IDS,
        "wall_filter_thresholds": {
            "clear": "ratio < 0.08",
            "uncertain": "0.08 <= ratio < 0.18",
            "blocked": "ratio >= 0.18",
        },
        "score_formula": (
            "+100*positive_clear_count +40*positive_uncertain_count -200*positive_blocked_count "
            "+80*negative_blocked_count -300*negative_clear_count -120*negative_uncertain_count "
            "-50*average_positive_overlap +20*average_negative_overlap"
        ),
        "layers": per_layer,
        "candidate_scores_ranked": score_rows,
    }
    return benchmark, score_rows, threshold_rows


def build_height_band_sweep(current_height_slice):
    bands = [
        {"band_name": "band_A_default_current", "z_min": current_height_slice.get("slice_z_min"), "z_max": current_height_slice.get("slice_z_max")},
        {"band_name": "band_B_lower_upper_1p20", "floor_relative_z_min": 0.35, "floor_relative_z_max": 1.20},
        {"band_name": "band_C_lower_upper_1p00", "floor_relative_z_min": 0.35, "floor_relative_z_max": 1.00},
        {"band_name": "band_D_lower_upper_0p90", "floor_relative_z_min": 0.35, "floor_relative_z_max": 0.90},
        {"band_name": "band_E_mid_0p35_0p90", "floor_relative_z_min": 0.35, "floor_relative_z_max": 0.90},
        {"band_name": "band_F_mid_0p45_1p10", "floor_relative_z_min": 0.45, "floor_relative_z_max": 1.10},
        {"band_name": "band_G_mid_0p50_1p00", "floor_relative_z_min": 0.50, "floor_relative_z_max": 1.00},
    ]
    floor_z = current_height_slice.get("floor_z")
    if floor_z is not None:
        for band in bands:
            if "floor_relative_z_min" in band:
                band["absolute_z_min_if_same_floor"] = round(float(floor_z) + float(band["floor_relative_z_min"]), 4)
                band["absolute_z_max_if_same_floor"] = round(float(floor_z) + float(band["floor_relative_z_max"]), 4)
    return {
        "height_band_sweep_performed": False,
        "reason_if_not_performed": (
            "No raw 3D wall point set or pre-blur histogram is available in the Step29A3 frozen artifacts. "
            "Generating height-band candidates would require a Stage-A debug rerun or a saved point-cloud export; "
            "this offline Step29A3R run intentionally did not reload models or rerun Stage-A."
        ),
        "current_height_slice_parameters": current_height_slice,
        "bands": bands,
        "layers_generated": [],
        "recommended_height_band": None,
        "warnings": [
            "Height-band sweep could not be performed offline.",
            "Add a Stage-A debug export for raw wall points or per-band raw histograms before the next full Stage-A rerun.",
        ],
        "recommended_debug_hook": {
            "target": "boxfusion/dynamic_room_segmenter.py",
            "location": "after pts_walls and histogram2d are computed in the segmentation wall raster path",
            "export": [
                "raw wall point histogram before clipping",
                "clipped histogram before normalization",
                "normalized pre-blur histogram",
                "blurred histogram",
                "binary pre-close threshold sweep",
                "post-close segmentation wall for provenance only",
            ],
            "note": "Keep segmentation_wall_processed and gateway_wall_raw_or_preclose separate in downstream metadata.",
        },
    }


def build_threshold_sweep(threshold_rows, benchmark):
    rows = []
    for row in threshold_rows:
        lname = row["layer_name"]
        rows.append({
            **row,
            "benchmark_hypotheses": benchmark["layers"][lname]["hypotheses"],
        })
    return {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "source_histogram_layer": "density_hist_saved",
        "threshold_factors": THRESHOLDS,
        "threshold_candidates": rows,
        "best_threshold_layer": max(rows, key=lambda r: r["score"])["layer_name"] if rows else None,
        "warnings": [
            "Threshold sweep is from saved clipped/normalized/blurred density_hist, not raw pre-blur histogram."
        ],
    }


def choose_recommendation(score_rows):
    automatic_rows = [
        r for r in score_rows
        if r["automatic"] and r["category"] == "gateway_wall_candidate"
    ]
    calibrated_rows = [
        r for r in score_rows
        if r["calibrated_from_user_labels"] or r["category"] == "calibrated_reference"
    ]
    best_auto = max(automatic_rows, key=lambda r: r["score"]) if automatic_rows else None
    best_cal = max(calibrated_rows, key=lambda r: r["score"]) if calibrated_rows else None

    if best_auto and best_auto["positive_blocked_count"] == 0 and best_auto["negative_clear_count"] == 0 and best_auto["negative_uncertain_count"] == 0:
        recommended = best_auto
        rerun_needed = False
        rationale = "Best automatic saved-density pre-close layer passes known positives and keeps r3_r11 negatives blocked."
    elif best_cal:
        recommended = best_cal
        rerun_needed = True
        rationale = (
            "Best benchmark score is from the calibrated Step29B1R gap-preserving reference. "
            "Use it for review, but do not treat it as the final automatic gateway wall layer."
        )
    elif best_auto:
        recommended = best_auto
        rerun_needed = True
        rationale = (
            "No candidate fully satisfies the doorway benchmark. The best automatic layer is useful as an interim "
            "saved-density pre-close candidate, but raw/pre-blur height-band export is recommended."
        )
    else:
        recommended = None
        rerun_needed = True
        rationale = "No gateway wall candidate was available."

    return recommended, best_auto, best_cal, rerun_needed, rationale


def create_visualizations(layers, metadata, benchmark, score_rows, threshold_rows,
                          hypotheses, room_mask, free_space, origin, resolution,
                          recommended, best_auto, best_cal):
    layer_inventory_names = [
        "segmentation_wall_processed_current",
        "density_hist_saved",
        "binary_preclose_thr_0p05",
        "binary_preclose_thr_0p08",
        "binary_preclose_thr_0p10",
        "binary_preclose_thr_0p12",
        "binary_preclose_thr_0p15",
        "binary_preclose_thr_0p18",
        "binary_preclose_thr_0p20",
        "binary_preclose_thr_0p25",
        "wall_thinned_from_processed",
        "wall_gap_preserving_calibrated_reference",
    ]
    for lname in layer_inventory_names:
        if lname in layers:
            plot_layer(
                VIS_DIR / "layer_inventory" / f"{lname}.png",
                layers[lname],
                lname,
                origin,
                resolution,
                cmap="magma" if lname == "density_hist_saved" else "gray",
            )

    # Doorway panels compare the important classes without making each file unreadably huge.
    hyp_by_id = {h["hypothesis_id"]: h for h in hypotheses}
    panel_layers = ["segmentation_wall_processed_current"]
    if best_auto:
        panel_layers.append(best_auto["layer_name"])
    if recommended and recommended["layer_name"] not in panel_layers:
        panel_layers.append(recommended["layer_name"])
    if best_cal and best_cal["layer_name"] not in panel_layers:
        panel_layers.append(best_cal["layer_name"])
    panel_layers = panel_layers[:4]

    for hid in POSITIVE_IDS:
        case = CASE_DIRS[hid]
        plot_doorway_case(
            VIS_DIR / "doorway_benchmark" / case / f"layer_comparison_{case}.png",
            case,
            [hyp_by_id[hid]],
            panel_layers,
            layers,
            {lname: benchmark["layers"][lname]["hypotheses"] for lname in panel_layers},
            room_mask,
            free_space,
            origin,
            resolution,
        )
    plot_doorway_case(
        VIS_DIR / "doorway_benchmark" / "r3_r11_negative" / "layer_comparison_r3_r11_negative.png",
        "r3_r11_negative",
        [hyp_by_id[hid] for hid in NEGATIVE_IDS],
        panel_layers,
        layers,
        {lname: benchmark["layers"][lname]["hypotheses"] for lname in panel_layers},
        room_mask,
        free_space,
        origin,
        resolution,
    )

    # Threshold sweep score plot.
    if threshold_rows:
        fig, ax = plt.subplots(1, 1, figsize=(9, 5))
        xs = [r["threshold_factor"] for r in threshold_rows]
        ys = [r["score"] for r in threshold_rows]
        ax.plot(xs, ys, marker="o", color="#1f77b4")
        ax.set_xlabel("threshold factor * max(saved density_hist)")
        ax.set_ylabel("benchmark score")
        ax.set_title("Step29A3R threshold sweep score plot, coordinate_view=map_xy")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(VIS_DIR / "threshold_sweep" / "threshold_sweep_score_plot.png", dpi=140)
        plt.close(fig)

        key_layers = [r["layer_name"] for r in threshold_rows]
        key_ids = POSITIVE_IDS + ["hyp_00824_r3_r11_01"]
        status_grid = np.zeros((len(key_ids), len(key_layers)), dtype=float)
        status_value = {"clear": 0.15, "uncertain": 0.55, "blocked": 0.95}
        for c, lname in enumerate(key_layers):
            for r, hid in enumerate(key_ids):
                status_grid[r, c] = status_value[benchmark["layers"][lname]["hypotheses"][hid]["wall_filter_status"]]
        fig, ax = plt.subplots(1, 1, figsize=(11, 4.5))
        im = ax.imshow(status_grid, origin="upper", cmap="RdYlGn_r", vmin=0, vmax=1, aspect="auto")
        ax.set_xticks(range(len(key_layers)))
        ax.set_xticklabels([str(r["threshold_factor"]) for r in threshold_rows], rotation=45, ha="right")
        ax.set_yticks(range(len(key_ids)))
        ax.set_yticklabels([hid.replace("hyp_00824_", "") for hid in key_ids])
        for r in range(len(key_ids)):
            for c in range(len(key_layers)):
                status = benchmark["layers"][key_layers[c]]["hypotheses"][key_ids[r]]["wall_filter_status"]
                ax.text(c, r, status[0].upper(), ha="center", va="center", fontsize=8)
        ax.set_title("Threshold sweep key doorway grid, coordinate_view=map_xy")
        fig.colorbar(im, ax=ax, label="clear -> blocked")
        fig.tight_layout()
        fig.savefig(VIS_DIR / "threshold_sweep" / "threshold_sweep_key_doorway_grid.png", dpi=140)
        plt.close(fig)

    # Height-band placeholder visualization.
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    ax.axis("off")
    ax.text(
        0.02, 0.75,
        "Height-band sweep not performed offline\ncoordinate_view=map_xy\n\n"
        "No raw 3D wall point set or pre-blur histogram was present in Step29A3 artifacts.\n"
        "Next full Stage-A rerun should export raw/pre-blur per-band histograms.",
        fontsize=11,
        va="top",
    )
    fig.tight_layout()
    fig.savefig(VIS_DIR / "height_band_sweep" / "height_band_score_plot.png", dpi=140)
    plt.close(fig)
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    ax.axis("off")
    ax.text(
        0.02, 0.75,
        "Height-band key doorway grid unavailable\ncoordinate_view=map_xy\n\n"
        "The script records planned bands and current z-slice in JSON.",
        fontsize=11,
        va="top",
    )
    fig.tight_layout()
    fig.savefig(VIS_DIR / "height_band_sweep" / "height_band_key_doorway_grid.png", dpi=140)
    plt.close(fig)

    # Recommended layer global overview.
    rec_name = recommended["layer_name"] if recommended else (best_auto["layer_name"] if best_auto else "segmentation_wall_processed_current")
    plot_layer(
        VIS_DIR / "recommended_layer" / "recommended_gateway_wall_layer_overview.png",
        layers[rec_name],
        f"recommended_gateway_wall_layer={rec_name}",
        origin,
        resolution,
    )

    key_hyps = [hyp_by_id[hid] for hid in POSITIVE_IDS + NEGATIVE_IDS]
    plot_doorway_case(
        VIS_DIR / "recommended_layer" / "recommended_gateway_wall_layer_key_doorways.png",
        "recommended layer key doorways",
        key_hyps,
        [rec_name],
        layers,
        {rec_name: benchmark["layers"][rec_name]["hypotheses"]},
        room_mask,
        free_space,
        origin,
        resolution,
    )

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    compare_layers = ["segmentation_wall_processed_current", rec_name]
    grid_shape = room_mask.shape
    ext = map_extent(origin, resolution, grid_shape)
    for ax, lname in zip(axes, compare_layers):
        rgb = make_overlay_rgb(room_mask, free_space, layers[lname] > 0)
        ax.imshow(rgb, origin="lower", extent=ext)
        ax.set_title(f"{lname}\ncoordinate_view=map_xy", fontsize=9)
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        draw_xy_arrows(ax, ext)
    fig.tight_layout()
    fig.savefig(VIS_DIR / "recommended_layer" / "recommended_vs_segmentation_wall_processed.png", dpi=140)
    plt.close(fig)


def write_readme(summary, recommendation, height_band_sweep, warnings):
    lines = [
        "# Step29A3R: Gateway Wall Evidence Export",
        "",
        f"Scene: `{SCENE_ID}`",
        "",
        "Step29A3R exports wall evidence layers for gateway filtering and benchmarks them against known true doorway positives and r3_r11 negative sanity hypotheses.",
        "",
        "## Why This Step Exists",
        "",
        "Step29B1R showed that Step29B1 `structural_wall` is exactly aligned to Step29A3 `final_walls_skeleton.png`, but that source is not raw or uninflated wall evidence. It is a Stage-A segmentation support raster produced from mid-height wall density by clipping/normalization, Gaussian blur, thresholding, 10 px padding, and one 3x3 cross morphological close. That is useful for watershed room segmentation, but it can close real doorways and should not be a hard gateway blocker.",
        "",
        "## Wall Concepts",
        "",
        "- `segmentation_wall_processed`: processed/closed wall support for watershed room segmentation and provenance.",
        "- `gateway_wall_raw_or_preclose`: raw, pre-blur, pre-close, or threshold-sweep wall evidence intended to preserve real door gaps.",
        "",
        "## Exported Layers",
        "",
        "The NPZ `generated/assets/00824_gateway_wall_layer_candidates_v0_1.npz` includes the current processed segmentation wall, the saved density histogram, pre-close threshold-sweep candidates, a thinned diagnostic wall, Step29B1R calibrated reference if available, and free/room/unknown references.",
        "",
        "The exact raw pre-blur wall histogram was not available in the frozen Step29A3 artifacts. The saved `run_2252_01_density_hist.png` is already clipped, normalized, and blurred.",
        "",
        "## Height-Band Sweep",
        "",
        f"Height-band sweep performed: `{height_band_sweep['height_band_sweep_performed']}`.",
        "",
        height_band_sweep["reason_if_not_performed"],
        "",
        "The current recorded Stage-A wall slice is absolute map/world z and is stored in `00824_gateway_wall_height_band_sweep_v0_1.json`.",
        "",
        "## Threshold Sweep",
        "",
        "Threshold factors `0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.25` were applied to the saved blurred density histogram without morphological close. Each candidate was benchmarked against known positives and negatives.",
        "",
        "## Doorway Benchmark",
        "",
        "Positives: `hyp_00824_r1_r3_01`, `hyp_00824_r3_r7_01`, `hyp_00824_r7_r11_01`, `hyp_00824_r8_r11_01`.",
        "",
        "Negatives: `hyp_00824_r3_r11_01`, `hyp_00824_r3_r11_02`, `hyp_00824_r3_r11_03`.",
        "",
        "A positive passes if it is `clear` or `uncertain`; a negative passes when it remains `blocked`.",
        "",
        "## Recommendation",
        "",
        f"Recommended gateway wall layer: `{recommendation['recommended_gateway_wall_layer']}`.",
        "",
        f"Fully automatic: `{recommendation['recommended_layer_is_automatic']}`.",
        "",
        f"Calibrated from user labels: `{recommendation['recommended_layer_calibrated_from_user_labels']}`.",
        "",
        f"Stage-A raw debug rerun recommended: `{recommendation['stage_a_raw_debug_rerun_recommended']}`.",
        "",
        recommendation["rationale"],
        "",
        "Later Step29B3R2 should consume the recommended gateway wall layer as a gateway-evidence input, while keeping `segmentation_wall_processed_current` only as segmentation/provenance wall. The `wall_core_blocked` result should be downgraded from hard reject to a conflict flag when doorway gap/support metrics and user labels disagree.",
        "",
        "No topology was generated, and no ROS/Nav2/Gazebo/AMCL/DWB/RViz process was run.",
        "",
        "## Key Outputs",
        "",
        "- `generated/assets/00824_gateway_wall_layer_candidates_v0_1.npz`",
        "- `generated/assets/00824_gateway_wall_layer_candidate_metadata_v0_1.json`",
        "- `generated/assets/00824_gateway_wall_layer_candidate_scores_v0_1.json`",
        "- `generated/assets/00824_gateway_wall_threshold_sweep_v0_1.json`",
        "- `generated/assets/00824_gateway_wall_height_band_sweep_v0_1.json`",
        "- `generated/assets/00824_gateway_wall_doorway_benchmark_v0_1.json`",
        "- `generated/assets/00824_gateway_wall_recommendation_v0_1.json`",
        "",
    ]
    if warnings:
        lines.extend(["## Warnings", ""])
        lines.extend([f"- {w}" for w in warnings])
        lines.append("")
    (OUTPUT_DIR / "README_step29a3r.md").write_text("\n".join(lines), encoding="utf-8")


def validate_outputs(layers, metadata, benchmark, recommendation, input_hashes_before, input_hashes_after):
    output_files = list(OUTPUT_DIR.rglob("*"))
    output_names = [p.name.lower() for p in output_files]
    no_step29c = not any("step29c" in str(p).lower() or "topology" in p.name.lower() for p in output_files)
    source_unchanged = input_hashes_before == input_hashes_after
    map_xy_pngs = list((VIS_DIR).rglob("*.png"))
    checks = {
        "output_directory_exists": OUTPUT_DIR.exists(),
        "wall_layer_candidates_npz_exists": (ASSETS_DIR / "00824_gateway_wall_layer_candidates_v0_1.npz").exists(),
        "candidate_metadata_json_exists": (ASSETS_DIR / "00824_gateway_wall_layer_candidate_metadata_v0_1.json").exists(),
        "candidate_scores_json_exists": (ASSETS_DIR / "00824_gateway_wall_layer_candidate_scores_v0_1.json").exists(),
        "height_band_sweep_json_exists": (ASSETS_DIR / "00824_gateway_wall_height_band_sweep_v0_1.json").exists(),
        "threshold_sweep_json_exists": (ASSETS_DIR / "00824_gateway_wall_threshold_sweep_v0_1.json").exists(),
        "doorway_benchmark_json_exists": (ASSETS_DIR / "00824_gateway_wall_doorway_benchmark_v0_1.json").exists(),
        "recommendation_json_exists": (ASSETS_DIR / "00824_gateway_wall_recommendation_v0_1.json").exists(),
        "summary_json_exists": (ASSETS_DIR / "00824_step29a3r_summary_v0_1.json").exists(),
        "readme_exists": (OUTPUT_DIR / "README_step29a3r.md").exists(),
        "no_step29c_topology_generated": no_step29c,
        "no_ros_nav2_gazebo_run": True,
        "source_artifacts_not_modified": source_unchanged,
        "map_xy_visualizations_used": len(map_xy_pngs) > 0,
        "segmentation_wall_processed_current_exported": "segmentation_wall_processed_current" in layers,
        "density_hist_saved_exported": "density_hist_saved" in layers,
        "binary_preclose_threshold_sweep_exported": all(layer_name_for_threshold(t) in layers for t in THRESHOLDS),
        "wall_gap_preserving_reference_loaded_or_explained": (
            "wall_gap_preserving_calibrated_reference" in layers
            or any("wall_gap_preserving" in w.lower() for w in recommendation.get("warnings", []))
        ),
        "r1_r3_positive_in_benchmark": "hyp_00824_r1_r3_01" in benchmark["positive_hypotheses"],
        "r3_r7_positive_in_benchmark": "hyp_00824_r3_r7_01" in benchmark["positive_hypotheses"],
        "r7_r11_positive_in_benchmark": "hyp_00824_r7_r11_01" in benchmark["positive_hypotheses"],
        "r8_r11_positive_in_benchmark": "hyp_00824_r8_r11_01" in benchmark["positive_hypotheses"],
        "r3_r11_negative_in_benchmark": all(h in benchmark["negative_hypotheses"] for h in NEGATIVE_IDS),
        "recommendation_records_whether_layer_is_automatic_or_calibrated": (
            "recommended_layer_is_automatic" in recommendation
            and "recommended_layer_calibrated_from_user_labels" in recommendation
        ),
        "recommendation_records_whether_stage_a_rerun_needed": "stage_a_raw_debug_rerun_recommended" in recommendation,
    }
    failed = [k for k, ok in checks.items() if not ok]
    return {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "checks": checks,
        "passed": len(failed) == 0,
        "failed_checks": failed,
        "source_artifact_hashes_before": input_hashes_before,
        "source_artifact_hashes_after": input_hashes_after,
        "notes": {
            "no_ros_nav2_gazebo_run": "Script contains no calls to ROS/Nav2/Gazebo/AMCL/DWB/RViz and was run as offline Python.",
            "no_step29c_topology_generated": "Output tree contains no Step29C/topology artifacts.",
        },
    }


def main():
    ensure_dirs()
    print("=" * 70)
    print("Step29A3R: Gateway wall evidence export and doorway benchmark")
    print("=" * 70)

    input_paths = [
        FINAL_WALLS_PATH,
        FINAL_FULL_MAP_PATH,
        FINAL_FREE_SPACE_PATH,
        FINAL_OUTSIDE_BOUNDARY_PATH,
        FINAL_GRID_META_PATH,
        DENSITY_HIST_PATH,
        BEV_NPZ_PATH,
        GLOBAL_ROOM_MASK_PATH,
        BEV_META_PATH,
        B1R_UNINFLATED_NPZ_PATH,
        B1R_CODE_AUDIT_PATH,
        B3R_HYPOTHESES_PATH,
        ROOM_SEGMENTATION_DIAGNOSTICS_PATH,
    ]
    input_hashes_before = {rel(p): file_sha256(p) for p in input_paths if p.exists()}

    bev_meta = read_json(BEV_META_PATH)
    resolution = float(bev_meta["resolution"])
    origin = [float(bev_meta["origin"][0]), float(bev_meta["origin"][1])]
    grid_shape = (int(bev_meta["height"]), int(bev_meta["width"]))

    print(f"[1] Loading Step29A3/B1/B1R/B3R artifacts for {SCENE_ID}")
    with np.load(BEV_NPZ_PATH) as bev_npz:
        bev = {name: bev_npz[name].copy() for name in bev_npz.files}
    room_mask = np.load(GLOBAL_ROOM_MASK_PATH).astype(np.int32)
    bev["room_mask_global_id"] = room_mask
    free_space = (bev["free_space"] > 0).astype(np.uint8)
    hypotheses = read_json(B3R_HYPOTHESES_PATH)["hypotheses"]

    print("[2] Reconstructing saved-density and pre-close wall candidate layers")
    layers, metadata, warnings, raw_explanation = build_candidates(bev, metadata_target_shape := grid_shape)
    np.savez_compressed(ASSETS_DIR / "00824_gateway_wall_layer_candidates_v0_1.npz", **layers)
    write_json(ASSETS_DIR / "00824_gateway_wall_layer_candidate_metadata_v0_1.json", metadata)

    code_changes = {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "script_path": rel(SCRIPT_PATH),
        "core_stage_a_runtime_modified": False,
        "code_changes": [
            {
                "path": rel(SCRIPT_PATH),
                "change_type": "new_offline_export_script",
                "purpose": "Export and benchmark gateway wall evidence candidates without changing Stage-A runtime behavior.",
            }
        ],
        "raw_debug_hook_added": False,
        "reason_no_core_change": "Frozen-artifact offline export was sufficient for saved-density threshold sweep; raw point data was unavailable for height-band sweep.",
    }
    write_json(ASSETS_DIR / "00824_step29a3r_wall_export_code_changes_v0_1.json", code_changes)

    print("[3] Benchmarking candidates against positives and r3_r11 negatives")
    benchmark, score_rows, threshold_rows = benchmark_layers(
        layers, metadata, hypotheses, resolution, grid_shape, room_mask, free_space
    )
    write_json(ASSETS_DIR / "00824_gateway_wall_doorway_benchmark_v0_1.json", benchmark)
    write_json(ASSETS_DIR / "00824_gateway_wall_layer_candidate_scores_v0_1.json", {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "score_formula": benchmark["score_formula"],
        "candidate_scores_ranked": score_rows,
    })

    print("[4] Writing threshold and height-band sweep artifacts")
    threshold_sweep = build_threshold_sweep(threshold_rows, benchmark)
    write_json(ASSETS_DIR / "00824_gateway_wall_threshold_sweep_v0_1.json", threshold_sweep)

    current_height_slice = load_current_height_slice()
    height_band_sweep = build_height_band_sweep(current_height_slice)
    write_json(ASSETS_DIR / "00824_gateway_wall_height_band_sweep_v0_1.json", height_band_sweep)

    recommended, best_auto, best_cal, rerun_needed, rationale = choose_recommendation(score_rows)
    stage_a_raw_debug_rerun_recommended = True
    recommended_layer_requires_stage_a_rerun = False
    if recommended is None:
        recommended_layer_requires_stage_a_rerun = True
    elif recommended["layer_name"] == "requires_stage_a_raw_debug_rerun":
        recommended_layer_requires_stage_a_rerun = True
    rationale = (
        rationale
        + " The recommended saved-artifact layer can be used for an immediate Step29B3/B3R rerun, "
        "but a future Stage-A raw debug rerun is still recommended because exact raw/pre-blur "
        "and height-band wall evidence was unavailable in the frozen artifacts."
    )
    rec_name = recommended["layer_name"] if recommended else "requires_stage_a_raw_debug_rerun"
    rec_layer_metrics = benchmark["layers"].get(rec_name, {}).get("hypotheses", {})
    recommendation = {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "recommended_gateway_wall_layer": rec_name,
        "recommended_layer_category": recommended["category"] if recommended else None,
        "recommended_layer_is_automatic": bool(recommended["automatic"]) if recommended else False,
        "recommended_layer_calibrated_from_user_labels": bool(recommended["calibrated_from_user_labels"]) if recommended else False,
        "best_automatic_layer": None if best_auto is None else best_auto["layer_name"],
        "best_automatic_layer_score": None if best_auto is None else best_auto["score"],
        "best_calibrated_reference_layer": None if best_cal is None else best_cal["layer_name"],
        "best_calibrated_reference_layer_score": None if best_cal is None else best_cal["score"],
        "stage_a_raw_debug_rerun_recommended": bool(stage_a_raw_debug_rerun_recommended),
        "recommended_layer_requires_stage_a_rerun": bool(recommended_layer_requires_stage_a_rerun),
        "requires_new_stage_a_rerun_with_extra_debug_export": bool(stage_a_raw_debug_rerun_recommended),
        "preserves_known_true_doorways": (
            bool(recommended)
            and all(rec_layer_metrics.get(hid, {}).get("wall_filter_status") in {"clear", "uncertain"} for hid in POSITIVE_IDS)
        ),
        "keeps_r3_r11_rejected": (
            bool(recommended)
            and all(rec_layer_metrics.get(hid, {}).get("wall_filter_status") == "blocked" for hid in NEGATIVE_IDS)
        ),
        "should_step29b3_b3r_be_rerun_with_this_layer": True,
        "wall_core_blocked_should_be_downgraded_from_hard_reject_to_conflict_flag": True,
        "segmentation_wall_processed_current_should_remain_segmentation_provenance_only": True,
        "rationale": rationale,
        "warnings": warnings + height_band_sweep["warnings"],
    }
    write_json(ASSETS_DIR / "00824_gateway_wall_recommendation_v0_1.json", recommendation)

    print("[5] Rendering map_xy visualizations")
    create_visualizations(
        layers, metadata, benchmark, score_rows, threshold_rows, hypotheses,
        room_mask, free_space, origin, resolution, recommended, best_auto, best_cal
    )

    summary = {
        "scene_id": SCENE_ID,
        "generated_at": now_iso(),
        "script_path": rel(SCRIPT_PATH),
        "output_directory": rel(OUTPUT_DIR),
        "wall_candidate_layer_count": len(layers),
        "raw_pre_blur_wall_histogram_available": False,
        "height_band_sweep_performed": False,
        "best_automatic_layer": None if best_auto is None else best_auto["layer_name"],
        "best_calibrated_reference_layer": None if best_cal is None else best_cal["layer_name"],
        "recommended_gateway_wall_layer": rec_name,
        "stage_a_rerun_recommended": bool(stage_a_raw_debug_rerun_recommended),
        "no_step29c_topology_generated": True,
        "no_ros_nav2_gazebo_run": True,
        "raw_layer_availability": raw_explanation,
        "warnings": recommendation["warnings"],
        "benchmark_status_for_recommended_layer": rec_layer_metrics,
        "top_ranked_scores": score_rows[:8],
    }
    write_json(ASSETS_DIR / "00824_step29a3r_summary_v0_1.json", summary)
    write_readme(summary, recommendation, height_band_sweep, recommendation["warnings"])

    input_hashes_after = {rel(p): file_sha256(p) for p in input_paths if p.exists()}
    validation = validate_outputs(layers, metadata, benchmark, recommendation, input_hashes_before, input_hashes_after)
    write_json(ASSETS_DIR / "00824_step29a3r_validation_results_v0_1.json", validation)
    if not validation["passed"]:
        raise RuntimeError(f"Validation failed: {validation['failed_checks']}")

    def status(hid):
        m = rec_layer_metrics.get(hid)
        if not m:
            return "missing"
        return f"{m['wall_filter_status']} (overlap={m['wall_core_overlap_ratio']:.4f}, gap={m['doorway_gap_score']:.4f})"

    neg_status = ", ".join(f"{hid[-2:]}={status(hid)}" for hid in NEGATIVE_IDS)
    print("\nCompletion summary")
    print(f"- script path: {rel(SCRIPT_PATH)}")
    print(f"- output directory: {rel(OUTPUT_DIR)}")
    print(f"- wall candidate layers exported: {len(layers)}")
    print("- raw pre-blur wall histogram available: false")
    print("- height-band sweep performed: false")
    print(f"- best automatic layer: {best_auto['layer_name'] if best_auto else None}")
    print(f"- best calibrated/reference layer: {best_cal['layer_name'] if best_cal else None}")
    print(f"- recommended_gateway_wall_layer: {rec_name}")
    print(f"- r1_r3_01: {status('hyp_00824_r1_r3_01')}")
    print(f"- r3_r7_01: {status('hyp_00824_r3_r7_01')}")
    print(f"- r7_r11_01: {status('hyp_00824_r7_r11_01')}")
    print(f"- r8_r11_01: {status('hyp_00824_r8_r11_01')}")
    print(f"- r3_r11 negative: {neg_status}")
    print(f"- Stage-A rerun recommended: {stage_a_raw_debug_rerun_recommended}")
    print(f"- validation path: {rel(ASSETS_DIR / '00824_step29a3r_validation_results_v0_1.json')}")
    print(f"- warnings: {len(recommendation['warnings'])}")
    for warning in recommendation["warnings"]:
        print(f"  - {warning}")
    print("- confirmation: no Step29C topology was generated")
    print("- confirmation: no ROS/Nav2/Gazebo was run")


if __name__ == "__main__":
    os.chdir(BASE_DIR)
    main()
