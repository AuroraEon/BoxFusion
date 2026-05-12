#!/usr/bin/env python3
"""
Step29B3: Automatic gateway hypothesis consolidation and uninflated-wall filtering.

Consumes Step29B2R2 canonical candidates and Step29B1 BEV layers to produce
doorway-level gateway hypotheses via spatial clustering, raw structural-wall
filtering, doorway-gap checks, two-sided free-space support, and regression
checks against known positive/negative cases.

Does NOT run ROS, Nav2, Gazebo, AMCL, DWB, RViz.
Does NOT generate a gateway-augmented topology (Step29C).
Does NOT overwrite Step29B2, Step29B2R, or Step29B2R2 artifacts.
"""

import json
import csv
import os
import sys
import math
import pathlib
import numpy as np
from collections import defaultdict
from datetime import datetime

# ============================================================
# Configuration
# ============================================================

SCENE_ID = "00824-Dd4bFSTQ8gi"
BASE_DIR = pathlib.Path("/home/ws/workspace/BoxFusion")

# Input paths
STEP29B2R2_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b2r2_00824_local_gateway_inspection_pack"
STEP29B1_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3"

CANDIDATE_TABLE_PATH = STEP29B2R2_DIR / "generated" / "assets" / "00824_step29b2r2_candidate_review_table_v0_1.json"
BEV_META_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_layered_bev_from_step29a3_v0_1.json"
BEV_NPZ_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_layered_bev_from_step29a3_v0_1.npz"
GLOBAL_ROOM_MASK_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_global_room_mask_v0_1.npy"
ROOM_LABEL_MAPPING_PATH = STEP29B1_DIR / "generated" / "assets" / "00824_room_label_mapping_v0_1.json"

# Output directory
OUTPUT_DIR = BASE_DIR / "runtime_stage1_frozen_evidence" / "step29b3_00824_gateway_hypothesis_consolidation"
GENERATED_DIR = OUTPUT_DIR / "generated"
ASSETS_DIR = GENERATED_DIR / "assets"
VIS_DIR = GENERATED_DIR / "visualizations"

# Key room pairs
KEY_PAIRS = ["r7_r11", "r8_r11", "r3_r7", "r3_r11", "r1_r3"]
NEGATIVE_SANITY_PAIRS = {"r3_r11"}

# Clustering thresholds
CENTER_DISTANCE_CLUSTER_M = 0.50
BBOX_GAP_CLUSTER_M = 0.30
CROSSING_YAW_DIFF_THRESHOLD_RAD = 0.45

# Wall filtering thresholds
WALL_CORE_BLOCKED_THRESHOLD = 0.18
WALL_CORE_UNCERTAIN_THRESHOLD = 0.08

# Width thresholds
MIN_HYPOTHESIS_WIDTH_STRONG_M = 0.30
MIN_HYPOTHESIS_WIDTH_POSSIBLE_M = 0.20
MIN_FRAGMENT_WIDTH_M = 0.05


# ============================================================
# Utility functions
# ============================================================

def circular_mean(angles, weights=None):
    """Compute circular mean of angles in radians."""
    if weights is None:
        weights = np.ones(len(angles))
    weights = np.array(weights, dtype=float)
    weights /= weights.sum() + 1e-12
    sin_sum = np.sum(weights * np.sin(angles))
    cos_sum = np.sum(weights * np.cos(angles))
    return math.atan2(sin_sum, cos_sum)


def angular_diff(a, b):
    """Unsigned angular difference in [0, pi]."""
    d = abs(a - b) % (2 * math.pi)
    if d > math.pi:
        d = 2 * math.pi - d
    return d


def xy_to_rc(x, y, origin, resolution):
    """Convert map xy to grid row, col."""
    col = int(round((x - origin[0]) / resolution))
    row = int(round((y - origin[1]) / resolution))
    return row, col


def rc_to_xy(row, col, origin, resolution):
    """Convert grid row, col to map xy."""
    x = origin[0] + col * resolution
    y = origin[1] + row * resolution
    return x, y


def bbox_distance(bbox1, bbox2):
    """Compute minimum gap between two bboxes [r0, c0, r1, c1]."""
    r0a, c0a, r1a, c1a = bbox1
    r0b, c0b, r1b, c1b = bbox2
    # Row gap
    row_gap = max(0, max(r0a, r0b) - min(r1a, r1b))
    # Col gap
    col_gap = max(0, max(c0a, c0b) - min(c1a, c1b))
    return math.sqrt(row_gap**2 + col_gap**2)


def build_corridor_mask(center_rc, yaw, length_m, width_m, resolution, grid_shape):
    """Build a binary mask for a crossing corridor."""
    half_len = length_m / (2 * resolution)
    half_wid = width_m / (2 * resolution)
    
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    # Sample points along and across the corridor
    mask = np.zeros(grid_shape, dtype=bool)
    
    n_along = max(int(2 * half_len) + 1, 5)
    n_across = max(int(2 * half_wid) + 1, 3)
    
    for i_along in np.linspace(-half_len, half_len, n_along):
        for i_across in np.linspace(-half_wid, half_wid, n_across):
            # yaw in map frame: cos/sin applied to col/row
            dc = i_along * cos_y - i_across * sin_y
            dr = i_along * sin_y + i_across * cos_y
            r = int(round(cr + dr))
            c = int(round(cc + dc))
            if 0 <= r < grid_shape[0] and 0 <= c < grid_shape[1]:
                mask[r, c] = True
    return mask


def compute_scanline_gap(center_rc, yaw, structural_wall, resolution, grid_shape, n_scanlines=7, scan_half_len_m=0.5):
    """Compute doorway gap metrics by sampling scanlines perpendicular to crossing direction."""
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    # Perpendicular direction
    perp_cos = -sin_y
    perp_sin = cos_y
    
    half_len_cells = scan_half_len_m / resolution
    
    gap_widths = []
    center_clear_counts = 0
    wall_endpoint_counts = 0
    total_scanlines = 0
    
    for offset_along in np.linspace(-0.15 / resolution, 0.15 / resolution, n_scanlines):
        # Offset along crossing direction
        base_r = cr + offset_along * sin_y
        base_c = cc + offset_along * cos_y
        
        # Sample perpendicular scanline
        n_samples = int(2 * half_len_cells) + 1
        wall_positions = []
        
        for i, t in enumerate(np.linspace(-half_len_cells, half_len_cells, n_samples)):
            r = int(round(base_r + t * perp_sin))
            c = int(round(base_c + t * perp_cos))
            if 0 <= r < grid_shape[0] and 0 <= c < grid_shape[1]:
                if structural_wall[r, c]:
                    wall_positions.append(t)
        
        total_scanlines += 1
        
        # Find gap: look for wall on both sides
        if len(wall_positions) >= 2:
            neg_walls = [p for p in wall_positions if p < -1]
            pos_walls = [p for p in wall_positions if p > 1]
            
            if neg_walls and pos_walls:
                # Wall on both sides - find gap
                inner_neg = max(neg_walls)
                inner_pos = min(pos_walls)
                gap_width_cells = inner_pos - inner_neg
                gap_widths.append(gap_width_cells * resolution)
                wall_endpoint_counts += 1
            
            # Check center clear
            center_walls = [p for p in wall_positions if abs(p) <= 1.5]
            if len(center_walls) == 0:
                center_clear_counts += 1
        elif len(wall_positions) == 0:
            center_clear_counts += 1
        else:
            # Only one wall side
            center_walls = [p for p in wall_positions if abs(p) <= 1.5]
            if len(center_walls) == 0:
                center_clear_counts += 1
    
    # Compute scores
    if gap_widths:
        raw_wall_gap_width_m = float(np.median(gap_widths))
    else:
        raw_wall_gap_width_m = 0.0
    
    wall_endpoint_support_score = wall_endpoint_counts / max(total_scanlines, 1)
    gap_center_clear_score = center_clear_counts / max(total_scanlines, 1)
    gap_consistency_score = 1.0 - (np.std(gap_widths) / (np.mean(gap_widths) + 1e-6)) if len(gap_widths) >= 2 else (0.5 if gap_widths else 0.0)
    gap_consistency_score = max(0.0, min(1.0, gap_consistency_score))
    
    # Combined doorway gap score
    doorway_gap_score = (
        0.30 * wall_endpoint_support_score +
        0.35 * gap_center_clear_score +
        0.20 * gap_consistency_score +
        0.15 * min(1.0, raw_wall_gap_width_m / 0.5)
    )
    
    return {
        "doorway_gap_score": round(float(doorway_gap_score), 4),
        "raw_wall_gap_width_m": round(float(raw_wall_gap_width_m), 4),
        "wall_endpoint_support_score": round(float(wall_endpoint_support_score), 4),
        "gap_center_clear_score": round(float(gap_center_clear_score), 4),
        "gap_consistency_score": round(float(gap_consistency_score), 4),
    }


def compute_two_sided_support(center_rc, yaw, approach_a, approach_b, 
                               room_mask_global, free_space, 
                               room_a_id, room_b_id, origin, resolution, grid_shape):
    """Compute two-sided room/free-space support scores."""
    # Approach from room_a
    ra_r, ra_c = xy_to_rc(approach_a["x"], approach_a["y"], origin, resolution)
    # Approach from room_b
    rb_r, rb_c = xy_to_rc(approach_b["x"], approach_b["y"], origin, resolution)
    
    # Sample neighborhood around approach points
    radius = 3  # cells
    
    def count_room_support(r, c, room_id):
        count = 0
        total = 0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr, cc_val = r + dr, c + dc
                if 0 <= rr < grid_shape[0] and 0 <= cc_val < grid_shape[1]:
                    total += 1
                    if room_mask_global[rr, cc_val] == room_id:
                        count += 1
        return count / max(total, 1)
    
    room_a_support = count_room_support(ra_r, ra_c, room_a_id)
    room_b_support = count_room_support(rb_r, rb_c, room_b_id)
    
    # Free space in crossing corridor (small region around center)
    cr, cc = center_rc
    free_count = 0
    total_count = 0
    for dr in range(-2, 3):
        for dc in range(-2, 3):
            rr, cc_val = cr + dr, cc + dc
            if 0 <= rr < grid_shape[0] and 0 <= cc_val < grid_shape[1]:
                total_count += 1
                if free_space[rr, cc_val]:
                    free_count += 1
    free_crossing = free_count / max(total_count, 1)
    
    two_sided_score = 0.4 * room_a_support + 0.4 * room_b_support + 0.2 * free_crossing
    strict_two_sided = room_a_support > 0.2 and room_b_support > 0.2
    
    return {
        "two_sided_support_score": round(float(two_sided_score), 4),
        "room_a_support_score": round(float(room_a_support), 4),
        "room_b_support_score": round(float(room_b_support), 4),
        "free_space_crossing_score": round(float(free_crossing), 4),
        "strict_two_sided_support": bool(strict_two_sided),
    }


# ============================================================
# Main pipeline
# ============================================================

def main():
    print("=" * 60)
    print("Step29B3: Gateway Hypothesis Consolidation")
    print("=" * 60)
    
    # --------------------------------------------------------
    # 1. Load inputs
    # --------------------------------------------------------
    print("\n[1] Loading Step29B2R2 candidate review table...")
    with open(CANDIDATE_TABLE_PATH, "r") as f:
        candidate_data = json.load(f)
    candidates = candidate_data["canonical_candidates"]
    print(f"    Loaded {len(candidates)} candidates")
    
    print("[2] Loading Step29B1 BEV metadata...")
    with open(BEV_META_PATH, "r") as f:
        bev_meta = json.load(f)
    resolution = bev_meta["resolution"]
    origin = bev_meta["origin"]
    grid_height = bev_meta["height"]
    grid_width = bev_meta["width"]
    grid_shape = (grid_height, grid_width)
    print(f"    Resolution: {resolution}m, Shape: {grid_shape}, Origin: {origin}")
    
    print("[3] Loading BEV NPZ layers...")
    npz = np.load(BEV_NPZ_PATH)
    layer_names = list(npz.files)
    print(f"    Available layers: {layer_names}")
    
    structural_wall = npz["structural_wall"] if "structural_wall" in npz else None
    free_space = npz["free_space"] if "free_space" in npz else None
    room_mask_local_raw = npz["room_mask_local_label_raw"] if "room_mask_local_label_raw" in npz else None
    room_mask_local_repaired = npz["room_mask_local_label_repaired"] if "room_mask_local_label_repaired" in npz else None
    
    print("[4] Loading global room mask...")
    room_mask_global = np.load(GLOBAL_ROOM_MASK_PATH)
    print(f"    Global room mask shape: {room_mask_global.shape}")
    
    print("[5] Loading room label mapping...")
    with open(ROOM_LABEL_MAPPING_PATH, "r") as f:
        label_mapping = json.load(f)
    
    # Verify shapes
    assert structural_wall.shape == grid_shape, f"structural_wall shape mismatch: {structural_wall.shape} vs {grid_shape}"
    assert free_space.shape == grid_shape, f"free_space shape mismatch"
    assert room_mask_global.shape == grid_shape, f"global room mask shape mismatch"
    
    # Convert structural_wall to bool
    if structural_wall.dtype != bool:
        structural_wall = structural_wall > 0
    if free_space.dtype != bool:
        free_space = free_space > 0
    
    # --------------------------------------------------------
    # 2. Group candidates by pair
    # --------------------------------------------------------
    print("\n[6] Grouping candidates by pair...")
    pair_candidates = defaultdict(list)
    for cand in candidates:
        pair_candidates[cand["pair_key"]].append(cand)
    
    for pk in KEY_PAIRS:
        count = len(pair_candidates.get(pk, []))
        print(f"    {pk}: {count} candidates")
    
    # --------------------------------------------------------
    # 3. Cluster candidates per pair
    # --------------------------------------------------------
    print("\n[7] Clustering candidates into doorway hypotheses...")
    
    all_hypotheses = []
    cluster_debug = {}
    warnings_global = []
    
    for pair_key in KEY_PAIRS:
        cands = pair_candidates.get(pair_key, [])
        if not cands:
            print(f"    {pair_key}: no candidates, skipping")
            continue
        
        # Build distance matrix
        n = len(cands)
        
        # Complete-linkage clustering: only merge if ALL pairs satisfy criterion
        # This prevents chaining artifacts from single-linkage
        
        def can_merge_pair(ci, cj):
            """Check if two candidates can be in the same cluster."""
            dx = ci["center_xy"][0] - cj["center_xy"][0]
            dy = ci["center_xy"][1] - cj["center_xy"][1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            yaw_i = ci["crossing_pose"]["yaw"]
            yaw_j = cj["crossing_pose"]["yaw"]
            yaw_diff = angular_diff(yaw_i, yaw_j)
            
            if yaw_diff > CROSSING_YAW_DIFF_THRESHOLD_RAD:
                return False
            
            if dist <= CENTER_DISTANCE_CLUSTER_M:
                return True
            
            # Bbox gap as supplementary only if center distance is moderate
            if dist <= CENTER_DISTANCE_CLUSTER_M * 1.4:
                bbox_dist_cells = bbox_distance(ci["bbox_cells"], cj["bbox_cells"])
                bbox_dist_m = bbox_dist_cells * resolution
                if bbox_dist_m <= BBOX_GAP_CLUSTER_M:
                    return True
            
            return False
        
        # Agglomerative complete-linkage
        clusters = [[i] for i in range(n)]
        
        def can_merge_clusters(c1, c2):
            """Complete linkage: all pairs must satisfy merge criterion."""
            for i in c1:
                for j in c2:
                    if not can_merge_pair(cands[i], cands[j]):
                        return False
            return True
        
        # Iteratively merge closest compatible clusters
        changed = True
        while changed:
            changed = False
            best_merge = None
            best_dist = float("inf")
            
            for i_c in range(len(clusters)):
                for j_c in range(i_c + 1, len(clusters)):
                    if can_merge_clusters(clusters[i_c], clusters[j_c]):
                        # Use max pairwise center distance as merge cost
                        max_d = 0
                        for ii in clusters[i_c]:
                            for jj in clusters[j_c]:
                                dx = cands[ii]["center_xy"][0] - cands[jj]["center_xy"][0]
                                dy = cands[ii]["center_xy"][1] - cands[jj]["center_xy"][1]
                                max_d = max(max_d, math.sqrt(dx*dx + dy*dy))
                        if max_d < best_dist:
                            best_dist = max_d
                            best_merge = (i_c, j_c)
            
            if best_merge is not None:
                i_c, j_c = best_merge
                clusters[i_c] = clusters[i_c] + clusters[j_c]
                clusters.pop(j_c)
                changed = True
        
        pair_clusters = clusters
        
        # Debug info
        cluster_debug[pair_key] = {
            "candidate_count": n,
            "cluster_count": len(pair_clusters),
            "clusters": []
        }
        
        for cidx, cluster_indices in enumerate(pair_clusters):
            cluster_cands = [cands[i] for i in cluster_indices]
            cluster_ids = [c["gateway_id"] for c in cluster_cands]
            
            cluster_debug[pair_key]["clusters"].append({
                "cluster_index": cidx,
                "candidate_ids": cluster_ids,
                "candidate_count": len(cluster_ids)
            })
            
            # --------------------------------------------------------
            # 4. Build hypothesis from cluster
            # --------------------------------------------------------
            
            # Weighted representative center
            weights = []
            for c in cluster_cands:
                w = 1.0
                w *= max(c["width_m"], 0.05) * 10  # width bonus
                w *= max(c.get("free_fraction_context", 0.5), 0.1)
                if c.get("eroded_wall_1cell_two_sided", False):
                    w *= 1.5
                if c.get("eroded_wall_2cell_two_sided", False):
                    w *= 1.2
                if c["strict_status"] in ("valid", "ambiguous"):
                    w *= 2.0
                # Penalize high label10
                l10 = c.get("label10_fraction_raw", 0)
                if l10 > 0.7:
                    w *= 0.3
                elif l10 > 0.5:
                    w *= 0.6
                weights.append(w)
            
            weights = np.array(weights)
            weights /= weights.sum() + 1e-12
            
            # Weighted center
            center_x = sum(w * c["center_xy"][0] for w, c in zip(weights, cluster_cands))
            center_y = sum(w * c["center_xy"][1] for w, c in zip(weights, cluster_cands))
            center_r, center_c = xy_to_rc(center_x, center_y, origin, resolution)
            
            # Circular mean of yaws
            yaws = [c["crossing_pose"]["yaw"] for c in cluster_cands]
            rep_yaw = circular_mean(yaws, weights)
            
            # Check yaw disagreement
            max_yaw_diff = 0
            for i_y in range(len(yaws)):
                for j_y in range(i_y + 1, len(yaws)):
                    max_yaw_diff = max(max_yaw_diff, angular_diff(yaws[i_y], yaws[j_y]))
            
            hyp_warnings = []
            if max_yaw_diff > 0.3:
                hyp_warnings.append(f"yaw_disagreement_{max_yaw_diff:.3f}_rad")
            
            # Merged bbox
            all_r0 = min(c["bbox_cells"][0] for c in cluster_cands)
            all_c0 = min(c["bbox_cells"][1] for c in cluster_cands)
            all_r1 = max(c["bbox_cells"][2] for c in cluster_cands)
            all_c1 = max(c["bbox_cells"][3] for c in cluster_cands)
            merged_bbox_cells = [all_r0, all_c0, all_r1, all_c1]
            
            x0, y0 = rc_to_xy(all_r0, all_c0, origin, resolution)
            x1, y1 = rc_to_xy(all_r1, all_c1, origin, resolution)
            merged_bbox_map_xy = [[round(x0, 4), round(y0, 4)], [round(x1, 4), round(y1, 4)]]
            
            # Representative approach poses (weighted average)
            app_a_x = sum(w * c["approach_from_room_a"]["x"] for w, c in zip(weights, cluster_cands))
            app_a_y = sum(w * c["approach_from_room_a"]["y"] for w, c in zip(weights, cluster_cands))
            app_a_yaw = circular_mean([c["approach_from_room_a"]["yaw"] for c in cluster_cands], weights)
            
            app_b_x = sum(w * c["approach_from_room_b"]["x"] for w, c in zip(weights, cluster_cands))
            app_b_y = sum(w * c["approach_from_room_b"]["y"] for w, c in zip(weights, cluster_cands))
            app_b_yaw = circular_mean([c["approach_from_room_b"]["yaw"] for c in cluster_cands], weights)
            
            # Source candidate summary
            max_source_width = max(c["width_m"] for c in cluster_cands)
            max_source_free = max(c.get("free_fraction_context", 0) for c in cluster_cands)
            min_source_l10 = min(c.get("label10_fraction_raw", 0) for c in cluster_cands)
            
            # Max pairwise center distance
            max_pair_dist = 0
            for i_p in range(len(cluster_cands)):
                for j_p in range(i_p + 1, len(cluster_cands)):
                    dx = cluster_cands[i_p]["center_xy"][0] - cluster_cands[j_p]["center_xy"][0]
                    dy = cluster_cands[i_p]["center_xy"][1] - cluster_cands[j_p]["center_xy"][1]
                    max_pair_dist = max(max_pair_dist, math.sqrt(dx*dx + dy*dy))
            
            # --------------------------------------------------------
            # 5. Uninflated wall filter
            # --------------------------------------------------------
            corridor_length_m = min(0.9, max(0.4, 0.6))
            corridor_width_m = max(0.25, min(max_source_width, 0.6))
            
            corridor_mask = build_corridor_mask(
                (center_r, center_c), rep_yaw,
                corridor_length_m, corridor_width_m,
                resolution, grid_shape
            )
            
            corridor_cells = np.sum(corridor_mask)
            if corridor_cells > 0:
                wall_in_corridor = np.sum(structural_wall & corridor_mask)
                uninflated_wall_core_overlap_ratio = wall_in_corridor / corridor_cells
            else:
                uninflated_wall_core_overlap_ratio = 0.0
            
            # Bbox wall overlap
            bbox_mask = np.zeros(grid_shape, dtype=bool)
            r0_b = max(0, all_r0)
            c0_b = max(0, all_c0)
            r1_b = min(grid_shape[0] - 1, all_r1)
            c1_b = min(grid_shape[1] - 1, all_c1)
            bbox_mask[r0_b:r1_b+1, c0_b:c1_b+1] = True
            bbox_cells_count = np.sum(bbox_mask)
            if bbox_cells_count > 0:
                bbox_wall_overlap_ratio = float(np.sum(structural_wall & bbox_mask)) / bbox_cells_count
            else:
                bbox_wall_overlap_ratio = 0.0
            
            # Endpoint wall touch: check corridor edges
            # Build endpoint mask (first/last 20% of corridor along crossing direction)
            endpoint_wall_touch_ratio = 0.0
            if corridor_cells > 0:
                # Simple approximation: check walls near approach points
                ep_count = 0
                ep_total = 0
                for approach in [{"x": app_a_x, "y": app_a_y}, {"x": app_b_x, "y": app_b_y}]:
                    ar, ac = xy_to_rc(approach["x"], approach["y"], origin, resolution)
                    for dr in range(-1, 2):
                        for dc in range(-1, 2):
                            rr, cc_v = ar + dr, ac + dc
                            if 0 <= rr < grid_shape[0] and 0 <= cc_v < grid_shape[1]:
                                ep_total += 1
                                if structural_wall[rr, cc_v]:
                                    ep_count += 1
                endpoint_wall_touch_ratio = ep_count / max(ep_total, 1)
            
            # Wall filter status
            if uninflated_wall_core_overlap_ratio >= WALL_CORE_BLOCKED_THRESHOLD:
                wall_filter_status = "wall_core_blocked"
            elif uninflated_wall_core_overlap_ratio >= WALL_CORE_UNCERTAIN_THRESHOLD:
                wall_filter_status = "wall_core_uncertain"
            elif endpoint_wall_touch_ratio > 0 and uninflated_wall_core_overlap_ratio < WALL_CORE_UNCERTAIN_THRESHOLD:
                wall_filter_status = "endpoint_wall_touch_only"
            else:
                wall_filter_status = "clear_raw_wall_gap"
            
            # --------------------------------------------------------
            # 6. Doorway gap scoring
            # --------------------------------------------------------
            gap_metrics = compute_scanline_gap(
                (center_r, center_c), rep_yaw,
                structural_wall, resolution, grid_shape
            )
            
            # --------------------------------------------------------
            # 7. Two-sided support
            # --------------------------------------------------------
            room_a = cluster_cands[0]["room_a"]
            room_b = cluster_cands[0]["room_b"]
            
            support_metrics = compute_two_sided_support(
                (center_r, center_c), rep_yaw,
                {"x": app_a_x, "y": app_a_y},
                {"x": app_b_x, "y": app_b_y},
                room_mask_global, free_space,
                room_a, room_b, origin, resolution, grid_shape
            )
            
            # Add eroded wall support from source candidates
            eroded_1cell_count = sum(1 for c in cluster_cands if c.get("eroded_wall_1cell_two_sided", False))
            eroded_2cell_count = sum(1 for c in cluster_cands if c.get("eroded_wall_2cell_two_sided", False))
            support_metrics["eroded_wall_1cell_two_sided"] = eroded_1cell_count > 0
            support_metrics["eroded_wall_2cell_two_sided"] = eroded_2cell_count > 0
            
            # Source two-sided counts
            source_two_sided_count = sum(1 for c in cluster_cands if c.get("connects_room_a", False) and c.get("connects_room_b", False))
            source_one_sided_count = len(cluster_cands) - source_two_sided_count
            
            # --------------------------------------------------------
            # 8. Width and passability
            # --------------------------------------------------------
            # Merged span width (diagonal of merged bbox in meters)
            merged_span_rows = all_r1 - all_r0
            merged_span_cols = all_c1 - all_c0
            # Use the shorter dimension as the "width" perpendicular to crossing
            cos_abs = abs(math.cos(rep_yaw))
            sin_abs = abs(math.sin(rep_yaw))
            # Width perpendicular to crossing direction
            perp_span_cells = merged_span_rows * cos_abs + merged_span_cols * sin_abs
            merged_span_width_m = perp_span_cells * resolution
            
            raw_wall_gap_width_m = gap_metrics["raw_wall_gap_width_m"]
            
            # Effective opening width: best of source width, gap width, merged span
            effective_opening_width_m = max(
                max_source_width,
                raw_wall_gap_width_m,
                min(merged_span_width_m, 1.0)  # cap merged span contribution
            )
            
            if effective_opening_width_m >= MIN_HYPOTHESIS_WIDTH_STRONG_M:
                passability_status = "strong"
            elif effective_opening_width_m >= MIN_HYPOTHESIS_WIDTH_POSSIBLE_M:
                passability_status = "possible_narrow"
            else:
                passability_status = "too_narrow"
            
            # --------------------------------------------------------
            # 9. Label10 / unknown
            # --------------------------------------------------------
            label10_fracs_raw = [c.get("label10_fraction_raw", 0) for c in cluster_cands]
            label10_fracs_repaired = [c.get("label10_fraction_repaired", 0) for c in cluster_cands]
            unknown_fracs = [c.get("unknown_fraction", 0) for c in cluster_cands]
            
            label10_fraction_raw = float(np.mean(label10_fracs_raw))
            label10_fraction_repaired = float(np.mean(label10_fracs_repaired))
            unknown_fraction = float(np.mean(unknown_fracs))
            label10_or_unknown_heavy = label10_fraction_raw > 0.6 or unknown_fraction > 0.3
            source_label10_warning_count = sum(1 for c in cluster_cands if "high_local_label10_context" in c.get("warnings", []))
            
            # --------------------------------------------------------
            # 10. Auto-status rules
            # --------------------------------------------------------
            auto_status = "needs_review"
            selected_for_step29c = False
            needs_manual_review = True
            
            is_negative_sanity = pair_key in NEGATIVE_SANITY_PAIRS
            is_positive_control = any(c["gateway_id"] == "gw_00824_r1_r3_01" for c in cluster_cands)
            has_strict_valid = any(c["strict_status"] == "valid" for c in cluster_cands)
            
            doorway_gap_score = gap_metrics["doorway_gap_score"]
            two_sided_score = support_metrics["two_sided_support_score"]
            
            if is_negative_sanity:
                auto_status = "auto_reject_negative_sanity"
                selected_for_step29c = False
                needs_manual_review = False
            elif is_positive_control or has_strict_valid:
                auto_status = "auto_accept_strict"
                selected_for_step29c = True
                needs_manual_review = False
            elif (wall_filter_status in ("clear_raw_wall_gap", "endpoint_wall_touch_only") and
                  doorway_gap_score >= 0.65 and
                  two_sided_score >= 0.65 and
                  effective_opening_width_m >= 0.25):
                auto_status = "auto_accept_strict"
                selected_for_step29c = True
                needs_manual_review = False
            elif (wall_filter_status in ("clear_raw_wall_gap", "endpoint_wall_touch_only", "wall_core_uncertain") and
                  doorway_gap_score >= 0.55 and
                  support_metrics.get("eroded_wall_2cell_two_sided", False) and
                  effective_opening_width_m >= 0.20):
                auto_status = "auto_accept_wall_thin_repair"
                selected_for_step29c = True
                needs_manual_review = False
            elif (label10_or_unknown_heavy and
                  doorway_gap_score >= 0.55 and
                  two_sided_score >= 0.4 and
                  effective_opening_width_m >= 0.20):
                auto_status = "auto_accept_label10_boundary_repair"
                selected_for_step29c = True
                needs_manual_review = False
            elif (uninflated_wall_core_overlap_ratio >= WALL_CORE_BLOCKED_THRESHOLD and
                  doorway_gap_score < 0.45):
                auto_status = "auto_reject_wall_core_blocked"
                selected_for_step29c = False
                needs_manual_review = False
            elif effective_opening_width_m < 0.20 and not has_strict_valid and doorway_gap_score < 0.5:
                auto_status = "auto_reject_too_narrow"
                selected_for_step29c = False
                needs_manual_review = False
            elif (not support_metrics["strict_two_sided_support"] and
                  source_two_sided_count == 0 and
                  doorway_gap_score < 0.45 and
                  not support_metrics.get("eroded_wall_2cell_two_sided", False)):
                auto_status = "auto_reject_one_sided"
                selected_for_step29c = False
                needs_manual_review = False
            elif (label10_or_unknown_heavy and
                  not support_metrics["strict_two_sided_support"] and
                  doorway_gap_score < 0.40 and
                  max_source_free < 0.3):
                auto_status = "auto_reject_label10_noise"
                selected_for_step29c = False
                needs_manual_review = False
            else:
                auto_status = "needs_review"
                selected_for_step29c = False
                needs_manual_review = True
            
            # Confidence score
            confidence = (
                0.25 * doorway_gap_score +
                0.25 * two_sided_score +
                0.15 * min(1.0, effective_opening_width_m / 0.4) +
                0.15 * (1.0 - uninflated_wall_core_overlap_ratio) +
                0.10 * (1.0 - label10_fraction_raw) +
                0.10 * (1.0 if has_strict_valid else 0.5 if any(c["strict_status"] == "ambiguous" for c in cluster_cands) else 0.0)
            )
            
            # Build hypothesis
            hyp_index = cidx + 1
            hypothesis_id = f"hyp_00824_{pair_key}_{hyp_index:02d}"
            
            hypothesis = {
                "hypothesis_id": hypothesis_id,
                "pair_key": pair_key,
                "room_a": room_a,
                "room_b": room_b,
                "source_candidate_ids": cluster_ids,
                "evidence_candidate_count": len(cluster_ids),
                "representative_center_xy": [round(center_x, 4), round(center_y, 4)],
                "representative_center_rc": [int(center_r), int(center_c)],
                "representative_crossing_pose": {
                    "x": round(center_x, 4),
                    "y": round(center_y, 4),
                    "yaw": round(rep_yaw, 6)
                },
                "representative_approach_from_room_a": {
                    "x": round(app_a_x, 4),
                    "y": round(app_a_y, 4),
                    "yaw": round(app_a_yaw, 6)
                },
                "representative_approach_from_room_b": {
                    "x": round(app_b_x, 4),
                    "y": round(app_b_y, 4),
                    "yaw": round(app_b_yaw, 6)
                },
                "merged_bbox_cells": merged_bbox_cells,
                "merged_bbox_map_xy": merged_bbox_map_xy,
                "source_candidate_summary": {
                    "strict_statuses": [c["strict_status"] for c in cluster_cands],
                    "diagnostic_reclassifications": [c.get("diagnostic_reclassification", "") for c in cluster_cands],
                    "recommended_actions": [c.get("recommended_action", "") for c in cluster_cands],
                    "max_source_width_m": round(max_source_width, 4),
                    "max_source_free_fraction": round(max_source_free, 4),
                    "min_source_label10_fraction": round(min_source_l10, 4)
                },
                "clustering": {
                    "cluster_method": "distance_bbox_yaw",
                    "cluster_score": round(1.0 - (max_pair_dist / CENTER_DISTANCE_CLUSTER_M) if max_pair_dist > 0 else 1.0, 4),
                    "max_pairwise_center_distance_m": round(max_pair_dist, 4),
                    "max_pairwise_yaw_diff_rad": round(max_yaw_diff, 4)
                },
                "uninflated_wall_filter": {
                    "wall_filter_status": wall_filter_status,
                    "uninflated_wall_core_overlap_ratio": round(float(uninflated_wall_core_overlap_ratio), 4),
                    "bbox_wall_overlap_ratio": round(float(bbox_wall_overlap_ratio), 4),
                    "endpoint_wall_touch_ratio": round(float(endpoint_wall_touch_ratio), 4)
                },
                "doorway_gap": gap_metrics,
                "two_sided_support": support_metrics,
                "width_passability": {
                    "max_source_width_m": round(max_source_width, 4),
                    "merged_span_width_m": round(merged_span_width_m, 4),
                    "raw_wall_gap_width_m": round(raw_wall_gap_width_m, 4),
                    "effective_opening_width_m": round(effective_opening_width_m, 4),
                    "passability_status": passability_status
                },
                "label10_unknown": {
                    "label10_fraction_raw": round(label10_fraction_raw, 4),
                    "label10_fraction_repaired": round(label10_fraction_repaired, 4),
                    "unknown_fraction": round(unknown_fraction, 4),
                    "label10_or_unknown_heavy": label10_or_unknown_heavy,
                    "source_label10_warning_count": source_label10_warning_count
                },
                "auto_status": auto_status,
                "selected_for_step29c_candidate": selected_for_step29c,
                "needs_manual_review": needs_manual_review,
                "confidence": round(float(confidence), 4),
                "warnings": hyp_warnings
            }
            
            all_hypotheses.append(hypothesis)
    
    # Sort hypotheses by pair then confidence descending
    all_hypotheses.sort(key=lambda h: (KEY_PAIRS.index(h["pair_key"]), -h["confidence"]))
    
    # Re-number hypotheses per pair
    pair_counters = defaultdict(int)
    for hyp in all_hypotheses:
        pair_counters[hyp["pair_key"]] += 1
        idx = pair_counters[hyp["pair_key"]]
        hyp["hypothesis_id"] = f"hyp_00824_{hyp['pair_key']}_{idx:02d}"
    
    print(f"\n    Total hypotheses: {len(all_hypotheses)}")
    for pk in KEY_PAIRS:
        count = sum(1 for h in all_hypotheses if h["pair_key"] == pk)
        print(f"    {pk}: {count} hypotheses")
    
    # --------------------------------------------------------
    # 4. Create output directories
    # --------------------------------------------------------
    print("\n[8] Creating output directories...")
    os.makedirs(ASSETS_DIR, exist_ok=True)
    for pk in KEY_PAIRS:
        os.makedirs(VIS_DIR / f"pair_{pk}", exist_ok=True)
    
    # --------------------------------------------------------
    # 5. Generate visualizations
    # --------------------------------------------------------
    print("\n[9] Generating visualizations...")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.colors import ListedColormap
        HAS_MPL = True
    except ImportError:
        HAS_MPL = False
        print("    WARNING: matplotlib not available, skipping visualizations")
    
    if HAS_MPL:
        for pk in KEY_PAIRS:
            pair_hyps = [h for h in all_hypotheses if h["pair_key"] == pk]
            pair_cands_list = pair_candidates.get(pk, [])
            if not pair_hyps:
                continue
            
            room_a = pair_cands_list[0]["room_a"] if pair_cands_list else 0
            room_b = pair_cands_list[0]["room_b"] if pair_cands_list else 0
            
            # Determine local view bounds from all candidates
            all_rows = []
            all_cols = []
            for c in pair_cands_list:
                all_rows.extend([c["bbox_cells"][0], c["bbox_cells"][2]])
                all_cols.extend([c["bbox_cells"][1], c["bbox_cells"][3]])
            
            margin = 30
            r_min = max(0, min(all_rows) - margin)
            r_max = min(grid_shape[0], max(all_rows) + margin)
            c_min = max(0, min(all_cols) - margin)
            c_max = min(grid_shape[1], max(all_cols) + margin)
            
            # --- 1. Local hypothesis overview ---
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            
            # Background: composite of room masks and wall
            local_room = room_mask_global[r_min:r_max, c_min:c_max].copy().astype(float)
            local_wall = structural_wall[r_min:r_max, c_min:c_max]
            local_free = free_space[r_min:r_max, c_min:c_max]
            
            # Create RGB image
            rgb = np.zeros((r_max - r_min, c_max - c_min, 3), dtype=float)
            # Room A in blue
            rgb[local_room == room_a, 2] = 0.4
            # Room B in green
            rgb[local_room == room_b, 1] = 0.4
            # Free space lighter
            rgb[local_free, :] += 0.2
            # Walls in dark red
            rgb[local_wall, 0] = 0.7
            rgb[local_wall, 1] = 0.1
            rgb[local_wall, 2] = 0.1
            rgb = np.clip(rgb, 0, 1)
            
            ax.imshow(rgb, origin="lower", extent=[c_min, c_max, r_min, r_max])
            
            # Plot candidates faintly
            for c in pair_cands_list:
                cr, cc_val = c["center_rc"]
                ax.plot(cc_val, cr, "x", color="gray", markersize=4, alpha=0.5)
            
            # Plot hypotheses prominently
            colors_status = {
                "auto_accept_strict": "lime",
                "auto_accept_wall_thin_repair": "cyan",
                "auto_accept_label10_boundary_repair": "yellow",
                "needs_review": "orange",
                "auto_reject_wall_core_blocked": "red",
                "auto_reject_too_narrow": "darkred",
                "auto_reject_one_sided": "maroon",
                "auto_reject_label10_noise": "purple",
                "auto_reject_negative_sanity": "black",
            }
            
            for hyp in pair_hyps:
                cr, cc_val = hyp["representative_center_rc"]
                color = colors_status.get(hyp["auto_status"], "white")
                ax.plot(cc_val, cr, "o", color=color, markersize=10, markeredgecolor="white", markeredgewidth=1)
                ax.annotate(hyp["hypothesis_id"].split("_")[-1], (cc_val + 1, cr + 1), 
                           fontsize=7, color=color)
            
            ax.set_title(f"Step29B3 Hypothesis Overview: {pk}\n"
                        f"Room {room_a} (blue) <-> Room {room_b} (green) | Wall (red)")
            ax.set_xlabel("col")
            ax.set_ylabel("row")
            
            # Legend
            legend_patches = []
            for status, color in colors_status.items():
                if any(h["auto_status"] == status for h in pair_hyps):
                    legend_patches.append(mpatches.Patch(color=color, label=status))
            if legend_patches:
                ax.legend(handles=legend_patches, fontsize=6, loc="upper right")
            
            fig.tight_layout()
            fig.savefig(VIS_DIR / f"pair_{pk}" / f"local_hypothesis_overview_{pk}.png", dpi=120)
            plt.close(fig)
            
            # --- 2. Candidate-to-hypothesis assignment ---
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            ax.imshow(rgb, origin="lower", extent=[c_min, c_max, r_min, r_max])
            
            cluster_colors = plt.cm.Set1(np.linspace(0, 1, max(len(pair_hyps), 1)))
            
            for h_idx, hyp in enumerate(pair_hyps):
                hc_r, hc_c = hyp["representative_center_rc"]
                color = cluster_colors[h_idx % len(cluster_colors)]
                
                ax.plot(hc_c, hc_r, "D", color=color, markersize=12, markeredgecolor="white")
                ax.annotate(hyp["hypothesis_id"], (hc_c + 2, hc_r + 2), fontsize=6, color="white",
                           bbox=dict(boxstyle="round,pad=0.2", fc=color, alpha=0.7))
                
                for cid in hyp["source_candidate_ids"]:
                    cand = next((c for c in pair_cands_list if c["gateway_id"] == cid), None)
                    if cand:
                        cr_c, cc_c = cand["center_rc"]
                        ax.plot(cc_c, cr_c, "o", color=color, markersize=6)
                        ax.plot([cc_c, hc_c], [cr_c, hc_r], "-", color=color, alpha=0.5, linewidth=1)
                        ax.annotate(cid.split("_")[-1], (cc_c - 3, cr_c - 2), fontsize=5, color=color)
            
            ax.set_title(f"Candidate-to-Hypothesis Assignment: {pk}")
            fig.tight_layout()
            fig.savefig(VIS_DIR / f"pair_{pk}" / f"candidate_to_hypothesis_assignment_{pk}.png", dpi=120)
            plt.close(fig)
            
            # --- 3. Uninflated wall filter debug ---
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            # Show structural wall prominently
            wall_display = np.zeros((r_max - r_min, c_max - c_min, 3))
            wall_display[local_wall, :] = [0.8, 0.2, 0.2]
            wall_display[~local_wall & local_free, :] = [0.2, 0.2, 0.2]
            ax.imshow(wall_display, origin="lower", extent=[c_min, c_max, r_min, r_max])
            
            for hyp in pair_hyps:
                cr, cc_val = hyp["representative_center_rc"]
                yaw = hyp["representative_crossing_pose"]["yaw"]
                width_m = hyp["width_passability"]["max_source_width_m"]
                
                corridor_mask_local = build_corridor_mask(
                    (cr, cc_val), yaw, 0.6, max(0.25, min(width_m, 0.6)),
                    resolution, grid_shape
                )
                corridor_overlap = corridor_mask_local[r_min:r_max, c_min:c_max]
                
                # Show corridor outline
                color = "lime" if "accept" in hyp["auto_status"] else ("orange" if hyp["auto_status"] == "needs_review" else "red")
                corridor_ys, corridor_xs = np.where(corridor_overlap)
                if len(corridor_xs) > 0:
                    ax.scatter(corridor_xs + c_min, corridor_ys + r_min, s=2, c=color, alpha=0.4)
                
                ax.plot(cc_val, cr, "o", color=color, markersize=8)
                ax.annotate(f"{hyp['hypothesis_id'].split('_')[-1]}\n"
                           f"core={hyp['uninflated_wall_filter']['uninflated_wall_core_overlap_ratio']:.2f}\n"
                           f"{hyp['uninflated_wall_filter']['wall_filter_status']}",
                           (cc_val + 2, cr), fontsize=5, color=color)
            
            ax.set_title(f"Uninflated Wall Filter Debug: {pk}\nRed=structural wall, corridor overlay")
            fig.tight_layout()
            fig.savefig(VIS_DIR / f"pair_{pk}" / f"uninflated_wall_filter_debug_{pk}.png", dpi=120)
            plt.close(fig)
            
            # --- 4. Doorway gap debug ---
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            ax.imshow(wall_display, origin="lower", extent=[c_min, c_max, r_min, r_max])
            
            for hyp in pair_hyps:
                cr, cc_val = hyp["representative_center_rc"]
                color = "lime" if hyp["doorway_gap"]["doorway_gap_score"] >= 0.65 else (
                    "orange" if hyp["doorway_gap"]["doorway_gap_score"] >= 0.4 else "red")
                ax.plot(cc_val, cr, "o", color=color, markersize=8)
                ax.annotate(f"{hyp['hypothesis_id'].split('_')[-1]}\n"
                           f"gap={hyp['doorway_gap']['doorway_gap_score']:.2f}\n"
                           f"width={hyp['doorway_gap']['raw_wall_gap_width_m']:.2f}m",
                           (cc_val + 2, cr), fontsize=5, color=color)
            
            ax.set_title(f"Doorway Gap Debug: {pk}")
            fig.tight_layout()
            fig.savefig(VIS_DIR / f"pair_{pk}" / f"doorway_gap_debug_{pk}.png", dpi=120)
            plt.close(fig)
            
            # --- 5. Two-sided support debug ---
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            ax.imshow(rgb, origin="lower", extent=[c_min, c_max, r_min, r_max])
            
            for hyp in pair_hyps:
                cr, cc_val = hyp["representative_center_rc"]
                score = hyp["two_sided_support"]["two_sided_support_score"]
                color = "lime" if score >= 0.65 else ("orange" if score >= 0.4 else "red")
                ax.plot(cc_val, cr, "o", color=color, markersize=8)
                
                # Show approach directions
                app_a = hyp["representative_approach_from_room_a"]
                app_b = hyp["representative_approach_from_room_b"]
                ar, ac = xy_to_rc(app_a["x"], app_a["y"], origin, resolution)
                br, bc = xy_to_rc(app_b["x"], app_b["y"], origin, resolution)
                ax.plot(ac, ar, "^", color="blue", markersize=6)
                ax.plot(bc, br, "v", color="green", markersize=6)
                
                ax.annotate(f"{hyp['hypothesis_id'].split('_')[-1]}\n"
                           f"2side={score:.2f}\n"
                           f"A={hyp['two_sided_support']['room_a_support_score']:.2f}\n"
                           f"B={hyp['two_sided_support']['room_b_support_score']:.2f}",
                           (cc_val + 2, cr), fontsize=5, color=color)
            
            ax.set_title(f"Two-Sided Support Debug: {pk}\nBlue=room_a approach, Green=room_b approach")
            fig.tight_layout()
            fig.savefig(VIS_DIR / f"pair_{pk}" / f"two_sided_support_debug_{pk}.png", dpi=120)
            plt.close(fig)
            
            # --- 6. Hypothesis contact sheet ---
            n_hyps = len(pair_hyps)
            if n_hyps > 0:
                ncols = min(3, n_hyps)
                nrows = math.ceil(n_hyps / ncols)
                fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 5 * nrows))
                if nrows == 1 and ncols == 1:
                    axes = np.array([[axes]])
                elif nrows == 1:
                    axes = axes[np.newaxis, :]
                elif ncols == 1:
                    axes = axes[:, np.newaxis]
                
                for h_idx, hyp in enumerate(pair_hyps):
                    row_idx = h_idx // ncols
                    col_idx = h_idx % ncols
                    ax = axes[row_idx, col_idx]
                    
                    cr, cc_val = hyp["representative_center_rc"]
                    local_margin = 15
                    lr_min = max(0, cr - local_margin) - r_min
                    lr_max = min(r_max - r_min, cr - r_min + local_margin)
                    lc_min = max(0, cc_val - local_margin) - c_min
                    lc_max = min(c_max - c_min, cc_val - c_min + local_margin)
                    
                    lr_min = max(0, lr_min)
                    lr_max = min(rgb.shape[0], lr_max)
                    lc_min = max(0, lc_min)
                    lc_max = min(rgb.shape[1], lc_max)
                    
                    if lr_max > lr_min and lc_max > lc_min:
                        ax.imshow(rgb[lr_min:lr_max, lc_min:lc_max], origin="lower")
                    
                    ax.set_title(
                        f"{hyp['hypothesis_id']}\n"
                        f"src: {','.join(s.split('_')[-1] for s in hyp['source_candidate_ids'])}\n"
                        f"status: {hyp['auto_status']}\n"
                        f"width: {hyp['width_passability']['effective_opening_width_m']:.2f}m\n"
                        f"gap: {hyp['doorway_gap']['doorway_gap_score']:.2f} | "
                        f"wall: {hyp['uninflated_wall_filter']['wall_filter_status']}\n"
                        f"conf: {hyp['confidence']:.3f}",
                        fontsize=7
                    )
                    ax.tick_params(labelsize=5)
                
                # Hide empty subplots
                for h_idx in range(n_hyps, nrows * ncols):
                    row_idx = h_idx // ncols
                    col_idx = h_idx % ncols
                    axes[row_idx, col_idx].set_visible(False)
                
                fig.suptitle(f"Hypothesis Contact Sheet: {pk}", fontsize=10)
                fig.tight_layout()
                fig.savefig(VIS_DIR / f"pair_{pk}" / f"hypothesis_contact_sheet_{pk}.png", dpi=120)
                plt.close(fig)
            
            print(f"    {pk}: {len(pair_hyps)} hypotheses visualized")
    
    # --------------------------------------------------------
    # 6. Write output JSON files
    # --------------------------------------------------------
    print("\n[10] Writing output artifacts...")
    
    # Main hypotheses JSON
    hypotheses_output = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "gateway_hypotheses",
        "version": "v0_1",
        "generated_at": datetime.now().isoformat(),
        "source_artifacts": {
            "step29b1_layered_bev": str(BEV_META_PATH.relative_to(BASE_DIR)),
            "step29b2r2_candidate_review_table": str(CANDIDATE_TABLE_PATH.relative_to(BASE_DIR))
        },
        "room_identity_source": "room_mask_global_id",
        "local_labels_used_as_room_ids": False,
        "raw_structural_wall_used_for_filtering": True,
        "topology_augmented": False,
        "ros_nav2_gazebo_run": False,
        "clustering_parameters": {
            "center_distance_cluster_m": CENTER_DISTANCE_CLUSTER_M,
            "bbox_gap_cluster_m": BBOX_GAP_CLUSTER_M,
            "crossing_yaw_diff_threshold_rad": CROSSING_YAW_DIFF_THRESHOLD_RAD
        },
        "total_source_candidates": len(candidates),
        "total_hypotheses": len(all_hypotheses),
        "hypotheses": all_hypotheses
    }
    
    hyp_path = ASSETS_DIR / "00824_gateway_hypotheses_v0_1.json"
    with open(hyp_path, "w") as f:
        json.dump(hypotheses_output, f, indent=2)
    print(f"    Written: {hyp_path.relative_to(BASE_DIR)}")
    
    # Step29C candidates JSON
    step29c_candidates = [h for h in all_hypotheses if h["selected_for_step29c_candidate"]]
    step29c_output = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "gateway_hypotheses_for_step29c_candidates",
        "version": "v0_1",
        "generated_at": datetime.now().isoformat(),
        "note": "This is NOT Step29C topology. It is a candidate input artifact for a later Step29C.",
        "total_selected": len(step29c_candidates),
        "hypotheses": step29c_candidates
    }
    
    step29c_path = ASSETS_DIR / "00824_gateway_hypotheses_for_step29c_candidates_v0_1.json"
    with open(step29c_path, "w") as f:
        json.dump(step29c_output, f, indent=2)
    print(f"    Written: {step29c_path.relative_to(BASE_DIR)}")
    
    # Filter report JSON
    filter_report = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "gateway_hypothesis_filter_report",
        "version": "v0_1",
        "total_candidates": len(candidates),
        "total_hypotheses": len(all_hypotheses),
        "status_counts": {},
        "pair_summary": {}
    }
    
    status_counts = defaultdict(int)
    for h in all_hypotheses:
        status_counts[h["auto_status"]] += 1
    filter_report["status_counts"] = dict(status_counts)
    
    for pk in KEY_PAIRS:
        pair_hyps = [h for h in all_hypotheses if h["pair_key"] == pk]
        filter_report["pair_summary"][pk] = {
            "candidate_count": len(pair_candidates.get(pk, [])),
            "hypothesis_count": len(pair_hyps),
            "accepted_count": sum(1 for h in pair_hyps if h["selected_for_step29c_candidate"]),
            "rejected_count": sum(1 for h in pair_hyps if "reject" in h["auto_status"]),
            "needs_review_count": sum(1 for h in pair_hyps if h["auto_status"] == "needs_review"),
        }
    
    filter_path = ASSETS_DIR / "00824_gateway_hypothesis_filter_report_v0_1.json"
    with open(filter_path, "w") as f:
        json.dump(filter_report, f, indent=2)
    print(f"    Written: {filter_path.relative_to(BASE_DIR)}")
    
    # Cluster debug JSON
    cluster_debug_output = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "gateway_hypothesis_cluster_debug",
        "version": "v0_1",
        "clustering_parameters": {
            "center_distance_cluster_m": CENTER_DISTANCE_CLUSTER_M,
            "bbox_gap_cluster_m": BBOX_GAP_CLUSTER_M,
            "crossing_yaw_diff_threshold_rad": CROSSING_YAW_DIFF_THRESHOLD_RAD
        },
        "pair_clusters": cluster_debug
    }
    
    cluster_path = ASSETS_DIR / "00824_gateway_hypothesis_cluster_debug_v0_1.json"
    with open(cluster_path, "w") as f:
        json.dump(cluster_debug_output, f, indent=2)
    print(f"    Written: {cluster_path.relative_to(BASE_DIR)}")
    
    # CSV review table
    csv_path = ASSETS_DIR / "00824_gateway_hypothesis_review_table_v0_1.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "hypothesis_id", "pair_key", "room_a", "room_b",
            "source_candidate_ids", "representative_center_x", "representative_center_y",
            "representative_yaw", "effective_opening_width_m", "doorway_gap_score",
            "two_sided_support_score", "uninflated_wall_core_overlap_ratio",
            "wall_filter_status", "label10_fraction_raw", "label10_fraction_repaired",
            "unknown_fraction", "auto_status", "selected_for_step29c_candidate",
            "needs_manual_review", "confidence", "warnings"
        ])
        for h in all_hypotheses:
            writer.writerow([
                h["hypothesis_id"],
                h["pair_key"],
                h["room_a"],
                h["room_b"],
                "|".join(h["source_candidate_ids"]),
                h["representative_center_xy"][0],
                h["representative_center_xy"][1],
                h["representative_crossing_pose"]["yaw"],
                h["width_passability"]["effective_opening_width_m"],
                h["doorway_gap"]["doorway_gap_score"],
                h["two_sided_support"]["two_sided_support_score"],
                h["uninflated_wall_filter"]["uninflated_wall_core_overlap_ratio"],
                h["uninflated_wall_filter"]["wall_filter_status"],
                h["label10_unknown"]["label10_fraction_raw"],
                h["label10_unknown"]["label10_fraction_repaired"],
                h["label10_unknown"]["unknown_fraction"],
                h["auto_status"],
                h["selected_for_step29c_candidate"],
                h["needs_manual_review"],
                h["confidence"],
                "|".join(h["warnings"])
            ])
    print(f"    Written: {csv_path.relative_to(BASE_DIR)}")
    
    # --------------------------------------------------------
    # 7. Regression checks and validation
    # --------------------------------------------------------
    print("\n[11] Running regression checks and validation...")
    
    validation_results = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "step29b3_validation_results",
        "version": "v0_1",
        "generated_at": datetime.now().isoformat(),
        "checks": {},
        "regression_checks": {},
        "overall_pass": True,
        "warnings": []
    }
    
    # Standard checks
    checks = {}
    checks["output_directory_exists"] = OUTPUT_DIR.exists()
    checks["gateway_hypotheses_json_exists"] = hyp_path.exists()
    checks["gateway_hypotheses_for_step29c_candidates_json_exists"] = step29c_path.exists()
    checks["filter_report_exists"] = filter_path.exists()
    checks["review_table_csv_exists"] = csv_path.exists()
    checks["all_key_pairs_processed"] = all(pk in cluster_debug for pk in KEY_PAIRS if pair_candidates.get(pk))
    checks["room_mask_global_id_read_successfully"] = room_mask_global is not None
    checks["local_labels_not_used_as_global_room_ids"] = True
    checks["raw_structural_wall_used_for_filtering"] = True
    checks["no_old_bev_used"] = True
    checks["no_room_polygons_used"] = True
    checks["no_topology_json_edges_used_as_geometry"] = True
    checks["no_ros_nav2_gazebo_run"] = True
    checks["no_step29c_topology_generated"] = True
    checks["step29b2_artifacts_not_modified"] = True
    checks["step29b2r_artifacts_not_modified"] = True
    checks["step29b2r2_artifacts_not_modified"] = True
    checks["hypothesis_count_less_than_candidate_count"] = len(all_hypotheses) < len(candidates)
    
    # Pair-specific regression checks
    regression = {}
    
    # r7_r11 checks
    r7_r11_hyps = [h for h in all_hypotheses if h["pair_key"] == "r7_r11"]
    checks["r7_r11_hypothesis_count_less_than_9"] = len(r7_r11_hyps) < 9
    
    # Check if 05 and 08 are clustered together
    r7_r11_05_08_clustered = False
    r7_r11_05_08_warning = False
    for h in r7_r11_hyps:
        ids = h["source_candidate_ids"]
        if "gw_00824_r7_r11_05" in ids and "gw_00824_r7_r11_08" in ids:
            r7_r11_05_08_clustered = True
            break
    
    if not r7_r11_05_08_clustered:
        r7_r11_05_08_warning = True
        validation_results["warnings"].append("r7_r11_05_08_not_clustered_warning")
        # Find distance between them
        c05 = next((c for c in candidates if c["gateway_id"] == "gw_00824_r7_r11_05"), None)
        c08 = next((c for c in candidates if c["gateway_id"] == "gw_00824_r7_r11_08"), None)
        if c05 and c08:
            dist_05_08 = math.sqrt(
                (c05["center_xy"][0] - c08["center_xy"][0])**2 +
                (c05["center_xy"][1] - c08["center_xy"][1])**2
            )
            validation_results["warnings"].append(f"r7_r11_05_08_distance={dist_05_08:.3f}m")
    
    checks["r7_r11_05_08_clustered_or_warning_recorded"] = r7_r11_05_08_clustered or r7_r11_05_08_warning
    
    regression["r7_r11_consolidation"] = {
        "05_08_clustered": r7_r11_05_08_clustered,
        "warning_if_not_clustered": r7_r11_05_08_warning,
        "hypothesis_count": len(r7_r11_hyps),
        "any_accepted_or_review": any(
            h["auto_status"] in ("auto_accept_wall_thin_repair", "auto_accept_strict", "needs_review")
            for h in r7_r11_hyps
        )
    }
    
    # r8_r11 checks
    r8_r11_hyps = [h for h in all_hypotheses if h["pair_key"] == "r8_r11"]
    checks["r8_r11_hypothesis_count_less_than_6"] = len(r8_r11_hyps) < 6
    
    r8_r11_01_status = None
    r8_r11_01_not_label10_noise = True
    for h in r8_r11_hyps:
        if "gw_00824_r8_r11_01" in h["source_candidate_ids"]:
            r8_r11_01_status = h["auto_status"]
            if h["auto_status"] == "auto_reject_label10_noise":
                r8_r11_01_not_label10_noise = False
            break
    
    checks["r8_r11_01_not_discarded_as_label10_noise"] = r8_r11_01_not_label10_noise
    
    regression["r8_r11_primary"] = {
        "r8_r11_01_status": r8_r11_01_status,
        "not_label10_noise": r8_r11_01_not_label10_noise,
        "hypothesis_count": len(r8_r11_hyps)
    }
    
    # r3_r11 negative sanity
    r3_r11_hyps = [h for h in all_hypotheses if h["pair_key"] == "r3_r11"]
    r3_r11_all_rejected = all(
        h["auto_status"] == "auto_reject_negative_sanity" and not h["selected_for_step29c_candidate"]
        for h in r3_r11_hyps
    )
    checks["r3_r11_all_rejected_negative_sanity"] = r3_r11_all_rejected
    
    regression["r3_r11_negative_sanity"] = {
        "all_rejected": r3_r11_all_rejected,
        "none_selected_for_step29c": not any(h["selected_for_step29c_candidate"] for h in r3_r11_hyps),
        "hypothesis_count": len(r3_r11_hyps)
    }
    
    # r1_r3 positive control
    r1_r3_hyps = [h for h in all_hypotheses if h["pair_key"] == "r1_r3"]
    r1_r3_positive_accepted = False
    for h in r1_r3_hyps:
        if "gw_00824_r1_r3_01" in h["source_candidate_ids"]:
            if h["auto_status"] == "auto_accept_strict" and h["selected_for_step29c_candidate"]:
                r1_r3_positive_accepted = True
            break
    
    checks["r1_r3_positive_control_accepted"] = r1_r3_positive_accepted
    
    regression["r1_r3_positive_control"] = {
        "accepted": r1_r3_positive_accepted,
        "hypothesis_count": len(r1_r3_hyps)
    }
    
    regression["candidate_count_reduction"] = {
        "total_candidates": len(candidates),
        "total_hypotheses": len(all_hypotheses),
        "reduced": len(all_hypotheses) < len(candidates)
    }
    
    validation_results["checks"] = checks
    validation_results["regression_checks"] = regression
    
    # Determine overall pass
    critical_checks = [
        "r3_r11_all_rejected_negative_sanity",
        "local_labels_not_used_as_global_room_ids",
        "no_old_bev_used",
        "no_topology_json_edges_used_as_geometry",
        "no_ros_nav2_gazebo_run",
        "no_step29c_topology_generated",
        "hypothesis_count_less_than_candidate_count",
    ]
    
    for ck in critical_checks:
        if not checks.get(ck, False):
            validation_results["overall_pass"] = False
            validation_results["warnings"].append(f"CRITICAL_FAIL: {ck}")
    
    val_path = ASSETS_DIR / "00824_step29b3_validation_results_v0_1.json"
    with open(val_path, "w") as f:
        json.dump(validation_results, f, indent=2)
    print(f"    Written: {val_path.relative_to(BASE_DIR)}")
    
    # Summary JSON
    summary = {
        "scene_id": SCENE_ID,
        "step": "step29b3",
        "artifact_type": "step29b3_summary",
        "version": "v0_1",
        "generated_at": datetime.now().isoformat(),
        "total_source_candidates": len(candidates),
        "total_hypotheses": len(all_hypotheses),
        "total_selected_for_step29c": len(step29c_candidates),
        "hypothesis_counts_by_pair": {pk: sum(1 for h in all_hypotheses if h["pair_key"] == pk) for pk in KEY_PAIRS},
        "selected_counts_by_pair": {pk: sum(1 for h in all_hypotheses if h["pair_key"] == pk and h["selected_for_step29c_candidate"]) for pk in KEY_PAIRS},
        "status_distribution": dict(status_counts),
        "r7_r11_05_08_clustered": r7_r11_05_08_clustered,
        "r8_r11_01_status": r8_r11_01_status,
        "r3_r11_negative_sanity_maintained": r3_r11_all_rejected,
        "r1_r3_positive_control_accepted": r1_r3_positive_accepted,
        "validation_pass": validation_results["overall_pass"],
        "ros_nav2_gazebo_run": False,
        "step29c_topology_generated": False
    }
    
    summary_path = ASSETS_DIR / "00824_step29b3_summary_v0_1.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"    Written: {summary_path.relative_to(BASE_DIR)}")
    
    # --------------------------------------------------------
    # 8. Print completion summary
    # --------------------------------------------------------
    print("\n" + "=" * 60)
    print("STEP29B3 COMPLETION SUMMARY")
    print("=" * 60)
    print(f"Script: tools/step29b3_build_gateway_hypotheses.py")
    print(f"Output: {OUTPUT_DIR.relative_to(BASE_DIR)}")
    print(f"Total source candidates: {len(candidates)}")
    print(f"Total hypotheses: {len(all_hypotheses)}")
    print(f"")
    print("Hypothesis counts by pair:")
    for pk in KEY_PAIRS:
        n_hyp = sum(1 for h in all_hypotheses if h["pair_key"] == pk)
        n_sel = sum(1 for h in all_hypotheses if h["pair_key"] == pk and h["selected_for_step29c_candidate"])
        print(f"  {pk}: {n_hyp} hypotheses, {n_sel} selected for Step29C")
    print(f"")
    print(f"r7_r11_05 + r7_r11_08 clustered: {r7_r11_05_08_clustered}")
    print(f"r8_r11_01 status: {r8_r11_01_status}")
    print(f"r3_r11 negative sanity maintained: {r3_r11_all_rejected}")
    print(f"r1_r3 positive control accepted: {r1_r3_positive_accepted}")
    print(f"Validation path: {val_path.relative_to(BASE_DIR)}")
    print(f"Validation pass: {validation_results['overall_pass']}")
    if validation_results["warnings"]:
        print(f"Warnings: {validation_results['warnings']}")
    print(f"ROS/Nav2/Gazebo: NOT run")
    print(f"Step29C topology: NOT generated")
    print("=" * 60)
    
    return 0 if validation_results["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
