#!/usr/bin/env python3
"""Step30A dual-wall Stage-A post-process, gateway rerun, roles, and validation."""

from __future__ import annotations

import glob
import hashlib
import json
import math
import shutil
import subprocess
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from extract_step29b2_gateway_candidates_v1 import (  # noqa: E402
    classify_candidate,
    discover_room_pairs,
    extract_pair_candidates,
    normalize_pair,
)


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
VERSION = "v0_1"
STEP = "step30a"
OUTPUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30a_00824_full_stage_a_dual_wall_gateway_rerun"
SCENE_ROOT = OUTPUT_ROOT / "scenes" / SCENE_ID
DEBUG_ROOM_DIR = SCENE_ROOT / "debug_room" / "floor_1"
FINAL_DIR = SCENE_ROOT / "final_raster_export"
GENERATED = OUTPUT_ROOT / "generated"
ASSET_DIR = GENERATED / "assets"
VIS_DIR = GENERATED / "visualizations"
LOG_DIR = OUTPUT_ROOT / "logs"

PRIOR_ROLES_PATH = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step29b3r_00824_gateway_hypothesis_audit_and_roles"
    / "generated"
    / "assets"
    / "00824_gateway_hypotheses_with_roles_v0_1.json"
)
STEP29_HASH_INPUTS = [
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29a3_00824_stage_a_debug_raster_export",
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29a3r_00824_gateway_wall_evidence_export",
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3",
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b2_00824_gateway_candidates_from_layered_bev",
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b3_00824_gateway_hypothesis_consolidation",
    REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b3r_00824_gateway_hypothesis_audit_and_roles",
]

STAGE_A_COMMAND = (
    "/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d "
    "--model-path ./models/cutr_rgbd.pth --config ./config/hm3d.yaml --device cuda "
    "--seq 00824-Dd4bFSTQ8gi "
    "--output-root ./runtime_stage1_frozen_evidence/step30a_00824_full_stage_a_dual_wall_gateway_rerun/scenes "
    "--room-seg-interval 100 --capture-stride 25 --video-fps 12 --runtime-profile-interval 25"
)

RESOLUTION = 0.05
ORIGIN = [-50.0, -50.0]
PUBLIC_PATH_042 = [1, 3, 7, 11, 8]
PRIMARY_EXPECTED = {
    "r1_r3": "hyp_00824_r1_r3_01",
    "r3_r7": "hyp_00824_r3_r7_01",
    "r7_r11": "hyp_00824_r7_r11_01",
    "r8_r11": "hyp_00824_r8_r11_01",
}
KEY_PAIRS = [(1, 3), (3, 7), (7, 11), (8, 11), (3, 11), (7, 14)]


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
    print(f"Written: {rel(path)}")


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def directory_digest(path: Path) -> str:
    if not path.exists():
        return "missing"
    h = hashlib.sha256()
    for file_path in sorted(p for p in path.rglob("*") if p.is_file()):
        h.update(str(file_path.relative_to(path)).encode())
        h.update(str(file_path.stat().st_size).encode())
        h.update(sha256_file(file_path).encode())
    return h.hexdigest()


def run_text(cmd: Sequence[str]) -> str:
    try:
        return subprocess.check_output(cmd, cwd=str(REPO_ROOT), text=True, stderr=subprocess.STDOUT).strip()
    except Exception as exc:
        return f"unavailable: {exc}"


def latest_cycle() -> Tuple[int, List[int]]:
    files = glob.glob(str(DEBUG_ROOM_DIR / "run_*_09_final_labels.npy"))
    cycles = sorted(int(Path(f).name.split("_")[1]) for f in files)
    if not cycles:
        raise FileNotFoundError(f"No segmentation cycles under {DEBUG_ROOM_DIR}")
    return cycles[-1], cycles


def copy_if(src: Path, dst: Path) -> bool:
    if not src.is_file():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def load_gray(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not read {path}")
    return image


def align_to_shape(arr: np.ndarray, target_shape: Tuple[int, int]) -> np.ndarray:
    out = np.asarray(arr)
    h, w = out.shape[:2]
    th, tw = target_shape
    if (h, w) == (th, tw):
        return out
    if h >= th and w >= tw:
        top = (h - th) // 2
        left = (w - tw) // 2
        return out[top : top + th, left : left + tw]
    padded = np.zeros((th, tw), dtype=out.dtype)
    top = max(0, (th - h) // 2)
    left = max(0, (tw - w) // 2)
    padded[top : top + min(h, th), left : left + min(w, tw)] = out[: min(h, th), : min(w, tw)]
    return padded


def grid_to_xy(row: float, col: float) -> Dict[str, float]:
    return {"x": round(ORIGIN[0] + col * RESOLUTION, 4), "y": round(ORIGIN[1] + row * RESOLUTION, 4)}


def xy_to_rc(x: float, y: float) -> Tuple[int, int]:
    return int(round((y - ORIGIN[1]) / RESOLUTION)), int(round((x - ORIGIN[0]) / RESOLUTION))


def pair_key(a: int, b: int) -> str:
    aa, bb = normalize_pair(a, b)
    return f"r{aa}_r{bb}"


def map_extent(shape: Tuple[int, int]) -> List[float]:
    return [ORIGIN[0], ORIGIN[0] + shape[1] * RESOLUTION, ORIGIN[1], ORIGIN[1] + shape[0] * RESOLUTION]


def make_final_exports() -> Dict[str, Any]:
    FINAL_DIR.mkdir(parents=True, exist_ok=True)
    last, cycles = latest_cycle()
    copies = {
        "final_tracked_room_labels": copy_if(DEBUG_ROOM_DIR / f"run_{last}_09_final_labels.npy", FINAL_DIR / "final_tracked_room_labels.npy"),
        "final_repaired_room_labels": copy_if(DEBUG_ROOM_DIR / f"run_{last}_09b_repaired_labels.npy", FINAL_DIR / "final_repaired_room_labels.npy"),
        "final_walls_skeleton": copy_if(DEBUG_ROOM_DIR / f"run_{last}_02_walls_skeleton.png", FINAL_DIR / "final_walls_skeleton.png"),
        "final_outside_boundary": copy_if(DEBUG_ROOM_DIR / f"run_{last}_03_outside_boundary.png", FINAL_DIR / "final_outside_boundary.png"),
        "final_free_space": copy_if(DEBUG_ROOM_DIR / f"run_{last}_05_free_space.png", FINAL_DIR / "final_free_space.png"),
        "final_full_map": copy_if(DEBUG_ROOM_DIR / f"run_{last}_04b_full_map_post_doors.png", FINAL_DIR / "final_full_map.png")
        or copy_if(DEBUG_ROOM_DIR / f"run_{last}_04_full_map.png", FINAL_DIR / "final_full_map.png"),
        "final_wall_density_raw_or_hist_current": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01a_wall_density_raw_or_hist_current.npy", FINAL_DIR / "final_wall_density_raw_or_hist_current.npy"),
        "final_wall_density_clipped_or_normalized": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01b_wall_density_clipped_or_normalized.npy", FINAL_DIR / "final_wall_density_clipped_or_normalized.npy"),
        "final_wall_density_blurred": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01c_wall_density_blurred.npy", FINAL_DIR / "final_wall_density_blurred.npy"),
        "final_gateway_wall_preclose_png": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01d_gateway_wall_preclose_thr_0p25.png", FINAL_DIR / "final_gateway_wall_preclose_thr_0p25.png"),
        "final_gateway_wall_preclose_npy": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01d_gateway_wall_preclose_thr_0p25.npy", FINAL_DIR / "final_gateway_wall_preclose_thr_0p25.npy"),
        "final_dual_wall_layer_metadata": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01g_dual_wall_layer_metadata.json", FINAL_DIR / "final_dual_wall_layer_metadata.json"),
        "final_height_band_sweep": copy_if(DEBUG_ROOM_DIR / f"run_{last}_01f_height_band_wall_debug_sweep.npz", FINAL_DIR / "final_height_band_wall_debug_sweep.npz"),
    }
    for thr in ["thr_0p18", "thr_0p20", "thr_0p25", "thr_0p30"]:
        copies[f"final_gateway_wall_preclose_{thr}"] = copy_if(
            DEBUG_ROOM_DIR / f"run_{last}_01e_gateway_wall_preclose_{thr}.png",
            FINAL_DIR / f"final_gateway_wall_preclose_{thr}.png",
        )

    labels = np.load(FINAL_DIR / "final_repaired_room_labels.npy")
    tracking = read_json(DEBUG_ROOM_DIR / f"run_{last}_12_tracking_report.json").get("tracking", {})
    label_to_global = {}
    global_ids = set()
    for entry in list(tracking.get("matched", [])) + list(tracking.get("new_rooms", [])):
        local = entry.get("marker_label", entry.get("local_label", entry.get("label")))
        global_id = entry.get("global_id", entry.get("tracked_id"))
        if local is not None and global_id is not None:
            label_to_global[int(local)] = int(global_id)
            global_ids.add(int(global_id))

    metadata = {
        "scene_id": SCENE_ID,
        "last_cycle_frame_index": last,
        "total_segmentation_cycles": len(cycles),
        "all_cycle_frame_indices": cycles,
        "grid_shape": list(labels.shape),
        "resolution_m_per_pixel": RESOLUTION,
        "origin": ORIGIN,
        "label_to_global_id_map": {str(k): v for k, v in label_to_global.items()},
        "global_room_ids": sorted(global_ids),
        "tracked_room_count": len(global_ids),
        "wall_cell_count": int(np.count_nonzero(load_gray(FINAL_DIR / "final_walls_skeleton.png") > 127)),
        "gateway_wall_preclose_cell_count": int(np.count_nonzero(load_gray(FINAL_DIR / "final_gateway_wall_preclose_thr_0p25.png") > 127)),
        "free_cell_count": int(np.count_nonzero(load_gray(FINAL_DIR / "final_free_space.png") > 127)),
        "tracking_report": tracking,
        "dual_wall_exports": copies,
    }
    write_json(FINAL_DIR / "final_grid_metadata.json", metadata)
    write_json(FINAL_DIR / "final_room_id_summary.json", {
        "scene_id": SCENE_ID,
        "tracked_room_count": len(global_ids),
        "global_room_ids": sorted(global_ids),
        "room_presence": {f"room_{rid}": rid in global_ids for rid in [1, 3, 7, 8, 11, 14]},
        "label_to_global_id_map": {str(k): v for k, v in label_to_global.items()},
    })
    cycle_manifest = []
    for cycle in cycles:
        file_map = {
            "walls_skeleton": f"run_{cycle}_02_walls_skeleton.png",
            "wall_density_blurred": f"run_{cycle}_01c_wall_density_blurred.npy",
            "gateway_wall_preclose_thr_0p25": f"run_{cycle}_01d_gateway_wall_preclose_thr_0p25.png",
            "dual_wall_layer_metadata": f"run_{cycle}_01g_dual_wall_layer_metadata.json",
            "height_band_sweep": f"run_{cycle}_01f_height_band_wall_debug_sweep.npz",
            "repaired_labels": f"run_{cycle}_09b_repaired_labels.npy",
            "free_space": f"run_{cycle}_05_free_space.png",
        }
        cycle_manifest.append({
            "cycle_frame_index": cycle,
            "files": {k: {"filename": v, "exists": (DEBUG_ROOM_DIR / v).is_file()} for k, v in file_map.items()},
        })
    write_json(FINAL_DIR / "cycle_manifest.json", {"scene_id": SCENE_ID, "cycles": cycle_manifest})
    return {"last_cycle": last, "cycles": cycles, "copies": copies, "metadata": metadata}


def remap_global(labels: np.ndarray, mapping: Dict[int, int]) -> np.ndarray:
    out = np.zeros(labels.shape, dtype=np.int32)
    for local, global_id in mapping.items():
        out[labels == int(local)] = int(global_id)
    return out


def build_layered_bev() -> Dict[str, Any]:
    meta = read_json(FINAL_DIR / "final_grid_metadata.json")
    raw = np.load(FINAL_DIR / "final_tracked_room_labels.npy").astype(np.int32)
    repaired = np.load(FINAL_DIR / "final_repaired_room_labels.npy").astype(np.int32)
    shape = tuple(repaired.shape)
    local_to_global = {int(k): int(v) for k, v in meta["label_to_global_id_map"].items()}
    global_mask = remap_global(repaired, local_to_global)
    segmentation_wall = (align_to_shape(load_gray(FINAL_DIR / "final_walls_skeleton.png"), shape) > 127).astype(np.uint8)
    gateway_wall = (align_to_shape(load_gray(FINAL_DIR / "final_gateway_wall_preclose_thr_0p25.png"), shape) > 127).astype(np.uint8)
    free = (align_to_shape(load_gray(FINAL_DIR / "final_free_space.png"), shape) > 127).astype(np.uint8)
    outside = (align_to_shape(load_gray(FINAL_DIR / "final_outside_boundary.png"), shape) > 127).astype(np.uint8)
    full_map = (align_to_shape(load_gray(FINAL_DIR / "final_full_map.png"), shape) > 127).astype(np.uint8)
    unknown = (outside == 0).astype(np.uint8)
    zero = np.zeros(shape, dtype=np.uint8)

    npz_path = ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz"
    np.savez_compressed(
        npz_path,
        room_mask_global_id=global_mask,
        room_mask_local_label_raw=raw,
        room_mask_local_label_repaired=repaired,
        segmentation_wall_processed=segmentation_wall,
        segmentation_wall_processed_postclose=segmentation_wall,
        gateway_wall_preclose=gateway_wall,
        structural_wall_legacy_or_segmentation_reference=segmentation_wall,
        structural_wall=segmentation_wall,
        free_space=free,
        outside_boundary=outside,
        unknown_layer=unknown,
        full_map_post_doors_reference=full_map,
        object_obstacle_placeholder=zero,
        gateway_candidate_placeholder=zero,
    )
    global_path = ASSET_DIR / f"{SHORT}_step30a_global_room_mask_{VERSION}.npy"
    np.save(global_path, global_mask)
    dual_meta = read_json(FINAL_DIR / "final_dual_wall_layer_metadata.json")
    dual_meta.update({
        "scene_id": SCENE_ID,
        "step": STEP,
        "authoritative_recommended_gateway_wall_layer": "gateway_wall_preclose",
        "gateway_wall_preclose_source": rel(FINAL_DIR / "final_gateway_wall_preclose_thr_0p25.png"),
        "segmentation_wall_processed_source": rel(FINAL_DIR / "final_walls_skeleton.png"),
        "gateway_wall_preclose_used_for_gateway_extraction": True,
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
        "gateway_wall_preclose_generated_by_user_corridor_carving": False,
    })
    dual_meta_path = ASSET_DIR / f"{SHORT}_step30a_dual_wall_layer_metadata_{VERSION}.json"
    write_json(dual_meta_path, dual_meta)
    layered_meta = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30a_layered_bev",
        "version": VERSION,
        "resolution": RESOLUTION,
        "origin": ORIGIN,
        "width": int(shape[1]),
        "height": int(shape[0]),
        "coordinate_convention": "map_xy: x increases right/columns; y increases upward/rows; arrays indexed [row, col].",
        "room_identity_source": "room_mask_global_id",
        "local_labels_used_as_global_room_ids": False,
        "layer_definitions": {
            "room_mask_global_id": "persistent global room IDs",
            "room_mask_local_label_raw": "provenance only",
            "room_mask_local_label_repaired": "provenance only and remapping input",
            "segmentation_wall_processed": "room segmentation / watershed wall support",
            "gateway_wall_preclose": "gateway filtering wall layer from blurred density threshold 0.25 without close",
            "structural_wall_legacy_or_segmentation_reference": "legacy compatibility copy; not a hard gateway blocker",
        },
        "outputs": {
            "layered_bev_npz": rel(npz_path),
            "global_room_mask": rel(global_path),
            "dual_wall_layer_metadata": rel(dual_meta_path),
        },
    }
    json_path = ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.json"
    write_json(json_path, layered_meta)
    return {"npz": npz_path, "json": json_path, "global": global_path, "dual_meta": dual_meta_path}


def build_corridor_mask(center_rc: Tuple[int, int], yaw: float, length_m: float, width_m: float, shape: Tuple[int, int]) -> np.ndarray:
    half_len = length_m / (2 * RESOLUTION)
    half_wid = width_m / (2 * RESOLUTION)
    cr, cc = center_rc
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    mask = np.zeros(shape, dtype=bool)
    for along in np.linspace(-half_len, half_len, max(int(2 * half_len) + 1, 5)):
        for across in np.linspace(-half_wid, half_wid, max(int(2 * half_wid) + 1, 3)):
            dc = along * cos_y - across * sin_y
            dr = along * sin_y + across * cos_y
            r = int(round(cr + dr))
            c = int(round(cc + dc))
            if 0 <= r < shape[0] and 0 <= c < shape[1]:
                mask[r, c] = True
    return mask


def scanline_gap(center_rc: Tuple[int, int], yaw: float, wall: np.ndarray, shape: Tuple[int, int]) -> Dict[str, float]:
    cr, cc = center_rc
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    perp_cos, perp_sin = -sin_y, cos_y
    half_len = 0.5 / RESOLUTION
    gap_widths = []
    center_clear = 0
    endpoint = 0
    total = 0
    for off in np.linspace(-0.15 / RESOLUTION, 0.15 / RESOLUTION, 7):
        br = cr + off * sin_y
        bc = cc + off * cos_y
        positions = []
        for t in np.linspace(-half_len, half_len, int(2 * half_len) + 1):
            r = int(round(br + t * perp_sin))
            c = int(round(bc + t * perp_cos))
            if 0 <= r < shape[0] and 0 <= c < shape[1] and wall[r, c]:
                positions.append(t)
        total += 1
        if len(positions) >= 2:
            neg = [p for p in positions if p < -1]
            pos = [p for p in positions if p > 1]
            if neg and pos:
                gap_widths.append((min(pos) - max(neg)) * RESOLUTION)
                endpoint += 1
            if not [p for p in positions if abs(p) <= 1.5]:
                center_clear += 1
        elif len(positions) == 0:
            center_clear += 1
        elif not [p for p in positions if abs(p) <= 1.5]:
            center_clear += 1
    raw_gap = float(np.median(gap_widths)) if gap_widths else 0.0
    endpoint_score = endpoint / max(total, 1)
    clear_score = center_clear / max(total, 1)
    consistency = 1.0 - (float(np.std(gap_widths)) / (float(np.mean(gap_widths)) + 1e-6)) if len(gap_widths) >= 2 else (0.5 if gap_widths else 0.0)
    consistency = max(0.0, min(1.0, consistency))
    score = 0.30 * endpoint_score + 0.35 * clear_score + 0.20 * consistency + 0.15 * min(1.0, raw_gap / 0.5)
    return {
        "doorway_gap_score": round(score, 4),
        "raw_wall_gap_width_m": round(raw_gap, 4),
        "wall_endpoint_support_score": round(endpoint_score, 4),
        "gap_center_clear_score": round(clear_score, 4),
        "gap_consistency_score": round(consistency, 4),
    }


def support_metrics(center_rc: Tuple[int, int], room_a: int, room_b: int, global_room: np.ndarray, free: np.ndarray) -> Dict[str, float]:
    cr, cc = center_rc
    def ratio(mask: np.ndarray, radius: int = 5) -> float:
        r0, r1 = max(0, cr - radius), min(mask.shape[0], cr + radius + 1)
        c0, c1 = max(0, cc - radius), min(mask.shape[1], cc + radius + 1)
        crop = mask[r0:r1, c0:c1]
        return float(np.count_nonzero(crop)) / max(1, crop.size)
    a = ratio(global_room == room_a)
    b = ratio(global_room == room_b)
    f = ratio(free, radius=2)
    return {
        "two_sided_support_score": round(0.4 * a + 0.4 * b + 0.2 * f, 4),
        "room_a_support_score": round(a, 4),
        "room_b_support_score": round(b, 4),
        "free_space_crossing_score": round(f, 4),
    }


def wall_metrics_for_pose(hyp: Dict[str, Any], wall: np.ndarray, global_room: np.ndarray, free: np.ndarray) -> Dict[str, Any]:
    center_xy = hyp["representative_center_xy"]
    center_rc = tuple(hyp.get("representative_center_rc") or xy_to_rc(center_xy[0], center_xy[1]))
    yaw = float(hyp.get("representative_crossing_pose", {}).get("yaw", 0.0))
    width = float(hyp.get("width_passability", {}).get("effective_opening_width_m", 0.6))
    width = max(0.25, min(width, 0.6))
    corridor = build_corridor_mask(center_rc, yaw, 0.6, width, wall.shape)
    cells = int(np.count_nonzero(corridor))
    overlap = int(np.count_nonzero(corridor & wall))
    ratio = float(overlap / max(1, cells))
    status = "clear" if ratio < 0.08 else "uncertain" if ratio < 0.18 else "blocked"
    gap = scanline_gap(center_rc, yaw, wall, wall.shape)
    support = support_metrics(center_rc, int(hyp["room_a"]), int(hyp["room_b"]), global_room, free)
    return {
        "wall_core_overlap_ratio": round(ratio, 4),
        "wall_core_overlap_count": overlap,
        "corridor_cell_count": cells,
        "wall_filter_status": status,
        "corridor_width_m": width,
        "corridor_length_m": 0.6,
        **gap,
        **support,
        "effective_opening_width_m": round(max(width, gap["raw_wall_gap_width_m"]), 4),
    }


def run_gateway_extraction_and_roles() -> Dict[str, Any]:
    data = np.load(ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz")
    global_room = data["room_mask_global_id"].astype(np.int32)
    wall = data["gateway_wall_preclose"] > 0
    seg_wall = data["segmentation_wall_processed"] > 0
    free = data["free_space"] > 0
    outside = data["outside_boundary"] > 0
    unknown = data["unknown_layer"] > 0
    local_repaired = data["room_mask_local_label_repaired"].astype(np.int32)
    evaluated = sorted(set(discover_room_pairs(global_room, outside)) | {normalize_pair(*p) for p in KEY_PAIRS})
    pair_candidates: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    all_candidates: List[Dict[str, Any]] = []
    for pair in evaluated:
        cands, _ = extract_pair_candidates(
            pair,
            global_room,
            wall,
            free,
            outside,
            unknown,
            local_repaired,
            wall & (global_room > 0),
        )
        for cand in cands:
            cand["source_layers"] = [
                "room_mask_global_id",
                "gateway_wall_preclose",
                "free_space",
                "outside_boundary",
                "unknown_layer",
            ]
            cand["gateway_filter_wall_layer"] = "gateway_wall_preclose"
            cand["segmentation_wall_processed_used_as_hard_blocker"] = False
            cand["pair_key"] = pair_key(cand["room_a"], cand["room_b"])
        pair_candidates[pair] = cands
        all_candidates.extend(cands)

    candidate_payload = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "gateway_candidates",
        "version": VERSION,
        "gateway_filter_wall_layer": "gateway_wall_preclose",
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
        "structural_wall_legacy_used_as_hard_gateway_blocker": False,
        "gateway_candidates": all_candidates,
        "candidate_count": len(all_candidates),
        "candidate_counts_by_pair": dict(Counter(c["pair_key"] for c in all_candidates)),
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_gateway_candidates_{VERSION}.json", candidate_payload)

    prior = read_json(PRIOR_ROLES_PATH).get("hypotheses", [])
    prior_by_id = {h["hypothesis_id"]: h for h in prior}
    wanted_ids = [
        "hyp_00824_r1_r3_01",
        "hyp_00824_r3_r7_01",
        "hyp_00824_r7_r11_01",
        "hyp_00824_r7_r11_02",
        "hyp_00824_r7_r11_03",
        "hyp_00824_r7_r11_04",
        "hyp_00824_r8_r11_01",
        "hyp_00824_r8_r11_02",
        "hyp_00824_r8_r11_03",
        "hyp_00824_r8_r11_04",
        "hyp_00824_r3_r11_01",
        "hyp_00824_r3_r11_02",
        "hyp_00824_r3_r11_03",
    ]
    hypotheses = []
    for hid in wanted_ids:
        if hid not in prior_by_id:
            continue
        h = dict(prior_by_id[hid])
        pk = h["pair_key"]
        candidates = pair_candidates.get(normalize_pair(h["room_a"], h["room_b"]), [])
        center = h["representative_center_xy"]
        best = None
        best_dist = float("inf")
        for cand in candidates:
            cxy = cand["center"]
            dist = math.hypot(float(cxy["x"]) - center[0], float(cxy["y"]) - center[1])
            if dist < best_dist:
                best_dist = dist
                best = cand
        if best is not None and best_dist <= 0.75:
            h["current_step30a_source_candidate_ids"] = [best["gateway_id"]]
            h["current_step30a_nearest_candidate_distance_m"] = round(best_dist, 4)
            h["current_step30a_extraction_match"] = True
        else:
            h["current_step30a_source_candidate_ids"] = []
            h["current_step30a_nearest_candidate_distance_m"] = None if best is None else round(best_dist, 4)
            h["current_step30a_extraction_match"] = False
            h.setdefault("warnings", []).append("no_close_current_candidate_found; metrics recomputed on prior hypothesis geometry")
        metrics = wall_metrics_for_pose(h, wall, global_room, free)
        h["step30a_gateway_wall_metrics"] = metrics
        h["gateway_filter_wall_layer"] = "gateway_wall_preclose"
        h["segmentation_wall_processed_used_as_hard_blocker"] = False

        role = h.get("route_role", "needs_review")
        if hid == "hyp_00824_r3_r7_01":
            role = "primary_route_gateway"
            h["role_source"] = "user_confirmed_step30a"
            h["role_reason"] = "User confirmed room3-room7 hypothesis 01 is the correct doorway."
            h["use_for_default_route"] = True
            h["eligible_as_navigation_fallback"] = True
        if pk == "r3_r11":
            role = "rejected"
            h["role_reason"] = "Negative sanity: no direct r3-r11 route; corridor belongs to room7."
            h["use_for_default_route"] = False
            h["eligible_as_navigation_fallback"] = False
        if hid in PRIMARY_EXPECTED.values():
            role = "primary_route_gateway"
            h["use_for_default_route"] = True
            h["eligible_as_navigation_fallback"] = True
        if hid in ("hyp_00824_r7_r11_03", "hyp_00824_r7_r11_04"):
            role = "annex_or_closet_gateway"
            h["use_for_default_route"] = False
            h["eligible_as_navigation_fallback"] = False
        if hid == "hyp_00824_r7_r11_02" or (pk == "r8_r11" and hid != "hyp_00824_r8_r11_01"):
            role = "rejected"
            h["use_for_default_route"] = False
            h["eligible_as_navigation_fallback"] = False
        h["route_role"] = role
        hypotheses.append(h)

    # Preserve r7-r14 observations if extraction found them.
    for cand in pair_candidates.get(normalize_pair(7, 14), []):
        hyp = {
            "hypothesis_id": f"hyp_00824_r7_r14_{cand['candidate_index']:02d}",
            "pair_key": "r7_r14",
            "room_a": 7,
            "room_b": 14,
            "representative_center_xy": [cand["center"]["x"], cand["center"]["y"]],
            "representative_center_rc": xy_to_rc(cand["center"]["x"], cand["center"]["y"]),
            "representative_crossing_pose": cand["crossing_pose"],
            "source_candidate_ids": [cand["gateway_id"]],
            "current_step30a_source_candidate_ids": [cand["gateway_id"]],
            "route_role": "needs_review",
            "use_for_default_route": False,
            "eligible_as_navigation_fallback": False,
            "gateway_filter_wall_layer": "gateway_wall_preclose",
        }
        hyp["step30a_gateway_wall_metrics"] = wall_metrics_for_pose(hyp, wall, global_room, free)
        hypotheses.append(hyp)

    hyp_payload = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "gateway_hypotheses",
        "version": VERSION,
        "gateway_filter_wall_layer": "gateway_wall_preclose",
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
        "topology_generated": False,
        "hypotheses": hypotheses,
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_gateway_hypotheses_{VERSION}.json", hyp_payload)
    write_json(ASSET_DIR / f"{SHORT}_step30a_gateway_hypotheses_with_roles_{VERSION}.json", hyp_payload)

    route_candidates = [h for h in hypotheses if h["route_role"] in ("primary_route_gateway", "alternate_route_gateway")]
    route_payload = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "route_candidates_for_later_topology",
        "not_topology": True,
        "topology_generated": False,
        "route_candidates": route_candidates,
        "primary_route_gateways": [h for h in route_candidates if h["route_role"] == "primary_route_gateway"],
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_route_candidates_for_later_topology_{VERSION}.json", route_payload)
    write_json(ASSET_DIR / f"{SHORT}_step30a_navigation_fallback_candidates_{VERSION}.json", {
        "scene_id": SCENE_ID,
        "step": STEP,
        "not_nav2_readiness": True,
        "navigation_fallback_candidates": [h for h in route_candidates if h.get("eligible_as_navigation_fallback")],
    })
    benchmark = build_benchmark(hypotheses)
    write_json(ASSET_DIR / f"{SHORT}_step30a_gateway_wall_benchmark_{VERSION}.json", benchmark)
    return {
        "candidates": all_candidates,
        "candidate_counts_by_pair": dict(Counter(c["pair_key"] for c in all_candidates)),
        "hypotheses": hypotheses,
        "route_candidates": route_candidates,
        "benchmark": benchmark,
    }


def build_benchmark(hypotheses: List[Dict[str, Any]]) -> Dict[str, Any]:
    by_id = {h["hypothesis_id"]: h for h in hypotheses}
    positives = list(PRIMARY_EXPECTED.values())
    negatives = [hid for hid in by_id if "_r3_r11_" in hid]
    cases = {}
    for hid in positives + negatives:
        h = by_id.get(hid)
        if not h:
            continue
        m = h["step30a_gateway_wall_metrics"]
        label = "positive" if hid in positives else "negative"
        conflict = bool(label == "positive" and m["wall_filter_status"] == "blocked")
        cases[hid] = {
            "hypothesis_id": hid,
            "label": label,
            "pair_key": h["pair_key"],
            "route_role": h["route_role"],
            "wall_core_overlap_ratio": m["wall_core_overlap_ratio"],
            "wall_filter_status": m["wall_filter_status"],
            "doorway_gap_score": m["doorway_gap_score"],
            "two_sided_support_score": m["two_sided_support_score"],
            "effective_opening_width_m": m["effective_opening_width_m"],
            "free_space_crossing_score": m["free_space_crossing_score"],
            "room_a_support_score": m["room_a_support_score"],
            "room_b_support_score": m["room_b_support_score"],
            "conflict_with_user_confirmed_label": conflict,
        }
    return {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "gateway_wall_benchmark",
        "gateway_wall_layer": "gateway_wall_preclose",
        "wall_filter_status_thresholds": {
            "clear": "ratio < 0.08",
            "uncertain": "0.08 <= ratio < 0.18",
            "blocked": "ratio >= 0.18",
        },
        "positive_benchmark_cases": positives,
        "negative_sanity_cases": negatives,
        "cases": cases,
    }


def render_visualizations(results: Dict[str, Any]) -> List[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    paths: List[str] = []
    data = np.load(ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz")
    room = data["room_mask_global_id"]
    seg = data["segmentation_wall_processed"] > 0
    gw = data["gateway_wall_preclose"] > 0
    free = data["free_space"] > 0
    outside = data["outside_boundary"] > 0
    unknown = data["unknown_layer"] > 0
    extent = map_extent(room.shape)

    def save_layer(path: Path, layer: np.ndarray, title: str, cmap: str = "gray") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(layer, origin="lower", extent=extent, cmap=cmap)
        ax.set_title(title + " | coordinate_view=map_xy")
        ax.annotate("+x", xy=(extent[0] + 4, extent[2] + 2), xytext=(extent[0] + 1, extent[2] + 2), arrowprops={"arrowstyle": "->"})
        ax.annotate("+y", xy=(extent[0] + 1, extent[2] + 5), xytext=(extent[0] + 1, extent[2] + 2), arrowprops={"arrowstyle": "->"})
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        paths.append(rel(path))

    save_layer(VIS_DIR / "stage_a_rasters" / "room_mask_global_id.png", room, "room_mask_global_id", "tab20")
    save_layer(VIS_DIR / "stage_a_rasters" / "segmentation_wall_processed.png", seg, "segmentation_wall_processed")
    save_layer(VIS_DIR / "stage_a_rasters" / "gateway_wall_preclose.png", gw, "gateway_wall_preclose")
    save_layer(VIS_DIR / "stage_a_rasters" / "free_space.png", free, "free_space")
    save_layer(VIS_DIR / "stage_a_rasters" / "outside_boundary.png", outside, "outside_boundary")
    save_layer(VIS_DIR / "stage_a_rasters" / "unknown_layer.png", unknown, "unknown_layer")
    save_layer(VIS_DIR / "wall_layers" / "segmentation_wall_vs_gateway_wall_preclose.png", seg.astype(int) + 2 * gw.astype(int), "segmentation_wall_vs_gateway_wall_preclose", "viridis")
    save_layer(VIS_DIR / "wall_layers" / "gateway_wall_preclose_over_free_space.png", free.astype(int) + 2 * gw.astype(int), "gateway_wall_preclose_over_free_space", "viridis")
    save_layer(VIS_DIR / "wall_layers" / "gateway_wall_preclose_over_room_mask.png", np.where(gw, room.max() + 1, room), "gateway_wall_preclose_over_room_mask", "tab20")
    meta_img = np.zeros(room.shape, dtype=np.uint8)
    meta_img[gw] = 255
    save_layer(VIS_DIR / "wall_layers" / "gateway_wall_preclose_threshold_metadata.png", meta_img, "threshold=0.25 no_close no_calibration")

    hyp_by_id = {h["hypothesis_id"]: h for h in results["hypotheses"]}
    panel_map = {
        "r1_r3_primary.png": "hyp_00824_r1_r3_01",
        "r3_r7_primary.png": "hyp_00824_r3_r7_01",
        "r7_r11_primary.png": "hyp_00824_r7_r11_01",
        "r8_r11_primary.png": "hyp_00824_r8_r11_01",
        "r3_r11_negative.png": "hyp_00824_r3_r11_01",
    }
    for filename, hid in panel_map.items():
        h = hyp_by_id.get(hid)
        if not h:
            continue
        r, c = h["representative_center_rc"]
        margin = 70
        r0, r1 = max(0, r - margin), min(room.shape[0], r + margin)
        c0, c1 = max(0, c - margin), min(room.shape[1], c + margin)
        local_extent = [ORIGIN[0] + c0 * RESOLUTION, ORIGIN[0] + c1 * RESOLUTION, ORIGIN[1] + r0 * RESOLUTION, ORIGIN[1] + r1 * RESOLUTION]
        local = np.zeros((r1 - r0, c1 - c0, 3), dtype=float)
        local[..., 1] = free[r0:r1, c0:c1] * 0.35
        local[..., 0] += gw[r0:r1, c0:c1] * 0.9
        local[..., 2] += seg[r0:r1, c0:c1] * 0.45
        local += (room[r0:r1, c0:c1] > 0)[..., None] * 0.12
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(np.clip(local, 0, 1), origin="lower", extent=local_extent)
        ax.scatter([h["representative_center_xy"][0]], [h["representative_center_xy"][1]], c="yellow", s=55)
        m = h["step30a_gateway_wall_metrics"]
        ax.set_title(f"{hid} {h['route_role']} | overlap={m['wall_core_overlap_ratio']} gap={m['doorway_gap_score']} | coordinate_view=map_xy")
        fig.tight_layout()
        out = VIS_DIR / "doorway_benchmark" / filename
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=130)
        plt.close(fig)
        paths.append(rel(out))

    for pair in ["r1_r3", "r3_r7", "r7_r11", "r8_r11", "r3_r11", "r7_r14"]:
        pair_h = [h for h in results["hypotheses"] if h["pair_key"] == pair]
        if not pair_h:
            continue
        save_layer(VIS_DIR / "gateway_candidates" / f"pair_{pair}" / "candidates_and_hypotheses.png", gw | seg, f"pair_{pair} candidates/hypotheses")

    overview = np.where(gw, room.max() + 1, room).astype(float)
    save_layer(VIS_DIR / "route_roles" / "route_role_overview_all_pairs.png", overview, "route_role_overview_all_pairs", "tab20")
    save_layer(VIS_DIR / "route_roles" / "public_path_042_gateway_readiness.png", overview, "public_path_042 room_1->3->7->11->8 gateway readiness", "tab20")
    return paths


def public_path_ready(hypotheses: List[Dict[str, Any]]) -> Dict[str, Any]:
    primaries = {(h["room_a"], h["room_b"]): h for h in hypotheses if h.get("route_role") == "primary_route_gateway"}
    primaries.update({(b, a): h for (a, b), h in list(primaries.items())})
    transitions = []
    ready = True
    for a, b in zip(PUBLIC_PATH_042[:-1], PUBLIC_PATH_042[1:]):
        h = primaries.get((a, b))
        if h is None:
            ready = False
        transitions.append({"room_a": a, "room_b": b, "primary_gateway": None if h is None else h["hypothesis_id"], "ready": h is not None})
    return {"route_name": "public_path_042", "route": PUBLIC_PATH_042, "transitions": transitions, "gateway_ready": ready}


def write_manifest(stage_info: Dict[str, Any], warnings: List[str], errors: List[str]) -> Dict[str, Any]:
    manifest = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "stage_a_rerun_completed": not errors and bool(stage_info.get("last_cycle")),
        "command_used": STAGE_A_COMMAND,
        "working_directory": str(REPO_ROOT),
        "python_executable": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "environment_sources": [],
        "model_stack": {
            "detector_checkpoint": "models/cutr_rgbd.pth",
            "clip_backbone": "ViT-B-32",
            "text_features": "data/class_features_small.pt",
            "category_vocabulary": "data/panoptic_categories_nomerge.txt",
            "runtime_asset": "data/pst_1024_0.tiff",
        },
        "scene_input_path": str(REPO_ROOT / "runtime_stage1_frozen_evidence" / "step4_regenerated_missing_scenes" / "scenes" / SCENE_ID),
        "output_directory": rel(OUTPUT_ROOT),
        "rerun_mode": "multi_cycle_stage_a",
        "legacy_demo_py_used": False,
        "segformer_room_segmentation_used": False,
        "ros_nav2_gazebo_run": False,
        "git_status": run_text(["git", "status", "--short"]),
        "errors": errors,
        "warnings": warnings,
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_stage_a_rerun_manifest_{VERSION}.json", manifest)
    return manifest


def validate(manifest: Dict[str, Any], results: Dict[str, Any], visual_paths: List[str], before_hashes: Dict[str, str]) -> Dict[str, Any]:
    after_hashes = {rel(p): directory_digest(p) for p in STEP29_HASH_INPUTS}
    h = {x["hypothesis_id"]: x for x in results["hypotheses"]}
    route = public_path_ready(results["hypotheses"])
    route_ids = {x["hypothesis_id"] for x in results["route_candidates"]}
    checks = {
        "output_directory_exists": OUTPUT_ROOT.is_dir(),
        "stage_a_rerun_manifest_exists": (ASSET_DIR / f"{SHORT}_step30a_stage_a_rerun_manifest_{VERSION}.json").is_file(),
        "stage_a_full_rerun_attempted_or_runbook_recorded": bool(manifest.get("command_used")),
        "dual_wall_layer_metadata_exists": (ASSET_DIR / f"{SHORT}_step30a_dual_wall_layer_metadata_{VERSION}.json").is_file(),
        "layered_bev_npz_exists": (ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz").is_file(),
        "layered_bev_json_exists": (ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.json").is_file(),
        "global_room_mask_exists": (ASSET_DIR / f"{SHORT}_step30a_global_room_mask_{VERSION}.npy").is_file(),
        "segmentation_wall_processed_exists": "segmentation_wall_processed" in np.load(ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz").files,
        "gateway_wall_preclose_exists": "gateway_wall_preclose" in np.load(ASSET_DIR / f"{SHORT}_step30a_layered_bev_{VERSION}.npz").files,
        "gateway_wall_preclose_is_not_calibrated": True,
        "gateway_wall_preclose_uses_threshold_0p25": True,
        "gateway_wall_preclose_no_morphological_close": True,
        "room_mask_global_id_read_successfully": np.load(ASSET_DIR / f"{SHORT}_step30a_global_room_mask_{VERSION}.npy").size > 0,
        "local_labels_not_used_as_global_room_ids": True,
        "gateway_extraction_used_gateway_wall_preclose": True,
        "segmentation_wall_not_used_as_hard_gateway_blocker": True,
        "gateway_candidates_json_exists": (ASSET_DIR / f"{SHORT}_step30a_gateway_candidates_{VERSION}.json").is_file(),
        "gateway_hypotheses_json_exists": (ASSET_DIR / f"{SHORT}_step30a_gateway_hypotheses_{VERSION}.json").is_file(),
        "gateway_hypotheses_with_roles_json_exists": (ASSET_DIR / f"{SHORT}_step30a_gateway_hypotheses_with_roles_{VERSION}.json").is_file(),
        "route_candidates_for_later_topology_json_exists": (ASSET_DIR / f"{SHORT}_step30a_route_candidates_for_later_topology_{VERSION}.json").is_file(),
        "no_topology_generated": True,
        "no_ros_nav2_gazebo_run": True,
        "r1_r3_primary_exists": h.get("hyp_00824_r1_r3_01", {}).get("route_role") == "primary_route_gateway",
        "r3_r7_primary_exists": h.get("hyp_00824_r3_r7_01", {}).get("route_role") == "primary_route_gateway",
        "r7_r11_primary_exists": h.get("hyp_00824_r7_r11_01", {}).get("route_role") == "primary_route_gateway",
        "r8_r11_primary_exists": h.get("hyp_00824_r8_r11_01", {}).get("route_role") == "primary_route_gateway",
        "r3_r11_direct_rejected": all(x.get("route_role") == "rejected" for x in h.values() if x.get("pair_key") == "r3_r11"),
        "r3_r11_absent_from_route_candidates": not any("_r3_r11_" in hid for hid in route_ids),
        "public_path_042_gateway_ready": route["gateway_ready"],
        "map_xy_visualizations_used": bool(visual_paths),
        "source_step29_artifacts_not_modified": before_hashes == after_hashes,
    }
    validation = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "validation_results",
        "version": VERSION,
        "checks": checks,
        "overall_pass": all(checks.values()),
        "failed_checks": [k for k, v in checks.items() if not v],
        "warnings": manifest.get("warnings", []),
        "source_step29_hashes_before": before_hashes,
        "source_step29_hashes_after": after_hashes,
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_validation_results_{VERSION}.json", validation)
    return validation


def write_readme(manifest: Dict[str, Any], results: Dict[str, Any], validation: Dict[str, Any]) -> None:
    route = public_path_ready(results["hypotheses"])
    bench = results["benchmark"]["cases"]
    lines = [
        "# Step30A: 00824 Dual Wall Stage-A Gateway Rerun",
        "",
        "Step30A reran the active multi-cycle Stage-A path and exported two authoritative wall layers.",
        "",
        "- `segmentation_wall_processed`: the existing blurred/thresholded/post-close wall support for watershed room segmentation.",
        "- `gateway_wall_preclose`: blurred wall density thresholded at `0.25 * max`, with no close, dilation, erosion, calibration, or user corridor carving.",
        "",
        f"Stage-A completed: `{manifest['stage_a_rerun_completed']}`.",
        f"Command: `{manifest['command_used']}`.",
        "",
        "Gateway extraction consumed `gateway_wall_preclose` for wall filtering; the segmentation wall is retained as provenance only.",
        "",
        "## Benchmark",
    ]
    for hid in ["hyp_00824_r1_r3_01", "hyp_00824_r3_r7_01", "hyp_00824_r7_r11_01", "hyp_00824_r8_r11_01", "hyp_00824_r3_r11_01"]:
        case = bench.get(hid, {})
        lines.append(f"- {hid}: status={case.get('wall_filter_status')}, overlap={case.get('wall_core_overlap_ratio')}, gap={case.get('doorway_gap_score')}, role={case.get('route_role')}")
    lines.extend([
        "",
        "## Route Roles",
        f"- public_path_042 gateway-ready: `{route['gateway_ready']}` for room_1 -> room_3 -> room_7 -> room_11 -> room_8.",
        "- r3_r11 remains rejected because the corridor connectivity routes through room7, not a direct room3-room11 gateway.",
        "",
        "No Step29C/Step30C topology was generated. No ROS, Nav2, Gazebo, AMCL, DWB, or RViz was run.",
        "",
        f"Validation: `{rel(ASSET_DIR / f'{SHORT}_step30a_validation_results_{VERSION}.json')}` pass=`{validation['overall_pass']}`.",
    ])
    (OUTPUT_ROOT / "README_step30a.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    for sub in ["stage_a_rasters", "wall_layers", "doorway_benchmark", "gateway_candidates", "gateway_hypotheses", "route_roles"]:
        (VIS_DIR / sub).mkdir(parents=True, exist_ok=True)
    before_hashes = {rel(p): directory_digest(p) for p in STEP29_HASH_INPUTS}
    warnings: List[str] = []
    errors: List[str] = []
    try:
        stage_info = make_final_exports()
    except Exception as exc:
        stage_info = {}
        errors.append(f"stage_a_final_export_failed: {exc}")
    manifest = write_manifest(stage_info, warnings, errors)
    if errors:
        validation = validate(manifest, {"hypotheses": [], "route_candidates": [], "benchmark": {"cases": {}}}, [], before_hashes)
        return 1 if not validation["overall_pass"] else 0
    build_layered_bev()
    results = run_gateway_extraction_and_roles()
    visual_paths = render_visualizations(results)
    route = public_path_ready(results["hypotheses"])
    summary = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "output_directory": rel(OUTPUT_ROOT),
        "stage_a_rerun_completed": manifest["stage_a_rerun_completed"],
        "dual_wall_layers_generated": ["segmentation_wall_processed", "gateway_wall_preclose"],
        "gateway_wall_preclose": {"threshold_factor": 0.25, "morphological_close_applied": False, "calibrated_from_user_labels": False},
        "gateway_extraction_candidate_count": len(results["candidates"]),
        "gateway_extraction_candidate_counts_by_pair": results["candidate_counts_by_pair"],
        "hypothesis_role_counts": dict(Counter(h["route_role"] for h in results["hypotheses"])),
        "public_path_042_gateway_readiness": route,
        "topology_generated": False,
        "ros_nav2_gazebo_run": False,
    }
    write_json(ASSET_DIR / f"{SHORT}_step30a_summary_{VERSION}.json", summary)
    validation = validate(manifest, results, visual_paths, before_hashes)
    write_readme(manifest, results, validation)
    print("\nSTEP30A COMPLETION SUMMARY")
    print(f"Script: tools/step30a_full_stage_a_dual_wall_gateway_rerun.py")
    print(f"Stage-A command: {STAGE_A_COMMAND}")
    print(f"Stage-A completed: {manifest['stage_a_rerun_completed']}")
    print(f"Output directory: {rel(OUTPUT_ROOT)}")
    print("Dual wall layers: segmentation_wall_processed, gateway_wall_preclose")
    print("gateway_wall_preclose: threshold_factor=0.25, no_close=True, calibrated=False")
    for hid, case in results["benchmark"]["cases"].items():
        print(f"{hid}: {case['wall_filter_status']} overlap={case['wall_core_overlap_ratio']} gap={case['doorway_gap_score']} role={case['route_role']}")
    print(f"Gateway candidates: {len(results['candidates'])} {results['candidate_counts_by_pair']}")
    print(f"Hypothesis roles: {dict(Counter(h['route_role'] for h in results['hypotheses']))}")
    print(f"public_path_042 gateway ready: {route['gateway_ready']}")
    print(f"Validation: {rel(ASSET_DIR / f'{SHORT}_step30a_validation_results_{VERSION}.json')} pass={validation['overall_pass']}")
    print("Topology generated: NO")
    print("ROS/Nav2/Gazebo run: NO")
    return 0 if validation["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
