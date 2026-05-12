"""
Extract Step29B2 room-boundary gateway candidates from the Step29B1 layered BEV.

This stage is deliberately offline and geometry-constrained. Room-pair operations
use only room_mask_global_id for room identity, plus structural/free/explored
layers from the Step29B1 NPZ. Local labels are read only for the mandatory
unmapped-label-10 audit.
"""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT_SCENE_ID = "00824"
VERSION = "v0_1"
MAP_FRAME = "map"
RESOLUTION = 0.05
ORIGIN = [-50.0, -50.0]

REPO_ROOT = Path(__file__).resolve().parents[1]
STEP29B1_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b1_00824_layered_bev_from_step29a3"
STEP29B1_ASSET_DIR = STEP29B1_ROOT / "generated" / "assets"
STEP29B2_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step29b2_00824_gateway_candidates_from_layered_bev"
ASSET_DIR = STEP29B2_ROOT / "generated" / "assets"
VIS_DIR = STEP29B2_ROOT / "generated" / "visualizations"
README_PATH = STEP29B2_ROOT / "README_step29b2.md"

STEP29B1_LAYERED_JSON = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.json"
STEP29B1_LAYERED_NPZ = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_layered_bev_from_step29a3_{VERSION}.npz"
STEP29B1_SUMMARY_JSON = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_summary_{VERSION}.json"
STEP29B1_VALIDATION_JSON = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_step29b1_validation_results_{VERSION}.json"
STEP29B1_GLOBAL_ROOM_MASK_NPY = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_global_room_mask_{VERSION}.npy"
STEP29B1_MAPPING_JSON = STEP29B1_ASSET_DIR / f"{SHORT_SCENE_ID}_room_label_mapping_{VERSION}.json"

GATEWAY_CANDIDATES_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_candidates_{VERSION}.json"
GATEWAY_GRAPH_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_graph_{VERSION}.json"
EXTRACTION_SUMMARY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_extraction_summary_{VERSION}.json"
VALIDATION_RESULTS_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_gateway_candidate_validation_results_{VERSION}.json"
ROOM_PAIR_ADJACENCY_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_room_pair_adjacency_candidates_{VERSION}.json"
REJECTED_ROOM_EDGES_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_rejected_room_edges_{VERSION}.json"
LABEL10_WALL_AUDIT_JSON = ASSET_DIR / f"{SHORT_SCENE_ID}_label10_and_wall_overlap_audit_{VERSION}.json"

EXPLICIT_ROOM_PAIRS = [(3, 11), (11, 7), (3, 7), (11, 8), (1, 3), (7, 11), (8, 11)]
KEY_ROOM_PAIRS = [(3, 11), (11, 7), (3, 7), (11, 8)]
PUBLIC_PATH_042 = [1, 3, 7, 11, 8]
PUBLIC_PATH_TRANSITIONS = list(zip(PUBLIC_PATH_042[:-1], PUBLIC_PATH_042[1:]))
EXPECTED_KEY_GLOBAL_ROOM_IDS = [1, 3, 7, 8, 11]
STATUS_VALUES = {"valid", "ambiguous", "misleading", "rejected"}
SOURCE_LAYERS = [
    "room_mask_global_id",
    "structural_wall",
    "free_space",
    "outside_boundary",
    "unknown_layer",
]
FORBIDDEN_GEOMETRY_SOURCES_NOT_USED = [
    "room polygons",
    "old navigation BEV",
    "selected_public_route_edges_carved_w0p6",
    "old route candidate BEV",
    "topology JSON edges as geometry",
    "Nav2 outputs",
    "Gazebo outputs",
    "AMCL/TF/DWB/ROS execution artifacts",
]


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


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


def meters_to_cells(meters: float) -> int:
    return max(1, int(round(meters / RESOLUTION)))


def disk_offsets(radius: int) -> List[Tuple[int, int]]:
    offsets: List[Tuple[int, int]] = []
    r2 = radius * radius
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dy * dy + dx * dx <= r2:
                offsets.append((dy, dx))
    return offsets


_OFFSETS_CACHE: Dict[int, List[Tuple[int, int]]] = {}


def get_offsets(radius: int) -> List[Tuple[int, int]]:
    if radius not in _OFFSETS_CACHE:
        _OFFSETS_CACHE[radius] = disk_offsets(radius)
    return _OFFSETS_CACHE[radius]


def shifted_or(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.astype(bool, copy=True)
    src = mask.astype(bool, copy=False)
    out = np.zeros(src.shape, dtype=bool)
    h, w = src.shape
    for dy, dx in get_offsets(radius):
        ys_src0 = max(0, -dy)
        ys_src1 = min(h, h - dy)
        xs_src0 = max(0, -dx)
        xs_src1 = min(w, w - dx)
        ys_dst0 = max(0, dy)
        ys_dst1 = min(h, h + dy)
        xs_dst0 = max(0, dx)
        xs_dst1 = min(w, w + dx)
        if ys_src0 < ys_src1 and xs_src0 < xs_src1:
            out[ys_dst0:ys_dst1, xs_dst0:xs_dst1] |= src[ys_src0:ys_src1, xs_src0:xs_src1]
    return out


def erode_disk(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.astype(bool, copy=True)
    return ~shifted_or(~mask.astype(bool, copy=False), radius)


def bbox_for_mask(mask: np.ndarray, margin: int = 0) -> Tuple[int, int, int, int]:
    ys, xs = np.where(mask)
    if ys.size == 0:
        return 0, 0, mask.shape[0] - 1, mask.shape[1] - 1
    return (
        max(int(ys.min()) - margin, 0),
        max(int(xs.min()) - margin, 0),
        min(int(ys.max()) + margin, mask.shape[0] - 1),
        min(int(xs.max()) + margin, mask.shape[1] - 1),
    )


def crop_slices_for_rooms(global_room: np.ndarray, room_ids: Iterable[int], margin: int = 90) -> Tuple[slice, slice]:
    combined = np.zeros(global_room.shape, dtype=bool)
    for room_id in room_ids:
        combined |= global_room == int(room_id)
    r0, c0, r1, c1 = bbox_for_mask(combined, margin=margin)
    return slice(r0, r1 + 1), slice(c0, c1 + 1)


def grid_to_xy(row: float, col: float) -> Dict[str, float]:
    return {
        "x": round(float(ORIGIN[0] + col * RESOLUTION), 4),
        "y": round(float(ORIGIN[1] + row * RESOLUTION), 4),
    }


def pose_from_row_col(row: float, col: float, yaw: float) -> Dict[str, float]:
    xy = grid_to_xy(row, col)
    xy["yaw"] = round(float(yaw), 6)
    return xy


def bbox_map_xy(bbox_cells: Sequence[int]) -> List[float]:
    r0, c0, r1, c1 = [float(v) for v in bbox_cells]
    xy0 = grid_to_xy(r0, c0)
    xy1 = grid_to_xy(r1, c1)
    return [xy0["x"], xy0["y"], xy1["x"], xy1["y"]]


def centroid(mask: np.ndarray) -> Optional[Tuple[float, float]]:
    ys, xs = np.where(mask)
    if ys.size == 0:
        return None
    return float(np.mean(ys)), float(np.mean(xs))


def connected_components(mask: np.ndarray, min_cells: int = 1) -> List[np.ndarray]:
    src = mask.astype(bool, copy=False)
    visited = np.zeros(src.shape, dtype=bool)
    coords = np.argwhere(src)
    components: List[np.ndarray] = []
    h, w = src.shape
    for start_r, start_c in coords:
        sr = int(start_r)
        sc = int(start_c)
        if visited[sr, sc]:
            continue
        q: deque[Tuple[int, int]] = deque([(sr, sc)])
        visited[sr, sc] = True
        cells: List[Tuple[int, int]] = []
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for nr in (r - 1, r, r + 1):
                for nc in (c - 1, c, c + 1):
                    if nr == r and nc == c:
                        continue
                    if nr < 0 or nc < 0 or nr >= h or nc >= w:
                        continue
                    if visited[nr, nc] or not src[nr, nc]:
                        continue
                    visited[nr, nc] = True
                    q.append((nr, nc))
        if len(cells) >= min_cells:
            comp = np.array(cells, dtype=np.int32)
            components.append(comp)
    return components


def reachable_room_interiors(
    seeds: np.ndarray,
    allowed: np.ndarray,
    interior_a: np.ndarray,
    interior_b: np.ndarray,
) -> Tuple[bool, bool, int]:
    if seeds.size == 0:
        return False, False, 0
    h, w = allowed.shape
    visited = np.zeros(allowed.shape, dtype=bool)
    q: deque[Tuple[int, int]] = deque()
    for r, c in seeds.tolist():
        rr = int(r)
        cc = int(c)
        if 0 <= rr < h and 0 <= cc < w and allowed[rr, cc] and not visited[rr, cc]:
            visited[rr, cc] = True
            q.append((rr, cc))
    touches_a = False
    touches_b = False
    count = 0
    while q:
        r, c = q.popleft()
        count += 1
        if interior_a[r, c]:
            touches_a = True
        if interior_b[r, c]:
            touches_b = True
        if touches_a and touches_b and count > 10:
            # Continue is unnecessary once both sides are proven.
            return True, True, count
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if nr < 0 or nc < 0 or nr >= h or nc >= w:
                continue
            if visited[nr, nc] or not allowed[nr, nc]:
                continue
            visited[nr, nc] = True
            q.append((nr, nc))
    return touches_a, touches_b, count


def rle_encode_component(component_local: np.ndarray) -> Tuple[List[int], List[List[int]]]:
    r0, c0, r1, c1 = bbox_for_component(component_local)
    h = r1 - r0 + 1
    w = c1 - c0 + 1
    mask = np.zeros((h, w), dtype=bool)
    mask[component_local[:, 0] - r0, component_local[:, 1] - c0] = True
    runs: List[List[int]] = []
    for rr in range(h):
        cols = np.where(mask[rr])[0]
        if cols.size == 0:
            continue
        start = int(cols[0])
        prev = int(cols[0])
        for col in cols[1:].tolist():
            col_i = int(col)
            if col_i == prev + 1:
                prev = col_i
            else:
                runs.append([rr, start, prev - start + 1])
                start = col_i
                prev = col_i
        runs.append([rr, start, prev - start + 1])
    return [int(r0), int(c0), int(r1), int(c1)], runs


def bbox_for_component(component_local: np.ndarray) -> Tuple[int, int, int, int]:
    return (
        int(component_local[:, 0].min()),
        int(component_local[:, 1].min()),
        int(component_local[:, 0].max()),
        int(component_local[:, 1].max()),
    )


def component_mask_from_cells(shape: Tuple[int, int], component_local: np.ndarray) -> np.ndarray:
    mask = np.zeros(shape, dtype=bool)
    mask[component_local[:, 0], component_local[:, 1]] = True
    return mask


def component_duplicate(component: np.ndarray, existing: List[np.ndarray], center_distance_cells: float = 6.0) -> bool:
    if not existing:
        return False
    comp_set = set((int(r), int(c)) for r, c in component.tolist())
    cy = float(np.mean(component[:, 0]))
    cx = float(np.mean(component[:, 1]))
    for prev in existing:
        prev_set = set((int(r), int(c)) for r, c in prev.tolist())
        overlap = len(comp_set & prev_set)
        if overlap and overlap / max(1, min(len(comp_set), len(prev_set))) >= 0.45:
            return True
        py = float(np.mean(prev[:, 0]))
        px = float(np.mean(prev[:, 1]))
        if math.hypot(cy - py, cx - px) <= center_distance_cells:
            return True
    return False


def normalize_pair(room_a: int, room_b: int) -> Tuple[int, int]:
    a = int(room_a)
    b = int(room_b)
    return (a, b) if a <= b else (b, a)


def display_pair(room_a: int, room_b: int) -> str:
    a, b = normalize_pair(room_a, room_b)
    return f"room_{a}<->room_{b}"


def build_boundary_band(room_a_mask: np.ndarray, room_b_mask: np.ndarray, radius_cells: int, explored: np.ndarray) -> np.ndarray:
    return shifted_or(room_a_mask, radius_cells) & shifted_or(room_b_mask, radius_cells) & explored


def load_layers() -> Dict[str, Any]:
    required = [
        STEP29B1_LAYERED_JSON,
        STEP29B1_LAYERED_NPZ,
        STEP29B1_SUMMARY_JSON,
        STEP29B1_VALIDATION_JSON,
        STEP29B1_GLOBAL_ROOM_MASK_NPY,
        STEP29B1_MAPPING_JSON,
    ]
    missing = [rel(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"Missing Step29B1 inputs: {missing}")
    data = np.load(STEP29B1_LAYERED_NPZ)
    return {
        "metadata": read_json(STEP29B1_LAYERED_JSON),
        "summary": read_json(STEP29B1_SUMMARY_JSON),
        "validation": read_json(STEP29B1_VALIDATION_JSON),
        "mapping": read_json(STEP29B1_MAPPING_JSON),
        "npz": data,
        "global_room_mask_npy": np.load(STEP29B1_GLOBAL_ROOM_MASK_NPY),
    }


def audit_label10_and_wall_overlap(layers: Dict[str, Any]) -> Dict[str, Any]:
    data = layers["npz"]
    global_room = data["room_mask_global_id"].astype(np.int32)
    local_repaired = data["room_mask_local_label_repaired"].astype(np.int32)
    wall = data["structural_wall"] > 0
    free = data["free_space"] > 0
    outside = data["outside_boundary"] > 0
    unknown = data["unknown_layer"] > 0
    room_positive = global_room > 0
    label10 = local_repaired == 10
    wall_room_overlap = wall & room_positive

    boundary_pairs = sorted({normalize_pair(a, b) for a, b in EXPLICIT_ROOM_PAIRS + KEY_ROOM_PAIRS + PUBLIC_PATH_TRANSITIONS})
    label10_boundary_counts: Dict[str, int] = {}
    overlap_boundary_counts: Dict[str, int] = {}
    label10_near_any_boundary = np.zeros(label10.shape, dtype=bool)
    overlap_near_any_key_pair = np.zeros(label10.shape, dtype=bool)
    for room_a, room_b in boundary_pairs:
        room_a_mask = global_room == room_a
        room_b_mask = global_room == room_b
        if not np.any(room_a_mask) or not np.any(room_b_mask):
            band = np.zeros(global_room.shape, dtype=bool)
        else:
            band = build_boundary_band(room_a_mask, room_b_mask, meters_to_cells(0.40), outside)
        key = display_pair(room_a, room_b)
        label10_boundary_counts[key] = int(np.count_nonzero(label10 & band))
        overlap_boundary_counts[key] = int(np.count_nonzero(wall_room_overlap & band))
        label10_near_any_boundary |= band
        if normalize_pair(room_a, room_b) in {normalize_pair(*p) for p in KEY_ROOM_PAIRS}:
            overlap_near_any_key_pair |= band

    label10_touches_rooms: Dict[str, bool] = {}
    label10_dilated = shifted_or(label10, 1)
    for room_id in [3, 7, 8, 11]:
        label10_touches_rooms[f"room_{room_id}"] = bool(np.any(label10_dilated & (global_room == room_id)))

    label10_count = int(np.count_nonzero(label10))
    label10_free = int(np.count_nonzero(label10 & free))
    label10_wall = int(np.count_nonzero(label10 & wall))
    label10_unknown = int(np.count_nonzero(label10 & unknown))
    label10_outside = int(np.count_nonzero(label10 & outside))
    label10_near_boundary_count = int(np.count_nonzero(label10 & label10_near_any_boundary))

    suspicious = (
        label10_count > 50
        and label10_free / max(1, label10_count) > 0.45
        and any(label10_touches_rooms.values())
        and label10_wall / max(1, label10_count) < 0.25
        and label10_unknown / max(1, label10_count) < 0.25
    )
    if label10_count == 0:
        label10_interpretation = "absent"
    elif suspicious:
        label10_interpretation = "suspicious_possible_missed_interior_region_but_not_used_for_step29b2"
    else:
        label10_interpretation = "boundary_wall_background_or_unmapped_artifact_not_used_as_room"

    overlap_by_room: Dict[str, int] = {}
    for room_id in sorted(int(v) for v in np.unique(global_room) if int(v) > 0):
        overlap_by_room[f"room_{room_id}"] = int(np.count_nonzero(wall_room_overlap & (global_room == room_id)))
    key_overlap_concentration = {
        key: count for key, count in overlap_boundary_counts.items() if key in {display_pair(*p) for p in KEY_ROOM_PAIRS}
    }

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2_label10_and_wall_overlap_audit",
        "version": VERSION,
        "label10_audit": {
            "local_repaired_label": 10,
            "cell_count": label10_count,
            "overlap_with_structural_wall": label10_wall,
            "overlap_with_unknown_layer": label10_unknown,
            "overlap_with_free_space": label10_free,
            "overlap_with_outside_boundary": label10_outside,
            "free_fraction": round(label10_free / max(1, label10_count), 6),
            "wall_fraction": round(label10_wall / max(1, label10_count), 6),
            "unknown_fraction": round(label10_unknown / max(1, label10_count), 6),
            "touches_key_global_rooms": label10_touches_rooms,
            "near_room_pair_boundary_cell_counts": label10_boundary_counts,
            "near_any_evaluated_boundary_count": label10_near_boundary_count,
            "strong_evidence_real_missed_room": bool(suspicious),
            "step29b2_usage": "not_used_as_room_for_gateway_extraction",
            "interpretation": label10_interpretation,
        },
        "wall_room_overlap_audit": {
            "overlap_cell_count": int(np.count_nonzero(wall_room_overlap)),
            "overlap_by_global_room_id": overlap_by_room,
            "near_key_pair_boundary_overlap_counts": key_overlap_concentration,
            "near_evaluated_pair_boundary_overlap_counts": overlap_boundary_counts,
            "policy": "wall-room overlap cells are not treated as reliable free space; candidates depending on nearby overlap are ambiguous or misleading.",
        },
    }


def discover_room_pairs(global_room: np.ndarray, outside: np.ndarray) -> List[Tuple[int, int]]:
    present = sorted(int(v) for v in np.unique(global_room) if int(v) > 0)
    selected: set[Tuple[int, int]] = set()
    auto_radius = meters_to_cells(0.60)
    for i, room_a in enumerate(present):
        mask_a = global_room == room_a
        dil_a = shifted_or(mask_a, auto_radius)
        for room_b in present[i + 1 :]:
            mask_b = global_room == room_b
            near = bool(np.any(dil_a & shifted_or(mask_b, auto_radius) & outside))
            if near:
                selected.add(normalize_pair(room_a, room_b))
    for pair in EXPLICIT_ROOM_PAIRS + PUBLIC_PATH_TRANSITIONS:
        selected.add(normalize_pair(*pair))
    return sorted(selected)


def classify_candidate(
    pair: Tuple[int, int],
    validation: Dict[str, Any],
    width_m: float,
    component_cell_count: int,
    free_fraction: float,
    unknown_fraction: float,
    wall_overlap_fraction: float,
    label10_fraction: float,
) -> Tuple[str, float, str]:
    connects_a = bool(validation["connects_room_a"])
    connects_b = bool(validation["connects_room_b"])
    two_sided = bool(validation["two_sided_connectivity"])
    if not two_sided:
        if connects_a or connects_b:
            return "rejected", 0.05, "one_sided_connectivity_only"
        return "rejected", 0.02, "no_two_sided_connectivity"
    if component_cell_count < 5:
        return "rejected", 0.10, "tiny_component"
    if width_m < 0.18:
        return "rejected", 0.12, "too_narrow_for_turtlebot3_passage"

    confidence = 0.35
    confidence += min(0.25, free_fraction * 0.28)
    confidence += min(0.20, width_m / 1.8)
    confidence += 0.12 if component_cell_count >= 20 else 0.04
    confidence -= min(0.20, unknown_fraction * 0.6)
    confidence -= min(0.18, wall_overlap_fraction * 0.5)
    confidence -= min(0.12, label10_fraction * 0.4)
    confidence = max(0.0, min(0.99, confidence))

    normalized = normalize_pair(*pair)
    if free_fraction < 0.20:
        return "misleading", min(confidence, 0.35), "weak_free_space_evidence"
    if label10_fraction > 0.30:
        return "ambiguous", min(confidence, 0.50), "depends_on_or_touches_unmapped_label10_region"
    if wall_overlap_fraction > 0.28:
        return "ambiguous", min(confidence, 0.52), "depends_on_wall_room_overlap_context"
    if width_m < 0.35:
        return "ambiguous", min(confidence, 0.58), "narrow_gateway_evidence"
    if unknown_fraction > 0.05:
        return "ambiguous", min(confidence, 0.55), "near_unknown_or_exterior_cells"
    if normalized == normalize_pair(3, 11) and confidence < 0.86:
        return "misleading", min(confidence, 0.55), "room3_room11_requires_stronger_direct_gateway_evidence"
    if confidence >= 0.62:
        return "valid", confidence, "valid_two_sided_free_boundary_component"
    return "ambiguous", confidence, "possible_gateway_but_evidence_incomplete"


def extract_pair_candidates(
    pair: Tuple[int, int],
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    outside: np.ndarray,
    unknown: np.ndarray,
    local_repaired: np.ndarray,
    wall_room_overlap: np.ndarray,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    room_a, room_b = normalize_pair(*pair)
    row_slice, col_slice = crop_slices_for_rooms(global_room, [room_a, room_b], margin=meters_to_cells(1.25))
    row_offset = int(row_slice.start or 0)
    col_offset = int(col_slice.start or 0)
    g = global_room[row_slice, col_slice]
    wall_c = wall[row_slice, col_slice]
    free_c = free[row_slice, col_slice]
    outside_c = outside[row_slice, col_slice]
    unknown_c = unknown[row_slice, col_slice]
    local_c = local_repaired[row_slice, col_slice]
    wall_overlap_c = wall_room_overlap[row_slice, col_slice]
    room_a_mask = g == room_a
    room_b_mask = g == room_b

    radius_cells_sequence = [meters_to_cells(v) for v in [0.15, 0.25, 0.40, 0.60]]
    radius_m_by_cells = {meters_to_cells(v): v for v in [0.15, 0.25, 0.40, 0.60]}
    near_boundary_detected = bool(np.any(build_boundary_band(room_a_mask, room_b_mask, meters_to_cells(0.60), outside_c)))

    if not np.any(room_a_mask) or not np.any(room_b_mask):
        return [], {
            "room_a": room_a,
            "room_b": room_b,
            "near_boundary_detected": False,
            "evaluated_boundary_radii_m": [0.15, 0.25, 0.40, 0.60],
            "notes": "one_or_both_room_masks_absent",
        }

    interior_a = erode_disk(room_a_mask, meters_to_cells(0.15))
    interior_b = erode_disk(room_b_mask, meters_to_cells(0.15))
    if not np.any(interior_a):
        interior_a = room_a_mask
    if not np.any(interior_b):
        interior_b = room_b_mask

    free_near = shifted_or(free_c, meters_to_cells(0.10))
    other_room = (g > 0) & (g != room_a) & (g != room_b)
    allowed_base = outside_c & ~unknown_c & ~wall_c & ~other_room
    candidates: List[Dict[str, Any]] = []
    existing_components: List[np.ndarray] = []
    raw_index = 0

    room_a_cent = centroid(room_a_mask)
    room_b_cent = centroid(room_b_mask)
    if room_a_cent and room_b_cent:
        dy = room_b_cent[0] - room_a_cent[0]
        dx = room_b_cent[1] - room_a_cent[1]
    else:
        dy, dx = 0.0, 1.0
    yaw = math.atan2(dy, dx)
    norm = math.hypot(dy, dx) or 1.0
    unit_row = dy / norm
    unit_col = dx / norm
    approach_cells = meters_to_cells(0.35)

    for radius_cells in radius_cells_sequence:
        band = build_boundary_band(room_a_mask, room_b_mask, radius_cells, outside_c)
        traversable = band & allowed_base & (free_c | free_near | room_a_mask | room_b_mask)
        comps = connected_components(traversable, min_cells=3)
        for comp in comps:
            if component_duplicate(comp, existing_components):
                continue
            existing_components.append(comp)
            raw_index += 1
            comp_mask = component_mask_from_cells(g.shape, comp)
            comp_context = shifted_or(comp_mask, 1)
            seed = comp
            allowed = allowed_base & (free_near | room_a_mask | room_b_mask | comp_mask)
            connects_a, connects_b, reachable_cells = reachable_room_interiors(seed, allowed, interior_a, interior_b)
            two_sided = connects_a and connects_b
            r0, c0, r1, c1 = bbox_for_component(comp)
            bbox_cells = [r0 + row_offset, c0 + col_offset, r1 + row_offset, c1 + col_offset]
            height_cells = r1 - r0 + 1
            width_cells = c1 - c0 + 1
            min_width_cells = int(min(height_cells, width_cells))
            width_m = round(min_width_cells * RESOLUTION, 4)
            component_cell_count = int(comp.shape[0])
            component_area_m2 = round(component_cell_count * RESOLUTION * RESOLUTION, 6)
            free_fraction = float(np.count_nonzero(comp_mask & free_c) / max(1, component_cell_count))
            unknown_fraction = float(np.count_nonzero(comp_context & unknown_c) / max(1, np.count_nonzero(comp_context)))
            wall_overlap_fraction = float(np.count_nonzero(comp_context & wall_overlap_c) / max(1, np.count_nonzero(comp_context)))
            label10_fraction = float(np.count_nonzero(comp_context & (local_c == 10)) / max(1, np.count_nonzero(comp_context)))
            validation = {
                "connects_room_a": bool(connects_a),
                "connects_room_b": bool(connects_b),
                "two_sided_connectivity": bool(two_sided),
                "blocked_by_wall_evidence": bool(np.any(comp_mask & wall_c)),
                "unknown_fraction": round(unknown_fraction, 6),
                "free_fraction": round(free_fraction, 6),
                "wall_overlap_fraction": round(wall_overlap_fraction, 6),
                "label10_fraction": round(label10_fraction, 6),
                "min_width_cells": min_width_cells,
                "width_m": width_m,
            }
            status, confidence, reason = classify_candidate(
                (room_a, room_b),
                validation,
                width_m,
                component_cell_count,
                free_fraction,
                unknown_fraction,
                wall_overlap_fraction,
                label10_fraction,
            )
            validation["status"] = status
            center_r_local = float(np.mean(comp[:, 0]))
            center_c_local = float(np.mean(comp[:, 1]))
            center_r = center_r_local + row_offset
            center_c = center_c_local + col_offset
            approach_a_r = center_r - unit_row * approach_cells
            approach_a_c = center_c - unit_col * approach_cells
            approach_b_r = center_r + unit_row * approach_cells
            approach_b_c = center_c + unit_col * approach_cells
            debug_bbox_local, runs = rle_encode_component(comp)
            candidate = {
                "gateway_id": f"gw_{SHORT_SCENE_ID}_r{room_a}_r{room_b}_{raw_index:02d}",
                "room_a": room_a,
                "room_b": room_b,
                "candidate_index": raw_index,
                "status": status,
                "selected_for_topology": False,
                "primary_for_room_pair": False,
                "center": pose_from_row_col(center_r, center_c, yaw),
                "crossing_pose": pose_from_row_col(center_r, center_c, yaw),
                "approach_from_room_a": pose_from_row_col(approach_a_r, approach_a_c, yaw),
                "approach_from_room_b": pose_from_row_col(approach_b_r, approach_b_c, yaw),
                "width_m": width_m,
                "clearance_m": width_m,
                "component_area_m2": component_area_m2,
                "component_cell_count": component_cell_count,
                "bbox_cells": bbox_cells,
                "bbox_map_xy": bbox_map_xy(bbox_cells),
                "boundary_dilation_radius_m": round(float(radius_m_by_cells[radius_cells]), 3),
                "source_layers": list(SOURCE_LAYERS),
                "confidence": round(float(confidence), 6),
                "validation": validation,
                "warning_or_rejection_reason": reason,
                "debug_component_rle": {
                    "bbox_cells": bbox_cells,
                    "local_bbox_within_pair_crop": debug_bbox_local,
                    "pair_crop_row_offset": row_offset,
                    "pair_crop_col_offset": col_offset,
                    "runs_row_offset_col_offset_length": runs,
                },
                "debug_reachable_cell_count": int(reachable_cells),
            }
            candidates.append(candidate)

    pair_summary = {
        "room_a": room_a,
        "room_b": room_b,
        "near_boundary_detected": near_boundary_detected,
        "evaluated_boundary_radii_m": [0.15, 0.25, 0.40, 0.60],
        "boundary_band_cell_counts": {
            str(radius_m_by_cells[radius]): int(
                np.count_nonzero(build_boundary_band(room_a_mask, room_b_mask, radius, outside_c))
            )
            for radius in radius_cells_sequence
        },
        "notes": "room_mask_global_id_pair_evaluated",
    }
    return candidates, pair_summary


def select_gateways(pair_to_candidates: Dict[Tuple[int, int], List[Dict[str, Any]]]) -> None:
    for pair, candidates in pair_to_candidates.items():
        valid = [c for c in candidates if c["status"] == "valid"]
        valid.sort(key=lambda c: (float(c["confidence"]), int(c["component_cell_count"]), float(c["width_m"])), reverse=True)
        if not valid:
            continue
        pair_norm = normalize_pair(*pair)
        best = valid[0]
        if pair_norm == normalize_pair(3, 11) and float(best["confidence"]) < 0.88:
            best["status"] = "misleading"
            best["selected_for_topology"] = False
            best["primary_for_room_pair"] = False
            best["warning_or_rejection_reason"] = "room3_room11_no_strong_direct_gateway_evidence"
            best["validation"]["status"] = "misleading"
            continue
        best["selected_for_topology"] = True
        best["primary_for_room_pair"] = True
        if pair_norm == normalize_pair(11, 7):
            for extra in valid[1:]:
                extra["selected_for_topology"] = False
                extra["primary_for_room_pair"] = False
                if float(extra["confidence"]) < 0.82:
                    extra["status"] = "misleading"
                    extra["validation"]["status"] = "misleading"
                    extra["warning_or_rejection_reason"] = "room11_room7_extra_candidate_not_selected_primary_and_likely_misleading"


def summarize_pairs(
    evaluated_pairs: Sequence[Tuple[int, int]],
    pair_to_candidates: Dict[Tuple[int, int], List[Dict[str, Any]]],
    base_pair_summaries: Dict[Tuple[int, int], Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    adjacency: List[Dict[str, Any]] = []
    rejected_edges: List[Dict[str, Any]] = []
    for pair in evaluated_pairs:
        room_a, room_b = normalize_pair(*pair)
        candidates = pair_to_candidates.get((room_a, room_b), [])
        counts = {status: sum(1 for c in candidates if c["status"] == status) for status in STATUS_VALUES}
        selected_ids = [c["gateway_id"] for c in candidates if c.get("selected_for_topology")]
        raw_count = len(candidates)
        base = base_pair_summaries.get((room_a, room_b), {})
        notes = base.get("notes", "")
        if normalize_pair(room_a, room_b) == normalize_pair(3, 11):
            notes += "; explicit suspected false direct connectivity pair"
        if normalize_pair(room_a, room_b) == normalize_pair(11, 7):
            notes += "; at most one primary gateway selected unless evidence proves multiple"
        adjacency.append(
            {
                "room_a": room_a,
                "room_b": room_b,
                "near_boundary_detected": bool(base.get("near_boundary_detected", False)),
                "raw_candidate_count": raw_count,
                "valid_candidate_count": counts["valid"],
                "ambiguous_candidate_count": counts["ambiguous"],
                "misleading_candidate_count": counts["misleading"],
                "rejected_candidate_count": counts["rejected"],
                "selected_gateway_ids": selected_ids,
                "notes": notes.strip("; "),
            }
        )
        if not selected_ids:
            if raw_count == 0:
                reason = "no_valid_gateway_component"
            elif counts["valid"] == 0:
                reason = "no_valid_gateway_component"
            else:
                reason = "valid_candidate_not_selected_due_to_selection_policy"
            rejected_edges.append(
                {
                    "room_a": room_a,
                    "room_b": room_b,
                    "reason": reason,
                    "raw_candidate_count": raw_count,
                    "rejected_candidate_count": counts["rejected"],
                    "ambiguous_candidate_count": counts["ambiguous"],
                    "misleading_candidate_count": counts["misleading"],
                    "debug_visualization_path": key_pair_visual_path(room_a, room_b),
                }
            )
    return adjacency, rejected_edges


def key_pair_visual_path(room_a: int, room_b: int) -> str:
    pair_norm = normalize_pair(room_a, room_b)
    mapping = {
        normalize_pair(3, 11): f"{SHORT_SCENE_ID}_step29b2_room3_room11_gateway_debug.png",
        normalize_pair(11, 7): f"{SHORT_SCENE_ID}_step29b2_room11_room7_gateway_debug.png",
        normalize_pair(3, 7): f"{SHORT_SCENE_ID}_step29b2_room3_room7_gateway_debug.png",
        normalize_pair(11, 8): f"{SHORT_SCENE_ID}_step29b2_room11_room8_gateway_debug.png",
    }
    return rel(VIS_DIR / mapping.get(pair_norm, f"{SHORT_SCENE_ID}_step29b2_gateway_candidates_overview.png"))


def public_path_readiness(pair_to_candidates: Dict[Tuple[int, int], List[Dict[str, Any]]]) -> Dict[str, Any]:
    transitions: List[Dict[str, Any]] = []
    all_ready = True
    for room_a, room_b in PUBLIC_PATH_TRANSITIONS:
        pair = normalize_pair(room_a, room_b)
        selected = [c for c in pair_to_candidates.get(pair, []) if c.get("selected_for_topology")]
        exists = bool(selected)
        if not exists:
            all_ready = False
        transitions.append(
            {
                "room_a": room_a,
                "room_b": room_b,
                "selected_gateway_exists": exists,
                "selected_gateway_id": selected[0]["gateway_id"] if selected else None,
                "status": selected[0]["status"] if selected else "missing",
                "reason_if_missing": None if selected else "no_selected_valid_gateway_evidence",
            }
        )
    return {
        "route_name": "public_path_042",
        "route": PUBLIC_PATH_042,
        "transitions": transitions,
        "route_gateway_ready": bool(all_ready),
    }


def extract_gateway_candidates() -> Dict[str, Any]:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    layers = load_layers()
    data = layers["npz"]
    global_room = data["room_mask_global_id"].astype(np.int32)
    global_room_npy = layers["global_room_mask_npy"].astype(np.int32)
    if global_room.shape != global_room_npy.shape or not np.array_equal(global_room, global_room_npy):
        raise ValueError("Step29B1 room_mask_global_id does not match 00824_global_room_mask_v0_1.npy")
    wall = data["structural_wall"] > 0
    free = data["free_space"] > 0
    outside = data["outside_boundary"] > 0
    unknown = data["unknown_layer"] > 0
    local_repaired = data["room_mask_local_label_repaired"].astype(np.int32)
    wall_room_overlap = wall & (global_room > 0)

    audit = audit_label10_and_wall_overlap(layers)
    write_json(LABEL10_WALL_AUDIT_JSON, audit)

    evaluated_pairs = discover_room_pairs(global_room, outside)
    pair_to_candidates: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    base_pair_summaries: Dict[Tuple[int, int], Dict[str, Any]] = {}
    for pair in evaluated_pairs:
        candidates, pair_summary = extract_pair_candidates(
            pair,
            global_room,
            wall,
            free,
            outside,
            unknown,
            local_repaired,
            wall_room_overlap,
        )
        norm = normalize_pair(*pair)
        pair_to_candidates[norm] = candidates
        base_pair_summaries[norm] = pair_summary
        print(f"Evaluated {display_pair(*norm)}: {len(candidates)} raw candidates")

    select_gateways(pair_to_candidates)
    all_candidates: List[Dict[str, Any]] = []
    for pair in evaluated_pairs:
        all_candidates.extend(pair_to_candidates.get(normalize_pair(*pair), []))

    adjacency, rejected_edges = summarize_pairs(evaluated_pairs, pair_to_candidates, base_pair_summaries)
    readiness = public_path_readiness(pair_to_candidates)
    selected_gateways = [c["gateway_id"] for c in all_candidates if c.get("selected_for_topology")]
    room_ids_present = sorted(int(v) for v in np.unique(global_room) if int(v) > 0)

    source_artifacts = {
        "layered_bev_json": rel(STEP29B1_LAYERED_JSON),
        "layered_bev_npz": rel(STEP29B1_LAYERED_NPZ),
        "global_room_mask": rel(STEP29B1_GLOBAL_ROOM_MASK_NPY),
        "room_label_mapping": rel(STEP29B1_MAPPING_JSON),
        "summary": rel(STEP29B1_SUMMARY_JSON),
        "validation_results": rel(STEP29B1_VALIDATION_JSON),
    }
    provenance = {
        "room_identity_source": "room_mask_global_id",
        "local_labels_used_as_room_ids": False,
        "local_label_usage": "room_mask_local_label_repaired was used only for the unmapped label 10 audit.",
        "room_polygons_used": False,
        "old_bev_used": False,
        "topology_json_used_as_geometry": False,
        "nav2_run": False,
        "gazebo_run": False,
        "ros_execution_run": False,
        "amcl_tf_dwb_run": False,
        "forbidden_geometry_sources_not_used": FORBIDDEN_GEOMETRY_SOURCES_NOT_USED,
    }
    candidates_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2_gateway_candidates",
        "version": VERSION,
        "map_frame": MAP_FRAME,
        "resolution": RESOLUTION,
        "origin": ORIGIN,
        "source_step29b1_artifacts": source_artifacts,
        "extraction_policy": provenance,
        "thresholds": {
            "structural_wall_boolean": "structural_wall > 0",
            "free_space_boolean": "free_space > 0",
            "outside_boundary_boolean": "outside_boundary > 0",
            "unknown_layer_boolean": "unknown_layer > 0",
            "interior_erosion_radius_m": 0.15,
            "boundary_dilation_radii_m": [0.15, 0.25, 0.40, 0.60],
            "free_near_radius_m": 0.10,
            "minimum_valid_width_m": 0.35,
        },
        "gateway_candidates": all_candidates,
    }
    graph_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2_gateway_graph_candidate",
        "version": VERSION,
        "source_step29b1_artifacts": source_artifacts,
        "map_frame": MAP_FRAME,
        "resolution": RESOLUTION,
        "origin": ORIGIN,
        "room_ids_present": room_ids_present,
        "gateway_candidates": all_candidates,
        "selected_gateways": selected_gateways,
        "rejected_room_edges": rejected_edges,
        "room_pair_summaries": adjacency,
        "public_path_042_gateway_readiness": readiness,
        "known_limitations": [
            "Gateway evidence is extracted from Stage-A raster layers only; no Nav2 or Gazebo validation was run.",
            "No topology augmentation artifact is produced in Step29B2; Step29C owns that work.",
            "Object obstacles are not inferred because object_obstacle_placeholder is empty.",
            "Wall-room overlap cells are audited and penalized, not treated as reliable traversable space.",
        ],
        "extraction_policy": provenance,
    }
    summary_counts = {status: sum(1 for c in all_candidates if c["status"] == status) for status in STATUS_VALUES}
    summary_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2_gateway_extraction_summary",
        "version": VERSION,
        "output_directory": rel(STEP29B2_ROOT),
        "gateway_candidates_json": rel(GATEWAY_CANDIDATES_JSON),
        "gateway_graph_json": rel(GATEWAY_GRAPH_JSON),
        "rejected_room_edges_json": rel(REJECTED_ROOM_EDGES_JSON),
        "room_pair_adjacency_candidates_json": rel(ROOM_PAIR_ADJACENCY_JSON),
        "label10_and_wall_overlap_audit_json": rel(LABEL10_WALL_AUDIT_JSON),
        "validation_results_json": rel(VALIDATION_RESULTS_JSON),
        "readme_path": rel(README_PATH),
        "visualization_directory": rel(VIS_DIR),
        "source_step29b1_artifacts": source_artifacts,
        "room_ids_present": room_ids_present,
        "room_pairs_evaluated": [{"room_a": a, "room_b": b} for a, b in evaluated_pairs],
        "raw_candidate_count": len(all_candidates),
        "valid_candidate_count": summary_counts["valid"],
        "ambiguous_candidate_count": summary_counts["ambiguous"],
        "misleading_candidate_count": summary_counts["misleading"],
        "rejected_candidate_count": summary_counts["rejected"],
        "selected_gateway_count": len(selected_gateways),
        "selected_gateway_ids": selected_gateways,
        "public_path_042_gateway_readiness": readiness,
        "label10_audit_result": audit["label10_audit"]["interpretation"],
        "wall_room_overlap_audit_result": audit["wall_room_overlap_audit"],
        "step29c_topology_augmentation_can_proceed": bool(readiness["route_gateway_ready"] and len(selected_gateways) > 0),
        "step29c_note": "Proceed only if downstream owner accepts the selected Step29B2 gateway evidence; no topology artifact was created here.",
        "extraction_policy": provenance,
    }

    write_json(GATEWAY_CANDIDATES_JSON, candidates_payload)
    write_json(GATEWAY_GRAPH_JSON, graph_payload)
    write_json(EXTRACTION_SUMMARY_JSON, summary_payload)
    write_json(ROOM_PAIR_ADJACENCY_JSON, {"scene_id": SCENE_ID, "artifact_type": "step29b2_room_pair_adjacency_candidates", "version": VERSION, "room_pair_adjacency_candidates": adjacency})
    write_json(REJECTED_ROOM_EDGES_JSON, {"scene_id": SCENE_ID, "artifact_type": "step29b2_rejected_room_edges", "version": VERSION, "rejected_room_edges": rejected_edges})
    return summary_payload


def main() -> None:
    summary = extract_gateway_candidates()
    print("\nStep29B2 extraction summary")
    print(f"Output directory: {summary['output_directory']}")
    print(f"Raw candidates: {summary['raw_candidate_count']}")
    print(f"Valid/Ambiguous/Misleading/Rejected: {summary['valid_candidate_count']}/{summary['ambiguous_candidate_count']}/{summary['misleading_candidate_count']}/{summary['rejected_candidate_count']}")
    print(f"Selected gateways: {summary['selected_gateway_count']}")
    print(f"public_path_042 ready: {summary['public_path_042_gateway_readiness']['route_gateway_ready']}")


if __name__ == "__main__":
    main()
