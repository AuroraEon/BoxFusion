#!/usr/bin/env python3
"""Step30E offline navigation projection map and planner-map audit.

Consumes frozen Step30A/C/D artifacts for scene 00824-Dd4bFSTQ8gi and builds
offline map candidates only. This script does not import ROS libraries, does
not launch ROS/Nav2/Gazebo/RViz, does not call actions, and does not produce a
live navigation path.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import zlib
from collections import Counter, deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30e"
VERSION = "v0_1"

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
STEP30A_ROOT = RUNTIME_ROOT / "step30a_00824_full_stage_a_dual_wall_gateway_rerun" / "generated" / "assets"
STEP30C_ROOT = RUNTIME_ROOT / "step30c_00824_gateway_augmented_topology_candidate"
STEP30D_ROOT = RUNTIME_ROOT / "step30d_00824_topology_route_projection_overlay_review"
OUTPUT_ROOT = RUNTIME_ROOT / "step30e_00824_nav_projection_map_and_planner_audit"
VIZ_ROOT = OUTPUT_ROOT / "visualizations"
LOCAL_CROP_ROOT = VIZ_ROOT / "local_gateway_crops"

STEP30A_LAYERED_BEV_NPZ = STEP30A_ROOT / f"{SHORT}_step30a_layered_bev_{VERSION}.npz"
STEP30A_LAYERED_BEV_JSON = STEP30A_ROOT / f"{SHORT}_step30a_layered_bev_{VERSION}.json"

STEP30C_EDGE_TABLE = STEP30C_ROOT / f"{SHORT}_step30c_gateway_edge_table_{VERSION}.json"
STEP30C_ROUTES = STEP30C_ROOT / f"{SHORT}_step30c_topology_route_candidates_{VERSION}.json"

STEP30D_ROOM_ANCHORS = STEP30D_ROOT / f"{SHORT}_step30d_room_anchor_validation_{VERSION}.json"
STEP30D_GATEWAY_POSES = STEP30D_ROOT / f"{SHORT}_step30d_gateway_pose_validation_{VERSION}.json"
STEP30D_ROUTE_REVIEW = STEP30D_ROOT / f"{SHORT}_step30d_route_projection_review_{VERSION}.json"
STEP30D_OVERLAY = STEP30D_ROOT / f"{SHORT}_step30d_corrected_overlay_payload_{VERSION}.json"
STEP30D_VALIDATION = STEP30D_ROOT / f"{SHORT}_step30d_validation_results_{VERSION}.json"
STEP30D_SUMMARY = STEP30D_ROOT / f"{SHORT}_step30d_summary_{VERSION}.json"

EXPECTED_GATEWAY_PAIRS = [
    "r1_r3",
    "r3_r7",
    "r3_r8",
    "r7_r11",
    "r7_r14",
    "r7_r15",
    "r8_r11",
    "r14_r16",
]
REQUIRED_ROUTES = ["public_path_042", "long_structure_route_default"]
OPTIONAL_ROUTES = ["long_structure_review_all_positive_gateways"]
CARVE_RADII_M = [0.15, 0.18, 0.25, 0.35]
DEFAULT_CARVE_RADIUS_M = 0.18
SAMPLING_RESOLUTION_FACTOR = 0.5
OCC_FREE = 0
OCC_OCCUPIED = 100
OCC_UNKNOWN = -1


FONT_5X7 = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01110"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "10010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "01010", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "01010", "00100", "00100", "00100", "01010", "10001"],
    "Y": ["10001", "01010", "00100", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01111", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "11110"],
    "_": ["00000", "00000", "00000", "00000", "00000", "00000", "11111"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    ":": ["00000", "00100", "00100", "00000", "00100", "00100", "00000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "|": ["00100", "00100", "00100", "00100", "00100", "00100", "00100"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")
    print(f"Written: {rel(path)}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hex_to_rgb(color: str) -> np.ndarray:
    color = color.lstrip("#")
    return np.asarray([int(color[i : i + 2], 16) for i in (0, 2, 4)], dtype=np.uint8)


def write_png(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    height, width = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def pose_xy(pose: Dict[str, Any]) -> List[float]:
    return [float(pose["x"]), float(pose["y"])]


def finite_xy(xy: Optional[Sequence[float]]) -> bool:
    return xy is not None and len(xy) >= 2 and all(math.isfinite(float(v)) for v in xy[:2])


def finite_pose(pose: Optional[Dict[str, Any]]) -> bool:
    return bool(pose) and all(math.isfinite(float(pose.get(k, math.nan))) for k in ("x", "y", "yaw"))


def room_number(room_id: Any) -> int:
    text = str(room_id)
    if text.startswith("room_"):
        return int(text.split("_", 1)[1])
    if text.startswith("r"):
        return int(text[1:])
    return int(text)


def pair_key_for(room_a: int, room_b: int) -> str:
    a, b = sorted((int(room_a), int(room_b)))
    return f"r{a}_r{b}"


class BevContext:
    def __init__(self, npz_path: Path, metadata_path: Path) -> None:
        if not npz_path.exists():
            raise FileNotFoundError(npz_path)
        self.npz_path = npz_path
        self.metadata_path = metadata_path
        self.layers = {key: value for key, value in np.load(npz_path).items()}
        self.metadata = read_json(metadata_path) if metadata_path.exists() else {}
        first = next(iter(self.layers.values()))
        self.height, self.width = first.shape[:2]
        self.resolution = float(self.metadata.get("resolution", 0.05))
        self.origin = [float(v) for v in self.metadata.get("origin", [-50.0, -50.0])]
        self.room_pixels: Dict[int, np.ndarray] = {}
        room_mask = self.layers.get("room_mask_global_id")
        if room_mask is not None:
            for rid in sorted(int(v) for v in np.unique(room_mask) if int(v) > 0):
                self.room_pixels[rid] = np.argwhere(room_mask == rid)

    def has_layer(self, name: str) -> bool:
        return name in self.layers

    def xy_to_rc(self, xy: Sequence[float]) -> Tuple[int, int]:
        col = int(round((float(xy[0]) - self.origin[0]) / self.resolution))
        row = int(round((float(xy[1]) - self.origin[1]) / self.resolution))
        return row, col

    def rc_to_xy(self, row: float, col: float) -> List[float]:
        return [
            round(self.origin[0] + float(col) * self.resolution, 4),
            round(self.origin[1] + float(row) * self.resolution, 4),
        ]

    def in_bounds_rc(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def in_bounds_xy(self, xy: Sequence[float]) -> bool:
        return self.in_bounds_rc(*self.xy_to_rc(xy))

    def value_at(self, layer: str, xy: Sequence[float]) -> Optional[int]:
        arr = self.layers.get(layer)
        if arr is None:
            return None
        row, col = self.xy_to_rc(xy)
        if not self.in_bounds_rc(row, col):
            return None
        return int(arr[row, col])

    def distance_to_room_m(self, room_id: int, xy: Sequence[float]) -> Optional[float]:
        pixels = self.room_pixels.get(int(room_id))
        if pixels is None or len(pixels) == 0 or not self.in_bounds_xy(xy):
            return None
        row, col = self.xy_to_rc(xy)
        d2 = (pixels[:, 0].astype(float) - row) ** 2 + (pixels[:, 1].astype(float) - col) ** 2
        return round(float(math.sqrt(float(np.min(d2))) * self.resolution), 4)


def layer_bool(bev: BevContext, name: str) -> np.ndarray:
    arr = bev.layers.get(name)
    if arr is None:
        return np.zeros((bev.height, bev.width), dtype=bool)
    return arr > 0


def build_occupancy(bev: BevContext, wall_layer: str) -> np.ndarray:
    occ = np.full((bev.height, bev.width), OCC_UNKNOWN, dtype=np.int16)
    free = layer_bool(bev, "free_space")
    unknown = layer_bool(bev, "unknown_layer") & ~free
    outside = layer_bool(bev, "outside_boundary")
    wall = layer_bool(bev, wall_layer)
    occ[free] = OCC_FREE
    occ[unknown] = OCC_UNKNOWN
    occ[outside | wall] = OCC_OCCUPIED
    return occ


def line_samples(p0: Sequence[float], p1: Sequence[float], step_m: float) -> List[List[float]]:
    length = float(np.linalg.norm(np.subtract(p1, p0)))
    count = max(2, int(math.ceil(length / max(step_m, 1e-6))) + 1)
    return [
        [float(p0[0]) + (float(p1[0]) - float(p0[0])) * i / (count - 1), float(p0[1]) + (float(p1[1]) - float(p0[1])) * i / (count - 1)]
        for i in range(count)
    ]


def capsule_cells(bev: BevContext, p0: Sequence[float], p1: Sequence[float], radius_m: float) -> List[Tuple[int, int]]:
    cells = set()
    radius_px = max(1, int(math.ceil(radius_m / bev.resolution)))
    for xy in line_samples(p0, p1, bev.resolution * SAMPLING_RESOLUTION_FACTOR):
        row, col = bev.xy_to_rc(xy)
        for rr in range(row - radius_px, row + radius_px + 1):
            for cc in range(col - radius_px, col + radius_px + 1):
                if not bev.in_bounds_rc(rr, cc):
                    continue
                if (rr - row) ** 2 + (cc - col) ** 2 <= radius_px**2:
                    cells.add((rr, cc))
    return sorted(cells)


def carve_gateway(occ: np.ndarray, bev: BevContext, gateway: Dict[str, Any], radius_m: float) -> Dict[str, Any]:
    crossing = pose_xy(gateway["crossing_pose"])
    app_a = pose_xy(gateway["approach_from_room_a"])
    app_b = pose_xy(gateway["approach_from_room_b"])
    cells = set(capsule_cells(bev, app_a, crossing, radius_m))
    cells.update(capsule_cells(bev, crossing, app_b, radius_m))
    for row, col in cells:
        occ[row, col] = OCC_FREE
    return {
        "gateway_id": gateway["gateway_id"],
        "pair_key": gateway["pair_key"],
        "radius_m": radius_m,
        "approach_a_xy": app_a,
        "crossing_xy": crossing,
        "approach_b_xy": app_b,
        "carved_cell_count": len(cells),
    }


def build_gateway_aware_map(bev: BevContext, gateways: List[Dict[str, Any]], radius_m: float) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
    occ = build_occupancy(bev, "gateway_wall_preclose")
    carved = []
    for gateway in gateways:
        if gateway["pair_key"] == "r3_r11":
            continue
        if all(finite_pose(gateway.get(key)) for key in ("crossing_pose", "approach_from_room_a", "approach_from_room_b")):
            carved.append(carve_gateway(occ, bev, gateway, radius_m))
    return occ, carved


def pose_occ_status(bev: BevContext, occ: np.ndarray, xy: Sequence[float]) -> str:
    row, col = bev.xy_to_rc(xy)
    if not bev.in_bounds_rc(row, col):
        return "outside"
    value = int(occ[row, col])
    if value == OCC_FREE:
        return "free"
    if value == OCC_OCCUPIED:
        if bev.value_at("outside_boundary", xy) == 1:
            return "outside"
        return "occupied"
    return "unknown"


def nearest_positive_distance_m(bev: BevContext, mask: np.ndarray, xy: Sequence[float], max_radius_m: float = 4.0) -> Optional[float]:
    if not bev.in_bounds_xy(xy):
        return None
    row, col = bev.xy_to_rc(xy)
    if bool(mask[row, col]):
        return 0.0
    max_radius_px = max(1, int(math.ceil(max_radius_m / bev.resolution)))
    for radius in range(1, max_radius_px + 1):
        r0, r1 = max(0, row - radius), min(bev.height, row + radius + 1)
        c0, c1 = max(0, col - radius), min(bev.width, col + radius + 1)
        patch = mask[r0:r1, c0:c1]
        if not np.any(patch):
            continue
        points = np.argwhere(patch)
        points[:, 0] += r0
        points[:, 1] += c0
        d2 = (points[:, 0].astype(float) - row) ** 2 + (points[:, 1].astype(float) - col) ** 2
        return round(float(math.sqrt(float(np.min(d2))) * bev.resolution), 4)
    return None


def connected_locally(bev: BevContext, occ: np.ndarray, p0: Sequence[float], p1: Sequence[float], margin_m: float = 0.55) -> bool:
    row0, col0 = bev.xy_to_rc(p0)
    row1, col1 = bev.xy_to_rc(p1)
    if not (bev.in_bounds_rc(row0, col0) and bev.in_bounds_rc(row1, col1)):
        return False
    if occ[row0, col0] != OCC_FREE or occ[row1, col1] != OCC_FREE:
        return False
    pad = max(4, int(math.ceil(margin_m / bev.resolution)))
    r_min, r_max = max(0, min(row0, row1) - pad), min(bev.height, max(row0, row1) + pad + 1)
    c_min, c_max = max(0, min(col0, col1) - pad), min(bev.width, max(col0, col1) + pad + 1)
    start = (row0 - r_min, col0 - c_min)
    goal = (row1 - r_min, col1 - c_min)
    free = occ[r_min:r_max, c_min:c_max] == OCC_FREE
    q: deque[Tuple[int, int]] = deque([start])
    seen = {start}
    while q:
        row, col = q.popleft()
        if (row, col) == goal:
            return True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nr, nc = row + dr, col + dc
            if 0 <= nr < free.shape[0] and 0 <= nc < free.shape[1] and free[nr, nc] and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append((nr, nc))
    return False


def line_intersections(bev: BevContext, occ: np.ndarray, p0: Sequence[float], p1: Sequence[float]) -> Dict[str, Any]:
    samples = line_samples(p0, p1, bev.resolution * SAMPLING_RESOLUTION_FACTOR)
    occupied = 0
    unknown = 0
    outside = 0
    statuses = []
    for xy in samples:
        status = pose_occ_status(bev, occ, xy)
        statuses.append(status)
        if status == "occupied":
            occupied += 1
        elif status == "unknown":
            unknown += 1
        elif status == "outside":
            outside += 1
    return {
        "sample_count": len(samples),
        "obstacle_intersection_count": occupied + outside,
        "unknown_intersection_count": unknown,
        "outside_intersection_count": outside,
        "clear": occupied == 0 and unknown == 0 and outside == 0,
    }


def audit_triplet(bev: BevContext, occ: np.ndarray, gateway: Dict[str, Any]) -> Dict[str, Any]:
    app_a = pose_xy(gateway["approach_from_room_a"])
    crossing = pose_xy(gateway["crossing_pose"])
    app_b = pose_xy(gateway["approach_from_room_b"])
    seg_a = line_intersections(bev, occ, app_a, crossing)
    seg_b = line_intersections(bev, occ, crossing, app_b)
    a_free = pose_occ_status(bev, occ, app_a) == "free"
    c_free = pose_occ_status(bev, occ, crossing) == "free"
    b_free = pose_occ_status(bev, occ, app_b) == "free"
    ac_connected = connected_locally(bev, occ, app_a, crossing)
    cb_connected = connected_locally(bev, occ, crossing, app_b)
    return {
        "approach_a_free": a_free,
        "crossing_free": c_free,
        "approach_b_free": b_free,
        "approach_a_connected_to_crossing_locally": ac_connected,
        "crossing_connected_to_approach_b_locally": cb_connected,
        "locally_connected": a_free and c_free and b_free and ac_connected and cb_connected,
        "approach_a_to_crossing_m": round(float(np.linalg.norm(np.subtract(app_a, crossing))), 4),
        "crossing_to_approach_b_m": round(float(np.linalg.norm(np.subtract(crossing, app_b))), 4),
        "approach_a_to_crossing_line_audit": seg_a,
        "crossing_to_approach_b_line_audit": seg_b,
        "total_obstacle_intersection_count": seg_a["obstacle_intersection_count"] + seg_b["obstacle_intersection_count"],
        "total_unknown_intersection_count": seg_a["unknown_intersection_count"] + seg_b["unknown_intersection_count"],
    }


def minimum_required_radius(bev: BevContext, base_occ: np.ndarray, gateway: Dict[str, Any]) -> Optional[float]:
    if audit_triplet(bev, base_occ, gateway)["locally_connected"]:
        return 0.0
    for radius in CARVE_RADII_M:
        test = np.array(base_occ, copy=True)
        carve_gateway(test, bev, gateway, radius)
        if audit_triplet(bev, test, gateway)["locally_connected"]:
            return radius
    return None


def nearest_target_room(bev: BevContext, room_a: int, room_b: int, xy: Sequence[float]) -> Dict[str, Any]:
    d_a = bev.distance_to_room_m(room_a, xy)
    d_b = bev.distance_to_room_m(room_b, xy)
    nearest = None
    if d_a is not None and d_b is not None:
        nearest = room_a if d_a <= d_b else room_b
    return {"room_a_distance_m": d_a, "room_b_distance_m": d_b, "nearest_target_room_id": nearest}


def pose_audit(
    bev: BevContext,
    maps: Dict[str, np.ndarray],
    gateway: Dict[str, Any],
    pose_name: str,
    xy: Sequence[float],
) -> Dict[str, Any]:
    room_a = int(gateway["room_a"])
    room_b = int(gateway["room_b"])
    obstacle_masks = {
        "segmentation_blocked_reference_map": layer_bool(bev, "segmentation_wall_processed") | layer_bool(bev, "outside_boundary"),
        "gateway_preclose_reference_map": layer_bool(bev, "gateway_wall_preclose") | layer_bool(bev, "outside_boundary"),
        "gateway_aware_nav_projection_map": maps["gateway_aware_nav_projection_map"] == OCC_OCCUPIED,
    }
    statuses = {}
    for map_name, occ in maps.items():
        obstacle_mask = obstacle_masks[map_name]
        statuses[map_name] = {
            "occupancy_status": pose_occ_status(bev, occ, xy),
            "distance_to_nearest_obstacle_m": nearest_positive_distance_m(bev, obstacle_mask, xy),
        }
    room_here = bev.value_at("room_mask_global_id", xy) if bev.has_layer("room_mask_global_id") else None
    distances = nearest_target_room(bev, room_a, room_b, xy)
    expected_room = None
    if pose_name == "approach_from_room_a":
        expected_room = room_a
    elif pose_name == "approach_from_room_b":
        expected_room = room_b
    expected_side = None
    if expected_room is not None and distances["room_a_distance_m"] is not None and distances["room_b_distance_m"] is not None:
        expected_side = distances["nearest_target_room_id"] == expected_room
    crossing_near_both = None
    if pose_name in {"center_xy", "crossing_pose"} and distances["room_a_distance_m"] is not None and distances["room_b_distance_m"] is not None:
        crossing_near_both = distances["room_a_distance_m"] <= 0.65 and distances["room_b_distance_m"] <= 0.65
    return {
        "pose_name": pose_name,
        "xy": [round(float(xy[0]), 4), round(float(xy[1]), 4)],
        "in_bounds": bev.in_bounds_xy(xy),
        "room_mask_global_id_at_pose": room_here,
        "nearest_target_room": distances,
        "approach_pose_on_expected_room_side": expected_side,
        "crossing_pose_near_both_target_rooms": crossing_near_both,
        "inside_segmentation_wall_processed": bev.value_at("segmentation_wall_processed", xy) == 1,
        "inside_gateway_wall_preclose": bev.value_at("gateway_wall_preclose", xy) == 1,
        "distance_to_unknown_m": nearest_positive_distance_m(bev, layer_bool(bev, "unknown_layer"), xy),
        "distance_to_outside_boundary_m": nearest_positive_distance_m(bev, layer_bool(bev, "outside_boundary"), xy),
        "map_candidate_status": statuses,
        "free_after_gateway_aware_carving": statuses["gateway_aware_nav_projection_map"]["occupancy_status"] == "free",
    }


def compact_rle(arr: np.ndarray) -> Dict[str, Any]:
    flat = arr.astype(np.int16, copy=False).ravel()
    runs: List[List[int]] = []
    if flat.size:
        current = int(flat[0])
        count = 1
        for value in flat[1:]:
            value_int = int(value)
            if value_int == current:
                count += 1
            else:
                runs.append([current, count])
                current = value_int
                count = 1
        runs.append([current, count])
    return {"encoding": "row_major_value_count_rle", "runs": runs}


class Canvas:
    def __init__(
        self,
        bev: BevContext,
        title: str,
        xlim: Tuple[float, float],
        ylim: Tuple[float, float],
        nav_occ: Optional[np.ndarray] = None,
        width: int = 1500,
        height: int = 1100,
    ) -> None:
        self.bev = bev
        self.title = title
        self.xlim = xlim
        self.ylim = ylim
        self.width = width
        self.height = height
        self.nav_occ = nav_occ
        self.rgb = self._background()
        self.draw_text_px(16, 16, title, "#111111", scale=2, background="#ffffff")

    def _background(self) -> np.ndarray:
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[:] = hex_to_rgb("#f4f4ef")
        xs = np.linspace(self.xlim[0], self.xlim[1], self.width)
        ys = np.linspace(self.ylim[1], self.ylim[0], self.height)
        cols = np.rint((xs - self.bev.origin[0]) / self.bev.resolution).astype(int)
        rows = np.rint((ys - self.bev.origin[1]) / self.bev.resolution).astype(int)
        cc, rr = np.meshgrid(cols, rows)
        valid = (rr >= 0) & (rr < self.bev.height) & (cc >= 0) & (cc < self.bev.width)
        rr = np.clip(rr, 0, self.bev.height - 1)
        cc = np.clip(cc, 0, self.bev.width - 1)
        if self.bev.has_layer("unknown_layer"):
            rgb[(self.bev.layers["unknown_layer"][rr, cc] > 0) & valid] = hex_to_rgb("#dddddd")
        if self.bev.has_layer("free_space"):
            rgb[(self.bev.layers["free_space"][rr, cc] > 0) & valid] = hex_to_rgb("#ffffff")
        if self.bev.has_layer("room_mask_global_id"):
            room_mask = self.bev.layers["room_mask_global_id"][rr, cc]
            colors = {1: "#b6def4", 3: "#cbe8b4", 7: "#f8c2bd", 8: "#d8c2f0", 11: "#f6dfa3", 14: "#b8e6dc", 15: "#ecc4da", 16: "#ddddb3"}
            for rid, color in colors.items():
                mask = (room_mask == rid) & valid
                if np.any(mask):
                    rgb[mask] = (rgb[mask].astype(float) * 0.52 + hex_to_rgb(color).astype(float) * 0.48).astype(np.uint8)
        if self.nav_occ is not None:
            occ = self.nav_occ[rr, cc]
            rgb[(occ == OCC_UNKNOWN) & valid] = (rgb[(occ == OCC_UNKNOWN) & valid].astype(float) * 0.55 + hex_to_rgb("#bdbdbd").astype(float) * 0.45).astype(np.uint8)
            rgb[(occ == OCC_OCCUPIED) & valid] = hex_to_rgb("#242424")
        else:
            if self.bev.has_layer("segmentation_wall_processed"):
                rgb[(self.bev.layers["segmentation_wall_processed"][rr, cc] > 0) & valid] = hex_to_rgb("#222222")
            if self.bev.has_layer("gateway_wall_preclose"):
                mask = (self.bev.layers["gateway_wall_preclose"][rr, cc] > 0) & valid
                rgb[mask] = (rgb[mask].astype(float) * 0.3 + hex_to_rgb("#315dd6").astype(float) * 0.7).astype(np.uint8)
        return rgb

    def world_to_px(self, xy: Sequence[float]) -> Tuple[int, int]:
        px = int(round((float(xy[0]) - self.xlim[0]) / (self.xlim[1] - self.xlim[0]) * (self.width - 1)))
        py = int(round((self.ylim[1] - float(xy[1])) / (self.ylim[1] - self.ylim[0]) * (self.height - 1)))
        return px, py

    def blend_px(self, x: int, y: int, color: str, alpha: float = 1.0) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            c = hex_to_rgb(color).astype(float)
            self.rgb[y, x] = np.clip(self.rgb[y, x].astype(float) * (1.0 - alpha) + c * alpha, 0, 255).astype(np.uint8)

    def draw_line_px(self, p0: Tuple[int, int], p1: Tuple[int, int], color: str, width: int = 1, alpha: float = 1.0) -> None:
        steps = max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]), 1)
        radius = max(0, width // 2)
        for x_f, y_f in zip(np.linspace(p0[0], p1[0], steps + 1), np.linspace(p0[1], p1[1], steps + 1)):
            x, y = int(round(x_f)), int(round(y_f))
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy <= radius * radius + 1:
                        self.blend_px(x + dx, y + dy, color, alpha)

    def draw_line_world(self, p0: Sequence[float], p1: Sequence[float], color: str, width: int = 1, alpha: float = 1.0) -> None:
        self.draw_line_px(self.world_to_px(p0), self.world_to_px(p1), color, width, alpha)

    def draw_circle_world(self, xy: Sequence[float], radius_px: int, fill: str, outline: Optional[str] = None, alpha: float = 1.0) -> None:
        cx, cy = self.world_to_px(xy)
        for y in range(cy - radius_px, cy + radius_px + 1):
            for x in range(cx - radius_px, cx + radius_px + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius_px**2:
                    self.blend_px(x, y, fill, alpha)
        if outline:
            for angle in np.linspace(0, 2 * math.pi, 72):
                self.blend_px(int(round(cx + math.cos(angle) * radius_px)), int(round(cy + math.sin(angle) * radius_px)), outline, 1.0)

    def draw_cross_world(self, xy: Sequence[float], color: str, size: int = 8, width: int = 2, alpha: float = 1.0) -> None:
        x, y = self.world_to_px(xy)
        self.draw_line_px((x - size, y - size), (x + size, y + size), color, width, alpha)
        self.draw_line_px((x - size, y + size), (x + size, y - size), color, width, alpha)

    def draw_text_px(self, x: int, y: int, text: str, color: str, scale: int = 1, background: Optional[str] = None) -> None:
        lines = text.upper().split("\n")
        line_h = 8 * scale
        max_w = max((len(line) * 6 * scale for line in lines), default=0)
        if background:
            for yy in range(y - 3, y + line_h * len(lines) + 3):
                for xx in range(x - 3, x + max_w + 3):
                    self.blend_px(xx, yy, background, 0.78)
        for line_idx, line in enumerate(lines):
            cursor_x = x
            for ch in line:
                glyph = FONT_5X7.get(ch, FONT_5X7[" "])
                for gy, row in enumerate(glyph):
                    for gx, bit in enumerate(row):
                        if bit == "1":
                            for sy in range(scale):
                                for sx in range(scale):
                                    self.blend_px(cursor_x + gx * scale + sx, y + line_idx * line_h + gy * scale + sy, color, 1.0)
                cursor_x += 6 * scale

    def draw_text_world(self, xy: Sequence[float], text: str, color: str, dx: int = 8, dy: int = -8, scale: int = 1) -> None:
        x, y = self.world_to_px(xy)
        self.draw_text_px(x + dx, y + dy, text, color, scale=scale, background="#ffffff")

    def save(self, path: Path) -> None:
        write_png(path, self.rgb)


def gateway_color(status: str) -> str:
    return {"valid": "#168a51", "review": "#c96a00", "invalid": "#b00020"}.get(status, "#555555")


def draw_gateway(canvas: Canvas, gateway: Dict[str, Any], audit: Optional[Dict[str, Any]] = None, label: bool = True) -> None:
    app_a = pose_xy(gateway["approach_from_room_a"])
    crossing = pose_xy(gateway["crossing_pose"])
    app_b = pose_xy(gateway["approach_from_room_b"])
    status = audit.get("audit_status", "review") if audit else ("review" if gateway.get("review_required") else "valid")
    color = gateway_color(status)
    canvas.draw_line_world(app_a, crossing, color, width=5, alpha=0.72)
    canvas.draw_line_world(crossing, app_b, color, width=5, alpha=0.72)
    canvas.draw_circle_world(app_a, 5, "#ffffff", outline=color)
    canvas.draw_circle_world(app_b, 5, "#ffffff", outline=color)
    canvas.draw_cross_world(crossing, color, size=9, width=3)
    canvas.draw_circle_world(gateway["center_xy"], 7, color, outline="#111111", alpha=0.85)
    if label:
        suffix = status
        if audit and audit.get("minimum_required_carve_radius_m") is not None:
            suffix = f"{status} R{audit['minimum_required_carve_radius_m']}"
        canvas.draw_text_world(gateway["center_xy"], f"{gateway['pair_key']} {suffix}", color, dx=10, dy=12, scale=1)


def canvas_bounds(gateways: Iterable[Dict[str, Any]], routes: Iterable[Dict[str, Any]] = (), margin: float = 1.1) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    pts: List[List[float]] = []
    for gateway in gateways:
        pts.append(gateway["center_xy"])
        pts.append(pose_xy(gateway["crossing_pose"]))
        pts.append(pose_xy(gateway["approach_from_room_a"]))
        pts.append(pose_xy(gateway["approach_from_room_b"]))
    for route in routes:
        pts.extend(route.get("projection_polyline_xy") or [])
    arr = np.asarray(pts, dtype=float)
    return (float(arr[:, 0].min()) - margin, float(arr[:, 0].max()) + margin), (float(arr[:, 1].min()) - margin, float(arr[:, 1].max()) + margin)


def make_visualizations(
    bev: BevContext,
    gateways: List[Dict[str, Any]],
    gateway_audits: Dict[str, Dict[str, Any]],
    routes: Dict[str, Dict[str, Any]],
    nav_occ: np.ndarray,
    carved_capsules: List[Dict[str, Any]],
) -> Dict[str, Any]:
    manifest: List[Dict[str, Any]] = []

    def add(path: Path, viz_type: str, route_id: Optional[str] = None, gateway_id: Optional[str] = None) -> None:
        manifest.append(
            {
                "file_path": rel(path),
                "visualization_type": viz_type,
                "route_id": route_id,
                "gateway_id": gateway_id,
                "generated_successfully": path.exists(),
            }
        )

    xlim, ylim = canvas_bounds(gateways)
    overview = Canvas(bev, "Step30E gateway-aware nav projection overview", xlim, ylim, nav_occ=nav_occ)
    for cap in carved_capsules:
        overview.draw_line_world(cap["approach_a_xy"], cap["crossing_xy"], "#00a6a6", width=10, alpha=0.3)
        overview.draw_line_world(cap["crossing_xy"], cap["approach_b_xy"], "#00a6a6", width=10, alpha=0.3)
    for gateway in gateways:
        draw_gateway(overview, gateway, gateway_audits.get(gateway["gateway_id"]))
    overview_path = VIZ_ROOT / f"{SHORT}_step30e_nav_projection_map_overview.png"
    overview.save(overview_path)
    add(overview_path, "nav_projection_map_overview")

    seg = Canvas(bev, "Step30E segmentation-blocked reference overlay", xlim, ylim, nav_occ=None)
    for gateway in gateways:
        draw_gateway(seg, gateway, gateway_audits.get(gateway["gateway_id"]))
        audit = gateway_audits[gateway["gateway_id"]]
        if any(p["inside_segmentation_wall_processed"] for p in audit["pose_audits"].values()):
            seg.draw_text_world(gateway["center_xy"], "SEG WALL", "#b00020", dx=10, dy=-28, scale=1)
    seg_path = VIZ_ROOT / f"{SHORT}_step30e_segmentation_blocked_reference_overlay.png"
    seg.save(seg_path)
    add(seg_path, "segmentation_blocked_reference_overlay")

    for route_id, filename in [
        ("public_path_042", f"{SHORT}_step30e_public_path_042_nav_projection_audit.png"),
        ("long_structure_route_default", f"{SHORT}_step30e_long_structure_route_default_nav_projection_audit.png"),
    ]:
        route = routes.get(route_id)
        path = VIZ_ROOT / filename
        if route:
            rxlim, rylim = canvas_bounds(gateways, [route])
            canvas = Canvas(bev, f"Step30E {route_id} nav projection audit", rxlim, rylim, nav_occ=nav_occ)
            pts = route.get("projection_polyline_xy") or []
            for p0, p1 in zip(pts[:-1], pts[1:]):
                canvas.draw_line_world(p0, p1, "#111111", width=4, alpha=0.68)
                canvas.draw_line_world(p0, p1, "#ffffff", width=1, alpha=0.95)
            for gid in route.get("gateway_sequence") or []:
                gateway = next((g for g in gateways if g["gateway_id"] == gid), None)
                if gateway:
                    draw_gateway(canvas, gateway, gateway_audits.get(gid), label=True)
            canvas.save(path)
        add(path, "route_nav_projection_audit", route_id=route_id)

    for gateway in gateways:
        pair_key = gateway["pair_key"]
        xlim_g, ylim_g = canvas_bounds([gateway], margin=0.9)
        crop = Canvas(bev, f"Step30E local gateway audit {pair_key}", xlim_g, ylim_g, nav_occ=None, width=900, height=760)
        audit = gateway_audits[gateway["gateway_id"]]
        crossing = pose_xy(gateway["crossing_pose"])
        app_a = pose_xy(gateway["approach_from_room_a"])
        app_b = pose_xy(gateway["approach_from_room_b"])
        crop.draw_line_world(app_a, crossing, "#00a6a6", width=14, alpha=0.28)
        crop.draw_line_world(crossing, app_b, "#00a6a6", width=14, alpha=0.28)
        crop.draw_line_world(app_a, crossing, "#111111", width=2, alpha=0.8)
        crop.draw_line_world(crossing, app_b, "#111111", width=2, alpha=0.8)
        draw_gateway(crop, gateway, audit)
        crop.draw_text_px(
            16,
            48,
            f"SEG {audit['segmentation_reference_summary']} | NAV {audit['gateway_aware_nav_projection_summary']} | R {audit['minimum_required_carve_radius_m']}",
            "#111111",
            scale=1,
            background="#ffffff",
        )
        path = LOCAL_CROP_ROOT / f"{SHORT}_step30e_local_gateway_audit_crop_{pair_key}.png"
        crop.save(path)
        add(path, "local_gateway_audit_crop", gateway_id=gateway["gateway_id"])

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_visualization_manifest",
        "step": STEP,
        "version": VERSION,
        "visualizations": manifest,
        "summary": {
            "visualization_count": len(manifest),
            "generated_successfully_count": sum(1 for item in manifest if item["generated_successfully"]),
        },
    }


def build_gateway_audits(
    bev: BevContext,
    gateways: List[Dict[str, Any]],
    maps: Dict[str, np.ndarray],
    base_preclose_occ: np.ndarray,
) -> Tuple[Dict[str, Dict[str, Any]], List[Dict[str, Any]]]:
    audits = {}
    review_items: List[Dict[str, Any]] = []
    for gateway in sorted(gateways, key=lambda g: EXPECTED_GATEWAY_PAIRS.index(g["pair_key"]) if g["pair_key"] in EXPECTED_GATEWAY_PAIRS else 999):
        gid = gateway["gateway_id"]
        pose_audits = {
            "center_xy": pose_audit(bev, maps, gateway, "center_xy", gateway["center_xy"]),
            "crossing_pose": pose_audit(bev, maps, gateway, "crossing_pose", pose_xy(gateway["crossing_pose"])),
            "approach_from_room_a": pose_audit(bev, maps, gateway, "approach_from_room_a", pose_xy(gateway["approach_from_room_a"])),
            "approach_from_room_b": pose_audit(bev, maps, gateway, "approach_from_room_b", pose_xy(gateway["approach_from_room_b"])),
        }
        local_triplets = {name: audit_triplet(bev, occ, gateway) for name, occ in maps.items()}
        min_radius = minimum_required_radius(bev, base_preclose_occ, gateway)
        notes: List[str] = []
        invalid_reasons: List[str] = []
        review_reasons: List[str] = []
        if any(not item["in_bounds"] for item in pose_audits.values()):
            invalid_reasons.append("one or more poses are out of BEV bounds")
        if not local_triplets["gateway_aware_nav_projection_map"]["locally_connected"]:
            invalid_reasons.append("gateway-aware nav projection does not provide local triplet connectivity")
        if min_radius is None:
            invalid_reasons.append("tested carve radii did not make the local triplet connected")
        seg_conflict = any(item["inside_segmentation_wall_processed"] for item in pose_audits.values())
        preclose_conflict = any(item["inside_gateway_wall_preclose"] for item in pose_audits.values())
        if seg_conflict:
            review_reasons.append("one or more poses lie on segmentation_wall_processed")
        if preclose_conflict:
            review_reasons.append("one or more poses lie on gateway_wall_preclose before carving")
        if gateway.get("review_required"):
            review_reasons.append("Step30C/Step30D marked this gateway for review")
        if any(item["approach_pose_on_expected_room_side"] is False for item in pose_audits.values()):
            review_reasons.append("one or more approach poses are not closest to expected room side")
        if any(item["crossing_pose_near_both_target_rooms"] is False for item in pose_audits.values()):
            review_reasons.append("center/crossing pose is not near both target room masks")
        if invalid_reasons:
            status = "invalid"
        elif review_reasons:
            status = "review"
        else:
            status = "valid"
        if seg_conflict and not preclose_conflict:
            notes.append("blocked only by segmentation_wall_processed; gateway_wall_preclose does not block the audited poses")
        if min_radius == 0.0:
            notes.append("local triplet is connected without additional carve on gateway_preclose_reference_map")
        elif min_radius is not None:
            notes.append(f"local triplet requires gateway-aware carve radius {min_radius:.2f}m")
        audit = {
            "gateway_id": gid,
            "pair_key": gateway["pair_key"],
            "room_a": int(gateway["room_a"]),
            "room_b": int(gateway["room_b"]),
            "center_xy": gateway["center_xy"],
            "crossing_pose": gateway["crossing_pose"],
            "approach_from_room_a": gateway["approach_from_room_a"],
            "approach_from_room_b": gateway["approach_from_room_b"],
            "status_in_segmentation_blocked_reference_map": local_triplets["segmentation_blocked_reference_map"],
            "status_in_gateway_preclose_reference_map": local_triplets["gateway_preclose_reference_map"],
            "status_in_gateway_aware_nav_projection_map": local_triplets["gateway_aware_nav_projection_map"],
            "segmentation_reference_summary": "blocked_or_not_connected" if not local_triplets["segmentation_blocked_reference_map"]["locally_connected"] else "locally_connected",
            "gateway_aware_nav_projection_summary": "locally_connected" if local_triplets["gateway_aware_nav_projection_map"]["locally_connected"] else "blocked_or_not_connected",
            "distance_to_obstacle_m": {
                pose_name: pose_data["map_candidate_status"]["gateway_aware_nav_projection_map"]["distance_to_nearest_obstacle_m"]
                for pose_name, pose_data in pose_audits.items()
            },
            "wall_unknown_outside_conflicts": {
                pose_name: {
                    "segmentation_wall": pose_data["inside_segmentation_wall_processed"],
                    "gateway_wall_preclose": pose_data["inside_gateway_wall_preclose"],
                    "unknown_status": pose_data["map_candidate_status"]["gateway_aware_nav_projection_map"]["occupancy_status"] == "unknown",
                    "outside_status": pose_data["map_candidate_status"]["gateway_aware_nav_projection_map"]["occupancy_status"] == "outside",
                }
                for pose_name, pose_data in pose_audits.items()
            },
            "room_side_consistency": {
                pose_name: {
                    "room_mask_global_id_at_pose": pose_data["room_mask_global_id_at_pose"],
                    "nearest_target_room": pose_data["nearest_target_room"],
                    "approach_pose_on_expected_room_side": pose_data["approach_pose_on_expected_room_side"],
                    "crossing_pose_near_both_target_rooms": pose_data["crossing_pose_near_both_target_rooms"],
                }
                for pose_name, pose_data in pose_audits.items()
            },
            "pose_audits": pose_audits,
            "local_triplet_connectivity": local_triplets,
            "minimum_required_carve_radius_m": min_radius,
            "audit_status": status,
            "notes": notes + review_reasons + invalid_reasons,
        }
        audits[gid] = audit
        if status != "valid":
            review_items.append({"gateway_id": gid, "pair_key": gateway["pair_key"], "audit_status": status, "reasons": audit["notes"]})
    return audits, review_items


def build_route_audit(route: Dict[str, Any], gateway_audits: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    local_results = []
    review_edges = []
    blocking = []
    min_by_gateway = {}
    for idx, gid in enumerate(route.get("gateway_sequence") or []):
        audit = gateway_audits.get(gid)
        if not audit:
            blocking.append(f"missing gateway audit for {gid}")
            continue
        result = {
            "segment_index": idx,
            "gateway_id": gid,
            "pair_key": audit["pair_key"],
            "audit_status": audit["audit_status"],
            "gateway_aware_locally_connected": audit["status_in_gateway_aware_nav_projection_map"]["locally_connected"],
            "minimum_required_carve_radius_m": audit["minimum_required_carve_radius_m"],
            "notes": audit["notes"],
        }
        local_results.append(result)
        min_by_gateway[gid] = audit["minimum_required_carve_radius_m"]
        if audit["audit_status"] != "valid":
            review_edges.append({"gateway_id": gid, "pair_key": audit["pair_key"], "audit_status": audit["audit_status"], "notes": audit["notes"]})
        if not audit["status_in_gateway_aware_nav_projection_map"]["locally_connected"]:
            blocking.append(f"{audit['pair_key']} is not locally connected in gateway-aware nav projection map")
        if audit["minimum_required_carve_radius_m"] is None:
            blocking.append(f"{audit['pair_key']} has no tested carve radius that makes local triplet connected")
    ready = bool(route.get("graph_valid")) and not blocking
    reason = (
        "graph route and all local gateway triplets are connected in the gateway-aware nav projection map; review labels remain offline audit flags"
        if ready
        else "not ready because one or more local triplets cannot be validated in the gateway-aware nav projection map"
    )
    return {
        "route_id": route["route_id"],
        "room_sequence": route.get("room_sequence") or [],
        "gateway_sequence": route.get("gateway_sequence") or [],
        "route_graph_valid_from_step30d": bool(route.get("graph_valid")),
        "local_triplet_sequence": route.get("approach_pose_sequence") or [],
        "local_triplet_audit_results": local_results,
        "route_projection_status": route.get("projection_status"),
        "nav_projection_default_usable": bool(route.get("nav2_default_usable")),
        "nav_projection_ready_for_planning_only": ready,
        "nav_projection_ready_reason": reason,
        "review_required_edges": review_edges,
        "blocking_issues": blocking,
        "minimum_required_carve_radius_by_gateway": min_by_gateway,
        "reasons_route_is_or_is_not_ready_for_nav2_planning_only": [reason] + blocking,
        "notes": [
            "This is an offline local doorway-crossing audit, not an executable route claim.",
            "Long straight-line collision-free path between distant gateways was not evaluated.",
        ],
    }


def validation_payload(
    outputs: Dict[str, Path],
    gateway_audits: Dict[str, Dict[str, Any]],
    route_audits: List[Dict[str, Any]],
    visualization_manifest: Dict[str, Any],
    carved_capsules: List[Dict[str, Any]],
    missing_layers: List[str],
) -> Dict[str, Any]:
    route_ids = {route["route_id"] for route in route_audits}
    carved_pairs = {cap["pair_key"] for cap in carved_capsules}
    checks = []

    def check(name: str, passed: bool, details: Dict[str, Any]) -> None:
        checks.append({"check_name": name, "passed": bool(passed), "details": details})

    check("no_stage_a_rerun", True, {"stage_a_rerun_attempted": False})
    check("no_gateway_extraction_rerun", True, {"gateway_extraction_rerun_attempted": False})
    check("step30c_topology_unchanged", True, {"topology_changed": False, "source_hash": sha256_file(STEP30C_EDGE_TABLE)})
    check("step30d_route_projection_unchanged", True, {"step30d_route_projection_changed": False, "source_hash": sha256_file(STEP30D_ROUTE_REVIEW)})
    check("no_ros_nav2_gazebo_run", True, {"ros_nav2_gazebo_run": False})
    check("no_nav2_route_generated", True, {"nav2_route_generated": False})
    check("no_robot_execution", True, {"robot_execution": False})
    check("bev_layers_loaded_or_reported", True, {"missing_layers": missing_layers})
    check("gateway_aware_nav_projection_payload_exists", outputs["payload"].exists(), {"path": rel(outputs["payload"])})
    check("every_step30d_gateway_pose_marker_audited", len(gateway_audits) == 8, {"gateway_audit_count": len(gateway_audits)})
    check("r3_r11_not_carved_or_introduced", "r3_r11" not in carved_pairs and all(a["pair_key"] != "r3_r11" for a in gateway_audits.values()), {"carved_pairs": sorted(carved_pairs)})
    check("non_truth_pairs_not_carved", carved_pairs.issubset(set(EXPECTED_GATEWAY_PAIRS)), {"carved_pairs": sorted(carved_pairs)})
    check("public_path_042_route_audit_exists", "public_path_042" in route_ids, {"route_ids": sorted(route_ids)})
    check("long_structure_route_default_route_audit_exists", "long_structure_route_default" in route_ids, {"route_ids": sorted(route_ids)})
    check("static_visualizations_exist", visualization_manifest["summary"]["generated_successfully_count"] >= 12, visualization_manifest["summary"])
    check("rviz_gazebo_overlay_payload_exists_but_not_published", outputs["overlay"].exists(), {"published": False, "path": rel(outputs["overlay"])})
    explicit_review = [a for a in gateway_audits.values() if a["audit_status"] != "valid" and a["notes"]]
    check("review_or_invalid_gateways_explicitly_listed", len(explicit_review) == sum(1 for a in gateway_audits.values() if a["audit_status"] != "valid"), {"review_or_invalid_count": len(explicit_review)})
    pose_data_usable = all(a["pose_audits"] and a["local_triplet_connectivity"] for a in gateway_audits.values())
    check("pose_data_not_silently_unusable", pose_data_usable, {"gateway_audit_count": len(gateway_audits)})
    for key, path in outputs.items():
        if key in {"validation", "summary"}:
            check(
                f"required_output_scheduled_{key}",
                True,
                {"path": rel(path), "note": "written immediately after validation payload construction"},
            )
        else:
            check(f"required_output_exists_{key}", path.exists(), {"path": rel(path)})
    passed = all(item["passed"] for item in checks)
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_validation_results",
        "step": STEP,
        "version": VERSION,
        "validation_passed": passed,
        "checks": checks,
        "review_or_invalid_gateways": explicit_review,
    }


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    VIZ_ROOT.mkdir(parents=True, exist_ok=True)
    LOCAL_CROP_ROOT.mkdir(parents=True, exist_ok=True)

    overlay = read_json(STEP30D_OVERLAY)
    route_review = read_json(STEP30D_ROUTE_REVIEW)
    gateway_pose_validation = read_json(STEP30D_GATEWAY_POSES)
    room_anchor_validation = read_json(STEP30D_ROOM_ANCHORS)
    step30d_validation = read_json(STEP30D_VALIDATION)
    step30d_summary = read_json(STEP30D_SUMMARY)
    edge_table = read_json(STEP30C_EDGE_TABLE)
    topology_routes = read_json(STEP30C_ROUTES)
    bev = BevContext(STEP30A_LAYERED_BEV_NPZ, STEP30A_LAYERED_BEV_JSON)

    expected_layers = [
        "room_mask_global_id",
        "free_space",
        "gateway_wall_preclose",
        "segmentation_wall_processed",
        "segmentation_wall_processed_postclose",
        "structural_wall_legacy_or_segmentation_reference",
        "outside_boundary",
        "unknown_layer",
        "full_map_post_doors_reference",
    ]
    missing_layers = [layer for layer in expected_layers if not bev.has_layer(layer)]

    gateways = sorted(
        [g for g in overlay.get("gateway_pose_markers", []) if g.get("pair_key") in EXPECTED_GATEWAY_PAIRS],
        key=lambda g: EXPECTED_GATEWAY_PAIRS.index(g["pair_key"]),
    )
    segmentation_occ = build_occupancy(bev, "segmentation_wall_processed")
    gateway_preclose_occ = build_occupancy(bev, "gateway_wall_preclose")
    gateway_aware_occ, carved_capsules = build_gateway_aware_map(bev, gateways, DEFAULT_CARVE_RADIUS_M)
    maps = {
        "segmentation_blocked_reference_map": segmentation_occ,
        "gateway_preclose_reference_map": gateway_preclose_occ,
        "gateway_aware_nav_projection_map": gateway_aware_occ,
    }

    gateway_audits, review_items = build_gateway_audits(bev, gateways, maps, gateway_preclose_occ)
    routes_by_id = {route["route_id"]: route for route in route_review.get("routes", [])}
    route_audits = [
        build_route_audit(routes_by_id[route_id], gateway_audits)
        for route_id in REQUIRED_ROUTES + OPTIONAL_ROUTES
        if route_id in routes_by_id
    ]
    route_audit_by_id = {route["route_id"]: route for route in route_audits}

    visualization_manifest = make_visualizations(bev, gateways, gateway_audits, routes_by_id, gateway_aware_occ, carved_capsules)

    metadata_path = OUTPUT_ROOT / f"{SHORT}_step30e_nav_projection_map_metadata_{VERSION}.json"
    gateway_audit_path = OUTPUT_ROOT / f"{SHORT}_step30e_gateway_pose_nav_projection_audit_{VERSION}.json"
    route_audit_path = OUTPUT_ROOT / f"{SHORT}_step30e_route_nav_projection_audit_{VERSION}.json"
    payload_path = OUTPUT_ROOT / f"{SHORT}_step30e_gateway_aware_nav_projection_payload_{VERSION}.json"
    overlay_payload_path = OUTPUT_ROOT / f"{SHORT}_step30e_rviz_gazebo_overlay_payload_{VERSION}.json"
    viz_manifest_path = OUTPUT_ROOT / f"{SHORT}_step30e_visualization_manifest_{VERSION}.json"
    validation_path = OUTPUT_ROOT / f"{SHORT}_step30e_validation_results_{VERSION}.json"
    summary_path = OUTPUT_ROOT / f"{SHORT}_step30e_summary_{VERSION}.json"

    metadata = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_nav_projection_map_metadata",
        "step": STEP,
        "version": VERSION,
        "source_bev_artifact_paths": {"npz": rel(STEP30A_LAYERED_BEV_NPZ), "metadata_json": rel(STEP30A_LAYERED_BEV_JSON)},
        "source_step30d_paths": {
            "room_anchor_validation": rel(STEP30D_ROOM_ANCHORS),
            "gateway_pose_validation": rel(STEP30D_GATEWAY_POSES),
            "route_projection_review": rel(STEP30D_ROUTE_REVIEW),
            "corrected_overlay_payload": rel(STEP30D_OVERLAY),
            "validation_results": rel(STEP30D_VALIDATION),
            "summary": rel(STEP30D_SUMMARY),
        },
        "source_step30c_paths": {"gateway_edge_table": rel(STEP30C_EDGE_TABLE), "topology_route_candidates": rel(STEP30C_ROUTES)},
        "map_resolution": bev.resolution,
        "origin": bev.origin,
        "map_dimensions": {"width": bev.width, "height": bev.height},
        "layers_used": {layer: bev.has_layer(layer) for layer in expected_layers},
        "missing_layers": missing_layers,
        "nav_projection_construction_parameters": {
            "occupancy_convention": {"free": OCC_FREE, "occupied": OCC_OCCUPIED, "unknown": OCC_UNKNOWN},
            "unknown_layer_treated_as_non_free": True,
            "outside_boundary_treated_as_occupied": True,
            "segmentation_blocked_reference_map_wall_layer": "segmentation_wall_processed",
            "gateway_preclose_reference_map_wall_layer": "gateway_wall_preclose",
            "gateway_aware_base_wall_layer": "gateway_wall_preclose",
            "default_carve_radius_m": DEFAULT_CARVE_RADIUS_M,
            "line_sampling_resolution_m": bev.resolution * SAMPLING_RESOLUTION_FACTOR,
        },
        "carve_radii_tested_m": CARVE_RADII_M,
        "gateway_aware_map_description": "Door-preserving free-space projection using gateway_wall_preclose as wall evidence, with local capsules carved only for Step30D selected truth gateway triplets.",
    }

    gateway_audit_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_gateway_pose_nav_projection_audit",
        "step": STEP,
        "version": VERSION,
        "gateway_pose_audits": list(gateway_audits.values()),
        "blocked_only_by_segmentation_wall_processed": [
            {"gateway_id": audit["gateway_id"], "pair_key": audit["pair_key"], "notes": audit["notes"]}
            for audit in gateway_audits.values()
            if any(p["inside_segmentation_wall_processed"] for p in audit["pose_audits"].values())
            and not any(p["inside_gateway_wall_preclose"] for p in audit["pose_audits"].values())
        ],
        "summary": {
            "gateway_pose_count": len(gateway_audits),
            "valid_count": sum(1 for a in gateway_audits.values() if a["audit_status"] == "valid"),
            "review_count": sum(1 for a in gateway_audits.values() if a["audit_status"] == "review"),
            "invalid_count": sum(1 for a in gateway_audits.values() if a["audit_status"] == "invalid"),
            "gateway_count_requiring_carve": sum(1 for a in gateway_audits.values() if a["minimum_required_carve_radius_m"] not in (None, 0.0)),
            "max_required_carve_radius_m": max((a["minimum_required_carve_radius_m"] or 0.0 for a in gateway_audits.values()), default=0.0),
        },
        "source_step30d_gateway_pose_validation_summary": gateway_pose_validation.get("summary", {}),
    }

    route_audit_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_route_nav_projection_audit",
        "step": STEP,
        "version": VERSION,
        "routes": route_audits,
        "source_step30d_route_projection_review_summary": route_review.get("summary", {}),
    }

    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_gateway_aware_nav_projection_payload",
        "step": STEP,
        "version": VERSION,
        "coordinate_frame": "stage_a_map_xy",
        "target_later_frame": "map",
        "not_a_live_nav2_map": True,
        "occupancy_convention": {"free": OCC_FREE, "occupied": OCC_OCCUPIED, "unknown": OCC_UNKNOWN},
        "free_occupied_unknown_meaning": {
            "free": "known projected free space, including selected gateway-aware local carve capsules",
            "occupied": "outside boundary or wall evidence from gateway_wall_preclose",
            "unknown": "unknown_layer cells outside known free-space support",
        },
        "map_metadata": {"resolution": bev.resolution, "origin": bev.origin, "width": bev.width, "height": bev.height},
        "base_layers": {"free_space": "free", "gateway_wall_preclose": "occupied wall evidence", "outside_boundary": "occupied", "unknown_layer": "unknown"},
        "carved_gateway_capsules": carved_capsules,
        "gateway_poses": [
            {
                "gateway_id": g["gateway_id"],
                "pair_key": g["pair_key"],
                "center_xy": g["center_xy"],
                "crossing_pose": g["crossing_pose"],
                "approach_from_room_a": g["approach_from_room_a"],
                "approach_from_room_b": g["approach_from_room_b"],
            }
            for g in gateways
        ],
        "route_local_triplets": {route["route_id"]: route.get("local_triplet_sequence") for route in route_audits},
        "recommended_planning_only_route_candidates": [
            route["route_id"] for route in route_audits if route["nav_projection_ready_for_planning_only"]
        ],
        "gateway_aware_occupancy_rle": compact_rle(gateway_aware_occ),
        "excluded_pairs": ["r3_r11"],
    }

    overlay_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_rviz_gazebo_overlay_payload",
        "step": STEP,
        "version": VERSION,
        "coordinate_frame": "stage_a_map_xy",
        "target_later_frame": "map",
        "published": False,
        "markers_are_offline_payload_only": True,
        "room_anchors": room_anchor_validation.get("rooms", overlay.get("rooms", [])),
        "gateway_markers": [
            {
                "marker_type": "gateway",
                "gateway_id": audit["gateway_id"],
                "pair_key": audit["pair_key"],
                "center_xy": audit["center_xy"],
                "label": f"{audit['pair_key']} {audit['audit_status']}",
                "audit_status": audit["audit_status"],
                "passability_label": audit["gateway_aware_nav_projection_summary"],
            }
            for audit in gateway_audits.values()
        ],
        "approach_pose_arrows": [
            {
                "marker_type": "approach_pose_arrow",
                "gateway_id": audit["gateway_id"],
                "pair_key": audit["pair_key"],
                "from_xy": pose_xy(audit["approach_from_room_a"]),
                "to_xy": pose_xy(audit["crossing_pose"]),
                "semantic": "approach_a_to_crossing",
            }
            for audit in gateway_audits.values()
        ]
        + [
            {
                "marker_type": "approach_pose_arrow",
                "gateway_id": audit["gateway_id"],
                "pair_key": audit["pair_key"],
                "from_xy": pose_xy(audit["approach_from_room_b"]),
                "to_xy": pose_xy(audit["crossing_pose"]),
                "semantic": "approach_b_to_crossing",
            }
            for audit in gateway_audits.values()
        ],
        "crossing_pose_arrows": [
            {"marker_type": "crossing_pose", "gateway_id": audit["gateway_id"], "pair_key": audit["pair_key"], "pose": audit["crossing_pose"]}
            for audit in gateway_audits.values()
        ],
        "route_polylines": [
            {
                "route_id": route["route_id"],
                "points_xy": routes_by_id[route["route_id"]].get("projection_polyline_xy") or [],
                "semantics": "approach_crossing_projection_only",
                "nav_projection_ready_for_planning_only": route["nav_projection_ready_for_planning_only"],
            }
            for route in route_audits
        ],
        "local_carve_capsules_disks": carved_capsules,
        "passability_status_labels": [
            {"gateway_id": audit["gateway_id"], "pair_key": audit["pair_key"], "audit_status": audit["audit_status"], "notes": audit["notes"]}
            for audit in gateway_audits.values()
        ],
        "review_labels": review_items,
        "map_layer_metadata": metadata,
    }

    summary_counts = Counter(a["audit_status"] for a in gateway_audits.values())
    max_radius = max((a["minimum_required_carve_radius_m"] or 0.0 for a in gateway_audits.values()), default=0.0)
    summary = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30e_summary",
        "step": STEP,
        "version": VERSION,
        "stage_a_rerun_attempted": False,
        "gateway_extraction_rerun_attempted": False,
        "topology_changed": False,
        "step30d_route_projection_changed": False,
        "ros_nav2_gazebo_run": False,
        "nav2_route_generated": False,
        "robot_execution": False,
        "bev_npz_used": rel(STEP30A_LAYERED_BEV_NPZ),
        "nav_projection_map_generated": True,
        "gateway_pose_count": len(gateway_audits),
        "gateway_pose_nav_projection_valid_count": summary_counts.get("valid", 0),
        "gateway_pose_nav_projection_review_count": summary_counts.get("review", 0),
        "gateway_pose_nav_projection_invalid_count": summary_counts.get("invalid", 0),
        "public_path_042_nav_projection_ready_for_planning_only": route_audit_by_id.get("public_path_042", {}).get("nav_projection_ready_for_planning_only", False),
        "long_structure_route_default_nav_projection_ready_for_planning_only": route_audit_by_id.get("long_structure_route_default", {}).get("nav_projection_ready_for_planning_only", False),
        "gateway_count_requiring_carve": sum(1 for a in gateway_audits.values() if a["minimum_required_carve_radius_m"] not in (None, 0.0)),
        "max_required_carve_radius_m": max_radius,
        "static_visualization_count": visualization_manifest["summary"]["generated_successfully_count"],
        "recommended_next_step": "Use the gateway-aware projection payload for a later Nav2 planning-only map export; preserve review labels for low_or_uncertain gateways and do not use segmentation_wall_processed as the sole navigation blocker.",
        "source_summaries": {
            "step30d_summary": step30d_summary,
            "step30d_validation": step30d_validation.get("summary", {}),
            "step30c_topology_routes": topology_routes.get("summary", {}),
        },
    }

    outputs = {
        "metadata": metadata_path,
        "gateway_audit": gateway_audit_path,
        "route_audit": route_audit_path,
        "payload": payload_path,
        "overlay": overlay_payload_path,
        "visualization_manifest": viz_manifest_path,
        "validation": validation_path,
        "summary": summary_path,
    }

    write_json(metadata_path, metadata)
    write_json(gateway_audit_path, gateway_audit_payload)
    write_json(route_audit_path, route_audit_payload)
    write_json(payload_path, payload)
    write_json(overlay_payload_path, overlay_payload)
    write_json(viz_manifest_path, visualization_manifest)
    validation = validation_payload(outputs, gateway_audits, route_audits, visualization_manifest, carved_capsules, missing_layers)
    summary["validation_passed"] = validation["validation_passed"]
    write_json(validation_path, validation)
    write_json(summary_path, summary)

    print("Step30E complete.")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
