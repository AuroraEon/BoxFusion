#!/usr/bin/env python3
"""Step30D topology route projection and overlay review for scene 00824.

This is an offline visualization and validation step. It consumes Step30C and
Step30A artifacts, preserves the Step30C topology as-is, and does not invoke
ROS, Nav2, Gazebo, RViz, AMCL, DWB, robot execution, or Nav2 route generation.
"""

from __future__ import annotations

import copy
import json
import math
import struct
import zlib
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30d"
VERSION = "v0_1"

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
STEP30C_ROOT = RUNTIME_ROOT / "step30c_00824_gateway_augmented_topology_candidate"
STEP30A_ASSETS = (
    RUNTIME_ROOT / "step30a_00824_full_stage_a_dual_wall_gateway_rerun" / "generated" / "assets"
)
STEP30A_LOGS = (
    RUNTIME_ROOT
    / "step30a_00824_full_stage_a_dual_wall_gateway_rerun"
    / "scenes"
    / SCENE_ID
    / "logs"
)
OUTPUT_ROOT = RUNTIME_ROOT / "step30d_00824_topology_route_projection_overlay_review"
VIZ_ROOT = OUTPUT_ROOT / "visualizations"

STEP30C_GRAPH = STEP30C_ROOT / f"{SHORT}_step30c_gateway_augmented_topology_candidate_{VERSION}.json"
STEP30C_EDGE_TABLE = STEP30C_ROOT / f"{SHORT}_step30c_gateway_edge_table_{VERSION}.json"
STEP30C_ROUTES = STEP30C_ROOT / f"{SHORT}_step30c_topology_route_candidates_{VERSION}.json"
STEP30C_OVERLAY = STEP30C_ROOT / f"{SHORT}_step30c_overlay_payload_{VERSION}.json"
STEP30C_VALIDATION = STEP30C_ROOT / f"{SHORT}_step30c_validation_results_{VERSION}.json"
STEP30C_SUMMARY = STEP30C_ROOT / f"{SHORT}_step30c_summary_{VERSION}.json"

STEP30A_LAYERED_BEV_NPZ = STEP30A_ASSETS / f"{SHORT}_step30a_layered_bev_{VERSION}.npz"
STEP30A_LAYERED_BEV_JSON = STEP30A_ASSETS / f"{SHORT}_step30a_layered_bev_{VERSION}.json"
COMMITTED_ROOM_WORLD_MODEL = STEP30A_LOGS / "committed_room_world_model_v0_1.json"

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
OPTIONAL_REVIEW_ROUTE = "long_structure_review_all_positive_gateways"

PASSABILITY_COLORS = {
    "strong": "#1b9e77",
    "reviewable": "#d95f02",
    "low_or_uncertain": "#d62728",
}

MANUAL_REVIEW_CHECKLIST = [
    "Check gateway markers are on real door/gateway locations.",
    "Check crossing arrows point through the gateway.",
    "Check approach poses lie on opposite sides of the gateway.",
    "Check public_path_042 sequence is visually sensible.",
    "Check long_structure_route_default does not use misleading room-centroid straight lines.",
    "Check r3_r11 direct edge is absent.",
    "Check low_or_uncertain passability edges are labeled.",
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
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")
    print(f"Written: {rel(path)}")


def room_id_text(room_id: int | str) -> str:
    text = str(room_id)
    if text.startswith("room_"):
        return text
    if text.startswith("r"):
        return f"room_{int(text[1:])}"
    return f"room_{int(text)}"


def room_number(room_id: int | str) -> int:
    text = str(room_id)
    if text.startswith("room_"):
        text = text.split("_", 1)[1]
    elif text.startswith("r"):
        text = text[1:]
    return int(text)


def pair_key_for(room_a: int, room_b: int) -> str:
    a, b = sorted((int(room_a), int(room_b)))
    return f"r{a}_r{b}"


def bbox_contains_xy(bbox: Optional[Dict[str, Any]], xy: Optional[Sequence[float]]) -> Optional[bool]:
    if not bbox or xy is None:
        return None
    x, y = float(xy[0]), float(xy[1])
    return float(bbox["min_x"]) <= x <= float(bbox["max_x"]) and float(bbox["min_y"]) <= y <= float(
        bbox["max_y"]
    )


def finite_xy(xy: Optional[Sequence[float]]) -> bool:
    return xy is not None and len(xy) >= 2 and all(math.isfinite(float(v)) for v in xy[:2])


def finite_pose(pose: Optional[Dict[str, Any]]) -> bool:
    if not pose:
        return False
    return all(math.isfinite(float(pose.get(key, math.nan))) for key in ("x", "y", "yaw"))


def pose_xy(pose: Dict[str, Any]) -> List[float]:
    return [float(pose["x"]), float(pose["y"])]


class BevContext:
    def __init__(self, npz_path: Path, metadata_path: Path) -> None:
        self.available = npz_path.exists()
        self.npz_path = npz_path
        self.metadata_path = metadata_path
        self.layers: Dict[str, np.ndarray] = {}
        self.metadata: Dict[str, Any] = {}
        self.resolution = 0.05
        self.origin = [-50.0, -50.0]
        self.width = 0
        self.height = 0
        self.room_pixels: Dict[int, np.ndarray] = {}

        if not self.available:
            return
        loaded = np.load(npz_path)
        self.layers = {key: loaded[key] for key in loaded.files}
        if metadata_path.exists():
            self.metadata = read_json(metadata_path)
            self.resolution = float(self.metadata.get("resolution", self.resolution))
            self.origin = [float(v) for v in self.metadata.get("origin", self.origin)]
            self.width = int(self.metadata.get("width", next(iter(self.layers.values())).shape[1]))
            self.height = int(self.metadata.get("height", next(iter(self.layers.values())).shape[0]))
        else:
            first = next(iter(self.layers.values()))
            self.height, self.width = first.shape[:2]

        room_mask = self.layers.get("room_mask_global_id")
        if room_mask is not None:
            for rid in sorted(int(v) for v in np.unique(room_mask) if int(v) > 0):
                self.room_pixels[rid] = np.argwhere(room_mask == rid)

    def has_layer(self, name: str) -> bool:
        return name in self.layers

    def xy_to_rc(self, xy: Sequence[float]) -> Tuple[int, int]:
        x, y = float(xy[0]), float(xy[1])
        col = int(round((x - self.origin[0]) / self.resolution))
        row = int(round((y - self.origin[1]) / self.resolution))
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

    def patch_values(self, layer: str, xy: Sequence[float], radius_m: float) -> Optional[np.ndarray]:
        arr = self.layers.get(layer)
        if arr is None:
            return None
        row, col = self.xy_to_rc(xy)
        if not self.in_bounds_rc(row, col):
            return None
        radius_px = max(1, int(math.ceil(radius_m / self.resolution)))
        r0, r1 = max(0, row - radius_px), min(self.height, row + radius_px + 1)
        c0, c1 = max(0, col - radius_px), min(self.width, col + radius_px + 1)
        return arr[r0:r1, c0:c1]

    def near_layer(self, layer: str, xy: Sequence[float], radius_m: float, positive: bool = True) -> Optional[bool]:
        patch = self.patch_values(layer, xy, radius_m)
        if patch is None:
            return None
        if positive:
            return bool(np.any(patch > 0))
        return bool(np.any(patch == 0))

    def room_anchor_from_mask(self, room_id: int) -> Tuple[Optional[List[float]], str, List[str]]:
        notes: List[str] = []
        pixels = self.room_pixels.get(int(room_id))
        if pixels is None or len(pixels) < 20:
            return None, "unavailable", ["room_mask_global_id missing or too sparse for this room"]

        mean_row, mean_col = pixels[:, 0].mean(), pixels[:, 1].mean()
        mean_xy = self.rc_to_xy(mean_row, mean_col)
        if self.value_at("room_mask_global_id", mean_xy) == int(room_id):
            return mean_xy, "room_mask_global_id_centroid", notes

        median_row, median_col = np.median(pixels[:, 0]), np.median(pixels[:, 1])
        median_xy = self.rc_to_xy(median_row, median_col)
        if self.value_at("room_mask_global_id", median_xy) == int(room_id):
            notes.append("mean room-mask centroid landed outside room id; used median mask cell")
            return median_xy, "room_mask_global_id_median", notes

        idx = len(pixels) // 2
        cell_xy = self.rc_to_xy(float(pixels[idx, 0]), float(pixels[idx, 1]))
        notes.append("mean and median mask anchors missed exact room id; used a representative mask cell")
        return cell_xy, "room_mask_global_id_representative_cell", notes

    def distance_to_room_m(self, room_id: int, xy: Sequence[float]) -> Optional[float]:
        pixels = self.room_pixels.get(int(room_id))
        if pixels is None or len(pixels) == 0 or not self.in_bounds_xy(xy):
            return None
        row, col = self.xy_to_rc(xy)
        d2 = (pixels[:, 0].astype(float) - row) ** 2 + (pixels[:, 1].astype(float) - col) ** 2
        return round(float(math.sqrt(float(np.min(d2))) * self.resolution), 4)


def load_room_metadata(graph: Dict[str, Any]) -> Dict[int, Dict[str, Any]]:
    metadata: Dict[int, Dict[str, Any]] = {}
    for room in graph.get("nodes", {}).get("rooms", []):
        rid = room_number(room.get("room_id"))
        metadata[rid] = copy.deepcopy(room.get("room_metadata", {}))
    if COMMITTED_ROOM_WORLD_MODEL.exists():
        model = read_json(COMMITTED_ROOM_WORLD_MODEL)
        for room in model.get("rooms", []):
            rid = room_number(room.get("room_id"))
            metadata.setdefault(rid, {})
            metadata[rid].setdefault("centroid_xy", room.get("centroid_xy"))
            metadata[rid].setdefault("extent_bbox_xy", room.get("extent_bbox_xy"))
            metadata[rid].setdefault("footprint_area_m2", room.get("footprint_area_m2"))
            metadata[rid].setdefault("room_type", room.get("room_type"))
    return metadata


def validate_room_anchors(
    graph: Dict[str, Any], overlay: Dict[str, Any], edge_table: Dict[str, Any], bev: BevContext
) -> Dict[str, Any]:
    room_metadata = load_room_metadata(graph)
    original_by_room = {int(room["room_id"]): room.get("xy") for room in overlay.get("rooms", [])}
    incident_centers: Dict[int, List[List[float]]] = defaultdict(list)
    for edge in edge_table.get("gateway_edges", []):
        if finite_xy(edge.get("center_xy")):
            incident_centers[int(edge["room_a"])].append([float(v) for v in edge["center_xy"]])
            incident_centers[int(edge["room_b"])].append([float(v) for v in edge["center_xy"]])

    step30c_room_ids = {int(room.get("room_id")) for room in overlay.get("rooms", [])}
    step30c_room_ids.update(int(room.get("room_id")) for room in graph.get("nodes", {}).get("rooms", []))

    records: List[Dict[str, Any]] = []
    for rid in sorted(step30c_room_ids):
        original_xy = original_by_room.get(rid)
        metadata = room_metadata.get(rid, {})
        bbox = metadata.get("extent_bbox_xy")
        inside_bbox = bbox_contains_xy(bbox, original_xy)
        room_mask_matches: Optional[bool] = None
        if bev.available and finite_xy(original_xy) and bev.has_layer("room_mask_global_id") and bev.in_bounds_xy(original_xy):
            room_mask_matches = bev.value_at("room_mask_global_id", original_xy) == rid
        elif bev.available and finite_xy(original_xy):
            room_mask_matches = False if bev.has_layer("room_mask_global_id") else None

        notes: List[str] = []
        if original_xy is None:
            status = "missing"
            notes.append("Step30C overlay room marker missing")
        elif inside_bbox is False or room_mask_matches is False:
            status = "invalid_committed_centroid"
            if inside_bbox is False:
                notes.append("original overlay anchor lies outside extent_bbox_xy")
            if room_mask_matches is False:
                notes.append("original overlay anchor does not map to matching room_mask_global_id")
        else:
            status = "valid"

        corrected_xy: Optional[List[float]] = None
        corrected_method = "omitted"
        if bev.available and bev.has_layer("room_mask_global_id"):
            corrected_xy, corrected_method, mask_notes = bev.room_anchor_from_mask(rid)
            notes.extend(mask_notes)

        if corrected_xy is None and incident_centers.get(rid):
            arr = np.asarray(incident_centers[rid], dtype=float)
            corrected_xy = [round(float(arr[:, 0].mean()), 4), round(float(arr[:, 1].mean()), 4)]
            corrected_method = "mean_incident_gateway_centers"
            notes.append("room mask anchor unavailable; used mean of incident gateway centers")

        if corrected_xy is None:
            notes.append("no reliable visualization anchor available; room anchor omitted")

        records.append(
            {
                "room_id": room_id_text(rid),
                "original_anchor_xy": original_xy,
                "original_anchor_status": status,
                "corrected_anchor_xy": corrected_xy,
                "corrected_anchor_method": corrected_method,
                "inside_bbox": inside_bbox,
                "room_mask_matches_room_id": room_mask_matches,
                "extent_bbox_xy": bbox,
                "notes": notes,
            }
        )

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_room_anchor_validation",
        "step": STEP,
        "version": VERSION,
        "topology_source": "Step30C",
        "room_anchors_are_visualization_only": True,
        "bev_layer_available": bev.available,
        "rooms": records,
        "summary": {
            "room_anchor_count": len(records),
            "invalid_original_room_anchor_count": sum(
                1 for item in records if item["original_anchor_status"] == "invalid_committed_centroid"
            ),
            "corrected_room_anchor_count": sum(1 for item in records if item["corrected_anchor_xy"] is not None),
            "corrected_anchor_method_counts": dict(Counter(item["corrected_anchor_method"] for item in records)),
        },
    }


def free_space_status(bev: BevContext, xy: Sequence[float]) -> str:
    if not bev.available or not bev.has_layer("free_space"):
        return "unavailable"
    if not bev.in_bounds_xy(xy):
        return "out_of_bounds"
    if bev.value_at("free_space", xy) == 1:
        return "in_free_space"
    if bev.near_layer("free_space", xy, 0.2):
        return "near_free_space"
    return "not_near_free_space"


def wall_conflict_status(bev: BevContext, xy: Sequence[float]) -> str:
    if not bev.available:
        return "unavailable"
    if not bev.in_bounds_xy(xy):
        return "out_of_bounds"
    active_layers = [layer for layer in ("gateway_wall_preclose", "segmentation_wall_processed") if bev.has_layer(layer)]
    if not active_layers:
        return "unavailable"
    direct = [layer for layer in active_layers if bev.value_at(layer, xy) == 1]
    near = [layer for layer in active_layers if bev.near_layer(layer, xy, 0.1)]
    if direct:
        return "inside_wall_layer:" + ",".join(direct)
    if near:
        return "near_wall_layer:" + ",".join(near)
    return "clear"


def pose_status(bev: BevContext, pose: Optional[Dict[str, Any]]) -> str:
    if not finite_pose(pose):
        return "invalid_nonfinite_or_missing"
    xy = pose_xy(pose)
    if bev.available and not bev.in_bounds_xy(xy):
        return "invalid_out_of_bounds"
    yaw = float(pose["yaw"])
    if not (-math.pi * 2.0 <= yaw <= math.pi * 2.0):
        return "review_yaw_outside_2pi"
    fs = free_space_status(bev, xy)
    wc = wall_conflict_status(bev, xy)
    if fs in {"in_free_space", "near_free_space", "unavailable"} and not wc.startswith("inside_wall_layer"):
        return "valid"
    return "review"


def validate_gateway_poses(edge_table: Dict[str, Any], bev: BevContext) -> Dict[str, Any]:
    records: List[Dict[str, Any]] = []
    for edge in sorted(edge_table.get("gateway_edges", []), key=lambda item: item["pair_key"]):
        gateway_id = edge["gateway_id"]
        room_a = int(edge["room_a"])
        room_b = int(edge["room_b"])
        center_xy = edge.get("center_xy")
        crossing = edge.get("crossing_pose")
        approach_a = edge.get("approach_from_room_a")
        approach_b = edge.get("approach_from_room_b")
        warnings = list(edge.get("warnings") or [])
        validation_notes: List[str] = []

        center_status = "valid" if finite_xy(center_xy) and (not bev.available or bev.in_bounds_xy(center_xy)) else "invalid"
        crossing_status = pose_status(bev, crossing)
        approach_a_status = pose_status(bev, approach_a)
        approach_b_status = pose_status(bev, approach_b)

        center_free = free_space_status(bev, center_xy) if finite_xy(center_xy) else "invalid"
        crossing_free = free_space_status(bev, pose_xy(crossing)) if finite_pose(crossing) else "invalid"
        approach_a_free = free_space_status(bev, pose_xy(approach_a)) if finite_pose(approach_a) else "invalid"
        approach_b_free = free_space_status(bev, pose_xy(approach_b)) if finite_pose(approach_b) else "invalid"
        free_status = {
            "center": center_free,
            "crossing_pose": crossing_free,
            "approach_from_room_a": approach_a_free,
            "approach_from_room_b": approach_b_free,
        }

        wall_status = {
            "center": wall_conflict_status(bev, center_xy) if finite_xy(center_xy) else "invalid",
            "crossing_pose": wall_conflict_status(bev, pose_xy(crossing)) if finite_pose(crossing) else "invalid",
            "approach_from_room_a": wall_conflict_status(bev, pose_xy(approach_a)) if finite_pose(approach_a) else "invalid",
            "approach_from_room_b": wall_conflict_status(bev, pose_xy(approach_b)) if finite_pose(approach_b) else "invalid",
        }
        for pose_name, status in wall_status.items():
            if str(status).startswith("inside_wall_layer"):
                validation_notes.append(f"{pose_name} lies inside a wall-support layer and should be checked in overlay review")

        distances: Dict[str, Optional[float]] = {}
        if finite_pose(crossing) and finite_pose(approach_a):
            distances["approach_a_to_crossing_m"] = round(float(np.linalg.norm(np.subtract(pose_xy(approach_a), pose_xy(crossing)))), 4)
        else:
            distances["approach_a_to_crossing_m"] = None
        if finite_pose(crossing) and finite_pose(approach_b):
            distances["approach_b_to_crossing_m"] = round(float(np.linalg.norm(np.subtract(pose_xy(approach_b), pose_xy(crossing)))), 4)
        else:
            distances["approach_b_to_crossing_m"] = None

        pose_distance_ok = all(v is not None and 0.2 <= v <= 0.8 for v in distances.values())
        pose_distance_status = "valid" if pose_distance_ok else "review"
        if not pose_distance_ok:
            validation_notes.append("one or more approach poses are outside the expected 0.2m to 0.8m crossing distance")

        side_distances = {
            "approach_from_room_a_to_room_a_m": bev.distance_to_room_m(room_a, pose_xy(approach_a)) if finite_pose(approach_a) else None,
            "approach_from_room_a_to_room_b_m": bev.distance_to_room_m(room_b, pose_xy(approach_a)) if finite_pose(approach_a) else None,
            "approach_from_room_b_to_room_a_m": bev.distance_to_room_m(room_a, pose_xy(approach_b)) if finite_pose(approach_b) else None,
            "approach_from_room_b_to_room_b_m": bev.distance_to_room_m(room_b, pose_xy(approach_b)) if finite_pose(approach_b) else None,
            "crossing_to_room_a_m": bev.distance_to_room_m(room_a, pose_xy(crossing)) if finite_pose(crossing) else None,
            "crossing_to_room_b_m": bev.distance_to_room_m(room_b, pose_xy(crossing)) if finite_pose(crossing) else None,
        }
        if not bev.available or not bev.has_layer("room_mask_global_id"):
            room_side_status = "unavailable"
        else:
            a_ok = (
                side_distances["approach_from_room_a_to_room_a_m"] is not None
                and side_distances["approach_from_room_a_to_room_b_m"] is not None
                and side_distances["approach_from_room_a_to_room_a_m"]
                <= side_distances["approach_from_room_a_to_room_b_m"] + 0.05
            )
            b_ok = (
                side_distances["approach_from_room_b_to_room_a_m"] is not None
                and side_distances["approach_from_room_b_to_room_b_m"] is not None
                and side_distances["approach_from_room_b_to_room_b_m"]
                <= side_distances["approach_from_room_b_to_room_a_m"] + 0.05
            )
            crossing_ok = (
                side_distances["crossing_to_room_a_m"] is not None
                and side_distances["crossing_to_room_b_m"] is not None
                and side_distances["crossing_to_room_a_m"] <= 0.65
                and side_distances["crossing_to_room_b_m"] <= 0.65
            )
            room_side_status = "valid" if a_ok and b_ok and crossing_ok else "review"
            if not a_ok:
                validation_notes.append("approach_from_room_a is not clearly closer to room_a mask cells")
            if not b_ok:
                validation_notes.append("approach_from_room_b is not clearly closer to room_b mask cells")
            if not crossing_ok:
                validation_notes.append("crossing pose is not close to both room masks")

        severe_statuses = {center_status, crossing_status, approach_a_status, approach_b_status}
        if any(str(status).startswith("invalid") for status in severe_statuses):
            overall = "invalid"
        elif (
            "review" in severe_statuses
            or room_side_status == "review"
            or pose_distance_status == "review"
            or any(str(status).startswith("inside_wall_layer") for status in wall_status.values())
        ):
            overall = "review"
        else:
            overall = "valid"

        records.append(
            {
                "gateway_id": gateway_id,
                "pair_key": edge["pair_key"],
                "room_a": room_a,
                "room_b": room_b,
                "center_xy_status": center_status,
                "crossing_pose_status": crossing_status,
                "approach_from_room_a_status": approach_a_status,
                "approach_from_room_b_status": approach_b_status,
                "room_side_consistency_status": room_side_status,
                "free_space_status": free_status,
                "wall_conflict_status": wall_status,
                "pose_distance_status": pose_distance_status,
                "pose_distances_m": distances,
                "room_mask_distances_m": side_distances,
                "passability_status_from_step30c": edge.get("passability_status"),
                "nav2_default_usable_from_step30c": edge.get("nav2_default_usable"),
                "review_required": bool(edge.get("review_required")),
                "overall_pose_validation_status": overall,
                "warnings": warnings,
                "validation_notes": validation_notes,
            }
        )

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_gateway_pose_validation",
        "step": STEP,
        "version": VERSION,
        "topology_source": "Step30C",
        "bev_layer_available": bev.available,
        "gateway_pose_validations": records,
        "summary": {
            "gateway_pose_count": len(records),
            "gateway_pose_valid_count": sum(1 for item in records if item["overall_pose_validation_status"] == "valid"),
            "gateway_pose_review_status_count": sum(1 for item in records if item["overall_pose_validation_status"] == "review"),
            "gateway_pose_invalid_count": sum(1 for item in records if item["overall_pose_validation_status"] == "invalid"),
            "gateway_pose_review_required_count": sum(1 for item in records if item["review_required"]),
            "passability_status_counts": dict(Counter(item["passability_status_from_step30c"] for item in records)),
        },
    }


def route_projection_points(route: Dict[str, Any]) -> List[List[float]]:
    points: List[List[float]] = []
    for approach in route.get("approach_pose_sequence", []):
        current = approach.get("approach_pose_from_current_room")
        crossing = route.get("crossing_pose_sequence", [])[int(approach["segment_index"])]
        nxt = approach.get("approach_pose_from_next_room")
        for pose in (current, crossing, nxt):
            if finite_pose(pose):
                xy = pose_xy(pose)
                if not points or np.linalg.norm(np.subtract(points[-1], xy)) > 1e-6:
                    points.append(xy)
    return points


def build_route_projection_review(routes: Dict[str, Any], gateway_validation: Dict[str, Any]) -> Dict[str, Any]:
    gateway_status = {
        item["gateway_id"]: item for item in gateway_validation.get("gateway_pose_validations", [])
    }
    records: List[Dict[str, Any]] = []
    for route in routes.get("route_candidates", []):
        validation_items = [gateway_status.get(gid) for gid in route.get("gateway_sequence", [])]
        missing_validations = [gid for gid, item in zip(route.get("gateway_sequence", []), validation_items) if item is None]
        invalid_items = [item for item in validation_items if item and item["overall_pose_validation_status"] == "invalid"]
        review_items = [item for item in validation_items if item and item["overall_pose_validation_status"] == "review"]
        projection_points = route_projection_points(route)
        if missing_validations or invalid_items or not projection_points:
            projection_status = "invalid"
        elif review_items or route.get("review_required_edges"):
            projection_status = "review"
        else:
            projection_status = "valid"

        records.append(
            {
                "route_id": route.get("route_id"),
                "room_sequence": route.get("room_sequence"),
                "gateway_sequence": route.get("gateway_sequence"),
                "crossing_pose_sequence": route.get("crossing_pose_sequence"),
                "approach_pose_sequence": route.get("approach_pose_sequence"),
                "projection_polyline_xy": projection_points,
                "graph_valid": route.get("graph_valid"),
                "nav2_default_usable": route.get("nav2_default_usable"),
                "review_required_edges": route.get("review_required_edges", []),
                "projection_status": projection_status,
                "pose_validation_status": {
                    "missing_gateway_pose_validations": missing_validations,
                    "invalid_gateway_pose_validations": [item["gateway_id"] for item in invalid_items],
                    "review_gateway_pose_validations": [item["gateway_id"] for item in review_items],
                },
                "notes": [
                    "Route projection uses approach and crossing poses, not committed room centroids.",
                    "Review-required passability edges are preserved and labeled.",
                ],
            }
        )

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_route_projection_review",
        "step": STEP,
        "version": VERSION,
        "topology_source": "Step30C",
        "topology_changed": False,
        "route_projections_are_offline_visualizations_only": True,
        "routes": records,
        "summary": {
            "route_count": len(records),
            "public_path_042_projection_available": any(
                item["route_id"] == "public_path_042" and item["projection_status"] != "invalid" for item in records
            ),
            "long_structure_route_default_projection_available": any(
                item["route_id"] == "long_structure_route_default" and item["projection_status"] != "invalid"
                for item in records
            ),
        },
    }


def corrected_overlay_payload(
    graph: Dict[str, Any],
    overlay: Dict[str, Any],
    edge_table: Dict[str, Any],
    room_validation: Dict[str, Any],
    routes_review: Dict[str, Any],
) -> Dict[str, Any]:
    anchors = {
        room_number(item["room_id"]): item for item in room_validation.get("rooms", [])
    }
    rooms = []
    for rid in sorted(anchors):
        item = anchors[rid]
        if item["corrected_anchor_xy"] is None:
            continue
        rooms.append(
            {
                "marker_id": room_id_text(rid),
                "marker_type": "room",
                "room_id": rid,
                "xy": item["corrected_anchor_xy"],
                "label": room_id_text(rid),
                "anchor_method": item["corrected_anchor_method"],
                "original_anchor_xy": item["original_anchor_xy"],
                "original_anchor_status": item["original_anchor_status"],
                "visualization_only": True,
            }
        )

    edge_segments = []
    for edge in edge_table.get("gateway_edges", []):
        for rid in (int(edge["room_a"]), int(edge["room_b"])):
            anchor = anchors.get(rid, {}).get("corrected_anchor_xy")
            if anchor is None or not finite_xy(edge.get("center_xy")):
                continue
            edge_segments.append(
                {
                    "segment_id": f"{room_id_text(rid)}__{edge['gateway_id']}",
                    "segment_type": "room_anchor_to_gateway_context",
                    "room_id": rid,
                    "gateway_id": edge["gateway_id"],
                    "pair_key": edge["pair_key"],
                    "points_xy": [anchor, edge["center_xy"]],
                    "passability_status": edge.get("passability_status"),
                    "nav2_default_usable": edge.get("nav2_default_usable"),
                    "visual_context_only": True,
                }
            )

    route_polylines = []
    for route in routes_review.get("routes", []):
        route_polylines.append(
            {
                "route_id": route["route_id"],
                "label": route["route_id"],
                "points_xy": route["projection_polyline_xy"],
                "gateway_sequence": route["gateway_sequence"],
                "graph_valid": route["graph_valid"],
                "nav2_default_usable": route["nav2_default_usable"],
                "review_required_edge_count": len(route["review_required_edges"]),
                "polyline_semantics": "approach_crossing_pose_projection_not_room_centroid_waypoints",
            }
        )

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_corrected_overlay_payload",
        "step": STEP,
        "version": VERSION,
        "coordinate_frame": overlay.get("coordinate_frame", "map_xy_meters"),
        "topology_source": "Step30C",
        "topology_changed": False,
        "topology_only": True,
        "nav2_route_generated": False,
        "robot_execution": False,
        "rooms": rooms,
        "gateway_markers": copy.deepcopy(overlay.get("gateway_markers", [])),
        "gateway_pose_markers": copy.deepcopy(edge_table.get("gateway_edges", [])),
        "edge_segments": edge_segments,
        "route_polylines": route_polylines,
        "marker_labels": copy.deepcopy(overlay.get("marker_labels", [])),
        "source_step30c_overlay_payload": rel(STEP30C_OVERLAY),
        "room_anchor_note": "Invalid committed centroids are not used as primary corrected anchors.",
    }


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
    ">": ["10000", "01000", "00100", "00010", "00100", "01000", "10000"],
    "|": ["00100", "00100", "00100", "00100", "00100", "00100", "00100"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}


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


class Canvas:
    def __init__(
        self,
        bev: BevContext,
        title: str,
        xlim: Tuple[float, float],
        ylim: Tuple[float, float],
        width: int = 1500,
        height: int = 1100,
    ) -> None:
        self.bev = bev
        self.title = title
        self.xlim = xlim
        self.ylim = ylim
        self.width = width
        self.height = height
        self.rgb = self._build_background()
        self.draw_text_px(18, 18, title.upper(), "#111111", scale=2, background="#ffffff")

    def _build_background(self) -> np.ndarray:
        x_min, x_max = self.xlim
        y_min, y_max = self.ylim
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[:] = hex_to_rgb("#f5f5f1")
        if not self.bev.available:
            return rgb

        xs = np.linspace(x_min, x_max, self.width)
        ys = np.linspace(y_max, y_min, self.height)
        cols = np.rint((xs - self.bev.origin[0]) / self.bev.resolution).astype(int)
        rows = np.rint((ys - self.bev.origin[1]) / self.bev.resolution).astype(int)
        cc, rr = np.meshgrid(cols, rows)
        valid = (rr >= 0) & (rr < self.bev.height) & (cc >= 0) & (cc < self.bev.width)
        rr_safe = np.clip(rr, 0, max(0, self.bev.height - 1))
        cc_safe = np.clip(cc, 0, max(0, self.bev.width - 1))

        if self.bev.has_layer("unknown_layer"):
            rgb[(self.bev.layers["unknown_layer"][rr_safe, cc_safe] > 0) & valid] = hex_to_rgb("#dedede")
        if self.bev.has_layer("outside_boundary"):
            rgb[(self.bev.layers["outside_boundary"][rr_safe, cc_safe] > 0) & valid] = hex_to_rgb("#cccccc")
        if self.bev.has_layer("free_space"):
            rgb[(self.bev.layers["free_space"][rr_safe, cc_safe] > 0) & valid] = hex_to_rgb("#ffffff")
        if self.bev.has_layer("room_mask_global_id"):
            room_mask = self.bev.layers["room_mask_global_id"][rr_safe, cc_safe]
            colors = {
                1: "#b6def4",
                3: "#cae8ba",
                7: "#f7c5be",
                8: "#dcc7f2",
                11: "#f7dfa2",
                14: "#b8e7dc",
                15: "#edc7db",
                16: "#ddddb5",
            }
            for rid, color in colors.items():
                mask = (room_mask == rid) & valid
                if np.any(mask):
                    rgb[mask] = (rgb[mask].astype(float) * 0.55 + hex_to_rgb(color).astype(float) * 0.45).astype(np.uint8)
        if self.bev.has_layer("segmentation_wall_processed"):
            rgb[(self.bev.layers["segmentation_wall_processed"][rr_safe, cc_safe] > 0) & valid] = hex_to_rgb("#222222")
        if self.bev.has_layer("gateway_wall_preclose"):
            mask = (self.bev.layers["gateway_wall_preclose"][rr_safe, cc_safe] > 0) & valid
            if np.any(mask):
                rgb[mask] = (rgb[mask].astype(float) * 0.35 + hex_to_rgb("#335cd9").astype(float) * 0.65).astype(np.uint8)
        return rgb

    def world_to_px(self, xy: Sequence[float]) -> Tuple[int, int]:
        x_min, x_max = self.xlim
        y_min, y_max = self.ylim
        px = int(round((float(xy[0]) - x_min) / (x_max - x_min) * (self.width - 1)))
        py = int(round((y_max - float(xy[1])) / (y_max - y_min) * (self.height - 1)))
        return px, py

    def blend_px(self, x: int, y: int, color: str, alpha: float = 1.0) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            c = hex_to_rgb(color).astype(float)
            self.rgb[y, x] = np.clip(self.rgb[y, x].astype(float) * (1.0 - alpha) + c * alpha, 0, 255).astype(np.uint8)

    def draw_line_px(self, p0: Tuple[int, int], p1: Tuple[int, int], color: str, width: int = 1, alpha: float = 1.0) -> None:
        x0, y0 = p0
        x1, y1 = p1
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        xs = np.linspace(x0, x1, steps + 1)
        ys = np.linspace(y0, y1, steps + 1)
        r = max(0, width // 2)
        for x_f, y_f in zip(xs, ys):
            x, y = int(round(x_f)), int(round(y_f))
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r + 1:
                        self.blend_px(x + dx, y + dy, color, alpha)

    def draw_line_world(self, xy0: Sequence[float], xy1: Sequence[float], color: str, width: int = 1, alpha: float = 1.0) -> None:
        self.draw_line_px(self.world_to_px(xy0), self.world_to_px(xy1), color, width=width, alpha=alpha)

    def draw_arrow_world(self, xy0: Sequence[float], xy1: Sequence[float], color: str, width: int = 3, alpha: float = 1.0) -> None:
        p0 = self.world_to_px(xy0)
        p1 = self.world_to_px(xy1)
        self.draw_line_px(p0, p1, color, width=width, alpha=alpha)
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dx, dy)
        if length < 1:
            return
        ux, uy = dx / length, dy / length
        left = (int(round(p1[0] - ux * 16 - uy * 7)), int(round(p1[1] - uy * 16 + ux * 7)))
        right = (int(round(p1[0] - ux * 16 + uy * 7)), int(round(p1[1] - uy * 16 - ux * 7)))
        self.draw_line_px(p1, left, color, width=width, alpha=alpha)
        self.draw_line_px(p1, right, color, width=width, alpha=alpha)

    def draw_circle_world(self, xy: Sequence[float], radius_px: int, fill: str, outline: Optional[str] = None, alpha: float = 1.0) -> None:
        cx, cy = self.world_to_px(xy)
        for y in range(cy - radius_px, cy + radius_px + 1):
            for x in range(cx - radius_px, cx + radius_px + 1):
                d2 = (x - cx) ** 2 + (y - cy) ** 2
                if d2 <= radius_px * radius_px:
                    self.blend_px(x, y, fill, alpha)
        if outline:
            for angle in np.linspace(0, 2 * math.pi, 72):
                self.blend_px(int(round(cx + math.cos(angle) * radius_px)), int(round(cy + math.sin(angle) * radius_px)), outline, 1.0)

    def draw_cross_world(self, xy: Sequence[float], color: str, size: int = 8, width: int = 2, alpha: float = 1.0) -> None:
        x, y = self.world_to_px(xy)
        self.draw_line_px((x - size, y - size), (x + size, y + size), color, width=width, alpha=alpha)
        self.draw_line_px((x - size, y + size), (x + size, y - size), color, width=width, alpha=alpha)

    def draw_text_px(
        self,
        x: int,
        y: int,
        text: str,
        color: str,
        scale: int = 2,
        background: Optional[str] = None,
        alpha: float = 1.0,
    ) -> None:
        lines = text.upper().split("\n")
        line_h = 8 * scale
        max_w = max((len(line) * 6 * scale for line in lines), default=0)
        if background:
            for yy in range(y - 3, y + line_h * len(lines) + 3):
                for xx in range(x - 3, x + max_w + 3):
                    self.blend_px(xx, yy, background, 0.78)
        for line_idx, line in enumerate(lines):
            cursor_x = x
            cursor_y = y + line_idx * line_h
            for ch in line:
                glyph = FONT_5X7.get(ch, FONT_5X7[" "])
                for gy, row in enumerate(glyph):
                    for gx, bit in enumerate(row):
                        if bit == "1":
                            for sy in range(scale):
                                for sx in range(scale):
                                    self.blend_px(cursor_x + gx * scale + sx, cursor_y + gy * scale + sy, color, alpha)
                cursor_x += 6 * scale

    def draw_text_world(
        self,
        xy: Sequence[float],
        text: str,
        color: str,
        dx: int = 8,
        dy: int = -8,
        scale: int = 2,
        background: Optional[str] = "#ffffff",
    ) -> None:
        x, y = self.world_to_px(xy)
        self.draw_text_px(x + dx, y + dy, text, color, scale=scale, background=background)

    def save(self, path: Path) -> None:
        write_png(path, self.rgb)


class Visualizer:
    def __init__(
        self,
        bev: BevContext,
        corrected_payload: Dict[str, Any],
        edge_table: Dict[str, Any],
        routes_review: Dict[str, Any],
    ) -> None:
        self.bev = bev
        self.corrected_payload = corrected_payload
        self.gateway_edges = {edge["gateway_id"]: edge for edge in edge_table.get("gateway_edges", [])}
        self.routes = {route["route_id"]: route for route in routes_review.get("routes", [])}
        self.manifest: List[Dict[str, Any]] = []

    def add_manifest(
        self,
        path: Path,
        visualization_type: str,
        route_id: Optional[str] = None,
        gateway_id: Optional[str] = None,
        ok: bool = True,
        notes: Optional[List[str]] = None,
    ) -> None:
        self.manifest.append(
            {
                "file_path": rel(path),
                "visualization_type": visualization_type,
                "route_id": route_id,
                "gateway_id": gateway_id,
                "generated_successfully": ok and path.exists(),
                "notes": notes or [],
            }
        )

    def canvas_bounds(self, extra_points: Optional[List[List[float]]] = None, margin: float = 1.2) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        all_points: List[List[float]] = []
        for room in self.corrected_payload.get("rooms", []):
            all_points.append(room["xy"])
            if finite_xy(room.get("original_anchor_xy")):
                all_points.append(room["original_anchor_xy"])
        for edge in self.gateway_edges.values():
            all_points.append(edge["center_xy"])
            for key in ("crossing_pose", "approach_from_room_a", "approach_from_room_b"):
                if finite_pose(edge.get(key)):
                    all_points.append(pose_xy(edge[key]))
        if extra_points:
            all_points.extend(extra_points)
        arr = np.asarray(all_points, dtype=float)
        return (
            (float(arr[:, 0].min()) - margin, float(arr[:, 0].max()) + margin),
            (float(arr[:, 1].min()) - margin, float(arr[:, 1].max()) + margin),
        )

    def setup_canvas(
        self,
        title: str,
        crop: Optional[Tuple[float, float, float, float]] = None,
        extra_points: Optional[List[List[float]]] = None,
        width: int = 1500,
        height: int = 1100,
    ) -> Canvas:
        if crop:
            xlim = (crop[0], crop[1])
            ylim = (crop[2], crop[3])
        else:
            xlim, ylim = self.canvas_bounds(extra_points=extra_points)
        return Canvas(self.bev, title, xlim, ylim, width=width, height=height)

    def draw_rooms(self, canvas: Canvas) -> None:
        for room in self.corrected_payload.get("rooms", []):
            x, y = room["xy"]
            canvas.draw_circle_world([x, y], 8, "#ffffff", outline="#222222")
            canvas.draw_text_world([x, y], room["label"], "#111111", dx=10, dy=-12, scale=2)
            original = room.get("original_anchor_xy")
            if finite_xy(original) and room.get("original_anchor_status") != "valid":
                canvas.draw_cross_world(original, "#777777", size=7, width=2, alpha=0.45)

    def draw_gateways(self, canvas: Canvas, label: bool = True) -> None:
        for edge in self.gateway_edges.values():
            x, y = edge["center_xy"]
            color = PASSABILITY_COLORS.get(edge.get("passability_status"), "#555555")
            canvas.draw_circle_world([x, y], 9 if edge.get("review_required") else 7, color, outline="#111111")
            if edge.get("review_required"):
                canvas.draw_cross_world([x, y], "#111111", size=6, width=1)
            if label:
                label_text = f"{edge['pair_key']} {edge.get('passability_status')}"
                canvas.draw_text_world([x, y], label_text, color, dx=10, dy=12, scale=1)

    def draw_context_edges(self, canvas: Canvas) -> None:
        for segment in self.corrected_payload.get("edge_segments", []):
            p0, p1 = segment["points_xy"]
            color = PASSABILITY_COLORS.get(segment.get("passability_status"), "#777777")
            canvas.draw_line_world(p0, p1, color, width=2, alpha=0.28)

    def draw_gateway_pose(self, canvas: Canvas, edge: Dict[str, Any], alpha: float = 1.0) -> None:
        color = PASSABILITY_COLORS.get(edge.get("passability_status"), "#555555")
        crossing = edge.get("crossing_pose")
        app_a = edge.get("approach_from_room_a")
        app_b = edge.get("approach_from_room_b")
        if finite_pose(app_a) and finite_pose(crossing):
            canvas.draw_arrow_world(pose_xy(app_a), pose_xy(crossing), color, width=3, alpha=alpha)
        if finite_pose(app_b) and finite_pose(crossing):
            canvas.draw_arrow_world(pose_xy(app_b), pose_xy(crossing), color, width=3, alpha=alpha)
        if finite_pose(app_a):
            canvas.draw_circle_world(pose_xy(app_a), 5, color, outline="#111111", alpha=alpha)
        if finite_pose(app_b):
            canvas.draw_circle_world(pose_xy(app_b), 5, color, outline="#111111", alpha=alpha)
        if finite_pose(crossing):
            canvas.draw_cross_world(pose_xy(crossing), color, size=9, width=3, alpha=alpha)

    def draw_route(self, canvas: Canvas, route: Dict[str, Any], upto_segment: Optional[int] = None) -> None:
        points = route.get("projection_polyline_xy") or []
        edge_count = len(route.get("gateway_sequence") or [])
        point_limit = len(points)
        if upto_segment is not None:
            point_limit = min(len(points), (upto_segment + 1) * 3)
        if point_limit >= 2:
            for p0, p1 in zip(points[: point_limit - 1], points[1:point_limit]):
                canvas.draw_line_world(p0, p1, "#111111", width=6, alpha=0.70)
                canvas.draw_line_world(p0, p1, "#ffffff", width=2, alpha=0.90)

        for idx, gid in enumerate(route.get("gateway_sequence") or []):
            if upto_segment is not None and idx > upto_segment:
                continue
            edge = self.gateway_edges.get(gid)
            if not edge:
                continue
            self.draw_gateway_pose(canvas, edge, alpha=1.0 if upto_segment is None or idx == upto_segment else 0.55)
            if edge.get("review_required"):
                canvas.draw_text_world(edge["center_xy"], "REVIEW", "#b00020", dx=12, dy=-28, scale=1)

        canvas.draw_text_px(
            18,
            50,
            f"{route['route_id']} | segments {edge_count} | projection: approach/crossing poses only",
            "#111111",
            scale=1,
            background="#ffffff",
        )

    def save_canvas(
        self,
        canvas: Canvas,
        path: Path,
        visualization_type: str,
        route_id: Optional[str] = None,
        gateway_id: Optional[str] = None,
    ) -> None:
        canvas.save(path)
        self.add_manifest(path, visualization_type, route_id=route_id, gateway_id=gateway_id)

    def static_topology_overlay(self) -> None:
        path = VIZ_ROOT / f"{SHORT}_step30d_gateway_topology_overlay_corrected_anchors.png"
        canvas = self.setup_canvas("Step30D gateway topology overlay with corrected room anchors")
        self.draw_context_edges(canvas)
        self.draw_rooms(canvas)
        self.draw_gateways(canvas)
        self.save_canvas(canvas, path, "static_gateway_topology_overlay_corrected_anchors")

    def route_overlay(self, route_id: str, filename: str, title: str, visualization_type: str) -> None:
        route = self.routes.get(route_id)
        path = VIZ_ROOT / filename
        if route is None:
            self.add_manifest(path, visualization_type, route_id=route_id, ok=False, notes=["route not present in Step30C route candidates"])
            return
        canvas = self.setup_canvas(title, extra_points=route.get("projection_polyline_xy") or [])
        self.draw_context_edges(canvas)
        self.draw_rooms(canvas)
        self.draw_gateways(canvas, label=False)
        self.draw_route(canvas, route)
        self.save_canvas(canvas, path, visualization_type, route_id=route_id)

    def local_gateway_crops(self) -> None:
        for pair_key in EXPECTED_GATEWAY_PAIRS:
            edges = [edge for edge in self.gateway_edges.values() if edge["pair_key"] == pair_key]
            if not edges:
                continue
            edge = edges[0]
            center = edge["center_xy"]
            crop = (center[0] - 1.4, center[0] + 1.4, center[1] - 1.4, center[1] + 1.4)
            canvas = self.setup_canvas(f"Step30D local gateway crop {pair_key}", crop=crop, width=900, height=900)
            self.draw_gateway_pose(canvas, edge)
            self.draw_gateways(canvas)
            canvas.draw_text_px(
                18,
                50,
                f"{edge['gateway_id']}\n{edge.get('passability_status')} | review={edge.get('review_required')}",
                "#111111",
                scale=1,
                background="#ffffff",
            )
            path = VIZ_ROOT / "local_gateway_crops" / f"{SHORT}_step30d_local_crop_{pair_key}.png"
            self.save_canvas(canvas, path, "local_gateway_crop", gateway_id=edge["gateway_id"])

    def replay_frames(self, route_id: str, frame_dir_name: str, gif_name: str) -> None:
        route = self.routes.get(route_id)
        frame_dir = VIZ_ROOT / frame_dir_name
        if route is None:
            self.add_manifest(frame_dir / "missing_route.txt", "replay_frames", route_id=route_id, ok=False, notes=["route not present"])
            return
        frames: List[Path] = []
        gateways = route.get("gateway_sequence") or []
        edges_by_index = route.get("approach_pose_sequence") or []
        for idx, gid in enumerate(gateways):
            edge = self.gateway_edges.get(gid, {})
            canvas = self.setup_canvas(
                f"Step30D replay {route_id}: segment {idx + 1}/{len(gateways)}",
                extra_points=route.get("projection_polyline_xy") or [],
            )
            self.draw_context_edges(canvas)
            self.draw_rooms(canvas)
            self.draw_gateways(canvas, label=False)
            self.draw_route(canvas, route, upto_segment=idx)
            segment = edges_by_index[idx] if idx < len(edges_by_index) else {}
            canvas.draw_text_px(
                18,
                82,
                "\n".join(
                    [
                        f"segment_index: {idx}",
                        f"{segment.get('from_room')} -> {segment.get('to_room')}",
                        f"gateway: {gid}",
                        f"passability: {edge.get('passability_status')}",
                        f"review_required: {edge.get('review_required')}",
                    ]
                ),
                "#111111",
                scale=1,
                background="#ffffff",
            )
            frame_path = frame_dir / f"frame_{idx:03d}.png"
            self.save_canvas(canvas, frame_path, "replay_frame", route_id=route_id, gateway_id=gid)
            frames.append(frame_path)

        gif_path = VIZ_ROOT / gif_name
        self.add_manifest(
            gif_path,
            "replay_gif",
            route_id=route_id,
            ok=False,
            notes=["GIF generation dependency unavailable in this environment; frame sequence generated"],
        )

    def generate_all(self) -> Dict[str, Any]:
        self.static_topology_overlay()
        self.route_overlay(
            "public_path_042",
            f"{SHORT}_step30d_public_path_042_overlay.png",
            "Step30D public_path_042 route projection",
            "static_route_overlay",
        )
        self.route_overlay(
            "long_structure_route_default",
            f"{SHORT}_step30d_long_structure_route_default_overlay.png",
            "Step30D long_structure_route_default route projection",
            "static_route_overlay",
        )
        self.route_overlay(
            OPTIONAL_REVIEW_ROUTE,
            f"{SHORT}_step30d_review_all_positive_gateways_overlay.png",
            "Step30D review-all-positive-gateways route projection",
            "static_review_route_overlay",
        )
        self.local_gateway_crops()
        self.replay_frames(
            "public_path_042",
            "replay_public_path_042_frames",
            f"{SHORT}_step30d_public_path_042_replay.gif",
        )
        self.replay_frames(
            "long_structure_route_default",
            "replay_long_structure_route_default_frames",
            f"{SHORT}_step30d_long_structure_route_default_replay.gif",
        )
        return {
            "scene_id": SCENE_ID,
            "artifact_type": "step30d_visualization_manifest",
            "step": STEP,
            "version": VERSION,
            "visualizations": self.manifest,
            "summary": {
                "visualization_count": len(self.manifest),
                "static_overlay_count": sum(
                    1
                    for item in self.manifest
                    if item["generated_successfully"] and item["visualization_type"].startswith("static")
                ),
                "replay_frame_count": sum(
                    1 for item in self.manifest if item["generated_successfully"] and item["visualization_type"] == "replay_frame"
                ),
                "gif_count": sum(
                    1 for item in self.manifest if item["generated_successfully"] and item["visualization_type"] == "replay_gif"
                ),
            },
        }


def validation_results(
    graph: Dict[str, Any],
    edge_table: Dict[str, Any],
    routes: Dict[str, Any],
    room_validation: Dict[str, Any],
    gateway_validation: Dict[str, Any],
    routes_review: Dict[str, Any],
    corrected_payload: Dict[str, Any],
    manifest: Dict[str, Any],
) -> Dict[str, Any]:
    step30c_pairs = sorted(edge["pair_key"] for edge in edge_table.get("gateway_edges", []))
    route_by_id = {route["route_id"]: route for route in routes_review.get("routes", [])}

    def check(name: str, passed: bool, details: Dict[str, Any]) -> Dict[str, Any]:
        return {"check": name, "passed": bool(passed), "details": details}

    invalid_original_using_centroid = [
        room["room_id"]
        for room in room_validation.get("rooms", [])
        if room["original_anchor_status"] == "invalid_committed_centroid"
        and room["corrected_anchor_method"] in {"committed_centroid", "original_overlay_anchor"}
    ]
    all_manifest_files_exist = all(
        (REPO_ROOT / item["file_path"]).exists()
        for item in manifest.get("visualizations", [])
        if item.get("generated_successfully")
    )
    review_edges = [
        edge
        for edge in edge_table.get("gateway_edges", [])
        if edge.get("review_required") or edge.get("passability_status") in {"reviewable", "low_or_uncertain"}
    ]
    labeled_review_edges = [
        edge
        for edge in gateway_validation.get("gateway_pose_validations", [])
        if edge.get("review_required") or edge.get("passability_status_from_step30c") in {"reviewable", "low_or_uncertain"}
    ]

    checks = [
        check(
            "step30c_topology_was_not_changed",
            graph.get("step") == "step30c" and sorted(step30c_pairs) == sorted(EXPECTED_GATEWAY_PAIRS) and "r3_r11" not in step30c_pairs,
            {"step30c_pair_keys": step30c_pairs, "expected_pair_keys": sorted(EXPECTED_GATEWAY_PAIRS), "r3_r11_absent": "r3_r11" not in step30c_pairs},
        ),
        check("no_stage_a_rerun", True, {"stage_a_rerun_attempted": False}),
        check("no_gateway_extraction_rerun", True, {"gateway_extraction_rerun_attempted": False}),
        check("no_ros_nav2_gazebo_run", True, {"ros_nav2_gazebo_run": False}),
        check("no_nav2_route_generated", True, {"nav2_route_generated": False}),
        check("room_anchor_validation_exists", len(room_validation.get("rooms", [])) > 0, {"room_count": len(room_validation.get("rooms", []))}),
        check(
            "invalid_room_centroids_not_used_as_primary_corrected_anchors",
            not invalid_original_using_centroid,
            {"offending_room_ids": invalid_original_using_centroid},
        ),
        check(
            "gateway_pose_validation_exists_for_all_8_gateway_edges",
            len(gateway_validation.get("gateway_pose_validations", [])) == 8,
            {"gateway_pose_validation_count": len(gateway_validation.get("gateway_pose_validations", []))},
        ),
        check(
            "public_path_042_route_projection_exists",
            route_by_id.get("public_path_042", {}).get("projection_status") != "invalid",
            {"projection_status": route_by_id.get("public_path_042", {}).get("projection_status")},
        ),
        check(
            "long_structure_route_default_route_projection_exists",
            route_by_id.get("long_structure_route_default", {}).get("projection_status") != "invalid",
            {"projection_status": route_by_id.get("long_structure_route_default", {}).get("projection_status")},
        ),
        check(
            "static_overlay_images_exist",
            manifest.get("summary", {}).get("static_overlay_count", 0) >= 4,
            {"static_overlay_count": manifest.get("summary", {}).get("static_overlay_count", 0)},
        ),
        check(
            "replay_frames_or_gif_exist_for_required_routes",
            all(
                any(
                    item.get("generated_successfully")
                    and item.get("route_id") == route_id
                    and item.get("visualization_type") in {"replay_frame", "replay_gif"}
                    for item in manifest.get("visualizations", [])
                )
                for route_id in REQUIRED_ROUTES
            ),
            {"required_routes": REQUIRED_ROUTES},
        ),
        check(
            "corrected_overlay_payload_exists",
            bool(corrected_payload.get("rooms")) and bool(corrected_payload.get("route_polylines")),
            {"room_marker_count": len(corrected_payload.get("rooms", [])), "route_polyline_count": len(corrected_payload.get("route_polylines", []))},
        ),
        check("every_visualization_file_listed_in_manifest_exists", all_manifest_files_exist, {}),
        check(
            "review_required_passability_edges_preserved_and_labeled",
            len(review_edges) == len(labeled_review_edges) and len(review_edges) > 0,
            {
                "step30c_review_required_or_low_passability_count": len(review_edges),
                "step30d_labeled_review_required_or_low_passability_count": len(labeled_review_edges),
                "pair_keys": [edge["pair_key"] for edge in review_edges],
            },
        ),
    ]
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_validation_results",
        "step": STEP,
        "version": VERSION,
        "checks": checks,
        "validation_passed": all(item["passed"] for item in checks),
    }


def build_summary(
    room_validation: Dict[str, Any],
    gateway_validation: Dict[str, Any],
    routes_review: Dict[str, Any],
    manifest: Dict[str, Any],
    validation: Dict[str, Any],
    bev: BevContext,
) -> Dict[str, Any]:
    room_summary = room_validation.get("summary", {})
    gateway_summary = gateway_validation.get("summary", {})
    route_summary = routes_review.get("summary", {})
    manifest_summary = manifest.get("summary", {})
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30d_summary",
        "step": STEP,
        "version": VERSION,
        "stage_a_rerun_attempted": False,
        "gateway_extraction_rerun_attempted": False,
        "topology_changed": False,
        "topology_source": "Step30C",
        "ros_nav2_gazebo_run": False,
        "nav2_route_generated": False,
        "robot_execution": False,
        "bev_npz_used": rel(bev.npz_path) if bev.available else None,
        "bev_layers_available": sorted(bev.layers) if bev.available else [],
        "room_anchor_count": room_summary.get("room_anchor_count", 0),
        "invalid_original_room_anchor_count": room_summary.get("invalid_original_room_anchor_count", 0),
        "corrected_room_anchor_count": room_summary.get("corrected_room_anchor_count", 0),
        "corrected_anchor_method_counts": room_summary.get("corrected_anchor_method_counts", {}),
        "gateway_pose_count": gateway_summary.get("gateway_pose_count", 0),
        "gateway_pose_valid_count": gateway_summary.get("gateway_pose_valid_count", 0),
        "gateway_pose_review_status_count": gateway_summary.get("gateway_pose_review_status_count", 0),
        "gateway_pose_invalid_count": gateway_summary.get("gateway_pose_invalid_count", 0),
        "gateway_pose_review_required_count": gateway_summary.get("gateway_pose_review_required_count", 0),
        "public_path_042_projection_available": route_summary.get("public_path_042_projection_available", False),
        "long_structure_route_default_projection_available": route_summary.get("long_structure_route_default_projection_available", False),
        "static_overlay_count": manifest_summary.get("static_overlay_count", 0),
        "replay_frame_count": manifest_summary.get("replay_frame_count", 0),
        "gif_count": manifest_summary.get("gif_count", 0),
        "validation_passed": validation.get("validation_passed", False),
        "manual_review_checklist": MANUAL_REVIEW_CHECKLIST,
    }


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    VIZ_ROOT.mkdir(parents=True, exist_ok=True)

    graph = read_json(STEP30C_GRAPH)
    edge_table = read_json(STEP30C_EDGE_TABLE)
    routes = read_json(STEP30C_ROUTES)
    overlay = read_json(STEP30C_OVERLAY)
    step30c_validation = read_json(STEP30C_VALIDATION)
    step30c_summary = read_json(STEP30C_SUMMARY)
    bev = BevContext(STEP30A_LAYERED_BEV_NPZ, STEP30A_LAYERED_BEV_JSON)

    room_validation = validate_room_anchors(graph, overlay, edge_table, bev)
    gateway_validation = validate_gateway_poses(edge_table, bev)
    routes_review = build_route_projection_review(routes, gateway_validation)
    corrected_payload = corrected_overlay_payload(graph, overlay, edge_table, room_validation, routes_review)

    visualizer = Visualizer(bev, corrected_payload, edge_table, routes_review)
    manifest = visualizer.generate_all()

    validation = validation_results(
        graph,
        edge_table,
        routes,
        room_validation,
        gateway_validation,
        routes_review,
        corrected_payload,
        manifest,
    )
    summary = build_summary(room_validation, gateway_validation, routes_review, manifest, validation, bev)

    provenance = {
        "source_step30c_graph_validation_passed": step30c_validation.get("validation_passed"),
        "source_step30c_summary_validation_passed": step30c_summary.get("validation_passed"),
        "source_artifacts": {
            "step30c_graph": rel(STEP30C_GRAPH),
            "step30c_edge_table": rel(STEP30C_EDGE_TABLE),
            "step30c_routes": rel(STEP30C_ROUTES),
            "step30c_overlay": rel(STEP30C_OVERLAY),
            "step30c_validation": rel(STEP30C_VALIDATION),
            "step30c_summary": rel(STEP30C_SUMMARY),
            "step30a_layered_bev_npz": rel(STEP30A_LAYERED_BEV_NPZ) if bev.available else None,
            "step30a_layered_bev_json": rel(STEP30A_LAYERED_BEV_JSON) if STEP30A_LAYERED_BEV_JSON.exists() else None,
        },
    }
    for payload in (room_validation, gateway_validation, routes_review, corrected_payload, manifest, validation, summary):
        payload["provenance"] = provenance

    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_room_anchor_validation_{VERSION}.json", room_validation)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_gateway_pose_validation_{VERSION}.json", gateway_validation)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_route_projection_review_{VERSION}.json", routes_review)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_corrected_overlay_payload_{VERSION}.json", corrected_payload)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_visualization_manifest_{VERSION}.json", manifest)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_validation_results_{VERSION}.json", validation)
    write_json(OUTPUT_ROOT / f"{SHORT}_step30d_summary_{VERSION}.json", summary)

    print(json.dumps(summary, indent=2, sort_keys=False))


if __name__ == "__main__":
    main()
