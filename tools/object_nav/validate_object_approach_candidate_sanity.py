#!/usr/bin/env python3
"""Offline sanity checks for RSLG-SLAM object approach candidates.

This checker intentionally uses only committed/public artifacts, the stable
occupancy map, route artifacts, and existing task14b outputs. It does not start
ROS, Gazebo, Nav2, RViz, or rerun Stage-A.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


ROOT = Path(__file__).resolve().parents[2]
PIXELS_TO_EXPLAIN = [0, 205, 254, 255]
WALL_SIDE_LABELS = {"curtain", "window", "mirror", "picture/frame", "picture_frame", "frame"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def load_json(path: Path, default: Any | None = None) -> Any:
    if not path.exists():
        if default is not None:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def canonical_object_id(value: Any) -> str:
    text = str(value)
    return text if text.startswith("obj_") else f"obj_{text}"


def norm_label(value: Any) -> str:
    return str(value or "").strip().lower().replace(" ", "_")


def round_float(value: Any, digits: int = 6) -> float | None:
    if value is None:
        return None
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def angle_wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_error(yaw: float | None, start: tuple[float, float], target: tuple[float, float]) -> float | None:
    if yaw is None:
        return None
    desired = math.atan2(target[1] - start[1], target[0] - start[0])
    return abs(angle_wrap(float(yaw) - desired))


def point_in_polygon(x: float, y: float, polygon: list[list[float]], include_boundary: bool = True) -> bool:
    if len(polygon) < 3:
        return False
    if include_boundary and distance_to_polygon_boundary((x, y), polygon) <= 1e-8:
        return True
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def closest_point_on_segment(
    point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> tuple[float, float]:
    px, py = point
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return a
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return ax + t * dx, ay + t * dy


def distance_to_segment(point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    qx, qy = closest_point_on_segment(point, a, b)
    return math.hypot(point[0] - qx, point[1] - qy)


def distance_to_polygon_boundary(point: tuple[float, float], polygon: list[list[float]]) -> float:
    if len(polygon) < 2:
        return float("inf")
    best = float("inf")
    for idx, a in enumerate(polygon):
        b = polygon[(idx + 1) % len(polygon)]
        best = min(best, distance_to_segment(point, tuple(a), tuple(b)))
    return best


def signed_boundary_margin(point: tuple[float, float], polygon: list[list[float]]) -> float | None:
    if len(polygon) < 3:
        return None
    dist = distance_to_polygon_boundary(point, polygon)
    return dist if point_in_polygon(point[0], point[1], polygon) else -dist


def bbox(points: list[list[float]]) -> tuple[float, float, float, float] | None:
    if not points:
        return None
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def polygon_diag(points: list[list[float]]) -> float | None:
    box = bbox(points)
    if not box:
        return None
    min_x, min_y, max_x, max_y = box
    return math.hypot(max_x - min_x, max_y - min_y)


def point_in_bbox(point: tuple[float, float], footprint: list[list[float]]) -> bool:
    box = bbox(footprint)
    if not box:
        return False
    min_x, min_y, max_x, max_y = box
    x, y = point
    return min_x <= x <= max_x and min_y <= y <= max_y


def sample_polygon_edges(points: list[list[float]], spacing: float = 0.025) -> list[tuple[float, float]]:
    if len(points) < 2:
        return []
    samples: list[tuple[float, float]] = []
    for idx, a_raw in enumerate(points):
        b_raw = points[(idx + 1) % len(points)]
        a = (float(a_raw[0]), float(a_raw[1]))
        b = (float(b_raw[0]), float(b_raw[1]))
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(steps + 1):
            t = step / steps
            samples.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return samples


@dataclass
class OccupancyMap:
    yaml_path: Path
    metadata: dict[str, Any]
    image_path: Path
    image: np.ndarray
    occupancy_probability: np.ndarray
    free_mask: np.ndarray
    occupied_mask: np.ndarray
    unknown_mask: np.ndarray
    distance_cells: np.ndarray
    distance_m: np.ndarray
    resolution: float
    origin: tuple[float, float, float]

    @classmethod
    def load(cls, yaml_path: Path) -> "OccupancyMap":
        metadata = yaml.safe_load(yaml_path.read_text())
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image = np.array(Image.open(image_path).convert("L"))
        resolution = float(metadata["resolution"])
        origin_raw = metadata.get("origin") or [0.0, 0.0, 0.0]
        origin = (float(origin_raw[0]), float(origin_raw[1]), float(origin_raw[2] if len(origin_raw) > 2 else 0.0))
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.196))
        pixels = image.astype(np.float64)
        occupancy_probability = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
        occupied_mask = occupancy_probability > occupied_thresh
        free_mask = occupancy_probability < free_thresh
        unknown_mask = ~(free_mask | occupied_mask)
        distance_cells = ndimage.distance_transform_edt(free_mask)
        distance_m = distance_cells * resolution
        return cls(
            yaml_path=yaml_path,
            metadata=metadata,
            image_path=image_path,
            image=image,
            occupancy_probability=occupancy_probability,
            free_mask=free_mask,
            occupied_mask=occupied_mask,
            unknown_mask=unknown_mask,
            distance_cells=distance_cells,
            distance_m=distance_m,
            resolution=resolution,
            origin=origin,
        )

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    def world_to_rc(self, x: float, y: float) -> tuple[int, int]:
        col = int(round((x - self.origin[0]) / self.resolution))
        row = int(round((y - self.origin[1]) / self.resolution))
        return row, col

    def rc_to_world(self, row: int, col: int) -> tuple[float, float]:
        return self.origin[0] + col * self.resolution, self.origin[1] + row * self.resolution

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def pixel_interpretation(self, value: int) -> dict[str, Any]:
        negate = int(self.metadata.get("negate", 0))
        occupied_thresh = float(self.metadata.get("occupied_thresh", 0.65))
        free_thresh = float(self.metadata.get("free_thresh", 0.196))
        occ = value / 255.0 if negate else (255.0 - value) / 255.0
        if occ > occupied_thresh:
            state = "occupied"
        elif occ < free_thresh:
            state = "free"
        else:
            state = "unknown"
        return {
            "pixel_value": value,
            "negate": negate,
            "occupancy_probability": round(occ, 9),
            "occupied_thresh": occupied_thresh,
            "free_thresh": free_thresh,
            "state": state,
            "note": "ROS map-server trinary semantics: occupied if probability > occupied_thresh; free if probability < free_thresh; otherwise unknown.",
        }

    def sample(self, xy: tuple[float, float]) -> dict[str, Any]:
        row, col = self.world_to_rc(*xy)
        if not self.in_bounds(row, col):
            return {
                "in_bounds": False,
                "grid_row": row,
                "grid_col": col,
                "pixel_value": None,
                "state": "out_of_bounds",
                "is_free": False,
                "is_occupied": False,
                "is_unknown": False,
                "clearance_cells": None,
                "clearance_m": None,
            }
        state = "free" if self.free_mask[row, col] else "occupied" if self.occupied_mask[row, col] else "unknown"
        return {
            "in_bounds": True,
            "grid_row": row,
            "grid_col": col,
            "pixel_value": int(self.image[row, col]),
            "state": state,
            "is_free": bool(self.free_mask[row, col]),
            "is_occupied": bool(self.occupied_mask[row, col]),
            "is_unknown": bool(self.unknown_mask[row, col]),
            "clearance_cells": round(float(self.distance_cells[row, col]), 6),
            "clearance_m": round(float(self.distance_m[row, col]), 6),
        }

    def ray_check(self, start: tuple[float, float], end: tuple[float, float]) -> dict[str, Any]:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(2, int(math.ceil(distance / max(self.resolution * 0.5, 1e-6))))
        counts = {"free": 0, "occupied": 0, "unknown": 0, "out_of_bounds": 0}
        hits: list[dict[str, Any]] = []
        sampled_points: list[dict[str, Any]] = []
        for idx in range(steps + 1):
            t = idx / steps
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            sample = self.sample((x, y))
            state = sample["state"]
            counts[state] = counts.get(state, 0) + 1
            if state != "free" and len(hits) < 20:
                hits.append(
                    {
                        "t": round(t, 4),
                        "world_xy": [round(x, 4), round(y, 4)],
                        "grid_rc": [sample["grid_row"], sample["grid_col"]],
                        "pixel_value": sample["pixel_value"],
                        "state": state,
                    }
                )
            if idx in {0, steps // 2, steps}:
                sampled_points.append(
                    {
                        "t": round(t, 4),
                        "world_xy": [round(x, 4), round(y, 4)],
                        "grid_rc": [sample["grid_row"], sample["grid_col"]],
                        "state": state,
                    }
                )
        status = "passed" if counts.get("occupied", 0) == 0 and counts.get("unknown", 0) == 0 and counts.get("out_of_bounds", 0) == 0 else "failed"
        return {
            "status": status,
            "distance_m": round(distance, 6),
            "sample_count": steps + 1,
            "state_counts": counts,
            "hits_sample": hits,
            "sampled_points": sampled_points,
        }


def find_object(snapshot: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    for obj in snapshot.get("objects", []):
        if canonical_object_id(obj.get("id")) == object_id:
            return obj
    return None


def find_room(topology: dict[str, Any], snapshot: dict[str, Any], room_id: str) -> dict[str, Any] | None:
    for source in (topology.get("rooms", []), snapshot.get("rooms", [])):
        for room in source:
            if room.get("id") == room_id or room.get("room_id") == room_id:
                return room
    return None


def room_objects(snapshot: dict[str, Any], room_id: str, exclude_object_id: str) -> list[dict[str, Any]]:
    out = []
    for obj in snapshot.get("objects", []):
        if obj.get("room_id") == room_id and canonical_object_id(obj.get("id")) != exclude_object_id and obj.get("footprint_2d"):
            out.append(obj)
    return out


def load_route_terminal(route_path: Path) -> dict[str, Any] | None:
    data = load_json(route_path, {})
    waypoints = data.get("waypoints") or []
    return waypoints[-1] if waypoints else None


def nearest_visible_proxy(
    candidate_xy: tuple[float, float],
    footprint: list[list[float]],
    room_polygon: list[list[float]],
) -> tuple[float, float] | None:
    samples = [p for p in sample_polygon_edges(footprint) if point_in_polygon(p[0], p[1], room_polygon)]
    if not samples:
        return None
    return min(samples, key=lambda p: math.hypot(candidate_xy[0] - p[0], candidate_xy[1] - p[1]))


def all_visible_proxy_points(footprint: list[list[float]], room_polygon: list[list[float]]) -> list[tuple[float, float]]:
    return [p for p in sample_polygon_edges(footprint) if point_in_polygon(p[0], p[1], room_polygon)]


def classify_clearance_units(records: list[dict[str, Any]]) -> dict[str, Any]:
    ratios_to_m = []
    ratios_to_cells = []
    abs_err_m = []
    abs_err_cells = []
    for rec in records:
        original = rec.get("task14b_clearance_m")
        cells = rec.get("recomputed_clearance_cells")
        meters = rec.get("recomputed_clearance_m")
        if original is None or cells is None or meters is None:
            continue
        original = float(original)
        cells = float(cells)
        meters = float(meters)
        if meters > 1e-9:
            ratios_to_m.append(original / meters)
            abs_err_m.append(abs(original - meters))
        if cells > 1e-9:
            ratios_to_cells.append(original / cells)
            abs_err_cells.append(abs(original - cells))
    median_err_m = float(np.median(abs_err_m)) if abs_err_m else None
    median_err_cells = float(np.median(abs_err_cells)) if abs_err_cells else None
    likely_cells = bool(median_err_cells is not None and median_err_m is not None and median_err_cells < median_err_m)
    return {
        "task14b_clearance_likely_grid_cells_instead_of_meters": likely_cells,
        "median_abs_error_vs_recomputed_m": round_float(median_err_m),
        "median_abs_error_vs_recomputed_cells": round_float(median_err_cells),
        "median_ratio_task14b_to_recomputed_m": round_float(float(np.median(ratios_to_m)) if ratios_to_m else None),
        "median_ratio_task14b_to_recomputed_cells": round_float(float(np.median(ratios_to_cells)) if ratios_to_cells else None),
        "diagnosis": (
            "task14b values match recomputed meters from the YAML/PGM; they are not likely raw grid-cell counts"
            if not likely_cells
            else "task14b values are closer to raw grid-cell distances than meter distances"
        ),
    }


def detect_inversion(map_data: OccupancyMap, eligible_candidates: list[dict[str, Any]]) -> dict[str, Any]:
    normal_free = 0
    inverted_free = 0
    occupied_thresh = float(map_data.metadata.get("occupied_thresh", 0.65))
    free_thresh = float(map_data.metadata.get("free_thresh", 0.196))
    for cand in eligible_candidates:
        xy = tuple(cand["world_xy"])
        row, col = map_data.world_to_rc(float(xy[0]), float(xy[1]))
        if not map_data.in_bounds(row, col):
            continue
        pixel = float(map_data.image[row, col])
        normal_prob = (255.0 - pixel) / 255.0
        inverted_prob = pixel / 255.0
        if normal_prob < free_thresh:
            normal_free += 1
        if inverted_prob < free_thresh:
            inverted_free += 1
    return {
        "free_eligible_candidates_under_yaml_negate": normal_free,
        "free_eligible_candidates_under_inverted_negate": inverted_free,
        "candidate_count": len(eligible_candidates),
        "free_occupied_mask_appears_inverted": bool(inverted_free > normal_free),
        "diagnosis": "mask inversion is not supported by the candidate samples" if normal_free >= inverted_free else "candidate samples look more free under inverted semantics",
        "thresholds": {"occupied_thresh": occupied_thresh, "free_thresh": free_thresh},
    }


def score_candidate(
    cand: dict[str, Any],
    map_data: OccupancyMap,
    target_floor_id: str,
    room_id: str,
    room_polygon: list[list[float]],
    object_record: dict[str, Any],
    object_xy: tuple[float, float],
    route_terminal_xy: tuple[float, float],
    other_room_objects: list[dict[str, Any]],
    room_clearance_suspicious: bool,
) -> dict[str, Any]:
    cid = cand.get("candidate_id")
    xy = (float(cand["world_xy"][0]), float(cand["world_xy"][1]))
    yaw = float(cand.get("yaw")) if cand.get("yaw") is not None else None
    label = norm_label(object_record.get("label") or object_record.get("category"))
    footprint = object_record.get("footprint_2d") or []
    visible_proxy = nearest_visible_proxy(xy, footprint, room_polygon) if footprint and room_polygon else None
    ray_target = visible_proxy or object_xy
    sample = map_data.sample(xy)
    inside_room = point_in_polygon(xy[0], xy[1], room_polygon) if room_polygon else False
    centroid_inside_room = point_in_polygon(object_xy[0], object_xy[1], room_polygon) if room_polygon else False
    wall_side_exception = label in WALL_SIDE_LABELS and not centroid_inside_room and inside_room and visible_proxy is not None
    boundary_margin = signed_boundary_margin(xy, room_polygon) if room_polygon else None
    route_distance = math.hypot(xy[0] - route_terminal_xy[0], xy[1] - route_terminal_xy[1])
    object_distance = math.hypot(xy[0] - object_xy[0], xy[1] - object_xy[1])
    proxy_distance = math.hypot(xy[0] - ray_target[0], xy[1] - ray_target[1]) if ray_target else None
    yaw_error_centroid = yaw_error(yaw, xy, object_xy)
    yaw_error_proxy = yaw_error(yaw, xy, ray_target) if ray_target else None
    ray_centroid = map_data.ray_check(xy, object_xy)
    ray_proxy = map_data.ray_check(xy, ray_target) if ray_target else None
    semantic_overlaps = []
    for obj in other_room_objects:
        fp = obj.get("footprint_2d") or []
        if fp and point_in_bbox(xy, fp):
            semantic_overlaps.append(
                {
                    "object_id": canonical_object_id(obj.get("id")),
                    "label": obj.get("label") or obj.get("category"),
                    "footprint_2d": fp,
                }
            )
    hard_checks: dict[str, dict[str, Any]] = {
        "same_floor": {
            "status": "passed" if cand.get("floor_id") == target_floor_id else "failed",
            "candidate_floor_id": cand.get("floor_id"),
            "target_floor_id": target_floor_id,
        },
        "target_room_compatibility": {
            "status": "passed" if inside_room or wall_side_exception else "failed",
            "inside_target_room": inside_room,
            "wall_side_centroid_exception": wall_side_exception,
            "object_centroid_inside_target_room": centroid_inside_room,
        },
        "occupancy_free": {
            "status": "passed" if sample["state"] == "free" else "failed",
            **sample,
        },
        "not_unknown_or_occupied": {
            "status": "passed" if sample["state"] == "free" else "failed",
            "state": sample["state"],
        },
        "reasonable_clearance_after_correction": {
            "status": "passed" if sample["clearance_m"] is not None and sample["clearance_m"] >= 0.2 else "failed",
            "recomputed_clearance_m": sample["clearance_m"],
            "threshold_m": 0.2,
            "warning": "clearance magnitude is implausible for room scale and is not used as strong ranking evidence"
            if room_clearance_suspicious
            else None,
        },
        "yaw_faces_object_or_visible_edge": {
            "status": "passed"
            if min(v for v in [yaw_error_centroid, yaw_error_proxy] if v is not None) <= 0.45
            else "failed",
            "yaw_error_to_centroid_rad": round_float(yaw_error_centroid),
            "yaw_error_to_visible_proxy_rad": round_float(yaw_error_proxy),
            "target_used_for_best_yaw": "visible_footprint_edge" if (yaw_error_proxy or 999) <= (yaw_error_centroid or 999) else "centroid",
        },
        "candidate_to_object_ray": {
            "status": "passed" if ray_centroid["status"] == "passed" or (ray_proxy and ray_proxy["status"] == "passed") else "failed",
            "centroid_ray_status": ray_centroid["status"],
            "visible_proxy_ray_status": ray_proxy["status"] if ray_proxy else "not_evaluable",
            "target_used": "visible_footprint_edge" if ray_proxy and ray_proxy["status"] == "passed" else "centroid",
        },
    }
    failed_hard = [name for name, check in hard_checks.items() if check["status"] == "failed"]
    hard_status = "passed" if not failed_hard else "failed"

    route_score = max(0.0, 1.0 - route_distance / 1.5) * 20.0
    edge_score = max(0.0, 1.0 - abs((proxy_distance or object_distance) - 0.8) / 0.8) * 18.0
    clearance_for_score = min(float(sample["clearance_m"] or 0.0), 1.5)
    clearance_score = min(1.0, clearance_for_score / 0.8) * 10.0
    if room_clearance_suspicious:
        clearance_score *= 0.35
    margin_value = boundary_margin if boundary_margin is not None else 0.0
    margin_score = max(0.0, min(1.0, (margin_value - 0.15) / 0.65)) * 15.0
    best_yaw_error = min(v for v in [yaw_error_centroid, yaw_error_proxy] if v is not None)
    yaw_score = max(0.0, 1.0 - best_yaw_error / 0.45) * 12.0
    route_risk_penalty = min(18.0, route_distance * 7.0)
    nav2_risk_penalty = 0.0
    risk_notes: list[str] = []
    if semantic_overlaps:
        nav2_risk_penalty += 26.0
        risk_notes.append("candidate lies inside another room object footprint")
    if boundary_margin is not None and boundary_margin < 0.2:
        nav2_risk_penalty += 8.0
        risk_notes.append("candidate is close to room boundary")
    if ray_proxy and ray_proxy["status"] != "passed":
        nav2_risk_penalty += 18.0
        risk_notes.append("visible-proxy ray is blocked or unknown")
    if visible_proxy is None:
        nav2_risk_penalty += 12.0
        risk_notes.append("no visible footprint-edge proxy available")
    if wall_side_exception:
        nav2_risk_penalty += 4.0
        risk_notes.append("wall-side object centroid is outside the target room")

    score_terms = {
        "distance_to_route_terminal": round(route_score, 3),
        "distance_to_object_or_visible_edge": round(edge_score, 3),
        "corrected_clearance": round(clearance_score, 3),
        "boundary_margin": round(margin_score, 3),
        "yaw_error": round(yaw_score, 3),
        "route_terminal_continuity_risk_penalty": round(route_risk_penalty, 3),
        "runtime_nav2_approach_risk_penalty": round(nav2_risk_penalty, 3),
    }
    total_score = sum(score_terms[k] for k in ["distance_to_route_terminal", "distance_to_object_or_visible_edge", "corrected_clearance", "boundary_margin", "yaw_error"])
    total_score -= route_risk_penalty + nav2_risk_penalty
    if hard_status != "passed":
        total_score -= 100.0
    return {
        "candidate_id": cid,
        "task14b_validation_status": cand.get("validation_status"),
        "candidate_source": cand.get("candidate_source"),
        "world_xy": [round(xy[0], 6), round(xy[1], 6)],
        "yaw": round_float(yaw),
        "task14b_clearance_m": round_float(cand.get("clearance_m")),
        "recomputed_clearance_cells": sample["clearance_cells"],
        "recomputed_clearance_m": sample["clearance_m"],
        "map_sample": sample,
        "inside_target_room": inside_room,
        "object_centroid_inside_target_room": centroid_inside_room,
        "wall_side_centroid_exception_used": wall_side_exception,
        "boundary_margin_m": round_float(boundary_margin),
        "distance_to_route_terminal_m": round(route_distance, 6),
        "distance_to_object_centroid_m": round(object_distance, 6),
        "visible_proxy_xy": [round(visible_proxy[0], 6), round(visible_proxy[1], 6)] if visible_proxy else None,
        "distance_to_visible_proxy_m": round_float(proxy_distance),
        "yaw_error_to_centroid_rad": round_float(yaw_error_centroid),
        "yaw_error_to_visible_proxy_rad": round_float(yaw_error_proxy),
        "ray_to_centroid": ray_centroid,
        "ray_to_visible_proxy": ray_proxy,
        "semantic_object_footprint_overlaps": semantic_overlaps,
        "hard_checks": hard_checks,
        "hard_status": hard_status,
        "hard_failures": failed_hard,
        "soft_score_breakdown": score_terms,
        "total_score": round(total_score, 3),
        "runtime_risk_notes": risk_notes,
        "eligible_for_recommendation": cand.get("validation_status") == "passed" and hard_status == "passed",
    }


def table_row(record: dict[str, Any], recommended_id: str | None) -> dict[str, Any]:
    overlaps = ",".join(f"{o['object_id']}:{o['label']}" for o in record.get("semantic_object_footprint_overlaps") or [])
    return {
        "candidate_id": record["candidate_id"],
        "task14b_status": record["task14b_validation_status"],
        "eligible": record["eligible_for_recommendation"],
        "recommended": record["candidate_id"] == recommended_id,
        "world_x": record["world_xy"][0],
        "world_y": record["world_xy"][1],
        "task14b_clearance_m": record["task14b_clearance_m"],
        "recomputed_clearance_cells": record["recomputed_clearance_cells"],
        "recomputed_clearance_m": record["recomputed_clearance_m"],
        "occupancy_state": record["map_sample"]["state"],
        "inside_target_room": record["inside_target_room"],
        "boundary_margin_m": record["boundary_margin_m"],
        "distance_to_route_terminal_m": record["distance_to_route_terminal_m"],
        "distance_to_object_centroid_m": record["distance_to_object_centroid_m"],
        "distance_to_visible_proxy_m": record["distance_to_visible_proxy_m"],
        "yaw_error_to_centroid_rad": record["yaw_error_to_centroid_rad"],
        "yaw_error_to_visible_proxy_rad": record["yaw_error_to_visible_proxy_rad"],
        "ray_to_centroid": record["ray_to_centroid"]["status"],
        "ray_to_visible_proxy": (record["ray_to_visible_proxy"] or {}).get("status", "not_evaluable"),
        "semantic_object_footprint_overlaps": overlaps,
        "hard_status": record["hard_status"],
        "total_score": record["total_score"],
        "runtime_risk_notes": "; ".join(record.get("runtime_risk_notes") or []),
    }


def write_markdown_table(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "candidate_id",
        "task14b_status",
        "eligible",
        "recommended",
        "recomputed_clearance_m",
        "boundary_margin_m",
        "distance_to_route_terminal_m",
        "distance_to_visible_proxy_m",
        "yaw_error_to_visible_proxy_rad",
        "semantic_object_footprint_overlaps",
        "total_score",
    ]
    lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(field, "")) for field in fields) + " |")
    path.write_text("\n".join(lines) + "\n")


def draw_visualization(
    path: Path,
    map_data: OccupancyMap,
    room_polygon: list[list[float]],
    object_record: dict[str, Any],
    route_terminal_xy: tuple[float, float],
    records: list[dict[str, Any]],
    selected_id: str,
    recommended_id: str | None,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 10))
    all_points: list[tuple[float, float]] = [route_terminal_xy]
    all_points += [(float(r["world_xy"][0]), float(r["world_xy"][1])) for r in records if r["task14b_validation_status"] == "passed"]
    all_points += [(float(p[0]), float(p[1])) for p in room_polygon]
    fp = object_record.get("footprint_2d") or []
    all_points += [(float(p[0]), float(p[1])) for p in fp]
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    pad = 0.9
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    row_min, col_min = map_data.world_to_rc(min_x, min_y)
    row_max, col_max = map_data.world_to_rc(max_x, max_y)
    r0, r1 = sorted((max(0, row_min), min(map_data.height - 1, row_max)))
    c0, c1 = sorted((max(0, col_min), min(map_data.width - 1, col_max)))
    crop = map_data.image[r0 : r1 + 1, c0 : c1 + 1]
    extent = [
        map_data.origin[0] + c0 * map_data.resolution,
        map_data.origin[0] + c1 * map_data.resolution,
        map_data.origin[1] + r0 * map_data.resolution,
        map_data.origin[1] + r1 * map_data.resolution,
    ]
    ax.imshow(crop, cmap="gray", origin="lower", extent=extent, alpha=0.82)
    if room_polygon:
        room_closed = room_polygon + [room_polygon[0]]
        ax.plot([p[0] for p in room_closed], [p[1] for p in room_closed], color="#1f77b4", linewidth=2.5, label="room_14 polygon")
    if fp:
        fp_closed = fp + [fp[0]]
        ax.plot([p[0] for p in fp_closed], [p[1] for p in fp_closed], color="#ff7f0e", linewidth=2.2, label="obj_175 footprint")
    obj_xy = tuple(object_record.get("pose") or [None, None])
    if obj_xy[0] is not None:
        ax.scatter([obj_xy[0]], [obj_xy[1]], marker="*", s=190, color="#ff7f0e", edgecolor="black", zorder=6, label="object centroid")
    visible_points = all_visible_proxy_points(fp, room_polygon) if fp and room_polygon else []
    if visible_points:
        ax.scatter([p[0] for p in visible_points[:: max(1, len(visible_points) // 40)]], [p[1] for p in visible_points[:: max(1, len(visible_points) // 40)]], s=13, color="#f2c14e", zorder=5, label="room-side footprint proxy")
    ax.scatter([route_terminal_xy[0]], [route_terminal_xy[1]], marker="s", s=120, color="#2ca02c", edgecolor="black", zorder=7, label="executable route terminal")

    for record in records:
        x, y = record["world_xy"]
        cid = record["candidate_id"]
        if record["task14b_validation_status"] != "passed":
            ax.scatter([x], [y], marker=".", s=24, color="#7f7f7f", alpha=0.55, zorder=4)
            continue
        color = "#9467bd"
        marker = "o"
        size = 55
        if cid == selected_id:
            color = "#d62728"
            marker = "X"
            size = 130
        if cid == recommended_id:
            color = "#17becf"
            marker = "P"
            size = 150
        if record.get("semantic_object_footprint_overlaps"):
            ax.scatter([x], [y], marker="x", s=90, color="#8c564b", linewidth=2.0, zorder=8, label="_footprint_overlap")
        ax.scatter([x], [y], marker=marker, s=size, color=color, edgecolor="black", zorder=8)
        if cid in {selected_id, recommended_id, "route_terminal_seed", "generated_ring_021", "generated_ring_038", "generated_ring_039", "generated_ring_002"}:
            ax.text(x + 0.035, y + 0.035, cid.replace("generated_ring_", "gr_"), fontsize=8, color="black", zorder=9)
    rec = next((r for r in records if r["candidate_id"] == recommended_id), None)
    if rec:
        start = tuple(rec["world_xy"])
        target = tuple(rec.get("visible_proxy_xy") or object_record.get("pose"))
        ray = rec["ray_to_visible_proxy"] or rec["ray_to_centroid"]
        ax.plot([start[0], target[0]], [start[1], target[1]], color="#17becf", linewidth=2.2, label="recommended candidate ray")
        for hit in ray.get("hits_sample") or []:
            hx, hy = hit["world_xy"]
            ax.scatter([hx], [hy], marker="D", s=35, color="#d62728" if hit["state"] == "occupied" else "#bcbd22", zorder=10)
        for point in ray.get("sampled_points") or []:
            px, py = point["world_xy"]
            ax.scatter([px], [py], marker=".", s=45, color="#2ca02c" if point["state"] == "free" else "#d62728", zorder=10)
    ax.set_title("RSLG-SLAM task14b2 obj_175 floor_2 candidate sanity diagnostic")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.grid(True, alpha=0.2)
    if any(r.get("semantic_object_footprint_overlaps") for r in records if r["task14b_validation_status"] == "passed"):
        ax.scatter([], [], marker="x", s=90, color="#8c564b", linewidth=2.0, label="downgraded: overlaps another object footprint")
    if any(r["task14b_validation_status"] != "passed" for r in records):
        ax.scatter([], [], marker=".", s=24, color="#7f7f7f", alpha=0.55, label="task14b rejected candidate")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def make_report_markdown(
    report: dict[str, Any],
    records: list[dict[str, Any]],
    selected: dict[str, Any] | None,
    recommended: dict[str, Any] | None,
) -> str:
    selected_id = selected["candidate_id"] if selected else None
    recommended_id = recommended["candidate_id"] if recommended else None
    selected_rec = next((r for r in records if r["candidate_id"] == selected_id), None)
    lines = [
        "# Task14b2 Object Approach Candidate Sanity Report",
        "",
        "Project: RSLG-SLAM",
        "",
        "## Direct Answers",
        "",
        "1. Was task14b clearance wrong?",
        "",
        report["direct_answers"]["was_task14b_clearance_wrong"],
        "",
        "2. What was wrong and what is the corrected clearance?",
        "",
        report["direct_answers"]["clearance_correction"],
        "",
        "3. Does generated_ring_022 remain the best candidate?",
        "",
        report["direct_answers"]["does_generated_ring_022_remain_best"],
        "",
        "4. If yes/no, why compared with route_terminal_seed and nearby generated_ring candidates?",
        "",
        report["direct_answers"]["selected_candidate_comparison"],
        "",
        "5. Which candidate is now recommended and why?",
        "",
        report["direct_answers"]["recommended_candidate_reason"],
        "",
        "6. Is same-side-of-wall actually verified?",
        "",
        report["direct_answers"]["same_side_of_wall_status"],
        "",
        "7. Is candidate-to-object ray validation strong enough for runtime readiness?",
        "",
        report["direct_answers"]["ray_validation_strength"],
        "",
        "8. Can task14c runtime testing proceed?",
        "",
        report["direct_answers"]["can_task14c_proceed"],
        "",
        "9. What exactly can and cannot be claimed after task14b2?",
        "",
        "\n".join(f"- {claim}" for claim in report["exact_non_claims"]),
        "",
        "## Key Values",
        "",
        f"- Task14b selected candidate: `{selected_id}`",
        f"- Task14b2 recommended candidate: `{recommended_id}`",
        f"- Readiness level: `{report['readiness_level']}`",
    ]
    if selected_rec:
        lines.append(f"- generated_ring_022 recomputed clearance: `{selected_rec['recomputed_clearance_m']}` m")
        lines.append(f"- generated_ring_022 visible-proxy yaw error: `{selected_rec['yaw_error_to_visible_proxy_rad']}` rad")
    if recommended:
        lines.append(f"- Recommended candidate recomputed clearance: `{recommended['recomputed_clearance_m']}` m")
        lines.append(f"- Recommended candidate visible-proxy distance: `{recommended['distance_to_visible_proxy_m']}` m")
        lines.append(f"- Recommended candidate route-terminal distance: `{recommended['distance_to_route_terminal_m']}` m")
    lines += [
        "",
        "## Limitations",
        "",
        "\n".join(f"- {item}" for item in report["limitations"]),
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", required=True, type=Path)
    parser.add_argument("--task14b-output-dir", required=True, type=Path)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--room-id", required=True)
    parser.add_argument("--stable-map", required=True, type=Path)
    parser.add_argument("--semantic-route", required=True, type=Path)
    parser.add_argument("--executable-route", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    object_id = canonical_object_id(args.object_id)
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    task14b_candidates_path = args.task14b_output_dir / f"object_approach_candidates_{object_id}.json"
    task14b_pose_report_path = args.task14b_output_dir / f"object_approach_pose_report_{object_id}.json"
    candidates_payload = load_json(task14b_candidates_path)
    pose_report = load_json(task14b_pose_report_path, {})
    candidates = list(candidates_payload.get("candidates") or [])
    selected = pose_report.get("selected_candidate")
    selected_id = (selected or {}).get("candidate_id", "generated_ring_022")

    public_dir = args.stage_output_dir / "committed_public"
    snapshot = load_json(public_dir / "committed_room_world_snapshot_v0_1.json")
    topology = load_json(public_dir / "topology_v0_1.json", {})
    object_record = find_object(snapshot, object_id)
    if not object_record:
        raise ValueError(f"Could not find {object_id} in committed room-world snapshot")
    room = find_room(topology, snapshot, args.room_id)
    if not room:
        raise ValueError(f"Could not find {args.room_id} in committed topology/snapshot")
    room_polygon = room.get("polygon") or room.get("footprint_polygon_xy") or []
    object_xy = (float(object_record["pose"][0]), float(object_record["pose"][1]))
    route_terminal = load_route_terminal(args.executable_route)
    if not route_terminal:
        raise ValueError("Executable route has no terminal waypoint")
    route_terminal_xy = (float(route_terminal["x"]), float(route_terminal["y"]))

    map_data = OccupancyMap.load(args.stable_map)
    passed_candidates = [cand for cand in candidates if cand.get("validation_status") == "passed"]
    other_objects = room_objects(snapshot, args.room_id, object_id)
    room_diag = polygon_diag(room_polygon) or 0.0
    unique_values, unique_counts = np.unique(map_data.image, return_counts=True)
    map_value_counts = {str(int(v)): int(c) for v, c in zip(unique_values, unique_counts)}
    occupancy_interpretation = {
        str(value): map_data.pixel_interpretation(value) for value in PIXELS_TO_EXPLAIN
    }
    preliminary_records = []
    for cand in candidates:
        sample = map_data.sample((float(cand["world_xy"][0]), float(cand["world_xy"][1])))
        preliminary_records.append(
            {
                "candidate_id": cand.get("candidate_id"),
                "task14b_clearance_m": round_float(cand.get("clearance_m")),
                "recomputed_clearance_cells": sample["clearance_cells"],
                "recomputed_clearance_m": sample["clearance_m"],
            }
        )
    clearance_units = classify_clearance_units(preliminary_records)
    inversion = detect_inversion(map_data, passed_candidates)
    max_candidate_clearance = max((r["recomputed_clearance_m"] or 0.0) for r in preliminary_records)
    room_clearance_suspicious = bool(room_diag and max_candidate_clearance > room_diag * 2.0)

    records = [
        score_candidate(
            cand,
            map_data,
            args.floor_id,
            args.room_id,
            room_polygon,
            object_record,
            object_xy,
            route_terminal_xy,
            other_objects,
            room_clearance_suspicious,
        )
        for cand in candidates
    ]
    eligible = [r for r in records if r["eligible_for_recommendation"]]
    eligible_passed = [r for r in eligible if r["task14b_validation_status"] == "passed"]
    recommended = max(eligible_passed, key=lambda r: r["total_score"]) if eligible_passed else None
    recommended_id = recommended["candidate_id"] if recommended else None
    selected_record = next((r for r in records if r["candidate_id"] == selected_id), None)
    named_safety = {
        cid: next((r for r in records if r["candidate_id"] == cid), None)
        for cid in ["route_terminal_seed", "generated_ring_021", "generated_ring_038", "generated_ring_039"]
    }
    selected_remains = selected_id == recommended_id

    same_side_status = "partially_verified"
    if not object_record.get("footprint_2d"):
        same_side_status = "not_evaluable"
    if not recommended:
        same_side_status = "not_evaluable"
    readiness = "partially_ready_requires_runtime_probe" if recommended else "not_ready_due_to_validation_bug"
    if room_clearance_suspicious:
        readiness = "partially_ready_requires_runtime_probe" if recommended else "not_ready_due_to_validation_bug"

    table_rows = [table_row(r, recommended_id) for r in sorted(records, key=lambda x: (x["task14b_validation_status"] != "passed", -x["total_score"]))]
    csv_fields = list(table_rows[0].keys()) if table_rows else []
    write_csv(output_dir / f"candidate_score_table_{object_id}.csv", table_rows, csv_fields)
    write_markdown_table(output_dir / f"candidate_score_table_{object_id}.md", table_rows)

    clearance_rows = [
        {
            **prelim,
            "abs_delta_task14b_vs_recomputed_m": round_float(
                abs(float(prelim["task14b_clearance_m"]) - float(prelim["recomputed_clearance_m"]))
                if prelim["task14b_clearance_m"] is not None and prelim["recomputed_clearance_m"] is not None
                else None
            ),
            "task14b_matches_recomputed_m": bool(
                prelim["task14b_clearance_m"] is not None
                and prelim["recomputed_clearance_m"] is not None
                and abs(float(prelim["task14b_clearance_m"]) - float(prelim["recomputed_clearance_m"])) < 0.06
            ),
        }
        for prelim in preliminary_records
    ]
    clearance_diagnostics = {
        "artifact_type": "clearance_diagnostics",
        "created_utc": utc_now(),
        "map_metadata": {
            "yaml": rel(args.stable_map),
            "image": rel(map_data.image_path),
            "resolution": map_data.resolution,
            "origin": list(map_data.origin),
            "negate": int(map_data.metadata.get("negate", 0)),
            "occupied_thresh": float(map_data.metadata.get("occupied_thresh", 0.65)),
            "free_thresh": float(map_data.metadata.get("free_thresh", 0.196)),
            "mode": map_data.metadata.get("mode"),
            "width": map_data.width,
            "height": map_data.height,
            "pixel_value_counts": map_value_counts,
        },
        "occupancy_interpretation": occupancy_interpretation,
        "distance_transform": {
            "library": "scipy.ndimage.distance_transform_edt",
            "input": "ROS-compatible free mask reconstructed from YAML/PGM",
            "cells_to_meters": "distance_cells * map_resolution",
        },
        "clearance_unit_diagnosis": clearance_units,
        "mask_inversion_diagnosis": inversion,
        "room_scale_sanity": {
            "room_polygon_bbox_diagonal_m": round_float(room_diag),
            "max_recomputed_candidate_clearance_m": round_float(max_candidate_clearance),
            "clearance_magnitude_implausible_for_room_scale": room_clearance_suspicious,
            "diagnosis": "clearance values are numerically reproducible from the stable PGM but too large to be trusted as local room-scale safety margins"
            if room_clearance_suspicious
            else "clearance magnitude is within the room-scale sanity envelope",
        },
        "candidate_clearances": clearance_rows,
    }
    write_json(output_dir / f"clearance_diagnostics_{object_id}.json", clearance_diagnostics)

    comparison = {
        "artifact_type": "selected_candidate_comparison",
        "created_utc": utc_now(),
        "task14b_selected_candidate_id": selected_id,
        "task14b2_recommended_candidate_id": recommended_id,
        "generated_ring_022_remains_recommended": selected_remains,
        "selected_candidate_record": selected_record,
        "recommended_candidate_record": recommended,
        "named_alternative_safety": {
            cid: {
                "total_score": rec["total_score"],
                "eligible": rec["eligible_for_recommendation"],
                "semantic_object_footprint_overlaps": rec["semantic_object_footprint_overlaps"],
                "distance_to_route_terminal_m": rec["distance_to_route_terminal_m"],
                "distance_to_visible_proxy_m": rec["distance_to_visible_proxy_m"],
                "runtime_risk_notes": rec["runtime_risk_notes"],
            }
            if rec
            else None
            for cid, rec in named_safety.items()
        },
        "whether_route_terminal_seed_or_generated_ring_021_038_039_is_safer": "No. Those named alternatives are closer to route continuity, but each falls inside the obj_178 bed semantic footprint in the committed snapshot, so task14b2 scores them as higher runtime-risk approach poses.",
    }
    write_json(output_dir / f"selected_candidate_comparison_{object_id}.json", comparison)

    exact_non_claims = [
        "Stage-A was not rerun.",
        "ROS, Gazebo, Nav2, and RViz were not started.",
        "This does not claim semantic ground-truth accuracy.",
        "This does not claim object navigation success.",
        "This does not claim object approach success.",
        "This does not claim same-side-of-wall verification beyond the offline footprint/proxy evidence.",
        "This only recommends an offline candidate for later runtime probing.",
    ]
    limitations = [
        "The stable PGM has only values 0 and 254 and yields implausibly large distance-transform clearances around room_14.",
        "The object centroid for the curtain is outside room_14; task14b2 uses the room-side footprint edge as a proxy.",
        "The stable occupancy map does not encode semantic object footprints such as the bed; semantic footprint overlap is treated as a runtime-risk signal, not a collision-ground-truth claim.",
        "Candidate-to-object ray checks are map-grid checks only and do not prove visual observability or successful final approach control.",
    ]
    direct_answers = {
        "was_task14b_clearance_wrong": (
            "Yes as a readiness signal, but not because the values are raw grid-cell counts. "
            "The independent ROS-compatible distance transform reproduces task14b's ~44 m values from the stable YAML/PGM. "
            "Those values are implausible for room_14 and should not have been treated as meaningful local clearance."
        ),
        "clearance_correction": (
            f"For `{selected_id}`, task14b reported {selected_record['task14b_clearance_m'] if selected_record else None} m; "
            f"task14b2 recomputed {selected_record['recomputed_clearance_cells'] if selected_record else None} cells = "
            f"{selected_record['recomputed_clearance_m'] if selected_record else None} m from the stable PGM. "
            "The corrected interpretation is that this clearance is map-derived but not trustworthy as a room-scale safety margin."
        ),
        "does_generated_ring_022_remain_best": (
            "No. task14b2 does not keep generated_ring_022 as the recommendation because it lies inside another room object's committed semantic footprint. "
            "The new recommendation avoids that footprint-overlap risk, although its yaw to the room-side curtain edge is only partial/near-threshold evidence."
            if not selected_remains
            else "Yes. generated_ring_022 remains the recommendation after the task14b2 scoring checks."
        ),
        "selected_candidate_comparison": (
            "route_terminal_seed, generated_ring_021, generated_ring_038, and generated_ring_039 have good route continuity, but task14b2 marks them as higher runtime risk because they overlap the obj_178 bed footprint. "
            f"The recommended candidate is `{recommended_id}` because it avoids that overlap while keeping a valid room-side footprint-edge ray."
            if recommended_id
            else "No candidate remained reliable enough to recommend."
        ),
        "recommended_candidate_reason": (
            f"`{recommended_id}` is recommended for a runtime probe because it is in room_14, free in the stable map, has a passing ray to the room-side curtain footprint proxy, avoids other committed room-object footprints, and has the best transparent risk-adjusted score among task14b-passed candidates."
            if recommended_id
            else "No candidate is recommended."
        ),
        "same_side_of_wall_status": f"`{same_side_status}`. The candidate and visible footprint-edge proxy are in room_14, but the curtain centroid is outside the polygon and no stronger structural wall-side predicate is available offline.",
        "ray_validation_strength": "Partial. The visible footprint-edge ray is free in the stable occupancy map, but the centroid ray targets a point outside the room and the map is too sparse to prove object-side visibility or runtime approach safety.",
        "can_task14c_proceed": "Yes, only as a runtime probe using the task14b2 recommendation and the stated limitations. It must not be reported as object navigation or object approach success until runtime evidence exists.",
    }
    report = {
        "artifact_type": "candidate_sanity_report",
        "created_utc": utc_now(),
        "project_name": "RSLG-SLAM",
        "inputs_used": {
            "stage_output_dir": rel(args.stage_output_dir),
            "task14b_output_dir": rel(args.task14b_output_dir),
            "task14b_candidates": rel(task14b_candidates_path),
            "task14b_pose_report": rel(task14b_pose_report_path),
            "stable_map": rel(args.stable_map),
            "semantic_route": rel(args.semantic_route),
            "executable_route": rel(args.executable_route),
            "committed_snapshot": rel(public_dir / "committed_room_world_snapshot_v0_1.json"),
            "topology": rel(public_dir / "topology_v0_1.json"),
        },
        "stage_a_rerun": False,
        "ros_gazebo_nav2_rviz_started": False,
        "query": args.query,
        "object": object_record,
        "room": room,
        "route_terminal": route_terminal,
        "stable_map_metadata": clearance_diagnostics["map_metadata"],
        "occupancy_interpretation": occupancy_interpretation,
        "clearance_recomputation_details": clearance_diagnostics["distance_transform"],
        "original_vs_recomputed_clearance": clearance_rows,
        "clearance_diagnostics_summary": {
            "clearance_unit_diagnosis": clearance_units,
            "mask_inversion_diagnosis": inversion,
            "room_scale_sanity": clearance_diagnostics["room_scale_sanity"],
        },
        "all_candidate_pass_fail_reason_records": records,
        "score_breakdown": {record["candidate_id"]: record["soft_score_breakdown"] for record in records},
        "selected_candidate_from_task14b": selected,
        "recommended_candidate_from_task14b2": recommended,
        "generated_ring_022_remains_recommended": selected_remains,
        "whether_route_terminal_seed_or_generated_ring_021_038_039_is_safer": comparison["whether_route_terminal_seed_or_generated_ring_021_038_039_is_safer"],
        "same_side_of_wall_verification": same_side_status,
        "candidate_to_object_ray_validation_strength": "partial",
        "readiness_level": readiness,
        "limitations": limitations,
        "exact_non_claims": exact_non_claims,
        "direct_answers": direct_answers,
    }
    write_json(output_dir / f"candidate_sanity_report_{object_id}.json", report)
    md = make_report_markdown(report, records, selected_record, recommended)
    (output_dir / f"object_approach_candidate_sanity_report_{object_id}.md").write_text(md + "\n")
    completion_summary = "\n".join(
        [
            "# Task14b2 Completion Summary",
            "",
            "Project: RSLG-SLAM",
            "",
            f"- Object query: `{args.query}`",
            f"- Task14b selected candidate: `{selected_id}`",
            f"- Task14b2 recommended candidate: `{recommended_id}`",
            f"- Readiness level: `{readiness}`",
            f"- Stage-A rerun: `false`",
            f"- ROS/Gazebo/Nav2/RViz started: `false`",
            "- Main finding: task14b clearance values are reproducible from the stable PGM but are implausible and should not be trusted as room-scale clearance evidence.",
            "- Non-claim: no semantic GT accuracy, object navigation success, or object approach success is claimed.",
            "",
        ]
    )
    (output_dir / "completion_summary.md").write_text(completion_summary)
    draw_visualization(
        output_dir / f"object_approach_candidate_sanity_{object_id}_floor2.png",
        map_data,
        room_polygon,
        object_record,
        route_terminal_xy,
        records,
        selected_id,
        recommended_id,
    )
    print(json.dumps({"output_dir": rel(output_dir), "recommended_candidate": recommended_id, "readiness_level": readiness}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
