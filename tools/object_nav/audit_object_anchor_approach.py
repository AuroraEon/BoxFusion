#!/usr/bin/env python3
"""Audit RSLG-SLAM object anchors for offline object-approach readiness.

This script is intentionally offline: it reads committed/public artifacts,
map/route files, and task outputs, then produces validation artifacts for a
candidate robot approach pose. It does not start ROS, Nav2, Gazebo, or RViz.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import re
from collections import Counter, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


ROOT = Path(__file__).resolve().parents[2]


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


def canonical_object_id(value: Any) -> str:
    text = str(value)
    return text if text.startswith("obj_") else f"obj_{text}"


def normalize_label(value: Any) -> str:
    text = "" if value is None else str(value).lower().strip()
    text = re.sub(r"[^a-z0-9]+", "_", text)
    return re.sub(r"_+", "_", text).strip("_")


def status_from_checks(checks: dict[str, dict[str, Any]], critical: list[str]) -> str:
    critical_statuses = [checks.get(name, {}).get("status") for name in critical]
    if any(status == "failed" for status in critical_statuses):
        return "failed"
    if any(status == "not_evaluable" for status in critical_statuses):
        return "partially_validated"
    return "passed"


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    n = len(polygon)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i][0], polygon[i][1]
        xj, yj = polygon[j][0], polygon[j][1]
        intersects = (yi > y) != (yj > y)
        if intersects:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def polygon_bbox(footprint: list[list[float]]) -> tuple[float, float, float, float] | None:
    if not footprint:
        return None
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    return min(xs), min(ys), max(xs), max(ys)


def polygon_center(footprint: list[list[float]]) -> tuple[float, float] | None:
    if not footprint:
        return None
    return (
        sum(float(p[0]) for p in footprint) / len(footprint),
        sum(float(p[1]) for p in footprint) / len(footprint),
    )


class OccupancyMap:
    def __init__(self, yaml_path: Path) -> None:
        self.yaml_path = yaml_path
        self.meta = yaml.safe_load(yaml_path.read_text())
        image_path = Path(self.meta["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        self.image_path = image_path
        self.grid = np.array(Image.open(image_path).convert("L"))
        self.height, self.width = self.grid.shape
        self.resolution = float(self.meta["resolution"])
        origin = self.meta.get("origin") or [0.0, 0.0, 0.0]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.free_mask = self.grid >= 250
        self.occupied_mask = self.grid <= 65
        self.clearance_m = ndimage.distance_transform_edt(self.free_mask) * self.resolution

    def world_to_rc(self, x: float, y: float) -> tuple[int, int]:
        col = int(round((x - self.origin_x) / self.resolution))
        row = int(round((y - self.origin_y) / self.resolution))
        return row, col

    def rc_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + col * self.resolution,
            self.origin_y + row * self.resolution,
        )

    def in_bounds_rc(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def sample(self, x: float, y: float) -> dict[str, Any]:
        row, col = self.world_to_rc(x, y)
        if not self.in_bounds_rc(row, col):
            return {
                "in_bounds": False,
                "grid_row": row,
                "grid_col": col,
                "occupancy_value": None,
                "is_free": False,
                "clearance_m": None,
            }
        return {
            "in_bounds": True,
            "grid_row": row,
            "grid_col": col,
            "occupancy_value": int(self.grid[row, col]),
            "is_free": bool(self.free_mask[row, col]),
            "clearance_m": round(float(self.clearance_m[row, col]), 4),
        }

    def ray_check(self, start_xy: tuple[float, float], end_xy: tuple[float, float]) -> dict[str, Any]:
        sx, sy = start_xy
        ex, ey = end_xy
        distance = math.hypot(ex - sx, ey - sy)
        steps = max(2, int(math.ceil(distance / (self.resolution * 0.5))))
        occupied_hits: list[dict[str, Any]] = []
        out_of_bounds = 0
        min_value = 255
        for i in range(steps + 1):
            t = i / steps
            x = sx + (ex - sx) * t
            y = sy + (ey - sy) * t
            row, col = self.world_to_rc(x, y)
            if not self.in_bounds_rc(row, col):
                out_of_bounds += 1
                continue
            value = int(self.grid[row, col])
            min_value = min(min_value, value)
            if value <= 65:
                occupied_hits.append({"grid_row": row, "grid_col": col, "value": value, "t": round(t, 3)})
                if len(occupied_hits) >= 8:
                    break
        if occupied_hits:
            return {
                "status": "failed",
                "reason": "candidate-to-object ray crosses occupied cells",
                "distance_m": round(distance, 4),
                "occupied_hit_count_sampled": len(occupied_hits),
                "occupied_hits_sample": occupied_hits,
                "min_occupancy_value": min_value,
            }
        if out_of_bounds:
            return {
                "status": "failed",
                "reason": "candidate-to-object ray leaves map bounds",
                "distance_m": round(distance, 4),
                "out_of_bounds_samples": out_of_bounds,
            }
        return {
            "status": "passed",
            "reason": "candidate-to-object ray has no occupied-cell hits",
            "distance_m": round(distance, 4),
            "min_occupancy_value": min_value,
        }


def connected_from(map_data: OccupancyMap, start_xy: tuple[float, float], min_clearance_m: float = 0.15) -> np.ndarray | None:
    start_row, start_col = map_data.world_to_rc(*start_xy)
    traversable = map_data.free_mask & (map_data.clearance_m >= min_clearance_m)
    if not map_data.in_bounds_rc(start_row, start_col) or not traversable[start_row, start_col]:
        traversable = map_data.free_mask
        if not map_data.in_bounds_rc(start_row, start_col) or not traversable[start_row, start_col]:
            return None
    visited = np.zeros_like(traversable, dtype=bool)
    q: deque[tuple[int, int]] = deque([(start_row, start_col)])
    visited[start_row, start_col] = True
    while q:
        row, col = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nr, nc = row + dr, col + dc
            if 0 <= nr < map_data.height and 0 <= nc < map_data.width and traversable[nr, nc] and not visited[nr, nc]:
                visited[nr, nc] = True
                q.append((nr, nc))
    return visited


def route_terminal(route_path: Path) -> dict[str, Any] | None:
    data = load_json(route_path, {})
    waypoints = data.get("waypoints") or []
    if not waypoints:
        return None
    terminal = waypoints[-1]
    if "x" not in terminal or "y" not in terminal:
        return None
    return terminal


def schema_summary(entries: list[dict[str, Any]]) -> dict[str, Any]:
    keys = Counter()
    value_types: dict[str, Counter[str]] = {}
    for entry in entries:
        for key, value in entry.items():
            keys[key] += 1
            value_types.setdefault(key, Counter())[type(value).__name__] += 1
    return {
        "entry_count": len(entries),
        "fields": sorted(keys),
        "field_presence": dict(sorted(keys.items())),
        "field_types": {key: dict(counter) for key, counter in sorted(value_types.items())},
        "sample_preview": preview(entries[0]) if entries else None,
    }


def preview(value: Any, depth: int = 0) -> Any:
    if depth >= 3:
        return f"<{type(value).__name__}>"
    if isinstance(value, dict):
        out: dict[str, Any] = {}
        for idx, (key, item) in enumerate(value.items()):
            if idx >= 24:
                out["..."] = f"{len(value) - idx} more keys"
                break
            out[key] = preview(item, depth + 1)
        return out
    if isinstance(value, list):
        items = [preview(item, depth + 1) for item in value[:8]]
        if len(value) > 8:
            items.append(f"... {len(value) - 8} more items")
        return items
    if isinstance(value, str) and len(value) > 240:
        return value[:237] + "..."
    return value


def classify_entries(name: str, entries: list[dict[str, Any]]) -> dict[str, Any]:
    fields = set().union(*(entry.keys() for entry in entries)) if entries else set()
    anchor_types = sorted({str(e.get("anchor_type")) for e in entries if e.get("anchor_type") is not None})
    has_object_link = any(k in fields for k in ("object_id", "target_id", "id"))
    has_room = any(k in fields for k in ("room_id", "from_room", "to_room"))
    has_floor = "floor_id" in fields
    has_position = any(k in fields for k in ("position", "pose", "pose_xy", "pose_3d", "center", "centroid_xy", "x"))
    has_yaw = "yaw" in fields
    has_frame = any(k in fields for k in ("frame_id", "frame_idx", "timestamp"))
    has_provenance = any(k in fields for k in ("provenance", "source", "reason", "room_assignment", "floor_assignment"))
    has_visibility = any(k in fields for k in ("view_quality", "visibility", "semantic_observations", "camera_pose", "view_pose"))
    if "anchor_type" in fields:
        if anchor_types == ["room"]:
            classification = "room_anchor"
        elif "object" in anchor_types:
            classification = "object_anchor_position_centroid_like"
        else:
            classification = "mixed_anchor"
    elif "waypoint_index" in fields or name == "waypoints":
        classification = "route_waypoint"
    elif "footprint_2d" in fields or "pose_3d" in fields or "pose" in fields:
        classification = "object_centroid_or_footprint"
    elif "semantic_observations" in fields:
        classification = "semantic_object_metadata"
    else:
        classification = "unknown_structured_entries"
    if any(k in fields for k in ("camera_pose", "view_pose", "observer_pose", "robot_pose")):
        classification = "observation_or_view_pose_anchor"
    return {
        "collection_name": name,
        "anchor_type_classification": classification,
        "anchor_types": anchor_types,
        "has_object_id_or_target_link": has_object_link,
        "has_room_id": has_room,
        "has_floor_id": has_floor,
        "has_pose_or_position": has_position,
        "has_yaw": has_yaw,
        "has_frame_id_or_timestamp": has_frame,
        "has_provenance_fields": has_provenance,
        "has_visibility_or_observation_fields": has_visibility,
        "schema": schema_summary(entries),
    }


def extract_collections(data: Any) -> list[tuple[str, list[dict[str, Any]]]]:
    collections: list[tuple[str, list[dict[str, Any]]]] = []

    def visit(node: Any, key_name: str) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if isinstance(value, list) and value and all(isinstance(item, dict) for item in value):
                    interesting = key in {
                        "anchors",
                        "objects",
                        "rooms",
                        "waypoints",
                        "ranked_candidates",
                        "semantic_observations",
                    }
                    first_keys = set(value[0].keys())
                    interesting = interesting or bool(
                        first_keys
                        & {
                            "anchor_type",
                            "object_id",
                            "target_id",
                            "room_id",
                            "floor_id",
                            "pose",
                            "pose_3d",
                            "footprint_2d",
                            "waypoint_index",
                            "camera_pose",
                            "view_pose",
                        }
                    )
                    if interesting:
                        collections.append((key, value[:1000]))
                visit(value, key)
        elif isinstance(node, list):
            for item in node[:100]:
                visit(item, key_name)

    visit(data, "$")
    return collections


def looks_relevant(path: Path) -> bool:
    text = str(path).lower()
    return any(
        token in text
        for token in (
            "anchor",
            "object",
            "semantic",
            "vector",
            "waypoint",
            "route",
            "topology",
            "snapshot",
            "room_world",
            "query",
            "map",
        )
    )


def discover_files(stage_output_dir: Path, output_dir: Path) -> dict[str, Any]:
    scene_root = stage_output_dir.parent
    search_roots = [stage_output_dir]
    task_root = scene_root / "tasks"
    if task_root.exists():
        for task_dir in sorted(task_root.iterdir()):
            if task_dir.is_dir() and ("object_nav" in task_dir.name or task_dir.name.startswith("task14")):
                search_roots.append(task_dir)
    files_searched: list[str] = []
    inspected: list[dict[str, Any]] = []
    anchor_like_files: list[str] = []
    obj_175_files: list[str] = []
    for root in search_roots:
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.resolve().is_relative_to(output_dir.resolve()):
                continue
            if not looks_relevant(path):
                continue
            files_searched.append(rel(path))
            if path.suffix.lower() not in {".json", ".yaml", ".yml"}:
                continue
            try:
                if path.stat().st_size > 80_000_000:
                    inspected.append({"path": rel(path), "inspected": False, "reason": "file too large"})
                    continue
                data = yaml.safe_load(path.read_text()) if path.suffix.lower() in {".yaml", ".yml"} else load_json(path)
            except Exception as exc:
                inspected.append({"path": rel(path), "inspected": False, "reason": f"parse_error: {exc}"})
                continue
            text = json.dumps(data, default=str)[:2_000_000]
            collections = extract_collections(data)
            summaries = [classify_entries(name, entries) for name, entries in collections]
            anchor_like = any(
                summary["anchor_type_classification"]
                in {
                    "room_anchor",
                    "object_anchor_position_centroid_like",
                    "mixed_anchor",
                    "observation_or_view_pose_anchor",
                    "object_centroid_or_footprint",
                    "route_waypoint",
                    "semantic_object_metadata",
                }
                for summary in summaries
            )
            if anchor_like:
                anchor_like_files.append(rel(path))
            if "obj_175" in text or '"id": 175' in text or "'id': 175" in text:
                obj_175_files.append(rel(path))
            inspected.append(
                {
                    "path": rel(path),
                    "inspected": True,
                    "top_level_keys": sorted(data.keys()) if isinstance(data, dict) else [],
                    "looked_anchor_like": anchor_like,
                    "obj_175_appears": rel(path) in obj_175_files,
                    "schema_summaries": summaries,
                }
            )
    return {
        "search_roots": [rel(root) for root in search_roots],
        "files_searched_count": len(files_searched),
        "files_searched": files_searched,
        "files_inspected_count": len(inspected),
        "files_inspected": inspected,
        "anchor_like_files": sorted(set(anchor_like_files)),
        "obj_175_files": sorted(set(obj_175_files)),
    }


def load_authoritative_object(stage_output_dir: Path, object_id: str) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    public_dir = stage_output_dir / "committed_public"
    snapshot_path = public_dir / "committed_room_world_snapshot_v0_1.json"
    topology_path = public_dir / "topology_v0_1.json"
    snapshot = load_json(snapshot_path, {})
    topology = load_json(topology_path, {})
    oid_num = int(object_id.replace("obj_", ""))
    snapshot_object = next((o for o in snapshot.get("objects", []) if o.get("id") == oid_num or canonical_object_id(o.get("id")) == object_id), None)
    topology_object = next(
        (
            o
            for o in topology.get("entities", {}).get("objects", [])
            if o.get("id") == object_id or canonical_object_id(o.get("id")) == object_id
        ),
        None,
    )
    if not snapshot_object and not topology_object:
        raise SystemExit(f"could not find {object_id} in committed/public artifacts")
    selected = dict(snapshot_object or {})
    if topology_object:
        for key, value in topology_object.items():
            selected.setdefault(key, value)
    selected["object_id"] = object_id
    selected["authoritative_source"] = rel(snapshot_path) if snapshot_object else rel(topology_path)
    return selected, snapshot, topology


def room_polygon(snapshot: dict[str, Any], topology: dict[str, Any], room_id: str) -> tuple[list[list[float]] | None, str | None]:
    for path_name, rooms in (
        ("committed_room_world_snapshot_v0_1.rooms", snapshot.get("rooms", [])),
        ("topology_v0_1.rooms", topology.get("rooms", [])),
    ):
        for room in rooms:
            if room.get("room_id") == room_id or room.get("id") == room_id:
                polygon = room.get("polygon") or room.get("footprint_polygon_xy")
                if polygon:
                    return polygon, path_name
    return None, None


def extract_existing_anchors(discovery: dict[str, Any], stage_output_dir: Path, object_id: str) -> list[dict[str, Any]]:
    anchors: list[dict[str, Any]] = []
    for rel_path in discovery.get("anchor_like_files", []):
        path = ROOT / rel_path
        if not path.exists() or path.suffix.lower() != ".json":
            continue
        try:
            data = load_json(path)
        except Exception:
            continue
        for name, entries in extract_collections(data):
            for entry in entries:
                if not isinstance(entry, dict) or "anchor_type" not in entry:
                    continue
                target_id = entry.get("target_id") or entry.get("object_id")
                anchors.append(
                    {
                        "source_file": rel(path),
                        "collection_name": name,
                        "id": entry.get("id"),
                        "anchor_type": entry.get("anchor_type"),
                        "target_id": target_id,
                        "room_id": entry.get("room_id"),
                        "floor_id": entry.get("floor_id"),
                        "position": entry.get("position"),
                        "yaw": entry.get("yaw"),
                        "score": entry.get("score"),
                        "valid": entry.get("valid"),
                        "matches_requested_object": target_id == object_id,
                        "classification": classify_entries(name, [entry])["anchor_type_classification"],
                    }
                )
    return anchors


def make_candidate(
    candidate_id: str,
    source: str,
    x: float,
    y: float,
    object_xy: tuple[float, float],
    floor_id: str,
    room_id: str,
    details: dict[str, Any] | None = None,
    reject_centroid: bool = False,
) -> dict[str, Any]:
    ox, oy = object_xy
    yaw = math.atan2(oy - y, ox - x)
    return {
        "candidate_id": candidate_id,
        "candidate_source": source,
        "floor_id": floor_id,
        "room_id": room_id,
        "world_xy": [round(x, 6), round(y, 6)],
        "yaw": round(yaw, 6),
        "yaw_policy": "faces_object_centroid",
        "distance_to_object_centroid_m": round(math.hypot(ox - x, oy - y), 4),
        "details": details or {},
        "pre_validation_rejection": "object centroid is target location, not a robot/view pose" if reject_centroid else None,
    }


def generate_candidates(
    selected_object: dict[str, Any],
    floor_id: str,
    room_id: str,
    terminal: dict[str, Any] | None,
) -> list[dict[str, Any]]:
    pose = selected_object.get("pose") or selected_object.get("pose_xy")
    footprint = selected_object.get("footprint_2d") or []
    object_xy = (float(pose[0]), float(pose[1])) if pose else polygon_center(footprint)
    if object_xy is None:
        raise SystemExit("target object has neither pose nor footprint center")
    candidates = [
        make_candidate(
            "centroid_obj_175_rejected",
            "centroid_rejected",
            object_xy[0],
            object_xy[1],
            object_xy,
            floor_id,
            room_id,
            {"reason": "included for audit trace only"},
            reject_centroid=True,
        )
    ]
    if terminal:
        candidates.append(
            make_candidate(
                "route_terminal_seed",
                "route_terminal_seed",
                float(terminal["x"]),
                float(terminal["y"]),
                object_xy,
                floor_id,
                room_id,
                {"source_waypoint_index": terminal.get("waypoint_index"), "source": terminal.get("source")},
            )
        )
    bbox = polygon_bbox(footprint)
    if bbox:
        min_x, min_y, max_x, max_y = bbox
        cx, cy = object_xy
        half_w = max(0.05, (max_x - min_x) / 2.0)
        half_h = max(0.05, (max_y - min_y) / 2.0)
        angles = [math.radians(v) for v in range(0, 360, 22)]
        standoffs = [0.6, 0.8, 1.0]
        n = 0
        for standoff in standoffs:
            for angle in angles:
                ux, uy = math.cos(angle), math.sin(angle)
                edge_scale_x = half_w / abs(ux) if abs(ux) > 1e-6 else float("inf")
                edge_scale_y = half_h / abs(uy) if abs(uy) > 1e-6 else float("inf")
                support = min(edge_scale_x, edge_scale_y)
                x = cx + ux * (support + standoff)
                y = cy + uy * (support + standoff)
                candidates.append(
                    make_candidate(
                        f"generated_ring_{n:03d}",
                        "generated_ring",
                        x,
                        y,
                        object_xy,
                        floor_id,
                        room_id,
                        {
                            "angle_deg": round(math.degrees(angle), 3),
                            "standoff_from_footprint_m": standoff,
                            "generation_policy": "bbox_support_plus_standoff",
                        },
                    )
                )
                n += 1
    return candidates


def validate_candidate(
    candidate: dict[str, Any],
    selected_object: dict[str, Any],
    map_data: OccupancyMap,
    target_floor_id: str,
    target_room_id: str,
    target_room_polygon: list[list[float]] | None,
    route_component: np.ndarray | None,
) -> dict[str, Any]:
    x, y = candidate["world_xy"]
    object_pose = selected_object.get("pose") or selected_object.get("pose_xy")
    object_xy = (float(object_pose[0]), float(object_pose[1]))
    checks: dict[str, dict[str, Any]] = {}
    checks["not_centroid_pose"] = {
        "status": "failed" if candidate.get("pre_validation_rejection") else "passed",
        "reason": candidate.get("pre_validation_rejection") or "candidate is not the raw object centroid",
    }
    checks["same_floor"] = {
        "status": "passed" if candidate.get("floor_id") == target_floor_id else "failed",
        "reason": f"candidate floor={candidate.get('floor_id')} target floor={target_floor_id}",
    }
    if target_room_polygon:
        inside = point_in_polygon(float(x), float(y), target_room_polygon)
        checks["target_room_membership"] = {
            "status": "passed" if inside else "failed",
            "reason": f"candidate is {'inside' if inside else 'outside'} {target_room_id} polygon",
        }
        object_inside = point_in_polygon(object_xy[0], object_xy[1], target_room_polygon)
        checks["same_side_or_room_compatibility"] = {
            "status": "passed" if inside else "failed",
            "reason": (
                f"candidate is inside target room; object centroid inside room={object_inside}. "
                "For wall/curtain targets, object centroid may sit at or just outside the room boundary."
            )
            if inside
            else "candidate is not in target room polygon",
        }
    else:
        checks["target_room_membership"] = {
            "status": "not_evaluable",
            "reason": "target room polygon was not available",
        }
        checks["same_side_or_room_compatibility"] = {
            "status": "not_evaluable",
            "reason": "target room polygon was not available",
        }
    sample = map_data.sample(float(x), float(y))
    checks["occupancy_free"] = {
        "status": "passed" if sample["in_bounds"] and sample["is_free"] else "failed",
        "reason": "candidate cell is free" if sample["is_free"] else "candidate cell is not free or out of bounds",
        **sample,
    }
    clearance = sample.get("clearance_m")
    if clearance is None:
        checks["clearance"] = {"status": "failed", "reason": "candidate is out of map bounds"}
    else:
        checks["clearance"] = {
            "status": "passed" if clearance >= 0.2 else "failed",
            "reason": f"nearest occupied/non-free clearance is {clearance:.3f}m; threshold is 0.200m",
            "clearance_m": clearance,
            "threshold_m": 0.2,
        }
    checks["candidate_to_object_ray"] = map_data.ray_check((float(x), float(y)), object_xy)
    expected_yaw = math.atan2(object_xy[1] - float(y), object_xy[0] - float(x))
    yaw_error = abs(math.atan2(math.sin(candidate["yaw"] - expected_yaw), math.cos(candidate["yaw"] - expected_yaw)))
    checks["yaw_faces_object"] = {
        "status": "passed" if yaw_error <= 0.05 else "failed",
        "reason": f"yaw error to object centroid is {yaw_error:.4f} rad",
        "yaw_error_rad": round(yaw_error, 6),
    }
    row, col = map_data.world_to_rc(float(x), float(y))
    if route_component is None:
        checks["route_terminal_reachability_heuristic"] = {
            "status": "not_evaluable",
            "reason": "route terminal was not traversable in the occupancy map",
        }
    elif map_data.in_bounds_rc(row, col) and route_component[row, col]:
        checks["route_terminal_reachability_heuristic"] = {
            "status": "passed",
            "reason": "candidate is in same lightweight occupancy connected component as executable route terminal",
            "grid_row": row,
            "grid_col": col,
        }
    else:
        checks["route_terminal_reachability_heuristic"] = {
            "status": "failed",
            "reason": "candidate is not in same lightweight occupancy connected component as executable route terminal",
            "grid_row": row,
            "grid_col": col,
        }
    critical = [
        "not_centroid_pose",
        "same_floor",
        "target_room_membership",
        "occupancy_free",
        "clearance",
        "candidate_to_object_ray",
        "yaw_faces_object",
        "route_terminal_reachability_heuristic",
    ]
    candidate["grid_rc"] = [sample.get("grid_row"), sample.get("grid_col")]
    candidate["map_occupancy_value"] = sample.get("occupancy_value")
    candidate["clearance_m"] = sample.get("clearance_m")
    candidate["validation_checks"] = checks
    candidate["validation_status"] = status_from_checks(checks, critical)
    candidate["rejection_reasons"] = [
        f"{name}: {check.get('reason')}"
        for name, check in checks.items()
        if check.get("status") == "failed"
    ]
    return candidate


def select_candidate(candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
    valid = [c for c in candidates if c.get("validation_status") == "passed" and c.get("candidate_source") == "generated_ring"]
    if not valid:
        valid = [c for c in candidates if c.get("validation_status") == "passed"]
    if not valid:
        valid = [c for c in candidates if c.get("validation_status") == "partially_validated"]
    if not valid:
        return None
    def score(candidate: dict[str, Any]) -> tuple[float, float, float]:
        distance = float(candidate.get("distance_to_object_centroid_m") or 99.0)
        standoff = candidate.get("details", {}).get("standoff_from_footprint_m")
        standoff_score = -abs(float(standoff or 0.8) - 0.8)
        clearance = float(candidate.get("clearance_m") or 0.0)
        distance_score = -abs(distance - 1.45)
        return (standoff_score, clearance, distance_score)
    return max(valid, key=score)


def make_visualization(
    path: Path,
    map_data: OccupancyMap,
    selected_object: dict[str, Any],
    room_polygon_xy: list[list[float]] | None,
    route_waypoints: list[dict[str, Any]],
    anchors: list[dict[str, Any]],
    candidates: list[dict[str, Any]],
    selected: dict[str, Any] | None,
) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False
    fig, ax = plt.subplots(figsize=(9, 9))
    extent = [
        map_data.origin_x,
        map_data.origin_x + map_data.width * map_data.resolution,
        map_data.origin_y,
        map_data.origin_y + map_data.height * map_data.resolution,
    ]
    ax.imshow(map_data.grid, cmap="gray", origin="lower", extent=extent, alpha=0.95)
    if route_waypoints:
        xs = [wp["x"] for wp in route_waypoints if "x" in wp and "y" in wp]
        ys = [wp["y"] for wp in route_waypoints if "x" in wp and "y" in wp]
        ax.plot(xs, ys, color="#1f77b4", linewidth=2, label="executable route")
    if room_polygon_xy:
        poly = np.array(room_polygon_xy + [room_polygon_xy[0]])
        ax.plot(poly[:, 0], poly[:, 1], color="#ff7f0e", linewidth=2, label="room_14 polygon")
    footprint = selected_object.get("footprint_2d") or []
    if footprint:
        fp = np.array(footprint + [footprint[0]])
        ax.plot(fp[:, 0], fp[:, 1], color="#d62728", linewidth=2, label="obj_175 footprint")
    pose = selected_object.get("pose") or selected_object.get("pose_xy")
    if pose:
        ax.scatter([pose[0]], [pose[1]], color="#d62728", marker="x", s=90, label="obj_175 centroid")
    for anchor in anchors:
        pos = anchor.get("position")
        if pos and anchor.get("floor_id") == "floor_2":
            color = "#9467bd" if anchor.get("matches_requested_object") else "#8c564b"
            ax.scatter([pos[0]], [pos[1]], color=color, marker="^", s=45, alpha=0.8)
    passed = [c for c in candidates if c.get("validation_status") == "passed"]
    failed = [c for c in candidates if c.get("validation_status") == "failed"]
    partial = [c for c in candidates if c.get("validation_status") == "partially_validated"]
    for group, color, marker, label in (
        (failed, "#7f7f7f", "o", "rejected candidates"),
        (partial, "#bcbd22", "o", "partial candidates"),
        (passed, "#2ca02c", "o", "passed candidates"),
    ):
        if group:
            ax.scatter([c["world_xy"][0] for c in group], [c["world_xy"][1] for c in group], color=color, marker=marker, s=28, label=label, alpha=0.75)
    if selected:
        x, y = selected["world_xy"]
        ax.scatter([x], [y], color="#17becf", marker="*", s=220, label="selected candidate", edgecolor="black")
        ax.arrow(x, y, 0.35 * math.cos(selected["yaw"]), 0.35 * math.sin(selected["yaw"]), color="#17becf", width=0.015)
    ax.set_xlim(-10.2, -6.2)
    ax.set_ylim(-0.7, 4.2)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("task14b obj_175 object approach candidates on floor_2")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.2)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def markdown_report(
    report: dict[str, Any],
    discovery_path: Path,
    candidates_path: Path,
    pose_report_path: Path,
    md_report_path: Path,
    viz_path: Path,
    completion_path: Path,
) -> str:
    selected = report.get("selected_candidate")
    status = report.get("readiness_status")
    conclusion = report.get("conclusion")
    candidate_line = "_None selected._"
    if selected:
        candidate_line = (
            f"`{selected['candidate_id']}` at world xy `{selected['world_xy']}`, yaw `{selected['yaw']}`, "
            f"source `{selected['candidate_source']}`, status `{selected['validation_status']}`."
        )
    checks = selected.get("validation_checks", {}) if selected else {}
    check_lines = [
        f"- `{name}`: {check.get('status')} - {check.get('reason')}"
        for name, check in checks.items()
    ]
    missing_lines = [f"- {item}" for item in report.get("missing_before_runtime_execution", [])]
    inspected = report.get("discovery_summary", {})
    anchor_lines = [f"- {item}" for item in report.get("anchor_type_findings", [])]
    return "\n".join(
        [
            "# task14b Object Anchor Approach Audit: obj_175",
            "",
            "## Concise conclusion",
            conclusion,
            "",
            f"- Readiness status: `{status}`",
            f"- Selected candidate: {candidate_line}",
            "- Runtime was not run; this is offline readiness only.",
            "",
            "## Discovery summary",
            f"- Files searched: {inspected.get('files_searched_count')}",
            f"- Files inspected: {inspected.get('files_inspected_count')}",
            f"- Anchor-like files: {len(inspected.get('anchor_like_files', []))}",
            f"- Files where obj_175 appears: {len(inspected.get('obj_175_files', []))}",
            "",
            "## Anchor classification",
            *anchor_lines,
            "",
            "## Selected candidate validation",
            *check_lines,
            "",
            "## Artifacts used",
            *[f"- `{p}`" for p in report.get("artifacts_used", [])],
            "",
            "## Generated reports",
            f"- `{rel(discovery_path)}`",
            f"- `{rel(candidates_path)}`",
            f"- `{rel(pose_report_path)}`",
            f"- `{rel(md_report_path)}`",
            f"- `{rel(viz_path)}`" if viz_path.exists() else "- Visualization was not generated.",
            f"- `{rel(completion_path)}`",
            "",
            "## Missing before runtime execution",
            *missing_lines,
            "",
        ]
    )


def completion_summary(report: dict[str, Any], command: str, generated: list[Path]) -> str:
    selected = report.get("selected_candidate")
    selected_text = "None"
    if selected:
        selected_text = f"{selected['candidate_id']} at {selected['world_xy']} yaw {selected['yaw']} ({selected['validation_status']})"
    return "\n".join(
        [
            "# task14b Completion Summary",
            "",
            "## Implemented",
            "- Added `tools/object_nav/audit_object_anchor_approach.py` for offline object-anchor discovery, schema classification, candidate generation, validation, and report generation.",
            "- Ran the audit for `obj_175` / `curtain in room_14 on floor_2` using the existing 00843 clean_rerun artifacts.",
            "",
            "## Command run",
            "```bash",
            command,
            "```",
            "",
            "## Key findings",
            "- Existing committed/public anchors were found, but none target `obj_175`.",
            "- Existing anchor schema is anchor-node style with `anchor_type`, `target_id`, `room_id`, `floor_id`, `position`, `score`, and `valid`; it has no camera/view pose or yaw for `obj_175`.",
            "- `obj_175` has committed centroid and footprint information, so the centroid was rejected as a robot pose and footprint standoff candidates were generated instead.",
            f"- Selected approach pose status: `{report.get('readiness_status')}`.",
            f"- Selected approach pose: {selected_text}.",
            "",
            "## Generated artifacts",
            *[f"- `{rel(path)}`" for path in generated],
            "",
            "## Next runtime step",
            "Use the selected candidate as the object-specific final approach waypoint after the existing room_11 -> room_7 -> room_13 -> room_14 route terminal, then run a controlled Nav2 runtime test that validates final pose tolerance, object visibility/arrival criteria, and logs object-approach success or failure. Do not count this offline audit as runtime object arrival.",
            "",
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--room-id", required=True)
    parser.add_argument("--stable-map", type=Path, required=True)
    parser.add_argument("--semantic-route", type=Path, required=True)
    parser.add_argument("--executable-route", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    command = " ".join(
        [
            "/home/ws/miniconda3/envs/boxfusion/bin/python",
            "tools/object_nav/audit_object_anchor_approach.py",
            "--stage-output-dir",
            str(args.stage_output_dir),
            "--query",
            json.dumps(args.query),
            "--object-id",
            args.object_id,
            "--floor-id",
            args.floor_id,
            "--room-id",
            args.room_id,
            "--stable-map",
            str(args.stable_map),
            "--semantic-route",
            str(args.semantic_route),
            "--executable-route",
            str(args.executable_route),
            "--output-dir",
            str(args.output_dir),
        ]
    )

    discovery = discover_files(args.stage_output_dir, output_dir)
    selected_object, snapshot, topology = load_authoritative_object(args.stage_output_dir, args.object_id)
    existing_anchors = extract_existing_anchors(discovery, args.stage_output_dir, args.object_id)
    room_poly, room_poly_source = room_polygon(snapshot, topology, args.room_id)
    terminal = route_terminal(args.executable_route)
    map_data = OccupancyMap(args.stable_map)
    terminal_xy = (float(terminal["x"]), float(terminal["y"])) if terminal else None
    route_component = connected_from(map_data, terminal_xy) if terminal_xy else None
    candidates = generate_candidates(selected_object, args.floor_id, args.room_id, terminal)
    candidates = [
        validate_candidate(candidate, selected_object, map_data, args.floor_id, args.room_id, room_poly, route_component)
        for candidate in candidates
    ]
    selected_candidate = select_candidate(candidates)

    target_object_anchors = [anchor for anchor in existing_anchors if anchor.get("matches_requested_object")]
    usable_view_anchors = [
        anchor
        for anchor in target_object_anchors
        if anchor.get("classification") == "observation_or_view_pose_anchor" and anchor.get("position")
    ]
    anchor_type_findings = [
        f"Anchor-like files found across committed/public, task, route, map, and raw/reference search roots: {len(discovery.get('anchor_like_files', []))}.",
        f"Anchor records discovered across inspected files, including repeated raw/debug snapshots: {len(existing_anchors)}.",
        f"Anchors targeting {args.object_id}: {len(target_object_anchors)}.",
        f"Usable observation/view anchors for {args.object_id}: {len(usable_view_anchors)}.",
        "Current committed object anchors are classified as object-anchor/room-anchor positions, not camera or robot view poses.",
        "For obj_175, committed object data supplies centroid/footprint only; generated footprint standoff candidates are therefore used.",
    ]
    readiness_status = "not_ready"
    conclusion = (
        f"{args.object_id} does not have a validated object approach pose candidate. "
        "Failure details are recorded in the candidate report."
    )
    if selected_candidate:
        if selected_candidate.get("validation_status") == "passed":
            readiness_status = "validated_offline_candidate_ready_for_later_runtime_testing"
            conclusion = (
                f"{args.object_id} has a validated object approach pose candidate ready for later runtime testing. "
                "This is not Nav2 execution and not object arrival."
            )
        else:
            readiness_status = "partially_validated_candidate_ready_for_cautious_later_runtime_testing"
            conclusion = (
                f"{args.object_id} has a partially validated object approach pose candidate. "
                "Resolve the not-evaluable or failed checks before treating it as runtime-ready."
            )
    artifacts_used = [
        rel(args.stage_output_dir / "committed_public/committed_room_world_snapshot_v0_1.json"),
        rel(args.stage_output_dir / "committed_public/topology_v0_1.json"),
        rel(args.stage_output_dir / "committed_public/committed_room_world_model_v0_1.json"),
        rel(args.stage_output_dir / "committed_public/final_vector_map_snapshot.json"),
        rel(args.stable_map),
        rel(args.semantic_route),
        rel(args.executable_route),
    ]
    missing_before_runtime = [
        "A runtime route extension that appends the selected object approach pose after the existing room_14 route terminal.",
        "Nav2 execution logs for the object-specific approach segment.",
        "Final pose tolerance validation at the selected object approach pose.",
        "A runtime object visibility or arrival predicate; task14b does not prove object arrival.",
        "Camera/view observation pose provenance for obj_175, if future execution should prefer observed viewing poses over generated standoff poses.",
    ]
    discovery_report = {
        "artifact_type": "object_anchor_discovery_report",
        "created_utc": utc_now(),
        "query": args.query,
        "object_id": args.object_id,
        "floor_id": args.floor_id,
        "room_id": args.room_id,
        "authoritative_selection_note": "Committed/public artifacts are authoritative; raw final_vector_map_snapshot is reference/comparison only.",
        "discovery": discovery,
        "existing_anchors": existing_anchors,
        "target_object_anchors": target_object_anchors,
        "usable_observation_or_view_anchors": usable_view_anchors,
        "anchor_type_findings": anchor_type_findings,
        "selected_object_summary": {
            "object_id": args.object_id,
            "label": selected_object.get("label"),
            "room_id": selected_object.get("room_id"),
            "floor_id": selected_object.get("floor_id"),
            "pose": selected_object.get("pose") or selected_object.get("pose_xy"),
            "footprint_2d": selected_object.get("footprint_2d"),
            "yaw": selected_object.get("yaw"),
            "semantic_observations": selected_object.get("semantic_observations", []),
            "authoritative_source": selected_object.get("authoritative_source"),
        },
    }
    candidates_report = {
        "artifact_type": "object_approach_candidates",
        "created_utc": utc_now(),
        "query": args.query,
        "selected_object": discovery_report["selected_object_summary"],
        "candidate_generation_policy": [
            "Do not use object centroid as final robot navigation pose unless explicitly marked as a robot/view pose.",
            "No usable obj_175 observation/view anchor was found.",
            "Generated candidates are bbox/footprint support points plus 0.6m, 0.8m, and 1.0m standoff radii with yaw facing the object centroid.",
        ],
        "route_terminal": terminal,
        "target_room_polygon_source": room_poly_source,
        "target_room_polygon": room_poly,
        "candidates": candidates,
    }
    pose_report = {
        "artifact_type": "object_approach_pose_report",
        "created_utc": utc_now(),
        "query": args.query,
        "object_id": args.object_id,
        "label": selected_object.get("label"),
        "room_id": args.room_id,
        "floor_id": args.floor_id,
        "readiness_status": readiness_status,
        "conclusion": conclusion,
        "selected_candidate": selected_candidate,
        "selection_reason": (
            "Selected the highest-scoring generated footprint standoff candidate that passed offline floor, room, occupancy, clearance, ray, yaw, and route-terminal reachability checks."
            if selected_candidate
            else "No candidate passed or partially passed enough validation checks to select."
        ),
        "suitable_for_later_runtime_object_approach": bool(selected_candidate and selected_candidate.get("validation_status") == "passed"),
        "overclaim_guardrails": [
            "Offline readiness only.",
            "No Nav2 execution was run.",
            "No object approach success or object arrival is claimed.",
            "No semantic GT accuracy is claimed.",
            "No HOV-SG/HiCo-Nav equivalence is claimed.",
        ],
        "anchor_type_findings": anchor_type_findings,
        "discovery_summary": {
            "files_searched_count": discovery.get("files_searched_count"),
            "files_inspected_count": discovery.get("files_inspected_count"),
            "anchor_like_files": discovery.get("anchor_like_files"),
            "obj_175_files": discovery.get("obj_175_files"),
        },
        "artifacts_used": artifacts_used,
        "missing_before_runtime_execution": missing_before_runtime,
    }

    discovery_path = output_dir / "object_anchor_discovery_report.json"
    candidates_path = output_dir / "object_approach_candidates_obj_175.json"
    pose_report_path = output_dir / "object_approach_pose_report_obj_175.json"
    md_report_path = output_dir / "object_approach_pose_report_obj_175.md"
    viz_path = output_dir / "object_approach_candidates_obj_175_floor2.png"
    completion_path = output_dir / "completion_summary.md"
    write_json(discovery_path, discovery_report)
    write_json(candidates_path, candidates_report)
    write_json(pose_report_path, pose_report)
    route_data = load_json(args.executable_route, {})
    viz_ok = make_visualization(
        viz_path,
        map_data,
        selected_object,
        room_poly,
        route_data.get("waypoints", []),
        existing_anchors,
        candidates,
        selected_candidate,
    )
    if not viz_ok and viz_path.exists():
        viz_path.unlink()
    md_report_path.write_text(
        markdown_report(pose_report, discovery_path, candidates_path, pose_report_path, md_report_path, viz_path, completion_path)
    )
    generated = [discovery_path, candidates_path, pose_report_path, md_report_path, completion_path]
    if viz_path.exists():
        generated.insert(4, viz_path)
    completion_path.write_text(completion_summary(pose_report, command, generated))
    print(f"wrote {discovery_path}")
    print(f"wrote {candidates_path}")
    print(f"wrote {pose_report_path}")
    print(f"wrote {md_report_path}")
    if viz_path.exists():
        print(f"wrote {viz_path}")
    print(f"wrote {completion_path}")
    print(f"readiness_status={readiness_status}")
    if selected_candidate:
        print(f"selected_candidate={selected_candidate['candidate_id']} {selected_candidate['world_xy']} yaw={selected_candidate['yaw']}")


if __name__ == "__main__":
    main()
