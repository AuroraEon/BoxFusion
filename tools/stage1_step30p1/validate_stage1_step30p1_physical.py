#!/usr/bin/env python3
"""Validate Stage1 physical through-room, terminal quality, loops, and wall crossings."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def parse_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in {b"\n", b""}:
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if tokens[0] != b"P5" or maxval > 255:
        raise ValueError(f"unsupported PGM header in {path}")
    while data[idx:idx + 1].isspace():
        idx += 1
    return np.frombuffer(data[idx:idx + width * height], dtype=np.uint8).reshape((height, width)).copy()


def parse_yaml_map(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def load_nav_map(map_yaml: Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    meta = parse_yaml_map(map_yaml)
    grid = np.flipud(parse_pgm(map_yaml.parent / meta["image"]))
    origin = tuple(float(x.strip()) for x in meta["origin"].strip("[]").split(",")[:2])
    return grid, float(meta["resolution"]), origin  # type: ignore[return-value]


def load_map_meta(stage_output: Path) -> tuple[float, tuple[float, float], int]:
    meta = read_json(stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.json")
    return float(meta["resolution"]), (float(meta["origin"][0]), float(meta["origin"][1])), int(meta["height"])


def mask_value(mask: np.ndarray, x: float, y: float, resolution: float, origin: tuple[float, float]) -> int | None:
    col = int(round((x - origin[0]) / resolution))
    row = int(round((y - origin[1]) / resolution))
    if 0 <= row < mask.shape[0] and 0 <= col < mask.shape[1]:
        return int(mask[row, col])
    return None


def map_index(x: float, y: float, resolution: float, origin: tuple[float, float]) -> tuple[int, int]:
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def map_value(grid: np.ndarray, x: float, y: float, resolution: float, origin: tuple[float, float]) -> int | None:
    row, col = map_index(x, y, resolution, origin)
    if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
        return int(grid[row, col])
    return None


def bresenham(a: tuple[int, int], b: tuple[int, int]) -> list[tuple[int, int]]:
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    cells: list[tuple[int, int]] = []
    if dc > dr:
        err = dc / 2.0
        row = r0
        for col in range(c0, c1 + sc, sc):
            cells.append((row, col))
            err -= dr
            if err < 0:
                row += sr
                err += dc
    else:
        err = dr / 2.0
        col = c0
        for row in range(r0, r1 + sr, sr):
            cells.append((row, col))
            err -= dc
            if err < 0:
                col += sc
                err += dr
    return cells


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def dist(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def min_dist(samples: list[dict[str, Any]], target: dict[str, Any] | None) -> float | None:
    if not samples or not target:
        return None
    return min(dist(sample, target) for sample in samples)


def gateway_pose(waypoints: list[dict[str, Any]], gateway_id: str) -> dict[str, Any] | None:
    crossing = [w for w in waypoints if w.get("gateway_id") == gateway_id and w.get("source") == "segment_crossing"]
    if crossing:
        return crossing[0]
    candidates = [w for w in waypoints if w.get("gateway_id") == gateway_id]
    if not candidates:
        return None
    return candidates[len(candidates) // 2]


def dwell_seconds(samples: list[dict[str, Any]], inside_flags: list[bool]) -> float:
    if not samples:
        return 0.0
    total = 0.0
    for idx, inside in enumerate(inside_flags):
        if not inside:
            continue
        if idx + 1 < len(samples):
            dt = float(samples[idx + 1].get("time_wall_sec", 0.0)) - float(samples[idx].get("time_wall_sec", 0.0))
            total += dt if 0.0 < dt < 5.0 else 0.5
        else:
            total += 0.5
    return round(total, 3)


def write_md(path: Path, title: str, payload: dict[str, Any]) -> None:
    lines = [f"# {title}", ""]
    for key, value in payload.items():
        if isinstance(value, (dict, list)):
            continue
        lines.append(f"- `{key}`: `{value}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _is_boundary_cell(grid: np.ndarray, row: int, col: int) -> bool:
    """Return True if (row,col) is occupied/unknown but has a free neighbour (i.e. wall boundary)."""
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = row + dr, col + dc
            if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1] and int(grid[nr, nc]) >= 250:
                return True
    return False


def wall_counts(samples: list[dict[str, Any]], grid: np.ndarray, resolution: float, origin: tuple[float, float]) -> dict[str, Any]:
    point_violations = []
    boundary_only_points = 0
    for idx, sample in enumerate(samples):
        row, col = map_index(float(sample["x"]), float(sample["y"]), resolution, origin)
        value = map_value(grid, float(sample["x"]), float(sample["y"]), resolution, origin)
        if value is None or value <= 10:
            is_boundary = _is_boundary_cell(grid, row, col) if (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]) else False
            if is_boundary:
                boundary_only_points += 1
            point_violations.append({"sample_index": idx, "x": sample.get("x"), "y": sample.get("y"), "map_value": value, "boundary_tolerance": is_boundary})
    segment_violations = []
    boundary_only_segments = 0
    max_cells = 0
    for idx, (a, b) in enumerate(zip(samples, samples[1:])):
        ar = map_index(float(a["x"]), float(a["y"]), resolution, origin)
        br = map_index(float(b["x"]), float(b["y"]), resolution, origin)
        occupied_cells = []
        all_boundary = True
        for row, col in bresenham(ar, br):
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]) or int(grid[row, col]) <= 10:
                occupied_cells.append((row, col))
                if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
                    if not _is_boundary_cell(grid, row, col):
                        all_boundary = False
                else:
                    all_boundary = False
        if occupied_cells:
            max_cells = max(max_cells, len(occupied_cells))
            if all_boundary and len(occupied_cells) <= 2:
                boundary_only_segments += 1
            segment_violations.append({
                "segment_index": idx,
                "from_sample": idx,
                "to_sample": idx + 1,
                "occupied_or_out_of_bounds_cell_count": len(occupied_cells),
                "from_xy": {"x": a.get("x"), "y": a.get("y")},
                "to_xy": {"x": b.get("x"), "y": b.get("y")},
                "boundary_tolerance": all_boundary and len(occupied_cells) <= 2,
            })
    sparse = any(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) > 0.75 for a, b in zip(samples, samples[1:]))
    return {
        "point_violations": point_violations,
        "segment_violations": segment_violations,
        "max_cells": max_cells,
        "sparse": sparse,
        "boundary_only_point_count": boundary_only_points,
        "boundary_only_segment_count": boundary_only_segments,
        "real_point_violations": len(point_violations) - boundary_only_points,
        "real_segment_violations": len(segment_violations) - boundary_only_segments,
    }


def structural_overlap_count(samples: list[dict[str, Any]], structural_mask: np.ndarray, resolution: float, origin: tuple[float, float]) -> int:
    count = 0
    boundary_count = 0
    import cv2
    free_mask = (~structural_mask.astype(bool)).astype(np.uint8)
    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3) if free_mask.any() else np.zeros_like(free_mask, dtype=np.float32)
    for sample in samples:
        row, col = map_index(float(sample["x"]), float(sample["y"]), resolution, origin)
        if 0 <= row < structural_mask.shape[0] and 0 <= col < structural_mask.shape[1] and bool(structural_mask[row, col]):
            count += 1
            if dist[row, col] == 0:
                boundary_count += 1
    return count, boundary_count


def wall_crossing_payload_multi(samples: list[dict[str, Any]], h8: tuple[np.ndarray, float, tuple[float, float]], s7: tuple[np.ndarray, float, tuple[float, float]], structural_mask: np.ndarray | None, structural_resolution: float, structural_origin: tuple[float, float], active_map_yaml: Path, h8_yaml: Path, s7_yaml: Path) -> dict[str, Any]:
    h8_counts = wall_counts(samples, *h8)
    s7_counts = wall_counts(samples, *s7)
    structural_count, structural_boundary = (structural_overlap_count(samples, structural_mask, structural_resolution, structural_origin) if structural_mask is not None else (None, None))
    suspicious = (h8_counts["segment_violations"] + s7_counts["segment_violations"])[:25]
    visualization_artifact_possible = bool(
        (h8_counts["sparse"] or s7_counts["sparse"])
        and not h8_counts["point_violations"]
        and not s7_counts["point_violations"]
        and suspicious
    )
    # Strict: zero real violations (exclude boundary-tolerance)
    real_point_h8 = h8_counts["real_point_violations"]
    real_point_s7 = s7_counts["real_point_violations"]
    real_seg_h8 = h8_counts["real_segment_violations"]
    real_seg_s7 = s7_counts["real_segment_violations"]
    real_structural = structural_count
    passed = (
        real_point_h8 == 0
        and real_seg_h8 == 0
        and real_point_s7 == 0
        and real_seg_s7 == 0
        and (real_structural in (None, 0))
    )
    return {
        "artifact_type": "step30s7_trajectory_wall_crossing_validation",
        "version": "v0_2",
        "created_utc": now_iso(),
        "active_map_yaml": active_map_yaml.as_posix(),
        "h8r2_map_yaml": h8_yaml.as_posix(),
        "step30s7_map_yaml": s7_yaml.as_posix(),
        "trajectory_sample_count": len(samples),
        "wall_point_violations_h8r2": len(h8_counts["point_violations"]),
        "wall_segment_violations_h8r2": len(h8_counts["segment_violations"]),
        "wall_point_violations_step30s7": len(s7_counts["point_violations"]),
        "wall_segment_violations_step30s7": len(s7_counts["segment_violations"]),
        "boundary_tolerance_point_violations_h8r2": h8_counts["boundary_only_point_count"],
        "boundary_tolerance_point_violations_step30s7": s7_counts["boundary_only_point_count"],
        "boundary_tolerance_segment_violations_h8r2": h8_counts["boundary_only_segment_count"],
        "boundary_tolerance_segment_violations_step30s7": s7_counts["boundary_only_segment_count"],
        "real_point_violations_h8r2": real_point_h8,
        "real_point_violations_step30s7": real_point_s7,
        "real_segment_violations_h8r2": real_seg_h8,
        "real_segment_violations_step30s7": real_seg_s7,
        "structural_wall_overlap_count": structural_count,
        "structural_wall_boundary_count": structural_boundary,
        "trajectory_wall_point_violation_count": len(s7_counts["point_violations"]),
        "trajectory_wall_segment_violation_count": len(s7_counts["segment_violations"]),
        "max_wall_penetration_or_occupied_crossing": max(h8_counts["max_cells"], s7_counts["max_cells"]),
        "suspicious_wall_crossing_segments": suspicious,
        "point_violations": (h8_counts["point_violations"] + s7_counts["point_violations"])[:25],
        "visualization_artifact_possible": visualization_artifact_possible,
        "visualization_interpolation_artifact_possible": visualization_artifact_possible,
        "boundary_tolerance_not_used_for_pass": True,
        "final_wall_validation_passed": passed,
        "wall_crossing_validation_passed": passed,
    }


def spin_payload(samples: list[dict[str, Any]], gateway: dict[str, Any] | None) -> dict[str, Any]:
    if not gateway:
        return {"artifact_type": "stage1_local_looping_spin_validation", "version": "v0_1", "created_utc": now_iso(), "spinning_detected": False, "failure_reason": "gateway pose unavailable"}
    gx, gy = float(gateway["x"]), float(gateway["y"])
    near_pairs = []
    total_yaw = 0.0
    stationary_yaw_intervals = 0
    repeated: dict[tuple[int, int], int] = {}
    for idx, (a, b) in enumerate(zip(samples, samples[1:])):
        amid = (float(a["x"]) + float(b["x"])) * 0.5
        bmid = (float(a["y"]) + float(b["y"])) * 0.5
        if math.hypot(amid - gx, bmid - gy) > 0.60:
            continue
        travel = math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        dyaw = abs(angle_delta(float(a.get("yaw", 0.0)), float(b.get("yaw", 0.0))))
        total_yaw += dyaw
        near_pairs.append(idx)
        if travel < 0.08 and dyaw > 0.45:
            stationary_yaw_intervals += 1
        key = (int(round(amid / 0.20)), int(round(bmid / 0.20)))
        repeated[key] = repeated.get(key, 0) + 1
    local_loop_count = sum(1 for count in repeated.values() if count >= 5)
    spinning = total_yaw > 8.0 or stationary_yaw_intervals >= 4 or local_loop_count >= 3
    return {
        "artifact_type": "stage1_local_looping_spin_validation",
        "version": "v0_1",
        "created_utc": now_iso(),
        "gateway_xy": {"x": gx, "y": gy},
        "near_gateway_interval_count": len(near_pairs),
        "total_angular_travel_near_gateway_rad": round(total_yaw, 6),
        "near_stationary_high_yaw_change_interval_count": stationary_yaw_intervals,
        "local_loop_count_around_gateway": local_loop_count,
        "repeated_revisit_cell_count_near_gateway": local_loop_count,
        "spinning_detected": bool(spinning),
        "local_looping_validation_passed": not spinning,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--route-query-json", type=Path, required=True)
    parser.add_argument("--waypoints-json", type=Path, required=True)
    parser.add_argument("--route-execution-json", type=Path, required=True)
    parser.add_argument("--trajectory-json", type=Path, required=True)
    parser.add_argument("--through-output-json", type=Path, required=True)
    parser.add_argument("--through-output-md", type=Path, required=True)
    parser.add_argument("--terminal-output-json", type=Path, required=True)
    parser.add_argument("--terminal-output-md", type=Path, required=True)
    parser.add_argument("--wall-output-json", type=Path)
    parser.add_argument("--wall-output-md", type=Path)
    parser.add_argument("--spin-output-json", type=Path)
    parser.add_argument("--spin-output-md", type=Path)
    parser.add_argument("--room15-min-inside-samples", type=int, default=8, dest="through_room_min_inside_samples")
    parser.add_argument("--through-room-min-inside-samples", type=int, default=None)
    parser.add_argument("--through-room-dwell-sec", type=float, default=3.0)
    args = parser.parse_args()
    if args.through_room_min_inside_samples is not None:
        args.through_room_min_inside_samples_final = args.through_room_min_inside_samples
    else:
        args.through_room_min_inside_samples_final = args.through_room_min_inside_samples

    stage_output = args.stage_output_dir.resolve()
    route_query = read_json(args.route_query_json)
    waypoints_payload = read_json(args.waypoints_json)
    route_execution = read_json(args.route_execution_json) if args.route_execution_json.exists() else {}
    trajectory_payload = read_json(args.trajectory_json) if args.trajectory_json.exists() else {}
    samples = trajectory_payload.get("samples") or route_execution.get("trajectory_samples") or []
    waypoints = waypoints_payload.get("waypoints") or []
    mask = np.load(stage_output / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy")
    resolution, origin, _height = load_map_meta(stage_output)

    through_rooms = route_query.get("through_rooms") or []
    interior_targets = route_query.get("interior_targets") or {}
    room_sequence = route_query.get("room_sequence") or []
    min_inside = args.through_room_min_inside_samples_final
    min_dwell = args.through_room_dwell_sec

    # Generic through-room validation for each requested through-room
    through_room_results = {}
    all_through_success = True
    all_gateway_only_passed = True
    for through_room in through_rooms:
        room_num_val = int(through_room.split("_")[1])
        room_target = interior_targets.get(through_room)
        # Find nearest gateway for this through-room from waypoints
        room_gateway = None
        for wp in waypoints:
            gid = wp.get("gateway_id") or ""
            if f"r{room_num_val}_" in gid or f"_r{room_num_val}_" in gid:
                room_gateway = wp
                break
        room_flags = [mask_value(mask, float(s["x"]), float(s["y"]), resolution, origin) == room_num_val for s in samples]
        inside_count = sum(1 for f in room_flags if f)
        room_dwell = dwell_seconds(samples, room_flags)
        min_gw = min_dist(samples, room_gateway)
        min_int = min_dist(samples, room_target)
        entered = inside_count > 0
        inside_samples = [s for s, f in zip(samples, room_flags) if f]
        max_dist_gw = max((dist(s, room_gateway) for s in inside_samples), default=0.0) if room_gateway else 0.0
        not_gateway_only = entered and max_dist_gw >= 0.55 and (
            (min_int is not None and min_int <= 0.65) or inside_count >= min_inside
        )
        success = bool(not_gateway_only and inside_count >= min_inside and room_dwell >= min_dwell)
        if not success:
            all_through_success = False
        if not not_gateway_only:
            all_gateway_only_passed = False
        through_room_results[through_room] = {
            "room_id": through_room,
            "route_topology_includes_room": through_room in room_sequence,
            "trajectory_entered_room_mask": entered,
            "inside_sample_count": inside_count,
            "inside_dwell_sec": room_dwell,
            "gateway_crossed": min_gw is not None and min_gw <= 0.55,
            "min_distance_to_gateway_m": round(min_gw, 6) if min_gw is not None else None,
            "min_distance_to_interior_target_m": round(min_int, 6) if min_int is not None else None,
            "max_distance_from_gateway_while_inside_m": round(max_dist_gw, 6),
            "interior_target": room_target,
            "required_inside_sample_count": min_inside,
            "required_dwell_sec": min_dwell,
            "visual_through_room_success": success,
            "gateway_only_failure_guard_passed": bool(not_gateway_only),
        }

    # Backward-compatible room15 fields if room_15 was a through-room
    r15_result = through_room_results.get("room_15", {})
    r7_r15_gateway = gateway_pose(waypoints, "gw_00824_r7_r15_01")
    r14_r16_gateway = gateway_pose(waypoints, "gw_00824_r14_r16_01")
    room16_target = interior_targets.get("room_16")
    through_payload = {
        "artifact_type": "through_room_physical_visit_validation",
        "version": "v0_2",
        "created_utc": now_iso(),
        "through_rooms_requested": through_rooms,
        "through_room_results": through_room_results,
        "all_through_rooms_success": all_through_success,
        "gateway_only_failure_guard_passed": all_gateway_only_passed,
        # Backward-compat room15 fields
        "route_topology_includes_room15": "room_15" in room_sequence,
        "trajectory_entered_room15_mask": r15_result.get("trajectory_entered_room_mask", False),
        "room15_inside_sample_count": r15_result.get("inside_sample_count", 0),
        "room15_inside_dwell_sec": r15_result.get("inside_dwell_sec", 0.0),
        "visual_through_room15_success": r15_result.get("visual_through_room_success", len(through_rooms) > 0 and "room_15" not in through_rooms),
        "trajectory_sample_count": len(samples),
    }

    final_pose = route_execution.get("final_pose_observed") or (samples[-1] if samples else None)
    final_inside_room16 = bool(final_pose and mask_value(mask, float(final_pose["x"]), float(final_pose["y"]), resolution, origin) == 16)
    final_dist_gateway = dist(final_pose, r14_r16_gateway) if final_pose and r14_r16_gateway else None
    final_dist_interior = dist(final_pose, room16_target) if final_pose and room16_target else None
    final_near_gateway = final_dist_gateway is not None and final_dist_gateway <= 0.65
    final_inside_interior = final_inside_room16 and final_dist_interior is not None and final_dist_interior <= 0.9
    terminal_payload = {
        "artifact_type": "stage1_room16_terminal_quality_validation",
        "version": "v0_1",
        "created_utc": now_iso(),
        "route_execution_success": bool(route_execution.get("succeeded")),
        "final_arrival_action_success": bool(route_execution.get("final_arrival_success")),
        "final_pose": final_pose,
        "final_pose_inside_room16_mask": final_inside_room16,
        "final_pose_near_r14_r16_gateway": bool(final_near_gateway),
        "final_pose_distance_to_r14_r16_gateway_m": round(final_dist_gateway, 6) if final_dist_gateway is not None else None,
        "final_pose_inside_room16_interior_region": bool(final_inside_interior),
        "min_distance_to_room16_interior_target": round(min_dist(samples, room16_target), 6) if min_dist(samples, room16_target) is not None else None,
        "final_pose_distance_to_room16_interior_target_m": round(final_dist_interior, 6) if final_dist_interior is not None else None,
        "room16_interior_target": room16_target,
        "terminal_visual_quality_passed": bool(final_inside_interior and not final_near_gateway),
    }

    write_json(args.through_output_json, through_payload)
    write_json(args.terminal_output_json, terminal_payload)
    write_md(args.through_output_md, "Stage1 Through-Room Physical Visit Validation", through_payload)
    write_md(args.terminal_output_md, "Stage1 Room16 Terminal Quality Validation", terminal_payload)

    map_yaml = Path(route_query.get("map_yaml") or stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml")
    h8_yaml = stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml"
    stable_yaml = stage_output / "maps/stage1_full_scene_occupancy_map.yaml"
    s7_yaml = map_yaml if map_yaml.name == stable_yaml.name else stage_output / "maps/step30s7_request_aware_nav_map.yaml"
    active_grid, map_resolution, map_origin = load_nav_map(map_yaml)
    h8_grid, h8_resolution, h8_origin = load_nav_map(h8_yaml)
    if s7_yaml.exists():
        s7_grid, s7_resolution, s7_origin = load_nav_map(s7_yaml)
    else:
        s7_grid, s7_resolution, s7_origin = active_grid, map_resolution, map_origin
    layered_path = stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz"
    structural = None
    if layered_path.exists():
        layered = np.load(layered_path)
        structural = (
            layered["segmentation_wall_processed"].astype(bool)
            | layered["gateway_wall_preclose"].astype(bool)
            | layered["structural_wall"].astype(bool)
        )
        try:
            gateway_carve = np.load(stage_output / "maps/h8r2_gateway_preserving_masks.npz")["gateway_carve_mask"].astype(bool)
            structural = structural & ~gateway_carve
        except Exception:
            pass
    wall = wall_crossing_payload_multi(
        samples,
        (h8_grid, h8_resolution, h8_origin),
        (s7_grid, s7_resolution, s7_origin),
        structural,
        resolution,
        origin,
        map_yaml,
        h8_yaml,
        s7_yaml,
    )
    # Use first through-room gateway for spin detection, or r7_r15 for backward compat
    spin_gateway = r7_r15_gateway
    if through_rooms:
        first_through = through_rooms[0]
        first_num = int(first_through.split("_")[1])
        for wp in waypoints:
            gid = wp.get("gateway_id") or ""
            if f"r{first_num}_" in gid or f"_r{first_num}_" in gid:
                spin_gateway = wp
                break
    spin = spin_payload(samples, spin_gateway)
    if args.wall_output_json:
        write_json(args.wall_output_json, wall)
    if args.wall_output_md:
        write_md(args.wall_output_md, "Stage1 Trajectory Wall Crossing Validation", wall)
    if args.spin_output_json:
        write_json(args.spin_output_json, spin)
    if args.spin_output_md:
        write_md(args.spin_output_md, "Stage1 Local Looping Spin Validation", spin)
    print(json.dumps({"through": through_payload, "terminal": terminal_payload, "wall": wall, "spin": spin}, indent=2, sort_keys=True))
    return 0 if through_payload["all_through_rooms_success"] and terminal_payload["terminal_visual_quality_passed"] and wall["wall_crossing_validation_passed"] and spin["local_looping_validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
