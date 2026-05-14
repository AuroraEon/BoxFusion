#!/usr/bin/env python3
"""Prepare a topology-valid semantic route with physical through-room targets."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import (
    OLD_STEP30S5_PROFILES,
    astar,
    derive_interior_target,
    load_config,
    load_gateway_lookup,
    load_nav_map,
    nearest_free_index,
    now_iso,
    pair_key,
    rc_to_xy,
    required_room_reasons,
    room_num,
    shortest_room_route,
    simplify_path,
    write_json,
)


def yaw_between(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.atan2(float(b["y"]) - float(a["y"]), float(b["x"]) - float(a["x"]))


def segment_points(gateway: dict[str, Any], from_room: str, to_room: str, gateway_id: str, segment_index: int, route_id: str) -> list[dict[str, Any]]:
    room_a = f"room_{gateway['room_a']}"
    a_is_from = room_a == from_room
    approach_from = gateway["representative_approach_from_room_a" if a_is_from else "representative_approach_from_room_b"]
    approach_to = gateway["representative_approach_from_room_b" if a_is_from else "representative_approach_from_room_a"]
    crossing = gateway["representative_crossing_pose"]
    pk = pair_key(from_room, to_room)
    rows = []
    for source, pose in [("segment_start", approach_from), ("segment_crossing", crossing), ("segment_goal", approach_to)]:
        rows.append({
            "route_id": route_id,
            "waypoint_index": -1,
            "x": round(float(pose["x"]), 4),
            "y": round(float(pose["y"]), 4),
            "yaw": round(float(pose.get("yaw", crossing.get("yaw", 0.0))), 6),
            "source": source,
            "from_room": from_room,
            "to_room": to_room,
            "gateway_id": gateway_id,
            "pair_key": pk,
            "segment_index": segment_index,
            "reason": "selected_gateway_hypothesis_representative_pose",
        })
    return rows


def append_bridge(waypoints: list[dict[str, Any]], target: dict[str, Any], grid, resolution: float, origin: tuple[float, float], route_id: str) -> None:
    if not waypoints:
        target = dict(target)
        target["waypoint_index"] = 0
        waypoints.append(target)
        return
    start = waypoints[-1]
    path = simplify_path(
        astar(
            grid,
            nearest_free_index(grid, float(start["x"]), float(start["y"]), resolution, origin),
            nearest_free_index(grid, float(target["x"]), float(target["y"]), resolution, origin),
        ),
        resolution,
        spacing_m=0.25,
    )
    for row, col in path[1:-1]:
        x, y = rc_to_xy(row, col, resolution, origin)
        waypoints.append({
            "route_id": route_id,
            "waypoint_index": len(waypoints),
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": 0.0,
            "source": "bridge",
            "from_room": start.get("to_room") or start.get("from_room"),
            "to_room": target.get("from_room") or target.get("to_room"),
            "gateway_id": None,
            "pair_key": None,
            "reason": {"type": "shortest_path_bridge_on_selected_nav_map"},
        })
    target = dict(target)
    target["waypoint_index"] = len(waypoints)
    waypoints.append(target)


def _xy_to_rc(x: float, y: float, resolution: float, origin: tuple[float, float]) -> tuple[int, int]:
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def _rc_to_xy(row: int, col: int, resolution: float, origin: tuple[float, float]) -> tuple[float, float]:
    return origin[0] + col * resolution, origin[1] + row * resolution


def _load_structural_mask(stage_output: Path) -> np.ndarray | None:
    layered_path = stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz"
    if not layered_path.exists():
        return None
    layered = np.load(layered_path)
    return (
        layered["segmentation_wall_processed"].astype(bool)
        | layered["gateway_wall_preclose"].astype(bool)
        | layered["structural_wall"].astype(bool)
    )


def _clearance_adjust_pose(
    pose: dict[str, Any],
    grid: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    room_mask: np.ndarray,
    structural_mask: np.ndarray | None,
    room_constraint: int | None,
    max_radius_m: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Snap a semantic waypoint to the best nearby free cell with better clearance."""
    free = (grid >= 250).astype(np.uint8)
    nav_dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * resolution
    if structural_mask is not None:
        struct_free = (~structural_mask.astype(bool)).astype(np.uint8)
        structural_dist = cv2.distanceTransform(struct_free, cv2.DIST_L2, 5) * resolution
    else:
        structural_dist = nav_dist

    start_row, start_col = _xy_to_rc(float(pose["x"]), float(pose["y"]), resolution, origin)
    radius_cells = max(1, int(round(max_radius_m / resolution)))
    best: tuple[float, int, int, float, float, float] | None = None
    for row in range(start_row - radius_cells, start_row + radius_cells + 1):
        for col in range(start_col - radius_cells, start_col + radius_cells + 1):
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                continue
            if int(grid[row, col]) < 250:
                continue
            if room_constraint is not None and int(room_mask[row, col]) != room_constraint:
                continue
            wx, wy = _rc_to_xy(row, col, resolution, origin)
            move = math.hypot(wx - float(pose["x"]), wy - float(pose["y"]))
            if move > max_radius_m:
                continue
            nav_clear = float(nav_dist[row, col])
            struct_clear = float(structural_dist[row, col])
            score = 2.5 * nav_clear + 1.0 * struct_clear - 0.9 * move
            if best is None or score > best[0]:
                best = (score, row, col, move, nav_clear, struct_clear)
    if best is None:
        return dict(pose), {
            "adjusted": False,
            "reason": "no_better_nearby_free_cell_found",
            "original_xy": {"x": pose["x"], "y": pose["y"]},
            "room_constraint": room_constraint,
        }
    _score, row, col, move, nav_clear, struct_clear = best
    x, y = _rc_to_xy(row, col, resolution, origin)
    adjusted = dict(pose)
    adjusted["x"] = round(x, 4)
    adjusted["y"] = round(y, 4)
    return adjusted, {
        "adjusted": move > resolution * 0.5,
        "original_xy": {"x": pose["x"], "y": pose["y"]},
        "adjusted_xy": {"x": adjusted["x"], "y": adjusted["y"]},
        "move_distance_m": round(move, 6),
        "nav_map_clearance_m": round(nav_clear, 6),
        "structural_wall_clearance_m": round(struct_clear, 6),
        "room_constraint": room_constraint,
        "method": "nearby_free_cell_maximizing_nav_and_structural_clearance",
    }


def clearance_adjust_segments(
    stage_output: Path,
    raw_segments: list[list[dict[str, Any]]],
    grid: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
) -> list[dict[str, Any]]:
    room_mask = np.load(stage_output / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy")
    structural_mask = _load_structural_mask(stage_output)
    adjustments: list[dict[str, Any]] = []
    for segment in raw_segments:
        for waypoint in segment:
            source = waypoint.get("source")
            if source == "segment_start":
                constraint = room_num(waypoint["from_room"])
                radius = 0.55
            elif source == "segment_goal":
                constraint = room_num(waypoint["to_room"])
                radius = 0.55
            elif source == "segment_crossing":
                constraint = None
                radius = 0.35
            else:
                continue
            adjusted, meta = _clearance_adjust_pose(waypoint, grid, resolution, origin, room_mask, structural_mask, constraint, radius)
            meta.update({
                "waypoint_source": source,
                "gateway_id": waypoint.get("gateway_id"),
                "from_room": waypoint.get("from_room"),
                "to_room": waypoint.get("to_room"),
            })
            waypoint.update(adjusted)
            waypoint["clearance_adjustment"] = meta
            adjustments.append(meta)
    return adjustments


def resolve_map(stage_output: Path, profile: str, map_yaml: Path | None) -> tuple[str, Path]:
    if profile in OLD_STEP30S5_PROFILES:
        raise ValueError(f"map profile {profile} has been removed from active Step30S7 runtime use")
    if map_yaml:
        return "custom", map_yaml.resolve()
    if profile in {"auto", "stable", "full_scene", "stage1_full_scene", "stage1_full_scene_occupancy"}:
        stable = stage_output / "maps/stage1_full_scene_occupancy_map.yaml"
        return "stage1_full_scene_occupancy", stable
    if profile == "h8r2":
        return "h8r2", stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml"
    if profile in {"request_aware", "step30s7_request_aware"}:
        s7 = stage_output / "maps/step30s7_request_aware_nav_map.yaml"
        return "step30s7_request_aware_debug", s7
    if profile == "reference_h8r2":
        return "h8r2", stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml"
    raise ValueError(f"unsupported map profile: {profile}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--goal-room", required=True)
    parser.add_argument("--through-rooms", nargs="*", default=[])
    parser.add_argument("--terminal-room", default=None)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--map-profile", default="auto")
    parser.add_argument("--room15-min-wall-distance-m", type=float, default=0.35)
    parser.add_argument("--room15-preferred-gateway-distance-m", type=float, default=0.80)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--waypoints-output-json", type=Path, required=True)
    parser.add_argument("--target-selection-output-json", type=Path)
    parser.add_argument("--target-selection-output-md", type=Path)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    terminal_room = args.terminal_room or args.goal_room
    config = load_config(stage_output)
    active_profile, map_yaml = resolve_map(stage_output, args.map_profile, args.map_yaml)
    grid, resolution, origin, _ = load_nav_map(map_yaml)
    bridge_grid = grid.copy()
    structural_for_bridge = _load_structural_mask(stage_output)
    if structural_for_bridge is not None:
        try:
            masks = np.load(stage_output / config["h8r2_masks_npz"])
            gateway_carve = masks["gateway_carve_mask"].astype(bool)
        except Exception:
            gateway_carve = np.zeros_like(structural_for_bridge, dtype=bool)
        bridge_grid[structural_for_bridge & ~gateway_carve] = 0
    gateways = load_gateway_lookup(stage_output, config)
    selected_pairs = config.get("selected_gateway_pairs") or {}
    room_sequence = shortest_room_route(config, args.start_room, args.goal_room, args.through_rooms)
    route_id = "room_chain_" + "_".join(f"r{room_num(room)}" for room in room_sequence)
    gateway_sequence = []
    raw_segments = []
    for idx, (src, dst) in enumerate(zip(room_sequence, room_sequence[1:])):
        gateway_id = selected_pairs[pair_key(src, dst)]
        gateway_sequence.append(gateway_id)
        raw_segments.append(segment_points(gateways[gateway_id], src, dst, gateway_id, idx, route_id))
    clearance_adjustments = clearance_adjust_segments(stage_output, raw_segments, grid, resolution, origin)

    gateway_targets = [{"x": p["x"], "y": p["y"]} for seg in raw_segments for p in seg]
    rooms_needing_targets = sorted(set(args.through_rooms + [terminal_room]), key=room_num)
    interior_targets = {
        room: derive_interior_target(stage_output, room, grid, resolution, origin, gateway_targets, args.room15_min_wall_distance_m, args.room15_preferred_gateway_distance_m)
        for room in rooms_needing_targets
    }

    waypoints: list[dict[str, Any]] = []
    revisit_counts: dict[str, int] = {}
    for seg_idx, segment in enumerate(raw_segments):
        for room in (segment[0]["from_room"], segment[-1]["to_room"]):
            revisit_counts[room] = revisit_counts.get(room, 0) + 1
        for point in segment:
            if not waypoints:
                point["waypoint_index"] = len(waypoints)
                waypoints.append(point)
            else:
                append_bridge(waypoints, point, bridge_grid, resolution, origin, route_id)
        arrived_room = segment[-1]["to_room"]
        next_room = room_sequence[seg_idx + 2] if seg_idx + 2 < len(room_sequence) else None
        if arrived_room in args.through_rooms and next_room and next_room != arrived_room:
            target = dict(interior_targets[arrived_room])
            target.update({"route_id": route_id, "source": f"{arrived_room.replace('_', '')}_interior_terminal", "from_room": arrived_room, "to_room": arrived_room, "gateway_id": None, "pair_key": None, "reason": "strict_through_room_physical_visit_target"})
            append_bridge(waypoints, target, bridge_grid, resolution, origin, route_id)

    if terminal_room in interior_targets:
        terminal = dict(interior_targets[terminal_room])
        terminal.update({"route_id": route_id, "source": f"{terminal_room.replace('_', '')}_interior_terminal", "from_room": terminal_room, "to_room": terminal_room, "gateway_id": None, "pair_key": None, "reason": "strict_terminal_visual_quality_target"})
        append_bridge(waypoints, terminal, bridge_grid, resolution, origin, route_id)

    for idx, waypoint in enumerate(waypoints[:-1]):
        waypoint["yaw"] = round(yaw_between(waypoint, waypoints[idx + 1]), 6)
    if waypoints:
        waypoints[-1]["yaw"] = waypoints[-2]["yaw"] if len(waypoints) > 1 else 0.0

    reasons = required_room_reasons(args.start_room, args.goal_room, args.through_rooms, terminal_room, room_sequence)
    payload = {
        "artifact_type": "step30s7_route_query_result",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "map_yaml": map_yaml.as_posix(),
        "map_profile": active_profile,
        "start_room": args.start_room,
        "goal_room": args.goal_room,
        "through_rooms": args.through_rooms,
        "terminal_room": terminal_room,
        "room_sequence": room_sequence,
        "included_room_reasons": reasons,
        "revisited_rooms": sorted([room for room, count in revisit_counts.items() if count > 1], key=room_num),
        "gateway_sequence": gateway_sequence,
        "forbidden_shortcuts_not_used": config.get("forbidden_gateway_pairs") or [],
        "interior_targets": interior_targets,
        "waypoints_path": args.waypoints_output_json.as_posix(),
        "waypoint_count": len(waypoints),
        "route_id": route_id,
        "not_room15_specific": True,
        "gateway_waypoint_clearance_adjustments": clearance_adjustments,
        "bridge_planner_avoids_structural_walls_except_selected_gateway_carves": True,
    }
    waypoint_payload = {
        "artifact_type": "step30s7_semantic_route_waypoints",
        "version": "v0_1",
        "created_utc": now_iso(),
        "route_id": route_id,
        "room_sequence": room_sequence,
        "gateway_sequence": gateway_sequence,
        "source": "externalized_gateway_config_and_selected_stable_full_scene_nav_map",
        "map_yaml": map_yaml.as_posix(),
        "interior_targets": interior_targets,
        "gateway_waypoint_clearance_adjustments": clearance_adjustments,
        "bridge_planner_avoids_structural_walls_except_selected_gateway_carves": True,
        "waypoints": waypoints,
    }
    write_json(args.waypoints_output_json, waypoint_payload)
    write_json(args.output_json, payload)
    args.output_md.write_text(
        "\n".join([
            "# Step30S7 Route Query Result",
            "",
            f"Route: `{' -> '.join(room_sequence)}`",
            f"Gateways: `{' -> '.join(gateway_sequence)}`",
            f"Waypoints: `{len(waypoints)}`",
            f"Map profile: `{active_profile}`",
            f"Map: `{map_yaml}`",
        ]) + "\n",
        encoding="utf-8",
    )
    if args.target_selection_output_json:
        write_json(args.target_selection_output_json, {"artifact_type": "step30s7_target_selection", "version": "v0_1", "created_utc": now_iso(), "map_yaml": map_yaml.as_posix(), "interior_targets": interior_targets})
    if args.target_selection_output_md:
        args.target_selection_output_md.write_text("# Step30S7 Target Selection\n\n" + "\n".join(f"- `{room}`: `({target['x']}, {target['y']})`" for room, target in interior_targets.items()) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
