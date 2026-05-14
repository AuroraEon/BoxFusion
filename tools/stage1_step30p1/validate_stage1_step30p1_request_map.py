#!/usr/bin/env python3
"""Validate Step30S7 request-aware map coverage and gateway connectivity."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import (
    astar,
    derive_interior_target,
    load_config,
    load_gateway_lookup,
    load_nav_map,
    nearest_free_index,
    now_iso,
    pair_key,
    read_json,
    required_room_reasons,
    room_num,
    shortest_room_route,
    write_json,
)


def component_stats(room_mask: np.ndarray, free: np.ndarray) -> tuple[int, int, np.ndarray]:
    count, labels = cv2.connectedComponents((room_mask & free).astype(np.uint8), 8)
    sizes = [int((labels == idx).sum()) for idx in range(1, count)]
    return max(0, count - 1), max(sizes) if sizes else 0, labels


def gateway_points(gateway: dict[str, Any], from_room: str, to_room: str) -> list[dict[str, Any]]:
    room_a = f"room_{gateway['room_a']}"
    a_is_from = room_a == from_room
    return [
        gateway["representative_approach_from_room_a" if a_is_from else "representative_approach_from_room_b"],
        gateway["representative_crossing_pose"],
        gateway["representative_approach_from_room_b" if a_is_from else "representative_approach_from_room_a"],
    ]


def route_gateway_points(route: list[str], gateways: dict[str, dict[str, Any]], config: dict[str, Any]) -> tuple[list[str], list[dict[str, Any]], dict[str, list[dict[str, Any]]]]:
    selected_pairs = config.get("selected_gateway_pairs") or {}
    ids = []
    points = []
    by_room: dict[str, list[dict[str, Any]]] = {}
    for a, b in zip(route, route[1:]):
        gid = selected_pairs[pair_key(a, b)]
        ids.append(gid)
        pts = gateway_points(gateways[gid], a, b)
        points.extend(pts)
        by_room.setdefault(a, []).extend(pts)
        by_room.setdefault(b, []).extend(pts)
    return ids, points, by_room


def validate_room(
    stage_output: Path,
    room: str,
    reasons: list[str],
    h8_grid: np.ndarray,
    s7_grid: np.ndarray,
    room_mask: np.ndarray,
    wall_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    gateway_refs: list[dict[str, Any]],
) -> dict[str, Any]:
    rid = room_num(room)
    mask = room_mask == rid
    h8_free = h8_grid >= 250
    s7_free = s7_grid >= 250
    comp_count, largest, labels = component_stats(mask, s7_free)
    distance_to_nonfree = cv2.distanceTransform((s7_free).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    target = None
    target_feasible = False
    target_connected = False
    target_gateway_distance = None
    target_wall_clearance = None
    try:
        target = derive_interior_target(stage_output, room, s7_grid, resolution, origin, gateway_refs)
        target_rc = nearest_free_index(s7_grid, float(target["x"]), float(target["y"]), resolution, origin)
        target_label = int(labels[target_rc])
        target_feasible = target_label > 0
        target_wall_clearance = float(distance_to_nonfree[target_rc])
        if gateway_refs:
            target_gateway_distance = min(math.hypot(float(target["x"]) - float(g["x"]), float(target["y"]) - float(g["y"])) for g in gateway_refs)
            for gateway in gateway_refs:
                try:
                    gateway_rc = nearest_free_index(s7_grid, float(gateway["x"]), float(gateway["y"]), resolution, origin)
                    if target_label > 0 and int(labels[gateway_rc]) == target_label:
                        target_connected = True
                        break
                    astar(s7_grid, gateway_rc, target_rc)
                    target_connected = True
                    break
                except Exception:
                    continue
        else:
            target_connected = target_feasible
    except Exception as exc:
        target = {"error": f"{type(exc).__name__}: {exc}"}
    semantic_area = int(mask.sum())
    s7_free_cells = int((mask & s7_free).sum())
    h8_free_cells = int((mask & h8_free).sum())
    coverage = s7_free_cells / max(1, semantic_area)
    h8_coverage = h8_free_cells / max(1, semantic_area)
    room15_extra = {}
    if room == "room_15":
        room15_extra = {
            "room15_free_space_greatly_improved_over_0p067221": coverage >= 0.70 and coverage > h8_coverage + 0.50,
            "room15_target_not_gateway_adjacent": target_gateway_distance is not None and target_gateway_distance >= 0.45,
            "room15_target_distance_from_r7_r15_gateway_ge_0p8m_if_feasible": target_gateway_distance is not None and target_gateway_distance >= 0.80,
            "room15_target_wall_clearance_ge_0p35m_if_feasible": target_wall_clearance is not None and target_wall_clearance >= 0.35,
        }
    new_wall_conflict_cells = int((mask & wall_mask & s7_free & ~h8_free).sum())
    passed = (
        semantic_area > 0
        and coverage >= 0.70
        and largest >= max(25, int(0.50 * s7_free_cells))
        and target_feasible
        and target_connected
        and new_wall_conflict_cells == 0
    )
    if room == "room_15":
        passed = passed and all(room15_extra.values())
    return {
        "room": room,
        "included_because": reasons,
        "semantic_mask_area_cells": semantic_area,
        "original_h8r2_free_space_overlap_fraction": round(h8_coverage, 6),
        "step30s7_free_space_overlap_fraction": round(coverage, 6),
        "original_h8r2_free_cells": h8_free_cells,
        "step30s7_free_cells": s7_free_cells,
        "largest_free_component_area_cells": largest,
        "connected_component_count": comp_count,
        "gateway_to_interior_connectivity": bool(target_connected),
        "interior_target_feasible": bool(target_feasible),
        "interior_target": target,
        "target_distance_from_nearest_gateway_m": round(target_gateway_distance, 6) if target_gateway_distance is not None else None,
        "target_wall_clearance_m": round(target_wall_clearance, 6) if target_wall_clearance is not None else None,
        "wall_clearance_conflict_cells": new_wall_conflict_cells,
        "preexisting_wall_label_free_overlap_cells": int((mask & wall_mask & s7_free & h8_free).sum()),
        "unknown_or_non_free_overlap_cells": int((mask & ~s7_free).sum()),
        **room15_extra,
        "validation_passed": bool(passed),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--goal-room", required=True)
    parser.add_argument("--through-rooms", nargs="*", default=[])
    parser.add_argument("--terminal-room", default=None)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    terminal = args.terminal_room or args.goal_room
    config = load_config(stage_output)
    route = shortest_room_route(config, args.start_room, args.goal_room, args.through_rooms)
    reasons = required_room_reasons(args.start_room, args.goal_room, args.through_rooms, terminal, route)
    h8_grid, resolution, origin, _ = load_nav_map(stage_output / config["h8r2_map_yaml"])
    s7_grid, s7_resolution, s7_origin, _ = load_nav_map((args.map_yaml or stage_output / "maps/step30s7_request_aware_nav_map.yaml").resolve())
    if resolution != s7_resolution or origin != s7_origin or h8_grid.shape != s7_grid.shape:
        raise RuntimeError("H8R2 and Step30S7 map geometry do not match")
    layered = np.load(stage_output / config["layered_bev_path"])
    room_mask = np.load(stage_output / config["room_mask_path"])
    wall_mask = layered["structural_wall"].astype(bool) | layered["segmentation_wall_processed"].astype(bool) | layered["gateway_wall_preclose"].astype(bool)
    gateways = load_gateway_lookup(stage_output, config)
    gateway_ids, all_gateway_points, by_room = route_gateway_points(route, gateways, config)
    rooms = [room for room in sorted(reasons, key=room_num)]
    room_results = {room: validate_room(stage_output, room, reasons[room], h8_grid, s7_grid, room_mask, wall_mask, resolution, origin, by_room.get(room, all_gateway_points)) for room in rooms}
    forbidden = {
        key: {
            "selected_by_config": key in (config.get("selected_gateway_pairs") or {}),
            "passed": key not in (config.get("selected_gateway_pairs") or {}),
        }
        for key in (config.get("forbidden_gateway_pairs") or [])
    }
    passed = all(item["validation_passed"] for item in room_results.values()) and all(item["passed"] for item in forbidden.values())
    payload = {
        "artifact_type": "step30s7_request_map_validation",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "map_yaml": (args.map_yaml or stage_output / "maps/step30s7_request_aware_nav_map.yaml").as_posix(),
        "request": {"start_room": args.start_room, "goal_room": args.goal_room, "through_rooms": args.through_rooms, "terminal_room": terminal},
        "resolved_route": route,
        "selected_gateway_ids": gateway_ids,
        "room_results": room_results,
        "forbidden_shortcut_checks": forbidden,
        "validation_passed": passed,
    }
    write_json(args.output_json, payload)
    manifest_path = stage_output / "maps/step30s7_request_aware_nav_map_manifest_v0_1.json"
    if manifest_path.exists():
        try:
            manifest = read_json(manifest_path)
            manifest["gateway_to_interior_connectivity_by_requested_room"] = {
                room: item["gateway_to_interior_connectivity"] for room, item in room_results.items()
            }
            manifest["validation_passed"] = passed
            manifest["eligible_for_active_use"] = passed
            write_json(manifest_path, manifest)
        except Exception:
            pass
    lines = ["# Step30S7 Request Map Validation", "", f"Validation passed: `{passed}`", f"Route: `{' -> '.join(route)}`", ""]
    for room, item in room_results.items():
        lines.append(f"- `{room}`: S7 free `{item['step30s7_free_space_overlap_fraction']}`, H8R2 free `{item['original_h8r2_free_space_overlap_fraction']}`, connected `{item['gateway_to_interior_connectivity']}`, passed `{item['validation_passed']}`")
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
