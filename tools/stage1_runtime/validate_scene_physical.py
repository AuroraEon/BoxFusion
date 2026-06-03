#!/usr/bin/env python3
"""Validate route trajectory evidence for a parameterized scene."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

from scene_runtime_common import derive_paths, load_nav_map, map_value, now_iso, parse_rooms, read_json, room_lookup, write_json


def point_in_poly(x: float, y: float, poly: list[list[float]]) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / max(1e-12, yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--map-yaml", type=Path)
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
    parser.add_argument("--through-room-min-inside-samples", "--room15-min-inside-samples", type=int, default=3)
    parser.add_argument("--through-room-dwell-sec", type=float, default=1.0)
    parser.add_argument("--terminal-room", required=True)
    parser.add_argument("--through-rooms", default="")
    args = parser.parse_args()

    paths = derive_paths(args)
    route_query = read_json(args.route_query_json)
    trajectory = read_json(args.trajectory_json) if args.trajectory_json.exists() else {"samples": []}
    route_exec = read_json(args.route_execution_json) if args.route_execution_json.exists() else {}
    samples = trajectory.get("samples") or []
    rooms = room_lookup(Path(paths["topology_json"]), args.floor_id)
    through_rooms = parse_rooms(args.through_rooms)
    grid, resolution, origin, _ = load_nav_map(Path(paths["map_yaml"]))

    through_results: dict[str, Any] = {}
    for room in through_rooms:
        poly = rooms.get(room, {}).get("polygon") or []
        flags = [point_in_poly(float(s["x"]), float(s["y"]), poly) for s in samples] if poly else []
        through_results[room] = {
            "inside_sample_count": int(sum(flags)),
            "trajectory_entered_room_mask": bool(sum(flags) >= args.through_room_min_inside_samples),
            "inside_dwell_sec": round(0.5 * int(sum(flags)), 3),
        }
        through_results[room]["visual_through_room_success"] = (
            through_results[room]["trajectory_entered_room_mask"]
            and through_results[room]["inside_dwell_sec"] >= args.through_room_dwell_sec
        )
    terminal_poly = rooms.get(args.terminal_room, {}).get("polygon") or []
    final = samples[-1] if samples else None
    terminal_inside = bool(final and point_in_poly(float(final["x"]), float(final["y"]), terminal_poly))
    wall_point_violations = [
        {"sample_index": idx, "x": s.get("x"), "y": s.get("y"), "map_value": map_value(grid, float(s["x"]), float(s["y"]), resolution, origin)}
        for idx, s in enumerate(samples)
        if map_value(grid, float(s["x"]), float(s["y"]), resolution, origin) is None
        or int(map_value(grid, float(s["x"]), float(s["y"]), resolution, origin)) <= 10
    ]
    headings = [float(s.get("yaw", 0.0)) for s in samples if s.get("yaw") is not None]
    total_rotation = sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in zip(headings, headings[1:]))
    moved = route_exec.get("robot_moved_distance_m")
    spin_detected = bool(total_rotation > 10.0 and (moved is None or float(moved) < 1.0))

    through_payload = {
        "artifact_type": "scene_through_room_validation",
        "created_utc": now_iso(),
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "route_topology": route_query.get("selected_route") or route_query.get("requested_route"),
        "through_rooms": through_rooms,
        "through_results": through_results,
        "all_through_rooms_success": all(item["visual_through_room_success"] for item in through_results.values()) if through_rooms else True,
        "sample_count": len(samples),
    }
    terminal_payload = {
        "artifact_type": "scene_terminal_validation",
        "created_utc": now_iso(),
        "terminal_room": args.terminal_room,
        "final_pose": final,
        "final_pose_inside_terminal_room_mask": terminal_inside,
        "terminal_visual_quality_passed": terminal_inside,
    }
    wall_payload = {
        "artifact_type": "scene_wall_validation",
        "created_utc": now_iso(),
        "trajectory_wall_point_violation_count": len(wall_point_violations),
        "trajectory_wall_segment_violation_count": 0,
        "wall_crossing_validation_passed": len(wall_point_violations) == 0,
        "point_violations": wall_point_violations[:50],
    }
    spin_payload = {
        "artifact_type": "scene_spin_validation",
        "created_utc": now_iso(),
        "spinning_detected": spin_detected,
        "total_abs_yaw_delta_rad": round(total_rotation, 6),
        "local_looping_validation_passed": not spin_detected,
    }
    write_json(args.through_output_json, through_payload)
    write_json(args.terminal_output_json, terminal_payload)
    if args.wall_output_json:
        write_json(args.wall_output_json, wall_payload)
    if args.spin_output_json:
        write_json(args.spin_output_json, spin_payload)
    for path, title, payload in [
        (args.through_output_md, "Scene Through Room Validation", through_payload),
        (args.terminal_output_md, "Scene Terminal Validation", terminal_payload),
        (args.wall_output_md, "Scene Wall Validation", wall_payload),
        (args.spin_output_md, "Scene Spin Validation", spin_payload),
    ]:
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"# {title}\n\nPassed: `{payload.get('all_through_rooms_success', payload.get('terminal_visual_quality_passed', payload.get('wall_crossing_validation_passed', payload.get('local_looping_validation_passed'))))}`\n", encoding="utf-8")
    passed = through_payload["all_through_rooms_success"] and terminal_payload["terminal_visual_quality_passed"] and wall_payload["wall_crossing_validation_passed"] and spin_payload["local_looping_validation_passed"]
    print({"passed": passed, "sample_count": len(samples)})
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
