#!/usr/bin/env python3
"""Validate or regenerate a scene semantic route and waypoint payload."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from scene_runtime_common import (
    derive_paths,
    load_nav_map,
    map_value,
    now_iso,
    pair_key,
    parse_rooms,
    read_json,
    room_lookup,
    route_id,
    write_json,
    yaw_between,
)


def gateway_for_pair(gateways: list[dict[str, Any]], a: str, b: str) -> dict[str, Any]:
    pk = pair_key(a, b)
    candidates = [g for g in gateways if g.get("pair_key") == pk]
    accepted = [g for g in candidates if (g.get("validation") or {}).get("accepted_for_carving", True)]
    if accepted:
        return sorted(accepted, key=lambda item: item.get("gateway_id", ""))[0]
    if candidates:
        return sorted(candidates, key=lambda item: item.get("gateway_id", ""))[0]
    raise ValueError(f"no gateway registry entry for route pair {a} -> {b} ({pk})")


def build_route_payload(args: argparse.Namespace, paths: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    topology_path = paths["topology_json"]
    registry_path = paths["gateway_registry_json"]
    map_yaml = paths["map_yaml"]
    assert isinstance(topology_path, Path) and isinstance(registry_path, Path) and isinstance(map_yaml, Path)
    rooms = room_lookup(topology_path, args.floor_id)
    registry = read_json(registry_path)
    gateways = registry.get("gateways", [])
    grid, resolution, origin, _ = load_nav_map(map_yaml)
    route_rooms = [args.start_room, *parse_rooms(args.through_rooms), args.goal_room]
    rid = route_id(args.floor_id, args.start_room, args.goal_room)
    waypoint_rows: list[dict[str, Any]] = []

    def append_room(room: str, source: str) -> None:
        item = rooms[room]
        x, y = item["center"]
        waypoint_rows.append({
            "scene_id": args.scene_id,
            "floor_id": args.floor_id,
            "route_id": rid,
            "waypoint_index": len(waypoint_rows),
            "x": float(x),
            "y": float(y),
            "yaw": None,
            "source": source,
            "from_room": room,
            "to_room": None,
            "gateway_id": None,
            "pair_key": None,
            "reason": "room center from topology",
            "occupancy_value_at_waypoint": map_value(grid, float(x), float(y), resolution, origin),
        })

    append_room(route_rooms[0], "room_center")
    edges = []
    for left, right in zip(route_rooms, route_rooms[1:]):
        gateway = gateway_for_pair(gateways, left, right)
        waypoint_rows.append({
            "scene_id": args.scene_id,
            "floor_id": args.floor_id,
            "route_id": rid,
            "waypoint_index": len(waypoint_rows),
            "x": float(gateway["x"]),
            "y": float(gateway["y"]),
            "yaw": float(gateway.get("yaw") or 0.0),
            "source": "gateway",
            "from_room": left,
            "to_room": right,
            "gateway_id": gateway["gateway_id"],
            "pair_key": gateway.get("pair_key") or pair_key(left, right),
            "reason": "gateway from Stage-A final_vector_map_snapshot gateway registry",
            "occupancy_value_at_waypoint": map_value(grid, float(gateway["x"]), float(gateway["y"]), resolution, origin),
        })
        append_room(right, "room_center")
        edges.append({
            "source": left,
            "target": right,
            "relation_type": gateway.get("topology_relation_type"),
            "confidence": 0.45,
            "status": gateway.get("topology_status"),
            "gateway_id": gateway["gateway_id"],
            "pair_key": gateway.get("pair_key") or pair_key(left, right),
        })
    for idx, waypoint in enumerate(waypoint_rows[:-1]):
        waypoint["yaw"] = yaw_between(waypoint, waypoint_rows[idx + 1])
    query = {
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "requested_route": route_rooms,
        "selected_route": route_rooms,
        "route_valid_in_topology": True,
        "edges": edges,
        "map_yaml": map_yaml.as_posix(),
    }
    waypoints = {
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "route_id": rid,
        "coordinate_frame": f"same as {args.floor_id} map YAML origin/resolution",
        "waypoints": waypoint_rows,
    }
    return query, waypoints


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--stage-a-output-dir", type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--route-query-json", type=Path)
    parser.add_argument("--waypoints-json", type=Path)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--goal-room", required=True)
    parser.add_argument("--through-rooms", default="")
    parser.add_argument("--terminal-room")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--waypoints-output-json", type=Path)
    parser.add_argument("--force-regenerate", action="store_true")
    args = parser.parse_args()

    if args.output_json and not args.route_query_json:
        args.route_query_json = args.output_json
    if args.waypoints_output_json and not args.waypoints_json:
        args.waypoints_json = args.waypoints_output_json
    paths = derive_paths(args)
    query_path = Path(args.route_query_json) if args.route_query_json else paths["route_query_json"]
    waypoints_path = Path(args.waypoints_json) if args.waypoints_json else paths["waypoints_json"]
    assert isinstance(query_path, Path) and isinstance(waypoints_path, Path)
    expected_query, expected_waypoints = build_route_payload(args, paths)

    existing_query = read_json(query_path) if query_path.exists() else None
    existing_waypoints = read_json(waypoints_path) if waypoints_path.exists() else None
    expected_route = expected_query["selected_route"]
    existing_route = (existing_query or {}).get("selected_route") or (existing_query or {}).get("requested_route")
    existing_waypoint_count = len((existing_waypoints or {}).get("waypoints") or [])
    regenerate = args.force_regenerate or not query_path.exists() or not waypoints_path.exists() or existing_route != expected_route
    if regenerate:
        write_json(query_path, expected_query)
        write_json(waypoints_path, expected_waypoints)
        active_query = expected_query
        active_waypoints = expected_waypoints
    else:
        active_query = existing_query
        active_waypoints = existing_waypoints

    waypoint_values = [w.get("occupancy_value_at_waypoint") for w in (active_waypoints or {}).get("waypoints", [])]
    payload = {
        "artifact_type": "scene_semantic_route_validation",
        "created_utc": now_iso(),
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "route_query_json": query_path.as_posix(),
        "waypoints_json": waypoints_path.as_posix(),
        "expected_route": expected_route,
        "active_route": (active_query or {}).get("selected_route") or (active_query or {}).get("requested_route"),
        "waypoint_count": len((active_waypoints or {}).get("waypoints") or []),
        "existing_waypoint_count": existing_waypoint_count,
        "regenerated": regenerate,
        "all_waypoints_free": all(value in (0, 254) for value in waypoint_values if value is not None),
        "gateway_waypoint_count": len([w for w in (active_waypoints or {}).get("waypoints", []) if w.get("source") == "gateway"]),
    }
    payload["passed"] = (
        payload["active_route"] == expected_route
        and payload["waypoint_count"] >= len(expected_route)
        and payload["gateway_waypoint_count"] == max(0, len(expected_route) - 1)
        and payload["all_waypoints_free"]
    )
    if args.output_json and args.output_json != query_path:
        write_json(args.output_json, payload)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(
            "\n".join([
                "# Scene Semantic Route Validation",
                "",
                f"Passed: `{payload['passed']}`",
                f"Regenerated: `{regenerate}`",
                f"Route: `{' -> '.join(payload['active_route'])}`",
                f"Waypoints: `{payload['waypoint_count']}`",
            ]) + "\n",
            encoding="utf-8",
        )
    print(payload)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
