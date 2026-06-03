#!/usr/bin/env python3
"""Validate gateway registry coverage for a parameterized scene route."""

from __future__ import annotations

import argparse
from pathlib import Path

from scene_runtime_common import derive_paths, pair_key, read_json, write_json, now_iso


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--stage-a-output-dir", type=Path)
    parser.add_argument("--route-query-json", type=Path, required=True)
    parser.add_argument("--waypoints-json", type=Path, required=True)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--start-room")
    parser.add_argument("--goal-room")
    parser.add_argument("--through-rooms")
    parser.add_argument("--terminal-room")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()

    paths = derive_paths(args)
    registry_path = paths["gateway_registry_json"]
    assert isinstance(registry_path, Path)
    route_query = read_json(Path(args.route_query_json))
    waypoints = read_json(Path(args.waypoints_json))
    registry = read_json(registry_path)
    gateways = {item["gateway_id"]: item for item in registry.get("gateways", [])}
    pairs = {item.get("pair_key"): item for item in registry.get("gateways", [])}
    route_rooms = route_query.get("selected_route") or route_query.get("room_sequence") or route_query.get("requested_route") or []
    route_pairs = [pair_key(a, b) for a, b in zip(route_rooms, route_rooms[1:])]
    waypoint_gateways = sorted({w.get("gateway_id") for w in waypoints.get("waypoints", []) if w.get("gateway_id")})
    missing_waypoint_gateways = [gid for gid in waypoint_gateways if gid not in gateways]
    missing_route_pairs = [pk for pk in route_pairs if pk not in pairs]
    invalid_gateway_waypoints = []
    for waypoint in waypoints.get("waypoints", []):
        gid = waypoint.get("gateway_id")
        if not gid:
            continue
        item = gateways.get(gid)
        if not item:
            continue
        connects = set(item.get("connects") or [])
        expected = {waypoint.get("from_room"), waypoint.get("to_room")} - {None}
        if expected and not expected <= connects:
            invalid_gateway_waypoints.append({
                "waypoint_index": waypoint.get("waypoint_index"),
                "gateway_id": gid,
                "expected": sorted(expected),
                "connects": sorted(connects),
            })
    payload = {
        "artifact_type": "scene_gateway_readiness_report",
        "created_utc": now_iso(),
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "gateway_registry": registry_path.as_posix(),
        "gateway_count": len(gateways),
        "route_rooms": route_rooms,
        "route_pairs": route_pairs,
        "waypoint_gateways": waypoint_gateways,
        "missing_route_pairs": missing_route_pairs,
        "missing_waypoint_gateways": missing_waypoint_gateways,
        "invalid_gateway_waypoints": invalid_gateway_waypoints,
        "all_registry_gateways_accepted_for_carving": all((g.get("validation") or {}).get("accepted_for_carving") for g in gateways.values()),
    }
    payload["passed"] = (
        registry_path.exists()
        and len(gateways) > 0
        and not missing_route_pairs
        and not missing_waypoint_gateways
        and not invalid_gateway_waypoints
    )
    write_json(args.output_json, payload)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.write_text(
        "\n".join([
            "# Scene Gateway Readiness Report",
            "",
            f"Passed: `{payload['passed']}`",
            f"Gateway registry: `{registry_path}`",
            f"Route: `{' -> '.join(route_rooms)}`",
            f"Route pairs: `{', '.join(route_pairs)}`",
            f"Waypoint gateways: `{', '.join(waypoint_gateways)}`",
            f"Missing route pairs: `{missing_route_pairs}`",
            f"Missing waypoint gateways: `{missing_waypoint_gateways}`",
        ]) + "\n",
        encoding="utf-8",
    )
    print(payload)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
