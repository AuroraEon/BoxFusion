#!/usr/bin/env python3
"""RSLGRouteResult -> lightweight PID follower runtime input adapter (Layer 4).

Converts one Layer 3 ``rslg_route_result`` into a RouteResult-derived PID follower
runtime input JSON (schema ``rslg_route_result_pid_runtime_input``). The PID
follower drives ``/cmd_vel`` from ``/odom`` feedback; it never uses Nav2, AMCL,
``map_server``, or ``nav2_map_server``.

Design rule for cross-floor routes: the PID input represents same-floor and
object-approach segments as executable waypoint segments, and represents the
vertical connector as a *separate* semantic handoff (visualization-only). It never
synthesizes a physical stair-climb trajectory or gait command.

This module is ROS/rclpy-free so the conda offline Python can build and validate
the runtime input without launching any node.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.runtime.route_result_adapter_common import (
    DEFAULT_FLOOR_Z_MAP,
    PROJECT,
    base_claim_boundary,
    blocked_legacy_rejected,
    connector_handoff_records,
    endpoint_record,
    executable_segments,
    flat_executable_waypoints,
    guard_route_result,
    identity_block,
    provenance_block,
    query_id,
    segment_waypoints,
    selected_goal,
    utc_now,
    floor_z,
)

SCHEMA_NAME = "rslg_route_result_pid_runtime_input"
SCHEMA_VERSION = "0.1"


def _runtime_segment(seg: dict[str, Any], floor_z_map: dict[str, float]) -> dict[str, Any]:
    floor_id = seg.get("floor_id")
    z = round(floor_z(floor_id, floor_z_map), 6)
    pts = segment_waypoints(seg)
    return {
        "segment_id": seg.get("segment_id"),
        "segment_type": seg.get("segment_type"),
        "floor_id": floor_id,
        "z": z,
        "from_room_id": seg.get("from_room_id"),
        "to_room_id": seg.get("to_room_id"),
        "executable": True,
        "segment_length": seg.get("segment_length"),
        "path_source": seg.get("path_source"),
        "path_found": seg.get("path_found"),
        "validation_status": seg.get("validation_status"),
        "waypoint_count": len(pts),
        "waypoints": [
            {"x": round(p["x"], 6), "y": round(p["y"], 6), "z": z, "floor_id": floor_id}
            for p in pts
        ],
    }


def build_pid_runtime_input(
    route_result: dict[str, Any],
    *,
    floor_z_map: Optional[dict[str, float]] = None,
    source_ref: Optional[str] = None,
) -> dict[str, Any]:
    """Build a ``rslg_route_result_pid_runtime_input`` payload from a RouteResult."""

    guard_route_result(route_result)
    fzmap = dict(floor_z_map or DEFAULT_FLOOR_Z_MAP)

    runtime_segments = [_runtime_segment(seg, fzmap) for seg in executable_segments(route_result)]
    handoffs = connector_handoff_records(route_result, fzmap)
    waypoints = flat_executable_waypoints(route_result, fzmap)
    endpoint = endpoint_record(route_result, fzmap)
    goal = selected_goal(route_result, fzmap)
    rejected = blocked_legacy_rejected(route_result)
    metric_path = route_result.get("metric_path") or {}

    payload: dict[str, Any] = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT,
        "generated_utc": utc_now(),
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "source_route_result": source_ref,
        "identity": identity_block(route_result),
        "runtime_policy": {
            "follower": "lightweight_pid_proportional_waypoint_follower",
            "motion_basis": "base_velocity_cmd_vel",
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
            "compatible_with_pid_follower": True,
            "requires_nav2": False,
            "requires_amcl": False,
            "requires_map_server": False,
            "requires_nav2_map_server": False,
        },
        "requires_nav2": False,
        "requires_amcl": False,
        "compatible_with_pid_follower": True,
        "floor_z_map": fzmap,
        "runtime_segments": runtime_segments,
        "connector_handoffs": handoffs,
        "runtime_waypoints": waypoints,
        "runtime_waypoint_count": len(waypoints),
        "selected_approach": goal,
        "selected_goal": goal,
        "endpoint": endpoint,
        "metric_path_scope": metric_path.get("metric_path_scope"),
        "metric_path_length": metric_path.get("path_length"),
        "rejected_runtime_candidates": rejected,
        "claim_boundary": base_claim_boundary(),
        "provenance": provenance_block(route_result, source_ref),
    }
    return payload


def read_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def _parse_floor_z_map(raw: Optional[str]) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(k): float(v) for k, v in json.loads(raw).items()}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-result-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args(argv)

    if args.no_canonical_write and "canonical" in args.output_json.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    route_result = read_json(args.route_result_json)
    payload = build_pid_runtime_input(
        route_result,
        floor_z_map=_parse_floor_z_map(args.floor_z_map),
        source_ref=args.route_result_json.as_posix(),
    )
    write_json(args.output_json, payload)
    print(
        json.dumps(
            {
                "schema_name": payload["schema_name"],
                "query_id": payload["identity"]["query_id"],
                "runtime_segment_count": len(payload["runtime_segments"]),
                "runtime_waypoint_count": payload["runtime_waypoint_count"],
                "connector_handoff_count": len(payload["connector_handoffs"]),
                "output_json": args.output_json.as_posix(),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
