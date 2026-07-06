#!/usr/bin/env python3
"""RSLGRouteResult -> z-aware overlay input adapter (Layer 4 visualization).

Converts one Layer 3 ``rslg_route_result`` into a RouteResult-derived z-aware
overlay input JSON (schema ``rslg_route_result_z_aware_overlay_input``). The
overlay is derived from:

* ``route_result.route_segments`` (executable same-floor / object-approach)
* explicit connector-handoff ``route_result.route_segments`` (vertical connector)
* ``route_result.metric_path`` (full stitched 2D path, used for a dense overlay
  line when segment waypoints are unavailable)

It projects 2D route points into visualization-only 3D coordinates using the
floor z map (floor_1 z=0.0, floor_2 z=1.6). The vertical connector is rendered as
a semantic handoff / connector line only, explicitly marked ``semantic_handoff``,
``visualization_only``, and ``not_physical_stair_climbing``.

This module is ROS/rclpy-free and importable by the ROS route follower.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import NON_TRANSITION_EDGE, TRUE_TRANSITION_EDGE
from tools.rslg_pipeline.runtime.route_result_adapter_common import (
    DEFAULT_FLOOR_Z_MAP,
    PROJECT,
    AdapterError,
    base_claim_boundary,
    connector_handoff_records,
    executable_segments,
    floor_z,
    guard_route_result,
    identity_block,
    provenance_block,
    query_id,
    segment_waypoints,
    selected_goal,
    utc_now,
    _yaw_between,
)

SCHEMA_NAME = "rslg_route_result_z_aware_overlay_input"
SCHEMA_VERSION = "0.1"
# Public alias used by z_aware_route_projection.py / the route follower.
OVERLAY_SCHEMA = SCHEMA_NAME

PHASE_FLOOR_1 = "floor_1"
PHASE_FLOOR_2 = "floor_2"
PHASE_TRANSITION = "vertical_transition"


def _phase_for_floor(floor_id: Optional[str]) -> str:
    if floor_id == "floor_1":
        return PHASE_FLOOR_1
    if floor_id == "floor_2":
        return PHASE_FLOOR_2
    return PHASE_TRANSITION


def overlay_waypoints_from_route_result(
    route_result: dict[str, Any], floor_z_map: dict[str, float]
) -> list[dict[str, Any]]:
    """Build the z-aware overlay waypoint list from executable route segments."""

    waypoints: list[dict[str, Any]] = []
    index = 0
    for seg in executable_segments(route_result):
        floor_id = seg.get("floor_id")
        phase = _phase_for_floor(floor_id)
        z = round(floor_z(floor_id, floor_z_map), 6)
        pts = segment_waypoints(seg)
        for local_index, pt in enumerate(pts):
            if waypoints and abs(pt["x"] - waypoints[-1]["x"]) < 1e-9 and abs(pt["y"] - waypoints[-1]["y"]) < 1e-9:
                continue
            waypoints.append(
                {
                    "index": index,
                    "waypoint_index": index,
                    "x": round(float(pt["x"]), 6),
                    "y": round(float(pt["y"]), 6),
                    "z": z,
                    "yaw": round(_yaw_between(pts, local_index), 6),
                    "floor_id": floor_id,
                    "route_phase": phase,
                    "segment_id": seg.get("segment_id"),
                    "source": "route_result_segment_waypoint",
                }
            )
            index += 1
    return waypoints


def build_z_aware_overlay_input(
    route_result: dict[str, Any],
    *,
    floor_z_map: Optional[dict[str, float]] = None,
    source_ref: Optional[str] = None,
    transition_edge: str = TRUE_TRANSITION_EDGE,
) -> dict[str, Any]:
    """Build a ``rslg_route_result_z_aware_overlay_input`` payload from a RouteResult."""

    if transition_edge == NON_TRANSITION_EDGE:
        raise AdapterError(
            f"{NON_TRANSITION_EDGE!r} is the non-transition edge and must never be the transition edge"
        )
    guard_route_result(route_result)
    fzmap = dict(floor_z_map or DEFAULT_FLOOR_Z_MAP)

    overlay_waypoints = overlay_waypoints_from_route_result(route_result, fzmap)

    # Anchor the final overlay waypoint to the selected object-approach goal so the
    # overlay ends at the same runtime goal as the PID follower.
    goal = selected_goal(route_result, fzmap)
    if goal and "x" in goal and overlay_waypoints:
        final = overlay_waypoints[-1]
        final["x"] = goal["x"]
        final["y"] = goal["y"]
        final["z"] = goal["z"]
        final["yaw"] = goal.get("yaw", final.get("yaw"))
        final["runtime_goal_candidate_id"] = goal.get("candidate_id")
        final["runtime_goal_role"] = "final_object_approach_goal"

    handoffs = connector_handoff_records(route_result, fzmap)
    z_values = [wp["z"] for wp in overlay_waypoints] + [h["from_z"] for h in handoffs] + [h["to_z"] for h in handoffs]
    z_min = round(min(z_values), 6) if z_values else None
    z_max = round(max(z_values), 6) if z_values else None

    transition_edges = [h.get("transition_edge") for h in handoffs if h.get("transition_edge")]
    transition_edge_used = transition_edges[0] if transition_edges else None
    vertical_transition_edge_is_e001 = bool(transition_edges) and all(
        edge == TRUE_TRANSITION_EDGE for edge in transition_edges
    )
    transition_detected = bool(handoffs) and z_min is not None and z_max is not None and (z_max - z_min) > 0.5

    metric_path = route_result.get("metric_path") or {}
    payload: dict[str, Any] = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT,
        "generated_utc": utc_now(),
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "source_route_result": source_ref,
        "identity": identity_block(route_result),
        "route_type": (route_result.get("identity") or {}).get("query_type"),
        "visualization_overlay": True,
        "visualization_only": True,
        "physical_climb_claimed": False,
        "requires_nav2": False,
        "requires_amcl": False,
        "requires_map_server": False,
        "requires_nav2_map_server": False,
        "floor_z_map": fzmap,
        "transition_edge_used": transition_edge_used,
        "non_transition_edge": NON_TRANSITION_EDGE,
        "vertical_transition_edge_is_e001": vertical_transition_edge_is_e001,
        "overlay_waypoint_count": len(overlay_waypoints),
        "overlay_waypoints": overlay_waypoints,
        "connector_handoffs": handoffs,
        "connector_handoff": handoffs[0] if handoffs else None,
        "metric_path_scope": metric_path.get("metric_path_scope"),
        "z_min": z_min,
        "z_max": z_max,
        "z_aware_visual_transition_detected": transition_detected,
        "final_waypoint": overlay_waypoints[-1] if overlay_waypoints else None,
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
    parser.add_argument("--vertical-transition-edge", default=TRUE_TRANSITION_EDGE)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args(argv)

    if args.no_canonical_write and "canonical" in args.output_json.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    route_result = read_json(args.route_result_json)
    payload = build_z_aware_overlay_input(
        route_result,
        floor_z_map=_parse_floor_z_map(args.floor_z_map),
        source_ref=args.route_result_json.as_posix(),
        transition_edge=args.vertical_transition_edge,
    )
    write_json(args.output_json, payload)
    print(
        json.dumps(
            {
                "schema_name": payload["schema_name"],
                "query_id": payload["identity"]["query_id"],
                "overlay_waypoint_count": payload["overlay_waypoint_count"],
                "z_min": payload["z_min"],
                "z_max": payload["z_max"],
                "z_aware_visual_transition_detected": payload["z_aware_visual_transition_detected"],
                "output_json": args.output_json.as_posix(),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
