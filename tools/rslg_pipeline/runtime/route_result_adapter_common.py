#!/usr/bin/env python3
"""Shared, dependency-light helpers for the RSLGRouteResult -> Layer 4 adapters.

Layer 3 owns route generation and emits the formal ``rslg_route_result`` schema.
Layer 4 runtime/visualization adapters consume ``RSLGRouteResult``-derived inputs
only; they never re-plan and never own a planner schema.

This module extracts the pieces the three Layer 4 adapters need (executable
same-floor / object-approach waypoint segments, semantic connector handoffs, the
selected object-approach goal, and the blocked-legacy rejected candidate) from an
``rslg_route_result`` payload. It is intentionally free of ROS / rclpy / numpy so
the conda offline Python can import and exercise it.

Truth guards enforced here:

* ``generated_ring_037`` (blocked legacy evidence) is never a runtime goal.
* ``vt_1_centerline_e003`` (non-transition edge) is never a transition edge.
* ``vt_1_centerline_e001`` is the only transition edge when one is needed.
* Floor z values (floor_1 z=0.0, floor_2 z=1.6) are visualization-only.
* A connector handoff is a semantic / visualization-only handoff, never a
  physical stair-climb / gait execution.
"""

from __future__ import annotations

import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
)

ROUTE_RESULT_SCHEMA_NAME = "rslg_route_result"
DEFAULT_FLOOR_Z_MAP = {"floor_1": 0.0, "floor_2": 1.6}

# Route segment types that the lightweight PID follower can execute as x/y
# waypoint segments on a single floor.
EXECUTABLE_SEGMENT_TYPES = (
    "same_floor",
    "same_floor_metric",
    "object_approach",
    "object_approach_metric",
)
# Route segment types that are semantic / visualization-only handoffs. These must
# never be turned into a physical stair-climbing trajectory.
CONNECTOR_SEGMENT_TYPES = ("vertical_transition", "connector_handoff")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class AdapterError(ValueError):
    """Raised when a RouteResult violates a Layer 4 adapter truth guard."""


def is_route_result(data: dict[str, Any]) -> bool:
    return isinstance(data, dict) and data.get("schema_name") == ROUTE_RESULT_SCHEMA_NAME


def require_route_result(data: dict[str, Any]) -> None:
    if not is_route_result(data):
        raise AdapterError(
            f"expected schema_name={ROUTE_RESULT_SCHEMA_NAME!r}, got {data.get('schema_name')!r}"
        )


def floor_z(floor_id: Optional[str], floor_z_map: dict[str, float]) -> float:
    if floor_id is None:
        return 0.0
    return float(floor_z_map.get(str(floor_id), 0.0))


def identity_block(route_result: dict[str, Any]) -> dict[str, Any]:
    identity = route_result.get("identity") or {}
    return {
        "query_id": identity.get("query_id"),
        "query_text": identity.get("query_text"),
        "query_type": identity.get("query_type"),
        "scene_id": identity.get("scene_id"),
        "planner_version": identity.get("planner_version"),
    }


def query_id(route_result: dict[str, Any]) -> str:
    qid = (route_result.get("identity") or {}).get("query_id")
    if not qid:
        raise AdapterError("route_result.identity.query_id is required")
    return str(qid)


def selected_candidate_id(route_result: dict[str, Any]) -> Optional[str]:
    selected = (route_result.get("approach") or {}).get("selected_approach")
    if isinstance(selected, dict) and selected.get("candidate_id"):
        return str(selected["candidate_id"])
    return None


def guard_selected_goal(route_result: dict[str, Any]) -> None:
    """A blocked legacy candidate must never be the selected runtime goal."""

    cid = selected_candidate_id(route_result)
    if cid and cid in BLOCKED_LEGACY_APPROACH_IDS:
        raise AdapterError(
            f"blocked legacy approach {cid!r} must never be the selected runtime goal"
        )


def _connector_sequence_by_id(route_result: dict[str, Any]) -> dict[str, dict[str, Any]]:
    by_id: dict[str, dict[str, Any]] = {}
    for connector in connector_sequence(route_result):
        connector_id = connector.get("connector_id")
        if connector_id:
            by_id[str(connector_id)] = connector
    return by_id


def transition_edges_used(route_result: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    seq_by_id = _connector_sequence_by_id(route_result)
    for segment in route_result.get("route_segments") or []:
        if not isinstance(segment, dict):
            continue
        if segment.get("segment_type") not in CONNECTOR_SEGMENT_TYPES:
            continue
        connector = seq_by_id.get(str(segment.get("connector_id")))
        edge = (
            segment.get("transition_edge")
            or segment.get("connector_edge")
            or (connector or {}).get("transition_edge")
        )
        if edge:
            edges.append(str(edge))
    return edges


def guard_transition_edges(route_result: dict[str, Any]) -> None:
    for edge in transition_edges_used(route_result):
        if edge == NON_TRANSITION_EDGE:
            raise AdapterError(
                f"non-transition edge {NON_TRANSITION_EDGE!r} must never be used as a transition edge"
            )
        if edge and edge != TRUE_TRANSITION_EDGE:
            raise AdapterError(
                f"unexpected transition edge {edge!r}; only {TRUE_TRANSITION_EDGE!r} is allowed"
            )


def guard_route_result(route_result: dict[str, Any]) -> None:
    """Run the full set of Layer 4 adapter truth guards."""

    require_route_result(route_result)
    guard_selected_goal(route_result)
    guard_transition_edges(route_result)
    runtime = route_result.get("runtime_interface") or {}
    if runtime.get("requires_nav2") not in (False, None):
        raise AdapterError("route_result.runtime_interface.requires_nav2 must be false")
    if runtime.get("requires_amcl") not in (False, None):
        raise AdapterError("route_result.runtime_interface.requires_amcl must be false")


def _yaw_between(points: list[dict[str, float]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    if index >= len(points) - 1:
        before, after = points[index - 1], points[index]
    else:
        before, after = points[index], points[index + 1]
    return math.atan2(float(after["y"]) - float(before["y"]), float(after["x"]) - float(before["x"]))


def segment_waypoints(segment: dict[str, Any]) -> list[dict[str, float]]:
    """Return the list of ``{x, y}`` waypoints for a route segment (may be empty)."""

    raw = segment.get("waypoints")
    if not isinstance(raw, list):
        return []
    out: list[dict[str, float]] = []
    for wp in raw:
        if isinstance(wp, dict) and "x" in wp and "y" in wp:
            out.append({"x": float(wp["x"]), "y": float(wp["y"])})
    return out


def _xy_pair(value: Any) -> Optional[dict[str, float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return {"x": float(value[0]), "y": float(value[1])}
    if isinstance(value, dict) and "x" in value and "y" in value:
        return {"x": float(value["x"]), "y": float(value["y"])}
    return None


def executable_segments(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the same-floor / object-approach segments the PID follower executes."""

    return [
        seg
        for seg in route_result.get("route_segments") or []
        if isinstance(seg, dict) and seg.get("segment_type") in EXECUTABLE_SEGMENT_TYPES
    ]


def connector_segments(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the vertical-transition / connector-handoff segments (semantic only)."""

    return [
        seg
        for seg in route_result.get("route_segments") or []
        if isinstance(seg, dict) and seg.get("segment_type") in CONNECTOR_SEGMENT_TYPES
    ]


def connector_sequence(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    seq = (route_result.get("semantic_route") or {}).get("connector_sequence") or []
    return [c for c in seq if isinstance(c, dict)]


def flat_executable_waypoints(
    route_result: dict[str, Any], floor_z_map: dict[str, float]
) -> list[dict[str, Any]]:
    """Flatten executable segments into a de-duplicated ordered waypoint list.

    Each waypoint carries ``waypoint_index``, ``x``, ``y``, visualization ``z``,
    ``floor_id``, ``yaw``, and ``segment_id``. Connector handoffs are skipped: the
    PID follower does not physically traverse the vertical connector, so no
    stair-climb waypoints are synthesized.
    """

    waypoints: list[dict[str, Any]] = []
    index = 0
    for seg in executable_segments(route_result):
        floor_id = seg.get("floor_id")
        pts = segment_waypoints(seg)
        for local_index, pt in enumerate(pts):
            if waypoints and abs(pt["x"] - waypoints[-1]["x"]) < 1e-9 and abs(pt["y"] - waypoints[-1]["y"]) < 1e-9:
                continue
            waypoints.append(
                {
                    "waypoint_index": index,
                    "x": round(float(pt["x"]), 6),
                    "y": round(float(pt["y"]), 6),
                    "z": round(floor_z(floor_id, floor_z_map), 6),
                    "floor_id": floor_id,
                    "yaw": round(_yaw_between(pts, local_index), 6),
                    "segment_id": seg.get("segment_id"),
                    "source": "route_result_segment_waypoint",
                }
            )
            index += 1
    return waypoints


def selected_goal(
    route_result: dict[str, Any], floor_z_map: dict[str, float]
) -> Optional[dict[str, Any]]:
    """Return the selected object-approach goal as an endpoint record, if present."""

    selected = (route_result.get("approach") or {}).get("selected_approach")
    if not isinstance(selected, dict):
        return None
    cid = selected.get("candidate_id")
    if cid in BLOCKED_LEGACY_APPROACH_IDS:
        raise AdapterError(f"blocked legacy approach {cid!r} must never be the selected goal")
    xy = selected.get("world_xy") or selected.get("position")
    floor_id = (route_result.get("target") or {}).get("target_floor_id") or "floor_2"
    record: dict[str, Any] = {
        "candidate_id": cid,
        "is_runtime_goal": bool(selected.get("is_runtime_goal", True)),
        "is_object_centroid": bool(selected.get("is_object_centroid", False)),
        "clearance_m": selected.get("clearance_m"),
        "yaw": selected.get("yaw"),
        "floor_id": floor_id,
        "z": round(floor_z(floor_id, floor_z_map), 6),
    }
    if isinstance(xy, (list, tuple)) and len(xy) >= 2:
        record["world_xy"] = [round(float(xy[0]), 6), round(float(xy[1]), 6)]
        record["x"] = round(float(xy[0]), 6)
        record["y"] = round(float(xy[1]), 6)
    return record


def endpoint_record(
    route_result: dict[str, Any], floor_z_map: dict[str, float]
) -> Optional[dict[str, Any]]:
    """Return the route endpoint (selected approach goal, else metric_path.endpoint)."""

    goal = selected_goal(route_result, floor_z_map)
    if goal and "world_xy" in goal:
        return goal
    endpoint = (route_result.get("metric_path") or {}).get("endpoint")
    if isinstance(endpoint, (list, tuple)) and len(endpoint) >= 2:
        floor_seq = (route_result.get("semantic_route") or {}).get("floor_sequence") or []
        floor_id = floor_seq[-1] if floor_seq else (route_result.get("target") or {}).get("target_floor_id")
        return {
            "candidate_id": None,
            "is_runtime_goal": True,
            "is_object_centroid": False,
            "floor_id": floor_id,
            "x": round(float(endpoint[0]), 6),
            "y": round(float(endpoint[1]), 6),
            "z": round(floor_z(floor_id, floor_z_map), 6),
            "world_xy": [round(float(endpoint[0]), 6), round(float(endpoint[1]), 6)],
        }
    return None


def blocked_legacy_rejected(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the blocked-legacy (``generated_ring_037``) rejected records, if any."""

    out: list[dict[str, Any]] = []
    for record in (route_result.get("approach") or {}).get("rejected_candidates") or []:
        if not isinstance(record, dict):
            continue
        cid = record.get("candidate_id")
        if cid in BLOCKED_LEGACY_APPROACH_IDS:
            xy = record.get("world_xy") or record.get("position")
            out.append(
                {
                    "candidate_id": cid,
                    "status": record.get("status", "blocked"),
                    "reason": record.get("reason"),
                    "evidence_only": True,
                    "is_runtime_goal": False,
                    "world_xy": list(xy) if isinstance(xy, (list, tuple)) else None,
                }
            )
    return out


def connector_handoff_records(
    route_result: dict[str, Any], floor_z_map: dict[str, float]
) -> list[dict[str, Any]]:
    """Build semantic connector-handoff records (visualization-only, not physical).

    Each record carries the transition edge, the from/to floors and their
    visualization z, and start/end points where available. Handoffs are explicitly
    flagged as semantic / visualization-only and never executable waypoint
    segments.
    """

    seq_by_id = _connector_sequence_by_id(route_result)
    records: list[dict[str, Any]] = []
    for order, seg in enumerate(connector_segments(route_result)):
        connector_id = seg.get("connector_id")
        connector = seq_by_id.get(str(connector_id), {}) if connector_id else {}
        transition_edge = (
            seg.get("transition_edge")
            or seg.get("connector_edge")
            or connector.get("transition_edge")
            or TRUE_TRANSITION_EDGE
        )
        if transition_edge == NON_TRANSITION_EDGE:
            raise AdapterError(
                f"non-transition edge {NON_TRANSITION_EDGE!r} must never be used as a transition edge"
            )
        from_floor = seg.get("source_floor") or seg.get("from_floor") or connector.get("from_floor")
        to_floor = seg.get("target_floor") or seg.get("to_floor") or connector.get("to_floor")
        pts = segment_waypoints(seg)
        start_pt = pts[0] if pts else _xy_pair(seg.get("start"))
        end_pt = pts[-1] if pts else _xy_pair(seg.get("end"))
        record = {
            "handoff_index": order,
            "connector_id": connector_id,
            "connector_id_alias": connector.get("connector_id_alias"),
            "transition_edge": transition_edge,
            "non_transition_edge": connector.get("non_transition_edge", NON_TRANSITION_EDGE),
            "from_floor": from_floor,
            "to_floor": to_floor,
            "from_z": round(floor_z(from_floor, floor_z_map), 6),
            "to_z": round(floor_z(to_floor, floor_z_map), 6),
            "start": (
                {"x": round(start_pt["x"], 6), "y": round(start_pt["y"], 6), "z": round(floor_z(from_floor, floor_z_map), 6)}
                if start_pt
                else None
            ),
            "end": (
                {"x": round(end_pt["x"], 6), "y": round(end_pt["y"], 6), "z": round(floor_z(to_floor, floor_z_map), 6)}
                if end_pt
                else None
            ),
            "handoff_type": "semantic_handoff",
            "visualization_only": True,
            "not_physical_stair_climbing": True,
            "executable_waypoint_segment": False,
        }
        records.append(record)
    return records


def base_claim_boundary() -> dict[str, bool]:
    """Return the shared Layer 4 adapter claim-boundary block."""

    return {
        "no_nav2_dependency": True,
        "no_amcl_dependency": True,
        "no_map_server_dependency": True,
        "no_nav2_map_server_dependency": True,
        "no_real_robot_claim": True,
        "no_collision_free_guarantee": True,
        "no_physical_stair_climbing_claim": True,
        "no_go2_control_claim": True,
        "floor_z_values_visualization_only": True,
        "connector_handoff_is_semantic_visualization_only": True,
    }


def provenance_block(route_result: dict[str, Any], source_ref: Optional[str]) -> dict[str, Any]:
    provenance = route_result.get("provenance") or {}
    return {
        "source_route_result": source_ref,
        "source_route_result_schema": route_result.get("schema_name"),
        "source_route_result_schema_version": route_result.get("schema_version"),
        "planner_version": (route_result.get("identity") or {}).get("planner_version"),
        "canonical_root": provenance.get("canonical_root"),
        "notes": (
            "Layer 4 adapter input derived from an RSLGRouteResult. Not a planner "
            "schema. Canonical real routes are comparison/provenance only."
        ),
    }


PROJECT = PROJECT_NAME
