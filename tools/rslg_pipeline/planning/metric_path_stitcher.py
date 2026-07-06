"""Dynamic metric-path stitching for the RSLG-SLAM static planner.

Converts a semantic room/floor/connector route into a **dynamically stitched**
metric path by planning per-floor occupancy-grid A* segments over the canonical
stable maps and joining them with a semantic connector handoff and a local
object-approach segment. RSLG-SLAM is the project name; ``BoxFusion`` is only a
historical repository path.

This module replaces the retired canonical full-route metric fallback: the
metric-path length is computed from stitched A* geometry, never copied from a
canonical real route. Canonical real-route lengths are used only as a
comparison/provenance number in the stitching report.

Guarantees:

* Same-floor movement -> ``same_floor_metric`` A* segment on the floor stable
  occupancy map (:class:`occupancy_planner.OccupancyPlanner`).
* Floor transition -> ``connector_handoff`` segment (semantic vertical handoff,
  transition edge ``vt_1_centerline_e001``). Never physical stair climbing.
* Final object approach -> ``object_approach_metric`` segment, preferring the
  selected candidate's reachable A* waypoints (``dynamic_local_approach``).
* A same-floor segment that cannot be dynamically planned is recorded as a
  ``semantic_only`` segment with an explicit fallback reason.
* No ROS/Gazebo/RViz/Nav2/AMCL/map_server; no canonical writes.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning import artifact_loader as loader
from tools.rslg_pipeline.planning.occupancy_planner import OccupancyPlanner
from tools.rslg_pipeline.project_truth import (
    NON_TRANSITION_EDGE,
    TRUE_TRANSITION_EDGE,
)

# Query types that end at an object approach point.
OBJECT_QUERY_TYPES = {"object_to_path", "object_in_room", "cross_floor_object"}

# (metric_path_scope, route_generation_scope, structure_sequence_required)
_SCOPE_BY_QUERY_TYPE: dict[str, tuple[str, str, bool]] = {
    "object_to_path": ("local_object_approach", "object_local", False),
    "object_in_room": ("full_route_to_object_approach", "full_cross_floor", True),
    "cross_floor_object": (
        "full_cross_floor_route_to_object_approach",
        "full_cross_floor",
        True,
    ),
    "cross_floor_room": ("full_cross_floor_room_route", "full_cross_floor", True),
    "floor_connector": ("floor_transition_handoff_route", "floor_transition", True),
    "room_gateway": ("same_floor_room_route", "same_floor", True),
}

_EPS = 1e-6


def route_scope(query_task: dict[str, Any]) -> dict[str, Any]:
    """Resolve metric-path scope / generation scope / structure-required flag."""

    query_type = query_task.get("query_type")
    query_id = query_task.get("query_id") or ""
    scope, gen_scope, structure_required = _SCOPE_BY_QUERY_TYPE.get(
        query_type, ("full_route", "full_cross_floor", True)
    )
    if "blocked_candidate_rejection" in query_id:
        scope = "full_route_to_valid_approach_with_blocked_candidate_rejection"
    return {
        "metric_path_scope": scope,
        "route_generation_scope": gen_scope,
        "structure_sequence_required": structure_required,
    }


def _to_xy(point: Any) -> list[float] | None:
    if isinstance(point, dict) and "x" in point and "y" in point:
        return [float(point["x"]), float(point["y"])]
    if isinstance(point, (list, tuple)) and len(point) >= 2:
        return [float(point[0]), float(point[1])]
    return None


def _as_points(seq: Any) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for item in seq or []:
        xy = _to_xy(item)
        if xy is not None:
            points.append({"x": round(xy[0], 6), "y": round(xy[1], 6)})
    return points


def _polyline_length(points: list[dict[str, float]]) -> float:
    return sum(
        math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        for a, b in zip(points, points[1:])
    )


class _FloorPlannerCache:
    """Lazily construct one :class:`OccupancyPlanner` per floor id."""

    def __init__(self, scene_context: dict[str, Any]) -> None:
        self._ctx = scene_context
        self._planners: dict[str, OccupancyPlanner | None] = {}
        self.errors: dict[str, str] = {}

    def get(self, floor_id: str | None) -> OccupancyPlanner | None:
        if not floor_id:
            return None
        if floor_id in self._planners:
            return self._planners[floor_id]
        inputs = loader.load_floor_occupancy_planner_inputs(self._ctx, floor_id)
        if inputs["status"] != "ok" or not inputs["map_yaml_exists"]:
            self._planners[floor_id] = None
            self.errors[floor_id] = f"stable map for {floor_id} missing"
            return None
        params = inputs["planner_params"]
        try:
            planner = OccupancyPlanner(
                Path(inputs["map_yaml_path"]),
                params["inflation_radius_m"],
                params["waypoint_spacing_m"],
            )
        except Exception as exc:  # pragma: no cover - defensive
            self._planners[floor_id] = None
            self.errors[floor_id] = f"could not load {floor_id} planner: {exc}"
            return None
        self._planners[floor_id] = planner
        return planner


def _same_floor_segment(
    planners: _FloorPlannerCache,
    *,
    index: int,
    floor_id: str | None,
    from_id: str,
    to_id: str,
    from_xy: list[float] | None,
    to_xy: list[float] | None,
    from_room_id: str | None,
    to_room_id: str | None,
) -> dict[str, Any]:
    """Plan one same-floor A* segment, degrading to ``semantic_only`` on failure."""

    seg_id = f"seg_{index:02d}_same_floor_metric_{from_id}_to_{to_id}"
    planner = planners.get(floor_id)
    if planner is None or from_xy is None or to_xy is None:
        reason = (
            planners.errors.get(floor_id, "missing floor planner")
            if planner is None
            else "missing anchor geometry"
        )
        return {
            "segment_id": seg_id,
            "segment_type": "semantic_only",
            "floor_id": floor_id,
            "from_room_id": from_room_id,
            "to_room_id": to_room_id,
            "connector_id": None,
            "start": from_xy,
            "end": to_xy,
            "waypoints": [],
            "segment_length": None,
            "path_found": False,
            "validation_status": "not_validated",
            "path_source": "semantic_only",
            "fallback_reason": reason,
        }
    try:
        points, meta = planner.plan_segment(from_xy, to_xy, seg_id)
    except Exception as exc:
        return {
            "segment_id": seg_id,
            "segment_type": "semantic_only",
            "floor_id": floor_id,
            "from_room_id": from_room_id,
            "to_room_id": to_room_id,
            "connector_id": None,
            "start": from_xy,
            "end": to_xy,
            "waypoints": [],
            "segment_length": None,
            "path_found": False,
            "validation_status": "failed",
            "path_source": "semantic_only",
            "fallback_reason": f"A* segment planning failed: {exc}",
        }
    passed = bool(meta.get("wall_crossing_validation_passed"))
    return {
        "segment_id": seg_id,
        "segment_type": "same_floor_metric",
        "floor_id": floor_id,
        "from_room_id": from_room_id,
        "to_room_id": to_room_id,
        "connector_id": None,
        "start": [points[0]["x"], points[0]["y"]] if points else from_xy,
        "end": [points[-1]["x"], points[-1]["y"]] if points else to_xy,
        "waypoints": points,
        "segment_length": round(float(meta.get("path_length_m", 0.0)), 6),
        "path_found": True,
        "validation_status": "passed" if passed else "partial",
        "path_source": "dynamic_stitched",
        "minimum_clearance_m": meta.get("minimum_clearance_m"),
    }


def _connector_handoff_segment(
    *, index: int, connector_geom: dict[str, Any]
) -> dict[str, Any]:
    """Build the semantic vertical-transition (handoff) segment."""

    connector_id = connector_geom.get("connector_id") or "vt_1"
    entry = _to_xy(connector_geom.get("entry_xy"))
    exit_ = _to_xy(connector_geom.get("exit_xy"))
    transition_edge = connector_geom.get("transition_edge") or TRUE_TRANSITION_EDGE
    if transition_edge == NON_TRANSITION_EDGE:
        # Never allow the non-transition edge to represent a transition.
        transition_edge = TRUE_TRANSITION_EDGE
    seg_id = f"seg_{index:02d}_connector_handoff_{connector_id}"
    if entry is None or exit_ is None:
        return {
            "segment_id": seg_id,
            "segment_type": "connector_handoff",
            "floor_id": None,
            "from_room_id": connector_geom.get("source_room"),
            "to_room_id": connector_geom.get("target_room"),
            "connector_id": connector_id,
            "transition_edge": transition_edge,
            "source_floor": connector_geom.get("from_floor"),
            "target_floor": connector_geom.get("to_floor"),
            "start": entry,
            "end": exit_,
            "waypoints": [],
            "segment_length": None,
            "path_found": False,
            "validation_status": "not_validated",
            "path_source": "connector_endpoint_fallback",
            "fallback_reason": "connector endpoint geometry incomplete",
            "physical_stair_climbing_claimed": False,
        }
    points = [
        {"x": round(entry[0], 6), "y": round(entry[1], 6)},
        {"x": round(exit_[0], 6), "y": round(exit_[1], 6)},
    ]
    chord = _polyline_length(points)
    return {
        "segment_id": seg_id,
        "segment_type": "connector_handoff",
        "floor_id": None,
        "from_room_id": connector_geom.get("source_room"),
        "to_room_id": connector_geom.get("target_room"),
        "connector_id": connector_id,
        "transition_edge": transition_edge,
        "source_floor": connector_geom.get("from_floor"),
        "target_floor": connector_geom.get("to_floor"),
        "start": entry,
        "end": exit_,
        "waypoints": points,
        "segment_length": round(chord, 6),
        "path_found": True,
        "validation_status": "passed",
        "path_source": "dynamic_connector_handoff",
        "z_span_m": connector_geom.get("z_span_m"),
        "physical_stair_climbing_claimed": False,
    }


def _object_approach_segment(
    planners: _FloorPlannerCache,
    *,
    index: int,
    floor_id: str | None,
    from_room_id: str | None,
    from_xy: list[float] | None,
    approach_result: dict[str, Any],
) -> dict[str, Any]:
    """Build the final local object-approach segment.

    Prefers the selected candidate's own reachable A* waypoints
    (``dynamic_local_approach``); otherwise plans A* on the target floor from the
    last room anchor to the approach point.
    """

    selected = approach_result.get("selected_approach") or {}
    candidate_id = selected.get("candidate_id")
    approach_xy = _to_xy(selected.get("world_xy"))
    seg_id = f"seg_{index:02d}_object_approach_metric_{candidate_id or 'approach'}"

    waypoints = _as_points(selected.get("a_star_waypoints"))
    if len(waypoints) >= 2:
        length = _polyline_length(waypoints)
        return {
            "segment_id": seg_id,
            "segment_type": "object_approach_metric",
            "floor_id": floor_id,
            "from_room_id": from_room_id,
            "to_room_id": from_room_id,
            "connector_id": None,
            "approach_candidate_id": candidate_id,
            "start": [waypoints[0]["x"], waypoints[0]["y"]],
            "end": [waypoints[-1]["x"], waypoints[-1]["y"]],
            "waypoints": waypoints,
            "segment_length": round(length, 6),
            "path_found": True,
            "validation_status": "passed",
            "path_source": "dynamic_local_approach",
        }

    # Fall back to planning A* from the last room anchor to the approach point.
    planner = planners.get(floor_id)
    if planner is not None and from_xy is not None and approach_xy is not None:
        try:
            points, meta = planner.plan_segment(from_xy, approach_xy, seg_id)
            passed = bool(meta.get("wall_crossing_validation_passed"))
            return {
                "segment_id": seg_id,
                "segment_type": "object_approach_metric",
                "floor_id": floor_id,
                "from_room_id": from_room_id,
                "to_room_id": from_room_id,
                "connector_id": None,
                "approach_candidate_id": candidate_id,
                "start": [points[0]["x"], points[0]["y"]] if points else from_xy,
                "end": [points[-1]["x"], points[-1]["y"]] if points else approach_xy,
                "waypoints": points,
                "segment_length": round(float(meta.get("path_length_m", 0.0)), 6),
                "path_found": True,
                "validation_status": "passed" if passed else "partial",
                "path_source": "dynamic_stitched",
            }
        except Exception as exc:
            reason = f"object approach A* failed: {exc}"
    else:
        reason = "missing floor planner or approach geometry"

    return {
        "segment_id": seg_id,
        "segment_type": "semantic_only",
        "floor_id": floor_id,
        "from_room_id": from_room_id,
        "to_room_id": from_room_id,
        "connector_id": None,
        "approach_candidate_id": candidate_id,
        "start": from_xy,
        "end": approach_xy,
        "waypoints": [],
        "segment_length": None,
        "path_found": False,
        "validation_status": "not_validated",
        "path_source": "semantic_only",
        "fallback_reason": reason,
    }


def _concat_waypoints(segments: list[dict[str, Any]]) -> list[dict[str, float]]:
    """Concatenate segment waypoints, dropping duplicate adjacent points."""

    path: list[dict[str, float]] = []
    for segment in segments:
        for point in segment.get("waypoints") or []:
            if not path:
                path.append(point)
                continue
            last = path[-1]
            if abs(point["x"] - last["x"]) < _EPS and abs(point["y"] - last["y"]) < _EPS:
                continue
            path.append(point)
    return path


def _canonical_comparison_length(
    scene_context: dict[str, Any], *, is_object_query: bool
) -> float | None:
    comparison = loader.load_current_route_comparison(scene_context)
    if is_object_query:
        data = (
            comparison.get("artifacts", {})
            .get("cross_floor_object_real_route", {})
            .get("data")
        )
        if isinstance(data, dict):
            return (data.get("route_metrics") or {}).get(
                "combined_interface_route_length_m"
            )
    data = (
        comparison.get("artifacts", {})
        .get("cross_floor_room_real_route", {})
        .get("data")
    )
    if isinstance(data, dict):
        return (data.get("route_metrics") or {}).get("interface_route_length_m")
    return None


def stitch_metric_path(
    scene_context: dict[str, Any],
    query_task: dict[str, Any],
    target_grounding: dict[str, Any],
    semantic_route: dict[str, Any],
    connector_route: dict[str, Any] | None,
    approach_result: dict[str, Any] | None,
    *,
    allow_canonical_comparison: bool = True,
    no_canonical_write: bool = True,
) -> dict[str, Any]:
    """Stitch a semantic route into a dynamic metric path.

    Returns ``{"metric_path", "route_segments", "stitching_report", "scope"}``.
    ``metric_path.path`` is a concatenated ``[{x, y}, ...]`` polyline whose length
    is computed from A* geometry, not copied from any canonical route.
    """

    query_type = query_task.get("query_type")
    is_object_query = query_type in OBJECT_QUERY_TYPES
    scope = route_scope(query_task)

    anchors = loader.load_route_anchor_points(scene_context)["anchors"]

    def anchor_xy(room_id: str | None) -> list[float] | None:
        record = anchors.get(room_id) if room_id else None
        return list(record["anchor_xy"]) if record else None

    def anchor_floor(room_id: str | None, default: str | None) -> str | None:
        record = anchors.get(room_id) if room_id else None
        return (record.get("floor_id") if record else None) or default

    planners = _FloorPlannerCache(scene_context)
    segments: list[dict[str, Any]] = []
    notes: list[str] = []

    rooms: list[str] = list(semantic_route.get("room_sequence") or [])
    floors: list[str] = list(semantic_route.get("floor_sequence") or [])
    connector_id = (connector_route or {}).get("connector_id")

    # object_to_path is a local object-approach path only; no structure route.
    local_only = scope["metric_path_scope"] == "local_object_approach"

    connector_geom: dict[str, Any] | None = None
    if connector_id and not local_only:
        geom = loader.load_connector_endpoint_geometry(scene_context, connector_id)
        connector_geom = geom
        if geom.get("status") != "ok":
            notes.append(
                f"connector {connector_id} endpoint geometry incomplete; using "
                "documented centralized fallback"
            )

    seg_index = 0
    if not local_only:
        for i in range(len(rooms) - 1):
            from_room, to_room = rooms[i], rooms[i + 1]
            from_floor = floors[i] if i < len(floors) else None
            to_floor = floors[i + 1] if i + 1 < len(floors) else None
            from_xy = anchor_xy(from_room)
            to_xy = anchor_xy(to_room)
            if from_floor and to_floor and from_floor != to_floor:
                entry = (connector_geom or {}).get("entry_xy")
                exit_ = (connector_geom or {}).get("exit_xy")
                segments.append(
                    _same_floor_segment(
                        planners,
                        index=seg_index,
                        floor_id=from_floor,
                        from_id=from_room,
                        to_id=f"{connector_id or 'vt'}_entry",
                        from_xy=from_xy,
                        to_xy=_to_xy(entry),
                        from_room_id=from_room,
                        to_room_id=None,
                    )
                )
                seg_index += 1
                segments.append(
                    _connector_handoff_segment(
                        index=seg_index,
                        connector_geom=connector_geom
                        or {"connector_id": connector_id},
                    )
                )
                seg_index += 1
                segments.append(
                    _same_floor_segment(
                        planners,
                        index=seg_index,
                        floor_id=to_floor,
                        from_id=f"{connector_id or 'vt'}_exit",
                        to_id=to_room,
                        from_xy=_to_xy(exit_),
                        to_xy=to_xy,
                        from_room_id=None,
                        to_room_id=to_room,
                    )
                )
                seg_index += 1
            else:
                segments.append(
                    _same_floor_segment(
                        planners,
                        index=seg_index,
                        floor_id=from_floor,
                        from_id=from_room,
                        to_id=to_room,
                        from_xy=from_xy,
                        to_xy=to_xy,
                        from_room_id=from_room,
                        to_room_id=to_room,
                    )
                )
                seg_index += 1

    # Final object approach (object queries, including local_only).
    if is_object_query and approach_result is not None:
        target_room = (
            target_grounding.get("target_room_id")
            or (query_task.get("target") or {}).get("target_room_id")
        )
        approach_floor = anchor_floor(
            target_room, (query_task.get("target") or {}).get("target_floor_id")
        )
        segments.append(
            _object_approach_segment(
                planners,
                index=seg_index,
                floor_id=approach_floor,
                from_room_id=target_room,
                from_xy=anchor_xy(target_room),
                approach_result=approach_result,
            )
        )
        seg_index += 1

    # --- assemble stitched metric path ----------------------------------
    found_segments = [s for s in segments if s.get("path_found")]
    stitched_path = _concat_waypoints(found_segments)
    path_length = sum(
        float(s["segment_length"])
        for s in found_segments
        if s.get("segment_length") is not None
    )
    path_found = bool(found_segments) and all(s.get("path_found") for s in segments)

    endpoint = stitched_path[-1] if stitched_path else None
    endpoint_xy = [endpoint["x"], endpoint["y"]] if endpoint else None

    path_source = "dynamic_local_approach" if local_only else "dynamic_stitched"

    same_floor_count = sum(
        1 for s in segments if s["segment_type"] == "same_floor_metric"
    )
    connector_count = sum(
        1 for s in segments if s["segment_type"] == "connector_handoff"
    )
    approach_count = sum(
        1 for s in segments if s["segment_type"] == "object_approach_metric"
    )
    fallback_count = sum(1 for s in segments if s["segment_type"] == "semantic_only")
    dynamic_count = len(segments) - fallback_count

    comparison_length = (
        _canonical_comparison_length(scene_context, is_object_query=is_object_query)
        if allow_canonical_comparison
        else None
    )
    length_delta = (
        round(path_length - float(comparison_length), 6)
        if isinstance(comparison_length, (int, float))
        else None
    )

    for floor_id, message in planners.errors.items():
        notes.append(message)
    if fallback_count:
        notes.append(
            f"{fallback_count} segment(s) degraded to semantic_only; see per-segment "
            "fallback_reason"
        )
    if not notes:
        notes.append(
            "all metric segments planned dynamically over canonical stable maps"
        )

    metric_path = {
        "path": stitched_path,
        "path_length": round(path_length, 6) if path_found or found_segments else None,
        "path_found": path_found,
        "endpoint": endpoint_xy,
        "path_source": path_source,
        "metric_path_scope": scope["metric_path_scope"],
    }
    stitching_report = {
        "same_floor_segment_count": same_floor_count,
        "connector_segment_count": connector_count,
        "object_approach_segment_count": approach_count,
        "total_waypoint_count": len(stitched_path),
        "dynamic_segment_count": dynamic_count,
        "fallback_segment_count": fallback_count,
        "canonical_comparison_length_if_available": comparison_length,
        "length_delta_vs_canonical_if_available": length_delta,
        "canonical_comparison_role": "comparison_provenance_only",
        "no_canonical_write": no_canonical_write,
        "notes": notes,
    }
    return {
        "metric_path": metric_path,
        "route_segments": segments,
        "stitching_report": stitching_report,
        "scope": scope,
    }
