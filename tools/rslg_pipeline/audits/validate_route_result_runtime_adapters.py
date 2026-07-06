#!/usr/bin/env python3
"""Static validator for RSLGRouteResult-derived Layer 4 adapter inputs.

Validates the PID follower runtime input, RViz marker input, and z-aware overlay
input JSONs produced by
``tools/rslg_pipeline/export_route_result_runtime_inputs.py``. This is a static,
lightweight check; it never launches ROS/Gazebo/RViz and computes no paper tables.

Enforced rules (RSLG-SLAM project truth):

* ``schema_name`` is one of the new RouteResult-derived adapter schemas.
* The source RouteResult identity (query_id) is present.
* ``requires_nav2`` / ``requires_amcl`` are false.
* No ``map_server`` / ``nav2_map_server`` active runtime dependency.
* ``generated_ring_037`` is never a selected/used runtime goal; it may appear only
  as a rejected / blocked / evidence marker.
* ``vt_1_centerline_e003`` (the non-transition edge) is never used as a transition
  edge.
* ``vt_1_centerline_e001`` is the only transition edge when one is needed.
* ``floor_1`` z=0.0 and ``floor_2`` z=1.6 are visualization-only.
* Connector handoff is ``semantic_handoff`` / ``visualization_only`` /
  ``not_physical_stair_climbing``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_ID,
    NON_TRANSITION_EDGE,
    OBJECT_FLOOR,
    TRUE_TRANSITION_EDGE,
)

PID_SCHEMA = "rslg_route_result_pid_runtime_input"
MARKER_SCHEMA = "rslg_route_result_rviz_marker_input"
OVERLAY_SCHEMA = "rslg_route_result_z_aware_overlay_input"
ADAPTER_SCHEMAS = {PID_SCHEMA, MARKER_SCHEMA, OVERLAY_SCHEMA}

FORBIDDEN_RUNTIME_TOKENS = ("nav2_map_server", "map_server")
CONNECTOR_SEGMENT_TYPES = {"vertical_transition", "connector_handoff"}
EXECUTABLE_SEGMENT_TYPES = {
    "same_floor",
    "same_floor_metric",
    "object_approach",
    "object_approach_metric",
}
FLOOR_Z_BY_ID = {"floor_1": 0.0, "floor_2": 1.6}


def _read_source_route_result(data: dict[str, Any], adapter_path: Path) -> Optional[dict[str, Any]]:
    source = data.get("source_route_result")
    if not source:
        return None
    source_path = Path(str(source))
    candidates = [source_path] if source_path.is_absolute() else [
        Path.cwd() / source_path,
        adapter_path.parent / source_path,
    ]
    for candidate in candidates:
        try:
            if candidate.exists():
                return json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
    return None


def _metric_path_scope_from_marker(data: dict[str, Any]) -> Optional[str]:
    for marker in data.get("markers") or []:
        meta = marker.get("metadata") or {}
        if meta.get("metric_path_scope"):
            return str(meta["metric_path_scope"])
    return None


def _selected_goal_candidate_id(data: dict[str, Any]) -> Optional[str]:
    for key in ("selected_goal", "selected_approach", "endpoint"):
        record = data.get(key)
        if isinstance(record, dict) and record.get("candidate_id"):
            return str(record["candidate_id"])
    for marker in data.get("markers") or []:
        meta = marker.get("metadata") or {}
        if (
            meta.get("visualization_role") == "selected_object_approach_goal"
            and meta.get("candidate_id")
        ):
            return str(meta["candidate_id"])
    final = data.get("final_waypoint")
    if isinstance(final, dict) and final.get("runtime_goal_candidate_id"):
        return str(final["runtime_goal_candidate_id"])
    for wp in data.get("overlay_waypoints") or []:
        if wp.get("runtime_goal_candidate_id"):
            return str(wp["runtime_goal_candidate_id"])
    return None


def _source_context(data: dict[str, Any], adapter_path: Path) -> dict[str, Any]:
    route_result = _read_source_route_result(data, adapter_path)
    identity = data.get("identity") or {}
    floor_z_map = data.get("floor_z_map") or {}
    context: dict[str, Any] = {
        "source_route_result_loaded": route_result is not None,
        "query_type": identity.get("query_type") or data.get("route_type"),
        "metric_path_scope": data.get("metric_path_scope") or _metric_path_scope_from_marker(data),
        "target_floor_id": None,
        "target_floor_z": None,
        "actual_connector_segment_count": 0,
        "actual_connector_transition_edges": [],
        "has_floor_1_metric_segment": False,
    }

    selected_candidate = _selected_goal_candidate_id(data)
    if route_result:
        rr_identity = route_result.get("identity") or {}
        context["query_type"] = rr_identity.get("query_type") or context["query_type"]
        context["metric_path_scope"] = (
            (route_result.get("metric_path") or {}).get("metric_path_scope")
            or context["metric_path_scope"]
        )
        target = route_result.get("target") or {}
        context["target_floor_id"] = (
            target.get("target_floor_id")
            or target.get("floor_id")
            or context["target_floor_id"]
        )
        route_segments = [
            seg for seg in route_result.get("route_segments") or [] if isinstance(seg, dict)
        ]
        connector_segments = [
            seg for seg in route_segments if seg.get("segment_type") in CONNECTOR_SEGMENT_TYPES
        ]
        context["actual_connector_segment_count"] = len(connector_segments)
        context["actual_connector_transition_edges"] = [
            str(edge)
            for seg in connector_segments
            for edge in (seg.get("transition_edge") or seg.get("connector_edge"),)
            if edge
        ]
        context["has_floor_1_metric_segment"] = any(
            seg.get("segment_type") in EXECUTABLE_SEGMENT_TYPES
            and seg.get("floor_id") == "floor_1"
            for seg in route_segments
        )

    if not context["target_floor_id"]:
        for key in ("selected_goal", "selected_approach", "endpoint"):
            record = data.get(key)
            if isinstance(record, dict) and record.get("floor_id"):
                context["target_floor_id"] = str(record["floor_id"])
                break
    if not context["target_floor_id"] and (
        context["query_type"] == "object_to_path"
        or context["metric_path_scope"] == "local_object_approach"
        or selected_candidate == CURRENT_OBJECT_APPROACH_ID
    ):
        context["target_floor_id"] = OBJECT_FLOOR

    target_floor_id = context["target_floor_id"]
    if target_floor_id:
        context["target_floor_z"] = float(
            floor_z_map.get(target_floor_id, FLOOR_Z_BY_ID.get(str(target_floor_id), 0.0))
        )
    return context


def _check_common(data: dict[str, Any], errors: list[str]) -> None:
    schema_name = data.get("schema_name")
    if schema_name not in ADAPTER_SCHEMAS:
        errors.append(f"schema_name {schema_name!r} is not a RouteResult-derived adapter schema")
    identity = data.get("identity") or {}
    if not identity.get("query_id"):
        errors.append("identity.query_id (source RouteResult identity) is required")
    if not data.get("source_route_result"):
        errors.append("source_route_result is required")
    if data.get("requires_nav2") is not False:
        errors.append("requires_nav2 must be false")
    if data.get("requires_amcl") is not False:
        errors.append("requires_amcl must be false")

    claim = data.get("claim_boundary") or {}
    if claim.get("no_nav2_dependency") is not True:
        errors.append("claim_boundary.no_nav2_dependency must be true")
    if claim.get("no_amcl_dependency") is not True:
        errors.append("claim_boundary.no_amcl_dependency must be true")
    if claim.get("no_map_server_dependency") is not True:
        errors.append("claim_boundary.no_map_server_dependency must be true")
    if claim.get("no_physical_stair_climbing_claim") is not True:
        errors.append("claim_boundary.no_physical_stair_climbing_claim must be true")

    floor_z_map = data.get("floor_z_map") or {}
    if "floor_1" in floor_z_map and float(floor_z_map["floor_1"]) != 0.0:
        errors.append("floor_z_map.floor_1 must be 0.0 (visualization-only)")
    if "floor_2" in floor_z_map and float(floor_z_map["floor_2"]) != 1.6:
        errors.append("floor_z_map.floor_2 must be 1.6 (visualization-only)")


def _handoff_records(data: dict[str, Any]) -> list[dict[str, Any]]:
    schema_name = data.get("schema_name")
    if schema_name in {PID_SCHEMA, OVERLAY_SCHEMA}:
        return [h for h in data.get("connector_handoffs") or [] if isinstance(h, dict)]
    if schema_name == MARKER_SCHEMA:
        handoffs: list[dict[str, Any]] = []
        for marker in data.get("markers") or []:
            meta = marker.get("metadata") or {}
            if meta.get("visualization_role") == "connector_handoff":
                handoffs.append(dict(meta))
        return handoffs
    return []


def _transition_edges_in_adapter(data: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    for handoff in _handoff_records(data):
        if handoff.get("transition_edge"):
            edges.append(str(handoff["transition_edge"]))
    if data.get("schema_name") == OVERLAY_SCHEMA and data.get("transition_edge_used"):
        edges.append(str(data["transition_edge_used"]))
    connector_handoff = data.get("connector_handoff")
    if isinstance(connector_handoff, dict) and connector_handoff.get("transition_edge"):
        edges.append(str(connector_handoff["transition_edge"]))
    return edges


def _z_values_in_overlay(data: dict[str, Any]) -> list[float]:
    z_values: list[float] = []
    for wp in data.get("overlay_waypoints") or []:
        if isinstance(wp, dict) and wp.get("z") is not None:
            z_values.append(float(wp["z"]))
    final = data.get("final_waypoint")
    if isinstance(final, dict) and final.get("z") is not None:
        z_values.append(float(final["z"]))
    for handoff in data.get("connector_handoffs") or []:
        if isinstance(handoff, dict):
            for key in ("from_z", "to_z"):
                if handoff.get(key) is not None:
                    z_values.append(float(handoff[key]))
    return z_values


def _check_transition_edges_against_source(
    data: dict[str, Any], context: dict[str, Any], errors: list[str]
) -> None:
    edges = _transition_edges_in_adapter(data)
    if TRUE_TRANSITION_EDGE in edges:
        if not context["source_route_result_loaded"]:
            errors.append(
                f"{TRUE_TRANSITION_EDGE!r} transition edge cannot be verified without source_route_result"
            )
        elif int(context["actual_connector_segment_count"]) == 0:
            errors.append(
                f"{TRUE_TRANSITION_EDGE!r} may be emitted only when the source RouteResult "
                "has an actual connector_handoff route segment"
            )


def _check_local_object_scope(
    data: dict[str, Any], context: dict[str, Any], errors: list[str]
) -> None:
    query_type = context.get("query_type")
    metric_path_scope = context.get("metric_path_scope")
    is_object_to_path = query_type == "object_to_path"
    is_local_object_approach = metric_path_scope == "local_object_approach"
    if not (is_object_to_path or is_local_object_approach):
        return

    handoffs = _handoff_records(data)
    if handoffs:
        errors.append(
            "object_to_path/local_object_approach adapter output must not emit connector handoffs"
        )
    if any(h.get("handoff_type") == "semantic_handoff" for h in handoffs):
        errors.append("local_object_approach must not emit a semantic_handoff connector")
    if _transition_edges_in_adapter(data):
        errors.append("object_to_path/local_object_approach must not emit transition edges")

    schema_name = data.get("schema_name")
    target_z = context.get("target_floor_z")
    if schema_name == PID_SCHEMA:
        for segment in data.get("runtime_segments") or []:
            if segment.get("segment_type") in CONNECTOR_SEGMENT_TYPES:
                errors.append("object_to_path runtime_segments must not include connector segments")
            if target_z is not None and segment.get("z") is not None:
                if abs(float(segment["z"]) - float(target_z)) > 1e-6:
                    errors.append("object_to_path runtime segment z must stay on the target floor")
    if schema_name == MARKER_SCHEMA:
        connector_markers = [
            marker
            for marker in data.get("markers") or []
            if (marker.get("metadata") or {}).get("visualization_role") == "connector_handoff"
        ]
        if connector_markers:
            errors.append("object_to_path RViz marker input must not include connector handoff markers")
    if schema_name == OVERLAY_SCHEMA:
        if data.get("z_aware_visual_transition_detected") is not False:
            errors.append("object_to_path/local_object_approach z overlay transition must be false")
        if target_z is not None:
            for key in ("z_min", "z_max"):
                if data.get(key) is None or abs(float(data[key]) - float(target_z)) > 1e-6:
                    errors.append(f"object_to_path/local_object_approach {key} must equal target floor z")
        has_floor1_z = any(abs(value - FLOOR_Z_BY_ID["floor_1"]) < 1e-6 for value in _z_values_in_overlay(data))
        if has_floor1_z and not context.get("has_floor_1_metric_segment"):
            errors.append("z-aware overlay includes floor_1 z without a source floor_1 metric segment")


def _check_object_target_goal(data: dict[str, Any], context: dict[str, Any], errors: list[str]) -> None:
    if context.get("query_type") != "object_to_path":
        return
    candidate_id = _selected_goal_candidate_id(data)
    if candidate_id and candidate_id != CURRENT_OBJECT_APPROACH_ID:
        errors.append(
            f"object_to_path selected runtime goal must be {CURRENT_OBJECT_APPROACH_ID!r}, got {candidate_id!r}"
        )


def _check_no_forbidden_dependency(data: dict[str, Any], errors: list[str]) -> None:
    """Forbid an *active* map_server / nav2_map_server runtime dependency.

    An explicit ``requires_*`` false flag is allowed; anything that asserts a
    map_server-style requirement true is rejected.
    """

    def scan(obj: Any) -> None:
        if isinstance(obj, dict):
            for key, value in obj.items():
                lkey = str(key).lower()
                if any(token in lkey for token in FORBIDDEN_RUNTIME_TOKENS):
                    if lkey.startswith("no_"):
                        # A "no_*" claim-boundary flag asserts absence; must be true.
                        if value not in (True, None):
                            errors.append(f"{key!r} asserts absence but is {value!r}")
                    elif lkey.startswith("requires_") or "required" in lkey:
                        # A "requires_*" flag must never be a positive dependency.
                        if value not in (False, None):
                            errors.append(f"forbidden active dependency flag {key!r}={value!r}")
                    else:
                        errors.append(f"forbidden map_server-style key {key!r}")
                scan(value)
        elif isinstance(obj, list):
            for item in obj:
                scan(item)

    scan(data)


def _check_transition_edges(edges: Iterable[str], errors: list[str]) -> None:
    for edge in edges:
        if edge == NON_TRANSITION_EDGE:
            errors.append(f"non-transition edge {NON_TRANSITION_EDGE!r} used as a transition edge")
        elif edge and edge != TRUE_TRANSITION_EDGE:
            errors.append(f"unexpected transition edge {edge!r}; only {TRUE_TRANSITION_EDGE!r} allowed")


def _check_connector_handoffs(handoffs: list[dict[str, Any]], errors: list[str]) -> None:
    for handoff in handoffs or []:
        if not isinstance(handoff, dict):
            continue
        if handoff.get("handoff_type") != "semantic_handoff":
            errors.append("connector handoff must be handoff_type=semantic_handoff")
        if handoff.get("visualization_only") is not True:
            errors.append("connector handoff must be visualization_only=true")
        if handoff.get("not_physical_stair_climbing") is not True:
            errors.append("connector handoff must be not_physical_stair_climbing=true")


def _check_ring_037(data: dict[str, Any], errors: list[str]) -> None:
    """``generated_ring_037`` may appear only as rejected/blocked evidence."""

    for blocked_id in BLOCKED_LEGACY_APPROACH_IDS:
        # Selected goal must never be the blocked candidate.
        for key in ("selected_goal", "selected_approach", "endpoint"):
            record = data.get(key)
            if isinstance(record, dict) and record.get("candidate_id") == blocked_id:
                errors.append(f"{blocked_id!r} must never be the {key}")
        # Any marker/record that uses it as runtime goal is forbidden.
        for marker in data.get("markers") or []:
            meta = marker.get("metadata") or {}
            if meta.get("candidate_id") == blocked_id and meta.get("used_as_runtime_goal") is True:
                errors.append(f"{blocked_id!r} marker must not be used_as_runtime_goal")
        for wp in data.get("overlay_waypoints") or []:
            if wp.get("runtime_goal_candidate_id") == blocked_id:
                errors.append(f"{blocked_id!r} must never be an overlay runtime goal")
        for wp in data.get("runtime_waypoints") or []:
            if wp.get("runtime_goal_candidate_id") == blocked_id:
                errors.append(f"{blocked_id!r} must never be a runtime waypoint goal")


def _validate_pid(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("requires_nav2") is not False:
        errors.append("requires_nav2 must be false")
    if data.get("requires_amcl") is not False:
        errors.append("requires_amcl must be false")
    policy = data.get("runtime_policy") or {}
    if policy.get("requires_map_server") not in (False, None):
        errors.append("runtime_policy.requires_map_server must be false")
    if policy.get("requires_nav2_map_server") not in (False, None):
        errors.append("runtime_policy.requires_nav2_map_server must be false")
    _check_transition_edges(
        (h.get("transition_edge") for h in data.get("connector_handoffs") or []), errors
    )
    _check_connector_handoffs(data.get("connector_handoffs") or [], errors)
    _check_ring_037(data, errors)
    return errors


def _validate_marker(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    edges: list[str] = []
    handoffs: list[dict[str, Any]] = []
    for marker in data.get("markers") or []:
        meta = marker.get("metadata") or {}
        if meta.get("visualization_role") == "connector_handoff":
            handoffs.append({**meta, "handoff_type": meta.get("handoff_type")})
            if meta.get("transition_edge"):
                edges.append(str(meta["transition_edge"]))
    _check_transition_edges(edges, errors)
    _check_connector_handoffs(handoffs, errors)
    _check_ring_037(data, errors)
    return errors


def _validate_overlay(data: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if data.get("visualization_only") is not True and data.get("visualization_overlay") is not True:
        errors.append("overlay must be visualization_only/visualization_overlay=true")
    if data.get("physical_climb_claimed") is not False:
        errors.append("overlay physical_climb_claimed must be false")
    edge = data.get("transition_edge_used")
    _check_transition_edges([edge] if edge else [], errors)
    _check_connector_handoffs(data.get("connector_handoffs") or [], errors)
    _check_ring_037(data, errors)
    return errors


def validate_adapter_file(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"path": path.as_posix(), "ok": False, "schema_name": None, "errors": [f"read/parse: {exc}"]}

    errors: list[str] = []
    context = _source_context(data, path)
    if data.get("source_route_result") and not context["source_route_result_loaded"]:
        errors.append("source_route_result could not be loaded for adapter/source consistency checks")
    _check_common(data, errors)
    _check_no_forbidden_dependency(data, errors)
    _check_transition_edges_against_source(data, context, errors)
    _check_local_object_scope(data, context, errors)
    _check_object_target_goal(data, context, errors)

    schema_name = data.get("schema_name")
    if schema_name == PID_SCHEMA:
        errors.extend(_validate_pid(data))
    elif schema_name == MARKER_SCHEMA:
        errors.extend(_validate_marker(data))
    elif schema_name == OVERLAY_SCHEMA:
        errors.extend(_validate_overlay(data))

    return {
        "path": path.as_posix(),
        "ok": not errors,
        "schema_name": schema_name,
        "query_id": (data.get("identity") or {}).get("query_id"),
        "errors": errors,
    }


def _collect_files(adapter_root: Path) -> list[Path]:
    files = sorted(adapter_root.glob("**/*.json"))
    return [f for f in files if not f.name.startswith("00_") and not f.name.startswith("01_")]


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--adapter-root", type=Path, default=None)
    group.add_argument("--adapter-json", type=Path, default=None)
    args = parser.parse_args(argv)

    if args.adapter_json is not None:
        paths = [args.adapter_json]
    else:
        paths = _collect_files(args.adapter_root)

    results = [validate_adapter_file(p) for p in paths]
    ok = bool(results) and all(item["ok"] for item in results)
    by_schema: dict[str, int] = {}
    for item in results:
        by_schema[item["schema_name"]] = by_schema.get(item["schema_name"], 0) + 1
    report = {
        "schema_name": "rslg_route_result_runtime_adapter_validation",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "classification": "route_result_runtime_adapter_validation_passed"
        if ok
        else "route_result_runtime_adapter_validation_failed",
        "ok": ok,
        "files_validated": len(results),
        "counts_by_schema": by_schema,
        "results": results,
    }
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
