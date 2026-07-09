#!/usr/bin/env python3
"""Build a task59 multi-floor ramp-surrogate runtime input for RSLG-SLAM.

The adapter consumes the already generated task56c RouteResult/PID runtime
input, preserves the route order, and replaces the semantic vertical connector
handoff with an executable Gazebo ramp-surrogate waypoint segment. It does not
run Stage-A, RGB-D inference, Nav2, AMCL, map_server, or physical robot code.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
SCENE_ID = "00843-DYehNKdT76V"
PROJECT_NAME = "RSLG-SLAM"
TASK_NAME = "task59_multifloor_ramp_gazebo_validation"
DEFAULT_TASK_DIR = (
    REPO_ROOT
    / "stage_outputs"
    / "rslg_slam"
    / SCENE_ID
    / "tasks"
    / TASK_NAME
)
TASK56C_ROOT = (
    REPO_ROOT
    / "stage_outputs"
    / "rslg_slam"
    / SCENE_ID
    / "tasks"
    / "task56c_pid_profile_promotion_and_regression_validation"
    / "regression_pack"
)
DEFAULT_QUERY_ID = "00843_cross_floor_object_curtain_room14"
FALLBACK_QUERY_ID = "00843_cross_floor_room_room2_to_room14"
DEFAULT_PROFILE_ID = "practical_zero_collision"
TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
FORBIDDEN_NON_TRANSITION_EDGE = "vt_1_centerline_e003"
SELECTED_APPROACH_ID = "generated_ring_002"
BLOCKED_CANDIDATE_ID = "generated_ring_037"
FLOOR_1_Z = 0.0
FLOOR_2_Z = 1.6
DEFAULT_RAMP_WAYPOINT_SPACING = 0.15
RAMP_SEGMENT_ID = f"ramp_surrogate_{TRUE_TRANSITION_EDGE}"
PLANNED_TOPIC = "/rslg/multifloor_planned_path_odom"
EXECUTED_TOPIC = "/rslg/multifloor_gazebo_executed_path"
ROBOT_POSE_TOPIC = "/rslg/multifloor_gazebo_robot_pose"
MARKER_TOPIC = "/rslg/multifloor_ramp_marker_array"


@dataclass(frozen=True)
class RoutePaths:
    route_result: Path
    pid_input: Path
    rviz_marker_input: Path
    z_aware_overlay_input: Path


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def route_paths(query_id: str) -> RoutePaths:
    return RoutePaths(
        route_result=TASK56C_ROOT / "route_results" / f"{query_id}_route_result.json",
        pid_input=TASK56C_ROOT / "runtime_adapter_inputs" / "pid_follower_inputs" / f"{query_id}_pid_runtime_input.json",
        rviz_marker_input=TASK56C_ROOT
        / "runtime_adapter_inputs"
        / "rviz_marker_inputs"
        / f"{query_id}_rviz_marker_input.json",
        z_aware_overlay_input=TASK56C_ROOT
        / "runtime_adapter_inputs"
        / "z_aware_overlay_inputs"
        / f"{query_id}_z_aware_overlay_input.json",
    )


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_between(a: Point3, b: Point3) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def distance_xy(a: Point3, b: Point3) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def candidate_text_counts(route_result: dict[str, Any], pid_input: dict[str, Any]) -> dict[str, int]:
    text = json.dumps(route_result, sort_keys=True) + json.dumps(pid_input, sort_keys=True)
    return {
        TRUE_TRANSITION_EDGE: text.count(TRUE_TRANSITION_EDGE),
        FORBIDDEN_NON_TRANSITION_EDGE: text.count(FORBIDDEN_NON_TRANSITION_EDGE),
        SELECTED_APPROACH_ID: text.count(SELECTED_APPROACH_ID),
        BLOCKED_CANDIDATE_ID: text.count(BLOCKED_CANDIDATE_ID),
    }


def floor_counts(waypoints: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for waypoint in waypoints:
        floor_id = str(waypoint.get("floor_id") or "")
        counts[floor_id] = counts.get(floor_id, 0) + 1
    return dict(sorted(counts.items()))


def waypoint_indices(waypoints: list[dict[str, Any]], floor_id: str) -> tuple[int | None, int | None]:
    indices = [int(wp.get("waypoint_index", index)) for index, wp in enumerate(waypoints) if wp.get("floor_id") == floor_id]
    if not indices:
        return None, None
    return min(indices), max(indices)


def selected_goal_id(pid_input: dict[str, Any]) -> str | None:
    for key in ("selected_goal", "selected_approach", "endpoint"):
        value = pid_input.get(key)
        if isinstance(value, dict) and value.get("candidate_id"):
            return str(value.get("candidate_id"))
    return None


def transition_edges(route_result: dict[str, Any], pid_input: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    semantic_route = route_result.get("semantic_route") or {}
    for connector in semantic_route.get("connector_sequence") or []:
        if isinstance(connector, dict) and connector.get("transition_edge"):
            edges.append(str(connector.get("transition_edge")))
    for handoff in pid_input.get("connector_handoffs") or []:
        if isinstance(handoff, dict) and handoff.get("transition_edge"):
            edges.append(str(handoff.get("transition_edge")))
    return edges


def rejected_candidate_ids(route_result: dict[str, Any], pid_input: dict[str, Any]) -> list[str]:
    ids: list[str] = []
    approach = route_result.get("approach") or {}
    for candidate in approach.get("rejected_candidates") or []:
        if isinstance(candidate, dict) and candidate.get("candidate_id"):
            ids.append(str(candidate.get("candidate_id")))
    for candidate in pid_input.get("rejected_runtime_candidates") or []:
        if isinstance(candidate, dict) and candidate.get("candidate_id"):
            ids.append(str(candidate.get("candidate_id")))
    return sorted(set(ids))


def blocked_candidate_selected(pid_input: dict[str, Any]) -> bool:
    return selected_goal_id(pid_input) == BLOCKED_CANDIDATE_ID


def assess_query(query_id: str) -> dict[str, Any]:
    paths = route_paths(query_id)
    exists = {
        "route_result": paths.route_result.is_file(),
        "pid_input": paths.pid_input.is_file(),
        "rviz_marker_input": paths.rviz_marker_input.is_file(),
        "z_aware_overlay_input": paths.z_aware_overlay_input.is_file(),
    }
    if not exists["route_result"] or not exists["pid_input"]:
        return {
            "query_id": query_id,
            "available": False,
            "exists": exists,
            "suitable": False,
            "reasons": ["route_result or pid_input is missing"],
        }

    route_result = read_json(paths.route_result)
    pid_input = read_json(paths.pid_input)
    waypoints = pid_input.get("runtime_waypoints") or []
    floors = floor_counts(waypoints)
    edges = transition_edges(route_result, pid_input)
    goal_id = selected_goal_id(pid_input)
    query_type = str((pid_input.get("identity") or {}).get("query_type") or "")
    validation = route_result.get("validation") or {}
    rejected_ids = rejected_candidate_ids(route_result, pid_input)
    text_counts = candidate_text_counts(route_result, pid_input)
    has_floor_1 = floors.get("floor_1", 0) > 0
    has_floor_2 = floors.get("floor_2", 0) > 0
    uses_true_edge = TRUE_TRANSITION_EDGE in edges
    e003_as_transition = FORBIDDEN_NON_TRANSITION_EDGE in edges
    ring037_selected = blocked_candidate_selected(pid_input)
    object_query = query_type == "cross_floor_object"
    object_approach_ok = (goal_id == SELECTED_APPROACH_ID) if object_query else True
    route_feasible = bool(validation.get("route_feasible"))
    suitable = (
        has_floor_1
        and has_floor_2
        and uses_true_edge
        and not e003_as_transition
        and not ring037_selected
        and object_approach_ok
        and route_feasible
    )
    reasons: list[str] = []
    if not has_floor_1:
        reasons.append("floor_1 has no runtime waypoints")
    if not has_floor_2:
        reasons.append("floor_2 has no runtime waypoints")
    if not uses_true_edge:
        reasons.append(f"{TRUE_TRANSITION_EDGE} is not a transition edge")
    if e003_as_transition:
        reasons.append(f"{FORBIDDEN_NON_TRANSITION_EDGE} appears as a transition edge")
    if ring037_selected:
        reasons.append(f"{BLOCKED_CANDIDATE_ID} is selected as the runtime goal")
    if not object_approach_ok:
        reasons.append(f"object query does not select {SELECTED_APPROACH_ID}")
    if not route_feasible:
        reasons.append("RouteResult validation does not mark route_feasible true")
    if not reasons:
        reasons.append("all task59 route-selection checks passed")
    first_floor_1, last_floor_1 = waypoint_indices(waypoints, "floor_1")
    first_floor_2, last_floor_2 = waypoint_indices(waypoints, "floor_2")
    return {
        "query_id": query_id,
        "available": True,
        "exists": exists,
        "suitable": suitable,
        "reasons": reasons,
        "query_type": query_type,
        "route_feasible": route_feasible,
        "floor_counts": floors,
        "floor_sequence": (route_result.get("semantic_route") or {}).get("floor_sequence") or [],
        "room_sequence": (route_result.get("semantic_route") or {}).get("room_sequence") or [],
        "connector_sequence": (route_result.get("semantic_route") or {}).get("connector_sequence") or [],
        "transition_edges": edges,
        "uses_true_transition_edge": uses_true_edge,
        "e003_as_transition": e003_as_transition,
        "selected_goal_candidate_id": goal_id,
        "generated_ring_002_final_approach": goal_id == SELECTED_APPROACH_ID,
        "generated_ring_037_selected": ring037_selected,
        "generated_ring_037_rejected_or_evidence": BLOCKED_CANDIDATE_ID in rejected_ids or text_counts[BLOCKED_CANDIDATE_ID] > 0,
        "text_counts": text_counts,
        "runtime_waypoint_count": len(waypoints),
        "floor_1_segment": {
            "source_waypoint_index_start": first_floor_1,
            "source_waypoint_index_end": last_floor_1,
            "count": floors.get("floor_1", 0),
        },
        "connector_segment": {
            "transition_edge": TRUE_TRANSITION_EDGE if uses_true_edge else None,
            "forbidden_non_transition_edge": FORBIDDEN_NON_TRANSITION_EDGE,
            "handoffs": pid_input.get("connector_handoffs") or [],
        },
        "floor_2_segment": {
            "source_waypoint_index_start": first_floor_2,
            "source_waypoint_index_end": last_floor_2,
            "count": floors.get("floor_2", 0),
        },
    }


def choose_query(requested_query_id: str | None) -> tuple[str, dict[str, Any], list[dict[str, Any]], str]:
    candidates = [requested_query_id] if requested_query_id else [DEFAULT_QUERY_ID, FALLBACK_QUERY_ID]
    audited = [assess_query(query_id) for query_id in candidates if query_id]
    if requested_query_id:
        audit = audited[0]
        if not audit.get("suitable"):
            raise SystemExit(f"Requested query is not suitable for task59: {requested_query_id}: {audit.get('reasons')}")
        return requested_query_id, audit, audited, "explicitly requested query passed task59 checks"
    preferred = audited[0]
    if preferred.get("suitable"):
        return DEFAULT_QUERY_ID, preferred, audited, (
            f"{DEFAULT_QUERY_ID} is the preferred object-target route and passes all task59 checks"
        )
    fallback = assess_query(FALLBACK_QUERY_ID)
    audited.append(fallback)
    if fallback.get("suitable"):
        return FALLBACK_QUERY_ID, fallback, audited, (
            f"{DEFAULT_QUERY_ID} was not suitable ({'; '.join(preferred.get('reasons', []))}); "
            f"{FALLBACK_QUERY_ID} passed task59 checks"
        )
    raise SystemExit(
        "No suitable task59 cross-floor route was found: "
        + "; ".join(f"{a['query_id']}: {a.get('reasons')}" for a in audited)
    )


def split_floor_waypoints(pid_input: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    waypoints = [wp for wp in pid_input.get("runtime_waypoints") or [] if isinstance(wp, dict)]
    floor_1 = [wp for wp in waypoints if wp.get("floor_id") == "floor_1"]
    floor_2 = [wp for wp in waypoints if wp.get("floor_id") == "floor_2"]
    if not floor_1 or not floor_2:
        raise ValueError("Selected PID input must contain both floor_1 and floor_2 runtime waypoints")
    return floor_1, floor_2


def point_from_waypoint(waypoint: dict[str, Any]) -> Point3:
    return Point3(as_float(waypoint.get("x")), as_float(waypoint.get("y")), as_float(waypoint.get("z")))


def switchback_control_points(start: Point3, end: Point3) -> list[Point3]:
    dx = end.x - start.x
    dy = end.y - start.y
    run = max(math.hypot(dx, dy), 1e-6)
    unit_x = dx / run
    unit_y = dy / run
    perp_x = -unit_y
    perp_y = unit_x
    lateral = 1.85
    along = min(1.4, run * 0.35)
    return [
        Point3(start.x, start.y, FLOOR_1_Z),
        Point3(start.x + unit_x * along + perp_x * lateral, start.y + unit_y * along + perp_y * lateral, 0.4),
        Point3(end.x - unit_x * along - perp_x * lateral, end.y - unit_y * along - perp_y * lateral, 1.2),
        Point3(end.x, end.y, FLOOR_2_Z),
    ]


def segment_metric(a: Point3, b: Point3) -> dict[str, float]:
    run = distance_xy(a, b)
    rise = b.z - a.z
    length_3d = math.sqrt(run * run + rise * rise)
    slope_degrees = math.degrees(math.atan2(abs(rise), run)) if run > 0 else 90.0
    return {
        "start_x": round(a.x, 6),
        "start_y": round(a.y, 6),
        "start_z": round(a.z, 6),
        "end_x": round(b.x, 6),
        "end_y": round(b.y, 6),
        "end_z": round(b.z, 6),
        "xy_run_m": round(run, 6),
        "rise_m": round(rise, 6),
        "length_3d_m": round(length_3d, 6),
        "slope_degrees": round(slope_degrees, 6),
        "yaw": round(yaw_between(a, b), 6),
    }


def ramp_floor_id(z: float) -> str:
    if z <= FLOOR_1_Z + 0.05:
        return "floor_1"
    if z >= FLOOR_2_Z - 0.05:
        return "floor_2"
    return "ramp_surrogate"


def generate_ramp_waypoints(control_points: list[Point3], spacing: float) -> list[dict[str, Any]]:
    ramp_waypoints: list[dict[str, Any]] = []
    local_index = 0
    for segment_index in range(len(control_points) - 1):
        a = control_points[segment_index]
        b = control_points[segment_index + 1]
        run = distance_xy(a, b)
        yaw = yaw_between(a, b)
        steps = max(1, int(math.ceil(run / spacing)))
        for step in range(1, steps + 1):
            t = step / steps
            x = a.x + (b.x - a.x) * t
            y = a.y + (b.y - a.y) * t
            z = a.z + (b.z - a.z) * t
            ramp_waypoints.append(
                {
                    "source_index": None,
                    "x": round(x, 6),
                    "y": round(y, 6),
                    "z": round(z, 6),
                    "yaw": round(yaw, 6),
                    "floor_id": ramp_floor_id(z),
                    "ramp_segment_id": RAMP_SEGMENT_ID,
                    "semantic_source": "ramp_surrogate_connector",
                    "is_ramp_waypoint": True,
                    "source_edge_id": TRUE_TRANSITION_EDGE,
                    "forbidden_edge_id": None,
                    "ramp_local_index": local_index,
                    "ramp_polyline_segment_index": segment_index,
                }
            )
            local_index += 1
    return ramp_waypoints


def original_waypoint_record(raw: dict[str, Any], new_index: int) -> dict[str, Any]:
    return {
        "index": new_index,
        "source_index": int(raw.get("waypoint_index", raw.get("index", new_index))),
        "x": round(as_float(raw.get("x")), 6),
        "y": round(as_float(raw.get("y")), 6),
        "z": round(as_float(raw.get("z")), 6),
        "yaw": round(as_float(raw.get("yaw")), 6),
        "floor_id": str(raw.get("floor_id") or ""),
        "ramp_segment_id": None,
        "semantic_source": str(raw.get("source") or "route_result_segment_waypoint"),
        "segment_id": str(raw.get("segment_id") or ""),
        "is_ramp_waypoint": False,
        "source_edge_id": None,
    }


def reindex_waypoints(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for index, record in enumerate(records):
        item = dict(record)
        item["index"] = index
        item["waypoint_index"] = index
        if "segment_id" not in item:
            item["segment_id"] = item.get("ramp_segment_id") or ""
        if "source" not in item:
            item["source"] = item.get("semantic_source") or ""
        out.append(item)
    return out


def build_runtime_input(
    *,
    query_id: str,
    route_result: dict[str, Any],
    pid_input: dict[str, Any],
    paths: RoutePaths,
    route_audit: dict[str, Any],
    selection_reason: str,
    ramp_spacing: float,
) -> dict[str, Any]:
    floor_1, floor_2 = split_floor_waypoints(pid_input)
    last_floor_1 = floor_1[-1]
    first_floor_2 = floor_2[0]
    start = point_from_waypoint(last_floor_1)
    end = point_from_waypoint(first_floor_2)
    raw_metric = segment_metric(start, end)
    control_points = switchback_control_points(start, end)
    ramp_metrics = [segment_metric(control_points[index], control_points[index + 1]) for index in range(len(control_points) - 1)]
    ramp_waypoints = generate_ramp_waypoints(control_points, ramp_spacing)
    source_floor_1 = [original_waypoint_record(wp, index) for index, wp in enumerate(floor_1)]
    source_floor_2 = [original_waypoint_record(wp, index) for index, wp in enumerate(floor_2[1:], start=len(source_floor_1) + len(ramp_waypoints))]
    waypoint_records = reindex_waypoints(source_floor_1 + ramp_waypoints + source_floor_2)
    equivalent_run = sum(metric["xy_run_m"] for metric in ramp_metrics)
    equivalent_slope = math.degrees(math.atan2(FLOOR_2_Z - FLOOR_1_Z, equivalent_run))
    max_segment_slope = max(metric["slope_degrees"] for metric in ramp_metrics)
    handoff = (pid_input.get("connector_handoffs") or [{}])[0]
    selected = pid_input.get("selected_approach") or pid_input.get("selected_goal") or pid_input.get("endpoint") or {}
    return {
        "schema_name": "rslg_multifloor_ramp_runtime_input",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "scene_id": SCENE_ID,
        "source_query_id": query_id,
        "selected_query_reason": selection_reason,
        "selected_route_result_json": str(paths.route_result),
        "selected_pid_input_json": str(paths.pid_input),
        "selected_rviz_marker_input_json": str(paths.rviz_marker_input) if paths.rviz_marker_input.is_file() else None,
        "route_mode": "multifloor_ramp_surrogate",
        "profile_id": DEFAULT_PROFILE_ID,
        "ramp_connector_edge_id": TRUE_TRANSITION_EDGE,
        "forbidden_transition_edge_ids": [FORBIDDEN_NON_TRANSITION_EDGE],
        "blocked_goal_candidate_ids": [BLOCKED_CANDIDATE_ID],
        "target_approach_id": SELECTED_APPROACH_ID
        if selected.get("candidate_id") == SELECTED_APPROACH_ID or route_audit.get("query_type") == "cross_floor_object"
        else None,
        "selected_goal_candidate_id": selected.get("candidate_id"),
        "floor_1_z": FLOOR_1_Z,
        "floor_2_z": FLOOR_2_Z,
        "floor_z_map": {"floor_1": FLOOR_1_Z, "floor_2": FLOOR_2_Z},
        "ramp_height": round(FLOOR_2_Z - FLOOR_1_Z, 6),
        "ramp_slope_degrees": round(max_segment_slope, 6),
        "ramp_equivalent_slope_degrees": round(equivalent_slope, 6),
        "ramp_waypoint_spacing": ramp_spacing,
        "ramp_geometry": {
            "geometry_type": "switchback_polyline_surrogate",
            "semantic_connector_preserved": TRUE_TRANSITION_EDGE,
            "forbidden_non_transition_edge": FORBIDDEN_NON_TRANSITION_EDGE,
            "raw_connector_metric": raw_metric,
            "raw_connector_too_steep_for_conservative_simulation": raw_metric["slope_degrees"] > 15.0,
            "extension_reason": (
                "The raw semantic handoff rises 1.6 m over about 4.2 m XY, which is near 21 degrees. "
                "Task59 therefore embeds vt_1_centerline_e001 as a longer switchback ramp surrogate for Gazebo."
            ),
            "control_points": [
                {"x": round(point.x, 6), "y": round(point.y, 6), "z": round(point.z, 6)}
                for point in control_points
            ],
            "segment_metrics": ramp_metrics,
            "total_xy_run_m": round(equivalent_run, 6),
            "total_ramp_waypoints": len(ramp_waypoints),
        },
        "source_route_summary": {
            "runtime_waypoints_before_ramp_insertion": len(pid_input.get("runtime_waypoints") or []),
            "floor_1_source_waypoints": len(floor_1),
            "floor_2_source_waypoints": len(floor_2),
            "first_source_waypoint": floor_1[0],
            "last_floor_1_waypoint": last_floor_1,
            "first_floor_2_waypoint": first_floor_2,
            "last_source_waypoint": floor_2[-1],
            "connector_handoff": handoff,
        },
        "waypoints": waypoint_records,
        "runtime_waypoints": waypoint_records,
        "waypoint_counts": {
            "total": len(waypoint_records),
            "floor_1_original": len(source_floor_1),
            "ramp_surrogate": len(ramp_waypoints),
            "floor_2_original_after_ramp": len(source_floor_2),
        },
        "planned_path_odom_policy": {
            "frame_id": "odom",
            "topic": PLANNED_TOPIC,
            "anchor_first_waypoint_to_odom_start": True,
            "path_source": "task59_multifloor_ramp_runtime_input.waypoints",
        },
        "actual_trajectory_policy": {
            "frame_id": "odom",
            "topic": EXECUTED_TOPIC,
            "robot_pose_topic": ROBOT_POSE_TOPIC,
            "marker_topic": MARKER_TOPIC,
            "source": "Gazebo /odom accumulated during task59 simulation",
        },
        "route_audit": route_audit,
        "claim_boundary": {
            "simulation_only": True,
            "gazebo_ramp_surrogate": True,
            "physical_stair_climbing_claimed": False,
            "physical_robot_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
            "global_collision_free_guarantee_claimed": False,
            "unitree_stair_gait_claimed": False,
        },
    }


def markdown_bool(value: Any) -> str:
    return "`true`" if bool(value) else "`false`"


def route_audit_markdown(
    *,
    selected_query_id: str,
    selected_audit: dict[str, Any],
    audited: list[dict[str, Any]],
    selection_reason: str,
) -> str:
    rows = [
        "| query | available | suitable | floor counts | transition edges | selected goal | notes |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for audit in audited:
        rows.append(
            "| `{query}` | {available} | {suitable} | `{floors}` | `{edges}` | `{goal}` | {notes} |".format(
                query=audit.get("query_id"),
                available=audit.get("available"),
                suitable=audit.get("suitable"),
                floors=audit.get("floor_counts"),
                edges=audit.get("transition_edges"),
                goal=audit.get("selected_goal_candidate_id"),
                notes="; ".join(audit.get("reasons") or []),
            )
        )
    return "\n".join(
        [
            "# Task59 Cross-Floor Route Audit",
            "",
            f"- Selected cross-floor query: `{selected_query_id}`",
            f"- Selection reason: {selection_reason}",
            f"- Does the selected route include floor_1? {markdown_bool((selected_audit.get('floor_counts') or {}).get('floor_1', 0) > 0)}",
            f"- Does the selected route include floor_2? {markdown_bool((selected_audit.get('floor_counts') or {}).get('floor_2', 0) > 0)}",
            f"- Does the selected route include `{TRUE_TRANSITION_EDGE}`? {markdown_bool(selected_audit.get('uses_true_transition_edge'))}",
            f"- Does it avoid `{FORBIDDEN_NON_TRANSITION_EDGE}` as a transition? {markdown_bool(not selected_audit.get('e003_as_transition'))}",
            f"- Does it avoid `{BLOCKED_CANDIDATE_ID}` as runtime goal? {markdown_bool(not selected_audit.get('generated_ring_037_selected'))}",
            f"- Does it keep `{SELECTED_APPROACH_ID}` as final object approach? {markdown_bool(selected_audit.get('generated_ring_002_final_approach'))}",
            f"- Waypoints before ramp insertion: `{selected_audit.get('runtime_waypoint_count')}`",
            "",
            "## Candidate Audit",
            "",
            *rows,
            "",
            "## Selected Route Segments",
            "",
            f"- Floor 1 segment: `{selected_audit.get('floor_1_segment')}`",
            f"- Connector segment: `{selected_audit.get('connector_segment')}`",
            f"- Floor 2 segment: `{selected_audit.get('floor_2_segment')}`",
            f"- Floor sequence: `{selected_audit.get('floor_sequence')}`",
            f"- Room sequence: `{selected_audit.get('room_sequence')}`",
            "",
            f"`{FORBIDDEN_NON_TRANSITION_EDGE}` is retained only as forbidden/non-transition evidence. "
            f"`{BLOCKED_CANDIDATE_ID}` is retained only as blocked/rejected/evidence if present in source artifacts.",
            "",
        ]
    )


def feasibility_markdown(runtime_input: dict[str, Any]) -> str:
    geometry = runtime_input["ramp_geometry"]
    raw = geometry["raw_connector_metric"]
    metrics = geometry["segment_metrics"]
    metric_lines = [
        "| segment | xy run m | rise m | slope deg | yaw rad |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for index, metric in enumerate(metrics):
        metric_lines.append(
            f"| {index} | {metric['xy_run_m']} | {metric['rise_m']} | {metric['slope_degrees']} | {metric['yaw']} |"
        )
    return "\n".join(
        [
            "# Task59 Connector And Ramp Feasibility Audit",
            "",
            f"- Selected query: `{runtime_input['source_query_id']}`",
            f"- Floor 1 endpoint before connector: `{runtime_input['source_route_summary']['last_floor_1_waypoint']}`",
            f"- Floor 2 endpoint after connector: `{runtime_input['source_route_summary']['first_floor_2_waypoint']}`",
            f"- Raw connector XY distance: `{raw['xy_run_m']}` m",
            f"- Height difference: `{runtime_input['ramp_height']}` m",
            f"- Raw direct slope angle: `{raw['slope_degrees']}` degrees",
            "- TurtleBot3 burger feasibility: the raw direct connector is treated as too steep for conservative "
            "Gazebo validation, so task59 does not use it as a physical ramp.",
            "- Ramp needs intermediate waypoints: `true`",
            "- Ramp needs length extension for simulation feasibility: `true`",
            "",
            "## Ramp-Surrogate Embedding",
            "",
            f"- Semantic connector identity preserved: `{TRUE_TRANSITION_EDGE}`",
            f"- Forbidden non-transition edge avoided: `{FORBIDDEN_NON_TRANSITION_EDGE}`",
            f"- Floor 1 surface z: `{runtime_input['floor_1_z']}`",
            f"- Floor 2 surface z: `{runtime_input['floor_2_z']}`",
            f"- Ramp waypoint spacing: `{runtime_input['ramp_waypoint_spacing']}` m",
            f"- Ramp equivalent slope: `{runtime_input['ramp_equivalent_slope_degrees']}` degrees",
            f"- Maximum ramp segment slope: `{runtime_input['ramp_slope_degrees']}` degrees",
            f"- Ramp waypoint count: `{runtime_input['waypoint_counts']['ramp_surrogate']}`",
            "",
            *metric_lines,
            "",
            "This is a simulation surrogate. It is not an exact physical stair model, not physical stair climbing, "
            "and not a claim about a real robot gait.",
            "",
        ]
    )


def runtime_design_markdown(runtime_input: dict[str, Any], output_json: Path) -> str:
    counts = runtime_input["waypoint_counts"]
    return "\n".join(
        [
            "# Task59 Multifloor Runtime Input Design",
            "",
            f"- Runtime input JSON: `{output_json}`",
            f"- Schema: `{runtime_input['schema_name']}@{runtime_input['schema_version']}`",
            f"- Route mode: `{runtime_input['route_mode']}`",
            f"- Source query: `{runtime_input['source_query_id']}`",
            f"- Source RouteResult: `{runtime_input['selected_route_result_json']}`",
            f"- Source PID input: `{runtime_input['selected_pid_input_json']}`",
            f"- Total waypoints after ramp insertion: `{counts['total']}`",
            f"- Original floor_1 waypoints: `{counts['floor_1_original']}`",
            f"- Ramp surrogate waypoints: `{counts['ramp_surrogate']}`",
            f"- Original floor_2 waypoints after ramp endpoint: `{counts['floor_2_original_after_ramp']}`",
            f"- Ramp connector edge: `{runtime_input['ramp_connector_edge_id']}`",
            f"- Forbidden transition edges: `{runtime_input['forbidden_transition_edge_ids']}`",
            f"- Blocked goal candidates: `{runtime_input['blocked_goal_candidate_ids']}`",
            f"- Target approach id: `{runtime_input['target_approach_id']}`",
            "",
            "The adapter preserves the original floor_1 route order, inserts a switchback ramp segment for "
            f"`{TRUE_TRANSITION_EDGE}`, then resumes the original floor_2 route order. "
            f"`{FORBIDDEN_NON_TRANSITION_EDGE}` is not used as a transition and `{BLOCKED_CANDIDATE_ID}` is not a runtime goal.",
            "",
            "## Topics",
            "",
            f"- Planned path topic: `{runtime_input['planned_path_odom_policy']['topic']}`",
            f"- Executed path topic: `{runtime_input['actual_trajectory_policy']['topic']}`",
            f"- Robot pose topic: `{runtime_input['actual_trajectory_policy']['robot_pose_topic']}`",
            f"- Marker topic: `{runtime_input['actual_trajectory_policy']['marker_topic']}`",
            f"- Frame: `{runtime_input['planned_path_odom_policy']['frame_id']}`",
            "",
            "The follower remains XY/yaw based. The Gazebo world supplies the collision geometry that raises the robot; "
            "the node does not command z directly.",
            "",
        ]
    )


def helper_script_content(kind: str, runtime_input: Path, task_dir: Path) -> str:
    repo_root = str(REPO_ROOT)
    run_script = f"{repo_root}/tools/rslg_pipeline/gazebo/run_multifloor_ramp_gazebo_validation.sh"
    rviz_config = f"{repo_root}/tools/rslg_pipeline/rviz/config/rslg_multifloor_ramp_actual_trajectory_showcase.rviz"
    common = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        f"REPO_ROOT={repo_root!r}",
        f"RUNTIME_INPUT={str(runtime_input)!r}",
        f"TASK_DIR={str(task_dir)!r}",
    ]
    if kind == "run_gazebo_gui.sh":
        body = [f'"{run_script}" --with-gazebo-gui --duration-sec "${{1:-420}}"']
    elif kind == "run_rviz_node.sh":
        body = [f'"{run_script}" --no-gazebo --with-rviz-node --duration-sec "${{1:-0}}"']
    elif kind == "run_rviz_gui.sh":
        body = [
            'if ! command -v rviz2 >/dev/null 2>&1; then',
            '  echo "rviz2 was not found in PATH." >&2',
            "  exit 69",
            "fi",
            f'rviz2 -d "{rviz_config}"',
        ]
    elif kind == "run_multifloor_follower.sh":
        body = [
            'ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"',
            '"${ROS_PYTHON}" "${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_multifloor_ramp_follower.py" \\',
            '  --runtime-input-json "${RUNTIME_INPUT}" \\',
            '  --profile-json "${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json" \\',
            f"  --profile-id {DEFAULT_PROFILE_ID} \\",
            '  --output-dir "${TASK_DIR}/multifloor_ramp_pack/manual_runs/follower_$(date -u +%Y%m%dT%H%M%SZ)" \\',
            "  --frame-id odom --anchor-first-waypoint-to-odom-start --stop-at-end --goal-timeout-sec \"${1:-420}\"",
        ]
    elif kind == "run_headless_validation.sh":
        body = [f'"{run_script}" --headless --duration-sec "${{1:-420}}"']
    else:
        raise ValueError(kind)
    return "\n".join(common + body) + "\n"


def write_helper_scripts(pack_dir: Path, runtime_input: Path, task_dir: Path) -> list[str]:
    scripts = [
        "run_gazebo_gui.sh",
        "run_rviz_node.sh",
        "run_rviz_gui.sh",
        "run_multifloor_follower.sh",
        "run_headless_validation.sh",
    ]
    written: list[str] = []
    for script in scripts:
        path = pack_dir / script
        write_text(path, helper_script_content(script, runtime_input, task_dir))
        path.chmod(path.stat().st_mode | 0o111)
        written.append(str(path))
    return written


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--task-dir", type=Path, default=DEFAULT_TASK_DIR)
    parser.add_argument("--ramp-waypoint-spacing", type=float, default=DEFAULT_RAMP_WAYPOINT_SPACING)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--write-audits", action="store_true")
    parser.add_argument("--write-helper-scripts", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    task_dir = args.task_dir
    pack_dir = task_dir / "multifloor_ramp_pack"
    runtime_dir = pack_dir / "runtime_inputs"
    selected_query, selected_audit, audited, selection_reason = choose_query(args.query_id)
    paths = route_paths(selected_query)
    route_result = read_json(paths.route_result)
    pid_input = read_json(paths.pid_input)
    runtime_input = build_runtime_input(
        query_id=selected_query,
        route_result=route_result,
        pid_input=pid_input,
        paths=paths,
        route_audit=selected_audit,
        selection_reason=selection_reason,
        ramp_spacing=float(args.ramp_waypoint_spacing),
    )
    output_json = args.output_json or runtime_dir / f"{selected_query}_multifloor_ramp_runtime_input.json"
    write_json(output_json, runtime_input)
    helper_scripts: list[str] = []
    if args.write_helper_scripts:
        helper_scripts = write_helper_scripts(pack_dir, output_json, task_dir)
    if args.write_audits:
        write_text(
            task_dir / "02_cross_floor_route_audit.md",
            route_audit_markdown(
                selected_query_id=selected_query,
                selected_audit=selected_audit,
                audited=audited,
                selection_reason=selection_reason,
            ),
        )
        write_text(task_dir / "03_connector_and_ramp_feasibility_audit.md", feasibility_markdown(runtime_input))
        write_text(task_dir / "05_multifloor_runtime_input_design.md", runtime_design_markdown(runtime_input, output_json))

    summary = {
        "ok": True,
        "runtime_input_json": str(output_json),
        "selected_query_id": selected_query,
        "selection_reason": selection_reason,
        "task_dir": str(task_dir),
        "helper_scripts": helper_scripts,
        "waypoint_counts": runtime_input["waypoint_counts"],
        "ramp_slope_degrees": runtime_input["ramp_slope_degrees"],
        "ramp_equivalent_slope_degrees": runtime_input["ramp_equivalent_slope_degrees"],
        "ramp_connector_edge_id": TRUE_TRANSITION_EDGE,
        "forbidden_transition_edge_ids": [FORBIDDEN_NON_TRANSITION_EDGE],
        "blocked_goal_candidate_ids": [BLOCKED_CANDIDATE_ID],
        "target_approach_id": runtime_input.get("target_approach_id"),
    }
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
