#!/usr/bin/env python3
"""RSLG-SLAM demo route classifier.

Given an RSLGRouteResult and/or a PID runtime input, infer the *natural* demo
runtime mode used by the task62 unified demo package. This module never runs
Nav2, AMCL, map_server, Stage-A, or raw RGB-D inference. It is a pure static
classifier used for demo route registration, dry-run validation, and guard
checks.

Modes:
  - same_floor_pid              : single-floor executable via Gazebo PID follower
  - scripted_stair_transition   : floor_1 + vt_1_centerline_e001 scripted stair + floor_2
  - connector_scripted_transition: connector-only demo of the vertical transition
  - dry_run_only                : guard/availability validation only (no live goal)

Guard invariants surfaced (never silently violated):
  - generated_ring_037 must never be the selected runtime goal.
  - vt_1_centerline_e003 (the non-transition edge) must not be a transition edge.
  - vt_1_centerline_e001 is the only valid scripted transition edge.
  - physical_stair_climbing_claim is always false.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

FORBIDDEN_RUNTIME_GOAL = "generated_ring_037"
FORBIDDEN_TRANSITION_EDGE = "vt_1_centerline_e003"  # non-transition edge; not a transition edge
VALID_TRANSITION_EDGE = "vt_1_centerline_e001"


def _load_json(path: Optional[str]) -> Optional[dict[str, Any]]:
    if not path:
        return None
    p = Path(path)
    if not p.is_file():
        return None
    with p.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _selected_goal_id(route_result: Optional[dict], pid_input: Optional[dict]) -> Optional[str]:
    if route_result:
        for cand in route_result.get("approach", {}).get("approach_candidates", []):
            if cand.get("is_runtime_goal"):
                return cand.get("candidate_id")
    if pid_input:
        sel = pid_input.get("selected_goal")
        if isinstance(sel, dict):
            return sel.get("candidate_id")
        if isinstance(sel, str):
            return sel
    return None


def _rejected_ids(route_result: Optional[dict], pid_input: Optional[dict]) -> list[str]:
    ids: list[str] = []
    if route_result:
        for cand in route_result.get("approach", {}).get("rejected_candidates", []):
            cid = cand.get("candidate_id")
            if cid:
                ids.append(cid)
    if pid_input:
        for cand in pid_input.get("rejected_runtime_candidates", []) or []:
            cid = cand.get("candidate_id") if isinstance(cand, dict) else cand
            if cid:
                ids.append(cid)
    return ids


def _floor_breakdown(pid_input: Optional[dict]) -> dict[str, int]:
    breakdown: dict[str, int] = {}
    if not pid_input:
        return breakdown
    for wp in pid_input.get("runtime_waypoints", []) or []:
        floor = wp.get("floor_id")
        if floor is None:
            continue
        breakdown[floor] = breakdown.get(floor, 0) + 1
    return breakdown


def classify(
    query_id: str,
    route_result: Optional[dict] = None,
    pid_input: Optional[dict] = None,
    query_json: Optional[dict] = None,
) -> dict[str, Any]:
    """Return a static classification of the demo route."""
    identity = (route_result or {}).get("identity", {})
    query_type = identity.get("query_type") or (query_json or {}).get("query_type")
    target = (route_result or {}).get("target", {}) or (query_json or {}).get("target", {})
    target_type = target.get("target_type")

    semantic_route = (route_result or {}).get("semantic_route", {})
    connectors = semantic_route.get("connector_sequence", []) or []
    transition_edges = [c.get("transition_edge") for c in connectors if c.get("transition_edge")]
    non_transition_edges = [
        c.get("non_transition_edge") for c in connectors if c.get("non_transition_edge")
    ]

    segments = (route_result or {}).get("route_segments", []) or []
    segment_types = [s.get("segment_type") for s in segments]
    has_connector_segment = "connector_handoff" in segment_types
    has_object_approach = "object_approach_metric" in segment_types

    floor_breakdown = _floor_breakdown(pid_input)
    connector_handoff_count = len(pid_input.get("connector_handoffs", []) or []) if pid_input else 0
    if connector_handoff_count == 0 and has_connector_segment:
        connector_handoff_count = segment_types.count("connector_handoff")

    floors_in_waypoints = [f for f in floor_breakdown]
    is_single_floor_execution = len(floors_in_waypoints) == 1

    selected_goal = _selected_goal_id(route_result, pid_input)
    rejected = _rejected_ids(route_result, pid_input)

    is_blocked_candidate = "blocked_candidate" in query_id

    # Natural route mode inference.
    if is_blocked_candidate:
        natural_mode = "dry_run_only"
    elif query_type == "floor_connector" or target_type == "floor_connector":
        natural_mode = "connector_scripted_transition"
    elif connector_handoff_count >= 1 and (len(floors_in_waypoints) >= 2 or has_connector_segment):
        natural_mode = "scripted_stair_transition"
    elif is_single_floor_execution:
        natural_mode = "same_floor_pid"
    elif has_object_approach and not has_connector_segment:
        natural_mode = "same_floor_pid"
    else:
        natural_mode = "dry_run_only"

    # Guard evaluation.
    guard = {
        "forbidden_runtime_goal_selected": selected_goal == FORBIDDEN_RUNTIME_GOAL,
        "forbidden_runtime_goal_present_as_rejected_only": (
            FORBIDDEN_RUNTIME_GOAL in rejected and selected_goal != FORBIDDEN_RUNTIME_GOAL
        ),
        "forbidden_transition_edge_used_as_transition": (
            FORBIDDEN_TRANSITION_EDGE in transition_edges
        ),
        "forbidden_transition_edge_present_as_non_transition_only": (
            FORBIDDEN_TRANSITION_EDGE in non_transition_edges
            and FORBIDDEN_TRANSITION_EDGE not in transition_edges
        ),
        "valid_transition_edge_used": VALID_TRANSITION_EDGE in transition_edges,
        "physical_stair_climbing_claim": False,
    }

    claim_boundary = (route_result or {}).get("claim_boundary", {})

    return {
        "query_id": query_id,
        "query_type": query_type,
        "target_type": target_type,
        "natural_route_mode": natural_mode,
        "floor_breakdown": floor_breakdown,
        "floors_in_waypoints": floors_in_waypoints,
        "runtime_waypoint_count": (pid_input or {}).get("runtime_waypoint_count"),
        "route_segment_types": segment_types,
        "has_connector_segment": has_connector_segment,
        "has_object_approach": has_object_approach,
        "connector_handoff_count": connector_handoff_count,
        "transition_edges": transition_edges,
        "non_transition_edges": non_transition_edges,
        "selected_runtime_goal": selected_goal,
        "forbidden_goal_in_rejected": FORBIDDEN_RUNTIME_GOAL in rejected,
        "guard": guard,
        "claim_boundary": {
            "no_nav2_dependency": claim_boundary.get("no_nav2_dependency", True),
            "no_amcl_dependency": claim_boundary.get("no_amcl_dependency", True),
            "no_physical_stair_climbing_claim": claim_boundary.get(
                "no_physical_stair_climbing_claim", True
            ),
            "no_real_robot_claim": claim_boundary.get("no_real_robot_claim", True),
            "no_collision_free_guarantee": claim_boundary.get(
                "no_collision_free_guarantee", True
            ),
        },
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query-id", required=True)
    parser.add_argument("--route-result-json", default=None)
    parser.add_argument("--pid-input-json", default=None)
    parser.add_argument("--query-json", default=None)
    parser.add_argument("--expected-mode", default=None)
    parser.add_argument("--output-json", default=None)
    args = parser.parse_args(argv)

    route_result = _load_json(args.route_result_json)
    pid_input = _load_json(args.pid_input_json)
    query_json = _load_json(args.query_json)

    result = classify(
        query_id=args.query_id,
        route_result=route_result,
        pid_input=pid_input,
        query_json=query_json,
    )
    if args.expected_mode:
        result["expected_mode"] = args.expected_mode
        result["expected_matches_natural"] = args.expected_mode == result["natural_route_mode"]

    text = json.dumps(result, indent=2)
    if args.output_json:
        Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output_json).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
