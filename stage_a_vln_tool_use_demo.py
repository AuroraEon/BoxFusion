from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.vln_closed_loop import VLNClosedLoopExecutor, extract_room_transition_observations
from boxfusion.vln_tool_use import MinimalVLNToolUseAdapter, render_teacher_brief


DEFAULT_OUTPUT_ROOT = Path("stage_a_outputs_vt_fallback_v01_rerun2")
DEFAULT_SEQUENCE_IDS = ("00843-DYehNKdT76V", "00847-bCPU9suPUw9")
REPORT_VERSION = "0.1"
IMPLEMENTATION_FILES = [
    "boxfusion/vln_tool_use.py",
    "stage_a_vln_tool_use_demo.py",
    "boxfusion/test_vln_tool_use.py",
]


def _json_dump(payload: Dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _room_number(room_id: str) -> str:
    return str(room_id).replace("room_", "room ")


def _score_route_case(case: Dict[str, Any]) -> Tuple[int, int, str]:
    route = dict(case.get("route") or {})
    return (
        int(route.get("hop_count") or 0),
        int(case.get("start_frame_idx") or 10**6),
        str(case.get("goal_room_id") or case.get("resolved_goal_room_id") or ""),
    )


def _route_matches_exact_prefix(transitions: List[Dict[str, Any]], start_index: int, plan_rooms: List[str]) -> bool:
    if not plan_rooms:
        return False
    observed = [item.get("room_id") for item in transitions[start_index : start_index + len(plan_rooms)]]
    return observed == list(plan_rooms)


def _load_adapter(output_root: Path, sequence_id: str) -> Tuple[MinimalVLNToolUseAdapter, Path, Path]:
    sequence_root = output_root / sequence_id
    topology_json = sequence_root / "logs" / "topology_v0_1.json"
    timeline_json = sequence_root / "logs" / "timeline.json"
    adapter = MinimalVLNToolUseAdapter.from_paths(topology_json=topology_json, timeline_json=timeline_json)
    return adapter, topology_json, timeline_json


def _observed_transitions(executor: VLNClosedLoopExecutor) -> List[Dict[str, Any]]:
    return extract_room_transition_observations(executor.observations)


def _object_label_counts(query_api: RoomTopologyQueryAPI) -> Dict[str, List[str]]:
    counts: Dict[str, List[str]] = {}
    for object_id in query_api.topology.list_object_ids():
        record = query_api.topology.get_object(object_id) or {}
        label = record.get("label") or record.get("normalized_label")
        room_id = record.get("room_id")
        if not label or room_id is None:
            continue
        counts.setdefault(str(label), [])
        if room_id not in counts[str(label)]:
            counts[str(label)].append(str(room_id))
    return counts


def _room_direction(query_api: RoomTopologyQueryAPI, start_room_id: str, goal_room_id: str) -> Optional[str]:
    start_room = query_api.topology.get_room(start_room_id) or {}
    goal_room = query_api.topology.get_room(goal_room_id) or {}
    floors = {
        str(item.get("floor_id")): dict(item)
        for item in query_api.topology.floor_records.values()
        if item.get("floor_id") is not None
    }
    try:
        start_order = int((floors.get(str(start_room.get("floor_id"))) or {}).get("display_order"))
        goal_order = int((floors.get(str(goal_room.get("floor_id"))) or {}).get("display_order"))
    except (TypeError, ValueError):
        return None
    if goal_order > start_order:
        return "upstairs"
    if goal_order < start_order:
        return "downstairs"
    return None


def _same_floor_room_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") != start_floor_id:
                continue
            query_result = query_api.query_route(start_room_id, goal_room_id)
            if not query_result.get("found"):
                continue
            if "vertical_transition" in list((query_result.get("route") or {}).get("used_relation_types", [])):
                continue
            candidate = {
                "start_room_id": start_room_id,
                "goal_room_id": goal_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "route": query_result.get("route"),
            }
            if best_case is None or _score_route_case(candidate) < _score_route_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a same-floor room case.")
    return best_case


def _cross_floor_room_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") == start_floor_id:
                continue
            query_result = query_api.query_route(start_room_id, goal_room_id)
            if not query_result.get("found"):
                continue
            if "vertical_transition" not in list((query_result.get("route") or {}).get("used_relation_types", [])):
                continue
            candidate = {
                "start_room_id": start_room_id,
                "goal_room_id": goal_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "route": query_result.get("route"),
            }
            if best_case is None or _score_route_case(candidate) < _score_route_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a cross-floor room case.")
    return best_case


def _routable_anchor_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        for anchor_id in query_api.topology.list_anchor_ids(valid_only=True):
            anchor_record = query_api.topology.get_anchor(anchor_id) or {}
            target_label = None
            if anchor_record.get("anchor_type") == "object":
                object_record = query_api.topology.get_object(anchor_record.get("target_id")) or {}
                target_label = object_record.get("label") or object_record.get("normalized_label")
            elif anchor_record.get("anchor_type") == "room":
                room_record = query_api.topology.get_room(anchor_record.get("target_id")) or {}
                target_label = room_record.get("room_type")
            if not target_label:
                continue
            query_result = query_api.query_route_to_anchor(start_room_id, anchor_id)
            if not query_result.get("found"):
                continue
            candidate = {
                "start_room_id": start_room_id,
                "anchor_id": anchor_id,
                "target_label": str(target_label),
                "start_frame_idx": start.get("frame_idx"),
                "route": query_result.get("route"),
                "resolved_goal_room_id": query_result.get("resolved_goal_room_id"),
            }
            if best_case is None or _score_route_case(candidate) < _score_route_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a routable anchor case.")
    return best_case


def _routable_object_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    label_counts = _object_label_counts(query_api)
    best_case: Optional[Dict[str, Any]] = None
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        for label, rooms in sorted(label_counts.items()):
            if len(rooms) != 1:
                continue
            query_result = query_api.query_route_to_object(start_room_id, object_label=label)
            if not query_result.get("found"):
                continue
            candidate = {
                "start_room_id": start_room_id,
                "object_label": label,
                "start_frame_idx": start.get("frame_idx"),
                "route": query_result.get("route"),
                "resolved_goal_room_id": query_result.get("resolved_goal_room_id"),
            }
            if best_case is None or _score_route_case(candidate) < _score_route_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a routable object case.")
    return best_case


def _ambiguous_object_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    label_counts = _object_label_counts(query_api)
    ambiguous_labels = sorted(label for label, rooms in label_counts.items() if len(rooms) > 1)
    if not ambiguous_labels:
        raise RuntimeError("Could not discover an ambiguous object label case.")
    return {
        "start_room_id": str(transitions[0].get("room_id")),
        "object_label": ambiguous_labels[0],
        "start_frame_idx": transitions[0].get("frame_idx"),
    }


def _successful_execute_case(
    adapter: MinimalVLNToolUseAdapter,
    transitions: List[Dict[str, Any]],
    *,
    cross_floor: bool,
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    if adapter.executor is None:
        raise RuntimeError("Execute discovery requires a closed-loop executor.")
    for start_index, start in enumerate(transitions):
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in adapter.query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = adapter.query_api.topology.get_room(goal_room_id) or {}
            if cross_floor and goal_room.get("floor_id") == start_floor_id:
                continue
            if (not cross_floor) and goal_room.get("floor_id") != start_floor_id:
                continue
            task_input = {"start_room_id": start_room_id, "target": {"target_type": "room", "goal_room_id": goal_room_id}}
            execution = adapter.executor.execute(
                task_input=task_input,
                route_policy="balanced",
                start_frame_idx=int(start.get("frame_idx")),
            )
            outcome = dict(execution.get("outcome") or {})
            if outcome.get("outcome_category") != "success":
                continue
            plan_rooms = list((((execution.get("plan") or {}).get("symbolic_plan") or {}).get("room_sequence")) or [])
            if len(plan_rooms) < 2:
                continue
            if not _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                continue
            relation_sequence = list((((execution.get("plan") or {}).get("symbolic_plan") or {}).get("relation_sequence")) or [])
            if cross_floor and "vertical_transition" not in relation_sequence:
                continue
            if (not cross_floor) and "vertical_transition" in relation_sequence:
                continue
            candidate = {
                "start_room_id": start_room_id,
                "goal_room_id": goal_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "execution": execution,
            }
            route = ((execution.get("plan") or {}).get("query_result") or {}).get("route") or {}
            candidate["route"] = route
            if best_case is None or _score_route_case(candidate) < _score_route_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a successful execute case.")
    return best_case


def _run_case(
    adapter: MinimalVLNToolUseAdapter,
    *,
    case_id: str,
    sequence_id: str,
    category: str,
    request_text: str,
    start_room_id: str,
    expectation: Dict[str, Any],
    start_frame_idx: Optional[int] = None,
) -> Dict[str, Any]:
    result = adapter.handle_request(
        request_text,
        start_room_id=start_room_id,
        route_policy="balanced",
        start_frame_idx=start_frame_idx,
    )
    parsed = dict(result.get("parsed_request") or {})
    teacher = dict(result.get("teacher_response") or {})
    backend = dict(result.get("backend_result") or {})
    tool_selection = dict(result.get("tool_selection") or {})
    pass_checks = [result.get("status") == expectation.get("status")]
    if expectation.get("target_type") is not None:
        pass_checks.append(parsed.get("target_type") == expectation.get("target_type"))
    if expectation.get("request_type") is not None:
        pass_checks.append(parsed.get("request_type") == expectation.get("request_type"))
    if expectation.get("tool_name") is not None:
        pass_checks.append(tool_selection.get("tool_name") == expectation.get("tool_name"))
    if expectation.get("needs_floor_switch") is True:
        pass_checks.append(bool(teacher.get("floor_switch_required")))
    if expectation.get("outcome_category") is not None:
        pass_checks.append(((backend.get("outcome") or {}).get("outcome_category")) == expectation.get("outcome_category"))
    return {
        "case_id": case_id,
        "sequence_id": sequence_id,
        "category": category,
        "request_text": request_text,
        "start_room_id": start_room_id,
        "start_frame_idx": start_frame_idx,
        "expectation": dict(expectation),
        "status": result.get("status"),
        "pass": all(pass_checks),
        "tool_selection": tool_selection,
        "parsed_request": parsed,
        "resolved_target": result.get("resolved_target"),
        "teacher_response": teacher,
        "backend_result": backend,
        "teacher_brief": render_teacher_brief(result),
    }


def _sequence_cases(sequence_id: str, adapter: MinimalVLNToolUseAdapter) -> List[Dict[str, Any]]:
    if adapter.executor is None:
        raise RuntimeError("Sequence demo requires an attached executor.")
    transitions = _observed_transitions(adapter.executor)
    query_api = adapter.query_api

    same_floor_room = _same_floor_room_case(query_api, transitions)
    cross_floor_room = _cross_floor_room_case(query_api, transitions)
    anchor_case = _routable_anchor_case(query_api, transitions)
    object_case = _routable_object_case(query_api, transitions)
    execute_case = _successful_execute_case(adapter, transitions, cross_floor=True)
    ambiguous_case = _ambiguous_object_case(query_api, transitions)

    cross_floor_direction = _room_direction(query_api, cross_floor_room["start_room_id"], cross_floor_room["goal_room_id"])
    execute_direction = _room_direction(query_api, execute_case["start_room_id"], execute_case["goal_room_id"])
    anchor_direction = _room_direction(query_api, anchor_case["start_room_id"], anchor_case["resolved_goal_room_id"])

    return [
        _run_case(
            adapter,
            case_id=f"{sequence_id}_same_floor_room_request",
            sequence_id=sequence_id,
            category="same_floor_room_request",
            request_text=f"How do I get to {_room_number(same_floor_room['goal_room_id'])}?",
            start_room_id=same_floor_room["start_room_id"],
            start_frame_idx=same_floor_room["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "room",
                "request_type": "route_explanation",
                "tool_name": "query_route",
                "needs_floor_switch": False,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_cross_floor_room_request",
            sequence_id=sequence_id,
            category="cross_floor_room_request",
            request_text=(
                f"How do I get to {_room_number(cross_floor_room['goal_room_id'])} {cross_floor_direction}?"
                if cross_floor_direction
                else f"How do I get to {_room_number(cross_floor_room['goal_room_id'])}?"
            ),
            start_room_id=cross_floor_room["start_room_id"],
            start_frame_idx=cross_floor_room["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "room",
                "request_type": "route_explanation",
                "tool_name": "query_route",
                "needs_floor_switch": True,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_anchor_request",
            sequence_id=sequence_id,
            category="anchor_request",
            request_text=(
                f"How do I get to {anchor_case['anchor_id']} {anchor_direction}?"
                if anchor_direction
                else f"How do I get to {anchor_case['anchor_id']}?"
            ),
            start_room_id=anchor_case["start_room_id"],
            start_frame_idx=anchor_case["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "anchor",
                "request_type": "route_explanation",
                "tool_name": "query_route_to_anchor",
                "needs_floor_switch": None,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_object_request",
            sequence_id=sequence_id,
            category="object_request",
            request_text=f"How do I get to the {object_case['object_label']}?",
            start_room_id=object_case["start_room_id"],
            start_frame_idx=object_case["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "object",
                "request_type": "route_explanation",
                "tool_name": "query_route_to_object",
                "needs_floor_switch": None,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_route_explanation_request",
            sequence_id=sequence_id,
            category="route_explanation_request",
            request_text=(
                f"Will I need to switch floors to reach {_room_number(cross_floor_room['goal_room_id'])} {cross_floor_direction}?"
                if cross_floor_direction
                else f"Will I need to switch floors to reach {_room_number(cross_floor_room['goal_room_id'])}?"
            ),
            start_room_id=cross_floor_room["start_room_id"],
            start_frame_idx=cross_floor_room["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "room",
                "request_type": "route_explanation",
                "tool_name": "query_route",
                "needs_floor_switch": True,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_execute_request",
            sequence_id=sequence_id,
            category="execute_request",
            request_text=(
                f"Start navigation to {_room_number(execute_case['goal_room_id'])} {execute_direction}."
                if execute_direction
                else f"Start navigation to {_room_number(execute_case['goal_room_id'])}."
            ),
            start_room_id=execute_case["start_room_id"],
            start_frame_idx=execute_case["start_frame_idx"],
            expectation={
                "status": "success",
                "target_type": "room",
                "request_type": "execute",
                "tool_name": "closed_loop_execute",
                "needs_floor_switch": True if execute_direction else None,
                "outcome_category": "success",
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_ambiguous_target_request",
            sequence_id=sequence_id,
            category="ambiguous_target_request",
            request_text=f"Go to the {ambiguous_case['object_label']}.",
            start_room_id=ambiguous_case["start_room_id"],
            start_frame_idx=ambiguous_case["start_frame_idx"],
            expectation={
                "status": "ambiguous",
                "target_type": "object",
                "request_type": "execute",
                "tool_name": None,
                "needs_floor_switch": None,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_unresolved_target_request",
            sequence_id=sequence_id,
            category="unresolved_target_request",
            request_text="Go to the kitchen.",
            start_room_id=str(transitions[0].get("room_id")),
            start_frame_idx=transitions[0].get("frame_idx"),
            expectation={
                "status": "unresolved",
                "target_type": "object",
                "request_type": "execute",
                "tool_name": None,
                "needs_floor_switch": None,
                "outcome_category": None,
            },
        ),
        _run_case(
            adapter,
            case_id=f"{sequence_id}_unsupported_request",
            sequence_id=sequence_id,
            category="unsupported_request",
            request_text="Explore every floor and choose the nicest room.",
            start_room_id=str(transitions[0].get("room_id")),
            start_frame_idx=transitions[0].get("frame_idx"),
            expectation={
                "status": "unsupported",
                "target_type": None,
                "request_type": None,
                "tool_name": None,
                "needs_floor_switch": None,
                "outcome_category": None,
            },
        ),
    ]


def sequence_report(output_root: Path, sequence_id: str) -> Dict[str, Any]:
    adapter, topology_json, timeline_json = _load_adapter(output_root, sequence_id)
    cases = _sequence_cases(sequence_id, adapter)
    pass_count = sum(1 for item in cases if item.get("pass"))
    return {
        "sequence_id": sequence_id,
        "topology_json": str(topology_json),
        "timeline_json": str(timeline_json),
        "cases": cases,
        "summary": {
            "case_count": len(cases),
            "pass_count": pass_count,
            "sequence_pass": pass_count == len(cases),
        },
    }


def build_report_payload(
    *,
    output_root: Path,
    sequence_payloads: List[Dict[str, Any]],
    route_policy: str,
    invoked_command: List[str],
    test_command: str,
    test_passed: bool,
) -> Dict[str, Any]:
    overall_landed = bool(test_passed) and all((sequence.get("summary") or {}).get("sequence_pass") for sequence in sequence_payloads)
    return {
        "title": "Limited LLM Tool-Use Layer v0.1",
        "version": REPORT_VERSION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "output_root": str(output_root),
        "route_policy": route_policy,
        "source_of_truth_summary": [
            "World Graph remains the entity-truth layer.",
            "Room-centric Queryable Topology remains the derived query and routing layer.",
            "Graph search remains inside the existing NetworkX-backed Query API.",
            "Minimal VLN Closed-Loop Execution v0.1 remains the execution backend for execute requests.",
            "This deliverable adds only a thin NL interpretation and tool-selection layer on top of the current stack.",
        ],
        "what_was_implemented": [
            "A constrained NL adapter that parses narrow room, anchor, object, explanation, and execute request families.",
            "Topology-metadata target disambiguation with explicit ambiguous, unresolved, and unsupported outcomes.",
            "Backend selection that delegates to query_route(...), query_route_to_anchor(...), query_route_to_object(...), or the existing closed-loop executor.",
            "Teacher-facing normalized responses that report interpreted intent, resolved target, route summary, floor switching, and execution outcome.",
            "A real-export demo harness with JSON, CSV, Markdown, and per-case trace artifacts for 00843 and 00847.",
        ],
        "files_changed": list(IMPLEMENTATION_FILES),
        "nl_request_schema": {
            "request_type": "route_explanation | execute",
            "target_type": "room | anchor | object",
            "target_text": "raw extracted target phrase after stripping leading articles and floor hints",
            "floor_hint": "None or absolute/relative floor hint metadata",
            "mode": "query_only | execute",
            "ambiguity_state": "none | ambiguous | unresolved",
            "resolved_target": "selected entity id plus resolved room/floor metadata when available",
        },
        "tool_selection_design": [
            "Room requests dispatch to query_route(...).",
            "Anchor requests dispatch to query_route_to_anchor(...).",
            "Object requests dispatch to query_route_to_object(...), preferring object_id when local disambiguation resolves a unique object.",
            "Execute requests dispatch to VLNClosedLoopExecutor.execute(...) with the existing symbolic task_input schema.",
            "Unsupported, ambiguous, and unresolved requests stop before backend routing and report honest failure states.",
        ],
        "backend_integration": [
            "No graph search is performed in the NL layer.",
            "The NL layer resolves only target references and floor hints using exported topology metadata.",
            "Query-only requests reuse the existing Query API explanations for route and floor-switch reporting.",
            "Execute requests reuse the existing closed-loop plan + replay execution model without adding a new planner.",
        ],
        "run_commands": [
            " ".join(invoked_command),
            test_command,
        ],
        "test_passed": bool(test_passed),
        "generated_artifacts": {},
        "sequences": sequence_payloads,
        "known_limitations": [
            "Room requests are only honest for explicit room ids or unique exported room_type names; the real 00843 and 00847 exports mostly expose room ids.",
            "Anchor requests are intentionally narrow and rely on explicit anchor ids or inspectable anchor-backed object/room metadata.",
            "Object and anchor targets remain room-level destinations; the system does not claim precise docking or manipulation.",
            "Relative floor hints are interpreted against exported floor ordering and current start-room context, not open-ended dialogue reasoning.",
            "The NL layer intentionally rejects broad search, exploration, and free-form planning requests as unsupported in v0.1.",
        ],
        "final_conclusion": {
            "landed": overall_landed,
            "summary": (
                "Limited LLM Tool-Use Layer v0.1 is landed on top of the existing Query API and minimal closed-loop stack."
                if overall_landed
                else "Limited LLM Tool-Use Layer v0.1 is not fully landed yet."
            ),
        },
    }


def render_markdown_report(report: Dict[str, Any]) -> str:
    conclusion = dict(report.get("final_conclusion") or {})
    lines = [
        "# Limited LLM Tool-Use Layer v0.1",
        "",
        "## Title and Scope",
        "",
        "- scope: constrained NL interpretation + backend tool-use over the existing Query API and minimal closed-loop executor",
        f"- status: `{'LANDED' if conclusion.get('landed') else 'NOT_LANDED'}`",
        "",
        "## Background / Source-of-Truth Summary",
        "",
    ]
    for item in report.get("source_of_truth_summary", []):
        lines.append(f"- {item}")
    lines.extend(["", "## What Was Implemented", ""])
    for item in report.get("what_was_implemented", []):
        lines.append(f"- {item}")
    lines.extend(["", "## Files Changed", ""])
    for path in report.get("files_changed", []):
        lines.append(f"- `{path}`")
    lines.extend(["", "## NL Request Schema / Interpretation Design", ""])
    for key, value in (report.get("nl_request_schema") or {}).items():
        lines.append(f"- {key}: {value}")
    lines.extend(["", "## How Tool Selection Works", ""])
    for item in report.get("tool_selection_design", []):
        lines.append(f"- {item}")
    lines.extend(["", "## How Backend Integration Works", ""])
    for item in report.get("backend_integration", []):
        lines.append(f"- {item}")
    lines.extend(["", "## Exact Run Commands", ""])
    for command in report.get("run_commands", []):
        lines.append(f"- `{command}`")
    lines.extend(["", "## Generated Artifacts", ""])
    for key, value in sorted((report.get("generated_artifacts") or {}).items()):
        lines.append(f"- {key}: `{value}`")
    lines.extend(["", "## Representative Request/Response Examples", ""])
    for sequence in report.get("sequences", []):
        lines.append(f"### {sequence.get('sequence_id')}")
        lines.append("")
        for case in list(sequence.get("cases", []))[:5]:
            lines.append(f"- request: `{case.get('request_text')}`")
            lines.append(f"- result: `{case.get('status')}` / pass=`{case.get('pass')}` / tool=`{(case.get('tool_selection') or {}).get('tool_name')}`")
            lines.append(f"- teacher-facing summary: {case.get('teacher_brief')}")
            lines.append("")
    lines.extend(["## Pass/Fail Summary", ""])
    for sequence in report.get("sequences", []):
        summary = dict(sequence.get("summary") or {})
        lines.append(
            f"- {sequence.get('sequence_id')}: pass_count={summary.get('pass_count')}/{summary.get('case_count')}, sequence_pass={summary.get('sequence_pass')}"
        )
    lines.extend(["", "## Known Limitations", ""])
    for item in report.get("known_limitations", []):
        lines.append(f"- {item}")
    lines.extend(
        [
            "",
            "## Final Conclusion on Whether Limited LLM Tool-Use Layer v0.1 Is Landed",
            "",
            f"- landed: `{conclusion.get('landed')}`",
            f"- summary: {conclusion.get('summary')}",
            f"- test_passed: `{report.get('test_passed')}`",
            "",
        ]
    )
    return "\n".join(lines)


def write_case_traces(report_root: Path, sequence_payloads: Iterable[Dict[str, Any]]) -> List[str]:
    written_paths: List[str] = []
    traces_root = report_root / "traces"
    for sequence in sequence_payloads:
        sequence_id = str(sequence.get("sequence_id"))
        for case in sequence.get("cases", []):
            case_path = traces_root / sequence_id / f"{case.get('case_id')}.json"
            _json_dump(case, case_path)
            written_paths.append(str(case_path))
    return written_paths


def write_csv_summary(path: Path, sequence_payloads: Iterable[Dict[str, Any]]) -> None:
    rows: List[Dict[str, Any]] = []
    for sequence in sequence_payloads:
        for case in sequence.get("cases", []):
            teacher = dict(case.get("teacher_response") or {})
            resolved_target = dict(case.get("resolved_target") or {})
            outcome = dict((case.get("backend_result") or {}).get("outcome") or {})
            rows.append(
                {
                    "sequence_id": sequence.get("sequence_id"),
                    "case_id": case.get("case_id"),
                    "category": case.get("category"),
                    "pass": case.get("pass"),
                    "status": case.get("status"),
                    "request_type": (case.get("parsed_request") or {}).get("request_type"),
                    "target_type": (case.get("parsed_request") or {}).get("target_type"),
                    "tool_name": (case.get("tool_selection") or {}).get("tool_name"),
                    "start_room_id": case.get("start_room_id"),
                    "resolved_room_id": resolved_target.get("resolved_room_id"),
                    "resolved_display_floor_id": resolved_target.get("resolved_display_floor_id"),
                    "floor_switch_required": teacher.get("floor_switch_required"),
                    "outcome_category": outcome.get("outcome_category"),
                    "outcome_reason": outcome.get("outcome_reason"),
                }
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the limited LLM Tool-Use v0.1 demo harness on real exports.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Stage A output root containing per-sequence topology and timeline exports.",
    )
    parser.add_argument(
        "--sequences",
        nargs="+",
        default=list(DEFAULT_SEQUENCE_IDS),
        help="Sequence ids to evaluate.",
    )
    parser.add_argument(
        "--route-policy",
        default="balanced",
        help="Existing Query API route policy to reuse.",
    )
    parser.add_argument(
        "--report-dir",
        default=None,
        help="Optional explicit report directory. Defaults to <output_root>/vln_tool_use_v0_1.",
    )
    parser.add_argument(
        "--test-passed",
        default="unknown",
        help="Optional test status propagated into the report.",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    report_dir = Path(args.report_dir) if args.report_dir is not None else output_root / "vln_tool_use_v0_1"
    report_dir.mkdir(parents=True, exist_ok=True)

    sequence_payloads = [
        sequence_report(output_root=output_root, sequence_id=sequence_id)
        for sequence_id in args.sequences
    ]
    test_command = "/home/aurora/miniconda3/envs/boxfusion/bin/python boxfusion/test_vln_tool_use.py"
    test_passed = str(args.test_passed).strip().lower() == "true"
    report = build_report_payload(
        output_root=output_root,
        sequence_payloads=sequence_payloads,
        route_policy=args.route_policy,
        invoked_command=[sys.executable, *sys.argv],
        test_command=test_command,
        test_passed=test_passed,
    )

    trace_paths = write_case_traces(report_dir, sequence_payloads)
    json_path = report_dir / "vln_tool_use_v0_1.json"
    csv_path = report_dir / "vln_tool_use_v0_1.csv"
    markdown_path = report_dir / "vln_tool_use_v0_1_report.md"
    report["generated_artifacts"] = {
        "json_report": str(json_path),
        "csv_summary": str(csv_path),
        "teacher_markdown_report": str(markdown_path),
        "trace_dir": str(report_dir / "traces"),
        "trace_file_count": len(trace_paths),
    }

    _json_dump(report, json_path)
    write_csv_summary(csv_path, sequence_payloads)
    markdown_path.write_text(render_markdown_report(report), encoding="utf-8")
    print(json.dumps(report["final_conclusion"], indent=2))
    print(str(markdown_path))


if __name__ == "__main__":
    main()
