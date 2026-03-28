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
from boxfusion.vln_end_to_end_demo import VLNEndToEndDemoOrchestrator


DEFAULT_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_1/scenes")
DEFAULT_SEQUENCE_IDS = ("00843-DYehNKdT76V", "00824-Dd4bFSTQ8gi")
DEFAULT_REPORT_DIR_NAME = "end_to_end_vln_demo_v0_1"
REPORT_VERSION = "0.1"
IMPLEMENTATION_FILES = [
    "boxfusion/vln_end_to_end_demo.py",
    "stage_a_end_to_end_vln_demo.py",
    "boxfusion/test_vln_end_to_end_demo.py",
]


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_dump(payload: Dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _room_number(room_id: str) -> str:
    return str(room_id).replace("room_", "room ")


def _load_orchestrator(output_root: Path, sequence_id: str) -> Tuple[VLNEndToEndDemoOrchestrator, Path, Path]:
    sequence_root = output_root / sequence_id
    topology_json = sequence_root / "logs" / "topology_v0_1.json"
    timeline_json = sequence_root / "logs" / "timeline.json"
    orchestrator = VLNEndToEndDemoOrchestrator.from_paths(topology_json=topology_json, timeline_json=timeline_json)
    return orchestrator, topology_json, timeline_json


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


def _score_route_case(case: Dict[str, Any]) -> Tuple[int, int, str]:
    route_summary = dict(case.get("route_summary") or {})
    return (
        int(route_summary.get("hop_count") or 0),
        int(case.get("start_frame_idx") or 10**6),
        str((case.get("resolved_target") or {}).get("resolved_room_id") or ""),
    )


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


def _matches_success(
    record: Dict[str, Any],
    *,
    request_type: str,
    target_type: str,
    tool_name: str,
    needs_floor_switch: Optional[bool] = None,
    needs_vertical_transition: Optional[bool] = None,
    outcome_category: Optional[str] = None,
) -> bool:
    if str(record.get("request_type")) != request_type:
        return False
    if str(record.get("target_type")) != target_type:
        return False
    backend_call = dict(record.get("selected_backend_call") or {})
    if backend_call.get("tool_name") != tool_name:
        return False
    final_outcome = dict(record.get("final_outcome") or {})
    if final_outcome.get("status") != "success":
        return False
    if outcome_category is not None and final_outcome.get("outcome_category") != outcome_category:
        return False
    floor_switch_summary = dict(record.get("floor_switch_summary") or {})
    if needs_floor_switch is not None and bool(floor_switch_summary.get("switch_required")) != needs_floor_switch:
        return False
    vertical_summary = dict(record.get("vertical_transition_summary") or {})
    if needs_vertical_transition is not None and bool(vertical_summary.get("used")) != needs_vertical_transition:
        return False
    return True


def _same_floor_room_explanation_case(
    orchestrator: VLNEndToEndDemoOrchestrator,
    transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    query_api = orchestrator.query_api
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") != start_floor_id:
                continue
            request_text = f"How do I get to {_room_number(goal_room_id)}?"
            record = orchestrator.run_nl_request(
                request_text,
                start_room_id=start_room_id,
                start_frame_idx=start.get("frame_idx"),
            )
            if not _matches_success(
                record,
                request_type="explain",
                target_type="room",
                tool_name="query_route",
                needs_floor_switch=False,
                needs_vertical_transition=False,
            ):
                continue
            candidate = {
                "request_text": request_text,
                "start_room_id": start_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "record": record,
            }
            if best_case is None or _score_route_case(record) < _score_route_case(best_case["record"]):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a same-floor room explanation case.")
    return best_case


def _cross_floor_room_explanation_case(
    orchestrator: VLNEndToEndDemoOrchestrator,
    transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    query_api = orchestrator.query_api
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") == start_floor_id:
                continue
            direction = _room_direction(query_api, start_room_id, goal_room_id)
            request_text = (
                f"How do I get to {_room_number(goal_room_id)} {direction}?"
                if direction
                else f"How do I get to {_room_number(goal_room_id)}?"
            )
            record = orchestrator.run_nl_request(
                request_text,
                start_room_id=start_room_id,
                start_frame_idx=start.get("frame_idx"),
            )
            if not _matches_success(
                record,
                request_type="explain",
                target_type="room",
                tool_name="query_route",
                needs_floor_switch=True,
                needs_vertical_transition=True,
            ):
                continue
            candidate = {
                "request_text": request_text,
                "start_room_id": start_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "record": record,
            }
            if best_case is None or _score_route_case(record) < _score_route_case(best_case["record"]):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a cross-floor room explanation case.")
    return best_case


def _same_floor_object_execution_case(
    orchestrator: VLNEndToEndDemoOrchestrator,
    transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    query_api = orchestrator.query_api
    label_counts = _object_label_counts(query_api)
    unique_labels = sorted(label for label, rooms in label_counts.items() if len(rooms) == 1)
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for label in unique_labels:
            record = orchestrator.run_nl_request(
                f"Go to the {label}.",
                start_room_id=start_room_id,
                start_frame_idx=start.get("frame_idx"),
            )
            resolved_target = dict(record.get("resolved_target") or {})
            if resolved_target.get("resolved_room_id") == start_room_id:
                continue
            goal_room = query_api.topology.get_room(resolved_target.get("resolved_room_id")) or {}
            if goal_room.get("floor_id") != start_floor_id:
                continue
            if not _matches_success(
                record,
                request_type="execute",
                target_type="object",
                tool_name="closed_loop_execute",
                needs_floor_switch=False,
                needs_vertical_transition=False,
                outcome_category="success",
            ):
                continue
            candidate = {
                "request_text": f"Go to the {label}.",
                "start_room_id": start_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "record": record,
            }
            if best_case is None or _score_route_case(record) < _score_route_case(best_case["record"]):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a same-floor object execution case.")
    return best_case


def _cross_floor_anchor_execution_case(
    orchestrator: VLNEndToEndDemoOrchestrator,
    transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    query_api = orchestrator.query_api
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for anchor_id in query_api.topology.list_anchor_ids(valid_only=True):
            anchor_record = query_api.topology.get_anchor(anchor_id) or {}
            goal_room_id = anchor_record.get("room_id")
            if not goal_room_id or goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") == start_floor_id:
                continue
            direction = _room_direction(query_api, start_room_id, goal_room_id)
            request_text = (
                f"Go to {anchor_id} {direction}."
                if direction
                else f"Go to {anchor_id}."
            )
            record = orchestrator.run_nl_request(
                request_text,
                start_room_id=start_room_id,
                start_frame_idx=start.get("frame_idx"),
            )
            if not _matches_success(
                record,
                request_type="execute",
                target_type="anchor",
                tool_name="closed_loop_execute",
                needs_floor_switch=True,
                needs_vertical_transition=True,
                outcome_category="success",
            ):
                continue
            candidate = {
                "request_text": request_text,
                "start_room_id": start_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "record": record,
            }
            if best_case is None or _score_route_case(record) < _score_route_case(best_case["record"]):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a cross-floor anchor execution case.")
    return best_case


def _cross_floor_room_execution_case(
    orchestrator: VLNEndToEndDemoOrchestrator,
    transitions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    best_case: Optional[Dict[str, Any]] = None
    query_api = orchestrator.query_api
    for start in transitions:
        start_room_id = str(start.get("room_id"))
        start_floor_id = start.get("floor_id")
        for goal_room_id in query_api.topology.list_room_ids():
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            if goal_room.get("floor_id") == start_floor_id:
                continue
            direction = _room_direction(query_api, start_room_id, goal_room_id)
            request_text = (
                f"Start navigation to {_room_number(goal_room_id)} {direction}."
                if direction
                else f"Start navigation to {_room_number(goal_room_id)}."
            )
            record = orchestrator.run_nl_request(
                request_text,
                start_room_id=start_room_id,
                start_frame_idx=start.get("frame_idx"),
            )
            if not _matches_success(
                record,
                request_type="execute",
                target_type="room",
                tool_name="closed_loop_execute",
                needs_floor_switch=True,
                needs_vertical_transition=True,
                outcome_category="success",
            ):
                continue
            candidate = {
                "request_text": request_text,
                "start_room_id": start_room_id,
                "start_frame_idx": start.get("frame_idx"),
                "record": record,
            }
            if best_case is None or _score_route_case(record) < _score_route_case(best_case["record"]):
                best_case = candidate
    if best_case is None:
        raise RuntimeError("Could not discover a cross-floor room execution case.")
    return best_case


def _ambiguous_object_case(query_api: RoomTopologyQueryAPI, transitions: List[Dict[str, Any]]) -> Dict[str, Any]:
    label_counts = _object_label_counts(query_api)
    ambiguous_labels = sorted(label for label, rooms in label_counts.items() if len(rooms) > 1)
    if not ambiguous_labels:
        raise RuntimeError("Could not discover an ambiguous object label case.")
    return {
        "request_text": f"Go to the {ambiguous_labels[0]}.",
        "start_room_id": str(transitions[0].get("room_id")),
        "start_frame_idx": transitions[0].get("frame_idx"),
    }


def _evaluate_case(
    *,
    sequence_id: str,
    category: str,
    expectation: Dict[str, Any],
    record: Dict[str, Any],
) -> Dict[str, Any]:
    final_outcome = dict(record.get("final_outcome") or {})
    floor_switch_summary = dict(record.get("floor_switch_summary") or {})
    vertical_transition_summary = dict(record.get("vertical_transition_summary") or {})
    selected_backend_call = dict(record.get("selected_backend_call") or {})
    checks = [final_outcome.get("status") == expectation.get("status")]
    if expectation.get("request_type") is not None:
        checks.append(record.get("request_type") == expectation.get("request_type"))
    if expectation.get("target_type") is not None:
        checks.append(record.get("target_type") == expectation.get("target_type"))
    if expectation.get("tool_name") is not None:
        checks.append(selected_backend_call.get("tool_name") == expectation.get("tool_name"))
    if expectation.get("needs_floor_switch") is not None:
        checks.append(bool(floor_switch_summary.get("switch_required")) == expectation.get("needs_floor_switch"))
    if expectation.get("needs_vertical_transition") is not None:
        checks.append(bool(vertical_transition_summary.get("used")) == expectation.get("needs_vertical_transition"))
    if expectation.get("outcome_category") is not None:
        checks.append(final_outcome.get("outcome_category") == expectation.get("outcome_category"))
    result = dict(record)
    result["sequence_id"] = sequence_id
    result["category"] = category
    result["expectation"] = dict(expectation)
    result["pass"] = all(checks)
    return result


def _sequence_cases(sequence_id: str, orchestrator: VLNEndToEndDemoOrchestrator) -> List[Dict[str, Any]]:
    if orchestrator.executor is None:
        raise RuntimeError("End-to-end demo requires an attached executor.")
    transitions = _observed_transitions(orchestrator.executor)
    same_floor_room = _same_floor_room_explanation_case(orchestrator, transitions)
    cross_floor_room = _cross_floor_room_explanation_case(orchestrator, transitions)
    same_floor_object = _same_floor_object_execution_case(orchestrator, transitions)
    cross_floor_anchor = _cross_floor_anchor_execution_case(orchestrator, transitions)
    cross_floor_execute_room = _cross_floor_room_execution_case(orchestrator, transitions)
    ambiguous_case = _ambiguous_object_case(orchestrator.query_api, transitions)
    unresolved_request = {
        "request_text": "Go to the zeppelin statue.",
        "start_room_id": str(transitions[0].get("room_id")),
        "start_frame_idx": transitions[0].get("frame_idx"),
    }
    unsupported_request = {
        "request_text": "Explore every floor and choose the nicest room.",
        "start_room_id": str(transitions[0].get("room_id")),
        "start_frame_idx": transitions[0].get("frame_idx"),
    }

    cases: List[Dict[str, Any]] = []
    for category, request_data, expectation in [
        (
            "same_floor_room_explanation",
            same_floor_room,
            {
                "status": "success",
                "request_type": "explain",
                "target_type": "room",
                "tool_name": "query_route",
                "needs_floor_switch": False,
                "needs_vertical_transition": False,
                "outcome_category": "success",
            },
        ),
        (
            "cross_floor_room_explanation",
            cross_floor_room,
            {
                "status": "success",
                "request_type": "explain",
                "target_type": "room",
                "tool_name": "query_route",
                "needs_floor_switch": True,
                "needs_vertical_transition": True,
                "outcome_category": "success",
            },
        ),
        (
            "same_floor_object_execution",
            same_floor_object,
            {
                "status": "success",
                "request_type": "execute",
                "target_type": "object",
                "tool_name": "closed_loop_execute",
                "needs_floor_switch": False,
                "needs_vertical_transition": False,
                "outcome_category": "success",
            },
        ),
        (
            "cross_floor_anchor_execution",
            cross_floor_anchor,
            {
                "status": "success",
                "request_type": "execute",
                "target_type": "anchor",
                "tool_name": "closed_loop_execute",
                "needs_floor_switch": True,
                "needs_vertical_transition": True,
                "outcome_category": "success",
            },
        ),
        (
            "cross_floor_room_execution",
            cross_floor_execute_room,
            {
                "status": "success",
                "request_type": "execute",
                "target_type": "room",
                "tool_name": "closed_loop_execute",
                "needs_floor_switch": True,
                "needs_vertical_transition": True,
                "outcome_category": "success",
            },
        ),
        (
            "ambiguous_target_case",
            ambiguous_case,
            {
                "status": "ambiguous",
                "request_type": "execute",
                "target_type": "object",
                "tool_name": None,
                "needs_floor_switch": None,
                "needs_vertical_transition": None,
                "outcome_category": None,
            },
        ),
        (
            "unresolved_target_case",
            unresolved_request,
            {
                "status": "unresolved",
                "request_type": "execute",
                "target_type": "object",
                "tool_name": None,
                "needs_floor_switch": None,
                "needs_vertical_transition": None,
                "outcome_category": None,
            },
        ),
        (
            "unsupported_request_case",
            unsupported_request,
            {
                "status": "unsupported",
                "request_type": None,
                "target_type": None,
                "tool_name": None,
                "needs_floor_switch": None,
                "needs_vertical_transition": None,
                "outcome_category": None,
            },
        ),
    ]:
        record = request_data["record"] if "record" in request_data else orchestrator.run_nl_request(
            request_data["request_text"],
            sequence_id=sequence_id,
            start_room_id=request_data["start_room_id"],
            start_frame_idx=request_data["start_frame_idx"],
            case_id=f"{sequence_id}_{category}",
        )
        if "record" in request_data:
            record = dict(record)
            record["sequence_id"] = sequence_id
            record["case_id"] = f"{sequence_id}_{category}"
        cases.append(
            _evaluate_case(
                sequence_id=sequence_id,
                category=category,
                expectation=expectation,
                record=record,
            )
        )
    return cases


def sequence_report(output_root: Path, sequence_id: str) -> Dict[str, Any]:
    orchestrator, topology_json, timeline_json = _load_orchestrator(output_root, sequence_id)
    cases = _sequence_cases(sequence_id, orchestrator)
    pass_count = sum(1 for case in cases if case.get("pass"))
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
    report_dir: Path,
    sequence_payloads: List[Dict[str, Any]],
    invoked_command: List[str],
    test_command: str,
    test_passed: bool,
) -> Dict[str, Any]:
    overall_landed = bool(test_passed) and all((sequence.get("summary") or {}).get("sequence_pass") for sequence in sequence_payloads)
    return {
        "title": "End-to-End VLN World-Model Demo Stack v0.1",
        "version": REPORT_VERSION,
        "generated_at_utc": _utc_now_iso(),
        "output_root": str(output_root),
        "report_dir": str(report_dir),
        "background_current_landed_layers": [
            "floor-aware World Model / Room Segmentation v0.1",
            "multi-floor Query API",
            "Minimal VLN Closed-Loop Execution v0.1",
            "Limited LLM Tool-Use Layer v0.1",
        ],
        "what_was_composed": [
            "Natural-language request intake through the existing constrained tool-use adapter.",
            "Structured task handoff into the existing Query API and closed-loop executor without moving planning logic upward.",
            "Unified teacher-facing records that show interpreted request, resolved target, route summary, floor switches, vertical transitions, and execution outcome.",
            "A real-export harness on 00843-DYehNKdT76V and 00824-Dd4bFSTQ8gi with honest ambiguous, unresolved, and unsupported coverage.",
        ],
        "architecture_flow": [
            "NL request -> constrained interpretation / tool-use",
            "constrained interpretation / tool-use -> structured task",
            "structured task -> query / route / target resolution",
            "structured task -> symbolic execution for execute requests",
            "backend result -> unified teacher-facing trace / report / artifacts",
        ],
        "files_changed": list(IMPLEMENTATION_FILES),
        "run_commands": [
            " ".join(invoked_command),
            test_command,
        ],
        "test_command": test_command,
        "test_passed": bool(test_passed),
        "sequences": sequence_payloads,
        "generated_artifacts": {},
        "known_limitations": [
            "The orchestration layer remains thin and does not introduce a new planner, graph search stack, or free-form agent loop.",
            "Room requests remain strongest for explicit room ids because the real exports expose limited stable room_type metadata.",
            "Anchor and object targets remain room-level destinations rather than precise docking or manipulation endpoints.",
            "Cross-floor behavior is only as explicit as the exported vertical_transition edges and replay observations.",
            "Unsupported, ambiguous, and unresolved cases are surfaced honestly instead of being auto-corrected by broader language reasoning.",
        ],
        "final_conclusion": {
            "landed": overall_landed,
            "summary": (
                "This is enough to claim an End-to-End VLN World-Model Demo Stack v0.1 is landed."
                if overall_landed
                else "This is not yet enough to claim an End-to-End VLN World-Model Demo Stack v0.1 is landed."
            ),
        },
    }


def _render_case_block(case: Dict[str, Any]) -> List[str]:
    resolved_target = dict(case.get("resolved_target") or {})
    route_summary = dict(case.get("route_summary") or {})
    floor_switch_summary = dict(case.get("floor_switch_summary") or {})
    final_outcome = dict(case.get("final_outcome") or {})
    lines = [
        f"- case: `{case.get('category')}` / pass=`{case.get('pass')}`",
        f"- request: `{case.get('raw_request')}`",
        f"- interpreted_request: `{case.get('request_type')}` / `{case.get('target_type')}`",
        (
            f"- resolved_target: `{resolved_target.get('selected_entity_id') or resolved_target.get('selected_label')}` "
            f"-> `{resolved_target.get('resolved_room_id')}` on `{resolved_target.get('resolved_display_floor_id')}`"
            if resolved_target
            else "- resolved_target: `None`"
        ),
        f"- backend_call: `{(case.get('selected_backend_call') or {}).get('tool_name')}`",
        f"- route_summary: {route_summary.get('summary')}",
        f"- floor_switch_summary: {floor_switch_summary.get('summary')}",
        (
            f"- execution_outcome: `{final_outcome.get('outcome_category')}` / `{final_outcome.get('outcome_reason')}`"
            if case.get("request_type") == "execute"
            else f"- final_status: `{final_outcome.get('status')}`"
        ),
        f"- teacher_summary: {((case.get('teacher_facing_record') or {}).get('summary'))}",
    ]
    return lines


def render_markdown_report(report: Dict[str, Any]) -> str:
    conclusion = dict(report.get("final_conclusion") or {})
    lines = [
        "# End-to-End VLN World-Model Demo Stack v0.1",
        "",
        "## Title and Scope",
        "",
        "- scope: unified teacher-facing composition across the existing world model, topology query API, limited NL tool-use layer, and minimal closed-loop executor",
        f"- status: `{'LANDED' if conclusion.get('landed') else 'NOT_LANDED'}`",
        "",
        "## Background / Current Landed Layers",
        "",
    ]
    for item in report.get("background_current_landed_layers", []):
        lines.append(f"- {item}")
    lines.extend(["", "## What Was Composed", ""])
    for item in report.get("what_was_composed", []):
        lines.append(f"- {item}")
    lines.extend(["", "## Files Changed", ""])
    for path in report.get("files_changed", []):
        lines.append(f"- `{path}`")
    lines.extend(["", "## End-to-End Architecture Flow", ""])
    for item in report.get("architecture_flow", []):
        lines.append(f"- {item}")
    lines.extend(["", "## Exact Run Commands", ""])
    for command in report.get("run_commands", []):
        lines.append(f"- `{command}`")
    lines.extend(["", "## Generated Artifact Paths", ""])
    for key, value in sorted((report.get("generated_artifacts") or {}).items()):
        lines.append(f"- {key}: `{value}`")
    lines.extend(["", "## Representative Example Cases", ""])
    for sequence in report.get("sequences", []):
        lines.append(f"### {sequence.get('sequence_id')}")
        lines.append("")
        for case in sequence.get("cases", []):
            lines.extend(_render_case_block(case))
            lines.append("")
    lines.extend(["## Execution / Explanation Examples", ""])
    for sequence in report.get("sequences", []):
        explain_case = next((case for case in sequence.get("cases", []) if case.get("request_type") == "explain"), None)
        execute_case = next((case for case in sequence.get("cases", []) if case.get("request_type") == "execute" and case.get("category") == "cross_floor_room_execution"), None)
        lines.append(f"- {sequence.get('sequence_id')} explanation example: {((explain_case or {}).get('teacher_facing_record') or {}).get('summary')}")
        lines.append(f"- {sequence.get('sequence_id')} execution example: {((execute_case or {}).get('teacher_facing_record') or {}).get('summary')}")
    lines.extend(["", "## Pass/Fail Summary", ""])
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
            "## Final Conclusion",
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
            final_outcome = dict(case.get("final_outcome") or {})
            route_summary = dict(case.get("route_summary") or {})
            resolved_target = dict(case.get("resolved_target") or {})
            floor_switch_summary = dict(case.get("floor_switch_summary") or {})
            vertical_transition_summary = dict(case.get("vertical_transition_summary") or {})
            rows.append(
                {
                    "sequence_id": sequence.get("sequence_id"),
                    "case_id": case.get("case_id"),
                    "category": case.get("category"),
                    "pass": case.get("pass"),
                    "request_type": case.get("request_type"),
                    "target_type": case.get("target_type"),
                    "status": final_outcome.get("status"),
                    "tool_name": (case.get("selected_backend_call") or {}).get("tool_name"),
                    "resolved_room_id": resolved_target.get("resolved_room_id"),
                    "resolved_display_floor_id": resolved_target.get("resolved_display_floor_id"),
                    "hop_count": route_summary.get("hop_count"),
                    "floor_switch_required": floor_switch_summary.get("switch_required"),
                    "vertical_transition_used": vertical_transition_summary.get("used"),
                    "outcome_category": final_outcome.get("outcome_category"),
                    "outcome_reason": final_outcome.get("outcome_reason"),
                }
            )
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the end-to-end VLN world-model demo stack harness on real exports.")
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
        "--report-dir",
        default=None,
        help="Optional explicit report directory. Defaults to <output_root>/end_to_end_vln_demo_v0_1.",
    )
    parser.add_argument(
        "--test-passed",
        default="unknown",
        help="Optional test status propagated into the report.",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    report_dir = Path(args.report_dir) if args.report_dir is not None else output_root / DEFAULT_REPORT_DIR_NAME
    report_dir.mkdir(parents=True, exist_ok=True)

    sequence_payloads = [
        sequence_report(output_root=output_root, sequence_id=sequence_id)
        for sequence_id in args.sequences
    ]
    test_command = "/home/aurora/miniconda3/envs/boxfusion/bin/python boxfusion/test_vln_end_to_end_demo.py"
    report = build_report_payload(
        output_root=output_root,
        report_dir=report_dir,
        sequence_payloads=sequence_payloads,
        invoked_command=[sys.executable, *sys.argv],
        test_command=test_command,
        test_passed=str(args.test_passed).strip().lower() == "true",
    )

    trace_paths = write_case_traces(report_dir, sequence_payloads)
    json_path = report_dir / "end_to_end_vln_demo_v0_1.json"
    csv_path = report_dir / "end_to_end_vln_demo_v0_1.csv"
    markdown_path = report_dir / "end_to_end_vln_demo_v0_1_report.md"
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
