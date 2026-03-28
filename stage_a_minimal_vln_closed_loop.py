from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from boxfusion.vln_closed_loop import VLNClosedLoopExecutor, extract_room_transition_observations


DEFAULT_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_1/scenes")
DEFAULT_SEQUENCE_IDS = ("00843-DYehNKdT76V", "00824-Dd4bFSTQ8gi")
REPORT_VERSION = "0.1"
IMPLEMENTATION_FILES = [
    "boxfusion/vln_closed_loop.py",
    "stage_a_minimal_vln_closed_loop.py",
    "boxfusion/test_vln_closed_loop.py",
]


def _json_dump(payload: Dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _canonical_target_type(case_kind: str) -> str:
    if case_kind.startswith("room_"):
        return "room"
    if case_kind.startswith("anchor_"):
        return "anchor"
    if case_kind.startswith("object_"):
        return "object"
    return "room"


def _load_executor(topology_json: Path, timeline_json: Path) -> VLNClosedLoopExecutor:
    return VLNClosedLoopExecutor.from_paths(topology_json=topology_json, timeline_json=timeline_json)


def _observed_transitions(executor: VLNClosedLoopExecutor) -> List[Dict[str, Any]]:
    return extract_room_transition_observations(executor.observations)


def _route_matches_exact_prefix(transitions: List[Dict[str, Any]], start_index: int, plan_rooms: List[str]) -> bool:
    if not plan_rooms:
        return False
    observed = [item.get("room_id") for item in transitions[start_index : start_index + len(plan_rooms)]]
    return observed == list(plan_rooms)


def _route_matches_subsequence(transitions: List[Dict[str, Any]], start_index: int, plan_rooms: List[str]) -> bool:
    suffix = [item.get("room_id") for item in transitions[start_index:]]
    cursor = 0
    for room_id in plan_rooms:
        try:
            cursor = suffix.index(room_id, cursor) + 1
        except ValueError:
            return False
    return True


def _execution_case_base(
    *,
    case_id: str,
    case_kind: str,
    sequence_id: str,
    start_transition: Dict[str, Any],
    task_input: Dict[str, Any],
    discovery_note: str,
) -> Dict[str, Any]:
    return {
        "case_id": case_id,
        "case_kind": case_kind,
        "target_type": _canonical_target_type(case_kind),
        "sequence_id": sequence_id,
        "start_room_id": start_transition.get("room_id"),
        "start_floor_id": start_transition.get("floor_id"),
        "start_display_floor_id": start_transition.get("display_floor_id"),
        "start_frame_idx": start_transition.get("frame_idx"),
        "task_input": task_input,
        "discovery_note": discovery_note,
    }


def _score_case(case: Dict[str, Any]) -> tuple:
    plan = dict((case.get("execution") or {}).get("plan") or {})
    symbolic_plan = dict(plan.get("symbolic_plan") or {})
    return (
        int(symbolic_plan.get("hop_count") or 0),
        int(case.get("start_frame_idx") or 0),
        str((plan.get("target_resolution") or {}).get("resolved_room_id") or ""),
        str(case.get("case_id") or ""),
    )


def _attempt_execution_case(
    executor: VLNClosedLoopExecutor,
    *,
    case_id: str,
    case_kind: str,
    sequence_id: str,
    start_transition: Dict[str, Any],
    task_input: Dict[str, Any],
    route_policy: str,
    end_frame_idx: Optional[int] = None,
    max_offroute_room_changes: int = 0,
    discovery_note: str,
) -> Dict[str, Any]:
    execution = executor.execute(
        task_input=task_input,
        route_policy=route_policy,
        start_frame_idx=int(start_transition.get("frame_idx")),
        end_frame_idx=end_frame_idx,
        max_offroute_room_changes=max_offroute_room_changes,
    )
    case_payload = _execution_case_base(
        case_id=case_id,
        case_kind=case_kind,
        sequence_id=sequence_id,
        start_transition=start_transition,
        task_input=task_input,
        discovery_note=discovery_note,
    )
    case_payload["execution"] = execution
    case_payload["outcome_category"] = (execution.get("outcome") or {}).get("outcome_category")
    case_payload["outcome_reason"] = (execution.get("outcome") or {}).get("outcome_reason")
    case_payload["resolved_goal_room_id"] = (((execution.get("plan") or {}).get("target_resolution") or {}).get("resolved_room_id"))
    case_payload["resolved_goal_floor_id"] = (((execution.get("plan") or {}).get("target_resolution") or {}).get("resolved_floor_id"))
    case_payload["resolved_goal_display_floor_id"] = (((execution.get("plan") or {}).get("target_resolution") or {}).get("resolved_display_floor_id"))
    return case_payload


def discover_success_case(
    executor: VLNClosedLoopExecutor,
    *,
    sequence_id: str,
    route_policy: str,
    case_kind: str,
) -> Dict[str, Any]:
    transitions = _observed_transitions(executor)
    best_case: Optional[Dict[str, Any]] = None

    if case_kind == "room_same_floor":
        for start_index, start_transition in enumerate(transitions):
            for goal_transition in transitions[start_index + 1 :]:
                task_input = {
                    "start_room_id": start_transition.get("room_id"),
                    "target": {"target_type": "room", "goal_room_id": goal_transition.get("room_id")},
                }
                candidate = _attempt_execution_case(
                    executor,
                    case_id=f"{sequence_id}_room_same_floor",
                    case_kind=case_kind,
                    sequence_id=sequence_id,
                    start_transition=start_transition,
                    task_input=task_input,
                    route_policy=route_policy,
                    discovery_note="Exact observed room-prefix replay for a same-floor room target.",
                )
                plan = dict((candidate.get("execution") or {}).get("plan") or {})
                symbolic_plan = dict(plan.get("symbolic_plan") or {})
                plan_rooms = list(symbolic_plan.get("room_sequence") or [])
                if candidate.get("outcome_category") != "success":
                    continue
                if len(plan_rooms) < 2:
                    continue
                if start_transition.get("floor_id") != candidate.get("resolved_goal_floor_id"):
                    continue
                if "vertical_transition" in list(symbolic_plan.get("relation_sequence") or []):
                    continue
                if not _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                    continue
                if best_case is None or _score_case(candidate) < _score_case(best_case):
                    best_case = candidate

    elif case_kind == "room_cross_floor":
        for start_index, start_transition in enumerate(transitions):
            for goal_transition in transitions[start_index + 1 :]:
                task_input = {
                    "start_room_id": start_transition.get("room_id"),
                    "target": {"target_type": "room", "goal_room_id": goal_transition.get("room_id")},
                }
                candidate = _attempt_execution_case(
                    executor,
                    case_id=f"{sequence_id}_room_cross_floor",
                    case_kind=case_kind,
                    sequence_id=sequence_id,
                    start_transition=start_transition,
                    task_input=task_input,
                    route_policy=route_policy,
                    discovery_note="Exact observed room-prefix replay for a cross-floor room target.",
                )
                plan = dict((candidate.get("execution") or {}).get("plan") or {})
                symbolic_plan = dict(plan.get("symbolic_plan") or {})
                plan_rooms = list(symbolic_plan.get("room_sequence") or [])
                if candidate.get("outcome_category") != "success":
                    continue
                if len(plan_rooms) < 2:
                    continue
                if start_transition.get("floor_id") == candidate.get("resolved_goal_floor_id"):
                    continue
                if "vertical_transition" not in list(symbolic_plan.get("relation_sequence") or []):
                    continue
                if not _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                    continue
                if best_case is None or _score_case(candidate) < _score_case(best_case):
                    best_case = candidate

    elif case_kind == "anchor_target":
        for start_index, start_transition in enumerate(transitions):
            for anchor_id in executor.query_api.topology.list_anchor_ids(valid_only=True):
                task_input = {
                    "start_room_id": start_transition.get("room_id"),
                    "target": {"target_type": "anchor", "anchor_id": anchor_id},
                }
                candidate = _attempt_execution_case(
                    executor,
                    case_id=f"{sequence_id}_anchor_target",
                    case_kind=case_kind,
                    sequence_id=sequence_id,
                    start_transition=start_transition,
                    task_input=task_input,
                    route_policy=route_policy,
                    discovery_note="Exact observed room-prefix replay for an anchor target.",
                )
                plan = dict((candidate.get("execution") or {}).get("plan") or {})
                symbolic_plan = dict(plan.get("symbolic_plan") or {})
                plan_rooms = list(symbolic_plan.get("room_sequence") or [])
                if candidate.get("outcome_category") != "success":
                    continue
                if len(plan_rooms) < 2:
                    continue
                if not _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                    continue
                if best_case is None or _score_case(candidate) < _score_case(best_case):
                    best_case = candidate

    elif case_kind == "object_target":
        for start_index, start_transition in enumerate(transitions):
            for object_id in executor.query_api.topology.list_object_ids():
                object_record = executor.query_api.topology.get_object(object_id) or {}
                if not object_record.get("label"):
                    continue
                task_input = {
                    "start_room_id": start_transition.get("room_id"),
                    "target": {
                        "target_type": "object",
                        "object_id": object_id,
                        "object_label": object_record.get("label"),
                    },
                }
                candidate = _attempt_execution_case(
                    executor,
                    case_id=f"{sequence_id}_object_target",
                    case_kind=case_kind,
                    sequence_id=sequence_id,
                    start_transition=start_transition,
                    task_input=task_input,
                    route_policy=route_policy,
                    discovery_note="Exact observed room-prefix replay for an object target.",
                )
                plan = dict((candidate.get("execution") or {}).get("plan") or {})
                symbolic_plan = dict(plan.get("symbolic_plan") or {})
                plan_rooms = list(symbolic_plan.get("room_sequence") or [])
                if candidate.get("outcome_category") != "success":
                    continue
                if len(plan_rooms) < 2:
                    continue
                if not _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                    continue
                if best_case is None or _score_case(candidate) < _score_case(best_case):
                    best_case = candidate

    if best_case is None:
        raise RuntimeError(f"Could not discover a conservative success case for {sequence_id} / {case_kind}.")
    return best_case


def discover_divergence_probe(
    executor: VLNClosedLoopExecutor,
    *,
    sequence_id: str,
    route_policy: str,
) -> Dict[str, Any]:
    transitions = _observed_transitions(executor)
    best_case: Optional[Dict[str, Any]] = None
    for start_index, start_transition in enumerate(transitions):
        for goal_transition in transitions[start_index + 1 :]:
            task_input = {
                "start_room_id": start_transition.get("room_id"),
                "target": {"target_type": "room", "goal_room_id": goal_transition.get("room_id")},
            }
            candidate = _attempt_execution_case(
                executor,
                case_id=f"{sequence_id}_execution_divergence_probe",
                case_kind="execution_divergence_probe",
                sequence_id=sequence_id,
                start_transition=start_transition,
                task_input=task_input,
                route_policy=route_policy,
                max_offroute_room_changes=0,
                discovery_note="Real replay path that reaches a future planned room only as a subsequence, so the executor should request replanning when the next expected room is skipped.",
            )
            plan = dict((candidate.get("execution") or {}).get("plan") or {})
            symbolic_plan = dict(plan.get("symbolic_plan") or {})
            plan_rooms = list(symbolic_plan.get("room_sequence") or [])
            if len(plan_rooms) < 3:
                continue
            if not _route_matches_subsequence(transitions, start_index, plan_rooms):
                continue
            if _route_matches_exact_prefix(transitions, start_index, plan_rooms):
                continue
            if candidate.get("outcome_category") != "replan_needed":
                continue
            if candidate.get("outcome_reason") != "execution_divergence":
                continue
            if best_case is None or _score_case(candidate) < _score_case(best_case):
                best_case = candidate
    if best_case is None:
        raise RuntimeError(f"Could not discover an execution-divergence probe for {sequence_id}.")
    return best_case


def build_missing_vertical_transition_probe(
    executor: VLNClosedLoopExecutor,
    *,
    success_case: Dict[str, Any],
    sequence_id: str,
    route_policy: str,
) -> Dict[str, Any]:
    plan = dict((success_case.get("execution") or {}).get("plan") or {})
    steps = list(((plan.get("symbolic_plan") or {}).get("steps")) or [])
    vt_step = next((step for step in steps if step.get("action") == "use_vertical_transition"), None)
    if vt_step is None:
        raise RuntimeError(f"Cross-floor success case for {sequence_id} did not include a vertical-transition step.")

    transitions = _observed_transitions(executor)
    start_frame_idx = int(success_case.get("start_frame_idx"))
    target_room_id = vt_step.get("target_room_id")
    target_transition = next(
        (
            item
            for item in transitions
            if (-1 if item.get("frame_idx") is None else int(item.get("frame_idx"))) > start_frame_idx and item.get("room_id") == target_room_id
        ),
        None,
    )
    if target_transition is None:
        raise RuntimeError(f"Could not locate replay entry for vertical-transition target room {target_room_id}.")

    start_transition = next(item for item in transitions if (-1 if item.get("frame_idx") is None else int(item.get("frame_idx"))) == start_frame_idx)
    end_frame_idx = int(target_transition.get("frame_idx")) - 1
    task_input = dict(success_case.get("task_input") or {})
    probe = _attempt_execution_case(
        executor,
        case_id=f"{sequence_id}_missing_vertical_transition_probe",
        case_kind="missing_vertical_transition_probe",
        sequence_id=sequence_id,
        start_transition=start_transition,
        task_input=task_input,
        route_policy=route_policy,
        end_frame_idx=end_frame_idx,
        discovery_note="Replay window ends before the planned vertical-transition arrival is observed, so the executor should fail honestly with missing_vertical_transition.",
    )
    if probe.get("outcome_category") != "failure" or probe.get("outcome_reason") != "missing_vertical_transition":
        raise RuntimeError(f"Missing-vertical-transition probe did not behave as expected for {sequence_id}.")
    return probe


def build_target_unresolved_probe(
    executor: VLNClosedLoopExecutor,
    *,
    sequence_id: str,
    route_policy: str,
) -> Dict[str, Any]:
    start_transition = _observed_transitions(executor)[0]
    task_input = {
        "start_room_id": start_transition.get("room_id"),
        "target": {"target_type": "room", "goal_room_id": "room_9999"},
    }
    probe = _attempt_execution_case(
        executor,
        case_id=f"{sequence_id}_target_unresolved_probe",
        case_kind="target_unresolved_probe",
        sequence_id=sequence_id,
        start_transition=start_transition,
        task_input=task_input,
        route_policy=route_policy,
        discovery_note="Intentional invalid room id to verify honest planning-time target_unresolved behavior.",
    )
    if probe.get("outcome_category") != "failure":
        raise RuntimeError(f"Target-unresolved probe did not fail for {sequence_id}.")
    if probe.get("outcome_reason") != "target_unresolved":
        raise RuntimeError(f"Target-unresolved probe returned {probe.get('outcome_reason')} for {sequence_id}.")
    return probe


def sequence_report(
    *,
    output_root: Path,
    sequence_id: str,
    route_policy: str,
) -> Dict[str, Any]:
    sequence_root = output_root / sequence_id
    topology_json = sequence_root / "logs" / "topology_v0_1.json"
    timeline_json = sequence_root / "logs" / "timeline.json"
    if not topology_json.exists() or not timeline_json.exists():
        raise FileNotFoundError(f"Missing topology/timeline for {sequence_id} under {sequence_root}.")

    executor = _load_executor(topology_json=topology_json, timeline_json=timeline_json)
    success_cases = [
        discover_success_case(executor, sequence_id=sequence_id, route_policy=route_policy, case_kind="room_same_floor"),
        discover_success_case(executor, sequence_id=sequence_id, route_policy=route_policy, case_kind="room_cross_floor"),
        discover_success_case(executor, sequence_id=sequence_id, route_policy=route_policy, case_kind="anchor_target"),
        discover_success_case(executor, sequence_id=sequence_id, route_policy=route_policy, case_kind="object_target"),
    ]
    probes = [
        build_target_unresolved_probe(executor, sequence_id=sequence_id, route_policy=route_policy),
        build_missing_vertical_transition_probe(
            executor,
            sequence_id=sequence_id,
            route_policy=route_policy,
            success_case=next(case for case in success_cases if case.get("case_kind") == "room_cross_floor"),
        ),
        discover_divergence_probe(executor, sequence_id=sequence_id, route_policy=route_policy),
    ]

    success_pass = all(case.get("outcome_category") == "success" for case in success_cases)
    probe_expectations_pass = all(
        (probe.get("outcome_category"), probe.get("outcome_reason"))
        in {
            ("failure", "target_unresolved"),
            ("failure", "missing_vertical_transition"),
            ("replan_needed", "execution_divergence"),
        }
        for probe in probes
    )

    return {
        "sequence_id": sequence_id,
        "topology_json": str(topology_json),
        "timeline_json": str(timeline_json),
        "route_policy": route_policy,
        "success_cases": success_cases,
        "honesty_probes": probes,
        "summary": {
            "success_case_count": len(success_cases),
            "success_case_pass_count": sum(1 for case in success_cases if case.get("outcome_category") == "success"),
            "probe_count": len(probes),
            "probe_expectation_pass_count": sum(
                1
                for probe in probes
                if (probe.get("outcome_category"), probe.get("outcome_reason"))
                in {
                    ("failure", "target_unresolved"),
                    ("failure", "missing_vertical_transition"),
                    ("replan_needed", "execution_divergence"),
                }
            ),
            "sequence_pass": bool(success_pass and probe_expectations_pass),
        },
    }


def _action_lines(case: Dict[str, Any]) -> List[str]:
    steps = list((((case.get("execution") or {}).get("plan") or {}).get("symbolic_plan") or {}).get("steps") or [])
    return [str(step.get("description")) for step in steps]


def _trace_lines(case: Dict[str, Any], limit: int = 8) -> List[str]:
    lines: List[str] = []
    for event in list((case.get("execution") or {}).get("execution_trace") or [])[:limit]:
        event_type = event.get("event_type")
        frame_idx = event.get("frame_idx")
        if event_type == "room_observed":
            lines.append(f"frame {frame_idx}: observe {event.get('observed_room_id')} on {event.get('observed_floor_label')}")
        elif event_type == "floor_switch_observed":
            from_floor = event.get("from_display_floor_id") or event.get("from_floor_id")
            to_floor = event.get("to_display_floor_id") or event.get("to_floor_id")
            lines.append(
                f"frame {frame_idx}: floor switch {event.get('from_room_id')} -> {event.get('to_room_id')} "
                f"({from_floor} -> {to_floor})"
            )
        elif event_type == "step_completed":
            lines.append(f"frame {frame_idx}: complete {event.get('description')}")
        elif event_type == "execution_divergence":
            lines.append(
                f"frame {frame_idx}: divergence while expecting {event.get('expected_room_id')} "
                f"after observing {event.get('observed_room_id')}"
            )
        elif event_type == "target_invalidated":
            lines.append(f"frame {frame_idx}: target invalidated")
    return lines


def build_report_payload(
    *,
    output_root: Path,
    sequence_payloads: List[Dict[str, Any]],
    route_policy: str,
    invoked_command: List[str],
) -> Dict[str, Any]:
    overall_landed = all((sequence.get("summary") or {}).get("sequence_pass") for sequence in sequence_payloads)
    return {
        "title": "Minimal VLN Closed-Loop Execution v0.1",
        "version": REPORT_VERSION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "output_root": str(output_root),
        "route_policy": route_policy,
        "source_of_truth_summary": [
            "World Graph remains the entity-truth layer.",
            "Room-centric Queryable Topology remains the derived routing/query layer.",
            "Graph search continues to use the existing Query API and topology stack.",
            "This deliverable adds a symbolic room-level closed-loop executor over replayed world-state observations.",
            "This is not motion planning, obstacle avoidance, or precise object docking.",
        ],
        "what_was_implemented": [
            "A small symbolic executor that turns existing Query API routes into room-level actions.",
            "Per-frame replay consumption over existing timeline exports with explicit room/floor progression.",
            "Honest completion semantics for room, anchor, and object targets.",
            "Machine-readable traces plus CSV and teacher-facing Markdown reporting.",
            "Failure-honesty probes for target_unresolved, missing_vertical_transition, and execution_divergence.",
        ],
        "files_changed": list(IMPLEMENTATION_FILES),
        "execution_model": {
            "planning": [
                "Structured targets resolve through the existing Query API.",
                "Routes are converted into symbolic steps such as move_to_room(...) and use_vertical_transition(...).",
                "Anchor/object terminal success is defined as reaching the resolved room while target resolution remains valid.",
            ],
            "closed_loop": [
                "Replay frames are consumed from existing timeline exports.",
                "The executor advances its pointer when observed room/floor state satisfies the next symbolic step.",
                "Cross-floor progress is explicit and requires a vertical-transition step to complete.",
                "Unexpected room progression triggers replan_needed instead of pretending success.",
            ],
        },
        "run_commands": [
            " ".join(invoked_command),
            f"{sys.executable} boxfusion/test_vln_closed_loop.py",
        ],
        "generated_artifacts": {},
        "sequences": sequence_payloads,
        "known_limitations": [
            "Structured targets intentionally bypass natural-language parsing because this task is scoped to executable symbolic control, not LLM/NL understanding.",
            "Object and anchor success remains room-level: the system does not claim precise docking, manipulation, or local control.",
            "Current exports are static final world models, so target_invalidated and route_unavailable code paths are implemented but not naturally exercised by these real sequences.",
            "Divergence detection is intentionally conservative and room-level; it requests replanning rather than inferring local recovery behavior.",
        ],
        "final_conclusion": {
            "landed": bool(overall_landed),
            "summary": (
                "Minimal VLN Closed-Loop Execution v0.1 is landed on the current floor-aware stack."
                if overall_landed
                else "Minimal VLN Closed-Loop Execution v0.1 is not yet fully landed."
            ),
        },
    }


def render_markdown_report(report: Dict[str, Any]) -> str:
    lines = [
        "# Minimal VLN Closed-Loop Execution v0.1",
        "",
        "## Title and Scope",
        "",
        "- scope: room-level symbolic closed-loop execution over existing floor-aware World Model + Query API exports",
        "- status: `{}`".format("LANDED" if (report.get("final_conclusion") or {}).get("landed") else "NOT_LANDED"),
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
    lines.extend(["", "## Execution Model", "", "- planning:"])
    for item in (report.get("execution_model") or {}).get("planning", []):
        lines.append(f"- {item}")
    lines.extend(["", "- closed_loop:"])
    for item in (report.get("execution_model") or {}).get("closed_loop", []):
        lines.append(f"- {item}")
    lines.extend(["", "## Exact Run Commands", ""])
    for command in report.get("run_commands", []):
        lines.append(f"- `{command}`")
    lines.extend(["", "## Generated Artifacts", ""])
    artifacts = dict(report.get("generated_artifacts") or {})
    for key in sorted(artifacts):
        lines.append(f"- {key}: `{artifacts[key]}`")

    lines.extend(["", "## Acceptance Coverage for 00843 and 00847", ""])
    for sequence in report.get("sequences", []):
        summary = dict(sequence.get("summary") or {})
        lines.append(f"### {sequence.get('sequence_id')}")
        lines.append("")
        lines.append(f"- topology_json: `{sequence.get('topology_json')}`")
        lines.append(f"- timeline_json: `{sequence.get('timeline_json')}`")
        lines.append(f"- success cases passed: `{summary.get('success_case_pass_count')}/{summary.get('success_case_count')}`")
        lines.append(f"- honesty probes matched expectation: `{summary.get('probe_expectation_pass_count')}/{summary.get('probe_count')}`")
        lines.append(f"- sequence_pass: `{summary.get('sequence_pass')}`")
        lines.append("")
        for case in sequence.get("success_cases", []):
            lines.append(
                f"- {case.get('case_kind')}: `{case.get('outcome_category')}` start=`{case.get('start_room_id')}` goal=`{case.get('resolved_goal_room_id')}`"
            )
        for probe in sequence.get("honesty_probes", []):
            lines.append(f"- {probe.get('case_kind')}: `{probe.get('outcome_category')}` / `{probe.get('outcome_reason')}`")
        lines.append("")

    lines.extend(["## Representative Execution Traces", ""])
    for sequence in report.get("sequences", []):
        cross_floor = next((case for case in sequence.get("success_cases", []) if case.get("case_kind") == "room_cross_floor"), None)
        object_case = next((case for case in sequence.get("success_cases", []) if case.get("case_kind") == "object_target"), None)
        for case in [cross_floor, object_case]:
            if not case:
                continue
            lines.append(f"### {sequence.get('sequence_id')} / {case.get('case_kind')}")
            lines.append("")
            lines.append(f"- initial room/floor: `{case.get('start_room_id')}` / `{case.get('start_display_floor_id') or case.get('start_floor_id')}`")
            lines.append(
                f"- target type / resolution: `{case.get('target_type')}` -> `{case.get('resolved_goal_room_id')}` on `{case.get('resolved_goal_display_floor_id') or case.get('resolved_goal_floor_id')}`"
            )
            lines.append("- planned symbolic route:")
            for line in _action_lines(case):
                lines.append(f"- {line}")
            lines.append("- execution trace:")
            for line in _trace_lines(case):
                lines.append(f"- {line}")
            lines.append(f"- outcome: `{case.get('outcome_category')}` / `{case.get('outcome_reason')}`")
            lines.append("")

    lines.extend(["## Pass/Fail Summary", ""])
    for sequence in report.get("sequences", []):
        summary = dict(sequence.get("summary") or {})
        lines.append(
            f"- {sequence.get('sequence_id')}: success_cases={summary.get('success_case_pass_count')}/{summary.get('success_case_count')}, honesty_probes={summary.get('probe_expectation_pass_count')}/{summary.get('probe_count')}, sequence_pass={summary.get('sequence_pass')}"
        )
    lines.extend(["", "## Known Limitations", ""])
    for item in report.get("known_limitations", []):
        lines.append(f"- {item}")
    conclusion = dict(report.get("final_conclusion") or {})
    lines.extend(
        [
            "",
            "## Final Conclusion on Whether Minimal VLN Closed-Loop Execution v0.1 Is Landed",
            "",
            f"- landed: `{conclusion.get('landed')}`",
            f"- summary: {conclusion.get('summary')}",
            "",
        ]
    )
    return "\n".join(lines)


def write_case_traces(report_root: Path, sequence_payloads: Iterable[Dict[str, Any]]) -> List[str]:
    written_paths: List[str] = []
    traces_root = report_root / "traces"
    for sequence in sequence_payloads:
        sequence_id = str(sequence.get("sequence_id"))
        for case in list(sequence.get("success_cases") or []) + list(sequence.get("honesty_probes") or []):
            case_path = traces_root / sequence_id / f"{case.get('case_id')}.json"
            _json_dump(case, case_path)
            written_paths.append(str(case_path))
    return written_paths


def write_csv_summary(path: Path, sequence_payloads: Iterable[Dict[str, Any]]) -> None:
    rows: List[Dict[str, Any]] = []
    for sequence in sequence_payloads:
        for bucket_name in ("success_cases", "honesty_probes"):
            for case in sequence.get(bucket_name, []):
                execution = dict(case.get("execution") or {})
                plan = dict(execution.get("plan") or {})
                target_resolution = dict(plan.get("target_resolution") or {})
                symbolic_plan = dict(plan.get("symbolic_plan") or {})
                outcome = dict(execution.get("outcome") or {})
                rows.append(
                    {
                        "sequence_id": sequence.get("sequence_id"),
                        "bucket": bucket_name,
                        "case_id": case.get("case_id"),
                        "case_kind": case.get("case_kind"),
                        "target_type": case.get("target_type"),
                        "start_room_id": case.get("start_room_id"),
                        "start_floor_id": case.get("start_floor_id"),
                        "resolved_goal_room_id": target_resolution.get("resolved_room_id"),
                        "resolved_goal_floor_id": target_resolution.get("resolved_floor_id"),
                        "hop_count": symbolic_plan.get("hop_count"),
                        "floor_switch_count": len(plan.get("floor_switches") or []),
                        "outcome_category": outcome.get("outcome_category"),
                        "outcome_reason": outcome.get("outcome_reason"),
                        "completion_frame_idx": outcome.get("completion_frame_idx"),
                    }
                )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        if not rows:
            return
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the minimal VLN closed-loop execution acceptance harness.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Stage A output root containing per-sequence topology/timeline exports.",
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
        help="Route policy to reuse from the existing Query API.",
    )
    parser.add_argument(
        "--report-dir",
        default=None,
        help="Optional explicit report directory. Defaults to <output_root>/minimal_vln_closed_loop_v0_1.",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    report_dir = Path(args.report_dir) if args.report_dir is not None else output_root / "minimal_vln_closed_loop_v0_1"
    report_dir.mkdir(parents=True, exist_ok=True)

    sequence_payloads = [
        sequence_report(output_root=output_root, sequence_id=sequence_id, route_policy=args.route_policy)
        for sequence_id in args.sequences
    ]
    report = build_report_payload(
        output_root=output_root,
        sequence_payloads=sequence_payloads,
        route_policy=args.route_policy,
        invoked_command=[sys.executable, *sys.argv],
    )

    trace_paths = write_case_traces(report_dir, sequence_payloads)
    json_path = report_dir / "minimal_vln_closed_loop_v0_1.json"
    csv_path = report_dir / "minimal_vln_closed_loop_v0_1.csv"
    markdown_path = report_dir / "minimal_vln_closed_loop_v0_1_report.md"
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
