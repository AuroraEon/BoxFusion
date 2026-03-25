from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from boxfusion.floor_artifacts import display_floor_label
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import _canonical_room_id
from boxfusion.vln_closed_loop import VLNClosedLoopExecutor
from boxfusion.vln_tool_use import MinimalVLNToolUseAdapter, render_teacher_brief


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _target_type_from_task_input(task_input: Dict[str, Any]) -> Optional[str]:
    target = dict(task_input.get("target") or {})
    target_type = str(target.get("target_type") or "").strip().lower()
    return target_type or None


def _request_type_from_tool_use(parsed_request: Dict[str, Any]) -> Optional[str]:
    request_type = str(parsed_request.get("request_type") or "").strip().lower()
    if request_type == "route_explanation":
        return "explain"
    if request_type == "execute":
        return "execute"
    return None


def _backend_owner(tool_name: Optional[str]) -> Optional[str]:
    if tool_name == "query_route":
        return "boxfusion.query_api.RoomTopologyQueryAPI.query_route"
    if tool_name == "query_route_to_anchor":
        return "boxfusion.query_api.RoomTopologyQueryAPI.query_route_to_anchor"
    if tool_name == "query_route_to_object":
        return "boxfusion.query_api.RoomTopologyQueryAPI.query_route_to_object"
    if tool_name == "closed_loop_execute":
        return "boxfusion.vln_closed_loop.VLNClosedLoopExecutor.execute"
    return None


def _status_from_resolution_failure(failure_reason: Optional[str]) -> str:
    reason = str(failure_reason or "").strip().lower()
    if "ambiguous" in reason:
        return "ambiguous"
    if "unsupported" in reason:
        return "unsupported"
    return "unresolved"


def _route_payload_from_backend(backend_result: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    payload = dict(backend_result or {})
    if "route" in payload:
        return dict(payload.get("route") or {})
    return dict((((payload.get("plan") or {}).get("query_result") or {}).get("route")) or {})


def _explanation_payload_from_backend(backend_result: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    payload = dict(backend_result or {})
    if "explanation" in payload:
        return dict(payload.get("explanation") or {})
    return dict((((payload.get("plan") or {}).get("query_result") or {}).get("explanation")) or {})


def _target_resolution_from_backend(backend_result: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    payload = dict(backend_result or {})
    if "target_resolution" in payload:
        return dict(payload.get("target_resolution") or {})
    return dict((((payload.get("plan") or {}).get("target_resolution")) or {}) or {})


def _selected_label(query_api: RoomTopologyQueryAPI, task_input: Dict[str, Any], resolved_room_id: Optional[str]) -> Optional[str]:
    target = dict(task_input.get("target") or {})
    target_type = _target_type_from_task_input(task_input)
    if target_type == "room":
        room_record = query_api.topology.get_room(resolved_room_id or target.get("goal_room_id")) or {}
        return room_record.get("room_type") or resolved_room_id or target.get("goal_room_id")
    if target_type == "anchor":
        anchor_id = target.get("anchor_id")
        anchor_record = query_api.topology.get_anchor(anchor_id) or {}
        return anchor_id or anchor_record.get("id")
    if target_type == "object":
        object_id = target.get("object_id")
        object_record = query_api.topology.get_object(object_id) if object_id else None
        if object_record:
            return object_record.get("label") or object_record.get("normalized_label") or object_id
        return target.get("object_label") or object_id
    return None


def _selected_entity_id(task_input: Dict[str, Any], target_resolution: Dict[str, Any]) -> Optional[str]:
    target = dict(task_input.get("target") or {})
    target_type = _target_type_from_task_input(task_input)
    if target_type == "room":
        return target.get("goal_room_id") or target_resolution.get("resolved_room_id")
    if target_type == "anchor":
        return target.get("anchor_id")
    if target_type == "object":
        matched = list(target_resolution.get("matched_entity_ids", []))
        return target.get("object_id") or (matched[0] if matched else None)
    return None


def _summarize_resolved_target(
    query_api: RoomTopologyQueryAPI,
    task_input: Dict[str, Any],
    target_resolution: Dict[str, Any],
) -> Dict[str, Any]:
    target_type = _target_type_from_task_input(task_input)
    resolved_room_id = target_resolution.get("resolved_room_id")
    floor_id = target_resolution.get("resolved_floor_id")
    display_floor_id = target_resolution.get("resolved_display_floor_id")
    return {
        "resolved": bool(target_resolution.get("resolved")),
        "target_type": target_type,
        "selected_entity_id": _selected_entity_id(task_input, target_resolution),
        "selected_label": _selected_label(query_api, task_input, resolved_room_id),
        "resolved_room_id": resolved_room_id,
        "resolved_floor_id": floor_id,
        "resolved_display_floor_id": display_floor_id,
        "resolved_floor_label": display_floor_label(floor_id, display_floor_id),
        "matched_entity_ids": list(target_resolution.get("matched_entity_ids", [])),
        "candidate_matches": list(target_resolution.get("candidate_matches", [])),
        "failure_reason": target_resolution.get("failure_reason"),
        "notes": list(target_resolution.get("notes", [])),
    }


def _floor_switch_summary(explanation: Dict[str, Any]) -> Dict[str, Any]:
    switches = list(explanation.get("floor_switches", []))
    return {
        "switch_required": bool(switches),
        "switch_count": len(switches),
        "switches": switches,
        "summary": (
            "No floor switch required."
            if not switches
            else "; ".join(
                (
                    f"{item.get('source_room_id')} -> {item.get('target_room_id')} "
                    f"({display_floor_label(item.get('from_floor_id'), item.get('from_display_floor_id'))} "
                    f"-> {display_floor_label(item.get('to_floor_id'), item.get('to_display_floor_id'))} "
                    f"via {item.get('relation_type')})"
                )
                for item in switches
            )
        ),
    }


def _vertical_transition_summary(route: Dict[str, Any], explanation: Dict[str, Any]) -> Dict[str, Any]:
    used_relation_types = list(route.get("used_relation_types", []))
    transition_edges = [
        dict(edge)
        for edge in route.get("edges", [])
        if edge.get("relation_type") == "vertical_transition"
    ]
    floor_switches = list(explanation.get("floor_switches", []))
    return {
        "used": "vertical_transition" in used_relation_types,
        "count": len(transition_edges),
        "transition_edges": transition_edges,
        "transition_ids": [
            transition_id
            for edge in transition_edges
            for transition_id in list((edge.get("metadata") or {}).get("transition_ids", []))
        ],
        "floor_switch_count": len(floor_switches),
    }


def _route_summary(route: Dict[str, Any], explanation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "attempted": bool(route.get("attempted")),
        "found": bool(route.get("found")),
        "hop_count": route.get("hop_count"),
        "room_sequence": list(route.get("room_sequence", [])),
        "used_relation_types": list(route.get("used_relation_types", [])),
        "route_confidence": route.get("route_confidence"),
        "total_cost": route.get("total_cost"),
        "summary": explanation.get("route_summary"),
        "hop_summaries": list(explanation.get("hop_summaries", [])),
        "failure_reason": route.get("failure_reason"),
    }


def _execution_trace_summary(backend_result: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if backend_result is None or "outcome" not in dict(backend_result):
        return None
    payload = dict(backend_result or {})
    outcome = dict(payload.get("outcome") or {})
    trace = list(payload.get("execution_trace", []))
    return {
        "event_count": len(trace),
        "events": trace,
        "observed_room_sequence": list(payload.get("observed_room_sequence", [])),
        "observed_floor_switches": list(payload.get("observed_floor_switches", [])),
        "completed_step_count": outcome.get("completed_step_count"),
        "total_step_count": outcome.get("total_step_count"),
        "completion_frame_idx": outcome.get("completion_frame_idx"),
    }


def _limitations_for(target_type: Optional[str], request_type: Optional[str], request_source: str) -> List[str]:
    limitations = [
        "The orchestration layer only composes existing modules; it does not add a new planner or graph-search implementation.",
        "World-graph truth, topology routing, and closed-loop execution remain in their existing backends.",
    ]
    if request_source == "nl":
        limitations.append(
            "Natural-language handling remains intentionally narrow and only performs constrained interpretation and tool selection."
        )
    if target_type == "room":
        limitations.append("Room requests stay limited to explicit room ids or unique exported room_type matches.")
    elif target_type == "anchor":
        limitations.append("Anchor requests stay limited to explicit anchor ids or anchors backed by inspectable metadata.")
    elif target_type == "object":
        limitations.append("Object requests remain room-level targets and do not claim precise docking or manipulation.")
    if request_type == "execute":
        limitations.append("Execution reuses the replay-driven minimal executor and does not add full embodied control.")
    return limitations


class VLNEndToEndDemoOrchestrator:
    """Thin end-to-end composition layer over the existing NL adapter, Query API, and executor."""

    def __init__(
        self,
        query_api: RoomTopologyQueryAPI,
        *,
        executor: Optional[VLNClosedLoopExecutor] = None,
        tool_use_adapter: Optional[MinimalVLNToolUseAdapter] = None,
    ) -> None:
        self.query_api = query_api
        self.executor = executor
        self.tool_use_adapter = tool_use_adapter or MinimalVLNToolUseAdapter(query_api=query_api, executor=executor)

    @classmethod
    def from_paths(
        cls,
        topology_json: Path,
        timeline_json: Optional[Path] = None,
    ) -> "VLNEndToEndDemoOrchestrator":
        tool_use_adapter = MinimalVLNToolUseAdapter.from_paths(
            topology_json=Path(topology_json),
            timeline_json=None if timeline_json is None else Path(timeline_json),
        )
        return cls(
            query_api=tool_use_adapter.query_api,
            executor=tool_use_adapter.executor,
            tool_use_adapter=tool_use_adapter,
        )

    def run_nl_request(
        self,
        raw_request: str,
        *,
        sequence_id: Optional[str] = None,
        start_room_id: Optional[str] = None,
        route_policy: str = "balanced",
        start_frame_idx: Optional[int] = None,
        end_frame_idx: Optional[int] = None,
        max_offroute_room_changes: int = 0,
        case_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        result = self.tool_use_adapter.handle_request(
            raw_request,
            start_room_id=start_room_id,
            route_policy=route_policy,
            start_frame_idx=start_frame_idx,
            end_frame_idx=end_frame_idx,
            max_offroute_room_changes=max_offroute_room_changes,
        )
        parsed_request = dict(result.get("parsed_request") or {})
        request_type = _request_type_from_tool_use(parsed_request)
        backend_result = dict(result.get("backend_result") or {}) if result.get("backend_result") is not None else None
        tool_selection = dict(result.get("tool_selection") or {}) if result.get("tool_selection") is not None else None
        teacher_response = dict(result.get("teacher_response") or {})
        route = _route_payload_from_backend(backend_result)
        explanation = _explanation_payload_from_backend(backend_result)
        floor_switch_summary = _floor_switch_summary(explanation) if explanation else {"switch_required": None, "switch_count": 0, "switches": [], "summary": None}
        final_outcome = self._final_outcome_from_nl_result(result, request_type=request_type)
        return {
            "version": "0.1",
            "generated_at_utc": _utc_now_iso(),
            "sequence_id": sequence_id or self.query_api.topology.sequence_id,
            "case_id": case_id,
            "request_source": "nl",
            "raw_request": raw_request,
            "interpreted_request": parsed_request,
            "request_type": request_type,
            "target_type": parsed_request.get("target_type"),
            "floor_hint": parsed_request.get("floor_hint"),
            "selected_backend_call": tool_selection,
            "resolved_target": dict(result.get("resolved_target") or {}) if result.get("resolved_target") is not None else None,
            "route_summary": _route_summary(route, explanation) if route else None,
            "floor_switch_summary": floor_switch_summary,
            "vertical_transition_summary": _vertical_transition_summary(route, explanation) if route else None,
            "execution_trace": _execution_trace_summary(backend_result),
            "final_outcome": final_outcome,
            "limitations_note": list(teacher_response.get("limitations", [])) or _limitations_for(
                parsed_request.get("target_type"),
                request_type,
                "nl",
            ),
            "teacher_facing_record": teacher_response,
            "teacher_brief": render_teacher_brief(result),
            "artifacts": {
                "route_policy": route_policy,
                "start_room_id": _canonical_room_id(start_room_id),
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "max_offroute_room_changes": int(max_offroute_room_changes),
                "tool_use_result": result,
            },
        }

    def run_structured_task(
        self,
        structured_request: Dict[str, Any],
        *,
        sequence_id: Optional[str] = None,
        route_policy: str = "balanced",
        case_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        request_type = str(structured_request.get("request_type") or "").strip().lower()
        task_input = {
            "start_room_id": structured_request.get("start_room_id"),
            "target": dict(structured_request.get("target") or {}),
        }
        start_frame_idx = structured_request.get("start_frame_idx")
        end_frame_idx = structured_request.get("end_frame_idx")
        max_offroute_room_changes = int(structured_request.get("max_offroute_room_changes") or 0)
        target_type = _target_type_from_task_input(task_input)
        if request_type not in {"explain", "execute"}:
            raise ValueError("Structured requests must use request_type 'explain' or 'execute'.")
        if target_type not in {"room", "anchor", "object"}:
            raise ValueError("Structured requests must use target_type 'room', 'anchor', or 'object'.")

        if request_type == "explain":
            tool_name, backend_result = self._dispatch_query_task(task_input=task_input, route_policy=route_policy)
            status = self._query_status_from_backend(backend_result)
        else:
            if self.executor is None:
                raise RuntimeError("Structured execute requests require an attached VLNClosedLoopExecutor.")
            tool_name = "closed_loop_execute"
            backend_result = self.executor.execute(
                task_input=task_input,
                route_policy=route_policy,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
                max_offroute_room_changes=max_offroute_room_changes,
            )
            outcome = dict(backend_result.get("outcome") or {})
            status = "success" if outcome.get("success") else str(outcome.get("outcome_category") or "failure")

        route = _route_payload_from_backend(backend_result)
        explanation = _explanation_payload_from_backend(backend_result)
        target_resolution = _target_resolution_from_backend(backend_result)
        resolved_target = _summarize_resolved_target(self.query_api, task_input, target_resolution)
        teacher_record = self._teacher_record_from_structured_result(
            request_type=request_type,
            task_input=task_input,
            route=route,
            explanation=explanation,
            resolved_target=resolved_target,
            backend_result=backend_result,
            status=status,
        )
        return {
            "version": "0.1",
            "generated_at_utc": _utc_now_iso(),
            "sequence_id": sequence_id or self.query_api.topology.sequence_id,
            "case_id": case_id,
            "request_source": "structured",
            "raw_request": structured_request.get("raw_request"),
            "interpreted_request": {
                "request_type": request_type,
                "target_type": target_type,
                "start_room_id": task_input.get("start_room_id"),
                "target": dict(task_input.get("target") or {}),
                "route_policy": route_policy,
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "max_offroute_room_changes": max_offroute_room_changes,
            },
            "request_type": request_type,
            "target_type": target_type,
            "floor_hint": None,
            "selected_backend_call": {
                "mode": "execute" if request_type == "execute" else "query_only",
                "tool_name": tool_name,
                "backend_owner": _backend_owner(tool_name),
                "task_input": task_input,
                "route_policy": route_policy,
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "max_offroute_room_changes": max_offroute_room_changes,
            },
            "resolved_target": resolved_target,
            "route_summary": _route_summary(route, explanation),
            "floor_switch_summary": _floor_switch_summary(explanation),
            "vertical_transition_summary": _vertical_transition_summary(route, explanation),
            "execution_trace": _execution_trace_summary(backend_result),
            "final_outcome": self._final_outcome_from_structured_result(
                request_type=request_type,
                backend_result=backend_result,
                status=status,
            ),
            "limitations_note": _limitations_for(target_type, request_type, "structured"),
            "teacher_facing_record": teacher_record,
            "teacher_brief": teacher_record.get("summary"),
            "artifacts": {
                "route_policy": route_policy,
                "backend_result": backend_result,
            },
        }

    def _dispatch_query_task(
        self,
        *,
        task_input: Dict[str, Any],
        route_policy: str,
    ) -> Tuple[str, Dict[str, Any]]:
        target = dict(task_input.get("target") or {})
        target_type = _target_type_from_task_input(task_input)
        if target_type == "room":
            return "query_route", self.query_api.query_route(
                start_room_id=task_input.get("start_room_id"),
                goal_room_id=target.get("goal_room_id"),
                route_policy=route_policy,
            )
        if target_type == "anchor":
            return "query_route_to_anchor", self.query_api.query_route_to_anchor(
                start_room_id=task_input.get("start_room_id"),
                anchor_id=target.get("anchor_id"),
                route_policy=route_policy,
            )
        return "query_route_to_object", self.query_api.query_route_to_object(
            start_room_id=task_input.get("start_room_id"),
            object_id=target.get("object_id"),
            object_label=target.get("object_label"),
            route_policy=route_policy,
        )

    def _query_status_from_backend(self, backend_result: Dict[str, Any]) -> str:
        target_resolution = dict(backend_result.get("target_resolution") or {})
        if not target_resolution.get("resolved"):
            return _status_from_resolution_failure(target_resolution.get("failure_reason"))
        route = dict(backend_result.get("route") or {})
        if route.get("found"):
            return "success"
        return str(route.get("failure_reason") or "route_not_found")

    def _teacher_record_from_structured_result(
        self,
        *,
        request_type: str,
        task_input: Dict[str, Any],
        route: Dict[str, Any],
        explanation: Dict[str, Any],
        resolved_target: Dict[str, Any],
        backend_result: Dict[str, Any],
        status: str,
    ) -> Dict[str, Any]:
        if request_type == "execute":
            outcome = dict(backend_result.get("outcome") or {})
            return {
                "summary": (
                    f"Structured execute request delegated to the existing minimal closed-loop executor. "
                    f"Outcome={outcome.get('outcome_category')} / {outcome.get('outcome_reason')}."
                ),
                "interpreted_intent": f"execute:{_target_type_from_task_input(task_input)}",
                "resolved_target": resolved_target,
                "route_summary": ((backend_result.get("plan") or {}).get("route_summary")),
                "floor_switch_required": bool(((backend_result.get("plan") or {}).get("floor_switches"))),
                "execution_result": outcome,
                "limitations": _limitations_for(_target_type_from_task_input(task_input), request_type, "structured"),
                "notes": list(resolved_target.get("notes", [])),
            }
        summary = explanation.get("summary") or f"Structured explain request completed with status={status}."
        return {
            "summary": f"Structured explain request delegated to the existing Query API. {summary}",
            "interpreted_intent": f"explain:{_target_type_from_task_input(task_input)}",
            "resolved_target": resolved_target,
            "route_summary": explanation.get("route_summary"),
            "floor_switch_required": bool(explanation.get("floor_switches")),
            "execution_result": None,
            "limitations": _limitations_for(_target_type_from_task_input(task_input), request_type, "structured"),
            "notes": list(explanation.get("notes", [])) + list(resolved_target.get("notes", [])),
        }

    def _final_outcome_from_nl_result(
        self,
        result: Dict[str, Any],
        *,
        request_type: Optional[str],
    ) -> Dict[str, Any]:
        status = str(result.get("status") or "unknown")
        backend_result = dict(result.get("backend_result") or {}) if result.get("backend_result") is not None else {}
        teacher_response = dict(result.get("teacher_response") or {})
        if request_type == "execute":
            outcome = dict(backend_result.get("outcome") or {})
            return {
                "status": status,
                "success": bool(outcome.get("success")),
                "outcome_category": outcome.get("outcome_category"),
                "outcome_reason": outcome.get("outcome_reason"),
                "notes": list(teacher_response.get("notes", [])),
            }
        route = dict(backend_result.get("route") or {})
        return {
            "status": status,
            "success": status == "success",
            "outcome_category": "success" if status == "success" else status,
            "outcome_reason": route.get("failure_reason") or status,
            "notes": list(teacher_response.get("notes", [])),
        }

    def _final_outcome_from_structured_result(
        self,
        *,
        request_type: str,
        backend_result: Dict[str, Any],
        status: str,
    ) -> Dict[str, Any]:
        if request_type == "execute":
            outcome = dict(backend_result.get("outcome") or {})
            return {
                "status": status,
                "success": bool(outcome.get("success")),
                "outcome_category": outcome.get("outcome_category"),
                "outcome_reason": outcome.get("outcome_reason"),
                "notes": [],
            }
        route = dict(backend_result.get("route") or {})
        return {
            "status": status,
            "success": status == "success",
            "outcome_category": "success" if status == "success" else status,
            "outcome_reason": route.get("failure_reason") or status,
            "notes": [],
        }
