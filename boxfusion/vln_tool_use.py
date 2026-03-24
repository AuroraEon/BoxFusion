from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import _canonical_anchor_id, _canonical_room_id
from boxfusion.scene_graph_builder import normalize_open_vocab_label
from boxfusion.template_grounding.normalizer import normalize_reference_slot, preprocess_instruction
from boxfusion.vln_closed_loop import VLNClosedLoopExecutor


_EXECUTE_PATTERNS: Sequence[re.Pattern[str]] = (
    re.compile(r"^(go to|navigate to|take me to|start navigation to|start navigating to)\s+(?P<target>.+)$"),
)
_EXPLAIN_PATTERNS: Sequence[re.Pattern[str]] = (
    re.compile(r"^(how do i get to)\s+(?P<target>.+)$"),
    re.compile(r"^(what route would i take to)\s+(?P<target>.+)$"),
    re.compile(r"^(what route do i take to)\s+(?P<target>.+)$"),
    re.compile(r"^(will i need to switch floors to(?: reach|get to)?)\s+(?P<target>.+)$"),
)
_ABSOLUTE_FLOOR_PATTERNS: Sequence[Tuple[re.Pattern[str], int]] = (
    (re.compile(r"\bon the first floor\b|\bfirst floor\b"), 1),
    (re.compile(r"\bon the second floor\b|\bsecond floor\b"), 2),
    (re.compile(r"\bon the third floor\b|\bthird floor\b"), 3),
    (re.compile(r"\bon the fourth floor\b|\bfourth floor\b"), 4),
    (re.compile(r"\bon floor[_ ]?(\d+)\b|\bfloor[_ ]?(\d+)\b"), -1),
)
_UPSTAIRS_RE = re.compile(r"\bupstairs\b")
_DOWNSTAIRS_RE = re.compile(r"\bdownstairs\b")
_ARTICLES_RE = re.compile(r"^(the|a|an)\s+")
_ANCHOR_SUFFIX_RE = re.compile(r"\s+anchor$")
_ROOM_ID_RE = re.compile(r"^room[_ ]?(\d+)$")


def _score_value(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _room_sort_key(room_record: Dict[str, Any], room_id: str) -> Tuple[int, str]:
    display_order = room_record.get("display_order")
    try:
        display_order_value = int(display_order)
    except (TypeError, ValueError):
        display_order_value = 10**6
    return (display_order_value, str(room_id))


def _serialize_object_candidate(record: Dict[str, Any], *, rank: int, match_type: str) -> Dict[str, Any]:
    return {
        "rank": int(rank),
        "entity_id": record.get("id"),
        "label": record.get("label"),
        "normalized_label": record.get("normalized_label"),
        "category": record.get("category"),
        "room_id": record.get("room_id"),
        "floor_id": record.get("floor_id"),
        "display_floor_id": record.get("display_floor_id"),
        "display_order": record.get("display_order"),
        "score": record.get("score"),
        "detection_confidence": record.get("detection_confidence"),
        "semantic_confidence": record.get("semantic_confidence"),
        "match_type": match_type,
    }


def _serialize_anchor_candidate(
    record: Dict[str, Any],
    *,
    rank: int,
    match_type: str,
    target_label: Optional[str],
) -> Dict[str, Any]:
    return {
        "rank": int(rank),
        "entity_id": record.get("id"),
        "anchor_type": record.get("anchor_type"),
        "room_id": record.get("room_id"),
        "floor_id": record.get("floor_id"),
        "display_floor_id": record.get("display_floor_id"),
        "display_order": record.get("display_order"),
        "target_id": record.get("target_id"),
        "valid": record.get("valid"),
        "score": record.get("score"),
        "match_type": match_type,
        "target_label": target_label,
    }


class MinimalVLNToolUseAdapter:
    """Thin, demo-honest NL tool-use layer over the existing Query API + closed-loop executor."""

    def __init__(
        self,
        query_api: RoomTopologyQueryAPI,
        executor: Optional[VLNClosedLoopExecutor] = None,
    ) -> None:
        self.query_api = query_api
        self.executor = executor

    @classmethod
    def from_paths(
        cls,
        topology_json: Path,
        timeline_json: Optional[Path] = None,
    ) -> "MinimalVLNToolUseAdapter":
        query_api = RoomTopologyQueryAPI.from_json(Path(topology_json))
        executor = None
        if timeline_json is not None:
            executor = VLNClosedLoopExecutor.from_paths(Path(topology_json), Path(timeline_json))
        return cls(query_api=query_api, executor=executor)

    def parse_request(
        self,
        instruction: str,
        *,
        start_room_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        text = preprocess_instruction(instruction)
        parsed: Dict[str, Any] = {
            "version": "0.1",
            "instruction": instruction,
            "normalized_instruction": text,
            "supported": False,
            "request_type": None,
            "mode": None,
            "target_type": None,
            "target_text": None,
            "raw_target_text": None,
            "floor_hint": None,
            "ambiguity_state": "unresolved",
            "start_room_id": _canonical_room_id(start_room_id),
            "parse_notes": [],
            "unsupported_reason": None,
        }

        if not text:
            parsed["unsupported_reason"] = "empty_request"
            parsed["parse_notes"] = ["Instruction is empty after normalization."]
            return parsed

        matched_mode = None
        matched_target = None
        for pattern in _EXPLAIN_PATTERNS:
            match = pattern.match(text)
            if match:
                matched_mode = ("route_explanation", "query_only")
                matched_target = match.group("target")
                break
        if matched_mode is None:
            for pattern in _EXECUTE_PATTERNS:
                match = pattern.match(text)
                if match:
                    matched_mode = ("execute", "execute")
                    matched_target = match.group("target")
                    break
        if matched_mode is None or matched_target is None:
            parsed["unsupported_reason"] = "unsupported_request"
            parsed["parse_notes"] = [
                "Supported families are room/object/anchor route explanation requests and execute requests.",
            ]
            return parsed

        floor_hint, target_text = self._extract_floor_hint(matched_target)
        target_text = _ARTICLES_RE.sub("", target_text).strip()
        if not target_text:
            parsed["unsupported_reason"] = "missing_target"
            parsed["parse_notes"] = ["Request matched a supported family, but no target text remained after normalization."]
            return parsed

        parsed["request_type"], parsed["mode"] = matched_mode
        parsed["raw_target_text"] = matched_target
        parsed["target_text"] = target_text
        parsed["floor_hint"] = floor_hint
        parsed["target_type"] = self._infer_target_type(target_text)
        parsed["supported"] = True
        parsed["ambiguity_state"] = "none"
        return parsed

    def handle_request(
        self,
        instruction: str,
        *,
        start_room_id: Optional[str] = None,
        route_policy: str = "balanced",
        start_frame_idx: Optional[int] = None,
        end_frame_idx: Optional[int] = None,
        max_offroute_room_changes: int = 0,
    ) -> Dict[str, Any]:
        parsed = self.parse_request(instruction, start_room_id=start_room_id)
        base_result = {
            "version": "0.1",
            "instruction": instruction,
            "route_policy": route_policy,
            "parsed_request": parsed,
            "resolved_target": None,
            "tool_selection": None,
            "backend_result": None,
            "status": None,
            "teacher_response": None,
        }

        if not parsed.get("supported"):
            base_result["status"] = "unsupported"
            base_result["teacher_response"] = self._unsupported_teacher_response(parsed)
            return base_result

        start_room_canonical = _canonical_room_id(start_room_id)
        if start_room_canonical is None:
            base_result["status"] = "unresolved"
            base_result["teacher_response"] = {
                "summary": "Start room context is required for VLN Tool-Use v0.1 requests.",
                "interpreted_intent": parsed.get("request_type"),
                "resolved_target": None,
                "route_summary": None,
                "floor_switch_required": None,
                "execution_result": None,
                "limitations": [
                    "v0.1 requires an explicit start room context and does not infer robot pose from dialogue.",
                ],
                "notes": ["No valid start_room_id was provided."],
            }
            return base_result

        resolution = self._resolve_request(parsed, start_room_id=start_room_canonical)
        base_result["resolved_target"] = resolution
        if not resolution.get("resolved"):
            status = str(resolution.get("ambiguity_state") or "unresolved")
            base_result["status"] = status
            base_result["teacher_response"] = self._resolution_failure_teacher_response(parsed, resolution)
            return base_result

        task_input = {
            "start_room_id": start_room_canonical,
            "target": dict(resolution.get("task_target") or {}),
        }
        if parsed.get("mode") == "execute":
            if self.executor is None:
                base_result["status"] = "unresolved"
                base_result["teacher_response"] = {
                    "summary": "Execution was requested, but no closed-loop executor is attached.",
                    "interpreted_intent": self._intent_summary(parsed),
                    "resolved_target": self._resolved_target_summary(resolution),
                    "route_summary": None,
                    "floor_switch_required": None,
                    "execution_result": {
                        "status": "not_started",
                        "outcome_category": None,
                        "outcome_reason": "executor_unavailable",
                    },
                    "limitations": [
                        "Execute mode must delegate to the existing minimal closed-loop executor.",
                    ],
                    "notes": ["Attach a timeline-backed VLNClosedLoopExecutor to run execute requests."],
                }
                return base_result
            base_result["tool_selection"] = {
                "mode": "execute",
                "tool_name": "closed_loop_execute",
                "backend_owner": "boxfusion.vln_closed_loop.VLNClosedLoopExecutor.execute",
                "task_input": task_input,
                "route_policy": route_policy,
                "start_frame_idx": start_frame_idx,
                "end_frame_idx": end_frame_idx,
                "max_offroute_room_changes": int(max_offroute_room_changes),
            }
            backend_result = self.executor.execute(
                task_input=task_input,
                route_policy=route_policy,
                start_frame_idx=start_frame_idx,
                end_frame_idx=end_frame_idx,
                max_offroute_room_changes=max_offroute_room_changes,
            )
            base_result["backend_result"] = backend_result
            outcome = dict(backend_result.get("outcome") or {})
            base_result["status"] = (
                "success" if outcome.get("success") else str(outcome.get("outcome_category") or "execution_failed")
            )
            base_result["teacher_response"] = self._execution_teacher_response(parsed, resolution, backend_result)
            return base_result

        tool_name, backend_result = self._dispatch_query(task_input=task_input, route_policy=route_policy)
        base_result["tool_selection"] = {
            "mode": "query_only",
            "tool_name": tool_name,
            "backend_owner": f"boxfusion.query_api.RoomTopologyQueryAPI.{tool_name}",
            "task_input": task_input,
            "route_policy": route_policy,
        }
        base_result["backend_result"] = backend_result
        base_result["status"] = "success" if backend_result.get("found") else "route_not_found"
        base_result["teacher_response"] = self._query_teacher_response(parsed, resolution, backend_result)
        return base_result

    def _extract_floor_hint(self, target_text: str) -> Tuple[Optional[Dict[str, Any]], str]:
        text = str(target_text).strip()
        floor_hint: Optional[Dict[str, Any]] = None
        for pattern, fixed_index in _ABSOLUTE_FLOOR_PATTERNS:
            match = pattern.search(text)
            if not match:
                continue
            floor_index = fixed_index
            if floor_index < 0:
                for group in match.groups():
                    if group is not None:
                        floor_index = int(group)
                        break
            floor_hint = {
                "raw_text": match.group(0),
                "kind": "absolute",
                "direction": None,
                "floor_index": floor_index,
                "floor_id": f"floor_{int(floor_index)}",
                "display_floor_id": f"floor_{int(floor_index)}",
            }
            text = (text[: match.start()] + " " + text[match.end() :]).strip()
            break
        if floor_hint is None and _UPSTAIRS_RE.search(text):
            match = _UPSTAIRS_RE.search(text)
            floor_hint = {
                "raw_text": match.group(0) if match else "upstairs",
                "kind": "relative",
                "direction": "up",
                "floor_index": None,
                "floor_id": None,
                "display_floor_id": None,
            }
            text = _UPSTAIRS_RE.sub(" ", text).strip()
        if floor_hint is None and _DOWNSTAIRS_RE.search(text):
            match = _DOWNSTAIRS_RE.search(text)
            floor_hint = {
                "raw_text": match.group(0) if match else "downstairs",
                "kind": "relative",
                "direction": "down",
                "floor_index": None,
                "floor_id": None,
                "display_floor_id": None,
            }
            text = _DOWNSTAIRS_RE.sub(" ", text).strip()
        return floor_hint, " ".join(text.split())

    def _infer_target_type(self, target_text: str) -> str:
        normalized = preprocess_instruction(target_text)
        if normalized.startswith("anchor ") or normalized.endswith(" anchor") or normalized.startswith("anchor_"):
            return "anchor"
        if self._explicit_room_id(target_text) is not None:
            return "room"
        if self._room_type_candidates(target_text):
            return "room"
        return "object"

    def _resolve_request(
        self,
        parsed_request: Dict[str, Any],
        *,
        start_room_id: str,
    ) -> Dict[str, Any]:
        target_type = str(parsed_request.get("target_type") or "")
        floor_hint_context = self._resolve_floor_hint(parsed_request.get("floor_hint"), start_room_id=start_room_id)
        if floor_hint_context.get("ok") is False:
            return {
                "resolved": False,
                "ambiguity_state": "unresolved",
                "target_type": target_type,
                "task_target": None,
                "resolution_source": None,
                "matched_entity_ids": [],
                "candidate_matches": [],
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": None,
                "notes": list(floor_hint_context.get("notes", [])),
                "floor_hint": floor_hint_context,
                "failure_reason": "floor_hint_unresolved",
            }
        if target_type == "room":
            return self._resolve_room_request(parsed_request, floor_hint_context=floor_hint_context)
        if target_type == "anchor":
            return self._resolve_anchor_request(parsed_request, floor_hint_context=floor_hint_context)
        return self._resolve_object_request(parsed_request, floor_hint_context=floor_hint_context)

    def _resolve_floor_hint(
        self,
        floor_hint: Optional[Dict[str, Any]],
        *,
        start_room_id: str,
    ) -> Dict[str, Any]:
        if floor_hint is None:
            return {"ok": True, "eligible_floor_ids": None, "notes": [], "hint": None}

        topology = self.query_api.topology
        floor_records = list(topology.floor_records.values())
        by_floor_id = {str(item.get("floor_id")): dict(item) for item in floor_records if item.get("floor_id") is not None}
        by_display_floor_id = {
            str(item.get("display_floor_id")): dict(item)
            for item in floor_records
            if item.get("display_floor_id") is not None
        }

        if floor_hint.get("kind") == "absolute":
            candidate = (
                by_display_floor_id.get(str(floor_hint.get("display_floor_id")))
                or by_floor_id.get(str(floor_hint.get("floor_id")))
            )
            if candidate is None:
                return {
                    "ok": False,
                    "eligible_floor_ids": [],
                    "notes": [f"Floor hint {floor_hint.get('raw_text')!r} does not match any exported floor."],
                    "hint": floor_hint,
                }
            return {
                "ok": True,
                "eligible_floor_ids": [candidate.get("floor_id")],
                "notes": [f"Floor hint {floor_hint.get('raw_text')!r} mapped to {candidate.get('display_floor_id') or candidate.get('floor_id')}."],
                "hint": dict(floor_hint),
            }

        start_room = topology.get_room(start_room_id) or {}
        start_floor_id = start_room.get("floor_id")
        if start_floor_id is None:
            return {
                "ok": False,
                "eligible_floor_ids": [],
                "notes": ["Relative floor hints require a start room with floor metadata."],
                "hint": floor_hint,
            }
        start_floor_record = by_floor_id.get(str(start_floor_id), {})
        try:
            start_order = int(start_floor_record.get("display_order"))
        except (TypeError, ValueError):
            return {
                "ok": False,
                "eligible_floor_ids": [],
                "notes": ["Relative floor hints require exported floor display_order metadata."],
                "hint": floor_hint,
            }

        eligible_floor_ids: List[str] = []
        for record in floor_records:
            try:
                display_order = int(record.get("display_order"))
            except (TypeError, ValueError):
                continue
            if floor_hint.get("direction") == "up" and display_order > start_order:
                eligible_floor_ids.append(str(record.get("floor_id")))
            elif floor_hint.get("direction") == "down" and display_order < start_order:
                eligible_floor_ids.append(str(record.get("floor_id")))
        if not eligible_floor_ids:
            return {
                "ok": False,
                "eligible_floor_ids": [],
                "notes": [
                    f"Relative floor hint {floor_hint.get('raw_text')!r} does not map to any exported floor from start floor {start_floor_record.get('display_floor_id') or start_floor_id}.",
                ],
                "hint": floor_hint,
            }
        return {
            "ok": True,
            "eligible_floor_ids": eligible_floor_ids,
            "notes": [
                f"Relative floor hint {floor_hint.get('raw_text')!r} expands to {', '.join(eligible_floor_ids)} from start floor {start_floor_record.get('display_floor_id') or start_floor_id}.",
            ],
            "hint": dict(floor_hint),
        }

    def _resolve_room_request(
        self,
        parsed_request: Dict[str, Any],
        *,
        floor_hint_context: Dict[str, Any],
    ) -> Dict[str, Any]:
        target_text = str(parsed_request.get("target_text") or "")
        floor_ids = floor_hint_context.get("eligible_floor_ids")
        topology = self.query_api.topology
        candidate_rooms: List[Tuple[str, Dict[str, Any], str]] = []

        explicit_room_id = self._explicit_room_id(target_text)
        if explicit_room_id is not None:
            room_record = topology.get_room(explicit_room_id)
            if room_record is not None:
                candidate_rooms.append((explicit_room_id, room_record, "room_id"))
        else:
            for room_id in self._room_type_candidates(target_text):
                room_record = topology.get_room(room_id) or {}
                candidate_rooms.append((room_id, room_record, "room_type"))

        unfiltered_count = len(candidate_rooms)
        if floor_ids is not None:
            candidate_rooms = [
                item for item in candidate_rooms if item[1].get("floor_id") in set(floor_ids)
            ]

        if not candidate_rooms:
            notes = []
            if explicit_room_id is not None:
                notes.append(f"Room reference {explicit_room_id!r} is not present in the topology export.")
            else:
                notes.append(
                    "Room requests are limited to explicit room ids or unique named room_type matches in the current export."
                )
            if unfiltered_count > 0 and floor_ids is not None:
                notes.append("A room matched before floor filtering, but the floor hint removed all candidates.")
            notes.extend(floor_hint_context.get("notes", []))
            return {
                "resolved": False,
                "ambiguity_state": "unresolved",
                "target_type": "room",
                "task_target": None,
                "resolution_source": None,
                "matched_entity_ids": [],
                "candidate_matches": [],
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": None,
                "notes": notes,
                "floor_hint": floor_hint_context,
                "failure_reason": "room_target_unresolved",
            }

        candidate_rooms = sorted(candidate_rooms, key=lambda item: _room_sort_key(item[1], item[0]))
        if len(candidate_rooms) > 1:
            return {
                "resolved": False,
                "ambiguity_state": "ambiguous",
                "target_type": "room",
                "task_target": None,
                "resolution_source": None,
                "matched_entity_ids": [room_id for room_id, _, _ in candidate_rooms],
                "candidate_matches": [
                    {
                        "entity_id": room_id,
                        "room_type": room_record.get("room_type"),
                        "floor_id": room_record.get("floor_id"),
                        "display_floor_id": room_record.get("display_floor_id"),
                        "match_type": match_type,
                    }
                    for room_id, room_record, match_type in candidate_rooms
                ],
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": None,
                "notes": [
                    f"Room target {target_text!r} matches multiple rooms after filtering.",
                    *list(floor_hint_context.get("notes", [])),
                ],
                "floor_hint": floor_hint_context,
                "failure_reason": "room_target_ambiguous",
            }

        room_id, room_record, match_type = candidate_rooms[0]
        return {
            "resolved": True,
            "ambiguity_state": "none",
            "target_type": "room",
            "task_target": {"target_type": "room", "goal_room_id": room_id},
            "resolution_source": match_type,
            "matched_entity_ids": [room_id],
            "candidate_matches": [
                {
                    "entity_id": room_id,
                    "room_type": room_record.get("room_type"),
                    "floor_id": room_record.get("floor_id"),
                    "display_floor_id": room_record.get("display_floor_id"),
                    "match_type": match_type,
                }
            ],
            "resolved_room_id": room_id,
            "resolved_floor_id": room_record.get("floor_id"),
            "resolved_display_floor_id": room_record.get("display_floor_id"),
            "selected_entity_id": room_id,
            "selected_label": room_record.get("room_type") or room_id,
            "notes": [f"Room target resolved via {match_type}.", *list(floor_hint_context.get("notes", []))],
            "floor_hint": floor_hint_context,
            "failure_reason": None,
        }

    def _resolve_object_request(
        self,
        parsed_request: Dict[str, Any],
        *,
        floor_hint_context: Dict[str, Any],
    ) -> Dict[str, Any]:
        target_text = str(parsed_request.get("target_text") or "")
        topology = self.query_api.topology
        query_token = normalize_reference_slot(target_text)
        query_normalized = normalize_open_vocab_label(target_text)
        eligible_floor_ids = None if floor_hint_context.get("eligible_floor_ids") is None else set(floor_hint_context.get("eligible_floor_ids"))

        candidates: List[Dict[str, Any]] = []
        for object_id in topology.list_object_ids():
            record = topology.get_object(object_id) or {}
            record_label = record.get("label") or record.get("normalized_label")
            if not record_label:
                continue
            raw_token = normalize_reference_slot(record_label)
            normalized_label = record.get("normalized_label") or normalize_open_vocab_label(record_label)
            if raw_token == query_token:
                match_rank = 0
                match_type = "exact_label"
            elif normalized_label == query_normalized:
                match_rank = 1
                match_type = "normalized_label"
            else:
                continue
            if eligible_floor_ids is not None and record.get("floor_id") not in eligible_floor_ids:
                continue
            room_id = record.get("room_id")
            if room_id not in topology.graph.nodes:
                continue
            candidates.append(
                {
                    "record": dict(record),
                    "match_rank": match_rank,
                    "match_type": match_type,
                    "score": _score_value(
                        record.get("semantic_confidence")
                        or record.get("detection_confidence")
                        or record.get("score")
                    ),
                }
            )

        if not candidates:
            return {
                "resolved": False,
                "ambiguity_state": "unresolved",
                "target_type": "object",
                "task_target": None,
                "resolution_source": None,
                "matched_entity_ids": [],
                "candidate_matches": [],
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": None,
                "notes": [
                    f"No routed object label match found for {target_text!r}.",
                    *list(floor_hint_context.get("notes", [])),
                ],
                "floor_hint": floor_hint_context,
                "failure_reason": "object_target_unresolved",
            }

        candidates = sorted(
            candidates,
            key=lambda item: (
                int(item["match_rank"]),
                -float(item["score"]),
                str(item["record"].get("id")),
            ),
        )
        matched_entity_ids = [str(item["record"].get("id")) for item in candidates]
        candidate_matches = [
            _serialize_object_candidate(item["record"], rank=index + 1, match_type=str(item["match_type"]))
            for index, item in enumerate(candidates)
        ]
        resolved_room_ids = sorted({str(item["record"].get("room_id")) for item in candidates})
        if len(resolved_room_ids) > 1:
            return {
                "resolved": False,
                "ambiguity_state": "ambiguous",
                "target_type": "object",
                "task_target": None,
                "resolution_source": "object_label",
                "matched_entity_ids": matched_entity_ids,
                "candidate_matches": candidate_matches,
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": target_text,
                "notes": [
                    f"Object label {target_text!r} remains ambiguous across rooms: {', '.join(resolved_room_ids)}.",
                    *list(floor_hint_context.get("notes", [])),
                ],
                "floor_hint": floor_hint_context,
                "failure_reason": "object_target_ambiguous",
            }

        selected = candidates[0]["record"]
        room_record = topology.get_room(selected.get("room_id")) or {}
        notes = list(floor_hint_context.get("notes", []))
        if len(candidates) > 1:
            notes.append(
                f"Multiple objects matched {target_text!r}, but they collapse to room {selected.get('room_id')} for routing."
            )
        return {
            "resolved": True,
            "ambiguity_state": "none",
            "target_type": "object",
            "task_target": {
                "target_type": "object",
                "object_id": selected.get("id"),
                "object_label": selected.get("label") or selected.get("normalized_label"),
            },
            "resolution_source": "object_label",
            "matched_entity_ids": matched_entity_ids,
            "candidate_matches": candidate_matches,
            "resolved_room_id": selected.get("room_id"),
            "resolved_floor_id": room_record.get("floor_id"),
            "resolved_display_floor_id": room_record.get("display_floor_id"),
            "selected_entity_id": selected.get("id"),
            "selected_label": selected.get("label") or selected.get("normalized_label"),
            "notes": notes,
            "floor_hint": floor_hint_context,
            "failure_reason": None,
        }

    def _resolve_anchor_request(
        self,
        parsed_request: Dict[str, Any],
        *,
        floor_hint_context: Dict[str, Any],
    ) -> Dict[str, Any]:
        target_text = str(parsed_request.get("target_text") or "")
        topology = self.query_api.topology
        eligible_floor_ids = None if floor_hint_context.get("eligible_floor_ids") is None else set(floor_hint_context.get("eligible_floor_ids"))

        explicit_anchor_id = _canonical_anchor_id(target_text)
        if explicit_anchor_id is not None:
            anchor_record = topology.get_anchor(explicit_anchor_id)
            if anchor_record is not None and bool(anchor_record.get("valid", True)):
                room_id = anchor_record.get("room_id")
                room_record = topology.get_room(room_id) or {}
                if eligible_floor_ids is None or anchor_record.get("floor_id") in eligible_floor_ids:
                    return {
                        "resolved": True,
                        "ambiguity_state": "none",
                        "target_type": "anchor",
                        "task_target": {"target_type": "anchor", "anchor_id": explicit_anchor_id},
                        "resolution_source": "anchor_id",
                        "matched_entity_ids": [explicit_anchor_id],
                        "candidate_matches": [
                            _serialize_anchor_candidate(
                                anchor_record,
                                rank=1,
                                match_type="anchor_id",
                                target_label=self._anchor_target_label(anchor_record),
                            )
                        ],
                        "resolved_room_id": room_id,
                        "resolved_floor_id": room_record.get("floor_id"),
                        "resolved_display_floor_id": room_record.get("display_floor_id"),
                        "selected_entity_id": explicit_anchor_id,
                        "selected_label": explicit_anchor_id,
                        "notes": ["Anchor target resolved directly from anchor id.", *list(floor_hint_context.get("notes", []))],
                        "floor_hint": floor_hint_context,
                        "failure_reason": None,
                    }

        anchor_query = _ANCHOR_SUFFIX_RE.sub("", target_text).strip()
        anchor_query = anchor_query.removeprefix("anchor ").strip()
        query_token = normalize_reference_slot(anchor_query)
        query_normalized = normalize_open_vocab_label(anchor_query)
        candidates: List[Dict[str, Any]] = []
        for anchor_id in topology.list_anchor_ids(valid_only=True):
            record = topology.get_anchor(anchor_id) or {}
            if eligible_floor_ids is not None and record.get("floor_id") not in eligible_floor_ids:
                continue
            target_label = self._anchor_target_label(record)
            search_fields = [anchor_id, target_label]
            room_record = topology.get_room(record.get("room_id")) or {}
            search_fields.append(room_record.get("room_type"))
            match_rank = None
            match_type = None
            for field in search_fields:
                if not field:
                    continue
                field_token = normalize_reference_slot(field)
                field_normalized = normalize_open_vocab_label(field)
                if field_token == query_token:
                    match_rank = 0
                    match_type = "anchor_target_label" if field != anchor_id else "anchor_id"
                    break
                if field_normalized == query_normalized:
                    match_rank = 1
                    match_type = "anchor_target_label"
                    break
            if match_rank is None:
                continue
            candidates.append(
                {
                    "record": dict(record),
                    "match_rank": match_rank,
                    "match_type": match_type,
                    "target_label": target_label,
                    "score": _score_value(record.get("score") or self._anchor_target_score(record)),
                }
            )

        if not candidates:
            return {
                "resolved": False,
                "ambiguity_state": "unresolved",
                "target_type": "anchor",
                "task_target": None,
                "resolution_source": None,
                "matched_entity_ids": [],
                "candidate_matches": [],
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": None,
                "notes": [
                    "Anchor requests are limited to explicit anchor ids or anchors backed by inspectable room/object metadata.",
                    *list(floor_hint_context.get("notes", [])),
                ],
                "floor_hint": floor_hint_context,
                "failure_reason": "anchor_target_unresolved",
            }

        candidates = sorted(
            candidates,
            key=lambda item: (
                int(item["match_rank"]),
                -float(item["score"]),
                str(item["record"].get("id")),
            ),
        )
        matched_entity_ids = [str(item["record"].get("id")) for item in candidates]
        candidate_matches = [
            _serialize_anchor_candidate(
                item["record"],
                rank=index + 1,
                match_type=str(item["match_type"]),
                target_label=item.get("target_label"),
            )
            for index, item in enumerate(candidates)
        ]
        resolved_room_ids = sorted({str(item["record"].get("room_id")) for item in candidates})
        if len(resolved_room_ids) > 1:
            return {
                "resolved": False,
                "ambiguity_state": "ambiguous",
                "target_type": "anchor",
                "task_target": None,
                "resolution_source": "anchor_target_label",
                "matched_entity_ids": matched_entity_ids,
                "candidate_matches": candidate_matches,
                "resolved_room_id": None,
                "resolved_floor_id": None,
                "resolved_display_floor_id": None,
                "selected_entity_id": None,
                "selected_label": anchor_query,
                "notes": [
                    f"Anchor target {anchor_query!r} matches multiple routed rooms: {', '.join(resolved_room_ids)}.",
                    *list(floor_hint_context.get("notes", [])),
                ],
                "floor_hint": floor_hint_context,
                "failure_reason": "anchor_target_ambiguous",
            }

        selected = candidates[0]["record"]
        room_record = topology.get_room(selected.get("room_id")) or {}
        notes = list(floor_hint_context.get("notes", []))
        if len(candidates) > 1:
            notes.append(
                f"Multiple anchors matched {anchor_query!r}, but they collapse to room {selected.get('room_id')} for routing."
            )
        return {
            "resolved": True,
            "ambiguity_state": "none",
            "target_type": "anchor",
            "task_target": {"target_type": "anchor", "anchor_id": selected.get("id")},
            "resolution_source": "anchor_target_label",
            "matched_entity_ids": matched_entity_ids,
            "candidate_matches": candidate_matches,
            "resolved_room_id": selected.get("room_id"),
            "resolved_floor_id": room_record.get("floor_id"),
            "resolved_display_floor_id": room_record.get("display_floor_id"),
            "selected_entity_id": selected.get("id"),
            "selected_label": candidates[0].get("target_label") or selected.get("id"),
            "notes": notes,
            "floor_hint": floor_hint_context,
            "failure_reason": None,
        }

    def _room_type_candidates(self, target_text: str) -> List[str]:
        normalized_target = normalize_reference_slot(target_text)
        if not normalized_target:
            return []
        matches: List[str] = []
        for room_id in self.query_api.topology.list_room_ids():
            room_record = self.query_api.topology.get_room(room_id) or {}
            room_type = room_record.get("room_type")
            if not room_type or str(room_type).lower() == "unknown":
                continue
            if normalize_reference_slot(room_type) == normalized_target:
                matches.append(room_id)
        return matches

    def _explicit_room_id(self, target_text: str) -> Optional[str]:
        normalized = preprocess_instruction(target_text)
        if normalized in self.query_api.topology.graph.nodes:
            return str(normalized)
        match = _ROOM_ID_RE.match(normalized)
        if match:
            return f"room_{int(match.group(1))}"
        return None

    def _anchor_target_label(self, anchor_record: Dict[str, Any]) -> Optional[str]:
        anchor_type = str(anchor_record.get("anchor_type") or "")
        target_id = anchor_record.get("target_id")
        if anchor_type == "object":
            object_record = self.query_api.topology.get_object(target_id) or {}
            return object_record.get("label") or object_record.get("normalized_label") or object_record.get("category")
        if anchor_type == "room":
            room_record = self.query_api.topology.get_room(target_id) or {}
            return room_record.get("room_type") or room_record.get("id")
        return None

    def _anchor_target_score(self, anchor_record: Dict[str, Any]) -> float:
        anchor_type = str(anchor_record.get("anchor_type") or "")
        target_id = anchor_record.get("target_id")
        if anchor_type == "object":
            object_record = self.query_api.topology.get_object(target_id) or {}
            return _score_value(
                object_record.get("semantic_confidence")
                or object_record.get("detection_confidence")
                or object_record.get("score")
            )
        return 0.0

    def _dispatch_query(
        self,
        *,
        task_input: Dict[str, Any],
        route_policy: str,
    ) -> Tuple[str, Dict[str, Any]]:
        target = dict(task_input.get("target") or {})
        target_type = str(target.get("target_type") or "")
        start_room_id = task_input.get("start_room_id")
        if target_type == "room":
            return "query_route", self.query_api.query_route(
                start_room_id=start_room_id,
                goal_room_id=target.get("goal_room_id"),
                route_policy=route_policy,
            )
        if target_type == "anchor":
            return "query_route_to_anchor", self.query_api.query_route_to_anchor(
                start_room_id=start_room_id,
                anchor_id=target.get("anchor_id"),
                route_policy=route_policy,
            )
        return "query_route_to_object", self.query_api.query_route_to_object(
            start_room_id=start_room_id,
            object_id=target.get("object_id"),
            object_label=target.get("object_label"),
            route_policy=route_policy,
        )

    def _intent_summary(self, parsed_request: Dict[str, Any]) -> str:
        request_type = parsed_request.get("request_type")
        target_type = parsed_request.get("target_type")
        target_text = parsed_request.get("target_text")
        return f"{request_type}:{target_type}:{target_text}"

    def _resolved_target_summary(self, resolution: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "target_type": resolution.get("target_type"),
            "selected_entity_id": resolution.get("selected_entity_id"),
            "selected_label": resolution.get("selected_label"),
            "resolved_room_id": resolution.get("resolved_room_id"),
            "resolved_floor_id": resolution.get("resolved_floor_id"),
            "resolved_display_floor_id": resolution.get("resolved_display_floor_id"),
            "resolution_source": resolution.get("resolution_source"),
        }

    def _query_teacher_response(
        self,
        parsed_request: Dict[str, Any],
        resolution: Dict[str, Any],
        backend_result: Dict[str, Any],
    ) -> Dict[str, Any]:
        explanation = dict(backend_result.get("explanation") or {})
        floor_switches = list(explanation.get("floor_switches", []))
        return {
            "summary": (
                f"Interpreted {parsed_request.get('request_type')} request and delegated to the existing Query API. "
                f"{explanation.get('summary')}"
            ),
            "interpreted_intent": self._intent_summary(parsed_request),
            "resolved_target": self._resolved_target_summary(resolution),
            "route_summary": explanation.get("route_summary"),
            "floor_switch_required": bool(floor_switches),
            "execution_result": {
                "status": "not_started",
                "outcome_category": None,
                "outcome_reason": None,
            },
            "limitations": self._limitations_for_request(parsed_request, mode="query_only"),
            "notes": list(explanation.get("notes", [])) + list(resolution.get("notes", [])),
            "hop_summaries": list(explanation.get("hop_summaries", [])),
        }

    def _execution_teacher_response(
        self,
        parsed_request: Dict[str, Any],
        resolution: Dict[str, Any],
        backend_result: Dict[str, Any],
    ) -> Dict[str, Any]:
        outcome = dict(backend_result.get("outcome") or {})
        plan = dict(backend_result.get("plan") or {})
        return {
            "summary": (
                f"Interpreted execute request and delegated to the existing minimal closed-loop executor. "
                f"Outcome={outcome.get('outcome_category')} / {outcome.get('outcome_reason')}."
            ),
            "interpreted_intent": self._intent_summary(parsed_request),
            "resolved_target": self._resolved_target_summary(resolution),
            "route_summary": plan.get("route_summary"),
            "floor_switch_required": bool(plan.get("floor_switches")),
            "execution_result": {
                "status": "completed" if outcome.get("success") else "failed",
                "outcome_category": outcome.get("outcome_category"),
                "outcome_reason": outcome.get("outcome_reason"),
                "completed_step_count": outcome.get("completed_step_count"),
                "total_step_count": outcome.get("total_step_count"),
                "completion_frame_idx": outcome.get("completion_frame_idx"),
            },
            "limitations": self._limitations_for_request(parsed_request, mode="execute"),
            "notes": list(resolution.get("notes", [])),
            "plan_steps": [
                str(step.get("description"))
                for step in (((plan.get("symbolic_plan") or {}).get("steps")) or [])
            ],
        }

    def _resolution_failure_teacher_response(
        self,
        parsed_request: Dict[str, Any],
        resolution: Dict[str, Any],
    ) -> Dict[str, Any]:
        target_type = resolution.get("target_type")
        if resolution.get("ambiguity_state") == "ambiguous":
            summary = f"Interpreted {parsed_request.get('request_type')} request, but the {target_type} target is ambiguous."
        else:
            summary = f"Interpreted {parsed_request.get('request_type')} request, but the {target_type} target could not be resolved."
        return {
            "summary": summary,
            "interpreted_intent": self._intent_summary(parsed_request),
            "resolved_target": None,
            "route_summary": None,
            "floor_switch_required": None,
            "execution_result": None,
            "limitations": self._limitations_for_request(parsed_request, mode=parsed_request.get("mode") or "query_only"),
            "notes": list(resolution.get("notes", [])),
            "candidate_matches": list(resolution.get("candidate_matches", [])),
        }

    def _unsupported_teacher_response(self, parsed_request: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "summary": "Request is outside the intentionally narrow NL Tool-Use v0.1 scope.",
            "interpreted_intent": None,
            "resolved_target": None,
            "route_summary": None,
            "floor_switch_required": None,
            "execution_result": None,
            "limitations": [
                "v0.1 only supports simple room/object/anchor route questions and simple execute requests.",
                "The NL layer does not do free-form planning, graph search, or broad conversational reasoning.",
            ],
            "notes": list(parsed_request.get("parse_notes", [])),
        }

    def _limitations_for_request(self, parsed_request: Dict[str, Any], *, mode: str) -> List[str]:
        limitations = [
            "The NL layer only performs constrained interpretation, target resolution, and backend tool selection.",
            "Graph search stays inside the existing Query API and execution stays inside the existing closed-loop stack.",
        ]
        target_type = parsed_request.get("target_type")
        if target_type == "room":
            limitations.append(
                "Room requests are limited to explicit room ids or unique exported room_type names."
            )
        elif target_type == "anchor":
            limitations.append(
                "Anchor requests are limited to explicit anchor ids or anchors backed by inspectable room/object metadata."
            )
        else:
            limitations.append(
                "Object requests remain room-level and do not claim precise docking or manipulation."
            )
        if mode == "execute":
            limitations.append(
                "Execute mode reuses the replay-based minimal closed-loop executor and does not add a new planner."
            )
        return limitations


def render_teacher_brief(result: Dict[str, Any]) -> str:
    teacher_response = dict(result.get("teacher_response") or {})
    parts = [str(teacher_response.get("summary") or "").strip()]
    resolved_target = dict(teacher_response.get("resolved_target") or {})
    if resolved_target:
        parts.append(
            "resolved="
            f"{resolved_target.get('target_type')}:{resolved_target.get('selected_entity_id') or resolved_target.get('selected_label')} "
            f"-> {resolved_target.get('resolved_room_id')}"
        )
    route_summary = teacher_response.get("route_summary")
    if route_summary:
        parts.append(str(route_summary))
    execution_result = dict(teacher_response.get("execution_result") or {})
    if execution_result:
        outcome_category = execution_result.get("outcome_category")
        outcome_reason = execution_result.get("outcome_reason")
        if outcome_category or outcome_reason:
            parts.append(f"execution={outcome_category}/{outcome_reason}")
    return " ".join(part for part in parts if part)
