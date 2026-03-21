from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import _canonical_anchor_id, _canonical_room_id
from boxfusion.template_grounding.formatter import format_result
from boxfusion.template_grounding.normalizer import normalize_reference_slot
from boxfusion.template_grounding.parser import TemplateParser
from boxfusion.template_grounding.types import ParsedTemplate


class TemplateGrounder:
    def __init__(
        self,
        query_api: RoomTopologyQueryAPI,
        parser: Optional[TemplateParser] = None,
    ) -> None:
        self.query_api = query_api
        self.parser = parser or TemplateParser()

    @classmethod
    def from_json(cls, topology_json: Path) -> "TemplateGrounder":
        return cls(RoomTopologyQueryAPI.from_json(Path(topology_json)))

    def ground(
        self,
        instruction: str,
        start_room_id: Optional[str] = None,
        route_policy: str = "balanced",
    ) -> Dict[str, Any]:
        try:
            parse = self.parser.parse(instruction)
            if not parse.ok:
                return format_result(
                    ok=False,
                    instruction=instruction,
                    parse=parse,
                    grounding={},
                    query_api_request=None,
                    query_api_result=None,
                    failure_stage="parse",
                    failure_reason=parse.failure_reason,
                    explanation=self._parse_failure_explanation(parse),
                )

            if str(route_policy) not in self.query_api.route_policy_presets:
                return format_result(
                    ok=False,
                    instruction=instruction,
                    parse=parse,
                    grounding={},
                    query_api_request=None,
                    query_api_result=None,
                    failure_stage="policy",
                    failure_reason="INVALID_POLICY",
                    explanation=(
                        f"Template matched, but route policy {route_policy!r} is not defined in the existing Query API."
                    ),
                )

            if parse.intent == "go_to_room":
                return self._handle_go_to_room(parse, instruction, start_room_id, route_policy)
            if parse.intent == "go_from_room_to_room":
                return self._handle_go_from_room_to_room(parse, instruction, route_policy)
            if parse.intent == "go_to_room_with_object":
                return self._handle_go_to_room_with_object(parse, instruction, start_room_id, route_policy)
            if parse.intent == "go_to_anchor":
                return self._handle_go_to_anchor(parse, instruction, start_room_id, route_policy)

            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding={},
                query_api_request=None,
                query_api_result=None,
                failure_stage="dispatch",
                failure_reason="INTERNAL_ERROR",
                explanation=f"Template {parse.template_id!r} matched, but its intent handler is not implemented.",
            )
        except Exception as exc:  # pragma: no cover - defensive boundary for CLI/demo use
            return format_result(
                ok=False,
                instruction=instruction,
                parse=None,
                grounding={},
                query_api_request=None,
                query_api_result=None,
                failure_stage="internal",
                failure_reason="INTERNAL_ERROR",
                explanation=f"Template grounding raised an internal error: {exc}.",
            )

    def _handle_go_to_room(
        self,
        parse: ParsedTemplate,
        instruction: str,
        start_room_id: Optional[str],
        route_policy: str,
    ) -> Dict[str, Any]:
        target_resolution = self._resolve_room_slot(parse.raw_slots["room"], parse.normalized_slots["room"])
        if not target_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=self._base_grounding_payload(
                    query_kind="route_to_room",
                    grounded_target_type="room",
                    grounded_target=target_resolution,
                    start_room_resolution=None,
                ),
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_target",
                failure_reason="TARGET_ROOM_UNRESOLVED",
                explanation=(
                    f"Matched room-target template, but the target room slot {parse.raw_slots['room']!r} "
                    f"did not resolve to a unique room."
                ),
            )

        start_resolution = self._resolve_required_start_room(start_room_id)
        if not start_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=self._base_grounding_payload(
                    query_kind="route_to_room",
                    grounded_target_type="room",
                    grounded_target=target_resolution,
                    start_room_resolution=start_resolution,
                ),
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_start_room",
                failure_reason=start_resolution["template_failure_reason"],
                explanation=start_resolution["explanation"],
            )

        request = {
            "method": "query_route",
            "kwargs": {
                "start_room_id": start_resolution["resolved_room_id"],
                "goal_room_id": target_resolution["resolved_room_id"],
                "route_policy": str(route_policy),
            },
        }
        grounding = self._base_grounding_payload(
            query_kind="route_to_room",
            grounded_target_type="room",
            grounded_target=target_resolution,
            start_room_resolution=start_resolution,
        )
        return self._execute_query(
            instruction=instruction,
            parse=parse,
            grounding=grounding,
            query_api_request=request,
        )

    def _handle_go_from_room_to_room(
        self,
        parse: ParsedTemplate,
        instruction: str,
        route_policy: str,
    ) -> Dict[str, Any]:
        start_resolution = self._resolve_room_slot(parse.raw_slots["room_a"], parse.normalized_slots["room_a"])
        if not start_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=self._base_grounding_payload(
                    query_kind="route_to_room",
                    grounded_target_type="room",
                    grounded_target=None,
                    start_room_resolution=start_resolution,
                ),
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_start_room",
                failure_reason="TARGET_ROOM_UNRESOLVED",
                explanation=(
                    f"Matched room-to-room template, but the start room slot {parse.raw_slots['room_a']!r} "
                    f"did not resolve to a unique room."
                ),
            )

        target_resolution = self._resolve_room_slot(parse.raw_slots["room_b"], parse.normalized_slots["room_b"])
        if not target_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=self._base_grounding_payload(
                    query_kind="route_to_room",
                    grounded_target_type="room",
                    grounded_target=target_resolution,
                    start_room_resolution=start_resolution,
                ),
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_target",
                failure_reason="TARGET_ROOM_UNRESOLVED",
                explanation=(
                    f"Matched room-to-room template, but the target room slot {parse.raw_slots['room_b']!r} "
                    f"did not resolve to a unique room."
                ),
            )

        request = {
            "method": "query_route",
            "kwargs": {
                "start_room_id": start_resolution["resolved_room_id"],
                "goal_room_id": target_resolution["resolved_room_id"],
                "route_policy": str(route_policy),
            },
        }
        grounding = self._base_grounding_payload(
            query_kind="route_to_room",
            grounded_target_type="room",
            grounded_target=target_resolution,
            start_room_resolution=start_resolution,
        )
        return self._execute_query(
            instruction=instruction,
            parse=parse,
            grounding=grounding,
            query_api_request=request,
        )

    def _handle_go_to_room_with_object(
        self,
        parse: ParsedTemplate,
        instruction: str,
        start_room_id: Optional[str],
        route_policy: str,
    ) -> Dict[str, Any]:
        start_resolution = self._resolve_required_start_room(start_room_id)
        object_label = parse.normalized_slots["object_label"]
        grounding = self._base_grounding_payload(
            query_kind="route_to_object",
            grounded_target_type="object",
            grounded_target={
                "input_value": parse.raw_slots["object_label"],
                "normalized_value": object_label,
                "target_type": "object",
            },
            start_room_resolution=start_resolution,
        )
        if not start_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_start_room",
                failure_reason=start_resolution["template_failure_reason"],
                explanation=start_resolution["explanation"],
            )

        request = {
            "method": "query_route_to_object",
            "kwargs": {
                "start_room_id": start_resolution["resolved_room_id"],
                "object_label": object_label,
                "route_policy": str(route_policy),
            },
        }
        return self._execute_query(
            instruction=instruction,
            parse=parse,
            grounding=grounding,
            query_api_request=request,
        )

    def _handle_go_to_anchor(
        self,
        parse: ParsedTemplate,
        instruction: str,
        start_room_id: Optional[str],
        route_policy: str,
    ) -> Dict[str, Any]:
        start_resolution = self._resolve_required_start_room(start_room_id)
        anchor_id = _canonical_anchor_id(parse.normalized_slots["anchor_id"])
        grounding = self._base_grounding_payload(
            query_kind="route_to_anchor",
            grounded_target_type="anchor",
            grounded_target={
                "input_value": parse.raw_slots["anchor_id"],
                "normalized_value": parse.normalized_slots["anchor_id"],
                "canonical_anchor_id": anchor_id,
                "target_type": "anchor",
            },
            start_room_resolution=start_resolution,
        )
        if not start_resolution["resolved"]:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_start_room",
                failure_reason=start_resolution["template_failure_reason"],
                explanation=start_resolution["explanation"],
            )
        if anchor_id is None:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=None,
                query_api_result=None,
                failure_stage="grounding_target",
                failure_reason="ANCHOR_ID_UNRESOLVED",
                explanation=f"Matched anchor template, but anchor id {parse.raw_slots['anchor_id']!r} is invalid.",
            )

        request = {
            "method": "query_route_to_anchor",
            "kwargs": {
                "start_room_id": start_resolution["resolved_room_id"],
                "anchor_id": anchor_id,
                "route_policy": str(route_policy),
            },
        }
        return self._execute_query(
            instruction=instruction,
            parse=parse,
            grounding=grounding,
            query_api_request=request,
        )

    def _execute_query(
        self,
        *,
        instruction: str,
        parse: ParsedTemplate,
        grounding: Dict[str, Any],
        query_api_request: Dict[str, Any],
    ) -> Dict[str, Any]:
        method_name = str(query_api_request["method"])
        kwargs = dict(query_api_request.get("kwargs") or {})
        method = getattr(self.query_api, method_name, None)
        if method is None:
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=query_api_request,
                query_api_result=None,
                failure_stage="query_api_adapter",
                failure_reason="QUERY_API_CALL_FAILED",
                explanation=f"Query API method {method_name!r} is unavailable.",
            )

        try:
            query_api_result = method(**kwargs)
        except Exception as exc:  # pragma: no cover - defensive boundary for CLI/demo use
            return format_result(
                ok=False,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=query_api_request,
                query_api_result=None,
                failure_stage="query_api_call",
                failure_reason="QUERY_API_CALL_FAILED",
                explanation=f"Calling the existing Query API failed with error: {exc}.",
            )

        grounding = dict(grounding)
        grounding["query_api_target_resolution"] = dict(query_api_result.get("target_resolution") or {})
        grounding["resolved_start_room_id"] = query_api_result.get("start_room_id")
        grounding["resolved_goal_room_id"] = query_api_result.get("resolved_goal_room_id")

        if query_api_result.get("found"):
            return format_result(
                ok=True,
                instruction=instruction,
                parse=parse,
                grounding=grounding,
                query_api_request=query_api_request,
                query_api_result=query_api_result,
                failure_stage=None,
                failure_reason=None,
                explanation=self._success_explanation(parse, query_api_result),
            )

        failure_stage = self._query_failure_stage(query_api_result)
        failure_reason = self._template_failure_reason(parse.intent, query_api_result)
        return format_result(
            ok=False,
            instruction=instruction,
            parse=parse,
            grounding=grounding,
            query_api_request=query_api_request,
            query_api_result=query_api_result,
            failure_stage=failure_stage,
            failure_reason=failure_reason,
            explanation=self._failure_explanation(parse, query_api_result),
        )

    def _resolve_required_start_room(self, start_room_id: Optional[str]) -> Dict[str, Any]:
        if start_room_id is None or not str(start_room_id).strip():
            return {
                "resolved": False,
                "template_failure_reason": "START_ROOM_REQUIRED",
                "input_value": start_room_id,
                "normalized_value": None,
                "resolved_room_id": None,
                "candidate_room_ids": [],
                "match_type": None,
                "notes": ["A start room context is required for this template family."],
                "explanation": "Template matched, but no start room context was provided for route execution.",
            }

        resolution = self._resolve_room_slot(start_room_id, normalize_reference_slot(start_room_id))
        resolution["template_failure_reason"] = (
            None if resolution["resolved"] else "TARGET_ROOM_UNRESOLVED"
        )
        resolution["explanation"] = (
            f"Template matched, but start room {start_room_id!r} did not resolve to a unique room."
            if not resolution["resolved"]
            else f"Start room {start_room_id!r} resolved to {resolution['resolved_room_id']}."
        )
        return resolution

    def _resolve_room_slot(self, raw_value: Any, normalized_value: str) -> Dict[str, Any]:
        topology = self.query_api.topology
        candidate_room_ids = []
        match_type = None
        canonical_room_id = _canonical_room_id(raw_value)
        if canonical_room_id is not None and canonical_room_id in topology.graph.nodes:
            candidate_room_ids.append(canonical_room_id)
            match_type = "room_id"
        elif str(raw_value).strip() in topology.graph.nodes:
            candidate_room_ids.append(str(raw_value).strip())
            match_type = "room_id"
        else:
            for room_id in topology.list_room_ids():
                room_record = topology.get_room(room_id) or {}
                room_type = room_record.get("room_type")
                if normalize_reference_slot(room_type) == normalized_value and normalized_value:
                    candidate_room_ids.append(room_id)
            if candidate_room_ids:
                match_type = "room_type"

        candidate_room_ids = sorted(set(candidate_room_ids), key=str)
        resolved_room_id = candidate_room_ids[0] if len(candidate_room_ids) == 1 else None
        return {
            "resolved": resolved_room_id is not None,
            "input_value": raw_value,
            "normalized_value": normalized_value,
            "resolved_room_id": resolved_room_id,
            "candidate_room_ids": candidate_room_ids,
            "match_type": match_type,
            "notes": [] if resolved_room_id is not None else ["Room slot did not resolve uniquely."],
        }

    def _base_grounding_payload(
        self,
        *,
        query_kind: str,
        grounded_target_type: str,
        grounded_target: Optional[Dict[str, Any]],
        start_room_resolution: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        return {
            "query_kind": str(query_kind),
            "grounded_target_type": str(grounded_target_type),
            "grounded_target": None if grounded_target is None else dict(grounded_target),
            "start_room_resolution": None if start_room_resolution is None else dict(start_room_resolution),
            "resolved_start_room_id": None if start_room_resolution is None else start_room_resolution.get("resolved_room_id"),
            "resolved_goal_room_id": None,
        }

    def _template_failure_reason(self, intent: Optional[str], query_api_result: Dict[str, Any]) -> str:
        backend_reason = str(query_api_result.get("failure_reason") or "")
        target_reason = str((query_api_result.get("target_resolution") or {}).get("failure_reason") or "")
        if backend_reason == "invalid_route_policy":
            return "INVALID_POLICY"
        if intent == "go_to_anchor":
            if target_reason:
                return "ANCHOR_ID_UNRESOLVED"
            return "QUERY_API_NO_ROUTE"
        if intent == "go_to_room_with_object":
            if target_reason:
                return "OBJECT_LABEL_UNRESOLVED"
            return "QUERY_API_NO_ROUTE"
        if intent in {"go_to_room", "go_from_room_to_room"} and backend_reason in {
            "invalid_goal_room_id",
            "goal_room_not_in_topology",
        }:
            return "TARGET_ROOM_UNRESOLVED"
        return "QUERY_API_NO_ROUTE"

    def _query_failure_stage(self, query_api_result: Dict[str, Any]) -> str:
        if (query_api_result.get("target_resolution") or {}).get("failure_reason"):
            return "query_api_resolution"
        if (query_api_result.get("route") or {}).get("attempted"):
            return "query_api_route"
        return "query_api"

    def _parse_failure_explanation(self, parse: ParsedTemplate) -> str:
        if parse.failure_reason == "UNSUPPORTED_TEMPLATE":
            return (
                "Instruction does not match any supported Template Grounding v0.1 pattern. "
                "Supported families are room, room-to-room, object-target room, and anchor-target routes."
            )
        if parse.failure_reason == "MISSING_REQUIRED_SLOT":
            return "Template matched, but at least one required slot is missing."
        return "Template parsing failed during slot extraction."

    def _success_explanation(self, parse: ParsedTemplate, query_api_result: Dict[str, Any]) -> str:
        api_explanation = dict(query_api_result.get("explanation") or {})
        summary = api_explanation.get("summary") or "Query API route succeeded."
        route_summary = api_explanation.get("route_summary")
        if route_summary:
            return f"Matched {parse.template_id!r} and delegated to the existing Query API. {summary} {route_summary}"
        return f"Matched {parse.template_id!r} and delegated to the existing Query API. {summary}"

    def _failure_explanation(self, parse: ParsedTemplate, query_api_result: Dict[str, Any]) -> str:
        api_explanation = dict(query_api_result.get("explanation") or {})
        summary = api_explanation.get("summary") or "The existing Query API returned a structured failure."
        target_summary = api_explanation.get("target_summary")
        if target_summary:
            return f"Matched {parse.template_id!r}, but grounding failed downstream. {summary} {target_summary}"
        return f"Matched {parse.template_id!r}, but grounding failed downstream. {summary}"
