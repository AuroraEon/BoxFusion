from __future__ import annotations

from typing import Any, Dict, Optional

from boxfusion.template_grounding.types import ParsedTemplate, TEMPLATE_GROUNDING_VERSION


def format_result(
    *,
    ok: bool,
    instruction: str,
    parse: Optional[ParsedTemplate],
    grounding: Optional[Dict[str, Any]],
    query_api_request: Optional[Dict[str, Any]],
    query_api_result: Optional[Dict[str, Any]],
    failure_stage: Optional[str],
    failure_reason: Optional[str],
    explanation: str,
) -> Dict[str, Any]:
    return {
        "ok": bool(ok),
        "instruction": instruction,
        "template_grounding_version": TEMPLATE_GROUNDING_VERSION,
        "parse": _format_parse(parse),
        "grounding": dict(grounding or {}),
        "query_api_request": None if query_api_request is None else dict(query_api_request),
        "query_api_result": None if query_api_result is None else dict(query_api_result),
        "failure_stage": failure_stage,
        "failure_reason": failure_reason,
        "explanation": explanation,
    }


def _format_parse(parse: Optional[ParsedTemplate]) -> Dict[str, Any]:
    if parse is None:
        return {
            "template_id": None,
            "intent": None,
            "raw_slots": {},
            "normalized_slots": {},
            "failure_reason": None,
            "notes": [],
        }
    return parse.to_dict()
