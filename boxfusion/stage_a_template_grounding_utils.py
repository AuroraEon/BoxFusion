from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
import re
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

FAMILY_DISPLAY_NAMES: Dict[str, str] = {
    "explicit_room_target": "room-id / explicit room target",
    "semantic_room_target": "semantic room-name target",
    "room_to_room": "room-to-room",
    "object_label_target": "object-label target",
    "anchor_id_target": "anchor-id target",
    "unsupported_template": "expected unsupported template",
    "unknown": "unknown",
}

FAMILY_ORDER: Tuple[str, ...] = (
    "explicit_room_target",
    "semantic_room_target",
    "room_to_room",
    "object_label_target",
    "anchor_id_target",
    "unsupported_template",
    "unknown",
)

RESULT_CATEGORY_ORDER: Tuple[str, ...] = (
    "PASS",
    "PARSE_FAIL",
    "GROUNDING_FAIL",
    "QUERY_FAIL",
    "NO_ROUTE",
    "UNSUPPORTED_ON_CURRENT_EXPORT",
)

_EXPLICIT_ROOM_RE = re.compile(r"^room_[0-9]+$")
_NON_SEMANTIC_ROOM_TYPES = {
    "",
    "unknown",
    "none",
    "null",
    "room",
    "other",
    "unlabeled",
    "unassigned",
}
_TRAILING_PUNCT_RE = re.compile(r"[.!?,;:]+$")
_INSTRUCTION_SPACE_RE = re.compile(r"\s+")
_REFERENCE_TOKEN_RE = re.compile(r"[^a-z0-9_/\- ]+")


DEFAULT_STATIC_ACCEPTANCE_CASES: Tuple[Dict[str, Any], ...] = (
    {
        "instruction": "go from room_10 to room_13",
        "family": "room_to_room",
        "source": "default_static",
    },
    {
        "instruction": "go to room_13",
        "family": "explicit_room_target",
        "source": "default_static",
    },
    {
        "instruction": "go to the room with refrigerator",
        "family": "object_label_target",
        "source": "default_static",
    },
    {
        "instruction": "go to the room with sofa",
        "family": "object_label_target",
        "source": "default_static",
    },
    {
        "instruction": "go to anchor anchor_1",
        "family": "anchor_id_target",
        "source": "default_static",
    },
    {
        "instruction": "go to anchor anchor_5",
        "family": "anchor_id_target",
        "source": "default_static",
    },
    {
        "instruction": "go to kitchen",
        "family": "semantic_room_target",
        "source": "default_static",
    },
    {
        "instruction": "go to the kitchen",
        "family": "semantic_room_target",
        "source": "default_static",
    },
    {
        "instruction": "go from living_room to bedroom",
        "family": "room_to_room",
        "source": "default_static",
    },
    {
        "instruction": "find sofa",
        "family": "unsupported_template",
        "source": "default_static",
    },
    {
        "instruction": "go near kitchen",
        "family": "unsupported_template",
        "source": "default_static",
    },
    {
        "instruction": "take me to bedroom",
        "family": "unsupported_template",
        "source": "default_static",
    },
    {
        "instruction": "go to object sofa",
        "family": "unsupported_template",
        "source": "default_static",
    },
)


def preprocess_instruction(instruction: Any) -> str:
    text = "" if instruction is None else str(instruction).strip().lower()
    text = _TRAILING_PUNCT_RE.sub("", text)
    return _INSTRUCTION_SPACE_RE.sub(" ", text).strip()


def normalize_reference_slot(value: Any) -> str:
    text = preprocess_instruction(value)
    text = _REFERENCE_TOKEN_RE.sub(" ", text)
    text = text.replace("-", " ")
    return "_".join(text.split())


def canonicalize_room_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    token = str(value).strip()
    if not token:
        return None
    if token.startswith("room_"):
        suffix = token[5:]
        return token if suffix.isdigit() else token
    if token.isdigit():
        return f"room_{int(token)}"
    return token


def family_display_name(family: Optional[str]) -> str:
    return FAMILY_DISPLAY_NAMES.get(str(family or "unknown"), str(family or "unknown"))


def build_export_metadata_summary(topology: Any, sample_limit: int = 5) -> Dict[str, Any]:
    inspection = topology.inspect_export(sample_limit=sample_limit)
    room_type_counter: Counter[str] = Counter()
    usable_room_type_counter: Counter[str] = Counter()

    for room_id in topology.list_room_ids():
        room_record = topology.get_room(room_id) or {}
        normalized_room_type = normalize_reference_slot(room_record.get("room_type"))
        room_type_counter[normalized_room_type or ""] += 1
        if _is_usable_room_type(normalized_room_type):
            usable_room_type_counter[normalized_room_type] += 1

    usable_room_types = sorted(usable_room_type_counter)
    usable_room_type_count = sum(int(count) for count in usable_room_type_counter.values())
    room_count = int(inspection.get("room_count", 0))
    semantic_room_name_available = bool(usable_room_types)

    diagnostics = list(inspection.get("diagnostics", []))
    if not semantic_room_name_available:
        diagnostics.append("semantic room-name grounding unavailable because room labels are missing or room_type is unknown")

    family_capabilities = {
        "explicit_room_target": {
            "available": room_count > 0,
            "note": None if room_count > 0 else "No rooms are present in this export.",
        },
        "semantic_room_target": {
            "available": semantic_room_name_available,
            "note": None
            if semantic_room_name_available
            else "Semantic room-name grounding is unavailable on this export because room labels are missing or room_type is unknown.",
        },
        "room_to_room": {
            "available": room_count > 0,
            "note": None if room_count > 0 else "No rooms are present in this export.",
        },
        "object_label_target": {
            "available": bool(inspection.get("object_label_count", 0)),
            "note": None
            if inspection.get("object_label_count", 0)
            else "Object-label grounding is unavailable on this export because no inspectable object labels are present.",
        },
        "anchor_id_target": {
            "available": bool(inspection.get("anchor_count", 0)),
            "note": None
            if inspection.get("anchor_count", 0)
            else "Anchor-id grounding is unavailable on this export because no anchors are present.",
        },
    }

    return {
        "inspection": inspection,
        "room_type_summary": {
            "room_type_histogram": dict(sorted(room_type_counter.items(), key=lambda item: item[0])),
            "usable_room_type_histogram": dict(sorted(usable_room_type_counter.items(), key=lambda item: item[0])),
            "usable_room_type_count": usable_room_type_count,
            "room_count": room_count,
            "usable_room_type_coverage": round((usable_room_type_count / room_count), 3) if room_count else 0.0,
            "usable_room_types": usable_room_types[:sample_limit],
        },
        "capabilities": family_capabilities,
        "diagnostics": diagnostics,
    }


def choose_default_start_room_id(
    topology: Any,
    preferred_start_room_id: Optional[str] = None,
) -> Tuple[Optional[str], bool]:
    room_ids = list(topology.list_room_ids())
    canonical_preferred = canonicalize_room_id(preferred_start_room_id)
    if canonical_preferred and canonical_preferred in room_ids:
        return canonical_preferred, False
    if room_ids:
        return room_ids[0], True
    return canonical_preferred, canonical_preferred is not None


def build_default_acceptance_cases(
    topology: Any,
    start_room_id: Optional[str],
    export_summary: Dict[str, Any],
    include_export_probes: bool = True,
) -> List[Dict[str, Any]]:
    cases = [dict(item, start_room_id=start_room_id if _family_requires_start_room(item["family"]) else None) for item in DEFAULT_STATIC_ACCEPTANCE_CASES]
    if include_export_probes:
        cases.extend(_build_export_probe_cases(topology, start_room_id, export_summary))
    return _dedupe_cases(cases)


def load_instruction_cases(
    path: Path,
    default_start_room_id: Optional[str] = None,
) -> List[Dict[str, Any]]:
    path = Path(path)
    suffix = path.suffix.lower()
    raw_text = path.read_text(encoding="utf-8")

    if suffix == ".json":
        payload = json.loads(raw_text)
    elif suffix in {".yaml", ".yml"}:
        payload = _load_yaml_payload(raw_text)
    else:
        payload = _load_text_payload(raw_text)

    cases = _normalize_case_payload(payload)
    normalized_cases = []
    for item in cases:
        family = item.get("family") or infer_instruction_family(item.get("instruction"))
        start_room_id = item.get("start_room_id", item.get("start_room", default_start_room_id))
        normalized_cases.append(
            {
                "instruction": str(item["instruction"]),
                "family": family,
                "start_room_id": start_room_id if _family_requires_start_room(family) else item.get("start_room_id", item.get("start_room")),
                "source": str(item.get("source", f"file:{path.name}")),
                "description": item.get("description"),
            }
        )
    return normalized_cases


def build_acceptance_case_result(
    *,
    case_id: str,
    case: Dict[str, Any],
    result: Dict[str, Any],
    export_summary: Dict[str, Any],
) -> Dict[str, Any]:
    parse = dict(result.get("parse") or {})
    query_api_result = result.get("query_api_result") or {}
    route = query_api_result.get("route") or {}
    family = str(case.get("family") or infer_instruction_family(case.get("instruction"), parse))
    family_capability = dict((export_summary.get("capabilities") or {}).get(family) or {})
    parse_success = parse.get("failure_reason") is None and bool(parse.get("template_id"))
    query_api_invoked = result.get("query_api_request") is not None
    query_api_returned = result.get("query_api_result") is not None
    query_api_call_success = bool(query_api_invoked and query_api_returned)
    route_found = bool(query_api_result.get("found"))
    route_attempted = bool(route.get("attempted"))
    category = classify_acceptance_result(result=result, family=family, export_summary=export_summary)

    notes = []
    notes.extend(str(item) for item in parse.get("notes", []))
    notes.extend(str(item) for item in (result.get("grounding") or {}).get("notes", []))
    notes.extend(str(item) for item in (query_api_result.get("target_resolution") or {}).get("notes", []))
    if family_capability.get("note") and category == "UNSUPPORTED_ON_CURRENT_EXPORT":
        notes.append(str(family_capability["note"]))
    notes.append(str(result.get("explanation", "")))
    notes = _dedupe_text(notes)

    return {
        "case_id": case_id,
        "instruction": case.get("instruction"),
        "family": family,
        "family_display_name": family_display_name(family),
        "source": case.get("source", "manual"),
        "start_room_id": case.get("start_room_id"),
        "route_policy": case.get("route_policy", "balanced"),
        "matched_template": parse.get("template_id"),
        "matched_intent": parse.get("intent"),
        "result_category": category,
        "passed": category == "PASS",
        "parse_success": parse_success,
        "grounding_success": bool(result.get("query_api_request") is not None),
        "query_api_invoked": query_api_invoked,
        "query_api_returned": query_api_returned,
        "query_api_call_success": query_api_call_success,
        "route_attempted": route_attempted,
        "route_found": route_found,
        "failure_stage": result.get("failure_stage"),
        "failure_reason": result.get("failure_reason"),
        "resolved_start_room_id": (result.get("grounding") or {}).get("resolved_start_room_id"),
        "resolved_goal_room_id": (result.get("grounding") or {}).get("resolved_goal_room_id"),
        "notes": notes,
        "grounding_result": result,
    }


def classify_acceptance_result(
    *,
    result: Dict[str, Any],
    family: str,
    export_summary: Dict[str, Any],
) -> str:
    if result.get("ok"):
        return "PASS"

    parse = dict(result.get("parse") or {})
    if parse.get("failure_reason"):
        return "PARSE_FAIL"

    family_capability = dict((export_summary.get("capabilities") or {}).get(family) or {})
    if family_capability.get("available") is False:
        return "UNSUPPORTED_ON_CURRENT_EXPORT"

    failure_stage = str(result.get("failure_stage") or "")
    query_api_result = result.get("query_api_result") or {}
    route = query_api_result.get("route") or {}

    if failure_stage in {"grounding_target", "grounding_start_room"}:
        return "GROUNDING_FAIL"
    if route.get("attempted") and not query_api_result.get("found"):
        return "NO_ROUTE"
    if failure_stage.startswith("query_api") or failure_stage in {"policy", "dispatch", "internal"}:
        return "QUERY_FAIL"
    return "GROUNDING_FAIL"


def infer_instruction_family(instruction: Optional[str], parse: Optional[Dict[str, Any]] = None) -> str:
    parse = dict(parse or {})
    intent = parse.get("intent")
    raw_slots = dict(parse.get("raw_slots") or {})
    normalized_instruction = preprocess_instruction(instruction)

    if intent == "go_to_anchor":
        return "anchor_id_target"
    if intent == "go_to_room_with_object":
        return "object_label_target"
    if intent == "go_from_room_to_room":
        return "room_to_room"
    if intent == "go_to_room":
        room_slot = str(raw_slots.get("room") or "")
        return "explicit_room_target" if _is_explicit_room_token(room_slot) else "semantic_room_target"

    if normalized_instruction.startswith("go to anchor "):
        return "anchor_id_target"
    if normalized_instruction.startswith("go to the room with "):
        return "object_label_target"
    if normalized_instruction.startswith("go from "):
        return "room_to_room"
    if normalized_instruction.startswith("go to object "):
        return "unsupported_template"
    if normalized_instruction.startswith("go near ") or normalized_instruction.startswith("find ") or normalized_instruction.startswith("take me to "):
        return "unsupported_template"
    if normalized_instruction.startswith("go to the "):
        room_token = normalized_instruction[len("go to the ") :]
        return "explicit_room_target" if _is_explicit_room_token(room_token) else "semantic_room_target"
    if normalized_instruction.startswith("go to "):
        room_token = normalized_instruction[len("go to ") :]
        return "explicit_room_target" if _is_explicit_room_token(room_token) else "semantic_room_target"
    return "unknown"


def aggregate_acceptance_results(
    *,
    topology_json: Path,
    sequence_id: Optional[str],
    route_policy: str,
    effective_start_room_id: Optional[str],
    auto_selected_start_room_id: bool,
    export_summary: Dict[str, Any],
    case_results: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    total_cases = len(case_results)
    category_counts = Counter(item.get("result_category") for item in case_results)
    failure_reason_counts = Counter(
        str(item.get("failure_reason"))
        for item in case_results
        if item.get("failure_reason")
    )

    by_family: Dict[str, Dict[str, Any]] = {}
    for family in FAMILY_ORDER:
        family_cases = [item for item in case_results if item.get("family") == family]
        if not family_cases:
            continue
        family_capability = dict((export_summary.get("capabilities") or {}).get(family) or {})
        family_category_counts = Counter(item.get("result_category") for item in family_cases)
        by_family[family] = {
            "family_display_name": family_display_name(family),
            "total_cases": len(family_cases),
            "passed_cases": sum(1 for item in family_cases if item.get("passed")),
            "failed_cases": sum(1 for item in family_cases if not item.get("passed")),
            "pass_rate": round(sum(1 for item in family_cases if item.get("passed")) / len(family_cases), 3),
            "result_category_histogram": {key: int(family_category_counts.get(key, 0)) for key in RESULT_CATEGORY_ORDER if family_category_counts.get(key, 0)},
            "available_on_current_export": family_capability.get("available"),
            "family_note": family_capability.get("note"),
            "cases": [item["case_id"] for item in family_cases],
        }

    return {
        "topology_json": str(Path(topology_json)),
        "sequence_id": sequence_id,
        "route_policy": str(route_policy),
        "effective_start_room_id": effective_start_room_id,
        "auto_selected_start_room_id": bool(auto_selected_start_room_id),
        "export_summary": export_summary,
        "total_cases": total_cases,
        "passed_cases": sum(1 for item in case_results if item.get("passed")),
        "failed_cases": sum(1 for item in case_results if not item.get("passed")),
        "pass_rate": round(sum(1 for item in case_results if item.get("passed")) / total_cases, 3) if total_cases else 0.0,
        "parse_successes": sum(1 for item in case_results if item.get("parse_success")),
        "parse_failures": sum(1 for item in case_results if not item.get("parse_success")),
        "grounding_successes": sum(1 for item in case_results if item.get("grounding_success")),
        "grounding_failures": sum(1 for item in case_results if not item.get("grounding_success") and item.get("parse_success")),
        "query_api_invoked_cases": sum(1 for item in case_results if item.get("query_api_invoked")),
        "query_api_returned_cases": sum(1 for item in case_results if item.get("query_api_returned")),
        "query_api_call_failures": sum(1 for item in case_results if item.get("query_api_invoked") and not item.get("query_api_call_success")),
        "route_found_cases": sum(1 for item in case_results if item.get("route_found")),
        "no_route_cases": sum(1 for item in case_results if item.get("result_category") == "NO_ROUTE"),
        "result_category_histogram": {key: int(category_counts.get(key, 0)) for key in RESULT_CATEGORY_ORDER if category_counts.get(key, 0)},
        "failure_reason_histogram": dict(sorted(failure_reason_counts.items(), key=lambda item: (-item[1], item[0]))),
        "by_family": by_family,
        "cases": list(case_results),
    }


def render_acceptance_markdown(report: Dict[str, Any]) -> str:
    export_summary = dict(report.get("export_summary") or {})
    inspection = dict(export_summary.get("inspection") or {})
    room_type_summary = dict(export_summary.get("room_type_summary") or {})

    lines = [
        "# Stage A Template Grounding Acceptance",
        "",
        "## Export Summary",
        f"- topology_json: `{report.get('topology_json')}`",
        f"- sequence_id: `{report.get('sequence_id')}`",
        f"- route_policy: `{report.get('route_policy')}`",
        f"- effective_start_room_id: `{report.get('effective_start_room_id')}`",
        f"- auto_selected_start_room_id: `{report.get('auto_selected_start_room_id')}`",
        f"- counts: rooms={inspection.get('room_count', 0)} objects={inspection.get('object_count', 0)} anchors={inspection.get('anchor_count', 0)} object_labels={inspection.get('object_label_count', 0)}",
        f"- usable_room_type_coverage: {room_type_summary.get('usable_room_type_count', 0)}/{room_type_summary.get('room_count', 0)} ({room_type_summary.get('usable_room_type_coverage', 0.0)})",
    ]

    diagnostics = list(export_summary.get("diagnostics") or [])
    if diagnostics:
        lines.extend(["", "## Export Diagnostics"])
        lines.extend([f"- {item}" for item in diagnostics])

    lines.extend(
        [
            "",
            "## Summary",
            f"- total_cases: {report.get('total_cases', 0)}",
            f"- passed_cases: {report.get('passed_cases', 0)}",
            f"- failed_cases: {report.get('failed_cases', 0)}",
            f"- pass_rate: {report.get('pass_rate', 0.0)}",
            f"- parse_successes: {report.get('parse_successes', 0)}",
            f"- grounding_successes: {report.get('grounding_successes', 0)}",
            f"- query_api_returned_cases: {report.get('query_api_returned_cases', 0)}",
            f"- route_found_cases: {report.get('route_found_cases', 0)}",
        ]
    )

    histogram = dict(report.get("result_category_histogram") or {})
    if histogram:
        lines.extend(["", "## Result Categories"])
        lines.extend([f"- {key}: {value}" for key, value in histogram.items()])

    by_family = dict(report.get("by_family") or {})
    if by_family:
        lines.extend(["", "## Family Breakdown"])
        for family in FAMILY_ORDER:
            family_report = dict(by_family.get(family) or {})
            if not family_report:
                continue
            lines.append(
                f"- {family_report.get('family_display_name')}: {family_report.get('passed_cases', 0)}/{family_report.get('total_cases', 0)} passed"
            )
            if family_report.get("family_note"):
                lines.append(f"  note: {family_report.get('family_note')}")

    lines.extend(["", "## Cases", "", "| case_id | family | instruction | start_room | category | template | goal_room | failure_reason |", "| --- | --- | --- | --- | --- | --- | --- | --- |"])
    for item in report.get("cases", []):
        lines.append(
            "| {case_id} | {family} | `{instruction}` | `{start}` | {category} | `{template}` | `{goal}` | `{reason}` |".format(
                case_id=item.get("case_id"),
                family=item.get("family_display_name"),
                instruction=item.get("instruction"),
                start=item.get("start_room_id"),
                category=item.get("result_category"),
                template=item.get("matched_template"),
                goal=item.get("resolved_goal_room_id"),
                reason=item.get("failure_reason"),
            )
        )
    return "\n".join(lines) + "\n"


def render_acceptance_console_summary(report: Dict[str, Any]) -> str:
    histogram = dict(report.get("result_category_histogram") or {})
    histogram_text = ", ".join(f"{key}={value}" for key, value in histogram.items()) or "no cases"
    return (
        f"Template grounding acceptance: {report.get('passed_cases', 0)}/{report.get('total_cases', 0)} passed "
        f"(pass_rate={report.get('pass_rate', 0.0)}). {histogram_text}."
    )


def build_minimal_vln_demo_payload(
    *,
    instruction: str,
    start_room_id: Optional[str],
    route_policy: str,
    result: Dict[str, Any],
    export_summary: Dict[str, Any],
) -> Dict[str, Any]:
    parse = dict(result.get("parse") or {})
    query_api_result = result.get("query_api_result") or {}
    route = dict(query_api_result.get("route") or {})
    family = infer_instruction_family(instruction, parse)
    category = classify_acceptance_result(result=result, family=family, export_summary=export_summary)
    target_resolution = dict(query_api_result.get("target_resolution") or {})
    room_sequence = list(route.get("room_sequence", []))
    relation_sequence = list(route.get("used_relation_types", []))

    return {
        "instruction": instruction,
        "route_policy": str(route_policy),
        "supported": parse.get("failure_reason") is None and bool(parse.get("template_id")),
        "supported_on_current_export": category not in {"UNSUPPORTED_ON_CURRENT_EXPORT", "PARSE_FAIL"},
        "result_category": category,
        "matched_template": parse.get("template_id"),
        "matched_intent": parse.get("intent"),
        "instruction_family": family,
        "instruction_family_display_name": family_display_name(family),
        "grounded_target_summary": build_grounded_target_summary(result=result),
        "start_room_summary": {
            "requested_start_room_id": start_room_id,
            "resolved_start_room_id": query_api_result.get("start_room_id") or (result.get("grounding") or {}).get("resolved_start_room_id"),
        },
        "resolved_goal_room_summary": {
            "resolved_goal_room_id": query_api_result.get("resolved_goal_room_id") or (result.get("grounding") or {}).get("resolved_goal_room_id"),
            "target_resolution_summary": (query_api_result.get("explanation") or {}).get("target_summary"),
            "target_resolution_failure_reason": target_resolution.get("failure_reason"),
        },
        "route_summary": {
            "found": bool(query_api_result.get("found")),
            "room_sequence": room_sequence,
            "hop_count": route.get("hop_count"),
            "relation_sequence": relation_sequence,
            "route_confidence": route.get("route_confidence"),
            "total_cost": route.get("total_cost"),
            "candidate_path_count": len(route.get("candidate_paths", [])),
            "steps": format_route_steps(query_api_result),
        },
        "teacher_explanation": build_teacher_explanation(
            instruction_family=family,
            result=result,
            export_summary=export_summary,
        ),
        "one_line_summary": build_one_line_demo_summary(
            instruction=instruction,
            result=result,
            family=family,
            export_summary=export_summary,
        ),
        "grounding_result": result,
    }


def build_grounded_target_summary(*, result: Dict[str, Any]) -> Dict[str, Any]:
    grounding = dict(result.get("grounding") or {})
    grounded_target = dict(grounding.get("grounded_target") or {})
    query_api_result = result.get("query_api_result") or {}
    target_resolution = dict(query_api_result.get("target_resolution") or {})
    return {
        "grounded_target_type": grounding.get("grounded_target_type"),
        "input_value": grounded_target.get("input_value"),
        "normalized_value": grounded_target.get("normalized_value"),
        "resolved_room_id": target_resolution.get("resolved_room_id") or grounding.get("resolved_goal_room_id"),
        "matched_entity_ids": list(target_resolution.get("matched_entity_ids", [])),
        "summary": (query_api_result.get("explanation") or {}).get("target_summary"),
    }


def build_teacher_explanation(
    *,
    instruction_family: str,
    result: Dict[str, Any],
    export_summary: Dict[str, Any],
) -> str:
    parse = dict(result.get("parse") or {})
    query_api_result = result.get("query_api_result") or {}
    route = dict(query_api_result.get("route") or {})
    parts = []

    if parse.get("template_id"):
        parts.append(f"Instruction matched the {family_display_name(instruction_family)} template family.")
    else:
        parts.append("Instruction did not match any supported Template Grounding v0.1 template.")

    target_summary = (query_api_result.get("explanation") or {}).get("target_summary")
    if target_summary:
        parts.append(target_summary)

    if query_api_result.get("found"):
        room_sequence = list(route.get("room_sequence", []))
        if room_sequence:
            parts.append(f"Computed room-level route: {' -> '.join(room_sequence)}.")
        if route.get("route_confidence") is not None and float(route.get("route_confidence")) < 0.35:
            parts.append(
                f"This is a weak route under the {query_api_result.get('route_policy', 'balanced')} policy because confidence is low."
            )
    elif route.get("attempted"):
        parts.append(
            f"No room-level route was found under the {query_api_result.get('route_policy', 'balanced')} policy."
        )

    family_capability = dict((export_summary.get("capabilities") or {}).get(instruction_family) or {})
    if family_capability.get("note") and not result.get("ok"):
        parts.append(str(family_capability["note"]))

    if result.get("failure_reason") and not result.get("ok"):
        parts.append(f"Failure reason: {result.get('failure_reason')}.")

    return " ".join(_dedupe_text(parts))


def build_one_line_demo_summary(
    *,
    instruction: str,
    result: Dict[str, Any],
    family: str,
    export_summary: Dict[str, Any],
) -> str:
    category = classify_acceptance_result(result=result, family=family, export_summary=export_summary)
    query_api_result = result.get("query_api_result") or {}
    goal_room_id = query_api_result.get("resolved_goal_room_id") or (result.get("grounding") or {}).get("resolved_goal_room_id")
    if category == "PASS":
        return f"{instruction!r} -> PASS, goal={goal_room_id}, route={' -> '.join((query_api_result.get('route') or {}).get('room_sequence', []))}"
    return f"{instruction!r} -> {category}, reason={result.get('failure_reason')}"


def format_route_steps(query_api_result: Dict[str, Any]) -> List[str]:
    route = dict(query_api_result.get("route") or {})
    room_sequence = list(route.get("room_sequence", []))
    edges = list(route.get("edges", []))
    if not room_sequence:
        return []
    if not edges and len(room_sequence) == 1:
        return [f"Step 1: stay in {room_sequence[0]} (already at the goal room)."]

    steps = []
    for index, edge in enumerate(edges, start=1):
        relation = edge.get("relation_type", "unknown_relation")
        confidence = edge.get("confidence")
        cost = edge.get("edge_cost")
        steps.append(
            f"Step {index}: go from {edge.get('source_room_id')} to {edge.get('target_room_id')} via {relation}"
            f" (conf={confidence}, cost={cost})."
        )
    return steps


def _build_export_probe_cases(
    topology: Any,
    start_room_id: Optional[str],
    export_summary: Dict[str, Any],
) -> List[Dict[str, Any]]:
    cases: List[Dict[str, Any]] = []
    room_ids = list(topology.list_room_ids())
    if room_ids:
        start_room = canonicalize_room_id(start_room_id) or room_ids[0]
        goal_room = next((room_id for room_id in room_ids if room_id != start_room), room_ids[0])
        cases.append(
            {
                "instruction": f"go to {goal_room}",
                "family": "explicit_room_target",
                "start_room_id": start_room,
                "source": "export_probe",
                "description": "Auto-generated explicit room target from the current export.",
            }
        )
        if len(room_ids) >= 2:
            cases.append(
                {
                    "instruction": f"go from {room_ids[0]} to {room_ids[-1]}",
                    "family": "room_to_room",
                    "start_room_id": None,
                    "source": "export_probe",
                    "description": "Auto-generated room-to-room probe from the current export.",
                }
            )

    object_labels = list(topology.list_object_labels())
    if object_labels:
        cases.append(
            {
                "instruction": f"go to the room with {object_labels[0]}",
                "family": "object_label_target",
                "start_room_id": start_room_id,
                "source": "export_probe",
                "description": "Auto-generated object-label probe from the current export.",
            }
        )

    anchor_ids = list(topology.list_anchor_ids(valid_only=True))
    if anchor_ids:
        cases.append(
            {
                "instruction": f"go to anchor {anchor_ids[0]}",
                "family": "anchor_id_target",
                "start_room_id": start_room_id,
                "source": "export_probe",
                "description": "Auto-generated anchor-id probe from the current export.",
            }
        )

    usable_room_types = list((export_summary.get("room_type_summary") or {}).get("usable_room_types") or [])
    if usable_room_types:
        cases.append(
            {
                "instruction": f"go to {usable_room_types[0]}",
                "family": "semantic_room_target",
                "start_room_id": start_room_id,
                "source": "export_probe",
                "description": "Auto-generated semantic room-name probe from the current export.",
            }
        )
    return cases


def _load_text_payload(raw_text: str) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    for line in raw_text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        family = None
        instruction = stripped
        for delimiter in ("\t", "|"):
            if delimiter in stripped:
                head, tail = stripped.split(delimiter, 1)
                if head.strip() in FAMILY_DISPLAY_NAMES and tail.strip():
                    family = head.strip()
                    instruction = tail.strip()
                break
        items.append({"instruction": instruction, "family": family})
    return items


def _load_yaml_payload(raw_text: str) -> Any:
    try:
        import yaml  # type: ignore

        return yaml.safe_load(raw_text)
    except ImportError:
        items = []
        for line in raw_text.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if stripped.startswith("- "):
                items.append(stripped[2:].strip())
        return items


def _normalize_case_payload(payload: Any) -> List[Dict[str, Any]]:
    if payload is None:
        return []
    if isinstance(payload, dict):
        items = payload.get("instructions", payload.get("cases", []))
    else:
        items = payload

    normalized: List[Dict[str, Any]] = []
    for item in items or []:
        if isinstance(item, str):
            normalized.append({"instruction": item})
        elif isinstance(item, dict) and item.get("instruction"):
            normalized.append(dict(item))
    return normalized


def _is_usable_room_type(normalized_room_type: Optional[str]) -> bool:
    token = str(normalized_room_type or "").strip()
    return token not in _NON_SEMANTIC_ROOM_TYPES


def _is_explicit_room_token(value: Any) -> bool:
    return bool(_EXPLICIT_ROOM_RE.fullmatch(str(value or "").strip()))


def _family_requires_start_room(family: Optional[str]) -> bool:
    return str(family) in {
        "explicit_room_target",
        "semantic_room_target",
        "object_label_target",
        "anchor_id_target",
    }


def _dedupe_cases(cases: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    seen = set()
    deduped = []
    for item in cases:
        key = (
            str(item.get("instruction")),
            str(item.get("family")),
            str(item.get("start_room_id")),
            str(item.get("source")),
        )
        if key in seen:
            continue
        seen.add(key)
        deduped.append(dict(item))
    return deduped


def _dedupe_text(items: Iterable[str]) -> List[str]:
    seen = set()
    deduped = []
    for item in items:
        text = str(item).strip()
        if not text or text in seen:
            continue
        seen.add(text)
        deduped.append(text)
    return deduped
