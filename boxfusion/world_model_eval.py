from __future__ import annotations

import argparse
import copy
import csv
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_OUTPUT_ROOT = Path("stage_a_outputs_vt_fallback_v01_rerun2")
DEFAULT_REPORT_DIR_NAME = "world_model_backend_eval_v0_2"
DEFAULT_TASK_SHEET = Path("evaluation/world_model_backend/tasks/multifloor_paper_eval_v0_2.json")
TASK_SHEET_VERSION = "0.2"

BOOLEAN_METRICS = (
    "RRA",
    "FCA",
    "TRA",
    "RSR",
    "CTC",
    "VTS",
    "SESR",
    "HOA",
    "FSR",
    "IPS",
    "SRA",
    "NL_E2E",
)

TASK_CSV_FIELDS = [
    "task_id",
    "seq_id",
    "task_family",
    "task_type",
    "start_room",
    "start_frame_idx",
    "end_frame_idx",
    "route_policy",
    "instruction",
    "target_spec",
    "expected_target_room",
    "expected_floor_id",
    "requires_vertical_transition",
    "expected_transition_count",
    "acceptable_transition_sequence",
    "expected_outcome",
    "expected_intent",
    "expected_target_type",
    "expected_slots",
    "subset_tags",
    "notes",
]

RESULT_CSV_FIELDS = [
    "variant_key",
    "variant_label",
    "seq_id",
    "task_id",
    "task_family",
    "task_type",
    "start_room",
    "instruction",
    "route_policy",
    "start_frame_idx",
    "end_frame_idx",
    "requires_vertical_transition",
    "expected_transition_count",
    "subset_tags",
    "expected_outcome",
    "actual_status",
    "expected_target_room",
    "actual_resolved_room",
    "expected_floor_id",
    "actual_resolved_floor_id",
    "route_found",
    "route_hop_count",
    "route_total_cost",
    "route_confidence",
    "actual_transition_count",
    "completed_step_count",
    "total_step_count",
    "execution_success",
    "execution_outcome_category",
    "execution_outcome_reason",
    "expected_intent",
    "actual_intent",
    "expected_target_type",
    "actual_target_type",
    "RRA",
    "FCA",
    "TRA",
    "RSR",
    "CTC",
    "VTS",
    "SESR",
    "HOA",
    "FSR",
    "IPS",
    "SRA",
    "NL_E2E",
    "pass",
]


@dataclass(frozen=True)
class VariantConfig:
    key: str
    label: str
    route_policy: str = "balanced"
    execution_validation: bool = True
    floor_agnostic: bool = False
    remap_vertical_transition: bool = False
    notes: str = ""


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_dump(payload: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _maybe_json_load(value: Any, default: Any) -> Any:
    if value in (None, "", []):
        return copy.deepcopy(default)
    if isinstance(value, (dict, list)):
        return value
    return json.loads(str(value))


def _parse_bool(value: Any) -> Optional[bool]:
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y"}:
        return True
    if text in {"0", "false", "no", "n"}:
        return False
    return None


def _parse_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    return int(value)


def _slugify(value: str) -> str:
    token = "".join(ch.lower() if ch.isalnum() else "_" for ch in str(value))
    while "__" in token:
        token = token.replace("__", "_")
    return token.strip("_")


def _markdown_escape(value: Any) -> str:
    return str(value).replace("|", "\\|")


def _latex_escape(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": "\\textbackslash{}",
        "&": "\\&",
        "%": "\\%",
        "$": "\\$",
        "#": "\\#",
        "_": "\\_",
        "{": "\\{",
        "}": "\\}",
    }
    for src, dst in replacements.items():
        text = text.replace(src, dst)
    return text


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def _write_markdown_table(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str], title: Optional[str] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines: List[str] = []
    if title:
        lines.append(f"# {title}")
        lines.append("")
    header = "| " + " | ".join(fieldnames) + " |"
    divider = "| " + " | ".join("---" for _ in fieldnames) + " |"
    lines.append(header)
    lines.append(divider)
    for row in rows:
        lines.append("| " + " | ".join(_markdown_escape(row.get(field, "")) for field in fieldnames) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_latex_table(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str], caption: str, label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    colspec = "l" * len(fieldnames)
    lines = [
        "\\begin{table}[t]",
        "\\centering",
        f"\\caption{{{_latex_escape(caption)}}}",
        f"\\label{{{_latex_escape(label)}}}",
        f"\\begin{{tabular}}{{{colspec}}}",
        "\\hline",
        " & ".join(_latex_escape(field) for field in fieldnames) + " \\\\",
        "\\hline",
    ]
    for row in rows:
        lines.append(" & ".join(_latex_escape(row.get(field, "")) for field in fieldnames) + " \\\\")
    lines.extend(["\\hline", "\\end{tabular}", "\\end{table}", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def _normalize_task(raw_task: Dict[str, Any]) -> Dict[str, Any]:
    task = dict(raw_task)
    required_fields = [
        "task_id",
        "seq_id",
        "start_room",
        "task_type",
        "target_spec",
        "expected_target_room",
        "expected_floor_id",
        "requires_vertical_transition",
        "expected_outcome",
    ]
    missing = [field for field in required_fields if field not in task]
    if missing:
        raise ValueError(f"Task {task.get('task_id')!r} is missing required fields: {', '.join(missing)}")

    task["task_id"] = str(task["task_id"])
    task["seq_id"] = str(task["seq_id"])
    task["start_room"] = str(task["start_room"])
    task["task_type"] = str(task["task_type"])
    task["task_family"] = str(task.get("task_family") or task["task_type"])
    task["route_policy"] = str(task.get("route_policy") or "balanced")
    task["instruction"] = None if task.get("instruction") in (None, "") else str(task.get("instruction"))
    task["target_spec"] = _maybe_json_load(task.get("target_spec"), {})
    task["expected_target_room"] = None if task.get("expected_target_room") in (None, "") else str(task.get("expected_target_room"))
    task["expected_floor_id"] = None if task.get("expected_floor_id") in (None, "") else str(task.get("expected_floor_id"))
    task["requires_vertical_transition"] = bool(_parse_bool(task.get("requires_vertical_transition")))
    task["expected_transition_count"] = _parse_int(task.get("expected_transition_count"))
    task["acceptable_transition_sequence"] = _maybe_json_load(task.get("acceptable_transition_sequence"), [])
    task["expected_outcome"] = str(task.get("expected_outcome"))
    task["expected_intent"] = None if task.get("expected_intent") in (None, "") else str(task.get("expected_intent"))
    task["expected_target_type"] = None if task.get("expected_target_type") in (None, "") else str(task.get("expected_target_type"))
    task["expected_slots"] = _maybe_json_load(task.get("expected_slots"), {})
    subset_tags = _maybe_json_load(task.get("subset_tags"), [])
    task["subset_tags"] = [str(item) for item in subset_tags if str(item).strip()]
    task["start_frame_idx"] = _parse_int(task.get("start_frame_idx"))
    task["end_frame_idx"] = _parse_int(task.get("end_frame_idx"))
    task["max_offroute_room_changes"] = int(task.get("max_offroute_room_changes", 0))
    task["notes"] = None if task.get("notes") in (None, "") else str(task.get("notes"))
    return task


def load_task_sheet(path: Path) -> Dict[str, Any]:
    path = Path(path)
    if path.suffix.lower() == ".json":
        payload = dict(_read_json(path))
        tasks = [_normalize_task(item) for item in payload.get("tasks", [])]
        return {
            "version": str(payload.get("version") or TASK_SHEET_VERSION),
            "title": payload.get("title") or path.stem,
            "task_sheet_path": str(path),
            "tasks": tasks,
            "metadata": dict(payload.get("metadata") or {}),
        }

    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            tasks = [_normalize_task(dict(row)) for row in reader]
        return {
            "version": TASK_SHEET_VERSION,
            "title": path.stem,
            "task_sheet_path": str(path),
            "tasks": tasks,
            "metadata": {},
        }

    raise ValueError(f"Unsupported task sheet format for {path}")


def write_task_sheet_csv(task_sheet: Dict[str, Any], path: Path) -> None:
    rows: List[Dict[str, Any]] = []
    for task in task_sheet.get("tasks", []):
        row = dict(task)
        row["target_spec"] = json.dumps(row.get("target_spec") or {}, ensure_ascii=True)
        row["acceptable_transition_sequence"] = json.dumps(row.get("acceptable_transition_sequence") or [], ensure_ascii=True)
        row["expected_slots"] = json.dumps(row.get("expected_slots") or {}, ensure_ascii=True)
        row["subset_tags"] = json.dumps(row.get("subset_tags") or [], ensure_ascii=True)
        rows.append(row)
    _write_csv(path, rows, TASK_CSV_FIELDS)


def get_variant_configs(selected_keys: Optional[Iterable[str]] = None) -> List[VariantConfig]:
    variants = [
        VariantConfig(
            key="full_system",
            label="A. Full system",
            route_policy="balanced",
            execution_validation=True,
            notes="Baseline: floor-aware topology, explicit vertical_transition edges, replay-backed execution validation.",
        ),
        VariantConfig(
            key="floor_agnostic",
            label="B. Floor-agnostic / no floor-aware filtering",
            route_policy="balanced",
            execution_validation=True,
            floor_agnostic=True,
            notes="Honest closest variant: collapse exported floor metadata to a single pseudo-floor while preserving graph connectivity.",
        ),
        VariantConfig(
            key="no_explicit_vertical_transition",
            label="C. No explicit vertical_transition requirement",
            route_policy="balanced",
            execution_validation=True,
            remap_vertical_transition=True,
            notes="Honest closest variant: remap vertical_transition edges to generic transition edges while preserving room connectivity.",
        ),
        VariantConfig(
            key="query_only",
            label="D. Query-only (no symbolic execution validation)",
            route_policy="balanced",
            execution_validation=False,
            notes="Execution tasks keep query/planning output but skip replay-backed symbolic validation.",
        ),
        VariantConfig(
            key="policy_strict",
            label="E. Route policy strict",
            route_policy="strict",
            execution_validation=True,
            notes="Policy comparison row using the existing strict Query API preset.",
        ),
        VariantConfig(
            key="policy_balanced",
            label="E. Route policy balanced",
            route_policy="balanced",
            execution_validation=True,
            notes="Policy comparison row using the existing balanced Query API preset.",
        ),
        VariantConfig(
            key="policy_exploratory",
            label="E. Route policy exploratory",
            route_policy="exploratory",
            execution_validation=True,
            notes="Policy comparison row using the existing exploratory Query API preset.",
        ),
    ]
    if selected_keys is None:
        return variants
    selected = {str(item).strip() for item in selected_keys if str(item).strip()}
    return [variant for variant in variants if variant.key in selected]


def transform_topology_payload(payload: Dict[str, Any], variant: VariantConfig) -> Dict[str, Any]:
    transformed = copy.deepcopy(payload)
    if variant.floor_agnostic:
        transformed["floors"] = [
            {
                "floor_id": "floor_agnostic",
                "display_floor_id": "floor_agnostic",
                "display_order": 0,
            }
        ]
        for room in transformed.get("rooms", []):
            room["floor_id"] = "floor_agnostic"
            room["display_floor_id"] = "floor_agnostic"
            room["display_order"] = 0
            room["floor_index"] = 0
        for entity_key in ("objects", "anchors"):
            for record in transformed.get("entities", {}).get(entity_key, []):
                record["floor_id"] = "floor_agnostic"
                record["display_floor_id"] = "floor_agnostic"
                record["display_order"] = 0

    if variant.remap_vertical_transition:
        for edge in transformed.get("edges", []):
            if edge.get("relation_type") == "vertical_transition":
                edge["relation_type"] = "transition"
        for room in transformed.get("rooms", []):
            metadata = room.get("metadata")
            if isinstance(metadata, dict):
                metadata["vertical_transition_variant"] = "remapped_to_transition"
    return transformed


def _load_runtime_context(sequence_root: Path, variant: VariantConfig) -> Dict[str, Any]:
    from boxfusion.query_api import RoomTopologyQueryAPI
    from boxfusion.room_topology import RoomTopology
    from boxfusion.vln_closed_loop import VLNClosedLoopExecutor, build_symbolic_plan, load_replay_observations
    from boxfusion.vln_end_to_end_demo import VLNEndToEndDemoOrchestrator
    from boxfusion.vln_tool_use import MinimalVLNToolUseAdapter

    topology_json = sequence_root / "logs" / "topology_v0_1.json"
    timeline_json = sequence_root / "logs" / "timeline.json"
    if not topology_json.exists():
        raise FileNotFoundError(f"Missing topology export: {topology_json}")
    if not timeline_json.exists():
        raise FileNotFoundError(f"Missing timeline export: {timeline_json}")

    raw_payload = dict(_read_json(topology_json))
    transformed_payload = transform_topology_payload(raw_payload, variant)
    topology = RoomTopology.from_dict(transformed_payload)
    query_api = RoomTopologyQueryAPI(topology)
    observations = load_replay_observations(timeline_json, query_api)
    executor = None
    if variant.execution_validation:
        executor = VLNClosedLoopExecutor(
            query_api=query_api,
            observations=observations,
            topology_json=topology_json,
            timeline_json=timeline_json,
        )
    tool_use_adapter = MinimalVLNToolUseAdapter(query_api=query_api, executor=executor)
    orchestrator = VLNEndToEndDemoOrchestrator(
        query_api=query_api,
        executor=executor,
        tool_use_adapter=tool_use_adapter,
    )
    return {
        "query_api": query_api,
        "executor": executor,
        "tool_use_adapter": tool_use_adapter,
        "orchestrator": orchestrator,
        "build_symbolic_plan": build_symbolic_plan,
        "sequence_root": sequence_root,
        "topology_json": topology_json,
        "timeline_json": timeline_json,
        "variant": variant,
    }


def _extract_route(backend_result: Dict[str, Any]) -> Dict[str, Any]:
    if "route" in backend_result:
        return dict(backend_result.get("route") or {})
    return dict((((backend_result.get("plan") or {}).get("query_result") or {}).get("route")) or {})


def _extract_explanation(backend_result: Dict[str, Any]) -> Dict[str, Any]:
    if "explanation" in backend_result:
        return dict(backend_result.get("explanation") or {})
    return dict((((backend_result.get("plan") or {}).get("query_result") or {}).get("explanation")) or {})


def _extract_target_resolution(backend_result: Dict[str, Any]) -> Dict[str, Any]:
    if "target_resolution" in backend_result:
        return dict(backend_result.get("target_resolution") or {})
    if (backend_result.get("plan") or {}).get("target_resolution") is not None:
        return dict((backend_result.get("plan") or {}).get("target_resolution") or {})
    return {}


def _status_from_failure_reason(failure_reason: Optional[str]) -> str:
    reason = str(failure_reason or "").strip().lower()
    if "ambiguous" in reason:
        return "ambiguous"
    if "unsupported" in reason:
        return "unsupported"
    if "query_only_variant" in reason:
        return "query_only"
    return "unresolved"


def _effective_route_policy(task: Dict[str, Any], variant: VariantConfig) -> str:
    if variant.key.startswith("policy_"):
        return variant.route_policy
    return str(task.get("route_policy") or variant.route_policy)


def _structured_status(task_type: str, backend_result: Dict[str, Any], variant: VariantConfig) -> str:
    if task_type.startswith("resolve_"):
        resolution = _extract_target_resolution(backend_result)
        if resolution.get("resolved"):
            return "success"
        return _status_from_failure_reason(resolution.get("failure_reason"))

    if task_type.startswith("query_"):
        target_resolution = _extract_target_resolution(backend_result)
        route = _extract_route(backend_result)
        if target_resolution and not target_resolution.get("resolved"):
            return _status_from_failure_reason(target_resolution.get("failure_reason"))
        if route.get("found"):
            return "success"
        if route.get("attempted"):
            return "route_not_found"
        return _status_from_failure_reason(route.get("failure_reason") or backend_result.get("failure_reason"))

    if task_type.startswith("execute_"):
        outcome = dict(backend_result.get("outcome") or {})
        plan = dict(backend_result.get("plan") or {})
        target_resolution = dict(plan.get("target_resolution") or {})
        if not variant.execution_validation and outcome.get("outcome_reason") == "query_only_variant":
            return "query_only"
        if outcome.get("success"):
            return "success"
        if target_resolution and not target_resolution.get("resolved"):
            return _status_from_failure_reason(target_resolution.get("failure_reason") or plan.get("planning_reason"))
        failure_reason = outcome.get("outcome_reason") or plan.get("planning_reason")
        return _status_from_failure_reason(failure_reason) if "ambiguous" in str(failure_reason) or "unresolved" in str(failure_reason) else str(outcome.get("outcome_category") or "failure")

    return "unknown"


def _transition_sequence(query_api: Any, route: Dict[str, Any]) -> List[Dict[str, Any]]:
    sequence: List[Dict[str, Any]] = []
    for edge in route.get("edges", []):
        source_room = query_api.topology.get_room(edge.get("source_room_id")) or {}
        target_room = query_api.topology.get_room(edge.get("target_room_id")) or {}
        source_floor = source_room.get("floor_id")
        target_floor = target_room.get("floor_id")
        if edge.get("relation_type") != "vertical_transition" and source_floor == target_floor:
            continue
        transition_ids = list((edge.get("metadata") or {}).get("transition_ids", []))
        sequence.append(
            {
                "transition_id": transition_ids[0] if transition_ids else None,
                "transition_ids": transition_ids,
                "from_room": edge.get("source_room_id"),
                "to_room": edge.get("target_room_id"),
                "from_floor_id": source_floor,
                "to_floor_id": target_floor,
                "relation_type": edge.get("relation_type"),
            }
        )
    return sequence


def _slot_dict_matches(expected: Dict[str, Any], actual: Dict[str, Any]) -> bool:
    for key, expected_value in expected.items():
        if isinstance(expected_value, dict):
            if not _slot_dict_matches(expected_value, dict(actual.get(key) or {})):
                return False
            continue
        if expected_value is None:
            continue
        if actual.get(key) != expected_value:
            return False
    return True


def _compare_transition_sequence(expected: List[Dict[str, Any]], actual: List[Dict[str, Any]]) -> bool:
    if not expected:
        return not actual
    if len(expected) != len(actual):
        return False
    for expected_step, actual_step in zip(expected, actual):
        for key in ("transition_id", "from_room", "to_room", "from_floor_id", "to_floor_id"):
            expected_value = expected_step.get(key)
            if expected_value in (None, ""):
                continue
            if actual_step.get(key) != expected_value:
                return False
    return True


def _build_metric_flags(task: Dict[str, Any], result: Dict[str, Any], variant: VariantConfig) -> Dict[str, Optional[bool]]:
    expected_target_room = task.get("expected_target_room")
    expected_floor_id = task.get("expected_floor_id")
    actual_room = result.get("actual_resolved_room")
    actual_floor = result.get("actual_resolved_floor_id")
    route_found = bool(result.get("route_found"))
    transition_count = int(result.get("actual_transition_count") or 0)
    actual_sequence = list(result.get("actual_transition_sequence") or [])
    actual_status = str(result.get("actual_status"))
    expected_status = str(task.get("expected_outcome"))
    execution_success = result.get("execution_success")

    rra = None
    if expected_target_room is not None:
        rra = actual_room == expected_target_room

    fca = None
    if expected_floor_id is not None:
        fca = actual_floor == expected_floor_id

    tra = None
    if expected_target_room is not None:
        tra = bool(rra) and (expected_floor_id is None or bool(fca))
    elif expected_floor_id is not None:
        tra = bool(fca)

    rsr = None
    if task["task_type"].startswith(("query_", "execute_")) and expected_target_room is not None and expected_status == "success":
        rsr = route_found and actual_room == expected_target_room

    ctc = None
    if task.get("requires_vertical_transition") or task.get("expected_transition_count") is not None:
        count_ok = True if task.get("expected_transition_count") is None else transition_count == int(task["expected_transition_count"])
        seq_ok = True if not task.get("acceptable_transition_sequence") else _compare_transition_sequence(
            list(task.get("acceptable_transition_sequence") or []),
            actual_sequence,
        )
        ctc = count_ok and seq_ok and (transition_count > 0 if task.get("requires_vertical_transition") else True)

    vts = None
    if expected_status == "success" and route_found and transition_count > 0:
        vts = all(step.get("relation_type") == "vertical_transition" for step in actual_sequence)

    sesr = None
    if task["task_type"].startswith("execute_") and variant.execution_validation and expected_status == "success":
        sesr = bool(execution_success)

    hoa = None
    if task["task_type"].startswith("execute_") and variant.execution_validation:
        hoa = actual_status == expected_status

    fsr = None
    if expected_status != "success":
        fsr = actual_status == "success"

    ips = None
    if task["task_type"] == "nl_request" and task.get("expected_intent") is not None:
        ips = result.get("actual_intent") == task.get("expected_intent")

    sra = None
    if task["task_type"] == "nl_request" and task.get("expected_intent") is not None:
        target_type_ok = (
            task.get("expected_target_type") is None
            or result.get("actual_target_type") == task.get("expected_target_type")
        )
        room_ok = True if expected_target_room is None else actual_room == expected_target_room
        slot_ok = True if not task.get("expected_slots") else _slot_dict_matches(
            dict(task.get("expected_slots") or {}),
            dict(result.get("actual_slots") or {}),
        )
        sra = target_type_ok and room_ok and slot_ok

    nl_e2e = None
    if task["task_type"] == "nl_request" and task.get("expected_intent") is not None:
        nl_e2e = (
            bool(ips)
            and bool(sra)
            and actual_status == expected_status
            and (True if expected_target_room is None else actual_room == expected_target_room)
        )

    return {
        "RRA": rra,
        "FCA": fca,
        "TRA": tra,
        "RSR": rsr,
        "CTC": ctc,
        "VTS": vts,
        "SESR": sesr,
        "HOA": hoa,
        "FSR": fsr,
        "IPS": ips,
        "SRA": sra,
        "NL_E2E": nl_e2e,
    }


def _normalize_result(task: Dict[str, Any], raw_result: Dict[str, Any], context: Dict[str, Any], variant: VariantConfig) -> Dict[str, Any]:
    query_api = context["query_api"]
    task_type = task["task_type"]
    status: str
    execution_success: Optional[bool] = None
    execution_outcome_category: Optional[str] = None
    execution_outcome_reason: Optional[str] = None
    actual_intent: Optional[str] = None
    actual_target_type: Optional[str] = None
    actual_slots: Dict[str, Any] = {}
    completed_step_count: Optional[int] = None
    total_step_count: Optional[int] = None

    if task_type == "nl_request":
        final_outcome = dict(raw_result.get("final_outcome") or {})
        status = str(final_outcome.get("status") or "unknown")
        execution_success = final_outcome.get("success")
        execution_outcome_category = final_outcome.get("outcome_category")
        execution_outcome_reason = final_outcome.get("outcome_reason")
        parsed_request = dict(raw_result.get("interpreted_request") or {})
        actual_intent = parsed_request.get("request_type")
        actual_target_type = raw_result.get("target_type")
        actual_slots = {
            "target_text": parsed_request.get("target_text"),
            "target_type": parsed_request.get("target_type"),
            "floor_hint": parsed_request.get("floor_hint"),
        }
        route = dict((raw_result.get("route_summary") or {}))
        nl_backend_result = dict(
            (((raw_result.get("artifacts") or {}).get("tool_use_result") or {}).get("backend_result") or {})
        )
        nl_outcome = dict(nl_backend_result.get("outcome") or {})
        completed_step_count = nl_outcome.get("completed_step_count")
        total_step_count = nl_outcome.get("total_step_count")
        actual_transition_sequence = _transition_sequence(
            query_api,
            {"edges": list((nl_backend_result.get("route") or {}).get("edges", []))},
        )
        actual_resolved = dict(raw_result.get("resolved_target") or {})
    else:
        status = _structured_status(task_type, raw_result, variant)
        route = _extract_route(raw_result)
        actual_transition_sequence = _transition_sequence(query_api, route)
        actual_resolved = _extract_target_resolution(raw_result)
        outcome = dict(raw_result.get("outcome") or {})
        execution_success = outcome.get("success")
        execution_outcome_category = outcome.get("outcome_category")
        execution_outcome_reason = outcome.get("outcome_reason")
        completed_step_count = outcome.get("completed_step_count")
        total_step_count = outcome.get("total_step_count")

    normalized = {
        "generated_at_utc": _utc_now_iso(),
        "variant_key": variant.key,
        "variant_label": variant.label,
        "variant_notes": variant.notes,
        "seq_id": task["seq_id"],
        "task_id": task["task_id"],
        "task_family": task["task_family"],
        "task_type": task["task_type"],
        "start_room": task["start_room"],
        "instruction": task.get("instruction"),
        "route_policy": _effective_route_policy(task, variant),
        "start_frame_idx": task.get("start_frame_idx"),
        "end_frame_idx": task.get("end_frame_idx"),
        "requires_vertical_transition": task.get("requires_vertical_transition"),
        "expected_transition_count": task.get("expected_transition_count"),
        "subset_tags": list(task.get("subset_tags") or []),
        "expected_outcome": task["expected_outcome"],
        "actual_status": status,
        "expected_target_room": task.get("expected_target_room"),
        "actual_resolved_room": actual_resolved.get("resolved_room_id"),
        "expected_floor_id": task.get("expected_floor_id"),
        "actual_resolved_floor_id": actual_resolved.get("resolved_floor_id"),
        "route_found": bool(route.get("found")) if route else False,
        "route_hop_count": route.get("hop_count") if route else None,
        "route_total_cost": route.get("total_cost") if route else None,
        "route_confidence": route.get("route_confidence") if route else None,
        "actual_transition_count": len(actual_transition_sequence),
        "actual_transition_sequence": actual_transition_sequence,
        "completed_step_count": completed_step_count,
        "total_step_count": total_step_count,
        "execution_success": execution_success,
        "execution_outcome_category": execution_outcome_category,
        "execution_outcome_reason": execution_outcome_reason,
        "expected_intent": task.get("expected_intent"),
        "actual_intent": actual_intent,
        "expected_target_type": task.get("expected_target_type"),
        "actual_target_type": actual_target_type,
        "actual_slots": actual_slots,
        "raw_result": raw_result,
        "resolved_target": actual_resolved,
        "pass": False,
    }
    metric_flags = _build_metric_flags(task, normalized, variant)
    normalized.update(metric_flags)
    applicable_checks = [value for value in metric_flags.values() if value is not None]
    normalized["pass"] = normalized["actual_status"] == task["expected_outcome"] and all(applicable_checks)
    return normalized


def _run_structured_task(task: Dict[str, Any], context: Dict[str, Any], variant: VariantConfig) -> Dict[str, Any]:
    query_api = context["query_api"]
    executor = context["executor"]
    build_symbolic_plan = context["build_symbolic_plan"]
    route_policy = _effective_route_policy(task, variant)
    target = dict(task.get("target_spec") or {})
    task_type = task["task_type"]

    if task_type == "query_room_route":
        return query_api.query_route(task["start_room"], target.get("goal_room_id"), route_policy=route_policy)
    if task_type == "query_object_route":
        kwargs = {"start_room_id": task["start_room"], "route_policy": route_policy}
        if target.get("object_id") is not None:
            kwargs["object_id"] = target.get("object_id")
        if target.get("object_label") is not None:
            kwargs["object_label"] = target.get("object_label")
        return query_api.query_route_to_object(**kwargs)
    if task_type == "query_anchor_route":
        return query_api.query_route_to_anchor(task["start_room"], target.get("anchor_id"), route_policy=route_policy)
    if task_type == "resolve_room":
        return {"target_resolution": query_api.resolve_room_target(target.get("goal_room_id"))}
    if task_type == "resolve_object":
        return {
            "target_resolution": query_api.resolve_object_room(
                object_id=target.get("object_id"),
                object_label=target.get("object_label"),
            )
        }
    if task_type == "resolve_anchor":
        return {"target_resolution": query_api.resolve_anchor_room(target.get("anchor_id"))}

    task_input = {"start_room_id": task["start_room"], "target": target}
    if task_type.startswith("execute_"):
        if executor is None:
            plan = build_symbolic_plan(query_api, task_input=task_input, route_policy=route_policy)
            return {
                "version": "0.1",
                "task_input": task_input,
                "route_policy": route_policy,
                "plan": plan,
                "execution_trace": [],
                "outcome": {
                    "outcome_category": "not_run",
                    "outcome_reason": "query_only_variant",
                    "success": False,
                    "completed_step_count": 0,
                    "total_step_count": len((((plan.get("symbolic_plan") or {}).get("steps")) or [])),
                },
            }
        return executor.execute(
            task_input=task_input,
            route_policy=route_policy,
            start_frame_idx=task.get("start_frame_idx"),
            end_frame_idx=task.get("end_frame_idx"),
            max_offroute_room_changes=task.get("max_offroute_room_changes", 0),
        )

    raise ValueError(f"Unsupported task_type {task_type}")


def run_task(task: Dict[str, Any], context: Dict[str, Any], variant: VariantConfig) -> Dict[str, Any]:
    if task["task_type"] == "nl_request":
        raw_result = context["orchestrator"].run_nl_request(
            task.get("instruction") or "",
            sequence_id=task["seq_id"],
            start_room_id=task["start_room"],
            route_policy=_effective_route_policy(task, variant),
            start_frame_idx=task.get("start_frame_idx"),
            end_frame_idx=task.get("end_frame_idx"),
            max_offroute_room_changes=task.get("max_offroute_room_changes", 0),
            case_id=f"{variant.key}_{task['task_id']}",
        )
    else:
        raw_result = _run_structured_task(task, context, variant)
    return _normalize_result(task, raw_result, context, variant)


def _metric_rate(results: Sequence[Dict[str, Any]], metric_name: str) -> Tuple[int, int, Optional[float]]:
    applicable = [item for item in results if item.get(metric_name) is not None]
    if not applicable:
        return (0, 0, None)
    numerator = sum(1 for item in applicable if bool(item.get(metric_name)))
    denominator = len(applicable)
    return numerator, denominator, numerator / denominator if denominator else None


def _format_rate(rate: Optional[float]) -> str:
    if rate is None:
        return "n/a"
    return f"{rate * 100:.1f}"


def _format_average(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    return f"{value:.2f}"


def _average_numeric(results: Sequence[Dict[str, Any]], field_name: str, *, predicate: Optional[Any] = None) -> Optional[float]:
    values: List[float] = []
    for item in results:
        if predicate is not None and not predicate(item):
            continue
        value = item.get(field_name)
        if value is None:
            continue
        values.append(float(value))
    if not values:
        return None
    return sum(values) / len(values)


def build_metric_row(label: str, results: Sequence[Dict[str, Any]], note: Optional[str] = None, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    row = {"label": label}
    for metric_name in BOOLEAN_METRICS:
        numerator, denominator, rate = _metric_rate(results, metric_name)
        row[metric_name] = _format_rate(rate)
        row[f"{metric_name}_raw"] = f"{numerator}/{denominator}" if denominator else "n/a"
    row["avg_route_hops"] = _format_average(_average_numeric(results, "route_hop_count", predicate=lambda item: item.get("route_found")))
    row["avg_route_cost"] = _format_average(_average_numeric(results, "route_total_cost", predicate=lambda item: item.get("route_found")))
    row["avg_transition_count"] = _format_average(_average_numeric(results, "actual_transition_count", predicate=lambda item: item.get("route_found")))
    row["task_count"] = len(results)
    row["pass_count"] = sum(1 for item in results if item.get("pass"))
    row["note"] = note or ""
    if extra:
        row.update(extra)
    return row


def _sequence_groups(results: Sequence[Dict[str, Any]]) -> Dict[Tuple[str, str], List[Dict[str, Any]]]:
    groups: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for item in results:
        groups.setdefault((str(item["variant_key"]), str(item["seq_id"])), []).append(item)
    return groups


def build_inventory_rows(tasks: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    counts: Dict[Tuple[str, str, str], int] = {}
    for task in tasks:
        key = (task["seq_id"], task["task_family"], task["task_type"])
        counts[key] = counts.get(key, 0) + 1
    rows = [
        {
            "seq_id": seq_id,
            "task_family": task_family,
            "task_type": task_type,
            "count": count,
        }
        for (seq_id, task_family, task_type), count in sorted(counts.items())
    ]
    rows.append(
        {
            "seq_id": "ALL",
            "task_family": "ALL",
            "task_type": "ALL",
            "count": len(tasks),
        }
    )
    return rows


def build_main_result_rows(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for item in results:
        if item["task_type"] == "nl_request":
            continue
        rows.append(
            {
                "task_id": item["task_id"],
                "seq_id": item["seq_id"],
                "task_family": item["task_family"],
                "task_type": item["task_type"],
                "start_room": item["start_room"],
                "expected_target_room": item["expected_target_room"],
                "actual_resolved_room": item["actual_resolved_room"],
                "expected_floor_id": item["expected_floor_id"],
                "actual_resolved_floor_id": item["actual_resolved_floor_id"],
                "route_found": item["route_found"],
                "route_hop_count": item["route_hop_count"],
                "actual_transition_count": item["actual_transition_count"],
                "execution_outcome_category": item["execution_outcome_category"] or "",
                "expected_outcome": item["expected_outcome"],
                "actual_status": item["actual_status"],
                "RRA": item["RRA"],
                "FCA": item["FCA"],
                "TRA": item["TRA"],
                "RSR": item["RSR"],
                "CTC": item["CTC"],
                "VTS": item["VTS"],
                "SESR": item["SESR"],
                "HOA": item["HOA"],
            }
        )
    return rows


def build_nl_result_rows(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for item in results:
        if item["task_type"] != "nl_request":
            continue
        rows.append(
            {
                "task_id": item["task_id"],
                "seq_id": item["seq_id"],
                "instruction": item["instruction"],
                "expected_intent": item["expected_intent"] or "",
                "actual_intent": item["actual_intent"] or "",
                "expected_target_room": item["expected_target_room"] or "",
                "actual_resolved_room": item["actual_resolved_room"] or "",
                "expected_outcome": item["expected_outcome"],
                "actual_status": item["actual_status"],
                "IPS": item["IPS"],
                "SRA": item["SRA"],
                "NL_E2E": item["NL_E2E"],
            }
        )
    return rows


def build_execution_rows(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        {
            "task_id": item["task_id"],
            "seq_id": item["seq_id"],
            "variant_key": item["variant_key"],
            "task_type": item["task_type"],
            "start_frame_idx": item.get("start_frame_idx") or "",
            "end_frame_idx": item.get("end_frame_idx") or "",
            "expected_outcome": item["expected_outcome"],
            "actual_status": item["actual_status"],
            "execution_outcome_category": item["execution_outcome_category"] or "",
            "execution_outcome_reason": item["execution_outcome_reason"] or "",
            "completed_step_count": item.get("completed_step_count"),
            "total_step_count": item.get("total_step_count"),
            "SESR": item["SESR"],
            "HOA": item["HOA"],
            "FSR": item["FSR"],
        }
        for item in results
        if item["task_type"].startswith("execute_")
    ]


def build_execution_summary_rows(results: Sequence[Dict[str, Any]], variant_lookup: Dict[str, VariantConfig]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for variant_key, variant in sorted(variant_lookup.items()):
        items = [item for item in results if item["variant_key"] == variant_key and item["task_type"].startswith("execute_")]
        if not items:
            continue
        actual_status_counts: Dict[str, int] = {}
        outcome_counts: Dict[str, int] = {}
        for item in items:
            status = str(item.get("actual_status") or "unknown")
            actual_status_counts[status] = actual_status_counts.get(status, 0) + 1
            category = str(item.get("execution_outcome_category") or "none")
            outcome_counts[category] = outcome_counts.get(category, 0) + 1
        row = build_metric_row(
            label=variant.label,
            results=items,
            note=variant.notes,
            extra={
                "variant_key": variant_key,
                "success_count": actual_status_counts.get("success", 0),
                "failure_count": actual_status_counts.get("failure", 0),
                "replan_needed_count": actual_status_counts.get("replan_needed", 0),
                "ambiguous_count": actual_status_counts.get("ambiguous", 0),
                "unsupported_count": actual_status_counts.get("unsupported", 0),
                "unresolved_count": actual_status_counts.get("unresolved", 0),
                "query_only_count": actual_status_counts.get("query_only", 0),
                "not_run_count": outcome_counts.get("not_run", 0),
            },
        )
        rows.append(row)
    return rows


def _subset_rows(
    results: Sequence[Dict[str, Any]],
    variant_lookup: Dict[str, VariantConfig],
    subset_specs: Sequence[Tuple[str, str, Any]],
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for variant_key, variant in sorted(variant_lookup.items()):
        variant_results = [item for item in results if item["variant_key"] == variant_key]
        for subset_key, subset_label, predicate in subset_specs:
            subset_results = [item for item in variant_results if predicate(item)]
            if not subset_results:
                continue
            row = build_metric_row(
                label=f"{variant.label} / {subset_label}",
                results=subset_results,
                note=variant.notes,
                extra={
                    "variant_key": variant_key,
                    "variant_label": variant.label,
                    "subset_key": subset_key,
                    "subset_label": subset_label,
                },
            )
            rows.append(row)
    return rows


def build_policy_comparison_rows(results: Sequence[Dict[str, Any]], variant_lookup: Dict[str, VariantConfig]) -> List[Dict[str, Any]]:
    policy_keys = [key for key in ("policy_strict", "policy_balanced", "policy_exploratory") if key in variant_lookup]
    rows: List[Dict[str, Any]] = []
    for variant_key in policy_keys:
        variant = variant_lookup[variant_key]
        items = [
            item
            for item in results
            if item["variant_key"] == variant_key
            and item["task_type"] == "query_room_route"
            and "policy_sensitive" in set(item.get("subset_tags") or [])
        ]
        if not items:
            continue
        row = build_metric_row(
            label=variant.label,
            results=items,
            note=variant.notes,
            extra={
                "variant_key": variant_key,
                "route_policy": variant.route_policy,
            },
        )
        rows.append(row)
    return rows


def build_policy_sensitive_detail_rows(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    relevant = [
        item
        for item in results
        if item["variant_key"] in {"policy_strict", "policy_balanced", "policy_exploratory"}
        and item["task_type"] == "query_room_route"
        and "policy_sensitive" in set(item.get("subset_tags") or [])
    ]
    grouped: Dict[str, Dict[str, Dict[str, Any]]] = {}
    for item in relevant:
        grouped.setdefault(item["task_id"], {})[item["variant_key"]] = item

    rows: List[Dict[str, Any]] = []
    for task_id, payload in sorted(grouped.items()):
        exemplar = next(iter(payload.values()))
        rows.append(
            {
                "task_id": task_id,
                "seq_id": exemplar["seq_id"],
                "expected_target_room": exemplar["expected_target_room"],
                "strict_hops": (payload.get("policy_strict") or {}).get("route_hop_count"),
                "balanced_hops": (payload.get("policy_balanced") or {}).get("route_hop_count"),
                "exploratory_hops": (payload.get("policy_exploratory") or {}).get("route_hop_count"),
                "strict_cost": (payload.get("policy_strict") or {}).get("route_total_cost"),
                "balanced_cost": (payload.get("policy_balanced") or {}).get("route_total_cost"),
                "exploratory_cost": (payload.get("policy_exploratory") or {}).get("route_total_cost"),
            }
        )
    return rows


def write_sequence_reports(report_dir: Path, results: Sequence[Dict[str, Any]], variant_lookup: Dict[str, VariantConfig]) -> None:
    groups = _sequence_groups(results)
    for (variant_key, seq_id), items in sorted(groups.items()):
        path = report_dir / "reports" / "per_sequence" / f"{seq_id}_{variant_key}.md"
        metric_row = build_metric_row(
            label=f"{seq_id} / {variant_key}",
            results=items,
            note=variant_lookup[variant_key].notes,
        )
        lines = [
            f"# Sequence Breakdown: {seq_id} / {variant_lookup[variant_key].label}",
            "",
            f"- Task count: {len(items)}",
            f"- Pass count: {metric_row['pass_count']}",
            f"- RRA: {metric_row['RRA']} ({metric_row['RRA_raw']})",
            f"- FCA: {metric_row['FCA']} ({metric_row['FCA_raw']})",
            f"- TRA: {metric_row['TRA']} ({metric_row['TRA_raw']})",
            f"- RSR: {metric_row['RSR']} ({metric_row['RSR_raw']})",
            f"- CTC: {metric_row['CTC']} ({metric_row['CTC_raw']})",
            f"- VTS: {metric_row['VTS']} ({metric_row['VTS_raw']})",
            f"- SESR: {metric_row['SESR']} ({metric_row['SESR_raw']})",
            f"- HOA: {metric_row['HOA']} ({metric_row['HOA_raw']})",
            f"- IPS: {metric_row['IPS']} ({metric_row['IPS_raw']})",
            f"- SRA: {metric_row['SRA']} ({metric_row['SRA_raw']})",
            f"- NL-E2E: {metric_row['NL_E2E']} ({metric_row['NL_E2E_raw']})",
            "",
            "## Tasks",
            "",
        ]
        task_rows = sorted(items, key=lambda row: (row["task_type"], row["task_id"]))
        header = "| task_id | task_type | expected_outcome | actual_status | expected_target_room | actual_resolved_room | pass |"
        divider = "| --- | --- | --- | --- | --- | --- | --- |"
        lines.extend([header, divider])
        for item in task_rows:
            lines.append(
                "| "
                + " | ".join(
                    _markdown_escape(value)
                    for value in (
                        item["task_id"],
                        item["task_type"],
                        item["expected_outcome"],
                        item["actual_status"],
                        item["expected_target_room"] or "",
                        item["actual_resolved_room"] or "",
                        item["pass"],
                    )
                )
                + " |"
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_per_sequence_tables(report_dir: Path, results: Sequence[Dict[str, Any]]) -> None:
    full_results = [item for item in results if item["variant_key"] == "full_system"]
    for seq_id in sorted({item["seq_id"] for item in full_results}):
        rows = build_main_result_rows([item for item in full_results if item["seq_id"] == seq_id])
        fields = [
            "task_id",
            "task_family",
            "task_type",
            "start_room",
            "expected_target_room",
            "actual_resolved_room",
            "expected_floor_id",
            "actual_resolved_floor_id",
            "route_found",
            "route_hop_count",
            "actual_transition_count",
            "expected_outcome",
            "actual_status",
            "RRA",
            "FCA",
            "TRA",
            "CTC",
            "VTS",
            "SESR",
            "HOA",
        ]
        base = report_dir / "tables" / "per_sequence" / f"{seq_id}_full_system_results"
        _write_csv(base.with_suffix(".csv"), rows, fields)
        _write_markdown_table(base.with_suffix(".md"), rows, fields, title=f"Per-Sequence Results: {seq_id}")


def write_diagnosis_reports(
    report_dir: Path,
    *,
    task_sheet: Dict[str, Any],
    ablation_rows: Sequence[Dict[str, Any]],
    execution_summary_rows: Sequence[Dict[str, Any]],
    policy_rows: Sequence[Dict[str, Any]],
) -> None:
    full_row = next((row for row in ablation_rows if row.get("variant_key") == "full_system"), None)
    floor_row = next((row for row in ablation_rows if row.get("variant_key") == "floor_agnostic"), None)
    vt_row = next((row for row in ablation_rows if row.get("variant_key") == "no_explicit_vertical_transition"), None)

    execution_full = next((row for row in execution_summary_rows if row.get("variant_key") == "full_system"), None)
    diagnosis_lines = [
        "# Paper-Facing Diagnosis",
        "",
        "## v0.1 Audit",
        "",
        "- Strong enough to keep: the full-system structured backend rows already establish that the room-centric queryable backend works end to end on both sequences.",
        "- Strong enough to keep: the floor-aware vs floor-agnostic comparison is directionally useful, because it exposes a real cross-floor dependence rather than a cosmetic ablation.",
        "- Saturated / weak in v0.1: the main structured table was nearly all-correct, so it read more like a sanity check than a paper claim.",
        "- Saturated / weak in v0.1: the no-explicit-vertical-transition row matched full-system connectivity, so it did not isolate edge typing semantics.",
        "- Saturated / weak in v0.1: the strict/balanced/exploratory policy rows were identical on correctness, so they were not main-paper material.",
        "- Saturated / weak in v0.1: symbolic execution only showed success cases and query-only omission, without enough honest replay-window stress.",
        "",
        "## v0.2 Main-Paper Recommendation",
        "",
        "- Keep the full-system structured results, but emphasize decomposed target metrics rather than only the combined TRA score.",
        "- Keep the floor-aware ablation with room accuracy, floor accuracy, combined TRA, and cross-floor subset slices.",
        "- Keep a compact symbolic honest-outcome summary now that success, failure, replan-needed, and planning ambiguity are all represented by replay-backed or planner-backed cases.",
        "- Keep a compact constrained NL table that includes supported, ambiguous, and unsupported requests instead of only easy success cases.",
        "",
        "## Appendix Recommendation",
        "",
        "- Move detailed per-task main tables, per-sequence tables, and raw aggregate CSV/JSON to the appendix or supplement.",
        "- Keep the no-explicit-vertical-transition row as appendix-first unless the paper explicitly discusses typed vertical-transition semantics; its main value is semantic trace preservation, not connectivity correctness.",
        "- Keep route-policy comparison as appendix-first even in v0.2; the informative signal is route efficiency on policy-sensitive same-floor tasks, not correctness.",
    ]
    if full_row is not None and floor_row is not None:
        diagnosis_lines.extend(
            [
                "",
                "## v0.2 Snapshot",
                "",
                f"- Full-system RRA/FCA/TRA: {full_row['RRA']} / {full_row['FCA']} / {full_row['TRA']}.",
                f"- Floor-agnostic RRA/FCA/TRA: {floor_row['RRA']} / {floor_row['FCA']} / {floor_row['TRA']}.",
            ]
        )
    if vt_row is not None:
        diagnosis_lines.append(f"- No-explicit-vertical-transition VTS: {vt_row['VTS']} ({vt_row['VTS_raw']}).")
    if execution_full is not None:
        diagnosis_lines.append(
            f"- Full-system execution outcomes: success={execution_full['success_count']}, failure={execution_full['failure_count']}, replan_needed={execution_full['replan_needed_count']}, ambiguous={execution_full['ambiguous_count']}."
        )
    diagnosis_path = report_dir / "reports" / "paper_facing_diagnosis.md"
    diagnosis_path.parent.mkdir(parents=True, exist_ok=True)
    diagnosis_path.write_text("\n".join(diagnosis_lines) + "\n", encoding="utf-8")

    appendix_lines = [
        "# Main-Paper vs Appendix Recommendation",
        "",
        "## Main Paper",
        "",
        "- Main structured backend summary with decomposed room/floor/combined target metrics.",
        "- Floor-aware ablation with same-floor vs cross-floor and transition-count subset slices.",
        "- Symbolic execution honest-outcome summary with explicit failure / replan-needed counts.",
        "- Constrained NL entry summary with success, ambiguity, and unsupported coverage.",
        "",
        "## Appendix",
        "",
        "- Per-sequence full-system tables and per-task JSON traces.",
        "- Detailed subset tables and transition-count slices.",
        "- Explicit vertical-transition semantics row if the paper does not foreground edge typing.",
        "- Policy comparison tables, unless route-policy efficiency becomes a named secondary claim.",
    ]
    if policy_rows:
        appendix_lines.extend(
            [
                "",
                "## Policy Appendix Note",
                "",
                "- v0.2 now contains policy-sensitive same-floor tasks. The meaningful signal is average route hops / cost, not pass-fail correctness.",
            ]
        )
    appendix_path = report_dir / "reports" / "main_paper_vs_appendix_recommendation.md"
    appendix_path.write_text("\n".join(appendix_lines) + "\n", encoding="utf-8")


def write_summary_report(
    report_dir: Path,
    *,
    task_sheet: Dict[str, Any],
    output_root: Path,
    results: Sequence[Dict[str, Any]],
    ablation_rows: Sequence[Dict[str, Any]],
    main_rows: Sequence[Dict[str, Any]],
) -> None:
    full_results = [item for item in results if item["variant_key"] == "full_system"]
    full_metric_row = build_metric_row("full_system", full_results)
    lines = [
        "# Floor-Aware Queryable World Model Evaluation Summary",
        "",
        f"- Generated at (UTC): {_utc_now_iso()}",
        f"- Output root: {output_root}",
        f"- Task sheet: {task_sheet.get('task_sheet_path')}",
        f"- Task count: {len(task_sheet.get('tasks', []))}",
        "",
        "## Scope",
        "",
        "- Evaluates the BoxFusion-derived world-model backend and interface layers, not an open-ended embodied navigation agent benchmark.",
        "- Preserves the existing architecture: world graph as truth layer, room-centric topology as derived routing layer, NetworkX-backed graph search through the current Query API, and the current closed-loop executor.",
        "- Uses room-level destinations for object and anchor targets.",
        "",
        "## Full-System Metrics",
        "",
        f"- RRA: {full_metric_row['RRA']} ({full_metric_row['RRA_raw']})",
        f"- FCA: {full_metric_row['FCA']} ({full_metric_row['FCA_raw']})",
        f"- TRA: {full_metric_row['TRA']} ({full_metric_row['TRA_raw']})",
        f"- RSR: {full_metric_row['RSR']} ({full_metric_row['RSR_raw']})",
        f"- CTC: {full_metric_row['CTC']} ({full_metric_row['CTC_raw']})",
        f"- VTS: {full_metric_row['VTS']} ({full_metric_row['VTS_raw']})",
        f"- SESR: {full_metric_row['SESR']} ({full_metric_row['SESR_raw']})",
        f"- HOA: {full_metric_row['HOA']} ({full_metric_row['HOA_raw']})",
        f"- FSR: {full_metric_row['FSR']} ({full_metric_row['FSR_raw']})",
        f"- IPS: {full_metric_row['IPS']} ({full_metric_row['IPS_raw']})",
        f"- SRA: {full_metric_row['SRA']} ({full_metric_row['SRA_raw']})",
        f"- NL-E2E: {full_metric_row['NL_E2E']} ({full_metric_row['NL_E2E_raw']})",
        "",
        "## Honest Limitations",
        "",
        "- Object duplication and merge issues can still affect label-based routing and resolve-only tasks.",
        "- Broad room semantic labels remain weak; explicit room ids are still the strongest room references in current exports.",
        "- Object and anchor destinations remain room-level, not precise docking targets.",
        "- Vertical-transition modeling remains minimal and only as explicit as the exported cross-floor edges.",
        "- The NL layer remains constrained to intent/slot extraction, tool selection, and response organization.",
        "",
        "## Artifacts",
        "",
        f"- Main structured results table rows: {len(main_rows)}",
        f"- Ablation rows: {len(ablation_rows)}",
        "- Per-task JSON traces are under `per_task/<variant>/`.",
        "- Per-sequence reports are under `reports/per_sequence/`.",
    ]
    path = report_dir / "reports" / "evaluation_summary.md"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def flatten_result_rows(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for item in results:
        row = {field: item.get(field) for field in RESULT_CSV_FIELDS if field in item}
        for metric_name in BOOLEAN_METRICS:
            row[metric_name] = item.get(metric_name)
        rows.append(row)
    return rows


def run_evaluation(
    *,
    task_sheet_path: Path,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    report_dir_name: str = DEFAULT_REPORT_DIR_NAME,
    variant_keys: Optional[Iterable[str]] = None,
    write_latex: bool = True,
) -> Dict[str, Any]:
    task_sheet = load_task_sheet(task_sheet_path)
    variants = get_variant_configs(variant_keys)
    variant_lookup = {variant.key: variant for variant in variants}
    report_dir = Path(output_root) / report_dir_name
    report_dir.mkdir(parents=True, exist_ok=True)

    _json_dump(task_sheet, report_dir / "task_sheet_used.json")
    write_task_sheet_csv(task_sheet, report_dir / "task_sheet_used.csv")

    runtime_cache: Dict[Tuple[str, str], Dict[str, Any]] = {}
    results: List[Dict[str, Any]] = []
    for variant in variants:
        for task in task_sheet.get("tasks", []):
            cache_key = (task["seq_id"], variant.key)
            if cache_key not in runtime_cache:
                runtime_cache[cache_key] = _load_runtime_context(Path(output_root) / task["seq_id"], variant)
            result = run_task(task, runtime_cache[cache_key], variant)
            results.append(result)
            _json_dump(result, report_dir / "per_task" / variant.key / f"{task['task_id']}.json")

    full_results = [item for item in results if item["variant_key"] == "full_system"]
    inventory_rows = build_inventory_rows(task_sheet.get("tasks", []))
    main_rows = build_main_result_rows(full_results)
    nl_rows = build_nl_result_rows(full_results)
    execution_rows = build_execution_rows(results)
    execution_summary_rows = build_execution_summary_rows(results, variant_lookup)
    ablation_rows = [
        build_metric_row(
            label=variant.label,
            results=[item for item in results if item["variant_key"] == variant.key],
            note=variant.notes,
            extra={
                "variant_key": variant.key,
                "route_policy": variant.route_policy,
                "execution_validation": variant.execution_validation,
            },
        )
        for variant in variants
    ]

    floor_scope_rows = _subset_rows(
        results,
        variant_lookup,
        (
            (
                "same_floor",
                "Same-floor",
                lambda item: item["expected_outcome"] == "success"
                and int(item.get("expected_transition_count") or 0) == 0
                and item.get("expected_target_room") is not None,
            ),
            (
                "cross_floor",
                "Cross-floor",
                lambda item: item["expected_outcome"] == "success"
                and (bool(item.get("requires_vertical_transition")) or int(item.get("expected_transition_count") or 0) > 0),
            ),
        ),
    )
    transition_subset_rows = _subset_rows(
        results,
        variant_lookup,
        (
            ("0_transition", "0-transition", lambda item: item["expected_outcome"] == "success" and int(item.get("expected_transition_count") or 0) == 0),
            ("1_transition", "1-transition", lambda item: item["expected_outcome"] == "success" and int(item.get("expected_transition_count") or 0) == 1),
            ("2_transition", "2-transition", lambda item: item["expected_outcome"] == "success" and int(item.get("expected_transition_count") or 0) == 2),
        ),
    )
    operation_subset_rows = _subset_rows(
        results,
        variant_lookup,
        (
            ("structured_query", "Structured query", lambda item: str(item["task_type"]).startswith("query_")),
            ("execute", "Execute", lambda item: str(item["task_type"]).startswith("execute_")),
            ("resolve_only", "Resolve-only", lambda item: str(item["task_type"]).startswith("resolve_")),
            ("nl_entry", "NL entry", lambda item: item["task_type"] == "nl_request"),
        ),
    )
    outcome_subset_rows = _subset_rows(
        results,
        variant_lookup,
        (
            ("positive_expected", "Positive (success expected)", lambda item: item["expected_outcome"] == "success"),
            ("ambiguous_expected", "Ambiguous expected", lambda item: item["expected_outcome"] == "ambiguous"),
            ("unsupported_expected", "Unsupported expected", lambda item: item["expected_outcome"] == "unsupported"),
            ("unresolved_expected", "Unresolved expected", lambda item: item["expected_outcome"] == "unresolved"),
            ("non_success_expected", "Negative / non-success expected", lambda item: item["expected_outcome"] != "success"),
        ),
    )
    policy_rows = build_policy_comparison_rows(results, variant_lookup)
    policy_detail_rows = build_policy_sensitive_detail_rows(results)

    sequence_metric_rows = [
        build_metric_row(
            label=f"{seq_id} / {variant_key}",
            results=items,
            note=variant_lookup[variant_key].notes,
            extra={"variant_key": variant_key, "seq_id": seq_id},
        )
        for (variant_key, seq_id), items in sorted(_sequence_groups(results).items())
    ]

    _write_csv(report_dir / "tables" / "task_inventory.csv", inventory_rows, ["seq_id", "task_family", "task_type", "count"])
    _write_markdown_table(report_dir / "tables" / "task_inventory.md", inventory_rows, ["seq_id", "task_family", "task_type", "count"], title="Task Inventory")

    main_fields = [
        "task_id",
        "seq_id",
        "task_family",
        "task_type",
        "start_room",
        "expected_target_room",
        "actual_resolved_room",
        "expected_floor_id",
        "actual_resolved_floor_id",
        "route_found",
        "route_hop_count",
        "actual_transition_count",
        "execution_outcome_category",
        "expected_outcome",
        "actual_status",
        "RRA",
        "FCA",
        "TRA",
        "RSR",
        "CTC",
        "VTS",
        "SESR",
        "HOA",
    ]
    _write_csv(report_dir / "tables" / "main_structured_query_execution_results.csv", main_rows, main_fields)
    _write_markdown_table(report_dir / "tables" / "main_structured_query_execution_results.md", main_rows, main_fields, title="Main Structured Query/Execution Results")

    ablation_fields = [
        "variant_key",
        "label",
        "route_policy",
        "execution_validation",
        "RRA",
        "RRA_raw",
        "FCA",
        "FCA_raw",
        "TRA",
        "TRA_raw",
        "RSR",
        "RSR_raw",
        "CTC",
        "CTC_raw",
        "VTS",
        "VTS_raw",
        "SESR",
        "SESR_raw",
        "HOA",
        "HOA_raw",
        "FSR",
        "FSR_raw",
        "IPS",
        "IPS_raw",
        "SRA",
        "SRA_raw",
        "NL_E2E",
        "NL_E2E_raw",
        "avg_route_hops",
        "avg_route_cost",
        "avg_transition_count",
        "task_count",
        "pass_count",
        "note",
    ]
    _write_csv(report_dir / "tables" / "floor_aware_ablation.csv", ablation_rows, ablation_fields)
    _write_markdown_table(report_dir / "tables" / "floor_aware_ablation.md", ablation_rows, ablation_fields, title="Floor-Aware Ablations")

    execution_fields = [
        "task_id",
        "seq_id",
        "variant_key",
        "task_type",
        "start_frame_idx",
        "end_frame_idx",
        "expected_outcome",
        "actual_status",
        "execution_outcome_category",
        "execution_outcome_reason",
        "completed_step_count",
        "total_step_count",
        "SESR",
        "HOA",
        "FSR",
    ]
    _write_csv(report_dir / "tables" / "symbolic_execution_honest_outcome.csv", execution_rows, execution_fields)
    _write_markdown_table(report_dir / "tables" / "symbolic_execution_honest_outcome.md", execution_rows, execution_fields, title="Symbolic Execution / Honest Outcome")
    execution_summary_fields = [
        "variant_key",
        "label",
        "success_count",
        "failure_count",
        "replan_needed_count",
        "ambiguous_count",
        "unsupported_count",
        "unresolved_count",
        "query_only_count",
        "not_run_count",
        "SESR",
        "SESR_raw",
        "HOA",
        "HOA_raw",
        "FSR",
        "FSR_raw",
        "task_count",
        "pass_count",
        "note",
    ]
    _write_csv(report_dir / "tables" / "symbolic_execution_summary.csv", execution_summary_rows, execution_summary_fields)
    _write_markdown_table(report_dir / "tables" / "symbolic_execution_summary.md", execution_summary_rows, execution_summary_fields, title="Symbolic Execution Summary")

    nl_fields = [
        "task_id",
        "seq_id",
        "instruction",
        "expected_intent",
        "actual_intent",
        "expected_target_room",
        "actual_resolved_room",
        "expected_outcome",
        "actual_status",
        "IPS",
        "SRA",
        "NL_E2E",
    ]
    _write_csv(report_dir / "tables" / "nl_entry_results.csv", nl_rows, nl_fields)
    _write_markdown_table(report_dir / "tables" / "nl_entry_results.md", nl_rows, nl_fields, title="NL Entry Results")

    _write_csv(report_dir / "aggregate_results.csv", flatten_result_rows(results), RESULT_CSV_FIELDS)
    _write_markdown_table(report_dir / "aggregate_results.md", flatten_result_rows(results), RESULT_CSV_FIELDS, title="Aggregate Results")
    _json_dump(
        {
            "generated_at_utc": _utc_now_iso(),
            "task_sheet_path": str(task_sheet_path),
            "output_root": str(output_root),
            "report_dir": str(report_dir),
            "variants": [variant.__dict__ for variant in variants],
            "results": results,
            "ablation_rows": ablation_rows,
            "sequence_metric_rows": sequence_metric_rows,
            "execution_summary_rows": execution_summary_rows,
            "floor_scope_rows": floor_scope_rows,
            "transition_subset_rows": transition_subset_rows,
            "operation_subset_rows": operation_subset_rows,
            "outcome_subset_rows": outcome_subset_rows,
            "policy_rows": policy_rows,
            "policy_detail_rows": policy_detail_rows,
        },
        report_dir / "aggregate_results.json",
    )
    sequence_metric_fields = [
        "variant_key",
        "seq_id",
        "label",
        "RRA",
        "RRA_raw",
        "FCA",
        "FCA_raw",
        "TRA",
        "TRA_raw",
        "RSR",
        "RSR_raw",
        "CTC",
        "CTC_raw",
        "VTS",
        "VTS_raw",
        "SESR",
        "SESR_raw",
        "HOA",
        "HOA_raw",
        "IPS",
        "IPS_raw",
        "SRA",
        "SRA_raw",
        "NL_E2E",
        "NL_E2E_raw",
        "avg_route_hops",
        "avg_route_cost",
        "avg_transition_count",
        "task_count",
        "pass_count",
        "note",
    ]
    _write_csv(report_dir / "tables" / "sequence_metric_breakdown.csv", sequence_metric_rows, sequence_metric_fields)
    _write_markdown_table(report_dir / "tables" / "sequence_metric_breakdown.md", sequence_metric_rows, sequence_metric_fields, title="Per-Sequence Metric Breakdown")

    subset_fields = [
        "variant_key",
        "variant_label",
        "subset_key",
        "subset_label",
        "RRA",
        "RRA_raw",
        "FCA",
        "FCA_raw",
        "TRA",
        "TRA_raw",
        "RSR",
        "RSR_raw",
        "CTC",
        "CTC_raw",
        "VTS",
        "VTS_raw",
        "SESR",
        "SESR_raw",
        "HOA",
        "HOA_raw",
        "FSR",
        "FSR_raw",
        "IPS",
        "IPS_raw",
        "SRA",
        "SRA_raw",
        "NL_E2E",
        "NL_E2E_raw",
        "avg_route_hops",
        "avg_route_cost",
        "avg_transition_count",
        "task_count",
        "pass_count",
        "note",
    ]
    _write_csv(report_dir / "tables" / "cross_floor_subset_results.csv", floor_scope_rows, subset_fields)
    _write_markdown_table(report_dir / "tables" / "cross_floor_subset_results.md", floor_scope_rows, subset_fields, title="Same-Floor vs Cross-Floor Subsets")
    _write_csv(report_dir / "tables" / "transition_count_subset_results.csv", transition_subset_rows, subset_fields)
    _write_markdown_table(report_dir / "tables" / "transition_count_subset_results.md", transition_subset_rows, subset_fields, title="Transition-Count Subsets")
    _write_csv(report_dir / "tables" / "query_execute_subset_results.csv", operation_subset_rows, subset_fields)
    _write_markdown_table(report_dir / "tables" / "query_execute_subset_results.md", operation_subset_rows, subset_fields, title="Query / Execute / Resolve / NL Subsets")
    _write_csv(report_dir / "tables" / "outcome_subset_results.csv", outcome_subset_rows, subset_fields)
    _write_markdown_table(report_dir / "tables" / "outcome_subset_results.md", outcome_subset_rows, subset_fields, title="Positive vs Negative Outcome Subsets")

    policy_fields = [
        "variant_key",
        "label",
        "route_policy",
        "TRA",
        "TRA_raw",
        "RSR",
        "RSR_raw",
        "avg_route_hops",
        "avg_route_cost",
        "task_count",
        "pass_count",
        "note",
    ]
    _write_csv(report_dir / "tables" / "policy_comparison_appendix.csv", policy_rows, policy_fields)
    _write_markdown_table(report_dir / "tables" / "policy_comparison_appendix.md", policy_rows, policy_fields, title="Policy Comparison (Appendix)")
    policy_detail_fields = [
        "task_id",
        "seq_id",
        "expected_target_room",
        "strict_hops",
        "balanced_hops",
        "exploratory_hops",
        "strict_cost",
        "balanced_cost",
        "exploratory_cost",
    ]
    _write_csv(report_dir / "tables" / "policy_sensitive_route_details.csv", policy_detail_rows, policy_detail_fields)
    _write_markdown_table(report_dir / "tables" / "policy_sensitive_route_details.md", policy_detail_rows, policy_detail_fields, title="Policy-Sensitive Route Details")

    if write_latex:
        _write_latex_table(
            report_dir / "tables" / "task_inventory.tex",
            inventory_rows,
            ["seq_id", "task_family", "task_type", "count"],
            caption="Task inventory.",
            label="tab:task_inventory",
        )
        _write_latex_table(
            report_dir / "tables" / "main_structured_query_execution_results.tex",
            main_rows,
            main_fields,
            caption="Main structured query and execution results.",
            label="tab:main_results",
        )
        _write_latex_table(
            report_dir / "tables" / "floor_aware_ablation.tex",
            ablation_rows,
            ablation_fields,
            caption="Floor-aware and policy ablations.",
            label="tab:ablations",
        )
        _write_latex_table(
            report_dir / "tables" / "symbolic_execution_honest_outcome.tex",
            execution_rows,
            execution_fields,
            caption="Symbolic execution and honest outcome results.",
            label="tab:symbolic_honest",
        )
        _write_latex_table(
            report_dir / "tables" / "nl_entry_results.tex",
            nl_rows,
            nl_fields,
            caption="NL entry-layer results.",
            label="tab:nl_results",
        )

    write_summary_report(
        report_dir,
        task_sheet=task_sheet,
        output_root=Path(output_root),
        results=results,
        ablation_rows=ablation_rows,
        main_rows=main_rows,
    )
    write_per_sequence_tables(report_dir, results)
    write_sequence_reports(report_dir, results, variant_lookup)
    write_diagnosis_reports(
        report_dir,
        task_sheet=task_sheet,
        ablation_rows=ablation_rows,
        execution_summary_rows=execution_summary_rows,
        policy_rows=policy_rows,
    )

    return {
        "task_sheet": task_sheet,
        "report_dir": str(report_dir),
        "task_count": len(task_sheet.get("tasks", [])),
        "result_count": len(results),
        "variant_count": len(variants),
    }


def _parse_variant_keys(text: Optional[str]) -> Optional[List[str]]:
    if text in (None, ""):
        return None
    return [item.strip() for item in str(text).split(",") if item.strip()]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the floor-aware queryable world-model backend evaluation package."
    )
    parser.add_argument(
        "--task-sheet",
        default=str(DEFAULT_TASK_SHEET),
        help="Path to a JSON or CSV evaluation task sheet.",
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Stage A output root containing per-sequence topology and timeline exports.",
    )
    parser.add_argument(
        "--report-dir",
        default=DEFAULT_REPORT_DIR_NAME,
        help="Report directory name under --output-root.",
    )
    parser.add_argument(
        "--variants",
        default="",
        help="Optional comma-separated subset of variant keys.",
    )
    parser.add_argument(
        "--skip-latex",
        action="store_true",
        help="Skip simple LaTeX table export.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    summary = run_evaluation(
        task_sheet_path=Path(args.task_sheet),
        output_root=Path(args.output_root),
        report_dir_name=args.report_dir,
        variant_keys=_parse_variant_keys(args.variants),
        write_latex=not bool(args.skip_latex),
    )
    print(json.dumps(summary, indent=2))
    return 0


__all__ = [
    "DEFAULT_OUTPUT_ROOT",
    "DEFAULT_REPORT_DIR_NAME",
    "DEFAULT_TASK_SHEET",
    "TASK_SHEET_VERSION",
    "VariantConfig",
    "build_arg_parser",
    "get_variant_configs",
    "load_task_sheet",
    "main",
    "run_evaluation",
    "transform_topology_payload",
    "write_task_sheet_csv",
]
