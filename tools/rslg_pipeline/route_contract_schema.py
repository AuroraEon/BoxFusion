"""Lightweight schema validation for RSLG-SLAM route contract stubs.

This validator checks task-evidence route contract stubs without building real
routes, requiring generated stage outputs, importing ROS/runtime modules, or
calling historical route-generation scripts.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping


PASS_CLASSIFICATION = "route_contract_stub_schema_validation_passed"
FAIL_CLASSIFICATION = "route_contract_stub_schema_validation_failed"

SCHEMA_NAME = "rslg_route_contract_stub"
SCHEMA_VERSION = "0.1"
STUB_CLASSIFICATION = "route_contract_stub_generated"
ALLOWED_STUB_TYPES = {"cross_floor_room", "cross_floor_object"}

REQUIRED_ROUTE_IDS = ["room_2", "room_3", "room_7", "room_13", "room_14"]
CONNECTOR_IDS = {"vt_1", "vc_vt_1"}
TRANSITION_EDGE_ID = "vt_1_centerline_e001"
NOT_TRANSITION_EDGE_ID = "vt_1_centerline_e003"

SAFE_FALSE_FLAGS = [
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "business_logic_migrated",
    "real_astar_route_generated",
    "stable_map_generated",
    "old_scripts_called",
    "is_final_navigation_artifact",
]

OBJECT_FIELDS = [
    "query",
    "object_id",
    "object_label",
    "target_floor",
    "target_room",
    "approach_candidate",
    "object_centroid_navigation_used",
    "real_object_approach_generated",
    "object_approach_is_stub",
]

FORBIDDEN_TRUE_FLAGS = [
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "footstep_planning_completed",
    "gait_control_completed",
    "contact_dynamics_handled",
    "dense_reconstruction_completed",
    "full_object_navigation_benchmark_completed",
    "amcl_success",
    "final_executable_route_generated",
    "final_runtime_validation_completed",
]

FORBIDDEN_CLAIM_PHRASES = [
    "physical stair climbing completed",
    "real robot stair climbing completed",
    "footstep planning completed",
    "gait control completed",
    "contact dynamics handled",
    "dense reconstruction completed",
    "full object navigation benchmark completed",
    "amcl success",
    "final executable route generated",
    "final runtime validation completed",
]

HISTORICAL_STAGE_OUTPUT_MARKERS = [
    "stage_outputs/stage1_00824_step30p1",
    "stage_outputs/stage1_generalization/00824",
    "clean_rerun/task24",
    "clean_rerun/task25",
]


def _check(name: str, ok: bool, details: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {"name": name, "status": "passed" if ok else "failed", "ok": ok, "details": dict(details or {})}


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
    for char in "-/.,;:()[]{}\"'":
        text = text.replace(char, " ")
    return " ".join(text.split())


def _walk(value: Any, path: str = "$") -> Iterator[tuple[str, Any]]:
    yield path, value
    if isinstance(value, Mapping):
        for key, item in value.items():
            yield from _walk(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _walk(item, f"{path}[{index}]")


def _string_values(value: Any) -> Iterator[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for item in value.values():
            yield from _string_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from _string_values(item)
    elif value is not None and not isinstance(value, bool):
        yield str(value)


def _extract_identifier_tokens(value: Any) -> set[str]:
    text = " ".join(_string_values(value)).lower()
    return set(re.findall(r"[a-z]+(?:_[a-z0-9]+)+", text))


def _is_not_applicable(value: Any) -> bool:
    if value is None:
        return True
    if value is False:
        return True
    if isinstance(value, str):
        return _normalize_text(value) in {"", "n a", "na", "none", "null", "not applicable", "not applicable for room stub"}
    if isinstance(value, list):
        return not value
    if isinstance(value, Mapping):
        return not value or _normalize_text(value.get("status", "")) in {"not applicable", "n a", "na"}
    return False


def _add_error(errors: list[str], message: str) -> None:
    errors.append(message)


def _add_warning(warnings: list[str], message: str) -> None:
    warnings.append(message)


def _validate_identity(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    expected = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "classification": STUB_CLASSIFICATION,
        "is_stub": True,
        "is_final_navigation_artifact": False,
    }
    observed = {key: stub.get(key) for key in expected}
    missing = [key for key in ["scene_id", "stub_type"] if key not in stub]
    mismatches = {key: {"expected": expected[key], "actual": stub.get(key)} for key in expected if stub.get(key) != expected[key]}
    for key, values in mismatches.items():
        _add_error(errors, f"identity field {key} expected {values['expected']!r}, got {values['actual']!r}")
    for key in missing:
        _add_error(errors, f"missing required identity field {key}")
    return _check("required_top_level_identity_fields", not mismatches and not missing, {"observed": observed, "missing": missing, "mismatches": mismatches})


def _validate_stub_type(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    stub_type = stub.get("stub_type")
    ok = stub_type in ALLOWED_STUB_TYPES
    if not ok:
        _add_error(errors, f"unsupported stub_type {stub_type!r}")
    return _check("allowed_stub_type", ok, {"stub_type": stub_type, "allowed_stub_types": sorted(ALLOWED_STUB_TYPES)})


def _validate_safe_flags(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    missing = [key for key in SAFE_FALSE_FLAGS if key not in stub]
    unsafe = {key: stub.get(key) for key in SAFE_FALSE_FLAGS if key in stub and stub.get(key) is not False}
    for key in missing:
        _add_error(errors, f"missing safety flag {key}")
    for key, value in unsafe.items():
        _add_error(errors, f"safety flag {key} must be false, got {value!r}")
    return _check("safety_and_claim_boundary_flags", not missing and not unsafe, {"missing": missing, "unsafe_values": unsafe})


def _validate_route_truth(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    route_sequence = stub.get("route_sequence")
    tokens = _extract_identifier_tokens(route_sequence)
    route_presence = {route_id: route_id in tokens for route_id in REQUIRED_ROUTE_IDS}
    connector_present = bool(tokens.intersection(CONNECTOR_IDS))
    transition_ok = stub.get("transition_edge_id") == TRANSITION_EDGE_ID
    not_transition_ok = stub.get("not_transition_edge_id") == NOT_TRANSITION_EDGE_ID
    if route_sequence is None:
        _add_error(errors, "missing route_sequence")
    for route_id, present in route_presence.items():
        if not present:
            _add_error(errors, f"route_sequence is missing {route_id}")
    if not connector_present:
        _add_error(errors, "route_sequence is missing vt_1 or vc_vt_1 connector")
    if not transition_ok:
        _add_error(errors, f"transition_edge_id must be {TRANSITION_EDGE_ID!r}")
    if not not_transition_ok:
        _add_error(errors, f"not_transition_edge_id must be {NOT_TRANSITION_EDGE_ID!r}")
    ok = route_sequence is not None and all(route_presence.values()) and connector_present and transition_ok and not_transition_ok
    return _check(
        "route_truth",
        ok,
        {
            "extracted_tokens": sorted(tokens),
            "required_route_ids_present": route_presence,
            "connector_present": connector_present,
            "transition_edge_id": stub.get("transition_edge_id"),
            "not_transition_edge_id": stub.get("not_transition_edge_id"),
        },
    )


def _validate_object_truth(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    if stub.get("stub_type") != "cross_floor_object":
        return _check("object_truth", True, {"skipped": True, "reason": "stub_type is not cross_floor_object"})

    expected = {
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor": "floor_2",
        "target_room": "room_14",
        "approach_candidate": "generated_ring_037",
        "object_centroid_navigation_used": False,
    }
    mismatches = {key: {"expected": expected[key], "actual": stub.get(key)} for key in expected if stub.get(key) != expected[key]}
    query_ok = "curtain" in _normalize_text(stub.get("query", ""))
    optional_flag_errors = {}
    if "real_object_approach_generated" in stub and stub.get("real_object_approach_generated") is not False:
        optional_flag_errors["real_object_approach_generated"] = stub.get("real_object_approach_generated")
    if "object_approach_is_stub" in stub and stub.get("object_approach_is_stub") is not True:
        optional_flag_errors["object_approach_is_stub"] = stub.get("object_approach_is_stub")
    if not query_ok:
        _add_error(errors, "cross_floor_object query must contain 'curtain'")
    for key, values in mismatches.items():
        _add_error(errors, f"object truth field {key} expected {values['expected']!r}, got {values['actual']!r}")
    for key, value in optional_flag_errors.items():
        _add_error(errors, f"optional object safety flag {key} has unsafe value {value!r}")
    return _check(
        "object_truth_for_cross_floor_object",
        query_ok and not mismatches and not optional_flag_errors,
        {
            "query_contains_curtain": query_ok,
            "mismatches": mismatches,
            "optional_flag_errors": optional_flag_errors,
        },
    )


def _validate_room_object_fields(stub: Mapping[str, Any], errors: list[str], warnings: list[str]) -> dict[str, Any]:
    if stub.get("stub_type") != "cross_floor_room":
        return _check("room_only_object_field_boundary", True, {"skipped": True, "reason": "stub_type is not cross_floor_room"})

    present = {key: stub.get(key) for key in OBJECT_FIELDS if key in stub}
    contradictions: dict[str, Any] = {}
    harmless: dict[str, Any] = {}
    for key, value in present.items():
        if _is_not_applicable(value):
            harmless[key] = value
            continue
        if key in {"real_object_approach_generated"} and value is False:
            harmless[key] = value
            continue
        if key in {"object_approach_is_stub"} and value is True:
            harmless[key] = value
            continue
        contradictions[key] = value
    for key, value in contradictions.items():
        _add_error(errors, f"cross_floor_room contradicts room-only scope with {key}={value!r}")
    if harmless:
        _add_warning(warnings, "cross_floor_room contains object-specific fields marked not applicable or harmless")
    return _check("room_only_object_field_boundary", not contradictions, {"present_object_fields": present, "harmless": harmless, "contradictions": contradictions})


def _validate_future_paths(stub: Mapping[str, Any], errors: list[str], warnings: list[str]) -> dict[str, Any]:
    problematic_required_flags: dict[str, Any] = {}
    future_or_expected_fields = []
    historical_path_references = []
    for path, value in _walk(stub):
        leaf = path.rsplit(".", 1)[-1].lower()
        if any(marker in leaf for marker in ["future", "expected"]):
            future_or_expected_fields.append(path)
        if leaf in {"required_for_validation", "required_for_schema_validation", "active_required_local_file", "existence_checked"} and value is True:
            problematic_required_flags[path] = value
        if isinstance(value, str):
            normalized_path = value.replace("\\", "/")
            if any(marker in normalized_path for marker in HISTORICAL_STAGE_OUTPUT_MARKERS):
                historical_path_references.append({"path": path, "value": value})
    for path in problematic_required_flags:
        _add_error(errors, f"future artifact path field is marked as an active required local file: {path}")
    if not future_or_expected_fields:
        _add_warning(warnings, "stub has no explicit expected/future artifact fields")
    if historical_path_references:
        _add_warning(warnings, "stub references historical stage_outputs paths; validator did not require them")
    return _check(
        "future_artifact_path_boundary",
        not problematic_required_flags,
        {
            "future_or_expected_fields": future_or_expected_fields,
            "problematic_required_flags": problematic_required_flags,
            "historical_path_references": historical_path_references,
            "missing_paths_checked": False,
        },
    )


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = words[max(0, index - 6) : index]
        prefix_text = " ".join(prefix)
        if any(marker in prefix_text for marker in ["not", "no", "without", "never", "unvalidated", "must not", "do not", "does not"]):
            return True
    return False


def _claim_boundary_strings(stub: Mapping[str, Any]) -> list[dict[str, str]]:
    claims = []
    for path, value in _walk(stub):
        path_lower = path.lower()
        if isinstance(value, str) and ("claim" in path_lower or "boundary" in path_lower or "status" in path_lower):
            claims.append({"path": path, "value": value})
    return claims


def _validate_forbidden_claims(stub: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    unsafe_flags = {key: stub.get(key) for key in FORBIDDEN_TRUE_FLAGS if stub.get(key) is True}
    phrase_hits = []
    for claim in _claim_boundary_strings(stub):
        text = _normalize_text(claim["value"])
        for phrase in FORBIDDEN_CLAIM_PHRASES:
            normalized_phrase = _normalize_text(phrase)
            if normalized_phrase in text and not _phrase_is_negated(text, normalized_phrase):
                phrase_hits.append({"path": claim["path"], "phrase": phrase, "value": claim["value"]})
    for key in unsafe_flags:
        _add_error(errors, f"forbidden claim flag {key} is true")
    for hit in phrase_hits:
        _add_error(errors, f"forbidden positive claim phrase {hit['phrase']!r} found at {hit['path']}")
    return _check("forbidden_claim_boundary", not unsafe_flags and not phrase_hits, {"unsafe_true_flags": unsafe_flags, "positive_phrase_hits": phrase_hits})


def _validate_output_layer(stub: Mapping[str, Any], warnings: list[str]) -> dict[str, Any]:
    layer_fields = {}
    for path, value in _walk(stub):
        key = path.rsplit(".", 1)[-1].lower()
        if key in {"layer", "artifact_layer", "output_layer", "expected_output_layer"}:
            layer_fields[path] = value
    if not layer_fields:
        _add_warning(warnings, "stub has no explicit Layer 3 or Navigation Interface Layer field")
        return _check("output_artifact_layer", True, {"layer_fields": {}, "warning": "missing optional layer field"})
    layer_ok = any("layer 3" in _normalize_text(value) or "navigation interface layer" in _normalize_text(value) for value in layer_fields.values())
    if not layer_ok:
        _add_warning(warnings, "stub has layer fields but none identify Layer 3: Navigation Interface Layer")
    return _check("output_artifact_layer", True, {"layer_fields": layer_fields, "layer3_or_navigation_interface_layer_present": layer_ok})


def _validate_artifact_role(stub: Mapping[str, Any], warnings: list[str]) -> dict[str, Any]:
    role = stub.get("artifact_role")
    if role is None:
        _add_warning(warnings, "stub has no explicit artifact_role field")
        return _check("artifact_role", True, {"artifact_role": None, "warning": "missing optional artifact_role"})
    role_ok = role == "route_contract_stub"
    if not role_ok:
        _add_warning(warnings, "stub artifact_role is present but is not route_contract_stub")
    return _check("artifact_role", True, {"artifact_role": role, "route_contract_stub_role_present": role_ok})


def _validate_stage_outputs_independence(stub: Mapping[str, Any], input_path: Path, errors: list[str]) -> dict[str, Any]:
    stage_outputs_required = stub.get("stage_outputs_required") is True
    if stage_outputs_required:
        _add_error(errors, "stage_outputs_required must be false for route contract stubs")
    return _check(
        "stage_outputs_independence",
        not stage_outputs_required,
        {
            "input_json": input_path.as_posix(),
            "stage_outputs_required": stub.get("stage_outputs_required"),
            "historical_stage_outputs_required": False,
            "input_file_existence_checked": True,
            "generated_output_paths_existence_checked": False,
        },
    )


def validate_route_contract_stub(stub: Mapping[str, Any], input_path: str | Path = "<memory>") -> dict[str, Any]:
    """Validate one loaded route contract stub and return a file report."""
    errors: list[str] = []
    warnings: list[str] = []
    path = Path(input_path)
    checks = [
        _validate_identity(stub, errors),
        _validate_stub_type(stub, errors),
        _validate_safe_flags(stub, errors),
        _validate_route_truth(stub, errors),
        _validate_object_truth(stub, errors),
        _validate_room_object_fields(stub, errors, warnings),
        _validate_future_paths(stub, errors, warnings),
        _validate_forbidden_claims(stub, errors),
        _validate_output_layer(stub, warnings),
        _validate_artifact_role(stub, warnings),
        _validate_stage_outputs_independence(stub, path, errors),
    ]
    ok = not errors
    return {
        "input_json": path.as_posix(),
        "ok": ok,
        "classification": PASS_CLASSIFICATION if ok else FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "stub_type": stub.get("stub_type"),
        "stub_classification": stub.get("classification"),
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
    }


def validate_route_contract_stub_files(input_jsons: Iterable[str | Path]) -> dict[str, Any]:
    """Load and validate one or more stub JSON files."""
    file_reports = []
    load_errors: list[str] = []
    for input_json in input_jsons:
        path = Path(input_json).expanduser().resolve()
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception as exc:
            load_errors.append(f"{path.as_posix()}: {exc}")
            file_reports.append(
                {
                    "input_json": path.as_posix(),
                    "ok": False,
                    "classification": FAIL_CLASSIFICATION,
                    "error_count": 1,
                    "warning_count": 0,
                    "checks": [_check("json_load", False, {"error": str(exc)})],
                    "errors": [str(exc)],
                    "warnings": [],
                }
            )
            continue
        if not isinstance(data, Mapping):
            message = "route contract stub JSON must be an object"
            file_reports.append(
                {
                    "input_json": path.as_posix(),
                    "ok": False,
                    "classification": FAIL_CLASSIFICATION,
                    "error_count": 1,
                    "warning_count": 0,
                    "checks": [_check("json_object", False, {"actual_type": type(data).__name__})],
                    "errors": [message],
                    "warnings": [],
                }
            )
            continue
        file_reports.append(validate_route_contract_stub(data, path))

    error_count = sum(report["error_count"] for report in file_reports)
    warning_count = sum(report["warning_count"] for report in file_reports)
    checks = [
        _check(
            "input_json_files_loaded",
            not load_errors,
            {"input_count": len(file_reports), "load_errors": load_errors},
        ),
        _check(
            "all_stub_schema_reports_passed",
            all(report.get("ok") for report in file_reports) and not load_errors,
            {"file_count": len(file_reports)},
        ),
    ]
    classification = PASS_CLASSIFICATION if error_count == 0 else FAIL_CLASSIFICATION
    return {
        "classification": classification,
        "ok": error_count == 0,
        "error_count": error_count,
        "warning_count": warning_count,
        "validated_files": [
            {
                "input_json": report["input_json"],
                "ok": report["ok"],
                "classification": report["classification"],
                "stub_type": report.get("stub_type"),
                "stub_classification": report.get("stub_classification"),
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
            }
            for report in file_reports
        ],
        "checks": checks,
        "file_reports": file_reports,
        "errors": [error for report in file_reports for error in report.get("errors", [])] + load_errors,
        "warnings": [warning for report in file_reports for warning in report.get("warnings", [])],
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "stage_outputs_required": False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate RSLG-SLAM route contract stub JSON schemas.")
    parser.add_argument("--input-json", action="append", required=True, help="Route contract stub JSON to validate. May be repeated.")
    parser.add_argument("--output-json", required=True, help="Write the validation report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    report = validate_route_contract_stub_files(args.input_json)
    output_path = Path(args.output_json).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=False)
        handle.write("\n")
    print(json.dumps({"classification": report["classification"], "error_count": report["error_count"], "warning_count": report["warning_count"], "output_json": output_path.as_posix()}, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
