"""Schema validation for RSLG-SLAM object-interface dry-runs and candidates.

The validator only reads JSON inputs and writes a validation report. It does
not resolve objects from current World Model Layer outputs, regenerate approach
geometry, generate routes, create runtime artifacts, or require historical
stage_outputs.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, save_json


PASS_CLASSIFICATION = "object_interface_schema_validation_passed"
FAIL_CLASSIFICATION = "object_interface_schema_validation_failed"

ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
DRY_RUN_CLASSIFICATION = "layer2_object_interface_artifact_dry_run_ready"
QUERY_CANDIDATE_CLASSIFICATION = "object_query_resolution_candidate_artifact_generated"
APPROACH_CANDIDATE_CLASSIFICATION = "object_approach_candidate_artifact_generated"
PACKAGE_CANDIDATE_CLASSIFICATION = "object_interface_package_candidate_artifact_generated"

EXPECTED_QUERY = "curtain in room_14 on floor_2"
EXPECTED_OBJECT_ID = "obj_175"
EXPECTED_OBJECT_LABEL = "curtain"
EXPECTED_TARGET_FLOOR = "floor_2"
EXPECTED_TARGET_ROOM = "room_14"
EXPECTED_APPROACH_ID = "generated_ring_002"

REQUIRED_DRY_RUN_FIELDS = [
    "classification",
    "error_count",
    "warning_count",
    "dry_run",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "query",
    "object_id",
    "approach_candidate_id",
    "stage_outputs_required",
    "historical_stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "final_formal_artifacts_generated",
    "object_query_resolution_generated",
    "object_approach_candidate_generated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "stable_map_generated",
    "vertical_connector_generated",
    "real_route_generated",
    "runtime_artifacts_generated",
    "validated_object_truth_used",
    "future_required_layer1_inputs",
    "future_required_layer2_inputs",
    "future_generated_artifacts",
    "planned_candidate_package_preview",
    "blocked_fields",
    "downstream_layer3_unblock_plan",
    "claim_boundary",
]

REQUIRED_QUERY_FIELDS = [
    "schema_name",
    "schema_version",
    "classification",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "query",
    "is_candidate",
    "is_final_formal_artifact",
    "generated_from_validated_milestones_or_contracts",
    "generated_from_current_layer1_outputs",
    "historical_stage_outputs_required",
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "object_query_resolution_generated_from_current_world_model",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "real_route_generated",
    "runtime_artifacts_generated",
    "validated_object_truth_lineage",
    "candidate_payload",
    "blocked_until_layer1_or_layer2_inputs",
    "downstream_layer3_unblock_role",
    "claim_boundary",
]

REQUIRED_APPROACH_FIELDS = [
    "schema_name",
    "schema_version",
    "classification",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "object_id",
    "approach_candidate_id",
    "is_candidate",
    "is_final_formal_artifact",
    "generated_from_validated_milestones_or_contracts",
    "generated_from_current_layer1_outputs",
    "historical_stage_outputs_required",
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "approach_geometry_regenerated",
    "approach_feasibility_revalidated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "real_route_generated",
    "runtime_artifacts_generated",
    "validated_object_truth_lineage",
    "candidate_payload",
    "blocked_until_layer1_or_layer2_inputs",
    "downstream_layer3_unblock_role",
    "claim_boundary",
]

REQUIRED_PACKAGE_FIELDS = [
    "schema_name",
    "schema_version",
    "classification",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "query",
    "object_id",
    "approach_candidate_id",
    "is_candidate",
    "is_final_formal_artifact",
    "generated_from_validated_milestones_or_contracts",
    "generated_from_current_layer1_outputs",
    "historical_stage_outputs_required",
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "object_query_resolution_generated_from_current_world_model",
    "approach_geometry_regenerated",
    "approach_feasibility_revalidated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "stable_map_generated",
    "vertical_connector_generated",
    "real_route_generated",
    "runtime_artifacts_generated",
    "candidate_artifact_references",
    "cross_floor_object_route_context",
    "blocked_until_layer1_or_layer2_inputs",
    "downstream_layer3_unblock_role",
    "claim_boundary",
]

SAFETY_FALSE_FLAGS = [
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "final_formal_artifacts_generated",
    "object_query_resolution_generated",
    "object_query_resolution_generated_from_current_world_model",
    "object_approach_candidate_generated",
    "approach_geometry_regenerated",
    "approach_feasibility_revalidated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "stable_map_generated",
    "vertical_connector_generated",
    "real_route_generated",
    "runtime_artifacts_generated",
]

FORBIDDEN_TRUE_FLAGS = [
    "final_object_interface_artifact_generated",
    "final_object_interface_artifacts_generated",
    "final_formal_artifacts_generated",
    "object_query_resolution_generated",
    "object_query_resolution_generated_from_current_world_model",
    "object_approach_candidate_generated",
    "approach_geometry_regenerated",
    "approach_feasibility_revalidated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
    "route_generated",
    "real_route_generated",
    "executable_route_generated",
    "executable_route_waypoints_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "gazebo_validation_completed",
    "rviz_validation_completed",
    "nav2_validation_completed",
    "object_navigation_benchmark_completed",
    "physical_object_approach_success_completed",
    "amcl_success",
]

FORBIDDEN_CLAIM_PHRASES = [
    "final object interface artifact generated",
    "real object query resolution from current world model completed",
    "real approach geometry regenerated",
    "approach feasibility revalidated",
    "centroid navigation used",
    "direct object centroid goal used",
    "route generated",
    "executable route generated",
    "runtime validation completed",
    "gazebo rviz nav2 validation completed",
    "object navigation benchmark completed",
    "physical object approach success completed",
    "amcl success",
]

DOWNSTREAM_REQUIRED_TERMS = [
    "cross floor object candidate route contract finalization",
    "target room resolution",
    "object approach append",
]


def _check(name: str, ok: bool, details: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {"name": name, "status": "passed" if ok else "failed", "ok": ok, "details": dict(details or {})}


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
    text = text.replace("*", " star ")
    for char in "-_/.,;:()[]{}\"'`":
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
        for key, item in value.items():
            yield str(key)
            yield from _string_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from _string_values(item)
    elif value is not None and not isinstance(value, bool):
        yield str(value)


def _add_error(errors: list[str], message: str) -> None:
    errors.append(message)


def _payload(report: Mapping[str, Any]) -> Mapping[str, Any]:
    value = report.get("candidate_payload")
    return value if isinstance(value, Mapping) else {}


def _detect_mode(data: Mapping[str, Any], requested_mode: str) -> str:
    if requested_mode != "auto":
        return requested_mode
    schema_name = data.get("schema_name")
    if schema_name == "rslg_object_query_resolution_candidate":
        return "query-candidate"
    if schema_name == "rslg_object_approach_candidate":
        return "approach-candidate"
    if schema_name == "rslg_object_interface_package_candidate":
        return "package-candidate"
    return "dry-run-report"


def _validate_identity(report: Mapping[str, Any], errors: list[str], mode: str) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    if report.get("artifact_layer") != ARTIFACT_LAYER:
        failures["artifact_layer"] = report.get("artifact_layer")
    if not report.get("scene_id"):
        failures["scene_id"] = report.get("scene_id")

    if mode == "dry-run-report":
        for field in REQUIRED_DRY_RUN_FIELDS:
            if field not in report:
                failures[field] = "missing"
        expected = {
            "classification": DRY_RUN_CLASSIFICATION,
            "dry_run": True,
            "artifact_role": "object_interface_package",
            "query": EXPECTED_QUERY,
            "object_id": EXPECTED_OBJECT_ID,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
        }
    elif mode == "query-candidate":
        for field in REQUIRED_QUERY_FIELDS:
            if field not in report:
                failures[field] = "missing"
        expected = {
            "schema_name": "rslg_object_query_resolution_candidate",
            "schema_version": "0.1",
            "classification": QUERY_CANDIDATE_CLASSIFICATION,
            "artifact_role": "object_query_resolution_candidate",
            "query": EXPECTED_QUERY,
            "is_candidate": True,
            "is_final_formal_artifact": False,
            "generated_from_validated_milestones_or_contracts": True,
            "generated_from_current_layer1_outputs": False,
        }
    elif mode == "approach-candidate":
        for field in REQUIRED_APPROACH_FIELDS:
            if field not in report:
                failures[field] = "missing"
        expected = {
            "schema_name": "rslg_object_approach_candidate",
            "schema_version": "0.1",
            "classification": APPROACH_CANDIDATE_CLASSIFICATION,
            "artifact_role": "object_approach_candidate",
            "object_id": EXPECTED_OBJECT_ID,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
            "is_candidate": True,
            "is_final_formal_artifact": False,
            "generated_from_validated_milestones_or_contracts": True,
            "generated_from_current_layer1_outputs": False,
        }
    else:
        for field in REQUIRED_PACKAGE_FIELDS:
            if field not in report:
                failures[field] = "missing"
        expected = {
            "schema_name": "rslg_object_interface_package_candidate",
            "schema_version": "0.1",
            "classification": PACKAGE_CANDIDATE_CLASSIFICATION,
            "artifact_role": "object_interface_package_candidate",
            "query": EXPECTED_QUERY,
            "object_id": EXPECTED_OBJECT_ID,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
            "is_candidate": True,
            "is_final_formal_artifact": False,
            "generated_from_validated_milestones_or_contracts": True,
            "generated_from_current_layer1_outputs": False,
        }

    for field, value in expected.items():
        if report.get(field) != value:
            failures[field] = report.get(field)
    for key, value in failures.items():
        _add_error(errors, f"{mode} identity field {key} has invalid value {value!r}")
    return _check(f"{mode}_identity_and_layer", not failures, {"failures": failures})


def _validate_safety_flags(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    for flag in SAFETY_FALSE_FLAGS:
        if flag in report and report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for flag in ["stage_outputs_required", "historical_stage_outputs_required"]:
        if flag in report and report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for key, value in failures.items():
        _add_error(errors, f"safety/stage-output flag {key} must be false, got {value!r}")
    return _check("safety_flags_and_stage_outputs_independence", not failures, {"failures": failures})


def _validate_object_truth(report: Mapping[str, Any], errors: list[str], mode: str) -> dict[str, Any]:
    source = report.get("validated_object_truth_used")
    if not isinstance(source, Mapping):
        source = report.get("validated_object_truth_lineage")
    payload = _payload(report)
    failures: dict[str, Any] = {}

    query = report.get("query", payload.get("query"))
    object_id = report.get("object_id", payload.get("resolved_object_id", payload.get("object_id")))
    object_label = payload.get("object_label")
    target_floor = payload.get("target_floor")
    target_room = payload.get("target_room")
    approach_id = report.get("approach_candidate_id", payload.get("approach_candidate_id"))
    if mode == "dry-run-report" and isinstance(source, Mapping):
        object_label = source.get("object_label")
        target_floor = source.get("target_floor")
        target_room = source.get("target_room")
        approach_id = source.get("approach_candidate_id", approach_id)

    if mode in {"dry-run-report", "query-candidate", "package-candidate"} and query != EXPECTED_QUERY:
        failures["query"] = query
    if object_id != EXPECTED_OBJECT_ID:
        failures["object_id"] = object_id
    if mode != "package-candidate" and object_label != EXPECTED_OBJECT_LABEL:
        failures["object_label"] = object_label
    if mode != "package-candidate" and target_floor != EXPECTED_TARGET_FLOOR:
        failures["target_floor"] = target_floor
    if mode != "package-candidate" and target_room != EXPECTED_TARGET_ROOM:
        failures["target_room"] = target_room
    if mode in {"dry-run-report", "approach-candidate", "package-candidate"} and approach_id != EXPECTED_APPROACH_ID:
        failures["approach_candidate_id"] = approach_id
    if isinstance(source, Mapping):
        if source.get("generated_from_current_layer1_outputs", False) is not False:
            failures["lineage.generated_from_current_layer1_outputs"] = source.get("generated_from_current_layer1_outputs")
        if source.get("world_model_rerun", False) is not False:
            failures["lineage.world_model_rerun"] = source.get("world_model_rerun")
        if source.get("object_centroid_navigation_used") is not False:
            failures["lineage.object_centroid_navigation_used"] = source.get("object_centroid_navigation_used")
    elif mode != "package-candidate":
        failures["validated_object_truth_lineage"] = type(source).__name__

    for key, value in failures.items():
        _add_error(errors, f"object truth check failed for {key}: {value!r}")
    return _check("validated_object_truth", not failures, {"failures": failures})


def _validate_candidate_payload_boundary(report: Mapping[str, Any], errors: list[str], mode: str) -> dict[str, Any]:
    payload = _payload(report)
    failures: dict[str, Any] = {}
    if mode == "query-candidate":
        expected = {
            "resolved_object_id": EXPECTED_OBJECT_ID,
            "object_label": EXPECTED_OBJECT_LABEL,
            "target_floor": EXPECTED_TARGET_FLOOR,
            "target_room": EXPECTED_TARGET_ROOM,
            "query_status": "candidate_resolution_from_validated_truth",
            "ambiguity_status": "not_recomputed_in_this_task",
            "source_status": "validated_milestone_truth_not_current_world_model_rerun",
        }
        for field, value in expected.items():
            if payload.get(field) != value:
                failures[field] = payload.get(field)
    elif mode == "approach-candidate":
        expected = {
            "object_id": EXPECTED_OBJECT_ID,
            "object_label": EXPECTED_OBJECT_LABEL,
            "target_floor": EXPECTED_TARGET_FLOOR,
            "target_room": EXPECTED_TARGET_ROOM,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
            "approach_status": "candidate_from_validated_truth_not_revalidated",
            "centroid_as_goal": False,
            "requires_future_clearance_or_runtime_probe": True,
            "source_status": "validated_milestone_truth_not_current_layer1_regeneration",
            "approach_geometry_status": "candidate_identifier_only_no_regenerated_geometry",
        }
        for field, value in expected.items():
            if payload.get(field) != value:
                failures[field] = payload.get(field)
    if mode == "approach-candidate":
        forbidden_geometry_keys = ["x", "y", "z", "yaw", "pose", "coordinates", "metric_coordinates"]
        present = [key for key in forbidden_geometry_keys if key in payload and payload.get(key)]
        if present:
            failures["unexpected_metric_geometry"] = present
    for key, value in failures.items():
        _add_error(errors, f"candidate payload boundary failed for {key}: {value!r}")
    return _check("candidate_payload_boundary", not failures, {"failures": failures})


def _validate_package_references_and_context(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    refs = report.get("candidate_artifact_references")
    context = report.get("cross_floor_object_route_context")
    text = _normalize_text(" ".join(_string_values({"refs": refs, "context": context})))
    required_terms = [
        "object_query_resolution_candidate_v0_1 json",
        "object_approach_candidate_v0_1 json",
        "room 2",
        "room 3",
        "room 7",
        "room 13",
        "room 14",
        "obj 175",
        "generated ring 037",
    ]
    connector_ok = "vt 1" in text or "vc vt 1" in text
    missing = [term for term in required_terms if _normalize_text(term) not in text]
    if not connector_ok:
        missing.append("vt_1 or vc_vt_1")
    for term in missing:
        _add_error(errors, f"package context missing {term!r}")
    return _check("package_references_and_cross_floor_context", not missing, {"missing": missing})


def _validate_downstream_plan(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    plan = report.get("downstream_layer3_unblock_role", report.get("downstream_layer3_unblock_plan"))
    text = _normalize_text(" ".join(_string_values(plan)))
    missing = [term for term in DOWNSTREAM_REQUIRED_TERMS if _normalize_text(term) not in text]
    completion_claims = []
    for path, value in _walk(plan):
        if path.endswith("completed_in_this_task") and value is not False:
            completion_claims.append({"path": path, "value": value})
    if missing:
        _add_error(errors, f"downstream Layer 3 unblock plan missing terms: {missing}")
    for claim in completion_claims:
        _add_error(errors, f"downstream Layer 3 plan completion flag must be false at {claim['path']}")
    return _check(
        "downstream_layer3_unblock_plan",
        not missing and not completion_claims,
        {"missing": missing, "completion_claims": completion_claims},
    )


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = " ".join(words[max(0, index - 8) : index])
        if any(marker in prefix for marker in ["not", "no", "without", "never", "unvalidated", "must not", "does not"]):
            return True
    return False


def _is_claim_path(path: str) -> bool:
    normalized = _normalize_text(path)
    if "not claimed" in normalized or "forbidden world model sources" in normalized:
        return False
    return any(marker in normalized for marker in ["claim", "status", "classification", "generated", "completed", "success"])


def _validate_forbidden_claims(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    unsafe_flags = []
    phrase_hits = []
    for path, value in _walk(report):
        leaf = path.rsplit(".", 1)[-1]
        if leaf in FORBIDDEN_TRUE_FLAGS and value is True:
            unsafe_flags.append({"path": path, "flag": leaf})
        if isinstance(value, str) and _is_claim_path(path):
            text = _normalize_text(value)
            for phrase in FORBIDDEN_CLAIM_PHRASES:
                normalized_phrase = _normalize_text(phrase)
                if normalized_phrase in text and not _phrase_is_negated(text, normalized_phrase):
                    phrase_hits.append({"path": path, "phrase": phrase, "value": value})
    for hit in unsafe_flags:
        _add_error(errors, f"forbidden positive claim flag {hit['flag']} is true at {hit['path']}")
    for hit in phrase_hits:
        _add_error(errors, f"forbidden positive claim phrase {hit['phrase']!r} found at {hit['path']}")
    return _check("forbidden_claim_boundary", not unsafe_flags and not phrase_hits, {"unsafe_true_flags": unsafe_flags, "positive_phrase_hits": phrase_hits})


def _validate_blocked_fields(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    blocked = report.get("blocked_until_layer1_or_layer2_inputs", report.get("blocked_fields"))
    text = _normalize_text(" ".join(_string_values(blocked)))
    required = [
        "object observations",
        "object room association",
        "approach geometry",
        "approach feasibility",
        "cross floor object candidate route contract finalization",
    ]
    missing = [term for term in required if _normalize_text(term) not in text]
    if not isinstance(blocked, list):
        _add_error(errors, "blocked object-interface fields must be listed")
    for term in missing:
        _add_error(errors, f"blocked object-interface fields missing {term!r}")
    return _check("blocked_fields", isinstance(blocked, list) and not missing, {"missing": missing})


def validate_object_interface_input(report: Mapping[str, Any], input_path: str | Path, mode: str) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    checks = [
        _validate_identity(report, errors, mode),
        _validate_safety_flags(report, errors),
        _validate_object_truth(report, errors, mode),
        _validate_downstream_plan(report, errors),
        _validate_forbidden_claims(report, errors),
        _validate_blocked_fields(report, errors),
    ]
    if mode in {"query-candidate", "approach-candidate"}:
        checks.append(_validate_candidate_payload_boundary(report, errors, mode))
    if mode == "package-candidate":
        checks.append(_validate_package_references_and_context(report, errors))

    ok = not errors
    return {
        "input_json": Path(input_path).as_posix(),
        "ok": ok,
        "classification": PASS_CLASSIFICATION if ok else FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "mode": mode,
        "input_classification": report.get("classification"),
        "scene_id": report.get("scene_id"),
        "query": report.get("query"),
        "object_id": report.get("object_id"),
        "approach_candidate_id": report.get("approach_candidate_id"),
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
    }


def validate_object_interface_files(input_jsons: Iterable[str | Path], mode: str = "auto") -> dict[str, Any]:
    file_reports: list[dict[str, Any]] = []
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
                    "mode": mode,
                    "input_classification": None,
                    "scene_id": None,
                    "query": None,
                    "object_id": None,
                    "approach_candidate_id": None,
                    "checks": [_check("json_load", False, {"error": str(exc)})],
                    "errors": [str(exc)],
                    "warnings": [],
                }
            )
            continue
        if not isinstance(data, Mapping):
            message = "object interface schema input JSON must be an object"
            file_reports.append(
                {
                    "input_json": path.as_posix(),
                    "ok": False,
                    "classification": FAIL_CLASSIFICATION,
                    "error_count": 1,
                    "warning_count": 0,
                    "mode": mode,
                    "input_classification": None,
                    "scene_id": None,
                    "query": None,
                    "object_id": None,
                    "approach_candidate_id": None,
                    "checks": [_check("json_object", False, {"actual_type": type(data).__name__})],
                    "errors": [message],
                    "warnings": [],
                }
            )
            continue
        detected_mode = _detect_mode(data, mode)
        file_reports.append(validate_object_interface_input(data, path, detected_mode))

    error_count = sum(int(report.get("error_count", 0) or 0) for report in file_reports)
    warning_count = sum(int(report.get("warning_count", 0) or 0) for report in file_reports)
    all_ok = error_count == 0 and not load_errors and bool(file_reports)
    checks = [
        _check("input_json_files_loaded", not load_errors and bool(file_reports), {"input_count": len(file_reports), "load_errors": load_errors}),
        _check("all_object_interface_schema_reports_passed", all(report.get("ok") for report in file_reports) and not load_errors, {"file_count": len(file_reports)}),
    ]
    return {
        "project_name": PROJECT_NAME,
        "classification": PASS_CLASSIFICATION if all_ok else FAIL_CLASSIFICATION,
        "ok": all_ok,
        "error_count": error_count,
        "warning_count": warning_count,
        "validated_files": [
            {
                "input_json": report["input_json"],
                "ok": report["ok"],
                "classification": report["classification"],
                "mode": report["mode"],
                "input_classification": report.get("input_classification"),
                "scene_id": report.get("scene_id"),
                "query": report.get("query"),
                "object_id": report.get("object_id"),
                "approach_candidate_id": report.get("approach_candidate_id"),
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
            }
            for report in file_reports
        ],
        "checks": checks,
        "file_reports": file_reports,
        "errors": [error for report in file_reports for error in report.get("errors", [])] + load_errors,
        "warnings": [warning for report in file_reports for warning in report.get("warnings", [])],
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "object_query_resolution_generated": False,
        "object_approach_candidate_generated": False,
        "approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "mode": mode,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate RSLG-SLAM object-interface dry-run reports and candidate artifacts."
    )
    parser.add_argument(
        "--input-json",
        action="append",
        required=True,
        help="Object-interface dry-run report or candidate artifact JSON to validate. May be repeated.",
    )
    parser.add_argument(
        "--mode",
        default="auto",
        choices=["dry-run-report", "query-candidate", "approach-candidate", "package-candidate", "auto"],
        help="Validation mode. Default auto detects each input.",
    )
    parser.add_argument("--output-json", required=True, help="Write the validation report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    report = validate_object_interface_files(args.input_json, mode=args.mode)
    output_path = Path(args.output_json).expanduser().resolve()
    save_json(output_path, report)
    print(
        json.dumps(
            {
                "classification": report["classification"],
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
                "output_json": output_path.as_posix(),
            },
            indent=2,
            sort_keys=False,
        )
    )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
