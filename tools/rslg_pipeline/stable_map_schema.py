"""Schema validation for RSLG-SLAM stable map dry-runs and candidates.

The validator only reads JSON inputs and writes a validation report. It does
not generate final stable maps, pixels, PGM/YAML files, runtime costmaps,
routes, runtime artifacts, or require historical stage_outputs.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, save_json


PASS_CLASSIFICATION = "stable_map_schema_validation_passed"
FAIL_CLASSIFICATION = "stable_map_schema_validation_failed"

ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
MAP_KIND = "stable_occupancy"
DRY_RUN_CLASSIFICATION = "layer2_stable_map_artifact_dry_run_ready"
CANDIDATE_CLASSIFICATION = "stable_map_package_candidate_artifact_generated"

REQUIRED_DRY_RUN_FIELDS = [
    "classification",
    "error_count",
    "warning_count",
    "dry_run",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "map_kind",
    "stage_outputs_required",
    "historical_stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "final_formal_artifacts_generated",
    "stable_map_pixels_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "runtime_costmap_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
    "semantic_floorplan_claimed_as_stable_map",
    "room_mask_claimed_as_stable_map",
    "future_required_layer1_inputs",
    "future_required_layer2_inputs",
    "future_generated_artifacts",
    "planned_candidate_package_preview",
    "blocked_fields",
    "downstream_layer3_unblock_plan",
    "claim_boundary",
]

REQUIRED_CANDIDATE_FIELDS = [
    "schema_name",
    "schema_version",
    "classification",
    "artifact_layer",
    "artifact_role",
    "scene_id",
    "map_kind",
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
    "stable_map_pixels_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "runtime_costmap_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
    "semantic_floorplan_claimed_as_stable_map",
    "room_mask_claimed_as_stable_map",
    "map_payload_status",
    "candidate_payload",
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
    "stable_map_pixels_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "runtime_costmap_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
    "semantic_floorplan_claimed_as_stable_map",
    "room_mask_claimed_as_stable_map",
]

STAGE_OUTPUT_FALSE_FLAGS = [
    "stage_outputs_required",
    "historical_stage_outputs_required",
]

FORBIDDEN_TRUE_FLAGS = [
    "final_stable_map_generated",
    "final_stable_map_artifacts_generated",
    "final_formal_artifacts_generated",
    "stable_map_pixels_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "runtime_costmap_generated",
    "route_generated",
    "real_route_generated",
    "executable_route_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "gazebo_validation_completed",
    "rviz_validation_completed",
    "nav2_validation_completed",
    "amcl_success",
    "external_gt_map_used",
    "simulator_navmesh_used",
]

FORBIDDEN_CLAIM_PHRASES = [
    "final stable map generated",
    "final stable map artifacts generated",
    "pgm generated",
    "map server yaml generated",
    "runtime costmap generated",
    "route generated",
    "executable route generated",
    "runtime validation completed",
    "gazebo rviz nav2 validation completed",
    "gazebo/rviz/nav2 validation completed",
    "amcl success",
    "external gt map used",
    "simulator navmesh used",
]

DOWNSTREAM_REQUIRED_TERMS = [
    "candidate route contract finalization",
    "a star bridge route planning",
    "executable route generation later",
]

MAP_DISTINCTION_TERMS = [
    "not semantic floorplan",
    "not room mask",
    "not runtime costmap",
    "not external gt map",
    "not simulator navmesh",
]


def _check(name: str, ok: bool, details: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {"name": name, "status": "passed" if ok else "failed", "ok": ok, "details": dict(details or {})}


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
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


def _validate_identity(report: Mapping[str, Any], errors: list[str], mode: str) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    if report.get("artifact_layer") != ARTIFACT_LAYER:
        failures["artifact_layer"] = report.get("artifact_layer")
    if report.get("map_kind") != MAP_KIND:
        failures["map_kind"] = report.get("map_kind")
    if not report.get("scene_id"):
        failures["scene_id"] = report.get("scene_id")

    if mode == "dry-run-report":
        for field in REQUIRED_DRY_RUN_FIELDS:
            if field not in report:
                failures[field] = "missing"
        if report.get("classification") != DRY_RUN_CLASSIFICATION:
            failures["classification"] = report.get("classification")
        if report.get("dry_run") is not True:
            failures["dry_run"] = report.get("dry_run")
        if report.get("artifact_role") != "stable_occupancy_map_package":
            failures["artifact_role"] = report.get("artifact_role")
    else:
        for field in REQUIRED_CANDIDATE_FIELDS:
            if field not in report:
                failures[field] = "missing"
        expected = {
            "schema_name": "rslg_stable_map_package_candidate",
            "schema_version": "0.1",
            "classification": CANDIDATE_CLASSIFICATION,
            "artifact_role": "stable_occupancy_map_package_candidate",
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
    for flag in STAGE_OUTPUT_FALSE_FLAGS:
        if flag in report and report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for key, value in failures.items():
        _add_error(errors, f"safety/stage-output flag {key} must be false, got {value!r}")
    return _check("safety_flags_and_stage_outputs_independence", not failures, {"failures": failures})


def _path_claims_generated(path: str, value: Any) -> bool:
    normalized_path = _normalize_text(path)
    if any(marker in normalized_path for marker in ["expected future files", "future generated artifacts"]):
        return False
    if not isinstance(value, str):
        return False
    text = _normalize_text(value)
    if not (text.endswith("pgm") or text.endswith("yaml") or text.endswith("yml")):
        return False
    return "future" not in text and "expected" not in text


def _validate_candidate_payload_boundary(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    payload = _payload(report)
    failures: dict[str, Any] = {}
    if report.get("map_payload_status") != "candidate_metadata_only_no_pixels":
        failures["map_payload_status"] = report.get("map_payload_status")
    if not isinstance(payload, Mapping) or not payload:
        failures["candidate_payload"] = type(report.get("candidate_payload")).__name__
    pixel_keys = []
    generated_file_claims = []
    for path, value in _walk(payload):
        leaf = path.rsplit(".", 1)[-1].lower()
        if leaf in {"pixels", "pixel_array", "pixel_arrays", "raster", "occupancy_grid"} and value:
            pixel_keys.append({"path": path, "value": value})
        if leaf in {"actual_pgm_path", "actual_map_server_yaml_path", "actual_runtime_costmap_path"} and value:
            generated_file_claims.append({"path": path, "value": value})
        if _path_claims_generated(path, value):
            generated_file_claims.append({"path": path, "value": value})
    if pixel_keys:
        failures["pixel_arrays"] = pixel_keys
    if generated_file_claims:
        failures["generated_file_claims"] = generated_file_claims
    for key, value in failures.items():
        _add_error(errors, f"candidate payload boundary failed for {key}: {value!r}")
    return _check("candidate_payload_metadata_only_boundary", not failures, {"failures": failures})


def _validate_map_distinction_policy(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    payload = _payload(report)
    policy = payload.get("map_distinction_policy") if isinstance(payload, Mapping) else None
    text = _normalize_text(" ".join(_string_values({"policy": policy, "claim_boundary": report.get("claim_boundary")})))
    failures: dict[str, Any] = {}
    if isinstance(policy, Mapping):
        expected_flags = {
            "not_semantic_floorplan": True,
            "not_room_mask": True,
            "not_runtime_costmap": True,
            "not_external_gt_map": True,
            "not_simulator_navmesh": True,
        }
        for field, expected in expected_flags.items():
            if policy.get(field) is not expected:
                failures[field] = policy.get(field)
    else:
        missing_terms = [term for term in MAP_DISTINCTION_TERMS if _normalize_text(term) not in text]
        if missing_terms:
            failures["missing_map_distinction_terms"] = missing_terms
    for key, value in failures.items():
        _add_error(errors, f"map distinction policy failed for {key}: {value!r}")
    return _check("map_distinction_policy", not failures, {"failures": failures})


def _validate_downstream_plan(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    plan = report.get("downstream_layer3_unblock_role", report.get("downstream_layer3_unblock_plan"))
    text = _normalize_text(" ".join(_string_values(plan)))
    # Accept both A* and normalized "a star" wording.
    text = text.replace("a route", "a star route")
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
        "map dimensions",
        "map resolution",
        "stable occupancy raster pixels",
        "map provenance json",
    ]
    missing = [term for term in required if _normalize_text(term) not in text]
    if not isinstance(blocked, list):
        _add_error(errors, "blocked stable-map fields must be listed")
    for term in missing:
        _add_error(errors, f"blocked stable-map fields missing {term!r}")
    return _check("blocked_fields", isinstance(blocked, list) and not missing, {"missing": missing})


def _detect_mode(data: Mapping[str, Any], requested_mode: str) -> str:
    if requested_mode in {"dry-run-report", "candidate-package"}:
        return requested_mode
    if data.get("schema_name") == "rslg_stable_map_package_candidate" or data.get("is_candidate") is True:
        return "candidate-package"
    return "dry-run-report"


def validate_stable_map_input(report: Mapping[str, Any], input_path: str | Path, mode: str) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    checks = [
        _validate_identity(report, errors, mode),
        _validate_safety_flags(report, errors),
        _validate_downstream_plan(report, errors),
        _validate_forbidden_claims(report, errors),
    ]
    if mode == "candidate-package":
        checks.extend(
            [
                _validate_candidate_payload_boundary(report, errors),
                _validate_map_distinction_policy(report, errors),
                _validate_blocked_fields(report, errors),
            ]
        )
    else:
        checks.append(_validate_blocked_fields(report, errors))

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
        "map_kind": report.get("map_kind"),
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
    }


def validate_stable_map_files(input_jsons: Iterable[str | Path], mode: str = "auto") -> dict[str, Any]:
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
                    "map_kind": None,
                    "checks": [_check("json_load", False, {"error": str(exc)})],
                    "errors": [str(exc)],
                    "warnings": [],
                }
            )
            continue
        if not isinstance(data, Mapping):
            message = "stable map schema input JSON must be an object"
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
                    "map_kind": None,
                    "checks": [_check("json_object", False, {"actual_type": type(data).__name__})],
                    "errors": [message],
                    "warnings": [],
                }
            )
            continue
        detected_mode = _detect_mode(data, mode)
        file_reports.append(validate_stable_map_input(data, path, detected_mode))

    error_count = sum(int(report.get("error_count", 0) or 0) for report in file_reports)
    warning_count = sum(int(report.get("warning_count", 0) or 0) for report in file_reports)
    all_ok = error_count == 0 and not load_errors and bool(file_reports)
    checks = [
        _check("input_json_files_loaded", not load_errors and bool(file_reports), {"input_count": len(file_reports), "load_errors": load_errors}),
        _check("all_stable_map_schema_reports_passed", all(report.get("ok") for report in file_reports) and not load_errors, {"file_count": len(file_reports)}),
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
                "map_kind": report.get("map_kind"),
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
        "final_formal_artifacts_generated": False,
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "runtime_costmap_generated": False,
        "mode": mode,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate RSLG-SLAM stable map dry-run reports and candidate packages."
    )
    parser.add_argument(
        "--input-json",
        action="append",
        required=True,
        help="Stable map dry-run report or candidate package JSON to validate. May be repeated.",
    )
    parser.add_argument(
        "--mode",
        default="auto",
        choices=["dry-run-report", "candidate-package", "auto"],
        help="Validation mode. Default auto detects each input.",
    )
    parser.add_argument("--output-json", required=True, help="Write the validation report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    report = validate_stable_map_files(args.input_json, mode=args.mode)
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
