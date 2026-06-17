"""Schema validation for RSLG-SLAM Layer 2 vertical connector artifacts.

This module validates dry-run reports and non-final candidate artifacts produced
by ``tools.rslg_pipeline.build_vertical_connectors``. It only reads JSON reports
and writes a validation report; it never generates final formal artifacts,
requires historical stage_outputs, calls old scripts, reruns the World Model
Layer, or launches runtime systems.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, save_json


PASS_CLASSIFICATION = "vertical_connector_schema_validation_passed"
FAIL_CLASSIFICATION = "vertical_connector_schema_validation_failed"

ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"

ALLOWED_DRY_RUN_CLASSIFICATIONS = {
    "layer2_vertical_connector_topology_dry_run_ready",
    "layer2_vertical_connector_dry_run_ready",
    "layer2_connector_graph_dry_run_ready",
    "layer2_cross_floor_topology_dry_run_ready",
}

EXPECTED_CLASSIFICATION_BY_KIND = {
    "all": "layer2_vertical_connector_topology_dry_run_ready",
    "vertical_connectors": "layer2_vertical_connector_dry_run_ready",
    "connector_graph": "layer2_connector_graph_dry_run_ready",
    "cross_floor_topology": "layer2_cross_floor_topology_dry_run_ready",
}

EXPECTED_PREVIEW_NAMES = {
    "vertical_connectors": "vertical_connectors_v0_1.json",
    "connector_graph": "stairs_or_vertical_connector_graph_v0_1.json",
    "cross_floor_topology": "cross_floor_topology_v0_1.json",
}

EXPECTED_PREVIEWS_BY_KIND = {
    "all": set(EXPECTED_PREVIEW_NAMES.values()),
    "vertical_connectors": {EXPECTED_PREVIEW_NAMES["vertical_connectors"]},
    "connector_graph": {EXPECTED_PREVIEW_NAMES["connector_graph"]},
    "cross_floor_topology": {EXPECTED_PREVIEW_NAMES["cross_floor_topology"]},
}

ALLOWED_CANDIDATE_CLASSIFICATIONS = {
    "vertical_connector_candidate_artifact_generated",
    "connector_graph_candidate_artifact_generated",
    "cross_floor_topology_candidate_artifact_generated",
}

EXPECTED_CANDIDATE_TYPE_FIELDS = {
    "vertical_connector_candidate_artifact_generated": {
        "schema_name": "rslg_vertical_connectors_candidate",
        "artifact_role": "vertical_connector_candidate_artifact",
    },
    "connector_graph_candidate_artifact_generated": {
        "schema_name": "rslg_stairs_or_vertical_connector_graph_candidate",
        "artifact_role": "vertical_connector_graph_candidate_artifact",
    },
    "cross_floor_topology_candidate_artifact_generated": {
        "schema_name": "rslg_cross_floor_topology_candidate",
        "artifact_role": "cross_floor_topology_candidate_artifact",
    },
}

REQUIRED_FALSE_FLAGS = [
    "final_formal_artifacts_generated",
    "vertical_connector_geometry_generated",
    "cross_floor_topology_generated",
    "stable_map_generated",
    "object_interface_generated",
    "real_route_generated",
    "runtime_artifacts_generated",
]

REQUIRED_GUARD_FALSE_FLAGS = [
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
]

OPTIONAL_FALSE_FLAGS = [
    "historical_stage_outputs_required",
    "business_logic_migrated",
]

CONNECTOR_IDS = {"vt_1", "vc_vt_1"}
SOURCE_FLOOR = "floor_1"
TARGET_FLOOR = "floor_2"
TRANSITION_EDGE_ID = "vt_1_centerline_e001"
NON_TRANSITION_EDGE_ID = "vt_1_centerline_e003"
ROUTE_CONTEXT_IDS = ["room_2", "room_3", "room_7", "room_13", "room_14"]
CENTERLINE_NODE_IDS = [
    "vt_1_centerline_n000",
    "vt_1_centerline_n001",
    "vt_1_centerline_n002",
    "vt_1_centerline_n003",
    "vt_1_centerline_n004",
]

BLOCKED_FIELD_TERMS = [
    "real connector geometry regenerated from current Layer 1 outputs",
    "real connector centerline fitted from current pose-height transition evidence",
    "real connector graph generated from current formal artifacts",
    "real cross-floor topology generated from current formal artifacts",
    "stable occupancy map verified locally",
    "route planner-ready topology package",
    "downstream final candidate route contract",
    "A* waypoint route",
    "executable route",
    "runtime validation",
]

COMMON_LAYER3_DEPENDENCIES = [
    "stable occupancy map package",
    "vertical connector artifact",
    "cross-floor topology artifact",
]

OBJECT_LAYER3_DEPENDENCIES = [
    "object query resolution artifact",
    "object approach candidate artifact",
]

FORBIDDEN_TRUE_FLAGS = [
    "is_final_formal_artifact",
    "final_layer2_artifacts_generated",
    "final_formal_artifacts_generated",
    "real_connector_geometry_generated",
    "vertical_connector_geometry_generated",
    "real_cross_floor_topology_generated",
    "cross_floor_topology_generated",
    "real_stable_map_generated",
    "stable_map_generated",
    "real_route_generated",
    "executable_route_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "footstep_planning_completed",
    "gait_control_completed",
    "contact_dynamics_handled",
    "footstep_gait_contact_planning_completed",
    "dense_reconstruction_completed",
    "amcl_success",
]

FORBIDDEN_CLAIM_PHRASES = [
    "final Layer 2 formal artifacts generated",
    "final formal artifacts generated",
    "real connector geometry generated",
    "real cross-floor topology generated",
    "real stable map generated",
    "real route generated",
    "executable route generated",
    "runtime validation completed",
    "Gazebo/RViz/Nav2 validation completed",
    "Gazebo RViz Nav2 validation completed",
    "physical stair climbing completed",
    "real robot stair climbing completed",
    "footstep planning completed",
    "gait control completed",
    "contact dynamics completed",
    "contact dynamics handled",
    "footstep/gait/contact planning completed",
    "dense reconstruction completed",
    "AMCL success",
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
        for item in value.values():
            yield from _string_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from _string_values(item)
    elif value is not None and not isinstance(value, bool):
        yield str(value)


def _identifier_tokens(value: Any) -> set[str]:
    text = " ".join(_string_values(value)).lower()
    return set(re.findall(r"[a-z]+(?:_[a-z0-9]+)+", text))


def _add_error(errors: list[str], message: str) -> None:
    errors.append(message)


def _truth(report: Mapping[str, Any]) -> Mapping[str, Any]:
    value = report.get("validated_truth_used")
    return value if isinstance(value, Mapping) else {}


def _candidate_payload(report: Mapping[str, Any]) -> Mapping[str, Any]:
    value = report.get("candidate_payload")
    return value if isinstance(value, Mapping) else {}


def _candidate_lineage(report: Mapping[str, Any]) -> Mapping[str, Any]:
    value = report.get("validated_truth_lineage")
    return value if isinstance(value, Mapping) else {}


def _lookup_nested(mapping: Mapping[str, Any], path: Iterable[str]) -> Any:
    current: Any = mapping
    for item in path:
        if not isinstance(current, Mapping):
            return None
        current = current.get(item)
    return current


def _candidate_value(report: Mapping[str, Any], paths: Iterable[Iterable[str]]) -> Any:
    payload = _candidate_payload(report)
    lineage = _candidate_lineage(report)
    for path in paths:
        value = _lookup_nested(payload, path)
        if value is not None:
            return value
    for path in paths:
        value = _lookup_nested(lineage, path)
        if value is not None:
            return value
    return None


def _validate_identity_and_layer(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    expected_classification = EXPECTED_CLASSIFICATION_BY_KIND.get(str(report.get("artifact_kind")))
    failures: dict[str, Any] = {}
    if report.get("dry_run") is not True:
        failures["dry_run"] = report.get("dry_run")
    if report.get("artifact_layer") != ARTIFACT_LAYER:
        failures["artifact_layer"] = report.get("artifact_layer")
    if not report.get("scene_id"):
        failures["scene_id"] = report.get("scene_id")
    if not report.get("artifact_kind"):
        failures["artifact_kind"] = report.get("artifact_kind")
    if report.get("classification") not in ALLOWED_DRY_RUN_CLASSIFICATIONS:
        failures["classification"] = report.get("classification")
    if expected_classification and report.get("classification") != expected_classification:
        failures["artifact_kind_classification_mismatch"] = {
            "artifact_kind": report.get("artifact_kind"),
            "expected": expected_classification,
            "actual": report.get("classification"),
        }
    for flag in REQUIRED_FALSE_FLAGS:
        if report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for key, value in failures.items():
        _add_error(errors, f"identity/layer field {key} has invalid value {value!r}")
    return _check(
        "dry_run_identity_and_layer_metadata",
        not failures,
        {
            "artifact_kind": report.get("artifact_kind"),
            "classification": report.get("classification"),
            "failures": failures,
        },
    )


def _validate_no_rerun_guards(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    for flag in REQUIRED_GUARD_FALSE_FLAGS:
        if report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for flag in OPTIONAL_FALSE_FLAGS:
        if flag in report and report.get(flag) is not False:
            failures[flag] = report.get(flag)
    for key, value in failures.items():
        _add_error(errors, f"stage-output/no-rerun guard {key} must be false, got {value!r}")
    return _check("stage_outputs_independence_and_no_rerun_guards", not failures, {"failures": failures})


def _centerline_nodes_are_historical_only(value: Any, truth: Mapping[str, Any]) -> bool:
    status_text = _normalize_text(
        " ".join(
            str(item)
            for item in [
                value,
                truth.get("centerline_node_status"),
                truth.get("centerline_nodes_status"),
            ]
        )
    )
    historical_markers = [
        "historical only",
        "historical truth only",
        "requires layer1 artifact",
        "requires layer 1 artifact",
        "requires current layer1 artifact",
        "requires current layer 1 artifact",
    ]
    return any(_normalize_text(marker) in status_text for marker in historical_markers)


def _validate_truth(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    truth = _truth(report)
    tokens = _identifier_tokens(truth)
    failures: dict[str, Any] = {}
    connector_present = bool(tokens.intersection(CONNECTOR_IDS))
    route_presence = {route_id: route_id in tokens for route_id in ROUTE_CONTEXT_IDS}
    node_value = truth.get("centerline_nodes")
    if not isinstance(truth, Mapping) or not truth:
        failures["validated_truth_used"] = "missing or not an object"
    if not connector_present:
        failures["connector_id"] = "missing vt_1 or vc_vt_1"
    if truth.get("source_floor") != SOURCE_FLOOR:
        failures["source_floor"] = truth.get("source_floor")
    if truth.get("target_floor") != TARGET_FLOOR:
        failures["target_floor"] = truth.get("target_floor")
    if truth.get("transition_edge") != TRANSITION_EDGE_ID:
        failures["transition_edge"] = truth.get("transition_edge")
    if truth.get("non_transition_edge") != NON_TRANSITION_EDGE_ID:
        failures["non_transition_edge"] = truth.get("non_transition_edge")
    missing_routes = [route_id for route_id, present in route_presence.items() if not present]
    if missing_routes:
        failures["route_context"] = {"missing": missing_routes}
    node_status = "not_included"
    if node_value is not None:
        node_status = "historical_or_layer1_required"
        if not _centerline_nodes_are_historical_only(node_value, truth):
            node_tokens = _identifier_tokens(node_value)
            missing_nodes = [node_id for node_id in CENTERLINE_NODE_IDS if node_id not in node_tokens]
            node_status = "validated"
            if missing_nodes:
                failures["centerline_nodes"] = {"missing": missing_nodes}
    for key, value in failures.items():
        _add_error(errors, f"validated connector truth check failed for {key}: {value!r}")
    return _check(
        "validated_connector_truth",
        not failures,
        {
            "connector_present": connector_present,
            "required_route_context_present": route_presence,
            "centerline_node_status": node_status,
            "failures": failures,
        },
    )


def _preview_name(preview: Mapping[str, Any]) -> str | None:
    for key in ["future_artifact_name", "artifact_name", "name", "filename"]:
        value = preview.get(key)
        if isinstance(value, str) and value.endswith(".json"):
            return Path(value).name
    for key in ["future_generated_output_path", "future_output_path", "expected_output_path"]:
        value = preview.get(key)
        if isinstance(value, str) and value.endswith(".json"):
            return Path(value).name
    return None


def _has_finalization_requirement(preview: Mapping[str, Any]) -> bool:
    if preview.get("requires_layer1_or_current_formal_artifacts_before_finalization") is True:
        return True
    for key, value in preview.items():
        if "requires" in str(key).lower() and "finalization" in str(key).lower() and value is True:
            return True
    return "before finalization" in _normalize_text(" ".join(_string_values(preview)))


def _has_future_path_marker(preview: Mapping[str, Any]) -> bool:
    if "future_generated_output_path" in preview:
        return True
    for key in preview:
        key_text = _normalize_text(key)
        if "future" in key_text and "path" in key_text:
            return True
    return False


def _validate_previews(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    artifact_kind = str(report.get("artifact_kind"))
    expected_names = EXPECTED_PREVIEWS_BY_KIND.get(artifact_kind, set())
    previews = report.get("planned_artifact_previews")
    failures: list[str] = []
    observed_names: set[str] = set()
    if not isinstance(previews, list):
        failures.append("planned_artifact_previews must be a list")
    else:
        for index, preview in enumerate(previews):
            if not isinstance(preview, Mapping):
                failures.append(f"planned_artifact_previews[{index}] must be an object")
                continue
            name = _preview_name(preview)
            if name:
                observed_names.add(name)
            if preview.get("preview_only") is not True:
                failures.append(f"preview {index} preview_only must be true")
            if preview.get("not_written_as_final_artifact") is not True:
                failures.append(f"preview {index} not_written_as_final_artifact must be true")
            if not _has_finalization_requirement(preview):
                failures.append(f"preview {index} must require Layer 1/current formal artifacts before finalization")
            if not _has_future_path_marker(preview):
                failures.append(f"preview {index} must mark future paths as future output paths")
            if preview.get("existence_checked") is True:
                failures.append(f"preview {index} must not require future path existence")
    missing_names = sorted(expected_names.difference(observed_names))
    if missing_names:
        failures.append(f"missing planned future artifact previews: {missing_names}")
    for failure in failures:
        _add_error(errors, failure)
    return _check(
        "planned_artifact_previews",
        not failures,
        {"artifact_kind": artifact_kind, "expected_names": sorted(expected_names), "observed_names": sorted(observed_names), "failures": failures},
    )


def _validate_blocked_fields(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    blocked = report.get("blocked_fields")
    blocked_text = _normalize_text(" ".join(_string_values(blocked)))
    missing = [term for term in BLOCKED_FIELD_TERMS if _normalize_text(term) not in blocked_text]
    if not isinstance(blocked, list):
        _add_error(errors, "blocked_fields must be a list")
    for term in missing:
        _add_error(errors, f"blocked_fields missing {term!r}")
    return _check("blocked_fields", isinstance(blocked, list) and not missing, {"missing": missing})


def _validate_downstream_plan(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    plan = report.get("downstream_layer3_unblock_plan")
    plan_text = _normalize_text(" ".join(_string_values(plan)))
    room_ok = {dep: _normalize_text(dep) in plan_text for dep in COMMON_LAYER3_DEPENDENCIES}
    object_ok = {dep: _normalize_text(dep) in plan_text for dep in COMMON_LAYER3_DEPENDENCIES + OBJECT_LAYER3_DEPENDENCIES}
    mentions_layer3 = "layer 3" in plan_text or "navigation interface layer" in plan_text
    mentions_unblock = "unblock" in plan_text or "finalization" in plan_text
    failures: dict[str, Any] = {}
    if not isinstance(plan, Mapping):
        failures["downstream_layer3_unblock_plan"] = "missing or not an object"
    if not all(room_ok.values()):
        failures["cross_floor_room_dependencies"] = {key: value for key, value in room_ok.items() if not value}
    if not all(object_ok.values()):
        failures["cross_floor_object_dependencies"] = {key: value for key, value in object_ok.items() if not value}
    if not mentions_layer3 or not mentions_unblock:
        failures["layer3_unblock_explanation"] = {"mentions_layer3": mentions_layer3, "mentions_unblock": mentions_unblock}
    for key, value in failures.items():
        _add_error(errors, f"downstream Layer 3 unblock plan check failed for {key}: {value!r}")
    return _check(
        "downstream_layer3_unblock_plan",
        not failures,
        {"cross_floor_room_dependencies": room_ok, "cross_floor_object_dependencies": object_ok, "failures": failures},
    )


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = " ".join(words[max(0, index - 8) : index])
        if any(marker in prefix for marker in ["not", "no", "without", "never", "unvalidated", "must not", "do not", "does not"]):
            return True
    return False


def _is_claim_path(path: str) -> bool:
    normalized = _normalize_text(path)
    if "not claimed" in normalized or "forbidden world model sources" in normalized:
        return False
    return any(marker in normalized for marker in ["claim", "boundary", "status", "classification", "validation", "generated", "completed", "success"])


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


def _validate_artifact_kind_consistency(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    artifact_kind = str(report.get("artifact_kind"))
    failures: dict[str, Any] = {}
    if artifact_kind == "vertical_connectors" and report.get("cross_floor_topology_generated") is not False:
        failures["cross_floor_topology_generated"] = report.get("cross_floor_topology_generated")
    if artifact_kind == "connector_graph" and report.get("vertical_connector_geometry_generated") is not False:
        failures["vertical_connector_geometry_generated"] = report.get("vertical_connector_geometry_generated")
    if artifact_kind == "cross_floor_topology":
        for flag in ["stable_map_generated", "real_route_generated", "runtime_artifacts_generated"]:
            if report.get(flag) is not False:
                failures[flag] = report.get(flag)
    if artifact_kind == "all":
        for flag in REQUIRED_FALSE_FLAGS:
            if report.get(flag) is not False:
                failures[flag] = report.get(flag)
    if artifact_kind not in EXPECTED_PREVIEWS_BY_KIND:
        failures["artifact_kind"] = artifact_kind
    for key, value in failures.items():
        _add_error(errors, f"artifact-kind consistency failed for {key}: {value!r}")
    return _check("artifact_kind_consistency", not failures, {"artifact_kind": artifact_kind, "failures": failures})


def validate_vertical_connector_report(report: Mapping[str, Any], input_path: str | Path = "<memory>") -> dict[str, Any]:
    """Validate one loaded vertical connector/topology dry-run report."""
    errors: list[str] = []
    warnings: list[str] = []
    checks = [
        _validate_identity_and_layer(report, errors),
        _validate_no_rerun_guards(report, errors),
        _validate_truth(report, errors),
        _validate_previews(report, errors),
        _validate_blocked_fields(report, errors),
        _validate_downstream_plan(report, errors),
        _validate_forbidden_claims(report, errors),
        _validate_artifact_kind_consistency(report, errors),
    ]
    ok = not errors
    return {
        "input_json": Path(input_path).as_posix(),
        "ok": ok,
        "classification": PASS_CLASSIFICATION if ok else FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_kind": report.get("artifact_kind"),
        "dry_run_classification": report.get("classification"),
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
    }


def _validate_candidate_identity_and_layer(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    classification = str(report.get("classification"))
    expected_type = EXPECTED_CANDIDATE_TYPE_FIELDS.get(classification, {})
    required_fields = [
        "schema_name",
        "schema_version",
        "classification",
        "artifact_layer",
        "artifact_role",
        "scene_id",
        "is_candidate",
        "is_final_formal_artifact",
        "generated_from_validated_milestones",
        "generated_from_current_layer1_outputs",
        "historical_stage_outputs_required",
        "stage_outputs_required",
        "world_model_rerun",
        "runtime_launched",
        "old_scripts_called",
        "business_logic_migrated",
        "real_geometry_regenerated",
        "real_topology_regenerated",
        "stable_map_generated",
        "object_interface_generated",
        "real_route_generated",
        "runtime_artifacts_generated",
        "validated_truth_lineage",
        "candidate_payload",
        "blocked_until_layer1_or_layer2_inputs",
        "downstream_layer3_unblock_role",
        "claim_boundary",
    ]
    for field in required_fields:
        if field not in report:
            failures[field] = "missing"
    if report.get("schema_version") != "0.1":
        failures["schema_version"] = report.get("schema_version")
    if report.get("artifact_layer") != ARTIFACT_LAYER:
        failures["artifact_layer"] = report.get("artifact_layer")
    if classification not in ALLOWED_CANDIDATE_CLASSIFICATIONS:
        failures["classification"] = report.get("classification")
    if expected_type and report.get("schema_name") != expected_type["schema_name"]:
        failures["schema_name"] = report.get("schema_name")
    if expected_type and report.get("artifact_role") != expected_type["artifact_role"]:
        failures["artifact_role"] = report.get("artifact_role")
    true_flags = ["is_candidate", "generated_from_validated_milestones"]
    for flag in true_flags:
        if report.get(flag) is not True:
            failures[flag] = report.get(flag)
    false_flags = [
        "is_final_formal_artifact",
        "generated_from_current_layer1_outputs",
        "historical_stage_outputs_required",
        "stage_outputs_required",
        "world_model_rerun",
        "runtime_launched",
        "old_scripts_called",
        "business_logic_migrated",
        "real_geometry_regenerated",
        "real_topology_regenerated",
        "stable_map_generated",
        "object_interface_generated",
        "real_route_generated",
        "runtime_artifacts_generated",
    ]
    for flag in false_flags:
        if report.get(flag) is not False:
            failures[flag] = report.get(flag)
    if not isinstance(report.get("validated_truth_lineage"), Mapping):
        failures["validated_truth_lineage"] = type(report.get("validated_truth_lineage")).__name__
    if not isinstance(report.get("candidate_payload"), Mapping):
        failures["candidate_payload"] = type(report.get("candidate_payload")).__name__
    if not isinstance(report.get("blocked_until_layer1_or_layer2_inputs"), list):
        failures["blocked_until_layer1_or_layer2_inputs"] = type(report.get("blocked_until_layer1_or_layer2_inputs")).__name__
    if not isinstance(report.get("downstream_layer3_unblock_role"), Mapping):
        failures["downstream_layer3_unblock_role"] = type(report.get("downstream_layer3_unblock_role")).__name__
    if not isinstance(report.get("claim_boundary"), Mapping):
        failures["claim_boundary"] = type(report.get("claim_boundary")).__name__
    for key, value in failures.items():
        _add_error(errors, f"candidate identity/layer field {key} has invalid value {value!r}")
    return _check(
        "candidate_identity_and_layer_metadata",
        not failures,
        {"classification": classification, "failures": failures},
    )


def _validate_candidate_truth(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    tokens = _identifier_tokens({"lineage": _candidate_lineage(report), "payload": _candidate_payload(report)})
    connector_present = bool(tokens.intersection(CONNECTOR_IDS))
    route_presence = {route_id: route_id in tokens for route_id in ROUTE_CONTEXT_IDS}
    source_floor = _candidate_value(report, [["source_floor"], ["floor_transition", "from"]])
    target_floor = _candidate_value(report, [["target_floor"], ["floor_transition", "to"]])
    transition_edge = _candidate_value(report, [["transition_edge"], ["transition_edge_reference"], ["floor_transition", "transition_edge"]])
    non_transition_edge = _candidate_value(report, [["non_transition_edge"], ["non_transition_edge_reference"]])
    if not connector_present:
        failures["connector_id"] = "missing vt_1 or vc_vt_1"
    if source_floor != SOURCE_FLOOR:
        failures["source_floor"] = source_floor
    if target_floor != TARGET_FLOOR:
        failures["target_floor"] = target_floor
    if transition_edge != TRANSITION_EDGE_ID:
        failures["transition_edge"] = transition_edge
    if non_transition_edge != NON_TRANSITION_EDGE_ID:
        failures["non_transition_edge"] = non_transition_edge
    missing_routes = [route_id for route_id, present in route_presence.items() if not present]
    if missing_routes:
        failures["route_context"] = {"missing": missing_routes}
    for key, value in failures.items():
        _add_error(errors, f"candidate validated truth check failed for {key}: {value!r}")
    return _check(
        "candidate_validated_truth",
        not failures,
        {
            "connector_present": connector_present,
            "required_route_context_present": route_presence,
            "failures": failures,
        },
    )


def _node_ids_from_payload(payload: Mapping[str, Any]) -> set[str]:
    ids: set[str] = set()
    for key in ["centerline_node_ids", "centerline_nodes", "nodes"]:
        value = payload.get(key)
        if isinstance(value, list):
            for item in value:
                if isinstance(item, str):
                    ids.add(item)
                elif isinstance(item, Mapping) and isinstance(item.get("node_id"), str):
                    ids.add(str(item["node_id"]))
    return ids


def _edge_record(payload: Mapping[str, Any], edge_id: str) -> Mapping[str, Any] | None:
    for item in payload.get("edges", []):
        if isinstance(item, Mapping) and item.get("edge_id") == edge_id:
            return item
    return None


def _validate_candidate_payload_by_type(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    payload = _candidate_payload(report)
    classification = report.get("classification")
    failures: dict[str, Any] = {}
    geometry_status = _normalize_text(
        str(payload.get("geometry_status", ""))
        + " "
        + str(_candidate_lineage(report).get("geometry_status", ""))
    )
    if "candidate topology only no regenerated geometry" not in geometry_status:
        failures["geometry_status"] = payload.get("geometry_status")

    if classification == "vertical_connector_candidate_artifact_generated":
        if payload.get("connector_id") not in CONNECTOR_IDS and payload.get("connector_id_alias") not in CONNECTOR_IDS:
            failures["connector_id"] = payload.get("connector_id")
        type_text = _normalize_text(" ".join(_string_values([payload.get("connector_family"), payload.get("connector_type")])))
        if "vertical connector" not in type_text or "stair connector candidate" not in type_text:
            failures["connector_family_or_type"] = type_text
        missing_nodes = sorted(set(CENTERLINE_NODE_IDS).difference(_node_ids_from_payload(payload)))
        if missing_nodes:
            failures["centerline_node_ids"] = {"missing": missing_nodes}
        if payload.get("coordinates_available_from_current_active_manifests") is not False:
            failures["coordinates_available_from_current_active_manifests"] = payload.get(
                "coordinates_available_from_current_active_manifests"
            )
    elif classification == "connector_graph_candidate_artifact_generated":
        if payload.get("graph_id") != "vc_graph_candidate_v0_1":
            failures["graph_id"] = payload.get("graph_id")
        missing_nodes = sorted(set(CENTERLINE_NODE_IDS).difference(_node_ids_from_payload(payload)))
        if missing_nodes:
            failures["nodes"] = {"missing": missing_nodes}
        transition_edge = _edge_record(payload, TRANSITION_EDGE_ID)
        non_transition_edge = _edge_record(payload, NON_TRANSITION_EDGE_ID)
        if not transition_edge or transition_edge.get("is_transition_edge") is not True:
            failures["transition_edge_record"] = transition_edge
        if not non_transition_edge or non_transition_edge.get("is_transition_edge") is not False:
            failures["non_transition_edge_record"] = non_transition_edge
        graph_status = payload.get("graph_status")
        if not isinstance(graph_status, Mapping) or graph_status.get("candidate_graph") is not True:
            failures["candidate_graph"] = graph_status
        if not isinstance(graph_status, Mapping) or graph_status.get("not_final_route_planner_graph") is not True:
            failures["graph_status"] = payload.get("graph_status")
    elif classification == "cross_floor_topology_candidate_artifact_generated":
        if payload.get("topology_id") != "cross_floor_topology_candidate_v0_1":
            failures["topology_id"] = payload.get("topology_id")
        route_context_tokens = _identifier_tokens(payload.get("route_context"))
        missing_context = [item for item in ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"] if item not in route_context_tokens]
        if missing_context:
            failures["route_context"] = {"missing": missing_context}
        candidate_status = payload.get("candidate_status")
        expected_status_flags = [
            "candidate_topology",
            "not_final_cross_floor_topology",
            "not_route_planner_ready_final_topology",
        ]
        for marker in expected_status_flags:
            if not isinstance(candidate_status, Mapping) or candidate_status.get(marker) is not True:
                failures.setdefault("candidate_status", []).append(marker)
        if _candidate_value(report, [["a_star_waypoints_generated"], ["candidate_status", "a_star_waypoints_generated"]]) is not False:
            failures["a_star_waypoints_generated"] = _candidate_value(
                report, [["a_star_waypoints_generated"], ["candidate_status", "a_star_waypoints_generated"]]
            )
        if _candidate_value(report, [["executable_route_generated"], ["candidate_status", "executable_route_generated"]]) is not False:
            failures["executable_route_generated"] = _candidate_value(
                report, [["executable_route_generated"], ["candidate_status", "executable_route_generated"]]
            )
    else:
        failures["classification"] = classification

    for key, value in failures.items():
        _add_error(errors, f"candidate payload check failed for {key}: {value!r}")
    return _check("candidate_payload_by_type", not failures, {"classification": classification, "failures": failures})


def _validate_candidate_blocked_and_downstream(report: Mapping[str, Any], errors: list[str]) -> dict[str, Any]:
    blocked = report.get("blocked_until_layer1_or_layer2_inputs")
    blocked_text = _normalize_text(" ".join(_string_values(blocked)))
    plan = report.get("downstream_layer3_unblock_role")
    plan_text = _normalize_text(" ".join(_string_values(plan)))
    failures: dict[str, Any] = {}
    missing_blocked = [term for term in BLOCKED_FIELD_TERMS if _normalize_text(term) not in blocked_text]
    if not isinstance(blocked, list):
        failures["blocked_until_layer1_or_layer2_inputs"] = type(blocked).__name__
    if missing_blocked:
        failures["blocked_terms"] = missing_blocked
    for dependency in COMMON_LAYER3_DEPENDENCIES + OBJECT_LAYER3_DEPENDENCIES:
        if _normalize_text(dependency) not in plan_text:
            failures.setdefault("downstream_layer3_unblock_role", []).append(dependency)
    for key, value in failures.items():
        _add_error(errors, f"candidate blocked/downstream check failed for {key}: {value!r}")
    return _check("candidate_blocked_fields_and_downstream_role", not failures, {"failures": failures})


def validate_candidate_artifact(report: Mapping[str, Any], input_path: str | Path = "<memory>") -> dict[str, Any]:
    """Validate one loaded vertical connector/topology candidate artifact."""
    errors: list[str] = []
    warnings: list[str] = []
    checks = [
        _validate_candidate_identity_and_layer(report, errors),
        _validate_candidate_truth(report, errors),
        _validate_candidate_payload_by_type(report, errors),
        _validate_candidate_blocked_and_downstream(report, errors),
        _validate_forbidden_claims(report, errors),
    ]
    ok = not errors
    return {
        "input_json": Path(input_path).as_posix(),
        "ok": ok,
        "classification": PASS_CLASSIFICATION if ok else FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_kind": report.get("artifact_role"),
        "dry_run_classification": None,
        "candidate_artifact_classification": report.get("classification"),
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
    }


def _detect_mode(data: Mapping[str, Any], requested_mode: str) -> str:
    if requested_mode in {"dry-run-report", "candidate-artifact"}:
        return requested_mode
    if data.get("is_candidate") is True or "candidate_payload" in data:
        return "candidate-artifact"
    return "dry-run-report"


def validate_vertical_connector_reports(input_jsons: Iterable[str | Path], mode: str = "auto") -> dict[str, Any]:
    """Load and validate one or more vertical connector dry-run reports or candidate artifacts."""
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
                    "artifact_kind": None,
                    "dry_run_classification": None,
                    "candidate_artifact_classification": None,
                    "checks": [_check("json_load", False, {"error": str(exc)})],
                    "errors": [str(exc)],
                    "warnings": [],
                }
            )
            continue
        if not isinstance(data, Mapping):
            message = "vertical connector schema input JSON must be an object"
            file_reports.append(
                {
                    "input_json": path.as_posix(),
                    "ok": False,
                    "classification": FAIL_CLASSIFICATION,
                    "error_count": 1,
                    "warning_count": 0,
                    "artifact_kind": None,
                    "dry_run_classification": None,
                    "candidate_artifact_classification": None,
                    "checks": [_check("json_object", False, {"actual_type": type(data).__name__})],
                    "errors": [message],
                    "warnings": [],
                }
            )
            continue
        detected_mode = _detect_mode(data, mode)
        if detected_mode == "candidate-artifact":
            file_reports.append(validate_candidate_artifact(data, path))
        else:
            file_reports.append(validate_vertical_connector_report(data, path))

    error_count = sum(int(report.get("error_count", 0) or 0) for report in file_reports)
    warning_count = sum(int(report.get("warning_count", 0) or 0) for report in file_reports)
    all_ok = error_count == 0 and not load_errors and bool(file_reports)
    checks = [
        _check("input_json_files_loaded", not load_errors and bool(file_reports), {"input_count": len(file_reports), "load_errors": load_errors}),
        _check("all_vertical_connector_schema_reports_passed", all(report.get("ok") for report in file_reports) and not load_errors, {"file_count": len(file_reports)}),
    ]
    return {
        "classification": PASS_CLASSIFICATION if all_ok else FAIL_CLASSIFICATION,
        "ok": all_ok,
        "error_count": error_count,
        "warning_count": warning_count,
        "validated_files": [
            {
                "input_json": report["input_json"],
                "ok": report["ok"],
                "classification": report["classification"],
                "artifact_kind": report.get("artifact_kind"),
                "dry_run_classification": report.get("dry_run_classification"),
                "candidate_artifact_classification": report.get("candidate_artifact_classification"),
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
        "mode": mode,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate RSLG-SLAM Layer 2 vertical connector/topology dry-run reports and candidate artifacts."
    )
    parser.add_argument(
        "--input-json",
        action="append",
        required=True,
        help="Vertical connector/topology dry-run report JSON to validate. May be repeated.",
    )
    parser.add_argument(
        "--mode",
        default="auto",
        choices=["dry-run-report", "candidate-artifact", "auto"],
        help="Validation mode. Default auto detects each input.",
    )
    parser.add_argument("--output-json", required=True, help="Write the validation report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    report = validate_vertical_connector_reports(args.input_json, mode=args.mode)
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
