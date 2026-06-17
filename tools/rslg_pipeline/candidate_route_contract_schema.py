"""Dry-run schema validation for future RSLG-SLAM candidate route contracts.

This validator consumes promotion dry-run reports from
``tools.rslg_pipeline.route_contract_promotion`` and checks that their
``candidate_contract_preview`` sections are compatible with the expected
future candidate route contract schema. It never writes final candidate route
contracts, generates A* routes, requires historical stage_outputs, calls old
scripts, reruns the World Model Layer, or launches runtime systems.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, load_json, save_json


PASS_CLASSIFICATION = "candidate_route_contract_schema_dry_run_passed"
FAIL_CLASSIFICATION = "candidate_route_contract_schema_dry_run_failed"
CONTRACT_PASS_CLASSIFICATION = "candidate_route_contract_schema_validation_passed"
CONTRACT_FAIL_CLASSIFICATION = "candidate_route_contract_schema_validation_failed"
INVALID_PROMOTION_REPORT_CLASSIFICATION = (
    "candidate_route_contract_schema_dry_run_blocked_promotion_report_invalid"
)
NO_CANDIDATE_PREVIEW_CLASSIFICATION = "candidate_route_contract_schema_dry_run_blocked_no_candidate_preview"

PROMOTION_READY_CLASSIFICATION = "route_contract_candidate_promotion_dry_run_ready"

ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"
ARTIFACT_ROLE = "candidate_route_contract_schema_dry_run"
SCHEMA_VALIDATION_SCOPE = "candidate_contract_preview_only"

FUTURE_SCHEMA_NAME = "rslg_candidate_route_contract"
FUTURE_SCHEMA_VERSION = "0.1"
FUTURE_ARTIFACT_ROLE = "candidate_route_contract"

READY_BUT_BLOCKED = "preview_schema_ready_but_finalization_blocked_by_layer2_artifacts"
BLOCKED_MISSING_TRUTH = "preview_schema_blocked_missing_required_truth"
BLOCKED_INVALID_CLAIM_BOUNDARY = "preview_schema_blocked_invalid_claim_boundary"

ALLOWED_SCHEMA_READINESS = {
    READY_BUT_BLOCKED,
    BLOCKED_MISSING_TRUTH,
    BLOCKED_INVALID_CLAIM_BOUNDARY,
}

REQUIRED_ROUTE_IDS = ["room_2", "room_3", "room_7", "room_13", "room_14"]
CONNECTOR_IDS = {"vt_1", "vc_vt_1"}
TRANSITION_EDGE_ID = "vt_1_centerline_e001"
NOT_TRANSITION_EDGE_ID = "vt_1_centerline_e003"

OBJECT_EXPECTED = {
    "object_id": "obj_175",
    "object_label": "curtain",
    "target_floor": "floor_2",
    "target_room": "room_14",
    "approach_candidate": "generated_ring_037",
    "object_centroid_navigation_used": False,
}

COMMON_FUTURE_LAYER2_DEPENDENCIES = [
    "stable occupancy map package",
    "vertical connector artifact",
    "cross-floor topology artifact",
]

OBJECT_FUTURE_LAYER2_DEPENDENCIES = [
    "object query resolution artifact",
    "object approach candidate artifact",
]

ROOM_DEPENDENCY_KEYS = {"vertical_connector", "connector_graph", "cross_floor_topology", "stable_map_package"}
OBJECT_DEPENDENCY_KEYS = ROOM_DEPENDENCY_KEYS | {
    "object_query_resolution",
    "object_approach",
    "object_interface_package",
}

ACTUAL_REQUIRED_FALSE_FLAGS = [
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "final_layer2_artifacts_generated",
    "final_layer3_route_contracts_generated",
    "real_astar_route_generated",
    "executable_route_generated",
    "runtime_input_package_generated",
    "runtime_artifacts_generated",
    "map_pixels_generated",
    "connector_geometry_regenerated",
    "object_approach_geometry_regenerated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
]

ACTUAL_FORBIDDEN_TRUE_FLAGS = [
    "final_route_contract_generated",
    "real_astar_route_generated",
    "executable_route_generated",
    "runtime_input_package_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "object_navigation_runtime_completed",
    "amcl_success",
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "final_stable_map_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
]

FUTURE_LAYER2_DEPENDENCIES = COMMON_FUTURE_LAYER2_DEPENDENCIES + OBJECT_FUTURE_LAYER2_DEPENDENCIES

COMMON_BLOCKED_LAYER2_FIELDS = [
    {
        "field": "real A* waypoint list",
        "blocked_reason": "requires stable occupancy map package and actual route generation",
    },
    {
        "field": "verified stable occupancy map reference",
        "blocked_reason": "requires verified stable occupancy map artifact from Layer 2",
    },
    {
        "field": "executable route waypoints",
        "blocked_reason": "requires finalized route generation against formal artifacts",
    },
    {
        "field": "verified vertical connector geometry from current formal artifacts",
        "blocked_reason": "requires current Layer 2 vertical connector artifact",
    },
    {
        "field": "verified cross-floor topology artifact from current formal artifacts",
        "blocked_reason": "requires current Layer 2 cross-floor topology artifact",
    },
    {
        "field": "collision validation",
        "blocked_reason": "requires generated route geometry and map validation",
    },
    {
        "field": "runtime trajectory",
        "blocked_reason": "requires a separate Layer 4 runtime validation task",
    },
]

OBJECT_BLOCKED_LAYER2_FIELDS = [
    {
        "field": "verified object query resolution artifact",
        "blocked_reason": "requires current Layer 2 object query resolution artifact",
    },
    {
        "field": "verified object approach candidate artifact",
        "blocked_reason": "requires current Layer 2 object approach candidate artifact",
    },
    {
        "field": "final object approach route",
        "blocked_reason": "requires finalized object approach geometry and route generation",
    },
    {
        "field": "object-facing approach validation",
        "blocked_reason": "requires object approach route validation in a separate validation task",
    },
]

BLOCKED_LAYER2_FIELDS = COMMON_BLOCKED_LAYER2_FIELDS + OBJECT_BLOCKED_LAYER2_FIELDS

FORBIDDEN_TRUE_FLAGS = [
    "final_route_contract_generated",
    "candidate_contract_generated",
    "real_astar_route_generated",
    "executable_route_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "footstep_planning_completed",
    "gait_control_completed",
    "contact_dynamics_handled",
    "dense_reconstruction_completed",
    "full_object_navigation_benchmark_completed",
    "amcl_success",
]

FORBIDDEN_CLAIM_PHRASES = [
    "final route contract generated",
    "real A* route generated",
    "executable route generated",
    "runtime validation completed",
    "Gazebo RViz Nav2 validation completed",
    "Gazebo/RViz/Nav2 validation completed",
    "physical stair climbing completed",
    "real robot stair climbing completed",
    "footstep planning completed",
    "gait control completed",
    "contact dynamics completed",
    "contact dynamics handled",
    "dense reconstruction completed",
    "full object-navigation benchmark completed",
    "full object navigation benchmark completed",
    "AMCL success",
]

REQUIRED_FALSE_TOP_LEVEL_FLAGS = [
    "candidate_contract_generated",
    "stage_outputs_required",
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
]

OPTIONAL_FALSE_TOP_LEVEL_FLAGS = [
    "is_final_route_contract",
    "final_route_contract_generated",
    "is_final_navigation_artifact",
]

REQUIRED_FALSE_PREVIEW_FLAGS = [
    "candidate_contract_generated",
    "is_final_route_contract",
]


def _load_json_object(path: str | Path) -> Mapping[str, Any]:
    target = Path(path).expanduser().resolve()
    with target.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, Mapping):
        raise ValueError("promotion report JSON must be an object")
    return data


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
    for char in "-_/.,;:()[]{}\"'":
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


def _field_item(field: str, value: Any, source: str) -> dict[str, Any]:
    return {"field": field, "value": value, "source": source}


def _blocked_item(field: str, reason: str, dependency: str | None = None) -> dict[str, Any]:
    item: dict[str, Any] = {
        "field": field,
        "status": "blocked_until_real_layer2_artifacts_exist",
        "blocked_reason": reason,
    }
    if dependency:
        item["future_dependency"] = dependency
    return item


def _find_promotable_value(item: Mapping[str, Any], field: str) -> Any:
    for record in item.get("promotable_fields", []):
        if isinstance(record, Mapping) and record.get("field") == field:
            return record.get("value")
    return None


def _candidate_kind_from_stub_type(stub_type: str) -> str:
    return {
        "cross_floor_room": "room_level_cross_floor_candidate_route_contract",
        "cross_floor_object": "object_level_cross_floor_candidate_route_contract",
    }.get(stub_type, "unsupported_route_contract_candidate")


def _validate_report_header(report: Mapping[str, Any], source: Path) -> list[str]:
    errors: list[str] = []
    if report.get("project_name") != PROJECT_NAME:
        errors.append(f"{source.as_posix()}: project_name must be {PROJECT_NAME!r}")
    if report.get("classification") != PROMOTION_READY_CLASSIFICATION:
        errors.append(
            f"{source.as_posix()}: classification must be {PROMOTION_READY_CLASSIFICATION!r}"
        )
    if int(report.get("error_count", 0) or 0) != 0:
        errors.append(f"{source.as_posix()}: promotion report error_count must be 0")
    if report.get("dry_run") is not True:
        errors.append(f"{source.as_posix()}: dry_run must be true")
    for flag in REQUIRED_FALSE_TOP_LEVEL_FLAGS:
        if report.get(flag) is not False:
            errors.append(f"{source.as_posix()}: {flag} must be false")
    for flag in OPTIONAL_FALSE_TOP_LEVEL_FLAGS:
        if flag in report and report.get(flag) is not False:
            errors.append(f"{source.as_posix()}: {flag} must be false when present")
    if not isinstance(report.get("promotion_items"), list):
        errors.append(f"{source.as_posix()}: promotion_items must be a list")
    return errors


def _validate_preview_flags(preview: Mapping[str, Any], errors: list[str]) -> None:
    required_true = {
        "preview_only": True,
        "not_written_as_final_artifact": True,
        "requires_layer2_artifacts_before_finalization": True,
    }
    for key, expected in required_true.items():
        if preview.get(key) is not expected:
            errors.append(f"candidate preview field {key} must be {expected!r}")
    for key in REQUIRED_FALSE_PREVIEW_FLAGS:
        if preview.get(key) is not False:
            errors.append(f"candidate preview field {key} must be false")


def _validate_route_truth(preview: Mapping[str, Any], errors: list[str]) -> list[dict[str, Any]]:
    route_sequence = preview.get("route_sequence")
    tokens = _extract_identifier_tokens(route_sequence)
    satisfied: list[dict[str, Any]] = []
    if not isinstance(route_sequence, list) or not route_sequence:
        errors.append("candidate preview route_sequence must be a non-empty list")
    route_presence = {route_id: route_id in tokens for route_id in REQUIRED_ROUTE_IDS}
    for route_id, present in route_presence.items():
        if present:
            satisfied.append(_field_item(f"route_sequence includes {route_id}", True, "candidate_contract_preview"))
        else:
            errors.append(f"candidate preview route_sequence is missing {route_id}")
    connector_present = bool(tokens.intersection(CONNECTOR_IDS))
    if connector_present:
        satisfied.append(
            _field_item("route_sequence includes vt_1 or vc_vt_1", True, "candidate_contract_preview")
        )
    else:
        errors.append("candidate preview route_sequence is missing vt_1 or vc_vt_1")

    transition = preview.get("transition_edge_truth", {})
    if not isinstance(transition, Mapping):
        errors.append("candidate preview transition_edge_truth must be an object")
        return satisfied
    if transition.get("transition_edge") == TRANSITION_EDGE_ID:
        satisfied.append(_field_item("transition_edge", TRANSITION_EDGE_ID, "candidate_contract_preview"))
    else:
        errors.append(f"transition_edge must be {TRANSITION_EDGE_ID!r}")
    if transition.get("not_transition_edge") == NOT_TRANSITION_EDGE_ID:
        satisfied.append(
            _field_item("not_transition_edge", NOT_TRANSITION_EDGE_ID, "candidate_contract_preview")
        )
    else:
        errors.append(f"not_transition_edge must be {NOT_TRANSITION_EDGE_ID!r}")
    return satisfied


def _validate_object_truth(preview: Mapping[str, Any], errors: list[str]) -> list[dict[str, Any]]:
    binding = preview.get("object_binding")
    satisfied: list[dict[str, Any]] = []
    if not isinstance(binding, Mapping):
        errors.append("cross-floor object candidate preview must include object_binding")
        return satisfied
    query_ok = "curtain" in _normalize_text(binding.get("query", ""))
    if query_ok:
        satisfied.append(_field_item("query contains curtain", binding.get("query"), "object_binding"))
    else:
        errors.append("object_binding query must contain curtain")
    for key, expected in OBJECT_EXPECTED.items():
        if binding.get(key) == expected:
            satisfied.append(_field_item(key, expected, "object_binding"))
        else:
            errors.append(f"object_binding {key} must be {expected!r}")
    return satisfied


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix_text = " ".join(words[max(0, index - 7) : index])
        if any(
            marker in prefix_text
            for marker in ["not", "no", "without", "never", "unvalidated", "must not", "do not", "does not"]
        ):
            return True
    return False


def _validate_claim_boundary(values: list[Any]) -> tuple[list[dict[str, Any]], list[str]]:
    errors: list[str] = []
    satisfied: list[dict[str, Any]] = []
    combined = values
    unsafe_flags: list[dict[str, Any]] = []
    positive_phrase_hits: list[dict[str, Any]] = []
    for value in combined:
        for path, item in _walk(value):
            leaf = path.rsplit(".", 1)[-1]
            if leaf in FORBIDDEN_TRUE_FLAGS and item is True:
                unsafe_flags.append({"path": path, "flag": leaf})
            if isinstance(item, str):
                text = _normalize_text(item)
                for phrase in FORBIDDEN_CLAIM_PHRASES:
                    normalized = _normalize_text(phrase)
                    if normalized in text and not _phrase_is_negated(text, normalized):
                        positive_phrase_hits.append({"path": path, "phrase": phrase, "value": item})
    for hit in unsafe_flags:
        errors.append(f"forbidden positive claim flag {hit['flag']} is true at {hit['path']}")
    for hit in positive_phrase_hits:
        errors.append(f"forbidden positive claim phrase {hit['phrase']!r} found at {hit['path']}")
    claim_fields = [
        "final route contract generated",
        "real A* route generated",
        "executable route generated",
        "runtime validation completed",
        "Gazebo/RViz/Nav2 validation completed",
        "physical stair climbing completed",
        "real robot stair climbing completed",
        "footstep/gait/contact planning completed",
        "dense reconstruction completed",
        "full object-navigation benchmark completed",
        "AMCL success",
    ]
    if not unsafe_flags and not positive_phrase_hits:
        satisfied.extend(_field_item(f"does not claim {field}", False, "claim_boundary") for field in claim_fields)
    return satisfied, errors


def _future_required_artifact_names(item: Mapping[str, Any]) -> set[str]:
    names = set()
    for artifact in item.get("future_required_artifacts", []):
        if isinstance(artifact, Mapping) and artifact.get("artifact"):
            names.add(str(artifact.get("artifact")))
    return names


def _blocked_field_names(item: Mapping[str, Any]) -> set[str]:
    fields = set()
    for blocked in item.get("blocked_fields", []):
        if isinstance(blocked, Mapping) and blocked.get("field"):
            fields.add(str(blocked.get("field")))
    return fields


def _future_layer2_dependencies_for_stub_type(stub_type: str) -> list[str]:
    dependencies = list(COMMON_FUTURE_LAYER2_DEPENDENCIES)
    if stub_type == "cross_floor_object":
        dependencies.extend(OBJECT_FUTURE_LAYER2_DEPENDENCIES)
    return dependencies


def _blocked_layer2_fields_for_stub_type(stub_type: str) -> list[dict[str, Any]]:
    fields = list(COMMON_BLOCKED_LAYER2_FIELDS)
    if stub_type == "cross_floor_object":
        fields.extend(OBJECT_BLOCKED_LAYER2_FIELDS)
    return fields


def _dependency_for_blocked_field(field: str) -> str | None:
    if "occupancy map" in field:
        return "stable occupancy map package"
    if "vertical connector" in field:
        return "vertical connector artifact"
    if "cross-floor topology" in field:
        return "cross-floor topology artifact"
    if "object query" in field:
        return "object query resolution artifact"
    if "object approach" in field or "object-facing" in field:
        return "object approach candidate artifact"
    if "waypoint" in field or "collision" in field:
        return "stable occupancy map package"
    return None


def _validate_blocked_dependencies(item: Mapping[str, Any], errors: list[str]) -> list[dict[str, Any]]:
    blocked: list[dict[str, Any]] = []
    source_stub_type = str(item.get("source_stub_type", ""))
    promotion_blocked_names = _blocked_field_names(item)
    if not promotion_blocked_names:
        errors.append("promotion item must include blocked_fields for finalization-only route data")
    for blocked_field in _blocked_layer2_fields_for_stub_type(source_stub_type):
        field = blocked_field["field"]
        blocked.append(_blocked_item(field, blocked_field["blocked_reason"], _dependency_for_blocked_field(field)))
    return blocked


def _validate_candidate_preview(
    *,
    report_path: Path,
    report: Mapping[str, Any],
    item: Mapping[str, Any],
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    preview = item.get("candidate_contract_preview")
    if not isinstance(preview, Mapping):
        errors.append("promotion item has no candidate_contract_preview object")
        preview = {}

    source_stub_type = str(item.get("source_stub_type", ""))
    candidate_route_kind = str(preview.get("candidate_route_kind") or _find_promotable_value(item, "candidate_route_kind"))
    expected_kind = _candidate_kind_from_stub_type(source_stub_type)
    if candidate_route_kind != expected_kind:
        errors.append(f"candidate_route_kind must be {expected_kind!r}")

    if item.get("schema_validation_passed") is not True:
        errors.append("source stub schema validation must have passed before candidate schema dry-run")
    if item.get("artifact_layer") != ARTIFACT_LAYER:
        errors.append(f"promotion item artifact_layer must be {ARTIFACT_LAYER!r}")

    _validate_preview_flags(preview, errors)

    satisfied_schema_fields: list[dict[str, Any]] = [
        _field_item("schema_name", FUTURE_SCHEMA_NAME, "candidate_schema_dry_run_definition"),
        _field_item("schema_version", FUTURE_SCHEMA_VERSION, "candidate_schema_dry_run_definition"),
        _field_item("artifact_layer", ARTIFACT_LAYER, "promotion_item"),
        _field_item("artifact_role", FUTURE_ARTIFACT_ROLE, "future_candidate_schema_definition"),
        _field_item("scene_id", preview.get("scene_id"), "candidate_contract_preview"),
        _field_item("route_kind", candidate_route_kind, "candidate_contract_preview"),
        _field_item("is_final_navigation_artifact", False, "candidate_schema_dry_run_definition"),
        _field_item("requires_finalization", True, "candidate_schema_dry_run_definition"),
        _field_item("preview_only", True, "candidate_contract_preview"),
        _field_item("not_written_as_final_artifact", True, "candidate_contract_preview"),
    ]

    if preview.get("scene_id") != item.get("scene_id"):
        errors.append("candidate preview scene_id must match promotion item scene_id")
    satisfied_schema_fields.extend(_validate_route_truth(preview, errors))
    if source_stub_type == "cross_floor_object":
        satisfied_schema_fields.extend(_validate_object_truth(preview, errors))

    claim_satisfied, claim_errors = _validate_claim_boundary([report, item, preview])
    satisfied_schema_fields.extend(claim_satisfied)
    errors.extend(claim_errors)

    blocked_schema_fields = _validate_blocked_dependencies(item, errors)
    future_required_artifacts = [
        {
            "artifact": name,
            "layer": "Layer 2: Formal Artifact Layer",
            "required_before_finalization": True,
            "present_for_dry_run": name in _future_required_artifact_names(item),
        }
        for name in _future_layer2_dependencies_for_stub_type(source_stub_type)
    ]

    if claim_errors:
        readiness = BLOCKED_INVALID_CLAIM_BOUNDARY
    elif errors:
        readiness = BLOCKED_MISSING_TRUTH
    else:
        readiness = READY_BUT_BLOCKED

    if readiness not in ALLOWED_SCHEMA_READINESS:
        warnings.append(f"unexpected schema_readiness {readiness!r}")

    return {
        "source_promotion_report": report_path.as_posix(),
        "source_stub_type": source_stub_type,
        "candidate_route_kind": candidate_route_kind,
        "scene_id": preview.get("scene_id") or item.get("scene_id"),
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": ARTIFACT_ROLE,
        "schema_validation_scope": SCHEMA_VALIDATION_SCOPE,
        "preview_only": True,
        "not_written_as_final_artifact": True,
        "requires_layer2_artifacts_before_finalization": True,
        "satisfied_schema_fields": satisfied_schema_fields,
        "blocked_schema_fields": blocked_schema_fields,
        "future_required_artifacts": future_required_artifacts,
        "claim_boundary": {
            "dry_run_only": True,
            "candidate_contract_generated": False,
            "final_route_contract_generated": False,
            "real_astar_route_generated": False,
            "executable_route_generated": False,
            "runtime_validation_completed": False,
            "gazebo_rviz_nav2_validation_completed": False,
            "physical_stair_climbing_completed": False,
            "real_robot_stair_climbing_completed": False,
            "footstep_gait_contact_planning_completed": False,
            "dense_reconstruction_completed": False,
            "full_object_navigation_benchmark_completed": False,
            "amcl_success": False,
            "source_claim_boundary": item.get("claim_boundary"),
        },
        "schema_readiness": readiness,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "errors": errors,
        "warnings": warnings,
    }


def _summarize_satisfied_fields(items: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    summary: dict[str, set[str]] = {}
    for item in items:
        source_type = str(item.get("source_stub_type"))
        for field in item.get("satisfied_schema_fields", []):
            if isinstance(field, Mapping):
                summary.setdefault(str(field.get("field")), set()).add(source_type)
    return [{"field": field, "source_stub_types": sorted(types)} for field, types in sorted(summary.items())]


def _summarize_blocked_fields(items: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    summary: dict[str, set[str]] = {}
    for item in items:
        source_type = str(item.get("source_stub_type"))
        for field in item.get("blocked_schema_fields", []):
            if isinstance(field, Mapping):
                summary.setdefault(str(field.get("field")), set()).add(source_type)
    return [{"field": field, "source_stub_types": sorted(types)} for field, types in sorted(summary.items())]


def _summarize_future_dependencies(items: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    summary: dict[str, set[str]] = {}
    for item in items:
        source_type = str(item.get("source_stub_type"))
        for name in _future_layer2_dependencies_for_stub_type(source_type):
            summary.setdefault(name, set())
        for artifact in item.get("future_required_artifacts", []):
            if isinstance(artifact, Mapping):
                summary.setdefault(str(artifact.get("artifact")), set()).add(source_type)
    return [
        {
            "artifact": artifact,
            "layer": "Layer 2: Formal Artifact Layer",
            "required_before_finalization": True,
            "source_stub_types": sorted(types),
        }
        for artifact, types in sorted(summary.items())
    ]


def _dependency_keys(contract: Mapping[str, Any]) -> set[str]:
    keys = set()
    for item in contract.get("layer2_candidate_dependencies", []):
        if isinstance(item, Mapping) and item.get("dependency_key"):
            keys.add(str(item.get("dependency_key")))
    return keys


def _contract_payload(contract: Mapping[str, Any]) -> Mapping[str, Any]:
    payload = contract.get("route_contract_payload", {})
    return payload if isinstance(payload, Mapping) else {}


def _validate_actual_contract(path: Path, contract: Mapping[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    route_kind = contract.get("route_kind")
    payload = _contract_payload(contract)

    if contract.get("artifact_layer") != ARTIFACT_LAYER:
        errors.append(f"{path.as_posix()}: artifact_layer must be {ARTIFACT_LAYER!r}")
    if route_kind not in {"cross_floor_room", "cross_floor_object"}:
        errors.append(f"{path.as_posix()}: route_kind must be cross_floor_room or cross_floor_object")
    for key, expected in {
        "schema_name": FUTURE_SCHEMA_NAME,
        "schema_version": FUTURE_SCHEMA_VERSION,
        "is_candidate": True,
        "is_final_route_contract": False,
        "generated_from_layer2_candidate_artifacts": True,
        "generated_from_final_layer2_artifacts": False,
        "stage_outputs_required": False,
    }.items():
        if contract.get(key) is not expected if isinstance(expected, bool) else contract.get(key) != expected:
            errors.append(f"{path.as_posix()}: {key} must be {expected!r}")
    if contract.get("historical_stage_outputs_required", False) is not False:
        errors.append(f"{path.as_posix()}: historical_stage_outputs_required must be false when present")
    for flag in ACTUAL_REQUIRED_FALSE_FLAGS:
        if contract.get(flag) is not False:
            errors.append(f"{path.as_posix()}: safety flag {flag} must be false")
    for flag in ACTUAL_FORBIDDEN_TRUE_FLAGS:
        if contract.get(flag, False) is True:
            errors.append(f"{path.as_posix()}: forbidden positive claim flag {flag} is true")

    dependency_keys = _dependency_keys(contract)
    if route_kind == "cross_floor_room":
        missing = sorted(ROOM_DEPENDENCY_KEYS - dependency_keys)
        forbidden = sorted((OBJECT_DEPENDENCY_KEYS - ROOM_DEPENDENCY_KEYS) & dependency_keys)
        if missing:
            errors.append(f"{path.as_posix()}: cross_floor_room missing dependency keys {missing}")
        if forbidden:
            errors.append(f"{path.as_posix()}: cross_floor_room must not require object dependency keys {forbidden}")
    elif route_kind == "cross_floor_object":
        missing = sorted(OBJECT_DEPENDENCY_KEYS - dependency_keys)
        if missing:
            errors.append(f"{path.as_posix()}: cross_floor_object missing dependency keys {missing}")

    text = _normalize_text(json.dumps(contract, sort_keys=True))
    raw_text = json.dumps(contract, sort_keys=True)
    if not any(connector_id in raw_text for connector_id in CONNECTOR_IDS):
        errors.append(f"{path.as_posix()}: connector must include vt_1 or vc_vt_1")
    for name, expected in {
        "transition_edge": TRANSITION_EDGE_ID,
        "non_transition_edge": NOT_TRANSITION_EDGE_ID,
        "source_floor": "floor_1",
        "target_floor": "floor_2",
    }.items():
        if expected not in raw_text:
            errors.append(f"{path.as_posix()}: missing {name} {expected!r}")
    for room_id in REQUIRED_ROUTE_IDS:
        if room_id not in raw_text:
            errors.append(f"{path.as_posix()}: route context missing {room_id}")
    if "floor 1" not in text or "floor 2" not in text:
        errors.append(f"{path.as_posix()}: floor transition floor_1 -> floor_2 must be present")

    if route_kind == "cross_floor_object":
        expected_object = {
            "query": "curtain in room_14 on floor_2",
            "object_id": "obj_175",
            "object_label": "curtain",
            "target_floor": "floor_2",
            "target_room": "room_14",
            "approach_candidate_id": "generated_ring_037",
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
            "approach_geometry_regenerated": False,
            "approach_feasibility_revalidated": False,
        }
        for key, expected in expected_object.items():
            actual = payload.get(key)
            if actual != expected:
                errors.append(f"{path.as_posix()}: route_contract_payload.{key} must be {expected!r}")

    claim_satisfied, claim_errors = _validate_claim_boundary([contract])
    errors.extend(f"{path.as_posix()}: {message}" for message in claim_errors)

    return {
        "input_json": path.as_posix(),
        "classification": contract.get("classification"),
        "scene_id": contract.get("scene_id"),
        "route_kind": route_kind,
        "artifact_layer": contract.get("artifact_layer"),
        "dependency_keys": sorted(dependency_keys),
        "dependency_scope_checked": route_kind in {"cross_floor_room", "cross_floor_object"},
        "cross_floor_truth_checked": True,
        "object_truth_checked": route_kind == "cross_floor_object",
        "claim_boundary_checked": True,
        "satisfied_claim_boundary_fields": claim_satisfied,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "errors": errors,
        "warnings": warnings,
    }


def validate_candidate_route_contract_files(input_jsons: Iterable[str | Path]) -> dict[str, Any]:
    paths = [Path(path).expanduser().resolve() for path in input_jsons]
    errors: list[str] = []
    warnings: list[str] = []
    validated_contracts: list[dict[str, Any]] = []
    for path in paths:
        try:
            data = load_json(path)
            if not isinstance(data, Mapping):
                raise ValueError("candidate route contract JSON must be an object")
        except Exception as exc:
            message = f"{path.as_posix()}: {exc}"
            errors.append(message)
            validated_contracts.append(
                {
                    "input_json": path.as_posix(),
                    "ok": False,
                    "error_count": 1,
                    "warning_count": 0,
                    "errors": [message],
                    "warnings": [],
                }
            )
            continue
        item = _validate_actual_contract(path, data)
        validated_contracts.append(item)
        errors.extend(item["errors"])
        warnings.extend(item["warnings"])

    route_kinds = {item.get("route_kind") for item in validated_contracts}
    if paths and {"cross_floor_room", "cross_floor_object"} - route_kinds:
        missing = sorted({"cross_floor_room", "cross_floor_object"} - route_kinds)
        errors.append(f"schema validation input set missing route kinds {missing}")

    return {
        "project_name": PROJECT_NAME,
        "classification": CONTRACT_PASS_CLASSIFICATION if not errors else CONTRACT_FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "schema_validation_scope": "candidate_route_contract_files",
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_route_contract_generated": False,
        "real_astar_route_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "validated_candidate_route_contracts": validated_contracts,
        "route_kind_dependency_scope": {
            "cross_floor_room": {
                "required_layer2_dependency_keys": sorted(ROOM_DEPENDENCY_KEYS),
                "forbidden_layer2_dependency_keys": sorted(OBJECT_DEPENDENCY_KEYS - ROOM_DEPENDENCY_KEYS),
                "dependency_scope_checked": "cross_floor_room" in route_kinds,
            },
            "cross_floor_object": {
                "required_layer2_dependency_keys": sorted(OBJECT_DEPENDENCY_KEYS),
                "forbidden_layer2_dependency_keys": [],
                "dependency_scope_checked": "cross_floor_object" in route_kinds,
            },
        },
        "claim_boundary_summary": {
            "candidate_only": True,
            "final_route_contract_generated": False,
            "real_astar_route_generated": False,
            "executable_route_generated": False,
            "runtime_input_package_generated": False,
            "runtime_validation_completed": False,
            "gazebo_rviz_nav2_validation_completed": False,
            "object_navigation_runtime_completed": False,
            "amcl_success": False,
            "physical_stair_climbing_completed": False,
            "real_robot_stair_climbing_completed": False,
            "final_stable_map_generated": False,
            "pgm_generated": False,
            "map_server_yaml_generated": False,
            "external_gt_map_used": False,
            "simulator_navmesh_used": False,
        },
        "errors": errors,
        "warnings": warnings,
    }


def validate_candidate_route_contract_schema_reports(promotion_reports: Iterable[str | Path]) -> dict[str, Any]:
    report_paths = [Path(path).expanduser().resolve() for path in promotion_reports]
    errors: list[str] = []
    warnings: list[str] = []
    validated_promotion_reports: list[dict[str, Any]] = []
    candidate_schema_items: list[dict[str, Any]] = []
    loaded_reports: list[tuple[Path, Mapping[str, Any]]] = []

    for path in report_paths:
        try:
            report = _load_json_object(path)
            loaded_reports.append((path, report))
        except Exception as exc:
            errors.append(f"{path.as_posix()}: {exc}")
            validated_promotion_reports.append(
                {
                    "promotion_report": path.as_posix(),
                    "ok": False,
                    "classification": None,
                    "error": str(exc),
                }
            )
            continue

        header_errors = _validate_report_header(report, path)
        validated_promotion_reports.append(
            {
                "promotion_report": path.as_posix(),
                "ok": not header_errors,
                "classification": report.get("classification"),
                "error_count": report.get("error_count"),
                "warning_count": report.get("warning_count"),
                "candidate_preview_count": sum(
                    1
                    for item in report.get("promotion_items", [])
                    if isinstance(item, Mapping) and isinstance(item.get("candidate_contract_preview"), Mapping)
                )
                if isinstance(report.get("promotion_items"), list)
                else 0,
                "errors": header_errors,
            }
        )
        errors.extend(header_errors)

    if errors:
        classification = INVALID_PROMOTION_REPORT_CLASSIFICATION
    else:
        for path, report in loaded_reports:
            for item in report.get("promotion_items", []):
                if not isinstance(item, Mapping):
                    errors.append(f"{path.as_posix()}: promotion item must be an object")
                    continue
                if not isinstance(item.get("candidate_contract_preview"), Mapping):
                    continue
                schema_item = _validate_candidate_preview(report_path=path, report=report, item=item)
                candidate_schema_items.append(schema_item)
                errors.extend(schema_item.get("errors", []))
                warnings.extend(schema_item.get("warnings", []))

        if not candidate_schema_items and not errors:
            classification = NO_CANDIDATE_PREVIEW_CLASSIFICATION
            errors.append("no candidate_contract_preview sections found in promotion reports")
        elif errors:
            classification = FAIL_CLASSIFICATION
        else:
            classification = PASS_CLASSIFICATION

    return {
        "project_name": PROJECT_NAME,
        "classification": classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "dry_run": True,
        "candidate_contract_generated": False,
        "final_route_contract_generated": False,
        "is_final_navigation_artifact": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "validated_promotion_reports": validated_promotion_reports,
        "candidate_schema_items": candidate_schema_items,
        "candidate_contract_future_schema": {
            "schema_name": FUTURE_SCHEMA_NAME,
            "schema_version": FUTURE_SCHEMA_VERSION,
            "artifact_layer": ARTIFACT_LAYER,
            "artifact_role": FUTURE_ARTIFACT_ROLE,
            "requires_finalization": True,
            "is_final_navigation_artifact": False,
        },
        "schema_satisfied_fields_summary": _summarize_satisfied_fields(candidate_schema_items),
        "schema_blocked_fields_summary": _summarize_blocked_fields(candidate_schema_items),
        "future_layer2_dependencies": _summarize_future_dependencies(candidate_schema_items),
        "route_kind_dependency_scope": {
            "cross_floor_room": {
                "required_layer2_dependencies": COMMON_FUTURE_LAYER2_DEPENDENCIES,
                "not_applicable_layer2_dependencies": OBJECT_FUTURE_LAYER2_DEPENDENCIES,
            },
            "cross_floor_object": {
                "required_layer2_dependencies": COMMON_FUTURE_LAYER2_DEPENDENCIES
                + OBJECT_FUTURE_LAYER2_DEPENDENCIES,
                "not_applicable_layer2_dependencies": [],
            },
        },
        "claim_boundary_summary": {
            "dry_run_only": True,
            "candidate_contract_generated": False,
            "final_route_contract_generated": False,
            "real_astar_route_generated": False,
            "executable_route_generated": False,
            "runtime_validation_completed": False,
            "gazebo_rviz_nav2_validation_completed": False,
            "physical_stair_climbing_completed": False,
            "real_robot_stair_climbing_completed": False,
            "footstep_gait_contact_planning_completed": False,
            "dense_reconstruction_completed": False,
            "full_object_navigation_benchmark_completed": False,
            "amcl_success": False,
        },
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate RSLG-SLAM candidate route contract previews or generated candidate contract files.",
    )
    parser.add_argument(
        "--promotion-report",
        action="append",
        default=[],
        help="Promotion dry-run report JSON. May be repeated.",
    )
    parser.add_argument(
        "--input-json",
        action="append",
        default=[],
        help="Generated candidate route contract JSON. May be repeated.",
    )
    parser.add_argument("--output-json", required=True, help="Write the candidate schema dry-run report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.input_json and args.promotion_report:
        report = {
            "project_name": PROJECT_NAME,
            "classification": CONTRACT_FAIL_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "errors": ["Pass either --input-json or --promotion-report, not both."],
            "warnings": [],
        }
    elif args.input_json:
        report = validate_candidate_route_contract_files(args.input_json)
    elif args.promotion_report:
        report = validate_candidate_route_contract_schema_reports(args.promotion_report)
    else:
        report = {
            "project_name": PROJECT_NAME,
            "classification": CONTRACT_FAIL_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "errors": ["Pass at least one --input-json or --promotion-report."],
            "warnings": [],
        }
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
        )
    )
    return 0 if report["classification"] in {PASS_CLASSIFICATION, CONTRACT_PASS_CLASSIFICATION} else 1


if __name__ == "__main__":
    raise SystemExit(main())
