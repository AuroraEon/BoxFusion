"""Dry-run promotion reports for RSLG-SLAM route contract stubs.

This module evaluates validated route contract stubs for possible promotion
into future candidate route contracts. It never writes final candidate
contracts, requires historical stage_outputs, calls old route-generation
scripts, reruns the World Model Layer, or launches runtime systems.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .common import PROJECT_NAME, save_json
from .route_contract_schema import ALLOWED_STUB_TYPES, validate_route_contract_stub_files


READY_CLASSIFICATION = "route_contract_candidate_promotion_dry_run_ready"
SCHEMA_FAILED_CLASSIFICATION = "route_contract_candidate_promotion_blocked_schema_validation_failed"
UNSUPPORTED_STUB_TYPE_CLASSIFICATION = "route_contract_candidate_promotion_blocked_unsupported_stub_type"

ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"
ARTIFACT_ROLE = "route_contract_candidate_promotion_dry_run"

COMMON_LAYER2_DEPENDENCIES = [
    "stable occupancy map package",
    "vertical connector artifact",
    "cross-floor topology artifact",
]

OBJECT_LAYER2_DEPENDENCIES = [
    "object query resolution artifact",
    "object approach candidate artifact",
]


def _candidate_kind(stub_type: str) -> str:
    return {
        "cross_floor_room": "room_level_cross_floor_candidate_route_contract",
        "cross_floor_object": "object_level_cross_floor_candidate_route_contract",
    }.get(stub_type, "unsupported_route_contract_candidate")


def _source_manifest_lineage(stub: Mapping[str, Any]) -> dict[str, Any]:
    lineage = {
        "generated_from": stub.get("generated_from"),
        "dry_run_plan_classification": stub.get("dry_run_plan_classification"),
    }
    if "source_manifest_lineage" in stub:
        lineage["source_manifest_lineage"] = stub.get("source_manifest_lineage")
    return lineage


def _schema_identity_lineage(stub: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_name": stub.get("schema_name"),
        "schema_version": stub.get("schema_version"),
        "stub_classification": stub.get("classification"),
        "artifact_layer": stub.get("artifact_layer"),
        "artifact_role": stub.get("artifact_role"),
        "is_stub": stub.get("is_stub"),
        "is_final_navigation_artifact": stub.get("is_final_navigation_artifact"),
    }


def _promotable_fields(stub: Mapping[str, Any]) -> list[dict[str, Any]]:
    stub_type = str(stub.get("stub_type", ""))
    fields = [
        {"field": "scene_id", "value": stub.get("scene_id"), "promotion_status": "safe_from_validated_stub"},
        {
            "field": "candidate_route_kind",
            "source_field": "stub_type",
            "source_value": stub_type,
            "value": _candidate_kind(stub_type),
            "promotion_status": "safe_from_validated_stub",
        },
        {
            "field": "route_sequence",
            "value": stub.get("route_sequence", []),
            "promotion_status": "safe_from_validated_milestones",
        },
        {
            "field": "transition_edge_truth",
            "value": {
                "transition_edge": stub.get("transition_edge_id"),
                "not_transition_edge": stub.get("not_transition_edge_id"),
            },
            "promotion_status": "safe_from_validated_stub",
        },
        {
            "field": "route_claim_boundary",
            "value": stub.get("claim_boundary"),
            "promotion_status": "safe_from_validated_stub",
        },
        {
            "field": "schema_identity_lineage",
            "value": _schema_identity_lineage(stub),
            "promotion_status": "safe_from_validated_stub",
        },
        {
            "field": "source_manifest_lineage",
            "value": _source_manifest_lineage(stub),
            "promotion_status": "safe_from_validated_stub_when_present",
        },
    ]
    if stub_type == "cross_floor_object":
        fields.extend(
            [
                {"field": "query", "value": stub.get("query"), "promotion_status": "safe_from_validated_stub"},
                {"field": "object_id", "value": stub.get("object_id"), "promotion_status": "safe_from_validated_stub"},
                {
                    "field": "object_label",
                    "value": stub.get("object_label"),
                    "promotion_status": "safe_from_validated_stub",
                },
                {
                    "field": "target_floor",
                    "value": stub.get("target_floor"),
                    "promotion_status": "safe_from_validated_stub",
                },
                {
                    "field": "target_room",
                    "value": stub.get("target_room"),
                    "promotion_status": "safe_from_validated_stub",
                },
                {
                    "field": "approach_candidate",
                    "value": stub.get("approach_candidate"),
                    "promotion_status": "safe_from_validated_stub",
                },
                {
                    "field": "object_centroid_navigation_used",
                    "value": stub.get("object_centroid_navigation_used"),
                    "promotion_status": "safe_from_validated_stub",
                },
            ]
        )
    return fields


def _blocked_fields(stub_type: str) -> list[dict[str, Any]]:
    blocked = [
        {
            "field": "real_astar_waypoint_list",
            "blocked_reason": "requires stable occupancy map package and actual route generation",
            "claim_boundary": "no real A* route samples generated in this task",
        },
        {
            "field": "verified_stable_occupancy_map_reference",
            "blocked_reason": "requires a real Layer 2 stable occupancy map package verified to exist locally",
            "claim_boundary": "dry-run does not verify generated map paths",
        },
        {
            "field": "verified_vertical_connector_geometry",
            "blocked_reason": "requires a real Layer 2 vertical connector artifact verified to exist locally",
            "claim_boundary": "dry-run does not verify generated connector geometry",
        },
        {
            "field": "verified_cross_floor_topology",
            "blocked_reason": "requires a real Layer 2 cross-floor topology artifact verified to exist locally",
            "claim_boundary": "dry-run does not verify generated topology",
        },
        {
            "field": "executable_route_waypoints",
            "blocked_reason": "requires route generation against formal Layer 2 artifacts",
            "claim_boundary": "no executable route generated",
        },
        {
            "field": "runtime_trajectory",
            "blocked_reason": "requires Layer 4 runtime validation",
            "claim_boundary": "runtime systems were not launched",
        },
        {
            "field": "collision_validation",
            "blocked_reason": "requires route geometry and validation against formal maps",
            "claim_boundary": "no collision-free execution claim is made",
        },
        {
            "field": "gazebo_rviz_nav2_validation",
            "blocked_reason": "requires explicit runtime launch task",
            "claim_boundary": "Gazebo, RViz, Nav2, and AMCL were not launched",
        },
        {
            "field": "physical_stair_climbing",
            "blocked_reason": "outside validated claim boundary",
            "claim_boundary": "no physical stair climbing claim is made",
        },
        {
            "field": "gait_footstep_contact_planning",
            "blocked_reason": "outside validated claim boundary",
            "claim_boundary": "no gait, footstep, or contact-dynamics claim is made",
        },
    ]
    if stub_type == "cross_floor_object":
        blocked.extend(
            [
                {
                    "field": "verified_object_query_resolution",
                    "blocked_reason": "requires object query resolution artifact generated from current Layer 2 artifacts",
                    "claim_boundary": "only the validated obj_175 identity can be previewed",
                },
                {
                    "field": "verified_object_approach_geometry_from_current_layer2_artifacts",
                    "blocked_reason": "requires object approach candidate artifact generated from current Layer 2 artifacts",
                    "claim_boundary": "only the validated generated_ring_037 identity can be previewed",
                },
                {
                    "field": "final_object_approach_route",
                    "blocked_reason": "requires finalized object approach geometry and route generation",
                    "claim_boundary": "no final object approach route generated",
                },
                {
                    "field": "object_facing_approach_validation",
                    "blocked_reason": "requires object approach validation in a separate validation task",
                    "claim_boundary": "no object-facing approach validation claim is made",
                },
            ]
        )
    return blocked


def _future_required_artifacts(stub_type: str) -> list[dict[str, Any]]:
    artifact_names = list(COMMON_LAYER2_DEPENDENCIES)
    if stub_type == "cross_floor_object":
        artifact_names.extend(OBJECT_LAYER2_DEPENDENCIES)
    return [
        {
            "artifact": name,
            "layer": "Layer 2: Formal Artifact Layer",
            "required_before_finalization": True,
            "applies_to_route_kind": stub_type,
        }
        for name in artifact_names
    ]


def _candidate_contract_preview(stub: Mapping[str, Any]) -> dict[str, Any]:
    stub_type = str(stub.get("stub_type", ""))
    preview: dict[str, Any] = {
        "preview_only": True,
        "not_written_as_final_artifact": True,
        "requires_layer2_artifacts_before_finalization": True,
        "candidate_route_kind": _candidate_kind(stub_type),
        "scene_id": stub.get("scene_id"),
        "route_sequence": stub.get("route_sequence", []),
        "transition_edge_truth": {
            "transition_edge": stub.get("transition_edge_id"),
            "not_transition_edge": stub.get("not_transition_edge_id"),
        },
        "is_final_route_contract": False,
        "candidate_contract_generated": False,
    }
    if stub_type == "cross_floor_object":
        preview["object_binding"] = {
            "query": stub.get("query"),
            "object_id": stub.get("object_id"),
            "object_label": stub.get("object_label"),
            "target_floor": stub.get("target_floor"),
            "target_room": stub.get("target_room"),
            "approach_candidate": stub.get("approach_candidate"),
            "object_centroid_navigation_used": stub.get("object_centroid_navigation_used"),
        }
    return preview


def _claim_boundary(stub: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "source_stub_claim_boundary": stub.get("claim_boundary"),
        "dry_run_only": True,
        "is_final_route_contract": False,
        "candidate_contract_generated": False,
        "protected_until_real_route_generation": [
            "A* waypoint list",
            "executable route waypoints",
            "stable occupancy map-backed route geometry",
            "runtime trajectory",
            "collision validation",
            "Gazebo/RViz/Nav2/AMCL validation",
            "physical stair climbing",
            "gait, footstep, and contact-dynamics planning",
        ],
    }


def _load_stub(path: str | Path) -> Mapping[str, Any]:
    with Path(path).expanduser().resolve().open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, Mapping):
        raise ValueError("route contract stub JSON must be an object")
    return data


def _promotion_item(stub_path: str | Path, stub: Mapping[str, Any], file_report: Mapping[str, Any]) -> dict[str, Any]:
    stub_type = str(stub.get("stub_type", ""))
    return {
        "source_stub_path": Path(stub_path).expanduser().resolve().as_posix(),
        "source_stub_type": stub.get("stub_type"),
        "schema_validation_passed": file_report.get("ok") is True,
        "schema_validation_error_count": file_report.get("error_count", 0),
        "schema_validation_warning_count": file_report.get("warning_count", 0),
        "scene_id": stub.get("scene_id"),
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": ARTIFACT_ROLE,
        "promotable_fields": _promotable_fields(stub),
        "blocked_fields": _blocked_fields(stub_type),
        "future_required_artifacts": _future_required_artifacts(stub_type),
        "candidate_contract_preview": _candidate_contract_preview(stub),
        "claim_boundary": _claim_boundary(stub),
    }


def _summarize_blocked_fields(items: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    summary: dict[str, set[str]] = {}
    for item in items:
        source_type = str(item.get("source_stub_type"))
        for blocked in item.get("blocked_fields", []):
            field = str(blocked.get("field"))
            summary.setdefault(field, set()).add(source_type)
    return [{"field": field, "source_stub_types": sorted(types)} for field, types in sorted(summary.items())]


def _summarize_future_dependencies(items: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    summary: dict[str, set[str]] = {}
    for item in items:
        source_type = str(item.get("source_stub_type"))
        for artifact in item.get("future_required_artifacts", []):
            name = str(artifact.get("artifact"))
            summary.setdefault(name, set()).add(source_type)
    return [
        {
            "artifact": name,
            "layer": "Layer 2: Formal Artifact Layer",
            "required_before_finalization": True,
            "source_stub_types": sorted(types),
        }
        for name, types in sorted(summary.items())
    ]


def _claim_boundary_summary(items: list[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "dry_run_only": True,
        "final_route_contract_generated": False,
        "candidate_contract_generated": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "protected_fields_remain_blocked": sorted(
            {
                str(field)
                for item in items
                for field in item.get("claim_boundary", {}).get("protected_until_real_route_generation", [])
            }
        ),
    }


def _route_kind_dependency_scope() -> dict[str, Any]:
    return {
        "cross_floor_room": {
            "required_layer2_dependencies": COMMON_LAYER2_DEPENDENCIES,
            "object_query_resolution_artifact": "not_applicable_to_cross_floor_room",
            "object_approach_candidate_artifact": "not_applicable_to_cross_floor_room",
        },
        "cross_floor_object": {
            "required_layer2_dependencies": COMMON_LAYER2_DEPENDENCIES + OBJECT_LAYER2_DEPENDENCIES,
            "object_query_resolution_artifact": "required_before_finalization",
            "object_approach_candidate_artifact": "required_before_finalization",
        },
    }


def build_promotion_dry_run_report(input_jsons: Iterable[str | Path]) -> dict[str, Any]:
    input_paths = [Path(path).expanduser().resolve() for path in input_jsons]
    schema_report = validate_route_contract_stub_files(input_paths)
    stubs_by_path: dict[str, Mapping[str, Any]] = {}
    load_errors: list[str] = []
    for path in input_paths:
        try:
            stubs_by_path[path.as_posix()] = _load_stub(path)
        except Exception as exc:
            load_errors.append(f"{path.as_posix()}: {exc}")

    unsupported_types = sorted(
        {
            str(stub.get("stub_type"))
            for stub in stubs_by_path.values()
            if stub.get("stub_type") not in ALLOWED_STUB_TYPES
        }
    )
    reports_by_path = {
        Path(report["input_json"]).expanduser().resolve().as_posix(): report
        for report in schema_report["file_reports"]
    }
    promotion_items = [
        _promotion_item(path, stub, reports_by_path.get(path.as_posix(), {}))
        for path in input_paths
        for stub in [stubs_by_path.get(path.as_posix())]
        if stub is not None
    ]

    schema_error_count = int(schema_report.get("error_count", 0)) + len(load_errors)
    schema_warning_count = int(schema_report.get("warning_count", 0))
    if unsupported_types:
        classification = UNSUPPORTED_STUB_TYPE_CLASSIFICATION
    elif schema_error_count:
        classification = SCHEMA_FAILED_CLASSIFICATION
    else:
        classification = READY_CLASSIFICATION

    report = {
        "project_name": PROJECT_NAME,
        "classification": classification,
        "error_count": 0 if classification == READY_CLASSIFICATION else max(1, schema_error_count),
        "warning_count": schema_warning_count,
        "dry_run": True,
        "is_final_route_contract": False,
        "candidate_contract_generated": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "validated_files": schema_report.get("validated_files", []),
        "promotion_items": promotion_items,
        "blocked_fields_summary": _summarize_blocked_fields(promotion_items),
        "future_layer2_dependencies": _summarize_future_dependencies(promotion_items),
        "route_kind_dependency_scope": _route_kind_dependency_scope(),
        "claim_boundary_summary": _claim_boundary_summary(promotion_items),
        "schema_validation_report": {
            "classification": schema_report.get("classification"),
            "error_count": schema_report.get("error_count"),
            "warning_count": schema_report.get("warning_count"),
        },
    }
    if load_errors:
        report["load_errors"] = load_errors
    if unsupported_types:
        report["unsupported_stub_types"] = unsupported_types
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dry-run promotion of RSLG-SLAM route contract stubs.")
    parser.add_argument("--input-json", action="append", required=True, help="Route contract stub JSON. May be repeated.")
    parser.add_argument("--output-json", required=True, help="Write the promotion dry-run report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    report = build_promotion_dry_run_report(args.input_json)
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
    return 0 if report["classification"] == READY_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
