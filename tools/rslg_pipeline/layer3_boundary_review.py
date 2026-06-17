"""Layer 3 boundary review before returning to Layer 2 regeneration.

This utility reviews the current route-contract dry-run chain for RSLG-SLAM.
It reads docs/manifests and small dry-run reports when present, but never
generates final route contracts, real routes, maps, connector geometry, object
approach candidates, runtime inputs, or runtime validation artifacts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .common import PROJECT_NAME, load_json, normalize_repo_relative, repo_path, resolve_repo_root, save_json
from .validate_artifacts import run_static_validation


COMPLETED_CLASSIFICATION = "layer3_boundary_review_completed"
FAILED_CLASSIFICATION = "layer3_boundary_review_failed"
STATIC_FAILED_CLASSIFICATION = "layer3_boundary_review_blocked_static_validation_failed"
CANDIDATE_FAILED_CLASSIFICATION = "layer3_boundary_review_blocked_candidate_schema_failed"

CURRENT_LAYER = "Layer 3: Navigation Interface Layer"
LAYER2_LAYER = "Layer 2: Formal Artifact Layer"

COMMON_LAYER2_DEPENDENCIES = [
    "stable occupancy map package",
    "vertical connector artifact",
    "cross-floor topology artifact",
]

OBJECT_LAYER2_DEPENDENCIES = [
    "object query resolution artifact",
    "object approach candidate artifact",
]

ROOM_BLOCKED_FIELDS = [
    "real A* waypoint list",
    "verified stable occupancy map reference",
    "verified vertical connector geometry",
    "verified cross-floor topology",
    "executable route waypoints",
    "collision validation",
    "runtime trajectory",
    "Gazebo/RViz/Nav2 validation",
]

OBJECT_BLOCKED_FIELDS = ROOM_BLOCKED_FIELDS + [
    "verified object query resolution",
    "verified object approach geometry from current Layer 2 artifacts",
    "final object approach route",
    "object-facing approach validation",
]

LAYER3_COMPLETED_CAPABILITIES = [
    "route contract dry-run planning",
    "route contract stub generation",
    "route contract stub schema validation",
    "route contract stub to candidate contract promotion dry-run",
    "candidate route contract schema dry-run validation",
]

LAYER3_NOT_COMPLETED_CAPABILITIES = [
    "final candidate route contract generation",
    "real A* waypoint generation",
    "executable route generation",
    "runtime input package generation",
    "runtime validation",
]

REPORT_FILENAMES = [
    "route_contract_stub_cross_floor_object_v0_1.json",
    "route_contract_stub_cross_floor_room_v0_1.json",
    "route_contract_stub_schema_validation_report_v0_1.json",
    "route_contract_candidate_promotion_dry_run_report_v0_1.json",
    "candidate_route_contract_schema_dry_run_report_v0_1.json",
]

DOCS_AND_MANIFESTS_READ = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/manifest_retention_policy.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
    "docs/rslg_slam/ai_handoff_context.md",
    "docs/rslg_slam/ai_handoff_file_list.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json",
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/manifests/manifest_retention_plan_v0_1.json",
    "docs/rslg_slam/manifests/rslg_pipeline_test_plan_manifest_v0_1.json",
]


def _load_report_if_present(path: Path) -> Mapping[str, Any] | None:
    if not path.is_file():
        return None
    data = load_json(path)
    if not isinstance(data, Mapping):
        raise ValueError(f"{path.as_posix()} must contain a JSON object")
    return data


def _read_evidence_reports(report_dir: Path, repo_root: Path) -> tuple[dict[str, Mapping[str, Any]], list[dict[str, Any]]]:
    reports: dict[str, Mapping[str, Any]] = {}
    used: list[dict[str, Any]] = []
    for filename in REPORT_FILENAMES:
        path = report_dir / filename
        report = _load_report_if_present(path)
        if report is None:
            continue
        reports[filename] = report
        used.append(
            {
                "path": normalize_repo_relative(path, repo_root),
                "classification": report.get("classification"),
                "error_count": report.get("error_count"),
                "warning_count": report.get("warning_count"),
            }
        )
    return reports, used


def _doc_read_records(repo_root: Path) -> list[dict[str, Any]]:
    records = []
    for rel_path in DOCS_AND_MANIFESTS_READ:
        path = repo_path(repo_root, rel_path)
        records.append({"path": rel_path, "exists": path.is_file()})
    return records


def _names(records: Iterable[Mapping[str, Any]], key: str) -> set[str]:
    result = set()
    for record in records:
        value = record.get(key)
        if value:
            result.add(str(value))
    return result


def _candidate_scope_review(candidate_report: Mapping[str, Any] | None) -> dict[str, Any]:
    if candidate_report is None:
        return {
            "candidate_schema_report_present": False,
            "cross_floor_room_object_dependencies_removed_or_not_applicable": True,
            "cross_floor_object_object_dependencies_preserved": False,
            "scope_errors": [],
            "scope_warnings": ["candidate schema report not present; scoped dependency details inferred from code contract"],
        }

    scope_errors: list[str] = []
    room_items = []
    object_items = []
    for item in candidate_report.get("candidate_schema_items", []):
        if not isinstance(item, Mapping):
            continue
        if item.get("source_stub_type") == "cross_floor_room":
            room_items.append(item)
        elif item.get("source_stub_type") == "cross_floor_object":
            object_items.append(item)

    for item in room_items:
        required = _names(item.get("future_required_artifacts", []), "artifact")
        blocked = _names(item.get("blocked_schema_fields", []), "field")
        leaked_required = sorted(required.intersection(OBJECT_LAYER2_DEPENDENCIES))
        leaked_blocked = sorted(field for field in blocked if "object" in field.lower())
        if leaked_required:
            scope_errors.append(f"cross_floor_room lists object dependencies as required: {leaked_required}")
        if leaked_blocked:
            scope_errors.append(f"cross_floor_room lists object blocked fields: {leaked_blocked}")

    object_dependencies_preserved = False
    for item in object_items:
        required = _names(item.get("future_required_artifacts", []), "artifact")
        if set(OBJECT_LAYER2_DEPENDENCIES).issubset(required):
            object_dependencies_preserved = True

    if object_items and not object_dependencies_preserved:
        scope_errors.append("cross_floor_object is missing object query/object approach Layer 2 dependencies")

    return {
        "candidate_schema_report_present": True,
        "candidate_schema_classification": candidate_report.get("classification"),
        "candidate_schema_error_count": candidate_report.get("error_count"),
        "cross_floor_room_items_checked": len(room_items),
        "cross_floor_object_items_checked": len(object_items),
        "cross_floor_room_object_dependencies_removed_or_not_applicable": not any(
            error.startswith("cross_floor_room") for error in scope_errors
        ),
        "cross_floor_object_object_dependencies_preserved": object_dependencies_preserved,
        "scope_errors": scope_errors,
        "scope_warnings": [],
    }


def _route_kind_dependency_scope() -> dict[str, Any]:
    return {
        "cross_floor_room": {
            "required_layer2_dependencies": COMMON_LAYER2_DEPENDENCIES,
            "object_query_resolution_artifact": "not_applicable_to_cross_floor_room",
            "object_approach_candidate_artifact": "not_applicable_to_cross_floor_room",
            "blocked_fields": ROOM_BLOCKED_FIELDS,
        },
        "cross_floor_object": {
            "required_layer2_dependencies": COMMON_LAYER2_DEPENDENCIES + OBJECT_LAYER2_DEPENDENCIES,
            "object_query_resolution_artifact": "required_before_finalization",
            "object_approach_candidate_artifact": "required_before_finalization",
            "blocked_fields": OBJECT_BLOCKED_FIELDS,
        },
    }


def build_layer3_boundary_review(repo_root: Path, scene_id: str, output_json: str | Path) -> dict[str, Any]:
    output_path = repo_path(repo_root, output_json)
    report_dir = output_path.parent
    errors: list[str] = []
    warnings: list[str] = []

    static_report = run_static_validation(repo_root)
    if static_report.get("ok") is not True:
        errors.append("static validation failed")

    evidence_reports, evidence_used = _read_evidence_reports(report_dir, repo_root)
    candidate_report = evidence_reports.get("candidate_route_contract_schema_dry_run_report_v0_1.json")
    candidate_schema_ok = True
    if candidate_report is not None:
        candidate_schema_ok = (
            candidate_report.get("classification") == "candidate_route_contract_schema_dry_run_passed"
            and int(candidate_report.get("error_count", 0) or 0) == 0
        )
        if not candidate_schema_ok:
            errors.append("candidate route contract schema dry-run report failed")
    else:
        warnings.append("candidate route contract schema dry-run report was not present beside the boundary review output")

    scope_review = _candidate_scope_review(candidate_report)
    warnings.extend(scope_review.get("scope_warnings", []))
    errors.extend(scope_review.get("scope_errors", []))

    if static_report.get("ok") is not True:
        classification = STATIC_FAILED_CLASSIFICATION
    elif not candidate_schema_ok or scope_review.get("scope_errors"):
        classification = CANDIDATE_FAILED_CLASSIFICATION
    else:
        classification = COMPLETED_CLASSIFICATION

    recommended_next_task = "task25p_layer2_vertical_connector_and_topology_artifact_dry_run"
    recommended_layer2_entrypoint = "tools/rslg_pipeline/build_vertical_connectors.py"

    report = {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "scene_id": scene_id,
        "classification": classification,
        "error_count": 0 if classification == COMPLETED_CLASSIFICATION else max(1, len(errors)),
        "warning_count": len(warnings),
        "review_only": True,
        "dry_run": True,
        "final_route_contract_generated": False,
        "candidate_contract_generated": False,
        "real_astar_route_generated": False,
        "stable_map_generated": False,
        "vertical_connector_geometry_generated": False,
        "object_approach_candidates_generated": False,
        "runtime_artifacts_generated": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "current_layer": CURRENT_LAYER,
        "layer3_completed_capabilities": LAYER3_COMPLETED_CAPABILITIES,
        "layer3_not_completed_capabilities": LAYER3_NOT_COMPLETED_CAPABILITIES,
        "layer3_stop_extending_for_now": classification == COMPLETED_CLASSIFICATION,
        "blocked_by_layer2_artifacts": {
            "cross_floor_room": ROOM_BLOCKED_FIELDS,
            "cross_floor_object": OBJECT_BLOCKED_FIELDS,
        },
        "route_kind_dependency_scope": _route_kind_dependency_scope(),
        "route_kind_dependency_scope_review": scope_review,
        "recommended_next_phase": "return_to_layer2_formal_artifact_regeneration",
        "recommended_next_task": recommended_next_task,
        "recommended_layer2_entrypoint": recommended_layer2_entrypoint,
        "recommended_layer2_entrypoint_rationale": (
            "A final cross-floor route contract needs stable maps, vertical connectors, and cross-floor topology. "
            "The next dry-run should start with vertical connector and topology artifacts because vt_1 transition "
            "truth is the central cross-floor dependency shared by both room and object route contracts; stable map "
            "formalization remains required as a following or companion Layer 2 task."
        ),
        "claim_boundary_review": {
            "review_only": True,
            "dry_run_only": True,
            "no_final_route_contract": True,
            "no_real_astar_route": True,
            "no_runtime_validation": True,
            "no_world_model_or_stage_a_rerun": True,
            "no_external_gt_or_manual_source_claim": True,
            "does_not_claim_dataset_side_slam_or_localization_accuracy": True,
            "does_not_claim_dense_reconstruction_or_robot_stair_climbing": True,
        },
        "evidence_reports_used": evidence_used,
        "docs_and_manifests_read": _doc_read_records(repo_root),
        "static_validation_summary": {
            "classification": static_report.get("classification"),
            "error_count": static_report.get("error_count"),
            "warning_count": static_report.get("warning_count"),
            "stage_outputs_required_for_validation": static_report.get("stage_outputs_required_for_validation"),
            "world_model_rerun": static_report.get("world_model_rerun"),
            "runtime_launched": static_report.get("runtime_launched"),
        },
        "candidate_schema_summary": {
            "present": candidate_report is not None,
            "classification": candidate_report.get("classification") if candidate_report else None,
            "error_count": candidate_report.get("error_count") if candidate_report else None,
            "warning_count": candidate_report.get("warning_count") if candidate_report else None,
        },
        "errors": errors,
        "warnings": warnings,
    }
    if classification not in {COMPLETED_CLASSIFICATION, STATIC_FAILED_CLASSIFICATION, CANDIDATE_FAILED_CLASSIFICATION}:
        report["classification"] = FAILED_CLASSIFICATION
        report["error_count"] = max(1, len(errors))
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Review RSLG-SLAM Layer 3 route-contract boundary.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--output-json", required=True, help="Write the boundary review report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    try:
        report = build_layer3_boundary_review(repo_root, args.scene_id, args.output_json)
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        report = {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": args.scene_id,
            "classification": FAILED_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "review_only": True,
            "dry_run": True,
            "final_route_contract_generated": False,
            "candidate_contract_generated": False,
            "real_astar_route_generated": False,
            "stable_map_generated": False,
            "vertical_connector_geometry_generated": False,
            "object_approach_candidates_generated": False,
            "runtime_artifacts_generated": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "current_layer": CURRENT_LAYER,
            "errors": [str(exc)],
            "warnings": [],
        }

    output_path = repo_path(repo_root, args.output_json)
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
    return 0 if report["classification"] == COMPLETED_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
