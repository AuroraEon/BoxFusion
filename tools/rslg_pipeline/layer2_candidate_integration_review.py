"""Layer 2 candidate artifact integration review for RSLG-SLAM.

This module reviews non-final Layer 2 candidate artifacts across vertical
connectors/topology, stable maps, and object interfaces. It may generate fresh
candidate artifacts through the canonical builders when requested, but it never
generates final Layer 2 artifacts, Layer 3 route contracts, routes, map pixels,
runtime inputs, or launches runtime systems.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .build_object_interfaces import (
    EXPECTED_APPROACH_ID,
    EXPECTED_OBJECT_ID,
    EXPECTED_OBJECT_LABEL,
    EXPECTED_QUERY,
    EXPECTED_TARGET_FLOOR,
    EXPECTED_TARGET_ROOM,
    generate_candidate_package as generate_object_candidates,
)
from .build_stable_maps import generate_candidate_package as generate_stable_map_candidate
from .build_vertical_connectors import generate_candidate_artifacts as generate_vertical_candidates
from .common import PROJECT_NAME, load_json, normalize_repo_relative, repo_path, resolve_repo_root, save_json
from .object_interface_schema import validate_object_interface_files
from .stable_map_schema import validate_stable_map_files
from .vertical_connector_schema import validate_vertical_connector_reports


ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
PASS_CLASSIFICATION = "layer2_candidate_artifact_integration_review_passed"
FAIL_CLASSIFICATION = "layer2_candidate_artifact_integration_review_failed"

ROOM_READY = "cross_floor_room_layer3_candidate_contract_dependencies_ready"
ROOM_NOT_READY = "cross_floor_room_layer3_candidate_contract_dependencies_not_ready"
OBJECT_READY = "cross_floor_object_layer3_candidate_contract_dependencies_ready"
OBJECT_NOT_READY = "cross_floor_object_layer3_candidate_contract_dependencies_not_ready"
LAYER3_READY = "layer3_candidate_route_contract_generation_ready"
LAYER3_NOT_READY = "layer3_candidate_route_contract_generation_not_ready"

CANDIDATE_FILENAMES = {
    "vertical_connector": "vertical_connectors_candidate_v0_1.json",
    "connector_graph": "stairs_or_vertical_connector_graph_candidate_v0_1.json",
    "cross_floor_topology": "cross_floor_topology_candidate_v0_1.json",
    "stable_map_package": "stable_occupancy_map_package_candidate_v0_1.json",
    "object_query_resolution": "object_query_resolution_candidate_v0_1.json",
    "object_approach": "object_approach_candidate_v0_1.json",
    "object_interface_package": "object_interface_package_candidate_v0_1.json",
}

EXPECTED_CLASSIFICATIONS = {
    "vertical_connector": "vertical_connector_candidate_artifact_generated",
    "connector_graph": "connector_graph_candidate_artifact_generated",
    "cross_floor_topology": "cross_floor_topology_candidate_artifact_generated",
    "stable_map_package": "stable_map_package_candidate_artifact_generated",
    "object_query_resolution": "object_query_resolution_candidate_artifact_generated",
    "object_approach": "object_approach_candidate_artifact_generated",
    "object_interface_package": "object_interface_package_candidate_artifact_generated",
}

VERTICAL_CONNECTOR_KEYS = ["vertical_connector", "connector_graph", "cross_floor_topology"]
OBJECT_KEYS = ["object_query_resolution", "object_approach", "object_interface_package"]

CONNECTOR_IDS = {"vt_1", "vc_vt_1"}
SOURCE_FLOOR = "floor_1"
TARGET_FLOOR = "floor_2"
TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"
ROUTE_CONTEXT_ROOMS = ["room_2", "room_3", "room_7", "room_13", "room_14"]


def _walk(value: Any) -> Iterator[Any]:
    yield value
    if isinstance(value, Mapping):
        for item in value.values():
            yield from _walk(item)
    elif isinstance(value, list):
        for item in value:
            yield from _walk(item)


def _strings(value: Any) -> Iterator[str]:
    for item in _walk(value):
        if isinstance(item, str):
            yield item


def _text(value: Any) -> str:
    return " ".join(_strings(value)).lower()


def _flag_false(report: Mapping[str, Any], name: str) -> bool:
    return report.get(name, False) is False


def _check(name: str, ok: bool, details: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {"name": name, "status": "passed" if ok else "failed", "ok": ok, "details": dict(details or {})}


def _candidate_paths_from_args(repo_root: Path, args: argparse.Namespace) -> dict[str, Path]:
    explicit = {
        "vertical_connector": args.vertical_connectors_json,
        "connector_graph": args.connector_graph_json,
        "cross_floor_topology": args.cross_floor_topology_json,
        "stable_map_package": args.stable_map_package_json,
        "object_query_resolution": args.object_query_resolution_json,
        "object_approach": args.object_approach_json,
        "object_interface_package": args.object_interface_package_json,
    }
    if any(explicit.values()):
        return {
            key: repo_path(repo_root, value) if value else repo_path(repo_root, "__missing__")
            for key, value in explicit.items()
        }
    if not args.candidate_root:
        return {}
    root = repo_path(repo_root, args.candidate_root)
    return {key: root / filename for key, filename in CANDIDATE_FILENAMES.items()}


def _load_candidates(paths: Mapping[str, Path], repo_root: Path) -> tuple[dict[str, dict[str, Any]], list[str]]:
    candidates: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    for key, path in paths.items():
        if not path.exists():
            errors.append(f"missing candidate artifact for {key}: {normalize_repo_relative(path, repo_root)}")
            continue
        try:
            data = load_json(path)
        except Exception as exc:
            errors.append(f"failed to load candidate artifact for {key}: {exc}")
            continue
        if not isinstance(data, dict):
            errors.append(f"candidate artifact for {key} must be a JSON object")
            continue
        candidates[key] = data
    return candidates, errors


def _generate_candidates_if_requested(repo_root: Path, scene_id: str, candidate_root: Path, reports_dir: Path) -> list[dict[str, Any]]:
    summaries = [
        (
            "layer2_vertical_connector_topology_candidate_generation_summary_v0_1.json",
            generate_vertical_candidates(repo_root, scene_id, "all", candidate_root),
        ),
        (
            "layer2_stable_map_candidate_generation_summary_v0_1.json",
            generate_stable_map_candidate(repo_root, scene_id, "stable_occupancy", candidate_root),
        ),
        (
            "layer2_object_interface_candidate_generation_summary_v0_1.json",
            generate_object_candidates(
                repo_root,
                scene_id,
                EXPECTED_QUERY,
                EXPECTED_OBJECT_ID,
                EXPECTED_APPROACH_ID,
                candidate_root,
            ),
        ),
    ]
    written = []
    for filename, summary in summaries:
        path = reports_dir / filename
        save_json(path, summary)
        written.append({"path": normalize_repo_relative(path, repo_root), "classification": summary.get("classification")})
    return written


def _candidate_boundary_checks(candidates: Mapping[str, Mapping[str, Any]], errors: list[str]) -> dict[str, Any]:
    checks = []
    for key, report in candidates.items():
        failures = {}
        expected = EXPECTED_CLASSIFICATIONS[key]
        if report.get("classification") != expected:
            failures["classification"] = {"expected": expected, "actual": report.get("classification")}
        if report.get("is_candidate") is not True:
            failures["is_candidate"] = report.get("is_candidate")
        if report.get("is_final_formal_artifact") is not False:
            failures["is_final_formal_artifact"] = report.get("is_final_formal_artifact")
        for flag in [
            "world_model_rerun",
            "runtime_launched",
            "old_scripts_called",
            "business_logic_migrated",
            "final_formal_artifacts_generated",
            "final_layer2_artifacts_generated",
            "final_layer3_route_contracts_generated",
            "real_route_generated",
            "executable_route_generated",
            "runtime_artifacts_generated",
        ]:
            if flag in report and report.get(flag) is not False:
                failures[flag] = report.get(flag)
        if failures:
            errors.append(f"candidate/final boundary failed for {key}: {failures}")
        checks.append(_check(f"{key}_candidate_final_boundary", not failures, {"failures": failures}))
    return {"checks": checks, "ok": all(item["ok"] for item in checks)}


def _vertical_consistency(candidates: Mapping[str, Mapping[str, Any]], errors: list[str]) -> dict[str, Any]:
    combined = {key: candidates.get(key, {}) for key in VERTICAL_CONNECTOR_KEYS}
    text = _text(combined)
    failures: dict[str, Any] = {}
    if not any(connector_id in text for connector_id in CONNECTOR_IDS):
        failures["connector_id"] = sorted(CONNECTOR_IDS)
    for name, expected in {
        "source_floor": SOURCE_FLOOR,
        "target_floor": TARGET_FLOOR,
        "transition_edge": TRANSITION_EDGE,
        "non_transition_edge": NON_TRANSITION_EDGE,
    }.items():
        if expected.lower() not in text:
            failures[name] = expected
    missing_rooms = [room for room in ROUTE_CONTEXT_ROOMS if room not in text]
    if missing_rooms:
        failures["route_context_rooms"] = missing_rooms
    if failures:
        errors.append(f"vertical connector consistency failed: {failures}")
    return _check("vertical_connector_consistency", not failures, {"failures": failures})


def _stable_map_consistency(stable_map: Mapping[str, Any] | None, errors: list[str]) -> dict[str, Any]:
    failures: dict[str, Any] = {}
    if not stable_map:
        failures["stable_map_package"] = "missing"
    else:
        expected_false = [
            "stable_map_pixels_generated",
            "pgm_generated",
            "map_server_yaml_generated",
            "runtime_costmap_generated",
            "external_gt_map_used",
            "simulator_navmesh_used",
            "semantic_floorplan_claimed_as_stable_map",
            "room_mask_claimed_as_stable_map",
        ]
        if stable_map.get("map_payload_status") != "candidate_metadata_only_no_pixels":
            failures["map_payload_status"] = stable_map.get("map_payload_status")
        for flag in expected_false:
            if stable_map.get(flag) is not False:
                failures[flag] = stable_map.get(flag)
    if failures:
        errors.append(f"stable map consistency failed: {failures}")
    return _check("stable_map_consistency", not failures, {"failures": failures})


def _object_consistency(candidates: Mapping[str, Mapping[str, Any]], errors: list[str]) -> dict[str, Any]:
    combined = {key: candidates.get(key, {}) for key in OBJECT_KEYS}
    text = _text(combined)
    failures: dict[str, Any] = {}
    for name, expected in {
        "query": EXPECTED_QUERY,
        "object_id": EXPECTED_OBJECT_ID,
        "object_label": EXPECTED_OBJECT_LABEL,
        "target_floor": EXPECTED_TARGET_FLOOR,
        "target_room": EXPECTED_TARGET_ROOM,
        "approach_candidate_id": EXPECTED_APPROACH_ID,
    }.items():
        if expected.lower() not in text:
            failures[name] = expected
    for key in OBJECT_KEYS:
        report = candidates.get(key, {})
        for flag in [
            "object_centroid_navigation_used",
            "direct_object_centroid_goal_used",
            "approach_geometry_regenerated",
            "approach_feasibility_revalidated",
        ]:
            if flag in report and report.get(flag) is not False:
                failures[f"{key}.{flag}"] = report.get(flag)
    if failures:
        errors.append(f"object interface consistency failed: {failures}")
    return _check("object_interface_consistency", not failures, {"failures": failures})


def _claim_boundary_checks(candidates: Mapping[str, Mapping[str, Any]], errors: list[str]) -> dict[str, Any]:
    generated_flags = [
        "final_layer2_artifacts_generated",
        "final_layer3_route_contracts_generated",
        "final_formal_artifacts_generated",
        "real_route_generated",
        "executable_route_generated",
        "runtime_artifacts_generated",
        "stable_map_pixels_generated",
        "pgm_generated",
        "map_server_yaml_generated",
        "vertical_connector_geometry_generated",
        "real_geometry_regenerated",
        "approach_geometry_regenerated",
        "approach_feasibility_revalidated",
        "object_centroid_navigation_used",
        "direct_object_centroid_goal_used",
    ]
    failures: dict[str, Any] = {}
    for key, report in candidates.items():
        for flag in generated_flags:
            if flag in report and report.get(flag) is not False:
                failures[f"{key}.{flag}"] = report.get(flag)
    if failures:
        errors.append(f"claim boundary checks failed: {failures}")
    return {
        "ok": not failures,
        "checks": [_check("non_final_non_runtime_claim_boundaries", not failures, {"failures": failures})],
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "vertical_connector_geometry_generated": False,
        "object_approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }


def _schema_validation(paths: Mapping[str, Path], repo_root: Path, reports_dir: Path) -> dict[str, Any]:
    vertical_report = validate_vertical_connector_reports([paths[key] for key in VERTICAL_CONNECTOR_KEYS], mode="auto")
    stable_report = validate_stable_map_files([paths["stable_map_package"]], mode="auto")
    object_report = validate_object_interface_files([paths[key] for key in OBJECT_KEYS], mode="auto")

    report_paths = {
        "vertical_connector": reports_dir / "vertical_connector_schema_validation_report_v0_1.json",
        "stable_map": reports_dir / "stable_map_schema_validation_report_v0_1.json",
        "object_interface": reports_dir / "object_interface_schema_validation_report_v0_1.json",
    }
    save_json(report_paths["vertical_connector"], vertical_report)
    save_json(report_paths["stable_map"], stable_report)
    save_json(report_paths["object_interface"], object_report)
    return {
        "vertical_connector": {
            "classification": vertical_report.get("classification"),
            "error_count": vertical_report.get("error_count"),
            "warning_count": vertical_report.get("warning_count"),
            "report_json": normalize_repo_relative(report_paths["vertical_connector"], repo_root),
        },
        "stable_map": {
            "classification": stable_report.get("classification"),
            "error_count": stable_report.get("error_count"),
            "warning_count": stable_report.get("warning_count"),
            "report_json": normalize_repo_relative(report_paths["stable_map"], repo_root),
        },
        "object_interface": {
            "classification": object_report.get("classification"),
            "error_count": object_report.get("error_count"),
            "warning_count": object_report.get("warning_count"),
            "report_json": normalize_repo_relative(report_paths["object_interface"], repo_root),
        },
    }


def build_integration_review(
    repo_root: Path,
    scene_id: str,
    candidate_paths: Mapping[str, Path],
    output_json: Path,
    *,
    generate_candidates: bool = False,
) -> dict[str, Any]:
    reports_dir = output_json.parent
    candidate_root = candidate_paths.get("vertical_connector", reports_dir / "candidate_artifacts").parent
    generation_reports = (
        _generate_candidates_if_requested(repo_root, scene_id, candidate_root, reports_dir)
        if generate_candidates
        else []
    )

    errors: list[str] = []
    warnings: list[str] = []
    candidates, load_errors = _load_candidates(candidate_paths, repo_root)
    errors.extend(load_errors)

    schema_status = _schema_validation(candidate_paths, repo_root, reports_dir) if not load_errors else {}
    boundary = _candidate_boundary_checks(candidates, errors)
    integration_checks = [
        _vertical_consistency(candidates, errors),
        _stable_map_consistency(candidates.get("stable_map_package"), errors),
        _object_consistency(candidates, errors),
    ]
    claim_boundary = _claim_boundary_checks(candidates, errors)

    schema_ok = {
        "vertical_connector": schema_status.get("vertical_connector", {}).get("classification")
        == "vertical_connector_schema_validation_passed"
        and schema_status.get("vertical_connector", {}).get("error_count") == 0,
        "stable_map": schema_status.get("stable_map", {}).get("classification") == "stable_map_schema_validation_passed"
        and schema_status.get("stable_map", {}).get("error_count") == 0,
        "object_interface": schema_status.get("object_interface", {}).get("classification")
        == "object_interface_schema_validation_passed"
        and schema_status.get("object_interface", {}).get("error_count") == 0,
    }
    present = {key: key in candidates for key in CANDIDATE_FILENAMES}
    room_ready = (
        present["vertical_connector"]
        and present["cross_floor_topology"]
        and present["stable_map_package"]
        and schema_ok["vertical_connector"]
        and schema_ok["stable_map"]
    )
    object_ready = room_ready and all(present[key] for key in OBJECT_KEYS) and schema_ok["object_interface"]
    classification = PASS_CLASSIFICATION if not errors and room_ready and object_ready else FAIL_CLASSIFICATION

    candidate_artifact_paths = {
        key: normalize_repo_relative(path, repo_root) for key, path in candidate_paths.items()
    }
    candidate_classifications = {key: candidates.get(key, {}).get("classification") for key in CANDIDATE_FILENAMES}

    return {
        "project_name": PROJECT_NAME,
        "classification": classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "review_scope": [
            "vertical connector / connector graph / cross-floor topology candidates",
            "stable occupancy map package candidate",
            "object query resolution / object approach / object interface package candidates",
            "cross_floor_room and cross_floor_object Layer 3 dependency readiness",
        ],
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "candidate_generation_requested": bool(generate_candidates),
        "candidate_generation_reports": generation_reports,
        "candidate_artifacts_reviewed": candidate_artifact_paths,
        "candidate_artifact_classifications": candidate_classifications,
        "candidate_artifact_schema_validation_status": schema_status,
        "cross_floor_room_dependency_readiness": ROOM_READY if room_ready else ROOM_NOT_READY,
        "cross_floor_object_dependency_readiness": OBJECT_READY if object_ready else OBJECT_NOT_READY,
        "integration_consistency_checks": {
            "candidate_final_boundary": boundary,
            "checks": integration_checks,
        },
        "claim_boundary_checks": claim_boundary,
        "layer3_readiness": LAYER3_READY if room_ready and object_ready else LAYER3_NOT_READY,
        "blocked_finalization_fields": [
            "final Layer 2 formal artifact promotion",
            "final Layer 3 route contract generation",
            "real A* route samples",
            "executable route waypoints",
            "runtime input packages",
            "stable map pixels / PGM / map_server YAML",
            "real connector geometry",
            "real object approach geometry and feasibility validation",
        ],
        "recommended_next_task": "task30_layer3_candidate_route_contract_finalization_from_layer2_candidates"
        if classification == PASS_CLASSIFICATION
        else "repair_layer2_candidate_artifact_integration_inputs_before_layer3",
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Review Layer 2 candidate artifact integration and Layer 3 readiness.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--candidate-root", default=None, help="Directory containing candidate artifacts.")
    parser.add_argument("--generate-candidates", action="store_true", help="Generate fresh candidates into --candidate-root first.")
    parser.add_argument("--vertical-connectors-json", default=None, help="Vertical connector candidate JSON.")
    parser.add_argument("--connector-graph-json", default=None, help="Connector graph candidate JSON.")
    parser.add_argument("--cross-floor-topology-json", default=None, help="Cross-floor topology candidate JSON.")
    parser.add_argument("--stable-map-package-json", default=None, help="Stable map package candidate JSON.")
    parser.add_argument("--object-query-resolution-json", default=None, help="Object query resolution candidate JSON.")
    parser.add_argument("--object-approach-json", default=None, help="Object approach candidate JSON.")
    parser.add_argument("--object-interface-package-json", default=None, help="Object interface package candidate JSON.")
    parser.add_argument("--output-json", required=True, help="Write the integration review JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    output_json = repo_path(repo_root, args.output_json)
    candidate_paths = _candidate_paths_from_args(repo_root, args)
    if not candidate_paths:
        result = {
            "project_name": PROJECT_NAME,
            "classification": FAIL_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "scene_id": args.scene_id,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "final_layer2_artifacts_generated": False,
            "final_layer3_route_contracts_generated": False,
            "real_route_generated": False,
            "runtime_artifacts_generated": False,
            "errors": ["Pass --candidate-root or all explicit candidate input paths."],
            "warnings": [],
            "recommended_next_task": "provide_layer2_candidate_artifact_inputs",
        }
    else:
        result = build_integration_review(
            repo_root,
            args.scene_id,
            candidate_paths,
            output_json,
            generate_candidates=bool(args.generate_candidates),
        )

    save_json(output_json, result)
    print(
        json.dumps(
            {
                "classification": result["classification"],
                "error_count": result["error_count"],
                "warning_count": result["warning_count"],
                "output_json": normalize_repo_relative(output_json, repo_root),
            },
            indent=2,
            sort_keys=False,
        )
    )
    return 0 if result["classification"] == PASS_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
