"""Readiness review before real RSLG-SLAM route generation.

This module reviews Layer 3 route plan previews and decides whether the
project is ready to generate real A* waypoint routes or executable route
artifacts. It is review-only: it does not generate routes, waypoints, runtime
inputs, stable map pixels, connector geometry, object approach geometry, or
runtime validation evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, load_json, normalize_repo_relative, repo_path, resolve_repo_root, save_json


COMPLETED_CLASSIFICATION = "route_generation_readiness_review_completed"
FAILED_CLASSIFICATION = "route_generation_readiness_review_failed"

ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"

ROUTE_PLAN_PREVIEW_READINESS = "route_plan_previews_ready_for_readiness_review"
CANDIDATE_CONTRACT_READINESS = "candidate_route_contracts_ready"
REAL_ROUTE_BLOCKED = "real_route_generation_blocked_requires_final_layer2_artifacts"
EXECUTABLE_ROUTE_BLOCKED = "executable_route_generation_blocked_requires_real_route"
RUNTIME_VALIDATION_BLOCKED = "runtime_validation_blocked_requires_executable_route_and_runtime_inputs"

SUPPORTED_ROUTE_KINDS = {"cross_floor_room", "cross_floor_object"}
EXPECTED_PREVIEW_CLASSIFICATIONS = {
    "cross_floor_room": "cross_floor_room_route_plan_preview_generated",
    "cross_floor_object": "cross_floor_object_route_plan_preview_generated",
}
EXPECTED_SEGMENTS = {
    "cross_floor_room": ["floor_1_room_segment", "vertical_transition_segment", "floor_2_room_segment"],
    "cross_floor_object": [
        "floor_1_room_segment",
        "vertical_transition_segment",
        "floor_2_room_segment",
        "object_approach_segment",
    ],
}

FINAL_LAYER2_COMMON_MISSING = [
    "final stable occupancy map package",
    "final vertical connector artifact",
    "final connector graph artifact",
    "final cross-floor topology artifact",
    "final route planner graph",
    "final map metadata package",
]

FINAL_LAYER2_OBJECT_MISSING = [
    "final object query resolution artifact",
    "final object approach artifact",
    "final object interface package",
    "approach feasibility / clearance validation evidence",
]

MISSING_PLANNER_INPUTS = [
    "final stable map raster or equivalent planner map",
    "final stable map metadata",
    "route planner graph",
    "floor transition planner edge",
    "route segment endpoint references",
    "planner-safe connector transition representation",
    "object approach endpoint or approach-stage plan for cross_floor_object",
]

MISSING_RUNTIME_INPUTS = [
    "executable route waypoints",
    "runtime route package",
    "RViz visualization inputs",
    "controller-compatible path representation",
    "runtime validation config",
    "Gazebo/RViz/Nav2 launch linkage, if used later",
]

FORBIDDEN_TRUE_FLAGS = [
    "real_astar_route_generated",
    "real_route_generated",
    "astar_waypoints_generated",
    "executable_route_generated",
    "runtime_input_package_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "amcl_success",
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "final_stable_map_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
]

FORBIDDEN_CLAIM_PHRASES = [
    "real A* route generated",
    "A* waypoints generated",
    "executable route generated",
    "runtime input package generated",
    "runtime validation completed",
    "Gazebo/RViz/Nav2 validation completed",
    "AMCL success",
    "physical stair climbing completed",
    "real robot stair climbing completed",
    "final stable map generated",
    "PGM/YAML generated",
    "external GT map used",
    "simulator navmesh used",
    "object centroid navigation used",
    "direct object centroid goal used",
]


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


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = " ".join(words[max(0, index - 8) : index])
        if any(marker in prefix for marker in ["not", "no", "without", "never", "must not", "do not", "does not"]):
            return True
    return False


def _payload(preview: Mapping[str, Any]) -> Mapping[str, Any]:
    payload = preview.get("route_plan_preview_payload", {})
    return payload if isinstance(payload, Mapping) else {}


def _segment_types(preview: Mapping[str, Any]) -> list[str]:
    segments = _payload(preview).get("segment_preview_list", [])
    if not isinstance(segments, list):
        return []
    return [
        str(segment["segment_type"])
        for segment in segments
        if isinstance(segment, Mapping) and segment.get("segment_type")
    ]


def _object_safety_flag(preview: Mapping[str, Any], key: str) -> Any:
    payload = _payload(preview)
    object_safety = payload.get("object_safety", {})
    if key in payload:
        return payload.get(key)
    if isinstance(object_safety, Mapping):
        return object_safety.get(key)
    return None


def _claim_boundary_findings(preview: Mapping[str, Any], source: Path) -> list[dict[str, Any]]:
    findings: list[dict[str, Any]] = []
    for path, value in _walk(preview):
        leaf = path.rsplit(".", 1)[-1]
        if leaf in FORBIDDEN_TRUE_FLAGS and value is True:
            findings.append(
                {
                    "source": source.as_posix(),
                    "path": path,
                    "claim": leaf,
                    "status": "failed",
                    "reason": "forbidden positive claim flag is true",
                }
            )
        if isinstance(value, str):
            text = _normalize_text(value)
            for phrase in FORBIDDEN_CLAIM_PHRASES:
                normalized = _normalize_text(phrase)
                if normalized in text and not _phrase_is_negated(text, normalized):
                    findings.append(
                        {
                            "source": source.as_posix(),
                            "path": path,
                            "claim": phrase,
                            "status": "failed",
                            "reason": "forbidden positive claim phrase appears without negation",
                        }
                    )
    return findings


def _load_preview(path: Path) -> tuple[Mapping[str, Any] | None, list[str]]:
    try:
        loaded = load_json(path)
    except Exception as exc:
        return None, [f"{path.as_posix()}: failed to load JSON: {exc}"]
    if not isinstance(loaded, Mapping):
        return None, [f"{path.as_posix()}: route plan preview must be a JSON object"]
    return loaded, []


def _preview_record(repo_root: Path, path: Path, preview: Mapping[str, Any] | None) -> dict[str, Any]:
    if preview is None:
        return {
            "path": normalize_repo_relative(path, repo_root),
            "exists": path.is_file(),
            "route_kind": None,
            "classification": None,
            "reviewed": False,
        }
    route_kind = str(preview.get("route_kind"))
    return {
        "path": normalize_repo_relative(path, repo_root),
        "exists": True,
        "route_kind": route_kind,
        "classification": preview.get("classification"),
        "artifact_layer": preview.get("artifact_layer"),
        "is_route_plan_preview": preview.get("is_route_plan_preview"),
        "is_final_route_plan": preview.get("is_final_route_plan"),
        "generated_from_candidate_route_contract": preview.get("generated_from_candidate_route_contract"),
        "generated_from_final_route_contract": preview.get("generated_from_final_route_contract"),
        "segment_types": _segment_types(preview),
        "reviewed": True,
    }


def _route_kind_review(route_kind: str, preview: Mapping[str, Any] | None) -> dict[str, Any]:
    expected_segments = EXPECTED_SEGMENTS[route_kind]
    segment_types = _segment_types(preview or {})
    object_segment_present = "object_approach_segment" in segment_types
    common = {
        "route_kind": route_kind,
        "route_plan_preview_exists": preview is not None,
        "route_plan_preview_classification": preview.get("classification") if preview else None,
        "expected_route_plan_preview_classification": EXPECTED_PREVIEW_CLASSIFICATIONS[route_kind],
        "segment_scope_valid": segment_types == expected_segments,
        "segment_types": segment_types,
        "expected_segment_types": expected_segments,
        "object_approach_segment_present": object_segment_present,
        "real_route_generation_status": "blocked",
    }
    if route_kind == "cross_floor_room":
        common.update(
            {
                "blocked_reason": "final Layer 2 planner inputs missing",
                "missing_artifacts": [
                    "final stable map package",
                    "final vertical connector/topology",
                    "final route planner graph",
                ],
            }
        )
    else:
        common.update(
            {
                "object_centroid_navigation_used": _object_safety_flag(preview or {}, "object_centroid_navigation_used")
                is True,
                "direct_object_centroid_goal_used": _object_safety_flag(preview or {}, "direct_object_centroid_goal_used")
                is True,
                "blocked_reason": "final Layer 2 planner inputs and final object approach evidence missing",
                "missing_artifacts": [
                    "final stable map package",
                    "final vertical connector/topology",
                    "final route planner graph",
                    "final object interface package",
                    "approach feasibility / clearance validation evidence",
                ],
            }
        )
    return common


def build_route_generation_readiness_review(
    repo_root: Path,
    scene_id: str,
    route_plan_previews: Iterable[str | Path],
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    preview_records: list[dict[str, Any]] = []
    previews_by_kind: dict[str, Mapping[str, Any]] = {}
    claim_failures: list[dict[str, Any]] = []

    for raw_path in route_plan_previews:
        path = repo_path(repo_root, raw_path)
        if not path.is_file():
            errors.append(f"route plan preview does not exist: {normalize_repo_relative(path, repo_root)}")
            preview_records.append(_preview_record(repo_root, path, None))
            continue
        preview, load_errors = _load_preview(path)
        errors.extend(load_errors)
        preview_records.append(_preview_record(repo_root, path, preview))
        if preview is None:
            continue
        route_kind = str(preview.get("route_kind"))
        if route_kind not in SUPPORTED_ROUTE_KINDS:
            errors.append(f"{path.as_posix()}: unsupported route_kind {route_kind!r}")
            continue
        if route_kind in previews_by_kind:
            errors.append(f"duplicate route plan preview for route_kind {route_kind!r}")
        previews_by_kind[route_kind] = preview
        if preview.get("scene_id") != scene_id:
            errors.append(f"{path.as_posix()}: scene_id must be {scene_id!r}")
        if preview.get("artifact_layer") != ARTIFACT_LAYER:
            errors.append(f"{path.as_posix()}: artifact_layer must be {ARTIFACT_LAYER!r}")
        if preview.get("classification") != EXPECTED_PREVIEW_CLASSIFICATIONS[route_kind]:
            errors.append(
                f"{path.as_posix()}: classification must be "
                f"{EXPECTED_PREVIEW_CLASSIFICATIONS[route_kind]!r}"
            )
        if preview.get("is_route_plan_preview") is not True:
            errors.append(f"{path.as_posix()}: is_route_plan_preview must be true")
        if preview.get("is_final_route_plan") is not False:
            errors.append(f"{path.as_posix()}: is_final_route_plan must be false")
        if preview.get("generated_from_candidate_route_contract") is not True:
            errors.append(f"{path.as_posix()}: generated_from_candidate_route_contract must be true")
        if preview.get("generated_from_final_route_contract") is not False:
            errors.append(f"{path.as_posix()}: generated_from_final_route_contract must be false")
        if _segment_types(preview) != EXPECTED_SEGMENTS[route_kind]:
            errors.append(
                f"{path.as_posix()}: segment scope must be {EXPECTED_SEGMENTS[route_kind]!r}, "
                f"got {_segment_types(preview)!r}"
            )
        claim_failures.extend(_claim_boundary_findings(preview, path))

    missing_route_kinds = sorted(SUPPORTED_ROUTE_KINDS - set(previews_by_kind))
    if missing_route_kinds:
        errors.append(f"missing route plan previews for route kinds {missing_route_kinds}")
    for finding in claim_failures:
        errors.append(f"{finding['source']}: forbidden claim boundary violation at {finding['path']}")

    route_kind_reviews = {
        route_kind: _route_kind_review(route_kind, previews_by_kind.get(route_kind))
        for route_kind in sorted(SUPPORTED_ROUTE_KINDS)
    }
    route_plan_previews_ready = not errors and set(previews_by_kind) == SUPPORTED_ROUTE_KINDS

    claim_boundary_checks = {
        "ok": not claim_failures,
        "checked_for": [
            "real A* route generated",
            "A* waypoints generated",
            "executable route generated",
            "runtime input package generated",
            "runtime validation completed",
            "Gazebo/RViz/Nav2 validation completed",
            "AMCL success",
            "physical stair climbing completed",
            "real robot stair climbing completed",
            "final stable map generated",
            "PGM/YAML generated",
            "external GT map used",
            "simulator navmesh used",
            "object centroid navigation used",
            "direct object centroid goal used",
        ],
        "failures": claim_failures,
        "summary": {
            "preview_only": route_plan_previews_ready,
            "final_stable_map_generated": False,
            "real_astar_route_generated": False,
            "astar_waypoints_generated": False,
            "executable_route_generated": False,
            "runtime_input_package_generated": False,
            "runtime_validation_completed": False,
            "gazebo_rviz_nav2_validation_completed": False,
            "amcl_success": False,
            "physical_stair_climbing_completed": False,
            "real_robot_stair_climbing_completed": False,
            "external_gt_map_used": False,
            "simulator_navmesh_used": False,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
        },
    }

    classification = COMPLETED_CLASSIFICATION if not errors else FAILED_CLASSIFICATION
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "review_scope": {
            "scope": "review route plan previews before real route generation",
            "review_only": True,
            "route_plan_preview_chain_reviewed": True,
            "real_route_generation_attempted": False,
            "executable_route_generation_attempted": False,
            "runtime_validation_attempted": False,
        },
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "final_route_plans_generated": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "route_plan_previews_reviewed": preview_records,
        "route_plan_preview_readiness": ROUTE_PLAN_PREVIEW_READINESS
        if route_plan_previews_ready
        else "route_plan_previews_not_ready_for_readiness_review",
        "candidate_route_contract_readiness": CANDIDATE_CONTRACT_READINESS
        if route_plan_previews_ready
        else "candidate_route_contracts_not_ready",
        "real_route_generation_readiness": REAL_ROUTE_BLOCKED,
        "executable_route_generation_readiness": EXECUTABLE_ROUTE_BLOCKED,
        "runtime_validation_readiness": RUNTIME_VALIDATION_BLOCKED,
        "missing_final_layer2_artifacts": FINAL_LAYER2_COMMON_MISSING + FINAL_LAYER2_OBJECT_MISSING,
        "missing_planner_inputs": MISSING_PLANNER_INPUTS,
        "missing_runtime_inputs": MISSING_RUNTIME_INPUTS,
        "route_kind_reviews": route_kind_reviews,
        "claim_boundary_checks": claim_boundary_checks,
        "readiness_decision": {
            "route_plan_preview_readiness": "ready" if route_plan_previews_ready else "not_ready",
            "candidate_route_contract_readiness": "ready" if route_plan_previews_ready else "not_ready",
            "real_astar_route_generation_readiness": "blocked",
            "executable_route_generation_readiness": "blocked",
            "runtime_validation_readiness": "blocked",
            "reason": "current chain uses candidate Layer 2 artifacts and candidate route contracts, not final Layer 2 formal artifacts",
        },
        "recommended_next_phase": "return_to_layer2_final_formal_artifact_generation",
        "recommended_next_task": (
            "generate_or_finalize_layer2_formal_artifacts_before_real_route_generation"
        ),
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Review readiness before real RSLG-SLAM route generation.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument(
        "--route-plan-preview",
        action="append",
        default=[],
        help="Route plan preview JSON. May be repeated.",
    )
    parser.add_argument("--output-json", required=True, help="Write the readiness review report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    if not args.route_plan_preview:
        report = {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "classification": FAILED_CLASSIFICATION,
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
            "real_astar_route_generated": False,
            "astar_waypoints_generated": False,
            "executable_route_generated": False,
            "runtime_input_package_generated": False,
            "runtime_artifacts_generated": False,
            "errors": ["Pass at least one --route-plan-preview."],
            "warnings": [],
        }
    else:
        report = build_route_generation_readiness_review(repo_root, args.scene_id, args.route_plan_preview)

    output_path = repo_path(repo_root, args.output_json)
    save_json(output_path, report)
    print(
        json.dumps(
            {
                "classification": report["classification"],
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
                "output_json": normalize_repo_relative(output_path, repo_root),
            },
            indent=2,
        )
    )
    return 0 if report["classification"] == COMPLETED_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
