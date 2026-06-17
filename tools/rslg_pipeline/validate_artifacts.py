"""Static validators for the canonical RSLG-SLAM pipeline skeleton.

These checks load project docs/manifests, verify naming and project-truth
contracts, and report guardrail status. They do not run the World Model Layer,
historical Stage-A entrypoints, ROS, Gazebo, RViz, Nav2, AMCL, or runtime demos.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Tuple

from .artifact_registry import ArtifactRegistry
from .common import (
    DEFAULT_REPO_ROOT,
    PROJECT_NAME,
    list_manifest_json_files,
    load_json,
    load_text,
    normalize_repo_relative,
    repo_path,
    resolve_repo_root,
    save_json,
    safe_stat,
)


REQUIRED_DOCS = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/manifest_retention_policy.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
]

CORE_MANIFESTS = [
    "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json",
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/manifests/manifest_retention_plan_v0_1.json",
]

EXPECTED_SKELETON_MODULES = [
    "tools/rslg_pipeline/__init__.py",
    "tools/rslg_pipeline/common.py",
    "tools/rslg_pipeline/artifact_registry.py",
    "tools/rslg_pipeline/validate_artifacts.py",
    "tools/rslg_pipeline/build_world_model.py",
    "tools/rslg_pipeline/build_stable_maps.py",
    "tools/rslg_pipeline/stable_map_schema.py",
    "tools/rslg_pipeline/build_vertical_connectors.py",
    "tools/rslg_pipeline/build_object_interfaces.py",
    "tools/rslg_pipeline/build_route_contracts.py",
    "tools/rslg_pipeline/route_contract_schema.py",
    "tools/rslg_pipeline/route_contract_promotion.py",
    "tools/rslg_pipeline/candidate_route_contract_schema.py",
    "tools/rslg_pipeline/layer3_boundary_review.py",
    "tools/rslg_pipeline/export_runtime_inputs.py",
]

CANONICAL_LAYER_NAMES = [
    "Layer 0: Input Layer",
    "Layer 1: World Model Layer",
    "Layer 2: Formal Artifact Layer",
    "Layer 3: Navigation Interface Layer",
    "Layer 4: Runtime Validation Layer",
]


def _add_check(
    checks: List[Dict[str, Any]],
    name: str,
    ok: bool,
    details: Dict[str, Any] | None = None,
    *,
    errors: List[str],
    warnings: List[str],
    error_message: str | None = None,
    warning_message: str | None = None,
) -> None:
    status = "passed" if ok else "failed"
    checks.append({"name": name, "status": status, "ok": ok, "details": details or {}})
    if not ok and error_message:
        errors.append(error_message)
    if warning_message:
        warnings.append(warning_message)


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
    for char in "-_/.,;:()[]{}\"'":
        text = text.replace(char, " ")
    return " ".join(text.split())


def _flatten_values(value: Any) -> Iterator[Any]:
    if isinstance(value, Mapping):
        for item in value.values():
            yield from _flatten_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from _flatten_values(item)
    else:
        yield value


def _contains_all_terms(values: Iterable[Any], required_terms: Iterable[str]) -> Dict[str, bool]:
    haystack = _normalize_text(" ".join(str(value) for value in values))
    return {term: _normalize_text(term) in haystack for term in required_terms}


def _read_existing_texts(repo_root: Path, paths: Iterable[str]) -> Dict[str, str]:
    texts: Dict[str, str] = {}
    for path in paths:
        target = repo_path(repo_root, path)
        if target.is_file():
            texts[path] = load_text(target)
    return texts


def validate_json_files(repo_root: Path) -> Tuple[Dict[str, Any], List[str]]:
    records = []
    errors = []
    for path in list_manifest_json_files(repo_root):
        rel_path = normalize_repo_relative(path, repo_root)
        try:
            load_json(path)
            records.append({"path": rel_path, "ok": True})
        except Exception as exc:  # pragma: no cover - defensive report path
            message = f"{rel_path}: {exc}"
            records.append({"path": rel_path, "ok": False, "error": str(exc)})
            errors.append(message)
    return {"files": records, "file_count": len(records), "ok": not errors}, errors


def check_manifest_index_references(repo_root: Path) -> Tuple[Dict[str, Any], List[str]]:
    registry = ArtifactRegistry(repo_root)
    validation = registry.validate_manifest_files(required=True)
    references = registry.summary()["manifests"]
    warnings = [
        f"Optional/planning manifest referenced by index is missing: {path}"
        for path in validation["missing_optional_or_planning"]
    ]
    return {
        "ok": bool(validation["ok"]),
        "index_path": normalize_repo_relative(registry.index_path, repo_root),
        "references": references,
        "missing_required": validation["missing_required"],
        "missing_optional_or_planning": validation["missing_optional_or_planning"],
    }, warnings


def check_required_paths(repo_root: Path, paths: Iterable[str]) -> Dict[str, Any]:
    records = [safe_stat(path, repo_root) for path in paths]
    missing = [record["path"] for record in records if not record["exists"]]
    return {"ok": not missing, "missing": missing, "paths": records}


def check_layer_naming(repo_root: Path) -> Dict[str, Any]:
    pipeline = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json"))
    layer_manifest = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json"))
    doc_texts = _read_existing_texts(
        repo_root,
        [
            "docs/rslg_slam/pipeline_architecture.md",
            "docs/rslg_slam/rslg_pipeline_skeleton.md",
            "docs/rslg_slam/rslg_pipeline_test_plan.md",
        ],
    )
    main_chain = pipeline.get("main_chain", [])
    canonical_steps = pipeline.get("canonical_run_steps", [])
    layer_names = [layer.get("layer") for layer in layer_manifest.get("layers", [])]
    allowed_context_markers = (
        "do not call",
        "must not call",
        "not call this",
        "not the stage-a layer",
        "should not call",
    )
    stage_a_layer_violations = []
    for source, text in {
        "pipeline_contract_manifest_v0_1.json": json.dumps(pipeline, sort_keys=True),
        "layer_artifacts_manifest_v0_1.json": json.dumps(layer_manifest, sort_keys=True),
        **doc_texts,
    }.items():
        for line_number, line in enumerate(text.splitlines(), start=1):
            if "Stage-A Layer" not in line:
                continue
            normalized = _normalize_text(line)
            if any(marker in normalized for marker in allowed_context_markers):
                continue
            stage_a_layer_violations.append({"source": source, "line": line_number, "text": line.strip()})
    canonical_steps_ok = {
        f"Layer {step.get('step')}: {step.get('name')}": step.get("name")
        for step in canonical_steps
        if isinstance(step, Mapping) and "step" in step and "name" in step
    }
    ok = (
        all(layer in main_chain for layer in CANONICAL_LAYER_NAMES)
        and all(layer in layer_names for layer in CANONICAL_LAYER_NAMES)
        and canonical_steps_ok.get("Layer 1: World Model Layer") == "World Model Layer"
        and not stage_a_layer_violations
    )
    return {
        "ok": ok,
        "canonical_layers_required": CANONICAL_LAYER_NAMES,
        "main_chain_has_all_canonical_layers": all(layer in main_chain for layer in CANONICAL_LAYER_NAMES),
        "main_chain_has_world_model_layer": "Layer 1: World Model Layer" in main_chain,
        "canonical_step_1_name": next((step.get("name") for step in canonical_steps if step.get("step") == 1), None),
        "layer_manifest_has_world_model_layer": "Layer 1: World Model Layer" in layer_names,
        "stage_a_layer_violations": stage_a_layer_violations,
    }


def check_project_truth(repo_root: Path) -> Dict[str, Any]:
    truth = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json"))
    index = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json"))
    project_doc = load_text(repo_path(repo_root, "docs/rslg_slam/project_contract.md"))
    repository_path = truth.get("repository_path")
    repository_path_ok = repository_path in {DEFAULT_REPO_ROOT.as_posix(), repo_root.as_posix()}
    boxfusion_note = " ".join([str(truth.get("repository_path_note", "")), project_doc])
    boxfusion_historical_only = "BoxFusion" in boxfusion_note and "historical" in boxfusion_note and "not the project name" in boxfusion_note
    input_terms = _contains_all_terms(
        list(_flatten_values(truth.get("scene_specific_external_inputs", [])))
        + list(_flatten_values(truth.get("scene_input_contract", {})))
        + [project_doc],
        ["RGB", "depth", "provided camera pose"],
    )
    forbidden_terms = _contains_all_terms(
        list(_flatten_values(truth.get("forbidden_world_model_sources", []))) + [project_doc],
        [
            "external GT floorplan",
            "external GT occupancy map",
            "manual stair centerline",
            "manual object target pose",
            "simulator navmesh",
        ],
    )
    # Accept the more explicit wording currently used in the manifest.
    if not forbidden_terms["manual stair centerline"]:
        forbidden_terms["manual stair centerline"] = _contains_all_terms(
            _flatten_values(truth.get("forbidden_world_model_sources", [])),
            ["manually drawn stair centerline"],
        )["manually drawn stair centerline"]
    if not forbidden_terms["manual object target pose"]:
        forbidden_terms["manual object target pose"] = _contains_all_terms(
            _flatten_values(truth.get("forbidden_world_model_sources", [])),
            ["manually specified object target pose"],
        )["manually specified object target pose"]
    claim_boundaries = _contains_all_terms(
        list(_flatten_values(truth.get("not_claimed_boundaries", []))) + [project_doc],
        [
            "dense reconstruction",
            "physical stair climbing",
            "footstep planning",
            "gait control",
            "contact dynamics",
            "AMCL success",
            "real robot stair climbing",
            "full object navigation benchmark",
        ],
    )
    slam_accuracy_claimed = truth.get("scene_input_contract", {}).get(
        "dataset_side_slam_or_localization_accuracy_claimed"
    )
    ok = (
        truth.get("project_name") == PROJECT_NAME
        and index.get("project_name") == PROJECT_NAME
        and repository_path_ok
        and boxfusion_historical_only
        and all(input_terms.values())
        and all(forbidden_terms.values())
        and all(claim_boundaries.values())
        and slam_accuracy_claimed is False
    )
    return {
        "ok": ok,
        "project_truth_project_name": truth.get("project_name"),
        "index_project_name": index.get("project_name"),
        "repository_path": repository_path,
        "repository_path_ok": repository_path_ok,
        "repository_path_note": truth.get("repository_path_note"),
        "boxfusion_historical_only": boxfusion_historical_only,
        "scene_specific_external_inputs": input_terms,
        "forbidden_world_model_sources": forbidden_terms,
        "not_claimed_boundaries": claim_boundaries,
        "dataset_side_slam_or_localization_accuracy_claimed": slam_accuracy_claimed,
    }


def check_stable_occupancy_map_contract(repo_root: Path) -> Dict[str, Any]:
    pipeline = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json"))
    layer_manifest = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json"))
    docs = _read_existing_texts(
        repo_root,
        [
            "docs/rslg_slam/pipeline_architecture.md",
            "docs/rslg_slam/rslg_pipeline_test_plan.md",
        ],
    )
    pipeline_contract = pipeline.get("stable_occupancy_map_contract", {})
    layer_contract = layer_manifest.get("stable_occupancy_map_contract", {})
    combined_values = [
        pipeline_contract,
        layer_contract,
        layer_manifest.get("map_distinctions", {}),
        *docs.values(),
    ]
    terms = _contains_all_terms(
        _flatten_values(combined_values),
        [
            "Layer 2: Formal Artifact Layer",
            "Layer 3: Navigation Interface Layer",
            "not external map",
            "not semantic floorplan",
            "not room mask",
            "not runtime costmap",
        ],
    )
    # The manifest also records the external-map prohibition as "external GT occupancy map".
    if not terms["not external map"]:
        terms["not external map"] = _contains_all_terms(
            _flatten_values(combined_values),
            ["external GT occupancy map"],
        )["external GT occupancy map"]
    boolean_flags = {
        "pipeline_not_external_map": pipeline_contract.get("not_external_map") is True,
        "pipeline_not_semantic_floorplan": pipeline_contract.get("not_semantic_floorplan") is True,
        "pipeline_not_room_mask": pipeline_contract.get("not_room_mask") is True,
        "pipeline_not_runtime_costmap": pipeline_contract.get("not_runtime_costmap") is True,
    }
    generation_layer_ok = pipeline_contract.get("generation_layer") == "Layer 2: Formal Artifact Layer"
    consumption_layer_ok = pipeline_contract.get("consumption_layer") == "Layer 3: Navigation Interface Layer"
    layer_generation_ok = layer_contract.get("generation_layer") == "Layer 2: Formal Artifact Layer"
    layer_consumption_ok = layer_contract.get("consumption_layer") == "Layer 3: Navigation Interface Layer"
    prohibitions_ok = {
        "not_external_map": terms["not external map"] or boolean_flags["pipeline_not_external_map"],
        "not_semantic_floorplan": terms["not semantic floorplan"] or boolean_flags["pipeline_not_semantic_floorplan"],
        "not_room_mask": terms["not room mask"] or boolean_flags["pipeline_not_room_mask"],
        "not_runtime_costmap": terms["not runtime costmap"] or boolean_flags["pipeline_not_runtime_costmap"],
    }
    ok = (
        generation_layer_ok
        and consumption_layer_ok
        and layer_generation_ok
        and layer_consumption_ok
        and terms["Layer 2: Formal Artifact Layer"]
        and terms["Layer 3: Navigation Interface Layer"]
        and all(prohibitions_ok.values())
        and all(boolean_flags.values())
    )
    return {
        "ok": ok,
        "pipeline_generation_layer": pipeline_contract.get("generation_layer"),
        "pipeline_consumption_layer": pipeline_contract.get("consumption_layer"),
        "layer_generation_layer": layer_contract.get("generation_layer"),
        "layer_consumption_layer": layer_contract.get("consumption_layer"),
        "required_terms_present": terms,
        "boolean_flags": boolean_flags,
        "prohibitions_ok": prohibitions_ok,
    }


def _find_dicts_with_key(value: Any, key: str) -> Iterator[Mapping[str, Any]]:
    if isinstance(value, Mapping):
        if key in value:
            yield value
        for item in value.values():
            yield from _find_dicts_with_key(item, key)
    elif isinstance(value, list):
        for item in value:
            yield from _find_dicts_with_key(item, key)


def check_validated_milestone_truth(repo_root: Path) -> Dict[str, Any]:
    manifest = load_json(repo_path(repo_root, "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json"))
    transition_records = list(_find_dicts_with_key(manifest, "transition_edge"))
    object_records = [
        record
        for record in _find_dicts_with_key(manifest, "object_id")
        if record.get("object_id") == "obj_175" or record.get("approach_candidate") == "generated_ring_037"
    ]
    transition_ok = any(
        record.get("transition_edge") == "vt_1_centerline_e001"
        and record.get("not_transition_edge") == "vt_1_centerline_e003"
        for record in transition_records
    )
    object_ok = any(
        record.get("object_id") == "obj_175"
        and record.get("approach_candidate") == "generated_ring_037"
        and record.get("object_centroid_navigation_used") is False
        for record in object_records
    )
    return {
        "ok": transition_ok and object_ok,
        "transition_edge_truth_present": transition_ok,
        "object_smoke_truth_present": object_ok,
        "transition_records_checked": len(transition_records),
        "object_records_checked": len(object_records),
    }


def check_stage_outputs_independence(repo_root: Path) -> Dict[str, Any]:
    stage_outputs = repo_path(repo_root, "stage_outputs")
    allowed_task25h_evidence = repo_path(
        repo_root,
        "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task25h_first_real_static_validator_integration",
    )
    return {
        "ok": True,
        "stage_outputs_required": False,
        "stage_outputs_exists": stage_outputs.exists(),
        "task25h_evidence_path_exists": allowed_task25h_evidence.exists(),
        "historical_stage_outputs_required": False,
        "policy": "Static validation does not require historical generated outputs.",
    }


def check_no_runtime_guard() -> Dict[str, Any]:
    denied_modules = ["rclpy", "launch", "gazebo", "rviz2", "nav2_simple_commander"]
    imported = [name for name in denied_modules if name in sys.modules]
    return {
        "ok": not imported,
        "runtime_modules_imported": imported,
        "runtime_launched": False,
        "policy": "Validator uses only Python standard-library/static project readers.",
    }


def check_no_world_model_rerun_guard() -> Dict[str, Any]:
    imported = [name for name in sys.modules if name == "boxfusion.stage_a_demo" or name.endswith(".stage_a_demo")]
    return {
        "ok": not imported,
        "stage_a_modules_imported": imported,
        "world_model_rerun": False,
        "stage_a_demo_invoked": False,
    }


def check_retention_policy(repo_root: Path) -> Dict[str, Any]:
    plan_path = repo_path(repo_root, "docs/rslg_slam/manifests/manifest_retention_plan_v0_1.json")
    policy_path = repo_path(repo_root, "docs/rslg_slam/manifest_retention_policy.md")
    plan_exists = plan_path.is_file()
    policy_exists = policy_path.is_file()
    plan = load_json(plan_path) if plan_exists else {}
    policy_text = load_text(policy_path) if policy_exists else ""
    plan_policy = plan.get("ordinary_task_manifest_policy", {})
    plan_allows_merge_or_delete = bool(plan_policy.get("temporary_planning_manifests_should_be_merged_or_deleted_when_complete"))
    policy_allows_merge_or_delete = all(
        term in _normalize_text(policy_text)
        for term in ["planning files", "merged", "deleted"]
    )
    return {
        "ok": (plan_exists or policy_exists) and (plan_allows_merge_or_delete or policy_allows_merge_or_delete),
        "manifest_retention_plan_exists": plan_exists,
        "manifest_retention_policy_exists": policy_exists,
        "plan_allows_merge_or_delete": plan_allows_merge_or_delete,
        "policy_allows_merge_or_delete": policy_allows_merge_or_delete,
    }


def check_tool_skeleton(repo_root: Path) -> Dict[str, Any]:
    records = [safe_stat(path, repo_root) for path in EXPECTED_SKELETON_MODULES]
    missing = [record["path"] for record in records if not record["exists"]]
    return {"ok": not missing, "missing": missing, "modules": records}


def check_protected_path_policy(repo_root: Path) -> Tuple[Dict[str, Any], List[str]]:
    manifest_path = repo_path(repo_root, "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json")
    if not manifest_path.is_file():
        return {"ok": True, "skipped": True, "reason": "protected assets manifest missing"}, []

    manifest = load_json(manifest_path)
    records = []
    warnings = []
    active_assets = manifest.get("active_protected_assets", manifest.get("protected_assets", []))
    historical_assets = manifest.get("historical_generated_evidence_paths", [])
    for asset in active_assets:
        path = asset.get("path")
        if not path:
            continue
        stat = safe_stat(path, repo_root)
        record = {
            "path": path,
            "ordinary_tasks_may_modify": asset.get("ordinary_tasks_may_modify"),
            "manifest_expected_exists": asset.get("exists"),
            "actual_exists": stat["exists"],
            "policy": asset.get("policy"),
        }
        records.append(record)
        if asset.get("exists") is True and not stat["exists"]:
            warnings.append(f"Active protected asset is absent: {path}")
    historical_records = []
    for asset in historical_assets:
        path = asset.get("path")
        if not path:
            continue
        stat = safe_stat(path, repo_root)
        historical_records.append(
            {
                "path": path,
                "actual_exists": stat["exists"],
                "required_for_static_validation": bool(asset.get("required_for_static_validation", False)),
                "policy": asset.get("policy"),
            }
        )
    return {
        "ok": True,
        "active_records": records,
        "historical_generated_records": historical_records,
        "warning_count": len(warnings),
    }, warnings


def run_static_validation(repo_root: Path) -> Dict[str, Any]:
    checks: List[Dict[str, Any]] = []
    errors: List[str] = []
    warnings: List[str] = []

    docs_check = check_required_paths(repo_root, REQUIRED_DOCS)
    _add_check(
        checks,
        "required_docs_exist",
        bool(docs_check["ok"]),
        docs_check,
        errors=errors,
        warnings=warnings,
        error_message="One or more required RSLG-SLAM docs are missing.",
    )

    manifest_check = check_required_paths(repo_root, CORE_MANIFESTS)
    _add_check(
        checks,
        "core_manifests_exist",
        bool(manifest_check["ok"]),
        manifest_check,
        errors=errors,
        warnings=warnings,
        error_message="One or more core RSLG-SLAM manifests are missing.",
    )

    json_report, json_errors = validate_json_files(repo_root)
    errors.extend(json_errors)
    _add_check(
        checks,
        "manifest_json_load_validation",
        bool(json_report["ok"]),
        json_report,
        errors=errors,
        warnings=warnings,
    )

    if errors:
        classification = "rslg_static_validation_failed"
        summary = {
            "classification": classification,
            "error_count": len(errors),
            "warning_count": len(warnings),
            "stage_outputs_required_for_validation": False,
            "world_model_rerun": False,
            "runtime_launched": False,
        }
        checks.append(
            {
                "name": "dependent_static_contract_checks",
                "status": "skipped",
                "ok": False,
                "details": {
                    "reason": "Skipped because required docs/manifests are missing or one or more manifest JSON files failed to parse."
                },
            }
        )
        return {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "mode": "static",
            "ok": False,
            "classification": classification,
            **summary,
            "checks": checks,
            "errors": errors,
            "warnings": warnings,
            "summary": summary,
        }

    registry_validation, registry_warnings = check_manifest_index_references(repo_root)
    warnings.extend(registry_warnings)
    _add_check(
        checks,
        "manifest_index_reference_validation",
        bool(registry_validation["ok"]),
        registry_validation,
        errors=errors,
        warnings=warnings,
        error_message="One or more core manifest index references are missing.",
    )

    layer_check = check_layer_naming(repo_root)
    _add_check(
        checks,
        "layer_naming_check",
        bool(layer_check["ok"]),
        layer_check,
        errors=errors,
        warnings=warnings,
        error_message="Layer naming check failed; Layer 1 must be World Model Layer.",
    )

    truth_check = check_project_truth(repo_root)
    _add_check(
        checks,
        "project_truth_check",
        bool(truth_check["ok"]),
        truth_check,
        errors=errors,
        warnings=warnings,
        error_message="Project truth check failed; verify name, path, inputs, forbidden sources, and claim boundaries.",
    )

    stable_map_check = check_stable_occupancy_map_contract(repo_root)
    _add_check(
        checks,
        "stable_occupancy_map_contract_check",
        bool(stable_map_check["ok"]),
        stable_map_check,
        errors=errors,
        warnings=warnings,
        error_message="Stable occupancy map contract check failed.",
    )

    milestone_check = check_validated_milestone_truth(repo_root)
    _add_check(
        checks,
        "validated_milestone_truth_check",
        bool(milestone_check["ok"]),
        milestone_check,
        errors=errors,
        warnings=warnings,
        error_message="Validated milestone truth check failed.",
    )

    stage_outputs_check = check_stage_outputs_independence(repo_root)
    _add_check(
        checks,
        "stage_outputs_independence_check",
        True,
        stage_outputs_check,
        errors=errors,
        warnings=warnings,
    )

    runtime_guard = check_no_runtime_guard()
    _add_check(
        checks,
        "no_runtime_guard",
        bool(runtime_guard["ok"]),
        runtime_guard,
        errors=errors,
        warnings=warnings,
        error_message="Runtime guard failed; runtime modules were imported.",
    )

    world_model_guard = check_no_world_model_rerun_guard()
    _add_check(
        checks,
        "no_world_model_rerun_guard",
        bool(world_model_guard["ok"]),
        world_model_guard,
        errors=errors,
        warnings=warnings,
        error_message="World Model rerun guard failed; Stage-A module was imported.",
    )

    retention_check = check_retention_policy(repo_root)
    _add_check(
        checks,
        "retention_policy_check",
        bool(retention_check["ok"]),
        retention_check,
        errors=errors,
        warnings=warnings,
        error_message="Retention policy check failed.",
    )

    skeleton_check = check_tool_skeleton(repo_root)
    _add_check(
        checks,
        "tool_skeleton_check",
        bool(skeleton_check["ok"]),
        skeleton_check,
        errors=errors,
        warnings=warnings,
        error_message="Expected tools/rslg_pipeline skeleton modules are missing.",
    )

    protected_check, protected_warnings = check_protected_path_policy(repo_root)
    warnings.extend(protected_warnings)
    _add_check(
        checks,
        "protected_path_policy_consistency_warning_only",
        True,
        protected_check,
        errors=errors,
        warnings=warnings,
    )

    ok = not errors
    classification = "rslg_static_validation_passed" if ok else "rslg_static_validation_failed"
    summary = {
        "classification": classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "stage_outputs_required_for_validation": False,
        "world_model_rerun": False,
        "runtime_launched": False,
    }
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "mode": "static",
        "ok": ok,
        "classification": classification,
        **summary,
        "checks": checks,
        "errors": errors,
        "warnings": warnings,
        "summary": summary,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run static RSLG-SLAM artifact validators.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--mode", default="static", choices=["static"], help="Validation mode.")
    parser.add_argument("--output", default=None, help="Optional JSON report output path.")
    parser.add_argument("--output-json", default=None, help="Optional JSON report output path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    report = run_static_validation(repo_root)
    output_path = args.output_json or args.output
    if output_path:
        save_json(repo_path(repo_root, output_path), report)
    print(json.dumps(report["summary"], indent=2, sort_keys=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
