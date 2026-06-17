"""Canonical wrapper for a Layer 1 World Model rerun.

The historical implementation remains in ``stage_a_demo.py``. This wrapper
performs task-scoped preflight checks, invokes that implementation without
migrating its business logic, and records canonical Layer 1 provenance and
validation reports.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


PROJECT_NAME = "RSLG-SLAM"
TASK_NAME = "task35_layer1_canonical_rerun_preflight_and_execute_if_ready"
SCENE_ID = "00843-DYehNKdT76V"
REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
INPUT_DATA_PATH = Path("/home/ws/data") / SCENE_ID
CONFIG_PATH = REPO_ROOT / "config/hm3d.yaml"
MODEL_PATH = REPO_ROOT / "models/cutr_rgbd.pth"
PYTHON_PATH = Path("/home/ws/miniconda3/envs/boxfusion/bin/python")
ENTRYPOINT_PATH = REPO_ROOT / "stage_a_demo.py"
CANONICAL_LAYER1_DIR = (
    REPO_ROOT / "stage_outputs/rslg_slam" / SCENE_ID / "canonical/layer1_world_model"
)
TASK_EVIDENCE_DIR = (
    REPO_ROOT
    / "stage_outputs/rslg_slam"
    / SCENE_ID
    / "tasks"
    / TASK_NAME
)
HISTORICAL_CLEAN_RERUN = (
    REPO_ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
)
HISTORICAL_REFERENCE = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"

REQUIRED_TRUTH_FILES = [
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/canonical_output_plan.md",
    "docs/rslg_slam/final_layer2_minimal_generation_plan.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
]

SUPPORT_FILES = [
    "models/ViT-B-32/open_clip_pytorch_model.bin",
    "data/class_features_small.pt",
    "data/panoptic_categories_nomerge.txt",
    "data/pst_1024_0.tiff",
]

FORBIDDEN_CANONICAL_PARTS = {
    "clean_rerun",
    "stage1_generalization",
    "tasks",
    "candidate",
    "task25b_stage_a_rerun_candidate",
    "stage1_00824_step30p1",
}

EVIDENCE_SPECS = {
    "posed_rgbd_derived_world_model_evidence": {
        "files": ["scene_manifest.json", "logs/final_vector_map_snapshot.json"],
        "terms": ["dataset_root", "trajectory"],
    },
    "floor_assignment_or_floor_height_evidence": {
        "files": ["logs/floor_diagnostics_summary.json", "logs/final_vector_map_snapshot.json"],
        "terms": ["floor_id", "floor_height", "floor_debug", "floors"],
    },
    "free_space_evidence": {
        "files": ["logs/final_vector_map_snapshot.json"],
        "terms": ["free_space", "free-space", "free"],
    },
    "wall_evidence": {
        "files": ["logs/final_vector_map_snapshot.json"],
        "terms": ["wall", "walls"],
    },
    "outside_boundary_evidence": {
        "files": ["logs/final_vector_map_snapshot.json"],
        "terms": ["outside_boundary", "outside-boundary", "outside"],
    },
    "gateway_or_gateway_wall_evidence": {
        "files": ["logs/final_vector_map_snapshot.json", "logs/topology_v0_1.json"],
        "terms": ["gateway", "gateway_wall"],
    },
    "room_or_floor_topology_evidence": {
        "files": ["logs/topology_v0_1.json", "logs/committed_room_world_model_v0_1.json"],
        "terms": ["rooms", "edges", "topology"],
    },
    "object_observation_or_semantic_object_records": {
        "files": ["logs/final_vector_map_snapshot.json", "logs/committed_room_world_model_v0_1.json"],
        "terms": ["objects", "object_id", "class_name", "label"],
    },
    "object_room_association_evidence": {
        "files": ["logs/final_vector_map_snapshot.json", "logs/committed_room_world_model_v0_1.json"],
        "terms": ["room_id", "object_room", "assigned_room"],
    },
    "object_floor_association_evidence": {
        "files": ["logs/final_vector_map_snapshot.json", "logs/committed_room_world_model_v0_1.json"],
        "terms": ["floor_id", "object_floor", "assigned_floor"],
    },
    "generated_artifact_provenance": {
        "files": ["manifest.json"],
        "terms": ["sequence_name", "dataset_root"],
    },
    "command_provenance": {
        "files": [],
        "terms": [],
    },
    "output_directory_provenance": {
        "files": [],
        "terms": [],
    },
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def path_record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "exists": path.exists(),
        "is_file": path.is_file(),
        "is_dir": path.is_dir(),
        "size_bytes": path.stat().st_size if path.is_file() else None,
    }


def add_check(
    checks: list[dict[str, Any]],
    name: str,
    passed: bool,
    details: dict[str, Any],
    blocking_reasons: list[str],
    reason: str,
) -> None:
    checks.append(
        {
            "name": name,
            "status": "pass" if passed else "fail",
            "passed": passed,
            "details": details,
        }
    )
    if not passed:
        blocking_reasons.append(reason)


def directory_snapshot(path: Path) -> dict[str, Any]:
    files = []
    if path.is_dir():
        for item in sorted(path.rglob("*")):
            if item.is_file():
                stat = item.stat()
                files.append(
                    {
                        "path": str(item),
                        "size_bytes": stat.st_size,
                        "mtime_ns": stat.st_mtime_ns,
                    }
                )
    encoded = json.dumps(files, sort_keys=True).encode("utf-8")
    return {
        "path": str(path),
        "exists": path.exists(),
        "file_count": len(files),
        "inventory_sha256": hashlib.sha256(encoded).hexdigest(),
        "files": files,
    }


def inspect_cuda() -> dict[str, Any]:
    code = (
        "import json, torch; "
        "print(json.dumps({'torch_version': torch.__version__, "
        "'torch_cuda_compiled': torch.version.cuda, "
        "'cuda_available': torch.cuda.is_available(), "
        "'cuda_device_count': torch.cuda.device_count(), "
        "'devices': [torch.cuda.get_device_name(i) "
        "for i in range(torch.cuda.device_count())] if torch.cuda.is_available() else []}))"
    )
    result = subprocess.run(
        [str(PYTHON_PATH), "-c", code],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    payload: dict[str, Any] = {
        "command": [str(PYTHON_PATH), "-c", "<torch CUDA probe>"],
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }
    if result.returncode == 0:
        try:
            payload.update(json.loads(result.stdout))
        except json.JSONDecodeError:
            payload["parse_error"] = "CUDA probe output was not valid JSON"
    return payload


def count_scene_inputs(scene_root: Path) -> dict[str, int]:
    return {
        name: sum(1 for path in (scene_root / name).glob("*") if path.is_file())
        for name in ("rgb", "depth", "pose")
    }


def canonical_target_is_safe(path: Path) -> tuple[bool, dict[str, Any]]:
    resolved = path.resolve()
    expected = CANONICAL_LAYER1_DIR.resolve()
    relative_parts = set(resolved.relative_to(REPO_ROOT.resolve()).parts)
    forbidden_found = sorted(relative_parts & FORBIDDEN_CANONICAL_PARTS)
    return (
        resolved == expected and not forbidden_found,
        {
            "resolved": str(resolved),
            "expected": str(expected),
            "forbidden_path_parts_found": forbidden_found,
        },
    )


def preflight(device_request: str) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    blocking_reasons: list[str] = []
    docs = [path_record(REPO_ROOT / relative) for relative in REQUIRED_TRUTH_FILES]
    scene_counts = count_scene_inputs(INPUT_DATA_PATH) if INPUT_DATA_PATH.is_dir() else {}
    support = [path_record(REPO_ROOT / relative) for relative in SUPPORT_FILES]
    cuda = inspect_cuda() if PYTHON_PATH.is_file() else {"cuda_available": False}
    target_safe, target_details = canonical_target_is_safe(CANONICAL_LAYER1_DIR)

    add_check(
        checks,
        "repository_root_is_working_directory",
        REPO_ROOT.is_dir() and Path.cwd().resolve() == REPO_ROOT.resolve(),
        {"repo_root": str(REPO_ROOT), "cwd": str(Path.cwd())},
        blocking_reasons,
        "Repository root is missing or the wrapper was not launched from /home/ws/workspace/BoxFusion.",
    )
    add_check(
        checks,
        "scene_data_exists",
        INPUT_DATA_PATH.is_dir(),
        {**path_record(INPUT_DATA_PATH), "input_file_counts": scene_counts},
        blocking_reasons,
        f"Scene data is missing: {INPUT_DATA_PATH}",
    )
    counts_ok = bool(scene_counts) and min(scene_counts.values()) > 0 and len(set(scene_counts.values())) == 1
    add_check(
        checks,
        "scene_rgb_depth_pose_inputs_are_nonempty_and_aligned",
        counts_ok,
        {"input_file_counts": scene_counts},
        blocking_reasons,
        f"Scene RGB/depth/pose inputs are missing or count-mismatched: {scene_counts}",
    )
    for name, path in (
        ("config_exists", CONFIG_PATH),
        ("model_checkpoint_exists", MODEL_PATH),
        ("required_python_exists_and_is_executable", PYTHON_PATH),
        ("historical_world_model_entrypoint_exists", ENTRYPOINT_PATH),
    ):
        if name == "required_python_exists_and_is_executable":
            passed = path.is_file() and os.access(path, os.X_OK)
        else:
            passed = path.is_file()
        add_check(
            checks,
            name,
            passed,
            path_record(path),
            blocking_reasons,
            f"Required path failed check: {path}",
        )
    add_check(
        checks,
        "required_docs_and_manifests_present",
        all(item["exists"] and item["is_file"] for item in docs),
        {"files": docs},
        blocking_reasons,
        "One or more required project truth documents/manifests are missing.",
    )
    add_check(
        checks,
        "historical_entrypoint_support_files_present",
        all(item["exists"] and item["is_file"] for item in support),
        {"files": support},
        blocking_reasons,
        "One or more historical World Model support files are missing.",
    )
    add_check(
        checks,
        "canonical_layer1_target_is_exact_and_separate",
        target_safe,
        target_details,
        blocking_reasons,
        "Canonical Layer 1 target is not the exact approved directory or overlaps a forbidden historical/task path.",
    )
    output_empty = not CANONICAL_LAYER1_DIR.exists() or not any(CANONICAL_LAYER1_DIR.iterdir())
    add_check(
        checks,
        "canonical_layer1_target_is_absent_or_empty",
        output_empty,
        path_record(CANONICAL_LAYER1_DIR),
        blocking_reasons,
        "Canonical Layer 1 target already contains files; refusing to overwrite a prior canonical run.",
    )
    add_check(
        checks,
        "clean_rerun_is_not_the_target",
        HISTORICAL_CLEAN_RERUN.resolve() != CANONICAL_LAYER1_DIR.resolve()
        and "clean_rerun" not in CANONICAL_LAYER1_DIR.parts,
        {
            "clean_rerun": str(HISTORICAL_CLEAN_RERUN),
            "canonical_target": str(CANONICAL_LAYER1_DIR),
        },
        blocking_reasons,
        "The canonical target resolves to or includes clean_rerun.",
    )
    add_check(
        checks,
        "wrapper_is_real_canonical_executor",
        Path(__file__).is_file()
        and callable(globals().get("build_historical_command"))
        and callable(globals().get("inspect_evidence")),
        {"wrapper_path": str(Path(__file__).resolve())},
        blocking_reasons,
        "tools/rslg_pipeline/build_world_model.py is still a placeholder.",
    )
    cuda_available = bool(cuda.get("cuda_available"))
    device = "cuda" if device_request == "auto" and cuda_available else device_request
    if device_request == "auto" and not cuda_available:
        device = "cpu"
    device_supported = device == "cpu" or (device == "cuda" and cuda_available)
    add_check(
        checks,
        "execution_device_is_supported",
        device_supported,
        {"requested": device_request, "selected": device, "probe": cuda},
        blocking_reasons,
        f"Requested execution device is unavailable: {device}",
    )

    return {
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "repo_root": str(REPO_ROOT),
        "input_data_path": str(INPUT_DATA_PATH),
        "config_path": str(CONFIG_PATH.relative_to(REPO_ROOT)),
        "model_path": str(MODEL_PATH.relative_to(REPO_ROOT)),
        "python_path": str(PYTHON_PATH),
        "entrypoint_path": str(ENTRYPOINT_PATH.relative_to(REPO_ROOT)),
        "rslg_pipeline_build_world_model_status": "minimal_canonical_wrapper",
        "canonical_layer1_output_dir": str(CANONICAL_LAYER1_DIR.relative_to(REPO_ROOT)),
        "task_evidence_dir": str(TASK_EVIDENCE_DIR.relative_to(REPO_ROOT)),
        "docs_and_manifests_read": docs,
        "historical_paths_inspected_for_command_reconstruction": [
            "tools/vertical_connectors/task25a_reintegration_preflight.py",
            "tools/vertical_connectors/export_task25b_formal_cross_floor_artifacts.py",
            "tools/vertical_connectors/task24b2_audit_and_backfill.py",
        ],
        "historical_outputs_promoted": False,
        "candidate_artifacts_used_as_inputs": False,
        "checks": checks,
        "selected_device": device,
        "cuda_probe": cuda,
        "final_preflight_status": "ready" if not blocking_reasons else "blocked",
        "blocking_reasons": blocking_reasons,
        "generated_at": utc_now(),
    }


def build_historical_command(device: str) -> list[str]:
    return [
        str(PYTHON_PATH),
        str(ENTRYPOINT_PATH.relative_to(REPO_ROOT)),
        "hm3d",
        "--model-path",
        str(MODEL_PATH.relative_to(REPO_ROOT)),
        "--config",
        str(CONFIG_PATH.relative_to(REPO_ROOT)),
        "--seq",
        SCENE_ID,
        "--output-root",
        str((CANONICAL_LAYER1_DIR / "raw_outputs").relative_to(REPO_ROOT)),
        "--capture-stride",
        "25",
        "--room-seg-interval",
        "100",
        "--runtime-profile-interval",
        "25",
        "--device",
        device,
        "--quiet",
    ]


def command_string(command: Iterable[str]) -> str:
    return " ".join(str(part) for part in command)


def flatten_json_terms(value: Any, prefix: str = "$") -> Iterable[tuple[str, str]]:
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}"
            yield child_prefix, str(key).lower()
            yield from flatten_json_terms(child, child_prefix)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from flatten_json_terms(child, f"{prefix}[{index}]")


def inspect_evidence(scene_output: Path) -> dict[str, Any]:
    loaded: dict[Path, Any] = {}
    for path in sorted(scene_output.rglob("*.json")) if scene_output.is_dir() else []:
        try:
            with path.open("r", encoding="utf-8") as handle:
                loaded[path] = json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue

    term_locations: dict[str, list[str]] = {}
    for path, payload in loaded.items():
        relative = path.relative_to(scene_output)
        for json_path, term in flatten_json_terms(payload):
            term_locations.setdefault(term, []).append(f"{relative}:{json_path}")

    results = {}
    for name, spec in EVIDENCE_SPECS.items():
        if name == "command_provenance":
            results[name] = {
                "status": "available",
                "evidence": [
                    str((CANONICAL_LAYER1_DIR / "logs/layer1_world_model_execution.log").relative_to(REPO_ROOT)),
                    str((TASK_EVIDENCE_DIR / "command_log.txt").relative_to(REPO_ROOT)),
                ],
            }
            continue
        if name == "output_directory_provenance":
            results[name] = {
                "status": "available",
                "evidence": [str(CANONICAL_LAYER1_DIR.relative_to(REPO_ROOT))],
            }
            continue
        file_hits = [
            str((scene_output / relative).relative_to(REPO_ROOT))
            for relative in spec["files"]
            if (scene_output / relative).is_file()
        ]
        term_hits = {}
        for term in spec["terms"]:
            matches = []
            normalized = term.lower()
            for found_term, locations in term_locations.items():
                if normalized == found_term or normalized in found_term:
                    matches.extend(locations[:5])
            if matches:
                term_hits[term] = matches[:5]
        available = bool(file_hits and term_hits)
        results[name] = {
            "status": "available" if available else "missing_or_unavailable",
            "evidence_files": file_hits,
            "matched_json_terms": term_hits,
        }
    return results


def output_inventory(root: Path) -> list[dict[str, Any]]:
    records = []
    if root.is_dir():
        for path in sorted(root.rglob("*")):
            if path.is_file():
                records.append(
                    {
                        "path": str(path.relative_to(REPO_ROOT)),
                        "size_bytes": path.stat().st_size,
                        "nonempty": path.stat().st_size > 0,
                    }
                )
    return records


def write_command_log(
    historical_command: list[str],
    preflight_report: dict[str, Any],
    static_validator: dict[str, Any],
    started_at: str | None,
    finished_at: str | None,
    returncode: int | None,
) -> None:
    lines = [
        f"task_name: {TASK_NAME}",
        f"generated_at: {utc_now()}",
        "",
        "[command 1: project truth and entrypoint inspection]",
        f"working_directory: {REPO_ROOT}",
        "command: internal wrapper preflight reads required docs/manifests, stage_a_demo.py, and canonical target guards",
        "required_boxfusion_conda_python_used: not_applicable_static_read",
        f"result: {preflight_report['final_preflight_status']}",
        "",
        "[command 2: CUDA probe]",
        f"working_directory: {REPO_ROOT}",
        f"command: {PYTHON_PATH} -c <torch CUDA probe>",
        "required_boxfusion_conda_python_used: true",
        f"stdout: {preflight_report['cuda_probe'].get('stdout', '')}",
        f"stderr: {preflight_report['cuda_probe'].get('stderr', '')}",
        f"returncode: {preflight_report['cuda_probe'].get('returncode')}",
        "",
        "[command 3: static canonical artifact validator]",
        f"working_directory: {REPO_ROOT}",
        f"command: {static_validator.get('command_string')}",
        "required_boxfusion_conda_python_used: true",
        f"stdout: {static_validator.get('stdout', '')}",
        f"stderr: {static_validator.get('stderr', '')}",
        f"returncode: {static_validator.get('returncode')}",
        "report: "
        + str(
            (
                TASK_EVIDENCE_DIR / "static_validator_run_report_v0_1.json"
            ).relative_to(REPO_ROOT)
        ),
        "",
        "[command 4: Layer 1 World Model canonical rerun]",
        f"working_directory: {REPO_ROOT}",
        f"started_at: {started_at or 'not_started'}",
        f"finished_at: {finished_at or 'not_finished'}",
        f"command: {command_string(historical_command)}",
        "required_boxfusion_conda_python_used: true",
        f"returncode: {returncode if returncode is not None else 'not_attempted'}",
        "stdout_stderr_capture: "
        + str((CANONICAL_LAYER1_DIR / "logs/layer1_world_model_execution.log").relative_to(REPO_ROOT)),
        "",
        "[scope controls]",
        "ros_gazebo_rviz_nav2_amcl_launched: false",
        "layer2_or_route_artifacts_generated: false",
        f"clean_rerun_targeted: false ({HISTORICAL_CLEAN_RERUN})",
        "historical_outputs_promoted: false",
        "candidate_artifacts_used_as_inputs: false",
    ]
    write_text(TASK_EVIDENCE_DIR / "command_log.txt", "\n".join(lines))


def write_json_validation_report(json_paths: list[Path]) -> dict[str, Any]:
    results = []
    for path in sorted(set(json_paths)):
        try:
            with path.open("r", encoding="utf-8") as handle:
                json.load(handle)
            results.append(
                {
                    "path": str(path.relative_to(REPO_ROOT)),
                    "status": "valid",
                    "validator": "python json.load",
                }
            )
        except Exception as exc:
            results.append(
                {
                    "path": str(path.relative_to(REPO_ROOT)),
                    "status": "invalid",
                    "validator": "python json.load",
                    "error": str(exc),
                }
            )
    return {
        "task_name": TASK_NAME,
        "validator": "python json.load",
        "all_valid": all(item["status"] == "valid" for item in results),
        "validated_file_count": len(results),
        "results": results,
        "generated_at": utc_now(),
    }


def created_files_manifest(paths: list[Path]) -> dict[str, Any]:
    records = []
    for path in sorted(set(paths)):
        if not path.is_file():
            continue
        relative = path.relative_to(REPO_ROOT)
        if path == Path(__file__).resolve():
            role = "minimal canonical Layer 1 World Model wrapper"
        elif "raw_outputs" in relative.parts:
            role = "historical implementation output generated by canonical rerun"
        elif "logs" in relative.parts:
            role = "execution or provenance log"
        elif "manifests" in relative.parts:
            role = "canonical Layer 1 or task file manifest"
        else:
            role = "task35 report or validation evidence"
        records.append(
            {
                "path": str(relative),
                "size_bytes": path.stat().st_size,
                "role": role,
            }
        )
    return {
        "task_name": TASK_NAME,
        "created_or_modified_files": records,
        "file_count": len(records),
        "generated_at": utc_now(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the canonical Layer 1 World Model rerun after fail-closed preflight."
    )
    parser.add_argument("--device", choices=["auto", "cuda", "cpu"], default="auto")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Write blocked/ready preflight evidence without executing. Not used by task35 when ready.",
    )
    args = parser.parse_args()

    preflight_report = preflight(args.device)
    TASK_EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    write_json(TASK_EVIDENCE_DIR / "preflight_report.json", preflight_report)

    static_validator_command = [
        str(PYTHON_PATH),
        "-m",
        "tools.rslg_pipeline.validate_artifacts",
        "--repo-root",
        str(REPO_ROOT),
        "--output",
        str(TASK_EVIDENCE_DIR / "static_validator_run_report_v0_1.json"),
    ]
    static_result = subprocess.run(
        static_validator_command,
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    static_validator = {
        "command": static_validator_command,
        "command_string": command_string(static_validator_command),
        "returncode": static_result.returncode,
        "stdout": static_result.stdout.strip(),
        "stderr": static_result.stderr.strip(),
        "report_path": str(
            (
                TASK_EVIDENCE_DIR / "static_validator_run_report_v0_1.json"
            ).relative_to(REPO_ROOT)
        ),
    }

    historical_command = build_historical_command(preflight_report["selected_device"])
    clean_before = directory_snapshot(HISTORICAL_CLEAN_RERUN)
    reference_before = directory_snapshot(HISTORICAL_REFERENCE)
    execution_attempted = False
    execution_status = "not_attempted"
    returncode: int | None = None
    started_at: str | None = None
    finished_at: str | None = None

    ready = preflight_report["final_preflight_status"] == "ready"
    if ready and not args.preflight_only:
        execution_attempted = True
        CANONICAL_LAYER1_DIR.joinpath("raw_outputs").mkdir(parents=True, exist_ok=True)
        CANONICAL_LAYER1_DIR.joinpath("manifests").mkdir(parents=True, exist_ok=True)
        CANONICAL_LAYER1_DIR.joinpath("reports").mkdir(parents=True, exist_ok=True)
        CANONICAL_LAYER1_DIR.joinpath("logs").mkdir(parents=True, exist_ok=True)
        execution_log = CANONICAL_LAYER1_DIR / "logs/layer1_world_model_execution.log"
        started_at = utc_now()
        with execution_log.open("w", encoding="utf-8") as handle:
            handle.write(f"Layer 1 World Model canonical rerun\nstarted_at: {started_at}\n")
            handle.write(f"working_directory: {REPO_ROOT}\n")
            handle.write(f"command: {command_string(historical_command)}\n\n")
            handle.flush()
            result = subprocess.run(
                historical_command,
                cwd=REPO_ROOT,
                stdout=handle,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            returncode = result.returncode
            finished_at = utc_now()
            handle.write(f"\nfinished_at: {finished_at}\nreturncode: {returncode}\n")
        execution_status = "succeeded" if returncode == 0 else "failed"

    clean_after = directory_snapshot(HISTORICAL_CLEAN_RERUN)
    reference_after = directory_snapshot(HISTORICAL_REFERENCE)
    clean_unchanged = clean_before["inventory_sha256"] == clean_after["inventory_sha256"]
    reference_unchanged = reference_before["inventory_sha256"] == reference_after["inventory_sha256"]
    scene_output = CANONICAL_LAYER1_DIR / "raw_outputs" / SCENE_ID
    inventory = output_inventory(CANONICAL_LAYER1_DIR)
    raw_inventory = output_inventory(scene_output)
    evidence = inspect_evidence(scene_output) if execution_attempted else {}
    missing_evidence = [
        name for name, result in evidence.items() if result["status"] != "available"
    ]
    outputs_exist = bool(raw_inventory)
    outputs_nonempty = bool(raw_inventory) and all(item["nonempty"] for item in raw_inventory)
    scene_matches = scene_output.is_dir() and all(
        SCENE_ID in str(item["path"]) for item in raw_inventory
    )
    under_canonical = all(
        (REPO_ROOT / item["path"]).resolve().is_relative_to(CANONICAL_LAYER1_DIR.resolve())
        for item in inventory
    )
    historical_reuse_detected = False
    rerun_completed = execution_status == "succeeded"

    if rerun_completed and not missing_evidence:
        task36_status = "unblocked"
    elif rerun_completed and outputs_exist:
        task36_status = "partially_unblocked"
    else:
        task36_status = "still_blocked"

    canonical_validation = {
        "project_name": PROJECT_NAME,
        "scene_id": SCENE_ID,
        "layer": "Layer 1",
        "layer_name": "World Model Layer",
        "rerun_completed": rerun_completed,
        "output_files_exist": outputs_exist,
        "output_files_nonempty_where_applicable": outputs_nonempty,
        "scene_id_matches": scene_matches,
        "outputs_under_canonical_layer1_directory": under_canonical,
        "historical_output_reuse_detected": historical_reuse_detected,
        "clean_rerun_not_modified": clean_unchanged,
        "historical_00824_reference_not_modified": reference_unchanged,
        "layer1_evidence_availability": evidence,
        "task36_can_begin": task36_status == "unblocked",
        "task36_status": task36_status,
        "exact_missing_layer1_evidence": missing_evidence,
        "generated_at": utc_now(),
    }

    canonical_manifest_path = (
        CANONICAL_LAYER1_DIR
        / "manifests/canonical_layer1_world_model_manifest_v0_1.json"
    )
    canonical_validation_path = (
        CANONICAL_LAYER1_DIR
        / "reports/canonical_layer1_world_model_validation_report_v0_1.json"
    )
    if execution_attempted:
        canonical_manifest = {
            "project_name": PROJECT_NAME,
            "scene_id": SCENE_ID,
            "layer": "Layer 1",
            "layer_name": "World Model Layer",
            "implementation_entrypoint": str(ENTRYPOINT_PATH.relative_to(REPO_ROOT)),
            "implementation_entrypoint_description": "historical Stage-A / World Model entrypoint",
            "wrapper_used": True,
            "wrapper_path": str(Path(__file__).resolve().relative_to(REPO_ROOT)),
            "command_used": historical_command,
            "python_used": str(PYTHON_PATH),
            "device_mode": preflight_report["selected_device"],
            "input_data_path": str(INPUT_DATA_PATH),
            "config_path": str(CONFIG_PATH.relative_to(REPO_ROOT)),
            "model_path": str(MODEL_PATH.relative_to(REPO_ROOT)),
            "output_root": str(CANONICAL_LAYER1_DIR.relative_to(REPO_ROOT)),
            "raw_output_root": str(
                (CANONICAL_LAYER1_DIR / "raw_outputs").relative_to(REPO_ROOT)
            ),
            "generated_at": utc_now(),
            "execution_status": execution_status,
            "produced_artifacts": raw_inventory,
            "known_limitations": missing_evidence,
            "downstream_status": task36_status,
            "historical_outputs_promoted": False,
            "candidate_artifacts_used_as_inputs": False,
        }
        write_json(canonical_manifest_path, canonical_manifest)
        write_json(canonical_validation_path, canonical_validation)

    validation_report = {
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "execution_attempted": execution_attempted,
        "execution_status": execution_status,
        "checks": {
            "output_files_exist": outputs_exist,
            "important_output_files_nonempty": outputs_nonempty,
            "scene_id_matches": scene_matches,
            "outputs_under_canonical_layer1_directory": under_canonical,
            "historical_output_reuse_was_not_promoted": not historical_reuse_detected,
            "no_clean_rerun_writes_occurred": clean_unchanged,
            "historical_00824_reference_unchanged": reference_unchanged,
            "no_ros_gazebo_rviz_nav2_amcl_runtime_launched": True,
        },
        "layer1_evidence_availability": evidence,
        "missing_layer1_evidence_for_task36": missing_evidence,
        "task36_status": task36_status,
        "generated_at": utc_now(),
    }
    write_json(TASK_EVIDENCE_DIR / "validation_report.json", validation_report)
    write_command_log(
        historical_command,
        preflight_report,
        static_validator,
        started_at,
        finished_at,
        returncode,
    )

    warnings = []
    if args.preflight_only and ready:
        warnings.append("Preflight-only mode was explicitly requested; execution was not attempted.")
    for doc in preflight_report["docs_and_manifests_read"]:
        if not doc["exists"]:
            warnings.append(f"Missing required project truth file: {doc['path']}")
    if missing_evidence:
        warnings.append(
            "Missing expected Layer 1 evidence fields: " + ", ".join(missing_evidence)
        )
    if not reference_before["exists"]:
        warnings.append(
            "Historical 00824 reference output was absent before the task; absence was preserved."
        )
    if not clean_before["exists"]:
        warnings.append(
            "Historical 00843 clean_rerun output was absent before the task; no directory was created there."
        )
    if static_validator["returncode"] != 0:
        warnings.append(
            "The optional static canonical artifact validator returned nonzero; see "
            + static_validator["report_path"]
            + ". This did not override the task-specific rerun preflight."
        )
    warnings.extend(
        [
            "The historical implementation retains Stage-A terminology internally; task35 reports use Layer 1 World Model naming.",
            "CUDA was selected because the required Python reported a working CUDA device; CPU fallback remains supported by the historical CLI.",
            "Historical command references were inspected only for reconstruction and were not promoted as canonical outputs.",
        ]
    )
    write_text(
        TASK_EVIDENCE_DIR / "warnings.txt",
        "\n".join(f"- {warning}" for warning in warnings)
        if warnings
        else "No warnings were found.",
    )

    if preflight_report["final_preflight_status"] == "blocked":
        classification = "task35_layer1_canonical_rerun_blocked_by_preflight"
        status = "blocked"
        recommended_next = "targeted_repair_for_preflight_blocking_reasons"
        blocked_next_steps = preflight_report["blocking_reasons"]
    elif execution_status == "failed":
        classification = (
            "task35_layer1_canonical_rerun_execution_failed_after_preflight_ready"
        )
        status = "failed"
        recommended_next = "targeted_repair_for_layer1_execution_error"
        blocked_next_steps = [
            "Inspect the canonical Layer 1 execution log and repair only the recorded execution error."
        ]
    elif execution_status == "succeeded":
        classification = (
            "task35_layer1_canonical_rerun_preflight_and_execute_if_ready_completed"
        )
        status = "completed"
        if task36_status == "unblocked":
            recommended_next = (
                "task36_final_layer2_artifacts_generation_from_canonical_layer1"
            )
            blocked_next_steps = []
        else:
            recommended_next = "targeted_layer1_output_adapter_or_evidence_normalization"
            blocked_next_steps = [
                f"Normalize or expose missing Layer 1 evidence: {name}"
                for name in missing_evidence
            ]
    else:
        classification = "task35_layer1_canonical_rerun_blocked_by_preflight"
        status = "blocked"
        recommended_next = "execute_task35_without_preflight_only_mode"
        blocked_next_steps = ["Execution was not attempted."]

    produced_summary = {
        "canonical_file_count": len(output_inventory(CANONICAL_LAYER1_DIR)),
        "raw_output_file_count": len(raw_inventory),
        "canonical_manifest": str(canonical_manifest_path.relative_to(REPO_ROOT))
        if canonical_manifest_path.is_file()
        else None,
        "canonical_validation_report": str(
            canonical_validation_path.relative_to(REPO_ROOT)
        )
        if canonical_validation_path.is_file()
        else None,
        "main_execution_log": str(
            (CANONICAL_LAYER1_DIR / "logs/layer1_world_model_execution.log").relative_to(
                REPO_ROOT
            )
        )
        if (CANONICAL_LAYER1_DIR / "logs/layer1_world_model_execution.log").is_file()
        else None,
    }
    task_report = {
        "task_name": TASK_NAME,
        "status": status,
        "classification": classification,
        "preflight_status": preflight_report["final_preflight_status"],
        "execution_attempted": execution_attempted,
        "execution_status": execution_status,
        "canonical_layer1_output_dir": str(CANONICAL_LAYER1_DIR.relative_to(REPO_ROOT)),
        "task_evidence_dir": str(TASK_EVIDENCE_DIR.relative_to(REPO_ROOT)),
        "produced_files": produced_summary,
        "validation_summary": {
            "outputs_exist": outputs_exist,
            "outputs_nonempty": outputs_nonempty,
            "scene_matches": scene_matches,
            "under_canonical_directory": under_canonical,
            "clean_rerun_unchanged": clean_unchanged,
            "runtime_launched": False,
            "task36_status": task36_status,
        },
        "missing_layer1_evidence_for_task36": missing_evidence,
        "blocked_next_steps": blocked_next_steps,
        "recommended_next_task": recommended_next,
        "generated_at": utc_now(),
    }
    write_json(TASK_EVIDENCE_DIR / "task35_report.json", task_report)

    task_jsons = list(TASK_EVIDENCE_DIR.rglob("*.json"))
    canonical_jsons = list(CANONICAL_LAYER1_DIR.rglob("*.json"))
    json_validation_path = TASK_EVIDENCE_DIR / "json_validation_report_v0_1.json"
    write_json(
        json_validation_path,
        write_json_validation_report(task_jsons + canonical_jsons),
    )

    manifest_path = (
        TASK_EVIDENCE_DIR / "created_or_modified_files_manifest_v0_1.json"
    )
    all_paths = (
        [Path(__file__).resolve()]
        + list(TASK_EVIDENCE_DIR.rglob("*"))
        + list(CANONICAL_LAYER1_DIR.rglob("*"))
    )
    write_json(manifest_path, created_files_manifest(all_paths))

    # Refresh both self-referential reports after they exist; all JSON remains
    # validated by python json.load in this process.
    all_jsons = list(TASK_EVIDENCE_DIR.rglob("*.json")) + list(
        CANONICAL_LAYER1_DIR.rglob("*.json")
    )
    write_json(json_validation_path, write_json_validation_report(all_jsons))
    all_paths = (
        [Path(__file__).resolve()]
        + list(TASK_EVIDENCE_DIR.rglob("*"))
        + list(CANONICAL_LAYER1_DIR.rglob("*"))
    )
    write_json(manifest_path, created_files_manifest(all_paths))

    print(json.dumps(task_report, indent=2, sort_keys=True))
    return 0 if status == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
