#!/usr/bin/env python3
"""Verify the cleaned Stage1 00824 Step30P1 milestone artifact."""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"
REPORT_JSON = "step30r3_cleanup_verification_report_v0_1.json"
REPORT_MD = "step30r3_cleanup_verification_report_v0_1.md"
TEXT_SUFFIXES = {".json", ".md", ".yaml", ".yml", ".py", ".sh", ".rviz", ".sdf", ".xml", ".txt", ".csv"}
OLD_STEP_REF_RE = re.compile(r"(runtime_stage1_frozen_evidence|referenced_artifacts)/step[0-9][^\s\"'<>),]*")


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def check(condition: bool, message: str, failures: list[str], passed: list[str]) -> None:
    if condition:
        passed.append(message)
    else:
        failures.append(message)


def check_file(path: Path, failures: list[str], passed: list[str], json_file: bool = False) -> None:
    if not path.is_file():
        failures.append(f"Missing required file: {rel(path)}")
        return
    if json_file:
        try:
            load_json(path)
        except Exception as exc:  # noqa: BLE001
            failures.append(f"Unreadable JSON file {rel(path)}: {exc}")
            return
    passed.append(f"Readable required file: {rel(path)}")


def text_files(root: Path) -> list[Path]:
    files: list[Path] = []
    if not root.exists():
        return files
    for path in root.rglob("*"):
        if ".git" in path.parts or not path.is_file():
            continue
        if path.suffix.lower() in TEXT_SUFFIXES or path.name.endswith(".launch.py"):
            files.append(path)
    return files


def contains_old_step_ref(path: Path) -> bool:
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except Exception:  # noqa: BLE001
        return False
    return bool(OLD_STEP_REF_RE.search(text))


def verify(stage_output: Path) -> dict[str, Any]:
    failures: list[str] = []
    passed: list[str] = []

    baselines = REPO_ROOT / "baselines"
    docs_current = REPO_ROOT / "docs/current_project_state.md"
    provenance = stage_output / "manifest/provenance_manifest_v0_2.json"
    deletion_manifest = stage_output / "manifest/deletion_manifest_step30r3_v0_1.json"

    check(stage_output.is_dir(), f"Stage output exists: {rel(stage_output)}", failures, passed)
    check((baselines / "README.md").is_file(), "baselines/ is reserved and has README.md", failures, passed)
    extra_baseline_paths = [p for p in baselines.iterdir() if p.name != "README.md"] if baselines.exists() else []
    check(not extra_baseline_paths, "baselines/ contains no milestone artifacts", failures, passed)

    if stage_output.exists():
        bad_ref_dirs = [p for p in stage_output.rglob("*") if "referenced_artifacts" in p.parts]
        check(not bad_ref_dirs, "No active artifact path contains referenced_artifacts", failures, passed)
        bad_step_dirs = [p for p in stage_output.rglob("*") if p.is_dir() and p.name.startswith("step")]
        check(not bad_step_dirs, "No active artifact directory name starts with step", failures, passed)

    required_json = [
        stage_output / "manifest/artifact_manifest_v0_2.json",
        provenance,
        deletion_manifest,
        stage_output / "manifest/path_relocation_manifest_v0_1.json",
        stage_output / "stage1_committed_public/committed_room_world_model_v0_1.json",
        stage_output / "stage1_committed_public/committed_room_world_snapshot_v0_1.json",
        stage_output / "stage1_committed_public/topology_query_report.json",
        stage_output / "stage1_committed_public/topology_v0_1.json",
        stage_output / "stage1_committed_public/step30p1_committed_public_artifact_index_v0_1.json",
        stage_output / "execution/step30p1_execute_room_chain_result_v0_1.json",
        stage_output / "execution/latest_planned_path.json",
        stage_output / "execution/latest_follow_path_slice.json",
        stage_output / "validation/step30p1_fallback_resume_report_v0_1.json",
        stage_output / "validation/step30p1_trajectory_quality_validation_v0_1.json",
        stage_output / "validation/step30p1_through_room_physical_visit_validation_v0_1.json",
        stage_output / "maps/map_manifest_v0_1.json",
        stage_output / "overlay/rviz_overlay_payload_v0_1.json",
        stage_output / "overlay/overlay_manifest_v0_1.json",
        stage_output / "route/room_chain_v0_1.json",
        stage_output / "route/gateway_sequence_v0_1.json",
        stage_output / "route/route_spec_v0_1.json",
        stage_output / "route/resolved_route_v0_1.json",
        stage_output / "gateway/selected_gateway_summary_v0_1.json",
        stage_output / "gateway/gateway_candidates_or_hypotheses_v0_1.json",
        stage_output / "gateway/gateway_truth_blind_selection_v0_1.json",
    ]
    required_text = [
        stage_output / "README.md",
        stage_output / "manifest/artifact_manifest_v0_2.md",
        stage_output / "execution/step30p1_runbook_v0_1.md",
        stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml",
        stage_output / "maps/h8r2_gateway_preserving_nav_map.pgm",
        stage_output / "maps/h8r2_gateway_preserving_masks.npz",
        stage_output / "nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py",
        stage_output / "nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml",
        stage_output / "nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf",
        REPO_ROOT / "stage_a_demo.py",
        docs_current,
    ]
    for path in required_json:
        check_file(path, failures, passed, json_file=True)
    for path in required_text:
        check_file(path, failures, passed, json_file=False)

    check(any((stage_output / "nav2/ros_package_templates").rglob("*")), "Nav2 ROS package templates exist", failures, passed)
    check(any((stage_output / "overlay/gazebo_overlay_assets").glob("*")), "Gazebo overlay assets exist", failures, passed)
    check(any((stage_output / "trajectories").glob("*")), "Trajectory files exist", failures, passed)
    check(any((stage_output / "regression").rglob("*")), "Bad fallback regression evidence exists", failures, passed)

    stage_a_scripts = sorted(p.name for p in REPO_ROOT.glob("stage_a_*.py") if p.name != "stage_a_demo.py")
    check(not stage_a_scripts, "No top-level stage_a_*.py remains except stage_a_demo.py", failures, passed)

    if provenance.exists():
        provenance_text = provenance.read_text(encoding="utf-8", errors="ignore")
        check("runtime_stage1_frozen_evidence/step" in provenance_text or "referenced_artifacts/step" in provenance_text, "Old StepXX provenance is preserved in provenance manifest", failures, passed)

    active_old_refs = []
    allowed_manifest_names = {
        "provenance_manifest_v0_2.json",
        "deletion_manifest_step30r3_v0_1.json",
        "path_relocation_manifest_v0_1.json",
        "restored_process_artifacts_provenance_v0_1.json",
        "restored_process_artifacts_provenance_v0_1.md",
    }
    for path in text_files(stage_output):
        if path.name in allowed_manifest_names:
            continue
        if contains_old_step_ref(path):
            active_old_refs.append(rel(path))
    check(not active_old_refs, "No active artifact file outside provenance/deletion manifests refers to old StepXX paths", failures, passed)

    python_old_refs = []
    for path in sorted(REPO_ROOT.rglob("*.py")):
        if ".git" in path.parts:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception:  # noqa: BLE001
            continue
        if re.search(r"runtime_stage1_frozen_evidence/step[0-9]", text):
            python_old_refs.append(rel(path))
    check(not python_old_refs, "No active Python script refers to deleted runtime_stage1_frozen_evidence/stepXX paths", failures, passed)

    summary = {
        "artifact_type": "stage1_step30p1_step30r3_cleanup_verification_report",
        "created_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "stage_output_dir": rel(stage_output),
        "passed": not failures,
        "failure_count": len(failures),
        "failures": failures,
        "passed_checks": passed,
    }
    return summary


def write_reports(stage_output: Path, report: dict[str, Any]) -> None:
    reports_dir = stage_output / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    (reports_dir / REPORT_JSON).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    status = "PASSED" if report["passed"] else "FAILED"
    lines = [
        "# Step30R3 Cleanup Verification Report",
        "",
        f"- Status: {status}",
        f"- Stage output: `{report['stage_output_dir']}`",
        f"- Failure count: {report['failure_count']}",
        "",
        "## Failures",
    ]
    if report["failures"]:
        lines.extend(f"- {failure}" for failure in report["failures"])
    else:
        lines.append("- None")
    lines.extend(["", "## Passed Checks"])
    lines.extend(f"- {item}" for item in report["passed_checks"])
    (reports_dir / REPORT_MD).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    report = verify(stage_output)
    write_reports(stage_output, report)
    print("Step30R3 cleanup verification " + ("PASSED" if report["passed"] else "FAILED"))
    print(f"Report: {rel(stage_output / 'reports' / REPORT_JSON)}")
    if report["failures"]:
        for failure in report["failures"]:
            print(f"- {failure}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
