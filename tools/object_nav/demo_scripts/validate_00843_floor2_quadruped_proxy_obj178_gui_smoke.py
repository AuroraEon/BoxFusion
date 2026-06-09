#!/usr/bin/env python3
"""Build task23b GUI smoke evidence for the obj178 quadruped proxy demo."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TASK_NAME = "task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"
CLAIM_BOUNDARY = "visual_kinematic_proxy_only"
FINAL_GUI_VALIDATED = "task23a_gui_demo_validated"
FINAL_GUI_PENDING = "task23a_headless_passed_gui_pending"
FINAL_GUI_ENV_BLOCKED = "task23a_gui_environment_blocked"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(payload, encoding="utf-8")


def command_output(command: list[str]) -> str:
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    return result.stdout.strip()


def process_seen(snapshot_paths: list[Path], pattern: str) -> bool:
    expr = re.compile(pattern)
    return any(expr.search(read_text(path)) for path in snapshot_paths)


def matching_pids(snapshot_paths: list[Path], pattern: str) -> set[str]:
    expr = re.compile(pattern)
    pids: set[str] = set()
    for path in snapshot_paths:
        for line in read_text(path).splitlines():
            if not expr.search(line):
                continue
            parts = line.split(maxsplit=4)
            if parts:
                pids.add(parts[0])
    return pids


def new_process_seen(snapshot_paths: list[Path], before_path: Path, pattern: str) -> bool:
    before_pids = matching_pids([before_path], pattern)
    after_paths = [path for path in snapshot_paths if path.name != before_path.name]
    return bool(matching_pids(after_paths, pattern) - before_pids)


def gui_environment(env_path: Path, gzclient_log: Path, rviz_log: Path) -> dict[str, Any]:
    env_text = read_text(env_path)
    values: dict[str, str] = {}
    for line in env_text.splitlines():
        if "=" not in line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()

    display = values.get("DISPLAY") or os.environ.get("DISPLAY", "")
    wayland = values.get("WAYLAND_DISPLAY") or os.environ.get("WAYLAND_DISPLAY", "")
    xdg_session = values.get("XDG_SESSION_TYPE") or os.environ.get("XDG_SESSION_TYPE", "")
    gzclient_text = read_text(gzclient_log)
    rviz_text = read_text(rviz_log)
    gui_log = "\n".join([gzclient_text, rviz_text]).lower()
    graphics_error_patterns = [
        "could not connect to display",
        "cannot connect to x server",
        "qt.qpa.xcb",
        "no protocol specified",
        "failed to create drawable",
        "libgl error",
        "failed to load driver",
        "failed to create opengl context",
        "segmentation fault",
        "core dumped",
    ]
    graphics_error = next((pattern for pattern in graphics_error_patterns if pattern in gui_log), "")
    return {
        "display": display,
        "wayland_display": wayland,
        "xdg_session_type": xdg_session,
        "has_display_or_wayland": bool(display or wayland),
        "gzclient_log_path": str(gzclient_log),
        "rviz2_log_path": str(rviz_log),
        "graphics_error_detected": bool(graphics_error),
        "graphics_error_pattern": graphics_error or None,
    }


def write_manual_template(path: Path, command_used: list[str]) -> None:
    lines = [
        "# Manual GUI Observation Template",
        "",
        f"task_name: {TASK_NAME}",
        f"timestamp: {utc_now()}",
        f"command_used: {' '.join(command_used)}",
        "",
        "gazebo_gui_observed_by_user: ",
        "rviz_overlay_observed_by_user: ",
        "quadruped_proxy_motion_observed_by_user: ",
        "notes: ",
        "",
        "This file captures human observation only. It does not replace process evidence for gzclient, gzserver, or rviz2.",
    ]
    write_text(path, "\n".join(lines) + "\n")


def build_report(
    *,
    task_dir: Path,
    run_dir: Path,
    run_id: str,
    runner_returncode: int,
    rviz_launch_attempted: bool,
    rviz_reason: str,
    rviz_config: str,
    keep_open_sec: float,
    command_used: list[str],
) -> dict[str, Any]:
    snapshot_dir = task_dir / "process_snapshots"
    snapshot_paths = sorted(snapshot_dir.glob("*.txt"))
    before_snapshot = snapshot_dir / "000_before_launch.txt"
    runtime_path = run_dir / "runtime_result.json"
    validation_path = run_dir / "task23a_validation_report.json"
    runtime = read_json(runtime_path)
    validation = read_json(validation_path)

    logs_dir = task_dir / "logs"
    gzserver_log = logs_dir / f"{run_id}_gzserver.log"
    gzclient_log = logs_dir / f"{run_id}_gzclient.log"
    rviz_log = logs_dir / f"{run_id}_rviz2.log"
    gui_env_path = task_dir / "gui_environment.txt"
    manual_template = task_dir / "manual_gui_observation_template.md"
    write_manual_template(manual_template, command_used)

    gzserver_started = new_process_seen(snapshot_paths, before_snapshot, r"\bgzserver\b") or (
        run_dir / "launch_logs/gazebo.log"
    ).exists()
    gzclient_started = new_process_seen(snapshot_paths, before_snapshot, r"\bgzclient\b")
    rviz2_started = new_process_seen(snapshot_paths, before_snapshot, r"\brviz2\b")
    quadruped_proxy_seen = new_process_seen(snapshot_paths, before_snapshot, r"quadruped_kinematic_proxy_node")
    runner_seen = new_process_seen(snapshot_paths, before_snapshot, r"run_lightweight_object_nav\.py")
    runtime_validation_passed = validation.get("validation_passed") is True
    runtime_success = runtime.get("success") is True
    environment = gui_environment(gui_env_path, gzclient_log, rviz_log)

    if runtime_validation_passed and gzclient_started and rviz2_started:
        final_status = FINAL_GUI_VALIDATED
    elif runtime_validation_passed and (
        not environment["has_display_or_wayland"] or environment["graphics_error_detected"]
    ):
        final_status = FINAL_GUI_ENV_BLOCKED
    else:
        final_status = FINAL_GUI_PENDING

    git_entries = [
        "tools/object_nav/demo_scripts/run_00843_floor2_quadruped_proxy_obj178_headless_demo.sh",
        "tools/object_nav/demo_scripts/run_00843_floor2_quadruped_proxy_obj178_gui_demo.sh",
        "tools/object_nav/demo_scripts/README_quadruped_proxy_obj178_demo.md",
        "tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_outputs.py",
        "tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_gui_smoke.py",
    ]
    git_status = command_output(["git", "status", "--short", "--", *git_entries])

    return {
        "artifact_type": "task23b_gui_smoke_report",
        "created_utc": utc_now(),
        "task_name": TASK_NAME,
        "claim_boundary": CLAIM_BOUNDARY,
        "source_scripts_checked": git_entries,
        "git_managed_demo_entries": [
            {"path": path, "exists": Path(path).exists(), "under_tools": path.startswith("tools/object_nav/demo_scripts/")}
            for path in git_entries
        ],
        "git_status_for_demo_entries": git_status,
        "run_id": run_id,
        "run_dir": str(run_dir),
        "runtime_result_path": str(runtime_path),
        "runtime_validation_report_path": str(validation_path),
        "runtime_validation_passed": runtime_validation_passed,
        "runtime_result_success_passed": runtime_success,
        "runner_returncode": runner_returncode,
        "formal_gui_command": command_used,
        "keep_open_sec": keep_open_sec,
        "rviz_config": rviz_config,
        "rviz_launch_attempted": rviz_launch_attempted,
        "rviz_missing_or_status_reason": rviz_reason,
        "gzserver_started": gzserver_started,
        "gzclient_started": gzclient_started,
        "rviz2_started": rviz2_started,
        "quadruped_proxy_node_seen": quadruped_proxy_seen,
        "lightweight_runner_seen": runner_seen,
        "prelaunch_matching_processes": {
            "gzserver_pids": sorted(matching_pids([before_snapshot], r"\bgzserver\b")),
            "gzclient_pids": sorted(matching_pids([before_snapshot], r"\bgzclient\b")),
            "rviz2_pids": sorted(matching_pids([before_snapshot], r"\brviz2\b")),
            "quadruped_proxy_node_pids": sorted(matching_pids([before_snapshot], r"quadruped_kinematic_proxy_node")),
            "lightweight_runner_pids": sorted(matching_pids([before_snapshot], r"run_lightweight_object_nav\.py")),
        },
        "process_snapshot_paths": [str(path) for path in snapshot_paths],
        "gui_environment_summary": environment,
        "log_paths": {
            "runner": str(logs_dir / f"{run_id}.log"),
            "gzserver": str(gzserver_log),
            "gzclient": str(gzclient_log),
            "rviz2": str(rviz_log),
            "quadruped_proxy_node": str(logs_dir / f"{run_id}_quadruped_proxy_node.log"),
            "spawn_entity": str(logs_dir / f"{run_id}_spawn_entity.log"),
        },
        "manual_observation_template_path": str(manual_template),
        "manual_observation_is_process_evidence": False,
        "final_status": final_status,
    }


def write_md(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# Task23b GUI Smoke Summary",
        "",
        f"- Task: `{report['task_name']}`",
        f"- Final status: `{report['final_status']}`",
        f"- Runtime validation passed: `{report['runtime_validation_passed']}`",
        f"- gzserver process observed: `{report['gzserver_started']}`",
        f"- gzclient process observed: `{report['gzclient_started']}`",
        f"- rviz2 process observed: `{report['rviz2_started']}`",
        f"- Quadruped proxy node observed: `{report['quadruped_proxy_node_seen']}`",
        f"- Lightweight runner observed: `{report['lightweight_runner_seen']}`",
        f"- Runtime result: `{report['runtime_result_path']}`",
        f"- Process snapshots: `{len(report['process_snapshot_paths'])}` files under `process_snapshots/`",
        f"- GUI environment file: `gui_environment.txt`",
        f"- Manual observation template: `{report['manual_observation_template_path']}`",
        "",
        "Manual observation is optional human evidence only and is not counted as process evidence.",
        "",
        "Claim boundary: `visual_kinematic_proxy_only`.",
    ]
    write_text(path, "\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-dir", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--runner-returncode", type=int, required=True)
    parser.add_argument("--rviz-launch-attempted", choices=["true", "false"], required=True)
    parser.add_argument("--rviz-reason", default="")
    parser.add_argument("--rviz-config", required=True)
    parser.add_argument("--keep-open-sec", type=float, required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--report-md", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()

    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    report = build_report(
        task_dir=args.task_dir.resolve(),
        run_dir=args.run_dir.resolve(),
        run_id=args.run_id,
        runner_returncode=args.runner_returncode,
        rviz_launch_attempted=args.rviz_launch_attempted == "true",
        rviz_reason=args.rviz_reason,
        rviz_config=args.rviz_config,
        keep_open_sec=args.keep_open_sec,
        command_used=command,
    )
    write_json(args.report_json.resolve(), report)
    write_md(args.report_md.resolve(), report)
    print(json.dumps({"final_status": report["final_status"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
