#!/usr/bin/env python3
"""Validate task23a obj178 quadruped visual-kinematic proxy demo outputs."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


CLAIM_BOUNDARY = "visual_kinematic_proxy_only"
MAX_DISTANCE_M = 0.35
MAX_RUNTIME_SEC = 420.0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_md(path: Path, report: dict[str, Any]) -> None:
    checks = report["checks"]
    lines = [
        "# Obj178 Quadruped Proxy Output Validation",
        "",
        f"- Run directory: `{report['run_dir']}`",
        f"- Overall pass: `{report['validation_passed']}`",
        f"- Failure count: `{report['failure_count']}`",
        "",
        "## Checks",
    ]
    for check in checks:
        lines.append(
            f"- `{check['name']}`: `{check['passed']}`"
            + (f" ({check['observed']})" if check.get("observed") is not None else "")
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def value_at(data: dict[str, Any], key: str) -> Any:
    return data.get(key)


def add_check(checks: list[dict[str, Any]], name: str, passed: bool, observed: Any = None) -> None:
    checks.append({"name": name, "passed": bool(passed), "observed": observed})


def validate_run(run_dir: Path) -> dict[str, Any]:
    runtime_path = run_dir / "runtime_result.json"
    summary_path = run_dir / "summary.json"
    runtime = read_json(runtime_path)
    summary = read_json(summary_path)
    robot_profile_report = read_json(run_dir / "robot_profile_report.json")
    checks: list[dict[str, Any]] = []

    add_check(checks, "runtime_result.json exists", runtime_path.exists(), str(runtime_path))
    add_check(checks, "summary.json exists", summary_path.exists(), str(summary_path))

    bool_fields = [
        "success",
        "object_facing_approach_success",
        "approach_position_reached",
        "approach_yaw_aligned",
        "wall_crossing_validation_passed",
    ]
    for field in bool_fields:
        add_check(checks, f"{field} is true", value_at(runtime, field) is True, value_at(runtime, field))

    add_check(
        checks,
        "final_control_path_source is dense_route_fallback",
        runtime.get("final_control_path_source") == "dense_route_fallback",
        runtime.get("final_control_path_source"),
    )
    add_check(checks, "failure_layer is null", runtime.get("failure_layer") is None, runtime.get("failure_layer"))
    add_check(checks, "failure_reason is null", runtime.get("failure_reason") is None, runtime.get("failure_reason"))

    target_distance = runtime.get("final_distance_to_target_room_terminal_m")
    add_check(
        checks,
        f"final_distance_to_target_room_terminal_m <= {MAX_DISTANCE_M}",
        isinstance(target_distance, (int, float)) and target_distance <= MAX_DISTANCE_M,
        target_distance,
    )
    approach_distance = runtime.get("final_distance_to_approach_candidate_m")
    add_check(
        checks,
        f"final_distance_to_approach_candidate_m <= {MAX_DISTANCE_M}",
        isinstance(approach_distance, (int, float)) and approach_distance <= MAX_DISTANCE_M,
        approach_distance,
    )
    duration = runtime.get("runtime_duration_sec")
    add_check(
        checks,
        f"runtime_duration_sec <= {MAX_RUNTIME_SEC}",
        isinstance(duration, (int, float)) and duration <= MAX_RUNTIME_SEC,
        duration,
    )

    claim_values = [
        runtime.get("claim_boundary"),
        summary.get("claim_boundary"),
        (robot_profile_report.get("robot_profile") or {}).get("claim_boundary"),
    ]
    present_claims = [value for value in claim_values if value is not None]
    add_check(
        checks,
        "claim_boundary is visual_kinematic_proxy_only when present",
        not present_claims or all(value == CLAIM_BOUNDARY for value in present_claims),
        present_claims,
    )

    if "no_nav2_action_servers_active" in runtime:
        add_check(
            checks,
            "no Nav2 action servers active if reported",
            runtime.get("no_nav2_action_servers_active") is True,
            runtime.get("no_nav2_action_servers_active"),
        )

    result_fields = [
        "success",
        "object_facing_approach_success",
        "approach_position_reached",
        "approach_yaw_aligned",
        "wall_crossing_validation_passed",
        "runtime_duration_sec",
        "final_distance_to_target_room_terminal_m",
        "final_distance_to_approach_candidate_m",
        "final_control_path_source",
        "tracking_quality_passed",
        "failure_layer",
        "failure_reason",
    ]
    result = {field: runtime.get(field) for field in result_fields}
    result["run_dir"] = str(run_dir)

    failures = [check for check in checks if not check["passed"]]
    return {
        "artifact_type": "task23a_obj178_output_validation",
        "created_utc": utc_now(),
        "run_dir": str(run_dir),
        "runtime_result_path": str(runtime_path),
        "summary_path": str(summary_path),
        "validation_passed": not failures,
        "failure_count": len(failures),
        "checks": checks,
        "result": result,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--report-json", type=Path)
    parser.add_argument("--report-md", type=Path)
    args = parser.parse_args()

    report = validate_run(args.run_dir.resolve())
    print(f"Run: {report['run_dir']}")
    print(f"Validation passed: {report['validation_passed']}")
    for check in report["checks"]:
        status = "PASS" if check["passed"] else "FAIL"
        print(f"{status} {check['name']}: {check.get('observed')}")

    if args.report_json:
        write_json(args.report_json.resolve(), report)
    if args.report_md:
        write_md(args.report_md.resolve(), report)
    return 0 if report["validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
