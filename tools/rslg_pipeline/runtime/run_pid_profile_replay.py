#!/usr/bin/env python3
"""Run lightweight PID replay with a named RSLG-SLAM runtime profile.

This wrapper reads ``configs/rslg_runtime_profiles/pid_profiles_v0_1.json``
and forwards the selected parameters to ``replay_pid_runtime_input``. It does
not import ROS, Nav2, AMCL, Gazebo, RViz, or any physical robot runtime.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.runtime import replay_pid_runtime_input

DEFAULT_PROFILE_JSON = Path("configs/rslg_runtime_profiles/pid_profiles_v0_1.json")
PARAM_FIELDS = (
    "dt",
    "max_linear_velocity",
    "max_angular_velocity",
    "linear_gain",
    "angular_gain",
    "waypoint_tolerance",
    "yaw_tolerance",
    "timeout_sec",
    "stuck_window_sec",
    "stuck_progress_epsilon",
    "robot_radius",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def load_profile(profile_json: Path, profile_id: str) -> tuple[dict[str, Any], dict[str, Any]]:
    payload = read_json(profile_json)
    profiles = payload.get("profiles") or {}
    if profile_id not in profiles:
        available = ", ".join(sorted(profiles))
        raise SystemExit(f"profile {profile_id!r} not found in {profile_json}; available: {available}")
    profile = profiles[profile_id]
    params = profile.get("params") or {}
    missing = [field for field in PARAM_FIELDS if field not in params]
    if missing:
        raise SystemExit(f"profile {profile_id!r} is missing parameter fields: {missing}")
    return payload, profile


def refuse_protected_output(output_dir: Path) -> None:
    parts = set(output_dir.resolve().parts)
    protected = {
        "canonical",
        "task56_pid_runtime_executable_replay_validation",
        "task56b_pid_replay_collision_diagnosis_and_mitigation_sweep",
    }
    overlap = sorted(parts.intersection(protected))
    if overlap:
        raise SystemExit(f"refusing protected output directory containing: {overlap}")


def build_replay_args(args: argparse.Namespace, profile: dict[str, Any]) -> argparse.Namespace:
    params = profile["params"]
    title = args.title or f"RSLG-SLAM PID Profile Replay: {args.profile_id}"
    return argparse.Namespace(
        pid_inputs_dir=args.pid_inputs_dir,
        route_results_dir=args.route_results_dir,
        z_aware_inputs_dir=args.z_aware_inputs_dir,
        output_dir=args.output_dir,
        dt=float(params["dt"]),
        max_linear_velocity=float(params["max_linear_velocity"]),
        max_angular_velocity=float(params["max_angular_velocity"]),
        linear_gain=float(params["linear_gain"]),
        angular_gain=float(params["angular_gain"]),
        waypoint_tolerance=float(params["waypoint_tolerance"]),
        yaw_tolerance=float(params["yaw_tolerance"]),
        timeout_sec=float(params["timeout_sec"]),
        stuck_window_sec=float(params["stuck_window_sec"]),
        stuck_progress_epsilon=float(params["stuck_progress_epsilon"]),
        robot_radius=float(params["robot_radius"]),
        floor_z_map=args.floor_z_map,
        title=title,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile-json", type=Path, default=DEFAULT_PROFILE_JSON)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--pid-inputs-dir", type=Path, required=True)
    parser.add_argument("--route-results-dir", type=Path, default=None)
    parser.add_argument("--z-aware-inputs-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--title", default=None)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    refuse_protected_output(args.output_dir)
    profile_payload, profile = load_profile(args.profile_json, args.profile_id)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(
        args.output_dir / "profile_run_metadata.json",
        {
            "schema_name": "rslg_pid_profile_replay_metadata",
            "schema_version": "0.1",
            "project_name": profile_payload.get("project_name", "RSLG-SLAM"),
            "generated_utc": utc_now(),
            "profile_json": args.profile_json.as_posix(),
            "profile_id": args.profile_id,
            "source_task": profile.get("source_task"),
            "source_candidate": profile.get("source_candidate"),
            "intended_use": profile.get("intended_use"),
            "params": profile.get("params"),
            "claim_boundary": profile_payload.get("claim_boundary"),
        },
    )
    return replay_pid_runtime_input.run(build_replay_args(args, profile))


if __name__ == "__main__":
    raise SystemExit(main())
