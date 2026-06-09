"""Report generation: summary, runtime_result, controller efficiency, diagnostics, etc."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

from .schemas import now_iso, route_length


def write_json(path: Path, payload: Any) -> None:
    import json
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def save_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({f: row.get(f, "") for f in fields})


def generate_controller_efficiency_report(
    telemetry: list[dict[str, Any]],
    trajectory: list[dict[str, Any]],
    total_steps: int,
    pose_changed_count: int,
    pose_repeated_count: int,
    angular_saturation_count: int,
    angular_sign_switch_count: int,
    zero_linear_count: int,
) -> dict[str, Any]:
    """Generate controller efficiency report from telemetry data."""
    traj_len = route_length(trajectory) if trajectory else 0
    runtime_dur = trajectory[-1]["t_sec"] if trajectory else 0
    room_route_steps = [s for s in telemetry if s.get("phase") == "room_route"]
    room_route_zero = sum(1 for s in room_route_steps if s.get("zero_linear_reason") != "none")
    return {
        "artifact_type": "task17c_controller_efficiency_report",
        "created_utc": now_iso(),
        "total_control_steps": total_steps,
        "tf_poll_count": pose_changed_count + pose_repeated_count,
        "tf_poll_repeated_pose_count": pose_repeated_count,
        "tf_poll_repeated_pose_ratio": round(pose_repeated_count / max(1, pose_changed_count + pose_repeated_count), 4),
        "control_step_count": total_steps,
        "control_step_repeated_pose_count": 0,
        "control_step_repeated_pose_ratio": 0.0,
        "angular_saturation_count": angular_saturation_count,
        "angular_saturation_ratio": round(angular_saturation_count / max(1, total_steps), 4),
        "angular_sign_switch_count": angular_sign_switch_count,
        "zero_linear_count": zero_linear_count,
        "zero_linear_ratio": round(zero_linear_count / max(1, total_steps), 4),
        "room_route_zero_linear_count": room_route_zero,
        "room_route_zero_linear_ratio": round(room_route_zero / max(1, len(room_route_steps)), 4),
        "room_route_total_steps": len(room_route_steps),
        "trajectory_length_m": round(traj_len, 6),
        "runtime_duration_sec": round(runtime_dur, 3),
        "cmd_publish_rate_hz": round(total_steps / max(0.01, runtime_dur), 2),
    }


def generate_path_tracking_diagnostics(
    telemetry: list[dict[str, Any]],
    efficiency: dict[str, Any],
    total_steps: int,
    angular_sign_switch_count: int,
) -> dict[str, Any]:
    """Generate path tracking diagnostics from telemetry."""
    cross_track_errors = [s.get("cross_track_error", 0) for s in telemetry if "cross_track_error" in s]
    return {
        "artifact_type": "task17c_path_tracking_diagnostics",
        "created_utc": now_iso(),
        "mean_cross_track_error_m": round(sum(cross_track_errors) / max(1, len(cross_track_errors)), 6) if cross_track_errors else None,
        "max_cross_track_error_m": round(max(cross_track_errors), 6) if cross_track_errors else None,
        "total_steps": total_steps,
        "angular_saturation_ratio": efficiency["angular_saturation_ratio"],
        "zero_linear_ratio": efficiency["zero_linear_ratio"],
        "room_route_zero_linear_ratio": efficiency["room_route_zero_linear_ratio"],
        "angular_sign_switch_count": angular_sign_switch_count,
        "tf_poll_repeated_pose_ratio": efficiency["tf_poll_repeated_pose_ratio"],
        "control_step_repeated_pose_ratio": efficiency["control_step_repeated_pose_ratio"],
    }


def save_telemetry_artifacts(out: Path, controller) -> None:
    """Save all telemetry CSV/JSON artifacts from a controller run."""
    telemetry = controller.telemetry
    trajectory = controller.trajectory
    commands = controller.commands

    write_json(out / "cmd_vel_log.json", {"artifact_type": "task17c_cmd_vel_log", "commands": commands})
    write_json(out / "executed_trajectory.json", {"artifact_type": "task17c_executed_trajectory", "samples": trajectory})
    write_json(out / "control_telemetry.json", {"artifact_type": "task17c_control_telemetry", "steps": telemetry})

    if telemetry:
        fields = list(telemetry[0].keys())
        save_csv(out / "control_telemetry.csv", telemetry, fields)
    if trajectory:
        save_csv(out / "executed_trajectory.csv", trajectory, list(trajectory[0].keys()))
    if commands:
        save_csv(out / "cmd_vel_log.csv", commands, ["t_sec", "phase", "linear_x", "angular_z"])

    # Efficiency report
    eff = generate_controller_efficiency_report(
        telemetry, trajectory,
        controller.total_steps, controller.pose_changed_count,
        controller.pose_repeated_count, controller.angular_saturation_count,
        controller.angular_sign_switch_count, controller.zero_linear_count,
    )
    write_json(out / "controller_efficiency_report.json", eff)

    # Diagnostics
    diag = generate_path_tracking_diagnostics(
        telemetry, eff, controller.total_steps, controller.angular_sign_switch_count,
    )
    write_json(out / "path_tracking_diagnostics.json", diag)


def generate_approach_yaw_validation_report(
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    params,
    runtime: dict[str, Any],
    approach_result: dict[str, Any],
    yaw_result: dict[str, Any],
) -> dict[str, Any]:
    """Generate approach_yaw_validation_report.json content."""
    return {
        "artifact_type": "task17c_approach_yaw_validation_report",
        "created_utc": now_iso(),
        "approach_candidate_id": candidate.get("candidate_id"),
        "approach_candidate_world_xy": candidate["world_xy"],
        "yaw_proxy_xy": proxy["yaw_proxy_xy"],
        "approach_position_tolerance_m": params.approach_position_tolerance_m,
        "approach_controller_stop_tolerance_m": params.approach_controller_stop_tolerance_m,
        "yaw_report_tolerance_rad": params.yaw_report_tolerance_rad,
        "xy_drift_limit_m": params.xy_drift_limit_m,
        "approach_position_reached": runtime.get("approach_position_reached"),
        "controller_reported_distance_m": approach_result.get("controller_reported_distance_m"),
        "post_settle_distance_m": approach_result.get("post_settle_distance_m"),
        "final_creep_attempted": approach_result.get("final_creep_attempted"),
        "final_creep_attempt_count": approach_result.get("final_creep_attempt_count"),
        "final_creep_distances_m": approach_result.get("final_creep_distances_m"),
        "final_remeasured_distance_m": approach_result.get("final_remeasured_distance_m"),
        "final_distance_to_approach_candidate_m": runtime.get("final_distance_to_approach_candidate_m"),
        "final_pose_after_settle": approach_result.get("final_pose_after_settle"),
        "final_pose_after_creep": approach_result.get("final_pose_after_creep"),
        "approach_yaw_aligned": runtime.get("approach_yaw_aligned"),
        "final_yaw_error_rad": runtime.get("final_yaw_error_rad"),
        "xy_drift_m": yaw_result.get("xy_drift_m"),
        "yaw_facing_success": runtime.get("yaw_facing_success"),
        "yaw_drift_success": runtime.get("yaw_drift_success"),
        "wall_crossing_validation_passed": runtime.get("wall_crossing_validation_passed"),
        "object_facing_approach_success": runtime.get("object_facing_approach_success"),
    }


def generate_summary(
    query: str,
    object_id: str | None,
    runtime: dict[str, Any],
) -> dict[str, Any]:
    """Generate final summary.json."""
    summary = {
        "artifact_type": "task17c_summary",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": query,
        "object_id": object_id,
        "backend": "curvature-based pure pursuit with dynamic lookahead and corner-rounded control path, without Nav2",
        "nav2_used": False,
        "success": runtime.get("success"),
        "failure_layer": runtime.get("failure_layer"),
        "failure_reason": runtime.get("failure_reason"),
    }

    runtime_evidence_fields = [
        "target_room",
        "floor_id",
        "runtime_execution_attempted",
        "gazebo_started_without_nav2",
        "no_nav2_action_servers_active",
        "object_facing_approach_success",
        "approach_position_reached",
        "approach_yaw_aligned",
        "yaw_facing_success",
        "yaw_drift_success",
        "wall_crossing_validation_passed",
        "final_control_path_source",
        "rounding_attempted",
        "rounding_applied",
        "fallback_to_original",
        "fallback_reason",
        "rounded_candidate_validation_passed",
        "rounded_candidate_length_m",
        "final_control_path_length_m",
        "final_distance_to_approach_candidate_m",
        "final_yaw_error_rad",
        "trajectory_length_m",
        "runtime_duration_sec",
        "trajectory_minimum_clearance_m",
    ]
    for field in runtime_evidence_fields:
        if field in runtime:
            summary[field] = runtime[field]

    return summary
