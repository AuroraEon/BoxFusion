"""Runtime trajectory validation and path deviation reporting."""

from __future__ import annotations

import math
from typing import Any

from .schemas import now_iso


def validate_trajectory(
    trajectory: list[dict[str, Any]],
    planner,
    candidate_world_xy: list[float],
    approach_position_tolerance_m: float,
    map_yaml_rel: str | None,
) -> dict[str, Any]:
    """Validate executed trajectory against occupancy map.

    Returns path_deviation_report dict with full and core (endpoint-excluded) validation.
    """
    excl_r = approach_position_tolerance_m
    core_traj = [
        s for s in trajectory
        if s.get("phase") != "yaw_alignment"
        and math.hypot(float(s["x"]) - candidate_world_xy[0], float(s["y"]) - candidate_world_xy[1]) > excl_r
    ]
    traj_validation = planner.validate_polyline(core_traj, require_inflated=False) if core_traj else {}
    full_traj_validation = planner.validate_polyline(trajectory, require_inflated=False) if trajectory else {}

    return {
        "artifact_type": "task17c_runtime_trajectory_validation",
        "created_utc": now_iso(),
        "map_yaml": map_yaml_rel,
        "validation_scope": "full",
        "failure_phase": None,
        "wall_crossing_validation_passed": traj_validation.get("wall_crossing_validation_passed") if core_traj else None,
        "trajectory_occupancy_validation": traj_validation if core_traj else None,
        "full_trajectory_occupancy_validation": full_traj_validation if trajectory else None,
        "trajectory_minimum_clearance_m": traj_validation.get("minimum_clearance_m") if core_traj else None,
        "occupied_or_invalid_sample_count": traj_validation.get("occupied_or_invalid_sample_count") if core_traj else None,
        "endpoint_exclusion_radius_m": excl_r,
        "excluded_sample_count": len(trajectory) - len(core_traj),
        "note": "wall_crossing excludes yaw_alignment phase and samples within approach_tolerance of approach candidate",
    }


def validate_trajectory_partial(
    trajectory: list[dict[str, Any]],
    planner,
    candidate_world_xy: list[float],
    approach_position_tolerance_m: float,
    map_yaml_rel: str | None,
    failure_phase: str,
) -> dict[str, Any]:
    """Partial trajectory validation for failed approach."""
    excl_r = approach_position_tolerance_m
    core_traj = [
        s for s in trajectory
        if s.get("phase") != "yaw_alignment"
        and math.hypot(float(s["x"]) - candidate_world_xy[0], float(s["y"]) - candidate_world_xy[1]) > excl_r
    ]
    traj_validation = planner.validate_polyline(core_traj, require_inflated=False) if core_traj else {}
    full_traj_validation = planner.validate_polyline(trajectory, require_inflated=False) if trajectory else {}

    return {
        "artifact_type": "task17c_runtime_trajectory_validation",
        "created_utc": now_iso(),
        "map_yaml": map_yaml_rel,
        "validation_scope": "partial_until_failure",
        "failure_phase": failure_phase,
        "wall_crossing_validation_passed": traj_validation.get("wall_crossing_validation_passed") if core_traj else None,
        "trajectory_occupancy_validation": traj_validation if core_traj else None,
        "full_trajectory_occupancy_validation": full_traj_validation if trajectory else None,
        "trajectory_minimum_clearance_m": traj_validation.get("minimum_clearance_m") if core_traj else None,
        "occupied_or_invalid_sample_count": traj_validation.get("occupied_or_invalid_sample_count") if core_traj else None,
        "endpoint_exclusion_radius_m": excl_r,
        "excluded_sample_count": len(trajectory) - len(core_traj),
        "note": f"partial validation: {failure_phase} failed before yaw alignment",
    }
