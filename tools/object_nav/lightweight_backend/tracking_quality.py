"""Tracking-quality metrics and visualizations for executed control paths."""

from __future__ import annotations

import math
import statistics
from pathlib import Path
from typing import Any

from .curvature_controller import project_onto_polyline
from .schemas import route_length


DEFAULT_THRESHOLDS = {
    "trajectory_to_control_path_length_ratio": 1.12,
    "mean_tracking_error_m": 0.25,
    "p95_tracking_error_m": 0.55,
    "max_tracking_error_m": 0.85,
    "zero_linear_ratio": 0.20,
    "angular_saturation_ratio": 0.35,
}


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _signed_error(sample: dict[str, Any], path: list[dict[str, Any]]) -> tuple[float, float]:
    px, py = float(sample["x"]), float(sample["y"])
    proj_x, proj_y, _arc, segment_index = project_onto_polyline(px, py, path)
    left, right = path[segment_index], path[min(segment_index + 1, len(path) - 1)]
    tx, ty = float(right["x"]) - float(left["x"]), float(right["y"]) - float(left["y"])
    length = math.hypot(tx, ty)
    signed = ((tx * (py - proj_y) - ty * (px - proj_x)) / length) if length > 1e-9 else 0.0
    return math.hypot(px - proj_x, py - proj_y), signed


def generate_tracking_quality_report(
    trajectory: list[dict[str, Any]],
    control_path: list[dict[str, Any]],
    commands: list[dict[str, Any]],
    max_angular_speed: float,
    thresholds: dict[str, float] | None = None,
) -> dict[str, Any]:
    thresholds = dict(thresholds or DEFAULT_THRESHOLDS)
    room_trajectory = [s for s in trajectory if s.get("phase") == "room_route"]
    room_commands = [c for c in commands if c.get("phase") == "room_route"]
    core_trajectory = room_trajectory or [s for s in trajectory if s.get("phase") != "yaw_alignment"]
    core_commands = room_commands or [c for c in commands if c.get("phase") != "yaw_alignment"]
    errors_and_signed = [_signed_error(sample, control_path) for sample in core_trajectory] if len(control_path) >= 2 else []
    errors = [item[0] for item in errors_and_signed]
    signed = [item[1] for item in errors_and_signed]

    nonzero_signs = [1 if value > 0.01 else -1 for value in signed if abs(value) > 0.01]
    zero_crossings = sum(1 for left, right in zip(nonzero_signs, nonzero_signs[1:]) if left != right)
    angular = [abs(float(c.get("angular_z", 0.0))) for c in core_commands]
    linear = [float(c.get("linear_x", 0.0)) for c in core_commands]
    saturation_ratio = (
        sum(1 for value in angular if value >= max_angular_speed * 0.95) / len(angular)
        if angular else 0.0
    )
    zero_linear_ratio = sum(1 for value in linear if abs(value) <= 1e-6) / len(linear) if linear else 0.0
    control_length = route_length(control_path)
    trajectory_length = route_length(core_trajectory)
    ratio = trajectory_length / control_length if control_length > 1e-9 else None

    metrics = {
        "trajectory_to_control_path_length_ratio": ratio,
        "mean_tracking_error_m": statistics.fmean(errors) if errors else None,
        "median_tracking_error_m": statistics.median(errors) if errors else None,
        "p90_tracking_error_m": _percentile(errors, 0.90),
        "p95_tracking_error_m": _percentile(errors, 0.95),
        "max_tracking_error_m": max(errors) if errors else None,
        "signed_lateral_error_zero_crossings": zero_crossings,
        "lateral_error_oscillation_count": zero_crossings,
        "angular_saturation_ratio": saturation_ratio,
        "zero_linear_ratio": zero_linear_ratio,
        "mean_abs_angular_velocity": statistics.fmean(angular) if angular else 0.0,
        "max_abs_angular_velocity": max(angular) if angular else 0.0,
        "mean_linear_velocity": statistics.fmean(linear) if linear else 0.0,
        "control_command_count": len(core_commands),
        "executed_trajectory_length_m": trajectory_length,
        "final_control_path_length_m": control_length,
    }
    rounded = {key: round(value, 6) if isinstance(value, float) else value for key, value in metrics.items()}
    checks = {
        key: rounded.get(key) is not None and float(rounded[key]) <= limit
        for key, limit in thresholds.items()
    }
    return {
        "artifact_type": "task22c_tracking_quality_report",
        "thresholds": thresholds,
        **rounded,
        "threshold_checks": checks,
        "tracking_quality_passed": bool(errors and commands and all(checks.values())),
        "tracking_error_samples": [
            {"t_sec": sample.get("t_sec"), "tracking_error_m": round(error, 6), "signed_lateral_error_m": round(lateral, 6)}
            for sample, (error, lateral) in zip(core_trajectory, errors_and_signed)
        ],
    }


def plot_tracking_error(report: dict[str, Any], path: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    samples = report.get("tracking_error_samples") or []
    if not samples:
        return
    fig, axis = plt.subplots(figsize=(11, 5))
    axis.plot([s["t_sec"] for s in samples], [s["tracking_error_m"] for s in samples], label="nearest-path error")
    axis.plot([s["t_sec"] for s in samples], [s["signed_lateral_error_m"] for s in samples], alpha=0.65, label="signed lateral error")
    axis.axhline(report["thresholds"]["mean_tracking_error_m"], color="#cc5500", linestyle="--", label="mean-error threshold")
    axis.axhline(0.0, color="black", linewidth=0.6)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("error (m)")
    axis.set_title("RSLG-SLAM quadruped visual-kinematic proxy tracking quality")
    axis.legend()
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)
