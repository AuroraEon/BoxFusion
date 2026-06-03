"""Shared data structures and defaults for the lightweight RSLG-SLAM executor backend."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def distance(a: dict[str, Any] | list | tuple, b: dict[str, Any] | list | tuple) -> float:
    if isinstance(a, dict):
        ax, ay = float(a["x"]), float(a["y"])
    else:
        ax, ay = float(a[0]), float(a[1])
    if isinstance(b, dict):
        bx, by = float(b["x"]), float(b["y"])
    else:
        bx, by = float(b[0]), float(b[1])
    return math.hypot(bx - ax, by - ay)


def route_length(points: list[dict[str, Any]]) -> float:
    return sum(distance(l, r) for l, r in zip(points, points[1:]))


@dataclass
class ControllerParams:
    """Parameters for the curvature-based pure pursuit controller."""

    max_linear_speed: float = 0.12
    max_angular_speed: float = 0.38
    lookahead_base: float = 0.65
    lookahead_min: float = 0.40
    lookahead_max: float = 1.00
    heading_kp: float = 0.80
    goal_tolerance: float = 0.30
    approach_position_tolerance_m: float = 0.35
    approach_controller_stop_tolerance_m: float = 0.30
    approach_creep_max_attempts: int = 5
    approach_creep_step_m: float = 0.04
    approach_settle_sec: float = 0.70
    approach_creep_boundary_upper_m: float = 0.38
    yaw_kp: float = 1.0
    max_yaw_angular_speed: float = 0.25
    yaw_internal_tolerance_rad: float = 0.38
    yaw_report_tolerance_rad: float = 0.50
    yaw_timeout_sec: float = 30.0
    xy_drift_limit_m: float = 0.12
    timeout_sec: float = 240.0
    path_deviation_limit_m: float = 0.75
    inflation_radius_m: float = 0.17
    path_spacing_m: float = 0.20
    control_path_spacing: float = 0.60
    control_path_gateway_spacing: float = 0.30
    angular_smoothing_alpha: float = 0.40
    angular_rate_limit: float = 0.15
    rotate_enter_rad: float = 1.30
    rotate_exit_rad: float = 0.90
    control_rate_hz: float = 10.0
    corner_rounding_chaikin_iterations: int = 2
    corner_rounding_resample_spacing: float = 0.25

    def to_json(self) -> dict[str, Any]:
        return {
            "artifact_type": "task17c_lightweight_controller_params",
            "created_utc": now_iso(),
            "controller_type": "curvature_based_pure_pursuit_with_dynamic_lookahead_and_corner_rounding",
            "cmd_vel_topic": "/cmd_vel",
            "lookahead_distance_base_m": self.lookahead_base,
            "lookahead_min_m": self.lookahead_min,
            "lookahead_max_m": self.lookahead_max,
            "max_linear_speed_mps": self.max_linear_speed,
            "max_angular_speed_radps": self.max_angular_speed,
            "heading_kp": self.heading_kp,
            "goal_tolerance_m": self.goal_tolerance,
            "approach_position_tolerance_m": self.approach_position_tolerance_m,
            "approach_controller_stop_tolerance_m": self.approach_controller_stop_tolerance_m,
            "approach_creep_max_attempts": self.approach_creep_max_attempts,
            "approach_creep_step_m": self.approach_creep_step_m,
            "approach_settle_sec": self.approach_settle_sec,
            "approach_creep_boundary_upper_m": self.approach_creep_boundary_upper_m,
            "yaw_kp": self.yaw_kp,
            "max_yaw_angular_speed_radps": self.max_yaw_angular_speed,
            "yaw_internal_tolerance_rad": self.yaw_internal_tolerance_rad,
            "yaw_report_tolerance_rad": self.yaw_report_tolerance_rad,
            "yaw_timeout_sec": self.yaw_timeout_sec,
            "xy_drift_limit_m": self.xy_drift_limit_m,
            "runtime_timeout_sec": self.timeout_sec,
            "path_deviation_limit_m": self.path_deviation_limit_m,
            "angular_smoothing_alpha": self.angular_smoothing_alpha,
            "angular_rate_limit_per_step": self.angular_rate_limit,
            "rotate_enter_threshold_rad": self.rotate_enter_rad,
            "rotate_exit_threshold_rad": self.rotate_exit_rad,
            "control_rate_hz": self.control_rate_hz,
            "control_path_normal_spacing_m": self.control_path_spacing,
            "control_path_gateway_spacing_m": self.control_path_gateway_spacing,
            "corner_rounding_method": f"chaikin_{self.corner_rounding_chaikin_iterations}_iterations",
            "corner_rounding_resample_spacing_m": self.corner_rounding_resample_spacing,
            "nav2_used": False,
        }


@dataclass
class ControlPathResult:
    """Result of control path building (simplification + rounding + fallback)."""

    control_path: list[dict[str, Any]]
    simplified_path: list[dict[str, Any]]
    simplification_report: dict[str, Any]
    rounding_report: dict[str, Any]
    rounding_attempted: bool = True
    rounding_applied: bool = False
    fallback_to_original: bool = False
    final_control_path_source: str = "simplified_control_path"
    rounded_candidate_validation_passed: bool = False
    fallback_reason: str | None = None


@dataclass
class ApproachCandidate:
    """Selected approach candidate for object-facing navigation."""

    candidate_id: str
    world_xy: list[float]
    yaw: float
    distance_to_route_terminal_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"candidate_id": self.candidate_id, "world_xy": self.world_xy, "yaw": self.yaw}
        if self.distance_to_route_terminal_m is not None:
            d["distance_to_route_terminal_m"] = self.distance_to_route_terminal_m
        return d


@dataclass
class YawProxy:
    """Object yaw proxy for approach facing direction."""

    yaw_proxy_source: str
    yaw_proxy_xy: list[float]

    def to_dict(self) -> dict[str, Any]:
        return {"yaw_proxy_source": self.yaw_proxy_source, "yaw_proxy_xy": self.yaw_proxy_xy}


@dataclass
class ExecutionResult:
    """Top-level execution result."""

    success: bool = False
    failure_layer: str | None = None
    failure_reason: str | None = None
    fields: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = {"success": self.success, "failure_layer": self.failure_layer, "failure_reason": self.failure_reason}
        d.update(self.fields)
        return d
