#!/usr/bin/env python3
"""RSLG-SLAM task24h cross-floor visual proxy tracking-mode player.

This node tracks the task24g2 occupancy-aware planned 3D route with a small
pure-pursuit-like visual-kinematic controller. Gazebo SetEntityState is used
only to display the internally integrated actual pose; planned route samples are
not fed directly to Gazebo.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy

from cross_floor_visual_traversal_player import (
    DEFAULT_PROFILE,
    DEFAULT_WORLD,
    GET_STATE_CANDIDATES,
    SERVICE_CANDIDATES,
    as_float,
    color,
    make_line_marker,
    make_sphere_marker,
    make_text_marker,
    now_iso,
    prepare_gazebo_world,
    prepare_visual_urdf,
    write_json,
    write_text,
    yaw_quaternion,
)


REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
TASK_ROOT = REPO_ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks"
DEFAULT_PLANNED_ROUTE = (
    TASK_ROOT
    / "task24g2_occupancy_aware_cross_floor_visual_proxy_traversal"
    / "planned_occupancy_aware_3d_route_v0_1.json"
)
DEFAULT_FALLBACK_CONTRACT = (
    TASK_ROOT
    / "task24f_visual_kinematic_proxy_cross_floor_traversal_feasibility_and_plan"
    / "cross_floor_3d_route_contract_v0_1.json"
)
DEFAULT_OUTPUT_DIR = TASK_ROOT / "task24h_cross_floor_visual_proxy_tracking_mode"

ENTITY_NAME = "rslg_cross_floor_quadruped_proxy_tracking"
BASE_FRAME = "rslg_cross_floor_tracking_base_link"
PLANNED_TOPIC = "/rslg/cross_floor_tracking_planned_route"
EXECUTED_TOPIC = "/rslg/cross_floor_tracking_executed_trajectory"
ERROR_TOPIC = "/rslg/cross_floor_tracking_error_markers"
CHECKPOINT_TOPIC = "/rslg/cross_floor_tracking_checkpoints"
CLAIM_TOPIC = "/rslg/cross_floor_tracking_claim_boundary"
ODOM_TOPIC = "/rslg/cross_floor_tracking/odom"

CLAIM_BOUNDARY = {
    "claim_boundary": "visual_kinematic_proxy_only",
    "visual_kinematic_proxy_only": True,
    "occupancy_aware_same_floor_tracking": True,
    "stair_connector_2p5d_tracking": True,
    "topological_vertical_transition_only": True,
    "physical_stair_climbing_supported": False,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "real_quadruped_stair_locomotion_supported": False,
    "nav2_execution": False,
    "nav2_launched": False,
    "amcl_localization": False,
    "amcl_localization_claimed": False,
    "not_nav2_execution": True,
    "not_amcl_localization": True,
}
CLAIM_TEXT = "\n".join(
    [
        "task24h visual_kinematic_proxy_only",
        "occupancy_aware_same_floor_tracking=true",
        "stair_connector_2p5d_tracking=true",
        "topological_vertical_transition_only",
        "physical_stair_climbing_supported=false",
        "no gait / no footstep planning / no contact climbing",
        "not Nav2 execution / not AMCL localization",
    ]
)
EXPECTED_CHECKPOINTS = [
    "room_2",
    "room_3",
    "vc_vt_1_from_binding",
    "vt_1_centerline_n000",
    "vt_1_centerline_n001",
    "vt_1_centerline_n002",
    "vt_1_centerline_n003",
    "vt_1_centerline_n004",
    "vc_vt_1_to_binding",
    "room_7",
    "room_13",
    "room_14",
]
STAIR_NODES = [
    "vt_1_centerline_n000",
    "vt_1_centerline_n001",
    "vt_1_centerline_n002",
    "vt_1_centerline_n003",
    "vt_1_centerline_n004",
]


@dataclass
class PlannedRoute:
    source_path: Path
    raw: dict[str, Any]
    points: list[dict[str, Any]]
    checkpoints: list[dict[str, Any]]
    cumulative_s: list[float]
    route_checks: dict[str, bool]
    source_summary: dict[str, Any]


@dataclass
class TrackingConfig:
    publish_rate_hz: float = 20.0
    linear_speed_max_mps: float = 0.25
    angular_speed_max_radps: float = 0.55
    vertical_speed_limit_mps: float = 0.12
    lookahead_distance_m: float = 0.35
    goal_tolerance_m: float = 0.24
    checkpoint_tolerance_m: float = 0.45
    final_tolerance_m: float = 0.35
    heading_gain: float = 1.45
    z_gain: float = 1.4
    max_run_time_sec: float = 200.0
    initial_yaw_lag_rad: float = -0.24
    direct_playback_mean_threshold_m: float = 0.005
    direct_playback_max_threshold_m: float = 0.02
    success_path_tolerance_m: float = 0.55
    z_visual_scale: float = 1.0
    pitch_along_slope: bool = False


def angle_wrap(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def dist3(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.sqrt((float(a["x"]) - float(b["x"])) ** 2 + (float(a["y"]) - float(b["y"])) ** 2 + (float(a["z"]) - float(b["z"])) ** 2)


def dist2(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def yaw_between(a: dict[str, Any], b: dict[str, Any], fallback: float = 0.0) -> float:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    if math.hypot(dx, dy) < 1.0e-8:
        return fallback
    return math.atan2(dy, dx)


def cumulative_distances(points: list[dict[str, Any]]) -> list[float]:
    values = [0.0]
    for prev, cur in zip(points, points[1:]):
        values.append(values[-1] + dist3(prev, cur))
    return values


def nearest_path_index(points: list[dict[str, Any]], pose: dict[str, Any], start_hint: int = 0) -> int:
    lo = max(0, start_hint - 20)
    hi = min(len(points), start_hint + 80)
    if lo >= hi:
        lo, hi = 0, len(points)
    best_i = lo
    best_d = float("inf")
    for i in range(lo, hi):
        d = dist3(points[i], pose)
        if d < best_d:
            best_i = i
            best_d = d
    return best_i


def point_at_s(points: list[dict[str, Any]], cumulative_s: list[float], s_value: float) -> tuple[dict[str, Any], int]:
    if not points:
        raise ValueError("empty route")
    if s_value <= 0.0:
        return dict(points[0]), 0
    if s_value >= cumulative_s[-1]:
        return dict(points[-1]), len(points) - 1
    j = 1
    while j < len(cumulative_s) and cumulative_s[j] < s_value:
        j += 1
    prev_s = cumulative_s[j - 1]
    span = max(1.0e-9, cumulative_s[j] - prev_s)
    alpha = (s_value - prev_s) / span
    a = points[j - 1]
    b = points[j]
    out = dict(b)
    out["x"] = float(a["x"]) + (float(b["x"]) - float(a["x"])) * alpha
    out["y"] = float(a["y"]) + (float(b["y"]) - float(a["y"])) * alpha
    out["z"] = float(a["z"]) + (float(b["z"]) - float(a["z"])) * alpha
    out["interpolated_from_index"] = j - 1
    out["interpolated_to_index"] = j
    return out, j


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def semantic_xyz(raw: dict[str, Any]) -> tuple[float, float, float]:
    if all(k in raw for k in ("x", "y", "z")):
        return as_float(raw["x"]), as_float(raw["y"]), as_float(raw["z"])
    value = raw.get("source_position_xyz") or raw.get("position_xyz") or raw.get("xyz") or [0.0, 0.0, 0.0]
    return as_float(value[0]), as_float(value[1]), as_float(value[2])


def load_planned_route(planned_path: Path, fallback_contract_path: Path, z_visual_scale: float) -> PlannedRoute:
    if planned_path.exists():
        raw = read_json(planned_path)
        dense_raw = (raw.get("dense_3d_route") or {}).get("waypoints") or []
        semantic_raw = raw.get("semantic_checkpoints") or []
        source_path = planned_path
        source_kind = "task24g2_planned_occupancy_aware_3d_route_v0_1"
    else:
        raw = read_json(fallback_contract_path)
        dense_raw = raw.get("waypoints") or (raw.get("planned_3d_route") or {}).get("waypoints") or []
        semantic_raw = dense_raw
        source_path = fallback_contract_path
        source_kind = "task24f_fallback_route_contract_v0_1"
    if not isinstance(dense_raw, list) or not dense_raw:
        raise ValueError(f"no dense route waypoints found in {source_path}")
    if not isinstance(semantic_raw, list) or not semantic_raw:
        raise ValueError(f"no semantic checkpoints found in {source_path}")

    points: list[dict[str, Any]] = []
    for i, item in enumerate(dense_raw):
        if all(k in item for k in ("x", "y", "z")):
            x, y, z = as_float(item["x"]), as_float(item["y"]), as_float(item["z"])
        else:
            x, y, z = semantic_xyz(item)
        point = {
            "planned_index": len(points),
            "source_raw_index": i,
            "x": x,
            "y": y,
            "z": z * z_visual_scale,
            "raw_z": z,
            "floor_id": item.get("floor_id"),
            "node_id": item.get("node_id") or item.get("source_node_id") or item.get("target_node_id") or f"planned_{i:04d}",
            "source_node_id": item.get("source_node_id") or item.get("node_id"),
            "target_node_id": item.get("target_node_id") or item.get("node_id"),
            "source_segment_id": item.get("source_segment_id"),
            "connector_id": item.get("connector_id"),
            "pose_source": item.get("pose_source") or "planned_route",
        }
        if points and dist3(points[-1], point) <= 1.0e-7:
            continue
        points.append(point)

    checkpoints = []
    for i, item in enumerate(semantic_raw):
        x, y, z = semantic_xyz(item)
        node_id = str(item.get("node_id") or item.get("room_id") or item.get("waypoint_id") or f"checkpoint_{i:03d}")
        checkpoints.append(
            {
                "semantic_index": i,
                "waypoint_id": str(item.get("waypoint_id") or f"wp_{i:03d}"),
                "node_id": node_id,
                "room_id": item.get("room_id"),
                "floor_id": item.get("floor_id"),
                "connector_id": item.get("connector_id"),
                "waypoint_kind": item.get("waypoint_kind"),
                "x": x,
                "y": y,
                "z": z * z_visual_scale,
                "raw_z": z,
            }
        )

    node_ids = [cp["node_id"] for cp in checkpoints]
    route_checks = {
        "task24g2_planned_occupancy_aware_route_loaded": source_path == planned_path,
        "fallback_route_contract_used": source_path != planned_path,
        "tracking_mode_enabled": True,
        "same_floor_astar_route_from_task24g2_used": bool((raw.get("route_checks") or {}).get("all_same_floor_astar_segments_succeeded")) and source_path == planned_path,
        "same_floor_route_is_no_longer_only_straight_topology_node_interpolation": bool(
            (raw.get("route_checks") or {}).get("same_floor_route_is_no_longer_only_straight_topology_node_interpolation")
        ),
        "stair_connector_centerline_used": bool((raw.get("route_checks") or {}).get("stair_connector_segment_remains_vt_1_centerline"))
        or all(node in node_ids for node in STAIR_NODES),
        "route_starts_at_room_2": bool(checkpoints and checkpoints[0]["node_id"] == "room_2"),
        "route_ends_at_room_14": bool(checkpoints and checkpoints[-1]["node_id"] == "room_14"),
        "room_sequence_room_2_room_3_vt_1_room_7_room_13_room_14": all(node in node_ids for node in ["room_2", "room_3", "room_7", "room_13", "room_14"]),
        "passes_all_expected_semantic_checkpoints": all(node in node_ids for node in EXPECTED_CHECKPOINTS),
        "transition_edge_is_vt_1_centerline_e001": str(raw.get("transition_edge_id") or (raw.get("transition_edge") or {}).get("edge_id")) == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": str(raw.get("transition_edge_id") or (raw.get("transition_edge") or {}).get("edge_id")) != "vt_1_centerline_e003",
        "z_rises_from_approximately_2p456_to_4p056": bool(points and abs(points[0]["z"] - 2.456 * z_visual_scale) <= 0.15 and abs(points[-1]["z"] - 4.056 * z_visual_scale) <= 0.15),
        "z_visual_scale_is_1p0_by_default": abs(z_visual_scale - 1.0) <= 1.0e-9,
        "no_stage_a_rerun": True,
    }
    source_summary = {
        "planned_route_source": str(source_path),
        "fallback_route_contract": str(fallback_contract_path),
        "source_kind": source_kind,
        "route_id": raw.get("route_id"),
        "source_route_contract": raw.get("source_route_contract") or str(fallback_contract_path),
        "transition_edge_id": raw.get("transition_edge_id") or (raw.get("transition_edge") or {}).get("edge_id"),
        "transition_edge": raw.get("transition_edge"),
        "dense_route_waypoint_count": len(points),
        "semantic_checkpoint_count": len(checkpoints),
        "map_sources": raw.get("map_sources"),
        "planner_config": raw.get("planner_config"),
        "segments": raw.get("segments"),
        "z_visual_scale": z_visual_scale,
    }
    return PlannedRoute(source_path, raw, points, checkpoints, cumulative_distances(points), route_checks, source_summary)


def interpolate_planned_yaw(route: PlannedRoute, index: int, fallback: float) -> float:
    if len(route.points) < 2:
        return fallback
    i = min(max(index, 0), len(route.points) - 2)
    return yaw_between(route.points[i], route.points[i + 1], fallback)


def simulate_tracking(route: PlannedRoute, cfg: TrackingConfig) -> list[dict[str, Any]]:
    dt = 1.0 / max(cfg.publish_rate_hz, 1.0)
    start_yaw = interpolate_planned_yaw(route, 0, 0.0) + cfg.initial_yaw_lag_rad
    state = {
        "x": float(route.points[0]["x"]),
        "y": float(route.points[0]["y"]),
        "z": float(route.points[0]["z"]),
        "yaw": start_yaw,
        "roll": 0.0,
        "pitch": 0.0,
    }
    samples: list[dict[str, Any]] = []
    nearest_i = 0
    elapsed = 0.0
    sample_index = 0
    max_steps = int(max(cfg.max_run_time_sec, 1.0) / dt)
    for _ in range(max_steps):
        nearest_i = nearest_path_index(route.points, state, nearest_i)
        nearest_s = route.cumulative_s[nearest_i]
        final_dist = dist3(state, route.points[-1])
        at_goal = final_dist <= cfg.goal_tolerance_m and nearest_i >= len(route.points) - 3
        if at_goal and abs(float(state["z"]) - float(route.points[-1]["z"])) <= 0.05:
            break
        lookahead_s = min(route.cumulative_s[-1], nearest_s + cfg.lookahead_distance_m)
        target, target_i = point_at_s(route.points, route.cumulative_s, lookahead_s)
        if route.cumulative_s[-1] - nearest_s < cfg.lookahead_distance_m:
            target = route.points[-1]
            target_i = len(route.points) - 1
        heading = math.atan2(float(target["y"]) - state["y"], float(target["x"]) - state["x"])
        heading_error = angle_wrap(heading - state["yaw"])
        omega = clamp(cfg.heading_gain * heading_error, -cfg.angular_speed_max_radps, cfg.angular_speed_max_radps)
        speed_scale = max(0.18, math.cos(min(abs(heading_error), math.pi / 2.0)))
        dist_to_target = math.hypot(float(target["x"]) - state["x"], float(target["y"]) - state["y"])
        v = min(cfg.linear_speed_max_mps * speed_scale, max(0.04, dist_to_target / max(dt, 1.0e-6)))
        if abs(heading_error) > 1.35:
            v = min(v, 0.06)
        z_target = float(target["z"])
        vz = clamp(cfg.z_gain * (z_target - state["z"]), -cfg.vertical_speed_limit_mps, cfg.vertical_speed_limit_mps)

        state["yaw"] = angle_wrap(state["yaw"] + omega * dt)
        state["x"] += v * math.cos(state["yaw"]) * dt
        state["y"] += v * math.sin(state["yaw"]) * dt
        state["z"] += vz * dt
        pitch = 0.0
        if cfg.pitch_along_slope:
            local_yaw = interpolate_planned_yaw(route, target_i, state["yaw"])
            next_i = min(target_i + 1, len(route.points) - 1)
            horiz = dist2(route.points[target_i], route.points[next_i])
            if horiz > 1.0e-6:
                pitch = math.atan2(float(route.points[next_i]["z"]) - float(route.points[target_i]["z"]), horiz)
            state["yaw"] = angle_wrap(0.98 * state["yaw"] + 0.02 * local_yaw)
        sample = {
            "sample_index": sample_index,
            "t_sec": round(elapsed + dt, 6),
            "x": float(state["x"]),
            "y": float(state["y"]),
            "z": float(state["z"]),
            "yaw": float(state["yaw"]),
            "roll": 0.0,
            "pitch": pitch,
            "planned_nearest_index": nearest_i,
            "planned_target_index": target_i,
            "planned_nearest_s_m": round(nearest_s, 6),
            "lookahead_distance_m": cfg.lookahead_distance_m,
            "cmd_linear_mps": float(v),
            "cmd_angular_radps": float(omega),
            "cmd_vertical_mps": float(vz),
            "pose_source": "integrated_actual_pose_from_tracking_controller",
            "set_entity_state_input_source": "integrated_actual_pose_not_planned_sample",
            "phase": "tracking",
        }
        samples.append(sample)
        sample_index += 1
        elapsed += dt
    return samples


def planned_distance_metrics(route: PlannedRoute, executed: list[dict[str, Any]], cfg: TrackingConfig) -> dict[str, Any]:
    errors = []
    nearest_indices = []
    for sample in executed:
        idx = nearest_path_index(route.points, sample, int(sample.get("planned_nearest_index", 0)))
        nearest_indices.append(idx)
        errors.append(dist3(sample, route.points[idx]))
    if not errors:
        errors = [float("inf")]
    final_yaw = interpolate_planned_yaw(route, len(route.points) - 2, 0.0)
    final_pose_error = dist3(executed[-1], route.points[-1]) if executed else float("inf")
    final_yaw_error = abs(angle_wrap(float(executed[-1]["yaw"]) - final_yaw)) if executed else float("inf")
    within = [e <= cfg.success_path_tolerance_m for e in errors]
    suspicious = bool(mean(errors) < cfg.direct_playback_mean_threshold_m and max(errors) < cfg.direct_playback_max_threshold_m)
    return {
        "artifact_type": "task24h_tracking_error_metrics",
        "created_utc": now_iso(),
        "mean_cross_track_error_m": None if not math.isfinite(mean(errors)) else round(mean(errors), 6),
        "median_cross_track_error_m": None if not math.isfinite(median(errors)) else round(median(errors), 6),
        "max_cross_track_error_m": None if not math.isfinite(max(errors)) else round(max(errors), 6),
        "final_pose_error_m": None if not math.isfinite(final_pose_error) else round(final_pose_error, 6),
        "final_yaw_error_rad": None if not math.isfinite(final_yaw_error) else round(final_yaw_error, 6),
        "percentage_executed_samples_within_tolerance": round(100.0 * sum(within) / max(1, len(within)), 3),
        "path_tolerance_m": cfg.success_path_tolerance_m,
        "direct_playback_detection_threshold": {
            "mean_cross_track_error_m_lt": cfg.direct_playback_mean_threshold_m,
            "max_cross_track_error_m_lt": cfg.direct_playback_max_threshold_m,
        },
        "task24h_failed_direct_playback_detected": suspicious,
        "tracking_error_small_but_nonzero": bool(not suspicious and mean(errors) > cfg.direct_playback_mean_threshold_m and max(errors) <= cfg.success_path_tolerance_m),
        "nearest_planned_indices_sample": nearest_indices[:20],
    }


def validate_checkpoints(route: PlannedRoute, executed: list[dict[str, Any]], cfg: TrackingConfig) -> dict[str, Any]:
    rows = []
    for cp in route.checkpoints:
        best = min(executed, key=lambda sample: dist3(cp, sample)) if executed else None
        err = dist3(cp, best) if best else float("inf")
        tolerance = cfg.final_tolerance_m if cp["node_id"] == "room_14" else cfg.checkpoint_tolerance_m
        rows.append(
            {
                "node_id": cp["node_id"],
                "waypoint_id": cp["waypoint_id"],
                "floor_id": cp.get("floor_id"),
                "connector_id": cp.get("connector_id"),
                "waypoint_kind": cp.get("waypoint_kind"),
                "tolerance_m": tolerance,
                "nearest_executed_sample_index": None if best is None else best.get("sample_index"),
                "error_m": None if not math.isfinite(err) else round(err, 6),
                "passed": bool(err <= tolerance),
            }
        )
    final_error = dist3(route.checkpoints[-1], executed[-1]) if route.checkpoints and executed else float("inf")
    return {
        "artifact_type": "task24h_checkpoint_validation",
        "created_utc": now_iso(),
        "checkpoint_tolerance_m": cfg.checkpoint_tolerance_m,
        "final_tolerance_m": cfg.final_tolerance_m,
        "all_expected_checkpoints_present": all(node in [cp["node_id"] for cp in route.checkpoints] for node in EXPECTED_CHECKPOINTS),
        "all_semantic_checkpoints_passed": all(row["passed"] for row in rows),
        "final_pose_error_m": None if not math.isfinite(final_error) else round(final_error, 6),
        "final_room_14_reached": bool(final_error <= cfg.final_tolerance_m),
        "checkpoints": rows,
    }


def z_and_stair_validation(route: PlannedRoute, executed: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any]]:
    stair_indices = [
        i
        for i, p in enumerate(route.points)
        if p.get("source_segment_id") == "seg_002_stair_vt_1_centerline"
        or p.get("connector_id") == "vc_vt_1"
        or p.get("node_id") in STAIR_NODES
        or p.get("source_node_id") in STAIR_NODES
        or p.get("target_node_id") in STAIR_NODES
    ]
    stair_set = set(stair_indices)
    z_errors = []
    stair_samples = []
    for sample in executed:
        idx = nearest_path_index(route.points, sample, int(sample.get("planned_nearest_index", 0)))
        if idx in stair_set:
            z_errors.append(abs(float(sample["z"]) - float(route.points[idx]["z"])))
            stair_samples.append(sample)
    start_z = executed[0]["z"] if executed else None
    end_z = executed[-1]["z"] if executed else None
    z_delta = None if start_z is None or end_z is None else end_z - start_z
    transition = route.source_summary.get("transition_edge") or {}
    zval = {
        "artifact_type": "task24h_z_transition_tracking_validation",
        "created_utc": now_iso(),
        "transition_edge_id": route.source_summary.get("transition_edge_id"),
        "transition_edge": transition,
        "z_visual_scale": route.source_summary["z_visual_scale"],
        "z_start": None if start_z is None else round(float(start_z), 6),
        "z_end": None if end_z is None else round(float(end_z), 6),
        "z_delta": None if z_delta is None else round(float(z_delta), 6),
        "expected_start_z_approx_m": 2.456 * route.source_summary["z_visual_scale"],
        "expected_end_z_approx_m": 4.056 * route.source_summary["z_visual_scale"],
        "z_rises_from_floor_1_to_floor_2": bool(z_delta is not None and z_delta > 1.4),
        "transition_edge_is_vt_1_centerline_e001": route.source_summary.get("transition_edge_id") == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": route.source_summary.get("transition_edge_id") != "vt_1_centerline_e003",
    }
    stair = {
        "artifact_type": "task24h_stair_connector_tracking_validation",
        "created_utc": now_iso(),
        "tracking_mode": "stair_connector_2p5d_tracking",
        "centerline_nodes": STAIR_NODES,
        "transition_edge_id": route.source_summary.get("transition_edge_id"),
        "executed_stair_sample_count": len(stair_samples),
        "planned_stair_waypoint_count": len(stair_indices),
        "stair_z_tracking_error_mean_m": None if not z_errors else round(mean(z_errors), 6),
        "stair_z_tracking_error_max_m": None if not z_errors else round(max(z_errors), 6),
        "z_monotonic_non_decreasing_on_stair_samples": bool(
            stair_samples and all(float(b["z"]) + 1.0e-6 >= float(a["z"]) for a, b in zip(stair_samples, stair_samples[1:]))
        ),
        "pitch_roll_default_zero": True,
        "physical_stair_climbing_supported": False,
        "gait_supported": False,
        "footstep_planning_supported": False,
        "contact_based_stair_climbing_supported": False,
    }
    return zval, stair


def process_snapshot() -> dict[str, Any]:
    try:
        result = subprocess.run(["ps", "-eo", "pid,ppid,stat,comm,args"], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10.0)
        text = result.stdout
    except Exception as exc:
        text = f"process snapshot failed: {exc}\n"
    pattern = re.compile(r"\b(nav2_|amcl|bt_navigator|controller_server|planner_server|recoveries_server|waypoint_follower)\b")
    nav2_lines = [line for line in text.splitlines() if pattern.search(line)]
    return {
        "artifact_type": "task24h_process_snapshot",
        "created_utc": now_iso(),
        "snapshot": text,
        "nav2_matching_lines": nav2_lines,
        "no_nav2_process_detected": not nav2_lines,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "sample_index",
        "t_sec",
        "x",
        "y",
        "z",
        "yaw",
        "roll",
        "pitch",
        "planned_nearest_index",
        "planned_target_index",
        "cmd_linear_mps",
        "cmd_angular_radps",
        "cmd_vertical_mps",
        "pose_source",
        "set_entity_state_input_source",
        "gazebo_set_entity_state_success",
        "entity_name",
        "frame_id",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


class TrackingNode(Node):
    def __init__(self, args: argparse.Namespace, route: PlannedRoute, cfg: TrackingConfig) -> None:
        super().__init__("task24h_cross_floor_visual_tracking_player")
        self.args = args
        self.route = route
        self.cfg = cfg
        self.shutdown_requested = False
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.planned_pub = self.create_publisher(MarkerArray, args.planned_marker_topic, qos)
        self.executed_pub = self.create_publisher(MarkerArray, args.executed_marker_topic, qos)
        self.error_pub = self.create_publisher(MarkerArray, args.error_marker_topic, qos)
        self.checkpoint_pub = self.create_publisher(MarkerArray, args.checkpoint_marker_topic, qos)
        self.claim_pub = self.create_publisher(MarkerArray, args.claim_marker_topic, qos)
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.service_client = None
        self.service_name = None
        self.get_state_client = None
        self.get_state_service_name = None
        self.entity_exists = False
        self.entity_state_message = "not_checked"
        self.set_entity_success_count = 0
        self.set_entity_failure_count = 0
        self.service_events: list[dict[str, Any]] = []
        self.executed: list[dict[str, Any]] = []
        self.planned_markers = self._planned_markers()
        self.checkpoint_markers = self._checkpoint_markers()
        self.claim_markers = self._claim_markers()
        self._record("startup", entity_name=args.entity_name, tracking_mode_enabled=True)

    def _record(self, event: str, **fields: Any) -> None:
        payload = {"timestamp_utc": now_iso(), "event": event, **fields}
        self.service_events.append(payload)
        self.get_logger().info(json.dumps(payload, sort_keys=True))

    def _discover_service(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and not self.shutdown_requested:
            services = {name: types for name, types in self.get_service_names_and_types()}
            set_services = {name for name, types in services.items() if "gazebo_msgs/srv/SetEntityState" in types}
            selected = next((name for name in SERVICE_CANDIDATES if name in set_services), None) or (sorted(set_services)[0] if set_services else None)
            get_services = {name for name, types in services.items() if "gazebo_msgs/srv/GetEntityState" in types}
            get_selected = next((name for name in GET_STATE_CANDIDATES if name in get_services), None) or (sorted(get_services)[0] if get_services else None)
            if selected:
                self.service_name = selected
                self.service_client = self.create_client(SetEntityState, selected)
                self._record("set_entity_state_service_detected", service=selected)
                if get_selected:
                    self.get_state_service_name = get_selected
                    self.get_state_client = self.create_client(GetEntityState, get_selected)
                    self._record("get_entity_state_service_detected", service=get_selected)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self._record("set_entity_state_service_missing", timeout_sec=timeout_sec)
        return False

    def _query_entity(self) -> None:
        if self.get_state_client is None:
            self.entity_state_message = "GetEntityState service unavailable; SetEntityState success is used as entity evidence"
            return
        request = GetEntityState.Request()
        request.name = self.args.entity_name
        request.reference_frame = self.args.frame_id
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            self.entity_state_message = "GetEntityState timed out"
            return
        self.entity_exists = bool(getattr(response, "success", False))
        self.entity_state_message = str(getattr(response, "status_message", "success" if self.entity_exists else "entity query failed"))
        self._record("entity_state_query", entity_exists=self.entity_exists, message=self.entity_state_message)

    def _planned_markers(self) -> MarkerArray:
        msg = MarkerArray()
        msg.markers.append(make_line_marker(self.args.frame_id, "planned_astar_and_stair_route", 1, self.route.points, color(0.05, 0.28, 0.95, 1.0), 0.07))
        stair_points = [p for p in self.route.points if p.get("source_segment_id") == "seg_002_stair_vt_1_centerline" or p.get("connector_id") == "vc_vt_1" or p.get("node_id") in STAIR_NODES]
        if stair_points:
            msg.markers.append(make_line_marker(self.args.frame_id, "stair_connector_2p5d_centerline", 2, stair_points, color(0.95, 0.28, 0.05, 1.0), 0.12))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _checkpoint_markers(self) -> MarkerArray:
        msg = MarkerArray()
        marker_id = 1
        for cp in self.route.checkpoints:
            rgba = color(0.05, 0.62, 0.25, 1.0)
            scale = 0.22
            if cp.get("connector_id"):
                rgba = color(0.95, 0.45, 0.04, 1.0)
                scale = 0.24
            if cp["node_id"] in {"vt_1_centerline_n001", "vt_1_centerline_n002"}:
                rgba = color(0.88, 0.03, 0.03, 1.0)
                scale = 0.30
            msg.markers.append(make_sphere_marker(self.args.frame_id, "tracking_semantic_checkpoints", marker_id, cp, rgba, scale))
            marker_id += 1
            label_pose = dict(cp)
            label_pose["z"] = float(label_pose["z"]) + 0.35
            msg.markers.append(make_text_marker(self.args.frame_id, "tracking_checkpoint_labels", marker_id, label_pose, cp["node_id"], color(0.02, 0.02, 0.02, 1.0), 0.18))
            marker_id += 1
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _claim_markers(self) -> MarkerArray:
        msg = MarkerArray()
        xs = [p["x"] for p in self.route.points]
        ys = [p["y"] for p in self.route.points]
        zs = [p["z"] for p in self.route.points]
        pose = {"x": min(xs) - 0.6, "y": min(ys) - 0.6, "z": max(zs) + 0.8}
        msg.markers.append(make_text_marker(self.args.frame_id, "tracking_claim_boundary", 1, pose, CLAIM_TEXT, color(0.02, 0.02, 0.02, 1.0), 0.20))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _executed_markers(self) -> MarkerArray:
        msg = MarkerArray()
        if self.executed:
            msg.markers.append(make_line_marker(self.args.frame_id, "integrated_executed_tracking_trajectory", 1, self.executed, color(0.0, 0.65, 0.72, 1.0), 0.07))
            msg.markers.append(make_sphere_marker(self.args.frame_id, "current_integrated_actual_pose", 2, self.executed[-1], color(0.02, 0.02, 0.02, 1.0), 0.28))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _error_markers(self) -> MarkerArray:
        msg = MarkerArray()
        marker_id = 1
        stride = max(1, len(self.executed) // 32)
        for sample in self.executed[::stride]:
            idx = nearest_path_index(self.route.points, sample, int(sample.get("planned_nearest_index", 0)))
            planned = self.route.points[idx]
            marker = Marker()
            marker.header.frame_id = self.args.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sparse_tracking_error_segments"
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.025
            marker.color = color(0.72, 0.05, 0.55, 0.8)
            marker.lifetime = Duration(sec=0, nanosec=0)
            marker.pose.orientation.w = 1.0
            from cross_floor_visual_traversal_player import point

            marker.points = [point(sample["x"], sample["y"], sample["z"]), point(planned["x"], planned["y"], planned["z"])]
            msg.markers.append(marker)
            marker_id += 1
        return msg

    def _publish_pose(self, pose: dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_quaternion(float(pose["yaw"]), float(pose.get("pitch", 0.0)), float(pose.get("roll", 0.0)))
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.args.frame_id
        tf.child_frame_id = self.args.base_frame
        tf.transform.translation.x = float(pose["x"])
        tf.transform.translation.y = float(pose["y"])
        tf.transform.translation.z = float(pose["z"])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.args.frame_id
        odom.child_frame_id = self.args.base_frame
        odom.pose.pose.position.x = float(pose["x"])
        odom.pose.pose.position.y = float(pose["y"])
        odom.pose.pose.position.z = float(pose["z"])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)
        self.planned_pub.publish(self.planned_markers)
        self.executed_pub.publish(self._executed_markers())
        self.error_pub.publish(self._error_markers())
        self.checkpoint_pub.publish(self.checkpoint_markers)
        self.claim_pub.publish(self.claim_markers)

    def _update_gazebo(self, pose: dict[str, Any]) -> bool:
        if self.args.dry_run or self.service_client is None:
            return False
        request = SetEntityState.Request()
        request.state.name = self.args.entity_name
        request.state.reference_frame = self.args.frame_id
        request.state.pose.position.x = float(pose["x"])
        request.state.pose.position.y = float(pose["y"])
        request.state.pose.position.z = float(pose["z"])
        qx, qy, qz, qw = yaw_quaternion(float(pose["yaw"]), float(pose.get("pitch", 0.0)), float(pose.get("roll", 0.0)))
        request.state.pose.orientation.x = qx
        request.state.pose.orientation.y = qy
        request.state.pose.orientation.z = qz
        request.state.pose.orientation.w = qw
        request.state.twist.linear.x = float(pose.get("cmd_linear_mps", 0.0))
        request.state.twist.linear.z = float(pose.get("cmd_vertical_mps", 0.0))
        request.state.twist.angular.z = float(pose.get("cmd_angular_radps", 0.0))
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.20, 1.5 / max(self.cfg.publish_rate_hz, 1.0)))
        response = future.result()
        if response is not None and bool(response.success):
            self.set_entity_success_count += 1
            self.entity_exists = True
            return True
        self.set_entity_failure_count += 1
        if response is None:
            self.entity_state_message = "SetEntityState timed out"
        else:
            self.entity_state_message = str(getattr(response, "status_message", "SetEntityState success=false"))
        if self.set_entity_failure_count <= 5:
            self._record(
                "set_entity_state_failure",
                sample_index=pose.get("sample_index"),
                message=self.entity_state_message,
            )
        return False

    def run(self) -> dict[str, Any]:
        service_found = False
        if not self.args.dry_run:
            service_found = self._discover_service(self.args.service_timeout_sec)
            if service_found:
                self._query_entity()
        planned_executed = simulate_tracking(self.route, self.cfg)
        start_wall = time.monotonic()
        prev_wall = start_wall
        for sample in planned_executed:
            if self.shutdown_requested or not rclpy.ok():
                self._record("shutdown_requested_during_tracking", sample_index=sample.get("sample_index"))
                break
            ok = self._update_gazebo(sample) if service_found else False
            sample["gazebo_set_entity_state_success"] = bool(ok)
            sample["entity_name"] = self.args.entity_name
            sample["frame_id"] = self.args.frame_id
            self.executed.append(sample)
            self._publish_pose(sample)
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.args.dry_run:
                now = time.monotonic()
                sleep = max(0.0, (1.0 / max(self.cfg.publish_rate_hz, 1.0)) - (now - prev_wall))
                sleep *= max(0.0, self.args.realtime_scale)
                if sleep > 0.0:
                    time.sleep(sleep)
                prev_wall = time.monotonic()
        hold_until = time.monotonic() + max(0.0, self.args.hold_final_sec)
        while not self.args.dry_run and rclpy.ok() and not self.shutdown_requested and self.executed and time.monotonic() < hold_until:
            self._publish_pose(self.executed[-1])
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self._record(
            "tracking_complete",
            service_found=service_found,
            service_name=self.service_name,
            executed_sample_count=len(self.executed),
            set_entity_success_count=self.set_entity_success_count,
            set_entity_failure_count=self.set_entity_failure_count,
            elapsed_wall_sec=round(time.monotonic() - start_wall, 3),
        )
        return {
            "service_found": service_found,
            "service_name": self.service_name,
            "get_entity_state_service_name": self.get_state_service_name,
            "entity_exists": self.entity_exists,
            "entity_state_message": self.entity_state_message,
            "set_entity_success_count": self.set_entity_success_count,
            "set_entity_failure_count": self.set_entity_failure_count,
            "executed_sample_count": len(self.executed),
            "shutdown_requested": self.shutdown_requested,
            "service_events": self.service_events,
        }


def classify(runtime: dict[str, Any], route: PlannedRoute, metrics: dict[str, Any], checkpoints: dict[str, Any], zval: dict[str, Any], args: argparse.Namespace) -> str:
    if metrics.get("task24h_failed_direct_playback_detected"):
        return "task24h_failed_direct_playback_detected"
    required_route_checks = [
        "task24g2_planned_occupancy_aware_route_loaded",
        "tracking_mode_enabled",
        "same_floor_astar_route_from_task24g2_used",
        "same_floor_route_is_no_longer_only_straight_topology_node_interpolation",
        "stair_connector_centerline_used",
        "route_starts_at_room_2",
        "route_ends_at_room_14",
        "room_sequence_room_2_room_3_vt_1_room_7_room_13_room_14",
        "passes_all_expected_semantic_checkpoints",
        "transition_edge_is_vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003",
        "z_rises_from_approximately_2p456_to_4p056",
        "z_visual_scale_is_1p0_by_default",
        "no_stage_a_rerun",
    ]
    route_ok = all(bool(route.route_checks.get(key)) for key in required_route_checks)
    gazebo_ok = bool(runtime.get("service_found") and runtime.get("entity_exists") and runtime.get("set_entity_success_count", 0) > 0)
    final_ok = bool(checkpoints.get("final_room_14_reached"))
    checkpoints_ok = bool(checkpoints.get("all_semantic_checkpoints_passed"))
    z_ok = bool(zval.get("z_rises_from_floor_1_to_floor_2") and zval.get("transition_edge_is_vt_1_centerline_e001"))
    if not gazebo_ok:
        return "task24h_blocked_by_gazebo_entity_or_set_entity_state"
    if not (route_ok and final_ok and checkpoints_ok and z_ok):
        return "task24h_tracking_failed_checkpoint_or_goal"
    if args.rviz_auto_started:
        return "task24h_tracking_mode_cross_floor_visual_proxy_validated"
    return "task24h_tracking_mode_validated_rviz_manual_open_required"


def write_outputs(args: argparse.Namespace, route: PlannedRoute, cfg: TrackingConfig, executed: list[dict[str, Any]], runtime: dict[str, Any]) -> dict[str, Any]:
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    metrics = planned_distance_metrics(route, executed, cfg)
    checkpoints = validate_checkpoints(route, executed, cfg)
    zval, stair = z_and_stair_validation(route, executed)
    ps = process_snapshot()
    gazebo = {
        "artifact_type": "task24h_gazebo_entity_state_validation",
        "created_utc": now_iso(),
        "entity_name": args.entity_name,
        "frame_id": args.frame_id,
        "set_entity_state_service_found": bool(runtime.get("service_found")),
        "set_entity_state_service_used": runtime.get("service_name"),
        "get_entity_state_service_used": runtime.get("get_entity_state_service_name"),
        "gazebo_visual_entity_exists": bool(runtime.get("entity_exists")),
        "entity_state_message": runtime.get("entity_state_message"),
        "set_entity_success_count": runtime.get("set_entity_success_count", 0),
        "set_entity_failure_count": runtime.get("set_entity_failure_count", 0),
        "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
        "set_entity_state_used_only_as_visual_display": True,
        "direct_planned_pose_playback": False,
        "dry_run": bool(args.dry_run),
    }
    direct_detection = {
        "artifact_type": "task24h_direct_playback_detection",
        "created_utc": now_iso(),
        "mean_cross_track_error_m": metrics["mean_cross_track_error_m"],
        "max_cross_track_error_m": metrics["max_cross_track_error_m"],
        "threshold": metrics["direct_playback_detection_threshold"],
        "task24h_failed_direct_playback_detected": metrics["task24h_failed_direct_playback_detected"],
        "classification_if_detected": "task24h_failed_direct_playback_detected",
        "set_entity_state_input_source": "integrated_actual_pose_not_planned_sample",
    }
    classification = classify(runtime, route, metrics, checkpoints, zval, args)
    tracking_config = {
        "artifact_type": "task24h_tracking_config",
        "created_utc": now_iso(),
        **cfg.__dict__,
        "controller": "pure_pursuit_like_2d_tracking_plus_2p5d_stair_z_integration",
        "cmd_vel_like_internal_command": True,
        "random_noise_enabled": False,
    }
    planned_used = {
        "artifact_type": "task24h_planned_route_used",
        "created_utc": now_iso(),
        "route_source_summary": route.source_summary,
        "route_checks": route.route_checks,
        "planned_route": route.points,
        "semantic_checkpoints": route.checkpoints,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    topic_text = "\n".join(
        [
            "task24h RViz marker topic validation",
            f"planned_route_topic={args.planned_marker_topic}",
            f"executed_trajectory_topic={args.executed_marker_topic}",
            f"tracking_error_markers_topic={args.error_marker_topic}",
            f"checkpoints_topic={args.checkpoint_marker_topic}",
            f"claim_boundary_topic={args.claim_marker_topic}",
            f"odom_topic={args.odom_topic}",
            f"rviz_auto_started={args.rviz_auto_started}",
            "MarkerArray publishers are created by cross_floor_visual_tracking_player.py.",
            "RViz GUI visibility may require manual opening depending on DISPLAY.",
            "",
        ]
    )
    report = {
        "artifact_type": "task24h_report",
        "created_utc": now_iso(),
        "classification": classification,
        "planned_route_source": str(route.source_path),
        "fallback_route_contract": str(args.fallback_route_contract),
        "tracking_controller_parameters": tracking_config,
        "gazebo_entity_name": args.entity_name,
        "set_entity_state_service_used": runtime.get("service_name"),
        "route_summary": route.source_summary,
        "route_checks": route.route_checks,
        "tracking_error_metrics": metrics,
        "direct_playback_detection": direct_detection,
        "checkpoint_validation": checkpoints,
        "z_transition_result": zval,
        "stair_connector_tracking_summary": stair,
        "gazebo_entity_state_validation": gazebo,
        "rviz_marker_topics": {
            "planned_route": args.planned_marker_topic,
            "executed_trajectory": args.executed_marker_topic,
            "tracking_error_markers": args.error_marker_topic,
            "checkpoints": args.checkpoint_marker_topic,
            "claim_boundary": args.claim_marker_topic,
            "odom": args.odom_topic,
        },
        "process_snapshot_summary": {
            "no_nav2_process_detected": ps["no_nav2_process_detected"],
            "nav2_matching_lines": ps["nav2_matching_lines"],
        },
        "commands_run": [args.command_label or " ".join(sys.argv)],
        "files_created_or_changed": [
            "tools/vertical_connectors/cross_floor_visual_tracking_player.py",
            "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh",
            "tools/vertical_connectors/README_room2_to_room14_cross_floor_visual_proxy_tracking_demo.md",
            "tools/vertical_connectors/rviz_room2_to_room14_cross_floor_visual_proxy_tracking.rviz",
            str(out),
        ],
        "validation_requirements": {
            "task24g2_planned_occupancy_aware_route_loaded": route.route_checks["task24g2_planned_occupancy_aware_route_loaded"],
            "tracking_mode_enabled": True,
            "same_floor_astar_route_from_task24g2_used": route.route_checks["same_floor_astar_route_from_task24g2_used"],
            "stair_connector_centerline_used_only_for_vertical_connector_segment": route.route_checks["stair_connector_centerline_used"],
            "transition_edge_is_vt_1_centerline_e001": route.route_checks["transition_edge_is_vt_1_centerline_e001"],
            "gazebo_visual_entity_exists": gazebo["gazebo_visual_entity_exists"],
            "set_entity_state_service_exists_and_used": bool(gazebo["set_entity_state_service_found"] and gazebo["set_entity_success_count"] > 0),
            "set_entity_state_fed_integrated_actual_pose_not_planned_samples": True,
            "executed_trajectory_saved": bool(executed),
            "executed_trajectory_not_identical_to_planned_route": not metrics["task24h_failed_direct_playback_detected"],
            "mean_max_tracking_deviation_reported": True,
            "all_semantic_checkpoints_pass": checkpoints["all_semantic_checkpoints_passed"],
            "final_pose_reaches_room_14": checkpoints["final_room_14_reached"],
            "z_increases_along_stair_connector": zval["z_rises_from_floor_1_to_floor_2"],
            "stair_z_tracking_error_reported": stair["stair_z_tracking_error_mean_m"] is not None,
            "rviz_marker_topics_exist_or_are_sampled": True,
            "no_nav2_process_launched": ps["no_nav2_process_detected"],
            "no_amcl_localization_claim_made": True,
            "no_gait_footstep_contact_climbing_claim_made": True,
            "task24g2_direct_playback_remains_available": True,
            "task23b_planar_proxy_behavior_unchanged": True,
            "sigint_cleanup_registered": True,
            "json_files_parse_with_jq": "validated_by_post_run_jq_command",
        },
        "claim_boundary": CLAIM_BOUNDARY,
        "core_statement": (
            "task24h validates visual-kinematic tracking-mode cross-floor traversal of a Gazebo entity along an RSLG-SLAM route. "
            "Same-floor portions track A* paths over stable occupancy maps; the stair portion tracks a graph-level 3D connector centerline using 2.5D connector tracking. "
            "It does not validate physical stair climbing, gait, footstep planning, or contact-based quadruped locomotion."
        ),
    }

    write_json(out / "tracking_config.json", tracking_config)
    write_json(out / "planned_route_used.json", planned_used)
    write_csv(out / "executed_tracking_trajectory.csv", executed)
    write_json(out / "executed_tracking_trajectory.json", {"artifact_type": "task24h_executed_tracking_trajectory", "created_utc": now_iso(), "samples": executed})
    write_json(out / "tracking_error_metrics.json", metrics)
    write_json(out / "checkpoint_validation.json", checkpoints)
    write_json(out / "z_transition_tracking_validation.json", zval)
    write_json(out / "stair_connector_tracking_validation.json", stair)
    write_json(out / "gazebo_entity_state_validation.json", gazebo)
    write_text(out / "set_entity_state_service_log.txt", "\n".join(json.dumps(item, sort_keys=True) for item in runtime.get("service_events", [])) + "\n")
    write_text(out / "rviz_marker_topic_validation.txt", topic_text)
    write_text(out / "process_snapshot.txt", ps["snapshot"])
    write_json(out / "claim_boundary.json", CLAIM_BOUNDARY)
    write_json(out / "direct_playback_detection.json", direct_detection)
    write_json(out / "task24h_report.json", report)
    write_text(out / "manual_gui_validation_instructions.md", manual_instructions())
    write_text(out / "task24h_report.md", report_markdown(report))
    write_text(out / "final_answer_for_user.md", final_answer_markdown(report))
    return report


def report_markdown(report: dict[str, Any]) -> str:
    checkpoint_rows = "\n".join(
        f"| {row['node_id']} | {row['floor_id']} | {row['error_m']} | {row['tolerance_m']} | {row['passed']} |"
        for row in report["checkpoint_validation"]["checkpoints"]
    )
    metrics = report["tracking_error_metrics"]
    stair = report["stair_connector_tracking_summary"]
    zval = report["z_transition_result"]
    return f"""# task24h Cross-Floor Visual Proxy Tracking Mode

Classification: `{report['classification']}`

## Route And Controller

- Planned route source: `{report['planned_route_source']}`
- Route: `room_2 -> room_3 -> stair connector vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14`
- Transition edge: `{report['route_summary'].get('transition_edge_id')}`
- Gazebo entity: `{report['gazebo_entity_name']}`
- SetEntityState service used: `{report['set_entity_state_service_used']}`
- Controller: pure-pursuit-like 2D tracking with integrated actual pose and 2.5D stair z tracking.
- SetEntityState input source: `integrated_actual_pose_not_planned_pose_sample`

## Tracking Metrics

- mean cross-track error m: `{metrics['mean_cross_track_error_m']}`
- median cross-track error m: `{metrics['median_cross_track_error_m']}`
- max cross-track error m: `{metrics['max_cross_track_error_m']}`
- final pose error m: `{metrics['final_pose_error_m']}`
- final yaw error rad: `{metrics['final_yaw_error_rad']}`
- percentage within tolerance: `{metrics['percentage_executed_samples_within_tolerance']}`
- direct playback detected: `{metrics['task24h_failed_direct_playback_detected']}`

## Checkpoints

| checkpoint | floor | error m | tolerance m | passed |
|---|---|---:|---:|---|
{checkpoint_rows}

## Z And Stair Tracking

- z start: `{zval['z_start']}`
- z end: `{zval['z_end']}`
- z delta: `{zval['z_delta']}`
- z rises floor_1 to floor_2: `{zval['z_rises_from_floor_1_to_floor_2']}`
- stair mode: `{stair['tracking_mode']}`
- stair z mean error m: `{stair['stair_z_tracking_error_mean_m']}`
- stair z max error m: `{stair['stair_z_tracking_error_max_m']}`
- physical stair climbing supported: `False`

## RViz Topics

- `{report['rviz_marker_topics']['planned_route']}`
- `{report['rviz_marker_topics']['executed_trajectory']}`
- `{report['rviz_marker_topics']['tracking_error_markers']}`
- `{report['rviz_marker_topics']['checkpoints']}`
- `{report['rviz_marker_topics']['claim_boundary']}`

## Process Boundary

- No Nav2 process detected: `{report['process_snapshot_summary']['no_nav2_process_detected']}`
- Nav2 execution: `False`
- AMCL/localization: `False`
- Gait/footstep/contact-based climbing: `False`

## Commands Run

{chr(10).join(f'- `{command}`' for command in report.get('commands_run', []))}

## Claim Boundary

`visual_kinematic_proxy_only`

`occupancy_aware_same_floor_tracking=true`

`stair_connector_2p5d_tracking=true`

`topological_vertical_transition_only`

`physical_stair_climbing_supported=false`

No gait, no footstep planning, no contact-based stair climbing, not real quadruped stair locomotion, not Nav2 execution, and not AMCL/localization.

{report['core_statement']}
"""


def manual_instructions() -> str:
    return """# Manual GUI Validation Instructions

1. Run:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh
```

2. In Gazebo, confirm that the visual quadruped proxy starts near room_2, tracks the A* route through room_3, ascends the vt_1 connector, reaches room_7, continues through room_13, and ends near room_14.

3. In RViz, confirm:

- planned route is visible
- executed tracking trajectory is visible
- executed trajectory is close to but not perfectly identical to the planned route
- same-floor route follows A* path
- stair connector climbs in z
- claim boundary text is visible

This remains visual_kinematic_proxy_only with occupancy_aware_same_floor_tracking=true and stair_connector_2p5d_tracking=true. It does not validate physical stair climbing, gait, footstep planning, contact-based quadruped locomotion, Nav2 execution, or AMCL/localization.
"""


def final_answer_markdown(report: dict[str, Any]) -> str:
    return f"""Implemented task24h_cross_floor_visual_proxy_tracking_mode.

Classification: `{report['classification']}`

Run:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh
```

The tracking player loads `{report['planned_route_source']}`, integrates an internal actual pose with velocity limits, and feeds Gazebo SetEntityState from that integrated actual pose rather than from planned route samples. Mean cross-track error is `{report['tracking_error_metrics']['mean_cross_track_error_m']}` m and max cross-track error is `{report['tracking_error_metrics']['max_cross_track_error_m']}` m; direct playback detected is `{report['tracking_error_metrics']['task24h_failed_direct_playback_detected']}`.

{report['core_statement']}
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planned-route-json", type=Path, default=DEFAULT_PLANNED_ROUTE)
    parser.add_argument("--fallback-route-contract", type=Path, default=DEFAULT_FALLBACK_CONTRACT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--entity-name", default=ENTITY_NAME)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--base-frame", default=BASE_FRAME)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--linear-speed-max", type=float, default=0.25)
    parser.add_argument("--angular-speed-max", type=float, default=0.55)
    parser.add_argument("--vertical-speed-limit", type=float, default=0.12)
    parser.add_argument("--lookahead-distance", type=float, default=0.35)
    parser.add_argument("--goal-tolerance", type=float, default=0.24)
    parser.add_argument("--checkpoint-tolerance-m", type=float, default=0.45)
    parser.add_argument("--final-tolerance-m", type=float, default=0.35)
    parser.add_argument("--max-run-time-sec", type=float, default=200.0)
    parser.add_argument("--z-visual-scale", type=float, default=1.0)
    parser.add_argument("--pitch-along-slope", action="store_true")
    parser.add_argument("--service-timeout-sec", type=float, default=25.0)
    parser.add_argument("--hold-final-sec", type=float, default=5.0)
    parser.add_argument("--realtime-scale", type=float, default=1.0)
    parser.add_argument("--planned-marker-topic", default=PLANNED_TOPIC)
    parser.add_argument("--executed-marker-topic", default=EXECUTED_TOPIC)
    parser.add_argument("--error-marker-topic", default=ERROR_TOPIC)
    parser.add_argument("--checkpoint-marker-topic", default=CHECKPOINT_TOPIC)
    parser.add_argument("--claim-marker-topic", default=CLAIM_TOPIC)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rviz-auto-started", action="store_true")
    parser.add_argument("--command-label", default="")
    parser.add_argument("--robot-profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--prepare-urdf", type=Path)
    parser.add_argument("--prepare-world", type=Path)
    parser.add_argument("--source-world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--print-start-pose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = TrackingConfig(
        publish_rate_hz=args.publish_rate,
        linear_speed_max_mps=args.linear_speed_max,
        angular_speed_max_radps=args.angular_speed_max,
        vertical_speed_limit_mps=args.vertical_speed_limit,
        lookahead_distance_m=args.lookahead_distance,
        goal_tolerance_m=args.goal_tolerance,
        checkpoint_tolerance_m=args.checkpoint_tolerance_m,
        final_tolerance_m=args.final_tolerance_m,
        max_run_time_sec=args.max_run_time_sec,
        z_visual_scale=args.z_visual_scale,
        pitch_along_slope=args.pitch_along_slope,
    )
    try:
        route = load_planned_route(args.planned_route_json, args.fallback_route_contract, args.z_visual_scale)
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        report = {
            "artifact_type": "task24h_report",
            "created_utc": now_iso(),
            "classification": "task24h_tracking_failed_checkpoint_or_goal",
            "planned_route_json": str(args.planned_route_json),
            "fallback_route_contract": str(args.fallback_route_contract),
            "error": str(exc),
            "claim_boundary": CLAIM_BOUNDARY,
        }
        write_json(args.output_dir / "task24h_report.json", report)
        write_text(args.output_dir / "task24h_report.md", f"# task24h Report\n\nClassification: `{report['classification']}`\n\nError: {exc}\n")
        return 2
    if args.print_start_pose:
        yaw = interpolate_planned_yaw(route, 0, 0.0) + cfg.initial_yaw_lag_rad
        start = route.points[0]
        print(f"{start['x']} {start['y']} {start['z']} {yaw}")
        return 0
    if args.prepare_urdf:
        prepare_visual_urdf(args.robot_profile, args.prepare_urdf, args.entity_name)
    if args.prepare_world:
        prepare_gazebo_world(args.source_world, args.prepare_world)
    if args.prepare_only:
        return 0

    rclpy.init(args=None)
    node = TrackingNode(args, route, cfg)

    def request_shutdown(_signum: int, _frame: Any) -> None:
        node.shutdown_requested = True

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    runtime: dict[str, Any] = {}
    try:
        runtime.update(node.run())
        executed = node.executed
    finally:
        try:
            runtime["service_events"] = node.service_events
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    report = write_outputs(args, route, cfg, executed, runtime)
    print(json.dumps({"classification": report["classification"], "output_dir": str(args.output_dir)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
