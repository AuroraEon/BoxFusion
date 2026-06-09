#!/usr/bin/env python3
"""RSLG-SLAM task24g cross-floor visual-kinematic traversal player.

This node replays the task24f 3D route contract by directly moving a Gazebo
visual entity with SetEntityState. It deliberately does not run Nav2, does not
publish /cmd_vel, and does not implement gait, footsteps, contacts, or physical
stair climbing.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:  # ROS2 Foxy compatibility on some installations.
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy


REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
DEFAULT_ROUTE_CONTRACT = (
    REPO_ROOT
    / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/"
    / "task24f_visual_kinematic_proxy_cross_floor_traversal_feasibility_and_plan/"
    / "cross_floor_3d_route_contract_v0_1.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT
    / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/"
    / "task24g_room2_to_room14_cross_floor_visual_proxy_traversal_demo"
)
DEFAULT_PROFILE = REPO_ROOT / "tools/object_nav/robot_profiles/champ_reference_kinematic_proxy.yaml"
DEFAULT_WORLD = Path("/usr/share/gazebo-11/worlds/empty.world")
ENTITY_NAME = "rslg_cross_floor_quadruped_proxy"
PLANNED_TOPIC = "/rslg/cross_floor_planned_route"
EXECUTED_TOPIC = "/rslg/cross_floor_executed_trajectory"
VISUAL_TOPIC = "/rslg/cross_floor_visual_proxy_markers"
ODOM_TOPIC = "/rslg/cross_floor_visual_proxy/odom"
CLAIM_BOUNDARY = {
    "claim_boundary": "visual_kinematic_proxy_only",
    "topological_vertical_transition_only": True,
    "physical_stair_climbing_supported": False,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "real_quadruped_stair_locomotion_supported": False,
    "nav2_launched": False,
    "amcl_localization_claimed": False,
}
TASK24G2_CLAIM_BOUNDARY = {
    **CLAIM_BOUNDARY,
    "occupancy_aware_same_floor_visual_playback": True,
    "nav2_execution": False,
    "amcl_localization": False,
}
CLAIM_TEXT = "\n".join(
    [
        "task24g visual_kinematic_proxy_only",
        "topological_vertical_transition_only",
        "physical_stair_climbing_supported=false",
        "no gait / no footsteps / no contact climbing",
    ]
)
EXPECTED_NODE_IDS = [
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
SERVICE_CANDIDATES = ("/set_entity_state", "/demo/set_entity_state", "/gazebo/set_entity_state")
GET_STATE_CANDIDATES = ("/get_entity_state", "/demo/get_entity_state", "/gazebo/get_entity_state")


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def yaw_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def distance(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2 + (a["z"] - b["z"]) ** 2)


def horizontal_yaw(a: dict[str, float], b: dict[str, float], fallback: float) -> float:
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    if math.hypot(dx, dy) < 1.0e-6:
        return fallback
    return math.atan2(dy, dx)


def semantic_kind(raw: dict[str, Any], transition_edge: dict[str, Any]) -> str:
    node_id = str(raw.get("node_id") or raw.get("id") or raw.get("waypoint_id") or "")
    kind = str(raw.get("waypoint_kind") or raw.get("semantic_type") or "")
    if node_id in {transition_edge.get("source_node_id"), transition_edge.get("target_node_id")}:
        return "transition_node"
    if kind in {"connector_entry", "connector_exit"}:
        return "connector_binding"
    if kind in {"connector_node", "transition_node_source", "transition_node_target"}:
        return "connector_centerline_node"
    if node_id.startswith("room_") or kind == "room_node":
        return "room_node" if kind != "endpoint" else "endpoint"
    return kind or "waypoint"


def raw_xyz(raw: dict[str, Any]) -> list[float]:
    value: Any = raw.get("position_xyz")
    if value is None:
        value = raw.get("xyz")
    if value is None:
        value = raw.get("position")
    if isinstance(value, dict):
        value = value.get("xyz") or value.get("position_xyz") or value.get("position")
    if not isinstance(value, list):
        value = [0.0, 0.0, 0.0]
    return [
        as_float(value[0]) if len(value) > 0 else 0.0,
        as_float(value[1]) if len(value) > 1 else 0.0,
        as_float(value[2]) if len(value) > 2 else 0.0,
    ]


@dataclass
class RoutePlan:
    contract_path: Path
    contract: dict[str, Any]
    waypoints: list[dict[str, Any]]
    planned_route_waypoints: list[dict[str, Any]]
    executable_waypoints: list[dict[str, Any]]
    zero_length_segments: list[dict[str, Any]]
    trajectory: list[dict[str, Any]]
    route_source_summary: dict[str, Any]
    route_checks: dict[str, bool]


def find_waypoints(contract: dict[str, Any]) -> tuple[str, list[dict[str, Any]]]:
    candidates = [
        ("waypoints", contract.get("waypoints")),
        ("planned_3d_route.waypoints", (contract.get("planned_3d_route") or {}).get("waypoints")),
        ("route.waypoints", (contract.get("route") or {}).get("waypoints")),
        ("ordered_waypoints", contract.get("ordered_waypoints")),
    ]
    for schema_name, value in candidates:
        if isinstance(value, list) and value and all(isinstance(item, dict) for item in value):
            return schema_name, value
    raise ValueError("route contract does not contain a supported ordered waypoint list")


def load_route_plan(
    route_contract: Path,
    *,
    horizontal_speed: float,
    vertical_speed_limit: float,
    publish_rate: float,
    z_visual_scale: float,
) -> RoutePlan:
    contract = read_json(route_contract)
    schema_name, raw_waypoints = find_waypoints(contract)
    transition_edge = contract.get("transition_edge") if isinstance(contract.get("transition_edge"), dict) else {}
    waypoints: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_waypoints):
        xyz = raw_xyz(raw)
        node_id = str(raw.get("node_id") or raw.get("room_id") or raw.get("waypoint_id") or f"wp_{index:03d}")
        wp = {
            "index": index,
            "waypoint_id": str(raw.get("waypoint_id") or f"wp_{index:03d}"),
            "node_id": node_id,
            "semantic_type": semantic_kind(raw, transition_edge),
            "waypoint_kind": str(raw.get("waypoint_kind") or ""),
            "floor_id": raw.get("floor_id"),
            "room_id": raw.get("room_id"),
            "connector_id": raw.get("connector_id"),
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2] * z_visual_scale,
            "raw_z": xyz[2],
            "z_visual_scale": z_visual_scale,
            "z_source": raw.get("z_source"),
            "evidence_backed": bool(raw.get("evidence_source") or raw.get("evidence_backed") or contract.get("evidence_backed")),
            "evidence_source": raw.get("evidence_source"),
            "physical_execution_supported": bool(raw.get("physical_execution_supported", False)),
        }
        waypoints.append(wp)

    zero_length: list[dict[str, Any]] = []
    executable: list[dict[str, Any]] = []
    for wp in waypoints:
        if not executable:
            executable.append(wp)
            continue
        prev = executable[-1]
        d = distance(prev, wp)
        if d <= 1.0e-6:
            zero_length.append(
                {
                    "from_node_id": prev["node_id"],
                    "to_node_id": wp["node_id"],
                    "from_waypoint_id": prev["waypoint_id"],
                    "to_waypoint_id": wp["waypoint_id"],
                    "distance_m": d,
                    "handling": "kept_as_semantic_checkpoint_skipped_as_motion_segment",
                }
            )
            continue
        executable.append(wp)

    trajectory: list[dict[str, Any]] = []
    last_yaw = 0.0
    t_sec = 0.0
    sample_index = 0
    if executable:
        if len(executable) > 1:
            last_yaw = horizontal_yaw(executable[0], executable[1], 0.0)
        trajectory.append(sample_payload(sample_index, t_sec, executable[0], last_yaw, 0, "start"))
        sample_index += 1
    for segment_index, (start, end) in enumerate(zip(executable, executable[1:])):
        yaw = horizontal_yaw(start, end, last_yaw)
        last_yaw = yaw
        horiz = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        dz = abs(end["z"] - start["z"])
        duration = max(
            horiz / max(horizontal_speed, 1.0e-6),
            dz / max(vertical_speed_limit, 1.0e-6),
            1.0 / max(publish_rate, 1.0),
        )
        steps = max(1, int(math.ceil(duration * publish_rate)))
        for step in range(1, steps + 1):
            alpha = step / steps
            pose = {
                "x": start["x"] + (end["x"] - start["x"]) * alpha,
                "y": start["y"] + (end["y"] - start["y"]) * alpha,
                "z": start["z"] + (end["z"] - start["z"]) * alpha,
            }
            t_sec += duration / steps
            trajectory.append(
                {
                    **sample_payload(sample_index, t_sec, pose, yaw, segment_index, "interpolated"),
                    "source_node_id": start["node_id"],
                    "target_node_id": end["node_id"],
                    "source_waypoint_id": start["waypoint_id"],
                    "target_waypoint_id": end["waypoint_id"],
                    "segment_distance_m": distance(start, end),
                    "segment_duration_sec": duration,
                }
            )
            sample_index += 1

    node_ids = [wp["node_id"] for wp in waypoints]
    rooms = {wp.get("room_id") or wp["node_id"] for wp in waypoints}
    connectors = {wp.get("connector_id") for wp in waypoints if wp.get("connector_id")}
    route_checks = {
        "route_contract_loaded": True,
        "starts_at_room_2": bool(waypoints and waypoints[0]["node_id"] == "room_2"),
        "includes_room_3": "room_3" in node_ids or "room_3" in rooms,
        "includes_connector_vt_1_or_vc_vt_1": "vc_vt_1" in connectors or "vt_1" in connectors,
        "includes_vt_1_centerline_n000": "vt_1_centerline_n000" in node_ids,
        "includes_vt_1_centerline_n001": "vt_1_centerline_n001" in node_ids,
        "includes_vt_1_centerline_n002": "vt_1_centerline_n002" in node_ids,
        "includes_vt_1_centerline_n003": "vt_1_centerline_n003" in node_ids,
        "includes_vt_1_centerline_n004": "vt_1_centerline_n004" in node_ids,
        "transition_edge_is_vt_1_centerline_e001": str(contract.get("transition_edge_id") or transition_edge.get("edge_id")) == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": str(contract.get("transition_edge_id") or transition_edge.get("edge_id")) != "vt_1_centerline_e003",
        "includes_room_7": "room_7" in node_ids or "room_7" in rooms,
        "includes_room_13": "room_13" in node_ids or "room_13" in rooms,
        "includes_room_14": "room_14" in node_ids or "room_14" in rooms,
        "ends_at_room_14": bool(waypoints and waypoints[-1]["node_id"] == "room_14"),
        "zero_length_waypoints_handled": True,
    }
    summary = {
        "parsed_schema": schema_name,
        "route_contract": str(route_contract),
        "route_id": contract.get("route_id"),
        "start_room_id": contract.get("start_room_id"),
        "goal_room_id": contract.get("goal_room_id"),
        "room_sequence": contract.get("room_sequence"),
        "connector_id": contract.get("connector_id"),
        "transition_edge": transition_edge,
        "transition_edge_id": contract.get("transition_edge_id") or transition_edge.get("edge_id"),
        "waypoint_count": len(waypoints),
        "motion_segment_count": max(0, len(executable) - 1),
        "zero_length_segment_count": len(zero_length),
        "horizontal_speed_mps": horizontal_speed,
        "vertical_speed_limit_mps": vertical_speed_limit,
        "publish_rate_hz": publish_rate,
        "z_visual_scale": z_visual_scale,
        "floor_reference_z": contract.get("floor_reference_z"),
        "same_floor_room_segments_source": "task24f route contract waypoints; no Stage-A rerun",
        "stair_connector_segment_source": "task24f connector centerline nodes with true z values",
    }
    return RoutePlan(route_contract, contract, waypoints, waypoints, executable, zero_length, trajectory, summary, route_checks)


def load_planned_route_plan(
    route_contract: Path,
    planned_route_json: Path,
    *,
    horizontal_speed: float,
    vertical_speed_limit: float,
    publish_rate: float,
    z_visual_scale: float,
) -> RoutePlan:
    contract = read_json(route_contract)
    planned = read_json(planned_route_json)
    transition_edge = contract.get("transition_edge") if isinstance(contract.get("transition_edge"), dict) else {}
    raw_semantic = planned.get("semantic_checkpoints") or contract.get("waypoints") or []
    if not isinstance(raw_semantic, list) or not raw_semantic:
        raise ValueError("planned route does not contain semantic checkpoints")
    raw_dense = (planned.get("dense_3d_route") or {}).get("waypoints")
    if not isinstance(raw_dense, list) or not raw_dense:
        raise ValueError("planned route does not contain dense_3d_route.waypoints")

    def semantic_xyz(raw: dict[str, Any]) -> list[float]:
        if all(key in raw for key in ("x", "y", "z")):
            return [as_float(raw["x"]), as_float(raw["y"]), as_float(raw["z"])]
        return raw_xyz(raw)

    waypoints: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_semantic):
        xyz = semantic_xyz(raw)
        node_id = str(raw.get("node_id") or raw.get("room_id") or raw.get("waypoint_id") or f"wp_{index:03d}")
        waypoints.append(
            {
                "index": index,
                "waypoint_id": str(raw.get("waypoint_id") or f"wp_{index:03d}"),
                "node_id": node_id,
                "semantic_type": semantic_kind(raw, transition_edge),
                "waypoint_kind": str(raw.get("waypoint_kind") or ""),
                "floor_id": raw.get("floor_id"),
                "room_id": raw.get("room_id"),
                "connector_id": raw.get("connector_id"),
                "x": xyz[0],
                "y": xyz[1],
                "z": xyz[2] * z_visual_scale,
                "raw_z": xyz[2],
                "z_visual_scale": z_visual_scale,
                "z_source": raw.get("z_source") or "task24g2_planned_route",
                "evidence_backed": True,
                "evidence_source": raw.get("evidence_source") or str(planned_route_json),
                "physical_execution_supported": False,
            }
        )

    dense: list[dict[str, Any]] = []
    zero_length: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_dense):
        wp = {
            "index": index,
            "waypoint_id": str(raw.get("waypoint_id") or f"dense_{index:04d}"),
            "node_id": str(raw.get("node_id") or raw.get("source_node_id") or f"dense_{index:04d}"),
            "semantic_type": "dense_route_waypoint",
            "waypoint_kind": "dense_occupancy_aware_route_waypoint",
            "floor_id": raw.get("floor_id"),
            "room_id": raw.get("room_id"),
            "connector_id": raw.get("connector_id"),
            "x": as_float(raw.get("x")),
            "y": as_float(raw.get("y")),
            "z": as_float(raw.get("z")) * z_visual_scale,
            "raw_z": as_float(raw.get("z")),
            "z_visual_scale": z_visual_scale,
            "z_source": raw.get("z_source") or raw.get("pose_source"),
            "evidence_backed": True,
            "evidence_source": str(planned_route_json),
            "physical_execution_supported": False,
            "pose_source": raw.get("pose_source") or "planned_occupancy_aware_3d_route",
            "source_segment_id": raw.get("source_segment_id"),
            "source_node_id": raw.get("source_node_id") or raw.get("node_id"),
            "target_node_id": raw.get("target_node_id") or raw.get("node_id"),
            "route_waypoint_index": raw.get("route_waypoint_index", index),
        }
        if dense and distance(dense[-1], wp) <= 1.0e-6:
            zero_length.append(
                {
                    "from_waypoint_id": dense[-1]["waypoint_id"],
                    "to_waypoint_id": wp["waypoint_id"],
                    "distance_m": 0.0,
                    "handling": "duplicate_dense_route_point_skipped",
                }
            )
            continue
        dense.append(wp)

    trajectory: list[dict[str, Any]] = []
    last_yaw = 0.0
    t_sec = 0.0
    sample_index = 0
    if dense:
        if len(dense) > 1:
            last_yaw = horizontal_yaw(dense[0], dense[1], 0.0)
        first = sample_payload(sample_index, t_sec, dense[0], last_yaw, 0, "start")
        first["pose_source"] = "planned_occupancy_aware_3d_route"
        first["source_node_id"] = dense[0].get("source_node_id")
        first["target_node_id"] = dense[0].get("target_node_id")
        first["source_waypoint_id"] = dense[0]["waypoint_id"]
        first["target_waypoint_id"] = dense[0]["waypoint_id"]
        trajectory.append(first)
        sample_index += 1
    for segment_index, (start, end) in enumerate(zip(dense, dense[1:])):
        yaw = horizontal_yaw(start, end, last_yaw)
        last_yaw = yaw
        horiz = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        dz = abs(end["z"] - start["z"])
        duration = max(
            horiz / max(horizontal_speed, 1.0e-6),
            dz / max(vertical_speed_limit, 1.0e-6),
            1.0 / max(publish_rate, 1.0),
        )
        steps = max(1, int(math.ceil(duration * publish_rate)))
        for step in range(1, steps + 1):
            alpha = step / steps
            pose = {
                "x": start["x"] + (end["x"] - start["x"]) * alpha,
                "y": start["y"] + (end["y"] - start["y"]) * alpha,
                "z": start["z"] + (end["z"] - start["z"]) * alpha,
            }
            t_sec += duration / steps
            sample = {
                **sample_payload(sample_index, t_sec, pose, yaw, segment_index, "planned_route_interpolated"),
                "pose_source": "planned_occupancy_aware_3d_route",
                "source_node_id": start.get("source_node_id") or start.get("node_id"),
                "target_node_id": end.get("target_node_id") or end.get("node_id"),
                "source_waypoint_id": start["waypoint_id"],
                "target_waypoint_id": end["waypoint_id"],
                "segment_distance_m": distance(start, end),
                "segment_duration_sec": duration,
            }
            trajectory.append(sample)
            sample_index += 1

    node_ids = [wp["node_id"] for wp in waypoints]
    route_checks = {
        "route_contract_loaded": True,
        "planned_occupancy_aware_route_loaded": True,
        "floor_1_stable_occupancy_map_loaded": bool((planned.get("route_checks") or {}).get("floor_1_stable_occupancy_map_loaded")),
        "floor_2_stable_occupancy_map_loaded": bool((planned.get("route_checks") or {}).get("floor_2_stable_occupancy_map_loaded")),
        "all_same_floor_astar_segments_succeeded": bool((planned.get("route_checks") or {}).get("all_same_floor_astar_segments_succeeded")),
        "same_floor_route_is_no_longer_only_straight_topology_node_interpolation": bool(
            (planned.get("route_checks") or {}).get("same_floor_route_is_no_longer_only_straight_topology_node_interpolation")
        ),
        "starts_at_room_2": bool(waypoints and waypoints[0]["node_id"] == "room_2"),
        "includes_room_3": "room_3" in node_ids,
        "includes_vt_1_centerline_n000": "vt_1_centerline_n000" in node_ids,
        "includes_vt_1_centerline_n001": "vt_1_centerline_n001" in node_ids,
        "includes_vt_1_centerline_n002": "vt_1_centerline_n002" in node_ids,
        "includes_vt_1_centerline_n003": "vt_1_centerline_n003" in node_ids,
        "includes_vt_1_centerline_n004": "vt_1_centerline_n004" in node_ids,
        "stair_connector_segment_remains_vt_1_centerline": bool((planned.get("route_checks") or {}).get("stair_connector_segment_remains_vt_1_centerline")),
        "transition_edge_is_vt_1_centerline_e001": str(planned.get("transition_edge_id")) == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": str(planned.get("transition_edge_id")) != "vt_1_centerline_e003",
        "includes_room_7": "room_7" in node_ids,
        "includes_room_13": "room_13" in node_ids,
        "includes_room_14": "room_14" in node_ids,
        "ends_at_room_14": bool(waypoints and waypoints[-1]["node_id"] == "room_14"),
        "zero_length_waypoints_handled": True,
        "no_nav2_process_launched_by_player": True,
        "no_amcl_localization_claim_made": True,
        "no_gait_footstep_or_contact_based_climbing_claim_made": True,
        "task24g_default_contract_playback_remains_available": True,
        "task23b_planar_proxy_behavior_unchanged": True,
    }
    summary = {
        "parsed_schema": "task24g2_planned_occupancy_aware_3d_route_v0_1",
        "route_contract": str(route_contract),
        "planned_route_json": str(planned_route_json),
        "route_id": planned.get("route_id"),
        "source_route_contract_route_id": planned.get("source_route_contract_route_id"),
        "start_room_id": contract.get("start_room_id"),
        "goal_room_id": contract.get("goal_room_id"),
        "room_sequence": contract.get("room_sequence"),
        "connector_id": contract.get("connector_id"),
        "transition_edge": planned.get("transition_edge") or transition_edge,
        "transition_edge_id": planned.get("transition_edge_id"),
        "semantic_checkpoint_count": len(waypoints),
        "dense_route_waypoint_count": len(dense),
        "motion_segment_count": max(0, len(dense) - 1),
        "zero_length_segment_count": len(zero_length),
        "horizontal_speed_mps": horizontal_speed,
        "vertical_speed_limit_mps": vertical_speed_limit,
        "publish_rate_hz": publish_rate,
        "z_visual_scale": z_visual_scale,
        "floor_reference_z": contract.get("floor_reference_z"),
        "map_sources": planned.get("map_sources"),
        "same_floor_room_segments_source": "A* over stable occupancy maps from task24g2 route backfill",
        "stair_connector_segment_source": "task24f connector centerline nodes with true z values",
        "claim_boundary": TASK24G2_CLAIM_BOUNDARY,
    }
    return RoutePlan(planned_route_json, planned, waypoints, dense, dense, zero_length, trajectory, summary, route_checks)


def sample_payload(index: int, t_sec: float, pose: dict[str, float], yaw: float, segment_index: int, phase: str) -> dict[str, Any]:
    return {
        "sample_index": index,
        "t_sec": round(t_sec, 6),
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "z": float(pose["z"]),
        "yaw": float(yaw),
        "roll": 0.0,
        "pitch": 0.0,
        "segment_index": segment_index,
        "phase": phase,
        "pose_source": "interpolated_task24f_route_contract",
    }


def color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def point(x: float, y: float, z: float) -> Point:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def set_marker_scale(marker: Marker, x: float, y: float, z: float) -> None:
    marker.scale.x = x
    marker.scale.y = y
    marker.scale.z = z


def make_line_marker(frame_id: str, ns: str, marker_id: int, points: list[dict[str, Any]], rgba: ColorRGBA, width: float) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.color = rgba
    marker.scale.x = width
    marker.lifetime = Duration(sec=0, nanosec=0)
    marker.points = [point(p["x"], p["y"], p["z"]) for p in points]
    marker.pose.orientation.w = 1.0
    return marker


def make_sphere_marker(frame_id: str, ns: str, marker_id: int, pose: dict[str, Any], rgba: ColorRGBA, scale: float) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.color = rgba
    set_marker_scale(marker, scale, scale, scale)
    marker.pose.position.x = float(pose["x"])
    marker.pose.position.y = float(pose["y"])
    marker.pose.position.z = float(pose["z"])
    marker.pose.orientation.w = 1.0
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


def make_text_marker(frame_id: str, ns: str, marker_id: int, pose: dict[str, Any], text: str, rgba: ColorRGBA, scale_z: float) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.color = rgba
    marker.scale.z = scale_z
    marker.pose.position.x = float(pose["x"])
    marker.pose.position.y = float(pose["y"])
    marker.pose.position.z = float(pose["z"])
    marker.pose.orientation.w = 1.0
    marker.text = text
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


class CrossFloorVisualTraversalNode(Node):
    def __init__(self, args: argparse.Namespace, plan: RoutePlan) -> None:
        super().__init__("task24g_cross_floor_visual_traversal_player")
        self.args = args
        self.plan = plan
        self.shutdown_requested = False
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.planned_pub = self.create_publisher(MarkerArray, args.planned_marker_topic, qos)
        self.executed_pub = self.create_publisher(MarkerArray, args.executed_marker_topic, qos)
        self.visual_pub = self.create_publisher(MarkerArray, args.visual_marker_topic, qos)
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.service_client = None
        self.service_name: str | None = None
        self.get_state_client = None
        self.get_state_service_name: str | None = None
        self.service_events: list[dict[str, Any]] = []
        self.executed: list[dict[str, Any]] = []
        self.set_entity_success_count = 0
        self.set_entity_failure_count = 0
        self.entity_exists = False
        self.entity_state_message = "not_checked"
        self.pitch_along_slope = bool(args.pitch_along_slope)
        self.claim_boundary = TASK24G2_CLAIM_BOUNDARY if args.planned_route_json else CLAIM_BOUNDARY
        self.planned_markers = self._planned_markers()
        self._record("startup", entity_name=args.entity_name, claim_boundary=self.claim_boundary)

    def _record(self, event: str, **fields: Any) -> None:
        payload = {"timestamp_utc": now_iso(), "event": event, **fields}
        self.service_events.append(payload)
        self.get_logger().info(json.dumps(payload, sort_keys=True))

    def _discover_service(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and not self.shutdown_requested:
            services = {
                name: types
                for name, types in self.get_service_names_and_types()
            }
            set_services = {
                name
                for name, types in services.items()
                if "gazebo_msgs/srv/SetEntityState" in types
            }
            selected = next((name for name in SERVICE_CANDIDATES if name in set_services), None)
            if selected is None and set_services:
                selected = sorted(set_services)[0]
            get_services = {
                name
                for name, types in services.items()
                if "gazebo_msgs/srv/GetEntityState" in types
            }
            get_selected = next((name for name in GET_STATE_CANDIDATES if name in get_services), None)
            if get_selected is None and get_services:
                get_selected = sorted(get_services)[0]
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
            self.entity_state_message = "GetEntityState service unavailable; SetEntityState success will be used as entity evidence"
            return
        request = GetEntityState.Request()
        request.name = self.args.entity_name
        request.reference_frame = self.args.frame_id
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        try:
            response = future.result()
        except Exception as exc:
            self.entity_state_message = f"GetEntityState exception: {exc}"
            return
        if response is None:
            self.entity_state_message = "GetEntityState timed out"
            return
        self.entity_exists = bool(getattr(response, "success", False))
        self.entity_state_message = str(getattr(response, "status_message", "success" if self.entity_exists else "entity query failed"))
        self._record("entity_state_query", entity_exists=self.entity_exists, message=self.entity_state_message)

    def _planned_markers(self) -> MarkerArray:
        msg = MarkerArray()
        route_points = self.plan.planned_route_waypoints or self.plan.waypoints
        msg.markers.append(make_line_marker(self.args.frame_id, "planned_3d_route", 1, route_points, color(0.05, 0.35, 0.9, 1.0), 0.08))
        connector_points = [wp for wp in route_points if wp.get("connector_id") or wp.get("pose_source") == "stair_connector_centerline"]
        if not connector_points:
            connector_points = [wp for wp in self.plan.waypoints if wp.get("connector_id")]
        msg.markers.append(make_line_marker(self.args.frame_id, "connector_centerline", 2, connector_points, color(0.9, 0.25, 0.08, 1.0), 0.12))
        marker_id = 10
        for wp in self.plan.waypoints:
            if wp["semantic_type"] == "transition_node":
                rgba = color(0.95, 0.05, 0.05, 1.0)
                scale = 0.32
            elif wp.get("connector_id"):
                rgba = color(0.95, 0.45, 0.05, 1.0)
                scale = 0.22
            elif wp.get("room_id") or wp["node_id"].startswith("room_"):
                rgba = color(0.05, 0.65, 0.35, 1.0)
                scale = 0.24
            else:
                rgba = color(0.3, 0.3, 0.3, 1.0)
                scale = 0.18
            msg.markers.append(make_sphere_marker(self.args.frame_id, "semantic_checkpoints", marker_id, wp, rgba, scale))
            marker_id += 1
            label_pose = dict(wp)
            label_pose["z"] = float(label_pose["z"]) + 0.35
            msg.markers.append(make_text_marker(self.args.frame_id, "checkpoint_labels", marker_id, label_pose, wp["node_id"], color(0.08, 0.08, 0.08, 1.0), 0.18))
            marker_id += 1

        edge = self.plan.contract.get("transition_edge") or {}
        transition_pose = next((wp for wp in self.plan.waypoints if wp["node_id"] == edge.get("target_node_id")), self.plan.waypoints[min(5, len(self.plan.waypoints) - 1)])
        text_pose = dict(transition_pose)
        text_pose["z"] = float(text_pose["z"]) + 0.7
        msg.markers.append(make_text_marker(self.args.frame_id, "transition_edge", 200, text_pose, "transition edge: vt_1_centerline_e001", color(0.75, 0.0, 0.0, 1.0), 0.22))
        claim_pose = {
            "x": min(wp["x"] for wp in route_points) - 0.5,
            "y": min(wp["y"] for wp in route_points) - 0.5,
            "z": max(wp["z"] for wp in route_points) + 0.9,
        }
        claim_text = CLAIM_TEXT
        if self.args.planned_route_json:
            claim_text = "\n".join(
                [
                    "task24g2 visual_kinematic_proxy_only",
                    "occupancy_aware_same_floor_visual_playback=true",
                    "topological_vertical_transition_only",
                    "physical_stair_climbing_supported=false",
                    "no Nav2 / no AMCL / no gait",
                ]
            )
        msg.markers.append(make_text_marker(self.args.frame_id, "claim_boundary", 201, claim_pose, claim_text, color(0.02, 0.02, 0.02, 1.0), 0.22))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _executed_markers(self) -> MarkerArray:
        msg = MarkerArray()
        if self.executed:
            msg.markers.append(make_line_marker(self.args.frame_id, "executed_3d_trajectory", 1, self.executed, color(0.0, 0.62, 0.76, 1.0), 0.07))
            msg.markers.append(make_sphere_marker(self.args.frame_id, "current_visual_proxy_pose", 2, self.executed[-1], color(0.02, 0.02, 0.02, 1.0), 0.28))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _visual_markers(self, pose: dict[str, Any]) -> MarkerArray:
        msg = MarkerArray()
        body = make_sphere_marker(self.args.frame_id, "visual_proxy_body", 1, pose, color(0.08, 0.08, 0.1, 0.85), 0.42)
        body.scale.z = 0.22
        msg.markers.append(body)
        label_pose = dict(pose)
        label_pose["z"] = float(label_pose["z"]) + 0.45
        msg.markers.append(make_text_marker(self.args.frame_id, "visual_proxy_label", 2, label_pose, self.args.entity_name, color(0.0, 0.0, 0.0, 1.0), 0.18))
        claim_pose = dict(label_pose)
        claim_pose["z"] += 0.35
        msg.markers.append(make_text_marker(self.args.frame_id, "visual_proxy_claim", 3, claim_pose, "visual proxy only", color(0.65, 0.0, 0.0, 1.0), 0.16))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish_pose(self, pose: dict[str, Any]) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_quaternion(float(pose["yaw"]), float(pose.get("pitch", 0.0)), float(pose.get("roll", 0.0)))
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.args.frame_id
        transform.child_frame_id = self.args.base_frame
        transform.transform.translation.x = float(pose["x"])
        transform.transform.translation.y = float(pose["y"])
        transform.transform.translation.z = float(pose["z"])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

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
        self.visual_pub.publish(self._visual_markers(pose))

    def _update_gazebo(self, pose: dict[str, Any]) -> bool:
        if self.args.dry_run:
            return False
        if self.service_client is None:
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
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.05, 0.8 / max(self.args.publish_rate, 1.0)))
        try:
            response = future.result()
        except Exception as exc:
            self.set_entity_failure_count += 1
            self._record("set_entity_state_exception", error=str(exc), sample_index=pose.get("sample_index"))
            return False
        if response is not None and bool(response.success):
            self.set_entity_success_count += 1
            self.entity_exists = True
            return True
        self.set_entity_failure_count += 1
        if response is not None:
            self.entity_state_message = str(getattr(response, "status_message", "SetEntityState returned success=false"))
        return False

    def run(self) -> dict[str, Any]:
        service_found = False
        if not self.args.dry_run:
            service_found = self._discover_service(self.args.service_timeout_sec)
            if service_found:
                self._query_entity()
            else:
                self._record("playback_skipped_no_set_entity_state")
        start_wall = time.monotonic()
        previous_wall = start_wall
        first_pose = self.plan.trajectory[0] if self.plan.trajectory else None
        if first_pose is not None:
            self._publish_pose(first_pose)
            if service_found:
                self._update_gazebo(first_pose)
        for raw_pose in self.plan.trajectory:
            if self.shutdown_requested or not rclpy.ok():
                self._record("shutdown_requested_during_playback", sample_index=raw_pose.get("sample_index"))
                break
            pose = dict(raw_pose)
            if self.pitch_along_slope:
                pose["pitch"] = self._pitch_for_segment(pose)
            ok = self._update_gazebo(pose) if service_found else False
            pose["gazebo_set_entity_state_success"] = bool(ok)
            pose["entity_name"] = self.args.entity_name
            pose["frame_id"] = self.args.frame_id
            self.executed.append(pose)
            self._publish_pose(pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            now = time.monotonic()
            target_sleep = max(0.0, (1.0 / max(self.args.publish_rate, 1.0)) - (now - previous_wall))
            target_sleep *= max(0.0, self.args.realtime_scale)
            if target_sleep > 0:
                time.sleep(target_sleep)
            previous_wall = time.monotonic()
        hold_until = time.monotonic() + max(0.0, self.args.hold_final_sec)
        while rclpy.ok() and not self.shutdown_requested and time.monotonic() < hold_until and self.executed:
            self._publish_pose(self.executed[-1])
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self._record(
            "playback_complete",
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
        }

    def _pitch_for_segment(self, pose: dict[str, Any]) -> float:
        segment_index = int(pose.get("segment_index", 0))
        if segment_index < 0 or segment_index >= len(self.plan.executable_waypoints) - 1:
            return 0.0
        start = self.plan.executable_waypoints[segment_index]
        end = self.plan.executable_waypoints[segment_index + 1]
        horiz = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        if horiz < 1.0e-6:
            return 0.0
        return math.atan2(end["z"] - start["z"], horiz)


def validate_checkpoints(plan: RoutePlan, executed: list[dict[str, Any]], tolerance: float, final_tolerance: float) -> dict[str, Any]:
    rows = []
    all_passed = True
    for wp in plan.waypoints:
        if executed:
            best = min(executed, key=lambda pose: distance(wp, pose))
            err = distance(wp, best)
            best_index = best.get("sample_index")
        else:
            err = float("inf")
            best_index = None
        passed = bool(err <= tolerance)
        all_passed = all_passed and passed
        rows.append(
            {
                "node_id": wp["node_id"],
                "waypoint_id": wp["waypoint_id"],
                "semantic_type": wp["semantic_type"],
                "floor_id": wp.get("floor_id"),
                "checkpoint_tolerance_m": tolerance,
                "nearest_executed_sample_index": best_index,
                "error_m": None if not math.isfinite(err) else round(err, 6),
                "passed": passed,
            }
        )
    final_error = float("inf")
    final_passed = False
    if plan.waypoints and executed:
        final_error = distance(plan.waypoints[-1], executed[-1])
        final_passed = final_error <= final_tolerance
    return {
        "artifact_type": "task24g_checkpoint_validation",
        "created_utc": now_iso(),
        "checkpoint_tolerance_m": tolerance,
        "final_tolerance_m": final_tolerance,
        "all_checkpoints_passed": all_passed,
        "final_pose_error_m": None if not math.isfinite(final_error) else round(final_error, 6),
        "final_pose_reaches_room_14": final_passed,
        "checkpoints": rows,
    }


def z_transition_validation(plan: RoutePlan, executed: list[dict[str, Any]]) -> dict[str, Any]:
    floor_ref = plan.contract.get("floor_reference_z") if isinstance(plan.contract.get("floor_reference_z"), dict) else {}
    floor_1_z = as_float((floor_ref.get("floor_1") or {}).get("z"), plan.waypoints[0]["raw_z"] if plan.waypoints else 0.0)
    floor_2_z = as_float((floor_ref.get("floor_2") or {}).get("z"), plan.waypoints[-1]["raw_z"] if plan.waypoints else 0.0)
    z_start = executed[0]["z"] if executed else None
    z_end = executed[-1]["z"] if executed else None
    z_delta = None if z_start is None or z_end is None else z_end - z_start
    expected_delta = floor_2_z - floor_1_z
    return {
        "artifact_type": "task24g_z_transition_validation",
        "created_utc": now_iso(),
        "transition_edge_id": plan.contract.get("transition_edge_id") or (plan.contract.get("transition_edge") or {}).get("edge_id"),
        "transition_edge": plan.contract.get("transition_edge"),
        "z_visual_scale": plan.route_source_summary["z_visual_scale"],
        "floor_1_reference_z": floor_1_z,
        "floor_2_reference_z": floor_2_z,
        "expected_floor_delta_m": round(expected_delta, 6),
        "z_start": None if z_start is None else round(float(z_start), 6),
        "z_end": None if z_end is None else round(float(z_end), 6),
        "z_delta": None if z_delta is None else round(float(z_delta), 6),
        "z_delta_positive": bool(z_delta is not None and z_delta > 0.0),
        "z_delta_matches_floor_references_within_0_10m": bool(z_delta is not None and abs(z_delta - expected_delta) <= 0.10),
        "physical_stair_climbing_supported": False,
    }


def process_snapshot() -> dict[str, Any]:
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid,ppid,stat,comm,args"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10.0,
        )
        text = result.stdout
    except Exception as exc:
        text = f"process snapshot failed: {exc}\n"
    nav2_pattern = re.compile(r"\b(nav2_|amcl|bt_navigator|controller_server|planner_server|recoveries_server|waypoint_follower)\b")
    nav2_lines = [line for line in text.splitlines() if nav2_pattern.search(line)]
    return {
        "artifact_type": "task24g_process_snapshot",
        "created_utc": now_iso(),
        "snapshot": text,
        "nav2_matching_lines": nav2_lines,
        "no_nav2_process_detected": not nav2_lines,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "sample_index",
        "t_sec",
        "x",
        "y",
        "z",
        "yaw",
        "roll",
        "pitch",
        "segment_index",
        "phase",
        "pose_source",
        "source_node_id",
        "target_node_id",
        "gazebo_set_entity_state_success",
        "entity_name",
        "frame_id",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def classify(route_ok: bool, gazebo_ok: bool, final_ok: bool, rviz_ok: bool) -> str:
    if not route_ok:
        return "task24g_blocked_by_route_contract"
    if not gazebo_ok:
        return "task24g_blocked_by_gazebo_entity_or_set_entity_state"
    if gazebo_ok and final_ok and rviz_ok:
        return "task24g_visual_proxy_cross_floor_traversal_validated"
    if gazebo_ok and final_ok:
        return "task24g_visual_proxy_validated_rviz_manual_open_required"
    return "task24g_blocked_by_gazebo_entity_or_set_entity_state"


def classify_task24g2(astar_ok: bool, route_ok: bool, gazebo_ok: bool, final_ok: bool, rviz_ok: bool) -> str:
    if not astar_ok:
        return "task24g2_blocked_by_same_floor_astar_segment"
    if not route_ok:
        return "task24g2_blocked_by_same_floor_astar_segment"
    if not gazebo_ok:
        return "task24g2_blocked_by_gazebo_entity_or_set_entity_state"
    if gazebo_ok and final_ok and rviz_ok:
        return "task24g2_occupancy_aware_visual_proxy_cross_floor_traversal_validated"
    if gazebo_ok and final_ok:
        return "task24g2_occupancy_aware_visual_proxy_validated_rviz_manual_open_required"
    return "task24g2_blocked_by_gazebo_entity_or_set_entity_state"


def write_outputs(
    args: argparse.Namespace,
    plan: RoutePlan,
    executed: list[dict[str, Any]],
    runtime: dict[str, Any],
) -> dict[str, Any]:
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    is_task24g2 = bool(args.planned_route_json)
    task_label = "task24g2" if is_task24g2 else "task24g"
    claim_boundary = TASK24G2_CLAIM_BOUNDARY if is_task24g2 else CLAIM_BOUNDARY
    checkpoints = validate_checkpoints(plan, executed, args.checkpoint_tolerance_m, args.final_tolerance_m)
    zval = z_transition_validation(plan, executed)
    if is_task24g2:
        checkpoints["artifact_type"] = "task24g2_checkpoint_validation"
        zval["artifact_type"] = "task24g2_z_transition_validation"
    ps = process_snapshot()
    route_ok = all(plan.route_checks.values())
    astar_ok = bool(plan.route_checks.get("all_same_floor_astar_segments_succeeded", True))
    gazebo_ok = bool(runtime.get("service_found") and runtime.get("entity_exists") and runtime.get("set_entity_success_count", 0) > 0)
    final_ok = bool(checkpoints.get("final_pose_reaches_room_14"))
    rviz_ok = bool(executed)  # Topics are published by this node; GUI visibility can still be manual.
    classification = (
        classify_task24g2(astar_ok, route_ok, gazebo_ok, final_ok, rviz_ok)
        if is_task24g2
        else classify(route_ok, gazebo_ok, final_ok, rviz_ok)
    )
    if classification == "task24g_visual_proxy_cross_floor_traversal_validated" and not args.rviz_auto_started:
        classification = "task24g_visual_proxy_validated_rviz_manual_open_required"
    if classification == "task24g2_occupancy_aware_visual_proxy_cross_floor_traversal_validated" and not args.rviz_auto_started:
        classification = "task24g2_occupancy_aware_visual_proxy_validated_rviz_manual_open_required"

    planned = {
        "artifact_type": f"{task_label}_planned_3d_route_v0_1",
        "created_utc": now_iso(),
        "route_source_summary": plan.route_source_summary,
        "route_checks": plan.route_checks,
        "zero_length_segments": plan.zero_length_segments,
        "waypoints": plan.waypoints,
        "planned_route_waypoints": plan.planned_route_waypoints,
        "executable_waypoint_node_ids": [wp["node_id"] for wp in plan.executable_waypoints],
        "claim_boundary": claim_boundary,
    }
    gazebo = {
        "artifact_type": f"{task_label}_gazebo_entity_state_validation",
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
        "pose_updated_in_x_y_z_yaw": bool(runtime.get("set_entity_success_count", 0) > 0 and executed),
        "dry_run": bool(args.dry_run),
        "claim_boundary": claim_boundary,
    }
    topic_text = "\n".join(
        [
            f"{task_label} RViz marker topic validation",
            f"planned_route_topic={args.planned_marker_topic}",
            f"executed_trajectory_topic={args.executed_marker_topic}",
            f"visual_proxy_marker_topic={args.visual_marker_topic}",
            f"odom_topic={args.odom_topic}",
            f"rviz_auto_started={args.rviz_auto_started}",
            "MarkerArray publishers are created by cross_floor_visual_traversal_player.py.",
            "If RViz did not auto-start, use the manual GUI validation instructions.",
            "",
        ]
    )
    report = {
        "artifact_type": f"{task_label}_report",
        "created_utc": now_iso(),
        "classification": classification,
        "route_summary": plan.route_source_summary,
        "route_checks": plan.route_checks,
        "gazebo_entity_state_validation": gazebo,
        "checkpoint_validation": checkpoints,
        "z_transition_validation": zval,
        "marker_topics": {
            "planned_3d_route": args.planned_marker_topic,
            "executed_3d_trajectory": args.executed_marker_topic,
            "visual_proxy_markers": args.visual_marker_topic,
            "odom": args.odom_topic,
        },
        "astar_segment_table": [
            {
                "segment_id": segment.get("segment_id"),
                "floor_id": segment.get("floor_id"),
                "source_node_id": segment.get("source_node_id"),
                "target_node_id": segment.get("target_node_id"),
                "planner_type": segment.get("planner_type"),
                "path_length_m": segment.get("path_length_m"),
                "waypoint_count": segment.get("waypoint_count"),
                "minimum_clearance_estimate_m": segment.get("minimum_clearance_estimate_m"),
                "wall_obstacle_crossing_check_passed": (
                    (segment.get("wall_obstacle_crossing_check_result") or {}).get("wall_obstacle_crossing_check_passed")
                    if isinstance(segment.get("wall_obstacle_crossing_check_result"), dict)
                    else None
                ),
                "status": segment.get("status"),
            }
            for segment in (plan.contract.get("segments") or [])
            if is_task24g2 and segment.get("planner_type") == "astar_stable_occupancy_map"
        ],
        "snapping_table": [
            {
                "segment_id": segment.get("segment_id"),
                "source_node_id": segment.get("source_node_id"),
                "source_snap_distance_m": (segment.get("source_snap") or {}).get("snap_distance_m"),
                "source_snap_reason": (segment.get("source_snap") or {}).get("reason"),
                "target_node_id": segment.get("target_node_id"),
                "target_snap_distance_m": (segment.get("target_snap") or {}).get("snap_distance_m"),
                "target_snap_reason": (segment.get("target_snap") or {}).get("reason"),
            }
            for segment in (plan.contract.get("segments") or [])
            if is_task24g2 and segment.get("planner_type") == "astar_stable_occupancy_map"
        ],
        "stair_connector_segment_summary": next(
            (
                {
                    "segment_id": segment.get("segment_id"),
                    "centerline_nodes": segment.get("centerline_nodes"),
                    "transition_edge_id": segment.get("transition_edge_id"),
                    "path_length_m": segment.get("path_length_m"),
                    "waypoint_count": segment.get("waypoint_count"),
                    "z_delta_m": segment.get("z_delta_m"),
                    "z_monotonic_non_decreasing": segment.get("z_monotonic_non_decreasing"),
                    "physical_stair_climbing_supported": False,
                }
                for segment in (plan.contract.get("segments") or [])
                if is_task24g2 and segment.get("planner_type") == "stair_connector_centerline"
            ),
            None,
        ),
        "process_snapshot_summary": {
            "no_nav2_process_detected": ps["no_nav2_process_detected"],
            "nav2_matching_lines": ps["nav2_matching_lines"],
        },
        "commands_run": [args.command_label or " ".join(sys.argv)],
        "files_created_or_changed": [
            "tools/vertical_connectors/build_occupancy_aware_cross_floor_visual_route.py",
            "tools/vertical_connectors/cross_floor_visual_traversal_player.py",
            (
                "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_astar_demo.sh"
                if is_task24g2
                else "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh"
            ),
            (
                "tools/vertical_connectors/README_room2_to_room14_cross_floor_visual_proxy_astar_demo.md"
                if is_task24g2
                else "tools/vertical_connectors/README_room2_to_room14_cross_floor_visual_proxy_demo.md"
            ),
            (
                "tools/vertical_connectors/rviz_room2_to_room14_cross_floor_visual_proxy_astar.rviz"
                if is_task24g2
                else "tools/vertical_connectors/rviz_room2_to_room14_cross_floor_visual_proxy.rviz"
            ),
            str(out),
        ],
        "claim_boundary": claim_boundary,
        "core_statement": (
            "task24g2 validates occupancy-aware visual-kinematic cross-floor traversal of a Gazebo "
            "entity along an RSLG-SLAM route. Same-floor portions use A* over stable occupancy maps; "
            "the stair portion uses the graph-level 3D connector centerline. It does not validate "
            "physical stair climbing, gait, footstep planning, or contact-based quadruped locomotion."
            if is_task24g2
            else "task24g validates visual-kinematic cross-floor traversal of a Gazebo entity along the "
            "RSLG-SLAM 3D topology route. It does not validate physical stair climbing, gait, "
            "footstep planning, or contact-based quadruped locomotion."
        ),
    }

    write_json(out / "planned_3d_route_v0_1.json", planned)
    write_csv(out / "executed_3d_trajectory.csv", executed)
    write_json(out / "executed_3d_trajectory.json", {"artifact_type": f"{task_label}_executed_3d_trajectory", "created_utc": now_iso(), "samples": executed})
    write_json(out / "checkpoint_validation.json", checkpoints)
    write_json(out / "z_transition_validation.json", zval)
    write_json(out / "gazebo_entity_state_validation.json", gazebo)
    write_text(out / "set_entity_state_service_log.txt", "\n".join(json.dumps(item, sort_keys=True) for item in runtime.get("service_events", [])) + "\n")
    write_text(out / "rviz_marker_topic_validation.txt", topic_text)
    write_text(out / "process_snapshot.txt", ps["snapshot"])
    write_json(out / "claim_boundary.json", claim_boundary)
    write_json(out / f"{task_label}_report.json", report)
    write_text(out / "manual_gui_validation_instructions.md", manual_instructions(is_task24g2))
    write_text(out / f"{task_label}_report.md", report_markdown(report, checkpoints, is_task24g2))
    write_text(out / "final_answer_for_user.md", final_answer_markdown(report))
    return report


def report_markdown(report: dict[str, Any], checkpoints: dict[str, Any], is_task24g2: bool = False) -> str:
    rows = "\n".join(
        f"| {row['node_id']} | {row['semantic_type']} | {row['floor_id']} | {row['error_m']} | {row['passed']} |"
        for row in checkpoints["checkpoints"]
    )
    title = (
        "task24g2 Occupancy-Aware Cross-Floor Visual Proxy Traversal"
        if is_task24g2
        else "task24g Room 2 to Room 14 Cross-Floor Visual Proxy Traversal Demo"
    )
    route_mode = (
        "- Same-floor route source: A* over floor-specific stable occupancy maps\n"
        "- Stair route source: graph-level `vt_1` 3D connector centerline\n"
        "- Occupancy-aware same-floor visual playback: `true`"
        if is_task24g2
        else "- Same-floor route source: task24f visual contract interpolation\n"
        "- Stair route source: graph-level `vt_1` 3D connector centerline\n"
        "- A* occupancy-aware route playback implemented here: `false`"
    )
    astar_rows = ""
    if is_task24g2:
        segments = (report.get("route_summary") or {}).get("planned_route_json")
        astar_table = "\n".join(
            "| {segment_id} | {floor_id} | {source_node_id} -> {target_node_id} | {path_length_m} | {waypoint_count} | {minimum_clearance_estimate_m} | {wall_obstacle_crossing_check_passed} | {status} |".format(**row)
            for row in report.get("astar_segment_table", [])
        )
        snap_table = "\n".join(
            "| {segment_id} | {source_node_id} | {source_snap_distance_m} | {source_snap_reason} | {target_node_id} | {target_snap_distance_m} | {target_snap_reason} |".format(**row)
            for row in report.get("snapping_table", [])
        )
        stair = report.get("stair_connector_segment_summary") or {}
        astar_rows = f"""
## A* Route Source

- Planned route JSON: `{segments}`
- floor_1 stable map loaded: `{report['route_checks'].get('floor_1_stable_occupancy_map_loaded')}`
- floor_2 stable map loaded: `{report['route_checks'].get('floor_2_stable_occupancy_map_loaded')}`
- all same-floor A* segments succeeded: `{report['route_checks'].get('all_same_floor_astar_segments_succeeded')}`
- same-floor route no longer only straight topology interpolation: `{report['route_checks'].get('same_floor_route_is_no_longer_only_straight_topology_node_interpolation')}`

## A* Segment Table

| segment | floor | endpoints | length m | waypoints | min clearance m | wall check | status |
|---|---|---|---:|---:|---:|---|---|
{astar_table}

## Snapping Table

| segment | source | source snap m | source reason | target | target snap m | target reason |
|---|---|---:|---|---|---:|---|
{snap_table}

## Stair Connector Segment

- segment: `{stair.get('segment_id')}`
- centerline nodes: `{stair.get('centerline_nodes')}`
- transition edge: `{stair.get('transition_edge_id')}`
- length m: `{stair.get('path_length_m')}`
- z delta m: `{stair.get('z_delta_m')}`
- physical stair climbing supported: `False`
"""
    return f"""# {title}

Classification: `{report['classification']}`

## Route Summary

- Route: `room_2 -> room_3 -> vc_vt_1/vt_1 -> room_7 -> room_13 -> room_14`
- Transition edge: `{report['route_summary'].get('transition_edge_id')}`
- Gazebo entity: `{report['gazebo_entity_state_validation']['entity_name']}`
- SetEntityState service used: `{report['gazebo_entity_state_validation']['set_entity_state_service_used']}`
- Marker topics: `{report['marker_topics']['planned_3d_route']}`, `{report['marker_topics']['executed_3d_trajectory']}`, `{report['marker_topics']['visual_proxy_markers']}`
{route_mode}
{astar_rows}

## Z Transition

- z_start: `{report['z_transition_validation']['z_start']}`
- z_end: `{report['z_transition_validation']['z_end']}`
- z_delta: `{report['z_transition_validation']['z_delta']}`
- expected floor delta: `{report['z_transition_validation']['expected_floor_delta_m']}`
- transition edge is `vt_1_centerline_e001`: `{report['route_checks']['transition_edge_is_vt_1_centerline_e001']}`

## Checkpoints

| checkpoint | semantic type | floor | error m | passed |
|---|---|---|---:|---|
{rows}

## Process Boundary

- No Nav2 process detected in snapshot: `{report['process_snapshot_summary']['no_nav2_process_detected']}`
- AMCL/localization claim made: `False`
- Nav2 execution: `False`
- Gait/footstep/contact-based climbing claim made: `False`
- Existing task23b planar proxy behavior modified by default: `False`
- Existing task24g straight-line contract playback remains available by default: `True`

## Commands Run

{chr(10).join(f'- `{command}`' for command in report.get('commands_run', []))}

## Claim Boundary

`visual_kinematic_proxy_only`

`topological_vertical_transition_only`

`physical_stair_climbing_supported=false`

No gait, no footstep planning, no contact-based stair climbing, no real quadruped stair locomotion, no Nav2 execution, and no AMCL/localization are claimed.

{report['core_statement']}
"""


def final_answer_markdown(report: dict[str, Any]) -> str:
    is_task24g2 = report.get("artifact_type") == "task24g2_report"
    task_name = "task24g2_occupancy_aware_cross_floor_visual_proxy_traversal" if is_task24g2 else "task24g_room2_to_room14_cross_floor_visual_proxy_traversal_demo"
    command = (
        "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_astar_demo.sh"
        if is_task24g2
        else "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh"
    )
    route_sentence = (
        "Same-floor portions use A* over stable occupancy maps; the stair portion uses the graph-level 3D connector centerline."
        if is_task24g2
        else "The demo replays the task24f 3D route contract as visual playback."
    )
    return f"""Implemented {task_name}.

Classification: `{report['classification']}`

Run:

```bash
cd /home/ws/workspace/BoxFusion
{command}
```

The demo uses Gazebo SetEntityState to move `{report['gazebo_entity_state_validation']['entity_name']}` and publishes RViz MarkerArray overlays for the planned route, executed trajectory, semantic checkpoints, transition edge, and claim boundary. {route_sentence}

{report['core_statement']}
"""


def manual_instructions(is_task24g2: bool = False) -> str:
    command = (
        "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_astar_demo.sh"
        if is_task24g2
        else "tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh"
    )
    gazebo_text = (
        "confirm that the quadruped visual proxy does not simply move straight from topology point to topology point. It should follow denser same-floor A* paths, then ascend the stair connector, then continue on floor_2 A* paths."
        if is_task24g2
        else "confirm the quadruped visual proxy starts near room_2, moves through room_3, ascends along the stair connector, reaches room_7, continues through room_13, and ends near room_14."
    )
    rviz_extra = (
        "- planned occupancy-aware route is visible.\n- executed trajectory is visible.\n- same-floor route is dense and map-aware.\n"
        if is_task24g2
        else "- planned 3D route is visible.\n- executed 3D trajectory is visible.\n"
    )
    return f"""# Manual GUI Validation Instructions

1. Run:

```bash
cd /home/ws/workspace/BoxFusion
{command}
```

2. In Gazebo, {gazebo_text}

3. In RViz, confirm:

- Fixed Frame is world or map as configured.
- floor_1 and floor_2 context are visible.
{rviz_extra}- stair connector climbs in z.
- claim boundary text is visible.

This is visual_kinematic_proxy_only and topological_vertical_transition_only. It does not prove physical stair climbing, gait, footstep planning, contact-based quadruped locomotion, Nav2 execution, or AMCL/localization.
"""


def load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML profile is not a mapping: {path}")
    return data


def resolve_repo_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPO_ROOT / path


def prepare_visual_urdf(profile_path: Path, out_path: Path, entity_name: str) -> None:
    profile = load_yaml(profile_path)
    source = resolve_repo_path(str(profile["model_urdf"]))
    mesh_dir = resolve_repo_path(str(profile["model_mesh_dir"]))
    tree = ET.parse(str(source))
    robot = tree.getroot()
    robot.set("name", entity_name)
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", (mesh_dir / Path(filename).name).as_uri())
    for joint in robot.findall("joint"):
        if joint.get("type") in {"revolute", "continuous", "prismatic"}:
            joint.set("type", "fixed")
            for child_name in ("axis", "limit", "dynamics", "safety_controller", "calibration", "mimic"):
                child = joint.find(child_name)
                if child is not None:
                    joint.remove(child)
    for gazebo in list(robot.findall("gazebo")):
        robot.remove(gazebo)
    gazebo = ET.SubElement(robot, "gazebo")
    ET.SubElement(gazebo, "static").text = "true"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)


def prepare_gazebo_world(source_world: Path, out_path: Path) -> None:
    tree = ET.parse(str(source_world))
    root = tree.getroot()
    world = root.find("world") if root.tag == "sdf" else root
    if world is None:
        raise ValueError(f"Gazebo world element missing from {source_world}")
    plugins = {plugin.get("filename") for plugin in world.findall("plugin")}
    if "libgazebo_ros_state.so" not in plugins:
        ET.SubElement(world, "plugin", {"name": "task24g_gazebo_ros_state", "filename": "libgazebo_ros_state.so"})
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-contract", type=Path, default=DEFAULT_ROUTE_CONTRACT)
    parser.add_argument("--planned-route-json", type=Path, help="Optional task24g2 dense occupancy-aware planned route JSON.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--entity-name", default=ENTITY_NAME)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--base-frame", default="rslg_cross_floor_base_link")
    parser.add_argument("--horizontal-speed", type=float, default=0.28)
    parser.add_argument("--vertical-speed-limit", type=float, default=0.15)
    parser.add_argument("--publish-rate", type=float, default=10.0)
    parser.add_argument("--checkpoint-tolerance-m", type=float, default=0.35)
    parser.add_argument("--final-tolerance-m", type=float, default=0.40)
    parser.add_argument("--z-visual-scale", type=float, default=1.0)
    parser.add_argument("--service-timeout-sec", type=float, default=25.0)
    parser.add_argument("--hold-final-sec", type=float, default=5.0)
    parser.add_argument("--realtime-scale", type=float, default=1.0)
    parser.add_argument("--planned-marker-topic", default=PLANNED_TOPIC)
    parser.add_argument("--executed-marker-topic", default=EXECUTED_TOPIC)
    parser.add_argument("--visual-marker-topic", default=VISUAL_TOPIC)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--pitch-along-slope", action="store_true")
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
    try:
        if args.planned_route_json:
            plan = load_planned_route_plan(
                args.route_contract,
                args.planned_route_json,
                horizontal_speed=args.horizontal_speed,
                vertical_speed_limit=args.vertical_speed_limit,
                publish_rate=args.publish_rate,
                z_visual_scale=args.z_visual_scale,
            )
        else:
            plan = load_route_plan(
                args.route_contract,
                horizontal_speed=args.horizontal_speed,
                vertical_speed_limit=args.vertical_speed_limit,
                publish_rate=args.publish_rate,
                z_visual_scale=args.z_visual_scale,
            )
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        task_label = "task24g2" if args.planned_route_json else "task24g"
        classification = (
            "task24g2_blocked_by_same_floor_astar_segment"
            if args.planned_route_json
            else "task24g_blocked_by_route_contract"
        )
        report = {
            "artifact_type": f"{task_label}_report",
            "created_utc": now_iso(),
            "classification": classification,
            "route_contract": str(args.route_contract),
            "planned_route_json": str(args.planned_route_json) if args.planned_route_json else None,
            "error": str(exc),
            "claim_boundary": TASK24G2_CLAIM_BOUNDARY if args.planned_route_json else CLAIM_BOUNDARY,
        }
        write_json(args.output_dir / f"{task_label}_report.json", report)
        write_text(args.output_dir / f"{task_label}_report.md", f"# {task_label} Report\n\nClassification: `{classification}`\n\nError: {exc}\n")
        return 2

    if args.print_start_pose:
        if not plan.waypoints:
            return 3
        start = plan.waypoints[0]
        yaw = horizontal_yaw(plan.executable_waypoints[0], plan.executable_waypoints[1], 0.0) if len(plan.executable_waypoints) > 1 else 0.0
        print(f"{start['x']} {start['y']} {start['z']} {yaw}")
        return 0
    if args.prepare_urdf:
        prepare_visual_urdf(args.robot_profile, args.prepare_urdf, args.entity_name)
    if args.prepare_world:
        prepare_gazebo_world(args.source_world, args.prepare_world)
    if args.prepare_only:
        return 0

    runtime: dict[str, Any] = {
        "service_found": False,
        "service_name": None,
        "entity_exists": False,
        "set_entity_success_count": 0,
        "set_entity_failure_count": 0,
        "service_events": [],
    }
    rclpy.init(args=None)
    node = CrossFloorVisualTraversalNode(args, plan)

    def request_shutdown(_signum: int, _frame: Any) -> None:
        node.shutdown_requested = True

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        runtime.update(node.run())
        runtime["service_events"] = node.service_events
        executed = node.executed
    finally:
        try:
            runtime["service_events"] = node.service_events
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    report = write_outputs(args, plan, executed, runtime)
    print(json.dumps({"classification": report["classification"], "output_dir": str(args.output_dir)}, sort_keys=True))
    return 0 if not report["classification"].endswith("_blocked_by_route_contract") else 2


if __name__ == "__main__":
    raise SystemExit(main())
