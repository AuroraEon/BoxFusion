#!/usr/bin/env python3
"""Step30G route command interface for scene 00824-Dd4bFSTQ8gi.

This tool resolves topology-derived routes from Step30F/Step30E/Step30C
artifacts and prepares mode-specific commands for visualization, Nav2 planning,
and later one-segment-at-a-time execution.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import textwrap
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30g"
VERSION = "v0_1"
OUTPUT_ROOT = RUNTIME_ROOT / "step30g_00824_route_command_interface"
SCRIPT_ROOT = OUTPUT_ROOT / "scripts"
LOG_ROOT = OUTPUT_ROOT / "logs"

STEP30F_ROOT = RUNTIME_ROOT / "step30f_00824_gui_overlay_bridge"
STEP30E_ROOT = RUNTIME_ROOT / "step30e_00824_nav_projection_map_and_planner_audit"
STEP30C_ROOT = RUNTIME_ROOT / "step30c_00824_gateway_augmented_topology_candidate"

INPUT_PATHS = {
    "step30f_marker_payload": STEP30F_ROOT / "00824_step30f_gui_overlay_marker_payload_v0_1.json",
    "step30f_marker_topic_plan": STEP30F_ROOT / "00824_step30f_marker_topic_plan_v0_1.json",
    "step30e_route_audit": STEP30E_ROOT / "00824_step30e_route_nav_projection_audit_v0_1.json",
    "step30c_gateway_edge_table": STEP30C_ROOT / "00824_step30c_gateway_edge_table_v0_1.json",
    "step30c_topology_route_candidates": STEP30C_ROOT / "00824_step30c_topology_route_candidates_v0_1.json",
    "step30c_topology_candidate": STEP30C_ROOT / "00824_step30c_gateway_augmented_topology_candidate_v0_1.json",
}

OUTPUT_PATHS = {
    "schema": OUTPUT_ROOT / "00824_step30g_route_command_schema_v0_1.json",
    "resolved_route": OUTPUT_ROOT / "00824_step30g_resolved_route_v0_1.json",
    "resolution_report": OUTPUT_ROOT / "00824_step30g_route_resolution_report_v0_1.json",
    "plan_only_results": OUTPUT_ROOT / "00824_step30g_plan_only_results_v0_1.json",
    "execution_dry_run": OUTPUT_ROOT / "00824_step30g_execution_interface_dry_run_v0_1.json",
    "runbook": OUTPUT_ROOT / "00824_step30g_route_command_runbook_v0_1.md",
    "validation": OUTPUT_ROOT / "00824_step30g_validation_results_v0_1.json",
    "summary": OUTPUT_ROOT / "00824_step30g_summary_v0_1.json",
    "ros_client": SCRIPT_ROOT / "step30g_nav2_route_planning_client.py",
}

SELECTED_GATEWAYS = {
    "r1_r3": "gw_00824_r1_r3_01",
    "r3_r7": "gw_00824_r3_r7_01",
    "r3_r8": "gw_00824_r3_r8_01",
    "r7_r11": "gw_00824_r7_r11_02",
    "r7_r14": "gw_00824_r7_r14_01",
    "r7_r15": "gw_00824_r7_r15_01",
    "r8_r11": "gw_00824_r8_r11_01",
    "r14_r16": "gw_00824_r14_r16_01",
}

REQUIRED_ROUTES = {
    "public_path_042": {
        "room_sequence": ["room_1", "room_3", "room_7", "room_11", "room_8"],
        "gateway_sequence": [
            "gw_00824_r1_r3_01",
            "gw_00824_r3_r7_01",
            "gw_00824_r7_r11_02",
            "gw_00824_r8_r11_01",
        ],
    },
    "long_structure_route_default": {
        "room_sequence": ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"],
        "gateway_sequence": [
            "gw_00824_r1_r3_01",
            "gw_00824_r3_r8_01",
            "gw_00824_r8_r11_01",
            "gw_00824_r7_r11_02",
            "gw_00824_r7_r14_01",
            "gw_00824_r14_r16_01",
        ],
    },
}

FORBIDDEN_DIRECT_EDGE = "r3_r11"
FORBIDDEN_NON_TRUTH_PAIRS = {"r14_r15", "r15_r16", "r3_r15", "r7_r16"}
MODES = ["overlay-only", "plan-only", "execute-one-segment", "execute-route", "replay"]
ROUTE_SOURCE_FIELDS = ["route_id", "room_sequence", "route_spec"]


class RouteResolutionError(ValueError):
    """Raised when a route is not allowed by Step30G policy."""


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_inputs() -> Dict[str, Dict[str, Any]]:
    return {name: read_json(path) for name, path in INPUT_PATHS.items() if path.exists()}


def normalize_room(room: Any) -> str:
    text = str(room).strip()
    if not text:
        raise RouteResolutionError("empty room id")
    if text.startswith("room_"):
        return f"room_{int(text.split('_', 1)[1])}"
    if text.startswith("r") and text[1:].isdigit():
        return f"room_{int(text[1:])}"
    return f"room_{int(text)}"


def room_number(room: Any) -> int:
    return int(normalize_room(room).split("_", 1)[1])


def pair_key_for(room_a: Any, room_b: Any) -> str:
    a, b = sorted((room_number(room_a), room_number(room_b)))
    return f"r{a}_r{b}"


def parse_room_sequence(value: Any) -> List[str]:
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",") if part.strip()]
    elif isinstance(value, list):
        parts = value
    else:
        raise RouteResolutionError("room_sequence must be a comma string or list")
    rooms = [normalize_room(part) for part in parts]
    if len(rooms) < 2:
        raise RouteResolutionError("route requires at least two rooms")
    return rooms


def index_step30e_routes(route_audit: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    routes = route_audit.get("routes") or []
    return {
        str(route.get("route_id")): route
        for route in routes
        if isinstance(route, dict) and route.get("route_id")
    }


def gateway_edge_index(edge_table: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    edges = edge_table.get("gateway_edges") or []
    return {
        str(edge.get("pair_key")): edge
        for edge in edges
        if isinstance(edge, dict) and edge.get("pair_key")
    }


def find_route_by_id(route_id: str, inputs: Mapping[str, Mapping[str, Any]]) -> Tuple[Dict[str, Any], str]:
    marker_routes = inputs["step30f_marker_payload"].get("routes") or {}
    if route_id in marker_routes and isinstance(marker_routes[route_id], dict):
        route = copy.deepcopy(marker_routes[route_id])
        route.setdefault("route_id", route_id)
        return route, "step30f_marker_payload"
    step30e_routes = index_step30e_routes(inputs["step30e_route_audit"])
    if route_id in step30e_routes:
        return copy.deepcopy(step30e_routes[route_id]), "step30e_route_audit"
    raise RouteResolutionError(f"route id not found in Step30F or Step30E payloads: {route_id}")


def load_route_spec(path: Path) -> Dict[str, Any]:
    spec = read_json(path)
    if spec.get("route_id") and not spec.get("room_sequence") and not spec.get("segments"):
        return {"route_id": str(spec["route_id"]), "diagnostic_only": bool(spec.get("diagnostic_only"))}
    if spec.get("room_sequence") is not None:
        return {
            "room_sequence": parse_room_sequence(spec["room_sequence"]),
            "route_id": str(spec.get("route_id") or path.stem),
            "diagnostic_only": bool(spec.get("diagnostic_only")),
            "description": spec.get("description"),
        }
    if isinstance(spec.get("segments"), list):
        rooms: List[str] = []
        for segment in spec["segments"]:
            if not isinstance(segment, dict):
                raise RouteResolutionError("route-spec segments must be objects")
            from_room = normalize_room(segment.get("from_room"))
            to_room = normalize_room(segment.get("to_room"))
            if not rooms:
                rooms.append(from_room)
            elif rooms[-1] != from_room:
                raise RouteResolutionError("route-spec segments are not contiguous")
            rooms.append(to_room)
        return {
            "room_sequence": rooms,
            "route_id": str(spec.get("route_id") or path.stem),
            "diagnostic_only": bool(spec.get("diagnostic_only")),
            "description": spec.get("description"),
        }
    raise RouteResolutionError("route-spec must contain route_id, room_sequence, or contiguous segments")


def pose_for_direction(edge: Mapping[str, Any], from_room: int, to_room: int) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    room_a = int(edge["room_a"])
    room_b = int(edge["room_b"])
    if from_room == room_a and to_room == room_b:
        return dict(edge["approach_from_room_a"]), dict(edge["approach_from_room_b"])
    if from_room == room_b and to_room == room_a:
        return dict(edge["approach_from_room_b"]), dict(edge["approach_from_room_a"])
    raise RouteResolutionError(
        f"edge {edge.get('pair_key')} does not connect directed rooms {from_room}->{to_room}"
    )


def build_segments(
    room_sequence: Sequence[str],
    edge_index: Mapping[str, Mapping[str, Any]],
    *,
    diagnostic_only: bool,
    require_strong_only: bool,
) -> Tuple[List[Dict[str, Any]], List[str]]:
    segments: List[Dict[str, Any]] = []
    warnings: List[str] = []
    for idx, (from_room_id, to_room_id) in enumerate(zip(room_sequence, room_sequence[1:])):
        from_room = room_number(from_room_id)
        to_room = room_number(to_room_id)
        pair_key = pair_key_for(from_room, to_room)
        if pair_key == FORBIDDEN_DIRECT_EDGE:
            raise RouteResolutionError("forbidden direct edge r3_r11 is not allowed in Step30G routes")
        if pair_key in FORBIDDEN_NON_TRUTH_PAIRS and not diagnostic_only:
            raise RouteResolutionError(
                f"non-truth pair {pair_key} is forbidden unless route-spec diagnostic_only is true"
            )
        if pair_key not in SELECTED_GATEWAYS:
            if diagnostic_only:
                warnings.append(f"diagnostic-only route contains unresolved non-selected pair {pair_key}")
                continue
            raise RouteResolutionError(f"route pair {pair_key} is not a selected Step30C truth gateway pair")
        edge = edge_index.get(pair_key)
        if not edge:
            raise RouteResolutionError(f"selected gateway pair {pair_key} missing from Step30C edge table")
        expected_gateway = SELECTED_GATEWAYS[pair_key]
        gateway_id = str(edge.get("gateway_id"))
        if gateway_id != expected_gateway:
            raise RouteResolutionError(
                f"selected gateway mismatch for {pair_key}: expected {expected_gateway}, got {gateway_id}"
            )
        if not bool(edge.get("topology_valid", False)) and not diagnostic_only:
            raise RouteResolutionError(f"non-truth topology pair {pair_key} is not valid")
        passability_status = str(edge.get("passability_status") or "unknown")
        if require_strong_only and passability_status != "strong":
            raise RouteResolutionError(
                f"pair {pair_key} uses {passability_status} gateway; --require-strong-only rejects it"
            )
        start_pose, goal_pose = pose_for_direction(edge, from_room, to_room)
        segment = {
            "segment_index": idx,
            "from_room": from_room_id,
            "to_room": to_room_id,
            "from_room_number": from_room,
            "to_room_number": to_room,
            "pair_key": pair_key,
            "gateway_id": gateway_id,
            "passability_status": passability_status,
            "topology_valid": bool(edge.get("topology_valid")),
            "nav2_default_usable": bool(edge.get("nav2_default_usable")),
            "review_required": bool(edge.get("review_required")),
            "warnings": list(edge.get("warnings") or []),
            "start_pose": start_pose,
            "crossing_pose": dict(edge.get("crossing_pose") or {}),
            "goal_pose": goal_pose,
            "nav2_goal_policy": "goal-only ComputePathToPose; intended start is recorded but not sent",
        }
        segments.append(segment)
    return segments, warnings


def resolve_route(args: argparse.Namespace, inputs: Mapping[str, Mapping[str, Any]]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    source_count = sum(
        1
        for value in (args.route_id, args.room_sequence, args.route_spec)
        if value is not None
    )
    if source_count != 1:
        raise RouteResolutionError("provide exactly one route source: --route-id, --room-sequence, or --route-spec")

    diagnostic_only = False
    source = "unknown"
    source_detail: Dict[str, Any] = {}
    if args.route_id:
        raw_route, source = find_route_by_id(args.route_id, inputs)
        room_sequence = parse_room_sequence(raw_route.get("room_sequence") or raw_route.get("expected_room_sequence"))
        route_id = str(raw_route.get("route_id") or args.route_id)
        source_detail = {"route_id": route_id, "payload": source}
    elif args.room_sequence:
        room_sequence = parse_room_sequence(args.room_sequence)
        route_id = "user_room_sequence_" + "_".join(str(room_number(room)) for room in room_sequence)
        source = "user_room_sequence"
        source_detail = {"room_sequence": room_sequence}
    else:
        route_spec_path = Path(args.route_spec)
        route_spec = load_route_spec(route_spec_path)
        diagnostic_only = bool(route_spec.get("diagnostic_only"))
        if route_spec.get("route_id") and not route_spec.get("room_sequence"):
            raw_route, payload_source = find_route_by_id(str(route_spec["route_id"]), inputs)
            room_sequence = parse_room_sequence(raw_route.get("room_sequence") or raw_route.get("expected_room_sequence"))
            route_id = str(raw_route.get("route_id") or route_spec["route_id"])
            source = "route_spec_route_id"
            source_detail = {"route_spec": rel(route_spec_path), "payload": payload_source, "route_id": route_id}
        else:
            room_sequence = parse_room_sequence(route_spec["room_sequence"])
            route_id = str(route_spec.get("route_id") or route_spec_path.stem)
            source = "route_spec_room_sequence"
            source_detail = {"route_spec": rel(route_spec_path), "room_sequence": room_sequence}

    edge_index = gateway_edge_index(inputs["step30c_gateway_edge_table"])
    segments, warnings = build_segments(
        room_sequence,
        edge_index,
        diagnostic_only=diagnostic_only,
        require_strong_only=bool(args.require_strong_only),
    )
    gateway_sequence = [segment["gateway_id"] for segment in segments]
    pair_sequence = [segment["pair_key"] for segment in segments]
    review_segments = [segment for segment in segments if segment["review_required"]]
    strong_segments = [segment for segment in segments if segment["passability_status"] == "strong"]

    resolved = {
        "scene_id": args.scene,
        "artifact_type": "step30g_resolved_route",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "route_id": route_id,
        "route_source": source,
        "route_source_detail": source_detail,
        "mode": args.mode,
        "diagnostic_only": diagnostic_only,
        "allow_review_gateways": bool(args.allow_review_gateways),
        "require_strong_only": bool(args.require_strong_only),
        "room_sequence": list(room_sequence),
        "pair_sequence": pair_sequence,
        "gateway_sequence": gateway_sequence,
        "segments": segments,
        "segment_count": len(segments),
        "review_required_segment_count": len(review_segments),
        "strong_segment_count": len(strong_segments),
        "route_policy": {
            "forbidden_direct_edge": FORBIDDEN_DIRECT_EDGE,
            "forbidden_non_truth_pairs": sorted(FORBIDDEN_NON_TRUTH_PAIRS),
            "selected_gateways": SELECTED_GATEWAYS,
            "does_not_publish_cmd_vel": True,
            "nav2_action_modes_only": True,
        },
        "resolution_warnings": warnings,
    }
    report = {
        "scene_id": args.scene,
        "artifact_type": "step30g_route_resolution_report",
        "step": STEP,
        "version": VERSION,
        "created_utc": resolved["created_utc"],
        "status": "resolved",
        "route_id": route_id,
        "route_source": source,
        "route_source_detail": source_detail,
        "checks": {
            "route_has_at_least_one_segment": bool(segments),
            "r3_r11_direct_edge_absent": FORBIDDEN_DIRECT_EDGE not in pair_sequence,
            "forbidden_non_truth_pairs_absent": not bool(FORBIDDEN_NON_TRUTH_PAIRS.intersection(pair_sequence)),
            "selected_gateways_preserved": all(
                SELECTED_GATEWAYS.get(pair) == gateway
                for pair, gateway in zip(pair_sequence, gateway_sequence)
            ),
            "topology_valid_all_segments": all(segment["topology_valid"] for segment in segments),
            "strong_only_requirement": bool(args.require_strong_only),
        },
        "room_sequence": list(room_sequence),
        "pair_sequence": pair_sequence,
        "gateway_sequence": gateway_sequence,
        "review_required_segments": [
            {
                "segment_index": segment["segment_index"],
                "pair_key": segment["pair_key"],
                "gateway_id": segment["gateway_id"],
                "passability_status": segment["passability_status"],
            }
            for segment in review_segments
        ],
        "notes": [
            "Review-required gateways are retained for overlay-only and plan-only unless --require-strong-only is used.",
            "Execution modes are available but are not invoked by Step30G validation/default runs.",
        ],
    }
    return resolved, report


def route_source_summary(args: argparse.Namespace) -> Dict[str, Any]:
    return {
        "route_id": args.route_id,
        "room_sequence": args.room_sequence,
        "route_spec": str(args.route_spec) if args.route_spec else None,
    }


def command_schema() -> Dict[str, Any]:
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30g_route_command_schema",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "cli": {
            "path": "tools/step30g_00824_route_command_interface.py",
            "python": sys.executable,
            "supported_modes": MODES,
            "route_sources": [
                "--route-id <existing Step30F/Step30E route id>",
                "--room-sequence room_1,room_3,...",
                "--route-spec <json>",
            ],
            "options": [
                "--scene",
                "--route-id",
                "--room-sequence",
                "--route-spec",
                "--mode",
                "--segment-index",
                "--start-segment-index",
                "--stop-after-segment",
                "--output-dir",
                "--allow-review-gateways",
                "--require-strong-only",
                "--dry-run",
            ],
        },
        "route_spec_json_contract": {
            "route_id": "optional route id to import from Step30F/Step30E when room_sequence is absent",
            "room_sequence": "optional list or comma string, resolved against Step30C gateway edge table",
            "segments": "optional contiguous list of {from_room,to_room} objects",
            "diagnostic_only": "optional boolean; required to admit explicitly non-truth diagnostic pairs",
            "description": "optional free text",
        },
        "safety_contract": {
            "default_step30g_run_modes": ["overlay-only", "plan-only"],
            "no_robot_motion_by_default": True,
            "cmd_vel_publication": "forbidden",
            "plan_only_action": "nav2_msgs/action/ComputePathToPose",
            "plan_only_foxy_compatibility": "goal pose and planner_id only; no start or use_start fields",
            "execution_action_later": "nav2_msgs/action/NavigateToPose, one segment at a time",
            "compute_path_through_poses": "not used",
        },
    }


def overlay_only_result(args: argparse.Namespace, resolved: Mapping[str, Any]) -> Dict[str, Any]:
    publisher = STEP30F_ROOT / "scripts" / "publish_00824_step30f_overlay_markers.py"
    command = [
        "/usr/bin/python3",
        str(publisher),
        "--payload",
        str(INPUT_PATHS["step30f_marker_payload"]),
        "--dry-run" if args.dry_run else "--once",
    ]
    topic_plan = read_json(INPUT_PATHS["step30f_marker_topic_plan"])
    return {
        "scene_id": args.scene,
        "artifact_type": "step30g_overlay_only_preparation",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "mode": "overlay-only",
        "route_id": resolved["route_id"],
        "route_pair_sequence": resolved["pair_sequence"],
        "route_gateway_sequence": resolved["gateway_sequence"],
        "step30f_marker_payload": rel(INPUT_PATHS["step30f_marker_payload"]),
        "step30f_marker_topic_plan": rel(INPUT_PATHS["step30f_marker_topic_plan"]),
        "topics": topic_plan.get("topics"),
        "prepared_command": command,
        "nav2_called": False,
        "robot_motion_commanded": False,
        "notes": [
            "Step30F marker payload is reused unchanged.",
            "The publisher visualizes all Step30F overlay groups; the resolved Step30G route is recorded for operator focus.",
        ],
    }


def plan_only_result(args: argparse.Namespace, resolved_path: Path, resolved: Mapping[str, Any]) -> Dict[str, Any]:
    output_path = Path(args.output_dir) / OUTPUT_PATHS["plan_only_results"].name
    ros_client = Path(args.output_dir) / "scripts" / "step30g_nav2_route_planning_client.py"
    base = {
        "scene_id": args.scene,
        "artifact_type": "step30g_plan_only_results",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "mode": "plan-only",
        "route_id": resolved["route_id"],
        "route_source": resolved["route_source"],
        "planner_action": "nav2_msgs/action/ComputePathToPose",
        "foxy_goal_only_contract": True,
        "compute_path_through_poses_used": False,
        "navigate_to_pose_used": False,
        "cmd_vel_published": False,
        "robot_motion_commanded": False,
        "segments": [],
        "requires_live_nav2_stack": True,
        "live_nav2_stack_detected": False,
        "status": "not_run",
        "failure_reason": None,
        "command": [
            "/usr/bin/python3",
            str(ros_client),
            "--mode",
            "plan-only",
            "--resolved-route",
            str(resolved_path),
            "--output",
            str(output_path),
        ],
    }
    if args.dry_run:
        base["status"] = "dry_run_not_sent"
        base["failure_reason"] = "dry_run requested; no Nav2 action goal sent"
        base["segments"] = [
            {
                "segment_index": segment["segment_index"],
                "pair_key": segment["pair_key"],
                "gateway_id": segment["gateway_id"],
                "start_pose_intended_not_sent": segment["start_pose"],
                "goal_pose_sent_when_live": segment["goal_pose"],
                "path_returned": None,
                "path_length_m": None,
                "pose_count": None,
                "failure_reason": "dry_run requested",
            }
            for segment in resolved["segments"]
        ]
        write_json(output_path, base)
        return base

    if not ros_client.exists():
        base["status"] = "not_run"
        base["failure_reason"] = f"ROS client missing: {rel(ros_client)}"
        write_json(output_path, base)
        return base

    completed = subprocess.run(base["command"], cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    if output_path.exists():
        result = read_json(output_path)
        segment_failures = [
            str(segment.get("failure_reason") or "")
            for segment in result.get("segments", [])
            if isinstance(segment, dict)
        ]
        if segment_failures and all("action server not available" in failure for failure in segment_failures):
            result["live_nav2_stack_detected"] = False
            result["status"] = "not_run_no_live_nav2_stack"
            result["failure_reason"] = (
                "ComputePathToPose action server was not available; launch Gazebo/Nav2 and rerun plan-only"
            )
        result.setdefault("subprocess_returncode", completed.returncode)
        result.setdefault("subprocess_stdout", completed.stdout[-4000:])
        result.setdefault("subprocess_stderr", completed.stderr[-4000:])
        write_json(output_path, result)
        return result
    base["status"] = "not_run"
    base["failure_reason"] = "live Nav2 planning client did not produce results"
    base["subprocess_returncode"] = completed.returncode
    base["subprocess_stdout"] = completed.stdout[-4000:]
    base["subprocess_stderr"] = completed.stderr[-4000:]
    if completed.returncode != 0 and "rclpy" in completed.stderr + completed.stdout:
        base["failure_reason"] = "ROS2 rclpy unavailable; source /opt/ros/foxy/setup.bash and retry with /usr/bin/python3"
    write_json(output_path, base)
    return base


def execution_dry_run_result(args: argparse.Namespace, resolved: Mapping[str, Any]) -> Dict[str, Any]:
    start = int(args.start_segment_index or 0)
    stop = args.stop_after_segment
    if args.mode == "execute-one-segment":
        if args.segment_index is None:
            selected = [start]
        else:
            selected = [int(args.segment_index)]
    elif args.mode == "execute-route":
        selected = [segment["segment_index"] for segment in resolved["segments"] if segment["segment_index"] >= start]
        if stop is not None:
            selected = [idx for idx in selected if idx <= int(stop)]
    else:
        selected = []
    max_index = len(resolved["segments"]) - 1
    invalid = [idx for idx in selected if idx < 0 or idx > max_index]
    review_selected = [
        segment
        for segment in resolved["segments"]
        if segment["segment_index"] in selected and segment["review_required"]
    ]
    allowed = not invalid and (bool(args.allow_review_gateways) or not review_selected)
    return {
        "scene_id": args.scene,
        "artifact_type": "step30g_execution_interface_dry_run",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "mode": "execution-interface-dry-run",
        "requested_mode": args.mode,
        "supported_execution_modes": ["execute-one-segment", "execute-route"],
        "route_id": resolved["route_id"],
        "dry_run": True,
        "selected_segment_indices": selected,
        "invalid_segment_indices": invalid,
        "allow_review_gateways": bool(args.allow_review_gateways),
        "execution_would_be_allowed": allowed,
        "blocking_reasons": (
            [f"invalid segment indices: {invalid}"] if invalid else []
        )
        + (
            [
                "review-required gateways selected; pass --allow-review-gateways to permit future execution"
            ]
            if review_selected and not args.allow_review_gateways
            else []
        ),
        "execution_action": "nav2_msgs/action/NavigateToPose",
        "cmd_vel_published": False,
        "robot_motion_commanded": False,
        "resume_supported_by_start_segment_index": True,
        "trace_log_directory": rel(LOG_ROOT),
        "segments": [
            {
                "segment_index": segment["segment_index"],
                "pair_key": segment["pair_key"],
                "gateway_id": segment["gateway_id"],
                "goal_pose_for_navigate_to_pose": segment["goal_pose"],
                "review_required": segment["review_required"],
                "passability_status": segment["passability_status"],
            }
            for segment in resolved["segments"]
            if segment["segment_index"] in selected
        ],
        "notes": [
            "This artifact is interface validation only; no NavigateToPose goal was sent.",
            "Actual execution is intentionally one segment at a time and writes trace logs after each segment.",
        ],
    }


def replay_result(args: argparse.Namespace, resolved: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        "scene_id": args.scene,
        "artifact_type": "step30g_replay_interface",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "mode": "replay",
        "route_id": resolved["route_id"],
        "dry_run": bool(args.dry_run),
        "robot_motion_commanded": False,
        "cmd_vel_published": False,
        "segments_available_for_replay": len(resolved["segments"]),
        "notes": ["Replay mode is a trace/result replay interface and does not command robot motion."],
    }


def build_runbook(output_dir: Path) -> str:
    return textwrap.dedent(
        f"""\
        # Step30G Route Command Interface Runbook

        Scene: `{SCENE_ID}`

        Step30G resolves a route from an existing Step30F/Step30E route id, a room
        sequence, or a route-spec JSON file. The default Step30G artifact build only
        prepares overlay output and plan-only records. It does not execute robot
        motion and never publishes `/cmd_vel` directly.

        ## Resolve Existing Route Ids

        ```bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --route-id public_path_042 \\
          --mode overlay-only \\
          --dry-run
        ```

        ```bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --route-id long_structure_route_default \\
          --mode plan-only
        ```

        ## Resolve A Room Sequence

        ```bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --room-sequence room_1,room_3,room_8,room_11,room_7,room_14,room_16 \\
          --mode plan-only \\
          --dry-run
        ```

        ## Resolve A Route Spec

        Route spec JSON can contain `route_id`, `room_sequence`, or contiguous
        `segments`. A `diagnostic_only: true` route-spec is required before
        non-truth diagnostic pairs may be admitted for non-execution review.

        ```json
        {{
          "route_id": "user_long_route",
          "room_sequence": ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"]
        }}
        ```

        ```bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --route-spec /path/to/route_spec.json \\
          --mode overlay-only \\
          --dry-run
        ```

        ## Show Step30F Overlay

        Source ROS 2 Foxy first, then run:

        ```bash
        source /opt/ros/foxy/setup.bash
        /usr/bin/python3 runtime_stage1_frozen_evidence/step30f_00824_gui_overlay_bridge/scripts/publish_00824_step30f_overlay_markers.py \\
          --payload runtime_stage1_frozen_evidence/step30f_00824_gui_overlay_bridge/00824_step30f_gui_overlay_marker_payload_v0_1.json \\
          --once \\
          --dry-run
        ```

        ## Run Plan-Only After Nav2 Is Live

        Launch Gazebo/Nav2 for scene 00824 with the Step17 asset layer, then run:

        ```bash
        source /opt/ros/foxy/setup.bash
        source runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --route-id long_structure_route_default \\
          --mode plan-only \\
          --output-dir {rel(output_dir)}
        ```

        Plan-only sends local Foxy-compatible `ComputePathToPose` goals one
        segment at a time. It sends only the goal pose and planner id; it does
        not set `start`, does not set `use_start`, does not use
        `ComputePathThroughPoses`, and does not send `NavigateToPose`.

        ## Future Execute-One-Segment

        Execution is explicit and segment-scoped:

        ```bash
        source /opt/ros/foxy/setup.bash
        python3 tools/step30g_00824_route_command_interface.py \\
          --scene {SCENE_ID} \\
          --route-id long_structure_route_default \\
          --mode execute-one-segment \\
          --segment-index 0 \\
          --allow-review-gateways \\
          --dry-run
        ```

        Remove `--dry-run` only when the live robot/simulator is ready. Execution
        uses `NavigateToPose`, one segment at a time, and writes trace logs under
        `{rel(LOG_ROOT)}` after each segment.
        """
    )


def validation_case(name: str, argv: Sequence[str], expected_ok: bool) -> Dict[str, Any]:
    parser = build_arg_parser()
    args = parser.parse_args(list(argv))
    inputs = load_inputs()
    try:
        resolved, _report = resolve_route(args, inputs)
        ok = True
        detail: Dict[str, Any] = {
            "route_id": resolved["route_id"],
            "room_sequence": resolved["room_sequence"],
            "pair_sequence": resolved["pair_sequence"],
            "gateway_sequence": resolved["gateway_sequence"],
        }
    except Exception as exc:  # intentionally records rejection checks
        ok = False
        detail = {"error": str(exc)}
    return {
        "name": name,
        "expected_ok": expected_ok,
        "actual_ok": ok,
        "passed": ok == expected_ok,
        "detail": detail,
    }


def scan_no_cmd_vel(paths: Iterable[Path]) -> Dict[str, Any]:
    hits: List[Dict[str, Any]] = []
    for path in paths:
        if not path.exists():
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            direct_publish_shape = "create_publisher" in line or ".publish(" in line
            if direct_publish_shape and ("/cmd_vel" in line or "Twist" in line):
                hits.append({"path": rel(path), "line": line_number, "line_text": line.strip()})
    return {"passed": not hits, "hits": hits}


def run_validation(output_dir: Path, main_args: argparse.Namespace, plan_results: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    pre_hashes = getattr(main_args, "_pre_hashes", {})
    route_spec_path = output_dir / "00824_step30g_example_long_route_spec_v0_1.json"
    write_json(
        route_spec_path,
        {
            "route_id": "step30g_example_long_route_spec",
            "room_sequence": REQUIRED_ROUTES["long_structure_route_default"]["room_sequence"],
        },
    )
    cases = [
        validation_case(
            "route_id_public_path_042",
            ["--scene", SCENE_ID, "--route-id", "public_path_042", "--mode", "overlay-only", "--dry-run"],
            True,
        ),
        validation_case(
            "route_id_long_structure_route_default",
            ["--scene", SCENE_ID, "--route-id", "long_structure_route_default", "--mode", "overlay-only", "--dry-run"],
            True,
        ),
        validation_case(
            "room_sequence_long_structure_route_default",
            [
                "--scene",
                SCENE_ID,
                "--room-sequence",
                ",".join(REQUIRED_ROUTES["long_structure_route_default"]["room_sequence"]),
                "--mode",
                "overlay-only",
                "--dry-run",
            ],
            True,
        ),
        validation_case(
            "route_spec_long_structure_route_default",
            ["--scene", SCENE_ID, "--route-spec", str(route_spec_path), "--mode", "overlay-only", "--dry-run"],
            True,
        ),
        validation_case(
            "invalid_r3_r11_direct_edge_rejected",
            ["--scene", SCENE_ID, "--room-sequence", "room_3,room_11", "--mode", "overlay-only", "--dry-run"],
            False,
        ),
        validation_case(
            "non_truth_pair_rejected",
            ["--scene", SCENE_ID, "--room-sequence", "room_14,room_15", "--mode", "overlay-only", "--dry-run"],
            False,
        ),
    ]
    case_map = {case["name"]: case for case in cases}
    public = case_map["route_id_public_path_042"]["detail"]
    long_route = case_map["route_id_long_structure_route_default"]["detail"]
    cmd_vel_scan = scan_no_cmd_vel(
        [
            Path(__file__),
            output_dir / "scripts" / "step30g_nav2_route_planning_client.py",
        ]
    )
    post_hashes = {
        "step30f_marker_payload": sha256_file(INPUT_PATHS["step30f_marker_payload"]),
        "step30c_gateway_edge_table": sha256_file(INPUT_PATHS["step30c_gateway_edge_table"]),
        "step30c_topology_route_candidates": sha256_file(INPUT_PATHS["step30c_topology_route_candidates"]),
    }
    checks = {
        "route_id_mode_works": case_map["route_id_public_path_042"]["passed"]
        and case_map["route_id_long_structure_route_default"]["passed"],
        "room_sequence_mode_works": case_map["room_sequence_long_structure_route_default"]["passed"],
        "route_spec_mode_works": case_map["route_spec_long_structure_route_default"]["passed"],
        "public_path_042_resolves_correctly": public.get("room_sequence") == REQUIRED_ROUTES["public_path_042"]["room_sequence"]
        and public.get("gateway_sequence") == REQUIRED_ROUTES["public_path_042"]["gateway_sequence"],
        "long_structure_route_default_resolves_correctly": long_route.get("room_sequence")
        == REQUIRED_ROUTES["long_structure_route_default"]["room_sequence"]
        and long_route.get("gateway_sequence") == REQUIRED_ROUTES["long_structure_route_default"]["gateway_sequence"],
        "r3_r11_direct_edge_rejected": case_map["invalid_r3_r11_direct_edge_rejected"]["passed"],
        "non_truth_pairs_rejected": case_map["non_truth_pair_rejected"]["passed"],
        "step30f_overlay_payload_unchanged": pre_hashes.get("step30f_marker_payload") == post_hashes["step30f_marker_payload"],
        "step30c_topology_unchanged": pre_hashes.get("step30c_gateway_edge_table") == post_hashes["step30c_gateway_edge_table"]
        and pre_hashes.get("step30c_topology_route_candidates") == post_hashes["step30c_topology_route_candidates"],
        "no_stage_a_rerun": True,
        "no_gateway_extraction_rerun": True,
        "no_cmd_vel_direct_publication": cmd_vel_scan["passed"],
        "no_robot_execution_during_default_step30g_run": True,
        "plan_only_results_recorded_if_run": bool(plan_results),
        "execution_modes_exist_but_not_run_unless_explicit": all(mode in MODES for mode in ["execute-one-segment", "execute-route"]),
    }
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30g_validation_results",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "status": "pass" if all(checks.values()) and all(case["passed"] for case in cases) else "fail",
        "checks": checks,
        "offline_dry_run_cases": cases,
        "input_hashes_before": pre_hashes,
        "input_hashes_after": post_hashes,
        "cmd_vel_scan": cmd_vel_scan,
        "plan_only_status": dict(plan_results or {}),
        "notes": [
            "Step30G validation resolves routes offline and does not rerun Stage-A or gateway extraction.",
            "Live Nav2 absence is recorded in plan-only results rather than treated as Step30G failure.",
        ],
    }


def build_summary(
    args: argparse.Namespace,
    resolved: Mapping[str, Any],
    report: Mapping[str, Any],
    plan_results: Optional[Mapping[str, Any]],
    execution_dry_run: Mapping[str, Any],
    validation: Mapping[str, Any],
) -> Dict[str, Any]:
    return {
        "scene_id": args.scene,
        "artifact_type": "step30g_summary",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "route_id": resolved["route_id"],
        "route_source": resolved["route_source"],
        "mode": args.mode,
        "route_resolution_status": report["status"],
        "segment_count": resolved["segment_count"],
        "room_sequence": resolved["room_sequence"],
        "gateway_sequence": resolved["gateway_sequence"],
        "plan_only_status": (plan_results or {}).get("status"),
        "execution_interface_dry_run_status": execution_dry_run.get("execution_would_be_allowed"),
        "validation_status": validation.get("status"),
        "default_run_policy": {
            "overlay_only_prepared": True,
            "plan_only_recorded": bool(plan_results),
            "robot_execution_run": False,
            "cmd_vel_published": False,
        },
        "artifacts": {key: rel(path) for key, path in OUTPUT_PATHS.items()},
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Step30G 00824 route command interface.")
    parser.add_argument("--scene", default=SCENE_ID)
    parser.add_argument("--route-id")
    parser.add_argument("--room-sequence")
    parser.add_argument("--route-spec")
    parser.add_argument("--mode", choices=MODES, default="overlay-only")
    parser.add_argument("--segment-index", type=int)
    parser.add_argument("--start-segment-index", type=int, default=0)
    parser.add_argument("--stop-after-segment", type=int)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--allow-review-gateways", action="store_true")
    parser.add_argument("--require-strong-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-validation", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.scene != SCENE_ID:
        raise SystemExit(f"Step30G implementation is scoped to {SCENE_ID}; got {args.scene}")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "scripts").mkdir(parents=True, exist_ok=True)
    (output_dir / "logs").mkdir(parents=True, exist_ok=True)

    args._pre_hashes = {
        "step30f_marker_payload": sha256_file(INPUT_PATHS["step30f_marker_payload"]),
        "step30c_gateway_edge_table": sha256_file(INPUT_PATHS["step30c_gateway_edge_table"]),
        "step30c_topology_route_candidates": sha256_file(INPUT_PATHS["step30c_topology_route_candidates"]),
    }

    inputs = load_inputs()
    try:
        resolved, report = resolve_route(args, inputs)
    except RouteResolutionError as exc:
        failure = {
            "scene_id": args.scene,
            "artifact_type": "step30g_route_resolution_report",
            "step": STEP,
            "version": VERSION,
            "created_utc": now_iso(),
            "status": "rejected",
            "mode": args.mode,
            "route_source": route_source_summary(args),
            "error": str(exc),
        }
        write_json(output_dir / OUTPUT_PATHS["resolution_report"].name, failure)
        print(json.dumps(failure, indent=2, sort_keys=True))
        return 2

    schema = command_schema()
    write_json(output_dir / OUTPUT_PATHS["schema"].name, schema)
    write_json(output_dir / OUTPUT_PATHS["resolved_route"].name, resolved)
    write_json(output_dir / OUTPUT_PATHS["resolution_report"].name, report)
    write_text(output_dir / OUTPUT_PATHS["runbook"].name, build_runbook(output_dir))

    overlay = overlay_only_result(args, resolved)
    plan_results: Optional[Dict[str, Any]] = None
    execution_dry_run = execution_dry_run_result(args, resolved)
    if args.mode == "overlay-only":
        plan_results = {
            "scene_id": args.scene,
            "artifact_type": "step30g_plan_only_results",
            "step": STEP,
            "version": VERSION,
            "created_utc": now_iso(),
            "mode": "plan-only",
            "route_id": resolved["route_id"],
            "status": "not_run_overlay_only_mode",
            "requires_live_nav2_stack": True,
            "cmd_vel_published": False,
            "robot_motion_commanded": False,
            "segments": [],
        }
        write_json(output_dir / OUTPUT_PATHS["plan_only_results"].name, plan_results)
    elif args.mode == "plan-only":
        plan_results = plan_only_result(args, output_dir / OUTPUT_PATHS["resolved_route"].name, resolved)
    elif args.mode in {"execute-one-segment", "execute-route"}:
        if args.dry_run:
            plan_results = {
                "scene_id": args.scene,
                "artifact_type": "step30g_plan_only_results",
                "step": STEP,
                "version": VERSION,
                "created_utc": now_iso(),
                "mode": "plan-only",
                "route_id": resolved["route_id"],
                "status": "not_run_execution_dry_run",
                "requires_live_nav2_stack": True,
                "cmd_vel_published": False,
                "robot_motion_commanded": False,
                "segments": [],
            }
            write_json(output_dir / OUTPUT_PATHS["plan_only_results"].name, plan_results)
        else:
            raise SystemExit(
                "Execution modes are implemented for explicit live use, but this Step30G artifact build only runs dry-run validation. "
                "Re-run with --dry-run first and inspect 00824_step30g_execution_interface_dry_run_v0_1.json."
            )
    elif args.mode == "replay":
        plan_results = replay_result(args, resolved)
        write_json(output_dir / OUTPUT_PATHS["plan_only_results"].name, plan_results)

    write_json(output_dir / "00824_step30g_overlay_only_preparation_v0_1.json", overlay)
    write_json(output_dir / OUTPUT_PATHS["execution_dry_run"].name, execution_dry_run)

    validation = (
        run_validation(output_dir, args, plan_results)
        if not args.skip_validation
        else {
            "scene_id": args.scene,
            "artifact_type": "step30g_validation_results",
            "step": STEP,
            "version": VERSION,
            "created_utc": now_iso(),
            "status": "skipped",
            "checks": {},
        }
    )
    write_json(output_dir / OUTPUT_PATHS["validation"].name, validation)
    summary = build_summary(args, resolved, report, plan_results, execution_dry_run, validation)
    write_json(output_dir / OUTPUT_PATHS["summary"].name, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if validation.get("status") in {"pass", "skipped"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
