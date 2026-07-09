#!/usr/bin/env python3
"""Build the task60 scripted stair-transition runtime input for RSLG-SLAM.

The adapter consumes the already validated task56c cross-floor RouteResult and
PID runtime input. It keeps the same semantic route order, executes floor_1 and
floor_2 as flat Gazebo PID segments, and replaces the vertical connector handoff
with explicit scripted stair keyframes. It does not run Stage-A, raw RGB-D
inference, Nav2, AMCL, map_server, or physical robot code.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from scripted_stair_transition_common import (  # noqa: E402
    BLOCKED_CANDIDATE_ID,
    DEFAULT_PROFILE_ID,
    DEFAULT_QUERY_ID,
    FALLBACK_QUERY_ID,
    FLOOR_1_Z,
    FLOOR_2_Z,
    FORBIDDEN_NON_TRANSITION_EDGE,
    PROJECT_NAME,
    RVIZ_RELATIVE_PATH,
    SCENE_ID,
    SELECTED_APPROACH_ID,
    TASK_NAME,
    TOPICS,
    TRUE_TRANSITION_EDGE,
    WORLD_RELATIVE_PATH,
    as_float,
    distance_xy,
    normalize_angle,
    pid_input_path,
    read_json,
    route_points,
    route_result_path,
    runtime_input_path,
    runtime_pack_dir,
    runtime_task_dir,
    utc_now,
    write_json,
    write_text,
    yaw_between,
)


REPO_ROOT = SCRIPT_DIR.parents[2]


def route_paths(query_id: str) -> dict[str, Path]:
    task56c = (
        REPO_ROOT
        / "stage_outputs"
        / "rslg_slam"
        / SCENE_ID
        / "tasks"
        / "task56c_pid_profile_promotion_and_regression_validation"
        / "regression_pack"
    )
    return {
        "route_result": route_result_path(REPO_ROOT, query_id),
        "pid_input": pid_input_path(REPO_ROOT, query_id),
        "rviz_marker_input": task56c
        / "runtime_adapter_inputs"
        / "rviz_marker_inputs"
        / f"{query_id}_rviz_marker_input.json",
        "z_aware_overlay_input": task56c
        / "runtime_adapter_inputs"
        / "z_aware_overlay_inputs"
        / f"{query_id}_z_aware_overlay_input.json",
    }


def floor_counts(waypoints: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for waypoint in waypoints:
        floor_id = str(waypoint.get("floor_id") or "")
        counts[floor_id] = counts.get(floor_id, 0) + 1
    return dict(sorted(counts.items()))


def selected_goal_id(pid_input: dict[str, Any]) -> str | None:
    for key in ("selected_goal", "selected_approach", "endpoint"):
        value = pid_input.get(key)
        if isinstance(value, dict) and value.get("candidate_id"):
            return str(value["candidate_id"])
    return None


def transition_edges(route_result: dict[str, Any], pid_input: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    semantic_route = route_result.get("semantic_route") or {}
    for connector in semantic_route.get("connector_sequence") or []:
        if isinstance(connector, dict) and connector.get("transition_edge"):
            edges.append(str(connector["transition_edge"]))
    for handoff in pid_input.get("connector_handoffs") or []:
        if isinstance(handoff, dict) and handoff.get("transition_edge"):
            edges.append(str(handoff["transition_edge"]))
    return edges


def rejected_candidate_ids(route_result: dict[str, Any], pid_input: dict[str, Any]) -> list[str]:
    ids: list[str] = []
    for candidate in (route_result.get("approach") or {}).get("rejected_candidates") or []:
        if isinstance(candidate, dict) and candidate.get("candidate_id"):
            ids.append(str(candidate["candidate_id"]))
    for candidate in pid_input.get("rejected_runtime_candidates") or []:
        if isinstance(candidate, dict) and candidate.get("candidate_id"):
            ids.append(str(candidate["candidate_id"]))
    return sorted(set(ids))


def first_connector_handoff(pid_input: dict[str, Any]) -> dict[str, Any]:
    for handoff in pid_input.get("connector_handoffs") or []:
        if isinstance(handoff, dict) and handoff.get("transition_edge") == TRUE_TRANSITION_EDGE:
            return handoff
    raise ValueError(f"No connector handoff for {TRUE_TRANSITION_EDGE} found in PID runtime input")


def assess_query(query_id: str) -> dict[str, Any]:
    paths = route_paths(query_id)
    exists = {key: value.is_file() for key, value in paths.items()}
    if not exists["route_result"] or not exists["pid_input"]:
        return {
            "query_id": query_id,
            "available": False,
            "suitable": False,
            "exists": exists,
            "reasons": ["route_result or pid_input is missing"],
        }

    route_result = read_json(paths["route_result"])
    pid_input = read_json(paths["pid_input"])
    waypoints = pid_input.get("runtime_waypoints") or []
    floors = floor_counts(waypoints)
    edges = transition_edges(route_result, pid_input)
    goal_id = selected_goal_id(pid_input)
    query_type = str((pid_input.get("identity") or {}).get("query_type") or "")
    validation = route_result.get("validation") or {}
    reasons: list[str] = []
    if floors.get("floor_1", 0) <= 0:
        reasons.append("floor_1 has no runtime waypoints")
    if floors.get("floor_2", 0) <= 0:
        reasons.append("floor_2 has no runtime waypoints")
    if TRUE_TRANSITION_EDGE not in edges:
        reasons.append(f"{TRUE_TRANSITION_EDGE} is not selected as a transition edge")
    if FORBIDDEN_NON_TRANSITION_EDGE in edges:
        reasons.append(f"{FORBIDDEN_NON_TRANSITION_EDGE} appears as an active transition edge")
    if goal_id == BLOCKED_CANDIDATE_ID:
        reasons.append(f"{BLOCKED_CANDIDATE_ID} is selected as the runtime goal")
    if query_type == "cross_floor_object" and goal_id != SELECTED_APPROACH_ID:
        reasons.append(f"object query does not select {SELECTED_APPROACH_ID}")
    if not bool(validation.get("route_feasible")):
        reasons.append("RouteResult validation route_feasible is not true")
    if not reasons:
        reasons.append("all scripted stair route-selection checks passed")
    return {
        "query_id": query_id,
        "available": True,
        "suitable": len(reasons) == 1 and reasons[0].startswith("all "),
        "exists": exists,
        "floor_counts": floors,
        "transition_edges": edges,
        "selected_goal_candidate_id": goal_id,
        "query_type": query_type,
        "route_feasible": bool(validation.get("route_feasible")),
        "blocked_candidate_ids": rejected_candidate_ids(route_result, pid_input),
        "reasons": reasons,
    }


def normalize_source_waypoint(
    raw: dict[str, Any],
    *,
    composite_index: int,
    composite_segment: str,
    executor_type: str,
    z: float,
) -> dict[str, Any]:
    return {
        "index": composite_index,
        "source_index": int(raw.get("waypoint_index", raw.get("index", composite_index))),
        "x": round(as_float(raw.get("x")), 6),
        "y": round(as_float(raw.get("y")), 6),
        "z": round(z, 6),
        "yaw": round(as_float(raw.get("yaw")), 6),
        "floor_id": str(raw.get("floor_id") or ""),
        "segment_id": str(raw.get("segment_id") or ""),
        "composite_segment": composite_segment,
        "executor_type": executor_type,
        "semantic_source": str(raw.get("source") or raw.get("semantic_source") or "route_result_segment_waypoint"),
        "scripted_transition": False,
        "source_edge_id": None,
        "waypoint_index": composite_index,
        "source": str(raw.get("source") or "route_result_segment_waypoint"),
    }


def make_keyframes(
    handoff: dict[str, Any],
    *,
    start_yaw: float,
    end_yaw: float,
    start_index: int,
    step_count: int,
    seconds_per_keyframe: float,
) -> list[dict[str, Any]]:
    start = {
        "x": as_float((handoff.get("start") or {}).get("x")),
        "y": as_float((handoff.get("start") or {}).get("y")),
        "z": FLOOR_1_Z,
    }
    end = {
        "x": as_float((handoff.get("end") or {}).get("x")),
        "y": as_float((handoff.get("end") or {}).get("y")),
        "z": FLOOR_2_Z,
    }
    heading_yaw = yaw_between(start, end)
    keyframes: list[dict[str, Any]] = []

    def interpolate(progress: float, z: float, yaw: float) -> dict[str, float]:
        return {
            "x": start["x"] + (end["x"] - start["x"]) * progress,
            "y": start["y"] + (end["y"] - start["y"]) * progress,
            "z": z,
            "yaw": yaw,
        }

    keyframe_number = 0
    for step in range(step_count + 1):
        progress = step / float(step_count)
        z = FLOOR_1_Z + (FLOOR_2_Z - FLOOR_1_Z) * progress
        yaw_progress = progress
        yaw = normalize_angle(start_yaw + normalize_angle(end_yaw - start_yaw) * yaw_progress)
        if step == 0:
            pose = interpolate(progress, z, yaw)
            phase = "stair_entry"
        else:
            previous_z = FLOOR_1_Z + (FLOOR_2_Z - FLOOR_1_Z) * ((step - 1) / float(step_count))
            tread_pose = interpolate(progress, previous_z, heading_yaw)
            keyframes.append(
                make_keyframe_record(
                    tread_pose,
                    composite_index=start_index + keyframe_number,
                    keyframe_number=keyframe_number,
                    step=step,
                    phase="tread",
                    time_sec=keyframe_number * seconds_per_keyframe,
                )
            )
            keyframe_number += 1
            pose = interpolate(progress, z, heading_yaw if step < step_count else yaw)
            phase = "riser" if step < step_count else "stair_exit"
        keyframes.append(
            make_keyframe_record(
                pose,
                composite_index=start_index + keyframe_number,
                keyframe_number=keyframe_number,
                step=step,
                phase=phase,
                time_sec=keyframe_number * seconds_per_keyframe,
            )
        )
        keyframe_number += 1
    return keyframes


def make_keyframe_record(
    pose: dict[str, float],
    *,
    composite_index: int,
    keyframe_number: int,
    step: int,
    phase: str,
    time_sec: float,
) -> dict[str, Any]:
    return {
        "index": composite_index,
        "waypoint_index": composite_index,
        "keyframe_index": keyframe_number,
        "visual_step_number": step,
        "stair_phase": phase,
        "time_sec": round(time_sec, 3),
        "x": round(pose["x"], 6),
        "y": round(pose["y"], 6),
        "z": round(pose["z"], 6),
        "yaw": round(pose["yaw"], 6),
        "floor_id": "transition",
        "segment_id": f"scripted_stair_{TRUE_TRANSITION_EDGE}",
        "composite_segment": "scripted_stair",
        "executor_type": "scripted_animation",
        "semantic_source": "scripted_stair_transition_keyframe",
        "scripted_transition": True,
        "source_edge_id": TRUE_TRANSITION_EDGE,
        "physical_stair_climbing_claim": False,
        "source": "scripted_stair_transition_keyframe",
    }


def split_runtime_waypoints(pid_input: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    raw_waypoints = [wp for wp in pid_input.get("runtime_waypoints") or [] if isinstance(wp, dict)]
    floor_1_raw = [wp for wp in raw_waypoints if wp.get("floor_id") == "floor_1"]
    floor_2_raw_all = [wp for wp in raw_waypoints if wp.get("floor_id") == "floor_2"]
    handoff = first_connector_handoff(pid_input)
    if not floor_1_raw or not floor_2_raw_all:
        raise ValueError("runtime input must contain both floor_1 and floor_2 waypoints")

    # The first floor_2 waypoint is the connector exit already represented by
    # the scripted stair keyframe, so floor_2 PID begins after that handoff.
    floor_2_raw = floor_2_raw_all[1:] if len(floor_2_raw_all) > 1 else floor_2_raw_all
    return floor_1_raw, floor_2_raw, handoff


def build_runtime_input(
    *,
    query_id: str,
    profile_id: str,
    stair_step_count: int,
    seconds_per_keyframe: float,
) -> dict[str, Any]:
    paths = route_paths(query_id)
    route_result = read_json(paths["route_result"])
    pid_input = read_json(paths["pid_input"])
    route_audit = assess_query(query_id)
    if not route_audit["suitable"]:
        raise ValueError(f"query is not suitable for task60 scripted stair demo: {route_audit['reasons']}")

    floor_1_raw, floor_2_raw, handoff = split_runtime_waypoints(pid_input)
    floor_1_segment: list[dict[str, Any]] = []
    floor_2_segment: list[dict[str, Any]] = []
    composite_index = 0
    for raw in floor_1_raw:
        floor_1_segment.append(
            normalize_source_waypoint(
                raw,
                composite_index=composite_index,
                composite_segment="floor_1",
                executor_type="gazebo_pid",
                z=FLOOR_1_Z,
            )
        )
        composite_index += 1

    floor_1_last = floor_1_segment[-1]
    floor_2_first = floor_2_raw[0]
    keyframes = make_keyframes(
        handoff,
        start_yaw=as_float(floor_1_last.get("yaw")),
        end_yaw=as_float(floor_2_first.get("yaw")),
        start_index=composite_index,
        step_count=stair_step_count,
        seconds_per_keyframe=seconds_per_keyframe,
    )
    composite_index += len(keyframes)

    for raw in floor_2_raw:
        floor_2_segment.append(
            normalize_source_waypoint(
                raw,
                composite_index=composite_index,
                composite_segment="floor_2",
                executor_type="gazebo_pid",
                z=FLOOR_2_Z,
            )
        )
        composite_index += 1

    selected_goal = pid_input.get("selected_goal") or pid_input.get("selected_approach") or {}
    source_route_summary = {
        "floor_sequence": (route_result.get("semantic_route") or {}).get("floor_sequence"),
        "room_sequence": (route_result.get("semantic_route") or {}).get("room_sequence"),
        "connector_sequence": (route_result.get("semantic_route") or {}).get("connector_sequence"),
        "route_feasible": (route_result.get("validation") or {}).get("route_feasible"),
    }
    runtime = {
        "schema_name": "rslg_scripted_stair_transition_runtime_input",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "scene_id": SCENE_ID,
        "source_query_id": query_id,
        "profile_id": profile_id,
        "source_route_result_json": str(paths["route_result"]),
        "source_pid_input_json": str(paths["pid_input"]),
        "source_rviz_marker_input_json": str(paths["rviz_marker_input"]),
        "source_z_aware_overlay_input_json": str(paths["z_aware_overlay_input"]),
        "route_mode": "scripted_stair_transition",
        "transition_edge_id": TRUE_TRANSITION_EDGE,
        "transition_executor_type": "scripted_animation",
        "physical_stair_climbing_claim": False,
        "floor_1_z": FLOOR_1_Z,
        "floor_2_z": FLOOR_2_Z,
        "floor_z_map": {"floor_1": FLOOR_1_Z, "floor_2": FLOOR_2_Z},
        "source_route_summary": source_route_summary,
        "route_audit": route_audit,
        "connector_handoff": handoff,
        "stair_transition": {
            "source_edge_id": TRUE_TRANSITION_EDGE,
            "executor_type": "scripted_animation",
            "keyframe_count": len(keyframes),
            "step_count": stair_step_count,
            "seconds_per_keyframe": seconds_per_keyframe,
            "start_pose": keyframes[0],
            "exit_pose": keyframes[-1],
            "scripted_transition": True,
            "physical_stair_climbing_claim": False,
        },
        "floor_1_segment": {
            "executor_type": "gazebo_pid",
            "waypoint_count": len(floor_1_segment),
            "start_pose": floor_1_segment[0],
            "end_pose": floor_1_segment[-1],
            "waypoints": floor_1_segment,
        },
        "scripted_stair_keyframes": keyframes,
        "floor_2_segment": {
            "executor_type": "gazebo_pid",
            "waypoint_count": len(floor_2_segment),
            "start_pose": floor_2_segment[0],
            "end_pose": floor_2_segment[-1],
            "waypoints": floor_2_segment,
        },
        "final_target": {
            "candidate_id": selected_goal.get("candidate_id") or SELECTED_APPROACH_ID,
            "target_approach_id": SELECTED_APPROACH_ID,
            "category": "curtain",
            "room_id": "room_14",
            "floor_id": "floor_2",
            "x": as_float(selected_goal.get("x", selected_goal.get("world_xy", [None, None])[0])),
            "y": as_float(selected_goal.get("y", selected_goal.get("world_xy", [None, None])[1])),
            "z": FLOOR_2_Z,
            "yaw": as_float(selected_goal.get("yaw")),
            "clearance_m": as_float(selected_goal.get("clearance_m"), 0.2),
        },
        "target_approach_id": SELECTED_APPROACH_ID,
        "selected_goal_candidate_id": selected_goal.get("candidate_id") or SELECTED_APPROACH_ID,
        "forbidden_transition_edge_ids": [FORBIDDEN_NON_TRANSITION_EDGE],
        "blocked_goal_candidate_ids": [BLOCKED_CANDIDATE_ID],
        "topics": TOPICS,
        "frame_id": "odom",
        "gazebo_world": str(REPO_ROOT / WORLD_RELATIVE_PATH),
        "rviz_config": str(REPO_ROOT / RVIZ_RELATIVE_PATH),
        "waypoint_counts": {
            "floor_1": len(floor_1_segment),
            "scripted_stair_keyframes": len(keyframes),
            "floor_2": len(floor_2_segment),
            "composite_total": len(floor_1_segment) + len(keyframes) + len(floor_2_segment),
            "source_floor_2_waypoints_including_connector_exit": len(floor_2_raw) + 1,
        },
        "guards": {
            "transition_edge_id": TRUE_TRANSITION_EDGE,
            "forbidden_transition_edge_ids": [FORBIDDEN_NON_TRANSITION_EDGE],
            "vt_1_centerline_e003_used_as_transition": False,
            "blocked_goal_candidate_ids": [BLOCKED_CANDIDATE_ID],
            "generated_ring_037_selected_as_runtime_goal": False,
            "generated_ring_002_selected_as_runtime_goal": True,
            "requires_nav2": False,
            "requires_amcl": False,
            "requires_map_server": False,
            "physical_stair_climbing_claim": False,
            "real_robot_deployment_claim": False,
            "global_collision_free_guarantee_claim": False,
        },
        "claim_boundary": {
            "semantic_topological_route": True,
            "gazebo_pid_flat_floor_segments": True,
            "scripted_stair_transition_animation": True,
            "physical_stair_climbing_claimed": False,
            "physical_robot_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
            "global_collision_free_guarantee_claimed": False,
        },
    }
    runtime["composite_waypoints"] = [
        *runtime["floor_1_segment"]["waypoints"],
        *runtime["scripted_stair_keyframes"],
        *runtime["floor_2_segment"]["waypoints"],
    ]
    runtime["composite_path_length_xy"] = round(
        sum(distance_xy(a, b) for a, b in zip(route_points(runtime), route_points(runtime)[1:])),
        6,
    )
    return runtime


def markdown_pose(raw: dict[str, Any]) -> str:
    return f"({as_float(raw.get('x')):.6f}, {as_float(raw.get('y')):.6f}, {as_float(raw.get('z')):.6f}, yaw={as_float(raw.get('yaw')):.6f})"


def write_route_audit(task_dir: Path, runtime: dict[str, Any]) -> None:
    audit = runtime["route_audit"]
    floor_1 = runtime["floor_1_segment"]
    floor_2 = runtime["floor_2_segment"]
    stair = runtime["stair_transition"]
    final_target = runtime["final_target"]
    lines = [
        "# Cross-Floor Route Segment Audit",
        "",
        f"- selected query: `{runtime['source_query_id']}`",
        f"- source RouteResult: `{runtime['source_route_result_json']}`",
        f"- source PID/runtime input: `{runtime['source_pid_input_json']}`",
        f"- includes floor_1: `{audit['floor_counts'].get('floor_1', 0) > 0}`",
        f"- includes floor_2: `{audit['floor_counts'].get('floor_2', 0) > 0}`",
        f"- connector transition edge: `{runtime['transition_edge_id']}`",
        f"- forbidden non-transition edge avoided: `{FORBIDDEN_NON_TRANSITION_EDGE}`",
        f"- floor_1 waypoints: `{floor_1['waypoint_count']}`",
        f"- floor_2 waypoints after connector exit: `{floor_2['waypoint_count']}`",
        f"- scripted stair keyframes: `{stair['keyframe_count']}`",
        f"- stair entrance pose: `{markdown_pose(stair['start_pose'])}`",
        f"- stair exit pose: `{markdown_pose(stair['exit_pose'])}`",
        f"- final target pose: `({final_target['x']:.6f}, {final_target['y']:.6f}, {final_target['z']:.6f}, yaw={final_target['yaw']:.6f})`",
        f"- final approach: `{runtime['target_approach_id']}`",
        f"- generated_ring_037 remains blocked/rejected evidence only: `{not runtime['guards']['generated_ring_037_selected_as_runtime_goal']}`",
        f"- vt_1_centerline_e003 used as transition: `{runtime['guards']['vt_1_centerline_e003_used_as_transition']}`",
        "",
        "What is executed physically in Gazebo:",
        "- floor_1 waypoints are followed with the TurtleBot3 PID /cmd_vel interface on the z=0.0 flat platform.",
        "- floor_2 waypoints are followed with the TurtleBot3 PID /cmd_vel interface on the z=1.6 flat platform when the Gazebo model-state transition is available.",
        "",
        "What is animated/scripted:",
        f"- `{TRUE_TRANSITION_EDGE}` is represented by scripted stair keyframes from the entrance pose to the exit pose.",
        "- The stair segment is visual/runtime-interface animation, not a physical stair-climbing validation.",
        "",
    ]
    write_text(task_dir / "03_cross_floor_route_segment_audit.md", "\n".join(lines))


def write_runtime_design(task_dir: Path, runtime: dict[str, Any]) -> None:
    lines = [
        "# Runtime Input Design",
        "",
        "The task60 runtime input is a segment-preserving adapter over the task56c cross-floor route.",
        "",
        f"- schema: `{runtime['schema_name']}@{runtime['schema_version']}`",
        f"- selected query: `{runtime['source_query_id']}`",
        f"- route mode: `{runtime['route_mode']}`",
        f"- transition edge: `{runtime['transition_edge_id']}`",
        f"- transition executor: `{runtime['transition_executor_type']}`",
        f"- physical stair-climbing claim: `{runtime['physical_stair_climbing_claim']}`",
        f"- floor_1 z: `{runtime['floor_1_z']}`",
        f"- floor_2 z: `{runtime['floor_2_z']}`",
        f"- floor_1 waypoint count: `{runtime['waypoint_counts']['floor_1']}`",
        f"- scripted stair keyframe count: `{runtime['waypoint_counts']['scripted_stair_keyframes']}`",
        f"- floor_2 waypoint count: `{runtime['waypoint_counts']['floor_2']}`",
        f"- composite point count: `{runtime['waypoint_counts']['composite_total']}`",
        f"- final approach: `{runtime['target_approach_id']}`",
        "",
        "Schema fields of interest:",
        "- `floor_1_segment`: flat Gazebo PID waypoints with `executor_type=gazebo_pid`.",
        "- `scripted_stair_keyframes`: step-wise XY/Z keyframes marked `scripted_transition=true` and `source_edge_id=vt_1_centerline_e001`.",
        "- `floor_2_segment`: flat Gazebo PID waypoints after the connector exit.",
        "- `guards`: active boundary flags for forbidden transition and blocked goal candidates.",
        "- `topics`: composite planned/executed path, robot pose, stair transition path, and marker topics in frame `odom`.",
        "",
        "Guard decisions:",
        f"- `{FORBIDDEN_NON_TRANSITION_EDGE}` is listed only under `forbidden_transition_edge_ids`.",
        f"- `{BLOCKED_CANDIDATE_ID}` is listed only under `blocked_goal_candidate_ids`.",
        f"- `{SELECTED_APPROACH_ID}` remains the selected object approach.",
        "",
    ]
    write_text(task_dir / "06_runtime_input_design.md", "\n".join(lines))


def helper_script_header() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail

# Robust RSLG-SLAM repository-root discovery.
# Walk upward from this script's directory until a directory that contains both
# tools/rslg_pipeline and stage_outputs is found. This avoids fragile fixed-depth
# "../../.." traversal (which previously resolved to /home/ws/workspace instead of
# /home/ws/workspace/BoxFusion and was further corrupted by a bad sed edit).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
while [[ "${REPO_ROOT}" != "/" ]]; do
  if [[ -d "${REPO_ROOT}/tools/rslg_pipeline" && -d "${REPO_ROOT}/stage_outputs" ]]; then
    break
  fi
  REPO_ROOT="$(dirname "${REPO_ROOT}")"
done
if [[ ! -d "${REPO_ROOT}/tools/rslg_pipeline" ]]; then
  echo "Could not locate RSLG-SLAM repo root from ${SCRIPT_DIR}" >&2
  exit 1
fi

TASK_DIR="${REPO_ROOT}/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task60_scripted_stair_transition_gazebo_rviz_demo"
QUERY_ID="${RSLG_QUERY_ID:-00843_cross_floor_object_curtain_room14}"
RUNTIME_INPUT="${TASK_DIR}/scripted_stair_pack/runtime_inputs/${QUERY_ID}_scripted_stair_transition_runtime_input.json"
PROFILE_JSON="${REPO_ROOT}/configs/rslg_runtime_profiles/pid_profiles_v0_1.json"
WORLD="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds/rslg_scripted_stair_transition_turtlebot3_burger.world"
RVIZ_CONFIG="${REPO_ROOT}/tools/rslg_pipeline/rviz/config/rslg_scripted_stair_transition_showcase.rviz"
ORCHESTRATOR="${REPO_ROOT}/tools/rslg_pipeline/gazebo/rslg_gazebo_scripted_stair_demo_orchestrator.py"
MAIN_DEMO_SCRIPT="${REPO_ROOT}/tools/rslg_pipeline/gazebo/run_scripted_stair_transition_demo.sh"
PYTHON="${RSLG_TOOL_PYTHON:-/home/ws/miniconda3/envs/boxfusion/bin/python}"
ROS_PYTHON="${RSLG_ROS_PYTHON:-/usr/bin/python3}"

rslg_print_paths() {
  echo "SCRIPT_NAME=$(basename "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")"
  echo "SCRIPT_DIR=${SCRIPT_DIR}"
  echo "REPO_ROOT=${REPO_ROOT}"
  echo "TASK_DIR=${TASK_DIR}"
  echo "QUERY_ID=${QUERY_ID}"
  echo "RUNTIME_INPUT=${RUNTIME_INPUT}"
  echo "PROFILE_JSON=${PROFILE_JSON}"
  echo "WORLD=${WORLD}"
  echo "RVIZ_CONFIG=${RVIZ_CONFIG}"
  echo "ORCHESTRATOR=${ORCHESTRATOR}"
  echo "MAIN_DEMO_SCRIPT=${MAIN_DEMO_SCRIPT}"
}

if [[ "${1:-}" == "--print-paths" ]]; then
  rslg_print_paths
  exit 0
fi
"""


def write_helper_scripts(pack_dir: Path) -> None:
    scripts = {
        "run_dry_run.sh": helper_script_header()
        + """
cd "${REPO_ROOT}"
exec "${MAIN_DEMO_SCRIPT}" --dry-run --query-id "${QUERY_ID}" --rviz-only-stair-fallback "$@"
""",
        "run_full_demo_headless_or_service.sh": helper_script_header()
        + """
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck source=/opt/ros/foxy/setup.bash
  source /opt/ros/foxy/setup.bash
fi
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # shellcheck source=/usr/share/gazebo/setup.sh
  source /usr/share/gazebo/setup.sh
fi
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
cd "${REPO_ROOT}"
exec "${MAIN_DEMO_SCRIPT}" --headless --no-rviz-gui --rviz-only-stair-fallback --duration-sec "${RSLG_SCRIPTED_STAIR_DURATION_SEC:-360}" --query-id "${QUERY_ID}" "$@"
""",
        "run_scripted_stair_node.sh": helper_script_header()
        + """
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck source=/opt/ros/foxy/setup.bash
  source /opt/ros/foxy/setup.bash
fi
cd "${REPO_ROOT}"
RUN_DIR="${TASK_DIR}/scripted_stair_pack/manual_runs/manual_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "${RUN_DIR}"
exec "${ROS_PYTHON}" "${ORCHESTRATOR}" \\
  --runtime-input-json "${RUNTIME_INPUT}" \\
  --profile-json "${PROFILE_JSON}" \\
  --profile-id "${RSLG_PROFILE_ID:-practical_zero_collision}" \\
  --output-dir "${RUN_DIR}" \\
  --frame-id odom \\
  --duration-sec "${RSLG_SCRIPTED_STAIR_DURATION_SEC:-360}" \\
  --anchor-first-waypoint-to-odom-start \\
  --rviz-only-stair-fallback \\
  --stop-at-end "$@"
""",
        "run_gazebo_gui.sh": helper_script_header()
        + """
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck source=/opt/ros/foxy/setup.bash
  source /opt/ros/foxy/setup.bash
elif [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_gazebo_gui] warning: ROS 2 is not sourced and /opt/ros/foxy/setup.bash was not found." >&2
fi
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # shellcheck source=/usr/share/gazebo/setup.sh
  source /usr/share/gazebo/setup.sh
fi
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/models:/opt/ros/foxy/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${REPO_ROOT}/tools/rslg_pipeline/gazebo/worlds:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
if [[ ! -f "${WORLD}" ]]; then
  echo "[run_gazebo_gui] error: world file not found: ${WORLD}" >&2
  exit 1
fi
cd "${REPO_ROOT}"
exec gazebo --verbose "${WORLD}" "$@"
""",
        "run_rviz_gui.sh": helper_script_header()
        + """
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck source=/opt/ros/foxy/setup.bash
  source /opt/ros/foxy/setup.bash
elif [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[run_rviz_gui] warning: ROS 2 is not sourced and /opt/ros/foxy/setup.bash was not found." >&2
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "[run_rviz_gui] error: rviz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi
cd "${REPO_ROOT}"
exec rviz2 -d "${RVIZ_CONFIG}" "$@"
""",
    }
    pack_dir.mkdir(parents=True, exist_ok=True)
    for name, content in scripts.items():
        path = pack_dir / name
        write_text(path, content.lstrip())
        path.chmod(path.stat().st_mode | 0o111)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query-id", default=DEFAULT_QUERY_ID)
    parser.add_argument("--fallback-query-id", default=FALLBACK_QUERY_ID)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--task-dir", type=Path, default=runtime_task_dir(REPO_ROOT))
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--stair-step-count", type=int, default=16)
    parser.add_argument("--seconds-per-keyframe", type=float, default=0.35)
    parser.add_argument("--write-audits", action="store_true")
    parser.add_argument("--write-helper-scripts", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    selected_query = args.query_id
    selected_audit = assess_query(selected_query)
    if not selected_audit["suitable"] and args.fallback_query_id:
        fallback_audit = assess_query(args.fallback_query_id)
        if fallback_audit["suitable"]:
            selected_query = args.fallback_query_id
        else:
            raise SystemExit(
                "Primary query and fallback query are not suitable: "
                + json.dumps({"primary": selected_audit, "fallback": fallback_audit}, indent=2)
            )
    elif not selected_audit["suitable"]:
        raise SystemExit("Selected query is not suitable: " + json.dumps(selected_audit, indent=2))

    runtime = build_runtime_input(
        query_id=selected_query,
        profile_id=args.profile_id,
        stair_step_count=max(2, int(args.stair_step_count)),
        seconds_per_keyframe=max(0.01, float(args.seconds_per_keyframe)),
    )
    output_json = args.output_json or runtime_input_path(REPO_ROOT, selected_query)
    write_json(output_json, runtime)
    if args.write_audits:
        args.task_dir.mkdir(parents=True, exist_ok=True)
        write_route_audit(args.task_dir, runtime)
        write_runtime_design(args.task_dir, runtime)
    if args.write_helper_scripts:
        write_helper_scripts(args.task_dir / "scripted_stair_pack")
    print(json.dumps({"status": "ok", "runtime_input_json": str(output_json), "query_id": selected_query}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
