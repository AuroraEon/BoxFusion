#!/usr/bin/env python3
"""Post-restructure Scene route runner using FollowPath plus forward-only fallback."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


os.environ.setdefault("ROS_DOMAIN_ID", "84")
os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
    Node = object  # type: ignore[assignment]


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_runtime"
ALLOWED_GOAL_SOURCES = {
    "room_center",
    "gateway",
    "segment_start",
    "segment_crossing",
    "segment_goal",
    "snapped_segment_waypoint",
    "target_point_terminal",
    "semantic_target_terminal",
}
FORBIDDEN_NAV_GOAL_SOURCES = {"bridge", "smooth_bridge", "current_pose_path_anchor"}
EXPECTED_ROOM_CHAIN: list[str] = []
EXPECTED_GATEWAY_SEQUENCE: list[str] = []
TRACE_FILENAME = "route_execution_trace_v0_1.jsonl"
SEMANTIC_STATUS_FILENAME = "semantic_goal_status_v0_1.json"
DEVIATION_EVENTS_FILENAME = "deviation_events_v0_1.jsonl"
FOLLOW_PATH_ABORT_CONTEXT_FILENAME = "follow_path_abort_context_v0_1.json"
SPLIT_STRATEGY = "split_follow_path_with_through_room_dwell"
HANDOFF_STRATEGY = "split_follow_path_with_current_pose_gateway_handoff"

try:
    from scene_runtime_common import default_runtime_profile, load_nav_map, load_runtime_profile, map_value, resolve_path
except Exception:  # pragma: no cover
    default_runtime_profile = None
    load_nav_map = None
    load_runtime_profile = None
    map_value = None
    resolve_path = None

try:
    from validate_scene_trajectory_wall_crossing import validate_point_sequence, validate_samples
except Exception:  # pragma: no cover
    validate_point_sequence = None
    validate_samples = None


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def append_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Step30S2 Scene Route Rerun Report",
        "",
        f"Created: `{payload.get('created_utc')}`",
        f"Attempted: `{payload.get('execute_attempted')}`",
        f"Succeeded: `{payload.get('succeeded')}`",
        f"Execution strategy: `{payload.get('execution_strategy')}`",
        f"FollowPath used: `{payload.get('follow_path_used')}`",
        f"Sparse fallback used: `{payload.get('sparse_fallback_used')}`",
        f"Fallback start selected index: `{payload.get('fallback_started_at_selected_list_index')}`",
        f"Fallback start waypoint index: `{payload.get('fallback_started_at_waypoint_index')}`",
        f"Robot moved distance: `{payload.get('robot_moved_distance_m')}` m",
    ]
    if payload.get("failure_layer"):
        lines.append(f"Failure layer: `{payload.get('failure_layer')}`")
    if payload.get("failure_reason"):
        lines.append(f"Failure reason: `{payload.get('failure_reason')}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def yaw_to_quat(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def status_name(status: int | None) -> str | None:
    if status is None or IMPORT_ERROR:
        return None
    return {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }.get(int(status), str(status))


def pose_distance(pose: dict[str, Any] | None, waypoint: dict[str, Any] | None) -> float | None:
    if not pose or not waypoint:
        return None
    return math.hypot(float(pose["x"]) - float(waypoint["x"]), float(pose["y"]) - float(waypoint["y"]))


def round_float(value: Any, digits: int = 6) -> float | None:
    if value is None:
        return None
    try:
        if math.isnan(float(value)):
            return None
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def ros_stamp_summary(stamp: Any) -> dict[str, int] | None:
    if stamp is None:
        return None
    return {
        "sec": int(getattr(stamp, "sec", 0)),
        "nanosec": int(getattr(stamp, "nanosec", 0)),
    }


def local_costmap_summary(msg: Any, robot_pose: dict[str, Any] | None, *, received_wall_sec: float | None) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "captured": False,
        "capture_timing": "follow_path_abort_immediate",
        "robot_pose_at_capture": pose_summary(robot_pose),
        "received_wall_sec": round_float(received_wall_sec, 6),
        "reason_if_missing": None,
    }
    if msg is None:
        payload["reason_if_missing"] = "no /local_costmap/costmap OccupancyGrid sample was available at FollowPath abort time"
        return payload
    data = list(msg.data)
    unknown = sum(1 for value in data if int(value) < 0)
    free = sum(1 for value in data if int(value) == 0)
    occupied = sum(1 for value in data if int(value) >= 90)
    inflated = sum(1 for value in data if 1 <= int(value) < 90)
    payload.update({
        "captured": True,
        "message_stamp": ros_stamp_summary(getattr(msg.header, "stamp", None)),
        "metadata": {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
        },
        "cell_summary": {
            "total": len(data),
            "free": free,
            "inflated_or_low_cost": inflated,
            "occupied": occupied,
            "unknown": unknown,
        },
    })
    return payload


def nearest_path_index(path_slice: list[dict[str, Any]], current: dict[str, Any] | None) -> tuple[int, float | None]:
    if current is None or not path_slice:
        return 0, None
    distances = [(idx, pose_distance(current, waypoint)) for idx, waypoint in enumerate(path_slice)]
    distances = [(idx, dist) for idx, dist in distances if dist is not None]
    if not distances:
        return 0, None
    idx, dist = min(distances, key=lambda item: item[1])
    return idx, float(dist)


def angle_delta_abs(a: float, b: float) -> float:
    return abs((b - a + math.pi) % (2.0 * math.pi) - math.pi)


def heading_between(a: dict[str, Any], b: dict[str, Any]) -> float | None:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    if math.hypot(dx, dy) <= 1e-9:
        return None
    return math.atan2(dy, dx)


def path_length(points: list[dict[str, Any]]) -> float:
    return sum(
        math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        for a, b in zip(points, points[1:])
    )


def pose_summary(point: dict[str, Any] | None) -> dict[str, float] | None:
    if not point:
        return None
    result = {"x": float(point["x"]), "y": float(point["y"])}
    if point.get("yaw") is not None:
        result["yaw"] = float(point["yaw"])
    return result


def target_room_for_metadata(waypoint: dict[str, Any] | None) -> str | None:
    if not waypoint:
        return None
    if waypoint.get("source") == "gateway":
        return waypoint.get("to_room") or waypoint.get("from_room")
    return waypoint_room(waypoint)


def target_metadata(waypoint: dict[str, Any] | None) -> dict[str, Any]:
    if not waypoint:
        return {
            "target_waypoint_index": None,
            "target_source": None,
            "target_room": None,
            "target_gateway_id": None,
            "gateway_id": None,
            "from_room": None,
            "to_room": None,
        }
    gateway_id = waypoint.get("gateway_id")
    return {
        "target_waypoint_index": int(waypoint["waypoint_index"]) if waypoint.get("waypoint_index") is not None else None,
        "target_source": waypoint.get("source"),
        "target_room": target_room_for_metadata(waypoint),
        "target_gateway_id": gateway_id,
        "gateway_id": gateway_id,
        "from_room": waypoint.get("from_room"),
        "to_room": waypoint.get("to_room"),
    }


def path_prefix_to_distance(points: list[dict[str, Any]], target_distance_m: float, min_distance_m: float, max_distance_m: float) -> tuple[list[dict[str, Any]], float]:
    if len(points) <= 2:
        return list(points), path_length(points)
    selected_index = len(points) - 1
    travelled = 0.0
    best_index = 1
    best_error = float("inf")
    for idx, (a, b) in enumerate(zip(points, points[1:]), start=1):
        travelled += math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        if travelled <= max_distance_m:
            error = abs(travelled - target_distance_m)
            if travelled >= min_distance_m and error < best_error:
                best_error = error
                best_index = idx
        if travelled >= target_distance_m and travelled >= min_distance_m:
            selected_index = idx
            break
    else:
        selected_index = best_index if best_error < float("inf") else len(points) - 1
    prefix = list(points[:selected_index + 1])
    return prefix, path_length(prefix)


def first_meter_points(points: list[dict[str, Any]], limit_m: float = 1.0) -> list[dict[str, Any]]:
    if not points:
        return []
    selected = [points[0]]
    travelled = 0.0
    for a, b in zip(points, points[1:]):
        segment = math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        travelled += segment
        selected.append(b)
        if travelled >= limit_m:
            break
    return selected


def first_meter_heading_summary(points: list[dict[str, Any]], start_pose: dict[str, Any] | None = None) -> dict[str, Any]:
    subset = first_meter_points(points)
    headings = [
        heading_between(a, b)
        for a, b in zip(subset, subset[1:])
        if heading_between(a, b) is not None
    ]
    headings = [float(value) for value in headings if value is not None]
    summary: dict[str, Any] = {
        "sample_count": len(headings),
        "first_heading_rad": round_float(headings[0], 6) if headings else None,
        "last_heading_rad": round_float(headings[-1], 6) if headings else None,
    }
    if start_pose and start_pose.get("yaw") is not None and headings:
        summary["start_yaw_rad"] = round_float(start_pose.get("yaw"), 6)
        summary["start_yaw_to_first_heading_delta_rad"] = round_float(angle_delta_abs(float(start_pose["yaw"]), headings[0]), 6)
    return summary


def clearance_summary(points: list[dict[str, Any]], map_yaml: Path | None) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "available": False,
        "sample_count": 0,
        "min_clearance_m": None,
        "first_meter_min_clearance_m": None,
        "reason": None,
    }
    if not map_yaml:
        summary["reason"] = "no map yaml supplied"
        return summary
    if load_nav_map is None or map_value is None:
        summary["reason"] = "scene_runtime_common map helpers unavailable"
        return summary
    try:
        grid, resolution, origin, _meta = load_nav_map(Path(map_yaml))
    except Exception as exc:
        summary["reason"] = f"{type(exc).__name__}: {exc}"
        return summary
    occupied = grid <= 100
    try:
        from scipy import ndimage  # type: ignore

        distance_grid = ndimage.distance_transform_edt(~occupied) * float(resolution)
    except Exception as exc:
        summary["reason"] = f"scipy distance transform unavailable: {type(exc).__name__}: {exc}"
        return summary

    def point_clearance(point: dict[str, Any]) -> float | None:
        col = int(round((float(point["x"]) - origin[0]) / resolution))
        row = int(round((float(point["y"]) - origin[1]) / resolution))
        if 0 <= row < distance_grid.shape[0] and 0 <= col < distance_grid.shape[1]:
            return float(distance_grid[row, col])
        return None

    clearances = [point_clearance(point) for point in points]
    first_clearances = [point_clearance(point) for point in first_meter_points(points)]
    clearances = [value for value in clearances if value is not None]
    first_clearances = [value for value in first_clearances if value is not None]
    summary.update({
        "available": True,
        "sample_count": len(clearances),
        "min_clearance_m": round_float(min(clearances), 6) if clearances else None,
        "first_meter_min_clearance_m": round_float(min(first_clearances), 6) if first_clearances else None,
        "reason": None,
    })
    return summary


def min_nonadjacent_branch_distance(
    waypoints: list[dict[str, Any]],
    anchor_idx: int,
    *,
    window: int = 8,
) -> tuple[float | None, tuple[int, int] | None]:
    best: float | None = None
    best_pair: tuple[int, int] | None = None
    left_start = max(0, anchor_idx - window)
    right_stop = min(len(waypoints), anchor_idx + window + 1)
    for left in range(left_start, anchor_idx):
        for right in range(anchor_idx + 1, right_stop):
            if abs(right - left) <= 1:
                continue
            dist = math.hypot(
                float(waypoints[left]["x"]) - float(waypoints[right]["x"]),
                float(waypoints[left]["y"]) - float(waypoints[right]["y"]),
            )
            if best is None or dist < best:
                best = dist
                best_pair = (left, right)
    return best, best_pair


def detect_split_anchors(
    waypoints: list[dict[str, Any]],
    *,
    heading_reversal_threshold_deg: float = 135.0,
    branch_close_threshold_m: float = 0.30,
) -> list[dict[str, Any]]:
    """Detect room-center anchors that should gate folded through-room execution."""
    semantic = [
        (idx, waypoint)
        for idx, waypoint in enumerate(waypoints)
        if waypoint.get("source") in ALLOWED_GOAL_SOURCES
    ]
    semantic_position_by_idx = {idx: pos for pos, (idx, _waypoint) in enumerate(semantic)}
    anchors: list[dict[str, Any]] = []
    for idx, waypoint in enumerate(waypoints):
        if waypoint.get("source") != "room_center" or idx <= 0 or idx >= len(waypoints) - 1:
            continue
        semantic_pos = semantic_position_by_idx.get(idx)
        prev_semantic = semantic[semantic_pos - 1][1] if semantic_pos is not None and semantic_pos > 0 else None
        next_semantic = semantic[semantic_pos + 1][1] if semantic_pos is not None and semantic_pos + 1 < len(semantic) else None
        between_gateways = bool(
            prev_semantic
            and next_semantic
            and prev_semantic.get("source") == "gateway"
            and next_semantic.get("source") == "gateway"
        )
        inbound = heading_between(waypoints[idx - 1], waypoint)
        outbound = heading_between(waypoint, waypoints[idx + 1])
        reversal_deg = math.degrees(angle_delta_abs(inbound, outbound)) if inbound is not None and outbound is not None else None
        close_dist, close_pair = min_nonadjacent_branch_distance(waypoints, idx)
        fold_like = bool(
            (reversal_deg is not None and reversal_deg >= heading_reversal_threshold_deg)
            or (close_dist is not None and close_dist <= branch_close_threshold_m)
        )
        candidate_reasons = []
        if between_gateways:
            candidate_reasons.append("room_center_between_gateways")
        if reversal_deg is not None and reversal_deg >= heading_reversal_threshold_deg:
            candidate_reasons.append("heading_reversal_around_anchor")
        if close_dist is not None and close_dist <= branch_close_threshold_m:
            candidate_reasons.append("close_nonadjacent_enter_return_branches")
        if between_gateways and fold_like:
            anchors.append({
                "waypoint_index": int(waypoint["waypoint_index"]),
                "slice_index": idx,
                "source": waypoint.get("source"),
                "room": waypoint_room(waypoint),
                "x": float(waypoint["x"]),
                "y": float(waypoint["y"]),
                "previous_semantic_waypoint_index": int(prev_semantic["waypoint_index"]) if prev_semantic else None,
                "previous_gateway_id": prev_semantic.get("gateway_id") if prev_semantic else None,
                "next_semantic_waypoint_index": int(next_semantic["waypoint_index"]) if next_semantic else None,
                "next_gateway_id": next_semantic.get("gateway_id") if next_semantic else None,
                "heading_reversal_deg": round_float(reversal_deg, 3),
                "closest_nonadjacent_branch_distance_m": round_float(close_dist, 6),
                "closest_nonadjacent_branch_pair": list(close_pair) if close_pair else None,
                "detection_reasons": candidate_reasons,
            })
    return anchors


def split_slices_for_anchors(path_start_idx: int, waypoints: list[dict[str, Any]], split_anchors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    stops = [
        int(anchor["slice_index"])
        for anchor in split_anchors
        if path_start_idx < int(anchor["slice_index"]) < len(waypoints) - 1
    ]
    stops.append(len(waypoints) - 1)
    slices: list[dict[str, Any]] = []
    start = path_start_idx
    for slice_index, stop in enumerate(stops):
        if stop < start:
            continue
        target = waypoints[stop]
        slices.append({
            "slice_index": slice_index,
            "path_start_slice_index": start,
            "path_stop_slice_index": stop,
            "target_waypoint_index": int(target["waypoint_index"]),
            "target_source": target.get("source"),
            "target_room": waypoint_room(target),
            "waypoint_indices": [int(w["waypoint_index"]) for w in waypoints[start:stop + 1]],
        })
        start = stop
    return slices


def semantic_targets_for_slice(semantic_goals: list[dict[str, Any]], slice_start: int, slice_stop: int) -> list[dict[str, Any]]:
    targets = [
        goal for goal in semantic_goals
        if slice_start < int(goal["waypoint_index"]) <= slice_stop
        and goal.get("source") in {"gateway", "room_center"}
    ]
    return targets


def waypoint_room(waypoint: dict[str, Any] | None) -> str | None:
    if not waypoint:
        return None
    return waypoint.get("from_room") or waypoint.get("to_room")


def find_anchor(waypoints: list[dict[str, Any]], *, source: str | None = None, room: str | None = None, gateway_suffix: str | None = None) -> dict[str, Any] | None:
    for waypoint in waypoints:
        if source is not None and waypoint.get("source") != source:
            continue
        if room is not None and waypoint_room(waypoint) != room:
            continue
        gateway_id = str(waypoint.get("gateway_id") or "")
        if gateway_suffix is not None and not gateway_id.endswith(gateway_suffix):
            continue
        return waypoint
    return None


def semantic_goal_tolerance(waypoint: dict[str, Any], args: argparse.Namespace) -> float:
    if waypoint.get("source") == "gateway":
        return float(args.gateway_semantic_tolerance_m)
    if waypoint.get("source") == "room_center":
        return float(args.room_center_semantic_tolerance_m)
    return float(args.goal_tolerance_m)


def semantic_status_for_row(row: dict[str, Any], waypoint: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    navigate = row.get("navigate") or {}
    tolerance = semantic_goal_tolerance(waypoint, args)
    final_distance = row.get("final_distance_to_goal_m")
    skip_distance = row.get("distance_to_goal_at_skip_m")
    status = str(navigate.get("status") or "")
    source = waypoint.get("source")
    terminal = bool(row.get("is_terminal_semantic_goal"))

    if navigate.get("attempted"):
        semantic_completed = bool(navigate.get("success")) or (final_distance is not None and float(final_distance) <= tolerance)
        mode = "navigate_to_pose_succeeded" if navigate.get("success") else "navigate_to_pose_distance_tolerance"
        if not semantic_completed:
            mode = "navigate_to_pose_failed"
        return {
            "semantic_completed": semantic_completed,
            "completion_mode": mode,
            "distance_to_goal_m": round_float(final_distance),
            "required_tolerance_m": tolerance,
            "reason": "NavigateToPose result is separate from fallback projection progress",
        }

    if status in {
        "completed_by_split_follow_path_anchor_validation",
        "completed_by_split_follow_path_terminal_tolerance",
        "completed_by_split_follow_path_slice_success",
    }:
        semantic_completed = final_distance is None or float(final_distance) <= tolerance or status == "completed_by_split_follow_path_slice_success"
        return {
            "semantic_completed": bool(semantic_completed),
            "completion_mode": status,
            "distance_to_goal_m": round_float(final_distance),
            "required_tolerance_m": tolerance,
            "reason": "split FollowPath semantic gate, not full-route projection",
        }

    if status == "split_follow_path_anchor_unfinished":
        return {
            "semantic_completed": False,
            "completion_mode": status,
            "distance_to_goal_m": round_float(final_distance),
            "required_tolerance_m": tolerance,
            "reason": "required split anchor was not reached within tolerance",
        }

    if status == "skipped_forward_progress":
        within_tolerance = skip_distance is not None and float(skip_distance) <= tolerance
        if source == "gateway":
            semantic_completed = bool(within_tolerance)
            reason = "route_progress_skip_gateway_within_tolerance" if semantic_completed else "route_progress_passed_gateway_but_gateway_distance_too_large"
        elif source == "room_center":
            semantic_completed = bool(within_tolerance and terminal)
            reason = (
                "terminal_room_center_skip_within_tolerance"
                if semantic_completed
                else "route_progress_passed_goal_but_goal_distance_too_large"
            )
        else:
            semantic_completed = True
            reason = "route_progress_skip_counts_as_execution_progress_for_non_semantic_bridge_like_goal"
        return {
            "semantic_completed": semantic_completed,
            "completion_mode": "skipped_by_route_projection",
            "distance_to_goal_at_skip_m": round_float(skip_distance),
            "required_tolerance_m": tolerance,
            "nearest_wp_at_skip": row.get("nearest_wp_at_skip"),
            "route_progress_index_at_skip": row.get("route_progress_index_at_skip"),
            "reason": reason,
        }

    return {
        "semantic_completed": False,
        "completion_mode": status or "not_attempted",
        "distance_to_goal_m": round_float(final_distance if final_distance is not None else skip_distance),
        "required_tolerance_m": tolerance,
        "reason": navigate.get("failure_reason") or "semantic goal was not completed",
    }


def first_semantic_index_at_or_after_progress(semantic_goals: list[dict[str, Any]], projected_path_index: int | None) -> int:
    if projected_path_index is None:
        return 0
    for idx, waypoint in enumerate(semantic_goals):
        if int(waypoint["waypoint_index"]) >= int(projected_path_index):
            return idx
    return len(semantic_goals)


def append_semantic_waypoint_result(
    base: dict[str, Any],
    semantic_goals: list[dict[str, Any]],
    waypoint: dict[str, Any],
    *,
    status: str,
    success: bool,
    final_distance: float | None = None,
    extra: dict[str, Any] | None = None,
) -> None:
    waypoint_index = int(waypoint["waypoint_index"])
    existing = [
        row for row in base.get("waypoint_results", [])
        if int(row.get("waypoint_index", -1)) == waypoint_index
    ]
    row = {
        "selected_list_index": next(
            (idx for idx, goal in enumerate(semantic_goals) if int(goal["waypoint_index"]) == waypoint_index),
            None,
        ),
        "waypoint_index": waypoint_index,
        "source": waypoint.get("source"),
        "gateway_id": waypoint.get("gateway_id"),
        "from_room": waypoint.get("from_room"),
        "to_room": waypoint.get("to_room"),
        "plan": {"attempted": False},
        "navigate": {"attempted": False, "status": status, "success": success},
        "final_distance_to_goal_m": round_float(final_distance, 6),
    }
    if extra:
        row.update(extra)
    if existing:
        existing[-1].update(row)
    else:
        base.setdefault("waypoint_results", []).append(row)


def save_controller_path_artifact(
    base: dict[str, Any],
    output_json: Path | None,
    *,
    path_id: str,
    action_type: str,
    points: list[dict[str, Any]],
    target: dict[str, Any] | None,
    start_pose: dict[str, Any] | None,
    target_pose: dict[str, Any] | None = None,
    map_yaml: Path | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    run_dir = (Path(output_json).resolve().parent if output_json else Path(".").resolve())
    sent_dir = run_dir / "sent_controller_paths"
    metadata = target_metadata(target)
    wall_crossing = None
    if points and map_yaml and validate_point_sequence is not None:
        wall_crossing = validate_point_sequence(points, Path(map_yaml))
    payload: dict[str, Any] = {
        "artifact_type": "sent_controller_path",
        "version": "v0_1",
        "created_utc": now_iso(),
        "path_id": path_id,
        "action_type": action_type,
        **metadata,
        "start_pose": pose_summary(start_pose or (points[0] if points else None)),
        "target_pose": pose_summary(target_pose or target or (points[-1] if points else None)),
        "pose_count": len(points),
        "path_length_m": round_float(path_length(points), 6),
        "wall_crossing": wall_crossing,
        "first_meter_heading_summary": first_meter_heading_summary(points, start_pose),
        "first_meter_clearance_summary": clearance_summary(points, map_yaml),
        "points": [
            {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "yaw": float(point.get("yaw", 0.0) or 0.0),
                "source": point.get("source"),
                "waypoint_index": point.get("waypoint_index"),
                "target_waypoint_index": point.get("target_waypoint_index"),
            }
            for point in points
        ],
    }
    if extra:
        payload.update(extra)
    path = sent_dir / f"{path_id}.json"
    write_json(path, payload)
    summary = {
        "path_id": path_id,
        "path": path.as_posix(),
        "action_type": action_type,
        **metadata,
        "pose_count": len(points),
        "path_length_m": payload["path_length_m"],
        "wall_crossing_validation_passed": (wall_crossing or {}).get("wall_crossing_validation_passed") if wall_crossing is not None else None,
        "first_meter_min_clearance_m": (payload.get("first_meter_clearance_summary") or {}).get("first_meter_min_clearance_m"),
    }
    base.setdefault("sent_controller_paths", []).append(summary)
    base["sent_controller_paths_dir"] = sent_dir.as_posix()
    return payload


def semantic_row_precedence(row: dict[str, Any]) -> int:
    navigate = row.get("navigate") or {}
    status = str(navigate.get("status") or "")
    semantic = row.get("semantic_status") or {}
    if status == "completed_by_split_follow_path_anchor_validation":
        return 100
    if status == "completed_by_split_follow_path_terminal_tolerance":
        return 95
    if navigate.get("attempted") and navigate.get("success"):
        return 90
    if semantic.get("semantic_completed"):
        return 80
    if navigate.get("attempted"):
        return 50
    if status == "split_follow_path_anchor_unfinished":
        return 40
    if status == "skipped_forward_progress":
        return 10
    return 0


def validate_split_anchor_at_pose(
    node: Any,
    waypoint: dict[str, Any],
    args: argparse.Namespace,
    *,
    label: str,
) -> dict[str, Any]:
    before_pose = node.current_pose()
    before_distance = pose_distance(before_pose, waypoint)
    tolerance = float(args.semantic_anchor_tolerance_m)
    validation = {
        "target_waypoint_index": int(waypoint["waypoint_index"]),
        "target_source": waypoint.get("source"),
        "target_room": waypoint_room(waypoint),
        "tolerance_m": tolerance,
        "distance_before_dwell_m": round_float(before_distance, 6),
        "dwell_sec": float(args.through_room_dwell_sec),
        "min_inside_samples_requested": int(args.through_room_min_inside_samples),
        "room_mask_validation_available": False,
        "room_mask_inside_sample_count": None,
        "room_mask_validation_note": "room mask validation is not wired into the runtime runner; using distance plus dwell only",
        "completed": bool(before_distance is not None and before_distance <= tolerance),
        "completion_mode": None,
    }
    if not validation["completed"]:
        validation["completion_mode"] = "distance_before_dwell_outside_tolerance"
        return validation
    dwell_result = node.dwell(
        float(args.through_room_dwell_sec),
        label,
        context={
            "slice_index": None,
            "active_slice_start_index": int(waypoint["waypoint_index"]),
            "active_slice_stop_index": int(waypoint["waypoint_index"]),
            "target_waypoint_index": int(waypoint["waypoint_index"]),
            "target_room": waypoint_room(waypoint),
        },
    )
    after_pose = node.current_pose()
    after_distance = pose_distance(after_pose, waypoint)
    validation["dwell_result"] = dwell_result
    validation["distance_after_dwell_m"] = round_float(after_distance, 6)
    validation["completed"] = bool(after_distance is not None and after_distance <= tolerance)
    validation["completion_mode"] = "distance_and_dwell" if validation["completed"] else "distance_after_dwell_outside_tolerance"
    return validation


class RouteNode(Node):  # pragma: no cover - ROS runtime only
    def __init__(self) -> None:
        super().__init__("boxfusion_stage1_scene_route_runner")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom: dict[str, Any] | None = None
        self.feedback_count = 0
        self.follow_feedback_count = 0
        self.trajectory_samples: list[dict[str, Any]] = []
        self.latest_cmd_vel: dict[str, float] | None = None
        self.latest_local_costmap: Any | None = None
        self.latest_local_costmap_received_wall_sec: float | None = None
        self.active_sample_context: dict[str, Any] = {}
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 20)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.local_costmap_cb, 1)

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "yaw": yaw_from_quat(q), "frame_id": msg.header.frame_id or "odom"}

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.latest_cmd_vel = {"linear_x": float(msg.linear.x), "angular_z": float(msg.angular.z)}

    def local_costmap_cb(self, msg: OccupancyGrid) -> None:
        self.latest_local_costmap = msg
        self.latest_local_costmap_received_wall_sec = time.time()

    def current_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            t = transform.transform.translation
            q = transform.transform.rotation
            return {"x": float(t.x), "y": float(t.y), "z": float(t.z), "yaw": yaw_from_quat(q), "frame_id": "map", "source": "/tf map->base_footprint"}
        except TransformException:
            if self.latest_odom:
                pose = dict(self.latest_odom)
                pose["frame_id"] = "map"
                pose["source"] = "/odom under static map->odom"
                return pose
            return None

    def sample_pose(self, phase: str, context: dict[str, Any] | None = None) -> None:
        pose = self.current_pose()
        if pose:
            pose["sample_index"] = len(self.trajectory_samples)
            pose["time_wall_sec"] = time.time()
            pose["phase"] = phase
            merged_context = dict(self.active_sample_context)
            if context:
                merged_context.update(context)
            pose.update({key: value for key, value in merged_context.items() if value is not None})
            if self.latest_cmd_vel is not None:
                pose["cmd_vel_linear_x"] = self.latest_cmd_vel["linear_x"]
                pose["cmd_vel_angular_z"] = self.latest_cmd_vel["angular_z"]
            self.trajectory_samples.append(pose)

    def make_pose(self, waypoint: dict[str, Any]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])
        yaw = waypoint.get("yaw", 0.0)
        q = yaw_to_quat(float(0.0 if yaw is None else yaw))
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        return pose

    def make_path(self, waypoints: list[dict[str, Any]]) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(waypoint) for waypoint in waypoints]
        return path

    def nav_path_to_waypoints(self, path: NavPath, target: dict[str, Any], *, label: str) -> list[dict[str, Any]]:
        converted: list[dict[str, Any]] = []
        target_index = int(target.get("waypoint_index", -1))
        for idx, pose_stamped in enumerate(path.poses):
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            converted.append({
                "x": float(p.x),
                "y": float(p.y),
                "yaw": yaw_from_quat(q),
                "source": "computed_path_pose",
                "computed_path_label": label,
                "waypoint_index": target_index if idx == len(path.poses) - 1 else target_index * 10000 + idx,
                "target_waypoint_index": target_index,
            })
        return converted

    def follow_path(self, waypoints: list[dict[str, Any]], timeout_sec: float, context: dict[str, Any] | None = None) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None, "path_pose_count": len(waypoints), "path_waypoint_indices": [int(w["waypoint_index"]) for w in waypoints]}
        if not waypoints:
            result["failure_reason"] = "empty FollowPath path"
            return result
        goal = FollowPath.Goal()
        goal.path = self.make_path(waypoints)
        if hasattr(goal, "controller_id"):
            goal.controller_id = "FollowPath"
        before = self.follow_feedback_count
        send_future = self.follow_path_client.send_goal_async(goal, feedback_callback=lambda _msg: setattr(self, "follow_feedback_count", self.follow_feedback_count + 1))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "FollowPath send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "FollowPath goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        deadline = time.time() + timeout_sec
        next_sample = 0.0
        previous_context = dict(self.active_sample_context)
        self.active_sample_context = dict(context or {})
        while rclpy.ok() and time.time() < deadline and not get_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose("follow_path")
                next_sample = time.time() + 0.5
        self.active_sample_context = previous_context
        if not get_future.done():
            try:
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception:
                pass
            result["status"] = "timeout"
            result["failure_reason"] = f"FollowPath timed out after {timeout_sec}s"
            result["feedback_count_delta"] = self.follow_feedback_count - before
            return result
        wrapped = get_future.result()
        status = int(wrapped.status) if wrapped is not None else None
        result["status"] = status_name(status)
        result["success"] = status == GoalStatus.STATUS_SUCCEEDED
        if not result["success"]:
            result["failure_reason"] = f"FollowPath returned {result['status']}"
            abort_pose = self.current_pose()
            previous_abort_context = dict(self.active_sample_context)
            abort_context = dict(context or {})
            abort_context["follow_path_terminal_status"] = result["status"]
            self.active_sample_context = abort_context
            self.sample_pose("follow_path")
            self.active_sample_context = previous_abort_context
            abort_sample = self.trajectory_samples[-1] if self.trajectory_samples and self.trajectory_samples[-1].get("follow_path_terminal_status") == result["status"] else None
            result["abort_pose_observed"] = pose_summary(abort_pose)
            result["abort_sample_index"] = abort_sample.get("sample_index") if abort_sample else None
            result["abort_trajectory_sample"] = abort_sample
            result["abort_local_costmap_snapshot"] = local_costmap_summary(
                self.latest_local_costmap,
                abort_pose,
                received_wall_sec=self.latest_local_costmap_received_wall_sec,
            )
        result["feedback_count_delta"] = self.follow_feedback_count - before
        return result

    def dwell(self, seconds: float, label: str, context: dict[str, Any] | None = None) -> dict[str, Any]:
        result = {"attempted": seconds > 0.0, "label": label, "requested_sec": seconds, "sample_count_before": len(self.trajectory_samples)}
        deadline = time.time() + max(0.0, seconds)
        next_sample = 0.0
        previous_context = dict(self.active_sample_context)
        self.active_sample_context = dict(context or {})
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose("post")
                next_sample = time.time() + 0.25
        self.active_sample_context = previous_context
        result["sample_count_after"] = len(self.trajectory_samples)
        result["recorded_sample_count"] = result["sample_count_after"] - result["sample_count_before"]
        result["success"] = True
        return result

    def compute_plan(self, waypoint: dict[str, Any], timeout_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None}
        goal = ComputePathToPose.Goal()
        goal.pose = self.make_pose(waypoint)
        goal.planner_id = "GridBased"
        send_future = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "ComputePathToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "ComputePathToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_future, timeout_sec=timeout_sec)
        wrapped = get_future.result()
        if wrapped is None:
            result["failure_reason"] = "ComputePathToPose result timeout"
            return result
        result["status"] = status_name(int(wrapped.status))
        result["success"] = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED and len(wrapped.result.path.poses) > 0
        result["path_pose_count"] = len(wrapped.result.path.poses) if wrapped and wrapped.result else 0
        if result["success"]:
            result["path_poses"] = self.nav_path_to_waypoints(wrapped.result.path, waypoint, label=f"computed_to_{waypoint.get('waypoint_index')}")
        if not result["success"]:
            result["failure_reason"] = f"ComputePathToPose returned {result['status']}"
        return result

    def navigate(self, waypoint: dict[str, Any], timeout_sec: float, context: dict[str, Any] | None = None) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None}
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(waypoint)
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=lambda _msg: setattr(self, "feedback_count", self.feedback_count + 1))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "NavigateToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "NavigateToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        deadline = time.time() + timeout_sec
        next_sample = 0.0
        previous_context = dict(self.active_sample_context)
        self.active_sample_context = dict(context or {})
        while rclpy.ok() and time.time() < deadline and not get_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose("fallback")
                next_sample = time.time() + 0.5
        self.active_sample_context = previous_context
        if not get_future.done():
            try:
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception:
                pass
            result["status"] = "timeout"
            result["failure_reason"] = f"NavigateToPose timed out after {timeout_sec}s"
            return result
        wrapped = get_future.result()
        status = int(wrapped.status) if wrapped is not None else None
        result["status"] = status_name(status)
        result["success"] = status == GoalStatus.STATUS_SUCCEEDED
        if not result["success"]:
            result["failure_reason"] = f"NavigateToPose returned {result['status']}"
        return result


def semantic_goals_from_waypoints(waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
    goals = [
        w for w in waypoints
        if w.get("source") in ALLOWED_GOAL_SOURCES
        or str(w.get("source", "")).endswith("_interior_terminal")
    ]
    forbidden = [w for w in goals if w.get("source") in FORBIDDEN_NAV_GOAL_SOURCES]
    if forbidden:
        indices = [int(w["waypoint_index"]) for w in forbidden]
        raise ValueError(f"bridge/smooth bridge waypoints selected as NavigateToPose goals: {indices}")
    return goals


def gazebo_reset_to_route_start(waypoint: dict[str, Any], model_name: str, timeout_sec: float) -> dict[str, Any]:
    result: dict[str, Any] = {
        "attempted": True,
        "success": False,
        "model_name": model_name,
        "target_pose": {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint.get("z", 0.08)),
            "yaw": float(waypoint.get("yaw", 0.0)),
        },
        "service_used": None,
        "failure_reason": None,
    }
    try:
        services = subprocess.run(
            ["ros2", "service", "list"],
            check=False,
            text=True,
            capture_output=True,
            timeout=max(3.0, min(timeout_sec, 10.0)),
        )
    except Exception as exc:
        result["failure_reason"] = f"could not list ROS services: {type(exc).__name__}: {exc}"
        return result
    service_names = set(services.stdout.splitlines())
    q = yaw_to_quat(float(waypoint.get("yaw", 0.0)))
    entity_yaml = (
        "{state: {"
        f"name: '{model_name}', "
        f"pose: {{position: {{x: {float(waypoint['x'])}, y: {float(waypoint['y'])}, z: {float(waypoint.get('z', 0.08))}}}, "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}}}, "
        "twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, "
        "reference_frame: 'world'}}"
    )
    model_yaml = (
        "{model_state: {"
        f"model_name: '{model_name}', "
        f"pose: {{position: {{x: {float(waypoint['x'])}, y: {float(waypoint['y'])}, z: {float(waypoint.get('z', 0.08))}}}, "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}}}, "
        "twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, "
        "reference_frame: 'world'}}"
    )
    candidates = [
        ("/set_entity_state", "gazebo_msgs/srv/SetEntityState", entity_yaml),
        ("/gazebo/set_entity_state", "gazebo_msgs/srv/SetEntityState", entity_yaml),
        ("/set_model_state", "gazebo_msgs/srv/SetModelState", model_yaml),
        ("/gazebo/set_model_state", "gazebo_msgs/srv/SetModelState", model_yaml),
    ]
    for service, service_type, payload in candidates:
        if service not in service_names:
            continue
        try:
            call = subprocess.run(
                ["ros2", "service", "call", service, service_type, payload],
                check=False,
                text=True,
                capture_output=True,
                timeout=timeout_sec,
            )
        except subprocess.TimeoutExpired:
            result["service_used"] = service
            result["failure_reason"] = f"{service} timed out after {timeout_sec}s"
            return result
        result["service_used"] = service
        result["service_type"] = service_type
        result["returncode"] = call.returncode
        result["stdout_tail"] = call.stdout[-1000:]
        result["stderr_tail"] = call.stderr[-1000:]
        if call.returncode == 0 and "success: True" in call.stdout:
            result["success"] = True
            result["failure_reason"] = None
            return result
        result["failure_reason"] = f"{service} returned {call.returncode}"
    if result["service_used"] is None:
        result["failure_reason"] = "no Gazebo entity/model state reset service was available"
    return result


def validate_route_artifacts(stage_output: Path, waypoints_payload: dict[str, Any] | None = None, expected_room_chain: list[str] | None = None, expected_gateway_sequence: list[str] | None = None) -> dict[str, Any]:
    rooms = None
    gateways = None
    source = "waypoints_payload"
    if waypoints_payload:
        rooms = waypoints_payload.get("room_sequence")
        if not rooms:
            seen = []
            for waypoint in waypoints_payload.get("waypoints") or []:
                for key in ("from_room", "to_room"):
                    room = waypoint.get(key)
                    if room and room not in seen:
                        seen.append(room)
            rooms = seen
        gateways = waypoints_payload.get("gateway_sequence") or [
            waypoint.get("gateway_id")
            for waypoint in waypoints_payload.get("waypoints") or []
            if waypoint.get("gateway_id")
        ]
    expected_rooms = expected_room_chain or EXPECTED_ROOM_CHAIN
    expected_gateways = expected_gateway_sequence or EXPECTED_GATEWAY_SEQUENCE
    return {
        "room_chain": rooms,
        "gateway_sequence": gateways,
        "resolved_route_path": None,
        "source": source,
        "expected_room_chain": expected_rooms,
        "expected_gateway_sequence": expected_gateways,
        "matches_scene_truth": True,
        "matches_expected_route": (not expected_rooms or rooms == expected_rooms) and (not expected_gateways or gateways == expected_gateways),
    }


def default_waypoints_path(args: argparse.Namespace, stage_output: Path) -> Path:
    if args.waypoints_json:
        return args.waypoints_json.resolve()
    profile_waypoints: Path | None = None
    if default_runtime_profile and load_runtime_profile and resolve_path:
        profile_path = resolve_path(args.runtime_profile) or default_runtime_profile(stage_output, args.floor_id or "floor_2")
        profile = load_runtime_profile(profile_path)
        profile_waypoints = resolve_path(profile.get("waypoints")) if profile.get("waypoints") else None
        if profile_waypoints:
            executable = profile_waypoints.with_name("executable_route_waypoints_v0_1.json")
            if executable.exists():
                return executable.resolve()
            return profile_waypoints.resolve()
    if args.floor_id and args.scene_id:
        route_dir = stage_output / "routes" / "room_routes"
        candidates = sorted(route_dir.glob(f"{args.floor_id}_*/executable_route_waypoints_v0_1.json"))
        if candidates:
            return candidates[0].resolve()
    historical = stage_output / "execution/scene_room_chain_reference_continuous_waypoints.json"
    return historical.resolve()


def infer_trace_sample(raw: dict[str, Any], waypoints: list[dict[str, Any]], anchors: dict[str, dict[str, Any] | None], first_wall_sec: float | None, progress_so_far: int | None) -> tuple[dict[str, Any], int | None]:
    nearest_idx, nearest_dist = nearest_path_index(waypoints, raw)
    nearest_wp = waypoints[nearest_idx] if waypoints else {}
    nearest_waypoint_index = int(nearest_wp.get("waypoint_index", nearest_idx)) if nearest_wp else None
    route_progress = max([value for value in [progress_so_far, nearest_waypoint_index] if value is not None], default=None)
    t0 = first_wall_sec if first_wall_sec is not None else raw.get("time_wall_sec")
    sample = {
        "sample_index": raw.get("sample_index"),
        "t_sec": round_float(float(raw.get("time_wall_sec", 0.0)) - float(t0 or 0.0), 3),
        "time_wall_sec": round_float(raw.get("time_wall_sec"), 6),
        "phase": raw.get("phase") or "unknown",
        "x": round_float(raw.get("x"), 6),
        "y": round_float(raw.get("y"), 6),
        "yaw": round_float(raw.get("yaw"), 6),
        "nearest_wp_index": nearest_waypoint_index,
        "nearest_wp_distance_m": round_float(nearest_dist, 6),
        "nearest_wp_source": nearest_wp.get("source"),
        "nearest_wp_gateway_id": nearest_wp.get("gateway_id"),
        "nearest_wp_room": waypoint_room(nearest_wp),
        "nearest_wp_segment_index": nearest_wp.get("segment_index"),
        "active_segment_index": nearest_wp.get("segment_index"),
        "dist_to_room13_center_m": round_float(pose_distance(raw, anchors.get("room13_center")), 6),
        "dist_to_gateway006_m": round_float(pose_distance(raw, anchors.get("gateway006")), 6),
        "dist_to_room14_center_m": round_float(pose_distance(raw, anchors.get("room14_center")), 6),
        "route_progress_index": route_progress,
        "cross_track_error_m": round_float(nearest_dist, 6),
        "cmd_vel_linear_x": round_float(raw.get("cmd_vel_linear_x"), 6),
        "cmd_vel_angular_z": round_float(raw.get("cmd_vel_angular_z"), 6),
    }
    return sample, route_progress


def build_route_execution_trace(samples: list[dict[str, Any]], waypoints: list[dict[str, Any]], anchors: dict[str, dict[str, Any] | None]) -> list[dict[str, Any]]:
    first_wall_sec = samples[0].get("time_wall_sec") if samples else None
    progress: int | None = None
    rows: list[dict[str, Any]] = []
    for raw in samples:
        row, progress = infer_trace_sample(raw, waypoints, anchors, first_wall_sec, progress)
        rows.append(row)
    return rows


def sample_at_or_after_progress(trace: list[dict[str, Any]], waypoint_index: int) -> dict[str, Any] | None:
    for sample in trace:
        progress = sample.get("route_progress_index")
        nearest = sample.get("nearest_wp_index")
        if (progress is not None and int(progress) >= waypoint_index) or (nearest is not None and int(nearest) >= waypoint_index):
            return sample
    return trace[-1] if trace else None


def enrich_waypoint_results_with_semantics(base: dict[str, Any], semantic_goals: list[dict[str, Any]], trace: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    goal_by_index = {int(goal["waypoint_index"]): goal for goal in semantic_goals}
    terminal_index = int(semantic_goals[-1]["waypoint_index"]) if semantic_goals else None
    event_statuses: list[dict[str, Any]] = []
    final_rows_by_index: dict[int, dict[str, Any]] = {}
    for row in base.get("waypoint_results", []):
        waypoint_index = int(row["waypoint_index"])
        waypoint = goal_by_index.get(waypoint_index)
        if waypoint is None:
            continue
        row["is_terminal_semantic_goal"] = terminal_index == waypoint_index
        if (row.get("navigate") or {}).get("status") == "skipped_forward_progress":
            skip_sample = sample_at_or_after_progress(trace, waypoint_index)
            if skip_sample:
                row["skip_pose"] = {key: skip_sample.get(key) for key in ("sample_index", "t_sec", "x", "y", "yaw")}
                row["nearest_wp_at_skip"] = skip_sample.get("nearest_wp_index")
                row["route_progress_index_at_skip"] = skip_sample.get("route_progress_index")
                row["distance_to_goal_at_skip_m"] = round_float(pose_distance(skip_sample, waypoint), 6)
        row["semantic_status"] = semantic_status_for_row(row, waypoint, args)
        event = {
            "selected_list_index": row.get("selected_list_index"),
            "waypoint_index": waypoint_index,
            "source": row.get("source"),
            "gateway_id": row.get("gateway_id"),
            "from_room": row.get("from_room"),
            "to_room": row.get("to_room"),
            "navigate": row.get("navigate"),
            "semantic_status": row.get("semantic_status"),
            "distance_to_goal_at_skip_m": row.get("distance_to_goal_at_skip_m"),
            "final_distance_to_goal_m": row.get("final_distance_to_goal_m"),
        }
        event_statuses.append(event)
        previous = final_rows_by_index.get(waypoint_index)
        if previous is None or semantic_row_precedence(row) >= semantic_row_precedence(previous):
            final_rows_by_index[waypoint_index] = row

    statuses: list[dict[str, Any]] = []
    for waypoint in semantic_goals:
        waypoint_index = int(waypoint["waypoint_index"])
        row = final_rows_by_index.get(waypoint_index)
        if row is None:
            continue
        statuses.append({
            "selected_list_index": row.get("selected_list_index"),
            "waypoint_index": waypoint_index,
            "source": row.get("source"),
            "gateway_id": row.get("gateway_id"),
            "from_room": row.get("from_room"),
            "to_room": row.get("to_room"),
            "navigate": row.get("navigate"),
            "semantic_status": row.get("semantic_status"),
            "distance_to_goal_at_skip_m": row.get("distance_to_goal_at_skip_m"),
            "final_distance_to_goal_m": row.get("final_distance_to_goal_m"),
        })
    suppressed = []
    final_keys = {(int(row["waypoint_index"]), str((row.get("navigate") or {}).get("status") or "")) for row in final_rows_by_index.values()}
    for event in event_statuses:
        key = (int(event["waypoint_index"]), str((event.get("navigate") or {}).get("status") or ""))
        if key not in final_keys:
            suppressed.append(event)
    base["semantic_goal_status_events"] = event_statuses
    base["semantic_goal_status_suppressed_duplicate_events"] = suppressed
    base["fallback_skipped_completed_semantic_waypoints"] = [
        int(row["waypoint_index"])
        for row in final_rows_by_index.values()
        if (row.get("navigate") or {}).get("status") == "skipped_forward_progress"
        and (row.get("semantic_status") or {}).get("semantic_completed")
    ]
    base["fallback_skipped_by_projection_semantic_waypoints"] = [
        int(row["waypoint_index"])
        for row in final_rows_by_index.values()
        if (row.get("navigate") or {}).get("status") == "skipped_forward_progress"
    ]
    base["semantic_goal_status_path"] = SEMANTIC_STATUS_FILENAME
    return {
        "artifact_type": "route_semantic_goal_status",
        "version": "v0_1",
        "created_utc": now_iso(),
        "goal_tolerance_m": float(args.goal_tolerance_m),
        "room_center_semantic_tolerance_m": float(args.room_center_semantic_tolerance_m),
        "gateway_semantic_tolerance_m": float(args.gateway_semantic_tolerance_m),
        "semantic_goals": statuses,
        "semantic_goal_events": event_statuses,
        "suppressed_duplicate_events": suppressed,
    }


def build_deviation_events(base: dict[str, Any], trace: list[dict[str, Any]], args: argparse.Namespace) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    prev_nearest: int | None = None
    in_high_error = False
    spin_start: dict[str, Any] | None = None
    spin_count = 0
    for sample in trace:
        nearest = sample.get("nearest_wp_index")
        if prev_nearest is not None and nearest is not None and int(nearest) - int(prev_nearest) > args.progress_jump_event_threshold:
            events.append({
                "event": "nearest_waypoint_index_jump_forward",
                "sample_index": sample.get("sample_index"),
                "t_sec": sample.get("t_sec"),
                "previous_nearest_wp_index": prev_nearest,
                "nearest_wp_index": nearest,
                "x": sample.get("x"),
                "y": sample.get("y"),
            })
        if nearest is not None:
            prev_nearest = int(nearest)
        error = sample.get("cross_track_error_m")
        if error is not None and float(error) > args.deviation_event_threshold_m and not in_high_error:
            in_high_error = True
            events.append({
                "event": "cross_track_error_exceeded",
                "threshold_m": float(args.deviation_event_threshold_m),
                "cross_track_error_m": error,
                "sample_index": sample.get("sample_index"),
                "t_sec": sample.get("t_sec"),
                "nearest_wp_index": sample.get("nearest_wp_index"),
                "x": sample.get("x"),
                "y": sample.get("y"),
            })
        elif error is not None and float(error) <= args.deviation_event_threshold_m * 0.75:
            in_high_error = False

        lin = sample.get("cmd_vel_linear_x")
        ang = sample.get("cmd_vel_angular_z")
        if lin is not None and ang is not None and abs(float(lin)) < 0.03 and abs(float(ang)) > 0.35:
            spin_count += 1
            spin_start = spin_start or sample
            if spin_count == int(args.spin_event_sample_count):
                events.append({
                    "event": "repeated_spin_cmd_vel",
                    "sample_index_start": spin_start.get("sample_index"),
                    "sample_index_end": sample.get("sample_index"),
                    "t_sec_start": spin_start.get("t_sec"),
                    "t_sec_end": sample.get("t_sec"),
                    "cmd_vel_linear_x": lin,
                    "cmd_vel_angular_z": ang,
                })
        else:
            spin_count = 0
            spin_start = None

    for attempt in base.get("follow_path_attempts", []):
        if attempt.get("attempted") and not attempt.get("success"):
            events.append({
                "event": "follow_path_aborted" if attempt.get("status") == "aborted" else "follow_path_failed",
                "status": attempt.get("status"),
                "failure_reason": attempt.get("failure_reason"),
                "projected_next_path_start_slice_index": attempt.get("projected_next_path_start_slice_index"),
                "projected_next_path_distance_m": attempt.get("projected_next_path_distance_m"),
            })
    if base.get("sparse_fallback_used"):
        events.append({
            "event": "fallback_started",
            "fallback_started_at_selected_list_index": base.get("fallback_started_at_selected_list_index"),
            "fallback_started_at_waypoint_index": base.get("fallback_started_at_waypoint_index"),
            "projected_next_path_start_slice_index": base.get("projected_next_path_start_slice_index"),
        })
    for row in base.get("waypoint_results", []):
        navigate = row.get("navigate") or {}
        if navigate.get("status") == "skipped_forward_progress" and row.get("source") == "room_center" and not (row.get("semantic_status") or {}).get("semantic_completed"):
            events.append({
                "event": "semantic_room_center_skipped_by_projection_not_completed",
                "waypoint_index": row.get("waypoint_index"),
                "from_room": row.get("from_room"),
                "distance_to_goal_at_skip_m": row.get("distance_to_goal_at_skip_m"),
                "required_tolerance_m": (row.get("semantic_status") or {}).get("required_tolerance_m"),
                "nearest_wp_at_skip": row.get("nearest_wp_at_skip"),
                "route_progress_index_at_skip": row.get("route_progress_index_at_skip"),
            })
        if navigate.get("attempted"):
            events.append({
                "event": "fallback_navigate_to_pose_completed" if navigate.get("success") else "fallback_navigate_to_pose_failed",
                "waypoint_index": row.get("waypoint_index"),
                "source": row.get("source"),
                "gateway_id": row.get("gateway_id"),
                "status": navigate.get("status"),
                "success": navigate.get("success"),
                "final_distance_to_goal_m": row.get("final_distance_to_goal_m"),
            })
    return events


def latest_trace_sample(trace: list[dict[str, Any]]) -> dict[str, Any] | None:
    return trace[-1] if trace else None


def follow_path_abort_trace_sample(trace: list[dict[str, Any]], attempt: dict[str, Any]) -> dict[str, Any] | None:
    abort_sample_index = attempt.get("abort_sample_index")
    if abort_sample_index is not None:
        for sample in trace:
            if sample.get("sample_index") is not None and int(sample["sample_index"]) == int(abort_sample_index):
                return sample
    follow_samples = [sample for sample in trace if sample.get("phase") == "follow_path"]
    if follow_samples:
        return follow_samples[-1]
    projected = attempt.get("projected_next_path_start_slice_index")
    if projected is not None:
        for sample in trace:
            nearest = sample.get("nearest_wp_index")
            progress = sample.get("route_progress_index")
            if (nearest is not None and int(nearest) >= int(projected)) or (progress is not None and int(progress) >= int(projected)):
                return sample
    return trace[-1] if trace else None


def failed_follow_path_for_abort_context(base: dict[str, Any]) -> dict[str, Any] | None:
    failed = [attempt for attempt in base.get("follow_path_attempts", []) if attempt.get("attempted") and not attempt.get("success")]
    if not failed:
        return None
    aborted = [attempt for attempt in failed if attempt.get("status") == "aborted"]
    return aborted[-1] if aborted else failed[-1]


def build_follow_path_abort_context(base: dict[str, Any], trace: list[dict[str, Any]], anchors: dict[str, dict[str, Any] | None]) -> dict[str, Any] | None:
    attempt = failed_follow_path_for_abort_context(base)
    if attempt is None:
        return None
    current = follow_path_abort_trace_sample(trace, attempt)
    sample_index = current.get("sample_index") if current else None
    preceding = [
        sample for sample in trace
        if sample_index is None or sample.get("sample_index") is None or int(sample.get("sample_index")) <= int(sample_index)
    ]
    abort_pose = attempt.get("abort_pose_observed") or ({key: current.get(key) for key in ("x", "y", "yaw", "t_sec", "sample_index")} if current else base.get("final_pose_observed"))
    embedded_costmap = attempt.get("abort_local_costmap_snapshot") or {}
    return {
        "artifact_type": "follow_path_abort_context",
        "version": "v0_1",
        "created_utc": now_iso(),
        "follow_path_attempt": attempt,
        "current_pose": abort_pose,
        "abort_pose_source": "follow_path_result_immediate_capture" if attempt.get("abort_pose_observed") else "trajectory_trace_or_final_pose_fallback",
        "abort_trace_sample_index": sample_index,
        "nearest_waypoint_index": current.get("nearest_wp_index") if current else None,
        "nearest_segment_index": current.get("active_segment_index") if current else None,
        "projected_route_progress_index": current.get("route_progress_index") if current else attempt.get("projected_next_path_start_slice_index"),
        "distance_to_route_m": current.get("cross_track_error_m") if current else None,
        "dist_to_room13_center_m": current.get("dist_to_room13_center_m") if current else None,
        "dist_to_gateway006_m": current.get("dist_to_gateway006_m") if current else None,
        "dist_to_room14_center_m": current.get("dist_to_room14_center_m") if current else None,
        "last_trace_samples": preceding[-20:],
        "fallback_started": bool(base.get("sparse_fallback_used")),
        "local_costmap_snapshot_available_at_abort": bool(embedded_costmap.get("captured")),
        "local_costmap_snapshot_timing": embedded_costmap.get("capture_timing"),
        "next_fallback_targets": [
            {
                "waypoint_index": row.get("waypoint_index"),
                "source": row.get("source"),
                "gateway_id": row.get("gateway_id"),
                "navigate": row.get("navigate"),
                "semantic_status": row.get("semantic_status"),
            }
            for row in base.get("waypoint_results", [])
        ],
        "anchor_waypoints": {
            name: {"waypoint_index": anchor.get("waypoint_index"), "x": anchor.get("x"), "y": anchor.get("y"), "source": anchor.get("source"), "gateway_id": anchor.get("gateway_id")}
            for name, anchor in anchors.items()
            if anchor
        },
    }


def build_local_costmap_snapshot(abort_context: dict[str, Any], trace: list[dict[str, Any]], anchors: dict[str, dict[str, Any] | None], stage_output: Path) -> dict[str, Any]:
    attempt = abort_context.get("follow_path_attempt") or {}
    abort_pose = abort_context.get("current_pose")
    embedded = dict(attempt.get("abort_local_costmap_snapshot") or {})
    payload: dict[str, Any] = {
        "artifact_type": "local_costmap_snapshot",
        "version": "v0_1",
        "created_utc": now_iso(),
        "captured": False,
        "capture_timing": embedded.get("capture_timing") or "unavailable",
        "robot_pose": abort_pose,
        "nearest_route_sample": follow_path_abort_trace_sample(trace, attempt) or latest_trace_sample(trace),
        "dist_to_room13_center_m": abort_context.get("dist_to_room13_center_m"),
        "dist_to_gateway006_m": abort_context.get("dist_to_gateway006_m"),
        "dist_to_room14_center_m": abort_context.get("dist_to_room14_center_m"),
        "robot_radius": None,
        "inflation_radius": None,
        "reason_if_missing": None,
        "abort_pose_match_tolerance_m": 0.05,
        "snapshot_pose_matches_abort_pose": None,
        "snapshot_pose_to_abort_pose_distance_m": None,
    }
    params_candidates = sorted((stage_output / "runtime" / "nav2").glob("*nav2_params.yaml"))
    if params_candidates:
        text = params_candidates[0].read_text(encoding="utf-8", errors="replace")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("robot_radius:") and payload["robot_radius"] is None:
                payload["robot_radius"] = round_float(stripped.split(":", 1)[1].strip())
            if stripped.startswith("inflation_radius:") and payload["inflation_radius"] is None:
                payload["inflation_radius"] = round_float(stripped.split(":", 1)[1].strip())
    if not embedded:
        payload["reason_if_missing"] = "no abort-time local costmap capture was attached to the failed FollowPath attempt"
        return payload
    payload.update(embedded)
    snapshot_pose = embedded.get("robot_pose_at_capture")
    distance = pose_distance(snapshot_pose, abort_pose)
    payload["snapshot_pose_to_abort_pose_distance_m"] = round_float(distance, 6)
    payload["snapshot_pose_matches_abort_pose"] = bool(distance is not None and distance <= payload["abort_pose_match_tolerance_m"])
    if not payload.get("captured") and not payload.get("reason_if_missing"):
        payload["reason_if_missing"] = "abort-time local costmap sample was unavailable; no post-fallback costmap was substituted"
    return payload


def write_route_diagnostic_artifacts(output_json: Path, base: dict[str, Any], waypoints: list[dict[str, Any]], semantic_goals: list[dict[str, Any]], args: argparse.Namespace, node: Any | None = None) -> None:
    run_dir = output_json.resolve().parent
    anchors = {
        "room13_center": find_anchor(waypoints, source="room_center", room="room_13"),
        "gateway006": find_anchor(waypoints, source="gateway", gateway_suffix="gateway_006"),
        "room14_center": find_anchor(waypoints, source="room_center", room="room_14"),
    }
    trace = build_route_execution_trace(base.get("trajectory_samples", []), waypoints, anchors)
    semantic_status = enrich_waypoint_results_with_semantics(base, semantic_goals, trace, args)
    events = build_deviation_events(base, trace, args)
    abort_context = build_follow_path_abort_context(base, trace, anchors)

    trace_path = run_dir / TRACE_FILENAME
    semantic_path = run_dir / SEMANTIC_STATUS_FILENAME
    events_path = run_dir / DEVIATION_EVENTS_FILENAME
    abort_path = run_dir / FOLLOW_PATH_ABORT_CONTEXT_FILENAME
    append_jsonl(trace_path, trace)
    write_json(semantic_path, semantic_status)
    append_jsonl(events_path, events)
    if abort_context is not None:
        attempt = abort_context.get("follow_path_attempt") or {}
        base["follow_path_abort_detected"] = attempt.get("status") == "aborted"
        base["follow_path_abort_reason"] = attempt.get("failure_reason")
        is_handoff_gateway_attempt = attempt.get("target_source") == "gateway" or attempt.get("target_gateway_id") is not None
        base["controller_handoff_failure_detected"] = bool(is_handoff_gateway_attempt and attempt.get("status") == "aborted")
        base["controller_handoff_failure_reason"] = attempt.get("failure_reason") if base["controller_handoff_failure_detected"] else None
        base["controller_handoff_abort_waypoint_index"] = abort_context.get("nearest_waypoint_index") if base["controller_handoff_failure_detected"] else None
        base["controller_handoff_abort_distance_to_gateway006_m"] = abort_context.get("dist_to_gateway006_m") if base["controller_handoff_failure_detected"] else None
        write_json(abort_path, abort_context)
        snapshot = build_local_costmap_snapshot(abort_context, trace, anchors, Path(base.get("stage_output_dir", ".")))
        write_json(run_dir / "local_costmap_snapshots" / "follow_path_abort_costmap.json", snapshot)
    if base.get("sent_controller_paths") is not None:
        sent_summary_path = run_dir / "sent_controller_paths_summary.json"
        write_json(sent_summary_path, {
            "artifact_type": "sent_controller_paths_summary",
            "version": "v0_1",
            "created_utc": now_iso(),
            "sent_controller_paths_dir": base.get("sent_controller_paths_dir"),
            "path_count": len(base.get("sent_controller_paths") or []),
            "paths": base.get("sent_controller_paths") or [],
        })
        base["sent_controller_paths_summary"] = sent_summary_path.as_posix()
    base["diagnostic_artifacts"] = {
        "route_execution_trace": trace_path.as_posix(),
        "semantic_goal_status": semantic_path.as_posix(),
        "deviation_events": events_path.as_posix(),
        "follow_path_abort_context": abort_path.as_posix() if abort_context is not None else None,
        "local_costmap_snapshot": (run_dir / "local_costmap_snapshots" / "follow_path_abort_costmap.json").as_posix() if abort_context is not None else None,
        "sent_controller_paths_summary": base.get("sent_controller_paths_summary"),
    }
    base["route_execution_trace_sample_count"] = len(trace)
    base["deviation_event_count"] = len(events)


def apply_wall_crossing_runtime_validation(base: dict[str, Any], args: argparse.Namespace) -> None:
    if not getattr(args, "validate_wall_crossing", False):
        base["terminal_reached"] = bool(base.get("final_arrival_success"))
        base["clean_runtime_success"] = (
            bool(base.get("succeeded"))
            and bool(base.get("terminal_reached"))
            and base.get("execution_strategy") == HANDOFF_STRATEGY
            and not bool(base.get("sparse_fallback_used"))
        )
        return
    output_path = getattr(args, "wall_crossing_output_json", None)
    map_yaml = getattr(args, "wall_crossing_map_yaml", None)
    if validate_samples is None:
        base["wall_crossing_validation_error"] = "validate_scene_trajectory_wall_crossing import unavailable"
        base["wall_crossing_validation_passed"] = None
        base["clean_runtime_success"] = False
        return
    if not map_yaml:
        base["wall_crossing_validation_error"] = "--wall-crossing-map-yaml is required with --validate-wall-crossing"
        base["wall_crossing_validation_passed"] = None
        base["clean_runtime_success"] = False
        return
    report = validate_samples(base.get("trajectory_samples", []), Path(map_yaml))
    base["wall_crossing_detected"] = bool(report.get("wall_crossing_detected"))
    base["wall_crossing_validation_passed"] = bool(report.get("wall_crossing_validation_passed"))
    base["per_phase_wall_crossing_counts"] = report.get("per_phase_wall_crossing_counts")
    base["spin_like_sample_counts"] = report.get("spin_like_sample_counts")
    base["terminal_reached"] = bool(base.get("final_arrival_success"))
    base["clean_runtime_success"] = (
        bool(base.get("succeeded"))
        and bool(base.get("terminal_reached"))
        and bool(report.get("wall_crossing_validation_passed"))
        and base.get("execution_strategy") == HANDOFF_STRATEGY
        and not bool(base.get("sparse_fallback_used"))
    )
    if output_path:
        write_json(Path(output_path), report)
        base["wall_crossing_validation_report"] = Path(output_path).as_posix()
    if base.get("wall_crossing_detected"):
        if base.get("succeeded"):
            base["failure_layer"] = "wall_crossing_validation"
            base["failure_reason"] = "terminal may have been reached, but trajectory crossed occupied map cells"
        base["succeeded"] = False


def run_route(args: argparse.Namespace, stage_output: Path) -> dict[str, Any]:
    waypoints_path = default_waypoints_path(args, stage_output)
    waypoints_payload = read_json(waypoints_path)
    expected_room_chain = args.expected_room_chain.split(",") if args.expected_room_chain else None
    expected_gateway_sequence = args.expected_gateway_sequence.split(",") if args.expected_gateway_sequence else None
    route_validation = validate_route_artifacts(stage_output, waypoints_payload, expected_room_chain, expected_gateway_sequence)
    waypoints = list(waypoints_payload.get("waypoints") or [])
    semantic_goals = semantic_goals_from_waypoints(waypoints)
    detected_split_anchors = detect_split_anchors(waypoints)
    if args.runtime_profile:
        runtime_profile_path = resolve_path(args.runtime_profile) if resolve_path else Path(args.runtime_profile).resolve()
    elif default_runtime_profile:
        runtime_profile_path = default_runtime_profile(stage_output, args.floor_id or "floor_2")
    else:
        runtime_profile_path = stage_output / "runtime" / "profiles" / f"{args.floor_id or 'floor_2'}_nav2" / "runtime_profile.json"
    runtime_profile_payload: dict[str, Any] = {}
    if runtime_profile_path.exists():
        try:
            runtime_profile_payload = read_json(runtime_profile_path)
        except Exception:
            runtime_profile_payload = {}
    base: dict[str, Any] = {
        "artifact_type": "step30s2_scene_route_rerun_report",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "runtime_profile": runtime_profile_path.as_posix(),
        "controller_profile": runtime_profile_payload.get("controller_profile", "baseline"),
        "nav2_params": runtime_profile_payload.get("nav2_params"),
        "waypoints_source": waypoints_path.as_posix(),
        "route_validation": route_validation,
        "localization_mode": "static_map_to_odom",
        "amcl_used": False,
        "direct_cmd_vel_published_by_harness": False,
        "bridge_waypoints_used_as_goals": False,
        "semantic_goal_waypoint_indices": [int(w["waypoint_index"]) for w in semantic_goals],
        "path_slice_waypoint_indices": [int(w["waypoint_index"]) for w in waypoints],
        "execute_attempted": False,
        "succeeded": False,
        "from_start_requested": bool(getattr(args, "from_start", False)),
        "reset_to_route_start_requested": bool(getattr(args, "reset_to_route_start", False)),
        "route_start_waypoint": {"waypoint_index": int(waypoints[0]["waypoint_index"]), "x": float(waypoints[0]["x"]), "y": float(waypoints[0]["y"]), "yaw": float(waypoints[0].get("yaw", 0.0))} if waypoints else None,
        "follow_path_used": False,
        "split_follow_path_used": False,
        "split_anchors": detected_split_anchors,
        "split_slices": [],
        "slice_replan_from_current_pose_used": False,
        "current_pose_gateway_handoff_used": bool(getattr(args, "current_pose_gateway_handoff", False) or args.execution_strategy == HANDOFF_STRATEGY),
        "slice1_strategy": "prebuilt_split_follow_path_slice",
        "handoff_attempts": [],
        "through_room_anchor_validation": [],
        "semantic_anchor_validation_settings": {
            "semantic_anchor_tolerance_m": float(args.semantic_anchor_tolerance_m),
            "through_room_dwell_sec": float(args.through_room_dwell_sec),
            "through_room_min_inside_samples": int(args.through_room_min_inside_samples),
        },
        "sparse_fallback_used": False,
        "clean_runtime_success_requirements": {
            "clean_runtime_success": True,
            "terminal_reached": True,
            "execution_strategy": HANDOFF_STRATEGY,
            "sparse_fallback_used": False,
        },
        "fallback_resume_policy": "progress_aware_projected_path_index_forward_only",
        "fallback_started_at_selected_list_index": None,
        "fallback_started_at_waypoint_index": None,
        "fallback_backtracking_detected": False,
        "follow_path_attempts": [],
        "waypoint_results": [],
    }
    route_expected_ok = bool(route_validation["matches_expected_route"])
    if not route_expected_ok and not args.allow_non_scene_truth:
        base.update({"failure_layer": "route artifact", "failure_reason": "route/gateway artifacts do not match the expected Scene route"})
        return base
    if IMPORT_ERROR:
        base.update({"failure_layer": "rclpy", "failure_reason": IMPORT_ERROR})
        return base
    if args.reset_to_route_start:
        base["reset_to_route_start_result"] = gazebo_reset_to_route_start(waypoints[0], args.robot_model_name, args.reset_timeout_sec)
        time.sleep(args.reset_settle_sec)

    rclpy.init(args=None)
    node = RouteNode()
    try:
        deadline = time.time() + args.pose_timeout_sec
        start_pose = None
        while rclpy.ok() and time.time() < deadline and start_pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            start_pose = node.current_pose()
        base["start_pose_observed"] = start_pose
        if start_pose is None:
            base.update({"failure_layer": "TF", "failure_reason": "could not observe robot pose"})
            return base
        base["execute_attempted"] = True
        path_start_idx, path_start_distance = nearest_path_index(waypoints, start_pose)
        base["observed_nearest_route_start_slice_index"] = path_start_idx
        base["observed_nearest_route_start_distance_m"] = round(path_start_distance, 6) if path_start_distance is not None else None
        if args.from_start:
            base["from_start_reset_verified"] = bool(
                path_start_distance is not None
                and path_start_idx <= args.from_start_max_start_slice_index
                and path_start_distance <= args.from_start_tolerance_m
            )
            if not base["from_start_reset_verified"]:
                base.update({
                    "failure_layer": "from_start_reset",
                    "failure_reason": (
                        "--from-start requested but robot pose was not near the beginning of the route "
                        f"(nearest_index={path_start_idx}, distance_m={path_start_distance})"
                    ),
                    "path_start_slice_index": path_start_idx,
                    "path_resume_start_slice_index": path_start_idx,
                })
                return base
            path_start_idx = 0
        elif path_start_distance is None or path_start_distance > args.resume_tolerance_m:
            path_start_idx = 0
        follow_path_slice = waypoints[path_start_idx:]
        base["path_start_slice_index"] = path_start_idx
        base["path_resume_start_slice_index"] = path_start_idx
        base["path_resume_start_distance_m"] = round(path_start_distance, 6) if path_start_distance is not None else None
        base["path_pose_count"] = len(follow_path_slice)
        base["follow_path_waypoint_indices"] = [int(w["waypoint_index"]) for w in follow_path_slice]
        write_json(args.latest_slice_output_json or stage_output / "post_restructure_validation/step30s2_latest_follow_path_slice.json", {
            "artifact_type": "step30s2_latest_follow_path_slice",
            "created_utc": now_iso(),
            "path_resume_start_slice_index": path_start_idx,
            "path_start_slice_index": path_start_idx,
            "waypoint_indices": [int(w["waypoint_index"]) for w in follow_path_slice],
            "poses": [{"x": float(w["x"]), "y": float(w["y"]), "source": w.get("source")} for w in follow_path_slice],
        })
        follow_ready = node.follow_path_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        nav_ready = node.nav_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        plan_ready = node.plan_client.wait_for_server(timeout_sec=3.0)
        base["action_servers"] = {"/follow_path": follow_ready, "/navigate_to_pose": nav_ready, "/compute_path_to_pose": plan_ready}
        projected_progress: int | None = None
        if follow_ready:
            base["follow_path_used"] = True
            if args.execution_strategy in {SPLIT_STRATEGY, HANDOFF_STRATEGY} or args.split_at_through_room_anchors or base["current_pose_gateway_handoff_used"]:
                base["execution_strategy"] = HANDOFF_STRATEGY if base["current_pose_gateway_handoff_used"] else SPLIT_STRATEGY
                base["split_follow_path_used"] = True
                split_slices = split_slices_for_anchors(path_start_idx, waypoints, detected_split_anchors)
                base["split_slices"] = split_slices
                if len(split_slices) <= 1:
                    # A route terminating at the entered room is monotonic: there is
                    # no outbound fold that requires a current-pose gateway handoff.
                    base["split_follow_path_used"] = False
                    base["current_pose_gateway_handoff_applicable"] = False
                    base["current_pose_gateway_handoff_not_applicable_reason"] = "generated route has no interior fold anchor"
                    save_controller_path_artifact(
                        base,
                        getattr(args, "_resolved_output_json", None) or args.output_json,
                        path_id="monotonic_route_followpath_path",
                        action_type="FollowPath",
                        points=follow_path_slice,
                        target=waypoints[-1],
                        start_pose=node.current_pose(),
                        map_yaml=args.wall_crossing_map_yaml,
                        extra={"handoff_not_applicable": True},
                    )
                    attempt = node.follow_path(
                        follow_path_slice,
                        args.follow_path_timeout_sec,
                        context={"segment": "monotonic_target_room_route", "handoff_not_applicable": True},
                    )
                    final_distance = pose_distance(node.current_pose(), waypoints[-1])
                    attempt["distance_to_terminal_after_attempt_m"] = round_float(final_distance, 6)
                    base["follow_path_attempts"].append(attempt)
                    base["follow_path_result"] = attempt
                    if attempt.get("success") or (final_distance is not None and final_distance <= args.goal_tolerance_m):
                        base.update({"succeeded": True, "final_arrival_success": True})
                        return base
                    base.update({
                        "failure_layer": "follow_path",
                        "failure_reason": attempt.get("failure_reason") or "monotonic generated route FollowPath did not reach its terminal",
                    })
                    return base
                split_replan_failed = False
                for planned_slice in split_slices:
                    slice_start = int(planned_slice["path_start_slice_index"])
                    slice_stop = int(planned_slice["path_stop_slice_index"])
                    slice_waypoints = waypoints[slice_start:slice_stop + 1]
                    target = waypoints[slice_stop]
                    is_required_anchor = any(int(anchor["slice_index"]) == slice_stop for anchor in detected_split_anchors)
                    context = {
                        "slice_index": int(planned_slice["slice_index"]),
                        "active_slice_start_index": slice_start,
                        "active_slice_stop_index": slice_stop,
                        "target_waypoint_index": int(target["waypoint_index"]),
                        "target_room": target_room_for_metadata(target),
                        "target_source": target.get("source"),
                        "target_gateway_id": target.get("gateway_id"),
                        "from_room": target.get("from_room"),
                        "to_room": target.get("to_room"),
                    }
                    if (args.replan_slice_from_current_pose or base["current_pose_gateway_handoff_used"]) and not is_required_anchor:
                        base["slice_replan_from_current_pose_used"] = True
                        base["slice1_strategy"] = "current_pose_gateway_handoff_staging_then_gateway_then_room_center" if base["current_pose_gateway_handoff_used"] else "current_pose_compute_path_to_pose_gateway_then_room_center_follow_path"
                        replan_targets = semantic_targets_for_slice(semantic_goals, slice_start, slice_stop)
                        if not replan_targets:
                            replan_targets = [target]
                        base.setdefault("slice_replan_targets", []).extend([
                            {
                                "slice_index": int(planned_slice["slice_index"]),
                                **target_metadata(item),
                            }
                            for item in replan_targets
                        ])
                        for subslice_index, subtarget in enumerate(replan_targets):
                            sub_context = dict(context)
                            sub_context.update({
                                "subslice_index": subslice_index,
                                **target_metadata(subtarget),
                            })
                            start_pose_for_plan = node.current_pose()
                            plan = node.compute_plan(subtarget, args.plan_timeout_sec) if plan_ready else {
                                "attempted": False,
                                "success": False,
                                "failure_reason": "/compute_path_to_pose action server not available",
                            }
                            computed_path = list(plan.get("path_poses") or [])
                            wall_validation = None
                            if computed_path and args.wall_crossing_map_yaml and validate_point_sequence is not None:
                                wall_validation = validate_point_sequence(computed_path, args.wall_crossing_map_yaml)
                            if computed_path:
                                path_id = f"slice{planned_slice['slice_index']}_sub{subslice_index}_current_pose_to_wp{int(subtarget['waypoint_index'])}_full_plan"
                                if base["current_pose_gateway_handoff_used"] and subtarget.get("source") == "gateway":
                                    suffix = str(subtarget.get("gateway_id") or f"wp{int(subtarget['waypoint_index'])}").split("_")[-1]
                                    path_id = f"slice{planned_slice['slice_index']}a_current_pose_to_gateway{suffix}_full_plan"
                                elif base["current_pose_gateway_handoff_used"] and subtarget.get("source") == "room_center":
                                    path_id = f"slice{planned_slice['slice_index'] + 1}_current_pose_to_{waypoint_room(subtarget)}_full_plan"
                                save_controller_path_artifact(
                                    base,
                                    getattr(args, "_resolved_output_json", None) or args.output_json,
                                    path_id=path_id,
                                    action_type="ComputePathToPose",
                                    points=computed_path,
                                    target=subtarget,
                                    start_pose=start_pose_for_plan,
                                    map_yaml=args.wall_crossing_map_yaml,
                                )
                            attempt = {
                                "attempted": False,
                                "goal_accepted": False,
                                "success": False,
                                "status": None,
                                "failure_reason": None,
                                "slice_index": int(planned_slice["slice_index"]),
                                "subslice_index": subslice_index,
                                "execution_method": "compute_path_to_pose_then_follow_path",
                                "compute_path_to_pose": plan,
                                "computed_path_wall_validation": wall_validation,
                                "path_start_slice_index": slice_start,
                                "path_stop_slice_index": slice_stop,
                                **target_metadata(subtarget),
                                "active_slice_projection_only": False,
                                "start_pose_for_plan": pose_summary(start_pose_for_plan),
                            }
                            if not plan.get("success") or not computed_path:
                                attempt["status"] = "blocked_by_compute_path_to_pose_failure"
                                attempt["failure_reason"] = plan.get("failure_reason") or "ComputePathToPose produced no path"
                                base["follow_path_attempts"].append(attempt)
                                base["follow_path_result"] = attempt
                                split_replan_failed = True
                                break
                            if wall_validation is not None and not wall_validation.get("wall_crossing_validation_passed"):
                                attempt["status"] = "blocked_by_computed_path_wall_crossing_validation"
                                attempt["failure_reason"] = "computed current-pose path intersects occupied map cells"
                                base["follow_path_attempts"].append(attempt)
                                base["follow_path_result"] = attempt
                                split_replan_failed = True
                                break

                            handoff_this_target = (
                                bool(base["current_pose_gateway_handoff_used"])
                                and subtarget.get("source") == "gateway"
                                and (
                                    args.handoff_target_waypoint_index is None
                                    or int(subtarget["waypoint_index"]) == int(args.handoff_target_waypoint_index)
                                )
                            )
                            if handoff_this_target:
                                prefix, staging_distance = path_prefix_to_distance(
                                    computed_path,
                                    float(args.handoff_target_distance_m),
                                    float(args.handoff_min_distance_m),
                                    float(args.handoff_max_distance_m),
                                )
                                staging_pose = prefix[-1] if prefix else None
                                staging_wall_validation = validate_point_sequence(prefix, args.wall_crossing_map_yaml) if prefix and args.wall_crossing_map_yaml and validate_point_sequence is not None else None
                                suffix = str(subtarget.get("gateway_id") or f"wp{int(subtarget['waypoint_index'])}").split("_")[-1]
                                staging_artifact = save_controller_path_artifact(
                                    base,
                                    getattr(args, "_resolved_output_json", None) or args.output_json,
                                    path_id=f"slice{planned_slice['slice_index']}a_handoff_staging_prefix_path",
                                    action_type="FollowPath",
                                    points=prefix,
                                    target=subtarget,
                                    start_pose=start_pose_for_plan,
                                    target_pose=staging_pose,
                                    map_yaml=args.wall_crossing_map_yaml,
                                    extra={
                                        "handoff_target_distance_m": float(args.handoff_target_distance_m),
                                        "handoff_min_distance_m": float(args.handoff_min_distance_m),
                                        "handoff_max_distance_m": float(args.handoff_max_distance_m),
                                        "staging_distance_m": round_float(staging_distance, 6),
                                        "staging_pose": pose_summary(staging_pose),
                                    },
                                )
                                handoff = {
                                    "target": target_metadata(subtarget),
                                    "staging_distance_m": round_float(staging_distance, 6),
                                    "staging_pose": pose_summary(staging_pose),
                                    "staging_wall_validation": staging_wall_validation,
                                    "staging_artifact": staging_artifact.get("path_id"),
                                }
                                base["handoff_attempts"].append(handoff)
                                if not prefix or (staging_wall_validation is not None and not staging_wall_validation.get("wall_crossing_validation_passed")):
                                    attempt["status"] = "blocked_by_handoff_staging_wall_crossing_validation"
                                    attempt["failure_reason"] = "handoff staging prefix intersects occupied map cells"
                                    base["follow_path_attempts"].append(attempt)
                                    base["follow_path_result"] = attempt
                                    split_replan_failed = True
                                    break
                                staging_context = dict(sub_context)
                                staging_context.update({
                                    "handoff_phase": "staging_prefix",
                                    "handoff_staging_distance_m": round_float(staging_distance, 6),
                                })
                                staging_follow = node.follow_path(prefix, args.handoff_follow_path_timeout_sec, context=staging_context)
                                handoff["staging_follow_path"] = staging_follow
                                distance_to_staging = pose_distance(node.current_pose(), staging_pose)
                                handoff["distance_to_staging_after_attempt_m"] = round_float(distance_to_staging, 6)
                                attempt.setdefault("handoff", {}).update(handoff)
                                staging_ok = bool(staging_follow.get("success")) or (distance_to_staging is not None and distance_to_staging <= float(args.handoff_staging_tolerance_m))
                                if not staging_ok:
                                    attempt.update(staging_follow)
                                    attempt["execution_method"] = "current_pose_gateway_handoff_staging_prefix"
                                    attempt["failure_reason"] = staging_follow.get("failure_reason") or "handoff staging FollowPath did not reach staging point"
                                    base["follow_path_attempts"].append(attempt)
                                    base["follow_path_result"] = attempt
                                    split_replan_failed = True
                                    break
                                gateway_start_pose = node.current_pose()
                                gateway_plan = node.compute_plan(subtarget, args.plan_timeout_sec) if plan_ready else {
                                    "attempted": False,
                                    "success": False,
                                    "failure_reason": "/compute_path_to_pose action server not available",
                                }
                                gateway_path = list(gateway_plan.get("path_poses") or [])
                                gateway_wall_validation = validate_point_sequence(gateway_path, args.wall_crossing_map_yaml) if gateway_path and args.wall_crossing_map_yaml and validate_point_sequence is not None else None
                                if gateway_path:
                                    save_controller_path_artifact(
                                        base,
                                        getattr(args, "_resolved_output_json", None) or args.output_json,
                                        path_id=f"slice{planned_slice['slice_index']}b_gateway{suffix}_remaining_path",
                                        action_type="FollowPath",
                                        points=gateway_path,
                                        target=subtarget,
                                        start_pose=gateway_start_pose,
                                        map_yaml=args.wall_crossing_map_yaml,
                                        extra={"compute_path_to_pose": gateway_plan},
                                    )
                                if not gateway_plan.get("success") or not gateway_path:
                                    attempt["status"] = "blocked_by_gateway_remaining_compute_path_to_pose_failure"
                                    attempt["failure_reason"] = gateway_plan.get("failure_reason") or "ComputePathToPose to gateway after staging produced no path"
                                    attempt.setdefault("handoff", {})["gateway_compute_path_to_pose"] = gateway_plan
                                    base["follow_path_attempts"].append(attempt)
                                    base["follow_path_result"] = attempt
                                    split_replan_failed = True
                                    break
                                if gateway_wall_validation is not None and not gateway_wall_validation.get("wall_crossing_validation_passed"):
                                    attempt["status"] = "blocked_by_gateway_remaining_wall_crossing_validation"
                                    attempt["failure_reason"] = "remaining path to gateway intersects occupied map cells"
                                    attempt.setdefault("handoff", {})["gateway_wall_validation"] = gateway_wall_validation
                                    base["follow_path_attempts"].append(attempt)
                                    base["follow_path_result"] = attempt
                                    split_replan_failed = True
                                    break
                                gateway_context = dict(sub_context)
                                gateway_context["handoff_phase"] = "gateway_remaining"
                                follow_attempt = node.follow_path(gateway_path, args.follow_path_timeout_sec / max(len(split_slices), 1), context=gateway_context)
                                attempt["execution_method"] = "current_pose_gateway_handoff_staging_then_remaining_gateway_follow_path"
                                attempt["handoff"]["gateway_compute_path_to_pose"] = gateway_plan
                                attempt["handoff"]["gateway_wall_validation"] = gateway_wall_validation
                                attempt["handoff"]["gateway_follow_path"] = follow_attempt
                            else:
                                follow_path_id = f"slice{planned_slice['slice_index']}_sub{subslice_index}_wp{int(subtarget['waypoint_index'])}_follow_path"
                                if base["current_pose_gateway_handoff_used"] and subtarget.get("source") == "room_center":
                                    follow_path_id = f"slice{planned_slice['slice_index'] + 1}_current_pose_to_{waypoint_room(subtarget)}_path"
                                save_controller_path_artifact(
                                    base,
                                    getattr(args, "_resolved_output_json", None) or args.output_json,
                                    path_id=follow_path_id,
                                    action_type="FollowPath",
                                    points=computed_path,
                                    target=subtarget,
                                    start_pose=start_pose_for_plan,
                                    map_yaml=args.wall_crossing_map_yaml,
                                )
                                follow_attempt = node.follow_path(computed_path, args.follow_path_timeout_sec / max(len(split_slices), 1), context=sub_context)
                            attempt.update(follow_attempt)
                            current_after = node.current_pose()
                            local_projection, local_projection_distance = nearest_path_index(waypoints, current_after)
                            projected_progress = local_projection
                            distance_to_subtarget = pose_distance(current_after, subtarget)
                            attempt.update({
                                "projected_next_path_start_slice_index": projected_progress,
                                "projected_next_path_distance_m": round_float(local_projection_distance, 6),
                                "distance_to_target_after_attempt_m": round_float(distance_to_subtarget, 6),
                            })
                            base["follow_path_attempts"].append(attempt)
                            base["follow_path_result"] = attempt
                            tolerance = semantic_goal_tolerance(subtarget, args)
                            if attempt.get("success") or (distance_to_subtarget is not None and distance_to_subtarget <= tolerance):
                                append_semantic_waypoint_result(
                                    base,
                                    semantic_goals,
                                    subtarget,
                                    status="completed_by_split_follow_path_terminal_tolerance",
                                    success=True,
                                    final_distance=distance_to_subtarget,
                                    extra={"execution_method": "current_pose_replanned_follow_path"},
                                )
                                continue
                            split_replan_failed = True
                            break
                        if split_replan_failed:
                            break
                        terminal_distance = pose_distance(node.current_pose(), target)
                        if terminal_distance is not None and terminal_distance <= args.goal_tolerance_m:
                            base.update({"succeeded": True, "final_arrival_success": True})
                            return base
                        split_replan_failed = True
                        break

                    save_controller_path_artifact(
                        base,
                        getattr(args, "_resolved_output_json", None) or args.output_json,
                        path_id=f"slice{planned_slice['slice_index']}_followpath_path",
                        action_type="FollowPath",
                        points=slice_waypoints,
                        target=target,
                        start_pose=node.current_pose(),
                        map_yaml=args.wall_crossing_map_yaml,
                    )
                    attempt = node.follow_path(slice_waypoints, args.follow_path_timeout_sec / max(len(split_slices), 1), context=context)
                    current_after = node.current_pose()
                    local_projection, local_projection_distance = nearest_path_index(slice_waypoints, current_after)
                    projected_progress = slice_start + local_projection
                    attempt.update({
                        "slice_index": int(planned_slice["slice_index"]),
                        "path_start_slice_index": slice_start,
                        "path_stop_slice_index": slice_stop,
                        **target_metadata(target),
                        "active_slice_projection_only": True,
                        "projected_next_path_start_slice_index": projected_progress,
                        "projected_next_path_distance_m": round_float(local_projection_distance, 6),
                    })
                    distance_to_target = pose_distance(current_after, target)
                    if is_required_anchor:
                        attempt["distance_to_split_target_after_attempt_m"] = round_float(distance_to_target, 6)
                    else:
                        attempt["distance_to_terminal_after_attempt_m"] = round_float(distance_to_target, 6)
                    base["follow_path_attempts"].append(attempt)
                    base["follow_path_result"] = attempt

                    if is_required_anchor:
                        validation = validate_split_anchor_at_pose(node, target, args, label=f"split_anchor_{target.get('source')}_{waypoint_room(target)}")
                        validation["slice_index"] = int(planned_slice["slice_index"])
                        validation["follow_path_status"] = attempt.get("status")
                        validation["follow_path_success"] = bool(attempt.get("success"))
                        base["through_room_anchor_validation"].append(validation)
                        append_semantic_waypoint_result(
                            base,
                            semantic_goals,
                            target,
                            status="completed_by_split_follow_path_anchor_validation" if validation.get("completed") else "split_follow_path_anchor_unfinished",
                            success=bool(validation.get("completed")),
                            final_distance=validation.get("distance_after_dwell_m", validation.get("distance_before_dwell_m")),
                            extra={"split_anchor_validation": validation},
                        )
                        if validation.get("completed"):
                            continue
                        if nav_ready:
                            base["sparse_fallback_used"] = True
                            base["fallback_resume_policy"] = "split_anchor_guard_current_anchor_only"
                            base["fallback_started_at_waypoint_index"] = int(target["waypoint_index"])
                            base["fallback_started_at_selected_list_index"] = next(
                                (idx for idx, goal in enumerate(semantic_goals) if int(goal["waypoint_index"]) == int(target["waypoint_index"])),
                                None,
                            )
                            row_extra = {"reason": "FollowPath slice did not validate required anchor; fallback is restricted to the unfinished anchor"}
                            nav_result = node.navigate(target, args.goal_timeout_sec)
                            final_distance = pose_distance(node.current_pose(), target)
                            row = {
                                "selected_list_index": base["fallback_started_at_selected_list_index"],
                                "waypoint_index": int(target["waypoint_index"]),
                                "source": target.get("source"),
                "gateway_id": target.get("gateway_id"),
                                "from_room": target.get("from_room"),
                                "to_room": target.get("to_room"),
                                "plan": {"attempted": False},
                                "navigate": nav_result,
                                "final_distance_to_goal_m": round_float(final_distance, 6),
                            }
                            row.update(row_extra)
                            base.setdefault("waypoint_results", []).append(row)
                            if nav_result.get("success") or (final_distance is not None and final_distance <= float(args.semantic_anchor_tolerance_m)):
                                validation = validate_split_anchor_at_pose(node, target, args, label=f"split_anchor_fallback_{target.get('source')}_{waypoint_room(target)}")
                                validation["slice_index"] = int(planned_slice["slice_index"])
                                validation["completion_after_fallback"] = True
                                base["through_room_anchor_validation"].append(validation)
                                if validation.get("completed"):
                                    append_semantic_waypoint_result(
                                        base,
                                        semantic_goals,
                                        target,
                                        status="completed_by_split_follow_path_anchor_validation",
                                        success=True,
                                        final_distance=validation.get("distance_after_dwell_m", validation.get("distance_before_dwell_m")),
                                        extra={"split_anchor_validation": validation},
                                    )
                                    continue
                        base.update({
                            "succeeded": False,
                            "final_arrival_success": False,
                            "failure_layer": "split_follow_path_anchor_validation",
                            "failure_reason": "required split anchor was not reached/validated; outbound slice was not sent",
                        })
                        return base

                    if attempt.get("success") or (distance_to_target is not None and distance_to_target <= args.goal_tolerance_m):
                        append_semantic_waypoint_result(
                            base,
                            semantic_goals,
                            target,
                            status="completed_by_split_follow_path_terminal_tolerance",
                            success=True,
                            final_distance=distance_to_target,
                        )
                        base.update({"succeeded": True, "final_arrival_success": True})
                        return base

                projected_progress = min(projected_progress if projected_progress is not None else path_start_idx, split_slices[-1]["path_stop_slice_index"])
            else:
                base["execution_strategy"] = "follow_path"
                attempt = node.follow_path(follow_path_slice, args.follow_path_timeout_sec)
                attempt["path_start_slice_index"] = path_start_idx
                current_after = node.current_pose()
                projected_progress, projected_distance = nearest_path_index(waypoints, current_after)
                attempt["projected_next_path_start_slice_index"] = projected_progress
                attempt["projected_next_path_distance_m"] = round(projected_distance, 6) if projected_distance is not None else None
                attempt["distance_to_terminal_after_attempt_m"] = round(pose_distance(current_after, waypoints[-1]) or 999.0, 6)
                base["follow_path_attempts"].append(attempt)
                base["follow_path_result"] = attempt
                if attempt.get("success") or (pose_distance(current_after, waypoints[-1]) is not None and pose_distance(current_after, waypoints[-1]) <= args.goal_tolerance_m):
                    base.update({"succeeded": True, "execution_strategy": "follow_path", "final_arrival_success": True})
                    return base
        if not nav_ready:
            base.update({"failure_layer": "fallback", "failure_reason": "/navigate_to_pose action server not available"})
            return base
        base["sparse_fallback_used"] = True
        if base.get("split_follow_path_used"):
            base["execution_strategy"] = f"{SPLIT_STRATEGY}_then_forward_only_sparse_fallback"
        else:
            base["execution_strategy"] = "follow_path_then_forward_only_sparse_fallback" if base.get("follow_path_used") else "forward_only_sparse_fallback"
        current_for_fallback = node.current_pose()
        pose_progress, pose_distance_m = nearest_path_index(waypoints, current_for_fallback)
        candidates = [value for value in [projected_progress, pose_progress] if value is not None]
        projected_progress = max(candidates) if candidates else None
        start_idx = first_semantic_index_at_or_after_progress(semantic_goals, projected_progress)
        required_interior_indices = [
            idx for idx, goal in enumerate(semantic_goals)
            if str(goal.get("source", "")).endswith("_interior_terminal")
        ]
        if args.enforce_interior_targets and required_interior_indices:
            unvisited_required = [
                idx for idx in required_interior_indices
                if projected_progress is None or int(semantic_goals[idx]["waypoint_index"]) >= max(0, int(projected_progress) - args.interior_target_progress_guard_window)
            ]
            if unvisited_required:
                start_idx = min(start_idx, min(unvisited_required))
        base["projected_next_path_start_slice_index"] = projected_progress
        base["current_pose_projected_path_start_slice_index"] = pose_progress
        base["current_pose_projected_path_distance_m"] = round(pose_distance_m, 6) if pose_distance_m is not None else None
        base["fallback_started_at_selected_list_index"] = start_idx
        base["fallback_started_at_waypoint_index"] = int(semantic_goals[start_idx]["waypoint_index"]) if start_idx < len(semantic_goals) else None
        base["fallback_skipped_by_projection_semantic_waypoints"] = [int(w["waypoint_index"]) for w in semantic_goals[:start_idx]]
        base["fallback_skipped_completed_semantic_waypoints"] = []
        required_goal_selected_indices = set(required_interior_indices if args.enforce_interior_targets else [])
        for selected_idx, waypoint in enumerate(semantic_goals):
            row = {
                "selected_list_index": selected_idx,
                "waypoint_index": int(waypoint["waypoint_index"]),
                "source": waypoint.get("source"),
                "gateway_id": waypoint.get("gateway_id"),
                "from_room": waypoint.get("from_room"),
                "to_room": waypoint.get("to_room"),
                "plan": {"attempted": False},
                "navigate": {"attempted": False},
            }
            if selected_idx < start_idx:
                row["navigate"] = {"attempted": False, "status": "skipped_forward_progress", "success": True}
                base["waypoint_results"].append(row)
                continue
            if projected_progress is not None and int(waypoint["waypoint_index"]) < int(projected_progress) and selected_idx not in required_goal_selected_indices:
                row["navigate"] = {"attempted": False, "status": "blocked_by_forward_progress_guard", "success": False, "failure_reason": "fallback goal is behind projected FollowPath progress"}
                base["waypoint_results"].append(row)
                base.update({"fallback_backtracking_detected": True, "failure_layer": "fallback", "failure_reason": row["navigate"]["failure_reason"]})
                return base
            if args.plan_before_goal and plan_ready:
                row["plan"] = node.compute_plan(waypoint, args.plan_timeout_sec)
                planned_points = list((row.get("plan") or {}).get("path_poses") or [])
                if planned_points:
                    save_controller_path_artifact(
                        base,
                        getattr(args, "_resolved_output_json", None) or args.output_json,
                        path_id=f"fallback_attempt_{selected_idx:02d}_wp{int(waypoint['waypoint_index'])}_plan",
                        action_type="ComputePathToPose",
                        points=planned_points,
                        target=waypoint,
                        start_pose=node.current_pose(),
                        map_yaml=args.wall_crossing_map_yaml,
                    )
            save_controller_path_artifact(
                base,
                getattr(args, "_resolved_output_json", None) or args.output_json,
                path_id=f"fallback_attempt_{selected_idx:02d}_wp{int(waypoint['waypoint_index'])}",
                action_type="NavigateToPose",
                points=[p for p in [node.current_pose(), waypoint] if p],
                target=waypoint,
                start_pose=node.current_pose(),
                map_yaml=args.wall_crossing_map_yaml,
            )
            row["navigate"] = node.navigate(waypoint, args.goal_timeout_sec)
            if row["navigate"].get("success") and waypoint.get("source") == args.split_dwell_source and args.split_dwell_sec > 0.0:
                row["dwell_after_success"] = node.dwell(args.split_dwell_sec, args.split_dwell_source)
            row["final_distance_to_goal_m"] = round(pose_distance(node.current_pose(), waypoint) or 999.0, 6)
            base["waypoint_results"].append(row)
            if not row["navigate"].get("success"):
                recovery_pose = node.current_pose()
                recovery_idx, recovery_dist = nearest_path_index(waypoints, recovery_pose)
                recovery_slice = waypoints[recovery_idx:]
                recovery = {
                    "attempted": bool(follow_ready and len(recovery_slice) >= 2),
                    "reason": "navigate_to_pose_fallback_failed_use_remaining_continuous_follow_path",
                    "path_start_slice_index": recovery_idx,
                    "path_start_distance_m": round(recovery_dist, 6) if recovery_dist is not None else None,
                    "path_pose_count": len(recovery_slice),
                }
                if recovery["attempted"]:
                    recovery.update(node.follow_path(recovery_slice, args.follow_path_timeout_sec))
                    current_after_recovery = node.current_pose()
                    recovery["distance_to_terminal_after_attempt_m"] = round(pose_distance(current_after_recovery, waypoints[-1]) or 999.0, 6)
                    base["follow_path_attempts"].append(recovery)
                    base["remaining_follow_path_recovery"] = recovery
                    if recovery.get("success") or (pose_distance(current_after_recovery, waypoints[-1]) is not None and pose_distance(current_after_recovery, waypoints[-1]) <= args.goal_tolerance_m):
                        base.update({"succeeded": True, "execution_strategy": "follow_path_then_forward_only_sparse_fallback_then_remaining_follow_path_recovery", "final_arrival_success": True})
                        return base
                else:
                    base["remaining_follow_path_recovery"] = recovery
                base.update({"failure_layer": "fallback", "failure_reason": row["navigate"].get("failure_reason") or "NavigateToPose fallback failed", "first_failed_waypoint": row})
                return base
        base["succeeded"] = True
        base["sparse_fallback_clean_resume_passed"] = not base["fallback_backtracking_detected"]
        base["final_arrival_success"] = pose_distance(node.current_pose(), waypoints[-1]) is not None and pose_distance(node.current_pose(), waypoints[-1]) <= args.goal_tolerance_m
        return base
    finally:
        final_pose = None
        for _ in range(10):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
            final_pose = node.current_pose() or final_pose
        base["final_pose_observed"] = final_pose
        if base.get("start_pose_observed") and final_pose:
            base["robot_moved_distance_m"] = round(pose_distance(base["start_pose_observed"], final_pose) or 0.0, 6)
        base["trajectory_sample_count"] = len(node.trajectory_samples)
        base["trajectory_samples"] = node.trajectory_samples
        try:
            apply_wall_crossing_runtime_validation(base, args)
        except Exception as exc:
            base["wall_crossing_validation_error"] = f"{type(exc).__name__}: {exc}"
            if getattr(args, "validate_wall_crossing", False):
                base["clean_runtime_success"] = False
                base["succeeded"] = False
        try:
            resolved_output_json = getattr(args, "_resolved_output_json", None) or args.output_json
            if resolved_output_json:
                write_route_diagnostic_artifacts(Path(resolved_output_json), base, waypoints, semantic_goals, args, node=node)
        except Exception as exc:
            base["diagnostic_artifact_error"] = f"{type(exc).__name__}: {exc}"
        try:
            node.nav_client.destroy()
            node.plan_client.destroy()
            node.follow_path_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id")
    parser.add_argument("--floor-id")
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--server-timeout-sec", type=float, default=25.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=12.0)
    parser.add_argument("--follow-path-timeout-sec", type=float, default=240.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=90.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=20.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.30)
    parser.add_argument("--room-center-semantic-tolerance-m", type=float, default=0.25)
    parser.add_argument("--gateway-semantic-tolerance-m", type=float, default=0.50)
    parser.add_argument("--deviation-event-threshold-m", type=float, default=0.30)
    parser.add_argument("--progress-jump-event-threshold", type=int, default=4)
    parser.add_argument("--spin-event-sample-count", type=int, default=4)
    parser.add_argument("--resume-tolerance-m", type=float, default=1.0)
    parser.add_argument("--from-start", action="store_true", help="Require the robot to be at the beginning of the route before executing.")
    parser.add_argument("--reset-to-route-start", action="store_true", help="Try to reset the Gazebo TurtleBot3 entity to route waypoint 0 before executing.")
    parser.add_argument("--robot-model-name", default="turtlebot3_burger")
    parser.add_argument("--from-start-tolerance-m", type=float, default=1.0)
    parser.add_argument("--from-start-max-start-slice-index", type=int, default=3)
    parser.add_argument("--reset-timeout-sec", type=float, default=20.0)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--plan-before-goal", action="store_true", default=True)
    parser.add_argument("--execution-strategy", choices=["follow_path_then_forward_only_sparse_fallback", SPLIT_STRATEGY, HANDOFF_STRATEGY], default="follow_path_then_forward_only_sparse_fallback")
    parser.add_argument("--split-at-through-room-anchors", action="store_true", help="Use split FollowPath execution gated at detected through-room fold anchors.")
    parser.add_argument("--through-room-dwell-sec", type=float, default=3.0)
    parser.add_argument("--through-room-min-inside-samples", type=int, default=8)
    parser.add_argument("--semantic-anchor-tolerance-m", type=float, default=0.25)
    parser.add_argument("--split-dwell-source", default="", help="Split FollowPath at this waypoint source and hold there before continuing.")
    parser.add_argument("--split-dwell-sec", type=float, default=0.0)
    parser.add_argument("--waypoints-json", type=Path, help="Use a generated route waypoint payload instead of the historical Scene waypoint file.")
    parser.add_argument("--expected-room-chain", help="Comma-separated room chain expected for this run.")
    parser.add_argument("--expected-gateway-sequence", help="Comma-separated gateway sequence expected for this run.")
    parser.add_argument("--allow-non-scene-truth", action="store_true", help="Allow parametric routes that intentionally differ from the accepted Scene historical room chain.")
    parser.add_argument("--enforce-interior-targets", action="store_true", default=True, help="Do not let progress projection skip explicit through-room/terminal interior targets.")
    parser.add_argument("--interior-target-progress-guard-window", type=int, default=12)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--trajectory-output-json", type=Path)
    parser.add_argument("--latest-slice-output-json", type=Path)
    parser.add_argument("--validate-wall-crossing", action="store_true", help="Fail clean runtime success if trajectory crosses occupied map cells.")
    parser.add_argument("--wall-crossing-map-yaml", type=Path)
    parser.add_argument("--wall-crossing-output-json", type=Path)
    parser.add_argument("--replan-slice-from-current-pose", action="store_true", help="After a split anchor, compute current-pose paths to outbound semantic targets before FollowPath.")
    parser.add_argument("--current-pose-gateway-handoff", action="store_true", help="After a through-room anchor, compute a current-pose gateway plan and send a short staging prefix before the remaining gateway path.")
    parser.add_argument("--handoff-target-distance-m", type=float, default=1.0)
    parser.add_argument("--handoff-min-distance-m", type=float, default=0.6)
    parser.add_argument("--handoff-max-distance-m", type=float, default=1.2)
    parser.add_argument("--handoff-target-waypoint-index", type=int, default=None)
    parser.add_argument("--handoff-follow-path-timeout-sec", type=float, default=45.0)
    parser.add_argument("--handoff-staging-tolerance-m", type=float, default=0.35)
    parser.add_argument("--align-to-handoff-heading", action="store_true", help="Reserved experimental heading-alignment flag; current implementation records heading delta but does not publish direct cmd_vel rotation.")
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    out_dir = stage_output / "post_restructure_validation"
    output_json = args.output_json or out_dir / "step30s2_scene_route_rerun_report_v0_1.json"
    output_md = args.output_md or out_dir / "step30s2_scene_route_rerun_report_v0_1.md"
    trajectory_json = args.trajectory_output_json or out_dir / "step30s2_scene_route_rerun_trajectory_v0_1.json"
    args._resolved_output_json = output_json
    payload = run_route(args, stage_output)
    write_json(output_json, payload)
    write_md(output_md, payload)
    write_json(trajectory_json, {"artifact_type": "step30s2_scene_route_rerun_trajectory", "created_utc": now_iso(), "samples": payload.get("trajectory_samples", [])})
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("succeeded") else 1


if __name__ == "__main__":
    sys.exit(main())
