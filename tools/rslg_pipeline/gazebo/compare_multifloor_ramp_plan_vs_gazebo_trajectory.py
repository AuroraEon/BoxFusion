#!/usr/bin/env python3
"""Compare a task59 ramp planned path with a Gazebo odom trajectory."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_multifloor_ramp_follower import Waypoint, count_by_segment, extract_waypoints, query_id_from_runtime_input  # noqa: E402
from rslg_gazebo_pid_follower import normalize_angle, read_json  # noqa: E402


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    segment: str = "unknown"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def waypoint_segment(waypoint: Waypoint) -> str:
    if waypoint.is_ramp_waypoint:
        return "ramp"
    if waypoint.floor_id == "floor_1":
        return "floor_1"
    if waypoint.floor_id == "floor_2":
        return "floor_2"
    return "other"


def planned_points_from_waypoints(waypoints: list[Waypoint]) -> list[Point3D]:
    return [Point3D(wp.x, wp.y, wp.z, wp.yaw, waypoint_segment(wp)) for wp in waypoints]


def read_executed_csv(path: Path) -> list[Point3D]:
    rows: list[Point3D] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if "odom_x" not in row or "odom_y" not in row:
                continue
            rows.append(
                Point3D(
                    x=as_float(row.get("odom_x")),
                    y=as_float(row.get("odom_y")),
                    z=as_float(row.get("odom_z")),
                    yaw=as_float(row.get("odom_yaw")),
                    segment="executed",
                )
            )
    if not rows:
        raise ValueError(f"No odom_x/odom_y rows found in {path}")
    return rows


def transform_plan_to_odom(planned: list[Point3D], odom_start: Point3D, *, anchor: bool) -> list[Point3D]:
    if not anchor:
        return planned
    first = planned[0]
    yaw_offset = normalize_angle(odom_start.yaw - first.yaw)
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    transformed: list[Point3D] = []
    for point in planned:
        dx = point.x - first.x
        dy = point.y - first.y
        transformed.append(
            Point3D(
                x=odom_start.x + dx * cos_yaw - dy * sin_yaw,
                y=odom_start.y + dx * sin_yaw + dy * cos_yaw,
                z=odom_start.z + (point.z - first.z),
                yaw=normalize_angle(point.yaw + yaw_offset),
                segment=point.segment,
            )
        )
    return transformed


def distance_xy(a: Point3D, b: Point3D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def point_segment_distance(point: Point3D, a: Point3D, b: Point3D) -> float:
    vx = b.x - a.x
    vy = b.y - a.y
    wx = point.x - a.x
    wy = point.y - a.y
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return distance_xy(point, a)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    projection = Point3D(a.x + t * vx, a.y + t * vy)
    return distance_xy(point, projection)


def nearest_path_distance(point: Point3D, planned: list[Point3D]) -> tuple[float, str]:
    if len(planned) == 1:
        return distance_xy(point, planned[0]), planned[0].segment
    best_distance = float("inf")
    best_segment = "unknown"
    for index in range(len(planned) - 1):
        distance = point_segment_distance(point, planned[index], planned[index + 1])
        if distance < best_distance:
            best_distance = distance
            if planned[index].segment == planned[index + 1].segment:
                best_segment = planned[index].segment
            else:
                best_segment = planned[index + 1].segment
    return best_distance, best_segment


def summarize_deviation(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "max": None}
    return {"count": len(values), "mean": round(sum(values) / len(values), 6), "max": round(max(values), 6)}


def build_skipped_summary(args: argparse.Namespace, reason: str) -> dict[str, Any]:
    runtime_input = read_json(args.runtime_input_json)
    query_id = query_id_from_runtime_input(runtime_input, args.query_id)
    waypoints = extract_waypoints(runtime_input, "all")
    return {
        "schema_name": "rslg_multifloor_ramp_plan_vs_gazebo_comparison",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "status": "skipped",
        "skip_reason": reason,
        "query_id": query_id,
        "runtime_input_json": str(args.runtime_input_json),
        "executed_trajectory_csv": None if args.trajectory_csv is None else str(args.trajectory_csv),
        "planned_point_count": len(waypoints),
        "segment_waypoints_total": count_by_segment(waypoints),
        "claim_boundary": {
            "gazebo_simulation_only": True,
            "physical_stair_climbing_claimed": False,
            "physical_robot_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
            "global_collision_free_guarantee_claimed": False,
        },
    }


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    runtime_input = read_json(args.runtime_input_json)
    query_id = query_id_from_runtime_input(runtime_input, args.query_id)
    waypoints = extract_waypoints(runtime_input, "all")
    planned_raw = planned_points_from_waypoints(waypoints)
    if args.trajectory_csv is None:
        if args.allow_missing:
            return build_skipped_summary(args, "no trajectory CSV was provided")
        raise ValueError("--trajectory-csv is required unless --allow-missing is used")
    if not args.trajectory_csv.is_file():
        if args.allow_missing:
            return build_skipped_summary(args, f"trajectory CSV not found: {args.trajectory_csv}")
        raise FileNotFoundError(args.trajectory_csv)
    executed = read_executed_csv(args.trajectory_csv)
    planned = transform_plan_to_odom(planned_raw, executed[0], anchor=args.anchor_first_waypoint_to_odom_start)
    deviations_by_segment: dict[str, list[float]] = {"floor_1": [], "ramp": [], "floor_2": [], "other": []}
    deviations: list[float] = []
    ramp_progression_hits = 0
    for point in executed:
        distance, segment = nearest_path_distance(point, planned)
        deviations.append(distance)
        deviations_by_segment.setdefault(segment, []).append(distance)
        if segment == "ramp":
            ramp_progression_hits += 1
    odom_z_values = [point.z for point in executed]
    odom_z_changed = max(odom_z_values) - min(odom_z_values) > 0.02 if odom_z_values else False
    summary = {
        "schema_name": "rslg_multifloor_ramp_plan_vs_gazebo_comparison",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "status": "completed",
        "query_id": query_id,
        "runtime_input_json": str(args.runtime_input_json),
        "executed_trajectory_csv": str(args.trajectory_csv),
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "planned_point_count": len(planned),
        "executed_point_count": len(executed),
        "segment_waypoints_total": count_by_segment(waypoints),
        "start_offset": round(distance_xy(executed[0], planned[0]), 6),
        "end_offset": round(distance_xy(executed[-1], planned[-1]), 6),
        "final_error": round(distance_xy(executed[-1], planned[-1]), 6),
        "mean_nearest_path_deviation": round(sum(deviations) / len(deviations), 6),
        "max_nearest_path_deviation": round(max(deviations), 6),
        "segment_deviations": {key: summarize_deviation(value) for key, value in deviations_by_segment.items()},
        "executed_path_includes_ramp_portion_by_xy_progression": ramp_progression_hits > 0,
        "ramp_progression_nearest_point_count": ramp_progression_hits,
        "odom_z_min": round(min(odom_z_values), 6) if odom_z_values else None,
        "odom_z_max": round(max(odom_z_values), 6) if odom_z_values else None,
        "odom_z_changed": odom_z_changed,
        "odom_z_note": "Gazebo/TurtleBot odom may remain planar; z constancy is reported but not treated as a standalone failure.",
        "first_planned_point": planned[0].__dict__,
        "last_planned_point": planned[-1].__dict__,
        "first_executed_point": executed[0].__dict__,
        "last_executed_point": executed[-1].__dict__,
        "claim_boundary": {
            "gazebo_simulation_only": True,
            "physical_stair_climbing_claimed": False,
            "physical_robot_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
            "global_collision_free_guarantee_claimed": False,
        },
    }
    return summary


def summary_markdown(summary: dict[str, Any]) -> str:
    if summary.get("status") == "skipped":
        return "\n".join(
            [
                "# Offline Plan vs Executed Ramp Comparison",
                "",
                "- status: `skipped`",
                f"- reason: {summary.get('skip_reason')}",
                f"- query_id: `{summary.get('query_id')}`",
                f"- planned points: `{summary.get('planned_point_count')}`",
                "",
                "No Gazebo execution trajectory was available, so no path deviation metrics were computed.",
                "",
            ]
        )
    segment_deviations = summary.get("segment_deviations") or {}
    return "\n".join(
        [
            "# Offline Plan vs Executed Ramp Comparison",
            "",
            "- status: `completed`",
            f"- query_id: `{summary['query_id']}`",
            f"- executed trajectory CSV: `{summary['executed_trajectory_csv']}`",
            f"- planned points: `{summary['planned_point_count']}`",
            f"- executed points: `{summary['executed_point_count']}`",
            f"- final error: `{summary['final_error']}` m",
            f"- mean nearest path deviation: `{summary['mean_nearest_path_deviation']}` m",
            f"- max nearest path deviation: `{summary['max_nearest_path_deviation']}` m",
            f"- floor_1 deviation: `{segment_deviations.get('floor_1')}`",
            f"- ramp deviation: `{segment_deviations.get('ramp')}`",
            f"- floor_2 deviation: `{segment_deviations.get('floor_2')}`",
            f"- start offset: `{summary['start_offset']}` m",
            f"- end offset: `{summary['end_offset']}` m",
            f"- executed path includes ramp portion by XY progression: `{summary['executed_path_includes_ramp_portion_by_xy_progression']}`",
            f"- odom z changed: `{summary['odom_z_changed']}`",
            "",
            "This is simplified Gazebo ramp simulation evidence. It is not physical stair climbing, not a real robot claim, "
            "and not a Nav2/AMCL/map_server result.",
            "",
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--trajectory-csv", type=Path, default=None)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--allow-missing", action="store_true")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    summary = build_summary(args)
    write_json(args.output_json, summary)
    write_text(args.output_md, summary_markdown(summary))
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
