#!/usr/bin/env python3
"""Compare an RSLG-SLAM planned route with a Gazebo executed odom trajectory."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_pid_follower import (  # noqa: E402
    DEFAULT_PROFILE_ID,
    extract_waypoints,
    load_profile,
    normalize_angle,
    query_id_from_pid_input,
    read_json,
)


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float
    yaw: float = 0.0


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


def read_executed_csv(path: Path) -> list[Point2D]:
    rows: list[Point2D] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if "odom_x" not in row or "odom_y" not in row:
                continue
            rows.append(Point2D(as_float(row.get("odom_x")), as_float(row.get("odom_y")), as_float(row.get("odom_yaw"))))
    if not rows:
        raise ValueError(f"No odom_x/odom_y rows found in {path}")
    return rows


def transform_plan_to_odom(waypoints: list[Any], odom_start: Point2D, *, anchor: bool) -> list[Point2D]:
    if not anchor:
        return [Point2D(wp.x, wp.y, wp.yaw) for wp in waypoints]
    first = waypoints[0]
    yaw_offset = normalize_angle(odom_start.yaw - first.yaw)
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    planned: list[Point2D] = []
    for waypoint in waypoints:
        dx = waypoint.x - first.x
        dy = waypoint.y - first.y
        planned.append(
            Point2D(
                x=odom_start.x + dx * cos_yaw - dy * sin_yaw,
                y=odom_start.y + dx * sin_yaw + dy * cos_yaw,
                yaw=normalize_angle(waypoint.yaw + yaw_offset),
            )
        )
    return planned


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def point_segment_distance(point: Point2D, a: Point2D, b: Point2D) -> float:
    vx = b.x - a.x
    vy = b.y - a.y
    wx = point.x - a.x
    wy = point.y - a.y
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return distance(point, a)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    projection = Point2D(a.x + t * vx, a.y + t * vy)
    return distance(point, projection)


def nearest_path_distance(point: Point2D, planned: list[Point2D]) -> float:
    if len(planned) == 1:
        return distance(point, planned[0])
    return min(point_segment_distance(point, planned[index], planned[index + 1]) for index in range(len(planned) - 1))


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    pid_input = read_json(args.pid_input_json)
    if args.profile_json is not None:
        profile, _params = load_profile(args.profile_json, args.profile_id)
    else:
        profile = {}
    waypoints = extract_waypoints(pid_input, args.floor_id)
    executed = read_executed_csv(args.trajectory_csv)
    planned = transform_plan_to_odom(waypoints, executed[0], anchor=args.anchor_first_waypoint_to_odom_start)
    deviations = [nearest_path_distance(point, planned) for point in executed]
    query_id = query_id_from_pid_input(pid_input, args.query_id)
    summary = {
        "schema_name": "rslg_plan_vs_gazebo_executed_trajectory_comparison",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "query_id": query_id,
        "profile_id": args.profile_id,
        "pid_input_json": str(args.pid_input_json),
        "profile_json": None if args.profile_json is None else str(args.profile_json),
        "executed_trajectory_csv": str(args.trajectory_csv),
        "floor_id_filter": args.floor_id,
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "planned_points": len(planned),
        "executed_points": len(executed),
        "start_offset": round(distance(executed[0], planned[0]), 6),
        "end_offset": round(distance(executed[-1], planned[-1]), 6),
        "final_error": round(distance(executed[-1], planned[-1]), 6),
        "mean_nearest_path_deviation": round(sum(deviations) / len(deviations), 6),
        "max_nearest_path_deviation": round(max(deviations), 6),
        "first_planned_point": {"x": round(planned[0].x, 6), "y": round(planned[0].y, 6), "yaw": round(planned[0].yaw, 6)},
        "last_planned_point": {"x": round(planned[-1].x, 6), "y": round(planned[-1].y, 6), "yaw": round(planned[-1].yaw, 6)},
        "first_executed_point": {
            "x": round(executed[0].x, 6),
            "y": round(executed[0].y, 6),
            "yaw": round(executed[0].yaw, 6),
        },
        "last_executed_point": {
            "x": round(executed[-1].x, 6),
            "y": round(executed[-1].y, 6),
            "yaw": round(executed[-1].yaw, 6),
        },
        "profile_evidence_summary": profile.get("evidence_summary") if isinstance(profile, dict) else None,
        "claim_boundary": {
            "gazebo_simulation_only": True,
            "collision_free_guarantee_claimed": False,
            "nav2_required": False,
            "amcl_required": False,
            "map_server_required": False,
            "physical_robot_claimed": False,
        },
    }
    return summary


def summary_markdown(summary: dict[str, Any]) -> str:
    return "\n".join(
        [
            "# Offline Plan vs Gazebo Executed Trajectory Comparison",
            "",
            f"- query_id: `{summary['query_id']}`",
            f"- profile_id: `{summary['profile_id']}`",
            f"- executed trajectory CSV: `{summary['executed_trajectory_csv']}`",
            f"- floor filter: `{summary['floor_id_filter']}`",
            f"- anchor-first-waypoint-to-odom-start: `{summary['anchor_first_waypoint_to_odom_start']}`",
            f"- planned points: `{summary['planned_points']}`",
            f"- executed points: `{summary['executed_points']}`",
            f"- start offset: `{summary['start_offset']}` m",
            f"- end offset / final error: `{summary['final_error']}` m",
            f"- mean nearest path deviation: `{summary['mean_nearest_path_deviation']}` m",
            f"- max nearest path deviation: `{summary['max_nearest_path_deviation']}` m",
            "",
            "This is same-floor Gazebo simulation evidence from odom. It is not a Nav2, AMCL, map_server, real robot, physical stair-climbing, or global collision-free guarantee claim.",
            "",
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-input-json", type=Path, required=True)
    parser.add_argument("--trajectory-csv", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, default=None)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--floor-id", default=None)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    summary = build_summary(args)
    write_json(args.output_json, summary)
    write_text(args.output_md, summary_markdown(summary))
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
