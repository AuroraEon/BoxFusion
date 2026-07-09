#!/usr/bin/env python3
"""Compare task60 scripted stair composite plan against an execution trajectory."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_scripted_stair_demo_orchestrator import query_id_from_runtime_input  # noqa: E402
from scripted_stair_transition_common import (  # noqa: E402
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
    AnchorTransform,
    Pose3D,
    count_route_segments,
    distance_xy,
    normalize_angle,
    read_json,
    route_points,
    transform_points,
    utc_now,
    write_json,
    write_text,
)


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def read_executed_csv(path: Path) -> list[Pose3D]:
    rows: list[Pose3D] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for index, row in enumerate(csv.DictReader(handle)):
            if "odom_x" not in row or "odom_y" not in row:
                continue
            segment = row.get("target_segment") or row.get("state") or "executed"
            if segment == "scripted_stair_transition":
                segment = "scripted_stair"
            rows.append(
                Pose3D(
                    x=as_float(row.get("odom_x")),
                    y=as_float(row.get("odom_y")),
                    z=as_float(row.get("odom_z")),
                    yaw=as_float(row.get("odom_yaw")),
                    segment=segment,
                    index=index,
                    scripted_transition=str(row.get("scripted_transition")).lower() == "true",
                    source_edge_id=row.get("source_edge_id") or None,
                )
            )
    if not rows:
        raise ValueError(f"No executed odom rows found in {path}")
    return rows


def point_segment_distance(point: Pose3D, a: Pose3D, b: Pose3D) -> float:
    vx = b.x - a.x
    vy = b.y - a.y
    wx = point.x - a.x
    wy = point.y - a.y
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return distance_xy(point, a)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    projection = Pose3D(a.x + t * vx, a.y + t * vy, a.z + (b.z - a.z) * t, a.yaw, segment=a.segment)
    return distance_xy(point, projection)


def nearest_path_distance(point: Pose3D, planned: list[Pose3D]) -> tuple[float, str]:
    if len(planned) == 1:
        return distance_xy(point, planned[0]), planned[0].segment
    best_distance = float("inf")
    best_segment = "other"
    for index in range(len(planned) - 1):
        distance = point_segment_distance(point, planned[index], planned[index + 1])
        if distance < best_distance:
            best_distance = distance
            best_segment = planned[index].segment if planned[index].segment == planned[index + 1].segment else planned[index + 1].segment
    return best_distance, best_segment


def summarize(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "max": None}
    return {"count": len(values), "mean": round(sum(values) / len(values), 6), "max": round(max(values), 6)}


def anchor_plan_to_executed(planned: list[Pose3D], executed: list[Pose3D], *, anchor: bool) -> list[Pose3D]:
    if not anchor or not planned or not executed:
        return planned
    first = planned[0]
    odom = executed[0]
    anchor_transform = AnchorTransform(
        route_x=first.x,
        route_y=first.y,
        route_z=first.z,
        route_yaw=first.yaw,
        odom_x=odom.x,
        odom_y=odom.y,
        odom_z=odom.z,
        odom_yaw=odom.yaw,
    )
    return transform_points(planned, anchor_transform)


def build_skipped_summary(args: argparse.Namespace, runtime_input: dict[str, Any], reason: str) -> dict[str, Any]:
    planned = route_points(runtime_input)
    return {
        "schema_name": "rslg_scripted_stair_composite_path_comparison",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "status": "skipped",
        "skip_reason": reason,
        "query_id": query_id_from_runtime_input(runtime_input, args.query_id),
        "runtime_input_json": str(args.runtime_input_json),
        "executed_trajectory_csv": None if args.trajectory_csv is None else str(args.trajectory_csv),
        "planned_composite_point_count": len(planned),
        "executed_composite_point_count": 0,
        "segment_waypoints_total": count_route_segments(planned),
        "transition_completion": None,
        "scripted_transition_marked": all(point.scripted_transition for point in planned if point.segment == "scripted_stair"),
        "transition_edge_id": runtime_input.get("transition_edge_id"),
        "physical_stair_climbing_claim": runtime_input.get("physical_stair_climbing_claim"),
        "claim_boundary": runtime_input.get("claim_boundary"),
    }


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    runtime_input = read_json(args.runtime_input_json)
    planned_raw = route_points(runtime_input)
    query_id = query_id_from_runtime_input(runtime_input, args.query_id)
    if args.trajectory_csv is None:
        if args.allow_missing:
            return build_skipped_summary(args, runtime_input, "no trajectory CSV was provided")
        raise ValueError("--trajectory-csv is required unless --allow-missing is used")
    if not args.trajectory_csv.is_file():
        if args.allow_missing:
            return build_skipped_summary(args, runtime_input, f"trajectory CSV not found: {args.trajectory_csv}")
        raise FileNotFoundError(args.trajectory_csv)
    executed = read_executed_csv(args.trajectory_csv)
    planned = anchor_plan_to_executed(planned_raw, executed, anchor=args.anchor_first_waypoint_to_odom_start)
    deviations: list[float] = []
    by_segment: dict[str, list[float]] = {"floor_1": [], "scripted_stair": [], "floor_2": [], "other": []}
    stair_hits = 0
    for point in executed:
        distance, segment = nearest_path_distance(point, planned)
        deviations.append(distance)
        by_segment.setdefault(segment, []).append(distance)
        if segment == "scripted_stair":
            stair_hits += 1
    final_error = round(distance_xy(executed[-1], planned[-1]), 6) if planned and executed else None
    execution_summary = read_json(args.execution_summary_json) if args.execution_summary_json and args.execution_summary_json.is_file() else {}
    return {
        "schema_name": "rslg_scripted_stair_composite_path_comparison",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "status": "completed",
        "query_id": query_id,
        "runtime_input_json": str(args.runtime_input_json),
        "executed_trajectory_csv": str(args.trajectory_csv),
        "execution_summary_json": None if args.execution_summary_json is None else str(args.execution_summary_json),
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "planned_composite_point_count": len(planned),
        "executed_composite_point_count": len(executed),
        "segment_waypoints_total": count_route_segments(planned),
        "final_error": final_error,
        "mean_nearest_path_deviation": round(sum(deviations) / len(deviations), 6) if deviations else None,
        "max_nearest_path_deviation": round(max(deviations), 6) if deviations else None,
        "segment_deviations": {key: summarize(value) for key, value in by_segment.items()},
        "transition_completion": stair_hits > 0 or execution_summary.get("stair_transition_status") == "passed_scripted",
        "scripted_stair_nearest_point_count": stair_hits,
        "scripted_transition_marked": all(point.scripted_transition for point in planned if point.segment == "scripted_stair"),
        "transition_edge_id": TRUE_TRANSITION_EDGE,
        "physical_stair_climbing_claim": runtime_input.get("physical_stair_climbing_claim"),
        "execution_status": execution_summary.get("status"),
        "floor_1_status": execution_summary.get("floor_1_status"),
        "stair_transition_status": execution_summary.get("stair_transition_status"),
        "floor_2_status": execution_summary.get("floor_2_status"),
        "final_target_status": execution_summary.get("final_target_status"),
        "claim_boundary": runtime_input.get("claim_boundary"),
    }


def summary_markdown(summary: dict[str, Any]) -> str:
    if summary.get("status") == "skipped":
        return "\n".join(
            [
                "# Offline Composite Path Summary",
                "",
                "- status: `skipped`",
                f"- reason: `{summary.get('skip_reason')}`",
                f"- planned composite points: `{summary.get('planned_composite_point_count')}`",
                f"- scripted transition marked: `{summary.get('scripted_transition_marked')}`",
                f"- physical stair-climbing claim: `{summary.get('physical_stair_climbing_claim')}`",
                "",
                "No execution trajectory was available, so deviation metrics were not computed.",
                "",
            ]
        )
    return "\n".join(
        [
            "# Offline Composite Path Summary",
            "",
            "- status: `completed`",
            f"- trajectory source: `{summary.get('executed_trajectory_csv')}`",
            f"- planned composite points: `{summary.get('planned_composite_point_count')}`",
            f"- executed composite points: `{summary.get('executed_composite_point_count')}`",
            f"- final error: `{summary.get('final_error')}`",
            f"- mean nearest-path deviation: `{summary.get('mean_nearest_path_deviation')}`",
            f"- max nearest-path deviation: `{summary.get('max_nearest_path_deviation')}`",
            f"- floor_1 deviation: `{summary.get('segment_deviations', {}).get('floor_1')}`",
            f"- scripted stair deviation: `{summary.get('segment_deviations', {}).get('scripted_stair')}`",
            f"- floor_2 deviation: `{summary.get('segment_deviations', {}).get('floor_2')}`",
            f"- transition completion: `{summary.get('transition_completion')}`",
            f"- scripted transition marked: `{summary.get('scripted_transition_marked')}`",
            f"- physical stair-climbing claim: `{summary.get('physical_stair_climbing_claim')}`",
            "",
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--trajectory-csv", type=Path, default=None)
    parser.add_argument("--execution-summary-json", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--allow-missing", action="store_true")
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
