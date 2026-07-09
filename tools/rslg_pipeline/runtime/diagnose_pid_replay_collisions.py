#!/usr/bin/env python3
"""Diagnose stable-map footprint collisions in lightweight PID replay outputs.

The tool is intentionally static and task-local: it reads existing replay
reports, trajectory CSVs, RouteResults, and PID runtime inputs, then writes
diagnostic summaries. It does not run ROS, Nav2, AMCL, Gazebo, RViz, Stage-A,
or raw RGB-D inference.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (  # noqa: E402
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
    PROJECT_NAME,
)
from tools.rslg_pipeline.runtime.replay_pid_runtime_input import (  # noqa: E402
    State,
    discover_canonical_roots,
    footprint_check,
    load_route_result,
    load_stable_maps,
    parse_floor_z_map,
    point_to_segment_distance,
    read_json,
    sanitize_waypoints,
    write_json,
)


DEFAULT_TARGET_SEGMENT = "seg_05_same_floor_metric_room_13_to_room_14"
DEFAULT_RADIUS_LIST = "0.10,0.12,0.15,0.18"


@dataclass
class PolylineNearest:
    distance_m: float
    waypoint_index: Optional[int]
    waypoint_x: Optional[float]
    waypoint_y: Optional[float]
    projection_x: Optional[float]
    projection_y: Optional[float]


def parse_radius_list(raw: str) -> list[float]:
    radii = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if not radii:
        raise ValueError("--robot-radius-list must contain at least one radius")
    return sorted(dict.fromkeys(round(radius, 6) for radius in radii))


def boolish(raw: Any) -> bool:
    if isinstance(raw, bool):
        return raw
    return str(raw).strip().lower() in {"true", "1", "yes"}


def ffloat(raw: Any) -> Optional[float]:
    if raw in {None, ""}:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def iint(raw: Any) -> Optional[int]:
    if raw in {None, ""}:
        return None
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        return None


def radius_key(radius: float) -> str:
    return f"{radius:.2f}"


def load_reports(baseline_replay_dir: Path) -> list[dict[str, Any]]:
    reports_dir = baseline_replay_dir / "reports"
    reports = []
    for path in sorted(reports_dir.glob("*_execution_report.json")):
        payload = read_json(path)
        payload["_report_path"] = path.as_posix()
        reports.append(payload)
    return reports


def load_pid_payloads(pid_inputs_dir: Path) -> dict[str, tuple[dict[str, Any], Path]]:
    payloads: dict[str, tuple[dict[str, Any], Path]] = {}
    for path in sorted(pid_inputs_dir.glob("*_pid_runtime_input.json")):
        payload = read_json(path)
        query_id = str((payload.get("identity") or {}).get("query_id") or path.stem.replace("_pid_runtime_input", ""))
        payloads[query_id] = (payload, path)
    return payloads


def choose_target_segment(reports: list[dict[str, Any]], requested: Optional[str]) -> str:
    if requested:
        return requested
    counts: dict[str, int] = {}
    for report in reports:
        for segment in report.get("segment_reports") or []:
            sid = str(segment.get("segment_id") or "")
            count = int(segment.get("collision_count") or 0)
            if sid and count:
                counts[sid] = counts.get(sid, 0) + count
    if counts:
        return sorted(counts.items(), key=lambda item: (-item[1], item[0]))[0][0]
    return DEFAULT_TARGET_SEGMENT


def read_trajectory_csv(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for index, row in enumerate(rows):
        row["_sample_index"] = index
    return rows


def trajectory_path(baseline_replay_dir: Path, query_id: str) -> Path:
    return baseline_replay_dir / "trajectories" / f"{query_id}_trajectory.csv"


def waypoint_xy(waypoint: dict[str, Any]) -> tuple[float, float]:
    return float(waypoint["x"]), float(waypoint["y"])


def segment_waypoints(pid_input: dict[str, Any], target_segment_id: str) -> list[dict[str, Any]]:
    return [wp for wp in sanitize_waypoints(pid_input) if str(wp.get("segment_id")) == target_segment_id]


def projection_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> tuple[float, float, float]:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return ax, ay, math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    qx = ax + t * vx
    qy = ay + t * vy
    return qx, qy, math.hypot(px - qx, py - qy)


def nearest_to_polyline(px: float, py: float, waypoints: list[dict[str, Any]]) -> PolylineNearest:
    nearest_wp: Optional[dict[str, Any]] = None
    nearest_wp_dist: Optional[float] = None
    for waypoint in waypoints:
        wx, wy = waypoint_xy(waypoint)
        dist = math.hypot(px - wx, py - wy)
        if nearest_wp_dist is None or dist < nearest_wp_dist:
            nearest_wp_dist = dist
            nearest_wp = waypoint

    best_dist: Optional[float] = None
    best_projection: tuple[Optional[float], Optional[float]] = (None, None)
    for before, after in zip(waypoints, waypoints[1:]):
        ax, ay = waypoint_xy(before)
        bx, by = waypoint_xy(after)
        _ = point_to_segment_distance(px, py, ax, ay, bx, by)
        qx, qy, dist = projection_on_segment(px, py, ax, ay, bx, by)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_projection = (qx, qy)

    if best_dist is None and nearest_wp_dist is not None:
        best_dist = nearest_wp_dist
        best_projection = waypoint_xy(nearest_wp or {})

    return PolylineNearest(
        distance_m=round(float(best_dist or 0.0), 6),
        waypoint_index=iint((nearest_wp or {}).get("waypoint_index")),
        waypoint_x=round(float((nearest_wp or {}).get("x")), 6) if nearest_wp else None,
        waypoint_y=round(float((nearest_wp or {}).get("y")), 6) if nearest_wp else None,
        projection_x=round(float(best_projection[0]), 6) if best_projection[0] is not None else None,
        projection_y=round(float(best_projection[1]), 6) if best_projection[1] is not None else None,
    )


def sample_segment_centerline(waypoints: list[dict[str, Any]], spacing_m: float) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    if not waypoints:
        return samples
    for pair_index, (before, after) in enumerate(zip(waypoints, waypoints[1:])):
        ax, ay = waypoint_xy(before)
        bx, by = waypoint_xy(after)
        dist = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax) if dist > 1e-12 else float(before.get("yaw") or 0.0)
        steps = max(1, int(math.ceil(dist / spacing_m)))
        if pair_index == 0:
            samples.append(
                {
                    "sample_index": len(samples),
                    "x": ax,
                    "y": ay,
                    "yaw": yaw,
                    "floor_id": before.get("floor_id"),
                    "z": before.get("z"),
                    "segment_id": before.get("segment_id"),
                    "nearest_waypoint_index": before.get("waypoint_index"),
                }
            )
        for step in range(1, steps + 1):
            alpha = step / steps
            samples.append(
                {
                    "sample_index": len(samples),
                    "x": ax + (bx - ax) * alpha,
                    "y": ay + (by - ay) * alpha,
                    "yaw": yaw,
                    "floor_id": after.get("floor_id"),
                    "z": after.get("z"),
                    "segment_id": after.get("segment_id"),
                    "nearest_waypoint_index": after.get("waypoint_index"),
                }
            )
    if len(waypoints) == 1:
        wp = waypoints[0]
        samples.append(
            {
                "sample_index": 0,
                "x": float(wp["x"]),
                "y": float(wp["y"]),
                "yaw": float(wp.get("yaw") or 0.0),
                "floor_id": wp.get("floor_id"),
                "z": wp.get("z"),
                "segment_id": wp.get("segment_id"),
                "nearest_waypoint_index": wp.get("waypoint_index"),
            }
        )
    return samples


def state_from_xy_record(record: dict[str, Any], floor_z_map: dict[str, float]) -> State:
    floor_id = str(record.get("floor_id") or "")
    return State(
        x=float(record["x"]),
        y=float(record["y"]),
        yaw=float(record.get("yaw") or 0.0),
        floor_id=floor_id,
        z=float(record.get("z") or floor_z_map.get(floor_id, 0.0)),
        segment_id=record.get("segment_id"),
        waypoint_index=int(record.get("waypoint_index") or record.get("nearest_waypoint_index") or 0),
        t=float(record.get("t") or 0.0),
    )


def footprint_counts(
    records: list[dict[str, Any]],
    stable_maps: dict[str, Any],
    radii: list[float],
    floor_z_map: dict[str, float],
) -> dict[str, dict[str, Any]]:
    counts: dict[str, dict[str, Any]] = {}
    for radius in radii:
        key = radius_key(radius)
        collision_count = 0
        invalid_count = 0
        min_clearance_values: list[float] = []
        for record in records:
            state = state_from_xy_record(record, floor_z_map)
            check = footprint_check(stable_maps, state, radius)
            if check.get("collision"):
                collision_count += 1
            if check.get("invalid"):
                invalid_count += 1
            if check.get("min_clearance_m") is not None:
                min_clearance_values.append(float(check["min_clearance_m"]))
        counts[key] = {
            "collision_count": collision_count,
            "invalid_cell_count": invalid_count,
            "min_clearance_m": round(min(min_clearance_values), 6) if min_clearance_values else None,
            "sample_count": len(records),
        }
    return counts


def annotate_collision_sample(
    query_id: str,
    row: dict[str, Any],
    segment_points: list[dict[str, Any]],
    stable_maps: dict[str, Any],
    radii: list[float],
    floor_z_map: dict[str, float],
) -> dict[str, Any]:
    x = float(row["x"])
    y = float(row["y"])
    nearest = nearest_to_polyline(x, y, segment_points)
    record: dict[str, Any] = {
        "query_id": query_id,
        "sample_index": iint(row.get("_sample_index")),
        "time_sec": ffloat(row.get("t")),
        "x": round(x, 6),
        "y": round(y, 6),
        "yaw": ffloat(row.get("yaw")),
        "floor_id": row.get("floor_id"),
        "z": ffloat(row.get("z")),
        "segment_id": row.get("segment_id"),
        "waypoint_index": iint(row.get("waypoint_index")),
        "event": row.get("event"),
        "v": ffloat(row.get("v")),
        "w": ffloat(row.get("w")),
        "tracking_error": ffloat(row.get("tracking_error")),
        "cell_state": row.get("cell_state"),
        "collision": boolish(row.get("collision")),
        "invalid_cell": boolish(row.get("invalid_cell")),
        "min_clearance_m": ffloat(row.get("min_clearance_m")),
        "nearest_waypoint_index": nearest.waypoint_index,
        "nearest_waypoint_x": nearest.waypoint_x,
        "nearest_waypoint_y": nearest.waypoint_y,
        "nearest_centerline_distance_m": nearest.distance_m,
        "nearest_centerline_projection_x": nearest.projection_x,
        "nearest_centerline_projection_y": nearest.projection_y,
    }
    state = state_from_xy_record(row, floor_z_map)
    for radius in radii:
        key = radius_key(radius)
        check = footprint_check(stable_maps, state, radius)
        record[f"radius_{key}_collision"] = bool(check.get("collision"))
        record[f"radius_{key}_invalid"] = bool(check.get("invalid"))
        record[f"radius_{key}_min_clearance_m"] = check.get("min_clearance_m")
    return record


def collision_segments_by_report(reports: list[dict[str, Any]], target_segment_id: str) -> list[dict[str, Any]]:
    rows = []
    for report in reports:
        for segment in report.get("segment_reports") or []:
            if str(segment.get("segment_id")) != target_segment_id:
                continue
            rows.append(
                {
                    "query_id": report.get("query_id"),
                    "classification": report.get("classification"),
                    "segment_id": target_segment_id,
                    "floor_id": segment.get("floor_id"),
                    "waypoints_reached": segment.get("waypoints_reached"),
                    "waypoints_total": segment.get("waypoints_total"),
                    "collision_count": int(segment.get("collision_count") or 0),
                    "invalid_cell_count": int(segment.get("invalid_cell_count") or 0),
                    "status": segment.get("status"),
                }
            )
    return rows


def write_samples_csv(path: Path, samples: list[dict[str, Any]], radii: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    base_fields = [
        "query_id",
        "sample_index",
        "time_sec",
        "x",
        "y",
        "yaw",
        "floor_id",
        "z",
        "segment_id",
        "waypoint_index",
        "event",
        "v",
        "w",
        "tracking_error",
        "cell_state",
        "collision",
        "invalid_cell",
        "min_clearance_m",
        "nearest_waypoint_index",
        "nearest_waypoint_x",
        "nearest_waypoint_y",
        "nearest_centerline_distance_m",
        "nearest_centerline_projection_x",
        "nearest_centerline_projection_y",
    ]
    radius_fields: list[str] = []
    for radius in radii:
        key = radius_key(radius)
        radius_fields.extend(
            [
                f"radius_{key}_collision",
                f"radius_{key}_invalid",
                f"radius_{key}_min_clearance_m",
            ]
        )
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=base_fields + radius_fields)
        writer.writeheader()
        for sample in samples:
            writer.writerow({field: sample.get(field, "") for field in base_fields + radius_fields})


def table_row(values: list[Any]) -> str:
    return "| " + " | ".join(str(value) for value in values) + " |"


def render_diagnostic_md(summary: dict[str, Any]) -> str:
    lines = [
        "# PID Replay Collision Diagnostic",
        "",
        f"- project: `{summary['project_name']}`",
        f"- target_segment_id: `{summary['target_segment_id']}`",
        f"- stable_map_checker: `{summary['stable_map_checker']['mode']}`",
        f"- checker_limitation: `{summary['stable_map_checker'].get('reason')}`",
        f"- robot_radius_list: `{summary['robot_radius_list']}`",
        f"- centerline_collision_at_0.18: `{summary['diagnosis']['centerline_footprint_collision_at_0_18']}`",
        f"- pid_trajectory_collision_at_0.18: `{summary['diagnosis']['pid_trajectory_footprint_collision_at_0_18']}`",
        f"- likely_root_cause: `{summary['diagnosis']['likely_root_cause']}`",
        f"- radius_sensitivity_likely: `{summary['diagnosis']['radius_sensitivity_likely']}`",
        f"- corner_cutting_likely: `{summary['diagnosis']['corner_cutting_likely']}`",
        "",
        "## Query Counts",
        "",
        table_row(["query_id", "class", "segment collisions", "centerline 0.18", "trajectory 0.18", "nearest centerline dist range"]),
        "|---|---|---:|---:|---:|---:|",
    ]
    for query in summary.get("queries") or []:
        dist_range = "n/a"
        if query.get("collision_nearest_centerline_distance_m"):
            dist = query["collision_nearest_centerline_distance_m"]
            dist_range = f"{dist['min']}..{dist['max']}"
        lines.append(
            table_row(
                [
                    f"`{query['query_id']}`",
                    f"`{query.get('classification')}`",
                    query.get("target_segment_collision_count"),
                    query.get("centerline_radius_counts", {}).get("0.18", {}).get("collision_count"),
                    query.get("trajectory_radius_counts", {}).get("0.18", {}).get("collision_count"),
                    dist_range,
                ]
            )
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            summary["diagnosis"]["interpretation"],
            "",
            "The diagnostic only reports map-backed footprint checks when the stable-map package can be loaded. It does not infer or fake collision-free status from missing map data.",
            "",
        ]
    )
    return "\n".join(lines)


def write_overlay_svg(path: Path, summary: dict[str, Any], samples: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    points: list[tuple[float, float]] = []
    for query in summary.get("queries") or []:
        for waypoint in query.get("target_segment_waypoints") or []:
            points.append((float(waypoint["x"]), float(waypoint["y"])))
    for sample in samples:
        points.append((float(sample["x"]), float(sample["y"])))
    if not points:
        path.write_text("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"640\" height=\"220\"></svg>\n", encoding="utf-8")
        return

    min_x = min(x for x, _ in points)
    max_x = max(x for x, _ in points)
    min_y = min(y for _, y in points)
    max_y = max(y for _, y in points)
    span_x = max(max_x - min_x, 0.5)
    span_y = max(max_y - min_y, 0.5)
    width = 920
    height = 520
    margin = 42
    scale = min((width - margin * 2) / span_x, (height - margin * 2) / span_y)

    def tx(x: float, y: float) -> tuple[float, float]:
        return margin + (x - min_x) * scale, height - margin - (y - min_y) * scale

    palette = ["#dc2626", "#2563eb", "#9333ea", "#ea580c", "#0891b2", "#16a34a"]
    body: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f"<title>{html.escape(summary['target_segment_id'])} collision overlay</title>",
        "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>",
        f"<text x=\"20\" y=\"28\" font-family=\"Arial, sans-serif\" font-size=\"15\" fill=\"#111827\">{html.escape(summary['target_segment_id'])}</text>",
    ]
    seen_polylines = set()
    for query_index, query in enumerate(summary.get("queries") or []):
        waypoints = query.get("target_segment_waypoints") or []
        encoded = " ".join(f"{tx(float(wp['x']), float(wp['y']))[0]:.2f},{tx(float(wp['x']), float(wp['y']))[1]:.2f}" for wp in waypoints)
        if encoded and encoded not in seen_polylines:
            seen_polylines.add(encoded)
            body.append(f'<polyline points="{encoded}" fill="none" stroke="#6b7280" stroke-width="3" stroke-dasharray="8 5" stroke-linecap="round" stroke-linejoin="round"/>')
        color = palette[query_index % len(palette)]
        for sample in samples:
            if sample.get("query_id") != query.get("query_id"):
                continue
            sx, sy = tx(float(sample["x"]), float(sample["y"]))
            body.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="4.5" fill="{color}" fill-opacity="0.78"/>')
    body.append("</svg>")
    path.write_text("\n".join(body) + "\n", encoding="utf-8")


def build_diagnosis(
    *,
    reports: list[dict[str, Any]],
    baseline_replay_dir: Path,
    route_results_dir: Path,
    pid_inputs_dir: Path,
    target_segment_id: str,
    radii: list[float],
    floor_z_map: dict[str, float],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    pid_by_query = load_pid_payloads(pid_inputs_dir)
    route_results: list[Optional[dict[str, Any]]] = []
    pid_payloads: list[dict[str, Any]] = []
    route_result_paths: dict[str, Optional[Path]] = {}
    for query_id, (pid_payload, pid_path) in pid_by_query.items():
        route_result, route_path = load_route_result(pid_payload, pid_path, route_results_dir)
        route_results.append(route_result)
        pid_payloads.append(pid_payload)
        route_result_paths[query_id] = route_path

    stable_maps, stable_map_info = load_stable_maps(discover_canonical_roots(pid_payloads, route_results))
    map_available = stable_map_info.get("mode") == "stable_map_footprint" and bool(stable_maps)

    segment_rows = collision_segments_by_report(reports, target_segment_id)
    samples: list[dict[str, Any]] = []
    query_summaries: list[dict[str, Any]] = []
    all_centerline_collision_018 = False
    all_pid_collision_018 = False
    radius_sensitive = False
    max_collision_centerline_distance = 0.0

    for report in reports:
        query_id = str(report.get("query_id") or "")
        pid_payload, _pid_path = pid_by_query.get(query_id, ({}, Path("")))
        segment_points = segment_waypoints(pid_payload, target_segment_id)
        trajectory_rows = []
        path = trajectory_path(baseline_replay_dir, query_id)
        if path.exists():
            trajectory_rows = [
                row
                for row in read_trajectory_csv(path)
                if str(row.get("segment_id")) == target_segment_id
            ]
        collision_rows = [row for row in trajectory_rows if boolish(row.get("collision"))]

        centerline_samples = sample_segment_centerline(segment_points, spacing_m=0.025)
        if map_available:
            centerline_counts = footprint_counts(centerline_samples, stable_maps, radii, floor_z_map)
            trajectory_counts = footprint_counts(trajectory_rows, stable_maps, radii, floor_z_map)
        else:
            centerline_counts = {
                radius_key(radius): {
                    "collision_count": None,
                    "invalid_cell_count": None,
                    "min_clearance_m": None,
                    "sample_count": len(centerline_samples),
                }
                for radius in radii
            }
            trajectory_counts = {
                radius_key(radius): {
                    "collision_count": None,
                    "invalid_cell_count": None,
                    "min_clearance_m": None,
                    "sample_count": len(trajectory_rows),
                }
                for radius in radii
            }

        nearest_distances: list[float] = []
        for row in collision_rows:
            sample = annotate_collision_sample(query_id, row, segment_points, stable_maps, radii, floor_z_map)
            samples.append(sample)
            nearest_distances.append(float(sample["nearest_centerline_distance_m"]))
        if nearest_distances:
            max_collision_centerline_distance = max(max_collision_centerline_distance, max(nearest_distances))

        c018 = centerline_counts.get("0.18", {}).get("collision_count")
        p018 = trajectory_counts.get("0.18", {}).get("collision_count")
        if isinstance(c018, int) and c018 > 0:
            all_centerline_collision_018 = True
        if isinstance(p018, int) and p018 > 0:
            all_pid_collision_018 = True
        min_key = radius_key(min(radii))
        if isinstance(p018, int) and isinstance(trajectory_counts.get(min_key, {}).get("collision_count"), int):
            min_radius_count = int(trajectory_counts[min_key]["collision_count"])
            if p018 > min_radius_count:
                radius_sensitive = True

        segment_report = next((row for row in segment_rows if row.get("query_id") == query_id), {})
        query_summaries.append(
            {
                "query_id": query_id,
                "classification": report.get("classification"),
                "target_segment_id": target_segment_id,
                "target_segment_floor_id": segment_report.get("floor_id"),
                "target_segment_collision_count": segment_report.get("collision_count", 0),
                "target_segment_invalid_cell_count": segment_report.get("invalid_cell_count", 0),
                "target_segment_status": segment_report.get("status"),
                "target_segment_waypoints": [
                    {
                        "waypoint_index": wp.get("waypoint_index"),
                        "x": wp.get("x"),
                        "y": wp.get("y"),
                        "yaw": wp.get("yaw"),
                        "floor_id": wp.get("floor_id"),
                    }
                    for wp in segment_points
                ],
                "target_segment_waypoint_count": len(segment_points),
                "trajectory_sample_count": len(trajectory_rows),
                "collision_sample_count": len(collision_rows),
                "centerline_sample_count": len(centerline_samples),
                "centerline_radius_counts": centerline_counts,
                "trajectory_radius_counts": trajectory_counts,
                "collision_nearest_centerline_distance_m": (
                    {
                        "min": round(min(nearest_distances), 6),
                        "max": round(max(nearest_distances), 6),
                        "mean": round(sum(nearest_distances) / len(nearest_distances), 6),
                    }
                    if nearest_distances
                    else None
                ),
                "route_result_path": route_result_paths.get(query_id).as_posix() if route_result_paths.get(query_id) else None,
            }
        )

    centerline_unknown = not map_available
    corner_cutting_likely = (
        not centerline_unknown
        and not all_centerline_collision_018
        and all_pid_collision_018
        and max_collision_centerline_distance > 0.02
    )
    if centerline_unknown:
        root_cause = "stable_map_checker_unavailable"
        interpretation = (
            "The stable-map footprint checker was not available, so the diagnostic cannot classify centerline "
            "versus PID trajectory collisions without faking map-backed results."
        )
    elif all_centerline_collision_018:
        root_cause = "planned_centerline_or_stable_map_boundary_clearance"
        interpretation = (
            "The planned segment centerline itself collides with the stable-map footprint at robot_radius=0.18. "
            "That points to route clearance or stable-map boundary sensitivity rather than a pure PID corner-cutting issue."
        )
    elif corner_cutting_likely:
        root_cause = "pid_corner_cutting"
        interpretation = (
            "The planned centerline is clear at robot_radius=0.18 while PID samples collide and deviate from the "
            "polyline near turns. That pattern is consistent with PID corner-cutting."
        )
    elif all_pid_collision_018 and radius_sensitive:
        root_cause = "footprint_clearance_sensitivity"
        interpretation = (
            "The PID trajectory collides at robot_radius=0.18 and improves at smaller radii. This indicates "
            "footprint-clearance sensitivity."
        )
    elif all_pid_collision_018:
        root_cause = "pid_trajectory_only_collision"
        interpretation = (
            "The PID trajectory collides while the centerline does not, but the measured deviation is small. "
            "This is a trajectory-only collision requiring tuning or replay-time waypoint shaping."
        )
    else:
        root_cause = "no_target_segment_collision_reproduced"
        interpretation = "No target-segment footprint collision was reproduced in the loaded trajectory samples."

    summary = {
        "schema_name": "rslg_pid_replay_collision_diagnostic",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "target_segment_id": target_segment_id,
        "baseline_replay_dir": baseline_replay_dir.as_posix(),
        "route_results_dir": route_results_dir.as_posix(),
        "pid_inputs_dir": pid_inputs_dir.as_posix(),
        "robot_radius_list": radii,
        "floor_z_map": floor_z_map,
        "stable_map_checker": stable_map_info,
        "query_count": len(reports),
        "queries_with_target_segment_collisions": [
            row["query_id"] for row in segment_rows if int(row.get("collision_count") or 0) > 0
        ],
        "target_segment_rows_from_reports": segment_rows,
        "collision_sample_count": len(samples),
        "diagnosis": {
            "centerline_footprint_collision_at_0_18": all_centerline_collision_018 if map_available else None,
            "pid_trajectory_footprint_collision_at_0_18": all_pid_collision_018 if map_available else None,
            "radius_sensitivity_likely": radius_sensitive if map_available else None,
            "corner_cutting_likely": corner_cutting_likely if map_available else None,
            "max_collision_nearest_centerline_distance_m": round(max_collision_centerline_distance, 6),
            "likely_root_cause": root_cause,
            "interpretation": interpretation,
        },
        "guard_counts_from_reports": {
            "generated_ring_037_selected_count": sum(
                1
                for report in reports
                if ((report.get("selected_object_approach") or {}).get("candidate_id") in BLOCKED_LEGACY_APPROACH_IDS)
                or report.get("blocked_candidate_selected")
            ),
            "vt_1_centerline_e003_transition_count": sum(
                1
                for report in reports
                if NON_TRANSITION_EDGE in set(report.get("transition_edges") or []) or report.get("forbidden_transition_used")
            ),
        },
        "queries": query_summaries,
    }
    return summary, samples


def run(args: argparse.Namespace) -> int:
    baseline_replay_dir = args.baseline_replay_dir
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    reports = load_reports(baseline_replay_dir)
    if not reports:
        raise SystemExit(f"no execution reports found under {baseline_replay_dir / 'reports'}")
    target_segment_id = choose_target_segment(reports, args.target_segment_id)
    radii = parse_radius_list(args.robot_radius_list)
    floor_z_map = parse_floor_z_map(args.floor_z_map)

    summary, samples = build_diagnosis(
        reports=reports,
        baseline_replay_dir=baseline_replay_dir,
        route_results_dir=args.route_results_dir,
        pid_inputs_dir=args.pid_inputs_dir,
        target_segment_id=target_segment_id,
        radii=radii,
        floor_z_map=floor_z_map,
    )

    stem = "seg_05" if target_segment_id == DEFAULT_TARGET_SEGMENT else target_segment_id.replace("/", "_")
    write_json(output_dir / "summary.json", summary)
    write_json(output_dir / f"{stem}_collision_samples.json", samples)
    write_samples_csv(output_dir / f"{stem}_collision_samples.csv", samples, radii)
    (output_dir / f"{stem}_diagnostic.md").write_text(render_diagnostic_md(summary), encoding="utf-8")
    write_overlay_svg(output_dir / f"{stem}_collision_overlay.svg", summary, samples)

    print(
        json.dumps(
            {
                "classification": "pid_replay_collision_diagnostic_completed",
                "target_segment_id": target_segment_id,
                "collision_sample_count": len(samples),
                "likely_root_cause": summary["diagnosis"]["likely_root_cause"],
                "output_dir": output_dir.as_posix(),
            },
            indent=2,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-replay-dir", type=Path, required=True)
    parser.add_argument("--route-results-dir", type=Path, required=True)
    parser.add_argument("--pid-inputs-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-segment-id", default=None)
    parser.add_argument("--robot-radius-list", default=DEFAULT_RADIUS_LIST)
    parser.add_argument("--floor-z-map", default=None)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
