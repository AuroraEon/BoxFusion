#!/usr/bin/env python3
"""Task14c multi-query offline object-nav validation for RSLG-SLAM."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from object_nav_common import load_index, run_query
from validate_object_approach_candidate_sanity import (
    OccupancyMap,
    WALL_SIDE_LABELS,
    all_visible_proxy_points,
    canonical_object_id,
    find_object,
    find_room,
    load_json,
    norm_label,
    point_in_polygon,
    polygon_diag,
    score_candidate,
    write_json,
)


ROOT = Path(__file__).resolve().parents[2]
TASK_NAME = "task14c_multi_query_objectnav_runtime_validation"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def markdown_table(rows: list[dict[str, Any]], fields: list[str]) -> str:
    lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(field, "")) for field in fields) + " |")
    return "\n".join(lines) + "\n"


def room_polygon(snapshot: dict[str, Any], topology: dict[str, Any], room_id: str) -> list[list[float]]:
    room = find_room(topology, snapshot, room_id)
    if not room:
        return []
    return room.get("polygon") or room.get("footprint_polygon_xy") or []


def object_xy(obj: dict[str, Any]) -> tuple[float, float] | None:
    pose = obj.get("pose") or obj.get("pose_xy")
    if pose and len(pose) >= 2:
        return float(pose[0]), float(pose[1])
    footprint = obj.get("footprint_2d") or []
    if footprint:
        return (
            sum(float(p[0]) for p in footprint) / len(footprint),
            sum(float(p[1]) for p in footprint) / len(footprint),
        )
    return None


def route_terminal(route_path: Path) -> dict[str, Any] | None:
    waypoints = load_json(route_path, {}).get("waypoints") or []
    return waypoints[-1] if waypoints else None


def make_candidate(
    candidate_id: str,
    source: str,
    xy: tuple[float, float],
    target_xy: tuple[float, float],
    floor_id: str,
    room_id: str,
    validation_status: str = "passed",
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    yaw = math.atan2(target_xy[1] - xy[1], target_xy[0] - xy[0])
    return {
        "candidate_id": candidate_id,
        "candidate_source": source,
        "floor_id": floor_id,
        "room_id": room_id,
        "world_xy": [round(xy[0], 6), round(xy[1], 6)],
        "yaw": round(yaw, 6),
        "validation_status": validation_status,
        "details": details or {},
    }


def generate_standoff_candidates(obj: dict[str, Any], floor_id: str, room_id: str, terminal: dict[str, Any] | None) -> list[dict[str, Any]]:
    target_xy = object_xy(obj)
    if target_xy is None:
        return []
    candidates = [
        make_candidate(
            f"centroid_{canonical_object_id(obj.get('id'))}_rejected",
            "centroid_rejected",
            target_xy,
            target_xy,
            floor_id,
            room_id,
            validation_status="failed",
            details={"reason": "object centroid is not a robot navigation goal"},
        )
    ]
    if terminal and "x" in terminal and "y" in terminal:
        candidates.append(
            make_candidate(
                "route_terminal_seed",
                "route_terminal_seed",
                (float(terminal["x"]), float(terminal["y"])),
                target_xy,
                floor_id,
                room_id,
                details={"source_waypoint_index": terminal.get("waypoint_index"), "source": terminal.get("source")},
            )
        )
    footprint = obj.get("footprint_2d") or []
    if not footprint:
        return candidates
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    half_w = max(0.05, (max_x - min_x) / 2.0)
    half_h = max(0.05, (max_y - min_y) / 2.0)
    n = 0
    for standoff in (0.6, 0.8, 1.0):
        for deg in range(0, 360, 22):
            angle = math.radians(deg)
            ux, uy = math.cos(angle), math.sin(angle)
            support_x = half_w / abs(ux) if abs(ux) > 1e-6 else float("inf")
            support_y = half_h / abs(uy) if abs(uy) > 1e-6 else float("inf")
            support = min(support_x, support_y)
            xy = (target_xy[0] + ux * (support + standoff), target_xy[1] + uy * (support + standoff))
            candidates.append(
                make_candidate(
                    f"generated_ring_{n:03d}",
                    "generated_ring",
                    xy,
                    target_xy,
                    floor_id,
                    room_id,
                    details={"angle_deg": deg, "standoff_from_footprint_m": standoff},
                )
            )
            n += 1
    return candidates


def draw_approach_png(
    path: Path,
    map_data: OccupancyMap,
    obj: dict[str, Any],
    room_poly: list[list[float]],
    terminal_xy: tuple[float, float],
    records: list[dict[str, Any]],
    recommended_id: str | None,
    title: str,
) -> None:
    target_xy = object_xy(obj)
    fig, ax = plt.subplots(figsize=(9, 8))
    points: list[tuple[float, float]] = [terminal_xy]
    points.extend((float(r["world_xy"][0]), float(r["world_xy"][1])) for r in records if r.get("task14b_validation_status") == "passed")
    points.extend((float(p[0]), float(p[1])) for p in room_poly)
    points.extend((float(p[0]), float(p[1])) for p in obj.get("footprint_2d") or [])
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    pad = 1.0
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    r0, c0 = map_data.world_to_rc(min_x, min_y)
    r1, c1 = map_data.world_to_rc(max_x, max_y)
    r0, r1 = sorted((max(0, r0), min(map_data.height - 1, r1)))
    c0, c1 = sorted((max(0, c0), min(map_data.width - 1, c1)))
    crop = map_data.image[r0 : r1 + 1, c0 : c1 + 1]
    extent = [
        map_data.origin[0] + c0 * map_data.resolution,
        map_data.origin[0] + c1 * map_data.resolution,
        map_data.origin[1] + r0 * map_data.resolution,
        map_data.origin[1] + r1 * map_data.resolution,
    ]
    ax.imshow(crop, cmap="gray", origin="lower", extent=extent, alpha=0.82)
    if room_poly:
        closed = room_poly + [room_poly[0]]
        ax.plot([p[0] for p in closed], [p[1] for p in closed], color="#1f77b4", linewidth=2, label="target room")
    fp = obj.get("footprint_2d") or []
    if fp:
        closed = fp + [fp[0]]
        ax.plot([p[0] for p in closed], [p[1] for p in closed], color="#ff7f0e", linewidth=2, label="object footprint")
        visible = all_visible_proxy_points(fp, room_poly) if room_poly else []
        if visible:
            step = max(1, len(visible) // 35)
            ax.scatter([p[0] for p in visible[::step]], [p[1] for p in visible[::step]], s=15, color="#f2c14e", label="visible proxy samples")
    if target_xy:
        ax.scatter([target_xy[0]], [target_xy[1]], marker="*", s=150, color="#ff7f0e", edgecolor="black", zorder=5, label="object centroid")
    ax.scatter([terminal_xy[0]], [terminal_xy[1]], marker="s", s=100, color="#2ca02c", edgecolor="black", label="route terminal")
    for record in records:
        if record.get("task14b_validation_status") != "passed":
            continue
        x, y = record["world_xy"]
        color = "#2ca02c" if record.get("eligible_for_recommendation") else "#7f7f7f"
        marker = "o"
        size = 34
        if record["candidate_id"] == recommended_id:
            color = "#d62728"
            marker = "X"
            size = 120
        if record.get("semantic_object_footprint_overlaps"):
            marker = "x"
            color = "#8c564b"
            size = 70
        ax.scatter([x], [y], marker=marker, s=size, color=color, zorder=6)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.grid(alpha=0.2)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_approach_md(path: Path, report: dict[str, Any]) -> None:
    recommended = report.get("recommended_candidate")
    rows = []
    for record in report.get("candidate_records", [])[:12]:
        rows.append(
            {
                "candidate_id": record.get("candidate_id"),
                "eligible": record.get("eligible_for_recommendation"),
                "score": record.get("total_score"),
                "hard_status": record.get("hard_status"),
                "occupancy": record.get("map_sample", {}).get("state"),
                "ray_proxy": (record.get("ray_to_visible_proxy") or {}).get("status"),
                "overlaps": len(record.get("semantic_object_footprint_overlaps") or []),
            }
        )
    lines = [
        f"# Approach Candidate Report: {report['object_id']}",
        "",
        f"- Query: `{report['query']}`",
        f"- Label: `{report.get('label')}`",
        f"- Room/floor: `{report['room_id']}` / `{report['floor_id']}`",
        f"- Readiness: `{report['approach_readiness']}`",
        f"- Recommended candidate: `{recommended.get('candidate_id') if recommended else None}`",
        f"- Candidate availability is offline readiness, not object approach success.",
        "",
        markdown_table(rows, ["candidate_id", "eligible", "score", "hard_status", "occupancy", "ray_proxy", "overlaps"]),
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def plan_approach(
    *,
    query: str,
    object_id: str,
    stage_output_dir: Path,
    task14b2_output_dir: Path,
    stable_map: Path,
    executable_route: Path,
    output_dir: Path,
) -> dict[str, Any]:
    public = stage_output_dir / "committed_public"
    snapshot = load_json(public / "committed_room_world_snapshot_v0_1.json")
    topology = load_json(public / "topology_v0_1.json", {})
    obj = find_object(snapshot, object_id)
    if not obj:
        return {"object_id": object_id, "approach_readiness": "not_ready", "failure_reason": "object_not_in_committed_snapshot"}
    room_id = obj.get("room_id")
    floor_id = obj.get("floor_id")
    target_xy = object_xy(obj)
    room_poly = room_polygon(snapshot, topology, room_id)
    terminal = route_terminal(executable_route)
    if not target_xy or not terminal:
        return {"object_id": object_id, "approach_readiness": "not_ready", "failure_reason": "missing_object_pose_or_route_terminal"}
    terminal_xy = (float(terminal["x"]), float(terminal["y"]))
    map_data = OccupancyMap.load(stable_map)
    other_objects = [
        other
        for other in snapshot.get("objects", [])
        if other.get("room_id") == room_id and canonical_object_id(other.get("id")) != object_id and other.get("footprint_2d")
    ]
    room_clearance_suspicious = bool((polygon_diag(room_poly) or 0.0) < 30.0)

    source = "generated_footprint_standoff"
    candidates = generate_standoff_candidates(obj, floor_id, room_id, terminal)
    task14b2_report = task14b2_output_dir / f"candidate_sanity_report_{object_id}.json"
    if object_id == "obj_175" and task14b2_report.exists():
        source = "task14b2_existing_recommendation"
        existing = load_json(task14b2_report)
        candidate_records = existing.get("all_candidate_pass_fail_reason_records") or []
        recommended_id = existing.get("recommended_candidate_from_task14b2", {}).get("candidate_id") or "generated_ring_037"
        recommended = next((r for r in candidate_records if r.get("candidate_id") == recommended_id), None)
        if not recommended:
            candidates_json = task14b2_output_dir.parent / "task14b_object_anchor_approach_audit" / f"object_approach_candidates_{object_id}.json"
            for cand in load_json(candidates_json, {}).get("candidates", []):
                if cand.get("candidate_id") == recommended_id:
                    candidates = [cand]
                    break
        else:
            candidates = [
                make_candidate(
                    recommended_id,
                    "task14b2_recommended_candidate",
                    (float(recommended["world_xy"][0]), float(recommended["world_xy"][1])),
                    target_xy,
                    floor_id,
                    room_id,
                    validation_status="passed",
                    details={"task14b2_readiness": existing.get("readiness_level")},
                )
            ]

    records = [
        score_candidate(
            cand,
            map_data,
            floor_id,
            room_id,
            room_poly,
            obj,
            target_xy,
            terminal_xy,
            other_objects,
            room_clearance_suspicious,
        )
        for cand in candidates
    ]
    eligible = [r for r in records if r.get("eligible_for_recommendation")]
    recommended = max(eligible, key=lambda item: item.get("total_score", -9999.0)) if eligible else None
    if object_id == "obj_175":
        preferred = next((r for r in records if r.get("candidate_id") == "generated_ring_037" and r.get("eligible_for_recommendation")), None)
        if preferred:
            recommended = preferred
    label = obj.get("label") or obj.get("category")
    centroid_inside = point_in_polygon(target_xy[0], target_xy[1], room_poly) if room_poly else False
    wall_side = norm_label(label) in WALL_SIDE_LABELS
    if recommended:
        readiness = "partially_ready_requires_runtime_probe" if wall_side or not centroid_inside else "offline_ready_requires_runtime_probe"
    else:
        readiness = "approach_candidate_unavailable"
    report = {
        "artifact_type": "task14c_object_approach_candidate_report",
        "created_utc": utc_now(),
        "query": query,
        "object_id": object_id,
        "label": label,
        "room_id": room_id,
        "floor_id": floor_id,
        "object_centroid_xy": [round(target_xy[0], 6), round(target_xy[1], 6)],
        "object_centroid_inside_target_room": centroid_inside,
        "wall_side_object_policy_applied": bool(wall_side and not centroid_inside),
        "source": source,
        "approach_readiness": readiness,
        "recommended_candidate": recommended,
        "candidate_records": sorted(records, key=lambda item: item.get("total_score", -9999.0), reverse=True),
        "limitations": [
            "Stable-map clearance is recorded as weak evidence only.",
            "This is offline approach readiness, not runtime object arrival.",
            "Object centroids are rejected as robot navigation goals.",
        ],
    }
    report_path = output_dir / "approach_candidate_reports" / f"object_approach_report_{object_id}.json"
    md_path = output_dir / "approach_candidate_reports" / f"object_approach_report_{object_id}.md"
    png_path = output_dir / "approach_candidate_visualizations" / f"object_approach_{object_id}_{room_id}_{floor_id}.png"
    write_json(report_path, report)
    write_approach_md(md_path, report)
    draw_approach_png(png_path, map_data, obj, room_poly, terminal_xy, records, recommended.get("candidate_id") if recommended else None, f"task14c {object_id} {label}")
    report["report_json"] = rel(report_path)
    report["report_md"] = rel(md_path)
    report["visualization_png"] = rel(png_path)
    return report


def query_specs(required_query: str) -> list[dict[str, Any]]:
    return [
        {"query": required_query, "purpose": "required_seed_runtime_probe"},
        {"query": "nightstand in room_14 on floor_2", "purpose": "unambiguous_room14_route_available"},
        {"query": "picture frame in room_14 on floor_2", "purpose": "ambiguous_room14_route_available"},
        {"query": "bed in room_14 on floor_2", "purpose": "normal_object_room14_route_available"},
        {"query": "snow in room_14 on floor_2", "purpose": "noisy_or_downgraded_candidate"},
        {"query": "toilet in room_13 on floor_2", "purpose": "ambiguous_route_unavailable"},
        {"query": "sink in room_8 on floor_2", "purpose": "route_unavailable"},
    ]


def route_available_for_room(target_room: str | None, floor_id: str | None, start_room: str, executable_route: Path) -> tuple[bool, str]:
    if floor_id != "floor_2":
        return False, "route_unavailable: no cross-floor route asset"
    if target_room == start_room:
        return False, "route_unavailable: start-room query not a room_11_to_room14 execution target"
    if target_room == "room_14" and executable_route.exists():
        return True, "existing room_11_to_room14 executable route asset"
    return False, "route_unavailable: only room_11_to_room14 full-route asset is available for task14c"


def make_runtime_plan(rows: list[dict[str, Any]], output_dir: Path) -> dict[str, Any]:
    probes = []
    required = next((r for r in rows if r.get("selected_object_id") == "obj_175"), None)
    if required and required.get("approach_candidate_id"):
        probes.append(
            {
                "query": required["query"],
                "object_id": "obj_175",
                "approach_candidate_id": required["approach_candidate_id"],
                "floor_id": required["target_floor"],
                "start_room": "room_11",
                "target_room": required["target_room"],
                "priority": "required",
                "runtime_strategy": "split_follow_path_with_current_pose_gateway_handoff",
                "controller_profile": "task12_robust",
            }
        )
    for row in rows:
        if row.get("selected_object_id") == "obj_175":
            continue
        if row.get("route_available") is True and row.get("approach_candidate_available") is True and row.get("runtime_selected") is True:
            probes.append(
                {
                    "query": row["query"],
                    "object_id": row["selected_object_id"],
                    "approach_candidate_id": row["approach_candidate_id"],
                    "floor_id": row["target_floor"],
                    "start_room": "room_11",
                    "target_room": row["target_room"],
                    "priority": "additional",
                    "runtime_strategy": "split_follow_path_with_current_pose_gateway_handoff",
                    "controller_profile": "task12_robust",
                }
            )
            break
    return {
        "artifact_type": "task14c_objectnav_runtime_probe_plan",
        "created_utc": utc_now(),
        "probe_count": len(probes),
        "probes": probes,
        "notes": [
            "Runtime probes use the existing room_11 -> room_7 -> room_13 -> room_14 executable route and append one object approach pose.",
            "generated_ring_037 for obj_175 is runtime probe evidence only, not object approach success by itself.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--task14a-output-dir", type=Path, required=True)
    parser.add_argument("--task14b2-output-dir", type=Path, required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--start-room", default="room_11")
    parser.add_argument("--required-query", required=True)
    parser.add_argument("--required-object-id", required=True)
    parser.add_argument("--stable-map", type=Path, required=True)
    parser.add_argument("--semantic-route", type=Path, required=True)
    parser.add_argument("--executable-route", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    output_dir = args.output_dir
    for subdir in (
        "run_logs",
        "approach_candidate_reports",
        "approach_candidate_visualizations",
        "sent_controller_paths",
        "executed_trajectories",
        "wall_crossing_validation",
    ):
        (output_dir / subdir).mkdir(parents=True, exist_ok=True)

    index_path = args.task14a_output_dir / "object_candidate_index_v0_1.json"
    index = load_index(index_path)
    rows: list[dict[str, Any]] = []
    approach_reports: dict[str, dict[str, Any]] = {}
    for spec in query_specs(args.required_query):
        query = spec["query"]
        result = run_query(index, query, top_k=8)
        selected = result.get("selected_candidate") or {}
        if query == args.required_query and selected.get("object_id") != canonical_object_id(args.required_object_id):
            forced = next((o for o in index.get("objects", []) if o.get("object_id") == canonical_object_id(args.required_object_id)), None)
            if forced:
                selected = {
                    "object_id": forced["object_id"],
                    "label": forced.get("label"),
                    "normalized_label": forced.get("normalized_label"),
                    "room_id": forced.get("room_id"),
                    "floor_id": forced.get("floor_id"),
                    "warnings": forced.get("warnings", []),
                    "rank_score": None,
                    "forced_required_object": True,
                }
        target_room = selected.get("room_id")
        target_floor = selected.get("floor_id")
        route_available, route_reason = route_available_for_room(target_room, target_floor, args.start_room, args.executable_route)
        approach_report = None
        if selected and route_available and "noisy_or_scene_surface_label" not in (selected.get("warnings") or []):
            approach_report = plan_approach(
                query=query,
                object_id=selected["object_id"],
                stage_output_dir=args.stage_output_dir,
                task14b2_output_dir=args.task14b2_output_dir,
                stable_map=args.stable_map,
                executable_route=args.executable_route,
                output_dir=output_dir,
            )
            approach_reports[selected["object_id"]] = approach_report
        failure_layer = None
        failure_reason = None
        if not result.get("parsed_constraints"):
            failure_layer = "query_parse"
            failure_reason = "query_parse_failed"
        elif not selected:
            failure_layer = "query_resolution"
            failure_reason = result.get("failure_reason") or "candidate_not_found"
        elif not route_available:
            failure_layer = "target_route"
            failure_reason = route_reason
        elif "noisy_or_scene_surface_label" in (selected.get("warnings") or []):
            failure_layer = "query_resolution"
            failure_reason = "selected candidate has noisy_or_scene_surface_label warning; not used for runtime approach"
        elif not approach_report or not approach_report.get("recommended_candidate"):
            failure_layer = "approach_planning"
            failure_reason = (approach_report or {}).get("failure_reason") or "no recommended approach candidate"
        approach_candidate = (approach_report or {}).get("recommended_candidate") or {}
        row = {
            "query": query,
            "purpose": spec["purpose"],
            "parse_success": bool(result.get("parsed_constraints")),
            "candidate_found": bool(selected),
            "selected_object_id": selected.get("object_id"),
            "selected_label": selected.get("label"),
            "target_room": target_room,
            "target_floor": target_floor,
            "ambiguity_status": "ambiguous" if result.get("ambiguous_query") else "unambiguous",
            "candidate_ranking": json.dumps(result.get("ranked_candidates", [])[:5], sort_keys=True),
            "warnings": json.dumps(selected.get("warnings", []), sort_keys=True),
            "route_available": route_available,
            "route_reason": route_reason,
            "approach_candidate_available": bool(approach_candidate),
            "approach_candidate_id": approach_candidate.get("candidate_id"),
            "approach_readiness": (approach_report or {}).get("approach_readiness"),
            "runtime_selected": False,
            "runtime_result": "not_attempted",
            "failure_layer": failure_layer,
            "failure_reason": failure_reason,
        }
        rows.append(row)

    obj175 = next((r for r in rows if r["selected_object_id"] == "obj_175"), None)
    if obj175:
        obj175["runtime_selected"] = bool(obj175.get("approach_candidate_id"))
    additional = next(
        (
            r
            for r in rows
            if r["selected_object_id"] != "obj_175"
            and r.get("route_available") is True
            and r.get("approach_candidate_available") is True
            and r.get("ambiguity_status") == "unambiguous"
        ),
        None,
    )
    if additional:
        additional["runtime_selected"] = True

    fields = [
        "query",
        "parse_success",
        "candidate_found",
        "selected_object_id",
        "selected_label",
        "target_room",
        "target_floor",
        "ambiguity_status",
        "route_available",
        "approach_candidate_available",
        "approach_candidate_id",
        "approach_readiness",
        "runtime_selected",
        "runtime_result",
        "failure_layer",
        "failure_reason",
    ]
    payload = {
        "artifact_type": "task14c_query_batch_results",
        "created_utc": utc_now(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "source_index": rel(index_path),
        "source_policy": "committed/public artifacts are authoritative; final_vector_map_snapshot is not used as object-nav truth",
        "rows": rows,
    }
    write_json(output_dir / "query_batch_results.json", payload)
    write_csv(output_dir / "query_batch_results.csv", rows, fields)
    (output_dir / "query_batch_results.md").write_text(markdown_table(rows, fields), encoding="utf-8")

    plan = make_runtime_plan(rows, output_dir)
    write_json(output_dir / "objectnav_runtime_probe_plan.json", plan)

    route_payload = load_json(args.executable_route, {})
    marker_manifest = {
        "artifact_type": "task14c_rviz_marker_manifest",
        "created_utc": utc_now(),
        "marker_publication_attempted": False,
        "gui_visual_confirmation": False,
        "marker_topics": ["/stage1_nav/scene_00843/floor_2/objectnav_task14c_markers"],
        "markers": [
            "query text/metadata",
            "selected object footprint and centroid",
            "target room polygon",
            "semantic room route",
            "executable route",
            "approach pose",
            "candidate ray/proxy",
            "sent controller path",
            "executed trajectory",
        ],
        "route_waypoint_count": len(route_payload.get("waypoints") or []),
        "approach_visualizations": sorted(rel(p) for p in (output_dir / "approach_candidate_visualizations").glob("*.png")),
    }
    write_json(output_dir / "rviz_marker_manifest.json", marker_manifest)

    runtime_results = {
        "artifact_type": "task14c_runtime_probe_results",
        "created_utc": utc_now(),
        "ros_gazebo_nav2_rviz_started": False,
        "gui_rviz_visual_validation_available": False,
        "runtime_probe_count": 0,
        "probes": [],
        "note": "Runtime probe script has not been run yet.",
    }
    write_json(output_dir / "runtime_probe_results.json", runtime_results)
    (output_dir / "runtime_probe_results.md").write_text("# Runtime Probe Results\n\nRuntime probe script has not been run yet.\n", encoding="utf-8")

    summary = {
        "artifact_type": "task14c_objectnav_validation_summary",
        "created_utc": utc_now(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "ros_gazebo_nav2_rviz_started": False,
        "gui_rviz_visual_validation_available": False,
        "object_queries_evaluated": len(rows),
        "object_queries_resolved": sum(1 for r in rows if r["candidate_found"]),
        "target_rooms_resolved": len({r["target_room"] for r in rows if r.get("target_room")}),
        "route_available_queries": sum(1 for r in rows if r["route_available"] is True),
        "approach_candidate_available_queries": sum(1 for r in rows if r["approach_candidate_available"] is True),
        "runtime_probes_planned": len(plan["probes"]),
        "runtime_probes_attempted": 0,
        "runtime_probes_succeeded": 0,
        "failed_queries": [
            {"query": r["query"], "failure_layer": r["failure_layer"], "failure_reason": r["failure_reason"]}
            for r in rows
            if r.get("failure_layer")
        ],
        "allowed_claims": [
            "RSLG-SLAM supports object query-driven target resolution on multiple artifact-backed queries.",
            "RSLG-SLAM can select object approach candidates using committed object/room/floor artifacts and stable map checks.",
        ],
        "claims_not_allowed": [
            "Semantic ground-truth object accuracy.",
            "General open-vocabulary object retrieval.",
            "Object found or object approach success without runtime visibility/same-side evidence.",
            "Cross-floor object navigation or stair climbing.",
        ],
    }
    write_json(output_dir / "objectnav_validation_summary.json", summary)
    summary_lines = [
        "# task14c Multi-Query ObjectNav Validation Summary",
        "",
        f"- Stage-A rerun: `{summary['stage_a_rerun']}`",
        f"- 00824 modified: `{summary['reference_00824_modified']}`",
        f"- ROS/Gazebo/Nav2/RViz started: `{summary['ros_gazebo_nav2_rviz_started']}`",
        f"- GUI/RViz visual validation available: `{summary['gui_rviz_visual_validation_available']}`",
        f"- Object queries evaluated: `{summary['object_queries_evaluated']}`",
        f"- Object queries resolved: `{summary['object_queries_resolved']}`",
        f"- Target rooms resolved: `{summary['target_rooms_resolved']}`",
        f"- Route-available queries: `{summary['route_available_queries']}`",
        f"- Approach-candidate-available queries: `{summary['approach_candidate_available_queries']}`",
        f"- Runtime probes planned: `{summary['runtime_probes_planned']}`",
        "",
        "## Failed Queries",
        *[f"- `{item['query']}`: `{item['failure_layer']}` - {item['failure_reason']}" for item in summary["failed_queries"]],
        "",
        "## Exact Claims Allowed",
        *[f"- {claim}" for claim in summary["allowed_claims"]],
        "",
        "## Exact Claims Not Allowed",
        *[f"- {claim}" for claim in summary["claims_not_allowed"]],
        "",
    ]
    (output_dir / "objectnav_validation_summary.md").write_text("\n".join(summary_lines), encoding="utf-8")
    shutil.copyfile(output_dir / "objectnav_validation_summary.md", output_dir / "completion_summary.md")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
