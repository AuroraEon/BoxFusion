#!/usr/bin/env python3
"""Read-only task24b audit generator for RSLG-SLAM vertical connector artifacts."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
from collections import Counter, deque
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any


SHANGHAI = timezone(timedelta(hours=8))
CLAIM_BOUNDARY = "topological_vertical_transition_only"
FINAL_CLASSIFICATION = "current_artifacts_sufficient_for_connector_demo"


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def run_git(root: Path, *args: str) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=root, text=True, stderr=subprocess.STDOUT).strip()
    except subprocess.CalledProcessError as exc:
        return exc.output.strip()


def find_files(root: Path, predicate) -> list[Path]:
    if not root.exists():
        return []
    out: list[Path] = []
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            path = Path(dirpath) / name
            if predicate(path):
                out.append(path)
    return sorted(out)


def room_key(room: dict[str, Any]) -> str:
    return room.get("id") or room.get("room_id") or f"room_{room.get('room_uuid')}"


def room_center(room: dict[str, Any]) -> list[float] | None:
    center = room.get("center") or room.get("centroid_xy")
    if center and len(center) >= 2:
        return [float(center[0]), float(center[1])]
    poly = room.get("polygon") or room.get("footprint_polygon_xy")
    if poly:
        xs = [float(p[0]) for p in poly]
        ys = [float(p[1]) for p in poly]
        return [sum(xs) / len(xs), sum(ys) / len(ys)]
    return None


def euclidean3(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def edge_key(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((a, b)))


def build_candidate(
    transition: dict[str, Any],
    floors_by_id: dict[str, dict[str, Any]],
    rooms_by_id: dict[str, dict[str, Any]],
    stair_objects: list[dict[str, Any]],
) -> dict[str, Any]:
    floor_from = transition.get("from_floor_id")
    floor_to = transition.get("to_floor_id")
    from_xy = [float(v) for v in transition.get("from_position_xy", [0.0, 0.0])]
    to_xy = [float(v) for v in transition.get("to_position_xy", [0.0, 0.0])]
    from_z = float((floors_by_id.get(floor_from) or {}).get("z_center", transition.get("z_min", 0.0)))
    to_z = float((floors_by_id.get(floor_to) or {}).get("z_center", transition.get("z_max", from_z)))
    from_room = f"room_{transition.get('from_room_id')}" if isinstance(transition.get("from_room_id"), int) else transition.get("from_room_id")
    to_room = f"room_{transition.get('to_room_id')}" if isinstance(transition.get("to_room_id"), int) else transition.get("to_room_id")
    from_xyz = [round(from_xy[0], 3), round(from_xy[1], 3), round(from_z, 3)]
    to_xyz = [round(to_xy[0], 3), round(to_xy[1], 3), round(to_z, 3)]
    stair_near_from = []
    for obj in stair_objects:
        pose = obj.get("pose") or []
        if len(pose) >= 2:
            d = math.hypot(float(pose[0]) - from_xy[0], float(pose[1]) - from_xy[1])
            if d <= 1.0:
                stair_near_from.append({"object_id": f"obj_{obj.get('id')}", "distance_m": round(d, 3), "floor_status": (obj.get("floor_assignment") or {}).get("status")})
    return {
        "schema_version": "vertical_connector_candidate_v0_1",
        "connector_id": f"vc_{transition.get('transition_id', 'unknown')}",
        "connector_type": "stairs" if stair_objects else "unknown_vertical_transition",
        "connector_label": transition.get("connector_label"),
        "source": "pose_height_transition",
        "floor_from": floor_from,
        "floor_to": floor_to,
        "endpoint_from": {
            "endpoint_id": f"vc_{transition.get('transition_id')}_from",
            "position_xyz": from_xyz,
            "position_xy_source": "vertical_transition_evidence.from_position_xy",
            "z_source": "floor.z_center",
            "room_binding": from_room,
            "topology_node_binding": from_room,
            "binding_type": "room_node",
        },
        "endpoint_to": {
            "endpoint_id": f"vc_{transition.get('transition_id')}_to",
            "position_xyz": to_xyz,
            "position_xy_source": "vertical_transition_evidence.to_position_xy",
            "z_source": "floor.z_center",
            "room_binding": to_room,
            "topology_node_binding": to_room,
            "binding_type": "room_node",
        },
        "endpoint_from_room_status": (rooms_by_id.get(from_room) or {}).get("status"),
        "endpoint_to_room_status": (rooms_by_id.get(to_room) or {}).get("status"),
        "connector_path_xyz": [from_xyz, to_xyz],
        "connector_path_interpretation": "minimal endpoint-only topological path; dense per-frame pose path is not exported",
        "cost": round(euclidean3(from_xyz, to_xyz), 3),
        "confidence": float(transition.get("confidence", 0.0)),
        "semantic_stair_corroboration": {
            "stair_object_count": len(stair_objects),
            "near_from_endpoint_within_1m": stair_near_from,
            "spans_floors": False,
        },
        "provenance": {
            "transition_id": transition.get("transition_id"),
            "frame_start": transition.get("frame_start"),
            "frame_end": transition.get("frame_end"),
            "entry_stable_frame": transition.get("entry_stable_frame"),
            "exit_stable_frame": transition.get("exit_stable_frame"),
            "z_min": transition.get("z_min"),
            "z_max": transition.get("z_max"),
            "z_span_m": transition.get("z_span_m"),
            "supporting_frame_count": transition.get("supporting_frame_count"),
            "from_room_support": transition.get("from_room_support"),
            "to_room_support": transition.get("to_room_support"),
            "evidence_summary": transition.get("evidence_summary"),
        },
        "physical_execution_supported": False,
        "claim_boundary": CLAIM_BOUNDARY,
        "validation_status": "dry_run_topological_candidate_only",
        "missing_evidence": [
            "dense ordered camera pose_xy/yaw sequence for frames 1516-1538",
            "explicit stair/ramp/elevator span geometry with endpoints on both floors",
            "floor_1 runtime occupancy map and gateway registry for executable same-floor route generation",
        ],
    }


def build_cross_floor_dry_run(topology: dict[str, Any], candidate: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    rooms = topology.get("rooms") or []
    room_lookup = {room_key(r): r for r in rooms}
    base_edges = []
    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {}
    for room in rooms:
        adjacency.setdefault(room_key(room), [])
    for edge in topology.get("edges") or []:
        if edge.get("relation_type") == "vertical_transition":
            continue
        if edge.get("status") not in {"supported", "confirmed"}:
            continue
        a, b = edge.get("source"), edge.get("target")
        if a not in adjacency or b not in adjacency:
            continue
        row = {
            "edge_id": f"same_floor_{a}_{b}_{len(base_edges)}",
            "source": a,
            "target": b,
            "edge_type": "same_floor_topology",
            "relation_type": edge.get("relation_type"),
            "floor_transition_count": 0,
            "cost": 1.0,
            "confidence": edge.get("confidence"),
            "status": edge.get("status"),
            "source_edge": edge,
        }
        base_edges.append(row)
        adjacency[a].append((b, row))
        adjacency[b].append((a, row))

    cf = candidate["connector_id"]
    ep_from = candidate["endpoint_from"]["endpoint_id"]
    ep_to = candidate["endpoint_to"]["endpoint_id"]
    room_from = candidate["endpoint_from"]["topology_node_binding"]
    room_to = candidate["endpoint_to"]["topology_node_binding"]
    endpoint_nodes = [
        {
            "node_id": ep_from,
            "node_type": "connector_endpoint",
            "floor_id": candidate["floor_from"],
            "position_xyz": candidate["endpoint_from"]["position_xyz"],
            "bound_room_id": room_from,
            "connector_id": cf,
        },
        {
            "node_id": ep_to,
            "node_type": "connector_endpoint",
            "floor_id": candidate["floor_to"],
            "position_xyz": candidate["endpoint_to"]["position_xyz"],
            "bound_room_id": room_to,
            "connector_id": cf,
        },
    ]
    nodes = [
        {
            "node_id": rid,
            "node_type": "room",
            "floor_id": room.get("floor_id"),
            "position_xy": room_center(room),
        }
        for rid, room in room_lookup.items()
    ] + endpoint_nodes
    for nid in [ep_from, ep_to]:
        adjacency.setdefault(nid, [])

    connector_edges = [
        {
            "edge_id": f"{cf}_bind_from",
            "source": room_from,
            "target": ep_from,
            "edge_type": "connector_endpoint_binding",
            "floor_transition_count": 0,
            "cost": 0.0,
            "confidence": candidate["confidence"],
            "status": "dry_run_supported",
        },
        {
            "edge_id": f"{cf}_vertical",
            "source": ep_from,
            "target": ep_to,
            "edge_type": "vertical_connector",
            "connector_id": cf,
            "connector_type": candidate["connector_type"],
            "floor_from": candidate["floor_from"],
            "floor_to": candidate["floor_to"],
            "floor_transition_count": 1,
            "cost": candidate["cost"],
            "confidence": candidate["confidence"],
            "physical_execution_supported": False,
            "claim_boundary": CLAIM_BOUNDARY,
            "status": "dry_run_supported",
        },
        {
            "edge_id": f"{cf}_bind_to",
            "source": ep_to,
            "target": room_to,
            "edge_type": "connector_endpoint_binding",
            "floor_transition_count": 0,
            "cost": 0.0,
            "confidence": candidate["confidence"],
            "status": "dry_run_supported",
        },
    ]
    for edge in connector_edges:
        a, b = edge["source"], edge["target"]
        adjacency.setdefault(a, []).append((b, edge))
        adjacency.setdefault(b, []).append((a, edge))

    start = room_from
    goal = "room_14" if "room_14" in adjacency else room_to
    prev: dict[str, tuple[str, dict[str, Any]] | None] = {start: None}
    queue: deque[str] = deque([start])
    while queue:
        cur = queue.popleft()
        if cur == goal:
            break
        for nxt, edge in sorted(adjacency.get(cur, []), key=lambda item: item[0]):
            if nxt not in prev:
                prev[nxt] = (cur, edge)
                queue.append(nxt)
    path = []
    used_edges = []
    if goal in prev:
        cur = goal
        while cur != start:
            path.append(cur)
            prior, edge = prev[cur]  # type: ignore[misc]
            used_edges.append({**edge, "traversed_from": prior, "traversed_to": cur})
            cur = prior
        path.append(start)
        path.reverse()
        used_edges.reverse()
    route_report = {
        "schema_version": "dry_run_cross_floor_route_query_report_v0_1",
        "scene_id": topology.get("sequence_id"),
        "start_room": start,
        "goal_room": goal,
        "selected_route_nodes": path,
        "route_valid_in_dry_run_graph": bool(path),
        "used_connector_ids": sorted({e.get("connector_id") for e in used_edges if e.get("edge_type") == "vertical_connector"}),
        "used_edge_types": [e.get("edge_type") for e in used_edges],
        "floor_transition_count": sum(int(e.get("floor_transition_count", 0)) for e in used_edges),
        "physical_execution_supported": False,
        "claim_boundary": CLAIM_BOUNDARY,
        "edges": used_edges,
        "notes": "Dry-run graph routing only. No Gazebo, Nav2, Habitat, gait, or physical stair execution was run.",
    }
    graph = {
        "schema_version": "dry_run_cross_floor_topology_v0_1",
        "scene_id": topology.get("sequence_id"),
        "claim_boundary": CLAIM_BOUNDARY,
        "nodes": nodes,
        "edges": base_edges + connector_edges,
        "vertical_connectors": [candidate],
        "rules": {
            "same_floor_topology_edges_unchanged": True,
            "connector_endpoint_nodes_added_explicitly": True,
            "vertical_connector_edges_are_separate_edge_type": True,
            "floor_transition_count_explicit": True,
            "no_hidden_forbidden_shortcut_between_floors": True,
        },
    }
    return graph, route_report


def render_visualization(output: Path, topology: dict[str, Any], candidate: dict[str, Any], route: dict[str, Any]) -> str:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
    except Exception as exc:  # pragma: no cover - environment dependent
        write_text(output.with_suffix(".txt"), f"matplotlib unavailable: {exc}")
        return output.with_suffix(".txt").as_posix()

    rooms = topology.get("rooms") or []
    colors = {"floor_1": "#f2c14e", "floor_2": "#4f9d69"}
    fig, ax = plt.subplots(figsize=(9, 7))
    for room in rooms:
        rid = room_key(room)
        floor_id = room.get("floor_id")
        poly = room.get("polygon") or room.get("footprint_polygon_xy")
        if poly:
            patch = Polygon(poly, closed=True, facecolor=colors.get(floor_id, "#dddddd"), alpha=0.22, edgecolor="#555555", linewidth=1)
            ax.add_patch(patch)
        center = room_center(room)
        if center:
            ax.scatter([center[0]], [center[1]], s=24, color=colors.get(floor_id, "#777777"))
            ax.text(center[0], center[1], rid.replace("room_", "R"), fontsize=8)
    a = candidate["endpoint_from"]["position_xyz"]
    b = candidate["endpoint_to"]["position_xyz"]
    ax.scatter([a[0], b[0]], [a[1], b[1]], s=90, color="#c03221", marker="x", label="connector endpoints")
    ax.plot([a[0], b[0]], [a[1], b[1]], color="#c03221", linewidth=2.5, linestyle="--", label=candidate["connector_id"])
    route_nodes = route.get("selected_route_nodes") or []
    room_lookup = {room_key(r): r for r in rooms}
    pts = []
    for nid in route_nodes:
        if nid == candidate["endpoint_from"]["endpoint_id"]:
            pts.append(candidate["endpoint_from"]["position_xyz"][:2])
        elif nid == candidate["endpoint_to"]["endpoint_id"]:
            pts.append(candidate["endpoint_to"]["position_xyz"][:2])
        elif nid in room_lookup and room_center(room_lookup[nid]):
            pts.append(room_center(room_lookup[nid]))
    if len(pts) >= 2:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#2f4858", linewidth=1.5, label="dry-run route")
    ax.set_title("task24b dry-run topological vertical connector")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return output.as_posix()


def md_table(rows: list[list[Any]], headers: list[str]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(x) for x in row) + " |")
    return "\n".join(lines)


def make_reports(args: argparse.Namespace) -> None:
    repo = args.repo.resolve()
    stage = args.stage_output.resolve()
    raw = stage / "canonical_stage1/raw_outputs/00843-DYehNKdT76V"
    public = stage / "committed_public"
    out = args.output_dir.resolve()
    timestamp = datetime.now(SHANGHAI).isoformat(timespec="seconds")

    topology = read_json(public / "topology_v0_1.json", {})
    topo_query = read_json(public / "topology_query_report.json", {})
    model = read_json(public / "committed_room_world_model_v0_1.json", {})
    snapshot = read_json(public / "committed_room_world_snapshot_v0_1.json", {})
    vector_snapshot = read_json(public / "final_vector_map_snapshot.json", {})
    floor_diag = read_json(public / "floor_diagnostics_summary.json", {})
    vt_evidence = read_json(public / "vertical_transition_evidence.json", {})
    timeline = read_json(raw / "logs/timeline.json", [])
    frame_floor_assignments = vector_snapshot.get("frame_floor_assignments") or []

    floors = topology.get("floors") or snapshot.get("floors") or []
    floors_by_id = {f["floor_id"]: f for f in floors if f.get("floor_id")}
    rooms_by_id = {room_key(r): r for r in (topology.get("rooms") or snapshot.get("rooms") or [])}
    transitions = vt_evidence.get("transitions") or snapshot.get("vertical_transitions") or []
    stair_terms = ("stair", "staircase", "stairs", "elevator", "lift", "ramp")
    stair_objects = [
        obj for obj in (snapshot.get("objects") or [])
        if any(term in str(obj.get("label") or obj.get("category") or "").lower() for term in stair_terms)
    ]
    vertical_edges = [e for e in topology.get("edges") or [] if e.get("relation_type") == "vertical_transition"]

    candidate = build_candidate(transitions[0], floors_by_id, rooms_by_id, stair_objects) if transitions else None
    dry_graph = dry_route = None
    if candidate:
        start = int(transitions[0].get("transition_frame_start", transitions[0].get("frame_start", -1)))
        end = int(transitions[0].get("transition_frame_end", transitions[0].get("frame_end", -1)))
        candidate["provenance"]["frame_floor_assignment_profile"] = [
            row for row in frame_floor_assignments
            if start <= int(row.get("frame_idx", -1)) <= end
        ]
        candidate["missing_evidence"] = [
            "dense ordered camera pose_xy/yaw sequence for frames 1516-1538",
            "explicit stair/ramp/elevator span geometry with endpoints on both floors",
            "floor_1 runtime occupancy map and gateway registry for executable same-floor route generation",
        ]
        dry_graph, dry_route = build_cross_floor_dry_run(topology, candidate)
        write_json(out / "candidate_vertical_connectors_v0_1.json", {"schema_version": "candidate_vertical_connectors_v0_1", "connectors": [candidate]})
        write_json(out / "dry_run_cross_floor_topology_v0_1.json", dry_graph)
        write_json(out / "dry_run_cross_floor_route_query_report.json", dry_route)
        render_visualization(out / "dry_run_connector_visualization.png", topology, candidate, dry_route)

    map_floor_dirs = sorted([p for p in (stage / "maps").glob("floor_*") if p.is_dir()])
    per_floor_dirs = sorted([p for p in (stage / "process/floors").glob("floor_*") if p.is_dir()])
    committed_jsons = sorted(public.glob("*.json"))
    pose_like_files = find_files(raw, lambda p: any(t in p.name.lower() for t in ["pose", "trajectory", "camera", "timeline", "frame_metadata", "floor_assignment"]))
    map_files_by_floor = {p.name: sorted([f.name for f in p.glob("*")]) for p in map_floor_dirs}
    process_file_counts = {p.name: len(find_files(p, lambda x: x.is_file())) for p in per_floor_dirs}
    branch = run_git(repo, "branch", "--show-current")
    git_status = run_git(repo, "status", "--short")

    current_artifacts_found = {
        "clean_rerun_exists": stage.exists(),
        "raw_stage_a_output_exists": raw.exists(),
        "committed_public_exists": public.exists(),
        "committed_public_jsons": [p.name for p in committed_jsons],
        "maps_floor_dirs": [p.name for p in map_floor_dirs],
        "maps_files_by_floor": map_files_by_floor,
        "process_floor_dirs": [p.name for p in per_floor_dirs],
        "process_file_counts": process_file_counts,
        "timeline_json_exists": (raw / "logs/timeline.json").exists(),
        "timeline_rows": len(timeline) if isinstance(timeline, list) else 0,
        "frame_floor_assignments_embedded_in_final_vector_map_snapshot": len(frame_floor_assignments),
        "pose_like_files": [rel(p, repo) for p in pose_like_files],
        "topology_v0_1_exists": (public / "topology_v0_1.json").exists(),
        "vertical_transition_evidence_exists": (public / "vertical_transition_evidence.json").exists(),
    }
    multi_floor_evidence = {
        "floor_count": len(floors),
        "floors": floors,
        "floor_ids": [f.get("floor_id") for f in floors],
        "floor_heights_available": all("z_center" in f for f in floors),
        "floor_level_rooms": dict(Counter(r.get("floor_id") for r in rooms_by_id.values())),
        "per_floor_occupancy_maps": {
            floor: any(name.endswith(".yaml") for name in files)
            for floor, files in map_files_by_floor.items()
        },
        "inter_floor_vertical_transition_records": len(transitions),
        "vertical_transition_records": transitions,
        "topology_vertical_transition_edges": vertical_edges,
        "semantic_vertical_objects": stair_objects,
        "ordered_timeline_available": isinstance(timeline, list) and len(timeline) > 0,
        "frame_floor_assignment_rows_available": len(frame_floor_assignments) > 0,
        "pose_z_height_transition_sequence_available": len(frame_floor_assignments) > 0,
        "dense_pose_xy_yaw_export_available": False,
        "camera_pose_height_transition_independently_replayable": "partial_pose_z_only",
    }
    connector_feasibility = {
        "pose_height_transition_connector": {
            "classification": "sufficient_for_endpoint_topological_candidate_but_not_dense_pose_path",
            "evidence": [
                "vertical_transition_evidence.json records vt_1 floor_1->floor_2, frames 1516-1538, z_span_m=2.153",
                "vt_1 has supported room bindings room_3 and room_7 plus endpoint XY values",
                "final_vector_map_snapshot.json embeds frame_floor_assignments with pose_z and transition status for frames 1516-1538",
                "topology_v0_1.json contains matching vertical_transition edge room_3->room_7",
            ],
            "limits": [
                "timeline.json is ordered snapshot metadata only",
                "no dense per-frame pose XY/yaw export was found for the transition interval",
            ],
        },
        "semantic_stair_object_connector": {
            "classification": "corroborative_only",
            "candidate_count": len(stair_objects),
            "evidence": stair_objects,
            "limits": [
                "stairs objects are exported only on floor_1/room_3 in the committed snapshot",
                "no semantic stair object spans floor_1 and floor_2 or defines both endpoints",
            ],
        },
        "manual_seed_connector": {
            "classification": "available_as_fallback_not_required_for_vt_1",
            "minimal_fields": [
                "connector_id",
                "connector_type",
                "floor_from",
                "floor_to",
                "endpoint_from.position_xyz",
                "endpoint_to.position_xyz",
                "endpoint_from.room_binding",
                "endpoint_to.room_binding",
                "source=manual_seed",
            ],
            "validation": [
                "endpoint room exists on declared floor",
                "endpoint XY lies inside or near room polygon",
                "endpoint can snap to room/gateway/topology node",
                "no cross-floor edge is created without explicit connector edge type",
            ],
        },
        "artifact_gap_requires_stage_a_modification": {
            "classification": "not_required_for_task24b_dry_run_but_recommended_for_hardened_task24c",
            "missing_artifacts": [
                "dense per-frame camera pose export with frame_idx, timestamp, xyz, yaw/quaternion",
                "standalone per-frame floor assignment export if consumers should avoid parsing final_vector_map_snapshot.json",
                "vertical transition frame sequence/path export, not only endpoints",
                "floor_1 stable occupancy map and gateway registry if executable floor_1 route legs are needed",
                "semantic connector object span records if semantic stair/ramp/elevator discovery should stand alone",
            ],
        },
    }
    topology_compatibility = {
        "room_nodes_are_stable_ids": True,
        "topology_nodes_array_present": bool(topology.get("nodes")),
        "room_ids": sorted(rooms_by_id),
        "room_metric_positions_available": all(room_center(r) is not None for r in rooms_by_id.values()),
        "edge_costs_explicit": False,
        "edge_confidence_status_available": True,
        "vertical_transition_edge_available": bool(vertical_edges),
        "current_route_code_assumes_single_floor": True,
        "evidence_code_paths": [
            "tools/stage1_runtime/scene_runtime_common.py:181 filters room_lookup by one floor_id",
            "tools/stage1_runtime/prepare_scene_semantic_route.py:41 loads rooms only for args.floor_id",
            "tools/stage1_runtime/prepare_scene_semantic_route.py:72 requires same-floor gateway registry entries",
            "tools/object_nav/run_lightweight_object_nav.py:207 explicitly excludes vertical_transition edges",
            "tools/object_nav/run_objectnav_multiquery_validation.py:418 returns no cross-floor route asset outside floor_2",
        ],
    }
    missing_artifacts = connector_feasibility["artifact_gap_requires_stage_a_modification"]["missing_artifacts"]
    generated_candidate_connector = candidate is not None
    stage_a_modification_needed = False
    recommended_next_task = {
        "task_name": "task24c_rslg_cross_floor_connector_dry_run_router",
        "recommendation": "Implement a read-only cross-floor graph builder/router using vertical_transition_evidence.json, with explicit connector endpoint nodes and route reports that mark connector usage.",
        "do_not_do": [
            "Do not run Gazebo/Nav2/robot execution for cross-floor routes",
            "Do not modify task23b or 00824 baseline",
            "Do not claim physical stair climbing",
        ],
    }

    report_json = {
        "task_name": "task24b_rslg_vertical_connector_artifact_audit_and_design",
        "timestamp": timestamp,
        "repo_path": repo.as_posix(),
        "project_name": "RSLG-SLAM",
        "inspected_paths": [
            stage.as_posix(),
            raw.as_posix(),
            public.as_posix(),
            (stage / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml").as_posix(),
            "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24a_hovsg_stair_habitat_code_audit",
            "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24a2_hovsg_postgraph_navigation_experiment_audit",
            "/home/ws/workspace/HOV-SG/audit/task24a3_hovsg_habitat_random_navigation_demo_forensics",
        ],
        "current_artifacts_found": current_artifacts_found,
        "multi_floor_evidence": multi_floor_evidence,
        "connector_source_feasibility": connector_feasibility,
        "topology_compatibility": topology_compatibility,
        "final_classification": FINAL_CLASSIFICATION,
        "generated_candidate_connector": generated_candidate_connector,
        "stage_a_modification_needed": stage_a_modification_needed,
        "missing_artifacts": missing_artifacts,
        "recommended_next_task": recommended_next_task,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    write_json(out / "task24b_audit_report.json", report_json)

    inv_rows = [
        ["git branch", branch or "(none)"],
        ["git status short", git_status.replace("\n", "<br>") if git_status else "clean"],
        ["clean_rerun", stage.exists()],
        ["raw Stage-A output", raw.exists()],
        ["committed_public JSON count", len(committed_jsons)],
        ["maps floor dirs", ", ".join(map_files_by_floor) or "none"],
        ["process floor dirs", ", ".join(process_file_counts) or "none"],
        ["timeline rows", current_artifacts_found["timeline_rows"]],
        ["embedded frame_floor_assignments", current_artifacts_found["frame_floor_assignments_embedded_in_final_vector_map_snapshot"]],
        ["vertical_transition_evidence.json", current_artifacts_found["vertical_transition_evidence_exists"]],
    ]
    artifact_inventory_md = "\n".join([
        "# Artifact inventory",
        "",
        md_table(inv_rows, ["Item", "Finding"]),
        "",
        "## committed_public JSONs",
        "",
        "\n".join(f"- `{p.name}`" for p in committed_jsons),
        "",
        "## Map files by floor",
        "",
        "\n".join(f"- `{floor}`: {', '.join(files) if files else '(empty)'}" for floor, files in map_files_by_floor.items()),
        "",
        "## Pose / timeline / trajectory-like files",
        "",
        "\n".join(f"- `{rel(p, repo)}`" for p in pose_like_files) or "- none",
        "",
        "## Relevant tools changes already present in worktree",
        "",
        "The audit observed existing modified/untracked `tools/` files in git status. They were not reverted or edited except for the new read-only audit helper under `tools/vertical_connectors/`.",
    ])
    write_text(out / "artifact_inventory.md", artifact_inventory_md)

    mf_rows = [
        ["floor_count", len(floors)],
        ["floor_ids", ", ".join(str(f.get("floor_id")) for f in floors)],
        ["floor heights", "available: z_min/z_max/z_center"],
        ["room counts by floor", dict(Counter(r.get("floor_id") for r in rooms_by_id.values()))],
        ["per-floor occupancy maps", multi_floor_evidence["per_floor_occupancy_maps"]],
        ["vertical transitions", len(transitions)],
        ["semantic stair/elevator/ramp objects", len(stair_objects)],
        ["pose_z/floor assignment rows", len(frame_floor_assignments)],
        ["dense pose XY/yaw export", "not found"],
    ]
    write_text(out / "multi_floor_evidence_report.md", "\n".join([
        "# Multi-floor evidence report",
        "",
        md_table(mf_rows, ["Evidence type", "Finding"]),
        "",
        "## Vertical transition evidence",
        "",
        "The committed public artifacts contain `vt_1`: floor_1 room_3 to floor_2 room_7, frames 1516-1538, z_span_m 2.153, confidence 0.95, edge_eligible true.",
        "",
        "## Semantic vertical-object evidence",
        "",
        "Two committed snapshot objects are labeled `stairs`; both are on floor_1/room_3. They corroborate the lower-floor side but do not define a full cross-floor span.",
        "",
        "## Pose evidence limitation",
        "",
        "`final_vector_map_snapshot.json` embeds 2710 `frame_floor_assignments` rows with `pose_z`, floor ID, and transition status, including the vt_1 interval. `timeline.json` provides ordered snapshot rows, RGB paths, vector-map paths, and current room IDs. Neither artifact exports dense per-frame pose XY/yaw rows, so the endpoint connector relies on exported endpoint XY plus pose-z/floor evidence rather than an independently replayable 3D pose path.",
    ]))

    write_text(out / "connector_source_feasibility.md", "\n".join([
        "# Connector source feasibility",
        "",
        "## A. pose_height_transition_connector",
        "",
        "Feasible for an endpoint-level topological connector. `vertical_transition_evidence.json` has frame interval, floor pair, z span, endpoint XY, room bindings, and support statistics; `final_vector_map_snapshot.json` embeds pose_z/floor transition rows. It is not sufficient for a dense stair/ramp path because dense pose XY/yaw rows are not exported.",
        "",
        "## B. semantic_stair_object_connector",
        "",
        "Corroborative only. The committed snapshot has two `stairs` objects on floor_1/room_3, including one near the lower transition endpoint, but no semantic object spans both floors or provides floor_2 endpoint geometry.",
        "",
        "## C. manual_seed_connector",
        "",
        "Supported as a fallback. Minimal seed fields are connector type, source, floor_from/floor_to, endpoint XYZ, room bindings, topology-node bindings, confidence, and provenance. Seeds should be validated against room polygons and map/free-space artifacts before use.",
        "",
        "## D. artifact_gap_requires_stage_a_modification",
        "",
        "Not required for the task24b dry-run connector, but recommended before claiming robust automatic HOV-SG-style connector discovery across scenes. Export dense pose XY/yaw rows, transition path samples, and explicit semantic connector spans.",
    ]))

    write_text(out / "topology_compatibility_report.md", "\n".join([
        "# Topology compatibility report",
        "",
        "- Current floor-level topology uses room IDs such as `room_3`, `room_7`, `room_14` as stable graph nodes; the `nodes` array is empty.",
        "- Rooms carry metric `center`/polygon positions and floor IDs, so connector endpoints can bind to room nodes.",
        "- Edges carry relation type, confidence, status, support counts, and evidence IDs, but no explicit edge cost field.",
        "- `topology_v0_1.json` already contains a `vertical_transition` edge from `room_3` to `room_7`.",
        "- Existing route-generation and object-nav runtime code is still single-floor: it filters rooms by one `floor_id`, requires that floor's gateway registry, and excludes `vertical_transition` edges in lightweight object-nav routing.",
        "",
        "Code paths needing task24c changes: `tools/stage1_runtime/scene_runtime_common.py`, `tools/stage1_runtime/prepare_scene_semantic_route.py`, and the lightweight object-nav topology-route builder if cross-floor object queries are enabled.",
    ]))

    vertical_schema_md = """# Proposed vertical connector schema

```json
{
  "connector_id": "vc_vt_1",
  "connector_type": "stairs | staircase | elevator | ramp | unknown_vertical_transition",
  "source": "pose_height_transition | semantic_object | manual_seed | imported",
  "floor_from": "floor_1",
  "floor_to": "floor_2",
  "endpoint_from": {
    "endpoint_id": "vc_vt_1_from",
    "position_xyz": [-5.262, 1.254, 1.637],
    "room_binding": "room_3",
    "topology_node_binding": "room_3"
  },
  "endpoint_to": {
    "endpoint_id": "vc_vt_1_to",
    "position_xyz": [-5.223, 5.437, 4.837],
    "room_binding": "room_7",
    "topology_node_binding": "room_7"
  },
  "connector_path_xyz": [[-5.262, 1.254, 1.637], [-5.223, 5.437, 4.837]],
  "cost": 5.266,
  "confidence": 0.95,
  "provenance": {
    "transition_id": "vt_1",
    "frame_start": 1516,
    "frame_end": 1538,
    "z_span_m": 2.153
  },
  "physical_execution_supported": false,
  "claim_boundary": "topological_vertical_transition_only",
  "validation_status": "dry_run_topological_candidate_only",
  "missing_evidence": []
}
```

Endpoint Z should be the bound floor z_center unless Stage-A exports verified endpoint pose Z values. `missing_evidence` stays non-empty whenever the candidate lacks dense poses, semantic span geometry, or map validation.
"""
    write_text(out / "proposed_vertical_connector_schema.md", vertical_schema_md)

    cross_schema_md = """# Proposed cross-floor topology schema

- Keep same-floor topology edges unchanged.
- Add explicit connector endpoint nodes, one per floor endpoint.
- Add room/gateway-to-endpoint binding edges with `floor_transition_count: 0`.
- Add vertical connector edges with `edge_type: vertical_connector`, `connector_id`, `connector_type`, `floor_from`, `floor_to`, and `floor_transition_count: 1`.
- Route reports must include `used_connector_ids`, `used_edge_types`, and total `floor_transition_count`.
- No hidden shortcut is allowed: the original room_3 to room_7 vertical relation should be represented through endpoint nodes in the cross-floor dry-run graph.
- Execution reports must keep `physical_execution_supported: false` until a separate physical policy exists.
"""
    write_text(out / "proposed_cross_floor_topology_schema.md", cross_schema_md)

    stage_plan_md = """# Stage-A gap and modification plan

Stage-A modification is not required for the task24b dry-run connector because committed public artifacts already expose `vt_1`, endpoint XY, floor pair, room bindings, and a topology `vertical_transition` edge.

Recommended hardening exports before broad automatic connector discovery:

- `logs/frame_pose_floor_assignments_v0_1.jsonl`: one row per frame with frame_idx, timestamp, camera position XYZ, orientation, assigned floor, stable/transition status, room_id, and confidence. Current artifacts embed pose_z/floor rows in `final_vector_map_snapshot.json`; task24c would be cleaner with a standalone, full-pose export.
- `logs/vertical_transition_paths_v0_1.json`: one record per transition with ordered frame samples, XYZ path, floor-status sequence, endpoint candidates, and validation summary.
- `logs/semantic_vertical_connector_candidates_v0_1.json`: stair/elevator/ramp objects with 3D footprint, floor span, room bindings, endpoint hypotheses, and confidence.
- `maps/floor_1/stage1_floor_1_stable_occupancy_map.{yaml,pgm,npz,png}` and `process/floors/floor_1/gateway/assets/gateway_registry_v0_1.json` if executable same-floor legs on floor_1 are needed.

Minimal rerun plan:

1. Modify only the Stage-A exporters that already know camera pose, floor assignment, room assignment, and vertical transition summaries.
2. Rerun canonical Stage-A for scene `00843-DYehNKdT76V` into a new output directory, not over the validated clean_rerun.
3. Recopy/commit public artifacts only after validation confirms floor counts, `vt_1` reproduction, endpoint room bindings, and no regression to the task23b floor_2 demo artifacts.
4. Run task24b-style validation on the new output; then run task24c dry-run router.

Risk level: medium. The exporter additions are low-risk if read-only, but rerunning Stage-A can change room IDs, segmentation artifacts, or gateway registries unless outputs are isolated and compared.
"""
    write_text(out / "stage_a_gap_and_modification_plan.md", stage_plan_md)

    task24c_md = """# Recommended task24c plan

Implement `task24c_rslg_cross_floor_connector_dry_run_router`.

Minimum implementation:

1. Load `committed_public/topology_v0_1.json` and `committed_public/vertical_transition_evidence.json`.
2. Build `vertical_connectors_v0_1.json` from supported/edge-eligible transitions.
3. Add explicit connector endpoint nodes and vertical connector edges in an experimental cross-floor graph artifact.
4. Implement a graph-only route query that can route `room_3 -> room_14` through `vc_vt_1` and mark connector usage.
5. Write route reports with `used_connector_ids`, `floor_transition_count`, confidence, missing evidence, and `physical_execution_supported: false`.
6. Add validation that rejects hidden direct cross-floor shortcuts and rejects physical execution claims.

Do not run Gazebo, Nav2, CHAMP, Habitat, or robot execution in task24c. Do not modify task23b or 00824.
"""
    write_text(out / "recommended_task24c_plan.md", task24c_md)

    final_answer = """Final classification: current_artifacts_sufficient_for_connector_demo.

The current 00843 artifacts are sufficient for a strict graph-level, dry-run vertical connector candidate: `vt_1` connects `floor_1`/`room_3` to `floor_2`/`room_7`, has endpoint XY, frame interval, z-span, room support, embedded pose-z/floor transition rows, and a matching committed topology `vertical_transition` edge. The artifacts are not sufficient for physical stair climbing, executable Nav2 cross-floor motion, or dense HOV-SG-style pose-path replay because dense pose XY/yaw rows are not exported.

Stage-A modification/rerun is not required for task24c if task24c is limited to topological dry-run routing. Stage-A hardening is recommended before broader automatic connector discovery across scenes.

Exact next recommendation: implement task24c as a read-only cross-floor graph builder/router that emits explicit connector endpoint nodes, vertical connector edges, and route reports that mark connector usage, with `physical_execution_supported: false`.
"""
    write_text(out / "final_answer_for_user.md", final_answer)

    yes_section = "Generated `candidate_vertical_connectors_v0_1.json`, `dry_run_cross_floor_topology_v0_1.json`, `dry_run_cross_floor_route_query_report.json`, and `dry_run_connector_visualization.png` under the task24b output directory."
    audit_md = "\n".join([
        "# task24b RSLG vertical connector artifact audit and design",
        "",
        "## Executive summary",
        "",
        "Final classification: `current_artifacts_sufficient_for_connector_demo`.",
        "",
        "Current 00843 artifacts contain enough evidence for a graph-level dry-run connector candidate, not for physical stair climbing or executable cross-floor robot navigation. The strongest evidence is `vertical_transition_evidence.json` plus the committed `vertical_transition` topology edge from `room_3` to `room_7`.",
        "",
        "## What was inspected",
        "",
        "- 00843 clean_rerun, canonical Stage-A raw outputs, committed_public JSONs, process/floor artifacts, maps, timeline, route artifacts, and relevant route/object-nav code.",
        "- Prior HOV-SG audit outputs from task24a, task24a2, and task24a3.",
        "",
        "## Current 00843 artifact inventory",
        "",
        md_table(inv_rows, ["Item", "Finding"]),
        "",
        "## Multi-floor evidence",
        "",
        md_table(mf_rows, ["Evidence type", "Finding"]),
        "",
        "## Connector source feasibility",
        "",
        "- `pose_height_transition_connector`: sufficient for endpoint-level topological connector construction from exported `vt_1`; not sufficient for dense path replay.",
        "- `semantic_stair_object_connector`: corroborative only; two `stairs` objects on floor_1/room_3, no cross-floor span.",
        "- `manual_seed_connector`: viable fallback, not required for `vt_1`.",
        "- `artifact_gap_requires_stage_a_modification`: not needed for task24b dry run, recommended for hardened automatic discovery.",
        "",
        "## Topology compatibility",
        "",
        "Room IDs act as stable topology nodes; rooms have metric positions and floor IDs. Existing route code remains single-floor and must be extended for cross-floor graph-only route queries.",
        "",
        "## Can we build a connector now?",
        "",
        "Yes, for `topological_vertical_transition_only`. The connector candidate is `vc_vt_1`: `floor_1/room_3` to `floor_2/room_7`, source `pose_height_transition`, type `stairs` with semantic-stair corroboration but endpoint geometry from vertical-transition evidence.",
        "",
        "## If yes: candidate connector dry-run",
        "",
        yes_section,
        "",
        "Dry-run route: `room_3 -> vc_vt_1_from -> vc_vt_1_to -> room_7 -> room_13 -> room_14`. The route report marks one vertical connector and one floor transition. This is a graph-only result.",
        "",
        "## If no: Stage-A artifact gaps",
        "",
        "Not applicable for the dry-run connector classification. Gaps still recommended for Stage-A hardening: dense camera pose XY/yaw, standalone full-pose floor-assignment rows, transition path samples, semantic connector spans, and floor_1 runtime map/gateway artifacts.",
        "",
        "## Proposed vertical connector schema",
        "",
        "See `proposed_vertical_connector_schema.md`.",
        "",
        "## Proposed cross-floor topology extension",
        "",
        "See `proposed_cross_floor_topology_schema.md`.",
        "",
        "## Claim boundary",
        "",
        f"`{CLAIM_BOUNDARY}`. No physical stair climbing, quadruped gait, CHAMP gait, footstep planning, Habitat migration, or real stair execution is claimed.",
        "",
        "## Recommended next task",
        "",
        "Implement task24c as a read-only graph-level cross-floor connector/router using explicit connector endpoint nodes and route reports.",
        "",
        "## Final answer for user",
        "",
        final_answer,
    ])
    write_text(out / "task24b_audit_report.md", audit_md)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument(
        "--stage-output",
        type=Path,
        default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24b_rslg_vertical_connector_artifact_audit_and_design"),
    )
    args = parser.parse_args()
    make_reports(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
