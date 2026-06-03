#!/usr/bin/env python3
"""Plan and execute dynamic artifact-backed object navigation for RSLG-SLAM."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OBJECT_NAV_DIR = Path(__file__).resolve().parent
if str(OBJECT_NAV_DIR) not in sys.path:
    sys.path.insert(0, str(OBJECT_NAV_DIR))
STAGE_RUNTIME_DIR = ROOT / "tools/stage1_runtime"
if str(STAGE_RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(STAGE_RUNTIME_DIR))

from object_nav_common import canonical_object_id, load_index, run_query  # noqa: E402
from scene_runtime_common import load_nav_map  # noqa: E402

SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task16_dynamic_object_query_route_generation_execution_and_wrapper"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
DEFAULT_STAGE = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
DEFAULT_OUT = TASKS_ROOT / TASK_NAME
DEFAULT_PROFILE = DEFAULT_STAGE / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"
INDEX_PATH = TASKS_ROOT / "task14a_object_nav_experiment_adapter/object_candidate_index_v0_1.json"
LAUNCHER = ROOT / "tools/stage1_runtime/launch_scene_gazebo_nav2.sh"
SEMANTIC_BUILDER = ROOT / "tools/stage1_runtime/prepare_scene_semantic_route.py"
EXECUTABLE_BUILDER = ROOT / "tools/stage1_runtime/build_scene_executable_route.py"
RUNTIME = ROOT / "tools/object_nav/run_objectnav_single_object_runtime.py"
DYNAMIC_PUBLISHER = ROOT / "tools/object_nav/publish_dynamic_objectnav_rviz_markers.py"
RVIZ_CONFIG = ROOT / "tools/object_nav/task16_dynamic_objectnav_demo.rviz"
FAILURE_LAYERS = {
    "query_resolution",
    "topology_route_generation",
    "executable_route_generation",
    "approach_candidate_generation",
    "yaw_proxy_generation",
    "bringup",
    "target_room_navigation",
    "approach_position_navigation",
    "yaw_alignment",
    "validation",
    "cleanup",
}


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = float(point[0]), float(point[1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        if (yi > y) != (yj > y):
            intersection = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < intersection:
                inside = not inside
        j = i
    return inside


class NavMap:
    def __init__(self, map_yaml: Path) -> None:
        self.grid, self.resolution, self.origin, self.meta = load_nav_map(map_yaml)
        free = (self.grid >= 250).astype(np.uint8)
        self.clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.resolution

    def rc(self, xy: tuple[float, float]) -> tuple[int, int]:
        return (
            int(round((xy[1] - self.origin[1]) / self.resolution)),
            int(round((xy[0] - self.origin[0]) / self.resolution)),
        )

    def sample(self, xy: tuple[float, float]) -> dict[str, Any]:
        row, col = self.rc(xy)
        in_bounds = 0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]
        value = int(self.grid[row, col]) if in_bounds else None
        free = bool(in_bounds and value is not None and value >= 250)
        return {
            "grid_row": row, "grid_col": col, "in_bounds": in_bounds,
            "pixel_value": value, "state": "free" if free else ("occupied_or_unknown" if in_bounds else "out_of_bounds"),
            "is_free": free, "clearance_m": round(float(self.clearance[row, col]), 6) if in_bounds else None,
        }

    def ray_check(self, start: tuple[float, float], target: tuple[float, float]) -> dict[str, Any]:
        distance = math.hypot(target[0] - start[0], target[1] - start[1])
        steps = max(1, int(math.ceil(distance / (self.resolution * 0.5))))
        violations = []
        for i in range(steps + 1):
            t = i / steps
            sample = self.sample((start[0] + (target[0] - start[0]) * t, start[1] + (target[1] - start[1]) * t))
            if sample["state"] != "free":
                violations.append({"t": round(t, 4), **sample})
        return {
            "status": "passed" if not violations else "failed",
            "distance_m": round(distance, 6),
            "sample_count": steps + 1,
            "violations": violations[:8],
        }


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def command_text(cmd: list[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in cmd)


def run_logged(cmd: list[str], log: Path, env: dict[str, str] | None = None) -> int:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w", encoding="utf-8") as handle:
        handle.write("$ " + command_text(cmd) + "\n")
        handle.flush()
        proc = subprocess.run(cmd, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT, text=True)
    return int(proc.returncode)


def capture(cmd: list[str], env: dict[str, str], timeout: float = 12.0) -> dict[str, Any]:
    try:
        proc = subprocess.run(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return {"command": cmd, "returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
    except Exception as exc:
        return {"command": cmd, "returncode": None, "error": f"{type(exc).__name__}: {exc}"}


def find_index_object(index: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    wanted = canonical_object_id(object_id)
    return next((obj for obj in index.get("objects", []) if obj.get("object_id") == wanted), None)


def resolve_object(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    index = load_index(INDEX_PATH)
    query_result = run_query(index, args.query, preferred_floor_id=args.floor_id, top_k=10)
    selected = query_result.get("selected_candidate")
    rationale = "highest-ranked artifact-backed candidate satisfying parsed room/floor constraints"
    if args.object_id:
        forced = find_index_object(index, args.object_id)
        if forced and forced.get("floor_id") == args.floor_id:
            selected = forced
            rationale = "explicit --object-id disambiguated matching artifact-backed query candidates"
        else:
            selected = None
            query_result["failure_reason"] = "explicit object id not found on requested floor"
    ambiguity = bool(query_result.get("ambiguous_query"))
    plan = {
        "artifact_type": "task16_dynamic_query_resolution",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "query_text": args.query,
        "object_id_request": args.object_id,
        "start_room": args.start_room,
        "floor_id_request": args.floor_id,
        "source_object_candidate_index": rel(INDEX_PATH),
        "candidate_query_result": query_result,
        "query_resolution_success": bool(selected),
        "resolved_object_id": selected.get("object_id") if selected else None,
        "label": selected.get("label") if selected else None,
        "room_id": selected.get("room_id") if selected else None,
        "floor_id": selected.get("floor_id") if selected else None,
        "confidence": selected.get("confidence") if selected else None,
        "provenance": selected.get("source_artifact_provenance") if selected else None,
        "ambiguity_status": "disambiguated_by_object_id" if args.object_id and selected and ambiguity else ("ambiguous" if ambiguity else "unambiguous"),
        "warnings": list(selected.get("warnings") or []) if selected else [],
        "selected_candidate_rationale": rationale if selected else None,
        "non_claims": [
            "Resolution is artifact-backed candidate selection, not semantic ground-truth accuracy.",
            "No open-vocabulary CLIP retrieval is implemented or claimed.",
            "Resolution is not visual object confirmation.",
        ],
    }
    write_json(out / "dynamic_query_resolution.json", plan)
    return selected, plan


def pair_key(a: str, b: str) -> str:
    return "__".join(sorted([a, b], key=lambda room: int(room.split("_")[1])))


def topology_route(args: argparse.Namespace, selected: dict[str, Any], out: Path) -> dict[str, Any]:
    public = args.stage_output_dir / "committed_public"
    topology = read_json(public / "topology_v0_1.json", {})
    model = read_json(public / "committed_room_world_model_v0_1.json", {})
    gateway_path = args.stage_output_dir / f"process/floors/{args.floor_id}/gateway/assets/gateway_registry_v0_1.json"
    gateway_registry = read_json(gateway_path, {})
    rooms = {room["id"]: room for room in topology.get("rooms", []) if room.get("floor_id") == args.floor_id}
    goal = selected["room_id"]
    gateways = {item.get("pair_key"): item for item in gateway_registry.get("gateways", [])}
    adj: dict[str, list[tuple[str, dict[str, Any]]]] = {room: [] for room in rooms}
    allowed_edges: list[dict[str, Any]] = []
    for edge in model.get("adjacency", []):
        a, b = edge.get("source"), edge.get("target")
        pk = pair_key(a, b) if a in rooms and b in rooms else None
        gateway = gateways.get(pk)
        allowed = (
            a in rooms
            and b in rooms
            and edge.get("relation_type") != "vertical_transition"
            and edge.get("status") in {"supported", "confirmed"}
            and gateway is not None
            and (gateway.get("validation") or {}).get("accepted_for_carving", True)
        )
        record = {**edge, "pair_key": pk, "gateway_id": (gateway or {}).get("gateway_id"), "eligible": allowed}
        if allowed:
            allowed_edges.append(record)
            adj[a].append((b, record))
            adj[b].append((a, record))
    queue: deque[str] = deque([args.start_room])
    prev: dict[str, tuple[str, dict[str, Any]] | None] = {args.start_room: None}
    while queue:
        cur = queue.popleft()
        if cur == goal:
            break
        for nxt, edge in sorted(adj.get(cur, []), key=lambda row: row[0]):
            if nxt not in prev:
                prev[nxt] = (cur, edge)
                queue.append(nxt)
    route: list[str] = []
    used: list[dict[str, Any]] = []
    if goal in prev:
        cur = goal
        while cur != args.start_room:
            route.append(cur)
            previous, edge = prev[cur]  # type: ignore[misc]
            used.append({**edge, "traversed_from": previous, "traversed_to": cur})
            cur = previous
        route.append(args.start_room)
        route.reverse()
        used.reverse()
    result = {
        "artifact_type": "task16_generated_topology_route",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "query": args.query,
        "start_room": args.start_room,
        "target_room": goal,
        "floor_id": args.floor_id,
        "route_generated": bool(route),
        "room_sequence": route,
        "topology_nodes_used": route,
        "topology_edges_used": used,
        "gateway_sequence": [edge["gateway_id"] for edge in used],
        "rationale": "shortest same-floor route over supported/confirmed committed topology relations with accepted gateway projection assets",
        "failure_reason": None if route else "no same-floor gateway-supported topology path from start room to target room",
        "authority_artifacts": [
            rel(public / "topology_v0_1.json"),
            rel(public / "topology_query_report.json"),
            rel(public / "committed_room_world_snapshot_v0_1.json"),
            rel(public / "committed_room_world_model_v0_1.json"),
        ],
        "projection_gateway_registry": rel(gateway_path),
        "eligible_same_floor_edges": allowed_edges,
    }
    write_json(out / "generated_topology_route.json", result)
    return result


def build_routes(args: argparse.Namespace, out: Path, topo: dict[str, Any], commands: dict[str, Any], env: dict[str, str]) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    rooms = topo["room_sequence"]
    map_yaml = args.stage_output_dir / f"maps/{args.floor_id}/stage1_{args.floor_id}_stable_occupancy_map.yaml"
    semantic_path = out / "semantic_route_waypoints_v0_1.json"
    route_query_path = out / "route_query_result_v0_1.json"
    semantic_validation = out / "route_generation_semantic_validation.json"
    semantic_cmd = [
        "/usr/bin/python3", str(SEMANTIC_BUILDER), "--scene-id", SCENE_ID, "--floor-id", args.floor_id,
        "--stage-output-dir", str(args.stage_output_dir), "--runtime-profile", str(args.runtime_profile),
        "--map-yaml", str(map_yaml), "--start-room", rooms[0], "--goal-room", rooms[-1],
        "--through-rooms", ",".join(rooms[1:-1]), "--route-query-json", str(route_query_path),
        "--waypoints-json", str(semantic_path), "--output-json", str(semantic_validation), "--force-regenerate",
    ]
    commands["semantic_route_generation_command"] = semantic_cmd
    semantic_rc = run_logged(semantic_cmd, out / "run_logs/semantic_route_generation.log", env)
    semantic = read_json(semantic_path, None)
    if semantic_rc != 0 or not semantic:
        return None, None
    semantic["room_sequence"] = rooms
    semantic["gateway_sequence"] = topo["gateway_sequence"]
    semantic["generation_provenance"] = {
        "topology_route": rel(out / "generated_topology_route.json"),
        "method": "gateway-aware semantic route generation from dynamically selected topology route",
    }
    write_json(semantic_path, semantic)
    write_json(out / "generated_semantic_route.json", semantic)
    executable_path = out / "executable_route_waypoints_v0_1.json"
    diagnostic_path = out / "route_generation_report.json"
    build_cmd = [
        "/usr/bin/python3", str(EXECUTABLE_BUILDER), "--stage-output-dir", str(args.stage_output_dir),
        "--floor-id", args.floor_id, "--map-yaml", str(map_yaml), "--semantic-waypoints-json", str(semantic_path),
        "--output-json", str(executable_path), "--diagnostics-json", str(diagnostic_path),
        "--output-md", str(out / "executable_route_report.md"),
    ]
    commands["executable_route_generation_command"] = build_cmd
    executable_rc = run_logged(build_cmd, out / "run_logs/executable_route_generation.log", env)
    executable = read_json(executable_path, None)
    diagnostics = read_json(diagnostic_path, {})
    if executable_rc != 0 or not executable or not diagnostics.get("passed"):
        return semantic, None
    executable["dynamic_route_provenance"] = {
        "query": args.query,
        "generated_topology_route": rel(out / "generated_topology_route.json"),
        "generated_semantic_route": rel(semantic_path),
        "not_a_manually_selected_room14_destination_route": True,
    }
    write_json(executable_path, executable)
    write_json(out / "generated_executable_route.json", executable)
    compare_prefix_reference(executable, args.stage_output_dir, out)
    visualize_plan(executable, semantic, None, out / "route_visualization.png")
    return semantic, executable


def compare_prefix_reference(executable: dict[str, Any], stage_dir: Path, out: Path) -> None:
    reference_path = stage_dir / "routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json"
    reference = read_json(reference_path, {})
    generated = executable.get("waypoints") or []
    reference_points = reference.get("waypoints") or []
    exact_prefix = len(reference_points) >= len(generated) and all(
        abs(float(a["x"]) - float(b["x"])) < 1e-8
        and abs(float(a["y"]) - float(b["y"])) < 1e-8
        and a.get("source") == b.get("source")
        for a, b in zip(generated, reference_points)
    )
    write_json(out / "route_reference_comparison.json", {
        "artifact_type": "task16_route_reference_comparison",
        "created_utc": now_iso(),
        "generated_route": rel(out / "executable_route_waypoints_v0_1.json"),
        "existing_room14_route_reference_only": rel(reference_path),
        "generated_room_sequence": executable.get("room_sequence"),
        "existing_room_sequence": reference.get("room_sequence"),
        "generated_is_exact_prefix_of_existing_route": exact_prefix,
        "statement": "The task16 destination route was generated from the selected topology route; the existing room14 route is compared only as a validation reference.",
    })


def object_proxy(snapshot: dict[str, Any], obj: dict[str, Any], selected_record: dict[str, Any] | None) -> dict[str, Any] | None:
    object_id = canonical_object_id(obj["object_id"])
    for anchor in snapshot.get("anchors", []):
        if anchor.get("anchor_type") == "object" and canonical_object_id(anchor.get("target_id")) == object_id and anchor.get("valid", True):
            xy = anchor.get("position") or []
            if len(xy) >= 2:
                return {
                    "yaw_proxy_source": "committed_room_world_snapshot_object_anchor",
                    "yaw_proxy_xy": [float(xy[0]), float(xy[1])],
                    "anchor_id": anchor.get("id"),
                    "reliable_or_derived": "committed_proxy_not_visual_confirmation",
                    "reliable": True,
                }
    visible = (selected_record or {}).get("visible_proxy_xy")
    if visible:
        return {
            "yaw_proxy_source": "derived_visible_footprint_edge_proxy",
            "yaw_proxy_xy": [float(visible[0]), float(visible[1])],
            "anchor_id": None,
            "reliable_or_derived": "derived_from_committed_object_footprint",
            "reliable": False,
        }
    pose = obj.get("pose_xy")
    if pose:
        return {
            "yaw_proxy_source": "derived_committed_object_centroid_proxy",
            "yaw_proxy_xy": [float(pose[0]), float(pose[1])],
            "anchor_id": None,
            "reliable_or_derived": "derived_centroid_not_visual_confirmation",
            "reliable": False,
        }
    return None


def generate_approach(args: argparse.Namespace, selected: dict[str, Any], executable: dict[str, Any], out: Path) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    map_yaml = args.stage_output_dir / f"maps/{args.floor_id}/stage1_{args.floor_id}_stable_occupancy_map.yaml"
    public = args.stage_output_dir / "committed_public"
    snapshot = read_json(public / "committed_room_world_snapshot_v0_1.json", {})
    topology = read_json(public / "topology_v0_1.json", {})
    proxy = object_proxy(snapshot, selected, None)
    if not proxy:
        write_json(out / "yaw_proxy_report.json", {"available": False, "failure_reason": "no object anchor, footprint proxy, or committed centroid available"})
        return None, None
    rooms = {room.get("id"): room for room in topology.get("rooms", [])}
    room_polygon = (rooms.get(selected["room_id"]) or {}).get("polygon") or []
    footprint = selected.get("footprint_2d") or []
    object_xy = tuple(float(v) for v in selected.get("pose_xy") or proxy["yaw_proxy_xy"])
    proxy_xy = tuple(proxy["yaw_proxy_xy"])
    terminal = (executable.get("waypoints") or [{}])[-1]
    terminal_xy = (float(terminal["x"]), float(terminal["y"]))
    map_data = NavMap(map_yaml)
    other_footprints = [
        (canonical_object_id(item.get("id")), item.get("label"), item.get("footprint_2d"))
        for item in snapshot.get("objects", [])
        if item.get("room_id") == selected["room_id"]
        and canonical_object_id(item.get("id")) != selected["object_id"]
        and item.get("footprint_2d")
    ]
    raw_candidates: list[tuple[str, tuple[float, float], str, dict[str, Any]]] = [
        ("object_centroid_rejected", object_xy, "object_centroid", {"reason": "centroid is not a robot goal"}),
        ("route_terminal_seed", terminal_xy, "route_terminal_seed", {}),
    ]
    if footprint:
        xs = [float(point[0]) for point in footprint]
        ys = [float(point[1]) for point in footprint]
        half_w = max(0.05, (max(xs) - min(xs)) / 2.0)
        half_h = max(0.05, (max(ys) - min(ys)) / 2.0)
        number = 0
        for standoff in (0.6, 0.8, 1.0):
            for degrees in range(0, 360, 22):
                angle = math.radians(degrees)
                ux, uy = math.cos(angle), math.sin(angle)
                edge_x = half_w / abs(ux) if abs(ux) > 1e-9 else float("inf")
                edge_y = half_h / abs(uy) if abs(uy) > 1e-9 else float("inf")
                support = min(edge_x, edge_y)
                xy = (object_xy[0] + ux * (support + standoff), object_xy[1] + uy * (support + standoff))
                raw_candidates.append((f"generated_ring_{number:03d}", xy, "generated_standoff_ring", {"standoff_m": standoff, "angle_deg": degrees}))
                number += 1
    records: list[dict[str, Any]] = []
    for candidate_id, xy, source, details in raw_candidates:
        target_overlap = bool(footprint and point_in_polygon(xy[0], xy[1], footprint))
        overlap_objects = [
            {"object_id": object_id, "label": label}
            for object_id, label, polygon in other_footprints
            if point_in_polygon(xy[0], xy[1], polygon)
        ]
        inside_room = bool(room_polygon and point_in_polygon(xy[0], xy[1], room_polygon))
        sample = map_data.sample(xy)
        proxy_ray = map_data.ray_check(xy, proxy_xy)
        rejected: list[str] = []
        if source == "object_centroid":
            rejected.append("object_centroid_is_not_navigation_goal")
        if target_overlap:
            rejected.append("overlaps_target_object_footprint")
        if overlap_objects:
            rejected.append("overlaps_other_committed_object_footprint_proxy")
        if not inside_room:
            rejected.append("outside_target_room")
        if not sample["is_free"]:
            rejected.append("occupied_or_unknown_map_cell")
        if sample["clearance_m"] is None or float(sample["clearance_m"]) < 0.2:
            rejected.append("insufficient_map_clearance")
        if proxy_ray["status"] != "passed":
            rejected.append("yaw_proxy_ray_crosses_occupied_or_unknown_cells")
        route_distance = math.hypot(xy[0] - terminal_xy[0], xy[1] - terminal_xy[1])
        proxy_distance = math.hypot(xy[0] - proxy_xy[0], xy[1] - proxy_xy[1])
        score = -route_distance - abs(proxy_distance - 0.85) + min(float(sample["clearance_m"] or 0.0), 1.0)
        records.append({
            "candidate_id": candidate_id,
            "candidate_source": source,
            "floor_id": selected["floor_id"],
            "room_id": selected["room_id"],
            "world_xy": [round(xy[0], 6), round(xy[1], 6)],
            "yaw": round(math.atan2(proxy_xy[1] - xy[1], proxy_xy[0] - xy[0]), 6),
            "yaw_policy": "face_committed_object_anchor_proxy" if proxy["reliable"] else "face_derived_object_proxy",
            "visible_proxy_xy": list(proxy_xy),
            "target_object_footprint_overlap": target_overlap,
            "semantic_object_footprint_overlaps": overlap_objects,
            "inside_target_room": inside_room,
            "map_sample": sample,
            "yaw_proxy_ray_check": proxy_ray,
            "distance_to_route_terminal_m": round(route_distance, 6),
            "distance_to_visible_proxy_m": round(proxy_distance, 6),
            "generation_details": details,
            "rejected_reasons": rejected,
            "hard_status": "passed" if not rejected else "failed",
            "eligible_for_recommendation": not rejected,
            "total_score": round(score, 6),
        })
    valid = [row for row in records if row["eligible_for_recommendation"] and row["candidate_source"] == "generated_standoff_ring"]
    recommended = max(valid, key=lambda row: float(row["total_score"])) if valid else None
    base_report = {
        "artifact_type": "task16_dynamic_object_approach_candidate_report",
        "created_utc": now_iso(),
        "query": args.query,
        "object_id": selected["object_id"],
        "label": selected.get("label"),
        "room_id": selected["room_id"],
        "floor_id": selected["floor_id"],
        "source_generation_method": "standoff_ring_from_committed_object_footprint_then_occupancy_room_overlap_and_ray_checks",
        "object_centroid_xy": list(object_xy),
        "target_object_footprint": footprint,
        "target_object_proxy_xy": proxy["yaw_proxy_xy"],
        "candidate_records": sorted(records, key=lambda row: row["total_score"], reverse=True),
        "recommended_candidate": recommended,
        "selected_candidate_id": recommended.get("candidate_id") if recommended else None,
        "selection_reason": "highest-scored generated standoff candidate passing room, occupancy, clearance, object-overlap, and proxy-ray checks" if recommended else None,
        "rejected_candidates": [{"candidate_id": row["candidate_id"], "reasons": row["rejected_reasons"]} for row in records if row["rejected_reasons"]],
        "required_rejection_checks": [
            "target object footprint overlap",
            "other committed object footprint proxy overlap",
            "target room membership",
            "occupied/unknown map cell and clearance",
            "occupied/unknown proxy-facing ray",
        ],
        "limitations": [
            "Approach generation uses committed proxy geometry and occupancy data; it is not visual object confirmation.",
            "Stable occupancy validation does not prove semantic ground-truth accuracy.",
        ],
    }
    write_json(out / "approach_candidate_report.json", base_report)
    write_json(out / "approach_candidate_reports" / f"object_approach_report_{selected['object_id']}.json", base_report)
    yaw_report = {
        "artifact_type": "task16_yaw_proxy_report",
        "created_utc": now_iso(),
        "object_id": selected["object_id"],
        **proxy,
        "expected_yaw_rad": recommended.get("yaw") if recommended else None,
        "selected_candidate_id": recommended.get("candidate_id") if recommended else None,
        "statement": "The yaw target is an artifact-backed proxy for facing alignment; it is not visual object confirmation.",
        "available": True,
    }
    write_json(out / "yaw_proxy_report.json", yaw_report)
    visualize_plan(executable, read_json(out / "semantic_route_waypoints_v0_1.json", {}), recommended, out / "route_visualization.png")
    return recommended, yaw_report


def visualize_plan(executable: dict[str, Any], semantic: dict[str, Any], candidate: dict[str, Any] | None, path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    route = executable.get("waypoints") or []
    sparse = semantic.get("waypoints") or []
    if not route:
        return
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot([p["x"] for p in route], [p["y"] for p in route], "-", color="#1679c4", label="generated executable route")
    ax.scatter([p["x"] for p in sparse], [p["y"] for p in sparse], color="#ef8b24", label="semantic/gateway anchors")
    if candidate:
        xy = candidate["world_xy"]
        proxy = candidate.get("visible_proxy_xy")
        ax.scatter([xy[0]], [xy[1]], marker="X", color="#239b56", s=85, label="selected approach")
        if proxy:
            ax.plot([xy[0], proxy[0]], [xy[1], proxy[1]], "--", color="#c2185b", label="facing ray")
            ax.scatter([proxy[0]], [proxy[1]], color="#c2185b", label="yaw proxy")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title("RSLG-SLAM task16 dynamic object-nav plan")
    ax.legend(fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def graph_snapshot(env: dict[str, str], path: Path) -> dict[str, Any]:
    data = {
        "created_utc": now_iso(),
        "nodes": capture(["ros2", "node", "list"], env),
        "topics": capture(["ros2", "topic", "list"], env),
        "actions": capture(["ros2", "action", "list"], env),
    }
    write_json(path, data)
    return data


def stop_process(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=8)
    except Exception:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def cleanup_runtime_remnants(path: Path, include_gui_client: bool = False) -> dict[str, Any]:
    snapshot = subprocess.run(["ps", "-eo", "pid=,args="], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    patterns = [
        "turtlebot3_gazebo/launch/robot_state_publisher.launch.py",
        "/opt/ros/foxy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf",
    ]
    if include_gui_client:
        patterns.append("gzclient")
    terminated: list[dict[str, Any]] = []
    for line in snapshot.stdout.splitlines():
        parts = line.strip().split(maxsplit=1)
        if len(parts) != 2:
            continue
        pid, command = int(parts[0]), parts[1]
        if pid == os.getpid() or not any(pattern in command for pattern in patterns):
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            terminated.append({"pid": pid, "command": command, "signal": "SIGTERM"})
        except ProcessLookupError:
            pass
    if terminated:
        time.sleep(1.0)
    payload = {"created_utc": now_iso(), "safe_patterns": patterns, "terminated": terminated}
    write_json(path, payload)
    return payload


def combine_trajectories(run_dir: Path, evidence: Path) -> bool:
    samples: list[dict[str, Any]] = []
    sources: list[str] = []
    for p in sorted((run_dir / "executed_trajectories").glob("*.json")):
        data = read_json(p, {})
        rows = data.get("samples") or []
        if rows:
            sources.append(rel(p) or "")
            samples.extend({**row, "source_file": rel(p)} for row in rows)
    if not samples:
        return False
    evidence.mkdir(parents=True, exist_ok=True)
    write_json(evidence / "executed_trajectory_combined.json", {"artifact_type": "task16_executed_trajectory", "sources": sources, "samples": samples})
    keys = ["x", "y", "yaw", "source_file"]
    with (evidence / "executed_trajectory_combined.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in samples:
            writer.writerow({key: row.get(key, "") for key in keys})
    return True


def screenshot(path: Path, env: dict[str, str]) -> dict[str, Any]:
    cmd = [
        "/usr/bin/python3", "-c",
        "import sys; from PyQt5.QtWidgets import QApplication; app=QApplication([]); "
        "ok=app.primaryScreen().grabWindow(0).save(sys.argv[1]); sys.exit(0 if ok else 2)",
        str(path),
    ]
    result = capture(cmd, env, 15.0)
    result["path"] = rel(path) if path.exists() else None
    result["saved"] = path.exists() and path.stat().st_size > 0
    result["visible_confirmation"] = False
    return result


def normalize_runtime_result(raw: dict[str, Any], planning: dict[str, Any], launch_ok: bool, execution_attempted: bool) -> dict[str, Any]:
    failure_layer = None
    reason = raw.get("failure_reason")
    if execution_attempted and not launch_ok:
        failure_layer, reason = "bringup", reason or "Gazebo/Nav2 bringup failed"
    elif execution_attempted and not raw.get("target_room_arrival"):
        failure_layer = "target_room_navigation"
    elif execution_attempted and not raw.get("approach_position_reached"):
        failure_layer = "approach_position_navigation"
    elif execution_attempted and not raw.get("approach_yaw_aligned"):
        failure_layer = "yaw_alignment"
    if reason is None and failure_layer:
        reason = raw.get("failure_reason") or f"{failure_layer} did not meet success criteria"
    result = {
        "artifact_type": "task16_dynamic_objectnav_runtime_result",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        **planning,
        "runtime_execution_attempted": execution_attempted,
        "gazebo_nav2_started": launch_ok,
        "target_room_arrival": bool(raw.get("target_room_arrival")),
        "approach_position_reached": bool(raw.get("approach_position_reached")),
        "approach_yaw_aligned": bool(raw.get("approach_yaw_aligned")),
        "object_facing_approach_success": bool(raw.get("object_facing_approach_success")),
        "nav2_clean_approach_success": bool(raw.get("nav2_clean_approach_success")),
        "room_route_sparse_fallback_used": bool(raw.get("room_route_sparse_fallback_used")),
        "yaw_alignment_direct_cmd_vel_fallback_used": bool(raw.get("yaw_alignment_direct_cmd_vel_fallback_used")),
        "fallback_used": bool(raw.get("fallback_used")),
        "final_distance_to_target_room_terminal_m": raw.get("final_distance_to_target_room_terminal_m"),
        "final_distance_to_approach_candidate_m": raw.get("final_distance_to_selected_approach_candidate_m"),
        "final_yaw_error_rad": raw.get("yaw_error_rad"),
        "final_yaw_error_deg": raw.get("yaw_error_deg"),
        "yaw_tolerance_rad": raw.get("yaw_alignment_tolerance_rad", 0.5),
        "xy_drift_during_yaw_alignment_m": raw.get("xy_drift_during_yaw_alignment_m"),
        "wall_crossing_validation_passed": raw.get("wall_crossing_validation_passed"),
        "failure_layer": failure_layer,
        "failure_reason": reason if failure_layer else None,
        "source_runtime_result": raw,
    }
    return result


def execute_runtime(args: argparse.Namespace, out: Path, selected: dict[str, Any], candidate: dict[str, Any], planning: dict[str, Any], commands: dict[str, Any], env: dict[str, str]) -> dict[str, Any]:
    run_dir = out / "runtime_run"
    evidence = out / "evidence"
    logs = out / "run_logs"
    for directory in (run_dir, evidence, logs):
        directory.mkdir(parents=True, exist_ok=True)
    map_yaml = args.stage_output_dir / f"maps/{args.floor_id}/stage1_{args.floor_id}_stable_occupancy_map.yaml"
    launch_cmd = [
        str(LAUNCHER), "--scene-id", SCENE_ID, "--floor-id", args.floor_id, "--stage-output-dir", str(args.stage_output_dir),
        "--runtime-profile", str(args.runtime_profile), "--map-yaml", str(map_yaml),
        "--gui" if args.gui else "--headless", "--ros-domain-id", str(args.ros_domain_id),
        "--log-dir", str(run_dir / "bringup_logs"), "--readiness-timeout-sec", "120",
        "--readiness-output-json", str(run_dir / "runtime_bringup_readiness.json"),
        "--readiness-output-md", str(run_dir / "runtime_bringup_readiness.md"), "--run-id", f"{TASK_NAME}_{selected['object_id']}",
    ]
    stop_cmd = [str(LAUNCHER), "--stage-output-dir", str(args.stage_output_dir), "--runtime-profile", str(args.runtime_profile), "--stop"]
    runtime_cmd = [
        "/usr/bin/python3", str(RUNTIME), "--stage-output-dir", str(args.stage_output_dir), "--task14c-output-dir", str(out),
        "--output-dir", str(run_dir), "--query", args.query, "--object-id", selected["object_id"],
        "--approach-candidate-id", candidate["candidate_id"], "--floor-id", args.floor_id, "--start-room", args.start_room,
        "--target-room", selected["room_id"], "--stable-map", str(map_yaml), "--semantic-route", str(out / "semantic_route_waypoints_v0_1.json"),
        "--room-route", str(out / "executable_route_waypoints_v0_1.json"), "--controller-profile", "task12_robust",
        "--execution-strategy", "split_follow_path_with_current_pose_gateway_handoff", "--two-segment-execution",
        "--object-facing-approach", "--position-first-approach", "--yaw-alignment-required",
        "--approach-position-tolerance-m", "0.35", "--yaw-alignment-tolerance-rad", "0.50",
        "--validate-wall-crossing", "--record-sent-controller-paths", "--record-executed-trajectory",
    ]
    if args.use_current_robot_pose_if_running:
        runtime_cmd.append("--use-current-robot-pose-if-running")
    commands.update({"launch_command": launch_cmd, "runtime_command": runtime_cmd, "stop_command": stop_cmd})
    write_json(out / "exact_runtime_commands.json", commands)
    launch_ok = False
    runtime_rc: int | None = None
    marker_proc: subprocess.Popen[Any] | None = None
    rviz_proc: subprocess.Popen[Any] | None = None
    screen: dict[str, Any] = {"saved": False, "visible_confirmation": False}
    try:
        if not args.use_current_robot_pose_if_running:
            run_logged(stop_cmd, logs / "prelaunch_stop.log", env)
            cleanup_runtime_remnants(evidence / "prelaunch_stale_process_cleanup.json", include_gui_client=args.gui)
            launch_ok = run_logged(launch_cmd, logs / "gazebo_nav2_bringup.log", env) == 0
        else:
            graph = graph_snapshot(env, evidence / "ros_graph_preexisting.json")
            launch_ok = "/navigate_to_pose" in (graph["actions"].get("stdout") or "")
            if not launch_ok:
                launch_ok = run_logged(launch_cmd, logs / "gazebo_nav2_bringup.log", env) == 0
        graph_snapshot(env, evidence / "ros_graph_after_launch.json")
        if launch_ok and args.gui and DYNAMIC_PUBLISHER.exists():
            marker_cmd = [
                "/usr/bin/python3", str(DYNAMIC_PUBLISHER), "--plan-json", str(out / "dynamic_query_plan.json"),
                "--run-dir", str(run_dir), "--evidence-dir", str(evidence), "--route-json", str(out / "executable_route_waypoints_v0_1.json"),
                "--semantic-route-json", str(out / "semantic_route_waypoints_v0_1.json"), "--marker-topic", "/task16/dynamic_object_nav_markers",
            ]
            commands["marker_publisher_command"] = marker_cmd
            commands["rviz_command"] = ["rviz2", "-d", str(RVIZ_CONFIG)]
            write_json(out / "exact_runtime_commands.json", commands)
            marker_log = (logs / "marker_publisher.log").open("w", encoding="utf-8")
            marker_proc = subprocess.Popen(marker_cmd, cwd=ROOT, env=env, stdout=marker_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            rviz_log = (logs / "rviz2.log").open("w", encoding="utf-8")
            rviz_proc = subprocess.Popen(commands["rviz_command"], cwd=ROOT, env=env, stdout=rviz_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            time.sleep(3.0)
        if launch_ok:
            runtime_rc = run_logged(runtime_cmd, logs / "dynamic_objectnav_runtime.log", env)
        if args.gui:
            time.sleep(max(2.0, args.keep_open_sec))
            graph_snapshot(env, evidence / "ros_graph_after_runtime.json")
            screen = screenshot(evidence / "gui_rviz_screenshot.png", env)
            write_json(evidence / "screenshot_capture.json", screen)
    finally:
        stop_process(rviz_proc)
        stop_process(marker_proc)
        if not args.use_current_robot_pose_if_running:
            cleanup_rc = run_logged(stop_cmd, logs / "final_stop.log", env)
            remnant_cleanup = cleanup_runtime_remnants(evidence / "postrun_stale_process_cleanup.json", include_gui_client=args.gui)
        else:
            cleanup_rc = 0
            remnant_cleanup = {"terminated": []}
        write_json(evidence / "cleanup_report.json", {
            "created_utc": now_iso(),
            "stop_command_executed": not args.use_current_robot_pose_if_running,
            "stop_returncode": cleanup_rc,
            "runtime_specific_remnants_terminated": remnant_cleanup.get("terminated"),
            "cleanup_passed": cleanup_rc == 0,
        })
    raw = read_json(run_dir / "object_facing_runtime_result.json", {})
    combine_trajectories(run_dir, evidence)
    result = normalize_runtime_result(raw, planning, launch_ok, True)
    result.update({
        "runtime_returncode": runtime_rc,
        "runtime_run_dir": rel(run_dir),
        "sent_controller_paths": rel(run_dir / "sent_controller_paths"),
        "executed_trajectory_json": rel(evidence / "executed_trajectory_combined.json"),
        "gui_rviz_launched": bool(args.gui and launch_ok),
        "marker_evidence_saved": (evidence / "marker_manifest.json").exists(),
        "screenshot_saved": bool(screen.get("saved")),
        "screenshot_visible_confirmation": bool(screen.get("visible_confirmation")),
        "map_render_warning": "GLSL link result" in (logs / "rviz2.log").read_text(encoding="utf-8", errors="replace") if (logs / "rviz2.log").exists() else False,
    })
    write_json(out / "runtime_result.json", result)
    return result


def write_reports(out: Path, plan: dict[str, Any], runtime: dict[str, Any]) -> None:
    summary = {
        "artifact_type": "task16_dynamic_objectnav_summary",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "dynamic_query_plan": rel(out / "dynamic_query_plan.json"),
        "runtime_result": rel(out / "runtime_result.json"),
        **{key: runtime.get(key) for key in [
            "query_resolution_success", "target_room_resolved", "dynamic_room_route_generated",
            "executable_route_generated", "approach_candidate_generated_or_selected", "yaw_proxy_available",
            "gazebo_nav2_started", "target_room_arrival", "approach_position_reached", "approach_yaw_aligned",
            "object_facing_approach_success", "nav2_clean_approach_success", "room_route_sparse_fallback_used",
            "yaw_alignment_direct_cmd_vel_fallback_used", "fallback_used", "wall_crossing_validation_passed",
            "failure_layer", "failure_reason",
        ]},
        "allowed_claims": [
            "Artifact-backed object query resolution to a reported target room/floor." if runtime.get("query_resolution_success") else None,
            "Dynamic topology-derived room-route generation for a non-room14 floor_2 target." if runtime.get("dynamic_room_route_generated") else None,
            "Dynamic approach candidate generation/selection from committed proxy geometry." if runtime.get("approach_candidate_generated_or_selected") else None,
            "Generated-route Gazebo/Nav2 execution for the selected non-room14 object." if runtime.get("object_facing_approach_success") else None,
        ],
        "forbidden_claims": [
            "semantic ground-truth accuracy",
            "open-vocabulary CLIP retrieval",
            "visual object found or visual object confirmation",
            "cross-floor navigation or TurtleBot3 stair traversal",
            "Nav2-clean object-facing approach when direct /cmd_vel yaw fallback is used",
            "RViz visual confirmation without informative visible evidence",
            "broad cross-sequence generalization",
        ],
    }
    summary["allowed_claims"] = [claim for claim in summary["allowed_claims"] if claim]
    write_json(out / "summary.json", summary)
    lines = [
        "# task16 Dynamic Object-Query Navigation",
        "",
        f"- Query: `{plan.get('query')}`; resolved object: `{plan.get('resolved_object_id')}` (`{plan.get('label')}`).",
        f"- Dynamic room route: `{' -> '.join(plan.get('room_sequence') or [])}`.",
        f"- Selected approach candidate: `{plan.get('selected_candidate_id')}` at `{plan.get('selected_candidate_xy')}`.",
        f"- Yaw proxy: `{plan.get('yaw_proxy_source')}` at `{plan.get('yaw_proxy_xy')}`.",
        f"- Runtime attempted: `{runtime.get('runtime_execution_attempted')}`; Gazebo/Nav2 started: `{runtime.get('gazebo_nav2_started')}`.",
        "",
        "| target-room arrival | approach position | yaw aligned | object-facing success | Nav2-clean | fallback | failure layer |",
        "| --- | --- | --- | --- | --- | --- | --- |",
        f"| {runtime.get('target_room_arrival')} | {runtime.get('approach_position_reached')} | {runtime.get('approach_yaw_aligned')} | {runtime.get('object_facing_approach_success')} | {runtime.get('nav2_clean_approach_success')} | {runtime.get('fallback_used')} | {runtime.get('failure_layer')} |",
        "",
        "The query and yaw target are derived from committed/public model artifacts; they do not establish visual object confirmation or semantic ground truth.",
    ]
    write_text(out / "report.md", "\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id")
    parser.add_argument("--start-room", default="room_11")
    parser.add_argument("--floor-id", default="floor_2")
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--gui", action="store_true")
    display.add_argument("--headless", action="store_true")
    parser.add_argument("--keep-open-sec", type=float, default=4.0)
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "84"))
    parser.add_argument("--runtime-profile", type=Path, default=DEFAULT_PROFILE)
    run_mode = parser.add_mutually_exclusive_group()
    run_mode.add_argument("--execute", action="store_true")
    run_mode.add_argument("--plan-only", action="store_true")
    parser.add_argument("--use-current-robot-pose-if-running", action="store_true")
    args = parser.parse_args()
    args.stage_output_dir = args.stage_output_dir.resolve()
    args.runtime_profile = args.runtime_profile.resolve()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env.setdefault("TURTLEBOT3_MODEL", "burger")
    commands: dict[str, Any] = {
        "artifact_type": "task16_exact_runtime_commands",
        "created_utc": now_iso(),
        "top_level_command": sys.argv,
        "environment": {"ROS_DOMAIN_ID": str(args.ros_domain_id), "TURTLEBOT3_MODEL": env["TURTLEBOT3_MODEL"], "DISPLAY": env.get("DISPLAY")},
    }
    selected, resolution = resolve_object(args, out)
    planning_metrics = {
        "query": args.query,
        "object_id": selected.get("object_id") if selected else None,
        "target_room": selected.get("room_id") if selected else None,
        "floor_id": selected.get("floor_id") if selected else None,
        "query_resolution_success": bool(selected),
        "target_room_resolved": bool(selected and selected.get("room_id")),
        "dynamic_room_route_generated": False,
        "executable_route_generated": False,
        "approach_candidate_generated_or_selected": False,
        "yaw_proxy_available": False,
    }
    if not selected:
        result = {**planning_metrics, "runtime_execution_attempted": False, "failure_layer": "query_resolution", "failure_reason": resolution.get("candidate_query_result", {}).get("failure_reason")}
        write_json(out / "runtime_result.json", result)
        write_reports(out, {"query": args.query}, result)
        return 1
    topo = topology_route(args, selected, out)
    if not topo["route_generated"]:
        result = {**planning_metrics, "runtime_execution_attempted": False, "failure_layer": "topology_route_generation", "failure_reason": topo["failure_reason"]}
        write_json(out / "runtime_result.json", result)
        write_reports(out, {"query": args.query, "resolved_object_id": selected["object_id"], "label": selected.get("label")}, result)
        return 1
    planning_metrics["dynamic_room_route_generated"] = True
    semantic, executable = build_routes(args, out, topo, commands, env)
    if not executable:
        result = {**planning_metrics, "runtime_execution_attempted": False, "failure_layer": "executable_route_generation", "failure_reason": "gateway-aware executable route projection failed; see route generation logs/report"}
        write_json(out / "runtime_result.json", result)
        write_reports(out, {"query": args.query, "resolved_object_id": selected["object_id"], "label": selected.get("label"), "room_sequence": topo["room_sequence"]}, result)
        return 1
    planning_metrics["executable_route_generated"] = True
    candidate, yaw_proxy = generate_approach(args, selected, executable, out)
    if not candidate:
        layer = "yaw_proxy_generation" if not yaw_proxy else "approach_candidate_generation"
        result = {**planning_metrics, "runtime_execution_attempted": False, "failure_layer": layer, "failure_reason": "no executable object standoff candidate with an available yaw proxy"}
        write_json(out / "runtime_result.json", result)
        write_reports(out, {"query": args.query, "resolved_object_id": selected["object_id"], "label": selected.get("label"), "room_sequence": topo["room_sequence"]}, result)
        return 1
    planning_metrics["approach_candidate_generated_or_selected"] = True
    planning_metrics["yaw_proxy_available"] = bool(yaw_proxy)
    plan = {
        "artifact_type": "task16_dynamic_object_query_plan",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": args.query,
        "query_resolution": resolution,
        "resolved_object_id": selected["object_id"],
        "label": selected.get("label"),
        "room_id": selected.get("room_id"),
        "floor_id": selected.get("floor_id"),
        "start_room": args.start_room,
        "room_sequence": topo["room_sequence"],
        "gateway_sequence": topo["gateway_sequence"],
        "generated_topology_route": rel(out / "generated_topology_route.json"),
        "semantic_route": rel(out / "semantic_route_waypoints_v0_1.json"),
        "executable_route": rel(out / "executable_route_waypoints_v0_1.json"),
        "selected_candidate_id": candidate["candidate_id"],
        "selected_candidate_xy": candidate["world_xy"],
        "approach_candidate_id": candidate["candidate_id"],
        "approach_candidate_world_xy": candidate["world_xy"],
        "object_proxy_target_for_yaw_alignment": {"x": yaw_proxy["yaw_proxy_xy"][0], "y": yaw_proxy["yaw_proxy_xy"][1], "source": yaw_proxy["yaw_proxy_source"]},
        "yaw_proxy_source": yaw_proxy["yaw_proxy_source"],
        "yaw_proxy_xy": yaw_proxy["yaw_proxy_xy"],
        "expected_yaw_rad": candidate["yaw"],
        "dynamic_planning_metrics": planning_metrics,
    }
    write_json(out / "dynamic_query_plan.json", plan)
    write_json(out / "exact_runtime_commands.json", commands)
    if not args.execute:
        result = normalize_runtime_result({}, planning_metrics, False, False)
        result["plan_only"] = True
        write_json(out / "runtime_result.json", result)
    else:
        if args.gui and not env.get("DISPLAY"):
            result = {**planning_metrics, "runtime_execution_attempted": False, "failure_layer": "bringup", "failure_reason": "--gui requested but DISPLAY is unset"}
            write_json(out / "runtime_result.json", result)
        else:
            result = execute_runtime(args, out, selected, candidate, planning_metrics, commands, env)
    write_reports(out, plan, result)
    print(json.dumps({"output_dir": rel(out), "object_id": selected["object_id"], "room_sequence": topo["room_sequence"], "runtime": result}, indent=2, sort_keys=True))
    if not args.execute:
        return 0
    return 0 if result.get("object_facing_approach_success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
