#!/usr/bin/env python3
"""RSLG-SLAM lightweight object-navigation executor (no Nav2) - reusable backend runner.

Extracted from task17c/e successful runs. Calls lightweight_backend modules for:
- Occupancy planning (A*, clearance, validation)
- Control path building (simplification + corner rounding + fallback)
- Curvature-based pure pursuit controller
- Approach + yaw alignment
- Runtime validation
- Reporting and visualization
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
THIS_DIR = Path(__file__).resolve().parent
STAGE_RUNTIME_DIR = ROOT / "tools/stage1_runtime"
for directory in (THIS_DIR, STAGE_RUNTIME_DIR):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

# Backend imports
BACKEND_DIR = THIS_DIR / "lightweight_backend"
if str(BACKEND_DIR.parent) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR.parent))

from lightweight_backend.schemas import (
    ControllerParams, angle_wrap, clamp, distance, now_iso, route_length,
)
from lightweight_backend.occupancy_planner import OccupancyPlanner
from lightweight_backend.control_path_builder import build_control_path
from lightweight_backend.curvature_controller import CurvatureController
from lightweight_backend.approach_yaw import run_approach_and_yaw
from lightweight_backend.runtime_validation import validate_trajectory, validate_trajectory_partial
from lightweight_backend.reporting import (
    write_json, write_text, save_csv, save_telemetry_artifacts,
    generate_approach_yaw_validation_report, generate_summary,
)
from lightweight_backend.visualization import plot_visualization, plot_approach_yaw_local_zoom
from lightweight_backend.tracking_quality import generate_tracking_quality_report, plot_tracking_error
from robot_adapters import adapter_type_for_profile, create_robot_adapter, load_robot_profile
from robot_adapters.base_adapter import BaseRobotAdapter

from object_nav_common import canonical_object_id, load_index, run_query  # noqa: E402

SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task18_reusable_lightweight_rslg_executor_backend"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
DEFAULT_STAGE = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
DEFAULT_OUTPUT = TASKS_ROOT / TASK_NAME
INDEX_PATH = TASKS_ROOT / "task14a_object_nav_experiment_adapter/object_candidate_index_v0_1.json"
DEFAULT_ROBOT_PROFILE = THIS_DIR / "robot_profiles/turtlebot3_burger.yaml"


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


def apply_robot_profile_defaults(args: argparse.Namespace, profile: dict[str, Any]) -> None:
    """Fill task18-compatible controller defaults from the selected robot profile."""
    profile_defaults = {
        "max_linear_speed": "max_linear_speed",
        "max_angular_speed": "max_angular_speed",
        "lookahead_distance": "lookahead_distance",
        "lookahead_min": "lookahead_min",
        "lookahead_max": "lookahead_max",
        "angular_smoothing_alpha": "angular_smoothing_alpha",
        "angular_rate_limit": "angular_rate_limit",
        "inflation_radius_m": "inflation_radius_m",
        "xy_drift_limit_m": "yaw_drift_limit_m",
        "approach_settle_sec": "settle_time_sec",
        "control_rate_hz": "control_rate_hz",
    }
    for arg_name, profile_key in profile_defaults.items():
        if getattr(args, arg_name, None) is None:
            fallback = {
                "lookahead_distance": 0.60,
                "lookahead_min": 0.40,
                "lookahead_max": 1.00,
                "angular_smoothing_alpha": 0.40,
                "angular_rate_limit": 0.15,
            }.get(arg_name)
            setattr(args, arg_name, profile.get(profile_key, fallback))


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    prev = len(polygon) - 1
    for i, pt in enumerate(polygon):
        xi, yi = float(pt[0]), float(pt[1])
        xj, yj = float(polygon[prev][0]), float(polygon[prev][1])
        if (yi > y) != (yj > y):
            at_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < at_x:
                inside = not inside
        prev = i
    return inside


def pair_key(left: str, right: str) -> str:
    return "__".join(sorted([left, right], key=lambda r: int(r.split("_")[1])))


# ─── Artifact loading, query, topology, approach ─────────────────────────────────

def stable_map_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    required = args.stage_output_dir / f"maps/{args.floor_id}/stage1_{args.floor_id}_stable_occupancy_map.yaml"
    selected = (args.map_yaml or required).resolve()
    return required.resolve(), selected


def artifact_paths(args: argparse.Namespace) -> dict[str, Path]:
    public = args.stage_output_dir / "committed_public"
    return {
        "topology": public / "topology_v0_1.json",
        "topology_query_report": public / "topology_query_report.json",
        "committed_room_world_snapshot": public / "committed_room_world_snapshot_v0_1.json",
        "committed_room_world_model": public / "committed_room_world_model_v0_1.json",
        "object_candidate_index": INDEX_PATH,
        "gateway_registry": args.stage_output_dir / f"process/floors/{args.floor_id}/gateway/assets/gateway_registry_v0_1.json",
        "gateway_validation_report": args.stage_output_dir / f"process/floors/{args.floor_id}/gateway/assets/gateway_validation_report_v0_1.json",
        "stable_occupancy_map_yaml": stable_map_paths(args)[1],
    }


def load_artifacts(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any], str | None]:
    required_map, selected_map = stable_map_paths(args)
    paths = artifact_paths(args)
    records = []
    failure = None
    for purpose, path in paths.items():
        exists = path.exists()
        records.append({"purpose": purpose, "path": rel(path), "exists": exists})
        if not exists and purpose != "gateway_validation_report":
            failure = f"required artifact missing: {purpose}: {path}"
    if selected_map != required_map:
        failure = f"map mismatch: expected {required_map}, got {selected_map}"
    report = {
        "artifact_type": "task17c_artifact_loading_report",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "artifact_records": records,
        "execution_occupancy_map_yaml": rel(selected_map),
        "success": failure is None,
        "failure_reason": failure,
    }
    write_json(out / "artifact_loading_report.json", report)
    return {k: read_json(p, {}) for k, p in paths.items() if p.suffix == ".json" and p.exists()}, failure


def resolve_query(args: argparse.Namespace, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    index = load_index(INDEX_PATH)
    query_result = run_query(index, args.query, preferred_floor_id=args.floor_id, top_k=10)
    selected = query_result.get("selected_candidate")
    if args.object_id:
        requested = canonical_object_id(args.object_id)
        selected = next((r for r in index.get("objects", []) if r.get("object_id") == requested and r.get("floor_id") == args.floor_id), None)
    report = {
        "artifact_type": "task17c_dynamic_query_resolution",
        "created_utc": now_iso(),
        "query_text": args.query,
        "object_id_request": args.object_id,
        "query_resolution_success": selected is not None,
        "object_id": selected.get("object_id") if selected else None,
        "target_room": selected.get("room_id") if selected else None,
        "failure_reason": None if selected else "query produced no candidate",
    }
    write_json(out / "dynamic_query_resolution.json", report)
    return selected, report


def generate_topology_route(args: argparse.Namespace, selected: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    topology = artifacts["topology"]
    model = artifacts["committed_room_world_model"]
    registry = artifacts["gateway_registry"]
    rooms = {r["id"]: r for r in topology.get("rooms", []) if r.get("floor_id") == args.floor_id}
    gateways = {item.get("pair_key"): item for item in registry.get("gateways", [])}
    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {rid: [] for rid in rooms}
    for edge in model.get("adjacency", []):
        left, right = edge.get("source"), edge.get("target")
        key = pair_key(left, right) if left in rooms and right in rooms else None
        gw = gateways.get(key)
        accepted = bool(gw and (gw.get("validation") or {}).get("accepted_for_carving", False))
        allowed = bool(left in rooms and right in rooms and edge.get("relation_type") != "vertical_transition" and edge.get("status") in {"supported", "confirmed"} and accepted)
        if allowed:
            adjacency[left].append((right, edge))
            adjacency[right].append((left, edge))
    target = selected["room_id"]
    queue = deque([args.start_room])
    prev: dict[str, tuple[str, dict[str, Any]] | None] = {args.start_room: None}
    while queue:
        cur = queue.popleft()
        if cur == target:
            break
        for neighbor, edge in sorted(adjacency.get(cur, []), key=lambda p: p[0]):
            if neighbor not in prev:
                prev[neighbor] = (cur, edge)
                queue.append(neighbor)
    room_sequence: list[str] = []
    used_edges: list[dict[str, Any]] = []
    if target in prev:
        cur = target
        while cur != args.start_room:
            room_sequence.append(cur)
            prior, edge = prev[cur]  # type: ignore
            used_edges.append({**edge, "traversed_from": prior, "traversed_to": cur})
            cur = prior
        room_sequence.append(args.start_room)
        room_sequence.reverse()
        used_edges.reverse()
    gw_ids = []
    for e in used_edges:
        k = pair_key(e.get("source", ""), e.get("target", ""))
        g = gateways.get(k)
        gw_ids.append((g or {}).get("gateway_id"))
    report = {
        "artifact_type": "task17c_generated_topology_route",
        "created_utc": now_iso(),
        "route_generated": bool(room_sequence),
        "room_sequence": room_sequence,
        "gateway_sequence": gw_ids,
        "failure_reason": None if room_sequence else "no path",
    }
    write_json(out / "generated_topology_route.json", report)
    return report


def semantic_anchors_fn(args: argparse.Namespace, topology_route: dict[str, Any], artifacts: dict[str, Any], out: Path) -> dict[str, Any]:
    rooms = {r["id"]: r for r in artifacts["topology"].get("rooms", [])}
    gateways = {g["gateway_id"]: g for g in artifacts["gateway_registry"].get("gateways", [])}
    room_sequence = topology_route["room_sequence"]
    gateway_sequence = topology_route["gateway_sequence"]
    anchors: list[dict[str, Any]] = []
    for i, rid in enumerate(room_sequence):
        center = rooms[rid]["center"]
        anchors.append({"source": "room_center", "room_id": rid, "x": float(center[0]), "y": float(center[1])})
        if i < len(gateway_sequence) and gateway_sequence[i]:
            gw = gateways[gateway_sequence[i]]
            anchors.append({"source": "gateway", "gateway_id": gw["gateway_id"], "from_room": rid, "to_room": room_sequence[i + 1], "x": float(gw["x"]), "y": float(gw["y"])})
    for i in range(len(anchors) - 1):
        anchors[i]["yaw"] = math.atan2(float(anchors[i + 1]["y"]) - float(anchors[i]["y"]), float(anchors[i + 1]["x"]) - float(anchors[i]["x"]))
    if anchors:
        anchors[-1]["yaw"] = anchors[-2].get("yaw", 0.0) if len(anchors) > 1 else 0.0
    payload = {
        "artifact_type": "task17c_generated_semantic_route",
        "created_utc": now_iso(),
        "room_sequence": room_sequence,
        "gateway_sequence": gateway_sequence,
        "waypoints": anchors,
        "map_yaml": rel(stable_map_paths(args)[1]),
    }
    write_json(out / "generated_semantic_route.json", payload)
    return payload


def executable_route(args: argparse.Namespace, semantic: dict[str, Any], planner: OccupancyPlanner, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    all_points: list[dict[str, Any]] = []
    segment_reports: list[dict[str, Any]] = []
    try:
        for i, (start, goal) in enumerate(zip(semantic["waypoints"], semantic["waypoints"][1:])):
            points, segment = planner.plan_segment((start["x"], start["y"]), (goal["x"], goal["y"]), f"{start['source']}->{goal['source']}")
            segment.update({"segment_index": i, "from_anchor": start, "to_anchor": goal})
            segment_reports.append(segment)
            for pt in points if not all_points else points[1:]:
                pt.update({"waypoint_index": len(all_points), "floor_id": args.floor_id, "source": "inflated_grid_astar", "semantic_segment_index": i, **planner.sample((pt["x"], pt["y"]))})
                all_points.append(pt)
        for l, r in zip(all_points, all_points[1:]):
            l["yaw"] = round(math.atan2(float(r["y"]) - float(l["y"]), float(r["x"]) - float(l["x"])), 6)
        if all_points:
            all_points[-1]["yaw"] = all_points[-2].get("yaw", 0.0) if len(all_points) > 1 else 0.0
        validation = planner.validate_polyline(all_points, require_inflated=True)
    except Exception as exc:
        report = {"artifact_type": "task17c_route_generation_report", "created_utc": now_iso(), "passed": False, "failure_reason": str(exc), "segments": segment_reports}
        write_json(out / "route_generation_report.json", report)
        return None, report
    report = {
        "artifact_type": "task17c_route_generation_report",
        "created_utc": now_iso(),
        "passed": bool(all_points and validation["wall_crossing_validation_passed"]),
        "floor_id": args.floor_id,
        "map_yaml": rel(planner.map_yaml),
        "inflation_radius_m": planner.inflation_radius_m,
        "spacing_m": planner.spacing_m,
        "executable_waypoint_count": len(all_points),
        "route_length_m": round(route_length(all_points), 6),
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "wall_crossing_validation_passed": validation["wall_crossing_validation_passed"],
        "segments": segment_reports,
        "failure_reason": None if validation["wall_crossing_validation_passed"] else "validation failed",
    }
    payload = {
        "artifact_type": "task17c_lightweight_dense_route",
        "created_utc": now_iso(),
        "scene_id": SCENE_ID,
        "floor_id": args.floor_id,
        "query": args.query,
        "room_sequence": semantic["room_sequence"],
        "gateway_sequence": semantic["gateway_sequence"],
        "map_yaml": rel(planner.map_yaml),
        "planner": "inflated_occupancy_grid_astar",
        "inflation_radius_m": planner.inflation_radius_m,
        "spacing_m": planner.spacing_m,
        "waypoints": all_points,
        "route_length_m": report["route_length_m"],
    }
    write_json(out / "generated_lightweight_dense_route.json", payload)
    write_json(out / "route_generation_report.json", report)
    return payload if report["passed"] else None, report


def yaw_proxy(selected: dict[str, Any], snapshot: dict[str, Any]) -> dict[str, Any] | None:
    oid = canonical_object_id(selected["object_id"])
    for anchor in snapshot.get("anchors", []):
        if anchor.get("anchor_type") == "object" and canonical_object_id(anchor.get("target_id")) == oid and anchor.get("valid", True):
            pos = anchor.get("position") or []
            if len(pos) >= 2:
                return {"yaw_proxy_source": "committed_room_world_snapshot_object_anchor", "yaw_proxy_xy": [float(pos[0]), float(pos[1])]}
    pos = selected.get("pose_xy")
    if pos:
        return {"yaw_proxy_source": "committed_object_centroid_proxy_fallback", "yaw_proxy_xy": [float(pos[0]), float(pos[1])]}
    return None


def approach_candidate_fn(args: argparse.Namespace, selected: dict[str, Any], semantic: dict[str, Any], artifacts: dict[str, Any], planner: OccupancyPlanner, out: Path) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    proxy = yaw_proxy(selected, artifacts["committed_room_world_snapshot"])
    if proxy is None:
        write_json(out / "approach_candidate_report.json", {"success": False, "failure_reason": "no proxy"})
        return None, None
    candidate = None
    # Generate approach candidates
    obj_xy = selected.get("pose_xy") or proxy["yaw_proxy_xy"]
    terminal = semantic["waypoints"][-1]
    room = next((r for r in artifacts["topology"].get("rooms", []) if r.get("id") == selected["room_id"]), {})
    generated = []
    for idx, angle_deg in enumerate(range(0, 360, 15)):
        angle = math.radians(angle_deg)
        xy = [float(obj_xy[0]) + 0.8 * math.cos(angle), float(obj_xy[1]) + 0.8 * math.sin(angle)]
        sample = planner.sample(xy)
        if sample["inflated_traversable"] and point_in_polygon(xy[0], xy[1], room.get("polygon", [])):
            generated.append({"candidate_id": f"task17c_ring_{idx:03d}", "world_xy": xy, "distance_to_route_terminal_m": distance(xy, (terminal["x"], terminal["y"]))})
    if generated:
        candidate = min(generated, key=lambda x: x["distance_to_route_terminal_m"])
    if candidate is None:
        write_json(out / "approach_candidate_report.json", {"success": False, "failure_reason": "no valid candidate"})
        return None, proxy
    candidate["yaw"] = round(math.atan2(proxy["yaw_proxy_xy"][1] - candidate["world_xy"][1], proxy["yaw_proxy_xy"][0] - candidate["world_xy"][0]), 6)
    # Validate
    sample = planner.sample(candidate["world_xy"])
    ray_pts = [{"x": candidate["world_xy"][0], "y": candidate["world_xy"][1]}, {"x": proxy["yaw_proxy_xy"][0], "y": proxy["yaw_proxy_xy"][1]}]
    ray_val = planner.validate_polyline(ray_pts, require_inflated=False)
    valid = bool(sample["inflated_traversable"] and ray_val["wall_crossing_validation_passed"])
    report = {
        "artifact_type": "task17c_approach_candidate_report",
        "created_utc": now_iso(),
        "success": valid,
        "selected_candidate_id": candidate.get("candidate_id"),
        "recommended_candidate": candidate,
        "failure_reason": None if valid else "validation failed",
    }
    write_json(out / "approach_candidate_report.json", report)
    return candidate if valid else None, proxy


# ─── ROS velocity execution ─────────────────────────────────────────────────────

def run_velocity_execution(
    params: ControllerParams,
    dense_route: dict[str, Any],
    control_path: list[dict[str, Any]],
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    planner: OccupancyPlanner,
    out: Path,
    adapter: BaseRobotAdapter,
) -> dict[str, Any]:
    try:
        import rclpy
    except Exception as exc:
        return {"success": False, "failure_layer": "pose_feedback", "failure_reason": f"ROS imports unavailable: {exc}"}

    result: dict[str, Any] = {"success": False, "failure_layer": None, "failure_reason": None}
    controller = CurvatureController(params)
    rclpy.init(args=None)
    node = adapter.initialize_ros_node()
    try:
        # Wait for pose
        deadline = time.monotonic() + 10.0
        initial_pose = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            initial_pose = node.pose()
            if initial_pose:
                break
        write_json(out / "pose_source_report.json", {
            "artifact_type": "task17c_pose_source_report", "created_utc": now_iso(),
            "pose_feedback_available": initial_pose is not None, "initial_pose": initial_pose,
        })
        if initial_pose is None:
            result.update({"failure_layer": "pose_feedback", "failure_reason": "no pose received"})
            return result

        # Follow control path
        dense_pts = dense_route.get("waypoints") or []
        room_follow = controller.follow_pure_pursuit(
            node, control_path, "room_route", params.goal_tolerance,
            dense_pts, candidate["world_xy"])
        write_json(out / "path_following_result.json", room_follow)
        if not room_follow["controller_success"]:
            # Partial trajectory validation
            deviation = validate_trajectory_partial(
                controller.trajectory, planner, candidate["world_xy"],
                params.approach_position_tolerance_m, rel(planner.map_yaml), "route_tracking")
            write_json(out / "path_deviation_report.json", deviation)
            result.update({"failure_layer": "route_tracking", "failure_reason": room_follow.get("failure_reason"),
                           "target_room_arrival": False, "room_follow": room_follow,
                           "wall_crossing_validation_passed": deviation.get("wall_crossing_validation_passed"),
                           "trajectory_minimum_clearance_m": deviation.get("trajectory_minimum_clearance_m")})
            return result

        # Settle after room route
        for _ in range(20):
            node.command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        # Approach + Yaw
        approach_result, yaw_result = run_approach_and_yaw(
            node, controller, candidate, proxy, planner, params, dense_pts)
        write_json(out / "approach_execution_result.json", approach_result)

        if not approach_result.get("approach_position_tolerance_reached"):
            deviation = validate_trajectory_partial(
                controller.trajectory, planner, candidate["world_xy"],
                params.approach_position_tolerance_m, rel(planner.map_yaml), "approach_tracking")
            write_json(out / "path_deviation_report.json", deviation)
            result.update({"failure_layer": "approach_tracking",
                           "failure_reason": approach_result.get("failure_reason") or "approach not reached",
                           "target_room_arrival": True, "approach": approach_result, "room_follow": room_follow,
                           "wall_crossing_validation_passed": deviation.get("wall_crossing_validation_passed"),
                           "trajectory_minimum_clearance_m": deviation.get("trajectory_minimum_clearance_m")})
            return result

        write_json(out / "yaw_alignment_result.json", yaw_result)

        # Full trajectory validation
        deviation = validate_trajectory(
            controller.trajectory, planner, candidate["world_xy"],
            params.approach_position_tolerance_m, rel(planner.map_yaml))
        write_json(out / "path_deviation_report.json", deviation)

        # Final success determination
        yaw_facing_success = bool(yaw_result.get("yaw_alignment_success"))
        yaw_drift_success = bool((yaw_result.get("xy_drift_m") or 0.0) <= params.xy_drift_limit_m)
        wall_validation_passed = bool(deviation.get("wall_crossing_validation_passed"))
        approach_reached = bool(approach_result.get("approach_position_tolerance_reached"))

        success = bool(approach_reached and yaw_facing_success and yaw_drift_success and wall_validation_passed)

        failure_layer = None
        failure_reason = None
        if not success:
            if not yaw_facing_success:
                failure_layer = "yaw_alignment"
                failure_reason = yaw_result.get("failure_reason") or "yaw tolerance not reached"
            elif not yaw_drift_success:
                failure_layer = "yaw_alignment"
                failure_reason = "xy drift exceeded during yaw alignment"
            elif not wall_validation_passed:
                failure_layer = "validation"
                failure_reason = "trajectory validation failed"

        result.update({
            "success": success,
            "failure_layer": failure_layer,
            "failure_reason": failure_reason,
            "initial_pose": initial_pose,
            "final_pose": yaw_result.get("final_pose"),
            "room_follow": room_follow,
            "approach": approach_result,
            "yaw": yaw_result,
            "target_room_arrival": True,
            "wall_crossing_validation_passed": wall_validation_passed,
            "yaw_facing_success": yaw_facing_success,
            "yaw_drift_success": yaw_drift_success,
            "trajectory_minimum_clearance_m": deviation.get("trajectory_minimum_clearance_m"),
        })
        return result
    finally:
        try:
            adapter.destroy_ros_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        # Save telemetry artifacts
        save_telemetry_artifacts(out, controller)


# ─── Execute wrapper ─────────────────────────────────────────────────────────────

def execute_runtime(params: ControllerParams, dense_route: dict[str, Any], control_path: list[dict[str, Any]],
                    candidate: dict[str, Any], proxy: dict[str, Any], planner: OccupancyPlanner, out: Path,
                    args: argparse.Namespace) -> dict[str, Any]:
    adapter = create_robot_adapter(
        args.robot_profile_data,
        root=ROOT,
        out=out,
        stage_output_dir=args.stage_output_dir,
        floor_id=args.floor_id,
        map_yaml=planner.map_yaml,
        ros_domain_id=str(args.ros_domain_id),
        gui=args.gui,
        top_level_command=sys.argv,
    )

    adapter.start_bringup()
    readiness = adapter.wait_until_ready()
    ready = bool(readiness.get("gazebo_started_without_nav2"))
    no_nav2 = adapter.validate_no_nav2()

    runtime: dict[str, Any]
    try:
        if not ready:
            runtime = {"success": False, "failure_layer": "no_nav2_bringup", "failure_reason": "bringup failed"}
        else:
            runtime = run_velocity_execution(params, dense_route, control_path, candidate, proxy, planner, out, adapter)
    finally:
        if args.keep_open_sec > 0 and ready:
            time.sleep(args.keep_open_sec)
        adapter.stop_bringup()

    runtime.update({
        "runtime_execution_attempted": ready,
        "gazebo_started_without_nav2": ready,
        "no_nav2_action_servers_active": no_nav2.get("no_nav2_action_servers_active"),
        "robot_profile": adapter.profile.get("robot_name"),
        "robot_profile_path": rel(args.robot_profile),
    })

    # Trajectory metrics
    traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
    if traj:
        runtime["trajectory_length_m"] = round(route_length(traj), 6)
        runtime["runtime_duration_sec"] = traj[-1].get("t_sec")

    write_json(out / "runtime_result.json", runtime)
    return runtime


# ─── Main ────────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id")
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--map-yaml", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--plan-only", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--keep-open-sec", type=float, default=0.0)
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "84"))
    parser.add_argument("--robot-profile", type=Path, default=DEFAULT_ROBOT_PROFILE)
    parser.add_argument("--pose-source", choices=["tf", "odom", "gazebo"], default="tf")
    # Controller params (defaults match task17e)
    parser.add_argument("--max-linear-speed", type=float, default=None)
    parser.add_argument("--max-angular-speed", type=float, default=None)
    parser.add_argument("--lookahead-distance", type=float, default=None)
    parser.add_argument("--lookahead-min", type=float, default=None)
    parser.add_argument("--lookahead-max", type=float, default=None)
    parser.add_argument("--angular-smoothing-alpha", type=float, default=None)
    parser.add_argument("--angular-rate-limit", type=float, default=None)
    parser.add_argument("--heading-kp", type=float, default=0.80)
    parser.add_argument("--goal-tolerance", type=float, default=0.30)
    parser.add_argument("--approach-position-tolerance-m", type=float, default=0.35)
    parser.add_argument("--approach-controller-stop-tolerance-m", type=float, default=0.30)
    parser.add_argument("--approach-creep-max-attempts", type=int, default=5)
    parser.add_argument("--approach-creep-step-m", type=float, default=0.04)
    parser.add_argument("--approach-settle-sec", type=float, default=None)
    parser.add_argument("--approach-creep-boundary-upper-m", type=float, default=0.38)
    parser.add_argument("--yaw-internal-tolerance-rad", type=float, default=0.38)
    parser.add_argument("--yaw-report-tolerance-rad", type=float, default=0.50)
    parser.add_argument("--xy-drift-limit-m", type=float, default=None)
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--path-deviation-limit-m", type=float, default=0.75)
    parser.add_argument("--inflation-radius-m", type=float, default=None)
    parser.add_argument("--path-spacing-m", type=float, default=0.20)
    parser.add_argument("--control-rate-hz", type=float, default=None)
    parser.add_argument("--yaw-kp", type=float, default=1.0)
    parser.add_argument("--max-yaw-angular-speed", type=float, default=0.25)
    parser.add_argument("--yaw-timeout-sec", type=float, default=30.0)
    parser.add_argument("--control-path-spacing", type=float, default=0.60)
    parser.add_argument("--control-path-gateway-spacing", type=float, default=0.30)
    parser.add_argument("--run-id", type=str, default=None)
    args = parser.parse_args()
    args.stage_output_dir = args.stage_output_dir.resolve()
    args.output_dir = args.output_dir.resolve()
    if args.map_yaml:
        args.map_yaml = args.map_yaml.resolve()
    args.robot_profile = args.robot_profile.resolve()
    args.robot_profile_data = load_robot_profile(args.robot_profile)
    apply_robot_profile_defaults(args, args.robot_profile_data)
    if not args.object_id:
        args.object_id = None

    run_id = args.run_id or "runtime_lightweight_executor"
    out = args.output_dir / run_id
    out.mkdir(parents=True, exist_ok=True)

    # Build controller params
    params = ControllerParams(
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        lookahead_base=args.lookahead_distance,
        lookahead_min=args.lookahead_min,
        lookahead_max=args.lookahead_max,
        heading_kp=args.heading_kp,
        goal_tolerance=args.goal_tolerance,
        approach_position_tolerance_m=args.approach_position_tolerance_m,
        approach_controller_stop_tolerance_m=args.approach_controller_stop_tolerance_m,
        approach_creep_max_attempts=args.approach_creep_max_attempts,
        approach_creep_step_m=args.approach_creep_step_m,
        approach_settle_sec=args.approach_settle_sec,
        approach_creep_boundary_upper_m=args.approach_creep_boundary_upper_m,
        yaw_kp=args.yaw_kp,
        max_yaw_angular_speed=args.max_yaw_angular_speed,
        yaw_internal_tolerance_rad=args.yaw_internal_tolerance_rad,
        yaw_report_tolerance_rad=args.yaw_report_tolerance_rad,
        yaw_timeout_sec=args.yaw_timeout_sec,
        xy_drift_limit_m=args.xy_drift_limit_m,
        timeout_sec=args.timeout_sec,
        path_deviation_limit_m=args.path_deviation_limit_m,
        inflation_radius_m=args.inflation_radius_m,
        path_spacing_m=args.path_spacing_m,
        control_rate_hz=args.control_rate_hz,
        control_path_spacing=args.control_path_spacing,
        control_path_gateway_spacing=args.control_path_gateway_spacing,
        angular_smoothing_alpha=args.angular_smoothing_alpha,
        angular_rate_limit=args.angular_rate_limit,
    )
    write_json(out / "robot_profile_report.json", {
        "artifact_type": "task20_robot_profile_report",
        "created_utc": now_iso(),
        "robot_profile_path": rel(args.robot_profile),
        "robot_profile": args.robot_profile_data,
        "adapter": adapter_type_for_profile(args.robot_profile_data),
        "nav2_used": False,
        "quadruped_support_claimed": False,
        "quadruped_visual_kinematic_proxy_claimed": (
            args.robot_profile_data.get("claim_boundary") == "visual_kinematic_proxy_only"
        ),
    })
    controller_report = params.to_json()
    controller_report["cmd_vel_topic"] = args.robot_profile_data.get("cmd_topic")
    controller_report["robot_profile"] = args.robot_profile_data.get("robot_name")
    controller_report["robot_profile_path"] = rel(args.robot_profile)
    write_json(out / "controller_params.json", controller_report)

    # Load artifacts
    artifacts, failure = load_artifacts(args, out)
    runtime: dict[str, Any] = {"success": False, "failure_layer": None, "failure_reason": None}
    if failure:
        runtime.update({"failure_layer": "artifact_loading", "failure_reason": failure})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Query resolution
    selected, resolution = resolve_query(args, out)
    if selected is None:
        runtime.update({"failure_layer": "query_resolution", "failure_reason": resolution["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1
    args.object_id = selected["object_id"]

    # Topology route
    topology = generate_topology_route(args, selected, artifacts, out)
    if not topology["route_generated"]:
        runtime.update({"failure_layer": "topology_route_generation", "failure_reason": topology["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Semantic anchors
    semantic = semantic_anchors_fn(args, topology, artifacts, out)

    # Dense A* route
    planner = OccupancyPlanner(stable_map_paths(args)[1], params.inflation_radius_m, params.path_spacing_m)
    dense_route, route_report = executable_route(args, semantic, planner, out)
    if dense_route is None:
        runtime.update({"failure_layer": "executable_route_generation", "failure_reason": route_report["failure_reason"]})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Control path building (simplification + rounding + fallback)
    cp_result = build_control_path(
        dense_route["waypoints"], planner, semantic.get("waypoints") or [], params)

    write_json(out / "control_path_simplification_report.json", cp_result.simplification_report)

    if not cp_result.simplification_report["wall_crossing_validation_passed"]:
        runtime.update({
            "failure_layer": cp_result.simplification_report.get("failure_layer"),
            "failure_reason": cp_result.simplification_report.get("failure_reason"),
        })
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Enrich rounding report
    cp_result.rounding_report["dense_route_length_m"] = dense_route.get("route_length_m")
    cp_result.rounding_report["simplified_control_path_length_m"] = cp_result.simplification_report.get("control_path_length_m")
    cp_result.rounding_report["dense_route_minimum_clearance_m"] = read_json(out / "route_generation_report.json", {}).get("minimum_clearance_m")
    cp_result.rounding_report["simplified_path_minimum_clearance_m"] = cp_result.simplification_report.get("minimum_clearance_m")
    write_json(out / "control_path_rounding_report.json", cp_result.rounding_report)

    control_path = cp_result.control_path
    write_json(out / "generated_lightweight_control_path.json", {
        "artifact_type": "task17c_lightweight_control_path",
        "created_utc": now_iso(),
        "scene_id": SCENE_ID, "floor_id": args.floor_id, "query": args.query,
        "map_yaml": rel(planner.map_yaml),
        "waypoints": control_path,
        "control_path_length_m": round(route_length(control_path), 6),
        "dense_route_length_m": dense_route["route_length_m"],
        "simplified_control_path_length_m": cp_result.simplification_report.get("control_path_length_m"),
        "rounding_attempted": cp_result.rounding_attempted,
        "rounding_applied": cp_result.rounding_applied,
        "fallback_to_original": cp_result.fallback_to_original,
        "final_control_path_source": cp_result.final_control_path_source,
        "rounded_candidate_length_m": cp_result.rounding_report.get("rounded_control_path_length_m"),
        "final_control_path_length_m": round(route_length(control_path), 6),
        "rounded_candidate_validation_passed": cp_result.rounded_candidate_validation_passed,
        "fallback_reason": cp_result.fallback_reason,
        "dense_route_generation_passed": cp_result.simplification_report.get("dense_route_generation_passed"),
        "simplified_path_validation_passed": cp_result.simplification_report.get("simplified_path_validation_passed"),
        "simplified_invalid_segment_count": cp_result.simplification_report.get("simplified_invalid_segment_count"),
        "dense_fallback_attempted": cp_result.simplification_report.get("dense_fallback_attempted"),
        "dense_fallback_validation_passed": cp_result.simplification_report.get("dense_fallback_validation_passed"),
        "fallback_to_dense_or_resampled": cp_result.simplification_report.get("fallback_to_dense_or_resampled"),
        "final_control_path_minimum_clearance_m": cp_result.simplification_report.get("final_control_path_minimum_clearance_m"),
        "wall_crossing_validation_passed": cp_result.simplification_report.get("wall_crossing_validation_passed"),
        "endpoint_consistency_passed": cp_result.simplification_report.get("endpoint_consistency_passed"),
    })

    # Approach candidate
    candidate, proxy_dict = approach_candidate_fn(args, selected, semantic, artifacts, planner, out)
    if candidate is None or proxy_dict is None:
        runtime.update({"failure_layer": "approach_candidate_generation", "failure_reason": "no valid approach candidate"})
        write_json(out / "runtime_result.json", runtime)
        return 1

    # Visualization label
    control_path_viz_label = cp_result.final_control_path_source.replace("_", " ")

    # Pre-execution visualization
    plot_visualization(planner, dense_route, control_path, semantic, candidate, [], proxy_dict,
                       out / "route_visualization_planned.png",
                       simplified_path=cp_result.simplified_path, control_path_label=control_path_viz_label)

    if args.execute:
        runtime = execute_runtime(params, dense_route, control_path, candidate, proxy_dict, planner, out, args)

        # Enrich runtime result
        approach_r = runtime.get("approach") or {}
        yaw_r = runtime.get("yaw") or {}
        room_r = runtime.get("room_follow") or {}
        runtime.update({
            "query": args.query, "object_id": selected["object_id"],
            "target_room": selected["room_id"], "floor_id": args.floor_id,
            "approach_position_reached": bool(approach_r.get("approach_position_tolerance_reached")),
            "approach_yaw_aligned": bool(yaw_r.get("yaw_alignment_success")),
            "object_facing_approach_success": bool(runtime.get("success")),
            "final_distance_to_target_room_terminal_m": room_r.get("final_distance_m"),
            "final_distance_to_approach_candidate_m": approach_r.get("final_distance_to_approach_candidate_m"),
            "final_yaw_error_rad": yaw_r.get("final_yaw_error_rad"),
            "final_control_path_source": cp_result.final_control_path_source,
            "rounding_attempted": cp_result.rounding_attempted,
            "rounding_applied": cp_result.rounding_applied,
            "fallback_to_original": cp_result.fallback_to_original,
            "rounded_candidate_validation_passed": cp_result.rounded_candidate_validation_passed,
            "fallback_reason": cp_result.fallback_reason,
            "dense_fallback_attempted": cp_result.simplification_report.get("dense_fallback_attempted"),
            "dense_fallback_validation_passed": cp_result.simplification_report.get("dense_fallback_validation_passed"),
            "fallback_to_dense_or_resampled": cp_result.simplification_report.get("fallback_to_dense_or_resampled"),
            "simplified_path_validation_passed": cp_result.simplification_report.get("simplified_path_validation_passed"),
            "simplified_invalid_segment_count": cp_result.simplification_report.get("simplified_invalid_segment_count"),
            "final_control_path_minimum_clearance_m": cp_result.simplification_report.get("final_control_path_minimum_clearance_m"),
        })
        traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
        commands = read_json(out / "cmd_vel_log.json", {}).get("commands") or []
        tracking_quality = generate_tracking_quality_report(
            traj, control_path, commands, params.max_angular_speed)
        write_json(out / "tracking_quality_report.json", tracking_quality)
        plot_tracking_error(tracking_quality, out / "route_following_tracking_error.png")
        runtime["tracking_quality_metrics"] = {
            key: value for key, value in tracking_quality.items()
            if key not in {"tracking_error_samples", "threshold_checks", "thresholds", "artifact_type"}
        }
        runtime["tracking_quality_passed"] = tracking_quality["tracking_quality_passed"]
        write_json(out / "runtime_result.json", runtime)

        # Post-execution visualization
        traj = read_json(out / "executed_trajectory.json", {}).get("samples") or []
        plot_visualization(planner, dense_route, control_path, semantic, candidate, traj, proxy_dict,
                           out / "route_visualization.png",
                           simplified_path=cp_result.simplified_path, control_path_label=control_path_viz_label)

        # Approach yaw local zoom
        yaw_r_file = read_json(out / "yaw_alignment_result.json", {})
        plot_approach_yaw_local_zoom(planner, candidate, proxy_dict, traj, yaw_r_file, out / "approach_yaw_local_zoom.png")

        # Approach yaw validation report
        ayv = generate_approach_yaw_validation_report(candidate, proxy_dict, params, runtime, approach_r, yaw_r)
        write_json(out / "approach_yaw_validation_report.json", ayv)
    else:
        runtime = {"success": False, "plan_only": True, "runtime_execution_attempted": False,
                   "final_control_path_source": cp_result.final_control_path_source,
                   "rounding_attempted": cp_result.rounding_attempted,
                   "rounding_applied": cp_result.rounding_applied,
                   "fallback_to_original": cp_result.fallback_to_original,
                   "rounded_candidate_validation_passed": cp_result.rounded_candidate_validation_passed,
                   "fallback_reason": cp_result.fallback_reason}
        write_json(out / "runtime_result.json", runtime)

    # Summary
    summary = generate_summary(args.query, args.object_id, runtime)
    write_json(out / "summary.json", summary)
    return 0 if args.plan_only or runtime.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
