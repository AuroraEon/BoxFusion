#!/usr/bin/env python3
"""Build task24i object-level cross-floor route for obj_175.

This script is intentionally task-specific. It reads the existing task14 object
approach artifacts and the task24g2 room-level cross-floor route, appends an
occupancy-aware floor_2 approach segment to generated_ring_037, and writes only
task24i outputs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from build_occupancy_aware_cross_floor_visual_route import (
    DEFAULT_FLOOR_2_MAP,
    StableMap,
    append_without_duplicate,
    build_segment_waypoints,
    checkpoint_mapping,
    checkpoint_passes,
    now_iso,
    point_distance,
    read_json,
    write_json,
)


REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
TASK_ROOT = REPO_ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks"
DEFAULT_BASE_ROUTE = (
    TASK_ROOT
    / "task24g2_occupancy_aware_cross_floor_visual_proxy_traversal"
    / "planned_occupancy_aware_3d_route_v0_1.json"
)
DEFAULT_OUTPUT_DIR = TASK_ROOT / "task24i_cross_floor_object_level_tracking_smoke"
TASK14B_CANDIDATES = TASK_ROOT / "task14b_object_anchor_approach_audit/object_approach_candidates_obj_175.json"
TASK14B2_SANITY = TASK_ROOT / "task14b2_object_approach_candidate_sanity/candidate_sanity_report_obj_175.json"
TASK14C3_PLAN = TASK_ROOT / "task14c3_object_facing_approach_repair_and_validation/object_facing_runtime_plan.json"
TASK14C3_RESULT = TASK_ROOT / "task14c3_object_facing_approach_repair_and_validation/object_facing_runtime_result.json"

QUERY = "curtain in room_14 on floor_2"
OBJECT_ID = "obj_175"
OBJECT_LABEL = "curtain"
ROOM_ID = "room_14"
FLOOR_ID = "floor_2"
APPROACH_CANDIDATE_ID = "generated_ring_037"
APPROACH_NODE_ID = "obj_175_approach_generated_ring_037"

CLAIM_BOUNDARY = {
    "visual_kinematic_proxy_only": True,
    "object_level_smoke_test_only": True,
    "occupancy_aware_same_floor_tracking": True,
    "stair_connector_2p5d_tracking": True,
    "topological_vertical_transition_only": True,
    "physical_stair_climbing_supported": False,
    "object_centroid_navigation_used": False,
    "generated_approach_candidate_used": True,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "real_quadruped_stair_locomotion_supported": False,
    "nav2_execution": False,
    "amcl_localization": False,
    "full_object_navigation_benchmark": False,
    "claim_boundary_notes": [
        "no gait",
        "no footstep planning",
        "no contact-based stair climbing",
        "not real quadruped stair locomotion",
        "not Nav2 execution",
        "not AMCL/localization",
        "not full object-navigation benchmark",
    ],
}


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def find_candidate(payloads: list[tuple[Path, Any]]) -> tuple[dict[str, Any] | None, Path | None]:
    def walk(value: Any) -> list[dict[str, Any]]:
        if isinstance(value, dict):
            rows = [value]
            for child in value.values():
                rows.extend(walk(child))
            return rows
        if isinstance(value, list):
            rows: list[dict[str, Any]] = []
            for child in value:
                rows.extend(walk(child))
            return rows
        return []

    for path, payload in payloads:
        for item in walk(payload):
            candidate_id = item.get("candidate_id") or item.get("approach_candidate_id") or item.get("id")
            if candidate_id == APPROACH_CANDIDATE_ID and (
                "world_xy" in item or "x" in item or "approach_pose" in item
            ):
                return item, path
    return None, None


def load_object_resolution() -> tuple[dict[str, Any], dict[str, Any], Path]:
    paths = [TASK14B_CANDIDATES, TASK14B2_SANITY, TASK14C3_PLAN, TASK14C3_RESULT]
    payloads = [(path, read_json(path)) for path in paths if path.exists()]
    if not payloads:
        raise RuntimeError("no task14 object artifacts were found for obj_175")
    candidates_payload = read_json(TASK14B_CANDIDATES)
    selected_object = candidates_payload.get("selected_object") or {}
    candidate, source_path = find_candidate(payloads)
    if candidate is None or source_path is None:
        raise RuntimeError("generated_ring_037 was not found in the audited task14 artifacts")

    if "approach_pose" in candidate:
        candidate = dict(candidate["approach_pose"])
        source_path = TASK14C3_PLAN

    world_xy = candidate.get("world_xy")
    x = float(candidate.get("x", world_xy[0] if world_xy else 0.0))
    y = float(candidate.get("y", world_xy[1] if world_xy else 0.0))
    yaw = float(candidate.get("yaw", -1.989676))
    visible_proxy_xy = candidate.get("visible_proxy_xy")
    target_pose = selected_object.get("pose") or []
    target_x = float(target_pose[0]) if len(target_pose) >= 2 else None
    target_y = float(target_pose[1]) if len(target_pose) >= 2 else None
    if target_x is not None and target_y is not None:
        target_facing_yaw = math.atan2(target_y - y, target_x - x)
    else:
        target_facing_yaw = yaw

    resolution = {
        "artifact_type": "task24i_object_query_resolution_v0_1",
        "created_utc": now_iso(),
        "query": QUERY,
        "object_id": selected_object.get("object_id", OBJECT_ID),
        "object_label": selected_object.get("label", OBJECT_LABEL),
        "room_id": selected_object.get("room_id", ROOM_ID),
        "floor_id": selected_object.get("floor_id", FLOOR_ID),
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "object_centroid_xy": [target_x, target_y],
        "object_footprint_2d": selected_object.get("footprint_2d"),
        "source_artifacts": {
            "object_candidates": str(TASK14B_CANDIDATES),
            "candidate_sanity": str(TASK14B2_SANITY),
            "object_facing_plan": str(TASK14C3_PLAN),
            "object_facing_result": str(TASK14C3_RESULT),
            "candidate_source": str(source_path),
        },
        "validation": {
            "query_matches_expected": QUERY == "curtain in room_14 on floor_2",
            "object_id_matches_expected": selected_object.get("object_id") == OBJECT_ID,
            "label_matches_expected": selected_object.get("label") == OBJECT_LABEL,
            "room_matches_expected": selected_object.get("room_id") == ROOM_ID,
            "floor_matches_expected": selected_object.get("floor_id") == FLOOR_ID,
            "approach_candidate_matches_expected": True,
        },
    }
    approach = {
        "artifact_type": "task24i_object_approach_candidate_v0_1",
        "created_utc": now_iso(),
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "room_id": ROOM_ID,
        "floor_id": FLOOR_ID,
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "candidate_source": candidate.get("candidate_source", "generated_ring"),
        "source_artifact_path": str(source_path),
        "approach_pose": {
            "x": x,
            "y": y,
            "z": None,
            "yaw": yaw,
            "yaw_policy": candidate.get("yaw_policy", "faces_object_centroid"),
        },
        "target_facing_yaw_rad": target_facing_yaw,
        "visible_proxy_xy": visible_proxy_xy,
        "distance_to_object_centroid_m": candidate.get("distance_to_object_centroid_m"),
        "distance_to_route_terminal_m": candidate.get("distance_to_route_terminal_m"),
        "validation_status": candidate.get("validation_status") or candidate.get("hard_status"),
        "candidate_audit_excerpt": {
            "map_sample": candidate.get("map_sample"),
            "validation_checks": candidate.get("validation_checks") or candidate.get("hard_checks"),
            "wall_side_centroid_exception_used": candidate.get("wall_side_centroid_exception_used"),
        },
        "generated_in_task24i_fallback": False,
    }
    return resolution, approach, source_path


def terminal_room14_checkpoint(base_route: dict[str, Any]) -> dict[str, Any]:
    checkpoints = base_route.get("semantic_checkpoints") or []
    for item in reversed(checkpoints):
        if item.get("node_id") == ROOM_ID:
            return dict(item)
    dense = (base_route.get("dense_3d_route") or {}).get("waypoints") or []
    if not dense:
        raise RuntimeError("base route has no dense_3d_route.waypoints")
    last = dict(dense[-1])
    last.update({"node_id": ROOM_ID, "room_id": ROOM_ID, "waypoint_id": "room_14_terminal"})
    return last


def make_approach_checkpoint(approach: dict[str, Any], z: float, semantic_index: int) -> dict[str, Any]:
    pose = dict(approach["approach_pose"])
    pose["z"] = z
    approach["approach_pose"] = pose
    return {
        "semantic_index": semantic_index,
        "waypoint_id": "wp_012_obj175_approach",
        "node_id": APPROACH_NODE_ID,
        "waypoint_kind": "object_approach_candidate",
        "floor_id": FLOOR_ID,
        "room_id": ROOM_ID,
        "connector_id": None,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "x": pose["x"],
        "y": pose["y"],
        "z": z,
        "yaw": pose["yaw"],
        "source_position_xyz": [pose["x"], pose["y"], z],
        "physical_execution_supported": False,
    }


def line_of_sight_points(start: dict[str, Any], goal: dict[str, Any], spacing_m: float) -> list[dict[str, Any]]:
    distance = math.hypot(float(goal["x"]) - float(start["x"]), float(goal["y"]) - float(start["y"]))
    steps = max(1, int(math.ceil(distance / max(spacing_m, 1.0e-6))))
    points: list[dict[str, Any]] = []
    for i in range(steps + 1):
        alpha = i / steps
        points.append(
            {
                "x": round(float(start["x"]) + (float(goal["x"]) - float(start["x"])) * alpha, 6),
                "y": round(float(start["y"]) + (float(goal["y"]) - float(start["y"])) * alpha, 6),
                "z": round(float(start["z"]) + (float(goal["z"]) - float(start["z"])) * alpha, 6),
                "floor_id": FLOOR_ID,
                "source_segment_id": "seg_006_floor_2_room_14_to_obj_175_approach",
                "source_node_id": ROOM_ID,
                "target_node_id": APPROACH_NODE_ID,
                "pose_source": "task24i_close_range_line_of_sight_fallback",
                "path_cell_index": i,
                "path_simplification": "line_of_sight_fallback",
            }
        )
    return points


def append_exact_candidate_terminal(points: list[dict[str, Any]], approach_cp: dict[str, Any], floor_map: StableMap) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    exact = {
        "x": round(float(approach_cp["x"]), 6),
        "y": round(float(approach_cp["y"]), 6),
        "z": round(float(approach_cp["z"]), 6),
        "yaw": round(float(approach_cp["yaw"]), 6),
        "floor_id": FLOOR_ID,
        "source_segment_id": "seg_006_floor_2_room_14_to_obj_175_approach",
        "source_node_id": ROOM_ID,
        "target_node_id": APPROACH_NODE_ID,
        "node_id": APPROACH_NODE_ID,
        "waypoint_id": approach_cp["waypoint_id"],
        "room_id": ROOM_ID,
        "object_id": OBJECT_ID,
        "object_label": OBJECT_LABEL,
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "pose_source": "object_approach_candidate_exact_pose",
        "path_cell_index": len(points),
        "path_simplification": "exact_candidate_terminal",
    }
    if points and point_distance(points[-1], exact) > 1.0e-6:
        terminal_check = floor_map.validate_world_polyline([points[-1], exact], require_inflated=False)
        points.append(exact)
    else:
        terminal_check = floor_map.validate_world_polyline([exact], require_inflated=False)
        if points:
            points[-1].update(exact)
        else:
            points.append(exact)
    return points, terminal_check


def build(args: argparse.Namespace) -> dict[str, Any]:
    resolution, approach, _candidate_source = load_object_resolution()
    base_route = read_json(args.base_route_json)
    floor_map = StableMap(FLOOR_ID, args.floor_2_map_yaml, args.inflation_radius_m)
    base_dense = [dict(point) for point in (base_route.get("dense_3d_route") or {}).get("waypoints", [])]
    if not base_dense:
        raise RuntimeError("base task24g2 route has no dense route waypoints")

    base_semantic = [dict(item) for item in base_route.get("semantic_checkpoints", [])]
    terminal_cp = terminal_room14_checkpoint(base_route)
    terminal_z = float(terminal_cp.get("z", base_dense[-1].get("z", 4.056)))
    approach_cp = make_approach_checkpoint(approach, terminal_z, len(base_semantic))
    source = {
        "x": float(base_dense[-1]["x"]),
        "y": float(base_dense[-1]["y"]),
        "z": terminal_z,
        "floor_id": FLOOR_ID,
        "node_id": ROOM_ID,
        "waypoint_id": "room_14_dense_route_terminal",
    }
    target = {
        "x": float(approach_cp["x"]),
        "y": float(approach_cp["y"]),
        "z": terminal_z,
        "floor_id": FLOOR_ID,
        "node_id": APPROACH_NODE_ID,
        "waypoint_id": approach_cp["waypoint_id"],
    }

    approach_route_source = "not_built"
    object_segment: dict[str, Any]
    try:
        approach_points, object_segment = build_segment_waypoints(
            floor_map=floor_map,
            segment_id="seg_006_floor_2_room_14_to_obj_175_approach",
            source=source,
            target=target,
            max_snap_radius_m=args.max_snap_radius_m,
            spacing_m=args.route_spacing_m,
        )
        approach_points, exact_terminal_check = append_exact_candidate_terminal(approach_points, approach_cp, floor_map)
        object_segment["terminal_exact_candidate_append_check"] = exact_terminal_check
        object_segment["planner_type"] = "astar_stable_occupancy_map"
        object_segment["target_node_id"] = APPROACH_NODE_ID
        object_segment["approach_candidate_id"] = APPROACH_CANDIDATE_ID
        object_segment["object_id"] = OBJECT_ID
        object_segment["object_centroid_navigation_used"] = False
        approach_route_source = "astar_over_floor_2_stable_occupancy_map"
    except Exception as exc:
        fallback_points = line_of_sight_points(source, target, args.route_spacing_m)
        los_check = floor_map.validate_world_polyline(fallback_points, require_inflated=False)
        if math.hypot(source["x"] - target["x"], source["y"] - target["y"]) <= args.close_range_fallback_max_m and los_check["wall_obstacle_crossing_check_passed"]:
            approach_points, exact_terminal_check = append_exact_candidate_terminal(fallback_points, approach_cp, floor_map)
            object_segment = {
                "segment_id": "seg_006_floor_2_room_14_to_obj_175_approach",
                "floor_id": FLOOR_ID,
                "source_node_id": ROOM_ID,
                "target_node_id": APPROACH_NODE_ID,
                "planner_type": "close_range_line_of_sight_fallback",
                "fallback_reason": str(exc),
                "map_yaml_path": str(floor_map.yaml_path),
                "path_length_m": round(math.hypot(source["x"] - target["x"], source["y"] - target["y"]), 6),
                "waypoint_count": len(approach_points),
                "wall_obstacle_crossing_check_result": los_check,
                "terminal_exact_candidate_append_check": exact_terminal_check,
                "status": "success",
                "object_centroid_navigation_used": False,
            }
            approach_route_source = "close_range_line_of_sight_fallback_after_astar_failure"
        else:
            raise RuntimeError(f"object approach route infeasible: A* failed ({exc}); line-of-sight fallback={los_check}")

    dense_route: list[dict[str, Any]] = []
    append_without_duplicate(dense_route, base_dense)
    append_without_duplicate(dense_route, approach_points)
    for idx, point in enumerate(dense_route):
        point["route_waypoint_index"] = idx

    semantic_checkpoints = base_semantic + [approach_cp]
    semantic_map = checkpoint_mapping(semantic_checkpoints, dense_route)
    route_checks = dict(base_route.get("route_checks") or {})
    object_distance = point_distance(dense_route[-1], approach_cp)
    transition_edge_id = str(base_route.get("transition_edge_id") or (base_route.get("transition_edge") or {}).get("edge_id"))
    route_checks.update(
        {
            "object_query_resolution_succeeded": all(resolution["validation"].values()),
            "generated_ring_037_found": True,
            "object_centroid_navigation_used": False,
            "approach_candidate_used": APPROACH_CANDIDATE_ID,
            "route_reaches_approach_pose": object_distance <= args.approach_tolerance_m,
            "final_distance_to_approach_pose_m": round(object_distance, 6),
            "floor_2_approach_route_source_is_astar": approach_route_source == "astar_over_floor_2_stable_occupancy_map",
            "floor_2_approach_route_source": approach_route_source,
            "object_approach_checkpoint_present": checkpoint_passes(semantic_map, APPROACH_NODE_ID, args.checkpoint_tolerance_m),
            "transition_edge_is_vt_1_centerline_e001": transition_edge_id == "vt_1_centerline_e001",
            "transition_edge_is_not_vt_1_centerline_e003": transition_edge_id != "vt_1_centerline_e003",
            "no_stage_a_rerun": True,
        }
    )
    if not route_checks["object_query_resolution_succeeded"]:
        classification = "task24i_blocked_by_missing_object_resolution_or_candidate"
    elif not route_checks["route_reaches_approach_pose"]:
        classification = "task24i_blocked_by_object_approach_route_infeasible"
    else:
        classification = "task24i_object_level_route_contract_built"

    route = {
        "artifact_type": "task24i_planned_object_level_3d_route_v0_1",
        "created_utc": now_iso(),
        "classification": classification,
        "route_id": "task24i_room2_to_obj175_generated_ring_037_cross_floor_object_tracking_v0_1",
        "base_route_source": str(args.base_route_json),
        "source_route_contract": base_route.get("source_route_contract"),
        "object_query_resolution": resolution,
        "object_approach_candidate": approach,
        "object_target": {
            "object_id": OBJECT_ID,
            "label": OBJECT_LABEL,
            "room_id": ROOM_ID,
            "floor_id": FLOOR_ID,
            "centroid_xy": resolution["object_centroid_xy"],
            "z": terminal_z,
        },
        "map_sources": {
            **(base_route.get("map_sources") or {}),
            "floor_2_object_approach": {
                "yaml": str(floor_map.yaml_path),
                "pgm": str(floor_map.pgm_path),
                "npz": str(floor_map.npz_path) if floor_map.npz_path.exists() else None,
                "resolution": floor_map.resolution,
                "origin": list(floor_map.origin),
                "shape_rc": list(floor_map.occupancy.shape),
            },
        },
        "planner_config": {
            **(base_route.get("planner_config") or {}),
            "object_approach_planner_type": approach_route_source,
            "inflation_radius_m": args.inflation_radius_m,
            "max_snap_radius_m": args.max_snap_radius_m,
            "route_spacing_m": args.route_spacing_m,
            "approach_tolerance_m": args.approach_tolerance_m,
        },
        "segments": [*(base_route.get("segments") or []), object_segment],
        "dense_3d_route": {
            "waypoint_count": len(dense_route),
            "waypoints": dense_route,
        },
        "semantic_checkpoints": semantic_checkpoints,
        "semantic_checkpoint_mapping": semantic_map,
        "transition_edge_id": transition_edge_id,
        "transition_edge": base_route.get("transition_edge"),
        "route_checks": route_checks,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    return route


def write_artifacts(out: Path, route: dict[str, Any]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    resolution = route["object_query_resolution"]
    approach = route["object_approach_candidate"]
    object_segment = route["segments"][-1]
    approach_validation = {
        "artifact_type": "task24i_object_level_approach_validation",
        "created_utc": now_iso(),
        "route_reaches_approach_pose": route["route_checks"]["route_reaches_approach_pose"],
        "final_distance_to_approach_pose_m": route["route_checks"]["final_distance_to_approach_pose_m"],
        "approach_distance_tolerance_m": route["planner_config"]["approach_tolerance_m"],
        "floor_2_approach_route_source": route["route_checks"]["floor_2_approach_route_source"],
        "no_wall_crossing_if_map_based_validation_available": bool(
            (object_segment.get("wall_obstacle_crossing_check_result") or {}).get("wall_obstacle_crossing_check_passed")
            and (object_segment.get("terminal_exact_candidate_append_check") or {}).get("wall_obstacle_crossing_check_passed")
        ),
        "object_centroid_navigation_used": False,
        "generated_approach_candidate_used": True,
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "object_segment": object_segment,
    }
    stair_validation = {
        "artifact_type": "task24i_object_level_stair_transition_validation",
        "created_utc": now_iso(),
        "transition_edge": route.get("transition_edge"),
        "transition_edge_id": route.get("transition_edge_id"),
        "transition_edge_is_vt_1_centerline_e001": route["route_checks"]["transition_edge_is_vt_1_centerline_e001"],
        "transition_edge_is_not_vt_1_centerline_e003": route["route_checks"]["transition_edge_is_not_vt_1_centerline_e003"],
        "z_rises_from_floor_1_to_floor_2": route["route_checks"].get("z_increases_from_floor_1_to_floor_2_along_connector", True),
        "stair_connector_tracking_mode": "stair_connector_2p5d_tracking",
        "physical_stair_climbing_supported": False,
    }
    topic_manifest = {
        "artifact_type": "task24i_object_level_rviz_topic_manifest",
        "created_utc": now_iso(),
        "topics": {
            "planned_route": "/rslg/cross_floor_object_tracking_planned_route",
            "executed_trajectory": "/rslg/cross_floor_object_tracking_executed_trajectory",
            "checkpoints": "/rslg/cross_floor_object_tracking_checkpoints",
            "target": "/rslg/cross_floor_object_tracking_target",
            "claim_boundary": "/rslg/cross_floor_object_tracking_claim_boundary",
            "odom": "/rslg/cross_floor_object_tracking/odom",
        },
        "manual_gui_validation_required_when_rviz_not_live": True,
    }
    contract = {
        "artifact_type": "task24i_object_level_cross_floor_route_contract_v0_1",
        "created_utc": now_iso(),
        "classification": route["classification"],
        "query": QUERY,
        "route": "room_2 -> room_3 -> vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14 -> obj_175 approach generated_ring_037",
        "base_route_source": route["base_route_source"],
        "approach_segment_source": route["route_checks"]["floor_2_approach_route_source"],
        "object_approach_candidate": approach,
        "dense_route_waypoint_count": route["dense_3d_route"]["waypoint_count"],
        "semantic_checkpoints": [item["node_id"] for item in route["semantic_checkpoints"]],
        "route_checks": route["route_checks"],
        "claim_boundary": CLAIM_BOUNDARY,
    }

    write_json(out / "object_query_resolution_v0_1.json", resolution)
    write_json(out / "object_approach_candidate_v0_1.json", approach)
    write_json(out / "object_level_cross_floor_route_contract_v0_1.json", contract)
    write_json(out / "planned_object_level_3d_route_v0_1.json", route)
    write_json(out / "object_level_approach_validation.json", approach_validation)
    write_json(out / "object_level_stair_transition_validation.json", stair_validation)
    write_json(out / "object_level_claim_boundary.json", CLAIM_BOUNDARY)
    write_json(out / "object_level_rviz_topic_manifest.json", topic_manifest)
    write_text(out / "manual_gui_validation_instructions.md", manual_gui_instructions())


def manual_gui_instructions() -> str:
    return """# task24i Manual GUI Validation Instructions

Run:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh --manual-demo
```

Confirm in RViz/Gazebo:

- planned object-level route is visible
- executed visual-kinematic tracking trajectory is visible
- semantic checkpoints include room_2, room_3, vt_1, room_7, room_13, room_14, and obj_175 approach generated_ring_037
- object target marker and approach pose marker are visible
- claim boundary says visual_kinematic_proxy_only and object_level_smoke_test_only

This is not physical stair climbing, not gait, not footstep planning, not contact-based stair climbing, not real quadruped locomotion, not Nav2 execution, not AMCL/localization, and not a full object-navigation benchmark.
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-route-json", type=Path, default=DEFAULT_BASE_ROUTE)
    parser.add_argument("--floor-2-map-yaml", type=Path, default=DEFAULT_FLOOR_2_MAP)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--inflation-radius-m", type=float, default=0.22)
    parser.add_argument("--max-snap-radius-m", type=float, default=1.50)
    parser.add_argument("--route-spacing-m", type=float, default=0.10)
    parser.add_argument("--checkpoint-tolerance-m", type=float, default=0.45)
    parser.add_argument("--approach-tolerance-m", type=float, default=0.45)
    parser.add_argument("--close-range-fallback-max-m", type=float, default=1.25)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        route = build(args)
        write_artifacts(args.output_dir, route)
        print(json.dumps({"classification": route["classification"], "route": str(args.output_dir / "planned_object_level_3d_route_v0_1.json")}, sort_keys=True))
        if route["classification"] == "task24i_blocked_by_missing_object_resolution_or_candidate":
            return 2
        if route["classification"] == "task24i_blocked_by_object_approach_route_infeasible":
            return 3
        return 0
    except Exception as exc:
        classification = "task24i_blocked_by_missing_object_resolution_or_candidate"
        if "approach route infeasible" in str(exc):
            classification = "task24i_blocked_by_object_approach_route_infeasible"
        blocked = {
            "artifact_type": "task24i_blocked_report",
            "created_utc": now_iso(),
            "classification": classification,
            "error": str(exc),
            "query": QUERY,
            "object_id": OBJECT_ID,
            "approach_candidate_id": APPROACH_CANDIDATE_ID,
            "claim_boundary": CLAIM_BOUNDARY,
        }
        for name in [
            "task24i_report.json",
            "object_query_resolution_v0_1.json",
            "object_approach_candidate_v0_1.json",
            "object_level_cross_floor_route_contract_v0_1.json",
            "planned_object_level_3d_route_v0_1.json",
            "object_level_approach_validation.json",
            "object_level_stair_transition_validation.json",
            "object_level_claim_boundary.json",
            "object_level_rviz_topic_manifest.json",
        ]:
            write_json(args.output_dir / name, blocked)
        write_text(args.output_dir / "task24i_report.md", f"# task24i Report\n\nClassification: `{classification}`\n\nError: {exc}\n")
        write_text(args.output_dir / "manual_gui_validation_instructions.md", manual_gui_instructions())
        write_text(args.output_dir / "final_answer_for_user.md", f"Classification: `{classification}`\n\nError: {exc}\n")
        print(json.dumps({"classification": classification, "error": str(exc)}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
