#!/usr/bin/env python3
"""Build Step 16 downstream navigation-projection audit artifacts.

This script reads only committed/public artifacts for the retained scenes. It
does not modify Stage-A runtime behavior, topology construction, route planning,
artifact schemas, or committed/public exports.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step16_navigation_projection_contract"

SCENES = {
    "00843-DYehNKdT76V": REPO_ROOT
    / "runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V",
    "00824-Dd4bFSTQ8gi": REPO_ROOT
    / "runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi",
    "00862-LT9Jq6dN3Ea": REPO_ROOT
    / "runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea",
    "00829-QaLdnwvtxbs": REPO_ROOT
    / "runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs",
}

COMMITTED_ARTIFACT_NAMES = (
    "topology_v0_1.json",
    "topology_query_report.json",
    "committed_room_world_model_v0_1.json",
    "committed_room_world_snapshot_v0_1.json",
)


def rel(path: Path) -> str:
    return str(path.relative_to(REPO_ROOT))


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def room_id(value: Any) -> str:
    text = str(value)
    if text.startswith("room_"):
        return text
    if text.isdigit():
        return f"room_{text}"
    return text


def room_local_int(value: Any) -> int | None:
    text = room_id(value)
    if text.startswith("room_") and text[5:].isdigit():
        return int(text[5:])
    return None


def edge_id(edge: dict[str, Any]) -> str:
    return f"{edge.get('source')}__{edge.get('relation_type')}__{edge.get('target')}"


def sorted_pair(a: Any, b: Any) -> tuple[str, str]:
    return tuple(sorted((room_id(a), room_id(b))))


def xy_from(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        if "x" in value and "y" in value:
            return [float(value["x"]), float(value["y"])]
        if "xy" in value:
            return xy_from(value["xy"])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
    return None


def collect_points(*collections: Any) -> list[list[float]]:
    points: list[list[float]] = []
    for collection in collections:
        if not collection:
            continue
        for item in collection:
            if isinstance(item, dict):
                for key in (
                    "center",
                    "pos_world",
                    "position",
                    "pose",
                    "pose_3d",
                    "from_position_xy",
                    "to_position_xy",
                ):
                    xy = xy_from(item.get(key))
                    if xy is not None:
                        points.append(xy)
                polygon = item.get("polygon") or item.get("footprint_2d")
                if isinstance(polygon, list):
                    for vertex in polygon:
                        xy = xy_from(vertex)
                        if xy is not None:
                            points.append(xy)
    return points


def coordinate_range(points: list[list[float]]) -> dict[str, float | None]:
    if not points:
        return {"x_min": None, "x_max": None, "y_min": None, "y_max": None}
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return {
        "x_min": round(min(xs), 3),
        "x_max": round(max(xs), 3),
        "y_min": round(min(ys), 3),
        "y_max": round(max(ys), 3),
    }


def gateway_index(gateways: list[dict[str, Any]]) -> dict[tuple[str, str], list[dict[str, Any]]]:
    index: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for gateway in gateways:
        connects = gateway.get("connects")
        if isinstance(connects, list) and len(connects) >= 2:
            index[sorted_pair(connects[0], connects[1])].append(gateway)
    return index


def evidence_index(evidences: Any) -> dict[str, dict[str, Any]]:
    if isinstance(evidences, list):
        return {str(item.get("evidence_id")): item for item in evidences if isinstance(item, dict)}
    if isinstance(evidences, dict):
        return evidences
    return {}


def evidence_types(edge: dict[str, Any], evidences: dict[str, dict[str, Any]]) -> Counter:
    counts = Counter()
    breakdown = edge.get("metadata", {}).get("evidence_type_breakdown", {})
    if isinstance(breakdown, dict):
        counts.update({str(k): int(v) for k, v in breakdown.items()})
    for evidence_id in edge.get("evidence_ids") or []:
        evidence = evidences.get(str(evidence_id), {})
        evidence_type = evidence.get("evidence_type")
        if evidence_type and evidence_type not in counts:
            counts[str(evidence_type)] += 1
    return counts


def gateway_waypoint(gateways: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not gateways:
        return None
    gateway = gateways[0]
    xy = xy_from(gateway.get("pos_world")) or xy_from(gateway.get("grid_pos"))
    if xy is None:
        return None
    return {
        "type": "gateway",
        "xy": [round(xy[0], 3), round(xy[1], 3)],
        "width_m": gateway.get("width_m"),
        "yaw": gateway.get("yaw"),
    }


def room_center_waypoint(room: dict[str, Any] | None) -> dict[str, Any] | None:
    if not room:
        return None
    xy = xy_from(room.get("center"))
    if xy is None:
        return None
    return {
        "type": "room_center",
        "room_id": room.get("id"),
        "floor_id": room.get("floor_id"),
        "xy": [round(xy[0], 3), round(xy[1], 3)],
    }


def classify_edge(
    edge: dict[str, Any],
    rooms: dict[str, dict[str, Any]],
    gateways_by_pair: dict[tuple[str, str], list[dict[str, Any]]],
    evidences: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    src = room_id(edge.get("source"))
    dst = room_id(edge.get("target"))
    relation = str(edge.get("relation_type", "unknown"))
    confidence = float(edge.get("confidence") or 0.0)
    support = int(edge.get("support_count") or 0)
    types = evidence_types(edge, evidences)
    pair_gateways = gateways_by_pair.get(sorted_pair(src, dst), [])
    src_floor = rooms.get(src, {}).get("floor_id")
    dst_floor = rooms.get(dst, {}).get("floor_id")
    same_floor = src_floor == dst_floor and src_floor is not None
    has_gateway = bool(pair_gateways)
    has_door = types.get("door_detection", 0) > 0
    has_trajectory = types.get("trajectory_transition", 0) > 0 or types.get("repeated_crossing", 0) > 0
    weak = confidence < 0.25 or (support <= 1 and relation in {"adjacent", "possible_connection"})

    disabled_reason = ""
    safety_label = "visualization_only"
    nav_policy = "visualization_only"
    primary = "unknown"

    if relation == "vertical_transition" or not same_floor:
        primary = "vertical_transition"
        nav_policy = "unsupported_multifloor"
        disabled_reason = "vertical_transition_requires_explicit_stairs_elevator_or_transition_model"
        safety_label = "marker_only_multifloor"
    elif relation == "possible_connection":
        primary = "possible_connection"
        nav_policy = "disabled_for_navigation"
        disabled_reason = "possible_connection_not_robot_traversable_without_explicit_validation"
        safety_label = "disabled_unvalidated_possible_connection"
    elif relation == "transition" and has_trajectory:
        primary = "trajectory_supported_transition"
        nav_policy = "robot_candidate_needs_validation"
        disabled_reason = "not_collision_free_and_requires_waypoint_planner_validation"
        safety_label = "candidate_not_collision_free"
    elif relation == "adjacent" and has_gateway and has_door:
        primary = "gateway_backed_passage"
        nav_policy = "robot_candidate_needs_validation"
        disabled_reason = "not_collision_free_and_requires_gateway_clearance_validation"
        safety_label = "candidate_not_collision_free"
    elif relation == "adjacent" and weak:
        primary = "weak_low_support_edge"
        nav_policy = "disabled_for_navigation"
        disabled_reason = "weak_or_low_confidence_adjacency_not_used_for_robot_execution"
        safety_label = "disabled_weak_edge"
    elif relation == "adjacent" and has_gateway:
        primary = "gateway_backed_passage"
        nav_policy = "robot_candidate_needs_validation"
        disabled_reason = "not_collision_free_and_requires_gateway_clearance_validation"
        safety_label = "candidate_not_collision_free"
    elif relation == "adjacent":
        primary = "polygon_proximity_adjacency"
        nav_policy = "visualization_only"
        disabled_reason = "polygon_adjacency_is_not_automatic_robot_traversability"
        safety_label = "visualization_only_polygon_adjacency"

    return {
        "edge_id": edge_id(edge),
        "source": src,
        "target": dst,
        "source_floor_id": src_floor,
        "target_floor_id": dst_floor,
        "same_floor": same_floor,
        "relation_type": relation,
        "confidence": confidence,
        "support_count": support,
        "edge_evidence_type": primary,
        "evidence_flags": sorted(types.keys()),
        "has_gateway_waypoint": has_gateway,
        "gateway_waypoint": gateway_waypoint(pair_gateways),
        "room_center_fallback_waypoints": [
            room_center_waypoint(rooms.get(src)),
            room_center_waypoint(rooms.get(dst)),
        ],
        "navigation_policy": nav_policy,
        "disabled_reason": disabled_reason,
        "not_collision_free": True,
        "requires_planner_validation": nav_policy == "robot_candidate_needs_validation",
        "safety_label": safety_label,
    }


def route_waypoints(
    first_route: dict[str, Any] | None,
    rooms: dict[str, dict[str, Any]],
    gateways_by_pair: dict[tuple[str, str], list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    if not first_route:
        return []
    sequence = [room_id(item) for item in first_route.get("room_sequence") or []]
    waypoints: list[dict[str, Any]] = []
    for index, current in enumerate(sequence):
        center = room_center_waypoint(rooms.get(current))
        if center:
            center["source"] = "topology_room_center"
            waypoints.append(center)
        if index + 1 < len(sequence):
            nxt = sequence[index + 1]
            gateway = gateway_waypoint(gateways_by_pair.get(sorted_pair(current, nxt), []))
            if gateway:
                gateway["source"] = "snapshot_gateway_pos_world"
                gateway["connects"] = [current, nxt]
                waypoints.append(gateway)
    return waypoints


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def ros_pkg_inventory() -> dict[str, Any]:
    wanted = "gazebo|nav2|turtlebot|slam_toolbox|map_server|amcl|robot_state_publisher|joint_state_publisher|tf2_ros"
    result = {
        "which_gazebo": shutil.which("gazebo"),
        "which_gzserver": shutil.which("gzserver"),
        "which_gzclient": shutil.which("gzclient"),
        "which_ros2": shutil.which("ros2"),
        "matching_ros2_packages": [],
    }
    if result["which_ros2"]:
        proc = subprocess.run(
            ["bash", "-lc", f"ros2 pkg list | rg \"{wanted}\" || true"],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        result["matching_ros2_packages"] = [line for line in proc.stdout.splitlines() if line.strip()]
    return result


def shallow_asset_inventory() -> dict[str, list[str]]:
    commands = {
        "gazebo_nav_assets": "find . -maxdepth 4 \\( -iname '*gazebo*' -o -iname '*.world' -o -iname '*.sdf' -o -iname '*.urdf' -o -iname '*nav2*' -o -iname '*map*.yaml' -o -iname '*.pgm' \\) -print | sort",
        "robot_launch_assets": "find . -maxdepth 4 \\( -iname '*turtlebot*' -o -iname '*robot*' -o -iname '*launch*.py' \\) -print | sort",
    }
    inventory: dict[str, list[str]] = {}
    for name, command in commands.items():
        proc = subprocess.run(
            ["bash", "-lc", command],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        inventory[name] = [line[2:] if line.startswith("./") else line for line in proc.stdout.splitlines()]
    return inventory


def main() -> None:
    tables_dir = OUTPUT_ROOT / "tables"
    scenes_dir = OUTPUT_ROOT / "scenes"
    manifest_dir = OUTPUT_ROOT / "manifest"
    all_edge_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []
    geometry_rows: list[dict[str, Any]] = []
    generated_files: list[str] = []

    for scene_id, scene_root in SCENES.items():
        logs = scene_root / "logs"
        topo = load_json(logs / "topology_v0_1.json")
        query = load_json(logs / "topology_query_report.json")
        model = load_json(logs / "committed_room_world_model_v0_1.json")
        snapshot = load_json(logs / "committed_room_world_snapshot_v0_1.json")

        rooms = {room_id(room.get("id")): room for room in topo.get("rooms", [])}
        gateways_by_pair = gateway_index(snapshot.get("gateways", []))
        evidences = evidence_index(topo.get("evidences"))
        classified = [
            classify_edge(edge, rooms, gateways_by_pair, evidences)
            for edge in topo.get("edges", [])
        ]

        scene_out = scenes_dir / scene_id
        scene_out.mkdir(parents=True, exist_ok=True)
        nav_topology_path = scene_out / "navigation_topology_v0_1.json"
        nav_waypoints_path = scene_out / "navigation_waypoints_v0_1.json"

        robot_candidates = [row for row in classified if row["navigation_policy"] == "robot_candidate_needs_validation"]
        disabled = [row for row in classified if row["navigation_policy"] == "disabled_for_navigation"]
        visualization_only = [row for row in classified if row["navigation_policy"] == "visualization_only"]
        unsupported_multifloor = [row for row in classified if row["navigation_policy"] == "unsupported_multifloor"]

        nav_topology = {
            "version": "0.1",
            "artifact_kind": "downstream_navigation_projection",
            "scene_id": scene_id,
            "frame_id": "boxfusion_map",
            "source_committed_artifact_paths": [rel(logs / name) for name in COMMITTED_ARTIFACT_NAMES],
            "source_policy": "committed_public_artifacts_only",
            "does_not_modify_committed_artifacts": True,
            "robot_enabled_edges": [],
            "robot_candidate_edges": robot_candidates,
            "disabled_edges": disabled,
            "visualization_only_edges": visualization_only,
            "unsupported_multifloor_edges": unsupported_multifloor,
            "vertical_transition_policy": "marker_only_unless_gazebo_world_models_stairs_elevator_or_transition",
            "weak_edge_policy": "disabled_for_robot_execution_by_default",
            "polygon_adjacency_policy": "visualization_only_unless_explicitly_validated",
            "safety_label": "no_collision_free_navigation_claim",
        }
        with nav_topology_path.open("w", encoding="utf-8") as handle:
            json.dump(nav_topology, handle, indent=2, sort_keys=True)
            handle.write("\n")
        generated_files.append(rel(nav_topology_path))

        route = query.get("first_route")
        nav_waypoints = {
            "version": "0.1",
            "artifact_kind": "downstream_navigation_waypoints",
            "scene_id": scene_id,
            "frame_id": "boxfusion_map",
            "source_committed_artifact_paths": [rel(logs / name) for name in COMMITTED_ARTIFACT_NAMES],
            "source_policy": "committed_public_artifacts_only",
            "not_collision_free": True,
            "requires_planner_validation": True,
            "waypoint_semantics": "room_center_and_gateway_demo_waypoints_not_local_planner_paths",
            "sample_route": route,
            "sample_route_waypoints": route_waypoints(route, rooms, gateways_by_pair),
        }
        with nav_waypoints_path.open("w", encoding="utf-8") as handle:
            json.dump(nav_waypoints, handle, indent=2, sort_keys=True)
            handle.write("\n")
        generated_files.append(rel(nav_waypoints_path))

        class_counts = Counter(row["edge_evidence_type"] for row in classified)
        policy_counts = Counter(row["navigation_policy"] for row in classified)
        for edge_class in sorted(class_counts):
            summary_rows.append(
                {
                    "scene_id": scene_id,
                    "edge_evidence_type": edge_class,
                    "edge_count": class_counts[edge_class],
                    "robot_enabled": 0,
                    "robot_candidate_needs_validation": sum(
                        1
                        for row in classified
                        if row["edge_evidence_type"] == edge_class
                        and row["navigation_policy"] == "robot_candidate_needs_validation"
                    ),
                    "visualization_only": sum(
                        1
                        for row in classified
                        if row["edge_evidence_type"] == edge_class
                        and row["navigation_policy"] == "visualization_only"
                    ),
                    "disabled_for_navigation": sum(
                        1
                        for row in classified
                        if row["edge_evidence_type"] == edge_class
                        and row["navigation_policy"] == "disabled_for_navigation"
                    ),
                    "unsupported_multifloor": sum(
                        1
                        for row in classified
                        if row["edge_evidence_type"] == edge_class
                        and row["navigation_policy"] == "unsupported_multifloor"
                    ),
                }
            )
        summary_rows.append(
            {
                "scene_id": scene_id,
                "edge_evidence_type": "TOTAL",
                "edge_count": len(classified),
                "robot_enabled": policy_counts["robot_enabled"],
                "robot_candidate_needs_validation": policy_counts["robot_candidate_needs_validation"],
                "visualization_only": policy_counts["visualization_only"],
                "disabled_for_navigation": policy_counts["disabled_for_navigation"],
                "unsupported_multifloor": policy_counts["unsupported_multifloor"],
            }
        )

        for row in classified:
            all_edge_rows.append(
                {
                    "scene_id": scene_id,
                    "edge_id": row["edge_id"],
                    "source": row["source"],
                    "target": row["target"],
                    "source_floor_id": row["source_floor_id"],
                    "target_floor_id": row["target_floor_id"],
                    "relation_type": row["relation_type"],
                    "confidence": row["confidence"],
                    "support_count": row["support_count"],
                    "edge_evidence_type": row["edge_evidence_type"],
                    "evidence_flags": ";".join(row["evidence_flags"]),
                    "has_gateway_waypoint": "yes" if row["has_gateway_waypoint"] else "no",
                    "navigation_policy": row["navigation_policy"],
                    "disabled_reason": row["disabled_reason"],
                    "not_collision_free": "yes",
                    "requires_planner_validation": "yes"
                    if row["requires_planner_validation"]
                    else "no",
                    "safety_label": row["safety_label"],
                }
            )

        points = collect_points(
            topo.get("rooms", []),
            snapshot.get("gateways", []),
            snapshot.get("anchors", []),
            snapshot.get("objects", []),
            snapshot.get("vertical_transitions", []),
        )
        ranges = coordinate_range(points)
        floor_z = [float(floor.get("z_center")) for floor in topo.get("floors", []) if floor.get("z_center") is not None]
        floor_sep = None
        if len(floor_z) >= 2:
            ordered = sorted(floor_z)
            floor_sep = round(min(abs(b - a) for a, b in zip(ordered, ordered[1:])), 3)

        geometry_rows.append(
            {
                "scene_id": scene_id,
                "room_polygons": len(topo.get("rooms", [])),
                "room_centers": sum(1 for room in topo.get("rooms", []) if xy_from(room.get("center"))),
                "gateway_positions": sum(1 for gateway in snapshot.get("gateways", []) if xy_from(gateway.get("pos_world"))),
                "anchors": len(snapshot.get("anchors", [])),
                "anchor_positions": sum(1 for anchor in snapshot.get("anchors", []) if xy_from(anchor.get("position"))),
                "objects": len(snapshot.get("objects", [])),
                "object_positions": sum(1 for obj in snapshot.get("objects", []) if xy_from(obj.get("pose")) or xy_from(obj.get("pose_3d"))),
                "object_labels": len({obj.get("label") for obj in snapshot.get("objects", []) if obj.get("label")}),
                "floors": len(topo.get("floors", [])),
                "vertical_transition_positions": sum(
                    1
                    for transition in snapshot.get("vertical_transitions", [])
                    if xy_from(transition.get("from_position_xy")) and xy_from(transition.get("to_position_xy"))
                ),
                "route_room_sequences": "yes" if query.get("first_route", {}).get("room_sequence") else "no",
                "route_relation_metadata": "yes" if query.get("first_route", {}).get("used_relation_types") else "no",
                "x_min": ranges["x_min"],
                "x_max": ranges["x_max"],
                "y_min": ranges["y_min"],
                "y_max": ranges["y_max"],
                "min_floor_z_separation": floor_sep,
                "topological_waypoint_graph": "yes",
                "approx_2d_floorplan_occupancy_map": "partial_polygons_only_not_occupancy",
                "room_center_gateway_waypoint_sequence": "yes_gateway_partial",
                "gazebo_marker_only_world": "yes",
                "gazebo_occupancy_map_server_world": "not_from_current_artifacts_without_rasterization_and_validation",
                "nav2_compatible_map": "no_requires_validated_occupancy_metadata_and_free_space",
            }
        )

    write_csv(
        tables_dir / "navigation_edge_validation_report.csv",
        all_edge_rows,
        [
            "scene_id",
            "edge_id",
            "source",
            "target",
            "source_floor_id",
            "target_floor_id",
            "relation_type",
            "confidence",
            "support_count",
            "edge_evidence_type",
            "evidence_flags",
            "has_gateway_waypoint",
            "navigation_policy",
            "disabled_reason",
            "not_collision_free",
            "requires_planner_validation",
            "safety_label",
        ],
    )
    generated_files.append(rel(tables_dir / "navigation_edge_validation_report.csv"))

    write_csv(
        tables_dir / "navigation_edge_class_summary.csv",
        summary_rows,
        [
            "scene_id",
            "edge_evidence_type",
            "edge_count",
            "robot_enabled",
            "robot_candidate_needs_validation",
            "visualization_only",
            "disabled_for_navigation",
            "unsupported_multifloor",
        ],
    )
    generated_files.append(rel(tables_dir / "navigation_edge_class_summary.csv"))

    write_csv(
        tables_dir / "navigation_geometry_availability.csv",
        geometry_rows,
        [
            "scene_id",
            "room_polygons",
            "room_centers",
            "gateway_positions",
            "anchors",
            "anchor_positions",
            "objects",
            "object_positions",
            "object_labels",
            "floors",
            "vertical_transition_positions",
            "route_room_sequences",
            "route_relation_metadata",
            "x_min",
            "x_max",
            "y_min",
            "y_max",
            "min_floor_z_separation",
            "topological_waypoint_graph",
            "approx_2d_floorplan_occupancy_map",
            "room_center_gateway_waypoint_sequence",
            "gazebo_marker_only_world",
            "gazebo_occupancy_map_server_world",
            "nav2_compatible_map",
        ],
    )
    generated_files.append(rel(tables_dir / "navigation_geometry_availability.csv"))

    inventory = {
        "shallow_asset_inventory": shallow_asset_inventory(),
        "ros_gazebo_environment": ros_pkg_inventory(),
    }
    inventory_path = tables_dir / "gazebo_environment_inventory.json"
    inventory_path.parent.mkdir(parents=True, exist_ok=True)
    with inventory_path.open("w", encoding="utf-8") as handle:
        json.dump(inventory, handle, indent=2, sort_keys=True)
        handle.write("\n")
    generated_files.append(rel(inventory_path))

    manifest = {
        "version": "0.1",
        "artifact_kind": "step16_navigation_projection_contract_manifest",
        "output_root": rel(OUTPUT_ROOT),
        "source_policy": "committed_public_artifacts_only",
        "stage_a_runtime_modified": False,
        "topology_construction_modified": False,
        "route_planning_modified": False,
        "artifact_schemas_modified": False,
        "generated_files": sorted(generated_files),
    }
    manifest_path = manifest_dir / "step16_navigation_projection_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(f"Wrote {rel(OUTPUT_ROOT)}")


if __name__ == "__main__":
    main()
