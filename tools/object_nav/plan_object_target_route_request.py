#!/usr/bin/env python3
"""Create an offline target-room-first route request from an object query result."""

from __future__ import annotations

import argparse
from pathlib import Path

from object_nav_common import FLOOR2_EXEC_ROUTE, FLOOR2_MAP_YAML, FLOOR2_SEMANTIC_ROUTE, TASK_DIR, load_index, load_json, route_assets_status, utc_now, write_json


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    parser.add_argument("--query-result", type=Path, required=True)
    parser.add_argument("--start-room-id", default="room_11")
    parser.add_argument("--output-dir", type=Path, default=TASK_DIR / "sample_route_requests")
    args = parser.parse_args()
    index = load_index(args.index)
    query = load_json(args.query_result, {})
    selected = query.get("selected_candidate")
    route_status = route_assets_status()
    validation_checks = []
    if not selected:
        route_request_status = "no_selected_candidate"
        fallback = query.get("failure_reason") or "query did not select a candidate"
        target_room = target_floor = None
    else:
        target_room = selected.get("room_id")
        target_floor = selected.get("floor_id")
        if target_floor == "floor_2" and route_status["available"] and target_room in set(route_status["room_sequence"]):
            route_request_status = "ready_for_later_runtime"
            fallback = None
        elif target_floor == "floor_2":
            route_request_status = "symbolic_only"
            fallback = "floor_2 map exists, but selected room is not covered by the existing room11-to-room14 route bridge"
        else:
            route_request_status = "symbolic_only"
            fallback = "target floor lacks a prepared task14a runtime route bridge"
    validation_checks.extend(
        [
            {"check": "selected_candidate_present", "passed": bool(selected)},
            {"check": "nav2_not_executed", "passed": True},
            {"check": "object_approach_pose_not_claimed", "passed": True},
            {"check": "floor2_map_yaml_available", "passed": FLOOR2_MAP_YAML.exists()},
            {"check": "floor2_route_assets_available", "passed": route_status["available"]},
            {"check": "target_room_in_existing_floor2_route", "passed": bool(selected and selected.get("room_id") in set(route_status["room_sequence"]))},
        ]
    )
    request = {
        "version": "v0_1",
        "artifact_type": "object_target_route_request",
        "scene_id": index.get("scene_id"),
        "created_utc": utc_now(),
        "execution_mode": "target_room_first",
        "route_request_status": route_request_status,
        "query": {
            "query_text": query.get("query_text"),
            "query_id": query.get("query_id"),
            "parsed_constraints": query.get("parsed_constraints"),
        },
        "selected_candidate": selected,
        "start_room_id": args.start_room_id,
        "target_room_id": target_room,
        "target_floor_id": target_floor,
        "route_intent": {
            "type": "room_to_room_target_room_first",
            "start_room_id": args.start_room_id,
            "goal_room_id": target_room,
            "goal_object_id": selected.get("object_id") if selected else None,
        },
        "expected_artifacts": {
            "map_yaml": str(FLOOR2_MAP_YAML) if target_floor == "floor_2" else None,
            "semantic_route_waypoints": str(FLOOR2_SEMANTIC_ROUTE) if target_floor == "floor_2" else None,
            "executable_route_waypoints": str(FLOOR2_EXEC_ROUTE) if target_floor == "floor_2" else None,
        },
        "failure_or_fallback_reason": fallback,
        "non_claims": [
            "No Gazebo/Nav2/ROS execution was performed.",
            "Target-room request readiness does not claim object arrival.",
            "Object approach pose requires later free-cell snap and clearance validation.",
        ],
    }
    manifest = {
        "version": "v0_1",
        "artifact_type": "object_nav_overlay_manifest",
        "scene_id": index.get("scene_id"),
        "created_utc": utc_now(),
        "query_result": str(args.query_result),
        "route_request": str(args.output_dir / "object_target_route_request_v0_1.json"),
        "selected_object_overlay": selected,
        "target_room_id": target_room,
        "target_floor_id": target_floor,
        "overlay_inputs": {
            "object_candidate_index": str(args.index),
            "map_yaml": str(FLOOR2_MAP_YAML) if target_floor == "floor_2" else None,
            "semantic_route_waypoints": str(FLOOR2_SEMANTIC_ROUTE) if target_floor == "floor_2" else None,
            "executable_route_waypoints": str(FLOOR2_EXEC_ROUTE) if target_floor == "floor_2" else None,
        },
        "overlay_status": "metadata_only_no_runtime_overlay_published",
    }
    validation = {
        "version": "v0_1",
        "artifact_type": "object_nav_validation",
        "scene_id": index.get("scene_id"),
        "created_utc": utc_now(),
        "route_request_status": route_request_status,
        "checks": validation_checks,
        "overall_passed": all(c["passed"] for c in validation_checks[:4]),
        "runtime_execution_performed": False,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / "object_target_route_request_v0_1.json", request)
    write_json(args.output_dir / "object_nav_overlay_manifest_v0_1.json", manifest)
    write_json(args.output_dir / "object_nav_validation_v0_1.json", validation)
    print(f"wrote {args.output_dir}")
    print(f"route_request_status={route_request_status}")


if __name__ == "__main__":
    main()
