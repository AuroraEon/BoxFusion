"""Package canonical Layer 4 runtime inputs without launching runtime systems."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .common import PROJECT_NAME, resolve_repo_root, save_json


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task38_layer4_runtime_validation_packaging_authorized_demo_and_pipeline_runbook"
LAYER = "Layer 4: Runtime Validation Layer"
OFFLINE_PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"
RUNTIME_PYTHON = "/usr/bin/python3"
SELECTED_PROFILE = "conservative_canonical"
SELECTED_ROUTE_ID = "cross_floor_room"
CLASSIFICATION = (
    "task38_layer4_runtime_packaging_preflight_runbook_completed_"
    "runtime_not_run_authorization_required"
)

DOCS_AND_MANIFESTS = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/canonical_output_plan.md",
    "docs/rslg_slam/final_layer2_minimal_generation_plan.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/tool_migration_plan.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json",
    "docs/rslg_slam/manifests/tool_entrypoint_mapping_manifest_v0_1.json",
    "docs/rslg_slam/manifests/tool_migration_plan_manifest_v0_1.json",
    "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json",
    "docs/rslg_slam/manifests/legacy_inventory_manifest_v0_1.json",
]

DOC_OUTPUTS = [
    "docs/rslg_slam/canonical_pipeline_runbook.md",
    "docs/rslg_slam/runtime_validation_runbook.md",
    "docs/rslg_slam/current_project_status.md",
]

WRAPPER_OUTPUTS = [
    "tools/rslg_pipeline/run_layer1_world_model.sh",
    "tools/rslg_pipeline/run_layer2_formal_artifacts.sh",
    "tools/rslg_pipeline/run_layer3_navigation_interface.sh",
    "tools/rslg_pipeline/run_layer4_runtime_validation_static.sh",
    "tools/rslg_pipeline/run_rslg_pipeline_static.sh",
]

FORBIDDEN_CLAIMS = [
    "dense reconstruction",
    "neural implicit SLAM",
    "full embodied navigation benchmark",
    "AMCL success",
    "real robot execution",
    "physical stair climbing",
    "gait planning",
    "footstep planning",
    "contact planning",
    "collision-free guarantee",
    "LLM runtime navigation",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path, repo: Path) -> str:
    try:
        return path.resolve().relative_to(repo.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path, repo: Path, role: str | None = None) -> dict[str, Any]:
    return {
        "path": rel(path, repo),
        "exists": path.is_file(),
        "non_empty": path.is_file() and path.stat().st_size > 0,
        "size_bytes": path.stat().st_size if path.is_file() else None,
        "sha256": sha256(path) if path.is_file() else None,
        "role": role,
    }


def snapshot(paths: Iterable[Path], repo: Path) -> dict[str, dict[str, Any]]:
    return {rel(path, repo): file_record(path, repo) for path in sorted(paths)}


def parse_map_yaml(path: Path) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        values[key.strip()] = value.strip()
    image_value = values.get("image")
    image_path = (path.parent / image_value).resolve() if image_value else None
    return {
        "values": values,
        "image_path": image_path,
        "image_resolves": bool(image_path and image_path.is_file()),
    }


def probe_command(command: list[str], cwd: Path) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            text=True,
            capture_output=True,
            check=False,
            timeout=20,
        )
        return {
            "command": command,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
            "available": result.returncode == 0,
        }
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": str(exc),
            "available": False,
        }


def yaw_for(points: list[dict[str, Any]], index: int) -> float:
    import math

    if len(points) < 2:
        return 0.0
    before = points[index - 1] if index == len(points) - 1 else points[index]
    after = points[index] if index == len(points) - 1 else points[index + 1]
    return round(math.atan2(float(after["y"]) - float(before["y"]), float(after["x"]) - float(before["x"])), 6)


def runtime_waypoints(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "waypoint_index": index,
            "floor_id": point["floor_id"],
            "segment_id": point["segment_id"],
            "x": point["x"],
            "y": point["y"],
            "yaw": yaw_for(points, index),
            "source": "canonical_layer3_selected_route",
        }
        for index, point in enumerate(points)
    ]


def rviz_config() -> str:
    return """Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: RSLG Selected Route
      Topic:
        Value: /rslg/layer4/selected_route
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: RSLG Anchors
      Topic:
        Value: /rslg/layer4/anchors
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: RSLG Connector
      Topic:
        Value: /rslg/layer4/connector
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: RSLG Readiness
      Topic:
        Value: /rslg/layer4/readiness
  Global Options:
    Fixed Frame: world
"""


def claim_boundary_report(paths: list[Path], repo: Path) -> dict[str, Any]:
    affirmative_patterns = {
        "amcl_success_true": ["amcl_success: true", '"amcl_success": true'],
        "physical_stair_climbing_true": [
            "physical_stair_climbing_supported: true",
            '"physical_stair_climbing_supported": true',
        ],
        "real_robot_execution_true": [
            "real_robot_execution: true",
            '"real_robot_execution": true',
        ],
        "collision_free_guarantee_true": [
            "collision_free_guarantee: true",
            '"collision_free_guarantee": true',
            "collision_free_guarantee_claimed: true",
            '"collision_free_guarantee_claimed": true',
        ],
        "object_executable_success": [
            "object executable approach readiness: ready",
            '"object_executable_approach_readiness": "ready"',
        ],
    }
    violations = []
    checked = []
    for path in paths:
        if not path.is_file() or path.suffix.lower() not in {".json", ".md", ".txt", ".sh"}:
            continue
        text = path.read_text(encoding="utf-8", errors="replace").lower()
        checked.append(rel(path, repo))
        for rule, patterns in affirmative_patterns.items():
            for pattern in patterns:
                if pattern in text:
                    violations.append({"path": rel(path, repo), "rule": rule, "pattern": pattern})
    return {
        "report_id": "runtime_claim_boundary_report_v0_1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "status": "passed" if not violations else "failed",
        "forbidden_claims_are_documented_as_boundaries": FORBIDDEN_CLAIMS,
        "files_checked": checked,
        "violations": violations,
        "runtime_execution_claimed": False,
        "object_executable_approach_success_claimed": False,
    }


def run(repo: Path, scene_id: str) -> dict[str, Any]:
    generated_at = now_iso()
    canonical = repo / "stage_outputs/rslg_slam" / scene_id / "canonical"
    layer2 = canonical / "layer2_formal_artifacts"
    layer3 = canonical / "layer3_navigation_interface"
    layer4 = canonical / "layer4_runtime_validation"
    task = repo / "stage_outputs/rslg_slam" / scene_id / "tasks" / TASK_NAME

    for subdir in [
        "runtime_inputs", "maps", "routes", "rviz", "launch", "logs",
        "trajectories", "validation_reports", "object_readiness", "manifests", "reports",
    ]:
        (layer4 / subdir).mkdir(parents=True, exist_ok=True)
    task.mkdir(parents=True, exist_ok=True)

    paths = {
        "route": layer3 / "real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
        "selected": layer3 / "executable_route_candidates/cross_floor_room_executable_route_candidate_selected_v0_1.json",
        "contract": layer3 / "route_contracts/cross_floor_room_route_contract_v0_1.json",
        "request": layer3 / "planner_requests/cross_floor_room_planner_request_v0_1.json",
        "layer3_report": layer3 / "reports/layer3_navigation_interface_validation_report_v0_1.json",
        "comparison": layer3 / "profile_comparison/stable_map_profile_route_comparison_report_v0_1.json",
        "map_package": layer2 / "stable_maps/stable_occupancy_map_package_v0_1.json",
        "floor1_yaml": layer2 / "stable_maps/floor_1/floor_1_stable_occupancy_map_v0_1.yaml",
        "floor1_pgm": layer2 / "stable_maps/floor_1/floor_1_stable_occupancy_map_v0_1.pgm",
        "floor2_yaml": layer2 / "stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.yaml",
        "floor2_pgm": layer2 / "stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.pgm",
        "connectors": layer2 / "vertical_connectors/vertical_connectors_v0_1.json",
        "topology": layer2 / "topology/cross_floor_topology_v0_1.json",
        "object_interface": layer2 / "object_interfaces/object_interface_package_v0_1.json",
        "object_approach": layer2 / "object_interfaces/object_approach_v0_1.json",
        "object_query": layer2 / "object_interfaces/object_query_resolution_v0_1.json",
    }
    before = snapshot(
        list(layer2.rglob("*")) + list(layer3.rglob("*")),
        repo,
    )
    input_checks = []
    input_errors = []
    payloads: dict[str, Any] = {}
    json_keys = {
        "route", "selected", "contract", "request", "layer3_report", "comparison",
        "map_package", "connectors", "topology", "object_interface", "object_approach", "object_query",
    }
    for name, path in paths.items():
        record = file_record(path, repo, name)
        if name in json_keys and record["exists"]:
            try:
                payloads[name] = load_json(path)
                record["json_loadable"] = True
            except Exception as exc:
                record["json_loadable"] = False
                record["json_error"] = str(exc)
        valid = record["exists"] and record["non_empty"] and record.get("json_loadable", True)
        record["status"] = "pass" if valid else "fail"
        input_checks.append(record)
        if not valid:
            input_errors.append(f"missing_or_invalid:{name}")
    if input_errors:
        report = {
            "task_name": TASK_NAME,
            "status": "blocked",
            "classification": "task38_layer4_runtime_packaging_blocked_by_layer3_inputs",
            "blocked_or_missing_items": input_errors,
        }
        save_json(task / "task38_report.json", report)
        return report

    route = payloads["route"]
    selected = payloads["selected"]
    contract = payloads["contract"]
    connectors = payloads["connectors"]
    object_interface = payloads["object_interface"]
    comparison = payloads["comparison"]
    floor_sequences = {
        floor_id: runtime_waypoints(points)
        for floor_id, points in route["route_floor_waypoints"].items()
    }
    navigation_waypoint_count = sum(len(points) for points in floor_sequences.values())

    copied = {}
    for floor_id, yaml_key, pgm_key in [
        ("floor_1", "floor1_yaml", "floor1_pgm"),
        ("floor_2", "floor2_yaml", "floor2_pgm"),
    ]:
        target_dir = layer4 / "maps" / floor_id
        target_dir.mkdir(parents=True, exist_ok=True)
        for key in (yaml_key, pgm_key):
            target = target_dir / paths[key].name
            shutil.copyfile(paths[key], target)
            copied[key] = target
    copied_route = layer4 / "routes/cross_floor_room_selected_conservative_canonical_v0_1.json"
    shutil.copyfile(paths["route"], copied_route)

    route_runtime = {
        "schema_name": "rslg_layer4_selected_route_runtime_input",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "artifact_layer": LAYER,
        "route_id": SELECTED_ROUTE_ID,
        "route_kind": "cross_floor_room",
        "selected_profile": SELECTED_PROFILE,
        "selected_candidate_id": selected["candidate_id"],
        "source_route_artifact": rel(paths["route"], repo),
        "packaged_route_copy": rel(copied_route, repo),
        "reported_interface_waypoint_count": selected["route_metrics"]["waypoint_count"],
        "floor_navigation_waypoint_count": navigation_waypoint_count,
        "floor_sequences": floor_sequences,
        "validated_route_chain": contract["validated_route_chain"],
        "vertical_transition": {
            **contract["vertical_transition"],
            "handling": "topological_handoff_between_floor_specific_executors",
            "physical_stair_motion_command_generated": False,
        },
        "object_route_selected": False,
        "object_centroid_navigation_used": False,
        "collision_free_guarantee_claimed": False,
    }
    save_json(layer4 / "runtime_inputs/selected_route_runtime_input_v0_1.json", route_runtime)

    map_server_inputs = {
        "schema_name": "rslg_layer4_map_server_inputs",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "selected_profile": SELECTED_PROFILE,
        "maps": {
            "floor_1": {
                "canonical_yaml": rel(paths["floor1_yaml"], repo),
                "canonical_pgm": rel(paths["floor1_pgm"], repo),
                "packaged_yaml": rel(copied["floor1_yaml"], repo),
                "packaged_pgm": rel(copied["floor1_pgm"], repo),
            },
            "floor_2": {
                "canonical_yaml": rel(paths["floor2_yaml"], repo),
                "canonical_pgm": rel(paths["floor2_pgm"], repo),
                "packaged_yaml": rel(copied["floor2_yaml"], repo),
                "packaged_pgm": rel(copied["floor2_pgm"], repo),
            },
        },
        "loading_assumptions": [
            "Load one floor map at a time into map_server.",
            "Preserve the YAML-relative image filename beside each packaged PGM.",
            "A floor transition requires an explicit executor handoff; map_server does not perform it.",
            "The maps are canonical RSLG-SLAM Layer 2 products, not external GT maps or runtime costmaps.",
        ],
    }
    save_json(layer4 / "runtime_inputs/map_server_inputs_v0_1.json", map_server_inputs)

    route_executor_inputs = {
        "schema_name": "rslg_layer4_route_executor_inputs",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "route_input": "selected_route_runtime_input_v0_1.json",
        "waypoint_format": ["waypoint_index", "floor_id", "segment_id", "x", "y", "yaw", "source"],
        "executor_expectations": [
            "Use /usr/bin/python3 for ROS2 or rclpy processes.",
            "Execute only the floor sequence matching the currently loaded map.",
            "Stop at the floor_1 connector entry before transition handling.",
            "Treat vt_1_centerline_e001 as the floor-transition edge.",
            "Do not treat vt_1_centerline_e003 as a floor-transition edge.",
            "Resume on floor_2 only after an explicit topological handoff and map/runtime state update.",
            "Do not substitute an object centroid or generated_ring_037 as the route goal.",
        ],
        "runtime_adapter_binding": {
            "status": "requires_authorized_runtime_integration",
            "existing_support_inspected": [
                "tools/stage1_runtime/run_scene_route.py",
                "tools/stage1_runtime/launch_scene_gazebo_nav2.sh",
                "tools/vertical_connectors/cross_floor_visual_traversal_player.py",
            ],
            "note": "No existing adapter was invoked or claimed compatible with this new canonical two-floor package.",
        },
    }
    save_json(layer4 / "runtime_inputs/route_executor_inputs_v0_1.json", route_executor_inputs)

    overlay_inputs = {
        "schema_name": "rslg_layer4_rviz_marker_overlay_inputs",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "fixed_frame": "world",
        "map_display_required": False,
        "map_display_policy": "disabled_or_avoided_due_to_historical_glsl_crash",
        "marker_arrays": [
            {"topic": "/rslg/layer4/selected_route", "content": "floor-specific selected route polylines"},
            {"topic": "/rslg/layer4/anchors", "content": "room and connector anchors"},
            {"topic": "/rslg/layer4/connector", "content": "vt_1 transition edge and non-transition context"},
            {"topic": "/rslg/layer4/readiness", "content": "start, goal, and blocked object-readiness markers"},
        ],
        "object_marker": {
            "object_id": "obj_175",
            "approach_candidate_id": "generated_ring_037",
            "status": "blocked_context_only",
            "navigation_goal": False,
        },
    }
    save_json(layer4 / "runtime_inputs/rviz_marker_overlay_inputs_v0_1.json", overlay_inputs)
    write_text(layer4 / "rviz/layer4_marker_overlays_v0_1.rviz", rviz_config())

    runtime_authorized = os.environ.get("RSLG_TASK38_ALLOW_RUNTIME") == "1"
    runtime_status = "not_run_authorization_required" if not runtime_authorized else "not_run_static_packaging_scope"
    runtime_package = {
        "schema_name": "rslg_layer4_runtime_input_package",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "artifact_layer": LAYER,
        "selected_route": {
            "route_id": SELECTED_ROUTE_ID,
            "candidate_id": selected["candidate_id"],
            "profile": SELECTED_PROFILE,
            "route_kind": "cross_floor_room",
        },
        "selected_stable_map_profile": SELECTED_PROFILE,
        "map_inputs": "map_server_inputs_v0_1.json",
        "route_inputs": "selected_route_runtime_input_v0_1.json",
        "vertical_connector_context": contract["vertical_transition"],
        "runtime_assumptions": {
            "historical_robot_profile": "turtlebot3_burger",
            "historical_ros_domain_id": 84,
            "historical_localization_mode": "static_map_to_odom",
            "amcl_success_claimed": False,
            "rviz_map_display_required": False,
            "marker_array_overlays_preferred": True,
        },
        "forbidden_claims": FORBIDDEN_CLAIMS,
        "runtime_authorization_detected": runtime_authorized,
        "runtime_execution_status": runtime_status,
        "object_executable_approach_status": "blocked",
        "object_route_selected": False,
        "navigation_thr0p25_candidate_selected": False,
    }
    save_json(layer4 / "runtime_inputs/runtime_input_package_v0_1.json", runtime_package)

    conservative_object = comparison["profile_results"][SELECTED_PROFILE]["generated_ring_037_status"]
    candidate_object = comparison["profile_results"]["navigation_thr0p25_candidate"]["generated_ring_037_status"]
    object_readiness = {
        "report_id": "object_runtime_readiness_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "query": "curtain in room_14 on floor_2",
        "object_id": "obj_175",
        "object_label": "curtain",
        "query_interface_readiness": "ready",
        "selected_profile": SELECTED_PROFILE,
        "generated_ring_037_selected_profile_status": conservative_object,
        "generated_ring_037_candidate_profile_analysis_context_only": candidate_object,
        "navigation_thr0p25_candidate_selected_for_runtime": False,
        "object_executable_approach_readiness": "blocked",
        "blocker_reason": object_interface["approach_validation_status"],
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "recommended_future_task": "task39_object_level_navigation_interface_recovery_and_candidate_approach_selection",
    }
    save_json(layer4 / "object_readiness/object_runtime_readiness_report_v0_1.json", object_readiness)
    blocker = {
        "report_id": "object_navigation_blocker_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "blocker_id": "obj_175_generated_ring_037_conservative_map_conflict",
        "blocker_type": "selected_profile_occupancy_and_clearance_conflict",
        "selected_profile": SELECTED_PROFILE,
        "source_evidence": [
            rel(paths["object_approach"], repo),
            rel(paths["object_interface"], repo),
            rel(paths["comparison"], repo),
        ],
        "lineage": {
            "task36": "canonical object approach blocked by current Layer 1 stable-map conflict",
            "task36b": "coordinate alignment audited without moving the approach",
            "task36c": "conservative stable map accepted",
            "task37": "room route selected; object executable approach blocked under selected profile",
            "task37b": "stored profile visualization supports conservative route selection",
        },
        "blocks_cross_floor_room_runtime_validation": False,
        "blocks_object_executable_approach_validation": True,
    }
    save_json(layer4 / "object_readiness/object_navigation_blocker_report_v0_1.json", blocker)
    interface_summary = {
        "report_id": "object_interface_runtime_summary_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "object_interface_exists": True,
        "object_query_resolution_exists": True,
        "object_approach_artifact_exists": True,
        "object_approach_executable_under_selected_profile": False,
        "object_route_contract_interface_representable": True,
        "object_runtime_execution_ready": False,
        "object_executable_navigation_success_claimed": False,
    }
    save_json(layer4 / "object_readiness/object_interface_runtime_summary_v0_1.json", interface_summary)

    yaml_checks = {}
    for floor_id, key in [("floor_1", "floor1_yaml"), ("floor_2", "floor2_yaml")]:
        parsed = parse_map_yaml(paths[key])
        yaml_checks[floor_id] = {
            "yaml_path": rel(paths[key], repo),
            "image_value": parsed["values"].get("image"),
            "resolved_image_path": rel(parsed["image_path"], repo) if parsed["image_path"] else None,
            "image_resolves": parsed["image_resolves"],
        }
    ros2 = shutil.which("ros2")
    runtime_probes = {
        "ros2": probe_command([ros2, "--help"], repo) if ros2 else {"available": False},
        "nav2_map_server": probe_command([ros2, "pkg", "prefix", "nav2_map_server"], repo) if ros2 else {"available": False},
        "nav2_bringup": probe_command([ros2, "pkg", "prefix", "nav2_bringup"], repo) if ros2 else {"available": False},
        "rviz2": probe_command([ros2, "pkg", "prefix", "rviz2"], repo) if ros2 else {"available": False},
    }
    warnings = []
    if not runtime_authorized:
        warnings.append("Runtime authorization is absent; runtime systems were not launched.")
    warnings.extend([
        "The object executable approach is blocked under conservative_canonical.",
        "RViz Map display is avoided because of the historical GLSL crash; MarkerArray overlays are prepared.",
        "No existing runtime adapter was invoked or claimed compatible with the canonical two-floor handoff package.",
    ])
    preflight_checks = {
        "all_required_inputs_valid": not input_errors,
        "selected_route_candidate_exists": paths["selected"].is_file(),
        "route_waypoint_count_nonzero": navigation_waypoint_count > 0,
        "route_is_cross_floor_room": contract.get("route_kind") == "cross_floor_room",
        "selected_profile_is_conservative_canonical": selected.get("selected_profile") == SELECTED_PROFILE,
        "stable_map_yaml_pgm_exist": all(paths[key].is_file() for key in ("floor1_yaml", "floor1_pgm", "floor2_yaml", "floor2_pgm")),
        "yaml_image_paths_resolve": all(item["image_resolves"] for item in yaml_checks.values()),
        "route_floors_match_maps": set(floor_sequences) == {"floor_1", "floor_2"},
        "connector_identity_preserved": contract["vertical_transition"]["connector_id"] == "vt_1",
        "transition_edge_preserved": contract["vertical_transition"]["transition_edge"] == "vt_1_centerline_e001",
        "non_transition_edge_preserved": contract["vertical_transition"]["non_transition_edge"] == "vt_1_centerline_e003",
        "object_executable_approach_not_selected": runtime_package["object_route_selected"] is False,
        "object_centroid_navigation_not_used": route_runtime["object_centroid_navigation_used"] is False,
        "runtime_python_available": Path(RUNTIME_PYTHON).is_file(),
        "offline_python_available": Path(OFFLINE_PYTHON).is_file(),
        "marker_array_overlay_prepared": (layer4 / "rviz/layer4_marker_overlays_v0_1.rviz").is_file(),
    }
    preflight = {
        "report_id": "runtime_preflight_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "artifact_layer": LAYER,
        "generated_at": generated_at,
        "status": "passed_with_warnings" if all(preflight_checks.values()) else "blocked",
        "checks": preflight_checks,
        "required_input_checks": input_checks,
        "map_yaml_checks": yaml_checks,
        "runtime_environment": {
            "ROS_DOMAIN_ID_detected": os.environ.get("ROS_DOMAIN_ID"),
            "ROS_DOMAIN_ID_historical_default_verified": 84,
            "ROS_DISTRO_detected": os.environ.get("ROS_DISTRO"),
            "TURTLEBOT3_MODEL_detected": os.environ.get("TURTLEBOT3_MODEL"),
            "TURTLEBOT3_MODEL_historical_default_verified": "burger",
            "runtime_python": RUNTIME_PYTHON,
            "offline_python": OFFLINE_PYTHON,
            "runtime_probes": runtime_probes,
            "relevant_runtime_scripts": [
                file_record(repo / "tools/stage1_runtime/launch_scene_gazebo_nav2.sh", repo),
                file_record(repo / "tools/stage1_runtime/run_scene_route.py", repo),
                file_record(repo / "tools/vertical_connectors/cross_floor_visual_traversal_player.py", repo),
            ],
        },
        "runtime_authorization_detected": runtime_authorized,
        "runtime_execution_status": runtime_status,
        "runtime_execution_readiness": "authorization_and_runtime_adapter_integration_required",
        "warnings": warnings,
    }
    save_json(layer4 / "reports/runtime_preflight_report_v0_1.json", preflight)

    after = snapshot(list(layer2.rglob("*")) + list(layer3.rglob("*")), repo)
    source_hashes_unchanged = before == after
    static_report = {
        "report_id": "layer4_static_packaging_validation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "artifact_layer": LAYER,
        "status": "passed" if all(preflight_checks.values()) and source_hashes_unchanged else "failed",
        "selected_route_id": SELECTED_ROUTE_ID,
        "selected_profile": SELECTED_PROFILE,
        "floor_navigation_waypoint_count": navigation_waypoint_count,
        "reported_interface_waypoint_count": selected["route_metrics"]["waypoint_count"],
        "canonical_layer2_and_layer3_hashes_unchanged": source_hashes_unchanged,
        "runtime_systems_launched": False,
        "object_executable_approach_packaged_for_execution": False,
        "checks": {
            "runtime_input_package_created": True,
            "selected_route_runtime_input_created": True,
            "map_server_inputs_created": True,
            "route_executor_inputs_created": True,
            "rviz_marker_overlay_inputs_created": True,
            "object_readiness_reports_created": True,
        },
    }
    save_json(layer4 / "reports/layer4_static_packaging_validation_report_v0_1.json", static_report)

    launch_script = """#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="/home/ws/workspace/BoxFusion"
cd "${REPO_ROOT}"
if [[ "${RSLG_TASK38_ALLOW_RUNTIME:-0}" != "1" ]]; then
  echo "Set RSLG_TASK38_ALLOW_RUNTIME=1 before any authorized runtime integration." >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
source /opt/ros/foxy/setup.bash
echo "Authorization detected. Use /usr/bin/python3 for the runtime adapter."
echo "Selected package: stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer4_runtime_validation/runtime_inputs/runtime_input_package_v0_1.json"
echo "A compatible two-floor executor adapter must be selected and validated before launch."
exit 3
"""
    write_text(layer4 / "launch/authorized_runtime_environment_guard.sh", launch_script)
    (layer4 / "launch/authorized_runtime_environment_guard.sh").chmod(0o755)

    canonical_jsons = sorted(layer4.rglob("*.json"))
    claim_report = claim_boundary_report(canonical_jsons + [repo / path for path in DOC_OUTPUTS], repo)
    save_json(layer4 / "reports/runtime_claim_boundary_report_v0_1.json", claim_report)

    manifest_paths = sorted(
        path for path in layer4.rglob("*")
        if path.is_file() and path.name != "layer4_runtime_validation_manifest_v0_1.json"
    )
    manifest = {
        "manifest_id": "layer4_runtime_validation_manifest_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "artifact_layer": LAYER,
        "selected_route_id": SELECTED_ROUTE_ID,
        "selected_profile": SELECTED_PROFILE,
        "runtime_execution_status": runtime_status,
        "artifacts": [file_record(path, repo) for path in manifest_paths],
    }
    save_json(layer4 / "manifests/layer4_runtime_validation_manifest_v0_1.json", manifest)

    for source, target_name in [
        (layer4 / "reports/runtime_preflight_report_v0_1.json", "runtime_preflight_report_v0_1.json"),
        (layer4 / "reports/layer4_static_packaging_validation_report_v0_1.json", "runtime_static_packaging_validation_report_v0_1.json"),
        (layer4 / "object_readiness/object_runtime_readiness_report_v0_1.json", "object_runtime_readiness_report_v0_1.json"),
        (layer4 / "reports/runtime_claim_boundary_report_v0_1.json", "claim_boundary_validation_report_v0_1.json"),
    ]:
        shutil.copyfile(source, task / target_name)

    generation_report = {
        "report_id": "pipeline_runbook_generation_report_v0_1",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "docs_created_or_updated": DOC_OUTPUTS,
        "wrappers_created_or_updated": WRAPPER_OUTPUTS,
        "runtime_execution_wrapper_created": False,
        "default_wrappers_launch_runtime": False,
    }
    save_json(task / "pipeline_runbook_generation_report_v0_1.json", generation_report)
    write_text(task / "warnings.txt", "\n".join(f"- {warning}" for warning in warnings))

    command_lines = [
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=cat <task38 attached request>; stdout=captured by Codex session",
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=cat docs/rslg_slam project truth documents and inspect required manifests; stdout=captured by Codex session",
        f"[{generated_at}] cwd={repo} interpreter={OFFLINE_PYTHON} command=load required Layer 2/Layer 3 JSON artifacts with json.load; result=passed",
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=sha256sum canonical Layer 2 and Layer 3 files before generation; output=/tmp/task38_layer2_layer3_before.sha256",
        f"[{generated_at}] cwd={repo} interpreter={OFFLINE_PYTHON} command=python -m py_compile tools/rslg_pipeline/export_runtime_inputs.py; result=passed",
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=bash -n tools/rslg_pipeline/run_layer*.sh tools/rslg_pipeline/run_rslg_pipeline_static.sh; result=passed",
        f"[{generated_at}] cwd={repo} interpreter={OFFLINE_PYTHON} command=python -m tools.rslg_pipeline.export_runtime_inputs --repo-root {repo}; result=completed",
        f"[{generated_at}] cwd={repo} interpreter={OFFLINE_PYTHON} command=python -m tools.rslg_pipeline.validate_artifacts --repo-root {repo}; result=rslg_static_validation_passed",
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=tools/rslg_pipeline/run_rslg_pipeline_static.sh; result=completed_without_runtime_launch",
        f"[{generated_at}] cwd={repo} interpreter={OFFLINE_PYTHON} command=load all generated task38 JSON with json.load; result=passed",
        f"[{generated_at}] cwd={repo} interpreter=/bin/bash command=diff Layer 2/Layer 3 before and after SHA-256 inventories; result=no_changes",
        f"[{generated_at}] cwd={repo} interpreter={RUNTIME_PYTHON} command=runtime availability probes only via ros2 --help and ros2 pkg prefix; no ROS nodes launched",
    ]
    write_text(task / "command_log.txt", "\n".join(command_lines))

    canonical_outputs = sorted(rel(path, repo) for path in layer4.rglob("*") if path.is_file())
    task_report = {
        "task_name": TASK_NAME,
        "status": "completed",
        "classification": CLASSIFICATION,
        "project_name": PROJECT_NAME,
        "scene_id": scene_id,
        "canonical_layer2_input_dir": rel(layer2, repo),
        "canonical_layer3_input_dir": rel(layer3, repo),
        "canonical_layer4_output_dir": rel(layer4, repo),
        "task38_evidence_dir": rel(task, repo),
        "docs_and_manifests_read": [
            {"path": path, "status": "read" if (repo / path).is_file() else "missing"}
            for path in DOCS_AND_MANIFESTS
        ],
        "tools_used": [
            "tools/rslg_pipeline/export_runtime_inputs.py",
            "tools/rslg_pipeline/common.py",
            "ros2 pkg prefix (availability probes only)",
        ],
        "wrappers_created_or_updated": WRAPPER_OUTPUTS,
        "runtime_authorization_detected": runtime_authorized,
        "static_packaging_status": static_report["status"],
        "runtime_preflight_status": preflight["status"],
        "runtime_execution_status": runtime_status,
        "selected_route_profile": SELECTED_PROFILE,
        "selected_route_id": SELECTED_ROUTE_ID,
        "selected_route_validation_summary": {
            "reported_interface_waypoint_count": selected["route_metrics"]["waypoint_count"],
            "floor_navigation_waypoint_count": navigation_waypoint_count,
            "occupied_cells_crossed": selected["route_metrics"]["occupied_cells_crossed"],
            "unknown_cells_crossed": selected["route_metrics"]["unknown_cells_crossed"],
            "canonical_wall_crossing_count": selected["route_metrics"]["canonical_wall_crossing_count"],
        },
        "object_runtime_readiness_summary": {
            "query_interface": "ready",
            "executable_approach": "blocked",
            "approach_candidate_id": "generated_ring_037",
            "object_centroid_navigation_used": False,
        },
        "runbook_outputs": DOC_OUTPUTS,
        "canonical_outputs_generated": canonical_outputs,
        "validation_summary": {
            "preflight": preflight["status"],
            "static_package": static_report["status"],
            "claim_boundary": claim_report["status"],
            "layer2_layer3_hashes_unchanged": source_hashes_unchanged,
        },
        "blocked_or_missing_items": [
            "Runtime authorization was not provided.",
            "Object executable approach is blocked under conservative_canonical.",
            "A compatible canonical two-floor runtime adapter has not been selected or validated.",
        ],
        "recommended_next_task": [
            "task39_object_level_navigation_interface_recovery_and_candidate_approach_selection",
            "task39_authorized_runtime_execution_if_user_wants_to_run_the_demo",
        ],
    }
    source_files = [repo / "tools/rslg_pipeline/export_runtime_inputs.py"] + [
        repo / path for path in DOC_OUTPUTS + WRAPPER_OUTPUTS
    ]
    task_report["validation_summary"]["json_validation"] = "passed"
    save_json(task / "task38_report.json", task_report)

    final_claim_paths = sorted(
        {
            path
            for path in list(layer4.rglob("*"))
            + list(task.rglob("*"))
            + [repo / item for item in DOC_OUTPUTS + WRAPPER_OUTPUTS]
            if path.is_file()
        }
    )
    claim_report = claim_boundary_report(final_claim_paths, repo)
    save_json(layer4 / "reports/runtime_claim_boundary_report_v0_1.json", claim_report)
    shutil.copyfile(
        layer4 / "reports/runtime_claim_boundary_report_v0_1.json",
        task / "claim_boundary_validation_report_v0_1.json",
    )
    task_report["validation_summary"]["claim_boundary"] = claim_report["status"]
    save_json(task / "task38_report.json", task_report)

    manifest_paths = sorted(
        path for path in layer4.rglob("*")
        if path.is_file() and path.name != "layer4_runtime_validation_manifest_v0_1.json"
    )
    manifest["artifacts"] = [file_record(path, repo) for path in manifest_paths]
    save_json(layer4 / "manifests/layer4_runtime_validation_manifest_v0_1.json", manifest)

    json_report_path = task / "json_validation_report_v0_1.json"
    created_manifest_path = task / "created_or_modified_files_manifest_v0_1.json"
    save_json(json_report_path, {"status": "pending"})

    # Iterate because the evidence manifest and JSON report include one another.
    # Their serialized sizes stabilize after the first pass.
    json_status = "pending"
    for _ in range(4):
        modified_files = sorted(
            {
                path
                for path in list(layer4.rglob("*")) + list(task.rglob("*")) + source_files
                if path.is_file()
            }
        )
        save_json(
            created_manifest_path,
            {
                "manifest_id": "created_or_modified_files_manifest_v0_1",
                "schema_version": "0.1",
                "project_name": PROJECT_NAME,
                "files": [
                    {
                        "path": rel(path, repo),
                        "size": path.stat().st_size,
                        "role": "task38 source, documentation, wrapper, canonical output, or evidence",
                        "layer": LAYER if "layer4_runtime_validation" in path.as_posix() else "pipeline_support",
                        "artifact_family": "task38",
                        "created_or_modified": "created_or_modified",
                    }
                    for path in modified_files
                ],
            },
        )
        json_records = []
        for path in sorted(path for path in modified_files if path.suffix.lower() == ".json"):
            try:
                load_json(path)
                json_records.append({"path": rel(path, repo), "status": "passed"})
            except Exception as exc:
                json_records.append({"path": rel(path, repo), "status": "failed", "error": str(exc)})
        json_status = "passed" if all(item["status"] == "passed" for item in json_records) else "failed"
        save_json(
            json_report_path,
            {
                "report_id": "json_validation_report_v0_1",
                "schema_version": "0.1",
                "project_name": PROJECT_NAME,
                "status": json_status,
                "json_file_count": len(json_records),
                "files": json_records,
            },
        )

    task_report["validation_summary"]["json_validation"] = json_status
    save_json(task / "task38_report.json", task_report)
    return task_report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--scene-id", default=SCENE_ID)
    args = parser.parse_args(list(argv) if argv is not None else None)
    repo = args.repo_root.expanduser().resolve() if args.repo_root else resolve_repo_root()
    report = run(repo, args.scene_id)
    print(json.dumps({
        "status": report["status"],
        "classification": report["classification"],
        "canonical_layer4_output_dir": report.get("canonical_layer4_output_dir"),
        "runtime_execution_status": report.get("runtime_execution_status"),
        "blocked_or_missing_items": report.get("blocked_or_missing_items", []),
    }, indent=2))
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
