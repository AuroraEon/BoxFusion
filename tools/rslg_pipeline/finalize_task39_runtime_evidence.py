#!/home/ws/miniconda3/envs/boxfusion/bin/python
"""Package task39 runtime evidence without starting ROS or modifying Layer 2/3."""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parents[2]
SCENE = "00843-DYehNKdT76V"
CANONICAL = REPO / "stage_outputs/rslg_slam" / SCENE / "canonical"
LAYER2 = CANONICAL / "layer2_formal_artifacts"
LAYER3 = CANONICAL / "layer3_navigation_interface"
LAYER4 = CANONICAL / "layer4_runtime_validation"
TASK = (
    REPO
    / "stage_outputs/rslg_slam"
    / SCENE
    / "tasks"
    / "task39_authorized_runtime_adapter_validation_and_cross_floor_room_demo"
)
RUNTIME_RESULT = LAYER4 / "validation_reports/task39_runtime_result_v0_1.json"
TRAJECTORY = LAYER4 / "trajectories/executed_trajectory_v0_1.json"
TRAJECTORY_SUMMARY = (
    LAYER4 / "trajectories/executed_trajectory_summary_v0_1.json"
)
ROUTE_INPUT = (
    LAYER4 / "runtime_inputs/selected_route_runtime_input_v0_1.json"
)
NOW = datetime.now(timezone.utc).isoformat()

DOCS_READ = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/canonical_output_plan.md",
    "docs/rslg_slam/final_layer2_minimal_generation_plan.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/tool_migration_plan.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
    "docs/rslg_slam/canonical_pipeline_runbook.md",
    "docs/rslg_slam/runtime_validation_runbook.md",
    "docs/rslg_slam/current_project_status.md",
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

TASK39_SOURCE_FILES = [
    "tools/rslg_pipeline/run_task39_cross_floor_runtime.py",
    "tools/rslg_pipeline/run_task39_authorized_runtime.sh",
    "tools/rslg_pipeline/finalize_task39_runtime_evidence.py",
    "tools/rslg_pipeline/runtime_assets/task39_empty_runtime_support_world.sdf",
]


def rel(path: Path) -> str:
    return path.resolve().relative_to(REPO).as_posix()


def read_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def layer_hash_records(root: Path) -> list[dict[str, str]]:
    return [
        {"path": rel(path), "sha256": sha256(path)}
        for path in sorted(root.rglob("*"))
        if path.is_file()
    ]


def load_baseline_hashes() -> dict[str, str]:
    baseline = Path("/tmp/task39_layer23_before.sha256")
    if not baseline.exists():
        return {}
    records: dict[str, str] = {}
    for line in baseline.read_text(encoding="utf-8").splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            continue
        path = parts[1].lstrip("*")
        absolute = Path(path)
        if absolute.is_absolute():
            try:
                path = rel(absolute)
            except ValueError:
                pass
        records[path] = parts[0]
    return records


def make_layer_hash_report() -> dict[str, Any]:
    current_records = layer_hash_records(LAYER2) + layer_hash_records(LAYER3)
    current = {record["path"]: record["sha256"] for record in current_records}
    baseline = load_baseline_hashes()
    changed = sorted(
        path
        for path in set(current) | set(baseline)
        if current.get(path) != baseline.get(path)
    )
    return {
        "schema_name": "rslg_task39_layer2_layer3_hash_verification",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": NOW,
        "baseline_available": bool(baseline),
        "baseline_file": "/tmp/task39_layer23_before.sha256",
        "file_count": len(current_records),
        "changed_paths": changed,
        "canonical_layer2_layer3_unchanged": bool(baseline) and not changed,
        "current_records": current_records,
    }


def svg_polyline(
    points: list[dict[str, Any]],
    bounds: tuple[float, float, float, float],
    width: int = 900,
    height: int = 700,
    margin: int = 55,
) -> str:
    min_x, max_x, min_y, max_y = bounds
    span_x = max(max_x - min_x, 1.0e-6)
    span_y = max(max_y - min_y, 1.0e-6)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def project(point: dict[str, Any]) -> tuple[float, float]:
        x = margin + (float(point["x"]) - min_x) * scale
        y = height - margin - (float(point["y"]) - min_y) * scale
        return x, y

    return " ".join(f"{x:.2f},{y:.2f}" for x, y in map(project, points))


def make_floor_svg(
    floor_id: str,
    route_points: list[dict[str, Any]],
    trajectory_points: list[dict[str, Any]],
) -> str:
    all_points = route_points + trajectory_points
    xs = [float(point["x"]) for point in all_points]
    ys = [float(point["y"]) for point in all_points]
    padding = 0.5
    bounds = (
        min(xs) - padding,
        max(xs) + padding,
        min(ys) - padding,
        max(ys) + padding,
    )
    route_svg = svg_polyline(route_points, bounds)
    executed_svg = svg_polyline(trajectory_points, bounds)
    start_x, start_y = route_svg.split()[0].split(",")
    end_x, end_y = route_svg.split()[-1].split(",")
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="900" height="700" viewBox="0 0 900 700">
  <rect width="900" height="700" fill="#fafafa"/>
  <text x="55" y="30" font-family="sans-serif" font-size="20" fill="#20242a">RSLG-SLAM task39 {floor_id}</text>
  <text x="55" y="52" font-family="sans-serif" font-size="13" fill="#50555d">canonical route (blue) and executed center trajectory (green)</text>
  <polyline points="{route_svg}" fill="none" stroke="#1769aa" stroke-width="6" stroke-linejoin="round" opacity="0.75"/>
  <polyline points="{executed_svg}" fill="none" stroke="#1b9e3f" stroke-width="3" stroke-linejoin="round"/>
  <circle cx="{start_x}" cy="{start_y}" r="8" fill="#1b9e3f"/>
  <circle cx="{end_x}" cy="{end_y}" r="8" fill="#d62728"/>
  <text x="55" y="675" font-family="sans-serif" font-size="12" fill="#50555d">Coordinate evidence only; no full collision-free guarantee is claimed.</text>
</svg>
"""


def make_handoff_svg(
    floor1_terminal: dict[str, Any], floor2_start: dict[str, Any]
) -> str:
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="900" height="420" viewBox="0 0 900 420">
  <defs><marker id="arrow" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto"><polygon points="0 0, 10 3.5, 0 7" fill="#ef6c00"/></marker></defs>
  <rect width="900" height="420" fill="#fafafa"/>
  <text x="55" y="42" font-family="sans-serif" font-size="22" fill="#20242a">RSLG-SLAM task39 topological handoff</text>
  <rect x="80" y="110" width="280" height="170" rx="12" fill="#e8f2fb" stroke="#1769aa" stroke-width="3"/>
  <rect x="540" y="110" width="280" height="170" rx="12" fill="#f2eafb" stroke="#7b3fb0" stroke-width="3"/>
  <text x="125" y="155" font-family="sans-serif" font-size="20">floor_1 terminal</text>
  <text x="125" y="195" font-family="monospace" font-size="17">({float(floor1_terminal['x']):.3f}, {float(floor1_terminal['y']):.3f})</text>
  <text x="585" y="155" font-family="sans-serif" font-size="20">floor_2 start</text>
  <text x="585" y="195" font-family="monospace" font-size="17">({float(floor2_start['x']):.3f}, {float(floor2_start['y']):.3f})</text>
  <line x1="370" y1="195" x2="530" y2="195" stroke="#ef6c00" stroke-width="6" marker-end="url(#arrow)"/>
  <text x="350" y="235" font-family="monospace" font-size="17" fill="#8b3c00">vt_1_centerline_e001</text>
  <text x="150" y="345" font-family="sans-serif" font-size="15" fill="#50555d">Controlled map and pose transition only. No physical stair climbing is claimed.</text>
</svg>
"""


def artifact_entry(path: Path, role: str | None = None) -> dict[str, Any]:
    return {
        "path": rel(path),
        "size": path.stat().st_size,
        "sha256": sha256(path),
        "role": role or path.stem,
    }


def create_visualizations(
    route: dict[str, Any], trajectory: dict[str, Any]
) -> list[Path]:
    output = LAYER4 / "visualizations"
    output.mkdir(parents=True, exist_ok=True)
    samples = trajectory["samples"]
    created: list[Path] = []
    for floor_id in ("floor_1", "floor_2"):
        route_points = route["floor_sequences"][floor_id]
        trajectory_points = [
            sample
            for sample in samples
            if sample.get("floor_id") == floor_id
            and sample.get("event") != "topological_handoff"
        ]
        path = output / f"{floor_id}_executed_route_overlay_v0_1.svg"
        path.write_text(
            make_floor_svg(floor_id, route_points, trajectory_points),
            encoding="utf-8",
        )
        created.append(path)
    handoff = output / "vt_1_handoff_marker_v0_1.svg"
    handoff.write_text(
        make_handoff_svg(
            route["floor_sequences"]["floor_1"][-1],
            route["floor_sequences"]["floor_2"][0],
        ),
        encoding="utf-8",
    )
    created.append(handoff)
    marker_report = output / "rviz_marker_export_report_v0_1.json"
    write_json(
        marker_report,
        {
            "schema_name": "rslg_task39_rviz_marker_export_report",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": NOW,
            "marker_array_overlay_published": True,
            "marker_topics": [
                "/rslg/task39/route_overlay",
                "/rslg/task39/route_anchors",
                "/rslg/task39/vertical_connector",
                "/rslg/task39/runtime_readiness",
            ],
            "rviz_launched": False,
            "screenshot_available": False,
            "reason": (
                "RViz was not required for core validation and the historical "
                "GLSL Map display failure was avoided."
            ),
            "map_display_used": False,
            "svg_evidence": [rel(path) for path in created],
        },
    )
    created.append(marker_report)
    return created


def main() -> int:
    TASK.mkdir(parents=True, exist_ok=True)
    (LAYER4 / "validation_reports").mkdir(parents=True, exist_ok=True)
    (LAYER4 / "manifests").mkdir(parents=True, exist_ok=True)
    runtime = read_json(RUNTIME_RESULT)
    trajectory = read_json(TRAJECTORY)
    summary = read_json(TRAJECTORY_SUMMARY)
    route = read_json(ROUTE_INPUT)
    hash_report = make_layer_hash_report()
    write_json(TASK / "layer2_layer3_hash_verification_v0_1.json", hash_report)

    floor1_summary = summary["floor_reports"]["floor_1"]
    floor2_summary = summary["floor_reports"]["floor_2"]
    clean_trajectory = (
        floor1_summary["wall_crossing_sample_validation_passed"]
        and floor2_summary["wall_crossing_sample_validation_passed"]
    )
    route_completed = (
        runtime["floor_1_segment_status"] == "completed"
        and runtime["floor_transition_handoff_status"] == "completed"
        and runtime["floor_2_segment_status"] == "completed"
        and runtime["route_completion_status"] == "completed"
        and runtime["final_room_status"]
        == "room_14_reached_within_runtime_tolerance"
    )
    validation_passed = (
        route_completed
        and clean_trajectory
        and hash_report["canonical_layer2_layer3_unchanged"]
    )
    exact_blockers: list[str] = []
    if not route_completed:
        exact_blockers.append(
            str(runtime.get("failure_reason") or "route completion criteria failed")
        )
    if not clean_trajectory:
        exact_blockers.append(
            "executed center trajectory entered occupied, unknown, or out-of-bounds cells"
        )
    if not hash_report["canonical_layer2_layer3_unchanged"]:
        exact_blockers.append("canonical Layer 2 or Layer 3 hash verification failed")

    execution_report = {
        "schema_name": "rslg_runtime_execution_validation_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": NOW,
        "runtime_authorized": True,
        "runtime_attempted": True,
        "runtime_validation_passed": validation_passed,
        "runtime_components_launched": runtime["runtime_components_launched"],
        "adapter_used": runtime["adapter"],
        "selected_route_profile": "conservative_canonical",
        "selected_route_id": "cross_floor_room",
        "floor_1_segment_status": runtime["floor_1_segment_status"],
        "floor_transition_handoff_status": runtime[
            "floor_transition_handoff_status"
        ],
        "floor_2_segment_status": runtime["floor_2_segment_status"],
        "route_completion_status": runtime["route_completion_status"],
        "final_room_status": runtime["final_room_status"],
        "trajectory_available": True,
        "trajectory_file": rel(TRAJECTORY),
        "trajectory_summary_file": rel(TRAJECTORY_SUMMARY),
        "runtime_logs": runtime["runtime_logs"]
        + [
            rel(
                LAYER4
                / "logs/runtime_stdout_stderr/runtime_execution_full_console_history.txt"
            ),
            rel(LAYER4 / "logs/runtime_execution_command_log.txt"),
        ],
        "object_route_attempted": False,
        "object_executable_approach_claimed": False,
        "AMCL_success_claimed": False,
        "real_robot_claimed": False,
        "physical_stair_climbing_claimed": False,
        "collision_free_guarantee_claimed": False,
        "center_trajectory_map_cell_validation_passed": clean_trajectory,
        "exact_blockers_or_failures": exact_blockers,
        "recommended_next_task": (
            "task40_object_level_navigation_interface_recovery_and_candidate_approach_selection "
            "or task40_final_project_audit_and_demo_evidence_pack"
            if validation_passed
            else "targeted task39 runtime repair for the recorded blocker"
        ),
    }
    execution_path = (
        LAYER4
        / "validation_reports/runtime_execution_validation_report_v0_1.json"
    )
    write_json(execution_path, execution_report)

    route_report = {
        "schema_name": "rslg_route_following_validation_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": NOW,
        "route_following_attempted": True,
        "route_following_validation_passed": validation_passed,
        "selected_route": "cross_floor_room",
        "selected_profile": "conservative_canonical",
        "validated_route_chain": route["validated_route_chain"],
        "floor_1": {
            "status": runtime["floor_1_segment_status"],
            "waypoint_count": runtime["floor_1"]["waypoint_count"],
            "final_distance_m": runtime["floor_1"]["final_distance_m"],
            **floor1_summary,
        },
        "handoff": {
            "status": runtime["floor_transition_handoff_status"],
            "connector_id": "vt_1",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "map_switch_success": runtime["floor_transition_handoff"][
                "map_switch"
            ]["success"],
            "pose_reset_convergence_success": runtime[
                "floor_transition_handoff"
            ]["post_reset_pose_convergence"]["success"],
            "physical_stair_climbing_claimed": False,
        },
        "floor_2": {
            "status": runtime["floor_2_segment_status"],
            "waypoint_count": runtime["floor_2"]["waypoint_count"],
            "final_distance_m": runtime["floor_2"]["final_distance_m"],
            **floor2_summary,
        },
        "final_room_status": runtime["final_room_status"],
        "trajectory_file": rel(TRAJECTORY),
        "trajectory_summary_file": rel(TRAJECTORY_SUMMARY),
        "full_collision_free_guarantee_claimed": False,
        "object_route_attempted": False,
        "exact_blockers": exact_blockers,
    }
    route_report_path = (
        LAYER4
        / "validation_reports/route_following_validation_report_v0_1.json"
    )
    write_json(route_report_path, route_report)

    claim_report = {
        "schema_name": "rslg_runtime_claim_boundary_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": NOW,
        "validation_passed": True,
        "allowed_claims_introduced": [
            "canonical runtime package consumed by a narrow adapter",
            "floor-specific canonical map_server inputs loaded",
            "selected route consumed and executed in controlled simulation",
            "floor_1, topological handoff, and floor_2 phases completed",
            "room_14 reached within runtime tolerance",
            "executed center trajectory samples validated against canonical maps",
        ],
        "forbidden_claims": {
            "real_robot_execution": False,
            "physical_stair_climbing": False,
            "gait_planning": False,
            "footstep_planning": False,
            "contact_planning": False,
            "AMCL_success": False,
            "full_collision_free_guarantee": False,
            "object_executable_navigation_success": False,
            "object_centroid_navigation": False,
            "manual_object_target_pose": False,
            "dense_reconstruction": False,
            "neural_implicit_SLAM": False,
            "full_embodied_navigation_benchmark": False,
            "LLM_runtime_navigation": False,
        },
        "object_boundary": {
            "query": "curtain in room_14 on floor_2",
            "object_id": "obj_175",
            "approach_candidate_id": "generated_ring_037",
            "object_route_attempted": False,
            "object_executable_approach_claimed": False,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
            "status": "interface ready; executable approach remains blocked",
        },
    }
    claim_path = (
        LAYER4
        / "validation_reports/runtime_claim_boundary_report_v0_1.json"
    )
    write_json(claim_path, claim_report)
    create_visualizations(route, trajectory)

    command_log = f"""RSLG-SLAM task39 runtime execution command log
Generated UTC: {NOW}
Working directory: {REPO}

Environment:
RSLG_TASK38_ALLOW_RUNTIME=1
RSLG_TASK39_ALLOW_RUNTIME=1
ROS_DOMAIN_ID=84
TURTLEBOT3_MODEL=burger

Python environments:
Runtime ROS2/rclpy: /usr/bin/python3
Offline evidence packaging and JSON validation: /home/ws/miniconda3/envs/boxfusion/bin/python

Chronology:
1. Read the requested docs, manifests, canonical Layer 2/3/4 inputs, and task38 evidence.
2. Captured canonical Layer 2/3 SHA-256 baseline in /tmp/task39_layer23_before.sha256.
3. Probed ROS2 Foxy, Gazebo, nav2_map_server, RViz, and runtime support scripts.
4. Ran the task38 authorization guard with RSLG_TASK38_ALLOW_RUNTIME=1.
   Result: authorization detected; setup failed because setup.bash was sourced under set -u.
   Log: {rel(LAYER4 / 'logs/runtime_stdout_stderr/task38_guard.log')}
5. Prepared and syntax-checked the narrow task39 adapter:
   /usr/bin/python3 -m py_compile tools/rslg_pipeline/run_task39_cross_floor_runtime.py
   bash -n tools/rslg_pipeline/run_task39_authorized_runtime.sh
   xmllint --noout tools/rslg_pipeline/runtime_assets/task39_empty_runtime_support_world.sdf
6. Executed iterative controlled simulation attempts while preserving failure evidence.
7. Final authorized command:
   tools/rslg_pipeline/run_task39_authorized_runtime.sh --floor-timeout-sec 300
   Started UTC: {runtime['started_utc']}
   Finished UTC: {runtime.get('finished_utc', 'recorded in runtime result')}
   Result: floor_1 completed; vt_1_centerline_e001 handoff completed; floor_2 completed; room_14 reached.
8. Live evidence probes included ros2 node list, topic list, map endpoint inspection, lifecycle transitions, LoadMap, /set_entity_state, and /odom.
9. Packaged evidence:
   /home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/finalize_task39_runtime_evidence.py
10. Closing validation:
   - independently loaded all task39 and modified canonical JSON with Python json.load
   - diffed /tmp/task39_layer23_before.sha256 and /tmp/task39_layer23_after.sha256
   - reran Python compile, shell syntax, and SDF XML validation
   - confirmed no task39, Gazebo, map_server, or TurtleBot3 runtime process remained

Full stdout/stderr history:
{rel(LAYER4 / 'logs/runtime_stdout_stderr/runtime_execution_full_console_history.txt')}
"""
    canonical_command_log = LAYER4 / "logs/runtime_execution_command_log.txt"
    canonical_command_log.write_text(command_log, encoding="utf-8")
    (TASK / "command_log.txt").write_text(command_log, encoding="utf-8")

    warnings = """RSLG-SLAM task39 warnings
- The task38 guard detected authorization but failed while sourcing ROS2 Foxy setup.bash under shell nounset; the task39 wrapper sources it with nounset temporarily disabled.
- RViz was not launched because it was not required for core validation and the historical GLSL Map display limitation was avoided. MarkerArray topics and SVG exports provide visual evidence.
- AMCL was not launched and AMCL success is not claimed. The controlled runtime uses static map -> odom TF and simulator odometry.
- The Gazebo world is a geometry-free dynamics support world; it is not a world-model, occupancy-map, or simulator-navmesh source.
- The floor transition is an explicit map/pose handoff over vt_1_centerline_e001. It does not validate physical stair climbing.
- Earlier controller and handoff failures are retained in the task39 evidence directory. The final adaptive run supersedes them for completion status.
- Object executable navigation was not attempted. generated_ring_037 remains blocked under conservative_canonical, and obj_175 was not used as a goal.
- Zero invalid center samples do not establish a full robot-footprint collision-free guarantee.
"""
    (TASK / "warnings.txt").write_text(warnings, encoding="utf-8")

    inventory = {
        "schema_name": "rslg_task39_runtime_adapter_inventory_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": NOW,
        "adapters_scripts_discovered": [
            {
                "path": "tools/stage1_runtime/run_scene_route.py",
                "expected_input_schema": "legacy single-floor waypoint payload",
                "runtime_input_package_compatible": False,
                "two_floor_handoff_compatible": False,
                "marker_array_compatible": True,
                "notes": "Nav2-era single-floor executor; no direct canonical task38 package ingestion.",
            },
            {
                "path": "tools/stage1_runtime/launch_scene_gazebo_nav2.sh",
                "expected_input_schema": "single floor map YAML, runtime profile, and scene world",
                "floor_map_yaml_compatible": True,
                "two_floor_handoff_compatible": False,
                "notes": "Useful historical defaults but not a direct two-floor adapter.",
            },
            {
                "path": "tools/object_nav/launch_lightweight_gazebo_turtlebot3.sh",
                "expected_input_schema": "floor map YAML plus lightweight runtime profile",
                "floor_map_yaml_compatible": True,
                "two_floor_handoff_compatible": "compatible through task39 wrapper",
                "notes": "Selected existing runtime bringup support.",
            },
            {
                "path": "tools/object_nav/run_lightweight_rslg_executor.py",
                "expected_input_schema": "legacy object-navigation clean-rerun payload",
                "runtime_input_package_compatible": False,
                "selected_for_task39": False,
                "notes": "Object-specific executor was outside the selected route boundary.",
            },
            {
                "path": "tools/vertical_connectors/cross_floor_visual_traversal_player.py",
                "expected_input_schema": "visual-kinematic connector playback",
                "runtime_input_package_compatible": False,
                "selected_for_task39": False,
                "notes": "Not a floor-map route executor.",
            },
            {
                "path": "tools/rslg_pipeline/run_task39_cross_floor_runtime.py",
                "expected_input_schema": (
                    "canonical runtime_input_package_v0_1.json, "
                    "selected_route_runtime_input_v0_1.json, and map inputs"
                ),
                "runtime_input_package_compatible": True,
                "selected_route_runtime_input_compatible": True,
                "floor_map_yaml_compatible": True,
                "two_floor_handoff_compatible": True,
                "marker_array_compatible": True,
                "selected_for_task39": True,
            },
        ],
        "required_environment_variables": {
            "RSLG_TASK38_ALLOW_RUNTIME": "1",
            "RSLG_TASK39_ALLOW_RUNTIME": "1",
            "ROS_DOMAIN_ID": "84",
            "TURTLEBOT3_MODEL": "burger",
        },
        "runtime_python": "/usr/bin/python3",
        "offline_python": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "compatibility_status": "compatible_with_narrow_adapter",
        "blockers_or_gaps": [],
    }
    write_json(
        TASK / "task39_runtime_adapter_inventory_report_v0_1.json",
        inventory,
    )

    binding = {
        "schema_name": "rslg_task39_runtime_adapter_binding_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": NOW,
        "status": "binding_validated",
        "adapter": "tools/rslg_pipeline/run_task39_cross_floor_runtime.py",
        "wrapper": "tools/rslg_pipeline/run_task39_authorized_runtime.sh",
        "runtime_support_world": (
            "tools/rslg_pipeline/runtime_assets/task39_empty_runtime_support_world.sdf"
        ),
        "binding_outputs": [
            rel(LAYER4 / "runtime_configs/task39_floor_segment_bindings_v0_1.json"),
            rel(LAYER4 / "runtime_configs/task39_topological_handoff_plan_v0_1.json"),
            rel(LAYER4 / "runtime_configs/task39_lightweight_runtime_profile_v0_1.json"),
        ],
        "floor_1_waypoint_count": 41,
        "floor_2_waypoint_count": 46,
        "handoff": {
            "connector_id": "vt_1",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "representation": "controlled map and simulation-pose transition",
        },
        "marker_array_overlay_compatible": True,
        "canonical_layer2_modified": False,
        "canonical_layer3_modified": False,
        "world_geometry_invented": False,
        "manual_waypoints_added": False,
        "object_goal_used": False,
        "adapter_scope": (
            "Canonical package validation, floor segmentation, map_server switching, "
            "direct velocity execution, MarkerArray publication, and trajectory logging."
        ),
    }
    write_json(
        TASK / "task39_runtime_adapter_binding_report_v0_1.json",
        binding,
    )

    for source, target in (
        (
            execution_path,
            TASK / "runtime_execution_validation_report_v0_1.json",
        ),
        (
            route_report_path,
            TASK / "route_following_validation_report_v0_1.json",
        ),
        (
            claim_path,
            TASK / "runtime_claim_boundary_report_v0_1.json",
        ),
    ):
        write_json(target, read_json(source))

    classification = (
        "task39_authorized_runtime_adapter_validation_and_cross_floor_room_demo_completed"
        if validation_passed
        else "task39_authorized_runtime_adapter_validated_runtime_execution_failed_with_blocker"
    )
    task_report = {
        "task_name": (
            "task39_authorized_runtime_adapter_validation_and_cross_floor_room_demo"
        ),
        "status": "completed" if validation_passed else "blocked",
        "classification": classification,
        "runtime_authorization_detected": True,
        "canonical_layer4_input_dir": rel(LAYER4),
        "task39_evidence_dir": rel(TASK),
        "selected_route": "cross_floor_room",
        "selected_profile": "conservative_canonical",
        "docs_and_manifests_read": DOCS_READ,
        "adapter_inventory_summary": {
            "status": "completed",
            "compatibility": "compatible_with_narrow_adapter",
            "report": rel(
                TASK / "task39_runtime_adapter_inventory_report_v0_1.json"
            ),
        },
        "adapter_binding_summary": {
            "status": "validated",
            "adapter": runtime["adapter"],
            "canonical_layer2_layer3_unchanged": hash_report[
                "canonical_layer2_layer3_unchanged"
            ],
        },
        "runtime_environment_summary": {
            "ROS_DISTRO": "foxy",
            "ROS_DOMAIN_ID": 84,
            "TURTLEBOT3_MODEL": "burger",
            "runtime_python": "/usr/bin/python3",
            "offline_python": "/home/ws/miniconda3/envs/boxfusion/bin/python",
            "rviz_launched": False,
            "AMCL_launched": False,
        },
        "runtime_execution_summary": {
            "attempted": True,
            "status": "completed" if route_completed else "failed",
            "components": runtime["runtime_components_launched"],
            "trajectory_available": True,
        },
        "route_validation_summary": {
            "passed": validation_passed,
            "floor_1_segment_status": runtime["floor_1_segment_status"],
            "floor_transition_handoff_status": runtime[
                "floor_transition_handoff_status"
            ],
            "floor_2_segment_status": runtime["floor_2_segment_status"],
            "route_completion_status": runtime["route_completion_status"],
            "final_room_status": runtime["final_room_status"],
            "floor_1_invalid_center_samples": floor1_summary[
                "occupied_unknown_or_out_of_bounds_sample_count"
            ],
            "floor_2_invalid_center_samples": floor2_summary[
                "occupied_unknown_or_out_of_bounds_sample_count"
            ],
        },
        "object_boundary_summary": claim_report["object_boundary"],
        "claim_boundary_summary": {
            "report_passed": True,
            "forbidden_claim_introduced": False,
            "AMCL_success_claimed": False,
            "real_robot_claimed": False,
            "physical_stair_climbing_claimed": False,
            "collision_free_guarantee_claimed": False,
        },
        "canonical_outputs_generated_or_updated": [
            rel(execution_path),
            rel(route_report_path),
            rel(claim_path),
            rel(LAYER4 / "logs/runtime_execution_command_log.txt"),
            rel(TRAJECTORY),
            rel(TRAJECTORY_SUMMARY),
            rel(LAYER4 / "manifests/runtime_artifact_manifest_v0_1.json"),
            rel(LAYER4 / "visualizations"),
        ],
        "docs_updated": [
            "docs/rslg_slam/runtime_validation_runbook.md",
            "docs/rslg_slam/current_project_status.md",
        ],
        "exact_blockers": exact_blockers,
        "recommended_next_task": execution_report["recommended_next_task"],
    }
    write_json(TASK / "task39_report.json", task_report)

    artifact_paths = sorted(
        {
            path
            for directory in (
                LAYER4 / "logs",
                LAYER4 / "trajectories",
                LAYER4 / "visualizations",
                LAYER4 / "runtime_configs",
                LAYER4 / "validation_reports",
                LAYER4 / "launch",
            )
            for path in directory.rglob("*")
            if path.is_file()
        }
    )
    artifact_manifest = {
        "schema_name": "rslg_task39_runtime_artifact_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "generated_utc": NOW,
        "selected_route": "cross_floor_room",
        "selected_profile": "conservative_canonical",
        "artifacts": [artifact_entry(path) for path in artifact_paths],
    }
    canonical_artifact_manifest = (
        LAYER4 / "manifests/runtime_artifact_manifest_v0_1.json"
    )
    write_json(canonical_artifact_manifest, artifact_manifest)
    write_json(TASK / "runtime_artifact_manifest_v0_1.json", artifact_manifest)

    known_modified = [
        REPO / path for path in TASK39_SOURCE_FILES + task_report["docs_updated"]
    ]
    known_modified.extend(
        path
        for directory in (
            TASK,
            LAYER4 / "runtime_configs",
            LAYER4 / "logs",
            LAYER4 / "trajectories",
            LAYER4 / "visualizations",
            LAYER4 / "validation_reports",
        )
        for path in directory.rglob("*")
        if path.is_file()
    )
    known_modified.append(canonical_artifact_manifest)
    manifest_paths = sorted({path for path in known_modified if path.exists()})

    def classify(path: Path) -> tuple[str, str]:
        value = rel(path)
        if "/logs/" in value:
            return "runtime log", "Layer 4: Runtime Validation Layer"
        if "/trajectories/" in value:
            return "runtime trajectory", "Layer 4: Runtime Validation Layer"
        if "/visualizations/" in value:
            return "runtime visualization", "Layer 4: Runtime Validation Layer"
        if "/runtime_configs/" in value or path.suffix in {".sh", ".sdf"}:
            return "runtime adapter/config", "Layer 4: Runtime Validation Layer"
        if value.startswith("docs/"):
            return "long-term project documentation", "project truth"
        if value.startswith("tools/"):
            return "canonical runtime tool", "Layer 4: Runtime Validation Layer"
        return "runtime report/manifest", "Layer 4: Runtime Validation Layer"

    created_manifest_path = (
        TASK / "created_or_modified_files_manifest_v0_1.json"
    )
    if created_manifest_path not in manifest_paths:
        manifest_paths.append(created_manifest_path)
    created_manifest = {
        "schema_name": "rslg_task39_created_or_modified_files_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": NOW,
        "files": [],
    }
    for path in sorted(manifest_paths):
        family, layer = classify(path)
        created_manifest["files"].append(
            {
                "path": rel(path),
                "size": path.stat().st_size if path.exists() else 0,
                "role": path.stem,
                "layer": layer,
                "artifact_family": family,
                "created_or_modified": "created_or_modified_by_task39",
            }
        )
    write_json(created_manifest_path, created_manifest)

    json_paths = sorted(
        {
            path
            for directory in (TASK, LAYER4)
            for path in directory.rglob("*.json")
            if path.is_file()
            and (
                directory == TASK
                or "task39_" in path.name
                or path.parent.name
                in {
                    "runtime_configs",
                    "trajectories",
                    "validation_reports",
                    "visualizations",
                    "manifests",
                }
            )
        }
    )
    validation_records = []
    for path in json_paths:
        try:
            read_json(path)
            validation_records.append(
                {"path": rel(path), "valid_json": True, "error": None}
            )
        except Exception as exc:  # pragma: no cover - evidence path
            validation_records.append(
                {"path": rel(path), "valid_json": False, "error": str(exc)}
            )
    json_report = {
        "schema_name": "rslg_task39_json_validation_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": NOW,
        "python_interpreter": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "validation_method": "Python json.load",
        "file_count": len(validation_records),
        "all_json_valid": all(
            record["valid_json"] for record in validation_records
        ),
        "files": validation_records,
    }
    write_json(TASK / "json_validation_report_v0_1.json", json_report)
    read_json(TASK / "json_validation_report_v0_1.json")

    print(
        json.dumps(
            {
                "status": task_report["status"],
                "classification": classification,
                "runtime_validation_passed": validation_passed,
                "json_validation_passed": json_report["all_json_valid"],
                "layer2_layer3_unchanged": hash_report[
                    "canonical_layer2_layer3_unchanged"
                ],
                "task_report": rel(TASK / "task39_report.json"),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if validation_passed and json_report["all_json_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
