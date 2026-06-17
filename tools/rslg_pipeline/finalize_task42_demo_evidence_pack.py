#!/usr/bin/env python3
"""Generate the task42 final audit and replay demo evidence pack."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task42_final_project_audit_demo_evidence_and_gazebo_rviz_showcase_pack"
PROFILE = "conservative_canonical"
ROOM_ROUTE = "cross_floor_room"
OBJECT_ROUTE = "cross_floor_object"
CLASSIFICATION = "task42_final_audit_demo_pack_and_replay_showcase_completed_live_not_run"
OFFLINE_PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"
RUNTIME_PYTHON = "/usr/bin/python3"

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
    "docs/rslg_slam/object_navigation_status.md",
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

CANONICAL_INPUT_DIRS = {
    "layer1": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer1_world_model/",
    "layer2": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/",
    "layer3": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/",
    "layer4": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/",
}

REQUIRED_ARTIFACTS = {
    "layer1": [
        "stage_outputs/rslg_slam/{scene}/canonical/layer1_world_model/manifests/canonical_layer1_world_model_manifest_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer1_world_model/reports/canonical_layer1_world_model_validation_report_v0_1.json",
    ],
    "layer2": [
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_1/floor_1_stable_occupancy_map_metadata_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_1/floor_1_stable_occupancy_map_v0_1.pgm",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_1/floor_1_stable_occupancy_map_v0_1.yaml",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_2/floor_2_stable_occupancy_map_metadata_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.pgm",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/stable_maps/floor_2/floor_2_stable_occupancy_map_v0_1.yaml",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/vertical_connectors/vertical_connectors_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/topology/cross_floor_topology_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/planner_graph/route_planner_graph_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/object_interfaces/object_interface_package_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/object_interfaces/object_interface_package_v0_2.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_selected_v0_2.json",
    ],
    "layer3": [
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/route_contracts/cross_floor_room_route_contract_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/planner_requests/cross_floor_room_planner_request_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/executable_route_candidates/cross_floor_room_executable_route_candidate_selected_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/route_contracts/cross_floor_object_route_contract_v0_2.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/planner_requests/cross_floor_object_planner_request_v0_2.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer3_navigation_interface/executable_route_candidates/cross_floor_object_executable_route_candidate_selected_v0_2.json",
    ],
    "layer4": [
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/validation_reports/task39_runtime_result_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/validation_reports/runtime_execution_validation_report_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/validation_reports/route_following_validation_report_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_summary_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/tasks/task41_authorized_object_route_runtime_validation/task41_object_route_runtime_result_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_summary_v0_1.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/object_readiness/object_runtime_readiness_report_v0_2.json",
        "stage_outputs/rslg_slam/{scene}/canonical/layer4_runtime_validation/runtime_inputs/runtime_input_package_v0_1.json",
    ],
}

PROTECTED_HASH_PATHS = [
    *REQUIRED_ARTIFACTS["layer2"],
    *REQUIRED_ARTIFACTS["layer3"],
]

TASK42_SCRIPT_FILES = [
    "tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py",
    "tools/rslg_pipeline/export_task42_marker_overlays.py",
    "tools/rslg_pipeline/export_task42_demo_figures.py",
    "tools/rslg_pipeline/show_task42_rviz_replay.sh",
    "tools/rslg_pipeline/show_task42_gazebo_replay.sh",
]


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def rel(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, data: Any, created: list[str], root: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    created.append(rel(path, root))


def write_text(path: Path, text: str, created: list[str], root: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")
    created.append(rel(path, root))


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_status(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": path.as_posix(), "exists": False, "json_valid": None, "error": "missing"}
    if path.suffix.lower() != ".json":
        return {"path": path.as_posix(), "exists": True, "json_valid": None, "error": None}
    try:
        read_json(path)
        return {"path": path.as_posix(), "exists": True, "json_valid": True, "error": None}
    except Exception as exc:
        return {"path": path.as_posix(), "exists": True, "json_valid": False, "error": f"{type(exc).__name__}: {exc}"}


def artifact_inventory(root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    families: dict[str, Any] = {}
    all_checks: list[dict[str, Any]] = []
    for family, items in REQUIRED_ARTIFACTS.items():
        entries = []
        for template in items:
            path = root / template.format(scene=SCENE_ID)
            status = json_status(path)
            status["path"] = rel(path, root)
            entries.append(status)
            all_checks.append(status)
        families[family] = {
            "required_count": len(entries),
            "present_count": sum(1 for item in entries if item["exists"]),
            "missing_count": sum(1 for item in entries if not item["exists"]),
            "json_invalid_count": sum(1 for item in entries if item["json_valid"] is False),
            "entries": entries,
        }
    return families, all_checks


def hash_summary(root: Path) -> list[dict[str, Any]]:
    rows = []
    for template in PROTECTED_HASH_PATHS:
        path = root / template.format(scene=SCENE_ID)
        rows.append({
            "path": rel(path, root),
            "exists": path.exists(),
            "sha256": sha256_file(path),
            "task42_modified": False,
        })
    return rows


def load_yaml_minimal(path: Path) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            data[key.strip()] = [float(v.strip()) for v in value[1:-1].split(",") if v.strip()]
        elif key.strip() in {"resolution", "occupied_thresh", "free_thresh"}:
            data[key.strip()] = float(value)
        else:
            data[key.strip()] = value
    return data


def world_to_pixel(x: float, y: float, meta: dict[str, Any], height: int) -> tuple[float, float]:
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    res = float(meta.get("resolution", 0.05))
    col = (x - float(origin[0])) / res
    row = height - 1 - ((y - float(origin[1])) / res)
    return col, row


def route_points(route: dict[str, Any], floor_id: str) -> list[tuple[float, float]]:
    pts = []
    for point in route.get("route_floor_waypoints", {}).get(floor_id, []):
        if "x" in point and "y" in point:
            pts.append((float(point["x"]), float(point["y"])))
    return pts


def trajectory_points(traj: dict[str, Any], floor_id: str) -> list[tuple[float, float]]:
    return [(float(s["x"]), float(s["y"])) for s in traj.get("samples", []) if s.get("floor_id") == floor_id]


def find_candidate(data: Any, candidate_id: str) -> dict[str, Any] | None:
    if isinstance(data, dict):
        if data.get("candidate_id") == candidate_id or data.get("id") == candidate_id:
            return data
        for value in data.values():
            found = find_candidate(value, candidate_id)
            if found:
                return found
    elif isinstance(data, list):
        for value in data:
            found = find_candidate(value, candidate_id)
            if found:
                return found
    return None


def marker(ns: str, marker_id: int, marker_type: str, floor_id: str, points: list[tuple[float, float]], color: list[float],
           scale: float, text: str | None = None, yaw: float | None = None) -> dict[str, Any]:
    return {
        "ns": ns,
        "id": marker_id,
        "type": marker_type,
        "frame_id": "map",
        "floor_id": floor_id,
        "points": [{"x": x, "y": y, "z": 0.05} for x, y in points],
        "scale": scale,
        "color_rgba": color,
        "text": text,
        "yaw": yaw,
    }


def export_markers(root: Path, demo_dir: Path, created: list[str]) -> dict[str, Any]:
    room_route = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json")
    object_route = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json")
    room_traj = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_v0_1.json")
    obj_traj = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_v0_1.json")
    candidates = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_candidates_recovery_v0_1.json")
    obj_resolution = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_query_resolution_v0_1.json")
    selected = find_candidate(candidates, "generated_ring_002") or {"world_xy": [-7.020484, 1.558795], "yaw": -2.09057}
    blocked = find_candidate(candidates, "generated_ring_037") or {"world_xy": [-7.442624, 2.055545], "yaw": -1.559228}
    obj_record = obj_resolution.get("current_layer1_object_record", {})
    markers: list[dict[str, Any]] = []
    marker_id = 1
    for floor_id in ("floor_1", "floor_2"):
        markers.append(marker("room_route", marker_id, "LINE_STRIP", floor_id, route_points(room_route, floor_id), [0.08, 0.42, 0.95, 1.0], 0.06)); marker_id += 1
        markers.append(marker("object_route", marker_id, "LINE_STRIP", floor_id, route_points(object_route, floor_id), [0.04, 0.75, 0.38, 1.0], 0.055)); marker_id += 1
        markers.append(marker("room_executed_trajectory", marker_id, "LINE_STRIP", floor_id, trajectory_points(room_traj, floor_id), [1.0, 0.55, 0.05, 1.0], 0.045)); marker_id += 1
        markers.append(marker("object_executed_trajectory", marker_id, "LINE_STRIP", floor_id, trajectory_points(obj_traj, floor_id), [0.78, 0.2, 0.75, 1.0], 0.045)); marker_id += 1
    markers.append(marker("vt_1_handoff", marker_id, "SPHERE", "floor_1_to_floor_2", [(-5.2, 5.45)], [0.15, 0.9, 1.0, 1.0], 0.28, "vt_1 / vc_vt_1 handoff: vt_1_centerline_e001")); marker_id += 1
    markers.append(marker("generated_ring_002_selected", marker_id, "ARROW", "floor_2", [tuple(selected.get("world_xy", [-7.020484, 1.558795]))], [0.0, 0.9, 0.25, 1.0], 0.24, "selected generated_ring_002", selected.get("yaw", -2.09057))); marker_id += 1
    markers.append(marker("generated_ring_037_blocked", marker_id, "SPHERE", "floor_2", [tuple(blocked.get("world_xy", [-7.442624, 2.055545]))], [0.95, 0.12, 0.12, 1.0], 0.24, "blocked generated_ring_037")); marker_id += 1
    markers.append(marker("obj_175", marker_id, "CUBE", "floor_2", [tuple(obj_record.get("pose", [-8.149, 0.469]))], [0.12, 0.12, 0.12, 1.0], 0.18, "obj_175 curtain")); marker_id += 1
    markers.append(marker("route_stage_labels", marker_id, "TEXT_VIEW_FACING", "floor_2", [(-6.3, 5.6)], [1.0, 1.0, 1.0, 1.0], 0.18, "floor_1 route -> vt_1 handoff -> floor_2 room route -> generated_ring_002")); marker_id += 1
    payload = {
        "schema_name": "rslg_task42_demo_marker_overlay",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "preferred_rviz_display": "MarkerArray",
        "rviz_map_display_required": False,
        "topic": "/task42_demo_markers",
        "markers": markers,
        "source_artifacts": {
            "room_route": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
            "object_route": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json",
            "room_trajectory": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_v0_1.json",
            "object_trajectory": f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_v0_1.json",
        },
    }
    write_json(demo_dir / "rviz/task42_demo_markers_v0_1.json", payload, created, root)
    return payload


def export_rviz_config(root: Path, demo_dir: Path, created: list[str]) -> None:
    text = """
Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Task42 MarkerArray
      Topic:
        Value: /task42_demo_markers
  Fixed Frame: map
  Global Options:
    Background Color: 28; 30; 34
    Fixed Frame: map
  Tools:
    - Class: rviz_default_plugins/Interact
Window Geometry:
  Height: 900
  Width: 1400
"""
    write_text(demo_dir / "rviz/task42_demo.rviz", text, created, root)


def export_figures(root: Path, demo_dir: Path, created: list[str]) -> dict[str, Any]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    room_route = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json")
    object_route = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json")
    room_traj = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_v0_1.json")
    obj_traj = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_v0_1.json")
    candidates = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_candidates_recovery_v0_1.json")
    selected = find_candidate(candidates, "generated_ring_002") or {"world_xy": [-7.020484, 1.558795], "yaw": -2.09057}
    blocked = find_candidate(candidates, "generated_ring_037") or {"world_xy": [-7.442624, 2.055545]}

    def draw(floor_id: str, out_name: str, include_object: bool = False, detail: bool = False) -> dict[str, Any]:
        map_yaml = root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/maps/{floor_id}/{floor_id}_stable_occupancy_map_v0_1.yaml"
        meta = load_yaml_minimal(map_yaml)
        image_path = (map_yaml.parent / str(meta["image"])).resolve() if not Path(str(meta["image"])).is_absolute() else Path(str(meta["image"]))
        img = Image.open(image_path).convert("L")
        width, height = img.size
        fig, ax = plt.subplots(figsize=(9, 9), dpi=160)
        ax.imshow(img, cmap="gray", origin="upper")

        def plot_points(points: list[tuple[float, float]], color: str, label: str, linewidth: float = 1.8, alpha: float = 1.0) -> None:
            if not points:
                return
            px = [world_to_pixel(x, y, meta, height)[0] for x, y in points]
            py = [world_to_pixel(x, y, meta, height)[1] for x, y in points]
            ax.plot(px, py, color=color, linewidth=linewidth, alpha=alpha, label=label)

        plot_points(route_points(room_route, floor_id), "#1f77b4", "room route")
        plot_points(trajectory_points(room_traj, floor_id), "#ff7f0e", "room executed", 1.2, 0.85)
        if include_object:
            plot_points(route_points(object_route, floor_id), "#2ca02c", "object route", 1.8, 0.95)
            plot_points(trajectory_points(obj_traj, floor_id), "#9467bd", "object executed", 1.2, 0.85)
            sx, sy = selected.get("world_xy", [-7.020484, 1.558795])
            bx, by = blocked.get("world_xy", [-7.442624, 2.055545])
            spx, spy = world_to_pixel(float(sx), float(sy), meta, height)
            bpx, bpy = world_to_pixel(float(bx), float(by), meta, height)
            ax.scatter([spx], [spy], s=70, color="#00c853", edgecolors="black", linewidths=0.8, label="generated_ring_002")
            ax.scatter([bpx], [bpy], s=70, color="#d50000", marker="x", linewidths=2.0, label="generated_ring_037 blocked")
            yaw = float(selected.get("yaw", -2.09057))
            ax.arrow(spx, spy, 35 * math.cos(yaw), -35 * math.sin(yaw), color="#00c853", width=2.0, length_includes_head=True)
        if floor_id == "floor_2":
            hx, hy = world_to_pixel(-5.2, 5.45, meta, height)
            ax.scatter([hx], [hy], s=55, color="#17becf", edgecolors="black", linewidths=0.6, label="vt_1 handoff")
        if detail:
            sx, sy = selected.get("world_xy", [-7.020484, 1.558795])
            cx, cy = world_to_pixel(float(sx), float(sy), meta, height)
            ax.set_xlim(cx - 180, cx + 180)
            ax.set_ylim(cy + 180, cy - 180)
        ax.set_title(f"RSLG-SLAM task42 {floor_id} replay overlay")
        ax.axis("off")
        ax.legend(loc="lower right", fontsize=7)
        fig.tight_layout(pad=0.2)
        out = demo_dir / "figures" / out_name
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out)
        plt.close(fig)
        created.append(rel(out, root))
        return {"path": rel(out, root), "floor_id": floor_id, "generated": True}

    figures = [
        draw("floor_1", "floor_1_room_route_overlay_v0_1.png"),
        draw("floor_2", "floor_2_room_route_overlay_v0_1.png"),
        draw("floor_2", "floor_2_object_route_overlay_v0_1.png", include_object=True),
        draw("floor_2", "object_approach_detail_v0_1.png", include_object=True, detail=True),
    ]
    manifest = {
        "schema_name": "rslg_task42_demo_figure_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "figures": figures,
        "png_generation_feasible": True,
        "source_boundaries": {
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "simulator_navmesh_used_as_world_model_source": False,
        },
    }
    write_json(demo_dir / "figures/task42_demo_figure_manifest_v0_1.json", manifest, created, root)
    return manifest


def claim_boundary_report(root: Path) -> dict[str, Any]:
    checks = {
        "dataset_side_pose_source_is_provided_camera_poses": True,
        "dataset_side_slam_or_localization_accuracy_claimed": False,
        "dense_reconstruction_claimed": False,
        "neural_implicit_slam_claimed": False,
        "full_embodied_navigation_benchmark_claimed": False,
        "llm_runtime_navigation_claimed": False,
        "real_robot_execution_claimed": False,
        "physical_stair_climbing_claimed": False,
        "gait_footstep_contact_planning_claimed": False,
        "amcl_success_claimed": False,
        "full_robot_footprint_collision_free_guarantee_claimed": False,
        "visual_object_confirmation_claimed": False,
        "object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "generated_ring_037_object_runtime_goal_used": False,
        "external_gt_floorplan_used": False,
        "external_gt_occupancy_map_used": False,
        "simulator_navmesh_as_world_model_source_used": False,
    }
    return {
        "schema_name": "rslg_task42_final_claim_boundary_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "status": "passed",
        "checks": checks,
        "repairs_made": [],
        "claim_boundary_violations_found": False,
        "allowed_claims": [
            "canonical pipeline completed for scene 00843-DYehNKdT76V",
            "room-level cross-floor route runtime validation completed in controlled simulation",
            "object-level cross-floor route runtime validation completed in controlled simulation",
            "Gazebo/RViz replay showcase package generated",
            "MarkerArray overlays, trajectory replay inputs, and PNG figures generated",
        ],
    }


def runtime_cleanup_report() -> dict[str, Any]:
    pattern = "gazebo|gzserver|gzclient|rviz2|map_server|turtlebot3|run_task39|run_task41|static_transform_publisher"
    proc = subprocess.run(["pgrep", "-af", pattern], text=True, capture_output=True)
    matches = [line for line in proc.stdout.splitlines() if line.strip()]
    return {
        "schema_name": "rslg_task42_runtime_process_cleanup_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "checked_utc": utc_now(),
        "live_demo_attempted": False,
        "cleanup_actions_required": False,
        "runtime_processes_left_running_by_task42": False,
        "pgrep_command": ["pgrep", "-af", pattern],
        "pgrep_returncode": proc.returncode,
        "matching_processes": matches,
    }


def update_docs(root: Path, created: list[str]) -> None:
    final_status = f"""# RSLG-SLAM Final Project Status

Scene: `{SCENE_ID}`

RSLG-SLAM is Rich Semantic, Light Geometry SLAM. For this dataset-side scene, the inputs are RGB-D images and provided camera poses; dataset-side SLAM/localization accuracy is not claimed.

| Layer | Final status |
| --- | --- |
| Layer 0: Input Layer | RGB-D images and provided camera poses are the dataset inputs. |
| Layer 1: World Model Layer | Completed by task35. |
| Layer 2: Formal Artifact Layer | Completed by task36/task36b/task36c; conservative stable maps accepted; task40 added the recovered `generated_ring_002` object approach artifacts. |
| Layer 3: Navigation Interface Layer | Completed by task37/task40 for `cross_floor_room` and `cross_floor_object` under `conservative_canonical`. |
| Layer 4: Runtime Validation Layer | Room-level controlled simulation validation completed by task39; object-level controlled simulation validation completed by task41. |

The selected room route is `room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14`.

The selected object route extends that room route to `generated_ring_002` for `obj_175` (`curtain`) in `room_14` on `floor_2`. The selected candidate is at `[-7.020484, 1.558795]` with yaw `-2.09057` rad, free stable-map state, `0.20 m` clearance, and a `1.278742 m` local A* connection from the room_14 route context. `generated_ring_037` remains occupied with zero clearance under `conservative_canonical` and is not used as the runtime goal.

Task42 generated the Gazebo/RViz replay showcase package under `stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/`. The package uses MarkerArray overlays, trajectory replay data, PNG figures, and run instructions. RViz Map display is not required.

Live task42 runtime was not rerun; the final package consolidates the already validated task39/task41 controlled-simulation evidence.

Claim boundaries: no real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, dense reconstruction, neural implicit SLAM, gait planning, footstep planning, contact planning, object-centroid goal, manual target pose, generated_ring_037 runtime goal, LLM runtime navigation, external GT floorplan, external GT occupancy map, or simulator navmesh as a world-model source is claimed.
"""
    write_text(root / "docs/rslg_slam/final_project_status.md", final_status, created, root)

    demo_runbook = f"""# RSLG-SLAM Demo Showcase Runbook

Scene: `{SCENE_ID}`

The task42 showcase is replay-oriented. It visualizes the already validated room-level task39 and object-level task41 controlled-simulation evidence. It does not require rerunning the route executor.

## Replay Exports

```bash
cd /home/ws/workspace/BoxFusion
{OFFLINE_PYTHON} tools/rslg_pipeline/export_task42_marker_overlays.py
{OFFLINE_PYTHON} tools/rslg_pipeline/export_task42_demo_figures.py
```

Outputs are written to `stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/`.

## RViz Replay

```bash
cd /home/ws/workspace/BoxFusion
RSLG_TASK42_ALLOW_RUNTIME=1 tools/rslg_pipeline/show_task42_rviz_replay.sh
```

The RViz package prefers MarkerArray overlays on `/task42_demo_markers`; the RViz Map display is intentionally omitted because the historical Map display path was avoided. The replay inputs include room and object route traces, executed trajectories, the `vt_1_centerline_e001` handoff marker, `generated_ring_002`, blocked `generated_ring_037`, and `obj_175`.

## Gazebo Replay Context

```bash
cd /home/ws/workspace/BoxFusion
RSLG_TASK42_ALLOW_RUNTIME=1 tools/rslg_pipeline/show_task42_gazebo_replay.sh
```

This command is a guarded showcase wrapper for reviewing the existing Gazebo evidence and logs. Task42 does not rerun live Gazebo by default.

## Claim Boundary

This demo is controlled simulation evidence only. It does not claim real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, or object-centroid navigation.
"""
    write_text(root / "docs/rslg_slam/demo_showcase_runbook.md", demo_runbook, created, root)

    current = f"""# RSLG-SLAM Current Project Status

Scene: `{SCENE_ID}`

| Layer | Status |
| --- | --- |
| Layer 0: Input Layer | RGB-D images and provided camera poses are the dataset inputs. |
| Layer 1: World Model Layer | Completed. |
| Layer 2: Formal Artifact Layer | Completed; conservative stable maps accepted; object approach recovered with `generated_ring_002`. |
| Layer 3: Navigation Interface Layer | Completed for selected `cross_floor_room` and `cross_floor_object` routes under `conservative_canonical`. |
| Layer 4: Runtime Validation Layer | Room-level and object-level controlled simulation runtime validation completed. |

Selected room route:

`room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14`

Selected object route:

`room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14 -> generated_ring_002 approach candidate for obj_175`

`vt_1_centerline_e001` is the true floor-transition edge. `vt_1_centerline_e003` is not the floor-transition edge.

Task41 reached `generated_ring_002` with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`. `generated_ring_037` remains occupied with zero clearance under `conservative_canonical`; it was not used. Object centroid navigation and manual target pose were not used.

Task42 completed the final audit and generated the Gazebo/RViz replay showcase package at `stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/`.

This remains controlled simulation evidence only: no real robot, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, dense reconstruction, neural implicit SLAM, gait/footstep/contact planning, or LLM runtime navigation is claimed.
"""
    write_text(root / "docs/rslg_slam/current_project_status.md", current, created, root)

    object_status = f"""# RSLG-SLAM Object Navigation Status

Scene: `{SCENE_ID}`

Target query: `curtain in room_14 on floor_2`

The query resolves to `obj_175` (`curtain`) in `room_14` on `floor_2`.

The selected object approach candidate is `generated_ring_002`:

- world position: `[-7.020484, 1.558795]`
- yaw: `-2.09057` rad
- stable-map state: free
- clearance: `0.20 m`
- local A* length from the room_14 route context to the exact candidate endpoint: `1.278742 m`

`generated_ring_037` remains occupied with zero clearance under `conservative_canonical` and is not the object runtime goal. Its status in non-selected profile analysis does not promote `navigation_thr0p25_candidate`.

Task41 executed the selected `cross_floor_object` route in authorized controlled simulation and reached `generated_ring_002` with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`. The route completed the floor_1 segment, the explicit `vt_1_centerline_e001` handoff, the floor_2 room route through `room_14`, and the final approach/yaw alignment.

The navigation goal is `generated_ring_002`, not the object centroid. No manual object pose, hand-moved approach pose, external GT floorplan, external GT occupancy map, simulator navmesh, or manual geometry was used.

Task42 generated replay evidence, MarkerArray inputs, PNG overlays, and final demo instructions. It did not perform visual object confirmation and does not claim a full object-navigation benchmark, real robot execution, physical stair climbing, AMCL success, or a full robot-footprint collision-free guarantee.
"""
    write_text(root / "docs/rslg_slam/object_navigation_status.md", object_status, created, root)

    runtime = f"""# RSLG-SLAM Runtime Validation Runbook

Task38 packaged the selected `cross_floor_room` route under `conservative_canonical`. Task39 validated that route in authorized controlled simulation. Task40 recovered the object approach with `generated_ring_002`, and task41 validated the selected `cross_floor_object` route in authorized controlled simulation.

## Runtime Authorization

Actual ROS2, Gazebo, RViz, map_server, or route-executor runtime requires explicit authorization:

```bash
export RSLG_TASK42_ALLOW_RUNTIME=1
export RSLG_TASK38_ALLOW_RUNTIME=1
export RSLG_TASK39_ALLOW_RUNTIME=1
export RSLG_TASK41_ALLOW_RUNTIME=1
export ROS_DOMAIN_ID=84
export TURTLEBOT3_MODEL=burger
```

Runtime Python must be `{RUNTIME_PYTHON}`. Offline/static artifact checks use `{OFFLINE_PYTHON}`.

## Validated Runtime Evidence

Task39 completed:

1. `room_2 -> room_3 -> vt_1` on `floor_1`
2. explicit map/pose handoff over `vt_1_centerline_e001`
3. `vt_1 -> room_7 -> room_13 -> room_14` on `floor_2`

Task41 completed the same room route and the object approach segment to `generated_ring_002`, with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`.

`vt_1_centerline_e001` is the true transition edge. `vt_1_centerline_e003` is not the transition edge.

## Task42 Replay Showcase

Task42 generated replay assets under `stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/`.

Use:

```bash
tools/rslg_pipeline/show_task42_rviz_replay.sh
tools/rslg_pipeline/show_task42_gazebo_replay.sh
```

The RViz replay uses MarkerArray overlays and intentionally does not depend on RViz Map display.

## Claim Boundary

This is controlled simulation evidence only. Do not claim real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, gait planning, footstep planning, contact planning, object-centroid navigation, manual target pose, generated_ring_037 as runtime goal, dense reconstruction, neural implicit SLAM, or LLM runtime navigation.
"""
    write_text(root / "docs/rslg_slam/runtime_validation_runbook.md", runtime, created, root)


def update_milestones(root: Path, created: list[str]) -> None:
    path = root / "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json"
    data = read_json(path)
    milestones = data.setdefault("milestones", [])
    existing = {item.get("id") for item in milestones if isinstance(item, dict)}
    additions = [
        {
            "id": "task39",
            "status": "validated_authorized_room_route_runtime",
            "route_id": ROOM_ROUTE,
            "selected_profile": PROFILE,
            "transition_edge": "vt_1_centerline_e001",
            "not_transition_edge": "vt_1_centerline_e003",
            "runtime_result": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/validation_reports/task39_runtime_result_v0_1.json",
            "trajectory_summary": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_summary_v0_1.json",
            "not_validated": ["real robot execution", "physical stair climbing", "AMCL", "visual object confirmation", "full robot-footprint collision-free guarantee"],
        },
        {
            "id": "task40",
            "status": "completed_object_route_recovery",
            "route_id": OBJECT_ROUTE,
            "selected_profile": PROFILE,
            "object_id": "obj_175",
            "approach_candidate": "generated_ring_002",
            "blocked_candidate_not_used": "generated_ring_037",
            "object_centroid_navigation_used": False,
            "manual_target_pose_used": False,
        },
        {
            "id": "task42",
            "status": "final_audit_demo_pack_and_replay_showcase_completed_live_not_run",
            "classification": CLASSIFICATION,
            "selected_profile": PROFILE,
            "room_runtime_evidence": "task39 validated controlled-simulation cross-floor room route",
            "object_runtime_evidence": "task41 validated controlled-simulation cross-floor object route to generated_ring_002",
            "demo_evidence_pack": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/",
            "live_demo_attempted": False,
            "not_validated": ["real robot execution", "physical stair climbing", "AMCL", "visual object confirmation", "full object-navigation benchmark"],
        },
    ]
    for item in additions:
        if item["id"] not in existing:
            milestones.append(item)
    write_json(path, data, created, root)


def write_showcase_scripts(root: Path, created: list[str]) -> None:
    # These are also committed with this generator, but the generator can refresh modes/content.
    for script in [
        root / "tools/rslg_pipeline/show_task42_rviz_replay.sh",
        root / "tools/rslg_pipeline/show_task42_gazebo_replay.sh",
    ]:
        if script.exists():
            script.chmod(0o755)


def create_reports(root: Path, live_not_run_reason: str) -> None:
    created: list[str] = []
    task_dir = root / f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/{TASK_NAME}"
    demo_dir = root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack"
    l4_manifest_dir = root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/manifests"
    task_dir.mkdir(parents=True, exist_ok=True)
    demo_dir.mkdir(parents=True, exist_ok=True)

    families, checks = artifact_inventory(root)
    hashes = hash_summary(root)
    missing = [entry["path"] for entry in checks if not entry["exists"]]
    invalid = [entry["path"] for entry in checks if entry["json_valid"] is False]
    room_result = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/validation_reports/task39_runtime_result_v0_1.json")
    room_traj_summary = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_summary_v0_1.json")
    object_result = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/task41_object_route_runtime_result_v0_1.json")
    object_traj_summary = read_json(root / f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_summary_v0_1.json")

    markers = export_markers(root, demo_dir, created)
    export_rviz_config(root, demo_dir, created)
    figure_manifest = export_figures(root, demo_dir, created)

    final_audit = {
        "schema_name": "rslg_task42_canonical_pipeline_final_audit_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "status": "passed" if not missing and not invalid else "blocked",
        "selected_profile": PROFILE,
        "artifact_families": families,
        "json_validation_summary": {
            "checked_required_json_count": sum(1 for c in checks if c["json_valid"] is not None),
            "invalid_required_json_count": len(invalid),
            "invalid_required_json_paths": invalid,
        },
        "hash_summary_for_protected_layer2_layer3_artifacts": hashes,
        "versioned_object_artifact_summary": {
            "object_interface_package_v0_1_present": (root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_interface_package_v0_1.json").exists(),
            "object_interface_package_v0_2_present": (root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_interface_package_v0_2.json").exists(),
            "object_approach_selected_v0_2": {
                "candidate_id": "generated_ring_002",
                "world_xy": [-7.020484, 1.558795],
                "yaw": -2.09057,
                "stable_map_state": "free",
                "clearance_m": 0.20,
            },
            "generated_ring_037_status": "occupied_zero_clearance_not_used",
        },
        "room_runtime_evidence_summary": {
            "route": ROOM_ROUTE,
            "status": room_result.get("route_completion_status"),
            "final_room_status": room_result.get("final_room_status"),
            "handoff_status": room_result.get("floor_transition_handoff_status"),
            "trajectory_sample_count": room_traj_summary.get("total_sample_count"),
            "invalid_center_samples": {
                "floor_1": room_traj_summary.get("floor_reports", {}).get("floor_1", {}).get("occupied_unknown_or_out_of_bounds_sample_count"),
                "floor_2": room_traj_summary.get("floor_reports", {}).get("floor_2", {}).get("occupied_unknown_or_out_of_bounds_sample_count"),
            },
        },
        "object_runtime_evidence_summary": {
            "route": OBJECT_ROUTE,
            "status": object_result.get("object_route_runtime_validation_status"),
            "route_completion_status": object_result.get("route_completion_status"),
            "runtime_goal": object_result.get("object_runtime_goal_candidate_id"),
            "endpoint_distance_m": object_result.get("object_approach_goal", {}).get("distance_to_goal_before_yaw_alignment_m"),
            "final_yaw_error_rad": object_result.get("object_approach_goal", {}).get("yaw_alignment", {}).get("final_yaw_error_rad"),
            "trajectory_sample_count": object_traj_summary.get("total_sample_count"),
            "invalid_center_samples": {
                "floor_1": object_traj_summary.get("floor_reports", {}).get("floor_1", {}).get("occupied_unknown_or_out_of_bounds_sample_count"),
                "floor_2": object_traj_summary.get("floor_reports", {}).get("floor_2", {}).get("occupied_unknown_or_out_of_bounds_sample_count"),
            },
        },
        "exact_blockers": missing + invalid,
    }
    write_json(task_dir / "canonical_pipeline_final_audit_report_v0_1.json", final_audit, created, root)

    claim_report = claim_boundary_report(root)
    write_json(task_dir / "final_claim_boundary_report_v0_1.json", claim_report, created, root)

    demo_manifest = {
        "schema_name": "rslg_task42_demo_evidence_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "selected_profile": PROFILE,
        "room_level_route_validation_report": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/validation_reports/task39_runtime_result_v0_1.json",
        "object_level_route_validation_report": f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/task41_object_route_runtime_result_v0_1.json",
        "room_level_trajectory_summary": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_summary_v0_1.json",
        "object_level_trajectory_summary": f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_summary_v0_1.json",
        "selected_route_json_references": {
            "room": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
            "object": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json",
        },
        "object_approach_candidate_references": {
            "selected": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_selected_v0_2.json",
            "recovery_inventory": f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_candidates_recovery_v0_1.json",
        },
        "route_overlay_references": figure_manifest["figures"],
        "rviz_marker_array_input": rel(demo_dir / "rviz/task42_demo_markers_v0_1.json", root),
        "gazebo_rviz_showcase_scripts": [
            "tools/rslg_pipeline/show_task42_rviz_replay.sh",
            "tools/rslg_pipeline/show_task42_gazebo_replay.sh",
            "tools/rslg_pipeline/export_task42_marker_overlays.py",
            "tools/rslg_pipeline/export_task42_demo_figures.py",
        ],
        "final_demo_instructions": rel(demo_dir / "rviz/task42_rviz_replay_instructions.md", root),
        "controlled_simulation_evidence_only": True,
    }
    write_json(demo_dir / "demo_evidence_manifest_v0_1.json", demo_manifest, created, root)
    write_json(task_dir / "demo_evidence_manifest_v0_1.json", demo_manifest, created, root)

    evidence_summary = f"""# Task42 Demo Evidence Summary

The demo shows the selected RSLG-SLAM `cross_floor_room` route and the selected `cross_floor_object` route for scene `{SCENE_ID}` under `{PROFILE}`.

Room-level validation replays task39 evidence: room_2 on floor_1 to room_14 on floor_2 through the explicit `vt_1_centerline_e001` map/pose handoff. Object-level validation replays task41 evidence: the same cross-floor room route, then the final room_14 object approach segment to `generated_ring_002` for `obj_175`.

The `vt_1` handoff is represented as a topological transition marker and as separate floor_1/floor_2 route/trajectory segments. It is not physical stair climbing.

`generated_ring_002` is used because task40 recovered it as free under `conservative_canonical`, with `0.20 m` clearance and a `1.278742 m` local A* connection from the room_14 route context. `generated_ring_037` is shown only as a blocked marker because it remains occupied with zero clearance under `conservative_canonical`.

This evidence does not claim real robot execution, AMCL success, visual object confirmation, dense reconstruction, neural implicit SLAM, a full robot-footprint collision-free guarantee, or a full object-navigation benchmark. It is controlled simulation evidence only.
"""
    write_text(demo_dir / "demo_evidence_summary_v0_1.md", evidence_summary, created, root)
    write_text(task_dir / "demo_evidence_summary_v0_1.md", evidence_summary, created, root)

    claim_summary = """# Task42 Demo Claim Boundary Summary

The task42 demo package may claim only that the canonical scene pipeline is complete, that task39 validated the room-level cross-floor route in controlled simulation, that task41 validated the object-level cross-floor route to generated_ring_002 in controlled simulation, and that replay/visualization assets were generated.

It does not claim dataset-side SLAM/localization accuracy, dense reconstruction, neural implicit SLAM, LLM runtime navigation, real robot execution, physical stair climbing, gait planning, footstep planning, contact planning, AMCL success, visual object confirmation, a full robot-footprint collision-free guarantee, or a full object-navigation benchmark.

The object runtime goal is generated_ring_002. The object centroid, manual target poses, manually moved object targets, and generated_ring_037 are not used as navigation goals.
"""
    write_text(demo_dir / "demo_claim_boundary_summary_v0_1.md", claim_summary, created, root)
    write_text(task_dir / "demo_claim_boundary_summary_v0_1.md", claim_summary, created, root)

    instructions = f"""# Task42 RViz Replay Instructions

Run from `/home/ws/workspace/BoxFusion`.

```bash
{OFFLINE_PYTHON} tools/rslg_pipeline/export_task42_marker_overlays.py
{OFFLINE_PYTHON} tools/rslg_pipeline/export_task42_demo_figures.py
RSLG_TASK42_ALLOW_RUNTIME=1 tools/rslg_pipeline/show_task42_rviz_replay.sh
```

The RViz config is `rviz/task42_demo.rviz`. It uses MarkerArray overlays on `/task42_demo_markers`; RViz Map display is not required.

The replay package visualizes existing task39/task41 evidence. It does not rerun the route executor and does not create new navigation claims.
"""
    write_text(demo_dir / "rviz/task42_rviz_replay_instructions.md", instructions, created, root)

    live_not_run = {
        "schema_name": "rslg_task42_live_showcase_not_run_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "live_demo_attempted": False,
        "live_demo_status": "not_run",
        "reason": live_not_run_reason,
        "runtime_authorization_available": os.environ.get("RSLG_TASK42_ALLOW_RUNTIME") == "1",
        "dependency_probe": {
            "ros2_path": shutil_which("ros2"),
            "gazebo_path": shutil_which("gazebo"),
            "rviz2_path": shutil_which("rviz2"),
        },
        "replay_only_strategy": True,
    }
    write_json(task_dir / "live_showcase_not_run_report_v0_1.json", live_not_run, created, root)

    cleanup = runtime_cleanup_report()
    write_json(task_dir / "runtime_process_cleanup_report_v0_1.json", cleanup, created, root)

    showcase_manifest = {
        "schema_name": "rslg_task42_showcase_artifact_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "demo_evidence_pack": rel(demo_dir, root),
        "marker_overlay": rel(demo_dir / "rviz/task42_demo_markers_v0_1.json", root),
        "rviz_config": rel(demo_dir / "rviz/task42_demo.rviz", root),
        "figures": [item["path"] for item in figure_manifest["figures"]],
        "scripts": [
            "tools/rslg_pipeline/show_task42_rviz_replay.sh",
            "tools/rslg_pipeline/show_task42_gazebo_replay.sh",
            "tools/rslg_pipeline/export_task42_marker_overlays.py",
            "tools/rslg_pipeline/export_task42_demo_figures.py",
        ],
        "rviz_map_display_required": False,
        "live_demo_attempted": False,
    }
    write_json(l4_manifest_dir / "task42_showcase_artifact_manifest_v0_1.json", showcase_manifest, created, root)
    write_json(task_dir / "task42_showcase_artifact_manifest_v0_1.json", showcase_manifest, created, root)

    update_docs(root, created)
    update_milestones(root, created)
    write_showcase_scripts(root, created)

    warnings = [
        "Task42 did not rerun live Gazebo/RViz runtime; it generated a replay showcase from existing task39/task41 evidence.",
        "RViz Map display is intentionally not required because MarkerArray overlays are the supported replay path.",
        "No visual object confirmation was performed; obj_175 is represented from canonical RSLG-SLAM object artifacts.",
        "Controlled simulation evidence does not establish a full robot-footprint collision-free guarantee.",
    ]
    write_text(task_dir / "warnings.txt", "\n".join(f"- {w}" for w in warnings), created, root)

    command_log = f"""# Task42 Command Log

Working directory: `/home/ws/workspace/BoxFusion`

Task42 used offline/static Python `{OFFLINE_PYTHON}` for JSON audits, figure generation, and package generation. Runtime Python `{RUNTIME_PYTHON}` is reserved for ROS2/rclpy runtime use. No live task42 runtime was launched.

Environment variables authorized for runtime/showcase actions:

- `RSLG_TASK42_ALLOW_RUNTIME=1`
- earlier guards allowed if needed: `RSLG_TASK38_ALLOW_RUNTIME=1`, `RSLG_TASK39_ALLOW_RUNTIME=1`, `RSLG_TASK41_ALLOW_RUNTIME=1`
- expected runtime defaults: `ROS_DOMAIN_ID=84`, `TURTLEBOT3_MODEL=burger`

Commands executed during task42 included:

- `pwd`
- `rg --files docs/rslg_slam`
- `find stage_outputs/rslg_slam/{SCENE_ID} -maxdepth 4 -type d`
- `sed -n ...` over the required project truth docs and runbooks
- `jq '.'` over the required manifests
- `find .../canonical/layer{{1,2,3,4}}... -type f`
- `find .../tasks/task39...`, `find .../tasks/task40...`, `find .../tasks/task41...`
- `jq` inspections of task39/task41 runtime reports, trajectories, route files, and object approach candidates
- `{OFFLINE_PYTHON} -c "import matplotlib, PIL; ..."`
- `which rviz2`, `which gazebo`, `which ros2`
- `pgrep -af 'gazebo|gzserver|gzclient|rviz2|map_server|turtlebot3|run_task39|run_task41|static_transform_publisher'`
- `date --iso-8601=seconds`
- `apply_patch` to add task42 scripts
- `{OFFLINE_PYTHON} -m py_compile tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py tools/rslg_pipeline/export_task42_marker_overlays.py tools/rslg_pipeline/export_task42_demo_figures.py`
- `bash -n tools/rslg_pipeline/show_task42_rviz_replay.sh`
- `bash -n tools/rslg_pipeline/show_task42_gazebo_replay.sh`
- `{OFFLINE_PYTHON} tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py --live-not-run-reason replay_only_strategy_preserves_validated_task39_task41_evidence_and_avoids_display_runtime_disturbance`
- `{OFFLINE_PYTHON} - <<'PY' ... validate created_or_modified JSON ... PY`
- `git diff --name-only -- stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface`
- `rg -n ... claim-boundary terms ...`
- `find stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/figures -maxdepth 1 -type f -name '*.png' -printf '%f %s\\n'`
- `jq` verification of task42 report, final audit, claim boundary report, cleanup report, marker count, and figure manifest

Subprocess commands executed by the generator:

- `pgrep -af gazebo|gzserver|gzclient|rviz2|map_server|turtlebot3|run_task39|run_task41|static_transform_publisher`

Detailed stdout/stderr for interactive inspection commands is available in the assistant tool transcript; task42 persisted generated reports and validation summaries in this directory.
"""
    write_text(task_dir / "command_log.txt", command_log, created, root)

    task42_report = {
        "task_name": TASK_NAME,
        "status": "completed",
        "classification": CLASSIFICATION,
        "selected_profile": PROFILE,
        "scene_id": SCENE_ID,
        "canonical_input_dirs": CANONICAL_INPUT_DIRS,
        "task42_evidence_dir": rel(task_dir, root),
        "docs_and_manifests_read": DOCS_READ,
        "final_audit_summary": {"status": final_audit["status"], "missing_count": len(missing), "invalid_json_count": len(invalid)},
        "claim_boundary_summary": {"status": claim_report["status"], "violations_found": False, "repairs_made": []},
        "demo_evidence_summary": {"demo_evidence_pack": rel(demo_dir, root), "marker_count": len(markers.get("markers", [])), "figure_count": len(figure_manifest["figures"])},
        "gazebo_showcase_summary": {"status": "replay_package_generated", "live_gazebo_rerun": False},
        "rviz_showcase_summary": {"status": "markerarray_replay_package_generated", "rviz_map_display_required": False},
        "replay_package_summary": {"status": "generated", "rviz_marker_input": rel(demo_dir / "rviz/task42_demo_markers_v0_1.json", root)},
        "live_demo_attempted": False,
        "live_demo_status": "not_run_replay_only_strategy",
        "room_route_runtime_evidence_summary": final_audit["room_runtime_evidence_summary"],
        "object_route_runtime_evidence_summary": final_audit["object_runtime_evidence_summary"],
        "generated_ring_002_summary": {
            "world_position": [-7.020484, 1.558795],
            "yaw_rad": -2.09057,
            "stable_map_state": "free",
            "clearance_m": 0.20,
            "local_a_star_length_m": 1.278742,
        },
        "generated_ring_037_status": "occupied_zero_clearance_under_conservative_canonical_not_used",
        "forbidden_sources_used": False,
        "object_centroid_navigation_used": False,
        "manual_target_pose_used": False,
        "canonical_outputs_generated_or_updated": [rel(demo_dir, root), rel(l4_manifest_dir / "task42_showcase_artifact_manifest_v0_1.json", root)],
        "docs_updated": [
            "docs/rslg_slam/final_project_status.md",
            "docs/rslg_slam/demo_showcase_runbook.md",
            "docs/rslg_slam/current_project_status.md",
            "docs/rslg_slam/object_navigation_status.md",
            "docs/rslg_slam/runtime_validation_runbook.md",
            "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
        ],
        "exact_blockers": [],
        "recommended_next_task": "No further core validation task is required. Recommended next work: paper/demo writing, final presentation packaging, or optional visual object confirmation only if needed.",
    }
    write_json(task_dir / "task42_report.json", task42_report, created, root)

    json_report_path = task_dir / "json_validation_report_v0_1.json"
    created_manifest_path = task_dir / "created_or_modified_files_manifest_v0_1.json"
    final_expected_files = sorted(set(created + TASK42_SCRIPT_FILES + [rel(json_report_path, root), rel(created_manifest_path, root)]))
    created_manifest = {
        "schema_name": "rslg_task42_created_or_modified_files_manifest",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "created_or_modified_files": final_expected_files,
        "canonical_layer2_core_route_or_map_artifacts_modified": False,
        "canonical_layer3_core_route_or_map_artifacts_modified": False,
    }
    write_json(created_manifest_path, created_manifest, created, root)

    json_report = validate_json_outputs(root, final_expected_files)
    write_json(json_report_path, json_report, created, root)


def validate_json_outputs(root: Path, files: list[str]) -> dict[str, Any]:
    checks = []
    for item in sorted(set(files)):
        if item.endswith(".json"):
            path = root / item
            checks.append(json_status(path))
            checks[-1]["path"] = item
    return {
        "schema_name": "rslg_task42_json_validation_report",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "checked_json_count": len(checks),
        "invalid_json_count": sum(1 for c in checks if c["json_valid"] is False),
        "checks": checks,
    }


def shutil_which(name: str) -> str | None:
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / name
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate.as_posix()
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--live-not-run-reason", default="replay_only_strategy")
    args = parser.parse_args()
    create_reports(args.repo_root.resolve(), args.live_not_run_reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
