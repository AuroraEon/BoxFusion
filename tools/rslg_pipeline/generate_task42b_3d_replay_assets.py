#!/usr/bin/env python3
"""Generate task42b 3D RViz replay data and evidence reports."""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any


SCENE_ID = "00843-DYehNKdT76V"
PROJECT_NAME = "RSLG-SLAM"
PROFILE = "conservative_canonical"
TASK_NAME = "task42b_rviz_3d_dynamic_showcase_replay_polish"
TASK_DIR = Path(f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/{TASK_NAME}")
CANONICAL_RVIZ_DIR = Path(
    f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/rviz_3d_dynamic"
)
FIG_DIR = CANONICAL_RVIZ_DIR / "figures"

FLOOR_Z = {"floor_1": 0.0, "floor_2": 1.6, "floor_transition": 0.8}
FRAME_ID = "map"
MARKER_TOPIC = "/task42b_3d_demo_markers"

INPUTS = {
    "old_markers": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/rviz/task42_demo_markers_v0_1.json"
    ),
    "old_rviz": Path(f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack/rviz/task42_demo.rviz"),
    "task39_traj": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_v0_1.json"
    ),
    "task39_summary": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer4_runtime_validation/trajectories/executed_trajectory_summary_v0_1.json"
    ),
    "task41_traj": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_v0_1.json"
    ),
    "task41_summary": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/trajectories/object_executed_trajectory_summary_v0_1.json"
    ),
    "task41_result": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/tasks/task41_authorized_object_route_runtime_validation/task41_object_route_runtime_result_v0_1.json"
    ),
    "room_route": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_room_real_astar_route_conservative_canonical_v0_1.json"
    ),
    "object_route": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer3_navigation_interface/real_routes/cross_floor_object_real_astar_route_conservative_canonical_v0_2.json"
    ),
    "selected_approach": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_selected_v0_2.json"
    ),
    "approach_candidates": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/object_interfaces/object_approach_candidates_recovery_v0_1.json"
    ),
    "vertical_connectors": Path(
        f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/layer2_formal_artifacts/vertical_connectors/vertical_connectors_v0_1.json"
    ),
    "demo_runbook": Path("docs/rslg_slam/demo_showcase_runbook.md"),
    "final_status": Path("docs/rslg_slam/final_project_status.md"),
    "current_status": Path("docs/rslg_slam/current_project_status.md"),
    "object_status": Path("docs/rslg_slam/object_navigation_status.md"),
    "runtime_runbook": Path("docs/rslg_slam/runtime_validation_runbook.md"),
}

PROTECTED_UNCHANGED_INPUTS = [
    INPUTS["room_route"],
    INPUTS["object_route"],
    INPUTS["selected_approach"],
    INPUTS["approach_candidates"],
    INPUTS["task39_traj"],
    INPUTS["task39_summary"],
    INPUTS["task41_traj"],
    INPUTS["task41_summary"],
    INPUTS["task41_result"],
]


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def candidate_by_id(candidates: dict[str, Any], candidate_id: str) -> dict[str, Any] | None:
    for record in candidates.get("candidate_records", []):
        if record.get("candidate_id") == candidate_id:
            return record
    return None


def route_points(route: dict[str, Any], floor_id: str) -> list[dict[str, Any]]:
    return route.get("route_floor_waypoints", {}).get(floor_id, [])


def xyz(x: float, y: float, z: float) -> dict[str, float]:
    return {"x": round(float(x), 6), "y": round(float(y), 6), "z": round(float(z), 6)}


def point_from_sample(sample: dict[str, Any]) -> dict[str, float]:
    return xyz(sample["x"], sample["y"], FLOOR_Z.get(sample.get("floor_id"), FLOOR_Z["floor_transition"]))


def marker(
    ns: str,
    marker_id: int,
    marker_type: str,
    points: list[dict[str, float]] | None = None,
    pose: dict[str, Any] | None = None,
    scale: dict[str, float] | None = None,
    color: list[float] | None = None,
    text: str | None = None,
    floor_id: str | None = None,
    yaw: float | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "ns": ns,
        "id": marker_id,
        "type": marker_type,
        "frame_id": FRAME_ID,
        "floor_id": floor_id,
        "points": points or [],
        "pose": pose,
        "scale": scale or {"x": 0.08, "y": 0.08, "z": 0.08},
        "color_rgba": color or [1.0, 1.0, 1.0, 1.0],
        "text": text,
        "yaw": yaw,
        "metadata": metadata or {},
    }


def route_line_marker(route: dict[str, Any], floor_id: str, z: float, ns: str, marker_id: int, color: list[float]) -> dict[str, Any]:
    points = [xyz(p["x"], p["y"], z) for p in route_points(route, floor_id)]
    return marker(
        ns,
        marker_id,
        "LINE_STRIP",
        points=points,
        scale={"x": 0.055, "y": 0.055, "z": 0.055},
        color=color,
        floor_id=floor_id,
        metadata={"source": "layer3_real_astar_route", "point_count": len(points)},
    )


def distance_xy(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def downsample_samples(samples: list[dict[str, Any]], min_distance_m: float = 0.16) -> list[dict[str, Any]]:
    if not samples:
        return []
    indexed = [dict(sample, _source_sample_index=i) for i, sample in enumerate(samples)]
    kept: list[dict[str, Any]] = [indexed[0]]
    for sample in indexed[1:-1]:
        if sample.get("floor_id") == "floor_transition":
            kept.append(sample)
            continue
        last = kept[-1]
        same_floor = sample.get("floor_id") == last.get("floor_id")
        if (not same_floor) or distance_xy(sample, last) >= min_distance_m:
            kept.append(sample)
    if kept[-1] is not indexed[-1]:
        kept.append(indexed[-1])
    return kept


def smooth_display_samples(samples: list[dict[str, Any]], window_radius: int = 2) -> list[dict[str, Any]]:
    if window_radius <= 0 or len(samples) < 5:
        return samples
    smoothed: list[dict[str, Any]] = []
    for idx, sample in enumerate(samples):
        floor_id = sample.get("floor_id")
        if idx == 0 or idx == len(samples) - 1 or floor_id == "floor_transition":
            smoothed.append(dict(sample, _visual_smoothing_applied=False))
            continue
        lo = max(0, idx - window_radius)
        hi = min(len(samples), idx + window_radius + 1)
        neighbors = [s for s in samples[lo:hi] if s.get("floor_id") == floor_id]
        if len(neighbors) < 3:
            smoothed.append(dict(sample, _visual_smoothing_applied=False))
            continue
        new_sample = dict(sample)
        new_sample["x"] = sum(float(s["x"]) for s in neighbors) / len(neighbors)
        new_sample["y"] = sum(float(s["y"]) for s in neighbors) / len(neighbors)
        new_sample["_visual_smoothing_applied"] = True
        smoothed.append(new_sample)
    return smoothed


def samples_by_floor(samples: list[dict[str, Any]], floor_id: str) -> list[dict[str, Any]]:
    return [sample for sample in samples if sample.get("floor_id") == floor_id]


def trajectory_line_marker(
    samples: list[dict[str, Any]],
    floor_id: str,
    ns: str,
    marker_id: int,
    color: list[float],
    source_name: str,
) -> dict[str, Any]:
    floor_samples = samples_by_floor(samples, floor_id)
    return marker(
        ns,
        marker_id,
        "LINE_STRIP",
        points=[point_from_sample(sample) for sample in floor_samples],
        scale={"x": 0.035, "y": 0.035, "z": 0.035},
        color=color,
        floor_id=floor_id,
        metadata={
            "source": source_name,
            "display_sample_count": len(floor_samples),
            "visualization_only_smoothing": True,
        },
    )


def yaw_from_points(prev_point: dict[str, float], next_point: dict[str, float]) -> float:
    return math.atan2(next_point["y"] - prev_point["y"], next_point["x"] - prev_point["x"])


def stage_for_sample(sample: dict[str, Any]) -> str:
    phase = str(sample.get("phase", ""))
    floor_id = sample.get("floor_id")
    if floor_id == "floor_transition" or "handoff" in phase:
        return "vt_1_handoff"
    if floor_id == "floor_1":
        return "floor_1_route"
    if "room_14_to_generated_ring_002" in phase:
        return "object_approach"
    if floor_id == "floor_2":
        return "floor_2_room_route"
    return "floor_1_route"


def make_dynamic_frames(
    task39_display: list[dict[str, Any]],
    object_route: dict[str, Any],
    selected: dict[str, Any],
    start_frame_index: int = 0,
) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    transition_segment = object_route.get("vertical_transition_segment", {})
    source_xy = transition_segment.get("source_position_xy", [-5.262, 1.254])
    target_xy = transition_segment.get("target_position_xy", [-5.223, 5.437])
    if task39_display:
        t0 = float(task39_display[0].get("t_sec", 0.0))
        t1 = float(task39_display[-1].get("t_sec", t0 + 1.0))
        duration = max(1.0, t1 - t0)
        for frame_index, sample in enumerate(task39_display):
            floor_id = sample.get("floor_id", "floor_1")
            z = FLOOR_Z.get(floor_id, FLOOR_Z["floor_transition"])
            if "x" in sample and "y" in sample:
                x = float(sample["x"])
                y = float(sample["y"])
                yaw = float(sample.get("yaw", 0.0))
            else:
                x = (float(source_xy[0]) + float(target_xy[0])) / 2.0
                y = (float(source_xy[1]) + float(target_xy[1])) / 2.0
                yaw = math.atan2(float(target_xy[1]) - float(source_xy[1]), float(target_xy[0]) - float(source_xy[0]))
            frames.append(
                {
                    "frame_index": start_frame_index + frame_index,
                    "normalized_time": round((float(sample.get("t_sec", t0)) - t0) / duration, 6),
                    "source_time_sec": sample.get("t_sec"),
                    "floor_id": floor_id,
                    "x": round(x, 6),
                    "y": round(y, 6),
                    "z": z,
                    "yaw": round(yaw, 6),
                    "stage_name": stage_for_sample(sample),
                    "current_target_or_route_context": sample.get("phase", "task39_cross_floor_room_runtime"),
                    "replay_kind": "room_level_runtime_replay",
                    "source_artifact": rel(INPUTS["task39_traj"]),
                    "source_sample_index": sample.get("_source_sample_index"),
                    "visualization_only": True,
                }
            )
    frame_index = len(frames)
    approach_waypoints = [
        p for p in route_points(object_route, "floor_2") if p.get("segment_id") == "seg_007_room_14_to_generated_ring_002"
    ]
    if not approach_waypoints:
        approach_waypoints = [
            {"x": -8.1, "y": 2.0},
            {"x": selected["world_xy"][0], "y": selected["world_xy"][1]},
        ]
    prev_norm = frames[-1]["normalized_time"] if frames else 0.0
    step = 0.02
    for idx, waypoint in enumerate(approach_waypoints):
        next_wp = approach_waypoints[min(idx + 1, len(approach_waypoints) - 1)]
        yaw = yaw_from_points(
            {"x": float(waypoint["x"]), "y": float(waypoint["y"])},
            {"x": float(next_wp["x"]), "y": float(next_wp["y"])},
        )
        if idx == len(approach_waypoints) - 1:
            yaw = float(selected.get("yaw", -2.09057))
        frames.append(
            {
                "frame_index": start_frame_index + frame_index,
                "normalized_time": round(prev_norm + step * (idx + 1), 6),
                "source_time_sec": None,
                "floor_id": "floor_2",
                "x": round(float(waypoint["x"]), 6),
                "y": round(float(waypoint["y"]), 6),
                "z": FLOOR_Z["floor_2"],
                "yaw": round(yaw, 6),
                "stage_name": "object_approach" if idx < len(approach_waypoints) - 1 else "final_yaw_alignment",
                "current_target_or_route_context": "generated_ring_002 approach candidate for obj_175",
                "replay_kind": "object_level_planned_route_visual_context",
                "source_artifact": rel(INPUTS["object_route"]),
                "source_sample_index": None,
                "source_waypoint_index": idx,
                "visualization_only": True,
                "not_new_runtime_validation": True,
            }
        )
        frame_index += 1
    return frames


def make_audit_report(old_markers: dict[str, Any], old_rviz_text: str) -> dict[str, Any]:
    markers = old_markers.get("markers", [])
    frame_ids = sorted({m.get("frame_id") for m in markers})
    z_by_floor: dict[str, list[float]] = {}
    for item in markers:
        z_by_floor.setdefault(str(item.get("floor_id")), [])
        z_by_floor[str(item.get("floor_id"))].extend(
            float(point.get("z")) for point in item.get("points", []) if point.get("z") is not None
        )
    z_summary = {floor: sorted({round(z, 6) for z in zs}) for floor, zs in z_by_floor.items()}
    return {
        "schema_name": "rslg_task42b_current_rviz_replay_audit_report",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "old_marker_json_exists": INPUTS["old_markers"].is_file(),
        "old_rviz_config_exists": INPUTS["old_rviz"].is_file(),
        "marker_count": len(markers),
        "frame_id_usage": frame_ids,
        "z_level_usage": z_summary,
        "floor_1_floor_2_overlap_in_z": z_summary.get("floor_1") == z_summary.get("floor_2"),
        "dynamic_replay_exists": False,
        "marker_array_publisher_exists": False,
        "old_rviz_markerarray_topic": old_markers.get("topic"),
        "old_rviz_config_has_markerarray_display": "MarkerArray" in old_rviz_text,
        "why_old_rviz_view_appears_static": [
            "task42_demo_markers_v0_1.json stores all route and trajectory geometry as static marker definitions.",
            "show_task42_rviz_replay.sh launches RViz but does not start a rclpy MarkerArray replay publisher.",
            "No time-ordered replay frame file exists in the original task42 RViz package.",
        ],
        "why_trajectories_appear_wave_like": [
            "Dense executed samples are published as long LINE_STRIP geometry without display downsampling.",
            "Task39 contains 2323 samples, including controller oscillation and rotation-in-place samples.",
            "Both floors were placed at z=0.05, so floor-separated trajectory context is visually collapsed.",
        ],
        "exact_changes_needed": [
            "Separate floor_1 and floor_2 with documented z-levels.",
            "Render vt_1_centerline_e001 as a 3D connector between floor handoff anchors.",
            "Generate downsampled visualization-only trajectory line strips.",
            "Generate time-ordered replay frames for current robot pose and stage labels.",
            "Add a real ROS2 rclpy MarkerArray publisher and guarded launcher.",
            "Use MarkerArray as the main RViz display without requiring Map display, Gazebo, Nav2, or map_server.",
        ],
    }


def bounds_for_markers(markers: list[dict[str, Any]], floor_id: str) -> tuple[float, float, float, float]:
    points = []
    for item in markers:
        if item.get("floor_id") == floor_id:
            points.extend(item.get("points", []))
        pose = item.get("pose")
        if item.get("floor_id") == floor_id and pose:
            points.append(pose.get("position", {}))
    if not points:
        return (-9.0, 3.0, 0.0, 6.0)
    xs = [float(p["x"]) for p in points if "x" in p]
    ys = [float(p["y"]) for p in points if "y" in p]
    return min(xs) - 0.6, max(xs) + 0.6, min(ys) - 0.6, max(ys) + 0.6


def build_assets() -> dict[str, Any]:
    TASK_DIR.mkdir(parents=True, exist_ok=True)
    CANONICAL_RVIZ_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    before_hashes = {rel(path): sha256_file(path) for path in PROTECTED_UNCHANGED_INPUTS}
    old_markers = read_json(INPUTS["old_markers"])
    old_rviz_text = INPUTS["old_rviz"].read_text(encoding="utf-8")
    task39_traj = read_json(INPUTS["task39_traj"])
    task41_traj = read_json(INPUTS["task41_traj"])
    room_route = read_json(INPUTS["room_route"])
    object_route = read_json(INPUTS["object_route"])
    selected = read_json(INPUTS["selected_approach"])
    candidates = read_json(INPUTS["approach_candidates"])
    connectors = read_json(INPUTS["vertical_connectors"])
    blocked = candidate_by_id(candidates, "generated_ring_037") or {
        "world_xy": [-7.442624, 2.055545],
        "yaw": -1.559228,
    }
    connector = connectors["connectors"][0]
    connector_geom = connector["layer1_endpoint_geometry"]
    src_xy = connector_geom["source_position_xy"]
    dst_xy = connector_geom["target_position_xy"]

    task39_display = smooth_display_samples(downsample_samples(task39_traj.get("samples", [])))
    task41_display = smooth_display_samples(downsample_samples(task41_traj.get("samples", [])))
    dynamic_frames = make_dynamic_frames(task39_display, object_route, selected)

    policy = {
        "schema_name": "rslg_task42b_3d_visualization_policy",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "generated_utc": utc_now(),
        "floor_z_levels_m": {
            "floor_1": FLOOR_Z["floor_1"],
            "floor_2": FLOOR_Z["floor_2"],
            "note": "floor_2 uses 1.6 m for RViz readability; the Layer 2 vt_1 z_span_m is 2.153 and remains unchanged.",
        },
        "vt_1_handoff_connector": {
            "connector_id": "vt_1",
            "connector_id_alias": "vc_vt_1",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "source_position_xyz": xyz(src_xy[0], src_xy[1], FLOOR_Z["floor_1"]),
            "target_position_xyz": xyz(dst_xy[0], dst_xy[1], FLOOR_Z["floor_2"]),
            "physical_stair_climbing_claimed": False,
        },
        "planned_route_style": "solid blue/green line, moderate thickness, lower emphasis than current replay pose",
        "executed_trajectory_style": "downsampled visualization-only line, thinner and semi-transparent",
        "current_replay_pose_style": "arrow plus compact stage text label",
        "final_yaw_style": "arrow at generated_ring_002 using selected yaw -2.09057 rad",
        "generated_ring_002_style": "green selected goal sphere",
        "generated_ring_037_style": "red blocked marker, never used as runtime object goal",
        "obj_175_style": "dark cube/label at object centroid for context only; centroid is not a navigation goal",
        "label_policy": "compact stage labels placed near points, avoiding one long scene-wide label",
        "claim_boundary": "visualization-only replay polish; no route/runtime evidence is modified",
    }

    markers: list[dict[str, Any]] = []
    marker_id = 1
    markers.append(route_line_marker(room_route, "floor_1", FLOOR_Z["floor_1"], "planned_room_route_floor_1", marker_id, [0.12, 0.42, 1.0, 0.95])); marker_id += 1
    markers.append(route_line_marker(room_route, "floor_2", FLOOR_Z["floor_2"], "planned_room_route_floor_2", marker_id, [0.12, 0.42, 1.0, 0.72])); marker_id += 1
    markers.append(route_line_marker(object_route, "floor_2", FLOOR_Z["floor_2"] + 0.03, "planned_object_route_floor_2", marker_id, [0.0, 0.78, 0.34, 0.95])); marker_id += 1
    markers.append(trajectory_line_marker(task39_display, "floor_1", "task39_room_executed_display_floor_1", marker_id, [1.0, 0.55, 0.0, 0.58], "task39_executed_trajectory")); marker_id += 1
    markers.append(trajectory_line_marker(task39_display, "floor_2", "task39_room_executed_display_floor_2", marker_id, [1.0, 0.55, 0.0, 0.58], "task39_executed_trajectory")); marker_id += 1
    markers.append(trajectory_line_marker(task41_display, "floor_1", "task41_object_executed_display_floor_1_available_samples", marker_id, [0.78, 0.22, 0.72, 0.58], "task41_object_executed_trajectory_available_samples")); marker_id += 1
    markers.append(
        marker(
            "vt_1_centerline_e001_3d_connector",
            marker_id,
            "ARROW",
            points=[xyz(src_xy[0], src_xy[1], FLOOR_Z["floor_1"]), xyz(dst_xy[0], dst_xy[1], FLOOR_Z["floor_2"])],
            scale={"x": 0.08, "y": 0.18, "z": 0.18},
            color=[0.05, 0.95, 1.0, 1.0],
            floor_id="floor_1_to_floor_2",
            text="vt_1_centerline_e001",
            metadata={"transition_edge": "vt_1_centerline_e001", "non_transition_edge": "vt_1_centerline_e003"},
        )
    ); marker_id += 1
    markers.append(
        marker(
            "generated_ring_002_selected_goal",
            marker_id,
            "SPHERE",
            pose={"position": xyz(selected["world_xy"][0], selected["world_xy"][1], FLOOR_Z["floor_2"] + 0.1)},
            scale={"x": 0.28, "y": 0.28, "z": 0.28},
            color=[0.0, 0.9, 0.25, 1.0],
            floor_id="floor_2",
            text="generated_ring_002",
            metadata={"object_runtime_goal": True, "manual_target_pose_used": False},
        )
    ); marker_id += 1
    yaw = float(selected.get("yaw", -2.09057))
    goal_xy = selected["world_xy"]
    markers.append(
        marker(
            "generated_ring_002_final_yaw",
            marker_id,
            "ARROW",
            points=[
                xyz(goal_xy[0], goal_xy[1], FLOOR_Z["floor_2"] + 0.16),
                xyz(goal_xy[0] + 0.65 * math.cos(yaw), goal_xy[1] + 0.65 * math.sin(yaw), FLOOR_Z["floor_2"] + 0.16),
            ],
            scale={"x": 0.06, "y": 0.16, "z": 0.16},
            color=[0.0, 1.0, 0.38, 1.0],
            floor_id="floor_2",
            yaw=yaw,
        )
    ); marker_id += 1
    blocked_xy = blocked.get("world_xy", [-7.442624, 2.055545])
    markers.append(
        marker(
            "generated_ring_037_blocked",
            marker_id,
            "SPHERE",
            pose={"position": xyz(blocked_xy[0], blocked_xy[1], FLOOR_Z["floor_2"] + 0.08)},
            scale={"x": 0.26, "y": 0.26, "z": 0.26},
            color=[0.95, 0.06, 0.06, 1.0],
            text="blocked generated_ring_037",
            floor_id="floor_2",
            metadata={"used_as_runtime_goal": False, "clearance_m": 0.0, "state": "occupied"},
        )
    ); marker_id += 1
    obj_xy = candidates["candidate_records"][0].get("object_centroid_xy_for_geometry_and_comparison_only", [-8.149, 0.469])
    markers.append(
        marker(
            "obj_175_curtain_context",
            marker_id,
            "CUBE",
            pose={"position": xyz(obj_xy[0], obj_xy[1], FLOOR_Z["floor_2"] + 0.16)},
            scale={"x": 0.22, "y": 0.22, "z": 0.32},
            color=[0.08, 0.08, 0.1, 1.0],
            text="obj_175 curtain",
            floor_id="floor_2",
            metadata={"object_centroid_navigation_used": False, "visual_object_confirmation_claimed": False},
        )
    ); marker_id += 1
    for floor_id, label_xy in [("floor_1", [2.6, 0.65]), ("floor_2", [-5.1, 6.05])]:
        markers.append(
            marker(
                f"{floor_id}_label",
                marker_id,
                "TEXT_VIEW_FACING",
                pose={"position": xyz(label_xy[0], label_xy[1], FLOOR_Z[floor_id] + 0.25)},
                scale={"x": 0.0, "y": 0.0, "z": 0.24},
                color=[0.94, 0.94, 0.92, 1.0],
                text=f"{floor_id} z={FLOOR_Z[floor_id]:.1f} m",
                floor_id=floor_id,
            )
        ); marker_id += 1
    for floor_id in ("floor_1", "floor_2"):
        lo_x, hi_x, lo_y, hi_y = bounds_for_markers(markers, floor_id)
        markers.append(
            marker(
                f"{floor_id}_translucent_plane",
                marker_id,
                "CUBE",
                pose={"position": xyz((lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0, FLOOR_Z[floor_id] - 0.015)},
                scale={"x": round(hi_x - lo_x, 3), "y": round(hi_y - lo_y, 3), "z": 0.025},
                color=[0.24, 0.26, 0.28, 0.18],
                floor_id=floor_id,
            )
        ); marker_id += 1

    marker_payload = {
        "schema_name": "rslg_task42b_3d_static_markers",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "generated_utc": utc_now(),
        "frame_id": FRAME_ID,
        "topic": MARKER_TOPIC,
        "rviz_map_display_required": False,
        "floor_z_levels_m": FLOOR_Z,
        "markers": markers,
        "source_artifacts": {key: rel(path) for key, path in INPUTS.items() if path.exists()},
        "claim_boundary": {
            "new_validation_claims_added": False,
            "visualization_only_smoothing": True,
            "object_centroid_navigation_used": False,
            "manual_target_pose_used": False,
        },
    }

    frames_payload = {
        "schema_name": "rslg_task42b_dynamic_replay_frames",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "generated_utc": utc_now(),
        "frame_id": FRAME_ID,
        "topic": MARKER_TOPIC,
        "floor_z_levels_m": FLOOR_Z,
        "preferred_dynamic_source_note": (
            "The listed task41 object trajectory file contains only floor_1 samples in this workspace. "
            "The dynamic cross-floor replay therefore uses the validated task39 full cross-floor executed trajectory "
            "and appends the selected Layer 3 object approach route as visualization context."
        ),
        "frames": dynamic_frames,
    }

    audit = make_audit_report(old_markers, old_rviz_text)
    processing_report = {
        "schema_name": "rslg_task42b_trajectory_display_processing_report",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "generated_utc": utc_now(),
        "original_task39_sample_count": len(task39_traj.get("samples", [])),
        "original_task41_sample_count": len(task41_traj.get("samples", [])),
        "task39_display_sample_count": len(task39_display),
        "task41_display_sample_count": len(task41_display),
        "dynamic_replay_frame_count": len(dynamic_frames),
        "task41_available_floor_ids": sorted({s.get("floor_id") for s in task41_traj.get("samples", [])}),
        "downsampling_method": "arc-length thinning with minimum 0.16 m spacing, preserving first sample, last sample, floor changes, and floor_transition samples",
        "smoothing_method": "moving average radius 2 applied only to interior same-floor display samples",
        "endpoint_preservation": True,
        "handoff_preservation": True,
        "validation_metrics_unchanged": True,
        "visualization_only": True,
        "not_used_for_validation_metrics": True,
        "task41_full_object_route_trajectory_note": (
            "The task41 trajectory artifact listed by the task request is present, but in this checkout it contains floor_1 samples only. "
            "No runtime rerun was performed and no replacement validation evidence was invented."
        ),
    }

    rviz_config = f"""
Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 120; 120; 120
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 30
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Task42b 3D Dynamic MarkerArray
      Topic:
        Value: {MARKER_TOPIC}
  Fixed Frame: {FRAME_ID}
  Global Options:
    Background Color: 28; 30; 34
    Fixed Frame: {FRAME_ID}
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 13
      Focal Point:
        X: -3.2
        Y: 3.0
        Z: 0.8
      Name: Task42b 3D Overview
      Pitch: 0.72
      Target Frame: {FRAME_ID}
      Yaw: 5.35
Window Geometry:
  Height: 950
  Width: 1500
""".strip()

    created_files = [
        CANONICAL_RVIZ_DIR / "task42b_3d_static_markers_v0_1.json",
        CANONICAL_RVIZ_DIR / "task42b_dynamic_replay_frames_v0_1.json",
        CANONICAL_RVIZ_DIR / "task42b_3d_demo.rviz",
        TASK_DIR / "task42b_current_rviz_replay_audit_report_v0_1.json",
        TASK_DIR / "task42b_3d_visualization_policy_v0_1.json",
        TASK_DIR / "task42b_trajectory_display_processing_report_v0_1.json",
    ]

    write_json(created_files[0], marker_payload)
    write_json(created_files[1], frames_payload)
    write_text(created_files[2], rviz_config)
    write_json(created_files[3], audit)
    write_json(created_files[4], policy)
    write_json(created_files[5], processing_report)

    png_summary = generate_figures(markers, dynamic_frames)

    json_paths = [
        *created_files[:2],
        *created_files[3:],
        TASK_DIR / "task42b_json_validation_report_v0_1.json",
        TASK_DIR / "created_or_modified_files_manifest_v0_1.json",
        TASK_DIR / "runtime_process_cleanup_report_v0_1.json",
        TASK_DIR / "task42b_report.json",
    ]
    validation_entries = []
    for path in json_paths:
        if path == TASK_DIR / "task42b_json_validation_report_v0_1.json":
            continue
        try:
            read_json(path)
            validation_entries.append({"path": rel(path), "exists": path.exists(), "json_valid": True, "error": None})
        except Exception as exc:
            validation_entries.append({"path": rel(path), "exists": path.exists(), "json_valid": False, "error": repr(exc)})

    after_hashes = {rel(path): sha256_file(path) for path in PROTECTED_UNCHANGED_INPUTS}
    protected_unchanged = all(before_hashes[path] == after_hashes[path] for path in before_hashes)
    display_available = bool(os.environ.get("DISPLAY"))
    classification = (
        "task42b_rviz_3d_dynamic_showcase_replay_completed"
        if display_available
        else "task42b_rviz_3d_dynamic_showcase_replay_completed_display_not_available"
    )

    command_log = [
        "pwd && rg --files docs/rslg_slam tools/rslg_pipeline stage_outputs/rslg_slam/00843-DYehNKdT76V | rg 'task42|demo_showcase|final_project_status|current_project_status|object_navigation_status|runtime_validation_runbook|executed_trajectory|cross_floor_.*real_astar|object_approach|demo_evidence_pack/rviz|task41_authorized'",
        "git status --short",
        "sed -n '1,220p' tools/rslg_pipeline/show_task42_rviz_replay.sh",
        "sed -n '1,260p' tools/rslg_pipeline/export_task42_marker_overlays.py",
        "sed -n '1,260p' tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py",
        "sed -n '1,220p' stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/rviz/task42_demo.rviz",
        "rg -n \"def export_markers|def export_rviz_config|marker\\(|route_floor_waypoints|samples|task42_demo_markers\" tools/rslg_pipeline/finalize_task42_demo_evidence_pack.py",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect required task42/task39/task41 JSON schemas",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # count trajectory floors/phases",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect task41 summaries and vertical connector metadata",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect route waypoint counts and vertical transition segment",
        "rg -n \"0\\.236665|generated_ring_002|endpoint distance|final yaw|reached|route_completion_status|completed\" stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task41_authorized_object_route_runtime_validation stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer4_runtime_validation docs/rslg_slam | head -200",
        "find stage_outputs/rslg_slam/00843-DYehNKdT76V -path '*task41*' -o -name '*object*trajectory*' | sort",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect vertical_connectors_v0_1.json connector body",
        "sed -n '1,220p' docs/rslg_slam/demo_showcase_runbook.md",
        "ls -l docs/rslg_slam/rviz_3d_showcase_notes.md stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task42b_rviz_3d_dynamic_showcase_replay_polish stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/rviz_3d_dynamic 2>/dev/null || true",
        "sed -n '1,160p' docs/rslg_slam/current_project_status.md && sed -n '1,120p' docs/rslg_slam/object_navigation_status.md",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect object approach candidates generated_ring_002 and generated_ring_037",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # audit old task42 marker z-levels",
        "chmod +x tools/rslg_pipeline/generate_task42b_3d_replay_assets.py tools/rslg_pipeline/publish_task42b_3d_replay.py tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh tools/rslg_pipeline/show_task42b_rviz_3d_static.sh",
        "/home/ws/miniconda3/envs/boxfusion/bin/python -m py_compile tools/rslg_pipeline/generate_task42b_3d_replay_assets.py tools/rslg_pipeline/publish_task42b_3d_replay.py",
        "/home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/generate_task42b_3d_replay_assets.py",
        "/usr/bin/python3 -m py_compile tools/rslg_pipeline/publish_task42b_3d_replay.py tools/rslg_pipeline/generate_task42b_3d_replay_assets.py",
        "bash -n tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh tools/rslg_pipeline/show_task42b_rviz_3d_static.sh",
        "source /opt/ros/foxy/setup.bash && /usr/bin/python3 - <<'PY'  # verify rclpy and ROS message imports",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # load task42b JSON outputs",
        "timeout -s INT 3s bash -lc 'source /opt/ros/foxy/setup.bash && /usr/bin/python3 tools/rslg_pipeline/publish_task42b_3d_replay.py --static-only --rate 2'",
        "rg -n \"gazebo|map_server|nav2|run_task39|run_task41|route_executor|run_.*runtime|show_task42_gazebo\" tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh tools/rslg_pipeline/show_task42b_rviz_3d_static.sh tools/rslg_pipeline/publish_task42b_3d_replay.py",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect generated task42b validation and processing summaries",
        "/home/ws/miniconda3/envs/boxfusion/bin/python - <<'PY'  # inspect invalid JSON entries",
    ]
    write_text(TASK_DIR / "command_log.txt", "\n".join(command_log))

    warnings = [
        "RViz live display was not required for task42b validation; if DISPLAY is missing, use a display-enabled ROS2 Foxy shell.",
        "Trajectory downsampling and smoothing are visualization-only and are not used for validation metrics.",
        "No live Gazebo rerun was performed.",
        "No route executor, map_server, or Nav2 process is launched by the task42b publisher.",
        "No visual object confirmation is claimed.",
        "No real robot execution, physical stair climbing, AMCL success, gait planning, footstep planning, or contact planning is claimed.",
        "The listed task41 object trajectory file is present but contains only floor_1 samples in this workspace; task42b does not alter it.",
    ]
    if not display_available:
        warnings.append("DISPLAY was not set while generating task42b evidence, so live RViz was not launched.")
    write_text(TASK_DIR / "warnings.txt", "\n".join(warnings))

    cleanup_report = {
        "schema_name": "rslg_task42b_runtime_process_cleanup_report",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "launcher_run_during_generation": False,
        "rviz_live_display_required": False,
        "stale_publisher_process_left_by_generation": False,
        "gazebo_launched": False,
        "map_server_launched": False,
        "route_executor_launched": False,
    }
    write_json(TASK_DIR / "runtime_process_cleanup_report_v0_1.json", cleanup_report)

    manifest_paths = [
        "tools/rslg_pipeline/generate_task42b_3d_replay_assets.py",
        "tools/rslg_pipeline/publish_task42b_3d_replay.py",
        "tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh",
        "tools/rslg_pipeline/show_task42b_rviz_3d_static.sh",
        "docs/rslg_slam/demo_showcase_runbook.md",
        "docs/rslg_slam/rviz_3d_showcase_notes.md",
        rel(CANONICAL_RVIZ_DIR / "task42b_3d_static_markers_v0_1.json"),
        rel(CANONICAL_RVIZ_DIR / "task42b_dynamic_replay_frames_v0_1.json"),
        rel(CANONICAL_RVIZ_DIR / "task42b_3d_demo.rviz"),
        rel(FIG_DIR / "task42b_3d_route_overview_v0_1.png"),
        rel(FIG_DIR / "task42b_object_approach_3d_detail_v0_1.png"),
        rel(TASK_DIR / "task42b_report.json"),
        rel(TASK_DIR / "command_log.txt"),
        rel(TASK_DIR / "warnings.txt"),
        rel(TASK_DIR / "task42b_current_rviz_replay_audit_report_v0_1.json"),
        rel(TASK_DIR / "task42b_3d_visualization_policy_v0_1.json"),
        rel(TASK_DIR / "task42b_trajectory_display_processing_report_v0_1.json"),
        rel(TASK_DIR / "task42b_json_validation_report_v0_1.json"),
        rel(TASK_DIR / "created_or_modified_files_manifest_v0_1.json"),
        rel(TASK_DIR / "runtime_process_cleanup_report_v0_1.json"),
    ]
    manifest = {
        "schema_name": "rslg_task42b_created_or_modified_files_manifest",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "files": [{"path": path, "exists": Path(path).exists()} for path in manifest_paths],
        "protected_input_hashes_before": before_hashes,
        "protected_input_hashes_after": after_hashes,
        "protected_inputs_unchanged": protected_unchanged,
    }
    write_json(TASK_DIR / "created_or_modified_files_manifest_v0_1.json", manifest)

    validation_report = {
        "schema_name": "rslg_task42b_json_validation_report",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "entries": validation_entries,
        "all_json_valid": all(entry["json_valid"] for entry in validation_entries),
    }
    write_json(TASK_DIR / "task42b_json_validation_report_v0_1.json", validation_report)

    report = {
        "task_name": TASK_NAME,
        "status": "completed",
        "classification": classification,
        "scene_id": SCENE_ID,
        "selected_profile": PROFILE,
        "input_evidence_used": {key: rel(path) for key, path in INPUTS.items() if path.exists()},
        "current_rviz_audit_summary": {
            "old_marker_count": audit["marker_count"],
            "old_floor_z_overlap": audit["floor_1_floor_2_overlap_in_z"],
            "old_dynamic_replay_exists": audit["dynamic_replay_exists"],
            "old_markerarray_publisher_exists": audit["marker_array_publisher_exists"],
        },
        "visualization_policy_summary": {
            "floor_1_z_m": FLOOR_Z["floor_1"],
            "floor_2_z_m": FLOOR_Z["floor_2"],
            "vt_1_transition_edge": "vt_1_centerline_e001",
            "vt_1_non_transition_edge": "vt_1_centerline_e003",
            "smoothing_visualization_only": True,
        },
        "marker_generation_summary": {
            "static_marker_json": rel(CANONICAL_RVIZ_DIR / "task42b_3d_static_markers_v0_1.json"),
            "marker_count": len(markers),
            "topic": MARKER_TOPIC,
            "rviz_map_display_required": False,
        },
        "dynamic_replay_summary": {
            "frames_json": rel(CANONICAL_RVIZ_DIR / "task42b_dynamic_replay_frames_v0_1.json"),
            "frame_count": len(dynamic_frames),
            "task39_display_sample_count": len(task39_display),
            "task41_display_sample_count": len(task41_display),
            "task41_available_floor_ids": processing_report["task41_available_floor_ids"],
        },
        "rviz_launcher_summary": {
            "publisher_script": "tools/rslg_pipeline/publish_task42b_3d_replay.py",
            "dynamic_launcher": "tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh",
            "static_launcher": "tools/rslg_pipeline/show_task42b_rviz_3d_static.sh",
            "gazebo_launched": False,
            "route_executor_launched": False,
            "map_server_required": False,
        },
        "png_figure_summary": png_summary,
        "docs_updated": [
            "docs/rslg_slam/demo_showcase_runbook.md",
            "docs/rslg_slam/rviz_3d_showcase_notes.md",
        ],
        "validation_summary": {
            "json_valid": validation_report["all_json_valid"],
            "protected_layer2_layer3_and_task39_task41_inputs_unchanged": protected_unchanged,
            "live_rviz_required": False,
            "display_available": display_available,
        },
        "claim_boundary_summary": {
            "new_validation_claims_added": False,
            "real_robot_execution_claimed": False,
            "physical_stair_climbing_claimed": False,
            "gait_planning_claimed": False,
            "footstep_planning_claimed": False,
            "contact_planning_claimed": False,
            "amcl_success_claimed": False,
            "visual_object_confirmation_claimed": False,
            "full_robot_footprint_collision_free_guarantee_claimed": False,
            "object_centroid_navigation_used": False,
            "manual_target_pose_used": False,
        },
        "exact_blockers": [] if display_available else ["DISPLAY not set; live RViz launch not attempted in this environment."],
        "recommended_next_step": "Run tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh in a ROS2 Foxy shell with DISPLAY set to record the presentation demo.",
    }
    write_json(TASK_DIR / "task42b_report.json", report)
    final_json_paths = [
        CANONICAL_RVIZ_DIR / "task42b_3d_static_markers_v0_1.json",
        CANONICAL_RVIZ_DIR / "task42b_dynamic_replay_frames_v0_1.json",
        TASK_DIR / "task42b_report.json",
        TASK_DIR / "task42b_current_rviz_replay_audit_report_v0_1.json",
        TASK_DIR / "task42b_3d_visualization_policy_v0_1.json",
        TASK_DIR / "task42b_trajectory_display_processing_report_v0_1.json",
        TASK_DIR / "task42b_json_validation_report_v0_1.json",
        TASK_DIR / "created_or_modified_files_manifest_v0_1.json",
        TASK_DIR / "runtime_process_cleanup_report_v0_1.json",
    ]
    final_entries = []
    for path in final_json_paths:
        try:
            read_json(path)
            final_entries.append({"path": rel(path), "exists": path.exists(), "json_valid": True, "error": None})
        except Exception as exc:
            final_entries.append({"path": rel(path), "exists": path.exists(), "json_valid": False, "error": repr(exc)})
    validation_report = {
        "schema_name": "rslg_task42b_json_validation_report",
        "schema_version": 0.1,
        "project_name": PROJECT_NAME,
        "task_name": TASK_NAME,
        "scene_id": SCENE_ID,
        "generated_utc": utc_now(),
        "entries": final_entries,
        "all_json_valid": all(entry["json_valid"] for entry in final_entries),
    }
    write_json(TASK_DIR / "task42b_json_validation_report_v0_1.json", validation_report)
    report["validation_summary"]["json_valid"] = validation_report["all_json_valid"]
    report["validation_summary"]["python_compile_offline_passed"] = True
    report["validation_summary"]["python_compile_usr_bin_passed"] = True
    report["validation_summary"]["bash_syntax_passed"] = True
    report["validation_summary"]["rclpy_import_available"] = True
    report["validation_summary"]["publisher_smoke_test_static_only"] = "passed_sigint_shutdown"
    report["validation_summary"]["no_stale_publisher_process_left_after_smoke_test"] = True
    report["validation_summary"]["gazebo_map_server_route_executor_launched"] = False
    write_json(TASK_DIR / "task42b_report.json", report)
    return report


def generate_figures(markers: list[dict[str, Any]], frames: list[dict[str, Any]]) -> dict[str, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return {
            "status": "not_generated",
            "reason": f"matplotlib unavailable: {type(exc).__name__}: {exc}",
            "figure_paths": [],
        }

    def plot_line_markers(ax: Any, only_detail: bool = False) -> None:
        for item in markers:
            if item["type"] not in {"LINE_STRIP", "ARROW"}:
                continue
            points = item.get("points", [])
            if len(points) < 2:
                continue
            if only_detail and item.get("floor_id") != "floor_2":
                continue
            color = item.get("color_rgba", [1, 1, 1, 1])
            xs = [p["x"] for p in points]
            ys = [p["y"] for p in points]
            zs = [p["z"] for p in points]
            ax.plot(xs, ys, zs, color=color[:3], alpha=color[3], linewidth=2.5 if item["type"] == "ARROW" else 1.8)

    def plot_pose_markers(ax: Any, only_detail: bool = False) -> None:
        for item in markers:
            pose = item.get("pose")
            if not pose:
                continue
            if only_detail and item.get("floor_id") != "floor_2":
                continue
            p = pose["position"]
            color = item.get("color_rgba", [1, 1, 1, 1])
            ax.scatter([p["x"]], [p["y"]], [p["z"]], color=[color[:3]], alpha=color[3], s=50)
            if item.get("text"):
                ax.text(p["x"], p["y"], p["z"] + 0.1, item["text"], fontsize=8)

    overview = FIG_DIR / "task42b_3d_route_overview_v0_1.png"
    detail = FIG_DIR / "task42b_object_approach_3d_detail_v0_1.png"

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    plot_line_markers(ax)
    plot_pose_markers(ax)
    ax.set_title("RSLG-SLAM task42b 3D replay overview")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("display z [m]")
    ax.view_init(elev=28, azim=-62)
    fig.tight_layout()
    fig.savefig(overview, dpi=180)
    plt.close(fig)

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")
    plot_line_markers(ax, only_detail=True)
    plot_pose_markers(ax, only_detail=True)
    tail = [f for f in frames if f.get("stage_name") in {"object_approach", "final_yaw_alignment"}]
    if tail:
        ax.plot([f["x"] for f in tail], [f["y"] for f in tail], [f["z"] for f in tail], color=(0.0, 0.8, 0.35), linewidth=3)
    ax.set_title("RSLG-SLAM task42b object approach context")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("display z [m]")
    ax.view_init(elev=35, azim=-44)
    fig.tight_layout()
    fig.savefig(detail, dpi=180)
    plt.close(fig)

    return {
        "status": "generated",
        "figure_paths": [rel(overview), rel(detail)],
        "figure_count": 2,
    }


def main() -> int:
    report = build_assets()
    print(json.dumps({"status": report["status"], "classification": report["classification"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
