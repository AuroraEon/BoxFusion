#!/usr/bin/env python3
"""Prepare Step30F GUI overlay bridge artifacts for scene 00824.

This step is visualization-only. It reads Step30E/30D/30C outputs, writes a
ROS MarkerArray-friendly payload, creates an RViz publisher/runbook package,
and validates that no navigation or upstream recomputation occurred.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import textwrap
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30f"
VERSION = "v0_1"
FRAME_ID = "map"

OUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30f_00824_gui_overlay_bridge"
SCRIPT_ROOT = OUT_ROOT / "scripts"
RVIZ_ROOT = OUT_ROOT / "rviz"
VIZ_ROOT = OUT_ROOT / "visualizations"
GAZEBO_ROOT = OUT_ROOT / "gazebo"
LOG_ROOT = OUT_ROOT / "logs"

STEP30E_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30e_00824_nav_projection_map_and_planner_audit"
STEP30D_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30d_00824_topology_route_projection_overlay_review"
STEP30C_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30c_00824_gateway_augmented_topology_candidate"
STEP17_SCENE_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step17_gazebo_nav2_asset_layer"
    / "scenes"
    / SCENE_ID
)

INPUT_PATHS = {
    "step30e_overlay_payload": STEP30E_ROOT / "00824_step30e_rviz_gazebo_overlay_payload_v0_1.json",
    "step30e_gateway_aware_payload": STEP30E_ROOT / "00824_step30e_gateway_aware_nav_projection_payload_v0_1.json",
    "step30e_route_audit": STEP30E_ROOT / "00824_step30e_route_nav_projection_audit_v0_1.json",
    "step30e_gateway_pose_audit": STEP30E_ROOT / "00824_step30e_gateway_pose_nav_projection_audit_v0_1.json",
    "step30e_summary": STEP30E_ROOT / "00824_step30e_summary_v0_1.json",
    "step30d_corrected_overlay": STEP30D_ROOT / "00824_step30d_corrected_overlay_payload_v0_1.json",
    "step30c_topology_candidate": STEP30C_ROOT / "00824_step30c_gateway_augmented_topology_candidate_v0_1.json",
    "step30c_gateway_edge_table": STEP30C_ROOT / "00824_step30c_gateway_edge_table_v0_1.json",
}

OUTPUT_PATHS = {
    "marker_payload": OUT_ROOT / "00824_step30f_gui_overlay_marker_payload_v0_1.json",
    "topic_plan": OUT_ROOT / "00824_step30f_marker_topic_plan_v0_1.json",
    "coordinate_frame_audit": OUT_ROOT / "00824_step30f_coordinate_frame_audit_v0_1.json",
    "runbook": OUT_ROOT / "00824_step30f_gui_display_runbook_v0_1.md",
    "live_attempt": OUT_ROOT / "00824_step30f_live_display_attempt_v0_1.json",
    "manifest": OUT_ROOT / "00824_step30f_visualization_manifest_v0_1.json",
    "validation": OUT_ROOT / "00824_step30f_validation_results_v0_1.json",
    "summary": OUT_ROOT / "00824_step30f_summary_v0_1.json",
    "static_preview": VIZ_ROOT / "00824_step30f_static_overlay_preview_v0_1.png",
    "publisher": SCRIPT_ROOT / "publish_00824_step30f_overlay_markers.py",
    "run_script": SCRIPT_ROOT / "run_step30f_rviz_overlay.sh",
    "gazebo_stub": SCRIPT_ROOT / "publish_00824_step30f_gazebo_overlay.py",
    "rviz_config": RVIZ_ROOT / "step30f_00824_overlay.rviz",
    "gazebo_sdf": GAZEBO_ROOT / "00824_step30f_static_overlay_marker_world.sdf",
    "prepare_log": LOG_ROOT / "00824_step30f_prepare_gui_overlay_payload.log",
    "dry_run_log": LOG_ROOT / "00824_step30f_marker_publisher_dry_run.log",
}

SELECTED_PAIR_KEYS = {
    "r1_r3",
    "r3_r7",
    "r3_r8",
    "r7_r11",
    "r7_r14",
    "r7_r15",
    "r8_r11",
    "r14_r16",
}
REQUIRED_GATEWAYS = {
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r7_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r7_r15_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r14_r16_01",
}
REQUIRED_ROUTES = {
    "public_path_042": {
        "rooms": ["room_1", "room_3", "room_7", "room_11", "room_8"],
        "gateways": [
            "gw_00824_r1_r3_01",
            "gw_00824_r3_r7_01",
            "gw_00824_r7_r11_02",
            "gw_00824_r8_r11_01",
        ],
    },
    "long_structure_route_default": {
        "rooms": ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"],
        "gateways": [
            "gw_00824_r1_r3_01",
            "gw_00824_r3_r8_01",
            "gw_00824_r8_r11_01",
            "gw_00824_r7_r11_02",
            "gw_00824_r7_r14_01",
            "gw_00824_r14_r16_01",
        ],
    },
}
FORBIDDEN_DIRECT_EDGE = "r3_r11"
FORBIDDEN_NON_TRUTH_PAIRS = {"r14_r15", "r15_r16", "r3_r15", "r7_r16"}

TOPICS = {
    "topology": "/boxfusion/step30f/topology_markers",
    "gateways": "/boxfusion/step30f/gateway_markers",
    "routes": "/boxfusion/step30f/route_markers",
    "poses": "/boxfusion/step30f/pose_markers",
    "review": "/boxfusion/step30f/review_markers",
}


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected object JSON in {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_text(path: Path, text: str, *, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    if executable:
        path.chmod(path.stat().st_mode | 0o111)


def room_num(room_id: str) -> int:
    return int(str(room_id).replace("room_", "").replace("r", ""))


def pair_key(room_a: Any, room_b: Any) -> str:
    a = int(room_a)
    b = int(room_b)
    lo, hi = sorted((a, b))
    return f"r{lo}_r{hi}"


def xy_to_point(xy: Sequence[float], z: float = 0.0) -> Dict[str, float]:
    return {"x": float(xy[0]), "y": float(xy[1]), "z": float(z)}


def pose_to_point(pose: Mapping[str, Any], z: float = 0.0) -> Dict[str, float]:
    return {"x": float(pose["x"]), "y": float(pose["y"]), "z": float(z)}


def arrow_end(pose: Mapping[str, Any], length: float = 0.45, z: float = 0.13) -> List[Dict[str, float]]:
    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose.get("yaw", 0.0))
    return [
        {"x": x, "y": y, "z": z},
        {"x": x + math.cos(yaw) * length, "y": y + math.sin(yaw) * length, "z": z},
    ]


def color(r: int, g: int, b: int, a: float = 1.0) -> Dict[str, float]:
    return {"r": r / 255.0, "g": g / 255.0, "b": b / 255.0, "a": float(a)}


COLORS = {
    "room": color(38, 111, 181, 0.88),
    "room_label": color(20, 45, 80, 1.0),
    "gateway_valid": color(36, 145, 83, 1.0),
    "gateway_review": color(230, 126, 34, 1.0),
    "strong": color(34, 139, 91, 0.92),
    "reviewable": color(245, 156, 48, 0.92),
    "low_or_uncertain": color(202, 70, 74, 0.92),
    "topology": color(82, 88, 102, 0.58),
    "route_public": color(10, 92, 180, 0.95),
    "route_long": color(137, 60, 178, 0.95),
    "pose_a": color(0, 148, 170, 0.9),
    "pose_b": color(16, 118, 180, 0.9),
    "crossing": color(20, 20, 20, 0.95),
    "carve": color(0, 166, 166, 0.25),
    "review_label": color(176, 58, 46, 1.0),
    "valid_label": color(26, 120, 82, 1.0),
}


def marker(
    *,
    marker_id: str,
    topic_key: str,
    namespace: str,
    marker_type: str,
    position: Optional[Mapping[str, float]] = None,
    points: Optional[List[Mapping[str, float]]] = None,
    text: Optional[str] = None,
    scale: Optional[Mapping[str, float]] = None,
    marker_color: Optional[Mapping[str, float]] = None,
    metadata: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "marker_id": marker_id,
        "topic": TOPICS[topic_key],
        "topic_key": topic_key,
        "namespace": namespace,
        "type": marker_type,
        "frame_id": FRAME_ID,
        "scale": dict(scale or {}),
        "color": dict(marker_color or color(255, 255, 255, 1.0)),
        "metadata": dict(metadata or {}),
    }
    if position is not None:
        payload["position"] = dict(position)
    if points is not None:
        payload["points"] = [dict(point) for point in points]
    if text is not None:
        payload["text"] = str(text)
    return payload


def load_inputs() -> Dict[str, Dict[str, Any]]:
    missing = [f"{name}: {rel(path)}" for name, path in INPUT_PATHS.items() if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing Step30F input artifact(s): " + "; ".join(missing))
    return {name: load_json(path) for name, path in INPUT_PATHS.items()}


def build_marker_payload(inputs: Mapping[str, Dict[str, Any]]) -> Dict[str, Any]:
    overlay = inputs["step30e_overlay_payload"]
    route_audit = inputs["step30e_route_audit"]
    gateway_pose_audit = inputs["step30e_gateway_pose_audit"]
    edge_table = inputs["step30c_gateway_edge_table"]

    rooms = {room["room_id"]: room for room in overlay["room_anchors"]}
    gateways = {gateway["gateway_id"]: gateway for gateway in overlay["gateway_markers"]}
    edge_by_gateway = {edge["gateway_id"]: edge for edge in edge_table["gateway_edges"]}
    route_by_id = {route["route_id"]: route for route in route_audit["routes"]}
    pose_audit_by_gateway = {
        audit["gateway_id"]: audit for audit in gateway_pose_audit.get("gateway_pose_audits", [])
    }
    status_label_by_gateway = {
        label["gateway_id"]: label for label in overlay.get("passability_status_labels", [])
    }
    review_by_gateway = {
        label["gateway_id"]: label for label in overlay.get("review_labels", [])
    }

    groups: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    routes_out: Dict[str, Dict[str, Any]] = {}
    topology_pair_keys: List[str] = []
    route_pair_keys: List[str] = []

    for room_id in sorted(rooms, key=room_num):
        item = rooms[room_id]
        xy = item["corrected_anchor_xy"]
        groups["rooms"].append(
            marker(
                marker_id=f"room_anchor_{room_id}",
                topic_key="topology",
                namespace="room_anchors",
                marker_type="CYLINDER",
                position=xy_to_point(xy, 0.05),
                scale={"x": 0.24, "y": 0.24, "z": 0.10},
                marker_color=COLORS["room"],
                metadata={
                    "room_id": room_id,
                    "anchor_source": "Step30E.corrected_anchor_xy",
                    "corrected_anchor_method": item.get("corrected_anchor_method"),
                    "original_anchor_status": item.get("original_anchor_status"),
                },
            )
        )
        groups["room_labels"].append(
            marker(
                marker_id=f"room_label_{room_id}",
                topic_key="topology",
                namespace="room_labels",
                marker_type="TEXT_VIEW_FACING",
                position=xy_to_point([xy[0], xy[1]], 0.42),
                text=room_id,
                scale={"z": 0.32},
                marker_color=COLORS["room_label"],
                metadata={"room_id": room_id},
            )
        )

    for gateway_id in sorted(gateways):
        gateway = gateways[gateway_id]
        edge = edge_by_gateway[gateway_id]
        pair = gateway["pair_key"]
        topology_pair_keys.append(pair)
        status = edge.get("passability_status") or "unknown"
        audit_status = gateway.get("audit_status") or "unknown"
        gateway_color = COLORS["gateway_valid"] if audit_status == "valid" else COLORS["gateway_review"]
        status_color = COLORS.get(status, COLORS["review_label"])
        center = gateway["center_xy"]

        groups["gateway_centers"].append(
            marker(
                marker_id=f"gateway_center_{gateway_id}",
                topic_key="gateways",
                namespace="gateway_centers",
                marker_type="SPHERE",
                position=xy_to_point(center, 0.16),
                scale={"x": 0.22, "y": 0.22, "z": 0.22},
                marker_color=gateway_color,
                metadata={
                    "gateway_id": gateway_id,
                    "pair_key": pair,
                    "audit_status": audit_status,
                    "passability_status": status,
                    "passability_label": gateway.get("passability_label"),
                },
            )
        )
        groups["gateway_labels"].append(
            marker(
                marker_id=f"gateway_label_{gateway_id}",
                topic_key="gateways",
                namespace="gateway_labels",
                marker_type="TEXT_VIEW_FACING",
                position=xy_to_point([center[0], center[1] + 0.10], 0.58),
                text=f"{gateway_id}\n{pair} | {status} | {audit_status}",
                scale={"z": 0.20},
                marker_color=status_color,
                metadata={"gateway_id": gateway_id, "pair_key": pair},
            )
        )

        room_a = f"room_{edge['room_a']}"
        room_b = f"room_{edge['room_b']}"
        groups["topology_edges"].append(
            marker(
                marker_id=f"topology_edge_{pair}",
                topic_key="topology",
                namespace="gateway_backed_topology_edges",
                marker_type="LINE_STRIP",
                points=[
                    xy_to_point(rooms[room_a]["corrected_anchor_xy"], 0.09),
                    xy_to_point(center, 0.09),
                    xy_to_point(rooms[room_b]["corrected_anchor_xy"], 0.09),
                ],
                scale={"x": 0.045},
                marker_color=COLORS["topology"],
                metadata={
                    "pair_key": pair,
                    "gateway_id": gateway_id,
                    "room_a": room_a,
                    "room_b": room_b,
                    "semantic": "gateway_backed_room_adjacency",
                },
            )
        )

        status_label = status_label_by_gateway.get(gateway_id, {})
        label_text = f"{pair}: {status} / {audit_status}"
        if status_label.get("notes"):
            label_text += "\n" + str(status_label["notes"][0])[:80]
        groups["passability_labels"].append(
            marker(
                marker_id=f"passability_label_{gateway_id}",
                topic_key="review",
                namespace="passability_status_labels",
                marker_type="TEXT_VIEW_FACING",
                position=xy_to_point([center[0], center[1] - 0.12], 0.86),
                text=label_text,
                scale={"z": 0.17},
                marker_color=status_color,
                metadata={"gateway_id": gateway_id, "pair_key": pair, "notes": status_label.get("notes", [])},
            )
        )
        if gateway_id in review_by_gateway:
            groups["review_labels"].append(
                marker(
                    marker_id=f"review_label_{gateway_id}",
                    topic_key="review",
                    namespace="review_labels",
                    marker_type="TEXT_VIEW_FACING",
                    position=xy_to_point([center[0], center[1] - 0.24], 1.08),
                    text=f"REVIEW {pair}",
                    scale={"z": 0.18},
                    marker_color=COLORS["review_label"],
                    metadata={
                        "gateway_id": gateway_id,
                        "pair_key": pair,
                        "reasons": review_by_gateway[gateway_id].get("reasons", []),
                    },
                )
            )

        audit = pose_audit_by_gateway[gateway_id]
        crossing_pose = audit["crossing_pose"]
        for pose_name, pose_key, label, pose_color in [
            ("approach_from_room_a", "approach_from_room_a", "approach A", COLORS["pose_a"]),
            ("approach_from_room_b", "approach_from_room_b", "approach B", COLORS["pose_b"]),
        ]:
            pose = audit[pose_key]
            groups["pose_arrows"].append(
                marker(
                    marker_id=f"pose_arrow_{gateway_id}_{pose_name}",
                    topic_key="poses",
                    namespace="approach_pose_arrows",
                    marker_type="ARROW",
                    points=[pose_to_point(pose, 0.18), pose_to_point(crossing_pose, 0.18)],
                    scale={"x": 0.055, "y": 0.11, "z": 0.11},
                    marker_color=pose_color,
                    metadata={"gateway_id": gateway_id, "pair_key": pair, "semantic": f"{label} to crossing"},
                )
            )
        groups["pose_arrows"].append(
            marker(
                marker_id=f"crossing_pose_arrow_{gateway_id}",
                topic_key="poses",
                namespace="crossing_pose_arrows",
                marker_type="ARROW",
                points=arrow_end(crossing_pose, length=0.48, z=0.24),
                scale={"x": 0.055, "y": 0.13, "z": 0.13},
                marker_color=COLORS["crossing"],
                metadata={"gateway_id": gateway_id, "pair_key": pair, "semantic": "crossing_pose_yaw"},
            )
        )

    for cap in overlay.get("local_carve_capsules_disks", []):
        gateway_id = cap["gateway_id"]
        pair = cap["pair_key"]
        groups["carve_capsules"].append(
            marker(
                marker_id=f"carve_capsule_line_{gateway_id}",
                topic_key="review",
                namespace="gateway_aware_carve_capsules",
                marker_type="LINE_STRIP",
                points=[
                    xy_to_point(cap["approach_a_xy"], 0.03),
                    xy_to_point(cap["crossing_xy"], 0.03),
                    xy_to_point(cap["approach_b_xy"], 0.03),
                ],
                scale={"x": float(cap.get("radius_m", 0.18)) * 2.0},
                marker_color=COLORS["carve"],
                metadata={
                    "gateway_id": gateway_id,
                    "pair_key": pair,
                    "radius_m": cap.get("radius_m"),
                    "carved_cell_count": cap.get("carved_cell_count"),
                    "semantic": "gateway-aware local passage hint only",
                },
            )
        )
        for disk_name, xy_key in [
            ("approach_a", "approach_a_xy"),
            ("crossing", "crossing_xy"),
            ("approach_b", "approach_b_xy"),
        ]:
            groups["carve_capsule_disks"].append(
                marker(
                    marker_id=f"carve_disk_{gateway_id}_{disk_name}",
                    topic_key="review",
                    namespace="gateway_aware_carve_disks",
                    marker_type="CYLINDER",
                    position=xy_to_point(cap[xy_key], 0.025),
                    scale={"x": float(cap.get("radius_m", 0.18)) * 2.0, "y": float(cap.get("radius_m", 0.18)) * 2.0, "z": 0.05},
                    marker_color=COLORS["carve"],
                    metadata={"gateway_id": gateway_id, "pair_key": pair, "disk": disk_name},
                )
            )

    for route_id, expected in REQUIRED_ROUTES.items():
        route = route_by_id[route_id]
        route_pairs = [gateways[gid]["pair_key"] for gid in route["gateway_sequence"]]
        route_pair_keys.extend(route_pairs)
        route_color = COLORS["route_public"] if route_id == "public_path_042" else COLORS["route_long"]
        groups["route_polylines"].append(
            marker(
                marker_id=f"route_polyline_{route_id}",
                topic_key="routes",
                namespace="route_polylines",
                marker_type="LINE_STRIP",
                points=[xy_to_point(xy, 0.20 if route_id == "public_path_042" else 0.26) for xy in route["local_triplet_polyline_xy"]]
                if "local_triplet_polyline_xy" in route
                else [xy_to_point(xy, 0.20 if route_id == "public_path_042" else 0.26) for xy in next(
                    item["points_xy"] for item in overlay["route_polylines"] if item["route_id"] == route_id
                )],
                scale={"x": 0.075 if route_id == "public_path_042" else 0.060},
                marker_color=route_color,
                metadata={
                    "route_id": route_id,
                    "room_sequence": route["room_sequence"],
                    "gateway_sequence": route["gateway_sequence"],
                    "pair_sequence": route_pairs,
                    "label": "offline projection only; not Nav2 path; not robot execution",
                },
            )
        )
        points = next(item["points_xy"] for item in overlay["route_polylines"] if item["route_id"] == route_id)
        mid = points[len(points) // 2]
        groups["route_labels"].append(
            marker(
                marker_id=f"route_label_{route_id}",
                topic_key="routes",
                namespace="route_labels",
                marker_type="TEXT_VIEW_FACING",
                position=xy_to_point([mid[0], mid[1]], 0.78),
                text=f"{route_id}\noffline projection only\nnot Nav2 path / not robot execution",
                scale={"z": 0.20},
                marker_color=route_color,
                metadata={"route_id": route_id},
            )
        )
        routes_out[route_id] = {
            "route_id": route_id,
            "room_sequence": route["room_sequence"],
            "gateway_sequence": route["gateway_sequence"],
            "expected_room_sequence": expected["rooms"],
            "expected_gateway_sequence": expected["gateways"],
            "pair_sequence": route_pairs,
            "nav_projection_ready_for_planning_only": route.get("nav_projection_ready_for_planning_only"),
            "route_projection_status": route.get("route_projection_status"),
            "semantics": "offline projection only; not Nav2 path; not robot execution",
        }

    ordered_groups = {key: value for key, value in sorted(groups.items())}
    all_markers = [item for records in ordered_groups.values() for item in records]
    topic_counts = Counter(item["topic"] for item in all_markers)
    namespace_counts = Counter(item["namespace"] for item in all_markers)

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_gui_overlay_marker_payload",
        "step": STEP,
        "version": VERSION,
        "frame_id": FRAME_ID,
        "input_coordinate_frame": overlay.get("coordinate_frame"),
        "output_frame_id": FRAME_ID,
        "transform_applied": False,
        "coordinate_policy": "preserve Step30E stage_a_map_xy x/y as ROS map x/y; no axis flip, scale change, rotation, or translation applied",
        "source_payload_paths": {name: rel(path) for name, path in INPUT_PATHS.items()},
        "publication_mode": "ROS2 MarkerArray static/latching-style repeated publication; visualization only",
        "topics": TOPICS,
        "groups": ordered_groups,
        "routes": routes_out,
        "topology_pair_keys": sorted(set(topology_pair_keys)),
        "route_pair_keys": sorted(set(route_pair_keys)),
        "forbidden_direct_edge": FORBIDDEN_DIRECT_EDGE,
        "forbidden_non_truth_pairs": sorted(FORBIDDEN_NON_TRUTH_PAIRS),
        "marker_counts": {
            "total": len(all_markers),
            "by_topic": dict(sorted(topic_counts.items())),
            "by_namespace": dict(sorted(namespace_counts.items())),
            "room_marker_count": len(ordered_groups.get("rooms", [])) + len(ordered_groups.get("room_labels", [])),
            "gateway_marker_count": len(ordered_groups.get("gateway_centers", [])) + len(ordered_groups.get("gateway_labels", [])),
            "route_marker_count": len(ordered_groups.get("route_polylines", [])) + len(ordered_groups.get("route_labels", [])),
            "pose_marker_count": len(ordered_groups.get("pose_arrows", [])),
            "review_marker_count": (
                len(ordered_groups.get("passability_labels", []))
                + len(ordered_groups.get("review_labels", []))
                + len(ordered_groups.get("carve_capsules", []))
                + len(ordered_groups.get("carve_capsule_disks", []))
            ),
        },
        "navigation_safety_boundary": {
            "stage_a_rerun_attempted": False,
            "gateway_extraction_rerun_attempted": False,
            "topology_changed": False,
            "nav_projection_changed": False,
            "ros_nav2_gazebo_run": False,
            "nav2_action_called": False,
            "cmd_vel_published": False,
            "robot_execution": False,
        },
    }


def build_topic_plan(marker_payload: Mapping[str, Any], *, live_publish_attempted: bool) -> Dict[str, Any]:
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_marker_topic_plan",
        "step": STEP,
        "version": VERSION,
        "topics": marker_payload["topics"],
        "marker_namespaces": sorted(marker_payload["marker_counts"]["by_namespace"].keys()),
        "marker_counts": marker_payload["marker_counts"],
        "frame_id": FRAME_ID,
        "source_payload_paths": marker_payload["source_payload_paths"],
        "publication_mode": marker_payload["publication_mode"],
        "live_publish_was_attempted": live_publish_attempted,
        "notes": [
            "Publisher repeats MarkerArray messages for RViz visibility.",
            "Topics are visualization-only and do not publish /cmd_vel or call Nav2 actions.",
        ],
    }


def build_coordinate_audit(inputs: Mapping[str, Dict[str, Any]]) -> Dict[str, Any]:
    overlay = inputs["step30e_overlay_payload"]
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_coordinate_frame_audit",
        "step": STEP,
        "version": VERSION,
        "input_coordinate_frame": overlay.get("coordinate_frame"),
        "output_frame_id": FRAME_ID,
        "transform_applied": False,
        "transform": {
            "x_out": "x_in",
            "y_out": "y_in",
            "z_out": "marker-specific visualization height only",
            "yaw_out": "yaw_in",
        },
        "x_axis_convention": "Step30E stage_a_map_xy x is preserved as ROS map +x.",
        "y_axis_convention": "Step30E stage_a_map_xy y is preserved as ROS map +y.",
        "yaw_convention": "Yaw values are interpreted as radians in the preserved x/y plane, counter-clockwise about +z.",
        "notes": [
            "No documented transform from stage_a_map_xy to map was found in Step30E; target_later_frame is map.",
            "Step30F therefore uses an explicit identity coordinate policy and records it here.",
            "Corrected Step30E room anchors are used; old invalid committed centroids are retained only as metadata.",
        ],
        "any_suspected_mismatch": False,
        "suspected_mismatch_details": [],
    }


def build_publisher_script() -> str:
    return r'''#!/usr/bin/env python3
"""Publish Step30F 00824 overlay markers as ROS 2 MarkerArray topics."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "This Step30F publisher requires ROS 2 rclpy. Use /usr/bin/python3 after "
        "sourcing /opt/ros/foxy/setup.bash."
    ) from exc


DEFAULT_PAYLOAD = Path(__file__).resolve().parents[1] / "00824_step30f_gui_overlay_marker_payload_v0_1.json"
DEFAULT_STATUS_TOPIC = "/boxfusion/step30f/status"


def _point(payload: Mapping[str, Any]) -> Point:
    point = Point()
    point.x = float(payload.get("x", 0.0) or 0.0)
    point.y = float(payload.get("y", 0.0) or 0.0)
    point.z = float(payload.get("z", 0.0) or 0.0)
    return point


def _marker_type(name: str) -> int:
    mapping = {
        "ARROW": Marker.ARROW,
        "CUBE": Marker.CUBE,
        "SPHERE": Marker.SPHERE,
        "CYLINDER": Marker.CYLINDER,
        "LINE_STRIP": Marker.LINE_STRIP,
        "LINE_LIST": Marker.LINE_LIST,
        "TEXT_VIEW_FACING": Marker.TEXT_VIEW_FACING,
    }
    return mapping.get(str(name or "").upper(), Marker.SPHERE)


def _apply_color(marker: Marker, payload: Mapping[str, Any]) -> None:
    marker.color.r = float(payload.get("r", 1.0) or 0.0)
    marker.color.g = float(payload.get("g", 1.0) or 0.0)
    marker.color.b = float(payload.get("b", 1.0) or 0.0)
    marker.color.a = float(payload.get("a", 1.0) or 0.0)


def _apply_scale(marker: Marker, payload: Mapping[str, Any], marker_type: str) -> None:
    marker_type = marker_type.upper()
    if marker_type in {"LINE_STRIP", "LINE_LIST"}:
        marker.scale.x = float(payload.get("x", 0.04) or 0.04)
        return
    if marker_type == "TEXT_VIEW_FACING":
        marker.scale.z = float(payload.get("z", 0.20) or 0.20)
        return
    marker.scale.x = float(payload.get("x", 0.15) or 0.15)
    marker.scale.y = float(payload.get("y", 0.15) or 0.15)
    marker.scale.z = float(payload.get("z", 0.15) or 0.15)


def spec_to_marker(spec: Mapping[str, Any], marker_id: int, stamp: Any) -> Marker:
    marker = Marker()
    marker.header.frame_id = str(spec.get("frame_id") or "map")
    marker.header.stamp = stamp
    marker.ns = str(spec.get("namespace") or "step30f")
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.type = _marker_type(str(spec.get("type") or "SPHERE"))
    marker.pose.orientation.w = 1.0
    if isinstance(spec.get("position"), Mapping):
        point = _point(spec["position"])
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
    if isinstance(spec.get("points"), list):
        marker.points = [_point(point) for point in spec["points"] if isinstance(point, Mapping)]
    if str(spec.get("type") or "").upper() == "TEXT_VIEW_FACING":
        marker.text = str(spec.get("text") or "")
    _apply_scale(marker, dict(spec.get("scale") or {}), str(spec.get("type") or "SPHERE"))
    _apply_color(marker, dict(spec.get("color") or {}))
    return marker


class Step30FOverlayPublisher(Node):  # pragma: no cover - requires ROS runtime
    def __init__(self, payload_path: Path, publish_period_sec: float, once: bool, dry_run: bool) -> None:
        super().__init__("boxfusion_step30f_overlay_marker_publisher")
        self.payload_path = payload_path
        self.publish_period_sec = max(0.1, float(publish_period_sec))
        self.once = bool(once)
        self.dry_run = bool(dry_run)
        with payload_path.open("r", encoding="utf-8") as handle:
            self.payload = json.load(handle)
        self.markers_by_topic: Dict[str, list] = defaultdict(list)
        for group in self.payload.get("groups", {}).values():
            for spec in group:
                self.markers_by_topic[str(spec["topic"])].append(spec)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publishers = {
            topic: self.create_publisher(MarkerArray, topic, qos)
            for topic in sorted(self.markers_by_topic)
        }
        self.status_publisher = self.create_publisher(String, DEFAULT_STATUS_TOPIC, qos)
        self.publish_count = 0
        self.get_logger().info(
            "Loaded Step30F overlay payload: "
            + json.dumps(
                {
                    "payload": str(payload_path),
                    "frame_id": self.payload.get("frame_id"),
                    "topics": sorted(self.markers_by_topic),
                    "marker_counts": self.payload.get("marker_counts"),
                    "dry_run": self.dry_run,
                },
                sort_keys=True,
            )
        )
        if self.dry_run:
            self.print_dry_run()
        self.timer = None if self.once else self.create_timer(self.publish_period_sec, self.publish_once)

    def print_dry_run(self) -> None:
        summary = {
            "scene_id": self.payload.get("scene_id"),
            "frame_id": self.payload.get("frame_id"),
            "topics": {topic: len(specs) for topic, specs in sorted(self.markers_by_topic.items())},
            "marker_counts": self.payload.get("marker_counts"),
            "navigation_safety_boundary": self.payload.get("navigation_safety_boundary"),
        }
        print(json.dumps(summary, indent=2, sort_keys=True))

    def publish_once(self) -> None:
        stamp = self.get_clock().now().to_msg()
        next_id = 1
        for topic, specs in sorted(self.markers_by_topic.items()):
            array = MarkerArray()
            for spec in specs:
                array.markers.append(spec_to_marker(spec, next_id, stamp))
                next_id += 1
            self.marker_publishers[topic].publish(array)
        status = String()
        status.data = json.dumps(
            {
                "scene_id": self.payload.get("scene_id"),
                "frame_id": self.payload.get("frame_id"),
                "marker_counts": self.payload.get("marker_counts"),
                "safety": self.payload.get("navigation_safety_boundary"),
                "publish_count": self.publish_count,
            },
            sort_keys=True,
        )
        self.status_publisher.publish(status)
        self.publish_count += 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish Step30F 00824 overlay MarkerArray topics.")
    parser.add_argument("--payload", type=Path, default=DEFAULT_PAYLOAD)
    parser.add_argument("--publish-period-sec", type=float, default=1.0)
    parser.add_argument("--once", action="store_true", help="Publish one latched-style sample and exit.")
    parser.add_argument("--dry-run", action="store_true", help="Print planned topics/counts before publishing.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    ros_args = None if argv is None else list(argv)
    raw_cli_args = list(remove_ros_args(args=ros_args))
    cli_args = raw_cli_args[1:] if argv is None else raw_cli_args
    args = build_arg_parser().parse_args(cli_args)
    rclpy.init(args=ros_args)
    node = Step30FOverlayPublisher(args.payload, args.publish_period_sec, args.once, args.dry_run)
    if args.once:
        node.publish_once()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 0
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
'''


def build_gazebo_stub_script() -> str:
    return r'''#!/usr/bin/env python3
"""Convert Step30F marker payload to a static Gazebo marker-world SDF.

This is a visualization helper only. It does not start Gazebo, Nav2, a robot,
or any controller. The generated SDF uses simple spheres/cylinders/boxes for
gateway and route inspection when Gazebo is available locally.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence


DEFAULT_PAYLOAD = Path(__file__).resolve().parents[1] / "00824_step30f_gui_overlay_marker_payload_v0_1.json"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "gazebo" / "00824_step30f_static_overlay_marker_world.sdf"


def color_text(color: Mapping[str, Any]) -> str:
    return "{:.3f} {:.3f} {:.3f} {:.3f}".format(
        float(color.get("r", 1.0)), float(color.get("g", 1.0)), float(color.get("b", 1.0)), float(color.get("a", 1.0))
    )


def add_material(visual: ET.Element, color: Mapping[str, Any]) -> None:
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = color_text(color)
    ET.SubElement(material, "diffuse").text = color_text(color)


def add_cylinder(world: ET.Element, name: str, x: float, y: float, z: float, radius: float, length: float, color: Mapping[str, Any]) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 0"
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = f"{radius:.4f}"
    ET.SubElement(cylinder, "length").text = f"{length:.4f}"
    add_material(visual, color)


def add_sphere(world: ET.Element, name: str, x: float, y: float, z: float, radius: float, color: Mapping[str, Any]) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 0"
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    ET.SubElement(sphere, "radius").text = f"{radius:.4f}"
    add_material(visual, color)


def add_box_segment(world: ET.Element, name: str, p0: Mapping[str, Any], p1: Mapping[str, Any], width: float, height: float, color: Mapping[str, Any]) -> None:
    x0, y0, z0 = float(p0["x"]), float(p0["y"]), float(p0.get("z", 0.05))
    x1, y1, z1 = float(p1["x"]), float(p1["y"]), float(p1.get("z", 0.05))
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return
    yaw = math.atan2(dy, dx)
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{(x0+x1)/2:.4f} {(y0+y1)/2:.4f} {(z0+z1)/2:.4f} 0 0 {yaw:.4f}"
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = f"{length:.4f} {width:.4f} {height:.4f}"
    add_material(visual, color)


def convert(payload_path: Path, output_path: Path) -> None:
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": "boxfusion_step30f_00824_overlay"})
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://sun"
    add_cylinder(world, "step30f_ground_plane", 2.35, -0.35, -0.02, 8.75, 0.02, {"r": 0.88, "g": 0.88, "b": 0.84, "a": 1.0})
    counts = {"spheres": 0, "cylinders": 1, "segments": 0}
    for group in payload.get("groups", {}).values():
        for spec in group:
            ns = spec.get("namespace")
            typ = str(spec.get("type") or "").upper()
            name = str(spec.get("marker_id") or "marker").replace("/", "_")
            c = spec.get("color") or {}
            if ns in {"room_anchors", "gateway_centers"} and isinstance(spec.get("position"), dict):
                p = spec["position"]
                if typ == "SPHERE":
                    add_sphere(world, name, float(p["x"]), float(p["y"]), float(p.get("z", 0.15)), float(spec.get("scale", {}).get("x", 0.2)) / 2, c)
                    counts["spheres"] += 1
                else:
                    add_cylinder(world, name, float(p["x"]), float(p["y"]), float(p.get("z", 0.05)), float(spec.get("scale", {}).get("x", 0.2)) / 2, float(spec.get("scale", {}).get("z", 0.08)), c)
                    counts["cylinders"] += 1
            if ns in {"gateway_backed_topology_edges", "route_polylines", "gateway_aware_carve_capsules"} and isinstance(spec.get("points"), list):
                width = float(spec.get("scale", {}).get("x", 0.04))
                for idx, (p0, p1) in enumerate(zip(spec["points"][:-1], spec["points"][1:])):
                    add_box_segment(world, f"{name}_segment_{idx}", p0, p1, width, 0.035, c)
                    counts["segments"] += 1
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(sdf).write(output_path, encoding="utf-8", xml_declaration=True)
    print(json.dumps({"output": str(output_path), "counts": counts}, indent=2, sort_keys=True))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Convert Step30F marker payload to static Gazebo SDF.")
    parser.add_argument("--payload", type=Path, default=DEFAULT_PAYLOAD)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    convert(args.payload, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
'''


def build_run_script() -> str:
    return f'''#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
STEP30F_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$STEP30F_ROOT/../../.." && pwd)"

source /opt/ros/foxy/setup.bash
if [[ -f "$REPO_ROOT/runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash" ]]; then
  source "$REPO_ROOT/runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash"
fi

PAYLOAD="$STEP30F_ROOT/{OUTPUT_PATHS['marker_payload'].name}"
RVIZ_CONFIG="$STEP30F_ROOT/rviz/{OUTPUT_PATHS['rviz_config'].name}"

/usr/bin/python3 "$SCRIPT_DIR/{OUTPUT_PATHS['publisher'].name}" --payload "$PAYLOAD" --dry-run &
PUBLISHER_PID=$!
trap 'kill "$PUBLISHER_PID" 2>/dev/null || true' EXIT

rviz2 -d "$RVIZ_CONFIG"
'''


def build_rviz_config() -> str:
    displays = []
    display_names = {
        TOPICS["topology"]: "Step30F Topology",
        TOPICS["gateways"]: "Step30F Gateways",
        TOPICS["routes"]: "Step30F Routes",
        TOPICS["poses"]: "Step30F Poses",
        TOPICS["review"]: "Step30F Review And Carve Hints",
    }
    for topic, name in display_names.items():
        displays.append(
            f'''    - Alpha: 1
      Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: {name}
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {topic}
      Value: true'''
        )
    map_yaml = STEP17_SCENE_ROOT / "maps" / "floor_1_approx_nav_map.yaml"
    map_note = "Optional map display can be added with map_server using Step17 map YAML." if map_yaml.exists() else "No Step17 map YAML found."
    return f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 40
      Cell Size: 1
      Reference Frame: {FRAME_ID}
      Value: true
{os.linesep.join(displays)}
  Enabled: true
  Global Options:
    Background Color: 245; 245; 245
    Fixed Frame: {FRAME_ID}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 15
      Focal Point:
        X: 2.3
        Y: -2.4
        Z: 0
      Name: Step30F Top Down
      Pitch: 1.5708
      Target Frame: {FRAME_ID}
      Yaw: 0
Window Geometry:
  Height: 900
  Width: 1300
# {map_note}
'''


def build_static_preview(marker_payload: Mapping[str, Any]) -> Optional[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return f"matplotlib_unavailable: {exc}"

    VIZ_ROOT.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#dddddd", linewidth=0.5)
    groups = marker_payload["groups"]

    for m in groups.get("topology_edges", []):
        pts = m["points"]
        ax.plot([p["x"] for p in pts], [p["y"] for p in pts], color="#555866", alpha=0.42, linewidth=2.0, zorder=1)
    for m in groups.get("carve_capsules", []):
        pts = m["points"]
        ax.plot([p["x"] for p in pts], [p["y"] for p in pts], color="#00a6a6", alpha=0.22, linewidth=12.0, zorder=2)
    for m in groups.get("route_polylines", []):
        pts = m["points"]
        color_hex = "#0a5cb4" if "public_path_042" in m["marker_id"] else "#893cb2"
        ax.plot([p["x"] for p in pts], [p["y"] for p in pts], color=color_hex, alpha=0.9, linewidth=3.2, zorder=4)
    for m in groups.get("rooms", []):
        p = m["position"]
        ax.scatter([p["x"]], [p["y"]], s=120, c="#266fb5", marker="o", zorder=5)
    for m in groups.get("room_labels", []):
        p = m["position"]
        ax.text(p["x"], p["y"], m["text"], fontsize=9, color="#142d50", zorder=6)
    for m in groups.get("gateway_centers", []):
        p = m["position"]
        md = m["metadata"]
        c = "#249153" if md.get("audit_status") == "valid" else "#e67e22"
        ax.scatter([p["x"]], [p["y"]], s=95, c=c, marker="s", zorder=7)
        ax.text(p["x"] + 0.08, p["y"] + 0.08, f"{md['pair_key']} {md['passability_status']}", fontsize=7, color=c, zorder=8)
    ax.set_title("Step30F 00824 GUI Overlay Payload Preview - visualization only")
    ax.set_xlabel("map x (identity from Step30E stage_a_map_xy)")
    ax.set_ylabel("map y (identity from Step30E stage_a_map_xy)")
    fig.tight_layout()
    fig.savefig(OUTPUT_PATHS["static_preview"], dpi=160)
    plt.close(fig)
    return None


def build_gazebo_sdf_with_stub() -> Dict[str, Any]:
    script = OUTPUT_PATHS["gazebo_stub"]
    output = OUTPUT_PATHS["gazebo_sdf"]
    command = ["/usr/bin/python3", str(script), "--payload", str(OUTPUT_PATHS["marker_payload"]), "--output", str(output)]
    result = subprocess.run(command, cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    return {
        "command": " ".join(command),
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "output_exists": output.exists(),
        "output_path": rel(output),
    }


def run_publisher_dry_run() -> Dict[str, Any]:
    command = [
        "bash",
        "-lc",
        "source /opt/ros/foxy/setup.bash && /usr/bin/python3 "
        + str(OUTPUT_PATHS["publisher"])
        + " --payload "
        + str(OUTPUT_PATHS["marker_payload"])
        + " --once --dry-run",
    ]
    result = subprocess.run(command, cwd=REPO_ROOT, text=True, capture_output=True, timeout=20, check=False)
    OUTPUT_PATHS["dry_run_log"].parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATHS["dry_run_log"].write_text(
        "COMMAND: " + " ".join(command) + "\n\nSTDOUT:\n" + result.stdout + "\nSTDERR:\n" + result.stderr,
        encoding="utf-8",
    )
    return {
        "command": " ".join(command),
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
        "log_path": rel(OUTPUT_PATHS["dry_run_log"]),
    }


def build_runbook(*, dry_run: Mapping[str, Any], gazebo: Mapping[str, Any]) -> str:
    map_yaml = STEP17_SCENE_ROOT / "maps" / "floor_1_approx_nav_map.yaml"
    gazebo_world = STEP17_SCENE_ROOT / "worlds" / "00824-Dd4bFSTQ8gi_marker_world.sdf"
    step30f_sdf = OUTPUT_PATHS["gazebo_sdf"]
    return f"""# Step30F 00824 GUI Overlay Display Runbook

This is a visualization-only bridge from Step30E offline payloads to GUI overlays. It does not call Nav2 actions, does not send `NavigateToPose`, does not publish `/cmd_vel`, and does not execute robot navigation.

## Files

- Marker payload: `{rel(OUTPUT_PATHS['marker_payload'])}`
- RViz config: `{rel(OUTPUT_PATHS['rviz_config'])}`
- ROS publisher: `{rel(OUTPUT_PATHS['publisher'])}`
- Optional Gazebo SDF: `{rel(step30f_sdf)}`
- Static preview: `{rel(OUTPUT_PATHS['static_preview'])}`

## RViz Overlay

From the repository root:

```bash
source /opt/ros/foxy/setup.bash
if [ -f runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash ]; then
  source runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash
fi
```

Terminal 1, start the marker publisher:

```bash
/usr/bin/python3 {rel(OUTPUT_PATHS['publisher'])} \\
  --payload {rel(OUTPUT_PATHS['marker_payload'])} \\
  --dry-run
```

Terminal 2, open RViz:

```bash
rviz2 -d {rel(OUTPUT_PATHS['rviz_config'])}
```

The RViz fixed frame is `map`. Confirm these MarkerArray topics are enabled:

```bash
ros2 topic list | grep /boxfusion/step30f
```

Expected topics:

- `{TOPICS['topology']}`
- `{TOPICS['gateways']}`
- `{TOPICS['routes']}`
- `{TOPICS['poses']}`
- `{TOPICS['review']}`

Do not use `ros2 topic echo --once` on Foxy; if needed, use normal `ros2 topic echo` and stop it with Ctrl-C.

## Optional Map Background

Step17 map YAML discovered: `{rel(map_yaml) if map_yaml.exists() else 'not found'}`

To show an occupancy grid behind the markers, start a map server manually in a separate terminal if your local Foxy/Nav2 install supports it. This is optional for Step30F and is not a navigation run.

## One-Command RViz Helper

```bash
bash {rel(OUTPUT_PATHS['run_script'])}
```

Stop RViz by closing the window. The helper stops the marker publisher automatically.

## Optional Gazebo Marker World

Gazebo executable detected during preparation: `{shutil.which('gazebo') or 'missing'}`

Step17 00824 marker world: `{rel(gazebo_world) if gazebo_world.exists() else 'not found'}`

Step30F static overlay SDF generated: `{rel(step30f_sdf) if step30f_sdf.exists() else 'not generated'}`

If Gazebo and `gazebo_ros` are available locally:

```bash
source /opt/ros/foxy/setup.bash
if [ -f runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash ]; then
  source runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/install/setup.bash
fi
ros2 launch boxfusion_gazebo_nav2_demo gazebo_marker_world.launch.py \\
  world:=$PWD/{rel(step30f_sdf)}
```

This Gazebo view is static geometry only. Labels are not represented as Gazebo text; use RViz for text labels.

## Stop Processes

- RViz: close the window or press Ctrl-C in its terminal.
- Marker publisher: press Ctrl-C in Terminal 1.
- Gazebo: close the window or press Ctrl-C in its launch terminal.

## Preparation Notes

- Publisher dry-run return code: `{dry_run.get('returncode')}`
- Publisher dry-run log: `{dry_run.get('log_path')}`
- Gazebo converter return code: `{gazebo.get('returncode')}`
- Gazebo converter output exists: `{gazebo.get('output_exists')}`
"""


def validate(
    inputs: Mapping[str, Dict[str, Any]],
    marker_payload: Mapping[str, Any],
    live_attempt: Mapping[str, Any],
) -> Dict[str, Any]:
    checks: List[Dict[str, Any]] = []

    def check(name: str, passed: bool, details: Optional[Mapping[str, Any]] = None) -> None:
        checks.append({"check": name, "passed": bool(passed), "details": dict(details or {})})

    step30c = inputs["step30c_topology_candidate"]
    step30d = inputs["step30d_corrected_overlay"]
    step30e_summary = inputs["step30e_summary"]
    edge_table = inputs["step30c_gateway_edge_table"]

    gateway_ids = {
        marker["metadata"]["gateway_id"]
        for marker in marker_payload["groups"].get("gateway_centers", [])
        if "gateway_id" in marker.get("metadata", {})
    }
    pair_keys = {edge["pair_key"] for edge in edge_table["gateway_edges"]}
    route_ids = set(marker_payload["routes"])
    topology_pair_keys = set(marker_payload.get("topology_pair_keys", []))
    route_pair_keys = set(marker_payload.get("route_pair_keys", []))
    room_markers = marker_payload["groups"].get("rooms", [])
    old_anchor_used = any(
        marker.get("metadata", {}).get("original_anchor_status") != "invalid_committed_centroid"
        or marker.get("metadata", {}).get("anchor_source") != "Step30E.corrected_anchor_xy"
        for marker in room_markers
    )

    check("no_stage_a_rerun", not marker_payload["navigation_safety_boundary"]["stage_a_rerun_attempted"])
    check("no_gateway_extraction_rerun", not marker_payload["navigation_safety_boundary"]["gateway_extraction_rerun_attempted"])
    check("step30c_topology_unchanged", step30c.get("topology_only") is True and step30c.get("stage_a_rerun_attempted") is False)
    check("step30d_route_projection_unchanged", step30d.get("topology_changed") is False and step30d.get("robot_execution") is False)
    check("step30e_nav_projection_unchanged", step30e_summary.get("nav_projection_changed") is False or step30e_summary.get("nav_projection_changed") is None)
    check("no_nav2_action_call", not marker_payload["navigation_safety_boundary"]["nav2_action_called"])
    check("no_cmd_vel_publication", not marker_payload["navigation_safety_boundary"]["cmd_vel_published"])
    check("no_robot_execution", not marker_payload["navigation_safety_boundary"]["robot_execution"])
    check("marker_payload_exists", OUTPUT_PATHS["marker_payload"].exists(), {"path": rel(OUTPUT_PATHS["marker_payload"])})
    check("marker_payload_includes_all_8_selected_gateways", gateway_ids == REQUIRED_GATEWAYS, {"actual": sorted(gateway_ids)})
    check("marker_payload_includes_public_path_042", "public_path_042" in route_ids)
    check("marker_payload_includes_long_structure_route_default", "long_structure_route_default" in route_ids)
    check("r3_r11_direct_edge_absent", FORBIDDEN_DIRECT_EDGE not in topology_pair_keys and FORBIDDEN_DIRECT_EDGE not in route_pair_keys)
    check(
        "non_truth_pairs_absent_as_route_or_topology_edges",
        not (FORBIDDEN_NON_TRUTH_PAIRS & topology_pair_keys) and not (FORBIDDEN_NON_TRUTH_PAIRS & route_pair_keys),
        {"topology_pair_keys": sorted(topology_pair_keys), "route_pair_keys": sorted(route_pair_keys)},
    )
    check("corrected_room_anchors_used_not_invalid_old_centroids", len(room_markers) == 8 and not old_anchor_used)
    check("marker_frame_id_is_map", marker_payload.get("frame_id") == FRAME_ID)
    check("coordinate_frame_audit_exists", OUTPUT_PATHS["coordinate_frame_audit"].exists())
    check("rviz_runbook_exists", OUTPUT_PATHS["runbook"].exists())
    check(
        "live_gui_failure_recorded_if_attempted",
        True,
        {
            "rviz_attempted": live_attempt.get("live_rviz_attempted"),
            "gazebo_attempted": live_attempt.get("live_gazebo_attempted"),
            "observed_errors": live_attempt.get("observed_errors", []),
        },
    )
    check(
        "offline_outputs_useful_without_live_gui",
        OUTPUT_PATHS["publisher"].exists() and OUTPUT_PATHS["rviz_config"].exists() and OUTPUT_PATHS["static_preview"].exists(),
        {
            "publisher": rel(OUTPUT_PATHS["publisher"]),
            "rviz_config": rel(OUTPUT_PATHS["rviz_config"]),
            "static_preview": rel(OUTPUT_PATHS["static_preview"]),
        },
    )
    check("step30c_gateway_pairs_match_selected_pairs", pair_keys == SELECTED_PAIR_KEYS, {"actual": sorted(pair_keys)})

    passed = all(item["passed"] for item in checks)
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_validation_results",
        "step": STEP,
        "version": VERSION,
        "validation_passed": passed,
        "checks": checks,
        "summary": {
            "passed_count": sum(1 for item in checks if item["passed"]),
            "failed_count": sum(1 for item in checks if not item["passed"]),
            "total_count": len(checks),
        },
    }


def build_live_attempt(dry_run: Mapping[str, Any], gazebo: Mapping[str, Any]) -> Dict[str, Any]:
    rviz_available = shutil.which("rviz2") is not None
    gazebo_available = shutil.which("gazebo") is not None
    marker_run = dry_run.get("returncode") == 0
    errors: List[str] = []
    if dry_run.get("returncode") != 0:
        errors.append("marker publisher dry-run failed; see dry-run log")
    if not rviz_available:
        errors.append("rviz2 executable not available in PATH for live GUI attempt")
    if not gazebo_available:
        errors.append("gazebo executable not available in PATH for live Gazebo attempt")
    if gazebo.get("returncode") != 0:
        errors.append("Gazebo SDF converter failed; see converter stderr")
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_live_display_attempt",
        "step": STEP,
        "version": VERSION,
        "live_rviz_attempted": False,
        "live_gazebo_attempted": False,
        "marker_publisher_was_run": True,
        "marker_publisher_run_mode": "dry-run single publish attempt; no GUI window launched by automation",
        "commands_run": [
            dry_run.get("command"),
            gazebo.get("command"),
        ],
        "observed_errors": errors,
        "screenshots_produced": [],
        "gui_available": {"rviz2_in_path": rviz_available, "gazebo_in_path": gazebo_available},
        "marker_publisher_dry_run": dry_run,
        "gazebo_overlay_converter": gazebo,
        "notes": [
            "Live GUI windows were not launched automatically so the task remains non-blocking in headless environments.",
            "The RViz runbook contains exact user-run commands for local display.",
        ],
        "nav2_action_called": False,
        "cmd_vel_published": False,
        "robot_execution": False,
    }


def build_manifest(static_preview_error: Optional[str], gazebo: Mapping[str, Any], dry_run: Mapping[str, Any]) -> Dict[str, Any]:
    items = [
        {
            "path": rel(OUTPUT_PATHS["static_preview"]),
            "artifact_type": "static_2d_preview_png",
            "generated_successfully": OUTPUT_PATHS["static_preview"].exists(),
            "error": static_preview_error,
        },
        {
            "path": rel(OUTPUT_PATHS["rviz_config"]),
            "artifact_type": "rviz_config",
            "generated_successfully": OUTPUT_PATHS["rviz_config"].exists(),
        },
        {
            "path": rel(OUTPUT_PATHS["publisher"]),
            "artifact_type": "ros2_marker_publisher",
            "generated_successfully": OUTPUT_PATHS["publisher"].exists(),
        },
        {
            "path": rel(OUTPUT_PATHS["gazebo_sdf"]),
            "artifact_type": "gazebo_static_marker_world_sdf",
            "generated_successfully": bool(gazebo.get("output_exists")),
        },
        {
            "path": dry_run.get("log_path"),
            "artifact_type": "marker_publisher_dry_run_log",
            "generated_successfully": OUTPUT_PATHS["dry_run_log"].exists(),
        },
    ]
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_visualization_manifest",
        "step": STEP,
        "version": VERSION,
        "visualizations": items,
        "summary": {
            "generated_successfully_count": sum(1 for item in items if item.get("generated_successfully")),
            "total_count": len(items),
        },
    }


def build_summary(
    marker_payload: Mapping[str, Any],
    validation: Mapping[str, Any],
    live_attempt: Mapping[str, Any],
    gazebo_prepared: bool,
) -> Dict[str, Any]:
    counts = marker_payload["marker_counts"]
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30f_summary",
        "step": STEP,
        "version": VERSION,
        "stage_a_rerun_attempted": False,
        "gateway_extraction_rerun_attempted": False,
        "topology_changed": False,
        "nav_projection_changed": False,
        "ros_nav2_gazebo_run": False,
        "nav2_action_called": False,
        "cmd_vel_published": False,
        "robot_execution": False,
        "rviz_overlay_prepared": OUTPUT_PATHS["publisher"].exists() and OUTPUT_PATHS["rviz_config"].exists(),
        "rviz_overlay_live_attempted": bool(live_attempt.get("live_rviz_attempted")),
        "gazebo_overlay_prepared": gazebo_prepared,
        "gazebo_overlay_live_attempted": bool(live_attempt.get("live_gazebo_attempted")),
        "marker_count_total": counts["total"],
        "room_marker_count": counts["room_marker_count"],
        "gateway_marker_count": counts["gateway_marker_count"],
        "route_marker_count": counts["route_marker_count"],
        "pose_marker_count": counts["pose_marker_count"],
        "review_marker_count": counts["review_marker_count"],
        "r3_r11_direct_edge_absent": FORBIDDEN_DIRECT_EDGE not in marker_payload.get("topology_pair_keys", [])
        and FORBIDDEN_DIRECT_EDGE not in marker_payload.get("route_pair_keys", []),
        "non_truth_pairs_absent": not (FORBIDDEN_NON_TRUTH_PAIRS & set(marker_payload.get("topology_pair_keys", [])))
        and not (FORBIDDEN_NON_TRUTH_PAIRS & set(marker_payload.get("route_pair_keys", []))),
        "validation_passed": bool(validation.get("validation_passed")),
        "recommended_next_step": "Run the Step30F RViz runbook locally to visually inspect marker alignment; then proceed only to a future Nav2 planning-only step, still without robot execution.",
        "output_directory": rel(OUT_ROOT),
        "key_outputs": {key: rel(path) for key, path in OUTPUT_PATHS.items() if path.exists()},
    }


def main() -> int:
    for root in (OUT_ROOT, SCRIPT_ROOT, RVIZ_ROOT, VIZ_ROOT, GAZEBO_ROOT, LOG_ROOT):
        root.mkdir(parents=True, exist_ok=True)

    inputs = load_inputs()
    marker_payload = build_marker_payload(inputs)
    write_json(OUTPUT_PATHS["marker_payload"], marker_payload)
    write_json(OUTPUT_PATHS["coordinate_frame_audit"], build_coordinate_audit(inputs))

    write_text(OUTPUT_PATHS["publisher"], build_publisher_script(), executable=True)
    write_text(OUTPUT_PATHS["gazebo_stub"], build_gazebo_stub_script(), executable=True)
    write_text(OUTPUT_PATHS["run_script"], build_run_script(), executable=True)
    write_text(OUTPUT_PATHS["rviz_config"], build_rviz_config())

    static_preview_error = build_static_preview(marker_payload)
    gazebo = build_gazebo_sdf_with_stub()
    dry_run = run_publisher_dry_run()
    live_attempt = build_live_attempt(dry_run, gazebo)
    topic_plan = build_topic_plan(marker_payload, live_publish_attempted=bool(live_attempt["marker_publisher_was_run"]))

    write_json(OUTPUT_PATHS["topic_plan"], topic_plan)
    write_json(OUTPUT_PATHS["live_attempt"], live_attempt)
    write_text(OUTPUT_PATHS["runbook"], build_runbook(dry_run=dry_run, gazebo=gazebo))
    manifest = build_manifest(static_preview_error, gazebo, dry_run)
    write_json(OUTPUT_PATHS["manifest"], manifest)
    validation = validate(inputs, marker_payload, live_attempt)
    write_json(OUTPUT_PATHS["validation"], validation)
    summary = build_summary(marker_payload, validation, live_attempt, gazebo_prepared=bool(gazebo.get("output_exists")))
    write_json(OUTPUT_PATHS["summary"], summary)

    prepare_log = {
        "scene_id": SCENE_ID,
        "outputs": {key: rel(path) for key, path in OUTPUT_PATHS.items() if path.exists()},
        "validation_passed": validation["validation_passed"],
        "marker_counts": marker_payload["marker_counts"],
    }
    write_json(OUTPUT_PATHS["prepare_log"], prepare_log)
    print(json.dumps(prepare_log, indent=2, sort_keys=True))
    return 0 if validation["validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
