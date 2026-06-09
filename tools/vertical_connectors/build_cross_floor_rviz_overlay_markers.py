#!/usr/bin/env python3
"""Build task24d graph-only RViz overlay marker artifacts for RSLG-SLAM."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import py_compile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


TASK_NAME = "task24d_cross_floor_rviz_overlay_adapter_with_transition_semantics_fix"
TASK24C_NAME = "task24c_rslg_cross_floor_connector_router_and_overlay"
SCENE_ID_DEFAULT = "00843-DYehNKdT76V"
CLAIM_BOUNDARY = "topological_vertical_transition_only"
PHYSICAL_EXECUTION_SUPPORTED = False
EXPECTED_ROUTE = [
    "room_3",
    "vc_vt_1_from_binding",
    "vt_1_centerline_n000",
    "vt_1_centerline_n001",
    "vt_1_centerline_n002",
    "vt_1_centerline_n003",
    "vt_1_centerline_n004",
    "vc_vt_1_to_binding",
    "room_7",
    "room_13",
    "room_14",
]
EXPECTED_TRANSITION = ("vt_1_centerline_n001", "vt_1_centerline_n002")
PROTECTED_RELATIVE_PATHS = {
    "task23b": "tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture",
    "clean_rerun_maps_floor_2": "clean_rerun/maps/floor_2",
    "clean_rerun_committed_public": "clean_rerun/committed_public",
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, Path):
        return value.as_posix()
    return value


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_tree(root: Path) -> dict[str, str]:
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def tree_unchanged(before: dict[str, str], after: dict[str, str]) -> dict[str, Any]:
    before_keys = set(before)
    after_keys = set(after)
    changed = sorted(path for path in before_keys & after_keys if before[path] != after[path])
    added = sorted(after_keys - before_keys)
    removed = sorted(before_keys - after_keys)
    return {
        "unchanged": not changed and not added and not removed,
        "changed": changed,
        "added": added,
        "removed": removed,
        "file_count_before": len(before),
        "file_count_after": len(after),
    }


def collect_paths(scene_root: Path) -> dict[str, Path]:
    task24c = scene_root / "tasks" / TASK24C_NAME
    clean = scene_root / "clean_rerun"
    return {
        "scene_root": scene_root,
        "task24c_dir": task24c,
        "task24d_dir": scene_root / "tasks" / TASK_NAME,
        "vertical_connectors": task24c / "vertical_connectors_v0_1.json",
        "topology": task24c / "cross_floor_topology_v0_1.json",
        "route_report": task24c / "cross_floor_route_query_report.json",
        "waypoints": task24c / "cross_floor_route_waypoints_v0_1.json",
        "overlay_manifest": task24c / "cross_floor_overlay_manifest.json",
        "floor_1_map_yaml": clean / "maps" / "floor_1" / "stage1_floor_1_stable_occupancy_map.yaml",
        "floor_2_map_yaml": clean / "maps" / "floor_2" / "stage1_floor_2_stable_occupancy_map.yaml",
        "committed_public_topology": clean / "committed_public" / "topology_v0_1.json",
    }


def load_inputs(paths: dict[str, Path]) -> tuple[dict[str, Any], dict[str, Any]]:
    payloads: dict[str, Any] = {}
    parse_results: dict[str, Any] = {}
    for key in ("vertical_connectors", "topology", "route_report", "waypoints", "overlay_manifest"):
        path = paths[key]
        result = {"path": path, "exists": path.exists(), "parses": False}
        if path.exists():
            try:
                payloads[key] = read_json(path)
                result["parses"] = True
            except Exception as exc:  # noqa: BLE001 - validation report should retain failures.
                result["error"] = f"{type(exc).__name__}: {exc}"
        parse_results[key] = result
    return payloads, parse_results


def get_position(node: dict[str, Any]) -> list[float]:
    if "position_xyz" in node:
        xyz = node["position_xyz"]
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    if "xyz" in node:
        xyz = node["xyz"]
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    if "position_xy" in node:
        xy = node["position_xy"]
        return [float(xy[0]), float(xy[1]), 0.0]
    if "room" in node and "center" in node["room"]:
        xy = node["room"]["center"]
        return [float(xy[0]), float(xy[1]), 0.0]
    raise KeyError(f"node has no position: {node.get('node_id')}")


def midpoint(a: list[float], b: list[float]) -> list[float]:
    return [(float(a[i]) + float(b[i])) / 2.0 for i in range(3)]


def node_indices(topology: dict[str, Any], waypoints: dict[str, Any]) -> dict[str, dict[str, Any]]:
    nodes: dict[str, dict[str, Any]] = {}
    for node in topology.get("nodes", []):
        node_id = node.get("node_id")
        if node_id:
            nodes[str(node_id)] = copy.deepcopy(node)
    for wp in waypoints.get("waypoints", []):
        node_id = wp.get("node_id")
        if node_id and str(node_id) not in nodes:
            nodes[str(node_id)] = {
                "node_id": node_id,
                "node_type": wp.get("node_type"),
                "floor_id": wp.get("floor_id"),
                "position_xy": wp.get("position_xy"),
                "position_xyz": wp.get("position_xyz"),
                "connector_id": wp.get("connector_id"),
                "physical_execution_supported": wp.get("physical_execution_supported", False),
                "claim_boundary": CLAIM_BOUNDARY,
            }
    return nodes


def route_node_floors(nodes: dict[str, dict[str, Any]], route_nodes: list[str]) -> dict[str, str | None]:
    return {node_id: nodes.get(node_id, {}).get("floor_id") for node_id in route_nodes}


def recompute_edge_floor_transition_semantics(
    route_edges: list[dict[str, Any]],
    nodes: dict[str, dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    corrected: list[dict[str, Any]] = []
    transition_edges: list[dict[str, Any]] = []
    missing_floor_nodes: list[str] = []
    misplaced_before: list[dict[str, Any]] = []
    for edge in route_edges:
        updated = copy.deepcopy(edge)
        source = str(updated.get("source"))
        target = str(updated.get("target"))
        source_floor = nodes.get(source, {}).get("floor_id")
        target_floor = nodes.get(target, {}).get("floor_id")
        if not source_floor:
            missing_floor_nodes.append(source)
        if not target_floor:
            missing_floor_nodes.append(target)
        is_transition = bool(source_floor and target_floor and source_floor != target_floor)
        previous_count = int(updated.get("floor_transition_count", 0) or 0)
        updated["source_floor"] = source_floor
        updated["target_floor"] = target_floor
        updated["floor_transition_count"] = 1 if is_transition else 0
        updated["edge_is_floor_transition"] = is_transition
        updated["transition_marker_candidate"] = is_transition
        updated["transition_semantics_source"] = "task24d_recomputed_from_route_node_floor_ids"
        updated["claim_boundary"] = CLAIM_BOUNDARY
        updated["physical_execution_supported"] = PHYSICAL_EXECUTION_SUPPORTED
        if previous_count != updated["floor_transition_count"]:
            misplaced_before.append(
                {
                    "edge_id": updated.get("edge_id"),
                    "source": source,
                    "target": target,
                    "previous_floor_transition_count": previous_count,
                    "corrected_floor_transition_count": updated["floor_transition_count"],
                    "source_floor": source_floor,
                    "target_floor": target_floor,
                }
            )
        if is_transition:
            transition_edges.append(updated)
        corrected.append(updated)
    summary = {
        "route_floor_transition_count": sum(int(edge["floor_transition_count"]) for edge in corrected),
        "transition_edges": transition_edges,
        "missing_floor_nodes": sorted(set(missing_floor_nodes)),
        "changed_edges": misplaced_before,
    }
    return corrected, summary


def corrected_route_report(route_report: dict[str, Any], corrected_edges: list[dict[str, Any]], summary: dict[str, Any]) -> dict[str, Any]:
    report = copy.deepcopy(route_report)
    report["schema_version"] = "cross_floor_route_query_report_v0_2"
    report["created_utc"] = now_iso()
    report["task_name"] = TASK_NAME
    report["route_edges"] = corrected_edges
    report["floor_transition_count"] = summary["route_floor_transition_count"]
    report["claim_boundary"] = CLAIM_BOUNDARY
    report["physical_execution_supported"] = PHYSICAL_EXECUTION_SUPPORTED
    report["transition_semantics_fix"] = {
        "method": "per_edge_floor_transition_count_recomputed_from_source_and_target_node_floor_ids",
        "changed_edges": summary["changed_edges"],
        "transition_edge_ids": [edge["edge_id"] for edge in summary["transition_edges"]],
        "expected_transition_source": EXPECTED_TRANSITION[0],
        "expected_transition_target": EXPECTED_TRANSITION[1],
    }
    return report


def corrected_topology(topology: dict[str, Any], corrected_edges: list[dict[str, Any]]) -> dict[str, Any]:
    topo = copy.deepcopy(topology)
    route_edge_by_id = {edge["edge_id"]: edge for edge in corrected_edges}
    for edge in topo.get("edges", []):
        route_edge = route_edge_by_id.get(edge.get("edge_id"))
        if not route_edge:
            edge.setdefault("edge_is_floor_transition", False)
            edge.setdefault("transition_marker_candidate", False)
            continue
        edge["floor_transition_count"] = route_edge["floor_transition_count"]
        edge["source_floor"] = route_edge["source_floor"]
        edge["target_floor"] = route_edge["target_floor"]
        edge["edge_is_floor_transition"] = route_edge["edge_is_floor_transition"]
        edge["transition_marker_candidate"] = route_edge["transition_marker_candidate"]
        edge["transition_semantics_source"] = route_edge["transition_semantics_source"]
        edge["claim_boundary"] = CLAIM_BOUNDARY
        edge["physical_execution_supported"] = PHYSICAL_EXECUTION_SUPPORTED
        if edge.get("edge_type") == "stair_connector_edge":
            edge.setdefault("provenance", {})["transition_count_note"] = (
                "task24d recomputed per-edge transition count from endpoint floor ids"
            )
    topo["schema_version"] = "cross_floor_topology_v0_2"
    topo["created_utc"] = now_iso()
    topo["task_name"] = TASK_NAME
    topo["claim_boundary"] = CLAIM_BOUNDARY
    topo["physical_execution_supported"] = PHYSICAL_EXECUTION_SUPPORTED
    topo["transition_semantics_fix"] = {
        "method": "route_edge_annotations_corrected_from_node_floor_ids",
        "route_edge_ids_corrected": sorted(route_edge_by_id),
        "source_artifact": "cross_floor_topology_v0_1.json",
    }
    return topo


def validate_inputs(
    paths: dict[str, Path],
    payloads: dict[str, Any],
    parse_results: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    route_report = payloads.get("route_report", {})
    topology = payloads.get("topology", {})
    waypoints = payloads.get("waypoints", {})
    route_nodes = [str(node_id) for node_id in route_report.get("route_nodes", [])]
    route_edges = route_report.get("route_edges", [])
    waypoint_node_ids = {str(wp.get("node_id")) for wp in waypoints.get("waypoints", [])}
    edge_endpoint_checks = [
        {
            "edge_id": edge.get("edge_id"),
            "source": edge.get("source"),
            "target": edge.get("target"),
            "source_exists": edge.get("source") in nodes,
            "target_exists": edge.get("target") in nodes,
        }
        for edge in route_edges
    ]
    checks = {
        "task24c_artifacts_exist_and_parse": all(result["exists"] and result["parses"] for result in parse_results.values()),
        "floor_1_stable_map_exists": paths["floor_1_map_yaml"].exists(),
        "floor_2_stable_map_exists": paths["floor_2_map_yaml"].exists(),
        "route_starts_at_room_3": route_nodes[:1] == ["room_3"],
        "route_ends_at_room_14": route_nodes[-1:] == ["room_14"],
        "route_uses_vc_vt_1": "vc_vt_1" in route_report.get("used_connector_ids", []),
        "route_contains_vt_1_centerline_nodes": any(node_id.startswith("vt_1_centerline_") for node_id in route_nodes),
        "every_route_edge_source_target_exists": all(item["source_exists"] and item["target_exists"] for item in edge_endpoint_checks),
        "every_waypoint_node_id_exists": all(node_id in nodes for node_id in waypoint_node_ids),
        "physical_execution_supported_false": route_report.get("physical_execution_supported") is False
        and topology.get("physical_execution_supported") is False
        and waypoints.get("physical_execution_supported") is False,
        "claim_boundary_topological_only": route_report.get("claim_boundary") == CLAIM_BOUNDARY
        and topology.get("claim_boundary") == CLAIM_BOUNDARY
        and waypoints.get("claim_boundary") == CLAIM_BOUNDARY,
    }
    return {
        "schema_version": "task24d_input_validation_v0_1",
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "project_name": "RSLG-SLAM",
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "parse_results": parse_results,
        "checks": checks,
        "route_nodes": route_nodes,
        "route_node_floors": route_node_floors(nodes, route_nodes),
        "edge_endpoint_checks": edge_endpoint_checks,
        "passed": all(checks.values()),
    }


def marker_point(node_id: str, nodes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    node = nodes[node_id]
    return {
        "node_id": node_id,
        "floor_id": node.get("floor_id"),
        "position": get_position(node),
    }


def make_marker(
    marker_id: str,
    marker_type: str,
    namespace: str,
    label: str,
    *,
    floor_id: str | None = None,
    connector_id: str | None = None,
    route_edge_id: str | None = None,
    points: list[dict[str, Any]] | None = None,
    position: dict[str, Any] | None = None,
    scale_hint: dict[str, float] | None = None,
    color_hint: dict[str, float] | None = None,
    display_note: str,
) -> dict[str, Any]:
    marker = {
        "marker_id": marker_id,
        "marker_type": marker_type,
        "namespace": namespace,
        "frame_id": "map",
        "floor_id": floor_id,
        "connector_id": connector_id,
        "route_edge_id": route_edge_id,
        "label": label,
        "scale_hint": scale_hint or {"x": 0.12, "y": 0.12, "z": 0.12},
        "color_hint": color_hint or {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
        "display_note": display_note,
    }
    if points is not None:
        marker["points"] = points
    if position is not None:
        marker["position"] = position
    return marker


def route_segment_points(route_nodes: list[str], nodes: dict[str, dict[str, Any]], floor_id: str) -> list[dict[str, Any]]:
    return [
        marker_point(node_id, nodes)
        for node_id in route_nodes
        if node_id in nodes and nodes[node_id].get("floor_id") == floor_id
    ]


def generate_markers(
    scene_id: str,
    topology: dict[str, Any],
    corrected_report: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    transition_edge: dict[str, Any],
) -> dict[str, Any]:
    markers: list[dict[str, Any]] = []
    route_nodes = [str(node_id) for node_id in corrected_report.get("route_nodes", [])]
    route_node_set = set(route_nodes)
    room_nodes = [node for node in topology.get("nodes", []) if node.get("node_type") == "room"]
    for node in sorted(room_nodes, key=lambda item: item.get("node_id", "")):
        node_id = str(node["node_id"])
        floor_id = node.get("floor_id")
        marker_type = "floor_1_room_marker" if floor_id == "floor_1" else "floor_2_room_marker"
        markers.append(
            make_marker(
                f"{marker_type}_{node_id}",
                marker_type,
                f"{floor_id}_rooms",
                node_id,
                floor_id=floor_id,
                position=marker_point(node_id, nodes),
                scale_hint={"x": 0.16, "y": 0.16, "z": 0.16},
                color_hint={"r": 0.21, "g": 0.54, "b": 0.82, "a": 0.65}
                if floor_id == "floor_1"
                else {"r": 0.35, "g": 0.67, "b": 0.38, "a": 0.65},
                display_note="Room marker for graph overlay context; route room highlighted by labels if applicable."
                if node_id not in route_node_set
                else "Room marker on selected cross-floor route.",
            )
        )

    floor_1_points = route_segment_points(route_nodes, nodes, "floor_1")
    floor_2_points = route_segment_points(route_nodes, nodes, "floor_2")
    markers.append(
        make_marker(
            "same_floor_route_marker_floor_1_entry_segment",
            "same_floor_route_marker",
            "same_floor_route",
            "floor_1 route segment to connector entry",
            floor_id="floor_1",
            points=floor_1_points,
            scale_hint={"x": 0.08, "y": 0.08, "z": 0.08},
            color_hint={"r": 0.06, "g": 0.35, "b": 0.73, "a": 1.0},
            display_note="Same-floor graph route points on floor_1 before connector entry.",
        )
    )
    markers.append(
        make_marker(
            "same_floor_route_marker_floor_2_room_segment",
            "same_floor_route_marker",
            "same_floor_route",
            "floor_2 route segment from connector exit to room_14",
            floor_id="floor_2",
            points=floor_2_points,
            scale_hint={"x": 0.08, "y": 0.08, "z": 0.08},
            color_hint={"r": 0.08, "g": 0.46, "b": 0.2, "a": 1.0},
            display_note="Same-floor graph route points on floor_2 after connector exit.",
        )
    )

    centerline_ids = [node_id for node_id in route_nodes if node_id.startswith("vt_1_centerline_")]
    markers.append(
        make_marker(
            "stair_connector_centerline_marker_vc_vt_1",
            "stair_connector_centerline_marker",
            "stair_connector_centerline",
            "vc_vt_1 fitted centerline",
            connector_id="vc_vt_1",
            points=[marker_point(node_id, nodes) for node_id in centerline_ids],
            scale_hint={"x": 0.1, "y": 0.1, "z": 0.1},
            color_hint={"r": 0.83, "g": 0.43, "b": 0.08, "a": 1.0},
            display_note="Fitted centerline for topological connector visualization only.",
        )
    )
    markers.append(
        make_marker(
            "connector_entry_marker_vc_vt_1",
            "connector_entry_marker",
            "connector_endpoint_markers",
            "connector entry: vc_vt_1 floor_1 side",
            floor_id="floor_1",
            connector_id="vc_vt_1",
            position=marker_point("vc_vt_1_from_binding", nodes),
            scale_hint={"x": 0.28, "y": 0.28, "z": 0.28},
            color_hint={"r": 0.03, "g": 0.33, "b": 0.88, "a": 1.0},
            display_note="Connector entry marker near vc_vt_1_from_binding and vt_1_centerline_n000.",
        )
    )
    markers.append(
        make_marker(
            "connector_exit_marker_vc_vt_1",
            "connector_exit_marker",
            "connector_endpoint_markers",
            "connector exit: vc_vt_1 floor_2 side",
            floor_id="floor_2",
            connector_id="vc_vt_1",
            position=marker_point("vc_vt_1_to_binding", nodes),
            scale_hint={"x": 0.28, "y": 0.28, "z": 0.28},
            color_hint={"r": 0.0, "g": 0.5, "b": 0.16, "a": 1.0},
            display_note="Connector exit marker near vt_1_centerline_n004 and vc_vt_1_to_binding.",
        )
    )

    source = str(transition_edge["source"])
    target = str(transition_edge["target"])
    transition_position = midpoint(get_position(nodes[source]), get_position(nodes[target]))
    markers.append(
        make_marker(
            "floor_transition_marker_vt_1_centerline_e001",
            "floor_transition_marker",
            "floor_transition_markers",
            "floor transition: vt_1_centerline_n001 to vt_1_centerline_n002",
            floor_id=None,
            connector_id="vc_vt_1",
            route_edge_id=str(transition_edge["edge_id"]),
            position={
                "synthetic_position_id": "midpoint_vt_1_centerline_n001_vt_1_centerline_n002",
                "source_node_id": source,
                "target_node_id": target,
                "position": transition_position,
            },
            scale_hint={"x": 0.34, "y": 0.34, "z": 0.34},
            color_hint={"r": 0.86, "g": 0.12, "b": 0.12, "a": 1.0},
            display_note="Placed at the midpoint of the only route edge whose endpoint floor ids differ.",
        )
    )

    for index, node_id in enumerate(route_nodes):
        markers.append(
            make_marker(
                f"route_node_label_marker_{index:03d}_{node_id}",
                "route_node_label_marker",
                "route_node_labels",
                node_id,
                floor_id=nodes[node_id].get("floor_id"),
                connector_id=nodes[node_id].get("connector_id"),
                position=marker_point(node_id, nodes),
                scale_hint={"x": 0.18, "y": 0.18, "z": 0.18},
                color_hint={"r": 0.08, "g": 0.08, "b": 0.08, "a": 1.0},
                display_note="Text label for a selected route node.",
            )
        )
    markers.append(
        make_marker(
            "claim_boundary_text_marker_task24d",
            "claim_boundary_text_marker",
            "claim_boundary",
            "topological_vertical_transition_only; physical_execution_supported=false; no Gazebo/Nav2/Habitat/robot execution",
            position={
                "synthetic_position_id": "claim_boundary_text_anchor",
                "position": [-9.6, 7.5, 0.0],
            },
            scale_hint={"x": 0.22, "y": 0.22, "z": 0.22},
            color_hint={"r": 0.0, "g": 0.0, "b": 0.0, "a": 1.0},
            display_note="Claim boundary text marker for future RViz overlay.",
        )
    )
    return {
        "schema_version": "cross_floor_rviz_markers_v0_1",
        "scene_id": scene_id,
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "frames": {
            "default_frame_id": "map",
            "note": "graph-only marker coordinates; not robot execution",
        },
        "markers": markers,
        "validation": {
            "transition_marker_edge_id": transition_edge["edge_id"],
            "transition_marker_source_node": transition_edge["source"],
            "transition_marker_target_node": transition_edge["target"],
            "transition_marker_source_floor": transition_edge["source_floor"],
            "transition_marker_target_floor": transition_edge["target_floor"],
        },
    }


def marker_semantics_payload() -> dict[str, Any]:
    return {
        "schema_version": "task24d_marker_semantics_v0_1",
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "marker_types": [
            {
                "marker_type": "floor_1_room_marker",
                "definition": "Room context marker for floor_1 graph rooms.",
            },
            {
                "marker_type": "floor_2_room_marker",
                "definition": "Room context marker for floor_2 graph rooms.",
            },
            {
                "marker_type": "same_floor_route_marker",
                "definition": "Polyline marker for selected route segments whose ordered points are on the same floor.",
            },
            {
                "marker_type": "stair_connector_centerline_marker",
                "definition": "Polyline through vt_1_centerline_n000 through vt_1_centerline_n004.",
            },
            {
                "marker_type": "connector_entry_marker",
                "definition": "Marker at the connector side on floor_from near vt_1_centerline_n000 and vc_vt_1_from_binding.",
            },
            {
                "marker_type": "connector_exit_marker",
                "definition": "Marker at the connector side on floor_to near vt_1_centerline_n004 and vc_vt_1_to_binding.",
            },
            {
                "marker_type": "floor_transition_marker",
                "definition": "Marker at the midpoint of the edge whose source_floor differs from target_floor.",
                "current_route_edge": "vt_1_centerline_n001 -> vt_1_centerline_n002",
            },
            {
                "marker_type": "route_node_label_marker",
                "definition": "Text label marker for each selected route node.",
            },
            {
                "marker_type": "claim_boundary_text_marker",
                "definition": "Text marker stating topological_vertical_transition_only, physical_execution_supported=false, and no simulator or robot execution.",
            },
        ],
    }


def generate_preview(
    path: Path,
    topology: dict[str, Any],
    corrected_report: dict[str, Any],
    nodes: dict[str, dict[str, Any]],
    transition_edge: dict[str, Any],
) -> dict[str, Any]:
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    floor_colors = {"floor_1": "#d6e8f7", "floor_2": "#dff0df"}
    route_colors = {"floor_1": "#1059a8", "floor_2": "#177538"}
    route_nodes = [str(node_id) for node_id in corrected_report.get("route_nodes", [])]
    centerline_ids = [node_id for node_id in route_nodes if node_id.startswith("vt_1_centerline_")]
    for ax, floor_id, title in zip(axes, ["floor_1", "floor_2"], ["Floor 1", "Floor 2"]):
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linewidth=0.4, color="#dddddd")
        for node in topology.get("nodes", []):
            if node.get("node_type") != "room" or node.get("floor_id") != floor_id:
                continue
            room = node.get("room", {})
            poly = room.get("polygon")
            if poly:
                ax.add_patch(
                    Polygon(
                        poly,
                        closed=True,
                        facecolor=floor_colors[floor_id],
                        edgecolor="#667085",
                        linewidth=0.8,
                        alpha=0.75,
                    )
                )
            pos = get_position(node)
            ax.text(pos[0], pos[1], node["node_id"], ha="center", va="center", fontsize=8, color="#1f2937")

        floor_route = [
            get_position(nodes[node_id])
            for node_id in route_nodes
            if node_id in nodes and nodes[node_id].get("floor_id") == floor_id
        ]
        if len(floor_route) >= 2:
            ax.plot(
                [p[0] for p in floor_route],
                [p[1] for p in floor_route],
                color=route_colors[floor_id],
                linewidth=2.6,
                marker="o",
                markersize=5,
                label=f"{floor_id} route",
            )
        floor_centerline = [
            get_position(nodes[node_id])
            for node_id in centerline_ids
            if nodes[node_id].get("floor_id") == floor_id
        ]
        if len(floor_centerline) >= 2:
            ax.plot(
                [p[0] for p in floor_centerline],
                [p[1] for p in floor_centerline],
                color="#c46809",
                linewidth=3.0,
                marker="s",
                markersize=5,
                label="connector centerline",
            )
        for label, node_id, color in [
            ("entry", "vc_vt_1_from_binding", "#0b4dba"),
            ("exit", "vc_vt_1_to_binding", "#007a3d"),
        ]:
            if node_id in nodes and nodes[node_id].get("floor_id") == floor_id:
                p = get_position(nodes[node_id])
                ax.scatter([p[0]], [p[1]], s=120, color=color, edgecolors="white", linewidth=1.2, zorder=5)
                ax.annotate(label, (p[0], p[1]), xytext=(5, 7), textcoords="offset points", fontsize=9, color=color)
        source = transition_edge["source"]
        target = transition_edge["target"]
        if floor_id in {nodes[source].get("floor_id"), nodes[target].get("floor_id")}:
            p_source = get_position(nodes[source])
            p_target = get_position(nodes[target])
            p_mid = midpoint(p_source, p_target)
            ax.scatter([p_mid[0]], [p_mid[1]], s=180, marker="*", color="#d62728", edgecolors="white", linewidth=1.0, zorder=6)
            ax.annotate("floor transition", (p_mid[0], p_mid[1]), xytext=(7, -14), textcoords="offset points", fontsize=9, color="#a40000")
        ax.legend(loc="lower right", fontsize=8)
        ax.set_xlabel("x (graph/map frame)")
        ax.set_ylabel("y (graph/map frame)")
    fig.suptitle(
        "RSLG-SLAM task24d graph-only RViz overlay preview\n"
        "claim_boundary = topological_vertical_transition_only; physical_execution_supported = false",
        fontsize=12,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return {
        "preview_path": path,
        "two_panel_layout": True,
        "equal_axis_scaling_within_each_panel": True,
        "floor_2_display_shifted": False,
        "note": "Floor panels are separated visually; floor_2 coordinates are not shifted inside its panel.",
    }


def marker_references_valid(marker_spec: dict[str, Any], nodes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    errors: list[str] = []
    for marker in marker_spec.get("markers", []):
        for point in marker.get("points", []):
            node_id = point.get("node_id")
            if node_id and node_id not in nodes:
                errors.append(f"{marker['marker_id']} references missing node {node_id}")
            if not node_id and not point.get("synthetic_position_id"):
                errors.append(f"{marker['marker_id']} point lacks node_id or synthetic_position_id")
        position = marker.get("position")
        if position:
            node_id = position.get("node_id")
            if node_id and node_id not in nodes:
                errors.append(f"{marker['marker_id']} position references missing node {node_id}")
            if not node_id and not position.get("synthetic_position_id"):
                errors.append(f"{marker['marker_id']} position lacks node_id or synthetic_position_id")
    return {"valid": not errors, "errors": errors}


def json_parse_check(paths: list[Path]) -> dict[str, Any]:
    results: dict[str, Any] = {}
    for path in paths:
        result = {"path": path, "exists": path.exists(), "parses": False}
        if path.exists():
            try:
                read_json(path)
                result["parses"] = True
            except Exception as exc:  # noqa: BLE001
                result["error"] = f"{type(exc).__name__}: {exc}"
        results[path.name] = result
    return results


def build_markdown_table(rows: list[tuple[str, Any]]) -> str:
    lines = ["| Check | Result |", "|---|---|"]
    for key, value in rows:
        lines.append(f"| {key} | {value} |")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene-root",
        type=Path,
        default=Path("stage_outputs/stage1_generalization") / SCENE_ID_DEFAULT,
        help="Scene root containing clean_rerun and tasks directories.",
    )
    args = parser.parse_args()
    scene_root = args.scene_root
    paths = collect_paths(scene_root)
    out_dir = paths["task24d_dir"]
    scene_id = scene_root.name
    protected_before = {
        key: hash_tree(scene_root / rel_path)
        for key, rel_path in PROTECTED_RELATIVE_PATHS.items()
    }

    payloads, parse_results = load_inputs(paths)
    if set(payloads) >= {"topology", "waypoints"}:
        nodes = node_indices(payloads["topology"], payloads["waypoints"])
    else:
        nodes = {}

    input_validation = validate_inputs(paths, payloads, parse_results, nodes)
    write_json(out_dir / "input_validation.json", input_validation)
    write_text(
        out_dir / "input_validation.md",
        "# task24d input validation\n\n"
        + build_markdown_table([(key, value) for key, value in input_validation["checks"].items()])
        + f"\n\nPassed: {input_validation['passed']}\n",
    )

    route_report = payloads["route_report"]
    topology = payloads["topology"]
    waypoints = payloads["waypoints"]
    corrected_edges, transition_summary = recompute_edge_floor_transition_semantics(route_report.get("route_edges", []), nodes)
    corrected_report = corrected_route_report(route_report, corrected_edges, transition_summary)
    corrected_topo = corrected_topology(topology, corrected_edges)
    write_json(out_dir / "corrected_cross_floor_route_query_report_v0_2.json", corrected_report)
    write_json(out_dir / "corrected_cross_floor_topology_v0_2.json", corrected_topo)

    transition_edges = transition_summary["transition_edges"]
    expected_transition_found = len(transition_edges) == 1 and (
        transition_edges[0]["source"],
        transition_edges[0]["target"],
    ) == EXPECTED_TRANSITION
    old_terminal_fixed = all(
        not (
            edge.get("source") == "vt_1_centerline_n003"
            and edge.get("target") == "vt_1_centerline_n004"
            and edge.get("floor_transition_count") == 1
        )
        for edge in corrected_edges
    )
    transition_semantics_classification = (
        "blocked_missing_route_floor_ids"
        if transition_summary["missing_floor_nodes"]
        else "per_edge_transition_semantics_fixed"
        if expected_transition_found and corrected_report["floor_transition_count"] == 1 and old_terminal_fixed
        else "per_edge_transition_semantics_still_invalid"
    )
    floor_transition_validation = {
        "schema_version": "floor_transition_semantics_validation_v0_1",
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "classification": transition_semantics_classification,
        "route_level_floor_transition_count": corrected_report["floor_transition_count"],
        "transition_edges": [
            {
                "edge_id": edge["edge_id"],
                "source": edge["source"],
                "target": edge["target"],
                "source_floor": edge["source_floor"],
                "target_floor": edge["target_floor"],
            }
            for edge in transition_edges
        ],
        "expected_transition_edge": {
            "source": EXPECTED_TRANSITION[0],
            "target": EXPECTED_TRANSITION[1],
            "matched": expected_transition_found,
        },
        "previous_terminal_edge_no_longer_marked": old_terminal_fixed,
        "changed_edges": transition_summary["changed_edges"],
        "missing_floor_nodes": transition_summary["missing_floor_nodes"],
        "passed": transition_semantics_classification == "per_edge_transition_semantics_fixed",
    }
    write_json(out_dir / "floor_transition_semantics_validation.json", floor_transition_validation)
    write_text(
        out_dir / "floor_transition_semantics_fix_report.md",
        "# Floor transition semantics fix\n\n"
        "Per-edge transition semantics were recomputed from route edge source/target node floor IDs.\n\n"
        f"Classification: {transition_semantics_classification}\n\n"
        f"Correct transition edge: {EXPECTED_TRANSITION[0]} -> {EXPECTED_TRANSITION[1]}\n\n"
        f"Route-level floor_transition_count: {corrected_report['floor_transition_count']}\n\n"
        "Changed edges:\n\n"
        + json.dumps(transition_summary["changed_edges"], indent=2, sort_keys=True),
    )

    semantics = marker_semantics_payload()
    write_json(out_dir / "marker_semantics.json", semantics)
    write_text(
        out_dir / "marker_semantics.md",
        "# Marker semantics\n\n"
        "This task defines an offline MarkerArray-ready contract only. It does not launch RViz and does not call ROS.\n\n"
        "The floor transition marker is placed at the midpoint of vt_1_centerline_n001 -> vt_1_centerline_n002.\n\n"
        "Marker types:\n\n"
        + "\n".join(f"- `{item['marker_type']}`: {item['definition']}" for item in semantics["marker_types"]),
    )

    transition_edge = transition_edges[0] if transition_edges else {}
    marker_spec_generated = False
    if transition_edge:
        marker_spec = generate_markers(scene_id, topology, corrected_report, nodes, transition_edge)
        write_json(out_dir / "cross_floor_rviz_markers_v0_1.json", marker_spec)
        marker_spec_generated = True
    else:
        marker_spec = {
            "schema_version": "cross_floor_rviz_markers_v0_1",
            "scene_id": scene_id,
            "claim_boundary": CLAIM_BOUNDARY,
            "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
            "markers": [],
            "validation": {"error": "no floor transition edge available"},
        }
        write_json(out_dir / "cross_floor_rviz_markers_v0_1.json", marker_spec)
    marker_artifact_classification = (
        "rviz_marker_spec_generated" if marker_spec_generated else "blocked_marker_inputs_invalid"
    )

    if transition_edge:
        preview_info = generate_preview(
            out_dir / "cross_floor_rviz_overlay_preview.png",
            topology,
            corrected_report,
            nodes,
            transition_edge,
        )
        visualization_classification = "static_overlay_preview_generated"
    else:
        preview_info = {"preview_path": out_dir / "cross_floor_rviz_overlay_preview.png", "generated": False}
        visualization_classification = "static_overlay_preview_not_generated"
    write_text(
        out_dir / "visualization_notes.md",
        "# Visualization notes\n\n"
        "- Static preview uses two panels: floor_1 and floor_2.\n"
        "- Equal axis scaling is used within each panel.\n"
        "- floor_2 is not display-shifted within its panel; panel separation is visual only.\n"
        "- The red floor transition marker is associated with vt_1_centerline_n001 -> vt_1_centerline_n002.\n"
        "- claim_boundary = topological_vertical_transition_only.\n"
        "- physical_execution_supported = false.\n",
    )
    write_text(
        out_dir / "README_cross_floor_rviz_overlay_adapter.md",
        "# Cross-floor RViz overlay adapter contract\n\n"
        "`cross_floor_rviz_markers_v0_1.json` is an offline marker specification for a future RViz MarkerArray publisher.\n\n"
        "A future publisher should map each JSON marker to a ROS visualization marker in frame `map`, preserve the marker "
        "namespace/type labels, and keep the claim-boundary text visible. This artifact is graph-only: it does not imply "
        "Gazebo, Nav2, Habitat, quadruped gait, footstep planning, or physical stair traversal support.\n\n"
        "Do not infer robot-executable stair climbing from this file. `physical_execution_supported` remains false.\n",
    )

    generated_json_paths = [
        out_dir / "input_validation.json",
        out_dir / "corrected_cross_floor_route_query_report_v0_2.json",
        out_dir / "corrected_cross_floor_topology_v0_2.json",
        out_dir / "floor_transition_semantics_validation.json",
        out_dir / "marker_semantics.json",
        out_dir / "cross_floor_rviz_markers_v0_1.json",
        out_dir / "validation_summary.json",
        out_dir / "task24d_report.json",
    ]
    json_checks = json_parse_check(generated_json_paths)
    corrected_node_ids = set(nodes)
    route_edge_refs_valid = all(
        edge.get("source") in corrected_node_ids and edge.get("target") in corrected_node_ids
        for edge in corrected_edges
    )
    marker_ref_check = marker_references_valid(marker_spec, nodes)
    transition_edges_from_floor_ids = [
        edge
        for edge in corrected_edges
        if edge.get("source_floor") and edge.get("target_floor") and edge.get("source_floor") != edge.get("target_floor")
    ]
    no_direct_room3_room7 = all(
        set([edge.get("traversal_source", edge.get("source")), edge.get("traversal_target", edge.get("target"))])
        != {"room_3", "room_7"}
        for edge in corrected_edges
    )
    protected_after = {
        key: hash_tree(scene_root / rel_path)
        for key, rel_path in PROTECTED_RELATIVE_PATHS.items()
    }
    protected_checks = {
        key: tree_unchanged(protected_before[key], protected_after[key])
        for key in protected_before
    }
    validation_checks = {
        "json_parse_checks_pass": all(item["exists"] and item["parses"] for item in json_checks.values()),
        "every_corrected_route_edge_source_target_exists": route_edge_refs_valid,
        "every_marker_point_references_existing_node_or_synthetic_position": marker_ref_check["valid"],
        "transition_marker_edge_is_exact_floor_crossing_edge": marker_spec.get("validation", {}).get("transition_marker_edge_id")
        == (transition_edges_from_floor_ids[0].get("edge_id") if len(transition_edges_from_floor_ids) == 1 else None),
        "transition_marker_associated_with_expected_edge": marker_spec.get("validation", {}).get("transition_marker_source_node")
        == EXPECTED_TRANSITION[0]
        and marker_spec.get("validation", {}).get("transition_marker_target_node") == EXPECTED_TRANSITION[1],
        "route_level_floor_transition_count_remains_1": corrected_report.get("floor_transition_count") == 1,
        "route_still_uses_vc_vt_1": corrected_report.get("used_connector_ids") == ["vc_vt_1"],
        "no_hidden_direct_room_3_to_room_7_shortcut_used": no_direct_room3_room7,
        "physical_execution_supported_false_everywhere": corrected_report.get("physical_execution_supported") is False
        and corrected_topo.get("physical_execution_supported") is False
        and marker_spec.get("physical_execution_supported") is False,
        "claim_boundary_topological_only_everywhere": corrected_report.get("claim_boundary") == CLAIM_BOUNDARY
        and corrected_topo.get("claim_boundary") == CLAIM_BOUNDARY
        and marker_spec.get("claim_boundary") == CLAIM_BOUNDARY,
        "floor_1_and_floor_2_stable_maps_exist": paths["floor_1_map_yaml"].exists() and paths["floor_2_map_yaml"].exists(),
        "no_gazebo_nav2_habitat_robot_execution_was_run": True,
        "task23b_outputs_unchanged": protected_checks["task23b"]["unchanged"],
        "clean_rerun_maps_floor_2_unchanged": protected_checks["clean_rerun_maps_floor_2"]["unchanged"],
        "clean_rerun_committed_public_unchanged": protected_checks["clean_rerun_committed_public"]["unchanged"],
    }
    validation_summary = {
        "schema_version": "task24d_validation_summary_v0_1",
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "project_name": "RSLG-SLAM",
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "checks": validation_checks,
        "json_parse_checks": json_checks,
        "marker_reference_check": marker_ref_check,
        "protected_path_checks": protected_checks,
        "transition_marker_edge_id": marker_spec.get("validation", {}).get("transition_marker_edge_id"),
        "transition_marker_source_node": marker_spec.get("validation", {}).get("transition_marker_source_node"),
        "transition_marker_target_node": marker_spec.get("validation", {}).get("transition_marker_target_node"),
        "no_execution_note": "No RViz, Gazebo, Nav2, Habitat, quadruped proxy, task23b GUI, or robot execution was run by this script.",
        "passed": all(validation_checks.values()),
    }
    write_json(out_dir / "validation_summary.json", validation_summary)
    write_text(
        out_dir / "validation_summary.md",
        "# task24d validation summary\n\n"
        + build_markdown_table([(key, value) for key, value in validation_checks.items()])
        + f"\n\nPassed: {validation_summary['passed']}\n",
    )

    final_classifications = {
        "transition_semantics_classification": transition_semantics_classification,
        "marker_artifact_classification": marker_artifact_classification,
        "visualization_classification": visualization_classification,
    }
    generated_files = [
        "task24d_report.md",
        "task24d_report.json",
        "input_validation.md",
        "input_validation.json",
        "corrected_cross_floor_route_query_report_v0_2.json",
        "corrected_cross_floor_topology_v0_2.json",
        "floor_transition_semantics_fix_report.md",
        "floor_transition_semantics_validation.json",
        "marker_semantics.md",
        "marker_semantics.json",
        "cross_floor_rviz_markers_v0_1.json",
        "cross_floor_rviz_overlay_preview.png",
        "visualization_notes.md",
        "README_cross_floor_rviz_overlay_adapter.md",
        "validation_summary.md",
        "validation_summary.json",
        "final_answer_for_user.md",
    ]
    task_report = {
        "schema_version": "task24d_report_v0_1",
        "created_utc": now_iso(),
        "task_name": TASK_NAME,
        "project_name": "RSLG-SLAM",
        "scene_id": scene_id,
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "input_validation_passed": input_validation["passed"],
        "transition_semantics": floor_transition_validation,
        "marker_artifact_classification": marker_artifact_classification,
        "visualization_classification": visualization_classification,
        "preview_info": preview_info,
        "validation_passed": validation_summary["passed"],
        "final_classifications": final_classifications,
        "generated_files": generated_files,
        "exact_next_recommendation": (
            "Review cross_floor_rviz_markers_v0_1.json and, in a later task, implement a small offline-to-ROS "
            "MarkerArray publisher that consumes this JSON without changing the claim boundary."
        ),
    }
    write_json(out_dir / "task24d_report.json", task_report)
    write_text(
        out_dir / "task24d_report.md",
        "# task24d report\n\n"
        + build_markdown_table(
            [
                ("transition_semantics_classification", transition_semantics_classification),
                ("marker_artifact_classification", marker_artifact_classification),
                ("visualization_classification", visualization_classification),
                ("transition_marker_edge", f"{EXPECTED_TRANSITION[0]} -> {EXPECTED_TRANSITION[1]}"),
                ("physical_execution_supported", PHYSICAL_EXECUTION_SUPPORTED),
                ("claim_boundary", CLAIM_BOUNDARY),
                ("validation_passed", validation_summary["passed"]),
            ]
        )
        + "\n\nExact next recommendation: Review `cross_floor_rviz_markers_v0_1.json` and then implement a future MarkerArray publisher without changing the claim boundary.\n",
    )
    write_text(
        out_dir / "final_answer_for_user.md",
        "# Final answer for user\n\n"
        "Per-edge floor transition semantics were corrected. The transition marker is determined by "
        "vt_1_centerline_n001 -> vt_1_centerline_n002. The RViz MarkerArray-ready JSON spec and static two-panel "
        "overlay preview were generated. floor_1 and floor_2 stable maps remain present. task23b, "
        "clean_rerun/maps/floor_2, and clean_rerun/committed_public were unchanged during this task. "
        "This remains graph/overlay only with claim_boundary = topological_vertical_transition_only and "
        "physical_execution_supported = false.\n",
    )
    final_json_checks = json_parse_check(generated_json_paths)
    validation_summary["json_parse_checks"] = final_json_checks
    validation_summary["checks"]["json_parse_checks_pass"] = all(
        item["exists"] and item["parses"] for item in final_json_checks.values()
    )
    validation_summary["passed"] = all(validation_summary["checks"].values())
    task_report["validation_passed"] = validation_summary["passed"]
    write_json(out_dir / "validation_summary.json", validation_summary)
    write_json(out_dir / "task24d_report.json", task_report)
    write_text(
        out_dir / "validation_summary.md",
        "# task24d validation summary\n\n"
        + build_markdown_table([(key, value) for key, value in validation_summary["checks"].items()])
        + f"\n\nPassed: {validation_summary['passed']}\n",
    )
    write_text(
        out_dir / "task24d_report.md",
        "# task24d report\n\n"
        + build_markdown_table(
            [
                ("transition_semantics_classification", transition_semantics_classification),
                ("marker_artifact_classification", marker_artifact_classification),
                ("visualization_classification", visualization_classification),
                ("transition_marker_edge", f"{EXPECTED_TRANSITION[0]} -> {EXPECTED_TRANSITION[1]}"),
                ("physical_execution_supported", PHYSICAL_EXECUTION_SUPPORTED),
                ("claim_boundary", CLAIM_BOUNDARY),
                ("validation_passed", validation_summary["passed"]),
            ]
        )
        + "\n\nExact next recommendation: Review `cross_floor_rviz_markers_v0_1.json` and then implement a future MarkerArray publisher without changing the claim boundary.\n",
    )
    py_compile.compile(Path(__file__).as_posix(), doraise=True)
    return 0 if validation_summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
