#!/usr/bin/env python3
"""Build graph-only cross-floor connector route artifacts for RSLG-SLAM."""

from __future__ import annotations

import argparse
import ast
import hashlib
import heapq
import json
import math
import os
import py_compile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


TASK_NAME = "task24c_rslg_cross_floor_connector_router_and_overlay"
SCENE_ID_DEFAULT = "00843-DYehNKdT76V"
CLAIM_BOUNDARY = "topological_vertical_transition_only"
PHYSICAL_EXECUTION_SUPPORTED = False


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


def sha256(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_tree(root: Path) -> dict[str, str]:
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): sha256(path) or ""
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def parse_map_yaml(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        value = raw_value.strip()
        if value.startswith("[") and value.endswith("]"):
            parsed: Any = ast.literal_eval(value)
        else:
            try:
                parsed = int(value)
            except ValueError:
                try:
                    parsed = float(value)
                except ValueError:
                    parsed = value
        meta[key.strip()] = parsed
    return meta


def read_pgm_header(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4 and idx < len(data):
        while idx < len(data) and data[idx : idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx : idx + 1] == b"#":
            while idx < len(data) and data[idx : idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx : idx + 1].isspace():
            idx += 1
        if start != idx:
            tokens.append(data[start:idx])
    if len(tokens) < 4:
        raise ValueError(f"incomplete PGM header: {path}")
    if tokens[0] != b"P5":
        raise ValueError(f"unsupported PGM magic {tokens[0]!r}: {path}")
    width, height, max_value = int(tokens[1]), int(tokens[2]), int(tokens[3])
    return {
        "magic": tokens[0].decode("ascii"),
        "width": width,
        "height": height,
        "max_value": max_value,
        "file_size_bytes": path.stat().st_size,
    }


def room_label(room_id: Any) -> str:
    text = str(room_id)
    if text.startswith("room_"):
        return text
    return f"room_{text}"


def xyz_for_node(node: dict[str, Any]) -> list[float]:
    xyz = node.get("xyz") or node.get("position_xyz")
    if xyz is None:
        raise KeyError(f"node lacks xyz/position_xyz: {node}")
    return [float(xyz[0]), float(xyz[1]), float(xyz[2])]


def distance_xyz(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def distance_xy(a: list[float], b: list[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def edge_base_cost(edge: dict[str, Any], rooms: dict[str, dict[str, Any]]) -> float:
    if "distance_3d_m" in edge:
        return float(edge["distance_3d_m"])
    source = edge.get("source")
    target = edge.get("target")
    if source in rooms and target in rooms:
        dist = distance_xy(rooms[source].get("center", [0.0, 0.0]), rooms[target].get("center", [0.0, 0.0]))
    else:
        dist = 1.0
    confidence = float(edge.get("confidence", 1.0) or 1.0)
    confidence = max(confidence, 0.05)
    return round(dist / confidence, 6)


def find_raw_trace(scene_root: Path) -> dict[str, Any]:
    task24b4 = scene_root / "tasks" / "task24b4_stable_map_preview_and_stair_graph_binding_fix" / "raw_transition_pose_trace_vt_1.json"
    task24b3 = scene_root / "tasks" / "task24b3_stair_trace_centerline_and_all_floor_map_regression_fix" / "raw_transition_pose_trace_vt_1.json"
    if task24b4.exists():
        return {
            "ref": "raw_transition_pose_trace_vt_1.json",
            "path": task24b4,
            "exists_at_task24b4_path": True,
            "provenance_note": "raw camera pose trace retained as provenance only",
        }
    if task24b3.exists():
        return {
            "ref": "raw_transition_pose_trace_vt_1.json",
            "path": task24b3,
            "exists_at_task24b4_path": False,
            "provenance_note": "task24b4 graph references this basename; source copy exists in task24b3 and remains provenance only",
        }
    return {
        "ref": "raw_transition_pose_trace_vt_1.json",
        "path": task24b4,
        "exists_at_task24b4_path": False,
        "provenance_note": "raw trace not found; connector geometry still uses fitted centerline nodes only",
    }


def collect_paths(scene_root: Path) -> dict[str, Path]:
    clean = scene_root / "clean_rerun"
    b4 = scene_root / "tasks" / "task24b4_stable_map_preview_and_stair_graph_binding_fix"
    return {
        "topology": clean / "committed_public" / "topology_v0_1.json",
        "vertical_evidence": clean / "committed_public" / "vertical_transition_evidence.json",
        "floor_1_yaml": clean / "maps" / "floor_1" / "stage1_floor_1_stable_occupancy_map.yaml",
        "floor_2_yaml": clean / "maps" / "floor_2" / "stage1_floor_2_stable_occupancy_map.yaml",
        "stairs_graph": b4 / "stairs_graph_v0_3.json",
        "sparse_graph": b4 / "sparse_stair_connector_graph_vt_1_v0_3.json",
    }


def validate_map_package(yaml_path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "yaml": yaml_path,
        "yaml_exists": yaml_path.exists(),
        "yaml_parses": False,
        "pgm_exists": False,
        "npz_exists": yaml_path.with_suffix(".npz").exists(),
        "preview_png_exists": yaml_path.with_name(yaml_path.stem + "_preview.png").exists(),
        "passed": False,
    }
    if not yaml_path.exists():
        return result
    try:
        meta = parse_map_yaml(yaml_path)
        result["yaml_parses"] = True
        result["yaml_meta"] = meta
        image_name = str(meta.get("image", ""))
        pgm_path = yaml_path.parent / image_name
        result["pgm"] = pgm_path
        result["pgm_exists"] = pgm_path.exists()
        if pgm_path.exists():
            result["pgm_header"] = read_pgm_header(pgm_path)
    except Exception as exc:  # noqa: BLE001 - validation should report all issues.
        result["error"] = f"{type(exc).__name__}: {exc}"
    result["passed"] = bool(
        result["yaml_exists"]
        and result["yaml_parses"]
        and result["pgm_exists"]
        and result["npz_exists"]
        and result["preview_png_exists"]
    )
    return result


def validate_inputs(scene_root: Path, paths: dict[str, Path], topology: dict[str, Any] | None, stairs_graph: dict[str, Any] | None, sparse_graph: dict[str, Any] | None) -> dict[str, Any]:
    required = {
        key: {"path": path, "exists": path.exists()}
        for key, path in paths.items()
    }
    maps = {
        "floor_1": validate_map_package(paths["floor_1_yaml"]),
        "floor_2": validate_map_package(paths["floor_2_yaml"]),
    }
    checks: dict[str, Any] = {}
    if sparse_graph:
        node_ids = {node.get("node_id") for node in sparse_graph.get("nodes", [])}
        bindings = sparse_graph.get("endpoint_bindings", {})
        from_id = bindings.get("from", {}).get("endpoint_node")
        to_id = bindings.get("to", {}).get("endpoint_node")
        checks["sparse_endpoint_bindings_exist"] = {
            "from": from_id,
            "to": to_id,
            "from_exists": from_id in node_ids,
            "to_exists": to_id in node_ids,
            "passed": from_id in node_ids and to_id in node_ids,
        }
        checks["sparse_claim_boundary"] = {
            "value": sparse_graph.get("claim_boundary"),
            "passed": sparse_graph.get("claim_boundary") == CLAIM_BOUNDARY,
        }
        checks["sparse_physical_execution_supported"] = {
            "value": sparse_graph.get("physical_execution_supported"),
            "passed": sparse_graph.get("physical_execution_supported") is PHYSICAL_EXECUTION_SUPPORTED,
        }
    if stairs_graph:
        stair_node_ids = {node.get("node_id") for node in stairs_graph.get("nodes", [])}
        bad_edges = [
            edge.get("edge_id")
            for edge in stairs_graph.get("edges", [])
            if edge.get("source") not in stair_node_ids or edge.get("target") not in stair_node_ids
        ]
        checks["stairs_graph_edge_endpoints_exist"] = {
            "bad_edge_ids": bad_edges,
            "passed": not bad_edges,
        }
        checks["stairs_claim_boundary"] = {
            "value": stairs_graph.get("claim_boundary"),
            "passed": stairs_graph.get("claim_boundary") == CLAIM_BOUNDARY,
        }
        checks["stairs_physical_execution_supported"] = {
            "value": stairs_graph.get("physical_execution_supported"),
            "passed": stairs_graph.get("physical_execution_supported") is PHYSICAL_EXECUTION_SUPPORTED,
        }
    raw_trace = find_raw_trace(scene_root)
    all_required_exist = all(item["exists"] for item in required.values())
    maps_present = maps["floor_1"]["passed"] and maps["floor_2"]["passed"]
    graph_valid = all(check.get("passed") for check in checks.values())
    return {
        "schema_version": "input_artifact_validation_v0_1",
        "task": TASK_NAME,
        "created_utc": now_iso(),
        "scene_root": scene_root,
        "required_inputs": required,
        "map_packages": maps,
        "graph_checks": checks,
        "raw_transition_pose_trace": raw_trace,
        "input_artifacts_valid": bool(all_required_exist and maps_present and graph_valid and topology is not None),
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
    }


def build_vertical_connectors(
    scene_root: Path,
    connector_id: str,
    topology: dict[str, Any],
    vertical_evidence: dict[str, Any],
    sparse_graph: dict[str, Any],
    paths: dict[str, Path],
) -> dict[str, Any]:
    nodes = sparse_graph.get("nodes", [])
    edges = sparse_graph.get("edges", [])
    node_by_id = {node["node_id"]: node for node in nodes}
    bindings = sparse_graph.get("endpoint_bindings", {})
    from_binding = bindings["from"]
    to_binding = bindings["to"]
    from_node_id = from_binding["endpoint_node"]
    to_node_id = to_binding["endpoint_node"]
    confidence = 0.0
    z_span_m = None
    for transition in vertical_evidence.get("summary", {}).get("transitions", []):
        if transition.get("transition_id") == sparse_graph.get("transition_id", "vt_1"):
            confidence = float(transition.get("confidence", confidence) or confidence)
            z_span_m = transition.get("z_span_m")
    for edge in topology.get("edges", []):
        if edge.get("relation_type") == "vertical_transition":
            records = edge.get("metadata", {}).get("transition_records", [])
            if sparse_graph.get("transition_id", "vt_1") in edge.get("metadata", {}).get("transition_ids", []):
                confidence = max(confidence, float(records[0].get("confidence", edge.get("confidence", 0.0))) if records else float(edge.get("confidence", 0.0)))
    cost = round(sum(float(edge.get("distance_3d_m", 1.0)) for edge in edges), 6)
    connector_nodes = [
        {
            **node,
            "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
            "claim_boundary": CLAIM_BOUNDARY,
        }
        for node in nodes
    ]
    connector_edges = [
        {
            **edge,
            "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
            "claim_boundary": CLAIM_BOUNDARY,
        }
        for edge in edges
    ]
    raw_trace = find_raw_trace(scene_root)
    connector = {
        "connector_id": connector_id,
        "source_transition_id": sparse_graph.get("transition_id", "vt_1"),
        "connector_type": "stairs",
        "source": "pose_height_transition_with_fitted_centerline",
        "graph_shape": sparse_graph.get("graph_shape", "straight"),
        "floor_from": sparse_graph.get("floor_from", from_binding.get("floor_from")),
        "floor_to": sparse_graph.get("floor_to", to_binding.get("floor_to")),
        "room_from": from_binding.get("bound_room_id") or from_binding.get("room_id"),
        "room_to": to_binding.get("bound_room_id") or to_binding.get("room_id"),
        "endpoint_from": {
            "node_id": from_node_id,
            "position_xyz": xyz_for_node(node_by_id[from_node_id]),
            "bound_room_id": from_binding.get("bound_room_id"),
            "bound_topology_node_id": from_binding.get("bound_topology_node_id"),
        },
        "endpoint_to": {
            "node_id": to_node_id,
            "position_xyz": xyz_for_node(node_by_id[to_node_id]),
            "bound_room_id": to_binding.get("bound_room_id"),
            "bound_topology_node_id": to_binding.get("bound_topology_node_id"),
        },
        "sparse_stair_connector_graph": {
            "nodes": connector_nodes,
            "edges": connector_edges,
        },
        "raw_transition_pose_trace_ref": raw_trace["ref"],
        "cost": cost,
        "confidence": round(confidence, 6),
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "claim_boundary": CLAIM_BOUNDARY,
        "provenance": {
            "scene_root": scene_root,
            "sparse_stair_connector_graph": paths["sparse_graph"],
            "stairs_graph": paths["stairs_graph"],
            "vertical_transition_evidence": paths["vertical_evidence"],
            "raw_transition_pose_trace_path": raw_trace["path"],
            "raw_trace_provenance_note": raw_trace["provenance_note"],
            "geometry_interpretation": sparse_graph.get("geometry_interpretation", "fitted connector centerline from pose trace, not physical stair mesh"),
            "z_span_m": z_span_m,
        },
        "missing_evidence": [
            "physical stair-step geometry",
            "robot-executable stair traversal validation",
            "footstep support evidence",
        ],
    }
    return {
        "schema_version": "vertical_connectors_v0_1",
        "scene_id": topology.get("sequence_id", SCENE_ID_DEFAULT),
        "claim_boundary": CLAIM_BOUNDARY,
        "connectors": [connector],
    }


def build_cross_floor_topology(
    topology: dict[str, Any],
    vertical_connectors: dict[str, Any],
) -> dict[str, Any]:
    connector = vertical_connectors["connectors"][0]
    rooms = {room["id"]: room for room in topology.get("rooms", [])}
    nodes: list[dict[str, Any]] = []
    for room in topology.get("rooms", []):
        nodes.append(
            {
                "node_id": room["id"],
                "node_type": "room",
                "floor_id": room.get("floor_id"),
                "position_xy": room.get("center"),
                "room": room,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
            }
        )
    from_endpoint_id = f"{connector['connector_id']}_from_binding"
    to_endpoint_id = f"{connector['connector_id']}_to_binding"
    nodes.extend(
        [
            {
                "node_id": from_endpoint_id,
                "node_type": "connector_endpoint",
                "connector_id": connector["connector_id"],
                "endpoint_role": "from",
                "floor_id": connector["floor_from"],
                "bound_room_id": connector["room_from"],
                "bound_topology_node_id": connector["endpoint_from"]["bound_topology_node_id"],
                "bound_stair_node_id": connector["endpoint_from"]["node_id"],
                "position_xyz": connector["endpoint_from"]["position_xyz"],
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
            },
            {
                "node_id": to_endpoint_id,
                "node_type": "connector_endpoint",
                "connector_id": connector["connector_id"],
                "endpoint_role": "to",
                "floor_id": connector["floor_to"],
                "bound_room_id": connector["room_to"],
                "bound_topology_node_id": connector["endpoint_to"]["bound_topology_node_id"],
                "bound_stair_node_id": connector["endpoint_to"]["node_id"],
                "position_xyz": connector["endpoint_to"]["position_xyz"],
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
            },
        ]
    )
    for stair_node in connector["sparse_stair_connector_graph"]["nodes"]:
        nodes.append(
            {
                "node_id": stair_node["node_id"],
                "node_type": "stair_connector_node",
                "connector_id": connector["connector_id"],
                "floor_id": connector["floor_from"] if float(stair_node.get("sample_t", 0.0)) < 0.5 else connector["floor_to"],
                "position_xyz": xyz_for_node(stair_node),
                "source_node": stair_node,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
            }
        )
    edges: list[dict[str, Any]] = []
    transformed_vertical_edges: list[dict[str, Any]] = []
    for idx, edge in enumerate(topology.get("edges", [])):
        source = edge.get("source")
        target = edge.get("target")
        same_floor = rooms.get(source, {}).get("floor_id") == rooms.get(target, {}).get("floor_id")
        if edge.get("relation_type") == "vertical_transition":
            transformed_vertical_edges.append(edge)
            continue
        if not same_floor:
            transformed_vertical_edges.append(edge)
            continue
        edges.append(
            {
                "edge_id": f"topo_e{idx:03d}_{edge.get('relation_type')}_{source}_{target}",
                "source": source,
                "target": target,
                "edge_type": "same_floor_topology",
                "relation_type": edge.get("relation_type"),
                "cost": edge_base_cost(edge, rooms),
                "floor_transition_count": 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {
                    "source_topology_edge_index": idx,
                    "source_topology_edge": edge,
                },
            }
        )
    binding_cost_from = distance_xy(
        rooms[connector["room_from"]]["center"],
        connector["endpoint_from"]["position_xyz"][:2],
    )
    binding_cost_to = distance_xy(
        rooms[connector["room_to"]]["center"],
        connector["endpoint_to"]["position_xyz"][:2],
    )
    edges.extend(
        [
            {
                "edge_id": f"{connector['connector_id']}_bind_from_room_to_endpoint",
                "source": connector["room_from"],
                "target": from_endpoint_id,
                "edge_type": "connector_endpoint_binding",
                "connector_id": connector["connector_id"],
                "cost": round(binding_cost_from, 6),
                "floor_transition_count": 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {"bound_stair_node_id": connector["endpoint_from"]["node_id"]},
            },
            {
                "edge_id": f"{connector['connector_id']}_bind_from_endpoint_to_centerline",
                "source": from_endpoint_id,
                "target": connector["endpoint_from"]["node_id"],
                "edge_type": "connector_endpoint_binding",
                "connector_id": connector["connector_id"],
                "cost": 0.0,
                "floor_transition_count": 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {"bound_stair_node_id": connector["endpoint_from"]["node_id"]},
            },
        ]
    )
    stair_edges = connector["sparse_stair_connector_graph"]["edges"]
    for stair_edge in stair_edges:
        is_terminal_transition = stair_edge.get("target") == connector["endpoint_to"]["node_id"]
        edges.append(
            {
                "edge_id": stair_edge["edge_id"],
                "source": stair_edge["source"],
                "target": stair_edge["target"],
                "edge_type": "stair_connector_edge",
                "connector_id": connector["connector_id"],
                "cost": float(stair_edge.get("distance_3d_m", 1.0)),
                "floor_transition_count": 1 if is_terminal_transition else 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {
                    "source_sparse_stair_connector_edge": stair_edge,
                    "transition_count_note": "the fitted stair connector contributes one total floor transition; assigned to the terminal centerline edge",
                },
            }
        )
    edges.extend(
        [
            {
                "edge_id": f"{connector['connector_id']}_bind_to_centerline_to_endpoint",
                "source": connector["endpoint_to"]["node_id"],
                "target": to_endpoint_id,
                "edge_type": "connector_endpoint_binding",
                "connector_id": connector["connector_id"],
                "cost": 0.0,
                "floor_transition_count": 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {"bound_stair_node_id": connector["endpoint_to"]["node_id"]},
            },
            {
                "edge_id": f"{connector['connector_id']}_bind_to_endpoint_to_room",
                "source": to_endpoint_id,
                "target": connector["room_to"],
                "edge_type": "connector_endpoint_binding",
                "connector_id": connector["connector_id"],
                "cost": round(binding_cost_to, 6),
                "floor_transition_count": 0,
                "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
                "claim_boundary": CLAIM_BOUNDARY,
                "provenance": {"bound_stair_node_id": connector["endpoint_to"]["node_id"]},
            },
        ]
    )
    return {
        "schema_version": "cross_floor_topology_v0_1",
        "scene_id": topology.get("sequence_id", SCENE_ID_DEFAULT),
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "nodes": nodes,
        "edges": edges,
        "transformed_vertical_edges": transformed_vertical_edges,
        "connectors": [
            {
                "connector_id": connector["connector_id"],
                "source_transition_id": connector["source_transition_id"],
                "room_from": connector["room_from"],
                "room_to": connector["room_to"],
                "connector_route_nodes": [
                    connector["room_from"],
                    from_endpoint_id,
                    *[node["node_id"] for node in connector["sparse_stair_connector_graph"]["nodes"]],
                    to_endpoint_id,
                    connector["room_to"],
                ],
            }
        ],
        "provenance": {
            "source_topology_schema_version": topology.get("version"),
            "direct_vertical_room_edges_are_not_routable_shortcuts": True,
            "same_floor_topology_note": "edges are preserved as single source records; the router may traverse them bidirectionally",
        },
    }


def dijkstra_route(cross_topology: dict[str, Any], start: str, goal: str, required_connector_id: str | None = None) -> dict[str, Any]:
    edge_by_id = {edge["edge_id"]: edge for edge in cross_topology.get("edges", [])}
    adjacency: dict[str, list[tuple[float, str, str, bool]]] = {}
    for edge in cross_topology.get("edges", []):
        source = edge["source"]
        target = edge["target"]
        cost = float(edge.get("cost", 1.0))
        undirected = edge["edge_type"] == "same_floor_topology"
        adjacency.setdefault(source, []).append((cost, target, edge["edge_id"], False))
        if undirected:
            adjacency.setdefault(target, []).append((cost, source, edge["edge_id"], True))
    queue: list[tuple[float, str]] = [(0.0, start)]
    best: dict[str, float] = {start: 0.0}
    previous: dict[str, tuple[str, str, bool]] = {}
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != best.get(node):
            continue
        if node == goal:
            break
        for edge_cost, next_node, edge_id, reversed_traversal in adjacency.get(node, []):
            next_cost = cost + edge_cost
            if next_cost < best.get(next_node, float("inf")):
                best[next_node] = next_cost
                previous[next_node] = (node, edge_id, reversed_traversal)
                heapq.heappush(queue, (next_cost, next_node))
    if goal not in best:
        return {
            "route_found": False,
            "failure_reason": f"no graph route from {start} to {goal}",
        }
    route_nodes = [goal]
    route_edges_rev: list[dict[str, Any]] = []
    cursor = goal
    while cursor != start:
        prev_node, edge_id, reversed_traversal = previous[cursor]
        edge = edge_by_id[edge_id]
        route_edges_rev.append(
            {
                "edge_id": edge_id,
                "source": edge["source"],
                "target": edge["target"],
                "traversal_source": prev_node,
                "traversal_target": cursor,
                "traversed_reverse": reversed_traversal,
                "edge_type": edge["edge_type"],
                "connector_id": edge.get("connector_id"),
                "cost": edge.get("cost"),
                "floor_transition_count": edge.get("floor_transition_count", 0),
            }
        )
        cursor = prev_node
        route_nodes.append(cursor)
    route_nodes.reverse()
    route_edges = list(reversed(route_edges_rev))
    used_connector_ids = sorted({edge["connector_id"] for edge in route_edges if edge.get("connector_id")})
    if required_connector_id and required_connector_id not in used_connector_ids:
        return {
            "route_found": False,
            "failure_reason": f"route found but did not use required connector {required_connector_id}",
            "candidate_route_nodes": route_nodes,
            "candidate_used_connector_ids": used_connector_ids,
        }
    node_by_id = {node["node_id"]: node for node in cross_topology.get("nodes", [])}
    floor_sequence = []
    for node_id in route_nodes:
        floor_id = node_by_id.get(node_id, {}).get("floor_id")
        if floor_id and (not floor_sequence or floor_sequence[-1] != floor_id):
            floor_sequence.append(floor_id)
    forbidden_shortcut = any(
        edge["traversal_source"] == "room_3" and edge["traversal_target"] == "room_7"
        or edge["traversal_source"] == "room_7" and edge["traversal_target"] == "room_3"
        for edge in route_edges
    )
    return {
        "route_found": True,
        "route_nodes": route_nodes,
        "route_edges": route_edges,
        "used_connector_ids": used_connector_ids,
        "floor_sequence": floor_sequence,
        "floor_transition_count": int(sum(int(edge.get("floor_transition_count", 0)) for edge in route_edges)),
        "route_cost": round(float(best[goal]), 6),
        "same_floor_edge_count": sum(1 for edge in route_edges if edge["edge_type"] == "same_floor_topology"),
        "connector_edge_count": sum(1 for edge in route_edges if edge["edge_type"] in {"connector_endpoint_binding", "stair_connector_edge"}),
        "forbidden_shortcut_detected": forbidden_shortcut,
    }


def build_route_report(cross_topology: dict[str, Any], start_room: str, goal_room: str, connector_id: str) -> dict[str, Any]:
    route = dijkstra_route(cross_topology, start_room, goal_room, connector_id)
    report = {
        "schema_version": "cross_floor_route_query_report_v0_1",
        "created_utc": now_iso(),
        "start_room": start_room,
        "goal_room": goal_room,
        "connector_id": connector_id,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    report.update(route)
    if report.get("route_found"):
        report["failure_reason"] = None
    return report


def build_waypoints(cross_topology: dict[str, Any], route_report: dict[str, Any]) -> dict[str, Any]:
    node_by_id = {node["node_id"]: node for node in cross_topology.get("nodes", [])}
    waypoints: list[dict[str, Any]] = []
    route_nodes = route_report.get("route_nodes", [])
    for idx, node_id in enumerate(route_nodes):
        node = node_by_id[node_id]
        node_type = node["node_type"]
        connector_id = node.get("connector_id")
        if node_type == "connector_endpoint" and node.get("endpoint_role") == "from":
            segment_type = "connector_entry"
        elif node_type == "connector_endpoint" and node.get("endpoint_role") == "to":
            segment_type = "connector_exit"
        elif node_type == "stair_connector_node":
            segment_type = "stair_connector"
        else:
            segment_type = "same_floor_room_route"
        waypoint = {
            "waypoint_id": f"wp_{idx:03d}",
            "node_id": node_id,
            "node_type": node_type,
            "floor_id": node.get("floor_id"),
            "room_id": node.get("bound_room_id") if node_type == "connector_endpoint" else (node_id if node_type == "room" else None),
            "connector_id": connector_id,
            "semantic_label": semantic_label(node),
            "segment_type": segment_type,
            "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        }
        if "position_xyz" in node:
            waypoint["position_xyz"] = node["position_xyz"]
        elif "position_xy" in node:
            waypoint["position_xy"] = node["position_xy"]
        waypoints.append(waypoint)
    return {
        "schema_version": "cross_floor_route_waypoints_v0_1",
        "created_utc": now_iso(),
        "scene_id": cross_topology.get("scene_id"),
        "start_room": route_report.get("start_room"),
        "goal_room": route_report.get("goal_room"),
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "waypoint_role": "graph_overlay_visualization_only_not_robot_executable",
        "waypoints": waypoints,
    }


def semantic_label(node: dict[str, Any]) -> str:
    if node["node_type"] == "room":
        return node["node_id"]
    if node["node_type"] == "connector_endpoint":
        return f"{node['connector_id']} {node['endpoint_role']} endpoint"
    if node["node_type"] == "stair_connector_node":
        return f"{node['connector_id']} fitted centerline"
    return node["node_id"]


def display_xy(node: dict[str, Any], floor_offsets: dict[str, float]) -> list[float] | None:
    floor_id = node.get("floor_id")
    offset = floor_offsets.get(floor_id, 0.0)
    if "position_xy" in node and node["position_xy"] is not None:
        return [float(node["position_xy"][0]) + offset, float(node["position_xy"][1])]
    if "position_xyz" in node and node["position_xyz"] is not None:
        return [float(node["position_xyz"][0]) + offset, float(node["position_xyz"][1])]
    return None


def floor_offsets_from_rooms(rooms: list[dict[str, Any]]) -> dict[str, float]:
    floor_1_x: list[float] = []
    floor_2_x: list[float] = []
    for room in rooms:
        xs = [float(point[0]) for point in room.get("polygon", [])]
        if room.get("floor_id") == "floor_1":
            floor_1_x.extend(xs)
        if room.get("floor_id") == "floor_2":
            floor_2_x.extend(xs)
    if not floor_1_x or not floor_2_x:
        return {"floor_1": 0.0, "floor_2": 14.0}
    offset_2 = max(floor_1_x) - min(floor_2_x) + 4.0
    return {"floor_1": 0.0, "floor_2": offset_2}


def generate_visualization(
    path: Path,
    topology: dict[str, Any],
    cross_topology: dict[str, Any],
    route_report: dict[str, Any],
) -> dict[str, Any]:
    rooms = topology.get("rooms", [])
    floor_offsets = floor_offsets_from_rooms(rooms)
    node_by_id = {node["node_id"]: node for node in cross_topology.get("nodes", [])}
    room_by_id = {room["id"]: room for room in rooms}
    fig, ax = plt.subplots(figsize=(14, 8))
    colors = {"floor_1": "#d8efe2", "floor_2": "#e6e1f5"}
    edge_colors = {"floor_1": "#648a73", "floor_2": "#7a6fa8"}
    for room in rooms:
        floor_id = room.get("floor_id")
        offset = floor_offsets.get(floor_id, 0.0)
        polygon = [[float(x) + offset, float(y)] for x, y in room.get("polygon", [])]
        if len(polygon) >= 3:
            ax.add_patch(
                Polygon(
                    polygon,
                    closed=True,
                    facecolor=colors.get(floor_id, "#eeeeee"),
                    edgecolor=edge_colors.get(floor_id, "#777777"),
                    linewidth=1.0,
                    alpha=0.55,
                )
            )
        center = room.get("center")
        if center:
            ax.text(float(center[0]) + offset, float(center[1]), room["id"], ha="center", va="center", fontsize=8)
    for edge in cross_topology.get("edges", []):
        if edge.get("edge_type") != "same_floor_topology":
            continue
        source = room_by_id.get(edge["source"])
        target = room_by_id.get(edge["target"])
        if not source or not target:
            continue
        floor_id = source.get("floor_id")
        offset = floor_offsets.get(floor_id, 0.0)
        sx, sy = source["center"]
        tx, ty = target["center"]
        ax.plot([sx + offset, tx + offset], [sy, ty], color=edge_colors.get(floor_id, "#777777"), linewidth=0.6, alpha=0.25)
    route_nodes = route_report.get("route_nodes", [])
    route_xy = [display_xy(node_by_id[node_id], floor_offsets) for node_id in route_nodes]
    route_xy = [xy for xy in route_xy if xy is not None]
    if route_xy:
        ax.plot([xy[0] for xy in route_xy], [xy[1] for xy in route_xy], color="#d43d51", linewidth=2.8, zorder=5)
        ax.scatter([xy[0] for xy in route_xy], [xy[1] for xy in route_xy], s=42, color="#d43d51", edgecolor="white", linewidth=0.8, zorder=6)
    for node_id in route_nodes:
        node = node_by_id[node_id]
        xy = display_xy(node, floor_offsets)
        if not xy:
            continue
        if node.get("node_type") == "connector_endpoint":
            ax.scatter([xy[0]], [xy[1]], marker="s", s=90, color="#1d6f8a", edgecolor="white", linewidth=1.0, zorder=7)
            ax.text(xy[0], xy[1] + 0.22, node.get("endpoint_role", "endpoint"), fontsize=8, color="#1d4f63", ha="center")
        if node.get("node_type") == "stair_connector_node":
            ax.scatter([xy[0]], [xy[1]], marker="^", s=50, color="#8a5a00", edgecolor="white", linewidth=0.8, zorder=7)
    ax.text(0.01, 0.98, "floor_1", transform=ax.transAxes, ha="left", va="top", fontsize=12, weight="bold")
    ax.text(0.58, 0.98, "floor_2", transform=ax.transAxes, ha="left", va="top", fontsize=12, weight="bold")
    ax.text(
        0.01,
        0.03,
        "claim_boundary = topological_vertical_transition_only\nphysical_execution_supported = false",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=10,
        bbox={"facecolor": "white", "edgecolor": "#999999", "alpha": 0.86},
    )
    ax.set_title("RSLG-SLAM graph-only cross-floor route: room_3 to room_14 via vc_vt_1")
    ax.set_xlabel("x/y floorplan coordinates, floor_2 shifted right for static overlay")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#dddddd", linewidth=0.4, alpha=0.5)
    xs: list[float] = []
    ys: list[float] = []
    for room in rooms:
        offset = floor_offsets.get(room.get("floor_id"), 0.0)
        for x, y in room.get("polygon", []):
            xs.append(float(x) + offset)
            ys.append(float(y))
    if xs and ys:
        ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
        ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return {
        "visualization": path,
        "floor_offsets": floor_offsets,
        "route_node_count": len(route_nodes),
        "equal_axis_scaling": True,
        "raw_trace_rendered_as_official_connector_geometry": False,
    }


def overlay_manifest(visualization_info: dict[str, Any], route_report: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    return {
        "schema_version": "cross_floor_overlay_manifest_v0_1",
        "created_utc": now_iso(),
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "visualization": output_dir.joinpath("cross_floor_route_visualization.png"),
        "route_report": output_dir.joinpath("cross_floor_route_query_report.json"),
        "waypoints": output_dir.joinpath("cross_floor_route_waypoints_v0_1.json"),
        "marker_concepts_for_future_rviz": [
            {"concept": "floor_1 rooms", "source": "topology_v0_1.json rooms where floor_id=floor_1"},
            {"concept": "floor_2 rooms", "source": "topology_v0_1.json rooms where floor_id=floor_2"},
            {"concept": "route nodes", "source": "cross_floor_route_waypoints_v0_1.json"},
            {"concept": "stair connector centerline", "source": "vertical_connectors_v0_1.json sparse_stair_connector_graph.nodes"},
            {"concept": "connector endpoints", "source": "cross_floor_topology_v0_1.json connector_endpoint nodes"},
            {"concept": "labels", "source": "semantic_label fields and room ids"},
        ],
        "rviz_launch_generated": False,
        "robot_execution_generated": False,
        "visualization_info": visualization_info,
        "used_connector_ids": route_report.get("used_connector_ids", []),
    }


def validate_generated(
    output_dir: Path,
    scene_root: Path,
    cross_topology: dict[str, Any],
    route_report: dict[str, Any],
    vertical_connectors: dict[str, Any],
    before_hashes: dict[str, dict[str, str]],
) -> dict[str, Any]:
    json_files = [
        "input_artifact_validation.json",
        "vertical_connectors_v0_1.json",
        "cross_floor_topology_v0_1.json",
        "cross_floor_route_query_report.json",
        "cross_floor_route_waypoints_v0_1.json",
        "cross_floor_overlay_manifest.json",
        "task24c_report.json",
    ]
    json_parse: dict[str, Any] = {}
    for name in json_files:
        path = output_dir / name
        try:
            read_json(path)
            json_parse[name] = {"exists": path.exists(), "parses": True}
        except Exception as exc:  # noqa: BLE001 - validation should report all issues.
            json_parse[name] = {"exists": path.exists(), "parses": False, "error": f"{type(exc).__name__}: {exc}"}
    nodes = cross_topology.get("nodes", [])
    node_ids = [node["node_id"] for node in nodes]
    node_set = set(node_ids)
    bad_edges = [
        edge["edge_id"]
        for edge in cross_topology.get("edges", [])
        if edge["source"] not in node_set or edge["target"] not in node_set
    ]
    edge_ids = {edge["edge_id"] for edge in cross_topology.get("edges", [])}
    missing_route_edges = [
        edge["edge_id"]
        for edge in route_report.get("route_edges", [])
        if edge["edge_id"] not in edge_ids
    ]
    connector = vertical_connectors["connectors"][0]
    stair_node_ids = {node["node_id"] for node in connector["sparse_stair_connector_graph"]["nodes"]}
    binding_targets_exist = (
        connector["endpoint_from"]["node_id"] in stair_node_ids
        and connector["endpoint_to"]["node_id"] in stair_node_ids
    )
    physical_flags_false = all(
        item.get("physical_execution_supported") is PHYSICAL_EXECUTION_SUPPORTED
        for item in [
            cross_topology,
            route_report,
            vertical_connectors["connectors"][0],
            *cross_topology.get("nodes", []),
            *cross_topology.get("edges", []),
        ]
    )
    claim_boundary_ok = all(
        item.get("claim_boundary") == CLAIM_BOUNDARY
        for item in [
            cross_topology,
            route_report,
            vertical_connectors,
            vertical_connectors["connectors"][0],
            *cross_topology.get("nodes", []),
            *cross_topology.get("edges", []),
        ]
    )
    clean = scene_root / "clean_rerun"
    task23b = scene_root / "tasks" / "task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"
    floor_2_after = hash_tree(clean / "maps" / "floor_2")
    task23b_after = hash_tree(task23b)
    validations = {
        "all_generated_json_parses": all(item["parses"] for item in json_parse.values()),
        "cross_floor_graph_node_ids_unique": len(node_ids) == len(node_set),
        "every_edge_source_target_exists": not bad_edges,
        "every_route_edge_exists_in_cross_floor_topology": not missing_route_edges,
        "route_starts_at_room_3_and_ends_at_room_14": route_report.get("route_nodes", [None])[0] == "room_3" and route_report.get("route_nodes", [None])[-1] == "room_14",
        "route_uses_vc_vt_1": "vc_vt_1" in route_report.get("used_connector_ids", []),
        "route_has_floor_transition_count_1": route_report.get("floor_transition_count") == 1,
        "no_direct_hidden_room_3_to_room_7_shortcut_used": not route_report.get("forbidden_shortcut_detected", True),
        "connector_endpoint_bindings_point_to_existing_fitted_centerline_nodes": binding_targets_exist,
        "physical_execution_supported_false_everywhere": physical_flags_false,
        "claim_boundary_topological_vertical_transition_only_everywhere": claim_boundary_ok,
        "floor_1_and_floor_2_stable_maps_exist": (
            (clean / "maps" / "floor_1" / "stage1_floor_1_stable_occupancy_map.yaml").exists()
            and (clean / "maps" / "floor_2" / "stage1_floor_2_stable_occupancy_map.yaml").exists()
        ),
        "gazebo_nav2_habitat_robot_execution_was_not_run": True,
        "clean_rerun_floor_2_map_unchanged_during_script": before_hashes.get("floor_2_map", {}) == floor_2_after,
        "task23b_outputs_unchanged_during_script": before_hashes.get("task23b", {}) == task23b_after,
    }
    return {
        "schema_version": "validation_summary_v0_1",
        "created_utc": now_iso(),
        "claim_boundary": CLAIM_BOUNDARY,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "json_parse": json_parse,
        "details": {
            "bad_cross_floor_edge_ids": bad_edges,
            "missing_route_edge_ids": missing_route_edges,
            "route_nodes": route_report.get("route_nodes", []),
            "route_edges": [edge.get("edge_id") for edge in route_report.get("route_edges", [])],
        },
        "checks": validations,
        "validation_passed": all(validations.values()) and all(item["parses"] for item in json_parse.values()),
    }


def write_md_reports(
    output_dir: Path,
    input_validation: dict[str, Any],
    vertical_connectors: dict[str, Any],
    cross_topology: dict[str, Any],
    route_report: dict[str, Any],
    validation: dict[str, Any],
    task_report: dict[str, Any],
) -> None:
    connector = vertical_connectors["connectors"][0]
    write_text(
        output_dir / "input_artifact_validation.md",
        "\n".join(
            [
                "# Task24c Input Artifact Validation",
                "",
                f"- input_artifacts_valid: {str(input_validation['input_artifacts_valid']).lower()}",
                f"- floor_1 map package passed: {str(input_validation['map_packages']['floor_1']['passed']).lower()}",
                f"- floor_2 map package passed: {str(input_validation['map_packages']['floor_2']['passed']).lower()}",
                f"- sparse endpoint binding check passed: {str(input_validation['graph_checks'].get('sparse_endpoint_bindings_exist', {}).get('passed')).lower()}",
                f"- stairs graph edge endpoint check passed: {str(input_validation['graph_checks'].get('stairs_graph_edge_endpoints_exist', {}).get('passed')).lower()}",
                f"- claim_boundary: {CLAIM_BOUNDARY}",
                f"- physical_execution_supported: false",
                "",
                "The raw transition pose trace is retained only as provenance and is not treated as physical stair geometry.",
            ]
        ),
    )
    write_text(
        output_dir / "vertical_connectors_summary.md",
        "\n".join(
            [
                "# Vertical Connectors Summary",
                "",
                f"- connector_id: {connector['connector_id']}",
                f"- source_transition_id: {connector['source_transition_id']}",
                f"- route binding: {connector['room_from']} -> {connector['endpoint_from']['node_id']} -> {connector['endpoint_to']['node_id']} -> {connector['room_to']}",
                f"- fitted centerline node count: {len(connector['sparse_stair_connector_graph']['nodes'])}",
                f"- fitted centerline edge count: {len(connector['sparse_stair_connector_graph']['edges'])}",
                f"- cost: {connector['cost']}",
                f"- confidence: {connector['confidence']}",
                f"- physical_execution_supported: false",
                f"- claim_boundary: {CLAIM_BOUNDARY}",
            ]
        ),
    )
    write_text(
        output_dir / "cross_floor_topology_summary.md",
        "\n".join(
            [
                "# Cross-Floor Topology Summary",
                "",
                f"- node_count: {len(cross_topology['nodes'])}",
                f"- edge_count: {len(cross_topology['edges'])}",
                f"- transformed_vertical_edges: {len(cross_topology['transformed_vertical_edges'])}",
                "- hidden direct room_3 -> room_7 cross-floor shortcut: not included as a routable edge",
                f"- physical_execution_supported: false",
                f"- claim_boundary: {CLAIM_BOUNDARY}",
            ]
        ),
    )
    write_text(
        output_dir / "cross_floor_route_summary.md",
        "\n".join(
            [
                "# Cross-Floor Route Summary",
                "",
                f"- route_found: {str(route_report.get('route_found')).lower()}",
                f"- start_room: {route_report.get('start_room')}",
                f"- goal_room: {route_report.get('goal_room')}",
                f"- route_nodes: {' -> '.join(route_report.get('route_nodes', []))}",
                f"- used_connector_ids: {', '.join(route_report.get('used_connector_ids', []))}",
                f"- floor_transition_count: {route_report.get('floor_transition_count')}",
                f"- route_cost: {route_report.get('route_cost')}",
                f"- forbidden_shortcut_detected: {str(route_report.get('forbidden_shortcut_detected')).lower()}",
                f"- physical_execution_supported: false",
                f"- claim_boundary: {CLAIM_BOUNDARY}",
            ]
        ),
    )
    write_text(
        output_dir / "validation_summary.md",
        "\n".join(
            [
                "# Task24c Validation Summary",
                "",
                f"- validation_passed: {str(validation['validation_passed']).lower()}",
                *[f"- {key}: {str(value).lower()}" for key, value in validation["checks"].items()],
            ]
        ),
    )
    write_text(
        output_dir / "task24c_report.md",
        "\n".join(
            [
                "# Task24c Report",
                "",
                f"- route_classification: {task_report['route_classification']}",
                f"- connector_classification: {task_report['connector_classification']}",
                f"- map_classification: {task_report['map_classification']}",
                f"- input_artifacts_valid: {str(task_report['input_artifacts_valid']).lower()}",
                f"- route_found: {str(task_report['route_found']).lower()}",
                f"- route_uses_vc_vt_1: {str(task_report['route_uses_vc_vt_1']).lower()}",
                f"- visualization_generated: {str(task_report['visualization_generated']).lower()}",
                f"- physical_execution_supported: false",
                f"- claim_boundary: {CLAIM_BOUNDARY}",
                "",
                "This task generated graph/router/overlay artifacts only. No Gazebo, Nav2, Habitat, quadruped proxy, task23b GUI, or robot execution was run.",
            ]
        ),
    )
    write_text(
        output_dir / "final_answer_for_user.md",
        "\n".join(
            [
                "# Final Answer For User",
                "",
                f"Input artifacts valid: {str(task_report['input_artifacts_valid']).lower()}",
                f"Stable maps present: {task_report['map_classification']}",
                "Generated vertical_connectors_v0_1.json: true",
                "Generated cross_floor_topology_v0_1.json: true",
                f"Route room_3 -> room_14 found: {str(task_report['route_found']).lower()}",
                f"Route uses vc_vt_1: {str(task_report['route_uses_vc_vt_1']).lower()}",
                f"Hidden direct cross-floor shortcuts avoided: {str(task_report['hidden_shortcuts_avoided']).lower()}",
                f"Visualization generated: {str(task_report['visualization_generated']).lower()}",
                "",
                "Exact next recommendation: use these graph-only artifacts as the contract for a future RViz overlay adapter, while keeping robot execution and physical stair traversal out of scope until separate physical evidence and execution support exist.",
            ]
        ),
    )


def classifications(input_validation: dict[str, Any], route_report: dict[str, Any]) -> dict[str, str]:
    floor_1_ok = input_validation["map_packages"]["floor_1"]["passed"]
    floor_2_ok = input_validation["map_packages"]["floor_2"]["passed"]
    if floor_1_ok and floor_2_ok:
        map_classification = "floor_1_and_floor_2_stable_maps_present"
    elif not floor_1_ok:
        map_classification = "missing_floor_1_stable_map"
    else:
        map_classification = "missing_floor_2_stable_map"
    sparse_exists = input_validation["required_inputs"]["sparse_graph"]["exists"]
    graph_ok = input_validation["graph_checks"].get("sparse_endpoint_bindings_exist", {}).get("passed") and input_validation["graph_checks"].get("stairs_graph_edge_endpoints_exist", {}).get("passed")
    if not sparse_exists:
        connector_classification = "blocked_missing_stair_graph"
    elif graph_ok:
        connector_classification = "connector_graph_valid"
    else:
        connector_classification = "connector_graph_invalid"
    if not input_validation["input_artifacts_valid"]:
        route_classification = "blocked_missing_connector_inputs" if not sparse_exists else "blocked_invalid_topology"
    elif route_report.get("route_found"):
        route_classification = "cross_floor_route_found_with_connector"
    else:
        route_classification = "cross_floor_route_not_found"
    return {
        "route_classification": route_classification,
        "connector_classification": connector_classification,
        "map_classification": map_classification,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", required=True, type=Path)
    parser.add_argument("--start-room", default="room_3")
    parser.add_argument("--goal-room", default="room_14")
    parser.add_argument("--connector-id", default="vc_vt_1")
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    scene_root = args.scene_root.resolve()
    output_dir = args.output_dir.resolve()
    clean = scene_root / "clean_rerun"
    task23b = scene_root / "tasks" / "task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"
    before_hashes = {
        "floor_2_map": hash_tree(clean / "maps" / "floor_2"),
        "task23b": hash_tree(task23b),
    }

    paths = collect_paths(scene_root)
    topology = read_json(paths["topology"]) if paths["topology"].exists() else None
    vertical_evidence = read_json(paths["vertical_evidence"]) if paths["vertical_evidence"].exists() else None
    stairs_graph = read_json(paths["stairs_graph"]) if paths["stairs_graph"].exists() else None
    sparse_graph = read_json(paths["sparse_graph"]) if paths["sparse_graph"].exists() else None

    input_validation = validate_inputs(scene_root, paths, topology, stairs_graph, sparse_graph)
    write_json(output_dir / "input_artifact_validation.json", input_validation)
    if not input_validation["input_artifacts_valid"]:
        write_text(output_dir / "input_artifact_validation.md", "# Task24c Input Artifact Validation\n\ninput_artifacts_valid: false")
        raise SystemExit("required task24c input validation failed; see input_artifact_validation.json")
    assert topology is not None and vertical_evidence is not None and sparse_graph is not None

    vertical_connectors = build_vertical_connectors(scene_root, args.connector_id, topology, vertical_evidence, sparse_graph, paths)
    write_json(output_dir / "vertical_connectors_v0_1.json", vertical_connectors)
    cross_topology = build_cross_floor_topology(topology, vertical_connectors)
    write_json(output_dir / "cross_floor_topology_v0_1.json", cross_topology)
    route_report = build_route_report(cross_topology, args.start_room, args.goal_room, args.connector_id)
    write_json(output_dir / "cross_floor_route_query_report.json", route_report)
    waypoints = build_waypoints(cross_topology, route_report) if route_report.get("route_found") else {
        "schema_version": "cross_floor_route_waypoints_v0_1",
        "created_utc": now_iso(),
        "route_found": False,
        "waypoints": [],
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    write_json(output_dir / "cross_floor_route_waypoints_v0_1.json", waypoints)
    visualization_info = generate_visualization(output_dir / "cross_floor_route_visualization.png", topology, cross_topology, route_report)
    manifest = overlay_manifest(visualization_info, route_report, output_dir)
    write_json(output_dir / "cross_floor_overlay_manifest.json", manifest)

    generated_files = [
        "input_artifact_validation.md",
        "input_artifact_validation.json",
        "vertical_connectors_v0_1.json",
        "vertical_connectors_summary.md",
        "cross_floor_topology_v0_1.json",
        "cross_floor_topology_summary.md",
        "cross_floor_route_query_report.json",
        "cross_floor_route_summary.md",
        "cross_floor_route_waypoints_v0_1.json",
        "cross_floor_route_visualization.png",
        "cross_floor_overlay_manifest.json",
        "validation_summary.md",
        "validation_summary.json",
        "task24c_report.md",
        "task24c_report.json",
        "final_answer_for_user.md",
    ]
    classifications_payload = classifications(input_validation, route_report)
    task_report = {
        "schema_version": "task24c_report_v0_1",
        "created_utc": now_iso(),
        "task": TASK_NAME,
        "scene_id": topology.get("sequence_id", SCENE_ID_DEFAULT),
        "input_artifacts_valid": input_validation["input_artifacts_valid"],
        "floor_1_stable_map_present": input_validation["map_packages"]["floor_1"]["passed"],
        "floor_2_stable_map_present": input_validation["map_packages"]["floor_2"]["passed"],
        "vertical_connectors_generated": True,
        "cross_floor_topology_generated": True,
        "route_found": route_report.get("route_found", False),
        "route_uses_vc_vt_1": args.connector_id in route_report.get("used_connector_ids", []),
        "hidden_shortcuts_avoided": not route_report.get("forbidden_shortcut_detected", True),
        "visualization_generated": (output_dir / "cross_floor_route_visualization.png").exists(),
        "generated_files": generated_files,
        "validation_passed": None,
        "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        "claim_boundary": CLAIM_BOUNDARY,
        "no_robot_execution_run": True,
        "next_recommendation": "Use these graph-only artifacts as the contract for a future RViz overlay adapter; keep robot execution and physical stair traversal out of scope until separate physical evidence and execution support exist.",
        **classifications_payload,
    }
    write_json(output_dir / "task24c_report.json", task_report)
    write_json(
        output_dir / "validation_summary.json",
        {
            "schema_version": "validation_summary_v0_1",
            "created_utc": now_iso(),
            "preliminary": True,
            "claim_boundary": CLAIM_BOUNDARY,
            "physical_execution_supported": PHYSICAL_EXECUTION_SUPPORTED,
        },
    )
    validation = validate_generated(output_dir, scene_root, cross_topology, route_report, vertical_connectors, before_hashes)
    task_report["validation_passed"] = validation["validation_passed"]
    write_json(output_dir / "task24c_report.json", task_report)
    write_json(output_dir / "validation_summary.json", validation)
    write_md_reports(output_dir, input_validation, vertical_connectors, cross_topology, route_report, validation, task_report)
    py_compile.compile(Path(__file__).as_posix(), doraise=True)
    print(json.dumps({"output_dir": output_dir.as_posix(), "route_found": route_report.get("route_found"), **classifications_payload}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
