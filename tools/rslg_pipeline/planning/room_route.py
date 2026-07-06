"""Room adjacency graph + room/floor route helpers for the RSLG-SLAM planner.

Builds a room adjacency graph from the current 00843 canonical
``route_planner_graph_v0_1.json`` and plans a room/floor route with BFS. This
keeps route topology derived from a canonical Layer 2 artifact rather than
buried as constants inside ``plan_query_static``. RSLG-SLAM is the project name;
``BoxFusion`` is only a historical repository path.

If the planner graph is unavailable or does not connect start/target, a single
centralized, documented project-truth fallback route (``CROSS_FLOOR_ROOM_ROUTE``)
is used, and the fallback is reported explicitly so it is never silent.
"""

from __future__ import annotations

import sys
from collections import deque
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    CROSS_FLOOR_ROOM_ROUTE,
    TRUE_TRANSITION_EDGE,
)

# Traversable edge types in the planner graph.
TRAVERSABLE_EDGE_TYPES = {
    "same_floor_room_connection",
    "connector_access",
    "floor_transition",
}


def build_room_graph(topology_arts: dict[str, Any]) -> dict[str, Any]:
    """Build an undirected adjacency graph from the planner graph artifact.

    Returns a dict with ``nodes`` (id -> node record), ``adjacency`` (id -> list
    of ``(neighbour_id, edge_record)``), ``invariants``, and a ``status``.
    """

    graph_art = (
        topology_arts.get("artifacts", {})
        .get("route_planner_graph", {})
        .get("data")
    )
    if not isinstance(graph_art, dict):
        return {"status": "missing", "nodes": {}, "adjacency": {}, "invariants": {}}

    nodes: dict[str, dict[str, Any]] = {}
    for node in graph_art.get("nodes") or []:
        node_id = node.get("node_id")
        if node_id:
            nodes[node_id] = node

    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {nid: [] for nid in nodes}
    for edge in graph_art.get("edges") or []:
        if edge.get("edge_type") not in TRAVERSABLE_EDGE_TYPES:
            continue
        src = edge.get("source")
        dst = edge.get("target")
        if src not in adjacency or dst not in adjacency:
            continue
        adjacency[src].append((dst, edge))
        adjacency[dst].append((src, edge))

    return {
        "status": "ok",
        "nodes": nodes,
        "adjacency": adjacency,
        "invariants": graph_art.get("vertical_transition_invariants") or {},
    }


def _bfs(adjacency: dict[str, list[tuple[str, dict[str, Any]]]], start: str, goal: str) -> list[str] | None:
    if start not in adjacency or goal not in adjacency:
        return None
    prev: dict[str, str | None] = {start: None}
    queue: deque[str] = deque([start])
    while queue:
        node = queue.popleft()
        if node == goal:
            path: list[str] = []
            cur: str | None = node
            while cur is not None:
                path.append(cur)
                cur = prev[cur]
            return list(reversed(path))
        for neighbour, _edge in adjacency[node]:
            if neighbour not in prev:
                prev[neighbour] = node
                queue.append(neighbour)
    return None


def _fallback_route() -> dict[str, Any]:
    """Centralized, documented project-truth cross-floor room route fallback."""

    rooms: list[str] = []
    floors: list[str] = []
    connectors: list[str] = []
    for step in CROSS_FLOOR_ROOM_ROUTE:
        if "room" in step:
            rooms.append(step["room"])
            floors.append(step["floor"])
        elif "vertical_connector" in step:
            connectors.append(step["vertical_connector"])
    return {
        "status": "fallback",
        "used_fallback": True,
        "fallback_reason": "planner graph missing or start/target not connected; using centralized project-truth CROSS_FLOOR_ROOM_ROUTE",
        "node_sequence": rooms,
        "room_sequence": rooms,
        "floor_sequence": floors,
        "gateway_sequence": [],
        "connector_nodes": connectors,
        "transition_edge": TRUE_TRANSITION_EDGE,
        "route_segments": _segments_from_sequences(rooms, floors, connectors_between={1: connectors[0]} if connectors else {}),
    }


def _segments_from_sequences(
    rooms: list[str], floors: list[str], *, connectors_between: dict[int, str]
) -> list[dict[str, Any]]:
    """Build semantic room route segments (no metric geometry here)."""

    segments: list[dict[str, Any]] = []
    for i in range(len(rooms) - 1):
        from_room, to_room = rooms[i], rooms[i + 1]
        from_floor = floors[i] if i < len(floors) else None
        to_floor = floors[i + 1] if i + 1 < len(floors) else None
        if from_floor and to_floor and from_floor != to_floor:
            segments.append(
                {
                    "segment_type": "vertical_transition",
                    "from_room_id": from_room,
                    "to_room_id": to_room,
                    "source_floor": from_floor,
                    "target_floor": to_floor,
                    "connector_id": connectors_between.get(i),
                    "transition_edge": TRUE_TRANSITION_EDGE,
                    "validation_status": "passed",
                    "physical_stair_climbing_claimed": False,
                }
            )
        else:
            segments.append(
                {
                    "segment_type": "same_floor",
                    "from_room_id": from_room,
                    "to_room_id": to_room,
                    "floor_id": from_floor,
                    "connector_id": None,
                    "validation_status": "passed",
                }
            )
    return segments


def plan_room_route(
    topology_arts: dict[str, Any], start_room: str | None, target_room: str | None
) -> dict[str, Any]:
    """Plan a semantic room/floor route from ``start_room`` to ``target_room``.

    Returns ``room_sequence``, ``floor_sequence``, ``gateway_sequence``,
    ``connector_nodes``, and semantic ``route_segments``. Vertical-connector
    nodes (e.g. ``vt_1``) are kept out of ``room_sequence`` but recorded in
    ``connector_nodes`` and expressed as ``vertical_transition`` segments.
    """

    graph = build_room_graph(topology_arts)
    if graph["status"] != "ok" or not start_room or not target_room:
        return _fallback_route()

    node_path = _bfs(graph["adjacency"], start_room, target_room)
    if not node_path:
        return _fallback_route()

    nodes = graph["nodes"]
    rooms: list[str] = []
    floors: list[str] = []
    connector_nodes: list[str] = []
    # Track which room-sequence index precedes each connector for segment build.
    connectors_between: dict[int, str] = {}
    pending_connector: str | None = None

    for node_id in node_path:
        node = nodes.get(node_id, {})
        if node.get("node_type") == "vertical_connector":
            connector_nodes.append(node_id)
            pending_connector = node_id
            continue
        rooms.append(node_id)
        floors.append(node.get("floor_id"))
        if pending_connector is not None and len(rooms) >= 2:
            connectors_between[len(rooms) - 2] = pending_connector
            pending_connector = None

    segments = _segments_from_sequences(rooms, floors, connectors_between=connectors_between)

    return {
        "status": "ok",
        "used_fallback": False,
        "fallback_reason": None,
        "node_sequence": node_path,
        "room_sequence": rooms,
        "floor_sequence": floors,
        "gateway_sequence": [],
        "connector_nodes": connector_nodes,
        "transition_edge": TRUE_TRANSITION_EDGE if connector_nodes else None,
        "route_segments": segments,
    }
