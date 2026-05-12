"""Route-to-waypoint adapter for committed/public BoxFusion artifacts.

The output is JSON-compatible and deliberately conservative: every waypoint is
marked as topological/demo, not collision-free, and not a robot-control command.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from boxfusion.ros_artifact_bridge import (
    DEFAULT_FRAME_ID,
    PAPER_SAFETY_LABEL,
    SNAPSHOT_FILENAME,
    TOPOLOGY_FILENAME,
    CommittedArtifactBundle,
    _as_float,
    _point_dict,
    _xy,
    _xyz,
    normalize_object_id,
    normalize_room_id,
)


ROOM_CENTER_LABEL = "Topological room-center demo waypoint; not a collision-free robot pose."
GATEWAY_LABEL = "Gateway-backed demo waypoint from committed snapshot evidence; not a clearance guarantee."
VERTICAL_LABEL = "Cross-floor topology marker only; not a robot traversal command."
SEMANTIC_TARGET_LABEL = "Semantic target marker from committed snapshot; not a collision-free robot pose."
ROUTE_LABEL = "Committed room-level route visualized as topological/demo waypoints."


def route_to_waypoints(
    route: Mapping[str, Any],
    artifacts: CommittedArtifactBundle | Mapping[str, Any],
    *,
    semantic_target: Optional[Mapping[str, Any]] = None,
    frame_id: str = DEFAULT_FRAME_ID,
) -> Dict[str, Any]:
    """Convert a room-level route into topological/demo waypoint JSON."""

    bundle = _coerce_bundle(artifacts)
    route_payload = _extract_route_payload(route)
    warnings: List[str] = []
    room_sequence = [
        room_id
        for room_id in (normalize_room_id(item) for item in route_payload.get("room_sequence") or [])
        if room_id is not None
    ]
    room_index = bundle.room_index()
    if not room_sequence:
        warnings.append("Route payload does not include a non-empty room_sequence.")
        return _result(frame_id=frame_id, waypoints=[], segments=[], warnings=warnings)

    for room_id in room_sequence:
        if room_id not in room_index:
            warnings.append(f"Route room {room_id} is not present in topology_v0_1.json.")

    waypoints: List[Dict[str, Any]] = []
    segments: List[Dict[str, Any]] = []

    start_room_id = room_sequence[0]
    start_waypoint = _room_center_waypoint(
        bundle,
        room_id=start_room_id,
        index=len(waypoints),
        kind="topological_demo_waypoint",
        warnings=warnings,
    )
    if start_waypoint is not None:
        waypoints.append(start_waypoint)

    route_edges = _route_edges(route_payload, room_sequence, bundle)
    for edge_index, edge in enumerate(route_edges):
        source_room_id = edge["source_room_id"]
        target_room_id = edge["target_room_id"]
        relation_type = str(edge.get("relation_type") or "unknown")
        before_count = len(waypoints)
        if relation_type == "vertical_transition":
            vertical_waypoints = _vertical_transition_waypoints(
                bundle,
                edge=edge,
                next_index=len(waypoints),
                warnings=warnings,
            )
            waypoints.extend(vertical_waypoints)
            if not vertical_waypoints:
                fallback = _room_center_waypoint(
                    bundle,
                    room_id=target_room_id,
                    index=len(waypoints),
                    kind="room_center_demo_waypoint",
                    edge=edge,
                    warnings=warnings,
                )
                if fallback is not None:
                    waypoints.append(fallback)
        else:
            gateway = _matching_gateway(bundle, source_room_id, target_room_id)
            if gateway is not None:
                waypoint = _gateway_waypoint(
                    bundle,
                    normalized_gateway=gateway,
                    edge=edge,
                    index=len(waypoints),
                    warnings=warnings,
                )
                if waypoint is not None:
                    waypoints.append(waypoint)
            else:
                warnings.append(
                    f"No matching snapshot gateway for {source_room_id}->{target_room_id}; "
                    "falling back to the next topology room center."
                )
                fallback = _room_center_waypoint(
                    bundle,
                    room_id=target_room_id,
                    index=len(waypoints),
                    kind="room_center_demo_waypoint",
                    edge=edge,
                    warnings=warnings,
                )
                if fallback is not None:
                    waypoints.append(fallback)

        new_waypoints = waypoints[before_count:]
        if new_waypoints:
            _append_segments(
                segments,
                waypoints=waypoints,
                new_waypoints=new_waypoints,
                relation_type=relation_type,
                edge_index=edge_index,
                vertical_only=relation_type == "vertical_transition",
            )

    target_waypoint = _semantic_target_waypoint(
        bundle,
        semantic_target=semantic_target,
        index=len(waypoints),
        warnings=warnings,
    )
    if target_waypoint is None:
        goal_room_id = room_sequence[-1]
        last_is_goal_room_center = (
            bool(waypoints)
            and waypoints[-1].get("room_id") == goal_room_id
            and (waypoints[-1].get("source") or {}).get("field") == "rooms[].center"
        )
        if not last_is_goal_room_center:
            target_waypoint = _room_center_waypoint(
                bundle,
                room_id=goal_room_id,
                index=len(waypoints),
                kind="topological_demo_waypoint",
                warnings=warnings,
            )
    if target_waypoint is not None:
        if not waypoints or waypoints[-1].get("waypoint_id") != target_waypoint.get("waypoint_id"):
            waypoints.append(target_waypoint)
            if len(waypoints) >= 2:
                segments.append(
                    _segment(
                        waypoints[-2]["waypoint_id"],
                        waypoints[-1]["waypoint_id"],
                        relation_type="route_endpoint",
                        visualization_only=True,
                        reason="Goal room center or semantic target marker appended as route endpoint.",
                    )
                )

    _reindex_waypoints(waypoints)
    return _result(frame_id=frame_id, waypoints=waypoints, segments=segments, warnings=warnings)


def load_route_json(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected route JSON object in {path}")
    return payload


def _coerce_bundle(artifacts: CommittedArtifactBundle | Mapping[str, Any]) -> CommittedArtifactBundle:
    if isinstance(artifacts, CommittedArtifactBundle):
        return artifacts
    if isinstance(artifacts, Mapping):
        scene_root = artifacts.get("scene_root")
        if scene_root is not None and "topology" not in artifacts:
            return CommittedArtifactBundle.from_scene_root(Path(scene_root))
        topology = artifacts.get("topology")
        snapshot = artifacts.get("snapshot")
        if isinstance(topology, Mapping) and isinstance(snapshot, Mapping):
            root = Path(scene_root or ".")
            return CommittedArtifactBundle(
                scene_root=root,
                topology_path=Path(artifacts.get("topology_path") or "logs/topology_v0_1.json"),
                query_report_path=Path(artifacts.get("query_report_path") or "logs/topology_query_report.json"),
                world_model_path=Path(artifacts.get("world_model_path") or "logs/committed_room_world_model_v0_1.json"),
                snapshot_path=Path(artifacts.get("snapshot_path") or "logs/committed_room_world_snapshot_v0_1.json"),
                topology=dict(topology),
                query_report=dict(artifacts.get("query_report") or {}),
                world_model=dict(artifacts.get("world_model") or {}),
                snapshot=dict(snapshot),
            )
    raise TypeError(
        "route_to_waypoints requires a CommittedArtifactBundle, a mapping with scene_root, "
        "or a mapping with loaded topology and snapshot dictionaries."
    )


def _extract_route_payload(route: Mapping[str, Any]) -> Dict[str, Any]:
    if isinstance(route.get("route"), Mapping):
        nested = dict(route["route"])
        for key in ("target_resolution", "resolved_goal_room_id", "query_type"):
            if key in route and key not in nested:
                nested[key] = route[key]
        return nested
    return dict(route)


def _route_edges(
    route_payload: Mapping[str, Any],
    room_sequence: Sequence[str],
    bundle: CommittedArtifactBundle,
) -> List[Dict[str, Any]]:
    raw_edges = [dict(item) for item in route_payload.get("edges") or [] if isinstance(item, Mapping)]
    if raw_edges:
        edges: List[Dict[str, Any]] = []
        for idx, raw_edge in enumerate(raw_edges):
            source_room_id = normalize_room_id(raw_edge.get("source_room_id") or raw_edge.get("source"))
            target_room_id = normalize_room_id(raw_edge.get("target_room_id") or raw_edge.get("target"))
            if source_room_id is None or target_room_id is None:
                continue
            edge = dict(raw_edge)
            edge.update(
                {
                    "edge_index": idx,
                    "source_room_id": source_room_id,
                    "target_room_id": target_room_id,
                    "relation_type": raw_edge.get("relation_type") or "unknown",
                    "confidence": raw_edge.get("confidence"),
                    "support_count": raw_edge.get("support_count"),
                }
            )
            edges.append(edge)
        return edges

    relation_types = list(route_payload.get("used_relation_types") or [])
    confidences = list(route_payload.get("edge_confidences") or [])
    topology_edge_by_pair = _topology_edge_by_pair(bundle)
    edges = []
    for idx, (source_room_id, target_room_id) in enumerate(zip(room_sequence, room_sequence[1:])):
        topology_edge = topology_edge_by_pair.get(frozenset((source_room_id, target_room_id)), {})
        edges.append(
            {
                "edge_index": idx,
                "source_room_id": source_room_id,
                "target_room_id": target_room_id,
                "relation_type": relation_types[idx] if idx < len(relation_types) else topology_edge.get("relation_type", "unknown"),
                "confidence": confidences[idx] if idx < len(confidences) else topology_edge.get("confidence"),
                "support_count": topology_edge.get("support_count"),
                "source_artifact": TOPOLOGY_FILENAME,
            }
        )
    return edges


def _topology_edge_by_pair(bundle: CommittedArtifactBundle) -> Dict[frozenset[str], Dict[str, Any]]:
    index: Dict[frozenset[str], Dict[str, Any]] = {}
    for edge in bundle.topology_edges:
        source_room_id = normalize_room_id(edge.get("source"))
        target_room_id = normalize_room_id(edge.get("target"))
        if source_room_id is not None and target_room_id is not None:
            index[frozenset((source_room_id, target_room_id))] = edge
    return index


def _room_floor(bundle: CommittedArtifactBundle, room_id: str) -> Dict[str, Any]:
    room = bundle.room_index().get(room_id) or {}
    return {
        "floor_id": room.get("floor_id"),
        "display_floor_id": room.get("display_floor_id"),
        "display_order": room.get("display_order"),
    }


def _room_center_waypoint(
    bundle: CommittedArtifactBundle,
    *,
    room_id: str,
    index: int,
    kind: str,
    warnings: List[str],
    edge: Optional[Mapping[str, Any]] = None,
) -> Optional[Dict[str, Any]]:
    room = bundle.room_index().get(room_id)
    if room is None:
        warnings.append(f"Cannot create room-center waypoint for missing topology room {room_id}.")
        return None
    center = _xy(room.get("center"))
    if center is None:
        warnings.append(f"Cannot create room-center waypoint for {room_id}: missing topology center.")
        return None
    z = bundle.floor_display_z(room.get("floor_id"))
    waypoint = {
        "index": index,
        "waypoint_id": f"wp_{index:03d}_{kind}_{room_id}",
        "kind": kind,
        "position": _point_dict(center[0], center[1], z),
        "source": {
            "artifact": TOPOLOGY_FILENAME,
            "field": "rooms[].center",
            "source_ids": {"room_id": room_id},
        },
        "room_id": room_id,
        "edge": _edge_payload(edge),
        "floor": _room_floor(bundle, room_id),
        "followable": True,
        "visualization_only": True,
        "not_collision_free": True,
        "paper_safety_label": ROOM_CENTER_LABEL,
    }
    return waypoint


def _matching_gateway(bundle: CommittedArtifactBundle, source_room_id: str, target_room_id: str) -> Optional[Dict[str, Any]]:
    pair = frozenset((source_room_id, target_room_id))
    candidates = [
        row
        for row in bundle.id_normalizer.normalized_gateways
        if row.get("room_a") is not None
        and row.get("room_b") is not None
        and frozenset((row["room_a"], row["room_b"])) == pair
        and _xy((row.get("record") or {}).get("pos_world")) is not None
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda row: int(row.get("index") or 0))
    return candidates[0]


def _gateway_waypoint(
    bundle: CommittedArtifactBundle,
    *,
    normalized_gateway: Mapping[str, Any],
    edge: Mapping[str, Any],
    index: int,
    warnings: List[str],
) -> Optional[Dict[str, Any]]:
    gateway = dict(normalized_gateway.get("record") or {})
    point = _xy(gateway.get("pos_world"))
    if point is None:
        warnings.append(f"Matched gateway {normalized_gateway.get('index')} has no usable pos_world.")
        return None
    z = bundle.floor_display_z(gateway.get("floor_id"))
    return {
        "index": index,
        "waypoint_id": f"wp_{index:03d}_gateway_{normalized_gateway.get('index')}",
        "kind": "gateway_demo_waypoint",
        "position": _point_dict(point[0], point[1], z),
        "source": {
            "artifact": SNAPSHOT_FILENAME,
            "field": "gateways[].pos_world",
            "source_ids": {
                "gateway_index": normalized_gateway.get("index"),
                "raw_connects": list(normalized_gateway.get("raw_connects") or []),
            },
        },
        "room_id": edge.get("target_room_id"),
        "edge": _edge_payload(edge),
        "floor": {
            "floor_id": gateway.get("floor_id"),
            "display_floor_id": gateway.get("display_floor_id"),
            "display_order": gateway.get("display_order"),
        },
        "followable": True,
        "visualization_only": True,
        "not_collision_free": True,
        "paper_safety_label": GATEWAY_LABEL,
    }


def _vertical_transition_waypoints(
    bundle: CommittedArtifactBundle,
    *,
    edge: Mapping[str, Any],
    next_index: int,
    warnings: List[str],
) -> List[Dict[str, Any]]:
    source_room_id = edge["source_room_id"]
    target_room_id = edge["target_room_id"]
    pair = frozenset((source_room_id, target_room_id))
    matches = [
        row
        for row in bundle.id_normalizer.normalized_vertical_transitions
        if row.get("room_a") is not None
        and row.get("room_b") is not None
        and frozenset((row["room_a"], row["room_b"])) == pair
    ]
    if not matches:
        warnings.append(f"No matching vertical transition endpoint record for {source_room_id}->{target_room_id}.")
        return []
    row = sorted(matches, key=lambda item: int(item.get("index") or 0))[0]
    transition = dict(row.get("record") or {})
    transition_id = row.get("transition_id") or f"vertical_transition_{row.get('index')}"
    endpoints = [
        ("from", transition.get("from_position_xy"), source_room_id, "from_floor_id", "from_display_floor_id", "from_display_order"),
        ("to", transition.get("to_position_xy"), target_room_id, "to_floor_id", "to_display_floor_id", "to_display_order"),
    ]
    waypoints: List[Dict[str, Any]] = []
    for endpoint_name, raw_position, room_id, floor_key, display_floor_key, display_order_key in endpoints:
        point = _xy(raw_position)
        if point is None:
            warnings.append(f"Vertical transition {transition_id} missing {endpoint_name}_position_xy.")
            continue
        z = bundle.floor_display_z(transition.get(floor_key))
        waypoint_index = next_index + len(waypoints)
        waypoints.append(
            {
                "index": waypoint_index,
                "waypoint_id": f"wp_{waypoint_index:03d}_{transition_id}_{endpoint_name}",
                "kind": "vertical_transition_marker_only",
                "position": _point_dict(point[0], point[1], z),
                "source": {
                    "artifact": SNAPSHOT_FILENAME,
                    "field": f"vertical_transitions[].{endpoint_name}_position_xy",
                    "source_ids": {"transition_id": transition_id, "endpoint": endpoint_name},
                },
                "room_id": room_id,
                "edge": _edge_payload(edge),
                "floor": {
                    "floor_id": transition.get(floor_key),
                    "display_floor_id": transition.get(display_floor_key),
                    "display_order": transition.get(display_order_key),
                },
                "followable": False,
                "visualization_only": True,
                "not_collision_free": True,
                "paper_safety_label": VERTICAL_LABEL,
            }
        )
    return waypoints


def _semantic_target_waypoint(
    bundle: CommittedArtifactBundle,
    *,
    semantic_target: Optional[Mapping[str, Any]],
    index: int,
    warnings: List[str],
) -> Optional[Dict[str, Any]]:
    if not semantic_target:
        return None
    record = dict(semantic_target)
    target_type = str(record.get("target_type") or record.get("type") or "").strip().lower()
    if not target_type:
        if "anchor_type" in record or str(record.get("id") or "").startswith("anchor_"):
            target_type = "anchor"
        else:
            target_type = "object"
    if target_type == "anchor":
        point = _xyz(record.get("position"), default_z=bundle.floor_display_z(record.get("floor_id")))
        field = "anchors[].position"
        target_id = str(record.get("id") or f"semantic_target_{index}")
    else:
        point = _xyz(record.get("pose_3d"), default_z=bundle.floor_display_z(record.get("floor_id"))) or _xyz(
            record.get("pose"), default_z=bundle.floor_display_z(record.get("floor_id"))
        )
        field = "objects[].pose_3d|pose"
        target_id = normalize_object_id(record.get("id")) or str(record.get("id") or f"semantic_target_{index}")
    if point is None:
        warnings.append("Semantic target was provided but has no usable committed snapshot position.")
        return None
    return {
        "index": index,
        "waypoint_id": f"wp_{index:03d}_semantic_target_{target_id}",
        "kind": "semantic_target_marker",
        "position": _point_dict(point[0], point[1], point[2]),
        "source": {
            "artifact": SNAPSHOT_FILENAME,
            "field": field,
            "source_ids": {"target_id": target_id, "target_type": target_type},
        },
        "room_id": normalize_room_id(record.get("room_id")),
        "edge": {},
        "floor": {
            "floor_id": record.get("floor_id"),
            "display_floor_id": record.get("display_floor_id"),
            "display_order": record.get("display_order"),
        },
        "followable": False,
        "visualization_only": True,
        "not_collision_free": True,
        "paper_safety_label": SEMANTIC_TARGET_LABEL,
    }


def _edge_payload(edge: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    if not edge:
        return {}
    return {
        "source_room_id": edge.get("source_room_id"),
        "target_room_id": edge.get("target_room_id"),
        "relation_type": edge.get("relation_type"),
        "confidence": edge.get("confidence"),
        "support_count": edge.get("support_count"),
        "edge_index": edge.get("edge_index"),
    }


def _append_segments(
    segments: List[Dict[str, Any]],
    *,
    waypoints: Sequence[Dict[str, Any]],
    new_waypoints: Sequence[Dict[str, Any]],
    relation_type: str,
    edge_index: int,
    vertical_only: bool,
) -> None:
    if len(waypoints) < 2:
        return
    start_idx = max(1, len(waypoints) - len(new_waypoints))
    for idx in range(start_idx, len(waypoints)):
        segments.append(
            _segment(
                waypoints[idx - 1]["waypoint_id"],
                waypoints[idx]["waypoint_id"],
                relation_type=relation_type,
                visualization_only=True,
                reason=(
                    "Vertical transition endpoint marker only; not followable."
                    if vertical_only
                    else f"Topological/demo segment for route edge {edge_index}; not collision-free."
                ),
            )
        )


def _segment(
    source_waypoint_id: str,
    target_waypoint_id: str,
    *,
    relation_type: str,
    visualization_only: bool,
    reason: str,
) -> Dict[str, Any]:
    return {
        "source_waypoint_id": source_waypoint_id,
        "target_waypoint_id": target_waypoint_id,
        "relation_type": relation_type,
        "visualization_only": bool(visualization_only),
        "followable": False if relation_type == "vertical_transition" else True,
        "not_collision_free": True,
        "reason": reason,
        "paper_safety_label": ROUTE_LABEL,
    }


def _reindex_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    for index, waypoint in enumerate(waypoints):
        waypoint["index"] = index


def _result(
    *,
    frame_id: str,
    waypoints: List[Dict[str, Any]],
    segments: List[Dict[str, Any]],
    warnings: List[str],
) -> Dict[str, Any]:
    return {
        "frame_id": frame_id,
        "waypoints": waypoints,
        "segments": segments,
        "warnings": warnings,
        "paper_safety_summary": {
            "label": ROUTE_LABEL,
            "not_collision_free": True,
            "visualization_only": True,
            "forbidden_semantics": [
                "collision-free path",
                "local plan",
                "costmap path",
                "Nav2 path",
                "robot control command",
                "obstacle avoidance",
                "full navigation",
                "BEV planning",
            ],
            "summary": PAPER_SAFETY_LABEL,
        },
    }
