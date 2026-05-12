#!/usr/bin/env python3
"""Step30C gateway-augmented topology candidate for scene 00824-Dd4bFSTQ8gi.

This script consumes the Step30B2 truth-blind automatic gateway route
candidates and converts them into a room-gateway-room topology candidate.

It intentionally does not rerun Stage-A, gateway extraction, ROS, Nav2,
Gazebo, AMCL, DWB, RViz, robot navigation, or Nav2 route generation.
"""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30c"
VERSION = "v0_1"

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
STEP30B2_ROOT = RUNTIME_ROOT / "step30b2_00824_candidate_level_gateway_truth_and_auto_selection"
STEP30A_LOG_ROOT = (
    RUNTIME_ROOT
    / "step30a_00824_full_stage_a_dual_wall_gateway_rerun"
    / "scenes"
    / SCENE_ID
    / "logs"
)
OUTPUT_ROOT = RUNTIME_ROOT / "step30c_00824_gateway_augmented_topology_candidate"

PRIMARY_ROUTE_CANDIDATES = (
    STEP30B2_ROOT / f"{SHORT}_step30b2_corrected_auto_route_candidates_for_step30c_{VERSION}.json"
)
AUTO_SELECTION = STEP30B2_ROOT / f"{SHORT}_step30b2_auto_gateway_selection_{VERSION}.json"
AUTO_VS_TRUTH_EVALUATION = STEP30B2_ROOT / f"{SHORT}_step30b2_auto_vs_truth_evaluation_{VERSION}.json"
STEP30B2_SUMMARY = STEP30B2_ROOT / f"{SHORT}_step30b2_summary_{VERSION}.json"

OPTIONAL_STAGE_A_ARTIFACTS = {
    "topology_v0_1": STEP30A_LOG_ROOT / "topology_v0_1.json",
    "topology_query_report": STEP30A_LOG_ROOT / "topology_query_report.json",
    "committed_room_world_model": STEP30A_LOG_ROOT / "committed_room_world_model_v0_1.json",
    "committed_room_world_snapshot": STEP30A_LOG_ROOT / "committed_room_world_snapshot_v0_1.json",
}

EXPECTED_POSITIVE_PAIRS = [
    "r1_r3",
    "r3_r7",
    "r7_r11",
    "r7_r14",
    "r14_r16",
    "r7_r15",
    "r8_r11",
    "r3_r8",
]
EXPECTED_PUBLIC_PATH_042_GATEWAYS = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r7_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r8_r11_01",
]
NEGATIVE_SANITY_PAIRS = ["r3_r11"]
NON_TRUTH_PAIRS = ["r14_r15", "r15_r16", "r3_r15", "r7_r16"]


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")
    print(f"Written: {rel(path)}")


def room_number(room_id: str | int) -> int:
    text = str(room_id)
    if text.startswith("room_"):
        text = text.split("_", 1)[1]
    if text.startswith("r"):
        text = text[1:]
    return int(text)


def room_node_id(room_id: int) -> str:
    return f"room_{room_id}"


def pair_key_for_rooms(room_a: int, room_b: int) -> str:
    low, high = sorted((int(room_a), int(room_b)))
    return f"r{low}_r{high}"


def parse_pair_key(pair_key: str) -> Tuple[int, int]:
    left, right = pair_key.split("_", 1)
    return room_number(left), room_number(right)


def pose_xy(pose: Dict[str, Any]) -> List[float]:
    return [float(pose["x"]), float(pose["y"])]


def load_optional_room_metadata() -> Tuple[Dict[int, Dict[str, Any]], Dict[str, str]]:
    room_metadata: Dict[int, Dict[str, Any]] = {}
    loaded_artifacts: Dict[str, str] = {}
    model_path = OPTIONAL_STAGE_A_ARTIFACTS["committed_room_world_model"]
    if not model_path.exists():
        return room_metadata, loaded_artifacts

    model = read_json(model_path)
    loaded_artifacts["committed_room_world_model"] = rel(model_path)
    for room in model.get("rooms", []):
        room_id = room_number(room["room_id"])
        room_metadata[room_id] = {
            "centroid_xy": room.get("centroid_xy"),
            "extent_bbox_xy": room.get("extent_bbox_xy"),
            "footprint_area_m2": room.get("footprint_area_m2"),
            "room_type": room.get("room_type"),
            "source_artifact": rel(model_path),
        }

    for key, path in OPTIONAL_STAGE_A_ARTIFACTS.items():
        if key == "committed_room_world_model":
            continue
        if path.exists():
            loaded_artifacts[key] = rel(path)
    return room_metadata, loaded_artifacts


def load_step30b2_route_candidates() -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, str]]:
    primary_payload = read_json(PRIMARY_ROUTE_CANDIDATES)
    validation_payloads = {
        "auto_gateway_selection": read_json(AUTO_SELECTION),
        "auto_vs_truth_evaluation": read_json(AUTO_VS_TRUTH_EVALUATION),
        "step30b2_summary": read_json(STEP30B2_SUMMARY),
    }
    source_artifacts = {
        "primary_step30b2_route_candidates": rel(PRIMARY_ROUTE_CANDIDATES),
        "auto_gateway_selection": rel(AUTO_SELECTION),
        "auto_vs_truth_evaluation": rel(AUTO_VS_TRUTH_EVALUATION),
        "step30b2_summary": rel(STEP30B2_SUMMARY),
    }
    return list(primary_payload.get("route_candidates", [])), validation_payloads, source_artifacts


def nav2_default_usable_for(candidate: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
    passability_status = candidate.get("passability_status")
    warnings = set(candidate.get("warnings") or [])
    if passability_status == "strong" and not any("passability" in warning for warning in warnings):
        return True, None
    if passability_status == "low_or_uncertain":
        return False, "low_or_uncertain_passability"
    if passability_status == "reviewable":
        return False, "reviewable_passability"
    return False, f"passability_status_{passability_status or 'missing'}"


def build_room_nodes(
    route_candidates: Sequence[Dict[str, Any]], room_metadata: Dict[int, Dict[str, Any]]
) -> List[Dict[str, Any]]:
    room_ids = sorted({room_id for candidate in route_candidates for room_id in parse_pair_key(candidate["pair_key"])})
    nodes = []
    for room_id in room_ids:
        node = {
            "node_id": room_node_id(room_id),
            "node_type": "room",
            "room_id": room_id,
        }
        metadata = room_metadata.get(room_id)
        if metadata:
            node["room_metadata"] = metadata
        nodes.append(node)
    return nodes


def build_gateway_nodes(route_candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    nodes = []
    for candidate in sorted(route_candidates, key=lambda item: item["pair_key"]):
        room_a, room_b = parse_pair_key(candidate["pair_key"])
        nav2_default_usable, review_reason = nav2_default_usable_for(candidate)
        topology_usage = {
            "topology_valid": True,
            "nav2_default_usable": nav2_default_usable,
            "review_required": not nav2_default_usable,
        }
        if review_reason:
            topology_usage["reason"] = review_reason

        nodes.append(
            {
                "node_id": f"gateway_{candidate['candidate_id']}",
                "node_type": "gateway",
                "gateway_id": candidate["candidate_id"],
                "pair_key": candidate["pair_key"],
                "room_a": room_a,
                "room_b": room_b,
                "center_xy": candidate.get("center_xy"),
                "crossing_pose": candidate.get("crossing_pose"),
                "approach_from_room_a": candidate.get("approach_from_room_a"),
                "approach_from_room_b": candidate.get("approach_from_room_b"),
                "passability_status": candidate.get("passability_status"),
                "warnings": candidate.get("warnings") or [],
                "automatic_role": candidate.get("automatic_role"),
                "automatic_score": candidate.get("automatic_score"),
                "score_breakdown": candidate.get("score_breakdown"),
                "topology_usage": topology_usage,
                "source_artifact": rel(PRIMARY_ROUTE_CANDIDATES),
                "selection_reason": candidate.get("selection_reason"),
                "supporting_candidate_ids": candidate.get("supporting_candidate_ids") or [],
            }
        )
    return nodes


def build_gateway_backed_edges(route_candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    edges = []
    for candidate in sorted(route_candidates, key=lambda item: item["pair_key"]):
        room_a, room_b = parse_pair_key(candidate["pair_key"])
        gateway_node = f"gateway_{candidate['candidate_id']}"
        gateway_id = candidate["candidate_id"]
        pair_key = candidate["pair_key"]
        nav2_default_usable, review_reason = nav2_default_usable_for(candidate)
        base = {
            "gateway_id": gateway_id,
            "pair_key": pair_key,
            "passability_status": candidate.get("passability_status"),
            "automatic_role": candidate.get("automatic_role"),
            "automatic_score": candidate.get("automatic_score"),
            "topology_valid": True,
            "nav2_default_usable": nav2_default_usable,
            "review_required": not nav2_default_usable,
            "source_artifact": rel(PRIMARY_ROUTE_CANDIDATES),
        }
        if review_reason:
            base["reason"] = review_reason

        for room_id, room_side in [(room_a, "room_a"), (room_b, "room_b")]:
            edges.append(
                {
                    "edge_id": f"{room_node_id(room_id)}__to__{gateway_node}",
                    "edge_type": "room_to_gateway",
                    "from_node_id": room_node_id(room_id),
                    "to_node_id": gateway_node,
                    "room_side": room_side,
                    **base,
                }
            )
            edges.append(
                {
                    "edge_id": f"{gateway_node}__to__{room_node_id(room_id)}",
                    "edge_type": "gateway_to_room",
                    "from_node_id": gateway_node,
                    "to_node_id": room_node_id(room_id),
                    "room_side": room_side,
                    **base,
                }
            )
    return edges


def build_derived_room_adjacency_edges(route_candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    edges = []
    for candidate in sorted(route_candidates, key=lambda item: item["pair_key"]):
        room_a, room_b = parse_pair_key(candidate["pair_key"])
        nav2_default_usable, review_reason = nav2_default_usable_for(candidate)
        edge = {
            "edge_id": f"{room_node_id(room_a)}__adjacent_via__{candidate['candidate_id']}__{room_node_id(room_b)}",
            "edge_type": "room_adjacency",
            "from_node_id": room_node_id(room_a),
            "to_node_id": room_node_id(room_b),
            "room_a": room_a,
            "room_b": room_b,
            "gateway_id": candidate["candidate_id"],
            "gateway_node_id": f"gateway_{candidate['candidate_id']}",
            "pair_key": candidate["pair_key"],
            "derived_from_gateway": True,
            "direct_room_room_edge": False,
            "topology_valid": True,
            "passability_status": candidate.get("passability_status"),
            "nav2_default_usable": nav2_default_usable,
            "review_required": not nav2_default_usable,
            "source_artifact": rel(PRIMARY_ROUTE_CANDIDATES),
        }
        if review_reason:
            edge["reason"] = review_reason
        edges.append(edge)
    return edges


def build_gateway_edge_table(route_candidates: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows = []
    for candidate in sorted(route_candidates, key=lambda item: item["pair_key"]):
        room_a, room_b = parse_pair_key(candidate["pair_key"])
        nav2_default_usable, review_reason = nav2_default_usable_for(candidate)
        row = {
            "pair_key": candidate["pair_key"],
            "room_a": room_a,
            "room_b": room_b,
            "gateway_id": candidate["candidate_id"],
            "center_xy": candidate.get("center_xy"),
            "crossing_pose": candidate.get("crossing_pose"),
            "approach_from_room_a": candidate.get("approach_from_room_a"),
            "approach_from_room_b": candidate.get("approach_from_room_b"),
            "passability_status": candidate.get("passability_status"),
            "topology_valid": True,
            "nav2_default_usable": nav2_default_usable,
            "review_required": not nav2_default_usable,
            "warnings": candidate.get("warnings") or [],
            "automatic_role": candidate.get("automatic_role"),
            "automatic_score": candidate.get("automatic_score"),
            "score_breakdown": candidate.get("score_breakdown"),
            "source_artifact": rel(PRIMARY_ROUTE_CANDIDATES),
        }
        if review_reason:
            row["reason"] = review_reason
        rows.append(row)
    return rows


def gateway_lookup(edge_table: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {row["pair_key"]: row for row in edge_table}


def build_route_candidate(
    route_id: str,
    description: str,
    room_sequence: Sequence[int],
    edge_table_by_pair: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    gateway_sequence = []
    crossing_pose_sequence = []
    approach_pose_sequence = []
    edge_records = []
    missing_pairs = []
    review_required_edges = []
    passability_statuses = []
    nav2_default_usable = True

    for index, (from_room, to_room) in enumerate(zip(room_sequence, room_sequence[1:])):
        pair_key = pair_key_for_rooms(from_room, to_room)
        row = edge_table_by_pair.get(pair_key)
        if not row:
            missing_pairs.append(pair_key)
            nav2_default_usable = False
            continue

        gateway_sequence.append(row["gateway_id"])
        crossing_pose_sequence.append(row["crossing_pose"])
        if from_room == row["room_a"]:
            approach_pose = row["approach_from_room_a"]
            exit_approach_pose = row["approach_from_room_b"]
        else:
            approach_pose = row["approach_from_room_b"]
            exit_approach_pose = row["approach_from_room_a"]
        approach_pose_sequence.append(
            {
                "segment_index": index,
                "from_room": from_room,
                "to_room": to_room,
                "pair_key": pair_key,
                "gateway_id": row["gateway_id"],
                "approach_pose_from_current_room": approach_pose,
                "approach_pose_from_next_room": exit_approach_pose,
            }
        )
        edge_record = {
            "segment_index": index,
            "from_room": from_room,
            "to_room": to_room,
            "pair_key": pair_key,
            "gateway_id": row["gateway_id"],
            "passability_status": row["passability_status"],
            "nav2_default_usable": row["nav2_default_usable"],
            "review_required": row["review_required"],
            "warnings": row["warnings"],
        }
        if row.get("reason"):
            edge_record["reason"] = row["reason"]
        edge_records.append(edge_record)
        passability_statuses.append(row["passability_status"])
        if not row["nav2_default_usable"]:
            nav2_default_usable = False
            review_required_edges.append(edge_record)

    graph_valid = not missing_pairs
    counts = Counter(passability_statuses)
    return {
        "route_id": route_id,
        "description": description,
        "candidate_type": "topology_query_candidate",
        "room_sequence": [room_node_id(room_id) for room_id in room_sequence],
        "room_id_sequence": list(room_sequence),
        "gateway_sequence": gateway_sequence,
        "crossing_pose_sequence": crossing_pose_sequence,
        "approach_pose_sequence": approach_pose_sequence,
        "edge_sequence": edge_records,
        "missing_pairs": missing_pairs,
        "graph_valid": graph_valid,
        "topology_only": True,
        "nav2_route_generated": False,
        "nav2_default_usable": graph_valid and nav2_default_usable,
        "review_required_edges": review_required_edges,
        "passability_summary": {
            "total_gateway_segments": len(edge_records),
            "status_counts": dict(sorted(counts.items())),
            "review_required_edge_count": len(review_required_edges),
            "nav2_ready_edge_count": sum(1 for edge in edge_records if edge["nav2_default_usable"]),
        },
    }


def build_route_candidates(edge_table: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    by_pair = gateway_lookup(edge_table)
    route_specs = [
        (
            "public_path_042",
            "Public-path-042-style topology route requested for Step30C validation.",
            [1, 3, 7, 11, 8],
        ),
        (
            "long_structure_route_default",
            "Longer gateway-connected structure route that avoids forcing r7_r15 by default.",
            [1, 3, 8, 11, 7, 14, 16],
        ),
        (
            "long_structure_review_all_positive_gateways",
            "Review edge-cover walk that includes every discovered positive gateway pair, with repeated review edges where graph parity requires it.",
            [1, 3, 8, 11, 7, 15, 7, 3, 7, 14, 16],
        ),
    ]
    return [build_route_candidate(route_id, description, rooms, by_pair) for route_id, description, rooms in route_specs]


def room_marker_xy(room_node: Dict[str, Any], edge_table: Sequence[Dict[str, Any]]) -> Optional[List[float]]:
    metadata = room_node.get("room_metadata") or {}
    centroid = metadata.get("centroid_xy")
    if centroid and len(centroid) == 2:
        return [float(centroid[0]), float(centroid[1])]

    room_id = room_node["room_id"]
    points = []
    for row in edge_table:
        if row["room_a"] == room_id or row["room_b"] == room_id:
            points.append(row["center_xy"])
    if not points:
        return None
    return [
        round(sum(float(point[0]) for point in points) / len(points), 4),
        round(sum(float(point[1]) for point in points) / len(points), 4),
    ]


def build_overlay_payload(
    room_nodes: Sequence[Dict[str, Any]],
    gateway_nodes: Sequence[Dict[str, Any]],
    edge_table: Sequence[Dict[str, Any]],
    route_candidates: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    room_markers = []
    room_xy_by_id: Dict[int, List[float]] = {}
    for node in room_nodes:
        xy = room_marker_xy(node, edge_table)
        if xy:
            room_xy_by_id[node["room_id"]] = xy
        room_markers.append(
            {
                "marker_id": node["node_id"],
                "marker_type": "room",
                "room_id": node["room_id"],
                "xy": xy,
                "label": node["node_id"],
            }
        )

    gateway_markers = [
        {
            "marker_id": node["gateway_id"],
            "marker_type": "gateway",
            "gateway_id": node["gateway_id"],
            "pair_key": node["pair_key"],
            "xy": node["center_xy"],
            "label": f"{node['pair_key']}:{node['gateway_id']}",
            "passability_status": node["passability_status"],
            "nav2_default_usable": node["topology_usage"]["nav2_default_usable"],
        }
        for node in gateway_nodes
    ]

    edge_segments = []
    for row in edge_table:
        for room_id in [row["room_a"], row["room_b"]]:
            room_xy = room_xy_by_id.get(room_id)
            if not room_xy:
                continue
            edge_segments.append(
                {
                    "segment_id": f"{room_node_id(room_id)}__{row['gateway_id']}",
                    "segment_type": "room_to_gateway",
                    "room_id": room_id,
                    "gateway_id": row["gateway_id"],
                    "pair_key": row["pair_key"],
                    "points_xy": [room_xy, row["center_xy"]],
                    "passability_status": row["passability_status"],
                    "nav2_default_usable": row["nav2_default_usable"],
                }
            )

    gateway_center_by_id = {row["gateway_id"]: row["center_xy"] for row in edge_table}
    route_polylines = []
    for route in route_candidates:
        points = []
        room_ids = route["room_id_sequence"]
        if room_ids and room_ids[0] in room_xy_by_id:
            points.append(room_xy_by_id[room_ids[0]])
        for gateway_id, next_room in zip(route["gateway_sequence"], room_ids[1:]):
            points.append(gateway_center_by_id[gateway_id])
            if next_room in room_xy_by_id:
                points.append(room_xy_by_id[next_room])
        route_polylines.append(
            {
                "route_id": route["route_id"],
                "label": route["route_id"],
                "points_xy": points,
                "gateway_sequence": route["gateway_sequence"],
                "graph_valid": route["graph_valid"],
                "nav2_default_usable": route["nav2_default_usable"],
                "review_required_edge_count": len(route["review_required_edges"]),
            }
        )

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30c_overlay_payload",
        "step": STEP,
        "version": VERSION,
        "coordinate_frame": "stage_a_map_xy",
        "topology_only": True,
        "rooms": room_markers,
        "gateway_markers": gateway_markers,
        "edge_segments": edge_segments,
        "route_polylines": route_polylines,
        "marker_labels": [
            {"marker_id": marker["marker_id"], "label": marker["label"]}
            for marker in room_markers + gateway_markers
        ],
    }


def required_gateway_fields_present(row: Dict[str, Any]) -> Tuple[bool, List[str]]:
    required_fields = [
        "gateway_id",
        "pair_key",
        "room_a",
        "room_b",
        "crossing_pose",
        "approach_from_room_a",
        "approach_from_room_b",
        "passability_status",
        "automatic_score",
        "score_breakdown",
        "source_artifact",
    ]
    missing = [field for field in required_fields if row.get(field) in (None, "")]
    return not missing, missing


def validate_step30c_outputs(
    edge_table: Sequence[Dict[str, Any]], route_candidates: Sequence[Dict[str, Any]]
) -> Dict[str, Any]:
    observed_pairs = sorted(row["pair_key"] for row in edge_table)
    observed_pair_set = set(observed_pairs)
    expected_pair_set = set(EXPECTED_POSITIVE_PAIRS)
    by_route_id = {route["route_id"]: route for route in route_candidates}
    public_route = by_route_id["public_path_042"]
    long_route = by_route_id["long_structure_route_default"]

    missing_field_details = []
    for row in edge_table:
        ok, missing = required_gateway_fields_present(row)
        if not ok:
            missing_field_details.append({"pair_key": row.get("pair_key"), "missing_fields": missing})

    checks = [
        {
            "check_id": "expected_positive_pairs_exact",
            "passed": observed_pair_set == expected_pair_set,
            "details": {
                "expected_pairs": sorted(expected_pair_set),
                "observed_pairs": observed_pairs,
                "missing_pairs": sorted(expected_pair_set - observed_pair_set),
                "unexpected_pairs": sorted(observed_pair_set - expected_pair_set),
            },
        },
        {
            "check_id": "r3_r11_absent",
            "passed": "r3_r11" not in observed_pair_set,
            "details": {"pair_key": "r3_r11", "observed": "r3_r11" in observed_pair_set},
        },
        {
            "check_id": "non_truth_pairs_absent",
            "passed": not (set(NON_TRUTH_PAIRS) & observed_pair_set),
            "details": {
                "non_truth_pairs": NON_TRUTH_PAIRS,
                "observed_non_truth_pairs": sorted(set(NON_TRUTH_PAIRS) & observed_pair_set),
            },
        },
        {
            "check_id": "gateway_edge_required_fields_present",
            "passed": not missing_field_details,
            "details": {"missing_field_details": missing_field_details},
        },
        {
            "check_id": "public_path_042_graph_valid",
            "passed": public_route["graph_valid"],
            "details": {
                "room_sequence": public_route["room_sequence"],
                "missing_pairs": public_route["missing_pairs"],
            },
        },
        {
            "check_id": "public_path_042_expected_gateway_sequence",
            "passed": public_route["gateway_sequence"] == EXPECTED_PUBLIC_PATH_042_GATEWAYS,
            "details": {
                "expected_gateway_sequence": EXPECTED_PUBLIC_PATH_042_GATEWAYS,
                "observed_gateway_sequence": public_route["gateway_sequence"],
            },
        },
        {
            "check_id": "long_route_candidate_graph_valid",
            "passed": long_route["graph_valid"],
            "details": {
                "route_id": long_route["route_id"],
                "room_sequence": long_route["room_sequence"],
                "gateway_sequence": long_route["gateway_sequence"],
                "missing_pairs": long_route["missing_pairs"],
            },
        },
        {
            "check_id": "no_ros_nav2_gazebo_run",
            "passed": True,
            "details": {
                "ros_nav2_gazebo_run": False,
                "nav2_route_generated": False,
                "robot_execution": False,
            },
        },
        {
            "check_id": "no_stage_a_rerun",
            "passed": True,
            "details": {"stage_a_rerun_attempted": False, "gateway_extraction_rerun_attempted": False},
        },
        {
            "check_id": "topology_candidate_only",
            "passed": True,
            "details": {"topology_generated": True, "robot_execution": False},
        },
    ]
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30c_validation_results",
        "step": STEP,
        "version": VERSION,
        "checks": checks,
        "validation_passed": all(check["passed"] for check in checks),
    }


def write_summary(
    edge_table: Sequence[Dict[str, Any]],
    route_candidates: Sequence[Dict[str, Any]],
    validation_results: Dict[str, Any],
) -> Dict[str, Any]:
    by_route_id = {route["route_id"]: route for route in route_candidates}
    public_route = by_route_id["public_path_042"]
    long_route = by_route_id["long_structure_route_default"]
    observed_pair_set = {row["pair_key"] for row in edge_table}
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30c_summary",
        "step": STEP,
        "version": VERSION,
        "gateway_edges_total": len(edge_table),
        "positive_truth_pairs_present": sorted(observed_pair_set & set(EXPECTED_POSITIVE_PAIRS)),
        "positive_truth_pairs_present_count": len(observed_pair_set & set(EXPECTED_POSITIVE_PAIRS)),
        "r3_r11_absent": "r3_r11" not in observed_pair_set,
        "non_truth_pairs_absent": not (observed_pair_set & set(NON_TRUTH_PAIRS)),
        "public_path_042_valid": public_route["graph_valid"]
        and public_route["gateway_sequence"] == EXPECTED_PUBLIC_PATH_042_GATEWAYS,
        "long_route_candidate_valid": long_route["graph_valid"],
        "nav2_ready_edge_count": sum(1 for row in edge_table if row["nav2_default_usable"]),
        "review_required_edge_count": sum(1 for row in edge_table if row["review_required"]),
        "topology_generated": True,
        "ros_nav2_gazebo_run": False,
        "stage_a_rerun_attempted": False,
        "gateway_extraction_rerun_attempted": False,
        "nav2_route_generated": False,
        "robot_execution": False,
        "validation_passed": validation_results["validation_passed"],
    }


def main() -> None:
    route_candidates, validation_payloads, source_artifacts = load_step30b2_route_candidates()
    room_metadata, optional_stage_a_artifacts = load_optional_room_metadata()
    source_artifacts.update({f"optional_{key}": value for key, value in optional_stage_a_artifacts.items()})

    room_nodes = build_room_nodes(route_candidates, room_metadata)
    gateway_nodes = build_gateway_nodes(route_candidates)
    room_gateway_edges = build_gateway_backed_edges(route_candidates)
    derived_room_edges = build_derived_room_adjacency_edges(route_candidates)
    edge_table = build_gateway_edge_table(route_candidates)
    route_candidate_rows = build_route_candidates(edge_table)
    overlay_payload = build_overlay_payload(room_nodes, gateway_nodes, edge_table, route_candidate_rows)
    validation_results = validate_step30c_outputs(edge_table, route_candidate_rows)
    summary = write_summary(edge_table, route_candidate_rows, validation_results)

    graph_payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30c_gateway_augmented_topology_candidate",
        "step": STEP,
        "version": VERSION,
        "topology_only": True,
        "nav2_route_generated": False,
        "robot_execution": False,
        "stage_a_rerun_attempted": False,
        "gateway_extraction_rerun_attempted": False,
        "ros_nav2_gazebo_run": False,
        "truth_blind_gateway_selection": True,
        "truth_used_for_selection": False,
        "source_artifacts": source_artifacts,
        "validation_inputs_read_for_provenance_only": {
            key: payload.get("artifact_type") for key, payload in validation_payloads.items()
        },
        "nodes": {
            "rooms": room_nodes,
            "gateways": gateway_nodes,
        },
        "edges": {
            "room_to_gateway_and_gateway_to_room": room_gateway_edges,
            "derived_room_adjacency": derived_room_edges,
        },
        "graph_statistics": {
            "room_node_count": len(room_nodes),
            "gateway_node_count": len(gateway_nodes),
            "room_gateway_directed_edge_count": len(room_gateway_edges),
            "derived_room_adjacency_count": len(derived_room_edges),
            "gateway_backed_topology_adjacency_count": len(edge_table),
        },
    }

    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_gateway_augmented_topology_candidate_{VERSION}.json",
        graph_payload,
    )
    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_gateway_edge_table_{VERSION}.json",
        {
            "scene_id": SCENE_ID,
            "artifact_type": "step30c_gateway_edge_table",
            "step": STEP,
            "version": VERSION,
            "source_artifact": rel(PRIMARY_ROUTE_CANDIDATES),
            "gateway_edges": edge_table,
        },
    )
    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_topology_route_candidates_{VERSION}.json",
        {
            "scene_id": SCENE_ID,
            "artifact_type": "step30c_topology_route_candidates",
            "step": STEP,
            "version": VERSION,
            "topology_only": True,
            "nav2_route_generated": False,
            "route_candidates": route_candidate_rows,
        },
    )
    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_overlay_payload_{VERSION}.json",
        overlay_payload,
    )
    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_validation_results_{VERSION}.json",
        validation_results,
    )
    write_json(
        OUTPUT_ROOT / f"{SHORT}_step30c_summary_{VERSION}.json",
        summary,
    )

    if not validation_results["validation_passed"]:
        raise SystemExit("Step30C validation failed")


if __name__ == "__main__":
    main()
