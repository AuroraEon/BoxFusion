import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopology, RoomTopologyBuilder
from stage_a_topology_acceptance import build_acceptance_report, build_topology_report


def _build_mock_vector_map() -> dict:
    return {
        "rooms": [
            {"id": 1, "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]},
            {"id": 2, "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]]},
            {"id": 3, "polygon": [[8.25, 0.0], [12.25, 0.0], [12.25, 4.0], [8.25, 4.0]]},
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
            {"connects": [2, 3], "type": "open_passage", "pos_world": [8.18, 2.0]},
        ],
        "objects": [
            {"id": 101, "label": "table", "room_uuid": 1},
            {"id": 201, "label": "sofa", "room_uuid": 2},
            {"id": 301, "label": "bed", "room_uuid": 3},
        ],
        "anchors": [
            {"id": "anchor_room_1", "room_id": "room_1", "target_id": "room_1", "anchor_type": "room"},
            {"id": "anchor_obj_201", "room_id": "room_2", "target_id": "obj_201", "anchor_type": "object"},
        ],
    }


def _build_transition_history() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": 1},
        {"frame_idx": 1, "timestamp": 0.1, "current_room_id": 1},
        {"frame_idx": 2, "timestamp": 0.2, "current_room_id": 2},
        {"frame_idx": 3, "timestamp": 0.3, "current_room_id": 2},
        {"frame_idx": 4, "timestamp": 0.4, "current_room_id": 1},
        {"frame_idx": 5, "timestamp": 0.5, "current_room_id": 1},
        {"frame_idx": 6, "timestamp": 0.6, "current_room_id": 2},
        {"frame_idx": 7, "timestamp": 0.7, "current_room_id": 2},
        {"frame_idx": 8, "timestamp": 0.8, "current_room_id": 3},
        {"frame_idx": 9, "timestamp": 0.9, "current_room_id": 3},
    ]


def _build_mixed_route_transition_history() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": 1},
        {"frame_idx": 1, "timestamp": 0.1, "current_room_id": 1},
        {"frame_idx": 2, "timestamp": 0.2, "current_room_id": 2},
        {"frame_idx": 3, "timestamp": 0.3, "current_room_id": 2},
    ]


def _build_possible_connection_vector_map() -> dict:
    return {
        "rooms": [
            {"id": 1, "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]},
            {"id": 2, "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]]},
            {"id": 3, "polygon": [[8.9, 0.0], [12.9, 0.0], [12.9, 4.0], [8.9, 4.0]]},
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
        ],
        "objects": [],
        "anchors": [],
    }


def _build_ambiguous_object_vector_map() -> dict:
    return {
        "rooms": [
            {"id": 1, "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]},
            {"id": 2, "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]]},
            {"id": 3, "polygon": [[8.25, 0.0], [12.25, 0.0], [12.25, 4.0], [8.25, 4.0]]},
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
            {"connects": [2, 3], "type": "open_passage", "pos_world": [8.18, 2.0]},
        ],
        "objects": [
            {"id": 101, "label": "table", "room_uuid": 1, "score": 0.91},
            {"id": 201, "label": "sofa", "room_uuid": 2, "score": 0.83},
            {"id": 301, "label": "sofa", "room_uuid": 3, "score": 0.77},
        ],
        "anchors": [
            {"id": "anchor_obj_201", "room_id": "room_2", "target_id": "obj_201", "anchor_type": "object"},
        ],
    }


def _build_fallback_metadata_vector_map() -> dict:
    return {
        "rooms": [
            {"id": 1, "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]},
            {"id": 2, "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]]},
            {"id": 3, "polygon": [[8.25, 0.0], [12.25, 0.0], [12.25, 4.0], [8.25, 4.0]]},
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
            {"connects": [2, 3], "type": "open_passage", "pos_world": [8.18, 2.0]},
        ],
        "objects": [
            {"id": 101, "label": "table", "room_uuid": 1},
            {"id": 201, "label": "sofa", "room_id": "room_2"},
            {"id": 301, "category": "bed", "room_id": 3},
        ],
        "anchors": [
            {"id": "anchor_room_2", "target_id": "room_2", "anchor_type": "room"},
            {"id": "anchor_obj_201", "target_id": "obj_201", "anchor_type": "object"},
        ],
    }


def _build_disconnected_vector_map() -> dict:
    return {
        "rooms": [
            {"id": 1, "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]},
            {"id": 2, "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]]},
            {"id": 3, "polygon": [[20.0, 0.0], [24.0, 0.0], [24.0, 4.0], [20.0, 4.0]]},
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
        ],
        "objects": [],
        "anchors": [],
    }


def run_mock_test() -> None:
    print("Running room topology validation...")
    topology = RoomTopologyBuilder().build(
        _build_mock_vector_map(),
        sequence_id="mock_sequence",
        transition_history=_build_transition_history(),
        metadata={"test_case": "mock_room_topology"},
    )

    errors = topology.validate()
    assert not errors, f"topology validation must pass: {errors}"

    assert topology.get_room("room_1") is not None
    assert topology.get_room_of_object(101) == "room_1"
    assert topology.get_room_of_anchor("anchor_obj_201") == "room_2"

    neighbors = topology.get_room_neighbors("room_1")
    neighbor_pairs = {(item["room_id"], item["relation_type"]) for item in neighbors}
    assert ("room_2", "transition") in neighbor_pairs
    assert ("room_2", "adjacent") in neighbor_pairs

    explanation = topology.explain_connection("room_1", "room_2")
    assert explanation["exists"], "room_1 and room_2 should be connected"
    relation_types = {item["relation_type"] for item in explanation["relations"]}
    assert "transition" in relation_types
    assert "adjacent" in relation_types
    transition_relation = next(item for item in explanation["relations"] if item["relation_type"] == "transition")
    assert transition_relation["support_count"] >= 3, "expected repeated crossings to contribute evidence"

    contents = topology.get_room_contents("room_2")
    assert contents["object_count"] == 1
    assert contents["anchor_count"] == 1

    route = topology.find_room_route("room_1", "room_3")
    assert route is not None, "route from room_1 to room_3 should exist"
    assert route["room_sequence"] == ["room_1", "room_2", "room_3"]
    assert route["used_relation_types"] == ["transition", "transition"], "routing should prefer transition edges"

    direct_path = topology.find_room_path("room_1", "room_2")
    assert direct_path["found"], "direct graph-search path should be found"
    assert direct_path["used_relation_types"] == ["transition"], "direct path should prefer transition over adjacent"
    assert direct_path["failure_reason"] is None
    assert direct_path["edges"][0]["relation_type"] == "transition"

    explained_path = topology.explain_room_path("room_1", "room_3")
    assert explained_path["found"]
    assert len(explained_path["hop_explanations"]) == 2
    assert explained_path["hop_explanations"][0]["relation_explanation"]["relation_type"] == "transition"

    candidate_paths = topology.find_candidate_room_paths("room_1", "room_3", max_paths=2)
    assert candidate_paths["found"]
    assert candidate_paths["paths"][0]["room_sequence"] == ["room_1", "room_2", "room_3"]

    summary = topology.summarize_room("room_2")
    assert summary["exists"]
    assert summary["object_count"] == 1

    mixed_topology = RoomTopologyBuilder().build(
        _build_mock_vector_map(),
        sequence_id="mixed_route_sequence",
        transition_history=_build_mixed_route_transition_history(),
        metadata={"test_case": "mixed_room_route"},
    )
    mixed_path = mixed_topology.find_room_path("room_1", "room_3")
    assert mixed_path["found"], "mixed transition + adjacent route should exist"
    assert mixed_path["used_relation_types"] == ["transition", "adjacent"]

    possible_topology = RoomTopologyBuilder().build(
        _build_possible_connection_vector_map(),
        sequence_id="possible_route_sequence",
        transition_history=_build_mixed_route_transition_history(),
        metadata={"test_case": "possible_connection_route"},
    )
    possible_path = possible_topology.find_room_path("room_1", "room_3")
    assert possible_path["found"], "possible_connection route should exist as fallback"
    assert possible_path["used_relation_types"] == ["transition", "possible_connection"]
    assert possible_path["edges"][-1]["relation_type"] == "possible_connection"

    filtered_path = possible_topology.find_room_path("room_1", "room_3", min_conf=0.1)
    assert not filtered_path["found"], "min_conf filtering should be able to remove weak fallback edges"
    assert filtered_path["failure_reason"] == "no_path_found"

    disconnected_topology = RoomTopologyBuilder().build(
        _build_disconnected_vector_map(),
        sequence_id="disconnected_sequence",
        transition_history=_build_mixed_route_transition_history(),
        metadata={"test_case": "disconnected_room_route"},
    )
    no_path = disconnected_topology.find_room_path("room_1", "room_3")
    assert not no_path["found"], "disconnected rooms should not produce a route"
    assert no_path["failure_reason"] == "no_path_found"

    with tempfile.TemporaryDirectory(prefix="room-topology-") as tmp_dir:
        tmp_root = Path(tmp_dir)
        json_path = topology.export_json(tmp_root / "topology_v0_1.json")
        report_path = topology.export_query_report(tmp_root / "topology_query_report.json")
        graphml_path = topology.export_graphml(tmp_root / "topology_v0_1.graphml")
        with open(json_path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        assert payload["version"] == "0.1"
        assert payload["sequence_id"] == "mock_sequence"
        assert payload["entities"]["objects"][0]["normalized_label"] == "table"
        assert Path(report_path).exists()
        assert Path(graphml_path).exists()
        reloaded_topology = RoomTopology.from_json(json_path)
        reloaded_path = reloaded_topology.find_room_path("room_1", "room_3")
        assert reloaded_path["found"], "reloaded topology JSON should support graph search"
        assert reloaded_path["room_sequence"] == ["room_1", "room_2", "room_3"]

        query_api = RoomTopologyQueryAPI(reloaded_topology)
        reloaded_object_route = query_api.query_route_to_object("room_3", object_label="table")
        assert reloaded_object_route["found"], "reloaded topology JSON should preserve object-label query support"
        assert reloaded_object_route["resolved_goal_room_id"] == "room_1"

        topology_report = build_topology_report(json_path, sample_limit=3)
        assert topology_report["inspection"]["room_count"] == 3
        assert topology_report["inspection"]["anchor_count"] == 2
        assert topology_report["inspection"]["object_count"] == 3
        assert topology_report["inspection"]["object_label_count"] == 3

        acceptance_report = build_acceptance_report(json_path, sample_limit=3, compare_policies=True)
        assert acceptance_report["acceptance"]["route_to_anchor"]["success"]
        assert acceptance_report["acceptance"]["route_to_object_id"]["success"]
        assert acceptance_report["acceptance"]["route_to_object_label"]["success"]
        assert "policy_results" in acceptance_report["acceptance"]["route_to_anchor"]

        legacy_payload = topology.to_dict(include_query_examples=False)
        legacy_payload.pop("entities", None)
        legacy_json_path = tmp_root / "legacy_topology_v0_1.json"
        with open(legacy_json_path, "w", encoding="utf-8") as f:
            json.dump(legacy_payload, f, indent=2)
        legacy_report = build_topology_report(legacy_json_path, sample_limit=3)
        assert "object label lookup unavailable" in legacy_report["inspection"]["diagnostics"]
        legacy_acceptance = build_acceptance_report(legacy_json_path, sample_limit=3)
        assert legacy_acceptance["acceptance"]["route_to_anchor"]["success"]
        assert legacy_acceptance["acceptance"]["route_to_object_id"]["success"]
        assert not legacy_acceptance["acceptance"]["route_to_object_label"]["success"]
        assert legacy_acceptance["acceptance"]["route_to_object_label"]["failure_reason"] == "object_label_lookup_unavailable"

    query_api = RoomTopologyQueryAPI(topology)
    room_query = query_api.query_route("room_1", "room_3")
    assert room_query["found"], "room-target query should resolve and route successfully"
    assert room_query["route"]["room_sequence"] == ["room_1", "room_2", "room_3"]
    assert room_query["target_resolution"]["resolved_room_id"] == "room_3"

    anchor_query = query_api.query_route_to_anchor("room_1", "anchor_obj_201")
    assert anchor_query["found"], "anchor-target query should resolve and route successfully"
    assert anchor_query["resolved_goal_room_id"] == "room_2"
    assert anchor_query["target_resolution"]["target_type"] == "anchor"

    object_id_query = query_api.query_route_to_object("room_3", object_id=101)
    assert object_id_query["found"], "object-id query should resolve and route successfully"
    assert object_id_query["resolved_goal_room_id"] == "room_1"
    assert object_id_query["target_resolution"]["matched_entity_ids"] == ["obj_101"]

    object_label_query = query_api.query_route_to_object("room_3", object_label="table")
    assert object_label_query["found"], "object-label query should resolve and route successfully"
    assert object_label_query["resolved_goal_room_id"] == "room_1"
    assert object_label_query["target_resolution"]["matched_entity_ids"] == ["obj_101"]
    assert "table" in object_label_query["explanation"]["target_summary"]

    missing_anchor_query = query_api.query_route_to_anchor("room_1", "anchor_missing")
    assert not missing_anchor_query["found"], "missing anchors should return a structured failure"
    assert missing_anchor_query["failure_reason"] == "anchor_not_found"
    assert not missing_anchor_query["route"]["attempted"]

    ambiguous_topology = RoomTopologyBuilder().build(
        _build_ambiguous_object_vector_map(),
        sequence_id="ambiguous_object_sequence",
        transition_history=_build_transition_history(),
        metadata={"test_case": "ambiguous_object_query"},
    )
    ambiguous_query_api = RoomTopologyQueryAPI(ambiguous_topology)
    ambiguous_object_query = ambiguous_query_api.query_route_to_object("room_1", object_label="sofa")
    assert not ambiguous_object_query["found"], "ambiguous object-label matches should not silently pick a room"
    assert ambiguous_object_query["failure_reason"] == "object_label_ambiguous"
    assert ambiguous_object_query["target_resolution"]["ambiguity"]["candidate_room_ids"] == ["room_2", "room_3"]

    fallback_topology = RoomTopologyBuilder().build(
        _build_fallback_metadata_vector_map(),
        sequence_id="fallback_metadata_sequence",
        transition_history=_build_transition_history(),
        metadata={"test_case": "fallback_metadata"},
    )
    assert fallback_topology.get_room_of_object(201) == "room_2"
    assert fallback_topology.get_room_of_object(301) == "room_3"
    assert fallback_topology.get_room_of_anchor("anchor_room_2") == "room_2"
    assert fallback_topology.get_room_of_anchor("anchor_obj_201") == "room_2"
    assert fallback_topology.get_object("obj_301")["label"] == "bed"
    assert "bed" in fallback_topology.list_object_labels()

    strict_query = RoomTopologyQueryAPI(possible_topology).query_route("room_1", "room_3", route_policy="strict")
    balanced_query = RoomTopologyQueryAPI(possible_topology).query_route("room_1", "room_3", route_policy="balanced")
    assert not strict_query["found"], "strict policy should be able to reject weak fallback paths"
    assert strict_query["failure_reason"] == "no_path_found"
    assert balanced_query["found"], "balanced policy should allow weak fallback paths when needed"
    assert balanced_query["route"]["used_relation_types"] == ["transition", "possible_connection"]

    print("Room topology validated.")


if __name__ == "__main__":
    run_mock_test()
