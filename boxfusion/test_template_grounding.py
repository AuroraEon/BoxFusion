import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopologyBuilder
from boxfusion.template_grounding import TemplateGrounder


def _build_named_room_vector_map() -> Dict[str, Any]:
    return {
        "rooms": [
            {
                "id": 1,
                "room_type": "kitchen",
                "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            },
            {
                "id": 2,
                "room_type": "living_room",
                "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]],
            },
            {
                "id": 3,
                "room_type": "bedroom",
                "polygon": [[8.25, 0.0], [12.25, 0.0], [12.25, 4.0], [8.25, 4.0]],
            },
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
            {"id": "anchor_5", "room_id": "room_3", "target_id": "room_3", "anchor_type": "room"},
        ],
    }


def _build_transition_history() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": 1},
        {"frame_idx": 1, "timestamp": 0.1, "current_room_id": 1},
        {"frame_idx": 2, "timestamp": 0.2, "current_room_id": 2},
        {"frame_idx": 3, "timestamp": 0.3, "current_room_id": 2},
        {"frame_idx": 4, "timestamp": 0.4, "current_room_id": 3},
        {"frame_idx": 5, "timestamp": 0.5, "current_room_id": 3},
    ]


def run_mock_test() -> None:
    topology = RoomTopologyBuilder().build(
        _build_named_room_vector_map(),
        sequence_id="template_grounding_sequence",
        transition_history=_build_transition_history(),
        metadata={"test_case": "template_grounding"},
    )
    grounder = TemplateGrounder(RoomTopologyQueryAPI(topology))

    room_query = grounder.ground("go to kitchen", start_room_id="living_room")
    assert room_query["ok"], "go to <room> should parse, ground, and route"
    assert room_query["parse"]["template_id"] == "go_to_room"
    assert room_query["grounding"]["resolved_goal_room_id"] == "room_1"
    assert room_query["query_api_request"]["method"] == "query_route"

    room_query_with_article = grounder.ground("go to the kitchen", start_room_id="room_2")
    assert room_query_with_article["ok"], "go to the <room> should route through the same adapter"
    assert room_query_with_article["parse"]["template_id"] == "go_to_the_room"
    assert room_query_with_article["query_api_result"]["route"]["room_sequence"] == ["room_2", "room_1"]

    room_to_room_query = grounder.ground("go from living_room to bedroom")
    assert room_to_room_query["ok"], "room-to-room template should resolve both endpoints"
    assert room_to_room_query["parse"]["intent"] == "go_from_room_to_room"
    assert room_to_room_query["query_api_result"]["start_room_id"] == "room_2"
    assert room_to_room_query["query_api_result"]["resolved_goal_room_id"] == "room_3"

    object_query = grounder.ground("go to the room with sofa", start_room_id="kitchen")
    assert object_query["ok"], "object-target template should delegate to query_route_to_object"
    assert object_query["query_api_request"]["method"] == "query_route_to_object"
    assert object_query["query_api_result"]["resolved_goal_room_id"] == "room_2"

    anchor_query = grounder.ground("go to anchor anchor_5", start_room_id="kitchen")
    assert anchor_query["ok"], "anchor-target template should delegate to query_route_to_anchor"
    assert anchor_query["query_api_request"]["method"] == "query_route_to_anchor"
    assert anchor_query["query_api_result"]["resolved_goal_room_id"] == "room_3"

    start_required = grounder.ground("go to kitchen")
    assert not start_required["ok"], "single-target route templates should require explicit start-room context"
    assert start_required["failure_reason"] == "START_ROOM_REQUIRED"

    unresolved_room = grounder.ground("go to pantry", start_room_id="living_room")
    assert not unresolved_room["ok"], "unknown room slots should fail explicitly"
    assert unresolved_room["failure_reason"] == "TARGET_ROOM_UNRESOLVED"

    unresolved_object = grounder.ground("go to the room with lamp", start_room_id="living_room")
    assert not unresolved_object["ok"], "missing object labels should fail explicitly"
    assert unresolved_object["failure_reason"] == "OBJECT_LABEL_UNRESOLVED"

    unresolved_anchor = grounder.ground("go to anchor anchor_missing", start_room_id="living_room")
    assert not unresolved_anchor["ok"], "missing anchors should fail explicitly"
    assert unresolved_anchor["failure_reason"] == "ANCHOR_ID_UNRESOLVED"

    unsupported_cases = [
        "find sofa",
        "go near kitchen",
        "take me to bedroom",
        "go to object sofa",
    ]
    for instruction in unsupported_cases:
        result = grounder.ground(instruction, start_room_id="living_room")
        assert not result["ok"], f"{instruction!r} should be rejected by the template parser"
        assert result["failure_reason"] == "UNSUPPORTED_TEMPLATE"

    print("Template grounding validated.")


if __name__ == "__main__":
    run_mock_test()
