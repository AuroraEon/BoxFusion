import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopology
from boxfusion.vln_closed_loop import VLNClosedLoopExecutor
from boxfusion.vln_tool_use import MinimalVLNToolUseAdapter


def _mock_topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_vln_tool_use",
        "floors": [
            {"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
        "rooms": [
            {
                "id": "room_1",
                "room_type": "kitchen",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 1,
                "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]],
            },
            {
                "id": "room_2",
                "room_type": "living_room",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 2,
                "polygon": [[1.1, 0], [2.1, 0], [2.1, 1], [1.1, 1]],
            },
            {
                "id": "room_3",
                "room_type": "bedroom",
                "floor_id": "floor_2",
                "display_floor_id": "floor_2",
                "display_order": 3,
                "polygon": [[2.2, 0], [3.2, 0], [3.2, 1], [2.2, 1]],
            },
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "transition",
                "confidence": 1.0,
                "support_count": 3,
                "metadata": {},
            },
            {
                "source": "room_2",
                "target": "room_3",
                "relation_type": "vertical_transition",
                "confidence": 0.8,
                "support_count": 2,
                "metadata": {"transition_ids": ["vt_1"]},
            },
        ],
        "indices": {
            "object_to_room": {
                "obj_101": "room_2",
                "obj_201": "room_3",
                "obj_301": "room_1",
                "obj_302": "room_3",
            },
            "anchor_to_room": {
                "anchor_obj_201": "room_3",
                "anchor_room_3": "room_3",
            },
            "room_to_objects": {
                "room_1": ["obj_301"],
                "room_2": ["obj_101"],
                "room_3": ["obj_201", "obj_302"],
            },
            "room_to_anchors": {
                "room_1": [],
                "room_2": [],
                "room_3": ["anchor_obj_201", "anchor_room_3"],
            },
        },
        "entities": {
            "objects": [
                {
                    "id": "obj_101",
                    "label": "sofa",
                    "normalized_label": "sofa",
                    "room_id": "room_2",
                    "floor_id": "floor_1",
                    "display_floor_id": "floor_1",
                    "score": 0.9,
                },
                {
                    "id": "obj_201",
                    "label": "toilet",
                    "normalized_label": "toilet",
                    "room_id": "room_3",
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                    "score": 0.95,
                },
                {
                    "id": "obj_301",
                    "label": "chair",
                    "normalized_label": "chair",
                    "room_id": "room_1",
                    "floor_id": "floor_1",
                    "display_floor_id": "floor_1",
                    "score": 0.75,
                },
                {
                    "id": "obj_302",
                    "label": "chair",
                    "normalized_label": "chair",
                    "room_id": "room_3",
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                    "score": 0.8,
                },
            ],
            "anchors": [
                {
                    "id": "anchor_obj_201",
                    "anchor_type": "object",
                    "room_id": "room_3",
                    "target_id": "obj_201",
                    "valid": True,
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                    "score": 0.9,
                },
                {
                    "id": "anchor_room_3",
                    "anchor_type": "room",
                    "room_id": "room_3",
                    "target_id": "room_3",
                    "valid": True,
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                    "score": 0.7,
                },
            ],
        },
        "evidences": [],
        "metadata": {},
    }


def _success_observations() -> List[Dict[str, Any]]:
    return [
        {
            "frame_idx": 0,
            "timestamp": 0.0,
            "current_room_id": "room_1",
            "current_floor_id": "floor_1",
            "current_display_floor_id": "floor_1",
            "current_floor_label": "floor_1",
        },
        {
            "frame_idx": 1,
            "timestamp": 1.0,
            "current_room_id": "room_2",
            "current_floor_id": "floor_1",
            "current_display_floor_id": "floor_1",
            "current_floor_label": "floor_1",
        },
        {
            "frame_idx": 2,
            "timestamp": 2.0,
            "current_room_id": "room_3",
            "current_floor_id": "floor_2",
            "current_display_floor_id": "floor_2",
            "current_floor_label": "floor_2",
        },
    ]


def run_mock_test() -> None:
    topology = RoomTopology.from_dict(_mock_topology_payload())
    query_api = RoomTopologyQueryAPI(topology)
    executor = VLNClosedLoopExecutor(query_api, _success_observations())
    adapter = MinimalVLNToolUseAdapter(query_api=query_api, executor=executor)

    explain_room = adapter.handle_request(
        "How do I get to the bedroom upstairs?",
        start_room_id="room_1",
    )
    assert explain_room["status"] == "success"
    assert explain_room["parsed_request"]["request_type"] == "route_explanation"
    assert explain_room["parsed_request"]["target_type"] == "room"
    assert explain_room["tool_selection"]["tool_name"] == "query_route"
    assert explain_room["teacher_response"]["floor_switch_required"] is True

    explain_anchor = adapter.handle_request(
        "How do I get to the toilet anchor upstairs?",
        start_room_id="room_1",
    )
    assert explain_anchor["status"] == "success"
    assert explain_anchor["parsed_request"]["target_type"] == "anchor"
    assert explain_anchor["tool_selection"]["tool_name"] == "query_route_to_anchor"
    assert explain_anchor["resolved_target"]["selected_entity_id"] == "anchor_obj_201"

    explain_object = adapter.handle_request(
        "How do I get to the sofa?",
        start_room_id="room_1",
    )
    assert explain_object["status"] == "success"
    assert explain_object["parsed_request"]["target_type"] == "object"
    assert explain_object["tool_selection"]["tool_name"] == "query_route_to_object"
    assert explain_object["resolved_target"]["selected_entity_id"] == "obj_101"

    execute_object = adapter.handle_request(
        "Start navigation to the toilet upstairs",
        start_room_id="room_1",
        start_frame_idx=0,
    )
    assert execute_object["status"] == "success"
    assert execute_object["parsed_request"]["request_type"] == "execute"
    assert execute_object["tool_selection"]["tool_name"] == "closed_loop_execute"
    assert execute_object["backend_result"]["outcome"]["outcome_category"] == "success"

    ambiguous_object = adapter.handle_request(
        "Go to the chair",
        start_room_id="room_1",
    )
    assert ambiguous_object["status"] == "ambiguous"
    assert ambiguous_object["resolved_target"]["failure_reason"] == "object_target_ambiguous"

    unsupported = adapter.handle_request(
        "Explore every floor and choose the nicest room.",
        start_room_id="room_1",
    )
    assert unsupported["status"] == "unsupported"
    assert unsupported["parsed_request"]["unsupported_reason"] == "unsupported_request"

    print("VLN tool-use layer validated.")


if __name__ == "__main__":
    run_mock_test()
