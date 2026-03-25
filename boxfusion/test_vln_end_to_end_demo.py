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
from boxfusion.vln_end_to_end_demo import VLNEndToEndDemoOrchestrator


def _mock_topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_end_to_end_vln_demo",
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
    orchestrator = VLNEndToEndDemoOrchestrator(query_api=query_api, executor=executor)

    explain_record = orchestrator.run_nl_request(
        "How do I get to the bedroom upstairs?",
        sequence_id="mock_seq",
        start_room_id="room_1",
    )
    assert explain_record["request_source"] == "nl"
    assert explain_record["request_type"] == "explain"
    assert explain_record["target_type"] == "room"
    assert explain_record["selected_backend_call"]["tool_name"] == "query_route"
    assert explain_record["floor_switch_summary"]["switch_required"] is True
    assert explain_record["vertical_transition_summary"]["used"] is True
    assert explain_record["final_outcome"]["status"] == "success"

    execute_record = orchestrator.run_nl_request(
        "Go to anchor_obj_201 upstairs.",
        sequence_id="mock_seq",
        start_room_id="room_1",
        start_frame_idx=0,
    )
    assert execute_record["request_type"] == "execute"
    assert execute_record["target_type"] == "anchor"
    assert execute_record["selected_backend_call"]["tool_name"] == "closed_loop_execute"
    assert execute_record["final_outcome"]["outcome_category"] == "success"
    assert execute_record["execution_trace"]["event_count"] > 0

    ambiguous_record = orchestrator.run_nl_request(
        "Go to the chair.",
        sequence_id="mock_seq",
        start_room_id="room_1",
    )
    assert ambiguous_record["final_outcome"]["status"] == "ambiguous"
    assert ambiguous_record["resolved_target"]["failure_reason"] == "object_target_ambiguous"

    structured_record = orchestrator.run_structured_task(
        {
            "request_type": "execute",
            "start_room_id": "room_1",
            "start_frame_idx": 0,
            "target": {
                "target_type": "object",
                "object_id": "obj_201",
                "object_label": "toilet",
            },
        },
        sequence_id="mock_seq",
    )
    assert structured_record["request_source"] == "structured"
    assert structured_record["request_type"] == "execute"
    assert structured_record["target_type"] == "object"
    assert structured_record["selected_backend_call"]["tool_name"] == "closed_loop_execute"
    assert structured_record["final_outcome"]["outcome_category"] == "success"
    assert structured_record["resolved_target"]["resolved_room_id"] == "room_3"

    print("End-to-end VLN demo orchestration validated.")


if __name__ == "__main__":
    run_mock_test()
