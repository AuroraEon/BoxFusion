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


def _mock_topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_vln_closed_loop",
        "floors": [
            {"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
        "rooms": [
            {"id": "room_1", "floor_id": "floor_1", "display_floor_id": "floor_1", "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]]},
            {"id": "room_2", "floor_id": "floor_1", "display_floor_id": "floor_1", "polygon": [[1.1, 0], [2.1, 0], [2.1, 1], [1.1, 1]]},
            {"id": "room_3", "floor_id": "floor_2", "display_floor_id": "floor_2", "polygon": [[2.2, 0], [3.2, 0], [3.2, 1], [2.2, 1]]},
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
            "object_to_room": {"obj_9": "room_3"},
            "anchor_to_room": {"anchor_obj_9": "room_3"},
            "room_to_objects": {"room_1": [], "room_2": [], "room_3": ["obj_9"]},
            "room_to_anchors": {"room_1": [], "room_2": [], "room_3": ["anchor_obj_9"]},
        },
        "entities": {
            "objects": [
                {
                    "id": "obj_9",
                    "label": "chair",
                    "normalized_label": "chair",
                    "room_id": "room_3",
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                }
            ],
            "anchors": [
                {
                    "id": "anchor_obj_9",
                    "anchor_type": "object",
                    "room_id": "room_3",
                    "target_id": "obj_9",
                    "valid": True,
                    "floor_id": "floor_2",
                    "display_floor_id": "floor_2",
                }
            ],
        },
        "evidences": [],
        "metadata": {},
    }


def _success_observations() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": "room_1", "current_floor_id": "floor_1", "current_display_floor_id": "floor_1", "current_floor_label": "floor_1"},
        {"frame_idx": 1, "timestamp": 1.0, "current_room_id": "room_2", "current_floor_id": "floor_1", "current_display_floor_id": "floor_1", "current_floor_label": "floor_1"},
        {"frame_idx": 2, "timestamp": 2.0, "current_room_id": "room_3", "current_floor_id": "floor_2", "current_display_floor_id": "floor_2", "current_floor_label": "floor_2"},
    ]


def _missing_vertical_transition_observations() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": "room_1", "current_floor_id": "floor_1", "current_display_floor_id": "floor_1", "current_floor_label": "floor_1"},
        {"frame_idx": 1, "timestamp": 1.0, "current_room_id": "room_2", "current_floor_id": "floor_1", "current_display_floor_id": "floor_1", "current_floor_label": "floor_1"},
    ]


def run_mock_test() -> None:
    topology = RoomTopology.from_dict(_mock_topology_payload())
    query_api = RoomTopologyQueryAPI(topology)

    success_executor = VLNClosedLoopExecutor(query_api, _success_observations())
    success_result = success_executor.execute(
        {
            "start_room_id": "room_1",
            "target": {"target_type": "object", "object_id": "obj_9", "object_label": "chair"},
        },
        route_policy="balanced",
        start_frame_idx=0,
    )
    assert success_result["outcome"]["outcome_category"] == "success"
    plan_steps = ((success_result.get("plan") or {}).get("symbolic_plan") or {}).get("steps") or []
    assert any(step.get("action") == "use_vertical_transition" for step in plan_steps)
    assert success_result["outcome"]["completion_room_id"] == "room_3"

    missing_vt_executor = VLNClosedLoopExecutor(query_api, _missing_vertical_transition_observations())
    missing_vt_result = missing_vt_executor.execute(
        {
            "start_room_id": "room_1",
            "target": {"target_type": "room", "goal_room_id": "room_3"},
        },
        route_policy="balanced",
        start_frame_idx=0,
    )
    assert missing_vt_result["outcome"]["outcome_category"] == "failure"
    assert missing_vt_result["outcome"]["outcome_reason"] == "missing_vertical_transition"

    print("VLN closed-loop executor validated.")


if __name__ == "__main__":
    run_mock_test()
