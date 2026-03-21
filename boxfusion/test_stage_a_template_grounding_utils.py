import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.stage_a_template_grounding_utils import (
    build_acceptance_case_result,
    build_export_metadata_summary,
    build_minimal_vln_demo_payload,
    choose_default_start_room_id,
    classify_acceptance_result,
    infer_instruction_family,
)


class _FakeTopology:
    sequence_id = "fake_sequence"

    def __init__(self, rooms: List[Dict[str, Any]], object_labels: List[str], anchor_ids: List[str]) -> None:
        self._rooms = {room["id"]: dict(room) for room in rooms}
        self._object_labels = list(object_labels)
        self._anchor_ids = list(anchor_ids)

    def inspect_export(self, sample_limit: int = 5) -> Dict[str, Any]:
        room_ids = self.list_room_ids()
        return {
            "room_count": len(room_ids),
            "anchor_count": len(self._anchor_ids),
            "object_count": len(self._object_labels),
            "object_label_count": len(self._object_labels),
            "room_ids": room_ids,
            "anchor_ids": list(self._anchor_ids),
            "object_ids": [f"obj_{index}" for index, _ in enumerate(self._object_labels, start=1)],
            "object_labels": list(self._object_labels),
            "sample_room_ids": room_ids[:sample_limit],
            "sample_anchor_ids": self._anchor_ids[:sample_limit],
            "sample_object_ids": [f"obj_{index}" for index, _ in enumerate(self._object_labels[:sample_limit], start=1)],
            "sample_object_labels": self._object_labels[:sample_limit],
            "diagnostics": [],
        }

    def list_room_ids(self) -> List[str]:
        return sorted(self._rooms)

    def get_room(self, room_id: str) -> Dict[str, Any]:
        return dict(self._rooms[room_id])

    def list_object_labels(self) -> List[str]:
        return list(self._object_labels)

    def list_anchor_ids(self, valid_only: bool = False) -> List[str]:
        return list(self._anchor_ids)


def run_mock_test() -> None:
    topology = _FakeTopology(
        rooms=[
            {"id": "room_1", "room_type": "unknown"},
            {"id": "room_2", "room_type": "living_room"},
        ],
        object_labels=[],
        anchor_ids=[],
    )
    export_summary = build_export_metadata_summary(topology)
    assert export_summary["room_type_summary"]["usable_room_type_count"] == 1
    assert export_summary["capabilities"]["semantic_room_target"]["available"]
    assert not export_summary["capabilities"]["object_label_target"]["available"]
    assert not export_summary["capabilities"]["anchor_id_target"]["available"]

    start_room_id, auto_selected = choose_default_start_room_id(topology)
    assert start_room_id == "room_1"
    assert auto_selected

    assert infer_instruction_family("go to room_7") == "explicit_room_target"
    assert infer_instruction_family("go to kitchen") == "semantic_room_target"
    assert infer_instruction_family("go to the room with sofa") == "object_label_target"
    assert infer_instruction_family("go to anchor anchor_5") == "anchor_id_target"
    assert infer_instruction_family("go from room_1 to room_2") == "room_to_room"
    assert infer_instruction_family("find sofa") == "unsupported_template"

    unsupported_export_result = {
        "ok": False,
        "parse": {
            "template_id": "go_to_anchor",
            "intent": "go_to_anchor",
            "raw_slots": {"anchor_id": "anchor_1"},
            "normalized_slots": {"anchor_id": "anchor_1"},
            "failure_reason": None,
            "notes": [],
        },
        "grounding": {},
        "query_api_request": {"method": "query_route_to_anchor", "kwargs": {}},
        "query_api_result": {
            "found": False,
            "route": {"attempted": False},
            "target_resolution": {"failure_reason": "anchor_not_found", "notes": ["Anchor anchor_1 is not present."]},
            "explanation": {"target_summary": "Anchor lookup failed."},
        },
        "failure_stage": "query_api_resolution",
        "failure_reason": "ANCHOR_ID_UNRESOLVED",
        "explanation": "Matched anchor template, but grounding failed downstream.",
    }
    assert (
        classify_acceptance_result(
            result=unsupported_export_result,
            family="anchor_id_target",
            export_summary=export_summary,
        )
        == "UNSUPPORTED_ON_CURRENT_EXPORT"
    )

    case_result = build_acceptance_case_result(
        case_id="case_001",
        case={
            "instruction": "go to anchor anchor_1",
            "family": "anchor_id_target",
            "start_room_id": "room_1",
            "route_policy": "balanced",
        },
        result=unsupported_export_result,
        export_summary=export_summary,
    )
    assert case_result["result_category"] == "UNSUPPORTED_ON_CURRENT_EXPORT"
    assert not case_result["passed"]

    pass_result = {
        "ok": True,
        "parse": {
            "template_id": "go_to_room",
            "intent": "go_to_room",
            "raw_slots": {"room": "room_2"},
            "normalized_slots": {"room": "room_2"},
            "failure_reason": None,
            "notes": [],
        },
        "grounding": {
            "grounded_target_type": "room",
            "grounded_target": {"input_value": "room_2", "normalized_value": "room_2"},
            "resolved_start_room_id": "room_1",
            "resolved_goal_room_id": "room_2",
        },
        "query_api_request": {"method": "query_route", "kwargs": {}},
        "query_api_result": {
            "found": True,
            "start_room_id": "room_1",
            "resolved_goal_room_id": "room_2",
            "route_policy": "balanced",
            "target_resolution": {
                "resolved_room_id": "room_2",
                "matched_entity_ids": ["room_2"],
                "notes": ["Goal room room_2 can be used directly for routing."],
            },
            "route": {
                "attempted": True,
                "hop_count": 1,
                "room_sequence": ["room_1", "room_2"],
                "used_relation_types": ["adjacent"],
                "route_confidence": 0.82,
                "total_cost": 1.2,
                "edges": [
                    {
                        "source_room_id": "room_1",
                        "target_room_id": "room_2",
                        "relation_type": "adjacent",
                        "confidence": 0.82,
                        "edge_cost": 1.2,
                    }
                ],
            },
            "explanation": {
                "target_summary": "Room target room_2 resolved directly.",
                "summary": "Route from room_1 to room_2 found under the balanced policy.",
            },
        },
        "failure_stage": None,
        "failure_reason": None,
        "explanation": "Matched 'go_to_room' and delegated to the existing Query API.",
    }

    demo_payload = build_minimal_vln_demo_payload(
        instruction="go to room_2",
        start_room_id="room_1",
        route_policy="balanced",
        result=pass_result,
        export_summary=export_summary,
    )
    assert demo_payload["result_category"] == "PASS"
    assert demo_payload["route_summary"]["room_sequence"] == ["room_1", "room_2"]
    assert demo_payload["route_summary"]["steps"]
    assert "Computed room-level route" in demo_payload["teacher_explanation"]

    print("Stage A template grounding utils validated.")


if __name__ == "__main__":
    run_mock_test()
