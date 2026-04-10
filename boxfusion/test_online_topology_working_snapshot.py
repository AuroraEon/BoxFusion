import copy
import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.online_topology_lifecycle import RoomLifecycleState
from boxfusion.online_topology_working_snapshot import (
    build_working_topology_snapshot,
    build_working_vs_committed_report,
)


def _room(room_id: int, x0: float) -> Dict[str, Any]:
    return {
        "id": f"room_{room_id}",
        "room_uuid": room_id,
        "room_type": "unknown",
        "floor_id": "floor_1",
        "floor_index": 0,
        "display_floor_id": "floor_1",
        "display_order": 0,
        "floor_assignment_confidence": 1.0,
        "status": "confirmed",
        "center": [x0 + 1.0, 1.0],
        "polygon": [[x0, 0.0], [x0 + 2.0, 0.0], [x0 + 2.0, 2.0], [x0, 2.0]],
        "area_m2": 4.0,
    }


def _base_topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_sequence",
        "floors": [
            {
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 0,
                "floor_index": 0,
                "status": "stable",
            }
        ],
        "rooms": [_room(1, 0.0), _room(2, 2.5), _room(3, 5.0), _room(4, 7.5)],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "adjacent",
                "confidence": 0.9,
                "status": "confirmed",
                "support_count": 2,
                "first_seen_step": 0,
                "last_seen_step": 10,
                "evidence_ids": ["ev_1"],
                "metadata": {"gateway_count": 1},
            },
            {
                "source": "room_2",
                "target": "room_3",
                "relation_type": "transition",
                "confidence": 0.8,
                "status": "confirmed",
                "support_count": 1,
                "first_seen_step": 10,
                "last_seen_step": 20,
                "evidence_ids": ["ev_2"],
                "metadata": {},
            },
            {
                "source": "room_3",
                "target": "room_4",
                "relation_type": "adjacent",
                "confidence": 0.5,
                "status": "supported",
                "support_count": 1,
                "first_seen_step": 20,
                "last_seen_step": 30,
                "evidence_ids": ["ev_3"],
                "metadata": {"gateway_count": 1},
            },
        ],
        "indices": {
            "object_to_room": {},
            "anchor_to_room": {},
            "room_to_objects": {},
            "room_to_anchors": {},
        },
        "entities": {"objects": [], "anchors": []},
        "evidences": [
            {"evidence_id": "ev_1", "evidence_type": "door_detection", "score": 1.0, "step_start": 0, "step_end": 10, "source_ref": "g1", "notes": "", "metadata": {}},
            {"evidence_id": "ev_2", "evidence_type": "trajectory_transition", "score": 1.0, "step_start": 10, "step_end": 20, "source_ref": "t1", "notes": "", "metadata": {}},
            {"evidence_id": "ev_3", "evidence_type": "boundary_contact", "score": 0.5, "step_start": 20, "step_end": 30, "source_ref": "g2", "notes": "", "metadata": {}},
        ],
        "metadata": {"builder_version": "0.1"},
        "query_examples": {"room_count": 4},
    }


def _lifecycle_room(
    room_id: str,
    *,
    lifecycle_state: str,
    present_in_latest_export: bool,
    candidate_complete: bool = False,
    commit_block_reasons: List[str] = None,
    candidate_block_reasons: List[str] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "dirty": lifecycle_state != RoomLifecycleState.COMMITTED.value,
        "dirty_reasons": [],
        "last_updated_frame_idx": 30,
        "last_updated_timestamp": 3.0,
        "first_seen_frame_idx": 0,
        "last_observed_frame_idx": 30,
        "last_departed_frame_idx": None,
        "export_observation_count": 3,
        "floor_id": "floor_1",
        "floor_status": "stable",
        "present_in_latest_export": present_in_latest_export,
        "room_signature_stability_count": 3,
        "gateway_signature_stability_count": 3,
        "containment_stability_count": 3,
        "candidate_complete": candidate_complete,
        "candidate_readiness_score": 4 if candidate_complete else 2,
        "candidate_complete_reasons": [],
        "candidate_block_reasons": list(candidate_block_reasons or []),
        "commit_block_reasons": list(commit_block_reasons or []),
        "stable_refresh_opportunities_since_structural_delta": 2,
        "last_structural_delta_frame_idx": None,
        "vertical_transition_statuses": [],
        "vertical_transition_ids": [],
        "tracking_summary": {},
        "trigger_counts": {},
        "last_room_signature_change_components": [],
        "room_signature_change_component_counts": {},
        "last_gateway_signature_change_components": [],
        "gateway_signature_change_component_counts": {},
        "recent_triggers": [],
    }


def _lifecycle_payload() -> Dict[str, Any]:
    rooms = [
        _lifecycle_room(
            "room_1",
            lifecycle_state=RoomLifecycleState.CANDIDATE_COMPLETE.value,
            present_in_latest_export=True,
            candidate_complete=True,
            commit_block_reasons=["gateway_structure_not_stable"],
        ),
        _lifecycle_room(
            "room_2",
            lifecycle_state=RoomLifecycleState.ACTIVE.value,
            present_in_latest_export=True,
            candidate_complete=False,
            commit_block_reasons=["room_currently_active"],
            candidate_block_reasons=["room_currently_active"],
        ),
        _lifecycle_room(
            "room_3",
            lifecycle_state=RoomLifecycleState.COMMITTED.value,
            present_in_latest_export=True,
        ),
        _lifecycle_room(
            "room_4",
            lifecycle_state=RoomLifecycleState.DISCOVERING.value,
            present_in_latest_export=True,
            commit_block_reasons=["room_signature_not_stable"],
            candidate_block_reasons=["room_signature_not_stable"],
        ),
    ]
    return {
        "version": "0.1",
        "sequence_id": "mock_sequence",
        "frame_idx": 30,
        "timestamp": 3.0,
        "summary": {},
        "rooms": rooms,
        "committed_rooms": ["room_3"],
    }


def test_working_topology_includes_candidate_complete_and_active_but_excludes_discovering() -> None:
    topology_payload = _base_topology_payload()
    lifecycle_payload = _lifecycle_payload()

    working_payload = build_working_topology_snapshot(
        topology_payload,
        lifecycle_payload,
        committed_room_ids=["room_3"],
        lifecycle_semantics="pre_finalize_working_view",
    )

    room_ids = [room["id"] for room in working_payload["rooms"]]
    assert room_ids == ["room_1", "room_2", "room_3"]
    assert working_payload["working_summary"]["candidate_complete_room_count"] == 1
    assert working_payload["working_summary"]["active_room_count"] == 1

    room_1 = next(room for room in working_payload["rooms"] if room["id"] == "room_1")
    room_2 = next(room for room in working_payload["rooms"] if room["id"] == "room_2")
    room_3 = next(room for room in working_payload["rooms"] if room["id"] == "room_3")
    assert room_1["debug_lifecycle"]["candidate_complete"] is True
    assert room_1["debug_lifecycle"]["provisional"] is True
    assert room_2["debug_lifecycle"]["lifecycle_state"] == RoomLifecycleState.ACTIVE.value
    assert room_2["debug_lifecycle"]["withheld_from_committed_projection"] is True
    assert room_3["debug_lifecycle"]["withheld_from_committed_projection"] is False


def test_comparison_report_identifies_working_only_rooms_without_mutating_public_topology() -> None:
    topology_payload = _base_topology_payload()
    original_topology_payload = copy.deepcopy(topology_payload)
    lifecycle_payload = _lifecycle_payload()

    working_payload = build_working_topology_snapshot(
        topology_payload,
        lifecycle_payload,
        committed_room_ids=["room_3"],
    )
    report = build_working_vs_committed_report(
        working_payload,
        topology_payload,
        lifecycle_payload,
        committed_room_ids=["room_3"],
    )

    assert topology_payload == original_topology_payload
    assert report["working_topology"]["room_count"] == 3
    assert report["committed_topology_projection"]["room_count"] == 1
    assert report["difference_summary"]["room_count_difference"] == 2
    assert report["difference_summary"]["edge_count_difference"] == 2
    assert report["difference_summary"]["gateway_count_difference"] == 1
    assert report["difference_summary"]["working_only_rooms"] == ["room_1", "room_2"]

    blocked_rooms = {room["room_id"]: room for room in report["rooms_blocked_from_commit"]}
    assert set(blocked_rooms) == {"room_1", "room_2"}
    assert blocked_rooms["room_1"]["provisional_class"] == "candidate_complete"
    assert blocked_rooms["room_2"]["lifecycle_state"] == RoomLifecycleState.ACTIVE.value
    assert report["withheld_topology_summary"]["commit_block_reason_counts"] == {
        "gateway_structure_not_stable": 1,
        "room_currently_active": 1,
    }


if __name__ == "__main__":
    test_working_topology_includes_candidate_complete_and_active_but_excludes_discovering()
    test_comparison_report_identifies_working_only_rooms_without_mutating_public_topology()
    print("ok")
