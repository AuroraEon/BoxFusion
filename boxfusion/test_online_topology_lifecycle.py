import os
import sys
from typing import Any, Dict, List, Optional

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.online_topology_lifecycle import (
    CandidateBlockReason,
    CommitBlockReason,
    OnlineTopologyLifecycleManager,
    RoomLifecycleState,
)


def _build_vector_map(
    *,
    rooms: List[Dict[str, Any]],
    gateways: Optional[List[Dict[str, Any]]] = None,
    objects: Optional[List[Dict[str, Any]]] = None,
    anchors: Optional[List[Dict[str, Any]]] = None,
    floors: Optional[List[Dict[str, Any]]] = None,
    frame_floor_assignments: Optional[List[Dict[str, Any]]] = None,
    vertical_transitions: Optional[List[Dict[str, Any]]] = None,
    room_floor_validation: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "floors": list(
            floors
            or [
                {
                    "floor_id": "floor_1",
                    "display_floor_id": "floor_1",
                    "display_order": 1,
                    "status": "stable",
                }
            ]
        ),
        "rooms": list(rooms),
        "gateways": list(gateways or []),
        "objects": list(objects or []),
        "anchors": list(anchors or []),
        "vertical_transitions": list(vertical_transitions or []),
        "frame_floor_assignments": list(frame_floor_assignments or []),
        "vertical_transition_summary": {
            "count": int(len(vertical_transitions or [])),
        },
        "room_floor_validation": dict(
            room_floor_validation
            or {
                "room_count": int(len(rooms)),
                "spanning_room_count": 0,
                "spanning_rooms": [],
                "valid": True,
            }
        ),
    }


def _room(
    room_id: int,
    *,
    floor_id: str = "floor_1",
    room_type: str = "unknown",
    status: str = "confirmed",
    polygon: Optional[List[List[float]]] = None,
    center: Optional[List[float]] = None,
    area_m2: Optional[float] = None,
) -> Dict[str, Any]:
    polygon = polygon or [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    return {
        "id": int(room_id),
        "room_id": f"room_{int(room_id)}",
        "floor_id": floor_id,
        "room_type": room_type,
        "status": status,
        "polygon": polygon,
        "center": center or [1.0, 1.0],
        "area_m2": 4.0 if area_m2 is None else float(area_m2),
    }


def test_candidate_complete_rooms_can_be_marked_committed_in_debug_report() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="mock_sequence")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_2"},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )

    pre_finalize = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=False,
    )
    room_1_before = next(item for item in pre_finalize["rooms"] if item["room_id"] == "room_1")
    room_2_before = next(item for item in pre_finalize["rooms"] if item["room_id"] == "room_2")
    assert room_1_before["candidate_complete"] is True
    assert room_1_before["lifecycle_state"] == RoomLifecycleState.CANDIDATE_COMPLETE.value
    assert room_1_before["candidate_readiness_score"] == 5
    assert CommitBlockReason.ROOM_CURRENTLY_ACTIVE.value not in room_1_before["commit_block_reasons"]
    assert room_2_before["candidate_complete"] is False
    assert CandidateBlockReason.ROOM_CURRENTLY_ACTIVE.value in room_2_before["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_CURRENTLY_ACTIVE.value in room_2_before["commit_block_reasons"]
    assert pre_finalize["summary"]["candidate_complete_room_count"] == 1
    assert pre_finalize["summary"]["candidate_complete_room_count_pre_finalize"] == 1
    assert pre_finalize["summary"]["commit_ready_room_count_pre_finalize"] == 1

    finalized = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=True,
    )
    room_1_after = next(item for item in finalized["rooms"] if item["room_id"] == "room_1")
    assert room_1_after["lifecycle_state"] == RoomLifecycleState.COMMITTED.value
    assert room_1_after["dirty"] is False
    assert finalized["summary"]["candidate_complete_room_count"] == 0
    assert finalized["summary"]["candidate_complete_room_count_pre_finalize"] == 1
    assert finalized["summary"]["commit_ready_room_count_pre_finalize"] == 1
    assert "room_1" in finalized["committed_rooms"]
    assert "room_2" in finalized["blocked_commit_reasons"]


def test_merge_pending_and_partial_vertical_transition_block_commit() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="blocked_sequence")
    vector_map = _build_vector_map(
        rooms=[_room(1)],
        vertical_transitions=[
            {
                "transition_id": "vt_1",
                "from_room_id": 1,
                "to_room_id": None,
                "status": "partial_room_association",
            }
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
        room_floor_validation={
            "room_count": 1,
            "spanning_room_count": 1,
            "spanning_rooms": [{"room_id": 1, "floors": ["floor_1", "floor_2"]}],
            "valid": False,
        },
    )
    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map,
        segmentation_updated=True,
        export_profile={
            "changed_room_structure_changed_ids": "room_1",
            "changed_room_object_delta_ids": "room_1",
        },
    )

    report = manager.finalize_report(
        frame_idx=0,
        timestamp=0.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["dirty"] is True
    assert room_1["lifecycle_state"] == RoomLifecycleState.MERGE_OR_SPLIT_PENDING.value
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value in room_1["commit_block_reasons"]
    assert CommitBlockReason.VERTICAL_TRANSITION_PARTIAL.value in room_1["commit_block_reasons"]
    assert CommitBlockReason.ROOM_FLOOR_VALIDATION_FAILED.value in room_1["commit_block_reasons"]


def test_candidate_complete_can_emerge_before_commit_when_only_gateway_merge_blocks_remain() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="candidate_vs_commit")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_structure_changed_ids": "room_1|room_2"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["candidate_complete"] is True
    assert room_1["lifecycle_state"] == RoomLifecycleState.MERGE_OR_SPLIT_PENDING.value
    assert room_1["candidate_readiness_score"] == 4
    assert room_1["candidate_block_reasons"] == []
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value in room_1["commit_block_reasons"]
    assert room_1["stable_refresh_opportunities_since_structural_delta"] == 0
    assert room_1["last_structural_delta_frame_idx"] == 10


def test_merge_pending_clears_after_repeated_stable_refreshes() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="merge_pending_decay")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_structure_changed_ids": "room_1|room_2"},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["candidate_complete"] is True
    assert room_1["lifecycle_state"] == RoomLifecycleState.CANDIDATE_COMPLETE.value
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value not in room_1["commit_block_reasons"]
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["stable_refresh_opportunities_since_structural_delta"] == 1
    assert room_1["last_structural_delta_frame_idx"] == 10


def test_repeated_room_signature_deltas_still_relatch_merge_pending_after_stable_window() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="repeated_signature_relatch")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_structure_changed_ids": "room_1|room_2"},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=30,
        timestamp=3.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )
    vector_map_c = _build_vector_map(
        rooms=[
            _room(1, polygon=[[0.0, 0.0], [2.3, 0.0], [2.3, 2.0], [0.0, 2.0]], center=[1.15, 1.0], area_m2=4.6),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 40, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_d = _build_vector_map(
        rooms=[
            _room(1, polygon=[[0.0, 0.0], [2.6, 0.0], [2.6, 2.0], [0.0, 2.0]], center=[1.3, 1.0], area_m2=5.2),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 50, "floor_id": "floor_1", "status": "stable"}],
    )
    manager.observe_export(
        frame_idx=40,
        timestamp=4.0,
        vector_map=vector_map_c,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=50,
        timestamp=5.0,
        vector_map=vector_map_d,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=50,
        timestamp=5.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["candidate_complete"] is False
    assert room_1["lifecycle_state"] == RoomLifecycleState.MERGE_OR_SPLIT_PENDING.value
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value in room_1["commit_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["stable_refresh_opportunities_since_structural_delta"] == 0
    assert room_1["last_structural_delta_frame_idx"] == 50
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "room_signature_changed"
    assert last_trigger["details"]["merge_pending_relatch"] is True
    assert last_trigger["details"]["stable_refresh_opportunities_before_change"] == 0


def test_late_room_signature_flicker_does_not_relatch_merge_pending_after_stable_window() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="late_room_signature_flicker")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_c = _build_vector_map(
        rooms=[
            _room(1, polygon=[[0.0, 0.0], [2.3, 0.0], [2.3, 2.0], [0.0, 2.0]], center=[1.15, 1.0], area_m2=4.6),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        frame_floor_assignments=[{"frame_idx": 50, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_structure_changed_ids": "room_1|room_2"},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=30,
        timestamp=3.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=40,
        timestamp=4.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=50,
        timestamp=5.0,
        vector_map=vector_map_c,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=50,
        timestamp=5.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["candidate_complete"] is False
    assert room_1["lifecycle_state"] == RoomLifecycleState.ACTIVE.value
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value not in room_1["commit_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["stable_refresh_opportunities_since_structural_delta"] == 0
    assert room_1["last_structural_delta_frame_idx"] == 10
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "room_signature_changed"
    assert last_trigger["details"]["merge_pending_relatch"] is False
    assert last_trigger["details"]["stable_refresh_opportunities_before_change"] == 3

    finalized = manager.finalize_report(
        frame_idx=50,
        timestamp=5.0,
        public_topology_export_succeeded=True,
    )
    room_1_after = next(item for item in finalized["rooms"] if item["room_id"] == "room_1")
    assert room_1_after["lifecycle_state"] != RoomLifecycleState.COMMITTED.value


def test_late_gateway_signature_flicker_does_not_relatch_merge_pending_after_stable_window() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="late_gateway_signature_flicker")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.35, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 40, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1|room_2"},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_a,
        segmentation_updated=False,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_a,
        segmentation_updated=False,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=30,
        timestamp=3.0,
        vector_map=vector_map_a,
        segmentation_updated=False,
        export_profile={},
    )
    manager.observe_export(
        frame_idx=40,
        timestamp=4.0,
        vector_map=vector_map_b,
        segmentation_updated=False,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=40,
        timestamp=4.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["candidate_complete"] is True
    assert room_1["lifecycle_state"] == RoomLifecycleState.CANDIDATE_COMPLETE.value
    assert CommitBlockReason.MERGE_OR_SPLIT_PENDING.value not in room_1["commit_block_reasons"]
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["stable_refresh_opportunities_since_structural_delta"] == 0
    assert room_1["last_structural_delta_frame_idx"] is None
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "gateway_structure_changed"
    assert last_trigger["details"]["merge_pending_relatch"] is False
    assert last_trigger["details"]["stable_refresh_opportunities_before_change"] == 4


def test_small_room_geometry_jitter_does_not_reset_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_jitter")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[0.0, 0.0], [2.04, 0.0], [2.04, 2.03], [0.0, 2.03]],
                center=[1.04, 1.03],
                area_m2=4.07,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 2
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 0
    assert room_1["room_signature_change_component_counts"] == {}


def test_small_jagged_polygon_perturbation_and_vertex_order_noise_do_not_reset_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_polygon_jaggedness")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
                center=[2.0, 2.0],
                area_m2=16.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[4.0, 0.0], [4.0, 1.0], [4.1, 1.1], [4.0, 1.2], [4.0, 4.0], [0.0, 4.0], [0.0, 0.0]],
                center=[2.0, 2.0],
                area_m2=16.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 2
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 0
    assert room_1["room_signature_change_component_counts"] == {}


def test_one_bucket_polygon_bbox_toggle_with_single_vertex_swap_does_not_reset_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_bbox_toggle_residual")

    polygon_a = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 3.0], [0.0, 2.0]]
    polygon_b = [[0.0, 0.0], [4.0, 0.0], [4.1, 2.0], [2.0, 3.0], [0.0, 2.0]]

    vector_map_a = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_a,
                center=[2.0, 1.4],
                area_m2=10.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_b,
                center=[2.0, 1.4],
                area_m2=10.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 2
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 0
    assert room_1["room_signature_change_component_counts"] == {}


def test_small_room_area_scalar_noise_does_not_reset_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_area_scalar_noise")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[0.0, 0.0], [1.9, 0.0], [1.9, 1.1], [0.0, 1.1]],
                center=[0.95, 0.55],
                area_m2=2.09,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[0.0, 0.0], [1.9, 0.0], [1.9, 1.1], [0.0, 1.1]],
                center=[0.95, 0.55],
                area_m2=2.11,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 2
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 0
    assert room_1["room_signature_change_component_counts"] == {}


def test_two_bucket_polygon_bbox_change_still_resets_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_bbox_toggle_meaningful")

    polygon_a = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 3.0], [0.0, 2.0]]
    polygon_b = [[0.0, 0.0], [4.0, 0.0], [4.2, 2.0], [2.0, 3.0], [0.0, 2.0]]

    vector_map_a = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_a,
                center=[2.0, 1.4],
                area_m2=10.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_b,
                center=[2.0, 1.4],
                area_m2=10.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 1
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 1
    assert room_1["last_room_signature_change_components"] == ["polygon"]
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "room_signature_changed"
    assert last_trigger["details"]["room_signature_change_components"] == ["polygon"]
    assert last_trigger["details"]["room_signature_polygon_change"] == {
        "prev_vertex_count": 5,
        "next_vertex_count": 5,
        "prev_bbox": [0, 0, 40, 30],
        "next_bbox": [0, 0, 42, 30],
        "changed_vertex_bucket_count": 2,
    }


def test_meaningful_room_geometry_change_still_resets_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_structural_change")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=[[0.0, 0.0], [2.26, 0.0], [2.26, 2.0], [0.0, 2.0]],
                center=[1.13, 1.0],
                area_m2=4.52,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 1
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 1
    assert room_1["last_room_signature_change_components"] == ["polygon", "center", "area_m2"]
    assert room_1["room_signature_change_component_counts"] == {
        "area_m2": 1,
        "center": 1,
        "polygon": 1,
    }
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "room_signature_changed"
    assert last_trigger["details"]["room_signature_change_components"] == ["polygon", "center", "area_m2"]
    assert last_trigger["details"]["room_signature_polygon_change"] == {
        "prev_vertex_count": 4,
        "next_vertex_count": 4,
        "prev_bbox": [0, 0, 20, 20],
        "next_bbox": [0, 0, 23, 20],
        "changed_vertex_bucket_count": 4,
    }


def test_ignored_room_signature_polygon_residual_preserves_candidate_commit_separation() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_candidate_commit_separation")

    polygon_a = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 3.0], [0.0, 2.0]]
    polygon_b = [[0.0, 0.0], [4.0, 0.0], [4.1, 2.0], [2.0, 3.0], [0.0, 2.0]]

    vector_map_a = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_a,
                center=[2.0, 1.4],
                area_m2=10.0,
            )
        ],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(
                1,
                polygon=polygon_b,
                center=[2.0, 1.4],
                area_m2=10.0,
            ),
            _room(2, polygon=[[5.0, 0.0], [7.0, 0.0], [7.0, 2.0], [5.0, 2.0]], center=[6.0, 1.0], area_m2=4.0),
        ],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1,room_2"},
    )

    pre_finalize = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1_before = next(item for item in pre_finalize["rooms"] if item["room_id"] == "room_1")
    assert room_1_before["candidate_complete"] is True
    assert room_1_before["lifecycle_state"] == RoomLifecycleState.CANDIDATE_COMPLETE.value
    assert room_1_before["trigger_counts"].get("room_signature_changed", 0) == 0
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value not in room_1_before["commit_block_reasons"]
    assert "room_1" not in pre_finalize["committed_rooms"]

    finalized = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=True,
    )
    room_1_after = next(item for item in finalized["rooms"] if item["room_id"] == "room_1")
    assert room_1_after["lifecycle_state"] == RoomLifecycleState.COMMITTED.value
    assert "room_1" in finalized["committed_rooms"]


def test_small_gateway_position_jitter_does_not_reset_gateway_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="gateway_signature_jitter")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.14, 1.04]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=False,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["gateway_signature_stability_count"] == 2
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("gateway_structure_changed", 0) == 0
    assert room_1["last_gateway_signature_change_components"] == []
    assert room_1["gateway_signature_change_component_counts"] == {}


def test_meaningful_gateway_position_change_still_resets_gateway_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="gateway_signature_structural_change")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.31, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=False,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["gateway_signature_stability_count"] == 1
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("gateway_structure_changed", 0) == 1
    assert room_1["last_gateway_signature_change_components"] == ["pos_world"]
    assert room_1["gateway_signature_change_component_counts"] == {"pos_world": 1}
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "gateway_structure_changed"
    assert last_trigger["details"]["gateway_signature_change_components"] == ["pos_world"]
    assert last_trigger["details"]["gateway_signature_change_summary"] == {
        "prev_gateway_count": 1,
        "next_gateway_count": 1,
        "prev_other_room_ids": ["room_2"],
        "next_other_room_ids": ["room_2"],
        "prev_positions": [[21, 10]],
        "next_positions": [[23, 10]],
    }


def test_gateway_membership_change_still_resets_gateway_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="gateway_signature_membership_change")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
            _room(3, polygon=[[0.0, 2.2], [2.0, 2.2], [2.0, 4.2], [0.0, 4.2]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
            _room(3, polygon=[[0.0, 2.2], [2.0, 2.2], [2.0, 4.2], [0.0, 4.2]]),
        ],
        gateways=[{"connects": [1, 3], "type": "door", "pos_world": [1.00, 2.10]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=3, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=False,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["gateway_signature_stability_count"] == 1
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("gateway_structure_changed", 0) == 1
    assert room_1["last_gateway_signature_change_components"] == ["other_room_id", "connects", "pos_world"]
    assert room_1["gateway_signature_change_component_counts"] == {
        "connects": 1,
        "other_room_id": 1,
        "pos_world": 1,
    }
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["details"]["gateway_signature_change_summary"] == {
        "prev_gateway_count": 1,
        "next_gateway_count": 1,
        "prev_other_room_ids": ["room_2"],
        "next_other_room_ids": ["room_3"],
        "prev_positions": [[21, 10]],
        "next_positions": [[10, 21]],
    }


def test_gateway_signature_canonicalizes_connected_room_order_and_id_shape() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="gateway_signature_canonical_connects")

    vector_map_a = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": ["room_2", "room_1"], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=False,
        export_profile={},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["gateway_signature_stability_count"] == 2
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value not in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("gateway_structure_changed", 0) == 0
    assert room_1["last_gateway_signature_change_components"] == []
    assert room_1["gateway_signature_change_component_counts"] == {}


def test_ignored_gateway_position_jitter_preserves_candidate_commit_public_separation() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="gateway_signature_candidate_commit_separation")

    vector_map_a = _build_vector_map(
        rooms=[_room(1)],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.10, 1.00]}],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_c = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.14, 1.04]}],
        frame_floor_assignments=[{"frame_idx": 20, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_room_tracking(frame_idx=10, timestamp=1.0, current_room_id=2, source="snapshot")
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_2"},
    )
    manager.observe_export(
        frame_idx=20,
        timestamp=2.0,
        vector_map=vector_map_c,
        segmentation_updated=False,
        export_profile={},
    )

    pre_finalize = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=False,
    )
    room_1_before = next(item for item in pre_finalize["rooms"] if item["room_id"] == "room_1")
    assert room_1_before["candidate_complete"] is True
    assert room_1_before["lifecycle_state"] == RoomLifecycleState.CANDIDATE_COMPLETE.value
    assert CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE.value not in room_1_before["commit_block_reasons"]
    assert room_1_before["gateway_signature_stability_count"] == 2
    assert room_1_before["trigger_counts"].get("gateway_structure_changed", 0) == 1
    assert "room_1" not in pre_finalize["committed_rooms"]

    finalized = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=True,
    )
    room_1_after = next(item for item in finalized["rooms"] if item["room_id"] == "room_1")
    assert room_1_after["lifecycle_state"] == RoomLifecycleState.COMMITTED.value
    assert "room_1" in finalized["committed_rooms"]


def test_room_status_change_still_resets_room_signature_stability() -> None:
    manager = OnlineTopologyLifecycleManager(sequence_id="room_signature_status_change")

    vector_map_a = _build_vector_map(
        rooms=[_room(1, status="confirmed")],
        frame_floor_assignments=[{"frame_idx": 0, "floor_id": "floor_1", "status": "stable"}],
    )
    vector_map_b = _build_vector_map(
        rooms=[_room(1, status="tentative")],
        frame_floor_assignments=[{"frame_idx": 10, "floor_id": "floor_1", "status": "stable"}],
    )

    manager.observe_room_tracking(frame_idx=0, timestamp=0.0, current_room_id=1, source="snapshot")
    manager.observe_export(
        frame_idx=0,
        timestamp=0.0,
        vector_map=vector_map_a,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )
    manager.observe_export(
        frame_idx=10,
        timestamp=1.0,
        vector_map=vector_map_b,
        segmentation_updated=True,
        export_profile={"changed_room_ids": "room_1"},
    )

    report = manager.finalize_report(
        frame_idx=10,
        timestamp=1.0,
        public_topology_export_succeeded=False,
    )
    room_1 = next(item for item in report["rooms"] if item["room_id"] == "room_1")
    assert room_1["room_signature_stability_count"] == 1
    assert CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["candidate_block_reasons"]
    assert CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE.value in room_1["commit_block_reasons"]
    assert room_1["trigger_counts"].get("room_signature_changed", 0) == 1
    assert room_1["last_room_signature_change_components"] == ["status"]
    assert room_1["room_signature_change_component_counts"] == {"status": 1}
    last_trigger = room_1["recent_triggers"][-1]
    assert last_trigger["trigger"] == "room_signature_changed"
    assert last_trigger["details"]["room_signature_change_components"] == ["status"]


def test_room_topology_and_query_api_default_semantics_remain_snapshot_based() -> None:
    try:
        from boxfusion.query_api import RoomTopologyQueryAPI
        from boxfusion.room_topology import RoomTopologyBuilder
    except ModuleNotFoundError as exc:
        if exc.name != "networkx":
            raise
        print("Skipping snapshot-topology smoke test because networkx is not installed.")
        return

    vector_map = _build_vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        objects=[{"id": 101, "label": "table", "room_uuid": 2}],
        anchors=[{"id": "anchor_obj_101", "room_id": "room_2", "target_id": "obj_101", "anchor_type": "object"}],
    )
    topology = RoomTopologyBuilder().build(
        vector_map,
        sequence_id="snapshot_semantics_sequence",
        transition_history=[
            {"frame_idx": 0, "timestamp": 0.0, "current_room_id": 1},
            {"frame_idx": 1, "timestamp": 0.1, "current_room_id": 2},
        ],
        metadata={"test_case": "online_topology_defaults_do_not_change_public_query_path"},
    )
    query_api = RoomTopologyQueryAPI(topology)
    route = query_api.query_route_to_object("room_1", object_label="table")
    assert route["found"] is True
    assert route["resolved_goal_room_id"] == "room_2"
    assert route["route"]["used_relation_types"] == ["transition"]


def run_mock_test() -> None:
    print("Running online topology lifecycle validation...")
    test_candidate_complete_rooms_can_be_marked_committed_in_debug_report()
    test_merge_pending_and_partial_vertical_transition_block_commit()
    test_candidate_complete_can_emerge_before_commit_when_only_gateway_merge_blocks_remain()
    test_merge_pending_clears_after_repeated_stable_refreshes()
    test_repeated_room_signature_deltas_still_relatch_merge_pending_after_stable_window()
    test_late_room_signature_flicker_does_not_relatch_merge_pending_after_stable_window()
    test_late_gateway_signature_flicker_does_not_relatch_merge_pending_after_stable_window()
    test_small_room_geometry_jitter_does_not_reset_room_signature_stability()
    test_small_jagged_polygon_perturbation_and_vertex_order_noise_do_not_reset_room_signature_stability()
    test_one_bucket_polygon_bbox_toggle_with_single_vertex_swap_does_not_reset_room_signature_stability()
    test_small_room_area_scalar_noise_does_not_reset_room_signature_stability()
    test_two_bucket_polygon_bbox_change_still_resets_room_signature_stability()
    test_meaningful_room_geometry_change_still_resets_room_signature_stability()
    test_ignored_room_signature_polygon_residual_preserves_candidate_commit_separation()
    test_small_gateway_position_jitter_does_not_reset_gateway_signature_stability()
    test_meaningful_gateway_position_change_still_resets_gateway_signature_stability()
    test_gateway_membership_change_still_resets_gateway_signature_stability()
    test_gateway_signature_canonicalizes_connected_room_order_and_id_shape()
    test_ignored_gateway_position_jitter_preserves_candidate_commit_public_separation()
    test_room_status_change_still_resets_room_signature_stability()
    test_room_topology_and_query_api_default_semantics_remain_snapshot_based()
    print("Online topology lifecycle validated.")


if __name__ == "__main__":
    run_mock_test()
