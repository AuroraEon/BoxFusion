import os
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, Dict, List, Optional

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.online_topology_lifecycle import OnlineTopologyLifecycleManager
from boxfusion.online_topology_timeline_eval import build_timeline_summary, load_json


def _build_vector_map(
    *,
    rooms: List[Dict[str, Any]],
    gateways: Optional[List[Dict[str, Any]]] = None,
    floors: Optional[List[Dict[str, Any]]] = None,
    frame_floor_assignments: Optional[List[Dict[str, Any]]] = None,
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
        "objects": [],
        "anchors": [],
        "vertical_transitions": [],
        "frame_floor_assignments": list(frame_floor_assignments or []),
        "vertical_transition_summary": {"count": 0},
        "room_floor_validation": {
            "room_count": int(len(rooms)),
            "spanning_room_count": 0,
            "spanning_rooms": [],
            "valid": True,
        },
    }


def _room(room_id: int, *, polygon: Optional[List[List[float]]] = None) -> Dict[str, Any]:
    return {
        "id": int(room_id),
        "room_id": f"room_{int(room_id)}",
        "floor_id": "floor_1",
        "room_type": "unknown",
        "status": "confirmed",
        "polygon": polygon or [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        "center": [1.0, 1.0],
        "area_m2": 4.0,
    }


def _candidate_then_commit_manager() -> OnlineTopologyLifecycleManager:
    manager = OnlineTopologyLifecycleManager(sequence_id="timeline_eval_sequence")

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
    return manager


def test_refresh_history_is_exported_and_summarized() -> None:
    manager = _candidate_then_commit_manager()
    payload = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=False,
    )

    assert payload["summary"]["refresh_count"] == 3
    assert len(payload["refresh_history"]) == 3

    summary = build_timeline_summary(payload)
    room_1 = next(item for item in summary["rooms"] if item["room_id"] == "room_1")

    assert summary["history_mode"] == "refresh_history"
    assert room_1["first_seen_frame"] == 0
    assert room_1["first_candidate_formed_frame"] == 10
    assert room_1["first_candidate_complete_frame"] == 10
    assert room_1["first_finalized_private_frame"] == 20
    assert room_1["first_commit_ready_frame"] == 20
    assert room_1["first_committed_frame"] is None
    assert room_1["final_publication_state"] == "COMMIT_READY"
    assert room_1["publication_state_path"] == [
        "ACTIVE_OBSERVING",
        "FINALIZATION_PENDING",
        "COMMIT_READY",
    ]
    assert room_1["candidate_but_blocked_refresh_count"] == 1
    assert room_1["merge_or_split_pending_refresh_count"] == 1
    assert room_1["last_blocker_set_before_commit"] == [
        "gateway_structure_not_stable",
        "merge_or_split_pending",
    ]


def test_committed_frame_is_reported_from_finalized_artifact() -> None:
    manager = _candidate_then_commit_manager()

    with TemporaryDirectory() as tmp_dir:
        out_path = Path(tmp_dir) / "online_topology_lifecycle_v0_1.json"
        manager.export_json(
            out_path,
            frame_idx=20,
            timestamp=2.0,
            public_topology_export_succeeded=True,
        )
        payload = load_json(out_path)

    summary = build_timeline_summary(payload)
    room_1 = next(item for item in summary["rooms"] if item["room_id"] == "room_1")

    assert room_1["final_lifecycle_state"] == "committed"
    assert room_1["final_publication_state"] == "PUBLISHED"
    assert room_1["first_commit_ready_frame"] == 20
    assert room_1["first_committed_frame"] == 20
    assert room_1["final_blocker_set"] == []


def test_final_only_artifact_falls_back_conservatively() -> None:
    manager = _candidate_then_commit_manager()
    payload = manager.finalize_report(
        frame_idx=20,
        timestamp=2.0,
        public_topology_export_succeeded=True,
    )
    payload.pop("refresh_history", None)
    payload["summary"].pop("refresh_count", None)

    summary = build_timeline_summary(payload)
    room_1 = next(item for item in summary["rooms"] if item["room_id"] == "room_1")

    assert summary["history_mode"] == "final_report_only"
    assert summary["history_limitations"]
    assert room_1["first_seen_frame"] == 0
    assert room_1["first_candidate_formed_frame"] is None
    assert room_1["first_candidate_complete_frame"] is None
    assert room_1["first_finalized_private_frame"] is None
    assert room_1["first_commit_ready_frame"] is None
    assert room_1["first_committed_frame"] == 20
    assert room_1["final_publication_state"] == "PUBLISHED"


def run_mock_test() -> None:
    print("Running online topology timeline evaluation tests...")
    test_refresh_history_is_exported_and_summarized()
    test_committed_frame_is_reported_from_finalized_artifact()
    test_final_only_artifact_falls_back_conservatively()
    print("Online topology timeline evaluation tests passed.")


if __name__ == "__main__":
    run_mock_test()
