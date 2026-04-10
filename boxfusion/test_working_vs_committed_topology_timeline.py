import copy
import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.working_vs_committed_topology_timeline import build_working_vs_committed_timeline


def _refresh_room(
    room_id: str,
    *,
    lifecycle_state: str,
    present_in_latest_export: bool,
    candidate_complete: bool = False,
    commit_block_reasons: List[str] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "candidate_complete": candidate_complete,
        "commit_block_reasons": list(commit_block_reasons or []),
        "present_in_latest_export": present_in_latest_export,
    }


def _final_room(
    room_id: str,
    *,
    lifecycle_state: str,
    present_in_latest_export: bool,
    candidate_complete: bool = False,
    commit_block_reasons: List[str] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "candidate_complete": candidate_complete,
        "commit_block_reasons": list(commit_block_reasons or []),
        "present_in_latest_export": present_in_latest_export,
    }


def _payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "withheld_timeline_sequence",
        "frame_idx": 30,
        "timestamp": 3.0,
        "summary": {
            "refresh_count": 3,
            "public_topology_export_succeeded": True,
        },
        "committed_rooms": ["room_1", "room_3"],
        "rooms": [
            _final_room("room_1", lifecycle_state="committed", present_in_latest_export=True),
            _final_room(
                "room_2",
                lifecycle_state="active",
                present_in_latest_export=True,
                commit_block_reasons=["room_signature_not_stable"],
            ),
            _final_room("room_3", lifecycle_state="committed", present_in_latest_export=False),
        ],
        "refresh_history": [
            {
                "frame_idx": 0,
                "timestamp": 0.0,
                "rooms": [
                    _refresh_room(
                        "room_1",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_currently_active", "room_signature_not_stable"],
                    ),
                ],
            },
            {
                "frame_idx": 10,
                "timestamp": 1.0,
                "rooms": [
                    _refresh_room(
                        "room_1",
                        lifecycle_state="merge_or_split_pending",
                        present_in_latest_export=True,
                        candidate_complete=True,
                        commit_block_reasons=["gateway_structure_not_stable"],
                    ),
                    _refresh_room(
                        "room_2",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                    _refresh_room(
                        "room_3",
                        lifecycle_state="candidate_complete",
                        present_in_latest_export=True,
                        candidate_complete=True,
                        commit_block_reasons=[],
                    ),
                ],
            },
            {
                "frame_idx": 20,
                "timestamp": 2.0,
                "rooms": [
                    _refresh_room(
                        "room_1",
                        lifecycle_state="candidate_complete",
                        present_in_latest_export=True,
                        candidate_complete=True,
                        commit_block_reasons=[],
                    ),
                    _refresh_room(
                        "room_2",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                    _refresh_room(
                        "room_3",
                        lifecycle_state="candidate_complete",
                        present_in_latest_export=False,
                        candidate_complete=True,
                        commit_block_reasons=[],
                    ),
                ],
            },
        ],
    }


def test_temporal_summary_tracks_working_before_terminal_commit() -> None:
    payload = _payload()
    original_payload = copy.deepcopy(payload)

    summary = build_working_vs_committed_timeline(payload)
    room_1 = next(item for item in summary["rooms"] if item["room_id"] == "room_1")

    assert payload == original_payload
    assert room_1["first_present_in_working_frame"] == 0
    assert room_1["first_candidate_complete_frame"] == 10
    assert room_1["first_commit_ready_frame"] == 20
    assert room_1["first_committed_frame"] == 30
    assert room_1["total_working_refreshes"] == 3
    assert room_1["total_withheld_refreshes"] == 3
    assert room_1["total_committed_refreshes"] == 0
    assert room_1["was_working_only_then_later_committed"] is True
    assert room_1["remained_working_only_to_end"] is False
    assert room_1["oscillated_lifecycle_state"] is False
    assert room_1["dominant_withhold_blockers"][0]["blocker"] == "commit_ready_pending_publication"
    assert room_1["blocker_transition_summary"]["transition_count"] == 2


def test_temporal_summary_tracks_room_withheld_to_end() -> None:
    summary = build_working_vs_committed_timeline(_payload())
    room_2 = next(item for item in summary["rooms"] if item["room_id"] == "room_2")

    assert room_2["first_present_in_working_frame"] == 10
    assert room_2["first_commit_ready_frame"] is None
    assert room_2["first_committed_frame"] is None
    assert room_2["total_working_refreshes"] == 2
    assert room_2["total_withheld_refreshes"] == 2
    assert room_2["remained_working_only_to_end"] is True
    assert room_2["dominant_withhold_blockers"] == [
        {"blocker": "room_signature_not_stable", "refresh_count": 2}
    ]


def test_lifecycle_committed_without_terminal_projection_membership_is_not_counted_as_committed() -> None:
    summary = build_working_vs_committed_timeline(_payload())
    room_3 = next(item for item in summary["rooms"] if item["room_id"] == "room_3")
    scene_summary = summary["scene_summary"]

    assert room_3["final_lifecycle_state"] == "committed"
    assert room_3["final_committed_projection_member"] is False
    assert room_3["first_committed_frame"] is None
    assert scene_summary["terminal_projection"]["working_room_count"] == 2
    assert scene_summary["terminal_projection"]["committed_room_count"] == 1
    assert scene_summary["terminal_projection"]["withheld_room_count"] == 1
    assert scene_summary["rooms_ever_working_only_count"] == 3
    assert scene_summary["rooms_working_only_then_committed_count"] == 1
    assert scene_summary["rooms_remaining_working_only_to_end_count"] == 1


if __name__ == "__main__":
    test_temporal_summary_tracks_working_before_terminal_commit()
    test_temporal_summary_tracks_room_withheld_to_end()
    test_lifecycle_committed_without_terminal_projection_membership_is_not_counted_as_committed()
    print("ok")
