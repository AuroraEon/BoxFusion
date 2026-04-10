import copy
import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.publication_candidate_survivability import build_publication_candidate_survivability


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
        "candidate_complete": bool(candidate_complete),
        "commit_block_reasons": list(commit_block_reasons or []),
        "present_in_latest_export": bool(present_in_latest_export),
    }


def _final_room(
    room_id: str,
    *,
    lifecycle_state: str,
    present_in_latest_export: bool,
    commit_block_reasons: List[str] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "present_in_latest_export": bool(present_in_latest_export),
        "commit_block_reasons": list(commit_block_reasons or []),
    }


def _policy_room(
    room_id: str,
    *,
    first_frame: int,
    actual_frame: int = None,
    publication_outcome: str,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "first_policy_qualification_frame": int(first_frame),
        "actual_committed_projection_frame": None if actual_frame is None else int(actual_frame),
        "first_simulated_publication_frame": int(first_frame),
        "simulated_publication_source": "policy_a",
        "publication_outcome": publication_outcome,
        "publication_lead_time_frames_vs_actual_committed_projection": None
        if actual_frame is None
        else int(actual_frame) - int(first_frame),
    }


def _payloads() -> Dict[str, Dict[str, Any]]:
    timeline_payload = {
        "sequence_id": "candidate_survivability_test_scene",
        "history_mode": "refresh_history",
        "input_frame_idx": 40,
        "input_timestamp": 4.0,
    }

    lifecycle_payload = {
        "sequence_id": "candidate_survivability_test_scene",
        "frame_idx": 40,
        "timestamp": 4.0,
        "rooms": [
            _final_room("room_stable", lifecycle_state="committed", present_in_latest_export=True),
            _final_room("room_changed", lifecycle_state="committed", present_in_latest_export=True),
            _final_room(
                "room_withheld",
                lifecycle_state="active",
                present_in_latest_export=True,
                commit_block_reasons=["gateway_structure_not_stable"],
            ),
            _final_room("room_gone", lifecycle_state="committed", present_in_latest_export=False),
        ],
        "refresh_history": [
            {
                "frame_idx": 0,
                "timestamp": 0.0,
                "rooms": [
                    _refresh_room(
                        "room_stable",
                        lifecycle_state="merge_or_split_pending",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                    _refresh_room(
                        "room_changed",
                        lifecycle_state="merge_or_split_pending",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                    _refresh_room(
                        "room_withheld",
                        lifecycle_state="merge_or_split_pending",
                        present_in_latest_export=True,
                        commit_block_reasons=["gateway_structure_not_stable"],
                    ),
                    _refresh_room(
                        "room_gone",
                        lifecycle_state="merge_or_split_pending",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                ],
            },
            {
                "frame_idx": 10,
                "timestamp": 1.0,
                "rooms": [
                    _refresh_room("room_stable", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room("room_changed", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room("room_withheld", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room("room_gone", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                ],
            },
            {
                "frame_idx": 20,
                "timestamp": 2.0,
                "rooms": [
                    _refresh_room("room_stable", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room(
                        "room_changed",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["room_signature_not_stable"],
                    ),
                    _refresh_room(
                        "room_withheld",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["gateway_structure_not_stable"],
                    ),
                    _refresh_room("room_gone", lifecycle_state="candidate_complete", present_in_latest_export=False),
                ],
            },
            {
                "frame_idx": 30,
                "timestamp": 3.0,
                "rooms": [
                    _refresh_room("room_stable", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room("room_changed", lifecycle_state="candidate_complete", present_in_latest_export=True, candidate_complete=True),
                    _refresh_room(
                        "room_withheld",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["gateway_structure_not_stable"],
                    ),
                    _refresh_room("room_gone", lifecycle_state="candidate_complete", present_in_latest_export=False),
                ],
            },
            {
                "frame_idx": 40,
                "timestamp": 4.0,
                "rooms": [
                    _refresh_room("room_stable", lifecycle_state="committed", present_in_latest_export=True),
                    _refresh_room("room_changed", lifecycle_state="committed", present_in_latest_export=True),
                    _refresh_room(
                        "room_withheld",
                        lifecycle_state="active",
                        present_in_latest_export=True,
                        commit_block_reasons=["gateway_structure_not_stable"],
                    ),
                    _refresh_room("room_gone", lifecycle_state="committed", present_in_latest_export=False),
                ],
            },
        ],
        "trigger_history": [
            {"frame_idx": 20, "timestamp": 2.0, "room_id": "room_changed", "trigger": "room_signature_changed"},
            {"frame_idx": 20, "timestamp": 2.0, "room_id": "room_withheld", "trigger": "gateway_structure_changed"},
            {"frame_idx": 20, "timestamp": 2.0, "room_id": "room_gone", "trigger": "room_removed"},
        ],
    }

    publication_policy_payload = {
        "sequence_id": "candidate_survivability_test_scene",
        "history_mode": "refresh_history",
        "input_frame_idx": 40,
        "input_timestamp": 4.0,
        "policy_results": [
            {
                "policy": {"policy_id": "policy_a", "family": "test"},
                "rooms": [
                    _policy_room("room_stable", first_frame=10, actual_frame=40, publication_outcome="earlier_than_actual"),
                    _policy_room("room_changed", first_frame=10, actual_frame=40, publication_outcome="earlier_than_actual"),
                    _policy_room("room_withheld", first_frame=10, actual_frame=None, publication_outcome="simulated_only_never_actual"),
                    _policy_room("room_gone", first_frame=10, actual_frame=None, publication_outcome="simulated_only_never_actual"),
                ],
            }
        ],
    }
    return {
        "timeline": timeline_payload,
        "lifecycle": lifecycle_payload,
        "publication_policy": publication_policy_payload,
    }


def test_survivability_classifies_candidate_outcomes() -> None:
    payloads = _payloads()
    summary = build_publication_candidate_survivability(
        payloads["publication_policy"],
        payloads["timeline"],
        payloads["lifecycle"],
    )

    policy_result = summary["policy_results"][0]
    rooms = {room["room_id"]: room for room in policy_result["rooms"]}

    assert rooms["room_stable"]["survivability_classification"] == "stable_survivor"
    assert rooms["room_stable"]["survived_to_terminal_without_major_change"] is True

    assert rooms["room_changed"]["survivability_classification"] == "present_but_changed"
    assert rooms["room_changed"]["room_signature_change_after_first_publication_count"] == 1
    assert rooms["room_changed"]["major_blocker_recurrence_after_first_publication"] == 1

    assert rooms["room_withheld"]["survivability_classification"] == "withheld_again_or_reblocked"
    assert rooms["room_withheld"]["terminal_present_in_committed_projection"] is False
    assert rooms["room_withheld"]["gateway_signature_change_after_first_publication_count"] == 1

    assert rooms["room_gone"]["survivability_classification"] == "disappeared_or_not_terminal"
    assert rooms["room_gone"]["terminal_present_in_export"] is False
    assert rooms["room_gone"]["room_removed_after_first_publication_count"] == 1

    summary_counts = policy_result["summary"]
    assert summary_counts["candidate_room_count"] == 4
    assert summary_counts["stable_survivor_count"] == 1
    assert summary_counts["present_but_changed_count"] == 1
    assert summary_counts["withheld_again_or_reblocked_count"] == 1
    assert summary_counts["disappeared_or_not_terminal_count"] == 1


def test_survivability_builder_does_not_mutate_inputs() -> None:
    payloads = _payloads()
    original = copy.deepcopy(payloads)

    build_publication_candidate_survivability(
        payloads["publication_policy"],
        payloads["timeline"],
        payloads["lifecycle"],
    )

    assert payloads == original


if __name__ == "__main__":
    test_survivability_classifies_candidate_outcomes()
    test_survivability_builder_does_not_mutate_inputs()
    print("ok")
