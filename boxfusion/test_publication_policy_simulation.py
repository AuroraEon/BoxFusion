import copy
import os
import sys
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.publication_policy_simulation import build_publication_policy_simulation


def _refresh_room(
    room_id: str,
    *,
    lifecycle_state: str,
    present_in_latest_export: bool,
    candidate_complete: bool = False,
    commit_block_reasons: List[str] = None,
    dirty: bool = True,
    stable_refresh_opportunities_since_structural_delta: int = 0,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "present_in_latest_export": present_in_latest_export,
        "candidate_complete": candidate_complete,
        "commit_block_reasons": list(commit_block_reasons or []),
        "dirty": bool(dirty),
        "stable_refresh_opportunities_since_structural_delta": int(stable_refresh_opportunities_since_structural_delta),
    }


def _timeline_room(
    room_id: str,
    *,
    first_committed_frame: int = None,
    dominant_withhold_blockers: List[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "first_committed_frame": first_committed_frame,
        "dominant_withhold_blockers": list(dominant_withhold_blockers or []),
    }


def _payloads() -> Dict[str, Dict[str, Any]]:
    refresh_history = [
        {
            "frame_idx": 0,
            "timestamp": 0.0,
            "rooms": [
                _refresh_room(
                    "room_1",
                    lifecycle_state="merge_or_split_pending",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="merge_or_split_pending",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_currently_active"],
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="merge_or_split_pending",
                    present_in_latest_export=True,
                    commit_block_reasons=["containment_not_stable"],
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
            ],
        },
        {
            "frame_idx": 10,
            "timestamp": 1.0,
            "rooms": [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=1,
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=1,
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
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
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    commit_block_reasons=["room_currently_active"],
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=2,
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
            ],
        },
        {
            "frame_idx": 30,
            "timestamp": 3.0,
            "rooms": [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=False,
                    candidate_complete=True,
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=1,
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=3,
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
            ],
        },
        {
            "frame_idx": 40,
            "timestamp": 4.0,
            "rooms": [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=False,
                    candidate_complete=True,
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=2,
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=4,
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
            ],
        },
        {
            "frame_idx": 50,
            "timestamp": 5.0,
            "rooms": [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=False,
                    candidate_complete=True,
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=3,
                ),
                _refresh_room(
                    "room_3",
                    lifecycle_state="candidate_complete",
                    present_in_latest_export=True,
                    candidate_complete=True,
                    stable_refresh_opportunities_since_structural_delta=5,
                ),
                _refresh_room(
                    "room_4",
                    lifecycle_state="active",
                    present_in_latest_export=True,
                    commit_block_reasons=["room_signature_not_stable"],
                ),
            ],
        },
    ]

    timeline_payload = {
        "sequence_id": "policy_sim_test_scene",
        "history_mode": "refresh_history",
        "input_frame_idx": 50,
        "input_timestamp": 5.0,
        "scene_summary": {
            "terminal_projection": {
                "frame_idx": 50,
                "timestamp": 5.0,
                "working_room_count": 3,
                "committed_room_count": 1,
                "withheld_room_count": 2,
            },
            "dominant_withhold_blockers": [
                {"blocker": "room_signature_not_stable", "refresh_count": 8},
                {"blocker": "room_currently_active", "refresh_count": 2},
            ],
            "final_static_comparison": {
                "working_room_count": 3,
                "committed_room_count": 1,
                "withheld_room_count": 2,
                "working_only_room_ids": ["room_2", "room_4"],
            },
            "counts_over_time": [
                {
                    "frame_idx": 0,
                    "committed_room_ids": [],
                },
                {
                    "frame_idx": 10,
                    "committed_room_ids": [],
                },
                {
                    "frame_idx": 20,
                    "committed_room_ids": [],
                },
                {
                    "frame_idx": 30,
                    "committed_room_ids": [],
                },
                {
                    "frame_idx": 40,
                    "committed_room_ids": [],
                },
                {
                    "frame_idx": 50,
                    "committed_room_ids": ["room_3"],
                },
            ],
        },
        "rooms": [
            _timeline_room(
                "room_1",
                dominant_withhold_blockers=[
                    {"blocker": "commit_ready_pending_publication", "refresh_count": 2},
                    {"blocker": "not_present_in_latest_export", "refresh_count": 3},
                ],
            ),
            _timeline_room(
                "room_2",
                dominant_withhold_blockers=[
                    {"blocker": "commit_ready_pending_publication", "refresh_count": 3},
                    {"blocker": "room_currently_active", "refresh_count": 2},
                ],
            ),
            _timeline_room(
                "room_3",
                first_committed_frame=50,
                dominant_withhold_blockers=[
                    {"blocker": "commit_ready_pending_publication", "refresh_count": 4},
                ],
            ),
            _timeline_room(
                "room_4",
                dominant_withhold_blockers=[
                    {"blocker": "room_signature_not_stable", "refresh_count": 6},
                ],
            ),
        ],
    }

    lifecycle_payload = {
        "sequence_id": "policy_sim_test_scene",
        "frame_idx": 50,
        "timestamp": 5.0,
        "rooms": [
            _refresh_room(
                "room_1",
                lifecycle_state="candidate_complete",
                present_in_latest_export=False,
                candidate_complete=True,
                dirty=False,
            ),
            _refresh_room(
                "room_2",
                lifecycle_state="candidate_complete",
                present_in_latest_export=True,
                candidate_complete=True,
                stable_refresh_opportunities_since_structural_delta=3,
            ),
            _refresh_room(
                "room_3",
                lifecycle_state="committed",
                present_in_latest_export=True,
                candidate_complete=True,
                dirty=False,
                stable_refresh_opportunities_since_structural_delta=5,
            ),
            _refresh_room(
                "room_4",
                lifecycle_state="active",
                present_in_latest_export=True,
                commit_block_reasons=["room_signature_not_stable"],
            ),
        ],
        "refresh_history": refresh_history,
    }

    topology_payload = {
        "rooms": [
            {"id": "room_2"},
            {"id": "room_3"},
            {"id": "room_4"},
        ],
        "edges": [
            {"source": "room_2", "target": "room_3", "relation_type": "adjacent"},
            {"source": "room_3", "target": "room_4", "relation_type": "adjacent"},
        ],
    }
    return {
        "timeline": timeline_payload,
        "lifecycle": lifecycle_payload,
        "topology": topology_payload,
    }


def _policy(summary: Dict[str, Any], policy_id: str) -> Dict[str, Any]:
    return next(item for item in summary["policy_results"] if item["policy"]["policy_id"] == policy_id)


def _room(policy_result: Dict[str, Any], room_id: str) -> Dict[str, Any]:
    return next(item for item in policy_result["rooms"] if item["room_id"] == room_id)


def test_publication_policy_simulation_preserves_inputs_and_debug_flags() -> None:
    payloads = _payloads()
    original_payloads = copy.deepcopy(payloads)

    summary = build_publication_policy_simulation(
        payloads["timeline"],
        payloads["lifecycle"],
        topology_payload=payloads["topology"],
    )

    assert payloads == original_payloads
    assert summary["debug_only"] is True
    assert summary["public_default"] is False
    assert summary["non_public"] is True
    assert summary["simulation_semantics"]["actual_public_semantics_changed"] is False


def test_candidate_complete_policy_publishes_earlier_and_tracks_actual_lead_time() -> None:
    summary = build_publication_policy_simulation(
        _payloads()["timeline"],
        _payloads()["lifecycle"],
        topology_payload=_payloads()["topology"],
    )
    policy_result = _policy(summary, "candidate_complete_blocker_free_n2")

    room_1 = _room(policy_result, "room_1")
    room_3 = _room(policy_result, "room_3")

    assert room_1["first_policy_qualification_frame"] == 20
    assert room_1["first_simulated_publication_frame"] == 20
    assert room_1["publication_outcome"] == "simulated_only_never_actual"
    assert room_3["first_policy_qualification_frame"] == 20
    assert room_3["first_simulated_publication_frame"] == 20
    assert room_3["publication_outcome"] == "earlier_than_actual"
    assert room_3["publication_lead_time_frames_vs_actual_committed_projection"] == 30


def test_blocker_recurrence_and_stronger_stability_gate_delay_publication() -> None:
    summary = build_publication_policy_simulation(
        _payloads()["timeline"],
        _payloads()["lifecycle"],
        topology_payload=_payloads()["topology"],
    )
    policy_result = _policy(summary, "commit_ready_export_stable2_n2")
    room_1 = _room(policy_result, "room_1")
    room_2 = _room(policy_result, "room_2")

    assert room_1["first_policy_qualification_frame"] is None
    assert room_1["publication_outcome"] == "still_unpublished"
    assert room_2["first_policy_qualification_frame"] == 40
    assert room_2["first_simulated_publication_frame"] == 40
    assert policy_result["summary"]["remaining_unpublished_room_ids"] == ["room_1", "room_4"]
    assert room_1["remaining_unpublished_risk_summary"][0]["reason"] == "not_present_in_latest_export"


def test_dirty_window_policy_adds_no_new_publication_and_room_can_still_publish_actual() -> None:
    summary = build_publication_policy_simulation(
        _payloads()["timeline"],
        _payloads()["lifecycle"],
        topology_payload=_payloads()["topology"],
    )
    policy_result = _policy(summary, "commit_ready_not_dirty_n2")
    room_3 = _room(policy_result, "room_3")
    room_4 = _room(policy_result, "room_4")

    assert room_3["first_policy_qualification_frame"] is None
    assert room_3["first_simulated_publication_frame"] == 50
    assert room_3["publication_outcome"] == "no_earlier_change"
    assert room_4["publication_outcome"] == "still_unpublished"
    assert any(item["reason"] == "dirty" for item in room_4["remaining_unpublished_risk_summary"]) is False
    assert policy_result["summary"]["maximum_additional_room_count_vs_actual"] == 0


def test_terminal_projection_uses_final_room_state_for_actual_baseline_and_delta() -> None:
    summary = build_publication_policy_simulation(
        _payloads()["timeline"],
        _payloads()["lifecycle"],
        topology_payload=_payloads()["topology"],
    )
    policy_result = _policy(summary, "candidate_complete_blocker_free_n2")
    terminal_row = policy_result["counts_over_time"][-1]
    terminal_delta = policy_result["summary"]["terminal_topology_delta_vs_actual"]

    assert terminal_row["row_source"] == "terminal_projection"
    assert terminal_row["actual_committed_room_ids"] == ["room_3"]
    assert terminal_row["additional_room_ids_vs_actual"] == ["room_2"]
    assert terminal_delta["actual_room_count"] == 1
    assert terminal_delta["simulated_room_count"] == 2
    assert terminal_delta["actual_room_ids"] == ["room_3"]
    assert terminal_delta["simulated_room_ids"] == ["room_2", "room_3"]
    assert terminal_delta["additional_edge_count_vs_actual"] == 1


if __name__ == "__main__":
    test_publication_policy_simulation_preserves_inputs_and_debug_flags()
    test_candidate_complete_policy_publishes_earlier_and_tracks_actual_lead_time()
    test_blocker_recurrence_and_stronger_stability_gate_delay_publication()
    test_dirty_window_policy_adds_no_new_publication_and_room_can_still_publish_actual()
    test_terminal_projection_uses_final_room_state_for_actual_baseline_and_delta()
    print("ok")
