import os
import sys
from typing import Any, Dict, List, Optional

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.online_topology_blocker_diag import build_blocker_diagnosis
from boxfusion.online_topology_timeline_eval import build_timeline_summary


def _refresh_room(
    room_id: str,
    *,
    lifecycle_state: str,
    candidate_complete: bool,
    candidate_block_reasons: Optional[List[str]] = None,
    commit_block_reasons: Optional[List[str]] = None,
    first_seen_frame_idx: int = 0,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "first_seen_frame_idx": int(first_seen_frame_idx),
        "last_departed_frame_idx": None,
        "lifecycle_state": lifecycle_state,
        "candidate_complete": bool(candidate_complete),
        "candidate_readiness_score": 0,
        "candidate_complete_reasons": [],
        "candidate_block_reasons": list(candidate_block_reasons or []),
        "commit_block_reasons": list(commit_block_reasons or []),
        "merge_pending": "merge_or_split_pending" in set(commit_block_reasons or []),
        "dirty": False,
        "present_in_latest_export": True,
        "floor_id": "floor_1",
        "floor_status": "confirmed",
        "stable_refresh_opportunities_since_structural_delta": 0,
    }


def _final_room(
    room_id: str,
    *,
    lifecycle_state: str,
    first_seen_frame_idx: int = 0,
    candidate_complete: bool = False,
    commit_block_reasons: Optional[List[str]] = None,
) -> Dict[str, Any]:
    return {
        "room_id": room_id,
        "lifecycle_state": lifecycle_state,
        "dirty": False,
        "dirty_reasons": [],
        "last_updated_frame_idx": 40,
        "last_updated_timestamp": 4.0,
        "first_seen_frame_idx": int(first_seen_frame_idx),
        "last_observed_frame_idx": 40,
        "last_departed_frame_idx": 20,
        "export_observation_count": 5,
        "floor_id": "floor_1",
        "floor_status": "confirmed",
        "present_in_latest_export": True,
        "room_signature_stability_count": 2,
        "gateway_signature_stability_count": 2,
        "containment_stability_count": 2,
        "candidate_complete": bool(candidate_complete),
        "candidate_readiness_score": 0,
        "candidate_complete_reasons": [],
        "candidate_block_reasons": [],
        "commit_block_reasons": list(commit_block_reasons or []),
        "stable_refresh_opportunities_since_structural_delta": 0,
        "last_structural_delta_frame_idx": 20,
        "vertical_transition_statuses": [],
        "vertical_transition_ids": [],
    }


def _refresh(frame_idx: int, rooms: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "frame_idx": int(frame_idx),
        "timestamp": float(frame_idx) / 10.0,
        "active_room_id": None,
        "active_floor_id": "floor_1",
        "active_floor_status": "confirmed",
        "summary": {
            "room_count": int(len(rooms)),
            "candidate_complete_room_count": int(sum(1 for room in rooms if room["candidate_complete"])),
            "commit_ready_room_count": int(sum(1 for room in rooms if not room["commit_block_reasons"])),
            "blocked_commit_room_count": int(sum(1 for room in rooms if room["commit_block_reasons"])),
        },
        "candidate_complete_rooms": [room["room_id"] for room in rooms if room["candidate_complete"]],
        "commit_ready_rooms": [room["room_id"] for room in rooms if not room["commit_block_reasons"]],
        "blocked_commit_reasons": {
            room["room_id"]: list(room["commit_block_reasons"])
            for room in rooms
            if room["commit_block_reasons"]
        },
        "rooms": rooms,
    }


def _scene_context(payload: Dict[str, Any], scene_id: str) -> Dict[str, Any]:
    return {
        "scene_id": scene_id,
        "sequence_id": payload.get("sequence_id"),
        "input_path": scene_id,
        "timeline_path": None,
        "lifecycle_path": f"{scene_id}/logs/online_topology_lifecycle_v0_1.json",
        "timeline_source": "derived_from_lifecycle",
        "timeline_summary": build_timeline_summary(payload),
        "lifecycle_payload": payload,
        "output_base_dir": ".",
    }


def _scene_one_payload() -> Dict[str, Any]:
    refresh_history = [
        _refresh(
            0,
            [
                _refresh_room(
                    "room_1",
                    lifecycle_state="active",
                    candidate_complete=False,
                    candidate_block_reasons=["room_signature_not_stable"],
                    commit_block_reasons=["merge_or_split_pending", "room_signature_not_stable"],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="active",
                    candidate_complete=False,
                    candidate_block_reasons=["gateway_structure_not_stable", "room_signature_not_stable"],
                    commit_block_reasons=[
                        "gateway_structure_not_stable",
                        "merge_or_split_pending",
                        "room_signature_not_stable",
                    ],
                ),
            ],
        ),
        _refresh(
            10,
            [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    candidate_complete=True,
                    commit_block_reasons=["merge_or_split_pending"],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="candidate_complete",
                    candidate_complete=True,
                    commit_block_reasons=["gateway_structure_not_stable", "merge_or_split_pending"],
                ),
            ],
        ),
        _refresh(
            20,
            [
                _refresh_room(
                    "room_1",
                    lifecycle_state="active",
                    candidate_complete=False,
                    candidate_block_reasons=["room_signature_not_stable"],
                    commit_block_reasons=["merge_or_split_pending", "room_signature_not_stable"],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="merge_or_split_pending",
                    candidate_complete=True,
                    commit_block_reasons=["gateway_structure_not_stable", "merge_or_split_pending"],
                ),
            ],
        ),
        _refresh(
            30,
            [
                _refresh_room(
                    "room_1",
                    lifecycle_state="candidate_complete",
                    candidate_complete=True,
                    commit_block_reasons=[],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="merge_or_split_pending",
                    candidate_complete=True,
                    commit_block_reasons=["gateway_structure_not_stable", "merge_or_split_pending"],
                ),
            ],
        ),
        _refresh(
            40,
            [
                _refresh_room(
                    "room_1",
                    lifecycle_state="committed",
                    candidate_complete=True,
                    commit_block_reasons=[],
                ),
                _refresh_room(
                    "room_2",
                    lifecycle_state="merge_or_split_pending",
                    candidate_complete=True,
                    commit_block_reasons=["gateway_structure_not_stable", "merge_or_split_pending"],
                ),
            ],
        ),
    ]
    return {
        "version": "0.1",
        "sequence_id": "scene_one_sequence",
        "frame_idx": 40,
        "timestamp": 4.0,
        "summary": {"refresh_count": len(refresh_history)},
        "rooms": [
            _final_room("room_1", lifecycle_state="committed", candidate_complete=True, commit_block_reasons=[]),
            _final_room(
                "room_2",
                lifecycle_state="merge_or_split_pending",
                candidate_complete=True,
                commit_block_reasons=["gateway_structure_not_stable", "merge_or_split_pending"],
            ),
        ],
        "refresh_history": refresh_history,
    }


def _scene_two_payload() -> Dict[str, Any]:
    refresh_history = [
        _refresh(
            0,
            [
                _refresh_room(
                    "room_3",
                    lifecycle_state="active",
                    candidate_complete=False,
                    candidate_block_reasons=["room_currently_active"],
                    commit_block_reasons=["room_currently_active"],
                ),
            ],
        ),
        _refresh(
            10,
            [
                _refresh_room(
                    "room_3",
                    lifecycle_state="active",
                    candidate_complete=False,
                    candidate_block_reasons=["room_currently_active"],
                    commit_block_reasons=["room_currently_active"],
                ),
            ],
        ),
    ]
    return {
        "version": "0.1",
        "sequence_id": "scene_two_sequence",
        "frame_idx": 10,
        "timestamp": 1.0,
        "summary": {"refresh_count": len(refresh_history)},
        "rooms": [
            _final_room(
                "room_3",
                lifecycle_state="active",
                candidate_complete=False,
                commit_block_reasons=["room_currently_active"],
            ),
        ],
        "refresh_history": refresh_history,
    }


def test_repeated_candidate_to_blocked_transitions_are_counted() -> None:
    report = build_blocker_diagnosis([_scene_context(_scene_one_payload(), "scene_one")])
    scene = report["scenes"][0]
    room_1 = next(room for room in scene["rooms"] if room["room_id"] == "room_1")

    assert room_1["first_candidate_frame"] == 10
    assert room_1["last_candidate_frame"] == 40
    assert room_1["first_finalized_private_frame"] == 30
    assert room_1["first_commit_ready_frame"] == 30
    assert room_1["first_committed_frame"] == 40
    assert room_1["final_publication_state"] == "PUBLISHED"
    assert room_1["final_finalization_blockers"] == []
    assert room_1["final_publication_blockers"] == []
    assert room_1["candidate_to_blocked_transition_count"] == 1
    assert room_1["blocked_to_candidate_transition_count"] == 2
    assert room_1["total_candidate_refreshes"] == 3
    assert room_1["total_candidate_but_blocked_refreshes"] == 1
    assert room_1["total_commit_ready_refreshes"] == 2
    assert room_1["dominant_blocker"] == "room_signature_not_stable"
    assert room_1["persistent_blockers"] == ["room_signature_not_stable"]
    assert room_1["transient_blockers"] == ["merge_or_split_pending"]
    assert room_1["final_unresolved_blockers"] == []
    assert room_1["longest_stable_window_in_refreshes"] == 2
    assert room_1["longest_blocked_window_in_refreshes"] == 1


def test_persistent_vs_transient_blockers_are_classified_conservatively() -> None:
    report = build_blocker_diagnosis([_scene_context(_scene_one_payload(), "scene_one")])
    scene = report["scenes"][0]
    room_2 = next(room for room in scene["rooms"] if room["room_id"] == "room_2")

    assert room_2["first_candidate_frame"] == 10
    assert room_2["first_committed_frame"] is None
    assert room_2["final_publication_state"] == "FINALIZATION_PENDING"
    assert room_2["final_finalization_blockers"] == [
        "gateway_structure_not_stable",
        "merge_or_split_pending",
    ]
    assert room_2["final_publication_blockers"] == []
    assert room_2["candidate_to_blocked_transition_count"] == 0
    assert room_2["total_candidate_but_blocked_refreshes"] == 4
    assert room_2["dominant_blocker"] == "gateway_structure_not_stable"
    assert room_2["persistent_blockers"] == [
        "gateway_structure_not_stable",
        "merge_or_split_pending",
    ]
    assert room_2["transient_blockers"] == ["room_signature_not_stable"]
    assert room_2["final_unresolved_blockers"] == [
        "gateway_structure_not_stable",
        "merge_or_split_pending",
    ]


def test_scene_level_aggregation_and_room_lists_work_across_two_scenes() -> None:
    report = build_blocker_diagnosis(
        [
            _scene_context(_scene_one_payload(), "scene_one"),
            _scene_context(_scene_two_payload(), "scene_two"),
        ]
    )
    aggregate = report["aggregate_summary"]

    assert aggregate["rooms_reached_candidate_but_never_committed"] == ["scene_one:room_2"]
    assert aggregate["rooms_committed_after_repeated_reblocking"] == ["scene_one:room_1"]
    top_blockers = [item["blocker"] for item in aggregate["blocker_frequency_summary"]]
    assert "gateway_structure_not_stable" in top_blockers
    assert "merge_or_split_pending" in top_blockers
    assert "room_currently_active" in top_blockers


def test_final_blocker_heuristics_separate_structural_from_transient_like_cases() -> None:
    report = build_blocker_diagnosis(
        [
            _scene_context(_scene_one_payload(), "scene_one"),
            _scene_context(_scene_two_payload(), "scene_two"),
        ]
    )
    scene_one = next(scene for scene in report["scenes"] if scene["scene_id"] == "scene_one")
    scene_two = next(scene for scene in report["scenes"] if scene["scene_id"] == "scene_two")

    room_2_heuristic = next(row for row in scene_one["final_blocker_heuristics"] if row["room_id"] == "room_2")
    room_3_heuristic = next(row for row in scene_two["final_blocker_heuristics"] if row["room_id"] == "room_3")
    room_3 = next(room for room in scene_two["rooms"] if room["room_id"] == "room_3")

    assert room_2_heuristic["classification"] == "likely_structural"
    assert room_3_heuristic["classification"] == "likely_noisy_or_transient"
    assert room_3["final_publication_state"] == "FINALIZED_PRIVATE"
    assert room_3["final_finalization_blockers"] == []
    assert room_3["final_publication_blockers"] == ["room_currently_active"]


def run_mock_test() -> None:
    print("Running online topology blocker diagnosis tests...")
    test_repeated_candidate_to_blocked_transitions_are_counted()
    test_persistent_vs_transient_blockers_are_classified_conservatively()
    test_scene_level_aggregation_and_room_lists_work_across_two_scenes()
    test_final_blocker_heuristics_separate_structural_from_transient_like_cases()
    print("Online topology blocker diagnosis tests passed.")


if __name__ == "__main__":
    run_mock_test()
