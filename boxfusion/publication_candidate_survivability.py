from __future__ import annotations

import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .online_topology_timeline_eval import load_json
from .publication_policy_simulation import (
    DEFAULT_JSON_NAME as SIMULATION_JSON_NAME,
    build_publication_policy_simulation,
    resolve_input_artifacts as resolve_simulation_input_artifacts,
)
from .working_vs_committed_topology_timeline import DEFAULT_JSON_NAME as TIMELINE_JSON_NAME


DEFAULT_JSON_NAME = "publication_candidate_survivability_v0_1.json"
MAJOR_STRUCTURAL_BLOCKERS = (
    "containment_not_stable",
    "floor_status_not_stable",
    "gateway_structure_not_stable",
    "merge_or_split_pending",
    "room_floor_validation_failed",
    "room_missing_from_latest_export",
    "room_signature_not_stable",
    "vertical_transition_partial",
)
_CANDIDATE_OUTCOMES = {"earlier_than_actual", "simulated_only_never_actual"}


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def default_output_path(input_artifact_path: Path) -> Path:
    return Path(input_artifact_path).parent / DEFAULT_JSON_NAME


def resolve_input_artifacts(input_path: Path) -> Dict[str, Optional[Path]]:
    path = Path(input_path)
    if path.name == SIMULATION_JSON_NAME:
        timeline_path = path.parent / TIMELINE_JSON_NAME
        lifecycle_path = path.parent / "online_topology_lifecycle_v0_1.json"
        if not timeline_path.exists():
            raise FileNotFoundError(f"Missing sibling timeline artifact for survivability input: {timeline_path}")
        if not lifecycle_path.exists():
            raise FileNotFoundError(f"Missing sibling lifecycle artifact for survivability input: {lifecycle_path}")
        topology_path = path.parent / "topology_v0_1.json"
        return {
            "publication_policy_simulation_path": path,
            "timeline_path": timeline_path,
            "lifecycle_path": lifecycle_path,
            "topology_path": topology_path if topology_path.exists() else None,
        }

    artifacts = resolve_simulation_input_artifacts(path)
    timeline_path = Path(artifacts["timeline_path"])
    simulation_path = timeline_path.parent / SIMULATION_JSON_NAME
    return {
        "publication_policy_simulation_path": simulation_path if simulation_path.exists() else None,
        "timeline_path": timeline_path,
        "lifecycle_path": Path(artifacts["lifecycle_path"]),
        "topology_path": None if artifacts.get("topology_path") is None else Path(artifacts["topology_path"]),
    }


def _sorted_str_list(values: Iterable[Any]) -> List[str]:
    return sorted(str(value) for value in values if value not in (None, ""))


def _ranked_reason_counts(counter: Counter) -> List[Dict[str, Any]]:
    ranked = sorted(counter.items(), key=lambda item: (-int(item[1]), item[0]))
    return [{"reason": str(reason), "count": int(count)} for reason, count in ranked]


def _ranked_blocker_counts(counter: Counter) -> List[Dict[str, Any]]:
    ranked = sorted(counter.items(), key=lambda item: (-int(item[1]), item[0]))
    return [{"blocker": str(blocker), "refresh_count": int(count)} for blocker, count in ranked]


def _first_non_none(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _room_lookup(rooms: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(rooms or []):
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _build_room_refresh_history(refresh_history: Sequence[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    by_room: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for refresh in list(refresh_history or []):
        frame_idx = refresh.get("frame_idx")
        timestamp = refresh.get("timestamp")
        for room in list(refresh.get("rooms") or []):
            room_id = room.get("room_id")
            if room_id is None:
                continue
            by_room[str(room_id)].append(
                {
                    "frame_idx": None if frame_idx is None else int(frame_idx),
                    "timestamp": timestamp,
                    "room": dict(room),
                }
            )
    return {room_id: list(entries) for room_id, entries in by_room.items()}


def _build_room_trigger_history(trigger_history: Sequence[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    by_room: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for trigger in list(trigger_history or []):
        room_id = trigger.get("room_id")
        if room_id is None:
            continue
        by_room[str(room_id)].append(dict(trigger))
    return {room_id: list(entries) for room_id, entries in by_room.items()}


def _count_trigger_events_after_frame(
    triggers: Sequence[Dict[str, Any]],
    *,
    trigger_name: str,
    frame_idx: int,
) -> int:
    count = 0
    for trigger in list(triggers or []):
        if str(trigger.get("trigger")) != str(trigger_name):
            continue
        trigger_frame = trigger.get("frame_idx")
        if trigger_frame is None or int(trigger_frame) <= int(frame_idx):
            continue
        count += 1
    return int(count)


def _post_publication_major_blockers(
    refresh_steps: Sequence[Dict[str, Any]],
    *,
    first_publication_frame: int,
) -> Tuple[int, Counter, int]:
    recurrence_counter: Counter = Counter()
    merge_pending_refresh_count = 0
    recurrence_refresh_count = 0
    for step in list(refresh_steps or []):
        frame_idx = step.get("frame_idx")
        if frame_idx is None or int(frame_idx) <= int(first_publication_frame):
            continue
        room = dict(step.get("room") or {})
        blockers = set(_sorted_str_list(room.get("commit_block_reasons") or []))
        hit_blockers = set(blockers) & set(MAJOR_STRUCTURAL_BLOCKERS)
        if not bool(room.get("present_in_latest_export")):
            hit_blockers.add("room_missing_from_latest_export")
        if str(room.get("lifecycle_state")) == "merge_or_split_pending":
            merge_pending_refresh_count += 1
            hit_blockers.add("merge_or_split_pending")
        if hit_blockers:
            recurrence_refresh_count += 1
            for blocker in sorted(hit_blockers):
                recurrence_counter[blocker] += 1
    return int(recurrence_refresh_count), recurrence_counter, int(merge_pending_refresh_count)


def _candidate_failure_reasons(room_summary: Dict[str, Any]) -> List[str]:
    reasons: List[str] = []
    if not bool(room_summary.get("terminal_present_in_export")):
        reasons.append("not_in_terminal_export")
    if bool(room_summary.get("terminal_present_in_export")) and not bool(
        room_summary.get("terminal_present_in_committed_projection")
    ):
        reasons.append("not_in_terminal_committed_projection")
    if int(room_summary.get("room_signature_change_after_first_publication_count") or 0) > 0:
        reasons.append("room_signature_changed_after_publication")
    if int(room_summary.get("gateway_signature_change_after_first_publication_count") or 0) > 0:
        reasons.append("gateway_structure_changed_after_publication")
    if int(room_summary.get("major_blocker_recurrence_after_first_publication") or 0) > 0:
        reasons.append("major_structural_blocker_recurred_after_publication")
    if int(room_summary.get("room_removed_after_first_publication_count") or 0) > 0:
        reasons.append("room_removed_after_publication")
    if int(room_summary.get("room_revisit_after_first_publication_count") or 0) > 0:
        reasons.append("room_revisit_after_publication")
    if int(room_summary.get("merge_or_split_pending_after_first_publication_count") or 0) > 0:
        reasons.append("merge_or_split_pending_after_publication")
    for blocker in list(room_summary.get("major_blocker_recurrence_summary") or []):
        reasons.append(f"major_blocker:{blocker['blocker']}")
    return reasons


def _classify_candidate(room_summary: Dict[str, Any]) -> str:
    if not bool(room_summary.get("terminal_present_in_export")):
        return "disappeared_or_not_terminal"
    if not bool(room_summary.get("terminal_present_in_committed_projection")):
        return "withheld_again_or_reblocked"
    if bool(room_summary.get("survived_to_terminal_without_major_change")):
        return "stable_survivor"
    return "present_but_changed"


def _candidate_room_summary(
    *,
    policy_id: str,
    candidate_room: Dict[str, Any],
    final_rooms: Dict[str, Dict[str, Any]],
    refresh_history_by_room: Dict[str, List[Dict[str, Any]]],
    trigger_history_by_room: Dict[str, List[Dict[str, Any]]],
    terminal_frame_idx: Optional[int],
) -> Dict[str, Any]:
    room_id = str(candidate_room.get("room_id"))
    publication_frame = candidate_room.get("first_simulated_publication_frame")
    if publication_frame is None:
        raise ValueError(f"Candidate room is missing first_simulated_publication_frame: {room_id}")
    publication_frame = int(publication_frame)

    final_room = dict(final_rooms.get(room_id) or {})
    final_lifecycle_state = str(final_room.get("lifecycle_state") or "not_present")
    final_blocker_set = [] if final_lifecycle_state == "committed" else _sorted_str_list(final_room.get("commit_block_reasons") or [])
    terminal_present_in_export = bool(final_room.get("present_in_latest_export"))
    terminal_present_in_committed_projection = bool(terminal_present_in_export and final_lifecycle_state == "committed")
    preserved_room_identity = bool(room_id in final_rooms and terminal_present_in_export)

    room_refreshes = list(refresh_history_by_room.get(room_id) or [])
    room_triggers = list(trigger_history_by_room.get(room_id) or [])
    room_signature_change_count = _count_trigger_events_after_frame(
        room_triggers,
        trigger_name="room_signature_changed",
        frame_idx=publication_frame,
    )
    gateway_signature_change_count = _count_trigger_events_after_frame(
        room_triggers,
        trigger_name="gateway_structure_changed",
        frame_idx=publication_frame,
    )
    room_removed_count = _count_trigger_events_after_frame(
        room_triggers,
        trigger_name="room_removed",
        frame_idx=publication_frame,
    )
    room_revisit_count = _count_trigger_events_after_frame(
        room_triggers,
        trigger_name="room_revisit",
        frame_idx=publication_frame,
    )
    major_blocker_recurrence, major_blocker_counter, merge_pending_count = _post_publication_major_blockers(
        room_refreshes,
        first_publication_frame=publication_frame,
    )

    publication_to_terminal_frame_span = None
    if terminal_frame_idx is not None:
        publication_to_terminal_frame_span = int(terminal_frame_idx) - int(publication_frame)

    survived_to_terminal_without_major_change = bool(
        terminal_present_in_export
        and terminal_present_in_committed_projection
        and preserved_room_identity
        and int(room_signature_change_count) == 0
        and int(gateway_signature_change_count) == 0
        and int(major_blocker_recurrence) == 0
    )

    room_summary = {
        "room_id": room_id,
        "policy_id": str(policy_id),
        "publication_outcome": str(candidate_room.get("publication_outcome")),
        "first_policy_qualification_frame": candidate_room.get("first_policy_qualification_frame"),
        "actual_committed_projection_frame": candidate_room.get("actual_committed_projection_frame"),
        "first_simulated_publication_frame": publication_frame,
        "publication_lead_time_frames_vs_actual_committed_projection": candidate_room.get(
            "publication_lead_time_frames_vs_actual_committed_projection"
        ),
        "publication_to_terminal_frame_span": publication_to_terminal_frame_span,
        "terminal_present_in_export": bool(terminal_present_in_export),
        "terminal_present_in_committed_projection": bool(terminal_present_in_committed_projection),
        "preserved_room_identity": bool(preserved_room_identity),
        "room_signature_preserved_after_first_publication": bool(int(room_signature_change_count) == 0),
        "gateway_relations_preserved_after_first_publication": bool(int(gateway_signature_change_count) == 0),
        "room_signature_change_after_first_publication_count": int(room_signature_change_count),
        "gateway_signature_change_after_first_publication_count": int(gateway_signature_change_count),
        "major_blocker_recurrence_after_first_publication": int(major_blocker_recurrence),
        "major_blocker_recurrence_summary": _ranked_blocker_counts(major_blocker_counter),
        "merge_or_split_pending_after_first_publication_count": int(merge_pending_count),
        "room_removed_after_first_publication_count": int(room_removed_count),
        "room_revisit_after_first_publication_count": int(room_revisit_count),
        "final_lifecycle_state": final_lifecycle_state,
        "final_blocker_set": list(final_blocker_set),
        "survived_to_terminal_without_major_change": bool(survived_to_terminal_without_major_change),
    }
    room_summary["survivability_classification"] = _classify_candidate(room_summary)
    room_summary["failure_reasons"] = _candidate_failure_reasons(room_summary)
    return room_summary


def _summarize_candidates(candidates: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    classification_counter: Counter = Counter()
    failure_reason_counter: Counter = Counter()
    blocker_counter: Counter = Counter()
    terminal_present_count = 0
    terminal_committed_count = 0
    for room in list(candidates or []):
        classification_counter[str(room.get("survivability_classification") or "unknown")] += 1
        if bool(room.get("terminal_present_in_export")):
            terminal_present_count += 1
        if bool(room.get("terminal_present_in_committed_projection")):
            terminal_committed_count += 1
        for reason in list(room.get("failure_reasons") or []):
            failure_reason_counter[str(reason)] += 1
        for blocker in list(room.get("major_blocker_recurrence_summary") or []):
            blocker_counter[str(blocker["blocker"])] += int(blocker["refresh_count"])

    candidate_count = int(len(list(candidates or [])))
    stable_count = int(classification_counter.get("stable_survivor", 0))
    return {
        "candidate_room_count": candidate_count,
        "stable_survivor_count": stable_count,
        "present_but_changed_count": int(classification_counter.get("present_but_changed", 0)),
        "withheld_again_or_reblocked_count": int(classification_counter.get("withheld_again_or_reblocked", 0)),
        "disappeared_or_not_terminal_count": int(classification_counter.get("disappeared_or_not_terminal", 0)),
        "stable_survivor_rate": None if candidate_count == 0 else round(float(stable_count / candidate_count), 3),
        "terminal_export_presence_rate": None
        if candidate_count == 0
        else round(float(terminal_present_count / candidate_count), 3),
        "terminal_committed_projection_rate": None
        if candidate_count == 0
        else round(float(terminal_committed_count / candidate_count), 3),
        "dominant_failure_reasons": _ranked_reason_counts(failure_reason_counter),
        "dominant_post_publication_major_blockers": _ranked_blocker_counts(blocker_counter),
    }


def _earliest_unique_candidate_rollup(policy_results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    earliest_by_room: Dict[str, Dict[str, Any]] = {}
    for policy_result in list(policy_results or []):
        for room in list(policy_result.get("rooms") or []):
            room_id = str(room.get("room_id"))
            previous = earliest_by_room.get(room_id)
            if previous is None or int(room["first_simulated_publication_frame"]) < int(previous["first_simulated_publication_frame"]):
                earliest_by_room[room_id] = {
                    "room_id": room_id,
                    "policy_id": str(policy_result.get("policy", {}).get("policy_id")),
                    "first_simulated_publication_frame": int(room["first_simulated_publication_frame"]),
                    "survivability_classification": str(room.get("survivability_classification")),
                    "survived_to_terminal_without_major_change": bool(
                        room.get("survived_to_terminal_without_major_change")
                    ),
                }
    return [earliest_by_room[room_id] for room_id in sorted(earliest_by_room)]


def build_publication_candidate_survivability(
    publication_policy_payload: Dict[str, Any],
    timeline_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
    *,
    source_artifacts: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    refresh_history = list(lifecycle_payload.get("refresh_history") or [])
    trigger_history = list(lifecycle_payload.get("trigger_history") or [])
    final_rooms = _room_lookup(lifecycle_payload.get("rooms") or [])
    refresh_history_by_room = _build_room_refresh_history(refresh_history)
    trigger_history_by_room = _build_room_trigger_history(trigger_history)

    terminal_frame_idx = timeline_payload.get("input_frame_idx")
    if terminal_frame_idx is None:
        terminal_frame_idx = lifecycle_payload.get("frame_idx")

    policy_results: List[Dict[str, Any]] = []
    for policy_result in list(publication_policy_payload.get("policy_results") or []):
        policy = dict(policy_result.get("policy") or {})
        candidate_rooms = [
            dict(room)
            for room in list(policy_result.get("rooms") or [])
            if str(room.get("publication_outcome")) in _CANDIDATE_OUTCOMES
            and room.get("first_simulated_publication_frame") is not None
        ]
        evaluated_rooms = [
            _candidate_room_summary(
                policy_id=str(policy.get("policy_id")),
                candidate_room=room,
                final_rooms=final_rooms,
                refresh_history_by_room=refresh_history_by_room,
                trigger_history_by_room=trigger_history_by_room,
                terminal_frame_idx=None if terminal_frame_idx is None else int(terminal_frame_idx),
            )
            for room in candidate_rooms
        ]
        policy_results.append(
            {
                "policy": policy,
                "summary": _summarize_candidates(evaluated_rooms),
                "rooms": evaluated_rooms,
            }
        )

    earliest_rollup = _earliest_unique_candidate_rollup(policy_results)
    return {
        "version": "0.1",
        "artifact_kind": "publication_candidate_survivability_debug",
        "debug_only": True,
        "public_default": False,
        "non_public": True,
        "non_default_policy_analysis": True,
        "truth_owner": "world_export",
        "sequence_id": _first_non_none(
            publication_policy_payload.get("sequence_id"),
            timeline_payload.get("sequence_id"),
            lifecycle_payload.get("sequence_id"),
        ),
        "history_mode": _first_non_none(
            publication_policy_payload.get("history_mode"),
            timeline_payload.get("history_mode"),
            "refresh_history" if refresh_history else "final_report_only",
        ),
        "input_frame_idx": _first_non_none(
            publication_policy_payload.get("input_frame_idx"),
            timeline_payload.get("input_frame_idx"),
            lifecycle_payload.get("frame_idx"),
        ),
        "input_timestamp": _first_non_none(
            publication_policy_payload.get("input_timestamp"),
            timeline_payload.get("input_timestamp"),
            lifecycle_payload.get("timestamp"),
        ),
        "refresh_count": int(len(refresh_history)),
        "room_count": int(len(final_rooms)),
        "survivability_semantics": {
            "evaluator_scope": "debug_only_non_default_early_publication_survivability",
            "actual_public_semantics_changed": False,
            "working_topology_exposed_publicly": False,
            "candidate_room_definition": (
                "policy room with publication_outcome in {earlier_than_actual, simulated_only_never_actual}"
            ),
            "terminal_present_in_export_definition": "final lifecycle room present_in_latest_export == true",
            "terminal_present_in_committed_projection_definition": (
                "terminal_present_in_export && final lifecycle_state == committed"
            ),
            "room_signature_change_after_publication_definition": (
                "count of trigger_history events named room_signature_changed with frame_idx > first_simulated_publication_frame"
            ),
            "gateway_signature_change_after_publication_definition": (
                "count of trigger_history events named gateway_structure_changed with frame_idx > first_simulated_publication_frame"
            ),
            "major_structural_blockers": list(MAJOR_STRUCTURAL_BLOCKERS),
            "major_blocker_recurrence_definition": (
                "count of refresh_history rows after first_simulated_publication_frame where the room is absent from the latest export or carries a major structural commit blocker"
            ),
            "survived_to_terminal_without_major_change_definition": (
                "terminal present in export and in final committed projection, same room_id still survives, and no room-signature changes, gateway-signature changes, or major structural blocker recurrences occur after first simulated publication"
            ),
            "classification_order": [
                "disappeared_or_not_terminal",
                "withheld_again_or_reblocked",
                "stable_survivor",
                "present_but_changed",
            ],
            "note": (
                "Debug-only survivability analysis over simulated policy candidates; this does not adopt any simulated policy and does not alter default committed/public semantics."
            ),
        },
        "scene_summary": {
            "earliest_unique_candidate_rollup": earliest_rollup,
            "earliest_unique_candidate_summary": _summarize_candidates(
                [
                    next(
                        room
                        for policy_result in policy_results
                        for room in list(policy_result.get("rooms") or [])
                        if str(room.get("room_id")) == entry["room_id"]
                        and str(policy_result.get("policy", {}).get("policy_id")) == entry["policy_id"]
                    )
                    for entry in earliest_rollup
                ]
            ),
            "per_policy_survivability": [
                {
                    "policy_id": str(policy_result.get("policy", {}).get("policy_id")),
                    **dict(policy_result.get("summary") or {}),
                }
                for policy_result in policy_results
            ],
        },
        "policy_results": policy_results,
        "source_artifacts": dict(source_artifacts or {}),
    }


def load_or_build_publication_policy_payload(
    *,
    simulation_path: Optional[Path],
    timeline_path: Path,
    lifecycle_path: Path,
    topology_path: Optional[Path],
) -> Dict[str, Any]:
    if simulation_path is not None and Path(simulation_path).exists():
        return load_json(Path(simulation_path))

    timeline_payload = load_json(Path(timeline_path))
    lifecycle_payload = load_json(Path(lifecycle_path))
    topology_payload = None if topology_path is None else load_json(Path(topology_path))
    return build_publication_policy_simulation(
        timeline_payload,
        lifecycle_payload,
        topology_payload=topology_payload,
        source_artifacts={
            "working_vs_committed_topology_timeline_json": str(timeline_path),
            "online_topology_lifecycle_json": str(lifecycle_path),
            "topology_json": None if topology_path is None else str(topology_path),
        },
    )
