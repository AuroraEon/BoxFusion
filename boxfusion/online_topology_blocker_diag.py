from __future__ import annotations

import json
from collections import Counter, defaultdict
from itertools import combinations
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .online_topology_timeline_eval import (
    DEFAULT_JSON_NAME as DEFAULT_TIMELINE_JSON_NAME,
    build_timeline_summary,
    load_json,
    resolve_lifecycle_artifact,
)


DEFAULT_JSON_NAME = "online_topology_blocker_diag_v0_1.json"
DEFAULT_MARKDOWN_NAME = "online_topology_blocker_diag_v0_1.md"

STRUCTURAL_FINAL_BLOCKERS: Set[str] = {
    "containment_not_stable",
    "floor_status_not_stable",
    "gateway_structure_not_stable",
    "merge_or_split_pending",
    "room_missing_from_latest_export",
    "room_signature_not_stable",
    "vertical_transition_not_stable",
}
LIKELY_TRANSIENT_FINAL_BLOCKERS: Set[str] = {
    "no_leave_like_signal",
    "room_currently_active",
}
PERSISTENT_BLOCKER_REFRESH_MIN = 2
PERSISTENT_BLOCKER_COVERAGE_MIN = 0.5


def _sorted_str_list(values: Iterable[Any]) -> List[str]:
    return sorted(str(value) for value in values if value not in (None, ""))


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_markdown(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text).rstrip() + "\n", encoding="utf-8")


def default_output_paths(scene_contexts: Sequence[Dict[str, Any]]) -> Tuple[Path, Path]:
    if len(scene_contexts) == 1:
        base_dir = Path(scene_contexts[0]["output_base_dir"])
    else:
        base_dir = Path.cwd()
    return base_dir / DEFAULT_JSON_NAME, base_dir / DEFAULT_MARKDOWN_NAME


def _scene_id_from_path(path: Path) -> str:
    resolved = Path(path)
    if resolved.is_file():
        resolved = resolved.parent
    if resolved.name == "logs" and resolved.parent.name:
        return str(resolved.parent.name)
    return str(resolved.name or resolved.stem)


def _timeline_from_lifecycle(
    lifecycle_path: Path,
    *,
    lifecycle_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    payload = lifecycle_payload if lifecycle_payload is not None else load_json(lifecycle_path)
    return build_timeline_summary(payload, source_path=lifecycle_path)


def _load_timeline_summary(
    timeline_path: Path,
    *,
    lifecycle_path: Optional[Path] = None,
) -> Dict[str, Any]:
    summary = load_json(timeline_path)
    if lifecycle_path is None:
        raw = str(summary.get("source_lifecycle_artifact") or "").strip()
        if raw:
            candidate = Path(raw)
            if not candidate.exists():
                candidate = timeline_path.parent / Path(raw).name
            if candidate.exists():
                lifecycle_path = candidate
    if lifecycle_path is not None and "source_lifecycle_artifact" not in summary:
        summary = dict(summary)
        summary["source_lifecycle_artifact"] = str(lifecycle_path)
    return summary


def load_scene_context(input_path: Path) -> Dict[str, Any]:
    path = Path(input_path)
    timeline_path: Optional[Path] = None
    lifecycle_path: Optional[Path] = None

    if path.is_dir():
        for candidate in (
            path / DEFAULT_TIMELINE_JSON_NAME,
            path / "logs" / DEFAULT_TIMELINE_JSON_NAME,
        ):
            if candidate.exists():
                timeline_path = candidate
                break
        for candidate in (
            path / "online_topology_lifecycle_v0_1.json",
            path / "logs" / "online_topology_lifecycle_v0_1.json",
        ):
            if candidate.exists():
                lifecycle_path = candidate
                break
    elif path.name == "summary.json":
        lifecycle_path = resolve_lifecycle_artifact(path)
        candidate = lifecycle_path.parent / DEFAULT_TIMELINE_JSON_NAME
        if candidate.exists():
            timeline_path = candidate
    elif path.exists():
        payload = load_json(path) if path.suffix == ".json" else {}
        keys = set(payload.keys()) if isinstance(payload, dict) else set()
        if path.name == DEFAULT_TIMELINE_JSON_NAME or {"history_mode", "rooms"} <= keys:
            timeline_path = path
            raw = str(payload.get("source_lifecycle_artifact") or "").strip()
            if raw:
                lifecycle_candidate = Path(raw)
                if not lifecycle_candidate.exists():
                    lifecycle_candidate = path.parent / Path(raw).name
                if lifecycle_candidate.exists():
                    lifecycle_path = lifecycle_candidate
            if lifecycle_path is None:
                sibling = path.parent / "online_topology_lifecycle_v0_1.json"
                if sibling.exists():
                    lifecycle_path = sibling
        elif path.name == "online_topology_lifecycle_v0_1.json" or {"summary", "rooms"} <= keys:
            lifecycle_path = path
            sibling = path.parent / DEFAULT_TIMELINE_JSON_NAME
            if sibling.exists():
                timeline_path = sibling

    if timeline_path is None and lifecycle_path is None:
        raise FileNotFoundError(f"Could not resolve a timeline or lifecycle artifact from: {path}")

    lifecycle_payload = load_json(lifecycle_path) if lifecycle_path is not None else None
    if timeline_path is not None:
        timeline_summary = _load_timeline_summary(timeline_path, lifecycle_path=lifecycle_path)
        timeline_source = "artifact"
    elif lifecycle_path is not None:
        timeline_summary = _timeline_from_lifecycle(lifecycle_path, lifecycle_payload=lifecycle_payload)
        timeline_source = "derived_from_lifecycle"
    else:
        raise FileNotFoundError(f"Missing both timeline and lifecycle artifacts for: {path}")

    output_base_dir = (
        timeline_path.parent
        if timeline_path is not None
        else lifecycle_path.parent
        if lifecycle_path is not None
        else Path.cwd()
    )
    scene_id = _scene_id_from_path(timeline_path or lifecycle_path or path)
    return {
        "scene_id": scene_id,
        "sequence_id": timeline_summary.get("sequence_id"),
        "input_path": str(path),
        "timeline_path": str(timeline_path) if timeline_path is not None else None,
        "lifecycle_path": str(lifecycle_path) if lifecycle_path is not None else None,
        "timeline_source": timeline_source,
        "timeline_summary": timeline_summary,
        "lifecycle_payload": lifecycle_payload,
        "output_base_dir": str(output_base_dir),
    }


def _final_room_lookup(payload: Optional[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    if payload is None:
        return {}
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(payload.get("rooms") or []):
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _timeline_room_lookup(summary: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(summary.get("rooms") or []):
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _room_ids_from_refresh_history(refresh_history: Sequence[Dict[str, Any]]) -> Set[str]:
    room_ids: Set[str] = set()
    for refresh in refresh_history:
        for room in list(refresh.get("rooms") or []):
            room_id = room.get("room_id")
            if room_id is not None:
                room_ids.add(str(room_id))
    return room_ids


def _collect_room_refreshes(
    scene_context: Dict[str, Any],
    room_id: str,
) -> Tuple[List[Dict[str, Any]], List[str]]:
    lifecycle_payload = scene_context.get("lifecycle_payload")
    limitations: List[str] = []
    refreshes: List[Dict[str, Any]] = []

    if lifecycle_payload is not None:
        for refresh in list(lifecycle_payload.get("refresh_history") or []):
            refresh_room = None
            for room in list(refresh.get("rooms") or []):
                if str(room.get("room_id")) == room_id:
                    refresh_room = room
                    break
            if refresh_room is None:
                continue
            candidate_complete = bool(refresh_room.get("candidate_complete"))
            candidate_block_reasons = _sorted_str_list(refresh_room.get("candidate_block_reasons") or [])
            commit_block_reasons = _sorted_str_list(refresh_room.get("commit_block_reasons") or [])
            refreshes.append(
                {
                    "frame_idx": int(refresh.get("frame_idx")),
                    "timestamp": round(float(refresh.get("timestamp", 0.0)), 3),
                    "lifecycle_state": str(refresh_room.get("lifecycle_state") or "unknown"),
                    "candidate_complete": candidate_complete,
                    "candidate_block_reasons": candidate_block_reasons,
                    "commit_block_reasons": commit_block_reasons,
                    "effective_blockers": commit_block_reasons if candidate_complete else candidate_block_reasons,
                    "effective_blocker_source": "commit_block_reasons" if candidate_complete else "candidate_block_reasons",
                }
            )
        return refreshes, limitations

    timeline_lookup = _timeline_room_lookup(scene_context["timeline_summary"])
    room = timeline_lookup.get(room_id, {})
    for step in list(room.get("refresh_timeline") or []):
        commit_block_reasons = _sorted_str_list(step.get("commit_block_reasons") or [])
        candidate_complete = bool(step.get("candidate_complete"))
        refreshes.append(
            {
                "frame_idx": int(step.get("frame_idx")),
                "timestamp": round(float(step.get("timestamp", 0.0)), 3),
                "lifecycle_state": str(step.get("lifecycle_state") or "unknown"),
                "candidate_complete": candidate_complete,
                "candidate_block_reasons": [],
                "commit_block_reasons": commit_block_reasons,
                "effective_blockers": commit_block_reasons if not candidate_complete else commit_block_reasons,
                "effective_blocker_source": "commit_block_reasons_fallback",
            }
        )
    if refreshes:
        limitations.append(
            "Lifecycle refresh_history was unavailable, so candidate->blocked reasons fall back to timeline commit blockers when a room leaves candidate-complete."
        )
    return refreshes, limitations


def _first_true_frame(refreshes: Sequence[Dict[str, Any]], key: str) -> Optional[int]:
    for refresh in refreshes:
        if bool(refresh.get(key)):
            return int(refresh["frame_idx"])
    return None


def _last_true_frame(refreshes: Sequence[Dict[str, Any]], key: str) -> Optional[int]:
    for refresh in reversed(list(refreshes)):
        if bool(refresh.get(key)):
            return int(refresh["frame_idx"])
    return None


def _longest_run(refreshes: Sequence[Dict[str, Any]], *, value: bool) -> int:
    longest = 0
    current = 0
    for refresh in refreshes:
        if bool(refresh.get("candidate_complete")) is value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return int(longest)


def _blocker_metrics(
    refreshes: Sequence[Dict[str, Any]],
    final_unresolved_blockers: Sequence[str],
) -> Dict[str, Dict[str, Any]]:
    final_unresolved = set(final_unresolved_blockers)
    blocker_names = sorted(
        {
            blocker
            for refresh in refreshes
            for blocker in list(refresh.get("effective_blockers") or [])
        }
    )
    blocked_refresh_count = sum(1 for refresh in refreshes if list(refresh.get("effective_blockers") or []))
    metrics: Dict[str, Dict[str, Any]] = {}
    for blocker in blocker_names:
        flags = [blocker in set(refresh.get("effective_blockers") or []) for refresh in refreshes]
        refresh_count = int(sum(flags))
        episode_count = 0
        max_run_length = 0
        current_run = 0
        first_frame: Optional[int] = None
        last_frame: Optional[int] = None
        for refresh, present in zip(refreshes, flags):
            if present:
                frame_idx = int(refresh["frame_idx"])
                first_frame = frame_idx if first_frame is None else first_frame
                last_frame = frame_idx
                if current_run == 0:
                    episode_count += 1
                current_run += 1
                max_run_length = max(max_run_length, current_run)
            else:
                current_run = 0
        metrics[blocker] = {
            "refresh_count": refresh_count,
            "episode_count": int(episode_count),
            "max_run_length": int(max_run_length),
            "first_frame": first_frame,
            "last_frame": last_frame,
            "blocked_refresh_coverage": round(
                float(refresh_count) / float(blocked_refresh_count), 3
            )
            if blocked_refresh_count
            else 0.0,
            "final_unresolved": blocker in final_unresolved,
        }
    return metrics


def _classify_blockers(
    metrics: Dict[str, Dict[str, Any]],
    *,
    final_unresolved_blockers: Sequence[str],
) -> Tuple[List[str], List[str], Optional[str]]:
    persistent: List[str] = []
    transient: List[str] = []
    final_unresolved = set(final_unresolved_blockers)
    ordered = sorted(
        metrics.items(),
        key=lambda item: (
            -int(item[1]["refresh_count"]),
            0 if item[0] in final_unresolved else 1,
            -int(item[1]["max_run_length"]),
            item[0],
        ),
    )
    dominant_blocker = ordered[0][0] if ordered else None
    for blocker, data in ordered:
        is_persistent = blocker in final_unresolved or (
            int(data["refresh_count"]) >= PERSISTENT_BLOCKER_REFRESH_MIN
            and (
                int(data["max_run_length"]) >= PERSISTENT_BLOCKER_REFRESH_MIN
                or float(data["blocked_refresh_coverage"]) >= PERSISTENT_BLOCKER_COVERAGE_MIN
            )
        )
        if is_persistent:
            persistent.append(blocker)
        else:
            transient.append(blocker)
    return persistent, transient, dominant_blocker


def _room_final_state(
    final_room: Dict[str, Any],
    timeline_room: Dict[str, Any],
) -> Tuple[str, List[str]]:
    final_state = str(
        final_room.get("lifecycle_state")
        or timeline_room.get("final_lifecycle_state")
        or "unknown"
    )
    final_blockers = _sorted_str_list(
        final_room.get("commit_block_reasons")
        or timeline_room.get("final_blocker_set")
        or []
    )
    if final_state == "committed":
        final_blockers = []
    return final_state, final_blockers


def _frame_from_final_commit(
    scene_context: Dict[str, Any],
    final_state: str,
    timeline_room: Dict[str, Any],
) -> Optional[int]:
    if final_state != "committed":
        return None
    if timeline_room.get("first_committed_frame") is not None:
        return int(timeline_room["first_committed_frame"])
    lifecycle_payload = scene_context.get("lifecycle_payload")
    if lifecycle_payload is not None and lifecycle_payload.get("frame_idx") is not None:
        return int(lifecycle_payload["frame_idx"])
    timeline_summary = scene_context.get("timeline_summary") or {}
    frame_idx = timeline_summary.get("input_frame_idx")
    return None if frame_idx is None else int(frame_idx)


def _heuristic_final_blocker_classification(
    room_summary: Dict[str, Any],
    blocker_metrics: Dict[str, Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    blockers = list(room_summary.get("final_unresolved_blockers") or [])
    if not blockers:
        return None
    blocker_set = set(blockers)
    if blocker_set <= LIKELY_TRANSIENT_FINAL_BLOCKERS:
        classification = "likely_noisy_or_transient"
        reason = "Final blockers are limited to active-room / leave-signal style blockers."
    elif blocker_set & STRUCTURAL_FINAL_BLOCKERS:
        classification = "likely_structural"
        reason = "Final blockers include structural stability blockers or unresolved merge gating."
        if blocker_set & LIKELY_TRANSIENT_FINAL_BLOCKERS:
            classification = "mixed_or_unclear"
            reason = "Final blockers mix structural stability blockers with active-room / leave-signal blockers."
    else:
        persistent_fraction = 0.0
        if blockers:
            persistent_fraction = sum(
                1.0
                for blocker in blockers
                if blocker in set(room_summary.get("persistent_blockers") or [])
            ) / float(len(blockers))
        classification = "mixed_or_unclear"
        reason = "Final blockers do not map cleanly to the structural/transient heuristic buckets."
        if persistent_fraction >= 0.75:
            classification = "likely_structural"
            reason = "Most final blockers also behave persistently across blocked refreshes."
    return {
        "room_id": room_summary.get("room_id"),
        "classification": classification,
        "final_unresolved_blockers": blockers,
        "reason": reason,
        "heuristic": True,
        "persistent_final_blockers": [
            blocker
            for blocker in blockers
            if blocker_metrics.get(blocker, {}).get("final_unresolved")
        ],
    }


def build_scene_blocker_diagnosis(scene_context: Dict[str, Any]) -> Dict[str, Any]:
    timeline_summary = dict(scene_context["timeline_summary"])
    lifecycle_payload = scene_context.get("lifecycle_payload")
    final_rooms = _final_room_lookup(lifecycle_payload)
    timeline_rooms = _timeline_room_lookup(timeline_summary)
    if lifecycle_payload is not None:
        all_room_ids = sorted(set(final_rooms.keys()) | _room_ids_from_refresh_history(list(lifecycle_payload.get("refresh_history") or [])))
    else:
        all_room_ids = sorted(timeline_rooms.keys())

    room_summaries: List[Dict[str, Any]] = []
    scene_limitations: List[str] = list(timeline_summary.get("history_limitations") or [])
    blocker_frequency = defaultdict(lambda: {"blocked_refresh_count": 0, "rooms": set(), "candidate_but_blocked_refresh_count": 0, "final_unresolved_room_count": 0})
    blocker_persistence = defaultdict(lambda: {"persistent_room_count": 0, "transient_room_count": 0, "final_unresolved_room_count": 0})
    blocker_cooccurrence = defaultdict(lambda: {"blocked_refresh_count": 0, "rooms": set()})
    final_commit_blockers = Counter()
    heuristic_rows: List[Dict[str, Any]] = []

    for room_id in all_room_ids:
        final_room = final_rooms.get(room_id, {})
        timeline_room = timeline_rooms.get(room_id, {})
        refreshes, room_limitations = _collect_room_refreshes(scene_context, room_id)
        scene_limitations.extend(room_limitations)
        final_state, final_unresolved_blockers = _room_final_state(final_room, timeline_room)

        first_candidate_frame = _first_true_frame(refreshes, "candidate_complete") if refreshes else timeline_room.get("first_candidate_complete_frame")
        last_candidate_frame = _last_true_frame(refreshes, "candidate_complete") if refreshes else (
            timeline_room.get("first_candidate_complete_frame")
            if timeline_room.get("final_lifecycle_state") == "candidate_complete"
            else None
        )
        first_commit_ready_frame = (
            _first_true_frame(
                [
                    {
                        "frame_idx": refresh["frame_idx"],
                        "commit_ready": not list(refresh.get("commit_block_reasons") or []),
                    }
                    for refresh in refreshes
                ],
                "commit_ready",
            )
            if refreshes
            else timeline_room.get("first_commit_ready_frame")
        )
        first_committed_frame = None
        if refreshes:
            first_committed_frame = _first_true_frame(
                [
                    {
                        "frame_idx": refresh["frame_idx"],
                        "is_committed": str(refresh.get("lifecycle_state")) == "committed",
                    }
                    for refresh in refreshes
                ],
                "is_committed",
            )
        if first_committed_frame is None:
            first_committed_frame = _frame_from_final_commit(scene_context, final_state, timeline_room)

        candidate_to_blocked_events: List[Dict[str, Any]] = []
        blocked_to_candidate_events: List[Dict[str, Any]] = []
        candidate_to_blocked_transition_count = 0
        blocked_to_candidate_transition_count = 0
        total_candidate_refreshes: Optional[int] = None
        total_candidate_but_blocked_refreshes: Optional[int] = None
        total_commit_ready_refreshes: Optional[int] = None
        longest_stable_window_in_refreshes: Optional[int] = None
        longest_blocked_window_in_refreshes: Optional[int] = None

        if refreshes:
            total_candidate_refreshes = int(sum(1 for refresh in refreshes if bool(refresh.get("candidate_complete"))))
            total_candidate_but_blocked_refreshes = int(
                sum(
                    1
                    for refresh in refreshes
                    if bool(refresh.get("candidate_complete")) and list(refresh.get("commit_block_reasons") or [])
                )
            )
            total_commit_ready_refreshes = int(
                sum(1 for refresh in refreshes if not list(refresh.get("commit_block_reasons") or []))
            )
            longest_stable_window_in_refreshes = _longest_run(refreshes, value=True)
            longest_blocked_window_in_refreshes = _longest_run(refreshes, value=False)
            previous_candidate: Optional[bool] = None
            for refresh in refreshes:
                candidate_complete = bool(refresh.get("candidate_complete"))
                if previous_candidate is True and not candidate_complete:
                    candidate_to_blocked_transition_count += 1
                    candidate_to_blocked_events.append(
                        {
                            "frame_idx": int(refresh["frame_idx"]),
                            "lifecycle_state": refresh.get("lifecycle_state"),
                            "candidate_block_reasons": list(refresh.get("candidate_block_reasons") or []),
                            "commit_block_reasons": list(refresh.get("commit_block_reasons") or []),
                            "effective_blocker_source": refresh.get("effective_blocker_source"),
                        }
                    )
                if previous_candidate is False and candidate_complete:
                    blocked_to_candidate_transition_count += 1
                    blocked_to_candidate_events.append(
                        {
                            "frame_idx": int(refresh["frame_idx"]),
                            "lifecycle_state": refresh.get("lifecycle_state"),
                            "commit_block_reasons": list(refresh.get("commit_block_reasons") or []),
                        }
                    )
                previous_candidate = candidate_complete

        blocker_metrics = _blocker_metrics(refreshes, final_unresolved_blockers)
        persistent_blockers, transient_blockers, dominant_blocker = _classify_blockers(
            blocker_metrics,
            final_unresolved_blockers=final_unresolved_blockers,
        )

        room_summary = {
            "room_id": room_id,
            "history_available": bool(refreshes),
            "refresh_count": int(len(refreshes)),
            "first_seen_frame": final_room.get("first_seen_frame_idx", timeline_room.get("first_seen_frame")),
            "first_candidate_frame": first_candidate_frame,
            "last_candidate_frame": last_candidate_frame,
            "first_commit_ready_frame": first_commit_ready_frame,
            "first_committed_frame": first_committed_frame,
            "final_state": final_state,
            "candidate_to_blocked_transition_count": int(candidate_to_blocked_transition_count),
            "blocked_to_candidate_transition_count": int(blocked_to_candidate_transition_count),
            "total_candidate_refreshes": total_candidate_refreshes,
            "total_candidate_but_blocked_refreshes": total_candidate_but_blocked_refreshes,
            "total_commit_ready_refreshes": total_commit_ready_refreshes,
            "dominant_blocker": dominant_blocker,
            "persistent_blockers": persistent_blockers,
            "transient_blockers": transient_blockers,
            "final_unresolved_blockers": final_unresolved_blockers,
            "longest_stable_window_in_refreshes": longest_stable_window_in_refreshes,
            "longest_blocked_window_in_refreshes": longest_blocked_window_in_refreshes,
            "candidate_to_blocked_events": candidate_to_blocked_events,
            "blocked_to_candidate_events": blocked_to_candidate_events,
        }
        room_summaries.append(room_summary)

        for refresh in refreshes:
            effective_blockers = list(refresh.get("effective_blockers") or [])
            if bool(refresh.get("candidate_complete")) and list(refresh.get("commit_block_reasons") or []):
                for blocker in list(refresh.get("commit_block_reasons") or []):
                    blocker_frequency[blocker]["candidate_but_blocked_refresh_count"] += 1
            for blocker in effective_blockers:
                blocker_frequency[blocker]["blocked_refresh_count"] += 1
                blocker_frequency[blocker]["rooms"].add(room_id)
            for blocker_a, blocker_b in combinations(sorted(set(effective_blockers)), 2):
                pair_key = f"{blocker_a} + {blocker_b}"
                blocker_cooccurrence[pair_key]["blocked_refresh_count"] += 1
                blocker_cooccurrence[pair_key]["rooms"].add(room_id)
        for blocker in persistent_blockers:
            blocker_persistence[blocker]["persistent_room_count"] += 1
        for blocker in transient_blockers:
            blocker_persistence[blocker]["transient_room_count"] += 1
        for blocker in final_unresolved_blockers:
            blocker_frequency[blocker]["final_unresolved_room_count"] += 1
            blocker_persistence[blocker]["final_unresolved_room_count"] += 1
            final_commit_blockers[blocker] += 1

        heuristic_row = _heuristic_final_blocker_classification(room_summary, blocker_metrics)
        if heuristic_row is not None:
            heuristic_rows.append(heuristic_row)

    rooms_reached_candidate_but_never_committed = [
        room["room_id"]
        for room in room_summaries
        if room.get("first_candidate_frame") is not None and room.get("first_committed_frame") is None
    ]
    rooms_committed_after_repeated_reblocking = [
        room["room_id"]
        for room in room_summaries
        if room.get("first_committed_frame") is not None and int(room.get("candidate_to_blocked_transition_count") or 0) > 0
    ]
    rooms_near_stable_but_repeatedly_reblocked = [
        {
            "room_id": room["room_id"],
            "candidate_to_blocked_transition_count": room["candidate_to_blocked_transition_count"],
            "longest_stable_window_in_refreshes": room["longest_stable_window_in_refreshes"],
            "final_state": room["final_state"],
        }
        for room in room_summaries
        if room.get("first_candidate_frame") is not None and int(room.get("candidate_to_blocked_transition_count") or 0) > 0
    ]

    blocker_frequency_summary = [
        {
            "blocker": blocker,
            "blocked_refresh_count": int(values["blocked_refresh_count"]),
            "rooms_affected_count": int(len(values["rooms"])),
            "candidate_but_blocked_refresh_count": int(values["candidate_but_blocked_refresh_count"]),
            "final_unresolved_room_count": int(values["final_unresolved_room_count"]),
        }
        for blocker, values in blocker_frequency.items()
    ]
    blocker_frequency_summary.sort(
        key=lambda item: (
            -int(item["blocked_refresh_count"]),
            -int(item["final_unresolved_room_count"]),
            -int(item["rooms_affected_count"]),
            item["blocker"],
        )
    )

    blocker_persistence_summary = [
        {
            "blocker": blocker,
            "persistent_room_count": int(values["persistent_room_count"]),
            "transient_room_count": int(values["transient_room_count"]),
            "final_unresolved_room_count": int(values["final_unresolved_room_count"]),
        }
        for blocker, values in blocker_persistence.items()
    ]
    blocker_persistence_summary.sort(
        key=lambda item: (
            -int(item["persistent_room_count"]),
            -int(item["final_unresolved_room_count"]),
            item["blocker"],
        )
    )

    blocker_cooccurrence_summary = [
        {
            "blockers": pair.split(" + "),
            "blocked_refresh_count": int(values["blocked_refresh_count"]),
            "room_count": int(len(values["rooms"])),
        }
        for pair, values in blocker_cooccurrence.items()
    ]
    blocker_cooccurrence_summary.sort(
        key=lambda item: (
            -int(item["blocked_refresh_count"]),
            -int(item["room_count"]),
            tuple(item["blockers"]),
        )
    )

    final_commit_blocker_summary = [
        {
            "blocker": blocker,
            "final_unresolved_room_count": int(count),
        }
        for blocker, count in final_commit_blockers.items()
    ]
    final_commit_blocker_summary.sort(
        key=lambda item: (-int(item["final_unresolved_room_count"]), item["blocker"])
    )

    scene_refresh_count = 0
    if lifecycle_payload is not None:
        scene_refresh_count = int(len(lifecycle_payload.get("refresh_history") or []))
    elif timeline_summary.get("refresh_count") is not None:
        scene_refresh_count = int(timeline_summary["refresh_count"])

    committed_room_count = int(sum(1 for room in room_summaries if room["final_state"] == "committed"))
    return {
        "scene_id": scene_context.get("scene_id"),
        "sequence_id": scene_context.get("sequence_id"),
        "history_mode": "lifecycle_refresh_history"
        if lifecycle_payload is not None and lifecycle_payload.get("refresh_history")
        else str(timeline_summary.get("history_mode") or "unknown"),
        "history_limitations": sorted(set(scene_limitations)),
        "source_artifacts": {
            "input_path": scene_context.get("input_path"),
            "timeline_artifact": scene_context.get("timeline_path"),
            "lifecycle_artifact": scene_context.get("lifecycle_path"),
            "timeline_source": scene_context.get("timeline_source"),
        },
        "room_count": int(len(room_summaries)),
        "refresh_count": scene_refresh_count,
        "committed_room_count": committed_room_count,
        "rooms": room_summaries,
        "blocker_frequency_summary": blocker_frequency_summary,
        "blocker_persistence_summary": blocker_persistence_summary,
        "blocker_cooccurrence_summary": blocker_cooccurrence_summary,
        "final_commit_blocker_summary": final_commit_blocker_summary,
        "rooms_reached_candidate_but_never_committed": rooms_reached_candidate_but_never_committed,
        "rooms_committed_after_repeated_reblocking": rooms_committed_after_repeated_reblocking,
        "rooms_near_stable_but_repeatedly_reblocked": rooms_near_stable_but_repeatedly_reblocked,
        "final_blocker_heuristics": heuristic_rows,
    }


def _aggregate_scene_lists(
    scene_summaries: Sequence[Dict[str, Any]],
    key: str,
) -> List[str]:
    values: List[str] = []
    for scene in scene_summaries:
        for room_id in list(scene.get(key) or []):
            values.append(f"{scene.get('scene_id')}:{room_id}")
    return values


def build_blocker_diagnosis(scene_contexts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    scene_summaries = [build_scene_blocker_diagnosis(scene_context) for scene_context in scene_contexts]
    scene_frequency = defaultdict(lambda: {"blocked_refresh_count": 0, "scene_count": 0, "room_count": 0, "final_unresolved_room_count": 0})
    seen_frequency_scenes: Dict[str, Set[str]] = defaultdict(set)
    scene_cooccurrence = defaultdict(lambda: {"blocked_refresh_count": 0, "scene_count": 0})
    seen_cooccurrence_scenes: Dict[str, Set[str]] = defaultdict(set)
    scene_persistence = defaultdict(lambda: {"persistent_room_count": 0, "transient_room_count": 0, "final_unresolved_room_count": 0})
    final_commit_blockers = Counter()

    for scene in scene_summaries:
        scene_id = str(scene.get("scene_id"))
        for item in list(scene.get("blocker_frequency_summary") or []):
            blocker = str(item["blocker"])
            scene_frequency[blocker]["blocked_refresh_count"] += int(item["blocked_refresh_count"])
            scene_frequency[blocker]["room_count"] += int(item["rooms_affected_count"])
            scene_frequency[blocker]["final_unresolved_room_count"] += int(item["final_unresolved_room_count"])
            if scene_id not in seen_frequency_scenes[blocker]:
                seen_frequency_scenes[blocker].add(scene_id)
                scene_frequency[blocker]["scene_count"] += 1
        for item in list(scene.get("blocker_cooccurrence_summary") or []):
            pair_key = " + ".join(item["blockers"])
            scene_cooccurrence[pair_key]["blocked_refresh_count"] += int(item["blocked_refresh_count"])
            if scene_id not in seen_cooccurrence_scenes[pair_key]:
                seen_cooccurrence_scenes[pair_key].add(scene_id)
                scene_cooccurrence[pair_key]["scene_count"] += 1
        for item in list(scene.get("blocker_persistence_summary") or []):
            blocker = str(item["blocker"])
            scene_persistence[blocker]["persistent_room_count"] += int(item["persistent_room_count"])
            scene_persistence[blocker]["transient_room_count"] += int(item["transient_room_count"])
            scene_persistence[blocker]["final_unresolved_room_count"] += int(item["final_unresolved_room_count"])
        for item in list(scene.get("final_commit_blocker_summary") or []):
            final_commit_blockers[str(item["blocker"])] += int(item["final_unresolved_room_count"])

    aggregate_frequency_summary = [
        {
            "blocker": blocker,
            "blocked_refresh_count": int(values["blocked_refresh_count"]),
            "scene_count": int(values["scene_count"]),
            "room_count": int(values["room_count"]),
            "final_unresolved_room_count": int(values["final_unresolved_room_count"]),
        }
        for blocker, values in scene_frequency.items()
    ]
    aggregate_frequency_summary.sort(
        key=lambda item: (
            -int(item["blocked_refresh_count"]),
            -int(item["final_unresolved_room_count"]),
            -int(item["scene_count"]),
            item["blocker"],
        )
    )

    aggregate_persistence_summary = [
        {
            "blocker": blocker,
            "persistent_room_count": int(values["persistent_room_count"]),
            "transient_room_count": int(values["transient_room_count"]),
            "final_unresolved_room_count": int(values["final_unresolved_room_count"]),
        }
        for blocker, values in scene_persistence.items()
    ]
    aggregate_persistence_summary.sort(
        key=lambda item: (
            -int(item["persistent_room_count"]),
            -int(item["final_unresolved_room_count"]),
            item["blocker"],
        )
    )

    aggregate_cooccurrence_summary = [
        {
            "blockers": pair.split(" + "),
            "blocked_refresh_count": int(values["blocked_refresh_count"]),
            "scene_count": int(values["scene_count"]),
        }
        for pair, values in scene_cooccurrence.items()
    ]
    aggregate_cooccurrence_summary.sort(
        key=lambda item: (
            -int(item["blocked_refresh_count"]),
            -int(item["scene_count"]),
            tuple(item["blockers"]),
        )
    )

    aggregate_final_commit_blocker_summary = [
        {
            "blocker": blocker,
            "final_unresolved_room_count": int(count),
        }
        for blocker, count in final_commit_blockers.items()
    ]
    aggregate_final_commit_blocker_summary.sort(
        key=lambda item: (-int(item["final_unresolved_room_count"]), item["blocker"])
    )

    return {
        "version": "0.1",
        "diagnosis_mode": "debug_only_post_hoc",
        "scene_count": int(len(scene_summaries)),
        "input_contract": {
            "preferred_artifacts": [
                "logs/online_topology_lifecycle_v0_1.json",
                "logs/online_topology_timeline_eval_v0_1.json",
            ],
            "minimal_stable_lifecycle_fields": {
                "top_level": [
                    "sequence_id",
                    "frame_idx",
                    "timestamp",
                    "rooms",
                    "refresh_history",
                ],
                "final_room_fields": [
                    "room_id",
                    "first_seen_frame_idx",
                    "lifecycle_state",
                    "commit_block_reasons",
                ],
                "refresh_room_fields": [
                    "room_id",
                    "lifecycle_state",
                    "candidate_complete",
                    "candidate_block_reasons",
                    "commit_block_reasons",
                ],
                "refresh_fields": [
                    "frame_idx",
                    "timestamp",
                    "rooms",
                ],
            },
            "timeline_only_fallback_fields": {
                "room_fields": [
                    "room_id",
                    "first_candidate_complete_frame",
                    "first_commit_ready_frame",
                    "first_committed_frame",
                    "final_lifecycle_state",
                    "final_blocker_set",
                    "refresh_timeline",
                ],
                "refresh_fields": [
                    "frame_idx",
                    "timestamp",
                    "lifecycle_state",
                    "candidate_complete",
                    "commit_ready",
                    "commit_block_reasons",
                ],
            },
            "notes": [
                "Diagnosis stays offline/debug-only and does not change runtime lifecycle behavior.",
                "Candidate->blocked attribution is strongest when lifecycle refresh_history is present because it retains candidate_block_reasons.",
            ],
        },
        "heuristic_notes": [
            "Persistent vs transient blocker labels use conservative offline thresholds over blocked refresh count, contiguous run length, and whether a blocker remains unresolved at the final frame.",
            "Structural vs noisy/transient final-blocker labels are heuristic and intended only for teacher-facing diagnosis.",
        ],
        "interpretation_notes": [
            "blocker_frequency_summary counts blocked refreshes across the observed room history and can therefore include bootstrap-stage blockers before a room becomes candidate-complete.",
            "final_commit_blocker_summary isolates blockers still unresolved in the final room state and is the safer summary for commit-prevention analysis.",
        ],
        "scenes": scene_summaries,
        "aggregate_summary": {
            "blocker_frequency_summary": aggregate_frequency_summary,
            "blocker_persistence_summary": aggregate_persistence_summary,
            "blocker_cooccurrence_summary": aggregate_cooccurrence_summary,
            "final_commit_blocker_summary": aggregate_final_commit_blocker_summary,
            "rooms_reached_candidate_but_never_committed": _aggregate_scene_lists(
                scene_summaries,
                "rooms_reached_candidate_but_never_committed",
            ),
            "rooms_committed_after_repeated_reblocking": _aggregate_scene_lists(
                scene_summaries,
                "rooms_committed_after_repeated_reblocking",
            ),
        },
    }


def _fmt_frame(value: Optional[int]) -> str:
    return "n/a" if value is None else str(int(value))


def _fmt_list(values: Sequence[str]) -> str:
    return ", ".join(str(value) for value in values) if values else "-"


def _top_items(rows: Sequence[Dict[str, Any]], key: str, *, limit: int = 5) -> str:
    clipped = list(rows[:limit])
    if not clipped:
        return "-"
    values = []
    for row in clipped:
        values.append(f"{row.get(key)} ({row.get('blocked_refresh_count') or row.get('final_unresolved_room_count')})")
    return "; ".join(values)


def render_blocker_diag_markdown(report: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append("# Online Topology Blocker Diagnosis")
    lines.append("")
    lines.append(f"- version: {report.get('version')}")
    lines.append(f"- diagnosis_mode: {report.get('diagnosis_mode')}")
    lines.append(f"- scene_count: {report.get('scene_count')}")
    lines.append("")
    lines.append("## Aggregate")
    lines.append("")
    aggregate = report.get("aggregate_summary") or {}
    for note in list(report.get("interpretation_notes") or []):
        lines.append(f"- note: {note}")
    lines.append(f"- top blocked-phase blocker frequency: {_top_items(aggregate.get('blocker_frequency_summary') or [], 'blocker')}")
    lines.append(f"- top final commit blockers: {_top_items(aggregate.get('final_commit_blocker_summary') or [], 'blocker')}")
    lines.append(
        f"- candidate but never committed: {_fmt_list(aggregate.get('rooms_reached_candidate_but_never_committed') or [])}"
    )
    lines.append(
        f"- committed after repeated re-blocking: {_fmt_list(aggregate.get('rooms_committed_after_repeated_reblocking') or [])}"
    )

    for scene in list(report.get("scenes") or []):
        lines.append("")
        lines.append(f"## Scene `{scene.get('scene_id')}`")
        lines.append("")
        lines.append(f"- sequence_id: {scene.get('sequence_id') or 'n/a'}")
        lines.append(f"- history_mode: {scene.get('history_mode')}")
        lines.append(f"- refresh_count: {scene.get('refresh_count')}")
        lines.append(f"- room_count: {scene.get('room_count')}")
        lines.append(f"- committed_room_count: {scene.get('committed_room_count')}")
        lines.append(f"- source_timeline: {scene.get('source_artifacts', {}).get('timeline_artifact') or 'derived'}")
        lines.append(f"- source_lifecycle: {scene.get('source_artifacts', {}).get('lifecycle_artifact') or 'n/a'}")
        for limitation in list(scene.get("history_limitations") or []):
            lines.append(f"- note: {limitation}")
        lines.append("")
        lines.append("| room_id | first_candidate | last_candidate | first_commit_ready | first_committed | final_state | cand->blocked | blocked->cand | cand_blocked_refreshes | dominant_blocker | final_unresolved |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for room in list(scene.get("rooms") or []):
            lines.append(
                "| {room_id} | {first_candidate} | {last_candidate} | {first_commit_ready} | {first_committed} | {final_state} | {cand_to_blocked} | {blocked_to_cand} | {cand_blocked_refreshes} | {dominant_blocker} | {final_unresolved} |".format(
                    room_id=room.get("room_id"),
                    first_candidate=_fmt_frame(room.get("first_candidate_frame")),
                    last_candidate=_fmt_frame(room.get("last_candidate_frame")),
                    first_commit_ready=_fmt_frame(room.get("first_commit_ready_frame")),
                    first_committed=_fmt_frame(room.get("first_committed_frame")),
                    final_state=room.get("final_state"),
                    cand_to_blocked=room.get("candidate_to_blocked_transition_count"),
                    blocked_to_cand=room.get("blocked_to_candidate_transition_count"),
                    cand_blocked_refreshes=room.get("total_candidate_but_blocked_refreshes")
                    if room.get("total_candidate_but_blocked_refreshes") is not None
                    else "n/a",
                    dominant_blocker=room.get("dominant_blocker") or "-",
                    final_unresolved=_fmt_list(room.get("final_unresolved_blockers") or []),
                )
            )
        lines.append("")
        lines.append(f"- blocked-phase blocker frequency: {_top_items(scene.get('blocker_frequency_summary') or [], 'blocker')}")
        lines.append(f"- final commit blockers: {_top_items(scene.get('final_commit_blocker_summary') or [], 'blocker')}")
        lines.append(
            f"- candidate but never committed: {_fmt_list(scene.get('rooms_reached_candidate_but_never_committed') or [])}"
        )
        lines.append(
            f"- committed after repeated re-blocking: {_fmt_list(scene.get('rooms_committed_after_repeated_reblocking') or [])}"
        )
        near_stable_rooms = [
            f"{item.get('room_id')} ({item.get('candidate_to_blocked_transition_count')} re-blocks)"
            for item in list(scene.get("rooms_near_stable_but_repeatedly_reblocked") or [])
        ]
        lines.append(f"- near-stable but repeatedly re-blocked: {_fmt_list(near_stable_rooms)}")
        heuristic_rows = list(scene.get("final_blocker_heuristics") or [])
        if heuristic_rows:
            lines.append("- heuristic final-blocker classification:")
            for row in heuristic_rows:
                lines.append(
                    "  - {room_id}: {classification}; blockers [{blockers}]".format(
                        room_id=row.get("room_id"),
                        classification=row.get("classification"),
                        blockers=_fmt_list(row.get("final_unresolved_blockers") or []),
                    )
                )
    return "\n".join(lines)
