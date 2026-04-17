from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .online_topology_lifecycle import derive_publication_diagnostics_from_payload
from .online_topology_timeline_eval import resolve_lifecycle_artifact


DEFAULT_JSON_NAME = "room_commit_diagnosis_v0_1.json"
DEFAULT_MARKDOWN_NAME = "room_commit_diagnosis_v0_1.md"
DEFAULT_STABILITY_REFRESH_THRESHOLD = 2
DEFAULT_CANDIDATE_READINESS_THRESHOLD = 4


def load_json(path: Path) -> Dict[str, Any]:
    return dict(json.loads(Path(path).read_text(encoding="utf-8")))


def load_optional_json(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    candidate = Path(path)
    if not candidate.exists():
        return None
    return load_json(candidate)


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_markdown(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text).rstrip() + "\n", encoding="utf-8")


def default_output_paths(base_dir: Path) -> Tuple[Path, Path]:
    base_dir = Path(base_dir)
    return base_dir / DEFAULT_JSON_NAME, base_dir / DEFAULT_MARKDOWN_NAME


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _canonical_room_id(value: Any) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.startswith("room_"):
        suffix = text[5:]
        if suffix.lstrip("-").isdigit() and int(suffix) < 0:
            return None
        return text
    if text.lstrip("-").isdigit():
        numeric_id = int(text)
        if numeric_id < 0:
            return None
        return f"room_{numeric_id}"
    return text


def _sorted_counter(counter: Counter) -> List[Dict[str, Any]]:
    return [
        {"name": str(name), "count": int(count)}
        for name, count in sorted(counter.items(), key=lambda item: (-int(item[1]), str(item[0])))
    ]


def _room_ids_from_rooms(rooms: Sequence[Dict[str, Any]]) -> List[str]:
    room_ids: List[str] = []
    seen = set()
    for room in list(rooms or []):
        room_id = _canonical_room_id(room.get("room_id") or room.get("id"))
        if room_id is None or room_id in seen:
            continue
        seen.add(room_id)
        room_ids.append(room_id)
    return room_ids


def _room_lookup(rooms: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(rooms or []):
        room_id = _canonical_room_id(room.get("room_id") or room.get("id"))
        if room_id is None:
            continue
        lookup[room_id] = dict(room)
    return lookup


def _path_from_summary(
    summary_payload: Optional[Dict[str, Any]],
    *,
    key: str,
    log_dir: Path,
    default_name: str,
) -> Optional[Path]:
    if summary_payload is not None:
        raw = _clean_optional_text(summary_payload.get(key))
        if raw is not None:
            candidate = Path(raw)
            if not candidate.is_absolute():
                candidate = log_dir / candidate.name
            if candidate.exists():
                return candidate
    candidate = log_dir / default_name
    return candidate if candidate.exists() else None


def load_scene_artifacts(input_path: Path) -> Dict[str, Any]:
    path = Path(input_path)
    if path.is_dir():
        if path.name == "logs":
            log_dir = path
            scene_root = path.parent
        elif (path / "logs").is_dir():
            scene_root = path
            log_dir = path / "logs"
        else:
            scene_root = path
            log_dir = path
    else:
        if path.name == "summary.json":
            log_dir = path.parent
        else:
            log_dir = path.parent
        scene_root = log_dir.parent if log_dir.name == "logs" else log_dir

    summary_path = log_dir / "summary.json"
    if path.is_file() and path.name == "summary.json":
        summary_path = path
    summary_payload = load_optional_json(summary_path)

    lifecycle_path: Optional[Path] = None
    try:
        lifecycle_path = resolve_lifecycle_artifact(path if path.exists() else log_dir)
    except FileNotFoundError:
        lifecycle_path = _path_from_summary(
            summary_payload,
            key="online_topology_lifecycle_json",
            log_dir=log_dir,
            default_name="online_topology_lifecycle_v0_1.json",
        )
    if lifecycle_path is None:
        raise FileNotFoundError(f"Could not resolve lifecycle artifact from: {path}")

    runtime_state_path = _path_from_summary(
        summary_payload,
        key="room_scoped_runtime_state_json",
        log_dir=log_dir,
        default_name="room_scoped_runtime_state_v0_1.json",
    )
    topology_path = _path_from_summary(
        summary_payload,
        key="topology_v0_1_json",
        log_dir=log_dir,
        default_name="topology_v0_1.json",
    )
    topology_query_report_path = _path_from_summary(
        summary_payload,
        key="topology_query_report_json",
        log_dir=log_dir,
        default_name="topology_query_report.json",
    )
    committed_room_world_model_path = _path_from_summary(
        summary_payload,
        key="committed_room_world_model_json",
        log_dir=log_dir,
        default_name="committed_room_world_model_v0_1.json",
    )

    lifecycle_payload = load_json(lifecycle_path)
    runtime_state_payload = load_optional_json(runtime_state_path)
    topology_payload = load_optional_json(topology_path)
    topology_query_report_payload = load_optional_json(topology_query_report_path)
    committed_room_world_model_payload = load_optional_json(committed_room_world_model_path)

    return {
        "input_path": str(path),
        "scene_root": str(scene_root),
        "log_dir": str(log_dir),
        "summary_path": None if not summary_path.exists() else str(summary_path),
        "lifecycle_path": str(lifecycle_path),
        "runtime_state_path": None if runtime_state_path is None else str(runtime_state_path),
        "topology_path": None if topology_path is None else str(topology_path),
        "topology_query_report_path": None if topology_query_report_path is None else str(topology_query_report_path),
        "committed_room_world_model_path": None
        if committed_room_world_model_path is None
        else str(committed_room_world_model_path),
        "summary_payload": summary_payload,
        "lifecycle_payload": lifecycle_payload,
        "runtime_state_payload": runtime_state_payload,
        "topology_payload": topology_payload,
        "topology_query_report_payload": topology_query_report_payload,
        "committed_room_world_model_payload": committed_room_world_model_payload,
    }


def _find_refresh_room(refresh: Dict[str, Any], room_id: str) -> Optional[Dict[str, Any]]:
    for room in list(refresh.get("rooms") or []):
        if _canonical_room_id(room.get("room_id")) == room_id:
            return dict(room)
    return None


def _publication_view(room_payload: Dict[str, Any], *, active_room_id: Optional[str]) -> Dict[str, Any]:
    if room_payload.get("publication_state"):
        return {
            "publication_state": str(room_payload.get("publication_state")),
            "candidate_room_formed_v1": bool(room_payload.get("candidate_room_formed_v1")),
            "leave_like_signal_v1": bool(room_payload.get("leave_like_signal_v1")),
            "leave_like_signal_v1_source": room_payload.get("leave_like_signal_v1_source"),
            "finalization_blockers": list(room_payload.get("finalization_blockers") or []),
            "publication_blockers": list(room_payload.get("publication_blockers") or []),
            "finalized_private": bool(room_payload.get("finalized_private")),
            "commit_ready": bool(room_payload.get("commit_ready")),
            "published": bool(room_payload.get("published")),
        }
    return derive_publication_diagnostics_from_payload(
        dict(room_payload),
        active_room_id=active_room_id,
        stability_refresh_threshold=DEFAULT_STABILITY_REFRESH_THRESHOLD,
    )


def _transition_summary(refresh_history: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    active_trace: List[str] = []
    transition_count = 0
    previous_active: Optional[str] = None
    for refresh in list(refresh_history or []):
        current_active = _canonical_room_id(refresh.get("active_room_id"))
        if current_active is None:
            continue
        if not active_trace or active_trace[-1] != current_active:
            active_trace.append(current_active)
        if previous_active is not None and current_active != previous_active:
            transition_count += 1
        previous_active = current_active
    return {
        "active_room_trace": active_trace,
        "unique_active_room_count": int(len(active_trace)),
        "room_transition_event_count": int(transition_count),
        "leave_like_transition_observed": bool(transition_count > 0),
    }


def _aggregate_top_names(counter_rows: Sequence[Dict[str, Any]], limit: int = 5) -> List[str]:
    return [str(row.get("name")) for row in list(counter_rows or [])[:limit] if row.get("name")]


def _classify_empty_public_result(
    *,
    public_room_count: int,
    candidate_room_formed_count: int,
    candidate_complete_count: int,
    internal_committed_room_count: int,
) -> Tuple[str, str]:
    if public_room_count > 0:
        return (
            "public_committed_rooms_available",
            "At least one room survived the committed/public filter, so the public topology is non-empty.",
        )
    if internal_committed_room_count > 0:
        return (
            "commit_succeeded_but_public_export_or_filter_removed_everything",
            "A room was committed internally, but zero committed rooms survived into the final public topology/query export.",
        )
    if candidate_room_formed_count <= 0:
        return (
            "no_useful_room_candidates",
            "No room ever progressed past early observation into a candidate_room_formed lifecycle state.",
        )
    if candidate_complete_count <= 0:
        return (
            "candidates_seen_but_never_candidate_complete",
            "Candidate rooms were seen, but none ever satisfied candidate_complete readiness.",
        )
    if internal_committed_room_count <= 0:
        return (
            "candidate_complete_reached_but_commit_blocked",
            "At least one room reached candidate_complete, but no room cleared commit blockers strongly enough to be committed internally.",
        )
    return (
        "candidate_complete_reached_but_commit_blocked",
        "At least one room reached candidate_complete, but no room cleared commit blockers strongly enough to be committed internally.",
    )


def _minimum_replay_standard(checks: Dict[str, bool]) -> Dict[str, Any]:
    requirements = [
        {
            "id": "multi_refresh_room_observation",
            "rule": (
                "At least one room should remain present across two or more export refreshes so the "
                "runtime can accumulate room/gateway/containment stability evidence."
            ),
            "satisfied": bool(checks.get("multi_refresh_room_observation")),
        },
        {
            "id": "room_transition_or_departure",
            "rule": (
                "The replay should contain a real room exit or room-to-room transition so one room is no longer the active room."
            ),
            "satisfied": bool(checks.get("room_transition_or_departure")),
        },
        {
            "id": "candidate_complete_room",
            "rule": (
                "At least one room should reach candidate_complete, which in this runtime means a readiness score of "
                f"{DEFAULT_CANDIDATE_READINESS_THRESHOLD} with stable room signature, gateway structure, containment, and floor assignment."
            ),
            "satisfied": bool(checks.get("candidate_complete_room")),
        },
        {
            "id": "commit_ready_room",
            "rule": (
                "After the last structural delta, the replay should include an additional stable refresh so merge/gateway blockers clear and "
                "a room can become commit-ready."
            ),
            "satisfied": bool(checks.get("commit_ready_room")),
        },
        {
            "id": "public_committed_room",
            "rule": "For demo-ready sequences, at least one internally committed room should survive the committed/public export filter.",
            "satisfied": bool(checks.get("public_committed_room")),
        },
    ]
    return {
        "derived_from_runtime_behavior": True,
        "stability_refresh_threshold": int(DEFAULT_STABILITY_REFRESH_THRESHOLD),
        "candidate_readiness_threshold": int(DEFAULT_CANDIDATE_READINESS_THRESHOLD),
        "checks": dict(checks),
        "requirements": requirements,
    }


def build_room_commit_diagnosis(scene_artifacts: Dict[str, Any]) -> Dict[str, Any]:
    lifecycle_payload = dict(scene_artifacts.get("lifecycle_payload") or {})
    runtime_state_payload = dict(scene_artifacts.get("runtime_state_payload") or {})
    topology_payload = dict(scene_artifacts.get("topology_payload") or {})
    topology_query_report_payload = dict(scene_artifacts.get("topology_query_report_payload") or {})
    committed_room_world_model_payload = dict(scene_artifacts.get("committed_room_world_model_payload") or {})
    summary_payload = dict(scene_artifacts.get("summary_payload") or {})

    refresh_history = list(lifecycle_payload.get("refresh_history") or [])
    final_rooms_lookup = _room_lookup(lifecycle_payload.get("rooms") or [])

    runtime_committed_room_ids = sorted(
        _room_ids_from_rooms(
            [{"room_id": room_id} for room_id in list(runtime_state_payload.get("committed_global_memory", {}).get("room_ids") or [])]
        )
    )
    lifecycle_committed_room_ids = sorted(
        _room_ids_from_rooms(
            [{"room_id": room_id} for room_id in list(lifecycle_payload.get("committed_rooms") or [])]
        )
    )
    world_model_room_ids = sorted(
        _room_ids_from_rooms(
            [{"room_id": room.get("room_id")} for room in list(committed_room_world_model_payload.get("rooms") or [])]
        )
    )
    internal_committed_room_ids = sorted(
        set(runtime_committed_room_ids) | set(lifecycle_committed_room_ids) | set(world_model_room_ids)
    )
    public_room_ids = sorted(_room_ids_from_rooms(topology_payload.get("rooms") or []))
    public_query_inspection = dict(topology_query_report_payload.get("inspection") or {})
    public_query_room_count = public_query_inspection.get("room_count")
    if public_query_room_count is None:
        public_query_room_count = len(public_room_ids)

    room_ids = set(final_rooms_lookup.keys()) | set(public_room_ids) | set(internal_committed_room_ids)
    for refresh in refresh_history:
        for room in list(refresh.get("rooms") or []):
            room_id = _canonical_room_id(room.get("room_id"))
            if room_id is not None:
                room_ids.add(room_id)

    room_summaries: List[Dict[str, Any]] = []
    candidate_block_counter: Counter = Counter()
    commit_block_counter: Counter = Counter()
    commit_block_counter_when_candidate_complete: Counter = Counter()
    finalization_blocker_counter: Counter = Counter()
    publication_blocker_counter: Counter = Counter()
    leave_like_source_counter: Counter = Counter()
    artifact_gaps: List[str] = []

    for key in ("runtime_state_path", "topology_path", "topology_query_report_path", "committed_room_world_model_path"):
        if scene_artifacts.get(key) is None:
            artifact_gaps.append(key)

    for room_id in sorted(room_ids):
        final_room = dict(final_rooms_lookup.get(room_id) or {})
        final_publication = _publication_view(final_room, active_room_id=lifecycle_payload.get("active_room_id"))
        refreshes: List[Dict[str, Any]] = []

        for refresh in refresh_history:
            refresh_room = _find_refresh_room(refresh, room_id)
            if refresh_room is None:
                continue
            publication_view = _publication_view(
                refresh_room,
                active_room_id=_canonical_room_id(refresh.get("active_room_id")),
            )
            merged = dict(refresh_room)
            merged.update(publication_view)
            refreshes.append(
                {
                    "frame_idx": int(refresh.get("frame_idx", -1)),
                    "candidate_room_formed_v1": bool(merged.get("candidate_room_formed_v1")),
                    "candidate_complete": bool(merged.get("candidate_complete")),
                    "commit_ready": bool(merged.get("commit_ready")),
                    "published": bool(merged.get("published")),
                    "leave_like_signal_v1": bool(merged.get("leave_like_signal_v1")),
                    "leave_like_signal_v1_source": _clean_optional_text(merged.get("leave_like_signal_v1_source")),
                    "candidate_block_reasons": list(merged.get("candidate_block_reasons") or []),
                    "commit_block_reasons": list(merged.get("commit_block_reasons") or []),
                    "finalization_blockers": list(merged.get("finalization_blockers") or []),
                    "publication_blockers": list(merged.get("publication_blockers") or []),
                    "publication_state": _clean_optional_text(merged.get("publication_state")) or "unknown",
                    "export_observation_count": int(merged.get("export_observation_count", 0) or 0),
                }
            )

        saw_candidate_room_formed = any(bool(refresh.get("candidate_room_formed_v1")) for refresh in refreshes) or bool(
            final_publication.get("candidate_room_formed_v1")
        )
        saw_candidate_complete = any(bool(refresh.get("candidate_complete")) for refresh in refreshes) or bool(
            final_room.get("candidate_complete")
        )
        saw_commit_ready = any(bool(refresh.get("commit_ready")) for refresh in refreshes) or bool(
            final_publication.get("commit_ready")
        )
        saw_published = room_id in set(internal_committed_room_ids) or any(bool(refresh.get("published")) for refresh in refreshes)
        saw_leave_like_signal = any(bool(refresh.get("leave_like_signal_v1")) for refresh in refreshes) or bool(
            final_publication.get("leave_like_signal_v1")
        )
        max_export_observation_count = max(
            [int(final_room.get("export_observation_count", 0) or 0)] + [int(refresh.get("export_observation_count", 0) or 0) for refresh in refreshes]
        )

        room_candidate_block_counter: Counter = Counter()
        room_commit_block_counter: Counter = Counter()
        room_commit_block_counter_when_candidate_complete: Counter = Counter()
        room_finalization_blocker_counter: Counter = Counter()
        room_publication_blocker_counter: Counter = Counter()
        room_leave_like_sources: Counter = Counter()

        for refresh in refreshes:
            room_candidate_block_counter.update(str(reason) for reason in list(refresh.get("candidate_block_reasons") or []))
            room_commit_block_counter.update(str(reason) for reason in list(refresh.get("commit_block_reasons") or []))
            if bool(refresh.get("candidate_complete")):
                room_commit_block_counter_when_candidate_complete.update(
                    str(reason) for reason in list(refresh.get("commit_block_reasons") or [])
                )
            room_finalization_blocker_counter.update(str(reason) for reason in list(refresh.get("finalization_blockers") or []))
            room_publication_blocker_counter.update(str(reason) for reason in list(refresh.get("publication_blockers") or []))
            source = _clean_optional_text(refresh.get("leave_like_signal_v1_source"))
            if bool(refresh.get("leave_like_signal_v1")) and source is not None:
                room_leave_like_sources[source] += 1

        candidate_block_counter.update(room_candidate_block_counter)
        commit_block_counter.update(room_commit_block_counter)
        commit_block_counter_when_candidate_complete.update(room_commit_block_counter_when_candidate_complete)
        finalization_blocker_counter.update(room_finalization_blocker_counter)
        publication_blocker_counter.update(room_publication_blocker_counter)
        leave_like_source_counter.update(room_leave_like_sources)

        final_commit_blockers = list(final_room.get("commit_block_reasons") or [])
        final_publication_state = _clean_optional_text(final_room.get("publication_state")) or _clean_optional_text(
            final_publication.get("publication_state")
        ) or "unknown"

        room_summaries.append(
            {
                "room_id": room_id,
                "refresh_count": int(len(refreshes)),
                "candidate_room_formed_ever": bool(saw_candidate_room_formed),
                "candidate_complete_ever": bool(saw_candidate_complete),
                "commit_ready_ever": bool(saw_commit_ready),
                "leave_like_signal_ever": bool(saw_leave_like_signal),
                "leave_like_signal_sources": _sorted_counter(room_leave_like_sources),
                "max_export_observation_count": int(max_export_observation_count),
                "internally_committed": room_id in set(internal_committed_room_ids),
                "publicly_visible": room_id in set(public_room_ids),
                "final_lifecycle_state": _clean_optional_text(final_room.get("lifecycle_state")) or "unknown",
                "final_publication_state": final_publication_state,
                "final_commit_block_reasons": list(final_commit_blockers),
                "top_candidate_block_reasons": _sorted_counter(room_candidate_block_counter),
                "top_commit_block_reasons": _sorted_counter(room_commit_block_counter),
                "top_commit_block_reasons_when_candidate_complete": _sorted_counter(
                    room_commit_block_counter_when_candidate_complete
                ),
                "top_finalization_blockers": _sorted_counter(room_finalization_blocker_counter),
                "top_publication_blockers": _sorted_counter(room_publication_blocker_counter),
                "final_room_signature_stability_count": int(final_room.get("room_signature_stability_count", 0) or 0),
                "final_gateway_signature_stability_count": int(final_room.get("gateway_signature_stability_count", 0) or 0),
                "final_containment_stability_count": int(final_room.get("containment_stability_count", 0) or 0),
            }
        )

    transition_summary = _transition_summary(refresh_history)
    candidate_room_formed_count = int(sum(1 for room in room_summaries if room["candidate_room_formed_ever"]))
    candidate_complete_count = int(sum(1 for room in room_summaries if room["candidate_complete_ever"]))
    commit_ready_count = int(sum(1 for room in room_summaries if room["commit_ready_ever"]))
    leave_like_room_count = int(sum(1 for room in room_summaries if room["leave_like_signal_ever"]))
    public_room_count = int(len(public_room_ids))
    internal_committed_room_count = int(len(internal_committed_room_ids))
    committed_rooms_missing_from_public = sorted(set(internal_committed_room_ids) - set(public_room_ids))

    classification, explanation = _classify_empty_public_result(
        public_room_count=public_room_count,
        candidate_room_formed_count=candidate_room_formed_count,
        candidate_complete_count=candidate_complete_count,
        internal_committed_room_count=internal_committed_room_count,
    )

    standard_checks = {
        "multi_refresh_room_observation": bool(any(room["max_export_observation_count"] >= DEFAULT_STABILITY_REFRESH_THRESHOLD for room in room_summaries)),
        "room_transition_or_departure": bool(
            transition_summary.get("room_transition_event_count", 0) > 0 or leave_like_room_count > 0
        ),
        "candidate_complete_room": bool(candidate_complete_count > 0),
        "commit_ready_room": bool(commit_ready_count > 0),
        "public_committed_room": bool(public_room_count > 0),
    }

    sequence_id = (
        _clean_optional_text(summary_payload.get("sequence_id"))
        or _clean_optional_text(lifecycle_payload.get("sequence_id"))
        or Path(scene_artifacts.get("scene_root") or scene_artifacts.get("log_dir") or ".").name
    )

    return {
        "version": "0.1",
        "artifact_kind": "room_commit_diagnosis",
        "artifact_surface": "diagnostic",
        "debug_only": True,
        "public_default": False,
        "non_public": True,
        "sequence_id": sequence_id,
        "input_path": scene_artifacts.get("input_path"),
        "artifacts": {
            "scene_root": scene_artifacts.get("scene_root"),
            "log_dir": scene_artifacts.get("log_dir"),
            "summary_path": scene_artifacts.get("summary_path"),
            "lifecycle_path": scene_artifacts.get("lifecycle_path"),
            "runtime_state_path": scene_artifacts.get("runtime_state_path"),
            "topology_path": scene_artifacts.get("topology_path"),
            "topology_query_report_path": scene_artifacts.get("topology_query_report_path"),
            "committed_room_world_model_path": scene_artifacts.get("committed_room_world_model_path"),
            "missing_artifacts": artifact_gaps,
        },
        "diagnosis": {
            "category": classification,
            "empty_public_topology": bool(public_room_count == 0),
            "explanation": explanation,
            "committed_only_semantics_note": (
                "An empty public topology is valid when no room reaches committed/published state; working or candidate rooms stay non-public."
            ),
        },
        "counts": {
            "observed_room_count": int(len(room_summaries)),
            "candidate_room_formed_count": candidate_room_formed_count,
            "candidate_complete_ever_count": candidate_complete_count,
            "commit_ready_ever_count": commit_ready_count,
            "leave_like_room_count": leave_like_room_count,
            "runtime_internal_committed_room_count": internal_committed_room_count,
            "public_topology_room_count": public_room_count,
            "public_query_room_count": int(public_query_room_count or 0),
            "committed_rooms_surviving_public_filter_count": int(len(set(internal_committed_room_ids) & set(public_room_ids))),
        },
        "room_transition_evidence": transition_summary,
        "room_sets": {
            "internal_committed_room_ids": internal_committed_room_ids,
            "public_room_ids": public_room_ids,
            "committed_rooms_missing_from_public_filter": committed_rooms_missing_from_public,
            "candidate_complete_room_ids": [room["room_id"] for room in room_summaries if room["candidate_complete_ever"]],
            "commit_ready_room_ids": [room["room_id"] for room in room_summaries if room["commit_ready_ever"]],
            "leave_like_room_ids": [room["room_id"] for room in room_summaries if room["leave_like_signal_ever"]],
        },
        "blockers": {
            "candidate_block_reasons": _sorted_counter(candidate_block_counter),
            "commit_block_reasons_any_refresh": _sorted_counter(commit_block_counter),
            "commit_block_reasons_when_candidate_complete": _sorted_counter(commit_block_counter_when_candidate_complete),
            "finalization_blockers": _sorted_counter(finalization_blocker_counter),
            "publication_blockers": _sorted_counter(publication_blocker_counter),
            "leave_like_signal_sources": _sorted_counter(leave_like_source_counter),
        },
        "minimum_replay_sequence_standard": _minimum_replay_standard(standard_checks),
        "room_summaries": room_summaries,
        "quick_readiness_diagnosis": {
            "top_candidate_blockers": _aggregate_top_names(_sorted_counter(candidate_block_counter)),
            "top_commit_blockers_after_candidate_complete": _aggregate_top_names(
                _sorted_counter(commit_block_counter_when_candidate_complete)
            ),
            "top_publication_blockers": _aggregate_top_names(_sorted_counter(publication_blocker_counter)),
        },
    }


def render_room_commit_diagnosis_markdown(report: Dict[str, Any]) -> str:
    diagnosis = dict(report.get("diagnosis") or {})
    counts = dict(report.get("counts") or {})
    transitions = dict(report.get("room_transition_evidence") or {})
    room_sets = dict(report.get("room_sets") or {})
    blockers = dict(report.get("blockers") or {})
    standard = dict(report.get("minimum_replay_sequence_standard") or {})
    checks = dict(standard.get("checks") or {})

    lines: List[str] = []
    lines.append("# Room Commit Diagnosis v0.1")
    lines.append("")
    lines.append(f"- sequence_id: {report.get('sequence_id')}")
    lines.append(f"- diagnosis_category: {diagnosis.get('category')}")
    lines.append(f"- explanation: {diagnosis.get('explanation')}")
    lines.append(f"- observed_room_count: {counts.get('observed_room_count')}")
    lines.append(f"- candidate_room_formed_count: {counts.get('candidate_room_formed_count')}")
    lines.append(f"- candidate_complete_ever_count: {counts.get('candidate_complete_ever_count')}")
    lines.append(f"- commit_ready_ever_count: {counts.get('commit_ready_ever_count')}")
    lines.append(f"- runtime_internal_committed_room_count: {counts.get('runtime_internal_committed_room_count')}")
    lines.append(f"- public_topology_room_count: {counts.get('public_topology_room_count')}")
    lines.append(f"- public_query_room_count: {counts.get('public_query_room_count')}")
    lines.append(f"- room_transition_event_count: {transitions.get('room_transition_event_count')}")
    lines.append(f"- leave_like_room_count: {counts.get('leave_like_room_count')}")
    if room_sets.get("committed_rooms_missing_from_public_filter"):
        lines.append(
            "- committed_rooms_missing_from_public_filter: {rooms}".format(
                rooms=", ".join(str(room_id) for room_id in list(room_sets.get("committed_rooms_missing_from_public_filter") or []))
            )
        )

    lines.append("")
    lines.append("## Blocker Summary")
    for label, key in (
        ("candidate_block_reasons", "candidate_block_reasons"),
        ("commit_block_reasons_when_candidate_complete", "commit_block_reasons_when_candidate_complete"),
        ("publication_blockers", "publication_blockers"),
    ):
        rows = list(blockers.get(key) or [])
        if not rows:
            lines.append(f"- {label}: none")
            continue
        summary = ", ".join(f"{row['name']} x{row['count']}" for row in rows[:6])
        lines.append(f"- {label}: {summary}")

    lines.append("")
    lines.append("## Minimum Replay Standard")
    for requirement in list(standard.get("requirements") or []):
        status = "PASS" if requirement.get("satisfied") else "FAIL"
        lines.append(f"- [{status}] {requirement.get('id')}: {requirement.get('rule')}")

    lines.append("")
    lines.append("## Room Summary")
    lines.append("| room_id | cand_formed | cand_complete | commit_ready | internal_commit | public | leave_like | final_state | final_publication_state |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for room in list(report.get("room_summaries") or []):
        lines.append(
            "| {room_id} | {cand_formed} | {cand_complete} | {commit_ready} | {internal_commit} | {public} | {leave_like} | {final_state} | {publication_state} |".format(
                room_id=room.get("room_id"),
                cand_formed="yes" if room.get("candidate_room_formed_ever") else "no",
                cand_complete="yes" if room.get("candidate_complete_ever") else "no",
                commit_ready="yes" if room.get("commit_ready_ever") else "no",
                internal_commit="yes" if room.get("internally_committed") else "no",
                public="yes" if room.get("publicly_visible") else "no",
                leave_like="yes" if room.get("leave_like_signal_ever") else "no",
                final_state=room.get("final_lifecycle_state"),
                publication_state=room.get("final_publication_state"),
            )
        )

    lines.append("")
    lines.append("## Standard Check Summary")
    for key in (
        "multi_refresh_room_observation",
        "room_transition_or_departure",
        "candidate_complete_room",
        "commit_ready_room",
        "public_committed_room",
    ):
        lines.append(f"- {key}: {'pass' if checks.get(key) else 'fail'}")

    return "\n".join(lines)
