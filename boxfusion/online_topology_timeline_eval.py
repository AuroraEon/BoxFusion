from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple


DEFAULT_JSON_NAME = "online_topology_timeline_eval_v0_1.json"
DEFAULT_MARKDOWN_NAME = "online_topology_timeline_eval_v0_1.md"


def load_json(path: Path) -> Dict[str, Any]:
    return dict(json.loads(path.read_text(encoding="utf-8")))


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_markdown(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text).rstrip() + "\n", encoding="utf-8")


def resolve_lifecycle_artifact(input_path: Path) -> Path:
    path = Path(input_path)
    if path.is_dir():
        direct = path / "online_topology_lifecycle_v0_1.json"
        nested = path / "logs" / "online_topology_lifecycle_v0_1.json"
        for candidate in (direct, nested):
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f"Could not find lifecycle artifact under directory: {path}")

    if path.name == "summary.json":
        summary = load_json(path)
        raw = str(summary.get("online_topology_lifecycle_json") or "").strip()
        candidates: List[Path] = []
        if raw:
            candidates.append(Path(raw))
            candidates.append(path.parent / Path(raw).name)
        candidates.append(path.parent / "online_topology_lifecycle_v0_1.json")
        for candidate in candidates:
            if candidate.exists():
                return candidate
        raise FileNotFoundError(f"Could not resolve lifecycle artifact from summary: {path}")

    if path.exists():
        return path

    raise FileNotFoundError(f"Lifecycle input does not exist: {path}")


def default_output_paths(lifecycle_path: Path) -> Tuple[Path, Path]:
    base_dir = lifecycle_path.parent
    return base_dir / DEFAULT_JSON_NAME, base_dir / DEFAULT_MARKDOWN_NAME


def _first_frame_from_final_room(room: Dict[str, Any]) -> Optional[int]:
    value = room.get("first_seen_frame_idx")
    return None if value is None else int(value)


def _final_committed_frame(payload: Dict[str, Any], room: Dict[str, Any]) -> Optional[int]:
    if str(room.get("lifecycle_state")) != "committed":
        return None
    frame_idx = payload.get("frame_idx")
    return None if frame_idx is None else int(frame_idx)


def _history_room_ids(refresh_history: Sequence[Dict[str, Any]]) -> Set[str]:
    room_ids: Set[str] = set()
    for refresh in refresh_history:
        for room in list(refresh.get("rooms") or []):
            room_id = room.get("room_id")
            if room_id is not None:
                room_ids.add(str(room_id))
    return room_ids


def _room_lookup(rooms: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in rooms:
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _sorted_reason_list(values: Iterable[Any]) -> List[str]:
    return sorted(str(item) for item in values if item not in (None, ""))


def build_timeline_summary(payload: Dict[str, Any], *, source_path: Optional[Path] = None) -> Dict[str, Any]:
    refresh_history = list(payload.get("refresh_history") or [])
    final_rooms = _room_lookup(payload.get("rooms") or [])
    all_room_ids = sorted(set(final_rooms.keys()) | _history_room_ids(refresh_history))
    history_available = bool(refresh_history)
    room_summaries: List[Dict[str, Any]] = []

    for room_id in all_room_ids:
        final_room = final_rooms.get(room_id, {})
        state_path: List[str] = []
        state_transition_events: List[Dict[str, Any]] = []
        blocker_transition_events: List[Dict[str, Any]] = []
        blocker_stats: Dict[str, Dict[str, Any]] = {}
        refresh_steps: List[Dict[str, Any]] = []
        previous_state: Optional[str] = None
        previous_blocks: Set[str] = set()
        first_seen_frame = _first_frame_from_final_room(final_room)
        first_candidate_complete_frame: Optional[int] = None
        first_commit_ready_frame: Optional[int] = None
        first_committed_frame: Optional[int] = None
        candidate_blocked_refresh_count = 0
        merge_pending_refresh_count = 0
        last_blocker_set_before_commit: Optional[List[str]] = None
        latest_nonempty_blocker_set: Optional[List[str]] = None

        for refresh in refresh_history:
            refresh_frame = int(refresh.get("frame_idx"))
            refresh_room = _room_lookup(refresh.get("rooms") or []).get(room_id)
            if refresh_room is None:
                continue

            lifecycle_state = str(refresh_room.get("lifecycle_state"))
            commit_block_reasons = _sorted_reason_list(refresh_room.get("commit_block_reasons") or [])
            commit_block_set = set(commit_block_reasons)
            candidate_complete = bool(refresh_room.get("candidate_complete"))
            commit_ready = not commit_block_reasons

            if first_seen_frame is None:
                seen_frame = refresh_room.get("first_seen_frame_idx")
                first_seen_frame = refresh_frame if seen_frame is None else int(seen_frame)
            if first_candidate_complete_frame is None and candidate_complete:
                first_candidate_complete_frame = refresh_frame
            if first_commit_ready_frame is None and commit_ready:
                first_commit_ready_frame = refresh_frame
                last_blocker_set_before_commit = list(latest_nonempty_blocker_set or [])
            if first_committed_frame is None and lifecycle_state == "committed":
                first_committed_frame = refresh_frame

            if candidate_complete and commit_block_reasons:
                candidate_blocked_refresh_count += 1
            if lifecycle_state == "merge_or_split_pending":
                merge_pending_refresh_count += 1
            if commit_block_reasons:
                latest_nonempty_blocker_set = list(commit_block_reasons)

            if lifecycle_state != previous_state:
                state_path.append(lifecycle_state)
                state_transition_events.append(
                    {
                        "frame_idx": refresh_frame,
                        "timestamp": round(float(refresh.get("timestamp", 0.0)), 3),
                        "lifecycle_state": lifecycle_state,
                    }
                )
                previous_state = lifecycle_state

            added_blocks = sorted(commit_block_set - previous_blocks)
            cleared_blocks = sorted(previous_blocks - commit_block_set)
            if added_blocks or cleared_blocks:
                blocker_transition_events.append(
                    {
                        "frame_idx": refresh_frame,
                        "timestamp": round(float(refresh.get("timestamp", 0.0)), 3),
                        "added": added_blocks,
                        "cleared": cleared_blocks,
                        "remaining": commit_block_reasons,
                    }
                )

            for blocker in commit_block_reasons:
                stats = blocker_stats.setdefault(
                    blocker,
                    {
                        "refresh_count": 0,
                        "first_frame": refresh_frame,
                        "last_frame": refresh_frame,
                        "clear_frames": [],
                    },
                )
                stats["refresh_count"] = int(stats["refresh_count"]) + 1
                stats["last_frame"] = refresh_frame
            for blocker in cleared_blocks:
                stats = blocker_stats.setdefault(
                    blocker,
                    {
                        "refresh_count": 0,
                        "first_frame": refresh_frame,
                        "last_frame": refresh_frame,
                        "clear_frames": [],
                    },
                )
                stats["clear_frames"].append(refresh_frame)

            refresh_steps.append(
                {
                    "frame_idx": refresh_frame,
                    "timestamp": round(float(refresh.get("timestamp", 0.0)), 3),
                    "lifecycle_state": lifecycle_state,
                    "candidate_complete": candidate_complete,
                    "commit_ready": commit_ready,
                    "commit_block_reasons": commit_block_reasons,
                }
            )
            previous_blocks = commit_block_set

        if first_committed_frame is None:
            first_committed_frame = _final_committed_frame(payload, final_room)
        final_lifecycle_state = str(final_room.get("lifecycle_state") or (state_path[-1] if state_path else "unknown"))
        final_blocker_set = _sorted_reason_list(final_room.get("commit_block_reasons") or [])
        if final_lifecycle_state == "committed":
            final_blocker_set = []

        ordered_blockers = sorted(
            blocker_stats.items(),
            key=lambda item: (-int(item[1]["refresh_count"]), int(item[1]["first_frame"]), item[0]),
        )
        room_summaries.append(
            {
                "room_id": room_id,
                "history_available": history_available and bool(refresh_steps),
                "refresh_count": int(len(refresh_steps)),
                "first_seen_frame": first_seen_frame,
                "first_candidate_complete_frame": first_candidate_complete_frame,
                "first_commit_ready_frame": first_commit_ready_frame,
                "first_committed_frame": first_committed_frame,
                "final_lifecycle_state": final_lifecycle_state,
                "lifecycle_path": state_path,
                "state_transition_events": state_transition_events,
                "major_blockers_encountered": [name for name, _ in ordered_blockers],
                "blocker_stats": {name: stats for name, stats in ordered_blockers},
                "blocker_transition_events": blocker_transition_events,
                "candidate_but_blocked_refresh_count": int(candidate_blocked_refresh_count),
                "merge_or_split_pending_refresh_count": int(merge_pending_refresh_count),
                "last_blocker_set_before_commit": last_blocker_set_before_commit if first_commit_ready_frame is not None else None,
                "final_blocker_set": final_blocker_set,
                "refresh_timeline": refresh_steps,
            }
        )

    summary: Dict[str, Any] = {
        "version": "0.1",
        "sequence_id": payload.get("sequence_id"),
        "history_mode": "refresh_history" if history_available else "final_report_only",
        "input_frame_idx": payload.get("frame_idx"),
        "input_timestamp": payload.get("timestamp"),
        "refresh_count": int(len(refresh_history)),
        "room_count": int(len(room_summaries)),
        "committed_room_count": int(sum(1 for room in room_summaries if room["final_lifecycle_state"] == "committed")),
        "rooms_with_candidate_blocked_phase": int(
            sum(1 for room in room_summaries if int(room["candidate_but_blocked_refresh_count"]) > 0)
        ),
        "rooms_with_merge_pending_phase": int(
            sum(1 for room in room_summaries if int(room["merge_or_split_pending_refresh_count"]) > 0)
        ),
        "history_limitations": [],
        "rooms": room_summaries,
    }
    if source_path is not None:
        summary["source_lifecycle_artifact"] = str(source_path)
    if not history_available:
        summary["history_limitations"].append(
            "Per-refresh lifecycle history is unavailable in this artifact, so timing fields beyond final-state evidence may be null."
        )
    return summary


def _fmt_frame(value: Optional[int]) -> str:
    return "n/a" if value is None else str(int(value))


def _fmt_list(values: Sequence[str]) -> str:
    return ", ".join(str(item) for item in values) if values else "-"


def render_timeline_markdown(summary: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append("# Online Topology Timeline Evaluation")
    lines.append("")
    lines.append(f"- sequence_id: {summary.get('sequence_id') or 'n/a'}")
    lines.append(f"- history_mode: {summary.get('history_mode')}")
    lines.append(f"- refresh_count: {summary.get('refresh_count')}")
    lines.append(f"- room_count: {summary.get('room_count')}")
    lines.append(f"- committed_room_count: {summary.get('committed_room_count')}")
    if summary.get("source_lifecycle_artifact"):
        lines.append(f"- source_lifecycle_artifact: {summary.get('source_lifecycle_artifact')}")
    for limitation in list(summary.get("history_limitations") or []):
        lines.append(f"- note: {limitation}")
    lines.append("")
    lines.append("| room_id | first_seen | first_candidate | first_commit_ready | first_committed | final_state | cand_blocked_refreshes | merge_pending_refreshes | final_blockers |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for room in list(summary.get("rooms") or []):
        lines.append(
            "| {room_id} | {first_seen} | {first_candidate} | {first_commit_ready} | {first_committed} | {final_state} | {cand_blocked} | {merge_pending} | {final_blockers} |".format(
                room_id=room.get("room_id"),
                first_seen=_fmt_frame(room.get("first_seen_frame")),
                first_candidate=_fmt_frame(room.get("first_candidate_complete_frame")),
                first_commit_ready=_fmt_frame(room.get("first_commit_ready_frame")),
                first_committed=_fmt_frame(room.get("first_committed_frame")),
                final_state=room.get("final_lifecycle_state"),
                cand_blocked=room.get("candidate_but_blocked_refresh_count"),
                merge_pending=room.get("merge_or_split_pending_refresh_count"),
                final_blockers=_fmt_list(room.get("final_blocker_set") or []),
            )
        )

    detailed_rooms = [
        room
        for room in list(summary.get("rooms") or [])
        if room.get("major_blockers_encountered") or room.get("blocker_transition_events")
    ]
    if detailed_rooms:
        lines.append("")
        lines.append("## Room Notes")
        lines.append("")
        for room in detailed_rooms:
            lines.append(
                "- `{room_id}`: states `{states}`; major blockers `{blockers}`; last blocker set before commit `{before_commit}`.".format(
                    room_id=room.get("room_id"),
                    states=" -> ".join(room.get("lifecycle_path") or ["unknown"]),
                    blockers=_fmt_list(room.get("major_blockers_encountered") or []),
                    before_commit=_fmt_list(room.get("last_blocker_set_before_commit") or []),
                )
            )
            for event in list(room.get("blocker_transition_events") or []):
                if not event.get("added") and not event.get("cleared"):
                    continue
                lines.append(
                    "  frame {frame_idx}: added [{added}] cleared [{cleared}] remaining [{remaining}]".format(
                        frame_idx=event.get("frame_idx"),
                        added=_fmt_list(event.get("added") or []),
                        cleared=_fmt_list(event.get("cleared") or []),
                        remaining=_fmt_list(event.get("remaining") or []),
                    )
                )
    return "\n".join(lines)
