from __future__ import annotations

import json
import statistics
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .artifact_contract import ARTIFACT_SURFACE_WORKING
from .online_topology_timeline_eval import load_json, resolve_lifecycle_artifact
from .online_topology_working_snapshot import WORKING_ELIGIBLE_LIFECYCLE_STATES


DEFAULT_JSON_NAME = "working_vs_committed_topology_timeline_v0_1.json"
DEFAULT_MARKDOWN_NAME = "working_vs_committed_topology_timeline_v0_1.md"
COMMIT_READY_PENDING_PUBLICATION = "commit_ready_pending_publication"


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_markdown(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text).rstrip() + "\n", encoding="utf-8")


def default_output_paths(lifecycle_path: Path) -> Tuple[Path, Path]:
    base_dir = Path(lifecycle_path).parent
    return base_dir / DEFAULT_JSON_NAME, base_dir / DEFAULT_MARKDOWN_NAME


def resolve_input_artifacts(input_path: Path) -> Dict[str, Optional[Path]]:
    lifecycle_path = resolve_lifecycle_artifact(Path(input_path))
    base_dir = lifecycle_path.parent

    report_path = base_dir / "working_vs_committed_topology_report_v0_1.json"
    working_path = base_dir / "working_topology_v0_1.json"
    topology_path = base_dir / "topology_v0_1.json"

    return {
        "lifecycle_path": lifecycle_path,
        "working_vs_committed_report_path": report_path if report_path.exists() else None,
        "working_topology_path": working_path if working_path.exists() else None,
        "topology_path": topology_path if topology_path.exists() else None,
    }


def _room_lookup(rooms: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in rooms:
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _history_room_ids(refresh_history: Sequence[Dict[str, Any]]) -> List[str]:
    room_ids = {
        str(room.get("room_id"))
        for refresh in refresh_history
        for room in list(refresh.get("rooms") or [])
        if room.get("room_id") is not None
    }
    return sorted(room_ids)


def _sorted_str_list(values: Iterable[Any]) -> List[str]:
    return sorted(str(item) for item in values if item not in (None, ""))


def _is_working_present(room: Optional[Dict[str, Any]]) -> bool:
    room = dict(room or {})
    return bool(room.get("present_in_latest_export")) and str(room.get("lifecycle_state")) in WORKING_ELIGIBLE_LIFECYCLE_STATES


def _is_committed_projection_member(room: Optional[Dict[str, Any]]) -> bool:
    room = dict(room or {})
    return _is_working_present(room) and str(room.get("lifecycle_state")) == "committed"


def _withhold_reason_tokens(room: Optional[Dict[str, Any]]) -> List[str]:
    room = dict(room or {})
    if not _is_working_present(room) or _is_committed_projection_member(room):
        return []
    commit_block_reasons = _sorted_str_list(room.get("commit_block_reasons") or [])
    if commit_block_reasons:
        return commit_block_reasons
    return [COMMIT_READY_PENDING_PUBLICATION]


def _ranked_reason_counts(counter: Counter) -> List[Dict[str, Any]]:
    ranked = sorted(counter.items(), key=lambda item: (-int(item[1]), item[0]))
    return [{"blocker": name, "refresh_count": int(count)} for name, count in ranked]


def _ranked_blocker_sets(counter: Counter) -> List[Dict[str, Any]]:
    ranked = sorted(counter.items(), key=lambda item: (-int(item[1]), list(item[0])))
    return [{"blockers": list(blockers), "refresh_count": int(count)} for blockers, count in ranked]


def _transition_summary(transitions: Dict[Tuple[Tuple[str, ...], Tuple[str, ...]], Dict[str, Any]]) -> Dict[str, Any]:
    ranked = sorted(
        transitions.items(),
        key=lambda item: (-int(item[1]["count"]), int(item[1]["first_frame"]), list(item[0][0]), list(item[0][1])),
    )
    return {
        "transition_count": int(len(ranked)),
        "transitions": [
            {
                "from_blockers": list(source),
                "to_blockers": list(target),
                "count": int(stats["count"]),
                "first_frame": int(stats["first_frame"]),
                "last_frame": int(stats["last_frame"]),
            }
            for (source, target), stats in ranked
        ],
    }


def _compressed_state_path(states: Sequence[str], final_state: Optional[str]) -> List[str]:
    compressed: List[str] = []
    for state in states:
        if not compressed or compressed[-1] != state:
            compressed.append(state)
    if final_state and (not compressed or compressed[-1] != final_state):
        compressed.append(final_state)
    return compressed


def _oscillated(path: Sequence[str]) -> bool:
    seen: set[str] = set()
    for state in path:
        if state in seen:
            return True
        seen.add(state)
    return False


def _round_float(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def build_working_vs_committed_timeline(
    payload: Dict[str, Any],
    *,
    source_artifacts: Optional[Dict[str, Any]] = None,
    source_path: Optional[Path] = None,
    final_comparison_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    refresh_history = list(payload.get("refresh_history") or [])
    history_available = bool(refresh_history)
    final_rooms = _room_lookup(payload.get("rooms") or [])
    all_room_ids = sorted(set(final_rooms) | set(_history_room_ids(refresh_history)))

    room_summaries: List[Dict[str, Any]] = []
    global_withhold_reason_counter: Counter = Counter()
    global_withhold_blocker_set_counter: Counter = Counter()

    for room_id in all_room_ids:
        final_room = dict(final_rooms.get(room_id) or {})
        room_refresh_count = 0
        total_working_refreshes = 0
        total_withheld_refreshes = 0
        total_committed_refreshes = 0
        first_present_in_working_frame: Optional[int] = None
        first_candidate_complete_frame: Optional[int] = None
        first_commit_ready_frame: Optional[int] = None
        first_committed_frame: Optional[int] = None
        prior_withhold_blockers: Optional[Tuple[str, ...]] = None
        withhold_reason_counter: Counter = Counter()
        withhold_blocker_set_counter: Counter = Counter()
        blocker_transition_stats: Dict[Tuple[Tuple[str, ...], Tuple[str, ...]], Dict[str, Any]] = {}
        lifecycle_states_in_history: List[str] = []

        for refresh in refresh_history:
            refresh_frame = int(refresh.get("frame_idx"))
            refresh_room = _room_lookup(refresh.get("rooms") or []).get(room_id)
            if refresh_room is None:
                prior_withhold_blockers = None
                continue

            room_refresh_count += 1
            lifecycle_state = str(refresh_room.get("lifecycle_state") or "unknown")
            lifecycle_states_in_history.append(lifecycle_state)
            commit_block_reasons = _sorted_str_list(refresh_room.get("commit_block_reasons") or [])

            if first_candidate_complete_frame is None and bool(refresh_room.get("candidate_complete")):
                first_candidate_complete_frame = refresh_frame

            working_present = _is_working_present(refresh_room)
            committed_projection_member = _is_committed_projection_member(refresh_room)
            withhold_blockers = tuple(_withhold_reason_tokens(refresh_room))

            if working_present:
                total_working_refreshes += 1
                if first_present_in_working_frame is None:
                    first_present_in_working_frame = refresh_frame
                if first_commit_ready_frame is None and not commit_block_reasons:
                    first_commit_ready_frame = refresh_frame

            if committed_projection_member:
                total_committed_refreshes += 1
                if first_committed_frame is None:
                    first_committed_frame = refresh_frame

            if working_present and not committed_projection_member:
                total_withheld_refreshes += 1
                for blocker in withhold_blockers:
                    withhold_reason_counter[blocker] += 1
                    global_withhold_reason_counter[blocker] += 1
                withhold_blocker_set_counter[withhold_blockers] += 1
                global_withhold_blocker_set_counter[withhold_blockers] += 1

                if prior_withhold_blockers is not None and prior_withhold_blockers != withhold_blockers:
                    key = (prior_withhold_blockers, withhold_blockers)
                    entry = blocker_transition_stats.setdefault(
                        key,
                        {
                            "count": 0,
                            "first_frame": refresh_frame,
                            "last_frame": refresh_frame,
                        },
                    )
                    entry["count"] = int(entry["count"]) + 1
                    entry["last_frame"] = refresh_frame
                prior_withhold_blockers = withhold_blockers
            else:
                prior_withhold_blockers = None

        final_lifecycle_state = str(final_room.get("lifecycle_state") or (lifecycle_states_in_history[-1] if lifecycle_states_in_history else "unknown"))
        final_working_present = _is_working_present(final_room)
        final_committed_projection_member = _is_committed_projection_member(final_room)
        final_withhold_blockers = _withhold_reason_tokens(final_room)
        final_commit_block_reasons = _sorted_str_list(final_room.get("commit_block_reasons") or [])

        if first_present_in_working_frame is None and final_working_present:
            final_frame = payload.get("frame_idx")
            first_present_in_working_frame = None if final_frame is None else int(final_frame)
        if first_candidate_complete_frame is None and bool(final_room.get("candidate_complete")):
            final_frame = payload.get("frame_idx")
            first_candidate_complete_frame = None if final_frame is None else int(final_frame)
        if first_commit_ready_frame is None and final_working_present and not final_commit_block_reasons:
            final_frame = payload.get("frame_idx")
            first_commit_ready_frame = None if final_frame is None else int(final_frame)
        if first_committed_frame is None and final_committed_projection_member:
            final_frame = payload.get("frame_idx")
            first_committed_frame = None if final_frame is None else int(final_frame)

        lifecycle_state_path = _compressed_state_path(lifecycle_states_in_history, final_lifecycle_state)
        committed_only_at_terminal_projection = bool(final_committed_projection_member and total_committed_refreshes == 0)
        ever_working_only_then_later_committed = bool(total_withheld_refreshes > 0 and final_committed_projection_member)
        remained_working_only_to_end = bool(total_withheld_refreshes > 0 and final_working_present and not final_committed_projection_member)

        room_summaries.append(
            {
                "room_id": room_id,
                "history_available": history_available and room_refresh_count > 0,
                "room_refresh_count": int(room_refresh_count),
                "first_present_in_working_frame": first_present_in_working_frame,
                "first_candidate_complete_frame": first_candidate_complete_frame,
                "first_commit_ready_frame": first_commit_ready_frame,
                "first_committed_frame": first_committed_frame,
                "final_lifecycle_state": final_lifecycle_state,
                "final_working_present": bool(final_working_present),
                "final_committed_projection_member": bool(final_committed_projection_member),
                "final_withhold_blockers": list(final_withhold_blockers),
                "total_working_refreshes": int(total_working_refreshes),
                "total_withheld_refreshes": int(total_withheld_refreshes),
                "total_committed_refreshes": int(total_committed_refreshes),
                "dominant_withhold_blockers": _ranked_reason_counts(withhold_reason_counter),
                "dominant_withhold_blocker_sets": _ranked_blocker_sets(withhold_blocker_set_counter),
                "blocker_transition_summary": {
                    "observed_sets": _ranked_blocker_sets(withhold_blocker_set_counter),
                    **_transition_summary(blocker_transition_stats),
                },
                "lifecycle_state_path": list(lifecycle_state_path),
                "oscillated_lifecycle_state": bool(_oscillated(lifecycle_state_path)),
                "was_working_only_then_later_committed": bool(ever_working_only_then_later_committed),
                "remained_working_only_to_end": bool(remained_working_only_to_end),
                "committed_projection_emerges_only_at_terminal_projection": bool(committed_only_at_terminal_projection),
            }
        )

    counts_over_time: List[Dict[str, Any]] = []
    for refresh in refresh_history:
        working_room_ids: List[str] = []
        committed_room_ids: List[str] = []
        withheld_room_ids: List[str] = []
        for room in list(refresh.get("rooms") or []):
            room_id = room.get("room_id")
            if room_id is None:
                continue
            canonical_room_id = str(room_id)
            if _is_committed_projection_member(room):
                working_room_ids.append(canonical_room_id)
                committed_room_ids.append(canonical_room_id)
            elif _is_working_present(room):
                working_room_ids.append(canonical_room_id)
                withheld_room_ids.append(canonical_room_id)
        counts_over_time.append(
            {
                "frame_idx": int(refresh.get("frame_idx")),
                "timestamp": _round_float(refresh.get("timestamp")),
                "working_room_count": int(len(working_room_ids)),
                "committed_room_count": int(len(committed_room_ids)),
                "withheld_room_count": int(len(withheld_room_ids)),
                "working_room_ids": sorted(working_room_ids),
                "committed_room_ids": sorted(committed_room_ids),
                "withheld_room_ids": sorted(withheld_room_ids),
            }
        )

    final_working_room_count = int(sum(1 for room in final_rooms.values() if _is_working_present(room)))
    final_committed_room_count = int(sum(1 for room in final_rooms.values() if _is_committed_projection_member(room)))
    final_withheld_room_count = int(final_working_room_count - final_committed_room_count)

    withheld_durations = [int(room["total_withheld_refreshes"]) for room in room_summaries if int(room["total_withheld_refreshes"]) > 0]
    scene_summary: Dict[str, Any] = {
        "counts_over_time": counts_over_time,
        "terminal_projection": {
            "frame_idx": payload.get("frame_idx"),
            "timestamp": payload.get("timestamp"),
            "working_room_count": int(final_working_room_count),
            "committed_room_count": int(final_committed_room_count),
            "withheld_room_count": int(final_withheld_room_count),
        },
        "rooms_ever_working_only_count": int(sum(1 for room in room_summaries if int(room["total_withheld_refreshes"]) > 0)),
        "rooms_working_only_then_committed_count": int(
            sum(1 for room in room_summaries if bool(room["was_working_only_then_later_committed"]))
        ),
        "rooms_remaining_working_only_to_end_count": int(
            sum(1 for room in room_summaries if bool(room["remained_working_only_to_end"]))
        ),
        "dominant_withhold_blockers": _ranked_reason_counts(global_withhold_reason_counter),
        "dominant_blocker_sets_across_withheld_periods": _ranked_blocker_sets(global_withhold_blocker_set_counter),
        "maximum_withheld_duration_refreshes": int(max(withheld_durations) if withheld_durations else 0),
        "mean_withheld_duration_refreshes": None if not withheld_durations else round(float(statistics.mean(withheld_durations)), 3),
        "median_withheld_duration_refreshes": None if not withheld_durations else round(float(statistics.median(withheld_durations)), 3),
    }

    if final_comparison_payload is not None:
        difference_summary = dict(final_comparison_payload.get("difference_summary") or {})
        withheld_summary = dict(final_comparison_payload.get("withheld_topology_summary") or {})
        scene_summary["final_static_comparison"] = {
            "working_room_count": int(dict(final_comparison_payload.get("working_topology") or {}).get("room_count", 0)),
            "committed_room_count": int(dict(final_comparison_payload.get("committed_topology_projection") or {}).get("room_count", 0)),
            "withheld_room_count": int(withheld_summary.get("withheld_room_count", 0)),
            "working_only_room_ids": list(difference_summary.get("working_only_rooms") or []),
        }

    summary: Dict[str, Any] = {
        "version": "0.1",
        "artifact_kind": "working_vs_committed_topology_timeline_debug",
        "artifact_surface": ARTIFACT_SURFACE_WORKING,
        "debug_only": True,
        "public_default": False,
        "non_public": True,
        "truth_owner": "world_export",
        "sequence_id": payload.get("sequence_id"),
        "history_mode": "refresh_history" if history_available else "final_report_only",
        "input_frame_idx": payload.get("frame_idx"),
        "input_timestamp": payload.get("timestamp"),
        "refresh_count": int(len(refresh_history)),
        "room_count": int(len(room_summaries)),
        "timeline_semantics": {
            "working_present_definition": "present_in_latest_export && lifecycle_state in eligible working states",
            "eligible_working_lifecycle_states": sorted(WORKING_ELIGIBLE_LIFECYCLE_STATES),
            "committed_projection_member_definition": "working_present && lifecycle_state == committed",
            "withheld_definition": "working_present && !committed_projection_member",
            "commit_ready_pending_publication_marker": COMMIT_READY_PENDING_PUBLICATION,
            "final_commit_interpretation": "final committed membership is aligned to committed projection semantics, not lifecycle committed state alone",
        },
        "history_limitations": [],
        "scene_summary": scene_summary,
        "rooms": room_summaries,
        "source_artifacts": dict(source_artifacts or {}),
    }
    if source_path is not None:
        summary["source_artifacts"]["lifecycle_json"] = str(source_path)
    if not history_available:
        summary["history_limitations"].append(
            "Per-refresh lifecycle history is unavailable in this artifact, so withheld-duration and blocker-transition fields are conservative."
        )
    return summary


def _fmt_frame(value: Optional[int]) -> str:
    return "n/a" if value is None else str(int(value))


def _fmt_blockers(values: Sequence[Dict[str, Any]]) -> str:
    if not values:
        return "-"
    return ", ".join(f"{item['blocker']} x{item['refresh_count']}" for item in list(values)[:3])


def render_working_vs_committed_timeline_markdown(summary: Dict[str, Any]) -> str:
    scene_summary = dict(summary.get("scene_summary") or {})
    terminal_projection = dict(scene_summary.get("terminal_projection") or {})
    lines: List[str] = []
    lines.append("# Working vs Committed Topology Timeline")
    lines.append("")
    lines.append(f"- sequence_id: {summary.get('sequence_id') or 'n/a'}")
    lines.append(f"- history_mode: {summary.get('history_mode')}")
    lines.append(f"- refresh_count: {summary.get('refresh_count')}")
    lines.append(f"- terminal working_room_count: {terminal_projection.get('working_room_count')}")
    lines.append(f"- terminal committed_room_count: {terminal_projection.get('committed_room_count')}")
    lines.append(f"- terminal withheld_room_count: {terminal_projection.get('withheld_room_count')}")
    lines.append(f"- rooms_ever_working_only_count: {scene_summary.get('rooms_ever_working_only_count')}")
    lines.append(f"- rooms_working_only_then_committed_count: {scene_summary.get('rooms_working_only_then_committed_count')}")
    lines.append(f"- rooms_remaining_working_only_to_end_count: {scene_summary.get('rooms_remaining_working_only_to_end_count')}")
    lines.append(f"- maximum_withheld_duration_refreshes: {scene_summary.get('maximum_withheld_duration_refreshes')}")
    lines.append(f"- mean_withheld_duration_refreshes: {scene_summary.get('mean_withheld_duration_refreshes')}")
    lines.append(f"- median_withheld_duration_refreshes: {scene_summary.get('median_withheld_duration_refreshes')}")
    for limitation in list(summary.get("history_limitations") or []):
        lines.append(f"- note: {limitation}")
    lines.append("")
    lines.append("| room_id | first_working | first_candidate | first_commit_ready | first_committed | final_state | working_refreshes | withheld_refreshes | committed_refreshes | end_status | dominant_withhold_blockers |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for room in list(summary.get("rooms") or []):
        if room.get("final_committed_projection_member"):
            end_status = "committed_projection"
        elif room.get("final_working_present"):
            end_status = "working_only"
        else:
            end_status = "not_in_terminal_working"
        lines.append(
            "| {room_id} | {first_working} | {first_candidate} | {first_commit_ready} | {first_committed} | {final_state} | {working_refreshes} | {withheld_refreshes} | {committed_refreshes} | {end_status} | {dominant_blockers} |".format(
                room_id=room.get("room_id"),
                first_working=_fmt_frame(room.get("first_present_in_working_frame")),
                first_candidate=_fmt_frame(room.get("first_candidate_complete_frame")),
                first_commit_ready=_fmt_frame(room.get("first_commit_ready_frame")),
                first_committed=_fmt_frame(room.get("first_committed_frame")),
                final_state=room.get("final_lifecycle_state"),
                working_refreshes=room.get("total_working_refreshes"),
                withheld_refreshes=room.get("total_withheld_refreshes"),
                committed_refreshes=room.get("total_committed_refreshes"),
                end_status=end_status,
                dominant_blockers=_fmt_blockers(room.get("dominant_withhold_blockers") or []),
            )
        )
    return "\n".join(lines)


def load_optional_json(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None or not Path(path).exists():
        return None
    return load_json(Path(path))
