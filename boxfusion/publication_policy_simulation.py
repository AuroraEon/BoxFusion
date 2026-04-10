from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .online_topology_timeline_eval import load_json
from .online_topology_working_snapshot import WORKING_ELIGIBLE_LIFECYCLE_STATES
from .working_vs_committed_topology_timeline import (
    DEFAULT_JSON_NAME as TIMELINE_JSON_NAME,
    resolve_input_artifacts as resolve_timeline_input_artifacts,
)


DEFAULT_JSON_NAME = "publication_policy_simulation_v0_1.json"
ACTUAL_PUBLICATION_SOURCE = "actual_committed_projection"


@dataclass(frozen=True)
class PublicationPolicySpec:
    policy_id: str
    family: str
    description: str
    required_consecutive_refreshes: int
    require_candidate_complete: bool = False
    require_not_dirty: bool = False
    minimum_stable_refresh_opportunities_since_structural_delta: Optional[int] = None

    def streak_ready(self, room: Optional[Dict[str, Any]]) -> bool:
        room = dict(room or {})
        if not _is_working_present(room):
            return False
        if _sorted_str_list(room.get("commit_block_reasons") or []):
            return False
        if self.require_candidate_complete and not bool(room.get("candidate_complete")):
            return False
        if self.require_not_dirty and bool(room.get("dirty")):
            return False
        return True

    def qualifies(self, room: Optional[Dict[str, Any]], streak: int) -> bool:
        room = dict(room or {})
        if not self.streak_ready(room):
            return False
        if int(streak) < int(self.required_consecutive_refreshes):
            return False
        min_stable = self.minimum_stable_refresh_opportunities_since_structural_delta
        if min_stable is not None and int(room.get("stable_refresh_opportunities_since_structural_delta") or 0) < int(min_stable):
            return False
        return True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "policy_id": self.policy_id,
            "family": self.family,
            "description": self.description,
            "required_consecutive_refreshes": int(self.required_consecutive_refreshes),
            "require_candidate_complete": bool(self.require_candidate_complete),
            "require_not_dirty": bool(self.require_not_dirty),
            "minimum_stable_refresh_opportunities_since_structural_delta": self.minimum_stable_refresh_opportunities_since_structural_delta,
        }


def default_policy_candidates() -> List[PublicationPolicySpec]:
    return [
        PublicationPolicySpec(
            policy_id="candidate_complete_blocker_free_n2",
            family="candidate_complete_blocker_free",
            description=(
                "Debug-only conservative simulation: publish when a room is candidate-complete, "
                "working-eligible, present in the latest export, and blocker-free for 2 consecutive refreshes."
            ),
            required_consecutive_refreshes=2,
            require_candidate_complete=True,
        ),
        PublicationPolicySpec(
            policy_id="commit_ready_export_stable2_n2",
            family="commit_ready_plus_structural_stability",
            description=(
                "Debug-only conservative simulation: publish when a room is commit-ready, "
                "working-eligible, present in the latest export for 2 consecutive refreshes, "
                "and stable_refresh_opportunities_since_structural_delta >= 2."
            ),
            required_consecutive_refreshes=2,
            minimum_stable_refresh_opportunities_since_structural_delta=2,
        ),
        PublicationPolicySpec(
            policy_id="commit_ready_not_dirty_n2",
            family="commit_ready_clean_window",
            description=(
                "Debug-only conservative simulation: publish when a room is commit-ready, "
                "working-eligible, present in the latest export, and not dirty for 2 consecutive refreshes."
            ),
            required_consecutive_refreshes=2,
            require_not_dirty=True,
        ),
    ]


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def default_output_path(timeline_path: Path) -> Path:
    return Path(timeline_path).parent / DEFAULT_JSON_NAME


def resolve_input_artifacts(input_path: Path) -> Dict[str, Optional[Path]]:
    path = Path(input_path)
    if path.name == TIMELINE_JSON_NAME:
        lifecycle_path = path.parent / "online_topology_lifecycle_v0_1.json"
        if not lifecycle_path.exists():
            raise FileNotFoundError(f"Missing sibling lifecycle artifact for timeline input: {path}")
        topology_path = path.parent / "topology_v0_1.json"
        report_path = path.parent / "working_vs_committed_topology_report_v0_1.json"
        return {
            "timeline_path": path,
            "lifecycle_path": lifecycle_path,
            "topology_path": topology_path if topology_path.exists() else None,
            "working_vs_committed_report_path": report_path if report_path.exists() else None,
        }

    artifacts = resolve_timeline_input_artifacts(path)
    lifecycle_path = Path(artifacts["lifecycle_path"])
    timeline_path = lifecycle_path.parent / TIMELINE_JSON_NAME
    if not timeline_path.exists():
        raise FileNotFoundError(
            f"Missing sibling working-vs-committed timeline artifact for publication-policy simulation: {timeline_path}"
        )
    return {
        "timeline_path": timeline_path,
        "lifecycle_path": lifecycle_path,
        "topology_path": None if artifacts.get("topology_path") is None else Path(artifacts["topology_path"]),
        "working_vs_committed_report_path": None
        if artifacts.get("working_vs_committed_report_path") is None
        else Path(artifacts["working_vs_committed_report_path"]),
    }


def _round_float(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _sorted_str_list(values: Iterable[Any]) -> List[str]:
    return sorted(str(value) for value in values if value not in (None, ""))


def _room_lookup(rooms: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(rooms or []):
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _history_room_ids(refresh_history: Sequence[Dict[str, Any]]) -> List[str]:
    room_ids = {
        str(room.get("room_id"))
        for refresh in list(refresh_history or [])
        for room in list(refresh.get("rooms") or [])
        if room.get("room_id") is not None
    }
    return sorted(room_ids)


def _is_working_present(room: Optional[Dict[str, Any]]) -> bool:
    room = dict(room or {})
    return bool(room.get("present_in_latest_export")) and str(room.get("lifecycle_state")) in WORKING_ELIGIBLE_LIFECYCLE_STATES


def _is_actual_committed_projection_member(room: Optional[Dict[str, Any]]) -> bool:
    room = dict(room or {})
    return _is_working_present(room) and str(room.get("lifecycle_state")) == "committed"


def _ranked_reason_counts(counter: Counter) -> List[Dict[str, Any]]:
    ranked = sorted(counter.items(), key=lambda item: (-int(item[1]), item[0]))
    return [{"reason": str(reason), "refresh_count": int(count)} for reason, count in ranked]


def _timeline_room_summary_lookup(timeline_payload: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(timeline_payload.get("rooms") or []):
        room_id = room.get("room_id")
        if room_id is None:
            continue
        lookup[str(room_id)] = dict(room)
    return lookup


def _timeline_counts_lookup(timeline_payload: Dict[str, Any]) -> Dict[int, Dict[str, Any]]:
    lookup: Dict[int, Dict[str, Any]] = {}
    scene_summary = dict(timeline_payload.get("scene_summary") or {})
    for row in list(scene_summary.get("counts_over_time") or []):
        frame_idx = row.get("frame_idx")
        if frame_idx is None:
            continue
        lookup[int(frame_idx)] = dict(row)
    return lookup


def _topology_counts_for_room_ids(topology_payload: Optional[Dict[str, Any]], room_ids: Iterable[str]) -> Dict[str, Any]:
    payload = dict(topology_payload or {})
    allowed = set(str(room_id) for room_id in room_ids)
    if not payload:
        return {
            "topology_payload_available": False,
            "room_count": int(len(allowed)),
            "edge_count": None,
            "room_ids": sorted(allowed),
            "relation_type_counts": {},
        }
    rooms = [dict(room) for room in list(payload.get("rooms") or []) if str(room.get("id")) in allowed]
    edges = [
        dict(edge)
        for edge in list(payload.get("edges") or [])
        if str(edge.get("source")) in allowed and str(edge.get("target")) in allowed
    ]
    relation_type_counts: Counter = Counter()
    for edge in edges:
        relation_type = str(edge.get("relation_type") or "unknown")
        relation_type_counts[relation_type] += 1
    return {
        "topology_payload_available": True,
        "room_count": int(len(rooms)),
        "edge_count": int(len(edges)),
        "room_ids": sorted(str(room.get("id")) for room in rooms if room.get("id") is not None),
        "relation_type_counts": dict(sorted((str(key), int(value)) for key, value in relation_type_counts.items())),
    }


def _policy_risk_counter(
    policy: PublicationPolicySpec,
    refresh_history: Sequence[Dict[str, Any]],
    room_id: str,
) -> Counter:
    reasons: Counter = Counter()
    streak = 0
    for refresh in list(refresh_history or []):
        room = _room_lookup(refresh.get("rooms") or []).get(room_id)
        if room is None:
            continue

        streak = int(streak) + 1 if policy.streak_ready(room) else 0
        if policy.qualifies(room, streak):
            continue

        if not bool(room.get("present_in_latest_export")):
            reasons["not_present_in_latest_export"] += 1
            continue

        if str(room.get("lifecycle_state")) not in WORKING_ELIGIBLE_LIFECYCLE_STATES:
            reasons[f"lifecycle_not_working_eligible:{room.get('lifecycle_state')}"] += 1

        if policy.require_candidate_complete and not bool(room.get("candidate_complete")):
            reasons["candidate_complete_required"] += 1

        commit_block_reasons = _sorted_str_list(room.get("commit_block_reasons") or [])
        if commit_block_reasons:
            for blocker in commit_block_reasons:
                reasons[blocker] += 1
            continue

        if policy.require_not_dirty and bool(room.get("dirty")):
            reasons["dirty"] += 1

        min_stable = policy.minimum_stable_refresh_opportunities_since_structural_delta
        if min_stable is not None and int(room.get("stable_refresh_opportunities_since_structural_delta") or 0) < int(min_stable):
            reasons[f"stable_refresh_opportunities_below_{int(min_stable)}"] += 1

        if policy.streak_ready(room) and int(streak) < int(policy.required_consecutive_refreshes):
            reasons["insufficient_consecutive_ready_refreshes"] += 1
    return reasons


def _policy_candidate_frame(
    policy: PublicationPolicySpec,
    refresh_history: Sequence[Dict[str, Any]],
    room_id: str,
) -> Optional[int]:
    streak = 0
    for refresh in list(refresh_history or []):
        room = _room_lookup(refresh.get("rooms") or []).get(room_id)
        streak = int(streak) + 1 if policy.streak_ready(room) else 0
        if policy.qualifies(room, streak):
            return int(refresh.get("frame_idx"))
    return None


def _actual_committed_room_ids_for_refresh(
    refresh: Dict[str, Any],
    *,
    timeline_counts_by_frame: Dict[int, Dict[str, Any]],
) -> List[str]:
    frame_idx = refresh.get("frame_idx")
    if frame_idx is not None:
        row = timeline_counts_by_frame.get(int(frame_idx))
        if row is not None:
            return _sorted_str_list(row.get("committed_room_ids") or [])
    return sorted(
        str(room.get("room_id"))
        for room in list(refresh.get("rooms") or [])
        if room.get("room_id") is not None and _is_actual_committed_projection_member(room)
    )


def _simulated_counts_row(
    *,
    frame_idx: int,
    timestamp: Optional[float],
    rooms_by_id: Dict[str, Dict[str, Any]],
    actual_committed_room_ids: Sequence[str],
    first_policy_frames: Dict[str, Optional[int]],
    row_source: str,
) -> Dict[str, Any]:
    policy_visible_room_ids = [
        room_id
        for room_id, room in rooms_by_id.items()
        if first_policy_frames.get(room_id) is not None
        and int(first_policy_frames[room_id]) <= int(frame_idx)
        and _is_working_present(room)
    ]
    simulated_published_room_ids = sorted(set(actual_committed_room_ids) | set(policy_visible_room_ids))
    additional_room_ids = sorted(set(simulated_published_room_ids) - set(actual_committed_room_ids))
    return {
        "frame_idx": int(frame_idx),
        "timestamp": _round_float(timestamp),
        "row_source": str(row_source),
        "actual_committed_room_count": int(len(actual_committed_room_ids)),
        "simulated_published_room_count": int(len(simulated_published_room_ids)),
        "additional_room_count_vs_actual": int(len(additional_room_ids)),
        "actual_committed_room_ids": list(actual_committed_room_ids),
        "simulated_published_room_ids": simulated_published_room_ids,
        "additional_room_ids_vs_actual": additional_room_ids,
    }


def build_publication_policy_simulation(
    timeline_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
    *,
    topology_payload: Optional[Dict[str, Any]] = None,
    source_artifacts: Optional[Dict[str, Any]] = None,
    policies: Optional[Sequence[PublicationPolicySpec]] = None,
) -> Dict[str, Any]:
    refresh_history = list(lifecycle_payload.get("refresh_history") or [])
    timeline_room_lookup = _timeline_room_summary_lookup(timeline_payload)
    timeline_counts_by_frame = _timeline_counts_lookup(timeline_payload)
    all_room_ids = sorted(set(timeline_room_lookup) | set(_history_room_ids(refresh_history)))
    final_rooms = _room_lookup(lifecycle_payload.get("rooms") or [])
    scene_summary = dict(timeline_payload.get("scene_summary") or {})
    terminal_projection = dict(scene_summary.get("terminal_projection") or {})
    policy_specs = list(policies or default_policy_candidates())

    policy_results: List[Dict[str, Any]] = []
    for policy in policy_specs:
        room_results: List[Dict[str, Any]] = []
        first_policy_frames: Dict[str, Optional[int]] = {}
        still_unpublished_risk_counter: Counter = Counter()

        for room_id in all_room_ids:
            timeline_room = dict(timeline_room_lookup.get(room_id) or {})
            actual_frame = timeline_room.get("first_committed_frame")
            actual_frame = None if actual_frame is None else int(actual_frame)
            policy_frame = _policy_candidate_frame(policy, refresh_history, room_id)

            first_simulated_publication_frame = actual_frame
            simulated_publication_source = ACTUAL_PUBLICATION_SOURCE if actual_frame is not None else None
            if policy_frame is not None and (first_simulated_publication_frame is None or int(policy_frame) < int(first_simulated_publication_frame)):
                first_simulated_publication_frame = int(policy_frame)
                simulated_publication_source = policy.policy_id

            lead_time_frames = None
            if actual_frame is not None and first_simulated_publication_frame is not None:
                lead_time_frames = int(actual_frame) - int(first_simulated_publication_frame)

            if policy_frame is not None and actual_frame is None:
                publication_outcome = "simulated_only_never_actual"
            elif policy_frame is not None and actual_frame is not None and int(policy_frame) < int(actual_frame):
                publication_outcome = "earlier_than_actual"
            elif first_simulated_publication_frame is None:
                publication_outcome = "still_unpublished"
            else:
                publication_outcome = "no_earlier_change"

            risk_counter = Counter()
            if first_simulated_publication_frame is None:
                risk_counter = _policy_risk_counter(policy, refresh_history, room_id)
                still_unpublished_risk_counter.update(risk_counter)

            room_results.append(
                {
                    "room_id": room_id,
                    "first_policy_qualification_frame": policy_frame,
                    "actual_committed_projection_frame": actual_frame,
                    "first_simulated_publication_frame": first_simulated_publication_frame,
                    "simulated_publication_source": simulated_publication_source,
                    "publication_outcome": publication_outcome,
                    "publication_lead_time_frames_vs_actual_committed_projection": lead_time_frames,
                    "dominant_actual_withhold_blockers": list(timeline_room.get("dominant_withhold_blockers") or []),
                    "remaining_unpublished_risk_summary": _ranked_reason_counts(risk_counter),
                }
            )
            first_policy_frames[room_id] = policy_frame

        counts_over_time: List[Dict[str, Any]] = []
        ever_additional_room_ids: set[str] = set()
        for refresh in refresh_history:
            frame_idx = int(refresh.get("frame_idx"))
            refresh_rooms = _room_lookup(refresh.get("rooms") or [])
            actual_committed_room_ids = _actual_committed_room_ids_for_refresh(
                refresh,
                timeline_counts_by_frame=timeline_counts_by_frame,
            )
            row = _simulated_counts_row(
                frame_idx=frame_idx,
                timestamp=refresh.get("timestamp"),
                rooms_by_id=refresh_rooms,
                actual_committed_room_ids=actual_committed_room_ids,
                first_policy_frames=first_policy_frames,
                row_source="refresh_history",
            )
            ever_additional_room_ids.update(row["additional_room_ids_vs_actual"])
            counts_over_time.append(row)

        terminal_frame_idx = terminal_projection.get("frame_idx")
        if terminal_frame_idx is None:
            terminal_frame_idx = lifecycle_payload.get("frame_idx")
        terminal_timestamp = terminal_projection.get("timestamp")
        if terminal_timestamp is None:
            terminal_timestamp = lifecycle_payload.get("timestamp")
        if terminal_frame_idx is not None and final_rooms:
            terminal_actual_committed_room_ids = sorted(
                room_id for room_id, room in final_rooms.items() if _is_actual_committed_projection_member(room)
            )
            terminal_row = _simulated_counts_row(
                frame_idx=int(terminal_frame_idx),
                timestamp=terminal_timestamp,
                rooms_by_id=final_rooms,
                actual_committed_room_ids=terminal_actual_committed_room_ids,
                first_policy_frames=first_policy_frames,
                row_source="terminal_projection",
            )
            if counts_over_time and int(counts_over_time[-1]["frame_idx"]) == int(terminal_frame_idx):
                counts_over_time[-1] = terminal_row
            else:
                counts_over_time.append(terminal_row)
            ever_additional_room_ids.update(terminal_row["additional_room_ids_vs_actual"])

        terminal_row = counts_over_time[-1] if counts_over_time else {
            "actual_committed_room_ids": [],
            "simulated_published_room_ids": [],
            "actual_committed_room_count": 0,
            "simulated_published_room_count": 0,
            "additional_room_count_vs_actual": 0,
            "additional_room_ids_vs_actual": [],
            "row_source": "none",
        }
        simulated_terminal_counts = _topology_counts_for_room_ids(topology_payload, terminal_row["simulated_published_room_ids"])
        actual_terminal_counts = _topology_counts_for_room_ids(topology_payload, terminal_row["actual_committed_room_ids"])

        policy_results.append(
            {
                "policy": policy.to_dict(),
                "summary": {
                    "rooms_published_earlier_than_actual_count": int(
                        sum(1 for room in room_results if room["publication_outcome"] == "earlier_than_actual")
                    ),
                    "rooms_published_under_simulation_but_never_in_actual_count": int(
                        sum(1 for room in room_results if room["publication_outcome"] == "simulated_only_never_actual")
                    ),
                    "rooms_remaining_unpublished_count": int(
                        sum(1 for room in room_results if room["publication_outcome"] == "still_unpublished")
                    ),
                    "earlier_than_actual_room_ids": [
                        room["room_id"] for room in room_results if room["publication_outcome"] == "earlier_than_actual"
                    ],
                    "simulated_only_never_actual_room_ids": [
                        room["room_id"]
                        for room in room_results
                        if room["publication_outcome"] == "simulated_only_never_actual"
                    ],
                    "earlier_or_additional_room_ids": [
                        room["room_id"]
                        for room in room_results
                        if room["publication_outcome"] in {"earlier_than_actual", "simulated_only_never_actual"}
                    ],
                    "remaining_unpublished_room_ids": [
                        room["room_id"] for room in room_results if room["publication_outcome"] == "still_unpublished"
                    ],
                    "maximum_additional_room_count_vs_actual": int(
                        max((row["additional_room_count_vs_actual"] for row in counts_over_time), default=0)
                    ),
                    "ever_additional_room_ids_vs_actual": sorted(ever_additional_room_ids),
                    "first_frame_with_additional_visibility": next(
                        (
                            int(row["frame_idx"])
                            for row in counts_over_time
                            if int(row["additional_room_count_vs_actual"]) > 0
                        ),
                        None,
                    ),
                    "terminal_topology_delta_vs_actual": {
                        "terminal_row_source": terminal_row["row_source"],
                        "simulated_room_count": int(simulated_terminal_counts["room_count"]),
                        "actual_room_count": int(actual_terminal_counts["room_count"]),
                        "additional_room_count_vs_actual": int(
                            simulated_terminal_counts["room_count"] - actual_terminal_counts["room_count"]
                        ),
                        "simulated_room_ids": list(simulated_terminal_counts["room_ids"]),
                        "actual_room_ids": list(actual_terminal_counts["room_ids"]),
                        "simulated_edge_count": simulated_terminal_counts["edge_count"],
                        "actual_edge_count": actual_terminal_counts["edge_count"],
                        "additional_edge_count_vs_actual": None
                        if simulated_terminal_counts["edge_count"] is None or actual_terminal_counts["edge_count"] is None
                        else int(simulated_terminal_counts["edge_count"] - actual_terminal_counts["edge_count"]),
                        "additional_room_ids_vs_actual": list(terminal_row["additional_room_ids_vs_actual"]),
                    },
                    "remaining_unpublished_risk_summary": _ranked_reason_counts(still_unpublished_risk_counter),
                },
                "rooms": room_results,
                "counts_over_time": counts_over_time,
            }
        )

    return {
        "version": "0.1",
        "artifact_kind": "publication_policy_simulation_debug",
        "debug_only": True,
        "public_default": False,
        "non_public": True,
        "non_default_policy_simulation": True,
        "truth_owner": "world_export",
        "sequence_id": timeline_payload.get("sequence_id") or lifecycle_payload.get("sequence_id"),
        "history_mode": timeline_payload.get("history_mode") or ("refresh_history" if refresh_history else "final_report_only"),
        "input_frame_idx": timeline_payload.get("input_frame_idx") or lifecycle_payload.get("frame_idx"),
        "input_timestamp": timeline_payload.get("input_timestamp") or lifecycle_payload.get("timestamp"),
        "refresh_count": int(len(refresh_history)),
        "room_count": int(len(all_room_ids)),
        "simulation_semantics": {
            "simulator_scope": "debug_only_non_default_publication_policy_simulation",
            "actual_truth_owner": "world_export",
            "actual_public_semantics_changed": False,
            "working_topology_exposed_publicly": False,
            "simulated_published_room_definition": (
                "actual committed projection membership union policy-qualified rooms that remain working-eligible and present in the latest export"
            ),
            "actual_committed_projection_definition": "working_present && lifecycle_state == committed",
            "working_present_definition": "present_in_latest_export && lifecycle_state in eligible working states",
            "eligible_working_lifecycle_states": sorted(WORKING_ELIGIBLE_LIFECYCLE_STATES),
            "note": (
                "Policy results are additive debug simulations over the existing timeline; they do not alter the adopted committed/public path."
            ),
        },
        "scene_summary": {
            "actual_terminal_projection": dict(scene_summary.get("terminal_projection") or {}),
            "actual_dominant_withhold_blockers": list(scene_summary.get("dominant_withhold_blockers") or []),
            "actual_final_static_comparison": dict(scene_summary.get("final_static_comparison") or {}),
        },
        "policy_results": policy_results,
        "source_artifacts": dict(source_artifacts or {}),
    }
