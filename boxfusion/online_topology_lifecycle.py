from __future__ import annotations

import hashlib
import json
import math
from collections import Counter, deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Set, Tuple


_ROOM_SIGNATURE_GEOMETRY_QUANTIZATION_M = 0.10
_ROOM_SIGNATURE_AREA_QUANTIZATION_M2 = 0.25
_GATEWAY_SIGNATURE_POSITION_QUANTIZATION_M = 0.10
_SIGNATURE_RELATCH_GRACE_EXTRA_REFRESHES = 1
_ROOM_SIGNATURE_POLYGON_JAGGEDNESS_EPSILON_BUCKETS = 1.0
_ROOM_SIGNATURE_POLYGON_SHORT_EDGE_BUCKETS = 2.0
_ROOM_SIGNATURE_COMPONENT_ORDER = (
    "room_id",
    "floor_id",
    "room_type",
    "status",
    "polygon",
    "center",
    "area_m2",
)
_GATEWAY_SIGNATURE_COMPONENT_ORDER = (
    "gateway_count",
    "other_room_id",
    "type",
    "relation_scope",
    "connects",
    "floor_id",
    "pos_world",
)


class _StrEnum(str, Enum):
    def __str__(self) -> str:
        return str(self.value)


class RoomLifecycleState(_StrEnum):
    DISCOVERING = "discovering"
    ACTIVE = "active"
    CANDIDATE_COMPLETE = "candidate_complete"
    COMMITTED = "committed"
    REVISITABLE = "revisitable"
    MERGE_OR_SPLIT_PENDING = "merge_or_split_pending"


class DirtyRoomReason(_StrEnum):
    DISCOVERED = "discovered"
    SEGMENTATION_REFRESH = "segmentation_refresh"
    CURRENT_ROOM_CHANGE = "current_room_change"
    FLOOR_TRANSITION = "floor_transition"
    FLOOR_STATUS_CHANGE = "floor_status_change"
    VERTICAL_TRANSITION_UPDATE = "vertical_transition_update"
    OBJECT_OR_ANCHOR_ASSIGNMENT_DELTA = "object_or_anchor_assignment_delta"
    ROOM_SIGNATURE_CHANGED = "room_signature_changed"
    GATEWAY_STRUCTURE_CHANGED = "gateway_structure_changed"
    TRACKING_STRUCTURE_DELTA = "tracking_structure_delta"
    ROOM_REMOVED = "room_removed"
    REVISIT = "revisit"


class CandidateCompleteReason(_StrEnum):
    LEFT_ACTIVE_ROOM = "left_active_room"
    ROOM_SIGNATURE_STABLE = "room_signature_stable"
    GATEWAY_STRUCTURE_STABLE = "gateway_structure_stable"
    CONTAINMENT_STABLE = "containment_stable"
    FLOOR_ASSIGNMENT_STABLE = "floor_assignment_stable"


class CandidateBlockReason(_StrEnum):
    ROOM_CURRENTLY_ACTIVE = "room_currently_active"
    NO_LEAVE_LIKE_SIGNAL = "no_leave_like_signal"
    ROOM_SIGNATURE_NOT_STABLE = "room_signature_not_stable"
    FLOOR_STATUS_NOT_STABLE = "floor_status_not_stable"
    ROOM_MISSING_FROM_LATEST_EXPORT = "room_missing_from_latest_export"
    INSUFFICIENT_READINESS_SCORE = "insufficient_readiness_score"


class CommitBlockReason(_StrEnum):
    ROOM_CURRENTLY_ACTIVE = "room_currently_active"
    NO_LEAVE_LIKE_SIGNAL = "no_leave_like_signal"
    ROOM_SIGNATURE_NOT_STABLE = "room_signature_not_stable"
    GATEWAY_STRUCTURE_NOT_STABLE = "gateway_structure_not_stable"
    CONTAINMENT_NOT_STABLE = "containment_not_stable"
    FLOOR_STATUS_NOT_STABLE = "floor_status_not_stable"
    MERGE_OR_SPLIT_PENDING = "merge_or_split_pending"
    VERTICAL_TRANSITION_PARTIAL = "vertical_transition_partial"
    ROOM_FLOOR_VALIDATION_FAILED = "room_floor_validation_failed"
    ROOM_MISSING_FROM_LATEST_EXPORT = "room_missing_from_latest_export"


@dataclass
class RoomWorkingTopologyStatus:
    room_id: str
    lifecycle_state: RoomLifecycleState = RoomLifecycleState.DISCOVERING
    dirty: bool = True
    dirty_reasons: Set[DirtyRoomReason] = field(default_factory=set)
    last_updated_frame_idx: Optional[int] = None
    last_updated_timestamp: Optional[float] = None
    first_seen_frame_idx: Optional[int] = None
    last_observed_frame_idx: Optional[int] = None
    last_departed_frame_idx: Optional[int] = None
    export_observation_count: int = 0
    floor_id: Optional[str] = None
    floor_status: Optional[str] = None
    present_in_latest_export: bool = False
    current_room_signature: Optional[str] = None
    room_signature_stability_count: int = 0
    current_gateway_signature: Optional[str] = None
    gateway_signature_stability_count: int = 0
    current_containment_signature: Optional[str] = None
    containment_stability_count: int = 0
    merge_pending: bool = False
    candidate_complete: bool = False
    candidate_readiness_score: int = 0
    candidate_complete_reasons: Set[CandidateCompleteReason] = field(default_factory=set)
    candidate_block_reasons: Set[CandidateBlockReason] = field(default_factory=set)
    commit_block_reasons: Set[CommitBlockReason] = field(default_factory=set)
    stable_refresh_opportunities_since_structural_delta: int = 0
    last_structural_delta_frame_idx: Optional[int] = None
    vertical_transition_statuses: Set[str] = field(default_factory=set)
    vertical_transition_ids: List[str] = field(default_factory=list)
    tracking_summary: Dict[str, int] = field(default_factory=dict)
    trigger_counts: Counter = field(default_factory=Counter)
    recent_triggers: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=16))
    last_room_signature_change_components: List[str] = field(default_factory=list)
    room_signature_change_component_counts: Counter = field(default_factory=Counter)
    current_room_signature_components: Optional[Dict[str, Any]] = field(default=None, repr=False)
    last_gateway_signature_change_components: List[str] = field(default_factory=list)
    gateway_signature_change_component_counts: Counter = field(default_factory=Counter)
    current_gateway_signature_components: Optional[Dict[str, Any]] = field(default=None, repr=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "room_id": self.room_id,
            "lifecycle_state": self.lifecycle_state.value,
            "dirty": bool(self.dirty),
            "dirty_reasons": sorted(reason.value for reason in self.dirty_reasons),
            "last_updated_frame_idx": self.last_updated_frame_idx,
            "last_updated_timestamp": self.last_updated_timestamp,
            "first_seen_frame_idx": self.first_seen_frame_idx,
            "last_observed_frame_idx": self.last_observed_frame_idx,
            "last_departed_frame_idx": self.last_departed_frame_idx,
            "export_observation_count": int(self.export_observation_count),
            "floor_id": self.floor_id,
            "floor_status": self.floor_status,
            "present_in_latest_export": bool(self.present_in_latest_export),
            "room_signature_stability_count": int(self.room_signature_stability_count),
            "gateway_signature_stability_count": int(self.gateway_signature_stability_count),
            "containment_stability_count": int(self.containment_stability_count),
            "candidate_complete": bool(self.candidate_complete),
            "candidate_readiness_score": int(self.candidate_readiness_score),
            "candidate_complete_reasons": sorted(reason.value for reason in self.candidate_complete_reasons),
            "candidate_block_reasons": sorted(reason.value for reason in self.candidate_block_reasons),
            "commit_block_reasons": sorted(reason.value for reason in self.commit_block_reasons),
            "stable_refresh_opportunities_since_structural_delta": int(self.stable_refresh_opportunities_since_structural_delta),
            "last_structural_delta_frame_idx": self.last_structural_delta_frame_idx,
            "vertical_transition_statuses": sorted(str(item) for item in self.vertical_transition_statuses),
            "vertical_transition_ids": list(self.vertical_transition_ids),
            "tracking_summary": dict(self.tracking_summary),
            "trigger_counts": dict(sorted((str(key), int(value)) for key, value in self.trigger_counts.items())),
            "last_room_signature_change_components": list(self.last_room_signature_change_components),
            "room_signature_change_component_counts": dict(
                sorted((str(key), int(value)) for key, value in self.room_signature_change_component_counts.items())
            ),
            "last_gateway_signature_change_components": list(self.last_gateway_signature_change_components),
            "gateway_signature_change_component_counts": dict(
                sorted((str(key), int(value)) for key, value in self.gateway_signature_change_component_counts.items())
            ),
            "recent_triggers": list(self.recent_triggers),
        }


class OnlineTopologyLifecycleManager:
    candidate_readiness_threshold = 4

    def __init__(
        self,
        *,
        sequence_id: Optional[str] = None,
        trigger_history_limit: int = 128,
        stability_refresh_threshold: int = 2,
    ) -> None:
        self.sequence_id = None if sequence_id is None else str(sequence_id)
        self.trigger_history_limit = max(8, int(trigger_history_limit))
        self.stability_refresh_threshold = max(1, int(stability_refresh_threshold))
        self.room_statuses: Dict[str, RoomWorkingTopologyStatus] = {}
        self.trigger_history: Deque[Dict[str, Any]] = deque(maxlen=self.trigger_history_limit)
        self.trigger_counts: Counter = Counter()
        self.refresh_history: List[Dict[str, Any]] = []
        self.active_room_id: Optional[str] = None
        self.active_floor_id: Optional[str] = None
        self.active_floor_status: Optional[str] = None
        self._last_tracking_key: Optional[Tuple[Any, ...]] = None
        self._last_export_key: Optional[Tuple[Any, ...]] = None
        self.signature_relatch_grace_refresh_threshold = (
            self.stability_refresh_threshold + _SIGNATURE_RELATCH_GRACE_EXTRA_REFRESHES
        )

    def observe_room_tracking(
        self,
        *,
        frame_idx: int,
        timestamp: float,
        current_room_id: Optional[Any],
        source: str,
    ) -> None:
        canonical_room_id = _canonical_room_id(current_room_id)
        tracking_key = (str(source), int(frame_idx), canonical_room_id)
        if tracking_key == self._last_tracking_key:
            return
        self._last_tracking_key = tracking_key

        if canonical_room_id is None:
            self.active_room_id = None
            return

        room = self._ensure_room(canonical_room_id, frame_idx=frame_idx, timestamp=timestamp)
        room.last_observed_frame_idx = int(frame_idx)
        room.last_updated_frame_idx = int(frame_idx)
        room.last_updated_timestamp = float(timestamp)
        if room.lifecycle_state not in {RoomLifecycleState.COMMITTED, RoomLifecycleState.MERGE_OR_SPLIT_PENDING}:
            room.lifecycle_state = RoomLifecycleState.ACTIVE

        if self.active_room_id is not None and self.active_room_id != canonical_room_id:
            previous = self._ensure_room(self.active_room_id, frame_idx=frame_idx, timestamp=timestamp)
            previous.last_departed_frame_idx = int(frame_idx)
            previous.last_updated_frame_idx = int(frame_idx)
            previous.last_updated_timestamp = float(timestamp)
            self._mark_dirty(
                previous,
                DirtyRoomReason.CURRENT_ROOM_CHANGE,
                trigger_name="current_room_change",
                frame_idx=frame_idx,
                timestamp=timestamp,
                details={"source": str(source), "next_room_id": canonical_room_id},
            )

        if (
            room.last_departed_frame_idx is not None
            and self.active_room_id != canonical_room_id
            and int(frame_idx) > int(room.last_departed_frame_idx)
        ):
            self._mark_dirty(
                room,
                DirtyRoomReason.REVISIT,
                trigger_name="room_revisit",
                frame_idx=frame_idx,
                timestamp=timestamp,
                details={"source": str(source), "last_departed_frame_idx": int(room.last_departed_frame_idx)},
            )
            if room.lifecycle_state == RoomLifecycleState.COMMITTED:
                room.lifecycle_state = RoomLifecycleState.REVISITABLE
            elif room.lifecycle_state != RoomLifecycleState.MERGE_OR_SPLIT_PENDING:
                room.lifecycle_state = RoomLifecycleState.ACTIVE

        self.active_room_id = canonical_room_id

    def observe_export(
        self,
        *,
        frame_idx: int,
        timestamp: float,
        vector_map: Optional[Dict[str, Any]],
        tracking_report: Optional[Dict[str, Any]] = None,
        segmentation_updated: bool = False,
        export_profile: Optional[Dict[str, Any]] = None,
    ) -> None:
        vector_map = dict(vector_map or {})
        if not vector_map:
            return

        room_signature_payloads = self._build_room_signature_payloads(vector_map)
        room_signatures = {
            room_id: _stable_hash(payload)
            for room_id, payload in room_signature_payloads.items()
        }
        export_key = (
            int(frame_idx),
            bool(segmentation_updated),
            _stable_hash(
                {
                    "room_signatures": room_signatures,
                    "vertical_transition_summary": dict(vector_map.get("vertical_transition_summary") or {}),
                    "room_local_delta": {
                        "changed_room_ids": dict(export_profile or {}).get("changed_room_ids"),
                        "changed_room_structure_changed_ids": dict(export_profile or {}).get("changed_room_structure_changed_ids"),
                        "changed_room_object_delta_ids": dict(export_profile or {}).get("changed_room_object_delta_ids"),
                        "changed_room_removed_ids": dict(export_profile or {}).get("changed_room_removed_ids"),
                    },
                }
            ),
        )
        if export_key == self._last_export_key:
            return
        self._last_export_key = export_key

        room_floor_lookup = self._build_room_floor_lookup(vector_map)
        floor_status_lookup = self._build_floor_status_lookup(vector_map)
        gateway_signature_payloads = self._build_gateway_signature_payloads(vector_map)
        gateway_signatures = {
            room_id: _stable_hash(payload)
            for room_id, payload in gateway_signature_payloads.items()
        }
        containment_signatures = self._build_containment_signatures(vector_map)
        vt_statuses, vt_ids = self._build_vertical_transition_lookup(vector_map)
        latest_floor_assignment = _latest_floor_assignment(vector_map)
        current_active_floor_id = latest_floor_assignment.get("floor_id")
        current_active_floor_status = latest_floor_assignment.get("status")

        if current_active_floor_id is not None:
            current_active_floor_id = str(current_active_floor_id)
        if current_active_floor_status is not None:
            current_active_floor_status = str(current_active_floor_status)

        if (
            current_active_floor_id is not None
            and self.active_floor_id is not None
            and current_active_floor_id != self.active_floor_id
        ):
            touched_room_ids = {
                room_id
                for room_id in (self.active_room_id, self._room_for_floor(current_active_floor_id))
                if room_id is not None
            }
            for room_id in sorted(touched_room_ids):
                room = self._ensure_room(room_id, frame_idx=frame_idx, timestamp=timestamp)
                self._mark_dirty(
                    room,
                    DirtyRoomReason.FLOOR_TRANSITION,
                    trigger_name="floor_transition",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                    details={
                        "from_floor_id": self.active_floor_id,
                        "to_floor_id": current_active_floor_id,
                    },
                )

        if (
            current_active_floor_status is not None
            and self.active_floor_status is not None
            and current_active_floor_status != self.active_floor_status
            and self.active_room_id is not None
        ):
            room = self._ensure_room(self.active_room_id, frame_idx=frame_idx, timestamp=timestamp)
            self._mark_dirty(
                room,
                DirtyRoomReason.FLOOR_STATUS_CHANGE,
                trigger_name="floor_status_change",
                frame_idx=frame_idx,
                timestamp=timestamp,
                details={
                    "from_status": self.active_floor_status,
                    "to_status": current_active_floor_status,
                },
            )

        if current_active_floor_id is not None:
            self.active_floor_id = current_active_floor_id
        if current_active_floor_status is not None:
            self.active_floor_status = current_active_floor_status

        room_ids_present = set(room_signatures.keys())
        room_ids_by_active_floor = {
            room_id
            for room_id, floor_id in room_floor_lookup.items()
            if self.active_floor_id is not None and floor_id == self.active_floor_id
        }
        if not room_ids_by_active_floor and self.active_room_id is not None:
            room_ids_by_active_floor = {self.active_room_id}

        removed_room_ids = {
            room_id
            for room_id, status in self.room_statuses.items()
            if status.present_in_latest_export and room_id not in room_ids_present
        }
        for room_id in sorted(removed_room_ids):
            room = self._ensure_room(room_id, frame_idx=frame_idx, timestamp=timestamp)
            room.present_in_latest_export = False
            self._set_merge_pending(room, frame_idx=frame_idx)
            self._mark_dirty(
                room,
                DirtyRoomReason.ROOM_REMOVED,
                trigger_name="room_removed",
                frame_idx=frame_idx,
                timestamp=timestamp,
            )

        tracking_report = dict(tracking_report or {})
        tracking_impacted_room_ids = self._tracking_impacted_room_ids(tracking_report)

        export_profile = dict(export_profile or {})
        structure_changed_room_ids = _parse_room_id_csv(export_profile.get("changed_room_structure_changed_ids"))
        object_delta_room_ids = _parse_room_id_csv(export_profile.get("changed_room_object_delta_ids"))
        removed_delta_room_ids = _parse_room_id_csv(export_profile.get("changed_room_removed_ids"))
        changed_room_ids = _parse_room_id_csv(export_profile.get("changed_room_ids"))

        floor_validation = dict(vector_map.get("room_floor_validation") or {})
        floor_validation_blocked_rooms = {
            _canonical_room_id(item.get("room_id"))
            for item in list(floor_validation.get("spanning_rooms") or [])
        }
        floor_validation_blocked_rooms.discard(None)

        for room_id in sorted(room_ids_present):
            room = self._ensure_room(room_id, frame_idx=frame_idx, timestamp=timestamp)
            structural_delta_seen = False
            room.present_in_latest_export = True
            room.export_observation_count += 1
            room.last_updated_frame_idx = int(frame_idx)
            room.last_updated_timestamp = float(timestamp)
            room.floor_id = room_floor_lookup.get(room_id)
            room.floor_status = floor_status_lookup.get(room.floor_id)
            room.tracking_summary = self._tracking_summary_for_room(room_id, tracking_report)

            if segmentation_updated and room_id in room_ids_by_active_floor:
                self._mark_dirty(
                    room,
                    DirtyRoomReason.SEGMENTATION_REFRESH,
                    trigger_name="segmentation_refresh",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                    details={"floor_id": room.floor_id},
                )

            if room_id in tracking_impacted_room_ids:
                self._set_merge_pending(room, frame_idx=frame_idx)
                structural_delta_seen = True
                self._mark_dirty(
                    room,
                    DirtyRoomReason.TRACKING_STRUCTURE_DELTA,
                    trigger_name="tracking_structure_delta",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                    details=room.tracking_summary,
                )

            if room_id in structure_changed_room_ids:
                self._set_merge_pending(room, frame_idx=frame_idx)
                structural_delta_seen = True
                self._mark_dirty(
                    room,
                    DirtyRoomReason.ROOM_SIGNATURE_CHANGED,
                    trigger_name="room_local_structure_delta",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                )

            if room_id in object_delta_room_ids:
                self._mark_dirty(
                    room,
                    DirtyRoomReason.OBJECT_OR_ANCHOR_ASSIGNMENT_DELTA,
                    trigger_name="room_local_object_delta",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                )

            if room_id in removed_delta_room_ids:
                self._set_merge_pending(room, frame_idx=frame_idx)
                structural_delta_seen = True
                self._mark_dirty(
                    room,
                    DirtyRoomReason.ROOM_REMOVED,
                    trigger_name="room_local_removed_delta",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                )

            if room_id in changed_room_ids and room_id not in structure_changed_room_ids and room_id not in object_delta_room_ids:
                self._mark_dirty(
                    room,
                    DirtyRoomReason.SEGMENTATION_REFRESH,
                    trigger_name="room_local_changed",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                )

            room_signature_changed = self._update_signature_state(
                room,
                next_signature=room_signatures.get(room_id),
                signature_attr="current_room_signature",
                stability_attr="room_signature_stability_count",
                dirty_reason=DirtyRoomReason.ROOM_SIGNATURE_CHANGED,
                trigger_name="room_signature_changed",
                frame_idx=frame_idx,
                timestamp=timestamp,
                mark_merge_pending=True,
                next_signature_components=room_signature_payloads.get(room_id),
                signature_components_attr="current_room_signature_components",
                change_component_counter_attr="room_signature_change_component_counts",
                last_change_components_attr="last_room_signature_change_components",
            )
            gateway_signature_changed = self._update_signature_state(
                room,
                next_signature=gateway_signatures.get(room_id),
                signature_attr="current_gateway_signature",
                stability_attr="gateway_signature_stability_count",
                dirty_reason=DirtyRoomReason.GATEWAY_STRUCTURE_CHANGED,
                trigger_name="gateway_structure_changed",
                frame_idx=frame_idx,
                timestamp=timestamp,
                mark_merge_pending=True,
                next_signature_components=gateway_signature_payloads.get(room_id),
                signature_components_attr="current_gateway_signature_components",
                change_component_counter_attr="gateway_signature_change_component_counts",
                last_change_components_attr="last_gateway_signature_change_components",
            )
            self._update_signature_state(
                room,
                next_signature=containment_signatures.get(room_id),
                signature_attr="current_containment_signature",
                stability_attr="containment_stability_count",
                dirty_reason=DirtyRoomReason.OBJECT_OR_ANCHOR_ASSIGNMENT_DELTA,
                trigger_name="containment_signature_changed",
                frame_idx=frame_idx,
                timestamp=timestamp,
                mark_merge_pending=False,
            )
            structural_delta_seen = bool(structural_delta_seen or room_signature_changed or gateway_signature_changed)
            if structural_delta_seen:
                room.stable_refresh_opportunities_since_structural_delta = 0
            else:
                room.stable_refresh_opportunities_since_structural_delta += 1
            previous_vt_statuses = set(room.vertical_transition_statuses)
            room.vertical_transition_statuses = set(vt_statuses.get(room_id, set()))
            room.vertical_transition_ids = list(vt_ids.get(room_id, []))
            if previous_vt_statuses and previous_vt_statuses != room.vertical_transition_statuses:
                self._mark_dirty(
                    room,
                    DirtyRoomReason.VERTICAL_TRANSITION_UPDATE,
                    trigger_name="vertical_transition_update",
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                    details={
                        "statuses": sorted(str(item) for item in room.vertical_transition_statuses),
                    },
                )

            if (
                room.room_signature_stability_count >= self.stability_refresh_threshold
                and room.gateway_signature_stability_count >= self.stability_refresh_threshold
            ):
                room.merge_pending = False

            self._evaluate_room_status(
                room,
                floor_validation_failed=room_id in floor_validation_blocked_rooms,
            )

        self._record_refresh_snapshot(frame_idx=frame_idx, timestamp=timestamp)

    def evaluate_commit_readiness(self) -> Dict[str, List[str]]:
        blocked: Dict[str, List[str]] = {}
        for room_id, status in sorted(self.room_statuses.items()):
            if status.commit_block_reasons:
                blocked[room_id] = sorted(reason.value for reason in status.commit_block_reasons)
        return blocked

    def finalize_report(
        self,
        *,
        frame_idx: Optional[int],
        timestamp: Optional[float],
        public_topology_export_succeeded: bool,
    ) -> Dict[str, Any]:
        candidate_complete_rooms_pre_finalize = [
            status.room_id
            for _, status in sorted(self.room_statuses.items())
            if status.candidate_complete
        ]
        commit_ready_rooms_pre_finalize = [
            status.room_id
            for _, status in sorted(self.room_statuses.items())
            if not status.commit_block_reasons
        ]
        if public_topology_export_succeeded:
            for status in self.room_statuses.values():
                if status.candidate_complete and not status.commit_block_reasons:
                    status.lifecycle_state = RoomLifecycleState.COMMITTED
                    status.candidate_complete = False
                    status.dirty = False
                    status.dirty_reasons.clear()

        rooms = [status.to_dict() for _, status in sorted(self.room_statuses.items())]
        dirty_rooms = [room["room_id"] for room in rooms if room["dirty"]]
        candidate_complete_rooms = [room["room_id"] for room in rooms if room["candidate_complete"]]
        committed_rooms = [room["room_id"] for room in rooms if room["lifecycle_state"] == RoomLifecycleState.COMMITTED.value]
        blocked_commit_rooms = {
            room["room_id"]: room["commit_block_reasons"]
            for room in rooms
            if room["commit_block_reasons"]
        }
        return {
            "version": "0.1",
            "sequence_id": self.sequence_id,
            "frame_idx": None if frame_idx is None else int(frame_idx),
            "timestamp": None if timestamp is None else float(timestamp),
            "active_room_id": self.active_room_id,
            "active_floor_id": self.active_floor_id,
            "active_floor_status": self.active_floor_status,
            "summary": {
                "room_count": int(len(rooms)),
                "refresh_count": int(len(self.refresh_history)),
                "dirty_room_count": int(len(dirty_rooms)),
                "candidate_complete_room_count": int(len(candidate_complete_rooms)),
                "candidate_complete_room_count_pre_finalize": int(len(candidate_complete_rooms_pre_finalize)),
                "committed_room_count": int(len(committed_rooms)),
                "commit_ready_room_count_pre_finalize": int(len(commit_ready_rooms_pre_finalize)),
                "blocked_commit_room_count": int(len(blocked_commit_rooms)),
                "trigger_count": int(sum(int(value) for value in self.trigger_counts.values())),
                "trigger_counts": dict(sorted((str(key), int(value)) for key, value in self.trigger_counts.items())),
                "public_topology_export_succeeded": bool(public_topology_export_succeeded),
            },
            "dirty_rooms": dirty_rooms,
            "candidate_complete_rooms": candidate_complete_rooms,
            "candidate_complete_rooms_pre_finalize": candidate_complete_rooms_pre_finalize,
            "committed_rooms": committed_rooms,
            "commit_ready_rooms_pre_finalize": commit_ready_rooms_pre_finalize,
            "blocked_commit_reasons": blocked_commit_rooms,
            "rooms": rooms,
            "refresh_history": list(self.refresh_history),
            "trigger_history": list(self.trigger_history),
        }

    def export_json(
        self,
        path: Path,
        *,
        frame_idx: Optional[int],
        timestamp: Optional[float],
        public_topology_export_succeeded: bool,
    ) -> Path:
        payload = self.finalize_report(
            frame_idx=frame_idx,
            timestamp=timestamp,
            public_topology_export_succeeded=public_topology_export_succeeded,
        )
        path = Path(path)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        return path

    def _ensure_room(self, room_id: str, *, frame_idx: int, timestamp: float) -> RoomWorkingTopologyStatus:
        room = self.room_statuses.get(room_id)
        if room is not None:
            return room
        room = RoomWorkingTopologyStatus(
            room_id=room_id,
            first_seen_frame_idx=int(frame_idx),
            last_updated_frame_idx=int(frame_idx),
            last_updated_timestamp=float(timestamp),
        )
        room.dirty_reasons.add(DirtyRoomReason.DISCOVERED)
        self.room_statuses[room_id] = room
        self._record_trigger(
            room,
            "room_discovered",
            frame_idx=frame_idx,
            timestamp=timestamp,
            details=None,
        )
        return room

    def _mark_dirty(
        self,
        room: RoomWorkingTopologyStatus,
        reason: DirtyRoomReason,
        *,
        trigger_name: str,
        frame_idx: int,
        timestamp: float,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        room.dirty = True
        room.dirty_reasons.add(reason)
        room.last_updated_frame_idx = int(frame_idx)
        room.last_updated_timestamp = float(timestamp)
        self._record_trigger(room, trigger_name, frame_idx=frame_idx, timestamp=timestamp, details=details)

    def _set_merge_pending(self, room: RoomWorkingTopologyStatus, *, frame_idx: int) -> None:
        room.merge_pending = True
        room.last_structural_delta_frame_idx = int(frame_idx)
        room.stable_refresh_opportunities_since_structural_delta = 0

    def _record_trigger(
        self,
        room: RoomWorkingTopologyStatus,
        trigger_name: str,
        *,
        frame_idx: int,
        timestamp: float,
        details: Optional[Dict[str, Any]],
    ) -> None:
        room.trigger_counts[str(trigger_name)] += 1
        self.trigger_counts[str(trigger_name)] += 1
        payload = {
            "frame_idx": int(frame_idx),
            "timestamp": float(timestamp),
            "room_id": room.room_id,
            "trigger": str(trigger_name),
        }
        if details:
            payload["details"] = dict(details)
        room.recent_triggers.append(dict(payload))
        self.trigger_history.append(dict(payload))

    def _record_refresh_snapshot(self, *, frame_idx: int, timestamp: float) -> None:
        rooms: List[Dict[str, Any]] = []
        candidate_complete_rooms: List[str] = []
        commit_ready_rooms: List[str] = []
        blocked_commit_reasons: Dict[str, List[str]] = {}

        for room_id, status in sorted(self.room_statuses.items()):
            candidate_block_reasons = sorted(reason.value for reason in status.candidate_block_reasons)
            commit_block_reasons = sorted(reason.value for reason in status.commit_block_reasons)
            room_payload = {
                "room_id": room_id,
                "first_seen_frame_idx": status.first_seen_frame_idx,
                "last_departed_frame_idx": status.last_departed_frame_idx,
                "lifecycle_state": status.lifecycle_state.value,
                "candidate_complete": bool(status.candidate_complete),
                "candidate_readiness_score": int(status.candidate_readiness_score),
                "candidate_complete_reasons": sorted(reason.value for reason in status.candidate_complete_reasons),
                "candidate_block_reasons": candidate_block_reasons,
                "commit_block_reasons": commit_block_reasons,
                "merge_pending": bool(status.merge_pending),
                "dirty": bool(status.dirty),
                "present_in_latest_export": bool(status.present_in_latest_export),
                "floor_id": status.floor_id,
                "floor_status": status.floor_status,
                "stable_refresh_opportunities_since_structural_delta": int(
                    status.stable_refresh_opportunities_since_structural_delta
                ),
            }
            rooms.append(room_payload)
            if status.candidate_complete:
                candidate_complete_rooms.append(room_id)
            if not status.commit_block_reasons:
                commit_ready_rooms.append(room_id)
            if commit_block_reasons:
                blocked_commit_reasons[room_id] = commit_block_reasons

        self.refresh_history.append(
            {
                "frame_idx": int(frame_idx),
                "timestamp": float(timestamp),
                "active_room_id": self.active_room_id,
                "active_floor_id": self.active_floor_id,
                "active_floor_status": self.active_floor_status,
                "summary": {
                    "room_count": int(len(rooms)),
                    "candidate_complete_room_count": int(len(candidate_complete_rooms)),
                    "commit_ready_room_count": int(len(commit_ready_rooms)),
                    "blocked_commit_room_count": int(len(blocked_commit_reasons)),
                },
                "candidate_complete_rooms": candidate_complete_rooms,
                "commit_ready_rooms": commit_ready_rooms,
                "blocked_commit_reasons": blocked_commit_reasons,
                "rooms": rooms,
            }
        )

    def _update_signature_state(
        self,
        room: RoomWorkingTopologyStatus,
        *,
        next_signature: Optional[str],
        signature_attr: str,
        stability_attr: str,
        dirty_reason: DirtyRoomReason,
        trigger_name: str,
        frame_idx: int,
        timestamp: float,
        mark_merge_pending: bool,
        next_signature_components: Optional[Dict[str, Any]] = None,
        signature_components_attr: Optional[str] = None,
        change_component_counter_attr: Optional[str] = None,
        last_change_components_attr: Optional[str] = None,
    ) -> bool:
        previous_signature = getattr(room, signature_attr)
        previous_signature_components = (
            getattr(room, signature_components_attr)
            if signature_components_attr
            else None
        )
        if next_signature is None:
            setattr(room, signature_attr, None)
            setattr(room, stability_attr, 0)
            if signature_components_attr:
                setattr(room, signature_components_attr, None)
            return False
        if previous_signature is None:
            setattr(room, signature_attr, str(next_signature))
            setattr(room, stability_attr, 1)
            if signature_components_attr:
                setattr(room, signature_components_attr, _clone_signature_components(next_signature_components))
            return False
        if previous_signature != next_signature:
            if trigger_name == "gateway_structure_changed":
                changed_components = _diff_gateway_signature_components(
                    previous_signature_components,
                    next_signature_components,
                )
            else:
                changed_components = _diff_signature_components(
                    previous_signature_components,
                    next_signature_components,
                )
            polygon_change_summary = _summarize_polygon_signature_change(
                previous_signature_components,
                next_signature_components,
            )
            if (
                trigger_name == "room_signature_changed"
                and _is_ignorable_room_signature_polygon_residual(
                    changed_components,
                    previous_signature_components,
                    next_signature_components,
                    polygon_change_summary,
                )
            ):
                setattr(room, signature_attr, str(next_signature))
                setattr(room, stability_attr, int(getattr(room, stability_attr, 0)) + 1)
                if signature_components_attr:
                    setattr(room, signature_components_attr, _clone_signature_components(next_signature_components))
                return False
            stable_refreshes_before_change = int(room.stable_refresh_opportunities_since_structural_delta)
            should_relatch_merge_pending = bool(
                mark_merge_pending and self._should_relatch_merge_pending_for_signature_change(room)
            )
            setattr(room, signature_attr, str(next_signature))
            setattr(room, stability_attr, 1)
            if signature_components_attr:
                setattr(room, signature_components_attr, _clone_signature_components(next_signature_components))
            if last_change_components_attr:
                setattr(room, last_change_components_attr, list(changed_components))
            if change_component_counter_attr:
                change_component_counter = getattr(room, change_component_counter_attr)
                for component in changed_components:
                    change_component_counter[str(component)] += 1
            if should_relatch_merge_pending:
                self._set_merge_pending(room, frame_idx=frame_idx)
            details: Optional[Dict[str, Any]] = None
            if mark_merge_pending:
                details = {
                    "merge_pending_relatch": bool(should_relatch_merge_pending),
                    "stable_refresh_opportunities_before_change": stable_refreshes_before_change,
                }
            if changed_components:
                details = dict(details or {})
                if trigger_name == "gateway_structure_changed":
                    details["gateway_signature_change_components"] = list(changed_components)
                    gateway_change_summary = _summarize_gateway_signature_change(
                        previous_signature_components,
                        next_signature_components,
                    )
                    if gateway_change_summary is not None:
                        details["gateway_signature_change_summary"] = gateway_change_summary
                else:
                    details["room_signature_change_components"] = list(changed_components)
                    if polygon_change_summary is not None:
                        details["room_signature_polygon_change"] = polygon_change_summary
            self._mark_dirty(
                room,
                dirty_reason,
                trigger_name=trigger_name,
                frame_idx=frame_idx,
                timestamp=timestamp,
                details=details,
            )
            return True
        setattr(room, stability_attr, int(getattr(room, stability_attr, 0)) + 1)
        if signature_components_attr:
            setattr(room, signature_components_attr, _clone_signature_components(next_signature_components))
        return False

    def _should_relatch_merge_pending_for_signature_change(self, room: RoomWorkingTopologyStatus) -> bool:
        if room.merge_pending:
            return True
        return room.stable_refresh_opportunities_since_structural_delta < self.signature_relatch_grace_refresh_threshold

    def _evaluate_room_status(
        self,
        room: RoomWorkingTopologyStatus,
        *,
        floor_validation_failed: bool,
    ) -> None:
        candidate_reasons: Set[CandidateCompleteReason] = set()
        candidate_blocks: Set[CandidateBlockReason] = set()
        commit_blocks: Set[CommitBlockReason] = set()

        if not room.present_in_latest_export:
            candidate_blocks.add(CandidateBlockReason.ROOM_MISSING_FROM_LATEST_EXPORT)
            commit_blocks.add(CommitBlockReason.ROOM_MISSING_FROM_LATEST_EXPORT)
        if room.room_id == self.active_room_id:
            candidate_blocks.add(CandidateBlockReason.ROOM_CURRENTLY_ACTIVE)
            commit_blocks.add(CommitBlockReason.ROOM_CURRENTLY_ACTIVE)
        else:
            if room.last_departed_frame_idx is not None or room.export_observation_count >= self.stability_refresh_threshold:
                candidate_reasons.add(CandidateCompleteReason.LEFT_ACTIVE_ROOM)
            else:
                candidate_blocks.add(CandidateBlockReason.NO_LEAVE_LIKE_SIGNAL)
                commit_blocks.add(CommitBlockReason.NO_LEAVE_LIKE_SIGNAL)

        if room.room_signature_stability_count >= self.stability_refresh_threshold:
            candidate_reasons.add(CandidateCompleteReason.ROOM_SIGNATURE_STABLE)
        else:
            candidate_blocks.add(CandidateBlockReason.ROOM_SIGNATURE_NOT_STABLE)
            commit_blocks.add(CommitBlockReason.ROOM_SIGNATURE_NOT_STABLE)

        if room.gateway_signature_stability_count >= self.stability_refresh_threshold:
            candidate_reasons.add(CandidateCompleteReason.GATEWAY_STRUCTURE_STABLE)
        else:
            commit_blocks.add(CommitBlockReason.GATEWAY_STRUCTURE_NOT_STABLE)

        if room.containment_stability_count >= self.stability_refresh_threshold:
            candidate_reasons.add(CandidateCompleteReason.CONTAINMENT_STABLE)
        else:
            commit_blocks.add(CommitBlockReason.CONTAINMENT_NOT_STABLE)

        if room.floor_status in (None, "", "stable", "confirmed"):
            candidate_reasons.add(CandidateCompleteReason.FLOOR_ASSIGNMENT_STABLE)
        else:
            candidate_blocks.add(CandidateBlockReason.FLOOR_STATUS_NOT_STABLE)
            commit_blocks.add(CommitBlockReason.FLOOR_STATUS_NOT_STABLE)

        if room.merge_pending:
            commit_blocks.add(CommitBlockReason.MERGE_OR_SPLIT_PENDING)

        if any(str(status) != "supported" for status in room.vertical_transition_statuses):
            commit_blocks.add(CommitBlockReason.VERTICAL_TRANSITION_PARTIAL)

        if floor_validation_failed:
            commit_blocks.add(CommitBlockReason.ROOM_FLOOR_VALIDATION_FAILED)

        candidate_readiness_score = len(candidate_reasons)
        if candidate_readiness_score < self.candidate_readiness_threshold:
            candidate_blocks.add(CandidateBlockReason.INSUFFICIENT_READINESS_SCORE)

        room.candidate_readiness_score = int(candidate_readiness_score)
        room.candidate_complete_reasons = set(candidate_reasons)
        room.candidate_block_reasons = set(candidate_blocks)
        room.commit_block_reasons = set(commit_blocks)
        room.candidate_complete = not candidate_blocks

        if room.lifecycle_state == RoomLifecycleState.COMMITTED:
            return
        if room.merge_pending:
            room.lifecycle_state = RoomLifecycleState.MERGE_OR_SPLIT_PENDING
            return
        if room.candidate_complete:
            room.lifecycle_state = RoomLifecycleState.CANDIDATE_COMPLETE
            return
        if room.export_observation_count <= 1 and room.last_departed_frame_idx is None:
            room.lifecycle_state = RoomLifecycleState.DISCOVERING
            return
        room.lifecycle_state = RoomLifecycleState.ACTIVE

    def _build_room_signature_payloads(self, vector_map: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        payloads: Dict[str, Dict[str, Any]] = {}
        for room_record in list(vector_map.get("rooms") or []):
            room_id = _canonical_room_id(room_record.get("room_id") or room_record.get("id"))
            if room_id is None:
                continue
            payloads[room_id] = {
                "room_id": room_id,
                "floor_id": room_record.get("floor_id"),
                "room_type": str(room_record.get("room_type", "unknown")),
                "status": str(room_record.get("status", "confirmed")),
                "polygon": _build_room_signature_polygon_payload(list(room_record.get("polygon") or [])),
                "center": [
                    _quantize_bucket(value, _ROOM_SIGNATURE_GEOMETRY_QUANTIZATION_M)
                    for value in list(room_record.get("center") or [])[:2]
                ],
                "area_m2": _quantize_bucket(
                    room_record.get("area_m2", 0.0) or 0.0,
                    _ROOM_SIGNATURE_AREA_QUANTIZATION_M2,
                ),
            }
        return payloads

    def _build_room_signatures(self, vector_map: Dict[str, Any]) -> Dict[str, str]:
        return {
            room_id: _stable_hash(payload)
            for room_id, payload in self._build_room_signature_payloads(vector_map).items()
        }

    def _build_gateway_signature_payloads(self, vector_map: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        by_room: Dict[str, List[Dict[str, Any]]] = {}
        for room_record in list(vector_map.get("rooms") or []):
            room_id = _canonical_room_id(room_record.get("room_id") or room_record.get("id"))
            if room_id is not None:
                by_room.setdefault(room_id, [])

        for gateway in list(vector_map.get("gateways") or []):
            connects = [
                _canonical_room_id(item)
                for item in list(gateway.get("connects") or [])
            ]
            connects = [item for item in connects if item is not None]
            if len(connects) != 2:
                continue
            sorted_connects = sorted(connects)
            pos_world = list(gateway.get("pos_world") or [])
            entry = {
                "type": gateway.get("type"),
                "relation_scope": gateway.get("relation_scope"),
                "connects": sorted_connects,
                "floor_id": gateway.get("floor_id"),
                # Bucket gateway position to suppress small center jitter while
                # preserving type/connectivity changes as hard resets.
                "pos_world": [
                    _quantize_bucket(value, _GATEWAY_SIGNATURE_POSITION_QUANTIZATION_M)
                    for value in pos_world[:2]
                ],
            }
            for room_id in sorted_connects:
                other_room_id = sorted_connects[1] if room_id == sorted_connects[0] else sorted_connects[0]
                by_room.setdefault(room_id, []).append(
                    {
                        "other_room_id": other_room_id,
                        **entry,
                    }
                )

        payloads: Dict[str, Dict[str, Any]] = {}
        for room_id, entries in by_room.items():
            canonical_entries = sorted(entries, key=lambda item: json.dumps(item, sort_keys=True))
            payloads[room_id] = {
                "gateway_count": int(len(canonical_entries)),
                "entries": canonical_entries,
            }
        return payloads

    def _build_gateway_signatures(self, vector_map: Dict[str, Any]) -> Dict[str, str]:
        return {
            room_id: _stable_hash(payload)
            for room_id, payload in self._build_gateway_signature_payloads(vector_map).items()
        }

    def _build_containment_signatures(self, vector_map: Dict[str, Any]) -> Dict[str, str]:
        payload_by_room: Dict[str, Dict[str, List[str]]] = {}
        for room_record in list(vector_map.get("rooms") or []):
            room_id = _canonical_room_id(room_record.get("room_id") or room_record.get("id"))
            if room_id is not None:
                payload_by_room.setdefault(room_id, {"objects": [], "anchors": []})

        for obj in list(vector_map.get("objects") or []):
            room_id = _canonical_room_id(obj.get("room_id") or obj.get("room_uuid"))
            if room_id is None:
                continue
            payload_by_room.setdefault(room_id, {"objects": [], "anchors": []})
            payload_by_room[room_id]["objects"].append(str(obj.get("id")))

        for anchor in list(vector_map.get("anchors") or []):
            room_id = _canonical_room_id(anchor.get("room_id"))
            if room_id is None:
                continue
            payload_by_room.setdefault(room_id, {"objects": [], "anchors": []})
            payload_by_room[room_id]["anchors"].append(str(anchor.get("id")))

        signatures: Dict[str, str] = {}
        for room_id, payload in payload_by_room.items():
            signatures[room_id] = _stable_hash(
                {
                    "objects": sorted(str(item) for item in payload.get("objects", [])),
                    "anchors": sorted(str(item) for item in payload.get("anchors", [])),
                }
            )
        return signatures

    def _build_vertical_transition_lookup(
        self,
        vector_map: Dict[str, Any],
    ) -> Tuple[Dict[str, Set[str]], Dict[str, List[str]]]:
        statuses: Dict[str, Set[str]] = {}
        transition_ids: Dict[str, List[str]] = {}
        for transition in list(vector_map.get("vertical_transitions") or []):
            transition_status = str(transition.get("status", "unknown"))
            transition_id = str(transition.get("transition_id", "unknown"))
            for raw_room_id in (transition.get("from_room_id"), transition.get("to_room_id")):
                room_id = _canonical_room_id(raw_room_id)
                if room_id is None:
                    continue
                statuses.setdefault(room_id, set()).add(transition_status)
                transition_ids.setdefault(room_id, []).append(transition_id)
        return statuses, transition_ids

    def _build_room_floor_lookup(self, vector_map: Dict[str, Any]) -> Dict[str, Optional[str]]:
        lookup: Dict[str, Optional[str]] = {}
        for room in list(vector_map.get("rooms") or []):
            room_id = _canonical_room_id(room.get("room_id") or room.get("id"))
            if room_id is None:
                continue
            floor_id = room.get("floor_id")
            lookup[room_id] = None if floor_id in (None, "") else str(floor_id)
        return lookup

    def _build_floor_status_lookup(self, vector_map: Dict[str, Any]) -> Dict[str, Optional[str]]:
        lookup: Dict[str, Optional[str]] = {}
        for floor in list(vector_map.get("floors") or []):
            floor_id = floor.get("floor_id")
            if floor_id in (None, ""):
                continue
            lookup[str(floor_id)] = None if floor.get("status") in (None, "") else str(floor.get("status"))
        return lookup

    def _tracking_impacted_room_ids(self, tracking_report: Dict[str, Any]) -> Set[str]:
        impacted: Set[str] = set()
        for key in ("new_rooms", "retained_missing", "dropped_missing"):
            for item in list(tracking_report.get(key) or []):
                room_id = _canonical_room_id(item.get("global_id"))
                if room_id is not None:
                    impacted.add(room_id)
        return impacted

    def _tracking_summary_for_room(self, room_id: str, tracking_report: Dict[str, Any]) -> Dict[str, int]:
        summary: Dict[str, int] = {}
        for key in ("matched", "new_rooms", "retained_missing", "dropped_missing"):
            count = 0
            for item in list(tracking_report.get(key) or []):
                if _canonical_room_id(item.get("global_id")) == room_id:
                    count += 1
            if count > 0:
                summary[f"{key}_count"] = int(count)
        return summary

    def _room_for_floor(self, floor_id: str) -> Optional[str]:
        for room_id, status in self.room_statuses.items():
            if status.floor_id == floor_id:
                return room_id
        return None


def _canonical_room_id(value: Optional[Any]) -> Optional[str]:
    if value in (None, ""):
        return None
    if isinstance(value, str) and value.startswith("room_"):
        return value
    try:
        return f"room_{int(value)}"
    except Exception:
        value_str = str(value)
        if value_str.startswith("room_"):
            return value_str
        return value_str


def _latest_floor_assignment(vector_map: Dict[str, Any]) -> Dict[str, Any]:
    assignments = list(vector_map.get("frame_floor_assignments") or [])
    if not assignments:
        return {}
    latest = max(assignments, key=lambda item: int(item.get("frame_idx", -1)))
    return {
        "floor_id": latest.get("floor_id"),
        "status": latest.get("status"),
    }


def _parse_room_id_csv(value: Optional[Any]) -> Set[str]:
    if value in (None, ""):
        return set()
    items: Set[str] = set()
    for token in str(value).split(","):
        token = token.strip()
        if not token:
            continue
        room_id = _canonical_room_id(token)
        if room_id is not None:
            items.add(room_id)
    return items


def _stable_hash(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.blake2b(encoded, digest_size=16).hexdigest()


def _build_room_signature_polygon_payload(polygon: List[Any]) -> List[List[int]]:
    quantized = [
        (
            _quantize_bucket(point[0], _ROOM_SIGNATURE_GEOMETRY_QUANTIZATION_M),
            _quantize_bucket(point[1], _ROOM_SIGNATURE_GEOMETRY_QUANTIZATION_M),
        )
        for point in list(polygon or [])
        if isinstance(point, (list, tuple)) and len(point) >= 2
    ]
    quantized = _dedupe_polygon_ring_points(quantized)
    if len(quantized) < 3:
        return [[int(point[0]), int(point[1])] for point in quantized]
    canonical = _canonicalize_polygon_ring_points(quantized)
    simplified = _simplify_polygon_ring_points(
        canonical,
        jaggedness_epsilon=_ROOM_SIGNATURE_POLYGON_JAGGEDNESS_EPSILON_BUCKETS,
        short_edge_threshold=_ROOM_SIGNATURE_POLYGON_SHORT_EDGE_BUCKETS,
    )
    canonical = _canonicalize_polygon_ring_points(simplified)
    return [[int(point[0]), int(point[1])] for point in canonical]


def _clone_signature_components(payload: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if payload is None:
        return None
    return json.loads(json.dumps(payload, sort_keys=True))


def _diff_signature_components(
    previous_payload: Optional[Dict[str, Any]],
    next_payload: Optional[Dict[str, Any]],
) -> List[str]:
    if previous_payload is None or next_payload is None:
        return []
    changed: List[str] = []
    for component in _ROOM_SIGNATURE_COMPONENT_ORDER:
        if previous_payload.get(component) != next_payload.get(component):
            changed.append(component)
    known_components = set(_ROOM_SIGNATURE_COMPONENT_ORDER)
    extra_components = sorted((set(previous_payload.keys()) | set(next_payload.keys())) - known_components)
    for component in extra_components:
        if previous_payload.get(component) != next_payload.get(component):
            changed.append(str(component))
    return changed


def _gateway_component_distribution(
    entries: List[Dict[str, Any]],
    component: str,
) -> List[str]:
    return sorted(
        json.dumps(entry.get(component), sort_keys=True, separators=(",", ":"))
        for entry in entries
    )


def _diff_gateway_signature_components(
    previous_payload: Optional[Dict[str, Any]],
    next_payload: Optional[Dict[str, Any]],
) -> List[str]:
    if previous_payload is None or next_payload is None:
        return []
    changed: List[str] = []
    previous_entries = list(previous_payload.get("entries") or [])
    next_entries = list(next_payload.get("entries") or [])
    previous_count = int(previous_payload.get("gateway_count", len(previous_entries)))
    next_count = int(next_payload.get("gateway_count", len(next_entries)))
    if previous_count != next_count:
        changed.append("gateway_count")
    for component in _GATEWAY_SIGNATURE_COMPONENT_ORDER:
        if component == "gateway_count":
            continue
        if _gateway_component_distribution(previous_entries, component) != _gateway_component_distribution(next_entries, component):
            changed.append(component)
    extra_components = sorted((set(previous_payload.keys()) | set(next_payload.keys())) - {"gateway_count", "entries"})
    for component in extra_components:
        if previous_payload.get(component) != next_payload.get(component):
            changed.append(str(component))
    return changed


def _dedupe_polygon_ring_points(points: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    deduped: List[Tuple[int, int]] = []
    for point in points:
        point_key = (int(point[0]), int(point[1]))
        if not deduped or deduped[-1] != point_key:
            deduped.append(point_key)
    if len(deduped) > 1 and deduped[0] == deduped[-1]:
        deduped.pop()
    return deduped


def _canonicalize_polygon_ring_points(points: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    if not points:
        return []
    if len(points) == 1:
        return [(int(points[0][0]), int(points[0][1]))]
    canonical_options: List[List[Tuple[int, int]]] = []
    for candidate in (list(points), list(reversed(points))):
        candidate = [(int(point[0]), int(point[1])) for point in candidate]
        min_index = min(range(len(candidate)), key=lambda idx: (candidate[idx][0], candidate[idx][1], idx))
        canonical_options.append(candidate[min_index:] + candidate[:min_index])
    return min(canonical_options)


def _point_distance(point_a: Tuple[int, int], point_b: Tuple[int, int]) -> float:
    return math.hypot(float(point_a[0] - point_b[0]), float(point_a[1] - point_b[1]))


def _point_to_segment_distance(
    point: Tuple[int, int],
    start: Tuple[int, int],
    end: Tuple[int, int],
) -> float:
    start_x = float(start[0])
    start_y = float(start[1])
    end_x = float(end[0])
    end_y = float(end[1])
    point_x = float(point[0])
    point_y = float(point[1])
    delta_x = end_x - start_x
    delta_y = end_y - start_y
    segment_length_sq = (delta_x * delta_x) + (delta_y * delta_y)
    if segment_length_sq <= 0.0:
        return math.hypot(point_x - start_x, point_y - start_y)
    t = ((point_x - start_x) * delta_x + (point_y - start_y) * delta_y) / segment_length_sq
    t = max(0.0, min(1.0, t))
    proj_x = start_x + (t * delta_x)
    proj_y = start_y + (t * delta_y)
    return math.hypot(point_x - proj_x, point_y - proj_y)


def _simplify_polygon_ring_points(
    points: List[Tuple[int, int]],
    *,
    jaggedness_epsilon: float,
    short_edge_threshold: float,
) -> List[Tuple[int, int]]:
    simplified = list(points)
    if len(simplified) < 4:
        return simplified

    while len(simplified) >= 4:
        changed = False
        next_points: List[Tuple[int, int]] = []
        point_count = len(simplified)
        for idx, point in enumerate(simplified):
            previous_point = simplified[idx - 1]
            next_point = simplified[(idx + 1) % point_count]
            deviation = _point_to_segment_distance(point, previous_point, next_point)
            shortest_adjacent_edge = min(
                _point_distance(previous_point, point),
                _point_distance(point, next_point),
            )
            if deviation <= jaggedness_epsilon and shortest_adjacent_edge <= short_edge_threshold:
                changed = True
                continue
            next_points.append(point)

        next_points = _dedupe_polygon_ring_points(next_points)
        if not changed or len(next_points) < 3:
            break
        simplified = next_points
    return simplified


def _polygon_bucket_bbox(points: List[Tuple[int, int]]) -> Optional[List[int]]:
    if not points:
        return None
    xs = [int(point[0]) for point in points]
    ys = [int(point[1]) for point in points]
    return [min(xs), min(ys), max(xs), max(ys)]


def _summarize_polygon_signature_change(
    previous_payload: Optional[Dict[str, Any]],
    next_payload: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if previous_payload is None or next_payload is None:
        return None
    previous_polygon = [
        (int(point[0]), int(point[1]))
        for point in list(previous_payload.get("polygon") or [])
        if isinstance(point, (list, tuple)) and len(point) >= 2
    ]
    next_polygon = [
        (int(point[0]), int(point[1]))
        for point in list(next_payload.get("polygon") or [])
        if isinstance(point, (list, tuple)) and len(point) >= 2
    ]
    if previous_polygon == next_polygon:
        return None
    return {
        "prev_vertex_count": int(len(previous_polygon)),
        "next_vertex_count": int(len(next_polygon)),
        "prev_bbox": _polygon_bucket_bbox(previous_polygon),
        "next_bbox": _polygon_bucket_bbox(next_polygon),
        "changed_vertex_bucket_count": int(len(set(previous_polygon) ^ set(next_polygon))),
    }


def _summarize_gateway_signature_change(
    previous_payload: Optional[Dict[str, Any]],
    next_payload: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if previous_payload is None or next_payload is None:
        return None
    previous_entries = list(previous_payload.get("entries") or [])
    next_entries = list(next_payload.get("entries") or [])
    if previous_entries == next_entries:
        return None
    previous_other_room_ids = sorted({str(entry.get("other_room_id")) for entry in previous_entries if entry.get("other_room_id") not in (None, "")})
    next_other_room_ids = sorted({str(entry.get("other_room_id")) for entry in next_entries if entry.get("other_room_id") not in (None, "")})
    previous_positions = sorted(
        [list(entry.get("pos_world") or []) for entry in previous_entries],
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":")),
    )
    next_positions = sorted(
        [list(entry.get("pos_world") or []) for entry in next_entries],
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":")),
    )
    summary = {
        "prev_gateway_count": int(previous_payload.get("gateway_count", len(previous_entries))),
        "next_gateway_count": int(next_payload.get("gateway_count", len(next_entries))),
        "prev_other_room_ids": previous_other_room_ids,
        "next_other_room_ids": next_other_room_ids,
    }
    if previous_positions != next_positions:
        summary["prev_positions"] = previous_positions
        summary["next_positions"] = next_positions
    return summary


def _is_ignorable_room_signature_polygon_residual(
    changed_components: List[str],
    previous_payload: Optional[Dict[str, Any]],
    next_payload: Optional[Dict[str, Any]],
    polygon_change_summary: Optional[Dict[str, Any]],
) -> bool:
    if changed_components != ["polygon"]:
        return False
    if previous_payload is None or next_payload is None or polygon_change_summary is None:
        return False
    if polygon_change_summary.get("prev_vertex_count") != polygon_change_summary.get("next_vertex_count"):
        return False
    if int(polygon_change_summary.get("changed_vertex_bucket_count", 0)) != 2:
        return False

    prev_bbox = polygon_change_summary.get("prev_bbox")
    next_bbox = polygon_change_summary.get("next_bbox")
    if not isinstance(prev_bbox, list) or not isinstance(next_bbox, list) or len(prev_bbox) != 4 or len(next_bbox) != 4:
        return False
    bbox_deltas = [int(next_bbox[idx]) - int(prev_bbox[idx]) for idx in range(4)]
    changed_bbox_indices = [idx for idx, delta in enumerate(bbox_deltas) if delta != 0]
    if len(changed_bbox_indices) != 1:
        return False
    changed_bbox_index = changed_bbox_indices[0]
    return abs(bbox_deltas[changed_bbox_index]) == 1


def _quantize_bucket(value: Any, step: float) -> int:
    if step <= 0.0:
        raise ValueError("quantization step must be positive")
    return int(math.floor((float(value) / float(step)) + 0.5))
