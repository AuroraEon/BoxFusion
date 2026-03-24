from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from boxfusion.floor_artifacts import canonicalize_floors


DEFAULT_FLOOR_MANAGER_CONFIG: Dict[str, float] = {
    "stable_assignment_margin_m": 0.75,
    "pending_assignment_margin_m": 1.10,
    "new_floor_min_separation_m": 2.20,
    "new_floor_confirm_frames": 3,
    "floor_band_padding_m": 0.75,
    "min_confirmed_support_frames": 5,
}


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


@dataclass
class FloorObservation:
    frame_idx: int
    timestamp: float
    pose_z: float
    floor_id: Optional[str]
    status: str
    confidence: float
    reason: str
    floor_index: Optional[int] = None
    is_transition: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame_idx": int(self.frame_idx),
            "timestamp": _round_float(self.timestamp),
            "pose_z": _round_float(self.pose_z),
            "floor_id": self.floor_id,
            "floor_index": self.floor_index,
            "status": self.status,
            "confidence": _round_float(self.confidence),
            "reason": self.reason,
            "is_transition": bool(self.is_transition),
        }


@dataclass
class FloorEvent:
    frame_idx: int
    event: str
    floor_id: Optional[str] = None
    pose_z: Optional[float] = None
    candidate_z: Optional[float] = None
    pending_sample_count: int = 0
    note: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame_idx": int(self.frame_idx),
            "event": str(self.event),
            "floor_id": self.floor_id,
            "pose_z": _round_float(self.pose_z),
            "candidate_z": _round_float(self.candidate_z),
            "pending_sample_count": int(self.pending_sample_count),
            "note": self.note,
        }


@dataclass
class FloorHypothesis:
    floor_id: str
    creation_order: int
    z_samples: List[float] = field(default_factory=list)
    keyframe_count: int = 0
    frame_count: int = 0
    first_seen_frame: Optional[int] = None
    last_seen_frame: Optional[int] = None
    status: str = "tentative"
    confidence: float = 0.0

    def observe(self, pose_z: float, frame_idx: int, is_keyframe: bool, cfg: Dict[str, float]) -> None:
        self.z_samples.append(float(pose_z))
        self.frame_count += 1
        if is_keyframe:
            self.keyframe_count += 1
        if self.first_seen_frame is None:
            self.first_seen_frame = int(frame_idx)
        self.last_seen_frame = int(frame_idx)
        min_support = max(1, int(cfg["min_confirmed_support_frames"]))
        self.status = "confirmed" if self.frame_count >= min_support else "tentative"
        self.confidence = min(1.0, float(self.frame_count) / float(min_support))

    @property
    def z_center(self) -> float:
        if not self.z_samples:
            return 0.0
        return float(np.median(np.asarray(self.z_samples, dtype=np.float32)))

    @property
    def z_min(self) -> float:
        if not self.z_samples:
            return 0.0
        return float(np.min(np.asarray(self.z_samples, dtype=np.float32)))

    @property
    def z_max(self) -> float:
        if not self.z_samples:
            return 0.0
        return float(np.max(np.asarray(self.z_samples, dtype=np.float32)))

    def to_dict(
        self,
        floor_index: int,
        cfg: Dict[str, float],
        display_floor_id: Optional[str] = None,
        display_order: Optional[int] = None,
    ) -> Dict[str, Any]:
        band_pad = float(cfg["floor_band_padding_m"])
        return {
            "floor_id": self.floor_id,
            "floor_index": int(floor_index),
            "display_floor_id": display_floor_id or self.floor_id,
            "display_order": int(display_order if display_order is not None else floor_index + 1),
            "z_min": _round_float(self.z_min - band_pad),
            "z_max": _round_float(self.z_max + band_pad),
            "z_center": _round_float(self.z_center),
            "confidence": _round_float(self.confidence),
            "status": self.status,
            "support_statistics": {
                "frame_count": int(self.frame_count),
                "keyframe_count": int(self.keyframe_count),
                "first_seen_frame": self.first_seen_frame,
                "last_seen_frame": self.last_seen_frame,
                "raw_z_min": _round_float(self.z_min),
                "raw_z_max": _round_float(self.z_max),
                "sample_count": int(len(self.z_samples)),
            },
        }


class FloorManager:
    """Stateful, runtime-friendly floor assignment based on pose height bands."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        raw_cfg = dict((config or {}).get("floor_segmentation", config or {}))
        self.config = dict(DEFAULT_FLOOR_MANAGER_CONFIG)
        self.config.update({key: raw_cfg[key] for key in raw_cfg if key in self.config})
        self.floors: Dict[str, FloorHypothesis] = {}
        self.observations: List[FloorObservation] = []
        self.events: List[FloorEvent] = []
        self._next_floor_number = 1
        self._pending_samples: List[float] = []

    def observe_pose(
        self,
        frame_idx: int,
        timestamp: float,
        pose_z: float,
        is_keyframe: bool = False,
    ) -> FloorObservation:
        pose_z = float(pose_z)
        stable_margin = float(self.config["stable_assignment_margin_m"])
        pending_margin = float(self.config["pending_assignment_margin_m"])
        separation = float(self.config["new_floor_min_separation_m"])
        confirm_frames = max(1, int(self.config["new_floor_confirm_frames"]))

        if not self.floors:
            floor = self._create_floor(pose_z, frame_idx, is_keyframe)
            self._record_event(
                frame_idx=frame_idx,
                event="bootstrap_floor_created",
                floor_id=floor.floor_id,
                pose_z=pose_z,
                candidate_z=pose_z,
                note="first floor bootstrapped from the first observation",
            )
            return self._record_observation(
                frame_idx=frame_idx,
                timestamp=timestamp,
                pose_z=pose_z,
                floor_id=floor.floor_id,
                status="stable",
                confidence=1.0,
                reason="bootstrap_floor",
            )

        nearest_floor = min(self.floors.values(), key=lambda item: abs(pose_z - item.z_center))
        nearest_distance = abs(pose_z - nearest_floor.z_center)
        if nearest_distance <= stable_margin:
            previous_status = nearest_floor.status
            nearest_floor.observe(pose_z, frame_idx, is_keyframe, self.config)
            self._pending_samples = []
            if previous_status != nearest_floor.status and nearest_floor.status == "confirmed":
                self._record_event(
                    frame_idx=frame_idx,
                    event="floor_support_confirmed",
                    floor_id=nearest_floor.floor_id,
                    pose_z=pose_z,
                    candidate_z=nearest_floor.z_center,
                    note="floor reached the confirmed support threshold",
                )
            return self._record_observation(
                frame_idx=frame_idx,
                timestamp=timestamp,
                pose_z=pose_z,
                floor_id=nearest_floor.floor_id,
                status="stable",
                confidence=max(0.55, 1.0 - nearest_distance / max(stable_margin, 1e-6)),
                reason="within_floor_band",
            )

        if nearest_distance <= pending_margin:
            return self._record_observation(
                frame_idx=frame_idx,
                timestamp=timestamp,
                pose_z=pose_z,
                floor_id=nearest_floor.floor_id,
                status="transition",
                confidence=max(0.2, 1.0 - nearest_distance / max(pending_margin, 1e-6)),
                reason="near_floor_band_boundary",
            )

        if self._is_new_floor_sample(pose_z, separation):
            if self._pending_samples and abs(np.mean(self._pending_samples) - pose_z) > pending_margin:
                self._record_event(
                    frame_idx=frame_idx,
                    event="tentative_floor_reset",
                    pose_z=pose_z,
                    candidate_z=float(np.mean(self._pending_samples)),
                    pending_sample_count=len(self._pending_samples),
                    note="tentative new-floor evidence reset because the next out-of-band sample jumped elsewhere",
                )
                self._pending_samples = []
            if not self._pending_samples:
                self._record_event(
                    frame_idx=frame_idx,
                    event="tentative_floor_started",
                    pose_z=pose_z,
                    candidate_z=pose_z,
                    pending_sample_count=1,
                    note="out-of-band height started a tentative new-floor hypothesis",
                )
            self._pending_samples.append(pose_z)
            if len(self._pending_samples) >= confirm_frames:
                candidate_z = float(np.median(np.asarray(self._pending_samples, dtype=np.float32)))
                floor = self._create_floor(candidate_z, frame_idx, is_keyframe)
                self._record_event(
                    frame_idx=frame_idx,
                    event="new_floor_confirmed",
                    floor_id=floor.floor_id,
                    pose_z=pose_z,
                    candidate_z=candidate_z,
                    pending_sample_count=len(self._pending_samples),
                    note="tentative height band accumulated enough out-of-band evidence to create a floor",
                )
                self._pending_samples = []
                return self._record_observation(
                    frame_idx=frame_idx,
                    timestamp=timestamp,
                    pose_z=pose_z,
                    floor_id=floor.floor_id,
                    status="stable",
                    confidence=0.85,
                    reason="new_floor_confirmed",
                )

        return self._record_observation(
            frame_idx=frame_idx,
            timestamp=timestamp,
            pose_z=pose_z,
            floor_id=None,
            status="transition",
            confidence=0.25,
            reason="between_known_floor_bands",
        )

    def assign_height(self, pose_z: float) -> Dict[str, Any]:
        if not self.floors:
            return {
                "floor_id": None,
                "floor_index": None,
                "display_floor_id": None,
                "display_order": None,
                "status": "unknown",
                "confidence": 0.0,
                "reason": "no_floors_available",
            }
        pose_z = float(pose_z)
        nearest_floor = min(self.floors.values(), key=lambda item: abs(pose_z - item.z_center))
        nearest_distance = abs(pose_z - nearest_floor.z_center)
        stable_margin = float(self.config["stable_assignment_margin_m"])
        display_meta = self._display_metadata_lookup().get(nearest_floor.floor_id, {})
        floor_index = display_meta.get("floor_index")
        if nearest_distance <= stable_margin:
            return {
                "floor_id": nearest_floor.floor_id,
                "floor_index": int(floor_index),
                "display_floor_id": display_meta.get("display_floor_id"),
                "display_order": display_meta.get("display_order"),
                "status": "stable",
                "confidence": max(0.5, 1.0 - nearest_distance / max(stable_margin, 1e-6)),
                "reason": "nearest_confirmed_floor",
            }
        return {
            "floor_id": nearest_floor.floor_id,
            "floor_index": int(floor_index),
            "display_floor_id": display_meta.get("display_floor_id"),
            "display_order": display_meta.get("display_order"),
            "status": "uncertain",
            "confidence": 0.25,
            "reason": "height_outside_floor_band",
        }

    def export_floors(self) -> List[Dict[str, Any]]:
        display_lookup = self._display_metadata_lookup()
        return [
            self.floors[floor_id].to_dict(
                floor_index=int(meta["floor_index"]),
                cfg=self.config,
                display_floor_id=str(meta["display_floor_id"]),
                display_order=int(meta["display_order"]),
            )
            for floor_id, meta in display_lookup.items()
        ]

    def export_assignment_history(self) -> List[Dict[str, Any]]:
        display_lookup = self._display_metadata_lookup()
        history: List[Dict[str, Any]] = []
        for item in self.observations:
            payload = item.to_dict()
            if item.floor_id in display_lookup:
                payload.update(display_lookup[item.floor_id])
            else:
                payload["display_floor_id"] = None
                payload["display_order"] = None
            history.append(payload)
        return history

    def export_debug_events(self) -> List[Dict[str, Any]]:
        display_lookup = self._display_metadata_lookup()
        events: List[Dict[str, Any]] = []
        for item in self.events:
            payload = item.to_dict()
            if item.floor_id in display_lookup:
                payload.update({
                    "display_floor_id": display_lookup[item.floor_id].get("display_floor_id"),
                    "display_order": display_lookup[item.floor_id].get("display_order"),
                })
            else:
                payload["display_floor_id"] = None
                payload["display_order"] = None
            events.append(payload)
        return events

    def export_pending_state(self) -> Dict[str, Any]:
        candidate_z = None
        if self._pending_samples:
            candidate_z = float(np.median(np.asarray(self._pending_samples, dtype=np.float32)))
        return {
            "pending_sample_count": int(len(self._pending_samples)),
            "candidate_z": _round_float(candidate_z),
            "status": "open" if self._pending_samples else "empty",
        }

    def _ordered_floor_ids(self) -> List[str]:
        return [
            floor.floor_id
            for floor in sorted(
                self.floors.values(),
                key=lambda item: (item.z_center, item.creation_order),
            )
        ]

    def _display_metadata_lookup(self) -> Dict[str, Dict[str, Any]]:
        canonical = canonicalize_floors(
            [
                {
                    "floor_id": floor.floor_id,
                    "creation_order": floor.creation_order,
                    "z_center": floor.z_center,
                }
                for floor in self.floors.values()
            ]
        )
        return {
            str(item["floor_id"]): {
                "floor_index": int(item["floor_index"]),
                "display_floor_id": str(item["display_floor_id"]),
                "display_order": int(item["display_order"]),
            }
            for item in canonical
        }

    def _is_new_floor_sample(self, pose_z: float, separation: float) -> bool:
        return all(abs(float(pose_z) - floor.z_center) >= separation for floor in self.floors.values())

    def _create_floor(self, pose_z: float, frame_idx: int, is_keyframe: bool) -> FloorHypothesis:
        floor_id = f"floor_{self._next_floor_number}"
        self._next_floor_number += 1
        floor = FloorHypothesis(floor_id=floor_id, creation_order=len(self.floors))
        floor.observe(pose_z, frame_idx, is_keyframe, self.config)
        self.floors[floor_id] = floor
        return floor

    def _record_observation(
        self,
        frame_idx: int,
        timestamp: float,
        pose_z: float,
        floor_id: Optional[str],
        status: str,
        confidence: float,
        reason: str,
    ) -> FloorObservation:
        floor_index = None
        if floor_id is not None and floor_id in self.floors:
            floor_index = self._ordered_floor_ids().index(floor_id)
        observation = FloorObservation(
            frame_idx=int(frame_idx),
            timestamp=float(timestamp),
            pose_z=float(pose_z),
            floor_id=floor_id,
            floor_index=None if floor_index is None else int(floor_index),
            status=str(status),
            confidence=float(confidence),
            reason=str(reason),
            is_transition=status in {"transition", "uncertain"},
        )
        self.observations.append(observation)
        return observation

    def _record_event(
        self,
        frame_idx: int,
        event: str,
        floor_id: Optional[str] = None,
        pose_z: Optional[float] = None,
        candidate_z: Optional[float] = None,
        pending_sample_count: int = 0,
        note: Optional[str] = None,
    ) -> None:
        self.events.append(
            FloorEvent(
                frame_idx=int(frame_idx),
                event=str(event),
                floor_id=floor_id,
                pose_z=pose_z,
                candidate_z=candidate_z,
                pending_sample_count=int(pending_sample_count),
                note=note,
            )
        )
