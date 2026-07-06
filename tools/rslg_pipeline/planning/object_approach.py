"""Current RSLG-SLAM object approach ownership and validation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping

from ..project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_CLEARANCE_M,
    CURRENT_OBJECT_APPROACH_ID,
    CURRENT_OBJECT_APPROACH_POSITION,
    CURRENT_OBJECT_APPROACH_YAW,
    OBJECT_FLOOR,
    OBJECT_ID,
    OBJECT_LABEL,
    OBJECT_QUERY,
    OBJECT_ROOM,
)


def angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_error(yaw: float | None, start_xy: Iterable[float], target_xy: Iterable[float]) -> float | None:
    if yaw is None:
        return None
    sx, sy = [float(v) for v in start_xy][:2]
    tx, ty = [float(v) for v in target_xy][:2]
    desired = math.atan2(ty - sy, tx - sx)
    return abs(angle_wrap(float(yaw) - desired))


@dataclass(frozen=True)
class ObjectApproachGoal:
    candidate_id: str
    position: tuple[float, float]
    yaw: float
    clearance_m: float
    object_id: str = OBJECT_ID
    object_label: str = OBJECT_LABEL
    query: str = OBJECT_QUERY
    floor: str = OBJECT_FLOOR
    room: str = OBJECT_ROOM

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "approach_candidate_id": self.candidate_id,
            "world_xy": list(self.position),
            "position": list(self.position),
            "yaw": self.yaw,
            "clearance_m": self.clearance_m,
            "object_id": self.object_id,
            "object_label": self.object_label,
            "query": self.query,
            "floor": self.floor,
            "room": self.room,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
            "is_object_centroid": False,
        }


def current_object_approach_goal() -> ObjectApproachGoal:
    return ObjectApproachGoal(
        candidate_id=CURRENT_OBJECT_APPROACH_ID,
        position=tuple(float(v) for v in CURRENT_OBJECT_APPROACH_POSITION),
        yaw=float(CURRENT_OBJECT_APPROACH_YAW),
        clearance_m=float(CURRENT_OBJECT_APPROACH_CLEARANCE_M),
    )


def blocked_legacy_approach_ids() -> tuple[str, ...]:
    return tuple(BLOCKED_LEGACY_APPROACH_IDS)


def candidate_id(record: Mapping[str, Any]) -> str | None:
    value = record.get("candidate_id", record.get("approach_candidate_id", record.get("approach_candidate")))
    return str(value) if value is not None else None


def is_current_goal(record: Mapping[str, Any]) -> bool:
    return candidate_id(record) == CURRENT_OBJECT_APPROACH_ID


def is_blocked_legacy_goal(record: Mapping[str, Any]) -> bool:
    cid = candidate_id(record)
    return cid in BLOCKED_LEGACY_APPROACH_IDS if cid is not None else False


def select_current_object_approach(candidates: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    for record in candidates:
        if is_current_goal(record):
            return dict(record)
    return current_object_approach_goal().to_dict()


def validate_current_object_approach(record: Mapping[str, Any]) -> dict[str, Any]:
    cid = candidate_id(record)
    position = record.get("world_xy", record.get("position"))
    errors: list[str] = []
    if cid != CURRENT_OBJECT_APPROACH_ID:
        errors.append(f"candidate id must be {CURRENT_OBJECT_APPROACH_ID!r}, got {cid!r}")
    if cid in BLOCKED_LEGACY_APPROACH_IDS:
        errors.append(f"{cid!r} is blocked legacy evidence only and must not be used as a runtime goal")
    if position is not None:
        xy = [float(v) for v in position][:2]
        expected = list(CURRENT_OBJECT_APPROACH_POSITION)
        if any(abs(xy[i] - expected[i]) > 1e-5 for i in range(2)):
            errors.append(f"position must match {expected!r}, got {xy!r}")
    if "yaw" in record and abs(float(record["yaw"]) - CURRENT_OBJECT_APPROACH_YAW) > 1e-5:
        errors.append(f"yaw must be {CURRENT_OBJECT_APPROACH_YAW!r}, got {record['yaw']!r}")
    if record.get("object_centroid_navigation_used") is True or record.get("direct_object_centroid_goal_used") is True:
        errors.append("object centroid navigation must be false")
    return {
        "ok": not errors,
        "candidate_id": cid,
        "expected_candidate_id": CURRENT_OBJECT_APPROACH_ID,
        "blocked_legacy_approach_ids": list(BLOCKED_LEGACY_APPROACH_IDS),
        "errors": errors,
    }

