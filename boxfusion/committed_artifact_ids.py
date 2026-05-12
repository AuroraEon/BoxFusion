"""ID normalization helpers for committed/public topology artifacts.

The committed room topology and room world model generally use canonical
``room_*`` ids, while the committed snapshot can still carry numeric room ids
in gateway and vertical-transition records.  This module keeps that join logic
explicit for audit tooling without changing runtime export semantics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


def canonical_room_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return None
        if token.startswith("room_"):
            return token
        if token.lstrip("-").isdigit():
            number = int(token)
            if number < 0:
                return None
            return f"room_{number}"
        return token
    try:
        number = int(value)
    except (TypeError, ValueError):
        return str(value)
    if number < 0:
        return None
    return f"room_{number}"


def canonical_object_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return None
        if token.startswith("obj_"):
            return token
        if token.lstrip("-").isdigit():
            number = int(token)
            if number < 0:
                return None
            return f"obj_{number}"
        return token
    try:
        number = int(value)
    except (TypeError, ValueError):
        return str(value)
    if number < 0:
        return None
    return f"obj_{number}"


@dataclass
class NormalizationWarning:
    context: str
    field: str
    value: Any
    message: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "context": self.context,
            "field": self.field,
            "value": self.value,
            "message": self.message,
        }


@dataclass
class CommittedArtifactIdNormalizer:
    """Normalize snapshot ids into canonical committed topology ids."""

    snapshot: Dict[str, Any]
    raw_to_canonical_room_id: Dict[str, str] = field(default_factory=dict)
    canonical_room_ids: set = field(default_factory=set)
    numeric_room_ids_mapped: set = field(default_factory=set)
    warnings: List[NormalizationWarning] = field(default_factory=list)
    normalized_gateways: List[Dict[str, Any]] = field(default_factory=list)
    normalized_vertical_transitions: List[Dict[str, Any]] = field(default_factory=list)
    normalized_object_room_assignments: List[Dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_snapshot(cls, snapshot: Optional[Dict[str, Any]]) -> "CommittedArtifactIdNormalizer":
        normalizer = cls(snapshot=dict(snapshot or {}))
        normalizer._build_room_maps()
        normalizer.normalized_gateways = normalizer.normalize_gateways()
        normalizer.normalized_vertical_transitions = normalizer.normalize_vertical_transitions()
        normalizer.normalized_object_room_assignments = normalizer.normalize_object_room_assignments()
        return normalizer

    def _add_room_mapping(self, raw: Any, canonical: Optional[str]) -> None:
        if raw is None or canonical is None:
            return
        key = str(raw)
        self.raw_to_canonical_room_id[key] = canonical
        self.raw_to_canonical_room_id[canonical] = canonical
        self.canonical_room_ids.add(canonical)
        if isinstance(raw, int) or (isinstance(raw, str) and raw.strip().lstrip("-").isdigit()):
            self.numeric_room_ids_mapped.add(str(raw))

    def _build_room_maps(self) -> None:
        for idx, room in enumerate(self.snapshot.get("rooms", []) or []):
            if not isinstance(room, dict):
                continue
            canonical = canonical_room_id(room.get("room_id") or room.get("id"))
            if canonical is None:
                self.warnings.append(
                    NormalizationWarning(
                        context=f"snapshot.rooms[{idx}]",
                        field="room_id",
                        value=room.get("room_id"),
                        message="room record does not expose a canonical room id",
                    )
                )
                continue
            self._add_room_mapping(room.get("room_id"), canonical)
            self._add_room_mapping(room.get("id"), canonical)
            self._add_room_mapping(room.get("room_uuid"), canonical)
            self._add_room_mapping(canonical, canonical)

    def normalize_room_id(self, value: Any, context: str, field: str) -> Optional[str]:
        if value is None:
            self.warnings.append(
                NormalizationWarning(
                    context=context,
                    field=field,
                    value=value,
                    message="missing room id",
                )
            )
            return None
        key = str(value)
        if key in self.raw_to_canonical_room_id:
            return self.raw_to_canonical_room_id[key]
        guessed = canonical_room_id(value)
        if guessed in self.canonical_room_ids:
            return guessed
        self.warnings.append(
            NormalizationWarning(
                context=context,
                field=field,
                value=value,
                message="room id could not be resolved against committed snapshot rooms",
            )
        )
        return guessed

    def normalize_gateways(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        for idx, gateway in enumerate(self.snapshot.get("gateways", []) or []):
            if not isinstance(gateway, dict):
                continue
            connects = list(gateway.get("connects") or [])
            normalized = [
                self.normalize_room_id(value, f"snapshot.gateways[{idx}]", f"connects[{pos}]")
                for pos, value in enumerate(connects[:2])
            ]
            while len(normalized) < 2:
                normalized.append(
                    self.normalize_room_id(None, f"snapshot.gateways[{idx}]", f"connects[{len(normalized)}]")
                )
            rows.append(
                {
                    "index": idx,
                    "raw_connects": connects,
                    "room_a": normalized[0],
                    "room_b": normalized[1],
                    "floor_id": gateway.get("floor_id"),
                    "record": dict(gateway),
                }
            )
        return rows

    def normalize_vertical_transitions(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        for idx, transition in enumerate(self.snapshot.get("vertical_transitions", []) or []):
            if not isinstance(transition, dict):
                continue
            room_a = self.normalize_room_id(
                transition.get("from_room_id"),
                f"snapshot.vertical_transitions[{idx}]",
                "from_room_id",
            )
            room_b = self.normalize_room_id(
                transition.get("to_room_id"),
                f"snapshot.vertical_transitions[{idx}]",
                "to_room_id",
            )
            rows.append(
                {
                    "index": idx,
                    "transition_id": transition.get("transition_id") or f"vertical_transition_{idx}",
                    "room_a": room_a,
                    "room_b": room_b,
                    "from_floor_id": transition.get("from_floor_id"),
                    "to_floor_id": transition.get("to_floor_id"),
                    "record": dict(transition),
                }
            )
        return rows

    def normalize_object_room_assignments(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        for idx, obj in enumerate(self.snapshot.get("objects", []) or []):
            if not isinstance(obj, dict):
                continue
            raw_room_id = obj.get("room_id")
            room_id = self.normalize_room_id(raw_room_id, f"snapshot.objects[{idx}]", "room_id")
            rows.append(
                {
                    "index": idx,
                    "object_id": canonical_object_id(obj.get("id")),
                    "raw_room_id": raw_room_id,
                    "room_id": room_id,
                    "record": dict(obj),
                }
            )
        return rows

    def summary(self) -> Dict[str, Any]:
        unresolved_gateway = [
            warning.to_dict()
            for warning in self.warnings
            if warning.context.startswith("snapshot.gateways")
        ]
        unresolved_vertical = [
            warning.to_dict()
            for warning in self.warnings
            if warning.context.startswith("snapshot.vertical_transitions")
        ]
        unresolved_objects = [
            warning.to_dict()
            for warning in self.warnings
            if warning.context.startswith("snapshot.objects")
        ]
        return {
            "total_snapshot_rooms": len(self.snapshot.get("rooms", []) or []),
            "total_numeric_room_ids_mapped": len(self.numeric_room_ids_mapped),
            "total_canonical_room_ids_mapped": len(self.canonical_room_ids),
            "unresolved_gateway_endpoints": len(unresolved_gateway),
            "unresolved_vertical_transition_endpoints": len(unresolved_vertical),
            "unresolved_object_room_assignments": len(unresolved_objects),
            "warnings": [warning.to_dict() for warning in self.warnings],
        }
