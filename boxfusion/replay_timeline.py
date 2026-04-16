from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from boxfusion.artifact_contract import canonical_timeline_row_kind
from boxfusion.floor_artifacts import display_floor_label


def _canonical_room_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return None
        if token.startswith("room_"):
            suffix = token[5:]
            if suffix.lstrip("-").isdigit() and int(suffix) < 0:
                return None
            return token
        if token.lstrip("-").isdigit():
            room_uuid = int(token)
            if room_uuid < 0:
                return None
            return f"room_{room_uuid}"
        return token
    room_uuid = int(value)
    if room_uuid < 0:
        return None
    return f"room_{room_uuid}"


def _canonical_room_from_timeline(value: Any) -> Optional[str]:
    canonical = _canonical_room_id(value)
    if canonical is not None:
        return canonical
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.isdigit():
        return f"room_{int(text)}"
    return None


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def load_replay_observations(timeline_json: Path, query_api: Any) -> List[Dict[str, Any]]:
    rows = json.loads(Path(timeline_json).read_text(encoding="utf-8"))
    observations: List[Dict[str, Any]] = []
    for row in rows:
        row_kind = canonical_timeline_row_kind(dict(row or {}))
        if row_kind is None:
            continue
        room_id = _canonical_room_from_timeline(row.get("current_room_id"))
        room_record = query_api.topology.get_room(room_id) if room_id else {}
        floor_id = room_record.get("floor_id") if room_record else None
        display_floor_id = room_record.get("display_floor_id") if room_record else None
        observations.append(
            {
                "frame_idx": row.get("frame_idx"),
                "timestamp": _round_float(row.get("timestamp")),
                "replay_frame_idx": row.get("replay_frame_idx"),
                "snapshot_idx": row.get("snapshot_idx"),
                "row_kind": row_kind,
                "current_room_id": room_id,
                "current_floor_id": floor_id,
                "current_display_floor_id": display_floor_id,
                "current_floor_label": display_floor_label(floor_id, display_floor_id),
                "vector_map_path": row.get("vector_map_path"),
                "rgb_path": row.get("rgb_path"),
                "raw_row": dict(row),
            }
        )
    return observations
