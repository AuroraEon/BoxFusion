from __future__ import annotations

from collections import Counter
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _int_or_none(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _float_or_none(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _room_display_label(room_id: Any) -> Optional[str]:
    if room_id is None:
        return None
    if isinstance(room_id, str):
        token = room_id.strip()
        if token.startswith("room_") and token.split("room_", 1)[-1].isdigit():
            return f"R{int(token.split('room_', 1)[-1])}"
        return token or None
    try:
        return f"R{int(room_id)}"
    except (TypeError, ValueError):
        return str(room_id)


def floor_sort_key(item: Dict[str, Any]) -> Tuple[Any, ...]:
    floor_id = str(item.get("floor_id", ""))
    z_center = _float_or_none(item.get("z_center"))
    creation_order = _int_or_none(item.get("creation_order"))
    floor_index = _int_or_none(item.get("floor_index"))
    display_order = _int_or_none(item.get("display_order"))
    if z_center is not None:
        return (0, z_center, creation_order if creation_order is not None else 10**6, floor_id)
    if floor_index is not None:
        return (1, floor_index, display_order if display_order is not None else 10**6, floor_id)
    if display_order is not None:
        return (2, display_order, floor_id)
    return (3, floor_id)


def canonicalize_floors(floors: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    canonical: List[Dict[str, Any]] = []
    ordered = [
        dict(item)
        for item in sorted(
            [dict(item) for item in floors if item.get("floor_id") is not None],
            key=floor_sort_key,
        )
    ]
    for floor_index, item in enumerate(ordered):
        floor_id = str(item.get("floor_id"))
        display_order = floor_index + 1
        payload = dict(item)
        payload["floor_id"] = floor_id
        payload["floor_index"] = int(floor_index)
        payload["display_order"] = int(display_order)
        payload["display_floor_id"] = f"floor_{display_order}"
        if "z_min" in payload:
            payload["z_min"] = _round_float(payload.get("z_min"))
        if "z_max" in payload:
            payload["z_max"] = _round_float(payload.get("z_max"))
        if "z_center" in payload:
            payload["z_center"] = _round_float(payload.get("z_center"))
        canonical.append(payload)
    return canonical


def build_floor_lookup(floors: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {
        str(item.get("floor_id")): dict(item)
        for item in canonicalize_floors(floors)
        if item.get("floor_id") is not None
    }


def attach_floor_metadata(
    payload: Dict[str, Any],
    floor_lookup: Dict[str, Dict[str, Any]],
    *,
    floor_id_key: str = "floor_id",
    floor_index_key: str = "floor_index",
    display_floor_id_key: str = "display_floor_id",
    display_order_key: str = "display_order",
) -> Dict[str, Any]:
    result = dict(payload)
    floor_id = _clean_optional_text(result.get(floor_id_key))
    if floor_id is None:
        result[floor_index_key] = result.get(floor_index_key)
        result[display_floor_id_key] = _clean_optional_text(result.get(display_floor_id_key))
        result[display_order_key] = result.get(display_order_key)
        return result
    floor_meta = dict(floor_lookup.get(floor_id) or {})
    result[floor_id_key] = floor_id
    result[floor_index_key] = floor_meta.get(floor_index_key, result.get(floor_index_key))
    result[display_floor_id_key] = floor_meta.get(
        display_floor_id_key,
        _clean_optional_text(result.get(display_floor_id_key)) or floor_id,
    )
    result[display_order_key] = floor_meta.get(display_order_key, result.get(display_order_key))
    return result


def display_floor_label(
    floor_id: Optional[str],
    display_floor_id: Optional[str] = None,
    *,
    floor_lookup: Optional[Dict[str, Dict[str, Any]]] = None,
    include_internal: bool = False,
) -> str:
    canonical_floor_id = _clean_optional_text(floor_id)
    canonical_display = _clean_optional_text(display_floor_id)
    if canonical_floor_id is not None and floor_lookup is not None and canonical_floor_id in floor_lookup:
        canonical_display = _clean_optional_text(
            floor_lookup[canonical_floor_id].get("display_floor_id")
        ) or canonical_floor_id
    if canonical_floor_id is None and canonical_display is None:
        return "floor_unknown"
    if include_internal and canonical_display and canonical_floor_id and canonical_display != canonical_floor_id:
        return f"{canonical_display} (internal {canonical_floor_id})"
    return str(canonical_display or canonical_floor_id)


def canonicalize_vertical_transition_record(
    item: Dict[str, Any],
    floor_lookup: Dict[str, Dict[str, Any]],
    *,
    transition_index: Optional[int] = None,
) -> Dict[str, Any]:
    payload = dict(item)
    if payload.get("transition_id") is None and transition_index is not None:
        payload["transition_id"] = f"vt_{int(transition_index)}"

    from_floor_id = _clean_optional_text(
        payload.get("from_floor_id")
        or dict(payload.get("from_room_support") or {}).get("floor_id")
    )
    to_floor_id = _clean_optional_text(
        payload.get("to_floor_id")
        or dict(payload.get("to_room_support") or {}).get("floor_id")
    )
    payload["from_floor_id"] = from_floor_id
    payload["to_floor_id"] = to_floor_id

    from_floor_meta = dict(floor_lookup.get(from_floor_id or "") or {})
    to_floor_meta = dict(floor_lookup.get(to_floor_id or "") or {})
    payload["from_floor_index"] = from_floor_meta.get("floor_index")
    payload["to_floor_index"] = to_floor_meta.get("floor_index")
    payload["from_display_order"] = from_floor_meta.get("display_order")
    payload["to_display_order"] = to_floor_meta.get("display_order")
    payload["from_display_floor_id"] = from_floor_meta.get(
        "display_floor_id",
        _clean_optional_text(payload.get("from_display_floor_id")) or from_floor_id,
    )
    payload["to_display_floor_id"] = to_floor_meta.get(
        "display_floor_id",
        _clean_optional_text(payload.get("to_display_floor_id")) or to_floor_id,
    )

    from_room_label = payload.get("from_room_label") or _room_display_label(payload.get("from_room_id"))
    to_room_label = payload.get("to_room_label") or _room_display_label(payload.get("to_room_id"))
    payload["from_room_label"] = from_room_label
    payload["to_room_label"] = to_room_label

    payload["transition_frame_start"] = payload.get(
        "transition_frame_start",
        payload.get("frame_start"),
    )
    payload["transition_frame_end"] = payload.get(
        "transition_frame_end",
        payload.get("frame_end"),
    )
    payload["display_floor_pair"] = (
        None
        if payload.get("from_display_floor_id") is None or payload.get("to_display_floor_id") is None
        else f"{payload.get('from_display_floor_id')}->{payload.get('to_display_floor_id')}"
    )
    connector_label = str(payload.get("connector_label", "unknown_vertical_connector"))
    z_span_m = _round_float(payload.get("z_span_m"))
    payload["z_span_m"] = z_span_m
    payload["evidence_summary"] = (
        f"{connector_label}: "
        f"{payload.get('from_display_floor_id') or '?'}:{from_room_label or '?'} -> "
        f"{payload.get('to_display_floor_id') or '?'}:{to_room_label or '?'} "
        f"(frames {payload.get('transition_frame_start')}-{payload.get('transition_frame_end')}, "
        f"z_span={z_span_m}m)"
    )
    return payload


def canonicalize_vertical_transitions(
    transitions: Sequence[Dict[str, Any]],
    floor_lookup: Dict[str, Dict[str, Any]],
) -> List[Dict[str, Any]]:
    canonical = [
        canonicalize_vertical_transition_record(
            dict(item),
            floor_lookup,
            transition_index=index,
        )
        for index, item in enumerate(transitions, start=1)
    ]
    canonical.sort(
        key=lambda item: (
            int(item.get("transition_frame_start", 10**9) or 10**9),
            str(item.get("transition_id")),
        )
    )
    return canonical


def build_vertical_transition_summary(
    transitions: Sequence[Dict[str, Any]],
    floor_lookup: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    canonical = canonicalize_vertical_transitions(transitions, floor_lookup)
    connector_label_counts = Counter(
        str(item.get("connector_label", "unknown_vertical_connector"))
        for item in canonical
    )
    edge_eligible_count = int(sum(1 for item in canonical if bool(item.get("edge_eligible", False))))
    partial_count = int(
        sum(
            1
            for item in canonical
            if not bool(item.get("edge_eligible", False))
            and (item.get("from_room_id") is not None or item.get("to_room_id") is not None)
        )
    )
    display_floor_pairs = Counter(
        str(item["display_floor_pair"])
        for item in canonical
        if item.get("display_floor_pair") is not None
    )
    return {
        "count": int(len(canonical)),
        "edge_eligible_count": edge_eligible_count,
        "partial_room_association_count": partial_count,
        "connector_label_counts": dict(connector_label_counts),
        "display_floor_pair_counts": dict(display_floor_pairs),
        "transitions": canonical,
    }


def build_fallback_summary(
    runs: Sequence[Dict[str, Any]],
    floors: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    canonical_floors = canonicalize_floors(floors)
    floor_lookup = build_floor_lookup(canonical_floors)
    normalized_runs: List[Dict[str, Any]] = []
    per_floor_runs: Dict[str, List[Dict[str, Any]]] = {
        str(item["floor_id"]): []
        for item in canonical_floors
        if item.get("floor_id") is not None
    }
    sequence_slice_mode_counts: Counter[str] = Counter()
    sequence_fallback_counts: Counter[str] = Counter()
    sequence_failure_reason_counts: Counter[str] = Counter()

    for item in runs:
        record = attach_floor_metadata(dict(item), floor_lookup)
        record["slice_mode"] = str(record.get("slice_mode", "unknown"))
        fallback_modes = [str(mode) for mode in record.get("fallback_modes", [])]
        fallback_used = bool(record.get("fallback_used")) or bool(fallback_modes)
        failure_reason = _clean_optional_text(record.get("failure_reason"))
        record["fallback_modes"] = fallback_modes
        record["fallback_used"] = fallback_used
        record["failure_reason"] = failure_reason
        normalized_runs.append(record)
        floor_id = _clean_optional_text(record.get("floor_id"))
        if floor_id is not None:
            per_floor_runs.setdefault(floor_id, []).append(record)
        sequence_slice_mode_counts[record["slice_mode"]] += 1
        for mode in fallback_modes:
            sequence_fallback_counts[mode] += 1
        if failure_reason:
            sequence_failure_reason_counts[failure_reason] += 1

    normalized_runs.sort(
        key=lambda item: (
            int(item.get("frame_idx", 10**9) or 10**9),
            int(item.get("display_order", 10**6) or 10**6),
            str(item.get("floor_id", "")),
        )
    )

    per_floor: List[Dict[str, Any]] = []
    for floor in canonical_floors:
        floor_id = str(floor.get("floor_id"))
        floor_runs = sorted(
            per_floor_runs.get(floor_id, []),
            key=lambda item: (
                int(item.get("frame_idx", 10**9) or 10**9),
                str(item.get("trigger_reason", "")),
            ),
        )
        floor_slice_mode_counts = Counter(str(item.get("slice_mode", "unknown")) for item in floor_runs)
        floor_fallback_counts = Counter(
            mode
            for item in floor_runs
            for mode in item.get("fallback_modes", [])
        )
        floor_failure_reason_counts = Counter(
            str(item["failure_reason"])
            for item in floor_runs
            if item.get("failure_reason") is not None
        )
        per_floor.append(
            {
                "floor_id": floor_id,
                "floor_index": floor.get("floor_index"),
                "display_floor_id": floor.get("display_floor_id"),
                "display_order": floor.get("display_order"),
                "run_count": int(len(floor_runs)),
                "successful_run_count": int(sum(1 for item in floor_runs if bool(item.get("success")))),
                "fallback_run_count": int(sum(1 for item in floor_runs if bool(item.get("fallback_used")))),
                "slice_mode_counts": dict(floor_slice_mode_counts),
                "fallback_counts": dict(floor_fallback_counts),
                "failure_reason_counts": dict(floor_failure_reason_counts),
                "runs": floor_runs,
            }
        )

    return {
        "run_count": int(len(normalized_runs)),
        "successful_run_count": int(sum(1 for item in normalized_runs if bool(item.get("success")))),
        "fallback_run_count": int(sum(item["fallback_run_count"] for item in per_floor)),
        "slice_mode_counts": dict(sequence_slice_mode_counts),
        "fallback_counts": dict(sequence_fallback_counts),
        "failure_reason_counts": dict(sequence_failure_reason_counts),
        "accounting_basis": {
            "fallback_run_count": "sum(per_floor fallback_run_count); each run counts once when fallback_used is true or fallback_modes is non-empty",
        },
        "per_floor": per_floor,
        "runs": normalized_runs,
    }
