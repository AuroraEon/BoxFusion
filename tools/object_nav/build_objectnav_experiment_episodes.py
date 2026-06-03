#!/usr/bin/env python3
"""Generate artifact-derived object-nav query episodes."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path

from object_nav_common import TASK_DIR, load_index, route_eligible, utc_now, write_json


def episode_reason(obj: dict, label_counts: Counter) -> tuple[bool, bool, list[str]]:
    reasons: list[str] = []
    runtime = route_eligible(obj)
    if obj.get("is_noisy_label"):
        reasons.append("noisy label; keep for audit but avoid runtime")
    if obj.get("is_uncertain_floor_assignment"):
        reasons.append("uncertain floor assignment")
    if obj.get("binding_status") != "explicit_valid":
        reasons.append(f"binding status {obj.get('binding_status')}")
    if obj.get("floor_id") != "floor_2":
        reasons.append("not on floor_2 runtime map")
    if obj.get("route_availability_status") != "floor2_runtime_candidate":
        reasons.append(f"route availability {obj.get('route_availability_status')}")
    if label_counts[obj.get("normalized_label")] > 1:
        reasons.append("duplicate/ambiguous label")
    if not reasons:
        reasons.append("stable floor_2 target-room candidate")
    return runtime, runtime and not obj.get("is_uncertain_floor_assignment"), reasons


def make_episode(ep_id: str, query_text: str, expected_ids: list[str], label: str, room_id: str | None, floor_id: str | None, runtime: bool, recommended: bool, reasons: list[str]) -> dict:
    return {
        "episode_id": ep_id,
        "query_text": query_text,
        "expected_object_ids": expected_ids,
        "expected_label": label,
        "expected_room_id": room_id,
        "expected_floor_id": floor_id,
        "evaluation_mode": "artifact_self_consistency",
        "runtime_candidate": runtime,
        "recommended_for_runtime": recommended,
        "reason": "; ".join(dict.fromkeys(reasons)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    parser.add_argument("--output", type=Path, default=TASK_DIR / "objectnav_experiment_episodes_v0_1.json")
    args = parser.parse_args()
    index = load_index(args.index)
    objects = index.get("objects", [])
    label_counts = Counter(o.get("normalized_label") for o in objects)
    by_label = defaultdict(list)
    for obj in objects:
        by_label[obj.get("normalized_label")].append(obj)
    episodes: list[dict] = []
    seen_object_only = set()
    for obj in objects:
        label = obj.get("label") or obj.get("normalized_label")
        normalized = obj.get("normalized_label")
        runtime, recommended, reasons = episode_reason(obj, label_counts)
        if normalized not in seen_object_only:
            seen_object_only.add(normalized)
            expected = [o["object_id"] for o in by_label[normalized]]
            label_runtime = any(route_eligible(o) for o in by_label[normalized])
            episodes.append(make_episode(f"ep_object_{normalized}", label, expected, normalized, None, None, label_runtime, False, ["object-only query; may be ambiguous"] + reasons))
        oid = obj["object_id"]
        if obj.get("room_id"):
            episodes.append(make_episode(f"ep_object_room_{oid}", f"{label} in {obj['room_id']}", [oid], normalized, obj.get("room_id"), None, runtime, recommended, reasons))
        if obj.get("floor_id"):
            floor_expected = [o["object_id"] for o in by_label[normalized] if o.get("floor_id") == obj.get("floor_id")]
            episodes.append(make_episode(f"ep_object_floor_{oid}", f"{label} on {obj['floor_id']}", floor_expected, normalized, None, obj.get("floor_id"), runtime, False, ["floor-constrained retrieval"] + reasons))
        if obj.get("room_id") and obj.get("floor_id"):
            episodes.append(make_episode(f"ep_object_room_floor_{oid}", f"{label} in {obj['room_id']} on {obj['floor_id']}", [oid], normalized, obj.get("room_id"), obj.get("floor_id"), runtime, recommended, reasons))
    payload = {
        "version": "v0_1",
        "artifact_type": "objectnav_experiment_episodes",
        "scene_id": index.get("scene_id"),
        "created_utc": utc_now(),
        "source_index": str(args.index),
        "episode_count": len(episodes),
        "episodes": episodes,
        "notes": [
            "Episodes use artifact-derived expectations for self-consistency only.",
            "recommended_for_runtime means target-room-first runtime may be attempted later; it does not validate object approach pose.",
        ],
    }
    write_json(args.output, payload)
    print(f"wrote {args.output}")
    print(f"episode_count={len(episodes)}")


if __name__ == "__main__":
    main()
