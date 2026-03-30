from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import (  # noqa: E402
    DEFAULT_LEGACY_SCENE_OUTPUT_ROOT,
    DEFAULT_SCENE_OUTPUT_ROOT,
    dump_jsonl,
    find_scene_root,
    load_jsonl,
)
from boxfusion.query_api import RoomTopologyQueryAPI  # noqa: E402
from boxfusion.scene_graph_builder import normalize_open_vocab_label  # noqa: E402
from stage_a_eval.build_tasks_hierarchical_overlap import (  # noqa: E402
    room_sort_key,
    scene_role,
)


DEFAULT_BASE_TASKS_PATH = Path("stage_a_eval/backend_tasks_v0_1.jsonl")
DEFAULT_TASKS_OUT = Path("stage_a_eval/backend_tasks_v0_2_augmented.jsonl")
BENCHMARK_SPLIT = "backend_eval_v0_2_augmented"
TASK_ORIGIN = "paper_supplement_sup01_augmented_probe_v0_2"
TARGET_SEQUENCE_NAMES: Sequence[str] = (
    "00843-DYehNKdT76V",
    "00873-bxsVRursffK",
    "00862-LT9Jq6dN3Ea",
)
OPTIONAL_SEQUENCE_NAMES: Sequence[str] = (
    "00829-QaLdnwvtxbs",
)
ALIAS_PRIORITY = ("sofa", "nightstand", "cabinet", "desk", "trash_can", "table")
PREFERRED_LABELS = {
    "bed",
    "bottle",
    "cabinet/shelf",
    "coffee table",
    "couch",
    "dining-table",
    "nightstand",
    "picture/frame",
    "pillow",
    "sofa",
    "stairs",
    "table",
    "tree",
}
AVOID_LABELS = {
    "air conditioner",
    "ceiling",
    "door",
    "floor",
    "light",
    "power outlet",
    "sky",
    "snow",
    "wall",
    "wall-wood",
    "window",
    "window-other",
}


def resolve_legacy_scene_output_root(*, allow_legacy_fallback: bool, legacy_scene_output_root: str) -> Path | None:
    if not allow_legacy_fallback:
        return None
    text = str(legacy_scene_output_root or "").strip()
    if not text:
        return None
    return Path(text)


def ordered_room_ids(query_api: RoomTopologyQueryAPI) -> List[str]:
    room_ids = list(query_api.topology.list_room_ids())
    return sorted(room_ids, key=lambda room_id: room_sort_key(query_api, room_id))


def label_score(label: str) -> Tuple[int, int, str]:
    lower = str(label or "").strip().lower()
    score = 0
    if lower in PREFERRED_LABELS:
        score += 30
    if lower in AVOID_LABELS:
        score -= 100
    if "/" in lower:
        score += 6
    if "-" in lower:
        score += 4
    if len(lower.split()) <= 2:
        score += 2
    return (-score, len(lower), lower)


def stringify_label(value: Any) -> str:
    return str(value or "").strip()


def build_label_index(query_api: RoomTopologyQueryAPI) -> Dict[str, Dict[str, Any]]:
    by_label: Dict[str, Dict[str, Any]] = {}
    for object_id in sorted(query_api.topology.list_object_ids()):
        record = query_api.topology.get_object(object_id) or {}
        label = stringify_label(record.get("label"))
        room_id = record.get("room_id")
        if not label or not room_id:
            continue
        key = label.lower()
        entry = by_label.setdefault(
            key,
            {
                "label": label,
                "normalized_label": normalize_open_vocab_label(label),
                "room_ids": set(),
                "object_ids": [],
                "count": 0,
            },
        )
        entry["room_ids"].add(room_id)
        entry["object_ids"].append(object_id)
        entry["count"] += 1
    for entry in by_label.values():
        entry["room_ids"] = sorted(entry["room_ids"])
    return by_label


def find_route_probe(
    query_api: RoomTopologyQueryAPI,
    *,
    target_room_id: str,
    prefer_cross_floor: Optional[bool],
) -> Optional[Dict[str, Any]]:
    target_room = query_api.topology.get_room(target_room_id) or {}
    for require_cross_floor in ([prefer_cross_floor] if prefer_cross_floor is not None else [None]):
        for start_room_id in ordered_room_ids(query_api):
            if start_room_id == target_room_id:
                continue
            start_room = query_api.topology.get_room(start_room_id) or {}
            is_cross_floor = start_room.get("floor_id") != target_room.get("floor_id")
            if require_cross_floor is not None and bool(is_cross_floor) != bool(require_cross_floor):
                continue
            result = query_api.query_route(
                start_room_id=start_room_id,
                goal_room_id=target_room_id,
                route_policy="balanced",
            )
            route = dict(result.get("route") or {})
            if route.get("found"):
                return {
                    "start_room_id": start_room_id,
                    "hop_count": route.get("hop_count"),
                    "transition_count": len((result.get("explanation") or {}).get("floor_switches", [])),
                    "is_cross_floor": bool(is_cross_floor),
                }
    if prefer_cross_floor is not None:
        return find_route_probe(query_api, target_room_id=target_room_id, prefer_cross_floor=None)
    return None


def choose_same_room_duplicate(query_api: RoomTopologyQueryAPI, label_index: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    candidates = [
        entry
        for entry in label_index.values()
        if int(entry["count"]) > 1 and len(entry["room_ids"]) == 1
    ]
    candidates = sorted(candidates, key=lambda item: (label_score(item["label"]), -int(item["count"])))
    for entry in candidates:
        room_id = entry["room_ids"][0]
        resolve_result = query_api.resolve_object_room(object_label=entry["label"])
        if not resolve_result.get("resolved"):
            continue
        route_probe = find_route_probe(query_api, target_room_id=room_id, prefer_cross_floor=True)
        if route_probe is None:
            continue
        return {
            "label": entry["label"],
            "query_label": entry["label"],
            "target_room_id": room_id,
            "target_floor_id": (query_api.topology.get_room(room_id) or {}).get("floor_id"),
            "route_probe": route_probe,
            "notes": [
                "duplicate-label referents remain resolvable when all matches stay inside one room",
                f"matched_count={entry['count']}",
            ],
        }
    raise RuntimeError("Could not find a same-room duplicate success probe.")


def choose_alias_probe(query_api: RoomTopologyQueryAPI, label_index: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    candidates: List[Dict[str, Any]] = []
    for entry in label_index.values():
        normalized_label = str(entry.get("normalized_label") or "").strip()
        if not normalized_label:
            continue
        raw_token = entry["label"].lower().replace(" ", "_")
        if normalized_label == raw_token:
            continue
        query_label = normalized_label.replace("_", " ")
        resolve_result = query_api.resolve_object_room(object_label=query_label)
        status = "success" if resolve_result.get("resolved") else str(resolve_result.get("failure_reason"))
        if status not in {"success", "object_label_ambiguous"}:
            continue
        route_probe = None
        target_room_id = resolve_result.get("resolved_room_id")
        target_floor_id = resolve_result.get("resolved_floor_id")
        if target_room_id is not None:
            route_probe = find_route_probe(query_api, target_room_id=target_room_id, prefer_cross_floor=True)
            if route_probe is None:
                continue
        candidates.append(
            {
                "label": entry["label"],
                "query_label": query_label,
                "normalized_label": normalized_label,
                "target_room_id": target_room_id,
                "target_floor_id": target_floor_id,
                "route_probe": route_probe,
                "expected_status": "success" if status == "success" else "ambiguous",
                "candidate_room_ids": sorted((resolve_result.get("ambiguity") or {}).get("candidate_room_ids", [])),
            }
        )

    def alias_sort_key(item: Dict[str, Any]) -> Tuple[int, int, int, str]:
        alias = str(item["query_label"]).replace(" ", "_")
        priority = ALIAS_PRIORITY.index(alias) if alias in ALIAS_PRIORITY else len(ALIAS_PRIORITY)
        success_rank = 0 if item["expected_status"] == "success" else 1
        room_span = len(item.get("candidate_room_ids") or [])
        return (priority, success_rank, room_span, str(item["label"]).lower())

    if not candidates:
        raise RuntimeError("Could not find an alias probe.")
    selected = sorted(candidates, key=alias_sort_key)[0]
    status_note = "alias resolves cleanly through normalized label lookup"
    if selected["expected_status"] != "success":
        status_note = "alias intentionally collides across multiple rooms and should abstain"
    selected["notes"] = [
        status_note,
        f"raw_label={selected['label']}",
        f"canonical_alias={selected['query_label']}",
    ]
    return selected


def choose_ambiguous_duplicate(query_api: RoomTopologyQueryAPI, label_index: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    candidates = [
        entry
        for entry in label_index.values()
        if len(entry["room_ids"]) > 1 and entry["label"].lower() not in AVOID_LABELS
    ]
    candidates = sorted(
        candidates,
        key=lambda item: (len(item["room_ids"]), label_score(item["label"]), -int(item["count"])),
    )
    for entry in candidates:
        resolve_result = query_api.resolve_object_room(object_label=entry["label"])
        if str(resolve_result.get("failure_reason")) != "object_label_ambiguous":
            continue
        return {
            "label": entry["label"],
            "query_label": entry["label"],
            "candidate_room_ids": sorted((resolve_result.get("ambiguity") or {}).get("candidate_room_ids", [])),
            "notes": [
                "duplicate label spans multiple rooms and should trigger backend abstention",
                f"matched_count={entry['count']}",
            ],
        }
    raise RuntimeError("Could not find a cross-room ambiguity probe.")


def near_miss_variants(label: str) -> Iterable[str]:
    label = stringify_label(label)
    if not label:
        return []
    candidates: List[str] = []
    if "/" in label:
        candidates.append(label.replace("/", " "))
    if " " in label:
        candidates.append(label.replace(" ", "/"))
        candidates.append(label.replace(" ", ""))
    if "-" in label:
        candidates.append(label.replace("-", "/"))
    lower = label.lower()
    if lower.endswith("s"):
        candidates.append(label[:-1])
    else:
        candidates.append(label + "s")
    candidates.append(label + " object")
    deduped: List[str] = []
    seen = {label.lower()}
    for item in candidates:
        token = item.lower()
        if token in seen:
            continue
        seen.add(token)
        deduped.append(item)
    return deduped


def choose_near_miss_probe(query_api: RoomTopologyQueryAPI, label_index: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    base_candidates = sorted(label_index.values(), key=lambda item: (label_score(item["label"]), len(item["room_ids"])))
    for entry in base_candidates:
        for query_label in near_miss_variants(entry["label"]):
            resolve_result = query_api.resolve_object_room(object_label=query_label)
            if str(resolve_result.get("failure_reason")) != "object_not_found":
                continue
            return {
                "label": entry["label"],
                "query_label": query_label,
                "notes": [
                    "near-miss distractor should be rejected instead of forced into a wrong room",
                    f"base_label={entry['label']}",
                ],
            }
    raise RuntimeError("Could not find a near-miss negative probe.")


def start_room_for_failure(query_api: RoomTopologyQueryAPI, candidate_room_ids: Sequence[str]) -> Optional[str]:
    candidate_set = {str(item) for item in candidate_room_ids}
    ordered = ordered_room_ids(query_api)
    for start_room_id in ordered:
        if start_room_id not in candidate_set:
            return start_room_id
    return ordered[0] if ordered else None


def make_object_task(
    *,
    sequence_name: str,
    scene_role_label: str,
    task_id: str,
    task_type: str,
    query_label: str,
    source_room_id: Optional[str],
    expected_statuses: Sequence[str],
    target_room_id: Optional[str],
    target_floor_id: Optional[str],
    probe_slice: str,
    probe_case_kind: str,
    notes: Sequence[str],
    candidate_room_ids: Optional[Sequence[str]] = None,
    requires_vertical_transition: bool = False,
    expected_transition_count: int = 0,
) -> Dict[str, Any]:
    expected_success = "success" in {str(item) for item in expected_statuses}
    return {
        "task_id": task_id,
        "split": BENCHMARK_SPLIT,
        "scene_id": sequence_name.split("-", 1)[0],
        "sequence_name": sequence_name,
        "task_family": "resolve-only" if task_type == "resolve_object" else "route-to-object",
        "task_type": task_type,
        "hierarchy_level": "resolve" if task_type == "resolve_object" else "query",
        "source_room_id": source_room_id,
        "target_floor_id": target_floor_id,
        "target_room_id": target_room_id,
        "target_object_id": None,
        "target_object_label": query_label,
        "target_anchor_id": None,
        "target_anchor_label": None,
        "needs_route": bool(task_type != "resolve_object"),
        "resolve_only": bool(task_type == "resolve_object"),
        "target_granularity": "object",
        "expected_floor_sensitive": bool(expected_success and requires_vertical_transition),
        "expected_room_sensitive": bool(expected_success and target_room_id is not None),
        "expected_success": bool(expected_success),
        "expected_statuses": list(expected_statuses),
        "expected_outcome": "success" if expected_success else "failure",
        "policy": "balanced",
        "hov_sg_overlap_split": "hm3d_active_overlap_v0_1",
        "scene_role": scene_role_label,
        "notes": list(notes),
        "target_spec": {
            "target_type": "object",
            "object_label": query_label,
        },
        "route_policy": "balanced",
        "start_room": source_room_id,
        "expected_target_room": target_room_id,
        "expected_floor_id": target_floor_id,
        "requires_vertical_transition": bool(requires_vertical_transition),
        "expected_transition_count": int(expected_transition_count),
        "task_origin": TASK_ORIGIN,
        "probe_slice": probe_slice,
        "probe_case_kind": probe_case_kind,
        "candidate_room_ids": list(candidate_room_ids or []),
    }


def augment_base_tasks(base_tasks: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    augmented: List[Dict[str, Any]] = []
    for task in base_tasks:
        item = dict(task)
        item["split"] = BENCHMARK_SPLIT
        item.setdefault("expected_statuses", ["success"])
        item.setdefault("probe_slice", "positive_seeded")
        item.setdefault("probe_case_kind", "seeded_positive")
        augmented.append(item)
    return augmented


def build_scene_augmented_tasks(sequence_name: str, scene_root: Path) -> List[Dict[str, Any]]:
    query_api = RoomTopologyQueryAPI.from_json(scene_root / "logs" / "topology_v0_1.json")
    label_index = build_label_index(query_api)
    scene_role_label = scene_role(sequence_name, query_api)

    same_room = choose_same_room_duplicate(query_api, label_index)
    alias_probe = choose_alias_probe(query_api, label_index)
    ambiguous = choose_ambiguous_duplicate(query_api, label_index)
    near_miss = choose_near_miss_probe(query_api, label_index)

    tasks: List[Dict[str, Any]] = []
    short_id = sequence_name.split("-", 1)[0]
    same_room_route = same_room["route_probe"]
    tasks.extend(
        [
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_same_room_dup_resolve",
                task_type="resolve_object",
                query_label=same_room["query_label"],
                source_room_id=None,
                expected_statuses=["success"],
                target_room_id=same_room["target_room_id"],
                target_floor_id=same_room["target_floor_id"],
                probe_slice="ambiguity",
                probe_case_kind="duplicate_label_same_room_success",
                notes=same_room["notes"],
            ),
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_same_room_dup_query",
                task_type="query_object_route",
                query_label=same_room["query_label"],
                source_room_id=same_room_route["start_room_id"],
                expected_statuses=["success"],
                target_room_id=same_room["target_room_id"],
                target_floor_id=same_room["target_floor_id"],
                probe_slice="ambiguity",
                probe_case_kind="duplicate_label_same_room_success",
                notes=same_room["notes"] + [f"start_room={same_room_route['start_room_id']}"],
                requires_vertical_transition=bool(same_room_route["transition_count"] > 0),
                expected_transition_count=int(same_room_route["transition_count"]),
            ),
        ]
    )

    alias_route = alias_probe.get("route_probe")
    alias_expected_statuses = [str(alias_probe["expected_status"])]
    alias_target_room = alias_probe.get("target_room_id")
    alias_target_floor = alias_probe.get("target_floor_id")
    alias_start_room = alias_route["start_room_id"] if alias_route else start_room_for_failure(query_api, alias_probe.get("candidate_room_ids", []))
    tasks.extend(
        [
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_alias_resolve",
                task_type="resolve_object",
                query_label=alias_probe["query_label"],
                source_room_id=None,
                expected_statuses=alias_expected_statuses,
                target_room_id=alias_target_room,
                target_floor_id=alias_target_floor,
                probe_slice="ambiguity",
                probe_case_kind="alias_probe",
                notes=alias_probe["notes"],
                candidate_room_ids=alias_probe.get("candidate_room_ids"),
            ),
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_alias_query",
                task_type="query_object_route",
                query_label=alias_probe["query_label"],
                source_room_id=alias_start_room,
                expected_statuses=alias_expected_statuses,
                target_room_id=alias_target_room,
                target_floor_id=alias_target_floor,
                probe_slice="ambiguity",
                probe_case_kind="alias_probe",
                notes=alias_probe["notes"] + ([f"start_room={alias_start_room}"] if alias_start_room else []),
                candidate_room_ids=alias_probe.get("candidate_room_ids"),
                requires_vertical_transition=bool(alias_route and alias_route["transition_count"] > 0),
                expected_transition_count=int(alias_route["transition_count"]) if alias_route else 0,
            ),
        ]
    )

    ambiguous_start = start_room_for_failure(query_api, ambiguous["candidate_room_ids"])
    tasks.extend(
        [
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_ambiguous_resolve",
                task_type="resolve_object",
                query_label=ambiguous["query_label"],
                source_room_id=None,
                expected_statuses=["ambiguous"],
                target_room_id=None,
                target_floor_id=None,
                probe_slice="ambiguity",
                probe_case_kind="duplicate_label_cross_room_abstain",
                notes=ambiguous["notes"],
                candidate_room_ids=ambiguous["candidate_room_ids"],
            ),
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_ambiguous_query",
                task_type="query_object_route",
                query_label=ambiguous["query_label"],
                source_room_id=ambiguous_start,
                expected_statuses=["ambiguous"],
                target_room_id=None,
                target_floor_id=None,
                probe_slice="ambiguity",
                probe_case_kind="duplicate_label_cross_room_abstain",
                notes=ambiguous["notes"] + ([f"start_room={ambiguous_start}"] if ambiguous_start else []),
                candidate_room_ids=ambiguous["candidate_room_ids"],
            ),
        ]
    )

    near_miss_start = ordered_room_ids(query_api)[0] if ordered_room_ids(query_api) else None
    tasks.extend(
        [
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_near_miss_resolve",
                task_type="resolve_object",
                query_label=near_miss["query_label"],
                source_room_id=None,
                expected_statuses=["not_found"],
                target_room_id=None,
                target_floor_id=None,
                probe_slice="hard_negative",
                probe_case_kind="near_miss_not_found",
                notes=near_miss["notes"],
            ),
            make_object_task(
                sequence_name=sequence_name,
                scene_role_label=scene_role_label,
                task_id=f"{short_id}_sup01_near_miss_query",
                task_type="query_object_route",
                query_label=near_miss["query_label"],
                source_room_id=near_miss_start,
                expected_statuses=["not_found"],
                target_room_id=None,
                target_floor_id=None,
                probe_slice="hard_negative",
                probe_case_kind="near_miss_not_found",
                notes=near_miss["notes"] + ([f"start_room={near_miss_start}"] if near_miss_start else []),
            ),
        ]
    )
    return tasks


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build the SUP-01 augmented backend task pack.")
    parser.add_argument(
        "--base-tasks",
        default=str(DEFAULT_BASE_TASKS_PATH),
        help="Existing canonical v0.1 task JSONL.",
    )
    parser.add_argument(
        "--scene-output-root",
        default=str(DEFAULT_SCENE_OUTPUT_ROOT),
        help="Preferred canonical scene root.",
    )
    parser.add_argument(
        "--legacy-scene-output-root",
        default=str(DEFAULT_LEGACY_SCENE_OUTPUT_ROOT),
        help="Migration-only read-only fallback root. Ignored unless --allow-legacy-fallback is set.",
    )
    parser.add_argument(
        "--allow-legacy-fallback",
        action="store_true",
        help="Allow a migration-only read-only fallback scan of the legacy root.",
    )
    parser.add_argument(
        "--tasks-out",
        default=str(DEFAULT_TASKS_OUT),
        help="Augmented JSONL output path.",
    )
    parser.add_argument(
        "--include-optional-single-floor",
        action="store_true",
        help="Also add the clean single-floor comparison slice on 00829.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    base_tasks = load_jsonl(Path(args.base_tasks))
    tasks = augment_base_tasks(base_tasks)
    legacy_scene_output_root = resolve_legacy_scene_output_root(
        allow_legacy_fallback=bool(args.allow_legacy_fallback),
        legacy_scene_output_root=args.legacy_scene_output_root,
    )

    selected_sequences = list(TARGET_SEQUENCE_NAMES)
    if bool(args.include_optional_single_floor):
        selected_sequences.extend(OPTIONAL_SEQUENCE_NAMES)

    for sequence_name in selected_sequences:
        scene_root = find_scene_root(
            sequence_name,
            preferred_root=Path(args.scene_output_root),
            fallback_roots=[] if legacy_scene_output_root is None else [legacy_scene_output_root],
        )
        if scene_root is None:
            raise FileNotFoundError(f"Scene root not found for {sequence_name}")
        tasks.extend(build_scene_augmented_tasks(sequence_name, scene_root))

    tasks.sort(key=lambda item: (str(item.get("sequence_name")), str(item.get("task_id"))))
    dump_jsonl(Path(args.tasks_out), tasks)

    summary = defaultdict(int)
    per_scene = defaultdict(int)
    for task in tasks:
        summary[str(task.get("probe_slice", "unknown"))] += 1
        per_scene[str(task.get("sequence_name"))] += 1
    print(
        json.dumps(
            {
                "tasks_out": str(args.tasks_out),
                "task_count": len(tasks),
                "probe_slice_counts": dict(sorted(summary.items())),
                "per_scene_counts": dict(sorted(per_scene.items())),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
