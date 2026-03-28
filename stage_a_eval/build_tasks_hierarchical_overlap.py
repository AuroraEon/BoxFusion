from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import (
    ACTIVE_SEQUENCE_NAMES,
    DEFAULT_LEGACY_SCENE_OUTPUT_ROOT,
    DEFAULT_SCENE_OUTPUT_ROOT,
    dump_jsonl,
    find_scene_root,
)
from boxfusion.query_api import RoomTopologyQueryAPI


DEFAULT_TASKS_OUT = Path("stage_a_eval/backend_tasks_v0_1.jsonl")
POLICY_SWEEP = ("strict", "balanced", "exploratory")
BENCHMARK_SPLIT = "backend_eval_v0_1"
HOVSG_OVERLAP_SPLIT = "hm3d_active_overlap_v0_1"
SCENE_ROLE_OVERRIDES = {
    "00824": "single_floor_counterexample",
    "00843": "multi_floor_primary",
}


def resolve_legacy_scene_output_root(*, allow_legacy_fallback: bool, legacy_scene_output_root: str) -> Path | None:
    if not allow_legacy_fallback:
        return None
    text = str(legacy_scene_output_root or "").strip()
    if not text:
        return None
    return Path(text)


def room_sort_key(query_api: RoomTopologyQueryAPI, room_id: str) -> Tuple[int, str]:
    room = query_api.topology.get_room(room_id) or {}
    return (int(room.get("display_order", 10**6) or 10**6), str(room_id))


def ordered_room_ids(query_api: RoomTopologyQueryAPI) -> List[str]:
    room_ids = list(query_api.topology.list_room_ids())
    return sorted(room_ids, key=lambda room_id: room_sort_key(query_api, room_id))


def route_probe(
    query_api: RoomTopologyQueryAPI,
    *,
    cross_floor: bool,
    route_policy: str,
) -> Optional[Dict[str, Any]]:
    for start_room_id in ordered_room_ids(query_api):
        start_room = query_api.topology.get_room(start_room_id) or {}
        for goal_room_id in ordered_room_ids(query_api):
            if goal_room_id == start_room_id:
                continue
            goal_room = query_api.topology.get_room(goal_room_id) or {}
            is_cross_floor = start_room.get("floor_id") != goal_room.get("floor_id")
            if bool(is_cross_floor) != bool(cross_floor):
                continue
            result = query_api.query_route(
                start_room_id=start_room_id,
                goal_room_id=goal_room_id,
                route_policy=route_policy,
            )
            route = dict(result.get("route") or {})
            if route.get("found"):
                return {
                    "start_room_id": start_room_id,
                    "goal_room_id": goal_room_id,
                    "goal_floor_id": goal_room.get("floor_id"),
                    "transition_count": len(result.get("explanation", {}).get("floor_switches", [])),
                }
    return None


def object_probe(
    query_api: RoomTopologyQueryAPI,
    *,
    cross_floor: bool,
) -> Optional[Dict[str, Any]]:
    for start_room_id in ordered_room_ids(query_api):
        start_room = query_api.topology.get_room(start_room_id) or {}
        for object_id in sorted(query_api.topology.list_object_ids()):
            record = query_api.topology.get_object(object_id) or {}
            target_room_id = record.get("room_id")
            if target_room_id is None or target_room_id == start_room_id:
                continue
            target_room = query_api.topology.get_room(target_room_id) or {}
            is_cross_floor = start_room.get("floor_id") != target_room.get("floor_id")
            if bool(is_cross_floor) != bool(cross_floor):
                continue
            result = query_api.query_route_to_object(
                start_room_id=start_room_id,
                object_id=object_id,
                route_policy="balanced",
            )
            if dict(result.get("route") or {}).get("found"):
                return {
                    "start_room_id": start_room_id,
                    "object_id": object_id,
                    "object_label": record.get("label"),
                    "target_room_id": target_room_id,
                    "target_floor_id": target_room.get("floor_id"),
                }
    return None


def first_object_probe(query_api: RoomTopologyQueryAPI) -> Optional[Dict[str, Any]]:
    for object_id in sorted(query_api.topology.list_object_ids()):
        record = query_api.topology.get_object(object_id) or {}
        target_room_id = record.get("room_id")
        if target_room_id is None:
            continue
        target_room = query_api.topology.get_room(target_room_id) or {}
        return {
            "object_id": object_id,
            "object_label": record.get("label"),
            "target_room_id": target_room_id,
            "target_floor_id": target_room.get("floor_id"),
        }
    return None


def first_anchor_probe(query_api: RoomTopologyQueryAPI) -> Optional[Dict[str, Any]]:
    for anchor_id in sorted(query_api.topology.list_anchor_ids(valid_only=True)):
        record = query_api.topology.get_anchor(anchor_id) or {}
        target_room_id = record.get("room_id")
        if target_room_id is None:
            continue
        target_room = query_api.topology.get_room(target_room_id) or {}
        return {
            "anchor_id": anchor_id,
            "target_room_id": target_room_id,
            "target_floor_id": target_room.get("floor_id"),
        }
    return None


def anchor_probe(
    query_api: RoomTopologyQueryAPI,
    *,
    cross_floor: bool,
) -> Optional[Dict[str, Any]]:
    for start_room_id in ordered_room_ids(query_api):
        start_room = query_api.topology.get_room(start_room_id) or {}
        for anchor_id in sorted(query_api.topology.list_anchor_ids(valid_only=True)):
            record = query_api.topology.get_anchor(anchor_id) or {}
            target_room_id = record.get("room_id")
            if target_room_id is None or target_room_id == start_room_id:
                continue
            target_room = query_api.topology.get_room(target_room_id) or {}
            is_cross_floor = start_room.get("floor_id") != target_room.get("floor_id")
            if bool(is_cross_floor) != bool(cross_floor):
                continue
            result = query_api.query_route_to_anchor(
                start_room_id=start_room_id,
                anchor_id=anchor_id,
                route_policy="balanced",
            )
            if dict(result.get("route") or {}).get("found"):
                return {
                    "start_room_id": start_room_id,
                    "anchor_id": anchor_id,
                    "target_room_id": target_room_id,
                    "target_floor_id": target_room.get("floor_id"),
                }
    return None


def scene_role(sequence_name: str, query_api: RoomTopologyQueryAPI) -> str:
    short_id = sequence_name.split("-", 1)[0]
    if short_id in SCENE_ROLE_OVERRIDES:
        return SCENE_ROLE_OVERRIDES[short_id]
    floor_ids = {
        str((query_api.topology.get_room(room_id) or {}).get("floor_id"))
        for room_id in query_api.topology.list_room_ids()
        if (query_api.topology.get_room(room_id) or {}).get("floor_id") is not None
    }
    if len(floor_ids) > 1:
        return "active_multifloor_scene"
    return "active_singlefloor_scene"


def make_task_record(
    *,
    sequence_name: str,
    task_id: str,
    task_family: str,
    task_type: str,
    hierarchy_level: str,
    scene_role_label: str,
    policy: str,
    source_room_id: Optional[str],
    target_granularity: str,
    target_room_id: Optional[str],
    target_floor_id: Optional[str],
    target_object_id: Optional[str] = None,
    target_object_label: Optional[str] = None,
    target_anchor_id: Optional[str] = None,
    target_anchor_label: Optional[str] = None,
    needs_route: bool,
    resolve_only: bool,
    expected_floor_sensitive: bool,
    expected_room_sensitive: bool,
    expected_success: bool,
    requires_vertical_transition: bool,
    expected_transition_count: int,
    notes: Optional[List[str]] = None,
) -> Dict[str, Any]:
    scene_id = sequence_name.split("-", 1)[0]
    target_spec: Dict[str, Any] = {"target_type": target_granularity}
    if target_room_id is not None:
        target_spec["goal_room_id"] = target_room_id
    if target_object_id is not None:
        target_spec["object_id"] = target_object_id
    if target_object_label is not None:
        target_spec["object_label"] = target_object_label
    if target_anchor_id is not None:
        target_spec["anchor_id"] = target_anchor_id

    return {
        "task_id": task_id,
        "split": BENCHMARK_SPLIT,
        "scene_id": scene_id,
        "sequence_name": sequence_name,
        "task_family": task_family,
        "task_type": task_type,
        "hierarchy_level": hierarchy_level,
        "source_room_id": source_room_id,
        "target_floor_id": target_floor_id,
        "target_room_id": target_room_id,
        "target_object_id": target_object_id,
        "target_object_label": target_object_label,
        "target_anchor_id": target_anchor_id,
        "target_anchor_label": target_anchor_label,
        "needs_route": bool(needs_route),
        "resolve_only": bool(resolve_only),
        "target_granularity": target_granularity,
        "expected_floor_sensitive": bool(expected_floor_sensitive),
        "expected_room_sensitive": bool(expected_room_sensitive),
        "expected_success": bool(expected_success),
        "policy": policy,
        "hov_sg_overlap_split": HOVSG_OVERLAP_SPLIT,
        "scene_role": scene_role_label,
        "notes": list(notes or []),
        "target_spec": target_spec,
        "route_policy": policy,
        "start_room": source_room_id,
        "expected_target_room": target_room_id,
        "expected_floor_id": target_floor_id,
        "requires_vertical_transition": bool(requires_vertical_transition),
        "expected_transition_count": int(expected_transition_count),
        "expected_outcome": "success" if expected_success else "failure",
        "task_origin": "auto_seed_from_topology_truth",
    }


def add_policy_room_tasks(
    tasks: List[Dict[str, Any]],
    *,
    sequence_name: str,
    task_prefix: str,
    scene_role_label: str,
    probe: Dict[str, Any],
) -> None:
    for route_policy in POLICY_SWEEP:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{task_prefix}_{route_policy}",
                task_family="room-to-room route",
                task_type="query_room_route",
                hierarchy_level="query",
                scene_role_label=scene_role_label,
                policy=route_policy,
                source_room_id=probe["start_room_id"],
                target_granularity="room",
                target_room_id=probe["goal_room_id"],
                target_floor_id=probe["goal_floor_id"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=bool(probe["transition_count"] > 0),
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=bool(probe["transition_count"] > 0),
                expected_transition_count=int(probe["transition_count"]),
                notes=["auto_seed_from_topology_truth"],
            )
        )


def build_scene_tasks(sequence_name: str, scene_root: Path) -> List[Dict[str, Any]]:
    topology_json = scene_root / "logs" / "topology_v0_1.json"
    timeline_json = scene_root / "logs" / "timeline.json"
    if not topology_json.exists():
        return []

    query_api = RoomTopologyQueryAPI.from_json(topology_json)
    tasks: List[Dict[str, Any]] = []
    scene_role_label = scene_role(sequence_name, query_api)

    same_floor_probe = route_probe(query_api, cross_floor=False, route_policy="balanced")
    cross_floor_probe = route_probe(query_api, cross_floor=True, route_policy="balanced")
    same_floor_object = object_probe(query_api, cross_floor=False)
    cross_floor_object = object_probe(query_api, cross_floor=True)
    same_floor_anchor = anchor_probe(query_api, cross_floor=False)
    cross_floor_anchor = anchor_probe(query_api, cross_floor=True)
    resolve_object = first_object_probe(query_api)
    resolve_anchor = first_anchor_probe(query_api)

    if resolve_object is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_resolve_object_seed",
                task_family="resolve-only",
                task_type="resolve_object",
                hierarchy_level="resolve",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=None,
                target_granularity="object",
                target_room_id=resolve_object["target_room_id"],
                target_floor_id=resolve_object["target_floor_id"],
                target_object_id=resolve_object["object_id"],
                target_object_label=resolve_object["object_label"],
                needs_route=False,
                resolve_only=True,
                expected_floor_sensitive=False,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=False,
                expected_transition_count=0,
                notes=["auto_seed_from_topology_truth", "seed_object_resolution"],
            )
        )

    if resolve_anchor is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_resolve_anchor_seed",
                task_family="resolve-only",
                task_type="resolve_anchor",
                hierarchy_level="resolve",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=None,
                target_granularity="anchor",
                target_room_id=resolve_anchor["target_room_id"],
                target_floor_id=resolve_anchor["target_floor_id"],
                target_anchor_id=resolve_anchor["anchor_id"],
                target_anchor_label=resolve_anchor["anchor_id"],
                needs_route=False,
                resolve_only=True,
                expected_floor_sensitive=False,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=False,
                expected_transition_count=0,
                notes=["auto_seed_from_topology_truth", "seed_anchor_resolution"],
            )
        )

    if same_floor_probe is not None:
        add_policy_room_tasks(
            tasks,
            sequence_name=sequence_name,
            task_prefix=f"{sequence_name.split('-', 1)[0]}_query_room_same_floor",
            scene_role_label=scene_role_label,
            probe=same_floor_probe,
        )
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_execute_room_same_floor_balanced",
                task_family="room-to-room route",
                task_type="execute_room_route",
                hierarchy_level="execute",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=same_floor_probe["start_room_id"],
                target_granularity="room",
                target_room_id=same_floor_probe["goal_room_id"],
                target_floor_id=same_floor_probe["goal_floor_id"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=False,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=False,
                expected_transition_count=int(same_floor_probe["transition_count"]),
                notes=[
                    "auto_seed_from_topology_truth",
                    "same_floor_room_execute",
                    "timeline_present:" + str(bool(timeline_json.exists())).lower(),
                ],
            )
        )

    if cross_floor_probe is not None:
        add_policy_room_tasks(
            tasks,
            sequence_name=sequence_name,
            task_prefix=f"{sequence_name.split('-', 1)[0]}_query_room_cross_floor",
            scene_role_label=scene_role_label,
            probe=cross_floor_probe,
        )
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_execute_room_cross_floor_balanced",
                task_family="room-to-room route",
                task_type="execute_room_route",
                hierarchy_level="execute",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=cross_floor_probe["start_room_id"],
                target_granularity="room",
                target_room_id=cross_floor_probe["goal_room_id"],
                target_floor_id=cross_floor_probe["goal_floor_id"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=True,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=True,
                expected_transition_count=int(cross_floor_probe["transition_count"]),
                notes=[
                    "auto_seed_from_topology_truth",
                    "cross_floor_room_execute",
                    "timeline_present:" + str(bool(timeline_json.exists())).lower(),
                ],
            )
        )

    if same_floor_object is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_query_object_same_floor_balanced",
                task_family="route-to-object",
                task_type="query_object_route",
                hierarchy_level="query",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=same_floor_object["start_room_id"],
                target_granularity="object",
                target_room_id=same_floor_object["target_room_id"],
                target_floor_id=same_floor_object["target_floor_id"],
                target_object_id=same_floor_object["object_id"],
                target_object_label=same_floor_object["object_label"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=False,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=False,
                expected_transition_count=0,
                notes=["auto_seed_from_topology_truth", "same_floor_object_query"],
            )
        )

    if cross_floor_object is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_query_object_cross_floor_balanced",
                task_family="route-to-object",
                task_type="query_object_route",
                hierarchy_level="query",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=cross_floor_object["start_room_id"],
                target_granularity="object",
                target_room_id=cross_floor_object["target_room_id"],
                target_floor_id=cross_floor_object["target_floor_id"],
                target_object_id=cross_floor_object["object_id"],
                target_object_label=cross_floor_object["object_label"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=True,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=True,
                expected_transition_count=1,
                notes=["auto_seed_from_topology_truth", "cross_floor_object_query"],
            )
        )

    if same_floor_anchor is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_query_anchor_same_floor_balanced",
                task_family="route-to-anchor",
                task_type="query_anchor_route",
                hierarchy_level="query",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=same_floor_anchor["start_room_id"],
                target_granularity="anchor",
                target_room_id=same_floor_anchor["target_room_id"],
                target_floor_id=same_floor_anchor["target_floor_id"],
                target_anchor_id=same_floor_anchor["anchor_id"],
                target_anchor_label=same_floor_anchor["anchor_id"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=False,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=False,
                expected_transition_count=0,
                notes=["auto_seed_from_topology_truth", "same_floor_anchor_query"],
            )
        )

    if cross_floor_anchor is not None:
        tasks.append(
            make_task_record(
                sequence_name=sequence_name,
                task_id=f"{sequence_name.split('-', 1)[0]}_query_anchor_cross_floor_balanced",
                task_family="route-to-anchor",
                task_type="query_anchor_route",
                hierarchy_level="query",
                scene_role_label=scene_role_label,
                policy="balanced",
                source_room_id=cross_floor_anchor["start_room_id"],
                target_granularity="anchor",
                target_room_id=cross_floor_anchor["target_room_id"],
                target_floor_id=cross_floor_anchor["target_floor_id"],
                target_anchor_id=cross_floor_anchor["anchor_id"],
                target_anchor_label=cross_floor_anchor["anchor_id"],
                needs_route=True,
                resolve_only=False,
                expected_floor_sensitive=True,
                expected_room_sensitive=True,
                expected_success=True,
                requires_vertical_transition=True,
                expected_transition_count=1,
                notes=["auto_seed_from_topology_truth", "cross_floor_anchor_query"],
            )
        )

    return tasks


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build seed hierarchical-overlap tasks from existing scene exports.")
    parser.add_argument(
        "--scene-output-root",
        default=str(DEFAULT_SCENE_OUTPUT_ROOT),
        help="Preferred regenerated scene root.",
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
        help="JSONL output path.",
    )
    parser.add_argument(
        "--sequences",
        nargs="*",
        default=list(ACTIVE_SEQUENCE_NAMES),
        help="Optional subset of full sequence names.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    scene_output_root = Path(args.scene_output_root)
    legacy_scene_output_root = resolve_legacy_scene_output_root(
        allow_legacy_fallback=bool(args.allow_legacy_fallback),
        legacy_scene_output_root=args.legacy_scene_output_root,
    )
    tasks: List[Dict[str, Any]] = []

    for sequence_name in [str(item) for item in args.sequences if str(item).strip()]:
        scene_root = find_scene_root(
            sequence_name,
            preferred_root=scene_output_root,
            fallback_roots=[] if legacy_scene_output_root is None else [legacy_scene_output_root],
        )
        if scene_root is None:
            continue
        tasks.extend(build_scene_tasks(sequence_name, scene_root))

    tasks.sort(key=lambda item: (str(item.get("sequence_name")), str(item.get("task_id"))))
    dump_jsonl(Path(args.tasks_out), tasks)
    print(json.dumps({"tasks_out": str(args.tasks_out), "task_count": len(tasks)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
