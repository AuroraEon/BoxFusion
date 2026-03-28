import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from boxfusion.floor_artifacts import (
    build_floor_lookup,
    canonicalize_floors,
    canonicalize_vertical_transition_record,
    display_floor_label,
)
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopology, RoomTopologyBuilder, _canonical_room_id


DEFAULT_SEQUENCE_DIRS = [
    "world_model_backend_outputs_v0_1/scenes/00843-DYehNKdT76V",
    "world_model_backend_outputs_v0_1/scenes/00824-Dd4bFSTQ8gi",
]


def _load_json(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _maybe_load_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    return _load_json(path)


def _load_final_vector_map(sequence_dir: Path) -> Tuple[Dict[str, Any], Path]:
    timeline = _load_json(sequence_dir / "logs" / "timeline.json")
    last_path: Optional[Path] = None
    for row in timeline:
        candidate = row.get("vector_map_path")
        if not candidate:
            continue
        candidate_path = Path(candidate)
        if candidate_path.exists():
            last_path = candidate_path
    if last_path is None:
        raise FileNotFoundError(f"No vector_map exports found in {sequence_dir / 'logs' / 'timeline.json'}")
    return _load_json(last_path), last_path


def _floor_signature_list(items: Sequence[Dict[str, Any]]) -> List[Tuple[Any, ...]]:
    return [
        (
            item.get("floor_id"),
            item.get("display_floor_id"),
            item.get("display_order"),
            item.get("floor_index"),
        )
        for item in canonicalize_floors(items)
    ]


def _room_counts_by_display_floor(vector_map: Dict[str, Any]) -> Dict[str, int]:
    floor_lookup = build_floor_lookup(vector_map.get("floors", []))
    counts: Dict[str, int] = {}
    for room in vector_map.get("rooms", []):
        floor_label = display_floor_label(
            room.get("floor_id"),
            room.get("display_floor_id"),
            floor_lookup=floor_lookup,
        )
        counts[floor_label] = counts.get(floor_label, 0) + 1
    return counts


def _transition_signature(item: Dict[str, Any], floor_lookup: Dict[str, Dict[str, Any]]) -> Tuple[Any, ...]:
    record = canonicalize_vertical_transition_record(dict(item), floor_lookup)
    return (
        record.get("transition_id"),
        _canonical_room_id(record.get("from_room_id")),
        _canonical_room_id(record.get("to_room_id")),
        record.get("from_floor_id"),
        record.get("to_floor_id"),
        record.get("from_display_floor_id"),
        record.get("to_display_floor_id"),
        record.get("transition_frame_start"),
        record.get("transition_frame_end"),
    )


def _topology_transition_records(topology: RoomTopology) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    seen = set()
    for _, _, data in topology.graph.edges(data=True):
        if data.get("relation_type") != "vertical_transition":
            continue
        for record in (data.get("metadata") or {}).get("transition_records", []):
            key = json.dumps(record, sort_keys=True)
            if key in seen:
                continue
            seen.add(key)
            records.append(dict(record))
    return records


def _compare_floor_catalogs(
    vector_map: Dict[str, Any],
    topology: RoomTopology,
    floor_debug: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    signatures = {
        "vector_map": _floor_signature_list(vector_map.get("floors", [])),
        "topology": _floor_signature_list(list(topology.floor_records.values())),
    }
    if floor_debug is not None:
        signatures["floor_debug"] = _floor_signature_list(floor_debug.get("per_floor", []))
    values = [value for value in signatures.values() if value]
    consistent = bool(values) and all(value == values[0] for value in values[1:])
    return {
        "name": "floor_catalog_consistency",
        "pass": consistent,
        "details": signatures,
    }


def _compare_vertical_transitions(
    vector_map: Dict[str, Any],
    topology: RoomTopology,
    floor_debug: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    floor_lookup = build_floor_lookup(vector_map.get("floors", []))
    signatures = {
        "vector_map": sorted(
            _transition_signature(item, floor_lookup)
            for item in vector_map.get("vertical_transitions", [])
        ),
        "topology": sorted(
            _transition_signature(item, floor_lookup)
            for item in _topology_transition_records(topology)
        ),
    }
    if floor_debug is not None:
        signatures["floor_debug"] = sorted(
            _transition_signature(item, floor_lookup)
            for item in (floor_debug.get("vertical_transition_summary") or {}).get("transitions", [])
        )
    values = [value for value in signatures.values() if value or value == []]
    consistent = bool(values) and all(value == values[0] for value in values[1:])
    return {
        "name": "vertical_transition_consistency",
        "pass": consistent,
        "details": signatures,
    }


def _fallback_signature(diagnostics: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "run_count": int(diagnostics.get("run_count", 0)),
        "fallback_run_count": int(diagnostics.get("fallback_run_count", 0)),
        "fallback_counts": dict(diagnostics.get("fallback_counts", {})),
        "per_floor": {
            str(item.get("floor_id")): {
                "run_count": int(item.get("run_count", 0)),
                "fallback_run_count": int(item.get("fallback_run_count", 0)),
                "fallback_counts": dict(item.get("fallback_counts", {})),
            }
            for item in diagnostics.get("per_floor", [])
            if item.get("floor_id") is not None
        },
    }


def _compare_fallbacks(
    vector_map: Dict[str, Any],
    floor_debug: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    vector_diag = dict(vector_map.get("room_segmentation_diagnostics") or {})
    debug_diag = dict((floor_debug or {}).get("room_segmentation_diagnostics") or {})
    vector_sig = _fallback_signature(vector_diag)
    debug_sig = _fallback_signature(debug_diag) if debug_diag else None
    vector_per_floor_sum = int(sum(item.get("fallback_run_count", 0) for item in vector_diag.get("per_floor", [])))
    consistent = int(vector_diag.get("fallback_run_count", 0)) == vector_per_floor_sum
    if debug_sig is not None:
        consistent = consistent and vector_sig == debug_sig
    return {
        "name": "fallback_summary_consistency",
        "pass": consistent,
        "details": {
            "vector_map": vector_sig,
            "floor_debug": debug_sig,
            "vector_map_per_floor_sum": vector_per_floor_sum,
        },
    }


def _ordered_room_ids(topology: RoomTopology) -> List[str]:
    return sorted(
        topology.list_room_ids(),
        key=lambda room_id: (
            int((topology.get_room(room_id) or {}).get("display_order", 10**6) or 10**6),
            room_id,
        ),
    )


def _pick_start_room(topology: RoomTopology, target_room_id: str) -> List[str]:
    target_room = topology.get_room(target_room_id) or {}
    target_floor = target_room.get("floor_id")
    room_ids = _ordered_room_ids(topology)
    cross_floor = [room_id for room_id in room_ids if room_id != target_room_id and (topology.get_room(room_id) or {}).get("floor_id") != target_floor]
    same_floor = [room_id for room_id in room_ids if room_id != target_room_id and (topology.get_room(room_id) or {}).get("floor_id") == target_floor]
    return cross_floor + same_floor + [target_room_id]


def _route_case(query_api: RoomTopologyQueryAPI, cross_floor: bool) -> Dict[str, Any]:
    topology = query_api.topology
    for start_room_id in _ordered_room_ids(topology):
        start_room = topology.get_room(start_room_id) or {}
        for goal_room_id in _ordered_room_ids(topology):
            if start_room_id == goal_room_id:
                continue
            goal_room = topology.get_room(goal_room_id) or {}
            if cross_floor and start_room.get("floor_id") == goal_room.get("floor_id"):
                continue
            if (not cross_floor) and start_room.get("floor_id") != goal_room.get("floor_id"):
                continue
            result = query_api.query_route(start_room_id, goal_room_id)
            if not result.get("found"):
                continue
            has_vertical = "vertical_transition" in result.get("route", {}).get("used_relation_types", [])
            if cross_floor and has_vertical:
                return {"success": True, "target_id": goal_room_id, "query_result": result}
            if (not cross_floor) and (not has_vertical):
                return {"success": True, "target_id": goal_room_id, "query_result": result}
    return {"success": False, "reason": "no_matching_route_case_found"}


def _target_case(query_api: RoomTopologyQueryAPI, target_type: str) -> Dict[str, Any]:
    topology = query_api.topology
    if target_type == "anchor":
        target_ids = topology.list_anchor_ids(valid_only=True)
        resolve_fn = query_api.resolve_anchor_room
        route_fn = lambda start_room_id, target_id: query_api.query_route_to_anchor(start_room_id, target_id)
    else:
        target_ids = topology.list_object_ids()
        resolve_fn = lambda target_id: query_api.resolve_object_room(object_id=target_id)
        route_fn = lambda start_room_id, target_id: query_api.query_route_to_object(start_room_id, object_id=target_id)
    for target_id in target_ids:
        resolution = resolve_fn(target_id)
        if not resolution.get("resolved"):
            continue
        target_room_id = str(resolution.get("resolved_room_id"))
        for start_room_id in _pick_start_room(topology, target_room_id):
            result = route_fn(start_room_id, target_id)
            if result.get("found"):
                return {"success": True, "target_id": target_id, "query_result": result}
    return {"success": False, "reason": f"no_routable_{target_type}_case_found"}


def _evaluate_case(name: str, query_result: Dict[str, Any], expected_cross_floor: Optional[bool]) -> Dict[str, Any]:
    explanation = dict(query_result.get("explanation") or {})
    route = dict(query_result.get("route") or {})
    target_resolution = dict(query_result.get("target_resolution") or {})
    floor_switches = list(explanation.get("floor_switches", []))
    used_relations = list(route.get("used_relation_types", []))
    found = bool(query_result.get("found"))
    start_room_id = query_result.get("start_room_id")
    resolved_goal_room_id = query_result.get("resolved_goal_room_id")
    explanation_text = " ".join(
        [
            explanation.get("summary", ""),
            explanation.get("route_summary", ""),
            " ".join(explanation.get("hop_summaries", [])),
        ]
    ).lower()
    topology = None
    start_floor_id = None
    goal_floor_id = None
    start_display_floor_id = None
    goal_display_floor_id = None
    if isinstance(query_result.get("_topology"), RoomTopology):
        topology = query_result["_topology"]
    if topology is not None:
        start_room = topology.get_room(start_room_id) or {}
        goal_room = topology.get_room(resolved_goal_room_id) or {}
        start_floor_id = start_room.get("floor_id")
        goal_floor_id = goal_room.get("floor_id")
        start_display_floor_id = start_room.get("display_floor_id")
        goal_display_floor_id = goal_room.get("display_floor_id")
    preserves_floor = target_resolution.get("resolved_floor_id") == goal_floor_id
    explicit_floor_switch = ("floor switch" in explanation_text and "vertical_transition" in explanation_text)
    cross_floor_observed = start_floor_id is not None and goal_floor_id is not None and start_floor_id != goal_floor_id
    if expected_cross_floor is True:
        passed = found and cross_floor_observed and ("vertical_transition" in used_relations) and bool(floor_switches) and explicit_floor_switch
    elif expected_cross_floor is False:
        passed = found and (not cross_floor_observed) and ("vertical_transition" not in used_relations) and (len(floor_switches) == 0)
    else:
        passed = found and preserves_floor and (not cross_floor_observed or explicit_floor_switch)
    return {
        "name": name,
        "pass": passed,
        "found": found,
        "start_room_id": start_room_id,
        "resolved_goal_room_id": resolved_goal_room_id,
        "start_floor_id": start_floor_id,
        "resolved_goal_floor_id": goal_floor_id,
        "start_display_floor_id": start_display_floor_id,
        "resolved_goal_display_floor_id": goal_display_floor_id,
        "target_resolution": target_resolution,
        "route_relations": used_relations,
        "hop_count": int(route.get("hop_count", 0) or 0),
        "floor_switch_count": int(len(floor_switches)),
        "floor_switches": floor_switches,
        "route_summary": explanation.get("route_summary"),
        "hop_summaries": list(explanation.get("hop_summaries", [])),
        "explanation_summary": explanation.get("summary"),
        "preserves_floor": preserves_floor,
    }


def _sequence_result(sequence_dir: Path) -> Dict[str, Any]:
    vector_map, vector_map_path = _load_final_vector_map(sequence_dir)
    floor_debug = _maybe_load_json(sequence_dir / "logs" / "floor_diagnostics_summary.json")
    topology = RoomTopologyBuilder().build_from_stage_a_sequence_dir(sequence_dir, sequence_id=sequence_dir.name)
    query_api = RoomTopologyQueryAPI(topology)

    checks = [
        _compare_floor_catalogs(vector_map, topology, floor_debug),
        _compare_vertical_transitions(vector_map, topology, floor_debug),
        _compare_fallbacks(vector_map, floor_debug),
    ]
    room_counts_by_floor = _room_counts_by_display_floor(vector_map)
    checks.append(
        {
            "name": "non_empty_exported_floors",
            "pass": bool(room_counts_by_floor) and all(count > 0 for count in room_counts_by_floor.values()),
            "details": room_counts_by_floor,
        }
    )

    same_floor_route = _route_case(query_api, cross_floor=False)
    cross_floor_route = _route_case(query_api, cross_floor=True)
    anchor_case = _target_case(query_api, "anchor")
    object_case = _target_case(query_api, "object")

    cases = []
    for name, payload, expected_cross_floor in [
        ("same_floor_route", same_floor_route, False),
        ("cross_floor_route", cross_floor_route, True),
        ("anchor_target", anchor_case, None),
        ("object_target", object_case, None),
    ]:
        if payload.get("success"):
            query_result = dict(payload["query_result"])
            query_result["_topology"] = topology
            case_result = _evaluate_case(name, query_result, expected_cross_floor)
            case_result["target_id"] = payload.get("target_id")
        else:
            case_result = {
                "name": name,
                "pass": False,
                "found": False,
                "target_id": None,
                "reason": payload.get("reason"),
            }
        cases.append(case_result)

    remaining_limitations = [
        "Cross-floor connectivity is still the minimal v0.1 vertical_transition expression, not full stair or landing semantic modeling.",
        "Object duplication and merge quality remain upstream limitations and are not changed by this closure pass.",
    ]

    sequence_pass = all(item.get("pass") for item in checks) and all(item.get("pass") for item in cases)
    return {
        "sequence_id": sequence_dir.name,
        "sequence_dir": str(sequence_dir),
        "vector_map_path": str(vector_map_path),
        "pass": sequence_pass,
        "floor_catalog": canonicalize_floors(vector_map.get("floors", [])),
        "room_counts_by_display_floor": room_counts_by_floor,
        "checks": checks,
        "cases": cases,
        "remaining_limitations": remaining_limitations,
    }


def _write_csv(path: Path, sequence_results: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "sequence_id",
                "case_name",
                "pass",
                "found",
                "target_id",
                "start_room_id",
                "resolved_goal_room_id",
                "start_display_floor_id",
                "resolved_goal_display_floor_id",
                "route_relations",
                "hop_count",
                "floor_switch_count",
                "preserves_floor",
                "route_summary",
            ],
        )
        writer.writeheader()
        for sequence in sequence_results:
            for case in sequence.get("cases", []):
                writer.writerow(
                    {
                        "sequence_id": sequence.get("sequence_id"),
                        "case_name": case.get("name"),
                        "pass": case.get("pass"),
                        "found": case.get("found"),
                        "target_id": case.get("target_id"),
                        "start_room_id": case.get("start_room_id"),
                        "resolved_goal_room_id": case.get("resolved_goal_room_id"),
                        "start_display_floor_id": case.get("start_display_floor_id"),
                        "resolved_goal_display_floor_id": case.get("resolved_goal_display_floor_id"),
                        "route_relations": "|".join(case.get("route_relations", [])),
                        "hop_count": case.get("hop_count"),
                        "floor_switch_count": case.get("floor_switch_count"),
                        "preserves_floor": case.get("preserves_floor"),
                        "route_summary": case.get("route_summary"),
                    }
                )


def _write_markdown(path: Path, sequence_results: Sequence[Dict[str, Any]]) -> None:
    lines = ["# Multi-floor Query API Acceptance", ""]
    for sequence in sequence_results:
        lines.append(f"## {sequence['sequence_id']}")
        lines.append("")
        lines.append(f"- overall: {'PASS' if sequence.get('pass') else 'FAIL'}")
        lines.append(f"- room counts by display floor: {sequence.get('room_counts_by_display_floor')}")
        lines.append(f"- floor catalog: {[(item.get('floor_id'), item.get('display_floor_id')) for item in sequence.get('floor_catalog', [])]}")
        for check in sequence.get("checks", []):
            lines.append(f"- check {check.get('name')}: {'PASS' if check.get('pass') else 'FAIL'}")
        for case in sequence.get("cases", []):
            lines.append(
                f"- case {case.get('name')}: {'PASS' if case.get('pass') else 'FAIL'} | "
                f"start={case.get('start_room_id')} -> goal={case.get('resolved_goal_room_id')} | "
                f"floors={case.get('start_display_floor_id')} -> {case.get('resolved_goal_display_floor_id')} | "
                f"relations={case.get('route_relations')} | switches={case.get('floor_switch_count')}"
            )
            if case.get("route_summary"):
                lines.append(f"  route_summary: {case.get('route_summary')}")
        lines.append("- remaining limitations:")
        for limitation in sequence.get("remaining_limitations", []):
            lines.append(f"  - {limitation}")
        lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run multi-floor Query API acceptance on exported Stage A sequences.")
    parser.add_argument("--sequence-dir", action="append", default=None, help="Stage A sequence directory; can be provided multiple times")
    parser.add_argument(
        "--output-dir",
        default="world_model_backend_outputs_v0_1/eval/backend_eval_v0_1/multifloor_query_acceptance_v0_1",
        help="Directory for combined acceptance artifacts",
    )
    args = parser.parse_args()

    sequence_dirs = [Path(item) for item in (args.sequence_dir or DEFAULT_SEQUENCE_DIRS)]
    sequence_results = [_sequence_result(path) for path in sequence_dirs]
    summary = {
        "overall_pass": all(item.get("pass") for item in sequence_results),
        "sequence_results": sequence_results,
        "pass_fail_summary": {
            item.get("sequence_id"): ("PASS" if item.get("pass") else "FAIL")
            for item in sequence_results
        },
    }

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "multifloor_query_acceptance_v0_1.json"
    csv_path = output_dir / "multifloor_query_acceptance_v0_1.csv"
    md_path = output_dir / "multifloor_query_acceptance_v0_1.md"

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    _write_csv(csv_path, sequence_results)
    _write_markdown(md_path, sequence_results)

    print(f"acceptance_json={json_path}")
    print(f"acceptance_csv={csv_path}")
    print(f"acceptance_md={md_path}")
    for item in sequence_results:
        print(f"{item['sequence_id']}={'PASS' if item.get('pass') else 'FAIL'}")


if __name__ == "__main__":
    main()
