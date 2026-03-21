import argparse
import json
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import _canonical_room_id


RouteFn = Callable[[str, Any, str], Dict[str, Any]]
ResolveFn = Callable[[Any], Dict[str, Any]]


def build_topology_report(
    topology_json: Path,
    sample_limit: int = 5,
) -> Dict[str, Any]:
    query_api = RoomTopologyQueryAPI.from_json(Path(topology_json))
    topology = query_api.topology
    inspection = topology.inspect_export(sample_limit=sample_limit)

    diagnostics: List[str] = []
    if int(inspection["room_count"]) == 0:
        diagnostics.append("no rooms present in this export")
    if int(inspection["anchor_count"]) == 0:
        diagnostics.append("no anchors present in this export")
    if int(inspection["object_count"]) == 0:
        diagnostics.append("no objects present in this export")
    elif int(inspection["object_label_count"]) == 0:
        diagnostics.append("object label lookup unavailable")
    if int(inspection["object_count"]) > 0 and not inspection["capabilities"]["object_metadata_present"]:
        diagnostics.append("object metadata missing")
    if int(inspection["anchor_count"]) > 0 and not inspection["capabilities"]["anchor_metadata_present"]:
        diagnostics.append("anchor metadata missing")

    return {
        "topology_json": str(Path(topology_json)),
        "sequence_id": topology.sequence_id,
        "topology_version": topology.version,
        "inspection": dict(inspection, diagnostics=diagnostics),
    }


def build_acceptance_report(
    topology_json: Path,
    start_room_id: Optional[str] = None,
    sample_limit: int = 5,
    anchor_id: Optional[str] = None,
    object_id: Optional[str] = None,
    object_label: Optional[str] = None,
    compare_policies: bool = False,
) -> Dict[str, Any]:
    topology_json = Path(topology_json)
    query_api = RoomTopologyQueryAPI.from_json(topology_json)
    topology = query_api.topology
    report = build_topology_report(topology_json, sample_limit=sample_limit)
    room_ids = topology.list_room_ids()

    acceptance = {
        "start_room_id": _canonical_room_id(start_room_id) if start_room_id else None,
        "route_to_anchor": _exercise_anchor_acceptance(
            query_api,
            room_ids,
            preferred_start_room_id=start_room_id,
            anchor_id=anchor_id,
            compare_policies=compare_policies,
        ),
        "route_to_object_id": _exercise_object_id_acceptance(
            query_api,
            room_ids,
            preferred_start_room_id=start_room_id,
            object_id=object_id,
            compare_policies=compare_policies,
        ),
        "route_to_object_label": _exercise_object_label_acceptance(
            query_api,
            room_ids,
            preferred_start_room_id=start_room_id,
            object_label=object_label,
            compare_policies=compare_policies,
        ),
    }

    acceptance["summary"] = {
        "route_to_anchor_succeeded": bool(acceptance["route_to_anchor"]["success"]),
        "route_to_object_id_succeeded": bool(acceptance["route_to_object_id"]["success"]),
        "route_to_object_label_succeeded": bool(acceptance["route_to_object_label"]["success"]),
        "failures": [
            f"{key}: {value['failure_reason']}"
            for key, value in acceptance.items()
            if isinstance(value, dict) and not value.get("success", False)
        ],
    }
    report["acceptance"] = acceptance
    return report


def _ordered_start_rooms(
    room_ids: Sequence[str],
    preferred_start_room_id: Optional[str],
    target_room_id: Optional[str],
) -> List[str]:
    canonical_preferred = _canonical_room_id(preferred_start_room_id) if preferred_start_room_id else None
    if canonical_preferred is not None:
        return [canonical_preferred]
    target_room_id = _canonical_room_id(target_room_id)
    other_rooms = [room_id for room_id in room_ids if room_id != target_room_id]
    if target_room_id in room_ids:
        other_rooms.append(target_room_id)
    return other_rooms


def _policy_summaries(
    route_fn: RouteFn,
    start_room_id: str,
    target: Any,
) -> Dict[str, Any]:
    summaries = {}
    for route_policy in ("strict", "balanced", "exploratory"):
        result = route_fn(start_room_id, target, route_policy)
        route = result.get("route", {})
        summaries[route_policy] = {
            "success": bool(result.get("found")),
            "failure_reason": result.get("failure_reason"),
            "resolved_goal_room_id": result.get("resolved_goal_room_id"),
            "hop_count": route.get("hop_count"),
            "room_sequence": list(route.get("room_sequence", [])),
            "used_relation_types": list(route.get("used_relation_types", [])),
        }
    return summaries


def _exercise_target_acceptance(
    *,
    label: str,
    candidates: Sequence[Any],
    no_candidates_reason: str,
    no_candidates_note: str,
    preferred_start_room_id: Optional[str],
    room_ids: Sequence[str],
    resolve_fn: ResolveFn,
    route_fn: RouteFn,
    compare_policies: bool,
    target_key: str,
    explicit_target_requested: bool,
) -> Dict[str, Any]:
    if not room_ids:
        return {
            "attempted": False,
            "success": False,
            target_key: None,
            "start_room_id": None,
            "resolved_room_id": None,
            "failure_reason": "no_rooms_present",
            "notes": ["No rooms are present in this topology export."],
        }
    if not candidates:
        return {
            "attempted": False,
            "success": False,
            target_key: None,
            "start_room_id": None,
            "resolved_room_id": None,
            "failure_reason": no_candidates_reason,
            "notes": [no_candidates_note],
        }

    first_resolution_failure: Optional[Dict[str, Any]] = None
    first_route_failure: Optional[Dict[str, Any]] = None
    attempted_candidates = 0
    for candidate in candidates:
        attempted_candidates += 1
        resolution = resolve_fn(candidate)
        if not resolution.get("resolved"):
            if first_resolution_failure is None:
                first_resolution_failure = {
                    target_key: candidate,
                    "failure_reason": resolution.get("failure_reason"),
                    "notes": list(resolution.get("notes", [])),
                }
            continue

        target_room_id = resolution.get("resolved_room_id")
        for start_room_id in _ordered_start_rooms(room_ids, preferred_start_room_id, target_room_id):
            result = route_fn(start_room_id, candidate, "balanced")
            if result.get("found"):
                payload = {
                    "attempted": True,
                    "success": True,
                    target_key: candidate,
                    "start_room_id": result.get("start_room_id"),
                    "resolved_room_id": result.get("resolved_goal_room_id"),
                    "failure_reason": None,
                    "notes": [
                        f"{label} query succeeded on a real exported target.",
                    ],
                    "route_policy": "balanced",
                    "query_result": result,
                    "attempted_candidate_count": attempted_candidates,
                }
                if compare_policies:
                    payload["policy_results"] = _policy_summaries(route_fn, start_room_id, candidate)
                return payload
            if first_route_failure is None:
                first_route_failure = {
                    target_key: candidate,
                    "start_room_id": start_room_id,
                    "resolved_room_id": result.get("resolved_goal_room_id"),
                    "failure_reason": result.get("failure_reason"),
                    "notes": list(result.get("target_resolution", {}).get("notes", [])),
                    "query_result": result,
                }

    if first_route_failure is not None:
        notes = list(first_route_failure.get("notes", []))
        notes.append("No routable target found after checking real exported targets.")
        return {
            "attempted": True,
            "success": False,
            target_key: first_route_failure.get(target_key),
            "start_room_id": first_route_failure.get("start_room_id"),
            "resolved_room_id": first_route_failure.get("resolved_room_id"),
            "failure_reason": "no_routable_target_found",
            "notes": notes,
            "query_result": first_route_failure.get("query_result"),
            "attempted_candidate_count": attempted_candidates,
        }

    if first_resolution_failure is not None:
        return {
            "attempted": True,
            "success": False,
            target_key: first_resolution_failure.get(target_key),
            "start_room_id": None,
            "resolved_room_id": None,
            "failure_reason": first_resolution_failure.get("failure_reason"),
            "notes": list(first_resolution_failure.get("notes", [])),
            "attempted_candidate_count": attempted_candidates,
        }

    fallback_reason = "requested_target_not_found" if explicit_target_requested else no_candidates_reason
    fallback_note = (
        f"Requested {label} target is not present in this export."
        if explicit_target_requested
        else no_candidates_note
    )
    return {
        "attempted": False,
        "success": False,
        target_key: None,
        "start_room_id": None,
        "resolved_room_id": None,
        "failure_reason": fallback_reason,
        "notes": [fallback_note],
    }


def _exercise_anchor_acceptance(
    query_api: RoomTopologyQueryAPI,
    room_ids: Sequence[str],
    preferred_start_room_id: Optional[str],
    anchor_id: Optional[str],
    compare_policies: bool,
) -> Dict[str, Any]:
    candidates = [anchor_id] if anchor_id else query_api.topology.list_anchor_ids(valid_only=True)
    if not candidates and anchor_id:
        candidates = [anchor_id]
    return _exercise_target_acceptance(
        label="anchor",
        candidates=candidates,
        no_candidates_reason="no_anchors_present",
        no_candidates_note="No anchors present in this export.",
        preferred_start_room_id=preferred_start_room_id,
        room_ids=room_ids,
        resolve_fn=query_api.resolve_anchor_room,
        route_fn=query_api.query_route_to_anchor,
        compare_policies=compare_policies,
        target_key="anchor_id",
        explicit_target_requested=anchor_id is not None,
    )


def _exercise_object_id_acceptance(
    query_api: RoomTopologyQueryAPI,
    room_ids: Sequence[str],
    preferred_start_room_id: Optional[str],
    object_id: Optional[str],
    compare_policies: bool,
) -> Dict[str, Any]:
    candidates = [object_id] if object_id is not None else query_api.topology.list_object_ids()
    return _exercise_target_acceptance(
        label="object id",
        candidates=candidates,
        no_candidates_reason="no_objects_present",
        no_candidates_note="No objects present in this export.",
        preferred_start_room_id=preferred_start_room_id,
        room_ids=room_ids,
        resolve_fn=lambda candidate: query_api.resolve_object_room(object_id=candidate),
        route_fn=lambda start_room_id, candidate, route_policy: query_api.query_route_to_object(
            start_room_id,
            object_id=candidate,
            route_policy=route_policy,
        ),
        compare_policies=compare_policies,
        target_key="object_id",
        explicit_target_requested=object_id is not None,
    )


def _exercise_object_label_acceptance(
    query_api: RoomTopologyQueryAPI,
    room_ids: Sequence[str],
    preferred_start_room_id: Optional[str],
    object_label: Optional[str],
    compare_policies: bool,
) -> Dict[str, Any]:
    candidates = [object_label] if object_label else query_api.topology.list_object_labels()
    return _exercise_target_acceptance(
        label="object label",
        candidates=candidates,
        no_candidates_reason="object_label_lookup_unavailable",
        no_candidates_note="This export does not contain inspectable object labels.",
        preferred_start_room_id=preferred_start_room_id,
        room_ids=room_ids,
        resolve_fn=lambda candidate: query_api.resolve_object_room(object_label=candidate),
        route_fn=lambda start_room_id, candidate, route_policy: query_api.query_route_to_object(
            start_room_id,
            object_label=candidate,
            route_policy=route_policy,
        ),
        compare_policies=compare_policies,
        target_key="object_label",
        explicit_target_requested=object_label is not None,
    )


def _print_text_report(
    report: Dict[str, Any],
    list_mode: str,
    limit: int,
) -> None:
    inspection = report["inspection"]
    print(f"topology_json: {report['topology_json']}")
    print(f"sequence_id: {report.get('sequence_id')}")
    print(
        "counts: "
        f"rooms={inspection['room_count']} "
        f"anchors={inspection['anchor_count']} "
        f"objects={inspection['object_count']} "
        f"object_labels={inspection['object_label_count']}"
    )
    if inspection.get("diagnostics"):
        print("diagnostics:")
        for item in inspection["diagnostics"]:
            print(f"  - {item}")

    list_mode = str(list_mode)
    limit = max(0, int(limit))
    list_payloads = {
        "rooms": inspection["room_ids"],
        "anchors": inspection["anchor_ids"],
        "objects": inspection["object_ids"],
        "object-labels": inspection["object_labels"],
    }
    if list_mode in {"all", "summary"}:
        print(f"sample_rooms: {inspection['sample_room_ids']}")
        print(f"sample_anchors: {inspection['sample_anchor_ids']}")
        print(f"sample_objects: {inspection['sample_object_ids']}")
        print(f"sample_object_labels: {inspection['sample_object_labels']}")
    if list_mode == "all":
        for key, values in list_payloads.items():
            print(f"{key}: {values[:limit] if limit else values}")
    elif list_mode in list_payloads:
        for value in (list_payloads[list_mode][:limit] if limit else list_payloads[list_mode]):
            print(value)

    acceptance = report.get("acceptance")
    if acceptance:
        print("acceptance:")
        print(f"  start_room_id: {acceptance.get('start_room_id')}")
        for key in ("route_to_anchor", "route_to_object_id", "route_to_object_label"):
            item = acceptance[key]
            status = "PASS" if item.get("success") else "FAIL"
            target = item.get("anchor_id") or item.get("object_id") or item.get("object_label")
            print(
                f"  {key}: {status} "
                f"target={target} start={item.get('start_room_id')} "
                f"goal={item.get('resolved_room_id')} "
                f"reason={item.get('failure_reason')}"
            )
            for note in item.get("notes", []):
                print(f"    note: {note}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect and acceptance-check exported topology JSON against the current Query API stack."
    )
    parser.add_argument("--topology-json", required=True, help="Path to logs/topology_v0_1.json")
    parser.add_argument(
        "--list",
        default="summary",
        choices=["summary", "rooms", "anchors", "objects", "object-labels", "all"],
        help="Which entities to print for inspection.",
    )
    parser.add_argument("--limit", type=int, default=20, help="Max items to print per entity list in text mode.")
    parser.add_argument("--acceptance", action="store_true", help="Run acceptance queries on real targets from the export.")
    parser.add_argument("--start", default=None, help="Optional structured start room id to force during acceptance.")
    parser.add_argument("--anchor-id", default=None, help="Optional anchor id to validate explicitly.")
    parser.add_argument("--object-id", default=None, help="Optional object id to validate explicitly.")
    parser.add_argument("--object-label", default=None, help="Optional object label to validate explicitly.")
    parser.add_argument(
        "--compare-policies",
        action="store_true",
        help="When an acceptance query succeeds, also summarize strict / balanced / exploratory results.",
    )
    parser.add_argument("--sample-limit", type=int, default=5, help="Number of sample ids/labels to include in reports.")
    parser.add_argument("--report-json-out", default=None, help="Optional path to save the structured report as JSON.")
    parser.add_argument("--json", action="store_true", help="Print the structured report as JSON.")
    args = parser.parse_args()

    if args.acceptance or args.anchor_id or args.object_id or args.object_label:
        report = build_acceptance_report(
            args.topology_json,
            start_room_id=args.start,
            sample_limit=args.sample_limit,
            anchor_id=args.anchor_id,
            object_id=args.object_id,
            object_label=args.object_label,
            compare_policies=args.compare_policies,
        )
    else:
        report = build_topology_report(args.topology_json, sample_limit=args.sample_limit)

    if args.report_json_out:
        report_path = Path(args.report_json_out)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)

    if args.json:
        print(json.dumps(report, indent=2))
        return
    _print_text_report(report, list_mode=args.list, limit=args.limit)


if __name__ == "__main__":
    main()
