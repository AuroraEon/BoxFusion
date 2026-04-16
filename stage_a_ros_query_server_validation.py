from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

from boxfusion.ros_query_server import BoxFusionRosQueryServerBackend


DEFAULT_BUNDLE_PATH = Path("codex_acceptance_audit/full_artifact/00843-DYehNKdT76V")


def _response_payload(result: Any) -> Dict[str, Any]:
    return dict(result.payload.get("payload") or {})


def _pick_route_to_room_case(backend: BoxFusionRosQueryServerBackend) -> Tuple[Dict[str, Any], Any]:
    room_ids = backend.query_api.topology.list_room_ids()
    for start_room_id in room_ids:
        for goal_room_id in room_ids:
            if start_room_id == goal_room_id:
                continue
            result = backend.route_to_room(start_room_id=start_room_id, goal_room_id=goal_room_id)
            payload = _response_payload(result)
            if payload.get("found"):
                return {
                    "start_room_id": start_room_id,
                    "goal_room_id": goal_room_id,
                    "route_policy": "balanced",
                }, result
    raise RuntimeError("No routable room pair found in the committed topology export.")


def _pick_object_label_case(backend: BoxFusionRosQueryServerBackend) -> Tuple[Dict[str, Any], Any]:
    for object_label in backend.query_api.topology.list_object_labels():
        result = backend.resolve_object_room(object_label=object_label)
        payload = _response_payload(result)
        if payload.get("resolved"):
            return {"object_label": object_label}, result
    raise RuntimeError("No uniquely resolvable object label found in the committed topology export.")


def _pick_route_to_object_case(backend: BoxFusionRosQueryServerBackend) -> Tuple[Dict[str, Any], Any]:
    room_ids = backend.query_api.topology.list_room_ids()
    for object_id in backend.query_api.topology.list_object_ids():
        resolve_result = backend.resolve_object_room(object_id=object_id)
        resolve_payload = _response_payload(resolve_result)
        if not resolve_payload.get("resolved"):
            continue
        target_room_id = str(resolve_payload.get("resolved_room_id"))
        start_candidates = [room_id for room_id in room_ids if room_id != target_room_id] + [target_room_id]
        for start_room_id in start_candidates:
            result = backend.route_to_object(start_room_id=start_room_id, object_id=object_id)
            payload = _response_payload(result)
            if payload.get("found"):
                return {
                    "start_room_id": start_room_id,
                    "object_id": object_id,
                    "route_policy": "balanced",
                }, result
    raise RuntimeError("No routable object target found in the committed topology export.")


def build_validation_report(backend: BoxFusionRosQueryServerBackend) -> Dict[str, Any]:
    topology_result = backend.get_topology()
    resolve_request, resolve_result = _pick_object_label_case(backend)
    room_route_request, room_route_result = _pick_route_to_room_case(backend)
    object_route_request, object_route_result = _pick_route_to_object_case(backend)

    room_route_payload = _response_payload(room_route_result)
    explain_result: Optional[Any] = None
    explain_request: Optional[Dict[str, Any]] = None
    room_sequence = list((room_route_payload.get("route") or {}).get("room_sequence", []))
    if len(room_sequence) >= 2:
        explain_request = {
            "room_a": room_sequence[0],
            "room_b": room_sequence[1],
        }
        explain_result = backend.explain_connection(**explain_request)

    return {
        "bundle": backend.bundle.describe(),
        "service_calls": {
            "GetTopology": {
                "request": {},
                "response": topology_result.payload,
            },
            "ResolveObjectRoom": {
                "request": resolve_request,
                "response": resolve_result.payload,
            },
            "RouteToRoom": {
                "request": room_route_request,
                "response": room_route_result.payload,
            },
            "RouteToObject": {
                "request": object_route_request,
                "response": object_route_result.payload,
            },
            "ExplainConnection": None
            if explain_result is None
            else {
                "request": explain_request,
                "response": explain_result.payload,
            },
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate the minimal committed-artifact-backed ROS query server backend on a real export bundle."
    )
    parser.add_argument(
        "--artifact-path",
        default=str(DEFAULT_BUNDLE_PATH),
        help="Path to a scene root, manifest.json, logs/summary.json, or logs/topology_v0_1.json.",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write the validation report JSON.",
    )
    args = parser.parse_args(argv)

    backend = BoxFusionRosQueryServerBackend.from_bundle_path(Path(args.artifact_path))
    report = build_validation_report(backend)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
