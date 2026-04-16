from __future__ import annotations

from pathlib import Path

from boxfusion.ros_query_server import (
    BoxFusionRosQueryServerBackend,
    DEFAULT_SERVICE_MODULE,
    build_arg_parser,
    load_committed_public_bundle,
)


TEST_BUNDLE_PATH = Path("codex_acceptance_audit/full_artifact/00843-DYehNKdT76V")


def test_load_committed_bundle_from_manifest() -> None:
    bundle = load_committed_public_bundle(TEST_BUNDLE_PATH / "manifest.json")
    info = bundle.describe()
    assert info["artifact_profile"] == "full_artifact"
    assert info["topology_surface"] == "public"
    assert info["topology_semantics"] == "committed_topology"
    assert bundle.topology_payload["sequence_id"] == "00843-DYehNKdT76V"


def test_backend_supports_minimal_query_surface() -> None:
    backend = BoxFusionRosQueryServerBackend.from_bundle_path(TEST_BUNDLE_PATH)

    topology_payload = backend.get_topology().payload["payload"]
    assert len(topology_payload["topology"]["rooms"]) >= 2

    resolve_payload = backend.resolve_object_room(object_label="couch").payload["payload"]
    assert resolve_payload["resolved"] is True

    room_ids = backend.query_api.topology.list_room_ids()
    route_room_payload = None
    for start_room_id in room_ids:
        for goal_room_id in room_ids:
            if start_room_id == goal_room_id:
                continue
            candidate = backend.route_to_room(
                start_room_id=start_room_id,
                goal_room_id=goal_room_id,
            ).payload["payload"]
            if candidate["found"]:
                route_room_payload = candidate
                break
        if route_room_payload is not None:
            break
    assert route_room_payload is not None
    assert route_room_payload["found"] is True

    route_object_payload = None
    for start_room_id in room_ids:
        candidate = backend.route_to_object(
            start_room_id=start_room_id,
            object_label="couch",
        ).payload["payload"]
        if candidate["found"]:
            route_object_payload = candidate
            break
    assert route_object_payload is not None
    assert route_object_payload["found"] is True


def test_ros_query_server_arg_parser_defaults_to_local_interface_package() -> None:
    args = build_arg_parser().parse_args([])
    assert args.service_module == DEFAULT_SERVICE_MODULE
