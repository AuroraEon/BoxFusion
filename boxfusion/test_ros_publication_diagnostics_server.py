from __future__ import annotations

from pathlib import Path

from boxfusion.ros_publication_diagnostics_server import (
    DEFAULT_SERVICE_MODULE,
    BoxFusionRosPublicationDiagnosticsBackend,
    build_arg_parser,
    load_publication_diagnostics_bundle,
)


FULL_BUNDLE_PATH = Path("codex_acceptance_audit/full_artifact/00843-DYehNKdT76V")
CORE_ONLY_BUNDLE_PATH = Path("codex_acceptance_audit/core_only/00843-DYehNKdT76V")
CANDIDATE1_BUNDLE_PATH = Path(
    "codex_perf_probe/server_preinfer_opt_pass/validation_candidate1_fast_gt_resize300/output/00843-DYehNKdT76V"
)


def _payload(result: object) -> dict:
    return dict(getattr(result, "payload").get("payload") or {})


def test_load_publication_diagnostics_bundle_from_manifest() -> None:
    bundle = load_publication_diagnostics_bundle(FULL_BUNDLE_PATH / "manifest.json")
    info = bundle.describe()
    assert info["artifact_profile"] == "full_artifact"
    assert info["artifact_surface"] == "lifecycle"
    assert info["artifact_semantics"] == "diagnostic_history"
    assert bundle.lifecycle_payload["sequence_id"] == "00843-DYehNKdT76V"


def test_publication_diagnostics_backend_distinguishes_published_vs_prepublication_rooms() -> None:
    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(CORE_ONLY_BUNDLE_PATH)

    payload = _payload(backend.get_publication_diagnostics())
    assert payload["artifact_surface"] == "lifecycle"
    assert payload["debug_only"] is True
    assert payload["public_contract"] is False
    assert payload["summary"]["published_room_count"] == 1
    assert payload["summary"]["pre_publication_room_count"] == 2

    room_lookup = {room["room_id"]: room for room in payload["rooms"]}
    assert room_lookup["room_1"]["publication_state"] == "PUBLISHED"
    assert room_lookup["room_1"]["published"] is True
    assert room_lookup["room_1"]["pre_publication_only"] is False
    assert room_lookup["room_2"]["publication_state"] == "ACTIVE_OBSERVING"
    assert room_lookup["room_2"]["finalization_blockers"] == [
        "containment_not_stable",
        "gateway_structure_not_stable",
        "room_signature_not_stable",
    ]
    assert room_lookup["room_2"]["publication_blockers"] == ["no_leave_like_signal"]
    assert room_lookup["room_2"]["published"] is False
    assert room_lookup["room_2"]["pre_publication_only"] is True
    assert room_lookup["room_3"]["publication_blockers"] == ["room_currently_active"]


def test_room_publication_state_service_style_response_handles_hits_and_misses() -> None:
    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(CORE_ONLY_BUNDLE_PATH)

    found_payload = _payload(backend.get_room_publication_state(room_id="room_1"))
    assert found_payload["room_found"] is True
    assert found_payload["room"]["publication_state"] == "PUBLISHED"
    assert found_payload["room"]["public_topology_membership"] == "published"

    normalized_payload = _payload(backend.get_room_publication_state(room_id="2"))
    assert normalized_payload["room_found"] is True
    assert normalized_payload["requested_room_id"] == "room_2"
    assert normalized_payload["room"]["public_topology_membership"] == "debug_only_pre_publication"

    miss_payload = _payload(backend.get_room_publication_state(room_id="room_999"))
    assert miss_payload["room_found"] is False
    assert miss_payload["room"] is None


def test_candidate_artifact_examples_match_expected_publication_states() -> None:
    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(CANDIDATE1_BUNDLE_PATH)
    payload = _payload(backend.get_publication_diagnostics())
    room_lookup = {room["room_id"]: room for room in payload["rooms"]}

    assert room_lookup["room_1"]["publication_state"] == "CANDIDATE_FORMED"
    assert room_lookup["room_1"]["finalization_blockers"] == [
        "containment_not_stable",
        "floor_status_not_stable",
        "gateway_structure_not_stable",
        "merge_or_split_pending",
        "room_signature_not_stable",
    ]
    assert room_lookup["room_1"]["publication_blockers"] == ["no_leave_like_signal"]
    assert room_lookup["room_1"]["published"] is False

    assert room_lookup["room_2"]["publication_state"] == "CANDIDATE_FORMED"
    assert room_lookup["room_2"]["publication_blockers"] == []
    assert room_lookup["room_3"]["publication_blockers"] == ["room_currently_active"]
    assert payload["summary"]["published_room_count"] == 0


def test_ros_publication_diagnostics_arg_parser_defaults_to_local_interface_package() -> None:
    args = build_arg_parser().parse_args([])
    assert args.service_module == DEFAULT_SERVICE_MODULE
