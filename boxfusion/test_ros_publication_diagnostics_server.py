from __future__ import annotations

import json
from pathlib import Path

from boxfusion.backend_eval_scaffold import write_scene_manifest
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
    assert payload["summary"]["published_room_count"] == 2
    assert payload["summary"]["published_room_ids"] == ["room_2", "room_3"]
    assert payload["summary"]["lifecycle_published_room_count"] == 1
    assert payload["summary"]["lifecycle_published_room_ids"] == ["room_1"]
    assert payload["summary"]["lifecycle_published_but_not_public_room_ids"] == ["room_1"]
    assert payload["summary"]["public_but_not_lifecycle_published_room_ids"] == ["room_2", "room_3"]
    assert payload["summary"]["pre_publication_room_count"] == 1

    room_lookup = {room["room_id"]: room for room in payload["rooms"]}
    assert room_lookup["room_1"]["publication_state"] == "PUBLISHED"
    assert room_lookup["room_1"]["published"] is False
    assert room_lookup["room_1"]["lifecycle_published"] is True
    assert room_lookup["room_1"]["pre_publication_only"] is True
    assert room_lookup["room_2"]["publication_state"] == "ACTIVE_OBSERVING"
    assert room_lookup["room_2"]["finalization_blockers"] == [
        "containment_not_stable",
        "gateway_structure_not_stable",
        "room_signature_not_stable",
    ]
    assert room_lookup["room_2"]["publication_blockers"] == ["no_leave_like_signal"]
    assert room_lookup["room_2"]["published"] is True
    assert room_lookup["room_2"]["lifecycle_published"] is False
    assert room_lookup["room_2"]["pre_publication_only"] is False
    assert room_lookup["room_3"]["publication_blockers"] == ["room_currently_active"]
    assert room_lookup["room_3"]["published"] is True


def test_room_publication_state_service_style_response_handles_hits_and_misses() -> None:
    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(CORE_ONLY_BUNDLE_PATH)

    found_payload = _payload(backend.get_room_publication_state(room_id="room_1"))
    assert found_payload["room_found"] is True
    assert found_payload["room"]["publication_state"] == "PUBLISHED"
    assert found_payload["room"]["public_topology_membership"] == "debug_only_pre_publication"

    normalized_payload = _payload(backend.get_room_publication_state(room_id="2"))
    assert normalized_payload["room_found"] is True
    assert normalized_payload["requested_room_id"] == "room_2"
    assert normalized_payload["room"]["public_topology_membership"] == "published"

    miss_payload = _payload(backend.get_room_publication_state(room_id="room_999"))
    assert miss_payload["room_found"] is False
    assert miss_payload["room"] is None


def test_publication_diagnostics_prefers_authoritative_public_membership_when_available(tmp_path: Path) -> None:
    scene_root = tmp_path / "scene" / "00843-DYehNKdT76V"
    logs_dir = scene_root / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    logs_dir.joinpath("topology_v0_1.json").write_text(
        json.dumps(
            {
                "version": "0.1",
                "sequence_id": scene_root.name,
                "floors": [{"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1}],
                "rooms": [
                    {"id": "room_1", "floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
                    {"id": "room_3", "floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 2},
                ],
                "edges": [],
                "entities": {"objects": [], "anchors": []},
                "metadata": {},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    logs_dir.joinpath("summary.json").write_text(
        json.dumps(
            {
                "sequence_id": scene_root.name,
                "output_mode": "core_only",
                "artifact_profile": "core_only",
                "final_room_count": 2,
                "topology_v0_1_json": str(logs_dir / "topology_v0_1.json"),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    logs_dir.joinpath("online_topology_lifecycle_v0_1.json").write_text(
        json.dumps(
            {
                "sequence_id": scene_root.name,
                "summary": {
                    "room_count": 3,
                    "published_room_count": 2,
                    "committed_room_count": 2,
                    "public_topology_export_succeeded": True,
                },
                "rooms": [
                    {
                        "room_id": "room_1",
                        "lifecycle_state": "committed",
                        "published": True,
                        "publication_state": "PUBLISHED",
                        "present_in_latest_export": True,
                    },
                    {
                        "room_id": "room_2",
                        "lifecycle_state": "committed",
                        "published": True,
                        "publication_state": "PUBLISHED",
                        "present_in_latest_export": False,
                    },
                    {
                        "room_id": "room_3",
                        "lifecycle_state": "merge_or_split_pending",
                        "published": False,
                        "publication_state": "CANDIDATE_FORMED",
                        "present_in_latest_export": True,
                    },
                ],
                "committed_rooms": ["room_1", "room_2"],
                "refresh_history": [],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    write_scene_manifest(scene_root, sequence_name=scene_root.name)

    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(scene_root)
    payload = _payload(backend.get_publication_diagnostics())
    room_lookup = {room["room_id"]: room for room in payload["rooms"]}

    assert payload["summary"]["published_room_count"] == 2
    assert payload["summary"]["published_room_ids"] == ["room_1", "room_3"]
    assert payload["summary"]["lifecycle_published_room_count"] == 2
    assert payload["summary"]["lifecycle_published_room_ids"] == ["room_1", "room_2"]
    assert payload["summary"]["lifecycle_published_but_not_public_room_ids"] == ["room_2"]
    assert payload["summary"]["public_but_not_lifecycle_published_room_ids"] == ["room_3"]
    assert room_lookup["room_2"]["published"] is False
    assert room_lookup["room_2"]["lifecycle_published"] is True
    assert room_lookup["room_2"]["public_topology_membership"] == "debug_only_pre_publication"
    assert room_lookup["room_3"]["published"] is True
    assert room_lookup["room_3"]["lifecycle_published"] is False
    assert room_lookup["room_3"]["public_topology_membership"] == "published"


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
    assert room_lookup["room_2"]["published"] is True
    assert room_lookup["room_3"]["publication_blockers"] == ["room_currently_active"]
    assert payload["summary"]["published_room_count"] == 4
    assert payload["summary"]["published_room_ids"] == ["room_2", "room_3", "room_4", "room_5"]
    assert payload["summary"]["lifecycle_published_room_count"] == 0


def test_ros_publication_diagnostics_arg_parser_defaults_to_local_interface_package() -> None:
    args = build_arg_parser().parse_args([])
    assert args.service_module == DEFAULT_SERVICE_MODULE
