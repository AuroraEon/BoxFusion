from __future__ import annotations

import copy
import json
import shlex
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict

from boxfusion.artifact_contract import ARTIFACT_PROFILE_CORE_ONLY
from boxfusion.backend_eval_scaffold import write_scene_manifest
from boxfusion.runtime_artifact_policy import RUNTIME_ARTIFACT_MODE_SERVICE
from boxfusion.runtime_snapshot import (
    build_runtime_snapshot_from_bundles,
    build_runtime_snapshot_from_live_stage_state,
    compare_minimal_public_topology_shadow_parity,
    compare_runtime_snapshot_shadow_parity,
)
from boxfusion.ros_publication_diagnostics_server import load_publication_diagnostics_bundle
from boxfusion.ros_query_server import load_committed_public_bundle
from boxfusion.runtime_export_coordinator import BoxFusionRuntimeExportCoordinator


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_runtime_export",
        "floors": [{"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1}],
        "rooms": [
            {
                "id": "room_1",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 1,
                "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]],
            },
            {
                "id": "room_3",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 2,
                "polygon": [[1, 0], [2, 0], [2, 1], [1, 1]],
            }
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_3",
                "relation_type": "adjacent",
                "confidence": 0.8,
                "status": "confirmed",
                "support_count": 1,
            }
        ],
        "indices": {
            "object_to_room": {"obj_11": "room_1", "obj_12": "room_3"},
            "anchor_to_room": {"anchor_1": "room_1"},
            "room_to_objects": {"room_1": ["obj_11"], "room_3": ["obj_12"]},
            "room_to_anchors": {"room_1": ["anchor_1"]},
        },
        "entities": {
            "objects": [
                {
                    "id": "obj_11",
                    "label": "chair",
                    "category": "chair",
                    "room_id": "room_1",
                    "floor_id": "floor_1",
                },
                {
                    "id": "obj_12",
                    "label": "table",
                    "category": "table",
                    "room_id": "room_3",
                    "floor_id": "floor_1",
                }
            ],
            "anchors": [
                {
                    "id": "anchor_1",
                    "anchor_type": "room_center",
                    "room_id": "room_1",
                    "floor_id": "floor_1",
                }
            ],
        },
        "evidences": [],
        "metadata": {},
    }


def _runtime_vector_map_payload() -> Dict[str, Any]:
    return {
        "floors": [{"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1}],
        "rooms": [{"id": 1, "floor_id": "floor_1"}, {"id": 3, "floor_id": "floor_1"}],
        "edges": [{"source": 1, "target": 3, "relation_type": "adjacent", "status": "confirmed"}],
        "objects": [
            {"id": 11, "label": "chair", "room_uuid": 1, "floor_id": "floor_1"},
            {"id": 12, "label": "table", "room_uuid": 3, "floor_id": "floor_1"},
        ],
        "anchors": [{"id": "anchor_1", "room_id": "room_1", "floor_id": "floor_1"}],
    }


def _write_scene_root(scene_root: Path) -> Path:
    logs = scene_root / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    _write_json(logs / "topology_v0_1.json", _topology_payload())
    _write_json(logs / "topology_query_report.json", {"room_count": 2, "object_count": 2, "anchor_count": 1, "edge_count": 1, "floor_count": 1})
    _write_json(logs / "vertical_transition_evidence.json", {"summary": {"count": 0}, "transitions": []})
    _write_json(logs / "floor_diagnostics_summary.json", {"per_floor": [{"floor_id": "floor_1"}]})
    _write_json(logs / "runtime_growth_profile.json", {"records": [], "summary": {"sample_count": 0}})
    (logs / "runtime_growth_profile.csv").write_text("frame_idx,total_step_sec\n", encoding="utf-8")
    _write_json(
        logs / "online_topology_lifecycle_v0_1.json",
        {
            "sequence_id": scene_root.name,
            "frame_idx": 5,
            "timestamp": 0.5,
            "summary": {"committed_room_count": 2},
            "rooms": [
                {
                    "room_id": "room_1",
                    "published": True,
                    "publication_state": "PUBLISHED",
                    "finalization_blockers": [],
                    "publication_blockers": [],
                },
                {
                    "room_id": "room_2",
                    "published": False,
                    "publication_state": "CANDIDATE",
                    "finalization_blockers": ["min_observation_count"],
                    "publication_blockers": ["not_committed"],
                },
                {
                    "room_id": "room_3",
                    "published": True,
                    "publication_state": "PUBLISHED",
                    "finalization_blockers": [],
                    "publication_blockers": [],
                },
            ],
            "refresh_history": [],
            "committed_rooms": ["room_1", "room_3"],
        },
    )
    _write_json(logs / "working_topology_v0_1.json", {"metadata": {"artifact_kind": "working_topology_debug"}})
    _write_json(logs / "working_vs_committed_topology_report_v0_1.json", {"artifact_kind": "working_vs_committed_topology_report_debug"})
    _write_json(logs / "working_vs_committed_topology_timeline_v0_1.json", {"artifact_kind": "working_vs_committed_topology_timeline_debug"})
    (logs / "working_vs_committed_topology_timeline_v0_1.md").write_text("# mock\n", encoding="utf-8")
    _write_json(
        logs / "summary.json",
        {
            "sequence_id": scene_root.name,
            "output_mode": "core_only",
            "artifact_profile": ARTIFACT_PROFILE_CORE_ONLY,
            "core_only_mode": True,
            "optional_demo_artifacts_enabled": False,
            "snapshot_count": 1,
            "segmentation_cycle_count": 3,
            "final_room_count": 2,
            "final_object_count": 2,
            "final_anchor_count": 1,
            "runtime_growth_summary": {"sample_count": 0},
        },
    )
    write_scene_manifest(scene_root, sequence_name=scene_root.name)
    return scene_root


def test_refresh_once_publishes_latest_pointer_usable_by_consumers(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00843-DYehNKdT76V")
    coordination_root = tmp_path / "exports"

    result = BoxFusionRuntimeExportCoordinator(
        source_scene_root=scene_root,
        coordination_root=coordination_root,
        refresh_interval_sec=0.01,
    ).refresh_once()

    latest_scene_root = coordination_root / "latest"
    latest_manifest_path = coordination_root / "latest_manifest.json"
    latest_export_path = coordination_root / "latest_export.json"

    assert latest_scene_root.is_symlink()
    assert latest_manifest_path.is_symlink()
    assert latest_export_path.exists()

    committed_bundle = load_committed_public_bundle(latest_scene_root)
    diagnostics_bundle = load_publication_diagnostics_bundle(latest_scene_root)
    latest_export = json.loads(latest_export_path.read_text(encoding="utf-8"))

    assert committed_bundle.topology_path.name == "topology_v0_1.json"
    assert diagnostics_bundle.lifecycle_path.name == "online_topology_lifecycle_v0_1.json"
    assert latest_export["committed_public"]["surface"] == "public"
    assert latest_export["lifecycle_debug"]["surface"] == "lifecycle"
    assert latest_export["runtime_snapshot"]["contract_version"] == "0.1"
    assert latest_export["runtime_snapshot"]["committed_public_export_surface"]["public_topology_meaning"] == "committed_published_only"
    assert latest_export["runtime_snapshot"]["committed_public_export_surface"]["room_ids"] == ["room_1", "room_3"]
    assert latest_export["runtime_snapshot"]["committed_public_export_surface"]["object_ids"] == ["obj_11", "obj_12"]
    assert latest_export["runtime_snapshot"]["lifecycle_debug_surface"]["non_published_room_count"] == 1
    assert latest_export["runtime_snapshot"]["lifecycle_debug_surface"]["committed_room_ids"] == ["room_1", "room_3"]
    assert latest_export["runtime_snapshot"]["lifecycle_debug_surface"]["non_published_rooms_public"] is False
    assert latest_export["sidecar_export"]["enabled"] is False
    assert latest_export["consumer_inputs"]["query_server_artifact_path"] == str(latest_scene_root)
    assert result.describe()["consumer_inputs"]["publication_diagnostics_artifact_path"] == str(latest_scene_root)


def test_refresh_can_wrap_producer_command_before_manifest_refresh(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00824-Dd4bFSTQ8gi")
    coordination_root = tmp_path / "exports"
    marker_path = scene_root / "producer_marker.txt"
    producer_code = f"from pathlib import Path; Path({str(marker_path)!r}).write_text('ok', encoding='utf-8')"
    command = (
        f"{shlex.quote(sys.executable)} -c "
        f"{shlex.quote(producer_code)}"
    )

    result = BoxFusionRuntimeExportCoordinator(
        source_scene_root=scene_root,
        coordination_root=coordination_root,
        producer_command=command,
        refresh_interval_sec=0.01,
    ).refresh_once()

    assert marker_path.read_text(encoding="utf-8") == "ok"
    assert result.producer_command == command
    assert (coordination_root / "refresh_history").exists()


def test_service_mode_snapshot_is_consumable_by_shadow_sidecar_and_keeps_public_queries(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00861-GLAQ4DNUx5U")
    coordination_root = tmp_path / "exports"

    result = BoxFusionRuntimeExportCoordinator(
        source_scene_root=scene_root,
        coordination_root=coordination_root,
        refresh_interval_sec=0.01,
        runtime_artifact_mode=RUNTIME_ARTIFACT_MODE_SERVICE,
        enable_sidecar_shadow_export=True,
    ).refresh_once()

    latest_scene_root = coordination_root / "latest"
    latest_export = json.loads((coordination_root / "latest_export.json").read_text(encoding="utf-8"))
    sidecar_path = Path(latest_export["sidecar_export"]["output_path"])
    sidecar_payload = json.loads(sidecar_path.read_text(encoding="utf-8"))
    committed_bundle = load_committed_public_bundle(latest_scene_root)
    diagnostics_bundle = load_publication_diagnostics_bundle(latest_scene_root)

    assert latest_export["runtime_artifact_policy"]["mode"] == "service"
    assert latest_export["runtime_artifact_policy"]["prepare_gt_visualization_pointcloud"] is False
    assert latest_export["runtime_artifact_policy"]["full_rgb_replay"] is False
    assert latest_export["runtime_artifact_policy"]["readonly_tail_reference_audit"] is False
    assert latest_export["runtime_artifact_policy"]["write_debug_room_artifacts"] is False
    assert latest_export["authoritative_export_path"]["sidecar_replaces_authoritative_export"] is False
    assert result.runtime_snapshot.to_dict()["committed_public_export_surface"]["surface"] == "public"
    assert result.sidecar_export_result.authoritative is False
    assert sidecar_payload["materialized_keys"] == [
        "minimal_runtime_state_summary",
        "minimal_public_topology_summary",
        "minimal_committed_public_topology_subset",
        "minimal_lifecycle_debug_linkage",
    ]
    assert sidecar_payload["shadow_materialization"]["authoritative"] is False
    assert sidecar_payload["shadow_materialization"]["public_topology"]["topology_semantics"] == "committed_topology"
    assert sidecar_payload["shadow_materialization"]["public_topology"]["room_ids"] == ["room_1", "room_3"]
    assert sidecar_payload["shadow_materialization"]["public_topology"]["object_ids"] == ["obj_11", "obj_12"]
    minimal_topology = sidecar_payload["shadow_materialization"]["minimal_public_topology_subset"]
    assert minimal_topology["authoritative"] is False
    assert minimal_topology["public_topology_meaning"] == "committed_published_only"
    assert minimal_topology["counts"] == {
        "edge_count": 1,
        "floor_count": 1,
        "object_room_membership_count": 2,
        "room_count": 2,
    }
    assert minimal_topology["rooms"] == [
        {
            "display_floor_id": "floor_1",
            "display_order": 1,
            "floor_id": "floor_1",
            "id": "room_1",
            "room_type": "unknown",
            "status": "confirmed",
        },
        {
            "display_floor_id": "floor_1",
            "display_order": 2,
            "floor_id": "floor_1",
            "id": "room_3",
            "room_type": "unknown",
            "status": "confirmed",
        },
    ]
    assert minimal_topology["edges"] == [
        {"relation_type": "adjacent", "source": "room_1", "status": "confirmed", "target": "room_3"}
    ]
    public_topology_comparison = compare_minimal_public_topology_shadow_parity(
        authoritative_topology_payload=committed_bundle.topology_payload,
        sidecar_payload=sidecar_payload,
    )
    assert public_topology_comparison.passed, public_topology_comparison.to_dict()
    sidecar_comparison = compare_runtime_snapshot_shadow_parity(
        authoritative_snapshot=result.runtime_snapshot,
        sidecar_payload=sidecar_payload,
    )
    assert sidecar_comparison.passed, sidecar_comparison.to_dict()
    assert committed_bundle.topology_payload["rooms"][0]["id"] == "room_1"
    assert diagnostics_bundle.lifecycle_payload["rooms"][1]["publication_state"] == "CANDIDATE"


def test_minimal_public_topology_shadow_parity_reports_diagnostic_mismatches(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00861-topology-shadow-diff")
    coordination_root = tmp_path / "exports"

    result = BoxFusionRuntimeExportCoordinator(
        source_scene_root=scene_root,
        coordination_root=coordination_root,
        refresh_interval_sec=0.01,
        enable_sidecar_shadow_export=True,
    ).refresh_once()

    sidecar_path = result.sidecar_export_result.output_path
    assert sidecar_path is not None
    sidecar_payload = json.loads(sidecar_path.read_text(encoding="utf-8"))
    changed_payload = copy.deepcopy(sidecar_payload)
    changed_payload["shadow_materialization"]["minimal_public_topology_subset"]["edges"][0]["relation_type"] = "possible_connection"

    comparison = compare_minimal_public_topology_shadow_parity(
        authoritative_topology_payload=result.committed_bundle.topology_payload,
        sidecar_payload=changed_payload,
        max_mismatches=4,
    )

    assert comparison.passed is False
    assert len(comparison.mismatches) <= 4
    assert any(item["key"] == "records.edges" for item in comparison.mismatches)
    assert all(item["diagnostic_only"] is True for item in comparison.mismatches)


def test_direct_live_state_snapshot_matches_manifest_backed_minimal_subset(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00862-direct-shadow")
    committed_bundle = load_committed_public_bundle(scene_root)
    diagnostics_bundle = load_publication_diagnostics_bundle(scene_root)
    created_at_utc = "2026-04-15T00:00:00+00:00"
    authoritative_snapshot = build_runtime_snapshot_from_bundles(
        committed_bundle=committed_bundle,
        diagnostics_bundle=diagnostics_bundle,
        created_at_utc=created_at_utc,
    )
    stage5_snapshot = SimpleNamespace(
        frame_idx=5,
        timestamp=0.5,
        vector_map=_runtime_vector_map_payload(),
        vector_map_path=None,
        segmentation_cycle_idx=3,
    )
    stage3_runtime_state = SimpleNamespace(
        sequence_id=scene_root.name,
        snapshots=[stage5_snapshot],
        latest_vector_map=_runtime_vector_map_payload(),
    )

    direct_snapshot = build_runtime_snapshot_from_live_stage_state(
        stage3_state=stage3_runtime_state,
        stage5_state=stage5_snapshot,
        committed_bundle=committed_bundle,
        diagnostics_bundle=diagnostics_bundle,
        created_at_utc=created_at_utc,
    )
    comparison = compare_runtime_snapshot_shadow_parity(
        authoritative_snapshot=authoritative_snapshot,
        shadow_snapshot=direct_snapshot,
    )

    assert direct_snapshot.to_dict()["runtime_maintained_state"]["final_room_ids"] == ["room_1", "room_3"]
    assert direct_snapshot.to_dict()["runtime_maintained_state"]["final_object_ids"] == ["obj_11", "obj_12"]
    assert comparison.passed, comparison.to_dict()


def test_shadow_parity_comparison_reports_bounded_diagnostic_mismatches(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "scene" / "00863-direct-shadow-diff")
    committed_bundle = load_committed_public_bundle(scene_root)
    diagnostics_bundle = load_publication_diagnostics_bundle(scene_root)
    created_at_utc = "2026-04-15T00:00:00+00:00"
    authoritative_snapshot = build_runtime_snapshot_from_bundles(
        committed_bundle=committed_bundle,
        diagnostics_bundle=diagnostics_bundle,
        created_at_utc=created_at_utc,
    )
    vector_map = _runtime_vector_map_payload()
    vector_map["objects"] = []
    stage5_snapshot = SimpleNamespace(
        frame_idx=5,
        timestamp=0.5,
        vector_map=vector_map,
        vector_map_path=None,
        segmentation_cycle_idx=3,
    )
    stage3_runtime_state = SimpleNamespace(
        sequence_id=scene_root.name,
        snapshots=[stage5_snapshot],
        latest_vector_map=vector_map,
    )

    direct_snapshot = build_runtime_snapshot_from_live_stage_state(
        stage3_state=stage3_runtime_state,
        stage5_state=stage5_snapshot,
        committed_bundle=committed_bundle,
        diagnostics_bundle=diagnostics_bundle,
        created_at_utc=created_at_utc,
    )
    comparison = compare_runtime_snapshot_shadow_parity(
        authoritative_snapshot=authoritative_snapshot,
        shadow_snapshot=direct_snapshot,
        max_mismatches=4,
    )

    assert comparison.passed is False
    assert len(comparison.mismatches) <= 4
    assert any(item["key"] == "runtime.final_object_count" for item in comparison.mismatches)
    assert all(item["diagnostic_only"] is True for item in comparison.mismatches)
