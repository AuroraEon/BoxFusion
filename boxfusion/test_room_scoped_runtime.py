from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from boxfusion.ros_publication_diagnostics_server import BoxFusionRosPublicationDiagnosticsBackend
from boxfusion.ros_query_server import BoxFusionRosQueryServerBackend
from boxfusion.room_scoped_runtime import RoomScopedRuntimeManager
from boxfusion.stage_a_demo import ClosedLoopDemoRecorder


def _room(
    room_id: int,
    *,
    floor_id: str = "floor_1",
    polygon: list[list[float]] | None = None,
) -> dict:
    polygon = polygon or [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    return {
        "id": room_id,
        "room_id": f"room_{room_id}",
        "floor_id": floor_id,
        "room_type": "unknown",
        "status": "confirmed",
        "center": [round(sum(point[0] for point in polygon) / len(polygon), 3), round(sum(point[1] for point in polygon) / len(polygon), 3)],
        "polygon": polygon,
        "area_m2": 4.0,
    }


def _vector_map(
    *,
    rooms: list[dict],
    gateways: list[dict] | None = None,
    objects: list[dict] | None = None,
) -> dict:
    return {
        "floors": [
            {
                "floor_id": "floor_1",
                "floor_index": 0,
                "display_floor_id": "floor_1",
                "display_order": 1,
                "status": "stable",
            }
        ],
        "rooms": list(rooms),
        "gateways": list(gateways or []),
        "vertical_transitions": [],
        "objects": list(objects or []),
        "anchors": [],
        "relationships": [],
        "frame_floor_assignments": [],
        "room_segmentation_diagnostics": {},
        "vertical_transition_summary": {"count": 0},
        "room_floor_validation": {
            "room_count": len(rooms),
            "spanning_room_count": 0,
            "spanning_rooms": [],
            "valid": True,
        },
        "floor_debug": {},
    }


def _pose(x: float, y: float = 1.0, z: float = 0.0) -> np.ndarray:
    pose = np.eye(4, dtype=np.float32)
    pose[0, 3] = float(x)
    pose[1, 3] = float(y)
    pose[2, 3] = float(z)
    return pose


def test_room_scoped_runtime_retrieves_relevant_committed_subset_only() -> None:
    manager = RoomScopedRuntimeManager(
        sequence_id="subset_sequence",
        committed_retrieval_limit=1,
    )
    manager.committed_rooms = {
        "room_1": {"room_id": "room_1", "floor_id": "floor_1", "centroid_xy": [0.0, 0.0], "neighbor_room_ids": []},
        "room_2": {"room_id": "room_2", "floor_id": "floor_1", "centroid_xy": [4.0, 1.0], "neighbor_room_ids": ["room_4"]},
        "room_3": {"room_id": "room_3", "floor_id": "floor_2", "centroid_xy": [0.0, 0.0], "neighbor_room_ids": []},
    }

    manager.observe(
        frame_idx=12,
        timestamp=1.2,
        current_room_id="room_4",
        vector_map=_vector_map(
            rooms=[_room(4, polygon=[[4.2, 0.0], [6.2, 0.0], [6.2, 2.0], [4.2, 2.0]])],
            gateways=[{"connects": [4, 2], "type": "door", "pos_world": [4.1, 1.0]}],
        ),
        export_profile={"changed_room_ids": "room_4"},
        lifecycle_manager=SimpleNamespace(room_statuses={}),
    )

    assert manager.local_state["active_room_id"] == "room_4"
    assert manager.local_state["retrieved_committed_room_ids"] == ["room_2"]
    assert "room_1" not in manager.local_state["retrieved_committed_room_ids"]
    assert "room_3" not in manager.local_state["retrieved_committed_room_ids"]


def test_recorder_exports_committed_only_public_artifacts_from_room_trigger(tmp_path: Path) -> None:
    recorder = ClosedLoopDemoRecorder(
        output_root=str(tmp_path),
        sequence_id="room_commit_sequence",
        capture_stride_frames=1,
        core_only=True,
    )

    image = np.zeros((4, 4, 3), dtype=np.uint8)
    vector_map_a = _vector_map(
        rooms=[_room(1)],
        objects=[{"id": 11, "label": "chair", "category": "chair", "room_uuid": 1, "floor_id": "floor_1"}],
    )
    vector_map_b = _vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        objects=[
            {"id": 11, "label": "chair", "category": "chair", "room_uuid": 1, "floor_id": "floor_1"},
            {"id": 12, "label": "table", "category": "table", "room_uuid": 2, "floor_id": "floor_1"},
        ],
    )

    recorder.record_snapshot(
        frame_idx=0,
        timestamp=0.0,
        image_rgb=image,
        pose=_pose(1.0),
        trajectory_xy=[(1.0, 1.0)],
        vector_map=vector_map_a,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=0,
        export_profile={"changed_room_ids": "room_1"},
    )
    recorder.record_snapshot(
        frame_idx=10,
        timestamp=1.0,
        image_rgb=image,
        pose=_pose(3.0),
        trajectory_xy=[(1.0, 1.0), (3.0, 1.0)],
        vector_map=vector_map_b,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=1,
        export_profile={"changed_room_ids": "room_2"},
    )
    recorder.record_snapshot(
        frame_idx=20,
        timestamp=2.0,
        image_rgb=image,
        pose=_pose(3.0),
        trajectory_xy=[(1.0, 1.0), (3.0, 1.0), (3.0, 1.0)],
        vector_map=vector_map_b,
        tracking_report={},
        segmentation_updated=False,
        segmentation_cycle_idx=2,
        export_profile={},
    )

    outputs = recorder.finalize(
        {
            "processed_frames": 3,
            "duration_sec": 2.0,
            "average_fps": 1.5,
        }
    )

    scene_root = Path(outputs["output_root"])
    summary = json.loads(scene_root.joinpath("logs", "summary.json").read_text(encoding="utf-8"))
    committed_world_model = json.loads(scene_root.joinpath("logs", "committed_room_world_model_v0_1.json").read_text(encoding="utf-8"))
    runtime_state = json.loads(scene_root.joinpath("logs", "room_scoped_runtime_state_v0_1.json").read_text(encoding="utf-8"))

    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(scene_root)
    diagnostics_backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(scene_root)

    topology_payload = dict(query_backend.get_topology().payload["payload"]["topology"])
    world_snapshot = dict(query_backend.get_world_snapshot().payload["payload"]["snapshot"])
    diagnostics_payload = dict(diagnostics_backend.get_publication_diagnostics().payload["payload"])

    assert summary["public_world_snapshot_path"].endswith("committed_room_world_snapshot_v0_1.json")
    assert [room["id"] for room in topology_payload["rooms"]] == ["room_1"]
    assert topology_payload["metadata"]["public_topology_meaning"] == "committed/published only"
    assert [room["id"] for room in world_snapshot["rooms"]] == [1]
    assert committed_world_model["summary"]["committed_room_ids"] == ["room_1"]
    assert committed_world_model["rooms"][0]["semantic_summary"]["dominant_object_labels"] == ["chair"]
    assert runtime_state["local_working_state"]["active_room_id"] == "room_2"
    assert runtime_state["committed_global_memory"]["room_ids"] == ["room_1"]
    assert diagnostics_payload["summary"]["published_room_count"] == 1
    assert diagnostics_payload["summary"]["pre_publication_room_count"] >= 1
    assert diagnostics_payload["public_topology_definition"] == "committed/published only"


def test_recorder_can_suppress_rich_service_debug_artifacts_without_breaking_public_bundle(tmp_path: Path) -> None:
    recorder = ClosedLoopDemoRecorder(
        output_root=str(tmp_path),
        sequence_id="suppressed_service_debug_sequence",
        capture_stride_frames=1,
        core_only=True,
        materialize_rich_service_debug_artifacts=False,
    )

    image = np.zeros((4, 4, 3), dtype=np.uint8)
    vector_map_a = _vector_map(
        rooms=[_room(1)],
        objects=[{"id": 11, "label": "chair", "category": "chair", "room_uuid": 1, "floor_id": "floor_1"}],
    )
    vector_map_b = _vector_map(
        rooms=[
            _room(1),
            _room(2, polygon=[[2.2, 0.0], [4.2, 0.0], [4.2, 2.0], [2.2, 2.0]]),
        ],
        gateways=[{"connects": [1, 2], "type": "door", "pos_world": [2.1, 1.0]}],
        objects=[
            {"id": 11, "label": "chair", "category": "chair", "room_uuid": 1, "floor_id": "floor_1"},
            {"id": 12, "label": "table", "category": "table", "room_uuid": 2, "floor_id": "floor_1"},
        ],
    )

    recorder.record_snapshot(
        frame_idx=0,
        timestamp=0.0,
        image_rgb=image,
        pose=_pose(1.0),
        trajectory_xy=[(1.0, 1.0)],
        vector_map=vector_map_a,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=0,
        export_profile={"changed_room_ids": "room_1"},
    )
    recorder.record_snapshot(
        frame_idx=10,
        timestamp=1.0,
        image_rgb=image,
        pose=_pose(3.0),
        trajectory_xy=[(1.0, 1.0), (3.0, 1.0)],
        vector_map=vector_map_b,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=1,
        export_profile={"changed_room_ids": "room_2"},
    )
    recorder.record_snapshot(
        frame_idx=20,
        timestamp=2.0,
        image_rgb=image,
        pose=_pose(3.0),
        trajectory_xy=[(1.0, 1.0), (3.0, 1.0), (3.0, 1.0)],
        vector_map=vector_map_b,
        tracking_report={},
        segmentation_updated=False,
        segmentation_cycle_idx=2,
        export_profile={},
    )

    outputs = recorder.finalize(
        {
            "processed_frames": 3,
            "duration_sec": 2.0,
            "average_fps": 1.5,
        }
    )

    scene_root = Path(outputs["output_root"])
    summary = json.loads(scene_root.joinpath("logs", "summary.json").read_text(encoding="utf-8"))
    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(scene_root)
    diagnostics_backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(scene_root)

    topology_payload = dict(query_backend.get_topology().payload["payload"]["topology"])
    diagnostics_payload = dict(diagnostics_backend.get_publication_diagnostics().payload["payload"])

    assert summary["materialize_rich_service_debug_artifacts"] is False
    assert summary["future_narrow_sidecar_target"] == "minimal_public_topology_subset"
    assert summary["room_scoped_runtime_state_json"] is None
    assert summary["working_topology_json"] is None
    assert summary["working_vs_committed_topology_report_json"] is None
    assert summary["working_vs_committed_topology_timeline_json"] is None
    assert summary["room_commit_diagnosis_json"] is None
    assert summary["full_final_vector_map_path"] is None
    assert sorted(summary["rich_service_debug_artifact_keys_suppressed"]) == [
        "full_vector_map_snapshot_json",
        "room_commit_diagnosis_json",
        "room_commit_diagnosis_md",
        "room_scoped_runtime_state_json",
        "working_topology_json",
        "working_vs_committed_topology_report_json",
        "working_vs_committed_topology_timeline_json",
        "working_vs_committed_topology_timeline_md",
    ]
    assert scene_root.joinpath("logs", "topology_v0_1.json").exists()
    assert scene_root.joinpath("logs", "topology_query_report.json").exists()
    assert scene_root.joinpath("logs", "committed_room_world_model_v0_1.json").exists()
    assert scene_root.joinpath("logs", "committed_room_world_snapshot_v0_1.json").exists()
    assert scene_root.joinpath("logs", "online_topology_lifecycle_v0_1.json").exists()
    assert not scene_root.joinpath("logs", "room_scoped_runtime_state_v0_1.json").exists()
    assert not scene_root.joinpath("logs", "working_topology_v0_1.json").exists()
    assert not scene_root.joinpath("logs", "room_commit_diagnosis_v0_1.json").exists()
    assert [room["id"] for room in topology_payload["rooms"]] == ["room_1"]
    assert diagnostics_payload["summary"]["published_room_count"] == 1
    assert float(summary["finalize_export_timing_sec"]["rich_service_debug_artifact_materialization_sec"]) == 0.0
