from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from boxfusion.room_commit_diagnosis import build_room_commit_diagnosis, load_scene_artifacts
from boxfusion.stage_a_demo import ClosedLoopDemoRecorder


def _room(
    room_id: int,
    *,
    floor_id: str = "floor_1",
    polygon: Optional[List[List[float]]] = None,
) -> Dict[str, Any]:
    polygon = polygon or [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    return {
        "id": int(room_id),
        "room_id": f"room_{int(room_id)}",
        "floor_id": floor_id,
        "room_type": "unknown",
        "status": "confirmed",
        "center": [
            round(sum(point[0] for point in polygon) / len(polygon), 3),
            round(sum(point[1] for point in polygon) / len(polygon), 3),
        ],
        "polygon": polygon,
        "area_m2": 4.0,
    }


def _vector_map(
    *,
    rooms: List[Dict[str, Any]],
    gateways: Optional[List[Dict[str, Any]]] = None,
    objects: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
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


def _build_scene_artifacts(
    *,
    lifecycle_payload: Dict[str, Any],
    runtime_state_payload: Optional[Dict[str, Any]] = None,
    topology_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "input_path": "synthetic",
        "scene_root": "synthetic_scene",
        "log_dir": "synthetic_scene/logs",
        "summary_path": None,
        "lifecycle_path": "synthetic_scene/logs/online_topology_lifecycle_v0_1.json",
        "runtime_state_path": None if runtime_state_payload is None else "synthetic_scene/logs/room_scoped_runtime_state_v0_1.json",
        "topology_path": None if topology_payload is None else "synthetic_scene/logs/topology_v0_1.json",
        "topology_query_report_path": None,
        "committed_room_world_model_path": None,
        "summary_payload": None,
        "lifecycle_payload": lifecycle_payload,
        "runtime_state_payload": runtime_state_payload,
        "topology_payload": topology_payload,
        "topology_query_report_payload": None,
        "committed_room_world_model_payload": None,
    }


def test_room_commit_diagnosis_reports_successful_public_room_for_minimal_replay(tmp_path: Path) -> None:
    recorder = ClosedLoopDemoRecorder(
        output_root=str(tmp_path),
        sequence_id="commit_success_case",
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

    outputs = recorder.finalize({"processed_frames": 3, "duration_sec": 2.0, "average_fps": 1.5})
    scene_root = Path(outputs["output_root"])
    diagnosis_payload = json.loads(scene_root.joinpath("logs", "room_commit_diagnosis_v0_1.json").read_text(encoding="utf-8"))

    assert diagnosis_payload["diagnosis"]["category"] == "public_committed_rooms_available"
    assert diagnosis_payload["counts"]["runtime_internal_committed_room_count"] == 1
    assert diagnosis_payload["counts"]["public_topology_room_count"] == 1
    assert diagnosis_payload["counts"]["candidate_complete_ever_count"] >= 1
    assert diagnosis_payload["room_transition_evidence"]["room_transition_event_count"] >= 1


def test_room_commit_diagnosis_reports_short_single_room_replay_as_never_candidate_complete(tmp_path: Path) -> None:
    recorder = ClosedLoopDemoRecorder(
        output_root=str(tmp_path),
        sequence_id="commit_failure_short_single_room",
        capture_stride_frames=1,
        core_only=True,
    )

    image = np.zeros((4, 4, 3), dtype=np.uint8)
    vector_map = _vector_map(rooms=[_room(1)])

    recorder.record_snapshot(
        frame_idx=0,
        timestamp=0.0,
        image_rgb=image,
        pose=_pose(1.0),
        trajectory_xy=[(1.0, 1.0)],
        vector_map=vector_map,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=0,
        export_profile={"changed_room_ids": "room_1"},
    )
    recorder.record_snapshot(
        frame_idx=10,
        timestamp=1.0,
        image_rgb=image,
        pose=_pose(1.2),
        trajectory_xy=[(1.0, 1.0), (1.2, 1.0)],
        vector_map=vector_map,
        tracking_report={},
        segmentation_updated=False,
        segmentation_cycle_idx=1,
        export_profile={},
    )

    outputs = recorder.finalize({"processed_frames": 2, "duration_sec": 1.0, "average_fps": 2.0})
    scene_root = Path(outputs["output_root"])
    diagnosis_payload = json.loads(scene_root.joinpath("logs", "room_commit_diagnosis_v0_1.json").read_text(encoding="utf-8"))

    assert diagnosis_payload["diagnosis"]["category"] == "candidates_seen_but_never_candidate_complete"
    assert diagnosis_payload["counts"]["candidate_room_formed_count"] >= 1
    assert diagnosis_payload["counts"]["candidate_complete_ever_count"] == 0
    assert diagnosis_payload["counts"]["runtime_internal_committed_room_count"] == 0
    assert diagnosis_payload["room_transition_evidence"]["room_transition_event_count"] == 0


def test_room_commit_diagnosis_separates_no_candidate_and_blocked_commit_cases() -> None:
    empty_lifecycle = {
        "sequence_id": "no_candidates_case",
        "rooms": [],
        "refresh_history": [],
        "committed_rooms": [],
    }
    no_candidates = build_room_commit_diagnosis(_build_scene_artifacts(lifecycle_payload=empty_lifecycle))
    assert no_candidates["diagnosis"]["category"] == "no_useful_room_candidates"

    blocked_lifecycle = {
        "sequence_id": "blocked_after_candidate_complete",
        "rooms": [
            {
                "room_id": "room_1",
                "lifecycle_state": "candidate_complete",
                "candidate_complete": True,
                "commit_block_reasons": ["gateway_structure_not_stable"],
                "publication_state": "FINALIZATION_PENDING",
                "candidate_room_formed_v1": True,
                "leave_like_signal_v1": True,
                "leave_like_signal_v1_source": "departed_active_room",
                "export_observation_count": 3,
            }
        ],
        "refresh_history": [
            {
                "frame_idx": 10,
                "timestamp": 1.0,
                "active_room_id": "room_2",
                "rooms": [
                    {
                        "room_id": "room_1",
                        "lifecycle_state": "candidate_complete",
                        "candidate_complete": True,
                        "commit_block_reasons": ["gateway_structure_not_stable"],
                        "candidate_block_reasons": [],
                        "publication_state": "FINALIZATION_PENDING",
                        "candidate_room_formed_v1": True,
                        "leave_like_signal_v1": True,
                        "leave_like_signal_v1_source": "departed_active_room",
                        "finalization_blockers": ["gateway_structure_not_stable"],
                        "publication_blockers": [],
                        "commit_ready": False,
                        "published": False,
                        "export_observation_count": 3,
                    }
                ],
            }
        ],
        "committed_rooms": [],
    }
    blocked = build_room_commit_diagnosis(_build_scene_artifacts(lifecycle_payload=blocked_lifecycle))
    assert blocked["diagnosis"]["category"] == "candidate_complete_reached_but_commit_blocked"
    assert blocked["counts"]["candidate_complete_ever_count"] == 1
    assert blocked["counts"]["runtime_internal_committed_room_count"] == 0
    assert blocked["quick_readiness_diagnosis"]["top_commit_blockers_after_candidate_complete"] == [
        "gateway_structure_not_stable"
    ]


def test_room_commit_diagnosis_detects_internal_commit_filtered_from_public() -> None:
    lifecycle_payload = {
        "sequence_id": "commit_filtered_case",
        "rooms": [
            {
                "room_id": "room_1",
                "lifecycle_state": "committed",
                "candidate_complete": False,
                "commit_block_reasons": [],
                "publication_state": "PUBLISHED",
                "candidate_room_formed_v1": True,
                "leave_like_signal_v1": True,
                "leave_like_signal_v1_source": "departed_active_room",
                "export_observation_count": 3,
            }
        ],
        "refresh_history": [
            {
                "frame_idx": 20,
                "timestamp": 2.0,
                "active_room_id": "room_2",
                "rooms": [
                    {
                        "room_id": "room_1",
                        "lifecycle_state": "committed",
                        "candidate_complete": False,
                        "commit_block_reasons": [],
                        "candidate_block_reasons": [],
                        "publication_state": "PUBLISHED",
                        "candidate_room_formed_v1": True,
                        "leave_like_signal_v1": True,
                        "leave_like_signal_v1_source": "departed_active_room",
                        "finalization_blockers": [],
                        "publication_blockers": [],
                        "commit_ready": True,
                        "published": True,
                        "export_observation_count": 3,
                    }
                ],
            }
        ],
        "committed_rooms": ["room_1"],
    }
    runtime_state_payload = {
        "committed_global_memory": {"room_ids": ["room_1"]},
    }
    topology_payload = {
        "rooms": [],
        "metadata": {"public_topology_meaning": "committed/published only"},
    }

    filtered = build_room_commit_diagnosis(
        _build_scene_artifacts(
            lifecycle_payload=lifecycle_payload,
            runtime_state_payload=runtime_state_payload,
            topology_payload=topology_payload,
        )
    )
    assert filtered["diagnosis"]["category"] == "commit_succeeded_but_public_export_or_filter_removed_everything"
    assert filtered["counts"]["runtime_internal_committed_room_count"] == 1
    assert filtered["counts"]["public_topology_room_count"] == 0
    assert filtered["room_sets"]["committed_rooms_missing_from_public_filter"] == ["room_1"]


def test_room_commit_diagnosis_loader_reads_generated_scene_outputs(tmp_path: Path) -> None:
    recorder = ClosedLoopDemoRecorder(
        output_root=str(tmp_path),
        sequence_id="loader_case",
        capture_stride_frames=1,
        core_only=True,
    )
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    vector_map = _vector_map(rooms=[_room(1)])
    recorder.record_snapshot(
        frame_idx=0,
        timestamp=0.0,
        image_rgb=image,
        pose=_pose(1.0),
        trajectory_xy=[(1.0, 1.0)],
        vector_map=vector_map,
        tracking_report={},
        segmentation_updated=True,
        segmentation_cycle_idx=0,
        export_profile={"changed_room_ids": "room_1"},
    )
    recorder.finalize({"processed_frames": 1, "duration_sec": 0.0, "average_fps": 1.0})

    scene_root = tmp_path / "loader_case"
    loaded = load_scene_artifacts(scene_root / "logs")
    assert loaded["lifecycle_payload"]["sequence_id"] == "loader_case"
