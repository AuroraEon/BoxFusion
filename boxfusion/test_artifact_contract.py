import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.artifact_contract import (
    ARTIFACT_PROFILE_CORE_ONLY,
    ARTIFACT_PROFILE_FULL,
    TIMELINE_ROW_KIND_SNAPSHOT,
    with_timeline_row_contract,
)
from boxfusion.backend_eval_scaffold import write_scene_manifest
from boxfusion.replay_timeline import load_replay_observations


class _FakeTopology:
    def __init__(self, rooms: Dict[str, Dict[str, Any]]) -> None:
        self._rooms = {str(key): dict(value) for key, value in rooms.items()}

    def get_room(self, room_id: str) -> Dict[str, Any]:
        return dict(self._rooms.get(str(room_id)) or {})


class _FakeQueryAPI:
    def __init__(self, rooms: Dict[str, Dict[str, Any]]) -> None:
        self.topology = _FakeTopology(rooms)


def _topology_payload() -> Dict[str, Any]:
    return {
        "version": "0.1",
        "sequence_id": "mock_artifact_contract",
        "floors": [
            {"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
        "rooms": [
            {"id": "room_1", "floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1, "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]]},
            {"id": "room_2", "floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2, "polygon": [[2, 0], [3, 0], [3, 1], [2, 1]]},
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "vertical_transition",
                "confidence": 0.9,
                "support_count": 1,
                "metadata": {"transition_ids": ["vt_1"]},
            }
        ],
        "indices": {
            "object_to_room": {},
            "anchor_to_room": {},
            "room_to_objects": {"room_1": [], "room_2": []},
            "room_to_anchors": {"room_1": [], "room_2": []},
        },
        "entities": {"objects": [], "anchors": []},
        "evidences": [],
        "metadata": {},
    }


def _snapshot_timeline_rows() -> List[Dict[str, Any]]:
    return [
        with_timeline_row_contract(
            {
                "row_type": "snapshot",
                "frame_idx": 0,
                "timestamp": 0.0,
                "replay_frame_idx": 0,
                "snapshot_idx": 0,
                "current_room_id": "room_1",
                "rgb_path": "rgb_000000.jpg",
                "vector_map_path": "vector_map_000000.json",
            },
            row_kind=TIMELINE_ROW_KIND_SNAPSHOT,
            replay_mode="snapshot_only",
        ),
        with_timeline_row_contract(
            {
                "row_type": "snapshot",
                "frame_idx": 10,
                "timestamp": 1.0,
                "replay_frame_idx": 1,
                "snapshot_idx": 1,
                "current_room_id": "room_2",
                "rgb_path": "rgb_000010.jpg",
                "vector_map_path": "vector_map_000010.json",
            },
            row_kind=TIMELINE_ROW_KIND_SNAPSHOT,
            replay_mode="snapshot_only",
        ),
    ]


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_scene_root(scene_root: Path, *, artifact_profile: str, include_timeline: bool) -> Path:
    logs = scene_root / "logs"
    logs.mkdir(parents=True, exist_ok=True)

    _write_json(logs / "topology_v0_1.json", _topology_payload())
    _write_json(
        logs / "topology_query_report.json",
        {"room_count": 2, "object_count": 0, "anchor_count": 0, "edge_count": 1, "floor_count": 2},
    )
    _write_json(logs / "vertical_transition_evidence.json", {"summary": {"count": 1}, "transitions": [{"transition_id": "vt_1"}]})
    _write_json(logs / "floor_diagnostics_summary.json", {"per_floor": [{"floor_id": "floor_1"}, {"floor_id": "floor_2"}]})
    _write_json(logs / "runtime_growth_profile.json", {"records": [], "summary": {"sample_count": 0}})
    (logs / "runtime_growth_profile.csv").write_text("frame_idx,total_step_sec\n", encoding="utf-8")
    _write_json(
        logs / "online_topology_lifecycle_v0_1.json",
        {
            "sequence_id": scene_root.name,
            "frame_idx": 10,
            "timestamp": 1.0,
            "summary": {"committed_room_count": 2},
            "rooms": [{"room_id": "room_1"}, {"room_id": "room_2"}],
            "refresh_history": [{"frame_idx": 0, "timestamp": 0.0, "rooms": [{"room_id": "room_1"}]}],
        },
    )
    _write_json(logs / "working_topology_v0_1.json", {"metadata": {"artifact_kind": "working_topology_debug"}})
    _write_json(logs / "working_vs_committed_topology_report_v0_1.json", {"artifact_kind": "working_vs_committed_topology_report_debug"})
    _write_json(logs / "working_vs_committed_topology_timeline_v0_1.json", {"artifact_kind": "working_vs_committed_topology_timeline_debug"})
    (logs / "working_vs_committed_topology_timeline_v0_1.md").write_text("# mock\n", encoding="utf-8")

    if include_timeline:
        timeline_rows = _snapshot_timeline_rows()
        _write_json(logs / "timeline.json", timeline_rows)
        (logs / "timeline.csv").write_text("row_kind,frame_idx,current_room_id\nsnapshot_frame,0,room_1\nsnapshot_frame,10,room_2\n", encoding="utf-8")

    _write_json(
        logs / "summary.json",
        {
            "sequence_id": scene_root.name,
            "output_mode": "core_only" if artifact_profile == ARTIFACT_PROFILE_CORE_ONLY else "full",
            "artifact_profile": artifact_profile,
            "core_only_mode": artifact_profile == ARTIFACT_PROFILE_CORE_ONLY,
            "optional_demo_artifacts_enabled": include_timeline,
            "snapshot_count": 2,
            "replay_mode": "snapshot_only",
            "runtime_growth_summary": {"sample_count": 0},
        },
    )
    return scene_root


def test_snapshot_only_timeline_loads_for_replay_consumers(tmp_path: Path) -> None:
    query_api = _FakeQueryAPI(
        {
            "room_1": {"floor_id": "floor_1", "display_floor_id": "floor_1"},
            "room_2": {"floor_id": "floor_2", "display_floor_id": "floor_2"},
        }
    )
    timeline_path = tmp_path / "timeline.json"
    _write_json(timeline_path, _snapshot_timeline_rows())

    observations = load_replay_observations(timeline_path, query_api)

    assert [item["current_room_id"] for item in observations] == ["room_1", "room_2"]
    assert all(item["row_kind"] == TIMELINE_ROW_KIND_SNAPSHOT for item in observations)
    assert observations[0]["current_floor_label"] == "floor_1"
    assert observations[1]["current_floor_label"] == "floor_2"


def test_core_only_manifest_declares_non_replay_capabilities_clearly(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "00843-DYehNKdT76V", artifact_profile=ARTIFACT_PROFILE_CORE_ONLY, include_timeline=False)

    manifest = write_scene_manifest(scene_root, sequence_name=scene_root.name)
    capabilities = manifest["artifact_capabilities"]

    assert manifest["artifact_profile"] == ARTIFACT_PROFILE_CORE_ONLY
    assert capabilities["final_state_query"] is True
    assert capabilities["final_state_route"] is True
    assert capabilities["final_state_eval"] is True
    assert capabilities["query_bundle_rebuild"] is True
    assert capabilities["checkpoint_replay"] is False
    assert capabilities["dense_replay"] is False
    assert capabilities["lifecycle_history"] is True
    assert capabilities["working_topology_history"] is True


def test_manifest_separates_public_working_and_lifecycle_surfaces(tmp_path: Path) -> None:
    scene_root = _write_scene_root(tmp_path / "00829-QaLdnwvtxbs", artifact_profile=ARTIFACT_PROFILE_FULL, include_timeline=True)

    manifest = write_scene_manifest(scene_root, sequence_name=scene_root.name)
    artifacts = manifest["artifacts"]
    timeline_contract = manifest["artifact_contract"]["timeline_contract"]
    capabilities = manifest["artifact_capabilities"]

    assert artifacts["topology_json"]["artifact_surface"] == "public"
    assert artifacts["timeline_json"]["artifact_surface"] == "public"
    assert artifacts["working_topology_json"]["artifact_surface"] == "working"
    assert artifacts["working_vs_committed_topology_timeline_json"]["artifact_surface"] == "working"
    assert artifacts["online_topology_lifecycle_json"]["artifact_surface"] == "lifecycle"
    assert artifacts["online_topology_lifecycle_json"]["artifact_semantics"] == "diagnostic_history"
    assert timeline_contract["row_kinds_present"] == ["snapshot_frame"]
    assert timeline_contract["timeline_frame_count"] == 2
    assert capabilities["checkpoint_replay"] is True
    assert capabilities["dense_replay"] is False


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        test_snapshot_only_timeline_loads_for_replay_consumers(tmp_path / "replay")
        test_core_only_manifest_declares_non_replay_capabilities_clearly(tmp_path / "core_only")
        test_manifest_separates_public_working_and_lifecycle_surfaces(tmp_path / "full_artifact")
    print("artifact contract ok")
