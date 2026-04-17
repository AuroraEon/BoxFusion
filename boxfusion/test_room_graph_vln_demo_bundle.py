import json
from pathlib import Path

from boxfusion.room_graph_vln_demo_bundle import build_room_graph_vln_demo_bundle


def _write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def test_room_graph_vln_demo_bundle_writes_index_and_queries(tmp_path: Path) -> None:
    scene_root = tmp_path / "mock_scene"
    logs_dir = scene_root / "logs"
    topology_payload = {
        "version": "0.1",
        "sequence_id": "mock_bundle_scene",
        "floors": [{"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1}],
        "rooms": [
            {
                "id": "room_1",
                "room_type": "entry",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 1,
                "center": [0.0, 0.0],
                "polygon": [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
            },
            {
                "id": "room_2",
                "room_type": "lounge",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 2,
                "center": [3.0, 0.0],
                "polygon": [[2.0, -1.0], [4.0, -1.0], [4.0, 1.0], [2.0, 1.0]],
            },
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "transition",
                "confidence": 1.0,
                "status": "confirmed",
                "support_count": 1,
                "metadata": {},
            }
        ],
        "indices": {"object_to_room": {}, "anchor_to_room": {}, "room_to_objects": {}, "room_to_anchors": {}},
        "entities": {"objects": [], "anchors": []},
        "evidences": [],
        "metadata": {},
    }
    world_model_payload = {
        "version": "0.1",
        "sequence_id": "mock_bundle_scene",
        "rooms": [
            {
                "room_id": "room_1",
                "room_type": "entry",
                "floor_id": "floor_1",
                "semantic_summary": {"object_label_counts": {"door": 1}, "dominant_object_labels": ["door"]},
            },
            {
                "room_id": "room_2",
                "room_type": "lounge",
                "floor_id": "floor_1",
                "semantic_summary": {"object_label_counts": {"couch": 2}, "dominant_object_labels": ["couch"]},
            },
        ],
    }
    summary_payload = {
        "sequence_id": "mock_bundle_scene",
        "topology_v0_1_json": str(logs_dir / "topology_v0_1.json"),
        "committed_room_world_model_json": str(logs_dir / "committed_room_world_model_v0_1.json"),
    }
    _write_json(logs_dir / "topology_v0_1.json", topology_payload)
    _write_json(logs_dir / "committed_room_world_model_v0_1.json", world_model_payload)
    _write_json(logs_dir / "summary.json", summary_payload)

    bundle_manifest = build_room_graph_vln_demo_bundle(
        spec={
            "title": "Mock Bundle",
            "subtitle": "Presentation bundle smoke test.",
            "recommended_semantic_whitelist": ["couch"],
            "scenes": [
                {
                    "scene_root": str(scene_root),
                    "display_name": "Mock Scene",
                    "role": "primary",
                    "summary": "Small committed/public demo scene.",
                    "recommended_semantic_whitelist": ["couch"],
                    "queries": [
                        {
                            "slug": "room_target_room_2",
                            "title": "Explicit room target",
                            "explanation": "Shows room-to-room routing.",
                            "start_room": "room_1",
                            "goal_room": "room_2",
                        },
                        {
                            "slug": "semantic_couch",
                            "title": "Semantic couch target",
                            "explanation": "Shows semantic-room-summary routing.",
                            "start_room": "room_1",
                            "semantic_target": "couch",
                        },
                    ],
                }
            ],
        },
        output_root=tmp_path / "bundle_out",
    )

    assert bundle_manifest["title"] == "Mock Bundle"
    scene_dir = tmp_path / "bundle_out" / "mock_scene"
    assert (tmp_path / "bundle_out" / "index.html").exists()
    assert (tmp_path / "bundle_out" / "demo_bundle_manifest.json").exists()
    assert (scene_dir / "index.html").exists()
    assert (scene_dir / "scene_bundle_summary.json").exists()
    assert (scene_dir / "mock_bundle_scene_room_graph_vln_demo_01_room_target_room_2.html").exists()
    assert (scene_dir / "mock_bundle_scene_room_graph_vln_demo_02_semantic_couch.json").exists()
