import json
from pathlib import Path

from boxfusion.room_graph_vln_demo import RoomGraphVLNDemo


def _write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def test_room_graph_vln_demo_room_and_semantic_queries(tmp_path: Path) -> None:
    scene_root = tmp_path / "mock_scene"
    logs_dir = scene_root / "logs"
    topology_payload = {
        "version": "0.1",
        "sequence_id": "mock_room_graph_demo",
        "floors": [
            {"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
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
                "room_type": "hall",
                "floor_id": "floor_1",
                "display_floor_id": "floor_1",
                "display_order": 2,
                "center": [3.0, 0.0],
                "polygon": [[2.0, -1.0], [4.0, -1.0], [4.0, 1.0], [2.0, 1.0]],
            },
            {
                "id": "room_3",
                "room_type": "bedroom",
                "floor_id": "floor_2",
                "display_floor_id": "floor_2",
                "display_order": 3,
                "center": [6.0, 0.0],
                "polygon": [[5.0, -1.0], [7.0, -1.0], [7.0, 1.0], [5.0, 1.0]],
            },
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "transition",
                "confidence": 0.2,
                "status": "weak",
                "support_count": 1,
                "evidence_ids": ["ev_t"],
                "metadata": {"evidence_type_breakdown": {"trajectory_transition": 1}},
            },
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "possible_connection",
                "confidence": 0.2,
                "status": "weak",
                "support_count": 1,
                "evidence_ids": ["ev_p"],
                "metadata": {"evidence_type_breakdown": {"boundary_contact": 1}},
            },
            {
                "source": "room_2",
                "target": "room_3",
                "relation_type": "vertical_transition",
                "confidence": 0.9,
                "status": "confirmed",
                "support_count": 1,
                "evidence_ids": ["ev_v"],
                "metadata": {"transition_ids": ["vt_1"], "evidence_type_breakdown": {"vertical_transition_observation": 1}},
            },
        ],
        "indices": {"object_to_room": {}, "anchor_to_room": {}, "room_to_objects": {}, "room_to_anchors": {}},
        "entities": {"objects": [], "anchors": []},
        "evidences": [
            {"evidence_id": "ev_t", "evidence_type": "trajectory_transition", "source_ref": "trajectory_transition"},
            {"evidence_id": "ev_p", "evidence_type": "boundary_contact", "source_ref": "boundary_contact"},
            {"evidence_id": "ev_v", "evidence_type": "vertical_transition_observation", "source_ref": "vertical_transition"},
        ],
        "metadata": {},
    }
    world_model_payload = {
        "version": "0.1",
        "sequence_id": "mock_room_graph_demo",
        "rooms": [
            {
                "room_id": "room_1",
                "room_type": "entry",
                "floor_id": "floor_1",
                "semantic_summary": {"object_label_counts": {"door": 1}, "dominant_object_labels": ["door"]},
                "bev_vln_hook": {"semantic_landmarks": ["door"]},
            },
            {
                "room_id": "room_2",
                "room_type": "hall",
                "floor_id": "floor_1",
                "semantic_summary": {"object_label_counts": {"stairs": 1}, "dominant_object_labels": ["stairs"]},
                "bev_vln_hook": {"semantic_landmarks": ["stairs"]},
            },
            {
                "room_id": "room_3",
                "room_type": "bedroom",
                "floor_id": "floor_2",
                "semantic_summary": {"object_label_counts": {"bed": 2, "pillow": 1}, "dominant_object_labels": ["bed"]},
                "bev_vln_hook": {"semantic_landmarks": ["bed", "pillow"]},
            },
        ],
    }
    summary_payload = {
        "sequence_id": "mock_room_graph_demo",
        "topology_v0_1_json": str(logs_dir / "topology_v0_1.json"),
        "committed_room_world_model_json": str(logs_dir / "committed_room_world_model_v0_1.json"),
    }
    _write_json(logs_dir / "topology_v0_1.json", topology_payload)
    _write_json(logs_dir / "committed_room_world_model_v0_1.json", world_model_payload)
    _write_json(logs_dir / "summary.json", summary_payload)
    audit_dir = tmp_path / "audit"
    audit_dir.mkdir(parents=True, exist_ok=True)
    (audit_dir / "topology_spurious_edge_candidates.csv").write_text(
        "\n".join(
            [
                "scene_id,edge_index,source_room,target_room,relation_type,candidate_type,confidence,support_count,evidence_id_count,has_gateway_match,has_vertical_transition_match,severity,notes",
                "mock_room_graph_demo,0,room_1,room_2,transition,low_support_edge,0.2,1,1,false,false,warning,selected weak route relation",
                "mock_room_graph_demo,1,room_1,room_2,possible_connection,low_support_edge,0.2,1,1,false,false,warning,same-pair alternate relation",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (audit_dir / "topology_missing_edge_candidates.csv").write_text(
        "\n".join(
            [
                "scene_id,room_a,room_b,floor_a,floor_b,candidate_type,evidence_source,distance_or_gap,has_gateway_record,has_vertical_transition_record,has_world_model_neighbor_relation,has_topology_edge,severity,notes",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    demo = RoomGraphVLNDemo.from_inputs(scene_root=scene_root)
    explicit_result = demo.build_demo(start_room="room_1", goal_room="bedroom")
    assert explicit_result["ok"] is True
    assert explicit_result["goal_resolution"]["resolved_room_id"] == "room_3"
    assert explicit_result["route"]["room_sequence"] == ["room_1", "room_2", "room_3"]
    assert explicit_result["next_hop"]["room_id"] == "room_2"

    semantic_result = demo.build_demo(start_room="room_1", semantic_target="bed")
    assert semantic_result["ok"] is True
    assert semantic_result["goal_resolution"]["resolved_room_id"] == "room_3"
    assert semantic_result["goal_resolution"]["candidate_matches"][0]["room_id"] == "room_3"

    refined_result = demo.build_demo(
        start_room="room_1",
        goal_room="room_3",
        include_route_edge_explanation=True,
        include_audit_overlays=True,
        audit_dir=audit_dir,
    )
    first_edge = refined_result["route_edge_explanation"][0]
    assert first_edge["relation_type"] == "transition"
    assert "trajectory_supported_transition" in first_edge["selected_relation_semantic_labels"]
    assert first_edge["same_pair_alternate_relation_count"] == 1
    assert any(
        row["candidate_label"] == "route_selected_weak_relation"
        for row in refined_result["audit_overlay_records"]
    )
    assert any(
        row["candidate_label"] == "same_pair_alternate_relation_visible"
        for row in refined_result["audit_overlay_records"]
    )

    html_payload = demo.render_html(semantic_result)
    assert "Spatial Public Room Graph" in html_payload
    assert "room_3" in html_payload
    refined_html = demo.render_html(refined_result)
    assert "Legend and Semantics" in refined_html
    assert "same-pair alternate relations" in refined_html
