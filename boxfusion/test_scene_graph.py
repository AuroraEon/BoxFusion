import os
import sys
from typing import List

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-boxfusion")

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.scene_graph_builder import ObjectNode, RoomNode, SemanticObservation, SemanticSceneGraph


def _build_mock_scene_graph() -> SemanticSceneGraph:
    sg = SemanticSceneGraph()

    room1 = RoomNode(
        id="room_1",
        room_type="living_room",
        polygon=[(0, 0), (5, 0), (5, 5), (0, 5)],
    )
    room2 = RoomNode(
        id="room_2",
        room_type="bedroom",
        polygon=[(5, 0), (10, 0), (10, 5), (5, 5)],
    )
    sg.add_room_node(room1)
    sg.add_room_node(room2)

    table = ObjectNode(
        id="obj_101",
        center=(2.5, 2.5, 0.4),
        bbox=(1.2, 0.8, 0.8),
        label="desk",
        category="furniture",
        clip_feature=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        confidence=0.62,
        room_id="room_1",
    )
    apple = ObjectNode(
        id="obj_102",
        center=(2.5, 2.5, 0.85),
        bbox=(0.1, 0.1, 0.1),
        label="apple",
        category="food",
        clip_feature=np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32),
        confidence=0.88,
        room_id="room_1",
    )
    chair = ObjectNode(
        id="obj_103",
        center=(1.4, 2.4, 0.25),
        bbox=(0.5, 0.5, 0.5),
        label="chair",
        category="furniture",
        clip_feature=np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32),
        confidence=0.91,
        room_id="room_1",
    )
    sofa = ObjectNode(
        id="obj_201",
        center=(7.5, 2.5, 0.3),
        bbox=(2.0, 1.0, 0.6),
        label="couch",
        category="furniture",
        clip_feature=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        confidence=0.81,
        room_id="room_2",
    )

    for obj in [table, apple, chair, sofa]:
        sg.add_object_node(obj)
        sg.add_inside_relation(obj.id, obj.room_id)

    sg.fuse_object_observation(
        "obj_101",
        SemanticObservation(
            label="table",
            category="furniture",
            clip_feature=np.array([0.7, 0.3, 0.0, 0.0], dtype=np.float32),
            detection_confidence=0.94,
            semantic_confidence=0.93,
            semantic_gap=0.22,
            association_confidence=0.95,
        ),
    )
    sg.fuse_object_observation(
        "obj_201",
        SemanticObservation(
            label="sofa",
            category="furniture",
            clip_feature=np.array([0.0, 0.0, 0.2, 0.8], dtype=np.float32),
            detection_confidence=0.92,
            semantic_confidence=0.89,
            semantic_gap=0.18,
            association_confidence=0.98,
        ),
    )

    sg.compute_spatial_relations(dist_threshold=1.5, z_tolerance=0.15)
    return sg


def _validate_semantic_fusion(sg: SemanticSceneGraph) -> None:
    table = sg.object_index["obj_101"]
    sofa = sg.object_index["obj_201"]
    apple = sg.object_index["obj_102"]

    assert table.obs_count == 2, "table should have two fused observations"
    assert table.canonical_label == "table", "desk/table fusion should normalize toward table"
    assert table.canonical_category == "furniture"
    assert table.label_scores["table"] > table.label_scores["desk"], "weighted fusion should beat the weaker observation"
    assert table.semantic_stability > 0.0
    assert table.clip_feature_fused is not None and table.clip_feature_fused.shape == (4,)

    assert sofa.obs_count == 2, "sofa should have two fused observations"
    assert sofa.canonical_label == "sofa", "couch should normalize to sofa"
    assert sofa.semantic_stability > 0.0

    assert apple.canonical_label == "apple"
    assert apple.obs_count == 1


def _validate_anchor_layer(sg: SemanticSceneGraph) -> List[str]:
    anchors = sg.build_anchor_layer(debug=True)
    anchor_ids = sorted(anchor.id for anchor in anchors)

    assert "anchor_room_1" in anchor_ids
    assert "anchor_room_2" in anchor_ids
    assert "anchor_obj_101" in anchor_ids, "table should be anchor-worthy"
    assert "anchor_obj_201" in anchor_ids, "sofa should be anchor-worthy"
    assert "anchor_obj_102" not in anchor_ids, "apple should be excluded from object anchors"

    for anchor in anchors:
        valid, _ = sg.validate_anchor(anchor.position, anchor.room_id, anchor.target_id, anchor.anchor_type)
        assert valid, f"{anchor.id} must pass anchor validation"
        assert sg.graph.has_edge(anchor.id, anchor.room_id)
        assert "IN_ROOM" in sg.get_relations_between(anchor.id, anchor.room_id)
        assert sg.graph.has_edge(anchor.id, anchor.target_id)
        assert "FOR" in sg.get_relations_between(anchor.id, anchor.target_id)

    return anchor_ids


def run_mock_test() -> None:
    print("Running scene graph validation...")
    sg = _build_mock_scene_graph()

    _validate_semantic_fusion(sg)
    anchor_ids = _validate_anchor_layer(sg)

    os.makedirs("./debug_mock", exist_ok=True)
    vis_path = "./debug_mock/mock_scene_graph_bev.png"
    sg.visualize_bev_graph(save_path=vis_path, show_anchor_candidates=True)

    print("Semantic fusion validated.")
    print(f"Anchors: {anchor_ids}")
    print(f"BEV visualization: {vis_path}")


if __name__ == "__main__":
    run_mock_test()
