import os
import sys
from typing import Any, Dict, List

import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.dynamic_room_segmenter import DynamicRoomSegmenter
from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
from boxfusion.floor_manager import FloorManager
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopologyBuilder
from boxfusion.scene_graph_builder import FloorNode, ObjectNode, RoomNode, SemanticSceneGraph


class _MockBoxes3D:
    def __init__(self, tensor: List[List[float]]) -> None:
        self.tensor = _MockTensor(tensor, dtype=np.float32)


class _MockTensor:
    def __init__(self, array: Any, dtype: Any) -> None:
        self._array = np.asarray(array, dtype=dtype)

    def cpu(self) -> "_MockTensor":
        return self

    def detach(self) -> "_MockTensor":
        return self

    def numpy(self) -> np.ndarray:
        return np.asarray(self._array)


class _MockInstances:
    def __init__(
        self,
        *,
        boxes: List[List[float]],
        categories: List[str],
        instance_ids: List[int],
        scores: List[float],
    ) -> None:
        self.pred_boxes_3d = _MockBoxes3D(boxes)
        self.categories = np.asarray(categories, dtype=object)
        self.init_id = _MockTensor(instance_ids, dtype=np.int64)
        self.scores = _MockTensor(scores, dtype=np.float32)
        self.embeddings = _MockTensor(
            [[float(instance_id), float(instance_id) + 0.5] for instance_id in instance_ids],
            dtype=np.float32,
        )
        self.semantic_confidences = _MockTensor(scores, dtype=np.float32)
        self.association_confidences = _MockTensor(np.ones(len(instance_ids), dtype=np.float32), dtype=np.float32)
        self.semantic_gaps = _MockTensor(np.zeros(len(instance_ids), dtype=np.float32), dtype=np.float32)
        self.view_qualities = _MockTensor(np.ones(len(instance_ids), dtype=np.float32), dtype=np.float32)


def _build_export_ready_segmenter() -> FloorAwareRoomSegmenter:
    segmenter = FloorAwareRoomSegmenter(
        resolution=0.05,
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        },
    )

    def pose(z: float) -> np.ndarray:
        mat = np.eye(4, dtype=np.float32)
        mat[2, 3] = float(z)
        return mat

    segmenter.observe_frame(frame_idx=0, timestamp=0.0, pose_matrix=pose(0.0), points_xyzrgb=None, is_keyframe=True)
    segmenter.observe_frame(frame_idx=1, timestamp=0.1, pose_matrix=pose(0.05), points_xyzrgb=None, is_keyframe=True)
    floor_state = segmenter._get_or_create_floor_state("floor_1")
    floor_state.last_segmentation_frame_idx = 10
    floor_state.local_to_world_room_id = {1: 1}
    floor_state.segmenter.last_room_markers = np.array([[1, 1], [1, 2]], dtype=np.int32)
    floor_state.segmenter.label_to_global = {1: 1}
    floor_state.segmenter.last_gateways = []
    floor_state.segmenter._grid_to_world = lambda u, v: np.array([float(u), float(v)], dtype=np.float32)
    floor_state.segmenter._vote_room_label_for_object = lambda *_args, **_kwargs: (1, 1, {1: 9})
    segmenter.last_room_markers = True
    segmenter.last_floor_observation = {"floor_id": "floor_1", "status": "stable"}
    return segmenter


def _build_two_room_export_ready_segmenter() -> FloorAwareRoomSegmenter:
    segmenter = _build_export_ready_segmenter()
    floor_state = segmenter.floor_states["floor_1"]
    floor_state.local_to_world_room_id = {1: 1, 2: 2, 3: 3}
    floor_state.segmenter._grid_to_world = lambda u, v: np.array([float(u) * 2.0, float(v) * 2.0], dtype=np.float32)
    floor_state.segmenter.last_room_markers = np.array(
        [
            [1, 1, 2, 2, 3, 3, 4],
            [1, 1, 2, 2, 3, 3, 4],
        ],
        dtype=np.int32,
    )
    floor_state.segmenter.label_to_global = {1: 1, 2: 2, 3: 3}

    def vote_room_label_for_object(center_xy, *_args, **_kwargs):
        if float(center_xy[0]) < 4.0:
            return (1, 1, {1: 9})
        if float(center_xy[0]) < 8.0:
            return (2, 2, {2: 9})
        return (3, 3, {3: 9})

    floor_state.segmenter._vote_room_label_for_object = vote_room_label_for_object
    return segmenter


def _build_room_and_unassigned_export_ready_segmenter() -> FloorAwareRoomSegmenter:
    segmenter = _build_export_ready_segmenter()
    floor_state = segmenter.floor_states["floor_1"]
    floor_state.local_to_world_room_id = {1: 1}
    floor_state.segmenter.last_room_markers = np.array([[1, 1], [1, 2]], dtype=np.int32)
    floor_state.segmenter.label_to_global = {1: 1}

    def vote_room_label_for_object(center_xy, *_args, **_kwargs):
        if float(center_xy[0]) < 4.0:
            return (1, 1, {1: 9})
        return (-1, -1, {})

    floor_state.segmenter._vote_room_label_for_object = vote_room_label_for_object
    return segmenter


def _mutate_two_room_segmenter_for_room2_geometry_change(segmenter: FloorAwareRoomSegmenter) -> None:
    floor_state = segmenter.floor_states["floor_1"]
    floor_state.segmenter.last_room_markers = np.array(
        [
            [1, 1, 2, 4, 3, 3, 4],
            [1, 1, 2, 2, 3, 3, 4],
        ],
        dtype=np.int32,
    )
    floor_state.segmenter.label_to_global = {1: 1, 2: 2, 3: 3}

    def vote_room_label_for_object(center_xy, *_args, **_kwargs):
        if float(center_xy[0]) < 4.0:
            return (1, 1, {1: 9})
        if float(center_xy[0]) < 8.0:
            return (2, 2, {2: 9})
        return (3, 3, {3: 9})

    floor_state.segmenter._vote_room_label_for_object = vote_room_label_for_object


def _advance_segmenter_frame(segmenter: FloorAwareRoomSegmenter, *, frame_idx: int, z: float = 0.05) -> None:
    pose = np.eye(4, dtype=np.float32)
    pose[2, 3] = float(z)
    segmenter.observe_frame(
        frame_idx=int(frame_idx),
        timestamp=float(frame_idx) * 0.1,
        pose_matrix=pose,
        points_xyzrgb=None,
        is_keyframe=True,
    )
    segmenter.last_floor_observation = {"floor_id": "floor_1", "status": "stable"}


def _build_multifloor_vector_map() -> Dict[str, Any]:
    return {
        "floors": [
            {
                "floor_id": "floor_1",
                "floor_index": 0,
                "z_min": -0.5,
                "z_max": 1.2,
                "z_center": 0.2,
                "confidence": 0.98,
                "status": "confirmed",
                "support_statistics": {"frame_count": 12, "keyframe_count": 6},
            },
            {
                "floor_id": "floor_2",
                "floor_index": 1,
                "z_min": 2.7,
                "z_max": 4.4,
                "z_center": 3.4,
                "confidence": 0.96,
                "status": "confirmed",
                "support_statistics": {"frame_count": 9, "keyframe_count": 4},
            },
        ],
        "rooms": [
            {
                "id": 1,
                "polygon": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
                "floor_id": "floor_1",
                "floor_index": 0,
                "floor_assignment_confidence": 0.98,
                "status": "confirmed",
            },
            {
                "id": 2,
                "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]],
                "floor_id": "floor_1",
                "floor_index": 0,
                "floor_assignment_confidence": 0.97,
                "status": "confirmed",
            },
            {
                "id": 3,
                "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]],
                "floor_id": "floor_2",
                "floor_index": 1,
                "floor_assignment_confidence": 0.96,
                "status": "confirmed",
            },
            {
                "id": 4,
                "polygon": [[8.2, 0.0], [12.2, 0.0], [12.2, 4.0], [8.2, 4.0]],
                "floor_id": "floor_2",
                "floor_index": 1,
                "floor_assignment_confidence": 0.95,
                "status": "confirmed",
            },
        ],
        "gateways": [
            {"connects": [1, 2], "type": "door", "pos_world": [4.05, 2.0]},
            {"connects": [3, 4], "type": "door", "pos_world": [8.15, 2.0]},
            {"connects": [2, 3], "type": "door", "pos_world": [6.1, 2.0]},
        ],
        "vertical_transitions": [
            {
                "transition_id": "vt_1",
                "type": "vertical_transition",
                "from_room_id": 2,
                "to_room_id": 3,
                "from_floor_id": "floor_1",
                "to_floor_id": "floor_2",
                "frame_start": 10,
                "frame_end": 14,
                "confidence": 0.92,
                "status": "confirmed",
                "notes": "stairs between floor_1 and floor_2",
            }
        ],
        "objects": [
            {"id": 101, "label": "desk", "room_uuid": 1, "floor_id": "floor_1", "score": 0.91},
            {"id": 401, "label": "bed", "room_uuid": 4, "floor_id": "floor_2", "score": 0.89},
        ],
        "anchors": [
            {
                "id": "anchor_room_1",
                "room_id": "room_1",
                "target_id": "room_1",
                "anchor_type": "room",
                "floor_id": "floor_1",
            },
            {
                "id": "anchor_obj_401",
                "room_id": "room_4",
                "target_id": "obj_401",
                "anchor_type": "object",
                "floor_id": "floor_2",
            },
        ],
    }


def _build_multifloor_transition_history() -> List[Dict[str, Any]]:
    return [
        {"frame_idx": 0, "timestamp": 0.0, "current_room_id": 1},
        {"frame_idx": 1, "timestamp": 0.1, "current_room_id": 1},
        {"frame_idx": 2, "timestamp": 0.2, "current_room_id": 2},
        {"frame_idx": 3, "timestamp": 0.3, "current_room_id": 2},
        {"frame_idx": 4, "timestamp": 0.4, "current_room_id": 3},
        {"frame_idx": 5, "timestamp": 0.5, "current_room_id": 3},
        {"frame_idx": 6, "timestamp": 0.6, "current_room_id": 4},
        {"frame_idx": 7, "timestamp": 0.7, "current_room_id": 4},
    ]


def _validate_floor_manager() -> None:
    manager = FloorManager(
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        }
    )
    zs = [0.0, 0.05, 1.45, 3.25, 3.35, 3.4]
    observations = [
        manager.observe_pose(frame_idx=i, timestamp=i * 0.1, pose_z=z, is_keyframe=(i % 2 == 0))
        for i, z in enumerate(zs)
    ]

    assert observations[0].floor_id == "floor_1"
    assert observations[2].status == "transition"
    assert observations[4].floor_id == "floor_2"
    assert observations[4].status == "stable"

    floors = manager.export_floors()
    assert len(floors) == 2
    assert floors[0]["floor_id"] == "floor_1"
    assert floors[1]["floor_id"] == "floor_2"
    assert floors[0]["floor_index"] == 0
    assert floors[1]["floor_index"] == 1
    assert floors[0]["display_floor_id"] == "floor_1"
    assert floors[1]["display_floor_id"] == "floor_2"

    history = manager.export_assignment_history()
    assert any(item["status"] == "transition" for item in history)
    assert history[-1]["floor_id"] == "floor_2"
    assert history[-1]["display_floor_id"] == "floor_2"


def _validate_multifloor_topology() -> None:
    topology = RoomTopologyBuilder().build(
        _build_multifloor_vector_map(),
        sequence_id="multifloor_mock_sequence",
        transition_history=_build_multifloor_transition_history(),
        metadata={"test_case": "floor_aware_world_graph_v0_1"},
    )

    errors = topology.validate()
    assert not errors, f"multifloor topology validation must pass: {errors}"

    inspection = topology.inspect_export(sample_limit=4)
    assert inspection["floor_count"] == 2
    assert inspection["vertical_transition_edge_count"] == 1

    room_2 = topology.get_room("room_2")
    room_3 = topology.get_room("room_3")
    assert room_2["floor_id"] == "floor_1"
    assert room_3["floor_id"] == "floor_2"

    cross_floor = topology.explain_connection("room_2", "room_3")
    relation_types = {item["relation_type"] for item in cross_floor["relations"]}
    assert relation_types == {"vertical_transition"}

    same_floor_neighbors = topology.get_room_neighbors("room_2")
    assert ("room_1", "transition") in {(item["room_id"], item["relation_type"]) for item in same_floor_neighbors}
    assert ("room_3", "adjacent") not in {(item["room_id"], item["relation_type"]) for item in same_floor_neighbors}
    assert ("room_3", "transition") not in {(item["room_id"], item["relation_type"]) for item in same_floor_neighbors}
    assert ("room_3", "vertical_transition") in {(item["room_id"], item["relation_type"]) for item in same_floor_neighbors}

    route = topology.find_room_path("room_1", "room_4")
    assert route["found"]
    assert route["room_sequence"] == ["room_1", "room_2", "room_3", "room_4"]
    assert route["used_relation_types"] == ["transition", "vertical_transition", "transition"]

    query_api = RoomTopologyQueryAPI(topology)
    strict_query = query_api.query_route("room_1", "room_4", route_policy="strict")
    assert strict_query["found"]
    assert strict_query["route"]["used_relation_types"] == ["transition", "vertical_transition", "transition"]
    assert strict_query["target_resolution"]["resolved_floor_id"] == "floor_2"
    assert len(strict_query["explanation"]["floor_switches"]) == 1
    assert strict_query["explanation"]["floor_switches"][0]["relation_type"] == "vertical_transition"
    assert "floor switch" in strict_query["explanation"]["route_summary"]
    assert any("vertical_transition" in item for item in strict_query["explanation"]["hop_summaries"])

    anchor_query = query_api.query_route_to_anchor("room_1", "anchor_obj_401")
    assert anchor_query["found"]
    assert anchor_query["target_resolution"]["resolved_floor_id"] == "floor_2"

    object_query = query_api.query_route_to_object("room_1", object_id=401)
    assert object_query["found"]
    assert object_query["target_resolution"]["resolved_floor_id"] == "floor_2"


def _validate_end_of_sequence_floor_flush() -> None:
    segmenter = FloorAwareRoomSegmenter(
        resolution=0.05,
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        },
    )

    def pose(z: float) -> np.ndarray:
        mat = np.eye(4, dtype=np.float32)
        mat[2, 3] = float(z)
        return mat

    points = np.array(
        [
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.1, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.1, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    segmenter.observe_frame(frame_idx=0, timestamp=0.0, pose_matrix=pose(0.0), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=1, timestamp=0.1, pose_matrix=pose(0.1), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=2, timestamp=0.2, pose_matrix=pose(3.0), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=3, timestamp=0.3, pose_matrix=pose(3.1), points_xyzrgb=points, is_keyframe=True)

    floor_1_state = segmenter.floor_states["floor_1"]
    floor_1_state.pending_chunks = []
    floor_2_state = segmenter.floor_states["floor_2"]

    def fake_segmentation(*_args, **_kwargs):
        floor_2_state.segmenter.last_tracking_report = {"matched": [], "new_rooms": [{"global_id": 1}], "retained_missing": [], "dropped_missing": []}
        floor_2_state.segmenter.last_room_markers = np.array([[1, 1], [1, 2]], dtype=np.int32)
        floor_2_state.segmenter.label_to_global = {1: 1}
        floor_2_state.segmenter.last_gateways = []
        return floor_2_state.segmenter.last_room_markers

    floor_2_state.segmenter.perform_segmentation = fake_segmentation

    report = segmenter.flush_pending_floor_segments(count=99)
    assert report["segmented_floor_ids"] == ["floor_2"]
    assert floor_2_state.segmentation_trigger_count == 1
    assert floor_2_state.last_segmentation_frame_idx == 99


def _validate_late_floor_starvation_regression() -> None:
    segmenter = FloorAwareRoomSegmenter(
        resolution=0.05,
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        },
    )

    def pose(z: float) -> np.ndarray:
        mat = np.eye(4, dtype=np.float32)
        mat[2, 3] = float(z)
        return mat

    points = np.array(
        [
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.1, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.1, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    segmenter.observe_frame(frame_idx=0, timestamp=0.0, pose_matrix=pose(4.3), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=1, timestamp=0.1, pose_matrix=pose(4.35), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=2, timestamp=0.2, pose_matrix=pose(-0.6), points_xyzrgb=points, is_keyframe=True)
    segmenter.observe_frame(frame_idx=3, timestamp=0.3, pose_matrix=pose(-0.55), points_xyzrgb=points, is_keyframe=True)

    late_floor_state = segmenter.floor_states["floor_2"]
    segmenter.floor_states["floor_1"].pending_chunks = []

    def fake_segmentation(*_args, **_kwargs):
        late_floor_state.segmenter.last_tracking_report = {
            "matched": [],
            "new_rooms": [{"global_id": 1}],
            "retained_missing": [],
            "dropped_missing": [],
        }
        late_floor_state.segmenter.last_room_markers = np.array([[1, 1], [1, 2]], dtype=np.int32)
        late_floor_state.segmenter.label_to_global = {1: 1}
        late_floor_state.segmenter.last_gateways = []
        return late_floor_state.segmenter.last_room_markers

    late_floor_state.segmenter.perform_segmentation = fake_segmentation

    report = segmenter.flush_pending_floor_segments(count=99)
    vector_map = segmenter.get_vector_map_data(save_scene_graph_vis=False)

    late_floor_export = next(item for item in vector_map["floors"] if item["floor_id"] == "floor_2")
    late_floor_rooms = [room for room in vector_map["rooms"] if room.get("floor_id") == "floor_2"]

    assert report["segmented_floor_ids"] == ["floor_2"]
    assert report["segmented_display_floor_ids"] == ["floor_1"]
    assert late_floor_export["display_floor_id"] == "floor_1"
    assert late_floor_export["floor_id"] == "floor_2"
    assert len(late_floor_rooms) == 1
    assert late_floor_rooms[0]["display_floor_id"] == "floor_1"


def _build_low_span_room_cloud() -> np.ndarray:
    xs = np.linspace(0.0, 4.0, 41)
    ys = np.linspace(0.0, 3.0, 31)
    zs = np.linspace(0.0, 1.0, 21)

    floor = np.array([[x, y, 0.05] for x in xs for y in ys], dtype=np.float64)
    ceiling = np.array([[x, y, 0.98] for x in xs for y in ys], dtype=np.float64)
    wall_x0 = np.array([[0.0, y, z] for y in ys for z in zs], dtype=np.float64)
    wall_x1 = np.array([[4.0, y, z] for y in ys for z in zs], dtype=np.float64)
    wall_y0 = np.array([[x, 0.0, z] for x in xs for z in zs], dtype=np.float64)
    wall_y1 = np.array([[x, 3.0, z] for x in xs for z in zs], dtype=np.float64)
    points = np.concatenate([floor, ceiling, wall_x0, wall_x1, wall_y0, wall_y1], axis=0)
    return points.astype(np.float64)


def _validate_low_span_slice_regression() -> None:
    segmenter = DynamicRoomSegmenter(resolution=0.05, config={"room_segmentation": {"tier2_enablement_mode": "off"}})
    markers = segmenter.perform_segmentation(_build_low_span_room_cloud(), all_pred_box=None, debug_path=None, count=7)

    assert markers is not None
    assert segmenter.last_failure_debug == {}
    assert segmenter.last_height_slice_debug["adaptive_applied"] is True
    assert segmenter.last_height_slice_debug["mode"] in {"adaptive_low_span", "adaptive_mid_band"}
    assert float(segmenter.last_height_slice_debug["slice_z_min"]) < float(segmenter.last_height_slice_debug["slice_z_max"])


def _validate_floor_aware_scene_graph() -> None:
    sg = SemanticSceneGraph()
    sg.add_floor_node(
        FloorNode(
            id="floor_1",
            floor_index=0,
            z_min=-0.5,
            z_max=1.2,
            z_center=0.2,
            confidence=0.98,
            status="confirmed",
            support_statistics={"frame_count": 5},
        )
    )
    sg.add_floor_node(
        FloorNode(
            id="floor_2",
            floor_index=1,
            z_min=2.7,
            z_max=4.4,
            z_center=3.4,
            confidence=0.96,
            status="confirmed",
            support_statistics={"frame_count": 4},
        )
    )
    sg.add_room_node(
        RoomNode(
            id="room_1",
            room_type="office",
            polygon=[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)],
            floor_id="floor_1",
            floor_assignment_confidence=0.98,
        )
    )
    sg.add_room_node(
        RoomNode(
            id="room_2",
            room_type="bedroom",
            polygon=[(6.0, 0.0), (11.0, 0.0), (11.0, 5.0), (6.0, 5.0)],
            floor_id="floor_2",
            floor_assignment_confidence=0.96,
        )
    )

    sg.add_object_node(
        ObjectNode(
            id="obj_101",
            center=(2.5, 2.5, 0.4),
            bbox=(1.0, 0.8, 0.8),
            label="desk",
            category="furniture",
            clip_feature=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            confidence=0.9,
            room_id="room_1",
            floor_id="floor_1",
        )
    )
    sg.add_object_node(
        ObjectNode(
            id="obj_201",
            center=(8.5, 2.5, 3.2),
            bbox=(1.5, 0.9, 0.8),
            label="sofa",
            category="furniture",
            clip_feature=np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32),
            confidence=0.88,
            room_id="room_2",
            floor_id="floor_2",
        )
    )
    sg.add_inside_relation("obj_101", "room_1")
    sg.add_inside_relation("obj_201", "room_2")

    anchors = sg.build_anchor_layer(debug=False)
    assert "ON_FLOOR" in sg.get_relations_between("room_1", "floor_1")
    assert "ON_FLOOR" in sg.get_relations_between("room_2", "floor_2")
    assert "ON_FLOOR" in sg.get_relations_between("obj_101", "floor_1")
    assert "ON_FLOOR" in sg.get_relations_between("obj_201", "floor_2")

    anchor_floor_ids = {anchor.id: anchor.floor_id for anchor in anchors}
    assert anchor_floor_ids["anchor_room_1"] == "floor_1"
    assert anchor_floor_ids["anchor_room_2"] == "floor_2"
    assert anchor_floor_ids["anchor_obj_101"] == "floor_1"
    assert anchor_floor_ids["anchor_obj_201"] == "floor_2"
    assert "ON_FLOOR" in sg.get_relations_between("anchor_room_1", "floor_1")
    assert "ON_FLOOR" in sg.get_relations_between("anchor_obj_201", "floor_2")



def _validate_vertical_transition_evidence_regression() -> None:
    segmenter = FloorAwareRoomSegmenter(
        resolution=0.05,
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        },
    )
    segmenter.frame_history = [
        {"frame_idx": 0, "status": "stable", "floor_id": "floor_1", "display_floor_id": "floor_1", "pose_z": 0.1, "position_xy": [5.2, 2.0]},
        {"frame_idx": 1, "status": "stable", "floor_id": "floor_1", "display_floor_id": "floor_1", "pose_z": 0.2, "position_xy": [5.8, 2.0]},
        {"frame_idx": 2, "status": "transition", "floor_id": "floor_1", "display_floor_id": "floor_1", "pose_z": 0.8, "position_xy": [6.0, 2.0]},
        {"frame_idx": 3, "status": "transition", "floor_id": "floor_1", "display_floor_id": "floor_1", "pose_z": 1.6, "position_xy": [6.05, 2.0]},
        {"frame_idx": 4, "status": "transition", "floor_id": "floor_2", "display_floor_id": "floor_2", "pose_z": 2.4, "position_xy": [6.1, 2.0]},
        {"frame_idx": 5, "status": "stable", "floor_id": "floor_2", "display_floor_id": "floor_2", "pose_z": 3.1, "position_xy": [6.2, 2.0]},
        {"frame_idx": 6, "status": "stable", "floor_id": "floor_2", "display_floor_id": "floor_2", "pose_z": 3.2, "position_xy": [6.3, 2.0]},
    ]
    rooms = [
        {
            "id": 2,
            "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]],
            "floor_id": "floor_1",
            "display_floor_id": "floor_1",
        },
        {
            "id": 3,
            "polygon": [[4.1, 0.0], [8.1, 0.0], [8.1, 4.0], [4.1, 4.0]],
            "floor_id": "floor_2",
            "display_floor_id": "floor_2",
        },
    ]

    transitions = segmenter._build_vertical_transitions(rooms)
    summary = segmenter._build_vertical_transition_summary(transitions)

    assert len(transitions) == 1
    transition = transitions[0]
    assert transition["type"] == "vertical_transition"
    assert transition["from_floor_id"] == "floor_1"
    assert transition["to_floor_id"] == "floor_2"
    assert transition["from_room_id"] == 2
    assert transition["to_room_id"] == 3
    assert transition["transition_frame_start"] == 2
    assert transition["transition_frame_end"] == 4
    assert transition["entry_stable_frame"] == 1
    assert transition["exit_stable_frame"] == 5
    assert transition["supporting_frame_count"] == 7
    assert transition["transition_frame_count"] == 3
    assert transition["connector_label"] in {"stair_like", "landing_like", "ramp_like", "unknown_vertical_connector"}
    assert transition["from_room_support"]["room_id"] == 2
    assert transition["to_room_support"]["room_id"] == 3
    assert transition["room_association_complete"] is True
    assert transition["edge_eligible"] is True
    assert transition["evidence_summary"]

    assert summary["count"] == 1
    assert summary["edge_eligible_count"] == 1
    assert summary["transitions"][0]["from_floor_id"] == "floor_1"
    assert summary["transitions"][0]["to_floor_id"] == "floor_2"
    assert summary["transitions"][0]["evidence_summary"]

    vector_map = _build_multifloor_vector_map()
    vector_map["vertical_transitions"] = transitions
    vector_map["vertical_transition_summary"] = summary

    topology = RoomTopologyBuilder().build(
        vector_map,
        sequence_id="vertical_transition_evidence_mock",
        transition_history=_build_multifloor_transition_history(),
        metadata={"test_case": "vertical_transition_evidence_regression"},
    )
    explanation = topology.explain_connection("room_2", "room_3")
    vertical_relation = next(item for item in explanation["relations"] if item["relation_type"] == "vertical_transition")
    relation_metadata = dict(vertical_relation.get("metadata", {}))

    assert relation_metadata["from_floor_ids"] == ["floor_1"]
    assert relation_metadata["to_floor_ids"] == ["floor_2"]
    assert relation_metadata["display_floor_pairs"] == ["floor_1->floor_2"]
    assert relation_metadata["supporting_frame_count_total"] == 7
    assert relation_metadata["transition_records"][0]["transition_frame_start"] == 2
    assert relation_metadata["transition_records"][0]["transition_frame_end"] == 4
    assert relation_metadata["transition_records"][0]["evidence_summary"]

    evidence_metadata = dict(vertical_relation["evidences"][0].get("metadata", {}))
    assert evidence_metadata["transition_count"] == 1
    assert evidence_metadata["from_floor_ids"] == ["floor_1"]
    assert evidence_metadata["to_floor_ids"] == ["floor_2"]
    assert evidence_metadata["transition_records"][0]["connector_label"] == transition["connector_label"]


def _validate_persistent_fallback_diagnostics_regression() -> None:
    segmenter = FloorAwareRoomSegmenter(
        resolution=0.05,
        config={
            "floor_segmentation": {
                "new_floor_confirm_frames": 2,
                "min_confirmed_support_frames": 2,
                "stable_assignment_margin_m": 0.7,
                "pending_assignment_margin_m": 1.0,
                "new_floor_min_separation_m": 2.0,
            }
        },
    )

    def pose(z: float) -> np.ndarray:
        mat = np.eye(4, dtype=np.float32)
        mat[2, 3] = float(z)
        return mat

    segmenter.observe_frame(frame_idx=0, timestamp=0.0, pose_matrix=pose(0.0), points_xyzrgb=None, is_keyframe=True)
    segmenter.observe_frame(frame_idx=1, timestamp=0.1, pose_matrix=pose(0.05), points_xyzrgb=None, is_keyframe=True)

    floor_state = segmenter._get_or_create_floor_state("floor_1")
    floor_state.segmenter.last_failure_debug = {}
    floor_state.segmenter.last_room_markers = np.array([[1, 1], [1, 2]], dtype=np.int32)

    floor_state.segmenter.last_height_slice_debug = {
        "mode": "adaptive_low_span",
        "adaptive_applied": True,
        "slice_z_min": 0.1,
        "slice_z_max": 0.8,
    }
    floor_state.segmenter.last_tracking_report = {
        "matched": [],
        "new_rooms": [{"global_id": 1}],
        "retained_missing": [],
        "dropped_missing": [],
    }
    adaptive_record = segmenter._record_segmentation_run(
        "floor_1",
        floor_state,
        trigger_reason="interval_trigger",
        active_status="stable",
        count=10,
        pending_chunk_count=2,
        pending_chunk_frame_indices=[0, 1],
        merged_point_count=120,
        success=True,
    )

    floor_state.segmenter.last_height_slice_debug = {
        "mode": "default",
        "adaptive_applied": False,
        "slice_z_min": 0.2,
        "slice_z_max": 1.8,
    }
    floor_state.segmenter.last_tracking_report = {
        "matched": [{"global_id": 1}],
        "new_rooms": [],
        "retained_missing": [],
        "dropped_missing": [],
    }
    default_record = segmenter._record_segmentation_run(
        "floor_1",
        floor_state,
        trigger_reason="interval_trigger",
        active_status="stable",
        count=11,
        pending_chunk_count=0,
        pending_chunk_frame_indices=[],
        merged_point_count=120,
        success=True,
    )

    diagnostics = segmenter._build_segmentation_diagnostics(segmenter.floor_manager.export_floors())
    assert diagnostics["run_count"] == 2
    assert diagnostics["successful_run_count"] == 2
    assert diagnostics["fallback_run_count"] == 1
    assert diagnostics["fallback_counts"] == {"adaptive_low_span": 1}
    assert diagnostics["slice_mode_counts"] == {"adaptive_low_span": 1, "default_slice": 1}

    per_floor = diagnostics["per_floor"][0]
    assert per_floor["fallback_run_count"] == 1
    assert per_floor["fallback_counts"] == {"adaptive_low_span": 1}
    assert adaptive_record["fallback_used"] is True
    assert adaptive_record["fallback_modes"] == ["adaptive_low_span"]
    assert adaptive_record["fallback_reason_summary"] == "slice=adaptive_low_span; adaptive_height_slice_applied"
    assert adaptive_record["pending_chunk_frame_start"] == 0
    assert adaptive_record["pending_chunk_frame_end"] == 1
    assert default_record["fallback_used"] is False
    assert default_record["fallback_modes"] == []
    assert default_record["fallback_reason_summary"] == "slice=default_slice"

    vector_map = segmenter.get_vector_map_data(save_scene_graph_vis=False)
    exported = dict(vector_map.get("room_segmentation_diagnostics") or {})
    assert exported["run_count"] == 2
    assert exported["fallback_run_count"] == 1
    assert exported["fallback_counts"] == {"adaptive_low_span": 1}
    assert exported["runs"][0]["primary_fallback_mode"] == "adaptive_low_span"
    assert exported["runs"][0]["pending_chunk_frame_count"] == 2
    assert exported["runs"][1]["fallback_used"] is False
    assert exported["runs"][1]["fallback_modes"] == []


def _validate_same_frame_full_export_reuse() -> None:
    segmenter = _build_export_ready_segmenter()
    instances = _MockInstances(
        boxes=[[0.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0]],
        categories=["chair"],
        instance_ids=[11],
        scores=[0.95],
    )

    vector_map_a = segmenter.get_vector_map_data(
        instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    profile_a = dict(segmenter.last_export_profile)
    vector_map_b = segmenter.get_vector_map_data(
        instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_a["cache_hit"] is False
    assert profile_b["cache_hit"] is True
    assert profile_b["cache_hit_kind"] == "same_frame_full_export_reuse"
    assert profile_b["same_frame_reuse_eligible"] is True
    assert profile_b["same_frame_reuse_blocker"] == "none"
    assert profile_b["total_sec"] == 0.0
    assert profile_b["object_export_sec"] == 0.0
    assert vector_map_a is vector_map_b
    assert len(vector_map_b["objects"]) == 1
    assert vector_map_b["objects"][0]["id"] == 11


def _validate_same_frame_object_export_memoization() -> None:
    segmenter = _build_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [0.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
            [1.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 22],
        scores=[0.95, 0.88],
    )
    updated_instances = _MockInstances(
        boxes=[
            [0.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
            [1.5, 0.7, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 22],
        scores=[0.95, 0.91],
    )

    vector_map_a = segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    cached_first_object = vector_map_a["objects"][0]
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_b["cache_hit"] is False
    assert profile_b["same_frame_cache_frame_match"] is True
    assert profile_b["same_frame_segmentation_token_match"] is True
    assert profile_b["same_frame_reuse_blocker"] == "object_state_signature_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["room_local_delta_changed_room_count"] == 1
    assert profile_b["changed_room_count"] == 1
    assert profile_b["changed_room_ids"] == "room_1"
    assert profile_b["changed_room_local_rebuild_used"] is True
    assert profile_b["full_fallback_rebuild_used"] is False
    assert profile_b["room_local_delta_reused_room_count"] == 0
    assert profile_b["room_local_delta_rebuilt_room_count"] == 1
    assert profile_b["object_export_reused_count"] == 0
    assert profile_b["object_export_rebuilt_count"] == 2
    assert profile_b["rebuilt_object_in_changed_rooms_count"] == 2
    assert vector_map_b["objects"][0] is not cached_first_object
    assert vector_map_b["objects"][1]["pose_3d"][1] == 0.7


def _validate_same_frame_anchor_reuse_for_unchanged_rooms() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )
    updated_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.2, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.91],
    )

    vector_map_a = segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    room_1_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_2_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_2"
        ]
    )

    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)
    room_1_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_2_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_2"
        ]
    )

    assert profile_b["cache_hit"] is False
    assert profile_b["same_frame_reuse_blocker"] == "object_state_signature_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["room_local_delta_changed_room_count"] == 1
    assert profile_b["changed_room_count"] == 1
    assert profile_b["changed_room_ids"] == "room_3"
    assert profile_b["changed_room_local_rebuild_used"] is True
    assert profile_b["room_local_delta_reused_room_count"] == 2
    assert profile_b["room_local_delta_rebuilt_room_count"] == 1
    assert profile_b["object_export_reused_count"] == 2
    assert profile_b["object_export_rebuilt_count"] == 1
    assert profile_b["rebuilt_object_in_changed_rooms_count"] == 1
    assert profile_b["anchor_reuse_room_count"] == 2
    assert profile_b["anchor_rebuild_room_count"] == 1
    assert room_1_anchor_snapshot_after == room_1_anchor_snapshot
    assert room_2_anchor_snapshot_after == room_2_anchor_snapshot


def _validate_cross_frame_object_export_reuse() -> None:
    segmenter = _build_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [0.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
            [1.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 22],
        scores=[0.95, 0.88],
    )
    updated_instances = _MockInstances(
        boxes=[
            [0.5, 0.5, 0.3, 0.6, 0.4, 0.8, 0.0],
            [1.5, 0.7, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 22],
        scores=[0.95, 0.91],
    )

    vector_map_a = segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    cached_first_object = vector_map_a["objects"][0]
    _advance_segmenter_frame(segmenter, frame_idx=11)
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=11,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_b["cache_hit"] is False
    assert profile_b["same_frame_cache_frame_match"] is False
    assert profile_b["same_frame_reuse_blocker"] == "frame_mismatch"
    assert profile_b["room_local_delta_export_used"] is False
    assert profile_b["changed_room_local_rebuild_used"] is False
    assert profile_b["full_fallback_rebuild_used"] is False
    assert profile_b["object_export_reused_count"] == 1
    assert profile_b["object_export_rebuilt_count"] == 1
    assert vector_map_b["objects"][0]["id"] == cached_first_object["id"]
    assert vector_map_b["objects"][0]["footprint_2d"] == cached_first_object["footprint_2d"]
    assert vector_map_b["objects"][1]["pose_3d"][1] == 0.7


def _validate_cross_frame_anchor_reuse_for_unchanged_rooms() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )
    updated_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.2, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.91],
    )

    vector_map_a = segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    room_1_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_2_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_2"
        ]
    )

    _advance_segmenter_frame(segmenter, frame_idx=11)
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=11,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)
    room_1_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_2_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_2"
        ]
    )

    assert profile_b["cache_hit"] is False
    assert profile_b["same_frame_cache_frame_match"] is False
    assert profile_b["same_frame_reuse_blocker"] == "frame_mismatch"
    assert profile_b["object_export_reused_count"] == 2
    assert profile_b["object_export_rebuilt_count"] == 1
    assert profile_b["changed_room_local_rebuild_used"] is False
    assert profile_b["full_fallback_rebuild_used"] is False
    assert profile_b["anchor_reuse_room_count"] == 2
    assert profile_b["anchor_rebuild_room_count"] == 1
    assert room_1_anchor_snapshot_after == room_1_anchor_snapshot
    assert room_2_anchor_snapshot_after == room_2_anchor_snapshot


def _validate_cross_frame_changed_room_rebuild_reduction() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )

    vector_map_a = segmenter.get_vector_map_data(
        instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    room_1_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_3_anchor_snapshot = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_a["anchors"]
            if item["room_id"] == "room_3"
        ]
    )

    _advance_segmenter_frame(segmenter, frame_idx=11)
    _mutate_two_room_segmenter_for_room2_geometry_change(segmenter)
    original_build_object_export_record = segmenter._build_object_export_record
    build_object_export_record_call_count = 0

    def _counting_build_object_export_record(*args, **kwargs):
        nonlocal build_object_export_record_call_count
        build_object_export_record_call_count += 1
        return original_build_object_export_record(*args, **kwargs)

    segmenter._build_object_export_record = _counting_build_object_export_record
    try:
        vector_map_b = segmenter.get_vector_map_data(
            instances,
            count=11,
            save_scene_graph_vis=False,
            instrumentation_context="stage5_snapshot_capture",
        )
    finally:
        segmenter._build_object_export_record = original_build_object_export_record
    profile_b = dict(segmenter.last_export_profile)
    room_1_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_1"
        ]
    )
    room_3_anchor_snapshot_after = sorted(
        [
            (item["id"], item["target_id"], tuple(item["position"]))
            for item in vector_map_b["anchors"]
            if item["room_id"] == "room_3"
        ]
    )

    assert profile_b["same_frame_cache_frame_match"] is False
    assert profile_b["same_frame_reuse_blocker"] == "frame_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["changed_room_local_rebuild_used"] is True
    assert profile_b["changed_room_locality_confident"] is True
    assert profile_b["full_fallback_rebuild_used"] is False
    assert profile_b["changed_room_count"] == 1
    assert profile_b["changed_room_ids"] == "room_2"
    assert profile_b["changed_room_structure_changed_count"] == 1
    assert profile_b["changed_room_structure_changed_ids"] == "room_2"
    assert profile_b["changed_room_removed_count"] == 0
    assert profile_b["object_export_reused_count"] == 2
    assert profile_b["object_export_rebuilt_count"] == 1
    assert profile_b["rebuilt_object_in_changed_rooms_count"] == 1
    assert build_object_export_record_call_count == 1
    assert profile_b["anchor_reuse_room_count"] == 2
    assert profile_b["anchor_rebuild_room_count"] == 1
    assert room_1_anchor_snapshot_after == room_1_anchor_snapshot
    assert room_3_anchor_snapshot_after == room_3_anchor_snapshot


def _validate_same_frame_room_local_delta_disappearance() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )
    updated_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table"],
        instance_ids=[11, 22],
        scores=[0.95, 0.88],
    )

    segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_b["same_frame_reuse_blocker"] == "object_state_signature_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["room_local_delta_changed_room_count"] == 1
    assert profile_b["changed_room_count"] == 1
    assert profile_b["changed_room_ids"] == "room_3"
    assert profile_b["changed_room_removed_count"] == 0
    assert profile_b["room_local_delta_reused_room_count"] == 2
    assert profile_b["room_local_delta_rebuilt_room_count"] == 1
    assert profile_b["object_export_reused_count"] == 2
    assert profile_b["object_export_rebuilt_count"] == 0
    assert [obj["id"] for obj in vector_map_b["objects"]] == [11, 22]


def _validate_same_frame_room_local_delta_unassigned_bucket() -> None:
    segmenter = _build_room_and_unassigned_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [8.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 99],
        scores=[0.95, 0.86],
    )
    updated_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [8.2, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "lamp"],
        instance_ids=[11, 99],
        scores=[0.95, 0.91],
    )

    vector_map_a = segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    cached_room_object = next(obj for obj in vector_map_a["objects"] if obj["id"] == 11)
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_b["same_frame_reuse_blocker"] == "object_state_signature_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["room_local_delta_changed_room_count"] == 0
    assert profile_b["changed_room_count"] == 0
    assert profile_b["changed_room_ids"] == ""
    assert profile_b["room_local_delta_reused_room_count"] == 1
    assert profile_b["room_local_delta_rebuilt_room_count"] == 0
    assert profile_b["room_local_delta_unassigned_bucket_rebuilt"] is True
    assert profile_b["object_export_reused_count"] == 1
    assert profile_b["object_export_rebuilt_count"] == 1
    reused_room_object = next(obj for obj in vector_map_b["objects"] if obj["id"] == 11)
    assert reused_room_object["room_id"] == cached_room_object["room_id"]
    assert reused_room_object["footprint_2d"] == cached_room_object["footprint_2d"]
    assert next(obj for obj in vector_map_b["objects"] if obj["id"] == 99)["room_id"] is None


def _validate_same_frame_room_local_delta_two_room_move() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    initial_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )
    updated_instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.2, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.92, 0.86],
    )

    segmenter.get_vector_map_data(
        initial_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage3_post_segmentation_refresh",
    )
    vector_map_b = segmenter.get_vector_map_data(
        updated_instances,
        count=10,
        save_scene_graph_vis=False,
        instrumentation_context="stage5_snapshot_capture",
    )
    profile_b = dict(segmenter.last_export_profile)

    assert profile_b["same_frame_reuse_blocker"] == "object_state_signature_mismatch"
    assert profile_b["room_local_delta_export_used"] is True
    assert profile_b["room_local_delta_changed_room_count"] == 2
    assert profile_b["changed_room_count"] == 2
    assert profile_b["changed_room_ids"] == "room_2|room_3"
    assert profile_b["room_local_delta_reused_room_count"] == 1
    assert profile_b["room_local_delta_rebuilt_room_count"] == 2
    assert profile_b["object_export_reused_count"] == 1
    assert profile_b["object_export_rebuilt_count"] == 2
    assert [obj["room_id"] for obj in vector_map_b["objects"]] == ["room_1", "room_3", "room_3"]


def _validate_object_export_floor_vote_cache_once_per_export() -> None:
    segmenter = _build_two_room_export_ready_segmenter()
    instances = _MockInstances(
        boxes=[
            [1.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [5.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
            [9.0, 1.0, 0.3, 0.6, 0.4, 0.8, 0.0],
        ],
        categories=["chair", "table", "lamp"],
        instance_ids=[11, 22, 33],
        scores=[0.95, 0.88, 0.86],
    )

    original_build_object_export_record = segmenter._build_object_export_record
    original_build_object_export_floor_vote_cache = segmenter._build_object_export_floor_vote_cache
    build_object_export_record_call_count = 0
    build_object_export_floor_vote_cache_call_count = 0

    def _counting_build_object_export_record(*args, **kwargs):
        nonlocal build_object_export_record_call_count
        build_object_export_record_call_count += 1
        return original_build_object_export_record(*args, **kwargs)

    def _counting_build_object_export_floor_vote_cache(*args, **kwargs):
        nonlocal build_object_export_floor_vote_cache_call_count
        build_object_export_floor_vote_cache_call_count += 1
        return original_build_object_export_floor_vote_cache(*args, **kwargs)

    segmenter._build_object_export_record = _counting_build_object_export_record
    segmenter._build_object_export_floor_vote_cache = _counting_build_object_export_floor_vote_cache
    try:
        segmenter.get_vector_map_data(
            instances,
            count=10,
            save_scene_graph_vis=False,
            instrumentation_context="stage5_snapshot_capture",
        )
    finally:
        segmenter._build_object_export_record = original_build_object_export_record
        segmenter._build_object_export_floor_vote_cache = original_build_object_export_floor_vote_cache

    profile = dict(segmenter.last_export_profile)
    assert profile["object_export_rebuilt_count"] == 3
    assert build_object_export_record_call_count == 3
    assert build_object_export_floor_vote_cache_call_count == 1



def run_mock_test() -> None:
    print("Running floor-aware world graph validation...")
    _validate_floor_manager()
    _validate_multifloor_topology()
    _validate_end_of_sequence_floor_flush()
    _validate_late_floor_starvation_regression()
    _validate_low_span_slice_regression()
    _validate_vertical_transition_evidence_regression()
    _validate_persistent_fallback_diagnostics_regression()
    _validate_same_frame_full_export_reuse()
    _validate_same_frame_object_export_memoization()
    _validate_same_frame_anchor_reuse_for_unchanged_rooms()
    _validate_cross_frame_object_export_reuse()
    _validate_cross_frame_anchor_reuse_for_unchanged_rooms()
    _validate_cross_frame_changed_room_rebuild_reduction()
    _validate_same_frame_room_local_delta_disappearance()
    _validate_same_frame_room_local_delta_unassigned_bucket()
    _validate_same_frame_room_local_delta_two_room_move()
    _validate_object_export_floor_vote_cache_once_per_export()
    _validate_floor_aware_scene_graph()
    print("Floor-aware world graph validated.")


if __name__ == "__main__":
    run_mock_test()
