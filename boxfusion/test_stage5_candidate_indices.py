import numpy as np

from demo import _build_floor_scoped_candidate_mask


class _MockTensor:
    def __init__(self, values, dtype):
        self._array = np.asarray(values, dtype=dtype)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self._array)


class _MockBoxes3D:
    def __init__(self, boxes):
        self.tensor = _MockTensor(boxes, dtype=np.float32)


class _MockInstances:
    def __init__(self, *, boxes, init_ids, frame_ids):
        self.pred_boxes_3d = _MockBoxes3D(boxes)
        self.init_id = _MockTensor(init_ids, dtype=np.int64)
        self.frame_id = _MockTensor(frame_ids, dtype=np.int64)

    def __len__(self):
        return int(len(self.init_id.numpy()))

    def __getitem__(self, item):
        indices = np.arange(len(self), dtype=np.int64)[item]
        indices = np.asarray(indices, dtype=np.int64).reshape(-1)
        return _MockInstances(
            boxes=self.pred_boxes_3d.tensor.numpy()[indices].tolist(),
            init_ids=self.init_id.numpy()[indices].tolist(),
            frame_ids=self.frame_id.numpy()[indices].tolist(),
        )

    def get(self, field_name):
        return getattr(self, field_name)


class _MockFloorHypothesis:
    def __init__(self, *, z_center, z_min, z_max, status="confirmed", frame_count=8, keyframe_count=4):
        self.z_center = float(z_center)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.status = str(status)
        self.frame_count = int(frame_count)
        self.keyframe_count = int(keyframe_count)


class _CountingFloorManager:
    def __init__(self):
        self.assign_height_calls = 0
        self.floors = {"floor_1": _MockFloorHypothesis(z_center=0.2, z_min=0.0, z_max=0.3)}

    def _ordered_floor_ids(self):
        return ["floor_1"]

    def assign_height(self, pose_z: float):
        self.assign_height_calls += 1
        return {
            "floor_id": "floor_1",
            "status": "stable",
            "confidence": 0.99,
            "reason": "mock_same_floor",
        }


class _MockRoomSegmenter:
    def __init__(self):
        self.floor_manager = _CountingFloorManager()
        self.last_floor_observation = {"floor_id": "floor_1", "status": "stable"}

    def describe_topology_status(self, pose_matrix=None):
        return {
            "active_floor_id": "floor_1",
            "active_floor_status": "stable",
            "active_room_id": 1,
            "active_room_available": True,
        }


def test_stage5_candidate_index_cache_reuses_floor_assignments_without_changing_mask():
    all_pred_box = _MockInstances(
        boxes=[
            [0.0, 0.0, 0.10, 1.0, 1.0, 1.0],
            [0.2, 0.1, 0.15, 1.0, 1.0, 1.0],
            [0.4, 0.1, 0.20, 1.0, 1.0, 1.0],
        ],
        init_ids=[100, 101, 102],
        frame_ids=[0, 1, 10],
    )
    room_segmenter = _MockRoomSegmenter()
    candidate_index_cache = {}

    first_mask, first_profile = _build_floor_scoped_candidate_mask(
        all_pred_box,
        retained_count=2,
        room_segmenter=room_segmenter,
        current_frame_idx=10,
        candidate_index_cache=candidate_index_cache,
        enable_readonly_tail_pruning=False,
    )
    second_mask, second_profile = _build_floor_scoped_candidate_mask(
        all_pred_box,
        retained_count=2,
        room_segmenter=room_segmenter,
        current_frame_idx=10,
        candidate_index_cache=candidate_index_cache,
        enable_readonly_tail_pruning=False,
    )

    assert first_mask.tolist() == second_mask.tolist()
    assert room_segmenter.floor_manager.assign_height_calls == 2
    assert first_profile["candidate_index_floor_assignment_cache_hits"] == 0
    assert first_profile["candidate_index_floor_assignment_cache_misses"] == 2
    assert second_profile["candidate_index_floor_assignment_cache_hits"] == 2
    assert second_profile["candidate_index_floor_assignment_cache_misses"] == 0
