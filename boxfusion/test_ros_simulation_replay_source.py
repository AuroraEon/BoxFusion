from __future__ import annotations

import json
from pathlib import Path

from boxfusion.ros_simulation_replay_source import load_simulation_capture_sequence


def test_load_simulation_capture_sequence_resolves_scene_root_and_frames(tmp_path: Path) -> None:
    scene_root = tmp_path / "stage_a_sequences" / "demo_sequence"
    logs_dir = scene_root / "logs"
    frames_dir = scene_root / "frames"
    logs_dir.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)

    rgb_path = frames_dir / "frame_000000_rgb.bin"
    depth_path = frames_dir / "frame_000000_depth.bin"
    rgb_path.write_bytes(b"\x01\x02\x03")
    depth_path.write_bytes(b"\x00\x00")

    capture_log = logs_dir / "simulation_ingress_frames.jsonl"
    row = {
        "frame_idx": 0,
        "rgb": {
            "width": 1,
            "height": 1,
            "encoding": "rgb8",
            "step": 3,
            "frame_id": "camera_color_optical_frame",
            "payload_path": "frames/frame_000000_rgb.bin",
        },
        "depth": {
            "width": 1,
            "height": 1,
            "encoding": "16UC1",
            "step": 2,
            "frame_id": "camera_depth_optical_frame",
            "payload_path": "frames/frame_000000_depth.bin",
        },
        "camera_info": {
            "width": 1,
            "height": 1,
            "k": [1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0],
            "d": [],
            "distortion_model": "plumb_bob",
            "frame_id": "camera_color_optical_frame",
        },
        "pose": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "frame_id": "map",
        },
    }
    capture_log.write_text(json.dumps(row) + "\n", encoding="utf-8")

    sequence = load_simulation_capture_sequence(capture_log)

    assert sequence.scene_root == scene_root.resolve()
    assert len(sequence.frames) == 1
    assert sequence.frames[0].frame_idx == 0
    assert sequence.resolve_payload_path(sequence.frames[0].rgb["payload_path"]) == rgb_path.resolve()
    assert sequence.resolve_payload_path(sequence.frames[0].depth["payload_path"]) == depth_path.resolve()
