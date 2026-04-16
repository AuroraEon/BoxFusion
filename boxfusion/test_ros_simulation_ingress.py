from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

from boxfusion.ros_publication_diagnostics_server import BoxFusionRosPublicationDiagnosticsBackend
from boxfusion.ros_query_server import BoxFusionRosQueryServerBackend
from boxfusion.ros_simulation_ingress import (
    SimulationCameraInfoInput,
    SimulationImageInput,
    SimulationPoseInput,
    StageASimulationSnapshotWriter,
    _simulation_pose_from_transform,
    _target_snapshot_reached,
    build_validation_report,
)


def _rgb(stamp_sec: float) -> SimulationImageInput:
    return SimulationImageInput(
        width=2,
        height=2,
        encoding="rgb8",
        step=6,
        stamp_sec=stamp_sec,
        frame_id="camera_color_optical_frame",
        data=b"\x00\x00\x00\xff\x00\x00\x00\xff\x00\xff\xff\xff",
    )


def _depth(stamp_sec: float) -> SimulationImageInput:
    return SimulationImageInput(
        width=2,
        height=2,
        encoding="32FC1",
        step=8,
        stamp_sec=stamp_sec,
        frame_id="camera_depth_optical_frame",
        data=b"\x00" * 16,
    )


def _camera_info(stamp_sec: float) -> SimulationCameraInfoInput:
    return SimulationCameraInfoInput(
        width=2,
        height=2,
        k=[1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0],
        d=[],
        distortion_model="plumb_bob",
        stamp_sec=stamp_sec,
        frame_id="camera_color_optical_frame",
    )


def _pose(stamp_sec: float, x: float) -> SimulationPoseInput:
    return SimulationPoseInput(
        x=x,
        y=0.0,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        stamp_sec=stamp_sec,
        frame_id="map",
        source="test_pose",
    )


def test_tf_transform_converts_to_simulation_pose() -> None:
    transform = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=250_000_000),
            frame_id="map",
        ),
        child_frame_id="camera_color_optical_frame",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.25, y=-0.5, z=2.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.8660254),
        ),
    )

    pose = _simulation_pose_from_transform(transform)

    assert pose.x == 1.25
    assert pose.y == -0.5
    assert pose.z == 2.0
    assert pose.qz == 0.5
    assert pose.qw == 0.8660254
    assert pose.stamp_sec == 12.25
    assert pose.frame_id == "map"
    assert pose.source == "tf"


def test_target_snapshot_reached_requires_positive_limit() -> None:
    assert _target_snapshot_reached(target_snapshot_count=1, recorded_snapshot_count=1) is True
    assert _target_snapshot_reached(target_snapshot_count=2, recorded_snapshot_count=1) is False
    assert _target_snapshot_reached(target_snapshot_count=0, recorded_snapshot_count=99) is False


def test_simulation_ingress_writes_stage_a_scene_and_refreshes_coordinator(tmp_path: Path) -> None:
    writer = StageASimulationSnapshotWriter(
        output_root=tmp_path / "runs",
        coordination_root=tmp_path / "coordination",
        sequence_name="sim_test_sequence",
    )

    sample, refresh = writer.record_sample(
        rgb=_rgb(1.0),
        depth=_depth(1.0),
        camera_info=_camera_info(1.0),
        pose=_pose(1.0, 0.0),
        refresh_coordinator=True,
    )

    assert sample["frame_idx"] == 0
    assert refresh is not None
    assert writer.frame_log_path.exists()
    assert writer.scene_root.joinpath("manifest.json").exists()
    assert writer.scene_root.joinpath("logs", "topology_v0_1.json").exists()
    assert writer.scene_root.joinpath("logs", "online_topology_lifecycle_v0_1.json").exists()
    assert writer.scene_root.joinpath(sample["rgb"]["payload_path"]).exists()
    assert refresh.latest_scene_root.exists()
    assert refresh.latest_export_metadata_path.exists()

    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(refresh.latest_scene_root)
    diagnostics_backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(refresh.latest_scene_root)
    topology_payload = dict(query_backend.get_topology().payload["payload"]["topology"])
    diagnostics_payload = dict(diagnostics_backend.get_publication_diagnostics().payload["payload"])

    assert [room["id"] for room in topology_payload["rooms"]] == ["room_1"]
    assert diagnostics_payload["summary"]["published_room_count"] == 1
    assert diagnostics_payload["public_topology_definition"] == "committed/published only"


def test_validation_report_exercises_existing_consumers(tmp_path: Path) -> None:
    report = build_validation_report(
        output_root=tmp_path / "runs",
        coordination_root=tmp_path / "coordination",
        sequence_name="sim_validation_sequence",
    )

    assert report["validation_kind"] == "ros2_simulation_ingress_stub"
    assert report["ingress"]["sample_count"] == 3
    assert report["coordinator"]["latest_pointer_exists"] is True
    assert report["query_server_check"]["room_count"] == 1
    assert report["query_server_check"]["world_snapshot_available"] is True
    assert report["publication_diagnostics_check"]["published_room_count"] == 1
    latest_export = Path(report["coordinator"]["latest_export_metadata_path"])
    assert json.loads(latest_export.read_text(encoding="utf-8"))["sidecar_export"]["enabled"] is False


def test_real_backend_mode_materializes_hm3d_input_by_default(tmp_path: Path) -> None:
    writer = StageASimulationSnapshotWriter(
        output_root=tmp_path / "runs",
        coordination_root=tmp_path / "coordination",
        sequence_name="sim_hm3d_backend_sequence",
        backend_mode="real",
    )

    writer.record_sample(
        rgb=_rgb(1.0),
        depth=_depth(1.0),
        camera_info=_camera_info(1.0),
        pose=_pose(1.0, 0.0),
        refresh_coordinator=False,
    )

    manifest = writer.materialize_real_backend_input_sequence()

    assert manifest["handoff_format"] == "hm3d"
    assert manifest["producer_dataset_arg"] == "hm3d"
    assert writer.real_backend_input_root.joinpath("rgb", "0.png").exists()
    assert writer.real_backend_input_root.joinpath("depth", "0.png").exists()
    assert writer.real_backend_input_root.joinpath("pose", "0.txt").exists()
    assert writer.real_backend_input_root.joinpath("boxfusion_hm3d_config.yaml").exists()
    assert not writer.real_backend_input_root.joinpath("all_poses.npy").exists()
    config_text = writer.real_backend_input_root.joinpath("boxfusion_hm3d_config.yaml").read_text(encoding="utf-8")
    assert "dataset: hm3d" in config_text
    assert "fx: 1.0" in config_text
    assert "cy: 1.0" in config_text
    assert manifest["hm3d_pose_frame_conversion"]["materialization_inverse"].startswith("pose_txt = inv")


def test_real_backend_mode_materializes_ca1m_input_and_can_fallback_to_stub(tmp_path: Path) -> None:
    writer = StageASimulationSnapshotWriter(
        output_root=tmp_path / "runs",
        coordination_root=tmp_path / "coordination",
        sequence_name="sim_real_backend_sequence",
        backend_mode="real",
        real_backend_handoff_format="ca1m",
        fallback_to_stub_on_real_backend_failure=True,
        real_backend_command=f"{sys.executable} -c \"import sys; sys.exit(7)\"",
    )

    writer.record_sample(
        rgb=_rgb(1.0),
        depth=_depth(1.0),
        camera_info=_camera_info(1.0),
        pose=_pose(1.0, 0.0),
        refresh_coordinator=False,
    )

    refresh = writer.refresh_current_backend()
    handoff = json.loads(writer.logs_dir.joinpath("simulation_ingress_backend_handoff.json").read_text(encoding="utf-8"))

    assert refresh.source_scene_root == writer.scene_root
    assert handoff["real_backend_handoff_format"] == "ca1m"
    assert handoff["status"] == "failed"
    assert handoff["producer_returncode"] == 7
    assert handoff["fallback_to_stub_on_real_backend_failure"] is True
    assert writer.real_backend_input_root.joinpath("rgb", "0.png").exists()
    assert writer.real_backend_input_root.joinpath("depth", "0.png").exists()
    assert writer.real_backend_input_root.joinpath("all_poses.npy").exists()
    assert writer.real_backend_input_root.joinpath("K_depth.txt").exists()
    assert writer.real_backend_input_root.joinpath("boxfusion_ca1m_config.yaml").exists()

    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(refresh.latest_scene_root)
    topology_payload = dict(query_backend.get_topology().payload["payload"]["topology"])
    assert [room["id"] for room in topology_payload["rooms"]] == ["room_1"]
