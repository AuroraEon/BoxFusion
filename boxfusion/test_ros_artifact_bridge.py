from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from boxfusion.ros_artifact_bridge import CommittedArtifactBundle, build_marker_specs, validate_coordinates
from boxfusion.route_to_waypoints import route_to_waypoints


SCENE_ROOT = Path("runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V")


def test_committed_artifact_bundle_loads_four_public_artifacts() -> None:
    bundle = CommittedArtifactBundle.from_scene_root(SCENE_ROOT)

    assert bundle.topology_path.name == "topology_v0_1.json"
    assert bundle.query_report_path.name == "topology_query_report.json"
    assert bundle.world_model_path.name == "committed_room_world_model_v0_1.json"
    assert bundle.snapshot_path.name == "committed_room_world_snapshot_v0_1.json"
    assert bundle.counts()["rooms"] == 11
    assert bundle.counts()["gateways"] == 12


def test_coordinate_validation_detects_risky_room_center_sources() -> None:
    bundle = CommittedArtifactBundle.from_scene_root(SCENE_ROOT)
    warnings = validate_coordinates(bundle)
    warning_codes = {warning["code"] for warning in warnings}

    assert "risky_room_center_sign_mismatch" in warning_codes


def test_route_to_waypoints_uses_gateway_when_pair_matches() -> None:
    bundle = CommittedArtifactBundle.from_scene_root(SCENE_ROOT)
    route = {"room_sequence": ["room_11", "room_9"], "used_relation_types": ["adjacent"]}

    result = route_to_waypoints(route, bundle)

    assert result["frame_id"] == "boxfusion_map"
    assert len(result["waypoints"]) == 3
    assert result["waypoints"][0]["source"]["artifact"] == "topology_v0_1.json"
    assert any(waypoint["kind"] == "gateway_demo_waypoint" for waypoint in result["waypoints"])
    assert all(waypoint["not_collision_free"] is True for waypoint in result["waypoints"])
    assert all(segment["not_collision_free"] is True for segment in result["segments"])


def test_route_to_waypoints_accepts_loaded_artifact_dicts() -> None:
    bundle = CommittedArtifactBundle.from_scene_root(SCENE_ROOT)

    result = route_to_waypoints(
        {"room_sequence": ["room_11", "room_9"], "used_relation_types": ["adjacent"]},
        {"topology": bundle.topology, "snapshot": bundle.snapshot, "world_model": bundle.world_model},
    )

    assert result["waypoints"][0]["room_id"] == "room_11"
    assert any(waypoint["kind"] == "gateway_demo_waypoint" for waypoint in result["waypoints"])


def test_marker_specs_are_ros_independent_and_grouped() -> None:
    bundle = CommittedArtifactBundle.from_scene_root(SCENE_ROOT)
    route_preview = route_to_waypoints(bundle.query_report["first_route"], bundle)

    marker_specs = build_marker_specs(bundle, route_waypoints=route_preview)

    assert "groups" in marker_specs
    assert marker_specs["counts"]["rooms"] >= bundle.counts()["rooms"]
    assert marker_specs["counts"]["gateways"] == bundle.counts()["gateways"]
    assert marker_specs["counts"]["waypoints"] == len(route_preview["waypoints"])
    sample_marker = marker_specs["groups"]["rooms"][0]
    assert sample_marker["frame_id"] == "boxfusion_map"
    assert sample_marker["visualization_only"] is True


def test_validation_cli_writes_preview_outputs(tmp_path: Path) -> None:
    cmd = [
        sys.executable,
        "tools/validate_ros_bridge_artifact_contract.py",
        "--scene-root",
        str(SCENE_ROOT),
        "--out-dir",
        str(tmp_path),
    ]
    subprocess.run(cmd, check=True, cwd=Path.cwd())

    report_path = tmp_path / "ros_bridge_artifact_contract_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["scene_count"] == 1
    row = report["rows"][0]
    assert row["scene_id"] == "00843-DYehNKdT76V"
    assert Path(row["outputs"]["marker_specs_preview"]).exists()
    assert Path(row["outputs"]["marker_specs"]).exists()
    assert Path(row["outputs"]["validation_warnings"]).exists()
    assert Path(row["outputs"]["route_waypoints_preview"]).exists()
