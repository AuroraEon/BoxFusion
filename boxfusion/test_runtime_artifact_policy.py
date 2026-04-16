from __future__ import annotations

import argparse

from boxfusion.runtime_artifact_policy import (
    RUNTIME_ARTIFACT_MODE_BENCHMARK,
    RUNTIME_ARTIFACT_MODE_SERVICE,
    resolve_runtime_artifact_policy,
)
from stage_a_demo import build_runtime_artifact_policy_from_args


def test_service_mode_disables_artifact_only_work_by_default() -> None:
    policy = resolve_runtime_artifact_policy(
        mode=RUNTIME_ARTIFACT_MODE_SERVICE,
        core_only=False,
        requested_gt_visualization=True,
        requested_scene_graph_vis=True,
        requested_full_rgb_replay=True,
        requested_readonly_tail_reference_audit=True,
    )

    assert policy.service_mode is True
    assert policy.core_only is True
    assert policy.optional_demo_artifacts is False
    assert policy.prepare_gt_visualization_pointcloud is False
    assert policy.save_scene_graph_visualizations is False
    assert policy.materialize_scene_graph_graphml is False
    assert policy.full_rgb_replay is False
    assert policy.readonly_tail_reference_audit is False
    assert policy.write_debug_room_artifacts is False
    assert policy.save_point_cloud is False
    assert "gt_visualization_pointcloud" in policy.deferred_artifact_work
    assert "debug_room_artifacts" in policy.deferred_artifact_work


def test_benchmark_mode_preserves_existing_requested_defaults() -> None:
    policy = resolve_runtime_artifact_policy(
        mode=RUNTIME_ARTIFACT_MODE_BENCHMARK,
        core_only=False,
        requested_gt_visualization=True,
        requested_scene_graph_vis=True,
        requested_full_rgb_replay=True,
        requested_readonly_tail_reference_audit=True,
    )

    assert policy.service_mode is False
    assert policy.core_only is False
    assert policy.optional_demo_artifacts is True
    assert policy.prepare_gt_visualization_pointcloud is True
    assert policy.save_scene_graph_visualizations is True
    assert policy.materialize_scene_graph_graphml is True
    assert policy.full_rgb_replay is True
    assert policy.readonly_tail_reference_audit is True
    assert policy.write_debug_room_artifacts is True
    assert policy.save_point_cloud is True


def test_stage_a_launcher_policy_helper_wires_service_switch() -> None:
    args = argparse.Namespace(
        runtime_artifact_mode=RUNTIME_ARTIFACT_MODE_BENCHMARK,
        service_mode=True,
        core_only=False,
        viz_on_gt_points=True,
        save_scene_graph_vis=True,
        full_rgb_replay=True,
        enable_readonly_tail_reference_audit=True,
    )

    policy = build_runtime_artifact_policy_from_args(args)

    assert policy.mode == RUNTIME_ARTIFACT_MODE_SERVICE
    assert policy.to_dict()["service_mode"] is True
    assert policy.to_dict()["prepare_gt_visualization_pointcloud"] is False
