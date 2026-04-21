from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple


RUNTIME_ARTIFACT_MODE_BENCHMARK = "benchmark"
RUNTIME_ARTIFACT_MODE_SERVICE = "service"
RUNTIME_ARTIFACT_MODE_DEBUG = "debug"
RUNTIME_ARTIFACT_MODES: Tuple[str, ...] = (
    RUNTIME_ARTIFACT_MODE_BENCHMARK,
    RUNTIME_ARTIFACT_MODE_SERVICE,
    RUNTIME_ARTIFACT_MODE_DEBUG,
)


@dataclass(frozen=True)
class RuntimeArtifactPolicy:
    mode: str
    core_only: bool
    prepare_gt_visualization_pointcloud: bool
    save_scene_graph_visualizations: bool
    materialize_scene_graph_graphml: bool
    full_rgb_replay: bool
    readonly_tail_reference_audit: bool
    write_debug_room_artifacts: bool
    save_point_cloud: bool
    optional_demo_artifacts: bool
    materialize_rich_service_debug_artifacts: bool
    deferred_artifact_work: Tuple[str, ...]
    notes: Tuple[str, ...]

    @property
    def service_mode(self) -> bool:
        return self.mode == RUNTIME_ARTIFACT_MODE_SERVICE

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "service_mode": bool(self.service_mode),
            "core_only": bool(self.core_only),
            "prepare_gt_visualization_pointcloud": bool(self.prepare_gt_visualization_pointcloud),
            "save_scene_graph_visualizations": bool(self.save_scene_graph_visualizations),
            "materialize_scene_graph_graphml": bool(self.materialize_scene_graph_graphml),
            "full_rgb_replay": bool(self.full_rgb_replay),
            "readonly_tail_reference_audit": bool(self.readonly_tail_reference_audit),
            "write_debug_room_artifacts": bool(self.write_debug_room_artifacts),
            "save_point_cloud": bool(self.save_point_cloud),
            "optional_demo_artifacts": bool(self.optional_demo_artifacts),
            "materialize_rich_service_debug_artifacts": bool(self.materialize_rich_service_debug_artifacts),
            "deferred_artifact_work": list(self.deferred_artifact_work),
            "notes": list(self.notes),
        }


def normalize_runtime_artifact_mode(
    mode: Optional[Any],
    *,
    service_mode: bool = False,
) -> str:
    if bool(service_mode):
        return RUNTIME_ARTIFACT_MODE_SERVICE
    normalized = str(mode or RUNTIME_ARTIFACT_MODE_BENCHMARK).strip().lower()
    if normalized == "full_artifact":
        normalized = RUNTIME_ARTIFACT_MODE_BENCHMARK
    if normalized not in RUNTIME_ARTIFACT_MODES:
        joined = ", ".join(RUNTIME_ARTIFACT_MODES)
        raise ValueError(f"runtime artifact mode must be one of: {joined}")
    return normalized


def resolve_runtime_artifact_policy(
    *,
    mode: Optional[Any] = None,
    service_mode: bool = False,
    core_only: bool = False,
    requested_gt_visualization: bool = True,
    requested_scene_graph_vis: bool = False,
    requested_full_rgb_replay: bool = False,
    requested_readonly_tail_reference_audit: bool = False,
    suppress_service_debug_artifacts: bool = False,
) -> RuntimeArtifactPolicy:
    normalized_mode = normalize_runtime_artifact_mode(mode, service_mode=service_mode)
    service = normalized_mode == RUNTIME_ARTIFACT_MODE_SERVICE
    effective_core_only = bool(core_only or service)
    optional_demo_artifacts = not effective_core_only
    materialize_rich_service_debug_artifacts = not bool(suppress_service_debug_artifacts)
    rich_service_debug_deferred = (
        "room_scoped_runtime_state_json",
        "full_vector_map_snapshot_json",
        "working_topology_json",
        "working_vs_committed_topology_report_json",
        "working_vs_committed_topology_timeline_json",
        "working_vs_committed_topology_timeline_md",
        "room_commit_diagnosis_json",
        "room_commit_diagnosis_md",
    )

    if service:
        deferred = (
            "gt_visualization_pointcloud",
            "scene_graph_png",
            "topology_graphml",
            "full_rgb_replay",
            "readonly_tail_reference_audit",
            "debug_room_artifacts",
            "global_point_cloud_ply",
        )
        if not materialize_rich_service_debug_artifacts:
            deferred = deferred + rich_service_debug_deferred
        return RuntimeArtifactPolicy(
            mode=normalized_mode,
            core_only=True,
            prepare_gt_visualization_pointcloud=False,
            save_scene_graph_visualizations=False,
            materialize_scene_graph_graphml=False,
            full_rgb_replay=False,
            readonly_tail_reference_audit=False,
            write_debug_room_artifacts=False,
            save_point_cloud=False,
            optional_demo_artifacts=False,
            materialize_rich_service_debug_artifacts=materialize_rich_service_debug_artifacts,
            deferred_artifact_work=deferred,
            notes=(
                "Service mode keeps runtime state and committed/public query outputs, but skips artifact-only visualization work by default.",
                "The optional service/debug suppression switch trims rich diagnostic materialization while leaving the authoritative committed/public bundle unchanged.",
                "Benchmark/debug modes retain the previous default behavior unless core_only is explicitly requested.",
            ),
        )

    deferred = rich_service_debug_deferred if not materialize_rich_service_debug_artifacts else ()
    return RuntimeArtifactPolicy(
        mode=normalized_mode,
        core_only=effective_core_only,
        prepare_gt_visualization_pointcloud=bool(requested_gt_visualization),
        save_scene_graph_visualizations=bool(requested_scene_graph_vis and optional_demo_artifacts),
        materialize_scene_graph_graphml=bool(optional_demo_artifacts),
        full_rgb_replay=bool(requested_full_rgb_replay and optional_demo_artifacts),
        readonly_tail_reference_audit=bool(requested_readonly_tail_reference_audit),
        write_debug_room_artifacts=bool(optional_demo_artifacts),
        save_point_cloud=bool(optional_demo_artifacts),
        optional_demo_artifacts=bool(optional_demo_artifacts),
        materialize_rich_service_debug_artifacts=materialize_rich_service_debug_artifacts,
        deferred_artifact_work=deferred,
        notes=(
            "This mode preserves the existing synchronous export path as authoritative.",
            "The optional service/debug suppression switch trims only rich non-authoritative debug/service artifacts and keeps the committed/public benchmark-facing contract intact.",
            "core_only still suppresses optional demo artifacts without changing public committed-query semantics.",
        ),
    )


def artifact_policy_from_mapping(payload: Dict[str, Any]) -> RuntimeArtifactPolicy:
    data = dict(payload or {})
    return RuntimeArtifactPolicy(
        mode=str(data.get("mode") or RUNTIME_ARTIFACT_MODE_BENCHMARK),
        core_only=bool(data.get("core_only", False)),
        prepare_gt_visualization_pointcloud=bool(data.get("prepare_gt_visualization_pointcloud", True)),
        save_scene_graph_visualizations=bool(data.get("save_scene_graph_visualizations", False)),
        materialize_scene_graph_graphml=bool(data.get("materialize_scene_graph_graphml", False)),
        full_rgb_replay=bool(data.get("full_rgb_replay", False)),
        readonly_tail_reference_audit=bool(data.get("readonly_tail_reference_audit", False)),
        write_debug_room_artifacts=bool(data.get("write_debug_room_artifacts", False)),
        save_point_cloud=bool(data.get("save_point_cloud", False)),
        optional_demo_artifacts=bool(data.get("optional_demo_artifacts", False)),
        materialize_rich_service_debug_artifacts=bool(data.get("materialize_rich_service_debug_artifacts", True)),
        deferred_artifact_work=tuple(str(item) for item in data.get("deferred_artifact_work", []) or []),
        notes=tuple(str(item) for item in data.get("notes", []) or []),
    )
