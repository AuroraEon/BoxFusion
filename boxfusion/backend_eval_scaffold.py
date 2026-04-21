from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional, Sequence

from boxfusion.artifact_contract import (
    active_artifact_profile,
    artifact_semantics_for_key,
    artifact_surface_for_key,
    build_artifact_contract,
    load_timeline_rows,
    summarize_artifact_surfaces,
    summarize_timeline_rows,
    surface_flags,
)


ACTIVE_HM3D_DATASET_ROOT = Path("/media/aurora/Program/dataset/hm3dsem_walks/val")
ACTIVE_SEQUENCE_NAMES: Sequence[str] = (
    "00824-Dd4bFSTQ8gi",
    "00829-QaLdnwvtxbs",
    "00843-DYehNKdT76V",
    "00861-GLAQ4DNUx5U",
    "00862-LT9Jq6dN3Ea",
    "00873-bxsVRursffK",
    "00877-4ok3usBNeis",
    "00890-6s7QHgap2fW",
)
HOVSG_OVERLAP_SCENE_IDS = {
    "00824",
    "00829",
    "00843",
    "00861",
    "00862",
    "00873",
    "00877",
    "00890",
}
DEFAULT_CANONICAL_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_2_final")
DEFAULT_SCENE_OUTPUT_ROOT = DEFAULT_CANONICAL_OUTPUT_ROOT / "scenes"
DEFAULT_LEGACY_SCENE_OUTPUT_ROOT = Path("stage_a_outputs_vt_fallback_v01_rerun2")
DEFAULT_EVAL_OUTPUT_ROOT = DEFAULT_CANONICAL_OUTPUT_ROOT / "eval" / "backend_eval_v0_1"
CORE_BACKEND_ARTIFACT_SPECS: Sequence[Dict[str, Any]] = (
    {
        "key": "manifest_json",
        "relative_path": "manifest.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "scene_summary_json",
        "relative_path": "logs/summary.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "topology_json",
        "relative_path": "logs/topology_v0_1.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "topology_query_report_json",
        "relative_path": "logs/topology_query_report.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "vertical_transition_evidence_json",
        "relative_path": "logs/vertical_transition_evidence.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "floor_diagnostics_summary_json",
        "relative_path": "logs/floor_diagnostics_summary.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": True,
    },
    {
        "key": "runtime_growth_profile_csv",
        "relative_path": "logs/runtime_growth_profile.csv",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "runtime_growth_profile_json",
        "relative_path": "logs/runtime_growth_profile.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "online_topology_lifecycle_json",
        "relative_path": "logs/online_topology_lifecycle_v0_1.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "working_topology_json",
        "relative_path": "logs/working_topology_v0_1.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "working_vs_committed_topology_report_json",
        "relative_path": "logs/working_vs_committed_topology_report_v0_1.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "working_vs_committed_topology_timeline_json",
        "relative_path": "logs/working_vs_committed_topology_timeline_v0_1.json",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
    {
        "key": "working_vs_committed_topology_timeline_md",
        "relative_path": "logs/working_vs_committed_topology_timeline_v0_1.md",
        "artifact_tier": "tier1_core_backend",
        "required_for_backend_eval": False,
    },
)
OPTIONAL_DEMO_ARTIFACT_SPECS: Sequence[Dict[str, Any]] = (
    {
        "key": "topology_graphml",
        "relative_path": "logs/topology_v0_1.graphml",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "timeline_json",
        "relative_path": "logs/timeline.json",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "timeline_csv",
        "relative_path": "logs/timeline.csv",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "room_segmentation_diagnostics_json",
        "relative_path": "logs/room_segmentation_diagnostics.json",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "final_bev_png",
        "relative_path": "final/{sequence_name}_final_bev.png",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "final_split_png",
        "relative_path": "final/{sequence_name}_final_split.png",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "final_demo_mp4",
        "relative_path": "final/{sequence_name}_closed_loop_demo.mp4",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
    {
        "key": "scene_report_md",
        "relative_path": "report.md",
        "artifact_tier": "tier2_optional_demo",
        "required_for_backend_eval": False,
    },
)
SCENE_ARTIFACT_SPECS: Sequence[Dict[str, Any]] = tuple(CORE_BACKEND_ARTIFACT_SPECS) + tuple(OPTIONAL_DEMO_ARTIFACT_SPECS)
BACKEND_REQUIRED_EXPORT_ARTIFACT_KEYS: Sequence[str] = tuple(
    spec["key"]
    for spec in CORE_BACKEND_ARTIFACT_SPECS
    if spec["key"] != "manifest_json" and bool(spec["required_for_backend_eval"])
)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def sequence_short_id(sequence_name: str) -> str:
    return str(sequence_name).split("-", 1)[0]


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def dump_jsonl(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True) + "\n")


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(dict(json.loads(line)))
    return rows


def file_size_bytes(path: Path) -> int:
    if not path.exists() or not path.is_file():
        return 0
    return int(path.stat().st_size)


def percentile(sorted_values: Sequence[float], q: float) -> Optional[float]:
    values = [float(item) for item in sorted_values]
    if not values:
        return None
    if len(values) == 1:
        return round(values[0], 3)
    q = min(max(float(q), 0.0), 1.0)
    idx = q * (len(values) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(values) - 1)
    weight = idx - lo
    return round(values[lo] * (1.0 - weight) + values[hi] * weight, 3)


def latency_summary_ms(values_ms: Sequence[float]) -> Dict[str, Optional[float]]:
    cleaned = sorted(float(item) for item in values_ms)
    if not cleaned:
        return {"mean_ms": None, "p50_ms": None, "p90_ms": None}
    return {
        "mean_ms": round(mean(cleaned), 3),
        "p50_ms": percentile(cleaned, 0.50),
        "p90_ms": percentile(cleaned, 0.90),
    }


def find_scene_root(
    sequence_name: str,
    *,
    preferred_root: Path,
    fallback_roots: Optional[Sequence[Path]] = None,
) -> Optional[Path]:
    candidates = [Path(preferred_root) / sequence_name]
    for root in fallback_roots or []:
        candidate = Path(root) / sequence_name
        if candidate not in candidates:
            candidates.append(candidate)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def scene_artifact_specs(scene_root: Path) -> List[Dict[str, Any]]:
    sequence_name = scene_root.name
    specs: List[Dict[str, Any]] = []
    for spec in SCENE_ARTIFACT_SPECS:
        resolved = dict(spec)
        resolved["path"] = scene_root / str(spec["relative_path"]).format(sequence_name=sequence_name)
        specs.append(resolved)
    return specs


def expected_scene_artifacts(scene_root: Path) -> Dict[str, Path]:
    return {str(spec["key"]): Path(spec["path"]) for spec in scene_artifact_specs(scene_root)}


def _artifact_record(scene_root: Path, spec: Dict[str, Any]) -> Dict[str, Any]:
    artifact_path = Path(spec["path"])
    exists = artifact_path.exists()
    surface = artifact_surface_for_key(str(spec["key"]))
    record = {
        "path": str(artifact_path),
        "relative_path": str(artifact_path.relative_to(scene_root)),
        "exists": bool(exists),
        "size_bytes": file_size_bytes(artifact_path),
        "artifact_tier": str(spec["artifact_tier"]),
        "required_for_backend_eval": bool(spec["required_for_backend_eval"]),
    }
    record.update(surface_flags(surface))
    record["artifact_semantics"] = artifact_semantics_for_key(str(spec["key"]))
    return record


def collect_scene_manifest(
    scene_root: Path,
    *,
    dataset_root: Optional[Path] = None,
    sequence_name: Optional[str] = None,
) -> Dict[str, Any]:
    scene_root = Path(scene_root)
    sequence_name = str(sequence_name or scene_root.name)
    artifacts = expected_scene_artifacts(scene_root)
    artifact_specs = scene_artifact_specs(scene_root)

    summary = load_json(artifacts["scene_summary_json"]) if artifacts["scene_summary_json"].exists() else {}
    topology = load_json(artifacts["topology_json"]) if artifacts["topology_json"].exists() else {}
    topology_report = load_json(artifacts["topology_query_report_json"]) if artifacts["topology_query_report_json"].exists() else {}
    floor_diag = load_json(artifacts["floor_diagnostics_summary_json"]) if artifacts["floor_diagnostics_summary_json"].exists() else {}
    vertical = load_json(artifacts["vertical_transition_evidence_json"]) if artifacts["vertical_transition_evidence_json"].exists() else {}
    working_vs_committed = (
        load_json(artifacts["working_vs_committed_topology_report_json"])
        if artifacts["working_vs_committed_topology_report_json"].exists()
        else {}
    )
    runtime_growth_summary = dict(summary.get("runtime_growth_summary") or {})
    artifact_profile = str(summary.get("artifact_profile") or active_artifact_profile(core_only=bool(summary.get("core_only_mode"))))
    timeline_rows = load_timeline_rows(artifacts["timeline_json"]) if artifacts["timeline_json"].exists() else []
    timeline_summary = summarize_timeline_rows(
        timeline_rows,
        timeline_path=artifacts["timeline_json"] if artifacts["timeline_json"].exists() else None,
    )

    floor_count = None
    if floor_diag.get("per_floor"):
        floor_count = int(len(floor_diag.get("per_floor", [])))
    elif topology_report.get("floor_count") is not None:
        floor_count = int(topology_report["floor_count"])
    elif topology.get("floors"):
        floor_count = int(len(topology.get("floors", [])))

    vertical_transition_count = None
    if dict(vertical.get("summary") or {}).get("count") is not None:
        vertical_transition_count = int(dict(vertical.get("summary") or {}).get("count"))
    elif summary.get("final_vertical_transition_count") is not None:
        vertical_transition_count = int(summary["final_vertical_transition_count"])

    room_count = topology_report.get("room_count")
    object_count = topology_report.get("object_count")
    anchor_count = topology_report.get("anchor_count")
    edge_count = topology_report.get("edge_count")
    object_label_count = topology_report.get("object_label_count")
    node_count = None
    if room_count is not None or object_count is not None or anchor_count is not None:
        node_count = int(room_count or 0) + int(object_count or 0) + int(anchor_count or 0)

    committed_projection = dict(working_vs_committed.get("committed_topology_projection") or {})
    working_projection = dict(working_vs_committed.get("working_topology") or {})
    withheld_summary = dict(working_vs_committed.get("withheld_topology_summary") or {})
    difference_summary = dict(working_vs_committed.get("difference_summary") or {})
    topology_comparison_summary = {
        "public_room_count": None if committed_projection.get("room_count") is None else int(committed_projection.get("room_count")),
        "public_edge_count": None if committed_projection.get("edge_count") is None else int(committed_projection.get("edge_count")),
        "public_gateway_count": None if committed_projection.get("gateway_count") is None else int(committed_projection.get("gateway_count")),
        "working_room_count": None if working_projection.get("room_count") is None else int(working_projection.get("room_count")),
        "working_edge_count": None if working_projection.get("edge_count") is None else int(working_projection.get("edge_count")),
        "working_gateway_count": None if working_projection.get("gateway_count") is None else int(working_projection.get("gateway_count")),
        "withheld_room_count": None if withheld_summary.get("withheld_room_count") is None else int(withheld_summary.get("withheld_room_count")),
        "withheld_edge_count": None if withheld_summary.get("withheld_edge_count") is None else int(withheld_summary.get("withheld_edge_count")),
        "withheld_gateway_count": None if withheld_summary.get("withheld_gateway_count") is None else int(withheld_summary.get("withheld_gateway_count")),
        "working_only_room_count": None if difference_summary.get("working_only_room_count") is None else int(difference_summary.get("working_only_room_count")),
        "edge_count_difference": None if difference_summary.get("edge_count_difference") is None else int(difference_summary.get("edge_count_difference")),
    }
    query_support_summary = {
        "room_count": None if topology_report.get("room_count") is None else int(topology_report.get("room_count")),
        "edge_count": None if topology_report.get("edge_count") is None else int(topology_report.get("edge_count")),
        "object_count": None if topology_report.get("object_count") is None else int(topology_report.get("object_count")),
        "anchor_count": None if topology_report.get("anchor_count") is None else int(topology_report.get("anchor_count")),
        "object_label_count": None if object_label_count is None else int(object_label_count),
        "capabilities": dict(topology_report.get("capabilities") or {}),
    }

    artifact_records = {
        str(spec["key"]): _artifact_record(scene_root, spec)
        for spec in artifact_specs
    }
    artifact_surface_summary = summarize_artifact_surfaces(artifact_records)
    tier1_keys = [str(spec["key"]) for spec in artifact_specs if str(spec["artifact_tier"]) == "tier1_core_backend"]
    tier2_keys = [str(spec["key"]) for spec in artifact_specs if str(spec["artifact_tier"]) == "tier2_optional_demo"]
    required_backend_keys = [str(spec["key"]) for spec in artifact_specs if bool(spec["required_for_backend_eval"])]
    available_backend_keys = [key for key in required_backend_keys if artifact_records.get(key, {}).get("exists")]
    missing_backend_keys = [key for key in required_backend_keys if key not in available_backend_keys]
    backend_artifact_size_total_bytes = int(sum(artifact_records[key]["size_bytes"] for key in tier1_keys))
    optional_demo_artifact_size_total_bytes = int(sum(artifact_records[key]["size_bytes"] for key in tier2_keys))
    artifact_size_total_bytes = int(sum(item["size_bytes"] for item in artifact_records.values()))

    objects_per_room = None
    anchors_per_room = None
    bytes_per_room = None
    all_tier_bytes_per_room = None
    if room_count:
        objects_per_room = round(float(object_count or 0) / float(room_count), 3)
        anchors_per_room = round(float(anchor_count or 0) / float(room_count), 3)
        bytes_per_room = round(float(backend_artifact_size_total_bytes) / float(room_count), 3)
        all_tier_bytes_per_room = round(float(artifact_size_total_bytes) / float(room_count), 3)

    notes: List[str] = []
    status = "missing_scene_root"
    if scene_root.exists():
        status = "partial_artifacts"
    if all(artifact_records.get(key, {}).get("exists") for key in BACKEND_REQUIRED_EXPORT_ARTIFACT_KEYS):
        status = "ready"
    if summary.get("topology_export_error"):
        status = "topology_export_error"
        notes.append(str(summary.get("topology_export_error")))
    if missing_backend_keys:
        notes.append("missing_backend_artifacts:" + ",".join(sorted(missing_backend_keys)))

    artifact_contract = build_artifact_contract(
        artifact_profile=artifact_profile,
        timeline_summary=timeline_summary,
        topology_json_exists=artifact_records.get("topology_json", {}).get("exists", False),
        topology_query_report_exists=artifact_records.get("topology_query_report_json", {}).get("exists", False),
        online_topology_lifecycle_exists=artifact_records.get("online_topology_lifecycle_json", {}).get("exists", False),
        working_vs_committed_timeline_exists=artifact_records.get("working_vs_committed_topology_timeline_json", {}).get("exists", False),
    )

    manifest = {
        "version": "0.1",
        "generated_at_utc": utc_now_iso(),
        "scene_id": sequence_short_id(sequence_name),
        "sequence_name": sequence_name,
        "scene_root": str(scene_root),
        "dataset_root": None if dataset_root is None else str(dataset_root),
        "dataset_sequence_dir": None if dataset_root is None else str(Path(dataset_root) / sequence_name),
        "artifact_profile": artifact_profile,
        "status": status,
        "notes": notes,
        "world_model_summary": {
            "floor_count": floor_count,
            "room_count": None if room_count is None else int(room_count),
            "object_count": None if object_count is None else int(object_count),
            "anchor_count": None if anchor_count is None else int(anchor_count),
            "edge_count": None if edge_count is None else int(edge_count),
            "node_count": node_count,
            "vertical_transition_count": vertical_transition_count,
            "vertical_transitions_present": None if vertical_transition_count is None else bool(vertical_transition_count > 0),
        },
        "runtime_summary": {
            "processed_frames": summary.get("processed_frames"),
            "duration_sec": summary.get("duration_sec"),
            "average_fps": summary.get("average_fps"),
            "output_mode": summary.get("output_mode"),
            "artifact_profile": artifact_profile,
            "core_only_mode": summary.get("core_only_mode"),
            "snapshot_count": summary.get("snapshot_count"),
            "timeline_frame_count": timeline_summary.get("timeline_frame_count"),
            "snapshot_timeline_frame_count": timeline_summary.get("snapshot_frame_count"),
            "dense_replay_frame_count": timeline_summary.get("dense_replay_frame_count"),
            "runtime_growth_profile_sample_count": runtime_growth_summary.get("sample_count"),
            "runtime_growth_profile_interval_frames": runtime_growth_summary.get("profile_interval_frames"),
            "runtime_growth_risk_flag": runtime_growth_summary.get("runtime_risk_flag"),
            "runtime_growth_risk_reasons": runtime_growth_summary.get("runtime_risk_reasons"),
            "runtime_growth_total_step_ratio": dict(runtime_growth_summary.get("growth_ratios") or {}).get("total_step_sec"),
            "runtime_growth_topology_ratio": dict(runtime_growth_summary.get("growth_ratios") or {}).get("topology_room_segmentation_sec"),
            "runtime_growth_feature_boxfusion_ratio": dict(runtime_growth_summary.get("growth_ratios") or {}).get("feature_boxfusion_sec"),
            "backend_artifact_size_total_bytes": backend_artifact_size_total_bytes,
            "optional_demo_artifact_size_total_bytes": optional_demo_artifact_size_total_bytes,
            "artifact_size_total_bytes": artifact_size_total_bytes,
        },
        "compactness_summary": {
            "objects_per_room": objects_per_room,
            "anchors_per_room": anchors_per_room,
            "bytes_per_room": bytes_per_room,
            "all_tier_bytes_per_room": all_tier_bytes_per_room,
        },
        "topology_comparison_summary": topology_comparison_summary,
        "query_support_summary": query_support_summary,
        "artifact_policy": {
            "tier1_label": "core_backend_artifacts",
            "tier2_label": "optional_demo_artifacts",
            "backend_eval_relies_on_tier1_only": True,
            "legacy_root_policy": "migration_only_read_only_fallback",
        },
        "artifact_contract": artifact_contract,
        "artifact_capabilities": dict(artifact_contract.get("capabilities") or {}),
        "artifact_surfaces": artifact_surface_summary,
        "artifact_tiers": {
            "tier1_core_backend": {
                "artifact_keys": tier1_keys,
                "required_for_backend_eval": True,
                "available_count": sum(1 for key in tier1_keys if artifact_records.get(key, {}).get("exists")),
                "size_bytes": backend_artifact_size_total_bytes,
            },
            "tier2_optional_demo": {
                "artifact_keys": tier2_keys,
                "required_for_backend_eval": False,
                "available_count": sum(1 for key in tier2_keys if artifact_records.get(key, {}).get("exists")),
                "size_bytes": optional_demo_artifact_size_total_bytes,
            },
        },
        "artifacts": artifact_records,
    }
    return manifest


def write_scene_manifest(
    scene_root: Path,
    *,
    dataset_root: Optional[Path] = None,
    sequence_name: Optional[str] = None,
) -> Dict[str, Any]:
    initial_manifest = collect_scene_manifest(
        scene_root,
        dataset_root=dataset_root,
        sequence_name=sequence_name,
    )
    manifest_path = Path(scene_root) / "manifest.json"
    dump_json(manifest_path, initial_manifest)
    manifest = collect_scene_manifest(
        scene_root,
        dataset_root=dataset_root,
        sequence_name=sequence_name,
    )
    dump_json(manifest_path, manifest)
    return manifest


def build_scene_registry(
    *,
    dataset_root: Path = ACTIVE_HM3D_DATASET_ROOT,
    sequence_names: Sequence[str] = ACTIVE_SEQUENCE_NAMES,
    scene_output_root: Path = DEFAULT_SCENE_OUTPUT_ROOT,
    legacy_scene_output_root: Optional[Path] = None,
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    fallback_roots = [legacy_scene_output_root] if legacy_scene_output_root else []

    for sequence_name in sequence_names:
        planned_root = Path(scene_output_root) / sequence_name
        discovered_root = find_scene_root(
            sequence_name,
            preferred_root=scene_output_root,
            fallback_roots=[root for root in fallback_roots if root is not None],
        )
        if discovered_root is not None:
            manifest = collect_scene_manifest(
                discovered_root,
                dataset_root=dataset_root,
                sequence_name=sequence_name,
            )
            manifest_path = planned_root / "manifest.json"
            discovered_manifest_path = discovered_root / "manifest.json"
            if discovered_root == planned_root and not discovered_manifest_path.exists():
                dump_json(manifest_path, manifest)
            artifact_source = "primary_root" if discovered_root == planned_root else "legacy_fallback"
        else:
            manifest = {
                "status": "pending_regeneration",
                "world_model_summary": {
                    "floor_count": None,
                    "vertical_transitions_present": None,
                },
                "notes": ["scene_output_missing"],
            }
            manifest_path = planned_root / "manifest.json"
            discovered_manifest_path = None
            artifact_source = "missing"

        rows.append(
            {
                "scene_id": sequence_short_id(sequence_name),
                "sequence_name": sequence_name,
                "dataset_root": str(dataset_root),
                "dataset_sequence_dir": str(Path(dataset_root) / sequence_name),
                "hovsg_overlap": bool(sequence_short_id(sequence_name) in HOVSG_OVERLAP_SCENE_IDS),
                "floor_count": dict(manifest.get("world_model_summary") or {}).get("floor_count"),
                "vertical_transitions_present": dict(manifest.get("world_model_summary") or {}).get("vertical_transitions_present"),
                "artifact_root": str(planned_root),
                "discovered_artifact_root": None if discovered_root is None else str(discovered_root),
                "manifest_path": str(manifest_path),
                "discovered_manifest_path": None if discovered_manifest_path is None else str(discovered_manifest_path),
                "artifact_source": artifact_source,
                "legacy_fallback_used": bool(discovered_root is not None and discovered_root != planned_root),
                "status": manifest.get("status"),
                "notes": list(manifest.get("notes") or []),
            }
        )

    return {
        "version": "0.1",
        "generated_at_utc": utc_now_iso(),
        "dataset_root": str(dataset_root),
        "scene_output_root": str(scene_output_root),
        "legacy_scene_output_root": None if legacy_scene_output_root is None else str(legacy_scene_output_root),
        "legacy_fallback_policy": "disabled_by_default_migration_only",
        "scenes": rows,
    }
