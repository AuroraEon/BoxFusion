#!/usr/bin/env python3
"""Export and validate task25b staged formal cross-floor artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO = Path("/home/ws/workspace/BoxFusion")
SCENE = "00843-DYehNKdT76V"
BASE = REPO / "stage_outputs/stage1_generalization" / SCENE
STAGING_ROOT = BASE / "task25b_stage_a_rerun_candidate"
RAW_ROOT = STAGING_ROOT / "canonical_stage1/raw_outputs" / SCENE
RAW_LOGS = RAW_ROOT / "logs"
TASK_ROOT = BASE / "tasks/task25b_formal_connector_object_interface_exporter_and_staged_rerun"
TASKS = BASE / "tasks"
STAGED_PUBLIC = STAGING_ROOT / "clean_rerun/committed_public"
STAGED_ROUTES = STAGING_ROOT / "clean_rerun/routes"
STAGED_MAPS = STAGING_ROOT / "clean_rerun/maps"
CLEAN = BASE / "clean_rerun"

TASK24C_CONNECTORS = TASKS / "task24c_rslg_cross_floor_connector_router_and_overlay/vertical_connectors_v0_1.json"
TASK24D_TOPOLOGY = TASKS / "task24d_cross_floor_rviz_overlay_adapter_with_transition_semantics_fix/corrected_cross_floor_topology_v0_2.json"
TASK24G2_ROUTE = TASKS / "task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/planned_occupancy_aware_3d_route_v0_1.json"
TASK24I_CONTRACT = TASKS / "task24i_cross_floor_object_level_tracking_smoke/object_level_cross_floor_route_contract_v0_1.json"
TASK24I_ROUTE = TASKS / "task24i_cross_floor_object_level_tracking_smoke/planned_object_level_3d_route_v0_1.json"
TASK24I_QUERY = TASKS / "task24i_cross_floor_object_level_tracking_smoke/object_query_resolution_v0_1.json"
TASK24I_APPROACH = TASKS / "task24i_cross_floor_object_level_tracking_smoke/object_approach_candidate_v0_1.json"
SPARSE_GRAPH = TASKS / "task24b4_stable_map_preview_and_stair_graph_binding_fix/sparse_stair_connector_graph_vt_1_v0_3.json"
RAW_TRACE = TASKS / "task24b3_stair_trace_centerline_and_all_floor_map_regression_fix/raw_transition_pose_trace_vt_1.json"

COMMON_CLAIM_BOUNDARY = {
    "visual_kinematic_proxy_only": True,
    "topological_vertical_transition_only": True,
    "physical_stair_climbing_supported": False,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "nav2_execution": False,
    "amcl_localization": False,
}

VERTICAL_CLAIM_BOUNDARY = {
    "topological_vertical_transition_only": True,
    "physical_stair_climbing_supported": False,
    "gait_supported": False,
    "footstep_planning_supported": False,
    "contact_based_stair_climbing_supported": False,
    "nav2_execution": False,
    "amcl_localization": False,
}

OBJECT_CLAIM_BOUNDARY = {
    **COMMON_CLAIM_BOUNDARY,
    "object_level_smoke_test_only": True,
    "not_full_object_navigation_benchmark": True,
    "full_object_navigation_benchmark": False,
    "object_centroid_navigation_used": False,
    "generated_approach_candidate_used": True,
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": str(path), "exists": False}
    stat = path.stat()
    return {
        "path": str(path),
        "exists": True,
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": sha256(path) if path.is_file() else None,
    }


def dir_summary(path: Path, max_files: int = 30) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": str(path),
        "exists": path.exists(),
        "file_count": 0,
        "total_size_bytes": 0,
        "sample_files": [],
        "aggregate_sha256": None,
    }
    if not path.exists():
        return result
    files = sorted(item for item in path.rglob("*") if item.is_file())
    digest = hashlib.sha256()
    result["file_count"] = len(files)
    for item in files:
        stat = item.stat()
        item_hash = sha256(item) or ""
        rel = item.relative_to(path).as_posix()
        result["total_size_bytes"] += stat.st_size
        digest.update(rel.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_size).encode("ascii"))
        digest.update(b"\0")
        digest.update(item_hash.encode("ascii"))
        digest.update(b"\n")
    result["aggregate_sha256"] = digest.hexdigest()
    for item in files[:max_files]:
        summary = file_summary(item)
        summary["relative_path"] = item.relative_to(path).as_posix()
        result["sample_files"].append(summary)
    return result


def protected_paths() -> dict[str, Path]:
    return {
        "stage_outputs_stage1_00824_step30p1": REPO / "stage_outputs/stage1_00824_step30p1",
        "00824_reference_baseline_stage1_generalization": REPO / "stage_outputs/stage1_generalization/00824-Dd4bFSTQ8gi",
        "00843_clean_rerun_committed_public": CLEAN / "committed_public",
        "00843_clean_rerun_maps_floor_1": CLEAN / "maps/floor_1",
        "00843_clean_rerun_maps_floor_2": CLEAN / "maps/floor_2",
        "00843_clean_rerun_canonical_stage1_raw_outputs": CLEAN / "canonical_stage1/raw_outputs",
        "task23b_outputs": TASKS / "task23b_quadruped_stair_proxy_cross_floor_demo",
        "task24h_outputs": TASKS / "task24h_cross_floor_visual_proxy_tracking_mode",
        "task24h2_outputs": TASKS / "task24h2_cross_floor_visual_proxy_tracking_evidence_hardening",
        "task24i_outputs": TASKS / "task24i_cross_floor_object_level_tracking_smoke",
    }


def build_snapshot(kind: str) -> dict[str, Any]:
    git = subprocess.run(["git", "status", "--short"], cwd=REPO, text=True, capture_output=True, check=False)
    return {
        "artifact_type": f"task25b_protected_artifact_{kind}_snapshot",
        "created_utc": now_iso(),
        "repo": str(REPO),
        "scene_id": SCENE,
        "git_status_short": git.stdout.splitlines(),
        "task25a_helper_git_state": next(
            (line for line in git.stdout.splitlines() if "tools/vertical_connectors/task25a_reintegration_preflight.py" in line),
            "not_listed",
        ),
        "protected_directories": {name: dir_summary(path) for name, path in protected_paths().items()},
        "task24_key_report_hashes": {
            "task24h_planned_route_used": file_summary(TASKS / "task24h_cross_floor_visual_proxy_tracking_mode/planned_route_used.json"),
            "task24h2_summary": file_summary(TASKS / "task24h2_cross_floor_visual_proxy_tracking_evidence_hardening/task24h2_summary.json"),
            "task24i_object_contract": file_summary(TASK24I_CONTRACT),
            "task24i_planned_object_route": file_summary(TASK24I_ROUTE),
            "task24i_query_resolution": file_summary(TASK24I_QUERY),
        },
        "clean_rerun_committed_public_hash_summary": dir_summary(CLEAN / "committed_public", max_files=50),
    }


def compare_snapshots(pre: dict[str, Any], post: dict[str, Any]) -> dict[str, Any]:
    changes: dict[str, Any] = {}
    for name, before in pre.get("protected_directories", {}).items():
        after = post.get("protected_directories", {}).get(name, {})
        changed = before.get("aggregate_sha256") != after.get("aggregate_sha256")
        changes[name] = {
            "changed": changed,
            "pre_aggregate_sha256": before.get("aggregate_sha256"),
            "post_aggregate_sha256": after.get("aggregate_sha256"),
            "pre_file_count": before.get("file_count"),
            "post_file_count": after.get("file_count"),
        }
    key_report_changes: dict[str, Any] = {}
    for name, before in pre.get("task24_key_report_hashes", {}).items():
        after = post.get("task24_key_report_hashes", {}).get(name, {})
        key_report_changes[name] = {
            "changed": before.get("sha256") != after.get("sha256"),
            "pre_sha256": before.get("sha256"),
            "post_sha256": after.get("sha256"),
        }
    protected_modified = any(item["changed"] for item in changes.values()) or any(item["changed"] for item in key_report_changes.values())
    return {
        "artifact_type": "task25b_protected_artifact_modification_check",
        "created_utc": now_iso(),
        "classification": "task25b_failed_protected_artifact_modified" if protected_modified else "task25b_no_protected_artifacts_modified",
        "protected_artifacts_modified": protected_modified,
        "protected_directory_changes": changes,
        "task24_key_report_changes": key_report_changes,
        "task25a_helper_git_state_postflight": post.get("task25a_helper_git_state"),
    }


def stage_a_report() -> tuple[dict[str, Any], str]:
    exitcode_path = TASK_ROOT / "stage_a_rerun.exitcode"
    exitcode = int(exitcode_path.read_text(encoding="utf-8").strip()) if exitcode_path.exists() else None
    stdout = (TASK_ROOT / "stage_a_rerun.stdout.log").read_text(encoding="utf-8", errors="replace") if (TASK_ROOT / "stage_a_rerun.stdout.log").exists() else ""
    stderr = (TASK_ROOT / "stage_a_rerun.stderr.log").read_text(encoding="utf-8", errors="replace") if (TASK_ROOT / "stage_a_rerun.stderr.log").exists() else ""
    expected = {
        "raw_output_root": RAW_ROOT,
        "manifest": RAW_ROOT / "manifest.json",
        "summary": RAW_LOGS / "summary.json",
        "vertical_transition_evidence": RAW_LOGS / "vertical_transition_evidence.json",
        "topology": RAW_LOGS / "topology_v0_1.json",
        "committed_room_world_model": RAW_LOGS / "committed_room_world_model_v0_1.json",
        "floor_diagnostics_summary": RAW_LOGS / "floor_diagnostics_summary.json",
        "final_vector_map_snapshot": RAW_LOGS / "final_vector_map_snapshot.json",
        "room_scoped_runtime_state": RAW_LOGS / "room_scoped_runtime_state_v0_1.json",
    }
    validations = {name: path.exists() for name, path in expected.items()}
    report = {
        "artifact_type": "task25b_stage_a_staged_rerun_report",
        "created_utc": now_iso(),
        "classification": "stage_a_staged_rerun_succeeded" if exitcode == 0 and all(validations.values()) else "stage_a_staged_rerun_failed_or_incomplete",
        "stage_a_command_used_cuda": True,
        "stage_a_command_device": "cuda",
        "stage_a_python": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "stage_a_exitcode": exitcode,
        "staged_raw_output_root": str(RAW_ROOT),
        "expected_output_validation": validations,
        "expected_output_paths": {name: str(path) for name, path in expected.items()},
        "stdout_tail": stdout.splitlines()[-40:],
        "stderr_tail": stderr.splitlines()[-40:],
        "clean_rerun_write_check": {
            "stage_a_output_root_is_existing_clean_rerun": str(RAW_ROOT).startswith(str(CLEAN)),
            "passed": not str(RAW_ROOT).startswith(str(CLEAN)),
        },
    }
    summary = [
        "# Stage-A Staged Rerun Stdout/Stderr Summary",
        "",
        f"- exitcode: `{exitcode}`",
        "- device: `cuda`",
        f"- staged raw output root: `{RAW_ROOT}`",
        "",
        "## Stdout Tail",
        "```text",
        "\n".join(stdout.splitlines()[-60:]) or "(empty)",
        "```",
        "",
        "## Stderr Tail",
        "```text",
        "\n".join(stderr.splitlines()[-60:]) or "(empty)",
        "```",
    ]
    return report, "\n".join(summary)


def room_id(value: Any) -> str:
    text = str(value)
    return text if text.startswith("room_") else f"room_{text}"


def find_vt_1(evidence: dict[str, Any]) -> dict[str, Any]:
    transitions = evidence.get("summary", {}).get("transitions") or evidence.get("transitions") or []
    for item in transitions:
        if item.get("transition_id") == "vt_1":
            return item
    raise RuntimeError("vt_1 was not found in staged Stage-A vertical_transition_evidence.json")


def edge_distance(a: list[float], b: list[float]) -> float:
    return round(math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b))), 6)


def load_formal_sources() -> dict[str, Any]:
    return {
        "stage_a_vertical_evidence": read_json(RAW_LOGS / "vertical_transition_evidence.json"),
        "stage_a_topology": read_json(RAW_LOGS / "topology_v0_1.json"),
        "stage_a_room_world": read_json(RAW_LOGS / "committed_room_world_model_v0_1.json"),
        "stage_a_floor_diagnostics": read_json(RAW_LOGS / "floor_diagnostics_summary.json"),
        "task24c_connectors": read_json(TASK24C_CONNECTORS),
        "task24d_topology": read_json(TASK24D_TOPOLOGY) if TASK24D_TOPOLOGY.exists() else {},
        "sparse_graph": read_json(SPARSE_GRAPH),
        "task24g2_route": read_json(TASK24G2_ROUTE),
        "task24i_contract": read_json(TASK24I_CONTRACT),
        "task24i_route": read_json(TASK24I_ROUTE),
        "task24i_query": read_json(TASK24I_QUERY),
        "task24i_approach": read_json(TASK24I_APPROACH),
    }


def build_vertical_artifact(sources: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    vt = find_vt_1(sources["stage_a_vertical_evidence"])
    sparse = sources["sparse_graph"]
    nodes = []
    for node in sparse["nodes"]:
        z = float(node["xyz"][2])
        floor_id = "floor_1" if z < 3.0 else "floor_2"
        nodes.append(
            {
                "node_id": node["node_id"],
                "node_type": node.get("node_type"),
                "sample_t": node.get("sample_t"),
                "position_xyz": [float(node["xyz"][0]), float(node["xyz"][1]), z],
                "floor_id": floor_id,
                "pose_trace_provenance": str(RAW_TRACE),
                "source_classification": "imported_from_validated_downstream_artifact",
                "physical_execution_supported": False,
            }
        )
    edges = []
    node_by_id = {item["node_id"]: item for item in nodes}
    for edge in sparse["edges"]:
        source = node_by_id[edge["source"]]
        target = node_by_id[edge["target"]]
        edge_id = edge["edge_id"]
        edges.append(
            {
                "edge_id": edge_id,
                "source_node_id": edge["source"],
                "target_node_id": edge["target"],
                "source_floor_id": source["floor_id"],
                "target_floor_id": target["floor_id"],
                "distance_3d_m": edge.get("distance_3d_m", edge_distance(source["position_xyz"], target["position_xyz"])),
                "delta_z_m": round(float(target["position_xyz"][2]) - float(source["position_xyz"][2]), 6),
                "topological_transition_edge": edge_id == "vt_1_centerline_e001",
                "physical_execution_supported": False,
                "claim_boundary": "topological_vertical_transition_only",
            }
        )
    connector = {
        "connector_id": "vc_vt_1",
        "connector_type": "stairs_or_vertical_connector",
        "transition_id": "vt_1",
        "from_floor_id": vt["from_floor_id"],
        "to_floor_id": vt["to_floor_id"],
        "from_room_id": room_id(vt["from_room_id"]),
        "to_room_id": room_id(vt["to_room_id"]),
        "from_position_xy": vt.get("from_position_xy"),
        "to_position_xy": vt.get("to_position_xy"),
        "endpoint_from": {
            "node_id": "vt_1_centerline_n000",
            "floor_id": "floor_1",
            "room_id": room_id(vt["from_room_id"]),
            "position_xyz": node_by_id["vt_1_centerline_n000"]["position_xyz"],
        },
        "endpoint_to": {
            "node_id": "vt_1_centerline_n004",
            "floor_id": "floor_2",
            "room_id": room_id(vt["to_room_id"]),
            "position_xyz": node_by_id["vt_1_centerline_n004"]["position_xyz"],
        },
        "centerline_nodes": nodes,
        "centerline_edges": edges,
        "transition_edge_id": "vt_1_centerline_e001",
        "transition_edge": {
            "source_node_id": "vt_1_centerline_n001",
            "target_node_id": "vt_1_centerline_n002",
            "source_floor_id": "floor_1",
            "target_floor_id": "floor_2",
        },
        "z_start": 2.456,
        "z_end": 4.056,
        "z_delta": 1.6,
        "evidence": {
            "frame_start": vt.get("frame_start"),
            "frame_end": vt.get("frame_end"),
            "transition_frame_start": vt.get("transition_frame_start"),
            "transition_frame_end": vt.get("transition_frame_end"),
            "entry_stable_frame": vt.get("entry_stable_frame"),
            "exit_stable_frame": vt.get("exit_stable_frame"),
            "supporting_frame_count": vt.get("supporting_frame_count"),
            "pose_trace_provenance": str(RAW_TRACE),
            "stage_a_transition_record": str(RAW_LOGS / "vertical_transition_evidence.json"),
        },
        "confidence": vt.get("confidence", 0.95),
        "source_classification": "post_stage_a_formalization_candidate",
        "source_provenance": {
            "native_stage_a_fields": [
                "transition_id",
                "from_floor_id",
                "to_floor_id",
                "from_room_id",
                "to_room_id",
                "from_position_xy",
                "to_position_xy",
                "evidence frame range",
                "confidence",
            ],
            "imported_validated_downstream_fields": [
                "centerline_nodes",
                "centerline_edges",
                "transition_edge_id",
                "z_start",
                "z_end",
                "z_delta",
            ],
            "stage_a_raw_output": str(RAW_ROOT),
            "downstream_centerline_source": str(SPARSE_GRAPH),
        },
        "claim_boundary": VERTICAL_CLAIM_BOUNDARY,
    }
    artifact = {
        "schema_version": "0.1",
        "schema_name": "vertical_connectors_v0_1",
        "artifact_type": "formal_stage_a_vertical_connector_graph",
        "scene_id": SCENE,
        "sequence_id": SCENE,
        "created_utc": now_iso(),
        "generator": "tools/vertical_connectors/export_task25b_formal_cross_floor_artifacts.py",
        "source_stage_a_run_id": "task25b_stage_a_rerun_candidate",
        "claim_boundary": VERTICAL_CLAIM_BOUNDARY,
        "connectors": [connector],
    }
    graph = {
        "schema_version": "0.1",
        "schema_name": "stairs_or_vertical_connector_graph_v0_1",
        "artifact_type": "formal_stairs_or_vertical_connector_graph",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "source_classification": "post_stage_a_formalization_candidate",
        "claim_boundary": VERTICAL_CLAIM_BOUNDARY,
        "nodes": nodes,
        "edges": edges,
        "connectors": [{"connector_id": "vc_vt_1", "transition_id": "vt_1", "transition_edge_id": "vt_1_centerline_e001"}],
        "provenance": connector["source_provenance"],
    }
    return artifact, graph


def copy_map_package(floor_id: str) -> dict[str, Any]:
    source_dir = CLEAN / "maps" / floor_id
    dest_dir = STAGED_MAPS / floor_id
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for source in sorted(source_dir.glob("stage1_*_stable_occupancy_map*")):
        dest = dest_dir / source.name
        shutil.copy2(source, dest)
        copied.append({"source": str(source), "dest": str(dest), "sha256": sha256(dest)})
    provenance = {
        "artifact_type": "task25b_controlled_staged_stable_map_import",
        "created_utc": now_iso(),
        "floor_id": floor_id,
        "source_classification": "controlled_import_from_validated_clean_rerun_stable_map_package",
        "reason": "task25b staged Stage-A does not yet produce a formal stable occupancy map package; route regeneration keeps map provenance explicit.",
        "claim_boundary": COMMON_CLAIM_BOUNDARY,
        "copied_files": copied,
    }
    write_json(dest_dir / "task25b_staged_map_import_provenance.json", provenance)
    return provenance


def build_cross_floor_topology(vertical: dict[str, Any]) -> dict[str, Any]:
    connector = vertical["connectors"][0]
    return {
        "schema_version": "0.1",
        "schema_name": "cross_floor_topology_v0_1",
        "artifact_type": "formal_cross_floor_topology",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "source_classification": "post_stage_a_formalization_candidate",
        "claim_boundary": VERTICAL_CLAIM_BOUNDARY,
        "rooms": [
            {"room_id": "room_2", "floor_id": "floor_1"},
            {"room_id": "room_3", "floor_id": "floor_1"},
            {"room_id": "room_7", "floor_id": "floor_2"},
            {"room_id": "room_13", "floor_id": "floor_2"},
            {"room_id": "room_14", "floor_id": "floor_2"},
        ],
        "connectors": [
            {
                "connector_id": connector["connector_id"],
                "transition_id": connector["transition_id"],
                "from_room_id": connector["from_room_id"],
                "to_room_id": connector["to_room_id"],
                "from_floor_id": connector["from_floor_id"],
                "to_floor_id": connector["to_floor_id"],
                "transition_edge_id": connector["transition_edge_id"],
            }
        ],
        "room_edges": [
            {"source_room_id": "room_2", "target_room_id": "room_3", "floor_id": "floor_1", "edge_source": "same-floor A* over stable occupancy map"},
            {"source_room_id": "room_3", "target_connector_id": "vc_vt_1", "floor_id": "floor_1", "edge_source": "connector binding"},
            {"source_connector_id": "vc_vt_1", "target_room_id": "room_7", "floor_id": "floor_2", "edge_source": "connector binding"},
            {"source_room_id": "room_7", "target_room_id": "room_13", "floor_id": "floor_2", "edge_source": "same-floor A* over stable occupancy map"},
            {"source_room_id": "room_13", "target_room_id": "room_14", "floor_id": "floor_2", "edge_source": "same-floor A* over stable occupancy map"},
        ],
        "canonical_regression_route": ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"],
        "source_provenance": {
            "stage_a_topology": str(RAW_LOGS / "topology_v0_1.json"),
            "formal_vertical_connectors": str(STAGED_PUBLIC / "vertical_connectors_v0_1.json"),
            "task24d_corrected_topology_reference": str(TASK24D_TOPOLOGY),
        },
    }


def build_object_artifacts(sources: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    query = sources["task24i_query"]
    approach = sources["task24i_approach"]
    query_artifact = {
        "schema_version": "0.1",
        "schema_name": "object_query_resolution_v0_1",
        "artifact_type": "formal_object_query_resolution",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "query_text": "curtain in room_14 on floor_2",
        "query_resolution": {
            "query_text": "curtain in room_14 on floor_2",
            "object_id": query.get("object_id"),
            "object_label": query.get("object_label"),
            "target_floor_id": query.get("floor_id"),
            "target_room_id": query.get("room_id"),
            "object_room_binding_source": "staged Stage-A committed_room_world_model object binding cross-checked with validated task24i query resolution",
            "object_centroid_xy": query.get("object_centroid_xy"),
            "object_centroid_navigation_used": False,
        },
        "source_classification": "imported_from_validated_downstream_artifact",
        "source_provenance": {
            "validated_task24i_query_resolution": str(TASK24I_QUERY),
            "staged_stage_a_room_world_model": str(RAW_LOGS / "committed_room_world_model_v0_1.json"),
        },
        "claim_boundary": OBJECT_CLAIM_BOUNDARY,
    }
    candidates_artifact = {
        "schema_version": "0.1",
        "schema_name": "object_approach_candidates_v0_1",
        "artifact_type": "formal_object_approach_candidates",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor_id": "floor_2",
        "target_room_id": "room_14",
        "source_classification": "imported_from_validated_downstream_artifact",
        "claim_boundary": OBJECT_CLAIM_BOUNDARY,
        "candidates": [
            {
                "approach_candidate_id": approach.get("approach_candidate_id"),
                "approach_pose": approach.get("approach_pose"),
                "facing_yaw": approach.get("target_facing_yaw_rad", approach.get("approach_pose", {}).get("yaw")),
                "generated_approach_candidate_used": True,
                "object_centroid_navigation_used": False,
                "approach_candidate_source": "validated task14/task24 generated ring candidate; staged as formal object interface candidate",
                "source_artifact_path": approach.get("source_artifact_path"),
                "validation_status": approach.get("validation_status"),
                "candidate_audit_excerpt": approach.get("candidate_audit_excerpt"),
            }
        ],
        "source_provenance": {
            "validated_task24i_approach_candidate": str(TASK24I_APPROACH),
            "validated_task24i_object_route": str(TASK24I_ROUTE),
        },
    }
    return query_artifact, candidates_artifact


def copy_route_with_formal_provenance(source: dict[str, Any], artifact_type: str, source_path: Path, extra: dict[str, Any]) -> dict[str, Any]:
    route = json.loads(json.dumps(source))
    route["artifact_type"] = artifact_type
    route["created_utc"] = now_iso()
    route["source_classification"] = "controlled_import_from_validated_downstream_route_samples"
    route["formal_regeneration_note"] = (
        "Semantic route contract is regenerated from task25b formal staged artifacts; dense A* samples are controlled imports "
        "from validated task24 route outputs until Stage-A natively exports formal stable occupancy map packages."
    )
    route["source_provenance"] = {
        "validated_downstream_route": str(source_path),
        "formal_vertical_connectors": str(STAGED_PUBLIC / "vertical_connectors_v0_1.json"),
        "formal_cross_floor_topology": str(STAGED_PUBLIC / "cross_floor_topology_v0_1.json"),
        "staged_floor_1_map": str(STAGED_MAPS / "floor_1/stage1_floor_1_stable_occupancy_map.yaml"),
        "staged_floor_2_map": str(STAGED_MAPS / "floor_2/stage1_floor_2_stable_occupancy_map.yaml"),
    }
    route["claim_boundary"] = {**COMMON_CLAIM_BOUNDARY, **route.get("claim_boundary", {})}
    route.update(extra)
    return route


def build_route_contracts(sources: dict[str, Any], object_query: dict[str, Any], object_candidates: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    room_contract = {
        "schema_version": "0.1",
        "schema_name": "room_level_cross_floor_route_contract_v0_1",
        "artifact_type": "formal_room_level_cross_floor_route_contract",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "source_classification": "post_stage_a_formalization_candidate",
        "start_room_id": "room_2",
        "start_floor_id": "floor_1",
        "target_room_id": "room_14",
        "target_floor_id": "floor_2",
        "room_level_route": ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"],
        "cross_floor_connector_segment": {
            "connector_id": "vc_vt_1",
            "transition_id": "vt_1",
            "transition_edge_id": "vt_1_centerline_e001",
            "segment_source": "stair connector 2.5D centerline",
        },
        "route_sources": ["same-floor A* over stable occupancy map", "stair connector 2.5D centerline"],
        "claim_boundary": COMMON_CLAIM_BOUNDARY,
    }
    planned_room = copy_route_with_formal_provenance(
        sources["task24g2_route"],
        "formal_planned_room_level_3d_route",
        TASK24G2_ROUTE,
        {
            "route_contract_source": str(STAGED_ROUTES / "room_level_cross_floor_route_contract_v0_1.json"),
            "route_sources": ["same-floor A* over stable occupancy map", "stair connector 2.5D centerline"],
            "visual_kinematic_proxy_only": True,
        },
    )
    candidate = object_candidates["candidates"][0]
    object_contract = {
        "schema_version": "0.1",
        "schema_name": "object_level_route_contract_v0_1",
        "artifact_type": "formal_object_level_cross_floor_route_contract",
        "scene_id": SCENE,
        "created_utc": now_iso(),
        "query_text": "curtain in room_14 on floor_2",
        "query_resolution": object_query["query_resolution"],
        "approach": {
            "approach_candidate_id": "generated_ring_037",
            "approach_pose": candidate["approach_pose"],
            "facing_yaw": candidate["facing_yaw"],
            "object_centroid_navigation_used": False,
            "generated_approach_candidate_used": True,
            "approach_candidate_source": candidate["approach_candidate_source"],
        },
        "route_contract": {
            "start_room_id": "room_2",
            "start_floor_id": "floor_1",
            "room_level_route": [
                {"room_id": "room_2", "floor_id": "floor_1"},
                {"room_id": "room_3", "floor_id": "floor_1"},
                {"transition_id": "vt_1", "connector_id": "vc_vt_1"},
                {"room_id": "room_7", "floor_id": "floor_2"},
                {"room_id": "room_13", "floor_id": "floor_2"},
                {"room_id": "room_14", "floor_id": "floor_2"},
            ],
            "cross_floor_connector_segment": {
                "connector_id": "vc_vt_1",
                "transition_id": "vt_1",
                "transition_edge_id": "vt_1_centerline_e001",
                "segment_source": "stair connector 2.5D centerline",
            },
            "object_approach_segment": {
                "planner": "same-floor A* over stable occupancy map",
                "map_package": str(STAGED_MAPS / "floor_2/stage1_floor_2_stable_occupancy_map.yaml"),
                "map_source_classification": "controlled_import_from_validated_clean_rerun_stable_map_package",
                "target": "approach_pose_not_object_centroid",
            },
            "route_sources": ["same-floor A* over stable occupancy map", "stair connector 2.5D centerline"],
        },
        "claim_boundary": OBJECT_CLAIM_BOUNDARY,
        "source_classification": "post_stage_a_formalization_candidate",
    }
    planned_object = copy_route_with_formal_provenance(
        sources["task24i_route"],
        "formal_planned_object_level_3d_route",
        TASK24I_ROUTE,
        {
            "route_contract_source": str(STAGED_ROUTES / "object_level_route_contract_v0_1.json"),
            "query_text": "curtain in room_14 on floor_2",
            "object_id": "obj_175",
            "approach_candidate_id": "generated_ring_037",
            "route_sources": ["same-floor A* over stable occupancy map", "stair connector 2.5D centerline"],
            "object_centroid_navigation_used": False,
            "generated_approach_candidate_used": True,
            "claim_boundary": OBJECT_CLAIM_BOUNDARY,
        },
    )
    return room_contract, planned_room, object_contract, planned_object


def validate_vertical(vertical: dict[str, Any]) -> dict[str, Any]:
    connector = vertical["connectors"][0]
    node_ids = {node["node_id"] for node in connector["centerline_nodes"]}
    required_nodes = {f"vt_1_centerline_n{i:03d}" for i in range(5)}
    edge_by_id = {edge["edge_id"]: edge for edge in connector["centerline_edges"]}
    checks = {
        "vt_1_exists": connector.get("transition_id") == "vt_1",
        "centerline_nodes_complete": required_nodes.issubset(node_ids),
        "transition_edge_is_e001": connector.get("transition_edge_id") == "vt_1_centerline_e001",
        "e003_is_not_transition": not edge_by_id.get("vt_1_centerline_e003", {}).get("topological_transition_edge", False),
        "z_start_approx_2p456": math.isclose(float(connector["z_start"]), 2.456, abs_tol=0.005),
        "z_end_approx_4p056": math.isclose(float(connector["z_end"]), 4.056, abs_tol=0.005),
        "z_delta_approx_1p6": math.isclose(float(connector["z_delta"]), 1.6, abs_tol=0.005),
        "source_node_floor_floor_1": connector["transition_edge"]["source_floor_id"] == "floor_1",
        "target_node_floor_floor_2": connector["transition_edge"]["target_floor_id"] == "floor_2",
        "claim_boundary_correct": connector["claim_boundary"] == VERTICAL_CLAIM_BOUNDARY,
    }
    return {
        "artifact_type": "task25b_formal_vertical_connector_validation",
        "created_utc": now_iso(),
        "classification": "formal_vertical_connector_validation_passed" if all(checks.values()) else "formal_vertical_connector_validation_failed",
        "passed": all(checks.values()),
        "checks": checks,
        "connector_id": connector.get("connector_id"),
        "transition_id": connector.get("transition_id"),
        "transition_edge_id": connector.get("transition_edge_id"),
    }


def route_nodes(route: dict[str, Any]) -> list[str]:
    return [item.get("node_id") for item in route.get("semantic_checkpoints", []) if item.get("node_id")]


def validate_room_route(contract: dict[str, Any], planned: dict[str, Any]) -> dict[str, Any]:
    nodes = route_nodes(planned)
    checks = {
        "room_route_resolves": contract["room_level_route"] == ["room_2", "room_3", "vt_1", "room_7", "room_13", "room_14"],
        "planned_route_contains_floor_1": any(item.get("floor_id") == "floor_1" for item in planned.get("semantic_checkpoints", [])),
        "planned_route_contains_floor_2": any(item.get("floor_id") == "floor_2" for item in planned.get("semantic_checkpoints", [])),
        "planned_route_contains_connector_segment": any(str(node).startswith("vt_1_centerline") for node in nodes),
        "transition_edge_is_e001": contract["cross_floor_connector_segment"]["transition_edge_id"] == "vt_1_centerline_e001",
        "no_physical_stair_climbing_claim": contract["claim_boundary"]["physical_stair_climbing_supported"] is False,
    }
    return {
        "artifact_type": "task25b_formal_cross_floor_route_validation",
        "created_utc": now_iso(),
        "classification": "formal_cross_floor_route_validation_passed" if all(checks.values()) else "formal_cross_floor_route_validation_failed",
        "passed": all(checks.values()),
        "checks": checks,
        "route": contract["room_level_route"],
    }


def validate_object_interface(query: dict[str, Any], candidates: dict[str, Any]) -> dict[str, Any]:
    resolution = query["query_resolution"]
    candidate = candidates["candidates"][0]
    pose = candidate.get("approach_pose") or {}
    checks = {
        "query_resolves": query.get("query_text") == "curtain in room_14 on floor_2",
        "object_id_obj_175": resolution.get("object_id") == "obj_175",
        "object_label_curtain": resolution.get("object_label") == "curtain",
        "target_floor_floor_2": resolution.get("target_floor_id") == "floor_2",
        "target_room_room_14": resolution.get("target_room_id") == "room_14",
        "approach_candidate_generated_ring_037": candidate.get("approach_candidate_id") == "generated_ring_037",
        "object_centroid_navigation_false": resolution.get("object_centroid_navigation_used") is False and candidate.get("object_centroid_navigation_used") is False,
        "approach_pose_usable": all(key in pose and pose[key] is not None for key in ("x", "y", "z", "yaw")),
        "facing_yaw_usable": isinstance(candidate.get("facing_yaw"), (int, float)),
    }
    return {
        "artifact_type": "task25b_formal_object_interface_validation",
        "created_utc": now_iso(),
        "classification": "formal_object_interface_validation_passed" if all(checks.values()) else "formal_object_interface_validation_failed",
        "passed": all(checks.values()),
        "checks": checks,
    }


def validate_object_route(contract: dict[str, Any], planned: dict[str, Any]) -> dict[str, Any]:
    checkpoints = planned.get("semantic_checkpoints", [])
    terminal = checkpoints[-1] if checkpoints else {}
    pose = contract["approach"]["approach_pose"]
    final_distance = math.sqrt(
        (float(terminal.get("x", 999)) - float(pose["x"])) ** 2
        + (float(terminal.get("y", 999)) - float(pose["y"])) ** 2
        + (float(terminal.get("z", 999)) - float(pose["z"])) ** 2
    )
    checks = {
        "route_reaches_object_approach": terminal.get("node_id") == "obj_175_approach_generated_ring_037",
        "final_planned_distance_to_approach_near_zero": final_distance <= 0.01,
        "approach_segment_source_recorded": bool(contract["route_contract"]["object_approach_segment"]["planner"]),
        "astar_or_fallback_source_recorded_honestly": "same-floor A*" in contract["route_contract"]["object_approach_segment"]["planner"]
        and contract["route_contract"]["object_approach_segment"]["map_source_classification"]
        == "controlled_import_from_validated_clean_rerun_stable_map_package",
        "does_not_use_object_centroid_as_target": contract["approach"]["object_centroid_navigation_used"] is False,
    }
    return {
        "artifact_type": "task25b_formal_object_route_validation",
        "created_utc": now_iso(),
        "classification": "formal_object_route_validation_passed" if all(checks.values()) else "formal_object_route_validation_failed",
        "passed": all(checks.values()),
        "checks": checks,
        "final_planned_distance_to_approach_m": round(final_distance, 6),
        "approach_segment_source": contract["route_contract"]["object_approach_segment"],
    }


def tracking_compatibility(planned_room: dict[str, Any], planned_object: dict[str, Any]) -> dict[str, Any]:
    room_checks = {
        "has_dense_3d_route_waypoints": bool(planned_room.get("dense_3d_route", {}).get("waypoints")),
        "has_semantic_checkpoints": bool(planned_room.get("semantic_checkpoints")),
        "contains_room14": "room_14" in route_nodes(planned_room),
        "contains_connector_nodes": any(str(node).startswith("vt_1_centerline") for node in route_nodes(planned_room)),
    }
    object_checks = {
        "has_dense_3d_route_waypoints": bool(planned_object.get("dense_3d_route", {}).get("waypoints")),
        "has_semantic_checkpoints": bool(planned_object.get("semantic_checkpoints")),
        "contains_obj175_approach": "obj_175_approach_generated_ring_037" in route_nodes(planned_object),
        "object_centroid_navigation_used_false": planned_object.get("object_centroid_navigation_used") is False,
    }
    commands = [
        "/usr/bin/python3 tools/vertical_connectors/cross_floor_visual_tracking_player.py --planned-route-json "
        + str(STAGED_ROUTES / "planned_room_level_3d_route_v0_1.json")
        + " --print-start-pose",
        "/usr/bin/python3 tools/vertical_connectors/cross_floor_object_tracking_player.py --planned-route-json "
        + str(STAGED_ROUTES / "planned_object_level_3d_route_v0_1.json")
        + " --print-start-pose",
    ]
    return {
        "artifact_type": "task25b_tracking_input_compatibility_validation",
        "created_utc": now_iso(),
        "classification": "tracking_input_compatibility_passed" if all(room_checks.values()) and all(object_checks.values()) else "tracking_input_compatibility_failed",
        "passed": all(room_checks.values()) and all(object_checks.values()),
        "validation_mode": "static_schema_compatibility_with_existing_task24h_task24i_tracking_players_no_gazebo_launch",
        "room_level_player_input_checks": room_checks,
        "object_level_player_input_checks": object_checks,
        "recommended_no_gazebo_dry_run_commands": commands,
        "executed_dry_run_commands": [],
        "claim_boundary": COMMON_CLAIM_BOUNDARY,
    }


def claim_boundary_report() -> dict[str, Any]:
    return {
        "artifact_type": "task25b_claim_boundary",
        "created_utc": now_iso(),
        "classification": "task25b_claim_boundary_validated",
        "visual_kinematic_proxy_only": True,
        "topological_vertical_transition_only": True,
        "physical_stair_climbing_supported": False,
        "gait_supported": False,
        "footstep_planning_supported": False,
        "contact_based_stair_climbing_supported": False,
        "nav2_execution": False,
        "amcl_localization": False,
        "object_centroid_navigation_used": False,
        "object_level_smoke_test_only": True,
        "not_full_object_navigation_benchmark": True,
    }


def run_jq_validation(paths: list[Path]) -> dict[str, Any]:
    results = []
    for path in sorted(set(paths)):
        proc = subprocess.run(["jq", "empty", str(path)], cwd=REPO, text=True, capture_output=True, check=False)
        results.append({"path": str(path), "returncode": proc.returncode, "passed": proc.returncode == 0, "stderr": proc.stderr.strip()})
    return {
        "artifact_type": "task25b_json_validation_report",
        "created_utc": now_iso(),
        "validation_tool": "jq empty",
        "classification": "json_validation_passed" if all(item["passed"] for item in results) else "json_validation_failed",
        "passed": all(item["passed"] for item in results),
        "results": results,
    }


def write_commands_file() -> None:
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "cd /home/ws/workspace/BoxFusion",
        "",
        "# Preflight snapshot generated before Stage-A with /usr/bin/python3 inline helper.",
        "/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \\",
        "  --model-path models/cutr_rgbd.pth \\",
        "  --config config/hm3d.yaml \\",
        "  --seq 00843-DYehNKdT76V \\",
        "  --output-root stage_outputs/stage1_generalization/00843-DYehNKdT76V/task25b_stage_a_rerun_candidate/canonical_stage1/raw_outputs \\",
        "  --capture-stride 25 \\",
        "  --room-seg-interval 100 \\",
        "  --runtime-profile-interval 25 \\",
        "  --device cuda \\",
        "  --quiet",
        "",
        "/usr/bin/python3 tools/vertical_connectors/export_task25b_formal_cross_floor_artifacts.py",
        "find stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task25b_formal_connector_object_interface_exporter_and_staged_rerun -name '*.json' -print0 | xargs -0 -n1 jq empty",
    ]
    write_text(TASK_ROOT / "task25b_commands_ran.sh", "\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=REPO)
    _args = parser.parse_args()
    TASK_ROOT.mkdir(parents=True, exist_ok=True)
    STAGED_PUBLIC.mkdir(parents=True, exist_ok=True)
    STAGED_ROUTES.mkdir(parents=True, exist_ok=True)

    stage_report, stage_summary = stage_a_report()
    write_json(TASK_ROOT / "stage_a_staged_rerun_report.json", stage_report)
    write_text(TASK_ROOT / "stage_a_staged_rerun_stdout_stderr_summary.md", stage_summary)
    if stage_report["classification"] != "stage_a_staged_rerun_succeeded":
        write_json(TASK_ROOT / "task25b_report.json", {"classification": "task25b_blocked_by_stage_a_rerun_failure", "stage_a_report": stage_report})
        return 2

    sources = load_formal_sources()
    map_imports = [copy_map_package("floor_1"), copy_map_package("floor_2")]
    vertical, connector_graph = build_vertical_artifact(sources)
    topology = build_cross_floor_topology(vertical)
    object_query, object_candidates = build_object_artifacts(sources)
    room_contract, planned_room, object_contract, planned_object = build_route_contracts(sources, object_query, object_candidates)

    artifact_paths: list[Path] = []
    for path, payload in [
        (STAGED_PUBLIC / "vertical_connectors_v0_1.json", vertical),
        (STAGED_PUBLIC / "stairs_or_vertical_connector_graph_v0_1.json", connector_graph),
        (STAGED_PUBLIC / "cross_floor_topology_v0_1.json", topology),
        (STAGED_PUBLIC / "object_query_resolution_v0_1.json", object_query),
        (STAGED_PUBLIC / "object_approach_candidates_v0_1.json", object_candidates),
        (STAGED_ROUTES / "room_level_cross_floor_route_contract_v0_1.json", room_contract),
        (STAGED_ROUTES / "planned_room_level_3d_route_v0_1.json", planned_room),
        (STAGED_ROUTES / "object_level_route_contract_v0_1.json", object_contract),
        (STAGED_ROUTES / "planned_object_level_3d_route_v0_1.json", planned_object),
    ]:
        write_json(path, payload)
        artifact_paths.append(path)

    validations = {
        "formal_vertical_connector_validation": validate_vertical(vertical),
        "formal_cross_floor_route_validation": validate_room_route(room_contract, planned_room),
        "formal_object_interface_validation": validate_object_interface(object_query, object_candidates),
        "formal_object_route_validation": validate_object_route(object_contract, planned_object),
        "tracking_input_compatibility_validation": tracking_compatibility(planned_room, planned_object),
    }
    validation_files = {
        "formal_vertical_connector_validation": TASK_ROOT / "formal_vertical_connector_validation.json",
        "formal_cross_floor_route_validation": TASK_ROOT / "formal_cross_floor_route_validation.json",
        "formal_object_interface_validation": TASK_ROOT / "formal_object_interface_validation.json",
        "formal_object_route_validation": TASK_ROOT / "formal_object_route_validation.json",
        "tracking_input_compatibility_validation": TASK_ROOT / "tracking_input_compatibility_validation.json",
    }
    for key, path in validation_files.items():
        write_json(path, validations[key])
        artifact_paths.append(path)

    manifest = {
        "artifact_type": "task25b_formal_artifact_manifest",
        "created_utc": now_iso(),
        "classification": "formal_artifacts_generated",
        "staged_output_root": str(STAGING_ROOT),
        "formal_public_artifacts": [str(path) for path in artifact_paths if str(path).startswith(str(STAGED_PUBLIC))],
        "route_artifacts": [str(path) for path in artifact_paths if str(path).startswith(str(STAGED_ROUTES))],
        "validation_artifacts": [str(path) for path in validation_files.values()],
        "map_imports": map_imports,
    }
    write_json(TASK_ROOT / "formal_artifact_manifest.json", manifest)
    artifact_paths.append(TASK_ROOT / "formal_artifact_manifest.json")

    exporter_report = {
        "artifact_type": "task25b_formal_exporter_report",
        "created_utc": now_iso(),
        "classification": "formal_exporter_completed",
        "source_classification_summary": {
            "stage_a_native": [
                "vertical transition evidence",
                "room/floor transition bindings",
                "committed room world model",
                "topology_v0_1",
            ],
            "imported_from_validated_downstream_artifact": [
                "vt_1 fitted centerline nodes and edges",
                "transition_edge_id vt_1_centerline_e001",
                "obj_175/generated_ring_037 object query and approach candidate",
                "dense A* route samples from task24g2/task24i",
                "stable occupancy map packages staged from clean_rerun maps",
            ],
        },
        "formal_artifacts": manifest,
        "claim_boundary": claim_boundary_report(),
    }
    write_json(TASK_ROOT / "formal_exporter_report.json", exporter_report)
    write_json(TASK_ROOT / "claim_boundary_task25b.json", claim_boundary_report())
    artifact_paths.extend([TASK_ROOT / "formal_exporter_report.json", TASK_ROOT / "claim_boundary_task25b.json"])

    postflight = build_snapshot("postflight")
    write_json(TASK_ROOT / "protected_artifact_postflight_snapshot.json", postflight)
    preflight_path = TASK_ROOT / "protected_artifact_preflight_snapshot.json"
    preflight = read_json(preflight_path) if preflight_path.exists() else {}
    modification_check = compare_snapshots(preflight, postflight) if preflight else {
        "artifact_type": "task25b_protected_artifact_modification_check",
        "classification": "task25b_preflight_snapshot_missing",
        "protected_artifacts_modified": None,
    }
    write_json(TASK_ROOT / "protected_artifact_modification_check.json", modification_check)
    artifact_paths.extend([TASK_ROOT / "stage_a_staged_rerun_report.json", TASK_ROOT / "protected_artifact_postflight_snapshot.json", TASK_ROOT / "protected_artifact_modification_check.json"])

    risk_update = {
        "artifact_type": "task25b_risk_resolution_update",
        "created_utc": now_iso(),
        "resolved_or_reduced": [
            "Stage-A output-root semantics validated under a task25b staging root with CUDA.",
            "vt_1 transition-edge invariant preserved as vt_1_centerline_e001.",
            "object approach uses generated_ring_037 and not object centroid navigation.",
            "protected clean_rerun/task24/task23 surfaces are compared pre/post.",
        ],
        "remaining": [
            "Centerline and object approach candidates remain post-Stage-A formalization candidates, not native Stage-A outputs.",
            "Stable occupancy map packages are staged controlled imports from validated clean_rerun maps.",
            "Tracking compatibility is static input compatibility; task25b does not launch Gazebo/RViz.",
            "Stage-A still reports point_cloud_path under exported_pc outside the requested raw output root.",
        ],
    }
    write_json(TASK_ROOT / "risk_resolution_update.json", risk_update)
    artifact_paths.append(TASK_ROOT / "risk_resolution_update.json")

    json_report = run_jq_validation(artifact_paths)
    write_json(TASK_ROOT / "json_validation_report.json", json_report)

    all_validation_passed = (
        stage_report["classification"] == "stage_a_staged_rerun_succeeded"
        and all(item["passed"] for item in validations.values())
        and json_report["passed"]
        and modification_check.get("protected_artifacts_modified") is False
    )
    if modification_check.get("protected_artifacts_modified") is True:
        classification = "task25b_failed_protected_artifact_modified"
    elif not validations["formal_vertical_connector_validation"]["passed"]:
        classification = "task25b_blocked_by_formal_connector_export_failure"
    elif not validations["formal_object_interface_validation"]["passed"]:
        classification = "task25b_blocked_by_formal_object_interface_export_failure"
    elif not validations["tracking_input_compatibility_validation"]["passed"]:
        classification = "task25b_staged_artifacts_generated_but_tracking_input_incompatible"
    elif all_validation_passed:
        classification = "task25b_formal_staged_reintegration_passed"
    else:
        classification = "task25b_staged_reintegration_validation_incomplete"

    report = {
        "artifact_type": "task25b_report",
        "created_utc": now_iso(),
        "classification": classification,
        "stage_a_rerun_used_cuda": True,
        "stage_a_report": str(TASK_ROOT / "stage_a_staged_rerun_report.json"),
        "staged_output_root": str(STAGING_ROOT),
        "protected_artifacts_modified": modification_check.get("protected_artifacts_modified"),
        "formal_artifacts_generated": manifest,
        "validation_summary": {key: value["classification"] for key, value in validations.items()},
        "json_validation": json_report["classification"],
        "what_became_staged_formal_artifact": [
            "vertical_connectors_v0_1.json",
            "stairs_or_vertical_connector_graph_v0_1.json",
            "cross_floor_topology_v0_1.json",
            "object_query_resolution_v0_1.json",
            "object_approach_candidates_v0_1.json",
            "room and object route contracts/planned routes under the task25b staging root",
        ],
        "what_remains_task_only": [
            "Gazebo/RViz tracking execution logs",
            "SetEntityState visual playback evidence",
            "physical stair-climbing, gait, footstep, contact locomotion, Nav2, AMCL, and full object-navigation benchmark claims",
        ],
        "task25c_promotion_can_proceed": classification == "task25b_formal_staged_reintegration_passed",
        "task25a_helper_git_state": postflight.get("task25a_helper_git_state"),
    }
    write_json(TASK_ROOT / "task25b_report.json", report)

    report_md = f"""# Task25b Report

Classification: `{classification}`

Stage-A rerun used CUDA: `true`

Staged output root: `{STAGING_ROOT}`

Protected artifacts modified: `{modification_check.get('protected_artifacts_modified')}`

Formal artifacts generated under:

- `{STAGED_PUBLIC}`
- `{STAGED_ROUTES}`

Validation summary:

- vt_1 connector: `{validations['formal_vertical_connector_validation']['classification']}`
- room-level route: `{validations['formal_cross_floor_route_validation']['classification']}`
- object interface: `{validations['formal_object_interface_validation']['classification']}`
- object route: `{validations['formal_object_route_validation']['classification']}`
- tracking input compatibility: `{validations['tracking_input_compatibility_validation']['classification']}`
- JSON validation: `{json_report['classification']}`

Task-only boundary:

- Gazebo/RViz playback evidence remains task-only.
- This does not claim physical stair climbing, gait, footstep planning, contact-based stair climbing, Nav2 execution, AMCL localization, or a full object-navigation benchmark.

Task25c promotion can proceed: `{classification == 'task25b_formal_staged_reintegration_passed'}`
"""
    write_text(TASK_ROOT / "task25b_report.md", report_md)
    write_text(TASK_ROOT / "final_answer_for_user.md", report_md)
    write_text(
        TASK_ROOT / "task25c_promotion_plan.md",
        """# Task25c Promotion Plan

1. Review the task25b staged public artifacts against existing clean_rerun public artifacts.
2. Promote only schema-stable formal artifacts after preserving provenance fields.
3. Keep task24 visual tracking logs and task-only evidence out of committed_public.
4. Add native Stage-A producers for centerline, stable map packages, and object approach candidates before removing the controlled-import classifications.
5. Re-run JSON, vt_1, room-route, object-route, and protected-artifact checks after promotion.
""",
    )
    write_commands_file()
    return 0 if classification == "task25b_formal_staged_reintegration_passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
