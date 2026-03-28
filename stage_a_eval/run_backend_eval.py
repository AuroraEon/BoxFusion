from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import (
    DEFAULT_EVAL_OUTPUT_ROOT,
    DEFAULT_LEGACY_SCENE_OUTPUT_ROOT,
    DEFAULT_SCENE_OUTPUT_ROOT,
    collect_scene_manifest,
    dump_json,
    find_scene_root,
    latency_summary_ms,
    load_jsonl,
)
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.vln_closed_loop import build_symbolic_plan


DEFAULT_REGISTRY_PATH = Path("stage_a_eval/scene_registry.json")
DEFAULT_TASKS_PATH = Path("stage_a_eval/backend_tasks_v0_1.jsonl")
RESULTS_JSON_NAME = "backend_eval_results.json"


def resolve_legacy_scene_output_root(*, allow_legacy_fallback: bool, legacy_scene_output_root: str) -> Path | None:
    if not allow_legacy_fallback:
        return None
    text = str(legacy_scene_output_root or "").strip()
    if not text:
        return None
    return Path(text)


def task_policy(task: Dict[str, Any]) -> str:
    return str(task.get("policy") or task.get("route_policy") or "balanced")


def task_source_room(task: Dict[str, Any]) -> Any:
    return task.get("source_room_id", task.get("start_room"))


def task_expected_outcome(task: Dict[str, Any]) -> str:
    if "expected_outcome" in task:
        return str(task.get("expected_outcome"))
    return "success" if bool(task.get("expected_success", True)) else "failure"


def safe_rate(numerator: int, denominator: int) -> Optional[float]:
    if denominator == 0:
        return None
    return round(float(numerator) / float(denominator), 4)


def format_rate(rate: Optional[float]) -> str:
    if rate is None:
        return "n/a"
    return f"{rate * 100.0:.1f}%"


def status_from_failure_reason(reason: Any) -> str:
    text = str(reason or "").strip().lower()
    if not text:
        return "failure"
    if "ambiguous" in text:
        return "ambiguous"
    if "unsupported" in text:
        return "unsupported"
    if "unresolved" in text:
        return "unresolved"
    if "invalid" in text:
        return "invalid"
    if "not_found" in text:
        return "not_found"
    return text


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def ordered_scene_lookup(registry: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        str(item.get("sequence_name")): dict(item)
        for item in registry.get("scenes", [])
        if item.get("sequence_name") is not None
    }


def load_registry(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_scene_context(sequence_name: str, scene_root: Path) -> Dict[str, Any]:
    topology_json = scene_root / "logs" / "topology_v0_1.json"
    query_api = RoomTopologyQueryAPI.from_json(topology_json)
    manifest = collect_scene_manifest(scene_root, sequence_name=sequence_name)
    return {
        "sequence_name": sequence_name,
        "scene_root": scene_root,
        "query_api": query_api,
        "manifest": manifest,
    }


def invoke_task(context: Dict[str, Any], task: Dict[str, Any]) -> Tuple[Dict[str, Any], float, str]:
    query_api = context["query_api"]
    task_type = str(task.get("task_type"))
    route_policy = task_policy(task)
    target_spec = dict(task.get("target_spec") or {})
    timing_bucket = "query_api"
    start_t = time.perf_counter()

    if task_type == "resolve_object":
        result = query_api.resolve_object_room(
            object_id=target_spec.get("object_id"),
            object_label=target_spec.get("object_label"),
        )
    elif task_type == "resolve_anchor":
        result = query_api.resolve_anchor_room(target_spec.get("anchor_id"))
    elif task_type == "resolve_room":
        result = query_api.resolve_room_target(target_spec.get("goal_room_id"))
    elif task_type == "query_room_route":
        result = query_api.query_route(
            start_room_id=task_source_room(task),
            goal_room_id=target_spec.get("goal_room_id"),
            route_policy=route_policy,
        )
    elif task_type == "query_object_route":
        result = query_api.query_route_to_object(
            start_room_id=task_source_room(task),
            object_id=target_spec.get("object_id"),
            object_label=target_spec.get("object_label"),
            route_policy=route_policy,
        )
    elif task_type == "query_anchor_route":
        result = query_api.query_route_to_anchor(
            start_room_id=task_source_room(task),
            anchor_id=target_spec.get("anchor_id"),
            route_policy=route_policy,
        )
    elif task_type == "execute_room_route":
        timing_bucket = "symbolic_plan"
        result = build_symbolic_plan(
            query_api,
            task_input={
                "start_room_id": task_source_room(task),
                "target": {"target_type": "room", "goal_room_id": target_spec.get("goal_room_id")},
            },
            route_policy=route_policy,
        )
    elif task_type == "execute_object_route":
        timing_bucket = "symbolic_plan"
        result = build_symbolic_plan(
            query_api,
            task_input={
                "start_room_id": task_source_room(task),
                "target": {
                    "target_type": "object",
                    "object_id": target_spec.get("object_id"),
                    "object_label": target_spec.get("object_label"),
                },
            },
            route_policy=route_policy,
        )
    elif task_type == "execute_anchor_route":
        timing_bucket = "symbolic_plan"
        result = build_symbolic_plan(
            query_api,
            task_input={
                "start_room_id": task_source_room(task),
                "target": {"target_type": "anchor", "anchor_id": target_spec.get("anchor_id")},
            },
            route_policy=route_policy,
        )
    else:
        raise ValueError(f"Unsupported task_type {task_type}")

    latency_ms = (time.perf_counter() - start_t) * 1000.0
    return result, latency_ms, timing_bucket


def normalize_task_result(task: Dict[str, Any], raw_result: Dict[str, Any], latency_ms: float, timing_bucket: str) -> Dict[str, Any]:
    task_type = str(task.get("task_type"))
    route_policy = task_policy(task)
    expected_outcome = task_expected_outcome(task)

    if task_type.startswith("resolve_"):
        target_resolution = dict(raw_result)
        route = {}
        actual_status = "success" if target_resolution.get("resolved") else status_from_failure_reason(target_resolution.get("failure_reason"))
    elif task_type.startswith("query_"):
        target_resolution = dict(raw_result.get("target_resolution") or {})
        route = dict(raw_result.get("route") or {})
        if target_resolution and not target_resolution.get("resolved"):
            actual_status = status_from_failure_reason(target_resolution.get("failure_reason"))
        elif route.get("found"):
            actual_status = "success"
        else:
            actual_status = status_from_failure_reason(route.get("failure_reason") or raw_result.get("failure_reason") or "route_not_found")
    else:
        target_resolution = dict(raw_result.get("target_resolution") or {})
        route = dict((raw_result.get("query_result") or {}).get("route") or {})
        if target_resolution and not target_resolution.get("resolved"):
            actual_status = status_from_failure_reason(target_resolution.get("failure_reason"))
        elif raw_result.get("plan_found"):
            actual_status = "success"
        else:
            actual_status = status_from_failure_reason(raw_result.get("planning_reason") or "route_not_found")

    expected_room = task.get("expected_target_room")
    expected_floor = task.get("expected_floor_id")
    actual_room = target_resolution.get("resolved_room_id")
    actual_floor = target_resolution.get("resolved_floor_id")
    route_found = None
    if task_type.startswith(("query_", "execute_")):
        route_found = bool(route.get("found"))

    return {
        "task_id": task.get("task_id"),
        "split": task.get("split"),
        "scene_id": task.get("scene_id"),
        "sequence_name": task.get("sequence_name"),
        "task_family": task.get("task_family"),
        "task_type": task_type,
        "hierarchy_level": task.get("hierarchy_level"),
        "scene_role": task.get("scene_role"),
        "hov_sg_overlap_split": task.get("hov_sg_overlap_split"),
        "policy": route_policy,
        "route_policy": route_policy,
        "source_room_id": task_source_room(task),
        "target_granularity": task.get("target_granularity"),
        "needs_route": task.get("needs_route"),
        "resolve_only": task.get("resolve_only"),
        "expected_floor_sensitive": task.get("expected_floor_sensitive"),
        "expected_room_sensitive": task.get("expected_room_sensitive"),
        "expected_outcome": expected_outcome,
        "expected_success": task.get("expected_success"),
        "actual_status": actual_status,
        "task_success": bool(actual_status == expected_outcome),
        "expected_target_room": expected_room,
        "actual_resolved_room": actual_room,
        "expected_floor_id": expected_floor,
        "actual_resolved_floor_id": actual_floor,
        "target_room_id": task.get("target_room_id"),
        "target_floor_id": task.get("target_floor_id"),
        "target_object_label": task.get("target_object_label"),
        "target_anchor_label": task.get("target_anchor_label"),
        "exact_room_hit": None if expected_room is None else bool(actual_room == expected_room),
        "exact_floor_hit": None if expected_floor is None else bool(actual_floor == expected_floor),
        "route_found": route_found,
        "route_hop_count": route.get("hop_count"),
        "route_total_cost": route.get("total_cost"),
        "latency_ms": round(float(latency_ms), 3),
        "timing_bucket": timing_bucket,
    }


def rate_for(rows: Sequence[Dict[str, Any]], field: str) -> Tuple[int, int, Optional[float]]:
    applicable = [row for row in rows if row.get(field) is not None]
    numerator = sum(1 for row in applicable if bool(row.get(field)))
    denominator = len(applicable)
    return numerator, denominator, safe_rate(numerator, denominator)


def scene_runtime_row(sequence_name: str, manifest: Dict[str, Any]) -> Dict[str, Any]:
    world = dict(manifest.get("world_model_summary") or {})
    runtime = dict(manifest.get("runtime_summary") or {})
    compactness = dict(manifest.get("compactness_summary") or {})
    return {
        "sequence_name": sequence_name,
        "scene_id": manifest.get("scene_id"),
        "status": manifest.get("status"),
        "floor_count": world.get("floor_count"),
        "room_count": world.get("room_count"),
        "object_count": world.get("object_count"),
        "anchor_count": world.get("anchor_count"),
        "node_count": world.get("node_count"),
        "edge_count": world.get("edge_count"),
        "vertical_transition_count": world.get("vertical_transition_count"),
        "backend_artifact_size_total_bytes": runtime.get("backend_artifact_size_total_bytes"),
        "optional_demo_artifact_size_total_bytes": runtime.get("optional_demo_artifact_size_total_bytes"),
        "artifact_size_total_bytes": runtime.get("artifact_size_total_bytes"),
        "processed_frames": runtime.get("processed_frames"),
        "duration_sec": runtime.get("duration_sec"),
        "average_fps": runtime.get("average_fps"),
        "objects_per_room": compactness.get("objects_per_room"),
        "anchors_per_room": compactness.get("anchors_per_room"),
        "bytes_per_room": compactness.get("bytes_per_room"),
    }


def summarize_results(
    *,
    task_results: Sequence[Dict[str, Any]],
    scene_manifests: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    success_count = sum(1 for row in task_results if row.get("task_success"))
    task_success_rate = safe_rate(success_count, len(task_results))

    resolve_rows = [row for row in task_results if str(row.get("task_type")).startswith("resolve_")]
    resolve_success_count = sum(1 for row in resolve_rows if row.get("actual_status") == "success")
    resolve_success_rate = safe_rate(resolve_success_count, len(resolve_rows))

    room_num, room_den, room_rate = rate_for(task_results, "exact_room_hit")
    floor_num, floor_den, floor_rate = rate_for(task_results, "exact_floor_hit")
    route_num, route_den, route_rate = rate_for(task_results, "route_found")

    query_latencies = [float(row["latency_ms"]) for row in task_results if row.get("timing_bucket") == "query_api"]
    symbolic_latencies = [float(row["latency_ms"]) for row in task_results if row.get("timing_bucket") == "symbolic_plan"]

    policy_breakdown: List[Dict[str, Any]] = []
    for route_policy in sorted({str(row.get("route_policy")) for row in task_results if row.get("route_policy")}):
        rows = [row for row in task_results if str(row.get("route_policy")) == route_policy]
        route_hits, route_total, route_policy_rate = rate_for(rows, "route_found")
        policy_breakdown.append(
            {
                "route_policy": route_policy,
                "task_count": len(rows),
                "task_success_rate": safe_rate(sum(1 for row in rows if row.get("task_success")), len(rows)),
                "route_found_rate": route_policy_rate,
                "latency_mean_ms": round(mean([float(row["latency_ms"]) for row in rows]), 3) if rows else None,
                "route_found_raw": f"{route_hits}/{route_total}" if route_total else "n/a",
            }
        )

    task_type_breakdown: List[Dict[str, Any]] = []
    for task_type in sorted({str(row.get("task_type")) for row in task_results}):
        rows = [row for row in task_results if str(row.get("task_type")) == task_type]
        route_hits, route_total, route_type_rate = rate_for(rows, "route_found")
        task_type_breakdown.append(
            {
                "task_type": task_type,
                "task_count": len(rows),
                "task_success_rate": safe_rate(sum(1 for row in rows if row.get("task_success")), len(rows)),
                "route_found_rate": route_type_rate,
                "latency_mean_ms": round(mean([float(row["latency_ms"]) for row in rows]), 3) if rows else None,
                "route_found_raw": f"{route_hits}/{route_total}" if route_total else "n/a",
            }
        )

    scene_rows = [
        scene_runtime_row(sequence_name, manifest)
        for sequence_name, manifest in sorted(scene_manifests.items())
    ]

    aggregate_backend_artifact_bytes = sum(
        int((manifest.get("runtime_summary") or {}).get("backend_artifact_size_total_bytes") or 0)
        for manifest in scene_manifests.values()
    )
    aggregate_optional_demo_artifact_bytes = sum(
        int((manifest.get("runtime_summary") or {}).get("optional_demo_artifact_size_total_bytes") or 0)
        for manifest in scene_manifests.values()
    )

    return {
        "task_count": len(task_results),
        "scene_count": len(scene_manifests),
        "task_success_rate": task_success_rate,
        "resolve_success_rate": resolve_success_rate,
        "exact_room_hit_rate": room_rate,
        "exact_floor_hit_rate": floor_rate,
        "route_found_rate": route_rate,
        "task_success_raw": f"{success_count}/{len(task_results)}" if task_results else "0/0",
        "resolve_success_raw": f"{resolve_success_count}/{len(resolve_rows)}" if resolve_rows else "0/0",
        "exact_room_hit_raw": f"{room_num}/{room_den}" if room_den else "n/a",
        "exact_floor_hit_raw": f"{floor_num}/{floor_den}" if floor_den else "n/a",
        "route_found_raw": f"{route_num}/{route_den}" if route_den else "n/a",
        "query_api_latency_ms": latency_summary_ms(query_latencies),
        "symbolic_plan_latency_ms": latency_summary_ms(symbolic_latencies),
        "aggregate_backend_artifact_size_bytes": aggregate_backend_artifact_bytes,
        "aggregate_optional_demo_artifact_size_bytes": aggregate_optional_demo_artifact_bytes,
        "policy_breakdown": policy_breakdown,
        "task_type_breakdown": task_type_breakdown,
        "scene_runtime_summary": scene_rows,
    }


def render_outputs(report_root: Path, payload: Dict[str, Any]) -> Dict[str, str]:
    report_root.mkdir(parents=True, exist_ok=True)

    aggregate = dict(payload.get("aggregate_summary") or {})
    task_results = list(payload.get("task_results") or [])
    scene_rows = list(aggregate.get("scene_runtime_summary") or [])
    policy_rows = list(aggregate.get("policy_breakdown") or [])
    task_type_rows = list(aggregate.get("task_type_breakdown") or [])

    results_json = report_root / RESULTS_JSON_NAME
    aggregate_json = report_root / "aggregate_summary.json"
    aggregate_csv = report_root / "aggregate_summary.csv"
    aggregate_md = report_root / "aggregate_summary.md"
    task_results_csv = report_root / "task_results.csv"
    scene_runtime_csv = report_root / "scene_runtime_summary.csv"
    policy_csv = report_root / "policy_breakdown.csv"
    task_type_csv = report_root / "task_type_breakdown.csv"

    dump_json(results_json, payload)
    dump_json(aggregate_json, aggregate)

    aggregate_row = {
        "task_count": aggregate.get("task_count"),
        "scene_count": aggregate.get("scene_count"),
        "task_success_rate": aggregate.get("task_success_rate"),
        "resolve_success_rate": aggregate.get("resolve_success_rate"),
        "exact_room_hit_rate": aggregate.get("exact_room_hit_rate"),
        "exact_floor_hit_rate": aggregate.get("exact_floor_hit_rate"),
        "route_found_rate": aggregate.get("route_found_rate"),
        "task_success_raw": aggregate.get("task_success_raw"),
        "resolve_success_raw": aggregate.get("resolve_success_raw"),
        "exact_room_hit_raw": aggregate.get("exact_room_hit_raw"),
        "exact_floor_hit_raw": aggregate.get("exact_floor_hit_raw"),
        "route_found_raw": aggregate.get("route_found_raw"),
        "query_api_mean_ms": dict(aggregate.get("query_api_latency_ms") or {}).get("mean_ms"),
        "query_api_p50_ms": dict(aggregate.get("query_api_latency_ms") or {}).get("p50_ms"),
        "query_api_p90_ms": dict(aggregate.get("query_api_latency_ms") or {}).get("p90_ms"),
        "symbolic_plan_mean_ms": dict(aggregate.get("symbolic_plan_latency_ms") or {}).get("mean_ms"),
        "symbolic_plan_p50_ms": dict(aggregate.get("symbolic_plan_latency_ms") or {}).get("p50_ms"),
        "symbolic_plan_p90_ms": dict(aggregate.get("symbolic_plan_latency_ms") or {}).get("p90_ms"),
        "aggregate_backend_artifact_size_bytes": aggregate.get("aggregate_backend_artifact_size_bytes"),
        "aggregate_optional_demo_artifact_size_bytes": aggregate.get("aggregate_optional_demo_artifact_size_bytes"),
    }
    write_csv(aggregate_csv, [aggregate_row], list(aggregate_row.keys()))

    if task_results:
        write_csv(
            task_results_csv,
            task_results,
            [
                "task_id",
                "split",
                "scene_id",
                "sequence_name",
                "task_family",
                "task_type",
                "hierarchy_level",
                "scene_role",
                "hov_sg_overlap_split",
                "policy",
                "route_policy",
                "source_room_id",
                "target_granularity",
                "needs_route",
                "resolve_only",
                "expected_floor_sensitive",
                "expected_room_sensitive",
                "expected_outcome",
                "expected_success",
                "actual_status",
                "task_success",
                "expected_target_room",
                "actual_resolved_room",
                "expected_floor_id",
                "actual_resolved_floor_id",
                "target_room_id",
                "target_floor_id",
                "target_object_label",
                "target_anchor_label",
                "exact_room_hit",
                "exact_floor_hit",
                "route_found",
                "route_hop_count",
                "route_total_cost",
                "latency_ms",
                "timing_bucket",
            ],
        )
    if scene_rows:
        write_csv(scene_runtime_csv, scene_rows, list(scene_rows[0].keys()))
    if policy_rows:
        write_csv(policy_csv, policy_rows, list(policy_rows[0].keys()))
    if task_type_rows:
        write_csv(task_type_csv, task_type_rows, list(task_type_rows[0].keys()))

    lines = [
        "# Backend Evaluation Summary",
        "",
        "## Scope",
        "",
        "- Evaluates Tier 1 core backend artifacts only: manifest, summary, topology, topology query report, vertical-transition evidence, and floor diagnostics.",
        "- Query tasks reuse the existing room-centric backend layers against seeded truth-layer overlap probes.",
        "- Query tasks reuse the current Query API; execute tasks reuse symbolic plan generation from the existing closed-loop executor stack.",
        "- Tier 2 demo outputs such as PNG, MP4, markdown, and appendix-style diagnostics are optional and excluded from backend compactness totals.",
        "- Limitation: the seeded tasks are backend-overlap probes generated from current scene exports, not external human-annotated navigation benchmarks.",
        "",
        "## Aggregate",
        "",
        f"- Task success rate: {format_rate(aggregate.get('task_success_rate'))} ({aggregate.get('task_success_raw')})",
        f"- Resolve success rate: {format_rate(aggregate.get('resolve_success_rate'))} ({aggregate.get('resolve_success_raw')})",
        f"- Exact room hit rate: {format_rate(aggregate.get('exact_room_hit_rate'))} ({aggregate.get('exact_room_hit_raw')})",
        f"- Exact floor hit rate: {format_rate(aggregate.get('exact_floor_hit_rate'))} ({aggregate.get('exact_floor_hit_raw')})",
        f"- Route-found rate: {format_rate(aggregate.get('route_found_rate'))} ({aggregate.get('route_found_raw')})",
        f"- Query API latency mean/p50/p90 (ms): {dict(aggregate.get('query_api_latency_ms') or {}).get('mean_ms')} / {dict(aggregate.get('query_api_latency_ms') or {}).get('p50_ms')} / {dict(aggregate.get('query_api_latency_ms') or {}).get('p90_ms')}",
        f"- Symbolic plan latency mean/p50/p90 (ms): {dict(aggregate.get('symbolic_plan_latency_ms') or {}).get('mean_ms')} / {dict(aggregate.get('symbolic_plan_latency_ms') or {}).get('p50_ms')} / {dict(aggregate.get('symbolic_plan_latency_ms') or {}).get('p90_ms')}",
        f"- Aggregate Tier 1 backend artifact bytes: {aggregate.get('aggregate_backend_artifact_size_bytes')}",
        f"- Aggregate Tier 2 optional demo artifact bytes: {aggregate.get('aggregate_optional_demo_artifact_size_bytes')}",
        "",
        "## Policy Breakdown",
        "",
        "| Policy | Tasks | Success Rate | Route-Found Rate | Mean Latency (ms) |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in policy_rows:
        lines.append(
            f"| {row.get('route_policy')} | {row.get('task_count')} | {format_rate(row.get('task_success_rate'))} | {format_rate(row.get('route_found_rate'))} | {row.get('latency_mean_ms')} |"
        )

    lines.extend(["", "## Task-Type Breakdown", "", "| Task Type | Tasks | Success Rate | Route-Found Rate | Mean Latency (ms) |", "| --- | ---: | ---: | ---: | ---: |"])
    for row in task_type_rows:
        lines.append(
            f"| {row.get('task_type')} | {row.get('task_count')} | {format_rate(row.get('task_success_rate'))} | {format_rate(row.get('route_found_rate'))} | {row.get('latency_mean_ms')} |"
        )

    lines.extend(["", "## Scene Runtime Summary", "", "| Scene | Floors | Rooms | Objects | Nodes | Edges | VT | Tier 1 Bytes | Tier 2 Bytes | Tier 1 Bytes/Room |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"])
    for row in scene_rows:
        lines.append(
            f"| {row.get('sequence_name')} | {row.get('floor_count')} | {row.get('room_count')} | {row.get('object_count')} | {row.get('node_count')} | {row.get('edge_count')} | {row.get('vertical_transition_count')} | {row.get('backend_artifact_size_total_bytes')} | {row.get('optional_demo_artifact_size_total_bytes')} | {row.get('bytes_per_room')} |"
        )

    aggregate_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "results_json": str(results_json),
        "aggregate_json": str(aggregate_json),
        "aggregate_csv": str(aggregate_csv),
        "aggregate_md": str(aggregate_md),
        "task_results_csv": str(task_results_csv),
        "scene_runtime_csv": str(scene_runtime_csv),
        "policy_csv": str(policy_csv),
        "task_type_csv": str(task_type_csv),
    }


def run_evaluation(
    *,
    registry: Dict[str, Any],
    tasks: Sequence[Dict[str, Any]],
    scene_output_root: Path,
    legacy_scene_output_root: Path | None,
) -> Dict[str, Any]:
    registry_lookup = ordered_scene_lookup(registry)
    context_cache: Dict[str, Dict[str, Any]] = {}
    scene_manifests: Dict[str, Dict[str, Any]] = {}
    task_results: List[Dict[str, Any]] = []

    for task in tasks:
        sequence_name = str(task.get("sequence_name"))
        scene_root = find_scene_root(
            sequence_name,
            preferred_root=scene_output_root,
            fallback_roots=[] if legacy_scene_output_root is None else [legacy_scene_output_root],
        )
        if scene_root is None:
            task_results.append(
                {
                    "task_id": task.get("task_id"),
                    "split": task.get("split"),
                    "scene_id": task.get("scene_id"),
                    "sequence_name": sequence_name,
                    "task_family": task.get("task_family"),
                    "task_type": task.get("task_type"),
                    "hierarchy_level": task.get("hierarchy_level"),
                    "scene_role": task.get("scene_role"),
                    "hov_sg_overlap_split": task.get("hov_sg_overlap_split"),
                    "policy": task_policy(task),
                    "route_policy": task_policy(task),
                    "source_room_id": task_source_room(task),
                    "target_granularity": task.get("target_granularity"),
                    "needs_route": task.get("needs_route"),
                    "resolve_only": task.get("resolve_only"),
                    "expected_floor_sensitive": task.get("expected_floor_sensitive"),
                    "expected_room_sensitive": task.get("expected_room_sensitive"),
                    "expected_outcome": task_expected_outcome(task),
                    "expected_success": task.get("expected_success"),
                    "actual_status": "skipped_missing_scene_artifacts",
                    "task_success": False,
                    "expected_target_room": task.get("expected_target_room"),
                    "actual_resolved_room": None,
                    "expected_floor_id": task.get("expected_floor_id"),
                    "actual_resolved_floor_id": None,
                    "target_room_id": task.get("target_room_id"),
                    "target_floor_id": task.get("target_floor_id"),
                    "target_object_label": task.get("target_object_label"),
                    "target_anchor_label": task.get("target_anchor_label"),
                    "exact_room_hit": False if task.get("expected_target_room") else None,
                    "exact_floor_hit": False if task.get("expected_floor_id") else None,
                    "route_found": None,
                    "route_hop_count": None,
                    "route_total_cost": None,
                    "latency_ms": None,
                    "timing_bucket": "none",
                }
            )
            continue

        if sequence_name not in context_cache:
            context_cache[sequence_name] = load_scene_context(sequence_name, scene_root)
            scene_manifests[sequence_name] = dict(context_cache[sequence_name]["manifest"])

        raw_result, latency_ms, timing_bucket = invoke_task(context_cache[sequence_name], task)
        task_results.append(normalize_task_result(task, raw_result, latency_ms, timing_bucket))

    aggregate_summary = summarize_results(task_results=task_results, scene_manifests=scene_manifests)
    return {
        "version": "0.1",
        "registry_path": registry.get("scene_output_root"),
        "task_count": len(tasks),
        "task_results": task_results,
        "scene_manifests": scene_manifests,
        "aggregate_summary": aggregate_summary,
        "scenes_in_registry": list(registry_lookup.keys()),
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the compact backend evaluation scaffold.")
    parser.add_argument(
        "--registry",
        default=str(DEFAULT_REGISTRY_PATH),
        help="Scene registry JSON path.",
    )
    parser.add_argument(
        "--tasks",
        default=str(DEFAULT_TASKS_PATH),
        help="Hierarchical-overlap task JSONL path.",
    )
    parser.add_argument(
        "--scene-output-root",
        default=str(DEFAULT_SCENE_OUTPUT_ROOT),
        help="Preferred regenerated scene root.",
    )
    parser.add_argument(
        "--legacy-scene-output-root",
        default=str(DEFAULT_LEGACY_SCENE_OUTPUT_ROOT),
        help="Migration-only read-only fallback root. Ignored unless --allow-legacy-fallback is set.",
    )
    parser.add_argument(
        "--allow-legacy-fallback",
        action="store_true",
        help="Allow a migration-only read-only fallback scan of the legacy root.",
    )
    parser.add_argument(
        "--report-root",
        default=str(DEFAULT_EVAL_OUTPUT_ROOT),
        help="Directory where compact evaluator artifacts should be written.",
    )
    parser.add_argument(
        "--render-only",
        default="",
        help="If set, skip task execution and re-render reports from an existing backend_eval_results.json payload.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    report_root = Path(args.report_root)
    if args.render_only:
        payload = json.loads(Path(args.render_only).read_text(encoding="utf-8"))
    else:
        registry = load_registry(Path(args.registry))
        tasks = load_jsonl(Path(args.tasks))
        legacy_scene_output_root = resolve_legacy_scene_output_root(
            allow_legacy_fallback=bool(args.allow_legacy_fallback),
            legacy_scene_output_root=args.legacy_scene_output_root,
        )
        payload = run_evaluation(
            registry=registry,
            tasks=tasks,
            scene_output_root=Path(args.scene_output_root),
            legacy_scene_output_root=legacy_scene_output_root,
        )

    outputs = render_outputs(report_root, payload)
    print(json.dumps(outputs, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
