#!/usr/bin/env python3
"""Run a bounded lightweight PID replay parameter and waypoint-shaping sweep.

Each candidate is replayed into its own task-local output directory. The tool
does not modify canonical RouteResults, QueryTasks, or baseline replay outputs.
It does not run ROS, Nav2, AMCL, Gazebo, RViz, Stage-A, or raw RGB-D inference.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import PROJECT_NAME  # noqa: E402
from tools.rslg_pipeline.runtime.replay_pid_runtime_input import read_json, write_json  # noqa: E402


DEFAULT_TARGET_SEGMENT = "seg_05_same_floor_metric_room_13_to_room_14"
BASELINE_PARAMS = {
    "dt": 0.1,
    "max_linear_velocity": 0.25,
    "max_angular_velocity": 0.8,
    "linear_gain": 0.8,
    "angular_gain": 1.5,
    "waypoint_tolerance": 0.12,
    "yaw_tolerance": 0.25,
    "timeout_sec": 240.0,
    "stuck_window_sec": 8.0,
    "stuck_progress_epsilon": 0.02,
    "robot_radius": 0.18,
}


@dataclass
class Candidate:
    candidate_id: str
    label: str
    params: dict[str, float]
    transform: dict[str, Any]


def merged_params(**updates: float) -> dict[str, float]:
    params = dict(BASELINE_PARAMS)
    params.update({key: value for key, value in updates.items() if value is not None})
    return params


def candidate_signature(params: dict[str, float], transform: dict[str, Any]) -> str:
    payload = {
        "params": {key: params[key] for key in sorted(params)},
        "transform": transform,
    }
    return json.dumps(payload, sort_keys=True)


def build_candidates(max_candidates: int) -> list[Candidate]:
    seeds: list[tuple[str, dict[str, float], dict[str, Any]]] = [
        ("baseline_fixed_parameters", merged_params(), {"mode": "none"}),
        ("lower_v_stricter_tol", merged_params(max_linear_velocity=0.15, angular_gain=2.0, waypoint_tolerance=0.08), {"mode": "none"}),
        ("very_low_v_tight_tol", merged_params(max_linear_velocity=0.10, max_angular_velocity=1.0, angular_gain=2.5, waypoint_tolerance=0.06, yaw_tolerance=0.20), {"mode": "none"}),
        ("moderate_v_high_turn_gain", merged_params(max_linear_velocity=0.20, max_angular_velocity=1.0, angular_gain=2.5, waypoint_tolerance=0.08, yaw_tolerance=0.20), {"mode": "none"}),
        ("lower_v_low_linear_gain", merged_params(max_linear_velocity=0.15, linear_gain=0.5, angular_gain=2.0, waypoint_tolerance=0.08), {"mode": "none"}),
        ("densify_010_baseline", merged_params(), {"mode": "densify", "spacing_m": 0.10}),
        ("densify_010_lower_v", merged_params(max_linear_velocity=0.15, angular_gain=2.0, waypoint_tolerance=0.08), {"mode": "densify", "spacing_m": 0.10}),
        ("densify_005_baseline", merged_params(), {"mode": "densify", "spacing_m": 0.05}),
        ("densify_005_tight_low_v", merged_params(max_linear_velocity=0.10, max_angular_velocity=1.0, angular_gain=2.5, waypoint_tolerance=0.06, yaw_tolerance=0.20), {"mode": "densify", "spacing_m": 0.05}),
        ("corner_guard_010_lower_v", merged_params(max_linear_velocity=0.15, max_angular_velocity=1.0, angular_gain=2.5, waypoint_tolerance=0.08, yaw_tolerance=0.20), {"mode": "corner_guard", "spacing_m": 0.10, "turn_angle_rad": 0.45, "guard_distance_m": 0.05}),
        ("corner_guard_005_tight_low_v", merged_params(max_linear_velocity=0.10, max_angular_velocity=1.0, angular_gain=2.5, waypoint_tolerance=0.06, yaw_tolerance=0.20), {"mode": "corner_guard", "spacing_m": 0.05, "turn_angle_rad": 0.45, "guard_distance_m": 0.05}),
    ]

    candidates: list[Candidate] = []
    seen: set[str] = set()

    def add(label: str, params: dict[str, float], transform: dict[str, Any]) -> None:
        signature = candidate_signature(params, transform)
        if signature in seen or len(candidates) >= max_candidates:
            return
        seen.add(signature)
        candidates.append(Candidate(f"candidate_{len(candidates):03d}_{label}", label, params, transform))

    for label, params, transform in seeds:
        add(label, params, transform)

    max_linear_values = [0.10, 0.15, 0.20, 0.25]
    max_angular_values = [0.8, 1.0, 0.6]
    linear_gains = [0.8, 0.5]
    angular_gains = [2.5, 2.0, 1.5]
    waypoint_tolerances = [0.06, 0.08, 0.10, 0.12]
    yaw_tolerances = [0.20, 0.25, 0.30]

    for waypoint_tolerance in waypoint_tolerances:
        for max_linear_velocity in max_linear_values:
            for angular_gain in angular_gains:
                for max_angular_velocity in max_angular_values:
                    for linear_gain in linear_gains:
                        for yaw_tolerance in yaw_tolerances:
                            if len(candidates) >= max_candidates:
                                return candidates
                            label = (
                                f"v{max_linear_velocity:.2f}_w{max_angular_velocity:.1f}_"
                                f"lg{linear_gain:.1f}_ag{angular_gain:.1f}_"
                                f"tol{waypoint_tolerance:.2f}_yaw{yaw_tolerance:.2f}"
                            ).replace(".", "p")
                            add(
                                label,
                                merged_params(
                                    max_linear_velocity=max_linear_velocity,
                                    max_angular_velocity=max_angular_velocity,
                                    linear_gain=linear_gain,
                                    angular_gain=angular_gain,
                                    waypoint_tolerance=waypoint_tolerance,
                                    yaw_tolerance=yaw_tolerance,
                                ),
                                {"mode": "none"},
                            )
    return candidates


def distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def yaw_to_next(waypoints: list[dict[str, Any]], index: int, fallback: float = 0.0) -> float:
    if index >= len(waypoints):
        return fallback
    current = waypoints[index]
    for nxt in waypoints[index + 1 :]:
        if nxt.get("floor_id") != current.get("floor_id"):
            break
        dx = float(nxt["x"]) - float(current["x"])
        dy = float(nxt["y"]) - float(current["y"])
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            return math.atan2(dy, dx)
    return float(current.get("yaw") if current.get("yaw") is not None else fallback)


def interpolate_waypoint(before: dict[str, Any], after: dict[str, Any], alpha: float, source: str) -> dict[str, Any]:
    x = float(before["x"]) + (float(after["x"]) - float(before["x"])) * alpha
    y = float(before["y"]) + (float(after["y"]) - float(before["y"])) * alpha
    z = float(before.get("z") or 0.0) + (float(after.get("z") or 0.0) - float(before.get("z") or 0.0)) * alpha
    yaw = math.atan2(float(after["y"]) - float(before["y"]), float(after["x"]) - float(before["x"]))
    item = dict(before)
    item.update(
        {
            "x": round(x, 6),
            "y": round(y, 6),
            "z": round(z, 6),
            "floor_id": after.get("floor_id") or before.get("floor_id"),
            "yaw": round(yaw, 6),
            "segment_id": before.get("segment_id") or after.get("segment_id"),
            "source": source,
        }
    )
    return item


def turn_angle(before: dict[str, Any], current: dict[str, Any], after: dict[str, Any]) -> float:
    heading_in = math.atan2(float(current["y"]) - float(before["y"]), float(current["x"]) - float(before["x"]))
    heading_out = math.atan2(float(after["y"]) - float(current["y"]), float(after["x"]) - float(current["x"]))
    return abs(math.atan2(math.sin(heading_out - heading_in), math.cos(heading_out - heading_in)))


def insert_corner_guard_points(
    original: list[dict[str, Any]],
    samples: list[dict[str, Any]],
    *,
    turn_angle_rad: float,
    guard_distance_m: float,
) -> list[dict[str, Any]]:
    guarded = list(samples)
    for index in range(1, len(original) - 1):
        before = original[index - 1]
        current = original[index]
        after = original[index + 1]
        if before.get("floor_id") != current.get("floor_id") or after.get("floor_id") != current.get("floor_id"):
            continue
        if turn_angle(before, current, after) < turn_angle_rad:
            continue
        in_dist = distance(before, current)
        out_dist = distance(current, after)
        if in_dist > 1e-9:
            alpha = max(0.0, min(1.0, (in_dist - guard_distance_m) / in_dist))
            guarded.append(interpolate_waypoint(before, current, alpha, "task56b_corner_guard_pre_turn"))
        guarded.append(dict(current, source="task56b_corner_guard_turn_vertex"))
        if out_dist > 1e-9:
            alpha = max(0.0, min(1.0, guard_distance_m / out_dist))
            guarded.append(interpolate_waypoint(current, after, alpha, "task56b_corner_guard_post_turn"))
    return sort_and_deduplicate_along_path(original, guarded)


def cumulative_position(original: list[dict[str, Any]], point: dict[str, Any]) -> float:
    if len(original) < 2:
        return 0.0
    best_s = 0.0
    best_dist: Optional[float] = None
    s = 0.0
    px = float(point["x"])
    py = float(point["y"])
    for before, after in zip(original, original[1:]):
        ax = float(before["x"])
        ay = float(before["y"])
        bx = float(after["x"])
        by = float(after["y"])
        vx = bx - ax
        vy = by - ay
        denom = vx * vx + vy * vy
        seg_len = math.sqrt(denom)
        if denom <= 1e-12:
            proj = 0.0
            dist = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / denom))
            qx = ax + t * vx
            qy = ay + t * vy
            proj = t * seg_len
            dist = math.hypot(px - qx, py - qy)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_s = s + proj
        s += seg_len
    return best_s


def sort_and_deduplicate_along_path(original: list[dict[str, Any]], waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
    ordered = sorted(waypoints, key=lambda item: cumulative_position(original, item))
    deduped: list[dict[str, Any]] = []
    for waypoint in ordered:
        if deduped and math.hypot(float(waypoint["x"]) - float(deduped[-1]["x"]), float(waypoint["y"]) - float(deduped[-1]["y"])) < 1e-5:
            continue
        deduped.append(waypoint)
    return deduped


def densify_segment(original: list[dict[str, Any]], transform: dict[str, Any]) -> list[dict[str, Any]]:
    mode = transform.get("mode")
    if mode not in {"densify", "corner_guard"} or len(original) < 2:
        return [dict(item) for item in original]
    spacing = float(transform.get("spacing_m") or 0.10)
    spacing = max(spacing, 0.01)
    samples: list[dict[str, Any]] = []
    for pair_index, (before, after) in enumerate(zip(original, original[1:])):
        seg_dist = distance(before, after)
        steps = max(1, int(math.ceil(seg_dist / spacing)))
        if pair_index == 0:
            samples.append(dict(before, source="task56b_densification_original"))
        for step in range(1, steps + 1):
            alpha = step / steps
            source = "task56b_densification_original" if step == steps else "task56b_densification_inserted"
            samples.append(interpolate_waypoint(before, after, alpha, source))
    if mode == "corner_guard":
        samples = insert_corner_guard_points(
            original,
            samples,
            turn_angle_rad=float(transform.get("turn_angle_rad") or 0.45),
            guard_distance_m=float(transform.get("guard_distance_m") or 0.05),
        )
    return samples


def reindex_waypoints(waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
    reindexed: list[dict[str, Any]] = []
    for index, waypoint in enumerate(waypoints):
        updated = dict(waypoint)
        updated["waypoint_index"] = index
        updated["yaw"] = round(yaw_to_next(waypoints, index, fallback=float(updated.get("yaw") or 0.0)), 6)
        reindexed.append(updated)
    return reindexed


def transform_runtime_waypoints(
    payload: dict[str, Any],
    *,
    target_segment_id: str,
    transform: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    mode = transform.get("mode")
    if mode in {None, "none"}:
        copied = json.loads(json.dumps(payload))
        copied["task56b_replay_transform"] = {"mode": "none", "target_segment_id": target_segment_id}
        return copied, {"mode": "none", "target_segment_id": target_segment_id, "inserted_waypoints": 0}

    copied = json.loads(json.dumps(payload))
    waypoints = copied.get("runtime_waypoints") or []
    target = [wp for wp in waypoints if str(wp.get("segment_id")) == target_segment_id]
    transformed_target = densify_segment(target, transform)
    replaced: list[dict[str, Any]] = []
    inserted = 0
    consumed_target = False
    for waypoint in waypoints:
        if str(waypoint.get("segment_id")) == target_segment_id:
            if not consumed_target:
                replaced.extend(transformed_target)
                inserted = max(0, len(transformed_target) - len(target))
                consumed_target = True
            continue
        replaced.append(waypoint)
    copied["runtime_waypoints"] = reindex_waypoints(replaced)

    for segment in copied.get("runtime_segments") or []:
        if str(segment.get("segment_id")) != target_segment_id:
            continue
        segment["waypoints"] = [
            {
                "x": wp["x"],
                "y": wp["y"],
                "z": wp.get("z"),
                "floor_id": wp.get("floor_id"),
            }
            for wp in transformed_target
        ]
        segment["waypoint_count"] = len(transformed_target)
        segment["task56b_replay_transform"] = dict(transform)

    transform_record = dict(transform)
    transform_record.update(
        {
            "target_segment_id": target_segment_id,
            "original_target_waypoint_count": len(target),
            "transformed_target_waypoint_count": len(transformed_target),
            "inserted_waypoints": inserted,
        }
    )
    copied["task56b_replay_transform"] = transform_record
    return copied, transform_record


def prepare_pid_inputs(
    source_dir: Path,
    destination_dir: Path,
    *,
    target_segment_id: str,
    transform: dict[str, Any],
) -> dict[str, Any]:
    destination_dir.mkdir(parents=True, exist_ok=True)
    transform_records: list[dict[str, Any]] = []
    for source_path in sorted(source_dir.glob("*_pid_runtime_input.json")):
        payload = read_json(source_path)
        transformed, record = transform_runtime_waypoints(payload, target_segment_id=target_segment_id, transform=transform)
        transform_records.append({"path": source_path.name, **record})
        write_json(destination_dir / source_path.name, transformed)
    return {
        "source_dir": source_dir.as_posix(),
        "destination_dir": destination_dir.as_posix(),
        "target_segment_id": target_segment_id,
        "transform": transform,
        "records": transform_records,
    }


def replay_command(
    *,
    replay_script: Path,
    pid_inputs_dir: Path,
    route_results_dir: Path,
    z_aware_inputs_dir: Optional[Path],
    output_dir: Path,
    floor_z_map: Optional[str],
    title: str,
    params: dict[str, float],
) -> list[str]:
    command = [
        sys.executable,
        replay_script.as_posix(),
        "--pid-inputs-dir",
        pid_inputs_dir.as_posix(),
        "--route-results-dir",
        route_results_dir.as_posix(),
        "--output-dir",
        output_dir.as_posix(),
        "--title",
        title,
    ]
    if z_aware_inputs_dir:
        command.extend(["--z-aware-inputs-dir", z_aware_inputs_dir.as_posix()])
    if floor_z_map:
        command.extend(["--floor-z-map", floor_z_map])
    for key in [
        "dt",
        "max_linear_velocity",
        "max_angular_velocity",
        "linear_gain",
        "angular_gain",
        "waypoint_tolerance",
        "yaw_tolerance",
        "timeout_sec",
        "stuck_window_sec",
        "stuck_progress_epsilon",
        "robot_radius",
    ]:
        command.extend([f"--{key.replace('_', '-')}", str(params[key])])
    return command


def summarize_replay(summary_path: Path, target_segment_id: str) -> dict[str, Any]:
    summary = read_json(summary_path)
    reports = summary.get("reports") or []
    target_collisions = 0
    target_invalid = 0
    total_collisions = 0
    total_invalid = 0
    stuck_timeout = 0
    for report in reports:
        total_collisions += int(report.get("collision_count") or 0)
        total_invalid += int(report.get("invalid_cell_count") or 0)
        stuck_timeout += int(bool(report.get("stuck_detected"))) + int(bool(report.get("timeout")))
        for segment in report.get("segment_reports") or []:
            if str(segment.get("segment_id")) == target_segment_id:
                target_collisions += int(segment.get("collision_count") or 0)
                target_invalid += int(segment.get("invalid_cell_count") or 0)
    return {
        "query_count": summary.get("query_count"),
        "success_count": summary.get("success_count"),
        "partial_count": summary.get("partial_count"),
        "failure_count": summary.get("failure_count"),
        "average_final_error": summary.get("average_final_error"),
        "average_waypoint_reached_ratio": summary.get("average_waypoint_reached_ratio"),
        "collision_check_mode": summary.get("collision_check_mode"),
        "collision_free_in_sim_count": summary.get("collision_free_in_sim_count"),
        "total_collision_count": total_collisions,
        "target_segment_collision_count": target_collisions,
        "invalid_cell_count": total_invalid,
        "target_segment_invalid_cell_count": target_invalid,
        "stuck_timeout_count": stuck_timeout,
        "generated_ring_037_selected_count": summary.get("generated_ring_037_selected_count"),
        "vt_1_centerline_e003_transition_count": summary.get("vt_1_centerline_e003_transition_count"),
        "summary_json": summary_path.as_posix(),
    }


def candidate_sort_key(candidate: dict[str, Any], baseline_invalid: Optional[int]) -> tuple[Any, ...]:
    metrics = candidate.get("metrics") or {}
    failed_run = candidate.get("returncode") != 0
    guard_bad = (
        int(metrics.get("generated_ring_037_selected_count") or 0) != 0
        or int(metrics.get("vt_1_centerline_e003_transition_count") or 0) != 0
    )
    failure_count = int(metrics["failure_count"]) if "failure_count" in metrics else 999
    stuck_timeout = int(metrics["stuck_timeout_count"]) if "stuck_timeout_count" in metrics else 999
    reach_ratio = float(metrics.get("average_waypoint_reached_ratio") or 0.0)
    invalid = int(metrics.get("invalid_cell_count") or 0)
    invalid_penalty = 0 if baseline_invalid is None or invalid <= baseline_invalid else 1
    final_error = float(metrics.get("average_final_error") or 999.0)
    final_error_penalty = 0 if final_error <= 0.15 else 1
    target_collisions = (
        int(metrics["target_segment_collision_count"])
        if "target_segment_collision_count" in metrics
        else 999999
    )
    total_collisions = int(metrics["total_collision_count"]) if "total_collision_count" in metrics else 999999
    return (
        failed_run,
        guard_bad,
        failure_count,
        stuck_timeout,
        invalid_penalty,
        -reach_ratio,
        target_collisions,
        total_collisions,
        final_error_penalty,
        final_error,
        candidate.get("candidate_id"),
    )


def render_sweep_md(summary: dict[str, Any]) -> str:
    lines = [
        "# PID Runtime Replay Parameter Sweep",
        "",
        f"- project: `{summary['project_name']}`",
        f"- title: `{summary['title']}`",
        f"- target_segment_id: `{summary['target_segment_id']}`",
        f"- candidates_tested: `{summary['candidates_tested']}`",
        f"- best_candidate_id: `{(summary.get('best_candidate') or {}).get('candidate_id')}`",
        f"- best_selection_reason: `{summary.get('best_selection_reason')}`",
        "",
        "| candidate | transform | success/partial/failed | target collisions | total collisions | final error | reach ratio | guards |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for candidate in summary.get("candidates") or []:
        metrics = candidate.get("metrics") or {}
        guards = (
            f"ring037={metrics.get('generated_ring_037_selected_count')}, "
            f"e003={metrics.get('vt_1_centerline_e003_transition_count')}"
        )
        lines.append(
            "| "
            f"`{candidate['candidate_id']}` | `{candidate['transform'].get('mode')}` | "
            f"{metrics.get('success_count')}/{metrics.get('partial_count')}/{metrics.get('failure_count')} | "
            f"{metrics.get('target_segment_collision_count')} | {metrics.get('total_collision_count')} | "
            f"{metrics.get('average_final_error')} | {metrics.get('average_waypoint_reached_ratio')} | {guards} |"
        )
    lines.extend(
        [
            "",
            "Selection requires zero generated_ring_037 selections, zero vt_1_centerline_e003 transitions, zero failures, no stuck/timeout, average waypoint reach ratio 1.0, no invalid-cell increase, then minimizes target and total stable-map footprint collisions.",
            "",
        ]
    )
    return "\n".join(lines)


def render_densification_md(summary: dict[str, Any]) -> str:
    lines = [
        "# Waypoint Densification And Corner Guard Trials",
        "",
        f"- target_segment_id: `{summary['target_segment_id']}`",
        "",
        "| candidate | mode | spacing | inserted waypoints | target collisions | success/partial/failed | final error | reach ratio |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for candidate in summary.get("candidates") or []:
        transform = candidate.get("transform") or {}
        if transform.get("mode") not in {"densify", "corner_guard"}:
            continue
        metrics = candidate.get("metrics") or {}
        records = (candidate.get("transform_manifest") or {}).get("records") or []
        inserted = max((int(record.get("inserted_waypoints") or 0) for record in records), default=0)
        lines.append(
            "| "
            f"`{candidate['candidate_id']}` | `{transform.get('mode')}` | {transform.get('spacing_m')} | "
            f"{inserted} | {metrics.get('target_segment_collision_count')} | "
            f"{metrics.get('success_count')}/{metrics.get('partial_count')}/{metrics.get('failure_count')} | "
            f"{metrics.get('average_final_error')} | {metrics.get('average_waypoint_reached_ratio')} |"
        )
    lines.extend(
        [
            "",
            "These are task-local replay-time waypoint transformations. They do not modify canonical RouteResults, QueryTasks, or planner behavior.",
            "",
        ]
    )
    return "\n".join(lines)


def write_candidate_csv(path: Path, candidates: list[dict[str, Any]]) -> None:
    fields = [
        "candidate_id",
        "label",
        "returncode",
        "transform_mode",
        "spacing_m",
        "success_count",
        "partial_count",
        "failure_count",
        "target_segment_collision_count",
        "total_collision_count",
        "invalid_cell_count",
        "stuck_timeout_count",
        "average_final_error",
        "average_waypoint_reached_ratio",
        "generated_ring_037_selected_count",
        "vt_1_centerline_e003_transition_count",
        "replay_output_dir",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for candidate in candidates:
            metrics = candidate.get("metrics") or {}
            transform = candidate.get("transform") or {}
            writer.writerow(
                {
                    "candidate_id": candidate.get("candidate_id"),
                    "label": candidate.get("label"),
                    "returncode": candidate.get("returncode"),
                    "transform_mode": transform.get("mode"),
                    "spacing_m": transform.get("spacing_m"),
                    "success_count": metrics.get("success_count"),
                    "partial_count": metrics.get("partial_count"),
                    "failure_count": metrics.get("failure_count"),
                    "target_segment_collision_count": metrics.get("target_segment_collision_count"),
                    "total_collision_count": metrics.get("total_collision_count"),
                    "invalid_cell_count": metrics.get("invalid_cell_count"),
                    "stuck_timeout_count": metrics.get("stuck_timeout_count"),
                    "average_final_error": metrics.get("average_final_error"),
                    "average_waypoint_reached_ratio": metrics.get("average_waypoint_reached_ratio"),
                    "generated_ring_037_selected_count": metrics.get("generated_ring_037_selected_count"),
                    "vt_1_centerline_e003_transition_count": metrics.get("vt_1_centerline_e003_transition_count"),
                    "replay_output_dir": candidate.get("replay_output_dir"),
                }
            )


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    candidates_root = output_dir / "candidates"
    candidates_root.mkdir(parents=True, exist_ok=True)
    replay_script = Path(__file__).with_name("replay_pid_runtime_input.py")
    repo_root = Path(__file__).resolve().parents[3]
    max_candidates = max(1, int(args.max_candidates))
    planned_candidates = build_candidates(max_candidates)

    executed: list[dict[str, Any]] = []
    for candidate in planned_candidates:
        candidate_dir = candidates_root / candidate.candidate_id
        pid_inputs_dir = candidate_dir / "pid_inputs"
        replay_output_dir = candidate_dir / "pid_replay"
        transform_manifest = prepare_pid_inputs(
            args.pid_inputs_dir,
            pid_inputs_dir,
            target_segment_id=args.target_segment_id,
            transform=candidate.transform,
        )
        write_json(candidate_dir / "transform_manifest.json", transform_manifest)
        command = replay_command(
            replay_script=replay_script,
            pid_inputs_dir=pid_inputs_dir,
            route_results_dir=args.route_results_dir,
            z_aware_inputs_dir=args.z_aware_inputs_dir,
            output_dir=replay_output_dir,
            floor_z_map=args.floor_z_map,
            title=f"{args.title}: {candidate.candidate_id}",
            params=candidate.params,
        )
        (candidate_dir / "replay_command.json").write_text(json.dumps(command, indent=2) + "\n", encoding="utf-8")
        proc = subprocess.run(command, cwd=repo_root, check=False, text=True, capture_output=True)
        (candidate_dir / "stdout.log").write_text(proc.stdout, encoding="utf-8")
        (candidate_dir / "stderr.log").write_text(proc.stderr, encoding="utf-8")
        metrics: dict[str, Any] = {}
        summary_path = replay_output_dir / "summary.json"
        if summary_path.exists():
            metrics = summarize_replay(summary_path, args.target_segment_id)
        executed.append(
            {
                "candidate_id": candidate.candidate_id,
                "label": candidate.label,
                "params": candidate.params,
                "transform": candidate.transform,
                "transform_manifest": transform_manifest,
                "returncode": proc.returncode,
                "replay_output_dir": replay_output_dir.as_posix(),
                "metrics": metrics,
            }
        )

    baseline_invalid = None
    if executed:
        baseline_invalid = int((executed[0].get("metrics") or {}).get("invalid_cell_count") or 0)
    ranked = sorted(executed, key=lambda item: candidate_sort_key(item, baseline_invalid))
    best_candidate = ranked[0] if ranked else None
    best_selection_reason = (
        "ranked_by_guard_constraints_failures_stuck_timeout_reach_ratio_invalid_cells_then_collision_count"
        if best_candidate
        else "no_candidates_executed"
    )

    summary = {
        "schema_name": "rslg_pid_runtime_replay_parameter_sweep",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "title": args.title,
        "target_segment_id": args.target_segment_id,
        "output_dir": output_dir.as_posix(),
        "candidate_limit": max_candidates,
        "candidates_tested": len(executed),
        "parameter_grid": {
            "max_linear_velocity": [0.10, 0.15, 0.20, 0.25],
            "max_angular_velocity": [0.6, 0.8, 1.0],
            "linear_gain": [0.5, 0.8],
            "angular_gain": [1.5, 2.0, 2.5],
            "waypoint_tolerance": [0.06, 0.08, 0.10, 0.12],
            "yaw_tolerance": [0.20, 0.25, 0.30],
        },
        "selection_criteria": [
            "generated_ring_037_selected_count == 0",
            "vt_1_centerline_e003_transition_count == 0",
            "failure_count == 0",
            "stuck_timeout_count == 0",
            "average_waypoint_reached_ratio == 1.0",
            "minimize stable-map footprint collisions",
            "prefer average_final_error <= 0.15",
            "do not increase invalid cells",
        ],
        "best_selection_reason": best_selection_reason,
        "best_candidate": best_candidate,
        "candidates": executed,
    }
    write_json(output_dir / "pid_parameter_sweep_summary.json", summary)
    (output_dir / "pid_parameter_sweep_summary.md").write_text(render_sweep_md(summary), encoding="utf-8")
    (output_dir / "waypoint_densification_and_corner_guard_summary.md").write_text(render_densification_md(summary), encoding="utf-8")
    write_candidate_csv(output_dir / "candidate_metrics.csv", executed)

    print(
        json.dumps(
            {
                "classification": "pid_runtime_replay_parameter_sweep_completed",
                "candidates_tested": len(executed),
                "best_candidate_id": (best_candidate or {}).get("candidate_id"),
                "output_dir": output_dir.as_posix(),
            },
            indent=2,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-inputs-dir", type=Path, required=True)
    parser.add_argument("--route-results-dir", type=Path, required=True)
    parser.add_argument("--z-aware-inputs-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--target-segment-id", default=DEFAULT_TARGET_SEGMENT)
    parser.add_argument("--max-candidates", type=int, default=40)
    parser.add_argument("--title", default="RSLG-SLAM PID Replay Mitigation Sweep")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
