#!/usr/bin/env python3
"""Compare 00824 and 00843 enter-and-return route execution evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]

DEFAULT_00824_RUN = ROOT / "stage_outputs/stage1_00824_step30p1/current_validation/room15_manual_watch_20260515_093900"
DEFAULT_00843_RUN = ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun/runs/active/task6_gui_debug"
DEFAULT_00843_ROUTE = ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun/routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json"
DEFAULT_OUT = ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task7_00824_00843_hairpin_compare"


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def waypoints_from(path: Path) -> list[dict[str, Any]]:
    data = load_json(path)
    wps = data.get("waypoints", data if isinstance(data, list) else [])
    return [dict(w, waypoint_index=int(w.get("waypoint_index", i))) for i, w in enumerate(wps)]


def norm_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


def point(w: dict[str, Any]) -> tuple[float, float]:
    return float(w["x"]), float(w["y"])


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_waypoint_index(wps: list[dict[str, Any]], x: float, y: float) -> tuple[int | None, float | None]:
    if not wps:
        return None, None
    p = (x, y)
    best_i = min(range(len(wps)), key=lambda i: dist(point(wps[i]), p))
    return int(wps[best_i]["waypoint_index"]), dist(point(wps[best_i]), p)


def nearest_waypoint_index_in_range(
    wps: list[dict[str, Any]], x: float, y: float, start_idx: int, stop_idx: int
) -> tuple[int | None, float | None]:
    candidates = [w for w in wps if start_idx <= int(w["waypoint_index"]) <= stop_idx]
    return nearest_waypoint_index(candidates, x, y)


def distance_point_to_segment(
    p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return dist(p, a)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    q = (ax + t * dx, ay + t * dy)
    return dist(p, q)


def nearest_ordered_segment(
    wps: list[dict[str, Any]], x: float, y: float, start_idx: int, stop_idx: int
) -> tuple[int | None, float | None]:
    p = (x, y)
    best = (None, None)
    for i in range(max(0, start_idx), min(len(wps) - 1, stop_idx) + 1):
        d = distance_point_to_segment(p, point(wps[i]), point(wps[i + 1]))
        if best[1] is None or d < best[1]:
            best = (int(wps[i]["waypoint_index"]), d)
    return best


def segment_heading(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.atan2(float(b["y"]) - float(a["y"]), float(b["x"]) - float(a["x"]))


def source_counts(wps: list[dict[str, Any]]) -> dict[str, int]:
    return dict(sorted(Counter(str(w.get("source", "")) for w in wps).items()))


def local_window(wps: list[dict[str, Any]], start: int, end: int) -> list[dict[str, Any]]:
    return [w for w in wps if start <= int(w["waypoint_index"]) <= end]


def geometry_metrics(name: str, wps: list[dict[str, Any]], fold_idx: int) -> dict[str, Any]:
    spacings = [dist(point(wps[i]), point(wps[i + 1])) for i in range(len(wps) - 1)]
    yaw_changes = [
        abs(math.degrees(norm_angle(float(wps[i + 1].get("yaw", 0.0)) - float(wps[i].get("yaw", 0.0)))))
        for i in range(len(wps) - 1)
    ]
    turn_angles = []
    for i in range(1, len(wps) - 1):
        h1 = segment_heading(wps[i - 1], wps[i])
        h2 = segment_heading(wps[i], wps[i + 1])
        turn_angles.append(abs(math.degrees(norm_angle(h2 - h1))))

    min_sep = None
    close_pairs = []
    for i in range(len(wps)):
        for j in range(i + 3, len(wps)):
            d = dist(point(wps[i]), point(wps[j]))
            if min_sep is None or d < min_sep:
                min_sep = d
            if d <= 0.15:
                close_pairs.append(
                    {
                        "i": int(wps[i]["waypoint_index"]),
                        "j": int(wps[j]["waypoint_index"]),
                        "distance_m": round(d, 6),
                    }
                )

    signs = []
    for i in range(len(wps) - 1):
        dy = float(wps[i + 1]["y"]) - float(wps[i]["y"])
        signs.append(1 if dy > 1e-6 else -1 if dy < -1e-6 else 0)
    sign_reversals = sum(1 for a, b in zip(signs, signs[1:]) if a and b and a != b)
    fold_wp = next((w for w in wps if int(w["waypoint_index"]) == fold_idx), None)

    return {
        "name": name,
        "window_waypoint_indices": [int(w["waypoint_index"]) for w in wps],
        "waypoint_count": len(wps),
        "source_counts": source_counts(wps),
        "local_route_length_m": round(sum(spacings), 6),
        "spacing_min_m": round(min(spacings), 6) if spacings else None,
        "spacing_mean_m": round(sum(spacings) / len(spacings), 6) if spacings else None,
        "spacing_max_m": round(max(spacings), 6) if spacings else None,
        "max_yaw_heading_change_deg": round(max(yaw_changes), 6) if yaw_changes else None,
        "cumulative_yaw_heading_change_deg": round(sum(yaw_changes), 6),
        "max_geometric_turn_deg": round(max(turn_angles), 6) if turn_angles else None,
        "cumulative_geometric_turn_deg": round(sum(turn_angles), 6),
        "heading_sign_reversals_y": sign_reversals,
        "min_non_adjacent_branch_separation_m": round(min_sep, 6) if min_sep is not None else None,
        "close_non_adjacent_pairs_le_0p15m": close_pairs,
        "semantic_anchor_at_fold": bool(fold_wp and str(fold_wp.get("source")) in {"room_center", "room15_interior_terminal"}),
        "fold_waypoint": fold_wp,
    }


def read_log_flags(nav2_log: Path) -> dict[str, Any]:
    text = nav2_log.read_text(encoding="utf-8", errors="replace") if nav2_log.exists() else ""
    pats = {
        "failed_to_make_progress": r"Failed to make progress",
        "no_valid_trajectories": r"No valid",
        "reached_goal_count": r"Reached the goal!",
        "follow_path_abort": r"\[follow_path\].*Aborting handle",
        "recovery_spin_mentions": r"\bspin\b|Spin",
    }
    out: dict[str, Any] = {}
    for k, pat in pats.items():
        matches = re.findall(pat, text, re.IGNORECASE)
        out[k] = len(matches) if k.endswith("_count") or k.endswith("_mentions") else bool(matches)
    return out


def max_stall_duration(rows: list[dict[str, Any]], lin_thresh: float = 0.02, ang_thresh: float = 0.5) -> float | None:
    spans = []
    start = None
    end = None
    for row in rows:
        lin = row.get("cmd_vel_linear_x")
        ang = row.get("cmd_vel_angular_z")
        t = row.get("t_sec")
        active = lin is not None and ang is not None and t is not None and abs(float(lin)) <= lin_thresh and abs(float(ang)) >= ang_thresh
        if active:
            if start is None:
                start = float(t)
            end = float(t)
        elif start is not None:
            spans.append(max(0.0, (end or start) - start))
            start = None
            end = None
    if start is not None:
        spans.append(max(0.0, (end or start) - start))
    return round(max(spans), 6) if spans else None


def jump_summary(indices: list[int | None]) -> tuple[list[dict[str, int]], int]:
    jumps = []
    backwards = 0
    prev = None
    for sample_i, idx in enumerate(indices):
        if idx is None:
            continue
        if prev is not None:
            delta = idx - prev
            if delta >= 4:
                jumps.append({"sample_index": sample_i, "from": prev, "to": idx, "delta": delta})
            if delta < 0:
                backwards += 1
        prev = idx
    return jumps, backwards


def execution_metrics(
    name: str,
    route_wps: list[dict[str, Any]],
    route_result_path: Path,
    trajectory_path: Path,
    nav2_log: Path,
    local_start: int,
    local_end: int,
    fold_idx: int,
    trace_path: Path | None = None,
    semantic_status_path: Path | None = None,
    spin_validation_path: Path | None = None,
) -> dict[str, Any]:
    result = load_json(route_result_path)
    traj = load_json(trajectory_path).get("samples", [])
    trace_rows = load_jsonl(trace_path) if trace_path else []
    rows_for_cmd = trace_rows if trace_rows else traj

    local_route = local_window(route_wps, local_start, local_end)
    route_by_idx = {int(w["waypoint_index"]): w for w in route_wps}
    full_nearest_indices = []
    full_yaw_errors = []
    active_nearest_indices = []
    active_yaw_errors = []
    ordered_segment_distances = []

    attempts = result.get("follow_path_attempts") or []
    split_at_fold = any(int(a.get("path_stop_slice_index", -9999)) == fold_idx for a in attempts)
    reached_fold = False
    for row in traj:
        if "x" not in row or "y" not in row:
            continue
        x = float(row["x"])
        y = float(row["y"])
        ni, _nd = nearest_waypoint_index(route_wps, x, y)
        full_nearest_indices.append(ni)
        if ni in route_by_idx and "yaw" in row:
            full_yaw_errors.append(abs(math.degrees(norm_angle(float(row["yaw"]) - float(route_by_idx[ni].get("yaw", 0.0))))))

        fold_wp_for_projection = route_by_idx.get(fold_idx)
        if split_at_fold and fold_wp_for_projection is not None and dist((x, y), point(fold_wp_for_projection)) <= 0.25:
            reached_fold = True
        if split_at_fold:
            active_start = 0 if not reached_fold else fold_idx
            active_stop = fold_idx if not reached_fold else max(route_by_idx)
            ai, _ad = nearest_waypoint_index_in_range(route_wps, x, y, active_start, active_stop)
        else:
            ai = ni
        active_nearest_indices.append(ai)
        if ai in route_by_idx and "yaw" in row:
            active_yaw_errors.append(abs(math.degrees(norm_angle(float(row["yaw"]) - float(route_by_idx[ai].get("yaw", 0.0))))))

        if local_start <= (ai if ai is not None else -9999) <= local_end:
            _seg_i, seg_d = nearest_ordered_segment(route_wps, float(row["x"]), float(row["y"]), local_start, local_end - 1)
            if seg_d is not None:
                ordered_segment_distances.append(seg_d)

    full_jumps, full_backwards = jump_summary(full_nearest_indices)
    active_jumps, active_backwards = jump_summary(active_nearest_indices)

    fold_wp = route_by_idx.get(fold_idx)
    min_fold_dist = None
    first_full_post_fold_before_anchor = None
    first_active_post_fold_before_anchor = None
    full_reached_fold = False
    active_reached_fold = False
    for row, full_idx, active_idx in zip(traj, full_nearest_indices, active_nearest_indices):
        if "x" not in row or "y" not in row or fold_wp is None:
            continue
        d = dist((float(row["x"]), float(row["y"])), point(fold_wp))
        if min_fold_dist is None or d < min_fold_dist:
            min_fold_dist = d
        if d <= 0.25:
            full_reached_fold = True
            active_reached_fold = True
        if full_idx is not None and full_idx > fold_idx and d > 0.25 and not full_reached_fold and first_full_post_fold_before_anchor is None:
            first_full_post_fold_before_anchor = {
                "sample_index": row.get("sample_index"),
                "nearest_waypoint_index": full_idx,
                "distance_to_fold_anchor_m": round(d, 6),
                "x": row.get("x"),
                "y": row.get("y"),
            }
        if active_idx is not None and active_idx > fold_idx and d > 0.25 and not active_reached_fold and first_active_post_fold_before_anchor is None:
            first_active_post_fold_before_anchor = {
                "sample_index": row.get("sample_index"),
                "nearest_waypoint_index": active_idx,
                "distance_to_fold_anchor_m": round(d, 6),
                "x": row.get("x"),
                "y": row.get("y"),
            }

    semantic_goals = load_json(semantic_status_path).get("semantic_goals", []) if semantic_status_path and semantic_status_path.exists() else []
    fold_semantic_status = next((g for g in semantic_goals if int(g.get("waypoint_index", -1)) == fold_idx), None)
    spin_validation = load_json(spin_validation_path) if spin_validation_path and spin_validation_path.exists() else None

    return {
        "name": name,
        "route_result_path": str(route_result_path.relative_to(ROOT)),
        "trajectory_path": str(trajectory_path.relative_to(ROOT)),
        "nav2_log_path": str(nav2_log.relative_to(ROOT)),
        "follow_path_used": result.get("follow_path_used"),
        "follow_path_succeeded": bool((result.get("follow_path_result") or {}).get("success"))
        if result.get("follow_path_result") is not None
        else None,
        "succeeded": result.get("succeeded"),
        "execution_strategy": result.get("execution_strategy"),
        "sparse_fallback_used": result.get("sparse_fallback_used"),
        "follow_path_attempts": result.get("follow_path_attempts"),
        "fallback_started_at_waypoint_index": result.get("fallback_started_at_waypoint_index"),
        "projected_next_path_start_slice_index": result.get("projected_next_path_start_slice_index"),
        "trajectory_sample_count": len(traj),
        "split_aware_projection_used": split_at_fold,
        "active_projection_jump_count_ge_4": len(active_jumps),
        "active_projection_jumps_ge_4": active_jumps,
        "active_projection_backward_count": active_backwards,
        "full_route_projection_jump_count_ge_4": len(full_jumps),
        "full_route_projection_jumps_ge_4": full_jumps,
        "full_route_projection_backward_count": full_backwards,
        "min_distance_to_fold_anchor_m": round(min_fold_dist, 6) if min_fold_dist is not None else None,
        "first_full_route_post_fold_projection_before_anchor_tolerance": first_full_post_fold_before_anchor,
        "first_active_post_fold_projection_before_anchor_tolerance": first_active_post_fold_before_anchor,
        "max_yaw_error_deg": round(max(active_yaw_errors), 6) if active_yaw_errors else None,
        "full_route_max_yaw_error_deg": round(max(full_yaw_errors), 6) if full_yaw_errors else None,
        "mean_local_ordered_segment_distance_m": round(sum(ordered_segment_distances) / len(ordered_segment_distances), 6)
        if ordered_segment_distances
        else None,
        "max_local_ordered_segment_distance_m": round(max(ordered_segment_distances), 6) if ordered_segment_distances else None,
        "spin_stall_duration_sec": max_stall_duration(rows_for_cmd),
        "local_spin_validation": spin_validation,
        "log_flags": read_log_flags(nav2_log),
        "fold_semantic_status": fold_semantic_status,
    }


def write_waypoint_window(path: Path, name: str, wps: list[dict[str, Any]]) -> None:
    payload = {
        "artifact_type": "task7_local_hairpin_waypoint_window",
        "name": name,
        "waypoints": wps,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def maybe_plot(out_dir: Path, cases: dict[str, dict[str, Any]]) -> list[str]:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return []

    generated = []
    for name, data in cases.items():
        fig, ax = plt.subplots(figsize=(6, 5))
        local = data["local_wps"]
        traj = data["trajectory"]
        ax.plot([w["x"] for w in local], [w["y"] for w in local], "-o", label="local route", linewidth=2, markersize=4)
        for w in local:
            ax.annotate(str(w["waypoint_index"]), (w["x"], w["y"]), fontsize=7)
        if traj:
            ax.plot([s["x"] for s in traj], [s["y"] for s in traj], ".", label="trajectory", alpha=0.45, markersize=3)
        ax.set_title(name)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25)
        ax.legend()
        p = out_dir / f"{name}_local_route.png"
        fig.tight_layout()
        fig.savefig(p, dpi=160)
        plt.close(fig)
        generated.append(p.name)

    fig, ax = plt.subplots(figsize=(7, 6))
    for name, data in cases.items():
        local = data["local_wps"]
        ax.plot([w["x"] for w in local], [w["y"] for w in local], "-o", label=name, linewidth=2, markersize=4)
    ax.set_title("Local hairpin route windows")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend()
    p = out_dir / "combined_hairpin_comparison.png"
    fig.tight_layout()
    fig.savefig(p, dpi=160)
    plt.close(fig)
    generated.append(p.name)
    return generated


def write_summary(out_dir: Path, geom: dict[str, Any], exe: dict[str, Any], plot_names: list[str]) -> None:
    g24 = geom["00824_room15"]
    g43 = geom["00843_room13"]
    e24 = exe["00824_room15"]
    e43 = exe["00843_room13"]
    lines = [
        "# Task7 00824 vs 00843 Enter-And-Return Comparison",
        "",
        "## Evidence Source",
        "",
        "- 00824 was analyzed from existing evidence; no new 00824 run was attempted.",
        f"- 00824 run: `{DEFAULT_00824_RUN.relative_to(ROOT)}`.",
        f"- 00843 run: `{DEFAULT_00843_RUN.relative_to(ROOT)}`.",
        "- Stage-A was not rerun and no 00824 artifacts were modified.",
        "",
        "## Local Windows",
        "",
        "- 00824 room15 window is waypoint 49-69 from `semantic_route_waypoints_v0_1.json`. This window is identified by metadata `room_7 -> room_15`, `room15_interior_terminal` at waypoint 59, then `room_15 -> room_7` exit through the same gateway.",
        "- 00843 room13 window is waypoint 33-51 from `executable_route_waypoints_v0_1.json`. This window is identified by gateway_004 at waypoint 33, `room_center` at waypoint 39, then exit toward gateway_006 at waypoint 51.",
        "",
        "## Geometry",
        "",
        f"- 00824 local route has {g24['waypoint_count']} waypoints, length {g24['local_route_length_m']} m, max yaw change {g24['max_yaw_heading_change_deg']} deg, max geometric turn {g24['max_geometric_turn_deg']} deg, and min non-adjacent branch separation {g24['min_non_adjacent_branch_separation_m']} m.",
        f"- 00843 local route has {g43['waypoint_count']} waypoints, length {g43['local_route_length_m']} m, max yaw change {g43['max_yaw_heading_change_deg']} deg, max geometric turn {g43['max_geometric_turn_deg']} deg, and min non-adjacent branch separation {g43['min_non_adjacent_branch_separation_m']} m.",
        "- Both routes have near-overlapping non-adjacent branches and a semantic anchor at the fold, so hairpin geometry alone does not explain the different outcome.",
        "",
        "## Execution",
        "",
        f"- 00824 used `{e24['execution_strategy']}`. It sent FollowPath in two successful slices: the first ended at room15 waypoint 59, then the runner dwelled, then the second FollowPath started at waypoint 59 and exited. Sparse fallback was `{e24['sparse_fallback_used']}`.",
        f"- 00843 used `{e43['execution_strategy']}`. It sent one 59-pose FollowPath, which returned aborted; fallback then started at waypoint {e43['fallback_started_at_waypoint_index']} and the overall run succeeded by sparse fallback.",
        f"- 00824 split-aware active projection jumps >=4 indices: {e24['active_projection_jump_count_ge_4']}; full-route nearest jumps were {e24['full_route_projection_jump_count_ge_4']}, which is expected to be misleading on the folded route.",
        f"- 00843 active projection jumps >=4 indices: {e43['active_projection_jump_count_ge_4']}; the jump from pre-fold waypoint 35 to post-fold waypoint 43 occurred while the robot was still {e43['first_active_post_fold_projection_before_anchor_tolerance']['distance_to_fold_anchor_m'] if e43['first_active_post_fold_projection_before_anchor_tolerance'] else 'unknown'} m from room13 center.",
        f"- 00824 minimum trajectory distance to fold anchor: {e24['min_distance_to_fold_anchor_m']} m. 00843 minimum trajectory distance to room13 center: {e43['min_distance_to_fold_anchor_m']} m.",
        f"- 00824 max nearest-waypoint yaw error: {e24['max_yaw_error_deg']} deg. 00843 max nearest-waypoint yaw error: {e43['max_yaw_error_deg']} deg.",
        f"- 00824 spin-stall duration from available cmd_vel evidence: {e24['spin_stall_duration_sec']}; its independent local spin validator reported `spinning_detected={((e24.get('local_spin_validation') or {}).get('spinning_detected'))}` and `near_stationary_high_yaw_change_interval_count={((e24.get('local_spin_validation') or {}).get('near_stationary_high_yaw_change_interval_count'))}`. 00843 cmd_vel spin-stall duration was {e43['spin_stall_duration_sec']} sec.",
        "",
        "## Controller Logs",
        "",
        f"- 00824 Nav2 log flags: `{e24['log_flags']}`.",
        f"- 00843 Nav2 log flags: `{e43['log_flags']}`.",
        "",
        "## Answer",
        "",
        "The evidence-backed difference is execution semantics at the fold, not the mere presence of an enter-and-return shape. 00824 converted the room15 fold into a split FollowPath with an enforced semantic stop/dwell at the interior anchor, so the controller completed the inbound branch before receiving the outbound branch. 00843 presented the room13 fold as one continuous FollowPath; the robot projected onto the post-fold branch before the room13 center semantic anchor was completed, then the progress checker aborted with `Failed to make progress`. The fallback recovered afterward, but the FollowPath action failed.",
        "",
        "What remains unknown: the existing 00824 evidence does not include per-sample cmd_vel fields, so 00824 spin-stall behavior is not directly comparable from controller commands. Trajectory/yaw/projection metrics are comparable; command-level comparison is available for 00843 only.",
        "",
        "## Recommended Next Change",
        "",
        "Generalize the 00824 execution contract: split FollowPath at required semantic interior anchors or fold anchors, validate/dwell there with an ordered-progress guard, and only then send the exit slice. This should be implemented before tuning DWB because it addresses the proven projection/semantic-skip failure mode.",
    ]
    if plot_names:
        lines += ["", "## Plots", "", *[f"- `{name}`" for name in plot_names]]
    (out_dir / "task7_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--00824-run-dir", type=Path, default=DEFAULT_00824_RUN)
    parser.add_argument("--00843-run-dir", type=Path, default=DEFAULT_00843_RUN)
    parser.add_argument("--00843-route", type=Path, default=DEFAULT_00843_ROUTE)
    args = parser.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    route24 = waypoints_from(args.__dict__["00824_run_dir"] / "semantic_route_waypoints_v0_1.json")
    route43 = waypoints_from(args.__dict__["00843_route"])
    local24 = local_window(route24, 49, 69)
    local43 = local_window(route43, 33, 51)

    write_waypoint_window(out_dir / "00824_room15_local_waypoints.json", "00824_room15", local24)
    write_waypoint_window(out_dir / "00843_room13_local_waypoints.json", "00843_room13", local43)

    geom = {
        "00824_room15": geometry_metrics("00824_room15", local24, 59),
        "00843_room13": geometry_metrics("00843_room13", local43, 39),
    }
    (out_dir / "local_hairpin_geometry_metrics.json").write_text(json.dumps(geom, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (out_dir / "local_hairpin_geometry_metrics.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "name",
            "waypoint_count",
            "local_route_length_m",
            "spacing_min_m",
            "spacing_mean_m",
            "spacing_max_m",
            "max_yaw_heading_change_deg",
            "cumulative_yaw_heading_change_deg",
            "max_geometric_turn_deg",
            "cumulative_geometric_turn_deg",
            "heading_sign_reversals_y",
            "min_non_adjacent_branch_separation_m",
            "semantic_anchor_at_fold",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in geom.values():
            writer.writerow({k: row.get(k) for k in fields})

    run24 = args.__dict__["00824_run_dir"]
    run43 = args.__dict__["00843_run_dir"]
    exe = {
        "00824_room15": execution_metrics(
            "00824_room15",
            route24,
            run24 / "route_execution_result_v0_1.json",
            run24 / "trajectory_sample_result_v0_1.json",
            run24 / "bringup_logs/nav2.log",
            49,
            69,
            59,
            spin_validation_path=run24 / "local_looping_spin_validation_v0_2.json",
        ),
        "00843_room13": execution_metrics(
            "00843_room13",
            route43,
            run43 / "route_execution_result_v0_1.json",
            run43 / "trajectory_sample_result_v0_1.json",
            run43 / "bringup_logs/nav2.log",
            33,
            51,
            39,
            trace_path=run43 / "route_execution_trace_v0_1.jsonl",
            semantic_status_path=run43 / "semantic_goal_status_v0_1.json",
        ),
    }
    (out_dir / "local_execution_metrics.json").write_text(json.dumps(exe, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    traj24 = load_json(run24 / "trajectory_sample_result_v0_1.json").get("samples", [])
    traj43 = load_json(run43 / "trajectory_sample_result_v0_1.json").get("samples", [])
    plot_names = maybe_plot(
        out_dir,
        {
            "00824_room15": {"local_wps": local24, "trajectory": traj24},
            "00843_room13": {"local_wps": local43, "trajectory": traj43},
        },
    )

    findings = {
        "task_id": "task7_00824_00843_hairpin_compare",
        "stage_a_rerun": False,
        "modified_00824": False,
        "new_00824_run_attempted": False,
        "new_00824_run_id": None,
        "new_00824_run_succeeded": None,
        "00824_local_window_found": bool(local24),
        "00843_local_window_found": bool(local43),
        "00824_follow_path_succeeded": exe["00824_room15"]["follow_path_succeeded"],
        "00843_follow_path_succeeded": exe["00843_room13"]["follow_path_succeeded"],
        "00824_failed_to_make_progress": exe["00824_room15"]["log_flags"]["failed_to_make_progress"],
        "00843_failed_to_make_progress": exe["00843_room13"]["log_flags"]["failed_to_make_progress"],
        "00824_projection_jump_count": exe["00824_room15"]["active_projection_jump_count_ge_4"],
        "00843_projection_jump_count": exe["00843_room13"]["active_projection_jump_count_ge_4"],
        "00824_max_yaw_error_deg": exe["00824_room15"]["max_yaw_error_deg"],
        "00843_max_yaw_error_deg": exe["00843_room13"]["max_yaw_error_deg"],
        "00824_max_heading_change_deg": geom["00824_room15"]["max_yaw_heading_change_deg"],
        "00843_max_heading_change_deg": geom["00843_room13"]["max_yaw_heading_change_deg"],
        "00824_min_branch_separation_m": geom["00824_room15"]["min_non_adjacent_branch_separation_m"],
        "00843_min_branch_separation_m": geom["00843_room13"]["min_non_adjacent_branch_separation_m"],
        "00824_spin_stall_duration_sec": exe["00824_room15"]["spin_stall_duration_sec"],
        "00843_spin_stall_duration_sec": exe["00843_room13"]["spin_stall_duration_sec"],
        "main_difference_supported_by_evidence": "00824 split the enter-room15/exit-room15 fold into two successful FollowPath goals with dwell at waypoint 59; 00843 sent the folded room13 route as one FollowPath, projected past waypoint 39 without completing the room13 center, then aborted with Failed to make progress before fallback recovered.",
        "recommended_next_change": "Generalize split FollowPath plus required semantic-anchor dwell/validation for folded enter-and-return anchors before sending the outbound slice.",
    }
    (out_dir / "task7_findings.json").write_text(json.dumps(findings, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    write_summary(out_dir, geom, exe, plot_names)
    print(json.dumps({"out_dir": str(out_dir), "plots": plot_names}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
