#!/usr/bin/env python3
"""Evaluate object-nav queries as artifact self-consistency checks."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

from object_nav_common import TASK_DIR, load_index, load_json, run_query, write_csv, write_json


def hit_at(ranked: list[dict], expected: set[str], k: int) -> bool:
    return any(r.get("object_id") in expected for r in ranked[:k])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    parser.add_argument("--episodes", type=Path, default=TASK_DIR / "objectnav_experiment_episodes_v0_1.json")
    parser.add_argument("--query-results", type=Path, nargs="*", default=[])
    parser.add_argument("--output-json", type=Path, default=TASK_DIR / "objectnav_query_metrics_summary_v0_1.json")
    parser.add_argument("--output-csv", type=Path, default=TASK_DIR / "objectnav_query_metrics_summary.csv")
    parser.add_argument("--failure-json", type=Path, default=TASK_DIR / "objectnav_failure_breakdown_v0_1.json")
    parser.add_argument("--report", type=Path, default=TASK_DIR / "objectnav_experiment_report.md")
    args = parser.parse_args()
    index = load_index(args.index)
    episodes = load_json(args.episodes, {}).get("episodes", [])
    supplied = {load_json(p, {}).get("query_text"): load_json(p, {}) for p in args.query_results}
    rows = []
    failure_counts = Counter()
    ambiguous = noisy = uncertain = 0
    for ep in episodes:
        result = supplied.get(ep["query_text"]) or run_query(index, ep["query_text"], top_k=10)
        ranked = result.get("ranked_candidates", [])
        selected = result.get("selected_candidate")
        expected = set(ep.get("expected_object_ids") or [])
        found = bool(selected)
        top1 = bool(selected and selected.get("object_id") in expected)
        row = {
            "episode_id": ep["episode_id"],
            "query_text": ep["query_text"],
            "query_type": result.get("parsed_constraints", {}).get("query_type"),
            "candidate_found": found,
            "top1_hit": top1,
            "top3_hit": hit_at(ranked, expected, 3),
            "top5_hit": hit_at(ranked, expected, 5),
            "floor_constraint_satisfied": True,
            "room_constraint_satisfied": True,
            "object_room_floor_consistent": top1,
            "ambiguous_query": bool(result.get("ambiguous_query")),
            "failure_reason": result.get("failure_reason") or "",
            "runtime_candidate": ep.get("runtime_candidate"),
            "recommended_for_runtime": ep.get("recommended_for_runtime"),
        }
        if selected:
            constraints = result.get("parsed_constraints", {})
            if constraints.get("preferred_floor_id"):
                row["floor_constraint_satisfied"] = selected.get("floor_id") == constraints.get("preferred_floor_id")
            if constraints.get("preferred_room_id"):
                row["room_constraint_satisfied"] = selected.get("room_id") == constraints.get("preferred_room_id")
        if result.get("failure_reason"):
            failure_counts[result["failure_reason"]] += 1
        if result.get("ambiguous_query"):
            ambiguous += 1
        if "noisy label" in ep.get("reason", ""):
            noisy += 1
        if "uncertain floor assignment" in ep.get("reason", ""):
            uncertain += 1
        rows.append(row)
    n = max(len(rows), 1)
    metrics = {
        "version": "v0_1",
        "metric_scope": "artifact_self_consistency",
        "episode_count": len(rows),
        "query_parse_success_rate": sum(1 for r in rows if r["query_type"]) / n,
        "candidate_found_rate": sum(1 for r in rows if r["candidate_found"]) / n,
        "top1_hit_rate": sum(1 for r in rows if r["top1_hit"]) / n,
        "top3_hit_rate": sum(1 for r in rows if r["top3_hit"]) / n,
        "top5_hit_rate": sum(1 for r in rows if r["top5_hit"]) / n,
        "floor_constraint_satisfaction_rate": sum(1 for r in rows if r["floor_constraint_satisfied"]) / n,
        "room_constraint_satisfaction_rate": sum(1 for r in rows if r["room_constraint_satisfied"]) / n,
        "object_room_floor_consistency_rate": sum(1 for r in rows if r["object_room_floor_consistent"]) / n,
        "ambiguous_query_rate": ambiguous / n,
        "no_candidate_rate": sum(1 for r in rows if not r["candidate_found"]) / n,
        "noisy_label_case_count": noisy,
        "uncertain_floor_assignment_case_count": uncertain,
        "recommended_runtime_episode_count": sum(1 for r in rows if r["recommended_for_runtime"]),
    }
    fields = [
        "episode_id",
        "query_text",
        "query_type",
        "candidate_found",
        "top1_hit",
        "top3_hit",
        "top5_hit",
        "floor_constraint_satisfied",
        "room_constraint_satisfied",
        "object_room_floor_consistent",
        "ambiguous_query",
        "failure_reason",
        "runtime_candidate",
        "recommended_for_runtime",
    ]
    write_json(args.output_json, metrics)
    write_csv(args.output_csv, rows, fields)
    write_json(args.failure_json, {"metric_scope": "artifact_self_consistency", "failure_reason_counts": dict(failure_counts), "rows_with_failure": [r for r in rows if r["failure_reason"]]})
    args.report.write_text(
        "# ObjectNav Experiment Report\n\n"
        "Metrics are artifact self-consistency metrics, not semantic GT accuracy.\n\n"
        f"- Episodes evaluated: {len(rows)}\n"
        f"- Candidate found rate: {metrics['candidate_found_rate']:.3f}\n"
        f"- Top-1 artifact-derived retrieval hit rate: {metrics['top1_hit_rate']:.3f}\n"
        f"- Top-3 artifact-derived retrieval hit rate: {metrics['top3_hit_rate']:.3f}\n"
        f"- Ambiguous query rate: {metrics['ambiguous_query_rate']:.3f}\n"
        f"- No candidate rate: {metrics['no_candidate_rate']:.3f}\n"
        f"- Recommended later runtime episodes: {metrics['recommended_runtime_episode_count']}\n\n"
        "Failure reason counts:\n\n"
        + "\n".join(f"- {k}: {v}" for k, v in sorted(failure_counts.items()))
        + ("\n" if failure_counts else "- None\n")
    )
    print(f"wrote {args.output_json}")
    print(f"episode_count={len(rows)}")


if __name__ == "__main__":
    main()
