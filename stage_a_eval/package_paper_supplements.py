from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Sequence

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import dump_json, find_scene_root  # noqa: E402
from boxfusion.query_api import RoomTopologyQueryAPI  # noqa: E402


DEFAULT_SCENE_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_DOCS_ROOT = Path("world_model_backend_outputs_v0_2_final/docs")
DEFAULT_EVAL_ROOT = Path("world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented")
DEFAULT_SCENE_ROLE_SUMMARY = DEFAULT_DOCS_ROOT / "backend_eval_scene_role_summary.csv"
DEFAULT_RUNTIME_READOUT = DEFAULT_DOCS_ROOT / "backend_eval_runtime_readout.csv"
FOUR_SCENE_ORDER = ("00843", "00873", "00829", "00862")
FOUR_SCENE_SEQUENCE_NAMES = {
    "00843": "00843-DYehNKdT76V",
    "00873": "00873-bxsVRursffK",
    "00829": "00829-QaLdnwvtxbs",
    "00862": "00862-LT9Jq6dN3Ea",
}
CASE_STUDY_EXAMPLES = {
    "00843": {
        "kind": "room",
        "start_room_id": "room_2",
        "goal_room_id": "room_11",
        "route_policy": "balanced",
    },
    "00873": {
        "kind": "object",
        "start_room_id": "room_14",
        "object_label": "tree",
        "route_policy": "balanced",
    },
    "00829": {
        "kind": "room",
        "start_room_id": "room_3",
        "goal_room_id": "room_7",
        "route_policy": "balanced",
    },
    "00862": {
        "kind": "room",
        "start_room_id": "room_40",
        "goal_room_id": "room_3",
        "route_policy": "balanced",
    },
}


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def load_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def format_bytes_mb(value: Any) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "n/a"
    return f"{numeric / (1024.0 * 1024.0):.2f} MB"


def format_stage_window(summary: Dict[str, Any], metric: str, window: str) -> Any:
    return dict(dict(summary.get("stage_windows_sec") or {}).get(metric) or {}).get(f"{window}_avg_sec")


def scene_runtime_payload(scene_root: Path) -> Dict[str, Any]:
    manifest = load_json(scene_root / "manifest.json")
    runtime_profile = load_json(scene_root / "logs" / "runtime_growth_profile.json")
    return {
        "manifest": manifest,
        "runtime_profile": runtime_profile,
    }


def load_scene_metadata(scene_root: Path, scene_role_rows: Sequence[Dict[str, str]], runtime_rows: Sequence[Dict[str, str]]) -> Dict[str, Any]:
    payload = scene_runtime_payload(scene_root)
    manifest = payload["manifest"]
    runtime_profile = payload["runtime_profile"]
    scene_id = str(manifest.get("scene_id"))
    role_row = next((row for row in scene_role_rows if str(row.get("scene_id")) == scene_id), {})
    runtime_row = next((row for row in runtime_rows if str(row.get("scene_id")) == scene_id), {})
    return {
        "scene_root": scene_root,
        "sequence_name": scene_root.name,
        "scene_id": scene_id,
        "manifest": manifest,
        "runtime_profile": runtime_profile,
        "role_row": role_row,
        "runtime_row": runtime_row,
    }


def build_runtime_tables(docs_root: Path, scene_rows: Sequence[Dict[str, Any]]) -> None:
    paper_rows: List[Dict[str, Any]] = []
    highlight_rows: List[Dict[str, Any]] = []
    figure_rows: List[Dict[str, Any]] = []
    metrics = (
        ("total_step_sec", "total_step"),
        ("feature_boxfusion_sec", "feature_boxfusion"),
        ("topology_room_segmentation_sec", "topology_room_segmentation"),
    )

    for scene in scene_rows:
        manifest = dict(scene["manifest"])
        runtime_summary = dict(manifest.get("runtime_summary") or {})
        world_summary = dict(manifest.get("world_model_summary") or {})
        growth_summary = dict(dict(scene["runtime_profile"]).get("summary") or {})
        role_row = dict(scene["role_row"])
        row = {
            "scene_id": scene["scene_id"],
            "sequence_name": scene["sequence_name"],
            "retention_role": role_row.get("retention_role"),
            "floor_count": world_summary.get("floor_count"),
            "room_count": world_summary.get("room_count"),
            "node_count": world_summary.get("node_count"),
            "duration_sec": runtime_summary.get("duration_sec"),
            "average_fps": runtime_summary.get("average_fps"),
            "sample_count": growth_summary.get("sample_count"),
            "segmentation_refresh_sample_count": growth_summary.get("segmentation_refresh_sample_count"),
            "early_total_step_sec": format_stage_window(growth_summary, "total_step_sec", "early"),
            "mid_total_step_sec": format_stage_window(growth_summary, "total_step_sec", "mid"),
            "late_total_step_sec": format_stage_window(growth_summary, "total_step_sec", "late"),
            "total_step_growth_ratio": dict(growth_summary.get("growth_ratios") or {}).get("total_step_sec"),
            "early_feature_boxfusion_sec": format_stage_window(growth_summary, "feature_boxfusion_sec", "early"),
            "mid_feature_boxfusion_sec": format_stage_window(growth_summary, "feature_boxfusion_sec", "mid"),
            "late_feature_boxfusion_sec": format_stage_window(growth_summary, "feature_boxfusion_sec", "late"),
            "feature_boxfusion_growth_ratio": dict(growth_summary.get("growth_ratios") or {}).get("feature_boxfusion_sec"),
            "early_topology_room_segmentation_sec": format_stage_window(growth_summary, "topology_room_segmentation_sec", "early"),
            "mid_topology_room_segmentation_sec": format_stage_window(growth_summary, "topology_room_segmentation_sec", "mid"),
            "late_topology_room_segmentation_sec": format_stage_window(growth_summary, "topology_room_segmentation_sec", "late"),
            "topology_room_segmentation_growth_ratio": dict(growth_summary.get("growth_ratios") or {}).get("topology_room_segmentation_sec"),
            "tier1_backend_bytes": runtime_summary.get("backend_artifact_size_total_bytes"),
            "tier2_optional_bytes": runtime_summary.get("optional_demo_artifact_size_total_bytes"),
            "runtime_risk_flag": growth_summary.get("runtime_risk_flag"),
            "runtime_risk_reasons": "|".join(growth_summary.get("runtime_risk_reasons") or []),
        }
        paper_rows.append(row)

        if scene["scene_id"] in FOUR_SCENE_ORDER:
            highlight_rows.append(
                {
                    **row,
                    "scene_duration_min": round(float(runtime_summary.get("duration_sec") or 0.0) / 60.0, 2),
                    "tier1_backend_mb": format_bytes_mb(runtime_summary.get("backend_artifact_size_total_bytes")),
                    "tier2_optional_mb": format_bytes_mb(runtime_summary.get("optional_demo_artifact_size_total_bytes")),
                    "role_readout": role_row.get("readout"),
                }
            )
            for metric, metric_label in metrics:
                early_value = format_stage_window(growth_summary, metric, "early")
                for window_order, window in enumerate(("early", "mid", "late"), start=1):
                    value_sec = format_stage_window(growth_summary, metric, window)
                    relative_to_early = None
                    if early_value not in (None, 0):
                        relative_to_early = round(float(value_sec) / float(early_value), 3)
                    figure_rows.append(
                        {
                            "scene_id": scene["scene_id"],
                            "sequence_name": scene["sequence_name"],
                            "retention_role": role_row.get("retention_role"),
                            "metric": metric_label,
                            "window": window,
                            "window_order": window_order,
                            "value_sec": value_sec,
                            "relative_to_early": relative_to_early,
                        }
                    )

    paper_rows.sort(key=lambda item: item["scene_id"])
    highlight_rows.sort(key=lambda item: FOUR_SCENE_ORDER.index(str(item["scene_id"])))
    figure_rows.sort(key=lambda item: (FOUR_SCENE_ORDER.index(str(item["scene_id"])), str(item["metric"]), int(item["window_order"])))

    write_csv(
        docs_root / "runtime_growth_paper_table.csv",
        paper_rows,
        list(paper_rows[0].keys()) if paper_rows else [],
    )
    write_csv(
        docs_root / "runtime_growth_highlight_table.csv",
        highlight_rows,
        list(highlight_rows[0].keys()) if highlight_rows else [],
    )
    write_csv(
        docs_root / "runtime_growth_figure_data.csv",
        figure_rows,
        list(figure_rows[0].keys()) if figure_rows else [],
    )

    readout_lines = [
        "# Runtime Growth Readout",
        "",
        "## What Is Paper-Ready",
        "",
        "- `runtime_growth_paper_table.csv` consolidates the current canonical runtime-growth evidence for all retained scenes.",
        "- `runtime_growth_highlight_table.csv` narrows the main-text comparison to `00843`, `00873`, `00829`, and `00862`.",
        "- `runtime_growth_figure_data.csv` is an export-ready long-form bundle for a compact early/mid/late line figure with three panels: total step, feature fusion, and topology/room segmentation.",
        "- No new scenes were rerun. All values come from the retained `runtime_growth_profile.json/csv` files already present in `world_model_backend_outputs_v0_2_final/scenes/*/logs/`.",
        "",
        "## Main Readout",
        "",
        "- `00843` remains the cleanest backend anchor: best multi-floor FPS, lowest multi-floor total-step growth ratio, and moderate late-window cost.",
        "- `00873` is still presentation-strong but materially heavier than `00843`, especially in feature-fusion growth.",
        "- `00829` is the clean single-floor comparison: low absolute cost, one-floor topology, but still noticeable growth in late windows.",
        "- `00862` is the honest runtime-risk case: it dominates duration, late total-step cost, feature-fusion growth, and topology growth.",
        "",
        "## Intended Figure",
        "",
        "- Plot `runtime_growth_figure_data.csv` as a 3-panel early/mid/late line chart across the four highlighted scenes.",
        "- Panel 1: `metric=total_step` for overall per-step runtime growth.",
        "- Panel 2: `metric=feature_boxfusion` for semantic fusion growth pressure.",
        "- Panel 3: `metric=topology_room_segmentation` for topology / room-segmentation growth pressure.",
        "- Use scene-role color coding: `00843` backend anchor, `00873` showcase, `00829` clean single-floor, `00862` difficult/runtime-risk.",
    ]
    (docs_root / "runtime_growth_readout.md").write_text("\n".join(readout_lines) + "\n", encoding="utf-8")


def run_case_study_query(api: RoomTopologyQueryAPI, scene_id: str) -> Dict[str, Any]:
    spec = CASE_STUDY_EXAMPLES[scene_id]
    if spec["kind"] == "room":
        result = api.query_route(
            start_room_id=spec["start_room_id"],
            goal_room_id=spec["goal_room_id"],
            route_policy=spec["route_policy"],
        )
        target_resolution = dict(result.get("target_resolution") or {})
        route = dict(result.get("route") or {})
        query_text = (
            f"query_room_route(start_room_id={spec['start_room_id']!r}, "
            f"goal_room_id={spec['goal_room_id']!r}, route_policy={spec['route_policy']!r})"
        )
    else:
        result = api.query_route_to_object(
            start_room_id=spec["start_room_id"],
            object_label=spec["object_label"],
            route_policy=spec["route_policy"],
        )
        target_resolution = dict(result.get("target_resolution") or {})
        route = dict(result.get("route") or {})
        query_text = (
            f"query_object_route(start_room_id={spec['start_room_id']!r}, "
            f"object_label={spec['object_label']!r}, route_policy={spec['route_policy']!r})"
        )
    explanation = dict(result.get("explanation") or {})
    return {
        "query_text": query_text,
        "resolved_room_id": target_resolution.get("resolved_room_id"),
        "resolved_floor_id": target_resolution.get("resolved_display_floor_id") or target_resolution.get("resolved_floor_id"),
        "hop_count": route.get("hop_count"),
        "route_relations": "|".join(route.get("used_relation_types") or []),
        "floor_switch_count": len(explanation.get("floor_switches") or []),
        "route_summary": explanation.get("route_summary"),
    }


def build_case_study_assets(scene: Dict[str, Any]) -> List[Dict[str, Any]]:
    scene_root = Path(scene["scene_root"])
    scene_id = scene["scene_id"]
    sequence_name = scene["sequence_name"]
    assets: List[Dict[str, Any]] = []

    def add_asset(asset_role: str, relative_path: str, usage: str, note: str) -> None:
        path = scene_root / relative_path
        assets.append(
            {
                "scene_id": scene_id,
                "sequence_name": sequence_name,
                "retention_role": scene["role_row"].get("retention_role"),
                "asset_role": asset_role,
                "asset_path": str(path),
                "exists_in_canonical_package": path.exists(),
                "recommended_usage": usage,
                "caption_note": note,
            }
        )

    if scene_id in {"00843", "00873", "00829"}:
        add_asset("primary_figure", f"final/{sequence_name}_final_bev.png", "Main panel BEV map", "Use the final BEV as the scene-wide spatial anchor.")
        add_asset("inset", f"final/{sequence_name}_final_split.png", "Inset split / floor-aware panel", "Use the split view to expose room ids and floor separation.")
        add_asset("support", "report.md", "Teacher-facing qualitative notes", "Pull revisit and packaging notes from the retained report.")
        add_asset("support", "logs/vertical_transition_evidence.json", "Cross-floor caption evidence", "Reference transition ids and floor-pair labels directly from the canonical evidence.")
    else:
        add_asset("primary_figure", "logs/runtime_growth_profile.csv", "Primary runtime-risk chart source", "Use this as the main visual for 00862 instead of a polished montage.")
        add_asset("inset", "logs/vertical_transition_evidence.json", "Vertical transition inset", "Show the two retained vertical transitions and their floor pairs.")
        add_asset("inset", "logs/topology_v0_1.json", "Topology inset source", "Use room / edge counts and floor count as the difficult-case qualitative inset.")
        add_asset("support", "logs/summary.json", "Runtime-risk notes", "The summary still references legacy Tier 2 paths, but the canonical package only retains the core backend logs.")
    return assets


def build_case_study_packet(docs_root: Path, scene_rows: Sequence[Dict[str, Any]]) -> None:
    ordered_rows = sorted(scene_rows, key=lambda item: FOUR_SCENE_ORDER.index(str(item["scene_id"])))
    asset_rows: List[Dict[str, Any]] = []
    packet_lines = [
        "# Four-Scene Case Study Packet",
        "",
        "This packet keeps the current backend-centric framing intact. It packages the retained canonical artifacts for figure assembly and captions without claiming new science.",
        "",
    ]
    caption_lines = [
        "# Scene Case Study Captions Draft",
        "",
        "## Four-Panel Overview Caption",
        "",
        "The four-scene packet separates roles on purpose: `00843` is the clean backend anchor, `00873` is the richer qualitative showcase, `00829` is the clean single-floor control, and `00862` is the runtime-risk / difficult-case honesty panel rather than a polished montage.",
        "",
    ]

    for scene in ordered_rows:
        scene_root = Path(scene["scene_root"])
        scene_id = scene["scene_id"]
        role_row = dict(scene["role_row"])
        runtime_row = dict(scene["runtime_row"])
        api = RoomTopologyQueryAPI.from_json(scene_root / "logs" / "topology_v0_1.json")
        query_example = run_case_study_query(api, scene_id)
        assets = build_case_study_assets(scene)
        asset_rows.extend(assets)

        primary_assets = [item for item in assets if item["asset_role"] == "primary_figure"]
        inset_assets = [item for item in assets if item["asset_role"] == "inset"]
        packet_lines.extend(
            [
                f"## {scene_id} ({scene['sequence_name']})",
                "",
                f"- Role summary: {role_row.get('readout')}",
                f"- Recommended primary figure assets: {'; '.join(item['asset_path'] for item in primary_assets)}",
                f"- Recommended inset assets: {'; '.join(item['asset_path'] for item in inset_assets)}",
                f"- Concise route/query example: `{query_example['query_text']}` -> room `{query_example['resolved_room_id']}`, floor `{query_example['resolved_floor_id']}`, hops={query_example['hop_count']}, floor_switches={query_example['floor_switch_count']}, relations={query_example['route_relations']}.",
                f"- Caption-ready notes: {runtime_row.get('runtime_readout') or role_row.get('recommended_use')}",
                "",
            ]
        )

        caption_lines.extend(
            [
                f"## {scene_id}",
                "",
                f"`{scene['sequence_name']}` ({role_row.get('retention_role')}): {role_row.get('readout')} The recommended example trace is `{query_example['query_text']}`, which yields `{query_example['hop_count']}` hop(s) and `{query_example['floor_switch_count']}` floor switch(es).",
                "",
            ]
        )

    write_csv(
        docs_root / "scene_case_study_assets.csv",
        asset_rows,
        list(asset_rows[0].keys()) if asset_rows else [],
    )
    (docs_root / "scene_case_study_packet.md").write_text("\n".join(packet_lines) + "\n", encoding="utf-8")
    (docs_root / "scene_case_study_captions_draft.md").write_text("\n".join(caption_lines) + "\n", encoding="utf-8")


def build_augmented_probe_readout(docs_root: Path, eval_root: Path) -> None:
    aggregate = load_json(eval_root / "aggregate_summary.json")
    probe_rows = read_csv_rows(eval_root / "probe_slice_breakdown.csv")
    task_rows = read_csv_rows(eval_root / "task_results.csv")
    status_counts_by_slice: Dict[str, Counter[str]] = {}
    case_counts_by_slice: Dict[str, Counter[str]] = {}
    for row in task_rows:
        probe_slice = str(row.get("probe_slice") or "unknown")
        status_counts_by_slice.setdefault(probe_slice, Counter())[str(row.get("actual_status") or "unknown")] += 1
        case_counts_by_slice.setdefault(probe_slice, Counter())[str(row.get("probe_case_kind") or "unknown")] += 1

    lines = [
        "# Augmented Probe Readout",
        "",
        "## What Changed",
        "",
        "- The original seeded positive suite is preserved and tagged as `positive_seeded` rather than overwritten.",
        "- A moderate SUP-01 augmentation adds label-based duplicate-referent, alias, and near-miss probes on `00843`, `00873`, `00862`, plus the optional clean single-floor slice on `00829`.",
        "- The evaluator now accepts explicit `expected_statuses`, so ambiguity abstentions and hard negatives are scored against the expected backend status instead of being treated as generic failure only.",
        "",
        "## Slice Summary",
        "",
        f"- Overall augmented task count: {aggregate.get('task_count')}",
        f"- Overall task success rate: {aggregate.get('task_success_raw')} ({aggregate.get('task_success_rate')})",
        "",
    ]
    for row in probe_rows:
        slice_name = str(row.get("probe_slice"))
        status_histogram = dict(sorted(status_counts_by_slice.get(slice_name, Counter()).items()))
        lines.append(
            f"- `{slice_name}`: {row.get('task_success_raw')} success-matched tasks, expected-failure tasks={row.get('expected_failure_task_count')}, status histogram={status_histogram}."
        )
    lines.extend(
        [
            "",
            "## What The New Slice Proves",
            "",
            "- The benchmark no longer tests only exact-id or seeded-positive object lookups; it now explicitly checks whether the backend can abstain when duplicate labels span multiple rooms.",
            "- Alias probes show what the current normalized-label lookup can and cannot do. In `00843`, alias lookup can still resolve a same-room duplicate cleanly; in the denser scenes, the same alias often becomes ambiguous and is expected to abstain.",
            "- Hard negatives now check near-miss distractors such as slash/space label confusions, which the old benchmark did not exercise at all.",
            "",
            "## What It Still Does Not Prove",
            "",
            "- This is still not open-vocabulary retrieval or a broad external benchmark.",
            "- The augmentation pressures label ambiguity and near-miss rejection, but it does not solve object identity / merge quality.",
            "- The suite remains backend-centric and room-centric: the evidence is about grounding to the correct room or abstaining honestly, not about end-to-end embodied agent performance.",
            "",
            "## Probe Mix",
            "",
        ]
    )
    for slice_name, counts in sorted(case_counts_by_slice.items()):
        lines.append(f"- `{slice_name}` case mix: {dict(sorted(counts.items()))}")
    (docs_root / "augmented_probe_readout.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_claim_boundary_note(docs_root: Path) -> None:
    lines = [
        "# Claim Boundary Note",
        "",
        "## Supported Now",
        "",
        "- The current canonical package supports a backend-centric claim that the retained floor-aware, room-centric world model is queryable, explainable, and executable on its own retained scenes.",
        "- The augmented SUP-01 slice now also supports a modest claim about ambiguity abstention and near-miss rejection under the existing label-based object lookup path.",
        "- The runtime-growth package supports a practical-but-bounded claim: query-time use is cheap, while scene construction cost grows substantially in the harder scenes.",
        "",
        "## Weakly Supported",
        "",
        "- Alias robustness is only weakly supported and should be framed as a bounded normalization behavior rather than broad semantic grounding.",
        "- Object-level identity consistency remains weakly supported because these probes still operate at room-resolution outcomes.",
        "- Qualitative case-study storytelling is strong, but it should still be presented as packaging of current evidence rather than a new experiment.",
        "",
        "## Deferred",
        "",
        "- Broad open-ended natural-language navigation or agentic planning claims should remain deferred.",
        "- Strong object identity / merge claims should remain deferred pending a dedicated identity-sensitive probe suite.",
        "- Large-scale scalability claims beyond the retained scenes should remain deferred because `00862` still reads as a clear runtime-risk outlier.",
    ]
    (docs_root / "claim_boundary_note.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Package the high-value paper supplements from canonical artifacts.")
    parser.add_argument(
        "--scene-root",
        default=str(DEFAULT_SCENE_ROOT),
        help="Canonical scene root.",
    )
    parser.add_argument(
        "--docs-root",
        default=str(DEFAULT_DOCS_ROOT),
        help="Docs output root.",
    )
    parser.add_argument(
        "--eval-root",
        default=str(DEFAULT_EVAL_ROOT),
        help="Augmented evaluator output root.",
    )
    parser.add_argument(
        "--scene-role-summary",
        default=str(DEFAULT_SCENE_ROLE_SUMMARY),
        help="Existing retained-scene role summary CSV.",
    )
    parser.add_argument(
        "--runtime-readout",
        default=str(DEFAULT_RUNTIME_READOUT),
        help="Existing runtime readout CSV.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    scene_role_rows = read_csv_rows(Path(args.scene_role_summary))
    runtime_rows = read_csv_rows(Path(args.runtime_readout))
    all_scene_rows: List[Dict[str, Any]] = []
    for row in sorted(scene_role_rows, key=lambda item: str(item.get("scene_id"))):
        sequence_name = str(row.get("sequence_name") or "").strip()
        if not sequence_name:
            continue
        scene_root = find_scene_root(sequence_name, preferred_root=Path(args.scene_root), fallback_roots=[])
        if scene_root is None:
            raise FileNotFoundError(f"Scene root not found for {sequence_name}")
        all_scene_rows.append(load_scene_metadata(scene_root, scene_role_rows, runtime_rows))

    focus_scene_rows = [
        row
        for row in all_scene_rows
        if str(row["scene_id"]) in FOUR_SCENE_ORDER
    ]

    docs_root = Path(args.docs_root)
    docs_root.mkdir(parents=True, exist_ok=True)

    build_runtime_tables(docs_root, all_scene_rows)
    build_case_study_packet(docs_root, focus_scene_rows)
    build_augmented_probe_readout(docs_root, Path(args.eval_root))
    build_claim_boundary_note(docs_root)

    dump_json(
        docs_root / "paper_supplement_pack_manifest.json",
        {
            "runtime_outputs": [
                str(docs_root / "runtime_growth_paper_table.csv"),
                str(docs_root / "runtime_growth_highlight_table.csv"),
                str(docs_root / "runtime_growth_figure_data.csv"),
                str(docs_root / "runtime_growth_readout.md"),
            ],
            "probe_outputs": [
                str(docs_root / "augmented_probe_readout.md"),
            ],
            "case_study_outputs": [
                str(docs_root / "scene_case_study_packet.md"),
                str(docs_root / "scene_case_study_assets.csv"),
                str(docs_root / "scene_case_study_captions_draft.md"),
            ],
            "honesty_outputs": [
                str(docs_root / "claim_boundary_note.md"),
            ],
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
