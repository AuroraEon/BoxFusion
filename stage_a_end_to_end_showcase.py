from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List

from stage_a_showcase_common import (
    ArtifactSet,
    DEFAULT_OUTPUT_ROOT,
    DEFAULT_SEQUENCE_IDS,
    load_json,
    markdown_table,
    print_artifacts,
    write_csv,
    write_json,
    write_text,
)


def _case_row(case: Dict[str, Any], trace_path: Path) -> Dict[str, Any]:
    selected_backend_call = dict(case.get("selected_backend_call") or {})
    resolved_target = dict(case.get("resolved_target") or {})
    route_summary = dict(case.get("route_summary") or {})
    final_outcome = dict(case.get("final_outcome") or {})
    return {
        "sequence_id": case.get("sequence_id"),
        "case_id": case.get("case_id"),
        "category": case.get("category"),
        "raw_nl_request": case.get("raw_request"),
        "interpreted_request": case.get("interpreted_request"),
        "selected_backend_tool": selected_backend_call.get("tool_name"),
        "resolved_target": resolved_target.get("selected_entity_id") or resolved_target.get("resolved_room_id"),
        "resolved_room_id": resolved_target.get("resolved_room_id"),
        "route_summary": route_summary.get("summary"),
        "execution_outcome": final_outcome.get("status"),
        "teacher_explanation": case.get("teacher_brief"),
        "limitations": case.get("limitations_note"),
        "trace_artifact_path": str(trace_path),
    }


def _render_markdown(report: Dict[str, Any]) -> str:
    rows = []
    for item in report["cases"]:
        rows.append(
            [
                item["sequence_id"],
                item["category"],
                item["selected_backend_tool"],
                item["resolved_target"],
                item["execution_outcome"],
                item["route_summary"] or item["teacher_explanation"],
            ]
        )
    lines = [
        "# End-to-End NL Showcase",
        "",
        "This teacher-facing report reuses the existing end-to-end NL orchestration export and keeps the LLM layer framed as constrained interpretation and tool selection only.",
        "",
        markdown_table(
            ["Sequence", "Case", "Tool", "Resolved Target", "Outcome", "Summary"],
            rows,
        ),
        "",
    ]
    for item in report["cases"]:
        lines.append(f"## {item['sequence_id']} / {item['category']}")
        lines.append("")
        lines.append(f"- raw request: `{item['raw_nl_request']}`")
        lines.append(f"- selected backend tool: `{item['selected_backend_tool']}`")
        lines.append(f"- resolved target: `{item['resolved_target']}`")
        lines.append(f"- route summary: {item['route_summary']}")
        lines.append(f"- outcome: `{item['execution_outcome']}`")
        lines.append(f"- teacher-facing explanation: {item['teacher_explanation']}")
        lines.append(f"- trace json: `{item['trace_artifact_path']}`")
        limitations = item.get("limitations") or []
        if limitations:
            lines.append("- limitations:")
            for limitation in limitations[:3]:
                lines.append(f"  - {limitation}")
        lines.append("")
    return "\n".join(lines)


def generate_end_to_end_showcase(
    *,
    output_root: Path,
    sequence_ids: List[str],
    showcase_dir: Path,
) -> ArtifactSet:
    source_path = output_root / "end_to_end_vln_demo_v0_1" / "end_to_end_vln_demo_v0_1.json"
    source = load_json(source_path)

    showcase_dir.mkdir(parents=True, exist_ok=True)
    traces_dir = showcase_dir / "traces"
    generated_paths: List[Path] = []
    cases: List[Dict[str, Any]] = []

    preferred = {
        "same_floor_room_explanation",
        "cross_floor_room_explanation",
        "same_floor_object_execution",
        "cross_floor_anchor_execution",
        "cross_floor_room_execution",
        "ambiguous_target_case",
        "unresolved_target_case",
        "unsupported_request_case",
    }
    for sequence in source.get("sequences", []):
        if sequence.get("sequence_id") not in sequence_ids:
            continue
        for case in sequence.get("cases", []):
            if case.get("category") not in preferred:
                continue
            trace_path = traces_dir / sequence["sequence_id"] / f"{case['case_id']}.json"
            write_json(trace_path, case)
            generated_paths.append(trace_path)
            cases.append(_case_row(case, trace_path))

    report = {
        "title": "End-to-End NL Showcase v0.1",
        "output_root": str(output_root),
        "source_json": str(source_path),
        "case_count": len(cases),
        "cases": cases,
    }
    json_path = showcase_dir / "end_to_end_showcase_v0_1.json"
    csv_path = showcase_dir / "end_to_end_showcase_v0_1.csv"
    md_path = showcase_dir / "end_to_end_showcase_v0_1.md"
    write_json(json_path, report)
    write_csv(
        csv_path,
        cases,
        [
            "sequence_id",
            "case_id",
            "category",
            "raw_nl_request",
            "selected_backend_tool",
            "resolved_target",
            "resolved_room_id",
            "route_summary",
            "execution_outcome",
            "trace_artifact_path",
        ],
    )
    write_text(md_path, _render_markdown(report))
    generated_paths.extend([json_path, csv_path, md_path])
    print_artifacts(generated_paths)
    return ArtifactSet(
        name="end_to_end_showcase",
        generated_files=[str(path) for path in generated_paths],
        summary=report,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a teacher-facing end-to-end NL showcase from existing demo outputs.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory that already contains Stage-A and NL demo outputs.")
    parser.add_argument("--sequence", action="append", default=None, help="Sequence id to include. Can be passed multiple times.")
    parser.add_argument(
        "--showcase-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "end_to_end_showcase_v0_1"),
        help="Output directory for teacher-facing end-to-end showcase artifacts.",
    )
    args = parser.parse_args()

    generate_end_to_end_showcase(
        output_root=Path(args.output_root),
        sequence_ids=list(args.sequence or DEFAULT_SEQUENCE_IDS),
        showcase_dir=Path(args.showcase_dir),
    )


if __name__ == "__main__":
    main()
