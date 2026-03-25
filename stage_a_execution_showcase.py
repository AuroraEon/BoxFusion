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


def _step_summary(case: Dict[str, Any]) -> List[str]:
    steps = list((((case.get("execution") or {}).get("plan") or {}).get("symbolic_plan") or {}).get("steps") or [])
    return [str(step.get("description")) for step in steps]


def _trace_summary(case: Dict[str, Any], limit: int = 6) -> List[str]:
    events = list((case.get("execution") or {}).get("execution_trace") or [])[:limit]
    lines: List[str] = []
    for event in events:
        kind = event.get("event_type")
        if kind == "room_observed":
            lines.append(
                f"frame {event.get('frame_idx')}: observe {event.get('observed_room_id')} on {event.get('observed_display_floor_id') or event.get('observed_floor_id')}"
            )
        elif kind == "floor_switch_observed":
            lines.append(
                f"frame {event.get('frame_idx')}: floor switch {event.get('from_room_id')} -> {event.get('to_room_id')} "
                f"({event.get('from_display_floor_id')} -> {event.get('to_display_floor_id')})"
            )
        elif kind == "step_completed":
            lines.append(f"frame {event.get('frame_idx')}: complete {event.get('description')}")
        elif kind == "execution_divergence":
            lines.append(
                f"frame {event.get('frame_idx')}: divergence after observing {event.get('observed_room_id')}, expected {event.get('expected_room_id')}"
            )
    return lines


def _target_label(case: Dict[str, Any]) -> str:
    target = dict((case.get("task_input") or {}).get("target") or {})
    target_type = target.get("target_type")
    if target_type == "room":
        return str(target.get("goal_room_id"))
    if target_type == "anchor":
        return str(target.get("anchor_id"))
    if target_type == "object":
        return str(target.get("object_label") or target.get("object_id"))
    return "unknown_target"


def _case_row(case: Dict[str, Any], sequence_id: str, trace_path: Path) -> Dict[str, Any]:
    execution = dict(case.get("execution") or {})
    plan = dict(execution.get("plan") or {})
    outcome = dict(execution.get("outcome") or {})
    floor_switches = list(plan.get("floor_switches") or [])
    route_summary = plan.get("route_summary")
    trace_lines = _trace_summary(case)
    return {
        "sequence_id": sequence_id,
        "case_id": case.get("case_id"),
        "case_kind": case.get("case_kind"),
        "start": f"{case.get('start_room_id')} @ {case.get('start_display_floor_id')}",
        "target": _target_label(case),
        "planned_route_summary": route_summary,
        "step_summary": _step_summary(case),
        "execution_trace_summary": trace_lines,
        "floor_switch_summary": (
            "none"
            if not floor_switches
            else "; ".join(
                f"{item.get('source_room_id')} -> {item.get('target_room_id')} "
                f"({item.get('from_display_floor_id')} -> {item.get('to_display_floor_id')})"
                for item in floor_switches
            )
        ),
        "final_outcome": outcome.get("outcome_category"),
        "final_reason": outcome.get("outcome_reason"),
        "trace_artifact_path": str(trace_path),
    }


def _render_markdown(report: Dict[str, Any]) -> str:
    rows = []
    for item in report["cases"]:
        rows.append(
            [
                item["sequence_id"],
                item["case_kind"],
                item["start"],
                item["target"],
                item["final_outcome"],
                item["final_reason"],
                item["planned_route_summary"],
            ]
        )
    lines = [
        "# Execution Showcase",
        "",
        "This report reuses the existing minimal closed-loop execution export and rewrites it into a compact teacher-facing format.",
        "",
        markdown_table(
            ["Sequence", "Case", "Start", "Target", "Outcome", "Reason", "Planned Route"],
            rows,
        ),
        "",
    ]
    for item in report["cases"]:
        lines.append(f"## {item['sequence_id']} / {item['case_kind']}")
        lines.append("")
        lines.append(f"- start: {item['start']}")
        lines.append(f"- target: {item['target']}")
        lines.append(f"- planned route: {item['planned_route_summary']}")
        lines.append(f"- floor switches: {item['floor_switch_summary']}")
        lines.append(f"- outcome: {item['final_outcome']} / {item['final_reason']}")
        lines.append(f"- trace json: `{item['trace_artifact_path']}`")
        lines.append("- symbolic steps:")
        for step in item["step_summary"]:
            lines.append(f"  - {step}")
        lines.append("- compact execution trace:")
        for line in item["execution_trace_summary"]:
            lines.append(f"  - {line}")
        lines.append("")
    return "\n".join(lines)


def generate_execution_showcase(
    *,
    output_root: Path,
    sequence_ids: List[str],
    showcase_dir: Path,
) -> ArtifactSet:
    source_path = output_root / "minimal_vln_closed_loop_v0_1" / "minimal_vln_closed_loop_v0_1.json"
    source = load_json(source_path)

    showcase_dir.mkdir(parents=True, exist_ok=True)
    traces_dir = showcase_dir / "traces"
    cases: List[Dict[str, Any]] = []
    generated_paths: List[Path] = []

    for sequence in source.get("sequences", []):
        sequence_id = sequence.get("sequence_id")
        if sequence_id not in sequence_ids:
            continue
        chosen_cases = []
        chosen_cases.extend(
            case
            for case in sequence.get("success_cases", [])
            if case.get("case_kind") in {"room_same_floor", "room_cross_floor", "anchor_target", "object_target"}
        )
        chosen_cases.extend(
            case
            for case in sequence.get("honesty_probes", [])
            if case.get("case_kind") in {"missing_vertical_transition_probe", "execution_divergence_probe"}
        )
        for case in chosen_cases:
            trace_path = traces_dir / sequence_id / f"{case['case_id']}.json"
            write_json(trace_path, case)
            generated_paths.append(trace_path)
            cases.append(_case_row(case, sequence_id, trace_path))

    report = {
        "title": "Execution Showcase v0.1",
        "output_root": str(output_root),
        "source_json": str(source_path),
        "case_count": len(cases),
        "cases": cases,
    }
    json_path = showcase_dir / "execution_showcase_v0_1.json"
    csv_path = showcase_dir / "execution_showcase_v0_1.csv"
    md_path = showcase_dir / "execution_showcase_v0_1.md"
    write_json(json_path, report)
    write_csv(
        csv_path,
        cases,
        [
            "sequence_id",
            "case_id",
            "case_kind",
            "start",
            "target",
            "planned_route_summary",
            "floor_switch_summary",
            "final_outcome",
            "final_reason",
            "trace_artifact_path",
        ],
    )
    write_text(md_path, _render_markdown(report))
    generated_paths.extend([json_path, csv_path, md_path])
    print_artifacts(generated_paths)
    return ArtifactSet(
        name="execution_showcase",
        generated_files=[str(path) for path in generated_paths],
        summary=report,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a teacher-facing execution showcase from existing closed-loop execution outputs.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory that already contains Stage-A and execution outputs.")
    parser.add_argument("--sequence", action="append", default=None, help="Sequence id to include. Can be passed multiple times.")
    parser.add_argument(
        "--showcase-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "execution_showcase_v0_1"),
        help="Output directory for teacher-facing execution showcase artifacts.",
    )
    args = parser.parse_args()

    generate_execution_showcase(
        output_root=Path(args.output_root),
        sequence_ids=list(args.sequence or DEFAULT_SEQUENCE_IDS),
        showcase_dir=Path(args.showcase_dir),
    )


if __name__ == "__main__":
    main()
