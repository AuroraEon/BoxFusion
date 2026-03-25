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


def _route_case_rows(case: Dict[str, Any], sequence_id: str) -> Dict[str, Any]:
    backend_map = {
        "same_floor_route": "RoomTopologyQueryAPI.query_route",
        "cross_floor_route": "RoomTopologyQueryAPI.query_route",
        "anchor_target": "RoomTopologyQueryAPI.query_route_to_anchor",
        "object_target": "RoomTopologyQueryAPI.query_route_to_object",
    }
    request_map = {
        "same_floor_route": "Structured: route from start_room_id to goal_room_id on the same floor.",
        "cross_floor_route": "Structured: route from start_room_id to goal_room_id across floors.",
        "anchor_target": "Structured: route from start_room_id to anchor_id.",
        "object_target": "Structured: route from start_room_id to object target.",
    }
    return {
        "sequence_id": sequence_id,
        "case_id": case.get("name"),
        "request_text": request_map.get(case.get("name"), "Structured Query API request."),
        "backend_call": backend_map.get(case.get("name")),
        "resolved_target": case.get("target_id") or case.get("resolved_goal_room_id"),
        "resolved_room_id": case.get("resolved_goal_room_id"),
        "floors_involved": f"{case.get('start_display_floor_id')} -> {case.get('resolved_goal_display_floor_id')}",
        "route_summary": case.get("route_summary"),
        "success": bool(case.get("pass")),
        "status": "success" if case.get("pass") else "failure",
        "explanation": case.get("explanation_summary") or case.get("route_summary"),
    }


def _failure_case_rows(sequence_id: str, tool_use_case: Dict[str, Any]) -> Dict[str, Any]:
    parsed = dict(tool_use_case.get("parsed_request") or {})
    return {
        "sequence_id": sequence_id,
        "case_id": tool_use_case.get("category"),
        "request_text": tool_use_case.get("request_text"),
        "backend_call": "none (target resolution stopped before Query API call)",
        "resolved_target": None,
        "resolved_room_id": None,
        "floors_involved": None,
        "route_summary": None,
        "success": False,
        "status": tool_use_case.get("status"),
        "explanation": (tool_use_case.get("teacher_response") or {}).get("summary")
        or parsed.get("unsupported_reason")
        or "Query was not dispatched because the request could not be resolved safely.",
    }


def _render_markdown(report: Dict[str, Any]) -> str:
    rows = []
    for item in report["cases"]:
        rows.append(
            [
                item["sequence_id"],
                item["case_id"],
                item["backend_call"],
                item["resolved_target"],
                item["floors_involved"],
                item["status"],
                item["route_summary"] or item["explanation"],
            ]
        )
    lines = [
        "# Query Showcase",
        "",
        "This teacher-facing report reuses Query API-produced acceptance results and adds one honest pre-routing limitation case per sequence.",
        "",
        markdown_table(
            ["Sequence", "Case", "Backend Call", "Resolved Target", "Floors", "Status", "Summary"],
            rows,
        ),
        "",
        "## Notes",
        "",
        "- Same-floor and cross-floor routing come directly from the existing multi-floor Query API acceptance export.",
        "- Anchor and object routing remain room-level destinations, consistent with the established architecture.",
        "- Ambiguous or unresolved cases are shown honestly as pre-routing failures rather than pretending the Query API answered them.",
        "",
    ]
    return "\n".join(lines)


def generate_query_showcase(
    *,
    output_root: Path,
    sequence_ids: List[str],
    showcase_dir: Path,
) -> ArtifactSet:
    acceptance_path = output_root / "multifloor_query_acceptance_v0_1" / "multifloor_query_acceptance_v0_1.json"
    tool_use_path = output_root / "vln_tool_use_v0_1" / "vln_tool_use_v0_1.json"
    acceptance = load_json(acceptance_path)
    tool_use = load_json(tool_use_path)

    cases: List[Dict[str, Any]] = []
    for sequence in acceptance.get("sequence_results", []):
        sequence_id = sequence.get("sequence_id")
        if sequence_id not in sequence_ids:
            continue
        for case in sequence.get("cases", []):
            cases.append(_route_case_rows(case, sequence_id))

    for sequence in tool_use.get("sequences", []):
        sequence_id = sequence.get("sequence_id")
        if sequence_id not in sequence_ids:
            continue
        ambiguous = next((case for case in sequence.get("cases", []) if case.get("category") == "ambiguous_target_request"), None)
        if ambiguous is not None:
            cases.append(_failure_case_rows(sequence_id, ambiguous))

    showcase_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "title": "Query Showcase v0.1",
        "output_root": str(output_root),
        "source_acceptance_json": str(acceptance_path),
        "source_tool_use_json": str(tool_use_path),
        "case_count": len(cases),
        "cases": cases,
    }
    json_path = showcase_dir / "query_showcase_v0_1.json"
    csv_path = showcase_dir / "query_showcase_v0_1.csv"
    md_path = showcase_dir / "query_showcase_v0_1.md"
    write_json(json_path, report)
    write_csv(
        csv_path,
        cases,
        [
            "sequence_id",
            "case_id",
            "request_text",
            "backend_call",
            "resolved_target",
            "resolved_room_id",
            "floors_involved",
            "route_summary",
            "status",
            "success",
            "explanation",
        ],
    )
    write_text(md_path, _render_markdown(report))
    print_artifacts([json_path, csv_path, md_path])
    return ArtifactSet(
        name="query_showcase",
        generated_files=[str(json_path), str(csv_path), str(md_path)],
        summary=report,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a teacher-facing query showcase from existing Query API acceptance outputs.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory that already contains Stage-A and acceptance outputs.")
    parser.add_argument("--sequence", action="append", default=None, help="Sequence id to include. Can be passed multiple times.")
    parser.add_argument(
        "--showcase-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "query_showcase_v0_1"),
        help="Output directory for teacher-facing query showcase artifacts.",
    )
    args = parser.parse_args()

    generate_query_showcase(
        output_root=Path(args.output_root),
        sequence_ids=list(args.sequence or DEFAULT_SEQUENCE_IDS),
        showcase_dir=Path(args.showcase_dir),
    )


if __name__ == "__main__":
    main()
