from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List

from stage_a_showcase_common import (
    ArtifactSet,
    DEFAULT_OUTPUT_ROOT,
    DEFAULT_SEQUENCE_IDS,
    build_contact_sheet,
    copy_if_exists,
    counts_by_floor,
    final_room_panel_paths,
    floor_display_label,
    ordered_floors,
    print_artifacts,
    sequence_assets,
    trim_markdown,
    write_json,
    write_text,
)


def _world_model_sequence_summary(output_root: Path, sequence_id: str, sequence_out: Path) -> Dict[str, Any]:
    assets = sequence_assets(output_root, sequence_id)
    topology = assets["topology"]
    summary = assets["summary"]
    floor_diag = assets["floor_diag"]
    vertical = assets["vertical"]
    counts = counts_by_floor(topology)
    floor_panels = final_room_panel_paths(assets["root"])

    copied_bev = copy_if_exists(
        Path(summary["final_map_png"]),
        sequence_out / f"{sequence_id}_final_bev.png",
    )
    copied_split = copy_if_exists(
        Path(summary["final_split_png"]),
        sequence_out / f"{sequence_id}_final_split.png",
    )

    panel_specs = []
    floor_rows: List[Dict[str, Any]] = []
    for floor in ordered_floors(topology):
        floor_id = str(floor.get("floor_id"))
        label = floor_display_label(floor)
        counts_row = counts.get(floor_id, {"rooms": 0, "objects": 0, "anchors": 0})
        note = (
            f"internal={floor_id} | rooms={counts_row['rooms']} | "
            f"objects={counts_row['objects']} | anchors={counts_row['anchors']}"
        )
        panel_path = floor_panels.get(floor_id)
        if panel_path is not None:
            panel_specs.append((f"{label}", note, panel_path))
        floor_rows.append(
            {
                "floor_id": floor_id,
                "display_floor_id": floor.get("display_floor_id"),
                "display_order": floor.get("display_order"),
                "z_min": floor.get("z_min"),
                "z_max": floor.get("z_max"),
                "z_center": floor.get("z_center"),
                "room_count": counts_row["rooms"],
                "object_count": counts_row["objects"],
                "anchor_count": counts_row["anchors"],
                "panel_path": None if panel_path is None else str(panel_path),
            }
        )

    panel_out = None
    if panel_specs:
        panel_out = build_contact_sheet(
            title=f"{sequence_id} Multi-floor World Model",
            subtitle="Small multiples reuse the latest per-floor room-segmentation panels from debug_room exports.",
            panels=panel_specs,
            output_path=sequence_out / f"{sequence_id}_multifloor_small_multiples.png",
            columns=2 if len(panel_specs) > 1 else 1,
        )

    vertical_summary = dict(vertical.get("summary") or {})
    transitions = list(vertical_summary.get("transitions") or vertical.get("transitions") or [])
    result = {
        "sequence_id": sequence_id,
        "floor_count": len(ordered_floors(topology)),
        "room_count": len(topology.get("rooms", [])),
        "object_count": len((topology.get("entities") or {}).get("objects", [])),
        "anchor_count": len((topology.get("entities") or {}).get("anchors", [])),
        "floor_rows": floor_rows,
        "vertical_transition_summary": {
            "count": vertical_summary.get("count", len(transitions)),
            "edge_eligible_count": vertical_summary.get("edge_eligible_count"),
            "connector_label_counts": vertical_summary.get("connector_label_counts"),
            "transitions": transitions,
        },
        "teacher_notes": {
            "what_to_look_at": [
                "The final BEV gives the whole-house floor-aware layout.",
                "The small multiples show each exported floor separately in display order, which is especially helpful on 00847 where internal floor ids and display order differ.",
                "Vertical transitions summarize the minimal explicit cross-floor links used later by routing and execution.",
            ],
            "presentation_note_excerpt": trim_markdown(assets["presentation_note"], limit=420),
            "limitations": [
                "Broad room semantics remain sparse, so floor structure is stronger than room-type naming.",
                "Vertical connectivity is still the minimal vertical_transition abstraction, not a full stair/landing model.",
                "Object counts may still reflect upstream duplication/merge weakness.",
            ],
        },
        "reused_artifacts": {
            "source_final_bev": summary.get("final_map_png"),
            "source_final_split": summary.get("final_split_png"),
            "source_floor_diagnostics": str(assets["logs_dir"] / "floor_diagnostics_summary.json"),
            "source_vertical_transition_evidence": str(assets["logs_dir"] / "vertical_transition_evidence.json"),
            "copied_final_bev": copied_bev,
            "copied_final_split": copied_split,
            "multifloor_small_multiples_png": None if panel_out is None else str(panel_out),
        },
    }
    return result


def _render_markdown(report: Dict[str, Any]) -> str:
    lines = [
        "# Multi-floor World-Model Showcase",
        "",
        "This advisor-facing layer reuses the existing Stage-A exports and highlights the floor-aware world model without changing backend semantics.",
        "",
    ]
    for sequence in report["sequences"]:
        lines.append(f"## {sequence['sequence_id']}")
        lines.append("")
        lines.append(
            f"- overview: floors={sequence['floor_count']}, rooms={sequence['room_count']}, "
            f"objects={sequence['object_count']}, anchors={sequence['anchor_count']}"
        )
        lines.append(
            f"- vertical transitions: count={sequence['vertical_transition_summary']['count']}, "
            f"edge_eligible={sequence['vertical_transition_summary'].get('edge_eligible_count')}, "
            f"connector_labels={sequence['vertical_transition_summary'].get('connector_label_counts')}"
        )
        lines.append("- floor ordering and per-floor counts:")
        for row in sequence["floor_rows"]:
            lines.append(
                f"  - {row['display_floor_id']} (internal {row['floor_id']}): "
                f"rooms={row['room_count']}, objects={row['object_count']}, anchors={row['anchor_count']}, "
                f"z_center={row['z_center']}"
            )
        transitions = sequence["vertical_transition_summary"]["transitions"]
        if transitions:
            lines.append("- vertical-transition highlights:")
            for item in transitions[:3]:
                lines.append(
                    f"  - {item.get('transition_id')}: {item.get('from_display_floor_id')}:{item.get('from_room_label')} "
                    f"-> {item.get('to_display_floor_id')}:{item.get('to_room_label')} | "
                    f"{item.get('connector_label')} | frames {item.get('transition_frame_start')}-{item.get('transition_frame_end')}"
                )
        lines.append("- what the figures mean:")
        for item in sequence["teacher_notes"]["what_to_look_at"]:
            lines.append(f"  - {item}")
        lines.append(f"- presentation-note excerpt: {sequence['teacher_notes']['presentation_note_excerpt']}")
        lines.append("- known limitations:")
        for item in sequence["teacher_notes"]["limitations"]:
            lines.append(f"  - {item}")
        lines.append("")
    return "\n".join(lines).strip() + "\n"


def generate_world_model_showcase(
    *,
    output_root: Path,
    sequence_ids: List[str],
    showcase_dir: Path,
) -> ArtifactSet:
    showcase_dir.mkdir(parents=True, exist_ok=True)
    sequence_payloads = []
    generated_paths: List[Path] = []
    for sequence_id in sequence_ids:
        sequence_out = showcase_dir / sequence_id
        sequence_out.mkdir(parents=True, exist_ok=True)
        payload = _world_model_sequence_summary(output_root, sequence_id, sequence_out)
        sequence_payloads.append(payload)
        reused = payload["reused_artifacts"]
        for key in ["copied_final_bev", "copied_final_split", "multifloor_small_multiples_png"]:
            if reused.get(key):
                generated_paths.append(Path(reused[key]))

    report = {
        "title": "Multi-floor World-Model Showcase v0.1",
        "output_root": str(output_root),
        "showcase_dir": str(showcase_dir),
        "sequences": sequence_payloads,
    }
    json_path = showcase_dir / "world_model_showcase_v0_1.json"
    md_path = showcase_dir / "world_model_showcase_v0_1.md"
    write_json(json_path, report)
    write_text(md_path, _render_markdown(report))
    generated_paths.extend([json_path, md_path])
    print_artifacts(generated_paths)
    return ArtifactSet(
        name="world_model",
        generated_files=[str(path) for path in generated_paths],
        summary=report,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a teacher-facing multi-floor world-model showcase from existing Stage-A exports.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory that already contains Stage-A sequence outputs.")
    parser.add_argument("--sequence", action="append", default=None, help="Sequence id to include. Can be passed multiple times.")
    parser.add_argument(
        "--showcase-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "world_model_showcase_v0_1"),
        help="Output directory for teacher-facing world-model artifacts.",
    )
    args = parser.parse_args()

    generate_world_model_showcase(
        output_root=Path(args.output_root),
        sequence_ids=list(args.sequence or DEFAULT_SEQUENCE_IDS),
        showcase_dir=Path(args.showcase_dir),
    )


if __name__ == "__main__":
    main()
