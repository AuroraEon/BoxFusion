from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List

from stage_a_end_to_end_showcase import generate_end_to_end_showcase
from stage_a_execution_showcase import generate_execution_showcase
from stage_a_query_showcase import generate_query_showcase
from stage_a_showcase_common import (
    ArtifactSet,
    DEFAULT_OUTPUT_ROOT,
    DEFAULT_SEQUENCE_IDS,
    print_artifacts,
    write_json,
    write_text,
)
from stage_a_world_model_showcase import generate_world_model_showcase


def _index_markdown(bundle_dir: Path, artifacts: Dict[str, ArtifactSet], sequence_ids: List[str]) -> str:
    lines = [
        "# Teacher-Facing Showcase Bundle",
        "",
        "This bundle stays deliberately thin: it reuses the existing Stage-A world-model, Query API, closed-loop execution, and constrained NL/tool-use outputs and repackages them for advisor presentation.",
        "",
        "## What To Open First",
        "",
        f"- world model summary: `{bundle_dir / 'world_model' / 'world_model_showcase_v0_1.md'}`",
        f"- query showcase: `{bundle_dir / 'query_showcase' / 'query_showcase_v0_1.md'}`",
        f"- execution showcase: `{bundle_dir / 'execution_showcase' / 'execution_showcase_v0_1.md'}`",
        f"- end-to-end NL showcase: `{bundle_dir / 'end_to_end_showcase' / 'end_to_end_showcase_v0_1.md'}`",
        "",
        "## Suggested Story",
        "",
        f"- Start with sequences: {', '.join(sequence_ids)}",
        "- First show that the exported world model is floor-aware and already has explicit vertical_transition support.",
        "- Then show that the Query API can answer same-floor, cross-floor, anchor, and object-target cases.",
        "- Then show the room-level closed-loop executor following those symbolic routes and failing honestly when replay evidence is missing or divergent.",
        "- Finish with the constrained NL layer to make it clear that natural language only triggers existing backend tools instead of replacing them.",
        "",
        "## Honest Limitations To State Out Loud",
        "",
        "- Object duplication and merge quality are still upstream weaknesses.",
        "- Room semantics are weaker than floor structure, so explicit room ids are still the most demo-honest room targets.",
        "- Anchor/object requests resolve to rooms, not precise docking positions.",
        "- Cross-floor connectivity is still the minimal vertical_transition abstraction.",
        "- The NL layer is not an open agent; it is a constrained interpretation and tool-selection adapter.",
        "",
    ]
    return "\n".join(lines)


def generate_showcase_bundle(
    *,
    output_root: Path,
    sequence_ids: List[str],
    bundle_dir: Path,
) -> Dict[str, object]:
    bundle_dir.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "world_model": generate_world_model_showcase(
            output_root=output_root,
            sequence_ids=sequence_ids,
            showcase_dir=bundle_dir / "world_model",
        ),
        "query_showcase": generate_query_showcase(
            output_root=output_root,
            sequence_ids=sequence_ids,
            showcase_dir=bundle_dir / "query_showcase",
        ),
        "execution_showcase": generate_execution_showcase(
            output_root=output_root,
            sequence_ids=sequence_ids,
            showcase_dir=bundle_dir / "execution_showcase",
        ),
        "end_to_end_showcase": generate_end_to_end_showcase(
            output_root=output_root,
            sequence_ids=sequence_ids,
            showcase_dir=bundle_dir / "end_to_end_showcase",
        ),
    }

    summary = {
        "title": "Teacher-Facing Showcase Bundle v0.1",
        "output_root": str(output_root),
        "bundle_dir": str(bundle_dir),
        "sequence_ids": sequence_ids,
        "artifacts": {
            key: {
                "name": value.name,
                "generated_files": value.generated_files,
            }
            for key, value in artifacts.items()
        },
    }
    json_path = bundle_dir / "showcase_bundle_v0_1.json"
    md_path = bundle_dir / "advisor_showcase_index_v0_1.md"
    write_json(json_path, summary)
    write_text(md_path, _index_markdown(bundle_dir, artifacts, sequence_ids))
    print_artifacts(
        [json_path, md_path]
        + [Path(path) for artifact in artifacts.values() for path in artifact.generated_files]
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a compact teacher-facing Stage-A showcase bundle from existing exports.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory that already contains Stage-A and showcase source outputs.")
    parser.add_argument("--sequence", action="append", default=None, help="Sequence id to include. Can be passed multiple times.")
    parser.add_argument(
        "--bundle-dir",
        default=str(DEFAULT_OUTPUT_ROOT / "showcase_bundle_v0_1"),
        help="Output directory for the combined teacher-facing bundle.",
    )
    args = parser.parse_args()

    generate_showcase_bundle(
        output_root=Path(args.output_root),
        sequence_ids=list(args.sequence or DEFAULT_SEQUENCE_IDS),
        bundle_dir=Path(args.bundle_dir),
    )


if __name__ == "__main__":
    main()
