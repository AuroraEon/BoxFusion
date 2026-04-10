from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.online_topology_blocker_diag import (
    build_blocker_diagnosis,
    default_output_paths,
    load_scene_context,
    render_blocker_diag_markdown,
    write_json,
    write_markdown,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a compact debug-only blocker diagnosis report from one or more online-topology scene artifacts."
    )
    parser.add_argument(
        "input_paths",
        nargs="+",
        help="One or more scene roots, logs/summary.json files, lifecycle artifacts, or timeline-eval artifacts.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional explicit JSON output path. Defaults beside the single input scene, or in the current working directory for multi-scene runs.",
    )
    parser.add_argument(
        "--md-out",
        type=Path,
        default=None,
        help="Optional explicit markdown output path. Defaults beside the single input scene, or in the current working directory for multi-scene runs.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene_contexts = [load_scene_context(Path(input_path)) for input_path in list(args.input_paths)]
    json_out, md_out = default_output_paths(scene_contexts)
    if args.json_out is not None:
        json_out = Path(args.json_out)
    if args.md_out is not None:
        md_out = Path(args.md_out)

    report = build_blocker_diagnosis(scene_contexts)
    markdown = render_blocker_diag_markdown(report)

    write_json(json_out, report)
    write_markdown(md_out, markdown)

    print(f"Scenes: {len(scene_contexts)}")
    for scene_context in scene_contexts:
        print(
            "  - {scene_id}: timeline={timeline} lifecycle={lifecycle}".format(
                scene_id=scene_context.get("scene_id"),
                timeline=scene_context.get("timeline_path") or "derived",
                lifecycle=scene_context.get("lifecycle_path") or "n/a",
            )
        )
    print(f"Blocker diagnosis JSON: {json_out}")
    print(f"Blocker diagnosis markdown: {md_out}")


if __name__ == "__main__":
    main()
