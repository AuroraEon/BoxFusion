from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.online_topology_timeline_eval import (
    build_timeline_summary,
    default_output_paths,
    load_json,
    render_timeline_markdown,
    resolve_lifecycle_artifact,
    write_json,
    write_markdown,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a compact debug-only room lifecycle timeline summary from an online-topology lifecycle artifact."
    )
    parser.add_argument(
        "input_path",
        help="Path to a scene root, logs/summary.json, or logs/online_topology_lifecycle_v0_1.json artifact.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional explicit JSON output path. Defaults beside the lifecycle artifact.",
    )
    parser.add_argument(
        "--md-out",
        type=Path,
        default=None,
        help="Optional explicit markdown output path. Defaults beside the lifecycle artifact.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lifecycle_path = resolve_lifecycle_artifact(Path(args.input_path))
    json_out, md_out = default_output_paths(lifecycle_path)
    if args.json_out is not None:
        json_out = Path(args.json_out)
    if args.md_out is not None:
        md_out = Path(args.md_out)

    payload = load_json(lifecycle_path)
    summary = build_timeline_summary(payload, source_path=lifecycle_path)
    markdown = render_timeline_markdown(summary)

    write_json(json_out, summary)
    write_markdown(md_out, markdown)

    print(f"Lifecycle artifact: {lifecycle_path}")
    print(f"Timeline JSON: {json_out}")
    print(f"Timeline markdown: {md_out}")


if __name__ == "__main__":
    main()
