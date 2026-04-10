from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.working_vs_committed_topology_timeline import (
    build_working_vs_committed_timeline,
    default_output_paths,
    load_optional_json,
    render_working_vs_committed_timeline_markdown,
    resolve_input_artifacts,
    write_json,
    write_markdown,
)
from boxfusion.online_topology_timeline_eval import load_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a debug-only temporal working-vs-committed topology timeline from lifecycle refresh history."
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
    artifacts = resolve_input_artifacts(Path(args.input_path))
    lifecycle_path = Path(artifacts["lifecycle_path"])
    json_out, md_out = default_output_paths(lifecycle_path)
    if args.json_out is not None:
        json_out = Path(args.json_out)
    if args.md_out is not None:
        md_out = Path(args.md_out)

    lifecycle_payload = load_json(lifecycle_path)
    comparison_payload = load_optional_json(artifacts.get("working_vs_committed_report_path"))
    summary = build_working_vs_committed_timeline(
        lifecycle_payload,
        source_path=lifecycle_path,
        source_artifacts={
            "working_vs_committed_topology_report_json": None
            if artifacts.get("working_vs_committed_report_path") is None
            else str(artifacts["working_vs_committed_report_path"]),
            "working_topology_json": None
            if artifacts.get("working_topology_path") is None
            else str(artifacts["working_topology_path"]),
            "topology_json": None
            if artifacts.get("topology_path") is None
            else str(artifacts["topology_path"]),
        },
        final_comparison_payload=comparison_payload,
    )
    markdown = render_working_vs_committed_timeline_markdown(summary)

    write_json(json_out, summary)
    write_markdown(md_out, markdown)

    print(f"Lifecycle artifact: {lifecycle_path}")
    if artifacts.get("working_vs_committed_report_path") is not None:
        print(f"Static comparison artifact: {artifacts['working_vs_committed_report_path']}")
    print(f"Working-vs-committed timeline JSON: {json_out}")
    print(f"Working-vs-committed timeline markdown: {md_out}")


if __name__ == "__main__":
    main()
