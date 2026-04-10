from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.online_topology_timeline_eval import load_json
from boxfusion.publication_policy_simulation import (
    build_publication_policy_simulation,
    default_output_path,
    resolve_input_artifacts,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a debug-only publication-policy simulation over the working-vs-committed timeline."
    )
    parser.add_argument(
        "input_path",
        help="Path to a scene root, logs/summary.json, logs/working_vs_committed_topology_timeline_v0_1.json, or logs/online_topology_lifecycle_v0_1.json artifact.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional explicit JSON output path. Defaults beside the timeline artifact.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    artifacts = resolve_input_artifacts(Path(args.input_path))
    timeline_path = Path(artifacts["timeline_path"])
    lifecycle_path = Path(artifacts["lifecycle_path"])
    json_out = default_output_path(timeline_path) if args.json_out is None else Path(args.json_out)

    timeline_payload = load_json(timeline_path)
    lifecycle_payload = load_json(lifecycle_path)
    topology_payload = None
    if artifacts.get("topology_path") is not None:
        topology_payload = load_json(Path(artifacts["topology_path"]))

    summary = build_publication_policy_simulation(
        timeline_payload,
        lifecycle_payload,
        topology_payload=topology_payload,
        source_artifacts={
            "working_vs_committed_topology_timeline_json": str(timeline_path),
            "online_topology_lifecycle_json": str(lifecycle_path),
            "topology_json": None if artifacts.get("topology_path") is None else str(artifacts["topology_path"]),
            "working_vs_committed_topology_report_json": None
            if artifacts.get("working_vs_committed_report_path") is None
            else str(artifacts["working_vs_committed_report_path"]),
        },
    )
    write_json(json_out, summary)

    print(f"Working-vs-committed timeline artifact: {timeline_path}")
    print(f"Lifecycle artifact: {lifecycle_path}")
    if artifacts.get("topology_path") is not None:
        print(f"Topology artifact: {artifacts['topology_path']}")
    print(f"Publication-policy simulation JSON: {json_out}")


if __name__ == "__main__":
    main()
