from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.online_topology_timeline_eval import load_json
from boxfusion.publication_candidate_survivability import (
    build_publication_candidate_survivability,
    default_output_path,
    load_or_build_publication_policy_payload,
    resolve_input_artifacts,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a debug-only survivability evaluation for early-publication candidates over existing policy simulation artifacts."
    )
    parser.add_argument(
        "input_path",
        help=(
            "Path to a scene root, logs/summary.json, logs/online_topology_lifecycle_v0_1.json, "
            "logs/working_vs_committed_topology_timeline_v0_1.json, or logs/publication_policy_simulation_v0_1.json artifact."
        ),
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional explicit JSON output path. Defaults beside the publication-policy simulation artifact.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    artifacts = resolve_input_artifacts(Path(args.input_path))
    timeline_path = Path(artifacts["timeline_path"])
    lifecycle_path = Path(artifacts["lifecycle_path"])
    simulation_path = artifacts.get("publication_policy_simulation_path")
    output_anchor_path = timeline_path if simulation_path is None else Path(simulation_path)
    json_out = default_output_path(output_anchor_path) if args.json_out is None else Path(args.json_out)

    timeline_payload = load_json(timeline_path)
    lifecycle_payload = load_json(lifecycle_path)
    publication_policy_payload = load_or_build_publication_policy_payload(
        simulation_path=None if simulation_path is None else Path(simulation_path),
        timeline_path=timeline_path,
        lifecycle_path=lifecycle_path,
        topology_path=None if artifacts.get("topology_path") is None else Path(artifacts["topology_path"]),
    )
    summary = build_publication_candidate_survivability(
        publication_policy_payload,
        timeline_payload,
        lifecycle_payload,
        source_artifacts={
            "publication_policy_simulation_json": None if simulation_path is None else str(simulation_path),
            "working_vs_committed_topology_timeline_json": str(timeline_path),
            "online_topology_lifecycle_json": str(lifecycle_path),
            "topology_json": None if artifacts.get("topology_path") is None else str(artifacts["topology_path"]),
        },
    )
    write_json(json_out, summary)

    print(f"Working-vs-committed timeline artifact: {timeline_path}")
    print(f"Lifecycle artifact: {lifecycle_path}")
    if simulation_path is not None and Path(simulation_path).exists():
        print(f"Publication-policy simulation artifact: {simulation_path}")
    if artifacts.get("topology_path") is not None:
        print(f"Topology artifact: {artifacts['topology_path']}")
    print(f"Publication-candidate survivability JSON: {json_out}")


if __name__ == "__main__":
    main()
