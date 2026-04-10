from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.online_topology_working_snapshot import (
    build_working_topology_snapshot,
    build_working_vs_committed_report,
    default_output_paths,
    load_json,
    resolve_topology_and_lifecycle_artifacts,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build debug-only working topology and working-vs-committed comparison artifacts."
    )
    parser.add_argument(
        "input_path",
        help="Path to a scene root, logs/summary.json, logs/topology_v0_1.json, or logs/online_topology_lifecycle_v0_1.json.",
    )
    parser.add_argument(
        "--working-json-out",
        type=Path,
        default=None,
        help="Optional explicit working topology JSON output path.",
    )
    parser.add_argument(
        "--report-json-out",
        type=Path,
        default=None,
        help="Optional explicit comparison report JSON output path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    topology_path, lifecycle_path = resolve_topology_and_lifecycle_artifacts(Path(args.input_path))
    working_json_out, report_json_out = default_output_paths(topology_path)
    if args.working_json_out is not None:
        working_json_out = Path(args.working_json_out)
    if args.report_json_out is not None:
        report_json_out = Path(args.report_json_out)

    topology_payload = load_json(topology_path)
    lifecycle_payload = load_json(lifecycle_path)
    working_payload = build_working_topology_snapshot(
        topology_payload,
        lifecycle_payload,
        committed_room_ids=lifecycle_payload.get("committed_rooms") or [],
        source_artifacts={
            "topology_json": str(topology_path),
            "lifecycle_json": str(lifecycle_path),
        },
        lifecycle_semantics="final_report_only",
    )
    comparison_payload = build_working_vs_committed_report(
        working_payload,
        topology_payload,
        lifecycle_payload,
        committed_room_ids=lifecycle_payload.get("committed_rooms") or [],
        source_artifacts={
            "topology_json": str(topology_path),
            "lifecycle_json": str(lifecycle_path),
            "working_topology_json": str(working_json_out),
        },
    )

    write_json(working_json_out, working_payload)
    write_json(report_json_out, comparison_payload)

    print(f"Topology artifact: {topology_path}")
    print(f"Lifecycle artifact: {lifecycle_path}")
    print(f"Working topology JSON: {working_json_out}")
    print(f"Comparison report JSON: {report_json_out}")


if __name__ == "__main__":
    main()
