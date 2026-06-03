#!/usr/bin/env python3
"""Build the offline object candidate index for task14a."""

from __future__ import annotations

import argparse
from pathlib import Path

from object_nav_common import (
    COMMITTED_SNAPSHOT_JSON,
    FINAL_VECTOR_JSON,
    TASK_DIR,
    TOPOLOGY_JSON,
    build_candidate_index_data,
    write_json,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topology", type=Path, default=TOPOLOGY_JSON)
    parser.add_argument("--snapshot", type=Path, default=COMMITTED_SNAPSHOT_JSON)
    parser.add_argument("--final-vector-map", type=Path, default=FINAL_VECTOR_JSON)
    parser.add_argument("--output", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    args = parser.parse_args()
    index = build_candidate_index_data(args.topology, args.snapshot, args.final_vector_map)
    write_json(args.output, index)
    print(f"wrote {args.output}")
    print(f"indexed_objects={index['counts']['indexed_objects']}")


if __name__ == "__main__":
    main()
