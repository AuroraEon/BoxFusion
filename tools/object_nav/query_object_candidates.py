#!/usr/bin/env python3
"""Query the offline object candidate index without LLM/VLM calls."""

from __future__ import annotations

import argparse
from pathlib import Path

from object_nav_common import TASK_DIR, load_index, run_query, write_json


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    parser.add_argument("--query", required=True)
    parser.add_argument("--preferred-room-id")
    parser.add_argument("--preferred-floor-id")
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--output", type=Path, default=TASK_DIR / "object_query_result_v0_1.json")
    args = parser.parse_args()
    result = run_query(load_index(args.index), args.query, args.preferred_room_id, args.preferred_floor_id, args.top_k)
    write_json(args.output, result)
    print(f"wrote {args.output}")
    if result.get("selected_candidate"):
        selected = result["selected_candidate"]
        print(f"selected={selected['object_id']} {selected['label']} {selected['room_id']} {selected['floor_id']}")
    else:
        print(f"failure_reason={result.get('failure_reason')}")


if __name__ == "__main__":
    main()
