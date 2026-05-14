#!/usr/bin/env python3
"""Deprecated compatibility wrapper for Step30S7 request map validation.

The old room15-only repair path was removed from active use. This entrypoint
now fails if asked to repair and otherwise directs callers to the Step30S7
request-aware validation gate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--repair-if-needed", action="store_true")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--visual-output-dir", type=Path)
    args = parser.parse_args()
    payload = {
        "artifact_type": "deprecated_room15_map_coverage_entrypoint",
        "active_repair_available": False,
        "replacement": "tools/stage1_step30p1/build_stage1_step30p1_request_aware_nav_map.py plus validate_stage1_step30p1_request_map.py",
        "message": "The room15-only map repair was removed from active Step30S7 runtime use.",
    }
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text("# Deprecated Room15 Map Coverage Entrypoint\n\nUse the Step30S7 request-aware projection and validation tools.\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 2 if args.repair_if_needed else 0


if __name__ == "__main__":
    raise SystemExit(main())

