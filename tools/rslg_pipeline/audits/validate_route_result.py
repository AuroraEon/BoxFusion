#!/usr/bin/env python3
"""CLI validator for RSLGRouteResult artifacts.

Validates one or more ``rslg_route_result`` JSON files against the RSLG-SLAM
schema and semantic rules (no Nav2, no AMCL, blocked legacy approach never a
runtime goal, non-transition edge never used as a transition edge).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning.route_result import SCHEMA_NAME, SCHEMA_VERSION, validate


def _validate_file(path: Path) -> dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"path": path.as_posix(), "ok": False, "errors": [f"could not read/parse: {exc}"]}
    ok, errors = validate(data)
    return {
        "path": path.as_posix(),
        "ok": ok,
        "schema_name": data.get("schema_name"),
        "schema_version": data.get("schema_version"),
        "query_type": (data.get("identity") or {}).get("query_type"),
        "errors": errors,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--route-result",
        "--route-result-json",
        dest="route_results",
        action="append",
        required=True,
        help="Path to an rslg_route_result JSON file (repeatable).",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    results = [_validate_file(Path(p)) for p in args.route_results]
    ok = all(item["ok"] for item in results)
    report = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "classification": "route_result_validation_passed" if ok else "route_result_validation_failed",
        "ok": ok,
        "results": results,
    }
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
