#!/usr/bin/env python3
"""CLI validator for RSLGQueryTask specs.

Validates one or more ``rslg_query_task`` JSON files against the RSLG-SLAM
schema and semantic rules (object-centric queries must forbid the blocked legacy
approach id as a runtime goal, cross-floor queries must forbid the non-transition
edge, DualMap support classification must be one of the allowed values).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning.query_task import SCHEMA_NAME, SCHEMA_VERSION, validate


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
        "query_id": data.get("query_id"),
        "query_type": data.get("query_type"),
        "errors": errors,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--query-json",
        dest="query_jsons",
        action="append",
        required=True,
        help="Path to an rslg_query_task JSON file (repeatable).",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    results = [_validate_file(Path(p)) for p in args.query_jsons]
    ok = all(item["ok"] for item in results)
    report = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "classification": "query_task_validation_passed" if ok else "query_task_validation_failed",
        "ok": ok,
        "results": results,
    }
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
