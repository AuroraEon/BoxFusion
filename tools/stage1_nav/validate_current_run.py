#!/usr/bin/env python3
"""Validate a completed Stage1 navigation run.

Wrapper around the internal physical validation with clean interface.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    internal = script_dir.parent / "stage1_step30p1" / "validate_stage1_step30p1_physical.py"
    if not internal.exists():
        print(f"[stage1_nav][ERROR] Internal script not found: {internal}", file=sys.stderr)
        return 1
    return subprocess.call([sys.executable, str(internal)] + sys.argv[1:], env=os.environ)


if __name__ == "__main__":
    raise SystemExit(main())
