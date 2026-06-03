#!/usr/bin/env python3
"""Reusable RSLG-SLAM dynamic object-nav wrapper with optional GUI/RViz evidence.

This entry point delegates to the task16 dynamic query pipeline.  It accepts
the pipeline arguments directly, including --query, --object-id, --start-room,
--floor-id, --output-dir, --ros-domain-id, --gui/--headless,
--keep-open-sec, --execute/--plan-only, and
--use-current-robot-pose-if-running.
"""

from __future__ import annotations

from run_dynamic_object_query_nav import main


if __name__ == "__main__":
    raise SystemExit(main())
