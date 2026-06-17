#!/usr/bin/env python3
"""Export task42 RViz MarkerArray replay inputs."""

from __future__ import annotations

from pathlib import Path

from tools.rslg_pipeline.finalize_task42_demo_evidence_pack import (
    SCENE_ID,
    export_markers,
    export_rviz_config,
)


def main() -> int:
    root = Path.cwd().resolve()
    demo_dir = root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack"
    created: list[str] = []
    export_markers(root, demo_dir, created)
    export_rviz_config(root, demo_dir, created)
    print(f"exported {len(created)} task42 RViz replay files")
    for item in created:
        print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
