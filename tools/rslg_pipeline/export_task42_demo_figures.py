#!/usr/bin/env python3
"""Export task42 demo-ready replay figures."""

from __future__ import annotations

from pathlib import Path

from tools.rslg_pipeline.finalize_task42_demo_evidence_pack import SCENE_ID, export_figures


def main() -> int:
    root = Path.cwd().resolve()
    demo_dir = root / f"stage_outputs/rslg_slam/{SCENE_ID}/canonical/demo_evidence_pack"
    created: list[str] = []
    manifest = export_figures(root, demo_dir, created)
    print(f"exported {len(manifest['figures'])} task42 demo figures")
    for item in created:
        print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
