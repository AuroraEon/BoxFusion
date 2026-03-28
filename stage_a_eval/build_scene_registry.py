from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Sequence

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import (
    ACTIVE_HM3D_DATASET_ROOT,
    ACTIVE_SEQUENCE_NAMES,
    DEFAULT_LEGACY_SCENE_OUTPUT_ROOT,
    DEFAULT_SCENE_OUTPUT_ROOT,
    build_scene_registry,
    dump_json,
    find_scene_root,
    write_scene_manifest,
)


DEFAULT_REGISTRY_OUT = Path("stage_a_eval/scene_registry.json")


def resolve_legacy_scene_output_root(*, allow_legacy_fallback: bool, legacy_scene_output_root: str) -> Path | None:
    if not allow_legacy_fallback:
        return None
    text = str(legacy_scene_output_root or "").strip()
    if not text:
        return None
    return Path(text)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build the active-scene registry and backfill per-scene manifest files.")
    parser.add_argument(
        "--dataset-root",
        default=str(ACTIVE_HM3D_DATASET_ROOT),
        help="Active HM3D dataset root.",
    )
    parser.add_argument(
        "--scene-output-root",
        default=str(DEFAULT_SCENE_OUTPUT_ROOT),
        help="Preferred regenerated per-scene output root.",
    )
    parser.add_argument(
        "--legacy-scene-output-root",
        default=str(DEFAULT_LEGACY_SCENE_OUTPUT_ROOT),
        help="Migration-only read-only fallback root. Ignored unless --allow-legacy-fallback is set.",
    )
    parser.add_argument(
        "--allow-legacy-fallback",
        action="store_true",
        help="Allow a migration-only read-only fallback scan of the legacy root.",
    )
    parser.add_argument(
        "--registry-out",
        default=str(DEFAULT_REGISTRY_OUT),
        help="Registry JSON output path.",
    )
    parser.add_argument(
        "--write-manifests",
        action="store_true",
        help="Backfill manifest.json files for discovered scene outputs.",
    )
    parser.add_argument(
        "--sequences",
        nargs="*",
        default=list(ACTIVE_SEQUENCE_NAMES),
        help="Optional subset of full HM3D sequence names.",
    )
    return parser


def maybe_write_manifests(
    *,
    dataset_root: Path,
    scene_output_root: Path,
    legacy_scene_output_root: Path | None,
    sequences: Sequence[str],
) -> int:
    written = 0
    for sequence_name in sequences:
        planned_root = Path(scene_output_root) / sequence_name
        scene_root = find_scene_root(
            sequence_name,
            preferred_root=scene_output_root,
            fallback_roots=[] if legacy_scene_output_root is None else [legacy_scene_output_root],
        )
        if scene_root is None:
            continue
        if scene_root != planned_root:
            continue
        write_scene_manifest(
            scene_root,
            dataset_root=dataset_root,
            sequence_name=sequence_name,
        )
        written += 1
    return written


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    dataset_root = Path(args.dataset_root)
    scene_output_root = Path(args.scene_output_root)
    legacy_scene_output_root = resolve_legacy_scene_output_root(
        allow_legacy_fallback=bool(args.allow_legacy_fallback),
        legacy_scene_output_root=args.legacy_scene_output_root,
    )
    sequences = [str(item) for item in args.sequences if str(item).strip()]

    if args.write_manifests:
        maybe_write_manifests(
            dataset_root=dataset_root,
            scene_output_root=scene_output_root,
            legacy_scene_output_root=legacy_scene_output_root,
            sequences=sequences,
        )

    registry = build_scene_registry(
        dataset_root=dataset_root,
        sequence_names=sequences,
        scene_output_root=scene_output_root,
        legacy_scene_output_root=legacy_scene_output_root,
    )
    dump_json(Path(args.registry_out), registry)
    print(json.dumps({"registry_out": str(args.registry_out), "scene_count": len(registry.get("scenes", []))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
