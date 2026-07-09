#!/usr/bin/env python3
"""Build a read-only Layer 0 input manifest for RSLG-SLAM.

This helper records dataset, path, and model/text provenance only. It does not
run inference, import legacy demo modules, require a GPU, or modify canonical
artifacts.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

PROJECT_NAME = "RSLG-SLAM"
SCHEMA_NAME = "rslg_layer0_input_manifest"
SCHEMA_VERSION = "0.1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _path_value(raw: str | None) -> str | None:
    if raw in (None, ""):
        return None
    return Path(raw).as_posix()


def _path_record(raw: str | None) -> dict[str, Any]:
    value = _path_value(raw)
    if value is None:
        return {"path": None, "exists": None, "is_file": None, "is_dir": None}
    path = Path(value)
    return {
        "path": value,
        "exists": path.exists(),
        "is_file": path.is_file(),
        "is_dir": path.is_dir(),
    }


def _is_under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def build_manifest(args: argparse.Namespace) -> dict[str, Any]:
    canonical_root = _path_value(args.canonical_root)
    intended_layer1_output_root = (
        f"{canonical_root}/layer1_world_model" if canonical_root else None
    )
    return {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT_NAME,
        "scene_id": args.scene_id,
        "dataset_name": args.dataset_name,
        "sequence_id": args.sequence_id,
        "rgb_root": _path_record(args.rgb_root),
        "depth_root": _path_record(args.depth_root),
        "pose_root": _path_record(args.pose_root),
        "semantic_assets": {
            "class_text_path": _path_record(args.class_text_path),
            "text_features_path": _path_record(args.text_features_path),
        },
        "config_path": _path_record(args.config_path),
        "model_checkpoint_path": _path_record(args.model_checkpoint_path),
        "clip_checkpoint_path": _path_record(args.clip_checkpoint_path),
        "text_features_path": _path_record(args.text_features_path),
        "class_text_path": _path_record(args.class_text_path),
        "frame_count_if_known": args.frame_count_if_known,
        "canonical_output_root": canonical_root,
        "intended_layer1_output_root": intended_layer1_output_root,
        "provenance_notes": args.provenance_notes,
        "claim_boundary": {
            "manifest_only": True,
            "runs_inference": False,
            "imports_legacy_stage_a": False,
            "requires_gpu": False,
            "modifies_canonical_artifacts": False,
            "query_task_is_layer3_input": True,
            "not_full_raw_rgbd_to_layer1_rerun": True,
        },
        "created_by": args.created_by,
        "created_at": utc_now(),
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", required=True, help="Manifest JSON path to write.")
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--dataset-name", default=None)
    parser.add_argument("--sequence-id", default=None)
    parser.add_argument("--rgb-root", default=None)
    parser.add_argument("--depth-root", default=None)
    parser.add_argument("--pose-root", default=None)
    parser.add_argument("--config-path", default=None)
    parser.add_argument("--model-checkpoint-path", default=None)
    parser.add_argument("--clip-checkpoint-path", default=None)
    parser.add_argument("--class-text-path", default=None)
    parser.add_argument("--text-features-path", default=None)
    parser.add_argument("--frame-count-if-known", type=int, default=None)
    parser.add_argument("--canonical-root", default=None)
    parser.add_argument("--provenance-notes", default=None)
    parser.add_argument("--created-by", default="tools/rslg_pipeline/build_input_manifest.py")
    parser.add_argument(
        "--no-canonical-write",
        action="store_true",
        help="Refuse to write the manifest under the recorded canonical root.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    output_json = Path(args.output_json)
    if args.no_canonical_write and args.canonical_root:
        if _is_under(output_json, Path(args.canonical_root)):
            print(
                json.dumps(
                    {
                        "ok": False,
                        "error": "--no-canonical-write refused output under canonical root",
                        "output_json": output_json.as_posix(),
                        "canonical_root": Path(args.canonical_root).as_posix(),
                    },
                    indent=2,
                    sort_keys=True,
                )
            )
            return 2

    manifest = build_manifest(args)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(manifest, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "ok": True,
                "schema_name": SCHEMA_NAME,
                "scene_id": args.scene_id,
                "output_json": output_json.as_posix(),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
