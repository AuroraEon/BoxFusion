from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set, Tuple

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from boxfusion.backend_eval_scaffold import (
    ACTIVE_SEQUENCE_NAMES,
    DEFAULT_SCENE_OUTPUT_ROOT,
    scene_artifact_specs,
    write_scene_manifest,
)


RETENTION_TIER_CHOICES: Sequence[str] = (
    "full",
    "core_only",
    "difficult_core_only",
)
HEAVY_PRUNE_DIRS: Sequence[str] = (
    "debug_room",
    "event_spotlights",
    "final",
    "rendered_frames",
    "rgb_frames",
    "scene_graph",
    "snapshots",
)
HEAVY_PRUNE_FILES: Sequence[str] = (
    "logs/presentation_note.md",
    "logs/revisit_diagnostics.csv",
    "logs/revisit_diagnostics.json",
    "logs/revisit_events.json",
    "logs/revisit_summary.md",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prune selected scene outputs down to Tier 1 backend artifacts. Dry-run by default."
    )
    parser.add_argument(
        "--scene-output-root",
        default=str(DEFAULT_SCENE_OUTPUT_ROOT),
        help="Primary per-scene output root.",
    )
    parser.add_argument(
        "--sequences",
        nargs="+",
        default=list(ACTIVE_SEQUENCE_NAMES),
        help="Scene sequence names to inspect/prune.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually delete files. Without this flag the script only reports a dry run.",
    )
    parser.add_argument(
        "--report-json",
        default=None,
        help="Optional JSON report path.",
    )
    parser.add_argument(
        "--retention-plan",
        default=None,
        help="Optional retention-plan JSON path used to filter or preserve scenes during pruning.",
    )
    parser.add_argument(
        "--plan-retention-tiers",
        nargs="+",
        choices=list(RETENTION_TIER_CHOICES),
        default=None,
        help="When used with --retention-plan, only prune scenes whose retention_tier matches one of these values.",
    )
    parser.add_argument(
        "--preserve-keep-tier2",
        action="store_true",
        help="When used with --retention-plan, skip scenes marked with keep_tier2=true.",
    )
    parser.add_argument(
        "--preserve-showcase",
        action="store_true",
        help="When used with --retention-plan, skip scenes whose role is showcase or whose keep_tier2 flag is true.",
    )
    return parser.parse_args(argv)


def iter_files(path: Path) -> Iterable[Path]:
    if path.is_file():
        yield path
        return
    if path.is_dir():
        for child in sorted(path.rglob("*")):
            if child.is_file():
                yield child


def path_size_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in iter_files(path))


def tier1_keep_set(scene_root: Path) -> Set[Path]:
    keep: Set[Path] = set()
    for spec in scene_artifact_specs(scene_root):
        if str(spec.get("artifact_tier")) != "tier1_core_backend":
            continue
        keep.add(Path(spec["path"]).resolve())
    keep.add((scene_root / "manifest.json").resolve())
    return keep


def has_marked_ancestor(path: Path, planned: Dict[Path, int]) -> bool:
    for ancestor in path.parents:
        if ancestor in planned:
            return True
    return False


def build_prune_plan(scene_root: Path) -> Tuple[List[Path], int]:
    keep = tier1_keep_set(scene_root)
    to_remove: Dict[Path, int] = {}

    for relative_dir in HEAVY_PRUNE_DIRS:
        target = scene_root / relative_dir
        if not target.exists():
            continue
        resolved = target.resolve()
        if resolved in keep:
            continue
        to_remove[target] = path_size_bytes(target)

    for relative_file in HEAVY_PRUNE_FILES:
        target = scene_root / relative_file
        if not target.exists():
            continue
        resolved = target.resolve()
        if resolved in keep:
            continue
        if has_marked_ancestor(target, to_remove):
            continue
        to_remove[target] = path_size_bytes(target)

    for spec in scene_artifact_specs(scene_root):
        if str(spec.get("artifact_tier")) != "tier2_optional_demo":
            continue
        target = Path(spec["path"])
        if not target.exists():
            continue
        resolved = target.resolve()
        if resolved in keep:
            continue
        if has_marked_ancestor(target, to_remove):
            continue
        to_remove[target] = path_size_bytes(target)

    ordered = sorted(to_remove)
    reclaimed = int(sum(to_remove[path] for path in ordered))
    return ordered, reclaimed


def prune_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def cleanup_empty_dirs(scene_root: Path) -> None:
    for path in sorted(scene_root.rglob("*"), reverse=True):
        if path.is_dir():
            try:
                path.rmdir()
            except OSError:
                pass


def existing_dataset_root(scene_root: Path) -> Path | None:
    manifest_path = scene_root / "manifest.json"
    if not manifest_path.exists():
        return None
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    dataset_root = payload.get("dataset_root")
    if not dataset_root:
        return None
    return Path(str(dataset_root))


def load_retention_lookup(path: Path) -> Dict[str, Dict[str, object]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = list(payload.get("scenes") or [])
    return {
        str(row.get("sequence_name")): dict(row)
        for row in rows
        if row.get("sequence_name") is not None
    }


def prune_filter_reason(
    sequence_name: str,
    *,
    retention_lookup: Dict[str, Dict[str, object]],
    plan_retention_tiers: Sequence[str] | None,
    preserve_keep_tier2: bool,
    preserve_showcase: bool,
) -> str | None:
    if not retention_lookup:
        return None

    record = dict(retention_lookup.get(sequence_name) or {})
    if plan_retention_tiers:
        retention_tier = str(record.get("retention_tier") or "")
        if retention_tier not in plan_retention_tiers:
            return "filtered_by_retention_tier"

    keep_tier2 = bool(record.get("keep_tier2"))
    role = str(record.get("role") or "")
    if preserve_showcase and (keep_tier2 or role == "showcase"):
        return "preserved_showcase"
    if preserve_keep_tier2 and keep_tier2:
        return "preserved_keep_tier2"
    return None


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    scene_output_root = Path(args.scene_output_root)
    retention_lookup: Dict[str, Dict[str, object]] = {}
    retention_plan_path = None
    if args.retention_plan:
        retention_plan_path = Path(args.retention_plan)
        retention_lookup = load_retention_lookup(retention_plan_path)
    report_rows = []
    total_reclaimable = 0

    for sequence_name in args.sequences:
        filter_reason = prune_filter_reason(
            sequence_name,
            retention_lookup=retention_lookup,
            plan_retention_tiers=args.plan_retention_tiers,
            preserve_keep_tier2=bool(args.preserve_keep_tier2),
            preserve_showcase=bool(args.preserve_showcase),
        )
        plan_record = dict(retention_lookup.get(sequence_name) or {})
        if filter_reason is not None:
            report_rows.append(
                {
                    "sequence_name": sequence_name,
                    "scene_root": str(scene_output_root / str(sequence_name)),
                    "status": filter_reason,
                    "mode": "apply" if args.apply else "dry_run",
                    "paths_to_remove": [],
                    "reclaimable_bytes": 0,
                    "retention_tier": plan_record.get("retention_tier"),
                    "keep_tier2": plan_record.get("keep_tier2"),
                    "role": plan_record.get("role"),
                }
            )
            continue

        scene_root = scene_output_root / str(sequence_name)
        if not scene_root.exists():
            report_rows.append(
                {
                    "sequence_name": sequence_name,
                    "scene_root": str(scene_root),
                    "status": "missing_scene_root",
                    "mode": "apply" if args.apply else "dry_run",
                    "paths_to_remove": [],
                    "reclaimable_bytes": 0,
                    "retention_tier": plan_record.get("retention_tier"),
                    "keep_tier2": plan_record.get("keep_tier2"),
                    "role": plan_record.get("role"),
                }
            )
            continue

        paths_to_remove, reclaimable_bytes = build_prune_plan(scene_root)
        total_reclaimable += reclaimable_bytes
        if args.apply:
            for path in paths_to_remove:
                prune_path(path)
            cleanup_empty_dirs(scene_root)
            write_scene_manifest(
                scene_root,
                dataset_root=existing_dataset_root(scene_root),
                sequence_name=sequence_name,
            )

        report_rows.append(
            {
                "sequence_name": sequence_name,
                "scene_root": str(scene_root),
                "status": "pruned" if args.apply else "planned",
                "mode": "apply" if args.apply else "dry_run",
                "paths_to_remove": [str(path) for path in paths_to_remove],
                "reclaimable_bytes": int(reclaimable_bytes),
                "retention_tier": plan_record.get("retention_tier"),
                "keep_tier2": plan_record.get("keep_tier2"),
                "role": plan_record.get("role"),
            }
        )

    payload = {
        "scene_output_root": str(scene_output_root),
        "mode": "apply" if args.apply else "dry_run",
        "keep_policy": "tier1_only",
        "retention_plan": None if retention_plan_path is None else str(retention_plan_path),
        "retention_tier_filters": list(args.plan_retention_tiers or []),
        "preserve_keep_tier2": bool(args.preserve_keep_tier2),
        "preserve_showcase": bool(args.preserve_showcase),
        "total_reclaimable_bytes": int(total_reclaimable),
        "scenes": report_rows,
    }
    if args.report_json:
        report_path = Path(args.report_json)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
