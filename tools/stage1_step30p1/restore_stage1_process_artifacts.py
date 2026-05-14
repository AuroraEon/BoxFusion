#!/usr/bin/env python3
"""Restore compact Stage1 process artifacts for the 00824 Step30P1 artifact."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"
PROVENANCE_REL = "manifest/provenance_manifest_v0_2.json"
OLD_PREFIX = "runtime_stage1_frozen_evidence/"
OLD_STEP_PATH_RE = re.compile(r"(runtime_stage1_frozen_evidence|referenced_artifacts)/step[0-9][^\s\"'<>),\]]*")
DEFAULT_BACKUP_ROOT = Path(os.environ.get("BOXFUSION_BACKUP_ROOT", "/home/ws/workspace/runtime_stage1_frozen_evidence"))
SELECTED_GATEWAY_PAIRS = [
    "r1_r3",
    "r3_r7",
    "r3_r8",
    "r7_r11",
    "r7_r14",
    "r7_r15",
    "r8_r11",
    "r14_r16",
]
SELECTED_GATEWAY_IDS = {
    "r1_r3": "gw_00824_r1_r3_01",
    "r3_r7": "gw_00824_r3_r7_01",
    "r3_r8": "gw_00824_r3_r8_01",
    "r7_r11": "gw_00824_r7_r11_02",
    "r7_r14": "gw_00824_r7_r14_01",
    "r7_r15": "gw_00824_r7_r15_01",
    "r8_r11": "gw_00824_r8_r11_01",
    "r14_r16": "gw_00824_r14_r16_01",
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def flatten_sources(value: Any) -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key == "source" and isinstance(item, str):
                found.append(item)
            else:
                found.extend(flatten_sources(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(flatten_sources(item))
    return found


def resolve_old_source(source: str, backup_root: Path, stage_output: Path) -> Path | None:
    if source.startswith(OLD_PREFIX):
        candidate = backup_root / source[len(OLD_PREFIX) :]
        if candidate.is_file():
            return candidate
    candidate = REPO_ROOT / source
    if candidate.is_file():
        return candidate
    candidate = stage_output / source
    if candidate.is_file():
        return candidate
    return None


def source_index(provenance: Any, backup_root: Path, stage_output: Path) -> dict[str, tuple[str, Path]]:
    index: dict[str, tuple[str, Path]] = {}
    for source in flatten_sources(provenance):
        resolved = resolve_old_source(source, backup_root, stage_output)
        if resolved:
            index.setdefault(resolved.name, (source, resolved))
    return index


def find_by_name(name: str, index: dict[str, tuple[str, Path]], backup_root: Path, stage_output: Path, hint: str | None = None) -> tuple[str, Path] | None:
    if name in index and (not hint or hint in index[name][1].as_posix()):
        return index[name]
    for root in (backup_root, stage_output):
        if not root.exists():
            continue
        matches = [
            path for path in root.rglob(name)
            if path.is_file() and "stage1_process" not in path.parts
        ]
        if hint:
            hinted = [path for path in matches if hint in path.as_posix()]
            if hinted:
                matches = hinted
        if matches:
            path = sorted(matches, key=lambda p: len(p.as_posix()))[0]
            source = path.as_posix()
            try:
                source = OLD_PREFIX + path.relative_to(backup_root).as_posix()
            except ValueError:
                try:
                    source = path.relative_to(REPO_ROOT).as_posix()
                except ValueError:
                    pass
            return source, path
    return None


def copy_one(source: str, src: Path, dst: Path, include_large_debug: bool, category: str) -> dict[str, Any] | None:
    source_size = src.stat().st_size
    source_sha = sha256_file(src)
    large_debug = any(part in {"rgb_frames", "snapshots", "debug_room"} for part in src.parts)
    if large_debug and not include_large_debug:
        return None
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    sanitized = False
    if dst.suffix.lower() in {".json", ".md", ".txt", ".csv", ".yaml", ".yml"}:
        text = dst.read_text(encoding="utf-8", errors="ignore")
        scrubbed = OLD_STEP_PATH_RE.sub("RESTORED_SOURCE_PATH_REDACTED_SEE_STAGE1_PROCESS_PROVENANCE", text)
        if scrubbed != text:
            dst.write_text(scrubbed, encoding="utf-8")
            sanitized = True
    return {
        "category": category,
        "destination": rel(dst),
        "sha256": sha256_file(dst),
        "size_bytes": dst.stat().st_size,
        "source": source,
        "source_sha256": source_sha,
        "source_size_bytes": source_size,
        "sanitized_old_step_paths_in_active_copy": sanitized,
    }


def add_name(
    requested: list[dict[str, str]],
    name: str,
    section: str,
    subdir: str,
    category: str,
) -> None:
    requested.append({"name": name, "section": section, "subdir": subdir, "category": category})


def requested_specs() -> list[dict[str, str]]:
    specs: list[dict[str, str]] = []
    for name in [
        "00824_step30a_global_room_mask_v0_1.npy",
        "00824_step30a_layered_bev_v0_1.json",
        "00824_step30a_layered_bev_v0_1.npz",
        "00824_step30a_dual_wall_layer_metadata_v0_1.json",
        "00824_step30a_summary_v0_1.json",
        "00824_step30a_stage_a_rerun_manifest_v0_1.json",
        "00824_step30a_validation_results_v0_1.json",
        "final_gateway_wall_preclose_thr_0p25.png",
        "final_walls_skeleton.png",
        "free_space.png",
        "gateway_wall_preclose.png",
        "outside_boundary.png",
        "room_mask_global_id.png",
        "segmentation_wall_processed.png",
        "unknown_layer.png",
        "gateway_wall_preclose_over_free_space.png",
        "gateway_wall_preclose_over_room_mask.png",
        "gateway_wall_preclose_threshold_metadata.png",
        "segmentation_wall_vs_gateway_wall_preclose.png",
    ]:
        subdir = "assets" if name.endswith((".json", ".npy", ".npz")) else "visualizations"
        add_name(specs, name, "room_segmentation", subdir, "room_segmentation")

    for name in [
        "00824_step30a_gateway_candidates_v0_1.json",
        "00824_step30a_gateway_hypotheses_v0_1.json",
        "00824_step30a_gateway_hypotheses_with_roles_v0_1.json",
        "00824_step30a_gateway_wall_benchmark_v0_1.json",
        "00824_step30a_route_candidates_for_later_topology_v0_1.json",
        "00824_step30b_8_gateway_benchmark_v0_1.json",
        "00824_step30b_gateway_hypotheses_v0_1.json",
        "00824_step30b_gateway_hypotheses_with_roles_v0_1.json",
        "00824_step30b_pair_local_visualization_manifest_v0_1.json",
        "00824_step30b_route_candidates_for_later_topology_v0_1.json",
        "00824_step30b_summary_v0_1.json",
        "00824_step30b_validation_results_v0_1.json",
        "00824_step30b2_auto_gateway_candidate_scores_v0_1.json",
        "00824_step30b2_auto_gateway_selection_v0_1.json",
        "00824_step30b2_auto_vs_truth_evaluation_v0_1.json",
        "00824_step30b2_candidate_level_gateway_truth_v0_1.json",
        "00824_step30b2_corrected_auto_route_candidates_for_step30c_v0_1.json",
        "00824_step30b2_failure_analysis_v0_1.json",
        "00824_step30b2_summary_v0_1.json",
        "00824_step30b2_validation_results_v0_1.json",
        "gateway_candidates_or_hypotheses_v0_1.json",
        "gateway_truth_blind_selection_v0_1.json",
        "selected_gateway_summary_v0_1.json",
    ]:
        add_name(specs, name, "gateway_extraction", "assets", "gateway_extraction")

    for pair, gateway_id in SELECTED_GATEWAY_IDS.items():
        add_name(specs, f"{gateway_id}_local.png", "gateway_extraction", f"visualizations/pair_local/{pair}", "gateway_visualization")
        add_name(specs, "overview_pair_local.png", "gateway_extraction", f"visualizations/pair_local/{pair}", "gateway_visualization")
    return specs


def destination_for(spec: dict[str, str], stage_process: Path) -> Path:
    name = spec["name"]
    subdir = spec["subdir"]
    if name == "overview_pair_local.png" and "/" in subdir:
        return stage_process / spec["section"] / subdir / name
    return stage_process / spec["section"] / subdir / name


def write_summary_files(stage_process: Path, room_files: list[dict[str, Any]], gateway_files: list[dict[str, Any]], missing: list[dict[str, str]], copied: list[dict[str, Any]]) -> None:
    readme = f"""# Stage1 Process Artifacts

This directory restores compact process artifacts for the cleaned 00824 Step30P1 milestone.

It preserves the files needed to explain room segmentation, dual-wall/gateway-wall-preclose processing, and gateway extraction/selection without restoring old StepXX runtime folders.

- `room_segmentation/`: masks, layered BEV assets, dual-wall metadata, and curated segmentation/preclose visualizations.
- `gateway_extraction/`: gateway candidates, hypotheses, truth-blind selection, benchmark records, selected summaries, and selected-gateway visualizations.
- `provenance/`: source paths, hashes, sizes, and restore notes. Historical StepXX paths appear only as provenance metadata.

Restored files: {len(copied)}
Missing files: {len(missing)}
"""
    (stage_process / "README.md").write_text(readme, encoding="utf-8")

    room_summary = f"""# Room Segmentation Process Summary

Restored compact Stage1 room-segmentation process artifacts for scene `00824-Dd4bFSTQ8gi`.

The set includes the global room mask, layered BEV JSON/NPZ, dual-wall metadata, gateway-wall-preclose and wall-skeleton rasters, plus curated raster/layer visualizations. It intentionally excludes full RGB frame dumps and broad debug-room sweeps unless `--include-large-debug` is used.

Restored room-segmentation files: {len(room_files)}
Missing requested room-segmentation files: {sum(1 for item in missing if item.get("section") == "room_segmentation")}
"""
    (stage_process / "room_segmentation/room_segmentation_summary_v0_1.md").write_text(room_summary, encoding="utf-8")

    gateway_summary = f"""# Gateway Extraction Process Summary

Restored compact gateway-extraction process artifacts for the eight selected 00824 gateway pairs:

{chr(10).join(f"- `{pair}` -> `{SELECTED_GATEWAY_IDS[pair]}`" for pair in SELECTED_GATEWAY_PAIRS)}

The set includes candidate and hypothesis records, benchmark and validation summaries, truth-blind selection records, and selected-gateway pair-local visualizations. It does not restore old StepXX folders as active directories.

Restored gateway-extraction files: {len(gateway_files)}
Missing requested gateway-extraction files: {sum(1 for item in missing if item.get("section") == "gateway_extraction")}
"""
    (stage_process / "gateway_extraction/gateway_extraction_summary_v0_1.md").write_text(gateway_summary, encoding="utf-8")

    prov_md = f"""# Restored Process Artifact Provenance

Created: `{now_iso()}`

The restore copied compact process artifacts into semantic directories under `stage_outputs/stage1_00824_step30p1/stage1_process/`.

Historical StepXX source paths are preserved only in the JSON provenance manifest. No active StepXX directory was recreated.

Restored files: {len(copied)}
Missing files: {len(missing)}
"""
    (stage_process / "provenance/restored_process_artifacts_provenance_v0_1.md").write_text(prov_md, encoding="utf-8")

    if missing:
        lines = [
            "# Missing Restore Sources",
            "",
            "The restore did not fabricate missing process artifacts. Provide a backup root containing these source files, or rerun with `--backup-root` pointing to the expected evidence tree.",
            "",
        ]
        for item in missing:
            lines.append(f"- `{item['name']}` for `{item['section']}`; expected source category `{item['category']}`")
        (stage_process / "provenance/missing_restore_sources_report_v0_1.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def public_file_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    public_rows = []
    for row in rows:
        public_rows.append({
            "category": row["category"],
            "destination": row["destination"],
            "sha256": row["sha256"],
            "size_bytes": row["size_bytes"],
            "sanitized_old_step_paths_in_active_copy": row.get("sanitized_old_step_paths_in_active_copy", False),
        })
    return public_rows


def restore(stage_output: Path, backup_root: Path, include_large_debug: bool) -> dict[str, Any]:
    provenance_path = stage_output / PROVENANCE_REL
    provenance = load_json(provenance_path) if provenance_path.is_file() else {}
    index = source_index(provenance, backup_root, stage_output)
    stage_process = stage_output / "stage1_process"
    copied: list[dict[str, Any]] = []
    missing: list[dict[str, str]] = []

    for spec in requested_specs():
        hint = None
        if "visualizations/pair_local/" in spec["subdir"]:
            hint = spec["subdir"].replace("visualizations/", "")
        found = find_by_name(spec["name"], index, backup_root, stage_output, hint)
        if not found:
            missing.append(spec)
            continue
        source, src = found
        row = copy_one(source, src, destination_for(spec, stage_process), include_large_debug, spec["category"])
        if row:
            copied.append(row)

    room_files = [row for row in copied if row["destination"].startswith(rel(stage_process / "room_segmentation"))]
    gateway_files = [row for row in copied if row["destination"].startswith(rel(stage_process / "gateway_extraction"))]
    room_manifest = {
        "artifact_type": "stage1_room_segmentation_process_manifest",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": rel(stage_output),
        "file_count": len(room_files),
        "files": public_file_rows(room_files),
        "missing": [item for item in missing if item["section"] == "room_segmentation"],
    }
    gateway_manifest = {
        "artifact_type": "stage1_gateway_extraction_process_manifest",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": rel(stage_output),
        "selected_gateway_pairs": SELECTED_GATEWAY_IDS,
        "file_count": len(gateway_files),
        "files": public_file_rows(gateway_files),
        "missing": [item for item in missing if item["section"] == "gateway_extraction"],
    }
    provenance_manifest = {
        "artifact_type": "stage1_restored_process_artifacts_provenance",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": rel(stage_output),
        "backup_root": backup_root.as_posix(),
        "source_provenance_manifest": rel(provenance_path),
        "policy": "Historical StepXX source paths are preserved only as provenance metadata. Active process artifact directories use semantic names.",
        "include_large_debug": include_large_debug,
        "restored_file_count": len(copied),
        "missing_file_count": len(missing),
        "restored_files": copied,
        "missing": missing,
    }
    write_json(stage_process / "room_segmentation/room_segmentation_manifest_v0_1.json", room_manifest)
    write_json(stage_process / "gateway_extraction/gateway_extraction_manifest_v0_1.json", gateway_manifest)
    write_json(stage_process / "provenance/restored_process_artifacts_provenance_v0_1.json", provenance_manifest)
    write_summary_files(stage_process, room_files, gateway_files, missing, copied)
    return provenance_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--backup-root", type=Path, default=DEFAULT_BACKUP_ROOT)
    parser.add_argument("--include-large-debug", action="store_true")
    args = parser.parse_args()
    payload = restore(args.stage_output_dir.resolve(), args.backup_root.resolve(), args.include_large_debug)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["missing_file_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
