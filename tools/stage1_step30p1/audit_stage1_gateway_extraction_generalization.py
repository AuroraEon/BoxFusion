#!/usr/bin/env python3
"""Step30S6 generalization audit for gateway extraction artifacts and code paths."""

from __future__ import annotations

import argparse
import ast
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"
DEFAULT_BACKUP_ROOT = Path("/home/ws/workspace/runtime_stage1_frozen_evidence")
DEFAULT_OUTPUT_DIR = (
    DEFAULT_STAGE_OUTPUT
    / "post_restructure_validation/step30s6_root_cause_and_gateway_generalization"
)

PATTERNS = {
    "hard_coded_scene_id": re.compile(r"00824|Dd4bFSTQ8gi"),
    "hard_coded_room_or_pair": re.compile(r"room_?15|room_?16|room_?7|r7[_-]r15|r14[_-]r16|r3[_-]r11", re.IGNORECASE),
    "gateway_id": re.compile(r"gw_00824_[a-z0-9_]+"),
    "truth_or_label": re.compile(r"truth|positive_control|negative_sanity|manual_visual|calibration_label", re.IGNORECASE),
    "forbidden": re.compile(r"forbidden|hard_negative", re.IGNORECASE),
    "threshold": re.compile(r"threshold|score|penalty|radius|factor|min_|max_", re.IGNORECASE),
    "route_dependency": re.compile(r"route|through_room|terminal_room|expected_gateway|KNOWN_EDGES|ROUTE_", re.IGNORECASE),
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(v) for v in value]
    return value


def candidate_files(stage_output: Path, backup_root: Path) -> list[Path]:
    files: list[Path] = []
    roots = [
        stage_output / "stage1_process/gateway_extraction",
        stage_output / "gateway",
        stage_output / "route",
        REPO_ROOT / "tools/stage1_step30p1",
        REPO_ROOT / "tools",
        REPO_ROOT / "stage_a_demo.py",
        REPO_ROOT / "boxfusion/stage_a_demo.py",
        backup_root / "step30a_00824_full_stage_a_dual_wall_gateway_rerun",
        backup_root / "step30b_00824_gateway_benchmark_expansion",
        backup_root / "step30b2_00824_candidate_level_gateway_truth_and_auto_selection",
        backup_root / "step30h8r2_00824_preclose_gateway_preserving_nav_map_repair/scripts",
    ]
    for root in roots:
        if root.is_file():
            files.append(root)
            continue
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in {".py", ".sh", ".json", ".md"}:
                if "__pycache__" not in path.parts:
                    files.append(path)
    return sorted(set(files))


def scan_text_files(files: list[Path]) -> dict[str, Any]:
    hits: list[dict[str, Any]] = []
    counts = {key: 0 for key in PATTERNS}
    for path in files:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            matched = [name for name, pattern in PATTERNS.items() if pattern.search(line)]
            if not matched:
                continue
            for name in matched:
                counts[name] += 1
            sample = line.strip()
            if len(sample) > 220:
                sample = sample[:217] + "..."
            hits.append({"path": rel(path), "line": lineno, "classes": matched, "sample": sample})
    return {
        "pattern_counts": counts,
        "sample_hits": hits[:400],
        "hit_count_total": len(hits),
    }


def py_constant_audit(files: list[Path]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    interesting_names = {
        "KNOWN_EDGES",
        "SELECTED_GATEWAY_IDS",
        "ALL_GATEWAY_IDS",
        "ALL_GATEWAY_PAIRS",
        "FORBIDDEN_UNDIRECTED_PAIRS",
        "FORBIDDEN_PAIRS",
        "ROUTE_GATEWAY_SEQUENCE",
        "ROUTE_ROOM_IDS",
        "EXPECTED_GATEWAY_SEQUENCE",
        "EXPECTED_ROOM_CHAIN",
    }
    for path in files:
        if path.suffix != ".py":
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
        except Exception:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                names = [target.id for target in node.targets if isinstance(target, ast.Name)]
                for name in names:
                    if name not in interesting_names:
                        continue
                    try:
                        value = ast.literal_eval(node.value)
                    except Exception:
                        value = "<non_literal>"
                    records.append({
                        "path": rel(path),
                        "line": getattr(node, "lineno", None),
                        "name": name,
                        "value_preview": value if isinstance(value, (str, int, float, list, tuple, dict, set)) else repr(value),
                        "classification": classify_constant(path, name),
                    })
    return records


def classify_constant(path: Path, name: str) -> str:
    p = path.as_posix()
    if "prepare_stage1_step30p1_room15_route" in p or "run_stage1_step30p1_route" in p or "validate_stage1_step30p1_physical" in p:
        return "scene_route_demo_specific_not_gateway_extraction"
    if "restore_stage1_process_artifacts" in p:
        return "artifact_restoration_selection_specific"
    if "step30h8r2" in p:
        return "map_projection_specific_risky_if_treated_as_general"
    return "unknown_needs_review"


def summarize_gateway_artifacts(stage_output: Path) -> dict[str, Any]:
    assets = stage_output / "stage1_process/gateway_extraction/assets"
    current_gateway = stage_output / "gateway"
    paths = {
        "step30a_candidates": assets / "00824_step30a_gateway_candidates_v0_1.json",
        "step30a_hypotheses": assets / "00824_step30a_gateway_hypotheses_v0_1.json",
        "step30a_hypotheses_with_roles": assets / "00824_step30a_gateway_hypotheses_with_roles_v0_1.json",
        "step30b2_scores": assets / "00824_step30b2_auto_gateway_candidate_scores_v0_1.json",
        "step30b2_selection": assets / "00824_step30b2_auto_gateway_selection_v0_1.json",
        "step30b2_truth": assets / "00824_step30b2_candidate_level_gateway_truth_v0_1.json",
        "step30b2_eval": assets / "00824_step30b2_auto_vs_truth_evaluation_v0_1.json",
        "current_public_candidates": current_gateway / "gateway_candidates_or_hypotheses_v0_1.json",
        "current_public_truth_blind_selection": current_gateway / "gateway_truth_blind_selection_v0_1.json",
        "current_public_selected_summary": current_gateway / "selected_gateway_summary_v0_1.json",
    }
    out: dict[str, Any] = {"paths": {k: rel(v) for k, v in paths.items() if v.exists()}}

    if paths["step30a_candidates"].exists():
        data = read_json(paths["step30a_candidates"])
        out["candidate_generation"] = {
            "artifact_type": data.get("artifact_type"),
            "gateway_filter_wall_layer": data.get("gateway_filter_wall_layer"),
            "segmentation_wall_processed_used_as_hard_gateway_blocker": data.get("segmentation_wall_processed_used_as_hard_gateway_blocker"),
            "structural_wall_legacy_used_as_hard_gateway_blocker": data.get("structural_wall_legacy_used_as_hard_gateway_blocker"),
            "candidate_count": data.get("candidate_count"),
            "candidate_counts_by_pair": data.get("candidate_counts_by_pair"),
            "pair_count": len(data.get("candidate_counts_by_pair") or {}),
        }
    if paths["step30a_hypotheses"].exists():
        data = read_json(paths["step30a_hypotheses"])
        out["hypothesis_generation"] = {
            "artifact_type": data.get("artifact_type"),
            "gateway_filter_wall_layer": data.get("gateway_filter_wall_layer"),
            "segmentation_wall_processed_used_as_hard_gateway_blocker": data.get("segmentation_wall_processed_used_as_hard_gateway_blocker"),
            "topology_generated": data.get("topology_generated"),
            "hypothesis_count": len(data.get("hypotheses") or []),
            "pair_keys": sorted({h.get("pair_key") for h in data.get("hypotheses", []) if h.get("pair_key")}),
        }
    if paths["step30a_hypotheses_with_roles"].exists():
        data = read_json(paths["step30a_hypotheses_with_roles"])
        roles = {}
        role_sources = {}
        calibration_labels = {}
        for h in data.get("hypotheses", []):
            roles[h.get("route_role", "missing")] = roles.get(h.get("route_role", "missing"), 0) + 1
            role_sources[h.get("role_source", "missing")] = role_sources.get(h.get("role_source", "missing"), 0) + 1
            calibration_labels[h.get("calibration_label", "missing")] = calibration_labels.get(h.get("calibration_label", "missing"), 0) + 1
        out["role_annotation_risk"] = {
            "roles": roles,
            "role_sources": role_sources,
            "calibration_labels": calibration_labels,
            "interpretation": "Step30A role annotations include positive_control labels; these are risky if reused as automatic selection, but Step30B2 selection artifacts report they were not used for truth-blind selection.",
        }
    if paths["step30b2_selection"].exists():
        data = read_json(paths["step30b2_selection"])
        selected_pairs = {}
        rejected_pairs = []
        for pair, item in (data.get("per_pair") or {}).items():
            if item.get("selected_primary_candidate_id"):
                selected_pairs[pair] = item.get("selected_primary_candidate_id")
            else:
                rejected_pairs.append(pair)
        out["truth_blind_selection"] = {
            "artifact_type": data.get("artifact_type"),
            "truth_blind": data.get("truth_blind"),
            "truth_used_for_selection": data.get("truth_used_for_selection"),
            "manual_truth_fields_used": data.get("manual_truth_fields_used"),
            "selection_policy": data.get("selection_policy"),
            "selected_pairs": selected_pairs,
            "rejected_pairs": sorted(rejected_pairs),
        }
    if paths["step30b2_truth"].exists():
        data = read_json(paths["step30b2_truth"])
        out["truth_benchmark"] = {
            "artifact_type": data.get("artifact_type"),
            "source": data.get("source"),
            "used_for_auto_selection": data.get("used_for_auto_selection"),
            "positive_pairs": data.get("positive_pairs"),
            "negative_sanity_pairs": data.get("negative_sanity_pairs"),
            "non_truth_pairs": data.get("non_truth_pairs"),
        }
    if paths["step30b2_eval"].exists():
        data = read_json(paths["step30b2_eval"])
        out["auto_vs_truth_evaluation"] = {
            "artifact_type": data.get("artifact_type"),
            "truth_used_for_selection": data.get("truth_used_for_selection"),
            "truth_file_read_only_after_auto_selection_written": data.get("truth_file_read_only_after_auto_selection_written"),
            "summary": data.get("summary"),
        }
    if paths["current_public_truth_blind_selection"].exists():
        data = read_json(paths["current_public_truth_blind_selection"])
        out["public_gateway_truth_blind_selection_file_audit"] = {
            "path": rel(paths["current_public_truth_blind_selection"]),
            "artifact_type": data.get("artifact_type"),
            "looks_like_truth_benchmark_not_selection": bool(data.get("artifact_type") == "step30b2_candidate_level_gateway_truth_benchmark" or "positive_pairs" in data),
            "recommendation": "Rename or replace this public summary with the actual Step30B2 auto selection to avoid implying truth labels are the selector.",
        }
    return out


def classify_logic(artifact_summary: dict[str, Any], constants: list[dict[str, Any]], scan: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    categories: dict[str, list[dict[str, Any]]] = {
        "generalizable_geometry_semantic_logic": [],
        "scene_specific_configuration_but_acceptable_if_externalized": [],
        "validation_only_truth_labels": [],
        "risky_target_specific_optimization": [],
        "hard_coded_non_generalizable_hack": [],
        "unknown_needs_review": [],
    }
    gen = artifact_summary.get("candidate_generation", {})
    hyp = artifact_summary.get("hypothesis_generation", {})
    if gen:
        categories["generalizable_geometry_semantic_logic"].append({
            "evidence": "Step30A candidate artifact reports gateway_filter_wall_layer and candidate_counts_by_pair.",
            "details": gen,
        })
    if hyp:
        categories["generalizable_geometry_semantic_logic"].append({
            "evidence": "Step30A hypotheses are generated over discovered pair keys, with gateway_wall_preclose as the filter wall layer.",
            "details": hyp,
        })
    truth = artifact_summary.get("truth_benchmark")
    if truth:
        categories["validation_only_truth_labels"].append({"evidence": "Candidate-level truth artifact marks used_for_auto_selection false.", "details": truth})
    eval_record = artifact_summary.get("auto_vs_truth_evaluation")
    if eval_record:
        categories["validation_only_truth_labels"].append({"evidence": "Auto-vs-truth evaluation says truth was read only after selection was written.", "details": eval_record})
    role_risk = artifact_summary.get("role_annotation_risk")
    if role_risk:
        categories["risky_target_specific_optimization"].append({"evidence": "Stage30A role/calibration annotations include positive_control labels.", "details": role_risk})
    public_file = artifact_summary.get("public_gateway_truth_blind_selection_file_audit")
    if public_file and public_file.get("looks_like_truth_benchmark_not_selection"):
        categories["risky_target_specific_optimization"].append({"evidence": "Current public gateway_truth_blind_selection_v0_1.json is actually the truth benchmark artifact.", "details": public_file})
    for c in constants:
        cls = c.get("classification")
        if cls == "map_projection_specific_risky_if_treated_as_general":
            categories["hard_coded_non_generalizable_hack"].append({"evidence": f"{c['name']} is hard-coded in a map projection script.", "details": c})
        elif cls in {"scene_route_demo_specific_not_gateway_extraction", "artifact_restoration_selection_specific"}:
            categories["scene_specific_configuration_but_acceptable_if_externalized"].append({"evidence": f"{c['name']} is scene-specific outside core gateway extraction.", "details": c})
        else:
            categories["unknown_needs_review"].append({"evidence": f"{c['name']} constant needs manual review.", "details": c})
    if scan["pattern_counts"].get("hard_coded_scene_id", 0):
        categories["scene_specific_configuration_but_acceptable_if_externalized"].append({
            "evidence": "Many current tools/artifacts are named for scene 00824; this is acceptable for a frozen milestone but not for reusable extraction code.",
            "details": {"count": scan["pattern_counts"]["hard_coded_scene_id"]},
        })
    return categories


def cross_scene_probe(backup_root: Path) -> dict[str, Any]:
    requested = ["00843", "00829", "00862"]
    scene_dirs: dict[str, list[str]] = {}
    gateway_like: dict[str, list[str]] = {}
    for scene in requested:
        dirs = [p for p in backup_root.rglob(f"{scene}*") if p.is_dir()]
        scene_dirs[scene] = [p.as_posix() for p in dirs[:20]]
        files = [p for p in backup_root.rglob(f"*{scene}*") if p.is_file() and "gateway" in p.name.lower()]
        gateway_like[scene] = [p.as_posix() for p in files[:20]]
    reusable_extractor_scripts = [
        p for p in (REPO_ROOT / "tools").rglob("*gateway*.py")
        if "stage1_step30p1" not in p.as_posix() and "__pycache__" not in p.as_posix()
    ]
    can_run = False
    blockers = []
    if not any(gateway_like.values()):
        blockers.append("No held-out 00843/00829/00862 Step30A/Step30B gateway extraction artifacts were found in the backup root.")
    if not reusable_extractor_scripts:
        blockers.append("No current reusable, scene-parameterized gateway extraction CLI was found; available current tools are 00824 Step30P1 support/restoration/validation tools.")
    else:
        blockers.append("Gateway-related scripts found are historical/support scripts and require 00824-specific filenames or frozen StepXX context.")
    return {
        "artifact_type": "step30s6_cross_scene_gateway_generalization_probe",
        "version": "v0_1",
        "created_utc": now_iso(),
        "requested_scenes": requested,
        "scene_directories_found": scene_dirs,
        "gateway_artifacts_found": gateway_like,
        "reusable_extractor_scripts_found": [rel(p) for p in reusable_extractor_scripts],
        "non_destructive_probe_run": False,
        "could_run_gateway_extraction_on_heldout_scene": can_run,
        "blockers": blockers,
        "conclusion": "Held-out scene assets exist for other steps, but no comparable gateway extraction inputs/CLI are available for a lightweight non-destructive run.",
    }


def required_answers(classification: dict[str, list[dict[str, Any]]], artifact_summary: dict[str, Any]) -> dict[str, str]:
    selection = artifact_summary.get("truth_blind_selection", {})
    generation = artifact_summary.get("candidate_generation", {})
    truth = artifact_summary.get("truth_benchmark", {})
    hardcoded = classification["hard_coded_non_generalizable_hack"]
    risky = classification["risky_target_specific_optimization"]
    return {
        "are_gateway_candidates_generated_from_geometry_or_hard_coded_edges": "Artifacts support geometry/semantic generation from room-pair candidates and gateway_wall_preclose, not direct hard-coded known-edge insertion. However, candidate IDs and selected route summaries are scene-specific artifacts.",
        "is_gateway_wall_preclose_used_correctly_and_generally": f"For 00824, yes: candidate generation reports gateway_filter_wall_layer={generation.get('gateway_filter_wall_layer')} and segmentation_wall_processed_used_as_hard_gateway_blocker={generation.get('segmentation_wall_processed_used_as_hard_gateway_blocker')}. Generality depends on reusing the same automatic thresholding policy on new scenes.",
        "are_truth_labels_used_only_for_evaluation": f"Step30B2 selection says truth_blind={selection.get('truth_blind')} and truth_used_for_selection={selection.get('truth_used_for_selection')}; truth benchmark says used_for_auto_selection={truth.get('used_for_auto_selection')}. This supports validation-only use, with the caveat that Step30A role annotations include positive_control labels.",
        "do_forbidden_shortcuts_influence_extraction": "In gateway extraction artifacts, forbidden/non-truth pairs are evaluated/rejected rather than inserted as positives. In H8R2 map projection code, forbidden pairs are hard-coded to avoid carving shortcuts; that is validation/projection policy, not extraction, and must be externalized before generalization claims.",
        "are_thresholds_documented_and_scene_independent": "Thresholds are documented in artifacts, but there is no evidence of multi-scene calibration. Treat them as scene-tested rather than scene-independent.",
        "is_there_room15_or_00824_specific_rule": "Yes in support/restoration/route/map projection code and artifact names. The core Step30B2 selection artifact does not show a room15 special case, but the surrounding pipeline is not scene-parameterized.",
        "would_method_run_on_another_scene_without_editing_code": "No, not as currently packaged. Current active support tools expect 00824 filenames and selected gateway IDs; no reusable held-out scene extractor CLI was found.",
        "what_must_change_to_make_generalizable": "Parameterize scene id, room ids, gateway ids, route rooms, forbidden edges, and thresholds into config; keep truth benchmarks separate from selection; expose a scene-agnostic gateway extraction CLI; add held-out scene probes.",
        "hard_coded_known_gateway_ids_or_forbidden_shortcuts": f"Hard-coded constants found: {len(hardcoded)} map/projection or route-support records; risky records: {len(risky)}.",
    }


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Step30S6 Gateway Extraction Generalization Audit",
        "",
        f"Created UTC: `{payload['created_utc']}`",
        "",
        "## Conclusion",
        "",
        payload["conclusion"],
        "",
        "## Required Answers",
        "",
    ]
    for key, value in payload["required_answers"].items():
        lines.append(f"- `{key}`: {value}")
    lines.extend(["", "## Classification Counts", ""])
    for key, items in payload["classification"].items():
        lines.append(f"- `{key}`: `{len(items)}`")
    lines.extend(["", "## Notable Recommendations", ""])
    for item in payload["recommendations"]:
        lines.append(f"- {item}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_cross_scene_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Step30S6 Cross-Scene Gateway Generalization Probe",
        "",
        f"Could run held-out gateway extraction: `{payload['could_run_gateway_extraction_on_heldout_scene']}`",
        "",
        "## Conclusion",
        "",
        payload["conclusion"],
        "",
        "## Blockers",
        "",
    ]
    for blocker in payload["blockers"]:
        lines.append(f"- {blocker}")
    lines.extend(["", "## Scene Directories Found", ""])
    for scene, dirs in payload["scene_directories_found"].items():
        lines.append(f"- `{scene}`: `{len(dirs)}` directories")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--backup-root", type=Path, default=DEFAULT_BACKUP_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    output_dir = args.output_dir.resolve()
    files = candidate_files(stage_output, args.backup_root)
    scan = scan_text_files(files)
    constants = py_constant_audit(files)
    artifacts = summarize_gateway_artifacts(stage_output)
    classification = classify_logic(artifacts, constants, scan)
    answers = required_answers(classification, artifacts)
    cross_scene = cross_scene_probe(args.backup_root)

    conclusion = (
        "The 00824 gateway selection artifacts support a truth-blind geometry/semantic selection pass, but the packaged pipeline is not yet generalizable: "
        "active support code and H8R2 map projection contain hard-coded 00824 route/gateway/forbidden constants, Step30A role annotations include positive-control labels, "
        "and the current public `gateway_truth_blind_selection_v0_1.json` is actually a truth benchmark artifact. No held-out scene gateway extraction run could be performed from available assets."
    )
    payload = {
        "artifact_type": "step30s6_gateway_extraction_generalization_audit",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": rel(stage_output),
        "backup_root": args.backup_root.as_posix(),
        "files_scanned_count": len(files),
        "text_scan": scan,
        "py_constant_audit": constants,
        "artifact_summary": artifacts,
        "classification": classification,
        "required_answers": answers,
        "conclusion": conclusion,
        "recommendations": [
            "Replace the misleading public gateway_truth_blind_selection_v0_1.json with the actual auto-selection artifact, or rename the truth benchmark file.",
            "Move selected gateway IDs, route room IDs, forbidden pairs, and map-carve gateway lists from code into per-scene config artifacts.",
            "Add a reusable gateway extraction CLI that accepts scene id, layered BEV, room mask, and config paths instead of 00824 filenames.",
            "Run the extraction and selection on at least one held-out scene before claiming gateway generalization.",
        ],
        "commands_run": [
            "python3 tools/stage1_step30p1/audit_stage1_gateway_extraction_generalization.py --stage-output-dir stage_outputs/stage1_00824_step30p1",
        ],
    }

    write_json(output_dir / "gateway_extraction_generalization_audit_v0_1.json", payload)
    write_md(output_dir / "gateway_extraction_generalization_audit_v0_1.md", payload)
    write_json(output_dir / "cross_scene_gateway_generalization_probe_v0_1.json", cross_scene)
    write_cross_scene_md(output_dir / "cross_scene_gateway_generalization_probe_v0_1.md", cross_scene)
    print(json.dumps({"succeeded": True, "output_dir": rel(output_dir), "files_scanned_count": len(files)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
