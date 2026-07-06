"""Task48a static guardrails for RSLG-SLAM project truth."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any, Iterable

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.common import resolve_repo_root
from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_ID,
    CURRENT_OBJECT_APPROACH_POSITION,
    CURRENT_OBJECT_APPROACH_YAW,
    FORMAL_COMMAND_SURFACE,
    NON_TRANSITION_EDGE,
    OBJECT_FLOOR,
    OBJECT_ID,
    OBJECT_LABEL,
    OBJECT_QUERY,
    OBJECT_ROOM,
    OFFICIAL_LAYERS,
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
)


FORBIDDEN_CLAIMS = (
    "dense reconstruction",
    "neural implicit SLAM",
    "full embodied navigation benchmark",
    "full BEV planner",
    "AMCL success",
    "LLM runtime system",
    "real robot deployment",
    "collision-free guarantee",
    "osmAG-Nav",
)


def _normalize(text: str) -> str:
    for char in "-_/.,;:()[]{}\"'`":
        text = text.replace(char, " ")
    return " ".join(text.lower().split())


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = _normalize(text).split()
    phrase_words = _normalize(phrase).split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = " ".join(words[max(0, index - 8) : index])
        if any(marker in prefix for marker in ("not", "no", "without", "never", "must not", "do not", "does not")):
            return True
    return False


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8") if path.is_file() else ""


def _line_context(lines: list[str], index: int) -> str:
    start = max(0, index - 12)
    end = min(len(lines), index + 2)
    return " ".join(lines[start:end]).lower()


def _blocked_legacy_context(text: str) -> bool:
    markers = (
        "blocked",
        "legacy",
        "not used",
        "not be used",
        "must not",
        "do not",
        "no ",
        "does not claim",
        "false",
        "context",
        "evidence",
        "occupied",
        "blocker",
        "not selected",
        "comparison",
        "not the object runtime goal",
        "not the runtime goal",
        "preserve",
        "preserved",
    )
    return any(marker in text for marker in markers)


def _formal_text_files(repo_root: Path) -> list[Path]:
    roots = [
        repo_root / "tools/rslg_pipeline",
        repo_root / "docs/rslg_slam_truth",
    ]
    result: list[Path] = []
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in {".py", ".md", ".sh"}:
                result.append(path)
    return sorted(result)


def _relative(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def check_project_truth_constants() -> tuple[bool, list[str]]:
    errors: list[str] = []
    if PROJECT_NAME != "RSLG-SLAM":
        errors.append("PROJECT_NAME must be RSLG-SLAM")
    if CURRENT_OBJECT_APPROACH_ID != "generated_ring_002":
        errors.append("CURRENT_OBJECT_APPROACH_ID must be generated_ring_002")
    if tuple(round(float(v), 6) for v in CURRENT_OBJECT_APPROACH_POSITION) != (-7.020484, 1.558795):
        errors.append("CURRENT_OBJECT_APPROACH_POSITION must be [-7.020484, 1.558795]")
    if round(float(CURRENT_OBJECT_APPROACH_YAW), 5) != -2.09057:
        errors.append("CURRENT_OBJECT_APPROACH_YAW must be -2.09057")
    if "generated_ring_037" not in BLOCKED_LEGACY_APPROACH_IDS:
        errors.append("generated_ring_037 must be listed as blocked legacy evidence")
    if TRUE_TRANSITION_EDGE != "vt_1_centerline_e001":
        errors.append("TRUE_TRANSITION_EDGE must be vt_1_centerline_e001")
    if NON_TRANSITION_EDGE != "vt_1_centerline_e003":
        errors.append("NON_TRANSITION_EDGE must be vt_1_centerline_e003")
    if (OBJECT_QUERY, OBJECT_ID, OBJECT_LABEL, OBJECT_FLOOR, OBJECT_ROOM) != (
        "curtain in room_14 on floor_2",
        "obj_175",
        "curtain",
        "floor_2",
        "room_14",
    ):
        errors.append("object navigation truth constants drifted")
    if "Layer 1: World Model Layer" not in OFFICIAL_LAYERS:
        errors.append("official layers must include Layer 1: World Model Layer")
    return not errors, errors


def check_generated_ring_037_usage(repo_root: Path) -> tuple[bool, list[str]]:
    errors: list[str] = []
    suspicious_patterns = (
        r"CURRENT_OBJECT_APPROACH_ID\s*=\s*[\"']generated_ring_037[\"']",
        r"APPROACH_CANDIDATE_ID\s*=\s*[\"']generated_ring_037[\"']",
        r"EXPECTED_APPROACH_ID\s*=\s*[\"']generated_ring_037[\"']",
        r"runtime_goal[\"']?\s*:\s*True.*generated_ring_037",
        r"navigation_goal[\"']?\s*:\s*True.*generated_ring_037",
    )
    blocked_id = BLOCKED_LEGACY_APPROACH_IDS[0]
    for path in _formal_text_files(repo_root):
        text = _read(path)
        if not text or blocked_id not in text:
            continue
        lines = text.splitlines()
        for index, line in enumerate(lines):
            if blocked_id not in line:
                continue
            context = _line_context(lines, index)
            if any(re.search(pattern, line) for pattern in suspicious_patterns):
                errors.append(f"{_relative(path, repo_root)}:{index + 1}: {line.strip()}")
            elif "generated_ring_037" in line and "runtime goal" in line.lower() and not _blocked_legacy_context(context):
                errors.append(f"{_relative(path, repo_root)}:{index + 1}: unclear runtime-goal context")
    return not errors, errors


def check_transition_edges(repo_root: Path) -> tuple[bool, list[str]]:
    errors: list[str] = []
    for path in _formal_text_files(repo_root):
        text = _read(path)
        if "transition_edge" in text and NON_TRANSITION_EDGE in text:
            for index, line in enumerate(text.splitlines(), start=1):
                normalized = _normalize(line)
                if "transition edge" in normalized and NON_TRANSITION_EDGE in line and "not" not in normalized and "non transition" not in normalized:
                    errors.append(f"{_relative(path, repo_root)}:{index}: {NON_TRANSITION_EDGE} appears as transition edge")
    return not errors, errors


def check_layer_naming(repo_root: Path) -> tuple[bool, list[str]]:
    errors: list[str] = []
    for path in [repo_root / "tools/rslg_pipeline" / name for name in ("README.md", "run_layer1_world_model.sh", "run_rslg_pipeline_static.sh")]:
        text = _read(path)
        if "Stage-A Layer" in text:
            errors.append(f"{_relative(path, repo_root)} uses Stage-A Layer wording")
    return not errors, errors


def check_forbidden_claims(repo_root: Path) -> tuple[bool, list[str]]:
    errors: list[str] = []
    docs = [
        repo_root / "tools/rslg_pipeline/README.md",
        repo_root / "docs/rslg_slam_truth/PROJECT_TRUTH.md",
        repo_root / "docs/rslg_slam_truth/LEGACY_00824_REFERENCE.md",
    ]
    for path in docs:
        text = _read(path)
        for phrase in FORBIDDEN_CLAIMS:
            for index, line in enumerate(text.splitlines()):
                if _normalize(phrase) not in _normalize(line):
                    continue
                context = _line_context(text.splitlines(), index)
                if (
                    _phrase_is_negated(context, phrase)
                    or "must not claim" in context
                    or "does not claim" in context
                    or "forbidden claims" in context
                    or "explicit non claims" in context
                    or "does not establish" in context
                ):
                    continue
                errors.append(f"{_relative(path, repo_root)}:{index + 1} contains unnegated forbidden claim phrase {phrase!r}")
    return not errors, errors


def check_formal_command_surface(repo_root: Path) -> tuple[bool, list[str]]:
    readme = _read(repo_root / "tools/rslg_pipeline/README.md")
    errors = []
    if FORMAL_COMMAND_SURFACE.as_posix() not in readme:
        errors.append("tools/rslg_pipeline/README.md must name tools/rslg_pipeline as the formal command surface")
    for layer in OFFICIAL_LAYERS:
        if layer not in readme:
            errors.append(f"tools/rslg_pipeline/README.md missing {layer}")
    return not errors, errors


def run_checks(repo_root: Path) -> dict[str, Any]:
    checks = []
    for name, fn in (
        ("project_truth_constants", lambda: check_project_truth_constants()),
        ("generated_ring_037_not_current_goal", lambda: check_generated_ring_037_usage(repo_root)),
        ("transition_edge_truth", lambda: check_transition_edges(repo_root)),
        ("layer_naming", lambda: check_layer_naming(repo_root)),
        ("forbidden_claims", lambda: check_forbidden_claims(repo_root)),
        ("formal_command_surface", lambda: check_formal_command_surface(repo_root)),
    ):
        ok, errors = fn()
        checks.append({"name": name, "ok": ok, "errors": errors})
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": "project_truth_validation_passed" if all(item["ok"] for item in checks) else "project_truth_validation_failed",
        "ok": all(item["ok"] for item in checks),
        "checks": checks,
    }


def main(argv: Iterable[str] | None = None) -> int:
    repo_root = resolve_repo_root(None)
    report = run_checks(repo_root)
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
