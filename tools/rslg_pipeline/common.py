"""Small shared helpers for the RSLG-SLAM pipeline tools."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from .project_truth import HISTORICAL_REPOSITORY_PATH, PROJECT_NAME

DEFAULT_REPO_ROOT = HISTORICAL_REPOSITORY_PATH
MANIFEST_INDEX_PATH = Path("docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json")


def resolve_repo_root(repo_root: Optional[str | Path] = None) -> Path:
    """Resolve the repository root without depending on generated outputs."""
    if repo_root is not None:
        return Path(repo_root).expanduser().resolve()

    start = Path(__file__).resolve()
    for parent in (start, *start.parents):
        if (parent / "docs/rslg_slam").is_dir() and (parent / "tools").is_dir():
            return parent
    if DEFAULT_REPO_ROOT.exists():
        return DEFAULT_REPO_ROOT.resolve()
    return Path.cwd().resolve()


def repo_path(repo_root: str | Path, path: str | Path) -> Path:
    """Return an absolute path under repo_root for relative inputs."""
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (Path(repo_root).expanduser().resolve() / candidate).resolve()


def normalize_repo_relative(path: str | Path, repo_root: str | Path) -> str:
    """Normalize a path as repo-relative when possible."""
    root = Path(repo_root).expanduser().resolve()
    candidate = repo_path(root, path)
    try:
        return candidate.relative_to(root).as_posix()
    except ValueError:
        return candidate.as_posix()


def load_json(path: str | Path) -> Any:
    """Load JSON with UTF-8 text handling."""
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_text(path: str | Path) -> str:
    """Load UTF-8 text for static contract checks."""
    return Path(path).read_text(encoding="utf-8")


def save_json(path: str | Path, data: Any) -> None:
    """Write compact, deterministic JSON for small reports/manifests."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=False)
        handle.write("\n")


def manifest_dir(repo_root: str | Path) -> Path:
    return repo_path(repo_root, "docs/rslg_slam/manifests")


def load_manifest_index(repo_root: str | Path) -> Mapping[str, Any]:
    return load_json(repo_path(repo_root, MANIFEST_INDEX_PATH))


def load_manifest(repo_root: str | Path, manifest_path: str | Path) -> Any:
    return load_json(repo_path(repo_root, manifest_path))


def list_json_files(root: str | Path) -> list[Path]:
    path = Path(root)
    if not path.exists():
        return []
    return sorted(p for p in path.glob("*.json") if p.is_file())


def list_manifest_json_files(repo_root: str | Path) -> list[Path]:
    """List JSON manifests without depending on generated outputs."""
    return list_json_files(manifest_dir(repo_root))


def safe_stat(path: str | Path, repo_root: Optional[str | Path] = None) -> Dict[str, Any]:
    """Return stat metadata without raising if a path is absent."""
    target = repo_path(repo_root, path) if repo_root is not None else Path(path).expanduser()
    exists = target.exists()
    result: Dict[str, Any] = {
        "path": normalize_repo_relative(target, repo_root) if repo_root is not None else target.as_posix(),
        "exists": exists,
    }
    if not exists:
        return result
    stat = target.stat()
    result.update(
        {
            "kind": "directory" if target.is_dir() else "file" if target.is_file() else "other",
            "size_bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
        }
    )
    return result


def placeholder_main(
    *,
    module_name: str,
    current_role: str,
    future_role: str,
    layer: str,
    argv: Optional[Iterable[str]] = None,
) -> int:
    """Shared CLI for task25g placeholder builder modules."""
    parser = argparse.ArgumentParser(
        description=f"{module_name}: RSLG-SLAM placeholder entrypoint for {layer}.",
    )
    parser.add_argument("--repo-root", default=str(DEFAULT_REPO_ROOT), help="Repository root path.")
    parser.add_argument("--describe", action="store_true", help="Describe the placeholder behavior.")
    parser.add_argument("--dry-run", action="store_true", help="Exit successfully without creating artifacts.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    summary = {
        "project_name": PROJECT_NAME,
        "module": module_name,
        "repo_root": repo_root.as_posix(),
        "layer": layer,
        "current_role": current_role,
        "future_role": future_role,
        "business_logic_migrated": False,
        "real_logic_status": "not_migrated_yet",
        "dry_run": bool(args.dry_run),
        "runtime_launched": False,
        "old_scripts_called": False,
        "large_artifacts_written": False,
    }
    print(json.dumps(summary, indent=2, sort_keys=False))
    return 0
