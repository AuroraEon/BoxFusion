"""Lightweight manifest registry reader for RSLG-SLAM.

The registry resolves files listed by
docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json. It performs only
static path and JSON loading work.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from .common import load_json, normalize_repo_relative, repo_path, resolve_repo_root


INDEX_RELATIVE_PATH = "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json"

CORE_MANIFEST_KEYS = {
    "project_truth_manifest",
    "pipeline_contract_manifest",
    "layer_artifacts_manifest",
    "workspace_policy_manifest",
    "protected_assets_manifest",
    "validated_milestones_manifest",
    "manifest_retention_plan",
}

PLANNING_OR_TEMPORARY_KEY_FRAGMENTS = (
    "cleanup",
    "legacy",
    "git",
    "gitignore",
    "tool_migration",
    "tool_entrypoint",
    "test_plan",
)


@dataclass(frozen=True)
class ManifestReference:
    key: str
    path: str
    absolute_path: Path
    exists: bool
    required: bool
    status: str


class ArtifactRegistry:
    """Read manifest references without running builders or runtime systems."""

    def __init__(self, repo_root: str | Path):
        self.repo_root = resolve_repo_root(repo_root)
        self.index_path = repo_path(self.repo_root, INDEX_RELATIVE_PATH)
        self.index = load_json(self.index_path)

    def manifest_references(self) -> list[ManifestReference]:
        manifests = self.index.get("manifests", {})
        if not isinstance(manifests, Mapping):
            return []
        references = []
        for key, path in sorted(manifests.items()):
            metadata = self._reference_metadata(str(key), str(path))
            absolute_path = repo_path(self.repo_root, path)
            references.append(
                ManifestReference(
                    key=str(key),
                    path=str(path),
                    absolute_path=absolute_path,
                    exists=absolute_path.is_file(),
                    required=metadata["required"],
                    status=metadata["status"],
                )
            )
        return references

    def _reference_metadata(self, key: str, path: str) -> Dict[str, Any]:
        manifests_metadata = self.index.get("manifest_metadata", {})
        if isinstance(manifests_metadata, Mapping):
            entry = manifests_metadata.get(key)
            if isinstance(entry, Mapping):
                status = str(entry.get("status", "core"))
                required = bool(entry.get("required", status not in {"optional", "deprecated", "planning"}))
                return {"required": required, "status": status}

        if key in CORE_MANIFEST_KEYS:
            return {"required": True, "status": "core"}
        if any(fragment in key for fragment in PLANNING_OR_TEMPORARY_KEY_FRAGMENTS):
            return {"required": False, "status": "planning"}
        if "plan" in Path(path).name or "mapping" in Path(path).name:
            return {"required": False, "status": "planning"}
        return {"required": True, "status": "core"}

    def validate_manifest_files(self, *, required: bool = True) -> Dict[str, Any]:
        references = self.manifest_references()
        missing_required = [ref.path for ref in references if ref.required and not ref.exists]
        missing_optional = [ref.path for ref in references if not ref.required and not ref.exists]
        ok = not required or not missing_required
        return {
            "ok": ok,
            "required": required,
            "missing_required": missing_required,
            "missing_optional_or_planning": missing_optional,
            "reference_count": len(references),
        }

    def load_referenced_manifests(self, *, required: bool = True) -> Dict[str, Any]:
        loaded: Dict[str, Any] = {}
        missing_required = []
        for ref in self.manifest_references():
            if not ref.exists:
                if ref.required:
                    missing_required.append(ref.path)
                continue
            loaded[ref.key] = load_json(ref.absolute_path)
        if required and missing_required:
            raise FileNotFoundError(f"Missing required manifest files: {missing_required}")
        return loaded

    def summary(self) -> Dict[str, Any]:
        references = self.manifest_references()
        return {
            "project_name": self.index.get("project_name"),
            "repo_root": self.repo_root.as_posix(),
            "index_path": normalize_repo_relative(self.index_path, self.repo_root),
            "manifest_count": len(references),
            "missing_manifest_count": sum(1 for ref in references if not ref.exists),
            "missing_required_manifest_count": sum(1 for ref in references if ref.required and not ref.exists),
            "manifests": [
                {
                    "key": ref.key,
                    "path": ref.path,
                    "exists": ref.exists,
                    "required": ref.required,
                    "status": ref.status,
                }
                for ref in references
            ],
        }


def load_registry(repo_root: Optional[str | Path] = None) -> ArtifactRegistry:
    return ArtifactRegistry(resolve_repo_root(repo_root))


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize RSLG-SLAM manifest registry references.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--required", action="store_true", help="Fail if referenced manifests are missing.")
    args = parser.parse_args()

    registry = load_registry(args.repo_root)
    validation = registry.validate_manifest_files(required=args.required)
    summary = registry.summary()
    summary["validation"] = validation
    print(json.dumps(summary, indent=2, sort_keys=False))
    return 0 if validation["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
