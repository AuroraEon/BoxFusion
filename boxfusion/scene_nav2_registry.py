"""Generic reader for downstream Gazebo/Nav2 scene registry entries."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping


class SceneNav2RegistryError(RuntimeError):
    """Raised when the downstream scene registry is missing or malformed."""


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise SceneNav2RegistryError(f"Could not read scene Nav2 registry {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise SceneNav2RegistryError(f"Could not parse scene Nav2 registry {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise SceneNav2RegistryError(f"Expected JSON object in scene Nav2 registry {path}")
    return payload


def _resolve_repo_path(repo_root: Path, value: Any) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    path = Path(text)
    return str(path if path.is_absolute() else (repo_root / path).resolve())


@dataclass(frozen=True)
class SceneNav2RegistryEntry:
    """One scene entry from ``runtime_stage1_frozen_evidence/scene_nav2_registry.json``."""

    scene_id: str
    registry_path: Path
    repo_root: Path
    raw_entry: Mapping[str, Any]

    def path(self, key: str) -> Path:
        value = self.raw_entry.get(key)
        if not value:
            raise SceneNav2RegistryError(f"Registry entry {self.scene_id} has no {key!r} path")
        return Path(_resolve_repo_path(self.repo_root, value))

    def optional_path(self, key: str) -> Path | None:
        value = self.raw_entry.get(key)
        if not value:
            return None
        return Path(_resolve_repo_path(self.repo_root, value))

    def as_resolved_dict(self) -> Dict[str, Any]:
        result = dict(self.raw_entry)
        for key, value in list(result.items()):
            if key.endswith(("_dir", "_file", "_json", "_yaml", "_image", "_world", "_params")):
                result[key] = _resolve_repo_path(self.repo_root, value)
        return result


def load_scene_nav2_registry_entry(
    registry_path: Path,
    scene_id: str,
    *,
    repo_root: Path | None = None,
) -> SceneNav2RegistryEntry:
    """Load and resolve a scene entry from the downstream Nav2 registry."""

    registry = Path(registry_path).resolve()
    root = Path(repo_root).resolve() if repo_root is not None else registry.parents[1].resolve()
    payload = _load_json(registry)
    scenes = payload.get("scenes")
    if not isinstance(scenes, dict):
        raise SceneNav2RegistryError(f"Registry {registry} does not contain a scenes object")
    entry = scenes.get(scene_id)
    if not isinstance(entry, dict):
        raise SceneNav2RegistryError(f"Scene {scene_id!r} is not present in registry {registry}")
    return SceneNav2RegistryEntry(
        scene_id=scene_id,
        registry_path=registry,
        repo_root=root,
        raw_entry=entry,
    )
