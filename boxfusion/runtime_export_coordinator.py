from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from boxfusion.backend_eval_scaffold import write_scene_manifest
from boxfusion.runtime_artifact_policy import (
    RUNTIME_ARTIFACT_MODE_BENCHMARK,
    RUNTIME_ARTIFACT_MODES,
    resolve_runtime_artifact_policy,
)
from boxfusion.runtime_snapshot import RuntimeStateSnapshot, build_runtime_snapshot_from_bundles
from boxfusion.ros_publication_diagnostics_server import (
    PublicationDiagnosticsBundle,
    load_publication_diagnostics_bundle,
)
from boxfusion.ros_query_server import CommittedPublicBundle, load_committed_public_bundle
from boxfusion.sidecar_exporter import (
    DisabledSidecarExporter,
    SidecarExportResult,
    SnapshotMetadataSidecarExporter,
)


DEFAULT_REFRESH_INTERVAL_SEC = 30.0


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _clean_optional_path(value: Any) -> Optional[Path]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    return Path(text)


def _parse_bool_token(value: Any) -> bool:
    if isinstance(value, bool):
        return bool(value)
    text = str(value or "").strip().lower()
    return text in {"1", "true", "yes", "on"}


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp")
    tmp_path.write_text(text, encoding="utf-8")
    os.replace(tmp_path, path)


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    _atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _atomic_replace_symlink(link_path: Path, target_path: Path) -> None:
    link_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_link = link_path.with_name(f".{link_path.name}.tmp")
    try:
        if tmp_link.exists() or tmp_link.is_symlink():
            tmp_link.unlink()
        os.symlink(str(target_path), str(tmp_link))
        os.replace(tmp_link, link_path)
    finally:
        if tmp_link.exists() or tmp_link.is_symlink():
            tmp_link.unlink()


@dataclass
class ExportRefreshResult:
    coordination_root: Path
    source_scene_root: Path
    manifest_path: Path
    latest_scene_root: Path
    latest_manifest_path: Path
    latest_export_metadata_path: Path
    refresh_record_path: Path
    committed_bundle: CommittedPublicBundle
    diagnostics_bundle: PublicationDiagnosticsBundle
    runtime_snapshot: RuntimeStateSnapshot
    sidecar_export_result: SidecarExportResult
    producer_command: Optional[str]
    producer_cwd: Optional[Path]
    refreshed_at_utc: str

    def describe(self) -> Dict[str, Any]:
        return {
            "refreshed_at_utc": self.refreshed_at_utc,
            "coordination_root": str(self.coordination_root),
            "source_scene_root": str(self.source_scene_root),
            "manifest_path": str(self.manifest_path),
            "latest_scene_root": str(self.latest_scene_root),
            "latest_manifest_path": str(self.latest_manifest_path),
            "latest_export_metadata_path": str(self.latest_export_metadata_path),
            "refresh_record_path": str(self.refresh_record_path),
            "producer_command": self.producer_command,
            "producer_cwd": None if self.producer_cwd is None else str(self.producer_cwd),
            "committed_public": self.committed_bundle.describe(),
            "lifecycle_debug": self.diagnostics_bundle.describe(),
            "runtime_snapshot": self.runtime_snapshot.to_dict(),
            "sidecar_export": self.sidecar_export_result.to_dict(),
            "consumer_inputs": {
                "query_server_artifact_path": str(self.latest_scene_root),
                "query_server_manifest_path": str(self.latest_manifest_path),
                "publication_diagnostics_artifact_path": str(self.latest_scene_root),
                "publication_diagnostics_manifest_path": str(self.latest_manifest_path),
            },
        }


class BoxFusionRuntimeExportCoordinator:
    def __init__(
        self,
        *,
        source_scene_root: Path,
        coordination_root: Path,
        dataset_root: Optional[Path] = None,
        sequence_name: Optional[str] = None,
        producer_command: Optional[str] = None,
        producer_cwd: Optional[Path] = None,
        refresh_interval_sec: float = DEFAULT_REFRESH_INTERVAL_SEC,
        runtime_artifact_mode: str = RUNTIME_ARTIFACT_MODE_BENCHMARK,
        service_mode: bool = False,
        enable_sidecar_shadow_export: bool = False,
        sidecar_output_dir: Optional[Path] = None,
    ) -> None:
        self.source_scene_root = Path(source_scene_root).resolve()
        self.coordination_root = Path(coordination_root).resolve()
        self.dataset_root = None if dataset_root is None else Path(dataset_root).resolve()
        self.sequence_name = _clean_optional_text(sequence_name) or self.source_scene_root.name
        self.producer_command = _clean_optional_text(producer_command)
        self.producer_cwd = None if producer_cwd is None else Path(producer_cwd).resolve()
        self.refresh_interval_sec = float(refresh_interval_sec)
        self.artifact_policy = resolve_runtime_artifact_policy(
            mode=runtime_artifact_mode,
            service_mode=service_mode,
        )
        self.enable_sidecar_shadow_export = bool(enable_sidecar_shadow_export)
        self.sidecar_output_dir = None if sidecar_output_dir is None else Path(sidecar_output_dir).resolve()

    @property
    def latest_scene_root(self) -> Path:
        return self.coordination_root / "latest"

    @property
    def latest_manifest_path(self) -> Path:
        return self.coordination_root / "latest_manifest.json"

    @property
    def latest_export_metadata_path(self) -> Path:
        return self.coordination_root / "latest_export.json"

    @property
    def refresh_history_dir(self) -> Path:
        return self.coordination_root / "refresh_history"

    @property
    def default_sidecar_output_dir(self) -> Path:
        return self.coordination_root / "sidecar_shadow"

    def _run_producer_command(self) -> None:
        if self.producer_command is None:
            return
        subprocess.run(
            shlex.split(self.producer_command),
            cwd=None if self.producer_cwd is None else str(self.producer_cwd),
            check=True,
        )

    def _refresh_manifest(self) -> Path:
        if not self.source_scene_root.exists():
            raise FileNotFoundError(f"Source scene root does not exist: {self.source_scene_root}")
        write_scene_manifest(
            self.source_scene_root,
            dataset_root=self.dataset_root,
            sequence_name=self.sequence_name,
        )
        manifest_path = self.source_scene_root / "manifest.json"
        if not manifest_path.exists():
            raise FileNotFoundError(f"Manifest refresh did not produce manifest.json: {manifest_path}")
        return manifest_path.resolve()

    def _build_refresh_payload(
        self,
        *,
        manifest_path: Path,
        committed_bundle: CommittedPublicBundle,
        diagnostics_bundle: PublicationDiagnosticsBundle,
        runtime_snapshot: RuntimeStateSnapshot,
        sidecar_export_result: SidecarExportResult,
        refreshed_at_utc: str,
    ) -> Dict[str, Any]:
        return {
            "version": "0.1",
            "refreshed_at_utc": refreshed_at_utc,
            "coordinator": {
                "source_scene_root": str(self.source_scene_root),
                "coordination_root": str(self.coordination_root),
                "sequence_name": self.sequence_name,
                "dataset_root": None if self.dataset_root is None else str(self.dataset_root),
                "refresh_interval_sec": self.refresh_interval_sec,
                "producer_command": self.producer_command,
                "producer_cwd": None if self.producer_cwd is None else str(self.producer_cwd),
                "snapshot_semantics": "periodic_latest_pointer",
            },
            "runtime_artifact_policy": self.artifact_policy.to_dict(),
            "stable_inputs": {
                "latest_scene_root": str(self.latest_scene_root),
                "latest_manifest_path": str(self.latest_manifest_path),
                "source_manifest_path": str(manifest_path),
            },
            "consumer_inputs": {
                "query_server_artifact_path": str(self.latest_scene_root),
                "query_server_manifest_path": str(self.latest_manifest_path),
                "publication_diagnostics_artifact_path": str(self.latest_scene_root),
                "publication_diagnostics_manifest_path": str(self.latest_manifest_path),
            },
            "committed_public": {
                "artifact_path": str(self.latest_scene_root),
                "manifest_path": str(self.latest_manifest_path),
                "source_topology_path": str(committed_bundle.topology_path),
                "surface": "public",
                "bundle": committed_bundle.describe(),
            },
            "lifecycle_debug": {
                "artifact_path": str(self.latest_scene_root),
                "manifest_path": str(self.latest_manifest_path),
                "source_lifecycle_path": str(diagnostics_bundle.lifecycle_path),
                "surface": "lifecycle",
                "bundle": diagnostics_bundle.describe(),
            },
            "runtime_snapshot": runtime_snapshot.to_dict(),
            "sidecar_export": sidecar_export_result.to_dict(),
            "authoritative_export_path": {
                "mode": "synchronous_current_path",
                "sidecar_replaces_authoritative_export": False,
                "scene_root": str(self.latest_scene_root),
                "manifest_path": str(self.latest_manifest_path),
            },
            "notes": [
                "The coordinator does not broaden topology semantics; public topology remains committed/published only.",
                "Non-PUBLISHED rooms remain available only through lifecycle/debug artifacts.",
                "The latest pointer targets the current Stage A scene root rather than introducing replay or incremental publication.",
                "Sidecar export is a shadow-only minimal parity subset and does not replace the current exporter.",
            ],
        }

    def _export_sidecar_snapshot(self, runtime_snapshot: RuntimeStateSnapshot) -> SidecarExportResult:
        if not self.enable_sidecar_shadow_export:
            return DisabledSidecarExporter().export(runtime_snapshot)
        output_dir = self.sidecar_output_dir or self.default_sidecar_output_dir
        return SnapshotMetadataSidecarExporter(output_dir=output_dir).export(runtime_snapshot)

    def refresh_once(self) -> ExportRefreshResult:
        self._run_producer_command()
        manifest_path = self._refresh_manifest()
        committed_bundle = load_committed_public_bundle(manifest_path)
        diagnostics_bundle = load_publication_diagnostics_bundle(manifest_path)
        refreshed_at_utc = _utc_now_iso()
        runtime_snapshot = build_runtime_snapshot_from_bundles(
            committed_bundle=committed_bundle,
            diagnostics_bundle=diagnostics_bundle,
            created_at_utc=refreshed_at_utc,
        )
        sidecar_export_result = self._export_sidecar_snapshot(runtime_snapshot)

        _atomic_replace_symlink(self.latest_scene_root, self.source_scene_root)
        _atomic_replace_symlink(self.latest_manifest_path, manifest_path)

        payload = self._build_refresh_payload(
            manifest_path=manifest_path,
            committed_bundle=committed_bundle,
            diagnostics_bundle=diagnostics_bundle,
            runtime_snapshot=runtime_snapshot,
            sidecar_export_result=sidecar_export_result,
            refreshed_at_utc=refreshed_at_utc,
        )
        timestamp_slug = refreshed_at_utc.replace("+00:00", "Z").replace(":", "")
        refresh_record_path = self.refresh_history_dir / f"{timestamp_slug}.json"
        _atomic_write_json(refresh_record_path, payload)
        _atomic_write_json(self.latest_export_metadata_path, payload)

        return ExportRefreshResult(
            coordination_root=self.coordination_root,
            source_scene_root=self.source_scene_root,
            manifest_path=manifest_path,
            latest_scene_root=self.latest_scene_root,
            latest_manifest_path=self.latest_manifest_path,
            latest_export_metadata_path=self.latest_export_metadata_path,
            refresh_record_path=refresh_record_path,
            committed_bundle=committed_bundle,
            diagnostics_bundle=diagnostics_bundle,
            runtime_snapshot=runtime_snapshot,
            sidecar_export_result=sidecar_export_result,
            producer_command=self.producer_command,
            producer_cwd=self.producer_cwd,
            refreshed_at_utc=refreshed_at_utc,
        )

    def run(
        self,
        *,
        run_once: bool = False,
        max_cycles: Optional[int] = None,
    ) -> ExportRefreshResult:
        if max_cycles is not None and int(max_cycles) <= 0:
            raise ValueError("max_cycles must be positive when provided.")
        cycle_count = 0
        last_result: Optional[ExportRefreshResult] = None
        while True:
            last_result = self.refresh_once()
            cycle_count += 1
            if run_once or (max_cycles is not None and cycle_count >= int(max_cycles)):
                return last_result
            time.sleep(max(self.refresh_interval_sec, 0.0))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Periodically refresh a stable latest-pointer for committed/public and lifecycle/debug Stage A exports."
    )
    parser.add_argument(
        "--source-scene-root",
        required=True,
        help="Path to the Stage A scene root that already contains or will contain manifest/log artifacts.",
    )
    parser.add_argument(
        "--coordination-root",
        required=True,
        help="Directory that will host the stable latest pointer and refresh metadata.",
    )
    parser.add_argument(
        "--dataset-root",
        default=None,
        help="Optional dataset root passed through when refreshing manifest.json.",
    )
    parser.add_argument(
        "--sequence-name",
        default=None,
        help="Optional sequence name override for manifest refresh.",
    )
    parser.add_argument(
        "--producer-command",
        default=None,
        help="Optional command to run before each refresh, e.g. a stage_a_demo.py invocation.",
    )
    parser.add_argument(
        "--producer-cwd",
        default=None,
        help="Optional working directory for --producer-command.",
    )
    parser.add_argument(
        "--refresh-interval-sec",
        type=float,
        default=DEFAULT_REFRESH_INTERVAL_SEC,
        help="Sleep duration between refreshes in periodic mode.",
    )
    parser.add_argument(
        "--runtime-artifact-mode",
        choices=RUNTIME_ARTIFACT_MODES,
        default=RUNTIME_ARTIFACT_MODE_BENCHMARK,
        help="Declare whether the producer side is benchmark/debug or service oriented. Service mode records artifact-only work as skipped/deferred.",
    )
    parser.add_argument(
        "--service-mode",
        action="store_true",
        help="Alias for --runtime-artifact-mode service.",
    )
    parser.add_argument(
        "--enable-sidecar-shadow-export",
        nargs="?",
        const="true",
        default="false",
        help="Write a shadow sidecar minimal parity subset from the runtime snapshot. Does not replace the synchronous export path.",
    )
    parser.add_argument(
        "--sidecar-output-dir",
        default=None,
        help="Optional output directory for --enable-sidecar-shadow-export. Defaults under coordination_root/sidecar_shadow.",
    )
    parser.add_argument(
        "--run-once",
        action="store_true",
        help="Refresh once and exit.",
    )
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=None,
        help="Optional upper bound for periodic refresh loops.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    coordinator = BoxFusionRuntimeExportCoordinator(
        source_scene_root=Path(args.source_scene_root),
        coordination_root=Path(args.coordination_root),
        dataset_root=_clean_optional_path(args.dataset_root),
        sequence_name=args.sequence_name,
        producer_command=args.producer_command,
        producer_cwd=_clean_optional_path(args.producer_cwd),
        refresh_interval_sec=args.refresh_interval_sec,
        runtime_artifact_mode=args.runtime_artifact_mode,
        service_mode=bool(args.service_mode),
        enable_sidecar_shadow_export=_parse_bool_token(args.enable_sidecar_shadow_export),
        sidecar_output_dir=_clean_optional_path(args.sidecar_output_dir),
    )
    result = coordinator.run(
        run_once=bool(args.run_once),
        max_cycles=args.max_cycles,
    )
    print(json.dumps(result.describe(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())
