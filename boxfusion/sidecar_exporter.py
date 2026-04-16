from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Protocol, Sequence

from boxfusion.runtime_snapshot import (
    RUNTIME_SNAPSHOT_SHADOW_PARITY_SCOPE,
    RuntimeStateSnapshot,
)


SIDECAR_EXPORTER_CONTRACT_VERSION = "0.1"


@dataclass(frozen=True)
class SidecarExportResult:
    enabled: bool
    mode: str
    output_path: Optional[Path]
    materialized_keys: Sequence[str]
    authoritative: bool
    message: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "contract_version": SIDECAR_EXPORTER_CONTRACT_VERSION,
            "enabled": bool(self.enabled),
            "mode": self.mode,
            "output_path": None if self.output_path is None else str(self.output_path),
            "materialized_keys": [str(item) for item in self.materialized_keys],
            "authoritative": bool(self.authoritative),
            "message": self.message,
        }


class RuntimeSnapshotSidecarExporter(Protocol):
    def export(self, snapshot: RuntimeStateSnapshot) -> SidecarExportResult:
        ...


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp")
    tmp_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp_path, path)


class SnapshotMetadataSidecarExporter:
    def __init__(
        self,
        *,
        output_dir: Path,
        mode: str = "shadow_minimal_subset",
        output_name: str = "runtime_snapshot_sidecar_shadow_subset_v0_1.json",
    ) -> None:
        self.output_dir = Path(output_dir)
        self.mode = str(mode)
        self.output_name = str(output_name)

    def export(self, snapshot: RuntimeStateSnapshot) -> SidecarExportResult:
        snapshot_payload = snapshot.to_dict()
        runtime_state = dict(snapshot_payload.get("runtime_maintained_state") or {})
        public_surface = dict(snapshot_payload.get("committed_public_export_surface") or {})
        lifecycle_surface = dict(snapshot_payload.get("lifecycle_debug_surface") or {})
        minimal_topology_subset = dict(snapshot_payload.get("minimal_public_topology_subset") or {})
        output_path = self.output_dir / self.output_name
        materialized_keys = [
            "minimal_runtime_state_summary",
            "minimal_public_topology_summary",
            "minimal_committed_public_topology_subset",
            "minimal_lifecycle_debug_linkage",
        ]
        payload = {
            "contract_version": SIDECAR_EXPORTER_CONTRACT_VERSION,
            "sidecar_mode": self.mode,
            "authoritative": False,
            "snapshot_contract_version": snapshot_payload.get("contract_version"),
            "source_scene_root": snapshot_payload.get("source_scene_root"),
            "created_at_utc": snapshot_payload.get("created_at_utc"),
            "materialized_keys": materialized_keys,
            "shadow_materialization": {
                "artifact_kind": "runtime_snapshot_shadow_minimal_export_subset",
                "scope": RUNTIME_SNAPSHOT_SHADOW_PARITY_SCOPE,
                "authoritative": False,
                "snapshot_contract_version": snapshot_payload.get("contract_version"),
                "runtime_state": {
                    "sequence_id": runtime_state.get("sequence_id"),
                    "frame_idx": runtime_state.get("frame_idx"),
                    "snapshot_count": runtime_state.get("snapshot_count"),
                    "segmentation_cycle_count": runtime_state.get("segmentation_cycle_count"),
                    "final_floor_count": runtime_state.get("final_floor_count"),
                    "final_room_count": runtime_state.get("final_room_count"),
                    "final_object_count": runtime_state.get("final_object_count"),
                    "final_anchor_count": runtime_state.get("final_anchor_count"),
                    "final_floor_ids": list(runtime_state.get("final_floor_ids") or []),
                    "final_room_ids": list(runtime_state.get("final_room_ids") or []),
                    "final_object_ids": list(runtime_state.get("final_object_ids") or []),
                    "final_anchor_ids": list(runtime_state.get("final_anchor_ids") or []),
                },
                "public_topology": {
                    "topology_path": public_surface.get("topology_path"),
                    "topology_semantics": public_surface.get("topology_semantics"),
                    "public_topology_meaning": public_surface.get("public_topology_meaning"),
                    "floor_count": public_surface.get("floor_count"),
                    "room_count": public_surface.get("room_count"),
                    "edge_count": public_surface.get("edge_count"),
                    "object_count": public_surface.get("object_count"),
                    "anchor_count": public_surface.get("anchor_count"),
                    "floor_ids": list(public_surface.get("floor_ids") or []),
                    "room_ids": list(public_surface.get("room_ids") or []),
                    "object_ids": list(public_surface.get("object_ids") or []),
                    "anchor_ids": list(public_surface.get("anchor_ids") or []),
                },
                "minimal_public_topology_subset": minimal_topology_subset,
                "lifecycle_debug": {
                    "lifecycle_path": lifecycle_surface.get("lifecycle_path"),
                    "committed_room_count": lifecycle_surface.get("committed_room_count"),
                    "non_published_room_count": lifecycle_surface.get("non_published_room_count"),
                    "publication_state_counts": dict(lifecycle_surface.get("publication_state_counts") or {}),
                    "committed_room_ids": list(lifecycle_surface.get("committed_room_ids") or []),
                    "non_published_room_ids": list(lifecycle_surface.get("non_published_room_ids") or []),
                    "non_published_rooms_public": lifecycle_surface.get("non_published_rooms_public"),
                },
            },
            "notes": [
                "Sidecar output is a shadow-only minimal parity subset.",
                "The existing synchronous exporter remains authoritative.",
                "The first real-export target is limited to committed/public floors, rooms, edge summaries, and object-room memberships.",
                "This file does not build vector maps, anchors, scene graphs, GraphML, visual artifacts, or rich diagnostics.",
                "Mismatches against the synchronous export are diagnostic only and not consumer-facing contract changes.",
            ],
        }
        _atomic_write_json(output_path, payload)
        return SidecarExportResult(
            enabled=True,
            mode=self.mode,
            output_path=output_path,
            materialized_keys=tuple(materialized_keys),
            authoritative=False,
            message="Wrote shadow sidecar minimal parity subset from the runtime snapshot.",
        )


class DisabledSidecarExporter:
    def export(self, snapshot: RuntimeStateSnapshot) -> SidecarExportResult:
        del snapshot
        return SidecarExportResult(
            enabled=False,
            mode="disabled",
            output_path=None,
            materialized_keys=(),
            authoritative=False,
            message="Sidecar export disabled; synchronous export path remains authoritative.",
        )
