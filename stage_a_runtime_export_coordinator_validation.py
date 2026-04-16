from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

from boxfusion.ros_publication_diagnostics_server import BoxFusionRosPublicationDiagnosticsBackend
from boxfusion.ros_query_server import BoxFusionRosQueryServerBackend
from boxfusion.runtime_export_coordinator import BoxFusionRuntimeExportCoordinator
from boxfusion.runtime_snapshot import (
    compare_minimal_public_topology_shadow_parity,
    compare_runtime_snapshot_shadow_parity,
)


DEFAULT_ARTIFACT_PATH = Path("codex_acceptance_audit/full_artifact/00843-DYehNKdT76V")
DEFAULT_COORDINATION_ROOT = Path("runtime_export_validation/latest_bundle")


def _payload(result: Any) -> Dict[str, Any]:
    return dict(result.payload.get("payload") or {})


def build_validation_report(*, artifact_path: Path, coordination_root: Path) -> Dict[str, Any]:
    coordinator = BoxFusionRuntimeExportCoordinator(
        source_scene_root=artifact_path,
        coordination_root=coordination_root,
        enable_sidecar_shadow_export=True,
    )
    refresh_result = coordinator.refresh_once()
    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(refresh_result.latest_scene_root)
    diagnostics_backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(refresh_result.latest_scene_root)
    topology_payload = _payload(query_backend.get_topology())
    diagnostics_payload = _payload(diagnostics_backend.get_publication_diagnostics())
    sidecar_payload = {}
    if refresh_result.sidecar_export_result.output_path is not None:
        sidecar_payload = json.loads(refresh_result.sidecar_export_result.output_path.read_text(encoding="utf-8"))
    shadow_parity = compare_runtime_snapshot_shadow_parity(
        authoritative_snapshot=refresh_result.runtime_snapshot,
        sidecar_payload=sidecar_payload,
    )
    public_topology_shadow_parity = compare_minimal_public_topology_shadow_parity(
        authoritative_topology_payload=refresh_result.committed_bundle.topology_payload,
        sidecar_payload=sidecar_payload,
    )
    return {
        "validation_kind": "runtime_export_coordinator",
        "refresh": refresh_result.describe(),
        "shadow_sidecar_parity_check": shadow_parity.to_dict(),
        "shadow_minimal_public_topology_parity_check": public_topology_shadow_parity.to_dict(),
        "query_server_check": {
            "artifact_path": str(refresh_result.latest_scene_root),
            "topology_path": topology_payload.get("topology_path"),
            "room_count": len(dict(topology_payload.get("topology") or {}).get("rooms") or []),
        },
        "publication_diagnostics_check": {
            "artifact_path": str(refresh_result.latest_scene_root),
            "room_count": len(list(diagnostics_payload.get("rooms") or [])),
            "published_room_count": dict(diagnostics_payload.get("summary") or {}).get("published_room_count"),
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate that the runtime/export coordinator exposes a stable latest bundle consumable by the existing ROS backends."
    )
    parser.add_argument(
        "--artifact-path",
        default=str(DEFAULT_ARTIFACT_PATH),
        help="Path to the Stage A scene root that already contains manifest/log artifacts.",
    )
    parser.add_argument(
        "--coordination-root",
        default=str(DEFAULT_COORDINATION_ROOT),
        help="Directory that will host the stable latest pointer for validation.",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write the validation report JSON.",
    )
    args = parser.parse_args(argv)

    report = build_validation_report(
        artifact_path=Path(args.artifact_path),
        coordination_root=Path(args.coordination_root),
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
