from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from boxfusion.ros_publication_diagnostics_server import BoxFusionRosPublicationDiagnosticsBackend


DEFAULT_ARTIFACT_PATHS = [
    Path("codex_acceptance_audit/core_only/00843-DYehNKdT76V"),
    Path("codex_perf_probe/server_preinfer_opt_pass/validation_candidate1_fast_gt_resize300/output/00843-DYehNKdT76V"),
    Path("codex_perf_probe/server_preinfer_opt_pass/validation_candidate2_fast_depth_kth150/output/00843-DYehNKdT76V"),
]


def _response_payload(result: Any) -> Dict[str, Any]:
    return dict(result.payload.get("payload") or {})


def _scene_report(artifact_path: Path) -> Dict[str, Any]:
    backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(artifact_path)
    diagnostics = _response_payload(backend.get_publication_diagnostics())
    room_examples: List[Dict[str, Any]] = []
    for room in list(diagnostics.get("rooms") or []):
        room_examples.append(
            {
                "room_id": room.get("room_id"),
                "publication_state": room.get("publication_state"),
                "finalization_blockers": room.get("finalization_blockers"),
                "publication_blockers": room.get("publication_blockers"),
                "published": room.get("published"),
                "pre_publication_only": room.get("pre_publication_only"),
            }
        )
    return {
        "artifact_path": str(Path(artifact_path).resolve()),
        "bundle": backend.bundle.describe(),
        "summary": diagnostics.get("summary"),
        "room_examples": room_examples,
    }


def build_validation_report(artifact_paths: Sequence[Path]) -> Dict[str, Any]:
    return {
        "validation_kind": "ros_publication_diagnostics_backend",
        "scenes": [_scene_report(Path(path)) for path in artifact_paths],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate the debug ROS publication diagnostics backend on saved lifecycle artifacts."
    )
    parser.add_argument(
        "--artifact-path",
        action="append",
        default=[],
        help="Scene root, manifest.json, logs/summary.json, logs/online_topology_lifecycle_v0_1.json, or logs/topology_v0_1.json. May be passed more than once.",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write the validation report JSON.",
    )
    args = parser.parse_args(argv)

    artifact_paths = [Path(item) for item in args.artifact_path] or list(DEFAULT_ARTIFACT_PATHS)
    report = build_validation_report(artifact_paths)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
