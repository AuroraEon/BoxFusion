from __future__ import annotations

import argparse
import copy
import importlib
import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from boxfusion.artifact_contract import ARTIFACT_SURFACE_LIFECYCLE
from boxfusion.online_topology_lifecycle import (
    PUBLICATION_STATE_MACHINE_VERSION,
    PUBLICATION_STATE_ORDER,
    derive_publication_diagnostics_from_payload,
)
from boxfusion.ros_query_server import (
    BundleResolutionError,
    ServiceResult,
    _clean_optional_text,
    _discover_candidate_files,
    _infer_artifact_profile,
    _load_json,
    _normalize_search_root_tokens,
    _parse_utc,
    _resolve_artifact_record_path,
    _resolve_path_token,
)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
except ImportError:  # pragma: no cover - exercised only in ROS-enabled environments
    rclpy = None
    Node = None
    remove_ros_args = None


DEFAULT_SERVICE_PREFIX = "/boxfusion/debug/publication"
DEFAULT_SERVICE_MODULE = "ros_interfaces.srv"
DEFAULT_STABILITY_REFRESH_THRESHOLD = 2
REQUIRED_SERVICE_TYPES = (
    "GetPublicationDiagnostics",
    "GetRoomPublicationState",
)


def _normalize_room_id(value: Any) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.startswith("room_"):
        return text
    if text.lstrip("-").isdigit():
        room_index = int(text)
        if room_index < 0:
            return None
        return f"room_{room_index}"
    return text


def _publication_view_for_room(
    room_payload: Dict[str, Any],
    *,
    active_room_id: Optional[Any],
    stability_refresh_threshold: int,
) -> Dict[str, Any]:
    return derive_publication_diagnostics_from_payload(
        dict(room_payload),
        active_room_id=active_room_id,
        stability_refresh_threshold=stability_refresh_threshold,
    )


@dataclass
class PublicationDiagnosticsBundle:
    scene_root: Path
    lifecycle_path: Path
    selection_reason: str
    manifest_path: Optional[Path] = None
    summary_path: Optional[Path] = None
    artifact_profile: Optional[str] = None
    artifact_capabilities: Optional[Dict[str, Any]] = None
    sequence_name: Optional[str] = None
    generated_at_utc: Optional[str] = None
    lifecycle_record: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        self.scene_root = Path(self.scene_root).resolve()
        self.lifecycle_path = Path(self.lifecycle_path).resolve()
        self.manifest_path = None if self.manifest_path is None else Path(self.manifest_path).resolve()
        self.summary_path = None if self.summary_path is None else Path(self.summary_path).resolve()
        self._manifest_payload: Optional[Dict[str, Any]] = None
        self._summary_payload: Optional[Dict[str, Any]] = None
        self._lifecycle_payload: Optional[Dict[str, Any]] = None

    @property
    def manifest_payload(self) -> Optional[Dict[str, Any]]:
        if self.manifest_path is None:
            return None
        if self._manifest_payload is None:
            self._manifest_payload = dict(_load_json(self.manifest_path))
        return self._manifest_payload

    @property
    def summary_payload(self) -> Optional[Dict[str, Any]]:
        if self.summary_path is None or not self.summary_path.exists():
            return None
        if self._summary_payload is None:
            self._summary_payload = dict(_load_json(self.summary_path))
        return self._summary_payload

    @property
    def lifecycle_payload(self) -> Dict[str, Any]:
        if self._lifecycle_payload is None:
            self._lifecycle_payload = dict(_load_json(self.lifecycle_path))
        return self._lifecycle_payload

    def describe(self) -> Dict[str, Any]:
        return {
            "scene_root": str(self.scene_root),
            "selection_reason": self.selection_reason,
            "manifest_path": None if self.manifest_path is None else str(self.manifest_path),
            "summary_path": None if self.summary_path is None else str(self.summary_path),
            "lifecycle_path": str(self.lifecycle_path),
            "sequence_name": self.sequence_name,
            "artifact_profile": self.artifact_profile,
            "artifact_capabilities": dict(self.artifact_capabilities or {}),
            "generated_at_utc": self.generated_at_utc,
            "artifact_surface": None if self.lifecycle_record is None else self.lifecycle_record.get("artifact_surface"),
            "artifact_semantics": None if self.lifecycle_record is None else self.lifecycle_record.get("artifact_semantics"),
        }

    @classmethod
    def from_manifest_path(
        cls,
        manifest_path: Path,
        *,
        selection_reason: str,
    ) -> "PublicationDiagnosticsBundle":
        manifest = dict(_load_json(manifest_path))
        artifacts = dict(manifest.get("artifacts") or {})
        lifecycle_record = dict(artifacts.get("online_topology_lifecycle_json") or {})
        if not lifecycle_record:
            raise BundleResolutionError(f"Manifest does not declare online_topology_lifecycle_json: {manifest_path}")
        if not bool(lifecycle_record.get("exists", False)):
            raise BundleResolutionError(f"Manifest lifecycle artifact is missing on disk: {manifest_path}")
        lifecycle_surface = _clean_optional_text(lifecycle_record.get("artifact_surface"))
        if lifecycle_surface is not None and lifecycle_surface != ARTIFACT_SURFACE_LIFECYCLE:
            raise BundleResolutionError(f"Manifest lifecycle artifact is not lifecycle/debug surface: {manifest_path}")

        scene_root = manifest_path.parent.resolve()
        lifecycle_path = _resolve_artifact_record_path(
            lifecycle_record,
            scene_root=scene_root,
            manifest_path=manifest_path,
        )
        if lifecycle_path is None:
            raise BundleResolutionError(f"Could not resolve lifecycle artifact from manifest: {manifest_path}")

        summary_record = dict(artifacts.get("scene_summary_json") or {})
        summary_path = None
        if summary_record and bool(summary_record.get("exists", False)):
            summary_path = _resolve_artifact_record_path(
                summary_record,
                scene_root=scene_root,
                manifest_path=manifest_path,
            )

        summary_payload = None if summary_path is None else dict(_load_json(summary_path))
        return cls(
            scene_root=scene_root,
            lifecycle_path=lifecycle_path,
            selection_reason=selection_reason,
            manifest_path=manifest_path,
            summary_path=summary_path,
            artifact_profile=_clean_optional_text(manifest.get("artifact_profile")) or _infer_artifact_profile(summary_payload),
            artifact_capabilities=dict(manifest.get("artifact_capabilities") or {}),
            sequence_name=_clean_optional_text(manifest.get("sequence_name")),
            generated_at_utc=_clean_optional_text(manifest.get("generated_at_utc")),
            lifecycle_record={
                "artifact_surface": lifecycle_surface or ARTIFACT_SURFACE_LIFECYCLE,
                "artifact_semantics": _clean_optional_text(lifecycle_record.get("artifact_semantics")) or "diagnostic_history",
            },
        )

    @classmethod
    def from_input_path(
        cls,
        input_path: Path,
        *,
        selection_reason: str,
    ) -> "PublicationDiagnosticsBundle":
        path = input_path.resolve()
        if path.is_dir():
            for candidate in (path / "manifest.json", path / "logs" / "manifest.json"):
                if candidate.exists():
                    return cls.from_manifest_path(candidate, selection_reason=selection_reason)

        if path.name == "manifest.json":
            return cls.from_manifest_path(path, selection_reason=selection_reason)

        for candidate in _discover_candidate_files(path, ["summary.json"]):
            if not candidate.exists() or candidate.name != "summary.json":
                continue
            summary_path = candidate.resolve()
            scene_root = summary_path.parent.parent if summary_path.parent.name == "logs" else summary_path.parent
            summary_payload = dict(_load_json(summary_path))
            lifecycle_path = _resolve_path_token(
                summary_payload.get("online_topology_lifecycle_json"),
                scene_root=scene_root,
                reference_path=summary_path,
            )
            if lifecycle_path is None:
                fallback = summary_path.parent / "online_topology_lifecycle_v0_1.json"
                if fallback.exists():
                    lifecycle_path = fallback.resolve()
            if lifecycle_path is None:
                raise BundleResolutionError(f"Missing lifecycle artifact beside summary: {summary_path}")
            return cls(
                scene_root=scene_root,
                lifecycle_path=lifecycle_path,
                selection_reason=selection_reason,
                summary_path=summary_path,
                artifact_profile=_infer_artifact_profile(summary_payload),
                artifact_capabilities=dict(summary_payload.get("artifact_capabilities") or {}),
                sequence_name=_clean_optional_text(summary_payload.get("sequence_id")),
                generated_at_utc=None,
                lifecycle_record={
                    "artifact_surface": ARTIFACT_SURFACE_LIFECYCLE,
                    "artifact_semantics": "diagnostic_history",
                },
            )

        for candidate in _discover_candidate_files(path, ["online_topology_lifecycle_v0_1.json", "topology_v0_1.json"]):
            if not candidate.exists():
                continue
            lifecycle_path = candidate.resolve()
            if candidate.name == "topology_v0_1.json":
                sibling_lifecycle = candidate.parent / "online_topology_lifecycle_v0_1.json"
                if not sibling_lifecycle.exists():
                    raise BundleResolutionError(f"Missing sibling lifecycle artifact for topology input: {candidate}")
                lifecycle_path = sibling_lifecycle.resolve()
            elif candidate.name != "online_topology_lifecycle_v0_1.json":
                continue
            scene_root = lifecycle_path.parent.parent if lifecycle_path.parent.name == "logs" else lifecycle_path.parent
            summary_path = lifecycle_path.parent / "summary.json"
            summary_payload = dict(_load_json(summary_path)) if summary_path.exists() else None
            return cls(
                scene_root=scene_root,
                lifecycle_path=lifecycle_path,
                selection_reason=selection_reason,
                summary_path=summary_path if summary_path.exists() else None,
                artifact_profile=_infer_artifact_profile(summary_payload),
                artifact_capabilities=dict((summary_payload or {}).get("artifact_capabilities") or {}),
                sequence_name=_clean_optional_text((summary_payload or {}).get("sequence_id")) or _clean_optional_text(scene_root.name),
                generated_at_utc=None,
                lifecycle_record={
                    "artifact_surface": ARTIFACT_SURFACE_LIFECYCLE,
                    "artifact_semantics": "diagnostic_history",
                },
            )

        raise BundleResolutionError(f"Could not resolve a lifecycle/publication diagnostics bundle from: {input_path}")


def discover_latest_publication_bundle(search_roots: Optional[Sequence[Path]] = None) -> PublicationDiagnosticsBundle:
    roots = [Path(root).resolve() for root in (search_roots or [Path.cwd()])]
    candidates: List[Tuple[float, PublicationDiagnosticsBundle]] = []
    for root in roots:
        if not root.exists():
            continue
        for manifest_path in root.rglob("manifest.json"):
            try:
                bundle = PublicationDiagnosticsBundle.from_manifest_path(
                    manifest_path,
                    selection_reason=f"discovered_latest_from:{root}",
                )
            except BundleResolutionError:
                continue
            generated_dt = _parse_utc(bundle.generated_at_utc)
            sort_key = generated_dt.timestamp() if generated_dt is not None else manifest_path.stat().st_mtime
            candidates.append((sort_key, bundle))
    if not candidates:
        joined = ", ".join(str(root) for root in roots)
        raise BundleResolutionError(f"No lifecycle/publication diagnostics manifest bundle found under: {joined}")
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def load_publication_diagnostics_bundle(
    input_path: Optional[Path] = None,
    *,
    search_roots: Optional[Sequence[Path]] = None,
) -> PublicationDiagnosticsBundle:
    if input_path is not None:
        return PublicationDiagnosticsBundle.from_input_path(
            Path(input_path),
            selection_reason=f"explicit_input:{Path(input_path).resolve()}",
        )
    return discover_latest_publication_bundle(search_roots=search_roots)


class BoxFusionRosPublicationDiagnosticsBackend:
    def __init__(
        self,
        bundle: PublicationDiagnosticsBundle,
        *,
        stability_refresh_threshold: int = DEFAULT_STABILITY_REFRESH_THRESHOLD,
    ) -> None:
        self.bundle = bundle
        self.stability_refresh_threshold = max(1, int(stability_refresh_threshold))
        self.lifecycle_payload = self.bundle.lifecycle_payload
        self._room_diagnostics_cache: Optional[List[Dict[str, Any]]] = None
        self._room_lookup_cache: Optional[Dict[str, Dict[str, Any]]] = None

    @classmethod
    def from_bundle_path(
        cls,
        input_path: Optional[Path] = None,
        *,
        search_roots: Optional[Sequence[Path]] = None,
        stability_refresh_threshold: int = DEFAULT_STABILITY_REFRESH_THRESHOLD,
    ) -> "BoxFusionRosPublicationDiagnosticsBackend":
        return cls(
            load_publication_diagnostics_bundle(input_path=input_path, search_roots=search_roots),
            stability_refresh_threshold=stability_refresh_threshold,
        )

    def _bundle_payload(self) -> Dict[str, Any]:
        return self.bundle.describe()

    def _ok(self, service_name: str, payload: Dict[str, Any]) -> ServiceResult:
        return ServiceResult(
            success=True,
            payload={
                "service": service_name,
                "bundle": self._bundle_payload(),
                "payload": payload,
            },
        )

    def _request_error(self, service_name: str, error_code: str, message: str) -> ServiceResult:
        return ServiceResult(
            success=False,
            error_code=error_code,
            payload={
                "service": service_name,
                "bundle": self._bundle_payload(),
                "error_code": error_code,
                "message": message,
            },
        )

    def _room_diagnostic_payload(self, room_payload: Dict[str, Any]) -> Dict[str, Any]:
        room = dict(room_payload)
        publication_view = _publication_view_for_room(
            room,
            active_room_id=self.lifecycle_payload.get("active_room_id"),
            stability_refresh_threshold=self.stability_refresh_threshold,
        )
        published = bool(publication_view.get("published"))
        return {
            "room_id": _clean_optional_text(room.get("room_id")),
            "lifecycle_state": _clean_optional_text(room.get("lifecycle_state")),
            "candidate_complete": bool(room.get("candidate_complete")),
            "present_in_latest_export": bool(room.get("present_in_latest_export")),
            "last_departed_frame_idx": room.get("last_departed_frame_idx"),
            "export_observation_count": room.get("export_observation_count"),
            "publication_state": str(publication_view.get("publication_state") or ""),
            "finalization_blockers": list(publication_view.get("finalization_blockers") or []),
            "publication_blockers": list(publication_view.get("publication_blockers") or []),
            "leave_like_signal_v1": bool(publication_view.get("leave_like_signal_v1")),
            "leave_like_signal_v1_source": publication_view.get("leave_like_signal_v1_source"),
            "finalized_private": bool(publication_view.get("finalized_private")),
            "commit_ready": bool(publication_view.get("commit_ready")),
            "published": published,
            "pre_publication_only": not published,
            "public_topology_membership": "published" if published else "debug_only_pre_publication",
            "debug_only": True,
            "public_contract": False,
        }

    def _room_diagnostics(self) -> List[Dict[str, Any]]:
        if self._room_diagnostics_cache is None:
            rooms = list(self.lifecycle_payload.get("rooms") or [])
            self._room_diagnostics_cache = [
                self._room_diagnostic_payload(room_payload)
                for room_payload in rooms
                if _clean_optional_text(dict(room_payload).get("room_id")) is not None
            ]
        return [copy.deepcopy(room) for room in self._room_diagnostics_cache]

    def _room_lookup(self) -> Dict[str, Dict[str, Any]]:
        if self._room_lookup_cache is None:
            self._room_lookup_cache = {
                str(room["room_id"]): room
                for room in self._room_diagnostics()
                if room.get("room_id")
            }
        return {room_id: copy.deepcopy(room) for room_id, room in self._room_lookup_cache.items()}

    def get_publication_diagnostics(self) -> ServiceResult:
        rooms = self._room_diagnostics()
        publication_state_counts = Counter(
            str(room.get("publication_state"))
            for room in rooms
            if _clean_optional_text(room.get("publication_state")) is not None
        )
        published_room_ids = [str(room["room_id"]) for room in rooms if bool(room.get("published"))]
        pre_publication_room_ids = [str(room["room_id"]) for room in rooms if not bool(room.get("published"))]
        payload_summary = dict(self.lifecycle_payload.get("summary") or {})
        return self._ok(
            "GetPublicationDiagnostics",
            {
                "artifact_surface": ARTIFACT_SURFACE_LIFECYCLE,
                "debug_only": True,
                "public_contract": False,
                "public_topology_definition": "committed/published only",
                "non_published_rooms_remain_internal": True,
                "publication_state_machine_version": str(
                    (self.lifecycle_payload.get("publication_state_machine") or {}).get("version")
                    or PUBLICATION_STATE_MACHINE_VERSION
                ),
                "publication_state_order": list(PUBLICATION_STATE_ORDER),
                "active_room_id": self.lifecycle_payload.get("active_room_id"),
                "sequence_id": self.lifecycle_payload.get("sequence_id"),
                "frame_idx": self.lifecycle_payload.get("frame_idx"),
                "timestamp": self.lifecycle_payload.get("timestamp"),
                "summary": {
                    "room_count": int(len(rooms)),
                    "published_room_count": int(len(published_room_ids)),
                    "pre_publication_room_count": int(len(pre_publication_room_ids)),
                    "published_room_ids": published_room_ids,
                    "pre_publication_room_ids": pre_publication_room_ids,
                    "publication_state_counts": {
                        state: int(publication_state_counts.get(state, 0))
                        for state in PUBLICATION_STATE_ORDER
                        if int(publication_state_counts.get(state, 0)) > 0
                    },
                    "source_refresh_count": payload_summary.get("refresh_count"),
                    "source_public_topology_export_succeeded": payload_summary.get("public_topology_export_succeeded"),
                },
                "rooms": rooms,
            },
        )

    def get_room_publication_state(self, *, room_id: Any) -> ServiceResult:
        normalized_room_id = _normalize_room_id(room_id)
        if normalized_room_id is None:
            return self._request_error(
                "GetRoomPublicationState",
                "missing_room_id",
                "room_id is required.",
            )
        room = self._room_lookup().get(normalized_room_id)
        return self._ok(
            "GetRoomPublicationState",
            {
                "room_found": room is not None,
                "requested_room_id": normalized_room_id,
                "public_topology_definition": "committed/published only",
                "non_published_rooms_remain_internal": True,
                "room": room,
            },
        )


def load_generated_service_types(module_name: str) -> Dict[str, Any]:
    module = importlib.import_module(module_name)
    missing = [name for name in REQUIRED_SERVICE_TYPES if not hasattr(module, name)]
    if missing:
        raise BundleResolutionError(
            f"Generated service module {module_name!r} is missing: {', '.join(missing)}"
        )
    return {name: getattr(module, name) for name in REQUIRED_SERVICE_TYPES}


if Node is not None:  # pragma: no branch - definition-only split

    class BoxFusionPublicationDiagnosticsNode(Node):  # pragma: no cover - requires ROS runtime
        def __init__(
            self,
            *,
            service_types: Dict[str, Any],
            backend: Optional[BoxFusionRosPublicationDiagnosticsBackend] = None,
            node_name: str = "boxfusion_publication_diagnostics_node",
            service_prefix: str = DEFAULT_SERVICE_PREFIX,
            artifact_path: Optional[Path] = None,
            artifact_search_roots: Optional[Sequence[Path]] = None,
            stability_refresh_threshold: int = DEFAULT_STABILITY_REFRESH_THRESHOLD,
        ) -> None:
            super().__init__(node_name)
            self.declare_parameter("artifact_path", "" if artifact_path is None else str(artifact_path))
            self.declare_parameter(
                "artifact_search_roots",
                [str(path) for path in (artifact_search_roots or [])],
            )
            self.declare_parameter("service_prefix", service_prefix)
            self.declare_parameter("stability_refresh_threshold", int(stability_refresh_threshold))

            configured_artifact_path = _clean_optional_text(self.get_parameter("artifact_path").value)
            configured_search_roots = _normalize_search_root_tokens(self.get_parameter("artifact_search_roots").value)
            configured_service_prefix = _clean_optional_text(self.get_parameter("service_prefix").value) or DEFAULT_SERVICE_PREFIX
            configured_threshold = int(self.get_parameter("stability_refresh_threshold").value)

            self.backend = backend or BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(
                input_path=None if configured_artifact_path is None else Path(configured_artifact_path),
                search_roots=configured_search_roots or None,
                stability_refresh_threshold=configured_threshold,
            )
            self.service_prefix = configured_service_prefix.rstrip("/")
            self._services = []
            self._register_service(
                "get_publication_diagnostics",
                service_types["GetPublicationDiagnostics"],
                self._handle_get_publication_diagnostics,
            )
            self._register_service(
                "get_room_publication_state",
                service_types["GetRoomPublicationState"],
                self._handle_get_room_publication_state,
            )

        def _service_path(self, leaf_name: str) -> str:
            return f"{self.service_prefix}/{leaf_name}"

        def _register_service(self, leaf_name: str, srv_type: Any, handler: Any) -> None:
            path = self._service_path(leaf_name)
            self._services.append(self.create_service(srv_type, path, handler))
            self.get_logger().info(f"Registered service: {path}")

        def _write_response(self, response: Any, result: ServiceResult) -> Any:
            success, error_code, json_payload = result.to_ros_payload()
            response.success = success
            response.error_code = error_code
            response.json_response = json_payload
            return response

        def _handle_get_publication_diagnostics(self, request: Any, response: Any) -> Any:
            del request
            return self._write_response(response, self.backend.get_publication_diagnostics())

        def _handle_get_room_publication_state(self, request: Any, response: Any) -> Any:
            return self._write_response(
                response,
                self.backend.get_room_publication_state(
                    room_id=getattr(request, "room_id", None),
                ),
            )

else:

    class BoxFusionPublicationDiagnosticsNode:  # pragma: no cover - import-time fallback
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise ImportError("rclpy is not installed; the ROS publication diagnostics node cannot be created in this environment.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve debug-only online-topology publication diagnostics over ROS 2 services."
    )
    parser.add_argument(
        "--artifact-path",
        default=None,
        help="Optional path to a scene root, manifest.json, logs/summary.json, logs/online_topology_lifecycle_v0_1.json, or logs/topology_v0_1.json.",
    )
    parser.add_argument(
        "--artifact-search-root",
        action="append",
        default=[],
        help="Optional search root for latest-manifest discovery. May be passed more than once.",
    )
    parser.add_argument(
        "--service-module",
        default=DEFAULT_SERVICE_MODULE,
        help="Generated ROS 2 service module path, e.g. ros_interfaces.srv.",
    )
    parser.add_argument("--node-name", default="boxfusion_publication_diagnostics_node")
    parser.add_argument("--service-prefix", default=DEFAULT_SERVICE_PREFIX)
    parser.add_argument(
        "--stability-refresh-threshold",
        type=int,
        default=DEFAULT_STABILITY_REFRESH_THRESHOLD,
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    if rclpy is None:
        raise ImportError("rclpy is not installed; install ROS 2 Python bindings to run the publication diagnostics node.")
    ros_args = None if argv is None else list(argv)
    raw_cli_args = list(remove_ros_args(args=ros_args)) if remove_ros_args is not None else list(argv or [])
    cli_args = raw_cli_args[1:] if argv is None else raw_cli_args
    args = build_arg_parser().parse_args(cli_args)
    service_types = load_generated_service_types(args.service_module)
    rclpy.init(args=ros_args)
    node = BoxFusionPublicationDiagnosticsNode(
        service_types=service_types,
        node_name=args.node_name,
        service_prefix=args.service_prefix,
        artifact_path=None if args.artifact_path is None else Path(args.artifact_path),
        artifact_search_roots=[Path(item) for item in args.artifact_search_root] or None,
        stability_refresh_threshold=args.stability_refresh_threshold,
    )
    node.get_logger().info(json.dumps(node.backend.bundle.describe(), indent=2, sort_keys=True))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())
