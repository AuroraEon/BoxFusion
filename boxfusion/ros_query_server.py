from __future__ import annotations

import argparse
import copy
import importlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from boxfusion.artifact_contract import (
    ARTIFACT_PROFILE_CORE_ONLY,
    ARTIFACT_PROFILE_FULL,
    ARTIFACT_SURFACE_PUBLIC,
)
from boxfusion.query_api import RoomTopologyQueryAPI

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
except ImportError:  # pragma: no cover - exercised only in ROS-enabled environments
    rclpy = None
    Node = None
    remove_ros_args = None


DEFAULT_SERVICE_PREFIX = "/boxfusion/query"
DEFAULT_SERVICE_MODULE = "ros_interfaces.srv"
DEFAULT_ROUTE_POLICY = "balanced"
REQUIRED_SERVICE_TYPES = (
    "GetWorldSnapshot",
    "GetTopology",
    "ResolveObjectRoom",
    "RouteToRoom",
    "RouteToObject",
    "ExplainConnection",
)


class BundleResolutionError(RuntimeError):
    pass


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _normalize_route_policy(value: Any) -> str:
    return _clean_optional_text(value) or DEFAULT_ROUTE_POLICY


def _normalize_search_root_tokens(values: Optional[Sequence[Any]]) -> List[Path]:
    paths: List[Path] = []
    for value in values or []:
        text = _clean_optional_text(value)
        if text is None:
            continue
        paths.append(Path(text))
    return paths


def _load_json(path: Path) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _parse_utc(value: Any) -> Optional[datetime]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    try:
        if text.endswith("Z"):
            return datetime.fromisoformat(text[:-1] + "+00:00")
        parsed = datetime.fromisoformat(text)
        if parsed.tzinfo is None:
            return parsed.replace(tzinfo=timezone.utc)
        return parsed.astimezone(timezone.utc)
    except ValueError:
        return None


def _discover_candidate_files(path: Path, names: Sequence[str]) -> List[Path]:
    candidates: List[Path] = []
    if path.is_dir():
        for name in names:
            candidates.extend(
                [
                    path / name,
                    path / "logs" / name,
                ]
            )
    else:
        candidates.append(path)
    seen = set()
    ordered: List[Path] = []
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        ordered.append(candidate)
    return ordered


def _resolve_path_token(
    token: Any,
    *,
    scene_root: Optional[Path] = None,
    reference_path: Optional[Path] = None,
) -> Optional[Path]:
    text = _clean_optional_text(token)
    if text is None:
        return None
    raw = Path(text)
    candidates = [raw]
    if not raw.is_absolute():
        candidates.append(Path.cwd() / raw)
        if scene_root is not None:
            candidates.append(scene_root / raw)
            candidates.append(scene_root / raw.name)
        if reference_path is not None:
            candidates.append(reference_path.parent / raw)
            candidates.append(reference_path.parent / raw.name)
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return None


def _resolve_artifact_record_path(
    record: Dict[str, Any],
    *,
    scene_root: Path,
    manifest_path: Path,
) -> Optional[Path]:
    relative_path = _clean_optional_text(record.get("relative_path"))
    if relative_path is not None:
        candidate = (scene_root / relative_path).resolve()
        if candidate.exists():
            return candidate
    return _resolve_path_token(record.get("path"), scene_root=scene_root, reference_path=manifest_path)


def _infer_artifact_profile(summary_payload: Optional[Dict[str, Any]]) -> Optional[str]:
    summary = dict(summary_payload or {})
    profile = _clean_optional_text(summary.get("artifact_profile"))
    if profile is not None:
        return profile
    output_mode = _clean_optional_text(summary.get("output_mode"))
    if output_mode == "core_only":
        return ARTIFACT_PROFILE_CORE_ONLY
    if output_mode in {"full", "full_artifact"}:
        return ARTIFACT_PROFILE_FULL
    return None


@dataclass
class CommittedPublicBundle:
    scene_root: Path
    topology_path: Path
    selection_reason: str
    manifest_path: Optional[Path] = None
    summary_path: Optional[Path] = None
    world_snapshot_path: Optional[Path] = None
    artifact_profile: Optional[str] = None
    artifact_capabilities: Optional[Dict[str, Any]] = None
    sequence_name: Optional[str] = None
    generated_at_utc: Optional[str] = None
    topology_record: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        self.scene_root = Path(self.scene_root).resolve()
        self.topology_path = Path(self.topology_path).resolve()
        self.manifest_path = None if self.manifest_path is None else Path(self.manifest_path).resolve()
        self.summary_path = None if self.summary_path is None else Path(self.summary_path).resolve()
        self.world_snapshot_path = None if self.world_snapshot_path is None else Path(self.world_snapshot_path).resolve()
        self._manifest_payload: Optional[Dict[str, Any]] = None
        self._summary_payload: Optional[Dict[str, Any]] = None
        self._topology_payload: Optional[Dict[str, Any]] = None
        self._world_snapshot_payload: Optional[Dict[str, Any]] = None

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
    def topology_payload(self) -> Dict[str, Any]:
        if self._topology_payload is None:
            self._topology_payload = dict(_load_json(self.topology_path))
        return self._topology_payload

    @property
    def world_snapshot_payload(self) -> Optional[Dict[str, Any]]:
        if self.world_snapshot_path is None or not self.world_snapshot_path.exists():
            return None
        if self._world_snapshot_payload is None:
            self._world_snapshot_payload = dict(_load_json(self.world_snapshot_path))
        return self._world_snapshot_payload

    def describe(self) -> Dict[str, Any]:
        return {
            "scene_root": str(self.scene_root),
            "selection_reason": self.selection_reason,
            "manifest_path": None if self.manifest_path is None else str(self.manifest_path),
            "summary_path": None if self.summary_path is None else str(self.summary_path),
            "topology_path": str(self.topology_path),
            "world_snapshot_path": None if self.world_snapshot_path is None else str(self.world_snapshot_path),
            "sequence_name": self.sequence_name,
            "artifact_profile": self.artifact_profile,
            "artifact_capabilities": dict(self.artifact_capabilities or {}),
            "generated_at_utc": self.generated_at_utc,
            "topology_surface": None if self.topology_record is None else self.topology_record.get("artifact_surface"),
            "topology_semantics": None if self.topology_record is None else self.topology_record.get("artifact_semantics"),
        }

    @classmethod
    def from_manifest_path(
        cls,
        manifest_path: Path,
        *,
        selection_reason: str,
    ) -> "CommittedPublicBundle":
        manifest = dict(_load_json(manifest_path))
        artifacts = dict(manifest.get("artifacts") or {})
        topology_record = dict(artifacts.get("topology_json") or {})
        if not topology_record:
            raise BundleResolutionError(f"Manifest does not declare topology_json: {manifest_path}")
        if not bool(topology_record.get("exists", False)):
            raise BundleResolutionError(f"Manifest topology_json is missing on disk: {manifest_path}")
        if str(topology_record.get("artifact_surface") or "") != ARTIFACT_SURFACE_PUBLIC:
            raise BundleResolutionError(f"Manifest topology_json is not public: {manifest_path}")
        if str(topology_record.get("artifact_semantics") or "") != "committed_topology":
            raise BundleResolutionError(f"Manifest topology_json is not committed topology: {manifest_path}")

        scene_root = manifest_path.parent.resolve()
        topology_path = _resolve_artifact_record_path(
            topology_record,
            scene_root=scene_root,
            manifest_path=manifest_path,
        )
        if topology_path is None:
            raise BundleResolutionError(f"Could not resolve topology_json from manifest: {manifest_path}")

        summary_record = dict(artifacts.get("scene_summary_json") or {})
        summary_path = None
        if summary_record and bool(summary_record.get("exists", False)):
            summary_path = _resolve_artifact_record_path(
                summary_record,
                scene_root=scene_root,
                manifest_path=manifest_path,
            )

        summary_payload = None if summary_path is None else dict(_load_json(summary_path))
        world_snapshot_path = None
        if summary_payload is not None:
            world_snapshot_path = _resolve_path_token(
                summary_payload.get("final_vector_map_path"),
                scene_root=scene_root,
                reference_path=summary_path,
            )

        return cls(
            scene_root=scene_root,
            topology_path=topology_path,
            selection_reason=selection_reason,
            manifest_path=manifest_path,
            summary_path=summary_path,
            world_snapshot_path=world_snapshot_path,
            artifact_profile=_clean_optional_text(manifest.get("artifact_profile")) or _infer_artifact_profile(summary_payload),
            artifact_capabilities=dict(manifest.get("artifact_capabilities") or {}),
            sequence_name=_clean_optional_text(manifest.get("sequence_name")),
            generated_at_utc=_clean_optional_text(manifest.get("generated_at_utc")),
            topology_record=topology_record,
        )

    @classmethod
    def from_input_path(
        cls,
        input_path: Path,
        *,
        selection_reason: str,
    ) -> "CommittedPublicBundle":
        path = input_path.resolve()
        if path.is_dir():
            manifest_candidate = path / "manifest.json"
            if manifest_candidate.exists():
                return cls.from_manifest_path(manifest_candidate, selection_reason=selection_reason)
            logs_manifest_candidate = path / "logs" / "manifest.json"
            if logs_manifest_candidate.exists():
                return cls.from_manifest_path(logs_manifest_candidate, selection_reason=selection_reason)

        if path.name == "manifest.json":
            return cls.from_manifest_path(path, selection_reason=selection_reason)

        for candidate in _discover_candidate_files(path, ["summary.json"]):
            if candidate.exists() and candidate.name == "summary.json":
                summary_path = candidate.resolve()
                scene_root = summary_path.parent.parent if summary_path.parent.name == "logs" else summary_path.parent
                topology_path = summary_path.parent / "topology_v0_1.json"
                if not topology_path.exists():
                    raise BundleResolutionError(f"Missing committed topology beside summary: {summary_path}")
                summary_payload = dict(_load_json(summary_path))
                return cls(
                    scene_root=scene_root,
                    topology_path=topology_path,
                    selection_reason=selection_reason,
                    summary_path=summary_path,
                    world_snapshot_path=_resolve_path_token(
                        summary_payload.get("final_vector_map_path"),
                        scene_root=scene_root,
                        reference_path=summary_path,
                    ),
                    artifact_profile=_infer_artifact_profile(summary_payload),
                    artifact_capabilities=dict(summary_payload.get("artifact_capabilities") or {}),
                    sequence_name=_clean_optional_text(summary_payload.get("sequence_id")),
                    generated_at_utc=None,
                    topology_record={
                        "artifact_surface": ARTIFACT_SURFACE_PUBLIC,
                        "artifact_semantics": "committed_topology",
                    },
                )

        for candidate in _discover_candidate_files(path, ["topology_v0_1.json", "working_topology_v0_1.json"]):
            if not candidate.exists():
                continue
            if candidate.name == "working_topology_v0_1.json":
                raise BundleResolutionError(f"Working topology is debug-only and not a valid public query input: {candidate}")
            if candidate.name != "topology_v0_1.json":
                continue
            topology_path = candidate.resolve()
            scene_root = topology_path.parent.parent if topology_path.parent.name == "logs" else topology_path.parent
            summary_path = topology_path.parent / "summary.json"
            summary_payload = dict(_load_json(summary_path)) if summary_path.exists() else None
            return cls(
                scene_root=scene_root,
                topology_path=topology_path,
                selection_reason=selection_reason,
                summary_path=summary_path if summary_path.exists() else None,
                world_snapshot_path=_resolve_path_token(
                    None if summary_payload is None else summary_payload.get("final_vector_map_path"),
                    scene_root=scene_root,
                    reference_path=summary_path if summary_path.exists() else topology_path,
                ),
                artifact_profile=_infer_artifact_profile(summary_payload),
                artifact_capabilities=dict((summary_payload or {}).get("artifact_capabilities") or {}),
                sequence_name=_clean_optional_text((summary_payload or {}).get("sequence_id")) or _clean_optional_text(topology_path.parent.parent.name),
                generated_at_utc=None,
                topology_record={
                    "artifact_surface": ARTIFACT_SURFACE_PUBLIC,
                    "artifact_semantics": "committed_topology",
                },
            )

        raise BundleResolutionError(f"Could not resolve a committed/public export bundle from: {input_path}")


def discover_latest_committed_bundle(search_roots: Optional[Sequence[Path]] = None) -> CommittedPublicBundle:
    roots = [Path(root).resolve() for root in (search_roots or [Path.cwd()])]
    candidates: List[Tuple[float, CommittedPublicBundle]] = []
    for root in roots:
        if not root.exists():
            continue
        for manifest_path in root.rglob("manifest.json"):
            try:
                bundle = CommittedPublicBundle.from_manifest_path(
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
        raise BundleResolutionError(f"No committed/public manifest bundle found under: {joined}")
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def load_committed_public_bundle(
    input_path: Optional[Path] = None,
    *,
    search_roots: Optional[Sequence[Path]] = None,
) -> CommittedPublicBundle:
    if input_path is not None:
        return CommittedPublicBundle.from_input_path(
            Path(input_path),
            selection_reason=f"explicit_input:{Path(input_path).resolve()}",
        )
    return discover_latest_committed_bundle(search_roots=search_roots)


@dataclass
class ServiceResult:
    success: bool
    payload: Dict[str, Any]
    error_code: str = ""

    def to_ros_payload(self) -> Tuple[bool, str, str]:
        return self.success, self.error_code, json.dumps(self.payload, indent=2, sort_keys=True)


class BoxFusionRosQueryServerBackend:
    def __init__(self, bundle: CommittedPublicBundle) -> None:
        self.bundle = bundle
        self.query_api = RoomTopologyQueryAPI.from_json(bundle.topology_path)

    @classmethod
    def from_bundle_path(
        cls,
        input_path: Optional[Path] = None,
        *,
        search_roots: Optional[Sequence[Path]] = None,
    ) -> "BoxFusionRosQueryServerBackend":
        return cls(load_committed_public_bundle(input_path=input_path, search_roots=search_roots))

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

    def get_world_snapshot(self) -> ServiceResult:
        snapshot_payload = self.bundle.world_snapshot_payload
        manifest_payload = self.bundle.manifest_payload or {}
        return self._ok(
            "GetWorldSnapshot",
            {
                "snapshot_available": snapshot_payload is not None,
                "snapshot_kind": None if snapshot_payload is None else "final_vector_map_snapshot",
                "snapshot_path": None if self.bundle.world_snapshot_path is None else str(self.bundle.world_snapshot_path),
                "world_model_summary": dict(manifest_payload.get("world_model_summary") or {}),
                "scene_summary": copy.deepcopy(self.bundle.summary_payload),
                "snapshot": copy.deepcopy(snapshot_payload),
            },
        )

    def get_topology(self) -> ServiceResult:
        return self._ok(
            "GetTopology",
            {
                "artifact_surface": ARTIFACT_SURFACE_PUBLIC,
                "topology_path": str(self.bundle.topology_path),
                "topology": copy.deepcopy(self.bundle.topology_payload),
            },
        )

    def resolve_object_room(
        self,
        *,
        object_id: Any = None,
        object_label: Any = None,
    ) -> ServiceResult:
        object_id = _clean_optional_text(object_id)
        object_label = _clean_optional_text(object_label)
        if bool(object_id) == bool(object_label):
            return self._request_error(
                "ResolveObjectRoom",
                "exactly_one_object_selector_required",
                "Provide exactly one of object_id or object_label.",
            )
        result = self.query_api.resolve_object_room(object_id=object_id, object_label=object_label)
        return self._ok("ResolveObjectRoom", result)

    def route_to_room(
        self,
        *,
        start_room_id: Any,
        goal_room_id: Any,
        route_policy: Any = DEFAULT_ROUTE_POLICY,
    ) -> ServiceResult:
        start_room_id = _clean_optional_text(start_room_id)
        goal_room_id = _clean_optional_text(goal_room_id)
        if start_room_id is None or goal_room_id is None:
            return self._request_error(
                "RouteToRoom",
                "missing_room_selector",
                "Both start_room_id and goal_room_id are required.",
            )
        result = self.query_api.query_route(
            start_room_id,
            goal_room_id,
            route_policy=_normalize_route_policy(route_policy),
        )
        return self._ok("RouteToRoom", result)

    def route_to_object(
        self,
        *,
        start_room_id: Any,
        object_id: Any = None,
        object_label: Any = None,
        route_policy: Any = DEFAULT_ROUTE_POLICY,
    ) -> ServiceResult:
        start_room_id = _clean_optional_text(start_room_id)
        object_id = _clean_optional_text(object_id)
        object_label = _clean_optional_text(object_label)
        if start_room_id is None:
            return self._request_error(
                "RouteToObject",
                "missing_start_room_id",
                "start_room_id is required.",
            )
        if bool(object_id) == bool(object_label):
            return self._request_error(
                "RouteToObject",
                "exactly_one_object_selector_required",
                "Provide exactly one of object_id or object_label.",
            )
        result = self.query_api.query_route_to_object(
            start_room_id,
            object_id=object_id,
            object_label=object_label,
            route_policy=_normalize_route_policy(route_policy),
        )
        return self._ok("RouteToObject", result)

    def explain_connection(self, *, room_a: Any, room_b: Any) -> ServiceResult:
        room_a = _clean_optional_text(room_a)
        room_b = _clean_optional_text(room_b)
        if room_a is None or room_b is None:
            return self._request_error(
                "ExplainConnection",
                "missing_room_selector",
                "Both room_a and room_b are required.",
            )
        result = self.query_api.topology.explain_connection(room_a, room_b)
        return self._ok("ExplainConnection", result)


def load_generated_service_types(module_name: str) -> Dict[str, Any]:
    module = importlib.import_module(module_name)
    missing = [name for name in REQUIRED_SERVICE_TYPES if not hasattr(module, name)]
    if missing:
        raise BundleResolutionError(
            f"Generated service module {module_name!r} is missing: {', '.join(missing)}"
        )
    return {name: getattr(module, name) for name in REQUIRED_SERVICE_TYPES}


if Node is not None:  # pragma: no branch - definition-only split

    class BoxFusionQueryServerNode(Node):  # pragma: no cover - requires ROS runtime
        def __init__(
            self,
            *,
            service_types: Dict[str, Any],
            backend: Optional[BoxFusionRosQueryServerBackend] = None,
            node_name: str = "boxfusion_query_server_node",
            service_prefix: str = DEFAULT_SERVICE_PREFIX,
            artifact_path: Optional[Path] = None,
            artifact_search_roots: Optional[Sequence[Path]] = None,
        ) -> None:
            super().__init__(node_name)
            self.declare_parameter("artifact_path", "" if artifact_path is None else str(artifact_path))
            self.declare_parameter(
                "artifact_search_roots",
                [str(path) for path in (artifact_search_roots or [])],
            )
            self.declare_parameter("service_prefix", service_prefix)

            configured_artifact_path = _clean_optional_text(self.get_parameter("artifact_path").value)
            configured_search_roots = _normalize_search_root_tokens(self.get_parameter("artifact_search_roots").value)
            configured_service_prefix = _clean_optional_text(self.get_parameter("service_prefix").value) or DEFAULT_SERVICE_PREFIX

            self.backend = backend or BoxFusionRosQueryServerBackend.from_bundle_path(
                input_path=None if configured_artifact_path is None else Path(configured_artifact_path),
                search_roots=configured_search_roots or None,
            )
            self.service_prefix = configured_service_prefix.rstrip("/")
            self._services = []
            self._register_service("get_world_snapshot", service_types["GetWorldSnapshot"], self._handle_get_world_snapshot)
            self._register_service("get_topology", service_types["GetTopology"], self._handle_get_topology)
            self._register_service("resolve_object_room", service_types["ResolveObjectRoom"], self._handle_resolve_object_room)
            self._register_service("route_to_room", service_types["RouteToRoom"], self._handle_route_to_room)
            self._register_service("route_to_object", service_types["RouteToObject"], self._handle_route_to_object)
            self._register_service("explain_connection", service_types["ExplainConnection"], self._handle_explain_connection)

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

        def _handle_get_world_snapshot(self, request: Any, response: Any) -> Any:
            del request
            return self._write_response(response, self.backend.get_world_snapshot())

        def _handle_get_topology(self, request: Any, response: Any) -> Any:
            del request
            return self._write_response(response, self.backend.get_topology())

        def _handle_resolve_object_room(self, request: Any, response: Any) -> Any:
            return self._write_response(
                response,
                self.backend.resolve_object_room(
                    object_id=getattr(request, "object_id", None),
                    object_label=getattr(request, "object_label", None),
                ),
            )

        def _handle_route_to_room(self, request: Any, response: Any) -> Any:
            return self._write_response(
                response,
                self.backend.route_to_room(
                    start_room_id=getattr(request, "start_room_id", None),
                    goal_room_id=getattr(request, "goal_room_id", None),
                    route_policy=getattr(request, "route_policy", None),
                ),
            )

        def _handle_route_to_object(self, request: Any, response: Any) -> Any:
            return self._write_response(
                response,
                self.backend.route_to_object(
                    start_room_id=getattr(request, "start_room_id", None),
                    object_id=getattr(request, "object_id", None),
                    object_label=getattr(request, "object_label", None),
                    route_policy=getattr(request, "route_policy", None),
                ),
            )

        def _handle_explain_connection(self, request: Any, response: Any) -> Any:
            return self._write_response(
                response,
                self.backend.explain_connection(
                    room_a=getattr(request, "room_a", None),
                    room_b=getattr(request, "room_b", None),
                ),
            )

else:

    class BoxFusionQueryServerNode:  # pragma: no cover - import-time fallback
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise ImportError("rclpy is not installed; the ROS query server node cannot be created in this environment.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve the committed/public BoxFusion query surface over ROS 2 services."
    )
    parser.add_argument(
        "--artifact-path",
        default=None,
        help="Optional path to a scene root, manifest.json, logs/summary.json, or logs/topology_v0_1.json.",
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
    parser.add_argument("--node-name", default="boxfusion_query_server_node")
    parser.add_argument("--service-prefix", default=DEFAULT_SERVICE_PREFIX)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    if rclpy is None:
        raise ImportError("rclpy is not installed; install ROS 2 Python bindings to run the query server node.")
    ros_args = None if argv is None else list(argv)
    raw_cli_args = list(remove_ros_args(args=ros_args)) if remove_ros_args is not None else list(argv or [])
    cli_args = raw_cli_args[1:] if argv is None else raw_cli_args
    args = build_arg_parser().parse_args(cli_args)
    service_types = load_generated_service_types(args.service_module)
    rclpy.init(args=ros_args)
    node = BoxFusionQueryServerNode(
        service_types=service_types,
        node_name=args.node_name,
        service_prefix=args.service_prefix,
        artifact_path=None if args.artifact_path is None else Path(args.artifact_path),
        artifact_search_roots=[Path(item) for item in args.artifact_search_root] or None,
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
