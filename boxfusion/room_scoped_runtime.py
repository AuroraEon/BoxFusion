from __future__ import annotations

import copy
import json
import os
import time
from collections import Counter, deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from boxfusion.online_topology_working_snapshot import build_committed_topology_projection


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _canonical_room_id(value: Any) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.startswith("room_"):
        suffix = text[5:]
        if suffix.lstrip("-").isdigit() and int(suffix) < 0:
            return None
        return text
    if text.lstrip("-").isdigit():
        numeric_id = int(text)
        if numeric_id < 0:
            return None
        return f"room_{numeric_id}"
    return text


def _canonical_object_id(value: Any) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.startswith("obj_"):
        suffix = text[4:]
        if suffix.lstrip("-").isdigit() and int(suffix) < 0:
            return None
        return text
    if text.lstrip("-").isdigit():
        numeric_id = int(text)
        if numeric_id < 0:
            return None
        return f"obj_{numeric_id}"
    return text


def _canonical_anchor_id(value: Any) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.lstrip("-").isdigit():
        numeric_id = int(text)
        if numeric_id < 0:
            return None
        return f"anchor_{numeric_id}"
    return text


def _stable_room_ids(values: Iterable[Any]) -> List[str]:
    seen: Set[str] = set()
    ordered: List[str] = []
    for value in values:
        room_id = _canonical_room_id(value)
        if room_id is None or room_id in seen:
            continue
        seen.add(room_id)
        ordered.append(room_id)
    return ordered


def _parse_room_id_csv(value: Any) -> List[str]:
    text = _clean_optional_text(value)
    if text is None:
        return []
    normalized = text.replace("|", ",")
    room_ids: List[str] = []
    seen: Set[str] = set()
    for token in normalized.split(","):
        room_id = _canonical_room_id(token)
        if room_id is None or room_id in seen:
            continue
        seen.add(room_id)
        room_ids.append(room_id)
    return room_ids


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _polygon_centroid(polygon: Sequence[Sequence[float]]) -> Optional[List[float]]:
    points = [list(point) for point in list(polygon or []) if len(list(point)) >= 2]
    if not points:
        return None
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    return [_round_float(sum(xs) / float(len(xs))), _round_float(sum(ys) / float(len(ys)))]


def _bbox_xy(polygon: Sequence[Sequence[float]]) -> Optional[Dict[str, float]]:
    points = [list(point) for point in list(polygon or []) if len(list(point)) >= 2]
    if not points:
        return None
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    return {
        "min_x": _round_float(min(xs)),
        "min_y": _round_float(min(ys)),
        "max_x": _round_float(max(xs)),
        "max_y": _round_float(max(ys)),
    }


def _euclidean_xy(point_a: Optional[Sequence[float]], point_b: Optional[Sequence[float]]) -> Optional[float]:
    if not point_a or not point_b or len(point_a) < 2 or len(point_b) < 2:
        return None
    dx = float(point_a[0]) - float(point_b[0])
    dy = float(point_a[1]) - float(point_b[1])
    return (dx * dx + dy * dy) ** 0.5


def _write_json(path: Path, payload: Dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def _unlink_if_exists(path: Path) -> None:
    try:
        if path.exists():
            path.unlink()
    except FileNotFoundError:
        return
    except OSError:
        if os.path.isdir(path):
            raise


class RoomScopedRuntimeManager:
    """Maintain a bounded local room state beside committed room memory.

    The manager intentionally stays summary-level. It does not replace the
    existing world export or topology builder; it provides the room-scoped
    online fusion/commit contract that decides which already-exported room facts
    become public.
    """

    def __init__(
        self,
        *,
        sequence_id: Optional[str] = None,
        local_window_size: int = 4,
        committed_retrieval_limit: int = 3,
        recent_history_limit: int = 12,
    ) -> None:
        self.sequence_id = _clean_optional_text(sequence_id)
        self.local_window_size = max(1, int(local_window_size))
        self.committed_retrieval_limit = max(1, int(committed_retrieval_limit))
        self.recent_history_limit = max(4, int(recent_history_limit))
        self.active_room_id: Optional[str] = None
        self.recent_room_trace: Deque[str] = deque(maxlen=self.recent_history_limit)
        self.local_state: Dict[str, Any] = {
            "frame_idx": None,
            "timestamp": None,
            "active_room_id": None,
            "active_floor_id": None,
            "local_room_ids": [],
            "changed_room_ids": [],
            "retrieved_committed_room_ids": [],
            "retrieval_policy": "adjacent_same_floor_then_nearest_committed_subset",
            "retrieval_limit": self.committed_retrieval_limit,
        }
        self.committed_rooms: Dict[str, Dict[str, Any]] = {}
        self.commit_history: List[Dict[str, Any]] = []
        self.refresh_history: List[Dict[str, Any]] = []
        self._last_vector_map: Dict[str, Any] = {}

    def observe_tracking(
        self,
        *,
        frame_idx: int,
        timestamp: float,
        current_room_id: Optional[Any],
    ) -> None:
        room_id = _canonical_room_id(current_room_id)
        self.active_room_id = room_id
        self.local_state["frame_idx"] = int(frame_idx)
        self.local_state["timestamp"] = float(timestamp)
        self.local_state["active_room_id"] = room_id
        if room_id is not None and (not self.recent_room_trace or self.recent_room_trace[-1] != room_id):
            self.recent_room_trace.append(room_id)

    def observe(
        self,
        *,
        frame_idx: int,
        timestamp: float,
        current_room_id: Optional[Any],
        vector_map: Optional[Dict[str, Any]],
        export_profile: Optional[Dict[str, Any]],
        lifecycle_manager: Any,
    ) -> None:
        self.observe_tracking(
            frame_idx=int(frame_idx),
            timestamp=float(timestamp),
            current_room_id=current_room_id,
        )
        world = dict(vector_map or {})
        if not world:
            return
        self._last_vector_map = copy.deepcopy(world)
        room_lookup = self._room_lookup(world)
        adjacency_lookup = self._adjacency_lookup(world)
        active_room_id = self.active_room_id
        active_floor_id = None
        if active_room_id is not None:
            active_floor_id = _clean_optional_text(dict(room_lookup.get(active_room_id) or {}).get("floor_id"))

        changed_room_ids = self._changed_room_ids(export_profile)
        local_room_ids = self._local_room_ids(
            active_room_id=active_room_id,
            changed_room_ids=changed_room_ids,
            adjacency_lookup=adjacency_lookup,
        )
        retrieved_committed_room_ids = self._retrieve_relevant_committed_subset(
            active_room_id=active_room_id,
            active_floor_id=active_floor_id,
            room_lookup=room_lookup,
            adjacency_lookup=adjacency_lookup,
        )

        self.local_state.update(
            {
                "frame_idx": int(frame_idx),
                "timestamp": float(timestamp),
                "active_room_id": active_room_id,
                "active_floor_id": active_floor_id,
                "local_room_ids": list(local_room_ids),
                "changed_room_ids": list(changed_room_ids),
                "retrieved_committed_room_ids": list(retrieved_committed_room_ids),
                "recent_room_trace": list(self.recent_room_trace),
            }
        )

        room_statuses = dict(getattr(lifecycle_manager, "room_statuses", {}) or {})
        for room_id, status in sorted(room_statuses.items()):
            if room_id not in room_lookup:
                continue
            if not bool(getattr(status, "candidate_complete", False)):
                continue
            if list(getattr(status, "commit_block_reasons", []) or []):
                continue
            if not bool(getattr(status, "present_in_latest_export", False)):
                continue
            self._commit_room(
                room_id=room_id,
                frame_idx=int(frame_idx),
                timestamp=float(timestamp),
                room_lookup=room_lookup,
                adjacency_lookup=adjacency_lookup,
                vector_map=world,
                lifecycle_status=status,
                retrieved_committed_room_ids=retrieved_committed_room_ids,
            )

        self.refresh_history.append(
            {
                "frame_idx": int(frame_idx),
                "timestamp": float(timestamp),
                "active_room_id": active_room_id,
                "active_floor_id": active_floor_id,
                "local_room_ids": list(local_room_ids),
                "changed_room_ids": list(changed_room_ids),
                "retrieved_committed_room_ids": list(retrieved_committed_room_ids),
                "committed_room_ids": sorted(self.committed_rooms),
            }
        )

    def committed_room_ids(self) -> List[str]:
        return sorted(self.committed_rooms)

    def build_public_topology_payload(self, topology_payload: Dict[str, Any]) -> Dict[str, Any]:
        committed_room_ids = self.committed_room_ids()
        committed_payload = build_committed_topology_projection(topology_payload, committed_room_ids)
        used_floor_ids = {
            _clean_optional_text(room.get("floor_id"))
            for room in list(committed_payload.get("rooms") or [])
            if room.get("floor_id") is not None
        }
        committed_payload["floors"] = [
            dict(floor)
            for floor in list(committed_payload.get("floors") or [])
            if _clean_optional_text(floor.get("floor_id")) in used_floor_ids
        ]
        committed_payload["metadata"] = dict(committed_payload.get("metadata") or {})
        committed_payload["metadata"].update(
            {
                "artifact_kind": "committed_topology_public",
                "artifact_surface": "public",
                "artifact_semantics": "committed_topology",
                "public_topology_meaning": "committed/published only",
                "debug_only": False,
                "public_default": True,
                "non_public": False,
                "room_scope_runtime_contract": {
                    "local_state": "bounded active-room working state only",
                    "committed_memory": "accepted room summaries only",
                    "fusion_policy": "current observations fuse with local state and selectively retrieved committed subset",
                },
                "committed_room_ids": list(committed_room_ids),
            }
        )
        return committed_payload

    def build_committed_world_snapshot(self, vector_map: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        world = dict(vector_map or self._last_vector_map or {})
        room_lookup = self._room_lookup(world)
        committed_room_ids = set(self.committed_room_ids())
        used_floor_ids = {
            _clean_optional_text(dict(room_lookup.get(room_id) or {}).get("floor_id"))
            for room_id in committed_room_ids
        }
        filtered_rooms = [
            dict(room_lookup[room_id])
            for room_id in sorted(committed_room_ids)
            if room_id in room_lookup
        ]
        filtered_objects = []
        for obj in list(world.get("objects") or []):
            room_id = _canonical_room_id(obj.get("room_id") or obj.get("room_uuid"))
            if room_id in committed_room_ids:
                filtered_objects.append(dict(obj))
        filtered_anchors = []
        for anchor in list(world.get("anchors") or []):
            room_id = _canonical_room_id(anchor.get("room_id") or anchor.get("target_id"))
            if room_id in committed_room_ids:
                filtered_anchors.append(dict(anchor))
        filtered_gateways = []
        for gateway in list(world.get("gateways") or []):
            connects = _stable_room_ids(gateway.get("connects") or [])
            if len(connects) == 2 and connects[0] in committed_room_ids and connects[1] in committed_room_ids:
                filtered_gateways.append(dict(gateway))
        filtered_vertical_transitions = []
        for transition in list(world.get("vertical_transitions") or []):
            from_room_id = _canonical_room_id(transition.get("from_room_id"))
            to_room_id = _canonical_room_id(transition.get("to_room_id"))
            if from_room_id in committed_room_ids and to_room_id in committed_room_ids:
                filtered_vertical_transitions.append(dict(transition))
        return {
            "version": "0.1",
            "artifact_kind": "committed_room_vector_map_snapshot",
            "artifact_surface": "public",
            "artifact_semantics": "committed_room_world_snapshot",
            "public_topology_meaning": "committed/published only",
            "sequence_id": self.sequence_id,
            "frame_idx": self.local_state.get("frame_idx"),
            "timestamp": self.local_state.get("timestamp"),
            "floors": [
                dict(floor)
                for floor in list(world.get("floors") or [])
                if _clean_optional_text(floor.get("floor_id")) in used_floor_ids
            ],
            "rooms": filtered_rooms,
            "gateways": filtered_gateways,
            "vertical_transitions": filtered_vertical_transitions,
            "objects": filtered_objects,
            "anchors": filtered_anchors,
            "metadata": {
                "committed_room_ids": sorted(committed_room_ids),
                "source": "room_scoped_runtime_manager",
                "query_semantics": "committed/published only",
            },
        }

    def build_committed_room_world_model(
        self,
        *,
        topology_payload: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        topology_lookup = self._topology_edge_lookup(dict(topology_payload or {}))
        room_records: List[Dict[str, Any]] = []
        adjacency_records: List[Dict[str, Any]] = []
        room_to_objects: Dict[str, List[str]] = {}
        for room_id in sorted(self.committed_rooms):
            record = copy.deepcopy(self.committed_rooms[room_id])
            edge_neighbors = topology_lookup.get(room_id, {})
            neighbor_room_ids = sorted(set(record.get("neighbor_room_ids") or []) | set(edge_neighbors))
            record["neighbor_room_ids"] = neighbor_room_ids
            record["connectivity"] = [
                {
                    "room_id": neighbor_room_id,
                    **dict(edge_neighbors.get(neighbor_room_id) or {"relation_type": "adjacent", "status": "supported"}),
                }
                for neighbor_room_id in neighbor_room_ids
            ]
            record["bev_vln_hook"] = {
                "graph_node_id": room_id,
                "centroid_xy": copy.deepcopy(record.get("centroid_xy")),
                "extent_bbox_xy": copy.deepcopy(record.get("extent_bbox_xy")),
                "footprint_polygon_xy": copy.deepcopy(record.get("footprint_polygon_xy")),
                "neighbor_room_ids": list(neighbor_room_ids),
                "semantic_landmarks": list(record.get("semantic_summary", {}).get("dominant_object_labels") or []),
            }
            room_records.append(record)
            room_to_objects[room_id] = list(record.get("object_ids") or [])
            for neighbor_room_id in neighbor_room_ids:
                if room_id >= neighbor_room_id:
                    continue
                edge_payload = dict(edge_neighbors.get(neighbor_room_id) or {"relation_type": "adjacent", "status": "supported"})
                adjacency_records.append(
                    {
                        "source": room_id,
                        "target": neighbor_room_id,
                        "relation_type": edge_payload.get("relation_type", "adjacent"),
                        "status": edge_payload.get("status", "supported"),
                    }
                )
        return {
            "version": "0.1",
            "artifact_kind": "committed_room_world_model",
            "artifact_surface": "public",
            "artifact_semantics": "committed_room_world_model",
            "public_topology_meaning": "committed/published only",
            "sequence_id": self.sequence_id,
            "generated_at_utc": _utc_now_iso(),
            "runtime_contract": {
                "local_working_state": "active room or bounded recent room window only; non-public",
                "committed_global_memory": "accepted room summaries only; public/default topology source",
                "selective_history_retrieval": "local fusion uses only a bounded relevant committed subset, never the entire committed history",
            },
            "summary": {
                "room_count": int(len(room_records)),
                "adjacency_count": int(len(adjacency_records)),
                "committed_room_ids": [record["room_id"] for record in room_records],
            },
            "rooms": room_records,
            "adjacency": adjacency_records,
            "indices": {
                "room_to_objects": room_to_objects,
            },
            "metadata": {
                "stable_room_id_namespace": "room_*",
                "future_bev_vln_hook": "room.bev_vln_hook",
                "deferred": [
                    "global_bev_raster",
                    "navigation_policy",
                    "sidecar_authority",
                ],
            },
        }

    def build_runtime_state_report(self) -> Dict[str, Any]:
        return {
            "version": "0.1",
            "artifact_kind": "room_scoped_runtime_state",
            "artifact_surface": "diagnostic",
            "debug_only": True,
            "public_default": False,
            "non_public": True,
            "sequence_id": self.sequence_id,
            "local_working_state": copy.deepcopy(self.local_state),
            "committed_global_memory": {
                "room_count": int(len(self.committed_rooms)),
                "room_ids": sorted(self.committed_rooms),
            },
            "commit_history": copy.deepcopy(self.commit_history),
            "refresh_history": copy.deepcopy(self.refresh_history),
        }

    def export_artifacts(
        self,
        *,
        log_dir: Path,
        final_vector_map: Optional[Dict[str, Any]],
        full_topology_payload: Dict[str, Any],
        materialize_runtime_state_report: bool = True,
    ) -> Dict[str, Any]:
        log_dir = Path(log_dir)
        committed_topology_payload = self.build_public_topology_payload(full_topology_payload)
        committed_room_world_model_payload = self.build_committed_room_world_model(
            topology_payload=committed_topology_payload,
        )
        committed_world_snapshot_payload = self.build_committed_world_snapshot(final_vector_map)
        runtime_state_payload = self.build_runtime_state_report()

        topology_path = log_dir / "topology_v0_1.json"
        topology_report_path = log_dir / "topology_query_report.json"
        world_model_path = log_dir / "committed_room_world_model_v0_1.json"
        runtime_state_path = log_dir / "room_scoped_runtime_state_v0_1.json"
        public_snapshot_path = log_dir / "committed_room_world_snapshot_v0_1.json"

        from boxfusion.room_topology import RoomTopology

        public_export_start = time.perf_counter()
        RoomTopology.from_dict(committed_topology_payload).export_json(topology_path)
        RoomTopology.from_dict(committed_topology_payload).export_query_report(topology_report_path)
        _write_json(world_model_path, committed_room_world_model_payload)
        _write_json(public_snapshot_path, committed_world_snapshot_payload)
        public_export_sec = time.perf_counter() - public_export_start

        runtime_state_export_sec = 0.0
        runtime_state_exported = False
        if materialize_runtime_state_report:
            runtime_state_start = time.perf_counter()
            _write_json(runtime_state_path, runtime_state_payload)
            runtime_state_export_sec = time.perf_counter() - runtime_state_start
            runtime_state_exported = True
        else:
            _unlink_if_exists(runtime_state_path)

        return {
            "topology_path": str(topology_path),
            "topology_query_report_path": str(topology_report_path),
            "committed_room_world_model_path": str(world_model_path),
            "room_scoped_runtime_state_path": None if not runtime_state_exported else str(runtime_state_path),
            "public_world_snapshot_path": str(public_snapshot_path),
            "committed_room_ids": self.committed_room_ids(),
            "committed_topology_payload": committed_topology_payload,
            "committed_room_world_model_payload": committed_room_world_model_payload,
            "runtime_state_payload": runtime_state_payload,
            "export_timing_sec": {
                "public_bundle_export_sec": round(float(public_export_sec), 6),
                "runtime_state_export_sec": round(float(runtime_state_export_sec), 6),
            },
        }

    def _changed_room_ids(self, export_profile: Optional[Dict[str, Any]]) -> List[str]:
        profile = dict(export_profile or {})
        room_ids = []
        for key in (
            "changed_room_ids",
            "changed_room_structure_changed_ids",
            "changed_room_object_delta_ids",
            "changed_room_removed_ids",
        ):
            room_ids.extend(_parse_room_id_csv(profile.get(key)))
        return _stable_room_ids(room_ids)

    def _local_room_ids(
        self,
        *,
        active_room_id: Optional[str],
        changed_room_ids: Sequence[str],
        adjacency_lookup: Dict[str, Set[str]],
    ) -> List[str]:
        ordered: List[str] = []
        if active_room_id is not None:
            ordered.append(active_room_id)
            ordered.extend(sorted(adjacency_lookup.get(active_room_id, set())))
        ordered.extend(changed_room_ids)
        ordered.extend(reversed(self.recent_room_trace))
        return _stable_room_ids(ordered)[: self.local_window_size]

    def _retrieve_relevant_committed_subset(
        self,
        *,
        active_room_id: Optional[str],
        active_floor_id: Optional[str],
        room_lookup: Dict[str, Dict[str, Any]],
        adjacency_lookup: Dict[str, Set[str]],
    ) -> List[str]:
        if not self.committed_rooms:
            return []
        active_room = dict(room_lookup.get(active_room_id) or {})
        active_centroid = active_room.get("center") or _polygon_centroid(active_room.get("polygon") or [])
        ranked: List[Tuple[int, float, int, str]] = []
        for rank_idx, room_id in enumerate(sorted(self.committed_rooms)):
            record = dict(self.committed_rooms.get(room_id) or {})
            score = 0
            if active_floor_id is not None and active_floor_id == record.get("floor_id"):
                score += 5
            if active_room_id is not None and room_id in adjacency_lookup.get(active_room_id, set()):
                score += 10
            if active_room_id is not None and active_room_id in set(record.get("neighbor_room_ids") or []):
                score += 4
            distance = _euclidean_xy(active_centroid, record.get("centroid_xy"))
            if distance is not None:
                if distance <= 2.5:
                    score += 3
                elif distance <= 5.0:
                    score += 2
                else:
                    score += 1
            ranked.append((score, 10**6 if distance is None else distance, rank_idx, room_id))
        ranked.sort(key=lambda item: (-item[0], item[1], item[2], item[3]))
        filtered = [room_id for score, _, _, room_id in ranked if score > 0]
        if not filtered:
            filtered = [item[3] for item in ranked]
        return filtered[: self.committed_retrieval_limit]

    def _commit_room(
        self,
        *,
        room_id: str,
        frame_idx: int,
        timestamp: float,
        room_lookup: Dict[str, Dict[str, Any]],
        adjacency_lookup: Dict[str, Set[str]],
        vector_map: Dict[str, Any],
        lifecycle_status: Any,
        retrieved_committed_room_ids: Sequence[str],
    ) -> None:
        room = dict(room_lookup.get(room_id) or {})
        polygon = [list(point) for point in list(room.get("polygon") or [])]
        centroid_xy = room.get("center") or _polygon_centroid(polygon)
        extent_bbox_xy = _bbox_xy(polygon)

        object_ids: List[str] = []
        object_label_counter: Counter = Counter()
        for obj in list(vector_map.get("objects") or []):
            object_room_id = _canonical_room_id(obj.get("room_id") or obj.get("room_uuid"))
            if object_room_id != room_id:
                continue
            object_id = _canonical_object_id(obj.get("id"))
            if object_id is not None:
                object_ids.append(object_id)
            object_label_counter[str(obj.get("label") or obj.get("category") or "unknown")] += 1

        anchor_ids: List[str] = []
        for anchor in list(vector_map.get("anchors") or []):
            anchor_room_id = _canonical_room_id(anchor.get("room_id") or anchor.get("target_id"))
            if anchor_room_id != room_id:
                continue
            anchor_id = _canonical_anchor_id(anchor.get("id"))
            if anchor_id is not None:
                anchor_ids.append(anchor_id)

        neighbor_room_ids = sorted(
            neighbor_room_id
            for neighbor_room_id in adjacency_lookup.get(room_id, set())
            if neighbor_room_id in self.committed_rooms or neighbor_room_id == room_id
        )
        if room_id in neighbor_room_ids:
            neighbor_room_ids.remove(room_id)

        record = {
            "room_id": room_id,
            "stable_room_id": room_id,
            "floor_id": _clean_optional_text(room.get("floor_id")),
            "room_type": _clean_optional_text(room.get("room_type")) or "unknown",
            "status": _clean_optional_text(room.get("status")) or "confirmed",
            "centroid_xy": copy.deepcopy(centroid_xy),
            "extent_bbox_xy": copy.deepcopy(extent_bbox_xy),
            "footprint_polygon_xy": copy.deepcopy(polygon),
            "footprint_area_m2": _round_float(room.get("area_m2")),
            "neighbor_room_ids": neighbor_room_ids,
            "object_ids": sorted(object_ids),
            "anchor_ids": sorted(anchor_ids),
            "semantic_summary": {
                "object_count": int(len(object_ids)),
                "anchor_count": int(len(anchor_ids)),
                "dominant_object_labels": [
                    label
                    for label, _ in sorted(object_label_counter.items(), key=lambda item: (-item[1], item[0]))[:5]
                ],
                "object_label_counts": dict(sorted(object_label_counter.items())),
            },
            "timestamps": {
                "first_seen_frame_idx": getattr(lifecycle_status, "first_seen_frame_idx", None),
                "last_observed_frame_idx": getattr(lifecycle_status, "last_observed_frame_idx", None),
                "committed_at_frame_idx": int(frame_idx),
                "committed_at_timestamp": float(timestamp),
            },
            "provenance": {
                "source": "room_scoped_runtime_manager",
                "observation_count": int(getattr(lifecycle_status, "export_observation_count", 0) or 0),
                "candidate_complete_reasons": sorted(
                    str(reason.value if hasattr(reason, "value") else reason)
                    for reason in list(getattr(lifecycle_status, "candidate_complete_reasons", []) or [])
                ),
                "selective_retrieval_context_room_ids": list(retrieved_committed_room_ids),
            },
        }
        self.committed_rooms[room_id] = record
        commit_event = {
            "room_id": room_id,
            "frame_idx": int(frame_idx),
            "timestamp": float(timestamp),
            "committed_room_count": int(len(self.committed_rooms)),
            "retrieved_committed_room_ids": list(retrieved_committed_room_ids),
        }
        if not self.commit_history or self.commit_history[-1] != commit_event:
            self.commit_history.append(commit_event)

    def _room_lookup(self, vector_map: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        lookup: Dict[str, Dict[str, Any]] = {}
        for room in list(vector_map.get("rooms") or []):
            room_id = _canonical_room_id(room.get("id") or room.get("room_id"))
            if room_id is None:
                continue
            lookup[room_id] = dict(room)
        return lookup

    def _adjacency_lookup(self, vector_map: Dict[str, Any]) -> Dict[str, Set[str]]:
        lookup: Dict[str, Set[str]] = {}
        for room_id in self._room_lookup(vector_map):
            lookup.setdefault(room_id, set())
        for gateway in list(vector_map.get("gateways") or []):
            connects = _stable_room_ids(gateway.get("connects") or [])
            if len(connects) != 2:
                continue
            lookup.setdefault(connects[0], set()).add(connects[1])
            lookup.setdefault(connects[1], set()).add(connects[0])
        for edge in list(vector_map.get("edges") or []):
            source = _canonical_room_id(edge.get("source") or edge.get("source_room_id"))
            target = _canonical_room_id(edge.get("target") or edge.get("target_room_id"))
            if source is None or target is None or source == target:
                continue
            lookup.setdefault(source, set()).add(target)
            lookup.setdefault(target, set()).add(source)
        return lookup

    def _topology_edge_lookup(self, topology_payload: Dict[str, Any]) -> Dict[str, Dict[str, Dict[str, Any]]]:
        lookup: Dict[str, Dict[str, Dict[str, Any]]] = {}
        for edge in list(topology_payload.get("edges") or []):
            source = _canonical_room_id(edge.get("source"))
            target = _canonical_room_id(edge.get("target"))
            if source is None or target is None or source == target:
                continue
            payload = {
                "relation_type": _clean_optional_text(edge.get("relation_type")) or "adjacent",
                "status": _clean_optional_text(edge.get("status")) or "supported",
            }
            lookup.setdefault(source, {})[target] = payload
            lookup.setdefault(target, {})[source] = payload
        return lookup
