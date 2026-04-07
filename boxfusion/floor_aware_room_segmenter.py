from __future__ import annotations

import hashlib
import json
import os
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import open3d as o3d

from boxfusion.dynamic_room_segmenter import DynamicRoomSegmenter
from boxfusion.floor_artifacts import (
    attach_floor_metadata,
    build_fallback_summary,
    build_floor_lookup,
    build_vertical_transition_summary,
    canonicalize_floors,
    canonicalize_vertical_transitions,
    display_floor_label,
)
from boxfusion.floor_manager import FloorManager
from boxfusion.scene_graph_builder import AnchorNode, FloorNode, ObjectNode, RoomNode, SemanticSceneGraph


@dataclass
class FloorState:
    floor_id: str
    segmenter: DynamicRoomSegmenter
    merged_points_xyzrgb: Optional[np.ndarray] = None
    pending_chunks: List[np.ndarray] = field(default_factory=list)
    pending_chunk_frame_indices: List[int] = field(default_factory=list)
    local_to_world_room_id: Dict[int, int] = field(default_factory=dict)
    last_tracking_report: Dict[str, Any] = field(default_factory=dict)
    last_segmentation_frame_idx: Optional[int] = None
    stable_frame_count: int = 0
    transition_frame_count: int = 0
    stable_keyframe_chunk_count: int = 0
    segmentation_trigger_count: int = 0
    exported_room_count: int = 0
    exported_object_count: int = 0
    exported_anchor_count: int = 0
    last_stable_frame_idx: Optional[int] = None
    last_transition_frame_idx: Optional[int] = None
    segmentation_reports: List[Dict[str, Any]] = field(default_factory=list)


class FloorAwareRoomSegmenter:
    """Wrap one room-segmentation state per floor and export a combined world graph."""

    def __init__(self, resolution: float = 0.05, config: Optional[Dict[str, Any]] = None) -> None:
        self.resolution = float(resolution)
        self.config = config or {}
        runtime_logging_cfg = dict((self.config or {}).get("runtime_logging", {}))
        self.runtime_log_level = str(runtime_logging_cfg.get("log_level", "summary") or "summary").strip().lower()
        self.runtime_quiet = bool(runtime_logging_cfg.get("quiet", False))
        self.floor_manager = FloorManager(config=config)
        self.floor_states: Dict[str, FloorState] = {}
        self.frame_history: List[Dict[str, Any]] = []
        self.last_floor_observation: Optional[Dict[str, Any]] = None
        self.last_tracking_report: Dict[str, Any] = {}
        self.last_room_markers: Optional[bool] = None
        self.last_floor_diagnostics: Dict[str, Any] = {}
        self.last_finalization_report: Dict[str, Any] = {}
        self.last_merge_profile: Dict[str, Any] = {}
        self.last_export_profile: Dict[str, Any] = {}
        self._latest_export_cache: Dict[str, Any] = {}
        self._latest_object_room_metadata_snapshot: Dict[str, Any] = {}
        self._next_world_room_id = 1

    def _log_verbose(self, message: str) -> None:
        if self.runtime_quiet:
            return
        if self.runtime_log_level != "verbose":
            return
        print(message)

    def _log_warning(self, message: str) -> None:
        print(message)

    def observe_frame(
        self,
        frame_idx: int,
        timestamp: float,
        pose_matrix: np.ndarray,
        points_xyzrgb: Optional[np.ndarray] = None,
        is_keyframe: bool = False,
    ) -> Dict[str, Any]:
        pose = np.asarray(pose_matrix, dtype=np.float32)
        pose_z = float(pose[2, 3])
        observation = self.floor_manager.observe_pose(
            frame_idx=int(frame_idx),
            timestamp=float(timestamp),
            pose_z=pose_z,
            is_keyframe=bool(is_keyframe),
        )
        payload = observation.to_dict()
        payload.update(
            {
                "position_xy": [round(float(pose[0, 3]), 3), round(float(pose[1, 3]), 3)],
                "is_keyframe": bool(is_keyframe),
            }
        )
        self.frame_history.append(payload)
        self.last_floor_observation = payload

        floor_state = None
        if observation.floor_id is not None:
            floor_state = self._get_or_create_floor_state(observation.floor_id)
            if observation.status == "stable":
                floor_state.stable_frame_count += 1
                floor_state.last_stable_frame_idx = int(frame_idx)
            elif observation.status in {"transition", "uncertain"}:
                floor_state.transition_frame_count += 1
                floor_state.last_transition_frame_idx = int(frame_idx)

        if points_xyzrgb is not None and observation.status == "stable" and floor_state is not None:
            floor_state.pending_chunks.append(np.asarray(points_xyzrgb, dtype=np.float64))
            floor_state.pending_chunk_frame_indices.append(int(frame_idx))
            floor_state.stable_keyframe_chunk_count += 1

        return payload

    def perform_segmentation(
        self,
        all_pred_box=None,
        debug_path: Optional[str] = None,
        count: int = 0,
    ):
        active_floor_id = (self.last_floor_observation or {}).get("floor_id")
        active_status = (self.last_floor_observation or {}).get("status")
        if active_floor_id is None or active_status != "stable":
            self.last_tracking_report = {
                "active_floor_id": active_floor_id,
                "active_floor_status": active_status,
                "matched": [],
                "new_rooms": [],
                "per_floor": {},
                "reason": "current_frame_not_assigned_to_stable_floor",
            }
            return None
        return self._perform_floor_segmentation(
            floor_id=str(active_floor_id),
            all_pred_box=all_pred_box,
            debug_path=debug_path,
            count=count,
            active_status=str(active_status),
            update_last_tracking=True,
            trigger_reason="scheduled_interval",
        )

    def flush_pending_floor_segments(
        self,
        all_pred_box=None,
        debug_path: Optional[str] = None,
        count: int = 0,
    ) -> Dict[str, Any]:
        canonical_floors = canonicalize_floors(self.floor_manager.export_floors())
        floor_ids = [str(item["floor_id"]) for item in canonical_floors]
        floor_export_lookup = build_floor_lookup(canonical_floors)
        floors = []
        segmented_floor_ids: List[str] = []
        segmented_display_floor_ids: List[str] = []
        for floor_id in floor_ids:
            floor_meta = dict(floor_export_lookup.get(floor_id) or {})
            floor_state = self.floor_states.get(floor_id)
            if floor_state is None:
                floors.append(
                    {
                        "floor_id": floor_id,
                        "display_floor_id": floor_meta.get("display_floor_id", floor_id),
                        "display_order": floor_meta.get("display_order"),
                        "pending_chunk_count_before": 0,
                        "segmented": False,
                        "reason": "no_floor_state",
                    }
                )
                continue
            pending_chunk_count = self._pending_chunk_count(floor_state)
            needs_flush = pending_chunk_count > 0
            entry = {
                "floor_id": floor_id,
                "display_floor_id": floor_meta.get("display_floor_id", floor_id),
                "display_order": floor_meta.get("display_order"),
                "pending_chunk_count_before": int(pending_chunk_count),
                "stable_frame_count": int(floor_state.stable_frame_count),
                "stable_keyframe_chunk_count": int(floor_state.stable_keyframe_chunk_count),
                "last_segmentation_frame_idx": None if floor_state.last_segmentation_frame_idx is None else int(floor_state.last_segmentation_frame_idx),
                "segmented": False,
                "reason": "no_pending_chunks",
            }
            if needs_flush:
                markers = self._perform_floor_segmentation(
                    floor_id=floor_id,
                    all_pred_box=all_pred_box,
                    debug_path=debug_path,
                    count=count,
                    active_status="finalize",
                    update_last_tracking=False,
                    trigger_reason="end_of_sequence_flush",
                )
                entry["segmented"] = markers is not None
                entry["reason"] = "segmented" if markers is not None else "segmentation_returned_none"
                if markers is not None:
                    segmented_floor_ids.append(floor_id)
                    segmented_display_floor_ids.append(str(entry["display_floor_id"]))
            floors.append(entry)
        report = {
            "frame_idx": int(count),
            "segmented_floor_count": int(len(segmented_floor_ids)),
            "segmented_floor_ids": segmented_floor_ids,
            "segmented_display_floor_ids": segmented_display_floor_ids,
            "floors": floors,
        }
        self.last_finalization_report = report
        return report

    def _perform_floor_segmentation(
        self,
        floor_id: str,
        all_pred_box=None,
        debug_path: Optional[str] = None,
        count: int = 0,
        active_status: Optional[str] = None,
        update_last_tracking: bool = False,
        trigger_reason: str = "manual",
    ):
        floor_state = self._get_or_create_floor_state(floor_id)
        pending_chunk_count = self._pending_chunk_count(floor_state)
        pending_chunk_frame_indices = [int(item) for item in floor_state.pending_chunk_frame_indices]
        merge_t0 = time.perf_counter()
        merged_points = self._merge_floor_points(floor_state)
        merge_total_sec = time.perf_counter() - merge_t0
        if merged_points is None or len(merged_points) == 0:
            if update_last_tracking:
                self.last_tracking_report = {
                    "active_floor_id": floor_id,
                    "active_floor_status": active_status,
                    "matched": [],
                    "new_rooms": [],
                    "per_floor": {},
                    "reason": "no_points_available_for_active_floor",
                }
            return None

        floor_debug_dir = None
        if debug_path:
            floor_debug_dir = os.path.join(debug_path, floor_id)
            os.makedirs(floor_debug_dir, exist_ok=True)

        markers = floor_state.segmenter.perform_segmentation(
            np.ascontiguousarray(merged_points[:, :3], dtype=np.float64),
            all_pred_box,
            debug_path=floor_debug_dir,
            count=count,
        )
        segmentation_run = self._record_segmentation_run(
            floor_id=floor_id,
            floor_state=floor_state,
            trigger_reason=trigger_reason,
            active_status=active_status,
            count=count,
            pending_chunk_count=pending_chunk_count,
            pending_chunk_frame_indices=pending_chunk_frame_indices,
            merged_point_count=0 if merged_points is None else int(len(merged_points)),
            success=markers is not None,
            merge_total_sec=merge_total_sec,
        )
        if markers is None:
            if update_last_tracking:
                self.last_tracking_report = {
                    "active_floor_id": floor_id,
                    "active_floor_status": active_status,
                    "matched": [],
                    "new_rooms": [],
                    "per_floor": {},
                    "reason": "segmentation_returned_none",
                    "segmentation_run": segmentation_run,
                }
            return None

        floor_state.last_segmentation_frame_idx = int(count)
        floor_state.segmentation_trigger_count += 1
        mapped_report = self._map_tracking_report(floor_id, floor_state.segmenter.last_tracking_report)
        mapped_report["segmentation_trigger_reason"] = trigger_reason
        mapped_report["segmentation_run"] = segmentation_run
        floor_state.last_tracking_report = mapped_report
        if update_last_tracking:
            self.last_tracking_report = {
                "active_floor_id": floor_id,
                "active_floor_status": active_status,
                "matched": list(mapped_report.get("matched", [])),
                "new_rooms": list(mapped_report.get("new_rooms", [])),
                "per_floor": {floor_id: mapped_report},
                "segmentation_trigger_reason": trigger_reason,
                "segmentation_run": segmentation_run,
            }
        self.last_room_markers = True
        return markers

    def get_vector_map_data(
        self,
        all_pred_box=None,
        count: Optional[int] = None,
        save_scene_graph_vis: bool = True,
        scene_graph_vis_dir: str = "./debug_room",
        instrumentation_context: str = "unspecified",
    ) -> Dict[str, Any]:
        export_t0 = time.perf_counter()
        floors = canonicalize_floors(self.floor_manager.export_floors())
        floor_lookup = build_floor_lookup(floors)
        segmentation_cache_token = self._build_segmentation_cache_token(floors)
        export_structure_cache_token = self._build_export_structure_cache_token(floors)
        prepared_object_state = self._prepare_object_export_state(all_pred_box)
        cached_export, reuse_diagnostics = self._evaluate_same_frame_reuse(
            count=count,
            segmentation_cache_token=segmentation_cache_token,
            object_state_signature=prepared_object_state.get("full_signature"),
            save_scene_graph_vis=save_scene_graph_vis,
            scene_graph_vis_dir=scene_graph_vis_dir,
        )
        structure_reuse_export = self._get_cached_export_for_structure_reuse(
            export_structure_cache_token=export_structure_cache_token,
        )
        if reuse_diagnostics.get("same_frame_reuse_eligible"):
            vector_data = cached_export["vector_data"]
            self.last_floor_diagnostics = dict(vector_data.get("floor_debug") or {})
            self._refresh_readonly_object_room_metadata_snapshot(
                frame_idx=count,
                object_cache_entries=cached_export.get("object_cache"),
            )
            cached_profile = dict(cached_export.get("profile") or {})
            self.last_export_profile = {
                "frame_idx": None if count is None else int(count),
                "instrumentation_context": str(instrumentation_context),
                "cache_hit": True,
                "build_executed": False,
                "cache_hit_kind": "same_frame_full_export_reuse",
                "total_sec": 0.0,
                "room_export_sec": 0.0,
                "vertical_transition_export_sec": 0.0,
                "object_export_sec": 0.0,
                "scene_graph_build_sec": 0.0,
                "spatial_relations_sec": 0.0,
                "anchor_build_sec": 0.0,
                "diagnostics_export_sec": 0.0,
                "object_export_reused_count": int(cached_profile.get("object_count", len(vector_data.get("objects", [])))),
                "object_export_rebuilt_count": 0,
                "room_local_delta_export_used": False,
                "room_local_delta_changed_room_count": 0,
                "room_local_delta_reused_room_count": int(cached_profile.get("room_count", len(vector_data.get("rooms", [])))),
                "room_local_delta_rebuilt_room_count": 0,
                "room_local_delta_unassigned_bucket_rebuilt": False,
                "changed_room_count": 0,
                "changed_room_ids": "",
                "changed_room_structure_changed_count": 0,
                "changed_room_structure_changed_ids": "",
                "changed_room_object_delta_count": 0,
                "changed_room_object_delta_ids": "",
                "changed_room_removed_count": 0,
                "changed_room_removed_ids": "",
                "changed_room_local_rebuild_used": False,
                "changed_room_locality_confident": True,
                "full_fallback_rebuild_used": False,
                "rebuilt_object_in_changed_rooms_count": 0,
                "room_count": int(cached_profile.get("room_count", len(vector_data.get("rooms", [])))),
                "gateway_count": int(cached_profile.get("gateway_count", len(vector_data.get("gateways", [])))),
                "vertical_transition_count": int(
                    cached_profile.get("vertical_transition_count", len(vector_data.get("vertical_transitions", [])))
                ),
                "object_count": int(cached_profile.get("object_count", len(vector_data.get("objects", [])))),
                "anchor_count": int(cached_profile.get("anchor_count", len(vector_data.get("anchors", [])))),
                **reuse_diagnostics,
            }
            return vector_data

        vector_data: Dict[str, Any] = {
            "map_info": {
                "resolution": self.resolution,
                "contract": "floor_aware_world_graph_v0_1",
                "teacher_facing_floor_order": "ascending_z_display_floor_id",
            },
            "floors": floors,
            "rooms": [],
            "gateways": [],
            "vertical_transitions": [],
            "objects": [],
            "anchors": [],
            "relationships": [],
            "frame_floor_assignments": self.floor_manager.export_assignment_history(),
            "room_segmentation_diagnostics": {},
            "vertical_transition_summary": {},
        }

        room_export_t0 = time.perf_counter()
        room_records, room_lookup = self._export_rooms_and_gateways(vector_data, floor_lookup)
        room_export_sec = time.perf_counter() - room_export_t0
        vector_data["room_floor_validation"] = self._build_room_floor_validation(room_records)
        vt_export_t0 = time.perf_counter()
        vector_data["vertical_transitions"] = canonicalize_vertical_transitions(
            self._build_vertical_transitions(room_records),
            floor_lookup,
        )
        vector_data["vertical_transition_summary"] = build_vertical_transition_summary(
            vector_data["vertical_transitions"],
            floor_lookup,
        )
        vertical_transition_export_sec = time.perf_counter() - vt_export_t0
        latest_cached_export = self._get_latest_export_cache()
        room_cache = self._build_room_export_cache(room_records)
        object_export_t0 = time.perf_counter()
        object_reuse_cache = None if structure_reuse_export is None else structure_reuse_export.get("object_cache")
        delta_export_metrics = {
            "room_local_delta_export_used": False,
            "room_local_delta_changed_room_count": 0,
            "room_local_delta_reused_room_count": 0,
            "room_local_delta_rebuilt_room_count": 0,
            "room_local_delta_unassigned_bucket_rebuilt": False,
            "changed_room_count": 0,
            "changed_room_ids": "",
            "changed_room_structure_changed_count": 0,
            "changed_room_structure_changed_ids": "",
            "changed_room_object_delta_count": 0,
            "changed_room_object_delta_ids": "",
            "changed_room_removed_count": 0,
            "changed_room_removed_ids": "",
            "changed_room_local_rebuild_used": False,
            "changed_room_locality_confident": False,
            "full_fallback_rebuild_used": False,
            "rebuilt_object_in_changed_rooms_count": 0,
        }
        delta_export_result = None
        if (
            latest_cached_export is not None
            and (
                reuse_diagnostics.get("same_frame_reuse_blocker") == "object_state_signature_mismatch"
                or structure_reuse_export is None
            )
        ):
            delta_export_result = self._build_room_local_delta_object_exports(
                prepared_object_state,
                floor_lookup,
                room_lookup,
                room_records=room_records,
                room_cache=room_cache,
                cached_export=latest_cached_export,
            )
        if delta_export_result is not None:
            (
                vector_data["objects"],
                embedding_lookup,
                object_cache_entries,
                object_export_reused_count,
                delta_export_metrics,
            ) = delta_export_result
        else:
            (
                vector_data["objects"],
                embedding_lookup,
                object_cache_entries,
                object_export_reused_count,
            ) = self._build_object_exports(
                prepared_object_state,
                floor_lookup,
                room_lookup,
                reuse_cache=object_reuse_cache,
            )
            delta_export_metrics["full_fallback_rebuild_used"] = bool(
                structure_reuse_export is None and latest_cached_export is not None
            )
        object_export_sec = time.perf_counter() - object_export_t0
        object_export_rebuilt_count = max(len(vector_data["objects"]) - int(object_export_reused_count), 0)
        delta_export_metrics["rebuilt_object_in_changed_rooms_count"] = int(
            delta_export_metrics.get("rebuilt_object_in_changed_rooms_count") or object_export_rebuilt_count
        )

        sg_build_t0 = time.perf_counter()
        sg = SemanticSceneGraph()
        for floor in floors:
            sg.add_floor_node(
                FloorNode(
                    id=str(floor["floor_id"]),
                    floor_index=int(floor.get("floor_index", 0)),
                    z_min=float(floor.get("z_min", 0.0) or 0.0),
                    z_max=float(floor.get("z_max", 0.0) or 0.0),
                    z_center=float(floor.get("z_center", 0.0) or 0.0),
                    confidence=float(floor.get("confidence", 0.0) or 0.0),
                    status=str(floor.get("status", "tentative")),
                    support_statistics=dict(floor.get("support_statistics") or {}),
                )
            )

        for room_info in room_records:
            sg.add_room_node(
                RoomNode(
                    id=f"room_{room_info['id']}",
                    room_type=str(room_info.get("room_type", "unknown")),
                    polygon=room_info.get("polygon", []),
                    floor_id=room_info.get("floor_id"),
                    floor_assignment_confidence=float(room_info.get("floor_assignment_confidence", 0.0) or 0.0),
                    status=str(room_info.get("status", "confirmed")),
                )
            )

        for obj_info in vector_data["objects"]:
            room_uuid = int(obj_info.get("room_uuid", -1))
            room_id = f"room_{room_uuid}" if room_uuid >= 0 else "room_unknown"
            if room_id == "room_unknown" and room_id not in sg.graph.nodes:
                sg.add_room_node(RoomNode(id=room_id, room_type="unknown", polygon=[]))
            clip_feature = None
            embedding_ref = obj_info.get("embedding_ref")
            if embedding_ref is not None:
                clip_feature = embedding_lookup.get(embedding_ref)
            o_node = ObjectNode(
                id=f"obj_{obj_info['id']}",
                center=tuple(obj_info.get("pose_3d", [0.0, 0.0, 0.0])),
                bbox=tuple(obj_info.get("size", [0.0, 0.0, 0.0])),
                label=obj_info.get("label", obj_info.get("category", "unknown")),
                category=obj_info.get("category", obj_info.get("label", "unknown")),
                clip_feature=clip_feature,
                confidence=float(obj_info.get("score", 1.0)),
                room_id=room_id,
                yaw=float(obj_info.get("yaw", 0.0)),
                footprint=obj_info.get("footprint_2d"),
                semantic_observations=obj_info.get("semantic_observations"),
                detection_confidence=float(obj_info.get("detection_confidence", obj_info.get("score", 1.0))),
                semantic_confidence=float(obj_info.get("semantic_confidence", obj_info.get("score", 1.0))),
                association_confidence=float(obj_info.get("association_confidence", 1.0)),
                semantic_gap=float(obj_info.get("semantic_gap", 0.0)),
                view_quality=float(obj_info.get("view_quality", 1.0)),
                floor_id=obj_info.get("floor_id"),
            )
            sg.add_object_node(o_node)
            if room_id in sg.graph.nodes:
                sg.add_inside_relation(o_node.id, room_id)
        scene_graph_build_sec = time.perf_counter() - sg_build_t0

        spatial_rel_t0 = time.perf_counter()
        sg.compute_spatial_relations(dist_threshold=1.0, z_tolerance=0.2)
        spatial_relations_sec = time.perf_counter() - spatial_rel_t0
        sg.reset_anchor_profile()
        anchor_t0 = time.perf_counter()
        (
            anchor_cache,
            anchor_reused_count,
            anchor_rebuilt_count,
            anchor_reuse_room_count,
            anchor_rebuild_room_count,
        ) = self._build_anchor_layer_with_cached_room_reuse(
            sg,
            room_records=room_records,
            objects=vector_data["objects"],
            prepared_object_state=prepared_object_state,
            cached_export=(
                structure_reuse_export
                if structure_reuse_export is not None
                else latest_cached_export
                if bool(delta_export_metrics.get("changed_room_local_rebuild_used"))
                else None
            ),
        )
        anchor_profile = sg.get_anchor_profile()
        anchor_build_sec = time.perf_counter() - anchor_t0

        for source, target, data in sg.graph.edges(data=True):
            vector_data["relationships"].append(
                {
                    "source": source,
                    "target": target,
                    "relation": data["relation"],
                }
            )
        vector_data["anchors"] = self._attach_display_floor_metadata(sg.export_anchor_data(), floor_lookup)
        diagnostics_t0 = time.perf_counter()
        vector_data["room_segmentation_diagnostics"] = self._build_segmentation_diagnostics(floors)
        vector_data["floor_debug"] = self._build_floor_debug_summary(vector_data)
        diagnostics_export_sec = time.perf_counter() - diagnostics_t0
        self.last_floor_diagnostics = vector_data["floor_debug"]

        if save_scene_graph_vis:
            os.makedirs(scene_graph_vis_dir, exist_ok=True)
            vis_path = os.path.join(
                scene_graph_vis_dir,
                f"scene_graph_{count if count is not None else 'latest'}.png",
            )
            try:
                sg.visualize_bev_graph(save_path=vis_path)
            except Exception:
                pass

        self.last_export_profile = {
            "frame_idx": None if count is None else int(count),
            "instrumentation_context": str(instrumentation_context),
            "cache_hit": False,
            "build_executed": True,
            "total_sec": float(time.perf_counter() - export_t0),
            "room_export_sec": float(room_export_sec),
            "vertical_transition_export_sec": float(vertical_transition_export_sec),
            "object_export_sec": float(object_export_sec),
            "scene_graph_build_sec": float(scene_graph_build_sec),
            "spatial_relations_sec": float(spatial_relations_sec),
            "anchor_build_sec": float(anchor_build_sec),
            "anchor_rebuild_room_anchor_sec": float(anchor_profile.get("room_anchor_total_sec", 0.0)),
            "anchor_rebuild_object_anchor_sec": float(anchor_profile.get("object_anchor_total_sec", 0.0)),
            "anchor_rebuild_candidate_generation_sec": float(anchor_profile.get("candidate_generation_sec", 0.0)),
            "anchor_rebuild_validate_sec": float(anchor_profile.get("validate_sec", 0.0)),
            "anchor_rebuild_score_sec": float(anchor_profile.get("score_sec", 0.0)),
            "anchor_rebuild_fallback_sec": float(anchor_profile.get("fallback_sec", 0.0)),
            "anchor_rebuild_insert_sec": float(anchor_profile.get("insert_sec", 0.0)),
            "anchor_rebuild_room_candidate_count": int(anchor_profile.get("room_anchor_candidate_count", 0.0)),
            "anchor_rebuild_object_candidate_count": int(anchor_profile.get("object_anchor_candidate_count", 0.0)),
            "anchor_rebuild_valid_candidate_count": int(anchor_profile.get("valid_candidate_count", 0.0)),
            "anchor_rebuild_fallback_candidate_count": int(anchor_profile.get("fallback_candidate_count", 0.0)),
            "anchor_reused_count": int(anchor_reused_count),
            "anchor_rebuilt_count": int(anchor_rebuilt_count),
            "anchor_reuse_room_count": int(anchor_reuse_room_count),
            "anchor_rebuild_room_count": int(anchor_rebuild_room_count),
            "diagnostics_export_sec": float(diagnostics_export_sec),
            "object_export_reused_count": int(object_export_reused_count),
            "object_export_rebuilt_count": int(object_export_rebuilt_count),
            "room_count": int(len(vector_data.get("rooms", []))),
            "gateway_count": int(len(vector_data.get("gateways", []))),
            "vertical_transition_count": int(len(vector_data.get("vertical_transitions", []))),
            "object_count": int(len(vector_data.get("objects", []))),
            "anchor_count": int(len(vector_data.get("anchors", []))),
            **delta_export_metrics,
            **reuse_diagnostics,
        }
        self._latest_export_cache = {
            "frame_idx": None if count is None else int(count),
            "segmentation_cache_token": segmentation_cache_token,
            "export_structure_cache_token": export_structure_cache_token,
            "object_state_signature": prepared_object_state.get("full_signature"),
            "vector_data": vector_data,
            "profile": dict(self.last_export_profile),
            "room_cache": room_cache,
            "object_cache": object_cache_entries,
            "anchor_cache": anchor_cache,
            "save_scene_graph_vis": bool(save_scene_graph_vis),
            "scene_graph_vis_dir": str(scene_graph_vis_dir),
        }
        self._refresh_readonly_object_room_metadata_snapshot(
            frame_idx=count,
            object_cache_entries=object_cache_entries,
        )

        return vector_data

    def _evaluate_same_frame_reuse(
        self,
        *,
        count: Optional[int],
        segmentation_cache_token: Tuple[Any, ...],
        object_state_signature: Optional[str],
        save_scene_graph_vis: bool,
        scene_graph_vis_dir: str,
    ) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        diagnostics = {
            "same_frame_cache_available": False,
            "same_frame_cache_frame_match": False,
            "same_frame_segmentation_token_match": False,
            "same_frame_object_state_match": False,
            "same_frame_visualization_match": False,
            "same_frame_reuse_eligible": False,
            "same_frame_reuse_blocker": "no_prior_export",
        }
        cached = self._latest_export_cache or {}
        if not cached:
            return None, diagnostics
        diagnostics["same_frame_cache_available"] = True
        if count is None:
            diagnostics["same_frame_reuse_blocker"] = "missing_frame_index"
            return None, diagnostics
        diagnostics["same_frame_cache_frame_match"] = cached.get("frame_idx") == int(count)
        if not diagnostics["same_frame_cache_frame_match"]:
            diagnostics["same_frame_reuse_blocker"] = "frame_mismatch"
            return None, diagnostics
        diagnostics["same_frame_segmentation_token_match"] = (
            cached.get("segmentation_cache_token") == segmentation_cache_token
        )
        if not diagnostics["same_frame_segmentation_token_match"]:
            diagnostics["same_frame_reuse_blocker"] = "segmentation_token_mismatch"
            return None, diagnostics

        diagnostics["same_frame_object_state_match"] = (
            cached.get("object_state_signature") == object_state_signature
        )
        diagnostics["same_frame_visualization_match"] = self._is_scene_graph_vis_compatible(
            cached,
            save_scene_graph_vis=save_scene_graph_vis,
            scene_graph_vis_dir=scene_graph_vis_dir,
        )
        if not diagnostics["same_frame_object_state_match"]:
            diagnostics["same_frame_reuse_blocker"] = "object_state_signature_mismatch"
            return cached, diagnostics
        if not diagnostics["same_frame_visualization_match"]:
            diagnostics["same_frame_reuse_blocker"] = "visualization_mismatch"
            return cached, diagnostics
        diagnostics["same_frame_reuse_eligible"] = True
        diagnostics["same_frame_reuse_blocker"] = "none"
        return cached, diagnostics

    def _is_scene_graph_vis_compatible(
        self,
        cached_export: Dict[str, Any],
        *,
        save_scene_graph_vis: bool,
        scene_graph_vis_dir: str,
    ) -> bool:
        if not save_scene_graph_vis:
            return True
        return bool(
            cached_export.get("save_scene_graph_vis")
            and str(cached_export.get("scene_graph_vis_dir")) == str(scene_graph_vis_dir)
        )

    def _get_cached_export_for_structure_reuse(
        self,
        *,
        export_structure_cache_token: str,
    ) -> Optional[Dict[str, Any]]:
        cached = self._latest_export_cache or {}
        if not cached:
            return None
        if cached.get("export_structure_cache_token") != export_structure_cache_token:
            return None
        return cached

    def _get_latest_export_cache(self) -> Optional[Dict[str, Any]]:
        cached = self._latest_export_cache or {}
        if not cached:
            return None
        return cached

    def get_readonly_object_room_metadata_snapshot(self) -> Dict[str, Any]:
        snapshot = dict(self._latest_object_room_metadata_snapshot or {})
        if not snapshot:
            return {}
        by_object_id = {}
        for object_id, metadata in dict(snapshot.get("by_object_id") or {}).items():
            by_object_id[int(object_id)] = dict(metadata or {})
        return {
            "frame_idx": None if snapshot.get("frame_idx") is None else int(snapshot.get("frame_idx")),
            "object_count": int(snapshot.get("object_count", len(by_object_id))),
            "by_object_id": by_object_id,
        }

    def _room_assignment_support_count(self, room_assignment: Dict[str, Any]) -> int:
        votes = dict((room_assignment or {}).get("votes") or {})
        if not votes:
            return 0
        try:
            return int(max(int(value) for value in votes.values()))
        except Exception:
            return 0

    def _cached_room_assignment_is_trusted(
        self,
        *,
        room_uuid: int,
        floor_id: Optional[str],
        room_assignment: Dict[str, Any],
    ) -> bool:
        if int(room_uuid) < 0 or floor_id in (None, ""):
            return False
        support_count = self._room_assignment_support_count(room_assignment)
        center_label = room_assignment.get("center_label")
        center_label_int = -1 if center_label in (None, "") else int(center_label)
        return bool(support_count >= 2 and (center_label_int > 0 or support_count >= 3))

    def _refresh_readonly_object_room_metadata_snapshot(
        self,
        *,
        frame_idx: Optional[int],
        object_cache_entries: Optional[Dict[int, Dict[str, Any]]],
    ) -> None:
        cache_lookup = dict(object_cache_entries or {})
        by_object_id: Dict[int, Dict[str, Any]] = {}
        for instance_id, raw_cached_entry in cache_lookup.items():
            cached_entry = dict(raw_cached_entry or {})
            cached_object = dict(cached_entry.get("object") or {})
            if not cached_object:
                continue

            room_assignment = dict(cached_object.get("room_assignment") or {})
            floor_id = cached_object.get("floor_id")
            room_uuid = int(cached_object.get("room_uuid", -1) or -1)
            center_label = room_assignment.get("center_label")
            center_label_int = -1 if center_label in (None, "") else int(center_label)
            support_count = self._room_assignment_support_count(room_assignment)
            normalized_floor_id = None if floor_id in (None, "") else str(floor_id)
            by_object_id[int(instance_id)] = {
                "room_uuid": int(room_uuid),
                "floor_id": normalized_floor_id,
                "support_count": int(support_count),
                "center_label": int(center_label_int),
                "trusted": bool(
                    self._cached_room_assignment_is_trusted(
                        room_uuid=room_uuid,
                        floor_id=normalized_floor_id,
                        room_assignment=room_assignment,
                    )
                ),
                "metadata_available": True,
            }

        self._latest_object_room_metadata_snapshot = {
            "frame_idx": None if frame_idx is None else int(frame_idx),
            "object_count": int(len(by_object_id)),
            "by_object_id": by_object_id,
        }

    def _build_anchor_layer_with_cached_room_reuse(
        self,
        sg: SemanticSceneGraph,
        *,
        room_records: Sequence[Dict[str, Any]],
        objects: Sequence[Dict[str, Any]],
        prepared_object_state: Dict[str, Any],
        cached_export: Optional[Dict[str, Any]],
    ) -> Tuple[Dict[str, Any], int, int, int, int]:
        object_signature_by_id = self._build_object_signature_lookup(prepared_object_state)
        room_signatures = self._build_anchor_room_signatures(
            room_records=room_records,
            objects=objects,
            object_signature_by_id=object_signature_by_id,
        )
        anchor_cache: Dict[str, Any] = {
            "room_signatures": dict(room_signatures),
            "anchors_by_room": {},
        }
        reused_anchor_count = 0
        reused_room_count = 0
        rebuilt_room_count = 0
        rebuilt_anchor_nodes: List[AnchorNode] = []
        reusable_room_ids: set[str] = set()

        cached_anchor_cache = None if cached_export is None else dict(cached_export.get("anchor_cache") or {})

        cached_room_signatures = dict((cached_anchor_cache or {}).get("room_signatures") or {})
        cached_anchors_by_room = dict((cached_anchor_cache or {}).get("anchors_by_room") or {})
        for room_info in room_records:
            room_id = f"room_{int(room_info['id'])}"
            cached_signature = cached_room_signatures.get(room_id)
            if cached_signature != room_signatures.get(room_id):
                continue
            cached_payloads = list(cached_anchors_by_room.get(room_id) or [])
            reusable_anchors = self._deserialize_reusable_anchor_payloads(
                sg,
                room_id=room_id,
                anchor_payloads=cached_payloads,
            )
            if reusable_anchors is None:
                continue
            sg.insert_anchor_nodes_into_graph(reusable_anchors)
            anchor_cache["anchors_by_room"][room_id] = [dict(item) for item in cached_payloads]
            reusable_room_ids.add(room_id)
            reused_room_count += 1
            reused_anchor_count += len(reusable_anchors)

        if reused_room_count < 2 or reused_anchor_count == 0:
            anchors = sg.build_anchor_layer(debug=False)
            return (
                self._build_anchor_cache_from_export(
                    room_signatures=room_signatures,
                    anchor_exports=[anchor.get_attributes() for anchor in anchors],
                ),
                0,
                int(len(anchors)),
                0,
                int(len(room_records)),
            )

        for room_info in room_records:
            room_id = f"room_{int(room_info['id'])}"
            if room_id in reusable_room_ids:
                continue
            rebuilt_room_count += 1
            room_anchor_nodes: List[AnchorNode] = []
            room_anchor = sg.generate_room_anchor(room_id, debug=False)
            if room_anchor is not None:
                room_anchor_nodes.append(room_anchor)
            for obj in sg._objects_in_room(room_id):
                object_anchor = sg.generate_object_anchor(obj.id, debug=False)
                if object_anchor is not None:
                    room_anchor_nodes.append(object_anchor)
            if room_anchor_nodes:
                sg.insert_anchor_nodes_into_graph(room_anchor_nodes)
                rebuilt_anchor_nodes.extend(room_anchor_nodes)
            anchor_cache["anchors_by_room"][room_id] = [
                dict(anchor.get_attributes()) for anchor in room_anchor_nodes
            ]

        return (
            anchor_cache,
            int(reused_anchor_count),
            int(len(rebuilt_anchor_nodes)),
            int(reused_room_count),
            int(rebuilt_room_count),
        )

    def _build_anchor_cache_from_export(
        self,
        *,
        room_signatures: Dict[str, str],
        anchor_exports: Sequence[Dict[str, Any]],
    ) -> Dict[str, Any]:
        anchors_by_room: Dict[str, List[Dict[str, Any]]] = {}
        for room_id in room_signatures:
            anchors_by_room[room_id] = []
        for anchor in anchor_exports:
            room_id = str(anchor.get("room_id") or "")
            anchors_by_room.setdefault(room_id, []).append(dict(anchor))
        return {
            "room_signatures": dict(room_signatures),
            "anchors_by_room": anchors_by_room,
        }

    def _build_object_signature_lookup(self, prepared_object_state: Dict[str, Any]) -> Dict[int, str]:
        instance_ids_value = prepared_object_state.get("instance_ids")
        if instance_ids_value is None:
            instance_ids = np.zeros((0,), dtype=np.int64)
        else:
            instance_ids = np.asarray(instance_ids_value, dtype=np.int64)
        per_object_signatures = list(prepared_object_state.get("per_object_signatures") or [])
        return {
            int(instance_ids[idx]): str(per_object_signatures[idx])
            for idx in range(min(len(instance_ids), len(per_object_signatures)))
        }

    def _build_anchor_room_signatures(
        self,
        *,
        room_records: Sequence[Dict[str, Any]],
        objects: Sequence[Dict[str, Any]],
        object_signature_by_id: Dict[int, str],
    ) -> Dict[str, str]:
        objects_by_room: Dict[str, List[Dict[str, Any]]] = {}
        for obj in objects:
            room_id = obj.get("room_id")
            if room_id is None:
                continue
            objects_by_room.setdefault(str(room_id), []).append(dict(obj))

        room_signatures: Dict[str, str] = {}
        for room_info in room_records:
            room_id = f"room_{int(room_info['id'])}"
            hasher = hashlib.blake2b(digest_size=16)
            hasher.update(room_id.encode("utf-8"))
            room_payload = self._room_cache_payload(room_info)
            hasher.update(
                json.dumps(room_payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
            )
            room_objects = sorted(
                objects_by_room.get(room_id, []),
                key=lambda item: int(item.get("id", -1)),
            )
            for obj in room_objects:
                object_id = int(obj.get("id", -1))
                object_signature = object_signature_by_id.get(object_id)
                if object_signature is None:
                    object_signature = f"missing_signature:{object_id}"
                hasher.update(f"{object_id}:{object_signature}".encode("utf-8"))
            room_signatures[room_id] = hasher.hexdigest()
        return room_signatures

    def _room_cache_payload(self, room_record: Dict[str, Any]) -> Dict[str, Any]:
        polygon = [
            [round(float(point[0]), 3), round(float(point[1]), 3)]
            for point in list(room_record.get("polygon") or [])
        ]
        center = list(room_record.get("center") or [])
        return {
            "room_id": str(room_record.get("room_id") or f"room_{int(room_record.get('id', -1))}"),
            "floor_id": room_record.get("floor_id"),
            "room_type": str(room_record.get("room_type", "unknown")),
            "status": str(room_record.get("status", "confirmed")),
            "polygon": polygon,
            "center": [round(float(value), 3) for value in center[:2]],
            "area_m2": round(float(room_record.get("area_m2", 0.0) or 0.0), 3),
        }

    def _room_signature_from_record(self, room_record: Dict[str, Any]) -> str:
        hasher = hashlib.blake2b(digest_size=16)
        hasher.update(
            json.dumps(
                self._room_cache_payload(room_record),
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        )
        return hasher.hexdigest()

    def _build_room_export_cache(self, room_records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
        room_signatures: Dict[str, str] = {}
        rooms_by_id: Dict[str, Dict[str, Any]] = {}
        for room_info in room_records:
            room_id = f"room_{int(room_info['id'])}"
            room_signatures[room_id] = self._room_signature_from_record(room_info)
            rooms_by_id[room_id] = self._room_cache_payload(room_info)
        return {
            "room_signatures": room_signatures,
            "rooms_by_id": rooms_by_id,
        }

    def _build_object_export_floor_vote_cache(self) -> Dict[str, Dict[str, Any]]:
        cache: Dict[str, Dict[str, Any]] = {}
        for floor_id, floor_state in self.floor_states.items():
            segmenter = floor_state.segmenter
            markers = getattr(segmenter, "last_room_markers", None)
            if markers is None:
                continue
            cache[str(floor_id)] = {
                "floor_state": floor_state,
                "wall_label": int(np.max(markers)),
            }
        return cache

    def _encode_room_ids(self, room_ids: Iterable[str]) -> str:
        cleaned = sorted({str(room_id) for room_id in room_ids if room_id not in (None, "", "__unassigned__")})
        return "|".join(cleaned)

    def _deserialize_reusable_anchor_payloads(
        self,
        sg: SemanticSceneGraph,
        *,
        room_id: str,
        anchor_payloads: Sequence[Dict[str, Any]],
    ) -> Optional[List[AnchorNode]]:
        if room_id not in sg.graph.nodes:
            return None
        anchors: List[AnchorNode] = []
        for payload in anchor_payloads:
            target_id = str(payload.get("target_id") or "")
            payload_room_id = str(payload.get("room_id") or "")
            if payload_room_id != room_id:
                return None
            if target_id not in sg.graph.nodes:
                return None
            position = payload.get("position") or [0.0, 0.0, 0.0]
            candidate_positions = payload.get("candidate_positions")
            anchors.append(
                AnchorNode(
                    id=str(payload.get("id")),
                    anchor_type=str(payload.get("anchor_type", "room")),
                    position=(
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                    room_id=payload_room_id,
                    target_id=target_id,
                    valid=bool(payload.get("valid", True)),
                    score=float(payload.get("score", 0.0)),
                    floor_id=None if payload.get("floor_id") is None else str(payload.get("floor_id")),
                    candidate_positions=None
                    if candidate_positions is None
                    else [tuple(map(float, item)) for item in candidate_positions],
                )
            )
        return anchors

    def _build_segmentation_cache_token(
        self,
        floors: Sequence[Dict[str, Any]],
    ) -> Tuple[Any, ...]:
        floor_token = tuple(
            (
                str(item.get("floor_id")),
                int(item.get("floor_index", 0) or 0),
                round(float(item.get("z_center", 0.0) or 0.0), 3),
                str(item.get("status", "unknown")),
            )
            for item in floors
        )
        floor_state_token = []
        for floor_id, floor_state in sorted(self.floor_states.items(), key=lambda item: item[0]):
            markers = floor_state.segmenter.last_room_markers
            markers_digest = None
            if markers is not None:
                marker_array = np.ascontiguousarray(np.asarray(markers, dtype=np.int32))
                markers_digest = hashlib.blake2b(marker_array.tobytes(), digest_size=8).hexdigest()
            floor_state_token.append(
                (
                    str(floor_id),
                    None if floor_state.last_segmentation_frame_idx is None else int(floor_state.last_segmentation_frame_idx),
                    int(len(floor_state.local_to_world_room_id)),
                    markers_digest,
                    int(len(getattr(floor_state.segmenter, "last_gateways", []) or [])),
                )
            )
        return (
            int(len(self.frame_history)),
            floor_token,
            tuple(floor_state_token),
        )

    def _build_export_structure_cache_token(
        self,
        floors: Sequence[Dict[str, Any]],
    ) -> str:
        hasher = hashlib.blake2b(digest_size=16)
        for floor in floors:
            hasher.update(
                json.dumps(
                    {
                        "floor_id": str(floor.get("floor_id")),
                        "floor_index": int(floor.get("floor_index", 0) or 0),
                        "display_floor_id": floor.get("display_floor_id"),
                        "display_order": None if floor.get("display_order") is None else int(floor.get("display_order")),
                        "status": str(floor.get("status", "unknown")),
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            )
        for floor_id, floor_state in sorted(self.floor_states.items(), key=lambda item: item[0]):
            markers = floor_state.segmenter.last_room_markers
            markers_digest = None
            if markers is not None:
                marker_array = np.ascontiguousarray(np.asarray(markers, dtype=np.int32))
                markers_digest = hashlib.blake2b(marker_array.tobytes(), digest_size=8).hexdigest()
            hasher.update(
                json.dumps(
                    {
                        "floor_id": str(floor_id),
                        "last_segmentation_frame_idx": None
                        if floor_state.last_segmentation_frame_idx is None
                        else int(floor_state.last_segmentation_frame_idx),
                        "local_to_world_room_id": [
                            [int(local_id), int(world_id)]
                            for local_id, world_id in sorted((floor_state.local_to_world_room_id or {}).items())
                        ],
                        "markers_digest": markers_digest,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            )
        return hasher.hexdigest()

    def _prepare_object_export_state(self, all_pred_box) -> Dict[str, Any]:
        if all_pred_box is None:
            return {
                "is_empty": True,
                "box_tensors": np.zeros((0, 7), dtype=np.float32),
                "categories": np.asarray([], dtype=object),
                "instance_ids": np.zeros((0,), dtype=np.int64),
                "scores": np.zeros((0,), dtype=np.float32),
                "semantic_confidences": np.zeros((0,), dtype=np.float32),
                "association_confidences": np.zeros((0,), dtype=np.float32),
                "semantic_gaps": np.zeros((0,), dtype=np.float32),
                "view_qualities": np.zeros((0,), dtype=np.float32),
                "embeddings": None,
                "has_embeddings": False,
                "per_object_signatures": [],
                "full_signature": "empty",
            }

        box_tensors = self._to_numpy_array(all_pred_box.pred_boxes_3d.tensor, dtype=np.float32)
        categories = np.asarray(all_pred_box.categories, dtype=object)
        instance_ids = self._to_numpy_array(all_pred_box.init_id, dtype=np.int64)
        scores = self._to_numpy_array(
            getattr(all_pred_box, "scores", np.ones(len(box_tensors), dtype=np.float32)),
            dtype=np.float32,
        )
        semantic_confidences = self._to_numpy_array(
            getattr(all_pred_box, "semantic_confidences", scores),
            dtype=np.float32,
        )
        association_confidences = self._to_numpy_array(
            getattr(all_pred_box, "association_confidences", np.ones(len(box_tensors), dtype=np.float32)),
            dtype=np.float32,
        )
        semantic_gaps = self._to_numpy_array(
            getattr(all_pred_box, "semantic_gaps", np.zeros(len(box_tensors), dtype=np.float32)),
            dtype=np.float32,
        )
        view_qualities = self._to_numpy_array(
            getattr(all_pred_box, "view_qualities", np.ones(len(box_tensors), dtype=np.float32)),
            dtype=np.float32,
        )
        has_embeddings = hasattr(all_pred_box, "embeddings")
        embeddings = None
        if has_embeddings:
            embeddings = self._to_numpy_array(getattr(all_pred_box, "embeddings"), dtype=np.float32)

        per_object_signatures: List[str] = []
        full_hasher = hashlib.blake2b(digest_size=16)
        for idx in range(len(box_tensors)):
            object_hasher = hashlib.blake2b(digest_size=16)
            object_hasher.update(np.ascontiguousarray(box_tensors[idx], dtype=np.float32).tobytes())
            object_hasher.update(np.ascontiguousarray(np.asarray([instance_ids[idx]], dtype=np.int64)).tobytes())
            object_hasher.update(str(categories[idx]).encode("utf-8"))
            object_hasher.update(
                np.ascontiguousarray(
                    np.asarray(
                        [
                            scores[idx],
                            semantic_confidences[idx],
                            association_confidences[idx],
                            semantic_gaps[idx],
                            view_qualities[idx],
                        ],
                        dtype=np.float32,
                    )
                ).tobytes()
            )
            if embeddings is not None:
                object_hasher.update(np.ascontiguousarray(embeddings[idx], dtype=np.float32).tobytes())
            signature = object_hasher.hexdigest()
            per_object_signatures.append(signature)
            full_hasher.update(signature.encode("ascii"))

        return {
            "is_empty": len(box_tensors) == 0,
            "box_tensors": box_tensors,
            "categories": categories,
            "instance_ids": instance_ids,
            "scores": scores,
            "semantic_confidences": semantic_confidences,
            "association_confidences": association_confidences,
            "semantic_gaps": semantic_gaps,
            "view_qualities": view_qualities,
            "embeddings": embeddings,
            "has_embeddings": bool(has_embeddings and embeddings is not None),
            "per_object_signatures": per_object_signatures,
            "full_signature": full_hasher.hexdigest(),
        }

    def _to_numpy_array(self, value: Any, dtype: Optional[np.dtype] = None) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            array = value.numpy()
        else:
            array = np.asarray(value)
        if dtype is not None:
            return np.asarray(array, dtype=dtype)
        return np.asarray(array)

    def update_latest_segmentation_export_metrics(self, frame_idx: int) -> None:
        if not self.last_export_profile:
            return
        for floor_state in self.floor_states.values():
            if not floor_state.segmentation_reports:
                continue
            latest = floor_state.segmentation_reports[-1]
            if int(latest.get("frame_idx", -1)) != int(frame_idx):
                continue
            latest["vector_map_export_after_segmentation_sec"] = float(self.last_export_profile.get("total_sec", 0.0))
            latest["vertical_transition_export_sec"] = float(self.last_export_profile.get("vertical_transition_export_sec", 0.0))
            latest["room_export_sec"] = float(self.last_export_profile.get("room_export_sec", 0.0))
            latest["object_export_sec"] = float(self.last_export_profile.get("object_export_sec", 0.0))
            latest["diagnostics_export_sec"] = float(self.last_export_profile.get("diagnostics_export_sec", 0.0))
            latest["current_vertical_transition_count"] = int(self.last_export_profile.get("vertical_transition_count", 0))
            break

    def export_segmentation_runs(self) -> List[Dict[str, Any]]:
        floors = canonicalize_floors(self.floor_manager.export_floors())
        floor_lookup = build_floor_lookup(floors)
        runs: List[Dict[str, Any]] = []
        for floor_id in self.floor_manager._ordered_floor_ids():
            floor_state = self.floor_states.get(str(floor_id))
            if floor_state is None:
                continue
            for report in floor_state.segmentation_reports:
                runs.append(attach_floor_metadata(dict(report), floor_lookup))
        return runs

    def describe_topology_status(self, pose_matrix: Optional[np.ndarray] = None) -> Dict[str, Any]:
        observation = dict(self.last_floor_observation or {})
        active_floor_id = observation.get("floor_id")
        active_room_id = None
        if pose_matrix is not None and active_floor_id is not None:
            floor_state = self.floor_states.get(str(active_floor_id))
            segmenter = None if floor_state is None else floor_state.segmenter
            if segmenter is not None and segmenter.last_room_markers is not None:
                pose = np.asarray(pose_matrix, dtype=np.float32)
                u_arr, v_arr, valid_mask = segmenter._world_to_grid(np.asarray([[pose[0, 3], pose[1, 3]]], dtype=np.float32))
                if bool(valid_mask[0]):
                    label = int(segmenter.last_room_markers[v_arr[0], u_arr[0]])
                    if label > 0 and label in segmenter.label_to_global:
                        local_room_id = int(segmenter.label_to_global[label])
                        active_room_id = floor_state.local_to_world_room_id.get(local_room_id)
        return {
            "active_floor_id": active_floor_id,
            "active_floor_status": observation.get("status"),
            "active_room_id": active_room_id,
            "active_room_available": active_room_id is not None,
            "room_leave_signal_exists": False,
            "topology_incremental_online": False,
            "topology_update_mode": "scheduled_segmentation_refresh_plus_full_export_rebuild",
            "topology_missing_online_trigger_structures": [
                "current_room_state",
                "room_exit_or_completion_signal",
                "incremental_room_buffer",
                "topology_delta_update_queue",
            ],
        }

    def _get_or_create_floor_state(self, floor_id: str) -> FloorState:
        floor_state = self.floor_states.get(floor_id)
        if floor_state is not None:
            return floor_state
        floor_state = FloorState(
            floor_id=floor_id,
            segmenter=DynamicRoomSegmenter(resolution=self.resolution, config=self.config),
        )
        self.floor_states[floor_id] = floor_state
        return floor_state

    def _merge_floor_points(self, floor_state: FloorState) -> Optional[np.ndarray]:
        chunks = []
        existing_count = 0 if floor_state.merged_points_xyzrgb is None else int(len(floor_state.merged_points_xyzrgb))
        if floor_state.merged_points_xyzrgb is not None:
            chunks.append(floor_state.merged_points_xyzrgb)
        chunks.extend(chunk for chunk in floor_state.pending_chunks if len(chunk) > 0)
        pending_point_count = int(sum(len(chunk) for chunk in floor_state.pending_chunks if len(chunk) > 0))
        floor_state.pending_chunks = []
        floor_state.pending_chunk_frame_indices = []
        if not chunks:
            self.last_merge_profile = {
                "existing_point_count": int(existing_count),
                "pending_point_count": int(pending_point_count),
                "merged_point_count_before_merge": int(existing_count + pending_point_count),
                "merged_point_count_after_downsample": 0 if floor_state.merged_points_xyzrgb is None else int(len(floor_state.merged_points_xyzrgb)),
                "floor_merge_sec": 0.0,
                "floor_downsample_sec": 0.0,
            }
            return floor_state.merged_points_xyzrgb
        merge_t0 = time.perf_counter()
        merged = np.concatenate(chunks, axis=0)
        floor_merge_sec = time.perf_counter() - merge_t0
        downsample_t0 = time.perf_counter()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(merged[:, :3], dtype=np.float64))
        if merged.shape[1] >= 6:
            pcd.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(merged[:, 3:6], dtype=np.float64))
        pcd = pcd.voxel_down_sample(voxel_size=self.resolution)
        floor_downsample_sec = time.perf_counter() - downsample_t0
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else np.zeros((len(points), 3), dtype=np.float64)
        floor_state.merged_points_xyzrgb = np.concatenate([points, colors], axis=1) if len(points) else None
        self.last_merge_profile = {
            "existing_point_count": int(existing_count),
            "pending_point_count": int(pending_point_count),
            "merged_point_count_before_merge": int(len(merged)),
            "merged_point_count_after_downsample": 0 if floor_state.merged_points_xyzrgb is None else int(len(floor_state.merged_points_xyzrgb)),
            "floor_merge_sec": float(floor_merge_sec),
            "floor_downsample_sec": float(floor_downsample_sec),
        }
        return floor_state.merged_points_xyzrgb

    def _map_tracking_report(self, floor_id: str, local_report: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        local_report = dict(local_report or {})
        floor_state = self._get_or_create_floor_state(floor_id)
        mapped = dict(local_report)
        for key in ("matched", "new_rooms", "retained_missing", "dropped_missing"):
            mapped_entries = []
            for item in local_report.get(key, []):
                entry = dict(item)
                local_room_id = int(entry.get("global_id", -1))
                if local_room_id >= 0:
                    world_room_id = floor_state.local_to_world_room_id.get(local_room_id)
                    if world_room_id is None:
                        world_room_id = self._next_world_room_id
                        self._next_world_room_id += 1
                        floor_state.local_to_world_room_id[local_room_id] = int(world_room_id)
                    entry["local_room_id"] = int(local_room_id)
                    entry["global_id"] = int(world_room_id)
                entry["floor_id"] = floor_id
                mapped_entries.append(entry)
            mapped[key] = mapped_entries
        return mapped

    def export_floor_diagnostics(
        self,
        vector_map: Optional[Dict[str, Any]] = None,
        total_frames: Optional[int] = None,
        last_segmentation_frame_idx: Optional[int] = None,
        room_seg_interval: Optional[int] = None,
    ) -> Dict[str, Any]:
        if vector_map is None:
            vector_map = self.get_vector_map_data(save_scene_graph_vis=False)
        floor_debug = dict(vector_map.get("floor_debug") or self._build_floor_debug_summary(vector_map))
        floor_debug["frame_floor_assignments"] = self.floor_manager.export_assignment_history()
        floor_debug["floor_hypothesis_events"] = self.floor_manager.export_debug_events()
        floor_debug["pending_candidate"] = self.floor_manager.export_pending_state()
        floor_debug["total_frames"] = None if total_frames is None else int(total_frames)
        floor_debug["last_segmentation_frame_idx"] = None if last_segmentation_frame_idx is None else int(last_segmentation_frame_idx)
        floor_debug["room_seg_interval"] = None if room_seg_interval is None else int(room_seg_interval)
        highest_floor = None
        if floor_debug.get("per_floor"):
            highest_floor = max(
                floor_debug["per_floor"],
                key=lambda item: int(item.get("first_seen_frame") or -1),
            )
            if last_segmentation_frame_idx is not None:
                highest_floor["appeared_after_last_scheduled_segmentation"] = (
                    highest_floor.get("first_seen_frame") is not None
                    and int(highest_floor["first_seen_frame"]) > int(last_segmentation_frame_idx)
                )
        floor_debug["highest_floor_check"] = highest_floor
        self.last_floor_diagnostics = floor_debug
        return floor_debug

    def save_floor_diagnostics(
        self,
        output_dir: str,
        vector_map: Optional[Dict[str, Any]] = None,
        total_frames: Optional[int] = None,
        last_segmentation_frame_idx: Optional[int] = None,
        room_seg_interval: Optional[int] = None,
        sequence_id: Optional[str] = None,
    ) -> Dict[str, str]:
        os.makedirs(output_dir, exist_ok=True)
        report = self.export_floor_diagnostics(
            vector_map=vector_map,
            total_frames=total_frames,
            last_segmentation_frame_idx=last_segmentation_frame_idx,
            room_seg_interval=room_seg_interval,
        )
        report["sequence_id"] = sequence_id
        json_path = os.path.join(output_dir, "floor_diagnostics_summary.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        md_path = os.path.join(output_dir, "floor_diagnostics_report.md")
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(self._build_floor_diagnostics_markdown(report))
        plot_path = os.path.join(output_dir, "floor_assignment_plot.png")
        self._save_floor_assignment_plot(plot_path, report)
        return {
            "json_path": json_path,
            "markdown_path": md_path,
            "plot_path": plot_path,
        }

    def _build_floor_debug_summary(self, vector_map: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        vector_map = dict(vector_map or {})
        floors = canonicalize_floors(list(vector_map.get("floors") or self.floor_manager.export_floors()))
        floor_lookup = build_floor_lookup(floors)
        rooms = list(vector_map.get("rooms") or [])
        objects = list(vector_map.get("objects") or [])
        anchors = list(vector_map.get("anchors") or [])
        history = [
            attach_floor_metadata(dict(item), floor_lookup)
            for item in list(vector_map.get("frame_floor_assignments") or self.floor_manager.export_assignment_history())
        ]
        raw_segmentation = dict(vector_map.get("room_segmentation_diagnostics") or {})
        segmentation_diagnostics = (
            build_fallback_summary(raw_segmentation.get("runs", []), floors)
            if raw_segmentation.get("runs") is not None
            else self._build_segmentation_diagnostics(floors)
        )
        vertical_transition_summary = build_vertical_transition_summary(
            vector_map.get("vertical_transitions", []),
            floor_lookup,
        )
        room_counts = {}
        object_counts = {}
        anchor_counts = {}
        for room in rooms:
            room_counts[room.get("floor_id")] = room_counts.get(room.get("floor_id"), 0) + 1
        for obj in objects:
            object_counts[obj.get("floor_id")] = object_counts.get(obj.get("floor_id"), 0) + 1
        for anchor in anchors:
            anchor_counts[anchor.get("floor_id")] = anchor_counts.get(anchor.get("floor_id"), 0) + 1

        floor_diag_lookup = {
            item.get("floor_id"): dict(item)
            for item in segmentation_diagnostics.get("per_floor", [])
            if item.get("floor_id") is not None
        }
        per_floor = []
        for floor in floors:
            floor_id = floor.get("floor_id")
            floor_state = self.floor_states.get(str(floor_id))
            stable_frames = sum(1 for item in history if item.get("floor_id") == floor_id and item.get("status") == "stable")
            transition_frames = sum(1 for item in history if item.get("floor_id") == floor_id and item.get("status") in {"transition", "uncertain"})
            support_stats = dict(floor.get("support_statistics") or {})
            floor_diag = dict(floor_diag_lookup.get(floor_id) or {})
            summary = {
                "floor_id": floor_id,
                "floor_index": floor.get("floor_index"),
                "display_floor_id": floor.get("display_floor_id", floor_id),
                "display_order": floor.get("display_order"),
                "z_min": floor.get("z_min"),
                "z_max": floor.get("z_max"),
                "z_center": floor.get("z_center"),
                "support_count": int(support_stats.get("frame_count", 0) or 0),
                "stable_frame_count": int(stable_frames),
                "transition_frame_count": int(transition_frames),
                "stable_keyframe_chunk_count": int(getattr(floor_state, "stable_keyframe_chunk_count", support_stats.get("keyframe_count", 0) or 0)),
                "segmentation_trigger_count": int(getattr(floor_state, "segmentation_trigger_count", 0)),
                "segmentation_run_count": int(floor_diag.get("run_count", 0)),
                "fallback_run_count": int(floor_diag.get("fallback_run_count", 0)),
                "fallback_counts": dict(floor_diag.get("fallback_counts", {})),
                "slice_mode_counts": dict(floor_diag.get("slice_mode_counts", {})),
                "exported_room_count": int(room_counts.get(floor_id, 0)),
                "exported_object_count": int(object_counts.get(floor_id, 0)),
                "exported_anchor_count": int(anchor_counts.get(floor_id, 0)),
                "first_seen_frame": support_stats.get("first_seen_frame"),
                "last_seen_frame": support_stats.get("last_seen_frame"),
                "last_segmentation_frame_idx": None if floor_state is None or floor_state.last_segmentation_frame_idx is None else int(floor_state.last_segmentation_frame_idx),
                "pending_chunk_count": 0 if floor_state is None else int(self._pending_chunk_count(floor_state)),
                "received_stable_chunks": bool(floor_state is not None and floor_state.stable_keyframe_chunk_count > 0),
                "has_segmented_rooms": bool(floor_state is not None and floor_state.segmenter.last_room_markers is not None),
            }
            if floor_state is not None:
                floor_state.exported_room_count = int(summary["exported_room_count"])
                floor_state.exported_object_count = int(summary["exported_object_count"])
                floor_state.exported_anchor_count = int(summary["exported_anchor_count"])
            per_floor.append(summary)

        return {
            "per_floor": per_floor,
            "finalization_report": self.last_finalization_report,
            "room_segmentation_diagnostics": segmentation_diagnostics,
            "vertical_transition_summary": vertical_transition_summary,
        }

    def _build_floor_diagnostics_markdown(self, report: Dict[str, Any]) -> str:
        lines = ["# Floor Diagnostics", ""]
        if report.get("sequence_id"):
            lines.append(f"- sequence_id: {report['sequence_id']}")
        lines.append(f"- total_frames: {report.get('total_frames')}")
        lines.append(f"- last_segmentation_frame_idx: {report.get('last_segmentation_frame_idx')}")
        lines.append(f"- room_seg_interval: {report.get('room_seg_interval')}")
        lines.append("")
        lines.append("## Per-floor summary")
        lines.append("")
        for floor in report.get("per_floor", []):
            lines.append(
                "- "
                f"{_display_floor_ref(floor.get('floor_id'), floor.get('display_floor_id'))} idx={floor.get('floor_index')} z=[{floor.get('z_min')}, {floor.get('z_max')}] center={floor.get('z_center')} "
                f"support={floor.get('support_count')} stable={floor.get('stable_frame_count')} transition={floor.get('transition_frame_count')} "
                f"stable_chunks={floor.get('stable_keyframe_chunk_count')} seg_triggers={floor.get('segmentation_trigger_count')} seg_runs={floor.get('segmentation_run_count', 0)} fallback_runs={floor.get('fallback_run_count', 0)} "
                f"exported rooms/objects/anchors={floor.get('exported_room_count')}/{floor.get('exported_object_count')}/{floor.get('exported_anchor_count')}"
            )
            if floor.get("fallback_counts"):
                lines.append(f"  fallback_counts={floor.get('fallback_counts')}")
        lines.append("")
        lines.append("## Vertical transitions")
        lines.append("")
        vertical_summary = dict(report.get("vertical_transition_summary") or {})
        lines.append(
            "- "
            f"count={vertical_summary.get('count', 0)} edge_eligible={vertical_summary.get('edge_eligible_count', 0)} "
            f"partial_room_association={vertical_summary.get('partial_room_association_count', 0)} "
            f"connector_labels={vertical_summary.get('connector_label_counts', {})}"
        )
        for item in vertical_summary.get("transitions", []):
            lines.append(
                "- "
                f"{item.get('transition_id')}: {item.get('from_display_floor_id')} -> {item.get('to_display_floor_id')} "
                f"rooms={item.get('from_room_label')} -> {item.get('to_room_label')} "
                f"frames={item.get('transition_frame_start')}-{item.get('transition_frame_end')} "
                f"label={item.get('connector_label')} confidence={item.get('confidence')} status={item.get('status')}"
            )
        lines.append("")
        lines.append("## Segmentation fallback diagnostics")
        lines.append("")
        segmentation_diagnostics = dict(report.get("room_segmentation_diagnostics") or {})
        lines.append(
            "- "
            f"run_count={segmentation_diagnostics.get('run_count', 0)} successful={segmentation_diagnostics.get('successful_run_count', 0)} "
            f"fallback_runs={segmentation_diagnostics.get('fallback_run_count', 0)} slice_modes={segmentation_diagnostics.get('slice_mode_counts', {})} "
            f"fallback_counts={segmentation_diagnostics.get('fallback_counts', {})}"
        )
        for item in segmentation_diagnostics.get("runs", []):
            lines.append(
                "- "
                f"{_display_floor_ref(item.get('floor_id'), item.get('display_floor_id'))} frame={item.get('frame_idx')} trigger={item.get('trigger_reason')} "
                f"slice={item.get('slice_mode')} fallbacks={item.get('fallback_modes') or ['none']} success={item.get('success')} "
                f"pending_frames={item.get('pending_chunk_frame_start')}->{item.get('pending_chunk_frame_end')} reason={item.get('fallback_reason_summary')}"
            )
        lines.append("")
        lines.append("## Floor hypothesis events")
        lines.append("")
        for event in report.get("floor_hypothesis_events", []):
            lines.append(
                "- "
                f"frame {event.get('frame_idx')}: {event.get('event')} floor={_display_floor_ref(event.get('floor_id'), event.get('display_floor_id'))} candidate_z={event.get('candidate_z')} "
                f"pending={event.get('pending_sample_count')} note={event.get('note')}"
            )
        lines.append("")
        lines.append("## End-of-sequence finalization")
        lines.append("")
        finalization = dict(report.get("finalization_report") or {})
        if not finalization:
            lines.append("- no explicit finalization report recorded")
        else:
            lines.append(f"- segmented_floor_count: {finalization.get('segmented_floor_count')}")
            for floor in finalization.get("floors", []):
                lines.append(
                    "- "
                    f"{_display_floor_ref(floor.get('floor_id'), floor.get('display_floor_id'))}: pending_before={floor.get('pending_chunk_count_before')} segmented={floor.get('segmented')} reason={floor.get('reason')}"
                )
        highest_floor = report.get("highest_floor_check")
        if highest_floor:
            lines.append("")
            lines.append("## Late-floor starvation check")
            lines.append("")
            lines.append(
                "- "
                f"{_display_floor_ref(highest_floor.get('floor_id'), highest_floor.get('display_floor_id'))}: first_seen={highest_floor.get('first_seen_frame')} last_seen={highest_floor.get('last_seen_frame')} "
                f"stable_chunks={highest_floor.get('stable_keyframe_chunk_count')} seg_triggers={highest_floor.get('segmentation_trigger_count')} "
                f"appeared_after_last_scheduled_segmentation={highest_floor.get('appeared_after_last_scheduled_segmentation')}"
            )
        return "\n".join(lines) + "\n"

    def _save_floor_assignment_plot(self, path: str, report: Dict[str, Any]) -> None:
        history = list(report.get("frame_floor_assignments") or [])
        if not history:
            return
        try:
            import matplotlib.pyplot as plt
        except Exception:
            return

        floor_ids = [item.get("floor_id") for item in history if item.get("floor_id") is not None]
        ordered_floor_ids = []
        for floor in report.get("per_floor", []):
            floor_id = floor.get("floor_id")
            if floor_id is not None and floor_id not in ordered_floor_ids:
                ordered_floor_ids.append(floor_id)
        for floor_id in floor_ids:
            if floor_id not in ordered_floor_ids:
                ordered_floor_ids.append(floor_id)
        palette = {}
        cmap = plt.get_cmap("tab10")
        for idx, floor_id in enumerate(ordered_floor_ids):
            palette[floor_id] = cmap(idx % 10)

        fig, ax = plt.subplots(figsize=(12, 4.8))
        for status, marker in (("stable", "o"), ("transition", "x"), ("uncertain", "^")):
            xs = []
            ys = []
            colors = []
            for item in history:
                if item.get("status") != status:
                    continue
                xs.append(int(item.get("frame_idx", 0)))
                ys.append(float(item.get("pose_z", 0.0)))
                colors.append(palette.get(item.get("floor_id"), (0.6, 0.6, 0.6, 1.0)))
            if xs:
                ax.scatter(xs, ys, c=colors, s=14 if status == "stable" else 24, marker=marker, alpha=0.8, label=status)

        for event in report.get("floor_hypothesis_events", []):
            if event.get("event") in {"bootstrap_floor_created", "new_floor_confirmed"}:
                ax.axvline(int(event.get("frame_idx", 0)), color=(0.4, 0.4, 0.4), linewidth=1.0, linestyle="--", alpha=0.5)

        ax.set_title("Frame Index vs Pose Z with Floor Assignment")
        ax.set_xlabel("frame_idx")
        ax.set_ylabel("pose_z (m)")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)

    def _pending_chunk_count(self, floor_state: FloorState) -> int:
        return int(sum(1 for chunk in floor_state.pending_chunks if len(chunk) > 0))

    def _export_rooms_and_gateways(
        self,
        vector_data: Dict[str, Any],
        floor_lookup: Dict[str, Dict[str, Any]],
    ) -> Tuple[List[Dict[str, Any]], Dict[int, Dict[str, Any]]]:
        room_records: List[Dict[str, Any]] = []
        room_lookup: Dict[int, Dict[str, Any]] = {}
        for floor_id, floor_state in sorted(self.floor_states.items(), key=lambda item: item[0]):
            segmenter = floor_state.segmenter
            if segmenter.last_room_markers is None:
                continue
            unique_labels = np.unique(segmenter.last_room_markers)
            wall_label = np.max(unique_labels)
            floor_meta = dict(floor_lookup.get(floor_id) or {})
            floor_conf = float(floor_meta.get("confidence", 0.0) or 0.0)
            for label in unique_labels:
                if label <= 0 or label == wall_label or label not in segmenter.label_to_global:
                    continue
                local_room_id = int(segmenter.label_to_global[label])
                world_room_id = floor_state.local_to_world_room_id.get(local_room_id)
                if world_room_id is None:
                    world_room_id = self._next_world_room_id
                    self._next_world_room_id += 1
                    floor_state.local_to_world_room_id[local_room_id] = int(world_room_id)
                mask = (segmenter.last_room_markers == label).astype(np.uint8) * 255
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    epsilon = 0.02 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)
                    polygon = []
                    for pt in approx:
                        world_xy = segmenter._grid_to_world(pt[0][0], pt[0][1])
                        polygon.append([round(float(world_xy[0]), 3), round(float(world_xy[1]), 3)])
                    center = _polygon_centroid(polygon)
                    room_record = {
                        "id": int(world_room_id),
                        "room_uuid": int(world_room_id),
                        "room_local_id": int(local_room_id),
                        "room_id": f"room_{int(world_room_id)}",
                        "floor_id": floor_id,
                        "floor_index": floor_meta.get("floor_index"),
                        "display_floor_id": floor_meta.get("display_floor_id", floor_id),
                        "display_order": floor_meta.get("display_order"),
                        "floor_assignment_confidence": round(float(floor_conf), 3),
                        "status": "confirmed",
                        "room_type": "unknown",
                        "polygon": polygon,
                        "center": [round(float(center[0]), 3), round(float(center[1]), 3)],
                        "area_m2": round(float(_polygon_area(polygon)), 3),
                    }
                    room_records.append(room_record)
                    room_lookup[int(world_room_id)] = room_record

            for gate in segmenter.last_gateways:
                connects = list(gate.get("connects", []))
                if len(connects) != 2:
                    continue
                mapped_rooms = []
                for local_room_id in connects:
                    world_room_id = floor_state.local_to_world_room_id.get(int(local_room_id))
                    if world_room_id is None:
                        break
                    mapped_rooms.append(int(world_room_id))
                if len(mapped_rooms) != 2:
                    continue
                gateway = dict(gate)
                gateway["connects"] = mapped_rooms
                gateway["floor_id"] = floor_id
                gateway["display_floor_id"] = floor_meta.get("display_floor_id", floor_id)
                gateway["display_order"] = floor_meta.get("display_order")
                gateway["relation_scope"] = "same_floor"
                vector_data["gateways"].append(gateway)

        room_records.sort(key=lambda item: int(item["id"]))
        vector_data["rooms"] = room_records
        return room_records, room_lookup

    def _record_segmentation_run(
        self,
        floor_id: str,
        floor_state: FloorState,
        trigger_reason: str,
        active_status: Optional[str],
        count: int,
        pending_chunk_count: int,
        pending_chunk_frame_indices: Sequence[int],
        merged_point_count: int,
        success: bool,
        merge_total_sec: float = 0.0,
    ) -> Dict[str, Any]:
        segmenter = floor_state.segmenter
        height_slice = dict(segmenter.last_height_slice_debug or {})
        failure_debug = dict(segmenter.last_failure_debug or {})
        tracking_report = dict(segmenter.last_tracking_report or {})
        segmentation_profile = dict(segmenter.last_segmentation_profile or {})
        merge_profile = dict(self.last_merge_profile or {})

        slice_mode = str(height_slice.get("mode", "unknown"))
        slice_mode_label = "default_slice" if slice_mode == "default" else slice_mode
        adaptive_applied = bool(height_slice.get("adaptive_applied", False))
        fallback_modes: List[str] = []
        if slice_mode in {"adaptive_low_span", "adaptive_mid_band"}:
            fallback_modes.append(slice_mode)
        if trigger_reason == "end_of_sequence_flush":
            fallback_modes.append("end_of_sequence_forced_segmentation")

        failure_reason = failure_debug.get("reason")
        if success:
            result_status = "rooms_returned"
        elif failure_reason:
            result_status = f"failed:{failure_reason}"
        else:
            result_status = "segmentation_returned_none"

        reason_parts = [f"slice={slice_mode_label}"]
        if adaptive_applied:
            reason_parts.append("adaptive_height_slice_applied")
        if trigger_reason == "end_of_sequence_flush":
            reason_parts.append("forced_end_of_sequence_flush")
        if failure_reason:
            reason_parts.append(f"failure={failure_reason}")
        fallback_reason_summary = "; ".join(reason_parts)

        run_record = {
            "run_id": f"{floor_id}_seg_{len(floor_state.segmentation_reports) + 1}",
            "floor_id": floor_id,
            "display_floor_id": None,
            "display_order": None,
            "frame_idx": int(count),
            "active_status": active_status,
            "trigger_reason": str(trigger_reason),
            "success": bool(success),
            "result_status": result_status,
            "pending_chunk_count": int(pending_chunk_count),
            "pending_chunk_frame_start": None if not pending_chunk_frame_indices else int(min(pending_chunk_frame_indices)),
            "pending_chunk_frame_end": None if not pending_chunk_frame_indices else int(max(pending_chunk_frame_indices)),
            "pending_chunk_frame_count": int(len(pending_chunk_frame_indices)),
            "pending_chunk_frame_indices": [int(item) for item in pending_chunk_frame_indices],
            "merged_point_count": int(merged_point_count),
            "merged_point_count_before_merge": int(merge_profile.get("merged_point_count_before_merge", merged_point_count)),
            "merged_point_count_after_downsample": int(merge_profile.get("merged_point_count_after_downsample", merged_point_count)),
            "wall_slice_point_count": int(segmentation_profile.get("wall_slice_point_count", 0)),
            "full_slice_point_count": int(segmentation_profile.get("full_slice_point_count", 0)),
            "grid_width": int(segmentation_profile.get("grid_width", 0)),
            "grid_height": int(segmentation_profile.get("grid_height", 0)),
            "grid_area": int(segmentation_profile.get("grid_area", 0)),
            "room_count_before_tracking": int(segmentation_profile.get("room_count_before_tracking", 0)),
            "room_count_after_tracking": int(segmentation_profile.get("room_count_after_tracking", 0)),
            "tracked_room_count": int(segmentation_profile.get("tracked_room_count", 0)),
            "gateway_room_pair_checks": int(segmentation_profile.get("gateway_room_pair_checks", 0)),
            "gateway_count": int(segmentation_profile.get("gateway_count", 0)),
            "current_vertical_transition_count": None,
            "floor_merge_sec": float(merge_profile.get("floor_merge_sec", merge_total_sec)),
            "floor_downsample_sec": float(merge_profile.get("floor_downsample_sec", 0.0)),
            "segmentation_total_sec": float(segmentation_profile.get("histogram_build_sec", 0.0))
            + float(segmentation_profile.get("segmentation_state_build_sec", 0.0))
            + float(segmentation_profile.get("room_tracking_sec", 0.0))
            + float(segmentation_profile.get("gateway_extraction_sec", 0.0)),
            "histogram_build_sec": float(segmentation_profile.get("histogram_build_sec", 0.0)),
            "segmentation_state_build_sec": float(segmentation_profile.get("segmentation_state_build_sec", 0.0)),
            "room_tracking_sec": float(segmentation_profile.get("room_tracking_sec", 0.0)),
            "gateway_extraction_sec": float(segmentation_profile.get("gateway_extraction_sec", 0.0)),
            "vector_map_export_after_segmentation_sec": 0.0,
            "vertical_transition_export_sec": 0.0,
            "room_export_sec": 0.0,
            "object_export_sec": 0.0,
            "diagnostics_export_sec": 0.0,
            "slice_mode": slice_mode_label,
            "adaptive_applied": bool(adaptive_applied),
            "fallback_used": bool(fallback_modes),
            "fallback_modes": list(fallback_modes),
            "primary_fallback_mode": None if not fallback_modes else str(fallback_modes[0]),
            "fallback_reason_summary": fallback_reason_summary,
            "failure_reason": failure_reason,
            "height_slice": {
                key: height_slice.get(key)
                for key in (
                    "floor_z",
                    "ceiling_z",
                    "span_z",
                    "slice_z_min",
                    "slice_z_max",
                    "full_z_max",
                    "mode",
                    "adaptive_applied",
                )
                if key in height_slice
            },
            "failure_debug": failure_debug,
            "tracking_summary": {
                "matched_count": int(len(tracking_report.get("matched", []))),
                "new_room_count": int(len(tracking_report.get("new_rooms", []))),
                "retained_missing_count": int(len(tracking_report.get("retained_missing", []))),
                "dropped_missing_count": int(len(tracking_report.get("dropped_missing", []))),
            },
            "room_count_after_segmentation": int(self._segmenter_room_count(segmenter)),
        }
        floor_state.segmentation_reports.append(run_record)
        self._log_verbose(
            "[FloorAwareRoomSegmenter] segmentation diagnostics -> "
            f"floor={floor_id}, trigger={trigger_reason}, slice={slice_mode_label}, "
            f"fallbacks={','.join(fallback_modes) if fallback_modes else 'none'}, "
            f"success={success}, frame={int(count)}"
        )
        return run_record

    def _segmenter_room_count(self, segmenter: DynamicRoomSegmenter) -> int:
        markers = getattr(segmenter, "last_room_markers", None)
        if markers is None:
            return 0
        unique_labels = np.unique(markers)
        if len(unique_labels) == 0:
            return 0
        wall_label = np.max(unique_labels)
        return int(sum(1 for label in unique_labels if label > 0 and label != wall_label))

    def _build_segmentation_diagnostics(
        self,
        floors: Sequence[Dict[str, Any]],
    ) -> Dict[str, Any]:
        floor_lookup = build_floor_lookup(floors)
        runs: List[Dict[str, Any]] = []
        for floor_id in self.floor_manager._ordered_floor_ids():
            floor_state = self.floor_states.get(str(floor_id))
            reports = list(getattr(floor_state, "segmentation_reports", []))
            for report in reports:
                runs.append(attach_floor_metadata(dict(report), floor_lookup))
        return build_fallback_summary(runs, floors)

    def _build_vertical_transition_summary(
        self,
        transitions: Sequence[Dict[str, Any]],
    ) -> Dict[str, Any]:
        return build_vertical_transition_summary(
            transitions,
            build_floor_lookup(self.floor_manager.export_floors()),
        )

    def _stable_floor_segments(self) -> List[Dict[str, Any]]:
        stable_segments: List[Dict[str, Any]] = []
        for item in self.frame_history:
            if item.get("status") != "stable" or item.get("floor_id") is None:
                continue
            if not stable_segments or stable_segments[-1]["floor_id"] != item.get("floor_id"):
                stable_segments.append(
                    {
                        "floor_id": item.get("floor_id"),
                        "display_floor_id": item.get("display_floor_id", item.get("floor_id")),
                        "frame_start": int(item.get("frame_idx", 0)),
                        "frame_end": int(item.get("frame_idx", 0)),
                        "items": [item],
                    }
                )
                continue
            stable_segments[-1]["frame_end"] = int(item.get("frame_idx", 0))
            stable_segments[-1]["items"].append(item)
        return stable_segments

    def _room_support_summary(
        self,
        items: Sequence[Dict[str, Any]],
        room_records: Sequence[Dict[str, Any]],
        floor_id: str,
    ) -> Dict[str, Any]:
        floor_rooms = [room for room in room_records if room.get("floor_id") == floor_id]
        if not items or not floor_rooms:
            return {
                "room_id": None,
                "room_id_label": None,
                "display_floor_id": None,
                "status": "unknown",
                "support_count": 0,
                "candidate_frame_count": int(len(items)),
                "position_xy": None,
                "sample_frames": [],
                "match_type_counts": {},
                "mean_distance_m": None,
            }

        room_votes: Dict[int, int] = {}
        room_direct_votes: Dict[int, int] = {}
        room_distances: Dict[int, List[float]] = {}
        room_frames: Dict[int, List[int]] = {}
        room_positions: Dict[int, List[List[float]]] = {}
        match_type_counts: Counter[str] = Counter()

        for item in items:
            position_xy = item.get("position_xy")
            if position_xy is None or len(position_xy) < 2:
                continue
            candidate = _room_candidate_for_pose(position_xy, floor_rooms, floor_id=floor_id, max_distance=3.2)
            if candidate is None:
                continue
            room = candidate["room"]
            room_id = int(room["id"])
            room_votes[room_id] = room_votes.get(room_id, 0) + 1
            room_frames.setdefault(room_id, []).append(int(item.get("frame_idx", 0)))
            room_positions.setdefault(room_id, []).append([round(float(position_xy[0]), 3), round(float(position_xy[1]), 3)])
            room_distances.setdefault(room_id, []).append(float(candidate["distance_m"]))
            match_type = str(candidate["match_type"])
            match_type_counts[match_type] += 1
            if match_type == "polygon_contains":
                room_direct_votes[room_id] = room_direct_votes.get(room_id, 0) + 1

        if not room_votes:
            return {
                "room_id": None,
                "room_id_label": None,
                "display_floor_id": None,
                "status": "unknown",
                "support_count": 0,
                "candidate_frame_count": int(len(items)),
                "position_xy": None,
                "sample_frames": [],
                "match_type_counts": {},
                "mean_distance_m": None,
            }

        best_room_id = sorted(
            room_votes,
            key=lambda room_id: (
                -int(room_votes[room_id]),
                -int(room_direct_votes.get(room_id, 0)),
                float(np.mean(room_distances.get(room_id, [999.0]))),
                room_id,
            ),
        )[0]
        best_room = next(room for room in floor_rooms if int(room["id"]) == int(best_room_id))
        support_count = int(room_votes[best_room_id])
        direct_support_count = int(room_direct_votes.get(best_room_id, 0))
        candidate_frame_count = int(len(items))
        if direct_support_count > 0 or support_count >= 2:
            status = "supported"
        elif support_count > 0:
            status = "weak"
        else:
            status = "unknown"
        sample_frames = room_frames.get(best_room_id, [])
        sample_positions = room_positions.get(best_room_id, [])
        representative_position = sample_positions[-1] if sample_positions else None
        return {
            "room_id": int(best_room_id),
            "room_id_label": f"R{int(best_room_id)}",
            "display_floor_id": best_room.get("display_floor_id", floor_id),
            "status": status,
            "support_count": support_count,
            "direct_support_count": direct_support_count,
            "candidate_frame_count": candidate_frame_count,
            "support_ratio": round(float(support_count / max(1, candidate_frame_count)), 3),
            "position_xy": representative_position,
            "sample_frames": [int(item) for item in sample_frames[:6]],
            "match_type_counts": dict(match_type_counts),
            "mean_distance_m": round(float(np.mean(room_distances.get(best_room_id, [0.0]))), 3),
        }

    def _classify_vertical_transition_interval(
        self,
        transition_items: Sequence[Dict[str, Any]],
        z_span: Optional[float],
    ) -> str:
        if not transition_items or z_span is None:
            return "unknown_vertical_connector"
        z_values = np.asarray(
            [float(item.get("pose_z")) for item in transition_items if item.get("pose_z") is not None],
            dtype=np.float32,
        )
        if len(z_values) < 2:
            return "unknown_vertical_connector"

        rounded_bins = np.round(z_values / 0.05) * 0.05
        dominant_bin_count = int(max(Counter(float(item) for item in rounded_bins).values(), default=0))
        diffs = np.diff(z_values)
        significant_diffs = diffs[np.abs(diffs) > 0.015]
        if len(significant_diffs) == 0:
            return "landing_like"

        overall_sign = np.sign(float(z_values[-1] - z_values[0]))
        if overall_sign == 0.0:
            overall_sign = float(np.sign(np.sum(significant_diffs)))
        if overall_sign == 0.0:
            overall_sign = 1.0
        monotonic_ratio = float(np.mean(np.sign(significant_diffs) == np.sign(overall_sign)))
        diff_std = float(np.std(significant_diffs)) if len(significant_diffs) > 1 else 0.0

        if dominant_bin_count >= max(5, int(round(0.35 * len(z_values)))) and float(z_span) >= 0.5:
            return "landing_like"
        if monotonic_ratio >= 0.85 and float(z_span) >= 1.0 and diff_std <= 0.03:
            return "ramp_like"
        if monotonic_ratio >= 0.7 and float(z_span) >= 1.0:
            return "stair_like"
        return "unknown_vertical_connector"

    def _vertical_transition_confidence(
        self,
        source_support: Dict[str, Any],
        target_support: Dict[str, Any],
        transition_items: Sequence[Dict[str, Any]],
        z_span: Optional[float],
        connector_label: str,
        room_association_complete: bool,
    ) -> float:
        confidence = 0.28
        for support in (source_support, target_support):
            status = str(support.get("status", "unknown"))
            if status == "supported":
                confidence += 0.18
            elif status == "weak":
                confidence += 0.08
        confidence += min(0.18, 0.015 * float(len(transition_items)))
        if z_span is not None:
            confidence += min(0.16, 0.07 * max(0.0, float(z_span) - 0.4))
        if connector_label in {"stair_like", "landing_like", "ramp_like"}:
            confidence += 0.08
        if not room_association_complete:
            confidence = min(confidence, 0.58)
        return max(0.25, min(0.95, confidence))

    def _attach_display_floor_metadata(
        self,
        records: Sequence[Dict[str, Any]],
        floor_lookup: Dict[str, Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        enriched: List[Dict[str, Any]] = []
        for item in records:
            record = dict(item)
            floor_id = record.get("floor_id")
            floor_meta = dict(floor_lookup.get(str(floor_id)) or {}) if floor_id is not None else {}
            if floor_meta:
                record["display_floor_id"] = floor_meta.get("display_floor_id", floor_id)
                record["display_order"] = floor_meta.get("display_order")
            else:
                record.setdefault("display_floor_id", floor_id)
                record.setdefault("display_order", None)
            enriched.append(record)
        return enriched

    def _build_object_export_record(
        self,
        object_index: int,
        *,
        prepared_object_state: Dict[str, Any],
        floor_lookup: Dict[str, Dict[str, Any]],
        room_lookup: Dict[int, Dict[str, Any]],
        object_floor_vote_cache: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> Tuple[Dict[str, Any], Optional[List[float]]]:
        box_tensors = prepared_object_state["box_tensors"]
        categories = prepared_object_state["categories"]
        instance_ids = prepared_object_state["instance_ids"]
        scores = prepared_object_state["scores"]
        semantic_confidences = prepared_object_state["semantic_confidences"]
        association_confidences = prepared_object_state["association_confidences"]
        semantic_gaps = prepared_object_state["semantic_gaps"]
        view_qualities = prepared_object_state["view_qualities"]
        has_embeddings = bool(prepared_object_state.get("has_embeddings", False))
        embeddings = prepared_object_state.get("embeddings")
        i = int(object_index)
        instance_id = int(instance_ids[i])
        cx, cy, cz = float(box_tensors[i, 0]), float(box_tensors[i, 1]), float(box_tensors[i, 2])
        dx, dy, dz = float(box_tensors[i, 3]), float(box_tensors[i, 4]), float(box_tensors[i, 5])
        yaw = float(box_tensors[i, 6]) if box_tensors.shape[1] > 6 else 0.0

        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[cos_y, -sin_y], [sin_y, cos_y]], dtype=np.float32)
        corners_local = np.array(
            [[dx / 2, dy / 2], [-dx / 2, dy / 2], [-dx / 2, -dy / 2], [dx / 2, -dy / 2]],
            dtype=np.float32,
        )
        corners_global = (rotation @ corners_local.T).T + np.array([cx, cy], dtype=np.float32)
        footprint_2d = np.round(corners_global, 3).tolist()

        floor_assignment = self.floor_manager.assign_height(cz)
        floor_id = floor_assignment.get("floor_id")
        room_uuid = -1
        room_assignment = {
            "method": "floor_then_footprint_vote_9pt",
            "floor_status": floor_assignment.get("status"),
            "floor_confidence": round(float(floor_assignment.get("confidence", 0.0) or 0.0), 3),
            "center_label": -1,
            "votes": {},
            "local_room_id": -1,
        }
        floor_vote_state = None if floor_id is None else dict((object_floor_vote_cache or {}).get(str(floor_id)) or {})
        floor_state = floor_vote_state.get("floor_state")
        if floor_state is None and floor_id in self.floor_states:
            floor_state = self.floor_states[floor_id]
        if floor_state is not None:
            if floor_state.segmenter.last_room_markers is not None:
                wall_label = int(
                    floor_vote_state.get("wall_label")
                    if floor_vote_state.get("wall_label") is not None
                    else np.max(floor_state.segmenter.last_room_markers)
                )
                local_room_id, center_label, room_votes = floor_state.segmenter._vote_room_label_for_object(
                    (cx, cy),
                    footprint_2d,
                    wall_label,
                )
                room_assignment["center_label"] = int(center_label) if center_label is not None else -1
                room_assignment["votes"] = {str(int(label)): int(value) for label, value in sorted(room_votes.items())}
                room_assignment["local_room_id"] = int(local_room_id)
                if int(local_room_id) >= 0:
                    room_uuid = int(floor_state.local_to_world_room_id.get(int(local_room_id), -1))

        obj_data = {
            "id": instance_id,
            "label": str(categories[i]),
            "category": str(categories[i]),
            "score": round(float(scores[i]), 3),
            "detection_confidence": round(float(scores[i]), 3),
            "semantic_confidence": round(float(semantic_confidences[i]), 3),
            "association_confidence": round(float(association_confidences[i]), 3),
            "semantic_gap": round(float(semantic_gaps[i]), 3),
            "view_quality": round(float(view_qualities[i]), 3),
            "floor_id": floor_id,
            "floor_index": floor_lookup.get(str(floor_id), {}).get("floor_index") if floor_id is not None else None,
            "display_floor_id": floor_lookup.get(str(floor_id), {}).get("display_floor_id") if floor_id is not None else None,
            "display_order": floor_lookup.get(str(floor_id), {}).get("display_order") if floor_id is not None else None,
            "floor_assignment": dict(floor_assignment),
            "room_uuid": int(room_uuid),
            "room_id": None if room_uuid < 0 else f"room_{int(room_uuid)}",
            "pose": [round(cx, 3), round(cy, 3)],
            "pose_3d": [round(cx, 3), round(cy, 3), round(cz, 3)],
            "size": [round(dx, 3), round(dy, 3), round(dz, 3)],
            "yaw": round(yaw, 3),
            "footprint_2d": footprint_2d,
            "room_assignment": room_assignment,
            "semantic_observations": [
                {
                    "label": str(categories[i]),
                    "category": str(categories[i]),
                    "detection_confidence": float(scores[i]),
                    "semantic_confidence": float(semantic_confidences[i]),
                    "association_confidence": float(association_confidences[i]),
                    "semantic_gap": float(semantic_gaps[i]),
                    "view_quality": float(view_qualities[i]),
                }
            ],
        }
        embedding_values = None
        if has_embeddings:
            embedding_ref = f"embedding_{instance_id}"
            obj_data["embedding_ref"] = embedding_ref
            embedding_values = embeddings[i].tolist()
        if room_uuid >= 0 and room_uuid in room_lookup:
            obj_data["floor_id"] = room_lookup[room_uuid].get("floor_id")
            obj_data["floor_index"] = room_lookup[room_uuid].get("floor_index")
            obj_data["display_floor_id"] = room_lookup[room_uuid].get("display_floor_id")
            obj_data["display_order"] = room_lookup[room_uuid].get("display_order")
        return obj_data, embedding_values

    def _try_reuse_cached_object_export_record(
        self,
        object_index: int,
        *,
        prepared_object_state: Dict[str, Any],
        floor_lookup: Dict[str, Dict[str, Any]],
        room_lookup: Dict[int, Dict[str, Any]],
        cached_entry: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        cached_object = dict(cached_entry.get("object") or {})
        if not cached_object:
            return None

        box_tensors = prepared_object_state["box_tensors"]
        i = int(object_index)
        current_floor_assignment = dict(self.floor_manager.assign_height(float(box_tensors[i, 2])) or {})
        current_floor_id = current_floor_assignment.get("floor_id")

        room_uuid = int(cached_object.get("room_uuid", -1))
        room_record = None if room_uuid < 0 else room_lookup.get(room_uuid)
        if room_record is not None and room_record.get("floor_id") != current_floor_id:
            return None

        cached_object["floor_assignment"] = current_floor_assignment
        cached_object["floor_id"] = current_floor_id
        cached_object["floor_index"] = current_floor_assignment.get("floor_index")
        cached_object["display_floor_id"] = current_floor_assignment.get("display_floor_id")
        cached_object["display_order"] = current_floor_assignment.get("display_order")

        room_assignment = dict(cached_object.get("room_assignment") or {})
        room_assignment["floor_status"] = current_floor_assignment.get("status")
        room_assignment["floor_confidence"] = round(float(current_floor_assignment.get("confidence", 0.0) or 0.0), 3)
        cached_object["room_assignment"] = room_assignment

        if room_record is not None:
            cached_object["floor_id"] = room_record.get("floor_id")
            cached_object["floor_index"] = room_record.get("floor_index")
            cached_object["display_floor_id"] = room_record.get("display_floor_id")
            cached_object["display_order"] = room_record.get("display_order")
        elif current_floor_id is not None:
            floor_meta = dict(floor_lookup.get(str(current_floor_id)) or {})
            if floor_meta:
                cached_object["floor_index"] = floor_meta.get("floor_index")
                cached_object["display_floor_id"] = floor_meta.get("display_floor_id")
                cached_object["display_order"] = floor_meta.get("display_order")
        return cached_object

    def _object_bucket_id_from_room(self, room_id: Optional[str]) -> str:
        if room_id in (None, ""):
            return "__unassigned__"
        return str(room_id)

    def _build_object_exports(
        self,
        prepared_object_state: Dict[str, Any],
        floor_lookup: Dict[str, Dict[str, Any]],
        room_lookup: Dict[int, Dict[str, Any]],
        reuse_cache: Optional[Dict[int, Dict[str, Any]]] = None,
    ) -> Tuple[List[Dict[str, Any]], Dict[str, List[float]], Dict[int, Dict[str, Any]], int]:
        if prepared_object_state.get("is_empty", False):
            return [], {}, {}, 0

        instance_ids = prepared_object_state["instance_ids"]
        per_object_signatures = prepared_object_state.get("per_object_signatures", [])
        has_embeddings = bool(prepared_object_state.get("has_embeddings", False))
        object_floor_vote_cache = self._build_object_export_floor_vote_cache()

        objects: List[Dict[str, Any]] = []
        embedding_lookup: Dict[str, List[float]] = {}
        cache_entries: Dict[int, Dict[str, Any]] = {}
        cache_hit_count = 0

        for i in range(len(instance_ids)):
            instance_id = int(instance_ids[i])
            object_signature = None if i >= len(per_object_signatures) else str(per_object_signatures[i])
            cached_entry = None if reuse_cache is None else reuse_cache.get(instance_id)
            if cached_entry is not None and cached_entry.get("signature") == object_signature:
                reused_object = self._try_reuse_cached_object_export_record(
                    i,
                    prepared_object_state=prepared_object_state,
                    floor_lookup=floor_lookup,
                    room_lookup=room_lookup,
                    cached_entry=cached_entry,
                )
                if reused_object is not None:
                    objects.append(reused_object)
                    cache_entries[instance_id] = {
                        "signature": object_signature,
                        "object": reused_object,
                        "embedding": cached_entry.get("embedding"),
                    }
                    if has_embeddings and reused_object.get("embedding_ref") is not None:
                        embedding_lookup[str(reused_object["embedding_ref"])] = list(cached_entry.get("embedding") or [])
                    cache_hit_count += 1
                    continue

            obj_data, embedding_values = self._build_object_export_record(
                i,
                prepared_object_state=prepared_object_state,
                floor_lookup=floor_lookup,
                room_lookup=room_lookup,
                object_floor_vote_cache=object_floor_vote_cache,
            )
            if has_embeddings and obj_data.get("embedding_ref") is not None and embedding_values is not None:
                embedding_lookup[str(obj_data["embedding_ref"])] = embedding_values
            objects.append(obj_data)
            cache_entries[instance_id] = {
                "signature": object_signature,
                "object": obj_data,
                "embedding": embedding_values,
            }

        return objects, embedding_lookup, cache_entries, int(cache_hit_count)

    def _build_room_local_delta_object_exports(
        self,
        prepared_object_state: Dict[str, Any],
        floor_lookup: Dict[str, Dict[str, Any]],
        room_lookup: Dict[int, Dict[str, Any]],
        *,
        room_records: Sequence[Dict[str, Any]],
        room_cache: Dict[str, Any],
        cached_export: Optional[Dict[str, Any]],
    ) -> Optional[Tuple[List[Dict[str, Any]], Dict[str, List[float]], Dict[int, Dict[str, Any]], int, Dict[str, Any]]]:
        if prepared_object_state.get("is_empty", False):
            return (
                [],
                {},
                {},
                0,
                {
                    "room_local_delta_export_used": True,
                    "room_local_delta_changed_room_count": 0,
                    "room_local_delta_reused_room_count": int(len(room_records)),
                    "room_local_delta_rebuilt_room_count": 0,
                    "room_local_delta_unassigned_bucket_rebuilt": False,
                    "changed_room_count": 0,
                    "changed_room_ids": "",
                    "changed_room_structure_changed_count": 0,
                    "changed_room_structure_changed_ids": "",
                    "changed_room_object_delta_count": 0,
                    "changed_room_object_delta_ids": "",
                    "changed_room_removed_count": 0,
                    "changed_room_removed_ids": "",
                    "changed_room_local_rebuild_used": True,
                    "changed_room_locality_confident": True,
                    "full_fallback_rebuild_used": False,
                    "rebuilt_object_in_changed_rooms_count": 0,
                },
            )
        if cached_export is None:
            return None

        cached_object_cache = dict(cached_export.get("object_cache") or {})
        if not cached_object_cache:
            return None
        cached_room_cache = dict(cached_export.get("room_cache") or {})
        cached_room_signatures = dict(cached_room_cache.get("room_signatures") or {})
        if not cached_room_signatures:
            return None
        current_room_signatures = dict((room_cache or {}).get("room_signatures") or {})
        current_room_ids = set(current_room_signatures.keys())
        cached_room_ids = set(cached_room_signatures.keys())

        instance_ids = np.asarray(prepared_object_state.get("instance_ids"), dtype=np.int64)
        per_object_signatures = list(prepared_object_state.get("per_object_signatures") or [])
        has_embeddings = bool(prepared_object_state.get("has_embeddings", False))
        object_floor_vote_cache = self._build_object_export_floor_vote_cache()

        current_ids_in_order = [int(item) for item in instance_ids.tolist()]
        current_signature_by_id = {
            int(instance_ids[idx]): str(per_object_signatures[idx])
            for idx in range(min(len(instance_ids), len(per_object_signatures)))
        }
        current_index_by_id = {int(instance_ids[idx]): int(idx) for idx in range(len(instance_ids))}
        current_id_set = set(current_ids_in_order)
        cached_id_set = {int(item) for item in cached_object_cache.keys()}
        unchanged_room_ids = {
            room_id
            for room_id in current_room_ids
            if cached_room_signatures.get(room_id) == current_room_signatures.get(room_id)
        }
        structure_changed_room_ids = current_room_ids - unchanged_room_ids
        removed_room_ids = cached_room_ids - current_room_ids

        prepared_entries: Dict[int, Dict[str, Any]] = {}
        fast_reuse_ids: set[int] = set()
        object_delta_room_ids: set[str] = set()
        impacted_room_ids: set[str] = set(structure_changed_room_ids | removed_room_ids)
        unassigned_bucket_rebuilt = False

        def _mark_bucket(bucket_id: str) -> None:
            nonlocal unassigned_bucket_rebuilt
            if bucket_id == "__unassigned__":
                unassigned_bucket_rebuilt = True
                return
            impacted_room_ids.add(bucket_id)
            object_delta_room_ids.add(bucket_id)

        for object_id in current_ids_in_order:
            object_signature = current_signature_by_id.get(object_id)
            cached_entry = cached_object_cache.get(object_id)
            prior_room_id = ((cached_entry.get("object") or {}).get("room_id")) if cached_entry is not None else None
            prior_bucket = self._object_bucket_id_from_room(prior_room_id)
            if (
                cached_entry is not None
                and object_signature is not None
                and cached_entry.get("signature") == object_signature
                and prior_bucket in unchanged_room_ids
            ):
                fast_reuse_ids.add(object_id)
                continue
            object_index = current_index_by_id.get(object_id)
            if object_index is None:
                continue
            obj_data, embedding_values = self._build_object_export_record(
                object_index,
                prepared_object_state=prepared_object_state,
                floor_lookup=floor_lookup,
                room_lookup=room_lookup,
                object_floor_vote_cache=object_floor_vote_cache,
            )
            prepared_entries[object_id] = {
                "signature": current_signature_by_id.get(object_id),
                "object": obj_data,
                "embedding": embedding_values,
            }
            current_bucket = self._object_bucket_id_from_room(obj_data.get("room_id"))
            if current_bucket != "__unassigned__" and current_bucket not in current_room_ids:
                return None
            if cached_entry is None:
                _mark_bucket(current_bucket)
                continue
            if object_signature != cached_entry.get("signature"):
                _mark_bucket(current_bucket)
                _mark_bucket(prior_bucket)
                continue
            if prior_bucket != current_bucket:
                _mark_bucket(current_bucket)
                _mark_bucket(prior_bucket)

        disappeared_ids = cached_id_set - current_id_set
        for object_id in sorted(disappeared_ids):
            prior_room_id = ((cached_object_cache.get(object_id) or {}).get("object") or {}).get("room_id")
            _mark_bucket(self._object_bucket_id_from_room(prior_room_id))

        current_rebuild_room_ids = impacted_room_ids & current_room_ids

        objects: List[Dict[str, Any]] = []
        embedding_lookup: Dict[str, List[float]] = {}
        cache_entries: Dict[int, Dict[str, Any]] = {}
        cache_hit_count = 0

        for object_id in current_ids_in_order:
            object_signature = current_signature_by_id.get(object_id)
            if object_id in fast_reuse_ids:
                cached_entry = cached_object_cache.get(object_id)
                if cached_entry is None:
                    return None
                prior_room_id = ((cached_entry.get("object") or {}).get("room_id"))
                prior_bucket = self._object_bucket_id_from_room(prior_room_id)
                if prior_bucket in current_rebuild_room_ids:
                    obj_data, embedding_values = self._build_object_export_record(
                        current_index_by_id[object_id],
                        prepared_object_state=prepared_object_state,
                        floor_lookup=floor_lookup,
                        room_lookup=room_lookup,
                        object_floor_vote_cache=object_floor_vote_cache,
                    )
                    objects.append(obj_data)
                    cache_entries[object_id] = {
                        "signature": object_signature,
                        "object": obj_data,
                        "embedding": embedding_values,
                    }
                    if has_embeddings and obj_data.get("embedding_ref") is not None:
                        embedding_lookup[str(obj_data["embedding_ref"])] = list(embedding_values or [])
                    continue
                reused_object = self._try_reuse_cached_object_export_record(
                    current_index_by_id[object_id],
                    prepared_object_state=prepared_object_state,
                    floor_lookup=floor_lookup,
                    room_lookup=room_lookup,
                    cached_entry=cached_entry,
                )
                if reused_object is None:
                    return None
                objects.append(reused_object)
                cache_entries[object_id] = {
                    "signature": object_signature,
                    "object": reused_object,
                    "embedding": cached_entry.get("embedding"),
                }
                if has_embeddings and reused_object.get("embedding_ref") is not None:
                    embedding_lookup[str(reused_object["embedding_ref"])] = list(cached_entry.get("embedding") or [])
                cache_hit_count += 1
                continue
            prepared_entry = prepared_entries.get(object_id)
            if prepared_entry is None:
                return None
            obj_data = prepared_entry["object"]
            current_bucket = self._object_bucket_id_from_room(obj_data.get("room_id"))
            current_room_changed = (
                current_bucket in current_rebuild_room_ids
                or (current_bucket == "__unassigned__" and unassigned_bucket_rebuilt)
            )
            cached_entry = cached_object_cache.get(object_id)
            if current_room_changed or cached_entry is None or cached_entry.get("signature") != object_signature:
                objects.append(obj_data)
                cache_entries[object_id] = prepared_entry
                if has_embeddings and obj_data.get("embedding_ref") is not None:
                    embedding_lookup[str(obj_data["embedding_ref"])] = list(prepared_entry.get("embedding") or [])
                continue

            prior_room_id = ((cached_entry.get("object") or {}).get("room_id"))
            if prior_room_id != obj_data.get("room_id"):
                return None
            reused_object = self._try_reuse_cached_object_export_record(
                current_index_by_id[object_id],
                prepared_object_state=prepared_object_state,
                floor_lookup=floor_lookup,
                room_lookup=room_lookup,
                cached_entry=cached_entry,
            )
            if reused_object is None:
                return None
            objects.append(reused_object)
            cache_entries[object_id] = {
                "signature": object_signature,
                "object": reused_object,
                "embedding": cached_entry.get("embedding"),
            }
            if has_embeddings and reused_object.get("embedding_ref") is not None:
                embedding_lookup[str(reused_object["embedding_ref"])] = list(cached_entry.get("embedding") or [])
            cache_hit_count += 1

        reused_room_ids = current_room_ids - current_rebuild_room_ids
        return (
            objects,
            embedding_lookup,
            cache_entries,
            int(cache_hit_count),
            {
                "room_local_delta_export_used": True,
                "room_local_delta_changed_room_count": int(len(impacted_room_ids)),
                "room_local_delta_reused_room_count": int(len(reused_room_ids)),
                "room_local_delta_rebuilt_room_count": int(len(current_rebuild_room_ids)),
                "room_local_delta_unassigned_bucket_rebuilt": bool(unassigned_bucket_rebuilt),
                "changed_room_count": int(len(impacted_room_ids)),
                "changed_room_ids": self._encode_room_ids(impacted_room_ids),
                "changed_room_structure_changed_count": int(len(structure_changed_room_ids)),
                "changed_room_structure_changed_ids": self._encode_room_ids(structure_changed_room_ids),
                "changed_room_object_delta_count": int(len(object_delta_room_ids)),
                "changed_room_object_delta_ids": self._encode_room_ids(object_delta_room_ids),
                "changed_room_removed_count": int(len(removed_room_ids)),
                "changed_room_removed_ids": self._encode_room_ids(removed_room_ids),
                "changed_room_local_rebuild_used": True,
                "changed_room_locality_confident": True,
                "full_fallback_rebuild_used": False,
                "rebuilt_object_in_changed_rooms_count": int(len(current_ids_in_order) - cache_hit_count),
            },
        )

    def _build_room_floor_validation(self, room_records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
        room_to_floor = {}
        spanning_rooms = []
        for room in room_records:
            room_id = int(room["id"])
            floor_id = room.get("floor_id")
            previous = room_to_floor.get(room_id)
            if previous is not None and previous != floor_id:
                spanning_rooms.append({"room_id": room_id, "floors": sorted({previous, floor_id})})
            else:
                room_to_floor[room_id] = floor_id
        return {
            "room_count": int(len(room_records)),
            "spanning_room_count": int(len(spanning_rooms)),
            "spanning_rooms": spanning_rooms,
            "valid": len(spanning_rooms) == 0,
        }

    def _build_vertical_transitions(self, room_records: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        room_list = [dict(item) for item in room_records]
        stable_segments = self._stable_floor_segments()
        vertical_transitions: List[Dict[str, Any]] = []
        for idx in range(len(stable_segments) - 1):
            left = stable_segments[idx]
            right = stable_segments[idx + 1]
            if left["floor_id"] == right["floor_id"]:
                continue

            transition_items = [
                item
                for item in self.frame_history
                if int(left["frame_end"]) < int(item.get("frame_idx", 0)) < int(right["frame_start"])
            ]
            source_support = self._room_support_summary(left["items"][-24:], room_list, floor_id=str(left["floor_id"]))
            target_support = self._room_support_summary(right["items"][:24], room_list, floor_id=str(right["floor_id"]))

            interval_items = list(left["items"][-2:]) + transition_items + list(right["items"][:2])
            z_values = [
                float(item.get("pose_z"))
                for item in interval_items
                if item.get("pose_z") is not None
            ]
            z_min = min(z_values) if z_values else None
            z_max = max(z_values) if z_values else None
            z_span = None if z_min is None or z_max is None else float(z_max - z_min)

            connector_label = self._classify_vertical_transition_interval(transition_items, z_span)
            room_association_complete = source_support.get("room_id") is not None and target_support.get("room_id") is not None
            confidence = self._vertical_transition_confidence(
                source_support=source_support,
                target_support=target_support,
                transition_items=transition_items,
                z_span=z_span,
                connector_label=connector_label,
                room_association_complete=room_association_complete,
            )
            status = (
                "supported"
                if room_association_complete
                else (
                    "partial_room_association"
                    if source_support.get("room_id") is not None or target_support.get("room_id") is not None
                    else "floor_change_observed"
                )
            )
            transition_frame_start = int(transition_items[0]["frame_idx"]) if transition_items else int(left["frame_end"])
            transition_frame_end = int(transition_items[-1]["frame_idx"]) if transition_items else int(right["frame_start"])
            supporting_frame_count = int(len(interval_items))
            transition_frame_count = int(len(transition_items))
            source_room_label = None if source_support.get("room_id") is None else f"R{int(source_support['room_id'])}"
            target_room_label = None if target_support.get("room_id") is None else f"R{int(target_support['room_id'])}"
            evidence_summary = (
                f"{connector_label} over {transition_frame_count} transition frame(s); "
                f"rooms={source_room_label or 'unknown'} -> {target_room_label or 'unknown'}; "
                f"z_span={None if z_span is None else round(float(z_span), 3)}m"
            )
            vertical_transitions.append(
                {
                    "transition_id": f"vt_{len(vertical_transitions) + 1}",
                    "type": "vertical_transition",
                    "connector_label": connector_label,
                    "from_room_id": None if source_support.get("room_id") is None else int(source_support["room_id"]),
                    "to_room_id": None if target_support.get("room_id") is None else int(target_support["room_id"]),
                    "from_floor_id": str(left["floor_id"]),
                    "to_floor_id": str(right["floor_id"]),
                    "from_display_floor_id": source_support.get("display_floor_id") or left.get("display_floor_id", left.get("floor_id")),
                    "to_display_floor_id": target_support.get("display_floor_id") or right.get("display_floor_id", right.get("floor_id")),
                    "frame_start": transition_frame_start,
                    "frame_end": transition_frame_end,
                    "transition_frame_start": transition_frame_start,
                    "transition_frame_end": transition_frame_end,
                    "entry_stable_frame": int(left["frame_end"]),
                    "exit_stable_frame": int(right["frame_start"]),
                    "supporting_frame_count": supporting_frame_count,
                    "transition_frame_count": transition_frame_count,
                    "z_min": None if z_min is None else round(float(z_min), 3),
                    "z_max": None if z_max is None else round(float(z_max), 3),
                    "z_span_m": None if z_span is None else round(float(z_span), 3),
                    "from_room_support": source_support,
                    "to_room_support": target_support,
                    "from_position_xy": source_support.get("position_xy"),
                    "to_position_xy": target_support.get("position_xy"),
                    "transition_status_counts": dict(Counter(str(item.get("status")) for item in transition_items)),
                    "transition_floor_votes": dict(Counter(str(item.get("floor_id")) for item in transition_items if item.get("floor_id") is not None)),
                    "room_association_complete": bool(room_association_complete),
                    "edge_eligible": bool(room_association_complete),
                    "confidence": round(float(confidence), 3),
                    "status": status,
                    "evidence_summary": evidence_summary,
                    "notes": "Cross-floor traversal inferred from transition interval plus nearby stable-room association.",
                }
            )
        return vertical_transitions


def _display_floor_ref(floor_id: Optional[str], display_floor_id: Optional[str]) -> str:
    return display_floor_label(
        floor_id,
        display_floor_id,
        include_internal=True,
    )


def _polygon_contains(point_xy: Sequence[float], polygon: Sequence[Sequence[float]]) -> bool:
    if len(polygon) < 3:
        return False
    contour = np.asarray(polygon, dtype=np.float32)
    return cv2.pointPolygonTest(contour, (float(point_xy[0]), float(point_xy[1])), False) >= 0


def _polygon_area(polygon: Sequence[Sequence[float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    pts = np.asarray(polygon, dtype=np.float32)
    x = pts[:, 0]
    y = pts[:, 1]
    return abs(0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _polygon_centroid(polygon: Sequence[Sequence[float]]) -> Tuple[float, float]:
    pts = np.asarray(polygon, dtype=np.float32)
    if len(pts) == 0:
        return (0.0, 0.0)
    if len(pts) < 3:
        return float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
    area = _polygon_area(polygon)
    if area < 1e-6:
        return float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
    cross = pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]
    scale = 1.0 / (6.0 * max(area, 1e-6))
    cx = scale * np.sum((pts[:, 0] + np.roll(pts[:, 0], -1)) * cross)
    cy = scale * np.sum((pts[:, 1] + np.roll(pts[:, 1], -1)) * cross)
    return float(cx), float(cy)


def _safe_norm(point_a: Sequence[float], point_b: Sequence[float]) -> float:
    return float(np.linalg.norm(np.asarray(point_a, dtype=np.float32) - np.asarray(point_b, dtype=np.float32)))


def _best_room_for_pose(
    position_xy: Sequence[float],
    rooms: Sequence[Dict[str, Any]],
    floor_id: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    candidate = _room_candidate_for_pose(position_xy, rooms, floor_id=floor_id, max_distance=2.0)
    return None if candidate is None else dict(candidate["room"])


def _room_candidate_for_pose(
    position_xy: Sequence[float],
    rooms: Sequence[Dict[str, Any]],
    floor_id: Optional[str] = None,
    max_distance: float = 2.0,
) -> Optional[Dict[str, Any]]:
    point_xy = (float(position_xy[0]), float(position_xy[1]))
    candidates = [room for room in rooms if floor_id is None or room.get("floor_id") == floor_id]
    if not candidates:
        return None
    for room in candidates:
        if _polygon_contains(point_xy, room.get("polygon", [])):
            return {
                "room": dict(room),
                "match_type": "polygon_contains",
                "distance_m": 0.0,
            }
    best_room = None
    best_distance = float("inf")
    for room in candidates:
        polygon = room.get("polygon", [])
        if not polygon:
            continue
        centroid = _polygon_centroid(polygon)
        distance = _safe_norm(point_xy, centroid)
        if distance < best_distance:
            best_distance = distance
            best_room = room
    if best_room is not None and best_distance <= float(max_distance):
        return {
            "room": dict(best_room),
            "match_type": "nearest_centroid",
            "distance_m": round(float(best_distance), 3),
        }
    return None
