import json
import csv
import numpy as np
import cv2
import os
import yaml
from typing import Dict, List, Optional, Tuple
from boxfusion.scene_graph_builder import SemanticSceneGraph, RoomNode, ObjectNode
from boxfusion.tier2_enablement import evaluate_tier2_enablement, finalize_tier2_enablement_for_shape

class DynamicRoomSegmenter:
    """
    结合 HOV-SG 密度直方图与分水岭算法的动态房间分割器。
    """
    def __init__(self, resolution=0.05, config=None):
        self.resolution = resolution
        room_cfg = dict((config or {}).get("room_segmentation", config or {}))
        
        # --- [修改点 1: 锁死全局原点] ---
        # 设定一个足够大的物理安全边界（假设室内场景不会超出当前 SLAM 坐标系系原点的 -50米）
        # 这个 origin_x 和 origin_y 在整个系统运行期间绝对不允许改变！
        self.origin_x = -50.0
        self.origin_y = -50.0
        
        # 宽高可以随着点云的输入动态向正方向扩展
        self.grid_width = 0
        self.grid_height = 0
        # -----------------------------

        self.last_room_markers = None
        self.last_gateways = []
        
        self.height_estimated = False
        self.slice_z_min = 2.8 
        self.slice_z_max = 3.7 
        self.full_z_max = 3.8
        # --- [新增：Room Tracking 状态] ---
        # 记录所有被确认的房间。格式: {global_id: {'mask': 2D_array, 'color': (B,G,R)}}
        self.tracked_rooms = {} 
        self.next_global_id = 1  # 递增的全局唯一 ID (等价于 UUID)
        self.tracking_iou_threshold = float(room_cfg.get("tracking_iou_threshold", 0.3)) # 认定为同一个房间的最小重叠率
        self.tracking_missed_cycles = int(room_cfg.get("tracking_missed_cycles", 2))
        self.label_to_global = {}
        self.last_tracking_report = {}
        self.last_door_debug = {}
        self.tier2_policy_base_decision = evaluate_tier2_enablement(config)
        self.tier2_policy_runtime_decision = dict(self.tier2_policy_base_decision)
        self.tier2_cfg = {
            "enabled": False,
            "central_roi": None,
            "small_region_area_ratio_thresh": float(room_cfg.get("tier2_small_region_area_ratio_thresh", 0.2)),
            "newborn_age_thresh": int(room_cfg.get("tier2_newborn_age_thresh", 2)),
            "cut_support_thresh": float(room_cfg.get("tier2_cut_support_thresh", 0.25)),
            "dominant_neighbor_ratio_thresh": float(room_cfg.get("tier2_dominant_neighbor_ratio_thresh", 0.6)),
            "cooldown_ttl": int(room_cfg.get("tier2_cooldown_ttl", 300)),
            "cooldown_seed_strength_thresh_m": float(room_cfg.get("tier2_cooldown_seed_strength_thresh", 0.65)),
            "cooldown_bbox_margin_px": int(room_cfg.get("tier2_cooldown_bbox_margin_px", 12)),
            "cooldown_persistent_seed_frames": int(room_cfg.get("tier2_cooldown_persistent_seed_frames", 2)),
            "region_match_coverage_thresh": float(room_cfg.get("tier2_region_match_coverage_thresh", 0.6)),
            "region_match_area_ratio_max": float(room_cfg.get("tier2_region_match_area_ratio_max", 2.5)),
        }
        self.tier2_cooldowns: List[Dict[str, object]] = []
        self.tier2_region_history: List[Dict[str, object]] = []
        self.tier2_next_cooldown_id = 1
        self.last_tier2_debug = {}
        # --------------------------------

    def _parse_roi(self, roi_value) -> Optional[Tuple[int, int, int, int]]:
        if roi_value is None:
            return None
        if not isinstance(roi_value, (list, tuple)) or len(roi_value) != 4:
            return None
        x0, y0, x1, y1 = [int(v) for v in roi_value]
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def _clip_roi_to_shape(self, roi: Optional[Tuple[int, int, int, int]], shape) -> Optional[Tuple[int, int, int, int]]:
        if roi is None:
            return None
        h, w = shape
        x0, y0, x1, y1 = roi
        x0 = max(0, min(w, x0))
        x1 = max(0, min(w, x1))
        y0 = max(0, min(h, y0))
        y1 = max(0, min(h, y1))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def _boxes_intersect(self, bbox_a: Tuple[int, int, int, int], bbox_b: Tuple[int, int, int, int]) -> bool:
        ax0, ay0, ax1, ay1 = bbox_a
        bx0, by0, bx1, by1 = bbox_b
        return ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1

    def _mask_bbox(self, mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        ys, xs = np.where(mask)
        if len(xs) == 0 or len(ys) == 0:
            return None
        return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1

    def _expand_bbox(self, bbox: Tuple[int, int, int, int], margin: int, shape) -> Tuple[int, int, int, int]:
        x0, y0, x1, y1 = bbox
        h, w = shape
        return (
            max(0, x0 - margin),
            max(0, y0 - margin),
            min(w, x1 + margin),
            min(h, y1 + margin),
        )

    def _tier2_enabled_for_shape(self, shape) -> bool:
        decision = self._resolve_tier2_policy(shape)
        return bool(decision.get("was_enabled", False))

    def _resolve_tier2_policy(self, shape) -> Dict[str, object]:
        runtime_decision = finalize_tier2_enablement_for_shape(self.tier2_policy_base_decision, shape)
        effective_roi = self._parse_roi(runtime_decision.get("effective_roi"))
        self.tier2_policy_runtime_decision = runtime_decision
        self.tier2_cfg["enabled"] = bool(runtime_decision.get("was_enabled", False))
        self.tier2_cfg["central_roi"] = effective_roi if self.tier2_cfg["enabled"] else None
        return runtime_decision

    def _get_active_tier2_cooldowns(self, frame_id: int, shape, mutate_state: bool = True) -> List[Dict[str, object]]:
        active: List[Dict[str, object]] = []
        for cooldown in self.tier2_cooldowns:
            if int(cooldown["expires_at_frame"]) < int(frame_id):
                continue
            clipped_bbox = self._clip_roi_to_shape(tuple(cooldown["bbox"]), shape)
            if clipped_bbox is None:
                continue
            cooldown_entry = cooldown if mutate_state else dict(cooldown)
            cooldown_entry["bbox"] = clipped_bbox
            active.append(cooldown_entry)
        if mutate_state:
            self.tier2_cooldowns = active
        return active

    def _build_cooldown_mask(self, active_cooldowns: List[Dict[str, object]], shape) -> np.ndarray:
        cooldown_mask = np.zeros(shape, dtype=np.uint8)
        for cooldown in active_cooldowns:
            x0, y0, x1, y1 = cooldown["bbox"]
            cooldown_mask[y0:y1, x0:x1] = 255
        return cooldown_mask

    def _contour_centroid(self, contour) -> List[float]:
        moments = cv2.moments(contour)
        if moments["m00"] != 0:
            return [
                round(float(moments["m10"] / moments["m00"]), 3),
                round(float(moments["m01"] / moments["m00"]), 3),
            ]
        x, y, w, h = cv2.boundingRect(contour)
        return [round(float(x + w / 2.0), 3), round(float(y + h / 2.0), 3)]

    def _apply_tier2_seed_cooldown(
        self,
        valid_contours,
        dist: np.ndarray,
        frame_id: int,
        shape,
        mutate_state: bool = True,
        record_decisions: bool = True,
    ):
        active_cooldowns = self._get_active_tier2_cooldowns(frame_id, shape, mutate_state=mutate_state)
        cooldown_mask = self._build_cooldown_mask(active_cooldowns, shape)
        if not active_cooldowns or len(valid_contours) == 0:
            return valid_contours, cooldown_mask, [], []

        kept_contours = []
        suppressed_events = []
        decision_rows = []
        weak_hit_ids = set()
        strength_thresh_px = float(self.tier2_cfg["cooldown_seed_strength_thresh_m"]) / max(self.resolution, 1e-6)
        persistent_seed_frames = max(1, int(self.tier2_cfg["cooldown_persistent_seed_frames"]))

        for seed_id, contour in enumerate(valid_contours, start=1):
            x, y, w, h = cv2.boundingRect(contour)
            contour_bbox = (int(x), int(y), int(x + w), int(y + h))
            overlapping = [cooldown for cooldown in active_cooldowns if self._boxes_intersect(contour_bbox, tuple(cooldown["bbox"]))]
            if not overlapping:
                kept_contours.append(contour)
                continue

            contour_mask = np.zeros(shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            contour_pixels = contour_mask > 0
            contour_area_px = int(contour_pixels.sum())
            peak_dist_px = float(dist[contour_pixels].max()) if contour_area_px > 0 else 0.0
            strong_enough = peak_dist_px >= strength_thresh_px
            centroid = self._contour_centroid(contour)
            overlap_details = []
            for cooldown in overlapping:
                x0, y0, x1, y1 = [int(v) for v in cooldown["bbox"]]
                overlap_px = int((contour_mask[y0:y1, x0:x1] > 0).sum())
                overlap_ratio = float(overlap_px / max(1, contour_area_px))
                previous_hits = int(cooldown.get("weak_seed_hits", 0))
                overlap_details.append(
                    {
                        "cooldown": cooldown,
                        "overlap_px": overlap_px,
                        "overlap_ratio": overlap_ratio,
                        "previous_hits": previous_hits,
                        "persistent_before": previous_hits >= persistent_seed_frames,
                    }
                )

            meaningful_overlap = [detail for detail in overlap_details if detail["overlap_px"] > 0]
            persistent_enough = any(detail["persistent_before"] for detail in meaningful_overlap)
            suppress = (not strong_enough) and bool(meaningful_overlap) and (not persistent_enough)
            if strong_enough:
                decision_reason = "exempt_strong_seed"
            elif not meaningful_overlap:
                decision_reason = "exempt_overlap_too_small"
            elif persistent_enough:
                decision_reason = "exempt_persistent_seed"
            else:
                decision_reason = "suppressed_weak_seed"

            if mutate_state and (not strong_enough):
                for detail in meaningful_overlap:
                    cooldown_id = int(detail["cooldown"]["id"])
                    if cooldown_id in weak_hit_ids:
                        continue
                    detail["cooldown"]["weak_seed_hits"] = detail["previous_hits"] + 1
                    weak_hit_ids.add(cooldown_id)

            if record_decisions:
                for detail in overlap_details:
                    cooldown = detail["cooldown"]
                    post_hits = int(cooldown.get("weak_seed_hits", detail["previous_hits"]))
                    decision_rows.append(
                        {
                            "frame_id": int(frame_id),
                            "seed_id": int(seed_id),
                            "seed_bbox": [int(contour_bbox[0]), int(contour_bbox[1]), int(contour_bbox[2]), int(contour_bbox[3])],
                            "seed_centroid": centroid,
                            "seed_area_px": int(contour_area_px),
                            "cooldown_id": int(cooldown["id"]),
                            "cooldown_bbox": [int(v) for v in cooldown["bbox"]],
                            "overlap_px": int(detail["overlap_px"]),
                            "overlap_ratio": round(float(detail["overlap_ratio"]), 6),
                            "seed_strength_px": round(peak_dist_px, 3),
                            "seed_strength_m": round(peak_dist_px * self.resolution, 3),
                            "strength_threshold_px": round(float(strength_thresh_px), 3),
                            "strength_threshold_m": round(float(self.tier2_cfg["cooldown_seed_strength_thresh_m"]), 3),
                            "persistence_count_before": int(detail["previous_hits"]),
                            "persistence_count_after": int(post_hits),
                            "persistent_seed_frames": int(persistent_seed_frames),
                            "suppression_decision": "suppressed" if suppress else "exempted",
                            "decision_reason": decision_reason,
                        }
                    )

            if suppress:
                suppressed_events.append(
                    {
                        "frame_id": int(frame_id),
                        "seed_id": int(seed_id),
                        "bbox": [int(contour_bbox[0]), int(contour_bbox[1]), int(contour_bbox[2]), int(contour_bbox[3])],
                        "centroid": centroid,
                        "seed_area_px": int(contour_area_px),
                        "peak_dist_px": round(peak_dist_px, 3),
                        "peak_dist_m": round(peak_dist_px * self.resolution, 3),
                        "cooldown_ids": [int(detail["cooldown"]["id"]) for detail in meaningful_overlap],
                        "overlap_ratios": [round(float(detail["overlap_ratio"]), 6) for detail in meaningful_overlap],
                        "persistence_counts_before": [int(detail["previous_hits"]) for detail in meaningful_overlap],
                        "reason": decision_reason,
                    }
                )
                continue

            kept_contours.append(contour)

        return kept_contours, cooldown_mask, suppressed_events, decision_rows

    def _extract_tier2_regions(self, markers: np.ndarray, wall_label: int, roi_bbox: Optional[Tuple[int, int, int, int]]):
        if roi_bbox is None:
            return []
        x0, y0, x1, y1 = roi_bbox
        roi_markers = markers[y0:y1, x0:x1]
        labels = sorted(int(label) for label in np.unique(roi_markers) if label > 0 and label != wall_label)
        regions = []
        for label in labels:
            roi_mask = roi_markers == label
            if not np.any(roi_mask):
                continue
            bbox_local = self._mask_bbox(roi_mask)
            if bbox_local is None:
                continue
            lx0, ly0, lx1, ly1 = bbox_local
            regions.append(
                {
                    "label": int(label),
                    "mask": roi_mask,
                    "area": int(roi_mask.sum()),
                    "bbox": [int(x0 + lx0), int(y0 + ly0), int(x0 + lx1), int(y0 + ly1)],
                }
            )
        return regions

    def _estimate_tier2_region_age(
        self,
        roi_mask: np.ndarray,
        area: int,
        roi_bbox: Tuple[int, int, int, int],
    ) -> int:
        best_age = 1
        coverage_thresh = float(self.tier2_cfg["region_match_coverage_thresh"])
        area_ratio_max = float(self.tier2_cfg["region_match_area_ratio_max"])
        curr_x0, curr_y0, curr_x1, curr_y1 = roi_bbox
        for prev_region in self.tier2_region_history:
            prev_mask = prev_region["mask"]
            prev_area = int(prev_region["area"])
            prev_x0, prev_y0, prev_x1, prev_y1 = prev_region["roi_bbox"]
            overlap_x0 = max(curr_x0, prev_x0)
            overlap_y0 = max(curr_y0, prev_y0)
            overlap_x1 = min(curr_x1, prev_x1)
            overlap_y1 = min(curr_y1, prev_y1)
            if overlap_x1 <= overlap_x0 or overlap_y1 <= overlap_y0:
                continue

            curr_overlap = roi_mask[
                overlap_y0 - curr_y0 : overlap_y1 - curr_y0,
                overlap_x0 - curr_x0 : overlap_x1 - curr_x0,
            ]
            prev_overlap = prev_mask[
                overlap_y0 - prev_y0 : overlap_y1 - prev_y0,
                overlap_x0 - prev_x0 : overlap_x1 - prev_x0,
            ]
            if curr_overlap.shape != prev_overlap.shape:
                continue

            intersection = int(np.logical_and(curr_overlap, prev_overlap).sum())
            if intersection <= 0:
                continue
            coverage = float(intersection / max(1, area))
            area_ratio = float(max(area, prev_area) / max(1, min(area, prev_area)))
            if coverage >= coverage_thresh and area_ratio <= area_ratio_max:
                best_age = max(best_age, int(prev_region["age"]) + 1)
        return best_age

    def _update_tier2_region_history(self, markers: np.ndarray, wall_label: int):
        roi_bbox = self._clip_roi_to_shape(self.tier2_cfg["central_roi"], markers.shape)
        if roi_bbox is None:
            self.tier2_region_history = []
            return

        new_history = []
        for region in self._extract_tier2_regions(markers, wall_label, roi_bbox):
            age = self._estimate_tier2_region_age(region["mask"], int(region["area"]), roi_bbox)
            new_history.append(
                {
                    "label": int(region["label"]),
                    "mask": region["mask"].copy(),
                    "area": int(region["area"]),
                    "age": int(age),
                    "bbox": list(region["bbox"]),
                    "roi_bbox": [int(v) for v in roi_bbox],
                }
            )
        self.tier2_region_history = new_history

    def _create_tier2_cooldown(self, bbox: Tuple[int, int, int, int], frame_id: int, source_label: int, target_label: int):
        cooldown_ttl = max(1, int(self.tier2_cfg["cooldown_ttl"]))
        self.tier2_cooldowns.append(
            {
                "id": int(self.tier2_next_cooldown_id),
                "bbox": tuple(bbox),
                "created_at_frame": int(frame_id),
                "expires_at_frame": int(frame_id + cooldown_ttl),
                "source_label": int(source_label),
                "target_label": int(target_label),
                "weak_seed_hits": 0,
            }
        )
        self.tier2_next_cooldown_id += 1

    def _apply_tier2_local_repair(self, markers: np.ndarray, wall_label: int, segmentation_state: dict, frame_id: int):
        policy_decision = self._resolve_tier2_policy(markers.shape)
        repaired_markers = markers.copy()
        tier2_debug = {
            "enabled": bool(policy_decision.get("was_enabled", False)),
            "roi_bbox": None,
            "candidate_metrics": [],
            "repair_events": [],
            "repaired_room_count": int(len([label for label in np.unique(markers) if label > 0 and label != wall_label])),
            "policy": {
                "mode": policy_decision.get("mode"),
                "was_enabled": bool(policy_decision.get("was_enabled", False)),
                "selected_roi": policy_decision.get("selected_roi"),
                "effective_roi": policy_decision.get("effective_roi"),
                "final_decision_reason": policy_decision.get("final_decision_reason"),
                "confidence": policy_decision.get("confidence"),
                "blocking_signals": policy_decision.get("blocking_signals", []),
            },
        }
        roi_bbox = self._clip_roi_to_shape(self.tier2_cfg["central_roi"], markers.shape)
        if not self._tier2_enabled_for_shape(markers.shape) or roi_bbox is None:
            return repaired_markers, tier2_debug

        x0, y0, x1, y1 = roi_bbox
        tier2_debug["roi_bbox"] = [int(x0), int(y0), int(x1), int(y1)]
        physical_walls = segmentation_state["physical_walls"]
        watershed_cuts = segmentation_state["watershed_cuts"]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        region_entries = self._extract_tier2_regions(markers, wall_label, roi_bbox)
        region_entries.sort(key=lambda item: (int(item["area"]), int(item["label"])))

        for region in region_entries:
            candidate_label = int(region["label"])
            candidate_mask = repaired_markers == candidate_label
            if not np.any(candidate_mask):
                continue

            candidate_bbox = self._mask_bbox(candidate_mask)
            if candidate_bbox is None or not self._boxes_intersect(candidate_bbox, roi_bbox):
                continue

            candidate_area = int(candidate_mask.sum())
            roi_mask = candidate_mask[y0:y1, x0:x1]
            age = self._estimate_tier2_region_age(roi_mask, int(region["area"]), roi_bbox)

            dilated_candidate = cv2.dilate(candidate_mask.astype(np.uint8), kernel)
            neighbor_stats = []
            for neighbor_label in sorted(int(label) for label in np.unique(repaired_markers) if label > 0 and label != wall_label and label != candidate_label):
                neighbor_mask = repaired_markers == neighbor_label
                if not np.any(neighbor_mask):
                    continue
                zone = (dilated_candidate > 0) & (cv2.dilate(neighbor_mask.astype(np.uint8), kernel) > 0)
                if not np.any(zone):
                    continue
                shared_boundary = int(zone.sum())
                wall_supported = int(np.logical_and(zone, physical_walls).sum())
                cut_length = int(np.logical_and(zone, watershed_cuts).sum())
                neighbor_stats.append(
                    {
                        "label": int(neighbor_label),
                        "shared_boundary": int(shared_boundary),
                        "wall_supported": int(wall_supported),
                        "cut_length": int(cut_length),
                        "zone": zone,
                        "area": int(neighbor_mask.sum()),
                    }
                )

            if not neighbor_stats:
                tier2_debug["candidate_metrics"].append(
                    {
                        "frame_id": int(frame_id),
                        "candidate_label": int(candidate_label),
                        "bbox": list(candidate_bbox),
                        "area": int(candidate_area),
                        "age": int(age),
                        "dominant_neighbor_label": -1,
                        "dominant_neighbor_area": 0,
                        "area_ratio": None,
                        "dominant_neighbor_ratio": 0.0,
                        "cut_support": None,
                        "shared_boundary": 0,
                        "wall_supported_cut_length": 0,
                        "unsupported_cut_length": 0,
                        "merged": False,
                        "reason": "no_neighbor_boundary",
                    }
                )
                continue

            total_shared_boundary = int(sum(item["shared_boundary"] for item in neighbor_stats))
            dominant = max(
                neighbor_stats,
                key=lambda item: (int(item["shared_boundary"]), int(item["area"]), -int(item["label"])),
            )
            dominant_neighbor_label = int(dominant["label"])
            dominant_neighbor_area = int(dominant["area"])
            area_ratio = float(candidate_area / max(1, dominant_neighbor_area))
            dominant_neighbor_ratio = float(dominant["shared_boundary"] / max(1, total_shared_boundary))
            supported_cut = int(dominant["wall_supported"])
            unsupported_cut = int(dominant["cut_length"])
            cut_support = float(supported_cut / max(1, supported_cut + unsupported_cut))

            reasons = []
            if area_ratio > float(self.tier2_cfg["small_region_area_ratio_thresh"]):
                reasons.append("area_ratio_too_large")
            if age > int(self.tier2_cfg["newborn_age_thresh"]):
                reasons.append("region_not_newborn")
            if cut_support > float(self.tier2_cfg["cut_support_thresh"]):
                reasons.append("boundary_has_wall_support")
            if dominant_neighbor_ratio < float(self.tier2_cfg["dominant_neighbor_ratio_thresh"]):
                reasons.append("neighbor_attachment_not_dominant")

            should_merge = len(reasons) == 0
            metric_row = {
                "frame_id": int(frame_id),
                "candidate_label": int(candidate_label),
                "bbox": list(candidate_bbox),
                "area": int(candidate_area),
                "age": int(age),
                "dominant_neighbor_label": int(dominant_neighbor_label),
                "dominant_neighbor_area": int(dominant_neighbor_area),
                "area_ratio": round(area_ratio, 6),
                "dominant_neighbor_ratio": round(dominant_neighbor_ratio, 6),
                "cut_support": round(cut_support, 6),
                "shared_boundary": int(dominant["shared_boundary"]),
                "wall_supported_cut_length": int(supported_cut),
                "unsupported_cut_length": int(unsupported_cut),
                "merged": bool(should_merge),
                "reason": "unsupported_child_split" if should_merge else ",".join(reasons),
            }
            tier2_debug["candidate_metrics"].append(metric_row)

            if not should_merge:
                continue

            repaired_markers[candidate_mask] = dominant_neighbor_label
            fill_zone = dominant["zone"] & watershed_cuts & (~physical_walls)
            repaired_markers[fill_zone] = dominant_neighbor_label

            cooldown_bbox = self._expand_bbox(
                candidate_bbox,
                int(self.tier2_cfg["cooldown_bbox_margin_px"]),
                repaired_markers.shape,
            )
            self._create_tier2_cooldown(cooldown_bbox, frame_id, candidate_label, dominant_neighbor_label)
            tier2_debug["repair_events"].append(
                {
                    "frame_id": int(frame_id),
                    "candidate_label": int(candidate_label),
                    "merged_into_label": int(dominant_neighbor_label),
                    "bbox": [int(v) for v in candidate_bbox],
                    "cooldown_bbox": [int(v) for v in cooldown_bbox],
                    "area": int(candidate_area),
                    "area_ratio": round(area_ratio, 6),
                    "dominant_neighbor_ratio": round(dominant_neighbor_ratio, 6),
                    "cut_support": round(cut_support, 6),
                    "age": int(age),
                    "cooldown_ttl": int(self.tier2_cfg["cooldown_ttl"]),
                    "reason": "unsupported_child_split",
                }
            )

        tier2_debug["repaired_room_count"] = int(
            len([label for label in np.unique(repaired_markers) if label > 0 and label != wall_label])
        )
        return repaired_markers, tier2_debug

    def _world_to_grid(self, points):
        pts_2d = np.atleast_2d(points)[:, :2]
        
        u = np.floor((pts_2d[:, 0] - self.origin_x) / self.resolution).astype(int)
        v = np.floor((pts_2d[:, 1] - self.origin_y) / self.resolution).astype(int)
        
        # 严禁 clip！生成合法性掩码 (valid mask)
        valid_mask = (u >= 0) & (u < self.grid_width) & (v >= 0) & (v < self.grid_height)
        
        # 对于越界的点，不强制拉回，而是给一个无效值 -1（下游需要配合过滤）
        u[~valid_mask] = -1
        v[~valid_mask] = -1
        
        return u, v, valid_mask

    def _grid_to_world(self, u, v):
        # 加上 0.5 确保反投时落在网格中心点，这是一个良好的工程契约
        x = (u + 0.5) * self.resolution + self.origin_x
        y = (v + 0.5) * self.resolution + self.origin_y
        return np.array([x, y])

    def _pad_mask_to_shape(self, mask, shape):
        target_h, target_w = shape
        src_h, src_w = mask.shape
        pad_h = max(0, target_h - src_h)
        pad_w = max(0, target_w - src_w)
        padded = np.pad(mask, ((0, pad_h), (0, pad_w)), mode='constant', constant_values=0)
        return padded[:target_h, :target_w]

    def _count_free_space_components(self, free_space):
        num_labels, _ = cv2.connectedComponents((free_space > 0).astype(np.uint8), connectivity=8)
        return max(0, int(num_labels) - 1)

    def _count_room_adjacency(self, markers, wall_label):
        room_labels = np.unique(markers)
        room_labels = room_labels[(room_labels > 0) & (room_labels != wall_label)]
        if len(room_labels) < 2:
            return 0

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        adjacency_pairs = set()
        room_masks = {int(lbl): (markers == lbl).astype(np.uint8) for lbl in room_labels}

        for i, id1 in enumerate(room_labels):
            dilated_1 = cv2.dilate(room_masks[int(id1)], kernel)
            for id2 in room_labels[i + 1:]:
                if cv2.countNonZero(cv2.bitwise_and(dilated_1, room_masks[int(id2)])) > 0:
                    adjacency_pairs.add((int(id1), int(id2)))

        return len(adjacency_pairs)

    def _build_segmentation_state(self, full_map, frame_id=0, mutate_cooldown_state: bool = True, record_cooldown_decisions: bool = True):
        free_space = cv2.bitwise_not(full_map)
        dist = cv2.distanceTransform(free_space, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        dist_norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        max_dist = float(np.max(dist))

        safe_dist_pixels = 0.4 / self.resolution
        if max_dist < safe_dist_pixels:
            safe_dist_pixels = max_dist * 0.6

        _, seed_mask = cv2.threshold(dist, safe_dist_pixels, 255, cv2.THRESH_BINARY)
        seed_mask = seed_mask.astype(np.uint8)
        seed_mask = cv2.morphologyEx(
            seed_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )

        contours, _ = cv2.findContours(seed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area_m = 0.25
        min_area_pixels = (min_area_m / self.resolution) ** 2
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area_pixels]
        cooldown_mask = np.zeros(seed_mask.shape, dtype=np.uint8)
        suppressed_seed_events = []
        cooldown_decision_rows = []
        if self._tier2_enabled_for_shape(seed_mask.shape):
            valid_contours, cooldown_mask, suppressed_seed_events, cooldown_decision_rows = self._apply_tier2_seed_cooldown(
                valid_contours,
                dist,
                frame_id,
                seed_mask.shape,
                mutate_state=mutate_cooldown_state,
                record_decisions=record_cooldown_decisions,
            )

        state = {
            "free_space": free_space,
            "dist": dist,
            "dist_norm": dist_norm,
            "seed_mask": seed_mask,
            "seed_components": len(contours),
            "valid_seed_count": len(valid_contours),
            "max_dist": max_dist,
            "min_area_m": min_area_m,
            "tier2_seed_cooldown_mask": cooldown_mask,
            "tier2_suppressed_seed_events": suppressed_seed_events,
            "tier2_cooldown_seed_decisions": cooldown_decision_rows,
        }

        if len(valid_contours) == 0:
            return state

        pre_markers = np.zeros(dist.shape, dtype=np.int32)
        for i in range(len(valid_contours)):
            cv2.drawContours(pre_markers, valid_contours, i, (i + 1), -1)

        bg_label = len(valid_contours) + 1
        cv2.circle(pre_markers, (3, 3), 1, bg_label, -1)

        watershed_markers = pre_markers.copy()
        full_map_rgb = cv2.cvtColor(full_map, cv2.COLOR_GRAY2BGR)
        cv2.watershed(full_map_rgb, watershed_markers)

        wall_label = bg_label + 1
        final_markers = watershed_markers.copy()
        final_markers[full_map > 0] = wall_label
        final_markers[final_markers == bg_label] = 0
        final_markers[final_markers == -1] = wall_label

        room_labels = np.unique(final_markers)
        room_labels = room_labels[(room_labels > 0) & (room_labels != wall_label)]

        state.update(
            {
                "pre_markers": pre_markers,
                "watershed_markers": watershed_markers,
                "markers": final_markers,
                "wall_label": wall_label,
                "watershed_cuts": watershed_markers == -1,
                "physical_walls": full_map > 0,
                "room_count": int(len(room_labels)),
                "free_space_components": self._count_free_space_components(free_space),
                "adjacency_count": self._count_room_adjacency(final_markers, wall_label),
            }
        )
        return state

    def _collect_door_carving_debug(self, pre_door_state, post_door_state, door_count):
        if pre_door_state is None or post_door_state is None:
            return {}

        return {
            "door_box_count": int(door_count),
            "room_count_pre_door": int(pre_door_state.get("room_count", 0)),
            "room_count_post_door": int(post_door_state.get("room_count", 0)),
            "room_count_changed": bool(pre_door_state.get("room_count", 0) != post_door_state.get("room_count", 0)),
            "free_space_components_pre_door": int(pre_door_state.get("free_space_components", 0)),
            "free_space_components_post_door": int(post_door_state.get("free_space_components", 0)),
            "free_space_topology_changed": bool(
                pre_door_state.get("free_space_components", 0) != post_door_state.get("free_space_components", 0)
            ),
            "adjacency_count_pre_door": int(pre_door_state.get("adjacency_count", 0)),
            "adjacency_count_post_door": int(post_door_state.get("adjacency_count", 0)),
            "room_adjacency_changed": bool(
                pre_door_state.get("adjacency_count", 0) != post_door_state.get("adjacency_count", 0)
            ),
            "seed_count_pre_door": int(pre_door_state.get("valid_seed_count", 0)),
            "seed_count_post_door": int(post_door_state.get("valid_seed_count", 0)),
        }

    def _update_room_tracking(self, markers, wall_label):
        current_labels = np.unique(markers)
        current_labels = current_labels[(current_labels > 0) & (current_labels != wall_label)]
        current_masks = {int(lbl): (markers == lbl).astype(np.uint8) for lbl in current_labels}

        candidate_pairs = []
        for lbl, curr_mask in current_masks.items():
            for gid, hist_data in self.tracked_rooms.items():
                hist_mask = self._pad_mask_to_shape(hist_data['mask'], curr_mask.shape)
                intersection = int(np.logical_and(curr_mask, hist_mask).sum())
                union = int(np.logical_or(curr_mask, hist_mask).sum())
                if union <= 0:
                    continue
                iou = float(intersection / union)
                if intersection > 0:
                    candidate_pairs.append((iou, intersection, int(gid), int(lbl)))

        candidate_pairs.sort(key=lambda item: (-item[0], -item[1], item[2], item[3]))

        matched_labels = set()
        matched_gids = set()
        new_tracked_rooms = {}
        self.label_to_global = {}
        report = {
            "threshold": float(self.tracking_iou_threshold),
            "missed_cycles_limit": int(self.tracking_missed_cycles),
            "matched": [],
            "new_rooms": [],
            "retained_missing": [],
            "dropped_missing": [],
        }

        for iou, intersection, gid, lbl in candidate_pairs:
            if iou < self.tracking_iou_threshold or lbl in matched_labels or gid in matched_gids:
                continue

            hist_data = self.tracked_rooms[gid]
            new_tracked_rooms[gid] = {
                'mask': current_masks[lbl],
                'color': hist_data['color'],
                'missed_cycles': 0,
            }
            self.label_to_global[lbl] = gid
            matched_labels.add(lbl)
            matched_gids.add(gid)
            report["matched"].append(
                {
                    "marker_label": int(lbl),
                    "global_id": int(gid),
                    "iou": round(float(iou), 6),
                    "intersection": int(intersection),
                }
            )

        for lbl in sorted(current_masks):
            if lbl in matched_labels:
                continue
            assigned_id = self.next_global_id
            self.next_global_id += 1
            room_color = tuple(np.random.randint(50, 255, size=3).tolist())
            new_tracked_rooms[assigned_id] = {
                'mask': current_masks[lbl],
                'color': room_color,
                'missed_cycles': 0,
            }
            self.label_to_global[lbl] = assigned_id
            report["new_rooms"].append({"marker_label": int(lbl), "global_id": int(assigned_id)})

        for gid, hist_data in self.tracked_rooms.items():
            if gid in matched_gids:
                continue
            missed_cycles = int(hist_data.get('missed_cycles', 0)) + 1
            if missed_cycles <= self.tracking_missed_cycles:
                retained = dict(hist_data)
                retained['mask'] = self._pad_mask_to_shape(hist_data['mask'], markers.shape)
                retained['missed_cycles'] = missed_cycles
                new_tracked_rooms[gid] = retained
                report["retained_missing"].append({"global_id": int(gid), "missed_cycles": int(missed_cycles)})
            else:
                report["dropped_missing"].append({"global_id": int(gid), "missed_cycles": int(missed_cycles)})

        self.tracked_rooms = new_tracked_rooms
        self.last_tracking_report = report
        return report

    def _room_label_from_world_point(self, point_xy, wall_label):
        u_arr, v_arr, valid_mask = self._world_to_grid(np.asarray([point_xy], dtype=np.float32))
        if not valid_mask[0]:
            return None
        label = int(self.last_room_markers[v_arr[0], u_arr[0]])
        if label <= 0 or label == wall_label or label not in self.label_to_global:
            return None
        return label

    def _vote_room_label_for_object(self, center_xy, footprint_2d, wall_label):
        # Keep the assignment lightweight: sample the footprint support points instead of rasterizing.
        sample_points = [np.asarray(center_xy, dtype=np.float32)]
        if footprint_2d:
            footprint_arr = [np.asarray(pt, dtype=np.float32) for pt in footprint_2d]
            sample_points.extend(footprint_arr)
            for idx in range(len(footprint_arr)):
                sample_points.append(0.5 * (footprint_arr[idx] + footprint_arr[(idx + 1) % len(footprint_arr)]))

        votes = {}
        center_label = None
        for idx, sample in enumerate(sample_points):
            label = self._room_label_from_world_point(sample, wall_label)
            if idx == 0:
                center_label = label
            if label is None:
                continue
            votes[label] = votes.get(label, 0) + 1

        if not votes:
            return -1, center_label, {}

        best_vote = max(votes.values())
        winners = [label for label, count in votes.items() if count == best_vote]
        if len(winners) == 1:
            return self.label_to_global[winners[0]], center_label, votes
        if center_label is not None and center_label in winners:
            return self.label_to_global[center_label], center_label, votes
        return -1, center_label, votes

    def perform_segmentation(self, all_pts_merged, all_pred_box=None, debug_path=None, count=0):
        all_pts_np = np.asarray(all_pts_merged)
        if len(all_pts_np) == 0: return None

        # 1. 动态高度切片
        if not self.height_estimated and len(all_pts_np) > 1000:
            z_values = all_pts_np[:, 2]
            floor_z = np.percentile(z_values, 2)
            ceiling_z = np.percentile(z_values, 98)
            self.slice_z_min = floor_z + 1.5   # 借鉴HOV-SG: 地板上1.5m
            self.slice_z_max = ceiling_z - 0.3 # 借鉴HOV-SG: 天花板下0.3m
            
            # HOV-SG 提取外部轮廓用的全尺寸切片高度 (去除天花板下0.2m)
            self.full_z_max = ceiling_z - 0.2
            self.height_estimated = True
            print(f"\n[RoomSegmenter] 动态高度 -> 地板: {floor_z:.2f}m | 天花板: {ceiling_z:.2f}m")

        max_x = np.max(all_pts_np[:, 0])
        max_y = np.max(all_pts_np[:, 1])
        
        # 计算当前点云需要的最大网格尺寸，并增加 20 个 pixel 的 padding 缓冲
        needed_width = int(np.ceil((max_x - self.origin_x) / self.resolution)) + 20
        needed_height = int(np.ceil((max_y - self.origin_y) / self.resolution)) + 20
        
        # 只增不减，确保画布稳定
        self.grid_width = max(self.grid_width, needed_width)
        self.grid_height = max(self.grid_height, needed_height)

        # 2. 准备 HOV-SG 所需的两组点云切片
        z_mask_walls = (all_pts_np[:, 2] >= self.slice_z_min) & (all_pts_np[:, 2] <= self.slice_z_max)
        z_mask_full = (all_pts_np[:, 2] < self.full_z_max)
        
        pts_walls = all_pts_np[z_mask_walls][:, [0, 1]]
        pts_full = all_pts_np[z_mask_full][:, [0, 1]]

        # 计算网格 Bin 数量时，直接使用当前的 grid_width 和 grid_height
        num_bins = (self.grid_width, self.grid_height)
        hist_range = [[self.origin_x, self.origin_x + self.grid_width * self.resolution], 
                      [self.origin_y, self.origin_y + self.grid_height * self.resolution]]

        # ---------------------------------------------------------
        # 步骤 A: 提取强化的墙壁骨架 (Walls Skeleton) 
        # ---------------------------------------------------------
        if len(pts_walls) == 0: return None
        
        hist, _, _ = np.histogram2d(pts_walls[:, 0], pts_walls[:, 1], bins=num_bins, range=hist_range)
        hist = hist.T  # <--- 【千万别漏】：必须转置，把 (W, H) 变成图像需要的 (H, W)！
        
        # 【核心修复 1】：过滤点云密度异常值！截断前 2% 的极高密度点
        hist_nonzero = hist[hist > 0]
        if len(hist_nonzero) > 0:
            p98 = np.percentile(hist_nonzero, 98)
            hist = np.clip(hist, 0, p98) 

        hist = cv2.normalize(hist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        hist = cv2.GaussianBlur(hist, (5, 5), 1)
        
        hist_threshold = 0.15 * np.max(hist)
        _, walls_skeleton = cv2.threshold(hist, hist_threshold, 255, cv2.THRESH_BINARY)

        walls_skeleton = cv2.copyMakeBorder(walls_skeleton, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
        kernel_cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        walls_skeleton = cv2.morphologyEx(walls_skeleton, cv2.MORPH_CLOSE, kernel_cross, iterations=1)

        # ---------------------------------------------------------
        # 步骤 B: 提取外部物理轮廓 (Outside Boundary)
        # ---------------------------------------------------------
        if len(pts_full) == 0: return None
        
        # <--- 【极度关键】：步骤 B 必须使用完全一致的 range，保证画布绝对重合
        hist_full, _, _ = np.histogram2d(pts_full[:, 0], pts_full[:, 1], bins=num_bins, range=hist_range)
        hist_full = hist_full.T  # <--- 这里也要转置！
        hist_full = cv2.normalize(hist_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        hist_full = cv2.GaussianBlur(hist_full, (21, 21), 2)
        _, outside_boundary = cv2.threshold(hist_full, 0, 255, cv2.THRESH_BINARY)

        outside_boundary = cv2.copyMakeBorder(outside_boundary, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
        kernel_rect5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        outside_boundary = cv2.morphologyEx(outside_boundary, cv2.MORPH_CLOSE, kernel_rect5, iterations=3)

        contours, _ = cv2.findContours(outside_boundary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        outside_boundary = np.zeros_like(outside_boundary)
        cv2.drawContours(outside_boundary, contours, -1, (255, 255, 255), -1)

        # ---------------------------------------------------------
        # 步骤 C: 合并生成 full_map 并处理语义挖空 (门)
        # ---------------------------------------------------------
        full_map_padded = cv2.bitwise_or(walls_skeleton, cv2.bitwise_not(outside_boundary))
        kernel_rect3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        full_map_padded = cv2.morphologyEx(full_map_padded, cv2.MORPH_CLOSE, kernel_rect3, iterations=2)
        
        # 去除 Padding 恢复原图尺寸
        h_p, w_p = full_map_padded.shape
        full_map = full_map_padded[10:h_p-10, 10:w_p-10]
        full_map_before_doors = full_map.copy()
        door_boxes_present = False
        door_box_count = 0

        if all_pred_box is not None:
            corners_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy()
            categories = all_pred_box.categories
            for i in range(len(corners_3d)):
                if 'door' in str(categories[i]).strip().lower():
                    door_boxes_present = True
                    door_box_count += 1
                    pts_2d = corners_3d[i, :, :2]
                    
                    # --- [新增：接收并使用 valid_mask 过滤越界坐标，防止后续崩溃] ---
                    box_u, box_v, valid_mask = self._world_to_grid(pts_2d)
                    box_u = box_u[valid_mask]
                    box_v = box_v[valid_mask]
                    
                    # 只有合法点 >= 3 个才能构成闭合多边形
                    if len(box_u) >= 3: 
                        hull = cv2.convexHull(np.column_stack((box_u, box_v)))
                        cv2.fillPoly(full_map, [hull], 0)

        # ---------------------------------------------------------
        # 步骤 D: 距离变换与分水岭 
        # ---------------------------------------------------------
        segmentation_state = self._build_segmentation_state(full_map, frame_id=count)
        print(
            f"[RoomSegmenter] 距离变换最大值: {segmentation_state['max_dist']:.2f} 像素 "
            f"(约 {segmentation_state['max_dist'] * self.resolution:.2f} 米)"
        )
        print(
            f"[RoomSegmenter] 提取到的原始轮廓: {segmentation_state['seed_components']}, "
            f"有效种子点(> {segmentation_state['min_area_m']}㎡): {segmentation_state['valid_seed_count']}"
        )

        if "markers" not in segmentation_state:
            print("[RoomSegmenter] 警告: 未能提取到有效种子点，分割终止。")
            return None

        raw_markers = segmentation_state["markers"]
        wall_label = segmentation_state["wall_label"]
        repaired_markers, tier2_debug = self._apply_tier2_local_repair(raw_markers, wall_label, segmentation_state, count)
        segmentation_state["repaired_markers"] = repaired_markers
        segmentation_state["tier2"] = tier2_debug
        markers = repaired_markers
        self.last_tier2_debug = tier2_debug
        if tier2_debug.get("repair_events"):
            print(
                f"[RoomSegmenter] Tier2 repaired {len(tier2_debug['repair_events'])} unsupported split(s) "
                f"in ROI {tier2_debug.get('roi_bbox')}"
            )
        self._update_tier2_region_history(markers, wall_label)

        # =========================================================
        # --- [新增：Room Tracking (IoU 掩码匹配)] ---
        # =========================================================
        tracking_report = self._update_room_tracking(markers, wall_label)
        # =========================================================

        self.last_room_markers = markers
        self._extract_gateways(markers, wall_label, all_pred_box)

        self.last_door_debug = {}
        if door_boxes_present:
            door_before_state = self._build_segmentation_state(
                full_map_before_doors,
                frame_id=count,
                mutate_cooldown_state=False,
                record_cooldown_decisions=False,
            ) if debug_path else None
            self.last_door_debug = self._collect_door_carving_debug(
                door_before_state,
                segmentation_state,
                door_box_count,
            )
            if self.last_door_debug:
                print(
                    "[RoomSegmenter] Door carving observability -> "
                    f"room_count_changed={self.last_door_debug['room_count_changed']}, "
                    f"room_adjacency_changed={self.last_door_debug['room_adjacency_changed']}, "
                    f"free_space_topology_changed={self.last_door_debug['free_space_topology_changed']}"
                )

        if debug_path:
            self._save_debug(
                debug_path,
                count,
                hist,
                walls_skeleton,
                outside_boundary,
                segmentation_state,
                tracking_report,
                full_map,
                full_map_before_doors if door_boxes_present else None,
                self.last_door_debug,
            )
            
        return markers

    def _extract_gateways(self, markers, wall_label, all_pred_box):
        self.last_gateways = []
        unique_labels = np.unique(markers)
        unique_labels = unique_labels[(unique_labels > 0) & (unique_labels != wall_label)] 
        
        # --- [核心修改 1：将 19x19 (0.95米) 缩小为 5x5 (0.25米)] ---
        # 0.25 米足以跨越 Watershed 算法留下的 1 像素分界线或薄门，但绝对无法穿透标准的承重墙
        kernel_gw = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # --------------------------------------------------------
        
        door_boxes_2d = []
        if all_pred_box is not None:
            corners_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy()
            categories = all_pred_box.categories
            for i in range(len(corners_3d)):
                if 'door' in str(categories[i]).lower():
                    pts_2d = corners_3d[i, :, :2]
                    door_boxes_2d.append(pts_2d)
        
        for i in range(len(unique_labels)):
            for j in range(i + 1, len(unique_labels)):
                id1 = unique_labels[i]
                id2 = unique_labels[j]
                
                mask1 = (markers == id1).astype(np.uint8)
                mask2 = (markers == id2).astype(np.uint8)
                
                # 轻微膨胀寻找真实相邻边界
                dilated_1 = cv2.dilate(mask1, kernel_gw)
                intersection = cv2.bitwise_and(dilated_1, mask2)
                
                if cv2.countNonZero(intersection) > 0:
                    # --- [核心修改 2：提取交集的连通块 (处理两个房间有多个门的情况)] ---
                    num_labels, labels_im, stats, centroids = cv2.connectedComponentsWithStats(intersection, connectivity=8)
                    
                    # 遍历每一个独立的相交区域 (跳过背景 0)
                    for k in range(1, num_labels):
                        # 过滤掉太小的噪点交集 (面积小于 3 个像素的偶然触碰)
                        if stats[k, cv2.CC_STAT_AREA] < 3:
                            continue
                            
                        # 提取该通道的所有像素点
                        y_coords, x_coords = np.where(labels_im == k)
                        points_grid = np.column_stack((x_coords, y_coords))
                        
                        # 1. 计算中心点
                        center_grid = centroids[k]
                        pos_world = self._grid_to_world(center_grid[0], center_grid[1])
                        
                        # 2. 估算物理宽度 (Width) 和 法向朝向 (Yaw)
                        # 利用最小外接矩形 (minAreaRect) 来拟合这个交集线段
                        if len(points_grid) >= 5:
                            rect = cv2.minAreaRect(points_grid.astype(np.float32))
                            (cx, cy), (w, h), angle = rect
                            # Gateway 通常是一条狭长地带，较长的边代表通行宽度
                            length_pixels = max(w, h)
                            width_m = length_pixels * self.resolution
                            # 法向垂直于门洞走向
                            yaw = np.deg2rad(angle) if w > h else np.deg2rad(angle + 90)
                        else:
                            # 像素太少，给个保守默认值
                            width_m = 3 * self.resolution
                            yaw = 0.0

                        # 3. 门洞类型研判 (视觉 Door Check)
                        is_door_visually = False
                        for door_pts in door_boxes_2d:
                            # 注意：这里调用了我们在 P0 修改过的带有 valid_mask 返回值的 _world_to_grid
                            door_u, door_v, valid_mask = self._world_to_grid(door_pts)
                            if not np.any(valid_mask): continue
                            
                            door_poly = np.column_stack((door_u[valid_mask], door_v[valid_mask])).astype(np.float32)
                            if len(door_poly) < 3: continue
                            
                            # 检查交集中心是否落在检测到的门框内，允许 3 像素的外扩容错率
                            if cv2.pointPolygonTest(door_poly, (float(center_grid[0]), float(center_grid[1])), True) >= -3.0:
                                is_door_visually = True
                                break
                        
                        # 4. 获取我们在 P0 建立的全局持久化 ID
                        global_id1 = self.label_to_global.get(id1, int(id1)) if hasattr(self, 'label_to_global') else int(id1)
                        global_id2 = self.label_to_global.get(id2, int(id2)) if hasattr(self, 'label_to_global') else int(id2)

                        self.last_gateways.append({
                            "type": "door" if is_door_visually else "open_passage",
                            "pos_world": [round(float(pos_world[0]), 3), round(float(pos_world[1]), 3)],
                            "connects": [int(global_id1), int(global_id2)],
                            "grid_pos": [int(center_grid[0]), int(center_grid[1])],
                            "width_m": round(float(width_m), 3),
                            "yaw": round(float(yaw), 3)
                        })

    def get_vector_map_data(self, all_pred_box=None, count=None):
        if self.last_room_markers is None: return {}
        
        vector_data = {
            "map_info": {
                "resolution": self.resolution,
                "origin_x": self.origin_x,
                "origin_y": self.origin_y,
                "grid_width": self.grid_width,
                "grid_height": self.grid_height,
                "contract": "floor_mapping_with_center_offset"
            },
            "rooms": [], 
            "gateways": [], 
            "objects": [],
            "anchors": [],
            "relationships": [],
        }
        
        unique_labels = np.unique(self.last_room_markers)
        wall_label = np.max(unique_labels)
        
        for label in unique_labels:
            if label <= 0 or label == wall_label: continue
            if label not in self.label_to_global: continue
            global_id = self.label_to_global[label]
            
            mask = (self.last_room_markers == label).astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                world_pts = []
                for pt in approx:
                    world_xy = self._grid_to_world(pt[0][0], pt[0][1])
                    world_pts.append(world_xy.tolist())
                vector_data["rooms"].append({"id": int(global_id), "polygon": world_pts})

        for gate in self.last_gateways:
            vector_data["gateways"].append(gate)
        
        if all_pred_box is not None:
            box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
            categories = all_pred_box.categories
            instance_ids = all_pred_box.init_id.cpu().numpy()
            
            scores = all_pred_box.scores.cpu().numpy() if hasattr(all_pred_box, 'scores') else np.ones(len(box_tensors))
            has_embeddings = hasattr(all_pred_box, 'embeddings')
            semantic_confidences = all_pred_box.semantic_confidences.cpu().numpy() if hasattr(all_pred_box, 'semantic_confidences') else scores
            association_confidences = all_pred_box.association_confidences.cpu().numpy() if hasattr(all_pred_box, 'association_confidences') else np.ones(len(box_tensors))
            semantic_gaps = all_pred_box.semantic_gaps.cpu().numpy() if hasattr(all_pred_box, 'semantic_gaps') else np.zeros(len(box_tensors))
            view_qualities = all_pred_box.view_qualities.cpu().numpy() if hasattr(all_pred_box, 'view_qualities') else np.ones(len(box_tensors))

            for i in range(len(box_tensors)):
                cx, cy, cz = float(box_tensors[i, 0]), float(box_tensors[i, 1]), float(box_tensors[i, 2])
                dx, dy, dz = float(box_tensors[i, 3]), float(box_tensors[i, 4]), float(box_tensors[i, 5])
                yaw = float(box_tensors[i, 6]) if box_tensors.shape[1] > 6 else 0.0

                cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
                corners_local = np.array([
                    [ dx/2,  dy/2],
                    [-dx/2,  dy/2],
                    [-dx/2, -dy/2],
                    [ dx/2, -dy/2]
                ])
                corners_global = (R @ corners_local.T).T + np.array([cx, cy])
                footprint_2d = np.round(corners_global, 3).tolist()
                room_uuid, center_label, room_votes = self._vote_room_label_for_object((cx, cy), footprint_2d, wall_label)

                obj_data = {
                    "id": int(instance_ids[i]),
                    "label": str(categories[i]),
                    "category": str(categories[i]),
                    "score": round(float(scores[i]), 3),
                    "detection_confidence": round(float(scores[i]), 3),
                    "semantic_confidence": round(float(semantic_confidences[i]), 3),
                    "association_confidence": round(float(association_confidences[i]), 3),
                    "semantic_gap": round(float(semantic_gaps[i]), 3),
                    "view_quality": round(float(view_qualities[i]), 3),
                    "room_uuid": int(room_uuid),
                    "pose": [round(cx, 3), round(cy, 3)],
                    "pose_3d": [round(cx, 3), round(cy, 3), round(cz, 3)],
                    "size": [round(dx, 3), round(dy, 3), round(dz, 3)],
                    "yaw": round(yaw, 3),
                    "footprint_2d": footprint_2d,
                    "room_assignment": {
                        "method": "footprint_vote_9pt",
                        "center_label": int(center_label) if center_label is not None else -1,
                        "votes": {str(int(label)): int(count) for label, count in sorted(room_votes.items())},
                    },
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
                
                if has_embeddings:
                    obj_data["embedding_ref"] = f"embedding_{int(instance_ids[i])}"
                
                vector_data["objects"].append(obj_data)
                
            if has_embeddings:
                emb_dict = {}
                for i, inst_id in enumerate(instance_ids):
                    emb_dict[f"embedding_{int(inst_id)}"] = all_pred_box.embeddings[i].numpy().tolist()
                vector_data["embeddings"] = emb_dict

        # ==========================================================
        # --- [新增：Scene Graph 构建、关系推理与可视化] ---
        # ==========================================================
        if all_pred_box is not None and len(vector_data["objects"]) > 0:
            sg = SemanticSceneGraph()
            embedding_lookup = vector_data.get("embeddings", {})

            # 1. 注入 Room 节点
            for room_info in vector_data["rooms"]:
                room_id = f"room_{room_info['id']}"
                r_node = RoomNode(
                    id=room_id,
                    room_type="unknown",
                    polygon=room_info["polygon"]
                )
                sg.add_room_node(r_node)

            # 2. 注入 Object 节点
            for obj_info in vector_data["objects"]:
                pos_3d = tuple(obj_info.get("pose_3d", [obj_info["pose"][0], obj_info["pose"][1], obj_info["size"][2] / 2.0]))
                bbox_3d = (obj_info["size"][0], obj_info["size"][1], obj_info["size"][2])
                room_uuid = obj_info.get("room_uuid", -1)
                if room_uuid is None or int(room_uuid) < 0:
                    room_id = "room_unknown"
                    if room_id not in sg.graph.nodes:
                        unknown_room = RoomNode(
                            id=room_id,
                            room_type="unknown",
                            polygon=[]
                        )
                        sg.add_room_node(unknown_room)
                else:
                    room_id = f"room_{int(room_uuid)}"
                    if room_id not in sg.graph.nodes:
                        missing_room = RoomNode(
                            id=room_id,
                            room_type="unknown",
                            polygon=[]
                        )
                        sg.add_room_node(missing_room)

                clip_feature = None
                embedding_ref = obj_info.get("embedding_ref")
                if embedding_ref is not None and embedding_ref in embedding_lookup:
                    clip_feature = embedding_lookup[embedding_ref]

                o_node = ObjectNode(
                    id=f"obj_{obj_info['id']}",
                    center=pos_3d,
                    bbox=bbox_3d,
                    label=obj_info.get("label", obj_info["category"]),
                    category=obj_info["category"],
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
                )
                sg.add_object_node(o_node)
                sg.add_inside_relation(o_node.id, room_id)

            # 3. 执行空间关系推理 (阈值可根据你的实际室内尺度微调)
            sg.compute_spatial_relations(dist_threshold=1.0, z_tolerance=0.2)
            sg.build_anchor_layer(debug=False)

            # 4. 将推理出的边导出到 JSON 数据中
            for source, target, data in sg.graph.edges(data=True):
                vector_data["relationships"].append({
                    "source": source,
                    "target": target,
                    "relation": data["relation"]
                })
            vector_data["anchors"] = sg.export_anchor_data()
            
            # 5. 生成并保存 2D 拓扑可视化
            vis_path = f"./debug_room/scene_graph_{count if count is not None else 'latest'}.png"
            try:
                sg.visualize_bev_graph(save_path=vis_path)
            except Exception as e:
                print(f"[警告] 场景图可视化失败: {e}")

        return vector_data

    def _render_room_debug_image(
        self,
        markers: np.ndarray,
        wall_label: int,
        physical_walls: Optional[np.ndarray] = None,
        watershed_cuts: Optional[np.ndarray] = None,
        use_tracking_colors: bool = False,
    ) -> np.ndarray:
        h, w = markers.shape
        vis = np.zeros((h, w, 3), dtype=np.uint8)
        unique_labels = np.unique(markers)

        for label in unique_labels:
            if label <= 0 or label == wall_label:
                continue
            if use_tracking_colors and label in self.label_to_global:
                gid = self.label_to_global[label]
                if gid in self.tracked_rooms:
                    vis[markers == label] = self.tracked_rooms[gid]["color"]
                    continue
            vis[markers == label] = (
                (60 + 53 * int(label)) % 255,
                (90 + 79 * int(label)) % 255,
                (130 + 31 * int(label)) % 255,
            )

        if physical_walls is not None:
            vis[physical_walls] = [255, 255, 255]
        if watershed_cuts is not None:
            vis[watershed_cuts] = [0, 0, 255]
        return vis

    # def save_room_mapping_to_yaml(self, all_pred_box, output_path="room_objects.yaml"):
    #     if all_pred_box is None or self.last_room_markers is None: return

    #     box_tensors = all_pred_box.pred_boxes_3d.tensor.cpu().numpy()
    #     categories = all_pred_box.categories
    #     instance_ids = all_pred_box.init_id.cpu().numpy()

    #     u_coords, v_coords = self._world_to_grid(box_tensors[:, :2])
    #     markers = self.last_room_markers
    #     wall_label = np.max(markers)

    #     room_data = {}
    #     for i in range(len(box_tensors)):
    #         u, v = u_coords[i], v_coords[i]
    #         # 增加安全检查以防坐标越界
    #         if 0 <= v < self.grid_height and 0 <= u < self.grid_width:
    #             label = markers[v, u]
    #         else:
    #             label = wall_label

    #         if label > 0 and label != wall_label:
    #             room_key = f"room_{int(label)}"
    #         else:
    #             room_key = "unassigned_or_wall"

    #         if room_key not in room_data:
    #             room_data[room_key] = []

    #         room_data[room_key].append({
    #             "instance_id": int(instance_ids[i]),
    #             "category": str(categories[i]),
    #             "position_world": [round(float(x), 3) for x in box_tensors[i, :3].tolist()]
    #         })

    #     os.makedirs(os.path.dirname(output_path), exist_ok=True)
    #     with open(output_path, 'w', encoding='utf-8') as f:
    #         yaml.dump(room_data, f, allow_unicode=True, sort_keys=False)

    def _save_debug(
        self,
        path,
        count,
        hist,
        walls_skeleton,
        outside_boundary,
        segmentation_state,
        tracking_report,
        full_map,
        full_map_before_doors=None,
        door_debug=None,
    ):
        os.makedirs(path, exist_ok=True)
        cv2.imwrite(f"{path}/run_{count}_01_density_hist.png", hist)
        cv2.imwrite(f"{path}/run_{count}_02_walls_skeleton.png", walls_skeleton)
        cv2.imwrite(f"{path}/run_{count}_03_outside_boundary.png", outside_boundary)
        if full_map_before_doors is not None:
            cv2.imwrite(f"{path}/run_{count}_04a_full_map_pre_doors.png", full_map_before_doors)
            cv2.imwrite(f"{path}/run_{count}_04b_full_map_post_doors.png", full_map)
            cv2.imwrite(
                f"{path}/run_{count}_04c_door_carve_delta.png",
                cv2.absdiff(full_map_before_doors, full_map),
            )
        else:
            cv2.imwrite(f"{path}/run_{count}_04_full_map.png", full_map)

        dist_norm = segmentation_state["dist_norm"]
        raw_markers = segmentation_state["markers"]
        repaired_markers = segmentation_state.get("repaired_markers", raw_markers)
        wall_label = segmentation_state["wall_label"]
        cv2.imwrite(f"{path}/run_{count}_05_free_space.png", segmentation_state["free_space"])
        cv2.imwrite(f"{path}/run_{count}_06_distance_map.png", cv2.applyColorMap(dist_norm, cv2.COLORMAP_JET))
        cv2.imwrite(f"{path}/run_{count}_07_seed_mask.png", segmentation_state["seed_mask"])
        cv2.imwrite(f"{path}/run_{count}_07b_seed_cooldown_mask.png", segmentation_state.get("tier2_seed_cooldown_mask", np.zeros_like(raw_markers, dtype=np.uint8)))
        np.save(f"{path}/run_{count}_08_pre_watershed_markers.npy", segmentation_state["pre_markers"])
        np.save(f"{path}/run_{count}_09_final_labels.npy", raw_markers)
        np.save(f"{path}/run_{count}_09b_repaired_labels.npy", repaired_markers)

        pre_markers_vis = np.zeros((*segmentation_state["pre_markers"].shape, 3), dtype=np.uint8)
        for label in np.unique(segmentation_state["pre_markers"]):
            if label > 0:
                pre_markers_vis[segmentation_state["pre_markers"] == label] = (
                    (60 + 53 * int(label)) % 255,
                    (90 + 79 * int(label)) % 255,
                    (130 + 31 * int(label)) % 255,
                )
        cv2.imwrite(f"{path}/run_{count}_08_pre_watershed_markers.png", pre_markers_vis)

        cuts_vis = np.zeros((*raw_markers.shape, 3), dtype=np.uint8)
        cuts_vis[segmentation_state["physical_walls"]] = [255, 255, 255]
        cuts_vis[segmentation_state["watershed_cuts"]] = [0, 0, 255]
        cv2.imwrite(f"{path}/run_{count}_10_wall_vs_watershed_cuts.png", cuts_vis)

        raw_vis = self._render_room_debug_image(
            raw_markers,
            wall_label,
            physical_walls=segmentation_state["physical_walls"],
            watershed_cuts=segmentation_state["watershed_cuts"],
            use_tracking_colors=False,
        )
        cv2.imwrite(f"{path}/run_{count}_11a_raw_rooms.png", raw_vis)

        vis = self._render_room_debug_image(
            repaired_markers,
            wall_label,
            physical_walls=segmentation_state["physical_walls"],
            watershed_cuts=segmentation_state["watershed_cuts"],
            use_tracking_colors=True,
        )

        for gate in self.last_gateways:
            u, v = gate['grid_pos']
            color = (0, 0, 255) if gate['type'] == 'door' else (0, 255, 255)
            cv2.circle(vis, (u, v), 6, color, -1)

        cv2.imwrite(f"{path}/run_{count}_11b_post_repair_rooms.png", vis)
        cv2.imwrite(f"{path}/run_{count}_11_final_rooms.png", vis)

        tier2_debug = segmentation_state.get("tier2", {})
        candidate_metrics = tier2_debug.get("candidate_metrics", [])
        candidate_metrics_path = f"{path}/run_{count}_13_tier2_candidate_metrics.csv"
        with open(candidate_metrics_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "frame_id",
                    "candidate_label",
                    "bbox",
                    "area",
                    "age",
                    "dominant_neighbor_label",
                    "dominant_neighbor_area",
                    "area_ratio",
                    "dominant_neighbor_ratio",
                    "cut_support",
                    "shared_boundary",
                    "wall_supported_cut_length",
                    "unsupported_cut_length",
                    "merged",
                    "reason",
                ],
            )
            writer.writeheader()
            for row in candidate_metrics:
                writer.writerow(row)

        repair_events_payload = {
            "frame_id": int(count),
            "roi_bbox": tier2_debug.get("roi_bbox"),
            "repair_events": tier2_debug.get("repair_events", []),
            "suppressed_seed_events": segmentation_state.get("tier2_suppressed_seed_events", []),
            "cooldown_seed_decisions": segmentation_state.get("tier2_cooldown_seed_decisions", []),
            "active_cooldowns": [
                {
                    "id": int(cooldown["id"]),
                    "bbox": [int(v) for v in cooldown["bbox"]],
                    "created_at_frame": int(cooldown["created_at_frame"]),
                    "expires_at_frame": int(cooldown["expires_at_frame"]),
                    "source_label": int(cooldown["source_label"]),
                    "target_label": int(cooldown["target_label"]),
                    "weak_seed_hits": int(cooldown.get("weak_seed_hits", 0)),
                }
                for cooldown in self.tier2_cooldowns
            ],
        }
        with open(f"{path}/run_{count}_14_tier2_repair_events.json", "w", encoding="utf-8") as f:
            json.dump(repair_events_payload, f, indent=2)

        cooldown_decisions_path = f"{path}/run_{count}_15_tier2_cooldown_decisions.csv"
        cooldown_decisions = segmentation_state.get("tier2_cooldown_seed_decisions", [])
        cooldown_decision_fields = [
            "frame_id",
            "seed_id",
            "seed_bbox",
            "seed_centroid",
            "seed_area_px",
            "cooldown_id",
            "cooldown_bbox",
            "overlap_px",
            "overlap_ratio",
            "seed_strength_px",
            "seed_strength_m",
            "strength_threshold_px",
            "strength_threshold_m",
            "persistence_count_before",
            "persistence_count_after",
            "persistent_seed_frames",
            "suppression_decision",
            "decision_reason",
        ]
        with open(cooldown_decisions_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=cooldown_decision_fields)
            writer.writeheader()
            for row in cooldown_decisions:
                writer.writerow(row)

        policy_decision_path = f"{path}/tier2a_enablement_decision.json"
        with open(policy_decision_path, "w", encoding="utf-8") as f:
            json.dump(self.tier2_policy_runtime_decision, f, indent=2)

        debug_summary = {
            "seed_components": int(segmentation_state["seed_components"]),
            "valid_seed_count": int(segmentation_state["valid_seed_count"]),
            "room_count": int(segmentation_state["room_count"]),
            "repaired_room_count": int(tier2_debug.get("repaired_room_count", segmentation_state["room_count"])),
            "free_space_components": int(segmentation_state["free_space_components"]),
            "adjacency_count": int(segmentation_state["adjacency_count"]),
            "max_dist": float(segmentation_state["max_dist"]),
            "tracking": tracking_report,
        }
        if door_debug:
            debug_summary["door_carving"] = door_debug
        if tier2_debug:
            debug_summary["tier2"] = {
                "enabled": bool(tier2_debug.get("enabled", False)),
                "roi_bbox": tier2_debug.get("roi_bbox"),
                "mode": tier2_debug.get("policy", {}).get("mode"),
                "policy_reason": tier2_debug.get("policy", {}).get("final_decision_reason"),
                "repair_event_count": int(len(tier2_debug.get("repair_events", []))),
                "suppressed_seed_event_count": int(len(segmentation_state.get("tier2_suppressed_seed_events", []))),
                "candidate_metrics_csv": os.path.basename(candidate_metrics_path),
                "repair_event_log_json": f"run_{count}_14_tier2_repair_events.json",
                "enablement_decision_json": os.path.basename(policy_decision_path),
            }

        with open(f"{path}/run_{count}_12_tracking_report.json", 'w', encoding='utf-8') as f:
            json.dump(debug_summary, f, indent=2)
