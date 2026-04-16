import os
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"

import argparse
import copy
import io
import glob
import itertools
import json
import numpy as np
import yaml
import torch
import torchvision
import sys
import uuid
import open3d as o3d
from pathlib import Path
from PIL import Image
from scipy.spatial.transform import Rotation
from tools.utils import * 
import torch.nn.functional as F
import time
import cv2
from contextlib import nullcontext, redirect_stdout

try:
    import rerun
    import rerun.blueprint as rrb
except ImportError:
    rerun = None
    rrb = None

try:
    import open_clip
except ImportError:
    open_clip = None

from boxfusion.cubify_transformer import make_cubify_transformer

from boxfusion.instances import Instances3D
from boxfusion.preprocessor import Augmentor, Preprocessor

from boxfusion.box_manager import BoxManager
from boxfusion.box_fusion import BoxFusion

from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
from boxfusion.runtime_console import RuntimeConsoleLogger
from boxfusion.runtime_instrumentation import RuntimeInstrumentation

WITHIN_FLOOR_RECENT_STEP_WINDOW = 25
WITHIN_FLOOR_NEAR_XY_THRESHOLD_M = 4.0
READONLY_TAIL_DRIFT_REFERENCE_MIN_RETAINED_COUNT = 60
FAST_GT_RGB_RESIZE = os.environ.get("BOXFUSION_FAST_GT_RGB_RESIZE", "1").strip().lower() not in {"0", "false", "no", "off"}


def _resize_rgb_to_depth(image_chw: torch.Tensor, target_hw) -> torch.Tensor:
    if tuple(image_chw.shape[-2:]) == tuple(target_hw):
        resized = image_chw
    else:
        resized = F.interpolate(
            image_chw.unsqueeze(0).float(),
            size=tuple(target_hw),
            mode="bicubic",
            align_corners=False,
        )[0]
    return resized.permute(1, 2, 0).float().div_(255.0)


def _should_defer_stage3_full_export(
    *,
    frame_idx: int,
    is_keyframe: bool,
    demo_recorder,
    segmentation_markers_available: bool,
) -> bool:
    if not bool(is_keyframe):
        return False
    if demo_recorder is None:
        return False
    if not bool(segmentation_markers_available):
        return False
    return bool(demo_recorder.should_capture(int(frame_idx), segmentation_updated=True))


def _box_center_z_values(instances):
    if instances is None or len(instances) == 0:
        return np.zeros((0,), dtype=np.float32)
    box_tensor = instances.get("pred_boxes_3d").tensor
    if hasattr(box_tensor, "detach"):
        box_tensor = box_tensor.detach().cpu().numpy()
    else:
        box_tensor = np.asarray(box_tensor)
    if box_tensor.ndim != 2 or box_tensor.shape[1] < 3:
        return np.zeros((len(instances),), dtype=np.float32)
    return np.asarray(box_tensor[:, 2], dtype=np.float32)


def _box_center_xy_values(instances):
    if instances is None or len(instances) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    box_tensor = instances.get("pred_boxes_3d").tensor
    if hasattr(box_tensor, "detach"):
        box_tensor = box_tensor.detach().cpu().numpy()
    else:
        box_tensor = np.asarray(box_tensor)
    if box_tensor.ndim != 2 or box_tensor.shape[1] < 2:
        return np.zeros((len(instances), 2), dtype=np.float32)
    return np.asarray(box_tensor[:, :2], dtype=np.float32)


def _build_floor_scoped_candidate_mask(
    all_pred_box,
    retained_count,
    room_segmenter,
    *,
    readonly_object_room_snapshot=None,
    enable_readonly_tail_pruning=True,
    box_manager=None,
    current_frame_idx=None,
    current_pose_xy=None,
    current_pose_matrix=None,
    recent_step_window=WITHIN_FLOOR_RECENT_STEP_WINDOW,
    nearby_xy_threshold_m=WITHIN_FLOOR_NEAR_XY_THRESHOLD_M,
):
    total_count = 0 if all_pred_box is None else int(len(all_pred_box))
    retained_count = int(max(0, min(retained_count, total_count)))
    active_mask = np.ones((total_count,), dtype=np.bool_)
    current_frame_mask = np.zeros((total_count,), dtype=np.bool_)
    if all_pred_box is not None and current_frame_idx is not None and hasattr(all_pred_box, "frame_id"):
        frame_ids = getattr(all_pred_box, "frame_id")
        if hasattr(frame_ids, "detach"):
            frame_ids = frame_ids.detach().cpu().numpy()
        else:
            frame_ids = np.asarray(frame_ids)
        frame_ids = np.asarray(frame_ids).reshape(-1)
        if frame_ids.shape[0] == total_count:
            current_frame_mask = frame_ids == int(current_frame_idx)
    retained_history_count = int(np.count_nonzero(~current_frame_mask[:retained_count]))
    observation = dict(getattr(room_segmenter, "last_floor_observation", {}) or {})
    active_floor_id = observation.get("floor_id")
    active_floor_status = observation.get("status")
    profile = {
        "active": False,
        "active_floor_id": None if active_floor_id is None else str(active_floor_id),
        "retained_before_count": int(retained_history_count),
        "retained_after_count": int(retained_history_count),
        "retained_pruned_count": 0,
        "retained_ambiguous_count": 0,
        "within_floor_active": False,
        "within_floor_recent_step_window": int(recent_step_window),
        "within_floor_nearby_xy_threshold_m": float(nearby_xy_threshold_m),
        "same_floor_stable_count": 0,
        "same_floor_recent_kept_count": 0,
        "same_floor_near_kept_count": 0,
        "same_floor_recent_or_near_kept_count": 0,
        "same_floor_fallback_kept_count": 0,
        "same_floor_pruned_within_floor_count": 0,
        "same_floor_missing_recency_count": 0,
        "same_floor_missing_xy_count": 0,
        "active_room_id": None,
        "cheap_room_pruning_active": False,
        "cheap_room_pruning_reason": "inactive",
        "cheap_room_pruning_tail_candidate_count": 0,
        "cheap_room_pruning_ambiguous_candidate_count": 0,
        "cheap_room_pruning_fallback_candidate_count": 0,
        "cheap_room_pruning_trusted_candidate_count": 0,
        "cheap_room_pruning_pruned_count": 0,
        "cheap_room_pruning_tail_retained_count": 0,
        "cheap_room_pruning_same_room_kept_count": 0,
        "cheap_room_pruning_untrusted_kept_count": 0,
        "reason": "no_retained_history",
    }
    if retained_history_count <= 0:
        return active_mask, profile
    if active_floor_id is None or active_floor_status != "stable":
        profile["reason"] = "current_frame_not_assigned_to_stable_floor"
        return active_mask, profile

    retained_center_z = _box_center_z_values(all_pred_box[:retained_count])
    retained_center_xy = _box_center_xy_values(all_pred_box[:retained_count])
    pose_xy = None
    if current_pose_xy is not None:
        pose_xy_arr = np.asarray(current_pose_xy, dtype=np.float32).reshape(-1)
        if pose_xy_arr.shape[0] >= 2 and np.all(np.isfinite(pose_xy_arr[:2])):
            pose_xy = pose_xy_arr[:2]
    latest_step_indices = None
    current_step_index = None
    if box_manager is not None and current_frame_idx is not None:
        latest_step_indices, _ = box_manager.latest_contributing_profiled_steps()
        current_step_index = box_manager.profiled_step_index_for_frame(current_frame_idx)
    ambiguous_count = 0
    pruned_count = 0
    same_floor_stable_count = 0
    same_floor_recent_kept_count = 0
    same_floor_near_kept_count = 0
    same_floor_recent_or_near_kept_count = 0
    same_floor_fallback_kept_count = 0
    same_floor_pruned_within_floor_count = 0
    same_floor_missing_recency_count = 0
    same_floor_missing_xy_count = 0
    ambiguous_kept_mask = np.zeros((retained_count,), dtype=np.bool_)
    fallback_kept_mask = np.zeros((retained_count,), dtype=np.bool_)
    for idx, center_z in enumerate(retained_center_z.tolist()):
        if current_frame_mask[idx]:
            continue
        assignment = dict(room_segmenter.floor_manager.assign_height(float(center_z)) or {})
        assigned_floor_id = assignment.get("floor_id")
        assigned_status = assignment.get("status")
        if assigned_floor_id is None or assigned_status != "stable":
            ambiguous_count += 1
            ambiguous_kept_mask[idx] = True
            continue
        if str(assigned_floor_id) == str(active_floor_id):
            same_floor_stable_count += 1
            recent_ok = False
            near_ok = False
            recency_available = False
            xy_available = False

            if latest_step_indices is not None and current_step_index is not None and idx < len(latest_step_indices):
                latest_step_idx = int(latest_step_indices[idx])
                if latest_step_idx >= 0:
                    recency_available = True
                    recent_ok = int(current_step_index) - latest_step_idx <= int(recent_step_window)

            if pose_xy is not None and idx < len(retained_center_xy):
                candidate_xy = np.asarray(retained_center_xy[idx], dtype=np.float32)
                if candidate_xy.shape[0] >= 2 and np.all(np.isfinite(candidate_xy[:2])):
                    xy_available = True
                    near_ok = float(np.linalg.norm(candidate_xy[:2] - pose_xy[:2])) <= float(nearby_xy_threshold_m)

            if recent_ok:
                same_floor_recent_kept_count += 1
            if near_ok:
                same_floor_near_kept_count += 1
            if recent_ok or near_ok:
                same_floor_recent_or_near_kept_count += 1
                continue

            if not recency_available:
                same_floor_missing_recency_count += 1
            if not xy_available:
                same_floor_missing_xy_count += 1
            if (not recency_available) or (not xy_available):
                same_floor_fallback_kept_count += 1
                fallback_kept_mask[idx] = True
                continue

            active_mask[idx] = False
            same_floor_pruned_within_floor_count += 1
            continue

        active_mask[idx] = False
        pruned_count += 1

    profile.update(
        {
            "active": True,
            "within_floor_active": True,
            "retained_after_count": int(retained_history_count - pruned_count - same_floor_pruned_within_floor_count),
            "retained_pruned_count": int(pruned_count),
            "retained_ambiguous_count": int(ambiguous_count),
            "same_floor_stable_count": int(same_floor_stable_count),
            "same_floor_recent_kept_count": int(same_floor_recent_kept_count),
            "same_floor_near_kept_count": int(same_floor_near_kept_count),
            "same_floor_recent_or_near_kept_count": int(same_floor_recent_or_near_kept_count),
            "same_floor_fallback_kept_count": int(same_floor_fallback_kept_count),
            "same_floor_pruned_within_floor_count": int(same_floor_pruned_within_floor_count),
            "same_floor_missing_recency_count": int(same_floor_missing_recency_count),
            "same_floor_missing_xy_count": int(same_floor_missing_xy_count),
            "reason": "same_stable_floor_recent_or_near",
        }
    )

    tail_candidate_mask = np.asarray(
        active_mask[:retained_count]
        & (~current_frame_mask[:retained_count])
        & (ambiguous_kept_mask | fallback_kept_mask),
        dtype=np.bool_,
    )
    tail_candidate_count = int(np.count_nonzero(tail_candidate_mask))
    ambiguous_tail_count = int(np.count_nonzero(tail_candidate_mask & ambiguous_kept_mask))
    fallback_tail_count = int(np.count_nonzero(tail_candidate_mask & fallback_kept_mask))
    profile.update(
        {
            "cheap_room_pruning_tail_candidate_count": int(tail_candidate_count),
            "cheap_room_pruning_ambiguous_candidate_count": int(ambiguous_tail_count),
            "cheap_room_pruning_fallback_candidate_count": int(fallback_tail_count),
            "cheap_room_pruning_tail_retained_count": int(tail_candidate_count),
        }
    )

    topology_status = {}
    if current_pose_matrix is not None:
        topology_status = dict(room_segmenter.describe_topology_status(pose_matrix=current_pose_matrix) or {})
    active_room_id = topology_status.get("active_room_id")
    profile["active_room_id"] = None if active_room_id is None else str(active_room_id)

    if tail_candidate_count <= 0:
        profile["cheap_room_pruning_reason"] = "no_tail_candidates"
        return active_mask, profile
    if not bool(enable_readonly_tail_pruning):
        profile["cheap_room_pruning_reason"] = "readonly_tail_pruning_disabled"
        return active_mask, profile
    if active_room_id is None:
        profile["cheap_room_pruning_reason"] = "active_room_unavailable"
        return active_mask, profile
    snapshot_lookup = dict((readonly_object_room_snapshot or {}).get("by_object_id") or {})
    if not snapshot_lookup:
        profile["cheap_room_pruning_reason"] = "readonly_room_snapshot_unavailable"
        return active_mask, profile

    retained_init_ids = np.asarray(all_pred_box.init_id[:retained_count].detach().cpu().numpy(), dtype=np.int64)
    cached_room_uuid = np.full((retained_count,), -1, dtype=np.int64)
    cached_room_floor_id = np.empty((retained_count,), dtype=object)
    cached_room_floor_id[:] = None
    cached_room_pruning_trusted = np.zeros((retained_count,), dtype=np.bool_)
    for idx, object_id in enumerate(retained_init_ids.tolist()):
        snapshot_entry = dict(snapshot_lookup.get(int(object_id)) or {})
        if not snapshot_entry:
            continue
        cached_room_uuid[idx] = int(snapshot_entry.get("room_uuid", -1) or -1)
        floor_id = snapshot_entry.get("floor_id")
        cached_room_floor_id[idx] = None if floor_id in (None, "") else str(floor_id)
        cached_room_pruning_trusted[idx] = bool(snapshot_entry.get("trusted", False))

    trusted_tail_mask = np.asarray(tail_candidate_mask & cached_room_pruning_trusted, dtype=np.bool_)
    trusted_candidate_count = int(np.count_nonzero(trusted_tail_mask))
    if trusted_candidate_count <= 0:
        profile["cheap_room_pruning_reason"] = "no_trusted_readonly_room_tail_candidates"
        return active_mask, profile

    pruned_by_cached_room = 0
    kept_same_room = 0
    for idx in np.flatnonzero(trusted_tail_mask):
        candidate_floor_id = None if cached_room_floor_id[idx] in (None, "") else str(cached_room_floor_id[idx])
        candidate_room_uuid = int(cached_room_uuid[idx])
        if candidate_floor_id != str(active_floor_id):
            active_mask[idx] = False
            pruned_by_cached_room += 1
            continue
        if candidate_room_uuid == int(active_room_id):
            kept_same_room += 1
            continue
        active_mask[idx] = False
        pruned_by_cached_room += 1

    profile.update(
        {
            "active": True,
            "retained_after_count": int(profile.get("retained_after_count", retained_history_count) - pruned_by_cached_room),
            "cheap_room_pruning_active": True,
            "cheap_room_pruning_reason": "readonly_active_room_tail_pruning",
            "cheap_room_pruning_trusted_candidate_count": int(trusted_candidate_count),
            "cheap_room_pruning_pruned_count": int(pruned_by_cached_room),
            "cheap_room_pruning_tail_retained_count": int(tail_candidate_count - pruned_by_cached_room),
            "cheap_room_pruning_same_room_kept_count": int(kept_same_room),
            "cheap_room_pruning_untrusted_kept_count": int(tail_candidate_count - trusted_candidate_count),
        }
    )
    return active_mask, profile


def _object_scope_label(pruning_profile, fallback_label):
    if bool(pruning_profile.get("cheap_room_pruning_active")):
        return "same_floor_recent_or_near_cached_room_tail_pruned_retained_history"
    if bool(pruning_profile.get("within_floor_active")):
        return "same_floor_recent_or_near_retained_history"
    return "same_floor_retained_history" if bool(pruning_profile.get("active")) else str(fallback_label)


def _object_scope_description(step_name, pruning_profile, fallback_description):
    if not bool(pruning_profile.get("active")):
        return str(fallback_description)
    if bool(pruning_profile.get("cheap_room_pruning_active")):
        if step_name == "spatial_association":
            return "3D OBB NMS over the accepted current-floor recent-or-near retained set, plus a conservative read-only exporter-room snapshot tail keep that only retains ambiguous/fallback history when trusted snapshot metadata agrees with the current active room"
        if step_name == "correspondence_association":
            return "small-object correspondence check uses the accepted current-floor recent-or-near retained set and the post-filter ambiguous/fallback tail only when trusted read-only snapshot metadata agrees with the current active room"
        if step_name == "boxfusion":
            return "scan retained current-floor objects from the accepted baseline, and only keep ambiguous/fallback tail candidates when trusted read-only snapshot metadata agrees with the current active room"
    if bool(pruning_profile.get("within_floor_active")):
        if step_name == "spatial_association":
            return "3D OBB NMS over retained objects on the current stable floor, keeping same-floor history only when recent<=25 profiled steps or XY<=4m, plus all current-frame boxes"
        if step_name == "correspondence_association":
            return "small-object correspondence check is seeded by current-frame keep set and restricted to retained current-floor objects that are recent<=25 profiled steps or XY<=4m"
        if step_name == "boxfusion":
            return "scan retained current-floor objects only when recent<=25 profiled steps or XY<=4m; optimize only fusion lists with >=3 retained observations and not already fused"
    if step_name == "spatial_association":
        return "3D OBB NMS over retained objects on the current stable floor plus all current-frame boxes"
    if step_name == "correspondence_association":
        return "small-object correspondence check is seeded by current-frame keep set and restricted to retained objects on the current stable floor"
    if step_name == "boxfusion":
        return "scan retained objects on the current stable floor; optimize only fusion lists with >=3 retained observations and not already fused"
    return str(fallback_description)


def _object_scope_notes(pruning_profile, extra_note=""):
    parts = [
        f"active_floor_id={pruning_profile.get('active_floor_id')}",
        f"retained_after_combined_filter={int(pruning_profile.get('retained_after_count', 0))}",
        f"retained_pruned_other_floors={int(pruning_profile.get('retained_pruned_count', 0))}",
        f"retained_ambiguous_kept_global={int(pruning_profile.get('retained_ambiguous_count', 0))}",
        f"same_floor_stable_candidates={int(pruning_profile.get('same_floor_stable_count', 0))}",
        f"same_floor_recent_kept={int(pruning_profile.get('same_floor_recent_kept_count', 0))}",
        f"same_floor_near_kept={int(pruning_profile.get('same_floor_near_kept_count', 0))}",
        f"same_floor_recent_or_near_kept={int(pruning_profile.get('same_floor_recent_or_near_kept_count', 0))}",
        f"same_floor_fallback_kept={int(pruning_profile.get('same_floor_fallback_kept_count', 0))}",
        f"same_floor_pruned_within_floor={int(pruning_profile.get('same_floor_pruned_within_floor_count', 0))}",
        f"same_floor_missing_recency={int(pruning_profile.get('same_floor_missing_recency_count', 0))}",
        f"same_floor_missing_xy={int(pruning_profile.get('same_floor_missing_xy_count', 0))}",
        f"active_room_id={pruning_profile.get('active_room_id')}",
        f"cheap_room_tail_candidates={int(pruning_profile.get('cheap_room_pruning_tail_candidate_count', 0))}",
        f"cheap_room_tail_ambiguous_candidates={int(pruning_profile.get('cheap_room_pruning_ambiguous_candidate_count', 0))}",
        f"cheap_room_tail_fallback_candidates={int(pruning_profile.get('cheap_room_pruning_fallback_candidate_count', 0))}",
        f"cheap_room_trusted_tail_candidates={int(pruning_profile.get('cheap_room_pruning_trusted_candidate_count', 0))}",
        f"cheap_room_tail_pruned={int(pruning_profile.get('cheap_room_pruning_pruned_count', 0))}",
        f"cheap_room_tail_retained={int(pruning_profile.get('cheap_room_pruning_tail_retained_count', 0))}",
        f"cheap_room_same_room_kept={int(pruning_profile.get('cheap_room_pruning_same_room_kept_count', 0))}",
        f"cheap_room_untrusted_kept={int(pruning_profile.get('cheap_room_pruning_untrusted_kept_count', 0))}",
        f"cheap_room_reason={pruning_profile.get('cheap_room_pruning_reason')}",
        f"recent_step_window={int(pruning_profile.get('within_floor_recent_step_window', WITHIN_FLOOR_RECENT_STEP_WINDOW))}",
        f"nearby_xy_threshold_m={float(pruning_profile.get('within_floor_nearby_xy_threshold_m', WITHIN_FLOOR_NEAR_XY_THRESHOLD_M)):.1f}",
        f"reason={pruning_profile.get('reason')}",
    ]
    if extra_note:
        parts.append(str(extra_note))
    return "; ".join(parts)


def _instances_field_numpy(instances, field_name, dtype):
    if instances is None or len(instances) == 0 or not hasattr(instances, field_name):
        return np.zeros((0,), dtype=dtype)
    value = getattr(instances, field_name)
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=dtype).reshape(-1)


def _format_id_csv(values, max_items=64):
    ordered = sorted(int(value) for value in values)
    if len(ordered) > max_items:
        visible = ordered[:max_items]
        return ",".join(str(item) for item in visible) + f",...(+{len(ordered) - max_items} more)"
    return ",".join(str(item) for item in ordered)


def _readonly_tail_reference_audit_enabled(runtime_logging_cfg):
    return bool((runtime_logging_cfg or {}).get("enable_readonly_tail_reference_audit", False))


def _readonly_tail_reference_audit_disabled_metrics(reason="audit_disabled"):
    return {
        "assoc_readonly_tail_reference_compared": False,
        "assoc_readonly_tail_reference_reason": str(reason),
        "assoc_readonly_tail_reference_pruned_object_count": 0,
        "assoc_readonly_tail_reference_pruned_object_ids": "",
        "assoc_readonly_tail_reference_object_count": None,
        "assoc_readonly_tail_candidate_object_count": None,
        "assoc_readonly_tail_object_count_delta": 0,
        "assoc_readonly_tail_reference_diverged": False,
        "assoc_readonly_tail_candidate_only_count": 0,
        "assoc_readonly_tail_candidate_only_ids": "",
        "assoc_readonly_tail_reference_only_count": 0,
        "assoc_readonly_tail_reference_only_ids": "",
        "assoc_readonly_tail_candidate_only_current_frame_count": 0,
        "assoc_readonly_tail_reference_only_pruned_tail_count": 0,
        "assoc_readonly_tail_reference_only_pruned_tail_ids": "",
    }


def _measure_readonly_tail_assoc_drift(
    *,
    cfg,
    box_manager,
    gap,
    current_frame_idx,
    intrinsic,
    all_kf_pose,
    image_height,
    image_width,
    pred_instances,
    cur_global_pred_box,
    pre_assoc_all_pred_box,
    pre_assoc_all_poses,
    pre_assoc_cam_poses,
    pre_assoc_box_manager,
    num_before_cat,
    candidate_floor_mask,
    reference_floor_mask,
    candidate_result_all_pred_box,
    internal_stdout_context,
):
    pruned_tail_indices = np.flatnonzero(
        np.asarray(reference_floor_mask[:num_before_cat] & ~candidate_floor_mask[:num_before_cat], dtype=np.bool_)
    )
    pre_assoc_init_ids = _instances_field_numpy(pre_assoc_all_pred_box, "init_id", np.int64)
    pruned_tail_object_ids = set(int(pre_assoc_init_ids[idx]) for idx in pruned_tail_indices.tolist() if idx < len(pre_assoc_init_ids))
    if int(num_before_cat) < int(READONLY_TAIL_DRIFT_REFERENCE_MIN_RETAINED_COUNT):
        return {
            "assoc_readonly_tail_reference_compared": False,
            "assoc_readonly_tail_reference_reason": "below_late_drift_reference_window",
            "assoc_readonly_tail_reference_pruned_object_count": int(len(pruned_tail_object_ids)),
            "assoc_readonly_tail_reference_pruned_object_ids": _format_id_csv(pruned_tail_object_ids),
            "assoc_readonly_tail_reference_object_count": None,
            "assoc_readonly_tail_candidate_object_count": None,
            "assoc_readonly_tail_object_count_delta": 0,
            "assoc_readonly_tail_reference_diverged": False,
            "assoc_readonly_tail_candidate_only_count": 0,
            "assoc_readonly_tail_candidate_only_ids": "",
            "assoc_readonly_tail_reference_only_count": 0,
            "assoc_readonly_tail_reference_only_ids": "",
            "assoc_readonly_tail_candidate_only_current_frame_count": 0,
            "assoc_readonly_tail_reference_only_pruned_tail_count": 0,
            "assoc_readonly_tail_reference_only_pruned_tail_ids": "",
        }
    if not pruned_tail_object_ids:
        return {
            "assoc_readonly_tail_reference_compared": False,
            "assoc_readonly_tail_reference_reason": "no_tail_mask_delta_vs_baseline",
            "assoc_readonly_tail_reference_pruned_object_count": 0,
            "assoc_readonly_tail_reference_pruned_object_ids": "",
            "assoc_readonly_tail_reference_object_count": None,
            "assoc_readonly_tail_candidate_object_count": None,
            "assoc_readonly_tail_object_count_delta": 0,
            "assoc_readonly_tail_reference_diverged": False,
            "assoc_readonly_tail_candidate_only_count": 0,
            "assoc_readonly_tail_candidate_only_ids": "",
            "assoc_readonly_tail_reference_only_count": 0,
            "assoc_readonly_tail_reference_only_ids": "",
            "assoc_readonly_tail_candidate_only_current_frame_count": 0,
            "assoc_readonly_tail_reference_only_pruned_tail_count": 0,
            "assoc_readonly_tail_reference_only_pruned_tail_ids": "",
        }

    shadow_all_pred_box = pre_assoc_all_pred_box.clone()
    shadow_all_poses = np.copy(pre_assoc_all_poses)
    shadow_box_manager = copy.deepcopy(pre_assoc_box_manager)

    with internal_stdout_context():
        shadow_mask, shadow_success_mask = Instances3D.spatial_association(
        shadow_all_pred_box,
        cfg["box_fusion"]["nms_threshold"],
        shadow_box_manager,
        pre_assoc_cam_poses,
        active_candidate_mask=reference_floor_mask,
    )
    shadow_cur_keep_idx = [idx - num_before_cat for idx in shadow_mask if idx >= num_before_cat]
    shadow_cur_success_nms = [idx - num_before_cat for idx in shadow_success_mask if idx >= num_before_cat]
    shadow_keep_idx = np.asarray(shadow_mask)

    if len(shadow_cur_keep_idx) > 0:
        with internal_stdout_context():
            shadow_all_pred_box, shadow_all_poses, shadow_keep_idx = Instances3D.correspondence_association(
                cfg,
                shadow_box_manager,
                shadow_cur_keep_idx,
                shadow_cur_success_nms,
                pred_instances,
                cur_global_pred_box,
                shadow_all_pred_box,
                shadow_all_poses,
                pre_assoc_cam_poses,
                current_frame_idx,
                shadow_mask,
                intrinsic,
                all_kf_pose,
                threshold=cfg["association"]["small_threshold"],
                H=image_height,
                W=image_width,
                allowed_global_indices=np.flatnonzero(reference_floor_mask[:num_before_cat]),
            )
        shadow_box_manager.update(shadow_keep_idx)
        if cfg["box_fusion"]["check_valid"]:
            with internal_stdout_context():
                shadow_all_pred_box = shadow_box_manager.check_valid_num(shadow_all_pred_box, current_frame_idx, gap)
    else:
        shadow_all_pred_box = shadow_all_pred_box[shadow_mask]
        shadow_all_poses = shadow_all_poses[shadow_mask]
        shadow_box_manager.update(shadow_keep_idx)

    candidate_ids = set(int(item) for item in _instances_field_numpy(candidate_result_all_pred_box, "init_id", np.int64).tolist())
    reference_ids = set(int(item) for item in _instances_field_numpy(shadow_all_pred_box, "init_id", np.int64).tolist())
    candidate_only_ids = candidate_ids - reference_ids
    reference_only_ids = reference_ids - candidate_ids
    candidate_frame_ids = _instances_field_numpy(candidate_result_all_pred_box, "frame_id", np.int64)
    candidate_init_ids = _instances_field_numpy(candidate_result_all_pred_box, "init_id", np.int64)
    candidate_only_current_frame_count = 0
    if candidate_frame_ids.shape[0] == candidate_init_ids.shape[0]:
        candidate_only_current_frame_count = int(
            sum(
                1
                for object_id, frame_id in zip(candidate_init_ids.tolist(), candidate_frame_ids.tolist())
                if int(object_id) in candidate_only_ids and int(frame_id) == int(current_frame_idx)
            )
        )
    reference_only_pruned_tail_ids = reference_only_ids & pruned_tail_object_ids

    return {
        "assoc_readonly_tail_reference_compared": True,
        "assoc_readonly_tail_reference_reason": "baseline_outer_filter_shadow_reference",
        "assoc_readonly_tail_reference_pruned_object_count": int(len(pruned_tail_object_ids)),
        "assoc_readonly_tail_reference_pruned_object_ids": _format_id_csv(pruned_tail_object_ids),
        "assoc_readonly_tail_reference_object_count": int(len(reference_ids)),
        "assoc_readonly_tail_candidate_object_count": int(len(candidate_ids)),
        "assoc_readonly_tail_object_count_delta": int(len(candidate_ids) - len(reference_ids)),
        "assoc_readonly_tail_reference_diverged": bool(candidate_only_ids or reference_only_ids),
        "assoc_readonly_tail_candidate_only_count": int(len(candidate_only_ids)),
        "assoc_readonly_tail_candidate_only_ids": _format_id_csv(candidate_only_ids),
        "assoc_readonly_tail_reference_only_count": int(len(reference_only_ids)),
        "assoc_readonly_tail_reference_only_ids": _format_id_csv(reference_only_ids),
        "assoc_readonly_tail_candidate_only_current_frame_count": int(candidate_only_current_frame_count),
        "assoc_readonly_tail_reference_only_pruned_tail_count": int(len(reference_only_pruned_tail_ids)),
        "assoc_readonly_tail_reference_only_pruned_tail_ids": _format_id_csv(reference_only_pruned_tail_ids),
    }


def run(
    cfg,
    model,
    dataset,
    clip_model,
    preprocess,
    tokenized_text,
    text_features,
    augmentor,
    preprocessor,
    score_thresh=0.0,
    viz_on_gt_points=False,
    gap=25,
    re_vis=True,
    room_seg_interval=100,
    demo_recorder=None,
    debug_room_dir="./debug_room",
    save_scene_graph_vis=True,
    max_frames=None,
    total_frames=None,
    save_point_cloud=True,
    write_debug_room_artifacts=True,
    runtime_console_config=None,
):
    if re_vis and (rerun is None or rrb is None):
        raise ImportError("rerun is required when visualization is enabled. Install rerun or set re_vis=False.")
    is_depth_model = "wide/depth" in augmentor.measurement_keys
    blueprint = None
    if re_vis:
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                contents=[
                    rrb.Horizontal(
                        contents=([
                        rrb.Spatial3DView(
                            name="World",
                            contents=[
                                "+ $origin/**",
                                "+ /device/wide/pred_instances/**",
                                # "+ /world/image/**"
                            ],
                            origin="/world"),
                        ])),
                    rrb.Horizontal(
                        contents=([
                            rrb.Spatial2DView(
                                name="Image",
                                origin="/device/wide/image",
                                contents=[
                                    "+ $origin/**",
                                    "+ /device/wide/pred_instances/**"
                                ])
                        ] + ([
                            # Only show this for RGB-D.
                            rrb.Spatial2DView(
                                name="Depth",
                                origin="/device/wide/depth")
                        ] if is_depth_model else [])),
                        name="Wide")
                ]))

    recording = None
    video_id = None

    device = model.pixel_mean

    count=0
    all_pred_box = None
    all_poses = None

    all_kf_pose = {}
    per_frame_ins = None #save every predicted boxes
    traj_xyz = []

    box_manager = BoxManager(cfg)
    Box_Fuser = BoxFusion(cfg)

    box_count = 0
    start_time = time.time()
    
    room_segmenter = FloorAwareRoomSegmenter(resolution=0.05, config=cfg)
    accumulated_all_pts = []
    latest_vector_map = None
    segmentation_cycle_idx = 0
    last_segmentation_frame_idx = None
    last_demo_frame = None

    if write_debug_room_artifacts:
        os.makedirs(debug_room_dir, exist_ok=True)

    instrumentation_output_dir = (
        Path(getattr(demo_recorder, "log_dir", debug_room_dir)) / "runtime_instrumentation"
    )
    runtime_profiler = RuntimeInstrumentation(
        sequence_id=str(getattr(demo_recorder, "sequence_id", "runtime_session")),
        output_dir=instrumentation_output_dir,
    )
    preinfer_detail_enabled = str(os.environ.get("BOXFUSION_PREINFER_DETAIL", "")).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    console_cfg = dict(cfg.get("runtime_logging", {}) or {})
    if runtime_console_config:
        console_cfg.update(runtime_console_config)
    runtime_artifact_policy = dict(console_cfg.get("runtime_artifact_policy") or {})
    readonly_tail_reference_audit_enabled = _readonly_tail_reference_audit_enabled(console_cfg)
    runtime_console = RuntimeConsoleLogger(
        sequence_id=str(getattr(demo_recorder, "sequence_id", "runtime_session")),
        quiet=bool(console_cfg.get("quiet", False)),
        log_level=str(console_cfg.get("log_level", "summary")),
        runtime_print_interval=console_cfg.get("runtime_print_interval"),
        per_profiled_frame_stdout=console_cfg.get("per_profiled_frame_stdout"),
    )
    runtime_console.scene_start(
        total_frames=total_frames,
        keyframe_gap=int(gap),
        room_seg_interval=int(room_seg_interval),
        instrumentation_dir=str(instrumentation_output_dir),
    )

    def apply_topology_status(frame_idx, pose_matrix):
        topology_status = room_segmenter.describe_topology_status(pose_matrix=pose_matrix)
        runtime_profiler.add_values(
            frame_idx,
            {
                "active_floor_id": topology_status.get("active_floor_id"),
                "active_floor_status": topology_status.get("active_floor_status"),
                "active_room_id": topology_status.get("active_room_id"),
                "active_room_available": bool(topology_status.get("active_room_available")),
                "room_leave_signal_exists": bool(topology_status.get("room_leave_signal_exists")),
                "topology_incremental_online": bool(topology_status.get("topology_incremental_online")),
                "topology_update_mode": topology_status.get("topology_update_mode"),
                "topology_missing_online_trigger_structures": "|".join(
                    str(item) for item in topology_status.get("topology_missing_online_trigger_structures", [])
                ),
            },
        )

    def log_export_profile(frame_idx, *, call_context, call_origin, segmentation_refresh_frame, stage_bucket=None):
        export_metrics = dict(room_segmenter.last_export_profile or {})
        if not export_metrics:
            return {}
        runtime_profiler.log_export_call(
            frame_idx,
            call_context=call_context,
            call_origin=call_origin,
            segmentation_refresh_frame=bool(segmentation_refresh_frame),
            metrics=export_metrics,
        )
        if stage_bucket == "stage3":
            runtime_profiler.add_values(
                frame_idx,
                {
                    "vector_map_export_after_segmentation_sec": float(export_metrics.get("total_sec", 0.0)),
                    "vertical_transition_export_sec": float(export_metrics.get("vertical_transition_export_sec", 0.0)),
                    "room_export_sec": float(export_metrics.get("room_export_sec", 0.0)),
                    "object_export_sec": float(export_metrics.get("object_export_sec", 0.0)),
                    "diagnostics_export_sec": float(export_metrics.get("diagnostics_export_sec", 0.0)),
                },
            )
            room_segmenter.update_latest_segmentation_export_metrics(frame_idx)
        elif stage_bucket == "stage5":
            runtime_profiler.add_values(
                frame_idx,
                {
                    "snapshot_export_sec": float(export_metrics.get("total_sec", 0.0)),
                    "vector_map_export_sec": float(export_metrics.get("total_sec", 0.0)),
                    "scene_graph_build_sec": float(export_metrics.get("scene_graph_build_sec", 0.0)),
                    "spatial_relations_sec": float(export_metrics.get("spatial_relations_sec", 0.0)),
                    "anchor_build_sec": float(export_metrics.get("anchor_build_sec", 0.0)),
                },
            )
        elif stage_bucket == "final":
            room_segmenter.update_latest_segmentation_export_metrics(frame_idx)
        return export_metrics

    def maybe_capture_demo_snapshot(
        frame_idx,
        timestamp,
        image_frame,
        pose_matrix,
        segmentation_updated=False,
        write_authoritative_debug_vector_map=False,
    ):
        nonlocal latest_vector_map
        capture_t0 = time.perf_counter()
        if demo_recorder is None or not demo_recorder.should_capture(frame_idx, segmentation_updated):
            return {"captured": False, "capture_total_sec": 0.0, "snapshot_export_sec": 0.0}

        snapshot_vector_map = latest_vector_map
        snapshot_export_sec = 0.0
        if room_segmenter.last_room_markers is not None:
            snapshot_vector_map = room_segmenter.get_vector_map_data(
                all_pred_box,
                count=frame_idx,
                save_scene_graph_vis=bool(save_scene_graph_vis and getattr(demo_recorder, "save_scene_graph_vis", False)),
                scene_graph_vis_dir=str(getattr(demo_recorder, "scene_graph_dir", debug_room_dir)),
                instrumentation_context="stage5_snapshot_capture",
            )
            latest_vector_map = snapshot_vector_map
            export_metrics = log_export_profile(
                frame_idx,
                call_context="stage5_snapshot_capture",
                call_origin="maybe_capture_demo_snapshot",
                segmentation_refresh_frame=bool(segmentation_updated),
                stage_bucket="stage5",
            )
            snapshot_export_sec = float(export_metrics.get("total_sec", 0.0))
            if write_authoritative_debug_vector_map and write_debug_room_artifacts:
                with open(os.path.join(debug_room_dir, f"vector_map_{frame_idx}.json"), "w", encoding="utf-8") as f:
                    json.dump(snapshot_vector_map, f, indent=2)

        demo_recorder.record_snapshot(
            frame_idx=frame_idx,
            timestamp=timestamp,
            image_rgb=image_frame,
            pose=pose_matrix,
            trajectory_xy=[(float(pt[0]), float(pt[1])) for pt in traj_xyz],
            vector_map=snapshot_vector_map,
            tracking_report=room_segmenter.last_tracking_report,
            segmentation_updated=segmentation_updated,
            segmentation_cycle_idx=segmentation_cycle_idx,
            last_segmentation_frame_idx=last_segmentation_frame_idx,
            export_profile=dict(room_segmenter.last_export_profile or {}),
        )
        return {
            "captured": True,
            "capture_total_sec": float(time.perf_counter() - capture_t0),
            "snapshot_export_sec": float(snapshot_export_sec),
        }

    def sync_segmentation_run_logs():
        runtime_profiler.sync_segmentation_runs(room_segmenter.export_segmentation_runs())

    def internal_stdout_context():
        if runtime_console.is_verbose():
            return nullcontext()
        return redirect_stdout(io.StringIO())
    
    # 在循环外初始化起点
    t_loop_start = time.time()
    
    for sample in dataset:
        if max_frames is not None and count >= max_frames:
            break
        is_last_frame = total_frames is not None and count == total_frames - 1
        is_keyframe = bool(count % gap == 0 or is_last_frame)
        runtime_profiler.mark_frame(
            count,
            sample_kind="frame",
            profiled_frame=bool(is_keyframe),
            is_keyframe=bool(is_keyframe),
            segmentation_refresh_frame=False,
        )
        # ---------------------------------------------------------
        # 阶段 1: 数据加载与预处理 (Data Loading & Preprocessing)
        # ---------------------------------------------------------
        t_data_end = time.time()
        preinfer_profile = {}
        preinfer_sample_unpack_t0 = time.perf_counter()
        
        sample_video_id = sample["meta"]["video_id"] #(['sensor_info', 'wide', 'gt', 'meta'])
        pose = sample['sensor_info'].gt.RT
        
        video_id = sample_video_id
        if ((recording is None) or (video_id != sample_video_id)) and re_vis:
            new_recording = rerun.new_recording(
                application_id=str(sample_video_id), recording_id=uuid.uuid4(), make_default=True)
            new_recording.send_blueprint(blueprint, make_active=True)
            rerun.spawn()
            recording = new_recording
        
        pose_np = pose.squeeze().cpu().numpy()
        RT = sample["sensor_info"].gt.RT[-1].numpy()

        sample_timestamp = float(np.asarray(sample["meta"]["timestamp"]).reshape(-1)[0])
        if re_vis:
            rerun.set_time_seconds("pts", sample_timestamp, recording=recording)

        # -> channels last.
        image = np.moveaxis(sample["wide"]["image"][-1].numpy(), 0, -1)  #[H,W,3]
        last_demo_frame = {
            "frame_idx": int(count),
            "timestamp": float(sample_timestamp),
            "image_rgb": np.asarray(image).copy(),
            "pose": np.asarray(RT, dtype=np.float32).copy(),
        }

        if re_vis:
            color_camera = rerun.Pinhole(
                image_from_camera=sample["sensor_info"].wide.image.K[-1].numpy(), resolution=sample["sensor_info"].wide.image.size)

        if is_depth_model and re_vis:
            # Show the depth being sent to the model.            
            depth_camera = rerun.Pinhole(
                image_from_camera=sample["sensor_info"].wide.depth.K[-1].numpy(), resolution=sample["sensor_info"].wide.depth.size)

        if Box_Fuser.update_K_flag == False:
            Box_Fuser.update_intrinsics(sample["sensor_info"].wide.image.size,sample["sensor_info"].wide.image.K[-1].numpy()) #size:[W,H]
        preinfer_profile["preinfer_sample_unpack_sec"] = float(time.perf_counter() - preinfer_sample_unpack_t0)

        xyzrgb = None
        gt_pointcloud_t0 = time.perf_counter()
        if viz_on_gt_points and sample["sensor_info"].has("gt"):
            # Backproject GT depth to world so we can compare our predictions.
            depth_gt = sample["wide"]["depth"][-1]
            if FAST_GT_RGB_RESIZE:
                matched_image = _resize_rgb_to_depth(sample["wide"]["image"][-1], depth_gt.shape)
            else:
                matched_image = torch.tensor(
                    np.array(Image.fromarray(image).resize((depth_gt.shape[1], depth_gt.shape[0]))),
                    dtype=torch.float32,
                ).div_(255.0)
            # Feel free to change max_depth, but know CA is only trained up to 5m.
            xyz, valid = unproject(depth_gt, sample["sensor_info"].gt.depth.K[-1], pose.squeeze(), max_depth=10.0)
            xyzrgb = torch.cat((xyz, matched_image), dim=-1)[valid]
        preinfer_profile["preinfer_gt_pointcloud_prep_sec"] = float(time.perf_counter() - gt_pointcloud_t0)
                    
        package_t0 = time.perf_counter()
        packaged = augmentor.package(sample)
        preinfer_profile["preinfer_augmentor_package_sec"] = float(time.perf_counter() - package_t0)
        augmentor_profile = dict(getattr(augmentor, "last_profile", {}) or {})
        if preinfer_detail_enabled:
            preinfer_profile["preinfer_augmentor_sensor_info_deepcopy_sec"] = float(
                augmentor_profile.get("sensor_info_deepcopy_sec", 0.0)
            )
            preinfer_profile["preinfer_augmentor_measurement_wrap_sec"] = float(
                augmentor_profile.get("measurement_wrap_sec", 0.0)
            )
        device_move_t0 = time.perf_counter()
        packaged = move_input_to_current_device(packaged, device)
        preinfer_profile["preinfer_device_move_sec"] = float(time.perf_counter() - device_move_t0)
        preprocess_t0 = time.perf_counter()
        packaged = preprocessor.preprocess([packaged])
        preinfer_profile["preinfer_preprocess_total_sec"] = float(time.perf_counter() - preprocess_t0)
        preprocessor_profile = dict(getattr(preprocessor, "last_profile", {}) or {})
        if preinfer_detail_enabled:
            preinfer_profile["preinfer_preprocess_normalize_sec"] = float(
                preprocessor_profile.get("normalize_total_sec", 0.0)
            )
            preinfer_profile["preinfer_preprocess_depth_normalize_sec"] = float(
                preprocessor_profile.get("depth_normalize_sec", 0.0)
            )
            preinfer_profile["preinfer_preprocess_image_normalize_sec"] = float(
                preprocessor_profile.get("image_normalize_sec", 0.0)
            )
            preinfer_profile["preinfer_preprocess_batch_sec"] = float(
                preprocessor_profile.get("batch_total_sec", 0.0)
            )
            preinfer_profile["preinfer_preprocess_image_batch_sec"] = float(
                preprocessor_profile.get("image_batch_sec", 0.0)
            )
            preinfer_profile["preinfer_preprocess_depth_batch_sec"] = float(
                preprocessor_profile.get("depth_batch_sec", 0.0)
            )

        # ---------------------------------------------------------
        # 阶段 2: 主模型推理 (Network Inference)
        # ---------------------------------------------------------
        t_infer_start = time.time()
        preinfer_total_sec = float(t_infer_start - t_data_end)
        preinfer_profile["preinfer_total_sec"] = preinfer_total_sec
        preinfer_accounted_sum_sec = float(
            preinfer_profile.get("preinfer_sample_unpack_sec", 0.0)
            + preinfer_profile.get("preinfer_gt_pointcloud_prep_sec", 0.0)
            + preinfer_profile.get("preinfer_augmentor_package_sec", 0.0)
            + preinfer_profile.get("preinfer_device_move_sec", 0.0)
            + preinfer_profile.get("preinfer_preprocess_total_sec", 0.0)
        )
        preinfer_profile["preinfer_accounted_sum_sec"] = preinfer_accounted_sum_sec
        preinfer_profile["preinfer_unaccounted_remainder_sec"] = float(
            preinfer_total_sec - preinfer_accounted_sum_sec
        )
        runtime_profiler.add_values(count, preinfer_profile)
        frame_floor_observed = False
        
        # Every gap nth frame is selected as keyframe
        if count % gap == 0 or is_last_frame:
            with torch.no_grad():
                pred_instances = model(packaged)[0] 

            pred_instances = pred_instances[pred_instances.scores >= float(score_thresh)]
 
            if cfg["detection"]["uv_bound"]:
                uv_mask = box_manager.check_uv_bounds(pred_instances.pred_proj_xy,image.shape[1],image.shape[0],ratio=cfg["detection"]["uv_bound_value"]) #[N]
                pred_instances = pred_instances[uv_mask]
            if cfg["detection"]["floor_mask"]:
                floor_mask = box_manager.check_floor_mask(pred_instances.pred_boxes_3d.tensor, ratio=cfg["detection"]["floor_ratio"])
                pred_instances = pred_instances[~floor_mask]

            # avoid first frame empty predictions
            if len(pred_instances) == 0 and count ==0:
                with torch.no_grad():
                    pred_instances = model(packaged)[0]
                pred_instances = pred_instances[pred_instances.scores >= float(cfg['detection']['score_thresh']/4)]
                if cfg["detection"]["uv_bound"]:
                    uv_mask = box_manager.check_uv_bounds(pred_instances.pred_proj_xy,image.shape[1],image.shape[0],ratio=cfg["detection"]["uv_bound_value"]) #[N]
                    pred_instances = pred_instances[uv_mask]
            
            # ================= 点云反投影增量收集 =================
            image_rgb = np.moveaxis(sample["wide"]["image"][-1].numpy(), 0, -1)
            depth_map = sample["wide"]["depth"][-1]
            K = sample["sensor_info"].wide.depth.K[-1]
            pose = sample['sensor_info'].gt.RT.squeeze()

            # 反投影并将当前帧有效点云收集起来
            xyz, valid = unproject(depth_map, K, pose, max_depth=8.0)
            matched_image = torch.tensor(np.array(Image.fromarray(image_rgb).resize((depth_map.shape[1], depth_map.shape[0]))))
            xyzrgb = torch.cat((xyz, matched_image / 255.0), dim=-1)[valid]
            
            # --- 极速降采样单帧点云 ---
            xyzrgb_np = xyzrgb.cpu().numpy()
            if xyzrgb_np.shape[0] > 0:
                pcd_frame = o3d.geometry.PointCloud()
                # 【关键修复】：强制转换为 float64 和连续内存，防止 Open3D 报错或静默失败！
                pcd_frame.points = o3d.utility.Vector3dVector(np.ascontiguousarray(xyzrgb_np[:, :3], dtype=np.float64))
                pcd_frame.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(xyzrgb_np[:, 3:6], dtype=np.float64))
                
                pcd_frame = pcd_frame.voxel_down_sample(voxel_size=0.05)
                
                xyzrgb_down = np.concatenate([np.asarray(pcd_frame.points), np.asarray(pcd_frame.colors)], axis=1)
                accumulated_all_pts.append(xyzrgb_down)
                with runtime_profiler.timer(count, "observe_frame_sec"):
                    room_segmenter.observe_frame(
                        frame_idx=count,
                        timestamp=sample_timestamp,
                        pose_matrix=RT,
                        points_xyzrgb=xyzrgb_down,
                        is_keyframe=True,
                    )
                frame_floor_observed = True
                
        t_infer_end = time.time()
        if is_keyframe:
            runtime_profiler.set_value(count, "pred_instance_count", int(len(pred_instances)))

        # ---------------------------------------------------------
        # 阶段 3: 房间拓扑分割 (Room Segmentation)
        # ---------------------------------------------------------
        t_seg_start = time.time()

        if not frame_floor_observed:
            with runtime_profiler.timer(count, "observe_frame_sec"):
                room_segmenter.observe_frame(
                    frame_idx=count,
                    timestamp=sample_timestamp,
                    pose_matrix=RT,
                    points_xyzrgb=None,
                    is_keyframe=False,
                )

        # 将分割触发逻辑提出来，只要是 100 的整数倍帧就会检查，不再受 gap 限制
        segmentation_updated = False
        stage3_export_deferred_to_stage5 = False
        if count % room_seg_interval == 0:
            if len(accumulated_all_pts) > 0:
                all_pts_merged = np.concatenate(accumulated_all_pts, axis=0)
                pcd_global = o3d.geometry.PointCloud()
                pcd_global.points = o3d.utility.Vector3dVector(np.ascontiguousarray(all_pts_merged[:, :3], dtype=np.float64))
                pcd_global.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(all_pts_merged[:, 3:6], dtype=np.float64))
                pcd_global = pcd_global.voxel_down_sample(voxel_size=0.05)
                downsampled_merged_pts = np.concatenate([np.asarray(pcd_global.points), np.asarray(pcd_global.colors)], axis=1)
                accumulated_all_pts = [downsampled_merged_pts]

                markers = room_segmenter.perform_segmentation(
                    all_pred_box=all_pred_box,
                    debug_path=debug_room_dir if write_debug_room_artifacts else None,
                    count=count,
                )
                runtime_profiler.add_values(
                    count,
                    {
                        "floor_merge_sec": float(room_segmenter.last_merge_profile.get("floor_merge_sec", 0.0)),
                        "floor_downsample_sec": float(room_segmenter.last_merge_profile.get("floor_downsample_sec", 0.0)),
                    },
                )
                runtime_profiler.add_values(
                    count,
                    {
                        "merged_point_count_before_merge": int(room_segmenter.last_merge_profile.get("merged_point_count_before_merge", 0)),
                        "merged_point_count_after_downsample": int(room_segmenter.last_merge_profile.get("merged_point_count_after_downsample", 0)),
                    },
                )
                active_floor_id_for_seg = (room_segmenter.last_floor_observation or {}).get("floor_id")
                active_floor_state = None if active_floor_id_for_seg is None else room_segmenter.floor_states.get(str(active_floor_id_for_seg))
                seg_profile = {} if active_floor_state is None else dict(active_floor_state.segmenter.last_segmentation_profile or {})
                runtime_profiler.add_values(
                    count,
                    {
                        "segmentation_total_sec": float(
                            room_segmenter.last_merge_profile.get("floor_merge_sec", 0.0)
                            + room_segmenter.last_merge_profile.get("floor_downsample_sec", 0.0)
                            + seg_profile.get("histogram_build_sec", 0.0)
                            + seg_profile.get("segmentation_state_build_sec", 0.0)
                            + seg_profile.get("room_tracking_sec", 0.0)
                            + seg_profile.get("gateway_extraction_sec", 0.0)
                        ),
                        "histogram_build_sec": float(seg_profile.get("histogram_build_sec", 0.0)),
                        "segmentation_state_build_sec": float(seg_profile.get("segmentation_state_build_sec", 0.0)),
                        "room_tracking_sec": float(seg_profile.get("room_tracking_sec", 0.0)),
                        "gateway_extraction_sec": float(seg_profile.get("gateway_extraction_sec", 0.0)),
                        "wall_slice_point_count": int(seg_profile.get("wall_slice_point_count", 0)),
                        "full_slice_point_count": int(seg_profile.get("full_slice_point_count", 0)),
                        "grid_width": int(seg_profile.get("grid_width", 0)),
                        "grid_height": int(seg_profile.get("grid_height", 0)),
                        "grid_area": int(seg_profile.get("grid_area", 0)),
                        "room_count_before_tracking": int(seg_profile.get("room_count_before_tracking", 0)),
                        "room_count_after_tracking": int(seg_profile.get("room_count_after_tracking", 0)),
                        "tracked_room_count": int(seg_profile.get("tracked_room_count", 0)),
                        "gateway_room_pair_checks": int(seg_profile.get("gateway_room_pair_checks", 0)),
                        "gateway_count": int(seg_profile.get("gateway_count", 0)),
                    },
                )

                if markers is not None:
                    stage3_export_deferred_to_stage5 = _should_defer_stage3_full_export(
                        frame_idx=count,
                        is_keyframe=is_keyframe,
                        demo_recorder=demo_recorder,
                        segmentation_markers_available=bool(room_segmenter.last_room_markers is not None),
                    )
                    vector_map = None
                    if not stage3_export_deferred_to_stage5:
                        vector_map = room_segmenter.get_vector_map_data(
                            all_pred_box,
                            count=count,
                            save_scene_graph_vis=save_scene_graph_vis,
                            scene_graph_vis_dir=debug_room_dir,
                            instrumentation_context="stage3_post_segmentation_refresh",
                        )
                        log_export_profile(
                            count,
                            call_context="stage3_post_segmentation_refresh",
                            call_origin="segmentation_refresh",
                            segmentation_refresh_frame=True,
                            stage_bucket="stage3",
                        )
                        latest_vector_map = vector_map
                        if write_debug_room_artifacts:
                            with open(os.path.join(debug_room_dir, f"vector_map_{count}.json"), "w", encoding="utf-8") as f:
                                json.dump(vector_map, f, indent=2)
                    segmentation_updated = True
                    segmentation_cycle_idx += 1
                    last_segmentation_frame_idx = int(count)
                    runtime_profiler.mark_frame(
                        count,
                        sample_kind="segmentation_refresh" if not is_keyframe else "keyframe_segmentation_refresh",
                        profiled_frame=True,
                        segmentation_refresh_frame=True,
                    )
                    runtime_console.segmentation_refresh(
                        frame_idx=count,
                        active_chunks=len(accumulated_all_pts),
                        updated=True,
                        room_count=None if vector_map is None else len(vector_map.get("rooms", [])),
                        object_count=None if vector_map is None else len(vector_map.get("objects", [])),
                        anchor_count=None if vector_map is None else len(vector_map.get("anchors", [])),
                    )
                sync_segmentation_run_logs()
            else:
                runtime_console.warning(
                    f"frame={int(count)} skipped segmentation refresh because no accumulated point cloud chunks were available"
                )
                
        t_seg_end = time.time()
        runtime_profiler.set_value(count, "stage3_total_sec", float(t_seg_end - t_seg_start))

        # ---------------------------------------------------------
        # 阶段 4: Rerun 可视化发送 (Visualization)
        # ---------------------------------------------------------
        t_rerun_start = time.time()
        
        # Hold off on logging anything until now, since the delay might confuse the user in the visualizer.
        if re_vis:
            pose_transform = rerun.Transform3D(
                translation=RT[:3, 3],
                rotation=rerun.Quaternion(xyzw=Rotation.from_matrix(RT[:3, :3]).as_quat()))
            rerun.log("/world/image", pose_transform)
            rerun.log("/world/image", color_camera)

            rerun.log("/device/wide/image", pose_transform)
            rerun.log("/device/wide/image", rerun.Image(image).compress())
            rerun.log("/device/wide/image", color_camera)
        traj_xyz.append(RT[:3, 3])
            

        if is_depth_model and re_vis:
            rerun.log("/device/wide/depth", rerun.DepthImage(sample["wide"]["depth"][-1].numpy()))
            rerun.log("/device/wide/depth", depth_camera)
        
        if xyzrgb is not None and re_vis:
            rerun.log("/world/xyz", rerun.Points3D(positions=xyzrgb[..., :3], colors=xyzrgb[..., 3:], radii=None))        

        # visualize the trajectory
        if cfg["vis"]["trajectory"] and re_vis:
            rerun.log("/world/trajectory", rerun.LineStrips3D([np.array(traj_xyz)[:count]], colors=[84,255,159]))
            
        t_rerun_end = time.time()

        # ---------------------------------------------------------
        # 阶段 5: BoxFusion 关联与更新 (BoxFusion & Feature Extraction)
        # ---------------------------------------------------------
        t_fusion_start = time.time()
        
        # only process keyframes
        if is_keyframe:
            
            all_kf_pose[count] = pose_np
            pose_np = np.expand_dims(pose_np,axis=0)
            pose_np = np.repeat(pose_np, repeats=len(pred_instances), axis=0) 
            
            if len(pred_instances)==0:
                all_pred_box = all_pred_box
                all_poses = all_poses
                box_count += len(pred_instances)
                box_manager.num_record[count] = box_count
                capture_result = maybe_capture_demo_snapshot(
                    frame_idx=count,
                    timestamp=sample_timestamp,
                    image_frame=image,
                    pose_matrix=RT,
                    segmentation_updated=segmentation_updated,
                    write_authoritative_debug_vector_map=stage3_export_deferred_to_stage5,
                )
                runtime_profiler.add_values(
                    count,
                    {
                        "snapshot_capture_sec": float(capture_result.get("capture_total_sec", 0.0)),
                        "stage5_total_sec": float(time.time() - t_fusion_start),
                        "data_preprocess_sec": float(t_data_end - t_loop_start),
                        "model_bbox_inference_sec": float(t_infer_end - t_infer_start),
                        "rerun_visualization_sec": float(t_rerun_end - t_rerun_start),
                        "total_step_sec": float(time.time() - t_loop_start),
                    },
                )
                if not segmentation_updated:
                    runtime_profiler.mark_frame(count, sample_kind="keyframe", profiled_frame=True, segmentation_refresh_frame=False)
                apply_topology_status(count, RT)
                runtime_profiler.add_values(
                    count,
                    {
                        "all_pred_box_after_association_count": None if all_pred_box is None else int(len(all_pred_box)),
                        "total_retained_object_count": None if all_pred_box is None else int(len(all_pred_box)),
                        "total_floor_count": int(len(room_segmenter.floor_manager.export_floors())),
                        "total_room_count": None if latest_vector_map is None else int(len(latest_vector_map.get("rooms", []))),
                        "total_retained_anchor_count": None if latest_vector_map is None else int(len(latest_vector_map.get("anchors", []))),
                        "current_vertical_transition_count": None if latest_vector_map is None else int(len(latest_vector_map.get("vertical_transitions", []))),
                    },
                )
                if demo_recorder is not None:
                    demo_recorder.record_frame(
                        frame_idx=count,
                        timestamp=sample_timestamp,
                        image_rgb=image,
                        pose=RT,
                        trajectory_xy=[(float(pt[0]), float(pt[1])) for pt in traj_xyz],
                        segmentation_cycle_idx=segmentation_cycle_idx,
                        last_segmentation_frame_idx=last_segmentation_frame_idx,
                    )
                    demo_recorder.record_runtime_growth(
                        frame_idx=count,
                        cumulative_processed_frames=count + 1,
                        vector_map=latest_vector_map,
                        global_box_count=None if all_pred_box is None else len(all_pred_box),
                        stage_timings={
                            "data_preprocess_sec": t_data_end - t_loop_start,
                            "model_bbox_inference_sec": t_infer_end - t_infer_start,
                            "topology_room_segmentation_sec": t_seg_end - t_seg_start,
                            "rerun_visualization_sec": t_rerun_end - t_rerun_start,
                            "feature_boxfusion_sec": time.time() - t_fusion_start,
                            "total_step_sec": time.time() - t_loop_start,
                        },
                        is_keyframe=bool(is_keyframe),
                        segmentation_updated=bool(segmentation_updated),
                    )
                count+=1
                t_loop_start = time.time()
                continue
            
            # add new properties for Instance3D predictions
            pred_instances.categories = np.array(['None'] * len(pred_instances)) # Initialize category labels as 'None' for all predicted instances
            pred_instances.embeddings = torch.zeros((len(pred_instances), 512))
            pred_instances.cam_pose = torch.from_numpy(pose_np) # Convert camera pose from numpy to tensor and assign to instances
            pred_instances.frame_id = torch.tensor([count]).repeat(pose_np.shape[0]) # Assign current frame ID to all instances in this frame
            pred_instances.init_id = box_count+torch.arange(len(pred_instances)) # Create unique initial IDs for each instance based on global box count
            pred_instances.valid_num = torch.zeros(len(pred_instances)) # Initialize validation counter to zero for all instances
            pred_instances.pred_boxes_3d.transform2world(pred_instances.cam_pose) # Transform 3D bounding boxes from camera coordinates to world coordinates
            pred_instances.project_3d_boxes(sample["sensor_info"].wide.depth.K[-1].numpy(), H=image.shape[0],W=image.shape[1]) # Project 3D boxes to 2D image coordinates using camera intrinsics

            # record how many boxes each keyframe has, so we know which box belongs to which frame
            box_count += len(pred_instances)
            box_manager.num_record[count] = box_count
 
            # first keyframe, initialize some data structures
            if all_pred_box is None and count<gap:
                
                #predict the semantic classes
                boxes = pred_instances.pred_boxes.cpu().numpy()
                #scale the boxes by
                boxes = scale_boxes(boxes,image.shape[0],image.shape[1],scale=1.5)

                clip_t0 = time.perf_counter()
                class_results, box_features = text_prompt(boxes, tokenized_text, text_features, image, clip_model, preprocess) #[N_box]
                runtime_profiler.add_value(count, "clip_classification_sec", time.perf_counter() - clip_t0)
                pred_instances.categories = class_results

                # --- [修改点 1：将特征挂载到实例上] ---
                # box_features 是 torch.Tensor，将其保留在 pred_instances 中
                pred_instances.embeddings = box_features.cpu() 
                # ------------------------------------

                all_pred_box = pred_instances
                all_poses = pose_np
                per_frame_ins = pred_instances
 
                #record the current frame boxes info
                box_manager.init_new_predictions(len(pred_instances),0)
                runtime_profiler.add_values(
                    count,
                    {
                        "all_pred_box_before_association_count": 0,
                        "all_pred_box_after_concat_count": int(len(pred_instances)),
                        "all_pred_box_after_association_count": int(len(pred_instances)),
                        "per_frame_ins_count": int(len(per_frame_ins)),
                        "cur_keep_idx_count": 0,
                        "cur_success_nms_count": 0,
                        "small_object_candidate_count": 0,
                        "boxfusion_candidates_scanned": 0,
                        "boxfusion_candidates_eligible": 0,
                        "boxfusion_candidates_optimized": 0,
                        "boxfusion_candidates_updated": 0,
                        "fusion_list_count": int(len(box_manager.fusion_list)),
                        "fusion_list_ge3_count": 0,
                        "fusion_list_len_mean": 1.0 if len(box_manager.fusion_list) > 0 else 0.0,
                        "fusion_list_len_p50": 1.0 if len(box_manager.fusion_list) > 0 else 0.0,
                        "fusion_list_len_p95": 1.0 if len(box_manager.fusion_list) > 0 else 0.0,
                        "fusion_list_len_max": 1.0 if len(box_manager.fusion_list) > 0 else 0.0,
                    },
                )
                runtime_profiler.log_history_scope(
                    count,
                    step_name="initial_keyframe_bootstrap",
                    scope_label="current_frame_only",
                    candidate_pool_before=0,
                    candidate_pool_after=len(pred_instances),
                    retained_history_pool=len(pred_instances),
                    filter_description="no retained history yet; initialize global object store from current frame",
                    notes="bootstrap frame initializes the retained object history",
                )

            else:
                with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                    box_manager.init_new_predictions(len(pred_instances), len(per_frame_ins))

                    num_before_cat = len(all_pred_box)
                    cur_global_pred_box = all_pred_box
                    runtime_profiler.set_value(count, "all_pred_box_before_association_count", int(num_before_cat))

                    all_pred_box = Instances3D.cat([all_pred_box, pred_instances])
                    per_frame_ins = Instances3D.cat([per_frame_ins, pred_instances])
                    runtime_profiler.set_value(count, "all_pred_box_after_concat_count", int(len(all_pred_box)))
                    runtime_profiler.set_value(count, "per_frame_ins_count", int(len(per_frame_ins)))
                    readonly_object_room_snapshot = room_segmenter.get_readonly_object_room_metadata_snapshot()
                with runtime_profiler.timer(count, "stage5_candidate_mask_prep_sec"):
                    if readonly_tail_reference_audit_enabled:
                        with runtime_profiler.timer(count, "stage5_assoc_reference_mask_prep_sec"):
                            association_reference_floor_mask, _ = _build_floor_scoped_candidate_mask(
                                all_pred_box,
                                retained_count=num_before_cat,
                                room_segmenter=room_segmenter,
                                readonly_object_room_snapshot=readonly_object_room_snapshot,
                                enable_readonly_tail_pruning=False,
                                box_manager=box_manager,
                                current_frame_idx=count,
                                current_pose_xy=RT[:2, 3],
                                current_pose_matrix=RT,
                            )
                    else:
                        association_reference_floor_mask = None
                        runtime_profiler.set_value(count, "stage5_assoc_reference_mask_prep_sec", 0.0)
                    with runtime_profiler.timer(count, "stage5_assoc_candidate_mask_prep_sec"):
                        association_floor_mask, association_floor_profile = _build_floor_scoped_candidate_mask(
                            all_pred_box,
                            retained_count=num_before_cat,
                            room_segmenter=room_segmenter,
                            readonly_object_room_snapshot=readonly_object_room_snapshot,
                            box_manager=box_manager,
                            current_frame_idx=count,
                            current_pose_xy=RT[:2, 3],
                            current_pose_matrix=RT,
                        )
                with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                    runtime_profiler.add_values(
                        count,
                        {
                            "object_floor_pruning_active": bool(association_floor_profile.get("active")),
                            "object_floor_pruning_active_floor_id": association_floor_profile.get("active_floor_id"),
                            "object_floor_pruning_reason": association_floor_profile.get("reason"),
                            "object_floor_pruning_retained_before_count": int(association_floor_profile.get("retained_before_count", 0)),
                            "object_floor_pruning_retained_after_count": int(association_floor_profile.get("retained_after_count", 0)),
                            "object_floor_pruning_retained_pruned_count": int(association_floor_profile.get("retained_pruned_count", 0)),
                            "object_floor_pruning_retained_ambiguous_count": int(association_floor_profile.get("retained_ambiguous_count", 0)),
                            "object_within_floor_filter_active": bool(association_floor_profile.get("within_floor_active")),
                            "object_within_floor_same_floor_stable_count": int(association_floor_profile.get("same_floor_stable_count", 0)),
                            "object_within_floor_recent_kept_count": int(association_floor_profile.get("same_floor_recent_kept_count", 0)),
                            "object_within_floor_near_kept_count": int(association_floor_profile.get("same_floor_near_kept_count", 0)),
                            "object_within_floor_recent_or_near_kept_count": int(association_floor_profile.get("same_floor_recent_or_near_kept_count", 0)),
                            "object_within_floor_fallback_kept_count": int(association_floor_profile.get("same_floor_fallback_kept_count", 0)),
                            "object_within_floor_pruned_count": int(association_floor_profile.get("same_floor_pruned_within_floor_count", 0)),
                            "object_within_floor_missing_recency_count": int(association_floor_profile.get("same_floor_missing_recency_count", 0)),
                            "object_within_floor_missing_xy_count": int(association_floor_profile.get("same_floor_missing_xy_count", 0)),
                            "object_cheap_room_pruning_active": bool(association_floor_profile.get("cheap_room_pruning_active")),
                            "object_cheap_room_pruning_reason": association_floor_profile.get("cheap_room_pruning_reason"),
                            "object_cheap_room_pruning_active_room_id": association_floor_profile.get("active_room_id"),
                            "object_cheap_room_pruning_tail_candidate_count": int(association_floor_profile.get("cheap_room_pruning_tail_candidate_count", 0)),
                            "object_cheap_room_pruning_ambiguous_candidate_count": int(association_floor_profile.get("cheap_room_pruning_ambiguous_candidate_count", 0)),
                            "object_cheap_room_pruning_fallback_candidate_count": int(association_floor_profile.get("cheap_room_pruning_fallback_candidate_count", 0)),
                            "object_cheap_room_pruning_trusted_candidate_count": int(association_floor_profile.get("cheap_room_pruning_trusted_candidate_count", 0)),
                            "object_cheap_room_pruning_pruned_count": int(association_floor_profile.get("cheap_room_pruning_pruned_count", 0)),
                            "object_cheap_room_pruning_tail_retained_count": int(association_floor_profile.get("cheap_room_pruning_tail_retained_count", 0)),
                            "object_cheap_room_pruning_same_room_kept_count": int(association_floor_profile.get("cheap_room_pruning_same_room_kept_count", 0)),
                            "object_cheap_room_pruning_untrusted_kept_count": int(association_floor_profile.get("cheap_room_pruning_untrusted_kept_count", 0)),
                        },
                    )

                    all_poses = np.concatenate((all_poses, pose_np), axis=0)
                with runtime_profiler.timer(count, "stage5_pre_assoc_clone_sec"):
                    pre_assoc_all_pred_box = all_pred_box.clone()
                    pre_assoc_all_poses = np.copy(all_poses)
                    pre_assoc_box_manager = copy.deepcopy(box_manager)

                '''
                STEP1: spatial association using 3D OBB NMS
                '''
                spatial_t0 = time.perf_counter()
                with internal_stdout_context():
                    mask, success_mask = Instances3D.spatial_association(
                        all_pred_box,
                        cfg["box_fusion"]["nms_threshold"],
                        box_manager,
                        per_frame_ins.cam_pose,
                        active_candidate_mask=association_floor_mask,
                    )
                runtime_profiler.add_value(count, "spatial_association_sec", time.perf_counter() - spatial_t0)
                with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                    cur_keep_idx = [i - num_before_cat for i in mask if i >= num_before_cat]
                    cur_success_nms = [i - num_before_cat for i in success_mask if i >= num_before_cat]
                    runtime_profiler.add_values(
                        count,
                        {
                            "cur_keep_idx_count": int(len(cur_keep_idx)),
                            "cur_success_nms_count": int(len(cur_success_nms)),
                            "small_object_candidate_count": int(len(cur_keep_idx)),
                        },
                    )
                    runtime_profiler.log_history_scope(
                        count,
                        step_name="spatial_association",
                        scope_label=_object_scope_label(association_floor_profile, "global_retained_history"),
                        candidate_pool_before=int(num_before_cat),
                        candidate_pool_after=int(association_floor_profile.get("retained_after_count", num_before_cat)),
                        retained_history_pool=int(num_before_cat),
                        filter_description=_object_scope_description(
                            "spatial_association",
                            association_floor_profile,
                            "3D OBB NMS over all retained global objects plus current-frame boxes",
                        ),
                        notes=_object_scope_notes(
                            association_floor_profile,
                            extra_note=f"post_nms_keep_count={len(mask)}",
                        ),
                    )
                
 
                keep_idx = np.asarray(mask)
                if len(cur_keep_idx)>0:
                    '''
                    STEP2: correspondence association for small objects
                    '''
                    corr_t0 = time.perf_counter()
                    with internal_stdout_context():
                        all_pred_box,all_poses,keep_idx = Instances3D.correspondence_association(
                            cfg, 
                            box_manager, 
                            cur_keep_idx, 
                            cur_success_nms,
                            pred_instances, 
                            cur_global_pred_box, 
                            all_pred_box,all_poses, 
                            per_frame_ins.cam_pose, 
                            count,
                            mask,
                            sample["sensor_info"].gt.depth.K[-1],
                            all_kf_pose,
                            threshold=cfg['association']['small_threshold'],
                            H=image.shape[0],
                            W=image.shape[1],
                            allowed_global_indices=np.flatnonzero(association_floor_mask[:num_before_cat]),
                            )
                    runtime_profiler.add_value(count, "correspondence_association_sec", time.perf_counter() - corr_t0)
                    with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                        runtime_profiler.log_history_scope(
                            count,
                            step_name="correspondence_association",
                            scope_label=_object_scope_label(association_floor_profile, "global_retained_history"),
                            candidate_pool_before=int(num_before_cat),
                            candidate_pool_after=int(association_floor_profile.get("retained_after_count", num_before_cat)),
                            retained_history_pool=int(num_before_cat),
                            filter_description=_object_scope_description(
                                "correspondence_association",
                                association_floor_profile,
                                "small-object correspondence check is seeded by current-frame keep set but still compares against retained global history",
                            ),
                            notes=_object_scope_notes(
                                association_floor_profile,
                                extra_note=f"small_object_candidate_count={len(cur_keep_idx)}",
                            ),
                        )

                        # update the fusion list based on keep_idx
                        box_manager.update(keep_idx)

                        if runtime_console.is_verbose():
                            runtime_console.info(f"[boxfusion] frame={int(count)} fusion_list={box_manager.fusion_list}")

                        # filter those evident wrong boxes that valid_num=0
                        if cfg['box_fusion']['check_valid']:
                            with internal_stdout_context():
                                all_pred_box = box_manager.check_valid_num(all_pred_box, count, gap)

                    '''
                    multi-view box fusion
                    '''
                    if runtime_console.is_verbose():
                        runtime_console.info(f"[boxfusion] frame={int(count)} num_record={box_manager.num_record}")
                    if cfg['box_fusion']['use']:
                        with runtime_profiler.timer(count, "stage5_candidate_mask_prep_sec"):
                            with runtime_profiler.timer(count, "stage5_boxfusion_mask_prep_sec"):
                                boxfusion_floor_mask, boxfusion_floor_profile = _build_floor_scoped_candidate_mask(
                                    all_pred_box,
                                    retained_count=len(all_pred_box),
                                    room_segmenter=room_segmenter,
                                    readonly_object_room_snapshot=readonly_object_room_snapshot,
                                    box_manager=box_manager,
                                    current_frame_idx=count,
                                    current_pose_xy=RT[:2, 3],
                                    current_pose_matrix=RT,
                                )
                        with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                            runtime_profiler.add_values(
                                count,
                                {
                                    "boxfusion_floor_pruning_active": bool(boxfusion_floor_profile.get("active")),
                                    "boxfusion_floor_pruning_active_floor_id": boxfusion_floor_profile.get("active_floor_id"),
                                    "boxfusion_floor_pruning_reason": boxfusion_floor_profile.get("reason"),
                                    "boxfusion_floor_pruning_retained_before_count": int(boxfusion_floor_profile.get("retained_before_count", 0)),
                                    "boxfusion_floor_pruning_retained_after_count": int(boxfusion_floor_profile.get("retained_after_count", 0)),
                                    "boxfusion_floor_pruning_retained_pruned_count": int(boxfusion_floor_profile.get("retained_pruned_count", 0)),
                                    "boxfusion_floor_pruning_retained_ambiguous_count": int(boxfusion_floor_profile.get("retained_ambiguous_count", 0)),
                                    "boxfusion_within_floor_filter_active": bool(boxfusion_floor_profile.get("within_floor_active")),
                                    "boxfusion_within_floor_same_floor_stable_count": int(boxfusion_floor_profile.get("same_floor_stable_count", 0)),
                                    "boxfusion_within_floor_recent_kept_count": int(boxfusion_floor_profile.get("same_floor_recent_kept_count", 0)),
                                    "boxfusion_within_floor_near_kept_count": int(boxfusion_floor_profile.get("same_floor_near_kept_count", 0)),
                                    "boxfusion_within_floor_recent_or_near_kept_count": int(boxfusion_floor_profile.get("same_floor_recent_or_near_kept_count", 0)),
                                    "boxfusion_within_floor_fallback_kept_count": int(boxfusion_floor_profile.get("same_floor_fallback_kept_count", 0)),
                                    "boxfusion_within_floor_pruned_count": int(boxfusion_floor_profile.get("same_floor_pruned_within_floor_count", 0)),
                                    "boxfusion_within_floor_missing_recency_count": int(boxfusion_floor_profile.get("same_floor_missing_recency_count", 0)),
                                    "boxfusion_within_floor_missing_xy_count": int(boxfusion_floor_profile.get("same_floor_missing_xy_count", 0)),
                                    "boxfusion_cheap_room_pruning_active": bool(boxfusion_floor_profile.get("cheap_room_pruning_active")),
                                    "boxfusion_cheap_room_pruning_reason": boxfusion_floor_profile.get("cheap_room_pruning_reason"),
                                    "boxfusion_cheap_room_pruning_active_room_id": boxfusion_floor_profile.get("active_room_id"),
                                    "boxfusion_cheap_room_pruning_tail_candidate_count": int(boxfusion_floor_profile.get("cheap_room_pruning_tail_candidate_count", 0)),
                                    "boxfusion_cheap_room_pruning_ambiguous_candidate_count": int(boxfusion_floor_profile.get("cheap_room_pruning_ambiguous_candidate_count", 0)),
                                    "boxfusion_cheap_room_pruning_fallback_candidate_count": int(boxfusion_floor_profile.get("cheap_room_pruning_fallback_candidate_count", 0)),
                                    "boxfusion_cheap_room_pruning_trusted_candidate_count": int(boxfusion_floor_profile.get("cheap_room_pruning_trusted_candidate_count", 0)),
                                    "boxfusion_cheap_room_pruning_pruned_count": int(boxfusion_floor_profile.get("cheap_room_pruning_pruned_count", 0)),
                                    "boxfusion_cheap_room_pruning_tail_retained_count": int(boxfusion_floor_profile.get("cheap_room_pruning_tail_retained_count", 0)),
                                    "boxfusion_cheap_room_pruning_same_room_kept_count": int(boxfusion_floor_profile.get("cheap_room_pruning_same_room_kept_count", 0)),
                                    "boxfusion_cheap_room_pruning_untrusted_kept_count": int(boxfusion_floor_profile.get("cheap_room_pruning_untrusted_kept_count", 0)),
                                },
                            )
                        with internal_stdout_context():
                            boxfusion_profile = Box_Fuser.boxfusion(
                                all_pred_box,
                                per_frame_ins,
                                box_manager,
                                active_candidate_mask=boxfusion_floor_mask,
                            )
                        runtime_profiler.add_values(
                            count,
                            {
                                "boxfusion_candidate_scan_sec": float(boxfusion_profile.get("boxfusion_candidate_scan_sec", 0.0)),
                                "boxfusion_optimization_sec": float(boxfusion_profile.get("boxfusion_optimization_sec", 0.0)),
                                "boxfusion_total_sec": float(boxfusion_profile.get("boxfusion_total_sec", 0.0)),
                                "boxfusion_candidates_scanned": int(boxfusion_profile.get("boxfusion_candidates_scanned", 0)),
                                "boxfusion_candidates_eligible": int(boxfusion_profile.get("boxfusion_candidates_eligible", 0)),
                                "boxfusion_candidates_optimized": int(boxfusion_profile.get("boxfusion_candidates_optimized", 0)),
                                "boxfusion_candidates_updated": int(boxfusion_profile.get("boxfusion_candidates_updated", 0)),
                                "fusion_list_count": int(boxfusion_profile.get("fusion_list_count", 0)),
                                "fusion_list_ge3_count": int(boxfusion_profile.get("fusion_list_ge3_count", 0)),
                                "fusion_list_len_mean": float(boxfusion_profile.get("fusion_list_len_mean", 0.0)),
                                "fusion_list_len_p50": float(boxfusion_profile.get("fusion_list_len_p50", 0.0)),
                                "fusion_list_len_p95": float(boxfusion_profile.get("fusion_list_len_p95", 0.0)),
                                "fusion_list_len_max": float(boxfusion_profile.get("fusion_list_len_max", 0.0)),
                            },
                        )
                        with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                            runtime_profiler.log_history_scope(
                                count,
                                step_name="boxfusion",
                                scope_label=_object_scope_label(boxfusion_floor_profile, "near_global_retained_history"),
                                candidate_pool_before=int(boxfusion_floor_profile.get("retained_before_count", len(all_pred_box))),
                                candidate_pool_after=int(boxfusion_profile.get("boxfusion_candidates_optimized", 0)),
                                retained_history_pool=int(boxfusion_floor_profile.get("retained_after_count", 0)),
                                filter_description=_object_scope_description(
                                    "boxfusion",
                                    boxfusion_floor_profile,
                                    "scan every retained object; optimize only fusion lists with >=3 retained observations and not already fused",
                                ),
                                notes=_object_scope_notes(
                                    boxfusion_floor_profile,
                                    extra_note="historical per_frame_ins store is cumulative and not window-pruned",
                                ),
                            )
                
                    #predict the semantic classes of remaining new boxes
                    with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                        cur_keep_idx = [i - num_before_cat for i in keep_idx if i >= num_before_cat]
                        cur_keep_idx_in_all = [i for i in range(keep_idx.shape[0]) if keep_idx[i] >= num_before_cat]

                    if len(cur_keep_idx)>0:
                        with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                            boxes = pred_instances.pred_boxes.cpu().numpy()
                            boxes = boxes[cur_keep_idx]
                            # scale the boxes
                            boxes = scale_boxes(boxes,image.shape[0],image.shape[1],scale=cfg['detection']['scale_box'])
                        # if len(pred_instances)>0:
                        clip_t0 = time.perf_counter()
                        class_results, box_features = text_prompt(boxes, tokenized_text, text_features, image, clip_model, preprocess) #[N_box]
                        runtime_profiler.add_value(count, "clip_classification_sec", time.perf_counter() - clip_t0)
                        with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                            all_pred_box.categories[cur_keep_idx_in_all] = class_results
                            # --- [修改点 2：同步更新增量特征] ---
                            if not hasattr(all_pred_box, 'embeddings'):
                                dim = box_features.shape[-1]
                                all_pred_box.embeddings = torch.zeros((len(all_pred_box), dim))
                            all_pred_box.embeddings[cur_keep_idx_in_all] = box_features.cpu()

                else: # no new box
                    with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                        all_pred_box = all_pred_box[mask]
                        all_poses = all_poses[mask]
                        box_manager.update(keep_idx)
                        if runtime_console.is_verbose():
                            runtime_console.info(
                                f"[boxfusion] frame={int(count)} all new boxes suppressed by NMS | fusion_list={box_manager.fusion_list}"
                            )
                        runtime_profiler.log_history_scope(
                            count,
                            step_name="spatial_association",
                            scope_label=_object_scope_label(association_floor_profile, "global_retained_history"),
                            candidate_pool_before=int(num_before_cat),
                            candidate_pool_after=int(association_floor_profile.get("retained_after_count", num_before_cat)),
                            retained_history_pool=int(num_before_cat),
                            filter_description=_object_scope_description(
                                "spatial_association",
                                association_floor_profile,
                                "all current-frame boxes suppressed by global NMS against retained history",
                            ),
                            notes=_object_scope_notes(
                                association_floor_profile,
                                extra_note="no remaining candidates reached correspondence or boxfusion",
                            ),
                        )
                with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                    runtime_profiler.set_value(count, "all_pred_box_after_association_count", int(len(all_pred_box)))
                if readonly_tail_reference_audit_enabled:
                    with runtime_profiler.timer(count, "stage5_tail_reference_audit_sec"):
                        readonly_tail_drift_metrics = _measure_readonly_tail_assoc_drift(
                            cfg=cfg,
                            box_manager=box_manager,
                            gap=gap,
                            current_frame_idx=count,
                            intrinsic=sample["sensor_info"].gt.depth.K[-1],
                            all_kf_pose=all_kf_pose,
                            image_height=image.shape[0],
                            image_width=image.shape[1],
                            pred_instances=pred_instances,
                            cur_global_pred_box=cur_global_pred_box,
                            pre_assoc_all_pred_box=pre_assoc_all_pred_box,
                            pre_assoc_all_poses=pre_assoc_all_poses,
                            pre_assoc_cam_poses=per_frame_ins.cam_pose,
                            pre_assoc_box_manager=pre_assoc_box_manager,
                            num_before_cat=num_before_cat,
                            candidate_floor_mask=association_floor_mask,
                            reference_floor_mask=association_reference_floor_mask,
                            candidate_result_all_pred_box=all_pred_box,
                            internal_stdout_context=internal_stdout_context,
                        )
                else:
                    runtime_profiler.set_value(count, "stage5_tail_reference_audit_sec", 0.0)
                    readonly_tail_drift_metrics = _readonly_tail_reference_audit_disabled_metrics()
                runtime_profiler.add_values(count, readonly_tail_drift_metrics)
                with runtime_profiler.timer(count, "stage5_bookkeeping_sec"):
                    fusion_lengths = [len(item) for item in box_manager.fusion_list]
                    if fusion_lengths:
                        fusion_lengths_sorted = sorted(fusion_lengths)
                        p50_idx = int(round((len(fusion_lengths_sorted) - 1) * 0.50))
                        p95_idx = int(round((len(fusion_lengths_sorted) - 1) * 0.95))
                        runtime_profiler.add_values(
                            count,
                            {
                                "fusion_list_count": int(len(fusion_lengths)),
                                "fusion_list_ge3_count": int(sum(1 for value in fusion_lengths if value >= 3)),
                                "fusion_list_len_mean": float(sum(fusion_lengths) / len(fusion_lengths)),
                                "fusion_list_len_p50": float(fusion_lengths_sorted[p50_idx]),
                                "fusion_list_len_p95": float(fusion_lengths_sorted[p95_idx]),
                                "fusion_list_len_max": float(max(fusion_lengths)),
                            },
                        )

            if re_vis:
                visualize_online_boxes(all_pred_box, prefix="/device/wide", boxes_3d_name="pred_boxes_3d", log_instances_name="pred_instances",count=count,save=False,show_class=cfg["vis"]["show_class"],show_label=cfg["vis"]["show_label"]) 

            capture_result = maybe_capture_demo_snapshot(
                frame_idx=count,
                timestamp=sample_timestamp,
                image_frame=image,
                pose_matrix=RT,
                segmentation_updated=segmentation_updated,
                write_authoritative_debug_vector_map=stage3_export_deferred_to_stage5,
            )
            runtime_profiler.add_value(count, "snapshot_capture_sec", float(capture_result.get("capture_total_sec", 0.0)))
                
        t_fusion_end = time.time()
        runtime_profiler.set_value(count, "stage5_total_sec", float(t_fusion_end - t_fusion_start))

        if segmentation_updated and not is_keyframe:
            capture_result = maybe_capture_demo_snapshot(
                frame_idx=count,
                timestamp=sample_timestamp,
                image_frame=image,
                pose_matrix=RT,
                segmentation_updated=True,
                write_authoritative_debug_vector_map=False,
            )
            runtime_profiler.add_value(count, "snapshot_capture_sec", float(capture_result.get("capture_total_sec", 0.0)))

        if is_keyframe and not segmentation_updated:
            runtime_profiler.mark_frame(count, sample_kind="keyframe", profiled_frame=True, segmentation_refresh_frame=False)

        runtime_profiler.add_values(
            count,
            {
                "data_preprocess_sec": float(t_data_end - t_loop_start),
                "model_bbox_inference_sec": float(t_infer_end - t_infer_start),
                "rerun_visualization_sec": float(t_rerun_end - t_rerun_start),
                "total_step_sec": float(t_fusion_end - t_loop_start),
                "total_floor_count": int(len(room_segmenter.floor_manager.export_floors())),
                "total_retained_object_count": None if all_pred_box is None else int(len(all_pred_box)),
                "all_pred_box_after_association_count": None if all_pred_box is None else int(len(all_pred_box)),
            },
        )
        if latest_vector_map is not None:
            runtime_profiler.add_values(
                count,
                {
                    "total_room_count": int(len(latest_vector_map.get("rooms", []))),
                    "total_retained_anchor_count": int(len(latest_vector_map.get("anchors", []))),
                    "current_vertical_transition_count": int(
                        len(latest_vector_map.get("vertical_transitions", []))
                    ),
                },
            )
        apply_topology_status(count, RT)

        if demo_recorder is not None:
            demo_recorder.record_frame(
                frame_idx=count,
                timestamp=sample_timestamp,
                image_rgb=image,
                pose=RT,
                trajectory_xy=[(float(pt[0]), float(pt[1])) for pt in traj_xyz],
                segmentation_cycle_idx=segmentation_cycle_idx,
                last_segmentation_frame_idx=last_segmentation_frame_idx,
            )

            demo_recorder.record_runtime_growth(
                frame_idx=count,
                cumulative_processed_frames=count + 1,
                vector_map=latest_vector_map,
                global_box_count=None if all_pred_box is None else len(all_pred_box),
                stage_timings={
                    "data_preprocess_sec": t_data_end - t_loop_start,
                    "model_bbox_inference_sec": t_infer_end - t_infer_start,
                    "topology_room_segmentation_sec": t_seg_end - t_seg_start,
                    "rerun_visualization_sec": t_rerun_end - t_rerun_start,
                    "feature_boxfusion_sec": t_fusion_end - t_fusion_start,
                    "total_step_sec": t_fusion_end - t_loop_start,
                },
                is_keyframe=bool(is_keyframe),
                segmentation_updated=bool(segmentation_updated),
            )

        # --- 打印本帧耗时统计（仅在关键帧打印） ---
        if is_keyframe:
            runtime_console.profiled_frame_timing(
                frame_idx=count,
                data_preprocess_sec=float(t_data_end - t_loop_start),
                inference_sec=float(t_infer_end - t_infer_start),
                stage3_sec=float(t_seg_end - t_seg_start),
                rerun_sec=float(t_rerun_end - t_rerun_start),
                stage5_sec=float(t_fusion_end - t_fusion_start),
            )
        runtime_console.progress(processed_frames=count + 1, total_frames=total_frames)

        count+=1
        
        # 为下一次循环重置起点
        t_loop_start = time.time()
        
    end_time = time.time()
    duration = end_time - start_time
    fps = (count / duration) if duration > 0 else 0.0

    vid_str = video_id[0] if isinstance(video_id, list) else video_id
    runtime_console.bind_sequence(vid_str)

    # save global boxes for evaluation
    if cfg['data']['output_dir'] is not None and cfg["eval"] and all_pred_box is not None:
        class_list = tokenized_text.tolist()
        class_idx = np.array([class_list.index(c) for c in all_pred_box.categories]) #[N]

        boxes_3d = all_pred_box.pred_boxes_3d.corners.cpu().numpy() # [N,8,3]
        if cfg['dataset'] == 'scannet':
            boxes_3d = post_process(boxes_3d)

        if boxes_3d.shape[0]>0:
            save_list = [[(int(0), (boxes_3d[n]), 1.0) for n in range(boxes_3d.shape[0])]] # list of tuples class_idx[n]
            save_box(save_list, os.path.join(cfg['data']['output_dir'], vid_str+"_boxes.pkl"))

    save_path = "./exported_pc/"
    if save_point_cloud:
        os.makedirs(save_path, exist_ok=True)
    pc_save_name = None
    # --- 新增开始：将收集到的点云列表合并并保存为 PLY ---
    if save_point_cloud and len(accumulated_all_pts) > 0 and vid_str is not None:
        all_pts_merged = np.concatenate(accumulated_all_pts, axis=0)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts_merged[:, :3])

        # 如果包含了颜色信息 (xyzrgb 维度为 6)
        if all_pts_merged.shape[1] == 6:
            pcd.colors = o3d.utility.Vector3dVector(all_pts_merged[:, 3:6])

        # 进行体素降采样以减小文件体积，0.02 表示 2cm 的体素大小
        pcd = pcd.voxel_down_sample(voxel_size=0.02)

        pc_save_name = os.path.join(save_path, f"{vid_str}_global_map.ply")

        o3d.io.write_point_cloud(pc_save_name, pcd)
    # --- 新增结束 ---

    final_frame_idx = max(count - 1, 0)
    final_flush_report = room_segmenter.flush_pending_floor_segments(
        all_pred_box=all_pred_box,
        debug_path=debug_room_dir if write_debug_room_artifacts else None,
        count=final_frame_idx,
    )
    if final_flush_report.get("segmented_floor_count", 0) > 0:
        segmentation_cycle_idx += int(final_flush_report["segmented_floor_count"])
        last_segmentation_frame_idx = int(final_frame_idx)
        if runtime_console.is_verbose():
            runtime_console.info(
                f"[finalize] segmented_floor_ids={final_flush_report.get('segmented_floor_ids', [])} "
                f"at frame={int(final_frame_idx)}"
            )

    if room_segmenter.last_room_markers is not None:
        latest_vector_map = room_segmenter.get_vector_map_data(
            all_pred_box,
            count=final_frame_idx,
            save_scene_graph_vis=False,
            scene_graph_vis_dir=str(getattr(demo_recorder, "scene_graph_dir", debug_room_dir)) if demo_recorder is not None else debug_room_dir,
            instrumentation_context="finalization_export",
        )
        log_export_profile(
            final_frame_idx,
            call_context="finalization_export",
            call_origin="run_finalize",
            segmentation_refresh_frame=bool(final_flush_report.get("segmented_floor_count", 0) > 0),
            stage_bucket="final",
        )
        if final_flush_report.get("segmented_floor_count", 0) > 0 and write_debug_room_artifacts:
            with open(os.path.join(debug_room_dir, f"vector_map_{final_frame_idx}_final_flush.json"), 'w') as f:
                json.dump(latest_vector_map, f, indent=2)
    sync_segmentation_run_logs()

    if write_debug_room_artifacts:
        diagnostics_dir = os.path.join(debug_room_dir, "floor_diagnostics")
        room_segmenter.save_floor_diagnostics(
            output_dir=diagnostics_dir,
            vector_map=latest_vector_map,
            total_frames=count,
            last_segmentation_frame_idx=last_segmentation_frame_idx,
            room_seg_interval=room_seg_interval,
            sequence_id=vid_str,
        )

    demo_outputs = None
    runtime_instrumentation_summary = runtime_profiler.finalize()
    runtime_instrumentation_paths = {
        "runtime_instrumentation_dir": str(instrumentation_output_dir),
        "runtime_instrumentation_frame_csv": str(instrumentation_output_dir / "per_profiled_frame.csv"),
        "runtime_instrumentation_export_csv": str(instrumentation_output_dir / "vector_map_export_calls.csv"),
        "runtime_instrumentation_history_csv": str(instrumentation_output_dir / "history_scope.csv"),
        "runtime_instrumentation_segmentation_csv": str(instrumentation_output_dir / "segmentation_runs.csv"),
        "runtime_instrumentation_summary_json": str(instrumentation_output_dir / "summary.json"),
    }
    if demo_recorder is not None:
        if final_flush_report.get("segmented_floor_count", 0) > 0 and last_demo_frame is not None:
            demo_recorder.record_snapshot(
                frame_idx=int(last_demo_frame["frame_idx"]),
                timestamp=float(last_demo_frame["timestamp"]),
                image_rgb=last_demo_frame["image_rgb"],
                pose=last_demo_frame["pose"],
                trajectory_xy=[(float(pt[0]), float(pt[1])) for pt in traj_xyz],
                vector_map=latest_vector_map,
                tracking_report=room_segmenter.last_tracking_report,
                segmentation_updated=True,
                segmentation_cycle_idx=segmentation_cycle_idx,
                last_segmentation_frame_idx=last_segmentation_frame_idx,
                export_profile=dict(room_segmenter.last_export_profile or {}),
            )
        demo_outputs = demo_recorder.finalize(
            {
                "processed_frames": int(count),
                "duration_sec": round(float(duration), 3),
                "average_fps": round(float(fps), 3),
                "sequence_id": vid_str,
                "point_cloud_path": pc_save_name,
                "final_vector_map_path": demo_recorder.latest_vector_map_path,
                "runtime_instrumentation_summary": runtime_instrumentation_summary,
                "runtime_artifact_mode": runtime_artifact_policy.get("mode"),
                "runtime_artifact_policy": runtime_artifact_policy,
                **runtime_instrumentation_paths,
            }
        )

    runtime_console.scene_finish(
        processed_frames=int(count),
        duration_sec=float(duration),
        fps=float(fps),
        output_paths={
            "point_cloud_path": pc_save_name,
            "runtime_instrumentation_dir": str(instrumentation_output_dir),
            "runtime_instrumentation_summary_json": str(instrumentation_output_dir / "summary.json"),
            "final_vector_map_path": None if demo_recorder is None else demo_recorder.latest_vector_map_path,
        },
    )

    return {
        "processed_frames": int(count),
        "duration_sec": float(duration),
        "average_fps": float(fps),
        "sequence_id": vid_str,
        "point_cloud_path": pc_save_name,
        "demo_outputs": demo_outputs,
        "runtime_instrumentation_summary": runtime_instrumentation_summary,
        "runtime_artifact_mode": runtime_artifact_policy.get("mode"),
        "runtime_artifact_policy": runtime_artifact_policy,
        **runtime_instrumentation_paths,
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("dataset_path", help="Path to the directory containing the .tar files, the full path to a single tar file (recommended), or a path to a txt file containing HTTP links. Using the value \"stream\" will attempt to stream from your device using the NeRFCapture app")
    parser.add_argument("--model-path", help="Path to the model to load")
    parser.add_argument("--config", default=None, type=str, help="config_path")
    parser.add_argument("--clip_path", default='./models/open_clip_pytorch_model.bin', type=str, help="Path to the CLIP model")
    parser.add_argument("--seq", default='None', type=str, help="config_path")
    parser.add_argument("--class_txt", default='./data/panoptic_categories_nomerge.txt', type=str, help="config_path")
    parser.add_argument("--every-nth-frame", default=None, type=int, help="Load every `n` frames")
    parser.add_argument("--viz-on-gt-points", default=True, action="store_true", help="Backproject the GT depth to form a point cloud in order to visualize the predictions")
    parser.add_argument("--device", default="cpu", help="Which device to push the model to (cpu, mps, cuda)")
    parser.add_argument("--video-ids", nargs="+", help="Subset of videos to execute on. By default, all. Ignored if a tar file is explicitly given or in stream mode.")
    parser.add_argument("--quiet", action="store_true", help="Keep stdout to warnings plus final summary lines")
    parser.add_argument("--log-level", choices=["summary", "verbose"], default="summary", help="Console verbosity for runtime progress")
    parser.add_argument("--runtime-print-interval", default=50, type=int, help="Progress print interval in processed frames; set 0 to disable periodic progress")
    parser.add_argument("--no-per-profiled-frame-stdout", action="store_true", help="Suppress per-profiled-frame timing lines even in verbose mode")
    parser.add_argument(
        "--enable-readonly-tail-reference-audit",
        action="store_true",
        help="Run the stage-5 shadow-reference readonly-tail audit. Disabled by default for normal runtime and benchmark runs.",
    )

    args = parser.parse_args()
    print("Command Line Args:", args)

    dataset_path = args.dataset_path
    use_cache = False
    
    if dataset_path.lower() in ["scannet", "ca1m", 'online', 'hm3d']:
        if not os.path.exists(args.config):
            raise ValueError("Missing config path")
        else:
            with open(args.config, 'r') as  f:
                cfg = yaml.full_load(f)
        cfg["runtime_logging"] = {
            "quiet": bool(args.quiet),
            "log_level": str(args.log_level),
            "runtime_print_interval": args.runtime_print_interval,
            "per_profiled_frame_stdout": bool(str(args.log_level) == "verbose" and not args.no_per_profiled_frame_stdout),
            "enable_readonly_tail_reference_audit": bool(args.enable_readonly_tail_reference_audit),
        }
        # load the customized sequence if given by the user
        if args.seq is not None:
            if dataset_path.lower()=='ca1m':
                if 'example' in cfg['data']['datadir']:
                    current_file_path = os.path.abspath(__file__)
                    current_dir = os.path.dirname(current_file_path)
                    cfg['data']['datadir'] = os.path.join(current_dir, cfg['data']['datadir'])

                else:
                    new_datadir = os.path.join(os.path.dirname(os.path.dirname(cfg['data']['datadir'])),  args.seq+'/')
                    cfg['data']['datadir'] = new_datadir
            
            # 修改 2：专门为 hm3d 增加路径拼接逻辑
            elif dataset_path.lower() == 'hm3d':
                # 因为你的 HM3D 结构是 .../val/00824-Dd4bFSTQ8gi，没有 frames 子目录
                # 所以我们只需拿到上级目录 (.../val)，然后拼上 seq 名称
                new_datadir = os.path.join(os.path.dirname(cfg['data']['datadir']), args.seq)
                cfg['data']['datadir'] = new_datadir
                
            else:
                new_datadir = os.path.join(os.path.dirname(os.path.dirname(cfg['data']['datadir'])),  args.seq+'/frames/')
                cfg['data']['datadir'] = new_datadir
                
            # eval only
            if os.path.exists(os.path.join(cfg['data']['output_dir'],args.seq+"_boxes.pkl")) and cfg["eval"]:
                print("Results for boxes already exist, skip evaluation")
                sys.exit(0)
        
        dataset = get_dataset(cfg)

    assert args.model_path is not None
    checkpoint = torch.load(args.model_path, map_location=args.device or "cpu")["model"]
    backbone_embedding_dimension = checkpoint["backbone.0.patch_embed.proj.weight"].shape[0]
        
    is_depth_model = True 
    model = make_cubify_transformer(dimension=backbone_embedding_dimension, depth_model=is_depth_model).eval()
    model.load_state_dict(checkpoint)

    dataset.load_arkit_depth = True
    if args.every_nth_frame is not None:
        dataset = itertools.islice(dataset, 0, None, args.every_nth_frame)

    augmentor = Augmentor(("wide/image", "wide/depth"))
    preprocessor = Preprocessor()
    
    # if args.device is not None:
    #     model = model.to(args.device)
    #     clip_model, preprocess = load_clip(args.clip_path)
    #     text_class = np.genfromtxt(args.class_txt, delimiter='\n', dtype=str) 
    #     text_features = torch.load('./data/class_features.pt').cuda()
    # --- 修改后 ---
    if args.device is not None:
        model = model.to(args.device)
        
        # ------------------- 修改开始 -------------------
        print("正在加载本地 CLIP 模型...")
        # 1. 设置为你下载权重的绝对路径
        local_model_path = '/home/aurora/workspace1/BoxFusion/models/ViT-B-32/open_clip_pytorch_model.bin'
        
        # 2. 直接使用 open_clip 加载本地权重 (注意：必须与 gen_features.py 中的 model_name 一致)
        clip_model, _, preprocess = open_clip.create_model_and_transforms(
            model_name='ViT-B-32', 
            pretrained=local_model_path
        )
        clip_model = clip_model.to(args.device).eval()
        
        text_class = np.genfromtxt(args.class_txt, delimiter='\n', dtype=str) 
        
        # 3. 加载你刚刚生成的新特征文件 (class_features_small.pt)
        print("正在加载新生成的文本特征...")
        text_features = torch.load('./data/class_features_small.pt').to(args.device)
        # ------------------- 修改结束 -------------------

    run(
        cfg,
        model,
        dataset,
        clip_model,
        preprocess,
        text_class,
        text_features,
        augmentor,
        preprocessor,
        score_thresh=cfg['detection']['score_thresh'],
        viz_on_gt_points=args.viz_on_gt_points,
        gap=cfg["data"]["gap"],
        re_vis=cfg['vis']['rerun'],
        runtime_console_config=dict(cfg.get("runtime_logging") or {}),
    )
