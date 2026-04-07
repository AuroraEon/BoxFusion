import argparse
import json
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _bool_arg(value: str) -> bool:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def _array_equal(left: Optional[np.ndarray], right: Optional[np.ndarray]) -> bool:
    if left is None and right is None:
        return True
    if left is None or right is None:
        return False
    return bool(np.array_equal(left, right))


def _build_snapshot(
    segmenter: Any,
    all_pred_box: Any,
    vector_map: Dict[str, Any],
) -> Dict[str, Any]:
    prepared = segmenter._prepare_object_export_state(all_pred_box)
    object_export_by_id = {
        int(obj["id"]): dict(obj)
        for obj in list(vector_map.get("objects") or [])
    }
    instance_ids_value = prepared.get("instance_ids")
    box_tensors_value = prepared.get("box_tensors")
    categories_value = prepared.get("categories")
    scores_value = prepared.get("scores")
    semantic_confidences_value = prepared.get("semantic_confidences")
    association_confidences_value = prepared.get("association_confidences")
    semantic_gaps_value = prepared.get("semantic_gaps")
    view_qualities_value = prepared.get("view_qualities")

    instance_ids = np.asarray(
        np.zeros((0,), dtype=np.int64) if instance_ids_value is None else instance_ids_value,
        dtype=np.int64,
    )
    box_tensors = np.asarray(
        np.zeros((0, 7), dtype=np.float32) if box_tensors_value is None else box_tensors_value,
        dtype=np.float32,
    )
    categories = np.asarray(
        np.asarray([], dtype=object) if categories_value is None else categories_value,
        dtype=object,
    )
    scores = np.asarray(np.zeros((0,), dtype=np.float32) if scores_value is None else scores_value, dtype=np.float32)
    semantic_confidences = np.asarray(
        np.zeros((0,), dtype=np.float32) if semantic_confidences_value is None else semantic_confidences_value,
        dtype=np.float32,
    )
    association_confidences = np.asarray(
        np.zeros((0,), dtype=np.float32)
        if association_confidences_value is None
        else association_confidences_value,
        dtype=np.float32,
    )
    semantic_gaps = np.asarray(
        np.zeros((0,), dtype=np.float32) if semantic_gaps_value is None else semantic_gaps_value,
        dtype=np.float32,
    )
    view_qualities = np.asarray(
        np.zeros((0,), dtype=np.float32) if view_qualities_value is None else view_qualities_value,
        dtype=np.float32,
    )
    embeddings = prepared.get("embeddings")
    per_object_signatures = list(prepared.get("per_object_signatures") or [])

    objects: Dict[int, Dict[str, Any]] = {}
    for idx in range(len(instance_ids)):
        object_id = int(instance_ids[idx])
        export_obj = dict(object_export_by_id.get(object_id) or {})
        room_id = export_obj.get("room_id")
        floor_id = export_obj.get("floor_id")
        objects[object_id] = {
            "signature": None if idx >= len(per_object_signatures) else str(per_object_signatures[idx]),
            "box_tensor": np.asarray(box_tensors[idx], dtype=np.float32).copy(),
            "category": str(categories[idx]),
            "scores": np.asarray(
                [
                    scores[idx],
                    semantic_confidences[idx],
                    association_confidences[idx],
                    semantic_gaps[idx],
                    view_qualities[idx],
                ],
                dtype=np.float32,
            ),
            "embedding": None if embeddings is None else np.asarray(embeddings[idx], dtype=np.float32).copy(),
            "room_id": None if room_id in (None, "") else str(room_id),
            "floor_id": None if floor_id in (None, "") else str(floor_id),
        }

    return {
        "full_signature": str(prepared.get("full_signature") or ""),
        "room_ids": sorted(
            str(room.get("room_id"))
            for room in list(vector_map.get("rooms") or [])
            if room.get("room_id") is not None
        ),
        "room_count": int(len(vector_map.get("rooms") or [])),
        "object_count": int(len(vector_map.get("objects") or [])),
        "objects": objects,
    }


def _compare_snapshots(stage3: Dict[str, Any], stage5: Dict[str, Any]) -> Dict[str, Any]:
    stage3_objects = dict(stage3.get("objects") or {})
    stage5_objects = dict(stage5.get("objects") or {})
    stage3_ids = set(stage3_objects.keys())
    stage5_ids = set(stage5_objects.keys())
    common_ids = sorted(stage3_ids & stage5_ids)

    new_ids = sorted(stage5_ids - stage3_ids)
    disappeared_ids = sorted(stage3_ids - stage5_ids)
    signature_changed_ids: List[int] = []
    geometry_changed_ids: List[int] = []
    category_changed_ids: List[int] = []
    scalar_state_changed_ids: List[int] = []
    embedding_changed_ids: List[int] = []
    room_assignment_changed_ids: List[int] = []
    floor_assignment_changed_ids: List[int] = []

    for object_id in common_ids:
        left = stage3_objects[object_id]
        right = stage5_objects[object_id]
        if left.get("signature") != right.get("signature"):
            signature_changed_ids.append(int(object_id))
        if not _array_equal(left.get("box_tensor"), right.get("box_tensor")):
            geometry_changed_ids.append(int(object_id))
        if str(left.get("category")) != str(right.get("category")):
            category_changed_ids.append(int(object_id))
        if not _array_equal(left.get("scores"), right.get("scores")):
            scalar_state_changed_ids.append(int(object_id))
        if not _array_equal(left.get("embedding"), right.get("embedding")):
            embedding_changed_ids.append(int(object_id))
        if left.get("room_id") != right.get("room_id"):
            room_assignment_changed_ids.append(int(object_id))
        if left.get("floor_id") != right.get("floor_id"):
            floor_assignment_changed_ids.append(int(object_id))

    changed_ids = sorted(set(new_ids) | set(disappeared_ids) | set(signature_changed_ids))
    changed_room_ids = sorted(
        {
            room_id
            for object_id in changed_ids
            for room_id in [
                (stage3_objects.get(object_id) or {}).get("room_id"),
                (stage5_objects.get(object_id) or {}).get("room_id"),
            ]
            if room_id not in (None, "")
        }
    )
    stage5_room_ids = set(stage5.get("room_ids") or [])
    changed_export_room_ids = sorted(room_id for room_id in changed_room_ids if room_id in stage5_room_ids)
    unassigned_changed_object_ids = sorted(
        object_id
        for object_id in changed_ids
        if ((stage3_objects.get(object_id) or {}).get("room_id") in (None, ""))
        and ((stage5_objects.get(object_id) or {}).get("room_id") in (None, ""))
    )

    return {
        "changed_ids": changed_ids,
        "changed_object_count": int(len(changed_ids)),
        "new_object_ids": new_ids,
        "new_object_count": int(len(new_ids)),
        "disappeared_object_ids": disappeared_ids,
        "disappeared_object_count": int(len(disappeared_ids)),
        "common_signature_changed_ids": signature_changed_ids,
        "common_signature_changed_count": int(len(signature_changed_ids)),
        "geometry_changed_ids": geometry_changed_ids,
        "geometry_changed_count": int(len(geometry_changed_ids)),
        "category_changed_ids": category_changed_ids,
        "category_changed_count": int(len(category_changed_ids)),
        "scalar_state_changed_ids": scalar_state_changed_ids,
        "scalar_state_changed_count": int(len(scalar_state_changed_ids)),
        "embedding_changed_ids": embedding_changed_ids,
        "embedding_changed_count": int(len(embedding_changed_ids)),
        "room_assignment_changed_ids": room_assignment_changed_ids,
        "room_assignment_changed_count": int(len(room_assignment_changed_ids)),
        "floor_assignment_changed_ids": floor_assignment_changed_ids,
        "floor_assignment_changed_count": int(len(floor_assignment_changed_ids)),
        "changed_room_ids": changed_room_ids,
        "changed_room_count": int(len(changed_room_ids)),
        "changed_export_room_ids": changed_export_room_ids,
        "changed_export_room_count": int(len(changed_export_room_ids)),
        "unassigned_changed_object_ids": unassigned_changed_object_ids,
        "unassigned_changed_object_count": int(len(unassigned_changed_object_ids)),
        "room_set_changed": sorted(set(stage3.get("room_ids") or []) ^ set(stage5.get("room_ids") or [])),
        "stage3_room_count": int(stage3.get("room_count", 0)),
        "stage5_room_count": int(stage5.get("room_count", 0)),
        "stage3_object_count": int(stage3.get("object_count", 0)),
        "stage5_object_count": int(stage5.get("object_count", 0)),
        "object_count_delta": int(stage5.get("object_count", 0) - stage3.get("object_count", 0)),
    }


def _percentile(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def _summary_stats(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"count": 0, "min": None, "mean": None, "median": None, "p90": None, "max": None}
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(values)),
        "min": float(np.min(arr)),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": _percentile(values, 90.0),
        "max": float(np.max(arr)),
    }


def _load_per_frame_metrics(path: str) -> Dict[int, Dict[str, Any]]:
    import csv

    rows: Dict[int, Dict[str, Any]] = {}
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_idx = int(row["frame_idx"])
            rows[frame_idx] = dict(row)
    return rows


def _build_sequence_summary(sequence_id: str, frames: List[Dict[str, Any]]) -> Dict[str, Any]:
    changed_counts = [frame["changed_object_count"] for frame in frames]
    rebuilt_counts = [frame["object_export_rebuilt_count"] for frame in frames]
    reused_counts = [frame["object_export_reused_count"] for frame in frames]
    changed_ratios = [frame["changed_object_ratio"] for frame in frames]
    rebuilt_ratios = [frame["rebuilt_object_ratio"] for frame in frames]
    changed_room_counts = [frame["changed_room_count"] for frame in frames]
    changed_export_room_counts = [frame["changed_export_room_count"] for frame in frames]
    unchanged_room_counts = [frame["unchanged_room_count"] for frame in frames]
    unchanged_room_ratios = [frame["unchanged_room_ratio"] for frame in frames]

    one_room = sum(1 for value in changed_room_counts if value == 1)
    few_room = sum(1 for value in changed_room_counts if value <= 2)
    unchanged_majority = sum(1 for value in unchanged_room_ratios if value > 0.5)

    aggregate_counters = Counter()
    frames_with = Counter()
    for frame in frames:
        for key in [
            "new_object_count",
            "disappeared_object_count",
            "geometry_changed_count",
            "category_changed_count",
            "scalar_state_changed_count",
            "embedding_changed_count",
            "room_assignment_changed_count",
            "floor_assignment_changed_count",
        ]:
            aggregate_counters[key] += int(frame.get(key, 0))
            if int(frame.get(key, 0)) > 0:
                frames_with[key] += 1

    return {
        "sequence_id": sequence_id,
        "fallback_frame_count": int(len(frames)),
        "changed_object_count_stats": _summary_stats(changed_counts),
        "rebuilt_object_count_stats": _summary_stats(rebuilt_counts),
        "reused_object_count_stats": _summary_stats(reused_counts),
        "changed_object_ratio_stats": _summary_stats(changed_ratios),
        "rebuilt_object_ratio_stats": _summary_stats(rebuilt_ratios),
        "changed_room_count_stats": _summary_stats(changed_room_counts),
        "changed_export_room_count_stats": _summary_stats(changed_export_room_counts),
        "unchanged_room_count_stats": _summary_stats(unchanged_room_counts),
        "unchanged_room_ratio_stats": _summary_stats(unchanged_room_ratios),
        "one_room_changed_frames": int(one_room),
        "few_room_changed_frames": int(few_room),
        "unchanged_room_majority_frames": int(unchanged_majority),
        "change_component_object_totals": dict(aggregate_counters),
        "change_component_frame_totals": dict(frames_with),
    }


def _run_analysis(args: argparse.Namespace) -> Dict[str, Any]:
    import open_clip
    import torch

    import stage_a_demo as stage_a_demo_module
    from boxfusion.cubify_transformer import make_cubify_transformer
    from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
    from boxfusion.preprocessor import Augmentor, Preprocessor

    checkpoint = torch.load(args.model_path, map_location=args.device or "cpu")["model"]
    backbone_embedding_dimension = checkpoint["backbone.0.patch_embed.proj.weight"].shape[0]
    model = make_cubify_transformer(dimension=backbone_embedding_dimension, depth_model=True).eval()
    model.load_state_dict(checkpoint)
    model = model.to(args.device)

    clip_path = stage_a_demo_module._resolve_clip_path(args.clip_path)
    text_features_path = stage_a_demo_module._resolve_text_features_path(args.text_features)
    clip_model, _, preprocess = open_clip.create_model_and_transforms(
        model_name=args.clip_model_name,
        pretrained=clip_path,
    )
    clip_model = clip_model.to(args.device).eval()

    text_class = np.genfromtxt(args.class_txt, delimiter="\n", dtype=str)
    text_features = torch.load(text_features_path, map_location=args.device).to(args.device)

    augmentor = Augmentor(("wide/image", "wide/depth"))
    preprocessor = Preprocessor()

    current_sequence: Dict[str, Optional[str]] = {"id": None}
    stage3_snapshots: Dict[str, Dict[int, Dict[str, Any]]] = defaultdict(dict)
    fallback_frames: Dict[str, List[Dict[str, Any]]] = defaultdict(list)

    original = FloorAwareRoomSegmenter.get_vector_map_data

    def wrapped_get_vector_map_data(self, *wrapper_args, **wrapper_kwargs):
        instrumentation_context = str(wrapper_kwargs.get("instrumentation_context", "unspecified"))
        count = wrapper_kwargs.get("count")
        vector_map = original(self, *wrapper_args, **wrapper_kwargs)
        sequence_id = current_sequence["id"]
        if sequence_id is None or count is None:
            return vector_map
        frame_idx = int(count)
        snapshot = _build_snapshot(self, wrapper_args[0] if wrapper_args else None, vector_map)
        profile = dict(self.last_export_profile or {})
        if instrumentation_context == "stage3_post_segmentation_refresh":
            stage3_snapshots[sequence_id][frame_idx] = snapshot
            return vector_map

        if instrumentation_context != "stage5_snapshot_capture":
            return vector_map
        if not bool(profile.get("build_executed")):
            return vector_map
        if str(profile.get("same_frame_reuse_blocker")) != "object_state_signature_mismatch":
            return vector_map
        if not bool(profile.get("same_frame_cache_frame_match")):
            return vector_map
        if not bool(profile.get("same_frame_segmentation_token_match")):
            return vector_map

        stage3_snapshot = stage3_snapshots[sequence_id].get(frame_idx)
        if stage3_snapshot is None:
            return vector_map

        delta = _compare_snapshots(stage3_snapshot, snapshot)
        if int(profile.get("object_export_rebuilt_count", 0)) > int(delta["changed_object_count"]):
            raise RuntimeError(
                f"{sequence_id} frame {frame_idx}: rebuilt count exceeds changed object count "
                f"({profile.get('object_export_rebuilt_count')} > {delta['changed_object_count']})"
            )

        stage5_room_count = int(delta["stage5_room_count"])
        changed_room_count = int(delta["changed_export_room_count"])
        frame_record = {
            "sequence_id": sequence_id,
            "frame_idx": frame_idx,
            "object_export_reused_count": int(profile.get("object_export_reused_count", 0)),
            "object_export_rebuilt_count": int(profile.get("object_export_rebuilt_count", 0)),
            "stage5_exported_object_count": int(profile.get("object_count", 0)),
            "stage5_exported_room_count": stage5_room_count,
            "changed_object_ratio": (
                float(delta["changed_object_count"]) / float(profile.get("object_count", 0))
                if int(profile.get("object_count", 0)) > 0
                else 0.0
            ),
            "rebuilt_object_ratio": (
                float(profile.get("object_export_rebuilt_count", 0)) / float(profile.get("object_count", 0))
                if int(profile.get("object_count", 0)) > 0
                else 0.0
            ),
            "changed_room_count": int(delta["changed_room_count"]),
            "changed_export_room_count": changed_room_count,
            "unchanged_room_count": int(max(stage5_room_count - changed_room_count, 0)),
            "unchanged_room_ratio": (
                float(max(stage5_room_count - changed_room_count, 0)) / float(stage5_room_count)
                if stage5_room_count > 0
                else 0.0
            ),
            **delta,
        }
        fallback_frames[sequence_id].append(frame_record)
        return vector_map

    FloorAwareRoomSegmenter.get_vector_map_data = wrapped_get_vector_map_data
    try:
        for sequence_id in args.sequence_ids:
            current_sequence["id"] = str(sequence_id)
            sequence_args = SimpleNamespace(
                dataset_path=args.dataset_path,
                model_path=args.model_path,
                config=args.config,
                seq=sequence_id,
                seqs=None,
                class_txt=args.class_txt,
                clip_path=args.clip_path,
                clip_model_name=args.clip_model_name,
                text_features=args.text_features,
                device=args.device,
                viz_on_gt_points=args.viz_on_gt_points,
                every_nth_frame=args.every_nth_frame,
                max_frames=args.max_frames,
                keyframe_gap=args.keyframe_gap,
                room_seg_interval=args.room_seg_interval,
                capture_stride=args.capture_stride,
                runtime_profile_interval=args.runtime_profile_interval,
                quiet=args.quiet,
                log_level=args.log_level,
                runtime_print_interval=args.runtime_print_interval,
                no_per_profiled_frame_stdout=args.no_per_profiled_frame_stdout,
                core_only=args.core_only,
                full_rgb_replay=args.full_rgb_replay,
                output_root=args.output_root,
                video_fps=args.video_fps,
                canvas_width=args.canvas_width,
                canvas_height=args.canvas_height,
                spotlight_count=args.spotlight_count,
                aggregate_name=args.aggregate_name,
                enable_rerun=args.enable_rerun,
                save_scene_graph_vis=args.save_scene_graph_vis,
            )
            stage_a_demo_module._run_single_sequence(
                sequence_args,
                model,
                clip_model,
                preprocess,
                text_class,
                text_features,
                augmentor,
                preprocessor,
                sequence_id,
            )
    finally:
        FloorAwareRoomSegmenter.get_vector_map_data = original

    sequence_summaries = {}
    for sequence_id in args.sequence_ids:
        sequence_summaries[sequence_id] = _build_sequence_summary(
            sequence_id=str(sequence_id),
            frames=sorted(fallback_frames.get(str(sequence_id), []), key=lambda item: int(item["frame_idx"])),
        )

    per_frame_csv_lookup = {
        sequence_id: _load_per_frame_metrics(
            os.path.join(args.output_root, str(sequence_id), "logs", "runtime_instrumentation", "per_profiled_frame.csv")
        )
        for sequence_id in args.sequence_ids
    }
    for sequence_id, frames in fallback_frames.items():
        for frame in frames:
            per_frame = per_frame_csv_lookup.get(sequence_id, {}).get(int(frame["frame_idx"]), {})
            frame["boxfusion_candidates_updated"] = int(per_frame.get("boxfusion_candidates_updated") or 0)
            frame["boxfusion_candidates_optimized"] = int(per_frame.get("boxfusion_candidates_optimized") or 0)
            frame["cur_keep_idx_count"] = int(per_frame.get("cur_keep_idx_count") or 0)
            frame["cur_success_nms_count"] = int(per_frame.get("cur_success_nms_count") or 0)
            frame["small_object_candidate_count"] = int(per_frame.get("small_object_candidate_count") or 0)
            frame["vector_map_export_same_frame_reuse_blockers"] = str(
                per_frame.get("vector_map_export_same_frame_reuse_blockers") or ""
            )

    combined_frames = sorted(
        [frame for frames in fallback_frames.values() for frame in frames],
        key=lambda item: (str(item["sequence_id"]), int(item["frame_idx"])),
    )
    combined_summary = _build_sequence_summary("combined", combined_frames)

    return {
        "sequences": {sequence_id: sorted(frames, key=lambda item: int(item["frame_idx"])) for sequence_id, frames in fallback_frames.items()},
        "sequence_summaries": sequence_summaries,
        "combined_summary": combined_summary,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze same-frame stage3->stage5 object deltas on fallback rebuilds.")
    parser.add_argument("dataset_path", choices=["CA1M", "scannet", "online", "hm3d"])
    parser.add_argument("--model-path", required=True, help="Path to the BoxFusion / Cubify checkpoint")
    parser.add_argument("--config", required=True, type=str, help="Config path")
    parser.add_argument("--sequence-ids", nargs="+", required=True, help="Scene sequence ids to replay")
    parser.add_argument("--class-txt", default="./data/panoptic_categories_nomerge.txt", type=str)
    parser.add_argument("--clip-path", default=None, type=str)
    parser.add_argument("--clip-model-name", default="ViT-B-32", type=str)
    parser.add_argument("--text-features", default=None, type=str)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--viz-on-gt-points", default=True, type=_bool_arg)
    parser.add_argument("--every-nth-frame", default=None, type=int)
    parser.add_argument("--max-frames", default=None, type=int)
    parser.add_argument("--keyframe-gap", default=None, type=int)
    parser.add_argument("--room-seg-interval", default=100, type=int)
    parser.add_argument("--capture-stride", default=None, type=int)
    parser.add_argument("--runtime-profile-interval", default=None, type=int)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--log-level", choices=["summary", "verbose"], default="summary")
    parser.add_argument("--runtime-print-interval", default=50, type=int)
    parser.add_argument("--no-per-profiled-frame-stdout", action="store_true")
    parser.add_argument("--core-only", action="store_true")
    parser.add_argument("--full-rgb-replay", action="store_true")
    parser.add_argument("--output-root", required=True, type=str)
    parser.add_argument("--video-fps", default=12, type=int)
    parser.add_argument("--canvas-width", default=1600, type=int)
    parser.add_argument("--canvas-height", default=900, type=int)
    parser.add_argument("--spotlight-count", default=0, type=int)
    parser.add_argument("--aggregate-name", default="stage_a_multi_sequence", type=str)
    parser.add_argument("--enable-rerun", action="store_true")
    parser.add_argument("--save-scene-graph-vis", action="store_true")
    parser.add_argument("--output-json", required=True, type=str)
    args = parser.parse_args()

    payload = _run_analysis(args)
    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


if __name__ == "__main__":
    main()
