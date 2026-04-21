import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from boxfusion.runtime_artifact_policy import (
    RUNTIME_ARTIFACT_MODE_BENCHMARK,
    RUNTIME_ARTIFACT_MODES,
    RuntimeArtifactPolicy,
    resolve_runtime_artifact_policy,
)

def _load_config(dataset_name: str, config_path: str, seq: Optional[str]) -> dict:
    if not os.path.exists(config_path):
        raise ValueError(f"Missing config path: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.full_load(f)

    dataset_name = dataset_name.lower()
    if seq is not None:
        if dataset_name == "ca1m":
            if "example" in cfg["data"]["datadir"]:
                current_dir = os.path.dirname(os.path.abspath(__file__))
                cfg["data"]["datadir"] = os.path.join(current_dir, cfg["data"]["datadir"])
            else:
                cfg["data"]["datadir"] = os.path.join(
                    os.path.dirname(os.path.dirname(cfg["data"]["datadir"])),
                    seq + "/",
                )
        elif dataset_name == "hm3d":
            cfg["data"]["datadir"] = os.path.join(os.path.dirname(cfg["data"]["datadir"]), seq)
        else:
            cfg["data"]["datadir"] = os.path.join(
                os.path.dirname(os.path.dirname(cfg["data"]["datadir"])),
                seq + "/frames/",
            )
    return cfg


def _resolve_clip_path(cli_value: Optional[str]) -> str:
    candidates = [
        cli_value,
        "./models/ViT-B-32/open_clip_pytorch_model.bin",
        "./models/open_clip_pytorch_model.bin",
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    raise FileNotFoundError("Could not find a CLIP checkpoint. Pass --clip-path explicitly.")


def _resolve_text_features_path(cli_value: Optional[str]) -> str:
    candidates = [
        cli_value,
        "./data/class_features_small.pt",
        "./data/class_features.pt",
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    raise FileNotFoundError("Could not find a text-feature file. Pass --text-features explicitly.")


def _load_text_features_tensor(torch_module, features_path: str, device: str):
    try:
        tensor = torch_module.load(features_path, map_location=device, weights_only=True)
    except TypeError:
        tensor = torch_module.load(features_path, map_location=device)
    return tensor


def _resolve_autogen_text_feature_path(cli_value: Optional[str], clip_model_name: str) -> str:
    if cli_value:
        return cli_value
    if str(clip_model_name).lower() == "vit-b-32":
        return "./data/class_features_small.pt"
    safe_name = str(clip_model_name).replace("/", "_").replace(" ", "_")
    return f"./data/class_features_{safe_name}.pt"


def _load_or_build_text_features(
    *,
    torch_module,
    open_clip_module,
    clip_model,
    clip_model_name: str,
    text_class,
    text_features_path: str,
    requested_text_features_path: Optional[str],
    device: str,
):
    text_features = _load_text_features_tensor(torch_module, text_features_path, device)
    if not hasattr(text_features, "shape"):
        raise TypeError(f"Expected a tensor in text feature file, got {type(text_features)}")
    if len(text_features.shape) != 2:
        raise ValueError(f"Expected [num_classes, dim] text features, got shape={tuple(text_features.shape)}")

    clip_embed_dim = None
    if hasattr(clip_model, "visual") and hasattr(clip_model.visual, "output_dim"):
        clip_embed_dim = int(clip_model.visual.output_dim)
    elif hasattr(clip_model, "text_projection") and clip_model.text_projection is not None:
        clip_embed_dim = int(clip_model.text_projection.shape[-1])
    else:
        clip_embed_dim = int(clip_model.encode_image(torch_module.zeros((1, 3, 224, 224), device=device)).shape[-1])
    text_embed_dim = int(text_features.shape[-1])

    if text_embed_dim == clip_embed_dim:
        return text_features.to(device), text_features_path, False

    print(
        "[warn] CLIP/text feature dim mismatch detected: "
        f"clip={clip_embed_dim}, text={text_embed_dim}; rebuilding text features for {clip_model_name}."
    )
    tokenizer = open_clip_module.get_tokenizer(clip_model_name)
    text_labels = text_class.tolist() if hasattr(text_class, "tolist") else list(text_class)
    tokenized = tokenizer(text_labels).to(device)
    with torch_module.no_grad():
        rebuilt = clip_model.encode_text(tokenized)
        rebuilt = rebuilt / rebuilt.norm(dim=-1, keepdim=True)

    save_path = _resolve_autogen_text_feature_path(requested_text_features_path, clip_model_name)
    save_dir = os.path.dirname(save_path)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
    torch_module.save(rebuilt.detach().cpu(), save_path)
    print(f"[info] Rebuilt text features saved to: {save_path}")
    return rebuilt.to(device), save_path, True


def _infer_sequence_id(cfg: dict, seq: Optional[str]) -> str:
    if seq:
        return seq
    datadir = Path(cfg["data"]["datadir"])
    if datadir.name == "frames" and datadir.parent.name:
        return datadir.parent.name
    return datadir.name.rstrip("/")


def _infer_dataset_root_for_manifest(cfg: dict, sequence_id: str) -> Optional[str]:
    datadir = Path(cfg["data"]["datadir"])
    if datadir.name == "frames" and datadir.parent.name == sequence_id:
        return str(datadir.parent.parent)
    if datadir.name == sequence_id:
        return str(datadir.parent)
    return None


def build_runtime_artifact_policy_from_args(args: argparse.Namespace) -> RuntimeArtifactPolicy:
    return resolve_runtime_artifact_policy(
        mode=getattr(args, "runtime_artifact_mode", RUNTIME_ARTIFACT_MODE_BENCHMARK),
        service_mode=bool(getattr(args, "service_mode", False)),
        core_only=bool(getattr(args, "core_only", False)),
        requested_gt_visualization=bool(getattr(args, "viz_on_gt_points", True)),
        requested_scene_graph_vis=bool(getattr(args, "save_scene_graph_vis", False)),
        requested_full_rgb_replay=bool(getattr(args, "full_rgb_replay", False)),
        requested_readonly_tail_reference_audit=bool(getattr(args, "enable_readonly_tail_reference_audit", False)),
    )


def apply_ablation_overrides(cfg: dict, args: argparse.Namespace) -> None:
    box_fusion_mode = str(getattr(args, "box_fusion_mode", "config") or "config").strip().lower()
    if box_fusion_mode == "on":
        cfg.setdefault("box_fusion", {})["use"] = True
    elif box_fusion_mode == "off":
        cfg.setdefault("box_fusion", {})["use"] = False


def _diagnostic_sort_key(item: dict) -> Tuple[float, float, int]:
    return (
        -float(item.get("teacher_evidence_score", 0.0)),
        -float(item.get("impact_score", 0.0)),
        int(item.get("frame_idx", 0)),
    )


def _run_single_sequence(
    args: argparse.Namespace,
    model,
    clip_model,
    preprocess,
    text_class,
    text_features,
    augmentor,
    preprocessor,
    seq: Optional[str],
):
    from boxfusion.stage_a_demo import ClosedLoopDemoRecorder
    from demo import run
    from tools.utils import get_dataset

    cfg = _load_config(args.dataset_path, args.config, seq)
    apply_ablation_overrides(cfg, args)
    if args.keyframe_gap is not None:
        cfg["data"]["gap"] = int(args.keyframe_gap)
    artifact_policy = build_runtime_artifact_policy_from_args(args)
    cfg["vis"]["rerun"] = bool(args.enable_rerun)
    cfg["runtime_logging"] = {
        "quiet": bool(args.quiet),
        "log_level": str(args.log_level),
        "runtime_print_interval": args.runtime_print_interval,
        "per_profiled_frame_stdout": bool(str(args.log_level) == "verbose" and not args.no_per_profiled_frame_stdout),
        "enable_readonly_tail_reference_audit": bool(artifact_policy.readonly_tail_reference_audit),
        "history_scope_mode": str(args.history_scope_mode),
        "box_fusion_mode": str(args.box_fusion_mode),
        "runtime_artifact_policy": artifact_policy.to_dict(),
    }

    dataset = get_dataset(cfg)
    if hasattr(dataset, "load_arkit_depth"):
        dataset.load_arkit_depth = True
    raw_total_frames = len(dataset) if hasattr(dataset, "__len__") else None
    if args.every_nth_frame is not None:
        dataset = itertools.islice(dataset, 0, None, args.every_nth_frame)
        if raw_total_frames is not None:
            raw_total_frames = math.ceil(raw_total_frames / args.every_nth_frame)
    if args.max_frames is not None:
        raw_total_frames = args.max_frames if raw_total_frames is None else min(raw_total_frames, args.max_frames)

    sequence_id = _infer_sequence_id(cfg, seq)
    recorder = ClosedLoopDemoRecorder(
        output_root=args.output_root,
        sequence_id=sequence_id,
        dataset_root=_infer_dataset_root_for_manifest(cfg, sequence_id),
        capture_stride_frames=args.capture_stride or int(cfg["data"]["gap"]),
        video_fps=args.video_fps,
        canvas_size=(args.canvas_width, args.canvas_height),
        save_scene_graph_vis=bool(artifact_policy.save_scene_graph_visualizations),
        spotlight_count=0 if artifact_policy.core_only else args.spotlight_count,
        room_seg_interval=args.room_seg_interval,
        max_frames=args.max_frames,
        full_rgb_replay=bool(artifact_policy.full_rgb_replay),
        per_frame_pose_overlay=True,
        core_only=artifact_policy.core_only,
        runtime_profile_interval=args.runtime_profile_interval,
    )

    result = run(
        cfg,
        model,
        dataset,
        clip_model,
        preprocess,
        text_class,
        text_features,
        augmentor,
        preprocessor,
        score_thresh=cfg["detection"]["score_thresh"],
        viz_on_gt_points=artifact_policy.prepare_gt_visualization_pointcloud,
        gap=cfg["data"]["gap"],
        re_vis=cfg["vis"]["rerun"],
        room_seg_interval=args.room_seg_interval,
        demo_recorder=recorder,
        debug_room_dir=str(recorder.output_root / "debug_room"),
        save_scene_graph_vis=artifact_policy.save_scene_graph_visualizations,
        max_frames=args.max_frames,
        total_frames=raw_total_frames,
        save_point_cloud=artifact_policy.save_point_cloud,
        write_debug_room_artifacts=artifact_policy.write_debug_room_artifacts,
        runtime_console_config=dict(cfg.get("runtime_logging") or {}),
    )

    summary_payload = None
    demo_outputs = result.get("demo_outputs") or {}
    summary_json = demo_outputs.get("summary_json")
    if summary_json and os.path.exists(summary_json):
        with open(summary_json, "r", encoding="utf-8") as f:
            summary_payload = json.load(f)
        summary_payload["summary_json"] = summary_json
        summary_payload["report_path"] = demo_outputs.get("report_path")
    return sequence_id, result, summary_payload


def _quality_bonus(assessment: str) -> float:
    return {
        "continuous_rgb_honest_bev": 1.1,
        "continuous_rgb_stepwise_map": 0.7,
        "reasonably_progressive": 1.0,
        "moderately_stepwise": 0.35,
        "may_look_sparse": -0.8,
    }.get(str(assessment), 0.0)


def _write_multi_sequence_summary(
    output_root: str,
    aggregate_name: str,
    sequence_summaries: List[Dict],
) -> dict:
    aggregate_dir = Path(output_root) / "_multi_sequence" / aggregate_name
    aggregate_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    runs = []
    for summary in sequence_summaries:
        diagnostics_path = summary.get("revisit_diagnostics_json")
        diagnostics = []
        if diagnostics_path and os.path.exists(diagnostics_path):
            with open(diagnostics_path, "r", encoding="utf-8") as f:
                diagnostics = json.load(f)

        strongest = sorted(diagnostics, key=_diagnostic_sort_key)[:3]
        best_event = strongest[0] if strongest else {}
        strong_revisit_count = sum(float(item.get("teacher_evidence_score", 0.0)) >= 6.0 for item in diagnostics)
        isolated_local_reuse_count = sum(
            item.get("local_merge_audit", {}).get("attribution_label") == "isolated_local_reuse"
            for item in diagnostics
        )
        likely_room_duplicate_count = sum(
            item.get("duplicate_room_evidence", {}).get("likelihood") == "likely"
            for item in diagnostics
        )
        likely_object_duplicate_count = sum(
            item.get("duplicate_object_evidence", {}).get("likelihood") == "likely"
            for item in diagnostics
        )
        presentation_assessment = str(summary.get("presentation_quality_note", {}).get("assessment", ""))
        teacher_presentation_score = round(
            float(best_event.get("teacher_evidence_score", 0.0)) * 2.0
            + strong_revisit_count * 0.7
            + isolated_local_reuse_count * 0.9
            - likely_room_duplicate_count * 1.2
            - likely_object_duplicate_count * 0.8
            + _quality_bonus(presentation_assessment),
            3,
        )
        row = {
            "sequence_id": summary.get("sequence_id"),
            "teacher_presentation_score": teacher_presentation_score,
            "snapshot_count": int(summary.get("snapshot_count", 0)),
            "replay_frame_count": int(summary.get("replay_frame_count", 0)),
            "segmentation_cycle_count": int(summary.get("segmentation_cycle_count", 0)),
            "final_rooms": int(summary.get("final_room_count", 0)),
            "final_objects": int(summary.get("final_object_count", 0)),
            "final_anchors": int(summary.get("final_anchor_count", 0)),
            "grouped_revisits": int(summary.get("revisit_diagnostic_count", 0)),
            "strong_revisits": int(strong_revisit_count),
            "isolated_local_reuse_revisits": int(isolated_local_reuse_count),
            "likely_room_duplicate_revisits": int(likely_room_duplicate_count),
            "likely_object_duplicate_revisits": int(likely_object_duplicate_count),
            "best_event_id": int(best_event.get("event_id", 0)) if best_event else 0,
            "best_event_teacher_score": round(float(best_event.get("teacher_evidence_score", 0.0)), 3) if best_event else 0.0,
            "best_event_trigger": best_event.get("trigger_reason", ""),
            "best_event_rooms": ",".join(str(v) for v in best_event.get("matched_room_ids", [])),
            "best_event_local_vs_global": best_event.get("local_merge_audit", {}).get("attribution_label", ""),
            "presentation_quality": presentation_assessment,
            "replay_mode": summary.get("replay_mode"),
            "summary_json": summary.get("summary_json"),
            "report_path": summary.get("report_path"),
        }
        rows.append(row)
        runs.append(
            {
                "summary": summary,
                "row": row,
                "top_events": strongest,
            }
        )

    rows.sort(
        key=lambda item: (
            -float(item["teacher_presentation_score"]),
            -float(item["best_event_teacher_score"]),
            str(item["sequence_id"]),
        )
    )
    for rank, row in enumerate(rows, start=1):
        row["rank"] = int(rank)

    best_sequence_id = rows[0]["sequence_id"] if rows else None
    json_path = aggregate_dir / "aggregate_summary.json"
    csv_path = aggregate_dir / "aggregate_summary.csv"
    md_path = aggregate_dir / "aggregate_summary.md"

    payload = {
        "aggregate_name": aggregate_name,
        "sequence_count": int(len(rows)),
        "best_sequence_id": best_sequence_id,
        "rows": rows,
        "runs": runs,
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    if rows:
        fieldnames = [
            "rank",
            "sequence_id",
            "teacher_presentation_score",
            "best_event_teacher_score",
            "grouped_revisits",
            "strong_revisits",
            "isolated_local_reuse_revisits",
            "likely_room_duplicate_revisits",
            "likely_object_duplicate_revisits",
            "snapshot_count",
            "replay_frame_count",
            "segmentation_cycle_count",
            "final_rooms",
            "final_objects",
            "final_anchors",
            "presentation_quality",
            "replay_mode",
            "best_event_id",
            "best_event_trigger",
            "best_event_rooms",
            "best_event_local_vs_global",
            "summary_json",
            "report_path",
        ]
        with open(csv_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    md_lines = [
        "# Multi-Sequence Stage A Summary",
        "",
        f"- Sequences compared: {len(rows)}",
        f"- Best teacher-facing sequence: {best_sequence_id}",
        "",
        "| Rank | Sequence | Teacher Score | Best Event | Strong Revisits | Dup Room | Dup Object | Presentation |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        md_lines.append(
            f"| {row['rank']} | {row['sequence_id']} | {row['teacher_presentation_score']:.2f} | "
            f"{row['best_event_teacher_score']:.2f} | {row['strong_revisits']} | "
            f"{row['likely_room_duplicate_revisits']} | {row['likely_object_duplicate_revisits']} | {row['presentation_quality']} |"
        )
    md_lines.extend(["", "## Notes", ""])
    for row in rows:
        md_lines.append(
            "- "
            f"{row['sequence_id']}: grouped revisits={row['grouped_revisits']}, strong revisits={row['strong_revisits']}, "
            f"replay={row['replay_mode']}, frames={row['replay_frame_count']}, "
            f"best event #{row['best_event_id']} ({row['best_event_trigger']}, rooms {row['best_event_rooms'] or 'unknown'}, "
            f"{row['best_event_local_vs_global'] or 'unknown local/global split'}), "
            f"duplicate warnings room/object={row['likely_room_duplicate_revisits']}/{row['likely_object_duplicate_revisits']}."
        )
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md_lines) + "\n")

    return {
        "aggregate_dir": str(aggregate_dir),
        "aggregate_summary_json": str(json_path),
        "aggregate_summary_csv": str(csv_path),
        "aggregate_summary_md": str(md_path),
        "best_sequence_id": best_sequence_id,
    }


def main() -> None:
    import open_clip
    import numpy as np
    import torch

    from boxfusion.cubify_transformer import make_cubify_transformer
    from boxfusion.preprocessor import Augmentor, Preprocessor

    parser = argparse.ArgumentParser(description="Stage A closed-loop demo exporter for BoxFusion")
    parser.add_argument("dataset_path", choices=["CA1M", "scannet", "online", "hm3d"])
    parser.add_argument("--model-path", required=True, help="Path to the BoxFusion / Cubify checkpoint")
    parser.add_argument("--config", required=True, type=str, help="Config path")
    parser.add_argument("--seq", default=None, type=str, help="Sequence id to run")
    parser.add_argument("--seqs", nargs="+", default=None, help="Optional multi-sequence run list")
    parser.add_argument("--class-txt", default="./data/panoptic_categories_nomerge.txt", type=str)
    parser.add_argument("--clip-path", default=None, type=str, help="Optional CLIP checkpoint path")
    parser.add_argument("--clip-model-name", default="ViT-B-32", type=str)
    parser.add_argument("--text-features", default=None, type=str, help="Optional precomputed text feature tensor")
    parser.add_argument("--device", default="cpu", help="cpu | cuda | mps")
    parser.add_argument("--viz-on-gt-points", default=True, action="store_true")
    parser.add_argument("--every-nth-frame", default=None, type=int)
    parser.add_argument("--max-frames", default=None, type=int)
    parser.add_argument("--keyframe-gap", default=None, type=int)
    parser.add_argument("--room-seg-interval", default=100, type=int)
    parser.add_argument(
        "--box-fusion-mode",
        choices=["config", "on", "off"],
        default="config",
        help="Keep the YAML setting or force BoxFusion on/off for paper ablations.",
    )
    parser.add_argument(
        "--history-scope-mode",
        choices=["selective_floor_aware", "broad_history"],
        default="selective_floor_aware",
        help="Keep the current selective floor-aware history mask or disable it for a broader-history baseline.",
    )
    parser.add_argument("--capture-stride", default=None, type=int, help="Snapshot stride in frames")
    parser.add_argument("--runtime-profile-interval", default=None, type=int, help="Frame interval used for runtime-growth profile sampling")
    parser.add_argument("--quiet", action="store_true", help="Keep stdout to warnings plus final summary lines")
    parser.add_argument("--log-level", choices=["summary", "verbose"], default="summary", help="Console verbosity for runtime progress")
    parser.add_argument("--runtime-print-interval", default=50, type=int, help="Progress print interval in processed frames; set 0 to disable periodic progress")
    parser.add_argument("--no-per-profiled-frame-stdout", action="store_true", help="Suppress per-profiled-frame timing lines even in verbose mode")
    parser.add_argument(
        "--enable-readonly-tail-reference-audit",
        action="store_true",
        help="Run the stage-5 shadow-reference readonly-tail audit. Disabled by default for normal runtime and benchmark runs.",
    )
    parser.add_argument(
        "--runtime-artifact-mode",
        choices=RUNTIME_ARTIFACT_MODES,
        default=RUNTIME_ARTIFACT_MODE_BENCHMARK,
        help="benchmark/debug keeps current artifact defaults; service skips artifact-only work by default.",
    )
    parser.add_argument(
        "--service-mode",
        action="store_true",
        help="Alias for --runtime-artifact-mode service.",
    )
    parser.add_argument("--core-only", action="store_true", help="Keep Tier 1 backend artifacts only and suppress optional demo/showcase outputs")
    parser.add_argument(
        "--full-rgb-replay",
        action="store_true",
        help="Export one video frame per processed dataset frame while holding the BEV semantic map between real snapshot refreshes",
    )
    parser.add_argument(
        "--output-root",
        default="./world_model_backend_outputs_v0_2_final/scenes",
        type=str,
        help="Per-scene output root. Defaults to the canonical final backend dataset root.",
    )
    parser.add_argument("--video-fps", default=12, type=int)
    parser.add_argument("--canvas-width", default=1600, type=int)
    parser.add_argument("--canvas-height", default=900, type=int)
    parser.add_argument("--spotlight-count", default=4, type=int, help="Number of strongest revisit events to export as spotlight frames")
    parser.add_argument("--aggregate-name", default="stage_a_multi_sequence", type=str, help="Output folder name for multi-sequence aggregate summaries")
    parser.add_argument("--enable-rerun", action="store_true", help="Keep rerun visualization enabled")
    parser.add_argument("--save-scene-graph-vis", action="store_true", help="Also save per-snapshot scene graph PNGs")
    args = parser.parse_args()

    try:
        checkpoint_blob = torch.load(args.model_path, map_location=args.device or "cpu", weights_only=True)
    except TypeError:
        checkpoint_blob = torch.load(args.model_path, map_location=args.device or "cpu")
    checkpoint = checkpoint_blob["model"]
    backbone_embedding_dimension = checkpoint["backbone.0.patch_embed.proj.weight"].shape[0]
    model = make_cubify_transformer(dimension=backbone_embedding_dimension, depth_model=True).eval()
    model.load_state_dict(checkpoint)
    model = model.to(args.device)

    clip_path = _resolve_clip_path(args.clip_path)
    text_features_path = _resolve_text_features_path(args.text_features)
    clip_model, _, preprocess = open_clip.create_model_and_transforms(
        model_name=args.clip_model_name,
        pretrained=clip_path,
    )
    clip_model = clip_model.to(args.device).eval()

    text_class = np.atleast_1d(np.genfromtxt(args.class_txt, delimiter="\n", dtype=str))
    text_features, resolved_text_features_path, rebuilt_text_features = _load_or_build_text_features(
        torch_module=torch,
        open_clip_module=open_clip,
        clip_model=clip_model,
        clip_model_name=args.clip_model_name,
        text_class=text_class,
        text_features_path=text_features_path,
        requested_text_features_path=args.text_features,
        device=args.device,
    )
    if rebuilt_text_features:
        print(f"[info] Using rebuilt text features at: {resolved_text_features_path}")

    augmentor = Augmentor(("wide/image", "wide/depth"))
    preprocessor = Preprocessor()

    requested_sequences = args.seqs if args.seqs else [args.seq]
    if not requested_sequences:
        requested_sequences = [None]

    sequence_summaries = []
    for seq in requested_sequences:
        sequence_id, result, summary_payload = _run_single_sequence(
            args,
            model,
            clip_model,
            preprocess,
            text_class,
            text_features,
            augmentor,
            preprocessor,
            seq,
        )

        print("\n=== Stage A Demo Package ===")
        print(f"Sequence: {sequence_id}")
        print(f"Processed frames: {result['processed_frames']}")
        if result.get("demo_outputs"):
            for key, value in result["demo_outputs"].items():
                print(f"{key}: {value}")
        if summary_payload is not None:
            sequence_summaries.append(summary_payload)

    if len(sequence_summaries) > 1:
        aggregate_outputs = _write_multi_sequence_summary(
            output_root=args.output_root,
            aggregate_name=args.aggregate_name,
            sequence_summaries=sequence_summaries,
        )
        print("\n=== Stage A Multi-Sequence Summary ===")
        for key, value in aggregate_outputs.items():
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
