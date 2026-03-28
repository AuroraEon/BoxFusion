import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


STRUCTURALISH_DUPLICATE_LABELS = {
    "baseboard",
    "blinds",
    "door",
    "doorframe",
    "floor lamp",
    "light",
    "mirror",
    "power outlet",
    "stairs",
    "switch",
    "wall-wood",
    "window",
    "window-other",
}
IGNORED_WARNING_LABELS = {"ceiling", "floor", "sky", "wall", "window"}
SEMANTICALLY_ODD_LABELS = {
    "pie",
    "snow",
}
PRESENTATION_SUSPICIOUS_LABELS = {
    "bathtub",
    "bed",
    "cabinet",
    "chair",
    "couch",
    "desk",
    "dresser",
    "pillow",
    "sink",
    "sofa",
    "table",
    "toilet",
}
PRESENTATION_NOISY_LABELS = {
    "baseball bat",
    "blender",
    "pie",
    "snow",
}
PRESENTATION_CATEGORY_PRIORITY = {
    "presentation_suspicious": 0,
    "presentation_mixed": 1,
    "presentation_conservative": 2,
    "no_duplicate_warning": 3,
}
DEFAULT_TARGET_SEQUENCES = ["00843-DYehNKdT76V", "00824-Dd4bFSTQ8gi"]


def _load_json(path: Path) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: Path, payload: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_text(path: Path, content: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(content.rstrip() + "\n")


def _round(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _label_key(label: Any) -> str:
    return str(label or "").strip().lower()


def _object_label(obj: Dict[str, Any]) -> str:
    return str(obj.get("label", obj.get("category", "obj")))


def _object_centroid(obj: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    pose = obj.get("pose")
    if isinstance(pose, (list, tuple)) and len(pose) >= 2:
        return float(pose[0]), float(pose[1])
    footprint = obj.get("footprint_2d") or []
    if not footprint:
        return None
    xs = [float(pt[0]) for pt in footprint]
    ys = [float(pt[1]) for pt in footprint]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _distance_xy(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> Optional[float]:
    if a is None or b is None:
        return None
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _bbox(poly: Sequence[Sequence[float]]) -> Optional[Tuple[float, float, float, float]]:
    if not poly:
        return None
    xs = [float(pt[0]) for pt in poly]
    ys = [float(pt[1]) for pt in poly]
    return min(xs), min(ys), max(xs), max(ys)


def _bbox_iou(poly_a: Sequence[Sequence[float]], poly_b: Sequence[Sequence[float]]) -> Optional[float]:
    bbox_a = _bbox(poly_a)
    bbox_b = _bbox(poly_b)
    if bbox_a is None or bbox_b is None:
        return None
    ax1, ay1, ax2, ay2 = bbox_a
    bx1, by1, bx2, by2 = bbox_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    inter_w = max(0.0, ix2 - ix1)
    inter_h = max(0.0, iy2 - iy1)
    inter = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    if denom <= 0.0:
        return None
    return inter / denom


def _footprint_area(obj: Dict[str, Any]) -> float:
    bbox = _bbox(obj.get("footprint_2d") or [])
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)
    size = obj.get("size") or []
    if len(size) >= 2:
        return float(size[0]) * float(size[1])
    return 0.0


def _object_volume(obj: Dict[str, Any]) -> float:
    size = obj.get("size") or []
    if len(size) >= 3:
        return float(size[0]) * float(size[1]) * float(size[2])
    if len(size) >= 2:
        return float(size[0]) * float(size[1])
    return 0.0


def _semantic_label_variants(obj: Dict[str, Any]) -> List[str]:
    variants = []
    for obs in obj.get("semantic_observations", []):
        label = str(obs.get("label", "")).strip()
        if label and label not in variants:
            variants.append(label)
    return variants


def _semantic_confidence(obj: Dict[str, Any]) -> float:
    value = obj.get("semantic_confidence", obj.get("score", 0.0))
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _sort_diagnostics(diagnostics: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return sorted(
        diagnostics,
        key=lambda item: (
            -float(item.get("teacher_evidence_score", 0.0)),
            -float(item.get("impact_score", 0.0)),
            int(item.get("frame_idx", 0)),
        ),
    )


class SequenceAudit:
    def __init__(self, output_root: Path, sequence_id: str) -> None:
        self.output_root = output_root
        self.sequence_id = sequence_id
        self.sequence_dir = output_root / sequence_id
        self.log_dir = self.sequence_dir / "logs"
        self.snapshot_dir = self.sequence_dir / "snapshots"
        self.summary_path = self.log_dir / "summary.json"
        self.diagnostics_path = self.log_dir / "revisit_diagnostics.json"
        if not self.summary_path.exists():
            raise FileNotFoundError(f"Missing summary for {sequence_id}: {self.summary_path}")
        if not self.diagnostics_path.exists():
            raise FileNotFoundError(f"Missing revisit diagnostics for {sequence_id}: {self.diagnostics_path}")
        self.summary = _load_json(self.summary_path)
        self.diagnostics = _load_json(self.diagnostics_path)
        self._vector_map_cache: Dict[int, Dict[str, Any]] = {}

    def load_vector_map(self, frame_idx: int) -> Dict[str, Any]:
        frame_idx = int(frame_idx)
        if frame_idx not in self._vector_map_cache:
            path = self.snapshot_dir / f"vector_map_{frame_idx:06d}.json"
            if not path.exists():
                raise FileNotFoundError(f"Missing vector map for {self.sequence_id} frame {frame_idx}: {path}")
            self._vector_map_cache[frame_idx] = _load_json(path)
        return self._vector_map_cache[frame_idx]


def _room_object_lookup(vector_map: Dict[str, Any], focus_rooms: Iterable[int]) -> Dict[int, List[Dict[str, Any]]]:
    focus_set = {int(room_id) for room_id in focus_rooms}
    grouped: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for obj in vector_map.get("objects", []):
        if "id" not in obj:
            continue
        room_id = int(obj.get("room_uuid", -1))
        if room_id < 0:
            continue
        if focus_set and room_id not in focus_set:
            continue
        label_key = _label_key(_object_label(obj))
        if label_key in IGNORED_WARNING_LABELS:
            continue
        grouped[room_id].append(obj)
    return grouped


def _build_cluster_components(nodes: Sequence[int], edges: Sequence[Tuple[int, int]]) -> List[List[int]]:
    adjacency: Dict[int, set] = {int(node): set() for node in nodes}
    for a, b in edges:
        adjacency[int(a)].add(int(b))
        adjacency[int(b)].add(int(a))
    seen = set()
    components = []
    for node in nodes:
        node = int(node)
        if node in seen:
            continue
        stack = [node]
        comp = []
        seen.add(node)
        while stack:
            current = stack.pop()
            comp.append(current)
            for nxt in sorted(adjacency[current]):
                if nxt in seen:
                    continue
                seen.add(nxt)
                stack.append(nxt)
        components.append(sorted(comp))
    return components


def _cross_label_replacements(
    before_objects: Sequence[Dict[str, Any]],
    after_objects: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    before_lookup = {int(obj["id"]): obj for obj in before_objects if "id" in obj}
    after_lookup = {int(obj["id"]): obj for obj in after_objects if "id" in obj}
    before_ids = set(before_lookup)
    after_ids = set(after_lookup)
    added_ids = sorted(after_ids - before_ids)
    removed_ids = sorted(before_ids - after_ids)
    replacements = []
    for removed_id in removed_ids:
        removed_obj = before_lookup[removed_id]
        removed_centroid = _object_centroid(removed_obj)
        if removed_centroid is None:
            continue
        for added_id in added_ids:
            added_obj = after_lookup[added_id]
            added_centroid = _object_centroid(added_obj)
            dist = _distance_xy(removed_centroid, added_centroid)
            if dist is None or dist > 0.65:
                continue
            removed_label = _object_label(removed_obj)
            added_label = _object_label(added_obj)
            if removed_label == added_label:
                continue
            replacements.append(
                {
                    "from_object_id": int(removed_id),
                    "to_object_id": int(added_id),
                    "from_label": removed_label,
                    "to_label": added_label,
                    "centroid_distance_m": _round(dist),
                }
            )
    return replacements


def _classify_cluster(
    label: str,
    room_id: int,
    cluster_objects: Sequence[Dict[str, Any]],
    pair_summaries: Sequence[Dict[str, Any]],
    before_ids: set,
    added_ids: set,
    retained_ids: set,
    semantic_replacements: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    label_key = _label_key(label)
    object_ids = [int(obj["id"]) for obj in cluster_objects if "id" in obj]
    preexisting_ids = [obj_id for obj_id in object_ids if obj_id in before_ids]
    added_cluster_ids = [obj_id for obj_id in object_ids if obj_id in added_ids]
    retained_cluster_ids = [obj_id for obj_id in object_ids if obj_id in retained_ids]
    max_iou = max((float(pair.get("footprint_iou", 0.0) or 0.0) for pair in pair_summaries), default=0.0)
    min_distance = min(
        (float(pair.get("centroid_distance_m", 1e9) or 1e9) for pair in pair_summaries),
        default=None,
    )
    max_volume = max((_object_volume(obj) for obj in cluster_objects), default=0.0)
    max_area = max((_footprint_area(obj) for obj in cluster_objects), default=0.0)
    semantic_variants = sorted({variant for obj in cluster_objects for variant in _semantic_label_variants(obj)})
    low_confidence = any(_semantic_confidence(obj) < 0.6 for obj in cluster_objects)
    nearby_cross_label = [
        item
        for item in semantic_replacements
        if item["from_object_id"] in object_ids or item["to_object_id"] in object_ids
    ]

    cause_scores = Counter()
    if label_key in STRUCTURALISH_DUPLICATE_LABELS or max_area < 0.05 or max_volume < 0.02:
        cause_scores["heuristic_over_triggering"] += 3
    if label_key in SEMANTICALLY_ODD_LABELS or low_confidence or len(semantic_variants) > 1 or nearby_cross_label:
        cause_scores["semantic_label_drift"] += 2
    if added_cluster_ids:
        cause_scores["revisit_time_reobservation"] += 2
    if not added_cluster_ids and len(preexisting_ids) == len(object_ids):
        cause_scores["tracking_fragmentation"] += 1
        cause_scores["heuristic_over_triggering"] += 1
    if max_iou >= 0.35 or (min_distance is not None and min_distance <= 0.12):
        cause_scores["spatial_overlap_without_merge"] += 3
    elif min_distance is not None and min_distance <= 0.22:
        cause_scores["spatial_overlap_without_merge"] += 1

    if not cause_scores:
        cause_scores["heuristic_over_triggering"] = 1

    ordered_causes = [cause for cause, _ in cause_scores.most_common()]
    primary_cause = ordered_causes[0]
    suspicious = primary_cause in {
        "spatial_overlap_without_merge",
        "tracking_fragmentation",
        "revisit_time_reobservation",
        "semantic_label_drift",
    } and label_key not in STRUCTURALISH_DUPLICATE_LABELS
    conservative = primary_cause == "heuristic_over_triggering" and len(added_cluster_ids) == 0

    note_parts = []
    if added_cluster_ids:
        note_parts.append("includes newly added object ids on the revisit effect frame")
    if len(preexisting_ids) == len(object_ids):
        note_parts.append("all clustered ids already existed before the revisit effect frame")
    if nearby_cross_label:
        note_parts.append("nearby removed/added objects suggest semantic relabeling drift")
    if max_iou >= 0.35:
        note_parts.append("footprints overlap strongly without an explicit merge")
    elif min_distance is not None and min_distance <= 0.12:
        note_parts.append("centroids are extremely close without an explicit merge")
    if label_key in STRUCTURALISH_DUPLICATE_LABELS:
        note_parts.append("label is a known conservative trigger category in this Stage A pass")
    if label_key in SEMANTICALLY_ODD_LABELS:
        note_parts.append("label looks semantically unstable for an indoor scene")

    return {
        "room_id": int(room_id),
        "label": label,
        "object_ids": sorted(object_ids),
        "cluster_size": int(len(object_ids)),
        "pair_count": int(len(pair_summaries)),
        "preexisting_object_ids": sorted(preexisting_ids),
        "added_object_ids": sorted(added_cluster_ids),
        "retained_object_ids": sorted(retained_cluster_ids),
        "centroid_distance_m_min": _round(min_distance),
        "centroid_distance_m_max": _round(
            max((float(pair.get("centroid_distance_m", 0.0) or 0.0) for pair in pair_summaries), default=0.0)
        ),
        "footprint_iou_max": _round(max_iou),
        "semantic_label_variants": semantic_variants,
        "nearby_cross_label_replacements": nearby_cross_label,
        "cause_scores": {key: int(value) for key, value in cause_scores.items()},
        "primary_cause": primary_cause,
        "supporting_causes": ordered_causes[1:3],
        "assessment": "likely_conservative" if conservative else ("likely_suspicious" if suspicious else "mixed"),
        "notes": note_parts,
        "pair_summaries": list(pair_summaries),
    }


def _cluster_signature(cluster: Dict[str, Any]) -> Tuple[int, str, Tuple[int, ...]]:
    return (
        int(cluster.get("room_id", -1)),
        str(cluster.get("label", "")),
        tuple(int(obj_id) for obj_id in cluster.get("object_ids", [])),
    )


def _presentation_category_note(category: str) -> str:
    if category == "presentation_suspicious":
        return "worth highlighting if a teacher asks about duplicate-object risk"
    if category == "presentation_mixed":
        return "worth a brief caveat, but not the main duplicate headline"
    if category == "presentation_conservative":
        return "better framed as low-priority presentation noise than as a strong failure case"
    return "no duplicate-object warning was raised for this grouped revisit"


def _classify_presentation_cluster(
    cluster: Dict[str, Any],
    repeated_signature_count: int,
) -> Dict[str, Any]:
    label = str(cluster.get("label", "obj"))
    label_key = _label_key(label)
    pair_summaries = list(cluster.get("pair_summaries", []))
    preexisting_pairs = sum(1 for pair in pair_summaries if bool(pair.get("pair_preexisted_before_event", False)))
    pair_count = max(1, int(cluster.get("pair_count", 0) or len(pair_summaries) or 1))
    preexisting_pair_fraction = preexisting_pairs / float(pair_count)
    added_object_ids = list(cluster.get("added_object_ids", []))
    all_preexisting_ids = bool(cluster.get("object_ids")) and (
        len(cluster.get("preexisting_object_ids", [])) == len(cluster.get("object_ids", []))
    ) and not added_object_ids
    any_new_pair_signal = any(
        (not bool(pair.get("pair_preexisted_before_event", False))) or bool(pair.get("includes_added_object", False))
        for pair in pair_summaries
    )
    max_iou = float(cluster.get("footprint_iou_max", 0.0) or 0.0)
    min_distance = cluster.get("centroid_distance_m_min")
    if min_distance is not None:
        min_distance = float(min_distance)

    conservative_score = 0
    suspicious_score = 0
    conservative_reasons: List[str] = []
    suspicious_reasons: List[str] = []

    if label_key in STRUCTURALISH_DUPLICATE_LABELS:
        conservative_score += 4
        conservative_reasons.append("structural/noisy label")
    if label_key in PRESENTATION_NOISY_LABELS or label_key in SEMANTICALLY_ODD_LABELS:
        conservative_score += 3
        conservative_reasons.append("semantically odd or presentation-noisy label")
    if repeated_signature_count >= 8:
        conservative_score += 4
        conservative_reasons.append("same room-local signature retriggered many times")
    elif repeated_signature_count >= 3:
        conservative_score += 3
        conservative_reasons.append("same room-local signature retriggered repeatedly")
    elif repeated_signature_count == 2:
        conservative_score += 1
        conservative_reasons.append("same room-local signature already repeated")
    if all_preexisting_ids or preexisting_pair_fraction >= 0.99:
        conservative_score += 3
        conservative_reasons.append("pair already existed before the revisit effect frame")
    elif preexisting_pair_fraction >= 0.75:
        conservative_score += 1
        conservative_reasons.append("most of the pair evidence was already present beforehand")
    if str(cluster.get("primary_cause")) == "heuristic_over_triggering":
        conservative_score += 3
        conservative_reasons.append("raw audit already tags this as heuristic over-triggering")
    if str(cluster.get("primary_cause")) == "semantic_label_drift" and (
        label_key in PRESENTATION_NOISY_LABELS or repeated_signature_count >= 3
    ):
        conservative_score += 2
        conservative_reasons.append("looks more like relabeling drift than a fresh duplicate")
    if max_iou < 0.10 and (min_distance is None or min_distance > 0.22):
        conservative_score += 1
        conservative_reasons.append("spatial overlap signal is weak")

    if label_key in PRESENTATION_SUSPICIOUS_LABELS:
        suspicious_score += 2
        suspicious_reasons.append("compact room-local object category")
    if added_object_ids:
        suspicious_score += 2
        suspicious_reasons.append("includes newly added object ids on the effect frame")
    if any_new_pair_signal:
        suspicious_score += 2
        suspicious_reasons.append("pair did not fully preexist before the revisit")
    if max_iou >= 0.40:
        suspicious_score += 3
        suspicious_reasons.append("strong footprint overlap without a merge")
    elif max_iou >= 0.25:
        suspicious_score += 2
        suspicious_reasons.append("clear footprint overlap")
    elif max_iou >= 0.15:
        suspicious_score += 1
        suspicious_reasons.append("some footprint overlap")
    if min_distance is not None:
        if min_distance <= 0.12:
            suspicious_score += 2
            suspicious_reasons.append("centroids are extremely close")
        elif min_distance <= 0.22:
            suspicious_score += 1
            suspicious_reasons.append("centroids stay fairly close")
    if str(cluster.get("primary_cause")) in {
        "spatial_overlap_without_merge",
        "revisit_time_reobservation",
        "tracking_fragmentation",
    }:
        suspicious_score += 2
        suspicious_reasons.append("raw audit found a more substantive duplicate pattern")
    if int(cluster.get("pair_count", 0)) >= 2 or int(cluster.get("cluster_size", 0)) >= 3:
        suspicious_score += 1
        suspicious_reasons.append("multiple close same-label objects are involved")

    if conservative_score >= suspicious_score + 3:
        category = "presentation_conservative"
    elif suspicious_score >= conservative_score + 2:
        category = "presentation_suspicious"
    else:
        category = "presentation_mixed"

    summary_reasons = suspicious_reasons[:2] if category == "presentation_suspicious" else conservative_reasons[:2]
    if category == "presentation_mixed":
        summary_reasons = suspicious_reasons[:1] + conservative_reasons[:1]

    return {
        "category": category,
        "closer_attention": bool(category != "presentation_conservative"),
        "conservative_score": int(conservative_score),
        "suspicious_score": int(suspicious_score),
        "repeated_signature_count": int(repeated_signature_count),
        "preexisting_pair_fraction": round(preexisting_pair_fraction, 3),
        "conservative_reasons": conservative_reasons,
        "suspicious_reasons": suspicious_reasons,
        "summary_reasons": summary_reasons,
        "teacher_note": _presentation_category_note(category),
    }


def _build_presentation_duplicate_summary(
    sequence: SequenceAudit,
    duplicate_payload: Dict[str, Any],
) -> Dict[str, Any]:
    event_audits = list(duplicate_payload.get("event_duplicate_audits", []))
    signature_counts = Counter()
    for audit in event_audits:
        for cluster in audit.get("duplicate_clusters", []):
            signature_counts[_cluster_signature(cluster)] += 1

    event_summaries = []
    room_stats: Dict[int, Dict[str, Any]] = defaultdict(
        lambda: {
            "categories": Counter(),
            "pair_counts": Counter(),
            "labels": Counter(),
            "warning_event_ids": set(),
        }
    )
    label_stats: Dict[str, Dict[str, Any]] = defaultdict(
        lambda: {
            "categories": Counter(),
            "pair_count": 0,
            "pair_counts": Counter(),
            "warning_event_ids": set(),
            "room_ids": set(),
        }
    )
    recurring_signature_map: Dict[Tuple[int, str, Tuple[int, ...]], Dict[str, Any]] = {}
    event_category_counts = Counter()
    pair_category_counts = Counter()
    cluster_category_counts = Counter()

    for audit in event_audits:
        warning_flag = bool(audit.get("warning_flag", False))
        category_pair_counts = Counter()
        category_cluster_counts = Counter()
        closer_labels = Counter()
        low_priority_labels = Counter()
        summary_reasons = []

        for cluster in audit.get("duplicate_clusters", []):
            signature = _cluster_signature(cluster)
            presentation = _classify_presentation_cluster(cluster, signature_counts[signature])
            cluster["presentation_duplicate"] = presentation
            category = presentation["category"]
            pair_count = int(cluster.get("pair_count", 0))
            category_pair_counts[category] += pair_count
            category_cluster_counts[category] += 1
            pair_category_counts[category] += pair_count
            cluster_category_counts[category] += 1
            summary_reasons.extend(presentation.get("summary_reasons", []))
            if presentation["closer_attention"]:
                closer_labels[str(cluster.get("label", "obj"))] += pair_count
            else:
                low_priority_labels[str(cluster.get("label", "obj"))] += pair_count
            room_id = int(cluster.get("room_id", -1))
            room_stats[room_id]["categories"][category] += 1
            room_stats[room_id]["pair_counts"][category] += pair_count
            room_stats[room_id]["labels"][str(cluster.get("label", "obj"))] += pair_count
            room_stats[room_id]["warning_event_ids"].add(int(audit["event_id"]))
            label = str(cluster.get("label", "obj"))
            label_stats[label]["categories"][category] += 1
            label_stats[label]["pair_count"] += pair_count
            label_stats[label]["pair_counts"][category] += pair_count
            label_stats[label]["warning_event_ids"].add(int(audit["event_id"]))
            label_stats[label]["room_ids"].add(room_id)

        if not warning_flag:
            event_category = "no_duplicate_warning"
        elif category_pair_counts["presentation_suspicious"] >= max(category_pair_counts["presentation_conservative"], 1):
            event_category = "presentation_suspicious"
        elif category_pair_counts["presentation_suspicious"] > 0 or category_pair_counts["presentation_mixed"] > 0:
            event_category = "presentation_mixed"
        else:
            event_category = "presentation_conservative"

        if warning_flag:
            event_category_counts[event_category] += 1
        audit["presentation_duplicate"] = {
            "category": event_category,
            "closer_attention": bool(event_category in {"presentation_mixed", "presentation_suspicious"}),
            "pair_category_counts": {key: int(value) for key, value in category_pair_counts.items()},
            "cluster_category_counts": {key: int(value) for key, value in category_cluster_counts.items()},
            "top_closer_attention_labels": [label for label, _ in closer_labels.most_common(3)],
            "top_low_priority_labels": [label for label, _ in low_priority_labels.most_common(3)],
            "summary_reasons": list(dict.fromkeys(summary_reasons))[:3],
            "teacher_note": _presentation_category_note(event_category),
        }
        event_summaries.append(
            {
                "event_id": int(audit["event_id"]),
                "frame_idx": int(audit["frame_idx"]),
                "effect_frame_idx": int(audit["effect_frame_idx"]),
                "matched_room_ids": [int(room_id) for room_id in audit.get("matched_room_ids", [])],
                "raw_warning_flag": warning_flag,
                "raw_candidate_pair_count": int(audit.get("candidate_pair_count", 0)),
                "raw_preexisting_pair_count": int(audit.get("preexisting_pair_count", 0)),
                "raw_new_or_changed_pair_count": int(audit.get("new_or_changed_pair_count", 0)),
                "raw_overall_assessment": str(audit.get("overall_assessment", "")),
                "presentation_category": event_category,
                "closer_attention": bool(event_category in {"presentation_mixed", "presentation_suspicious"}),
                "top_closer_attention_labels": [label for label, _ in closer_labels.most_common(4)],
                "top_low_priority_labels": [label for label, _ in low_priority_labels.most_common(4)],
                "teacher_note": audit["presentation_duplicate"]["teacher_note"],
                "summary_reasons": audit["presentation_duplicate"]["summary_reasons"],
                "teacher_evidence_score": float(audit.get("teacher_evidence_score", 0.0)),
            }
        )

    for audit in event_audits:
        for cluster in audit.get("duplicate_clusters", []):
            presentation = cluster.get("presentation_duplicate", {})
            repeated_count = int(presentation.get("repeated_signature_count", 1))
            if repeated_count < 3 or presentation.get("category") != "presentation_conservative":
                continue
            signature = _cluster_signature(cluster)
            recurring_signature_map[signature] = {
                "room_id": int(cluster.get("room_id", -1)),
                "label": str(cluster.get("label", "obj")),
                "object_ids": [int(obj_id) for obj_id in cluster.get("object_ids", [])],
                "repeated_warning_event_count": repeated_count,
                "pair_count": int(cluster.get("pair_count", 0)),
                "teacher_note": presentation.get("teacher_note"),
            }

    recurring_signatures = sorted(
        recurring_signature_map.values(),
        key=lambda item: (
            -int(item["repeated_warning_event_count"]),
            -int(item["pair_count"]),
            str(item["label"]).lower(),
            int(item["room_id"]),
        )
    )

    def _room_payload(room_id: int, stats: Dict[str, Any]) -> Dict[str, Any]:
        closer_pair_count = int(
            stats["pair_counts"]["presentation_suspicious"] + stats["pair_counts"]["presentation_mixed"]
        )
        return {
            "room_id": int(room_id),
            "warning_event_count": int(len(stats["warning_event_ids"])),
            "closer_attention_pair_count": closer_pair_count,
            "low_priority_pair_count": int(stats["pair_counts"]["presentation_conservative"]),
            "top_labels": [label for label, _ in stats["labels"].most_common(4)],
            "presentation_breakdown": {key: int(value) for key, value in stats["categories"].items()},
        }

    rooms_to_discuss = sorted(
        (_room_payload(room_id, stats) for room_id, stats in room_stats.items()),
        key=lambda item: (
            -int(item["closer_attention_pair_count"]),
            -int(item["warning_event_count"]),
            int(item["room_id"]),
        ),
    )
    labels_to_discuss = []
    low_priority_labels = []
    for label, stats in label_stats.items():
        payload = {
            "label": label,
            "warning_event_count": int(len(stats["warning_event_ids"])),
            "pair_count": int(stats["pair_count"]),
            "closer_attention_pair_count": int(
                stats["pair_counts"]["presentation_suspicious"] + stats["pair_counts"]["presentation_mixed"]
            ),
            "low_priority_pair_count": int(stats["pair_counts"]["presentation_conservative"]),
            "room_ids": sorted(int(room_id) for room_id in stats["room_ids"]),
            "presentation_breakdown": {key: int(value) for key, value in stats["categories"].items()},
        }
        closer_count = int(
            stats["categories"]["presentation_suspicious"] + stats["categories"]["presentation_mixed"]
        )
        if closer_count > 0:
            payload["closer_attention_cluster_count"] = closer_count
            labels_to_discuss.append(payload)
        if int(stats["categories"]["presentation_conservative"]) > 0:
            payload["low_priority_cluster_count"] = int(stats["categories"]["presentation_conservative"])
            low_priority_labels.append(payload)

    labels_to_discuss.sort(
        key=lambda item: (
            -int(item.get("presentation_breakdown", {}).get("presentation_suspicious", 0)),
            -int(item.get("presentation_breakdown", {}).get("presentation_mixed", 0)),
            -int(item["pair_count"]),
            str(item["label"]).lower(),
        )
    )
    low_priority_labels.sort(
        key=lambda item: (
            -int(item.get("presentation_breakdown", {}).get("presentation_conservative", 0)),
            -int(item["pair_count"]),
            str(item["label"]).lower(),
        )
    )

    event_summaries.sort(
        key=lambda item: (
            PRESENTATION_CATEGORY_PRIORITY.get(item["presentation_category"], 99),
            -float(item.get("teacher_evidence_score", 0.0)),
            int(item["frame_idx"]),
        )
    )
    event_summary_by_id = {int(item["event_id"]): item for item in event_summaries}
    for row in duplicate_payload.get("csv_rows", []):
        event_summary = event_summary_by_id.get(int(row["event_id"]))
        if not event_summary:
            row["presentation_category"] = "no_duplicate_warning"
            row["presentation_closer_attention"] = False
            continue
        row["presentation_category"] = event_summary["presentation_category"]
        row["presentation_closer_attention"] = bool(event_summary["closer_attention"])

    raw_summary = duplicate_payload.get("summary", {})
    closer_attention_event_count = int(
        event_category_counts["presentation_suspicious"] + event_category_counts["presentation_mixed"]
    )
    closer_attention_pair_count = int(
        pair_category_counts["presentation_suspicious"] + pair_category_counts["presentation_mixed"]
    )
    headline_findings = [
        (
            f"Raw duplicate warnings stay available ({int(raw_summary.get('duplicate_warning_event_count', 0))} events, "
            f"{int(raw_summary.get('duplicate_candidate_pair_count', 0))} close-pair triggers), but the presentation layer "
            f"shrinks the teacher-facing concern set to {closer_attention_event_count} mixed/suspicious events."
        ),
        (
            f"{int(raw_summary.get('preexisting_pair_fraction', 0.0) * 100):d}% of raw warning pairs already existed before the revisit effect frame, "
            "so repeated warnings should not be framed as fresh duplicate creation by default."
        ),
    ]
    if low_priority_labels:
        low_priority_only_labels = [
            item for item in low_priority_labels if int(item.get("closer_attention_pair_count", 0)) == 0
        ]
        chosen_low_priority_labels = low_priority_only_labels or low_priority_labels
        headline_findings.append(
            "Lower-priority presentation noise is dominated by "
            + ", ".join(f"`{item['label']}`" for item in chosen_low_priority_labels[:4])
            + "."
        )
    if labels_to_discuss:
        headline_findings.append(
            "If a teacher asks about object duplication, focus on "
            + ", ".join(f"`{item['label']}`" for item in labels_to_discuss[:4])
            + "."
        )

    teacher_talking_points = [
        "Lead with the suspicious or mixed subset, not the raw duplicate-warning total.",
        "Describe conservative warnings as repeated low-priority triggers that often predate the revisit effect frame.",
        "If pressed for concrete examples, point to the top mixed/suspicious rooms and labels below rather than to `wall-wood`, `light`, or similar structural labels.",
    ]

    return {
        "sequence_id": sequence.sequence_id,
        "categorization_note": "This is a presentation-oriented post-hoc heuristic layered on top of the raw duplicate audit. It does not replace the raw diagnostics or prove backend correctness.",
        "category_definitions": {
            "presentation_conservative": "Low-priority presentation noise: repeated pre-existing pairs, structural/noisy labels, or weak overlap patterns.",
            "presentation_mixed": "Worth a brief caveat: some duplicate signal remains, but the evidence is partly conservative or long-lived.",
            "presentation_suspicious": "Closer attention: compact semantic objects or new/revisit-local overlap patterns that are more worth discussing.",
        },
        "summary": {
            "raw_warning_event_count": int(raw_summary.get("duplicate_warning_event_count", 0)),
            "raw_candidate_pair_count": int(raw_summary.get("duplicate_candidate_pair_count", 0)),
            "raw_preexisting_pair_fraction": float(raw_summary.get("preexisting_pair_fraction", 0.0)),
            "presentation_conservative_event_count": int(event_category_counts["presentation_conservative"]),
            "presentation_mixed_event_count": int(event_category_counts["presentation_mixed"]),
            "presentation_suspicious_event_count": int(event_category_counts["presentation_suspicious"]),
            "closer_attention_event_count": closer_attention_event_count,
            "presentation_conservative_pair_count": int(pair_category_counts["presentation_conservative"]),
            "presentation_mixed_pair_count": int(pair_category_counts["presentation_mixed"]),
            "presentation_suspicious_pair_count": int(pair_category_counts["presentation_suspicious"]),
            "closer_attention_pair_count": closer_attention_pair_count,
            "presentation_conservative_cluster_count": int(cluster_category_counts["presentation_conservative"]),
            "presentation_mixed_cluster_count": int(cluster_category_counts["presentation_mixed"]),
            "presentation_suspicious_cluster_count": int(cluster_category_counts["presentation_suspicious"]),
        },
        "headline_findings": headline_findings,
        "teacher_talking_points": teacher_talking_points,
        "labels_to_discuss": labels_to_discuss,
        "low_priority_labels": low_priority_labels,
        "rooms_to_discuss": rooms_to_discuss,
        "strongest_events_to_discuss": [
            item for item in event_summaries if item["closer_attention"]
        ][:8],
        "event_summaries": event_summaries,
        "recurring_low_priority_signatures": recurring_signatures[:12],
    }


def _event_duplicate_audit(sequence: SequenceAudit, event: Dict[str, Any]) -> Dict[str, Any]:
    before_frame = int(event["comparison_frames"]["before_frame_idx"])
    effect_frame = int(event["effect_frame_idx"])
    before_map = sequence.load_vector_map(before_frame)
    effect_map = sequence.load_vector_map(effect_frame)
    focus_rooms = [int(room_id) for room_id in event.get("matched_room_ids", []) if int(room_id) >= 0]
    before_grouped = _room_object_lookup(before_map, focus_rooms)
    effect_grouped = _room_object_lookup(effect_map, focus_rooms)
    retained_ids = set(event.get("object_fusion_updates", {}).get("retained_object_ids", []))
    added_ids = set(event.get("object_fusion_updates", {}).get("added_object_ids", []))
    duplicate_pairs = []
    clusters = []
    cause_counter = Counter()
    assessment_counter = Counter()

    for room_id in sorted(set(before_grouped) | set(effect_grouped)):
        room_before = before_grouped.get(room_id, [])
        room_after = effect_grouped.get(room_id, [])
        semantic_replacements = _cross_label_replacements(room_before, room_after)
        label_groups: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
        for obj in room_after:
            label_groups[_object_label(obj)].append(obj)

        before_ids = {int(obj["id"]) for obj in room_before if "id" in obj}
        for label, objects in sorted(label_groups.items()):
            if len(objects) < 2:
                continue
            nodes = [int(obj["id"]) for obj in objects if "id" in obj]
            if len(nodes) < 2:
                continue
            lookup = {int(obj["id"]): obj for obj in objects if "id" in obj}
            edges = []
            pair_summaries = []
            for idx, obj_a in enumerate(objects):
                obj_a_id = int(obj_a["id"])
                for obj_b in objects[idx + 1:]:
                    obj_b_id = int(obj_b["id"])
                    dist = _distance_xy(_object_centroid(obj_a), _object_centroid(obj_b))
                    if dist is None or dist > 0.45:
                        continue
                    iou = _bbox_iou(obj_a.get("footprint_2d") or [], obj_b.get("footprint_2d") or [])
                    pair_payload = {
                        "object_ids": [obj_a_id, obj_b_id],
                        "room_id": int(room_id),
                        "label": label,
                        "centroid_distance_m": _round(dist),
                        "footprint_iou": _round(iou),
                        "pair_preexisted_before_event": bool(obj_a_id in before_ids and obj_b_id in before_ids),
                        "includes_added_object": bool(obj_a_id in added_ids or obj_b_id in added_ids),
                        "fully_retained_pair": bool(obj_a_id in retained_ids and obj_b_id in retained_ids),
                    }
                    pair_summaries.append(pair_payload)
                    edges.append((obj_a_id, obj_b_id))
                    duplicate_pairs.append(pair_payload)

            if not edges:
                continue
            components = _build_cluster_components(nodes, edges)
            for component in components:
                if len(component) < 2:
                    continue
                component_objects = [lookup[obj_id] for obj_id in component if obj_id in lookup]
                component_pairs = [
                    pair for pair in pair_summaries if set(pair["object_ids"]).issubset(set(component))
                ]
                cluster_summary = _classify_cluster(
                    label=label,
                    room_id=int(room_id),
                    cluster_objects=component_objects,
                    pair_summaries=component_pairs,
                    before_ids=before_ids,
                    added_ids=added_ids,
                    retained_ids=retained_ids,
                    semantic_replacements=semantic_replacements,
                )
                clusters.append(cluster_summary)
                cause_counter[cluster_summary["primary_cause"]] += 1
                assessment_counter[cluster_summary["assessment"]] += 1

    duplicate_pairs.sort(
        key=lambda item: (
            not bool(item.get("pair_preexisted_before_event", False)),
            float(item.get("centroid_distance_m", 99.0) or 99.0),
            str(item.get("label", "")),
        )
    )
    clusters.sort(
        key=lambda item: (
            0 if item["assessment"] == "likely_suspicious" else (1 if item["assessment"] == "mixed" else 2),
            float(item.get("centroid_distance_m_min", 99.0) or 99.0),
            str(item.get("label", "")),
        )
    )
    pair_preexisting = sum(1 for pair in duplicate_pairs if pair["pair_preexisted_before_event"])
    pair_new = len(duplicate_pairs) - pair_preexisting
    if assessment_counter["likely_suspicious"] > 0:
        overall = "likely_suspicious"
    elif assessment_counter["mixed"] > 0 or pair_new > 0:
        overall = "mixed"
    elif duplicate_pairs:
        overall = "likely_conservative"
    else:
        overall = "no_duplicate_signal"

    return {
        "event_id": int(event["event_id"]),
        "frame_idx": int(event["frame_idx"]),
        "effect_frame_idx": int(effect_frame),
        "teacher_evidence_score": float(event.get("teacher_evidence_score", 0.0)),
        "impact_score": float(event.get("impact_score", 0.0)),
        "matched_room_ids": [int(room_id) for room_id in focus_rooms],
        "warning_flag": bool(event.get("duplicate_object_warning", {}).get("flag", False)),
        "candidate_pair_count": int(len(duplicate_pairs)),
        "cluster_count": int(len(clusters)),
        "preexisting_pair_count": int(pair_preexisting),
        "new_or_changed_pair_count": int(pair_new),
        "overall_assessment": overall,
        "primary_causes": [cause for cause, _ in cause_counter.most_common(3)],
        "cause_counts": {key: int(value) for key, value in cause_counter.items()},
        "assessment_counts": {key: int(value) for key, value in assessment_counter.items()},
        "duplicate_pairs": duplicate_pairs,
        "duplicate_clusters": clusters,
    }


def _event_spotlight_score(event: Dict[str, Any], event_audit: Dict[str, Any]) -> float:
    score = float(event.get("teacher_evidence_score", 0.0))
    updates = event.get("object_fusion_updates", {})
    score += min(float(updates.get("merged_object_count", 0)), 3.0) * 0.9
    score += min(float(updates.get("retained_object_count", 0)), 20.0) * 0.04
    score += min(float(updates.get("added_object_count", 0)), 4.0) * 0.12
    if event_audit.get("overall_assessment") == "likely_suspicious":
        score -= 1.2
    elif event_audit.get("overall_assessment") == "mixed":
        score -= 0.5
    return round(score, 3)


def _build_duplicate_markdown(sequence_id: str, payload: Dict[str, Any]) -> str:
    presentation = payload.get("presentation_duplicate_summary", {})
    presentation_summary = presentation.get("summary", {})
    lines = [
        f"# Object Duplicate Audit: {sequence_id}",
        "",
        "## Summary",
        "",
        f"- Grouped revisit events: {payload['summary']['grouped_revisit_events']}",
        f"- Duplicate-object warning events: {payload['summary']['duplicate_warning_event_count']}",
        f"- Duplicate candidate pairs: {payload['summary']['duplicate_candidate_pair_count']}",
        f"- Pre-existing pair share: {payload['summary']['preexisting_pair_fraction']:.3f}",
        f"- Event assessment mix: conservative/mixed/suspicious = {payload['summary']['likely_conservative_event_count']}/{payload['summary']['mixed_event_count']}/{payload['summary']['likely_suspicious_event_count']}",
        "",
    ]
    if presentation_summary:
        lines.extend(
            [
                "## Presentation-Aware Read",
                "",
                (
                    f"- Raw warning events / candidate pairs: {presentation_summary['raw_warning_event_count']} / "
                    f"{presentation_summary['raw_candidate_pair_count']}"
                ),
                (
                    f"- Presentation categories: conservative={presentation_summary['presentation_conservative_event_count']} events "
                    f"({presentation_summary['presentation_conservative_pair_count']} pairs), mixed={presentation_summary['presentation_mixed_event_count']} "
                    f"({presentation_summary['presentation_mixed_pair_count']} pairs), suspicious={presentation_summary['presentation_suspicious_event_count']} "
                    f"({presentation_summary['presentation_suspicious_pair_count']} pairs)"
                ),
                (
                    f"- Closer-attention subset to discuss with a teacher: {presentation_summary['closer_attention_event_count']} events / "
                    f"{presentation_summary['closer_attention_pair_count']} pairs"
                ),
                f"- Scope note: {presentation.get('categorization_note')}",
                "",
            ]
        )
        if presentation.get("labels_to_discuss"):
            lines.append(
                "- Main labels to discuss if asked: "
                + ", ".join(f"`{item['label']}`" for item in presentation["labels_to_discuss"][:5])
                + "."
            )
        if presentation.get("low_priority_labels"):
            low_priority_only_labels = [
                item for item in presentation["low_priority_labels"] if int(item.get("closer_attention_pair_count", 0)) == 0
            ]
            chosen_low_priority_labels = low_priority_only_labels or presentation["low_priority_labels"]
            lines.append(
                "- Lower-priority repeated labels: "
                + ", ".join(f"`{item['label']}`" for item in chosen_low_priority_labels[:5])
                + "."
            )
        lines.extend(["", "## Rooms With Most Duplicate Warnings", ""])
    else:
        lines.extend([
        "## Rooms With Most Duplicate Warnings",
        "",
        ])
    for room_item in payload["rooms_with_most_duplicate_warnings"][:6]:
        lines.append(
            "- "
            f"Room {room_item['room_id']}: warning events={room_item['warning_event_count']}, "
            f"candidate pairs={room_item['candidate_pair_count']}, "
            f"top labels={', '.join(room_item['top_labels']) or 'none'}, "
            f"dominant causes={', '.join(room_item['dominant_causes']) or 'none'}."
        )
    lines.extend(["", "## Label Hotspots", ""])
    for label_item in payload["labels_with_most_duplicate_warnings"][:10]:
        lines.append(
            "- "
            f"{label_item['label']}: pairs={label_item['candidate_pair_count']}, "
            f"events={label_item['warning_event_count']}, "
            f"rooms={','.join(str(v) for v in label_item['room_ids']) or 'none'}, "
            f"dominant causes={', '.join(label_item['dominant_causes']) or 'none'}."
        )
    lines.extend(["", "## Most Important Notes", ""])
    for note in payload["headline_findings"]:
        lines.append(f"- {note}")
    return "\n".join(lines)


def _build_presentation_duplicate_markdown(sequence_id: str, payload: Dict[str, Any]) -> str:
    summary = payload.get("summary", {})
    lines = [
        f"# Presentation Duplicate Summary: {sequence_id}",
        "",
        "## Headline",
        "",
        f"- Raw warning events / candidate pairs: {summary.get('raw_warning_event_count', 0)} / {summary.get('raw_candidate_pair_count', 0)}",
        (
            f"- Presentation categories: conservative={summary.get('presentation_conservative_event_count', 0)} events "
            f"({summary.get('presentation_conservative_pair_count', 0)} pairs), mixed={summary.get('presentation_mixed_event_count', 0)} "
            f"({summary.get('presentation_mixed_pair_count', 0)} pairs), suspicious={summary.get('presentation_suspicious_event_count', 0)} "
            f"({summary.get('presentation_suspicious_pair_count', 0)} pairs)"
        ),
        (
            f"- Closer-attention subset: {summary.get('closer_attention_event_count', 0)} events / "
            f"{summary.get('closer_attention_pair_count', 0)} pairs"
        ),
        f"- Pre-existing pair share in the raw warnings: {summary.get('raw_preexisting_pair_fraction', 0.0):.3f}",
        f"- Scope note: {payload.get('categorization_note')}",
        "",
        "## Teacher-Facing Interpretation",
        "",
    ]
    for note in payload.get("headline_findings", []):
        lines.append(f"- {note}")
    lines.extend(["", "## Rooms / Labels To Discuss", ""])
    for room_item in payload.get("rooms_to_discuss", [])[:5]:
        if int(room_item.get("closer_attention_pair_count", 0)) <= 0:
            continue
        lines.append(
            "- "
            f"Room {room_item['room_id']}: closer-attention pairs={room_item['closer_attention_pair_count']}, "
            f"low-priority pairs={room_item['low_priority_pair_count']}, "
            f"top labels={', '.join(room_item['top_labels']) or 'none'}."
        )
    for label_item in payload.get("labels_to_discuss", [])[:8]:
        lines.append(
            "- "
            f"{label_item['label']}: closer-attention clusters="
            f"{int(label_item.get('presentation_breakdown', {}).get('presentation_suspicious', 0)) + int(label_item.get('presentation_breakdown', {}).get('presentation_mixed', 0))}, "
            f"closer-attention pairs={label_item.get('closer_attention_pair_count', 0)}, rooms={','.join(str(v) for v in label_item['room_ids']) or 'none'}."
        )
    lines.extend(["", "## Lower-Priority Recurring Warnings", ""])
    for label_item in payload.get("low_priority_labels", [])[:8]:
        lines.append(
            "- "
            f"{label_item['label']}: conservative clusters={label_item.get('presentation_breakdown', {}).get('presentation_conservative', 0)}, "
            f"low-priority pairs={label_item.get('low_priority_pair_count', 0)}, rooms={','.join(str(v) for v in label_item['room_ids']) or 'none'}."
        )
    if payload.get("recurring_low_priority_signatures"):
        lines.extend(["", "## Repeated Signatures", ""])
        for item in payload["recurring_low_priority_signatures"][:8]:
            lines.append(
                "- "
                f"Room {item['room_id']} `{item['label']}` ids={item['object_ids']}: repeated across "
                f"{item['repeated_warning_event_count']} warning events."
            )
    lines.extend(["", "## Strongest Closer-Attention Events", ""])
    for item in payload.get("strongest_events_to_discuss", [])[:6]:
        lines.append(
            "- "
            f"Event #{item['event_id']} at frame {item['frame_idx']}: category={item['presentation_category']}; "
            f"rooms={','.join(str(v) for v in item['matched_room_ids']) or 'unknown'}; "
            f"closer-attention labels={', '.join(item['top_closer_attention_labels']) or 'none'}; "
            f"low-priority labels={', '.join(item['top_low_priority_labels']) or 'none'}."
        )
    lines.extend(["", "## Category Definitions", ""])
    for key, note in payload.get("category_definitions", {}).items():
        lines.append(f"- `{key}`: {note}")
    return "\n".join(lines)


def _build_report_presentation_duplicate_section(payload: Dict[str, Any]) -> str:
    summary = payload.get("summary", {})
    lines = [
        "## Presentation-Aware Duplicate Notes",
        "",
        (
            f"- Raw duplicate-object warning events / candidate pairs: {summary.get('raw_warning_event_count', 0)} / "
            f"{summary.get('raw_candidate_pair_count', 0)}"
        ),
        (
            f"- Presentation categories: conservative={summary.get('presentation_conservative_event_count', 0)} events "
            f"({summary.get('presentation_conservative_pair_count', 0)} pairs), mixed={summary.get('presentation_mixed_event_count', 0)} "
            f"({summary.get('presentation_mixed_pair_count', 0)} pairs), suspicious={summary.get('presentation_suspicious_event_count', 0)} "
            f"({summary.get('presentation_suspicious_pair_count', 0)} pairs)"
        ),
        (
            f"- Events that deserve closer attention if asked: {summary.get('closer_attention_event_count', 0)} "
            f"({summary.get('closer_attention_pair_count', 0)} pairs across mixed+suspicious categories)"
        ),
    ]
    labels_to_discuss = payload.get("labels_to_discuss", [])
    if labels_to_discuss:
        lines.append(
            "- Teacher-facing labels to discuss first: "
            + ", ".join(f"`{item['label']}`" for item in labels_to_discuss[:4])
            + "."
        )
    low_priority_labels = payload.get("low_priority_labels", [])
    if low_priority_labels:
        low_priority_only_labels = [
            item for item in low_priority_labels if int(item.get("closer_attention_pair_count", 0)) == 0
        ]
        chosen_low_priority_labels = low_priority_only_labels or low_priority_labels
        lines.append(
            "- Lower-priority recurring labels to de-emphasize: "
            + ", ".join(f"`{item['label']}`" for item in chosen_low_priority_labels[:4])
            + "."
        )
    rooms_to_discuss = [item for item in payload.get("rooms_to_discuss", []) if item.get("closer_attention_pair_count", 0) > 0]
    if rooms_to_discuss:
        lines.append(
            "- Rooms most worth discussing if a duplicate question comes up: "
            + ", ".join(f"room {item['room_id']}" for item in rooms_to_discuss[:3])
            + "."
        )
    lines.append(
        "- Presentation note: this is a post-hoc reporting layer over the raw audit, not a proof that the mapper merged or failed to merge correctly."
    )
    return "\n".join(lines)


def _upsert_report_section(report_path: Path, payload: Dict[str, Any]) -> None:
    if not report_path.exists():
        return
    content = report_path.read_text(encoding="utf-8")
    start_marker = "<!-- presentation-duplicate-summary:start -->"
    end_marker = "<!-- presentation-duplicate-summary:end -->"
    section = "\n".join(
        [
            start_marker,
            _build_report_presentation_duplicate_section(payload),
            end_marker,
            "",
        ]
    )
    if start_marker in content and end_marker in content:
        prefix, remainder = content.split(start_marker, 1)
        _, suffix = remainder.split(end_marker, 1)
        content = prefix.rstrip() + "\n\n" + section + suffix.lstrip("\n")
    else:
        anchor = "## What Is Reused / Merged During Revisit"
        if anchor in content:
            content = content.replace(anchor, section + anchor, 1)
        else:
            content = content.rstrip() + "\n\n" + section
    _write_text(report_path, content)


def _build_merge_markdown(sequence_id: str, payload: Dict[str, Any]) -> str:
    lines = [
        f"# Revisit Object Merge Audit: {sequence_id}",
        "",
        "## Event-Centered Highlights",
        "",
    ]
    for event in payload["top_events"]:
        lines.append(
            "- "
            f"Event #{event['event_id']} at frame {event['frame_idx']}: rooms={','.join(str(v) for v in event['matched_room_ids']) or 'unknown'}, "
            f"retained/merged/added/removed={event['retained_object_count']}/{event['merged_object_count']}/{event['added_object_count']}/{event['removed_object_count']}, "
            f"duplicate assessment={event['duplicate_assessment']}, presentation category={event.get('presentation_duplicate_category', 'n/a')}, verdict={event['verdict']}."
        )
        lines.append(f"  Reason: {event['verdict_reason']}")
        if event.get("presentation_duplicate_note"):
            lines.append(f"  Presentation note: {event['presentation_duplicate_note']}")
    lines.extend(
        [
            "",
            "## Scope Notes",
            "",
            "- This is a Stage A audit over dataset-pose replays, not an online SLAM backend evaluation.",
            "- Duplicate-object verdicts remain heuristic and room-local.",
            "- Object reuse/fusion evidence here is event-centered and uses before/effect-frame comparisons only.",
        ]
    )
    return "\n".join(lines)


def _build_sequence_audit(sequence: SequenceAudit) -> Dict[str, Any]:
    diagnostics = sequence.diagnostics
    event_audits = {}
    duplicate_warning_events = []
    room_stats: Dict[int, Dict[str, Any]] = defaultdict(lambda: {
        "warning_event_count": 0,
        "candidate_pair_count": 0,
        "labels": Counter(),
        "causes": Counter(),
        "assessments": Counter(),
    })
    label_stats: Dict[str, Dict[str, Any]] = defaultdict(lambda: {
        "candidate_pair_count": 0,
        "warning_event_ids": set(),
        "room_ids": set(),
        "causes": Counter(),
    })
    cause_counts = Counter()
    candidate_pair_count = 0
    preexisting_pair_count = 0

    for event in diagnostics:
        audit = _event_duplicate_audit(sequence, event)
        event_audits[int(event["event_id"])] = audit
        if not audit["warning_flag"]:
            continue
        duplicate_warning_events.append(event)
        candidate_pair_count += int(audit["candidate_pair_count"])
        preexisting_pair_count += int(audit["preexisting_pair_count"])
        cause_counts.update(audit.get("cause_counts", {}))
        for room_id in event.get("matched_room_ids", []):
            room_stats[int(room_id)]["warning_event_count"] += 1
            room_stats[int(room_id)]["candidate_pair_count"] += int(audit["candidate_pair_count"])
            room_stats[int(room_id)]["assessments"][audit["overall_assessment"]] += 1
        for cluster in audit.get("duplicate_clusters", []):
            room_stats[int(cluster["room_id"])]["labels"][cluster["label"]] += int(cluster["pair_count"])
            room_stats[int(cluster["room_id"])]["causes"][cluster["primary_cause"]] += 1
            label_stats[cluster["label"]]["candidate_pair_count"] += int(cluster["pair_count"])
            label_stats[cluster["label"]]["warning_event_ids"].add(int(event["event_id"]))
            label_stats[cluster["label"]]["room_ids"].add(int(cluster["room_id"]))
            label_stats[cluster["label"]]["causes"][cluster["primary_cause"]] += 1

    event_assessment_counter = Counter(
        audit["overall_assessment"] for audit in event_audits.values() if audit["warning_flag"]
    )
    rooms_payload = []
    for room_id, stats in sorted(
        room_stats.items(),
        key=lambda item: (
            -int(item[1]["warning_event_count"]),
            -int(item[1]["candidate_pair_count"]),
            int(item[0]),
        ),
    ):
        rooms_payload.append(
            {
                "room_id": int(room_id),
                "warning_event_count": int(stats["warning_event_count"]),
                "candidate_pair_count": int(stats["candidate_pair_count"]),
                "top_labels": [label for label, _ in stats["labels"].most_common(5)],
                "dominant_causes": [cause for cause, _ in stats["causes"].most_common(3)],
                "assessment_breakdown": {key: int(value) for key, value in stats["assessments"].items()},
            }
        )

    labels_payload = []
    for label, stats in sorted(
        label_stats.items(),
        key=lambda item: (
            -int(item[1]["candidate_pair_count"]),
            str(item[0]).lower(),
        ),
    ):
        labels_payload.append(
            {
                "label": label,
                "candidate_pair_count": int(stats["candidate_pair_count"]),
                "warning_event_count": int(len(stats["warning_event_ids"])),
                "room_ids": sorted(int(room_id) for room_id in stats["room_ids"]),
                "dominant_causes": [cause for cause, _ in stats["causes"].most_common(3)],
            }
        )

    headline_findings = []
    if candidate_pair_count > 0:
        headline_findings.append(
            f"{preexisting_pair_count}/{candidate_pair_count} duplicate-warning pairs were already present before the revisit effect frame."
        )
    if rooms_payload:
        room_item = rooms_payload[0]
        headline_findings.append(
            f"Room {room_item['room_id']} is the biggest warning hotspot, driven mostly by {', '.join(room_item['top_labels'][:3]) or 'mixed labels'}."
        )
    if labels_payload:
        label_item = labels_payload[0]
        headline_findings.append(
            f"The single most common label in duplicate warnings is `{label_item['label']}`, which appears in {label_item['candidate_pair_count']} close-proximity pairs."
        )
    if event_assessment_counter["likely_conservative"] > 0:
        headline_findings.append(
            f"{event_assessment_counter['likely_conservative']} warning events look mostly conservative rather than revisit-created failures."
        )
    if event_assessment_counter["likely_suspicious"] > 0:
        headline_findings.append(
            f"{event_assessment_counter['likely_suspicious']} warning events still look genuinely suspicious and deserve manual spotlight review."
        )

    event_rows = []
    for event in _sort_diagnostics(diagnostics):
        audit = event_audits[int(event["event_id"])]
        event_rows.append(
            {
                "event_id": int(event["event_id"]),
                "frame_idx": int(event["frame_idx"]),
                "teacher_evidence_score": float(event.get("teacher_evidence_score", 0.0)),
                "matched_room_ids": ",".join(str(v) for v in event.get("matched_room_ids", [])),
                "warning_flag": bool(audit["warning_flag"]),
                "overall_assessment": audit["overall_assessment"],
                "candidate_pair_count": int(audit["candidate_pair_count"]),
                "cluster_count": int(audit["cluster_count"]),
                "preexisting_pair_count": int(audit["preexisting_pair_count"]),
                "new_or_changed_pair_count": int(audit["new_or_changed_pair_count"]),
                "primary_causes": ",".join(audit.get("primary_causes", [])),
                "retained_object_count": int(event.get("object_fusion_updates", {}).get("retained_object_count", 0)),
                "merged_object_count": int(event.get("object_fusion_updates", {}).get("merged_object_count", 0)),
                "added_object_count": int(event.get("object_fusion_updates", {}).get("added_object_count", 0)),
                "removed_object_count": int(event.get("object_fusion_updates", {}).get("removed_object_count", 0)),
            }
        )

    summary_payload = {
        "grouped_revisit_events": int(len(diagnostics)),
        "duplicate_warning_event_count": int(len(duplicate_warning_events)),
        "duplicate_candidate_pair_count": int(candidate_pair_count),
        "preexisting_pair_fraction": 0.0 if candidate_pair_count == 0 else round(preexisting_pair_count / candidate_pair_count, 3),
        "likely_conservative_event_count": int(event_assessment_counter["likely_conservative"]),
        "mixed_event_count": int(event_assessment_counter["mixed"]),
        "likely_suspicious_event_count": int(event_assessment_counter["likely_suspicious"]),
    }

    return {
        "sequence_id": sequence.sequence_id,
        "summary": summary_payload,
        "headline_findings": headline_findings,
        "cause_breakdown": [
            {"cause": cause, "cluster_count": int(count)}
            for cause, count in cause_counts.most_common()
        ],
        "rooms_with_most_duplicate_warnings": rooms_payload,
        "labels_with_most_duplicate_warnings": labels_payload,
        "event_duplicate_audits": [event_audits[int(event["event_id"])] for event in _sort_diagnostics(diagnostics)],
        "csv_rows": event_rows,
    }


def _build_revisit_merge_audit(sequence: SequenceAudit, duplicate_payload: Dict[str, Any], top_k: int) -> Dict[str, Any]:
    diagnostics_by_id = {int(item["event_id"]): item for item in sequence.diagnostics}
    top_events = []
    ranking_rows = []

    for event in _sort_diagnostics(sequence.diagnostics):
        audit = next(
            item for item in duplicate_payload["event_duplicate_audits"] if int(item["event_id"]) == int(event["event_id"])
        )
        spotlight_score = _event_spotlight_score(event, audit)
        ranking_rows.append((spotlight_score, event, audit))

    ranking_rows.sort(key=lambda item: (-float(item[0]), int(item[1]["frame_idx"])))
    for spotlight_score, event, audit in ranking_rows[:top_k]:
        updates = event.get("object_fusion_updates", {})
        per_room = updates.get("per_room", [])
        presentation_event = audit.get("presentation_duplicate", {})
        room_local_audits = []
        for room_item in per_room:
            room_id = int(room_item["room_id"])
            room_clusters = [
                cluster for cluster in audit.get("duplicate_clusters", []) if int(cluster["room_id"]) == room_id
            ]
            suspicious_clusters = [
                cluster for cluster in room_clusters if cluster["assessment"] != "likely_conservative"
            ]
            retained_count = len(room_item.get("retained_object_ids", []))
            merged_count = len(room_item.get("merged_object_ids", []))
            added_count = len(room_item.get("added_object_ids", []))
            removed_count = len(room_item.get("removed_object_ids", []))
            if room_clusters and suspicious_clusters:
                verdict = "ambiguous"
                reason = "room reuse is clear, but unresolved duplicate clusters still muddy the object story"
            elif merged_count > 0 or retained_count > 0:
                verdict = "convincing_reuse_or_fusion"
                reason = "room-local object ids were retained or explicitly merged without suspicious duplicate clusters"
            elif added_count > 0 and not room_clusters:
                verdict = "partial_reuse_with_new_objects"
                reason = "the revisit kept room identity and also added fresh objects without duplicate warnings"
            else:
                verdict = "ambiguous"
                reason = "room identity was reused, but the object-level evidence is thin"
            room_local_audits.append(
                {
                    "room_id": room_id,
                    "retained_object_ids": room_item.get("retained_object_ids", []),
                    "merged_object_ids": room_item.get("merged_object_ids", []),
                    "merged_into_object_ids": room_item.get("merged_into_object_ids", []),
                    "added_object_ids": room_item.get("added_object_ids", []),
                    "removed_object_ids": room_item.get("removed_object_ids", []),
                    "merge_pairs": room_item.get("merge_pairs", []),
                    "suspected_duplicate_clusters": room_clusters,
                    "verdict": verdict,
                    "reason": reason,
                }
            )

        if any(item["verdict"] == "ambiguous" for item in room_local_audits):
            verdict = "ambiguous"
            verdict_reason = "at least one revisited room still has unresolved object-duplicate ambiguity"
        elif any(item["verdict"] == "partial_reuse_with_new_objects" for item in room_local_audits):
            verdict = "partial_reuse_with_new_objects"
            verdict_reason = "the event shows reuse, but part of the object story is still additive rather than fused"
        else:
            verdict = "convincing_reuse_or_fusion"
            verdict_reason = "matched room ids are reused and the room-local object evidence reads cleanly"

        top_events.append(
            {
                "event_id": int(event["event_id"]),
                "frame_idx": int(event["frame_idx"]),
                "effect_frame_idx": int(event["effect_frame_idx"]),
                "spotlight_score": float(spotlight_score),
                "teacher_evidence_score": float(event.get("teacher_evidence_score", 0.0)),
                "matched_room_ids": [int(room_id) for room_id in event.get("matched_room_ids", [])],
                "retained_object_count": int(updates.get("retained_object_count", 0)),
                "merged_object_count": int(updates.get("merged_object_count", 0)),
                "added_object_count": int(updates.get("added_object_count", 0)),
                "removed_object_count": int(updates.get("removed_object_count", 0)),
                "duplicate_assessment": audit["overall_assessment"],
                "duplicate_primary_causes": audit.get("primary_causes", []),
                "presentation_duplicate_category": presentation_event.get("category"),
                "presentation_duplicate_note": presentation_event.get("teacher_note"),
                "presentation_duplicate_labels": presentation_event.get("top_closer_attention_labels", []),
                "room_local_audits": room_local_audits,
                "verdict": verdict,
                "verdict_reason": verdict_reason,
            }
        )

    return {
        "sequence_id": sequence.sequence_id,
        "top_events": top_events,
    }


def _comparison_recommendation(
    sequence_payloads: Sequence[Dict[str, Any]],
    spotlight_rankings: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    by_sequence = {item["sequence_id"]: item for item in sequence_payloads}
    ordered = sorted(
        sequence_payloads,
        key=lambda item: (
            -float(item["clarity_score"]),
            -float(item["duplicate_safety_score"]),
            str(item["sequence_id"]),
        ),
    )
    safer = sorted(
        sequence_payloads,
        key=lambda item: (
            -float(item["duplicate_safety_score"]),
            -float(item["clarity_score"]),
            str(item["sequence_id"]),
        ),
    )
    best_spotlight = spotlight_rankings[0] if spotlight_rankings else None
    main_demo = ordered[0]["sequence_id"] if ordered else None
    backup_demo = None
    if safer:
        backup_demo = safer[0]["sequence_id"]
        if backup_demo == main_demo and len(safer) > 1:
            backup_demo = safer[1]["sequence_id"]
    return {
        "main_demo_sequence": main_demo,
        "backup_demo_sequence": backup_demo,
        "best_single_revisit_spotlight": best_spotlight,
        "per_sequence": by_sequence,
    }


def _build_sequence_comparison(sequence_payloads: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    ordered = sorted(sequence_payloads, key=lambda item: str(item["sequence_id"]))
    if len(ordered) != 2:
        raise ValueError("The current comparison writer expects exactly two sequences.")
    a_item, b_item = ordered
    if a_item["clarity_score"] >= b_item["clarity_score"]:
        clearer = a_item["sequence_id"]
    else:
        clearer = b_item["sequence_id"]
    if a_item["duplicate_safety_score"] >= b_item["duplicate_safety_score"]:
        safer = a_item["sequence_id"]
    else:
        safer = b_item["sequence_id"]
    return {
        "sequences": ordered,
        "which_tells_revisit_story_more_clearly": clearer,
        "which_looks_safer_for_object_duplication": safer,
    }


def _build_comparison_markdown(comparison: Dict[str, Any], recommendation: Dict[str, Any]) -> str:
    seq_a, seq_b = comparison["sequences"]
    lines = [
        f"# Sequence Comparison: {seq_a['sequence_id']} vs {seq_b['sequence_id']}",
        "",
        "## Headline",
        "",
        f"- Clearer revisit/reuse story: {comparison['which_tells_revisit_story_more_clearly']}",
        f"- Safer with respect to object duplication: {comparison['which_looks_safer_for_object_duplication']}",
        f"- Recommended main demo: {recommendation['main_demo_sequence']}",
        f"- Recommended backup demo: {recommendation['backup_demo_sequence']}",
        "",
        "## Sequence Notes",
        "",
    ]
    for item in comparison["sequences"]:
        lines.append(
            "- "
            f"{item['sequence_id']}: clarity_score={item['clarity_score']:.2f}, "
            f"duplicate_safety_score={item['duplicate_safety_score']:.2f}, "
            f"strong_revisits={item['strong_revisit_count']}, "
            f"duplicate warning events={item['duplicate_warning_event_count']}, "
            f"closer-attention warning events={item.get('closer_attention_event_count', item['likely_suspicious_event_count'])}, "
            f"presentation-suspicious events={item['likely_suspicious_event_count']}, "
            f"top reason={item['comparison_note']}."
        )
    if recommendation.get("best_single_revisit_spotlight"):
        spotlight = recommendation["best_single_revisit_spotlight"]
        lines.extend(
            [
                "",
                "## Best Single Spotlight",
                "",
                "- "
                f"{spotlight['sequence_id']} event #{spotlight['event_id']} at frame {spotlight['frame_idx']}: "
                f"spotlight_score={spotlight['spotlight_score']:.2f}; "
                f"{spotlight['reason']}",
            ]
        )
    return "\n".join(lines)


def _build_spotlight_markdown(rankings: Sequence[Dict[str, Any]], recommendation: Dict[str, Any]) -> str:
    lines = [
        "# Spotlight Ranking",
        "",
        "## Ranked Events",
        "",
    ]
    for item in rankings:
        lines.append(
            "- "
            f"Rank {item['rank']}: {item['sequence_id']} event #{item['event_id']} at frame {item['frame_idx']} "
            f"(spotlight_score={item['spotlight_score']:.2f}, presentation={item.get('presentation_duplicate_category', 'n/a')}) | {item['reason']}"
        )
    lines.extend(
        [
            "",
            "## Recommendations",
            "",
            f"- Best main demo sequence: {recommendation['main_demo_sequence']}",
            f"- Best backup demo sequence: {recommendation['backup_demo_sequence']}",
        ]
    )
    if recommendation.get("best_single_revisit_spotlight"):
        spotlight = recommendation["best_single_revisit_spotlight"]
        lines.append(
            f"- Best single revisit spotlight event: {spotlight['sequence_id']} event #{spotlight['event_id']} at frame {spotlight['frame_idx']}"
        )
    return "\n".join(lines)


def _summarize_sequence_for_comparison(
    sequence_id: str,
    duplicate_payload: Dict[str, Any],
    merge_payload: Dict[str, Any],
) -> Dict[str, Any]:
    summary = duplicate_payload["summary"]
    presentation_summary = duplicate_payload.get("presentation_duplicate_summary", {}).get("summary", {})
    top_events = merge_payload["top_events"]
    top3_avg = 0.0
    if top_events:
        top3_avg = sum(float(item["spotlight_score"]) for item in top_events[:3]) / min(3, len(top_events))
    clarity_score = round(
        top3_avg
        + min(float(summary["grouped_revisit_events"]), 120.0) * 0.02
        + min(float(summary["duplicate_warning_event_count"]), 100.0) * 0.0,
        3,
    )
    duplicate_safety_score = round(
        10.0
        - float(presentation_summary.get("presentation_suspicious_event_count", summary["likely_suspicious_event_count"])) * 0.18
        - float(presentation_summary.get("presentation_mixed_event_count", summary["mixed_event_count"])) * 0.08
        - float(summary["duplicate_warning_event_count"]) * 0.03
        + float(presentation_summary.get("presentation_conservative_event_count", summary["likely_conservative_event_count"])) * 0.04
        + float(summary["preexisting_pair_fraction"]) * 2.0,
        3,
    )
    warning_count = max(1, int(summary["duplicate_warning_event_count"]))
    suspicious_ratio = float(
        presentation_summary.get("presentation_suspicious_event_count", summary["likely_suspicious_event_count"])
    ) / float(warning_count)
    conservative_ratio = float(
        presentation_summary.get("presentation_conservative_event_count", summary["likely_conservative_event_count"])
    ) / float(warning_count)
    if conservative_ratio >= 0.55:
        comparison_note = "warning count is high, but most flagged pairs look pre-existing or conservative"
    elif suspicious_ratio >= 0.75 and warning_count <= 30:
        comparison_note = "fewer warnings overall, but most of them stay concentrated in riskier object clusters"
    else:
        comparison_note = "warning mix is balanced enough to require room-local review rather than a single headline"
    return {
        "sequence_id": sequence_id,
        "strong_revisit_count": int(
            sum(float(item.get("teacher_evidence_score", 0.0)) >= 6.0 for item in duplicate_payload["event_duplicate_audits"])
        ),
        "duplicate_warning_event_count": int(summary["duplicate_warning_event_count"]),
        "likely_suspicious_event_count": int(
            presentation_summary.get("presentation_suspicious_event_count", summary["likely_suspicious_event_count"])
        ),
        "likely_conservative_event_count": int(
            presentation_summary.get("presentation_conservative_event_count", summary["likely_conservative_event_count"])
        ),
        "presentation_mixed_event_count": int(
            presentation_summary.get("presentation_mixed_event_count", summary["mixed_event_count"])
        ),
        "closer_attention_event_count": int(
            presentation_summary.get(
                "closer_attention_event_count",
                summary["likely_suspicious_event_count"] + summary["mixed_event_count"],
            )
        ),
        "clarity_score": clarity_score,
        "duplicate_safety_score": duplicate_safety_score,
        "comparison_note": comparison_note,
    }


def run_targeted_audit(
    output_root: Path,
    sequence_ids: Sequence[str],
    comparison_name: str,
    per_sequence_top_k: int,
    spotlight_rank_limit: int,
) -> Dict[str, Any]:
    comparison_dir = output_root / "_multi_sequence" / comparison_name
    comparison_dir.mkdir(parents=True, exist_ok=True)
    sequence_results = []
    spotlight_candidates = []

    for sequence_id in sequence_ids:
        sequence = SequenceAudit(output_root=output_root, sequence_id=sequence_id)
        duplicate_payload = _build_sequence_audit(sequence)
        presentation_payload = _build_presentation_duplicate_summary(sequence, duplicate_payload)
        duplicate_payload["presentation_duplicate_summary"] = presentation_payload
        merge_payload = _build_revisit_merge_audit(
            sequence=sequence,
            duplicate_payload=duplicate_payload,
            top_k=per_sequence_top_k,
        )
        duplicate_json = sequence.log_dir / "object_duplicate_audit.json"
        duplicate_csv = sequence.log_dir / "object_duplicate_audit.csv"
        duplicate_md = sequence.log_dir / "object_duplicate_audit.md"
        presentation_json = sequence.log_dir / "presentation_duplicate_summary.json"
        presentation_md = sequence.log_dir / "presentation_duplicate_summary.md"
        merge_json = sequence.log_dir / "revisit_object_merge_audit.json"
        merge_md = sequence.log_dir / "revisit_object_merge_audit.md"
        _write_json(duplicate_json, duplicate_payload)
        _write_csv(
            duplicate_csv,
            duplicate_payload["csv_rows"],
            fieldnames=[
                "event_id",
                "frame_idx",
                "teacher_evidence_score",
                "matched_room_ids",
                "warning_flag",
                "overall_assessment",
                "presentation_category",
                "presentation_closer_attention",
                "candidate_pair_count",
                "cluster_count",
                "preexisting_pair_count",
                "new_or_changed_pair_count",
                "primary_causes",
                "retained_object_count",
                "merged_object_count",
                "added_object_count",
                "removed_object_count",
            ],
        )
        _write_text(duplicate_md, _build_duplicate_markdown(sequence_id, duplicate_payload))
        _write_json(presentation_json, presentation_payload)
        _write_text(presentation_md, _build_presentation_duplicate_markdown(sequence_id, presentation_payload))
        _write_json(merge_json, merge_payload)
        _write_text(merge_md, _build_merge_markdown(sequence_id, merge_payload))
        _upsert_report_section(sequence.sequence_dir / "report.md", presentation_payload)

        for event in merge_payload["top_events"]:
            reason_parts = []
            if event["verdict"] == "convincing_reuse_or_fusion":
                reason_parts.append("clean room-local reuse/fusion evidence")
            elif event["verdict"] == "partial_reuse_with_new_objects":
                reason_parts.append("clear room reuse with some additive object growth")
            else:
                reason_parts.append("strong revisit signal but still some object-level ambiguity")
            if event["duplicate_assessment"] == "likely_conservative":
                reason_parts.append("duplicate warnings look mostly conservative")
            elif event["duplicate_assessment"] == "likely_suspicious":
                reason_parts.append("duplicate warnings still need caution")
            if event.get("presentation_duplicate_category") == "presentation_conservative":
                reason_parts.append("presentation-aware duplicate view is low-priority")
            elif event.get("presentation_duplicate_category") == "presentation_suspicious":
                reason_parts.append("presentation-aware duplicate view still deserves closer attention")
            elif event.get("presentation_duplicate_category") == "presentation_mixed":
                reason_parts.append("presentation-aware duplicate view is mixed rather than headline-level")
            spotlight_candidates.append(
                {
                    "sequence_id": sequence_id,
                    "event_id": int(event["event_id"]),
                    "frame_idx": int(event["frame_idx"]),
                    "spotlight_score": float(event["spotlight_score"]),
                    "teacher_evidence_score": float(event["teacher_evidence_score"]),
                    "matched_room_ids": list(event["matched_room_ids"]),
                    "duplicate_assessment": event["duplicate_assessment"],
                    "presentation_duplicate_category": event.get("presentation_duplicate_category"),
                    "verdict": event["verdict"],
                    "reason": "; ".join(reason_parts),
                }
            )

        sequence_results.append(
            {
                "sequence_id": sequence_id,
                "duplicate_payload": duplicate_payload,
                "merge_payload": merge_payload,
                "comparison_summary": _summarize_sequence_for_comparison(sequence_id, duplicate_payload, merge_payload),
                "output_paths": {
                    "object_duplicate_audit_json": str(duplicate_json),
                    "object_duplicate_audit_csv": str(duplicate_csv),
                    "object_duplicate_audit_md": str(duplicate_md),
                    "presentation_duplicate_summary_json": str(presentation_json),
                    "presentation_duplicate_summary_md": str(presentation_md),
                    "revisit_object_merge_audit_json": str(merge_json),
                    "revisit_object_merge_audit_md": str(merge_md),
                },
            }
        )

    spotlight_candidates.sort(
        key=lambda item: (
            -float(item["spotlight_score"]),
            -float(item["teacher_evidence_score"]),
            str(item["sequence_id"]),
            int(item["frame_idx"]),
        )
    )
    spotlight_rankings = []
    for rank, item in enumerate(spotlight_candidates[:spotlight_rank_limit], start=1):
        spotlight_rankings.append({**item, "rank": int(rank)})

    comparison_payload = _build_sequence_comparison(
        [item["comparison_summary"] for item in sequence_results]
    )
    recommendation_payload = _comparison_recommendation(
        [item["comparison_summary"] for item in sequence_results],
        spotlight_rankings,
    )

    comparison_json = comparison_dir / "sequence_comparison_00847_vs_00843.json"
    comparison_md = comparison_dir / "sequence_comparison_00847_vs_00843.md"
    spotlight_json = comparison_dir / "spotlight_ranking.json"
    spotlight_md = comparison_dir / "spotlight_ranking.md"
    _write_json(
        comparison_json,
        {
            **comparison_payload,
            "recommendation": recommendation_payload,
        },
    )
    _write_text(comparison_md, _build_comparison_markdown(comparison_payload, recommendation_payload))
    _write_json(
        spotlight_json,
        {
            "rankings": spotlight_rankings,
            "recommendation": recommendation_payload,
        },
    )
    _write_text(spotlight_md, _build_spotlight_markdown(spotlight_rankings, recommendation_payload))

    return {
        "output_root": str(output_root),
        "comparison_dir": str(comparison_dir),
        "sequence_results": sequence_results,
        "comparison_json": str(comparison_json),
        "comparison_md": str(comparison_md),
        "spotlight_json": str(spotlight_json),
        "spotlight_md": str(spotlight_md),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Targeted Stage A duplicate/object-merge audit for existing sequence outputs")
    parser.add_argument("--output-root", default="stage_a_outputs1", help="Root folder containing per-sequence Stage A outputs")
    parser.add_argument("--seqs", nargs="+", default=DEFAULT_TARGET_SEQUENCES, help="Target sequence ids to audit")
    parser.add_argument("--comparison-name", default="targeted_stage_a_audit", help="Subdirectory under _multi_sequence for head-to-head outputs")
    parser.add_argument("--top-events-per-sequence", default=4, type=int, help="How many strong event-centered merge audits to keep per sequence")
    parser.add_argument("--spotlight-rank-limit", default=8, type=int, help="How many cross-sequence spotlight events to rank")
    args = parser.parse_args()

    result = run_targeted_audit(
        output_root=Path(args.output_root),
        sequence_ids=args.seqs,
        comparison_name=args.comparison_name,
        per_sequence_top_k=int(args.top_events_per_sequence),
        spotlight_rank_limit=int(args.spotlight_rank_limit),
    )
    manifest = {
        "output_root": result["output_root"],
        "comparison_dir": result["comparison_dir"],
        "comparison_json": result["comparison_json"],
        "comparison_md": result["comparison_md"],
        "spotlight_json": result["spotlight_json"],
        "spotlight_md": result["spotlight_md"],
        "sequence_outputs": {
            item["sequence_id"]: item["output_paths"] for item in result["sequence_results"]
        },
    }
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
