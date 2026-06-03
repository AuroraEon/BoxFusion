#!/usr/bin/env python3
"""Audit task14a object-navigation experiment readiness."""

from __future__ import annotations

import argparse
from pathlib import Path

from object_nav_common import (
    COMMITTED_MODEL_JSON,
    COMMITTED_SNAPSHOT_JSON,
    FINAL_VECTOR_JSON,
    TASK_DIR,
    TOPOLOGY_JSON,
    TOPOLOGY_QUERY_REPORT_JSON,
    build_candidate_index_data,
    embedding_vector_status,
    inventory_from_index,
    markdown_table,
    route_eligible,
    slim_objects,
    write_json,
)


def runtime_feasibility_rows(index: dict) -> list[dict]:
    rows = []
    route_status = index.get("route_asset_status", {})
    for obj in index.get("objects", []):
        if obj.get("floor_id") != "floor_2":
            continue
        suitable = route_eligible(obj) and not obj.get("is_uncertain_floor_assignment")
        reasons = []
        if suitable:
            reasons.append("suitable for later target-room-first runtime; map and route bridge are present")
        else:
            if obj.get("is_noisy_label"):
                reasons.append("noisy label")
            if obj.get("is_uncertain_floor_assignment"):
                reasons.append("uncertain floor assignment")
            if obj.get("route_availability_status") != "floor2_runtime_candidate":
                reasons.append(obj.get("route_availability_status"))
        rows.append(
            {
                "object_id": obj.get("object_id"),
                "label": obj.get("label"),
                "room_id": obj.get("room_id"),
                "floor_id": obj.get("floor_id"),
                "confidence": obj.get("confidence"),
                "why": "; ".join(reasons) or "usable only as artifact query case",
                "expected_target_room": obj.get("room_id"),
                "floor_2_map_route_assets_available": route_status.get("available"),
            }
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--index", type=Path, default=TASK_DIR / "object_candidate_index_v0_1.json")
    parser.add_argument("--output-dir", type=Path, default=TASK_DIR)
    args = parser.parse_args()
    if args.index.exists():
        from object_nav_common import load_index

        index = load_index(args.index)
    else:
        index = build_candidate_index_data()
    inventory = inventory_from_index(index)
    embed = embedding_vector_status([COMMITTED_SNAPSHOT_JSON, FINAL_VECTOR_JSON])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / "experiment_corpus_inventory.json", inventory)
    runtime_rows = runtime_feasibility_rows(index)
    artifacts = [TOPOLOGY_JSON, TOPOLOGY_QUERY_REPORT_JSON, COMMITTED_SNAPSHOT_JSON, COMMITTED_MODEL_JSON, FINAL_VECTOR_JSON]
    label_hist = inventory["object_labels_histogram"]
    top_labels = sorted(label_hist.items(), key=lambda kv: (-kv[1], kv[0]))[:15]
    recommended = [
        o
        for o in index.get("objects", [])
        if route_eligible(o) and not o.get("is_uncertain_floor_assignment")
    ]
    (args.output_dir / "task14a_readiness_summary.md").write_text(
        "# Task14a Readiness Summary\n\n"
        "## Inspected Artifacts\n\n"
        + "\n".join(f"- {p}" + ("" if p.exists() else " (missing)") for p in artifacts)
        + "\n\n## Object Counts\n\n"
        f"- Public topology objects: {inventory['total_public_objects']}\n"
        f"- Committed snapshot objects: {inventory['total_committed_objects']}\n"
        f"- Indexed objects: {inventory['indexed_objects']}\n"
        f"- Raw final vector map objects: {index.get('counts', {}).get('raw_final_vector_objects')} "
        f"({index.get('counts', {}).get('raw_final_vector_objects_with_room_id')} room-bound, "
        f"{index.get('counts', {}).get('raw_final_vector_objects_without_room_id')} without room_id)\n\n"
        "## Binding Quality\n\n"
        f"- Objects by floor: {inventory['objects_by_floor']}\n"
        f"- Objects by room: {inventory['objects_by_room']}\n"
        f"- Missing/invalid binding candidates: {len(inventory['missing_binding_candidates'])}\n"
        f"- Uncertain floor assignment candidates: {len(inventory['uncertain_floor_assignment_candidates'])}\n\n"
        "## Label Histogram\n\n"
        + "\n".join(f"- {label}: {count}" for label, count in top_labels)
        + "\n\n## Noisy Or Uncertain Candidates\n\n"
        f"- Noisy label candidates: {len(inventory['noisy_label_candidates'])}\n"
        f"- Labels treated as noisy/surface labels include sky, snow, oyster, floor, ceiling, wall-wood, roof.\n\n"
        "## Floor 2 Runtime Candidate Availability\n\n"
        f"- Floor_2 committed objects: {len(inventory['floor_2_object_candidates'])}\n"
        f"- Route-eligible floor_2 candidates: {len(inventory['route_eligible_candidates'])}\n"
        f"- Current floor_2 route bridge rooms: {index.get('route_asset_status', {}).get('room_sequence')}\n\n"
        "## GT Availability\n\n"
        "Current query cases are artifact-derived self-consistency cases. Semantic accuracy requires aligned HM3DSem/dataset GT or human review.\n\n"
        "## Embedding Provenance\n\n"
        f"- Embedding refs seen: {embed['embedding_refs_seen']}\n"
        f"- Resolved embedding vectors/files found: {embed['embedding_vectors_resolved']}\n"
        "- CLIP/open-vocabulary AUC is unsupported unless object vectors, text-query index, and label GT are exported or recomputed.\n\n"
        "## Approach-Pose Feasibility\n\n"
        "Target-room navigation is feasible for later runtime experiments. Object approach pose reliability is not claimed until free-cell snap and clearance validation artifacts exist.\n\n"
        "## Recommended Runtime Episodes\n\n"
        + markdown_table(slim_objects(recommended), ["object_id", "label", "room_id", "floor_id", "confidence", "floor_assignment_status"], limit=20)
    )
    (args.output_dir / "gt_availability_notes.md").write_text(
        "# GT Availability Notes\n\n"
        "- Artifact-derived query cases can be used for self-consistency evaluation.\n"
        "- True semantic accuracy needs HM3DSem/dataset ground truth or human review aligned to this scene.\n"
        "- HiCo-Nav and HOV-SG prediction outputs must not be used as ground truth.\n"
        "- HOV-SG/HM3DSem-style scene_info or semantic annotations may be usable later only if aligned to 00843-DYehNKdT76V.\n"
    )
    (args.output_dir / "embedding_provenance_notes.md").write_text(
        "# Embedding Provenance Notes\n\n"
        f"- Embedding refs seen in inspected objects: {embed['embedding_refs_seen']}.\n"
        f"- Inline embedding vectors or separate embedding files resolved: {embed['embedding_vectors_resolved']}.\n"
        f"- Candidate embedding-like files: {embed['candidate_embedding_files']}.\n\n"
        "Because embedding_ref values do not resolve to actual vectors in the inspected committed/public outputs, CLIP/open-vocabulary AUC is not currently supported.\n\n"
        "Required for that evaluation:\n"
        "- Object embedding vectors.\n"
        "- Text-query embedding index.\n"
        "- Object label ground truth.\n"
        "- Optional Stage-A embedding export or offline crop embedding recomputation.\n"
    )
    (args.output_dir / "runtime_feasibility_notes.md").write_text(
        "# Runtime Feasibility Notes\n\n"
        "No runtime execution was performed. These are later-task candidates only.\n\n"
        + markdown_table(runtime_rows, ["object_id", "label", "room_id", "floor_id", "confidence", "why", "expected_target_room", "floor_2_map_route_assets_available"])
    )
    print(f"wrote audit outputs in {args.output_dir}")


if __name__ == "__main__":
    main()
