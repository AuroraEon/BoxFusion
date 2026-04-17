from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from boxfusion.room_commit_diagnosis import (
    build_room_commit_diagnosis,
    default_output_paths,
    load_scene_artifacts,
    render_room_commit_diagnosis_markdown,
    write_json,
    write_markdown,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Explain why a replay run did or did not produce committed/public rooms."
    )
    parser.add_argument(
        "input_path",
        help="Scene root, logs directory, logs/summary.json, or online_topology_lifecycle_v0_1.json.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional explicit JSON output path. Defaults beside the resolved logs directory.",
    )
    parser.add_argument(
        "--md-out",
        type=Path,
        default=None,
        help="Optional explicit markdown output path. Defaults beside the resolved logs directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scene_artifacts = load_scene_artifacts(Path(args.input_path))
    diagnosis = build_room_commit_diagnosis(scene_artifacts)
    markdown = render_room_commit_diagnosis_markdown(diagnosis)

    default_json_out, default_md_out = default_output_paths(Path(scene_artifacts["log_dir"]))
    json_out = Path(args.json_out) if args.json_out is not None else default_json_out
    md_out = Path(args.md_out) if args.md_out is not None else default_md_out

    write_json(json_out, diagnosis)
    write_markdown(md_out, markdown)

    print(f"sequence_id: {diagnosis.get('sequence_id')}")
    print(f"diagnosis_category: {diagnosis.get('diagnosis', {}).get('category')}")
    print(f"explanation: {diagnosis.get('diagnosis', {}).get('explanation')}")
    print(f"public_topology_room_count: {diagnosis.get('counts', {}).get('public_topology_room_count')}")
    print(f"runtime_internal_committed_room_count: {diagnosis.get('counts', {}).get('runtime_internal_committed_room_count')}")
    print(f"candidate_room_formed_count: {diagnosis.get('counts', {}).get('candidate_room_formed_count')}")
    print(f"candidate_complete_ever_count: {diagnosis.get('counts', {}).get('candidate_complete_ever_count')}")
    print(f"room_transition_event_count: {diagnosis.get('room_transition_evidence', {}).get('room_transition_event_count')}")
    print(f"Diagnosis JSON: {json_out}")
    print(f"Diagnosis markdown: {md_out}")


if __name__ == "__main__":
    main()
