import json
import os
import sys
import tempfile
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.world_model_eval import (
    VariantConfig,
    build_metric_row,
    load_task_sheet,
    transform_topology_payload,
    write_task_sheet_csv,
)


def _sample_task_sheet_payload() -> dict:
    return {
        "version": "0.1",
        "title": "mock",
        "tasks": [
            {
                "task_id": "mock_query",
                "seq_id": "mock_seq",
                "task_family": "room_to_room_same_floor_route_query",
                "task_type": "query_room_route",
                "start_room": "room_1",
                "target_spec": {"target_type": "room", "goal_room_id": "room_2"},
                "expected_target_room": "room_2",
                "expected_floor_id": "floor_1",
                "requires_vertical_transition": False,
                "expected_transition_count": 0,
                "acceptable_transition_sequence": [],
                "expected_outcome": "success",
            },
            {
                "task_id": "mock_nl",
                "seq_id": "mock_seq",
                "task_family": "constrained_nl_triggered_query_subset",
                "task_type": "nl_request",
                "start_room": "room_1",
                "instruction": "How do I get to room 2?",
                "target_spec": {"target_type": "room", "goal_room_id": "room_2"},
                "expected_target_room": "room_2",
                "expected_floor_id": "floor_1",
                "requires_vertical_transition": False,
                "expected_transition_count": 0,
                "acceptable_transition_sequence": [],
                "expected_outcome": "success",
                "expected_intent": "route_explanation",
                "expected_target_type": "room",
                "expected_slots": {"target_text": "room 2"},
            },
        ],
    }


def _sample_topology_payload() -> dict:
    return {
        "version": "0.1",
        "sequence_id": "mock_seq",
        "floors": [
            {"floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
        "rooms": [
            {"id": "room_1", "floor_id": "floor_1", "display_floor_id": "floor_1", "display_order": 1},
            {"id": "room_2", "floor_id": "floor_2", "display_floor_id": "floor_2", "display_order": 2},
        ],
        "edges": [
            {
                "source": "room_1",
                "target": "room_2",
                "relation_type": "vertical_transition",
                "metadata": {"transition_ids": ["vt_1"]},
            }
        ],
        "entities": {
            "objects": [
                {"id": "obj_1", "label": "chair", "room_id": "room_2", "floor_id": "floor_2"},
            ],
            "anchors": [
                {"id": "anchor_obj_1", "anchor_type": "object", "room_id": "room_2", "target_id": "obj_1", "floor_id": "floor_2"},
            ],
        },
    }


def run_mock_test() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        json_path = temp_path / "tasks.json"
        csv_path = temp_path / "tasks.csv"

        json_path.write_text(json.dumps(_sample_task_sheet_payload(), indent=2), encoding="utf-8")
        loaded_json = load_task_sheet(json_path)
        assert len(loaded_json["tasks"]) == 2
        assert loaded_json["tasks"][1]["expected_slots"]["target_text"] == "room 2"

        write_task_sheet_csv(loaded_json, csv_path)
        loaded_csv = load_task_sheet(csv_path)
        assert len(loaded_csv["tasks"]) == 2
        assert loaded_csv["tasks"][0]["target_spec"]["goal_room_id"] == "room_2"
        assert loaded_csv["tasks"][0]["acceptable_transition_sequence"] == []

    floor_variant = VariantConfig(key="floor_agnostic", label="floor", floor_agnostic=True)
    floor_payload = transform_topology_payload(_sample_topology_payload(), floor_variant)
    assert floor_payload["floors"][0]["floor_id"] == "floor_agnostic"
    assert all(room["floor_id"] == "floor_agnostic" for room in floor_payload["rooms"])

    vt_variant = VariantConfig(key="no_vt", label="no_vt", remap_vertical_transition=True)
    vt_payload = transform_topology_payload(_sample_topology_payload(), vt_variant)
    assert vt_payload["edges"][0]["relation_type"] == "transition"

    metric_row = build_metric_row(
        "mock",
        [
            {"RRA": True, "FCA": True, "TRA": True, "RSR": True, "CTC": None, "VTS": True, "SESR": True, "HOA": True, "FSR": False, "IPS": True, "SRA": True, "NL_E2E": True, "pass": True},
            {"RRA": False, "FCA": False, "TRA": False, "RSR": True, "CTC": None, "VTS": False, "SESR": None, "HOA": None, "FSR": False, "IPS": None, "SRA": None, "NL_E2E": None, "pass": False},
        ],
    )
    assert metric_row["RRA_raw"] == "1/2"
    assert metric_row["FCA_raw"] == "1/2"
    assert metric_row["TRA_raw"] == "1/2"
    assert metric_row["RSR_raw"] == "2/2"
    assert metric_row["VTS_raw"] == "1/2"
    assert metric_row["FSR_raw"] == "0/2"

    print("World-model evaluation utilities validated.")


if __name__ == "__main__":
    run_mock_test()
