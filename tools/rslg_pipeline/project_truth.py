"""Central project truth constants for the formal RSLG-SLAM pipeline."""

from __future__ import annotations

from pathlib import Path


PROJECT_NAME = "RSLG-SLAM"
HISTORICAL_REPOSITORY_PATH = Path("/home/ws/workspace/BoxFusion")

MAIN_SCENE_ID = "00843-DYehNKdT76V"
CANONICAL_ROOT = HISTORICAL_REPOSITORY_PATH / "stage_outputs" / "rslg_slam" / MAIN_SCENE_ID
CANONICAL_ARTIFACT_DIR = CANONICAL_ROOT / "canonical"
TASK_OUTPUT_ROOT = CANONICAL_ROOT / "tasks"

OFFICIAL_LAYERS = (
    "Layer 0: Input Layer",
    "Layer 1: World Model Layer",
    "Layer 2: Formal Artifact Layer",
    "Layer 3: Navigation Interface Layer",
    "Layer 4: Runtime Validation Layer",
)

POSITIONING = (
    "Rich Semantic + Light Geometry",
    "posed RGB-D semantic-topological world-modeling backend",
    "navigation-interface-oriented light-geometry semantic topology",
    "structured semantic-topological navigation interface",
    "formal artifacts for route and validation",
)

FORBIDDEN_CLAIMS = (
    "dense reconstruction",
    "neural implicit SLAM",
    "full embodied navigation benchmark",
    "full BEV planner",
    "AMCL success",
    "LLM runtime system",
    "real robot deployment",
    "collision-free guarantee",
    "osmAG-Nav",
)

CROSS_FLOOR_ROOM_ROUTE = (
    {"room": "room_2", "floor": "floor_1"},
    {"room": "room_3", "floor": "floor_1"},
    {"vertical_connector": "vt_1", "connector_id": "vc_vt_1"},
    {"room": "room_7", "floor": "floor_2"},
    {"room": "room_13", "floor": "floor_2"},
    {"room": "room_14", "floor": "floor_2"},
)

TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"

OBJECT_QUERY = "curtain in room_14 on floor_2"
OBJECT_ID = "obj_175"
OBJECT_LABEL = "curtain"
OBJECT_FLOOR = "floor_2"
OBJECT_ROOM = "room_14"

CURRENT_OBJECT_APPROACH_ID = "generated_ring_002"
CURRENT_OBJECT_APPROACH_POSITION = (-7.020484, 1.558795)
CURRENT_OBJECT_APPROACH_YAW = -2.09057
CURRENT_OBJECT_APPROACH_CLEARANCE_M = 0.20

BLOCKED_LEGACY_APPROACH_IDS = ("generated_ring_037",)

OFFLINE_PYTHON = Path("/home/ws/miniconda3/envs/boxfusion/bin/python")
ROS_PYTHON = Path("/usr/bin/python3")

FORMAL_COMMAND_SURFACE = Path("tools/rslg_pipeline")


def canonical_root_str() -> str:
    return CANONICAL_ROOT.as_posix()


def current_object_approach_record() -> dict[str, object]:
    return {
        "candidate_id": CURRENT_OBJECT_APPROACH_ID,
        "object_id": OBJECT_ID,
        "label": OBJECT_LABEL,
        "floor": OBJECT_FLOOR,
        "room": OBJECT_ROOM,
        "position": list(CURRENT_OBJECT_APPROACH_POSITION),
        "yaw": CURRENT_OBJECT_APPROACH_YAW,
        "clearance_m": CURRENT_OBJECT_APPROACH_CLEARANCE_M,
        "is_object_centroid": False,
    }
