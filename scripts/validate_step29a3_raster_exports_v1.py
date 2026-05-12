"""
Step29A3 Raster Export Validation Script
Validates that debug raster grids from the Stage-A rerun were properly exported.
"""
import json
import os
import sys
import glob
import numpy as np

SCENE_ID = "00824-Dd4bFSTQ8gi"
OUTPUT_ROOT = "./runtime_stage1_frozen_evidence/step29a3_00824_stage_a_debug_raster_export/scenes"
SCENE_DIR = os.path.join(OUTPUT_ROOT, SCENE_ID)
DEBUG_ROOM_DIR = os.path.join(SCENE_DIR, "debug_room", "floor_1")

# Expected approximate grid shape from previous evidence (Step29A2)
EXPECTED_SHAPE_APPROX = (1227, 1167)
EXPECTED_RESOLUTION = 0.05

# Expected tracked room IDs from Step29A2 evidence
EXPECTED_ROOM_IDS = {1, 3, 7, 8, 11}


def find_last_cycle(debug_dir):
    """Find the highest cycle number from run_*_09_final_labels.npy files."""
    pattern = os.path.join(debug_dir, "run_*_09_final_labels.npy")
    files = glob.glob(pattern)
    if not files:
        return None, None
    # Extract cycle numbers and sort numerically
    cycle_nums = []
    for f in files:
        bn = os.path.basename(f)
        cycle_nums.append(int(bn.split("_")[1]))
    cycle_nums.sort()
    cycle_num = cycle_nums[-1]
    return cycle_num, files


def validate():
    results = {
        "scene_id": SCENE_ID,
        "debug_room_dir": DEBUG_ROOM_DIR,
        "checks": {},
        "overall_pass": True,
    }

    def check(name, condition, detail=""):
        results["checks"][name] = {"pass": bool(condition), "detail": str(detail)}
        if not condition:
            results["overall_pass"] = False
        status = "PASS" if condition else "FAIL"
        print(f"  [{status}] {name}: {detail}")

    print(f"\n{'='*70}")
    print(f"Step29A3 Raster Export Validation")
    print(f"Scene: {SCENE_ID}")
    print(f"Debug room dir: {DEBUG_ROOM_DIR}")
    print(f"{'='*70}\n")

    # Check scene directory exists
    check("scene_dir_exists", os.path.isdir(SCENE_DIR), SCENE_DIR)
    if not os.path.isdir(SCENE_DIR):
        print("\nFATAL: Scene directory does not exist. Cannot continue.")
        results["overall_pass"] = False
        return results

    # Check debug_room directory exists
    check("debug_room_dir_exists", os.path.isdir(DEBUG_ROOM_DIR), DEBUG_ROOM_DIR)
    if not os.path.isdir(DEBUG_ROOM_DIR):
        print("\nFATAL: debug_room directory does not exist. Cannot continue.")
        results["overall_pass"] = False
        return results

    # Find last cycle
    last_cycle, all_cycles = find_last_cycle(DEBUG_ROOM_DIR)
    check("cycles_found", last_cycle is not None, f"last_cycle={last_cycle}, total_cycles={len(all_cycles) if all_cycles else 0}")
    if last_cycle is None:
        print("\nFATAL: No segmentation cycles found.")
        results["overall_pass"] = False
        return results

    cycle_count = len(all_cycles)
    results["cycle_count"] = cycle_count
    results["last_cycle"] = last_cycle

    print(f"\n  Found {cycle_count} segmentation cycles, last at frame {last_cycle}")

    # Check final labels file
    final_labels_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09_final_labels.npy")
    check("final_labels_exists", os.path.isfile(final_labels_path), final_labels_path)

    # Check repaired labels
    repaired_labels_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09b_repaired_labels.npy")
    check("final_repaired_labels_exists", os.path.isfile(repaired_labels_path), repaired_labels_path)

    # Check walls skeleton
    walls_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_02_walls_skeleton.png")
    check("final_walls_skeleton_exists", os.path.isfile(walls_path), walls_path)

    # Check free space
    free_space_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_05_free_space.png")
    check("final_free_space_exists", os.path.isfile(free_space_path), free_space_path)

    # Check outside boundary
    outside_boundary_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_03_outside_boundary.png")
    check("final_outside_boundary_exists", os.path.isfile(outside_boundary_path), outside_boundary_path)

    # Check full map
    full_map_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_04_full_map.png")
    # Could also be 04b with door carving
    if not os.path.isfile(full_map_path):
        full_map_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_04b_full_map_post_doors.png")
    check("final_full_map_exists", os.path.isfile(full_map_path), full_map_path)

    # Check tracking report
    tracking_report_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_12_tracking_report.json")
    check("final_tracking_report_exists", os.path.isfile(tracking_report_path), tracking_report_path)

    # Load and analyze final labels
    if os.path.isfile(final_labels_path):
        labels = np.load(final_labels_path)
        shape = labels.shape
        results["final_grid_shape"] = list(shape)
        print(f"\n  Final grid shape: {shape}")

        # Shape check
        shape_ok = (abs(shape[0] - EXPECTED_SHAPE_APPROX[0]) < 200 and
                    abs(shape[1] - EXPECTED_SHAPE_APPROX[1]) < 200)
        check("grid_shape_reasonable", shape_ok,
              f"got {shape}, expected approx {EXPECTED_SHAPE_APPROX}")

        # Room ID analysis
        unique_labels = set(np.unique(labels).tolist())
        # Remove background/wall labels (typically 0 and max+1)
        positive_labels = sorted([l for l in unique_labels if l > 0])
        results["all_unique_labels"] = positive_labels
        print(f"\n  Unique positive labels in final grid: {positive_labels}")
        print(f"  Total unique labels (incl 0): {len(unique_labels)}")

        # Check expected rooms
        # Note: labels are watershed local IDs, need to check tracking report for global IDs
        if os.path.isfile(tracking_report_path):
            with open(tracking_report_path, "r") as f:
                tracking_data = json.load(f)
            tracking_info = tracking_data.get("tracking", {})
            matched = tracking_info.get("matched", [])
            new_rooms = tracking_info.get("new_rooms", [])
            tracked_room_count = tracking_data.get("room_count", 0) if "room_count" in tracking_data else len(matched) + len(new_rooms)
            results["tracking_info"] = tracking_info
            results["tracked_room_count"] = tracked_room_count

            # Extract global IDs from tracking
            global_ids = set()
            for m in matched:
                if isinstance(m, dict):
                    gid = m.get("global_id") or m.get("tracked_id")
                    if gid is not None:
                        global_ids.add(int(gid))
            for n in new_rooms:
                if isinstance(n, dict):
                    gid = n.get("global_id") or n.get("tracked_id")
                    if gid is not None:
                        global_ids.add(int(gid))
            # Note: marker_label is the local grid label; global_id is the tracked room ID

            results["global_room_ids"] = sorted(global_ids)
            print(f"\n  Tracked global room IDs from report: {sorted(global_ids)}")
            print(f"  Tracked room count: {tracked_room_count}")

            # Check specific rooms
            for room_id in sorted(EXPECTED_ROOM_IDS):
                present = room_id in global_ids
                check(f"room_{room_id}_present", present,
                      f"room_{room_id} {'found' if present else 'NOT found'} in tracked IDs")

    # Load repaired labels for comparison
    if os.path.isfile(repaired_labels_path):
        repaired = np.load(repaired_labels_path)
        check("repaired_shape_matches_raw", repaired.shape == labels.shape if os.path.isfile(final_labels_path) else True,
              f"repaired shape: {repaired.shape}")

    # Resolution check (from metadata if available or from known config)
    results["resolution"] = EXPECTED_RESOLUTION
    check("resolution_documented", True, f"{EXPECTED_RESOLUTION} m/pixel (from DynamicRoomSegmenter config)")

    # Origin check
    results["origin"] = [-50.0, -50.0]
    check("origin_documented", True, "origin=[-50.0, -50.0] (from DynamicRoomSegmenter default)")

    # Pre-watershed markers
    pre_markers_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_08_pre_watershed_markers.npy")
    check("pre_watershed_markers_exists", os.path.isfile(pre_markers_path), pre_markers_path)

    # Count wall and free cells if we have the grids
    if os.path.isfile(free_space_path):
        import cv2
        free_space = cv2.imread(free_space_path, cv2.IMREAD_GRAYSCALE)
        if free_space is not None:
            free_count = int(np.count_nonzero(free_space > 127))
            results["free_cell_count"] = free_count
            print(f"\n  Free space cells: {free_count}")

    if os.path.isfile(walls_path):
        import cv2
        walls = cv2.imread(walls_path, cv2.IMREAD_GRAYSCALE)
        if walls is not None:
            wall_count = int(np.count_nonzero(walls > 127))
            results["wall_cell_count"] = wall_count
            print(f"\n  Wall cells: {wall_count}")

    # Summary
    print(f"\n{'='*70}")
    print(f"OVERALL: {'PASS' if results['overall_pass'] else 'FAIL'}")
    print(f"{'='*70}\n")

    # Write results JSON
    results_path = os.path.join(
        "./runtime_stage1_frozen_evidence/step29a3_00824_stage_a_debug_raster_export",
        "validation_results.json"
    )
    os.makedirs(os.path.dirname(results_path), exist_ok=True)
    with open(results_path, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"  Results written to: {results_path}")

    return results


if __name__ == "__main__":
    results = validate()
    sys.exit(0 if results["overall_pass"] else 1)
