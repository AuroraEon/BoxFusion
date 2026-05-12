"""
Step29A3 Final Raster Export Post-Processor
Reads the last segmentation cycle debug outputs and creates canonical final_* exports
with metadata JSON files required for Step29B readiness.
"""
import json
import os
import sys
import glob
import shutil
import numpy as np
import cv2

SCENE_ID = "00824-Dd4bFSTQ8gi"
OUTPUT_ROOT = "./runtime_stage1_frozen_evidence/step29a3_00824_stage_a_debug_raster_export/scenes"
SCENE_DIR = os.path.join(OUTPUT_ROOT, SCENE_ID)
DEBUG_ROOM_DIR = os.path.join(SCENE_DIR, "debug_room", "floor_1")
FINAL_EXPORT_DIR = os.path.join(SCENE_DIR, "final_raster_export")


def find_last_cycle(debug_dir):
    """Find the highest cycle number from run_*_09_final_labels.npy files."""
    pattern = os.path.join(debug_dir, "run_*_09_final_labels.npy")
    files = glob.glob(pattern)
    if not files:
        return None, []
    # Extract all cycle numbers
    all_cycles = []
    for f in files:
        bn = os.path.basename(f)
        all_cycles.append(int(bn.split("_")[1]))
    all_cycles.sort()
    cycle_num = all_cycles[-1]
    return cycle_num, all_cycles


def create_final_exports():
    os.makedirs(FINAL_EXPORT_DIR, exist_ok=True)

    if not os.path.isdir(DEBUG_ROOM_DIR):
        print(f"ERROR: Debug room directory not found: {DEBUG_ROOM_DIR}")
        sys.exit(1)

    last_cycle, all_cycles = find_last_cycle(DEBUG_ROOM_DIR)
    if last_cycle is None:
        print("ERROR: No segmentation cycles found")
        sys.exit(1)

    print(f"Creating final exports from cycle {last_cycle} ({len(all_cycles)} total cycles)")

    # Copy final labels
    src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09_final_labels.npy")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_tracked_room_labels.npy")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"  Copied: {dst}")

    # Copy repaired labels
    src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09b_repaired_labels.npy")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_repaired_room_labels.npy")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"  Copied: {dst}")

    # Copy walls skeleton
    src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_02_walls_skeleton.png")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_walls_skeleton.png")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"  Copied: {dst}")

    # Copy outside boundary
    src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_03_outside_boundary.png")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_outside_boundary.png")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"  Copied: {dst}")

    # Copy full map (handle door carving variants)
    full_map_src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_04_full_map.png")
    if not os.path.isfile(full_map_src):
        full_map_src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_04b_full_map_post_doors.png")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_full_map.png")
    if os.path.isfile(full_map_src):
        shutil.copy2(full_map_src, dst)
        print(f"  Copied: {dst}")

    # Copy free space
    src = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_05_free_space.png")
    dst = os.path.join(FINAL_EXPORT_DIR, "final_free_space.png")
    if os.path.isfile(src):
        shutil.copy2(src, dst)
        print(f"  Copied: {dst}")

    # Load labels for metadata
    labels_path = os.path.join(FINAL_EXPORT_DIR, "final_tracked_room_labels.npy")
    repaired_path = os.path.join(FINAL_EXPORT_DIR, "final_repaired_room_labels.npy")

    labels = np.load(labels_path) if os.path.isfile(labels_path) else None
    repaired = np.load(repaired_path) if os.path.isfile(repaired_path) else None

    # Use repaired as the authoritative final labels
    final_labels = repaired if repaired is not None else labels

    # Load tracking report
    tracking_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_12_tracking_report.json")
    tracking_data = {}
    if os.path.isfile(tracking_path):
        with open(tracking_path, "r") as f:
            tracking_data = json.load(f)

    # Extract room ID info
    tracking_info = tracking_data.get("tracking", {})
    matched = tracking_info.get("matched", [])
    new_rooms = tracking_info.get("new_rooms", [])
    global_ids = set()
    label_to_global = {}
    for m in matched:
        if isinstance(m, dict):
            gid = m.get("global_id") or m.get("tracked_id")
            local = m.get("marker_label") or m.get("local_label") or m.get("label")
            if gid is not None:
                global_ids.add(int(gid))
            if gid is not None and local is not None:
                label_to_global[int(local)] = int(gid)
    for n in new_rooms:
        if isinstance(n, dict):
            gid = n.get("global_id") or n.get("tracked_id")
            local = n.get("marker_label") or n.get("local_label") or n.get("label")
            if gid is not None:
                global_ids.add(int(gid))
            if gid is not None and local is not None:
                label_to_global[int(local)] = int(gid)

    # Compute cell counts
    wall_count = 0
    free_count = 0
    walls_img_path = os.path.join(FINAL_EXPORT_DIR, "final_walls_skeleton.png")
    free_img_path = os.path.join(FINAL_EXPORT_DIR, "final_free_space.png")
    if os.path.isfile(walls_img_path):
        walls_img = cv2.imread(walls_img_path, cv2.IMREAD_GRAYSCALE)
        if walls_img is not None:
            wall_count = int(np.count_nonzero(walls_img > 127))
    if os.path.isfile(free_img_path):
        free_img = cv2.imread(free_img_path, cv2.IMREAD_GRAYSCALE)
        if free_img is not None:
            free_count = int(np.count_nonzero(free_img > 127))

    # Create grid metadata
    grid_metadata = {
        "scene_id": SCENE_ID,
        "last_cycle_frame_index": int(last_cycle),
        "total_segmentation_cycles": int(len(all_cycles)),
        "all_cycle_frame_indices": all_cycles,
        "grid_shape": list(final_labels.shape) if final_labels is not None else None,
        "resolution_m_per_pixel": 0.05,
        "origin": [-50.0, -50.0],
        "origin_note": "DynamicRoomSegmenter default locked origin (x=-50, y=-50 meters)",
        "tracked_room_count": int(len(global_ids)),
        "global_room_ids": sorted(global_ids),
        "label_to_global_id_map": {str(k): v for k, v in label_to_global.items()},
        "wall_cell_count": wall_count,
        "free_cell_count": free_count,
        "tracking_report": tracking_info,
    }

    metadata_path = os.path.join(FINAL_EXPORT_DIR, "final_grid_metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(grid_metadata, f, indent=2)
    print(f"  Written: {metadata_path}")

    # Create room ID summary
    room_summary = {
        "scene_id": SCENE_ID,
        "tracked_room_count": int(len(global_ids)),
        "global_room_ids": sorted(global_ids),
        "room_presence": {},
        "label_to_global_id_map": {str(k): v for k, v in label_to_global.items()},
    }
    for room_id in sorted(global_ids):
        room_summary["room_presence"][f"room_{room_id}"] = True
    # Check specific expected rooms
    for expected_id in [1, 3, 7, 8, 11]:
        key = f"room_{expected_id}"
        if key not in room_summary["room_presence"]:
            room_summary["room_presence"][key] = False

    room_summary_path = os.path.join(FINAL_EXPORT_DIR, "final_room_id_summary.json")
    with open(room_summary_path, "w") as f:
        json.dump(room_summary, f, indent=2)
    print(f"  Written: {room_summary_path}")

    # Create per-cycle manifest
    cycle_manifest = []
    for cycle in all_cycles:
        entry = {
            "cycle_frame_index": cycle,
            "files": {}
        }
        file_map = {
            "final_labels": f"run_{cycle}_09_final_labels.npy",
            "repaired_labels": f"run_{cycle}_09b_repaired_labels.npy",
            "pre_watershed_markers": f"run_{cycle}_08_pre_watershed_markers.npy",
            "walls_skeleton": f"run_{cycle}_02_walls_skeleton.png",
            "outside_boundary": f"run_{cycle}_03_outside_boundary.png",
            "full_map": f"run_{cycle}_04_full_map.png",
            "free_space": f"run_{cycle}_05_free_space.png",
            "tracking_report": f"run_{cycle}_12_tracking_report.json",
        }
        for role, filename in file_map.items():
            path = os.path.join(DEBUG_ROOM_DIR, filename)
            entry["files"][role] = {
                "filename": filename,
                "exists": os.path.isfile(path),
            }
            # Check alternate full_map paths
            if role == "full_map" and not os.path.isfile(path):
                alt = f"run_{cycle}_04b_full_map_post_doors.png"
                alt_path = os.path.join(DEBUG_ROOM_DIR, alt)
                if os.path.isfile(alt_path):
                    entry["files"][role] = {
                        "filename": alt,
                        "exists": True,
                        "note": "door_carving_variant"
                    }
        cycle_manifest.append(entry)

    manifest_path = os.path.join(FINAL_EXPORT_DIR, "cycle_manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(cycle_manifest, f, indent=2)
    print(f"  Written: {manifest_path}")

    print(f"\nFinal exports complete in: {FINAL_EXPORT_DIR}")
    print(f"  Grid shape: {final_labels.shape if final_labels is not None else 'N/A'}")
    print(f"  Resolution: 0.05 m/pixel")
    print(f"  Origin: [-50.0, -50.0]")
    print(f"  Tracked room IDs: {sorted(global_ids)}")
    print(f"  Total cycles: {len(all_cycles)}")


if __name__ == "__main__":
    create_final_exports()
