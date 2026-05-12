"""
Step29A3 Final Raster Visualization Script
Renders visualizations of the final-cycle debug rasters from the Stage-A rerun.
"""
import json
import os
import sys
import glob
import numpy as np
import cv2

SCENE_ID = "00824-Dd4bFSTQ8gi"
OUTPUT_ROOT = "./runtime_stage1_frozen_evidence/step29a3_00824_stage_a_debug_raster_export/scenes"
SCENE_DIR = os.path.join(OUTPUT_ROOT, SCENE_ID)
DEBUG_ROOM_DIR = os.path.join(SCENE_DIR, "debug_room", "floor_1")
VIS_DIR = "./generated/visualizations"


def find_last_cycle(debug_dir):
    """Find the highest cycle number from run_*_09_final_labels.npy files."""
    pattern = os.path.join(debug_dir, "run_*_09_final_labels.npy")
    files = glob.glob(pattern)
    if not files:
        return None
    cycle_nums = [int(os.path.basename(f).split("_")[1]) for f in files]
    return max(cycle_nums)


def colorize_labels(labels, background_val=0):
    """Create a colorized visualization of label map."""
    h, w = labels.shape
    vis = np.zeros((h, w, 3), dtype=np.uint8)
    unique = np.unique(labels)
    for label in unique:
        if label == background_val:
            continue
        # Generate distinct color per label
        r = (60 + 53 * int(label)) % 255
        g = (90 + 79 * int(label)) % 255
        b = (130 + 31 * int(label)) % 255
        vis[labels == label] = [b, g, r]
    return vis


def render_all():
    os.makedirs(VIS_DIR, exist_ok=True)

    if not os.path.isdir(DEBUG_ROOM_DIR):
        print(f"ERROR: Debug room directory not found: {DEBUG_ROOM_DIR}")
        sys.exit(1)

    last_cycle = find_last_cycle(DEBUG_ROOM_DIR)
    if last_cycle is None:
        print("ERROR: No segmentation cycles found")
        sys.exit(1)

    print(f"Rendering visualizations for last cycle: {last_cycle}")

    # 1. Final room labels
    labels_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09_final_labels.npy")
    repaired_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_09b_repaired_labels.npy")
    
    # Use repaired if available, else raw
    use_path = repaired_path if os.path.isfile(repaired_path) else labels_path
    if os.path.isfile(use_path):
        labels = np.load(use_path)
        vis_labels = colorize_labels(labels)
        out_path = os.path.join(VIS_DIR, "00824_step29a3_final_room_labels.png")
        cv2.imwrite(out_path, vis_labels)
        print(f"  Written: {out_path}")
    else:
        print(f"  WARNING: No labels file found")
        labels = None

    # 2. Walls skeleton
    walls_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_02_walls_skeleton.png")
    if os.path.isfile(walls_path):
        walls = cv2.imread(walls_path)
        out_path = os.path.join(VIS_DIR, "00824_step29a3_final_walls_skeleton.png")
        cv2.imwrite(out_path, walls)
        print(f"  Written: {out_path}")
    else:
        print(f"  WARNING: Walls skeleton not found")
        walls = None

    # 3. Free space
    free_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_05_free_space.png")
    if os.path.isfile(free_path):
        free_space = cv2.imread(free_path)
        out_path = os.path.join(VIS_DIR, "00824_step29a3_final_free_space.png")
        cv2.imwrite(out_path, free_space)
        print(f"  Written: {out_path}")
    else:
        print(f"  WARNING: Free space not found")
        free_space = None

    # 4. Outside boundary
    outside_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_03_outside_boundary.png")
    if os.path.isfile(outside_path):
        outside = cv2.imread(outside_path)
        out_path = os.path.join(VIS_DIR, "00824_step29a3_final_outside_boundary.png")
        cv2.imwrite(out_path, outside)
        print(f"  Written: {out_path}")
    else:
        print(f"  WARNING: Outside boundary not found")
        outside = None

    # 5. Room-wall overlay
    if labels is not None and walls is not None:
        overlay = vis_labels.copy()
        walls_gray = cv2.imread(walls_path, cv2.IMREAD_GRAYSCALE)
        if walls_gray is not None:
            # Resize walls to match labels if needed
            if walls_gray.shape != labels.shape:
                walls_gray = cv2.resize(walls_gray, (labels.shape[1], labels.shape[0]), interpolation=cv2.INTER_NEAREST)
            wall_mask = walls_gray > 127
            overlay[wall_mask] = [255, 255, 255]  # White walls on colored rooms
        out_path = os.path.join(VIS_DIR, "00824_step29a3_final_room_wall_overlay.png")
        cv2.imwrite(out_path, overlay)
        print(f"  Written: {out_path}")

    # 6. Key room presence debug
    KEY_ROOMS = [3, 7, 8, 11]
    if labels is not None:
        h, w = labels.shape
        # Create a grid visualization showing each key room highlighted
        n_rooms = len(KEY_ROOMS)
        cell_h = h
        cell_w = w
        grid = np.zeros((cell_h * 2, cell_w * 2, 3), dtype=np.uint8)

        # Load tracking report to get global IDs mapping
        tracking_path = os.path.join(DEBUG_ROOM_DIR, f"run_{last_cycle}_12_tracking_report.json")
        global_to_local = {}
        if os.path.isfile(tracking_path):
            with open(tracking_path, "r") as f:
                tracking_data = json.load(f)
            tracking_info = tracking_data.get("tracking", {})
            for m in tracking_info.get("matched", []):
                if isinstance(m, dict):
                    gid = m.get("global_id") or m.get("tracked_id")
                    local = m.get("marker_label") or m.get("local_label") or m.get("label")
                    if gid is not None and local is not None:
                        global_to_local[int(gid)] = int(local)
            for n in tracking_info.get("new_rooms", []):
                if isinstance(n, dict):
                    gid = n.get("global_id") or n.get("tracked_id")
                    local = n.get("marker_label") or n.get("local_label") or n.get("label")
                    if gid is not None and local is not None:
                        global_to_local[int(gid)] = int(local)

        for idx, room_id in enumerate(KEY_ROOMS):
            row = idx // 2
            col = idx % 2
            cell = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

            # Try to find this room - check both direct label and global-to-local mapping
            local_label = global_to_local.get(room_id, room_id)
            mask = labels == local_label
            if np.any(mask):
                cell[mask] = [0, 255, 0]  # Green for present
                # Add label
                ys, xs = np.where(mask)
                cy, cx = int(np.mean(ys)), int(np.mean(xs))
                cv2.putText(cell, f"room_{room_id}", (cx - 40, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                status_text = f"room_{room_id}: PRESENT (label={local_label})"
            else:
                status_text = f"room_{room_id}: NOT FOUND"
                cv2.putText(cell, status_text, (50, cell_h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Draw background (faint labels)
            bg = colorize_labels(labels) // 4
            cell = cv2.addWeighted(bg, 0.3, cell, 0.7, 0)

            grid[row * cell_h:(row + 1) * cell_h, col * cell_w:(col + 1) * cell_w] = cell
            print(f"  {status_text}")

        out_path = os.path.join(VIS_DIR, "00824_step29a3_key_room_presence_debug.png")
        cv2.imwrite(out_path, grid)
        print(f"  Written: {out_path}")

    print(f"\nAll visualizations written to: {VIS_DIR}/")


if __name__ == "__main__":
    render_all()
