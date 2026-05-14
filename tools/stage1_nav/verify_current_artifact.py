#!/usr/bin/env python3
"""Verify Stage1 navigation artifacts are present and consistent."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


def check(label: str, condition: bool, detail: str = "") -> dict:
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {label}" + (f": {detail}" if detail else ""))
    return {"label": label, "passed": condition, "detail": detail}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    stage = args.stage_output_dir.resolve()
    checks = []

    print(f"Verifying Stage1 artifacts in: {stage}")
    print()

    # Gateway registry
    registry_path = stage / "gateway/gateway_registry_v0_1.json"
    checks.append(check("Gateway registry exists", registry_path.exists()))
    if registry_path.exists():
        reg = json.loads(registry_path.read_text())
        gw_count = reg.get("gateway_count", 0)
        checks.append(check("Gateway count >= 8", gw_count >= 8, f"count={gw_count}"))
        by_id = reg.get("gateways_by_id", {})
        for required_gw in ["gw_00824_r1_r3_01", "gw_00824_r3_r8_01", "gw_00824_r3_r7_01",
                            "gw_00824_r8_r11_01", "gw_00824_r7_r11_02", "gw_00824_r7_r14_01",
                            "gw_00824_r7_r15_01", "gw_00824_r14_r16_01"]:
            entry = by_id.get(required_gw, {})
            has_pose = bool(entry.get("representative_crossing_pose"))
            checks.append(check(f"Gateway {required_gw} has pose data", has_pose))

    # Config
    config_path = stage / "config/step30s7_gateway_projection_config_v0_1.json"
    checks.append(check("Gateway projection config exists", config_path.exists()))

    # Maps
    h8r2_map = stage / "maps/h8r2_gateway_preserving_nav_map.yaml"
    checks.append(check("H8R2 reference map exists", h8r2_map.exists()))
    stable_map = stage / "maps/stage1_full_scene_occupancy_map.yaml"
    checks.append(check("Stable full-scene occupancy map exists", stable_map.exists()))

    # Room segmentation
    room_mask = stage / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy"
    checks.append(check("Room mask exists", room_mask.exists()))

    # Layered BEV
    bev = stage / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz"
    checks.append(check("Layered BEV exists", bev.exists()))

    # Gateway hypotheses
    hyp = stage / "stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json"
    checks.append(check("Gateway hypotheses exists", hyp.exists()))

    # Route
    route = stage / "route/resolved_route_v0_1.json"
    checks.append(check("Resolved route exists", route.exists()))

    # RViz config
    rviz = stage / "rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz"
    checks.append(check("RViz config exists", rviz.exists()))

    # Topology
    topo = stage / "stage1_committed_public/topology_v0_1.json"
    checks.append(check("Topology exists", topo.exists()))

    # Nav2 config
    nav2_config = stage / "nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml"
    checks.append(check("Nav2 config exists", nav2_config.exists()))

    # Gazebo world
    world = stage / "nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf"
    checks.append(check("Gazebo world exists", world.exists()))

    # Check no active Step30S5 patched maps
    s5_deprecated = stage / "maps/deprecated_step30s5_room15_patch"
    s5_active_yaml = stage / "maps/step30s5_room15_patched_nav_map.yaml"
    checks.append(check("No active Step30S5 patched map", not s5_active_yaml.exists(),
                         "deprecated dir OK" if s5_deprecated.exists() else ""))

    print()
    passed = sum(1 for c in checks if c["passed"])
    total = len(checks)
    all_pass = passed == total
    print(f"Result: {passed}/{total} checks passed" + (" - ALL PASS" if all_pass else " - SOME FAILED"))

    payload = {
        "artifact_type": "stage1_artifact_verification",
        "version": "v0_1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "stage_output_dir": stage.as_posix(),
        "checks": checks,
        "passed": passed,
        "total": total,
        "all_passed": all_pass,
    }

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(payload, indent=2) + "\n")

    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
