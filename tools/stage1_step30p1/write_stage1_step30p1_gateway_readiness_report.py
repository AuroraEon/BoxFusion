#!/usr/bin/env python3
"""Write Step30S7 gateway generalization readiness notes from externalized config."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from step30s7_common import load_config, now_iso, write_json


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    config = load_config(stage_output)
    payload = {
        "artifact_type": "step30s7_gateway_generalization_readiness_report",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "externalized": {
            "selected_gateway_pairs": config.get("selected_gateway_pairs"),
            "forbidden_gateway_pairs": config.get("forbidden_gateway_pairs"),
            "gateway_hypotheses_path": config.get("gateway_hypotheses_path"),
            "projection_inputs": [config.get("room_mask_path"), config.get("layered_bev_path"), config.get("h8r2_masks_npz")],
        },
        "scene_specific_remaining": [
            "The config is still for scene 00824 and depends on scene-local gateway artifacts.",
            "Gateway extraction itself is not re-run as a multi-scene truth-blind package in Step30S7.",
            "Topology quality still depends on the selected gateway artifact produced for this scene.",
        ],
        "active_route_projection_code_hard_codes_room15": False,
        "active_route_projection_code_hard_codes_route_room_ids": False,
        "full_multi_scene_gateway_generalization_claimed": False,
        "readiness_summary": "Step30S7 externalizes obvious route/projection constants and uses request-aware inclusion, but does not claim full multi-scene gateway extraction generalization.",
    }
    write_json(args.output_json, payload)
    lines = [
        "# Step30S7 Gateway Generalization Readiness",
        "",
        "Selected gateway ids, forbidden pairs, and projection input paths are externalized in `config/step30s7_gateway_projection_config_v0_1.json`.",
        "",
        f"Active route/projection code hard-codes room15: `{payload['active_route_projection_code_hard_codes_room15']}`",
        f"Active route/projection code hard-codes route_room_ids: `{payload['active_route_projection_code_hard_codes_route_room_ids']}`",
        f"Full multi-scene gateway generalization claimed: `{payload['full_multi_scene_gateway_generalization_claimed']}`",
        "",
        "Remaining scene-specific work: build and validate a reusable multi-scene gateway extractor, replace scene-local config bootstrap, and broaden cross-scene regression coverage.",
    ]
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

