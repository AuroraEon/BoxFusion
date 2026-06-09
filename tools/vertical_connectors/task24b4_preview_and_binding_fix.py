#!/usr/bin/env python3
"""Task24b4 stable-map preview and stair-graph endpoint binding fix."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import py_compile
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


TASK_NAME = "task24b4_stable_map_preview_and_stair_graph_binding_fix"
SCENE_ID = "00843-DYehNKdT76V"
CLAIM_BOUNDARY = "topological_vertical_transition_only"
PREVIEW_CONVENTION = (
    "human_display_png_is_color_remap_of_pgm_stored_image_orientation"
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    return value


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def hash_tree(root: Path) -> dict[str, str]:
    if not root.exists():
        return {}
    return {
        path.relative_to(root).as_posix(): sha256(path) or ""
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4:
        while idx < len(data) and data[idx : idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx : idx + 1] == b"#":
            while idx < len(data) and data[idx : idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx : idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    if tokens[0] != b"P5":
        raise ValueError(f"unsupported PGM magic in {path}: {tokens[0]!r}")
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f"unsupported PGM max value in {path}: {maxval}")
    while idx < len(data) and data[idx : idx + 1].isspace():
        idx += 1
    payload = data[idx : idx + width * height]
    if len(payload) != width * height:
        raise ValueError(f"PGM payload size mismatch in {path}")
    return np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()


def parse_map_yaml(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def preview_rgb_from_pgm(pgm: np.ndarray) -> np.ndarray:
    rgb = np.zeros((*pgm.shape, 3), dtype=np.uint8)
    rgb[pgm >= 250] = (245, 245, 245)
    rgb[(pgm > 10) & (pgm < 250)] = (112, 112, 112)
    rgb[pgm <= 10] = (20, 20, 20)
    return rgb


def write_preview_from_pgm(preview_path: Path, pgm_path: Path) -> None:
    rgb = preview_rgb_from_pgm(read_pgm(pgm_path))
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(preview_path.as_posix(), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError(f"failed to write preview PNG: {preview_path}")


def read_preview_gray(path: Path) -> np.ndarray:
    image = cv2.imread(path.as_posix(), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"failed to read preview PNG: {path}")
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        image = image[:, :, :3]
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def preview_class(image: np.ndarray) -> np.ndarray:
    return np.where(image <= 50, 0, np.where(image < 200, 1, 2)).astype(np.uint8)


def pgm_class(pgm: np.ndarray) -> np.ndarray:
    return np.where(pgm <= 10, 0, np.where(pgm < 250, 1, 2)).astype(np.uint8)


def transform_arrays(pgm: np.ndarray) -> dict[str, np.ndarray]:
    return {
        "pgm_identity": pgm,
        "flipud_pgm": np.flipud(pgm),
        "fliplr_pgm": np.fliplr(pgm),
        "flipud_fliplr_pgm": np.flipud(np.fliplr(pgm)),
    }


def classify_preview_against_pgm(preview_path: Path, pgm_path: Path) -> dict[str, Any]:
    pgm = read_pgm(pgm_path)
    preview = read_preview_gray(preview_path)
    preview_cls = preview_class(preview)
    comparisons: dict[str, Any] = {}
    best_name = None
    best_diff = None
    for name, arr in transform_arrays(pgm).items():
        arr_cls = pgm_class(arr)
        comparable = arr_cls.shape == preview_cls.shape
        diff = int((arr_cls != preview_cls).sum()) if comparable else None
        equal = bool(comparable and diff == 0)
        comparisons[name] = {
            "comparable": comparable,
            "different_cells_by_class": diff,
            "equal_by_class": equal,
        }
        if comparable and (best_diff is None or diff < best_diff):
            best_name = name
            best_diff = diff
    return {
        "preview": preview_path.as_posix(),
        "pgm": pgm_path.as_posix(),
        "preview_shape_hw": list(preview_cls.shape),
        "pgm_shape_hw": list(pgm.shape),
        "best_orientation_match": best_name,
        "best_orientation_different_cells": best_diff,
        "comparisons": comparisons,
        "sha256": sha256(preview_path),
    }


def image_class_equal(path_a: Path, path_b: Path) -> bool:
    a = preview_class(read_preview_gray(path_a))
    b = preview_class(read_preview_gray(path_b))
    return bool(a.shape == b.shape and np.array_equal(a, b))


def npz_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    with np.load(path) as data:
        return {
            "exists": True,
            "keys": sorted(data.files),
            "arrays": {
                key: {"shape": list(data[key].shape), "dtype": str(data[key].dtype)}
                for key in data.files
            },
        }


def map_dimension_check(yaml_path: Path) -> dict[str, Any]:
    meta = parse_map_yaml(yaml_path)
    pgm_path = yaml_path.parent / str(meta["image"])
    pgm = read_pgm(pgm_path)
    npz = npz_summary(yaml_path.with_suffix(".npz"))
    npz_shapes: dict[str, list[int]] = {}
    if npz["exists"]:
        for key, value in npz["arrays"].items():
            shape = value["shape"]
            if len(shape) >= 2:
                npz_shapes[key] = shape[:2]
    shape_matches = {
        key: value == list(pgm.shape) for key, value in npz_shapes.items()
    }
    return {
        "yaml": yaml_path.as_posix(),
        "pgm": pgm_path.as_posix(),
        "yaml_image_exists": pgm_path.exists(),
        "pgm_shape_hw": list(pgm.shape),
        "npz": npz,
        "npz_2d_shape_matches_pgm": shape_matches,
        "passed": bool(
            pgm_path.exists() and all(shape_matches.values())
        ),
    }


def copy_if_exists(src: Path, dst: Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def preview_audit_and_fix(
    scene_root: Path,
    task24b3: Path,
    out: Path,
) -> tuple[dict[str, Any], str]:
    clean_floor2 = scene_root / "clean_rerun/maps/floor_2"
    b3_floor2 = task24b3 / "corrected_generated_maps/floor_2_regression_check"
    b3_floor1 = task24b3 / "corrected_generated_maps/floor_1"

    clean_floor2_pgm = clean_floor2 / "stage1_floor_2_stable_occupancy_map.pgm"
    clean_floor2_preview = clean_floor2 / "stage1_floor_2_stable_occupancy_map_preview.png"
    b3_floor2_pgm = b3_floor2 / "stage1_floor_2_stable_occupancy_map.pgm"
    b3_floor2_preview = b3_floor2 / "stage1_floor_2_stable_occupancy_map_preview.png"
    b3_floor1_pgm = b3_floor1 / "stage1_floor_1_stable_occupancy_map.pgm"

    if not clean_floor2_preview.exists():
        classification = "blocked_missing_validated_preview"
    else:
        classification = "runtime_pgm_passed_preview_fixed"

    pgm_byte_exact = bool(
        clean_floor2_pgm.read_bytes() == b3_floor2_pgm.read_bytes()
    )
    clean_preview_cmp = classify_preview_against_pgm(
        clean_floor2_preview, clean_floor2_pgm
    )
    b3_preview_cmp = classify_preview_against_pgm(b3_floor2_preview, b3_floor2_pgm)

    corrected_floor2_preview = (
        out
        / "corrected_preview_maps/floor_2/stage1_floor_2_stable_occupancy_map_preview.png"
    )
    corrected_floor1_preview = (
        out
        / "corrected_preview_maps/floor_1/stage1_floor_1_stable_occupancy_map_preview.png"
    )
    write_preview_from_pgm(corrected_floor2_preview, b3_floor2_pgm)
    write_preview_from_pgm(corrected_floor1_preview, b3_floor1_pgm)

    corrected_floor2_cmp = classify_preview_against_pgm(
        corrected_floor2_preview, b3_floor2_pgm
    )
    corrected_floor1_cmp = classify_preview_against_pgm(
        corrected_floor1_preview, b3_floor1_pgm
    )
    corrected_matches_validated = (
        clean_floor2_preview.exists()
        and image_class_equal(corrected_floor2_preview, clean_floor2_preview)
    )
    wrong_flip_equal = corrected_floor2_cmp["comparisons"]["flipud_pgm"][
        "equal_by_class"
    ]

    package = out / "corrected_floor1_candidate_package"
    stem = "stage1_floor_1_stable_occupancy_map"
    copied = {
        "yaml": copy_if_exists(b3_floor1 / f"{stem}.yaml", package / f"{stem}.yaml"),
        "pgm": copy_if_exists(b3_floor1 / f"{stem}.pgm", package / f"{stem}.pgm"),
        "npz": copy_if_exists(b3_floor1 / f"{stem}.npz", package / f"{stem}.npz"),
        "provenance": copy_if_exists(
            b3_floor1 / f"{stem}_provenance.json",
            package / f"{stem}_provenance.json",
        ),
    }
    copy_if_exists(corrected_floor1_preview, package / f"{stem}_preview.png")
    copied["preview"] = True

    if not pgm_byte_exact:
        classification = "runtime_pgm_regression_failed"
    elif not corrected_matches_validated and clean_floor2_preview.exists():
        classification = "runtime_pgm_passed_preview_still_wrong"

    comparison = {
        "scene_id": SCENE_ID,
        "created_utc": now_iso(),
        "preview_convention": PREVIEW_CONVENTION,
        "validated_preview_present": clean_floor2_preview.exists(),
        "floor_2_pgm_byte_exact_validated_vs_task24b3_corrected": pgm_byte_exact,
        "validated_clean_floor_2_preview": clean_preview_cmp,
        "task24b3_corrected_floor_2_preview": b3_preview_cmp,
        "corrected_task24b4_floor_2_preview": corrected_floor2_cmp,
        "corrected_task24b4_floor_1_preview": corrected_floor1_cmp,
        "bug_diagnosis": {
            "task24b3_preview_orientation": b3_preview_cmp[
                "best_orientation_match"
            ],
            "validated_preview_orientation": clean_preview_cmp[
                "best_orientation_match"
            ],
            "exact_bug": (
                "task24b3 rendered preview from flipud(PGM), while the validated "
                "human-readable preview is a color remap of the stored PGM image"
            ),
        },
        "corrected_floor_2_preview_matches_validated_by_class": corrected_matches_validated,
        "corrected_floor_2_preview_equals_wrong_vertical_flip_by_class": wrong_flip_equal,
        "floor_1_candidate_package_files_copied": copied,
        "stable_map_classification": classification,
    }
    write_json(out / "preview_orientation_comparison.json", comparison)
    write_text(
        out / "preview_orientation_audit.md",
        f"""# Preview Orientation Audit

Project: RSLG-SLAM.

Task: `{TASK_NAME}`.

Claim boundary: `{CLAIM_BOUNDARY}`.

## Finding

The task24b3 floor_2 runtime PGM is byte-exact against the validated clean_rerun floor_2 PGM: `{pgm_byte_exact}`.

The validated clean_rerun floor_2 preview best matches: `{clean_preview_cmp['best_orientation_match']}`.

The task24b3 generated floor_2 preview best matches: `{b3_preview_cmp['best_orientation_match']}`.

Exact preview bug: task24b3 rendered the human-readable preview from the world-grid image, which is `flipud(PGM)` after runtime PGM packaging. Runtime map data was not changed.

## Fix

Task24b4 renders preview PNGs as a color remap of the stored PGM image orientation:

`{PREVIEW_CONVENTION}`

Corrected floor_2 preview matches validated clean preview by occupancy class: `{corrected_matches_validated}`.

Corrected floor_2 preview still equals the wrong vertical flip convention: `{wrong_flip_equal}`.

Corrected floor_1 preview was generated using the same convention: `{corrected_floor1_cmp['best_orientation_match']}`.

## Outputs

- `corrected_preview_maps/floor_2/stage1_floor_2_stable_occupancy_map_preview.png`
- `corrected_preview_maps/floor_1/stage1_floor_1_stable_occupancy_map_preview.png`
- `corrected_floor1_candidate_package/`

Stable map classification: `{classification}`.
""",
    )
    return comparison, classification


def raw_trace_ids(raw_trace: dict[str, Any]) -> set[str]:
    ids: set[str] = set()
    for sample in raw_trace.get("samples", []):
        if "node_id" in sample:
            ids.add(str(sample["node_id"]))
        if "sample_index" in sample:
            ids.add(f"vt_1_n{int(sample['sample_index']):03d}")
    return ids


def sorted_centerline_nodes(graph: dict[str, Any]) -> list[dict[str, Any]]:
    nodes = graph.get("fitted_stair_centerline") or graph.get("nodes") or []
    return sorted(nodes, key=lambda node: float(node.get("sample_t", 0.0)))


def fix_one_graph(
    graph: dict[str, Any],
    raw_trace: dict[str, Any],
    source_name: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    fixed = copy.deepcopy(graph)
    centerline = sorted_centerline_nodes(fixed)
    if len(centerline) < 2:
        raise ValueError(f"{source_name} has fewer than two fitted centerline nodes")
    from_node = str(centerline[0]["node_id"])
    to_node = str(centerline[-1]["node_id"])
    original = copy.deepcopy(fixed.get("endpoint_bindings", {}))
    raw_from = original.get("from", {}).get("endpoint_node", "vt_1_n000")
    raw_to = original.get("to", {}).get("endpoint_node", "vt_1_n022")
    floor_from = fixed.get("floor_from")
    floor_to = fixed.get("floor_to")

    def binding(role: str, endpoint_node: str, raw_ref: str) -> dict[str, Any]:
        old = original.get(role, {})
        endpoint_floor = floor_from if role == "from" else floor_to
        return {
            "endpoint_role": role,
            "floor_from": floor_from,
            "floor_to": floor_to,
            "endpoint_floor": endpoint_floor,
            "floor": endpoint_floor,
            "room_id": old.get("room_id"),
            "bound_room_id": old.get("room_id"),
            "floor_topology_node": old.get("floor_topology_node"),
            "bound_topology_node_id": old.get("floor_topology_node"),
            "gateway_id": old.get("gateway_id"),
            "endpoint_node": endpoint_node,
            "endpoint_fitted_centerline_node": endpoint_node,
            "raw_trace_endpoint_ref": raw_ref,
            "provenance": {
                "source_endpoint_binding": old,
                "fix_task": TASK_NAME,
                "fix_reason": (
                    "endpoint binding must point to fitted_stair_centerline node "
                    "IDs that exist in the sparse_stair_connector_graph"
                ),
                "ordered_transition_pose_trace_ref": fixed.get(
                    "ordered_transition_pose_trace_ref"
                ),
            },
        }

    fixed["endpoint_bindings"] = {
        "from": binding("from", from_node, str(raw_from)),
        "to": binding("to", to_node, str(raw_to)),
    }
    fixed["raw_trace_endpoint_ref"] = {
        "from": str(raw_from),
        "to": str(raw_to),
        "provenance_role": "ordered_transition_pose_trace provenance only",
    }
    fixed["physical_execution_supported"] = False
    fixed["claim_boundary"] = CLAIM_BOUNDARY
    fixed["schema_version"] = "0.3"
    fixed["binding_fix"] = {
        "created_utc": now_iso(),
        "source_graph": source_name,
        "task": TASK_NAME,
        "geometry_changed": False,
        "from_endpoint_node_before": raw_from,
        "to_endpoint_node_before": raw_to,
        "from_endpoint_node_after": from_node,
        "to_endpoint_node_after": to_node,
    }

    validation = validate_graph(fixed, raw_trace, source_name)
    return fixed, validation


def validate_graph(
    graph: dict[str, Any],
    raw_trace: dict[str, Any],
    graph_name: str,
) -> dict[str, Any]:
    node_ids = {str(node["node_id"]) for node in graph.get("nodes", [])}
    edge_refs_valid = []
    missing_edge_refs = []
    for edge in graph.get("edges", []):
        ok = str(edge.get("source")) in node_ids and str(edge.get("target")) in node_ids
        edge_refs_valid.append(ok)
        if not ok:
            missing_edge_refs.append(edge)
    endpoint_refs = {
        role: binding.get("endpoint_node")
        for role, binding in graph.get("endpoint_bindings", {}).items()
    }
    endpoint_nodes_exist = {
        role: str(node_id) in node_ids for role, node_id in endpoint_refs.items()
    }
    raw_ids = raw_trace_ids(raw_trace)
    raw_refs = {
        role: binding.get("raw_trace_endpoint_ref")
        for role, binding in graph.get("endpoint_bindings", {}).items()
    }
    raw_refs_exist = {role: str(ref) in raw_ids for role, ref in raw_refs.items()}
    centerline_nodes = sorted_centerline_nodes(graph)
    passed = bool(
        len(centerline_nodes) >= 2
        and len(graph.get("edges", [])) >= 1
        and all(edge_refs_valid)
        and all(endpoint_nodes_exist.values())
        and all(raw_refs_exist.values())
        and graph.get("physical_execution_supported") is False
        and graph.get("claim_boundary") == CLAIM_BOUNDARY
    )
    return {
        "graph": graph_name,
        "node_count": len(node_ids),
        "edge_count": len(graph.get("edges", [])),
        "fitted_centerline_node_count": len(centerline_nodes),
        "endpoint_refs": endpoint_refs,
        "endpoint_nodes_exist": endpoint_nodes_exist,
        "edge_refs_all_exist": all(edge_refs_valid),
        "missing_edge_refs": missing_edge_refs,
        "raw_trace_endpoint_refs": raw_refs,
        "raw_trace_endpoint_refs_exist": raw_refs_exist,
        "physical_execution_supported_is_false": graph.get(
            "physical_execution_supported"
        )
        is False,
        "claim_boundary_valid": graph.get("claim_boundary") == CLAIM_BOUNDARY,
        "passed": passed,
    }


def endpoint_binding_fix(task24b3: Path, out: Path) -> tuple[dict[str, Any], str]:
    stairs_path = task24b3 / "stairs_graph_v0_2.json"
    sparse_path = task24b3 / "sparse_stair_connector_graph_vt_1.json"
    raw_path = task24b3 / "raw_transition_pose_trace_vt_1.json"
    if not stairs_path.exists() or not sparse_path.exists() or not raw_path.exists():
        validation = {
            "classification": "blocked_missing_stair_graph_inputs",
            "required_inputs_present": {
                "stairs_graph_v0_2": stairs_path.exists(),
                "sparse_stair_connector_graph_vt_1": sparse_path.exists(),
                "raw_transition_pose_trace_vt_1": raw_path.exists(),
            },
            "passed": False,
        }
        write_json(out / "endpoint_binding_validation.json", validation)
        return validation, "blocked_missing_stair_graph_inputs"

    raw_trace = read_json(raw_path)
    stairs = read_json(stairs_path)
    sparse = read_json(sparse_path)
    fixed_stairs, stairs_validation = fix_one_graph(
        stairs, raw_trace, "stairs_graph_v0_2.json"
    )
    fixed_sparse, sparse_validation = fix_one_graph(
        sparse, raw_trace, "sparse_stair_connector_graph_vt_1.json"
    )
    write_json(out / "stairs_graph_v0_3.json", fixed_stairs)
    write_json(out / "sparse_stair_connector_graph_vt_1_v0_3.json", fixed_sparse)

    passed = bool(stairs_validation["passed"] and sparse_validation["passed"])
    classification = "endpoint_binding_fixed" if passed else "endpoint_binding_still_invalid"
    validation = {
        "created_utc": now_iso(),
        "classification": classification,
        "claim_boundary": CLAIM_BOUNDARY,
        "raw_transition_pose_trace_preserved_as_provenance_only": True,
        "stairs_graph_v0_3": stairs_validation,
        "sparse_stair_connector_graph_vt_1_v0_3": sparse_validation,
        "passed": passed,
    }
    write_json(out / "endpoint_binding_validation.json", validation)
    write_text(
        out / "endpoint_binding_fix_report.md",
        f"""# Endpoint Binding Fix Report

Project: RSLG-SLAM.

Task: `{TASK_NAME}`.

Claim boundary: `{CLAIM_BOUNDARY}`.

## Finding

The task24b3 fitted stair centerline graph nodes are:

`{', '.join(node['node_id'] for node in sorted_centerline_nodes(stairs))}`

The task24b3 endpoint bindings pointed to raw ordered_transition_pose_trace sample IDs:

- from: `{stairs.get('endpoint_bindings', {}).get('from', {}).get('endpoint_node')}`
- to: `{stairs.get('endpoint_bindings', {}).get('to', {}).get('endpoint_node')}`

Those raw IDs are provenance references and are not node IDs in the sparse_stair_connector_graph.

## Fix

The from endpoint binding now points to `{fixed_stairs['endpoint_bindings']['from']['endpoint_node']}`.

The to endpoint binding now points to `{fixed_stairs['endpoint_bindings']['to']['endpoint_node']}`.

Raw trace endpoint references are preserved separately:

- from: `{fixed_stairs['endpoint_bindings']['from']['raw_trace_endpoint_ref']}`
- to: `{fixed_stairs['endpoint_bindings']['to']['raw_trace_endpoint_ref']}`

No straight centerline geometry was changed.

`physical_execution_supported` remains `false`.

Classification: `{classification}`.
""",
    )
    return validation, classification


def write_manual_copy_commands(
    out: Path,
    stable_map_classification: str,
    floor1_classification: str,
) -> None:
    if stable_map_classification != "runtime_pgm_passed_preview_fixed":
        write_text(
            out / "manual_floor_1_map_copy_commands.md",
            f"""# Manual Floor-1 Map Copy Commands

No install commands are provided because preview validation did not pass.

Stable map classification: `{stable_map_classification}`.

Floor_1 packaging classification: `{floor1_classification}`.

Diagnostic commands only:

```bash
SCENE_ROOT=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V
TASK_OUT="${{SCENE_ROOT}}/tasks/{TASK_NAME}"
python3 -m json.tool "${{TASK_OUT}}/preview_orientation_comparison.json" >/dev/null
python3 -m json.tool "${{TASK_OUT}}/task24b4_report.json" >/dev/null
```

No physical cross-floor execution is claimed.
""",
        )
        return

    write_text(
        out / "manual_floor_1_map_copy_commands.md",
        f"""# Manual Floor-1 Map Copy Commands

User must run these commands manually. Task24b4 did not install floor_1 into clean_rerun.

The floor_1 package is a candidate. Runtime PGM/YAML/NPZ convention is inherited from the floor_2 regression. The preview PNG is corrected only for human-readable display. No physical cross-floor execution is claimed.

Classification: `{floor1_classification}`.

```bash
SCENE_ROOT=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V
TASK_OUT="${{SCENE_ROOT}}/tasks/{TASK_NAME}"
SRC="${{TASK_OUT}}/corrected_floor1_candidate_package"
DST="${{SCENE_ROOT}}/clean_rerun/maps/floor_1"

mkdir -p "${{DST}}"
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.yaml "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.pgm "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.npz "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map_preview.png "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map_provenance.json "${{DST}}"/
```
""",
    )


def json_parse_checks(out: Path) -> dict[str, bool]:
    checks: dict[str, bool] = {}
    for path in sorted(out.rglob("*.json")):
        try:
            json.loads(path.read_text(encoding="utf-8"))
            checks[path.relative_to(out).as_posix()] = True
        except Exception:
            checks[path.relative_to(out).as_posix()] = False
    return checks


def write_reports(
    out: Path,
    preview_result: dict[str, Any],
    endpoint_result: dict[str, Any],
    stable_map_classification: str,
    floor1_classification: str,
    stair_graph_classification: str,
    no_overwrite_checks: dict[str, bool],
    script_compile_passed: bool,
) -> dict[str, Any]:
    floor1_yaml = (
        out
        / "corrected_floor1_candidate_package/stage1_floor_1_stable_occupancy_map.yaml"
    )
    dimension_checks = {}
    floor2_pgm = Path(
        preview_result["corrected_task24b4_floor_2_preview"]["pgm"]
    )
    floor2_yaml = floor2_pgm.with_suffix(".yaml")
    if floor2_yaml.exists():
        dimension_checks["floor_2_regression_package"] = map_dimension_check(
            floor2_yaml
        )
    if floor1_yaml.exists():
        dimension_checks["floor_1_candidate_package"] = map_dimension_check(floor1_yaml)
    json_checks = json_parse_checks(out)
    validation = {
        "created_utc": now_iso(),
        "stable_map_checks": {
            "floor_2_pgm_byte_exact_equals_validated": preview_result[
                "floor_2_pgm_byte_exact_validated_vs_task24b3_corrected"
            ],
            "floor_2_preview_no_longer_matches_wrong_vertical_flip": not preview_result[
                "corrected_floor_2_preview_equals_wrong_vertical_flip_by_class"
            ],
            "floor_2_preview_matches_validated_if_available": preview_result[
                "corrected_floor_2_preview_matches_validated_by_class"
            ],
            "floor_1_preview_uses_same_convention_as_floor_2": preview_result[
                "corrected_task24b4_floor_1_preview"
            ]["best_orientation_match"]
            == preview_result["corrected_task24b4_floor_2_preview"][
                "best_orientation_match"
            ],
            "yaml_pgm_npz_dimensions_consistent": all(
                check["passed"] for check in dimension_checks.values()
            ),
            "clean_rerun_maps_floor_2_unchanged": no_overwrite_checks[
                "clean_rerun_maps_floor_2_unchanged"
            ],
            "clean_rerun_committed_public_unchanged": no_overwrite_checks[
                "clean_rerun_committed_public_unchanged"
            ],
            "task23b_outputs_unchanged": no_overwrite_checks[
                "task23b_outputs_unchanged"
            ],
        },
        "dimension_checks": dimension_checks,
        "stair_graph_checks": endpoint_result,
        "python_compile_check": script_compile_passed,
        "json_parse_checks": json_checks,
        "classifications": {
            "stable_map": stable_map_classification,
            "floor_1_packaging": floor1_classification,
            "stair_graph": stair_graph_classification,
        },
    }
    stable_pass = all(validation["stable_map_checks"].values())
    graph_pass = bool(endpoint_result.get("passed"))
    json_pass = all(json_checks.values())
    validation["overall_passed"] = bool(
        stable_pass and graph_pass and json_pass and script_compile_passed
    )
    write_json(out / "task24b4_report.json", validation)
    write_json(out / "validation_summary.json", validation)
    final_json_checks = json_parse_checks(out)
    validation["json_parse_checks"] = final_json_checks
    validation["overall_passed"] = bool(
        stable_pass
        and graph_pass
        and all(final_json_checks.values())
        and script_compile_passed
    )
    write_json(out / "task24b4_report.json", validation)
    write_json(out / "validation_summary.json", validation)
    json_pass = all(final_json_checks.values())
    write_text(
        out / "validation_summary.md",
        f"""# Validation Summary

Project: RSLG-SLAM.

Task: `{TASK_NAME}`.

Overall passed: `{validation['overall_passed']}`.

## Stable Map Checks

- floor_2 PGM byte-exact equals validated floor_2 PGM: `{validation['stable_map_checks']['floor_2_pgm_byte_exact_equals_validated']}`
- floor_2 corrected preview no longer matches the wrong vertical flip convention: `{validation['stable_map_checks']['floor_2_preview_no_longer_matches_wrong_vertical_flip']}`
- floor_2 corrected preview matches validated preview when available: `{validation['stable_map_checks']['floor_2_preview_matches_validated_if_available']}`
- floor_1 corrected preview uses the same preview convention as floor_2: `{validation['stable_map_checks']['floor_1_preview_uses_same_convention_as_floor_2']}`
- YAML/PGM/NPZ dimensions are consistent: `{validation['stable_map_checks']['yaml_pgm_npz_dimensions_consistent']}`
- clean_rerun/maps/floor_2 unchanged: `{validation['stable_map_checks']['clean_rerun_maps_floor_2_unchanged']}`
- clean_rerun/committed_public unchanged: `{validation['stable_map_checks']['clean_rerun_committed_public_unchanged']}`
- task23b outputs unchanged: `{validation['stable_map_checks']['task23b_outputs_unchanged']}`

## Stair Graph Checks

- endpoint binding node IDs exist: `{endpoint_result.get('stairs_graph_v0_3', {}).get('endpoint_nodes_exist')}`
- edge source/target node IDs exist: `{endpoint_result.get('stairs_graph_v0_3', {}).get('edge_refs_all_exist')}`
- raw trace endpoint refs exist: `{endpoint_result.get('stairs_graph_v0_3', {}).get('raw_trace_endpoint_refs_exist')}`
- physical_execution_supported is false: `{endpoint_result.get('stairs_graph_v0_3', {}).get('physical_execution_supported_is_false')}`
- claim_boundary is `{CLAIM_BOUNDARY}`: `{endpoint_result.get('stairs_graph_v0_3', {}).get('claim_boundary_valid')}`

## Tooling Checks

- Python compile check for new script: `{script_compile_passed}`
- JSON parse checks passed: `{json_pass}`

## Classifications

- stable map: `{stable_map_classification}`
- floor_1 packaging: `{floor1_classification}`
- stair graph: `{stair_graph_classification}`
""",
    )
    write_text(
        out / "task24b4_report.md",
        f"""# Task24b4 Report

Project: RSLG-SLAM.

Task: `{TASK_NAME}`.

Claim boundary: `{CLAIM_BOUNDARY}`.

## Stable Map Preview

The floor_2 runtime PGM remains byte-exact with the validated clean_rerun floor_2 PGM.

The preview bug was in PNG generation only: task24b3 rendered the preview from `flipud(PGM)`. Task24b4 renders preview PNGs as a color remap of the stored PGM image orientation.

Stable map classification: `{stable_map_classification}`.

Floor_1 packaging classification: `{floor1_classification}`.

## Stair Graph Binding

Endpoint bindings now point to fitted centerline nodes that exist in the sparse_stair_connector_graph:

- from: `{endpoint_result.get('stairs_graph_v0_3', {}).get('endpoint_refs', {}).get('from')}`
- to: `{endpoint_result.get('stairs_graph_v0_3', {}).get('endpoint_refs', {}).get('to')}`

Raw ordered_transition_pose_trace endpoint IDs are preserved separately as provenance refs.

Stair graph classification: `{stair_graph_classification}`.

## Validation

Overall passed: `{validation['overall_passed']}`.

No Stage-A, Gazebo, Nav2, Habitat, task23b rerun, or physical stair execution was performed.
""",
    )
    write_text(
        out / "final_answer_for_user.md",
        f"""# Final Answer For User

1. floor_2 runtime PGM still matches the validated clean_rerun PGM: `{preview_result['floor_2_pgm_byte_exact_validated_vs_task24b3_corrected']}`.
2. floor_2 preview orientation is fixed: `{stable_map_classification == 'runtime_pgm_passed_preview_fixed'}`.
3. floor_1 preview was regenerated with the corrected convention: `{floor1_classification == 'floor_1_candidate_ready_for_manual_install'}`.
4. floor_1 candidate package is ready for manual installation: `{floor1_classification == 'floor_1_candidate_ready_for_manual_install'}`.
5. stairs graph endpoint bindings now point to existing fitted centerline nodes: `{stair_graph_classification == 'endpoint_binding_fixed'}`.
6. Generated files are under `{out}`.
7. Exact next recommendation: manually inspect the corrected floor_1 candidate package and, if accepted, run the generated manual copy commands. Do not start task24c until that manual install decision is made.
""",
    )
    return validation


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path("/home/ws/workspace/BoxFusion"),
    )
    parser.add_argument(
        "--scene-root",
        type=Path,
        default=Path(
            "stage_outputs/stage1_generalization/00843-DYehNKdT76V"
        ),
    )
    args = parser.parse_args()
    repo = args.repo.resolve()
    scene_root = (repo / args.scene_root).resolve()
    task24b3 = (
        scene_root
        / "tasks/task24b3_stair_trace_centerline_and_all_floor_map_regression_fix"
    )
    out = scene_root / f"tasks/{TASK_NAME}"
    out.mkdir(parents=True, exist_ok=True)

    protected_before = {
        "clean_rerun_maps_floor_2": hash_tree(scene_root / "clean_rerun/maps/floor_2"),
        "clean_rerun_committed_public": hash_tree(
            scene_root / "clean_rerun/committed_public"
        ),
        "task23b_outputs": hash_tree(
            scene_root
            / "tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"
        ),
    }

    preview_result, stable_map_classification = preview_audit_and_fix(
        scene_root, task24b3, out
    )
    endpoint_result, stair_graph_classification = endpoint_binding_fix(task24b3, out)

    floor1_package = out / "corrected_floor1_candidate_package"
    if stable_map_classification == "runtime_pgm_passed_preview_fixed" and (
        floor1_package / "stage1_floor_1_stable_occupancy_map_preview.png"
    ).exists():
        floor1_classification = "floor_1_candidate_ready_for_manual_install"
    elif floor1_package.exists():
        floor1_classification = "floor_1_candidate_generated_but_preview_failed"
    else:
        floor1_classification = "floor_1_candidate_blocked"

    write_manual_copy_commands(
        out, stable_map_classification, floor1_classification
    )

    protected_after = {
        "clean_rerun_maps_floor_2": hash_tree(scene_root / "clean_rerun/maps/floor_2"),
        "clean_rerun_committed_public": hash_tree(
            scene_root / "clean_rerun/committed_public"
        ),
        "task23b_outputs": hash_tree(
            scene_root
            / "tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"
        ),
    }
    no_overwrite_checks = {
        "clean_rerun_maps_floor_2_unchanged": protected_before[
            "clean_rerun_maps_floor_2"
        ]
        == protected_after["clean_rerun_maps_floor_2"],
        "clean_rerun_committed_public_unchanged": protected_before[
            "clean_rerun_committed_public"
        ]
        == protected_after["clean_rerun_committed_public"],
        "task23b_outputs_unchanged": protected_before["task23b_outputs"]
        == protected_after["task23b_outputs"],
    }
    try:
        py_compile.compile(__file__, doraise=True)
        script_compile_passed = True
    except py_compile.PyCompileError:
        script_compile_passed = False

    validation = write_reports(
        out,
        preview_result,
        endpoint_result,
        stable_map_classification,
        floor1_classification,
        stair_graph_classification,
        no_overwrite_checks,
        script_compile_passed,
    )
    return 0 if validation["overall_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
