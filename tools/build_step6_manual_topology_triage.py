"""Build Step 6 manual topology inspection and triage artifacts.

This script is intentionally report-only. It reads committed/public-artifact
visualization outputs from Step 5 and non-authoritative audit overlays from
Step 4, then writes inspection inventories and suggested pre-review triage.
It does not modify topology, routing, Stage-A runtime behavior, or artifact
export semantics.
"""

from __future__ import annotations

import csv
import html
import json
import os
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
STEP4_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step4_topology_audit_batch"
STEP5_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step5_enhanced_vln_visualization"
DOCS_ROOT = REPO_ROOT / "docs"
PACKAGE_ROOT = (
    REPO_ROOT
    / "runtime_stage1_frozen_evidence"
    / "step6_manual_topology_inspection_package"
)

VIS_INV_CSV = DOCS_ROOT / "step6_visualization_output_inventory.csv"
VIS_INV_MD = DOCS_ROOT / "step6_visualization_output_inventory.md"
VISIBLE_CSV = DOCS_ROOT / "step6_candidate_visibility_inventory.csv"
VISIBLE_MD = DOCS_ROOT / "step6_candidate_visibility_inventory.md"
TRIAGE_CSV = DOCS_ROOT / "step6_suggested_candidate_triage.csv"
TRIAGE_MD = DOCS_ROOT / "step6_suggested_candidate_triage.md"
FINAL_CSV = DOCS_ROOT / "step6_manual_visual_inspection_and_topology_candidate_triage.csv"
FINAL_MD = DOCS_ROOT / "step6_manual_visual_inspection_and_topology_candidate_triage.md"
PACKAGE_INDEX = PACKAGE_ROOT / "index.html"
MANUAL_TEMPLATE = PACKAGE_ROOT / "manual_judgment_template.csv"


VIS_INV_FIELDS = [
    "scene_id",
    "query_id",
    "html_path",
    "json_sidecar_path",
    "html_exists",
    "json_exists",
    "json_valid",
    "has_route_edge_explanation",
    "has_gateway_overlay",
    "has_vertical_transition_overlay",
    "has_semantic_room_summary",
    "has_audit_overlay_disclaimer",
    "unresolved_id_warning_count",
    "notes",
]

VISIBLE_FIELDS = [
    "scene_id",
    "candidate_id",
    "candidate_type",
    "rooms_or_edge",
    "source_csv",
    "visible_in_step5_page",
    "html_path",
    "json_sidecar_path",
    "on_selected_route",
    "same_floor_or_cross_floor",
    "has_gateway_match",
    "has_vertical_transition_match",
    "support_count",
    "confidence",
    "severity",
    "evidence_strength",
    "notes",
]

TRIAGE_FIELDS = [
    "scene_id",
    "candidate_id",
    "candidate_type",
    "rooms_or_edge",
    "suggested_triage_label",
    "suggested_recommended_action",
    "paper_risk_pre_review",
    "gazebo_risk_pre_review",
    "reason",
    "needs_human_review",
    "related_html_page",
    "notes",
]

FINAL_FIELDS = [
    "scene_id",
    "candidate_id",
    "candidate_type",
    "rooms_or_edge",
    "source_csv",
    "visible_in_step5_page",
    "on_selected_route",
    "suggested_triage_label",
    "suggested_recommended_action",
    "paper_risk_pre_review",
    "gazebo_risk_pre_review",
    "needs_human_review",
    "related_html_page",
    "notes",
]

MANUAL_FIELDS = [
    "scene_id",
    "candidate_id",
    "candidate_type",
    "rooms_or_edge",
    "visual_judgment",
    "manual_confidence",
    "reason",
    "recommended_action",
    "paper_risk_after_review",
    "gazebo_risk_after_review",
    "reviewer_notes",
]


def rel(path: Path | str | None) -> str:
    if not path:
        return ""
    p = Path(path)
    if not p.is_absolute():
        return p.as_posix()
    try:
        return p.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return p.as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def pair_key(a: str | None, b: str | None) -> tuple[str, str]:
    aa = str(a or "")
    bb = str(b or "")
    return tuple(sorted((aa, bb)))


def yn(value: Any) -> str:
    return "true" if bool(value) else "false"


def h(value: Any) -> str:
    return html.escape(str(value if value is not None else ""))


def table(rows: list[dict[str, Any]], fields: list[str], limit: int | None = None) -> str:
    shown = rows[:limit] if limit else rows
    if not shown:
        return "_No rows._\n"
    lines = ["|" + "|".join(fields) + "|", "|" + "|".join(["---"] * len(fields)) + "|"]
    for row in shown:
        vals = []
        for field in fields:
            value = str(row.get(field, "")).replace("\n", " ")
            vals.append(value.replace("|", "\\|"))
        lines.append("|" + "|".join(vals) + "|")
    if limit and len(rows) > limit:
        lines.append(f"\n_Showing {limit} of {len(rows)} rows. See CSV for all rows._")
    return "\n".join(lines) + "\n"


@dataclass(frozen=True)
class QueryPage:
    scene_id: str
    query_id: str
    html_path: Path
    json_path: Path
    data: dict[str, Any]


def load_manifest() -> dict[str, Any]:
    return read_json(STEP5_ROOT / "demo_bundle_manifest.json")


def load_query_pages(manifest: dict[str, Any]) -> tuple[list[QueryPage], dict[str, Path]]:
    pages: list[QueryPage] = []
    scene_indices: dict[str, Path] = {}
    for scene in manifest.get("scenes", []):
        scene_id = scene.get("display_name") or scene.get("scene_id")
        scene_dir = STEP5_ROOT / scene["scene_dir_name"]
        scene_indices[scene_id] = scene_dir / "index.html"
        for query in scene.get("queries", []):
            html_path = scene_dir / query["html_name"]
            json_path = scene_dir / query["json_name"]
            data = read_json(json_path) if json_path.exists() else {}
            pages.append(
                QueryPage(
                    scene_id=scene_id,
                    query_id=query.get("slug") or Path(query["html_name"]).stem,
                    html_path=html_path,
                    json_path=json_path,
                    data=data,
                )
            )
    return pages, scene_indices


def build_visualization_inventory(
    manifest: dict[str, Any], pages: list[QueryPage], scene_indices: dict[str, Path]
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for scene in manifest.get("scenes", []):
        scene_id = scene.get("display_name") or scene.get("scene_id")
        scene_dir = STEP5_ROOT / scene["scene_dir_name"]
        scene_index = scene_indices[scene_id]
        scene_summary = scene_dir / "scene_bundle_summary.json"
        summary_valid = False
        if scene_summary.exists():
            try:
                read_json(scene_summary)
                summary_valid = True
            except json.JSONDecodeError:
                summary_valid = False
        rows.append(
            {
                "scene_id": scene_id,
                "query_id": "__scene_index__",
                "html_path": rel(scene_index),
                "json_sidecar_path": rel(scene_summary),
                "html_exists": yn(scene_index.exists()),
                "json_exists": yn(scene_summary.exists()),
                "json_valid": yn(summary_valid),
                "has_route_edge_explanation": "false",
                "has_gateway_overlay": "false",
                "has_vertical_transition_overlay": "false",
                "has_semantic_room_summary": "false",
                "has_audit_overlay_disclaimer": yn(
                    scene_index.exists()
                    and "not authoritative" in scene_index.read_text(encoding="utf-8", errors="ignore").lower()
                ),
                "unresolved_id_warning_count": "",
                "notes": "per-scene Step 5 index page",
            }
        )

    for page in pages:
        html_text = page.html_path.read_text(encoding="utf-8", errors="ignore") if page.html_path.exists() else ""
        data = page.data
        features = data.get("enhanced_features", {})
        disclaimer = data.get("audit_overlay_disclaimer") or ""
        rows.append(
            {
                "scene_id": page.scene_id,
                "query_id": page.query_id,
                "html_path": rel(page.html_path),
                "json_sidecar_path": rel(page.json_path),
                "html_exists": yn(page.html_path.exists()),
                "json_exists": yn(page.json_path.exists()),
                "json_valid": yn(bool(data)),
                "has_route_edge_explanation": yn(
                    features.get("include_route_edge_explanation") and data.get("route_edge_explanation") is not None
                ),
                "has_gateway_overlay": yn(
                    features.get("include_gateway_overlays") and data.get("gateway_overlay_records") is not None
                ),
                "has_vertical_transition_overlay": yn(
                    features.get("include_vertical_transition_overlays")
                    and data.get("vertical_transition_overlay_records") is not None
                ),
                "has_semantic_room_summary": yn(
                    features.get("include_semantic_room_summary")
                    and data.get("semantic_room_summary_records") is not None
                ),
                "has_audit_overlay_disclaimer": yn(
                    "not authoritative" in disclaimer.lower()
                    or "not authoritative" in html_text.lower()
                    or "not confirmed ground truth" in html_text.lower()
                ),
                "unresolved_id_warning_count": len(data.get("unresolved_id_warnings", [])),
                "notes": "per-query Step 5 enhanced visualization page",
            }
        )
    return rows


def scene_audit_dirs() -> dict[str, Path]:
    dirs = {}
    for path in sorted(STEP4_ROOT.iterdir()):
        if path.is_dir():
            summary = path / "topology_audit_summary.json"
            if summary.exists():
                scene_id = read_json(summary).get("scene_id") or path.name
                dirs[scene_id] = path
    return dirs


def collect_scene_context(
    pages: list[QueryPage],
) -> tuple[
    dict[str, list[QueryPage]],
    dict[str, set[tuple[str, str]]],
    dict[str, set[tuple[str, str]]],
    dict[str, set[tuple[str, str]]],
    dict[str, list[dict[str, Any]]],
]:
    pages_by_scene: dict[str, list[QueryPage]] = defaultdict(list)
    route_pairs: dict[str, set[tuple[str, str]]] = defaultdict(set)
    gateway_pairs: dict[str, set[tuple[str, str]]] = defaultdict(set)
    vertical_pairs: dict[str, set[tuple[str, str]]] = defaultdict(set)
    overlays: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for page in pages:
        pages_by_scene[page.scene_id].append(page)
        for edge in page.data.get("route_edge_explanation", []):
            route_pairs[page.scene_id].add(pair_key(edge.get("source_room"), edge.get("target_room")))
        for gateway in page.data.get("gateway_overlay_records", []):
            gateway_pairs[page.scene_id].add(pair_key(gateway.get("room_a"), gateway.get("room_b")))
        for transition in page.data.get("vertical_transition_overlay_records", []):
            vertical_pairs[page.scene_id].add(pair_key(transition.get("room_a"), transition.get("room_b")))
        for overlay in page.data.get("audit_overlay_records", []):
            item = dict(overlay)
            item["_html_path"] = page.html_path
            item["_json_path"] = page.json_path
            overlays[page.scene_id].append(item)
    return pages_by_scene, route_pairs, gateway_pairs, vertical_pairs, overlays


def overlay_match(
    candidate_type: str,
    room_a: str,
    room_b: str,
    relation_type: str,
    overlay: dict[str, Any],
) -> bool:
    if overlay.get("candidate_type") != candidate_type:
        return False
    if pair_key(room_a, room_b) != pair_key(overlay.get("room_a"), overlay.get("room_b")):
        return False
    if candidate_type == "low_support_edge":
        return (overlay.get("relation_type") or "") == (relation_type or "")
    return True


def first_visible_overlay(
    scene_overlays: list[dict[str, Any]],
    candidate_type: str,
    room_a: str,
    room_b: str,
    relation_type: str = "",
) -> dict[str, Any] | None:
    for overlay in scene_overlays:
        if overlay_match(candidate_type, room_a, room_b, relation_type, overlay):
            return overlay
    return None


def evidence_strength(candidate_type: str, on_route: bool, has_gateway: bool, has_vertical: bool) -> str:
    if "route_consistency" in candidate_type:
        return "strong"
    if candidate_type in {
        "missing_endpoint_edge",
        "self_loop_edge",
        "cross_floor_non_vertical_edge",
        "vertical_transition_missing_record",
    }:
        return "strong"
    if candidate_type == "low_support_edge" and on_route:
        return "medium"
    if candidate_type == "missing_edge_candidate_neighbor_mismatch":
        return "medium" if not (has_gateway or has_vertical) else "strong"
    if candidate_type == "geometry_close_no_edge":
        return "weak"
    return "medium"


def build_candidates(
    audit_dirs: dict[str, Path],
    pages: list[QueryPage],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    (
        _pages_by_scene,
        route_pairs,
        gateway_pairs,
        vertical_pairs,
        overlays,
    ) = collect_scene_context(pages)
    candidates: list[dict[str, Any]] = []
    metadata: dict[str, Any] = {
        "route_consistency_rows": [],
        "id_normalization": {},
        "edge_audit_warning_count_by_scene": {},
    }

    for scene_id, audit_dir in sorted(audit_dirs.items()):
        edge_rows = read_csv(audit_dir / "topology_edge_audit.csv")
        edge_by_index = {row.get("edge_id_or_index"): row for row in edge_rows}
        edge_warnings = [row for row in edge_rows if row.get("severity") != "ok"]
        metadata["edge_audit_warning_count_by_scene"][scene_id] = len(edge_warnings)

        id_summary_path = audit_dir / "topology_id_normalization_summary.json"
        if id_summary_path.exists():
            metadata["id_normalization"][scene_id] = read_json(id_summary_path)

        route_rows = read_csv(audit_dir / "topology_route_consistency.csv")
        metadata["route_consistency_rows"].extend(route_rows)

        for row in read_csv(audit_dir / "topology_spurious_edge_candidates.csv"):
            room_a = row.get("source_room", "")
            room_b = row.get("target_room", "")
            relation_type = row.get("relation_type", "")
            pair = pair_key(room_a, room_b)
            edge_row = edge_by_index.get(row.get("edge_index"), {})
            visible = first_visible_overlay(
                overlays.get(scene_id, []),
                row.get("candidate_type", ""),
                room_a,
                room_b,
                relation_type,
            )
            on_route = pair in route_pairs.get(scene_id, set())
            has_gateway = row.get("has_gateway_match") == "True" or pair in gateway_pairs.get(scene_id, set())
            has_vertical = row.get("has_vertical_transition_match") == "True" or pair in vertical_pairs.get(scene_id, set())
            cross = edge_row.get("is_cross_floor") == "True"
            candidates.append(
                {
                    "scene_id": scene_id,
                    "candidate_id": f"spurious_edge_{row.get('edge_index')}",
                    "candidate_type": row.get("candidate_type", ""),
                    "rooms_or_edge": f"{room_a} - {room_b} ({relation_type}, edge_index={row.get('edge_index')})",
                    "source_csv": rel(audit_dir / "topology_spurious_edge_candidates.csv"),
                    "visible_in_step5_page": yn(visible),
                    "html_path": rel(visible.get("_html_path")) if visible else "",
                    "json_sidecar_path": rel(visible.get("_json_path")) if visible else "",
                    "on_selected_route": yn(on_route),
                    "same_floor_or_cross_floor": "cross_floor" if cross else "same_floor",
                    "has_gateway_match": yn(has_gateway),
                    "has_vertical_transition_match": yn(has_vertical),
                    "support_count": row.get("support_count", ""),
                    "confidence": row.get("confidence", ""),
                    "severity": row.get("severity", ""),
                    "evidence_strength": evidence_strength(row.get("candidate_type", ""), on_route, has_gateway, has_vertical),
                    "notes": row.get("notes", ""),
                    "_room_a": room_a,
                    "_room_b": room_b,
                    "_relation_type": relation_type,
                }
            )

        missing_rows = read_csv(audit_dir / "topology_missing_edge_candidates.csv")
        for index, row in enumerate(missing_rows, start=1):
            room_a = row.get("room_a", "")
            room_b = row.get("room_b", "")
            pair = pair_key(room_a, room_b)
            visible = first_visible_overlay(
                overlays.get(scene_id, []),
                row.get("candidate_type", ""),
                room_a,
                room_b,
            )
            has_gateway = row.get("has_gateway_record") == "True" or pair in gateway_pairs.get(scene_id, set())
            has_vertical = row.get("has_vertical_transition_record") == "True" or pair in vertical_pairs.get(scene_id, set())
            cross = row.get("floor_a") and row.get("floor_b") and row.get("floor_a") != row.get("floor_b")
            candidates.append(
                {
                    "scene_id": scene_id,
                    "candidate_id": f"missing_edge_{index:03d}",
                    "candidate_type": row.get("candidate_type", ""),
                    "rooms_or_edge": f"{room_a} - {room_b}",
                    "source_csv": rel(audit_dir / "topology_missing_edge_candidates.csv"),
                    "visible_in_step5_page": yn(visible),
                    "html_path": rel(visible.get("_html_path")) if visible else "",
                    "json_sidecar_path": rel(visible.get("_json_path")) if visible else "",
                    "on_selected_route": yn(pair in route_pairs.get(scene_id, set())),
                    "same_floor_or_cross_floor": "cross_floor" if cross else "same_floor",
                    "has_gateway_match": yn(has_gateway),
                    "has_vertical_transition_match": yn(has_vertical),
                    "support_count": "",
                    "confidence": "",
                    "severity": row.get("severity", ""),
                    "evidence_strength": evidence_strength(
                        row.get("candidate_type", ""),
                        pair in route_pairs.get(scene_id, set()),
                        has_gateway,
                        has_vertical,
                    ),
                    "notes": row.get("notes", ""),
                    "_room_a": room_a,
                    "_room_b": room_b,
                    "_relation_type": "",
                    "_evidence_source": row.get("evidence_source", ""),
                    "_has_world_model_neighbor_relation": row.get("has_world_model_neighbor_relation", ""),
                }
            )

        represented_spurious = {candidate["candidate_id"].replace("spurious_edge_", "") for candidate in candidates if candidate["scene_id"] == scene_id}
        for row in edge_warnings:
            edge_index = row.get("edge_id_or_index", "")
            if edge_index in represented_spurious:
                continue
            room_a = row.get("source_room", "")
            room_b = row.get("target_room", "")
            pair = pair_key(room_a, room_b)
            has_gateway = row.get("has_gateway_match") == "True" or pair in gateway_pairs.get(scene_id, set())
            has_vertical = row.get("has_vertical_transition_match") == "True" or pair in vertical_pairs.get(scene_id, set())
            visible = first_visible_overlay(
                overlays.get(scene_id, []),
                "low_support_edge",
                room_a,
                room_b,
                row.get("relation_type", ""),
            )
            candidates.append(
                {
                    "scene_id": scene_id,
                    "candidate_id": f"edge_audit_issue_{edge_index}",
                    "candidate_type": "edge_audit_warning",
                    "rooms_or_edge": f"{room_a} - {room_b} ({row.get('relation_type', '')}, edge_index={edge_index})",
                    "source_csv": rel(audit_dir / "topology_edge_audit.csv"),
                    "visible_in_step5_page": yn(visible),
                    "html_path": rel(visible.get("_html_path")) if visible else "",
                    "json_sidecar_path": rel(visible.get("_json_path")) if visible else "",
                    "on_selected_route": yn(pair in route_pairs.get(scene_id, set())),
                    "same_floor_or_cross_floor": "cross_floor" if row.get("is_cross_floor") == "True" else "same_floor",
                    "has_gateway_match": yn(has_gateway),
                    "has_vertical_transition_match": yn(has_vertical),
                    "support_count": row.get("support_count", ""),
                    "confidence": row.get("confidence", ""),
                    "severity": row.get("severity", ""),
                    "evidence_strength": evidence_strength("edge_audit_warning", pair in route_pairs.get(scene_id, set()), has_gateway, has_vertical),
                    "notes": row.get("notes", ""),
                    "_room_a": room_a,
                    "_room_b": room_b,
                    "_relation_type": row.get("relation_type", ""),
                }
            )

        for row in route_rows:
            if row.get("severity") == "ok":
                continue
            room_a = row.get("source_room", "")
            room_b = row.get("target_room", "")
            candidates.append(
                {
                    "scene_id": scene_id,
                    "candidate_id": f"route_consistency_{row.get('route_id', '')}",
                    "candidate_type": "route_consistency_warning",
                    "rooms_or_edge": f"{room_a} -> {room_b}",
                    "source_csv": rel(audit_dir / "topology_route_consistency.csv"),
                    "visible_in_step5_page": "false",
                    "html_path": "",
                    "json_sidecar_path": "",
                    "on_selected_route": "true",
                    "same_floor_or_cross_floor": "unknown",
                    "has_gateway_match": "false",
                    "has_vertical_transition_match": "false",
                    "support_count": "",
                    "confidence": "",
                    "severity": row.get("severity", ""),
                    "evidence_strength": "strong",
                    "notes": row.get("notes", ""),
                    "_room_a": room_a,
                    "_room_b": room_b,
                    "_relation_type": "",
                }
            )

    return candidates, metadata


def suggest_triage(candidate: dict[str, Any]) -> dict[str, Any]:
    ctype = candidate["candidate_type"]
    on_route = candidate["on_selected_route"] == "true"
    has_gateway = candidate["has_gateway_match"] == "true"
    has_vertical = candidate["has_vertical_transition_match"] == "true"
    cross_floor = candidate["same_floor_or_cross_floor"] == "cross_floor"
    severity = candidate.get("severity", "")

    if ctype == "route_consistency_warning" or severity in {"error", "fatal"}:
        return {
            "label": "possible_topology_export_issue",
            "action": "investigate_topology_export",
            "paper": "high",
            "gazebo": "high",
            "reason": "Route consistency warning/error is a strong structural signal and should be investigated before robot-facing use.",
            "needs_human_review": "true",
        }

    if cross_floor and not has_vertical:
        return {
            "label": "possible_vertical_transition_issue",
            "action": "investigate_vertical_transition_generation",
            "paper": "medium",
            "gazebo": "high",
            "reason": "Candidate crosses floors without visible vertical-transition evidence.",
            "needs_human_review": "true",
        }

    if ctype == "missing_edge_candidate_neighbor_mismatch":
        if not has_gateway and not has_vertical:
            return {
                "label": "neighbor_semantics_investigation",
                "action": "investigate_world_model_neighbor_semantics",
                "paper": "medium",
                "gazebo": "medium",
                "reason": "Committed room-world neighbor/connectivity relation has no committed topology edge and no gateway or vertical-transition match.",
                "needs_human_review": "true",
            }
        return {
            "label": "possible_topology_export_issue",
            "action": "investigate_topology_export",
            "paper": "medium",
            "gazebo": "high",
            "reason": "Neighbor/connectivity candidate has gateway or vertical evidence but no committed topology edge.",
            "needs_human_review": "true",
        }

    if ctype == "geometry_close_no_edge":
        if not has_gateway and not has_vertical and not on_route:
            return {
                "label": "weak_heuristic_ignore_for_now",
                "action": "defer_to_future_branch",
                "paper": "low",
                "gazebo": "low",
                "reason": "Geometry-close/no-edge is centroid or proximity-only evidence with no gateway, transition, object, or selected-route support.",
                "needs_human_review": "false",
            }
        return {
            "label": "unclear_needs_human_review",
            "action": "improve_visualization_only",
            "paper": "medium",
            "gazebo": "medium",
            "reason": "Geometry candidate has additional context and should be visually checked rather than repaired automatically.",
            "needs_human_review": "true",
        }

    if ctype == "low_support_edge":
        if on_route and not has_gateway and not has_vertical:
            return {
                "label": "possible_gateway_generation_issue",
                "action": "investigate_gateway_generation",
                "paper": "medium",
                "gazebo": "medium",
                "reason": "Low-support same-floor committed edge appears on a selected route without gateway or vertical-transition evidence.",
                "needs_human_review": "true",
            }
        if on_route:
            return {
                "label": "visualization_only",
                "action": "improve_caption_or_explanation",
                "paper": "medium",
                "gazebo": "medium",
                "reason": "Low-support committed edge is route-visible; explanation should make clear it is a committed edge plus non-authoritative audit overlay.",
                "needs_human_review": "true",
            }
        return {
            "label": "likely_acceptable",
            "action": "no_action_needed",
            "paper": "low",
            "gazebo": "low",
            "reason": "Low-support committed edge is not on selected routes and has no cross-floor structural inconsistency in the audit.",
            "needs_human_review": "false",
        }

    return {
        "label": "unclear_needs_human_review",
        "action": "defer_to_future_branch",
        "paper": "unknown",
        "gazebo": "unknown",
        "reason": "Candidate type is not covered by the conservative pre-review rule set.",
        "needs_human_review": "true",
    }


def build_triage_rows(candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for candidate in candidates:
        suggestion = suggest_triage(candidate)
        rows.append(
            {
                "scene_id": candidate["scene_id"],
                "candidate_id": candidate["candidate_id"],
                "candidate_type": candidate["candidate_type"],
                "rooms_or_edge": candidate["rooms_or_edge"],
                "suggested_triage_label": suggestion["label"],
                "suggested_recommended_action": suggestion["action"],
                "paper_risk_pre_review": suggestion["paper"],
                "gazebo_risk_pre_review": suggestion["gazebo"],
                "reason": suggestion["reason"],
                "needs_human_review": suggestion["needs_human_review"],
                "related_html_page": candidate["html_path"],
                "notes": candidate["notes"],
            }
        )
    return rows


def candidate_instruction(candidate: dict[str, Any], triage: dict[str, Any]) -> str:
    ctype = candidate["candidate_type"]
    if ctype == "low_support_edge":
        if candidate["on_selected_route"] == "true":
            return "Open the linked Step 5 route page, inspect whether the route-edge explanation, room summaries, and gateway/transition overlays make this committed low-support edge understandable. Do not remove or repair the edge from this package."
        return "Spot-check whether the low-support committed edge is visually plausible as a non-route relation. Treat it as a candidate only, not as a confirmed topology error."
    if ctype == "missing_edge_candidate_neighbor_mismatch":
        return "Compare room-summary neighbor text with the floor-separated topology view. If the relation looks semantic or lifecycle-derived rather than navigable, mark it as schema/semantics mismatch."
    if ctype == "geometry_close_no_edge":
        return "Check only if time permits. This is proximity-only evidence; do not let it drive repair without stronger route, gateway, transition, or semantic evidence."
    return f"Review using the conservative suggested label {triage['suggested_triage_label']} and record a manual judgment in the template."


def css() -> str:
    return """
body { font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 0; color: #172033; background: #f7f8fa; }
header { background: #263238; color: white; padding: 24px 32px; }
main { padding: 24px 32px; }
section { margin: 0 0 28px; }
table { border-collapse: collapse; width: 100%; background: white; font-size: 13px; }
th, td { border: 1px solid #d6dbe1; padding: 7px 8px; text-align: left; vertical-align: top; }
th { background: #eef2f5; }
.note { background: #fff8df; border: 1px solid #ead48a; padding: 10px 12px; margin: 12px 0; }
.pill { display: inline-block; padding: 2px 6px; border-radius: 999px; background: #e8eef8; }
a { color: #0b5cad; }
code { background: #eef2f5; padding: 1px 4px; border-radius: 3px; }
"""


def html_table(rows: list[dict[str, Any]], fields: list[str]) -> str:
    if not rows:
        return "<p>No candidate rows.</p>"
    out = ["<table><thead><tr>"]
    out += [f"<th>{h(field)}</th>" for field in fields]
    out.append("</tr></thead><tbody>")
    for row in rows:
        out.append("<tr>")
        for field in fields:
            value = row.get(field, "")
            out.append(f"<td>{h(value)}</td>")
        out.append("</tr>")
    out.append("</tbody></table>")
    return "".join(out)


def relative_href(from_file: Path, target_rel: str) -> str:
    target = REPO_ROOT / target_rel
    try:
        return Path(rel(target.relative_to(from_file.parent))).as_posix()
    except Exception:
        return rel(Path(target).resolve())


def make_link(from_file: Path, target_rel: str, label: str | None = None) -> str:
    if not target_rel:
        return ""
    target = REPO_ROOT / target_rel
    href_text = os.path.relpath(target.resolve(), from_file.parent.resolve())
    return f'<a href="{h(href_text)}">{h(label or target_rel)}</a>'


def build_inspection_package(candidates: list[dict[str, Any]], triage_rows: list[dict[str, Any]], pages: list[QueryPage]) -> None:
    PACKAGE_ROOT.mkdir(parents=True, exist_ok=True)
    by_scene: dict[str, list[dict[str, Any]]] = defaultdict(list)
    triage_by_key = {(row["scene_id"], row["candidate_id"]): row for row in triage_rows}
    pages_by_scene: dict[str, list[QueryPage]] = defaultdict(list)
    for page in pages:
        pages_by_scene[page.scene_id].append(page)
    for candidate in candidates:
        row = dict(candidate)
        triage = triage_by_key[(candidate["scene_id"], candidate["candidate_id"])]
        row.update(
            {
                "suggested_triage_label": triage["suggested_triage_label"],
                "suggested_recommended_action": triage["suggested_recommended_action"],
                "inspection_instruction": candidate_instruction(candidate, triage),
            }
        )
        by_scene[candidate["scene_id"]].append(row)

    scene_links = []
    for scene_id, scene_rows in sorted(by_scene.items()):
        scene_file = PACKAGE_ROOT / f"{scene_id.lower()}_inspection.html"
        scene_links.append((scene_id, scene_file.name, len(scene_rows)))
        type_counts = Counter(row["candidate_type"] for row in scene_rows)
        page_links = "".join(
            f"<li>{make_link(scene_file, rel(page.html_path), page.query_id)} "
            f"({make_link(scene_file, rel(page.json_path), 'JSON sidecar')})</li>"
            for page in pages_by_scene.get(scene_id, [])
        )
        sections = []
        for ctype, _count in sorted(type_counts.items()):
            rows = [row for row in scene_rows if row["candidate_type"] == ctype]
            sections.append(
                f"<section><h2>{h(ctype)}</h2>"
                + html_table(
                    rows,
                    [
                        "candidate_id",
                        "rooms_or_edge",
                        "visible_in_step5_page",
                        "on_selected_route",
                        "has_gateway_match",
                        "has_vertical_transition_match",
                        "suggested_triage_label",
                        "suggested_recommended_action",
                        "html_path",
                        "inspection_instruction",
                    ],
                )
                + "</section>"
            )
        write_text(
            scene_file,
            "<!doctype html><html><head><meta charset='utf-8'>"
            f"<title>Step 6 Manual Inspection - {h(scene_id)}</title><style>{css()}</style></head><body>"
            f"<header><h1>Step 6 Manual Inspection: {h(scene_id)}</h1>"
            "<p>Candidate overlays are non-authoritative visual aids only. Do not treat them as confirmed topology errors.</p></header>"
            "<main>"
            f"<p>{make_link(scene_file, rel(PACKAGE_INDEX), 'Back to package index')} | "
            f"{make_link(scene_file, rel(MANUAL_TEMPLATE), 'Manual judgment template')}</p>"
            "<section><h2>Relevant Step 5 Pages</h2><ul>"
            + page_links
            + "</ul></section>"
            "<section><h2>Recommended Review Boundary</h2>"
            "<div class='note'>Use the linked Step 5 floor-separated topology view, route edge explanation, gateway overlays, vertical-transition overlays, semantic summaries, and audit overlay disclaimer. Do not edit topology, routing, artifact exports, or audit candidates during this review.</div>"
            "</section>"
            + "".join(sections)
            + "</main></body></html>",
        )

    index_rows = [
        {
            "scene_id": scene_id,
            "inspection_page": f'<a href="{h(filename)}">{h(filename)}</a>',
            "candidate_count": count,
        }
        for scene_id, filename, count in scene_links
    ]
    index_table = "<table><thead><tr><th>scene_id</th><th>inspection_page</th><th>candidate_count</th></tr></thead><tbody>"
    for row in index_rows:
        index_table += f"<tr><td>{h(row['scene_id'])}</td><td>{row['inspection_page']}</td><td>{h(row['candidate_count'])}</td></tr>"
    index_table += "</tbody></table>"

    write_text(
        PACKAGE_INDEX,
        "<!doctype html><html><head><meta charset='utf-8'>"
        f"<title>Step 6 Manual Topology Inspection Package</title><style>{css()}</style></head><body>"
        "<header><h1>Step 6 Manual Topology Inspection Package</h1>"
        "<p>Organized links and candidate metadata for human review. Audit overlays are not authoritative topology.</p></header>"
        "<main>"
        "<section><h2>Scope</h2>"
        "<div class='note'>This package consumes Step 5 enhanced visualization outputs and Step 4 audit outputs only. It does not add graph semantics, repair topology, change routing, implement BEV, or prepare ROS/Gazebo bridge code.</div>"
        f"<p>{make_link(PACKAGE_INDEX, rel(MANUAL_TEMPLATE), 'Open manual judgment template')}</p>"
        "</section>"
        "<section><h2>Scene Pages</h2>"
        + index_table
        + "</section>"
        "<section><h2>Allowed Manual Values</h2>"
        "<p><strong>visual_judgment:</strong> acceptable_committed_topology, likely_visualization_false_positive, likely_schema_semantics_mismatch, possible_missing_connectivity, possible_spurious_connectivity, unclear_needs_more_evidence, weak_heuristic_ignore_for_now</p>"
        "<p><strong>recommended_action:</strong> no_action_needed, improve_caption_or_explanation, improve_visualization_only, investigate_world_model_neighbor_semantics, investigate_topology_export, investigate_gateway_generation, investigate_vertical_transition_generation, investigate_room_segmentation_or_lifecycle, defer_to_future_branch</p>"
        "<p><strong>paper/gazebo risk:</strong> low, medium, high, blocker, unknown</p>"
        "</section>"
        "</main></body></html>",
    )

    manual_rows = [
        {
            "scene_id": candidate["scene_id"],
            "candidate_id": candidate["candidate_id"],
            "candidate_type": candidate["candidate_type"],
            "rooms_or_edge": candidate["rooms_or_edge"],
            "visual_judgment": "",
            "manual_confidence": "",
            "reason": "",
            "recommended_action": "",
            "paper_risk_after_review": "",
            "gazebo_risk_after_review": "",
            "reviewer_notes": "",
        }
        for candidate in candidates
    ]
    write_csv(MANUAL_TEMPLATE, MANUAL_FIELDS, manual_rows)


def summary_counts(rows: list[dict[str, Any]], field: str) -> str:
    counts = Counter(row.get(field, "") for row in rows)
    return ", ".join(f"{key}: {value}" for key, value in sorted(counts.items()))


def risk_counts(rows: list[dict[str, Any]]) -> Counter:
    c = Counter()
    for row in rows:
        c[f"paper_{row['paper_risk_pre_review']}"] += 1
        c[f"gazebo_{row['gazebo_risk_pre_review']}"] += 1
    return c


def build_markdown_reports(
    vis_rows: list[dict[str, Any]],
    candidates: list[dict[str, Any]],
    triage_rows: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> None:
    query_rows = [row for row in vis_rows if row["query_id"] != "__scene_index__"]
    scene_index_rows = [row for row in vis_rows if row["query_id"] == "__scene_index__"]
    all_json_valid = all(row["json_valid"] == "true" for row in vis_rows if row["json_exists"] == "true")
    all_disclaimers = all(row["has_audit_overlay_disclaimer"] == "true" for row in query_rows)
    screenshot_note = (
        "Screenshot capture was skipped: no readily available browser or HTML-to-image command "
        "was found in the environment. Open the HTML package manually."
    )

    write_text(
        VIS_INV_MD,
        "# Step 6 Visualization Output Inventory\n\n"
        "This inventory inspects the Step 5 enhanced visualization bundle without changing any runtime or topology files.\n\n"
        f"- Step 5 scene index pages found: {len(scene_index_rows)}\n"
        f"- Step 5 query pages found: {len(query_rows)}\n"
        f"- JSON sidecars valid where present: {'yes' if all_json_valid else 'no'}\n"
        f"- Query pages retaining audit overlay disclaimer: {'yes' if all_disclaimers else 'no'}\n"
        f"- Screenshot status: {screenshot_note}\n\n"
        "## Inventory Preview\n\n"
        + table(vis_rows, VIS_INV_FIELDS, limit=20)
        + f"\nFull CSV: `{rel(VIS_INV_CSV)}`\n",
    )

    visible_count = sum(1 for row in candidates if row["visible_in_step5_page"] == "true")
    route_count = sum(1 for row in candidates if row["on_selected_route"] == "true")
    write_text(
        VISIBLE_MD,
        "# Step 6 Candidate Visibility Inventory\n\n"
        "Candidate rows are aggregated from Step 4 audit outputs and matched to Step 5 JSON sidecars. "
        "Visibility means the candidate appears in at least one enhanced visualization page as a non-authoritative audit overlay.\n\n"
        f"- Candidate issue rows: {len(candidates)}\n"
        f"- Visible in Step 5 pages: {visible_count}\n"
        f"- On a selected Step 5 route by room pair: {route_count}\n"
        f"- Candidate types: {summary_counts(candidates, 'candidate_type')}\n"
        f"- Evidence strengths: {summary_counts(candidates, 'evidence_strength')}\n\n"
        "## Candidate Preview\n\n"
        + table(candidates, VISIBLE_FIELDS, limit=30)
        + f"\nFull CSV: `{rel(VISIBLE_CSV)}`\n",
    )

    by_scene_lines = []
    for scene_id in sorted({row["scene_id"] for row in triage_rows}):
        rows = [row for row in triage_rows if row["scene_id"] == scene_id]
        by_scene_lines.append(f"- {scene_id}: {summary_counts(rows, 'suggested_triage_label')}")
    by_type_lines = []
    for ctype in sorted({row["candidate_type"] for row in triage_rows}):
        rows = [row for row in triage_rows if row["candidate_type"] == ctype]
        by_type_lines.append(f"- {ctype}: {summary_counts(rows, 'suggested_triage_label')}")

    write_text(
        TRIAGE_MD,
        "# Step 6 Suggested Candidate Triage\n\n"
        "These labels are conservative pre-review suggestions, not ground-truth decisions. "
        "Final human-judgment fields remain blank in the manual inspection template.\n\n"
        "## Summary By Scene\n\n"
        + "\n".join(by_scene_lines)
        + "\n\n## Summary By Candidate Type\n\n"
        + "\n".join(by_type_lines)
        + "\n\n## Triage Preview\n\n"
        + table(triage_rows, TRIAGE_FIELDS, limit=30)
        + f"\nFull CSV: `{rel(TRIAGE_CSV)}`\n",
    )

    final_rows = []
    triage_by_key = {(row["scene_id"], row["candidate_id"]): row for row in triage_rows}
    for candidate in candidates:
        triage = triage_by_key[(candidate["scene_id"], candidate["candidate_id"])]
        final_rows.append(
            {
                "scene_id": candidate["scene_id"],
                "candidate_id": candidate["candidate_id"],
                "candidate_type": candidate["candidate_type"],
                "rooms_or_edge": candidate["rooms_or_edge"],
                "source_csv": candidate["source_csv"],
                "visible_in_step5_page": candidate["visible_in_step5_page"],
                "on_selected_route": candidate["on_selected_route"],
                "suggested_triage_label": triage["suggested_triage_label"],
                "suggested_recommended_action": triage["suggested_recommended_action"],
                "paper_risk_pre_review": triage["paper_risk_pre_review"],
                "gazebo_risk_pre_review": triage["gazebo_risk_pre_review"],
                "needs_human_review": triage["needs_human_review"],
                "related_html_page": triage["related_html_page"],
                "notes": candidate["notes"],
            }
        )

    route_ok = all(row.get("severity") == "ok" for row in metadata.get("route_consistency_rows", []))
    id_warning_total = 0
    for item in metadata.get("id_normalization", {}).values():
        id_warning_total += len(item.get("warnings", []))
        id_warning_total += int(item.get("unresolved_gateway_endpoints", 0) or 0)
        id_warning_total += int(item.get("unresolved_vertical_transition_endpoints", 0) or 0)
        id_warning_total += int(item.get("unresolved_object_room_assignments", 0) or 0)

    label_counts = Counter(row["suggested_triage_label"] for row in triage_rows)
    route_risk_rows = [
        row
        for row in final_rows
        if row["on_selected_route"] == "true"
        and row["suggested_triage_label"]
        in {"possible_gateway_generation_issue", "possible_topology_export_issue", "possible_vertical_transition_issue", "visualization_only"}
    ]
    next_step = "Option 1: Topology export / gateway / vertical-transition investigation."
    next_step_reason = (
        "Selected because low-support committed-edge candidates appear on selected Step 5 routes, "
        "including route-visible cases without gateway or vertical-transition matches. Route consistency itself is ok, "
        "but this repeated medium-risk explanation/evidence pattern should be investigated before minimal robot-facing bridge work."
    )
    if not route_risk_rows and label_counts.get("possible_topology_export_issue", 0) == 0 and label_counts.get("possible_gateway_generation_issue", 0) == 0:
        if label_counts.get("visualization_only", 0):
            next_step = "Option 3: Improve visualization/captions only."
            next_step_reason = "Selected because pre-review risks are dominated by explanation wording rather than topology/export signals."
        else:
            next_step = "Option 2: Prepare minimal Gazebo/ROS bridge."
            next_step_reason = "Selected because no high/blocker pre-review topology risks remain and candidates are mostly weak heuristics or non-route low-support edges."

    safe_rows = [row for row in triage_rows if row["suggested_triage_label"] == "likely_acceptable"]
    weak_rows = [row for row in triage_rows if row["suggested_triage_label"] == "weak_heuristic_ignore_for_now"]
    human_rows = [row for row in triage_rows if row["needs_human_review"] == "true"]
    topology_rows = [
        row
        for row in triage_rows
        if row["suggested_triage_label"]
        in {"possible_topology_export_issue", "possible_gateway_generation_issue", "possible_vertical_transition_issue"}
    ]
    neighbor_rows = [row for row in triage_rows if row["suggested_triage_label"] == "neighbor_semantics_investigation"]

    write_text(
        FINAL_MD,
        "# Step 6 Manual Visual Inspection and Topology Candidate Triage\n\n"
        "## 1. Executive Summary\n\n"
        "Step 6 produced a manual inspection package and conservative pre-review triage over Step 4 audit candidates visible through the Step 5 enhanced visualization bundle. "
        "No topology repair, Stage-A runtime change, topology construction change, artifact export change, BEV work, or ROS/Gazebo bridge work was performed. "
        "Audit candidates remain non-authoritative and are not treated as confirmed topology errors.\n\n"
        f"- Candidate issue rows triaged: {len(candidates)}\n"
        f"- Visible in Step 5 enhanced pages: {visible_count}\n"
        f"- Candidate rows on selected Step 5 route pairs: {route_count}\n"
        f"- Route consistency audit rows all ok: {'yes' if route_ok else 'no'}\n"
        f"- ID-normalization unresolved/warning count: {id_warning_total}\n\n"
        "## 2. Inputs Consumed\n\n"
        f"- Step 4 audit root: `{rel(STEP4_ROOT)}`\n"
        f"- Step 5 visualization root: `{rel(STEP5_ROOT)}`\n"
        "- Step 5 manifest, per-scene indexes, per-query HTML pages, and per-query JSON sidecars\n"
        "- Step 4 topology_edge_audit.csv, topology_missing_edge_candidates.csv, topology_spurious_edge_candidates.csv, topology_route_consistency.csv, and topology_id_normalization_summary.json\n\n"
        "## 3. Artifact Boundary Confirmation\n\n"
        "Only committed/public artifact-backed Step 5 visualization outputs and optional Step 4 audit overlays were used. "
        "The report does not use working topology, sidecar shadow export, online lifecycle state, room-scoped runtime state, legacy demo outputs, SegFormer room segmentation, Tier-2 repair, or explicit room rebirth/re-identification as authoritative truth.\n\n"
        "## 4. Step 5 Visualization Output Inventory Summary\n\n"
        f"- Scene index rows: {len(scene_index_rows)}\n"
        f"- Query visualization rows: {len(query_rows)}\n"
        f"- All listed JSON sidecars parse: {'yes' if all_json_valid else 'no'}\n"
        f"- Audit overlay disclaimer present on linked query pages: {'yes' if all_disclaimers else 'no'}\n"
        f"- Inventory CSV: `{rel(VIS_INV_CSV)}`\n\n"
        "## 5. Candidate Visibility Summary\n\n"
        f"- Candidate types: {summary_counts(candidates, 'candidate_type')}\n"
        f"- Evidence strengths: {summary_counts(candidates, 'evidence_strength')}\n"
        f"- Visibility CSV: `{rel(VISIBLE_CSV)}`\n\n"
        "## 6. Suggested Triage Summary By Scene\n\n"
        + "\n".join(by_scene_lines)
        + "\n\n## 7. Suggested Triage Summary By Candidate Type\n\n"
        + "\n".join(by_type_lines)
        + "\n\n## 8. Manual Inspection Package Location\n\n"
        f"- Package index: `{rel(PACKAGE_INDEX)}`\n"
        f"- Manual judgment template: `{rel(MANUAL_TEMPLATE)}`\n"
        f"- Screenshot status: {screenshot_note}\n\n"
        "## 9. Candidate Classes That Appear Safe / Low Risk Before Human Review\n\n"
        f"- `likely_acceptable`: {len(safe_rows)} low-support committed-edge candidates that are not on selected Step 5 routes and have no cross-floor structural inconsistency.\n\n"
        "## 10. Candidate Classes That Need Human Visual Review\n\n"
        f"- Human-review suggested rows: {len(human_rows)}\n"
        f"- Neighbor-semantics candidates: {len(neighbor_rows)}\n"
        f"- Route-visible low-support/explanation candidates: {len(route_risk_rows)}\n\n"
        "## 11. Candidate Classes That May Require Topology/Export Investigation\n\n"
        f"- Topology/export/gateway/vertical-transition investigation rows: {len(topology_rows)}\n"
        "- Strong route consistency and ID-normalization structural checks are clean in the current audit bundle, but route-visible low-support edges without gateway evidence remain medium-risk for explanation and downstream bridge preparation.\n\n"
        "## 12. Candidate Classes That Should Not Drive Repair\n\n"
        f"- `weak_heuristic_ignore_for_now`: {len(weak_rows)} geometry-close/no-edge rows. These are proximity-only audit candidates and should not drive topology repair without stronger evidence.\n\n"
        "## 13. Readiness For Minimal Gazebo/ROS Bridge Preparation\n\n"
        "The system is not blocked by route-consistency errors or ID-normalization unresolved endpoint warnings, and the Step 5 visualization bundle is stable enough for manual inspection. "
        "However, repeated route-visible low-support committed edges without gateway matches create medium pre-review risk for robot-facing demonstration. "
        "Minimal Gazebo/ROS bridge preparation should wait until the route-visible topology/export/gateway explanation pattern is inspected.\n\n"
        "## 14. Recommended Next Step\n\n"
        f"{next_step}\n\n"
        f"Reason: {next_step_reason}\n\n"
        f"Final candidate CSV: `{rel(FINAL_CSV)}`\n",
    )

    write_csv(FINAL_CSV, FINAL_FIELDS, final_rows)


def main() -> None:
    manifest = load_manifest()
    pages, scene_indices = load_query_pages(manifest)
    vis_rows = build_visualization_inventory(manifest, pages, scene_indices)
    audit_dirs = scene_audit_dirs()
    candidates, metadata = build_candidates(audit_dirs, pages)
    triage_rows = build_triage_rows(candidates)
    build_inspection_package(candidates, triage_rows, pages)

    write_csv(VIS_INV_CSV, VIS_INV_FIELDS, vis_rows)
    write_csv(VISIBLE_CSV, VISIBLE_FIELDS, candidates)
    write_csv(TRIAGE_CSV, TRIAGE_FIELDS, triage_rows)
    build_markdown_reports(vis_rows, candidates, triage_rows, metadata)

    print(f"Wrote {rel(VIS_INV_CSV)}")
    print(f"Wrote {rel(VISIBLE_CSV)}")
    print(f"Wrote {rel(TRIAGE_CSV)}")
    print(f"Wrote {rel(FINAL_CSV)}")
    print(f"Wrote {rel(PACKAGE_INDEX)}")
    print(f"Wrote {rel(MANUAL_TEMPLATE)}")


if __name__ == "__main__":
    main()
