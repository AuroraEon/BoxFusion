from __future__ import annotations

import json
import html
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from boxfusion.room_graph_vln_demo import RoomGraphVLNDemo, write_room_graph_vln_demo


def _escape(value: Any) -> str:
    return html.escape("" if value is None else str(value), quote=True)


def _slugify(value: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in str(value).strip().lower())
    safe = "_".join(part for part in safe.split("_") if part)
    return safe or "item"


@dataclass
class BundleQuery:
    slug: str
    title: str
    explanation: str
    start_room: Optional[str] = None
    goal_room: Optional[str] = None
    semantic_target: Optional[str] = None
    include_audit_overlays: Optional[bool] = None
    audit_dir: Optional[Path] = None

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "BundleQuery":
        query = cls(
            slug=str(payload.get("slug") or ""),
            title=str(payload.get("title") or ""),
            explanation=str(payload.get("explanation") or ""),
            start_room=payload.get("start_room"),
            goal_room=payload.get("goal_room"),
            semantic_target=payload.get("semantic_target"),
            include_audit_overlays=payload.get("include_audit_overlays"),
            audit_dir=None if payload.get("audit_dir") is None else Path(str(payload.get("audit_dir"))),
        )
        if not query.slug:
            raise ValueError("Bundle query is missing slug.")
        if not query.title:
            raise ValueError(f"Bundle query {query.slug!r} is missing title.")
        if not query.explanation:
            raise ValueError(f"Bundle query {query.slug!r} is missing explanation.")
        if bool(query.goal_room) == bool(query.semantic_target):
            raise ValueError(f"Bundle query {query.slug!r} must provide exactly one of goal_room or semantic_target.")
        return query


@dataclass
class BundleScene:
    scene_root: Path
    display_name: str
    role: str
    summary: str
    recommended_semantic_whitelist: List[str]
    queries: List[BundleQuery]
    audit_dir: Optional[Path] = None

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "BundleScene":
        scene_root = Path(str(payload.get("scene_root") or ""))
        if not str(scene_root):
            raise ValueError("Bundle scene is missing scene_root.")
        display_name = str(payload.get("display_name") or scene_root.name)
        role = str(payload.get("role") or "supporting")
        summary = str(payload.get("summary") or "")
        whitelist = [str(item) for item in payload.get("recommended_semantic_whitelist", []) if str(item).strip()]
        queries = [BundleQuery.from_dict(dict(item)) for item in payload.get("queries", [])]
        if not queries:
            raise ValueError(f"Bundle scene {display_name!r} does not declare any queries.")
        return cls(
            scene_root=scene_root,
            display_name=display_name,
            role=role,
            summary=summary,
            recommended_semantic_whitelist=whitelist,
            queries=queries,
            audit_dir=None if payload.get("audit_dir") is None else Path(str(payload.get("audit_dir"))),
        )


def _render_scene_index(scene_payload: Dict[str, Any], *, bundle_title: str) -> str:
    whitelist = scene_payload.get("recommended_semantic_whitelist") or []
    query_cards: List[str] = []
    for query in scene_payload.get("queries", []):
        target_text = query.get("goal_room") or query.get("semantic_target")
        query_cards.append(
            "<article class='card'>"
            f"<h2>{_escape(query.get('title'))}</h2>"
            f"<p class='meta'>{_escape(query.get('query_mode'))} | start={_escape(query.get('start_room'))} | target={_escape(target_text)}</p>"
            f"<p>{_escape(query.get('explanation'))}</p>"
            f"<p class='route'>Path: {_escape(' -> '.join(query.get('room_sequence') or []))}</p>"
            f"<p class='route'>Next hop: {_escape(query.get('next_hop'))}</p>"
            "<div class='links'>"
            f"<a href='{_escape(query.get('html_name'))}'>Open HTML</a>"
            f"<a href='{_escape(query.get('json_name'))}'>Open JSON</a>"
            "</div>"
            "</article>"
        )
    whitelist_html = "".join(f"<span class='pill'>{_escape(item)}</span>" for item in whitelist) or "<span class='pill'>none</span>"
    bundle_index_href = scene_payload.get("bundle_index_href") or "../index.html"
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>{_escape(bundle_title)} | {_escape(scene_payload.get("display_name"))}</title>
  <style>
    :root {{
      --bg: #f6f1e8;
      --panel: #fffdf8;
      --ink: #1f2937;
      --muted: #5a6473;
      --line: #dfd5c4;
      --accent: #c8553d;
      --shadow: 0 14px 36px rgba(31, 41, 55, 0.09);
      --font-sans: "Avenir Next", "Segoe UI", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 24px;
      font-family: var(--font-sans);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(242, 193, 78, 0.2), transparent 32%),
        radial-gradient(circle at top right, rgba(90, 169, 230, 0.16), transparent 28%),
        var(--bg);
    }}
    .shell {{
      max-width: 1240px;
      margin: 0 auto;
      display: grid;
      gap: 18px;
    }}
    .hero, .panel, .card {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      box-shadow: var(--shadow);
    }}
    .hero, .panel {{
      padding: 22px;
    }}
    .hero h1 {{
      margin: 0 0 8px 0;
      font-size: 30px;
    }}
    .sub {{
      color: var(--muted);
      max-width: 980px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
      margin-top: 16px;
    }}
    .metric {{
      background: #faf6ed;
      border: 1px solid #ebdfcd;
      border-radius: 16px;
      padding: 14px;
    }}
    .metric .label {{
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
      font-size: 11px;
    }}
    .metric .value {{
      margin-top: 6px;
      font-size: 20px;
      font-weight: 700;
    }}
    .pill-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-top: 12px;
    }}
    .pill {{
      display: inline-flex;
      align-items: center;
      padding: 6px 10px;
      border-radius: 999px;
      background: #f4eee3;
      border: 1px solid #e6d9c7;
      font-size: 12px;
      color: #384657;
    }}
    .cards {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 14px;
    }}
    .card {{
      padding: 18px;
    }}
    .card h2 {{
      margin: 0 0 8px 0;
      font-size: 20px;
    }}
    .meta, .route {{
      color: var(--muted);
      font-size: 14px;
    }}
    .links {{
      display: flex;
      gap: 12px;
      margin-top: 14px;
    }}
    a {{
      color: var(--accent);
      text-decoration: none;
      font-weight: 700;
    }}
    @media (max-width: 900px) {{
      body {{ padding: 14px; }}
    }}
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <h1>{_escape(scene_payload.get("display_name"))}</h1>
      <div class="sub">{_escape(scene_payload.get("summary"))}</div>
      <div class="grid">
        <div class="metric"><div class="label">Role</div><div class="value">{_escape(scene_payload.get("role"))}</div></div>
        <div class="metric"><div class="label">Public Rooms</div><div class="value">{_escape(scene_payload.get("room_count"))}</div></div>
        <div class="metric"><div class="label">Edges</div><div class="value">{_escape(scene_payload.get("edge_count"))}</div></div>
        <div class="metric"><div class="label">Queries</div><div class="value">{_escape(len(scene_payload.get("queries") or []))}</div></div>
      </div>
      <div class="pill-row">{whitelist_html}</div>
    </section>
    <section class="panel">
      <h2>Semantics Note</h2>
      <p class="meta">Committed topology edges are the public downstream graph. Gateway and vertical-transition layers are optional overlays, and audit candidates remain non-authoritative diagnostics.</p>
    </section>
    <section class="panel">
      <h2>Query Pages</h2>
      <div class="cards">
        {''.join(query_cards)}
      </div>
    </section>
    <section class="panel">
      <a href="{_escape(bundle_index_href)}">Back to bundle index</a>
    </section>
  </div>
</body>
</html>
"""


def _render_bundle_index(bundle_manifest: Dict[str, Any]) -> str:
    scene_cards: List[str] = []
    for scene in bundle_manifest.get("scenes", []):
        whitelist_html = "".join(
            f"<span class='pill'>{_escape(item)}</span>" for item in scene.get("recommended_semantic_whitelist") or []
        ) or "<span class='pill'>none</span>"
        scene_cards.append(
            "<article class='card'>"
            f"<h2>{_escape(scene.get('display_name'))}</h2>"
            f"<p class='meta'>{_escape(scene.get('role'))} | public rooms={_escape(scene.get('room_count'))} | edges={_escape(scene.get('edge_count'))}</p>"
            f"<p>{_escape(scene.get('summary'))}</p>"
            f"<div class='pill-row'>{whitelist_html}</div>"
            "<div class='links'>"
            f"<a href='{_escape(scene.get('scene_dir_name'))}/index.html'>Open scene page</a>"
            f"<a href='{_escape(scene.get('scene_dir_name'))}/scene_bundle_summary.json'>Scene JSON</a>"
            "</div>"
            "</article>"
        )
    global_whitelist = "".join(
        f"<span class='pill'>{_escape(item)}</span>"
        for item in bundle_manifest.get("recommended_semantic_whitelist") or []
    ) or "<span class='pill'>none</span>"
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>{_escape(bundle_manifest.get("title"))}</title>
  <style>
    :root {{
      --bg: #f7f2e8;
      --panel: #fffdf9;
      --ink: #1f2937;
      --muted: #5a6473;
      --line: #dfd5c4;
      --accent: #c8553d;
      --shadow: 0 14px 36px rgba(31, 41, 55, 0.09);
      --font-sans: "Avenir Next", "Segoe UI", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 24px;
      font-family: var(--font-sans);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(242, 193, 78, 0.2), transparent 34%),
        radial-gradient(circle at top right, rgba(90, 169, 230, 0.16), transparent 28%),
        var(--bg);
    }}
    .shell {{
      max-width: 1280px;
      margin: 0 auto;
      display: grid;
      gap: 18px;
    }}
    .hero, .panel, .card {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      box-shadow: var(--shadow);
    }}
    .hero, .panel {{
      padding: 22px;
    }}
    .hero h1 {{
      margin: 0 0 8px 0;
      font-size: 32px;
    }}
    .sub {{
      color: var(--muted);
      max-width: 1020px;
    }}
    .pill-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-top: 12px;
    }}
    .pill {{
      display: inline-flex;
      align-items: center;
      padding: 6px 10px;
      border-radius: 999px;
      background: #f4eee3;
      border: 1px solid #e6d9c7;
      font-size: 12px;
      color: #384657;
    }}
    .cards {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
      gap: 14px;
    }}
    .card {{
      padding: 18px;
    }}
    .card h2 {{
      margin: 0 0 8px 0;
      font-size: 20px;
    }}
    .meta {{
      color: var(--muted);
      font-size: 14px;
    }}
    .links {{
      display: flex;
      gap: 12px;
      margin-top: 14px;
    }}
    a {{
      color: var(--accent);
      text-decoration: none;
      font-weight: 700;
    }}
    code {{
      font-family: "SFMono-Regular", Consolas, monospace;
      font-size: 12px;
    }}
    @media (max-width: 900px) {{
      body {{ padding: 14px; }}
    }}
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <h1>{_escape(bundle_manifest.get("title"))}</h1>
      <div class="sub">{_escape(bundle_manifest.get("subtitle"))}</div>
      <div class="pill-row">{global_whitelist}</div>
    </section>
    <section class="panel">
      <h2>Bundle Semantics</h2>
      <p class="meta">Each query page stays on committed/public artifacts. Audit overlays are non-authoritative, same-pair visibility does not imply route selection, and routing behavior is unchanged.</p>
    </section>
    <section class="panel">
      <h2>Scene Pages</h2>
      <div class="cards">
        {''.join(scene_cards)}
      </div>
    </section>
    <section class="panel">
      <a href="demo_bundle_manifest.json">Open bundle manifest JSON</a>
    </section>
  </div>
</body>
</html>
"""


def _scene_output_dir(output_root: Path, scene: BundleScene, *, scenes_subdir: bool = False) -> Path:
    base = output_root / "scenes" if scenes_subdir else output_root
    return base / _slugify(scene.display_name)


def load_demo_bundle_spec(spec_path: Path) -> Dict[str, Any]:
    return json.loads(Path(spec_path).read_text(encoding="utf-8"))


def build_room_graph_vln_demo_bundle(*, spec: Dict[str, Any], output_root: Path) -> Dict[str, Any]:
    title = str(spec.get("title") or "BoxFusion Room-Graph VLN Demo Bundle")
    subtitle = str(spec.get("subtitle") or "")
    bundle_whitelist = [str(item) for item in spec.get("recommended_semantic_whitelist", []) if str(item).strip()]
    enhanced_options = dict(spec.get("enhanced_options") or {})
    scenes_subdir = str(spec.get("output_layout") or "").strip() == "step5_scenes_queries"
    audit_root = None if spec.get("audit_root") is None else Path(str(spec.get("audit_root")))
    scenes = [BundleScene.from_dict(dict(item)) for item in spec.get("scenes", [])]
    if not scenes:
        raise ValueError("Bundle spec does not contain any scenes.")

    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    bundle_manifest: Dict[str, Any] = {
        "title": title,
        "subtitle": subtitle,
        "recommended_semantic_whitelist": bundle_whitelist,
        "scenes": [],
    }

    for scene in scenes:
        demo = RoomGraphVLNDemo.from_inputs(scene_root=scene.scene_root)
        scene_dir = _scene_output_dir(output_root, scene, scenes_subdir=scenes_subdir)
        query_dir = scene_dir / "queries" if scenes_subdir else scene_dir
        scene_dir.mkdir(parents=True, exist_ok=True)
        query_dir.mkdir(parents=True, exist_ok=True)
        scene_audit_dir = scene.audit_dir
        if scene_audit_dir is None and audit_root is not None:
            scene_audit_dir = audit_root / scene.display_name
        scene_payload = {
            "scene_root": str(scene.scene_root),
            "display_name": scene.display_name,
            "role": scene.role,
            "summary": scene.summary,
            "scene_dir_name": f"scenes/{scene_dir.name}" if scenes_subdir else scene_dir.name,
            "bundle_index_href": "../../index.html" if scenes_subdir else "../index.html",
            "recommended_semantic_whitelist": list(scene.recommended_semantic_whitelist),
            "room_count": len(demo.public_room_ids),
            "edge_count": len(demo.topology_payload.get("edges", [])),
            "queries": [],
        }
        for idx, query in enumerate(scene.queries, start=1):
            filename_base = (
                _slugify(query.slug)
                if scenes_subdir
                else f"{demo.sequence_id}_room_graph_vln_demo_{idx:02d}_{_slugify(query.slug)}"
            )
            html_name = f"{filename_base}.html"
            json_name = f"{filename_base}.json"
            html_rel = f"queries/{html_name}" if scenes_subdir else html_name
            json_rel = f"queries/{json_name}" if scenes_subdir else json_name
            query_audit_dir = query.audit_dir or scene_audit_dir
            include_audit = (
                bool(query.include_audit_overlays)
                if query.include_audit_overlays is not None
                else bool(enhanced_options.get("include_audit_overlays", False))
            )
            demo_result = demo.build_demo(
                start_room=query.start_room,
                goal_room=query.goal_room,
                semantic_target=query.semantic_target,
                title=query.title,
                include_snapshot_overlays=bool(enhanced_options.get("include_snapshot_overlays", False)),
                include_gateway_overlays=bool(enhanced_options.get("include_gateway_overlays", False)),
                include_vertical_transition_overlays=bool(
                    enhanced_options.get("include_vertical_transition_overlays", False)
                ),
                include_route_edge_explanation=bool(enhanced_options.get("include_route_edge_explanation", False)),
                include_semantic_room_summary=bool(enhanced_options.get("include_semantic_room_summary", False)),
                include_audit_overlays=include_audit,
                audit_dir=query_audit_dir,
            )
            if not demo_result.get("ok"):
                raise RuntimeError(
                    f"Query {query.slug!r} failed for scene {scene.display_name}: {demo_result.get('failure_reason')}"
                )
            demo_result["presentation"] = {
                "bundle_title": title,
                "scene_display_name": scene.display_name,
                "scene_role": scene.role,
                "query_slug": query.slug,
                "explanation": query.explanation,
            }
            write_room_graph_vln_demo(
                demo_result,
                demo=demo,
                html_out=query_dir / html_name,
                json_out=query_dir / json_name,
            )
            scene_payload["queries"].append(
                {
                    "slug": query.slug,
                    "title": query.title,
                    "explanation": query.explanation,
                    "query_mode": "explicit_room" if query.goal_room is not None else "semantic_room_summary",
                    "start_room": (demo_result.get("start_resolution") or {}).get("resolved_room_id"),
                    "goal_room": query.goal_room,
                    "semantic_target": query.semantic_target,
                    "resolved_goal_room": (demo_result.get("goal_resolution") or {}).get("resolved_room_id"),
                    "room_sequence": list((demo_result.get("route") or {}).get("room_sequence") or []),
                    "next_hop": (demo_result.get("next_hop") or {}).get("room_id"),
                    "html_name": html_rel,
                    "json_name": json_rel,
                }
            )
        (scene_dir / "scene_bundle_summary.json").write_text(json.dumps(scene_payload, indent=2) + "\n", encoding="utf-8")
        (scene_dir / "index.html").write_text(
            _render_scene_index(scene_payload, bundle_title=title),
            encoding="utf-8",
        )
        bundle_manifest["scenes"].append(scene_payload)

    (output_root / "demo_bundle_manifest.json").write_text(json.dumps(bundle_manifest, indent=2) + "\n", encoding="utf-8")
    (output_root / "index.html").write_text(_render_bundle_index(bundle_manifest), encoding="utf-8")
    return bundle_manifest
