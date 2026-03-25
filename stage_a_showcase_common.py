from __future__ import annotations

import csv
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont


DEFAULT_OUTPUT_ROOT = Path("stage_a_outputs_vt_fallback_v01_rerun2")
DEFAULT_SEQUENCE_IDS = ("00843-DYehNKdT76V", "00847-bCPU9suPUw9")


def load_json(path: Path) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    if not headers:
        return ""
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        cells = []
        for value in row:
            text = "" if value is None else str(value)
            cells.append(text.replace("\n", "<br>"))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def sequence_root(output_root: Path, sequence_id: str) -> Path:
    return Path(output_root) / sequence_id


def sequence_assets(output_root: Path, sequence_id: str) -> Dict[str, Any]:
    root = sequence_root(output_root, sequence_id)
    logs_dir = root / "logs"
    summary = load_json(logs_dir / "summary.json")
    topology = load_json(logs_dir / "topology_v0_1.json")
    floor_diag = load_json(logs_dir / "floor_diagnostics_summary.json")
    vertical = load_json(logs_dir / "vertical_transition_evidence.json")
    presentation_note_path = logs_dir / "presentation_note.md"
    presentation_note = presentation_note_path.read_text(encoding="utf-8") if presentation_note_path.exists() else ""
    return {
        "sequence_id": sequence_id,
        "root": root,
        "logs_dir": logs_dir,
        "summary": summary,
        "topology": topology,
        "floor_diag": floor_diag,
        "vertical": vertical,
        "presentation_note": presentation_note,
    }


def ordered_floors(topology: Dict[str, Any]) -> List[Dict[str, Any]]:
    floors = [dict(item) for item in topology.get("floors", [])]
    return sorted(
        floors,
        key=lambda item: (
            int(item.get("display_order", 10**6) or 10**6),
            float(item.get("z_center", 10**6) or 10**6),
            str(item.get("floor_id") or ""),
        ),
    )


def floor_display_label(item: Dict[str, Any]) -> str:
    return str(item.get("display_floor_id") or item.get("floor_id") or "floor_unknown")


def floor_lookup(topology: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        str(item.get("floor_id")): dict(item)
        for item in ordered_floors(topology)
        if item.get("floor_id") is not None
    }


def room_lookup(topology: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    rooms = {}
    for room in topology.get("rooms", []):
        room_id = room.get("id") or room.get("room_id")
        if room_id is not None:
            rooms[str(room_id)] = dict(room)
    return rooms


def counts_by_floor(topology: Dict[str, Any]) -> Dict[str, Dict[str, int]]:
    floors = {
        str(item.get("floor_id")): {"rooms": 0, "objects": 0, "anchors": 0}
        for item in ordered_floors(topology)
        if item.get("floor_id") is not None
    }
    for room in topology.get("rooms", []):
        floor_id = room.get("floor_id")
        if floor_id is not None:
            floors.setdefault(str(floor_id), {"rooms": 0, "objects": 0, "anchors": 0})
            floors[str(floor_id)]["rooms"] += 1
    for obj in (topology.get("entities") or {}).get("objects", []):
        floor_id = obj.get("floor_id")
        if floor_id is not None:
            floors.setdefault(str(floor_id), {"rooms": 0, "objects": 0, "anchors": 0})
            floors[str(floor_id)]["objects"] += 1
    for anchor in (topology.get("entities") or {}).get("anchors", []):
        floor_id = anchor.get("floor_id")
        if floor_id is not None:
            floors.setdefault(str(floor_id), {"rooms": 0, "objects": 0, "anchors": 0})
            floors[str(floor_id)]["anchors"] += 1
    return floors


def final_room_panel_paths(sequence_dir: Path) -> Dict[str, Path]:
    panels: Dict[str, Tuple[int, Path]] = {}
    for candidate in sequence_dir.glob("debug_room/floor_*/run_*_11_final_rooms.png"):
        floor_id = candidate.parent.name
        stem = candidate.stem
        try:
            run_token = stem.split("_", 2)[1]
            run_idx = int(run_token)
        except (IndexError, ValueError):
            run_idx = -1
        previous = panels.get(floor_id)
        if previous is None or run_idx >= previous[0]:
            panels[floor_id] = (run_idx, candidate)
    return {floor_id: path for floor_id, (_, path) in panels.items()}


def copy_if_exists(src: Path, dst: Path) -> Optional[str]:
    if not src.exists():
        return None
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst)


def trim_markdown(text: str, limit: int = 600) -> str:
    cleaned = " ".join(line.strip() for line in text.splitlines() if line.strip())
    if len(cleaned) <= limit:
        return cleaned
    return cleaned[: limit - 3].rstrip() + "..."


def _font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def build_contact_sheet(
    *,
    title: str,
    subtitle: str,
    panels: Sequence[Tuple[str, str, Path]],
    output_path: Path,
    columns: int = 2,
    tile_width: int = 580,
    image_height: int = 360,
) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    columns = max(1, columns)
    rows = max(1, math.ceil(len(panels) / columns))
    pad = 24
    header_h = 120
    tile_h = image_height + 110
    canvas_w = columns * tile_width + (columns + 1) * pad
    canvas_h = header_h + rows * tile_h + (rows + 1) * pad
    canvas = Image.new("RGB", (canvas_w, canvas_h), color=(248, 246, 240))
    draw = ImageDraw.Draw(canvas)

    draw.rectangle((0, 0, canvas_w, header_h), fill=(31, 60, 87))
    draw.text((pad, 20), title, fill="white", font=_font(34))
    draw.text((pad, 66), subtitle, fill=(223, 232, 240), font=_font(18))

    for index, (label, note, image_path) in enumerate(panels):
        row = index // columns
        col = index % columns
        x0 = pad + col * (tile_width + pad)
        y0 = header_h + pad + row * tile_h
        x1 = x0 + tile_width
        y1 = y0 + tile_h
        draw.rectangle((x0, y0, x1, y1), fill="white", outline=(207, 212, 218), width=2)
        draw.text((x0 + 16, y0 + 14), label, fill=(32, 36, 39), font=_font(24))
        draw.text((x0 + 16, y0 + 52), note, fill=(91, 99, 107), font=_font(16))

        image = Image.open(image_path).convert("RGB")
        image.thumbnail((tile_width - 32, image_height))
        image_x = x0 + (tile_width - image.width) // 2
        image_y = y0 + 92 + max(0, (image_height - image.height) // 2)
        canvas.paste(image, (image_x, image_y))

    canvas.save(output_path)
    return output_path


def print_artifacts(paths: Iterable[Path]) -> None:
    for path in paths:
        print(path.resolve())


@dataclass
class ArtifactSet:
    name: str
    generated_files: List[str]
    summary: Dict[str, Any]
