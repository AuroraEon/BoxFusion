from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any, Dict, List, Optional


def load_rows(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append(row)
    return rows


def as_float(value: Any) -> Optional[float]:
    text = str(value or "").strip()
    if not text:
        return None
    return float(text)


def as_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a compact queryability timeline plot from figure-data CSV.")
    parser.add_argument("--figure-data", required=True, help="CSV produced by build_queryability_timeline.py")
    parser.add_argument("--output", required=True, help="PNG output path")
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    rows = load_rows(Path(args.figure_data))
    frames = [int(row["frame_idx"]) for row in rows]
    query_success = [100.0 * float(as_float(row["overall_query_success_rate"]) or 0.0) for row in rows]
    route_found = [100.0 * float(as_float(row["overall_route_found_rate"]) or 0.0) for row in rows]

    fig, ax = plt.subplots(figsize=(8.6, 4.8), dpi=160)
    ax.plot(frames, query_success, label="Query Success", color="#146c94", linewidth=2.4)
    ax.plot(frames, route_found, label="Route Found", color="#cc5803", linewidth=2.4)

    milestone_specs = [
        ("marker_same_floor_provisional", "Same-floor provisional", "#2a9d8f"),
        ("marker_multi_floor_catalog", "Multi-floor catalog", "#8f5ccf"),
        ("marker_cross_floor_reliable", "Reliable cross-floor", "#d1495b"),
    ]
    for field, label, color in milestone_specs:
        for row in rows:
            if not as_bool(row.get(field)):
                continue
            frame = int(row["frame_idx"])
            y = 100.0 * float(as_float(row["overall_query_success_rate"]) or 0.0)
            ax.axvline(frame, color=color, linestyle="--", linewidth=1.2, alpha=0.8)
            ax.scatter([frame], [y], color=color, s=45, zorder=3)
            ax.text(frame + 10, min(99.0, y + 5.0), label, color=color, fontsize=8)

    ax.set_title("Snapshot-Time Queryability Emergence")
    ax.set_xlabel("Frame Index")
    ax.set_ylabel("Rate (%)")
    ax.set_ylim(0.0, 105.0)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(frameon=False, loc="lower right")
    fig.tight_layout()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    print(f"plot_png={output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
