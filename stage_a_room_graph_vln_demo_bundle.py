from __future__ import annotations

import argparse
from pathlib import Path

from boxfusion.room_graph_vln_demo_bundle import build_room_graph_vln_demo_bundle, load_demo_bundle_spec


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a presentation-ready room-graph VLN demo bundle from committed/public scene outputs."
    )
    parser.add_argument("--spec", required=True, help="JSON spec describing scenes, queries, and presentation notes.")
    parser.add_argument("--output-root", required=True, help="Directory where the demo bundle should be written.")
    args = parser.parse_args()

    spec_path = Path(args.spec)
    output_root = Path(args.output_root)
    bundle_manifest = build_room_graph_vln_demo_bundle(
        spec=load_demo_bundle_spec(spec_path),
        output_root=output_root,
    )
    print(f"title: {bundle_manifest.get('title')}")
    print(f"scene_count: {len(bundle_manifest.get('scenes') or [])}")
    print(f"bundle_index: {output_root / 'index.html'}")
    print(f"bundle_manifest: {output_root / 'demo_bundle_manifest.json'}")


if __name__ == "__main__":
    main()
