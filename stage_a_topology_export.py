import argparse
from pathlib import Path

from boxfusion.room_topology import RoomTopologyBuilder


def main() -> None:
    parser = argparse.ArgumentParser(description="Build room-centric queryable topology v0.1 from an existing Stage A export.")
    parser.add_argument("--sequence-dir", required=True, help="Path to a Stage A sequence directory, e.g. ./stage_a_outputs/<sequence_id>")
    parser.add_argument("--sequence-id", default=None, help="Optional sequence id override")
    parser.add_argument("--json-out", default=None, help="Optional explicit JSON output path")
    parser.add_argument("--query-report-out", default=None, help="Optional explicit query-report JSON output path")
    parser.add_argument("--graphml-out", default=None, help="Optional explicit GraphML output path")
    args = parser.parse_args()

    sequence_dir = Path(args.sequence_dir)
    topology = RoomTopologyBuilder().build_from_stage_a_sequence_dir(
        sequence_dir,
        sequence_id=args.sequence_id,
    )

    json_out = Path(args.json_out) if args.json_out else sequence_dir / "logs" / "topology_v0_1.json"
    query_report_out = (
        Path(args.query_report_out)
        if args.query_report_out
        else sequence_dir / "logs" / "topology_query_report.json"
    )
    graphml_out = Path(args.graphml_out) if args.graphml_out else sequence_dir / "logs" / "topology_v0_1.graphml"

    topology.export_json(json_out)
    topology.export_query_report(query_report_out)
    topology.export_graphml(graphml_out)

    print(f"topology_json={json_out}")
    print(f"query_report_json={query_report_out}")
    print(f"topology_graphml={graphml_out}")


if __name__ == "__main__":
    main()
