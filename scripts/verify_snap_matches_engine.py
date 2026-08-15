"""
Verifies that GraphIndex.snap() (Python) agrees with the C++ engine's
snap_to_nearest_node() on random points.

Why this exists: checkpoint 5's LRU route cache is keyed by
(start_node, end_node, ...), and the Python server resolves those node ids
itself so a cache hit can skip the subprocess entirely. That means the same
"which node is nearest" rule is now implemented twice, in two languages.
If they ever disagree, the cache would key an entry under a node the engine
never actually routed from - silently serving a route for the wrong
endpoints. This script is the check that they don't.

Run from the project root:
    python scripts\\verify_snap_matches_engine.py [num_points]
"""
import random
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from graph_index import GraphIndex  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
NODES_CSV = PROJECT_ROOT / "data" / "nodes.csv"
EDGES_CSV = PROJECT_ROOT / "data" / "edges.csv"
ROUTE_ENGINE = PROJECT_ROOT / "cpp" / "build" / "route_engine.exe"

# Bhubaneswar bbox, same as the exporter geocoded.
SOUTH, NORTH = 20.1002964, 20.4202964
WEST, EAST = 85.6794521, 85.9994521

SNAP_RE = re.compile(r"Snapped (start|end)\s*-> node index (\d+) \(osm_id (\d+)\)")


def main():
    num_points = int(sys.argv[1]) if len(sys.argv) > 1 else 25
    random.seed(1234)  # reproducible point selection

    print(f"Loading graph index (Python)...")
    idx = GraphIndex(NODES_CSV, EDGES_CSV)
    print(f"Loaded {idx.node_count()} nodes, {idx.edge_count()} edges")

    mismatches = 0
    checked = 0

    for i in range(num_points):
        slat = random.uniform(SOUTH, NORTH)
        slon = random.uniform(WEST, EAST)
        elat = random.uniform(SOUTH, NORTH)
        elon = random.uniform(WEST, EAST)

        py_start, _ = idx.snap(slat, slon)
        py_end, _ = idx.snap(elat, elon)

        out_json = PROJECT_ROOT / "scratch_snapcheck.json"
        result = subprocess.run(
            [
                str(ROUTE_ENGINE), str(NODES_CSV), str(EDGES_CSV),
                str(slat), str(slon), str(elat), str(elon), str(out_json),
                "--algo=astar",
            ],
            cwd=str(PROJECT_ROOT), capture_output=True, text=True, timeout=60,
        )

        engine = {}
        for which, _idx_str, osm in SNAP_RE.findall(result.stderr):
            engine[which] = int(osm)

        if "start" not in engine or "end" not in engine:
            print(f"  [{i+1}] could not parse engine snap output - skipping")
            continue

        checked += 1
        ok_start = py_start == engine["start"]
        ok_end = py_end == engine["end"]
        if not (ok_start and ok_end):
            mismatches += 1
            print(f"  [{i+1}] MISMATCH")
            print(f"       point ({slat:.6f},{slon:.6f}) -> python {py_start} vs engine {engine['start']}")
            print(f"       point ({elat:.6f},{elon:.6f}) -> python {py_end} vs engine {engine['end']}")
        else:
            print(f"  [{i+1}] ok  start={py_start}  end={py_end}")

    out_json = PROJECT_ROOT / "scratch_snapcheck.json"
    if out_json.exists():
        out_json.unlink()

    print()
    print(f"Checked {checked} point pairs ({checked * 2} snaps): {mismatches} mismatch(es)")
    return 1 if mismatches else 0


if __name__ == "__main__":
    sys.exit(main())
