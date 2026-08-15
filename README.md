# Smart Route & ETA Engine

A routing service built on real OpenStreetMap road data, comparing multiple pathfinding algorithms (Dijkstra, A*, Bidirectional Dijkstra, Yen's k-shortest-paths) with a live-updating cache layer and a search-visualization demo.

Built and tested against the real road network of Bhubaneswar, India (40,111 nodes, 98,211 directed edges) — configurable to any city (see [Using a different city](#using-a-different-city)).

## Overview

This project computes routes on a real, imported road network rather than a synthetic graph. It supports:

- Four interchangeable pathfinding algorithms behind a common interface
- Two weight strategies — shortest distance, or estimated travel time (from OSM speed-limit tags where available, falling back to per-road-class defaults)
- Yen's algorithm for k-shortest-paths, with a chain-of-responsibility validation layer (reachability, sanity bounds, closed-road checks)
- Live road closures / congestion, applied at request time with no restart or re-export required
- An LRU route cache with precise, edge-path-based invalidation — closing a road only evicts the cache entries that actually use it
- A search-visualization demo animating how each algorithm explores the graph

The C++ engine handles the pathfinding (performance-critical, stateless, one-shot). A Python server handles orchestration — caching, live road conditions, HTTP — and shells out to the compiled engine per request. See [Architecture](#architecture) for why it's split this way.

## Features

- **Dijkstra** — baseline shortest-path search
- **A\*** — haversine-heuristic-guided search
- **Bidirectional Dijkstra** — simultaneous search from both endpoints
- **Yen's k-shortest-paths** — top-k distinct, validated routes
- **Pluggable weight strategies** — distance or estimated time
- **Live edge conditions** — mark roads closed or congested through the UI; routes recompute around them
- **LRU cache with precise invalidation** — keyed by (start, end, algorithm, weight strategy, k); evicts only affected entries on a road closure, flushes on reopening (see [Limitations](#known-limitations) for why these two cases differ)
- **Search visualization** — watch Dijkstra, A*, and Bidirectional Dijkstra explore the same query side by side

## Architecture

```
Browser (Leaflet map + vanilla JS)
        │  HTTP
        ▼
Python dev_server.py (single-threaded)
  - serves the static frontend
  - /compute, /compute_k, /conditions, /cache/stats
  - snaps clicks to graph nodes (own Python implementation,
    verified to agree with the C++ engine's logic)
  - checks its in-process LRU cache
        │  on a cache miss: subprocess call
        ▼
cpp/build/route_engine.exe (stateless, one-shot)
  - loads the graph (CSR format)
  - runs the requested algorithm
  - writes the result to a JSON file, which the server reads back
```

The Python and C++ halves only need to agree on a file format (CSV in, JSON out) — the C++ side stays a stateless, testable computation engine, while the mutable state (cache, live conditions, HTTP handling) lives in Python, where it's faster to iterate on. This does mean the "snap click to nearest road" logic is genuinely duplicated in both languages; that duplication is covered by an explicit verification script (`scripts/verify_snap_matches_engine.py`) rather than assumed to be correct.

## Setup / How to run

**Prerequisites**

- Python 3.12 (standard library only — no `pip install` needed)
- CMake >= 3.15
- A C++14-capable compiler (built and verified against MinGW.org GCC 6.3.0; see [Limitations](#known-limitations) for why not a newer toolchain)

**Steps**

Two things are excluded from this repo (`.gitignore`) and must be generated locally: `data/*.csv` and `cpp/build/`.

```bash
# 1. Generate the road network (queries Nominatim + Overpass APIs, ~30-90s)
python scripts\export_osm.py
# writes data/nodes.csv, data/edges.csv

# 2. Build the C++ engine
cmake -S cpp -B cpp\build -G "MinGW Makefiles" ^
  -DCMAKE_CXX_COMPILER="C:\path\to\your\g++.exe" ^
  -DCMAKE_MAKE_PROGRAM="C:\path\to\your\mingw32-make.exe"
cmake --build cpp\build
# produces cpp\build\route_engine.exe, benchmark.exe, test_validation_rules.exe

# 3. Start the dev server (from the project root)
python scripts\dev_server.py
# single-threaded; loads the graph into memory (~460ms for 40k nodes)

# 4. Open http://localhost:8765 in a browser
```

Adjust the compiler paths in step 2 to wherever MinGW/g++ is actually installed on your machine.

## Using a different city

There's no command-line flag for this — edit one constant in `scripts/export_osm.py`:

```python
PLACE_QUERY = "Bhubaneswar, India"
```

Change it to any place name, e.g. `"Pune, India"`, then re-run the export script and restart the dev server (it loads the graph at startup):

```bash
python scripts\export_osm.py
python scripts\dev_server.py
```

The city name is resolved to a bounding box via Nominatim geocoding at run time — nothing else in the codebase hardcodes Bhubaneswar-specific values.

## Benchmark results

From `benchmark_results.txt` — 40,111 nodes, 98,211 directed edges, 18 random source-target pairs (seed 42).

**Node expansion** (the reliable metric — exact, not timing-dependent):

| Weight strategy | A* vs Dijkstra | Bidirectional Dijkstra vs Dijkstra |
|---|---|---|
| distance | 74.3% fewer nodes | 18.0% fewer nodes |
| time | 53.3% fewer nodes | 24.9% fewer nodes |

**Wall-clock** (noisier — this toolchain's `steady_clock` has ~1ms resolution, so each figure is a mean of 25 repeated runs):

| Weight strategy | A* speedup | Bidirectional Dijkstra speedup |
|---|---|---|
| distance | 0.83x (slower) | 0.98x (roughly even) |
| time | 0.51x (slower) | 1.19x (modest win) |

A* and Bidirectional Dijkstra reliably expand far fewer nodes than plain Dijkstra — that part is exact and reproducible. Wall-clock time doesn't track that 1:1: A* pays a haversine heuristic call per relaxed edge, and Bidirectional Dijkstra runs two priority queues plus a meeting check, so the clock only shows a win once the nodes saved outweigh that per-node overhead. **The honest claim is "expands far fewer nodes; wall-clock benefit is inconsistent on this hardware/toolchain," not "is faster."**

**Cache hit/miss speedup** — two real, conflicting measurements exist for this:

| Measurement | MISS | HIT | Speedup |
|---|---|---|---|
| First session | 833 ms | 86-91 ms | ~9.2x |
| Re-measured later, same code, same query | 833 ms | 265-278 ms | ~4x |

Both are real `curl` calls against the real server; nothing in the cache code changed between them, and the cache's own 27-case test suite passed both times. The likely cause is unrelated background load on the dev machine, not a regression. **Don't cite either number as fixed** — a cache hit is reliably, meaningfully faster than a miss; the exact multiplier varies by machine load and should be re-measured on an idle machine if quoted.

## Known limitations

This is a local demo/learning project, not a production deployment. Specifically:

- `route_engine.exe` is a Windows binary, built with a 2016-era, 32-bit-only compiler (MinGW.org GCC 6.3.0) — not cross-platform, not built for any deployment target.
- The dev server is deliberately single-threaded — the cache and live-conditions store mutate shared state with no locking, so a concurrent/threaded server would be unsafe with the current code as-is.
- Runs on `localhost` only — not deployed anywhere, no HTTPS, no authentication or authorization on any endpoint.
- **Horizontal scaling was designed for, not built.** The current cache is an in-process Python dict — with multiple server instances, each would have its own separate cache, fragmenting hit rate. The documented fix is a shared external cache (e.g. Redis) plus multiple worker processes behind a load balancer. This is a design decision on record, not implemented, since the multi-instance problem it would solve doesn't exist yet at this project's scale.
- **Road closures and congestion are simulated, not live** — entered manually by clicking a road in the UI's "Edit mode." There is no GPS feed, traffic API, or automatic data source of any kind.
- **~24% of graph nodes have real isolation gaps.** 9,794 of 40,111 exported nodes have a 60-250m gap to their nearest road-connected neighbor, because the OSM export filter excludes `highway=service` (the tag used for internal/campus/parking roads). A concrete example: IIT Bhubaneswar's campus interior has a 2,002m gap to the nearest graph node, since its internal roads are entirely untagged in the exported graph. Practical effect: two clicks that look clearly distinct on the map can snap to the same graph node. This specific failure mode is now caught explicitly (an HTTP 422 error instead of a fake zero-distance route), but the underlying data coverage gap itself is not fixed.
## Tech stack

- **Python 3.12** — standard library only (`urllib`, `json`, `math`, `csv`, `http.server`); no external packages
- **C++14** — compiled with MinGW.org GCC 6.3.0
- **CMake** >= 3.15
- **Leaflet 1.9.4** — via CDN, no build step, no bundler
- **OpenStreetMap / Overpass API** — source of the road network data
- **Windows** — the build is currently Windows-specific (MinGW paths, `.exe` output)
