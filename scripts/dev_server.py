"""
Local dev server for the Smart Route & ETA Engine demo pages.

Serves frontend/ as static files and adds the dynamic endpoints the demo
needs. Checkpoints 3-4 added /compute and /compute_k, which shell out to
the already-built route_engine.exe so a click triggers a real computation.
Checkpoint 5 adds, in front of those same subprocess calls:

- a server-side set of live road conditions (closed / congested edges),
  passed to the engine on every call via its existing --closed= flag, and
- an LRU cache of computed results, so a repeat query skips the subprocess
  entirely.

The C++ engine is still a one-shot process - it is deliberately NOT turned
into a persistent server. Everything stateful lives here.

Endpoints
    GET  /compute      ?start_lat&start_lon&end_lat&end_lon
    GET  /compute_k    ?start_lat&start_lon&end_lat&end_lon&k&algo
    GET  /conditions            -> current closed/congested set
    POST /conditions/mark       {lat, lon, kind: closed|congested, multiplier}
    POST /conditions/clear
    GET  /cache/stats           -> hit/miss/eviction counters

THREADING: this uses a single-threaded HTTPServer on purpose. The cache and
conditions store are not concurrency-safe (see route_cache.py), so the
server must not handle requests in parallel. Checkpoint 3-4 used
ThreadingHTTPServer; that was downgraded here so the documented
single-threaded assumption is actually enforced rather than merely assumed.

Run once, from the eta/ project root:
    python scripts\\dev_server.py
then open http://localhost:8765
"""
import http.server
import json
import subprocess
import sys
import time
import urllib.parse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from graph_index import GraphIndex, haversine_m  # noqa: E402
from route_cache import CLOSED, CacheEntry, LiveConditions, LruRouteCache, extract_edge_paths  # noqa: E402

PORT = 8765
PROJECT_ROOT = Path(__file__).resolve().parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"
ROUTE_ENGINE = PROJECT_ROOT / "cpp" / "build" / "route_engine.exe"
NODES_CSV = PROJECT_ROOT / "data" / "nodes.csv"
EDGES_CSV = PROJECT_ROOT / "data" / "edges.csv"
ROUTE_JSON = FRONTEND_DIR / "route.json"
STEPS_JSON = FRONTEND_DIR / "steps.json"
ROUTE_K_JSON = FRONTEND_DIR / "route_k.json"

# Files each endpoint's engine invocation produces. A cache entry stores
# the contents of exactly these, so a hit can restore them without running
# anything.
COMPUTE_ALGOS = ("dijkstra", "astar", "bidijkstra")
COMPUTE_ROUTE_FILES = [f"route_{a}.json" for a in COMPUTE_ALGOS]
COMPUTE_STEP_FILES = [f"steps_{a}.json" for a in COMPUTE_ALGOS]
COMPUTE_FILES = COMPUTE_ROUTE_FILES + COMPUTE_STEP_FILES

# Process-wide state. Safe as plain globals only because the server is
# single-threaded - see the module docstring.
GRAPH = None
CONDITIONS = LiveConditions()
CACHE = LruRouteCache()


def read_frontend_files(names):
    out = {}
    for name in names:
        path = FRONTEND_DIR / name
        if path.exists():
            out[name] = path.read_text(encoding="utf-8")
    return out


def write_frontend_files(files):
    for name, content in files.items():
        (FRONTEND_DIR / name).write_text(content, encoding="utf-8")


def run_engine(extra_args, output_path, trace_path=None):
    """Runs route_engine.exe with the current live conditions applied."""
    cmd = [
        str(ROUTE_ENGINE), str(NODES_CSV), str(EDGES_CSV),
        *extra_args,
        str(output_path),
    ]
    if trace_path:
        cmd.append(f"--trace={trace_path}")
    closed_arg = CONDITIONS.to_cli_arg()
    if closed_arg:
        cmd.append(f"--closed={closed_arg}")
    return subprocess.run(cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True, timeout=60)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(FRONTEND_DIR), **kwargs)

    # ---------------- routing ----------------

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        params = urllib.parse.parse_qs(parsed.query)
        if parsed.path == "/compute":
            self.handle_compute(params)
        elif parsed.path == "/compute_k":
            self.handle_compute_k(params)
        elif parsed.path == "/conditions":
            self.send_json(200, {"ok": True, "conditions": CONDITIONS.as_list()})
        elif parsed.path == "/cache/stats":
            self.send_json(200, {"ok": True, "cache": CACHE.stats()})
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/conditions/mark":
            self.handle_mark_condition(self.read_json_body())
        elif parsed.path == "/conditions/clear":
            self.handle_clear_conditions()
        else:
            self.send_error(404, "No such endpoint")

    def read_json_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    # ---------------- route computation (cached) ----------------

    def snap_endpoints(self, params):
        """lat/lon query params -> snapped OSM node ids.

        Done here in Python rather than by reading the engine's stderr,
        because the cache key needs the node ids *before* we decide whether
        to run the engine at all. Verified to agree with the engine's own
        snap - see scripts/verify_snap_matches_engine.py.
        """
        start_lat = float(params["start_lat"][0])
        start_lon = float(params["start_lon"][0])
        end_lat = float(params["end_lat"][0])
        end_lon = float(params["end_lon"][0])
        start_node, _ = GRAPH.snap(start_lat, start_lon)
        end_node, _ = GRAPH.snap(end_lat, end_lon)
        return (start_lat, start_lon, end_lat, end_lon, start_node, end_node)

    def handle_compute(self, params):
        # Timer starts before snapping, not after: snapping is real work
        # this request pays for (~40k haversine evaluations per endpoint),
        # and it happens on hits too. Starting the clock after it would
        # report a hit as far cheaper than it actually is.
        t0 = time.perf_counter()
        try:
            slat, slon, elat, elon, start_node, end_node = self.snap_endpoints(params)
        except (KeyError, ValueError, IndexError):
            self.send_error(400, "start_lat/start_lon/end_lat/end_lon must all be numeric query params")
            return

        # Logged unconditionally (not just on the same-node path below) so
        # the raw click payload is always inspectable, not just when
        # something looks wrong - the point of this log line is to answer
        # "what did the browser actually send" without having to guess.
        self.log_request_snap("compute", slat, slon, elat, elon, start_node, end_node)

        if start_node == end_node:
            self.send_json(422, self.same_node_response(slat, slon, elat, elon, start_node))
            return

        key = (start_node, end_node, "both", "distance", 1)

        cached = CACHE.get(key)
        if cached is not None:
            write_frontend_files(cached.files)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            self.log_cache("HIT ", key, elapsed_ms)
            self.send_json(200, {
                "ok": True, "cached": True, "elapsed_ms": round(elapsed_ms, 1),
                "log": cached.meta.get("log", ""),
            })
            return

        result = run_engine(
            [str(slat), str(slon), str(elat), str(elon), "--algo=both", "--weight=distance"],
            ROUTE_JSON, STEPS_JSON,
        )
        if result.returncode != 0:
            self.send_json(500, {"ok": False, "error": result.stderr.strip() or "route_engine.exe failed"})
            return

        files = read_frontend_files(COMPUTE_FILES)
        edge_paths = []
        for name in COMPUTE_ROUTE_FILES:
            if name in files:
                edge_paths.extend(extract_edge_paths(json.loads(files[name])))
        CACHE.put(key, CacheEntry(files, edge_paths, {"log": result.stderr.strip()}))

        elapsed_ms = (time.perf_counter() - t0) * 1000
        self.log_cache("MISS", key, elapsed_ms)
        self.send_json(200, {
            "ok": True, "cached": False, "elapsed_ms": round(elapsed_ms, 1),
            "log": result.stderr.strip(),
        })

    def handle_compute_k(self, params):
        t0 = time.perf_counter()  # before snapping - see handle_compute()
        try:
            slat, slon, elat, elon, start_node, end_node = self.snap_endpoints(params)
            k = int(params.get("k", ["3"])[0])
        except (KeyError, ValueError, IndexError):
            self.send_error(400, "start_lat/start_lon/end_lat/end_lon must be numeric; k must be an integer")
            return

        self.log_request_snap("compute_k", slat, slon, elat, elon, start_node, end_node)

        if start_node == end_node:
            self.send_json(422, self.same_node_response(slat, slon, elat, elon, start_node))
            return

        algo = params.get("algo", ["astar"])[0]
        if algo not in ("dijkstra", "astar", "bidijkstra"):
            self.send_error(400, "algo must be dijkstra, astar, or bidijkstra")
            return

        key = (start_node, end_node, algo, "distance", k)

        cached = CACHE.get(key)
        if cached is not None:
            write_frontend_files(cached.files)
            elapsed_ms = (time.perf_counter() - t0) * 1000
            self.log_cache("HIT ", key, elapsed_ms)
            self.send_json(200, {
                "ok": True, "cached": True, "elapsed_ms": round(elapsed_ms, 1),
                "log": cached.meta.get("log", ""),
            })
            return

        result = run_engine(
            [str(slat), str(slon), str(elat), str(elon), f"--algo={algo}", "--weight=distance", f"--k={k}"],
            ROUTE_K_JSON,
        )
        if result.returncode != 0:
            self.send_json(500, {"ok": False, "error": result.stderr.strip() or "route_engine.exe failed"})
            return

        files = read_frontend_files(["route_k.json"])
        edge_paths = []
        if "route_k.json" in files:
            edge_paths = extract_edge_paths(json.loads(files["route_k.json"]))
        CACHE.put(key, CacheEntry(files, edge_paths, {"log": result.stderr.strip()}))

        elapsed_ms = (time.perf_counter() - t0) * 1000
        self.log_cache("MISS", key, elapsed_ms)
        self.send_json(200, {
            "ok": True, "cached": False, "elapsed_ms": round(elapsed_ms, 1),
            "log": result.stderr.strip(),
        })

    # ---------------- live road conditions ----------------

    def handle_mark_condition(self, body):
        try:
            lat = float(body["lat"])
            lon = float(body["lon"])
        except (KeyError, TypeError, ValueError):
            self.send_error(400, "body must be JSON with numeric lat and lon")
            return

        kind = body.get("kind", "closed")
        if kind == "closed":
            multiplier = CLOSED
        else:
            try:
                multiplier = float(body.get("multiplier", 3.0))
            except (TypeError, ValueError):
                multiplier = 3.0
            if multiplier < 1.0:
                multiplier = 1.0  # mirrors the engine's clamp; < 1 breaks A* admissibility

        edge = GRAPH.nearest_edge(lat, lon)
        if edge is None:
            self.send_json(404, {"ok": False, "error": "no road found near that point"})
            return

        # A real-world closure blocks the street in both directions, and our
        # graph models each direction as its own edge, so mark the reverse
        # too when it exists (a genuine one-way street simply has none).
        targets = [(edge["u"], edge["v"])]
        if GRAPH.has_edge(edge["v"], edge["u"]):
            targets.append((edge["v"], edge["u"]))

        effects = [CONDITIONS.mark(u, v, multiplier) for u, v in targets]

        # Precise invalidation for tightening changes; a relax has no
        # edge_path-membership argument behind it and needs a full flush.
        # See route_cache.py / NOTES.md.
        if "relax" in effects:
            dropped = CACHE.flush()
            invalidation = {"mode": "flush", "entries_dropped": dropped}
        else:
            evicted = []
            for u, v in targets:
                evicted.extend(CACHE.invalidate_edge(u, v))
            invalidation = {"mode": "precise", "entries_dropped": len(evicted),
                            "entries_kept": len(CACHE)}

        sys.stderr.write(
            f"[conditions] {kind} {targets} -> cache {invalidation['mode']}, "
            f"dropped {invalidation['entries_dropped']}, {len(CACHE)} entr(ies) left\n")

        self.send_json(200, {
            "ok": True,
            "kind": kind,
            "multiplier": None if multiplier == CLOSED else multiplier,
            "edges": [{"u": u, "v": v} for u, v in targets],
            "u_coord": edge["u_coord"],
            "v_coord": edge["v_coord"],
            "distance_m": round(edge["distance_m"], 1),
            "invalidation": invalidation,
            "conditions": CONDITIONS.as_list(),
        })

    def handle_clear_conditions(self):
        had = CONDITIONS.clear()
        # Clearing *relaxes* the graph: routes that detoured around a
        # closure may no longer be optimal, and those routes do not contain
        # the re-opened edge, so edge_path membership cannot find them.
        # A full flush is the only correct response here.
        dropped = CACHE.flush()
        sys.stderr.write(f"[conditions] cleared {had} condition(s) -> cache flushed, dropped {dropped}\n")
        self.send_json(200, {
            "ok": True, "cleared": had,
            "invalidation": {"mode": "flush", "entries_dropped": dropped},
            "conditions": [],
        })

    # ---------------- plumbing ----------------

    def log_request_snap(self, endpoint, slat, slon, elat, elon, start_node, end_node):
        """Logs exactly what a /compute[_k] request carried and resolved to.

        Exists so "what coordinates did the browser actually send" is a log
        line, not a guess - added specifically because the previous
        investigation of this class of bug had no visibility into the raw
        click payload versus the snapped node ids, which made "did two
        different clicks collapse onto one node" unanswerable without this.
        """
        flag = "  <-- SAME NODE" if start_node == end_node else ""
        sys.stderr.write(
            f"[{endpoint}] click: start=({slat:.6f},{slon:.6f}) end=({elat:.6f},{elon:.6f}) "
            f"-> snapped: start_node={start_node} end_node={end_node}{flag}\n")

    def same_node_response(self, slat, slon, elat, elon, node):
        """Both clicks resolved to the identical graph node.

        Not a crash and not necessarily a mistaken click: two visually
        distinct points can legitimately be nearer to the same intersection
        than to any other one, especially in sparsely-mapped areas (large
        campuses, rural roads, anywhere OSM has few road segments) where
        snap-to-node's effective "capture radius" around a single node can
        span tens of meters. Running the engine anyway would produce a
        technically-correct-but-useless answer - a 1-node, 0-cost path
        (source==target was deliberately verified to behave exactly this
        way, correctly, at the engine layer - see NOTES.md checkpoint 5).
        The fix belongs at this layer, in front of the engine, not inside
        it: say plainly what happened instead of rendering a route that
        looks like "1 points, 0.00 km".
        """
        click_distance_m = haversine_m(slat, slon, elat, elon)
        node_lat, node_lon = GRAPH.coord_by_id[node]
        sys.stderr.write(
            f"[compute] REJECTED: both points snapped to node {node} "
            f"(raw clicks were {click_distance_m:.1f} m apart)\n")
        return {
            "ok": False,
            "same_node": True,
            "node": node,
            "node_coord": [node_lat, node_lon],
            "click_distance_m": round(click_distance_m, 1),
            "error": (
                f"Both points snapped to the same road intersection - your clicks were "
                f"{click_distance_m:.0f} m apart, but the nearest mapped road node to both "
                f"is the same one (likely a sparsely-mapped area with few nearby "
                f"intersections). Pick two points further apart, or closer to different roads."
            ),
        }

    def log_cache(self, verdict, key, elapsed_ms):
        sys.stderr.write(f"[cache {verdict}] {key} in {elapsed_ms:.0f} ms "
                         f"({len(CACHE)}/{CACHE.max_entries} entries)\n")

    def send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


def main():
    global GRAPH

    if not ROUTE_ENGINE.exists():
        print(f"error: {ROUTE_ENGINE} not found - build it first (see NOTES.md).", file=sys.stderr)
        sys.exit(1)

    print("Loading graph index for snapping / nearest-edge lookup...")
    t0 = time.perf_counter()
    GRAPH = GraphIndex(NODES_CSV, EDGES_CSV)
    print(f"Indexed {GRAPH.node_count()} nodes, {GRAPH.edge_count()} edges "
          f"in {(time.perf_counter() - t0) * 1000:.0f} ms")

    # Single-threaded on purpose: the cache and conditions store are not
    # concurrency-safe. See the module docstring.
    server = http.server.HTTPServer(("localhost", PORT), Handler)
    print(f"Serving {FRONTEND_DIR} at http://localhost:{PORT}  (single-threaded)")
    print(f"Route cache: LRU, max {CACHE.max_entries} entries")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
