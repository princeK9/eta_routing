"""
Checkpoint 5: the live-conditions store and the LRU route cache that sits
in front of route_engine.exe.

Architecture (fixed, not incidental): the cache lives here, in the Python
dev-server layer, wrapping the existing subprocess calls. A HIT skips the
subprocess entirely and serves stored output; a MISS runs
route_engine.exe exactly as before. The C++ engine stays a one-shot
process - it is deliberately NOT a persistent server.

CONCURRENCY ASSUMPTION: single-threaded request handling. Neither class
below takes a lock, and both mutate shared state (OrderedDict ordering,
the conditions dict) during a request. dev_server.py runs a
single-threaded HTTPServer specifically so this assumption actually holds
rather than being merely hoped for. Making this concurrency-safe is
separate scope - see NOTES.md.
"""
import math
from collections import OrderedDict

# Bounded to a fixed entry count. See NOTES.md for the sizing argument and
# the measured per-entry footprint behind this number.
CACHE_MAX_ENTRIES = 64

CLOSED = float("inf")


class LiveConditions:
    """Server-side set of closed/congested edges: (u_osm, v_osm) -> multiplier.

    `CLOSED` (infinity) means impassable; a finite value > 1.0 scales that
    edge's cost. Edges are identified by OSM node-id pair - the same stable
    identity the engine's --closed= flag and route JSON's edge_path_osm use
    - rather than by CSR edge index, which is an internal artifact of a
    particular Graph::load run and means nothing across a process boundary.
    """

    def __init__(self):
        self._by_edge = {}

    def __len__(self):
        return len(self._by_edge)

    def multiplier(self, u, v):
        return self._by_edge.get((u, v), 1.0)

    def mark(self, u, v, multiplier):
        """Apply a condition to edge (u, v).

        Returns "tighten" if this made the edge strictly more expensive (or
        newly restricted), "relax" if it made it cheaper than it already
        was, and "noop" if nothing changed. The caller uses that to decide
        between precise cache invalidation and a full flush - see
        NOTES.md for why the relax direction can't use precise
        invalidation.
        """
        key = (u, v)
        old = self._by_edge.get(key, 1.0)
        if multiplier == old:
            return "noop"
        self._by_edge[key] = multiplier
        return "tighten" if multiplier > old else "relax"

    def clear(self):
        had = len(self._by_edge)
        self._by_edge.clear()
        return had

    def to_cli_arg(self):
        """Renders the whole set as the engine's --closed= value.

        Grammar (checkpoint 4 extended in checkpoint 5): "u:v" = closed,
        "u:v:multiplier" = congested. Returns "" when there's nothing to
        send, so the caller can omit the flag entirely.
        """
        parts = []
        for (u, v), mult in self._by_edge.items():
            if mult == CLOSED or math.isinf(mult):
                parts.append(f"{u}:{v}")
            else:
                parts.append(f"{u}:{v}:{mult}")
        return ",".join(parts)

    def as_list(self):
        """JSON-friendly view for the UI."""
        out = []
        for (u, v), mult in self._by_edge.items():
            out.append({
                "u": u,
                "v": v,
                "kind": "closed" if (mult == CLOSED or math.isinf(mult)) else "congested",
                "multiplier": None if (mult == CLOSED or math.isinf(mult)) else mult,
            })
        return out


class CacheEntry:
    """One cached computation: the output files, plus the routes' edge paths.

    `edge_paths` is the list of per-route edge paths (each a list of
    (u, v) OSM pairs) exactly as the engine reported them - this is the
    "cache entry must store the edge_path(s)" requirement. `edge_set` is
    the same data flattened into a set, kept alongside purely so
    invalidation is an O(1) membership test per entry instead of an O(path
    length) scan.
    """

    __slots__ = ("files", "edge_paths", "edge_set", "meta")

    def __init__(self, files, edge_paths, meta=None):
        self.files = files
        self.edge_paths = edge_paths
        self.edge_set = {edge for path in edge_paths for edge in path}
        self.meta = meta or {}

    def uses_edge(self, u, v):
        return (u, v) in self.edge_set

    def approx_bytes(self):
        return sum(len(name) + len(content) for name, content in self.files.items())


class LruRouteCache:
    """Fixed-capacity LRU over computed routing results.

    Keyed by (start_node, end_node, algo, weight_strategy, k) - all the
    inputs that determine the engine's output *except* live road
    conditions, which are deliberately not in the key. Conditions are
    handled by invalidation instead: including them would mean every
    condition change silently orphaned the entire cache (a new key space),
    which is exactly the wasteful behavior precise invalidation exists to
    avoid.
    """

    def __init__(self, max_entries=CACHE_MAX_ENTRIES):
        self.max_entries = max_entries
        self._entries = OrderedDict()
        self.hits = 0
        self.misses = 0
        self.evictions_lru = 0
        self.evictions_invalidated = 0
        self.flushes = 0

    def __len__(self):
        return len(self._entries)

    def get(self, key):
        entry = self._entries.get(key)
        if entry is None:
            self.misses += 1
            return None
        self._entries.move_to_end(key)  # most-recently-used goes to the end
        self.hits += 1
        return entry

    def put(self, key, entry):
        if key in self._entries:
            self._entries.move_to_end(key)
        self._entries[key] = entry
        while len(self._entries) > self.max_entries:
            self._entries.popitem(last=False)  # evict least-recently-used
            self.evictions_lru += 1

    def invalidate_edge(self, u, v):
        """Evict exactly the entries whose stored route uses edge (u, v).

        This is the precise-invalidation core. It does NOT recompute
        anything - evicted keys simply become misses, and the next request
        for one recomputes it naturally with the new conditions applied.

        Correct only for *tightening* changes (closing a road, or raising a
        congestion multiplier). Making an edge more expensive can't
        invalidate a cached route that never used it: every alternative
        either uses the edge (now worse) or doesn't (unchanged), so a route
        that was optimal and avoided the edge is still optimal. The reverse
        direction - relaxing - has no such guarantee and must use flush().
        See NOTES.md.
        """
        doomed = [k for k, entry in self._entries.items() if entry.uses_edge(u, v)]
        for k in doomed:
            del self._entries[k]
        self.evictions_invalidated += len(doomed)
        return doomed

    def flush(self):
        """Drop everything. Used when conditions are relaxed (see above)."""
        count = len(self._entries)
        self._entries.clear()
        self.flushes += 1
        return count

    def stats(self):
        total = self.hits + self.misses
        return {
            "entries": len(self._entries),
            "max_entries": self.max_entries,
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": round(self.hits / total, 3) if total else None,
            "evictions_lru": self.evictions_lru,
            "evictions_invalidated": self.evictions_invalidated,
            "flushes": self.flushes,
            "approx_bytes": sum(e.approx_bytes() for e in self._entries.values()),
        }

    def keys(self):
        return list(self._entries.keys())


def extract_edge_paths(doc):
    """Pulls edge_path_osm out of a route JSON document, in either shape.

    Handles both output shapes the engine produces: the single-route shape
    (top-level `edge_path_osm`, used by k=1 and by every --algo=both file)
    and the multi-route shape (a `routes` array, each with its own
    `edge_path_osm`). Returns a list of paths, each a list of (u, v) tuples.
    """
    paths = []
    if isinstance(doc.get("routes"), list):
        for route in doc["routes"]:
            raw = route.get("edge_path_osm") or []
            paths.append([(int(a), int(b)) for a, b in raw])
    elif doc.get("edge_path_osm"):
        raw = doc["edge_path_osm"]
        paths.append([(int(a), int(b)) for a, b in raw])
    return paths
