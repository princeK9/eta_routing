"""
Isolated tests for the checkpoint 5 LRU cache + conditions store.

No framework (consistent with this project's zero-new-dependency stance)
and no assert() - plain checks that report and set an exit code, so they
can't silently pass. (The C++ side learned that lesson the hard way in
checkpoint 4: assert() compiles away under NDEBUG. Different language, same
principle - a test that can't fail loudly isn't a test.)

Run from the project root:
    python scripts\\test_route_cache.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_cache import CLOSED, CacheEntry, LiveConditions, LruRouteCache, extract_edge_paths  # noqa: E402

FAILURES = []


def check(condition, label):
    if condition:
        print(f"  ok: {label}")
    else:
        print(f"  FAIL: {label}")
        FAILURES.append(label)


def entry_using(edges):
    """A CacheEntry whose single route uses exactly `edges`."""
    return CacheEntry(files={"route.json": "{}"}, edge_paths=[list(edges)])


def test_lru_ordering():
    print("LRU ordering / capacity:")
    cache = LruRouteCache(max_entries=3)
    for i in range(3):
        cache.put(f"k{i}", entry_using([(i, i + 1)]))
    check(len(cache) == 3, "holds up to capacity")

    cache.get("k0")  # k0 becomes most-recently-used
    cache.put("k3", entry_using([(9, 9)]))  # should evict k1 (now the LRU), not k0
    check(len(cache) == 3, "stays at capacity after overflow")
    check(cache.get("k0") is not None, "recently-used entry survived eviction")
    check(cache.get("k1") is None, "least-recently-used entry was evicted")
    check(cache.evictions_lru == 1, "counted exactly one LRU eviction")


def test_precise_invalidation():
    print("Precise invalidation (edge_path membership):")
    cache = LruRouteCache(max_entries=10)
    # Three routes; only A and C traverse the edge (100, 200).
    cache.put("A", entry_using([(1, 2), (100, 200), (2, 3)]))
    cache.put("B", entry_using([(1, 2), (7, 8)]))
    cache.put("C", entry_using([(100, 200)]))
    cache.put("D", entry_using([(200, 100)]))  # the *reverse* edge - a different directed edge

    doomed = cache.invalidate_edge(100, 200)
    check(sorted(doomed) == ["A", "C"], "evicted exactly the entries whose edge_path contains the edge")
    check(cache.get("B") is not None, "kept an entry that does not use the edge")
    check(cache.get("D") is not None, "kept the reverse-direction entry (distinct directed edge)")
    check(cache.evictions_invalidated == 2, "counted exactly two invalidation evictions")

    # The point of precision: a flush would have destroyed B and D too.
    check(len(cache) == 2, "cache retained the still-valid entries rather than flushing")


def test_invalidate_nothing_when_unused():
    print("Invalidation of an edge no cached route uses:")
    cache = LruRouteCache(max_entries=10)
    cache.put("A", entry_using([(1, 2)]))
    cache.put("B", entry_using([(3, 4)]))
    doomed = cache.invalidate_edge(55, 66)
    check(doomed == [], "evicted nothing")
    check(len(cache) == 2, "left the cache untouched")


def test_flush():
    print("Flush (used for relaxing changes):")
    cache = LruRouteCache(max_entries=10)
    cache.put("A", entry_using([(1, 2)]))
    cache.put("B", entry_using([(3, 4)]))
    dropped = cache.flush()
    check(dropped == ["A", "B"], "reported which entries it dropped (now keys, not just a count)")
    check(len(cache) == 0, "cache is empty after flush")


def test_multi_route_entry():
    print("Multi-route entry (k-shortest-paths):")
    # An entry holding 3 alternative routes; only the 2nd uses (50, 60).
    entry = CacheEntry(files={"route_k.json": "{}"}, edge_paths=[
        [(1, 2), (2, 3)],
        [(1, 2), (50, 60)],
        [(9, 10)],
    ])
    check(entry.uses_edge(50, 60), "detects an edge used by any one of the stored routes")
    check(not entry.uses_edge(77, 88), "does not false-positive on an unused edge")

    cache = LruRouteCache(max_entries=10)
    cache.put("K", entry)
    check(cache.invalidate_edge(50, 60) == ["K"], "evicts the whole entry if any of its routes uses the edge")


def test_conditions_tighten_vs_relax():
    print("LiveConditions tighten/relax classification:")
    lc = LiveConditions()
    check(lc.mark(1, 2, 2.0) == "tighten", "new congestion is a tighten")
    check(lc.mark(1, 2, 3.0) == "tighten", "raising a multiplier is a tighten")
    check(lc.mark(1, 2, 1.5) == "relax", "lowering a multiplier is a relax")
    check(lc.mark(1, 2, 1.5) == "noop", "re-applying the same multiplier is a noop")
    check(lc.mark(1, 2, CLOSED) == "tighten", "closing an already-congested edge is a tighten")


def test_conditions_cli_arg():
    print("LiveConditions -> --closed= CLI argument:")
    lc = LiveConditions()
    check(lc.to_cli_arg() == "", "empty when nothing is marked")
    lc.mark(10, 20, CLOSED)
    check(lc.to_cli_arg() == "10:20", "closed edge renders as u:v")
    lc2 = LiveConditions()
    lc2.mark(30, 40, 2.5)
    check(lc2.to_cli_arg() == "30:40:2.5", "congested edge renders as u:v:multiplier")


def test_extract_edge_paths():
    print("extract_edge_paths handles both engine output shapes:")
    single = {"path_found": True, "edge_path_osm": [[1, 2], [2, 3]]}
    check(extract_edge_paths(single) == [[(1, 2), (2, 3)]], "single-route shape (k=1 / --algo=both)")

    multi = {"routes": [
        {"edge_path_osm": [[1, 2]]},
        {"edge_path_osm": [[3, 4], [4, 5]]},
    ]}
    check(extract_edge_paths(multi) == [[(1, 2)], [(3, 4), (4, 5)]], "multi-route shape (k>1)")

    check(extract_edge_paths({"path_found": False}) == [], "no-path document yields no paths")


def main():
    for fn in [
        test_lru_ordering,
        test_precise_invalidation,
        test_invalidate_nothing_when_unused,
        test_flush,
        test_multi_route_entry,
        test_conditions_tighten_vs_relax,
        test_conditions_cli_arg,
        test_extract_edge_paths,
    ]:
        fn()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("All cache/conditions checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
