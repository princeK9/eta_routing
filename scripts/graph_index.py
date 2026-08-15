"""
Read-only, in-memory index over the exported graph CSVs, for the dev
server's use only (checkpoint 5).

Two jobs, both of which the server needs to do *without* invoking
route_engine.exe:

1. snap(lat, lon) -> OSM node id. The LRU route cache is keyed by
   (start_node, end_node, ...), so the server has to resolve a click to a
   node id *before* deciding hit-or-miss - if it had to run the engine to
   learn the snapped node, a "hit" would already have paid the full
   subprocess cost and the cache would be pointless.

2. nearest_edge(lat, lon) -> the road segment nearest a click, so the UI
   can let a user mark a road closed/congested by clicking near it.

This deliberately duplicates the C++ snap_to_nearest_node() logic in
Python. That duplication is a real cost (two implementations of one rule
that must agree), and it's accepted here because the mandated architecture
- cache in the Python layer, keyed by node - *requires* the Python layer to
know the snapped node independently. The mitigation is verification rather
than trust: scripts/verify_snap_matches_engine.py checks Python's snap
against the engine's own reported snap over many random points. See
NOTES.md.

Uses only the Python standard library, consistent with the rest of this
project.
"""
import csv
import heapq
import math
from pathlib import Path

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters.

    Deliberately the same formula and same Earth radius as the C++
    haversine_meters() in cpp/src/snap.cpp - snap() below must agree with
    the engine's snap, and using a cheaper approximation here (e.g. flat
    equirectangular) would risk disagreeing on near-ties.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _point_segment_distance_m(plat, plon, alat, alon, blat, blon):
    """Distance in meters from point P to segment AB.

    Projects to a local flat plane first (meters east/north relative to A),
    which is accurate at city scale and lets this be plain vector math.
    Unlike snap(), this doesn't have to match any C++ behavior - nothing in
    the engine computes point-to-segment distance - so the cheaper planar
    approximation is fine here.
    """
    lat_scale = 111320.0
    lon_scale = 111320.0 * math.cos(math.radians(alat))

    px = (plon - alon) * lon_scale
    py = (plat - alat) * lat_scale
    bx = (blon - alon) * lon_scale
    by = (blat - alat) * lat_scale

    seg_len_sq = bx * bx + by * by
    if seg_len_sq <= 0.0:
        return math.hypot(px, py)  # degenerate segment (A == B)

    t = (px * bx + py * by) / seg_len_sq
    t = max(0.0, min(1.0, t))  # clamp to the segment, not the infinite line
    return math.hypot(px - t * bx, py - t * by)


class GraphIndex:
    def __init__(self, nodes_csv: Path, edges_csv: Path):
        self.node_ids = []   # parallel arrays, index-aligned
        self.node_lats = []
        self.node_lons = []
        self.coord_by_id = {}

        with open(nodes_csv, newline="") as f:
            for row in csv.DictReader(f):
                nid = int(row["node_id"])
                lat = float(row["lat"])
                lon = float(row["lon"])
                self.node_ids.append(nid)
                self.node_lats.append(lat)
                self.node_lons.append(lon)
                self.coord_by_id[nid] = (lat, lon)

        # Directed edges as (u_osm, v_osm), plus an incidence index so
        # nearest_edge() can look at just the edges touching a few nearby
        # nodes instead of scanning all ~98k every click.
        self.edges = []
        self.edges_by_node = {}
        self.edge_pairs = set()  # for O(1) "does the reverse direction exist?"
        with open(edges_csv, newline="") as f:
            for row in csv.DictReader(f):
                u = int(row["u"])
                v = int(row["v"])
                idx = len(self.edges)
                self.edges.append((u, v))
                self.edge_pairs.add((u, v))
                self.edges_by_node.setdefault(u, []).append(idx)
                self.edges_by_node.setdefault(v, []).append(idx)

    def has_edge(self, u, v):
        return (u, v) in self.edge_pairs

    def node_count(self):
        return len(self.node_ids)

    def edge_count(self):
        return len(self.edges)

    def snap(self, lat, lon):
        """Nearest graph node to (lat, lon), as an OSM node id.

        Brute-force linear scan, mirroring the C++ implementation exactly
        (same formula, same tie-breaking by first-lowest-index-wins, since
        both use strict < when comparing).
        """
        best_id = None
        best_dist = float("inf")
        ids = self.node_ids
        lats = self.node_lats
        lons = self.node_lons
        for i in range(len(ids)):
            d = haversine_m(lat, lon, lats[i], lons[i])
            if d < best_dist:
                best_dist = d
                best_id = ids[i]
        return best_id, best_dist

    def nearest_edge(self, lat, lon, candidate_nodes=40):
        """Road segment nearest to (lat, lon).

        Two-stage so a click doesn't cost a full 98k-edge scan: take the
        `candidate_nodes` nearest nodes, then measure true point-to-segment
        distance only for edges touching one of them. A segment can only be
        near the click if at least one of its endpoints is reasonably near
        too (our edges are intersection-to-intersection, with shape points
        collapsed - see checkpoint 1), so this finds the right road for any
        realistic click on a visible street.

        Returns None if nothing is in range, else a dict describing the edge.
        """
        scored = heapq.nsmallest(
            candidate_nodes,
            range(len(self.node_ids)),
            key=lambda i: haversine_m(lat, lon, self.node_lats[i], self.node_lons[i]),
        )

        seen = set()
        best = None
        best_dist = float("inf")
        for i in scored:
            nid = self.node_ids[i]
            for eidx in self.edges_by_node.get(nid, ()):
                if eidx in seen:
                    continue
                seen.add(eidx)
                u, v = self.edges[eidx]
                ulat, ulon = self.coord_by_id[u]
                vlat, vlon = self.coord_by_id[v]
                d = _point_segment_distance_m(lat, lon, ulat, ulon, vlat, vlon)
                if d < best_dist:
                    best_dist = d
                    best = (u, v, ulat, ulon, vlat, vlon)

        if best is None:
            return None
        u, v, ulat, ulon, vlat, vlon = best
        return {
            "u": u,
            "v": v,
            "u_coord": [ulat, ulon],
            "v_coord": [vlat, vlon],
            "distance_m": best_dist,
        }
