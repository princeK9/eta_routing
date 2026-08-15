"""
Export the drivable OSM road network for Bhubaneswar, India into two plain
CSV files the C++ engine can load directly:

    data/nodes.csv  -> node_id,lat,lon
        real intersections / dead ends
    data/edges.csv  -> u,v,length_m,highway,maxspeed_kmh
        directed road segments. `highway` is the OSM road class (residential,
        primary, ...) and `maxspeed_kmh` is the tagged speed limit if OSM had
        one for that way, else empty - both feed the C++ side's time-based
        edge-weight strategy (checkpoint 2).

Uses only the Python standard library (urllib, json, math, csv) - see
NOTES.md for why this doesn't use osmnx/pandas.

Run once, manually, whenever you want to (re)pull the road network:

    python export_osm.py
"""
import http.client
import json
import math
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import defaultdict
from pathlib import Path

NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"
# The main instance (overpass-api.de) is a shared public server that
# regularly returns 504 Gateway Timeout under load - observed firsthand
# while building this script. Kumi Systems' mirror runs the same Overpass
# API software against the same OSM data, so falling back to it on failure
# is safe and keeps this "run once" script from being at the mercy of one
# server's load.
OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
)
PLACE_QUERY = "Bhubaneswar, India"
OUT_DIR = Path(__file__).resolve().parent.parent / "data"
USER_AGENT = "smart-route-eta-engine/checkpoint1 (learning project)"

# Road types a car can actually drive on - excludes footways, cycleways,
# rail, etc.
DRIVABLE_HIGHWAY_TYPES = (
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "unclassified", "residential", "living_street",
    "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
)


def geocode_bbox(place: str):
    """Look up a place name via Nominatim and return (south, west, north, east).

    We geocode instead of hardcoding Bhubaneswar's coordinates so the script
    stays reusable for other cities later - and it turned out to be more
    reliable than Overpass's own area-by-name lookup (that returned zero
    results for this admin boundary; a plain lat/lon bounding box query
    against Overpass has no such ambiguity).
    """
    params = urllib.parse.urlencode({"q": place, "format": "json", "limit": 1})
    req = urllib.request.Request(f"{NOMINATIM_URL}?{params}", headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=30) as resp:
        results = json.loads(resp.read().decode("utf-8"))
    if not results:
        raise RuntimeError(f"Nominatim found nothing for '{place}'")
    south, north, west, east = (float(x) for x in results[0]["boundingbox"])
    return south, west, north, east


def build_query(bbox) -> str:
    south, west, north, east = bbox
    highway_pattern = "|".join(DRIVABLE_HIGHWAY_TYPES)
    return f"""
[out:json][timeout:180];
(
  way["highway"~"^({highway_pattern})$"]({south},{west},{north},{east});
);
out body;
>;
out skel qt;
"""


def fetch_overpass(query: str) -> dict:
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    # Overpass rejects requests with Python's default User-Agent (HTTP 406);
    # it just wants something identifying, per its usage policy.
    headers = {"User-Agent": USER_AGENT}

    last_error = None
    for url in OVERPASS_URLS:
        for attempt in range(1, 2):
            print(f"Querying {url} (attempt {attempt}, can take ~30-90s for a city this size)...")
            try:
                req = urllib.request.Request(url, data=data, headers=headers)
                with urllib.request.urlopen(req, timeout=200) as resp:
                    return json.loads(resp.read().decode("utf-8"))
            except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, http.client.IncompleteRead,
                    ConnectionError, OSError) as exc:
                last_error = exc
                print(f"  failed: {exc!r}")
                time.sleep(5)
    raise RuntimeError(f"All Overpass endpoints failed; last error: {last_error!r}")


def parse_maxspeed_kmh(tag_value):
    """Parse an OSM maxspeed tag ("50", "50 mph", "national", ...) into km/h.

    Returns None if the tag is missing or not a plain numeric value - OSM
    also allows values like "national"/"none"/"walk" that would need a
    country-specific lookup table to resolve, which is out of scope here.
    The C++ side falls back to a per-road-class default speed in that case.
    """
    if not tag_value:
        return None
    parts = tag_value.strip().split()
    try:
        value = float(parts[0])
    except (ValueError, IndexError):
        return None
    if len(parts) > 1 and parts[1].lower() == "mph":
        value *= 1.60934
    return value


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def main():
    bbox = geocode_bbox(PLACE_QUERY)
    print(f"Geocoded '{PLACE_QUERY}' -> bbox (south,west,north,east) = {bbox}")
    osm = fetch_overpass(build_query(bbox))

    node_coords = {}  # osm node id -> (lat, lon)
    ways = []  # list of {"nodes": [ids...], "oneway": bool}

    for el in osm["elements"]:
        if el["type"] == "node":
            node_coords[el["id"]] = (el["lat"], el["lon"])
        elif el["type"] == "way":
            tags = el.get("tags", {})
            oneway = tags.get("oneway") in ("yes", "true", "1")
            ways.append({
                "nodes": el["nodes"],
                "oneway": oneway,
                "highway": tags.get("highway", ""),
                "maxspeed_kmh": parse_maxspeed_kmh(tags.get("maxspeed")),
            })

    print(f"Fetched {len(node_coords)} raw nodes, {len(ways)} ways")

    # A raw OSM node is a "real" graph node (intersection or dead end) if
    # more than one way touches it, or if it's the first/last node of a
    # way (a dead end has nowhere else to route to, so it must be routable
    # too). Every other node is just a shape point describing the curve of
    # a road between two intersections - not something a router needs to
    # reason about - so it gets collapsed away below.
    node_way_count = defaultdict(int)
    for way in ways:
        for nid in set(way["nodes"]):
            node_way_count[nid] += 1

    real_node_ids = {nid for nid, count in node_way_count.items() if count > 1}
    for way in ways:
        if way["nodes"]:
            real_node_ids.add(way["nodes"][0])
            real_node_ids.add(way["nodes"][-1])
    real_node_ids &= node_coords.keys()

    # Collapse each way into edges between consecutive real nodes, summing
    # the haversine length of every shape point in between. highway/maxspeed
    # are uniform along a single OSM way, so every edge collapsed out of the
    # same way just inherits that way's tags.
    edges = []  # (u, v, length_m, highway, maxspeed_kmh)
    for way in ways:
        nodes = way["nodes"]
        if not nodes:
            continue
        highway = way["highway"]
        maxspeed = way["maxspeed_kmh"]
        seg_start = nodes[0]
        seg_len = 0.0
        for i in range(1, len(nodes)):
            a, b = nodes[i - 1], nodes[i]
            if a not in node_coords or b not in node_coords:
                continue
            seg_len += haversine_m(*node_coords[a], *node_coords[b])
            if b in real_node_ids:
                edges.append((seg_start, b, seg_len, highway, maxspeed))
                if not way["oneway"]:
                    edges.append((b, seg_start, seg_len, highway, maxspeed))
                seg_start = b
                seg_len = 0.0

    print(f"Collapsed to {len(real_node_ids)} intersection nodes, {len(edges)} directed edges")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(OUT_DIR / "nodes.csv", "w", newline="") as f:
        f.write("node_id,lat,lon\n")
        for nid in sorted(real_node_ids):
            lat, lon = node_coords[nid]
            f.write(f"{nid},{lat},{lon}\n")

    with open(OUT_DIR / "edges.csv", "w", newline="") as f:
        f.write("u,v,length_m,highway,maxspeed_kmh\n")
        for u, v, length, highway, maxspeed in edges:
            maxspeed_str = f"{maxspeed:.1f}" if maxspeed is not None else ""
            f.write(f"{u},{v},{length:.3f},{highway},{maxspeed_str}\n")

    print(f"Wrote {OUT_DIR / 'nodes.csv'} and {OUT_DIR / 'edges.csv'}")


if __name__ == "__main__":
    main()
