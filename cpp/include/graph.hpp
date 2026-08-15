#pragma once
#include <cstdint>
#include <string>
#include <vector>

// OSM road classes we distinguish for speed purposes. Order/values aren't
// meaningful beyond identity - see weight_strategy.cpp for the default
// speed assigned to each.
enum class RoadClass : uint8_t {
    Motorway,
    Trunk,
    Primary,
    Secondary,
    Tertiary,
    Unclassified,
    Residential,
    LivingStreet,
    MotorwayLink,
    TrunkLink,
    PrimaryLink,
    SecondaryLink,
    TertiaryLink,
    Unknown,
};

// Maps an OSM `highway` tag value (e.g. "primary") to a RoadClass, or
// RoadClass::Unknown for anything unrecognized.
RoadClass parse_road_class(const std::string& highway_tag);

// A directed road network stored in Compressed Sparse Row (CSR) form.
//
// Why CSR instead of e.g. an adjacency list (vector<vector<Edge>>):
// Dijkstra's inner loop is "for every outgoing edge of node u". CSR packs
// all edges, sorted by source node, into flat arrays, so that loop becomes
// a contiguous scan between two known offsets (row_ptr[u] .. row_ptr[u+1])
// - no per-node heap allocation, no pointer chasing, good cache locality.
// The tradeoff is CSR is built once and not meant to be mutated (no cheap
// "add an edge" later), which is fine: the road network is a static export
// loaded once per run.
struct Graph {
    // Node arrays, indexed 0..num_nodes-1 (a dense id we assign on load).
    std::vector<int64_t> osm_id;  // original OSM node id, kept for reference/debug
    std::vector<double> lat;
    std::vector<double> lon;

    // CSR adjacency: outgoing edges of node u are
    //   col_index[row_ptr[u] .. row_ptr[u+1])
    // with matching per-edge data in weight[...]/road_class[...]/
    // tagged_maxspeed_kmh[...].
    std::vector<int32_t> row_ptr;    // size num_nodes + 1
    std::vector<int32_t> col_index;  // size num_edges

    // Per-edge attributes. `weight` is the one fixed fact about an edge
    // (its physical length); everything an edge might *cost* to traverse is
    // a matter of policy, computed on top of these by a WeightStrategy (see
    // weight_strategy.hpp) rather than baked in here.
    std::vector<double> weight;                // size num_edges, length in meters
    std::vector<RoadClass> road_class;         // size num_edges
    std::vector<float> tagged_maxspeed_kmh;    // size num_edges, 0 if OSM had no usable maxspeed tag

    // Reverse CSR: the *incoming* edges of node v are
    //   rev_col_index[rev_row_ptr[v] .. rev_row_ptr[v+1])
    // giving the source node of each such edge, with rev_edge_index[...]
    // giving that edge's index into the forward arrays above (weight/
    // road_class/tagged_maxspeed_kmh) - no per-edge data is duplicated,
    // this is purely a second index over the same edges. Exists so a
    // backward search (BidirectionalDijkstraPathfinder) can ask "which
    // nodes have an edge into v" in O(1)-per-edge time, the same way the
    // forward CSR answers "which edges leave u" - without it, that
    // question would need an O(E) scan of every edge per query.
    std::vector<int32_t> rev_row_ptr;    // size num_nodes + 1
    std::vector<int32_t> rev_col_index;  // size num_edges - source node of each incoming edge
    std::vector<int32_t> rev_edge_index;  // size num_edges - forward-array index of that same edge

    int num_nodes() const { return static_cast<int>(lat.size()); }
    int num_edges() const { return static_cast<int>(col_index.size()); }

    // Loads nodes_csv (node_id,lat,lon) and edges_csv
    // (u,v,length_m,highway,maxspeed_kmh), remaps OSM node ids to dense
    // 0..N-1 indices, and builds the CSR arrays above. Throws
    // std::runtime_error on malformed input. The highway/maxspeed columns
    // are optional for backward compatibility with checkpoint 1 data -
    // missing values default to RoadClass::Unknown / untagged.
    static Graph load(const std::string& nodes_csv, const std::string& edges_csv);
};
