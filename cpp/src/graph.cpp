#include "graph.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

}  // namespace

RoadClass parse_road_class(const std::string& highway_tag) {
    static const std::unordered_map<std::string, RoadClass> kByTag = {
        {"motorway", RoadClass::Motorway},
        {"trunk", RoadClass::Trunk},
        {"primary", RoadClass::Primary},
        {"secondary", RoadClass::Secondary},
        {"tertiary", RoadClass::Tertiary},
        {"unclassified", RoadClass::Unclassified},
        {"residential", RoadClass::Residential},
        {"living_street", RoadClass::LivingStreet},
        {"motorway_link", RoadClass::MotorwayLink},
        {"trunk_link", RoadClass::TrunkLink},
        {"primary_link", RoadClass::PrimaryLink},
        {"secondary_link", RoadClass::SecondaryLink},
        {"tertiary_link", RoadClass::TertiaryLink},
    };
    auto it = kByTag.find(highway_tag);
    return it == kByTag.end() ? RoadClass::Unknown : it->second;
}

Graph Graph::load(const std::string& nodes_csv, const std::string& edges_csv) {
    Graph g;

    // --- Pass 1: read nodes, assign dense 0..N-1 indices in file order ---
    std::ifstream nf(nodes_csv);
    if (!nf) throw std::runtime_error("Cannot open nodes csv: " + nodes_csv);

    std::unordered_map<int64_t, int32_t> id_to_index;
    std::string line;
    std::getline(nf, line);  // header: node_id,lat,lon
    while (std::getline(nf, line)) {
        if (line.empty()) continue;
        auto fields = split_csv_line(line);
        if (fields.size() < 3) throw std::runtime_error("Malformed node row: " + line);
        int64_t id = std::stoll(fields[0]);
        double lat = std::stod(fields[1]);
        double lon = std::stod(fields[2]);
        int32_t idx = static_cast<int32_t>(g.osm_id.size());
        id_to_index[id] = idx;
        g.osm_id.push_back(id);
        g.lat.push_back(lat);
        g.lon.push_back(lon);
    }

    const int n = g.num_nodes();

    // --- Pass 2: read edges into a temporary flat list, resolving OSM ids
    //     to dense indices as we go ---
    std::ifstream ef(edges_csv);
    if (!ef) throw std::runtime_error("Cannot open edges csv: " + edges_csv);

    struct RawEdge {
        int32_t src, dst;
        double w;
        RoadClass road_class;
        float maxspeed_kmh;
    };
    std::vector<RawEdge> raw_edges;

    std::getline(ef, line);  // header: u,v,length_m[,highway,maxspeed_kmh]
    while (std::getline(ef, line)) {
        if (line.empty()) continue;
        auto fields = split_csv_line(line);
        if (fields.size() < 3) throw std::runtime_error("Malformed edge row: " + line);
        int64_t u = std::stoll(fields[0]);
        int64_t v = std::stoll(fields[1]);
        double w = std::stod(fields[2]);

        // highway/maxspeed are optional columns, for backward compatibility
        // with checkpoint 1's 3-column edges.csv.
        RoadClass road_class = fields.size() > 3 ? parse_road_class(fields[3]) : RoadClass::Unknown;
        float maxspeed_kmh = (fields.size() > 4 && !fields[4].empty()) ? std::stof(fields[4]) : 0.0f;

        auto uit = id_to_index.find(u);
        auto vit = id_to_index.find(v);
        if (uit == id_to_index.end() || vit == id_to_index.end()) {
            continue;  // defensive: skip edges referencing an unknown node
        }
        raw_edges.push_back({uit->second, vit->second, w, road_class, maxspeed_kmh});
    }

    // --- Build CSR via a counting-sort style bucket pass: O(N + E), no
    //     need to sort the edge list. ---
    std::vector<int32_t> degree(n, 0);
    for (const auto& e : raw_edges) degree[e.src]++;

    g.row_ptr.assign(n + 1, 0);
    for (int i = 0; i < n; ++i) g.row_ptr[i + 1] = g.row_ptr[i] + degree[i];

    g.col_index.assign(raw_edges.size(), 0);
    g.weight.assign(raw_edges.size(), 0.0);
    g.road_class.assign(raw_edges.size(), RoadClass::Unknown);
    g.tagged_maxspeed_kmh.assign(raw_edges.size(), 0.0f);

    // The forward CSR doesn't store each edge's source node explicitly
    // (it's implicit in which node's row_ptr bucket the edge landed in),
    // but the reverse CSR below needs it, so it's captured here while
    // we're already walking raw_edges with e.src in hand.
    std::vector<int32_t> edge_src(raw_edges.size(), 0);

    // cursor[i] tracks the next free slot in node i's bucket; walking through
    // raw_edges once and dropping each edge into its bucket via the cursor
    // reproduces the same grouping a sort would give, in linear time.
    std::vector<int32_t> cursor(g.row_ptr.begin(), g.row_ptr.end() - 1);
    for (const auto& e : raw_edges) {
        int32_t pos = cursor[e.src]++;
        g.col_index[pos] = e.dst;
        g.weight[pos] = e.w;
        g.road_class[pos] = e.road_class;
        g.tagged_maxspeed_kmh[pos] = e.maxspeed_kmh;
        edge_src[pos] = e.src;
    }

    // --- Build the reverse CSR with the identical bucket-fill technique,
    //     just keyed by destination (col_index) instead of source. ---
    std::vector<int32_t> in_degree(n, 0);
    for (int32_t v : g.col_index) in_degree[v]++;

    g.rev_row_ptr.assign(n + 1, 0);
    for (int i = 0; i < n; ++i) g.rev_row_ptr[i + 1] = g.rev_row_ptr[i] + in_degree[i];

    g.rev_col_index.assign(g.col_index.size(), 0);
    g.rev_edge_index.assign(g.col_index.size(), 0);

    std::vector<int32_t> rev_cursor(g.rev_row_ptr.begin(), g.rev_row_ptr.end() - 1);
    for (int32_t e = 0; e < g.num_edges(); ++e) {
        int32_t dst = g.col_index[e];
        int32_t pos = rev_cursor[dst]++;
        g.rev_col_index[pos] = edge_src[e];
        g.rev_edge_index[pos] = e;
    }

    return g;
}
