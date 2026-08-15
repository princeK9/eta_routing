#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "graph.hpp"
#include "pathfinder.hpp"
#include "route_validator.hpp"
#include "snap.hpp"
#include "weight_strategy.hpp"
#include "yen_ksp.hpp"

namespace {

// How many extra candidates beyond the requested K to ask Yen's algorithm
// for, so the validation chain has something to fall back on when it
// rejects one. See run_multi() and NOTES.md.
constexpr int kValidationBuffer = 5;
constexpr int kMaxK = 10;  // sanity cap - this feature is scoped to "top 2-3", not "top 500"

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " <nodes.csv> <edges.csv> <start_lat> <start_lon> <end_lat> <end_lon> <output.json>"
                 " [--algo=dijkstra|astar|bidijkstra|both] [--weight=distance|time] [--trace=<path>]"
                 " [--k=<N>] [--closed=<u_osm>:<v_osm>,...]\n";
}

// One live road condition, as sent by the dev server on every call
// (checkpoint 5). `multiplier` is infinity for a fully closed road, or a
// finite value > 1 for a congested one.
struct LiveEdgeCondition {
    int64_t u_osm;
    int64_t v_osm;
    double multiplier;
};

// Parses the --closed= value: a comma-separated list of "u:v" (closed) or
// "u:v:multiplier" (congested, e.g. "u:v:2.5" = 2.5x cost).
//
// Checkpoint 4 defined this flag as "u:v" pairs only; checkpoint 5 extends
// the *grammar* of the existing flag rather than adding a second
// --congested= flag, so there's still exactly one way for the server to
// describe road conditions and old two-part values keep working unchanged.
std::vector<LiveEdgeCondition> parse_closed_edges(const std::string& spec) {
    std::vector<LiveEdgeCondition> conditions;
    if (spec.empty()) return conditions;
    std::stringstream ss(spec);
    std::string token;
    while (std::getline(ss, token, ',')) {
        size_t first = token.find(':');
        if (first == std::string::npos) continue;
        size_t second = token.find(':', first + 1);

        LiveEdgeCondition c;
        c.u_osm = std::stoll(token.substr(0, first));
        if (second == std::string::npos) {
            c.v_osm = std::stoll(token.substr(first + 1));
            c.multiplier = std::numeric_limits<double>::infinity();  // closed
        } else {
            c.v_osm = std::stoll(token.substr(first + 1, second - first - 1));
            c.multiplier = std::stod(token.substr(second + 1));
            // Clamp: a multiplier below 1.0 would make an edge *cheaper*
            // than the uncongested cost A*'s heuristic was computed
            // against, breaking admissibility (see weight_strategy.hpp).
            // Congestion only ever slows traffic down, so clamping is both
            // correct and semantically right.
            if (c.multiplier < 1.0) {
                std::cerr << "warning: multiplier " << c.multiplier << " for " << c.u_osm << ":" << c.v_osm
                          << " clamped to 1.0 (values < 1 would break A* admissibility)\n";
                c.multiplier = 1.0;
            }
        }
        conditions.push_back(c);
    }
    return conditions;
}

// Resolves live conditions (expressed as OSM node-id pairs, the stable
// cross-process identity) into the CSR-index-space vectors the search
// actually consumes: `banned_edges` for closed roads and `multipliers` for
// congested ones.
//
// Closed roads are *banned* rather than given a huge finite multiplier so
// a closure is genuinely impassable - a big-but-finite cost would still let
// a route drive down a closed road when it's the only option, which is
// wrong. Banning reuses checkpoint 4's SearchConstraints, already supported
// by all three Pathfinders.
//
// Note both directions of "the same road" are distinct directed edges here;
// the server sends whichever direction(s) it means. Parallel edges sharing
// one (u,v) OSM pair are all affected together, matching checkpoint 4's
// NoClosedRoadsRule semantics ("that road segment is shut").
struct ResolvedConditions {
    std::vector<bool> banned_edges;
    std::vector<double> multipliers;
    int closed_edges_matched = 0;
    int congested_edges_matched = 0;
    int unmatched_conditions = 0;
};

ResolvedConditions resolve_conditions(const Graph& g, const std::vector<LiveEdgeCondition>& conditions) {
    ResolvedConditions out;
    out.banned_edges.assign(static_cast<size_t>(g.num_edges()), false);
    out.multipliers.assign(static_cast<size_t>(g.num_edges()), 1.0);
    if (conditions.empty()) return out;

    std::unordered_map<int64_t, int32_t> osm_to_index;
    osm_to_index.reserve(static_cast<size_t>(g.num_nodes()) * 2);
    for (int32_t i = 0; i < g.num_nodes(); ++i) osm_to_index[g.osm_id[i]] = i;

    for (const auto& c : conditions) {
        auto uit = osm_to_index.find(c.u_osm);
        auto vit = osm_to_index.find(c.v_osm);
        if (uit == osm_to_index.end() || vit == osm_to_index.end()) {
            out.unmatched_conditions++;
            continue;
        }
        int32_t u = uit->second, v = vit->second;
        bool matched_any = false;
        for (int32_t e = g.row_ptr[u]; e < g.row_ptr[u + 1]; ++e) {
            if (g.col_index[e] != v) continue;
            matched_any = true;
            if (c.multiplier == std::numeric_limits<double>::infinity()) {
                out.banned_edges[static_cast<size_t>(e)] = true;
                out.closed_edges_matched++;
            } else {
                // max(): if two conditions touch the same edge, the worst
                // one wins rather than the last one parsed.
                out.multipliers[static_cast<size_t>(e)] =
                    std::max(out.multipliers[static_cast<size_t>(e)], c.multiplier);
                out.congested_edges_matched++;
            }
        }
        if (!matched_any) out.unmatched_conditions++;
    }
    return out;
}

std::unique_ptr<Pathfinder> make_pathfinder(const std::string& algo_name) {
    if (algo_name == "dijkstra") return std::make_unique<DijkstraPathfinder>();
    if (algo_name == "astar") return std::make_unique<AStarPathfinder>();
    if (algo_name == "bidijkstra") return std::make_unique<BidirectionalDijkstraPathfinder>();
    return nullptr;
}

// Inserts "_suffix" before a path's extension - "frontend/route.json" with
// suffix "dijkstra" becomes "frontend/route_dijkstra.json". Used by
// --algo=both to derive two output filenames (one per algorithm) from a
// single <output.json>/--trace argument, so a checkpoint 3 demo comparing
// both algorithms on the same source/target is still a single CLI
// invocation instead of two.
std::string with_suffix(const std::string& path, const std::string& suffix) {
    size_t dot = path.find_last_of('.');
    size_t slash = path.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        return path + "_" + suffix;
    }
    return path.substr(0, dot) + "_" + suffix + path.substr(dot);
}

// Emits the route's edges as [u_osm, v_osm] pairs (checkpoint 5).
//
// PathResult::edge_path holds CSR edge indices, which are an artifact of
// how Graph::load happened to bucket this run's edges - meaningless to any
// other process. The dev server's cache needs a *stable* edge identity to
// test "does this cached route use the edge that just got closed?", so the
// cross-process encoding is OSM node-id pairs, exactly the identity
// checkpoint 4's NoClosedRoadsRule and --closed= already use. Derived from
// consecutive path nodes rather than edge_path so it needs no edge->source
// lookup (the forward CSR doesn't store one).
void write_edge_path_osm(std::ofstream& out, const Graph& g, const PathResult& result, const char* indent) {
    out << indent << "\"edge_path_osm\": [";
    for (size_t i = 0; i + 1 < result.path.size(); ++i) {
        out << (i == 0 ? "" : ", ") << "[" << g.osm_id[result.path[i]] << ", " << g.osm_id[result.path[i + 1]]
            << "]";
    }
    out << "],\n";
}

void write_route_json(const std::string& output_path, const Graph& g, const Pathfinder& finder,
                       const WeightStrategy& weights, const PathResult& result, double elapsed_ms) {
    std::ofstream out(output_path);
    if (!out) throw std::runtime_error("Failed to open output file: " + output_path);
    out << std::fixed << std::setprecision(6);

    if (result.path.empty()) {
        out << "{\"path_found\": false, \"coordinates\": []}\n";
        return;
    }

    out << "{\n";
    out << "  \"path_found\": true,\n";
    out << "  \"algorithm\": \"" << finder.name() << "\",\n";
    out << "  \"weight_strategy\": \"" << weights.name() << "\",\n";
    out << "  \"cost\": " << result.cost << ",\n";
    // distance_m is only meaningful (and only emitted) when the active
    // strategy's cost *is* distance - kept for the checkpoint 1 frontend,
    // which reads this field directly and defaults to weight=distance.
    if (weights.name() == "distance") {
        out << "  \"distance_m\": " << result.cost << ",\n";
    }
    out << "  \"nodes_expanded\": " << result.nodes_expanded << ",\n";
    out << "  \"elapsed_ms\": " << elapsed_ms << ",\n";
    write_edge_path_osm(out, g, result, "  ");
    out << "  \"coordinates\": [\n";
    for (size_t i = 0; i < result.path.size(); ++i) {
        int32_t idx = result.path[i];
        out << "    [" << g.lat[idx] << ", " << g.lon[idx] << "]";
        out << (i + 1 < result.path.size() ? ",\n" : "\n");
    }
    out << "  ]\n}\n";
}

// Writes the checkpoint 3 visitation trace: one [lat, lon] per node, in the
// order DijkstraPathfinder/AStarPathfinder finalized it. The frontend
// replays this array with a short delay between entries to animate the
// search "wave" before drawing the final route.
void write_trace_json(const std::string& trace_path, const Graph& g, const Pathfinder& finder,
                       const WeightStrategy& weights, const std::vector<int32_t>& visit_order) {
    std::ofstream out(trace_path);
    if (!out) throw std::runtime_error("Failed to open trace file: " + trace_path);
    out << std::fixed << std::setprecision(6);

    out << "{\n";
    out << "  \"algorithm\": \"" << finder.name() << "\",\n";
    out << "  \"weight_strategy\": \"" << weights.name() << "\",\n";
    out << "  \"steps\": [\n";
    for (size_t i = 0; i < visit_order.size(); ++i) {
        int32_t idx = visit_order[i];
        out << "    [" << g.lat[idx] << ", " << g.lon[idx] << "]";
        out << (i + 1 < visit_order.size() ? ",\n" : "\n");
    }
    out << "  ]\n}\n";
}

// Runs one algorithm end to end (search, write route JSON, optionally write
// a trace JSON) and logs a one-line summary to stderr. Shared by the normal
// single-algorithm path and by --algo=both's loop over both algorithms, so
// that mode isn't a copy-pasted duplicate of the single-run logic.
void run_one(const Graph& g, const WeightStrategy& weights, const Pathfinder& finder, int32_t start, int32_t end,
             const std::string& output_path, const std::string& trace_path,
             const SearchConstraints* constraints) {
    std::vector<int32_t> visit_order;
    std::vector<int32_t>* trace_ptr = trace_path.empty() ? nullptr : &visit_order;

    auto t0 = std::chrono::steady_clock::now();
    PathResult result = finder.find_path(g, weights, start, end, trace_ptr, constraints);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cerr << finder.name() << " (" << weights.name() << ") done in " << ms << " ms, expanded "
              << result.nodes_expanded << " nodes\n";

    write_route_json(output_path, g, finder, weights, result, ms);
    if (result.path.empty()) {
        std::cerr << "No path found between the snapped nodes.\n";
        return;
    }
    std::cerr << "Route: " << result.path.size() << " nodes, cost " << result.cost << " (" << weights.name()
              << ") -> wrote " << output_path << "\n";

    if (trace_ptr) {
        write_trace_json(trace_path, g, finder, weights, visit_order);
        std::cerr << "Trace: " << visit_order.size() << " visited nodes -> wrote " << trace_path << "\n";
    }
}

// checkpoint 4: the --k>1 output shape is genuinely different from
// write_route_json's (a `routes` array instead of one top-level
// `coordinates`), not a variant of it - a single route and a set of
// alternatives aren't the same kind of answer, so this is a separate
// writer rather than a `routes.size()==1` special case bolted onto
// write_route_json.
void write_multi_route_json(const std::string& output_path, const Graph& g, const Pathfinder& finder,
                             const WeightStrategy& weights, const std::vector<PathResult>& routes, int k_requested,
                             int candidates_considered, int candidates_rejected) {
    std::ofstream out(output_path);
    if (!out) throw std::runtime_error("Failed to open output file: " + output_path);
    out << std::fixed << std::setprecision(6);

    out << "{\n";
    out << "  \"path_found\": " << (routes.empty() ? "false" : "true") << ",\n";
    out << "  \"algorithm\": \"" << finder.name() << "\",\n";
    out << "  \"weight_strategy\": \"" << weights.name() << "\",\n";
    out << "  \"k_requested\": " << k_requested << ",\n";
    out << "  \"k_returned\": " << routes.size() << ",\n";
    out << "  \"candidates_considered\": " << candidates_considered << ",\n";
    out << "  \"candidates_rejected\": " << candidates_rejected << ",\n";
    out << "  \"routes\": [\n";
    for (size_t r = 0; r < routes.size(); ++r) {
        const PathResult& res = routes[r];
        out << "    {\n";
        out << "      \"cost\": " << res.cost << ",\n";
        if (weights.name() == "distance") {
            out << "      \"distance_m\": " << res.cost << ",\n";
        }
        write_edge_path_osm(out, g, res, "      ");
        out << "      \"coordinates\": [\n";
        for (size_t i = 0; i < res.path.size(); ++i) {
            int32_t idx = res.path[i];
            out << "        [" << g.lat[idx] << ", " << g.lon[idx] << "]";
            out << (i + 1 < res.path.size() ? ",\n" : "\n");
        }
        out << "      ]\n";
        out << "    }" << (r + 1 < routes.size() ? ",\n" : "\n");
    }
    out << "  ]\n}\n";
}

// Composes Yen's algorithm with the validation chain: ask Yen's for
// `k + kValidationBuffer` candidates (cheap - it's the same Dijkstra/A*
// calls either way, just a few more spur searches), then walk that
// cost-ordered pool and keep the first `k` that pass every rule. A
// candidate the chain rejects is simply skipped - the next-cheapest
// candidate from the same pool takes its place, which is what "next-best
// candidate takes its place" means in this design (see NOTES.md for why
// this - generate-then-filter - was chosen over interleaving validation
// into Yen's own loop).
void run_multi(const Graph& g, const WeightStrategy& weights, const Pathfinder& finder, int32_t start, int32_t end,
               const std::string& output_path, int k, const std::vector<std::pair<int64_t, int64_t>>& closed_edges,
               const SearchConstraints* constraints) {
    int pool_size = k + kValidationBuffer;
    std::cerr << "Requesting up to " << k << " validated route(s) (Yen's pool size " << pool_size << ")...\n";

    auto t0 = std::chrono::steady_clock::now();
    std::vector<PathResult> pool = yen_k_shortest_paths(g, weights, finder, start, end, pool_size, constraints);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cerr << "Yen's algorithm found " << pool.size() << " candidate route(s) in " << ms << " ms\n";

    if (pool.empty()) {
        write_multi_route_json(output_path, g, finder, weights, {}, k, 0, 0);
        std::cerr << "No path found between the snapped nodes.\n";
        return;
    }

    RouteValidationChain chain;
    chain.add_rule(std::make_unique<ReachabilityRule>(start, end));
    chain.add_rule(std::make_unique<SanityBoundsRule>(pool.front().cost, /*max_multiple=*/3.0));
    chain.add_rule(std::make_unique<NoClosedRoadsRule>(closed_edges));

    std::vector<PathResult> accepted;
    int rejected = 0;
    for (const PathResult& candidate : pool) {
        if (static_cast<int>(accepted.size()) >= k) break;
        std::string reason;
        if (chain.validate(g, candidate, &reason)) {
            accepted.push_back(candidate);
        } else {
            rejected++;
            std::cerr << "Candidate (cost " << candidate.cost << ") rejected by " << reason << "\n";
        }
    }

    std::cerr << "Accepted " << accepted.size() << "/" << k << " requested route(s) from " << pool.size()
              << " candidate(s) considered (" << rejected << " rejected)\n";
    for (size_t i = 0; i < accepted.size(); ++i) {
        std::cerr << "Route " << (i + 1) << ": " << accepted[i].path.size() << " nodes, cost " << accepted[i].cost
                  << "\n";
    }

    write_multi_route_json(output_path, g, finder, weights, accepted, k, static_cast<int>(pool.size()), rejected);
    std::cerr << "Wrote " << output_path << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> positional;
    std::string algo_name = "astar";
    std::string weight_name = "distance";
    std::string trace_path;    // empty = tracing disabled
    std::string closed_spec;   // empty = no closed-road demo data
    int k = 1;                 // 1 = checkpoint 1-3 single-route behavior, unchanged

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--algo=", 0) == 0) {
            algo_name = arg.substr(7);
        } else if (arg.rfind("--weight=", 0) == 0) {
            weight_name = arg.substr(9);
        } else if (arg.rfind("--trace=", 0) == 0) {
            trace_path = arg.substr(8);
        } else if (arg.rfind("--k=", 0) == 0) {
            k = std::stoi(arg.substr(4));
        } else if (arg.rfind("--closed=", 0) == 0) {
            closed_spec = arg.substr(9);
        } else {
            positional.push_back(arg);
        }
    }

    if (positional.size() != 7) {
        print_usage(argv[0]);
        return 1;
    }
    if (k < 1 || k > kMaxK) {
        std::cerr << "--k must be between 1 and " << kMaxK << " (got " << k << ")\n";
        return 1;
    }
    if (k > 1 && algo_name == "both") {
        std::cerr << "--algo=both is not supported together with --k>1 (pick one algorithm for k-shortest-paths)\n";
        return 1;
    }

    const std::string nodes_csv = positional[0];
    const std::string edges_csv = positional[1];
    const double start_lat = std::stod(positional[2]);
    const double start_lon = std::stod(positional[3]);
    const double end_lat = std::stod(positional[4]);
    const double end_lon = std::stod(positional[5]);
    const std::string output_path = positional[6];

    std::cerr << "Loading graph...\n";
    Graph g = Graph::load(nodes_csv, edges_csv);
    std::cerr << "Loaded " << g.num_nodes() << " nodes, " << g.num_edges() << " edges\n";

    std::unique_ptr<WeightStrategy> weights;
    if (weight_name == "distance") {
        weights = std::make_unique<DistanceWeightStrategy>();
    } else if (weight_name == "time") {
        weights = std::make_unique<TimeWeightStrategy>(compute_max_speed_kmh(g));
    } else {
        std::cerr << "Unknown --weight value: " << weight_name << " (expected distance|time)\n";
        return 1;
    }

    int32_t start = snap_to_nearest_node(g, start_lat, start_lon);
    int32_t end = snap_to_nearest_node(g, end_lat, end_lon);
    std::cerr << "Snapped start -> node index " << start << " (osm_id " << g.osm_id[start] << ")\n";
    std::cerr << "Snapped end   -> node index " << end << " (osm_id " << g.osm_id[end] << ")\n";

    // --- checkpoint 5: apply live road conditions to this one run ---
    // Closed roads become banned edges (SearchConstraints, checkpoint 4);
    // congested roads become a CongestionWeightStrategy decorator over
    // whichever base cost model was selected. Both apply to *every*
    // algorithm and both k paths below, since they're folded into the
    // weights/constraints the searches already take.
    std::vector<LiveEdgeCondition> conditions = parse_closed_edges(closed_spec);
    ResolvedConditions resolved = resolve_conditions(g, conditions);
    if (!conditions.empty()) {
        std::cerr << "Live conditions: " << conditions.size() << " requested -> " << resolved.closed_edges_matched
                  << " closed edge(s), " << resolved.congested_edges_matched << " congested edge(s)";
        if (resolved.unmatched_conditions > 0) {
            std::cerr << ", " << resolved.unmatched_conditions << " unmatched (no such edge in graph)";
        }
        std::cerr << "\n";
    }

    const bool any_closed = resolved.closed_edges_matched > 0;
    const bool any_congested = resolved.congested_edges_matched > 0;

    SearchConstraints live_constraints;
    if (any_closed) live_constraints.banned_edges = &resolved.banned_edges;
    const SearchConstraints* constraints_ptr = any_closed ? &live_constraints : nullptr;

    std::unique_ptr<CongestionWeightStrategy> congestion;
    if (any_congested) congestion = std::make_unique<CongestionWeightStrategy>(*weights, resolved.multipliers);
    const WeightStrategy& effective_weights = any_congested ? static_cast<const WeightStrategy&>(*congestion)
                                                            : static_cast<const WeightStrategy&>(*weights);

    if (algo_name == "both") {
        // Run Dijkstra, A*, and bidirectional Dijkstra on the identical
        // source/target, each to its own suffixed output (and trace) file,
        // so a demo comparing all three search shapes is one command
        // instead of three. `--algo=both` now runs three algorithms, not
        // two - the flag value keeps its original name (rather than being
        // renamed to something like "all") because it's the exact string
        // dev_server.py's /compute endpoint already sends; extending what
        // it does here means that endpoint didn't need to change at all.
        for (const std::string& algo : {std::string("dijkstra"), std::string("astar"), std::string("bidijkstra")}) {
            auto finder = make_pathfinder(algo);
            run_one(g, effective_weights, *finder, start, end, with_suffix(output_path, algo),
                    trace_path.empty() ? "" : with_suffix(trace_path, algo), constraints_ptr);
        }
        return 0;
    }

    std::unique_ptr<Pathfinder> finder = make_pathfinder(algo_name);
    if (!finder) {
        std::cerr << "Unknown --algo value: " << algo_name << " (expected dijkstra|astar|bidijkstra|both)\n";
        return 1;
    }
    std::cerr << "Algorithm: " << finder->name() << " | Weight strategy: " << weights->name() << "\n";

    if (k > 1) {
        // checkpoint 4: K-shortest-paths + validation, a fully separate
        // output path from run_one() below - see run_multi()'s doc comment.
        if (!trace_path.empty()) {
            std::cerr << "warning: --trace is ignored when --k>1 (the checkpoint 3 visitation trace is defined for "
                          "a single search, not a k-shortest-paths run)\n";
        }
        // NoClosedRoadsRule still gets the closed list as a belt-and-braces
        // check, though it's now effectively redundant: closed edges are
        // banned during the search itself, so no candidate can contain one.
        // Kept rather than deleted - it's checkpoint 4's working code, it
        // costs one cheap scan per candidate, and it would still catch a
        // regression in the banning path.
        std::vector<std::pair<int64_t, int64_t>> closed_pairs;
        for (const auto& c : conditions) {
            if (c.multiplier == std::numeric_limits<double>::infinity()) {
                closed_pairs.push_back({c.u_osm, c.v_osm});
            }
        }
        run_multi(g, effective_weights, *finder, start, end, output_path, k, closed_pairs, constraints_ptr);
        return 0;
    }

    run_one(g, effective_weights, *finder, start, end, output_path, trace_path, constraints_ptr);
    return 0;
}
