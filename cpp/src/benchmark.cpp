// Benchmarks DijkstraPathfinder, AStarPathfinder, and
// BidirectionalDijkstraPathfinder against each other on the same set of
// random source/target pairs, under each WeightStrategy, and writes a plain
// text comparison report. This is the actual evidence for "A* and
// bidirectional Dijkstra both expand far fewer nodes than plain Dijkstra,
// for different reasons" - every number in the report comes from running
// all three algorithms on the real Bhubaneswar graph, not a simulation.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

#include "graph.hpp"
#include "pathfinder.hpp"
#include "weight_strategy.hpp"

namespace {

struct RunStats {
    bool found = false;
    double cost = 0.0;
    int64_t nodes_expanded = 0;
    double elapsed_us = 0.0;
};

// A single Dijkstra/A* query on this graph finishes in low single-digit
// milliseconds, but this toolchain's std::chrono::steady_clock turned out
// (empirically, comparing early single-shot timings against each other) to
// only have ~1ms resolution - too coarse to time one query directly without
// the reading being dominated by quantization noise rather than actual
// work. Standard fix: run the same query `kReps` times back to back and
// divide the total elapsed time by the count, which pushes the measured
// interval well above the clock's tick size. nodes_expanded is deterministic
// for a given (finder, weights, s, t), so it only needs to be read once.
constexpr int kReps = 25;

RunStats time_run(const Pathfinder& finder, const Graph& g, const WeightStrategy& w, int32_t s, int32_t t) {
    PathResult r = finder.find_path(g, w, s, t);
    RunStats stats;
    stats.found = !r.path.empty();
    stats.cost = r.cost;
    stats.nodes_expanded = r.nodes_expanded;

    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kReps; ++i) {
        PathResult rep = finder.find_path(g, w, s, t);
        (void)rep;
    }
    auto t1 = std::chrono::steady_clock::now();
    stats.elapsed_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / kReps;
    return stats;
}

struct Row {
    int32_t source, target;
    RunStats dij, ast, bidij;
};

struct Pair {
    int32_t source, target;
};

// Samples `num_pairs` random (source, target) node pairs that are actually
// reachable from each other (the Bhubaneswar bbox clips a handful of tiny
// disconnected components at its edges, so not every random pair is
// reachable). Reachability is a property of the graph's topology, not of
// which WeightStrategy is active (every strategy here uses strictly
// positive edge costs), so one fixed set of pairs can be reused for every
// weight strategy's benchmark section - which is what lets the report
// compare distance-weighted vs time-weighted results on literally the same
// routes. Fixed RNG seed -> reproducible pairs across runs, while still
// being genuinely randomly sampled rather than hand-picked.
std::vector<Pair> sample_reachable_pairs(const Graph& g, const Pathfinder& reachability_check,
                                          const WeightStrategy& w, std::mt19937& rng, int num_pairs) {
    std::uniform_int_distribution<int32_t> pick(0, g.num_nodes() - 1);
    std::vector<Pair> pairs;
    int attempts = 0;
    while (static_cast<int>(pairs.size()) < num_pairs && attempts < num_pairs * 50) {
        attempts++;
        int32_t s = pick(rng);
        int32_t t = pick(rng);
        if (s == t) continue;
        PathResult r = reachability_check.find_path(g, w, s, t);
        if (r.path.empty()) continue;  // unreachable pair, resample
        pairs.push_back({s, t});
    }
    return pairs;
}

std::vector<Row> run_pairs(const Graph& g, const WeightStrategy& w, const Pathfinder& dijkstra,
                            const Pathfinder& astar, const Pathfinder& bidijkstra, const std::vector<Pair>& pairs) {
    std::vector<Row> rows;
    rows.reserve(pairs.size());
    for (const auto& p : pairs) {
        Row row;
        row.source = p.source;
        row.target = p.target;
        row.dij = time_run(dijkstra, g, w, p.source, p.target);
        row.ast = time_run(astar, g, w, p.source, p.target);
        row.bidij = time_run(bidijkstra, g, w, p.source, p.target);
        rows.push_back(row);
    }
    return rows;
}

// Column layout: cost/expanded/time for all three algorithms, but no
// per-pair pairwise cut%/speedup columns (with three algorithms that's three
// pairwise comparisons - dij-vs-ast, dij-vs-bidij, ast-vs-bidij - which
// doesn't fit readably as extra per-row columns). Those ratios are computed
// once, from the aggregate sums, in the "Averages" block instead - which is
// also where checkpoint 2's version put the numbers actually worth citing.
void write_section(std::ofstream& out, const std::string& title, const std::string& units,
                    const std::vector<Row>& rows) {
    constexpr int kWidth = 4 + 9 + 9 + 13 + 13 + 13 + 7 + 12 + 12 + 13 + 10 + 10 + 11;

    out << "=== Weight strategy: " << title << " (cost in " << units << ") ===\n\n";
    out << std::left << std::setw(4) << "#" << std::right << std::setw(9) << "source" << std::setw(9) << "target"
        << std::setw(13) << "dij_cost" << std::setw(13) << "ast_cost" << std::setw(13) << "bidij_cost"
        << std::setw(7) << "match" << std::setw(12) << "dij_exp" << std::setw(12) << "ast_exp" << std::setw(13)
        << "bidij_exp" << std::setw(10) << "dij_us" << std::setw(10) << "ast_us" << std::setw(11) << "bidij_us"
        << "\n";
    out << std::string(kWidth, '-') << "\n";

    double sum_dij_exp = 0, sum_ast_exp = 0, sum_bidij_exp = 0;
    double sum_dij_us = 0, sum_ast_us = 0, sum_bidij_us = 0;
    int mismatches = 0;

    for (size_t i = 0; i < rows.size(); ++i) {
        const Row& r = rows[i];
        double tol = std::max(1e-3, 1e-6 * r.dij.cost);
        bool match = std::fabs(r.dij.cost - r.ast.cost) < tol && std::fabs(r.dij.cost - r.bidij.cost) < tol;
        if (!match) mismatches++;

        out << std::left << std::setw(4) << (i + 1) << std::right << std::setw(9) << r.source << std::setw(9)
            << r.target << std::setw(13) << std::fixed << std::setprecision(2) << r.dij.cost << std::setw(13)
            << r.ast.cost << std::setw(13) << r.bidij.cost << std::setw(7) << (match ? "yes" : "NO") << std::setw(12)
            << r.dij.nodes_expanded << std::setw(12) << r.ast.nodes_expanded << std::setw(13)
            << r.bidij.nodes_expanded << std::setw(10) << std::setprecision(1) << r.dij.elapsed_us << std::setw(10)
            << r.ast.elapsed_us << std::setw(11) << r.bidij.elapsed_us << "\n";

        sum_dij_exp += r.dij.nodes_expanded;
        sum_ast_exp += r.ast.nodes_expanded;
        sum_bidij_exp += r.bidij.nodes_expanded;
        sum_dij_us += r.dij.elapsed_us;
        sum_ast_us += r.ast.elapsed_us;
        sum_bidij_us += r.bidij.elapsed_us;
    }

    size_t n = rows.size();
    out << std::string(kWidth, '-') << "\n";
    out << "Averages over " << n << " runs:\n";
    out << "  nodes expanded:  dijkstra=" << std::fixed << std::setprecision(0) << (sum_dij_exp / n)
        << "   astar=" << (sum_ast_exp / n) << " (" << std::setprecision(1)
        << (100.0 * (1.0 - sum_ast_exp / sum_dij_exp)) << "% fewer than dijkstra)"
        << "   bidijkstra=" << std::setprecision(0) << (sum_bidij_exp / n) << " (" << std::setprecision(1)
        << (100.0 * (1.0 - sum_bidij_exp / sum_dij_exp)) << "% fewer than dijkstra)\n";
    out << "  time:            dijkstra=" << std::setprecision(1) << (sum_dij_us / n)
        << " us   astar=" << (sum_ast_us / n) << " us (" << std::setprecision(2) << (sum_dij_us / sum_ast_us)
        << "x)   bidijkstra=" << std::setprecision(1) << (sum_bidij_us / n) << " us ("
        << std::setprecision(2) << (sum_dij_us / sum_bidij_us) << "x)\n";
    out << "  cost agreement:  " << (n - mismatches) << "/" << n
        << " runs where dijkstra/astar/bidijkstra all found the same-cost optimal path\n\n";
}

}  // namespace

int main(int argc, char** argv) {
    const std::string nodes_csv = argc > 1 ? argv[1] : "data/nodes.csv";
    const std::string edges_csv = argc > 2 ? argv[2] : "data/edges.csv";
    const int num_pairs = argc > 3 ? std::stoi(argv[3]) : 15;
    const std::string out_path = argc > 4 ? argv[4] : "benchmark_results.txt";

    std::cerr << "Loading graph...\n";
    Graph g = Graph::load(nodes_csv, edges_csv);
    std::cerr << "Loaded " << g.num_nodes() << " nodes, " << g.num_edges() << " edges\n";

    std::mt19937 rng(42);  // fixed seed: reproducible pair selection across runs
    DijkstraPathfinder dijkstra;
    AStarPathfinder astar;
    BidirectionalDijkstraPathfinder bidijkstra;

    DistanceWeightStrategy distance_w;
    double max_speed = compute_max_speed_kmh(g);
    TimeWeightStrategy time_w(max_speed);
    std::cerr << "Free-flow speed bound for time heuristic: " << max_speed << " km/h (max over all edges)\n";

    std::cerr << "Sampling " << num_pairs << " reachable random pairs...\n";
    std::vector<Pair> pairs = sample_reachable_pairs(g, dijkstra, distance_w, rng, num_pairs);
    std::cerr << "Got " << pairs.size() << " pairs; running all three algorithms x both weight strategies on them...\n";

    std::vector<Row> distance_rows = run_pairs(g, distance_w, dijkstra, astar, bidijkstra, pairs);
    std::vector<Row> time_rows = run_pairs(g, time_w, dijkstra, astar, bidijkstra, pairs);

    std::time_t now = std::time(nullptr);
    char time_buf[64];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    std::ofstream out(out_path);
    out << "Smart Route & ETA Engine -- Dijkstra vs A* vs Bidirectional Dijkstra Benchmark\n";
    out << "Graph: Bhubaneswar, India -- " << g.num_nodes() << " nodes, " << g.num_edges() << " directed edges\n";
    out << "Random seed: 42 (reproducible pair selection)\n";
    out << "Generated: " << time_buf << "\n\n";

    write_section(out, "distance", "meters", distance_rows);
    write_section(out, "time", "seconds", time_rows);

    out << "Notes:\n";
    out << "- The percentages in \"Averages\" are how many fewer nodes astar/bidijkstra expanded vs\n";
    out << "  dijkstra to solve the same query.\n";
    out << "- All three algorithms stop as soon as the search is provably done (target popped for\n";
    out << "  dijkstra/astar; frontiers can't improve the best meeting point for bidijkstra) rather\n";
    out << "  than running to completion over the whole graph, so nodes_expanded reflects real work\n";
    out << "  done per query, not a fixed cost proportional to graph size.\n";
    out << "- The \"(Nx)\" figures next to astar/bidijkstra's average time are wall-clock speedup vs\n";
    out << "  dijkstra (dijkstra_us / that algorithm's us), from the same process/run.\n";
    out << "- *_us columns are each the mean of " << kReps
        << " repeated runs of that exact query, not a single\n";
    out << "  timing - this toolchain's steady_clock has ~1ms resolution, too coarse to time one\n";
    out << "  sub-millisecond query directly, so timings are averaged to rise above that noise floor.\n";
    out << "- nodes_expanded is exact (not timing-dependent) and is the more reliable metric here:\n";
    out << "  wall-clock speedup can be inconsistent or even negative despite large node-count cuts,\n";
    out << "  because astar pays a real per-node cost (a haversine heuristic call per relaxed edge)\n";
    out << "  that dijkstra doesn't, and bidijkstra pays for running two priority queues and a\n";
    out << "  meeting check instead of one - the win only shows up on the clock once the nodes saved\n";
    out << "  outweigh that per-node overhead. See NOTES.md for the full discussion.\n";

    out.close();
    std::cerr << "Wrote " << out_path << "\n";
    return 0;
}
