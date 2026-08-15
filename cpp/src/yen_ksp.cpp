#include "yen_ksp.hpp"

#include <algorithm>

namespace {

bool same_path(const std::vector<int32_t>& a, const std::vector<int32_t>& b) { return a == b; }

bool pool_contains(const std::vector<PathResult>& a, const std::vector<PathResult>& b,
                    const std::vector<int32_t>& path) {
    for (const auto& p : a) {
        if (same_path(p.path, path)) return true;
    }
    for (const auto& p : b) {
        if (same_path(p.path, path)) return true;
    }
    return false;
}

}  // namespace

std::vector<PathResult> yen_k_shortest_paths(const Graph& g, const WeightStrategy& weights, const Pathfinder& finder,
                                              int32_t source, int32_t target, int num_paths,
                                              const SearchConstraints* global_constraints) {
    std::vector<PathResult> A;  // accepted shortest paths so far, in non-decreasing cost order

    // Baselines every per-spur ban vector starts from, so globally-closed
    // edges/nodes stay banned across all of Yen's internal searches.
    const std::vector<bool> base_banned_edges =
        (global_constraints && global_constraints->banned_edges)
            ? *global_constraints->banned_edges
            : std::vector<bool>(static_cast<size_t>(g.num_edges()), false);
    const std::vector<bool> base_banned_nodes =
        (global_constraints && global_constraints->banned_nodes)
            ? *global_constraints->banned_nodes
            : std::vector<bool>(static_cast<size_t>(g.num_nodes()), false);

    PathResult first = finder.find_path(g, weights, source, target, nullptr, global_constraints);
    if (first.path.empty()) return A;  // no path at all - nothing more to try
    A.push_back(std::move(first));

    std::vector<PathResult> B;  // candidate pool, unsorted; picked from by linear min-scan (small by construction)

    while (static_cast<int>(A.size()) < num_paths) {
        const PathResult& prev = A.back();  // spur from the most recently accepted path, per the algorithm

        for (size_t i = 0; i + 1 < prev.path.size(); ++i) {
            int32_t spur_node = prev.path[i];

            // Root path: prev.path[0..i] inclusive. Its cost is the sum of
            // prev's first i edges (prev.edge_path[0..i-1]).
            std::vector<int32_t> root_path(prev.path.begin(), prev.path.begin() + static_cast<long>(i) + 1);
            double root_cost = 0.0;
            for (size_t j = 0; j < i; ++j) root_cost += weights.edge_cost(g, prev.edge_path[j]);

            // Ban the edge leaving spur_node in every already-accepted path
            // that shares this exact root path - otherwise the spur search
            // would just rediscover a path already in A. Starts from the
            // global bans (live closed roads) rather than all-false, so a
            // closed road can't sneak back in on a spur search.
            std::vector<bool> banned_edges = base_banned_edges;
            for (const auto& p : A) {
                if (p.path.size() <= i) continue;
                if (!std::equal(root_path.begin(), root_path.end(), p.path.begin())) continue;
                if (i < p.edge_path.size()) banned_edges[static_cast<size_t>(p.edge_path[i])] = true;
            }

            // Ban every root-path node except spur_node itself, so the
            // resulting combined path can't loop back through its own root.
            std::vector<bool> banned_nodes = base_banned_nodes;
            for (size_t j = 0; j + 1 < root_path.size(); ++j) {
                banned_nodes[static_cast<size_t>(root_path[j])] = true;
            }

            SearchConstraints constraints;
            constraints.banned_edges = &banned_edges;
            constraints.banned_nodes = &banned_nodes;

            PathResult spur = finder.find_path(g, weights, spur_node, target, nullptr, &constraints);
            if (spur.path.empty()) continue;  // no detour survives these bans from this spur node

            std::vector<int32_t> total_path(root_path.begin(), root_path.end() - 1);
            total_path.insert(total_path.end(), spur.path.begin(), spur.path.end());

            if (pool_contains(A, B, total_path)) continue;  // already have this exact route

            std::vector<int32_t> total_edges(prev.edge_path.begin(),
                                              prev.edge_path.begin() + static_cast<long>(i));
            total_edges.insert(total_edges.end(), spur.edge_path.begin(), spur.edge_path.end());

            PathResult candidate;
            candidate.path = std::move(total_path);
            candidate.edge_path = std::move(total_edges);
            candidate.cost = root_cost + spur.cost;
            B.push_back(std::move(candidate));
        }

        if (B.empty()) break;  // exhausted every candidate - no more distinct loopless paths exist

        auto min_it = std::min_element(
            B.begin(), B.end(), [](const PathResult& a, const PathResult& b) { return a.cost < b.cost; });
        A.push_back(std::move(*min_it));
        B.erase(min_it);
    }

    return A;
}
