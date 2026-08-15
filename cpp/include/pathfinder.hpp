#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "graph.hpp"
#include "weight_strategy.hpp"

struct PathResult {
    std::vector<int32_t> path;       // node indices, source..target in order; empty if unreachable
    std::vector<int32_t> edge_path;  // CSR edge index used between path[i] and path[i+1]; size == path.size()-1
    double cost = 0.0;               // total cost in the active WeightStrategy's units
    int64_t nodes_expanded = 0;      // nodes popped off the open set and relaxed - the benchmark's headline number
};

// Restricts a search to a subset of the graph without mutating the Graph
// itself: any edge/node flagged true here is treated as absent. Both
// vectors, if provided, must be sized to g.num_edges()/g.num_nodes().
// Left null (the default) on every call site except checkpoint 4's Yen's
// algorithm, so this costs nothing anywhere else - same pattern as
// `visit_order` above (checkpoint 3): an optional pointer, checked once
// per relaxation, that lets a caller change search behavior without a new
// Pathfinder implementation. This is precisely what "reuse the existing
// Pathfinder interface instead of duplicating search logic" means in
// practice - see NOTES.md and yen_ksp.hpp.
struct SearchConstraints {
    const std::vector<bool>* banned_nodes = nullptr;  // banned_nodes[v] == true -> v may not be visited
    const std::vector<bool>* banned_edges = nullptr;  // banned_edges[e] == true -> edge e may not be taken
};

// Common interface for single-source-single-target shortest path search over
// a CSR Graph under a given WeightStrategy.
//
// Dijkstra and A* both implement this so callers (the CLI, the benchmark)
// can pick an algorithm without knowing anything about its internals - a
// Strategy pattern, chosen specifically so A* is a second implementation of
// one shared interface rather than a special case bolted onto Dijkstra or a
// wholesale replacement of it.
class Pathfinder {
public:
    virtual ~Pathfinder() = default;

    // If `visit_order` is non-null, every node's index is appended to it at
    // the moment that node is finalized (popped and expanded), in order -
    // the raw material for the checkpoint 3 frontend's search animation.
    // Left null (the default), tracing costs nothing beyond one pointer
    // check per expansion, which is why the benchmark never passes it: it
    // must not slow down the numbers it's measuring.
    virtual PathResult find_path(const Graph& g, const WeightStrategy& weights, int32_t source, int32_t target,
                                  std::vector<int32_t>* visit_order = nullptr,
                                  const SearchConstraints* constraints = nullptr) const = 0;
    virtual std::string name() const = 0;
};

// Textbook single-source Dijkstra, stopped as soon as the target is
// finalized (see pathfinder.cpp for why that's valid) rather than computing
// distances to every node in the graph.
class DijkstraPathfinder : public Pathfinder {
public:
    PathResult find_path(const Graph& g, const WeightStrategy& weights, int32_t source, int32_t target,
                          std::vector<int32_t>* visit_order = nullptr,
                          const SearchConstraints* constraints = nullptr) const override;
    std::string name() const override { return "dijkstra"; }
};

// A*: identical search skeleton to DijkstraPathfinder, with one structural
// difference - the priority queue orders nodes by g-score + heuristic
// instead of g-score alone, so the search is biased toward the target
// instead of expanding uniformly in all directions.
class AStarPathfinder : public Pathfinder {
public:
    PathResult find_path(const Graph& g, const WeightStrategy& weights, int32_t source, int32_t target,
                          std::vector<int32_t>* visit_order = nullptr,
                          const SearchConstraints* constraints = nullptr) const override;
    std::string name() const override { return "astar"; }
};

// Bidirectional Dijkstra: runs two ordinary Dijkstra searches at once - one
// forward from `source` over the graph's normal (outgoing) adjacency, one
// backward from `target` over its reverse adjacency (Graph::rev_row_ptr et
// al.) - always expanding next from whichever of the two frontiers has the
// smaller tentative distance, until neither frontier can possibly improve
// on the best meeting point found so far. See NOTES.md for why this
// typically expands far fewer nodes than a one-directional search on the
// same query (the geometric intuition: two circles meeting need a much
// smaller combined radius than one circle reaching all the way across).
class BidirectionalDijkstraPathfinder : public Pathfinder {
public:
    PathResult find_path(const Graph& g, const WeightStrategy& weights, int32_t source, int32_t target,
                          std::vector<int32_t>* visit_order = nullptr,
                          const SearchConstraints* constraints = nullptr) const override;
    std::string name() const override { return "bidijkstra"; }
};
