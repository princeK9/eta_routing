#include "pathfinder.hpp"

#include <algorithm>
#include <limits>
#include <queue>

namespace {

void reconstruct(const std::vector<int32_t>& parent, const std::vector<int32_t>& parent_edge, int32_t target,
                  std::vector<int32_t>* out_path, std::vector<int32_t>* out_edges) {
    for (int32_t cur = target; cur != -1; cur = parent[cur]) {
        out_path->push_back(cur);
        if (parent[cur] != -1) out_edges->push_back(parent_edge[cur]);
    }
    std::reverse(out_path->begin(), out_path->end());
    std::reverse(out_edges->begin(), out_edges->end());
}

inline bool is_banned_edge(const SearchConstraints* constraints, int32_t edge) {
    return constraints && constraints->banned_edges && (*constraints->banned_edges)[edge];
}

inline bool is_banned_node(const SearchConstraints* constraints, int32_t node) {
    return constraints && constraints->banned_nodes && (*constraints->banned_nodes)[node];
}

// Splices together a path found by BidirectionalDijkstraPathfinder from
// its two halves. `parent_f`/`parent_edge_f` are the ordinary forward
// search's parent arrays (source-rooted) - reconstruct() above handles
// those the same way Dijkstra/A* do. The backward half needs its own
// logic: `parent_b[v]` is the node the *backward* search reached `v` from
// while walking the reverse graph from `target`, so walking
// meeting_node -> parent_b[meeting_node] -> parent_b[parent_b[...]] -> ...
// -> target, *without* reversing, already yields the correct forward-graph
// order meeting_node..target (each step in that walk corresponds to a real
// forward edge, tracked in parent_edge_b). See NOTES.md for why.
void reconstruct_bidirectional(const std::vector<int32_t>& parent_f, const std::vector<int32_t>& parent_edge_f,
                                const std::vector<int32_t>& parent_b, const std::vector<int32_t>& parent_edge_b,
                                int32_t meeting_node, std::vector<int32_t>* out_path,
                                std::vector<int32_t>* out_edges) {
    reconstruct(parent_f, parent_edge_f, meeting_node, out_path, out_edges);  // source .. meeting_node

    for (int32_t cur = meeting_node; parent_b[cur] != -1; cur = parent_b[cur]) {
        out_edges->push_back(parent_edge_b[cur]);
        out_path->push_back(parent_b[cur]);
    }
}

}  // namespace

// Both DijkstraPathfinder and AStarPathfinder below share the same shape:
// a min-priority-queue of (priority, node) pairs, a `closed` set marking
// nodes whose shortest cost is finalized, and lazy deletion (stale queue
// entries for an already-closed node are just skipped when popped, rather
// than removed from the queue up front - std::priority_queue has no
// decrease-key operation, so this is the standard workaround).
//
// The *only* structural difference is what gets pushed as the priority:
// Dijkstra uses the accumulated cost so far (g-score); A* uses g-score plus
// a heuristic estimate of the remaining cost, which is what lets it head
// toward the target instead of expanding uniformly outward. Both stop as
// soon as the target is popped from the open set, which is valid for
// Dijkstra given non-negative edge weights, and valid for A* given a
// consistent heuristic (see weight_strategy.hpp) - in both cases, once a
// node is popped with the minimum priority in the frontier, its cost is
// already final.
//
// checkpoint 4 addition: `constraints`, if non-null, lets a caller (Yen's
// algorithm) ban specific edges/nodes for this one search without touching
// the Graph - see SearchConstraints in pathfinder.hpp for why.

PathResult DijkstraPathfinder::find_path(const Graph& g, const WeightStrategy& weights, int32_t source,
                                          int32_t target, std::vector<int32_t>* visit_order,
                                          const SearchConstraints* constraints) const {
    const int n = g.num_nodes();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int32_t> parent(n, -1);
    std::vector<int32_t> parent_edge(n, -1);
    std::vector<bool> closed(n, false);
    dist[source] = 0.0;

    using QueueEntry = std::pair<double, int32_t>;  // (dist, node)
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    open.push({0.0, source});

    int64_t expanded = 0;
    while (!open.empty()) {
        int32_t u = open.top().second;
        open.pop();
        if (closed[u]) continue;
        if (u == target) break;
        closed[u] = true;
        expanded++;
        if (visit_order) visit_order->push_back(u);

        for (int32_t e = g.row_ptr[u]; e < g.row_ptr[u + 1]; ++e) {
            if (is_banned_edge(constraints, e)) continue;
            int32_t v = g.col_index[e];
            if (closed[v] || is_banned_node(constraints, v)) continue;
            double nd = dist[u] + weights.edge_cost(g, e);
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                parent_edge[v] = e;
                open.push({nd, v});
            }
        }
    }

    PathResult result;
    if (dist[target] == std::numeric_limits<double>::infinity()) return result;  // empty path = not found
    result.cost = dist[target];
    result.nodes_expanded = expanded;
    reconstruct(parent, parent_edge, target, &result.path, &result.edge_path);
    return result;
}

PathResult AStarPathfinder::find_path(const Graph& g, const WeightStrategy& weights, int32_t source,
                                       int32_t target, std::vector<int32_t>* visit_order,
                                       const SearchConstraints* constraints) const {
    const int n = g.num_nodes();
    std::vector<double> g_score(n, std::numeric_limits<double>::infinity());
    std::vector<int32_t> parent(n, -1);
    std::vector<int32_t> parent_edge(n, -1);
    std::vector<bool> closed(n, false);
    g_score[source] = 0.0;

    using QueueEntry = std::pair<double, int32_t>;  // (f_score = g + h, node)
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    open.push({weights.heuristic(g, source, target), source});

    int64_t expanded = 0;
    while (!open.empty()) {
        int32_t u = open.top().second;
        open.pop();
        if (closed[u]) continue;
        if (u == target) break;
        closed[u] = true;
        expanded++;
        if (visit_order) visit_order->push_back(u);

        for (int32_t e = g.row_ptr[u]; e < g.row_ptr[u + 1]; ++e) {
            if (is_banned_edge(constraints, e)) continue;
            int32_t v = g.col_index[e];
            if (closed[v] || is_banned_node(constraints, v)) continue;
            double tentative = g_score[u] + weights.edge_cost(g, e);
            if (tentative < g_score[v]) {
                g_score[v] = tentative;
                parent[v] = u;
                parent_edge[v] = e;
                open.push({tentative + weights.heuristic(g, v, target), v});
            }
        }
    }

    PathResult result;
    if (g_score[target] == std::numeric_limits<double>::infinity()) return result;  // empty path = not found
    result.cost = g_score[target];
    result.nodes_expanded = expanded;
    reconstruct(parent, parent_edge, target, &result.path, &result.edge_path);
    return result;
}

// Runs a forward Dijkstra from `source` and a backward Dijkstra from
// `target` (the latter over Graph::rev_row_ptr/rev_col_index - the reverse
// adjacency built in Graph::load) at the same time, always expanding next
// from whichever frontier's smallest tentative distance is currently
// lower. Each side tracks its own dist/parent/parent_edge/closed arrays -
// the two searches don't interact except through `best_meeting_cost`.
//
// Termination: standard bidirectional-Dijkstra stopping rule. Once the sum
// of both frontiers' smallest tentative distances is >= the best meeting
// cost found so far, no future expansion on either side can possibly
// improve on it (every unexplored candidate on either side already costs
// at least that much to reach), so the search can stop even though neither
// side has necessarily reached the other's start point.
//
// Meeting check: whenever either side finalizes (closes) a node u, if the
// *other* side has any tentative distance to u at all (finite, not
// necessarily closed yet), that's a candidate complete path through u -
// checked against the best found so far.
PathResult BidirectionalDijkstraPathfinder::find_path(const Graph& g, const WeightStrategy& weights, int32_t source,
                                                       int32_t target, std::vector<int32_t>* visit_order,
                                                       const SearchConstraints* constraints) const {
    const int n = g.num_nodes();
    const double kInf = std::numeric_limits<double>::infinity();

    std::vector<double> dist_f(n, kInf), dist_b(n, kInf);
    std::vector<int32_t> parent_f(n, -1), parent_b(n, -1);
    std::vector<int32_t> parent_edge_f(n, -1), parent_edge_b(n, -1);
    std::vector<bool> closed_f(n, false), closed_b(n, false);
    dist_f[source] = 0.0;
    dist_b[target] = 0.0;

    using QueueEntry = std::pair<double, int32_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_f, open_b;
    open_f.push({0.0, source});
    open_b.push({0.0, target});

    double best_meeting_cost = kInf;
    int32_t meeting_node = -1;
    int64_t expanded = 0;

    while (!open_f.empty() && !open_b.empty()) {
        while (!open_f.empty() && closed_f[open_f.top().second]) open_f.pop();
        while (!open_b.empty() && closed_b[open_b.top().second]) open_b.pop();
        if (open_f.empty() || open_b.empty()) break;

        double top_f = open_f.top().first;
        double top_b = open_b.top().first;
        if (top_f + top_b >= best_meeting_cost) break;  // provably can't improve further - see doc comment

        if (top_f <= top_b) {
            int32_t u = open_f.top().second;
            open_f.pop();
            closed_f[u] = true;
            expanded++;
            if (visit_order) visit_order->push_back(u);
            if (dist_b[u] < kInf && dist_f[u] + dist_b[u] < best_meeting_cost) {
                best_meeting_cost = dist_f[u] + dist_b[u];
                meeting_node = u;
            }
            for (int32_t e = g.row_ptr[u]; e < g.row_ptr[u + 1]; ++e) {
                if (is_banned_edge(constraints, e)) continue;
                int32_t v = g.col_index[e];
                if (closed_f[v] || is_banned_node(constraints, v)) continue;
                double nd = dist_f[u] + weights.edge_cost(g, e);
                if (nd < dist_f[v]) {
                    dist_f[v] = nd;
                    parent_f[v] = u;
                    parent_edge_f[v] = e;
                    open_f.push({nd, v});
                }
            }
        } else {
            int32_t u = open_b.top().second;
            open_b.pop();
            closed_b[u] = true;
            expanded++;
            if (visit_order) visit_order->push_back(u);
            if (dist_f[u] < kInf && dist_f[u] + dist_b[u] < best_meeting_cost) {
                best_meeting_cost = dist_f[u] + dist_b[u];
                meeting_node = u;
            }
            for (int32_t e = g.rev_row_ptr[u]; e < g.rev_row_ptr[u + 1]; ++e) {
                int32_t orig_edge = g.rev_edge_index[e];  // this edge, in the forward graph, is v -> u
                if (is_banned_edge(constraints, orig_edge)) continue;
                int32_t v = g.rev_col_index[e];
                if (closed_b[v] || is_banned_node(constraints, v)) continue;
                double nd = dist_b[u] + weights.edge_cost(g, orig_edge);
                if (nd < dist_b[v]) {
                    dist_b[v] = nd;
                    parent_b[v] = u;
                    parent_edge_b[v] = orig_edge;
                    open_b.push({nd, v});
                }
            }
        }
    }

    PathResult result;
    if (meeting_node == -1) return result;  // empty path = not found
    result.cost = best_meeting_cost;
    result.nodes_expanded = expanded;
    reconstruct_bidirectional(parent_f, parent_edge_f, parent_b, parent_edge_b, meeting_node, &result.path,
                               &result.edge_path);
    return result;
}
