#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "graph.hpp"

// A pluggable cost model: how expensive is it to traverse a given edge, and
// what's a lower-bound (never-overestimating) estimate of remaining cost
// from any node to a target?
//
// Bundling both together in one interface is deliberate. A*'s heuristic
// must be expressed in the *same units* as whatever it's being compared
// against - seconds if edge costs are travel time, meters if they're
// distance - and coupling them here makes it structurally impossible to
// accidentally pair a distance heuristic with a time-based cost model (or
// vice versa), which would silently make A* return non-optimal routes.
class WeightStrategy {
public:
    virtual ~WeightStrategy() = default;

    // Cost of traversing edge `edge_index` (an index into
    // g.col_index/g.weight/...), in this strategy's units.
    virtual double edge_cost(const Graph& g, int32_t edge_index) const = 0;

    // Admissible AND consistent lower bound on the remaining cost from node
    // `from` to node `to`, in the same units as edge_cost(). "Admissible"
    // (never overestimates the true remaining cost) is what makes A*
    // optimal; "consistent" (h(u) <= edge_cost(u,v) + h(v) for every edge)
    // is the stronger property that also justifies closing nodes for good
    // the first time they're popped, which is what our A* implementation
    // does. See NOTES.md for why both strategies below satisfy this.
    virtual double heuristic(const Graph& g, int32_t from, int32_t to) const = 0;

    virtual std::string name() const = 0;
};

// Edge cost = raw segment length in meters (checkpoint 1's behavior).
// Heuristic = straight-line (haversine) distance to the target, which can
// never exceed the true shortest-path distance - the standard admissible
// heuristic for distance-based routing.
class DistanceWeightStrategy : public WeightStrategy {
public:
    double edge_cost(const Graph& g, int32_t edge_index) const override;
    double heuristic(const Graph& g, int32_t from, int32_t to) const override;
    std::string name() const override { return "distance"; }
};

// Edge cost = estimated travel time in seconds: edge length divided by a
// speed derived from OSM's `maxspeed` tag if the edge had one, else a
// default speed for the edge's road class (see weight_strategy.cpp).
//
// Heuristic = straight-line distance to the target divided by
// `free_flow_speed_kmh` - the fastest speed anywhere in the graph. Dividing
// by the fastest possible speed keeps the heuristic a lower bound: no real
// path can cover ground faster than that, so the estimated time can never
// exceed the true remaining time.
class TimeWeightStrategy : public WeightStrategy {
public:
    explicit TimeWeightStrategy(double free_flow_speed_kmh);
    double edge_cost(const Graph& g, int32_t edge_index) const override;
    double heuristic(const Graph& g, int32_t from, int32_t to) const override;
    std::string name() const override { return "time"; }

private:
    double free_flow_speed_mps_;
};

// Decorator (checkpoint 5): wraps any other WeightStrategy and multiplies
// each edge's cost by a per-edge live multiplier - the "congested" half of
// live edge-weight updates. Wrapping rather than modifying
// Distance/TimeWeightStrategy means congestion composes with *either* cost
// model for free, and the underlying strategies stay exactly as
// checkpoints 2-4 left them.
//
// CRITICAL correctness constraint: every multiplier must be >= 1.0.
// heuristic() deliberately delegates to the base strategy *unchanged* -
// which stays admissible only because congestion can only ever make real
// edges more expensive, never cheaper, so a lower bound computed on
// uncongested costs is still a lower bound on congested ones. A multiplier
// below 1.0 would let the true remaining cost drop below the heuristic's
// estimate, breaking admissibility and silently making A* return
// non-optimal routes. main.cpp clamps to >= 1.0 for exactly this reason.
//
// name() also delegates, so route JSON keeps reporting "distance"/"time"
// (what the cost model *is*) rather than leaking that a decorator is in
// play - the frontend and checkpoint 1's route.json contract depend on
// that field's existing values.
class CongestionWeightStrategy : public WeightStrategy {
public:
    CongestionWeightStrategy(const WeightStrategy& base, const std::vector<double>& multipliers);
    double edge_cost(const Graph& g, int32_t edge_index) const override;
    double heuristic(const Graph& g, int32_t from, int32_t to) const override;
    std::string name() const override;

private:
    const WeightStrategy& base_;
    const std::vector<double>& multipliers_;  // size num_edges, 1.0 = uncongested
};

// Default free-flow speed (km/h) assigned to an edge with no OSM maxspeed
// tag, based on its road class.
double default_speed_kmh(RoadClass cls);

// Scans every edge in the graph and returns the fastest speed (km/h) any
// edge could be traversed at (tagged maxspeed if present, else the default
// for its road class). Used as TimeWeightStrategy's free-flow upper bound,
// so the heuristic is tied to what's actually in the data instead of a
// guessed constant.
double compute_max_speed_kmh(const Graph& g);
