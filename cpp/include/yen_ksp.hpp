#pragma once
#include <cstdint>
#include <vector>

#include "graph.hpp"
#include "pathfinder.hpp"
#include "weight_strategy.hpp"

// Yen's algorithm: computes up to `num_paths` distinct loopless shortest
// paths from source to target, in non-decreasing cost order.
//
// Deliberately knows nothing about route validation - it just enumerates
// candidates in cost order. The chain-of-responsibility validator
// (route_validator.hpp) is a separate, independent piece; main.cpp composes
// the two by requesting a slightly larger pool than it needs and filtering
// it. See NOTES.md for why that split (rather than validating inside this
// function) is the better design here.
//
// Returns fewer than `num_paths` entries if the graph doesn't have that
// many distinct loopless paths between source and target (a normal,
// non-error outcome - the loop simply runs out of candidates).
//
// `global_constraints` (checkpoint 5, optional) are bans that apply to
// *every* search this function runs - live closed roads, in practice.
// They have to be threaded in explicitly rather than just passed to the
// first find_path call: Yen's builds a fresh SearchConstraints per spur
// search (to ban already-used edges), and a fresh all-false ban vector
// would silently re-permit globally-closed edges on every spur. So each
// spur's ban vectors are *initialized from* the global set and then
// added to. Getting this wrong would produce k-shortest routes that
// happily drive down closed roads.
std::vector<PathResult> yen_k_shortest_paths(const Graph& g, const WeightStrategy& weights, const Pathfinder& finder,
                                              int32_t source, int32_t target, int num_paths,
                                              const SearchConstraints* global_constraints = nullptr);
