#pragma once
#include <cstdint>

#include "graph.hpp"

// Finds the graph node nearest to a given (lat, lon), by great-circle
// distance.
//
// Implementation is a brute-force linear scan over all nodes: O(N) per
// query. At Bhubaneswar's scale (tens of thousands of nodes) that's low
// single-digit milliseconds, so it's the right choice for a first
// checkpoint - a spatial index (k-d tree / R-tree) is the obvious next
// optimization if profiling ever shows snap-to-node actually matters.
int32_t snap_to_nearest_node(const Graph& g, double query_lat, double query_lon);

// Great-circle (haversine) distance between two lat/lon points, in meters.
double haversine_meters(double lat1, double lon1, double lat2, double lon2);
