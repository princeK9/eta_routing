#include "weight_strategy.hpp"

#include <algorithm>

#include "snap.hpp"  // haversine_meters

double default_speed_kmh(RoadClass cls) {
    switch (cls) {
        case RoadClass::Motorway:
            return 90.0;
        case RoadClass::Trunk:
            return 70.0;
        case RoadClass::Primary:
            return 60.0;
        case RoadClass::Secondary:
            return 50.0;
        case RoadClass::Tertiary:
            return 40.0;
        case RoadClass::Unclassified:
            return 30.0;
        case RoadClass::Residential:
            return 30.0;
        case RoadClass::LivingStreet:
            return 15.0;
        case RoadClass::MotorwayLink:
            return 50.0;
        case RoadClass::TrunkLink:
            return 40.0;
        case RoadClass::PrimaryLink:
            return 40.0;
        case RoadClass::SecondaryLink:
            return 30.0;
        case RoadClass::TertiaryLink:
            return 30.0;
        case RoadClass::Unknown:
        default:
            return 30.0;  // conservative default for untagged/unrecognized roads
    }
}

double compute_max_speed_kmh(const Graph& g) {
    double max_speed = 0.0;
    for (int32_t e = 0; e < g.num_edges(); ++e) {
        double speed = g.tagged_maxspeed_kmh[e] > 0.0f ? g.tagged_maxspeed_kmh[e] : default_speed_kmh(g.road_class[e]);
        max_speed = std::max(max_speed, speed);
    }
    return max_speed > 0.0 ? max_speed : 50.0;  // fallback for a pathological empty graph
}

double DistanceWeightStrategy::edge_cost(const Graph& g, int32_t edge_index) const { return g.weight[edge_index]; }

double DistanceWeightStrategy::heuristic(const Graph& g, int32_t from, int32_t to) const {
    return haversine_meters(g.lat[from], g.lon[from], g.lat[to], g.lon[to]);
}

TimeWeightStrategy::TimeWeightStrategy(double free_flow_speed_kmh) : free_flow_speed_mps_(free_flow_speed_kmh / 3.6) {}

double TimeWeightStrategy::edge_cost(const Graph& g, int32_t edge_index) const {
    double speed_kmh = g.tagged_maxspeed_kmh[edge_index] > 0.0f ? g.tagged_maxspeed_kmh[edge_index]
                                                                 : default_speed_kmh(g.road_class[edge_index]);
    double speed_mps = speed_kmh / 3.6;
    return g.weight[edge_index] / speed_mps;  // seconds
}

double TimeWeightStrategy::heuristic(const Graph& g, int32_t from, int32_t to) const {
    double dist_m = haversine_meters(g.lat[from], g.lon[from], g.lat[to], g.lon[to]);
    return dist_m / free_flow_speed_mps_;  // seconds
}

CongestionWeightStrategy::CongestionWeightStrategy(const WeightStrategy& base,
                                                    const std::vector<double>& multipliers)
    : base_(base), multipliers_(multipliers) {}

double CongestionWeightStrategy::edge_cost(const Graph& g, int32_t edge_index) const {
    return base_.edge_cost(g, edge_index) * multipliers_[edge_index];
}

double CongestionWeightStrategy::heuristic(const Graph& g, int32_t from, int32_t to) const {
    // Deliberately NOT scaled - see the header comment. Multipliers are
    // clamped to >= 1.0, so the base strategy's lower bound remains a valid
    // (if looser) lower bound under congestion.
    return base_.heuristic(g, from, to);
}

std::string CongestionWeightStrategy::name() const { return base_.name(); }
