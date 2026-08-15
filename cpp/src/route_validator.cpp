#include "route_validator.hpp"

NoClosedRoadsRule::NoClosedRoadsRule(std::vector<std::pair<int64_t, int64_t>> closed_osm_edges)
    : closed_osm_edges_(std::move(closed_osm_edges)) {}

bool NoClosedRoadsRule::check(const Graph& g, const PathResult& candidate, std::string* reason) const {
    if (closed_osm_edges_.empty()) return true;
    for (size_t i = 0; i + 1 < candidate.path.size(); ++i) {
        int64_t u_osm = g.osm_id[candidate.path[i]];
        int64_t v_osm = g.osm_id[candidate.path[i + 1]];
        for (const auto& closed : closed_osm_edges_) {
            if (closed.first == u_osm && closed.second == v_osm) {
                if (reason) {
                    *reason = "uses closed road " + std::to_string(u_osm) + " -> " + std::to_string(v_osm);
                }
                return false;
            }
        }
    }
    return true;
}

SanityBoundsRule::SanityBoundsRule(double best_cost, double max_multiple)
    : best_cost_(best_cost), max_multiple_(max_multiple) {}

bool SanityBoundsRule::check(const Graph& /*g*/, const PathResult& candidate, std::string* reason) const {
    double limit = best_cost_ * max_multiple_;
    if (candidate.cost > limit) {
        if (reason) {
            *reason = "cost " + std::to_string(candidate.cost) + " exceeds " + std::to_string(max_multiple_) +
                       "x the best route's cost (" + std::to_string(best_cost_) + ")";
        }
        return false;
    }
    return true;
}

ReachabilityRule::ReachabilityRule(int32_t source, int32_t target) : source_(source), target_(target) {}

bool ReachabilityRule::check(const Graph& /*g*/, const PathResult& candidate, std::string* reason) const {
    if (candidate.path.empty() || candidate.path.front() != source_ || candidate.path.back() != target_) {
        if (reason) *reason = "path does not connect the requested source and target";
        return false;
    }
    return true;
}

void RouteValidationChain::add_rule(std::unique_ptr<ValidationRule> rule) { rules_.push_back(std::move(rule)); }

bool RouteValidationChain::validate(const Graph& g, const PathResult& candidate, std::string* reason) const {
    for (const auto& rule : rules_) {
        std::string local_reason;
        if (!rule->check(g, candidate, &local_reason)) {
            if (reason) *reason = rule->name() + ": " + local_reason;
            return false;
        }
    }
    return true;
}
