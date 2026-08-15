#include "snap.hpp"

#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusMeters = 6371000.0;
}  // namespace

double haversine_meters(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * kPi / 180.0;
    double phi2 = lat2 * kPi / 180.0;
    double dphi = (lat2 - lat1) * kPi / 180.0;
    double dlambda = (lon2 - lon1) * kPi / 180.0;

    double a = std::sin(dphi / 2) * std::sin(dphi / 2) +
               std::cos(phi1) * std::cos(phi2) * std::sin(dlambda / 2) * std::sin(dlambda / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return kEarthRadiusMeters * c;
}

int32_t snap_to_nearest_node(const Graph& g, double query_lat, double query_lon) {
    int32_t best = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int32_t i = 0; i < g.num_nodes(); ++i) {
        double d = haversine_meters(query_lat, query_lon, g.lat[i], g.lon[i]);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}
