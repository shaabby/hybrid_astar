#include "Heuristic.hpp"

#include <cmath>

void Heuristic::prepare(const GridMap&, const Car&, const HybridAstarConfig&) {}

void EuclideanHeuristic::prepare(const GridMap& map,
                                 const Car&,
                                 const HybridAstarConfig&) {
    goal_ = map.goal();
}

double EuclideanHeuristic::estimate(const CarPose& pose) const {
    const double dx = pose.x - goal_.x;
    const double dy = pose.y - goal_.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::string EuclideanHeuristic::name() const {
    return "euclidean";
}
