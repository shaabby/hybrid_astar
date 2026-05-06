#include "Heuristic.hpp"
#include "HybridAstar.hpp"
#include "ReedsShepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

int indexOf(int x, int y, int width) {
    return y * width + x;
}

bool inBounds(int x, int y, int width, int height) {
    return x >= 0 && y >= 0 && x < width && y < height;
}

bool withinInflationRadius(int candidate_x,
                           int candidate_y,
                           int obstacle_x,
                           int obstacle_y,
                           double radius) {
    if (radius <= 0.0) {
        return candidate_x == obstacle_x && candidate_y == obstacle_y;
    }

    const double center_x = static_cast<double>(candidate_x) + 0.5;
    const double center_y = static_cast<double>(candidate_y) + 0.5;
    const double obstacle_left = static_cast<double>(obstacle_x);
    const double obstacle_right = obstacle_left + 1.0;
    const double obstacle_bottom = static_cast<double>(obstacle_y);
    const double obstacle_top = obstacle_bottom + 1.0;

    const double dx = std::max({
        obstacle_left - center_x,
        0.0,
        center_x - obstacle_right
    });
    const double dy = std::max({
        obstacle_bottom - center_y,
        0.0,
        center_y - obstacle_top
    });
    return dx * dx + dy * dy <= radius * radius;
}

std::vector<std::uint8_t> makeInflatedObstacles(const GridMap& map,
                                                const Car& car,
                                                double inflate_alpha) {
    const int width = map.width();
    const int height = map.height();
    std::vector<std::uint8_t> inflated(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

    const double radius = std::max(0.0, inflate_alpha) * car.config().width * 0.5;
    const int cell_radius = static_cast<int>(std::ceil(radius));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!map.isObstacle(x, y)) {
                continue;
            }

            for (int yy = y - cell_radius; yy <= y + cell_radius; ++yy) {
                for (int xx = x - cell_radius; xx <= x + cell_radius; ++xx) {
                    if (!inBounds(xx, yy, width, height)) {
                        continue;
                    }
                    if (withinInflationRadius(xx, yy, x, y, radius)) {
                        inflated[static_cast<std::size_t>(indexOf(xx, yy, width))] = 1;
                    }
                }
            }
        }
    }

    return inflated;
}

std::vector<double> runReverseDijkstra(const GridMap& map,
                                       const std::vector<std::uint8_t>& blocked,
                                       double xy_resolution) {
    const int width = map.width();
    const int height = map.height();
    std::vector<double> distance(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
        kInfinity);

    const int goal_x = static_cast<int>(std::floor(map.goal().x));
    const int goal_y = static_cast<int>(std::floor(map.goal().y));
    if (!inBounds(goal_x, goal_y, width, height)) {
        return distance;
    }

    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const int goal_index = indexOf(goal_x, goal_y, width);
    distance[static_cast<std::size_t>(goal_index)] = 0.0;
    open.push({0.0, goal_index});

    const double straight_cost = std::max(1.0e-9, xy_resolution);
    const double diagonal_cost = std::sqrt(2.0) * straight_cost;
    const std::array<std::array<int, 3>, 8> neighbors = {{
        {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}}, {{0, -1, 0}},
        {{1, 1, 1}}, {{1, -1, 1}}, {{-1, 1, 1}}, {{-1, -1, 1}}
    }};

    while (!open.empty()) {
        const auto [current_distance, current_index] = open.top();
        open.pop();
        if (current_distance > distance[static_cast<std::size_t>(current_index)]) {
            continue;
        }

        const int x = current_index % width;
        const int y = current_index / width;
        for (const auto& neighbor : neighbors) {
            const int nx = x + neighbor[0];
            const int ny = y + neighbor[1];
            if (!inBounds(nx, ny, width, height)) {
                continue;
            }

            const int next_index = indexOf(nx, ny, width);
            if (blocked[static_cast<std::size_t>(next_index)] != 0
                && next_index != goal_index) {
                continue;
            }

            const double step_cost = neighbor[2] == 0
                ? straight_cost
                : diagonal_cost;
            const double next_distance = current_distance + step_cost;
            if (next_distance < distance[static_cast<std::size_t>(next_index)]) {
                distance[static_cast<std::size_t>(next_index)] = next_distance;
                open.push({next_distance, next_index});
            }
        }
    }

    return distance;
}

} // namespace

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

void CombinedHeuristic::prepare(const GridMap& map,
                                const Car& car,
                                const HybridAstarConfig& config) {
    goal_ = map.goal();
    width_ = map.width();
    height_ = map.height();
    xy_resolution_ = std::max(1.0e-9, config.xy_resolution);
    min_turning_radius_ = car.minTurningRadius();
    reeds_shepp_sample_step_ = config.step_size;
    max_steer_ = car.maxSteer();
    obstacle_enabled_ = config.enable_obstacle_heuristic;

    obstacle_distance_.clear();
    if (obstacle_enabled_) {
        const std::vector<std::uint8_t> inflated = makeInflatedObstacles(
            map, car, config.obstacle_heuristic_inflate_alpha);
        obstacle_distance_ = runReverseDijkstra(map, inflated, xy_resolution_);
    }
}

double CombinedHeuristic::estimate(const CarPose& pose) const {
    const double non_obs = nonObstacleEstimate(pose);
    const double obs = obstacle_enabled_ ? obstacleEstimate(pose) : 0.0;
    return std::max(non_obs, obs);
}

std::string CombinedHeuristic::name() const {
    return "combined";
}

double CombinedHeuristic::euclidean(const CarPose& pose) const {
    const double dx = pose.x - goal_.x;
    const double dy = pose.y - goal_.y;
    return std::sqrt(dx * dx + dy * dy);
}

double CombinedHeuristic::obstacleEstimate(const CarPose& pose) const {
    if (obstacle_distance_.empty()) {
        return euclidean(pose);
    }

    const int x = static_cast<int>(std::floor(pose.x));
    const int y = static_cast<int>(std::floor(pose.y));
    if (!inBounds(x, y, width_, height_)) {
        return euclidean(pose);
    }

    const double distance = obstacle_distance_[static_cast<std::size_t>(
        indexOf(x, y, width_))];
    if (!std::isfinite(distance)) {
        return euclidean(pose);
    }
    return distance;
}

double CombinedHeuristic::nonObstacleEstimate(const CarPose& pose) const {
    const ReedsSheppGenerator generator(
        min_turning_radius_, reeds_shepp_sample_step_, max_steer_);
    if (const std::optional<double> distance =
            generator.estimateDistance(pose, goal_)) {
        return *distance;
    }
    return euclidean(pose);
}
