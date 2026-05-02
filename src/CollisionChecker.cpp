#include "CollisionChecker.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

using Point2 = std::array<double, 2>;
using Quad = std::array<Point2, 4>;

double dot(const Point2& lhs, const Point2& rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1];
}

void project(const Quad& polygon, const Point2& axis, double& min, double& max) {
    min = dot(polygon[0], axis);
    max = min;
    for (std::size_t i = 1; i < polygon.size(); ++i) {
        const double value = dot(polygon[i], axis);
        min = std::min(min, value);
        max = std::max(max, value);
    }
}

bool overlapsOnAxis(const Quad& lhs, const Quad& rhs, const Point2& axis) {
    constexpr double kOverlapEpsilon = 1.0e-9;
    double lhs_min = 0.0;
    double lhs_max = 0.0;
    double rhs_min = 0.0;
    double rhs_max = 0.0;
    project(lhs, axis, lhs_min, lhs_max);
    project(rhs, axis, rhs_min, rhs_max);
    return lhs_max >= rhs_min - kOverlapEpsilon
        && rhs_max >= lhs_min - kOverlapEpsilon;
}

bool rectanglesOverlap(const Quad& lhs, const Quad& rhs) {
    const std::array<Point2, 4> axes = {{
        {lhs[1][0] - lhs[0][0], lhs[1][1] - lhs[0][1]},
        {lhs[2][0] - lhs[1][0], lhs[2][1] - lhs[1][1]},
        {1.0, 0.0},
        {0.0, 1.0}
    }};

    for (const Point2& axis : axes) {
        if (!overlapsOnAxis(lhs, rhs, axis)) {
            return false;
        }
    }
    return true;
}

Quad gridCellCorners(int x, int y) {
    const double left = static_cast<double>(x);
    const double bottom = static_cast<double>(y);
    const double right = left + 1.0;
    const double top = bottom + 1.0;
    return {{
        {left, bottom},
        {right, bottom},
        {right, top},
        {left, top}
    }};
}

Quad vehicleCorners(const Car& car,
                    const CarPose& pose,
                    double safety_margin) {
    const VehicleConfig& vehicle = car.config();
    const double margin = std::max(0.0, safety_margin);
    const double front = vehicle.length - vehicle.rear_to_center + margin;
    const double rear = -vehicle.rear_to_center - margin;
    const double half_width = vehicle.width * 0.5 + margin;

    const Quad local = {{
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    }};

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    Quad world{};

    for (std::size_t i = 0; i < local.size(); ++i) {
        const double lx = local[i][0];
        const double ly = local[i][1];
        world[i] = {
            pose.x + lx * c - ly * s,
            pose.y + lx * s + ly * c
        };
    }

    return world;
}

} // namespace

VehicleCollisionChecker::VehicleCollisionChecker(
    const GridMap& map,
    const Car& car,
    VehicleCollisionConfig config)
    : map_(map),
      car_(car),
      config_(config) {}

bool VehicleCollisionChecker::collides(const CarPose& pose) const {
    const Quad corners = vehicleCorners(car_, pose, config_.safety_margin);

    double min_x = corners[0][0];
    double max_x = corners[0][0];
    double min_y = corners[0][1];
    double max_y = corners[0][1];
    for (const Point2& corner : corners) {
        min_x = std::min(min_x, corner[0]);
        max_x = std::max(max_x, corner[0]);
        min_y = std::min(min_y, corner[1]);
        max_y = std::max(max_y, corner[1]);
    }

    const int x0 = static_cast<int>(std::floor(min_x));
    const int x1 = static_cast<int>(std::floor(max_x));
    const int y0 = static_cast<int>(std::floor(min_y));
    const int y1 = static_cast<int>(std::floor(max_y));

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const Quad cell = gridCellCorners(x, y);
            if (!rectanglesOverlap(corners, cell)) {
                continue;
            }

            if (!map_.inBounds(x, y)) {
                if (config_.treat_out_of_bounds_as_collision) {
                    return true;
                }
                continue;
            }

            if (map_.isObstacle(x, y)) {
                return true;
            }
        }
    }

    return false;
}

bool VehicleCollisionChecker::isCollisionFree(
    const std::vector<CarPose>& samples) const {
    return !firstCollisionIndex(samples).has_value();
}

std::optional<std::size_t> VehicleCollisionChecker::firstCollisionIndex(
    const std::vector<CarPose>& samples) const {
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (collides(samples[i])) {
            return i;
        }
    }
    return std::nullopt;
}

ReedsSheppCollisionChecker::ReedsSheppCollisionChecker(
    const GridMap& map,
    const Car& car,
    ReedsSheppCollisionConfig config)
    : vehicle_checker_(map, car, config.vehicle),
      config_(config) {}

bool ReedsSheppCollisionChecker::isCollisionFree(
    const ReedsSheppPath& path) const {
    return !firstCollisionSample(path).has_value();
}

std::optional<std::size_t> ReedsSheppCollisionChecker::firstCollisionSample(
    const ReedsSheppPath& path) const {
    if (path.samples.empty()) {
        if (config_.require_non_empty_samples) {
            return 0;
        }
        return std::nullopt;
    }
    return vehicle_checker_.firstCollisionIndex(path.samples);
}

bool ReedsSheppCollisionChecker::collides(const CarPose& pose) const {
    return vehicle_checker_.collides(pose);
}
