#include "Car.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kEpsilon = 1.0e-9;

} // namespace

Car::Car() = default;

Car::Car(VehicleConfig config)
    : config_(config) {}

double Car::length() const {
    return config_.length;
}

double Car::width() const {
    return config_.width;
}

double Car::wheelbase() const {
    return config_.wheelbase;
}

double Car::rearToCenter() const {
    return config_.rear_to_center;
}

double Car::maxSteer() const {
    return config_.max_steer;
}

double Car::minTurningRadius() const {
    const double tan_steer = std::tan(config_.max_steer);
    if (std::abs(tan_steer) < kEpsilon) {
        return std::numeric_limits<double>::infinity();
    }
    return config_.wheelbase / std::abs(tan_steer);
}

const VehicleConfig& Car::config() const {
    return config_;
}

double Car::clampSteer(double steer) const {
    return std::clamp(steer, -config_.max_steer, config_.max_steer);
}

CarPose Car::step(const CarPose& pose,
                  double steer,
                  int direction,
                  double distance) const {
    const int sign = direction < 0 ? -1 : 1;
    const double clamped_steer = clampSteer(steer);
    const double signed_distance = static_cast<double>(sign) * distance;

    CarPose next = pose;
    const double tan_steer = std::tan(clamped_steer);

    if (std::abs(config_.wheelbase) <= kEpsilon
        || std::abs(tan_steer) <= kEpsilon) {
        next.x += signed_distance * std::cos(pose.theta);
        next.y += signed_distance * std::sin(pose.theta);
    } else {
        const double radius = config_.wheelbase / tan_steer;
        const double next_theta = pose.theta + signed_distance / radius;

        next.x += radius * (std::sin(next_theta) - std::sin(pose.theta));
        next.y += radius * (std::cos(pose.theta) - std::cos(next_theta));
        next.theta = next_theta;
    }

    next.steer = clamped_steer;
    next.direction = sign;
    return next;
}

std::array<std::array<double, 2>, 4> Car::bodyCorners(const CarPose& pose) const {
    const double front = config_.length - config_.rear_to_center;
    const double rear = -config_.rear_to_center;
    const double half_width = config_.width * 0.5;

    const std::array<std::array<double, 2>, 4> local = {{
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    }};

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    std::array<std::array<double, 2>, 4> world{};

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
