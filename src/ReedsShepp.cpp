#include "ReedsShepp.hpp"

#include "OmplReedsShepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1.0e-9;
constexpr double kSnapPositionTolerance = 1.0e-4;
constexpr double kSnapThetaTolerance = 1.0e-4;

double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

double distance2d(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

bool reachesGoal(const CarPose& pose,
                 const CarPose& goal,
                 double position_tolerance,
                 double theta_tolerance) {
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= position_tolerance
        && std::abs(normalizeAngle(pose.theta - goal.theta)) <= theta_tolerance;
}

bool canSnapToGoal(const CarPose& pose, const CarPose& goal) {
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= kSnapPositionTolerance
        && std::abs(normalizeAngle(pose.theta - goal.theta))
            <= kSnapThetaTolerance;
}

ompl_rs::State toNormalizedState(const CarPose& pose, double radius) {
    return {
        pose.x / radius,
        pose.y / radius,
        normalizeAngle(pose.theta)
    };
}

CarPose toCarPose(const ompl_rs::State& state,
                  double radius,
                  double steer,
                  int direction) {
    return {
        .x = state.x * radius,
        .y = state.y * radius,
        .theta = normalizeAngle(state.yaw),
        .steer = steer,
        .direction = direction
    };
}

ReedsSheppSegmentType convertType(ompl_rs::SegmentType type) {
    switch (type) {
    case ompl_rs::SegmentType::Left:
        return ReedsSheppSegmentType::Left;
    case ompl_rs::SegmentType::Right:
        return ReedsSheppSegmentType::Right;
    case ompl_rs::SegmentType::Straight:
    case ompl_rs::SegmentType::Nop:
        return ReedsSheppSegmentType::Straight;
    }
    return ReedsSheppSegmentType::Straight;
}

double steerFor(ompl_rs::SegmentType type, double max_steer) {
    switch (type) {
    case ompl_rs::SegmentType::Left:
        return max_steer;
    case ompl_rs::SegmentType::Right:
        return -max_steer;
    case ompl_rs::SegmentType::Straight:
    case ompl_rs::SegmentType::Nop:
        return 0.0;
    }
    return 0.0;
}

struct SegmentControl {
    ompl_rs::SegmentType type = ompl_rs::SegmentType::Straight;
    int direction = 1;
};

SegmentControl controlAtDistance(const ompl_rs::Path& path, double distance) {
    double travelled = 0.0;
    SegmentControl last;

    for (std::size_t i = 0; i < path.length.size(); ++i) {
        if (path.type[i] == ompl_rs::SegmentType::Nop
            || std::abs(path.length[i]) <= kEpsilon) {
            continue;
        }

        last.type = path.type[i];
        last.direction = path.length[i] < 0.0 ? -1 : 1;

        travelled += std::abs(path.length[i]);
        if (distance <= travelled + kEpsilon) {
            return last;
        }
    }

    return last;
}

} // namespace

ReedsSheppGenerator::ReedsSheppGenerator(double min_turning_radius,
                                         double sample_step,
                                         double max_steer,
                                         double goal_position_tolerance,
                                         double goal_theta_tolerance)
    : min_turning_radius_(min_turning_radius),
      sample_step_(sample_step),
      max_steer_(max_steer),
      goal_position_tolerance_(goal_position_tolerance),
      goal_theta_tolerance_(goal_theta_tolerance) {}

std::optional<ReedsSheppPath> ReedsSheppGenerator::generate(
    const CarPose& start,
    const Pose2D& goal) const {
    CarPose goal_pose;
    goal_pose.x = goal.x;
    goal_pose.y = goal.y;
    goal_pose.theta = goal.theta;
    goal_pose.direction = 1;
    return generate(start, goal_pose);
}

std::optional<ReedsSheppPath> ReedsSheppGenerator::generate(
    const CarPose& start,
    const CarPose& goal) const {
    if (min_turning_radius_ <= kEpsilon || sample_step_ <= kEpsilon
        || !std::isfinite(min_turning_radius_)) {
        return std::nullopt;
    }

    const ompl_rs::State normalized_start =
        toNormalizedState(start, min_turning_radius_);
    const ompl_rs::State normalized_goal =
        toNormalizedState(goal, min_turning_radius_);
    const ompl_rs::Path vendor_path =
        ompl_rs::shortestPath(normalized_start, normalized_goal);

    if (!std::isfinite(vendor_path.total_length)
        || vendor_path.total_length <= kEpsilon) {
        return std::nullopt;
    }

    ReedsSheppPath result;
    result.total_length = vendor_path.total_length * min_turning_radius_;
    result.word = "OMPL_RS_" + std::to_string(vendor_path.type_index);

    for (std::size_t i = 0; i < vendor_path.length.size(); ++i) {
        if (vendor_path.type[i] == ompl_rs::SegmentType::Nop
            || std::abs(vendor_path.length[i]) <= kEpsilon) {
            continue;
        }
        result.segments.push_back({
            .type = convertType(vendor_path.type[i]),
            .length = vendor_path.length[i] * min_turning_radius_,
            .direction = vendor_path.length[i] < 0.0
                ? ReedsSheppDirection::Backward
                : ReedsSheppDirection::Forward
        });
    }

    const double normalized_step = sample_step_ / min_turning_radius_;
    for (double distance = normalized_step;
         distance < vendor_path.total_length - kEpsilon;
         distance += normalized_step) {
        const SegmentControl control = controlAtDistance(vendor_path, distance);
        const ompl_rs::State sampled =
            ompl_rs::interpolate(normalized_start, vendor_path, distance);
        result.samples.push_back(toCarPose(
            sampled,
            min_turning_radius_,
            steerFor(control.type, max_steer_),
            control.direction));
    }

    const SegmentControl final_control =
        controlAtDistance(vendor_path, vendor_path.total_length);
    const ompl_rs::State final_state =
        ompl_rs::interpolate(normalized_start, vendor_path, vendor_path.total_length);
    CarPose final_pose = toCarPose(
        final_state,
        min_turning_radius_,
        steerFor(final_control.type, max_steer_),
        final_control.direction);

    if (canSnapToGoal(final_pose, goal)) {
        final_pose.x = goal.x;
        final_pose.y = goal.y;
        final_pose.theta = normalizeAngle(goal.theta);
    }

    result.samples.push_back(final_pose);

    if (!reachesGoal(result.samples.back(), goal, goal_position_tolerance_,
                    goal_theta_tolerance_)) {
        return std::nullopt;
    }

    return result;
}

std::optional<double> ReedsSheppGenerator::estimateDistance(
    const CarPose& start,
    const Pose2D& goal) const {
    CarPose goal_pose;
    goal_pose.x = goal.x;
    goal_pose.y = goal.y;
    goal_pose.theta = goal.theta;
    goal_pose.direction = 1;
    return estimateDistance(start, goal_pose);
}

std::optional<double> ReedsSheppGenerator::estimateDistance(
    const CarPose& start,
    const CarPose& goal) const {
    if (min_turning_radius_ <= kEpsilon || !std::isfinite(min_turning_radius_)) {
        return std::nullopt;
    }

    const ompl_rs::State normalized_start =
        toNormalizedState(start, min_turning_radius_);
    const ompl_rs::State normalized_goal =
        toNormalizedState(goal, min_turning_radius_);
    const double distance =
        ompl_rs::distance(normalized_start, normalized_goal)
        * min_turning_radius_;

    if (!std::isfinite(distance)) {
        return std::nullopt;
    }
    return distance;
}
