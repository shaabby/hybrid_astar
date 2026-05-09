#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <optional>
#include <string>
#include <vector>

enum class ReedsSheppSegmentType {
    Left,
    Straight,
    Right
};

enum class ReedsSheppDirection {
    Forward,
    Backward
};

struct ReedsSheppSegment {
    ReedsSheppSegmentType type = ReedsSheppSegmentType::Straight;
    double length = 0.0;
    ReedsSheppDirection direction = ReedsSheppDirection::Forward;
};

struct ReedsSheppPath {
    std::vector<ReedsSheppSegment> segments;
    std::vector<CarPose> samples;
    double total_length = 0.0;
    std::string word;
};

class ReedsSheppGenerator {
public:
    ReedsSheppGenerator(double min_turning_radius,
                        double sample_step,
                        double max_steer = 0.0,
                        double goal_position_tolerance = 1.0e-4,
                        double goal_theta_tolerance = 1.0e-4);

    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const Pose2D& goal) const;

    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const CarPose& goal) const;

    [[nodiscard]] std::optional<double> estimateDistance(
        const CarPose& start,
        const Pose2D& goal) const;

    [[nodiscard]] std::optional<double> estimateDistance(
        const CarPose& start,
        const CarPose& goal) const;

private:
    double min_turning_radius_ = 1.0;
    double sample_step_ = 0.2;
    double max_steer_ = 0.0;
    double goal_position_tolerance_ = 1.0e-4;
    double goal_theta_tolerance_ = 1.0e-4;
};
