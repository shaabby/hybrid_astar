#include "Car.hpp"
#include "CollisionChecker.hpp"
#include "GridMap.hpp"
#include "ReedsShepp.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSampleStep = 0.2;
constexpr double kPoseTolerance = 1.0e-3;
constexpr double kDistanceTolerance = 1.0e-6;

double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

double distance2d(const CarPose& lhs, const CarPose& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    return std::sqrt(dx * dx + dy * dy);
}

double distance2d(const Pose2D& lhs, const Pose2D& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    return std::sqrt(dx * dx + dy * dy);
}

CarPose carPose(double x, double y, double theta) {
    return {
        .x = x,
        .y = y,
        .theta = theta,
        .steer = 0.0,
        .direction = 1
    };
}

CarPose carPose(const Pose2D& pose) {
    return carPose(pose.x, pose.y, pose.theta);
}

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool reachesGoal(const ReedsSheppPath& path, const CarPose& goal) {
    if (path.samples.empty()) {
        return false;
    }

    const CarPose& final_pose = path.samples.back();
    return distance2d(final_pose, goal) <= kPoseTolerance
        && std::abs(normalizeAngle(final_pose.theta - goal.theta))
            <= kPoseTolerance;
}

bool samplesAreFinite(const ReedsSheppPath& path) {
    for (const CarPose& sample : path.samples) {
        if (!std::isfinite(sample.x) || !std::isfinite(sample.y)
            || !std::isfinite(sample.theta) || !std::isfinite(sample.steer)) {
            return false;
        }
    }
    return true;
}

bool hasBackwardSegment(const ReedsSheppPath& path) {
    for (const ReedsSheppSegment& segment : path.segments) {
        if (segment.direction == ReedsSheppDirection::Backward) {
            return true;
        }
    }
    return false;
}

const char* segmentTypeName(ReedsSheppSegmentType type) {
    switch (type) {
    case ReedsSheppSegmentType::Left:
        return "Left";
    case ReedsSheppSegmentType::Straight:
        return "Straight";
    case ReedsSheppSegmentType::Right:
        return "Right";
    }
    return "Straight";
}

const char* directionName(ReedsSheppDirection direction) {
    switch (direction) {
    case ReedsSheppDirection::Forward:
        return "Forward";
    case ReedsSheppDirection::Backward:
        return "Backward";
    }
    return "Forward";
}

void writePose(std::ostream& out, const CarPose& pose, int indent) {
    const std::string pad(static_cast<std::size_t>(indent), ' ');
    out << pad << "{\"x\": " << pose.x
        << ", \"y\": " << pose.y
        << ", \"theta\": " << pose.theta
        << ", \"steer\": " << pose.steer
        << ", \"direction\": " << pose.direction << "}";
}

void writePath(std::ostream& out,
               const std::string& name,
               const ReedsSheppPath& path,
               int indent) {
    const std::string pad(static_cast<std::size_t>(indent), ' ');
    out << pad << "{\n";
    out << pad << "  \"name\": \"" << name << "\",\n";
    out << pad << "  \"word\": \"" << path.word << "\",\n";
    out << pad << "  \"total_length\": " << path.total_length << ",\n";
    out << pad << "  \"segments\": [\n";
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const ReedsSheppSegment& segment = path.segments[i];
        out << pad << "    {\"type\": \"" << segmentTypeName(segment.type)
            << "\", \"length\": " << segment.length
            << ", \"direction\": \"" << directionName(segment.direction)
            << "\"}";
        if (i + 1 < path.segments.size()) {
            out << ',';
        }
        out << '\n';
    }
    out << pad << "  ],\n";
    out << pad << "  \"samples\": [\n";
    for (std::size_t i = 0; i < path.samples.size(); ++i) {
        writePose(out, path.samples[i], indent + 4);
        if (i + 1 < path.samples.size()) {
            out << ',';
        }
        out << '\n';
    }
    out << pad << "  ]\n";
    out << pad << "}";
}

bool writeOutputFile(const std::string& output_path,
                     const GridMap& map,
                     const Car& car,
                     const ReedsSheppPath& straight_path,
                     const ReedsSheppPath& map_path,
                     const ReedsSheppPath& reverse_path) {
    const std::filesystem::path path(output_path);
    std::filesystem::create_directories(path.parent_path());

    std::ofstream out(path);
    if (!out) {
        return false;
    }

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"map\": {\n";
    out << "    \"source\": \"map/empty.json\",\n";
    out << "    \"width\": " << map.width() << ",\n";
    out << "    \"height\": " << map.height() << ",\n";
    out << "    \"obstacle_count\": " << map.obstacleCount() << "\n";
    out << "  },\n";
    out << "  \"vehicle\": {\n";
    out << "    \"length\": " << car.length() << ",\n";
    out << "    \"width\": " << car.width() << ",\n";
    out << "    \"wheelbase\": " << car.wheelbase() << ",\n";
    out << "    \"rear_to_center\": " << car.rearToCenter() << ",\n";
    out << "    \"max_steer\": " << car.maxSteer() << ",\n";
    out << "    \"min_turning_radius\": " << car.minTurningRadius() << "\n";
    out << "  },\n";
    out << "  \"paths\": [\n";
    writePath(out, "straight", straight_path, 4);
    out << ",\n";
    writePath(out, "map_start_to_goal", map_path, 4);
    out << ",\n";
    writePath(out, "reverse_coverage", reverse_path, 4);
    out << "\n";
    out << "  ]\n";
    out << "}\n";

    return static_cast<bool>(out);
}

bool checkGeneratedPath(const std::string& name,
                        const ReedsSheppGenerator& generator,
                        const ReedsSheppCollisionChecker& collision_checker,
                        const CarPose& start,
                        const CarPose& goal) {
    const std::optional<ReedsSheppPath> path = generator.generate(start, goal);
    bool ok = true;
    ok &= expect(path.has_value(), name + " should generate a path");
    if (!path) {
        return false;
    }

    ok &= expect(!path->samples.empty(), name + " should contain samples");
    ok &= expect(!path->segments.empty(), name + " should contain segments");
    ok &= expect(std::isfinite(path->total_length) && path->total_length > 0.0,
                 name + " should have a positive finite length");
    ok &= expect(samplesAreFinite(*path), name + " samples should be finite");
    ok &= expect(reachesGoal(*path, goal), name + " should reach the goal");
    ok &= expect(collision_checker.isCollisionFree(*path),
                 name + " should be collision-free on map/empty.json");

    const std::optional<double> estimated =
        generator.estimateDistance(start, goal);
    ok &= expect(estimated.has_value(), name + " should estimate distance");
    if (estimated) {
        ok &= expect(std::abs(*estimated - path->total_length)
                         <= kDistanceTolerance,
                     name + " distance estimate should match generated length");
    }

    return ok;
}

} // namespace

int main() {
    try {
        const std::string map_path =
            std::string(HYBRID_ASTAR_SOURCE_DIR) + "/map/empty.json";
        const std::string output_path =
            std::string(HYBRID_ASTAR_SOURCE_DIR)
            + "/output/reeds_shepp_empty_map_test.json";
        const GridMap map = MapLoader::loadJson(map_path);
        const Car car(VehicleConfig{
            .length = 4.5,
            .width = 2.0,
            .wheelbase = 2.7,
            .rear_to_center = 1.35,
            .max_steer = 0.61
        });
        const VehicleCollisionChecker vehicle_checker(map, car);
        const ReedsSheppCollisionChecker collision_checker(map, car);
        const ReedsSheppGenerator generator(
            car.minTurningRadius(), kSampleStep, car.maxSteer());

        bool ok = true;
        ok &= expect(!map.empty(), "map/empty.json should load");
        ok &= expect(map.width() == 60 && map.height() == 36,
                     "map/empty.json should keep its expected dimensions");
        ok &= expect(map.obstacleCount() > 0,
                     "map/empty.json should keep boundary obstacles");
        ok &= expect(!vehicle_checker.collides(carPose(map.start())),
                     "start pose should not collide");
        ok &= expect(!vehicle_checker.collides(carPose(map.goal())),
                     "goal pose should not collide");
        ok &= expect(!vehicle_checker.collides(carPose(20.5, 18.0, 0.0)),
                     "interior test pose should not collide");

        const CarPose straight_start = carPose(6.5, 6.5, 0.0);
        const CarPose straight_goal = carPose(20.5, 6.5, 0.0);
        const std::optional<ReedsSheppPath> straight_path =
            generator.generate(straight_start, straight_goal);
        ok &= checkGeneratedPath("straight path",
                                 generator,
                                 collision_checker,
                                 straight_start,
                                 straight_goal);

        const CarPose map_start = carPose(map.start());
        const CarPose map_goal = carPose(map.goal());
        const std::optional<ReedsSheppPath> map_path_rs =
            generator.generate(map_start, map_goal);
        ok &= expect(map_path_rs.has_value(),
                     "map start-to-goal path should generate");
        if (map_path_rs) {
            const double euclidean_lower_bound =
                distance2d(map.start(), map.goal());
            ok &= expect(map_path_rs->total_length >= euclidean_lower_bound,
                         "map start-to-goal length should respect Euclidean lower bound");
            ok &= expect(samplesAreFinite(*map_path_rs),
                         "map start-to-goal samples should be finite");
            ok &= expect(reachesGoal(*map_path_rs, map_goal),
                         "map start-to-goal path should reach the goal");
            ok &= expect(collision_checker.isCollisionFree(*map_path_rs),
                         "map start-to-goal path should be collision-free");

            const std::optional<double> estimate =
                generator.estimateDistance(map_start, map_goal);
            ok &= expect(estimate.has_value(),
                         "map start-to-goal distance should estimate");
            if (estimate) {
                ok &= expect(std::abs(*estimate - map_path_rs->total_length)
                                 <= kDistanceTolerance,
                             "map start-to-goal estimate should match length");
            }
        }

        const CarPose reverse_start = carPose(24.0, 18.0, 0.0);
        const CarPose reverse_goal = carPose(21.0, 18.0, 0.0);
        const std::optional<ReedsSheppPath> reverse_path =
            generator.generate(reverse_start, reverse_goal);
        ok &= expect(reverse_path.has_value(),
                     "reverse coverage path should generate");
        if (reverse_path) {
            ok &= expect(hasBackwardSegment(*reverse_path),
                         "reverse coverage path should include a backward segment");
            ok &= expect(reachesGoal(*reverse_path, reverse_goal),
                         "reverse coverage path should reach the goal");
            ok &= expect(collision_checker.isCollisionFree(*reverse_path),
                         "reverse coverage path should be collision-free");
        }

        const ReedsSheppGenerator bad_radius(
            0.0, kSampleStep, car.maxSteer());
        ok &= expect(!bad_radius.generate(straight_start, straight_goal),
                     "zero turning radius should fail path generation");

        const ReedsSheppGenerator bad_step(
            car.minTurningRadius(), 0.0, car.maxSteer());
        ok &= expect(!bad_step.generate(straight_start, straight_goal),
                     "zero sample step should fail path generation");

        if (straight_path && map_path_rs && reverse_path) {
            ok &= expect(writeOutputFile(output_path,
                                         map,
                                         car,
                                         *straight_path,
                                         *map_path_rs,
                                         *reverse_path),
                         "should write output/reeds_shepp_empty_map_test.json");
            ok &= expect(std::filesystem::exists(output_path),
                         "output/reeds_shepp_empty_map_test.json should exist");
            ok &= expect(std::filesystem::file_size(output_path) > 0,
                         "output/reeds_shepp_empty_map_test.json should not be empty");
        } else {
            ok &= expect(false,
                         "all generated paths are required before writing output file");
        }

        if (!ok) {
            return EXIT_FAILURE;
        }

        std::cout << "Reeds-Shepp empty map tests passed\n"
                  << "Wrote " << output_path << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
