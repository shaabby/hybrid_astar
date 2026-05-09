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

    enum WordIndex {
        WI_LSL = 0, WI_RSR, WI_LSR, WI_RSL,
        WI_RLR, WI_LRL,
        WI_RSRSRS, WI_RSLRS, WI_LSRLS, WI_LSLRS,
        WI_RSL3, WI_LSR3,
        WI_RSRSL, WI_RSLRSL, WI_LSRLSL, WI_LSLRSL,
        WI_RSLR3, WI_LSRR3,
        WI_LSLR3, WI_RSLR4, WI_LSRR4, WI_RSRL3,
        WI_RSRLR3, WI_LSRIRL3, WI_RSRLR4, WI_LSRIRL4,
        WI_LRLR3, WI_RLRL3,
        WI_RLR_L3, WI_LRL_R3,
        WI_LSL_S3, WI_RSR_S3, WI_LSR_S3, WI_RSL_S3,
        WI_RSR_SL4, WI_RSL_RS4, WI_RSRSL4, WI_LSL_RS4,
        WI_RSRS4, WI_LSLS4, WI_RSLR5, WI_LSRR5,
        WI_COUNT
    };

    struct WordCandidate {
        std::vector<ReedsSheppSegment> segments;
        double total_length = 0.0;
        std::string word;
        bool valid = false;
    };

    struct LocalState {
        double x;
        double y;
        double theta;
    };

    LocalState toLocal(const CarPose& start, const Pose2D& goal, double radius) const;
    LocalState toLocal(const CarPose& start, const CarPose& goal, double radius) const;

    WordCandidate generateWord(const LocalState& s, WordIndex idx) const;

    WordCandidate lsl(const LocalState& s) const;
    WordCandidate rsr(const LocalState& s) const;
    WordCandidate lsr(const LocalState& s) const;
    WordCandidate rsl(const LocalState& s) const;
    WordCandidate rlr(const LocalState& s) const;
    WordCandidate lrl(const LocalState& s) const;

    WordCandidate rsrsrs(const LocalState& s) const;
    WordCandidate rslrs(const LocalState& s) const;
    WordCandidate lsrlrs(const LocalState& s) const;
    WordCandidate lslrs(const LocalState& s) const;
    WordCandidate rsl3(const LocalState& s) const;
    WordCandidate lsr3(const LocalState& s) const;

    WordCandidate rsrsl(const LocalState& s) const;
    WordCandidate rslrsl(const LocalState& s) const;
    WordCandidate lsrlsl(const LocalState& s) const;
    WordCandidate lslrsl(const LocalState& s) const;
    WordCandidate rslr3(const LocalState& s) const;
    WordCandidate lsrr3(const LocalState& s) const;

    WordCandidate lslr3(const LocalState& s) const;
    WordCandidate rslr4(const LocalState& s) const;
    WordCandidate lsrr4(const LocalState& s) const;
    WordCandidate rsrl3(const LocalState& s) const;

    WordCandidate rsrlr3(const LocalState& s) const;
    WordCandidate lsrirl3(const LocalState& s) const;
    WordCandidate rsrlr4(const LocalState& s) const;
    WordCandidate lsrirl4(const LocalState& s) const;

    WordCandidate lrlr3(const LocalState& s) const;
    WordCandidate rlrl3(const LocalState& s) const;

    WordCandidate rlrL3(const LocalState& s) const;
    WordCandidate lrlR3(const LocalState& s) const;

    WordCandidate lslS3(const LocalState& s) const;
    WordCandidate rsrS3(const LocalState& s) const;
    WordCandidate lsrS3(const LocalState& s) const;
    WordCandidate rslS3(const LocalState& s) const;

    WordCandidate rsrSl4(const LocalState& s) const;
    WordCandidate rslRs4(const LocalState& s) const;
    WordCandidate rsrsl4(const LocalState& s) const;
    WordCandidate lslRs4(const LocalState& s) const;

    WordCandidate rsrs4(const LocalState& s) const;
    WordCandidate lsls4(const LocalState& s) const;
    WordCandidate rslr5(const LocalState& s) const;
    WordCandidate lsrr5(const LocalState& s) const;

    CarPose advancePose(const CarPose& pose,
                        ReedsSheppSegmentType type,
                        ReedsSheppDirection dir,
                        double distance,
                        double radius) const;

    CarPose traceEndpoint(const CarPose& start,
                          const std::vector<ReedsSheppSegment>& segments,
                          double radius) const;

    std::vector<CarPose> samplePath(const CarPose& start,
                                    const std::vector<ReedsSheppSegment>& segments,
                                    double radius) const;
};
