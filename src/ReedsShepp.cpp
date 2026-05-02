#include "ReedsShepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

struct DubinsCandidate {
    std::array<ReedsSheppSegmentType, 3> types{};
    std::array<double, 3> lengths{};
    std::string word;
};

double mod2pi(double angle) {
    double result = std::fmod(angle, 2.0 * kPi);
    if (result < 0.0) {
        result += 2.0 * kPi;
    }
    return result;
}

double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

double pathLength(const std::vector<ReedsSheppSegment>& segments) {
    double length = 0.0;
    for (const ReedsSheppSegment& segment : segments) {
        length += std::abs(segment.length);
    }
    return length;
}

char segmentName(ReedsSheppSegmentType type) {
    switch (type) {
    case ReedsSheppSegmentType::Left:
        return 'L';
    case ReedsSheppSegmentType::Straight:
        return 'S';
    case ReedsSheppSegmentType::Right:
        return 'R';
    }
    return '?';
}

std::string wordName(const std::array<ReedsSheppSegmentType, 3>& types) {
    std::string word;
    for (ReedsSheppSegmentType type : types) {
        if (!word.empty()) {
            word += '-';
        }
        word += segmentName(type);
    }
    return word;
}

void addCandidate(std::vector<DubinsCandidate>& candidates,
                  const std::array<ReedsSheppSegmentType, 3>& types,
                  const std::array<double, 3>& lengths) {
    candidates.push_back({types, lengths, wordName(types)});
}

void addLsl(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double p2 = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb);
    if (p2 < -kEpsilon) {
        return;
    }

    const double tmp = std::atan2(cb - ca, d + sa - sb);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Left,
                  ReedsSheppSegmentType::Straight,
                  ReedsSheppSegmentType::Left},
                 {mod2pi(-alpha + tmp),
                  std::sqrt(std::max(0.0, p2)),
                  mod2pi(beta - tmp)});
}

void addRsr(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double p2 = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (-sa + sb);
    if (p2 < -kEpsilon) {
        return;
    }

    const double tmp = std::atan2(ca - cb, d - sa + sb);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Right,
                  ReedsSheppSegmentType::Straight,
                  ReedsSheppSegmentType::Right},
                 {mod2pi(alpha - tmp),
                  std::sqrt(std::max(0.0, p2)),
                  mod2pi(-beta + tmp)});
}

void addLsr(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double p2 = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb);
    if (p2 < -kEpsilon) {
        return;
    }

    const double p = std::sqrt(std::max(0.0, p2));
    const double tmp = std::atan2(-ca - cb, d + sa + sb)
        - std::atan2(-2.0, p);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Left,
                  ReedsSheppSegmentType::Straight,
                  ReedsSheppSegmentType::Right},
                 {mod2pi(-alpha + tmp),
                  p,
                  mod2pi(-beta + tmp)});
}

void addRsl(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double p2 = -2.0 + d * d + 2.0 * c_ab - 2.0 * d * (sa + sb);
    if (p2 < -kEpsilon) {
        return;
    }

    const double p = std::sqrt(std::max(0.0, p2));
    const double tmp = std::atan2(ca + cb, d - sa - sb)
        - std::atan2(2.0, p);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Right,
                  ReedsSheppSegmentType::Straight,
                  ReedsSheppSegmentType::Left},
                 {mod2pi(alpha - tmp),
                  p,
                  mod2pi(beta - tmp)});
}

void addRlr(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    if (std::abs(tmp) > 1.0 + kEpsilon) {
        return;
    }

    const double p = mod2pi(2.0 * kPi - std::acos(std::clamp(tmp, -1.0, 1.0)));
    const double t = mod2pi(alpha
        - std::atan2(ca - cb, d - sa + sb)
        + p * 0.5);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Right,
                  ReedsSheppSegmentType::Left,
                  ReedsSheppSegmentType::Right},
                 {t, p, mod2pi(alpha - beta - t + p)});
}

void addLrl(std::vector<DubinsCandidate>& candidates,
            double alpha,
            double beta,
            double d) {
    const double sa = std::sin(alpha);
    const double sb = std::sin(beta);
    const double ca = std::cos(alpha);
    const double cb = std::cos(beta);
    const double c_ab = std::cos(alpha - beta);

    const double tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;
    if (std::abs(tmp) > 1.0 + kEpsilon) {
        return;
    }

    const double p = mod2pi(2.0 * kPi - std::acos(std::clamp(tmp, -1.0, 1.0)));
    const double t = mod2pi(-alpha
        - std::atan2(ca - cb, d + sa - sb)
        + p * 0.5);
    addCandidate(candidates,
                 {ReedsSheppSegmentType::Left,
                  ReedsSheppSegmentType::Right,
                  ReedsSheppSegmentType::Left},
                 {t, p, mod2pi(beta - alpha - t + p)});
}

std::vector<DubinsCandidate> makeDubinsCandidates(const CarPose& start,
                                                  const CarPose& goal,
                                                  double radius) {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double c = std::cos(start.theta);
    const double s = std::sin(start.theta);
    const double local_x = (c * dx + s * dy) / radius;
    const double local_y = (-s * dx + c * dy) / radius;
    const double local_theta = normalizeAngle(goal.theta - start.theta);

    const double d = std::hypot(local_x, local_y);
    const double theta = std::atan2(local_y, local_x);
    const double alpha = mod2pi(-theta);
    const double beta = mod2pi(local_theta - theta);

    std::vector<DubinsCandidate> candidates;
    addLsl(candidates, alpha, beta, d);
    addRsr(candidates, alpha, beta, d);
    addLsr(candidates, alpha, beta, d);
    addRsl(candidates, alpha, beta, d);
    addRlr(candidates, alpha, beta, d);
    addLrl(candidates, alpha, beta, d);
    return candidates;
}

std::vector<ReedsSheppSegment> toSegments(const DubinsCandidate& candidate,
                                          double radius) {
    std::vector<ReedsSheppSegment> segments;
    segments.reserve(candidate.types.size());
    for (std::size_t i = 0; i < candidate.types.size(); ++i) {
        if (std::abs(candidate.lengths[i]) <= kEpsilon) {
            continue;
        }
        segments.push_back({candidate.types[i], candidate.lengths[i] * radius});
    }
    return segments;
}

CarPose advancePose(const CarPose& pose,
                    ReedsSheppSegmentType type,
                    double signed_distance,
                    double radius,
                    double max_steer) {
    CarPose next = pose;
    next.direction = signed_distance < 0.0 ? -1 : 1;

    if (type == ReedsSheppSegmentType::Straight) {
        next.x += signed_distance * std::cos(pose.theta);
        next.y += signed_distance * std::sin(pose.theta);
        next.steer = 0.0;
        return next;
    }

    const double turn_sign = type == ReedsSheppSegmentType::Left ? 1.0 : -1.0;
    const double curvature = turn_sign / radius;
    const double next_theta = pose.theta + curvature * signed_distance;
    const double inverse_curvature = 1.0 / curvature;

    next.x += inverse_curvature * (std::sin(next_theta) - std::sin(pose.theta));
    next.y += inverse_curvature * (std::cos(pose.theta) - std::cos(next_theta));
    next.theta = normalizeAngle(next_theta);
    next.steer = turn_sign * max_steer;
    return next;
}

std::vector<CarPose> sampleSegments(const CarPose& start,
                                    const CarPose& goal,
                                    const std::vector<ReedsSheppSegment>& segments,
                                    double radius,
                                    double sample_step,
                                    double max_steer) {
    std::vector<CarPose> samples;
    CarPose pose = start;

    for (const ReedsSheppSegment& segment : segments) {
        double remaining = std::abs(segment.length);
        const double sign = segment.length < 0.0 ? -1.0 : 1.0;

        while (remaining > kEpsilon) {
            const double step = std::min(sample_step, remaining);
            pose = advancePose(pose, segment.type, sign * step, radius, max_steer);
            samples.push_back(pose);
            remaining -= step;
        }
    }

    if (samples.empty()) {
        samples.push_back(goal);
    } else {
        CarPose& last = samples.back();
        last.x = goal.x;
        last.y = goal.y;
        last.theta = normalizeAngle(goal.theta);
    }
    return samples;
}

} // namespace

ReedsSheppGenerator::ReedsSheppGenerator(double min_turning_radius,
                                         double sample_step,
                                         double max_steer)
    : min_turning_radius_(min_turning_radius),
      sample_step_(sample_step),
      max_steer_(max_steer) {}

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
    if (min_turning_radius_ <= kEpsilon || sample_step_ <= kEpsilon) {
        return std::nullopt;
    }

    std::vector<ReedsSheppPath> paths;
    for (const DubinsCandidate& candidate
         : makeDubinsCandidates(start, goal, min_turning_radius_)) {
        std::vector<ReedsSheppSegment> segments = toSegments(
            candidate, min_turning_radius_);
        const double total_length = pathLength(segments);
        paths.push_back({
            segments,
            sampleSegments(start, goal, segments, min_turning_radius_,
                           sample_step_, max_steer_),
            total_length,
            candidate.word
        });
    }

    for (const DubinsCandidate& candidate
         : makeDubinsCandidates(goal, start, min_turning_radius_)) {
        std::vector<ReedsSheppSegment> segments = toSegments(
            candidate, min_turning_radius_);
        std::reverse(segments.begin(), segments.end());
        for (ReedsSheppSegment& segment : segments) {
            segment.length = -segment.length;
        }

        const double total_length = pathLength(segments);
        paths.push_back({
            segments,
            sampleSegments(start, goal, segments, min_turning_radius_,
                           sample_step_, max_steer_),
            total_length,
            "rev(" + candidate.word + ")"
        });
    }

    if (paths.empty()) {
        return std::nullopt;
    }

    return *std::min_element(
        paths.begin(), paths.end(),
        [](const ReedsSheppPath& lhs, const ReedsSheppPath& rhs) {
            return lhs.total_length < rhs.total_length;
        });
}
