#include "ReedsShepp.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1.0e-9;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

double mod2pi(double angle) {
    double result = std::fmod(angle, kTwoPi);
    if (result < 0.0) {
        result += kTwoPi;
    }
    return result;
}

double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

double pathLength(const std::vector<ReedsSheppSegment>& segments) {
    double length = 0.0;
    for (const ReedsSheppSegment& seg : segments) {
        length += std::abs(seg.length);
    }
    return length;
}

double tauOmega(double u, double v, double w) {
    if (std::abs(w) > kEpsilon) {
        const double cos_w = std::cos(w);
        const double numerator = u * u + v * v - 2.0 * u * v * cos_w;
        if (numerator >= 0.0) {
            return std::acos(std::clamp((u * u + v * v - 2.0 * u * v * cos_w) /
                                       (2.0 * u * v), -1.0, 1.0)) / w;
        }
    }
    return 0.0;
}

} // namespace

ReedsSheppGenerator::ReedsSheppGenerator(double min_turning_radius,
                                         double sample_step,
                                         double max_steer)
    : min_turning_radius_(min_turning_radius),
      sample_step_(sample_step),
      max_steer_(max_steer) {}

ReedsSheppGenerator::LocalState ReedsSheppGenerator::toLocal(
    const CarPose& start, const Pose2D& goal, double radius) const {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double c = std::cos(start.theta);
    const double s = std::sin(start.theta);
    return {
        (c * dx + s * dy) / radius,
        (-s * dx + c * dy) / radius,
        normalizeAngle(goal.theta - start.theta)
    };
}

ReedsSheppGenerator::LocalState ReedsSheppGenerator::toLocal(
    const CarPose& start, const CarPose& goal, double radius) const {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double c = std::cos(start.theta);
    const double s = std::sin(start.theta);
    return {
        (c * dx + s * dy) / radius,
        (-s * dx + c * dy) / radius,
        normalizeAngle(goal.theta - start.theta)
    };
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2;

    if (p1 <= kEpsilon) {
        return wc;
    }

    const double p2 = 2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));
    if (p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p2 - v2));
    const double t_val = std::atan2(v - p, u);
    const double u_mid = u + p;
    const double t_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && t_mid > kEpsilon && u_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, t_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + u_mid + t_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsr(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSR";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p2 = 2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p2 - v2));
    const double t_val = std::atan2(-v - p, u);
    const double u_mid = u + p;
    const double t_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && t_mid > kEpsilon && u_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, -t_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + u_mid - t_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsr(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSR";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-u * std::sin(t) + v * std::cos(t),
                                     u * std::cos(t) + v * std::sin(t) - 2.0);
    const double t_mid = mod2pi(t - t_val);
    const double u_mid = u * std::cos(t_val) + v * std::sin(t_val);

    if (t_val > kEpsilon && t_mid > kEpsilon && u_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, t_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + t_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(u * std::sin(t) - v * std::cos(t),
                                     -u * std::cos(t) - v * std::sin(t) + 2.0);
    const double t_mid = mod2pi(t - t_val);
    const double u_mid = u * std::cos(t_val) - v * std::sin(t_val);

    if (t_val < -kEpsilon && t_mid > kEpsilon && u_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, -t_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p - t_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rlr(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RLR";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = (6.0 - u2 - v2 + 2.0 * u * v * std::cos(t) +
                       2.0 * u * std::sin(t) - 2.0 * v * std::cos(t)) / 8.0;

    if (std::abs(p1) > 1.0 + kEpsilon) {
        return wc;
    }

    const double p = mod2pi(2.0 * kPi - std::acos(std::clamp(p1, -1.0, 1.0)));
    const double t_val = mod2pi(std::atan2(u, v) -
                                 std::atan2(v * std::sin(t), u * std::cos(t) + v * std::sin(t)));
    const double u_mid = mod2pi(t_val - t);
    const double v_mid = mod2pi(-t_val + t + p);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lrl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LRL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = (6.0 - u2 - v2 + 2.0 * u * v * std::cos(t) -
                       2.0 * u * std::sin(t) + 2.0 * v * std::cos(t)) / 8.0;

    if (std::abs(p1) > 1.0 + kEpsilon) {
        return wc;
    }

    const double p = mod2pi(2.0 * kPi - std::acos(std::clamp(p1, -1.0, 1.0)));
    const double t_val = mod2pi(std::atan2(-u, -v) +
                                 std::atan2(v * std::cos(t) - u * std::sin(t),
                                          u * std::cos(t) + v * std::sin(t)));
    const double u_mid = mod2pi(-t_val + t);
    const double v_mid = mod2pi(-t_val + t - p);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrsrs(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRSRS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double t_mid = mod2pi(t + t_val);
    const double u_mid = kPi - t_val;

    if (t_val < -kEpsilon && t_mid > kEpsilon && u_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, t_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, u_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + t_mid + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslrs(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSLRS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));
    const double p2 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(p1);
    const double t_val = std::atan2(-v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + p2) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrlrs(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRLRS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));
    const double p2 = -2.0 + p1;

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(p1);
    const double t_val = std::atan2(v, u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + p1) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lslrs(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSLRS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));
    const double p2 = 2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + p2) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsl3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSL|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslrsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSLRSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t))) / 2.0, -1.0, 1.0)));
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrlsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRLSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t))) / 2.0, -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lslrsl(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSLRSL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t))) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSLR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lslr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSLR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslr4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSLR*";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrr4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRR*";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrl3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRL|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrlr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRLR";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));
    const double p2 = 16.0 - u2 - v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double p_mid = std::sqrt(std::max(0.0, p2)) / 2.0;
    const double t_val = std::atan2(-v, -u);
    const double u_mid = mod2pi(tauOmega(p, p_mid, t));
    const double v_mid = mod2pi(t - t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrirl3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRIRL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));
    const double p2 = 16.0 - p1;

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double p_mid = std::sqrt(std::max(0.0, p2)) / 2.0;
    const double t_val = std::atan2(v, u);
    const double u_mid = mod2pi(tauOmega(p, p_mid, t));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p_mid, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid + p_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrlr4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRLR*";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));
    const double p2 = 16.0 - p1;

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double p_mid = std::sqrt(std::max(0.0, p2)) / 2.0;
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(tauOmega(p, p_mid, t));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrirl4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRIRL*";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));
    const double p2 = 16.0 - p1;

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double p_mid = std::sqrt(std::max(0.0, p2)) / 2.0;
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(tauOmega(p, p_mid, t));
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p_mid, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, -v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid + p_mid - v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lrlr3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LRLR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rlrl3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RLRL|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rlrL3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RLR|L";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lrlR3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LRL|R";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Left, v_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lslS3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSL|S";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));
    const double p2 = 2.0 + u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon || p2 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + p2) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrS3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSR|S";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));
    const double p2 = 2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon || p2 < kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + p2) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrS3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSR|S";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslS3(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSL|S";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrSl4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSR|SL";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslRs4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSL|RS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + u2 + v2 + 2.0 * (u * std::cos(t) - v * std::sin(t))) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (-t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrsl4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRSL*";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lslRs4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSL|RS";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = -2.0 + u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = mod2pi(std::acos(std::clamp((-2.0 + u2 + v2 - 2.0 * (u * std::cos(t) - v * std::sin(t))) / (2.0 * p), -1.0, 1.0)));
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon && p > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, v_mid, ReedsSheppDirection::Backward}
        };
        wc.total_length = (t_val + p + u_mid + v_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rsrs4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSRS|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsls4(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSLS|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) + v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::rslr5(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "RSLR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 + 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(-v, -u);
    const double u_mid = kPi - t_val;
    const double v_mid = mod2pi(t + t_val);

    if (t_val < -kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Right, -t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Left, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (-t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::lsrr5(
    const LocalState& s) const {
    WordCandidate wc;
    wc.word = "LSRR|";

    const double u = s.x;
    const double v = s.y;
    const double t = mod2pi(s.theta);

    const double u2 = u * u;
    const double v2 = v * v;
    const double p1 = 4.0 - u2 - v2 - 2.0 * (u * std::cos(t) - v * std::sin(t));

    if (p1 < -kEpsilon) {
        return wc;
    }

    const double p = std::sqrt(std::max(0.0, p1));
    const double t_val = std::atan2(v, -u);
    const double u_mid = kPi + t_val;
    const double v_mid = mod2pi(t - t_val);

    if (t_val > kEpsilon && u_mid > kEpsilon && v_mid > kEpsilon) {
        wc.segments = {
            {ReedsSheppSegmentType::Left, t_val, ReedsSheppDirection::Forward},
            {ReedsSheppSegmentType::Straight, p, ReedsSheppDirection::Backward},
            {ReedsSheppSegmentType::Right, u_mid, ReedsSheppDirection::Forward}
        };
        wc.total_length = (t_val + p + u_mid) * min_turning_radius_;
        wc.valid = true;
    }
    return wc;
}

ReedsSheppGenerator::WordCandidate ReedsSheppGenerator::generateWord(
    const LocalState& s, WordIndex idx) const {
    switch (idx) {
    case WI_LSL: return lsl(s);
    case WI_RSR: return rsr(s);
    case WI_LSR: return lsr(s);
    case WI_RSL: return rsl(s);
    case WI_RLR: return rlr(s);
    case WI_LRL: return lrl(s);
    case WI_RSRSRS: return rsrsrs(s);
    case WI_RSLRS: return rslrs(s);
    case WI_LSRLS: return lsrlrs(s);
    case WI_LSLRS: return lslrs(s);
    case WI_RSL3: return rsl3(s);
    case WI_LSR3: return lsr3(s);
    case WI_RSRSL: return rsrsl(s);
    case WI_RSLRSL: return rslrsl(s);
    case WI_LSRLSL: return lsrlsl(s);
    case WI_LSLRSL: return lslrsl(s);
    case WI_RSLR3: return rslr3(s);
    case WI_LSRR3: return lsrr3(s);
    case WI_LSLR3: return lslr3(s);
    case WI_RSLR4: return rslr4(s);
    case WI_LSRR4: return lsrr4(s);
    case WI_RSRL3: return rsrl3(s);
    case WI_RSRLR3: return rsrlr3(s);
    case WI_LSRIRL3: return lsrirl3(s);
    case WI_RSRLR4: return rsrlr4(s);
    case WI_LSRIRL4: return lsrirl4(s);
    case WI_LRLR3: return lrlr3(s);
    case WI_RLRL3: return rlrl3(s);
    case WI_RLR_L3: return rlrL3(s);
    case WI_LRL_R3: return lrlR3(s);
    case WI_LSL_S3: return lslS3(s);
    case WI_RSR_S3: return rsrS3(s);
    case WI_LSR_S3: return lsrS3(s);
    case WI_RSL_S3: return rslS3(s);
    case WI_RSR_SL4: return rsrSl4(s);
    case WI_RSL_RS4: return rslRs4(s);
    case WI_RSRSL4: return rsrsl4(s);
    case WI_LSL_RS4: return lslRs4(s);
    case WI_RSRS4: return rsrs4(s);
    case WI_LSLS4: return lsls4(s);
    case WI_RSLR5: return rslr5(s);
    case WI_LSRR5: return lsrr5(s);
    default: return {};
    }
}

CarPose ReedsSheppGenerator::advancePose(const CarPose& pose,
                                        ReedsSheppSegmentType type,
                                        ReedsSheppDirection dir,
                                        double distance,
                                        double radius) const {
    CarPose next = pose;
    next.direction = (dir == ReedsSheppDirection::Forward) ? 1 : -1;
    const double signed_dist = next.direction * distance;

    if (type == ReedsSheppSegmentType::Straight) {
        next.x += signed_dist * std::cos(pose.theta);
        next.y += signed_dist * std::sin(pose.theta);
        next.steer = 0.0;
        return next;
    }

    const double turn_sign = (type == ReedsSheppSegmentType::Left) ? 1.0 : -1.0;
    const double curvature = turn_sign / radius;
    const double next_theta = pose.theta + curvature * signed_dist;

    next.x += (1.0 / curvature) * (std::sin(next_theta) - std::sin(pose.theta));
    next.y += (1.0 / curvature) * (std::cos(pose.theta) - std::cos(next_theta));
    next.theta = normalizeAngle(next_theta);
    next.steer = turn_sign * max_steer_;
    return next;
}

std::vector<CarPose> ReedsSheppGenerator::samplePath(
    const CarPose& start,
    const CarPose& goal,
    const std::vector<ReedsSheppSegment>& segments,
    double radius) const {
    std::vector<CarPose> samples;
    CarPose pose = start;

    for (const ReedsSheppSegment& seg : segments) {
        double remaining = std::abs(seg.length);
        const double sign = (seg.length < 0.0) ? -1.0 : 1.0;

        while (remaining > kEpsilon) {
            const double step = std::min(sample_step_, remaining);
            pose = advancePose(pose, seg.type, seg.direction, sign * step, radius);
            samples.push_back(pose);
            remaining -= step;
        }
    }

    if (samples.empty()) {
        samples.push_back(goal);
    } else {
        samples.back().x = goal.x;
        samples.back().y = goal.y;
        samples.back().theta = normalizeAngle(goal.theta);
    }
    return samples;
}

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

    const LocalState s = toLocal(start, goal, min_turning_radius_);

    std::vector<WordCandidate> valid_candidates;

    for (int i = 0; i < WI_COUNT; ++i) {
        WordCandidate wc = generateWord(s, static_cast<WordIndex>(i));
        if (wc.valid && wc.total_length < kInfinity) {
            valid_candidates.push_back(wc);
        }
    }

    if (valid_candidates.empty()) {
        return std::nullopt;
    }

    auto best = std::min_element(
        valid_candidates.begin(), valid_candidates.end(),
        [](const WordCandidate& lhs, const WordCandidate& rhs) {
            return lhs.total_length < rhs.total_length;
        });

    ReedsSheppPath result;
    result.segments = best->segments;
    result.total_length = best->total_length;
    result.word = best->word;
    result.samples = samplePath(start, goal, result.segments, min_turning_radius_);

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
    if (min_turning_radius_ <= kEpsilon) {
        return std::nullopt;
    }

    const LocalState s = toLocal(start, goal, min_turning_radius_);

    std::optional<double> best_length;

    for (int i = 0; i < WI_COUNT; ++i) {
        WordCandidate wc = generateWord(s, static_cast<WordIndex>(i));
        if (wc.valid && (!best_length || wc.total_length < *best_length)) {
            best_length = wc.total_length;
        }
    }

    return best_length;
}
