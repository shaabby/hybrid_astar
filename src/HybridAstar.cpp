#include "HybridAstar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Node {
    CarPose pose;
    int x_index = 0;
    int y_index = 0;
    int theta_index = 0;
    double g = 0.0;
    double h = 0.0;
    double f = 0.0;
    int parent = -1;
    std::vector<CarPose> segment;
};

struct OpenEntry {
    double f = 0.0;
    int node_id = -1;

    bool operator<(const OpenEntry& other) const {
        return f > other.f;
    }
};

double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

double angleDiff(double lhs, double rhs) {
    return std::abs(normalizeAngle(lhs - rhs));
}

double distance2d(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

int thetaIndex(double theta, int bins) {
    const double normalized = normalizeAngle(theta);
    const double shifted = normalized + kPi;
    int index = static_cast<int>(std::floor(shifted / (2.0 * kPi) * bins));
    if (index >= bins) {
        index = bins - 1;
    }
    return std::max(0, index);
}

std::int64_t makeKey(int x_index, int y_index, int theta_index) {
    return (static_cast<std::int64_t>(x_index) << 40)
        ^ (static_cast<std::int64_t>(y_index) << 20)
        ^ static_cast<std::int64_t>(theta_index);
}

Node makeNode(const CarPose& pose,
              const GridMap& map,
              const HybridAstarConfig& config) {
    Node node;
    node.pose = pose;
    node.x_index = static_cast<int>(std::floor(pose.x / config.xy_resolution));
    node.y_index = static_cast<int>(std::floor(pose.y / config.xy_resolution));
    node.theta_index = thetaIndex(pose.theta, config.theta_bins);
    node.h = distance2d(pose.x, pose.y, map.goal().x, map.goal().y);
    node.f = node.g + node.h;
    return node;
}

bool isGoal(const CarPose& pose,
            const GridMap& map,
            const HybridAstarConfig& config) {
    const Pose2D& goal = map.goal();
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= config.goal_xy_tolerance
        && angleDiff(pose.theta, goal.theta) <= config.goal_theta_tolerance;
}

bool collidesCenter(const GridMap& map, const CarPose& pose) {
    const int x = static_cast<int>(std::floor(pose.x));
    const int y = static_cast<int>(std::floor(pose.y));
    return map.isObstacle(x, y);
}

std::vector<CarPose> reconstructPath(const std::vector<Node>& nodes, int goal_id) {
    std::vector<int> ids;
    for (int id = goal_id; id >= 0; id = nodes[id].parent) {
        ids.push_back(id);
    }
    std::reverse(ids.begin(), ids.end());

    std::vector<CarPose> path;
    if (ids.empty()) {
        return path;
    }

    path.push_back(nodes[ids.front()].pose);
    for (std::size_t i = 1; i < ids.size(); ++i) {
        const std::vector<CarPose>& segment = nodes[ids[i]].segment;
        path.insert(path.end(), segment.begin(), segment.end());
    }
    return path;
}

} // namespace

HybridAstar::HybridAstar(HybridAstarConfig config)
    : config_(config) {}

PlanResult HybridAstar::plan(const GridMap& map, const Car& car) const {
    PlanResult result;

    CarPose start{
        .x = map.start().x,
        .y = map.start().y,
        .theta = map.start().theta,
        .steer = 0.0,
        .direction = 1
    };

    if (collidesCenter(map, start)) {
        return result;
    }

    std::vector<Node> nodes;
    nodes.reserve(4096);

    Node start_node = makeNode(start, map, config_);
    start_node.g = 0.0;
    start_node.f = start_node.h;
    nodes.push_back(start_node);

    std::priority_queue<OpenEntry> open;
    open.push({start_node.f, 0});

    std::unordered_map<std::int64_t, double> best_g;
    std::unordered_set<std::int64_t> closed;
    best_g[makeKey(start_node.x_index, start_node.y_index, start_node.theta_index)] = 0.0;

    const std::vector<int> directions = config_.allow_reverse
        ? std::vector<int>{1, -1}
        : std::vector<int>{1};
    const std::vector<double> steers = {-car.maxSteer(), 0.0, car.maxSteer()};
    const int substeps = std::max(1, static_cast<int>(std::round(config_.primitive_length / config_.step_size)));

    int iterations = 0;
    while (!open.empty() && iterations < config_.max_iterations) {
        ++iterations;

        const OpenEntry current_entry = open.top();
        open.pop();

        const int current_id = current_entry.node_id;
        const Node current = nodes[current_id];
        const std::int64_t current_key = makeKey(
            current.x_index, current.y_index, current.theta_index);

        if (closed.contains(current_key)) {
            continue;
        }
        closed.insert(current_key);
        result.expanded.push_back(current.pose);

        if (isGoal(current.pose, map, config_)) {
            result.success = true;
            result.path = reconstructPath(nodes, current_id);
            return result;
        }

        for (int direction : directions) {
            for (double steer : steers) {
                CarPose pose = current.pose;
                std::vector<CarPose> segment;
                segment.reserve(static_cast<std::size_t>(substeps));

                bool collision = false;
                for (int i = 0; i < substeps; ++i) {
                    pose = car.step(pose, steer, direction, config_.step_size);
                    pose.theta = normalizeAngle(pose.theta);

                    if (collidesCenter(map, pose)) {
                        collision = true;
                        break;
                    }
                    segment.push_back(pose);
                }

                if (collision || segment.empty()) {
                    continue;
                }

                Node next = makeNode(pose, map, config_);
                next.parent = current_id;
                next.segment = std::move(segment);

                const double reverse_cost = direction < 0 ? config_.reverse_penalty : 1.0;
                const double steer_cost = 1.0 + std::abs(steer) * config_.steer_penalty;
                next.g = current.g + config_.primitive_length * reverse_cost * steer_cost;
                next.h = distance2d(next.pose.x, next.pose.y, map.goal().x, map.goal().y);
                next.f = next.g + next.h;

                const std::int64_t next_key = makeKey(
                    next.x_index, next.y_index, next.theta_index);
                if (closed.contains(next_key)) {
                    continue;
                }

                const auto best = best_g.find(next_key);
                if (best != best_g.end() && best->second <= next.g) {
                    continue;
                }

                const int next_id = static_cast<int>(nodes.size());
                best_g[next_key] = next.g;
                nodes.push_back(std::move(next));
                open.push({nodes[next_id].f, next_id});
            }
        }
    }

    return result;
}
