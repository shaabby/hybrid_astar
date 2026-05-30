#include "HybridAstar.hpp"
#include "CollisionChecker.hpp"
#include "Heuristic.hpp"
#include "ReedsShepp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

/// @brief 圆周率常量。
constexpr double kPi = 3.14159265358979323846;

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void copyHeuristicTiming(TimingBreakdown& timing, const Heuristic& heuristic) {
    const auto* combined = dynamic_cast<const CombinedHeuristic*>(&heuristic);
    if (combined == nullptr) {
        return;
    }

    const HeuristicTiming& heuristic_timing = combined->timing();
    timing.obstacle_collect_ms = heuristic_timing.obstacle_collect_ms;
    timing.visibility_points_ms = heuristic_timing.visibility_points_ms;
    timing.visibility_graph_ms = heuristic_timing.visibility_graph_ms;
    timing.visibility_dijkstra_ms = heuristic_timing.visibility_dijkstra_ms;
    timing.obstacle_lookup_ms = heuristic_timing.obstacle_lookup_ms;
    timing.reverse_dijkstra_inflation_ms =
        heuristic_timing.reverse_dijkstra_inflation_ms;
    timing.reverse_dijkstra_ms = heuristic_timing.reverse_dijkstra_ms;
    timing.non_obstacle_heuristic_ms = heuristic_timing.non_obstacle_heuristic_ms;
    timing.obstacle_heuristic_ms = heuristic_timing.obstacle_heuristic_ms;
    timing.heuristic_estimate_calls = heuristic_timing.heuristic_estimate_calls;
}

/**
 * @brief Hybrid A* 搜索节点。
 *
 * 同时保存连续位姿（用于路径输出）和离散索引（用于 closed set 去重）。
 * segment 记录从父节点到本节点的连续轨迹段，最终回溯时拼接为完整路径。
 */
struct Node {
    CarPose pose;          ///< 连续位姿
    int x_index = 0;       ///< x 方向离散索引
    int y_index = 0;       ///< y 方向离散索引
    int theta_index = 0;   ///< 航向角离散分箱索引
    double g = 0.0;        ///< 从起点到当前节点的累计代价
    double h = 0.0;        ///< 启发式估计代价（到目标的距离）
    double f = 0.0;        ///< f = g + h，优先队列排序依据
    int parent = -1;       ///< 父节点在 nodes 数组中的索引，-1 表示起点
    std::vector<CarPose> segment; ///< 从父节点运动到本节点的连续轨迹
};

struct StateKey {
    int x_index = 0;
    int y_index = 0;
    int theta_index = 0;

    bool operator==(const StateKey& other) const = default;
};

struct StateKeyHash {
    std::size_t operator()(const StateKey& key) const {
        std::size_t seed = std::hash<int>{}(key.x_index);
        seed ^= std::hash<int>{}(key.y_index) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(key.theta_index) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        return seed;
    }
};

/**
 * @brief 优先队列（open set）条目。
 *
 * std::priority_queue 默认是大顶堆，因此 operator< 反向比较，
 * 使 f 值最小的节点位于堆顶。
 */
struct OpenEntry {
    double f = 0.0;        ///< 节点的 f 值
    int node_id = -1;      ///< 节点在 nodes 数组中的索引

    bool operator<(const OpenEntry& other) const {
        return f > other.f;
    }
};

/**
 * @brief 将角度归一化到 (-π, π] 区间。
 * @param[in] angle 输入角度，弧度
 * @return 归一化后的角度
 */
double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

/**
 * @brief 计算两个角度之间的最小差值。
 * @param[in] lhs 第一个角度，弧度
 * @param[in] rhs 第二个角度，弧度
 * @return 最小角度差，范围 [0, π]
 */
double angleDiff(double lhs, double rhs) {
    return std::abs(normalizeAngle(lhs - rhs));
}

/**
 * @brief 计算二维欧几里得距离。
 */
double distance2d(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 将连续航向角离散化为分箱索引。
 * @param[in] theta 连续航向角，弧度
 * @param[in] bins  总的分箱数量
 * @return 离散化后的索引，范围 [0, bins)
 */
int thetaIndex(double theta, int bins) {
    const double normalized = normalizeAngle(theta);
    const double shifted = normalized + kPi;
    int index = static_cast<int>(std::floor(shifted / (2.0 * kPi) * bins));
    if (index >= bins) {
        index = bins - 1;
    }
    return std::max(0, index);
}

StateKey makeKey(const Node& node) {
    return {
        .x_index = node.x_index,
        .y_index = node.y_index,
        .theta_index = node.theta_index
    };
}

/**
 * @brief 根据连续位姿和配置构造搜索节点。
 * @param[in] pose      连续位姿
 * @param[in] config    规划器配置
 * @param[in] heuristic 启发函数
 * @return 初始化后的 Node，g 为 0，h 由启发函数估计
 */
Node makeNode(const CarPose& pose,
              const HybridAstarConfig& config,
              const Heuristic& heuristic) {
    Node node;
    node.pose = pose;
    node.x_index = static_cast<int>(std::floor(pose.x / config.xy_resolution));
    node.y_index = static_cast<int>(std::floor(pose.y / config.xy_resolution));
    node.theta_index = thetaIndex(pose.theta, config.theta_bins);
    node.h = heuristic.estimate(pose);
    node.f = node.g + node.h;
    return node;
}

/**
 * @brief 判断当前位姿是否到达目标区域。
 *
 * 同时检查位置距离和航向角差值是否在容差范围内。
 */
bool isGoal(const CarPose& pose,
            const GridMap& map,
            const HybridAstarConfig& config) {
    const Pose2D& goal = map.goal();
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= config.goal_xy_tolerance
        && angleDiff(pose.theta, goal.theta) <= config.goal_theta_tolerance;
}

bool shouldTryAnalyticExpansion(const CarPose& pose,
                                const GridMap& map,
                                const HybridAstarConfig& config,
                                int iterations) {
    if (!config.enable_analytic_expansion) {
        return false;
    }

    const int interval = std::max(1, config.analytic_expansion_interval);
    if (iterations % interval != 0) {
        return false;
    }

    const Pose2D& goal = map.goal();
    return distance2d(pose.x, pose.y, goal.x, goal.y)
        <= config.analytic_expansion_distance;
}

std::optional<ReedsSheppPath> tryAnalyticExpansion(
    const ReedsSheppGenerator& generator,
    const CarPose& pose,
    const GridMap& map,
    const HybridAstarConfig& config,
    const ReedsSheppCollisionChecker& collision_checker,
    TimingBreakdown& timing) {
    const Pose2D& goal = map.goal();
    const std::vector<double> xy_offsets = {
        0.0,
        config.goal_xy_tolerance,
        -config.goal_xy_tolerance,
        config.goal_xy_tolerance * 0.5,
        -config.goal_xy_tolerance * 0.5
    };
    const std::vector<double> theta_offsets = {
        0.0,
        config.goal_theta_tolerance,
        -config.goal_theta_tolerance
    };

    for (double dx : xy_offsets) {
        for (double dy : xy_offsets) {
            if (std::hypot(dx, dy) > config.goal_xy_tolerance + 1.0e-9) {
                continue;
            }
            for (double dtheta : theta_offsets) {
                CarPose candidate_goal;
                candidate_goal.x = goal.x + dx;
                candidate_goal.y = goal.y + dy;
                candidate_goal.theta = normalizeAngle(goal.theta + dtheta);
                candidate_goal.direction = 1;

                std::optional<ReedsSheppPath> candidate;
                if (config.enable_timing) {
                    const auto generation_start = Clock::now();
                    candidate = generator.generate(pose, candidate_goal);
                    timing.analytic_rs_generation_ms += elapsedMs(
                        generation_start, Clock::now());
                    ++timing.analytic_rs_generation_calls;
                } else {
                    candidate = generator.generate(pose, candidate_goal);
                }

                bool collision_free = false;
                if (candidate) {
                    if (config.enable_timing) {
                        const auto collision_start = Clock::now();
                        collision_free = collision_checker.isCollisionFree(*candidate);
                        timing.analytic_collision_check_ms += elapsedMs(
                            collision_start, Clock::now());
                        ++timing.analytic_collision_check_calls;
                    } else {
                        collision_free = collision_checker.isCollisionFree(*candidate);
                    }
                }

                if (candidate && collision_free) {
                    return candidate;
                }
            }
        }
    }

    return std::nullopt;
}

/**
 * @brief 计算子节点的累计 g 代价。
 */
double computeTraversalCost(const Node& parent,
                            int direction,
                            double steer,
                            double segment_length,
                            double max_steer,
                            const HybridAstarConfig& config) {
    const double reverse_factor = direction < 0 ? config.reverse_penalty : 1.0;
    const double safe_max_steer = std::max(1.0e-9, std::abs(max_steer));
    const double steer_ratio = std::abs(steer) / safe_max_steer;
    const double steer_change_ratio = std::abs(steer - parent.pose.steer)
        / safe_max_steer;
    const double gear_switch_cost = parent.pose.direction != direction
        ? config.gear_switch_penalty
        : 0.0;
    const double steer_change_cost =
        config.steer_change_penalty * steer_change_ratio;

    return parent.g
        + segment_length * reverse_factor
        + segment_length * config.steer_penalty * steer_ratio
        + gear_switch_cost
        + steer_change_cost;
}

void computeHeuristicCost(Node& node, const Heuristic& heuristic) {
    node.h = heuristic.estimate(node.pose);
    node.f = node.g + node.h;
}

std::vector<int> reconstructNodeIds(const std::vector<Node>& nodes, int goal_id) {
    std::vector<int> ids;
    for (int id = goal_id; id >= 0; id = nodes[id].parent) {
        ids.push_back(id);
    }
    std::reverse(ids.begin(), ids.end());
    return ids;
}

std::vector<CarPose> reconstructPathFromIds(
    const std::vector<Node>& nodes,
    const std::vector<int>& ids,
    std::vector<int>* frame_starts = nullptr) {
    std::vector<CarPose> path;
    if (ids.empty()) {
        return path;
    }

    path.push_back(nodes[ids.front()].pose);
    if (frame_starts != nullptr) {
        frame_starts->clear();
        frame_starts->push_back(0);
    }
    for (std::size_t i = 1; i < ids.size(); ++i) {
        const std::vector<CarPose>& segment = nodes[ids[i]].segment;
        if (frame_starts != nullptr) {
            frame_starts->push_back(static_cast<int>(path.size()));
        }
        path.insert(path.end(), segment.begin(), segment.end());
    }
    return path;
}

void markSolutionEdges(std::vector<SearchTreeEdge>& edges,
                       const std::vector<int>& solution_ids) {
    if (solution_ids.size() < 2) {
        return;
    }
    for (SearchTreeEdge& edge : edges) {
        edge.in_solution = false;
    }
    for (std::size_t i = 1; i < solution_ids.size(); ++i) {
        const int parent = solution_ids[i - 1];
        const int child = solution_ids[i];
        for (SearchTreeEdge& edge : edges) {
            if (edge.parent == parent && edge.child == child) {
                edge.in_solution = true;
                break;
            }
        }
    }
}

void setSuccessfulSearchResult(PlanResult& result,
                               const std::vector<Node>& nodes,
                               int goal_id) {
    result.success = true;
    result.solution_node_ids = reconstructNodeIds(nodes, goal_id);
    result.path = reconstructPathFromIds(
        nodes, result.solution_node_ids,
        &result.solution_path_frame_starts);
    markSolutionEdges(result.search_tree, result.solution_node_ids);
}

} // namespace

HybridAstar::HybridAstar(HybridAstarConfig config,
                         std::shared_ptr<Heuristic> heuristic)
    : config_(config),
      heuristic_(std::move(heuristic)) {
    if (!heuristic_) {
        heuristic_ = std::make_shared<CombinedHeuristic>();
    }
}

PlanResult HybridAstar::plan(const GridMap& map, const Car& car) const {
    PlanResult result;
    if (config_.debug) {
        std::cerr << "[debug] planner: prepare heuristic "
                  << heuristicName() << '\n';
    }
    if (config_.enable_timing) {
        const auto prepare_start = Clock::now();
        heuristic_->prepare(map, car, config_);
        result.timing.heuristic_prepare_ms = elapsedMs(
            prepare_start, Clock::now());
    } else {
        heuristic_->prepare(map, car, config_);
    }
    const Heuristic& heuristic = *heuristic_;
    copyHeuristicTiming(result.timing, heuristic);
    if (config_.debug) {
        std::cerr << "[debug] planner: initialize collision checkers\n";
    }
    VehicleCollisionConfig collision_config;
    collision_config.safety_margin = config_.collision_safety_margin;
    const VehicleCollisionChecker collision_checker(map, car, collision_config);

    ReedsSheppCollisionConfig rs_collision_config;
    rs_collision_config.vehicle = collision_config;
    const ReedsSheppCollisionChecker rs_collision_checker(
        map, car, rs_collision_config);
    const ReedsSheppGenerator rs_generator(
        car.minTurningRadius(),
        config_.step_size,
        car.maxSteer(),
        config_.goal_xy_tolerance,
        config_.goal_theta_tolerance);

    // ------------------------------------------------------------------
    // 1. 初始化起点
    // ------------------------------------------------------------------
    CarPose start{
        .x = map.start().x,
        .y = map.start().y,
        .theta = map.start().theta,
        .steer = 0.0,
        .direction = 1
    };

    if (collision_checker.collides(start)) {
        if (config_.debug) {
            std::cerr << "[debug] planner: start pose is in collision\n";
        }
        return result; // 起点在障碍物内，直接返回失败
    }

    std::vector<Node> nodes;
    nodes.reserve(4096);

    Node start_node = makeNode(start, config_, heuristic);
    start_node.g = 0.0;
    start_node.f = start_node.h;
    nodes.push_back(start_node);

    // ------------------------------------------------------------------
    // 2. 初始化 open set（优先队列）和 closed set
    // ------------------------------------------------------------------
    std::priority_queue<OpenEntry> open;
    open.push({start_node.f, 0});

    std::unordered_map<StateKey, double, StateKeyHash> best_g;
    std::unordered_set<StateKey, StateKeyHash> closed;
    best_g[makeKey(start_node)] = 0.0;
    if (config_.debug) {
        std::cerr << "[debug] planner: start node ready"
                  << " h=" << start_node.h
                  << " open=" << open.size() << '\n';
    }

    // ------------------------------------------------------------------
    // 3. 预计算运动基元控制量
    // ------------------------------------------------------------------
    const std::vector<int> directions = config_.allow_reverse
        ? std::vector<int>{1, -1}
        : std::vector<int>{1};
    const std::vector<double> steers = {-car.maxSteer(), 0.0, car.maxSteer()};
    const int substeps = std::max(1, static_cast<int>(std::round(
        config_.primitive_length / config_.step_size)));
    if (config_.debug) {
        std::cerr << "[debug] planner: motion primitives ready"
                  << " directions=" << directions.size()
                  << " steers=" << steers.size()
                  << " substeps=" << substeps << '\n';
        std::cerr << "[debug] planner: search loop start"
                  << " max_iterations=" << config_.max_iterations << '\n';
    }

    // ------------------------------------------------------------------
    // 4. 主搜索循环（A*）
    // ------------------------------------------------------------------
    int iterations = 0;
    Clock::time_point search_start;
    if (config_.enable_timing) {
        search_start = Clock::now();
    }
    const auto finishResult = [&](PlanResult& plan_result) {
        if (config_.enable_timing) {
            plan_result.timing.search_loop_ms = elapsedMs(
                search_start, Clock::now());
            copyHeuristicTiming(plan_result.timing, heuristic);
        }
    };
    while (!open.empty() && iterations < config_.max_iterations) {
        ++iterations;
        result.iterations = iterations;

        const OpenEntry current_entry = open.top();
        open.pop();

        const int current_id = current_entry.node_id;
        const Node current = nodes[current_id];
        const StateKey current_key = makeKey(current);

        const auto current_best = best_g.find(current_key);
        if (current_best != best_g.end()
            && current.g > current_best->second + 1.0e-9) {
            continue;
        }

        // 跳过已关闭的节点（优先队列中可能存在过期条目）
        if (closed.contains(current_key)) {
            continue;
        }
        closed.insert(current_key);
        result.expanded.push_back(current.pose);
        const int progress_interval =
            std::max(1, config_.debug_progress_interval);
        if (config_.debug && iterations % progress_interval == 0) {
            const Pose2D& goal = map.goal();
            std::cerr << "[debug] planner: iteration=" << iterations
                      << " open=" << open.size()
                      << " closed=" << closed.size()
                      << " nodes=" << nodes.size()
                      << " current=(" << current.pose.x << ", "
                      << current.pose.y << ", " << current.pose.theta << ")"
                      << " distance_to_goal="
                      << distance2d(current.pose.x, current.pose.y,
                                    goal.x, goal.y)
                      << '\n';
        }

        // 到达目标，重建路径并返回
        if (isGoal(current.pose, map, config_)) {
            setSuccessfulSearchResult(result, nodes, current_id);
            result.generated_nodes = nodes.size();
            result.open_remaining = open.size();
            if (config_.debug) {
                std::cerr << "[debug] planner: goal reached by search"
                          << " iterations=" << result.iterations
                          << " path_poses=" << result.path.size()
                          << " expanded=" << result.expanded.size()
                          << '\n';
            }
            finishResult(result);
            return result;
        }

        // Reeds-Shepp analytic expansion: 在接近目标时尝试直接连接终点。
        //

        if (shouldTryAnalyticExpansion(
                current.pose, map, config_, iterations)) {
            if (config_.enable_timing) {
                ++result.timing.analytic_attempts;
            }
            Clock::time_point analytic_start;
            if (config_.enable_timing) {
                analytic_start = Clock::now();
            }
            const std::optional<ReedsSheppPath> analytic_path =
                tryAnalyticExpansion(rs_generator, current.pose, map,
                                     config_, rs_collision_checker,
                                     result.timing);
            if (config_.enable_timing) {
                result.timing.analytic_expansion_ms += elapsedMs(
                    analytic_start, Clock::now());
            }
            if (analytic_path) {
                if (config_.enable_timing) {
                    ++result.timing.analytic_successes;
                }
                setSuccessfulSearchResult(result, nodes, current_id);
                result.path.insert(result.path.end(),
                                   analytic_path->samples.begin(),
                                   analytic_path->samples.end());
                result.generated_nodes = nodes.size();
                result.open_remaining = open.size();
                if (config_.debug) {
                    std::cerr << "[debug] planner: analytic expansion succeeded"
                              << " iterations=" << result.iterations
                              << " path_poses=" << result.path.size()
                              << " expanded=" << result.expanded.size()
                              << '\n';
                }
                finishResult(result);
                return result;
            }
        }

        // ------------------------------------------------------------------
        // 5. 扩展邻居：遍历所有 (direction, steer) 组合
        // ------------------------------------------------------------------
        for (int direction : directions) {
            for (double steer : steers) {
                SearchTreeEdge edge;
                edge.parent = current_id;
                edge.from = current.pose;
                CarPose pose = current.pose;
                std::vector<CarPose> segment;
                segment.reserve(static_cast<std::size_t>(substeps));

                // 5.1 前向模拟：用自行车模型积分一段轨迹
                bool collision = false;
                for (int i = 0; i < substeps; ++i) {
                    pose = car.step(pose, steer, direction, config_.step_size);
                    pose.theta = normalizeAngle(pose.theta);

                    bool pose_collides = false;
                    if (config_.enable_timing) {
                        const auto collision_start = Clock::now();
                        pose_collides = collision_checker.collides(pose);
                        result.timing.primitive_collision_check_ms += elapsedMs(
                            collision_start, Clock::now());
                        ++result.timing.primitive_collision_check_calls;
                    } else {
                        pose_collides = collision_checker.collides(pose);
                    }

                    if (pose_collides) {
                        collision = true;
                        break;
                    }
                    segment.push_back(pose);

                    if (isGoal(pose, map, config_)) {
                        setSuccessfulSearchResult(result, nodes, current_id);
                        result.path.insert(result.path.end(),
                                           segment.begin(), segment.end());
                        result.generated_nodes = nodes.size();
                        result.open_remaining = open.size();
                        if (config_.debug) {
                            std::cerr << "[debug] planner: goal reached during primitive"
                                      << " iterations=" << result.iterations
                                      << " path_poses=" << result.path.size()
                                      << " expanded=" << result.expanded.size()
                                      << '\n';
                        }
                        finishResult(result);
                        return result;
                    }
                }

                if (collision || segment.empty()) {
                    edge.to = pose;
                    edge.segment = std::move(segment);
                    edge.collision = collision;
                    result.search_tree.push_back(std::move(edge));
                    continue; // 碰撞或没有移动，放弃该分支
                }

                // 5.2 构造子节点。先只计算 g，去重后再评估昂贵的启发式 h。
                Node next;
                next.pose = pose;
                next.x_index = static_cast<int>(
                    std::floor(pose.x / config_.xy_resolution));
                next.y_index = static_cast<int>(
                    std::floor(pose.y / config_.xy_resolution));
                next.theta_index = thetaIndex(pose.theta, config_.theta_bins);
                next.parent = current_id;
                next.segment = std::move(segment);

                // 5.3 计算累计 g 代价（含倒车、转向、换挡和转向变化惩罚）
                const double segment_length = config_.step_size
                    * static_cast<double>(next.segment.size());
                next.g = computeTraversalCost(current, direction, steer,
                                              segment_length, car.maxSteer(),
                                              config_);

                // 5.4 去重：检查 closed set 和 best_g
                const StateKey next_key = makeKey(next);
                if (closed.contains(next_key)) {
                    edge.to = pose;
                    edge.segment = next.segment;
                    edge.duplicate = true;
                    result.search_tree.push_back(std::move(edge));
                    continue;
                }

                const auto best = best_g.find(next_key);
                if (best != best_g.end() && best->second <= next.g) {
                    edge.to = pose;
                    edge.segment = next.segment;
                    edge.duplicate = true;
                    result.search_tree.push_back(std::move(edge));
                    continue; // 已有更优路径到达该离散状态
                }

                // 5.5 加入 open set
                computeHeuristicCost(next, heuristic);
                const int next_id = static_cast<int>(nodes.size());
                edge.child = next_id;
                edge.to = next.pose;
                edge.segment = next.segment;
                edge.accepted = true;
                result.search_tree.push_back(std::move(edge));
                best_g[next_key] = next.g;
                nodes.push_back(std::move(next));
                open.push({nodes[next_id].f, next_id});
            }
        }
    }

    // 达到最大迭代次数仍未找到路径，返回失败
    result.generated_nodes = nodes.size();
    result.open_remaining = open.size();
    if (config_.debug) {
        std::cerr << "[debug] planner: search failed"
                  << " iterations=" << result.iterations
                  << " expanded=" << result.expanded.size()
                  << " nodes=" << result.generated_nodes
                  << " open=" << result.open_remaining
                  << '\n';
    }
    finishResult(result);
    return result;
}

std::string HybridAstar::heuristicName() const {
    return heuristic_ ? heuristic_->name() : "unknown";
}
