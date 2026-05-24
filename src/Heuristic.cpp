/**
 * @file Heuristic.cpp
 * @brief Hybrid A*启发函数实现
 *
 * 实现欧几里得距离启发式和组合启发式。
 * 组合启发式综合无障碍Reeds-Shepp距离和基于可视点的图搜索代价。
 */

#include "Heuristic.hpp"
#include "HybridAstar.hpp"
#include "LineOfSight.hpp"
#include "ReedsShepp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#define debug(...) do { \
    fprintf(stderr, __VA_ARGS__); \
} while (0)

namespace {

/// @brief 无穷大常量
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct HeuristicDiffLogStats {
    std::uint64_t positive = 0;
    std::uint64_t negative = 0;
    std::uint64_t zero = 0;

    ~HeuristicDiffLogStats() {
        debug("[heuristic] diff_summary positive=%llu negative=%llu zero=%llu total=%llu\n",
              static_cast<unsigned long long>(positive),
              static_cast<unsigned long long>(negative),
              static_cast<unsigned long long>(zero),
              static_cast<unsigned long long>(positive + negative + zero));
    }
};

HeuristicDiffLogStats& heuristicDiffLogStats() {
    static HeuristicDiffLogStats stats;
    return stats;
}

/**
 * @brief 计算二维欧几里得距离
 * @param[in] lhs 第一个二维点
 * @param[in] rhs 第二个二维点
 * @return 两点间的欧几里得距离
 */
double distance2d(Point2D lhs, Point2D rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    return std::sqrt(dx * dx + dy * dy);
}

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief 将Pose2D转换为Point2D
 * @param[in] goal 二维位姿
 * @return 二维点
 */
Point2D goalPoint(const Pose2D& goal) {
    return {.x = goal.x, .y = goal.y};
}

/**
 * @brief 在图上运行Dijkstra算法计算到起点的最短距离
 * @param[in] graph       邻接表表示的图
 * @param[in] start_index 起点索引
 * @return 每个节点到起点的最短距离数组
 */
std::vector<double> runGraphDijkstra(
    const std::vector<std::vector<std::pair<int, double>>>& graph,
    int start_index) {
    std::vector<double> distance(graph.size(), kInfinity);
    if (start_index < 0 || start_index >= static_cast<int>(graph.size())) {
        return distance;
    }
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    distance[static_cast<std::size_t>(start_index)] = 0.0;
    open.push({0.0, start_index});

    while (!open.empty()) {
        const auto [current_distance, current] = open.top();
        open.pop();
        if (current_distance > distance[static_cast<std::size_t>(current)]) {
            continue;
        }

        for (const auto& [next, edge_cost] : graph[static_cast<std::size_t>(current)]) {
            const double next_distance = current_distance + edge_cost;
            if (next_distance < distance[static_cast<std::size_t>(next)]) {
                distance[static_cast<std::size_t>(next)] = next_distance;
                open.push({next_distance, next});
            }
        }
    }

    return distance;
}

/**
 * @brief 检查单元格是否为障碍物或地图外区域
 * @param[in] map 栅格地图
 * @param[in] x   单元格x坐标
 * @param[in] y   单元格y坐标
 * @return true如果是障碍物或地图外
 */
bool cellObstacleOrBlockedOutside(const GridMap& map, int x, int y) {
    return !map.inBounds(x, y) || map.isObstacle(x, y);
}

/**
 * @brief 统计单元格周围的障碍物数量
 * @param[in] map 栅格地图
 * @param[in] x   单元格x坐标
 * @param[in] y   单元格y坐标
 * @return 四个角点中障碍物/边界点的数量
 */
int surroundingObstacleCount(const GridMap& map, int x, int y) {
    int count = 0;
    count += cellObstacleOrBlockedOutside(map, x - 1, y - 1) ? 1 : 0;
    count += cellObstacleOrBlockedOutside(map, x, y - 1) ? 1 : 0;
    count += cellObstacleOrBlockedOutside(map, x - 1, y) ? 1 : 0;
    count += cellObstacleOrBlockedOutside(map, x, y) ? 1 : 0;
    return count;
}

} // namespace

/** @brief 基类prepare默认空实现。 */
void Heuristic::prepare(const GridMap&, const Car&, const HybridAstarConfig&) {}

/**
 * @brief 准备欧几里得启发式
 * @param[in] map 栅格地图
 */
void EuclideanHeuristic::prepare(const GridMap& map,
                                 const Car&,
                                 const HybridAstarConfig&) {
    goal_ = map.goal();
}

/**
 * @brief 计算欧几里得距离估计
 * @param[in] pose 当前位姿
 * @return 到目标的欧几里得距离
 */
double EuclideanHeuristic::estimate(const CarPose& pose) const {
    const double dx = pose.x - goal_.x;
    const double dy = pose.y - goal_.y;
    return std::sqrt(dx * dx + dy * dy);
}

/** @brief 返回启发式名称。 */
std::string EuclideanHeuristic::name() const {
    return "euclidean";
}

/**
 * @brief 准备组合启发式
 *
 * 构建可视点图：提取位于障碍物边界上的网格点作为可视点，
 * 在所有相互可视的点之间建立边，运行Dijkstra计算到目标的最短路径代价。
 *
 * @param[in] map    栅格地图
 * @param[in] car    车辆模型
 * @param[in] config 规划器配置
 */
void CombinedHeuristic::prepare(const GridMap& map,
                                const Car& car,
                                const HybridAstarConfig& config) {
    if (config.debug) {
        std::cerr << "[debug] heuristic: prepare combined heuristic\n";
    }
    goal_ = map.goal();
    width_ = map.width();
    height_ = map.height();
    xy_resolution_ = std::max(1.0e-9, config.xy_resolution);
    min_turning_radius_ = car.minTurningRadius();
    reeds_shepp_sample_step_ = config.step_size;
    max_steer_ = car.maxSteer();
    obstacle_enabled_ = config.enable_obstacle_heuristic;
    debug_enabled_ = config.debug;
    timing_enabled_ = config.enable_timing;
    timing_ = HeuristicTiming{};

    // 重置可视点图数据
    obstacle_cells_.clear();
    visibility_points_.clear();
    visibility_graph_.clear();
    visibility_distance_to_goal_.clear();
    obstacle_lookup_.clear();
    visibility_goal_index_ = -1;
    obstacle_lookup_width_ = 0;
    obstacle_lookup_height_ = 0;
    obstacle_lookup_resolution_ = std::max(
        1.0e-9, config.obstacle_lookup_resolution);

    if (obstacle_enabled_) {
        if (config.debug) {
            std::cerr << "[debug] heuristic: collect obstacle cells\n";
        }
        Clock::time_point obstacle_collect_start;
        if (timing_enabled_) {
            obstacle_collect_start = Clock::now();
        }
        // 收集所有障碍物单元格
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (map.isObstacle(x, y)) {
                    obstacle_cells_.insert({x, y});
                }
            }
        }
        if (timing_enabled_) {
            timing_.obstacle_collect_ms += elapsedMs(
                obstacle_collect_start, Clock::now());
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: obstacle cells="
                      << obstacle_cells_.size() << '\n';
            std::cerr << "[debug] heuristic: extract visibility points\n";
        }

        Clock::time_point visibility_points_start;
        if (timing_enabled_) {
            visibility_points_start = Clock::now();
        }
        // 提取可视点：只有1个角点接触障碍物的网格角点
        for (int y = 0; y <= height_; ++y) {
            for (int x = 0; x <= width_; ++x) {
                if (surroundingObstacleCount(map, x, y) == 1) {
                    visibility_points_.push_back({
                        .x = static_cast<double>(x),
                        .y = static_cast<double>(y)
                    });
                }
            }
        }
        if (timing_enabled_) {
            timing_.visibility_points_ms += elapsedMs(
                visibility_points_start, Clock::now());
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: visibility points="
                      << visibility_points_.size() << '\n';
        }

        // 将目标点添加到可视点列表末尾
        visibility_goal_index_ = static_cast<int>(visibility_points_.size());
        visibility_points_.push_back(goalPoint(goal_));

        // 构建可视点之间的边
        visibility_graph_.assign(visibility_points_.size(), {});
        if (config.debug) {
            std::cerr << "[debug] heuristic: build visibility graph"
                      << " candidates=" << visibility_points_.size() << '\n';
        }
        Clock::time_point visibility_graph_start;
        if (timing_enabled_) {
            visibility_graph_start = Clock::now();
        }
        std::size_t visibility_edge_count = 0;
        for (std::size_t i = 0; i < visibility_points_.size(); ++i) {
            if (config.debug) {
                std::cerr << "[debug] heuristic: visibility graph progress "
                          << (i + 1) << "/" << visibility_points_.size()
                          << '\n';
            }
            for (std::size_t j = i + 1; j < visibility_points_.size(); ++j) {
                const Point2D from = visibility_points_[i];
                const Point2D to = visibility_points_[j];

                // 仅当两点间有直接视域时才添加边
                if (!hasLineOfSight(from, to, obstacle_cells_)) {
                    continue;
                }
                const double edge_cost = distance2d(from, to);
                visibility_graph_[i].push_back({
                    static_cast<int>(j),
                    edge_cost
                });
                visibility_graph_[j].push_back({
                    static_cast<int>(i),
                    edge_cost
                });
                ++visibility_edge_count;
            }
        }
        if (timing_enabled_) {
            timing_.visibility_graph_ms += elapsedMs(
                visibility_graph_start, Clock::now());
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: visibility graph edges="
                      << visibility_edge_count << '\n';
        }

        // 运行Dijkstra计算从目标点到所有可视点的最短距离
        if (config.debug) {
            std::cerr << "[debug] heuristic: run visibility graph Dijkstra\n";
        }
        Clock::time_point visibility_dijkstra_start;
        if (timing_enabled_) {
            visibility_dijkstra_start = Clock::now();
        }
        visibility_distance_to_goal_ =
            runGraphDijkstra(visibility_graph_, visibility_goal_index_);
        if (timing_enabled_) {
            timing_.visibility_dijkstra_ms += elapsedMs(
                visibility_dijkstra_start, Clock::now());
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: Dijkstra complete\n";
            std::cerr << "[debug] heuristic: build obstacle lookup\n";
        }
        Clock::time_point obstacle_lookup_start;
        if (timing_enabled_) {
            obstacle_lookup_start = Clock::now();
        }
        buildObstacleLookup();
        if (timing_enabled_) {
            timing_.obstacle_lookup_ms += elapsedMs(
                obstacle_lookup_start, Clock::now());
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: obstacle lookup="
                      << obstacle_lookup_width_ << "x"
                      << obstacle_lookup_height_ << '\n';
        }
    } else if (config.debug) {
        std::cerr << "[debug] heuristic: obstacle heuristic disabled\n";
    }
}

/**
 * @brief 计算组合启发式估计
 * @param[in] pose 当前位姿
 * @return max(非障碍物估计, 障碍物估计)
 */
double CombinedHeuristic::estimate(const CarPose& pose) const {
    if (timing_enabled_) {
        ++timing_.heuristic_estimate_calls;
    }

    double non_obs = 0.0;
    if (timing_enabled_) {
        const auto non_obstacle_start = Clock::now();
        non_obs = nonObstacleEstimate(pose);
        timing_.non_obstacle_heuristic_ms += elapsedMs(
            non_obstacle_start, Clock::now());
    } else {
        non_obs = nonObstacleEstimate(pose);
    }

    double obs = 0.0;
    if (obstacle_enabled_) {
        if (timing_enabled_) {
            const auto obstacle_start = Clock::now();
            obs = obstacleEstimate(pose);
            timing_.obstacle_heuristic_ms += elapsedMs(
                obstacle_start, Clock::now());
        } else {
            obs = obstacleEstimate(pose);
        }
    }

    const double diff = non_obs - obs;
    HeuristicDiffLogStats& stats = heuristicDiffLogStats();
    if (diff > 0.0) {
        ++stats.positive;
    } else if (diff < 0.0) {
        ++stats.negative;
    } else {
        ++stats.zero;
    }
    return std::max(non_obs, obs);
}

/** @brief 返回启发式名称。 */
std::string CombinedHeuristic::name() const {
    return "combined";
}

const HeuristicTiming& CombinedHeuristic::timing() const {
    return timing_;
}

/**
 * @brief 欧几里得距离辅助函数
 */
double CombinedHeuristic::euclidean(const CarPose& pose) const {
    const double dx = pose.x - goal_.x;
    const double dy = pose.y - goal_.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 使用可视点图计算单个二维点的障碍物启发式。
 */
double CombinedHeuristic::obstacleEstimateAt(Point2D current) const {
    if (visibility_points_.empty() || visibility_distance_to_goal_.empty()) {
        const double dx = current.x - goal_.x;
        const double dy = current.y - goal_.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    const Point2D goal = goalPoint(goal_);
    const double dx = current.x - goal.x;
    const double dy = current.y - goal.y;
    const double direct_distance = std::sqrt(dx * dx + dy * dy);

    if (hasLineOfSight(current, goal, obstacle_cells_)) {
        return direct_distance;
    }

    double best = kInfinity;
    for (std::size_t i = 0; i < visibility_points_.size(); ++i) {
        if (static_cast<int>(i) == visibility_goal_index_) {
            continue;
        }
        const double suffix =
            visibility_distance_to_goal_[static_cast<std::size_t>(i)];
        if (!std::isfinite(suffix)) {
            continue;
        }
        const Point2D waypoint = visibility_points_[i];
        if (!hasLineOfSight(current, waypoint, obstacle_cells_)) {
            continue;
        }
        best = std::min(best, distance2d(current, waypoint) + suffix);
    }

    return std::isfinite(best) ? best : direct_distance;
}

void CombinedHeuristic::buildObstacleLookup() {
    obstacle_lookup_width_ = std::max(
        2, static_cast<int>(std::ceil(static_cast<double>(width_)
                                      / obstacle_lookup_resolution_)) + 1);
    obstacle_lookup_height_ = std::max(
        2, static_cast<int>(std::ceil(static_cast<double>(height_)
                                      / obstacle_lookup_resolution_)) + 1);
    obstacle_lookup_.assign(
        static_cast<std::size_t>(obstacle_lookup_width_ * obstacle_lookup_height_),
        0.0);

    for (int y = 0; y < obstacle_lookup_height_; ++y) {
        for (int x = 0; x < obstacle_lookup_width_; ++x) {
            const Point2D current{
                .x = static_cast<double>(x) * obstacle_lookup_resolution_,
                .y = static_cast<double>(y) * obstacle_lookup_resolution_
            };
            obstacle_lookup_[static_cast<std::size_t>(
                y * obstacle_lookup_width_ + x)] = obstacleEstimateAt(current);
        }
    }
}

/**
 * @brief 使用预计算查表结果获取障碍物启发式
 * @param[in] pose 当前位姿
 * @return 到目标的最短可视路径代价
 */
double CombinedHeuristic::obstacleEstimate(const CarPose& pose) const {
    if (obstacle_lookup_.empty()
        || obstacle_lookup_width_ <= 0
        || obstacle_lookup_height_ <= 0) {
        return euclidean(pose);
    }

    const int x0 = std::clamp(
        static_cast<int>(std::floor(pose.x / obstacle_lookup_resolution_)),
        0,
        obstacle_lookup_width_ - 1);
    const int y0 = std::clamp(
        static_cast<int>(std::floor(pose.y / obstacle_lookup_resolution_)),
        0,
        obstacle_lookup_height_ - 1);
    const int x1 = std::min(x0 + 1, obstacle_lookup_width_ - 1);
    const int y1 = std::min(y0 + 1, obstacle_lookup_height_ - 1);

    const double v00 = obstacle_lookup_[static_cast<std::size_t>(
        y0 * obstacle_lookup_width_ + x0)];
    const double v10 = obstacle_lookup_[static_cast<std::size_t>(
        y0 * obstacle_lookup_width_ + x1)];
    const double v01 = obstacle_lookup_[static_cast<std::size_t>(
        y1 * obstacle_lookup_width_ + x0)];
    const double v11 = obstacle_lookup_[static_cast<std::size_t>(
        y1 * obstacle_lookup_width_ + x1)];
    return std::min(std::min(v00, v10), std::min(v01, v11));
}

/**
 * @brief 非障碍物启发式：使用Reeds-Shepp距离
 * @param[in] pose 当前位姿
 * @return Reeds-Shepp风格的无障碍最短距离估计
 */
double CombinedHeuristic::nonObstacleEstimate(const CarPose& pose) const {
    const ReedsSheppGenerator generator(
        min_turning_radius_, reeds_shepp_sample_step_, max_steer_);
    if (const std::optional<double> distance =
            generator.estimateDistance(pose, goal_)) {
        return *distance;
    }
    return euclidean(pose);
}
