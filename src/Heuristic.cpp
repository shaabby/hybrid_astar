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

    // 重置可视点图数据
    obstacle_cells_.clear();
    visibility_points_.clear();
    visibility_graph_.clear();
    visibility_distance_to_goal_.clear();
    visibility_goal_index_ = -1;

    if (obstacle_enabled_) {
        if (config.debug) {
            std::cerr << "[debug] heuristic: collect obstacle cells\n";
        }
        // 收集所有障碍物单元格
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (map.isObstacle(x, y)) {
                    obstacle_cells_.insert({x, y});
                }
            }
        }
        if (config.debug) {
            std::cerr << "[debug] heuristic: obstacle cells="
                      << obstacle_cells_.size() << '\n';
            std::cerr << "[debug] heuristic: extract visibility points\n";
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
        if (config.debug) {
            std::cerr << "[debug] heuristic: visibility graph edges="
                      << visibility_edge_count << '\n';
        }

        // 运行Dijkstra计算从目标点到所有可视点的最短距离
        if (config.debug) {
            std::cerr << "[debug] heuristic: run visibility graph Dijkstra\n";
        }
        visibility_distance_to_goal_ =
            runGraphDijkstra(visibility_graph_, visibility_goal_index_);
        if (config.debug) {
            std::cerr << "[debug] heuristic: Dijkstra complete\n";
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
    const double non_obs = nonObstacleEstimate(pose);
    const double obs = obstacle_enabled_ ? obstacleEstimate(pose) : 0.0;
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

/**
 * @brief 欧几里得距离辅助函数
 */
double CombinedHeuristic::euclidean(const CarPose& pose) const {
    const double dx = pose.x - goal_.x;
    const double dy = pose.y - goal_.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 使用可视点图计算障碍物启发式
 * @param[in] pose 当前位姿
 * @return 到目标的最短可视路径代价
 *
 * 如果当前点到目标有直接视域，返回欧几里得距离。
 * 否则，通过可视点中转：找到所有与当前点有视域的可视点，
 * 选择 distance(current, waypoint) + suffix(waypoint) 最小的组合。
 */
double CombinedHeuristic::obstacleEstimate(const CarPose& pose) const {
    if (visibility_points_.empty() || visibility_distance_to_goal_.empty()) {
        return euclidean(pose);
    }

    const Point2D current{.x = pose.x, .y = pose.y};
    const Point2D goal = goalPoint(goal_);

    // 直接可视则返回欧几里得距离
    if (hasLineOfSight(current, goal, obstacle_cells_)) {
        return euclidean(pose);
    }

    // 通过可视点中转寻找最短路径
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

    return std::isfinite(best) ? best : euclidean(pose);
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
