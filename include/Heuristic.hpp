/**
 * @file Heuristic.hpp
 * @brief Hybrid A*启发函数定义
 *
 * 定义启发函数接口和两种实现：欧几里得距离启发式
 * 和考虑障碍物的组合启发式，用于指导搜索方向。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"
#include "LineOfSight.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

struct HybridAstarConfig;
enum class ObstacleHeuristicType;

struct HeuristicTiming {
    double obstacle_collect_ms = 0.0;
    double visibility_points_ms = 0.0;
    double visibility_graph_ms = 0.0;
    double visibility_dijkstra_ms = 0.0;
    double obstacle_lookup_ms = 0.0;
    double reverse_dijkstra_inflation_ms = 0.0;
    double reverse_dijkstra_ms = 0.0;
    double non_obstacle_heuristic_ms = 0.0;
    double obstacle_heuristic_ms = 0.0;
    std::size_t heuristic_estimate_calls = 0;
};

/**
 * @brief Hybrid A* 启发函数接口。
 *
 * prepare 用于在规划开始前缓存地图、车辆或配置相关的数据；
 * estimate 在搜索过程中为任意车辆位姿估计到目标的剩余代价。
 */
class Heuristic {
public:
    virtual ~Heuristic() = default;

    virtual void prepare(const GridMap& map,
                         const Car& car,
                         const HybridAstarConfig& config);

    [[nodiscard]] virtual double estimate(const CarPose& pose) const = 0;
    [[nodiscard]] virtual std::string name() const = 0;
};

/**
 * @brief 欧几里得距离启发函数。
 *
 * 只估计当前位置到目标位置的直线距离，不考虑障碍物和车辆非完整约束。
 */
class EuclideanHeuristic final : public Heuristic {
public:
    void prepare(const GridMap& map,
                 const Car& car,
                 const HybridAstarConfig& config) override;

    [[nodiscard]] double estimate(const CarPose& pose) const override;
    [[nodiscard]] std::string name() const override;

private:
    Pose2D goal_;
};

/**
 * @brief 组合启发函数。
 *
 * h = max(h_non_obs, h_obs)，其中 h_non_obs 使用无障碍 Reeds-Shepp 风格距离，
 * h_obs 使用障碍边界点可视图上的点机器人最短路距离。
 */
class CombinedHeuristic final : public Heuristic {
public:
    void prepare(const GridMap& map,
                 const Car& car,
                 const HybridAstarConfig& config) override;

    [[nodiscard]] double estimate(const CarPose& pose) const override;
    [[nodiscard]] std::string name() const override;
    [[nodiscard]] const HeuristicTiming& timing() const;

private:
    [[nodiscard]] double euclidean(const CarPose& pose) const;
    [[nodiscard]] double obstacleEstimate(const CarPose& pose) const;
    [[nodiscard]] double visibilityGraphObstacleEstimate(const CarPose& pose) const;
    [[nodiscard]] double reverseDijkstraObstacleEstimate(const CarPose& pose) const;
    [[nodiscard]] double obstacleEstimateAt(Point2D current) const;
    [[nodiscard]] double nonObstacleEstimate(const CarPose& pose) const;
    void buildObstacleLookup();

    Pose2D goal_;
    int width_ = 0;
    int height_ = 0;
    double xy_resolution_ = 1.0;
    double min_turning_radius_ = 1.0;
    double reeds_shepp_sample_step_ = 0.2;
    double max_steer_ = 0.0;
    bool obstacle_enabled_ = false;
    bool debug_enabled_ = false;
    bool timing_enabled_ = false;
    mutable HeuristicTiming timing_;
    ObstacleHeuristicType obstacle_heuristic_type_;
    double obstacle_heuristic_inflation_alpha_ = 1.0;
    std::vector<double> reverse_dijkstra_distance_;
    ObstacleSet obstacle_cells_;
    std::vector<Point2D> visibility_points_;
    std::vector<std::vector<std::pair<int, double>>> visibility_graph_;
    std::vector<double> visibility_distance_to_goal_;
    std::vector<double> obstacle_lookup_;
    int visibility_goal_index_ = -1;
    int obstacle_lookup_width_ = 0;
    int obstacle_lookup_height_ = 0;
    double obstacle_lookup_resolution_ = 1.0;
};
