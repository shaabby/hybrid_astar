#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <string>

struct HybridAstarConfig;

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
