/**
 * @file CollisionChecker.hpp
 * @brief 碰撞检测器定义
 *
 * 提供车辆矩形footprint和Reeds-Shepp采样路径的碰撞检测功能，
 * 用于验证规划路径的可行性。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"
#include "ReedsShepp.hpp"

#include <cstddef>
#include <optional>
#include <vector>

/**
 * @brief 车辆矩形 footprint 碰撞检测参数。
 */
struct VehicleCollisionConfig {
    double safety_margin = 0.0;                 ///< 车身长宽额外外扩距离
    bool treat_out_of_bounds_as_collision = true; ///< 地图外是否视为碰撞
};

/**
 * @brief 连续车辆位姿的矩形 footprint 碰撞检测器。
 */
class VehicleCollisionChecker {
public:
    VehicleCollisionChecker(const GridMap& map,
                            const Car& car,
                            VehicleCollisionConfig config = {});

    [[nodiscard]] bool collides(const CarPose& pose) const;
    [[nodiscard]] bool isCollisionFree(
        const std::vector<CarPose>& samples) const;
    [[nodiscard]] std::optional<std::size_t> firstCollisionIndex(
        const std::vector<CarPose>& samples) const;

private:
    const GridMap& map_;
    const Car& car_;
    VehicleCollisionConfig config_;
};

/**
 * @brief Reeds-Shepp 路径碰撞检测参数。
 */
struct ReedsSheppCollisionConfig {
    VehicleCollisionConfig vehicle;
    bool require_non_empty_samples = true; ///< samples 为空时是否判为碰撞
};

/**
 * @brief Reeds-Shepp 采样路径碰撞检测器。
 *
 * ReedsSheppPath::samples 应当已经按足够小的 sample_step 采样；
 * 该检测器逐个采样位姿检查车辆矩形 footprint。
 */
class ReedsSheppCollisionChecker {
public:
    ReedsSheppCollisionChecker(const GridMap& map,
                               const Car& car,
                               ReedsSheppCollisionConfig config = {});

    [[nodiscard]] bool isCollisionFree(const ReedsSheppPath& path) const;
    [[nodiscard]] std::optional<std::size_t> firstCollisionSample(
        const ReedsSheppPath& path) const;
    [[nodiscard]] bool collides(const CarPose& pose) const;

private:
    VehicleCollisionChecker vehicle_checker_;
    ReedsSheppCollisionConfig config_;
};
