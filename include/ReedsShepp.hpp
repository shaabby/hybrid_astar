#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <optional>
#include <string>
#include <vector>

/**
 * @brief Reeds-Shepp 路径段类型。
 *
 * Left/Right 表示以最小转弯半径左/右转，Straight 表示直线段。
 * 段长度使用有符号距离：正数为前进，负数为倒车。
 */
enum class ReedsSheppSegmentType {
    Left,
    Straight,
    Right
};

struct ReedsSheppSegment {
    ReedsSheppSegmentType type = ReedsSheppSegmentType::Straight;
    double length = 0.0; ///< 有符号物理长度，单位与地图坐标一致
};

struct ReedsSheppPath {
    std::vector<ReedsSheppSegment> segments;
    std::vector<CarPose> samples; ///< 采样位姿，不包含起点，包含终点
    double total_length = 0.0;
    std::string word;
};

/**
 * @brief Reeds-Shepp 风格路径生成器。
 *
 * 当前生成器提供标准 Dubins 六类曲线及其时间反向候选，统一输出为
 * Reeds-Shepp 的有符号段表示。后续可在同一接口下继续补充混合倒车
 * cusp 的完整 Reeds-Shepp words。
 */
class ReedsSheppGenerator {
public:
    ReedsSheppGenerator(double min_turning_radius,
                        double sample_step,
                        double max_steer = 0.0);

    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const Pose2D& goal) const;

    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const CarPose& goal) const;

private:
    double min_turning_radius_ = 1.0;
    double sample_step_ = 0.2;
    double max_steer_ = 0.0;
};
