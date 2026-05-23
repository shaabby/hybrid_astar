/**
 * @file ReedsShepp.hpp
 * @brief Reeds-Shepp路径生成器定义
 *
 * 定义Reeds-Shepp路径段类型、路径结构和生成器接口。
 * Reeds-Shepp路径是一种考虑车辆倒车能力的最短路径。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <optional>
#include <string>
#include <vector>

/**
 * @brief Reeds-Shepp路径段类型
 */
enum class ReedsSheppSegmentType {
    Left,    ///< 左转圆弧
    Straight, ///< 直行
    Right   ///< 右转圆弧
};

/**
 * @brief 行驶方向
 */
enum class ReedsSheppDirection {
    Forward,  ///< 前进
    Backward ///< 倒车
};

/**
 * @brief 单个路径段
 */
struct ReedsSheppSegment {
    ReedsSheppSegmentType type = ReedsSheppSegmentType::Straight; ///< 段类型
    double length = 0.0;                                        ///< 段长度
    ReedsSheppDirection direction = ReedsSheppDirection::Forward; ///< 行驶方向
};

/**
 * @brief Reeds-Shepp完整路径
 */
struct ReedsSheppPath {
    std::vector<ReedsSheppSegment> segments; ///< 路径段列表
    std::vector<CarPose> samples;            ///< 路径采样点序列
    double total_length = 0.0;               ///< 总路径长度
    std::string word;                       ///< OMPL路径类型标识
};

/**
 * @brief Reeds-Shepp路径生成器
 *
 * 使用OMPL库计算车辆最短路径，支持前进和倒车。
 */
class ReedsSheppGenerator {
public:
    /**
     * @brief 构造路径生成器
     * @param[in] min_turning_radius     最小转弯半径
     * @param[in] sample_step            采样步长
     * @param[in] max_steer              最大前轮转向角
     * @param[in] goal_position_tolerance 目标位置容差
     * @param[in] goal_theta_tolerance    目标航向角容差
     */
    ReedsSheppGenerator(double min_turning_radius,
                        double sample_step,
                        double max_steer = 0.0,
                        double goal_position_tolerance = 1.0e-4,
                        double goal_theta_tolerance = 1.0e-4);

    /**
     * @brief 生成从起点到二维目标点的路径
     * @param[in] start 起始车辆位姿
     * @param[in] goal  目标2D位姿（不含航向角详情）
     * @return 生成的路径，失败时返回nullopt
     */
    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const Pose2D& goal) const;

    /**
     * @brief 生成从起点到完整目标位姿的路径
     * @param[in] start 起始车辆位姿
     * @param[in] goal  目标车辆位姿
     * @return 生成的路径，失败时返回nullopt
     */
    [[nodiscard]] std::optional<ReedsSheppPath> generate(
        const CarPose& start,
        const CarPose& goal) const;

    /**
     * @brief 估算起点到二维目标点的最短距离
     * @param[in] start 起始车辆位姿
     * @param[in] goal  目标2D位姿
     * @return 估算距离，失败时返回nullopt
     */
    [[nodiscard]] std::optional<double> estimateDistance(
        const CarPose& start,
        const Pose2D& goal) const;

    /**
     * @brief 估算起点到完整目标位姿的最短距离
     * @param[in] start 起始车辆位姿
     * @param[in] goal  目标车辆位姿
     * @return 估算距离，失败时返回nullopt
     */
    [[nodiscard]] std::optional<double> estimateDistance(
        const CarPose& start,
        const CarPose& goal) const;

private:
    double min_turning_radius_ = 1.0;         ///< 最小转弯半径
    double sample_step_ = 0.2;                ///< 采样步长
    double max_steer_ = 0.0;                 ///< 最大前轮转向角
    double goal_position_tolerance_ = 1.0e-4; ///< 目标位置容差
    double goal_theta_tolerance_ = 1.0e-4;    ///< 目标航向角容差
};