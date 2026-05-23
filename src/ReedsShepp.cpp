/**
 * @file ReedsShepp.cpp
 * @brief Reeds-Shepp路径生成器实现
 *
 * 使用OMPL库计算车辆最短路径，支持前进和倒车操作。
 * 路径被采样为离散的位姿序列供碰撞检测使用。
 */

#include "ReedsShepp.hpp"

#include "OmplReedsShepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

/// @brief 圆周率常量
constexpr double kPi = 3.14159265358979323846;
/// @brief 2π常量
constexpr double kTwoPi = 2.0 * kPi;
/// @brief 浮点数比较epsilon
constexpr double kEpsilon = 1.0e-9;
/// @brief 位置对齐容差
constexpr double kSnapPositionTolerance = 1.0e-4;
/// @brief 角度对齐容差
constexpr double kSnapThetaTolerance = 1.0e-4;

/**
 * @brief 将角度归一化到(-π, π]区间
 * @param[in] angle 输入角度
 * @return 归一化后的角度
 */
double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

/**
 * @brief 计算二维欧几里得距离
 */
double distance2d(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief 检查路径是否到达目标
 * @param[in] pose              当前位姿
 * @param[in] goal              目标位姿
 * @param[in] position_tolerance 位置容差
 * @param[in] theta_tolerance    角度容差
 */
bool reachesGoal(const CarPose& pose,
                 const CarPose& goal,
                 double position_tolerance,
                 double theta_tolerance) {
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= position_tolerance
        && std::abs(normalizeAngle(pose.theta - goal.theta)) <= theta_tolerance;
}

/**
 * @brief 检查是否可以通过微调到达目标
 * @param[in] pose  当前位姿
 * @param[in] goal 目标位姿
 * @return true如果位置和角度都非常接近
 */
bool canSnapToGoal(const CarPose& pose, const CarPose& goal) {
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= kSnapPositionTolerance
        && std::abs(normalizeAngle(pose.theta - goal.theta))
            <= kSnapThetaTolerance;
}

/**
 * @brief 将车辆位姿转换为归一化状态（用于OMPL）
 * @param[in] pose   车辆位姿
 * @param[in] radius 转弯半径
 * @return 归一化状态
 */
ompl_rs::State toNormalizedState(const CarPose& pose, double radius) {
    return {
        pose.x / radius,
        pose.y / radius,
        normalizeAngle(pose.theta)
    };
}

/**
 * @brief 将归一化状态转换为车辆位姿
 * @param[in] state    归一化状态
 * @param[in] radius   转弯半径
 * @param[in] steer    转向角
 * @param[in] direction 行驶方向
 * @return 车辆位姿
 */
CarPose toCarPose(const ompl_rs::State& state,
                  double radius,
                  double steer,
                  int direction) {
    return {
        .x = state.x * radius,
        .y = state.y * radius,
        .theta = normalizeAngle(state.yaw),
        .steer = steer,
        .direction = direction
    };
}

/**
 * @brief 转换路径段类型
 * @param[in] type OMPL路径段类型
 * @return 对应的ReedsShepp路径段类型
 */
ReedsSheppSegmentType convertType(ompl_rs::SegmentType type) {
    switch (type) {
    case ompl_rs::SegmentType::Left:
        return ReedsSheppSegmentType::Left;
    case ompl_rs::SegmentType::Right:
        return ReedsSheppSegmentType::Right;
    case ompl_rs::SegmentType::Straight:
    case ompl_rs::SegmentType::Nop:
        return ReedsSheppSegmentType::Straight;
    }
    return ReedsSheppSegmentType::Straight;
}

/**
 * @brief 根据路径段类型获取转向角
 * @param[in] type      路径段类型
 * @param[in] max_steer 最大转向角
 * @return 转向角值
 */
double steerFor(ompl_rs::SegmentType type, double max_steer) {
    switch (type) {
    case ompl_rs::SegmentType::Left:
        return max_steer;
    case ompl_rs::SegmentType::Right:
        return -max_steer;
    case ompl_rs::SegmentType::Straight:
    case ompl_rs::SegmentType::Nop:
        return 0.0;
    }
    return 0.0;
}

/**
 * @brief 路径段控制信息
 */
struct SegmentControl {
    ompl_rs::SegmentType type = ompl_rs::SegmentType::Straight; ///< 段类型
    int direction = 1;                                          ///< 行驶方向
};

/**
 * @brief 获取指定距离处的路径控制信息
 * @param[in] path    OMPL路径
 * @param[in] distance 距离
 * @return 控制信息
 */
SegmentControl controlAtDistance(const ompl_rs::Path& path, double distance) {
    double travelled = 0.0;
    SegmentControl last;

    for (std::size_t i = 0; i < path.length.size(); ++i) {
        if (path.type[i] == ompl_rs::SegmentType::Nop
            || std::abs(path.length[i]) <= kEpsilon) {
            continue;
        }

        last.type = path.type[i];
        last.direction = path.length[i] < 0.0 ? -1 : 1;

        travelled += std::abs(path.length[i]);
        if (distance <= travelled + kEpsilon) {
            return last;
        }
    }

    return last;
}

} // namespace

/**
 * @brief 构造Reeds-Shepp路径生成器
 */
ReedsSheppGenerator::ReedsSheppGenerator(double min_turning_radius,
                                         double sample_step,
                                         double max_steer,
                                         double goal_position_tolerance,
                                         double goal_theta_tolerance)
    : min_turning_radius_(min_turning_radius),
      sample_step_(sample_step),
      max_steer_(max_steer),
      goal_position_tolerance_(goal_position_tolerance),
      goal_theta_tolerance_(goal_theta_tolerance) {}

/**
 * @brief 生成到二维目标点的路径
 */
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

/**
 * @brief 生成到完整目标位姿的路径
 */
std::optional<ReedsSheppPath> ReedsSheppGenerator::generate(
    const CarPose& start,
    const CarPose& goal) const {
    if (min_turning_radius_ <= kEpsilon || sample_step_ <= kEpsilon
        || !std::isfinite(min_turning_radius_)) {
        return std::nullopt;
    }

    // 归一化坐标后调用OMPL
    const ompl_rs::State normalized_start =
        toNormalizedState(start, min_turning_radius_);
    const ompl_rs::State normalized_goal =
        toNormalizedState(goal, min_turning_radius_);
    const ompl_rs::Path vendor_path =
        ompl_rs::shortestPath(normalized_start, normalized_goal);

    if (!std::isfinite(vendor_path.total_length)
        || vendor_path.total_length <= kEpsilon) {
        return std::nullopt;
    }

    // 构建结果
    ReedsSheppPath result;
    result.total_length = vendor_path.total_length * min_turning_radius_;
    result.word = "OMPL_RS_" + std::to_string(vendor_path.type_index);

    // 转换路径段
    for (std::size_t i = 0; i < vendor_path.length.size(); ++i) {
        if (vendor_path.type[i] == ompl_rs::SegmentType::Nop
            || std::abs(vendor_path.length[i]) <= kEpsilon) {
            continue;
        }
        result.segments.push_back({
            .type = convertType(vendor_path.type[i]),
            .length = vendor_path.length[i] * min_turning_radius_,
            .direction = vendor_path.length[i] < 0.0
                ? ReedsSheppDirection::Backward
                : ReedsSheppDirection::Forward
        });
    }

    // 采样路径点
    const double normalized_step = sample_step_ / min_turning_radius_;
    for (double distance = normalized_step;
         distance < vendor_path.total_length - kEpsilon;
         distance += normalized_step) {
        const SegmentControl control = controlAtDistance(vendor_path, distance);
        const ompl_rs::State sampled =
            ompl_rs::interpolate(normalized_start, vendor_path, distance);
        result.samples.push_back(toCarPose(
            sampled,
            min_turning_radius_,
            steerFor(control.type, max_steer_),
            control.direction));
    }

    // 添加终点
    const SegmentControl final_control =
        controlAtDistance(vendor_path, vendor_path.total_length);
    const ompl_rs::State final_state =
        ompl_rs::interpolate(normalized_start, vendor_path, vendor_path.total_length);
    CarPose final_pose = toCarPose(
        final_state,
        min_turning_radius_,
        steerFor(final_control.type, max_steer_),
        final_control.direction);

    // 对齐到目标
    if (canSnapToGoal(final_pose, goal)) {
        final_pose.x = goal.x;
        final_pose.y = goal.y;
        final_pose.theta = normalizeAngle(goal.theta);
    }

    result.samples.push_back(final_pose);

    // 验证到达目标
    if (!reachesGoal(result.samples.back(), goal, goal_position_tolerance_,
                    goal_theta_tolerance_)) {
        return std::nullopt;
    }

    return result;
}

/**
 * @brief 估算到二维目标点的距离
 */
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

/**
 * @brief 估算到完整目标位姿的距离
 */
std::optional<double> ReedsSheppGenerator::estimateDistance(
    const CarPose& start,
    const CarPose& goal) const {
    if (min_turning_radius_ <= kEpsilon || !std::isfinite(min_turning_radius_)) {
        return std::nullopt;
    }

    const ompl_rs::State normalized_start =
        toNormalizedState(start, min_turning_radius_);
    const ompl_rs::State normalized_goal =
        toNormalizedState(goal, min_turning_radius_);
    const double distance =
        ompl_rs::distance(normalized_start, normalized_goal)
        * min_turning_radius_;

    if (!std::isfinite(distance)) {
        return std::nullopt;
    }
    return distance;
}