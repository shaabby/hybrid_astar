/**
 * @file Car.cpp
 * @brief 车辆运动学模型实现
 *
 * 实现简化自行车模型的前向模拟，包括位姿更新、
 * 转向角限幅和车身角点计算。
 */

#include "Car.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

/// @brief 浮点数比较的epsilon容差
constexpr double kEpsilon = 1.0e-9;

} // namespace

/**
 * @brief 使用自定义配置构造车辆。
 * @param[in] config 车辆物理参数
 */
Car::Car(VehicleConfig config)
    : config_(config) {}

/** @brief 返回车辆长度。 */
double Car::length() const {
    return config_.length;
}

/** @brief 返回车辆宽度。 */
double Car::width() const {
    return config_.width;
}

/** @brief 返回车辆轴距。 */
double Car::wheelbase() const {
    return config_.wheelbase;
}

/** @brief 返回后轴到车身中心的距离。 */
double Car::rearToCenter() const {
    return config_.rear_to_center;
}

/** @brief 返回最大前轮转向角。 */
double Car::maxSteer() const {
    return config_.max_steer;
}

/**
 * @brief 计算最小转弯半径
 * @return 由轴距和最大转向角推导的最小转弯半径
 *
 * 根据自行车模型公式：R = L / tan(δ_max)
 */
double Car::minTurningRadius() const {
    const double tan_steer = std::tan(config_.max_steer);
    if (std::abs(tan_steer) < kEpsilon) {
        return std::numeric_limits<double>::infinity();
    }
    return config_.wheelbase / std::abs(tan_steer);
}

/** @brief 返回车辆配置副本。 */
const VehicleConfig& Car::config() const {
    return config_;
}

/**
 * @brief 将转向角限幅到允许范围
 * @param[in] steer 原始转向角（弧度）
 * @return 限幅后的转向角
 */
double Car::clampSteer(double steer) const {
    return std::clamp(steer, -config_.max_steer, config_.max_steer);
}

/**
 * @brief 沿给定控制量前进一步
 * @param[in] pose      当前位姿
 * @param[in] steer     前轮转向角（弧度）
 * @param[in] direction 行驶方向，1=前进，-1=倒车
 * @param[in] distance  行驶距离
 * @return 下一时刻的位姿
 *
 * 使用自行车模型进行运动学积分：
 * - 当转向角为0时，车辆直线行驶
 * - 当转向角非0时，车辆沿圆弧运动
 */
CarPose Car::step(const CarPose& pose,
                  double steer,
                  int direction,
                  double distance) const {
    const int sign = direction < 0 ? -1 : 1;
    const double clamped_steer = clampSteer(steer);
    const double signed_distance = static_cast<double>(sign) * distance;

    CarPose next = pose;
    const double tan_steer = std::tan(clamped_steer);

    // 直线行驶或原地转向
    if (std::abs(config_.wheelbase) <= kEpsilon
        || std::abs(tan_steer) <= kEpsilon) {
        next.x += signed_distance * std::cos(pose.theta);
        next.y += signed_distance * std::sin(pose.theta);
    } else {
        // 沿圆弧运动
        const double radius = config_.wheelbase / tan_steer;
        const double next_theta = pose.theta + signed_distance / radius;

        next.x += radius * (std::sin(next_theta) - std::sin(pose.theta));
        next.y += radius * (std::cos(pose.theta) - std::cos(next_theta));
        next.theta = next_theta;
    }

    next.steer = clamped_steer;
    next.direction = sign;
    return next;
}

/**
 * @brief 计算车身四个角点的世界坐标
 * @param[in] pose 车辆位姿
 * @return 4个角点的世界坐标数组，按顺时针顺序：前左→前右→后右→后左
 *
 * 角点从车辆局部坐标系（前左角为正x、正y）变换到世界坐标系。
 */
std::array<std::array<double, 2>, 4> Car::bodyCorners(const CarPose& pose) const {
    const double front = config_.length - config_.rear_to_center;
    const double rear = -config_.rear_to_center;
    const double half_width = config_.width * 0.5;

    // 车辆局部坐标系下的四个角点
    const std::array<std::array<double, 2>, 4> local = {{
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    }};

    // 旋转变换到世界坐标系
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    std::array<std::array<double, 2>, 4> world{};

    for (std::size_t i = 0; i < local.size(); ++i) {
        const double lx = local[i][0];
        const double ly = local[i][1];
        world[i] = {
            pose.x + lx * c - ly * s,
            pose.y + lx * s + ly * c
        };
    }

    return world;
}
