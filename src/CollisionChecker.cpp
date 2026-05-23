/**
 * @file CollisionChecker.cpp
 * @brief 车辆碰撞检测器实现
 *
 * 实现基于分离轴定理（SAT）的矩形重叠检测，
 * 以及车辆矩形footprint与栅格地图障碍物的碰撞检测。
 */

#include "CollisionChecker.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

using Point2 = std::array<double, 2>;
using Quad = std::array<Point2, 4>;

/**
 * @brief 计算两个向量的点积
 * @param[in] lhs 左向量
 * @param[in] rhs 右向量
 * @return 点积结果
 */
double dot(const Point2& lhs, const Point2& rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1];
}

/**
 * @brief 将多边形投影到给定轴上
 * @param[in]  polygon 要投影的多边形
 * @param[in]  axis    投影轴（单位向量）
 * @param[out] min     投影最小值
 * @param[out] max     投影最大值
 */
void project(const Quad& polygon, const Point2& axis, double& min, double& max) {
    min = dot(polygon[0], axis);
    max = min;
    for (std::size_t i = 1; i < polygon.size(); ++i) {
        const double value = dot(polygon[i], axis);
        min = std::min(min, value);
        max = std::max(max, value);
    }
}

/**
 * @brief 检查两个多边形在给定轴上是否重叠
 * @param[in] lhs   第一个多边形
 * @param[in] rhs   第二个多边形
 * @param[in] axis  分离轴
 * @return true如果重叠，false如果分离
 */
bool overlapsOnAxis(const Quad& lhs, const Quad& rhs, const Point2& axis) {
    constexpr double kOverlapEpsilon = 1.0e-9;
    double lhs_min = 0.0;
    double lhs_max = 0.0;
    double rhs_min = 0.0;
    double rhs_max = 0.0;
    project(lhs, axis, lhs_min, lhs_max);
    project(rhs, axis, rhs_min, rhs_max);
    return lhs_max >= rhs_min - kOverlapEpsilon
        && rhs_max >= lhs_min - kOverlapEpsilon;
}

/**
 * @brief 使用分离轴定理检查两个四边形是否重叠
 * @param[in] lhs 第一个矩形
 * @param[in] rhs 第二个矩形
 * @return true如果重叠，false如果分离
 */
bool rectanglesOverlap(const Quad& lhs, const Quad& rhs) {
    // 四个分离轴：两条边向量 + 标准x和y轴
    const std::array<Point2, 4> axes = {{
        {lhs[1][0] - lhs[0][0], lhs[1][1] - lhs[0][1]},
        {lhs[2][0] - lhs[1][0], lhs[2][1] - lhs[1][1]},
        {1.0, 0.0},
        {0.0, 1.0}
    }};

    for (const Point2& axis : axes) {
        if (!overlapsOnAxis(lhs, rhs, axis)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 获取栅格单元的四个角点
 * @param[in] x 栅格x索引
 * @param[in] y 栅格y索引
 * @return 四个角点坐标（左下、右下、右上、左上）
 */
Quad gridCellCorners(int x, int y) {
    const double left = static_cast<double>(x);
    const double bottom = static_cast<double>(y);
    const double right = left + 1.0;
    const double top = bottom + 1.0;
    return {{
        {left, bottom},
        {right, bottom},
        {right, top},
        {left, top}
    }};
}

/**
 * @brief 计算车辆在给定姿态下的世界坐标角点
 * @param[in] car           车辆模型
 * @param[in] pose          车辆位姿
 * @param[in] safety_margin 安全外扩距离
 * @return 四个角点的世界坐标
 */
Quad vehicleCorners(const Car& car,
                    const CarPose& pose,
                    double safety_margin) {
    const VehicleConfig& vehicle = car.config();
    const double margin = std::max(0.0, safety_margin);
    const double front = vehicle.length - vehicle.rear_to_center + margin;
    const double rear = -vehicle.rear_to_center - margin;
    const double half_width = vehicle.width * 0.5 + margin;

    // 车辆局部坐标系下的角点
    const Quad local = {{
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    }};

    // 旋转变换到世界坐标系
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    Quad world{};

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

} // namespace

/**
 * @brief 构造车辆碰撞检测器
 * @param[in] map    栅格地图
 * @param[in] car    车辆模型
 * @param[in] config 碰撞检测配置
 */
VehicleCollisionChecker::VehicleCollisionChecker(
    const GridMap& map,
    const Car& car,
    VehicleCollisionConfig config)
    : map_(map),
      car_(car),
      config_(config) {}

/**
 * @brief 检查单个位姿是否与障碍物碰撞
 * @param[in] pose 车辆位姿
 * @return true如果碰撞，false如果自由
 *
 * 计算车辆角点包围盒，对包围盒内的所有栅格
 * 使用SAT算法检测矩形重叠。
 */
bool VehicleCollisionChecker::collides(const CarPose& pose) const {
    const Quad corners = vehicleCorners(car_, pose, config_.safety_margin);

    // 计算车辆角点包围盒
    double min_x = corners[0][0];
    double max_x = corners[0][0];
    double min_y = corners[0][1];
    double max_y = corners[0][1];
    for (const Point2& corner : corners) {
        min_x = std::min(min_x, corner[0]);
        max_x = std::max(max_x, corner[0]);
        min_y = std::min(min_y, corner[1]);
        max_y = std::max(max_y, corner[1]);
    }

    // 遍历包围盒内的所有栅格
    const int x0 = static_cast<int>(std::floor(min_x));
    const int x1 = static_cast<int>(std::floor(max_x));
    const int y0 = static_cast<int>(std::floor(min_y));
    const int y1 = static_cast<int>(std::floor(max_y));

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const Quad cell = gridCellCorners(x, y);
            if (!rectanglesOverlap(corners, cell)) {
                continue;
            }

            // 地图外区域处理
            if (!map_.inBounds(x, y)) {
                if (config_.treat_out_of_bounds_as_collision) {
                    return true;
                }
                continue;
            }

            // 障碍物检测
            if (map_.isObstacle(x, y)) {
                return true;
            }
        }
    }

    return false;
}

/**
 * @brief 检查一系列位姿是否全部无碰撞
 * @param[in] samples 位姿序列
 * @return true如果全部自由，false如果有碰撞
 */
bool VehicleCollisionChecker::isCollisionFree(
    const std::vector<CarPose>& samples) const {
    return !firstCollisionIndex(samples).has_value();
}

/**
 * @brief 返回序列中第一个碰撞位姿的索引
 * @param[in] samples 位姿序列
 * @return 第一个碰撞的索引，无碰撞则返回nullopt
 */
std::optional<std::size_t> VehicleCollisionChecker::firstCollisionIndex(
    const std::vector<CarPose>& samples) const {
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (collides(samples[i])) {
            return i;
        }
    }
    return std::nullopt;
}

/**
 * @brief 构造Reeds-Shepp路径碰撞检测器
 * @param[in] map    栅格地图
 * @param[in] car    车辆模型
 * @param[in] config 碰撞检测配置
 */
ReedsSheppCollisionChecker::ReedsSheppCollisionChecker(
    const GridMap& map,
    const Car& car,
    ReedsSheppCollisionConfig config)
    : vehicle_checker_(map, car, config.vehicle),
      config_(config) {}

/**
 * @brief 检查Reeds-Shepp路径是否无碰撞
 * @param[in] path Reeds-Shepp路径
 * @return true如果自由，false如果碰撞
 */
bool ReedsSheppCollisionChecker::isCollisionFree(
    const ReedsSheppPath& path) const {
    return !firstCollisionSample(path).has_value();
}

/**
 * @brief 返回路径中第一个碰撞采样点的索引
 * @param[in] path Reeds-Shepp路径
 * @return 第一个碰撞的采样点索引，无碰撞则返回nullopt
 */
std::optional<std::size_t> ReedsSheppCollisionChecker::firstCollisionSample(
    const ReedsSheppPath& path) const {
    if (path.samples.empty()) {
        if (config_.require_non_empty_samples) {
            return 0;
        }
        return std::nullopt;
    }
    return vehicle_checker_.firstCollisionIndex(path.samples);
}

/**
 * @brief 检查单个位姿是否碰撞
 * @param[in] pose 车辆位姿
 * @return true如果碰撞，false如果自由
 */
bool ReedsSheppCollisionChecker::collides(const CarPose& pose) const {
    return vehicle_checker_.collides(pose);
}