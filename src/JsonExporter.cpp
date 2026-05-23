/**
 * @file JsonExporter.cpp
 * @brief JSON数据导出器实现
 *
 * 将规划结果序列化为JSON字符串，供浏览器端可视化使用。
 */

#include "JsonExporter.hpp"

#include <iomanip>
#include <sstream>
#include <string>

/**
 * @brief 导出完整规划结果
 * @param[in] map      栅格地图
 * @param[in] car      车辆模型
 * @param[in] path     路径点序列
 * @param[in] expanded 搜索扩展节点
 * @return JSON字符串
 */
std::string JsonExporter::exportPath(const GridMap& map,
                                      const Car& car,
                                      const std::vector<CarPose>& path,
                                      const std::vector<CarPose>& expanded) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"map\": " << exportMap(map) << ",\n";
    out << "  \"vehicle\": " << exportVehicle(car.config()) << ",\n";
    out << "  \"path\": " << exportPathPoints(path) << ",\n";
    out << "  \"expanded\": " << exportPathPoints(expanded) << "\n";
    out << "}\n";
    return out.str();
}

/**
 * @brief 导出地图数据
 * @param[in] map 栅格地图
 * @return JSON对象字符串
 */
std::string JsonExporter::exportMap(const GridMap& map) {
    const Pose2D& start = map.start();
    const Pose2D& goal = map.goal();

    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "    \"width\": " << map.width() << ",\n";
    out << "    \"height\": " << map.height() << ",\n";
    out << "    \"start\": {\"x\": " << start.x
        << ", \"y\": " << start.y
        << ", \"theta\": " << start.theta << "},\n";
    out << "    \"goal\": {\"x\": " << goal.x
        << ", \"y\": " << goal.y
        << ", \"theta\": " << goal.theta << "},\n";
    out << "    \"obstacles\": [\n";

    bool first = true;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (!map.isObstacle(x, y)) {
                continue;
            }
            if (!first) {
                out << ",\n";
            }
            first = false;
            out << "      {\"x\": " << x << ", \"y\": " << y << "}";
        }
    }
    out << "\n    ]\n";
    out << "  }";
    return out.str();
}

/**
 * @brief 导出车辆配置
 * @param[in] vehicle 车辆物理参数
 * @return JSON对象字符串
 */
std::string JsonExporter::exportVehicle(const VehicleConfig& vehicle) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "    \"length\": " << vehicle.length << ",\n";
    out << "    \"width\": " << vehicle.width << ",\n";
    out << "    \"wheelbase\": " << vehicle.wheelbase << ",\n";
    out << "    \"rearToCenter\": " << vehicle.rear_to_center << ",\n";
    out << "    \"maxSteer\": " << vehicle.max_steer << "\n";
    out << "  }";
    return out.str();
}

/**
 * @brief 导出路径点序列
 * @param[in] path 路径点序列
 * @return JSON数组字符串
 */
std::string JsonExporter::exportPathPoints(
    const std::vector<CarPose>& path) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "[\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        const CarPose& pose = path[i];
        out << "    {\"x\": " << pose.x
            << ", \"y\": " << pose.y
            << ", \"theta\": " << pose.theta
            << ", \"steer\": " << pose.steer
            << ", \"direction\": " << pose.direction << "}";
        if (i + 1 < path.size()) {
            out << ",";
        }
        out << "\n";
    }
    out << "  ]";
    return out.str();
}