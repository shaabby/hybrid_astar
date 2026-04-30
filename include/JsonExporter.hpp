#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <string>
#include <vector>

/**
 * @brief JSON 数据导出器。
 *
 * 将规划结果序列化为 JSON 字符串，供浏览器端可视化使用。
 */
class JsonExporter {
public:
    /**
     * @brief 导出完整规划结果（地图 + 车辆 + 路径 + 扩展节点）。
     * @param[in] map       栅格地图
     * @param[in] car       车辆模型
     * @param[in] path      路径点序列
     * @param[in] expanded  扩展过的搜索节点，用于可视化搜索过程
     * @return 格式化后的 JSON 字符串
     */
    [[nodiscard]] static std::string exportPath(
        const GridMap& map,
        const Car& car,
        const std::vector<CarPose>& path,
        const std::vector<CarPose>& expanded = {});

    /**
     * @brief 单独导出地图数据为 JSON 对象字符串。
     * @param[in] map 栅格地图
     * @return JSON 对象字符串
     */
    [[nodiscard]] static std::string exportMap(const GridMap& map);

    /**
     * @brief 单独导出车辆配置为 JSON 对象字符串。
     * @param[in] vehicle 车辆物理参数
     * @return JSON 对象字符串
     */
    [[nodiscard]] static std::string exportVehicle(const VehicleConfig& vehicle);

    /**
     * @brief 单独导出路径点为 JSON 数组字符串。
     * @param[in] path 路径点序列
     * @return JSON 数组字符串
     */
    [[nodiscard]] static std::string exportPathPoints(
        const std::vector<CarPose>& path);
};
