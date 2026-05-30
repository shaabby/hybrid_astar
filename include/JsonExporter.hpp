#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <string>
#include <vector>

struct SearchTreeEdge;

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
     * @param[in] search_tree 搜索树扩展边
     * @param[in] solution_node_ids 最终解链搜索节点 id
     * @param[in] solution_path_frame_starts 解节点对应路径帧
     * @return 格式化后的 JSON 字符串
     */
    [[nodiscard]] static std::string exportPath(
        const GridMap& map,
        const Car& car,
        const std::vector<CarPose>& path,
        const std::vector<CarPose>& expanded = {},
        const std::vector<SearchTreeEdge>& search_tree = {},
        const std::vector<int>& solution_node_ids = {},
        const std::vector<int>& solution_path_frame_starts = {});

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

    /** @brief 单独导出整数数组。 */
    [[nodiscard]] static std::string exportIntArray(
        const std::vector<int>& values);

    /** @brief 单独导出搜索树扩展边数组。 */
    [[nodiscard]] static std::string exportSearchTree(
        const std::vector<SearchTreeEdge>& edges);
};
