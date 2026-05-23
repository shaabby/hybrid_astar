/**
 * @file VisualizationData.hpp
 * @brief 可视化数据结构定义
 *
 * 定义规划结果可视化所需的数据结构，包括地图、车辆配置、
 * 路径点和搜索扩展节点。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <vector>

/**
 * @brief 规划结果可视化数据结构
 *
 * 聚合地图、车辆配置、规划路径和搜索扩展节点，
 * 供FltkViewer和HtmlWriter使用。
 */
struct VisualizationData {
    const GridMap& map;                 ///< 栅格地图引用
    VehicleConfig vehicle;             ///< 车辆物理配置
    const std::vector<CarPose>& path;  ///< 规划路径点序列
    const std::vector<CarPose>& expanded; ///< 搜索扩展节点序列
};