/**
 * @file AppConfig.hpp
 * @brief 应用程序配置定义
 *
 * 定义应用程序配置结构和YAML格式加载器，
 * 用于从配置文件读取地图路径、车辆参数和规划器参数。
 */

#pragma once

#include "Car.hpp"
#include "HybridAstar.hpp"

#include <string>

/** @brief 应用程序配置结构，聚合地图、车辆和规划器配置。 */
struct AppConfig {
    std::string map_path;
    VehicleConfig vehicle;
    HybridAstarConfig hybrid_astar;
};

/** @brief 应用程序配置加载器。 */
class AppConfigLoader {
public:
    /** @brief 从YAML文件加载完整配置。 */
    [[nodiscard]] static AppConfig loadYaml(const std::string& path);
};
