/**
 * @file GridMap.hpp
 * @brief 栅格地图定义
 *
 * 定义二维栅格地图数据结构，支持障碍物存储、起点/终点设置，
 * 以及JSON格式地图文件的加载。
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief 二维平面位姿。
 */
struct Pose2D {
    double x = 0.0;     ///< x 坐标
    double y = 0.0;     ///< y 坐标
    double theta = 0.0; ///< 航向角，弧度
};

/**
 * @brief 栅格地图，存储障碍物、起点和终点。
 *
 * 内部使用一维 vector 存储二维栅格，0 表示自由空间，1 表示障碍物。
 */
class GridMap {
public:
    /** @brief 构造空地图。 */
    GridMap() = default;

    /**
     * @brief 构造指定尺寸的空白地图。
     * @param[in] width  地图宽度（栅格数）
     * @param[in] height 地图高度（栅格数）
     */
    GridMap(int width, int height);

    [[nodiscard]] int width() const;
    [[nodiscard]] int height() const;

    /** @brief 判断地图是否未初始化（宽或高为 0）。 */
    [[nodiscard]] bool empty() const;

    /**
     * @brief 判断坐标是否在地图范围内。
     * @param[in] x 栅格列索引
     * @param[in] y 栅格行索引
     */
    [[nodiscard]] bool inBounds(int x, int y) const;

    /**
     * @brief 判断指定栅格是否为障碍物。
     * @param[in] x 栅格列索引
     * @param[in] y 栅格行索引
     */
    [[nodiscard]] bool isObstacle(int x, int y) const;

    /**
     * @brief 判断指定栅格是否为自由空间。
     * @param[in] x 栅格列索引
     * @param[in] y 栅格行索引
     */
    [[nodiscard]] bool isFree(int x, int y) const;

    /** @brief 返回当前障碍物总数。 */
    [[nodiscard]] int obstacleCount() const;

    /**
     * @brief 重置地图尺寸，清除所有数据。
     * @param[in] width  新宽度
     * @param[in] height 新高度
     */
    void resize(int width, int height);

    /** @brief 清除所有障碍物。 */
    void clearObstacles();

    /**
     * @brief 设置指定栅格的障碍物状态。
     * @param[in] x         栅格列索引
     * @param[in] y         栅格行索引
     * @param[in] occupied  true = 设为障碍物，false = 清除障碍物
     */
    void setObstacle(int x, int y, bool occupied = true);

    /** @brief 设置起点位姿。 */
    void setStart(Pose2D pose);

    /** @brief 设置目标点位姿。 */
    void setGoal(Pose2D pose);

    [[nodiscard]] const Pose2D& start() const;
    [[nodiscard]] const Pose2D& goal() const;

    /**
     * @brief 返回底层栅格数据。
     * @return 0 = 自由，1 = 障碍物
     */
    [[nodiscard]] const std::vector<std::uint8_t>& cells() const;

private:
    [[nodiscard]] int index(int x, int y) const;

    int width_ = 0;
    int height_ = 0;
    int obstacle_count_ = 0;
    Pose2D start_;
    Pose2D goal_;
    std::vector<std::uint8_t> cells_;
};

/**
 * @brief 从 JSON 文件加载地图。
 */
class MapLoader {
public:
    /**
     * @brief 读取 JSON 文件并构造 GridMap。
     * @param[in] path JSON 文件路径
     * @return 构造好的栅格地图
     * @throw std::runtime_error 文件无法打开或解析失败
     */
    static GridMap loadJson(const std::string& path);
};
