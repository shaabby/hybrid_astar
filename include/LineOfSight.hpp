/**
 * @file LineOfSight.hpp
 * @brief 视线检测定义
 *
 * 使用Supercover DDA算法计算两点之间的所有栅格，
 * 并判断是否存在无障碍的视线连接。
 */

#pragma once

#include <cstddef>
#include <unordered_set>
#include <vector>

/**
 * @brief 二维平面点坐标。
 */
struct Point2D {
    double x = 0.0; ///< x 坐标
    double y = 0.0; ///< y 坐标
};

/**
 * @brief 栅格哈希单元格坐标。
 */
struct HashCell {
    int x = 0; ///< 列索引
    int y = 0; ///< 行索引

    bool operator==(const HashCell& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief HashCell 哈希函数对象。
 */
struct HashCellHasher {
    std::size_t operator()(const HashCell& cell) const;
};

using ObstacleSet = std::unordered_set<HashCell, HashCellHasher>;

/**
 * @brief 计算两点之间的Supercover DDA栅格序列。
 *
 * Supercover DDA 保证线段经过的所有栅格都被包含，
 * 包括沿线段"紧贴"的对角相邻栅格，比普通DDA覆盖更完整。
 *
 * @param[in] a     起点坐标
 * @param[in] b     终点坐标
 * @param[in] eps   极小值，用于浮点数比较
 * @return 按顺序经过的所有栅格坐标
 */
[[nodiscard]] std::vector<HashCell> supercoverDdaCells(Point2D a,
                                                       Point2D b,
                                                       double eps = 1.0e-9);

/**
 * @brief 判断两点之间是否存在无障碍视线。
 *
 * 使用Supercover DDA算法获取线段经过的所有栅格，
 * 若其中任意一格为障碍物则返回false。
 *
 * @param[in] a          起点坐标
 * @param[in] b          终点坐标
 * @param[in] obstacles  障碍物集合
 * @param[in] eps        极小值，用于浮点数比较
 * @return true = 两点间无障碍，false = 存在障碍物遮挡
 */
[[nodiscard]] bool hasLineOfSight(Point2D a,
                                  Point2D b,
                                  const ObstacleSet& obstacles,
                                  double eps = 1.0e-9);