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

/** @brief 二维平面点坐标。 */
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

/** @brief 栅格哈希单元格坐标。 */
struct HashCell {
    int x = 0;
    int y = 0;

    bool operator==(const HashCell& other) const {
        return x == other.x && y == other.y;
    }
};

/** @brief HashCell哈希函数对象。 */
struct HashCellHasher {
    std::size_t operator()(const HashCell& cell) const;
};

using ObstacleSet = std::unordered_set<HashCell, HashCellHasher>;

[[nodiscard]] std::vector<HashCell> supercoverDdaCells(Point2D a,
                                                       Point2D b,
                                                       double eps = 1.0e-9);

[[nodiscard]] bool hasLineOfSight(Point2D a,
                                  Point2D b,
                                  const ObstacleSet& obstacles,
                                  double eps = 1.0e-9);
