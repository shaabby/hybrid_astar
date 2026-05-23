/**
 * @file LineOfSight.cpp
 * @brief 超级覆盖DDA视域算法实现
 *
 * 本模块提供基于网格的视域计算，使用超级覆盖DDA（数字差分分析器）算法。
 * 用于确定线段经过的所有网格单元，并检测障碍物是否阻挡路径。
 */

#include "LineOfSight.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

/**
 * @brief 将浮点值向下取整到最近的整数网格坐标
 * @param value 浮点坐标值
 * @return 单元格索引（输入值的向下取整）
 */
int floorToCell(double value) {
    return static_cast<int>(std::floor(value));
}

/**
 * @brief 返回数值的符号，带有epsilon容差
 * @param value 要判断的值
 * @param eps 零值比较的epsilon容差
 * @return value > eps返回1，value < -eps返回-1，否则返回0
 */
int sign(double value, double eps) {
    if (value > eps) {
        return 1;
    }
    if (value < -eps) {
        return -1;
    }
    return 0;
}

/**
 * @brief 如果单元格不存在则将其追加到列表中
 * @param cells 要追加的向量
 * @param cell 要添加的单元格
 *
 * 执行线性搜索以避免单元格列表中的重复。
 */
void appendUnique(std::vector<HashCell>& cells, HashCell cell) {
    if (!cells.empty() && cells.back() == cell) {
        return;
    }
    cells.push_back(cell);
}

/**
 * @brief 检查值是否在网格线上（epsilon容差范围内）
 * @param value 要检查的坐标值
 * @param eps 比较用的epsilon容差
 * @return 如果值在网格线上返回true，否则返回false
 */
bool onGridLine(double value, double eps) {
    return std::abs(value - std::round(value)) <= eps;
}

std::vector<int> directedAxisCells(double coordinate,
                                   int step,
                                   bool is_end_point,
                                   double eps) {
    const int cell = floorToCell(coordinate);
    if (!onGridLine(coordinate, eps)) {
        return {cell};
    }
    if (step == 0) {
        return {cell, cell - 1};
    }
    if (step > 0) {
        return {is_end_point ? cell - 1 : cell};
    }
    return {is_end_point ? cell : cell - 1};
}

void appendCellsForDirectedPoint(std::vector<HashCell>& cells,
                                 Point2D point,
                                 int step_x,
                                 int step_y,
                                 bool is_end_point,
                                 double eps) {
    const std::vector<int> x_cells =
        directedAxisCells(point.x, step_x, is_end_point, eps);
    const std::vector<int> y_cells =
        directedAxisCells(point.y, step_y, is_end_point, eps);

    for (const int x : x_cells) {
        for (const int y : y_cells) {
            appendUnique(cells, {x, y});
        }
    }
}

int startCellIndex(double coordinate, int step, double eps) {
    if (step < 0 && onGridLine(coordinate, eps)) {
        return floorToCell(coordinate) - 1;
    }
    return floorToCell(coordinate);
}

double firstGridBoundary(double coordinate, int step, double eps) {
    if (step > 0) {
        return static_cast<double>(std::floor(coordinate) + 1);
    }
    if (onGridLine(coordinate, eps)) {
        return static_cast<double>(std::floor(coordinate) - 1);
    }
    return static_cast<double>(std::floor(coordinate));
}

} // namespace

/**
 * @brief HashCell结构体的哈希函数
 * @param cell 要哈希的单元格
 * @return 结合x和y坐标的哈希值
 *
 * 将32位的x和y坐标组合成单个size_t值。
 */
std::size_t HashCellHasher::operator()(const HashCell& cell) const {
    const std::uint64_t x = static_cast<std::uint32_t>(cell.x);
    const std::uint64_t y = static_cast<std::uint32_t>(cell.y);
    return static_cast<std::size_t>((x << 32U) ^ y);
}

/**
 * @brief 使用超级覆盖DDA算法计算线段经过的所有网格单元
 * @param a 线段的起点
 * @param b 线段的终点
 * @param eps 数值比较的epsilon容差（默认：1.0e-9）
 * @return 线段经过的所有HashCell对象的向量（超级覆盖）
 *
 * 超级覆盖DDA算法确保包含线段接触的所有单元格，
 * 而不仅仅是线段穿过中心的单元格。这是通过追踪线段何时
 * 经过网格边界并在这些点包含相邻单元格来实现的。
 *
 * 算法使用参数化线条遍历，其中t_max_x和t_max_y
 * 分别决定线段何时穿过垂直和水平网格线。
 */
std::vector<HashCell> supercoverDdaCells(Point2D a, Point2D b, double eps) {
    std::vector<HashCell> cells;

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const int step_x = sign(dx, eps);
    const int step_y = sign(dy, eps);

    if (step_x == 0 && step_y == 0) {
        appendUnique(cells, {floorToCell(a.x), floorToCell(a.y)});
        return cells;
    }

    int x = startCellIndex(a.x, step_x, eps);
    int y = startCellIndex(a.y, step_y, eps);
    appendCellsForDirectedPoint(cells, a, step_x, step_y, false, eps);

    const bool horizontal_on_grid_line = step_y == 0 && onGridLine(a.y, eps);
    const bool vertical_on_grid_line = step_x == 0 && onGridLine(a.x, eps);

    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_x = step_x == 0 ? infinity : 1.0 / std::abs(dx);
    const double t_delta_y = step_y == 0 ? infinity : 1.0 / std::abs(dy);

    double t_max_x = step_x == 0
        ? infinity
        : (firstGridBoundary(a.x, step_x, eps) - a.x) / dx;
    double t_max_y = step_y == 0
        ? infinity
        : (firstGridBoundary(a.y, step_y, eps) - a.y) / dy;

    if (t_max_x < 0.0) {
        t_max_x = 0.0;
    }
    if (t_max_y < 0.0) {
        t_max_y = 0.0;
    }

    while (std::min(t_max_x, t_max_y) < 1.0 - eps) {
        if (t_max_x < t_max_y - eps) {
            x += step_x;
            t_max_x += t_delta_x;
        } else if (t_max_y < t_max_x - eps) {
            y += step_y;
            t_max_y += t_delta_y;
        } else {
            appendUnique(cells, {x + step_x, y});
            appendUnique(cells, {x, y + step_y});
            x += step_x;
            y += step_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
        }
        appendUnique(cells, {x, y});
        if (horizontal_on_grid_line) {
            appendUnique(cells, {x, y - 1});
        }
        if (vertical_on_grid_line) {
            appendUnique(cells, {x - 1, y});
        }
    }
    appendCellsForDirectedPoint(cells, b, step_x, step_y, true, eps);

    return cells;
}


/**
 * @brief 判断两点之间是否存在畅通的视域
 * @param a 起点
 * @param b 终点
 * @param obstacles 包含障碍物的单元格集合
 * @param eps 比较用的epsilon容差（默认：1.0e-9）
 * @return 如果视域畅通返回true，被阻挡则返回false
 *
 * 计算线段的超级覆盖（它接触的所有单元格），
 * 只要路径接触到障碍物单元格就认为被阻挡。
 */
bool hasLineOfSight(Point2D a,
                    Point2D b,
                    const ObstacleSet& obstacles,
                    double eps) {
    for (const HashCell& cell : supercoverDdaCells(a, b, eps)) {
        if (obstacles.contains(cell)) {
            return false;
        }
    }
    return true;
}
