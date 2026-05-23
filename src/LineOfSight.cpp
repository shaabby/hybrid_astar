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
 * @brief 计算沿移动方向遇到的第一个网格线边界
 * @param coordinate 当前坐标值
 * @param step 步进方向（+1或-1）
 * @return 遇到的第一个网格边界的坐标
 *
 * 正向步进时，返回下一个整数边界（floor + 1）。
 * 负向步进时，返回当前整数边界（floor）。
 */
double firstGridBoundary(double coordinate, int step) {
    if (step > 0) {
        return static_cast<double>(std::floor(coordinate) + 1);
    }
    return static_cast<double>(std::floor(coordinate));
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

/**
 * @brief 将点所在的所有单元格追加到单元格列表
 * @param cells 要追加细胞的向量
 * @param point 要处理的点
 * @param eps 网格线检测的epsilon容差
 *
 * 当点恰好落在网格线或网格交点时，
 * 会添加额外的相邻单元格以实现超级覆盖属性。
 */
void appendCellsForPoint(std::vector<HashCell>& cells, Point2D point, double eps) {
    const int x = floorToCell(point.x);
    const int y = floorToCell(point.y);
    appendUnique(cells, {x, y});

    if (onGridLine(point.x, eps)) {
        appendUnique(cells, {x - 1, y});
    }
    if (onGridLine(point.y, eps)) {
        appendUnique(cells, {x, y - 1});
    }
    if (onGridLine(point.x, eps) && onGridLine(point.y, eps)) {
        appendUnique(cells, {x - 1, y - 1});
    }
}

/**
 * @brief Liang-Barsky风格的线条裁剪（下界）
 * @param p 裁剪边界的方向分量
 * @param q 边界端点的偏移量
 * @param t0 当前下界参数（会被原地修改）
 * @param t1 当前上界参数（会被原地修改）
 * @param eps 比较用的epsilon容差
 * @return 如果线段与边界相交返回true，被拒绝则返回false
 *
 * 实现Liang-Barsky线条裁剪算法的一边。
 * 更新t0和t1以反映新的裁剪区间。
 */
bool clipLower(double p, double q, double& t0, double& t1, double eps) {
    if (std::abs(p) <= eps) {
        return q >= 0.0;
    }

    const double r = q / p;
    if (p < 0.0) {
        if (r > t1) {
            return false;
        }
        t0 = std::max(t0, r);
    } else {
        if (r < t0) {
            return false;
        }
        t1 = std::min(t1, r);
    }
    return true;
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

    int x = floorToCell(a.x);
    int y = floorToCell(a.y);
    const int end_x = floorToCell(b.x);
    const int end_y = floorToCell(b.y);
    appendCellsForPoint(cells, a, eps);

    if (step_x == 0 && step_y == 0) {
        return cells;
    }

    const bool horizontal_on_grid_line = step_y == 0 && onGridLine(a.y, eps);
    const bool vertical_on_grid_line = step_x == 0 && onGridLine(a.x, eps);

    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_x = step_x == 0 ? infinity : 1.0 / std::abs(dx);
    const double t_delta_y = step_y == 0 ? infinity : 1.0 / std::abs(dy);

    double t_max_x = step_x == 0
        ? infinity
        : (firstGridBoundary(a.x, step_x) - a.x) / dx;
    double t_max_y = step_y == 0
        ? infinity
        : (firstGridBoundary(a.y, step_y) - a.y) / dy;

    if (t_max_x < 0.0) {
        t_max_x = 0.0;
    }
    if (t_max_y < 0.0) {
        t_max_y = 0.0;
    }

    while (x != end_x || y != end_y) {
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
    appendCellsForPoint(cells, b, eps);

    return cells;
}

/**
 * @brief 检查线段是否进入指定单元格的内部
 * @param a 线段的起点
 * @param b 线段的终点
 * @param cell 要测试的单元格
 * @param eps 比较用的epsilon容差（默认：1.0e-9）
 * @return 如果线段进入单元格内部返回true，否则返回false
 *
 * 使用Liang-Barsky算法将线段裁剪到单元格内部边界
 * （向内收缩eps以避免边界情况）。如果裁剪后的
 * 参数区间[t0, t1]有正长度，则线段进入内部。
 */
bool segmentEntersCellInterior(Point2D a,
                               Point2D b,
                               HashCell cell,
                               double eps) {
    const double xmin = static_cast<double>(cell.x) + eps;
    const double xmax = static_cast<double>(cell.x + 1) - eps;
    const double ymin = static_cast<double>(cell.y) + eps;
    const double ymax = static_cast<double>(cell.y + 1) - eps;
    if (xmin >= xmax || ymin >= ymax) {
        return false;
    }

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    double t0 = 0.0;
    double t1 = 1.0;

    if (!clipLower(-dx, a.x - xmin, t0, t1, eps)) {
        return false;
    }
    if (!clipLower(dx, xmax - a.x, t0, t1, eps)) {
        return false;
    }
    if (!clipLower(-dy, a.y - ymin, t0, t1, eps)) {
        return false;
    }
    if (!clipLower(dy, ymax - a.y, t0, t1, eps)) {
        return false;
    }

    return t0 <= t1 + eps;
}

/**
 * @brief 判断两点之间是否存在畅通的视域
 * @param a 起点
 * @param b 终点
 * @param obstacles 包含障碍物的单元格集合
 * @param eps 比较用的epsilon容差（默认：1.0e-9）
 * @return 如果视域畅通返回true，被阻挡则返回false
 *
 * 首先计算线段的超级覆盖（它接触的所有单元格），
 * 然后检查每个障碍物单元格，判断线段是否真正进入该单元格内部。
 * 单元格是障碍物并不足以阻挡视线，如果线段只经过其边界或角落。
 */
bool hasLineOfSight(Point2D a,
                    Point2D b,
                    const ObstacleSet& obstacles,
                    double eps) {
    for (const HashCell& cell : supercoverDdaCells(a, b, eps)) {
        if (!obstacles.contains(cell)) {
            continue;
        }
        if (segmentEntersCellInterior(a, b, cell, eps)) {
            return false;
        }
    }
    return true;
}
