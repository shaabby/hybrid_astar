#include "HybridAstar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {

/// @brief 圆周率常量。
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief Hybrid A* 搜索节点。
 *
 * 同时保存连续位姿（用于路径输出）和离散索引（用于 closed set 去重）。
 * segment 记录从父节点到本节点的连续轨迹段，最终回溯时拼接为完整路径。
 */
struct Node {
    CarPose pose;          ///< 连续位姿
    int x_index = 0;       ///< x 方向离散索引
    int y_index = 0;       ///< y 方向离散索引
    int theta_index = 0;   ///< 航向角离散分箱索引
    double g = 0.0;        ///< 从起点到当前节点的累计代价
    double h = 0.0;        ///< 启发式估计代价（到目标的距离）
    double f = 0.0;        ///< f = g + h，优先队列排序依据
    int parent = -1;       ///< 父节点在 nodes 数组中的索引，-1 表示起点
    std::vector<CarPose> segment; ///< 从父节点运动到本节点的连续轨迹
};

/**
 * @brief 优先队列（open set）条目。
 *
 * std::priority_queue 默认是大顶堆，因此 operator< 反向比较，
 * 使 f 值最小的节点位于堆顶。
 */
struct OpenEntry {
    double f = 0.0;        ///< 节点的 f 值
    int node_id = -1;      ///< 节点在 nodes 数组中的索引

    bool operator<(const OpenEntry& other) const {
        return f > other.f;
    }
};

/**
 * @brief 将角度归一化到 (-π, π] 区间。
 * @param[in] angle 输入角度，弧度
 * @return 归一化后的角度
 */
double normalizeAngle(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

/**
 * @brief 计算两个角度之间的最小差值。
 * @param[in] lhs 第一个角度，弧度
 * @param[in] rhs 第二个角度，弧度
 * @return 最小角度差，范围 [0, π]
 */
double angleDiff(double lhs, double rhs) {
    return std::abs(normalizeAngle(lhs - rhs));
}

/**
 * @brief 计算二维欧几里得距离。
 */
double distance2d(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

using Point2 = std::array<double, 2>;
using Quad = std::array<Point2, 4>;

double dot(const Point2& lhs, const Point2& rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1];
}

void project(const Quad& polygon, const Point2& axis, double& min, double& max) {
    min = dot(polygon[0], axis);
    max = min;
    for (std::size_t i = 1; i < polygon.size(); ++i) {
        const double value = dot(polygon[i], axis);
        min = std::min(min, value);
        max = std::max(max, value);
    }
}

bool overlapsOnAxis(const Quad& lhs, const Quad& rhs, const Point2& axis) {
    constexpr double kOverlapEpsilon = 1.0e-9;
    double lhs_min = 0.0;
    double lhs_max = 0.0;
    double rhs_min = 0.0;
    double rhs_max = 0.0;
    project(lhs, axis, lhs_min, lhs_max);
    project(rhs, axis, rhs_min, rhs_max);
    return lhs_max >= rhs_min - kOverlapEpsilon
        && rhs_max >= lhs_min - kOverlapEpsilon;
}

bool rectanglesOverlap(const Quad& lhs, const Quad& rhs) {
    const std::array<Point2, 4> axes = {{
        {lhs[1][0] - lhs[0][0], lhs[1][1] - lhs[0][1]},
        {lhs[2][0] - lhs[1][0], lhs[2][1] - lhs[1][1]},
        {1.0, 0.0},
        {0.0, 1.0}
    }};

    for (const Point2& axis : axes) {
        if (!overlapsOnAxis(lhs, rhs, axis)) {
            return false;
        }
    }
    return true;
}

Quad gridCellCorners(int x, int y) {
    const double left = static_cast<double>(x);
    const double bottom = static_cast<double>(y);
    const double right = left + 1.0;
    const double top = bottom + 1.0;
    return {{
        {left, bottom},
        {right, bottom},
        {right, top},
        {left, top}
    }};
}

/**
 * @brief 将连续航向角离散化为分箱索引。
 * @param[in] theta 连续航向角，弧度
 * @param[in] bins  总的分箱数量
 * @return 离散化后的索引，范围 [0, bins)
 */
int thetaIndex(double theta, int bins) {
    const double normalized = normalizeAngle(theta);
    const double shifted = normalized + kPi;
    int index = static_cast<int>(std::floor(shifted / (2.0 * kPi) * bins));
    if (index >= bins) {
        index = bins - 1;
    }
    return std::max(0, index);
}

/**
 * @brief 将三个离散索引编码为一个 64 位整数，用于 closed set 和 best_g 的键。
 *
 * 使用位运算保证 (x, y, theta) 组合的唯一性。
 */
std::int64_t makeKey(int x_index, int y_index, int theta_index) {
    return (static_cast<std::int64_t>(x_index) << 40)
        ^ (static_cast<std::int64_t>(y_index) << 20)
        ^ static_cast<std::int64_t>(theta_index);
}

/**
 * @brief 根据连续位姿和配置构造搜索节点。
 * @param[in] pose   连续位姿
 * @param[in] map    栅格地图（用于计算启发式）
 * @param[in] config 规划器配置
 * @return 初始化后的 Node，g 为 0，h 为到目标的欧几里得距离
 */
Node makeNode(const CarPose& pose,
              const GridMap& map,
              const HybridAstarConfig& config) {
    Node node;
    node.pose = pose;
    node.x_index = static_cast<int>(std::floor(pose.x / config.xy_resolution));
    node.y_index = static_cast<int>(std::floor(pose.y / config.xy_resolution));
    node.theta_index = thetaIndex(pose.theta, config.theta_bins);
    node.h = distance2d(pose.x, pose.y, map.goal().x, map.goal().y);
    node.f = node.g + node.h;
    return node;
}

/**
 * @brief 判断当前位姿是否到达目标区域。
 *
 * 同时检查位置距离和航向角差值是否在容差范围内。
 */
bool isGoal(const CarPose& pose,
            const GridMap& map,
            const HybridAstarConfig& config) {
    const Pose2D& goal = map.goal();
    return distance2d(pose.x, pose.y, goal.x, goal.y) <= config.goal_xy_tolerance
        && angleDiff(pose.theta, goal.theta) <= config.goal_theta_tolerance;
}

/**
 * @brief 检查车辆矩形 footprint 是否与障碍栅格相交。
 */
bool collidesVehicle(const GridMap& map, const Car& car, const CarPose& pose) {
    const Quad corners = car.bodyCorners(pose);

    double min_x = corners[0][0];
    double max_x = corners[0][0];
    double min_y = corners[0][1];
    double max_y = corners[0][1];
    for (const Point2& corner : corners) {
        min_x = std::min(min_x, corner[0]);
        max_x = std::max(max_x, corner[0]);
        min_y = std::min(min_y, corner[1]);
        max_y = std::max(max_y, corner[1]);
    }

    const int x0 = static_cast<int>(std::floor(min_x));
    const int x1 = static_cast<int>(std::floor(max_x));
    const int y0 = static_cast<int>(std::floor(min_y));
    const int y1 = static_cast<int>(std::floor(max_y));

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            if (map.isObstacle(x, y)
                && rectanglesOverlap(corners, gridCellCorners(x, y))) {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief 计算子节点的 g、h、f 代价并写入 Node。
 *
 * g 包含运动基元长度、倒车惩罚和转向惩罚。
 * h 使用欧几里得距离启发式。
 *
 * @param[in,out] node      待更新的子节点
 * @param[in]     parent_g  父节点累计代价 g
 * @param[in]     direction 行驶方向
 * @param[in]     steer     前轮转向角
 * @param[in]     config    规划器配置
 * @param[in]     map       栅格地图
 */
void computeCost(Node& node,
                 double parent_g,
                 int direction,
                 double steer,
                 const HybridAstarConfig& config,
                 const GridMap& map) {
    const double reverse_cost = direction < 0 ? config.reverse_penalty : 1.0;
    const double steer_cost = 1.0 + std::abs(steer) * config.steer_penalty;
    node.g = parent_g + config.primitive_length * reverse_cost * steer_cost;
    node.h = distance2d(node.pose.x, node.pose.y, map.goal().x, map.goal().y);
    node.f = node.g + node.h;
}

/**
 * @brief 从目标节点回溯到起点，重建完整路径。
 *
 * 通过 parent 指针链逆向遍历，反转后按正向顺序输出。
 * 相邻节点之间的 segment 被依次拼接，形成连续轨迹。
 *
 * @param[in] nodes   所有已生成节点的数组
 * @param[in] goal_id 目标节点在数组中的索引
 * @return 从起点到终点的连续位姿序列
 */
std::vector<CarPose> reconstructPath(const std::vector<Node>& nodes, int goal_id) {
    std::vector<int> ids;
    for (int id = goal_id; id >= 0; id = nodes[id].parent) {
        ids.push_back(id);
    }
    std::reverse(ids.begin(), ids.end());

    std::vector<CarPose> path;
    if (ids.empty()) {
        return path;
    }

    path.push_back(nodes[ids.front()].pose);
    for (std::size_t i = 1; i < ids.size(); ++i) {
        const std::vector<CarPose>& segment = nodes[ids[i]].segment;
        path.insert(path.end(), segment.begin(), segment.end());
    }
    return path;
}

} // namespace

HybridAstar::HybridAstar(HybridAstarConfig config)
    : config_(config) {}

PlanResult HybridAstar::plan(const GridMap& map, const Car& car) const {
    PlanResult result;

    // ------------------------------------------------------------------
    // 1. 初始化起点
    // ------------------------------------------------------------------
    CarPose start{
        .x = map.start().x,
        .y = map.start().y,
        .theta = map.start().theta,
        .steer = 0.0,
        .direction = 1
    };

    if (collidesVehicle(map, car, start)) {
        return result; // 起点在障碍物内，直接返回失败
    }

    std::vector<Node> nodes;
    nodes.reserve(4096);

    Node start_node = makeNode(start, map, config_);
    start_node.g = 0.0;
    start_node.f = start_node.h;
    nodes.push_back(start_node);

    // ------------------------------------------------------------------
    // 2. 初始化 open set（优先队列）和 closed set
    // ------------------------------------------------------------------
    std::priority_queue<OpenEntry> open;
    open.push({start_node.f, 0});

    std::unordered_map<std::int64_t, double> best_g;
    std::unordered_set<std::int64_t> closed;
    best_g[makeKey(start_node.x_index, start_node.y_index, start_node.theta_index)] = 0.0;

    // ------------------------------------------------------------------
    // 3. 预计算运动基元控制量
    // ------------------------------------------------------------------
    const std::vector<int> directions = config_.allow_reverse
        ? std::vector<int>{1, -1}
        : std::vector<int>{1};
    const std::vector<double> steers = {-car.maxSteer(), 0.0, car.maxSteer()};
    const int substeps = std::max(1, static_cast<int>(std::round(
        config_.primitive_length / config_.step_size)));

    // ------------------------------------------------------------------
    // 4. 主搜索循环（A*）
    // ------------------------------------------------------------------
    int iterations = 0;
    while (!open.empty() && iterations < config_.max_iterations) {
        ++iterations;

        const OpenEntry current_entry = open.top();
        open.pop();

        const int current_id = current_entry.node_id;
        const Node current = nodes[current_id];
        const std::int64_t current_key = makeKey(
            current.x_index, current.y_index, current.theta_index);

        // 跳过已关闭的节点（优先队列中可能存在过期条目）
        if (closed.contains(current_key)) {
            continue;
        }
        closed.insert(current_key);
        result.expanded.push_back(current.pose);

        // 到达目标，重建路径并返回
        if (isGoal(current.pose, map, config_)) {
            result.success = true;
            result.path = reconstructPath(nodes, current_id);
            return result;
        }

        // ------------------------------------------------------------------
        // 5. 扩展邻居：遍历所有 (direction, steer) 组合
        // ------------------------------------------------------------------
        for (int direction : directions) {
            for (double steer : steers) {
                CarPose pose = current.pose;
                std::vector<CarPose> segment;
                segment.reserve(static_cast<std::size_t>(substeps));

                // 5.1 前向模拟：用自行车模型积分一段轨迹
                bool collision = false;
                for (int i = 0; i < substeps; ++i) {
                    pose = car.step(pose, steer, direction, config_.step_size);
                    pose.theta = normalizeAngle(pose.theta);

                    if (collidesVehicle(map, car, pose)) {
                        collision = true;
                        break;
                    }
                    segment.push_back(pose);
                }

                if (collision || segment.empty()) {
                    continue; // 碰撞或没有移动，放弃该分支
                }

                // 5.2 构造子节点
                Node next = makeNode(pose, map, config_);
                next.parent = current_id;
                next.segment = std::move(segment);

                // 5.3 计算代价（含倒车和转向惩罚）
                computeCost(next, current.g, direction, steer, config_, map);

                // 5.4 去重：检查 closed set 和 best_g
                const std::int64_t next_key = makeKey(
                    next.x_index, next.y_index, next.theta_index);
                if (closed.contains(next_key)) {
                    continue;
                }

                const auto best = best_g.find(next_key);
                if (best != best_g.end() && best->second <= next.g) {
                    continue; // 已有更优路径到达该离散状态
                }

                // 5.5 加入 open set
                const int next_id = static_cast<int>(nodes.size());
                best_g[next_key] = next.g;
                nodes.push_back(std::move(next));
                open.push({nodes[next_id].f, next_id});
            }
        }
    }

    // 达到最大迭代次数仍未找到路径，返回失败
    return result;
}
