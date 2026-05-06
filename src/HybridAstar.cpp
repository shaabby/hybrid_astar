#include "HybridAstar.hpp"
#include "CollisionChecker.hpp"
#include "Heuristic.hpp"
#include "ReedsShepp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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
 * @param[in] pose      连续位姿
 * @param[in] config    规划器配置
 * @param[in] heuristic 启发函数
 * @return 初始化后的 Node，g 为 0，h 由启发函数估计
 */
Node makeNode(const CarPose& pose,
              const HybridAstarConfig& config,
              const Heuristic& heuristic) {
    Node node;
    node.pose = pose;
    node.x_index = static_cast<int>(std::floor(pose.x / config.xy_resolution));
    node.y_index = static_cast<int>(std::floor(pose.y / config.xy_resolution));
    node.theta_index = thetaIndex(pose.theta, config.theta_bins);
    node.h = heuristic.estimate(pose);
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

bool shouldTryAnalyticExpansion(const CarPose& pose,
                                const GridMap& map,
                                const HybridAstarConfig& config,
                                int iterations) {
    if (!config.enable_analytic_expansion) {
        return false;
    }

    const int interval = std::max(1, config.analytic_expansion_interval);
    if (iterations % interval != 0) {
        return false;
    }

    const Pose2D& goal = map.goal();
    return distance2d(pose.x, pose.y, goal.x, goal.y)
        <= config.analytic_expansion_distance;
}

std::optional<ReedsSheppPath> tryAnalyticExpansion(
    const ReedsSheppGenerator& generator,
    const CarPose& pose,
    const GridMap& map,
    const ReedsSheppCollisionChecker& collision_checker) {
    const std::optional<ReedsSheppPath> candidate = generator.generate(
        pose, map.goal());

    if (!candidate || !collision_checker.isCollisionFree(*candidate)) {
        return std::nullopt;
    }

    return candidate;
}

/**
 * @brief 计算子节点的 g、h、f 代价并写入 Node。
 *
 * g 包含实际轨迹段长度、倒车惩罚、转向惩罚、换挡惩罚和转向变化惩罚。
 * h 由注入的启发函数计算。
 *
 * @param[in,out] node      待更新的子节点
 * @param[in]     parent    父节点
 * @param[in]     direction 行驶方向
 * @param[in]     steer     前轮转向角
 * @param[in]     segment_length 实际运动轨迹段长度
 * @param[in]     max_steer 最大前轮转向角
 * @param[in]     config    规划器配置
 * @param[in]     heuristic 启发函数
 */
void computeCost(Node& node,
                 const Node& parent,
                 int direction,
                 double steer,
                 double segment_length,
                 double max_steer,
                 const HybridAstarConfig& config,
                 const Heuristic& heuristic) {
    const double reverse_factor = direction < 0 ? config.reverse_penalty : 1.0;
    const double safe_max_steer = std::max(1.0e-9, std::abs(max_steer));
    const double steer_ratio = std::abs(steer) / safe_max_steer;
    const double steer_change_ratio = std::abs(steer - parent.pose.steer)
        / safe_max_steer;
    const double gear_switch_cost = parent.pose.direction != direction
        ? config.gear_switch_penalty
        : 0.0;
    const double steer_change_cost =
        config.steer_change_penalty * steer_change_ratio;

    node.g = parent.g
        + segment_length * reverse_factor
        + segment_length * config.steer_penalty * steer_ratio
        + gear_switch_cost
        + steer_change_cost;
    node.h = heuristic.estimate(node.pose);
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

HybridAstar::HybridAstar(HybridAstarConfig config,
                         std::shared_ptr<Heuristic> heuristic)
    : config_(config),
      heuristic_(std::move(heuristic)) {
    if (!heuristic_) {
        heuristic_ = std::make_shared<CombinedHeuristic>();
    }
}

PlanResult HybridAstar::plan(const GridMap& map, const Car& car) const {
    PlanResult result;
    heuristic_->prepare(map, car, config_);
    const Heuristic& heuristic = *heuristic_;
    VehicleCollisionConfig collision_config;
    collision_config.safety_margin = config_.collision_safety_margin;
    const VehicleCollisionChecker collision_checker(map, car, collision_config);

    ReedsSheppCollisionConfig rs_collision_config;
    rs_collision_config.vehicle = collision_config;
    const ReedsSheppCollisionChecker rs_collision_checker(
        map, car, rs_collision_config);
    const ReedsSheppGenerator rs_generator(
        car.minTurningRadius(),
        config_.step_size,
        car.maxSteer());

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

    if (collision_checker.collides(start)) {
        return result; // 起点在障碍物内，直接返回失败
    }

    std::vector<Node> nodes;
    nodes.reserve(4096);

    Node start_node = makeNode(start, config_, heuristic);
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

        // Reeds-Shepp analytic expansion: 在接近目标时尝试直接连接终点。
        //
        // 当前生成器仍是最小实现（Dubins + reverse-Dubins 候选），后续优化点：
        // 1. 补齐完整 Reeds-Shepp cusp words；
        // 2. 加入倒车、换挡、转向变化等代价后再选择候选；
        // 3. 按距离、启发式收益或失败历史自适应触发。
        if (shouldTryAnalyticExpansion(
                current.pose, map, config_, iterations)) {
            if (const std::optional<ReedsSheppPath> analytic_path =
                    tryAnalyticExpansion(rs_generator, current.pose, map,
                                         rs_collision_checker)) {
                result.success = true;
                result.path = reconstructPath(nodes, current_id);
                result.path.insert(result.path.end(),
                                   analytic_path->samples.begin(),
                                   analytic_path->samples.end());
                return result;
            }
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

                    if (collision_checker.collides(pose)) {
                        collision = true;
                        break;
                    }
                    segment.push_back(pose);
                }

                if (collision || segment.empty()) {
                    continue; // 碰撞或没有移动，放弃该分支
                }

                // 5.2 构造子节点
                Node next = makeNode(pose, config_, heuristic);
                next.parent = current_id;
                next.segment = std::move(segment);

                // 5.3 计算代价（含倒车、转向、换挡和转向变化惩罚）
                const double segment_length = config_.step_size
                    * static_cast<double>(next.segment.size());
                computeCost(next, current, direction, steer, segment_length,
                            car.maxSteer(), config_, heuristic);

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

std::string HybridAstar::heuristicName() const {
    return heuristic_ ? heuristic_->name() : "unknown";
}
