/**
 * @file HybridAstar.hpp
 * @brief Hybrid A*路径规划器定义
 *
 * 在三维状态空间(x, y, theta)中搜索连续、车辆可行的路径。
 * 与普通A*不同，Hybrid A*的每次节点扩展使用车辆运动学模型生成连续轨迹段。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <memory>
#include <vector>

class Heuristic;

enum class ObstacleHeuristicType {
    VisibilityGraph,
    ReverseDijkstra
};

/**
 * @brief Hybrid A* 规划器配置参数
 */
struct HybridAstarConfig {
    double xy_resolution;              ///< 位置离散分辨率
    int theta_bins;                    ///< 航向角离散分箱数
    double step_size;                  ///< 数值积分步长
    double primitive_length;           ///< 单个运动基元长度
    double goal_xy_tolerance;          ///< 目标位置容差
    double goal_theta_tolerance;       ///< 目标航向角容差（弧度）
    double reverse_penalty;            ///< 倒车代价惩罚系数
    double steer_penalty;              ///< 转向代价惩罚系数
    double gear_switch_penalty;        ///< 前进/倒车切换惩罚
    double steer_change_penalty;       ///< 相邻运动基元转向变化惩罚
    int max_iterations;                ///< 最大搜索迭代次数
    bool allow_reverse;                ///< 是否允许倒车运动
    bool enable_analytic_expansion;    ///< 是否启用Reeds-Shepp直连目标
    double analytic_expansion_distance; ///< 尝试直连目标的距离阈值
    int analytic_expansion_interval;   ///< 每隔多少次扩展尝试一次直连
    double collision_safety_margin;    ///< 车辆footprint碰撞检测安全外扩
    bool enable_obstacle_heuristic;    ///< 是否启用障碍物启发式
    double obstacle_lookup_resolution;  ///< 障碍物启发式查表分辨率
    ObstacleHeuristicType obstacle_heuristic_type =
        ObstacleHeuristicType::VisibilityGraph;
    double obstacle_heuristic_inflation_alpha = 1.0;
    bool enable_timing;                 ///< 是否启用细分计时统计
    bool debug;                        ///< 是否输出运行阶段调试信息
    int debug_progress_interval;       ///< 搜索循环调试输出间隔
};

struct TimingBreakdown {
    double heuristic_prepare_ms = 0.0;
    double search_loop_ms = 0.0;
    double obstacle_collect_ms = 0.0;
    double visibility_points_ms = 0.0;
    double visibility_graph_ms = 0.0;
    double visibility_dijkstra_ms = 0.0;
    double obstacle_lookup_ms = 0.0;
    double reverse_dijkstra_inflation_ms = 0.0;
    double reverse_dijkstra_ms = 0.0;
    double non_obstacle_heuristic_ms = 0.0;
    double obstacle_heuristic_ms = 0.0;
    std::size_t heuristic_estimate_calls = 0;
    double primitive_collision_check_ms = 0.0;
    std::size_t primitive_collision_check_calls = 0;
    double analytic_expansion_ms = 0.0;
    std::size_t analytic_attempts = 0;
    std::size_t analytic_successes = 0;
    double analytic_rs_generation_ms = 0.0;
    std::size_t analytic_rs_generation_calls = 0;
    double analytic_collision_check_ms = 0.0;
    std::size_t analytic_collision_check_calls = 0;
};

/**
 * @brief 搜索树中的一条扩展边，用于可视化调试。
 */
struct SearchTreeEdge {
    int parent = -1;                 ///< 父搜索节点 id
    int child = -1;                  ///< 子搜索节点 id，未进入 open 时为 -1
    int open_order = -1;             ///< 子节点进入 open 的顺序
    int close_order = -1;            ///< 子节点进入 closed set 的顺序
    int pop_order = -1;              ///< 子节点从 open 弹出的顺序
    CarPose from;                    ///< 父节点位姿
    CarPose to;                      ///< 扩展尝试结束位姿
    std::vector<CarPose> segment;    ///< 从父节点到子节点的运动轨迹
    bool accepted = false;           ///< 是否成功加入 open/nodes
    bool collision = false;          ///< 是否因碰撞失败
    bool duplicate = false;          ///< 是否因 closed/best_g 被拒绝
    bool in_solution = false;        ///< 是否属于最终解 parent 链
};

/**
 * @brief 规划结果
 */
struct PlanResult {
    bool success = false;      ///< 是否找到可行路径
    std::vector<CarPose> path;     ///< 最终路径点序列
    std::vector<CarPose> expanded; ///< 扩展过的搜索节点（用于调试可视化）
    std::vector<SearchTreeEdge> search_tree; ///< 搜索树扩展边
    std::vector<int> solution_node_ids;      ///< 最终解链中的搜索节点 id
    std::vector<int> solution_open_orders;   ///< 最终解节点进入 open 的顺序
    std::vector<int> solution_close_orders;  ///< 最终解节点进入 closed set 的顺序
    std::vector<int> solution_pop_orders;    ///< 最终解节点从 open 弹出的顺序
    std::vector<int> solution_path_frame_starts; ///< 解节点对应的路径帧
    int iterations = 0;            ///< 搜索循环迭代次数
    std::size_t generated_nodes = 0; ///< 搜索过程中生成的节点数量
    std::size_t open_remaining = 0;  ///< 结束时 open set 中剩余条目数量
    TimingBreakdown timing;          ///< 细分计时统计
};

/**
 * @brief Hybrid A* 路径规划器
 *
 * 在三维状态空间(x, y, theta)中搜索连续、车辆可行的路径。
 * 与普通A*不同，Hybrid A*的每次节点扩展使用车辆运动学模型生成连续轨迹段，
 * 而不是离散网格移动。规划结果可以直接用于车辆跟踪控制。
 */
class HybridAstar {
public:
    /**
     * @brief 构造规划器
     * @param[in] config     规划参数，默认使用内置默认值
     * @param[in] heuristic  启发函数，默认为CombinedHeuristic
     */
    explicit HybridAstar(HybridAstarConfig config,
                         std::shared_ptr<Heuristic> heuristic = nullptr);

    /**
     * @brief 在指定地图上执行路径规划
     * @param[in] map 栅格地图（包含障碍物、起点、终点）
     * @param[in] car 车辆模型（用于运动学前向模拟）
     * @return 规划结果，包含路径和扩展节点
     */
    [[nodiscard]] PlanResult plan(const GridMap& map, const Car& car) const;

    /** @brief 返回当前使用的启发式名称。 */
    [[nodiscard]] std::string heuristicName() const;

private:
    HybridAstarConfig config_;           ///< 规划器配置
    std::shared_ptr<Heuristic> heuristic_; ///< 启发函数
};
