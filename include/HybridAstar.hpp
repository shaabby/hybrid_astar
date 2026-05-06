#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <memory>
#include <vector>

class Heuristic;

/**
 * @brief Hybrid A* 规划器配置参数。
 */
struct HybridAstarConfig {
    double xy_resolution = 1.0;         ///< 位置离散分辨率
    int theta_bins = 72;                ///< 航向角离散分箱数
    double step_size = 0.2;             ///< 数值积分步长
    double primitive_length = 1.2;      ///< 单个运动基元长度
    double goal_xy_tolerance = 1.8;     ///< 目标位置容差
    double goal_theta_tolerance = 0.7;  ///< 目标航向角容差，弧度
    double reverse_penalty = 1.4;       ///< 倒车代价惩罚系数
    double steer_penalty = 0.15;        ///< 转向代价惩罚系数
    double gear_switch_penalty = 4.0;   ///< 前进/倒车切换惩罚
    double steer_change_penalty = 1.0;  ///< 相邻运动基元转向变化惩罚
    int max_iterations = 120000;        ///< 最大搜索迭代次数
    bool allow_reverse = true;          ///< 是否允许倒车运动
    bool enable_analytic_expansion = true; ///< 是否启用 Reeds-Shepp 直连目标
    double analytic_expansion_distance = 12.0; ///< 尝试直连目标的距离阈值
    int analytic_expansion_interval = 10; ///< 每隔多少次扩展尝试一次直连
    double collision_safety_margin = 0.0; ///< 车辆 footprint 碰撞检测安全外扩
    bool enable_obstacle_heuristic = true; ///< 是否启用二维障碍物启发式
    double obstacle_heuristic_inflate_alpha = 1.0; ///< 障碍物启发式膨胀系数
};

/**
 * @brief 规划结果。
 */
struct PlanResult {
    bool success = false;               ///< 是否找到可行路径
    std::vector<CarPose> path;          ///< 最终路径点序列
    std::vector<CarPose> expanded;      ///< 扩展过的搜索节点（用于调试可视化）
};

/**
 * @brief Hybrid A* 路径规划器。
 *
 * 在三维状态空间 (x, y, theta) 中搜索连续、车辆可行的路径。
 * 与普通 A* 不同，Hybrid A* 的每次节点扩展使用车辆运动学模型生成连续轨迹段。
 */
class HybridAstar {
public:
    /**
     * @brief 构造规划器。
     * @param[in] config 规划参数，默认使用内置默认值
     */
    explicit HybridAstar(HybridAstarConfig config = {},
                         std::shared_ptr<Heuristic> heuristic = nullptr);

    /**
     * @brief 在指定地图上执行路径规划。
     * @param[in] map 栅格地图（包含障碍物、起点、终点）
     * @param[in] car 车辆模型（用于运动学前向模拟）
     * @return 规划结果，包含路径和扩展节点
     */
    [[nodiscard]] PlanResult plan(const GridMap& map, const Car& car) const;
    [[nodiscard]] std::string heuristicName() const;

private:
    HybridAstarConfig config_;
    std::shared_ptr<Heuristic> heuristic_;
};
