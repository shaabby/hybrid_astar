/**
 * @file ExperimentLogger.hpp
 * @brief 实验数据记录器定义
 *
 * 提供将规划实验结果追加到CSV文件的功能，
 * 用于批量实验和结果对比分析。
 */

#pragma once

#include "GridMap.hpp"
#include "HybridAstar.hpp"

#include <cstddef>
#include <filesystem>
#include <string>

/**
 * @brief 单条实验记录
 */
struct ExperimentLogEntry {
    std::string parameter_group; ///< 参数组名称
    std::string map_path;       ///< 地图文件路径
    bool success = false;       ///< 规划是否成功
    std::size_t path_poses = 0; ///< 路径点数
    std::size_t expanded_nodes = 0; ///< 扩展节点数
    int iterations = 0;         ///< 搜索循环迭代次数
    std::size_t generated_nodes = 0; ///< 生成节点数
    std::size_t open_remaining = 0;  ///< 结束时 open set 剩余条目数
    double runtime_ms = 0.0;   ///< 运行时间（毫秒）
    double heuristic_prepare_ms = 0.0;
    double search_loop_ms = 0.0;
    double obstacle_collect_ms = 0.0;
    double visibility_points_ms = 0.0;
    double visibility_graph_ms = 0.0;
    double visibility_dijkstra_ms = 0.0;
    double obstacle_lookup_ms = 0.0;
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
    std::string heuristic_name; ///< 使用的启发式名称
};

/**
 * @brief 实验数据记录器
 *
 * 将每次规划实验的关键指标追加到CSV文件，
 * 便于后续分析和批量对比。
 */
class ExperimentLogger {
public:
    /**
     * @brief 追加实验结果到CSV文件
     * @param[in] path   CSV文件路径
     * @param[in] entry  实验记录数据
     * @param[in] map    栅格地图
     * @param[in] config 规划器配置
     *
     * 如果文件不存在，会先写入表头行。
     */
    static void appendCsv(const std::filesystem::path& path,
                          const ExperimentLogEntry& entry,
                          const GridMap& map,
                          const HybridAstarConfig& config);
};