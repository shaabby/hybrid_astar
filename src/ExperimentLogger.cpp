/**
 * @file ExperimentLogger.cpp
 * @brief 实验数据记录器实现
 *
 * 将规划实验结果追加到CSV文件，包括地图路径、规划结果、
 * 运行时间和配置参数。
 */

#include "ExperimentLogger.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

/**
 * @brief 获取UTC时间戳字符串
 * @return ISO 8601格式的时间戳，如 "2026-05-22T19:00:00Z"
 */
std::string timestampUtc() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm utc_time{};
#if defined(_WIN32)
    gmtime_s(&utc_time, &now_time);
#else
    gmtime_r(&now_time, &utc_time);
#endif

    std::ostringstream out;
    out << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

/**
 * @brief 对CSV字段进行转义处理
 * @param[in] value 原始字段值
 * @return 转义后的字段值
 *
 * 如果字段包含引号、逗号或换行符，则用双引号包裹，
 * 并对内部引号进行双重化处理。
 */
std::string csvEscape(const std::string& value) {
    bool needs_quotes = false;
    for (char ch : value) {
        if (ch == '"' || ch == ',' || ch == '\n' || ch == '\r') {
            needs_quotes = true;
            break;
        }
    }

    if (!needs_quotes) {
        return value;
    }

    std::string escaped = "\"";
    for (char ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += '"';
    return escaped;
}

/**
 * @brief 检查文件是否需要写入表头
 * @param[in] path CSV文件路径
 * @return true如果文件不存在或为空，false否则
 */
bool shouldWriteHeader(const std::filesystem::path& path) {
    std::error_code error;
    return !std::filesystem::exists(path, error)
        || std::filesystem::file_size(path, error) == 0;
}

/**
 * @brief 写入CSV表头行
 * @param[out] output 输出流
 */
void writeHeader(std::ofstream& output) {
    output
        << "timestamp,"
        << "parameter_group,"
        << "map_path,"
        << "success,"
        << "path_poses,"
        << "expanded_nodes,"
        << "iterations,"
        << "generated_nodes,"
        << "open_remaining,"
        << "runtime_ms,"
        << "heuristic_prepare_ms,"
        << "search_loop_ms,"
        << "obstacle_collect_ms,"
        << "visibility_points_ms,"
        << "visibility_graph_ms,"
        << "visibility_dijkstra_ms,"
        << "obstacle_lookup_ms,"
        << "non_obstacle_heuristic_ms,"
        << "obstacle_heuristic_ms,"
        << "heuristic_estimate_calls,"
        << "primitive_collision_check_ms,"
        << "primitive_collision_check_calls,"
        << "analytic_expansion_ms,"
        << "analytic_attempts,"
        << "analytic_successes,"
        << "analytic_rs_generation_ms,"
        << "analytic_rs_generation_calls,"
        << "analytic_collision_check_ms,"
        << "analytic_collision_check_calls,"
        << "start_x,"
        << "start_y,"
        << "start_theta,"
        << "goal_x,"
        << "goal_y,"
        << "goal_theta,"
        << "max_iterations,"
        << "theta_bins,"
        << "reverse_penalty,"
        << "steer_penalty,"
        << "gear_switch_penalty,"
        << "steer_change_penalty,"
        << "analytic_expansion_distance,"
        << "analytic_expansion_interval,"
        << "heuristic_name,"
        << "enable_obstacle_heuristic,"
        << "enable_analytic_expansion\n";
}

} // namespace

/**
 * @brief 追加实验结果到CSV文件
 * @param[in] path   CSV文件路径
 * @param[in] entry  实验记录数据
 * @param[in] map    栅格地图
 * @param[in] config 规划器配置
 *
 * 如果文件所在目录不存在，会自动创建。
 * 如果文件不存在或为空，会先写入表头行。
 */
void ExperimentLogger::appendCsv(const std::filesystem::path& path,
                                 const ExperimentLogEntry& entry,
                                 const GridMap& map,
                                 const HybridAstarConfig& config) {
    // 确保父目录存在
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    const bool write_header = shouldWriteHeader(path);
    std::ofstream output(path, std::ios::app);
    if (!output) {
        throw std::runtime_error("Failed to write experiment log: "
                                 + path.string());
    }

    // 首次写入时输出表头
    if (write_header) {
        writeHeader(output);
    }

    const Pose2D& start = map.start();
    const Pose2D& goal = map.goal();

    // 写入数据行
    output << std::fixed << std::setprecision(6)
           << timestampUtc() << ','
           << csvEscape(entry.parameter_group) << ','
           << csvEscape(entry.map_path) << ','
           << (entry.success ? 1 : 0) << ','
           << entry.path_poses << ','
           << entry.expanded_nodes << ','
           << entry.iterations << ','
           << entry.generated_nodes << ','
           << entry.open_remaining << ','
           << entry.runtime_ms << ','
           << entry.heuristic_prepare_ms << ','
           << entry.search_loop_ms << ','
           << entry.obstacle_collect_ms << ','
           << entry.visibility_points_ms << ','
           << entry.visibility_graph_ms << ','
           << entry.visibility_dijkstra_ms << ','
           << entry.obstacle_lookup_ms << ','
           << entry.non_obstacle_heuristic_ms << ','
           << entry.obstacle_heuristic_ms << ','
           << entry.heuristic_estimate_calls << ','
           << entry.primitive_collision_check_ms << ','
           << entry.primitive_collision_check_calls << ','
           << entry.analytic_expansion_ms << ','
           << entry.analytic_attempts << ','
           << entry.analytic_successes << ','
           << entry.analytic_rs_generation_ms << ','
           << entry.analytic_rs_generation_calls << ','
           << entry.analytic_collision_check_ms << ','
           << entry.analytic_collision_check_calls << ','
           << start.x << ','
           << start.y << ','
           << start.theta << ','
           << goal.x << ','
           << goal.y << ','
           << goal.theta << ','
           << config.max_iterations << ','
           << config.theta_bins << ','
           << config.reverse_penalty << ','
           << config.steer_penalty << ','
           << config.gear_switch_penalty << ','
           << config.steer_change_penalty << ','
           << config.analytic_expansion_distance << ','
           << config.analytic_expansion_interval << ','
           << csvEscape(entry.heuristic_name) << ','
           << (config.enable_obstacle_heuristic ? 1 : 0) << ','
           << (config.enable_analytic_expansion ? 1 : 0)
           << '\n';
}