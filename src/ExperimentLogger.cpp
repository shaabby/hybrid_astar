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
        << "map_path,"
        << "success,"
        << "path_poses,"
        << "expanded_nodes,"
        << "runtime_ms,"
        << "start_x,"
        << "start_y,"
        << "start_theta,"
        << "goal_x,"
        << "goal_y,"
        << "goal_theta,"
        << "reverse_penalty,"
        << "steer_penalty,"
        << "gear_switch_penalty,"
        << "steer_change_penalty,"
        << "heuristic_name,"
        << "enable_obstacle_heuristic,"
        << "obstacle_heuristic_inflate_alpha,"
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
           << csvEscape(entry.map_path) << ','
           << (entry.success ? 1 : 0) << ','
           << entry.path_poses << ','
           << entry.expanded_nodes << ','
           << entry.runtime_ms << ','
           << start.x << ','
           << start.y << ','
           << start.theta << ','
           << goal.x << ','
           << goal.y << ','
           << goal.theta << ','
           << config.reverse_penalty << ','
           << config.steer_penalty << ','
           << config.gear_switch_penalty << ','
           << config.steer_change_penalty << ','
           << csvEscape(entry.heuristic_name) << ','
           << (config.enable_obstacle_heuristic ? 1 : 0) << ','
           << config.obstacle_heuristic_inflate_alpha << ','
           << (config.enable_analytic_expansion ? 1 : 0)
           << '\n';
}