#include "ExperimentLogger.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

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

bool shouldWriteHeader(const std::filesystem::path& path) {
    std::error_code error;
    return !std::filesystem::exists(path, error)
        || std::filesystem::file_size(path, error) == 0;
}

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

void ExperimentLogger::appendCsv(const std::filesystem::path& path,
                                 const ExperimentLogEntry& entry,
                                 const GridMap& map,
                                 const HybridAstarConfig& config) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    const bool write_header = shouldWriteHeader(path);
    std::ofstream output(path, std::ios::app);
    if (!output) {
        throw std::runtime_error("Failed to write experiment log: "
                                 + path.string());
    }

    if (write_header) {
        writeHeader(output);
    }

    const Pose2D& start = map.start();
    const Pose2D& goal = map.goal();

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
