#pragma once

#include "GridMap.hpp"
#include "HybridAstar.hpp"

#include <cstddef>
#include <filesystem>
#include <string>

struct ExperimentLogEntry {
    std::string map_path;
    bool success = false;
    std::size_t path_poses = 0;
    std::size_t expanded_nodes = 0;
    double runtime_ms = 0.0;
    std::string heuristic_name;
};

class ExperimentLogger {
public:
    static void appendCsv(const std::filesystem::path& path,
                          const ExperimentLogEntry& entry,
                          const GridMap& map,
                          const HybridAstarConfig& config);
};
