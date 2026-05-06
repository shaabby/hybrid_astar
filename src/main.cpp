#include "Car.hpp"
#include "ExperimentLogger.hpp"
#include "GridMap.hpp"
#include "HtmlWriter.hpp"
#include "HybridAstar.hpp"
#include "JsonExporter.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    output << text;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string map_path = argc > 1
                                         ? argv[1]
                                         : "map/default_map.json";
        const GridMap map = MapLoader::loadJson(map_path);
        const Car car;
        const HybridAstarConfig config;
        const HybridAstar planner(config);
        const auto plan_start = std::chrono::steady_clock::now();
        const PlanResult plan = planner.plan(map, car);
        const auto plan_end = std::chrono::steady_clock::now();

        std::filesystem::create_directories("output");
        ExperimentLogEntry log_entry;
        log_entry.map_path = map_path;
        log_entry.success = plan.success;
        log_entry.path_poses = plan.path.size();
        log_entry.expanded_nodes = plan.expanded.size();
        log_entry.runtime_ms = std::chrono::duration<double, std::milli>(
            plan_end - plan_start).count();
        log_entry.heuristic_name = planner.heuristicName();
        ExperimentLogger::appendCsv(
            "output/experiments.csv", log_entry, map, config);

        if (!plan.success) {
            throw std::runtime_error("Hybrid A* failed to find a path");
        }

        const std::string json = JsonExporter::exportPath(
            map, car, plan.path, plan.expanded);
        writeTextFile("output/result.json", json);
        writeTextFile("output/demo.html", HtmlWriter::wrap(json));

        const Pose2D& start = map.start();
        const Pose2D& goal = map.goal();

        std::cout << "Loaded grid map\n";
        std::cout << "  file: " << map_path << '\n';
        std::cout << "  size: " << map.width() << " x " << map.height() << '\n';
        std::cout << "  obstacles: " << map.obstacleCount() << '\n';
        std::cout << "  start: (" << start.x << ", " << start.y << ", "
                  << start.theta << ")\n";
        std::cout << "  goal: (" << goal.x << ", " << goal.y << ", " << goal.theta
                  << ")\n";
        std::cout << "Generated Hybrid A* path\n";
        std::cout << "  poses: " << plan.path.size() << '\n';
        std::cout << "  expanded: " << plan.expanded.size() << '\n';
        std::cout << "  runtime_ms: " << log_entry.runtime_ms << '\n';
        std::cout << "  output/result.json\n";
        std::cout << "  output/demo.html\n";
        std::cout << "  output/experiments.csv\n";

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
