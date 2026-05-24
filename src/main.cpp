#include "AppConfig.hpp"
#include "Car.hpp"
#include "ExperimentLogger.hpp"
#include "FltkViewer.hpp"
#include "GridMap.hpp"
#include "HtmlWriter.hpp"
#include "HybridAstar.hpp"
#include "JsonExporter.hpp"
#include "VisualizationData.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct AppOptions {
    std::string config_path;
    bool show_viewer = true;
};

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    output << text;
}

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [--no-view] config.yaml\n"
        << "       " << executable << " --help\n";
}

void debugStage(const std::string& message) {
    std::cerr << "[debug] " << message << '\n';
}

AppOptions parseOptions(int argc, char* argv[]) {
    AppOptions options;
    bool saw_config_path = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--no-view" || arg == "--html-only") {
            options.show_viewer = false;
            continue;
        }
        if (!arg.empty() && arg.front() == '-') {
            throw std::runtime_error("Unknown option: " + arg);
        }
        if (saw_config_path) {
            throw std::runtime_error("Only one config path can be specified");
        }
        options.config_path = arg;
        saw_config_path = true;
    }

    if (!saw_config_path) {
        throw std::runtime_error("Missing config yaml path");
    }

    return options;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        debugStage("parse command line options");
        const AppOptions options = parseOptions(argc, argv);
        debugStage("load config: " + options.config_path);
        const AppConfig app_config = AppConfigLoader::loadYaml(options.config_path);
        debugStage("load map: " + app_config.map_path);
        const GridMap map = MapLoader::loadJson(app_config.map_path);
        debugStage("construct vehicle model and planner");
        const Car car(app_config.vehicle);
        const HybridAstarConfig config = app_config.hybrid_astar;
        const HybridAstar planner(config);
        debugStage("run Hybrid A* planning");
        const auto plan_start = std::chrono::steady_clock::now();
        const PlanResult plan = planner.plan(map, car);
        const auto plan_end = std::chrono::steady_clock::now();

        debugStage("create output directory");
        std::filesystem::create_directories("output");
        debugStage("append experiment log");
        ExperimentLogEntry log_entry;
        log_entry.parameter_group = "single";
        log_entry.map_path = app_config.map_path;
        log_entry.success = plan.success;
        log_entry.path_poses = plan.path.size();
        log_entry.expanded_nodes = plan.expanded.size();
        log_entry.iterations = plan.iterations;
        log_entry.generated_nodes = plan.generated_nodes;
        log_entry.open_remaining = plan.open_remaining;
        log_entry.runtime_ms = std::chrono::duration<double, std::milli>(
            plan_end - plan_start).count();
        log_entry.heuristic_prepare_ms = plan.timing.heuristic_prepare_ms;
        log_entry.search_loop_ms = plan.timing.search_loop_ms;
        log_entry.obstacle_collect_ms = plan.timing.obstacle_collect_ms;
        log_entry.visibility_points_ms = plan.timing.visibility_points_ms;
        log_entry.visibility_graph_ms = plan.timing.visibility_graph_ms;
        log_entry.visibility_dijkstra_ms = plan.timing.visibility_dijkstra_ms;
        log_entry.obstacle_lookup_ms = plan.timing.obstacle_lookup_ms;
        log_entry.non_obstacle_heuristic_ms = plan.timing.non_obstacle_heuristic_ms;
        log_entry.obstacle_heuristic_ms = plan.timing.obstacle_heuristic_ms;
        log_entry.heuristic_estimate_calls = plan.timing.heuristic_estimate_calls;
        log_entry.primitive_collision_check_ms = plan.timing.primitive_collision_check_ms;
        log_entry.primitive_collision_check_calls = plan.timing.primitive_collision_check_calls;
        log_entry.analytic_expansion_ms = plan.timing.analytic_expansion_ms;
        log_entry.analytic_attempts = plan.timing.analytic_attempts;
        log_entry.analytic_successes = plan.timing.analytic_successes;
        log_entry.analytic_rs_generation_ms = plan.timing.analytic_rs_generation_ms;
        log_entry.analytic_rs_generation_calls = plan.timing.analytic_rs_generation_calls;
        log_entry.analytic_collision_check_ms = plan.timing.analytic_collision_check_ms;
        log_entry.analytic_collision_check_calls = plan.timing.analytic_collision_check_calls;
        log_entry.heuristic_name = planner.heuristicName();
        ExperimentLogger::appendCsv(
            "output/experiments.csv", log_entry, map, config);

        if (!plan.success) {
            throw std::runtime_error(
                "Hybrid A* failed to find a path; iterations="
                + std::to_string(plan.iterations)
                + ", expanded=" + std::to_string(plan.expanded.size())
                + ", generated_nodes=" + std::to_string(plan.generated_nodes)
                + ", open_remaining=" + std::to_string(plan.open_remaining));
        }

        debugStage("export path json");
        const std::string json = JsonExporter::exportPath(
            map, car, plan.path, plan.expanded);
        debugStage("write output files");
        writeTextFile("output/result.json", json);
        writeTextFile("output/demo.html", HtmlWriter::wrap(json));

        const Pose2D& start = map.start();
        const Pose2D& goal = map.goal();

        std::cout << "Loaded grid map\n";
        std::cout << "  config: " << options.config_path << '\n';
        std::cout << "  file: " << app_config.map_path << '\n';
        std::cout << "  size: " << map.width() << " x " << map.height() << '\n';
        std::cout << "  obstacles: " << map.obstacleCount() << '\n';
        std::cout << "  start: (" << start.x << ", " << start.y << ", "
                  << start.theta << ")\n";
        std::cout << "  goal: (" << goal.x << ", " << goal.y << ", " << goal.theta
                  << ")\n";
        std::cout << "Generated Hybrid A* path\n";
        std::cout << "  poses: " << plan.path.size() << '\n';
        std::cout << "  expanded: " << plan.expanded.size() << '\n';
        std::cout << "  iterations: " << plan.iterations << '\n';
        std::cout << "  generated_nodes: " << plan.generated_nodes << '\n';
        std::cout << "  runtime_ms: " << log_entry.runtime_ms << '\n';
        std::cout << "  output/result.json\n";
        std::cout << "  output/demo.html\n";
        std::cout << "  output/experiments.csv\n";

        if (options.show_viewer) {
            debugStage("open FLTK viewer");
            VisualizationData visualization{
                .map = map,
                .vehicle = car.config(),
                .path = plan.path,
                .expanded = plan.expanded
            };
            FltkViewer viewer(visualization);
            return viewer.run();
        }

        debugStage("finished without viewer");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
