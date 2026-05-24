#include "AppConfig.hpp"
#include "Car.hpp"
#include "ExperimentLogger.hpp"
#include "HtmlWriter.hpp"
#include "HybridAstar.hpp"
#include "JsonExporter.hpp"
#include "GridMap.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::filesystem::path groups_path = "config/testbench/generated/groups.txt";
    std::filesystem::path maps_dir = "map";
    std::filesystem::path output_path = "output/testbench.csv";
    std::filesystem::path output_map_dir = "output/testbench_maps";
};

struct ParameterGroup {
    std::string name;
    std::filesystem::path config_path;
};

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [--groups groups.txt] [--maps map_dir] [--output result.csv] [--output-map-dir dir]\n"
        << "       " << executable << " --help\n";
}

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--groups" || arg == "--maps" || arg == "--output" || arg == "--output-map-dir") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for option: " + arg);
            }
            const std::filesystem::path value = argv[++i];
            if (arg == "--groups") {
                options.groups_path = value;
            } else if (arg == "--maps") {
                options.maps_dir = value;
            } else if (arg == "--output") {
                options.output_path = value;
            } else {
                options.output_map_dir = value;
            }
            continue;
        }
        throw std::runtime_error("Unknown option: " + arg);
    }
    return options;
}

std::vector<ParameterGroup> loadGroups(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open groups file: " + path.string());
    }

    std::vector<ParameterGroup> groups;
    std::string name;
    std::string config_path;
    while (input >> name >> config_path) {
        groups.push_back(ParameterGroup{name, config_path});
    }
    if (groups.empty()) {
        throw std::runtime_error("No parameter groups found: " + path.string());
    }
    return groups;
}

std::vector<std::filesystem::path> discoverMaps(const std::filesystem::path& maps_dir) {
    std::vector<std::filesystem::path> maps;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(maps_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            maps.push_back(entry.path());
        }
    }
    std::ranges::sort(maps);
    if (maps.empty()) {
        throw std::runtime_error("No .json maps found in: " + maps_dir.string());
    }
    return maps;
}

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    output << text;
}

void exportOutputMap(const std::filesystem::path& output_map_dir,
                     const std::string& group_name,
                     const std::filesystem::path& map_path,
                     const GridMap& map,
                     const Car& car,
                     const PlanResult& plan) {
    const std::filesystem::path base_path =
        output_map_dir / group_name / map_path.stem();
    const std::string json = JsonExporter::exportPath(
        map, car, plan.path, plan.expanded);
    writeTextFile(base_path.string() + ".json", json);
    writeTextFile(base_path.string() + ".html", HtmlWriter::wrap(json));
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseOptions(argc, argv);
        const std::vector<ParameterGroup> groups = loadGroups(options.groups_path);
        const std::vector<std::filesystem::path> maps = discoverMaps(options.maps_dir);

        int total_runs = 0;
        int success_runs = 0;
        int failed_runs = 0;

        for (const ParameterGroup& group : groups) {
            const AppConfig group_config = AppConfigLoader::loadYaml(group.config_path.string());
            const Car car(group_config.vehicle);

            for (const std::filesystem::path& map_path : maps) {
                ++total_runs;
                try {
                    const GridMap map = MapLoader::loadJson(map_path.string());
                    const HybridAstar planner(group_config.hybrid_astar);

                    const auto plan_start = std::chrono::steady_clock::now();
                    const PlanResult plan = planner.plan(map, car);
                    const auto plan_end = std::chrono::steady_clock::now();

                    ExperimentLogEntry log_entry;
                    log_entry.parameter_group = group.name;
                    log_entry.map_path = map_path.string();
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
                        options.output_path, log_entry, map, group_config.hybrid_astar);
                    exportOutputMap(
                        options.output_map_dir, group.name, map_path, map, car, plan);

                    if (plan.success) {
                        ++success_runs;
                    } else {
                        ++failed_runs;
                    }

                    std::cout << '[' << total_runs << "] "
                              << group.name << ' ' << map_path.string()
                              << " success=" << (plan.success ? 1 : 0)
                              << " runtime_ms=" << log_entry.runtime_ms
                              << " expanded=" << log_entry.expanded_nodes << '\n';
                } catch (const std::exception& error) {
                    ++failed_runs;
                    std::cerr << "Run failed: group=" << group.name
                              << " map=" << map_path.string()
                              << " error=" << error.what() << '\n';
                }
            }
        }

        std::cout << "Testbench finished\n"
                  << "  groups: " << groups.size() << '\n'
                  << "  maps: " << maps.size() << '\n'
                  << "  total_runs: " << total_runs << '\n'
                  << "  success_runs: " << success_runs << '\n'
                  << "  failed_runs: " << failed_runs << '\n'
                  << "  output: " << options.output_path.string() << '\n'
                  << "  output_map_dir: " << options.output_map_dir.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
