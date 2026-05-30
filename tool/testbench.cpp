#include "AppConfig.hpp"
#include "Car.hpp"
#include "ExperimentLogger.hpp"
#include "HybridAstar.hpp"
#include "JsonExporter.hpp"
#include "GridMap.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Options {
    std::optional<std::filesystem::path> groups_path;
    std::optional<std::filesystem::path> base_config_path;
    std::filesystem::path maps_dir = "map";
    std::filesystem::path output_path = "output/testbench.csv";
    std::filesystem::path output_map_dir = "output/testbench_maps";
    std::vector<std::pair<std::string, std::vector<std::string>>> params;
};

struct ParameterGroup {
    std::string name;
    AppConfig config;
};

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [--groups groups.txt] [--maps map_dir] [--output result.csv] [--output-map-dir dir]\n"
        << "       " << executable << " --base-config config.yaml --param key=v1,v2,... [--param key=v1,v2,...] [--maps map_dir] [--output result.csv] [--output-map-dir dir]\n"
        << "       " << executable << " --help\n";
}

std::vector<std::string> split(std::string_view text, char delimiter) {
    std::vector<std::string> parts;
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const std::size_t end = text.find(delimiter, begin);
        const std::size_t stop = end == std::string_view::npos ? text.size() : end;
        parts.emplace_back(text.substr(begin, stop - begin));
        if (end == std::string_view::npos) {
            break;
        }
        begin = end + 1;
    }
    return parts;
}

std::string trim(std::string_view text) {
    std::size_t begin = 0;
    while (begin < text.size()
           && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
        ++begin;
    }

    std::size_t end = text.size();
    while (end > begin
           && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }

    return std::string(text.substr(begin, end - begin));
}

double parseDouble(std::string_view value, const std::string& key) {
    std::stringstream input{std::string(value)};
    double parsed = 0.0;
    input >> parsed;
    if (!input || !input.eof()) {
        throw std::runtime_error("Parameter " + key + " expects a number");
    }
    return parsed;
}

int parseInt(std::string_view value, const std::string& key) {
    std::stringstream input{std::string(value)};
    int parsed = 0;
    input >> parsed;
    if (!input || !input.eof()) {
        throw std::runtime_error("Parameter " + key + " expects an integer");
    }
    return parsed;
}

bool parseBool(std::string value, const std::string& key) {
    std::ranges::transform(value, value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (value == "true" || value == "1" || value == "yes") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no") {
        return false;
    }
    throw std::runtime_error("Parameter " + key + " expects a bool");
}

ObstacleHeuristicType parseObstacleHeuristicType(std::string value,
                                                 const std::string& key) {
    std::ranges::transform(value, value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (value == "visibility_graph") {
        return ObstacleHeuristicType::VisibilityGraph;
    }
    if (value == "reverse_dijkstra") {
        return ObstacleHeuristicType::ReverseDijkstra;
    }
    throw std::runtime_error(
        "Parameter " + key + " expects visibility_graph or reverse_dijkstra");
}

std::string slugValue(std::string value) {
    for (char& ch : value) {
        if (std::isalnum(static_cast<unsigned char>(ch)) == 0) {
            ch = '_';
        }
    }
    return value;
}

void applyOverride(AppConfig& config,
                   const std::string& key,
                   const std::string& raw_value) {
    const std::string value = trim(raw_value);
    if (key == "map_path") {
        config.map_path = value;
    } else if (key == "vehicle.length") {
        config.vehicle.length = parseDouble(value, key);
    } else if (key == "vehicle.width") {
        config.vehicle.width = parseDouble(value, key);
    } else if (key == "vehicle.wheelbase") {
        config.vehicle.wheelbase = parseDouble(value, key);
    } else if (key == "vehicle.rear_to_center") {
        config.vehicle.rear_to_center = parseDouble(value, key);
    } else if (key == "vehicle.max_steer") {
        config.vehicle.max_steer = parseDouble(value, key);
    } else if (key == "hybrid_astar.xy_resolution") {
        config.hybrid_astar.xy_resolution = parseDouble(value, key);
    } else if (key == "hybrid_astar.theta_bins") {
        config.hybrid_astar.theta_bins = parseInt(value, key);
    } else if (key == "hybrid_astar.step_size") {
        config.hybrid_astar.step_size = parseDouble(value, key);
    } else if (key == "hybrid_astar.primitive_length") {
        config.hybrid_astar.primitive_length = parseDouble(value, key);
    } else if (key == "hybrid_astar.goal_xy_tolerance") {
        config.hybrid_astar.goal_xy_tolerance = parseDouble(value, key);
    } else if (key == "hybrid_astar.goal_theta_tolerance") {
        config.hybrid_astar.goal_theta_tolerance = parseDouble(value, key);
    } else if (key == "hybrid_astar.reverse_penalty") {
        config.hybrid_astar.reverse_penalty = parseDouble(value, key);
    } else if (key == "hybrid_astar.steer_penalty") {
        config.hybrid_astar.steer_penalty = parseDouble(value, key);
    } else if (key == "hybrid_astar.gear_switch_penalty") {
        config.hybrid_astar.gear_switch_penalty = parseDouble(value, key);
    } else if (key == "hybrid_astar.steer_change_penalty") {
        config.hybrid_astar.steer_change_penalty = parseDouble(value, key);
    } else if (key == "hybrid_astar.max_iterations") {
        config.hybrid_astar.max_iterations = parseInt(value, key);
    } else if (key == "hybrid_astar.allow_reverse") {
        config.hybrid_astar.allow_reverse = parseBool(value, key);
    } else if (key == "hybrid_astar.enable_analytic_expansion") {
        config.hybrid_astar.enable_analytic_expansion = parseBool(value, key);
    } else if (key == "hybrid_astar.analytic_expansion_distance") {
        config.hybrid_astar.analytic_expansion_distance = parseDouble(value, key);
    } else if (key == "hybrid_astar.analytic_expansion_interval") {
        config.hybrid_astar.analytic_expansion_interval = parseInt(value, key);
    } else if (key == "hybrid_astar.collision_safety_margin") {
        config.hybrid_astar.collision_safety_margin = parseDouble(value, key);
    } else if (key == "hybrid_astar.enable_obstacle_heuristic") {
        config.hybrid_astar.enable_obstacle_heuristic = parseBool(value, key);
    } else if (key == "hybrid_astar.obstacle_lookup_resolution") {
        config.hybrid_astar.obstacle_lookup_resolution = parseDouble(value, key);
    } else if (key == "hybrid_astar.obstacle_heuristic_type") {
        config.hybrid_astar.obstacle_heuristic_type =
            parseObstacleHeuristicType(value, key);
    } else if (key == "hybrid_astar.obstacle_heuristic_inflation_alpha") {
        config.hybrid_astar.obstacle_heuristic_inflation_alpha =
            parseDouble(value, key);
    } else if (key == "hybrid_astar.enable_timing") {
        config.hybrid_astar.enable_timing = parseBool(value, key);
    } else if (key == "hybrid_astar.debug") {
        config.hybrid_astar.debug = parseBool(value, key);
    } else if (key == "hybrid_astar.debug_progress_interval") {
        config.hybrid_astar.debug_progress_interval = parseInt(value, key);
    } else {
        throw std::runtime_error("Unknown override parameter: " + key);
    }
}

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--groups" || arg == "--base-config"
            || arg == "--maps" || arg == "--output"
            || arg == "--output-map-dir" || arg == "--param") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for option: " + arg);
            }
            const std::string raw_value = argv[++i];
            if (arg == "--groups") {
                options.groups_path = std::filesystem::path(raw_value);
            } else if (arg == "--base-config") {
                options.base_config_path = std::filesystem::path(raw_value);
            } else if (arg == "--maps") {
                options.maps_dir = std::filesystem::path(raw_value);
            } else if (arg == "--output") {
                options.output_path = std::filesystem::path(raw_value);
            } else if (arg == "--output-map-dir") {
                options.output_map_dir = std::filesystem::path(raw_value);
            } else {
                const std::size_t equal = raw_value.find('=');
                if (equal == std::string::npos) {
                    throw std::runtime_error(
                        "--param expects key=v1,v2,... but got: " + raw_value);
                }
                const std::string key = trim(raw_value.substr(0, equal));
                const std::string values_text = raw_value.substr(equal + 1);
                if (key.empty() || values_text.empty()) {
                    throw std::runtime_error(
                        "--param expects key=v1,v2,... but got: " + raw_value);
                }
                std::vector<std::string> values;
                for (const std::string& item : split(values_text, ',')) {
                    const std::string value = trim(item);
                    if (value.empty()) {
                        throw std::runtime_error(
                            "--param has an empty value: " + raw_value);
                    }
                    values.push_back(value);
                }
                options.params.push_back({key, values});
            }
            continue;
        }
        throw std::runtime_error("Unknown option: " + arg);
    }

    if (options.groups_path.has_value() == options.base_config_path.has_value()) {
        throw std::runtime_error(
            "Specify exactly one of --groups or --base-config");
    }
    if (options.groups_path && !options.params.empty()) {
        throw std::runtime_error("--param cannot be used together with --groups");
    }
    if (options.base_config_path && options.params.empty()) {
        throw std::runtime_error(
            "--base-config requires at least one --param key=v1,v2,...");
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
        groups.push_back(ParameterGroup{
            name,
            AppConfigLoader::loadYaml(config_path)
        });
    }
    if (groups.empty()) {
        throw std::runtime_error("No parameter groups found: " + path.string());
    }
    return groups;
}

void buildGeneratedGroups(const AppConfig& base_config,
                          const std::vector<std::pair<std::string, std::vector<std::string>>>& params,
                          std::size_t index,
                          AppConfig current_config,
                          std::string current_name,
                          std::vector<ParameterGroup>& groups) {
    if (index >= params.size()) {
        groups.push_back(ParameterGroup{
            current_name.empty() ? "base" : current_name,
            std::move(current_config)
        });
        return;
    }

    const auto& [key, values] = params[index];
    for (const std::string& value : values) {
        AppConfig next_config = current_config;
        applyOverride(next_config, key, value);

        std::string next_name = current_name;
        if (!next_name.empty()) {
            next_name += "__";
        }
        next_name += key;
        next_name += "_";
        next_name += slugValue(value);

        buildGeneratedGroups(
            base_config, params, index + 1, std::move(next_config),
            std::move(next_name), groups);
    }
}

std::vector<ParameterGroup> generateGroups(
    const std::filesystem::path& base_config_path,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& params) {
    const AppConfig base_config = AppConfigLoader::loadYaml(base_config_path.string());
    std::vector<ParameterGroup> groups;
    buildGeneratedGroups(base_config, params, 0, base_config, "", groups);
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
        map, car, plan.path, plan.expanded, plan.search_tree,
        plan.solution_node_ids, plan.solution_open_orders,
        plan.solution_close_orders, plan.solution_pop_orders,
            plan.solution_path_frame_starts);
    writeTextFile(base_path.string() + ".json", json);
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseOptions(argc, argv);
        const std::vector<ParameterGroup> groups = options.groups_path
            ? loadGroups(*options.groups_path)
            : generateGroups(*options.base_config_path, options.params);
        const std::vector<std::filesystem::path> maps = discoverMaps(options.maps_dir);

        int total_runs = 0;
        int success_runs = 0;
        int failed_runs = 0;

        for (const ParameterGroup& group : groups) {
            const Car car(group.config.vehicle);

            for (const std::filesystem::path& map_path : maps) {
                ++total_runs;
                try {
                    const GridMap map = MapLoader::loadJson(map_path.string());
                    const HybridAstar planner(group.config.hybrid_astar);

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
                    log_entry.reverse_dijkstra_inflation_ms = plan.timing.reverse_dijkstra_inflation_ms;
                    log_entry.reverse_dijkstra_ms = plan.timing.reverse_dijkstra_ms;
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
                        options.output_path, log_entry, map, group.config.hybrid_astar);
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
