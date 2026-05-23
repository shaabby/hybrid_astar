#include "AppConfig.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>

namespace {

struct ParsedLine {
    int line_number = 0;
    std::string section;
    std::string key;
    std::string value;
};

struct ConfigSeenFields {
    std::unordered_set<std::string> top_level;
    std::unordered_set<std::string> vehicle;
    std::unordered_set<std::string> hybrid_astar;
};

std::string readTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open config yaml: " + path);
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
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

std::string stripComment(std::string_view line) {
    bool in_single_quote = false;
    bool in_double_quote = false;

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '\'' && !in_double_quote) {
            in_single_quote = !in_single_quote;
        } else if (ch == '"' && !in_single_quote) {
            in_double_quote = !in_double_quote;
        } else if (ch == '#' && !in_single_quote && !in_double_quote) {
            return trim(line.substr(0, i));
        }
    }

    return trim(line);
}

int indentWidth(std::string_view line) {
    int width = 0;
    for (const char ch : line) {
        if (ch == ' ') {
            ++width;
            continue;
        }
        if (ch == '\t') {
            throw std::runtime_error("YAML tabs are not supported");
        }
        break;
    }
    return width;
}

std::string unquote(std::string value) {
    if (value.size() < 2) {
        return value;
    }

    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string linePrefix(const ParsedLine& line) {
    return "line " + std::to_string(line.line_number) + " field "
        + (line.section.empty() ? line.key : line.section + "." + line.key);
}

double parseDouble(const ParsedLine& line) {
    const std::string value = unquote(line.value);
    double parsed = 0.0;
    const char* begin = value.data();
    const char* end = value.data() + value.size();
    const std::from_chars_result result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        throw std::runtime_error(linePrefix(line) + " expects a number");
    }
    return parsed;
}

int parseInt(const ParsedLine& line) {
    const std::string value = unquote(line.value);
    int parsed = 0;
    const char* begin = value.data();
    const char* end = value.data() + value.size();
    const std::from_chars_result result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        throw std::runtime_error(linePrefix(line) + " expects an integer");
    }
    return parsed;
}

bool parseBool(const ParsedLine& line) {
    std::string value = unquote(line.value);
    std::ranges::transform(value, value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (value == "true" || value == "1" || value == "yes") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no") {
        return false;
    }
    throw std::runtime_error(linePrefix(line) + " expects a bool");
}

void requireFields(const std::unordered_set<std::string>& seen,
                   std::initializer_list<std::string_view> required,
                   std::string_view prefix) {
    for (std::string_view key : required) {
        if (!seen.contains(std::string(key))) {
            throw std::runtime_error(
                "config yaml requires " + std::string(prefix)
                + std::string(key));
        }
    }
}

void validateRequiredFields(const ConfigSeenFields& seen) {
    requireFields(seen.top_level, {"map_path"}, "");
    requireFields(seen.vehicle,
                  {"length", "width", "wheelbase", "rear_to_center",
                   "max_steer"},
                  "vehicle.");
    requireFields(seen.hybrid_astar,
                  {"xy_resolution", "theta_bins", "step_size",
                   "primitive_length", "goal_xy_tolerance",
                   "goal_theta_tolerance", "reverse_penalty",
                   "steer_penalty", "gear_switch_penalty",
                   "steer_change_penalty", "max_iterations",
                   "allow_reverse", "enable_analytic_expansion",
                   "analytic_expansion_distance",
                   "analytic_expansion_interval",
                   "collision_safety_margin",
                   "enable_obstacle_heuristic",
                   "obstacle_lookup_resolution", "debug",
                   "debug_progress_interval"},
                  "hybrid_astar.");
}

void markSeen(ConfigSeenFields& seen, const ParsedLine& line) {
    if (line.section.empty()) {
        seen.top_level.insert(line.key);
    } else if (line.section == "vehicle") {
        seen.vehicle.insert(line.key);
    } else if (line.section == "hybrid_astar") {
        seen.hybrid_astar.insert(line.key);
    }
}

void applyTopLevel(AppConfig& config, const ParsedLine& line) {
    if (line.key == "map_path") {
        config.map_path = unquote(line.value);
        return;
    }
    if (line.key == "name") {
        return;
    }
    throw std::runtime_error(linePrefix(line) + " is not recognized");
}

void applyVehicle(VehicleConfig& config, const ParsedLine& line) {
    if (line.key == "length") {
        config.length = parseDouble(line);
    } else if (line.key == "width") {
        config.width = parseDouble(line);
    } else if (line.key == "wheelbase") {
        config.wheelbase = parseDouble(line);
    } else if (line.key == "rear_to_center") {
        config.rear_to_center = parseDouble(line);
    } else if (line.key == "max_steer") {
        config.max_steer = parseDouble(line);
    } else {
        throw std::runtime_error(linePrefix(line) + " is not recognized");
    }
}

void applyHybridAstar(HybridAstarConfig& config, const ParsedLine& line) {
    if (line.key == "xy_resolution") {
        config.xy_resolution = parseDouble(line);
    } else if (line.key == "theta_bins") {
        config.theta_bins = parseInt(line);
    } else if (line.key == "step_size") {
        config.step_size = parseDouble(line);
    } else if (line.key == "primitive_length") {
        config.primitive_length = parseDouble(line);
    } else if (line.key == "goal_xy_tolerance") {
        config.goal_xy_tolerance = parseDouble(line);
    } else if (line.key == "goal_theta_tolerance") {
        config.goal_theta_tolerance = parseDouble(line);
    } else if (line.key == "reverse_penalty") {
        config.reverse_penalty = parseDouble(line);
    } else if (line.key == "steer_penalty") {
        config.steer_penalty = parseDouble(line);
    } else if (line.key == "gear_switch_penalty") {
        config.gear_switch_penalty = parseDouble(line);
    } else if (line.key == "steer_change_penalty") {
        config.steer_change_penalty = parseDouble(line);
    } else if (line.key == "max_iterations") {
        config.max_iterations = parseInt(line);
    } else if (line.key == "allow_reverse") {
        config.allow_reverse = parseBool(line);
    } else if (line.key == "enable_analytic_expansion") {
        config.enable_analytic_expansion = parseBool(line);
    } else if (line.key == "analytic_expansion_distance") {
        config.analytic_expansion_distance = parseDouble(line);
    } else if (line.key == "analytic_expansion_interval") {
        config.analytic_expansion_interval = parseInt(line);
    } else if (line.key == "collision_safety_margin") {
        config.collision_safety_margin = parseDouble(line);
    } else if (line.key == "enable_obstacle_heuristic") {
        config.enable_obstacle_heuristic = parseBool(line);
    } else if (line.key == "obstacle_lookup_resolution") {
        config.obstacle_lookup_resolution = parseDouble(line);
    } else if (line.key == "debug") {
        config.debug = parseBool(line);
    } else if (line.key == "debug_progress_interval") {
        config.debug_progress_interval = parseInt(line);
    } else {
        throw std::runtime_error(linePrefix(line) + " is not recognized");
    }
}

void applyLine(AppConfig& config,
               ConfigSeenFields& seen,
               const ParsedLine& line) {
    if (line.section.empty()) {
        applyTopLevel(config, line);
    } else if (line.section == "vehicle") {
        applyVehicle(config.vehicle, line);
    } else if (line.section == "hybrid_astar") {
        applyHybridAstar(config.hybrid_astar, line);
    } else {
        throw std::runtime_error(linePrefix(line) + " has unknown section");
    }
    markSeen(seen, line);
}

} // namespace

AppConfig AppConfigLoader::loadYaml(const std::string& path) {
    const std::string text = readTextFile(path);
    AppConfig config;
    ConfigSeenFields seen;
    std::string current_section;

    std::size_t line_start = 0;
    int line_number = 0;
    while (line_start <= text.size()) {
        ++line_number;
        const std::size_t line_end = text.find('\n', line_start);
        const std::string_view raw_line(
            text.data() + line_start,
            (line_end == std::string::npos ? text.size() : line_end) - line_start);
        line_start = line_end == std::string::npos ? text.size() + 1 : line_end + 1;

        const std::string stripped = stripComment(raw_line);
        if (stripped.empty()) {
            continue;
        }

        const int indent = indentWidth(raw_line);
        if (indent != 0 && indent != 2) {
            throw std::runtime_error(
                "line " + std::to_string(line_number)
                + " uses unsupported indentation");
        }

        const std::size_t colon = stripped.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error(
                "line " + std::to_string(line_number) + " is missing ':'");
        }

        const std::string key = trim(std::string_view(stripped).substr(0, colon));
        const std::string value =
            trim(std::string_view(stripped).substr(colon + 1));
        if (key.empty()) {
            throw std::runtime_error(
                "line " + std::to_string(line_number) + " has an empty key");
        }

        if (indent == 0 && value.empty()) {
            if (key != "vehicle" && key != "hybrid_astar") {
                throw std::runtime_error(
                    "line " + std::to_string(line_number)
                    + " has unknown section: " + key);
            }
            current_section = key;
            continue;
        }

        if (value.empty()) {
            throw std::runtime_error(
                "line " + std::to_string(line_number) + " has an empty value");
        }

        ParsedLine parsed{
            .line_number = line_number,
            .section = indent == 0 ? std::string{} : current_section,
            .key = key,
            .value = value
        };
        if (indent != 0 && current_section.empty()) {
            throw std::runtime_error(
                "line " + std::to_string(line_number)
                + " is indented but no section is active");
        }
        applyLine(config, seen, parsed);
    }

    validateRequiredFields(seen);

    return config;
}
