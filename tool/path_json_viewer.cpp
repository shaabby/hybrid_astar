#include "FltkViewer.hpp"
#include "GridMap.hpp"
#include "HybridAstar.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct PathRecord {
    std::string name;
    std::string word;
    std::vector<CarPose> samples;
};

struct LoadedResult {
    GridMap map;
    VehicleConfig vehicle;
    std::vector<PathRecord> paths;
    std::vector<SearchTreeEdge> search_tree;
    std::vector<int> solution_node_ids;
    std::vector<int> solution_open_orders;
    std::vector<int> solution_close_orders;
    std::vector<int> solution_path_frame_starts;
};

struct ToolOptions {
    std::string json_path = "output/result.json";
    std::optional<std::string> path_selector;
    bool list_paths = false;
};

std::string readTextFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open JSON file: " + path.string());
    }

    return {
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()
    };
}

std::size_t skipWhitespace(const std::string& text, std::size_t pos) {
    while (pos < text.size()
           && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }
    return pos;
}

std::optional<std::size_t> findFieldValue(const std::string& object,
                                          const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    std::size_t pos = 0;
    while ((pos = object.find(needle, pos)) != std::string::npos) {
        std::size_t colon = skipWhitespace(object, pos + needle.size());
        if (colon < object.size() && object[colon] == ':') {
            return skipWhitespace(object, colon + 1);
        }
        pos += needle.size();
    }
    return std::nullopt;
}

double readNumberField(const std::string& object,
                       const std::string& key,
                       double fallback,
                       bool required) {
    const std::optional<std::size_t> pos = findFieldValue(object, key);
    if (!pos) {
        if (required) {
            throw std::runtime_error("Missing numeric field: " + key);
        }
        return fallback;
    }

    const char* start = object.c_str() + *pos;
    char* end = nullptr;
    const double value = std::strtod(start, &end);
    if (end == start) {
        throw std::runtime_error("Invalid numeric field: " + key);
    }
    return value;
}

int readIntField(const std::string& object,
                 const std::string& key,
                 int fallback,
                 bool required) {
    return static_cast<int>(readNumberField(object, key, fallback, required));
}

bool readBoolField(const std::string& object,
                   const std::string& key,
                   bool fallback,
                   bool required) {
    const std::optional<std::size_t> pos = findFieldValue(object, key);
    if (!pos) {
        if (required) {
            throw std::runtime_error("Missing boolean field: " + key);
        }
        return fallback;
    }
    if (object.compare(*pos, 4, "true") == 0) {
        return true;
    }
    if (object.compare(*pos, 5, "false") == 0) {
        return false;
    }
    throw std::runtime_error("Invalid boolean field: " + key);
}

std::optional<std::string> readStringFieldOptional(const std::string& object,
                                                   const std::string& key) {
    const std::optional<std::size_t> value_pos = findFieldValue(object, key);
    if (!value_pos || *value_pos >= object.size() || object[*value_pos] != '"') {
        return std::nullopt;
    }

    std::string value;
    for (std::size_t i = *value_pos + 1; i < object.size(); ++i) {
        if (object[i] == '\\' && i + 1 < object.size()) {
            value.push_back(object[i + 1]);
            ++i;
            continue;
        }
        if (object[i] == '"') {
            return value;
        }
        value.push_back(object[i]);
    }
    throw std::runtime_error("Unclosed string field: " + key);
}

std::string readCompoundField(const std::string& json,
                              const std::string& key,
                              char open,
                              char close,
                              const std::string& label) {
    const std::optional<std::size_t> value_pos = findFieldValue(json, key);
    if (!value_pos || *value_pos >= json.size() || json[*value_pos] != open) {
        throw std::runtime_error("Missing " + label + " field: " + key);
    }

    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = *value_pos; i < json.size(); ++i) {
        const char c = json[i];
        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
        } else if (c == open) {
            ++depth;
        } else if (c == close) {
            --depth;
            if (depth == 0) {
                return json.substr(*value_pos, i - *value_pos + 1);
            }
        }
    }

    throw std::runtime_error("Unclosed " + label + " field: " + key);
}

std::string readObjectField(const std::string& json, const std::string& key) {
    return readCompoundField(json, key, '{', '}', "object");
}

std::optional<std::string> readObjectFieldOptional(const std::string& json,
                                                   const std::string& key) {
    try {
        return readObjectField(json, key);
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }
}

std::string readArrayField(const std::string& json, const std::string& key) {
    return readCompoundField(json, key, '[', ']', "array");
}

std::optional<std::string> readArrayFieldOptional(const std::string& json,
                                                  const std::string& key) {
    try {
        return readArrayField(json, key);
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }
}

std::vector<std::string> readObjectsInArray(const std::string& array_text) {
    std::vector<std::string> objects;
    int depth = 0;
    std::size_t start = std::string::npos;

    for (std::size_t i = 0; i < array_text.size(); ++i) {
        if (array_text[i] == '{') {
            if (depth == 0) {
                start = i;
            }
            ++depth;
        } else if (array_text[i] == '}') {
            --depth;
            if (depth == 0 && start != std::string::npos) {
                objects.push_back(array_text.substr(start, i - start + 1));
                start = std::string::npos;
            }
        }
    }

    if (depth != 0) {
        throw std::runtime_error("Malformed object array");
    }
    return objects;
}

Pose2D readPose2D(const std::string& object) {
    return {
        readNumberField(object, "x", 0.0, true),
        readNumberField(object, "y", 0.0, true),
        readNumberField(object, "theta", 0.0, false)
    };
}

CarPose readCarPose(const std::string& object) {
    return {
        .x = readNumberField(object, "x", 0.0, true),
        .y = readNumberField(object, "y", 0.0, true),
        .theta = readNumberField(object, "theta", 0.0, false),
        .steer = readNumberField(object, "steer", 0.0, false),
        .direction = readIntField(object, "direction", 1, false)
    };
}

std::vector<CarPose> readPoseArray(const std::string& array_text) {
    std::vector<CarPose> poses;
    for (const std::string& object : readObjectsInArray(array_text)) {
        poses.push_back(readCarPose(object));
    }
    return poses;
}

std::vector<int> readIntArray(const std::string& array_text) {
    std::vector<int> values;
    std::size_t pos = 0;
    while (pos < array_text.size()) {
        pos = skipWhitespace(array_text, pos);
        if (pos >= array_text.size() || array_text[pos] == ']') {
            break;
        }
        if (array_text[pos] == '[' || array_text[pos] == ',') {
            ++pos;
            continue;
        }

        const char* start = array_text.c_str() + pos;
        char* end = nullptr;
        const long value = std::strtol(start, &end, 10);
        if (end == start) {
            throw std::runtime_error("Invalid integer array value");
        }
        values.push_back(static_cast<int>(value));
        pos = static_cast<std::size_t>(end - array_text.c_str());
    }
    return values;
}

SearchTreeEdge readSearchTreeEdge(const std::string& object) {
    SearchTreeEdge edge;
    edge.parent = readIntField(object, "parent", -1, true);
    edge.child = readIntField(object, "child", -1, false);
    edge.open_order = readIntField(object, "open_order", -1, false);
    edge.close_order = readIntField(object, "close_order", -1, false);
    edge.accepted = readBoolField(object, "accepted", false, false);
    edge.collision = readBoolField(object, "collision", false, false);
    edge.duplicate = readBoolField(object, "duplicate", false, false);
    edge.in_solution = readBoolField(object, "in_solution", false, false);
    if (const std::optional<std::string> from =
            readObjectFieldOptional(object, "from")) {
        edge.from = readCarPose(*from);
    }
    if (const std::optional<std::string> to =
            readObjectFieldOptional(object, "to")) {
        edge.to = readCarPose(*to);
    }
    if (const std::optional<std::string> segment =
            readArrayFieldOptional(object, "segment")) {
        edge.segment = readPoseArray(*segment);
    }
    return edge;
}

std::vector<SearchTreeEdge> readSearchTreeArray(
    const std::string& array_text) {
    std::vector<SearchTreeEdge> edges;
    for (const std::string& object : readObjectsInArray(array_text)) {
        edges.push_back(readSearchTreeEdge(object));
    }
    return edges;
}

std::filesystem::path sourcePath(const std::string& path) {
    std::filesystem::path candidate(path);
    if (candidate.is_absolute()) {
        return candidate;
    }
    return std::filesystem::path(HYBRID_ASTAR_SOURCE_DIR) / candidate;
}

GridMap readMapObject(const std::string& map_object) {
    if (const std::optional<std::string> source =
            readStringFieldOptional(map_object, "source")) {
        GridMap sourced_map = MapLoader::loadJson(sourcePath(*source).string());
        if (const std::optional<std::string> start =
                readObjectFieldOptional(map_object, "start")) {
            sourced_map.setStart(readPose2D(*start));
        }
        if (const std::optional<std::string> goal =
                readObjectFieldOptional(map_object, "goal")) {
            sourced_map.setGoal(readPose2D(*goal));
        }
        return sourced_map;
    }

    const int width = readIntField(map_object, "width", 0, true);
    const int height = readIntField(map_object, "height", 0, true);
    GridMap map(width, height);

    if (const std::optional<std::string> start =
            readObjectFieldOptional(map_object, "start")) {
        map.setStart(readPose2D(*start));
    }
    if (const std::optional<std::string> goal =
            readObjectFieldOptional(map_object, "goal")) {
        map.setGoal(readPose2D(*goal));
    }

    if (const std::optional<std::string> obstacles =
            readArrayFieldOptional(map_object, "obstacles")) {
        for (const std::string& object : readObjectsInArray(*obstacles)) {
            const int x0 = readIntField(object, "x", 0, true);
            const int y0 = readIntField(object, "y", 0, true);
            const int w = std::max(1, readIntField(object, "w", 1, false));
            const int h = std::max(1, readIntField(object, "h", 1, false));
            for (int y = y0; y < y0 + h; ++y) {
                for (int x = x0; x < x0 + w; ++x) {
                    map.setObstacle(x, y, true);
                }
            }
        }
    }

    return map;
}

VehicleConfig readVehicleObject(const std::string& vehicle_object) {
    VehicleConfig vehicle;
    vehicle.length = readNumberField(vehicle_object, "length",
                                     vehicle.length, false);
    vehicle.width = readNumberField(vehicle_object, "width",
                                    vehicle.width, false);
    vehicle.wheelbase = readNumberField(vehicle_object, "wheelbase",
                                        vehicle.wheelbase, false);
    vehicle.rear_to_center = readNumberField(
        vehicle_object, "rearToCenter", vehicle.rear_to_center, false);
    vehicle.rear_to_center = readNumberField(
        vehicle_object, "rear_to_center", vehicle.rear_to_center, false);
    vehicle.max_steer = readNumberField(vehicle_object, "maxSteer",
                                        vehicle.max_steer, false);
    vehicle.max_steer = readNumberField(vehicle_object, "max_steer",
                                        vehicle.max_steer, false);
    return vehicle;
}

PathRecord readPathObject(const std::string& path_object, int index) {
    const std::string samples = readArrayField(path_object, "samples");
    PathRecord path;
    path.name = readStringFieldOptional(path_object, "name")
        .value_or("path " + std::to_string(index + 1));
    path.word = readStringFieldOptional(path_object, "word").value_or("");
    path.samples = readPoseArray(samples);
    return path;
}

LoadedResult loadResultJson(const std::filesystem::path& json_path) {
    const std::string json = readTextFile(json_path);
    LoadedResult result;
    result.map = readMapObject(readObjectField(json, "map"));

    if (const std::optional<std::string> vehicle =
            readObjectFieldOptional(json, "vehicle")) {
        result.vehicle = readVehicleObject(*vehicle);
    }

    if (const std::optional<std::string> path =
            readArrayFieldOptional(json, "path")) {
        result.paths.push_back({
            .name = "planner path",
            .word = "",
            .samples = readPoseArray(*path)
        });
    }

    if (const std::optional<std::string> paths =
            readArrayFieldOptional(json, "paths")) {
        int index = 0;
        for (const std::string& object : readObjectsInArray(*paths)) {
            result.paths.push_back(readPathObject(object, index));
            ++index;
        }
    }

    if (const std::optional<std::string> solution_node_ids =
            readArrayFieldOptional(json, "solution_node_ids")) {
        result.solution_node_ids = readIntArray(*solution_node_ids);
    }

    if (const std::optional<std::string> solution_open_orders =
            readArrayFieldOptional(json, "solution_open_orders")) {
        result.solution_open_orders = readIntArray(*solution_open_orders);
    }

    if (const std::optional<std::string> solution_close_orders =
            readArrayFieldOptional(json, "solution_close_orders")) {
        result.solution_close_orders = readIntArray(*solution_close_orders);
    }

    if (const std::optional<std::string> solution_path_frame_starts =
            readArrayFieldOptional(json, "solution_path_frame_starts")) {
        result.solution_path_frame_starts =
            readIntArray(*solution_path_frame_starts);
    }

    if (const std::optional<std::string> search_tree =
            readArrayFieldOptional(json, "search_tree")) {
        result.search_tree = readSearchTreeArray(*search_tree);
    }

    if (result.paths.empty()) {
        throw std::runtime_error(
            "JSON does not contain path or paths[].samples");
    }

    return result;
}

std::size_t selectPath(const LoadedResult& result,
                       const std::optional<std::string>& selector) {
    if (!selector) {
        return 0;
    }

    try {
        std::size_t consumed = 0;
        const std::size_t index = static_cast<std::size_t>(
            std::stoul(*selector, &consumed));
        if (consumed == selector->size() && index < result.paths.size()) {
            return index;
        }
    } catch (const std::exception&) {
    }

    for (std::size_t i = 0; i < result.paths.size(); ++i) {
        if (result.paths[i].name == *selector
            || result.paths[i].word == *selector) {
            return i;
        }
    }

    throw std::runtime_error("Path selector did not match any path: "
                             + *selector);
}

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [result.json] [--path name|index]\n"
        << "       " << executable << " [result.json] --list\n"
        << "       " << executable << " --help\n\n"
        << "Examples:\n"
        << "  " << executable << " output/result.json\n"
        << "  " << executable
        << " output/reeds_shepp_empty_map_test.json --path map_start_to_goal\n";
}

ToolOptions parseOptions(int argc, char* argv[]) {
    ToolOptions options;
    bool saw_json_path = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--list") {
            options.list_paths = true;
            continue;
        }
        if (arg == "--path") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--path requires a value");
            }
            options.path_selector = argv[++i];
            continue;
        }
        if (!arg.empty() && arg.front() == '-') {
            throw std::runtime_error("Unknown option: " + arg);
        }
        if (saw_json_path) {
            throw std::runtime_error("Only one JSON path can be specified");
        }
        options.json_path = arg;
        saw_json_path = true;
    }

    return options;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const ToolOptions options = parseOptions(argc, argv);
        LoadedResult result = loadResultJson(options.json_path);

        if (options.list_paths) {
            for (std::size_t i = 0; i < result.paths.size(); ++i) {
                std::cout << i << ": " << result.paths[i].name;
                if (!result.paths[i].word.empty()) {
                    std::cout << " (" << result.paths[i].word << ")";
                }
                std::cout << ", samples=" << result.paths[i].samples.size()
                          << '\n';
            }
            return 0;
        }

        const std::size_t path_index =
            selectPath(result, options.path_selector);
        const PathRecord& selected = result.paths[path_index];
        if (selected.samples.empty()) {
            throw std::runtime_error("Selected path has no samples");
        }

        std::cout << "Loaded path JSON\n";
        std::cout << "  file: " << options.json_path << '\n';
        std::cout << "  map: " << result.map.width() << " x "
                  << result.map.height() << '\n';
        std::cout << "  paths: " << result.paths.size() << '\n';
        std::cout << "  selected: " << selected.name;
        if (!selected.word.empty()) {
            std::cout << " (" << selected.word << ")";
        }
        std::cout << '\n';
        std::cout << "  samples: " << selected.samples.size() << '\n';
        std::cout << "  search_tree_edges: "
                  << result.search_tree.size() << '\n';

        FltkViewer viewer(result.map, result.vehicle, selected.samples,
                          result.search_tree,
                          result.solution_node_ids,
                          result.solution_open_orders,
                          result.solution_close_orders,
                          result.solution_path_frame_starts);
        return viewer.run();
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
