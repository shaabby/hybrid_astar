#include "GridMap.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>
#include <string>

namespace {

std::string readTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open map json: " + path);
    }

    return {
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()
    };
}

double readNumberField(const std::string& object,
                       const std::string& key,
                       double fallback,
                       bool required) {
    const std::regex pattern("\"" + key + R"("\s*:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?))");
    std::smatch match;
    if (std::regex_search(object, match, pattern)) {
        return std::stod(match[1].str());
    }
    if (required) {
        throw std::runtime_error("Missing numeric field: " + key);
    }
    return fallback;
}

int readIntField(const std::string& object,
                 const std::string& key,
                 int fallback,
                 bool required) {
    return static_cast<int>(readNumberField(object, key, fallback, required));
}

std::string readObjectField(const std::string& json, const std::string& key) {
    const std::regex start_pattern("\"" + key + R"("\s*:\s*\{)");
    std::smatch match;
    if (!std::regex_search(json, match, start_pattern)) {
        throw std::runtime_error("Missing object field: " + key);
    }

    std::size_t pos = static_cast<std::size_t>(match.position() + match.length() - 1);
    int depth = 0;
    for (std::size_t i = pos; i < json.size(); ++i) {
        if (json[i] == '{') {
            ++depth;
        } else if (json[i] == '}') {
            --depth;
            if (depth == 0) {
                return json.substr(pos, i - pos + 1);
            }
        }
    }

    throw std::runtime_error("Unclosed object field: " + key);
}

std::string readArrayField(const std::string& json, const std::string& key) {
    const std::regex start_pattern("\"" + key + R"("\s*:\s*\[)");
    std::smatch match;
    if (!std::regex_search(json, match, start_pattern)) {
        throw std::runtime_error("Missing array field: " + key);
    }

    std::size_t pos = static_cast<std::size_t>(match.position() + match.length() - 1);
    int depth = 0;
    for (std::size_t i = pos; i < json.size(); ++i) {
        if (json[i] == '[') {
            ++depth;
        } else if (json[i] == ']') {
            --depth;
            if (depth == 0) {
                return json.substr(pos, i - pos + 1);
            }
        }
    }

    throw std::runtime_error("Unclosed array field: " + key);
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

Pose2D readPose(const std::string& object) {
    return {
        readNumberField(object, "x", 0.0, true),
        readNumberField(object, "y", 0.0, true),
        readNumberField(object, "theta", 0.0, false)
    };
}

} // namespace

GridMap::GridMap(int width, int height) {
    resize(width, height);
}

int GridMap::width() const {
    return width_;
}

int GridMap::height() const {
    return height_;
}

bool GridMap::empty() const {
    return width_ <= 0 || height_ <= 0 || cells_.empty();
}

bool GridMap::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool GridMap::isObstacle(int x, int y) const {
    if (!inBounds(x, y)) {
        return true;
    }
    return cells_[index(x, y)] != 0;
}

bool GridMap::isFree(int x, int y) const {
    return inBounds(x, y) && !isObstacle(x, y);
}

int GridMap::obstacleCount() const {
    return obstacle_count_;
}

void GridMap::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("GridMap size must be positive");
    }

    width_ = width;
    height_ = height;
    obstacle_count_ = 0;
    cells_.assign(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0);
}

void GridMap::clearObstacles() {
    std::fill(cells_.begin(), cells_.end(), 0);
    obstacle_count_ = 0;
}

void GridMap::setObstacle(int x, int y, bool occupied) {
    if (!inBounds(x, y)) {
        return;
    }

    std::uint8_t& cell = cells_[index(x, y)];
    const bool was_occupied = cell != 0;
    if (occupied && !was_occupied) {
        cell = 1;
        ++obstacle_count_;
    } else if (!occupied && was_occupied) {
        cell = 0;
        --obstacle_count_;
    }
}

void GridMap::setStart(Pose2D pose) {
    start_ = pose;
}

void GridMap::setGoal(Pose2D pose) {
    goal_ = pose;
}

const Pose2D& GridMap::start() const {
    return start_;
}

const Pose2D& GridMap::goal() const {
    return goal_;
}

const std::vector<std::uint8_t>& GridMap::cells() const {
    return cells_;
}

int GridMap::index(int x, int y) const {
    return y * width_ + x;
}

GridMap MapLoader::loadJson(const std::string& path) {
    const std::string json = readTextFile(path);

    const int width = readIntField(json, "width", 0, true);
    const int height = readIntField(json, "height", 0, true);

    GridMap map(width, height);
    map.setStart(readPose(readObjectField(json, "start")));
    map.setGoal(readPose(readObjectField(json, "goal")));

    const std::string obstacles = readArrayField(json, "obstacles");
    for (const std::string& object : readObjectsInArray(obstacles)) {
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

    return map;
}
