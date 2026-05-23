/**
 * @file GridMap.cpp
 * @brief 栅格地图实现
 *
 * 实现栅格地图的创建、障碍物管理、起点终点设置
 * 以及JSON格式地图文件的加载。
 */

#include "GridMap.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>
#include <string>

namespace {

/**
 * @brief 读取文本文件内容
 * @param[in] path 文件路径
 * @return 文件内容字符串
 * @throw std::runtime_error 文件无法打开时
 */
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

/**
 * @brief 从JSON对象中读取数值字段
 * @param[in] object  JSON对象字符串
 * @param[in] key     字段名
 * @param[in] fallback 字段不存在时的默认值
 * @param[in] required 是否必填
 * @return 解析出的数值
 * @throw std::runtime_error required为true但字段缺失时
 */
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

/**
 * @brief 从JSON对象中读取整数字段
 * @param[in] object  JSON对象字符串
 * @param[in] key     字段名
 * @param[in] fallback 字段不存在时的默认值
 * @param[in] required 是否必填
 * @return 解析出的整数值
 */
int readIntField(const std::string& object,
                 const std::string& key,
                 int fallback,
                 bool required) {
    return static_cast<int>(readNumberField(object, key, fallback, required));
}

/**
 * @brief 从JSON中读取对象类型字段
 * @param[in] json JSON完整字符串
 * @param[in] key  对象字段名
 * @return 对象内容（包括大括号）
 * @throw std::runtime_error 字段不存在或未闭合时
 */
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

/**
 * @brief 从JSON中读取数组类型字段
 * @param[in] json JSON完整字符串
 * @param[in] key  数组字段名
 * @return 数组内容（包括方括号）
 * @throw std::runtime_error 字段不存在或未闭合时
 */
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

/**
 * @brief 从数组字符串中解析所有对象
 * @param[in] array_text 数组内容字符串
 * @return 对象字符串列表
 * @throw std::runtime_error 数组格式错误时
 */
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

/**
 * @brief 从JSON对象解析位姿
 * @param[in] object JSON对象字符串
 * @return 解析后的2D位姿
 */
Pose2D readPose(const std::string& object) {
    return {
        readNumberField(object, "x", 0.0, true),
        readNumberField(object, "y", 0.0, true),
        readNumberField(object, "theta", 0.0, false)
    };
}

} // namespace

/**
 * @brief 构造指定尺寸的空白地图
 * @param[in] width  地图宽度（栅格数）
 * @param[in] height 地图高度（栅格数）
 */
GridMap::GridMap(int width, int height) {
    resize(width, height);
}

/** @brief 返回地图宽度。 */
int GridMap::width() const {
    return width_;
}

/** @brief 返回地图高度。 */
int GridMap::height() const {
    return height_;
}

/** @brief 判断地图是否未初始化。 */
bool GridMap::empty() const {
    return width_ <= 0 || height_ <= 0 || cells_.empty();
}

/**
 * @brief 判断坐标是否在地图范围内
 * @param[in] x 栅格列索引
 * @param[in] y 栅格行索引
 * @return true如果在范围内
 */
bool GridMap::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
}

/**
 * @brief 判断指定栅格是否为障碍物
 * @param[in] x 栅格列索引
 * @param[in] y 栅格行索引
 * @return true如果是障碍物，地图外或自由空间返回false
 */
bool GridMap::isObstacle(int x, int y) const {
    if (!inBounds(x, y)) {
        return true;
    }
    return cells_[index(x, y)] != 0;
}

/**
 * @brief 判断指定栅格是否为自由空间
 * @param[in] x 栅格列索引
 * @param[in] y 栅格行索引
 * @return true如果是自由空间
 */
bool GridMap::isFree(int x, int y) const {
    return inBounds(x, y) && !isObstacle(x, y);
}

/** @brief 返回当前障碍物总数。 */
int GridMap::obstacleCount() const {
    return obstacle_count_;
}

/**
 * @brief 重置地图尺寸，清除所有数据
 * @param[in] width  新宽度
 * @param[in] height 新高度
 * @throw std::runtime_error 宽高非正时
 */
void GridMap::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("GridMap size must be positive");
    }

    width_ = width;
    height_ = height;
    obstacle_count_ = 0;
    cells_.assign(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0);
}

/** @brief 清除所有障碍物。 */
void GridMap::clearObstacles() {
    std::fill(cells_.begin(), cells_.end(), 0);
    obstacle_count_ = 0;
}

/**
 * @brief 设置指定栅格的障碍物状态
 * @param[in] x         栅格列索引
 * @param[in] y         栅格行索引
 * @param[in] occupied  true=设为障碍物，false=清除障碍物
 */
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

/** @brief 设置起点位姿。 */
void GridMap::setStart(Pose2D pose) {
    start_ = pose;
}

/** @brief 设置目标点位姿。 */
void GridMap::setGoal(Pose2D pose) {
    goal_ = pose;
}

/** @brief 返回起点位姿引用。 */
const Pose2D& GridMap::start() const {
    return start_;
}

/** @brief 返回目标点位姿引用。 */
const Pose2D& GridMap::goal() const {
    return goal_;
}

/** @brief 返回底层栅格数据引用。 */
const std::vector<std::uint8_t>& GridMap::cells() const {
    return cells_;
}

/** @brief 将二维坐标转换为一维索引。 */
int GridMap::index(int x, int y) const {
    return y * width_ + x;
}

/**
 * @brief 从JSON文件加载地图
 * @param[in] path JSON文件路径
 * @return 构造好的栅格地图
 * @throw std::runtime_error 文件无法打开或解析失败
 */
GridMap MapLoader::loadJson(const std::string& path) {
    const std::string json = readTextFile(path);

    // 读取地图尺寸
    const int width = readIntField(json, "width", 0, true);
    const int height = readIntField(json, "height", 0, true);

    GridMap map(width, height);

    // 读取起点和终点
    map.setStart(readPose(readObjectField(json, "start")));
    map.setGoal(readPose(readObjectField(json, "goal")));

    // 读取障碍物列表
    const std::string obstacles = readArrayField(json, "obstacles");
    for (const std::string& object : readObjectsInArray(obstacles)) {
        const int x0 = readIntField(object, "x", 0, true);
        const int y0 = readIntField(object, "y", 0, true);
        const int w = std::max(1, readIntField(object, "w", 1, false));
        const int h = std::max(1, readIntField(object, "h", 1, false));

        // 填充矩形障碍物区域
        for (int y = y0; y < y0 + h; ++y) {
            for (int x = x0; x < x0 + w; ++x) {
                map.setObstacle(x, y, true);
            }
        }
    }

    return map;
}