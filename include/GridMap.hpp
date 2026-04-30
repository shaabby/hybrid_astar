#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

class GridMap {
public:
    GridMap() = default;
    GridMap(int width, int height);

    [[nodiscard]] int width() const;
    [[nodiscard]] int height() const;
    [[nodiscard]] bool empty() const;
    [[nodiscard]] bool inBounds(int x, int y) const;
    [[nodiscard]] bool isObstacle(int x, int y) const;
    [[nodiscard]] bool isFree(int x, int y) const;
    [[nodiscard]] int obstacleCount() const;

    void resize(int width, int height);
    void clearObstacles();
    void setObstacle(int x, int y, bool occupied = true);
    void setStart(Pose2D pose);
    void setGoal(Pose2D pose);

    [[nodiscard]] const Pose2D& start() const;
    [[nodiscard]] const Pose2D& goal() const;
    [[nodiscard]] const std::vector<std::uint8_t>& cells() const;

private:
    [[nodiscard]] int index(int x, int y) const;

    int width_ = 0;
    int height_ = 0;
    int obstacle_count_ = 0;
    Pose2D start_;
    Pose2D goal_;
    std::vector<std::uint8_t> cells_;
};

class MapLoader {
public:
    static GridMap loadJson(const std::string& path);
};
