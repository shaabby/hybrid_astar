#pragma once

#include <cstddef>
#include <unordered_set>
#include <vector>

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct HashCell {
    int x = 0;
    int y = 0;

    bool operator==(const HashCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct HashCellHasher {
    std::size_t operator()(const HashCell& cell) const;
};

using ObstacleSet = std::unordered_set<HashCell, HashCellHasher>;

[[nodiscard]] std::vector<HashCell> supercoverDdaCells(Point2D a,
                                                       Point2D b,
                                                       double eps = 1.0e-9);

[[nodiscard]] bool segmentEntersCellInterior(Point2D a,
                                             Point2D b,
                                             HashCell cell,
                                             double eps = 1.0e-9);

[[nodiscard]] bool hasLineOfSight(Point2D a,
                                  Point2D b,
                                  const ObstacleSet& obstacles,
                                  double eps = 1.0e-9);
