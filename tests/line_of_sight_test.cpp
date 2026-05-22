#include "LineOfSight.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectVisible(Point2D a,
                   Point2D b,
                   const ObstacleSet& obstacles,
                   const std::string& message) {
    return expect(hasLineOfSight(a, b, obstacles), message);
}

bool expectBlocked(Point2D a,
                   Point2D b,
                   const ObstacleSet& obstacles,
                   const std::string& message) {
    return expect(!hasLineOfSight(a, b, obstacles), message);
}

bool containsCell(const std::vector<HashCell>& cells, HashCell target) {
    for (const HashCell& cell : cells) {
        if (cell == target) {
            return true;
        }
    }
    return false;
}

} // namespace

int main() {
    bool ok = true;

    ok &= expectVisible(
        {0.0, 1.0}, {3.0, 1.0}, ObstacleSet{{1, 1}},
        "line exactly along obstacle bottom edge should be visible");

    ok &= expectVisible(
        {0.0, 0.0}, {2.0, 2.0}, ObstacleSet{{1, 0}, {0, 1}},
        "line through shared obstacle corner should be visible");

    ok &= expectBlocked(
        {0.2, 0.2}, {3.8, 3.8}, ObstacleSet{{2, 2}},
        "diagonal line through obstacle interior should be blocked");

    ok &= expectBlocked(
        {0.2, 1.5}, {3.8, 1.5}, ObstacleSet{{2, 1}},
        "horizontal line through obstacle interior should be blocked");

    ok &= expectBlocked(
        {1.5, 0.2}, {1.5, 3.8}, ObstacleSet{{1, 2}},
        "vertical line through obstacle interior should be blocked");

    ok &= expectVisible(
        {0.2, 0.2}, {3.8, 0.8}, ObstacleSet{{2, 2}},
        "slanted line missing obstacle should be visible");

    ok &= expectBlocked(
        {0.2, 2.8}, {3.8, 1.2}, ObstacleSet{{2, 1}},
        "non-45-degree slanted line through obstacle interior should be blocked");

    ok &= expectVisible(
        {1.0, 0.0}, {1.0, 3.0}, ObstacleSet{{1, 1}, {0, 1}},
        "vertical line exactly on obstacle boundary should be visible");

    const std::vector<HashCell> diagonal_cells =
        supercoverDdaCells({0.2, 0.2}, {3.8, 3.8});
    ok &= expect(
        containsCell(diagonal_cells, {1, 1})
            && containsCell(diagonal_cells, {2, 2})
            && containsCell(diagonal_cells, {3, 3}),
        "supercover DDA should enumerate diagonal cells touched by the segment");

    const std::vector<HashCell> corner_cells =
        supercoverDdaCells({0.0, 0.0}, {2.0, 2.0});
    ok &= expect(
        containsCell(corner_cells, {1, 0})
            && containsCell(corner_cells, {0, 1})
            && containsCell(corner_cells, {1, 1}),
        "supercover DDA should include side cells touched at a grid corner");

    const std::vector<HashCell> boundary_cells =
        supercoverDdaCells({0.0, 1.0}, {3.0, 1.0});
    ok &= expect(
        containsCell(boundary_cells, {1, 1})
            && containsCell(boundary_cells, {1, 0}),
        "supercover DDA should include both rows touched by a grid-line segment");

    if (!ok) {
        return 1;
    }

    std::cout << "line_of_sight_test passed\n";
    return 0;
}
