#include "LineOfSight.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

class TestRunner {
public:
    void expect(bool condition, std::string_view message) {
        ++checks_;
        if (condition) {
            return;
        }

        ++failures_;
        std::cerr << "FAIL: " << message << '\n';
    }

    [[nodiscard]] int result() const {
        if (failures_ != 0) {
            std::cerr << failures_ << " of " << checks_ << " checks failed\n";
            return 1;
        }

        std::cout << checks_ << " line_of_sight checks passed\n";
        return 0;
    }

private:
    int checks_ = 0;
    int failures_ = 0;
};

bool containsCell(const std::vector<HashCell>& cells, HashCell target) {
    return std::ranges::find(cells, target) != cells.end();
}

std::vector<HashCell> sortedCells(std::vector<HashCell> cells) {
    std::ranges::sort(cells, [](HashCell lhs, HashCell rhs) {
        if (lhs.x != rhs.x) {
            return lhs.x < rhs.x;
        }
        return lhs.y < rhs.y;
    });
    return cells;
}

void expectVisible(TestRunner& runner,
                   Point2D a,
                   Point2D b,
                   const ObstacleSet& obstacles,
                   std::string_view message) {
    runner.expect(hasLineOfSight(a, b, obstacles), message);
}

void expectBlocked(TestRunner& runner,
                   Point2D a,
                   Point2D b,
                   const ObstacleSet& obstacles,
                   std::string_view message) {
    runner.expect(!hasLineOfSight(a, b, obstacles), message);
}

void expectCellsContain(TestRunner& runner,
                        const std::vector<HashCell>& cells,
                        const std::vector<HashCell>& expected,
                        std::string_view message) {
    const bool contains_all =
        std::ranges::all_of(expected, [&](HashCell cell) {
            return containsCell(cells, cell);
        });
    runner.expect(contains_all, message);
}

void expectCellsEqualIgnoringOrder(TestRunner& runner,
                                   std::vector<HashCell> actual,
                                   std::vector<HashCell> expected,
                                   std::string_view message) {
    runner.expect(sortedCells(std::move(actual)) == sortedCells(std::move(expected)),
                  message);
}

void testHasLineOfSight(TestRunner& runner) {
    expectVisible(runner,
                  {0.2, 0.2},
                  {3.8, 0.8},
                  ObstacleSet{{2, 2}},
                  "line missing obstacle should be visible");

    expectVisible(runner,
                  {0.2, 0.2},
                  {3.8, 3.8},
                  ObstacleSet{},
                  "empty obstacle set should be visible");

    expectVisible(runner,
                  {0.0, 0.0},
                  {2.0, 2.0},
                  ObstacleSet{{-1, 0}, {0, -1}, {-1, -1}},
                  "start-point cells opposite the movement direction should be ignored");

    expectBlocked(runner,
                  {0.2, 0.2},
                  {3.8, 3.8},
                  ObstacleSet{{2, 2}},
                  "diagonal line touching obstacle cell should be blocked");

    expectBlocked(runner,
                  {0.2, 1.5},
                  {3.8, 1.5},
                  ObstacleSet{{2, 1}},
                  "horizontal line touching obstacle cell should be blocked");

    expectBlocked(runner,
                  {1.5, 0.2},
                  {1.5, 3.8},
                  ObstacleSet{{1, 2}},
                  "vertical line touching obstacle cell should be blocked");

    expectBlocked(runner,
                  {0.2, 2.8},
                  {3.8, 1.2},
                  ObstacleSet{{2, 1}},
                  "slanted line touching obstacle cell should be blocked");

    expectBlocked(runner,
                  {0.0, 1.0},
                  {3.0, 1.0},
                  ObstacleSet{{1, 1}},
                  "grid-line contact with obstacle cell should be blocked");

    expectBlocked(runner,
                  {0.0, 0.0},
                  {2.0, 2.0},
                  ObstacleSet{{1, 0}},
                  "shared-corner contact with obstacle cell should be blocked");
}

void testSupercoverDdaCells(TestRunner& runner) {
    expectCellsContain(runner,
                       supercoverDdaCells({0.2, 0.2}, {3.8, 3.8}),
                       {{1, 1}, {2, 2}, {3, 3}},
                       "diagonal segment should enumerate diagonal cells");

    expectCellsContain(runner,
                       supercoverDdaCells({0.0, 0.0}, {2.0, 2.0}),
                       {{1, 0}, {0, 1}, {1, 1}},
                       "corner crossing should include touched side cells");

    expectCellsContain(runner,
                       supercoverDdaCells({0.0, 1.0}, {3.0, 1.0}),
                       {{1, 1}, {1, 0}},
                       "grid-line segment should include both touched rows");

    expectCellsEqualIgnoringOrder(
        runner,
        supercoverDdaCells({0.2, 0.2}, {3.8, 3.8}),
        supercoverDdaCells({3.8, 3.8}, {0.2, 0.2}),
        "reversed diagonal segment should cover the same cells");
}

void testDegenerateSegments(TestRunner& runner) {
    expectCellsContain(runner,
                       supercoverDdaCells({1.2, 2.8}, {1.2, 2.8}),
                       {{1, 2}},
                       "single point should include its containing cell");

    expectCellsContain(runner,
                       supercoverDdaCells({1.0, 2.0}, {1.0, 2.0}),
                       {{1, 2}},
                       "single grid-corner point should include only its containing cell");

    expectBlocked(runner,
                  {1.2, 2.8},
                  {1.2, 2.8},
                  ObstacleSet{{1, 2}},
                  "single point inside obstacle cell should be blocked");

    expectVisible(runner,
                  {1.0, 2.0},
                  {1.0, 2.0},
                  ObstacleSet{{0, 1}},
                  "single grid-corner point should ignore adjacent obstacle cells");
}

} // namespace

int main() {
    TestRunner runner;

    testHasLineOfSight(runner);
    testSupercoverDdaCells(runner);
    testDegenerateSegments(runner);

    return runner.result();
}
