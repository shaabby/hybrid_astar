#include "AppConfig.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

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

        std::cout << checks_ << " app_config checks passed\n";
        return 0;
    }

private:
    int checks_ = 0;
    int failures_ = 0;
};

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1.0e-12;
}

void writeTextFile(const std::filesystem::path& path, std::string_view text) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("failed to write test file: " + path.string());
    }
    output << text;
}

std::filesystem::path writeConfig(std::string_view name, std::string_view text) {
    const std::filesystem::path directory =
        std::filesystem::current_path() / "app_config_test_tmp";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / std::string(name);
    writeTextFile(path, text);
    return path;
}

void testLoadsFields(TestRunner& runner) {
    const std::filesystem::path path = writeConfig(
        "hybrid_astar_app_config_valid.yaml",
        R"(# comment
map_path: "map/simple01.json"

vehicle:
  length: 4.7
  width: 2.1
  wheelbase: 2.8
  rear_to_center: 1.4
  max_steer: 0.7

hybrid_astar:
  xy_resolution: 0.5
  theta_bins: 180
  step_size: 0.1
  primitive_length: 1.0
  goal_xy_tolerance: 0.8
  goal_theta_tolerance: 0.15
  reverse_penalty: 1.5
  steer_penalty: 0.2
  gear_switch_penalty: 2.0
  steer_change_penalty: 0.3
  max_iterations: 42
  allow_reverse: yes
  enable_analytic_expansion: 0
  analytic_expansion_distance: 10.5
  analytic_expansion_interval: 3
  collision_safety_margin: 0.05
  enable_obstacle_heuristic: true
  obstacle_heuristic_inflate_alpha: 1.25
  debug: no
  debug_progress_interval: 7
)");

    const AppConfig config = AppConfigLoader::loadYaml(path.string());
    runner.expect(config.map_path == "map/simple01.json",
                  "map_path should load");
    runner.expect(near(config.vehicle.length, 4.7),
                  "vehicle.length should load");
    runner.expect(near(config.vehicle.width, 2.1),
                  "vehicle.width should load");
    runner.expect(near(config.vehicle.wheelbase, 2.8),
                  "vehicle.wheelbase should load");
    runner.expect(near(config.vehicle.rear_to_center, 1.4),
                  "vehicle.rear_to_center should load");
    runner.expect(near(config.vehicle.max_steer, 0.7),
                  "vehicle.max_steer should load");
    runner.expect(near(config.hybrid_astar.xy_resolution, 0.5),
                  "hybrid_astar.xy_resolution should load");
    runner.expect(config.hybrid_astar.theta_bins == 180,
                  "hybrid_astar.theta_bins should load");
    runner.expect(near(config.hybrid_astar.step_size, 0.1),
                  "hybrid_astar.step_size should load");
    runner.expect(config.hybrid_astar.max_iterations == 42,
                  "hybrid_astar.max_iterations should load");
    runner.expect(config.hybrid_astar.allow_reverse,
                  "hybrid_astar.allow_reverse should parse yes");
    runner.expect(!config.hybrid_astar.enable_analytic_expansion,
                  "hybrid_astar.enable_analytic_expansion should parse 0");
    runner.expect(!config.hybrid_astar.debug,
                  "hybrid_astar.debug should parse no");
}

void expectThrows(TestRunner& runner,
                  std::string_view name,
                  std::string_view text,
                  std::string_view message) {
    const std::filesystem::path path = writeConfig(std::string(name), text);
    bool threw = false;
    try {
        (void)AppConfigLoader::loadYaml(path.string());
    } catch (const std::exception&) {
        threw = true;
    }
    runner.expect(threw, message);
}

void testErrors(TestRunner& runner) {
    expectThrows(runner,
                 "hybrid_astar_app_config_missing_map.yaml",
                 "vehicle:\n  length: 4.5\n",
                 "missing map_path should throw");
    expectThrows(runner,
                 "hybrid_astar_app_config_missing_vehicle.yaml",
                 "map_path: map/default_map.json\n",
                 "missing vehicle fields should throw");
    expectThrows(runner,
                 "hybrid_astar_app_config_missing_planner.yaml",
                 R"(map_path: map/default_map.json
vehicle:
  length: 4.5
  width: 2.0
  wheelbase: 2.7
  rear_to_center: 1.35
  max_steer: 0.61
)",
                 "missing planner fields should throw");
    expectThrows(runner,
                 "hybrid_astar_app_config_unknown.yaml",
                 "map_path: map/default_map.json\nhybrid_astar:\n  nope: 1\n",
                 "unknown field should throw");
    expectThrows(runner,
                 "hybrid_astar_app_config_bad_type.yaml",
                 "map_path: map/default_map.json\nhybrid_astar:\n  theta_bins: x\n",
                 "bad type should throw");
}

} // namespace

int main() {
    TestRunner runner;
    testLoadsFields(runner);
    testErrors(runner);
    return runner.result();
}
