#pragma once

#include "Car.hpp"
#include "HybridAstar.hpp"

#include <string>

struct AppConfig {
    std::string map_path;
    VehicleConfig vehicle;
    HybridAstarConfig hybrid_astar;
};

class AppConfigLoader {
public:
    [[nodiscard]] static AppConfig loadYaml(const std::string& path);
};
