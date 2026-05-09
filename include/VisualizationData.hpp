#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <vector>

struct VisualizationData {
    const GridMap& map;
    VehicleConfig vehicle;
    const std::vector<CarPose>& path;
    const std::vector<CarPose>& expanded;
};
