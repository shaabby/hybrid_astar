#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <string>
#include <vector>

class JsonExporter {
public:
    [[nodiscard]] static std::string exportPath(
        const GridMap& map,
        const Car& car,
        const std::vector<CarPose>& path);

    [[nodiscard]] static std::string exportMap(const GridMap& map);
    [[nodiscard]] static std::string exportVehicle(const VehicleConfig& vehicle);
    [[nodiscard]] static std::string exportPathPoints(
        const std::vector<CarPose>& path);
};