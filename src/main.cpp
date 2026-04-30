#include "Car.hpp"
#include "GridMap.hpp"
#include "HtmlWriter.hpp"
#include "JsonExporter.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<CarPose> makeStraightPath(const GridMap& map, const Car& car) {
    const Pose2D& start = map.start();
    CarPose pose{
        .x = start.x,
        .y = start.y,
        .theta = start.theta,
        .steer = 0.0,
        .direction = 1
    };

    std::vector<CarPose> path;
    path.push_back(pose);

    constexpr int frame_count = 90;
    constexpr double step_distance = 0.18;
    for (int i = 0; i < frame_count; ++i) {
        pose = car.step(pose, 0.0, 1, step_distance);
        path.push_back(pose);
    }

    return path;
}

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    output << text;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string map_path = argc > 1
                                         ? argv[1]
                                         : "map/hybrid_astar_map_defalt.json";
        const GridMap map = MapLoader::loadJson(map_path);
        const Car car;
        const std::vector<CarPose> path = makeStraightPath(map, car);

        std::filesystem::create_directories("output");
        const std::string json = JsonExporter::exportPath(map, car, path);
        writeTextFile("output/result.json", json);
        writeTextFile("output/demo.html", HtmlWriter::wrap(json));

        const Pose2D& start = map.start();
        const Pose2D& goal = map.goal();

        std::cout << "Loaded grid map\n";
        std::cout << "  file: " << map_path << '\n';
        std::cout << "  size: " << map.width() << " x " << map.height() << '\n';
        std::cout << "  obstacles: " << map.obstacleCount() << '\n';
        std::cout << "  start: (" << start.x << ", " << start.y << ", "
                  << start.theta << ")\n";
        std::cout << "  goal: (" << goal.x << ", " << goal.y << ", " << goal.theta
                  << ")\n";
        std::cout << "Generated straight demo path\n";
        std::cout << "  poses: " << path.size() << '\n';
        std::cout << "  output/result.json\n";
        std::cout << "  output/demo.html\n";

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}