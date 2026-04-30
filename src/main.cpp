#include "Car.hpp"
#include "GridMap.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
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

void writeJsonString(std::ostream& out,
                     const GridMap& map,
                     const Car& car,
                     const std::vector<CarPose>& path) {
    const VehicleConfig& vehicle = car.config();
    const Pose2D& start = map.start();
    const Pose2D& goal = map.goal();

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"map\": {\n";
    out << "    \"width\": " << map.width() << ",\n";
    out << "    \"height\": " << map.height() << ",\n";
    out << "    \"start\": {\"x\": " << start.x << ", \"y\": " << start.y
        << ", \"theta\": " << start.theta << "},\n";
    out << "    \"goal\": {\"x\": " << goal.x << ", \"y\": " << goal.y
        << ", \"theta\": " << goal.theta << "},\n";
    out << "    \"obstacles\": [\n";

    bool first_obstacle = true;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (!map.isObstacle(x, y)) {
                continue;
            }
            if (!first_obstacle) {
                out << ",\n";
            }
            first_obstacle = false;
            out << "      {\"x\": " << x << ", \"y\": " << y << "}";
        }
    }
    out << "\n";
    out << "    ]\n";
    out << "  },\n";

    out << "  \"vehicle\": {\n";
    out << "    \"length\": " << vehicle.length << ",\n";
    out << "    \"width\": " << vehicle.width << ",\n";
    out << "    \"wheelbase\": " << vehicle.wheelbase << ",\n";
    out << "    \"rearToCenter\": " << vehicle.rear_to_center << ",\n";
    out << "    \"maxSteer\": " << vehicle.max_steer << "\n";
    out << "  },\n";

    out << "  \"path\": [\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        const CarPose& pose = path[i];
        out << "    {\"x\": " << pose.x << ", \"y\": " << pose.y
            << ", \"theta\": " << pose.theta
            << ", \"steer\": " << pose.steer
            << ", \"direction\": " << pose.direction << "}";
        if (i + 1 < path.size()) {
            out << ",";
        }
        out << "\n";
    }
    out << "  ],\n";
    out << "  \"expanded\": []\n";
    out << "}\n";
}

std::string buildJsonString(const GridMap& map,
                            const Car& car,
                            const std::vector<CarPose>& path) {
    std::ostringstream out;
    writeJsonString(out, map, car, path);
    return out.str();
}

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write file: " + path.string());
    }
    output << text;
}

std::string buildDemoHtml(const std::string& json) {
    return R"(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Hybrid A* Straight Car Demo</title>
  <style>
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: #f4f5f7;
      color: #17202a;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main { width: min(1120px, calc(100vw - 32px)); }
    .bar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }
    h1 { margin: 0; font-size: 20px; }
    button {
      border: 1px solid #b8c0cc;
      background: white;
      border-radius: 6px;
      padding: 8px 12px;
      font: inherit;
      cursor: pointer;
    }
    canvas {
      display: block;
      width: 100%;
      height: auto;
      background: white;
      border: 1px solid #cfd6df;
    }
  </style>
</head>
<body>
  <main>
    <div class="bar">
      <h1>Hybrid A* Output Loop Demo: Straight Vehicle Motion</h1>
      <button id="toggle" type="button">Pause</button>
    </div>
    <canvas id="canvas" width="1100" height="720"></canvas>
  </main>

  <script id="planner-data" type="application/json">
)" + json + R"(  </script>

  <script>
    const data = JSON.parse(document.getElementById("planner-data").textContent);
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");
    const margin = 42;
    const scale = Math.min(
      (canvas.width - margin * 2) / data.map.width,
      (canvas.height - margin * 2) / data.map.height
    );
    let frame = 0;
    let playing = true;

    function wx(x) { return margin + x * scale; }
    function wy(y) { return margin + (data.map.height - y) * scale; }

    function drawGrid() {
      ctx.strokeStyle = "#e5e7eb";
      ctx.lineWidth = 1;
      for (let x = 0; x <= data.map.width; ++x) {
        ctx.beginPath();
        ctx.moveTo(wx(x), wy(0));
        ctx.lineTo(wx(x), wy(data.map.height));
        ctx.stroke();
      }
      for (let y = 0; y <= data.map.height; ++y) {
        ctx.beginPath();
        ctx.moveTo(wx(0), wy(y));
        ctx.lineTo(wx(data.map.width), wy(y));
        ctx.stroke();
      }
    }

    function drawObstacles() {
      ctx.fillStyle = "#111827";
      for (const obs of data.map.obstacles) {
        ctx.fillRect(wx(obs.x), wy(obs.y + 1), scale, scale);
      }
    }

    function drawPose(pose, color, label) {
      const x = wx(pose.x);
      const y = wy(pose.y);
      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x, y, scale * 0.36, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(pose.theta) * scale * 1.2, y - Math.sin(pose.theta) * scale * 1.2);
      ctx.stroke();
      ctx.fillStyle = "#111827";
      ctx.font = "700 14px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(label, x, y - scale * 0.72);
      ctx.restore();
    }

    function drawPath(limit) {
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 3;
      ctx.beginPath();
      for (let i = 0; i <= limit; ++i) {
        const p = data.path[i];
        if (i === 0) ctx.moveTo(wx(p.x), wy(p.y));
        else ctx.lineTo(wx(p.x), wy(p.y));
      }
      ctx.stroke();
    }

    function drawCar(pose) {
      const v = data.vehicle;
      ctx.save();
      ctx.translate(wx(pose.x), wy(pose.y));
      ctx.rotate(-pose.theta);

      ctx.fillStyle = "#f97316";
      ctx.strokeStyle = "#9a3412";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.rect(
        -v.rearToCenter * scale,
        -v.width * scale / 2,
        v.length * scale,
        v.width * scale
      );
      ctx.fill();
      ctx.stroke();

      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo((v.length - v.rearToCenter) * scale, 0);
      ctx.stroke();

      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawGrid();
      drawObstacles();
      drawPose(data.map.start, "#16a34a", "S");
      drawPose(data.map.goal, "#dc2626", "G");
      drawPath(frame);
      drawCar(data.path[frame]);
    }

    function tick() {
      if (playing) {
        frame = (frame + 1) % data.path.length;
      }
      draw();
      requestAnimationFrame(tick);
    }

    document.getElementById("toggle").addEventListener("click", () => {
      playing = !playing;
      document.getElementById("toggle").textContent = playing ? "Pause" : "Play";
    });

    draw();
    requestAnimationFrame(tick);
  </script>
</body>
</html>
)";
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string map_path = argc > 1 ? argv[1] : "map/hybrid_astar_map_defalt.json";
        const GridMap map = MapLoader::loadJson(map_path);
        const Car car;
        const std::vector<CarPose> path = makeStraightPath(map, car);

        std::filesystem::create_directories("output");
        const std::string json = buildJsonString(map, car, path);
        writeTextFile("output/result.json", json);
        writeTextFile("output/demo.html", buildDemoHtml(json));

        const Pose2D& start = map.start();
        const Pose2D& goal = map.goal();

        std::cout << "Loaded grid map\n";
        std::cout << "  file: " << map_path << '\n';
        std::cout << "  size: " << map.width() << " x " << map.height() << '\n';
        std::cout << "  obstacles: " << map.obstacleCount() << '\n';
        std::cout << "  start: (" << start.x << ", " << start.y << ", " << start.theta << ")\n";
        std::cout << "  goal: (" << goal.x << ", " << goal.y << ", " << goal.theta << ")\n";
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
