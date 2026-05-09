#include "FltkCanvas.hpp"

#include <FL/fl_draw.H>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

constexpr int kMargin = 42;
Fl_Color rgb(unsigned int value) {
    return fl_rgb_color(
        static_cast<unsigned char>((value >> 16) & 0xff),
        static_cast<unsigned char>((value >> 8) & 0xff),
        static_cast<unsigned char>(value & 0xff));
}

} // namespace

FltkCanvas::FltkCanvas(int x, int y, int w, int h, const VisualizationData& data)
    : Fl_Widget(x, y, w, h),
      data_(data) {}

void FltkCanvas::setFrame(int frame) {
    frame_ = std::clamp(frame, 0, std::max(0, frameCount() - 1));
    redraw();
}

int FltkCanvas::frame() const {
    return frame_;
}

int FltkCanvas::frameCount() const {
    return static_cast<int>(data_.path.size());
}

double FltkCanvas::scale() const {
    const int usable_w = std::max(1, w() - kMargin * 2);
    const int usable_h = std::max(1, h() - kMargin * 2);
    return std::min(
        static_cast<double>(usable_w) / static_cast<double>(data_.map.width()),
        static_cast<double>(usable_h) / static_cast<double>(data_.map.height()));
}

double FltkCanvas::worldX(double value) const {
    return static_cast<double>(x()) + kMargin + value * scale();
}

double FltkCanvas::worldY(double value) const {
    return static_cast<double>(y()) + kMargin
        + (static_cast<double>(data_.map.height()) - value) * scale();
}

void FltkCanvas::draw() {
    fl_color(rgb(0xffffff));
    fl_rectf(x(), y(), w(), h());
    fl_color(rgb(0xcfd6df));
    fl_rect(x(), y(), w(), h());

    drawGrid();
    drawObstacles();
    drawPoseMarker(data_.map.start(), 0x16a34a, "S");
    drawPoseMarker(data_.map.goal(), 0xdc2626, "G");
    drawPath();

    if (!data_.path.empty()) {
        drawCar(data_.path[static_cast<std::size_t>(frame_)]);
    }
}

void FltkCanvas::drawGrid() const {
    fl_color(rgb(0xe5e7eb));
    fl_line_style(FL_SOLID, 1);

    for (int grid_x = 0; grid_x <= data_.map.width(); ++grid_x) {
        fl_line(
            static_cast<int>(std::round(worldX(grid_x))),
            static_cast<int>(std::round(worldY(0))),
            static_cast<int>(std::round(worldX(grid_x))),
            static_cast<int>(std::round(worldY(data_.map.height()))));
    }

    for (int grid_y = 0; grid_y <= data_.map.height(); ++grid_y) {
        fl_line(
            static_cast<int>(std::round(worldX(0))),
            static_cast<int>(std::round(worldY(grid_y))),
            static_cast<int>(std::round(worldX(data_.map.width()))),
            static_cast<int>(std::round(worldY(grid_y))));
    }

    fl_line_style(0);
}

void FltkCanvas::drawObstacles() const {
    const double s = scale();
    fl_color(rgb(0x111827));
    for (int grid_y = 0; grid_y < data_.map.height(); ++grid_y) {
        for (int grid_x = 0; grid_x < data_.map.width(); ++grid_x) {
            if (!data_.map.isObstacle(grid_x, grid_y)) {
                continue;
            }
            fl_rectf(
                static_cast<int>(std::round(worldX(grid_x))),
                static_cast<int>(std::round(worldY(grid_y + 1))),
                static_cast<int>(std::ceil(s)),
                static_cast<int>(std::ceil(s)));
        }
    }
}

void FltkCanvas::drawPoseMarker(
    const Pose2D& pose, unsigned int color, const char* label) const {
    const double s = scale();
    const int marker_radius = std::max(4, static_cast<int>(std::round(s * 0.36)));
    const int cx = static_cast<int>(std::round(worldX(pose.x)));
    const int cy = static_cast<int>(std::round(worldY(pose.y)));

    fl_color(rgb(color));
    fl_pie(cx - marker_radius, cy - marker_radius,
           marker_radius * 2, marker_radius * 2, 0.0, 360.0);
    fl_line_style(FL_SOLID, 3);
    fl_line(cx, cy,
            static_cast<int>(std::round(worldX(pose.x + std::cos(pose.theta) * 1.2))),
            static_cast<int>(std::round(worldY(pose.y + std::sin(pose.theta) * 1.2))));
    fl_line_style(0);

    fl_color(rgb(0x111827));
    fl_font(FL_HELVETICA_BOLD, 14);
    const int label_width = static_cast<int>(fl_width(label));
    fl_draw(label, cx - label_width / 2,
            static_cast<int>(std::round(cy - s * 0.72)));
}

void FltkCanvas::drawPath() const {
    if (data_.path.empty()) {
        return;
    }

    fl_color(rgb(0x2563eb));
    fl_line_style(FL_SOLID, 3);
    const int limit = std::clamp(frame_, 0, frameCount() - 1);

    for (int i = 1; i <= limit; ++i) {
        const CarPose& a = data_.path[static_cast<std::size_t>(i - 1)];
        const CarPose& b = data_.path[static_cast<std::size_t>(i)];
        fl_line(
            static_cast<int>(std::round(worldX(a.x))),
            static_cast<int>(std::round(worldY(a.y))),
            static_cast<int>(std::round(worldX(b.x))),
            static_cast<int>(std::round(worldY(b.y))));
    }

    fl_line_style(0);
}

void FltkCanvas::drawCar(const CarPose& pose) const {
    const VehicleConfig& vehicle = data_.vehicle;
    const double front = vehicle.length - vehicle.rear_to_center;
    const double rear = -vehicle.rear_to_center;
    const double half_width = vehicle.width * 0.5;
    const double c = std::cos(pose.theta);
    const double st = std::sin(pose.theta);

    const double local[4][2] = {
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    };

    int px[4]{};
    int py[4]{};
    for (int i = 0; i < 4; ++i) {
        const double wx = pose.x + local[i][0] * c - local[i][1] * st;
        const double wy = pose.y + local[i][0] * st + local[i][1] * c;
        px[i] = static_cast<int>(std::round(worldX(wx)));
        py[i] = static_cast<int>(std::round(worldY(wy)));
    }

    fl_color(rgb(0xf97316));
    fl_polygon(px[0], py[0], px[1], py[1], px[2], py[2], px[3], py[3]);
    fl_color(rgb(0x9a3412));
    fl_line_style(FL_SOLID, 2);
    for (int i = 0; i < 4; ++i) {
        const int next = (i + 1) % 4;
        fl_line(px[i], py[i], px[next], py[next]);
    }

    const int rear_x = static_cast<int>(std::round(worldX(pose.x)));
    const int rear_y = static_cast<int>(std::round(worldY(pose.y)));
    const int front_x = static_cast<int>(std::round(
        worldX(pose.x + front * std::cos(pose.theta))));
    const int front_y = static_cast<int>(std::round(
        worldY(pose.y + front * std::sin(pose.theta))));

    fl_color(rgb(0xffffff));
    fl_line_style(FL_SOLID, 3);
    fl_line(rear_x, rear_y, front_x, front_y);
    fl_pie(rear_x - 4, rear_y - 4, 8, 8, 0.0, 360.0);
    fl_line_style(0);
}
