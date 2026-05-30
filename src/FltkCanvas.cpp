/**
 * @file FltkCanvas.cpp
 * @brief FLTK画布组件实现
 *
 * 实现基于FLTK的规划结果可视化画布，支持网格绘制、
 * 障碍物渲染、路径动画和车辆姿态显示。
 */

#include "FltkCanvas.hpp"

#include <FL/fl_draw.H>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

/// @brief 画布边距
constexpr int kMargin = 42;

/**
 * @brief 将RGB整数转换为FLTK颜色
 * @param[in] value RGB颜色值（如0x16a34a）
 * @return FLTK颜色类型
 */
Fl_Color rgb(unsigned int value) {
    return fl_rgb_color(
        static_cast<unsigned char>((value >> 16) & 0xff),
        static_cast<unsigned char>((value >> 8) & 0xff),
        static_cast<unsigned char>(value & 0xff));
}

} // namespace

/**
 * @brief 构造FLTK画布
 * @param[in] x      窗口x坐标
 * @param[in] y      窗口y坐标
 * @param[in] w      宽度
 * @param[in] h      高度
 * @param[in] map      地图引用
 * @param[in] vehicle  车辆配置
 * @param[in] path     路径采样点
 */
FltkCanvas::FltkCanvas(int x,
                       int y,
                       int w,
                       int h,
                       const GridMap& map,
                       const VehicleConfig& vehicle,
                       const std::vector<CarPose>& path,
                       const std::vector<SearchTreeEdge>& search_tree,
                       const std::vector<int>& solution_node_ids,
                       const std::vector<int>& solution_path_frame_starts)
    : Fl_Widget(x, y, w, h),
      map_(map),
      vehicle_(vehicle),
      path_(path),
      search_tree_(search_tree),
      solution_node_ids_(solution_node_ids),
      solution_path_frame_starts_(solution_path_frame_starts) {}

/**
 * @brief 设置当前帧号并重绘
 * @param[in] frame 帧号
 */
void FltkCanvas::setFrame(int frame) {
    frame_ = std::clamp(frame, 0, std::max(0, frameCount() - 1));
    redraw();
}

/** @brief 返回当前帧号。 */
int FltkCanvas::frame() const {
    return frame_;
}

/** @brief 返回总帧数（路径点数）。 */
int FltkCanvas::frameCount() const {
    return static_cast<int>(path_.size());
}

/**
 * @brief 计算世界坐标到屏幕坐标的缩放因子
 * @return 缩放比例
 */
double FltkCanvas::scale() const {
    const int usable_w = std::max(1, w() - kMargin * 2);
    const int usable_h = std::max(1, h() - kMargin * 2);
    return std::min(
        static_cast<double>(usable_w) / static_cast<double>(map_.width()),
        static_cast<double>(usable_h) / static_cast<double>(map_.height()));
}

/**
 * @brief 将世界坐标X转换为屏幕坐标
 * @param[in] value 世界坐标X值
 * @return 屏幕坐标
 */
double FltkCanvas::worldX(double value) const {
    return static_cast<double>(x()) + kMargin + value * scale();
}

/**
 * @brief 将世界坐标Y转换为屏幕坐标
 * @param[in] value 世界坐标Y值
 * @return 屏幕坐标
 *
 * Y轴翻转：世界坐标系Y向上为正，画布Y向下为正。
 */
double FltkCanvas::worldY(double value) const {
    return static_cast<double>(y()) + kMargin
        + (static_cast<double>(map_.height()) - value) * scale();
}

/** @brief FLTK重绘回调，绘制整个场景。 */
void FltkCanvas::draw() {
    fl_color(rgb(0xffffff));
    fl_rectf(x(), y(), w(), h());
    fl_color(rgb(0xcfd6df));
    fl_rect(x(), y(), w(), h());

    drawGrid();
    drawObstacles();
    drawPoseMarker(map_.start(), 0x16a34a, "S");
    drawPoseMarker(map_.goal(), 0xdc2626, "G");
    drawPath();
    drawCurrentSearchBranches();

    // 绘制当前帧对应的车辆姿态
    if (!path_.empty()) {
        drawCar(path_[static_cast<std::size_t>(frame_)]);
    }
}

/** @brief 绘制栅格背景。 */
void FltkCanvas::drawGrid() const {
    fl_color(rgb(0xe5e7eb));
    fl_line_style(FL_SOLID, 1);

    // 垂直网格线
    for (int grid_x = 0; grid_x <= map_.width(); ++grid_x) {
        fl_line(
            static_cast<int>(std::round(worldX(grid_x))),
            static_cast<int>(std::round(worldY(0))),
            static_cast<int>(std::round(worldX(grid_x))),
            static_cast<int>(std::round(worldY(map_.height()))));
    }

    // 水平网格线
    for (int grid_y = 0; grid_y <= map_.height(); ++grid_y) {
        fl_line(
            static_cast<int>(std::round(worldX(0))),
            static_cast<int>(std::round(worldY(grid_y))),
            static_cast<int>(std::round(worldX(map_.width()))),
            static_cast<int>(std::round(worldY(grid_y))));
    }

    fl_line_style(0);
}

/** @brief 绘制所有障碍物栅格。 */
void FltkCanvas::drawObstacles() const {
    const double s = scale();
    fl_color(rgb(0x111827));
    for (int grid_y = 0; grid_y < map_.height(); ++grid_y) {
        for (int grid_x = 0; grid_x < map_.width(); ++grid_x) {
            if (!map_.isObstacle(grid_x, grid_y)) {
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

/**
 * @brief 绘制位姿标记（起点/终点）
 * @param[in] pose  2D位姿
 * @param[in] color RGB颜色值
 * @param[in] label 显示标签（S或G）
 */
void FltkCanvas::drawPoseMarker(
    const Pose2D& pose, unsigned int color, const char* label) const {
    const double s = scale();
    const int marker_radius = std::max(4, static_cast<int>(std::round(s * 0.36)));
    const int cx = static_cast<int>(std::round(worldX(pose.x)));
    const int cy = static_cast<int>(std::round(worldY(pose.y)));

    // 绘制圆形标记和方向线
    fl_color(rgb(color));
    fl_pie(cx - marker_radius, cy - marker_radius,
           marker_radius * 2, marker_radius * 2, 0.0, 360.0);
    fl_line_style(FL_SOLID, 3);
    fl_line(cx, cy,
            static_cast<int>(std::round(worldX(pose.x + std::cos(pose.theta) * 1.2))),
            static_cast<int>(std::round(worldY(pose.y + std::sin(pose.theta) * 1.2))));
    fl_line_style(0);

    // 绘制标签
    fl_color(rgb(0x111827));
    fl_font(FL_HELVETICA_BOLD, 14);
    const int label_width = static_cast<int>(fl_width(label));
    fl_draw(label, cx - label_width / 2,
            static_cast<int>(std::round(cy - s * 0.72)));
}

/** @brief 绘制从起点到当前帧的路径线段。 */
void FltkCanvas::drawPath() const {
    if (path_.empty()) {
        return;
    }

    fl_color(rgb(0x2563eb));
    fl_line_style(FL_SOLID, 3);
    const int limit = std::clamp(frame_, 0, frameCount() - 1);

    // 绘制到当前帧为止的所有路径线段
    for (int i = 1; i <= limit; ++i) {
        const CarPose& a = path_[static_cast<std::size_t>(i - 1)];
        const CarPose& b = path_[static_cast<std::size_t>(i)];
        fl_line(
            static_cast<int>(std::round(worldX(a.x))),
            static_cast<int>(std::round(worldY(a.y))),
            static_cast<int>(std::round(worldX(b.x))),
            static_cast<int>(std::round(worldY(b.y))));
    }

    fl_line_style(0);
}

std::size_t FltkCanvas::currentSolutionNodeIndex() const {
    if (solution_node_ids_.empty() || solution_path_frame_starts_.empty()) {
        return 0;
    }

    const int current_frame = std::clamp(frame_, 0, std::max(0, frameCount() - 1));
    std::size_t selected = 0;
    const std::size_t count = std::min(
        solution_node_ids_.size(), solution_path_frame_starts_.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (solution_path_frame_starts_[i] <= current_frame) {
            selected = i;
        } else {
            break;
        }
    }
    return selected;
}

void FltkCanvas::drawCurrentSearchBranches() const {
    if (solution_node_ids_.empty() || solution_path_frame_starts_.empty()) {
        return;
    }

    fl_color(rgb(0xdc2626));
    fl_line_style(FL_SOLID, 2);
    const std::size_t last_solution_index = currentSolutionNodeIndex();
    const std::size_t count = std::min(
        solution_node_ids_.size(), last_solution_index + 1);
    for (std::size_t solution_index = 0; solution_index < count; ++solution_index) {
        const int node_id = solution_node_ids_[solution_index];
        for (const SearchTreeEdge& edge : search_tree_) {
            if (edge.parent != node_id || !edge.accepted || edge.in_solution) {
                continue;
            }
            if (edge.segment.empty()) {
                fl_line(
                    static_cast<int>(std::round(worldX(edge.from.x))),
                    static_cast<int>(std::round(worldY(edge.from.y))),
                    static_cast<int>(std::round(worldX(edge.to.x))),
                    static_cast<int>(std::round(worldY(edge.to.y))));
                continue;
            }

            CarPose previous = edge.from;
            for (const CarPose& pose : edge.segment) {
                fl_line(
                    static_cast<int>(std::round(worldX(previous.x))),
                    static_cast<int>(std::round(worldY(previous.y))),
                    static_cast<int>(std::round(worldX(pose.x))),
                    static_cast<int>(std::round(worldY(pose.y))));
                previous = pose;
            }
        }
    }
    fl_line_style(0);
}

/**
 * @brief 绘制车辆轮廓和方向
 * @param[in] pose 车辆位姿
 */
void FltkCanvas::drawCar(const CarPose& pose) const {
    const double front = vehicle_.length - vehicle_.rear_to_center;
    const double rear = -vehicle_.rear_to_center;
    const double half_width = vehicle_.width * 0.5;
    const double c = std::cos(pose.theta);
    const double st = std::sin(pose.theta);

    // 车辆局部坐标系下的四个角点
    const double local[4][2] = {
        {front, half_width},
        {front, -half_width},
        {rear, -half_width},
        {rear, half_width}
    };

    // 变换到世界坐标
    int px[4]{};
    int py[4]{};
    for (int i = 0; i < 4; ++i) {
        const double wx = pose.x + local[i][0] * c - local[i][1] * st;
        const double wy = pose.y + local[i][0] * st + local[i][1] * c;
        px[i] = static_cast<int>(std::round(worldX(wx)));
        py[i] = static_cast<int>(std::round(worldY(wy)));
    }

    // 绘制车身填充
    fl_color(rgb(0xf97316));
    fl_polygon(px[0], py[0], px[1], py[1], px[2], py[2], px[3], py[3]);

    // 绘制车身轮廓
    fl_color(rgb(0x9a3412));
    fl_line_style(FL_SOLID, 2);
    for (int i = 0; i < 4; ++i) {
        const int next = (i + 1) % 4;
        fl_line(px[i], py[i], px[next], py[next]);
    }

    // 绘制车辆朝向线
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
