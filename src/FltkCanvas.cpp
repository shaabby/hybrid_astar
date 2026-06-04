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
#include <limits>
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
                       const std::vector<int>& solution_open_orders,
                       const std::vector<int>& solution_close_orders,
                       const std::vector<int>& solution_pop_orders,
                       const std::vector<int>& solution_path_frame_starts)
    : Fl_Widget(x, y, w, h),
      map_(map),
      vehicle_(vehicle),
      path_(path),
      search_tree_(search_tree),
      solution_node_ids_(solution_node_ids),
      solution_open_orders_(solution_open_orders),
      solution_close_orders_(solution_close_orders),
      solution_pop_orders_(solution_pop_orders),
      solution_path_frame_starts_(solution_path_frame_starts) {}

FltkCanvas::~FltkCanvas() {
    if (static_offscreen_) {
        fl_delete_offscreen(static_offscreen_);
        static_offscreen_ = 0;
    }
    if (frame_offscreen_) {
        fl_delete_offscreen(frame_offscreen_);
        frame_offscreen_ = 0;
    }
}

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
    return static_cast<double>(draw_origin_x_) + kMargin + value * scale();
}

/**
 * @brief 将世界坐标Y转换为屏幕坐标
 * @param[in] value 世界坐标Y值
 * @return 屏幕坐标
 *
 * Y轴翻转：世界坐标系Y向上为正，画布Y向下为正。
 */
double FltkCanvas::worldY(double value) const {
    return static_cast<double>(draw_origin_y_) + kMargin
        + (static_cast<double>(map_.height()) - value) * scale();
}

/** @brief FLTK重绘回调，利用分层离屏缓冲区实现增量路径绘制。 */
void FltkCanvas::draw() {
    if (w() <= 0 || h() <= 0) {
        return;
    }

    const std::size_t current_idx = currentSolutionNodeIndex();

    // 在以下情况重建静态离屏缓冲区：
    //   - 尚未创建
    //   - 窗口尺寸改变
    //   - 搜索树的可视范围改变（解节点切换）
    if (!static_offscreen_
        || w() != cached_w_
        || h() != cached_h_
        || cached_tree_index_ != current_idx) {

        ensureStaticOffscreen();
        if (static_offscreen_) {
            renderStaticOffscreen();
        }
        cached_w_ = w();
        cached_h_ = h();
        cached_tree_index_ = current_idx;

        // 静态层变化 → 帧缓冲作废，下次强制重建
        deleteFrameOffscreen();
        path_applied_to_frame_ = -1;
    }

    if (static_offscreen_ && !path_.empty()) {
        // 正常路径：blit 帧缓冲（含静态内容 + 路径轨迹）→ 屏幕
        ensureFrameOffscreen();
        if (frame_offscreen_) {
            if (path_applied_to_frame_ < 0 || frame_ < path_applied_to_frame_) {
                // 首次绘制 / 向后拖拽 → 重建帧缓冲（静态底图 + 全量路径）
                resetFrameBuffer();
                if (frame_ > 0) {
                    applyPathIncremental(1, frame_);
                }
                path_applied_to_frame_ = frame_;
            } else if (frame_ > path_applied_to_frame_) {
                // 向前播放 → 仅增量绘制新增线段
                applyPathIncremental(path_applied_to_frame_ + 1, frame_);
                path_applied_to_frame_ = frame_;
            }
            fl_copy_offscreen(x(), y(), w(), h(), frame_offscreen_, 0, 0);
        }
    } else if (static_offscreen_) {
        // 空路径 → 直接 blit 静态层
        fl_copy_offscreen(x(), y(), w(), h(), static_offscreen_, 0, 0);
    } else {
        // 离屏缓冲区创建失败时的 fallback 路径
        draw_origin_x_ = x();
        draw_origin_y_ = y();
        fl_color(rgb(0xffffff));
        fl_rectf(x(), y(), w(), h());
        fl_color(rgb(0xcfd6df));
        fl_rect(x(), y(), w(), h());
        drawGrid();
        drawObstacles();
        drawPoseMarker(map_.start(), 0x16a34a, "S");
        drawPoseMarker(map_.goal(), 0xdc2626, "G");
        drawCurrentSearchBranches();
        drawPath();
    }

    // 动态元素：车辆（直接画屏，每帧位置不同）
    draw_origin_x_ = x();
    draw_origin_y_ = y();
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
    if (search_tree_.empty() || solution_open_orders_.empty()
        || solution_pop_orders_.empty()) {
        return;
    }

    fl_color(rgb(0xdc2626));
    fl_line_style(FL_SOLID, 1);

    const std::size_t solution_index = currentSolutionNodeIndex();
    // 打开窗口：累积画到下一个解节点进入 open 之前
    const int open_end = solution_index + 1 < solution_open_orders_.size()
        ? solution_open_orders_[solution_index + 1]
        : std::numeric_limits<int>::max();
    // 剪枝窗口：剪除已弹出但未进入 closed 的边
    const int prune_end = solution_index < solution_pop_orders_.size()
        ? solution_pop_orders_[solution_index]
        : std::numeric_limits<int>::max();

    for (const SearchTreeEdge& edge : search_tree_) {
        if (!edge.accepted || edge.in_solution
            || edge.open_order <= 0
            || edge.open_order >= open_end) {
            continue;
        }
        if (edge.close_order < 0 && edge.pop_order > 0
            && edge.pop_order <= prune_end) {
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

void FltkCanvas::ensureStaticOffscreen() {
    if (static_offscreen_) {
        fl_delete_offscreen(static_offscreen_);
        static_offscreen_ = 0;
    }
    if (w() > 0 && h() > 0) {
        static_offscreen_ = fl_create_offscreen(w(), h());
    }
}

void FltkCanvas::renderStaticOffscreen() {
    fl_begin_offscreen(static_offscreen_);

    draw_origin_x_ = 0;
    draw_origin_y_ = 0;

    fl_color(rgb(0xffffff));
    fl_rectf(0, 0, w(), h());
    fl_color(rgb(0xcfd6df));
    fl_rect(0, 0, w(), h());
    drawGrid();
    drawObstacles();
    drawPoseMarker(map_.start(), 0x16a34a, "S");
    drawPoseMarker(map_.goal(), 0xdc2626, "G");
    drawCurrentSearchBranches();

    fl_end_offscreen();
}

void FltkCanvas::ensureFrameOffscreen() {
    if (frame_offscreen_) {
        return;
    }
    if (w() > 0 && h() > 0) {
        frame_offscreen_ = fl_create_offscreen(w(), h());
    }
}

void FltkCanvas::resetFrameBuffer() {
    fl_begin_offscreen(frame_offscreen_);
    draw_origin_x_ = 0;
    draw_origin_y_ = 0;
    fl_copy_offscreen(0, 0, w(), h(), static_offscreen_, 0, 0);
    fl_end_offscreen();
}

void FltkCanvas::applyPathIncremental(int from_frame, int to_frame) {
    fl_begin_offscreen(frame_offscreen_);
    draw_origin_x_ = 0;
    draw_origin_y_ = 0;

    fl_color(rgb(0x2563eb));
    fl_line_style(FL_SOLID, 3);

    const int frame_count = frameCount();
    const int from = std::max(1, from_frame);
    const int to = std::clamp(to_frame, from, frame_count - 1);

    for (int i = from; i <= to; ++i) {
        const CarPose& a = path_[static_cast<std::size_t>(i - 1)];
        const CarPose& b = path_[static_cast<std::size_t>(i)];
        fl_line(
            static_cast<int>(std::round(worldX(a.x))),
            static_cast<int>(std::round(worldY(a.y))),
            static_cast<int>(std::round(worldX(b.x))),
            static_cast<int>(std::round(worldY(b.y))));
    }

    fl_line_style(0);
    fl_end_offscreen();
}

void FltkCanvas::deleteFrameOffscreen() {
    if (frame_offscreen_) {
        fl_delete_offscreen(frame_offscreen_);
        frame_offscreen_ = 0;
    }
}
