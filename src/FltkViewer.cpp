/**
 * @file FltkViewer.cpp
 * @brief FLTK规划结果查看器实现
 *
 * 实现基于FLTK的交互式路径规划可视化界面，
 * 支持播放控制、时间轴滑块和帧步进。
 */

#include "FltkViewer.hpp"

#include <FL/Fl.H>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

/// @brief 每帧持续时间（秒），30fps
constexpr double kFrameSeconds = 1.0 / 30.0;

/**
 * @brief 获取路径最后一帧的索引
 * @param[in] path 路径采样点
 * @return 最后一帧索引，无路径时返回0
 */
int lastFrame(const std::vector<CarPose>& path) {
    return std::max(0, static_cast<int>(path.size()) - 1);
}

} // namespace

/**
 * @brief 构造FLTK查看器窗口
 * @param[in] map      地图引用
 * @param[in] vehicle  车辆配置引用
 * @param[in] path     路径采样点引用
 *
 * 创建窗口、按钮、滑块和画布组件，设置回调函数。
 */
FltkViewer::FltkViewer(const GridMap& map,
                       const VehicleConfig& vehicle,
                       const std::vector<CarPose>& path,
                       const std::vector<SearchTreeEdge>& search_tree,
                       const std::vector<int>& solution_node_ids,
                       const std::vector<int>& solution_open_orders,
                       const std::vector<int>& solution_close_orders,
                       const std::vector<int>& solution_pop_orders,
                       const std::vector<int>& solution_path_frame_starts)
    : map_(map),
      vehicle_(vehicle),
      path_(path),
      search_tree_(search_tree),
      solution_node_ids_(solution_node_ids),
      solution_open_orders_(solution_open_orders),
      solution_close_orders_(solution_close_orders),
      solution_pop_orders_(solution_pop_orders),
      solution_path_frame_starts_(solution_path_frame_starts) {
    // 窗口尺寸常量
    constexpr int window_w = 1160;
    constexpr int window_h = 820;
    constexpr int pad = 16;
    constexpr int top_h = 50;
    constexpr int button_w = 86;
    constexpr int button_h = 34;
    constexpr int slider_w = 280;
    constexpr int label_w = 90;

    // 创建主窗口
    window_ = std::make_unique<Fl_Double_Window>(
        window_w, window_h, "Hybrid A* Path Planning Demo");

    // 标题
    Fl_Box* title = new Fl_Box(pad, 10, 360, 32, "Hybrid A* Path Planning Demo");
    title->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    title->labelfont(FL_HELVETICA_BOLD);
    title->labelsize(20);

    // 计算控件布局位置
    const int controls_x = window_w - pad - label_w - slider_w
        - button_w * 3 - pad * 4;

    // 创建按钮
    toggle_button_ = new Fl_Button(controls_x, 10, button_w, button_h, "Pause");
    step_button_ = new Fl_Button(
        controls_x + button_w + pad, 10, button_w, button_h, "Step");
    reset_button_ = new Fl_Button(
        controls_x + (button_w + pad) * 2, 10, button_w, button_h, "Reset");

    // 创建滑块
    slider_ = new Fl_Hor_Value_Slider(
        controls_x + (button_w + pad) * 3, 10, slider_w, button_h);

    // 创建帧号标签
    frame_label_ = new Fl_Box(
        controls_x + (button_w + pad) * 3 + slider_w + pad,
        10, label_w, button_h);
    frame_label_->align(FL_ALIGN_RIGHT | FL_ALIGN_INSIDE);

    // 配置滑块
    slider_->type(FL_HORIZONTAL);
    slider_->bounds(0.0, static_cast<double>(lastFrame(path_)));
    slider_->step(1.0);
    slider_->value(0.0);

    // 创建画布
    canvas_ = new FltkCanvas(
        pad, top_h + pad, window_w - pad * 2, window_h - top_h - pad * 2,
        map_, vehicle_, path_, search_tree_, solution_node_ids_,
        solution_open_orders_, solution_close_orders_,
        solution_pop_orders_,
        solution_path_frame_starts_);

    // 绑定回调
    toggle_button_->callback(toggleCallback, this);
    step_button_->callback(stepCallback, this);
    reset_button_->callback(resetCallback, this);
    slider_->callback(sliderCallback, this);

    window_->resizable(canvas_);
    window_->end();
    syncControls();
}

/** @brief 进入FLTK事件循环。 */
int FltkViewer::run() {
    window_->show();
    Fl::add_timeout(kFrameSeconds, timerCallback, this);
    return Fl::run();
}

/** @brief 播放/暂停按钮回调。 */
void FltkViewer::toggleCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->togglePlayback();
}

/** @brief 步进按钮回调。 */
void FltkViewer::stepCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->step();
}

/** @brief 重置按钮回调。 */
void FltkViewer::resetCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->reset();
}

/** @brief 滑块值改变回调。 */
void FltkViewer::sliderCallback(Fl_Widget*, void* user_data) {
    auto* viewer = static_cast<FltkViewer*>(user_data);
    viewer->playing_ = false;
    viewer->setFrame(static_cast<int>(std::round(viewer->slider_->value())));
}

/** @brief 定时器回调。 */
void FltkViewer::timerCallback(void* user_data) {
    auto* viewer = static_cast<FltkViewer*>(user_data);
    viewer->tick();
    Fl::repeat_timeout(kFrameSeconds, timerCallback, user_data);
}

/** @brief 切换播放/暂停状态。 */
void FltkViewer::togglePlayback() {
    if (!playing_ && canvas_->frame() >= lastFrame(path_)) {
        setFrame(0);
    }
    playing_ = !playing_;
    syncControls();
}

/** @brief 前进一帧。 */
void FltkViewer::step() {
    playing_ = false;
    setFrame(canvas_->frame() + 1);
}

/** @brief 重置到第一帧。 */
void FltkViewer::reset() {
    playing_ = false;
    setFrame(0);
}

/** @brief 设置当前帧号并同步控件。 */
void FltkViewer::setFrame(int frame) {
    canvas_->setFrame(std::clamp(frame, 0, lastFrame(path_)));
    syncControls();
}

/** @brief 同步所有控件状态到当前帧。 */
void FltkViewer::syncControls() {
    const int total = std::max(1, canvas_->frameCount());
    const int current = std::clamp(canvas_->frame() + 1, 1, total);

    // 更新帧号标签
    char buffer[64]{};
    std::snprintf(buffer, sizeof(buffer), "%d / %d", current, total);
    frame_label_text_ = buffer;
    frame_label_->label(frame_label_text_.c_str());

    // 更新滑块值和按钮文字
    slider_->value(static_cast<double>(canvas_->frame()));
    toggle_button_->label(playing_ ? "Pause" : "Start");
    window_->redraw();
}

/** @brief 自动播放 tick，每帧调用一次。 */
void FltkViewer::tick() {
    if (!playing_ || path_.empty()) {
        return;
    }

    if (canvas_->frame() < lastFrame(path_)) {
        setFrame(canvas_->frame() + 1);
    } else {
        playing_ = false;
        syncControls();
    }
}
