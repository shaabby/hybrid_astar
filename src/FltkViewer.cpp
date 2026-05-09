#include "FltkViewer.hpp"

#include <FL/Fl.H>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr double kFrameSeconds = 1.0 / 30.0;

int lastFrame(const VisualizationData& data) {
    return std::max(0, static_cast<int>(data.path.size()) - 1);
}

} // namespace

FltkViewer::FltkViewer(const VisualizationData& data)
    : data_(data) {
    constexpr int window_w = 1160;
    constexpr int window_h = 820;
    constexpr int pad = 16;
    constexpr int top_h = 50;
    constexpr int button_w = 86;
    constexpr int button_h = 34;
    constexpr int slider_w = 280;
    constexpr int label_w = 90;

    window_ = std::make_unique<Fl_Double_Window>(
        window_w, window_h, "Hybrid A* Path Planning Demo");

    Fl_Box* title = new Fl_Box(pad, 10, 360, 32, "Hybrid A* Path Planning Demo");
    title->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
    title->labelfont(FL_HELVETICA_BOLD);
    title->labelsize(20);

    const int controls_x = window_w - pad - label_w - slider_w
        - button_w * 3 - pad * 4;
    toggle_button_ = new Fl_Button(controls_x, 10, button_w, button_h, "Pause");
    step_button_ = new Fl_Button(
        controls_x + button_w + pad, 10, button_w, button_h, "Step");
    reset_button_ = new Fl_Button(
        controls_x + (button_w + pad) * 2, 10, button_w, button_h, "Reset");
    slider_ = new Fl_Hor_Value_Slider(
        controls_x + (button_w + pad) * 3, 10, slider_w, button_h);
    frame_label_ = new Fl_Box(
        controls_x + (button_w + pad) * 3 + slider_w + pad,
        10, label_w, button_h);
    frame_label_->align(FL_ALIGN_RIGHT | FL_ALIGN_INSIDE);

    slider_->type(FL_HORIZONTAL);
    slider_->bounds(0.0, static_cast<double>(lastFrame(data_)));
    slider_->step(1.0);
    slider_->value(0.0);

    canvas_ = new FltkCanvas(
        pad, top_h + pad, window_w - pad * 2, window_h - top_h - pad * 2, data_);

    toggle_button_->callback(toggleCallback, this);
    step_button_->callback(stepCallback, this);
    reset_button_->callback(resetCallback, this);
    slider_->callback(sliderCallback, this);

    window_->resizable(canvas_);
    window_->end();
    syncControls();
}

int FltkViewer::run() {
    window_->show();
    Fl::add_timeout(kFrameSeconds, timerCallback, this);
    return Fl::run();
}

void FltkViewer::toggleCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->togglePlayback();
}

void FltkViewer::stepCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->step();
}

void FltkViewer::resetCallback(Fl_Widget*, void* user_data) {
    static_cast<FltkViewer*>(user_data)->reset();
}

void FltkViewer::sliderCallback(Fl_Widget*, void* user_data) {
    auto* viewer = static_cast<FltkViewer*>(user_data);
    viewer->playing_ = false;
    viewer->setFrame(static_cast<int>(std::round(viewer->slider_->value())));
}

void FltkViewer::timerCallback(void* user_data) {
    auto* viewer = static_cast<FltkViewer*>(user_data);
    viewer->tick();
    Fl::repeat_timeout(kFrameSeconds, timerCallback, user_data);
}

void FltkViewer::togglePlayback() {
    if (!playing_ && canvas_->frame() >= lastFrame(data_)) {
        setFrame(0);
    }
    playing_ = !playing_;
    syncControls();
}

void FltkViewer::step() {
    playing_ = false;
    setFrame(canvas_->frame() + 1);
}

void FltkViewer::reset() {
    playing_ = false;
    setFrame(0);
}

void FltkViewer::setFrame(int frame) {
    canvas_->setFrame(std::clamp(frame, 0, lastFrame(data_)));
    syncControls();
}

void FltkViewer::syncControls() {
    const int total = std::max(1, canvas_->frameCount());
    const int current = std::clamp(canvas_->frame() + 1, 1, total);

    char buffer[64]{};
    std::snprintf(buffer, sizeof(buffer), "%d / %d", current, total);
    frame_label_text_ = buffer;
    frame_label_->label(frame_label_text_.c_str());

    slider_->value(static_cast<double>(canvas_->frame()));
    toggle_button_->label(playing_ ? "Pause" : "Start");
    window_->redraw();
}

void FltkViewer::tick() {
    if (!playing_ || data_.path.empty()) {
        return;
    }

    if (canvas_->frame() < lastFrame(data_)) {
        setFrame(canvas_->frame() + 1);
    } else {
        playing_ = false;
        syncControls();
    }
}
