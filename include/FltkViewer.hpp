#pragma once

#include "FltkCanvas.hpp"
#include "VisualizationData.hpp"

#include <FL/Fl_Box.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Hor_Value_Slider.H>

#include <memory>
#include <string>

class FltkViewer {
public:
    explicit FltkViewer(const VisualizationData& data);

    int run();

private:
    static void toggleCallback(Fl_Widget* widget, void* user_data);
    static void stepCallback(Fl_Widget* widget, void* user_data);
    static void resetCallback(Fl_Widget* widget, void* user_data);
    static void sliderCallback(Fl_Widget* widget, void* user_data);
    static void timerCallback(void* user_data);

    void togglePlayback();
    void step();
    void reset();
    void setFrame(int frame);
    void syncControls();
    void tick();

    const VisualizationData& data_;
    std::unique_ptr<Fl_Double_Window> window_;
    FltkCanvas* canvas_ = nullptr;
    Fl_Button* toggle_button_ = nullptr;
    Fl_Button* step_button_ = nullptr;
    Fl_Button* reset_button_ = nullptr;
    Fl_Hor_Value_Slider* slider_ = nullptr;
    Fl_Box* frame_label_ = nullptr;
    bool playing_ = true;
    std::string frame_label_text_;
};
