/**
 * @file FltkViewer.hpp
 * @brief FLTK规划结果查看器定义
 *
 * 提供基于FLTK的交互式路径规划可视化界面，
 * 支持播放控制、时间轴滑块和帧步进。
 */

#pragma once

#include "Car.hpp"
#include "FltkCanvas.hpp"
#include "GridMap.hpp"

#include <FL/Fl_Box.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Hor_Value_Slider.H>

#include <memory>
#include <string>

/**
 * @brief FLTK交互式规划结果查看器
 *
 * 提供播放/暂停、步进、重置按钮和时间轴滑块，
 * 用于动画展示Hybrid A*路径规划结果。
 */
class FltkViewer {
public:
    /**
     * @brief 构造查看器
     * @param[in] map      地图引用
     * @param[in] vehicle  车辆配置引用
     * @param[in] path     路径采样点引用
     */
    FltkViewer(const GridMap& map,
               const VehicleConfig& vehicle,
               const std::vector<CarPose>& path,
               const std::vector<SearchTreeEdge>& search_tree,
               const std::vector<int>& solution_node_ids,
               const std::vector<int>& solution_open_orders,
               const std::vector<int>& solution_close_orders,
               const std::vector<int>& solution_path_frame_starts);

    /** @brief 进入FLTK事件循环，返回窗口关闭状态。 */
    int run();

private:
    /** @brief 播放/暂停按钮回调。 */
    static void toggleCallback(Fl_Widget* widget, void* user_data);
    /** @brief 步进按钮回调。 */
    static void stepCallback(Fl_Widget* widget, void* user_data);
    /** @brief 重置按钮回调。 */
    static void resetCallback(Fl_Widget* widget, void* user_data);
    /** @brief 滑块值改变回调。 */
    static void sliderCallback(Fl_Widget* widget, void* user_data);
    /** @brief 定时器回调，用于自动播放。 */
    static void timerCallback(void* user_data);

    /** @brief 切换播放/暂停状态。 */
    void togglePlayback();
    /** @brief 前进一帧。 */
    void step();
    /** @brief 重置到第一帧。 */
    void reset();
    /** @brief 设置当前帧号。 */
    void setFrame(int frame);
    /** @brief 同步控件状态。 */
    void syncControls();
    /** @brief 定时触发器，播放下一帧。 */
    void tick();

    const GridMap& map_;                                 ///< 地图引用
    const VehicleConfig& vehicle_;                       ///< 车辆配置引用
    const std::vector<CarPose>& path_;                   ///< 路径采样点引用
    const std::vector<SearchTreeEdge>& search_tree_;     ///< 搜索树边引用
    const std::vector<int>& solution_node_ids_;          ///< 最终解节点 id 引用
    const std::vector<int>& solution_open_orders_;       ///< 解节点open顺序引用
    const std::vector<int>& solution_close_orders_;      ///< 解节点close顺序引用
    const std::vector<int>& solution_path_frame_starts_; ///< 解节点路径帧引用
    std::unique_ptr<Fl_Double_Window> window_;          ///< 主窗口
    FltkCanvas* canvas_ = nullptr;                      ///< 画布组件
    Fl_Button* toggle_button_ = nullptr;                 ///< 播放/暂停按钮
    Fl_Button* step_button_ = nullptr;                  ///< 步进按钮
    Fl_Button* reset_button_ = nullptr;                 ///< 重置按钮
    Fl_Hor_Value_Slider* slider_ = nullptr;              ///< 时间轴滑块
    Fl_Box* frame_label_ = nullptr;                      ///< 帧号标签
    bool playing_ = true;                                ///< 是否正在播放
    std::string frame_label_text_;                       ///< 帧号文本缓存
};
