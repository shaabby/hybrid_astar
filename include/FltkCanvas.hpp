/**
 * @file FltkCanvas.hpp
 * @brief FLTK绘图画布组件定义
 *
 * 基于FLTK的widget实现，负责在窗口中渲染栅格地图、
 * 障碍物、车辆位姿标记和路径曲线。
 */

#pragma once

#include "VisualizationData.hpp"

#include <FL/Fl_Widget.H>

class FltkCanvas : public Fl_Widget {
public:
    FltkCanvas(int x, int y, int w, int h, const VisualizationData& data);

    void setFrame(int frame);
    [[nodiscard]] int frame() const;
    [[nodiscard]] int frameCount() const;

private:
    void draw() override;

    [[nodiscard]] double scale() const;
    [[nodiscard]] double worldX(double value) const;
    [[nodiscard]] double worldY(double value) const;

    void drawGrid() const;
    void drawObstacles() const;
    void drawPoseMarker(const Pose2D& pose, unsigned int color, const char* label) const;
    void drawPath() const;
    void drawCar(const CarPose& pose) const;

    const VisualizationData& data_;
    int frame_ = 0;
};
