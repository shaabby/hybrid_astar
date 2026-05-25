/**
 * @file FltkCanvas.hpp
 * @brief FLTK绘图画布组件定义
 *
 * 基于FLTK的widget实现，负责在窗口中渲染栅格地图、
 * 障碍物、车辆位姿标记和路径曲线。
 */

#pragma once

#include "Car.hpp"
#include "GridMap.hpp"

#include <FL/Fl_Widget.H>

#include <vector>

class FltkCanvas : public Fl_Widget {
public:
    /**
     * @brief 构造画布
     * @param[in] x     窗口x坐标
     * @param[in] y     窗口y坐标
     * @param[in] w     宽度
     * @param[in] h     高度
     * @param[in] map      地图引用
     * @param[in] vehicle  车辆配置
     * @param[in] path     路径采样点
     */
    FltkCanvas(int x,
               int y,
               int w,
               int h,
               const GridMap& map,
               const VehicleConfig& vehicle,
               const std::vector<CarPose>& path);

    /**
     * @brief 设置当前显示帧号
     * @param[in] frame 帧号索引
     */
    void setFrame(int frame);

    /** @brief 返回当前帧号。 */
    [[nodiscard]] int frame() const;

    /** @brief 返回总帧数。 */
    [[nodiscard]] int frameCount() const;

private:
    void draw() override;

    [[nodiscard]] double scale() const;
    [[nodiscard]] double worldX(double value) const;
    [[nodiscard]] double worldY(double value) const;

    /** @brief 绘制栅格背景。 */
    void drawGrid() const;
    /** @brief 绘制障碍物栅格。 */
    void drawObstacles() const;
    /** @brief 绘制车辆位姿标记。 */
    void drawPoseMarker(const Pose2D& pose, unsigned int color, const char* label) const;
    /** @brief 绘制规划路径曲线。 */
    void drawPath() const;
    /** @brief 绘制车辆轮廓。 */
    void drawCar(const CarPose& pose) const;

    const GridMap& map_;                 ///< 地图引用
    const VehicleConfig& vehicle_;       ///< 车辆配置引用
    const std::vector<CarPose>& path_;   ///< 路径采样点引用
    int frame_ = 0;                      ///< 当前帧号
};
