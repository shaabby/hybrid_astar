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
#include "HybridAstar.hpp"

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
               const std::vector<CarPose>& path,
               const std::vector<SearchTreeEdge>& search_tree,
               const std::vector<int>& solution_node_ids,
               const std::vector<int>& solution_open_orders,
               const std::vector<int>& solution_close_orders,
                       const std::vector<int>& solution_pop_orders,
               const std::vector<int>& solution_path_frame_starts);

    ~FltkCanvas() override;

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
    /** @brief 绘制截至当前最终解节点扩展出的未选中分支。 */
    void drawCurrentSearchBranches() const;
    /** @brief 返回当前帧对应的最终解节点序号。 */
    [[nodiscard]] std::size_t currentSolutionNodeIndex() const;
    /** @brief 绘制车辆轮廓。 */
    void drawCar(const CarPose& pose) const;

    /** @brief 确保离屏缓冲区存在且尺寸正确。 */
    void ensureOffscreen();
    /** @brief 将静态背景和搜索树渲染到离屏缓冲区。 */
    void renderOffscreen();

    const GridMap& map_;                 ///< 地图引用
    const VehicleConfig& vehicle_;       ///< 车辆配置引用
    const std::vector<CarPose>& path_;   ///< 路径采样点引用
    const std::vector<SearchTreeEdge>& search_tree_; ///< 搜索树边引用
    const std::vector<int>& solution_node_ids_; ///< 最终解节点 id 引用
    const std::vector<int>& solution_open_orders_; ///< 解节点open顺序引用
    const std::vector<int>& solution_close_orders_; ///< 解节点close顺序引用
    const std::vector<int>& solution_pop_orders_; ///< 解节点pop顺序引用
    const std::vector<int>& solution_path_frame_starts_; ///< 解节点路径帧引用
    int frame_ = 0;                      ///< 当前帧号

    // 离屏缓冲区缓存
    Fl_Offscreen composite_offscreen_ = 0;
    std::size_t cached_tree_index_ = static_cast<std::size_t>(-1);
    int cached_w_ = 0;
    int cached_h_ = 0;

    // 坐标原点偏移，支持离屏绘制 (0,0) 和屏幕绘制 (x(), y())
    mutable int draw_origin_x_ = 0;
    mutable int draw_origin_y_ = 0;
};
