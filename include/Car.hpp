#pragma once

#include <array>

/**
 * @brief 车辆位姿，包含后轴中心坐标、航向角、转向角和行驶方向。
 */
struct CarPose {
    double x = 0.0;       ///< 后轴中心 x 坐标
    double y = 0.0;       ///< 后轴中心 y 坐标
    double theta = 0.0;   ///< 车身航向角，弧度
    double steer = 0.0;   ///< 前轮转向角，弧度
    int direction = 1;    ///< 1 = 前进，-1 = 倒车
};

/**
 * @brief 车辆物理配置参数。
 */
struct VehicleConfig {
    double length;          ///< 车身长度
    double width;           ///< 车身宽度
    double wheelbase;       ///< 轴距
    double rear_to_center;  ///< 后轴到车身中心的距离
    double max_steer;       ///< 最大前轮转向角，弧度
};

/**
 * @brief 车辆运动学模型。
 *
 * 使用简化自行车模型进行前向模拟，提供步进积分、转向角限幅和车身角点计算。
 */
class Car {
public:
    /**
     * @brief 使用自定义配置构造车辆。
     * @param[in] config 车辆物理参数
     */
    explicit Car(VehicleConfig config);

    [[nodiscard]] double length() const;
    [[nodiscard]] double width() const;
    [[nodiscard]] double wheelbase() const;
    [[nodiscard]] double rearToCenter() const;
    [[nodiscard]] double maxSteer() const;

    /** @brief 最小转弯半径，由轴距和最大转向角推导。 */
    [[nodiscard]] double minTurningRadius() const;

    [[nodiscard]] const VehicleConfig& config() const;

    /**
     * @brief 将转向角限幅到 [-max_steer, +max_steer] 范围内。
     * @param[in] steer 原始转向角，弧度
     * @return 限幅后的转向角
     */
    [[nodiscard]] double clampSteer(double steer) const;

    /**
     * @brief 沿给定控制量前进一步。
     * @param[in] pose        当前位姿
     * @param[in] steer       前轮转向角，弧度
     * @param[in] direction   行驶方向，1 = 前进，-1 = 倒车
     * @param[in] distance    行驶距离
     * @return 下一步的位姿
     */
    [[nodiscard]] CarPose step(const CarPose& pose,
                               double steer,
                               int direction,
                               double distance) const;

    /**
     * @brief 计算车身四个角在世界坐标系中的位置。
     *
     * 角点从车辆局部坐标系的前左开始，按顺时针顺序返回：
     * 前左 → 前右 → 后右 → 后左。
     *
     * @param[in] pose 车辆位姿
     * @return 4 个角点的世界坐标，每个点为 {x, y}
     */
    [[nodiscard]] std::array<std::array<double, 2>, 4> bodyCorners(
        const CarPose& pose) const;

private:
    VehicleConfig config_;
};
