#pragma once

#include <array>

struct CarPose {
    double x = 0.0;       // Rear axle center x.
    double y = 0.0;       // Rear axle center y.
    double theta = 0.0;   // Body heading, radians.
    double steer = 0.0;   // Front wheel steering angle, radians.
    int direction = 1;    // 1 = forward, -1 = reverse.
};

struct VehicleConfig {
    double length = 4.5;
    double width = 2.0;
    double wheelbase = 2.7;
    double rear_to_center = 1.35;
    double max_steer = 0.61;
};

class Car {
public:
    Car();
    explicit Car(VehicleConfig config);

    [[nodiscard]] double length() const;
    [[nodiscard]] double width() const;
    [[nodiscard]] double wheelbase() const;
    [[nodiscard]] double rearToCenter() const;
    [[nodiscard]] double maxSteer() const;
    [[nodiscard]] double minTurningRadius() const;
    [[nodiscard]] const VehicleConfig& config() const;

    [[nodiscard]] double clampSteer(double steer) const;
    [[nodiscard]] CarPose step(const CarPose& pose,
                               double steer,
                               int direction,
                               double distance) const;

    // Corners are returned in world coordinates, starting at front-left and
    // going clockwise in the vehicle local frame.
    [[nodiscard]] std::array<std::array<double, 2>, 4> bodyCorners(const CarPose& pose) const;

private:
    VehicleConfig config_;
};
