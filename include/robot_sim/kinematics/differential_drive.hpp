#pragma once

#include <cmath>

#include "robot_sim/geometry/pose2d.hpp"
#include "robot_sim/geometry/twist2d.hpp"

namespace robot_sim {

struct WheelCommand {
    double left_rad_s{0.0};
    double right_rad_s{0.0};
    bool clipped{false};

    [[nodiscard]] bool is_finite() const {
        return std::isfinite(left_rad_s) && std::isfinite(right_rad_s);
    }
};

struct DifferentialDriveParameters {
    double wheel_radius_m{0.05};
    double wheelbase_m{0.30};
    double max_wheel_rad_s{12.0};
};

class DifferentialDriveKinematics {
public:
    explicit DifferentialDriveKinematics(DifferentialDriveParameters parameters);

    [[nodiscard]] const DifferentialDriveParameters& parameters() const {
        return parameters_;
    }

    [[nodiscard]] Twist2D forward(const WheelCommand& wheels) const;
    [[nodiscard]] WheelCommand inverse(const Twist2D& twist) const;

private:
    DifferentialDriveParameters parameters_;
};

[[nodiscard]] Pose2D integrate_twist(
    const Pose2D& pose,
    const Twist2D& twist,
    double dt,
    double angular_epsilon = 1.0e-9
);

}  // namespace robot_sim
