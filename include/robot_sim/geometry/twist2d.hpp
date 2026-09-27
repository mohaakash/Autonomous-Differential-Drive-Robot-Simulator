#pragma once

#include <cmath>

namespace robot_sim {

struct Twist2D {
    double linear_x{0.0};
    double angular_z{0.0};

    constexpr Twist2D() = default;
    constexpr Twist2D(double linear_value, double angular_value)
        : linear_x(linear_value), angular_z(angular_value) {}

    [[nodiscard]] bool is_finite() const {
        return std::isfinite(linear_x) && std::isfinite(angular_z);
    }
};

}  // namespace robot_sim
