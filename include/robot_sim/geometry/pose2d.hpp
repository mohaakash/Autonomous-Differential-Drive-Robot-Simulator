#pragma once

#include "robot_sim/geometry/vector2.hpp"

namespace robot_sim {

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    constexpr Pose2D() = default;
    constexpr Pose2D(double x_value, double y_value, double theta_value)
        : x(x_value), y(y_value), theta(theta_value) {}

    [[nodiscard]] constexpr Vector2 position() const { return {x, y}; }
    [[nodiscard]] bool is_finite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(theta);
    }
};

}  // namespace robot_sim
