#pragma once

#include <stdexcept>

#include "robot_sim/geometry/angle.hpp"
#include "robot_sim/geometry/pose2d.hpp"

namespace robot_sim {

struct Transform2D {
    Vector2 translation{};
    double yaw{0.0};

    constexpr Transform2D() = default;
    constexpr Transform2D(Vector2 translation_value, double yaw_value)
        : translation(translation_value), yaw(yaw_value) {}

    [[nodiscard]] Vector2 apply(Vector2 point) const {
        validate();
        return translation + rotate(point, yaw);
    }

    [[nodiscard]] Vector2 apply_inverse(Vector2 point) const {
        validate();
        return rotate(point - translation, -yaw);
    }

    [[nodiscard]] Transform2D inverse() const {
        validate();
        return {rotate(-translation, -yaw), wrap_to_pi(-yaw)};
    }

    [[nodiscard]] Transform2D operator*(const Transform2D& rhs) const {
        validate();
        rhs.validate();
        return {apply(rhs.translation), wrap_to_pi(yaw + rhs.yaw)};
    }

    [[nodiscard]] static Transform2D from_pose(const Pose2D& pose) {
        if (!pose.is_finite()) {
            throw std::invalid_argument("pose must be finite");
        }
        return {{pose.x, pose.y}, wrap_to_pi(pose.theta)};
    }

private:
    void validate() const {
        if (!translation.is_finite() || !std::isfinite(yaw)) {
            throw std::invalid_argument("transform must be finite");
        }
    }
};

}  // namespace robot_sim
