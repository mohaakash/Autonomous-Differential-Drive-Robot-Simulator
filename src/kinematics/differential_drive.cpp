#include "robot_sim/kinematics/differential_drive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "robot_sim/geometry/angle.hpp"

namespace robot_sim {
namespace {

void validate_parameters(const DifferentialDriveParameters& parameters) {
    if (!std::isfinite(parameters.wheel_radius_m) ||
        !std::isfinite(parameters.wheelbase_m) ||
        !std::isfinite(parameters.max_wheel_rad_s)) {
        throw std::invalid_argument("kinematics parameters must be finite");
    }
    if (parameters.wheel_radius_m <= 0.0) {
        throw std::invalid_argument("wheel radius must be greater than zero");
    }
    if (parameters.wheelbase_m <= 0.0) {
        throw std::invalid_argument("wheelbase must be greater than zero");
    }
    if (parameters.max_wheel_rad_s <= 0.0) {
        throw std::invalid_argument("maximum wheel speed must be greater than zero");
    }
}

void validate_twist(const Twist2D& twist) {
    if (!twist.is_finite()) {
        throw std::invalid_argument("twist must be finite");
    }
}

}  // namespace

DifferentialDriveKinematics::DifferentialDriveKinematics(
    DifferentialDriveParameters parameters
)
    : parameters_(parameters) {
    validate_parameters(parameters_);
}

Twist2D DifferentialDriveKinematics::forward(const WheelCommand& wheels) const {
    if (!wheels.is_finite()) {
        throw std::invalid_argument("wheel command must be finite");
    }

    const double radius = parameters_.wheel_radius_m;
    const double wheelbase = parameters_.wheelbase_m;
    return {
        radius * (wheels.right_rad_s + wheels.left_rad_s) / 2.0,
        radius * (wheels.right_rad_s - wheels.left_rad_s) / wheelbase,
    };
}

WheelCommand DifferentialDriveKinematics::inverse(const Twist2D& twist) const {
    validate_twist(twist);

    const double radius = parameters_.wheel_radius_m;
    const double half_wheelbase = parameters_.wheelbase_m / 2.0;
    double left = (twist.linear_x - half_wheelbase * twist.angular_z) / radius;
    double right = (twist.linear_x + half_wheelbase * twist.angular_z) / radius;

    const double maximum = std::max(std::abs(left), std::abs(right));
    bool clipped = false;
    if (maximum > parameters_.max_wheel_rad_s) {
        const double scale = parameters_.max_wheel_rad_s / maximum;
        left *= scale;
        right *= scale;
        clipped = true;
    }

    return {left, right, clipped};
}

Pose2D integrate_twist(
    const Pose2D& pose,
    const Twist2D& twist,
    double dt,
    double angular_epsilon
) {
    if (!pose.is_finite()) {
        throw std::invalid_argument("pose must be finite");
    }
    validate_twist(twist);
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("time step must be finite and greater than zero");
    }
    if (!std::isfinite(angular_epsilon) || angular_epsilon <= 0.0) {
        throw std::invalid_argument(
            "angular epsilon must be finite and greater than zero"
        );
    }

    Pose2D result = pose;
    if (std::abs(twist.angular_z) < angular_epsilon) {
        result.x += twist.linear_x * std::cos(pose.theta) * dt;
        result.y += twist.linear_x * std::sin(pose.theta) * dt;
    } else {
        const double delta_theta = twist.angular_z * dt;
        const double radius = twist.linear_x / twist.angular_z;
        result.x += radius *
            (std::sin(pose.theta + delta_theta) - std::sin(pose.theta));
        result.y -= radius *
            (std::cos(pose.theta + delta_theta) - std::cos(pose.theta));
    }

    result.theta = wrap_to_pi(pose.theta + twist.angular_z * dt);
    return result;
}

}  // namespace robot_sim
