#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "robot_sim/geometry/angle.hpp"
#include "robot_sim/geometry/transform2d.hpp"
#include "robot_sim/kinematics/differential_drive.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

constexpr double kTolerance = 1.0e-12;

void expect_near(double actual, double expected, double tolerance = kTolerance) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_NEAR(actual, expected, tolerance);
#else
    assert(std::abs(actual - expected) <= tolerance);
#endif
}

void expect_true(bool value) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_TRUE(value);
#else
    assert(value);
#endif
}

template <typename Function>
void expect_invalid_argument(Function&& function) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_THROW(function(), std::invalid_argument);
#else
    bool threw = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
#endif
}

void test_angle_helpers() {
    expect_near(robot_sim::wrap_to_pi(0.0), 0.0);
    expect_near(robot_sim::wrap_to_pi(robot_sim::kPi), -robot_sim::kPi);
    expect_near(robot_sim::wrap_to_pi(-robot_sim::kPi), -robot_sim::kPi);
    expect_near(robot_sim::wrap_to_pi(1.5 * robot_sim::kPi), -0.5 * robot_sim::kPi);
    expect_near(
        robot_sim::shortest_angular_distance(0.9 * robot_sim::kPi,
                                              -0.9 * robot_sim::kPi),
        0.2 * robot_sim::kPi
    );
    expect_invalid_argument([] {
        (void)robot_sim::wrap_to_pi(std::numeric_limits<double>::quiet_NaN());
    });
}

void test_transform_helpers() {
    const robot_sim::Transform2D transform(
        robot_sim::Vector2{2.0, 1.0}, 0.5 * robot_sim::kPi
    );
    const robot_sim::Vector2 point{1.0, 0.0};
    const robot_sim::Vector2 transformed = transform.apply(point);
    expect_near(transformed.x, 2.0);
    expect_near(transformed.y, 2.0);

    const robot_sim::Vector2 recovered = transform.apply_inverse(transformed);
    expect_near(recovered.x, point.x);
    expect_near(recovered.y, point.y);

    const robot_sim::Transform2D identity = transform * transform.inverse();
    const robot_sim::Vector2 identity_point = identity.apply(point);
    expect_near(identity_point.x, point.x);
    expect_near(identity_point.y, point.y);
    expect_near(identity.yaw, 0.0);
}

robot_sim::DifferentialDriveKinematics make_kinematics() {
    return robot_sim::DifferentialDriveKinematics({0.1, 0.4, 5.0});
}

void test_forward_and_inverse_kinematics() {
    const auto kinematics = make_kinematics();

    const robot_sim::Twist2D straight = kinematics.forward({2.0, 2.0, false});
    expect_near(straight.linear_x, 0.2);
    expect_near(straight.angular_z, 0.0);

    const robot_sim::Twist2D rotation = kinematics.forward({-2.0, 2.0, false});
    expect_near(rotation.linear_x, 0.0);
    expect_near(rotation.angular_z, 1.0);

    const robot_sim::Twist2D desired{0.30, -0.5};
    const robot_sim::WheelCommand wheels = kinematics.inverse(desired);
    expect_true(!wheels.clipped);
    const robot_sim::Twist2D recovered = kinematics.forward(wheels);
    expect_near(recovered.linear_x, desired.linear_x);
    expect_near(recovered.angular_z, desired.angular_z);
}

void test_wheel_saturation() {
    const auto kinematics = make_kinematics();
    const robot_sim::WheelCommand clipped = kinematics.inverse({1.0, 4.0});
    expect_true(clipped.clipped);
    expect_true(std::abs(clipped.left_rad_s) <= 5.0);
    expect_true(std::abs(clipped.right_rad_s) <= 5.0);
    expect_near(clipped.left_rad_s / clipped.right_rad_s, 2.0 / 18.0);
}

void test_pose_integration() {
    const robot_sim::Pose2D origin{};
    const robot_sim::Pose2D straight = robot_sim::integrate_twist(
        origin, {1.0, 0.0}, 0.5
    );
    expect_near(straight.x, 0.5);
    expect_near(straight.y, 0.0);
    expect_near(straight.theta, 0.0);

    const robot_sim::Pose2D arc = robot_sim::integrate_twist(
        origin, {1.0, 1.0}, 0.5 * robot_sim::kPi
    );
    expect_near(arc.x, 1.0);
    expect_near(arc.y, 1.0);
    expect_near(arc.theta, 0.5 * robot_sim::kPi);

    const robot_sim::Pose2D wrapped = robot_sim::integrate_twist(
        {0.0, 0.0, robot_sim::kPi - 0.1}, {0.0, 1.0}, 0.2
    );
    expect_near(wrapped.theta, -robot_sim::kPi + 0.1);
}

void test_validation() {
    expect_invalid_argument([] {
        robot_sim::DifferentialDriveKinematics({0.0, 0.4, 5.0});
    });
    expect_invalid_argument([] {
        robot_sim::DifferentialDriveKinematics({0.1, -0.4, 5.0});
    });
    expect_invalid_argument([] {
        const auto kinematics = make_kinematics();
        (void)kinematics.forward({std::numeric_limits<double>::infinity(), 0.0});
    });
    expect_invalid_argument([] {
        (void)robot_sim::integrate_twist({}, {0.0, 0.0}, 0.0);
    });
}

void run_all_kinematics_tests() {
    test_angle_helpers();
    test_transform_helpers();
    test_forward_and_inverse_kinematics();
    test_wheel_saturation();
    test_pose_integration();
    test_validation();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase1Kinematics, AngleHelpers) {
    test_angle_helpers();
}

TEST(Phase1Kinematics, TransformHelpers) {
    test_transform_helpers();
}

TEST(Phase1Kinematics, ForwardAndInverse) {
    test_forward_and_inverse_kinematics();
}

TEST(Phase1Kinematics, WheelSaturation) {
    test_wheel_saturation();
}

TEST(Phase1Kinematics, PoseIntegration) {
    test_pose_integration();
}

TEST(Phase1Kinematics, Validation) {
    test_validation();
}

#else

void run_kinematics_tests() {
    run_all_kinematics_tests();
}

#endif
