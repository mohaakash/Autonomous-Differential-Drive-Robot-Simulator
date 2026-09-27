#include <cassert>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "robot_sim/collision/collision_checker.hpp"
#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/experiments/experiment_runner.hpp"
#include "robot_sim/safety/forward_safety_monitor.hpp"
#include "robot_sim/sensors/lidar.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

robot_sim::OccupancyGrid make_hardening_map() {
    std::istringstream input(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 5\n"
        "height 5\n"
        "data\n"
        "#####\n"
        "#...#\n"
        "#...#\n"
        "#...#\n"
        "#####\n"
    );
    return robot_sim::OccupancyGrid::parse(input, "hardening-map");
}

template <typename Function>
void expect_runtime_error(Function&& function) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_THROW(function(), std::runtime_error);
#else
    bool threw = false;
    try {
        function();
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
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

void test_malformed_map_and_sensor_inputs() {
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 1\norigin 0 0\nwidth 2\nheight 2\n##\n##\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "missing-data");
    });
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 0\norigin 0 0\nwidth 1\nheight 1\ndata\n#\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "zero-resolution");
    });

    const robot_sim::OccupancyGrid map = make_hardening_map();
    expect_invalid_argument([&map] {
        robot_sim::LidarParameters parameters;
        parameters.ray_count = 0;
        [[maybe_unused]] robot_sim::LidarSensor sensor(map, parameters);
    });
    expect_invalid_argument([&map] {
        robot_sim::LidarParameters parameters;
        parameters.angle_min_rad = 1.0;
        parameters.angle_max_rad = -1.0;
        [[maybe_unused]] robot_sim::LidarSensor sensor(map, parameters);
    });
    expect_invalid_argument([&map] {
        robot_sim::LidarParameters parameters;
        parameters.noise_stddev_m = -0.1;
        [[maybe_unused]] robot_sim::LidarSensor sensor(map, parameters);
    });

    robot_sim::LidarSensor sensor(map, {});
    expect_invalid_argument([&sensor] {
        (void)sensor.measure(
            {std::numeric_limits<double>::quiet_NaN(), 1.5, 0.0}
        );
    });
    expect_invalid_argument([&sensor] {
        (void)sensor.measure({1.5, 1.5, 0.0}, std::numeric_limits<double>::infinity());
    });
    expect_invalid_argument([&sensor] {
        (void)sensor.measure(
            {1.5, 1.5, 0.0},
            0.0,
            {{2.0, 1.0, 1.0, 2.0}}
        );
    });
}

void test_invalid_safety_collision_and_simulation_inputs() {
    expect_invalid_argument([] {
        [[maybe_unused]] robot_sim::ForwardSafetyMonitor monitor({-0.1, 0.3});
    });
    expect_invalid_argument([] {
        [[maybe_unused]] robot_sim::ForwardSafetyMonitor monitor({0.2, 0.0});
    });

    const robot_sim::OccupancyGrid map = make_hardening_map();
    const robot_sim::CollisionChecker checker(map, {0.2});
    expect_invalid_argument([&checker] {
        (void)checker.is_pose_valid(
            {1.5, 1.5, 0.0},
            {{1.0, 1.0, 1.0, 2.0}}
        );
    });
    expect_invalid_argument([&checker] {
        (void)checker.is_motion_valid(
            {1.5, 1.5, 0.0},
            {2.0, 1.5, 0.0},
            {},
            std::numeric_limits<double>::quiet_NaN()
        );
    });

    const robot_sim::PurePursuitController controller({
        0.5, 0.15, 0.3, 0.5, 2.0, 2.0, 1.5, 0.1, 0.12,
    });
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    expect_invalid_argument([&] {
        [[maybe_unused]] const robot_sim::FixedStepSimulator simulator(
            controller,
            kinematics,
            checker,
            {0.02, 1.0, robot_sim::SafetyPolicy::Stop}
        );
    });
    expect_invalid_argument([] {
        robot_sim::ExperimentConfig config;
        config.robot_radius_m = -0.2;
        (void)robot_sim::run_experiment(config);
    });
}

void run_all_hardening_tests() {
    test_malformed_map_and_sensor_inputs();
    test_invalid_safety_collision_and_simulation_inputs();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase7Hardening, MalformedMapAndSensorInputs) {
    test_malformed_map_and_sensor_inputs();
}

TEST(Phase7Hardening, InvalidSafetyCollisionAndSimulationInputs) {
    test_invalid_safety_collision_and_simulation_inputs();
}

#else

void run_hardening_tests() {
    run_all_hardening_tests();
}

#endif
