#include <cassert>
#include <cmath>
#include <sstream>

#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/geometry/angle.hpp"
#include "robot_sim/safety/forward_safety_monitor.hpp"
#include "robot_sim/sensors/lidar.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

robot_sim::OccupancyGrid make_sensor_map() {
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
    return robot_sim::OccupancyGrid::parse(input, "sensor-test-map");
}

robot_sim::OccupancyGrid make_simulation_map() {
    std::istringstream input(
        "resolution 0.5\n"
        "origin 0.0 0.0\n"
        "width 12\n"
        "height 8\n"
        "data\n"
        "############\n"
        "#..........#\n"
        "#..........#\n"
        "#..........#\n"
        "#..........#\n"
        "#..........#\n"
        "#..........#\n"
        "############\n"
    );
    return robot_sim::OccupancyGrid::parse(input, "sensor-simulation-test-map");
}

robot_sim::LidarParameters make_lidar_parameters() {
    return {
        {},
        0.0,
        -0.5 * robot_sim::kPi,
        0.5 * robot_sim::kPi,
        3,
        0.02,
        10.0,
        0.0,
        42,
    };
}

void expect_true(bool value) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_TRUE(value);
#else
    assert(value);
#endif
}

void expect_false(bool value) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_FALSE(value);
#else
    assert(!value);
#endif
}

void expect_near(double actual, double expected, double tolerance) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_NEAR(actual, expected, tolerance);
#else
    assert(std::abs(actual - expected) <= tolerance);
#endif
}

void test_lidar_rays_and_mount_transform() {
    const robot_sim::OccupancyGrid map = make_sensor_map();
    robot_sim::LidarSensor sensor(map, make_lidar_parameters());
    const robot_sim::LaserScan scan = sensor.measure({2.5, 2.5, 0.0});

    expect_true(scan.ranges_m.size() == 3);
    expect_true(scan.has_return.size() == 3);
    expect_near(scan.angle_at(0), -0.5 * robot_sim::kPi, 1.0e-12);
    expect_near(scan.angle_at(1), 0.0, 1.0e-12);
    expect_near(scan.angle_at(2), 0.5 * robot_sim::kPi, 1.0e-12);
    for (std::size_t index = 0; index < scan.ranges_m.size(); ++index) {
        expect_near(scan.ranges_m[index], 1.5, 1.0e-12);
        expect_true(scan.has_return[index] != 0);
    }

    robot_sim::LidarParameters mounted_parameters = make_lidar_parameters();
    mounted_parameters.mount_translation = {0.5, 0.0};
    mounted_parameters.ray_count = 1;
    mounted_parameters.angle_min_rad = 0.0;
    mounted_parameters.angle_max_rad = 0.0;
    robot_sim::LidarSensor mounted_sensor(map, mounted_parameters);
    const robot_sim::LaserScan mounted_scan = mounted_sensor.measure(
        {2.5, 2.5, 0.0}
    );
    expect_near(mounted_scan.ranges_m.front(), 1.0, 1.0e-12);
}

void test_lidar_dynamic_obstacle_and_seeded_noise() {
    const robot_sim::OccupancyGrid map = make_sensor_map();
    robot_sim::LidarParameters parameters = make_lidar_parameters();
    parameters.ray_count = 1;
    parameters.angle_min_rad = 0.0;
    parameters.angle_max_rad = 0.0;
    parameters.noise_stddev_m = 0.10;

    const std::vector<robot_sim::AxisAlignedRectangle> obstacles{
        {3.0, 2.0, 3.5, 3.0},
    };
    robot_sim::LidarSensor first_sensor(map, parameters);
    robot_sim::LidarSensor second_sensor(map, parameters);
    const robot_sim::LaserScan first = first_sensor.measure(
        {2.5, 2.5, 0.0},
        1.0,
        obstacles
    );
    const robot_sim::LaserScan second = second_sensor.measure(
        {2.5, 2.5, 0.0},
        1.0,
        obstacles
    );
    expect_true(first.has_return.front() != 0);
    expect_near(first.ranges_m.front(), second.ranges_m.front(), 1.0e-12);
    expect_true(first.ranges_m.front() >= parameters.range_min_m);
    expect_true(first.ranges_m.front() <= parameters.range_max_m);

    parameters.noise_stddev_m = 0.0;
    robot_sim::LidarSensor exact_sensor(map, parameters);
    const robot_sim::LaserScan exact = exact_sensor.measure(
        {2.5, 2.5, 0.0},
        0.0,
        obstacles
    );
    expect_near(exact.ranges_m.front(), 0.5, 1.0e-12);
}

void test_forward_safety_validation_and_decisions() {
    const robot_sim::ForwardSafetyMonitor monitor({0.20, 0.60});
    const robot_sim::LaserScan blocked{
        0.0,
        -0.5,
        0.5,
        0.02,
        10.0,
        {2.0, 0.50, 2.0},
        {1, 1, 1},
    };
    const robot_sim::SafetyDecision blocked_decision = monitor.evaluate(blocked);
    expect_true(blocked_decision.status == robot_sim::SafetyStatus::ObstacleAhead);
    expect_true(blocked_decision.blocking_ray_index == 1);
    expect_near(blocked_decision.minimum_forward_range_m, 0.50, 1.0e-12);

    const robot_sim::LaserScan no_return{
        0.0,
        0.0,
        0.0,
        0.02,
        10.0,
        {10.0},
        {0},
    };
    expect_true(
        monitor.evaluate(no_return).status == robot_sim::SafetyStatus::Safe
    );

    const robot_sim::LaserScan invalid{
        0.0,
        0.0,
        0.0,
        0.02,
        10.0,
        {},
        {},
    };
    expect_true(
        monitor.evaluate(invalid).status == robot_sim::SafetyStatus::InvalidScan
    );
}

robot_sim::PurePursuitController make_controller() {
    return robot_sim::PurePursuitController({
        0.75,
        0.20,
        0.40,
        0.50,
        2.0,
        2.0,
        1.5,
        0.10,
        0.12,
    });
}

robot_sim::SimulationResult run_safety_case(robot_sim::SafetyPolicy policy) {
    const robot_sim::OccupancyGrid map = make_simulation_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult plan = planner.plan_cells({5, 2}, {5, 8});
    assert(plan.success());

    const robot_sim::Vector2 start = map.cell_center({5, 2});
    const robot_sim::Vector2 goal = map.cell_center({5, 8});
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    robot_sim::LidarParameters lidar_parameters{
        {},
        0.0,
        -0.50,
        0.50,
        5,
        0.02,
        5.0,
        0.0,
        7,
    };
    robot_sim::LidarSensor lidar(map, lidar_parameters);
    robot_sim::ForwardSafetyMonitor safety_monitor({0.50, 0.60});
    const std::vector<robot_sim::DynamicObstacleEvent> events{
        {5, {2.0, 1.0, 2.5, 1.5}},
    };
    const robot_sim::FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 25.0, policy},
        &lidar,
        &safety_monitor,
        events
    );
    return simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        plan.path
    );
}

void test_safety_stop_precedes_collision() {
    const robot_sim::SimulationResult result = run_safety_case(
        robot_sim::SafetyPolicy::Stop
    );
    expect_true(result.status == robot_sim::SimulationStatus::SafetyStop);
    expect_true(result.safety_stop_count == 1);
    expect_false(result.trace.empty());
    expect_true(result.trace.back().safety_blocked);
    expect_false(result.trace.back().collision);
}

void test_replan_failure_is_reported() {
    const robot_sim::OccupancyGrid map = make_simulation_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult plan = planner.plan_cells({5, 2}, {5, 8});
    assert(plan.success());
    const robot_sim::Vector2 start = map.cell_center({5, 2});
    const robot_sim::Vector2 goal = map.cell_center({5, 8});
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    robot_sim::LidarParameters lidar_parameters{
        {}, 0.0, -0.50, 0.50, 5, 0.02, 5.0, 0.0, 11,
    };
    robot_sim::LidarSensor lidar(map, lidar_parameters);
    robot_sim::ForwardSafetyMonitor safety_monitor({0.50, 0.60});
    const robot_sim::FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 1.0, robot_sim::SafetyPolicy::Replan},
        &lidar,
        &safety_monitor,
        {{0, {1.8, 1.0, 2.3, 1.5}}}
    );

    const robot_sim::SimulationResult result = simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        plan.path
    );
    expect_true(result.status == robot_sim::SimulationStatus::ReplanFailure);
    expect_true(result.replan_count == 0);
    expect_true(result.trace.size() == 1);
    expect_true(result.trace.front().safety_blocked);
}

void test_replan_callback_recovers_with_turning_step() {
    const robot_sim::OccupancyGrid map = make_simulation_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult plan = planner.plan_cells({5, 2}, {5, 8});
    assert(plan.success());
    const robot_sim::Vector2 start = map.cell_center({5, 2});
    const robot_sim::Vector2 goal = map.cell_center({5, 8});
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    robot_sim::LidarParameters lidar_parameters{
        {}, 0.0, -0.50, 0.50, 5, 0.02, 5.0, 0.0, 13,
    };
    robot_sim::LidarSensor lidar(map, lidar_parameters);
    robot_sim::ForwardSafetyMonitor safety_monitor({0.50, 0.60});
    const std::vector<robot_sim::DynamicObstacleEvent> events{
        {0, {1.8, 1.0, 2.3, 1.5}},
    };
    std::size_t callback_count = 0;
    const robot_sim::ReplanCallback replan =
        [&callback_count, start, goal](
            const robot_sim::Pose2D&,
            const robot_sim::Pose2D&,
            const std::vector<robot_sim::AxisAlignedRectangle>&
        ) -> std::optional<robot_sim::PlannedPath> {
            ++callback_count;
            return robot_sim::PlannedPath{
                {},
                {
                    start,
                    {start.x, 2.0},
                    {goal.x, 2.0},
                    goal,
                },
                0.0,
            };
        };
    const robot_sim::FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 25.0, robot_sim::SafetyPolicy::Replan},
        &lidar,
        &safety_monitor,
        events,
        replan
    );

    const robot_sim::SimulationResult result = simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        plan.path
    );
    expect_true(result.status == robot_sim::SimulationStatus::GoalReached);
    expect_true(result.replan_count >= 1);
    expect_true(callback_count >= 1);
    expect_true(result.trace.size() > 1);
    expect_true(result.trace.front().safety_blocked);
}

void run_all_sensor_and_safety_tests() {
    test_lidar_rays_and_mount_transform();
    test_lidar_dynamic_obstacle_and_seeded_noise();
    test_forward_safety_validation_and_decisions();
    test_safety_stop_precedes_collision();
    test_replan_failure_is_reported();
    test_replan_callback_recovers_with_turning_step();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase5Sensors, LidarRaysAndMountTransform) {
    test_lidar_rays_and_mount_transform();
}

TEST(Phase5Sensors, LidarDynamicObstacleAndSeededNoise) {
    test_lidar_dynamic_obstacle_and_seeded_noise();
}

TEST(Phase5Safety, ForwardSafetyValidationAndDecisions) {
    test_forward_safety_validation_and_decisions();
}

TEST(Phase5Safety, SafetyStopPrecedesCollision) {
    test_safety_stop_precedes_collision();
}

TEST(Phase5Safety, ReplanFailureIsReported) {
    test_replan_failure_is_reported();
}

TEST(Phase5Safety, ReplanCallbackRecoversWithTurningStep) {
    test_replan_callback_recovers_with_turning_step();
}

#else

void run_sensor_and_safety_tests() {
    run_all_sensor_and_safety_tests();
}

#endif
