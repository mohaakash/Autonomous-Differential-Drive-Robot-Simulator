#include <cassert>
#include <cmath>
#include <sstream>

#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/geometry/angle.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

robot_sim::OccupancyGrid make_static_map() {
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
    return robot_sim::OccupancyGrid::parse(input, "simulation-test-map");
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

void test_controller_status_and_limits() {
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::PlannedPath path{
        {},
        {{1.0, 1.0}, {2.0, 1.0}, {3.0, 1.0}},
        2.0,
    };

    const robot_sim::ControlOutput tracking = controller.update(
        {1.0, 1.0, 0.0},
        path,
        {3.0, 1.0, 0.0}
    );
    expect_true(tracking.status == robot_sim::ControllerStatus::Tracking);
    expect_true(tracking.command.linear_x > 0.0);
    expect_near(tracking.command.angular_z, 0.0, 1.0e-12);
    expect_true(tracking.command.linear_x <= 0.50);
    expect_true(std::abs(tracking.command.angular_z) <= 2.0);

    const robot_sim::ControlOutput rotate = controller.update(
        {2.95, 1.0, 0.0},
        path,
        {3.0, 1.0, 0.5 * robot_sim::kPi}
    );
    expect_true(rotate.status == robot_sim::ControllerStatus::Tracking);
    expect_near(rotate.command.linear_x, 0.0, 1.0e-12);
    expect_true(rotate.command.angular_z > 0.0);

    const robot_sim::ControlOutput reached = controller.update(
        {3.0, 1.0, 0.5 * robot_sim::kPi},
        path,
        {3.0, 1.0, 0.5 * robot_sim::kPi}
    );
    expect_true(reached.status == robot_sim::ControllerStatus::GoalReached);
    expect_near(reached.command.linear_x, 0.0, 1.0e-12);
    expect_near(reached.command.angular_z, 0.0, 1.0e-12);

    const robot_sim::ControlOutput invalid = controller.update(
        {0.0, 0.0, 0.0},
        {},
        {1.0, 1.0, 0.0}
    );
    expect_true(invalid.status == robot_sim::ControllerStatus::InvalidInput);
}

robot_sim::SimulationResult run_case(
    const robot_sim::OccupancyGrid& map,
    robot_sim::GridCell start_cell,
    robot_sim::GridCell goal_cell
) {
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult plan = planner.plan_cells(start_cell, goal_cell);
    assert(plan.success());

    const robot_sim::Vector2 start_point = map.cell_center(start_cell);
    const robot_sim::Vector2 goal_point = map.cell_center(goal_cell);
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    const robot_sim::FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 25.0}
    );
    return simulator.run(
        {start_point.x, start_point.y, 0.0},
        {goal_point.x, goal_point.y, 0.0},
        plan.path
    );
}

void test_multiple_static_map_goals() {
    const robot_sim::OccupancyGrid map = make_static_map();
    const robot_sim::SimulationResult straight = run_case(map, {5, 2}, {5, 8});
    const robot_sim::SimulationResult diagonal = run_case(map, {5, 2}, {2, 8});

    for (const robot_sim::SimulationResult& result : {straight, diagonal}) {
        expect_true(result.status == robot_sim::SimulationStatus::GoalReached);
        expect_true(result.trace.size() > 1);
        expect_true(result.travelled_distance_m > 0.0);
        expect_true(result.final_goal_position_error_m <= 0.10);
        expect_true(result.final_heading_error_rad <= 0.12);
        expect_true(result.command_clip_count == 0);
        for (const robot_sim::TraceSample& sample : result.trace) {
            expect_false(sample.collision);
            expect_true(std::abs(sample.requested_twist.angular_z) <= 2.0 + 1.0e-12);
            expect_true(sample.requested_twist.linear_x <= 0.50 + 1.0e-12);
        }
    }
}

void test_timeout_and_invalid_initial_pose() {
    const robot_sim::OccupancyGrid map = make_static_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult plan = planner.plan_cells({5, 2}, {5, 8});
    const robot_sim::PurePursuitController controller = make_controller();
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});

    const robot_sim::FixedStepSimulator short_simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 0.02}
    );
    const robot_sim::Vector2 start = map.cell_center({5, 2});
    const robot_sim::Vector2 goal = map.cell_center({5, 8});
    const robot_sim::SimulationResult timeout = short_simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        plan.path
    );
    expect_true(timeout.status == robot_sim::SimulationStatus::Timeout);

    const robot_sim::FixedStepSimulator normal_simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 1.0}
    );
    const robot_sim::SimulationResult invalid = normal_simulator.run(
        {0.1, 0.1, 0.0},
        {goal.x, goal.y, 0.0},
        plan.path
    );
    expect_true(invalid.status == robot_sim::SimulationStatus::InvalidInput);
}

void run_all_simulation_tests() {
    test_controller_status_and_limits();
    test_multiple_static_map_goals();
    test_timeout_and_invalid_initial_pose();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase4Simulation, ControllerStatusAndLimits) {
    test_controller_status_and_limits();
}

TEST(Phase4Simulation, MultipleStaticMapGoals) {
    test_multiple_static_map_goals();
}

TEST(Phase4Simulation, TimeoutAndInvalidInitialPose) {
    test_timeout_and_invalid_initial_pose();
}

#else

void run_simulation_tests() {
    run_all_simulation_tests();
}

#endif
