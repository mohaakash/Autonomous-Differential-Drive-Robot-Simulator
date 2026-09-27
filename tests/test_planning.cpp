#include <cassert>
#include <cmath>
#include <sstream>

#include "robot_sim/planning/a_star.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

robot_sim::OccupancyGrid parse_map(const char* contents) {
    std::istringstream input(contents);
    return robot_sim::OccupancyGrid::parse(input, "planner-test-map");
}

robot_sim::OccupancyGrid make_empty_map() {
    return parse_map(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 7\n"
        "height 7\n"
        "data\n"
        "#######\n"
        "#.....#\n"
        "#.....#\n"
        "#..S..#\n"
        "#.....#\n"
        "#...G.#\n"
        "#######\n"
    );
}

robot_sim::OccupancyGrid make_obstacle_map() {
    return parse_map(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 9\n"
        "height 7\n"
        "data\n"
        "#########\n"
        "#.......#\n"
        "#..###..#\n"
        "#..#....#\n"
        "#S.#.G..#\n"
        "#.......#\n"
        "#########\n"
    );
}

robot_sim::OccupancyGrid make_unreachable_map() {
    return parse_map(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 7\n"
        "height 3\n"
        "data\n"
        "#######\n"
        "#S###G#\n"
        "#######\n"
    );
}

robot_sim::OccupancyGrid make_inflation_map() {
    return parse_map(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 7\n"
        "height 7\n"
        "data\n"
        "#######\n"
        "#.....#\n"
        "#.....#\n"
        "#..#..#\n"
        "#.....#\n"
        "#.....#\n"
        "#######\n"
    );
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

void test_empty_map_path() {
    const robot_sim::OccupancyGrid map = make_empty_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult result = planner.plan_cells({3, 3}, {5, 4});

    expect_true(result.success());
    expect_true(result.status == robot_sim::PlanStatus::Success);
    expect_true(result.path.cells.front() == robot_sim::GridCell{3, 3});
    expect_true(result.path.cells.back() == robot_sim::GridCell{5, 4});
    expect_true(result.path.cells.size() == result.path.waypoints.size());
    expect_true(result.path.length_m > 0.0);

    const robot_sim::PlanResult repeated = planner.plan_cells({3, 3}, {5, 4});
    expect_true(result.path.cells == repeated.path.cells);

    const robot_sim::Vector2 start = map.cell_center({3, 3});
    const robot_sim::Vector2 goal = map.cell_center({5, 4});
    const robot_sim::PlanResult world_result = planner.plan_world(start, goal);
    expect_true(world_result.success());
    expect_true(world_result.path.cells == result.path.cells);

    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    for (std::size_t index = 0; index < result.path.cells.size(); ++index) {
        expect_true(planner.is_planning_free(result.path.cells[index]));
        if (index == 0) {
            continue;
        }
        const robot_sim::Vector2 previous = result.path.waypoints[index - 1];
        const robot_sim::Vector2 current = result.path.waypoints[index];
        expect_true(collision_checker.is_motion_valid(
            {previous.x, previous.y, 0.0},
            {current.x, current.y, 0.0}
        ));
    }
}

void test_obstacle_route_and_inflation() {
    const robot_sim::OccupancyGrid map = make_obstacle_map();
    const robot_sim::AStarPlanner planner(map, 0.20);
    const robot_sim::PlanResult result = planner.plan_cells({4, 1}, {4, 5});

    expect_true(result.success());
    expect_true(result.path.cells.size() >= 5);
    for (const robot_sim::GridCell cell : result.path.cells) {
        expect_true(planner.is_planning_free(cell));
        expect_false(cell == robot_sim::GridCell{4, 3});
    }

    const robot_sim::OccupancyGrid inflation_map = make_inflation_map();
    const robot_sim::AStarPlanner inflated_planner(inflation_map, 0.60);
    expect_false(inflated_planner.is_planning_free({3, 2}));
    expect_false(inflated_planner.is_planning_free({2, 3}));
    expect_true(inflated_planner.is_planning_free({2, 2}));
}

void test_unreachable_and_invalid_requests() {
    const robot_sim::OccupancyGrid map = make_unreachable_map();
    const robot_sim::AStarPlanner planner(map, 0.20);

    const robot_sim::PlanResult unreachable = planner.plan_cells({1, 1}, {1, 5});
    expect_true(unreachable.status == robot_sim::PlanStatus::Unreachable);
    expect_false(unreachable.success());
    expect_true(unreachable.path.cells.empty());

    const robot_sim::PlanResult start_outside = planner.plan_cells({-1, 1}, {1, 5});
    expect_true(start_outside.status == robot_sim::PlanStatus::StartOutsideMap);

    const robot_sim::PlanResult goal_outside = planner.plan_cells({1, 1}, {3, 5});
    expect_true(goal_outside.status == robot_sim::PlanStatus::GoalOutsideMap);

    const robot_sim::PlanResult start_occupied = planner.plan_cells({0, 0}, {1, 5});
    expect_true(start_occupied.status == robot_sim::PlanStatus::StartOccupied);

    expect_true(
        robot_sim::plan_status_name(robot_sim::PlanStatus::Unreachable) ==
        "unreachable"
    );
}

void run_all_planning_tests() {
    test_empty_map_path();
    test_obstacle_route_and_inflation();
    test_unreachable_and_invalid_requests();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase3Planning, EmptyMapPath) {
    test_empty_map_path();
}

TEST(Phase3Planning, ObstacleRouteAndInflation) {
    test_obstacle_route_and_inflation();
}

TEST(Phase3Planning, UnreachableAndInvalidRequests) {
    test_unreachable_and_invalid_requests();
}

#else

void run_planning_tests() {
    run_all_planning_tests();
}

#endif
