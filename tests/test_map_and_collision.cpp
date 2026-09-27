#include <cassert>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "robot_sim/collision/collision_checker.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

robot_sim::OccupancyGrid make_test_map() {
    std::istringstream input(
        "resolution 1.0\n"
        "origin 0.0 0.0\n"
        "width 6\n"
        "height 5\n"
        "data\n"
        "######\n"
        "#....#\n"
        "#..#G#\n"
        "#S...#\n"
        "######\n"
    );
    return robot_sim::OccupancyGrid::parse(input, "test-map");
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

void expect_near(double actual, double expected) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_NEAR(actual, expected, 1.0e-12);
#else
    assert(std::abs(actual - expected) <= 1.0e-12);
#endif
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

void test_map_loading_and_coordinates() {
    const robot_sim::OccupancyGrid map = make_test_map();
    expect_true(map.width() == 6);
    expect_true(map.height() == 5);
    expect_near(map.resolution_m(), 1.0);
    expect_true(map.is_occupied({0, 0}));
    expect_true(map.is_free({1, 1}));
    expect_true(map.is_occupied({2, 3}));
    expect_true(map.is_occupied({-1, 0}));

    expect_true(map.start_marker().has_value());
    expect_true(map.goal_marker().has_value());
    expect_true(*map.start_marker() == robot_sim::GridCell{3, 1});
    expect_true(*map.goal_marker() == robot_sim::GridCell{2, 4});

    const robot_sim::Vector2 start_center = map.cell_center({3, 1});
    expect_near(start_center.x, 1.5);
    expect_near(start_center.y, 1.5);
    const auto start_cell = map.world_to_cell(start_center);
    expect_true(start_cell.has_value());
    expect_true(*start_cell == robot_sim::GridCell{3, 1});

    expect_false(map.world_to_cell({-0.01, 1.0}).has_value());
    expect_false(map.world_to_cell({6.0, 1.0}).has_value());
    expect_false(map.world_to_cell({1.0, 5.0}).has_value());
}

void test_malformed_maps() {
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 1\norigin 0 0\nwidth 2\nheight 1\ndata\n#?\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "invalid-symbol");
    });
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 1\norigin 0 0\nwidth 2\nheight 1\ndata\n##\n..\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "extra-row");
    });
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 1\norigin 0 0\nwidth 2\nheight 1\ndata\n#\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "wrong-width");
    });
    expect_runtime_error([] {
        std::istringstream input(
            "resolution 1\norigin 0 0\nwidth 2\nheight 1\ndata\nSS\n"
        );
        (void)robot_sim::OccupancyGrid::parse(input, "duplicate-start");
    });
}

void test_collision_and_motion() {
    const robot_sim::OccupancyGrid map = make_test_map();
    const robot_sim::CollisionChecker checker(map, {0.20});

    expect_true(checker.is_pose_valid({1.5, 1.5, 0.0}));
    expect_false(checker.is_pose_valid({3.5, 2.5, 0.0}));
    expect_false(checker.is_pose_valid({0.1, 1.5, 0.0}));
    expect_false(checker.is_pose_valid({1.5, 0.1, 0.0}));
    expect_false(checker.is_pose_valid({std::nan(""), 1.5, 0.0}));

    expect_false(
        checker.is_motion_valid({1.5, 2.5, 0.0}, {4.5, 2.5, 0.0})
    );
    expect_true(
        checker.is_motion_valid({1.5, 1.5, 0.0}, {4.5, 1.5, 0.0})
    );
    expect_invalid_argument([&checker] {
        (void)checker.is_motion_valid(
            {1.5, 1.5, 0.0}, {2.5, 1.5, 0.0}, 0.0
        );
    });
}

void run_all_map_and_collision_tests() {
    test_map_loading_and_coordinates();
    test_malformed_maps();
    test_collision_and_motion();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase2MapAndCollision, MapLoadingAndCoordinates) {
    test_map_loading_and_coordinates();
}

TEST(Phase2MapAndCollision, MalformedMaps) {
    test_malformed_maps();
}

TEST(Phase2MapAndCollision, CollisionAndMotion) {
    test_collision_and_motion();
}

#else

void run_map_and_collision_tests() {
    run_all_map_and_collision_tests();
}

#endif
