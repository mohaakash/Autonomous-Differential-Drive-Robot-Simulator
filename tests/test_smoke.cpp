#include <cassert>
#include <string_view>

#include "robot_sim/build_info.hpp"

#ifdef ROBOT_SIM_HAS_EIGEN
#include <Eigen/Core>
#endif

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>

TEST(Phase0Smoke, BuildInfoIsAvailable) {
    EXPECT_FALSE(robot_sim::kProjectName.empty());
    EXPECT_EQ(robot_sim::kProjectVersion, std::string_view{"0.1.0"});
}

#ifdef ROBOT_SIM_HAS_EIGEN
TEST(Phase0Smoke, EigenIsUsable) {
    const Eigen::Vector2d point(3.0, 4.0);
    EXPECT_DOUBLE_EQ(point.norm(), 5.0);
}
#endif

#else

void run_smoke_tests() {
    assert(!robot_sim::kProjectName.empty());
    assert(robot_sim::kProjectVersion == std::string_view{"0.1.0"});
}

#endif
