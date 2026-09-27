cmake_minimum_required(VERSION 3.21)

project(differential_drive_simulator
    VERSION 0.1.0
    DESCRIPTION "A 2D autonomous differential-drive robot simulator"
    LANGUAGES CXX
)

option(ROBOT_SIM_BUILD_TESTS "Build the simulator unit tests" ON)
option(
    ROBOT_SIM_BUILD_VIEWER
    "Build the optional interactive raylib viewer when available"
    ON
)
option(
    ROBOT_SIM_FETCH_RAYLIB
    "Fetch raylib for the optional interactive viewer"
    OFF
)
option(
    ROBOT_SIM_WARNINGS_AS_ERRORS
    "Treat compiler warnings as errors"
    OFF
)
option(
    ROBOT_SIM_FETCH_DEPENDENCIES
    "Fetch Eigen and GoogleTest when they are not installed"
    OFF
)

set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_sim_warnings INTERFACE)
if(MSVC)
    target_compile_options(robot_sim_warnings INTERFACE /W4)
    if(ROBOT_SIM_WARNINGS_AS_ERRORS)
        target_compile_options(robot_sim_warnings INTERFACE /WX)
    endif()
else()
    target_compile_options(robot_sim_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
    )
    if(ROBOT_SIM_WARNINGS_AS_ERRORS)
        target_compile_options(robot_sim_warnings INTERFACE -Werror)
    endif()
endif()

add_library(robot_sim_core STATIC
    src/collision/collision_checker.cpp
    src/control/pure_pursuit.cpp
    src/experiments/experiment_runner.cpp
    src/kinematics/differential_drive.cpp
    src/map/occupancy_grid.cpp
    src/planning/a_star.cpp
    src/safety/forward_safety_monitor.cpp
    src/sensors/lidar.cpp
    src/simulation/fixed_step_simulator.cpp
)
target_compile_features(robot_sim_core PUBLIC cxx_std_17)
target_include_directories(robot_sim_core PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
)
target_link_libraries(robot_sim_core PUBLIC robot_sim_warnings)

include(cmake/Dependencies.cmake)

add_executable(robot_sim
    src/main.cpp
)
target_link_libraries(robot_sim PRIVATE
    robot_sim_core
)

add_executable(robot_sim_benchmark
    src/benchmark_main.cpp
)
target_link_libraries(robot_sim_benchmark PRIVATE
    robot_sim_core
)

if(ROBOT_SIM_BUILD_VIEWER AND ROBOT_SIM_RAYLIB_AVAILABLE)
    add_executable(robot_sim_viewer
        src/viewer_main.cpp
    )
    target_link_libraries(robot_sim_viewer PRIVATE
        robot_sim_core
        ${ROBOT_SIM_RAYLIB_TARGET}
    )
elseif(ROBOT_SIM_BUILD_VIEWER)
    message(STATUS "raylib not found; robot_sim_viewer is disabled")
endif()

if(ROBOT_SIM_BUILD_TESTS)
    enable_testing()

    add_executable(robot_sim_tests
        tests/test_main.cpp
        tests/test_map_and_collision.cpp
        tests/test_planning.cpp
        tests/test_experiment_runner.cpp
        tests/test_hardening.cpp
        tests/test_sensor_and_safety.cpp
        tests/test_simulation.cpp
        tests/test_smoke.cpp
        tests/test_kinematics.cpp
    )
    target_link_libraries(robot_sim_tests PRIVATE
        robot_sim_core
    )
    target_compile_definitions(robot_sim_tests PRIVATE
        ROBOT_SIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    )

    if(ROBOT_SIM_EIGEN_AVAILABLE)
        target_compile_definitions(robot_sim_tests PRIVATE ROBOT_SIM_HAS_EIGEN=1)
        target_link_libraries(robot_sim_tests PRIVATE Eigen3::Eigen)
    endif()

    if(ROBOT_SIM_GTEST_AVAILABLE)
        target_compile_definitions(robot_sim_tests PRIVATE ROBOT_SIM_HAS_GTEST=1)
        target_link_libraries(robot_sim_tests PRIVATE GTest::gtest_main)
    endif()

    add_test(NAME robot_sim_unit COMMAND robot_sim_tests)
endif()

message(STATUS "${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "C++ standard: C++17")
message(STATUS "Build tests: ${ROBOT_SIM_BUILD_TESTS}")
message(STATUS "Build viewer: ${ROBOT_SIM_BUILD_VIEWER}")
message(STATUS "raylib available: ${ROBOT_SIM_RAYLIB_AVAILABLE}")
message(STATUS "Eigen available: ${ROBOT_SIM_EIGEN_AVAILABLE}")
message(STATUS "GoogleTest available: ${ROBOT_SIM_GTEST_AVAILABLE}")
