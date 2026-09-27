#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robot_sim/experiments/experiment_runner.hpp"

#ifdef ROBOT_SIM_HAS_GTEST
#include <gtest/gtest.h>
#endif

namespace {

void expect_true(bool value) {
#ifdef ROBOT_SIM_HAS_GTEST
    EXPECT_TRUE(value);
#else
    assert(value);
#endif
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

void test_reproducible_artifact_bundle() {
    const std::string suffix = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()
    );
    const std::filesystem::path first_output =
        std::filesystem::temp_directory_path() / ("robot_sim_phase6_first_" + suffix);
    const std::filesystem::path second_output =
        std::filesystem::temp_directory_path() / ("robot_sim_phase6_second_" + suffix);
    const std::filesystem::path map_path =
        std::filesystem::path(ROBOT_SIM_SOURCE_DIR) / "maps/simple_room.map";

    const robot_sim::ExperimentConfig first_config{
        map_path,
        first_output,
        42,
        0.20,
        0.02,
        60.0,
    };
    const robot_sim::ExperimentSummary first = robot_sim::run_experiment(
        first_config
    );
    const robot_sim::ExperimentSummary second = robot_sim::run_experiment({
        map_path,
        second_output,
        42,
        0.20,
        0.02,
        60.0,
    });

    expect_true(first.plan_status == robot_sim::PlanStatus::Success);
    expect_true(first.simulation_status == robot_sim::SimulationStatus::GoalReached);
    expect_true(first.success);
    expect_true(second.success);

    const std::vector<std::string> artifact_names{
        "resolved_config.yaml",
        "summary.json",
        "trace.csv",
        "planned_path.csv",
        "lidar.csv",
        "map_overlay.svg",
    };
    for (const std::string& name : artifact_names) {
        const std::filesystem::path first_path = first_output / name;
        const std::filesystem::path second_path = second_output / name;
        expect_true(std::filesystem::exists(first_path));
        expect_true(std::filesystem::file_size(first_path) > 0);
        expect_true(read_file(first_path) == read_file(second_path));
    }

    const std::string summary = read_file(first_output / "summary.json");
    const std::string trace = read_file(first_output / "trace.csv");
    const std::string visualization = read_file(first_output / "map_overlay.svg");
    expect_true(summary.find("\"status\": \"goal_reached\"") != std::string::npos);
    expect_true(summary.find("\"artifacts\"") != std::string::npos);
    expect_true(trace.find("step,time_s,x_m,y_m") == 0);
    expect_true(visualization.find("<svg") == 0);

    std::error_code error;
    std::filesystem::remove_all(first_output, error);
    std::filesystem::remove_all(second_output, error);
}

void run_all_experiment_runner_tests() {
    test_reproducible_artifact_bundle();
}

}  // namespace

#ifdef ROBOT_SIM_HAS_GTEST

TEST(Phase6Experiments, ReproducibleArtifactBundle) {
    test_reproducible_artifact_bundle();
}

#else

void run_experiment_runner_tests() {
    run_all_experiment_runner_tests();
}

#endif
