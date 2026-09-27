#pragma once

#include <cstdint>
#include <filesystem>

#include "robot_sim/planning/a_star.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

namespace robot_sim {

struct ExperimentConfig {
    std::filesystem::path map_path{"maps/simple_room.map"};
    std::filesystem::path output_directory{"results/simple_room_goal"};
    std::uint32_t random_seed{42};
    double robot_radius_m{0.20};
    double simulation_dt_s{0.02};
    double maximum_duration_s{60.0};
};

struct ExperimentSummary {
    PlanStatus plan_status{PlanStatus::Unreachable};
    SimulationStatus simulation_status{SimulationStatus::InvalidInput};
    bool success{false};
    std::filesystem::path output_directory;
};

// Runs a marker-based map scenario and writes a reproducible artifact bundle.
// The output directory is created if necessary and existing artifact files are
// replaced. Planning failures still produce a summary and map visualization.
[[nodiscard]] ExperimentSummary run_experiment(const ExperimentConfig& config);

}  // namespace robot_sim
