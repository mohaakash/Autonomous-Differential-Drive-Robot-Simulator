#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_sim/build_info.hpp"
#include "robot_sim/collision/collision_checker.hpp"
#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/kinematics/differential_drive.hpp"
#include "robot_sim/map/occupancy_grid.hpp"
#include "robot_sim/planning/a_star.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

namespace {

std::size_t parse_runs(const char* value) {
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(value, &consumed);
    if (value[consumed] != '\0' || parsed == 0 ||
        parsed > std::numeric_limits<std::size_t>::max()) {
        throw std::invalid_argument("run count must be a positive integer");
    }
    return static_cast<std::size_t>(parsed);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::filesystem::path map_path = argc >= 2
            ? argv[1]
            : "maps/simple_room.map";
        const std::size_t run_count = argc >= 3 ? parse_runs(argv[2]) : 100;
        if (argc > 3) {
            throw std::invalid_argument(
                "usage: robot_sim_benchmark [MAP] [RUN_COUNT]"
            );
        }

        const robot_sim::OccupancyGrid map =
            robot_sim::OccupancyGrid::load_from_file(map_path);
        if (!map.start_marker().has_value() || !map.goal_marker().has_value()) {
            throw std::invalid_argument("benchmark map must contain S and G markers");
        }
        const robot_sim::AStarPlanner planner(map, 0.20);
        const robot_sim::PlanResult plan = planner.plan_cells(
            *map.start_marker(),
            *map.goal_marker()
        );
        if (!plan.success()) {
            throw std::runtime_error(
                "benchmark planning failed: " +
                std::string(robot_sim::plan_status_name(plan.status))
            );
        }

        const robot_sim::Vector2 start = map.cell_center(*map.start_marker());
        const robot_sim::Vector2 goal = map.cell_center(*map.goal_marker());
        const robot_sim::PurePursuitController controller({
            0.75, 0.20, 0.40, 0.50, 2.0, 2.0, 1.5, 0.10, 0.12,
        });
        const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
        const robot_sim::CollisionChecker collision_checker(map, {0.20});
        const robot_sim::FixedStepSimulator simulator(
            controller,
            kinematics,
            collision_checker,
            {0.02, 60.0}
        );

        std::vector<double> durations_ms;
        durations_ms.reserve(run_count);
        std::size_t steps = 0;
        for (std::size_t run = 0; run < run_count; ++run) {
            const auto begin = std::chrono::steady_clock::now();
            const robot_sim::SimulationResult result = simulator.run(
                {start.x, start.y, 0.0},
                {goal.x, goal.y, 0.0},
                plan.path
            );
            const auto end = std::chrono::steady_clock::now();
            if (result.status != robot_sim::SimulationStatus::GoalReached) {
                throw std::runtime_error(
                    "benchmark simulation failed: " +
                    std::string(robot_sim::simulation_status_name(result.status))
                );
            }
            steps = result.trace.empty() ? 0 : result.trace.back().step;
            durations_ms.push_back(
                std::chrono::duration<double, std::milli>(end - begin).count()
            );
        }

        const double total_ms = std::accumulate(
            durations_ms.begin(), durations_ms.end(), 0.0
        );
        const auto minimum = std::min_element(durations_ms.begin(), durations_ms.end());
        const auto maximum = std::max_element(durations_ms.begin(), durations_ms.end());
        std::cout << std::fixed << std::setprecision(3)
                  << "project: " << robot_sim::kProjectName << ' '
                  << robot_sim::kProjectVersion << '\n'
                  << "map: " << map_path.string() << '\n'
                  << "runs: " << run_count << '\n'
                  << "steps_per_run: " << steps << '\n'
                  << "mean_run_ms: " << total_ms / static_cast<double>(run_count) << '\n'
                  << "min_run_ms: " << *minimum << '\n'
                  << "max_run_ms: " << *maximum << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
