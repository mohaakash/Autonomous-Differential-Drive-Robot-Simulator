#pragma once

#include <string_view>
#include <vector>

#include "robot_sim/collision/collision_checker.hpp"

namespace robot_sim {

enum class PlanStatus {
    Success,
    StartOutsideMap,
    GoalOutsideMap,
    StartOccupied,
    GoalOccupied,
    Unreachable,
};

[[nodiscard]] std::string_view plan_status_name(PlanStatus status);

struct PlannedPath {
    std::vector<GridCell> cells;
    std::vector<Vector2> waypoints;
    double length_m{0.0};
};

struct PlanResult {
    PlanStatus status{PlanStatus::Unreachable};
    PlannedPath path;

    [[nodiscard]] bool success() const {
        return status == PlanStatus::Success;
    }
};

class AStarPlanner {
public:
    AStarPlanner(const OccupancyGrid& map, double robot_radius_m);

    [[nodiscard]] PlanResult plan_cells(
        GridCell start,
        GridCell goal
    ) const;

    [[nodiscard]] PlanResult plan_world(
        Vector2 start_world,
        Vector2 goal_world
    ) const;

    // This is the derived occupancy view after footprint inflation.
    [[nodiscard]] bool is_planning_free(GridCell cell) const;

    [[nodiscard]] double robot_radius_m() const {
        return collision_checker_.footprint_radius_m();
    }

private:
    [[nodiscard]] std::size_t index(GridCell cell) const;
    [[nodiscard]] GridCell cell_from_index(std::size_t index) const;
    [[nodiscard]] bool is_transition_valid(GridCell from, GridCell to) const;
    [[nodiscard]] PlannedPath make_path(
        const std::vector<GridCell>& cells
    ) const;

    const OccupancyGrid& map_;
    CollisionChecker collision_checker_;
    std::vector<char> inflated_occupied_;
};

}  // namespace robot_sim
