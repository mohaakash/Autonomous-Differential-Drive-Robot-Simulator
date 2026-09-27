#include "robot_sim/planning/a_star.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace robot_sim {
namespace {

struct OpenEntry {
    GridCell cell;
    double g{0.0};
    double f{0.0};
    double h{0.0};
};

struct OpenEntryCompare {
    bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const {
        if (lhs.f != rhs.f) {
            return lhs.f > rhs.f;
        }
        if (lhs.h != rhs.h) {
            return lhs.h > rhs.h;
        }
        if (lhs.cell.row != rhs.cell.row) {
            return lhs.cell.row > rhs.cell.row;
        }
        return lhs.cell.column > rhs.cell.column;
    }
};

constexpr std::array<GridCell, 8> kNeighborOffsets{{
    {-1, 0},
    {0, -1},
    {0, 1},
    {1, 0},
    {-1, -1},
    {-1, 1},
    {1, -1},
    {1, 1},
}};

[[nodiscard]] double octile_distance(GridCell from, GridCell to) {
    const double dx = static_cast<double>(
        std::abs(to.column - from.column)
    );
    const double dy = static_cast<double>(
        std::abs(to.row - from.row)
    );
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

[[nodiscard]] double step_cost(GridCell from, GridCell to) {
    const int row_delta = std::abs(to.row - from.row);
    const int column_delta = std::abs(to.column - from.column);
    return row_delta == 1 && column_delta == 1 ? std::sqrt(2.0) : 1.0;
}

}  // namespace

std::string_view plan_status_name(PlanStatus status) {
    switch (status) {
        case PlanStatus::Success:
            return "success";
        case PlanStatus::StartOutsideMap:
            return "start_outside_map";
        case PlanStatus::GoalOutsideMap:
            return "goal_outside_map";
        case PlanStatus::StartOccupied:
            return "start_occupied";
        case PlanStatus::GoalOccupied:
            return "goal_occupied";
        case PlanStatus::Unreachable:
            return "unreachable";
    }
    return "unknown";
}

AStarPlanner::AStarPlanner(
    const OccupancyGrid& map,
    double robot_radius_m
)
    : map_(map), collision_checker_(map, {robot_radius_m}) {
    inflated_occupied_.resize(
        static_cast<std::size_t>(map_.width() * map_.height()),
        0
    );

    for (int row = 0; row < map_.height(); ++row) {
        for (int column = 0; column < map_.width(); ++column) {
            const GridCell cell{row, column};
            const Vector2 centre = map_.cell_center(cell);
            const Pose2D pose{centre.x, centre.y, 0.0};
            const bool blocked = map_.is_occupied(cell) ||
                !collision_checker_.is_pose_valid(pose);
            inflated_occupied_[index(cell)] = blocked ? 1 : 0;
        }
    }
}

PlanResult AStarPlanner::plan_cells(GridCell start, GridCell goal) const {
    if (!map_.is_inside(start)) {
        return {PlanStatus::StartOutsideMap, {}};
    }
    if (!map_.is_inside(goal)) {
        return {PlanStatus::GoalOutsideMap, {}};
    }
    if (!is_planning_free(start)) {
        return {PlanStatus::StartOccupied, {}};
    }
    if (!is_planning_free(goal)) {
        return {PlanStatus::GoalOccupied, {}};
    }

    if (start == goal) {
        return {PlanStatus::Success, make_path({start})};
    }

    const std::size_t cell_count = inflated_occupied_.size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(cell_count, infinity);
    std::vector<int> came_from(cell_count, -1);
    std::vector<char> closed(cell_count, 0);
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open;

    g_score[index(start)] = 0.0;
    open.push({start, 0.0, octile_distance(start, goal), octile_distance(start, goal)});

    while (!open.empty()) {
        const OpenEntry current = open.top();
        open.pop();
        const std::size_t current_index = index(current.cell);

        if (closed[current_index] || current.g > g_score[current_index]) {
            continue;
        }
        closed[current_index] = 1;

        if (current.cell == goal) {
            std::vector<GridCell> cells;
            GridCell cell = goal;
            cells.push_back(cell);
            while (cell != start) {
                const int parent_index = came_from[index(cell)];
                if (parent_index < 0) {
                    throw std::logic_error("A* path has no parent for a reachable cell");
                }
                cell = cell_from_index(static_cast<std::size_t>(parent_index));
                cells.push_back(cell);
            }
            std::reverse(cells.begin(), cells.end());
            return {PlanStatus::Success, make_path(cells)};
        }

        for (const GridCell offset : kNeighborOffsets) {
            const GridCell neighbor{
                current.cell.row + offset.row,
                current.cell.column + offset.column,
            };
            if (!is_planning_free(neighbor) ||
                closed[index(neighbor)] ||
                !is_transition_valid(current.cell, neighbor)) {
                continue;
            }

            const double tentative_g = current.g + step_cost(current.cell, neighbor);
            const std::size_t neighbor_index = index(neighbor);
            if (tentative_g >= g_score[neighbor_index]) {
                continue;
            }

            came_from[neighbor_index] = static_cast<int>(current_index);
            g_score[neighbor_index] = tentative_g;
            const double h = octile_distance(neighbor, goal);
            open.push({neighbor, tentative_g, tentative_g + h, h});
        }
    }

    return {PlanStatus::Unreachable, {}};
}

PlanResult AStarPlanner::plan_world(
    Vector2 start_world,
    Vector2 goal_world
) const {
    const std::optional<GridCell> start = map_.world_to_cell(start_world);
    if (!start.has_value()) {
        return {PlanStatus::StartOutsideMap, {}};
    }
    const std::optional<GridCell> goal = map_.world_to_cell(goal_world);
    if (!goal.has_value()) {
        return {PlanStatus::GoalOutsideMap, {}};
    }
    return plan_cells(*start, *goal);
}

bool AStarPlanner::is_planning_free(GridCell cell) const {
    return map_.is_inside(cell) && inflated_occupied_[index(cell)] == 0;
}

std::size_t AStarPlanner::index(GridCell cell) const {
    return static_cast<std::size_t>(cell.row * map_.width() + cell.column);
}

GridCell AStarPlanner::cell_from_index(std::size_t cell_index) const {
    const int width = map_.width();
    return {
        static_cast<int>(cell_index / static_cast<std::size_t>(width)),
        static_cast<int>(cell_index % static_cast<std::size_t>(width)),
    };
}

bool AStarPlanner::is_transition_valid(GridCell from, GridCell to) const {
    if (!map_.is_inside(from) || !map_.is_inside(to) ||
        !is_planning_free(from) || !is_planning_free(to)) {
        return false;
    }

    const Vector2 from_point = map_.cell_center(from);
    const Vector2 to_point = map_.cell_center(to);
    return collision_checker_.is_motion_valid(
        {from_point.x, from_point.y, 0.0},
        {to_point.x, to_point.y, 0.0},
        map_.resolution_m() / 4.0
    );
}

PlannedPath AStarPlanner::make_path(const std::vector<GridCell>& cells) const {
    PlannedPath path;
    path.cells = cells;
    path.waypoints.reserve(cells.size());
    for (const GridCell cell : cells) {
        path.waypoints.push_back(map_.cell_center(cell));
    }

    for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
        const Vector2 delta = path.waypoints[i] - path.waypoints[i - 1];
        path.length_m += delta.norm();
    }
    return path;
}

}  // namespace robot_sim
