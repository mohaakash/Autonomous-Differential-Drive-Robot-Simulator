#pragma once

#include <cstddef>
#include <string_view>

#include "robot_sim/geometry/pose2d.hpp"
#include "robot_sim/geometry/twist2d.hpp"
#include "robot_sim/planning/a_star.hpp"

namespace robot_sim {

struct PurePursuitParameters {
    double lookahead_m{0.50};
    double minimum_lookahead_m{0.15};
    double nominal_linear_m_s{0.45};
    double max_linear_m_s{0.60};
    double max_angular_rad_s{2.50};
    double heading_gain{2.0};
    double goal_slowdown_gain{1.5};
    double goal_position_tolerance_m{0.10};
    double goal_heading_tolerance_rad{0.12};
};

enum class ControllerStatus {
    Tracking,
    GoalReached,
    InvalidInput,
};

[[nodiscard]] std::string_view controller_status_name(ControllerStatus status);

struct ControlOutput {
    Twist2D command{};
    ControllerStatus status{ControllerStatus::InvalidInput};
    double goal_position_error_m{0.0};
    double heading_error_rad{0.0};
    double cross_track_error_m{0.0};
    std::size_t selected_waypoint_index{0};
    bool command_limited{false};
};

class PurePursuitController {
public:
    explicit PurePursuitController(PurePursuitParameters parameters);

    [[nodiscard]] const PurePursuitParameters& parameters() const {
        return parameters_;
    }

    [[nodiscard]] ControlOutput update(
        const Pose2D& pose,
        const PlannedPath& path,
        const Pose2D& goal
    ) const;

private:
    PurePursuitParameters parameters_;
};

}  // namespace robot_sim
