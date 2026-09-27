#include "robot_sim/control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "robot_sim/geometry/angle.hpp"

namespace robot_sim {
namespace {

constexpr double kCurvatureEpsilon = 1.0e-12;

void validate_parameters(const PurePursuitParameters& parameters) {
    const double values[] = {
        parameters.lookahead_m,
        parameters.minimum_lookahead_m,
        parameters.nominal_linear_m_s,
        parameters.max_linear_m_s,
        parameters.max_angular_rad_s,
        parameters.heading_gain,
        parameters.goal_slowdown_gain,
        parameters.goal_position_tolerance_m,
        parameters.goal_heading_tolerance_rad,
    };
    for (const double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("pure-pursuit parameters must be finite");
        }
    }
    if (parameters.lookahead_m <= 0.0 ||
        parameters.minimum_lookahead_m <= 0.0 ||
        parameters.nominal_linear_m_s <= 0.0 ||
        parameters.max_linear_m_s <= 0.0 ||
        parameters.max_angular_rad_s <= 0.0 ||
        parameters.heading_gain <= 0.0 ||
        parameters.goal_slowdown_gain <= 0.0 ||
        parameters.goal_position_tolerance_m <= 0.0 ||
        parameters.goal_heading_tolerance_rad <= 0.0) {
        throw std::invalid_argument(
            "pure-pursuit parameters must be greater than zero"
        );
    }
    if (parameters.minimum_lookahead_m > parameters.lookahead_m) {
        throw std::invalid_argument(
            "minimum lookahead cannot exceed lookahead distance"
        );
    }
    if (parameters.nominal_linear_m_s > parameters.max_linear_m_s) {
        throw std::invalid_argument(
            "nominal speed cannot exceed maximum linear speed"
        );
    }
}

[[nodiscard]] double point_segment_distance(
    Vector2 point,
    Vector2 start,
    Vector2 end
) {
    const Vector2 segment = end - start;
    const double length_squared = segment.squared_norm();
    if (length_squared <= kCurvatureEpsilon) {
        return (point - start).norm();
    }

    const double projection = std::clamp(
        dot(point - start, segment) / length_squared,
        0.0,
        1.0
    );
    return (point - (start + projection * segment)).norm();
}

[[nodiscard]] double cross_track_error(
    Vector2 point,
    const PlannedPath& path
) {
    if (path.waypoints.size() == 1) {
        return (point - path.waypoints.front()).norm();
    }

    double minimum_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < path.waypoints.size(); ++index) {
        minimum_distance = std::min(
            minimum_distance,
            point_segment_distance(
                point,
                path.waypoints[index - 1],
                path.waypoints[index]
            )
        );
    }
    return minimum_distance;
}

[[nodiscard]] std::size_t closest_waypoint(
    Vector2 point,
    const PlannedPath& path
) {
    std::size_t closest = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.waypoints.size(); ++index) {
        const double distance = (point - path.waypoints[index]).squared_norm();
        if (distance < closest_distance) {
            closest_distance = distance;
            closest = index;
        }
    }
    return closest;
}

[[nodiscard]] std::size_t select_lookahead_waypoint(
    Vector2 point,
    const PlannedPath& path,
    double lookahead_m
) {
    const std::size_t closest = closest_waypoint(point, path);
    double accumulated_distance = 0.0;
    for (std::size_t index = closest + 1; index < path.waypoints.size(); ++index) {
        accumulated_distance += (
            path.waypoints[index] - path.waypoints[index - 1]
        ).norm();
        if (accumulated_distance >= lookahead_m) {
            return index;
        }
    }
    return path.waypoints.size() - 1;
}

[[nodiscard]] double clamp_angular_command(
    double angular,
    double maximum,
    bool& limited
) {
    const double clamped = std::clamp(angular, -maximum, maximum);
    limited = limited || clamped != angular;
    return clamped;
}

}  // namespace

std::string_view controller_status_name(ControllerStatus status) {
    switch (status) {
        case ControllerStatus::Tracking:
            return "tracking";
        case ControllerStatus::GoalReached:
            return "goal_reached";
        case ControllerStatus::InvalidInput:
            return "invalid_input";
    }
    return "unknown";
}

PurePursuitController::PurePursuitController(
    PurePursuitParameters parameters
)
    : parameters_(parameters) {
    validate_parameters(parameters_);
}

ControlOutput PurePursuitController::update(
    const Pose2D& pose,
    const PlannedPath& path,
    const Pose2D& goal
) const {
    ControlOutput output;
    output.goal_position_error_m = std::hypot(goal.x - pose.x, goal.y - pose.y);
    output.heading_error_rad = shortest_angular_distance(pose.theta, goal.theta);

    if (!pose.is_finite() || !goal.is_finite() || path.waypoints.empty()) {
        output.status = ControllerStatus::InvalidInput;
        return output;
    }
    for (const Vector2 waypoint : path.waypoints) {
        if (!waypoint.is_finite()) {
            output.status = ControllerStatus::InvalidInput;
            return output;
        }
    }

    output.cross_track_error_m = cross_track_error(pose.position(), path);
    output.selected_waypoint_index = select_lookahead_waypoint(
        pose.position(),
        path,
        parameters_.lookahead_m
    );

    if (output.goal_position_error_m <= parameters_.goal_position_tolerance_m) {
        if (std::abs(output.heading_error_rad) <= parameters_.goal_heading_tolerance_rad) {
            output.status = ControllerStatus::GoalReached;
            return output;
        }

        output.status = ControllerStatus::Tracking;
        output.command.linear_x = 0.0;
        output.command.angular_z = clamp_angular_command(
            parameters_.heading_gain * output.heading_error_rad,
            parameters_.max_angular_rad_s,
            output.command_limited
        );
        return output;
    }

    const Vector2 target = path.waypoints[output.selected_waypoint_index];
    const double cosine = std::cos(pose.theta);
    const double sine = std::sin(pose.theta);
    const double world_delta_x = target.x - pose.x;
    const double world_delta_y = target.y - pose.y;
    const double target_x_robot = cosine * world_delta_x + sine * world_delta_y;
    const double target_y_robot = -sine * world_delta_x + cosine * world_delta_y;
    const double target_distance = std::hypot(target_x_robot, target_y_robot);
    const double lookahead_distance = std::max(
        target_distance,
        parameters_.minimum_lookahead_m
    );

    output.status = ControllerStatus::Tracking;
    if (target_x_robot < 0.0) {
        output.command.linear_x = 0.0;
        output.command.angular_z = clamp_angular_command(
            parameters_.heading_gain * std::atan2(target_y_robot, target_x_robot),
            parameters_.max_angular_rad_s,
            output.command_limited
        );
        return output;
    }

    const double curvature = 2.0 * target_y_robot /
        (lookahead_distance * lookahead_distance);
    double linear = std::min(
        parameters_.nominal_linear_m_s,
        parameters_.max_linear_m_s
    );

    const double goal_speed_limit =
        output.goal_position_error_m * parameters_.goal_slowdown_gain;
    if (goal_speed_limit < linear) {
        linear = goal_speed_limit;
        output.command_limited = true;
    }
    if (std::abs(curvature) > kCurvatureEpsilon) {
        const double curvature_speed_limit =
            parameters_.max_angular_rad_s / std::abs(curvature);
        if (curvature_speed_limit < linear) {
            linear = curvature_speed_limit;
            output.command_limited = true;
        }
    }

    output.command.linear_x = std::clamp(linear, 0.0, parameters_.max_linear_m_s);
    output.command.angular_z = clamp_angular_command(
        output.command.linear_x * curvature,
        parameters_.max_angular_rad_s,
        output.command_limited
    );
    return output;
}

}  // namespace robot_sim
