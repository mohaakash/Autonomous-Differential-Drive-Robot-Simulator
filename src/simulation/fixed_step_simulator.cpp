#include "robot_sim/simulation/fixed_step_simulator.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "robot_sim/geometry/angle.hpp"

namespace robot_sim {
namespace {

void validate_parameters(const SimulationParameters& parameters) {
    if (!std::isfinite(parameters.dt_s) ||
        !std::isfinite(parameters.max_duration_s) ||
        parameters.dt_s <= 0.0 ||
        parameters.max_duration_s <= 0.0 ||
        parameters.max_duration_s < parameters.dt_s) {
        throw std::invalid_argument(
            "simulation time step and duration must be finite, positive, "
            "and duration must be at least one time step"
        );
    }
}

}  // namespace

std::string_view simulation_status_name(SimulationStatus status) {
    switch (status) {
        case SimulationStatus::GoalReached:
            return "goal_reached";
        case SimulationStatus::Timeout:
            return "timeout";
        case SimulationStatus::Collision:
            return "collision";
        case SimulationStatus::SafetyStop:
            return "safety_stop";
        case SimulationStatus::ReplanFailure:
            return "replan_failure";
        case SimulationStatus::ControlFailure:
            return "control_failure";
        case SimulationStatus::InvalidInput:
            return "invalid_input";
    }
    return "unknown";
}

FixedStepSimulator::FixedStepSimulator(
    const PurePursuitController& controller,
    const DifferentialDriveKinematics& kinematics,
    const CollisionChecker& collision_checker,
    SimulationParameters parameters,
    LidarSensor* lidar_sensor,
    ForwardSafetyMonitor* safety_monitor,
    std::vector<DynamicObstacleEvent> obstacle_events,
    ReplanCallback replan_callback
)
    : controller_(controller),
      kinematics_(kinematics),
      collision_checker_(collision_checker),
      parameters_(parameters),
      lidar_sensor_(lidar_sensor),
      safety_monitor_(safety_monitor),
      obstacle_events_(std::move(obstacle_events)),
      replan_callback_(std::move(replan_callback)) {
    validate_parameters(parameters_);
    if (parameters_.safety_policy != SafetyPolicy::Disabled &&
        (lidar_sensor_ == nullptr || safety_monitor_ == nullptr)) {
        throw std::invalid_argument(
            "enabled safety policy requires both LiDAR and safety monitor"
        );
    }
    for (const DynamicObstacleEvent& event : obstacle_events_) {
        if (!event.obstacle.is_valid()) {
            throw std::invalid_argument("dynamic obstacle event is invalid");
        }
    }
}

SimulationResult FixedStepSimulator::run(
    const Pose2D& initial_pose,
    const Pose2D& goal_pose,
    const PlannedPath& path
) const {
    SimulationResult result;
    result.final_pose = initial_pose;

    std::vector<AxisAlignedRectangle> active_obstacles;
    for (const DynamicObstacleEvent& event : obstacle_events_) {
        if (event.activate_step == 0) {
            active_obstacles.push_back(event.obstacle);
        }
    }

    if (!initial_pose.is_finite() || !goal_pose.is_finite() ||
        path.waypoints.empty() ||
        !collision_checker_.is_pose_valid(initial_pose, active_obstacles)) {
        result.status = SimulationStatus::InvalidInput;
        return result;
    }

    PlannedPath active_path = path;
    Pose2D pose = initial_pose;
    double time_s = 0.0;
    std::size_t step = 0;

    const auto record_sample = [
        &result,
        &goal_pose
    ](
        std::size_t sample_step,
        double sample_time,
        const Pose2D& sample_pose,
        const ControlOutput& control,
        const Twist2D& executed_twist,
        const WheelCommand& wheels,
        bool collision,
        bool safety_blocked,
        bool replanned
    ) {
        const double goal_error = std::hypot(
            goal_pose.x - sample_pose.x,
            goal_pose.y - sample_pose.y
        );
        result.final_pose = sample_pose;
        result.final_goal_position_error_m = goal_error;
        result.final_heading_error_rad = std::abs(
            shortest_angular_distance(sample_pose.theta, goal_pose.theta)
        );
        result.max_cross_track_error_m = std::max(
            result.max_cross_track_error_m,
            control.cross_track_error_m
        );
        if (wheels.clipped) {
            ++result.command_clip_count;
        }
        result.trace.push_back({
            sample_step,
            sample_time,
            sample_pose,
            control.command,
            executed_twist,
            wheels,
            goal_error,
            control.heading_error_rad,
            control.cross_track_error_m,
            control.selected_waypoint_index,
            control.command_limited || wheels.clipped,
            collision,
            safety_blocked,
            replanned,
        });
    };

    while (time_s <= parameters_.max_duration_s + 1.0e-12) {
        active_obstacles.clear();
        for (const DynamicObstacleEvent& event : obstacle_events_) {
            if (event.activate_step <= step) {
                active_obstacles.push_back(event.obstacle);
            }
        }

        bool safety_blocked = false;
        bool replanned = false;
        std::optional<ControlOutput> safety_override;
        if (parameters_.safety_policy != SafetyPolicy::Disabled) {
            const LaserScan scan = lidar_sensor_->measure(
                pose,
                time_s,
                active_obstacles
            );
            const SafetyDecision safety = safety_monitor_->evaluate(scan);
            if (safety.status == SafetyStatus::InvalidScan) {
                const ControlOutput invalid_control = controller_.update(
                    pose,
                    active_path,
                    goal_pose
                );
                record_sample(
                    step,
                    time_s,
                    pose,
                    invalid_control,
                    {},
                    {},
                    false,
                    false,
                    false
                );
                result.status = SimulationStatus::ControlFailure;
                return result;
            }

            if (safety.blocked()) {
                safety_blocked = true;
                const ControlOutput blocked_control = controller_.update(
                    pose,
                    active_path,
                    goal_pose
                );
                const ControlOutput stopped_control{
                    {},
                    blocked_control.status,
                    blocked_control.goal_position_error_m,
                    blocked_control.heading_error_rad,
                    blocked_control.cross_track_error_m,
                    blocked_control.selected_waypoint_index,
                    true,
                };
                if (parameters_.safety_policy == SafetyPolicy::Stop) {
                    record_sample(
                        step,
                        time_s,
                        pose,
                        stopped_control,
                        {},
                        {},
                        false,
                        true,
                        false
                    );
                    ++result.safety_stop_count;
                    result.status = SimulationStatus::SafetyStop;
                    return result;
                }

                if (!replan_callback_) {
                    record_sample(
                        step,
                        time_s,
                        pose,
                        stopped_control,
                        {},
                        {},
                        false,
                        true,
                        false
                    );
                    result.status = SimulationStatus::ReplanFailure;
                    return result;
                }

                const std::optional<PlannedPath> replanned_path = replan_callback_(
                    pose,
                    goal_pose,
                    active_obstacles
                );
                if (!replanned_path.has_value() ||
                    replanned_path->waypoints.empty()) {
                    record_sample(
                        step,
                        time_s,
                        pose,
                        stopped_control,
                        {},
                        {},
                        false,
                        true,
                        false
                    );
                    result.status = SimulationStatus::ReplanFailure;
                    return result;
                }

                active_path = *replanned_path;
                ++result.replan_count;
                replanned = true;
                const ControlOutput replanned_control = controller_.update(
                    pose,
                    active_path,
                    goal_pose
                );
                safety_override = replanned_control;
                safety_override->command.linear_x = 0.0;
                safety_override->command_limited = true;
            }
        }

        const ControlOutput control = safety_override.has_value()
            ? *safety_override
            : controller_.update(pose, active_path, goal_pose);
        const WheelCommand zero_wheels{};
        const Twist2D zero_twist{};

        if (control.status == ControllerStatus::InvalidInput) {
            record_sample(
                step,
                time_s,
                pose,
                control,
                zero_twist,
                zero_wheels,
                false,
                safety_blocked,
                replanned
            );
            result.status = SimulationStatus::ControlFailure;
            return result;
        }
        if (control.status == ControllerStatus::GoalReached) {
            record_sample(
                step,
                time_s,
                pose,
                control,
                zero_twist,
                zero_wheels,
                false,
                safety_blocked,
                replanned
            );
            result.status = SimulationStatus::GoalReached;
            return result;
        }

        const WheelCommand wheels = kinematics_.inverse(control.command);
        const Twist2D executed_twist = kinematics_.forward(wheels);
        const Pose2D candidate = integrate_twist(
            pose,
            executed_twist,
            parameters_.dt_s
        );
        if (!collision_checker_.is_motion_valid(
                pose,
                candidate,
                active_obstacles
            )) {
            record_sample(
                step,
                time_s,
                pose,
                control,
                executed_twist,
                wheels,
                true,
                safety_blocked,
                replanned
            );
            result.status = SimulationStatus::Collision;
            return result;
        }

        result.travelled_distance_m += (
            candidate.position() - pose.position()
        ).norm();
        pose = candidate;
        time_s += parameters_.dt_s;
        ++step;
        record_sample(
            step,
            time_s,
            pose,
            control,
            executed_twist,
            wheels,
            false,
            safety_blocked,
            replanned
        );

        if (time_s >= parameters_.max_duration_s - 1.0e-12) {
            result.status = SimulationStatus::Timeout;
            return result;
        }
    }

    result.status = SimulationStatus::Timeout;
    return result;
}

}  // namespace robot_sim
