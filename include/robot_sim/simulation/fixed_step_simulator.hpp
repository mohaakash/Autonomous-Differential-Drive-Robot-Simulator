#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "robot_sim/collision/collision_checker.hpp"
#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/geometry/axis_aligned_rectangle.hpp"
#include "robot_sim/kinematics/differential_drive.hpp"
#include "robot_sim/safety/forward_safety_monitor.hpp"
#include "robot_sim/sensors/lidar.hpp"

namespace robot_sim {

enum class SafetyPolicy {
    Disabled,
    Stop,
    Replan,
};

struct SimulationParameters {
    double dt_s{0.02};
    double max_duration_s{60.0};
    SafetyPolicy safety_policy{SafetyPolicy::Disabled};
};

enum class SimulationStatus {
    GoalReached,
    Timeout,
    Collision,
    SafetyStop,
    ReplanFailure,
    ControlFailure,
    InvalidInput,
};

[[nodiscard]] std::string_view simulation_status_name(SimulationStatus status);

struct TraceSample {
    std::size_t step{0};
    double time_s{0.0};
    Pose2D pose{};
    Twist2D requested_twist{};
    Twist2D executed_twist{};
    WheelCommand wheel_command{};
    double goal_position_error_m{0.0};
    double heading_error_rad{0.0};
    double cross_track_error_m{0.0};
    std::size_t selected_waypoint_index{0};
    bool command_limited{false};
    bool collision{false};
    bool safety_blocked{false};
    bool replanned{false};
};

struct SimulationResult {
    SimulationStatus status{SimulationStatus::InvalidInput};
    Pose2D final_pose{};
    double travelled_distance_m{0.0};
    double max_cross_track_error_m{0.0};
    double final_goal_position_error_m{0.0};
    double final_heading_error_rad{0.0};
    std::size_t command_clip_count{0};
    std::size_t replan_count{0};
    std::size_t safety_stop_count{0};
    std::vector<TraceSample> trace;
};

struct DynamicObstacleEvent {
    std::size_t activate_step{0};
    AxisAlignedRectangle obstacle{};
};

using ReplanCallback = std::function<std::optional<PlannedPath>(
    const Pose2D& current_pose,
    const Pose2D& goal_pose,
    const std::vector<AxisAlignedRectangle>& active_obstacles
)>;

class FixedStepSimulator {
public:
    FixedStepSimulator(
        const PurePursuitController& controller,
        const DifferentialDriveKinematics& kinematics,
        const CollisionChecker& collision_checker,
        SimulationParameters parameters,
        LidarSensor* lidar_sensor = nullptr,
        ForwardSafetyMonitor* safety_monitor = nullptr,
        std::vector<DynamicObstacleEvent> obstacle_events = {},
        ReplanCallback replan_callback = {}
    );

    [[nodiscard]] SimulationResult run(
        const Pose2D& initial_pose,
        const Pose2D& goal_pose,
        const PlannedPath& path
    ) const;

    [[nodiscard]] const SimulationParameters& parameters() const {
        return parameters_;
    }

private:
    const PurePursuitController& controller_;
    const DifferentialDriveKinematics& kinematics_;
    const CollisionChecker& collision_checker_;
    SimulationParameters parameters_;
    LidarSensor* lidar_sensor_;
    ForwardSafetyMonitor* safety_monitor_;
    std::vector<DynamicObstacleEvent> obstacle_events_;
    ReplanCallback replan_callback_;
};

}  // namespace robot_sim
