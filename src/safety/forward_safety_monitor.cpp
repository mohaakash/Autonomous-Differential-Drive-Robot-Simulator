#include "robot_sim/safety/forward_safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_sim {

std::string_view safety_status_name(SafetyStatus status) {
    switch (status) {
        case SafetyStatus::Safe:
            return "safe";
        case SafetyStatus::ObstacleAhead:
            return "obstacle_ahead";
        case SafetyStatus::InvalidScan:
            return "invalid_scan";
    }
    return "unknown";
}

ForwardSafetyMonitor::ForwardSafetyMonitor(
    ForwardSafetyParameters parameters
)
    : parameters_(parameters) {
    if (!std::isfinite(parameters_.sector_half_angle_rad) ||
        !std::isfinite(parameters_.minimum_forward_range_m) ||
        parameters_.sector_half_angle_rad < 0.0 ||
        parameters_.minimum_forward_range_m <= 0.0) {
        throw std::invalid_argument("forward safety parameters are invalid");
    }
}

SafetyDecision ForwardSafetyMonitor::evaluate(const LaserScan& scan) const {
    if (scan.ranges_m.size() != scan.has_return.size() ||
        scan.ranges_m.empty() ||
        !std::isfinite(scan.angle_min_rad) ||
        !std::isfinite(scan.angle_increment_rad) ||
        !std::isfinite(scan.range_min_m) ||
        !std::isfinite(scan.range_max_m) ||
        scan.range_min_m < 0.0 ||
        scan.range_max_m < scan.range_min_m) {
        return {SafetyStatus::InvalidScan, 0.0, 0};
    }

    SafetyDecision decision{
        SafetyStatus::Safe,
        scan.range_max_m,
        0,
    };
    for (std::size_t index = 0; index < scan.ranges_m.size(); ++index) {
        const double angle = scan.angle_at(index);
        const double range = scan.ranges_m[index];
        if (!std::isfinite(range) ||
            range < scan.range_min_m || range > scan.range_max_m) {
            return {SafetyStatus::InvalidScan, 0.0, index};
        }
        if (std::abs(angle) > parameters_.sector_half_angle_rad ||
            scan.has_return[index] == 0) {
            continue;
        }
        if (range < decision.minimum_forward_range_m) {
            decision.minimum_forward_range_m = range;
            decision.blocking_ray_index = index;
        }
    }

    if (decision.minimum_forward_range_m < parameters_.minimum_forward_range_m) {
        decision.status = SafetyStatus::ObstacleAhead;
    }
    return decision;
}

}  // namespace robot_sim
