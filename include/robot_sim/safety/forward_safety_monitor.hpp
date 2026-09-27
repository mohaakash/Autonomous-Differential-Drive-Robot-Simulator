#pragma once

#include <cstddef>
#include <string_view>

#include "robot_sim/sensors/lidar.hpp"

namespace robot_sim {

struct ForwardSafetyParameters {
    double sector_half_angle_rad{0.45};
    double minimum_forward_range_m{0.35};
};

enum class SafetyStatus {
    Safe,
    ObstacleAhead,
    InvalidScan,
};

[[nodiscard]] std::string_view safety_status_name(SafetyStatus status);

struct SafetyDecision {
    SafetyStatus status{SafetyStatus::InvalidScan};
    double minimum_forward_range_m{0.0};
    std::size_t blocking_ray_index{0};

    [[nodiscard]] bool blocked() const {
        return status == SafetyStatus::ObstacleAhead;
    }
};

class ForwardSafetyMonitor {
public:
    explicit ForwardSafetyMonitor(ForwardSafetyParameters parameters);

    [[nodiscard]] const ForwardSafetyParameters& parameters() const {
        return parameters_;
    }

    [[nodiscard]] SafetyDecision evaluate(const LaserScan& scan) const;

private:
    ForwardSafetyParameters parameters_;
};

}  // namespace robot_sim
