#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "robot_sim/geometry/axis_aligned_rectangle.hpp"
#include "robot_sim/geometry/pose2d.hpp"
#include "robot_sim/map/occupancy_grid.hpp"

namespace robot_sim {

struct LidarParameters {
    Vector2 mount_translation{0.0, 0.0};
    double mount_yaw_rad{0.0};
    double angle_min_rad{-1.5707963267948966};
    double angle_max_rad{1.5707963267948966};
    std::size_t ray_count{181};
    double range_min_m{0.02};
    double range_max_m{8.0};
    double noise_stddev_m{0.0};
    std::uint32_t seed{42};
};

struct LaserScan {
    double stamp_s{0.0};
    double angle_min_rad{0.0};
    double angle_increment_rad{0.0};
    double range_min_m{0.0};
    double range_max_m{0.0};
    std::vector<double> ranges_m;
    std::vector<char> has_return;

    [[nodiscard]] double angle_at(std::size_t index) const;
};

class LidarSensor {
public:
    LidarSensor(const OccupancyGrid& map, LidarParameters parameters);

    [[nodiscard]] const LidarParameters& parameters() const {
        return parameters_;
    }

    [[nodiscard]] LaserScan measure(
        const Pose2D& pose,
        double stamp_s = 0.0,
        const std::vector<AxisAlignedRectangle>& extra_obstacles = {}
    );

    void reseed(std::uint32_t seed);

private:
    [[nodiscard]] double nearest_hit_distance(
        Vector2 origin,
        Vector2 direction,
        const std::vector<AxisAlignedRectangle>& extra_obstacles
    ) const;

    const OccupancyGrid& map_;
    LidarParameters parameters_;
    std::mt19937 random_engine_;
};

}  // namespace robot_sim
