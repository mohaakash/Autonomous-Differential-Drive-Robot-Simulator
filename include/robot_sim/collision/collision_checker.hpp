#pragma once

#include <vector>

#include "robot_sim/geometry/axis_aligned_rectangle.hpp"
#include "robot_sim/geometry/pose2d.hpp"
#include "robot_sim/map/occupancy_grid.hpp"

namespace robot_sim {

struct CircularFootprint {
    double radius_m{0.20};
};

class CollisionChecker {
public:
    CollisionChecker(const OccupancyGrid& map, CircularFootprint footprint);

    [[nodiscard]] double footprint_radius_m() const {
        return footprint_.radius_m;
    }

    [[nodiscard]] bool is_pose_valid(const Pose2D& pose) const;

    [[nodiscard]] bool is_pose_valid(
        const Pose2D& pose,
        const std::vector<AxisAlignedRectangle>& extra_obstacles
    ) const;

    [[nodiscard]] bool is_motion_valid(
        const Pose2D& from,
        const Pose2D& to
    ) const;

    [[nodiscard]] bool is_motion_valid(
        const Pose2D& from,
        const Pose2D& to,
        double max_sample_spacing_m
    ) const;

    [[nodiscard]] bool is_motion_valid(
        const Pose2D& from,
        const Pose2D& to,
        const std::vector<AxisAlignedRectangle>& extra_obstacles
    ) const;

    [[nodiscard]] bool is_motion_valid(
        const Pose2D& from,
        const Pose2D& to,
        const std::vector<AxisAlignedRectangle>& extra_obstacles,
        double max_sample_spacing_m
    ) const;

private:
    [[nodiscard]] bool intersects_occupied_cell(
        const Pose2D& pose,
        GridCell cell
    ) const;

    [[nodiscard]] bool intersects_rectangle(
        const Pose2D& pose,
        const AxisAlignedRectangle& rectangle
    ) const;

    const OccupancyGrid& map_;
    CircularFootprint footprint_;
};

}  // namespace robot_sim
