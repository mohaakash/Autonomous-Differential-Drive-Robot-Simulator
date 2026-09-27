#include "robot_sim/collision/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "robot_sim/geometry/angle.hpp"

namespace robot_sim {

CollisionChecker::CollisionChecker(
    const OccupancyGrid& map,
    CircularFootprint footprint
)
    : map_(map), footprint_(footprint) {
    if (!std::isfinite(footprint_.radius_m) || footprint_.radius_m <= 0.0) {
        throw std::invalid_argument(
            "circular footprint radius must be finite and greater than zero"
        );
    }
}

bool CollisionChecker::is_pose_valid(const Pose2D& pose) const {
    return is_pose_valid(pose, {});
}

bool CollisionChecker::is_pose_valid(
    const Pose2D& pose,
    const std::vector<AxisAlignedRectangle>& extra_obstacles
) const {
    if (!pose.is_finite()) {
        return false;
    }

    for (const AxisAlignedRectangle& obstacle : extra_obstacles) {
        if (!obstacle.is_valid()) {
            throw std::invalid_argument("dynamic obstacle rectangle is invalid");
        }
    }

    const MapBounds map_bounds = map_.bounds();
    if (pose.x - footprint_.radius_m < map_bounds.min_x ||
        pose.x + footprint_.radius_m > map_bounds.max_x ||
        pose.y - footprint_.radius_m < map_bounds.min_y ||
        pose.y + footprint_.radius_m > map_bounds.max_y) {
        return false;
    }

    for (int row = 0; row < map_.height(); ++row) {
        for (int column = 0; column < map_.width(); ++column) {
            const GridCell cell{row, column};
            if (map_.is_occupied(cell) && intersects_occupied_cell(pose, cell)) {
                return false;
            }
        }
    }

    for (const AxisAlignedRectangle& obstacle : extra_obstacles) {
        if (intersects_rectangle(pose, obstacle)) {
            return false;
        }
    }
    return true;
}

bool CollisionChecker::is_motion_valid(
    const Pose2D& from,
    const Pose2D& to
) const {
    return is_motion_valid(from, to, map_.resolution_m() / 2.0);
}

bool CollisionChecker::is_motion_valid(
    const Pose2D& from,
    const Pose2D& to,
    double max_sample_spacing_m
) const {
    return is_motion_valid(from, to, {}, max_sample_spacing_m);
}

bool CollisionChecker::is_motion_valid(
    const Pose2D& from,
    const Pose2D& to,
    const std::vector<AxisAlignedRectangle>& extra_obstacles
) const {
    return is_motion_valid(
        from,
        to,
        extra_obstacles,
        map_.resolution_m() / 2.0
    );
}

bool CollisionChecker::is_motion_valid(
    const Pose2D& from,
    const Pose2D& to,
    const std::vector<AxisAlignedRectangle>& extra_obstacles,
    double max_sample_spacing_m
) const {
    if (!std::isfinite(max_sample_spacing_m) || max_sample_spacing_m <= 0.0) {
        throw std::invalid_argument(
            "maximum sample spacing must be finite and greater than zero"
        );
    }
    if (!from.is_finite() || !to.is_finite()) {
        return false;
    }

    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const int sample_count = std::max(
        1,
        static_cast<int>(std::ceil(distance / max_sample_spacing_m))
    );
    const double heading_delta = shortest_angular_distance(from.theta, to.theta);

    for (int sample = 0; sample <= sample_count; ++sample) {
        const double fraction = static_cast<double>(sample) /
            static_cast<double>(sample_count);
        const Pose2D pose{
            from.x + fraction * (to.x - from.x),
            from.y + fraction * (to.y - from.y),
            wrap_to_pi(from.theta + fraction * heading_delta),
        };
        if (!is_pose_valid(pose, extra_obstacles)) {
            return false;
        }
    }
    return true;
}

bool CollisionChecker::intersects_occupied_cell(
    const Pose2D& pose,
    GridCell cell
) const {
    const Vector2 centre = map_.cell_center(cell);
    const double half_extent = map_.resolution_m() / 2.0;
    const double min_x = centre.x - half_extent;
    const double max_x = centre.x + half_extent;
    const double min_y = centre.y - half_extent;
    const double max_y = centre.y + half_extent;

    const double closest_x = std::clamp(pose.x, min_x, max_x);
    const double closest_y = std::clamp(pose.y, min_y, max_y);
    const double dx = pose.x - closest_x;
    const double dy = pose.y - closest_y;
    return dx * dx + dy * dy <
        footprint_.radius_m * footprint_.radius_m;
}

bool CollisionChecker::intersects_rectangle(
    const Pose2D& pose,
    const AxisAlignedRectangle& rectangle
) const {
    const double closest_x = std::clamp(pose.x, rectangle.min_x, rectangle.max_x);
    const double closest_y = std::clamp(pose.y, rectangle.min_y, rectangle.max_y);
    const double dx = pose.x - closest_x;
    const double dy = pose.y - closest_y;
    return dx * dx + dy * dy <
        footprint_.radius_m * footprint_.radius_m;
}

}  // namespace robot_sim
