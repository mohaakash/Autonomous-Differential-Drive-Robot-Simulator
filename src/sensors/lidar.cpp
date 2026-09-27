#include "robot_sim/sensors/lidar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "robot_sim/geometry/transform2d.hpp"

namespace robot_sim {
namespace {

constexpr double kRayParallelEpsilon = 1.0e-12;

void validate_parameters(const LidarParameters& parameters) {
    if (!parameters.mount_translation.is_finite() ||
        !std::isfinite(parameters.mount_yaw_rad) ||
        !std::isfinite(parameters.angle_min_rad) ||
        !std::isfinite(parameters.angle_max_rad) ||
        !std::isfinite(parameters.range_min_m) ||
        !std::isfinite(parameters.range_max_m) ||
        !std::isfinite(parameters.noise_stddev_m)) {
        throw std::invalid_argument("LiDAR parameters must be finite");
    }
    if (parameters.ray_count == 0) {
        throw std::invalid_argument("LiDAR ray count must be greater than zero");
    }
    if (parameters.angle_max_rad < parameters.angle_min_rad) {
        throw std::invalid_argument("LiDAR maximum angle must not be below minimum angle");
    }
    if (parameters.range_min_m < 0.0 ||
        parameters.range_max_m <= parameters.range_min_m) {
        throw std::invalid_argument("LiDAR range limits are invalid");
    }
    if (parameters.noise_stddev_m < 0.0) {
        throw std::invalid_argument("LiDAR noise standard deviation cannot be negative");
    }
}

[[nodiscard]] std::optional<double> ray_rectangle_distance(
    Vector2 origin,
    Vector2 direction,
    const AxisAlignedRectangle& rectangle
) {
    if (!rectangle.is_valid()) {
        throw std::invalid_argument("LiDAR obstacle rectangle is invalid");
    }

    double near_distance = 0.0;
    double far_distance = std::numeric_limits<double>::infinity();
    const double origins[] = {origin.x, origin.y};
    const double directions[] = {direction.x, direction.y};
    const double minimums[] = {rectangle.min_x, rectangle.min_y};
    const double maximums[] = {rectangle.max_x, rectangle.max_y};

    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(directions[axis]) < kRayParallelEpsilon) {
            if (origins[axis] < minimums[axis] || origins[axis] > maximums[axis]) {
                return std::nullopt;
            }
            continue;
        }

        double first = (minimums[axis] - origins[axis]) / directions[axis];
        double second = (maximums[axis] - origins[axis]) / directions[axis];
        if (first > second) {
            std::swap(first, second);
        }
        near_distance = std::max(near_distance, first);
        far_distance = std::min(far_distance, second);
        if (near_distance > far_distance) {
            return std::nullopt;
        }
    }

    if (far_distance < 0.0) {
        return std::nullopt;
    }
    return near_distance >= 0.0 ? near_distance : 0.0;
}

[[nodiscard]] AxisAlignedRectangle cell_rectangle(
    const OccupancyGrid& map,
    GridCell cell
) {
    const Vector2 centre = map.cell_center(cell);
    const double half_extent = map.resolution_m() / 2.0;
    return {
        centre.x - half_extent,
        centre.y - half_extent,
        centre.x + half_extent,
        centre.y + half_extent,
    };
}

}  // namespace

double LaserScan::angle_at(std::size_t index) const {
    if (index >= ranges_m.size()) {
        throw std::out_of_range("LiDAR scan index is outside the range array");
    }
    return angle_min_rad + static_cast<double>(index) * angle_increment_rad;
}

LidarSensor::LidarSensor(
    const OccupancyGrid& map,
    LidarParameters parameters
)
    : map_(map),
      parameters_(parameters),
      random_engine_(parameters.seed) {
    validate_parameters(parameters_);
}

LaserScan LidarSensor::measure(
    const Pose2D& pose,
    double stamp_s,
    const std::vector<AxisAlignedRectangle>& extra_obstacles
) {
    if (!pose.is_finite() || !std::isfinite(stamp_s)) {
        throw std::invalid_argument("LiDAR pose and timestamp must be finite");
    }

    const Transform2D world_from_base = Transform2D::from_pose(pose);
    const Transform2D base_from_lidar(
        parameters_.mount_translation,
        parameters_.mount_yaw_rad
    );
    const Transform2D world_from_lidar = world_from_base * base_from_lidar;
    const Vector2 origin = world_from_lidar.translation;

    LaserScan scan;
    scan.stamp_s = stamp_s;
    scan.angle_min_rad = parameters_.angle_min_rad;
    scan.angle_increment_rad = parameters_.ray_count == 1
        ? 0.0
        : (parameters_.angle_max_rad - parameters_.angle_min_rad) /
            static_cast<double>(parameters_.ray_count - 1);
    scan.range_min_m = parameters_.range_min_m;
    scan.range_max_m = parameters_.range_max_m;
    scan.ranges_m.reserve(parameters_.ray_count);
    scan.has_return.reserve(parameters_.ray_count);

    std::normal_distribution<double> noise(0.0, parameters_.noise_stddev_m);
    for (std::size_t index = 0; index < parameters_.ray_count; ++index) {
        const double sensor_angle = parameters_.ray_count == 1
            ? 0.5 * (parameters_.angle_min_rad + parameters_.angle_max_rad)
            : parameters_.angle_min_rad +
                static_cast<double>(index) * scan.angle_increment_rad;
        const double world_angle = world_from_lidar.yaw + sensor_angle;
        const Vector2 direction{std::cos(world_angle), std::sin(world_angle)};
        const double hit_distance = nearest_hit_distance(
            origin,
            direction,
            extra_obstacles
        );
        const bool hit = std::isfinite(hit_distance) &&
            hit_distance <= parameters_.range_max_m;
        if (!hit) {
            scan.ranges_m.push_back(parameters_.range_max_m);
            scan.has_return.push_back(0);
            continue;
        }

        double measured = hit_distance;
        if (parameters_.noise_stddev_m > 0.0) {
            measured += noise(random_engine_);
        }
        measured = std::clamp(
            measured,
            parameters_.range_min_m,
            parameters_.range_max_m
        );
        scan.ranges_m.push_back(measured);
        scan.has_return.push_back(1);
    }
    return scan;
}

void LidarSensor::reseed(std::uint32_t seed) {
    random_engine_.seed(seed);
}

double LidarSensor::nearest_hit_distance(
    Vector2 origin,
    Vector2 direction,
    const std::vector<AxisAlignedRectangle>& extra_obstacles
) const {
    double nearest = std::numeric_limits<double>::infinity();
    for (int row = 0; row < map_.height(); ++row) {
        for (int column = 0; column < map_.width(); ++column) {
            const GridCell cell{row, column};
            if (!map_.is_occupied(cell)) {
                continue;
            }
            const std::optional<double> distance = ray_rectangle_distance(
                origin,
                direction,
                cell_rectangle(map_, cell)
            );
            if (distance.has_value()) {
                nearest = std::min(nearest, *distance);
            }
        }
    }

    for (const AxisAlignedRectangle& obstacle : extra_obstacles) {
        const std::optional<double> distance = ray_rectangle_distance(
            origin,
            direction,
            obstacle
        );
        if (distance.has_value()) {
            nearest = std::min(nearest, *distance);
        }
    }
    return nearest;
}

}  // namespace robot_sim
