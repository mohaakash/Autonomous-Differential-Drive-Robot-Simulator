# Coordinate frames and transformations

## Frames

The baseline simulator uses three frames:

- `world`: fixed map frame. Obstacles and planned waypoints are expressed here.
- `base_link`: robot body frame. Its origin is the robot reference point, normally the geometric centre; its positive `x` axis points forward and positive `y` points left.
- `lidar`: sensor frame. Its origin and yaw relative to `base_link` are configurable.

Frames are right-handed in 2D. Positive yaw is counter-clockwise.

## Transform notation

`T_world_base` maps a point from `base_link` into `world`. For pose `(x, y, theta)`:

```text
T_world_base = [ cos(theta)  -sin(theta)  x ]
                [ sin(theta)   cos(theta)  y ]
                [     0             0      1 ]
```

For a point `(px, py)` in `base_link`:

```text
p_world.x = x + cos(theta) * px - sin(theta) * py
p_world.y = y + sin(theta) * px + cos(theta) * py
```

The inverse transform is obtained by applying the inverse rotation and then translating by `(-x, -y)` in the world frame.

## LiDAR extrinsic transform

Let the LiDAR mount be `(tx, ty, yaw)` in `base_link`. A point in the LiDAR frame is first transformed by the mount transform and then by the robot pose:

```text
p_world = T_world_base * T_base_lidar * p_lidar
```

For ray casting, the sensor origin in world coordinates is the transformed origin. A ray direction with sensor-frame angle `alpha` has world heading:

```text
theta_ray = theta_robot + yaw_mount + alpha
```

The range is a scalar; its hit point is reconstructed in `lidar` and transformed only when world coordinates are needed for visualization or collision reasoning.

## Grid/world conversion

The map document defines the authoritative conversion. With map origin `(origin_x, origin_y)`, resolution `resolution`, width `W`, height `H`, and file row `row` counted from the top:

```text
column = floor((x - origin_x) / resolution)
row    = H - 1 - floor((y - origin_y) / resolution) - 1
```

Implement the conversion in one shared utility and test it at cell centres and boundaries. Do not reimplement it independently in the planner, visualizer, and collision checker.

## Angle handling

Use one wrapped-angle helper:

```text
wrap_to_pi(angle) -> [-pi, pi)
```

Heading error is always computed as:

```text
heading_error = wrap_to_pi(target_heading - current_heading)
```

Comparing raw angles can incorrectly report a nearly full revolution at the `-pi/pi` boundary.

## Common frame mistakes to test

- Applying the robot translation before rotating a body-frame point.
- Using the LiDAR mount yaw with the wrong sign.
- Treating a range value as a world-coordinate distance from the robot centre when the sensor is offset.
- Mixing degrees and radians.
- Using a bottom-origin map formula for a top-origin file.
