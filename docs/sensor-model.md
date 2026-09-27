# Simulated LiDAR model

## Purpose

LiDAR is used to demonstrate frame transformations, ray casting, measurement limits, deterministic noise, and route-safety behavior. It is not intended to reproduce a particular physical scanner in the first release.

## Configuration

The baseline sensor has:

- `range_min` and `range_max` in metres;
- `angle_min` and `angle_max` in radians relative to the LiDAR frame;
- `ray_count` samples including both endpoints when `ray_count > 1`;
- a mount translation and yaw relative to `base_link`;
- optional Gaussian range noise with standard deviation `noise_stddev`;
- a deterministic random seed inherited from the scenario.

When `ray_count == 1`, use the midpoint angle. Invalid ranges or a non-positive ray count are configuration errors.

## Ray casting

For each ray angle `alpha`:

1. Transform the sensor origin into `world`.
2. Compute world ray heading `theta_robot + yaw_mount + alpha`.
3. Intersect the ray with map boundaries and occupied rectangles.
4. Select the nearest valid intersection.
5. Clamp the result to the sensor range limits.
6. Add noise if enabled, then clamp again to valid output bounds.

If no obstacle is hit within range, return `range_max` and set a `has_return` flag to false, or use the documented no-return convention consistently. Do not use a magic value without documenting it.

The hit distance is measured from the LiDAR origin, not from the robot centre.

## Noise and determinism

Use a seeded pseudo-random generator owned by the run or sensor instance. Do not use a global random source. Record the seed and noise parameters with the output.

With noise disabled, scans for the same state and map must be identical. With noise enabled, the same seed and step sequence must reproduce the same scan sequence.

## Safety interpretation

The safety layer should inspect the scan in a forward path corridor rather than treating any close side return as a route block. A minimum forward clearance can be defined as:

```text
required_clearance = robot_radius + stopping_margin
```

The initial implementation may use a simpler forward-sector threshold, but the sector and threshold must be configurable and included in the run report.

## Sensor tests

- A ray aimed at a known wall returns the analytically expected distance.
- Rotating the robot rotates all world hit points while preserving sensor-frame ranges.
- Translating the LiDAR mount changes the origin of the measurement as expected.
- A maximum-range no-hit is distinguished from a finite obstacle return.
- Noise-disabled runs are bit-for-bit or tolerance-equivalent deterministic.
- A close obstacle in the forward sector triggers the configured safety action.

## Current implementation

The standalone C++ implementation is `LidarSensor` in `include/robot_sim/sensors/lidar.hpp`. Occupied cells are ray-tested as axis-aligned rectangles; scenario code may add axis-aligned dynamic rectangles for an individual measurement. A no-return is represented by `range_max_m` with `has_return == false`, while a clipped finite hit has `has_return == true`. The Phase 5 tests cover wall distances, mount translation, dynamic obstacles, seeded noise, forward-sector filtering, and simulator safety outcomes.
