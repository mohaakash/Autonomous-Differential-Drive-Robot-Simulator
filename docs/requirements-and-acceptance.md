# Requirements and acceptance criteria

## Product goal

Build a self-contained 2D simulator that loads a map, plans a collision-free route, follows it with a differential-drive model, senses obstacles with simulated LiDAR, and reports measurable navigation results.

The first release is a software-only simulator. It does not depend on physical motors, sensors, ROS2, Gazebo, paid services, or a network connection.

## Required capabilities

### Map and scenario

1. Load a documented occupancy-grid map.
2. Represent free and occupied cells without ambiguity.
3. Accept start and goal poses in world coordinates.
4. Reject malformed maps, out-of-bounds poses, occupied start cells, and occupied goal cells.
5. Report whether the goal is reachable before starting motion.

### Robot and motion

1. Store a pose `(x, y, theta)` and configurable wheel radius and wheelbase.
2. Convert wheel angular velocities to body linear/angular velocity.
3. Convert body commands back to wheel commands.
4. Enforce wheel-speed, linear-speed, and angular-speed limits.
5. Integrate pose using a fixed simulation step.
6. Detect a collision before accepting unsafe motion as successful.

### Planning and control

1. Inflate occupied space by the configured robot footprint clearance.
2. Find an A* path on an 8-connected grid.
3. Return an explicit unreachable result when no path exists.
4. Convert the grid path into world-frame waypoints.
5. Follow the path with bounded pure-pursuit control.
6. Stop only after position and heading tolerances are both satisfied.

### Sensing and safety

1. Simulate configurable LiDAR angular span, ray count, maximum range, and mount transform.
2. Return range measurements in the sensor frame with a clear no-return value.
3. Support deterministic measurement noise through a seed.
4. Stop or replan if an obstacle invalidates the active route.
5. Record emergency stops, replans, and collisions separately.

### Reporting

1. Save the scenario configuration and random seed with each run.
2. Report completion status, final errors, path length, duration, collisions, replans, and control-step timing fields. The current runner leaves per-step wall-clock timing as `null`; the benchmark reports wall-clock performance separately.
3. Produce machine-readable CSV/JSON output and a human-readable SVG map overlay.
4. Make a run reproducible from a clean checkout using documented commands.

## Non-functional requirements

- Build with C++17 and CMake.
- Compile with at least `-Wall -Wextra -Wpedantic` on GCC or Clang.
- Keep core simulation and planning logic independent of the visualization backend.
- Use fixed-step simulation for tests; wall-clock rendering must not affect physics.
- Use explicit units in names or documentation where a value could be ambiguous.
- Do not report portfolio metrics until they come from saved experiment output.

## Out of scope for the first release

- Physical hardware drivers
- 3D dynamics
- SLAM or map generation from raw sensor data
- Dynamic-obstacle prediction
- Real-time guarantees
- Deep-learning perception
- ROS2, RViz2, or Gazebo integration

These can be added as extensions without changing the standalone simulator's core contracts.

## Acceptance scenarios

The baseline is acceptable when all of the following are true:

| Scenario | Expected result |
| --- | --- |
| Empty map, equal wheel speeds | Robot travels straight and remains collision-free |
| Empty map, opposite wheel speeds | Robot rotates about its centre without translating materially |
| Route around a rectangular obstacle | A* returns a path that contains no inflated occupied cell |
| Enclosed goal | Planner returns unreachable and the simulator does not move |
| Reachable goal | Robot stops within configured position and heading tolerances |
| Obstacle introduced on active route | Robot performs the configured stop or replan action before collision |
| Same seed and configuration twice | Results are identical within serialization precision |
| Malformed map/configuration | Program exits non-zero with a useful error |

## Definition of done

The first release is complete when a new developer can build it, run the tests, execute at least one map scenario, inspect a visualization or saved trace, and understand the result using only the repository README and these documents.
