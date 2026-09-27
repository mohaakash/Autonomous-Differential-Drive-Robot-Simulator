# System architecture

## Design goals

The simulator should keep deterministic robotics logic separate from presentation and file-format code. Each module should have one clear owner for its state and a small, testable interface.

## Components

```text
                 +----------------+
                 | Scenario/Input |
                 +--------+-------+
                          |
                          v
 +-----------+    +------+-------+    +----------------+
 | Map Loader|--->| Map/Collision|<---| Robot Geometry |
 +-----------+    +------+-------+    +----------------+
                          |
                          v
                    +-----+------+
                    | A* Planner |
                    +-----+------+
                          |
                       Waypoints
                          |
                          v
 +-------------+    +----+-----+    +----------------+
 | LiDAR Model |--->| Controller|--->| Kinematics     |
 +------+------+    +----+-----+    +-------+--------+
        |                 |                  |
        |                 v                  v
        |          +------+-----+      +-----+------+
        +--------->| Safety     |<-----| Pose State |
                   +------+-----+      +-----+------+
                          |                  |
                          +--------+---------+
                                   v
                         +---------+----------+
                         | Metrics/Visualizer|
                         +--------------------+
```

## Module responsibilities

### Input and configuration

The current command-line runner accepts a marker-based map, output directory, and random seed. The core modules receive typed parameter structures; downstream modules do not parse command-line strings or assume a YAML parser exists. Future configuration-file support must preserve this boundary.

### Map and collision

Owns the occupancy grid, world/grid conversion, obstacle inflation, footprint checks, and boundary checks. It is the authority on whether a pose or swept motion is valid.

### Robot model and kinematics

Owns physical dimensions, wheel state, pose integration, velocity conversion, angle normalization, and command saturation. It must not know how a path was planned.

### Planner

Consumes a collision/planning view of the map and start/goal cells. Returns either a validated grid path or an explicit failure reason. It does not move the robot.

### Controller

Consumes the current pose, reference path, and control limits. Produces a body-frame twist command and a controller status such as `TRACKING`, `GOAL_REACHED`, or `STOPPED`.

### Sensor model

Consumes the true world state and sensor configuration. Produces a LiDAR scan with ranges, angles, timestamps, and a random-seed/run identifier. It does not update robot pose.

### Safety and simulation orchestration

Owns the order of operations, collision response, replanning policy, stop conditions, and run status. It coordinates modules but should not duplicate their mathematics.

### Metrics and visualization

Receives snapshots and events. It may draw or serialize them, but must not change simulation state.

## Simulation-step contract

For each fixed step `dt`:

1. Read the current true pose and active route.
2. Generate the LiDAR scan from the current world state.
3. Evaluate safety conditions and route validity.
4. If safe, ask the controller for a body twist; otherwise issue a zero command or trigger replanning.
5. Convert the twist to wheel commands and apply configured saturation.
6. Integrate the candidate pose.
7. Check the candidate pose and motion segment for collision.
8. Commit the pose only if the collision policy allows it; otherwise stop and record the event.
9. Update travelled distance, errors, timing, and trace samples.
10. Check goal, timeout, collision, and failure termination conditions.

Planning is performed before the first step and again only when the replan policy requires it. Visualization and file output must observe state after the step and must not control the step rate.

## Core data contracts

The exact C++ types may vary, but the following concepts should exist:

```text
Pose2D        { x, y, theta }
Twist2D       { linear_x, angular_z }
WheelCommand  { left_rad_s, right_rad_s, clipped }
GridCell      { row, column }
LaserScan     { stamp, angle_min, angle_increment, ranges, max_range }
Path         { grid_cells, world_waypoints, length }
PlannedPath   { waypoints, length }
RunMetrics    { status, errors, duration, collisions, replans, timing }
```

All public operations should document units, frame, ownership, and failure behavior.

## Failure boundaries

- Input failures stop before simulation starts.
- No-path failures produce a completed diagnostic run with status `UNREACHABLE`, not a fake path.
- Controller failures issue a stop command and status `CONTROL_FAILURE`.
- Collision failures stop the run and preserve the last valid pose plus the rejected candidate.
- Output failures are reported distinctly from navigation failures.
